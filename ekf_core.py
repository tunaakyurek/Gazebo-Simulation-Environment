#!/usr/bin/env python3

import numpy as np
from typing import Tuple, Optional, Dict, Any
from ekf_parameters import EKFParameters
from ekf_dynamics import drone_dynamics_imu, calculate_F_sensor_only, wrap_to_pi

class ExtendedKalmanFilter:
    """Extended Kalman Filter for drone state estimation"""
    
    def __init__(self, params: EKFParameters):
        self.params = params
        
        # State: [pos(3); vel(3); att(3)] in NED
        self.x_est: np.ndarray = np.zeros(9)
        self.P: np.ndarray = np.eye(9) * 0.1
        
        # State bounds for validation
        self.pos_lim = 1000.0    # Position limit (m)
        self.vel_lim = 50.0      # Velocity limit (m/s)
        self.roll_pitch_lim = np.pi/2  # roll/pitch limit (rad)
        
        # Innovation statistics for monitoring
        self.innovation_stats = {
            'gps': {'count': 0, 'sum_sq': 0.0, 'rejected': 0},
            'baro': {'count': 0, 'sum_sq': 0.0, 'rejected': 0},
            'mag': {'count': 0, 'sum_sq': 0.0, 'rejected': 0}
        }
        
        # Adaptive noise scaling
        self.current_noise_scale = 1.0
        self.gps_outage_duration = 0.0
        self.last_gps_time = 0.0
    
    def initialize(self, x0: np.ndarray, P0: Optional[np.ndarray] = None):
        """
        Initialize EKF with initial state and covariance.
        
        Args:
            x0: Initial state estimate [pos(3); vel(3); att(3)]
            P0: Initial covariance matrix (9x9), if None uses default
        """
        self.x_est = x0.copy()
        if P0 is not None:
            self.P = P0.copy()
        else:
            # Default initial covariance
            self.P = np.diag([
                0.5**2, 0.5**2, 0.4**2,  # Position
                0.2**2, 0.2**2, 0.2**2,  # Velocity
                np.deg2rad(2)**2, np.deg2rad(2)**2, np.deg2rad(3)**2  # Attitude
            ])
        
        # Reset statistics
        for key in self.innovation_stats:
            self.innovation_stats[key] = {'count': 0, 'sum_sq': 0.0, 'rejected': 0}
    
    def predict(self, imu: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF prediction step using IMU mechanization.
        
        Args:
            imu: IMU measurements [accel(3); gyro(3)] in body frame
            dt: time step (s)
        
        Returns:
            x_pred: predicted state
            P_pred: predicted covariance
        """
        # Input validation and clamping
        if not self._validate_state(self.x_est) or not self._validate_covariance(self.P):
            print("Warning: EKF: Invalid input state or covariance. Using safe defaults.")
            self.x_est = np.zeros(9)
            self.P = 0.1 * np.eye(9)
        
        # Clamp state to reasonable bounds
        self._clamp_state(self.x_est)
        
        # Nonlinear prediction using IMU mechanization
        try:
            x_dot = drone_dynamics_imu(0, self.x_est, imu, self.params)
            x_pred = self.x_est + x_dot * dt
        except Exception as e:
            print(f"Warning: EKF: Error in IMU mechanization: {e}. Using simple integration.")
            x_pred = self.x_est + np.concatenate([self.x_est[3:6], np.zeros(6)]) * dt
        
        # Clamp predicted roll/pitch; wrap yaw
        x_pred[6:8] = np.clip(x_pred[6:8], -np.deg2rad(60), np.deg2rad(60))
        x_pred[8] = wrap_to_pi(x_pred[8])
        
        # Validate prediction
        if not self._validate_state(x_pred):
            print("Warning: EKF: Invalid prediction. Using previous state.")
            x_pred = self.x_est.copy()
        
        # Calculate Jacobian
        F = calculate_F_sensor_only(x_pred, imu, dt)
        
        # Process noise with adaptive scaling
        Q = self.params.Q * dt * self.current_noise_scale
        
        # Ensure Q is positive definite
        Q = self._ensure_positive_definite(Q)
        
        # Covariance prediction
        P_pred = F @ self.P @ F.T + Q
        
        # Condition P_pred
        P_pred = self._ensure_positive_definite(P_pred)
        
        return x_pred, P_pred
    
    def update_gps(self, x_pred: np.ndarray, P_pred: np.ndarray, 
                   gps_meas: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        GPS measurement update.
        
        Args:
            x_pred: predicted state
            P_pred: predicted covariance
            gps_meas: GPS position measurement [x, y, z]
        
        Returns:
            x_est: updated state
            P: updated covariance
        """
        # GPS measures position
        H = np.zeros((3, 9))
        H[0:3, 0:3] = np.eye(3)
        
        R = self.params.R_gps
        
        # Innovation
        y = gps_meas - H @ x_pred
        
        # Innovation covariance
        S = H @ P_pred @ H.T + R
        
        # Add regularization
        S = S + 1e-6 * np.eye(3)
        
        # Check condition number
        if np.linalg.cond(S) > 1e12:
            print("Warning: EKF: GPS innovation covariance is ill-conditioned. Skipping update.")
            self._update_innovation_stats('gps', y, rejected=True)
            return x_pred, P_pred
        
        # Kalman gain
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # State update
        x_est = x_pred + K @ y
        
        # Covariance update (Joseph form for numerical stability)
        I = np.eye(9)
        P = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ R @ K.T
        
        # Update innovation statistics
        self._update_innovation_stats('gps', y)
        
        # Update GPS outage tracking
        self.last_gps_time = 0.0  # Reset GPS outage timer
        self.gps_outage_duration = 0.0
        
        return x_est, P
    
    def update_baro(self, x_pred: np.ndarray, P_pred: np.ndarray, 
                    baro_meas: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Barometer measurement update.
        
        Args:
            x_pred: predicted state
            P_pred: predicted covariance
            baro_meas: barometer altitude measurement
        
        Returns:
            x_est: updated state
            P: updated covariance
        """
        # Barometer measures altitude = -z (NED -> altitude)
        H = np.zeros((1, 9))
        H[0, 2] = -1  # -z component
        
        R = self.params.R_baro
        
        # Innovation
        y = baro_meas - H @ x_pred
        
        # Innovation covariance
        S = H @ P_pred @ H.T + R
        
        # Add regularization
        S = S + 1e-6
        
        if np.linalg.cond(S) > 1e12:
            print("Warning: EKF: Baro innovation covariance is ill-conditioned. Skipping update.")
            self._update_innovation_stats('baro', y, rejected=True)
            return x_pred, P_pred
        
        # Kalman gain
        K = P_pred @ H.T / S
        
        # State update
        x_est = x_pred + K * y
        
        # Covariance update
        I = np.eye(9)
        P = (I - K @ H) @ P_pred @ (I - K @ H).T + K * R * K.T
        
        # Update innovation statistics
        self._update_innovation_stats('baro', y)
        
        return x_est, P
    
    def update_mag(self, x_pred: np.ndarray, P_pred: np.ndarray, 
                   mag_meas: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Magnetometer measurement update.
        
        Args:
            x_pred: predicted state
            P_pred: predicted covariance
            mag_meas: magnetometer yaw measurement
        
        Returns:
            x_est: updated state
            P: updated covariance
        """
        # Magnetometer measures yaw angle psi
        H = np.zeros((1, 9))
        H[0, 8] = 1  # yaw component
        
        R = self.params.R_mag
        
        # Innovation with angle wrapping
        y = wrap_to_pi(mag_meas - x_pred[8])
        
        # Innovation covariance
        S = H @ P_pred @ H.T + R
        
        # Add regularization
        S = S + 1e-6
        
        if np.linalg.cond(S) > 1e12:
            print("Warning: EKF: Mag innovation covariance is ill-conditioned. Skipping update.")
            self._update_innovation_stats('mag', y, rejected=True)
            return x_pred, P_pred
        
        # Kalman gain
        K = P_pred @ H.T / S
        
        # State update
        x_est = x_pred + K * y
        
        # Covariance update
        I = np.eye(9)
        P = (I - K @ H) @ P_pred @ (I - K @ H).T + K * R * K.T
        
        # Update innovation statistics
        self._update_innovation_stats('mag', y)
        
        return x_est, P
    
    def update_accel_tilt(self, x_pred: np.ndarray, P_pred: np.ndarray, 
                         imu: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Accelerometer tilt update (roll/pitch from gravity vector).
        
        Args:
            x_pred: predicted state
            P_pred: predicted covariance
            imu: IMU measurements [accel(3); gyro(3)]
        
        Returns:
            x_est: updated state
            P: updated covariance
        """
        # Use accelerometer to correct roll/pitch assuming quasi-static
        g = self.params.g
        g_norm = np.linalg.norm(g)
        if g_norm < 1e-6:
            g_norm = 9.81
        
        psi = x_pred[8]
        if hasattr(psi, '__len__') and len(psi) > 1:
            psi = psi[0]  # Take first element if array
        psi = float(psi)
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        Rz = np.array([
            [cos_psi, -sin_psi, 0],
            [sin_psi, cos_psi, 0],
            [0, 0, 1]
        ])
        
        a_b = imu[0:3]
        a_n_partial = Rz @ a_b  # approximate body->NED using yaw only
        
        # Compensate gravity sign: specific force f_b = R'*(a - g)
        g_n_est = -a_n_partial  # sign so that at rest a_b ~ -R'*g
        
        # Convert gravity direction to roll/pitch
        gx, gy, gz = g_n_est[0], g_n_est[1], g_n_est[2]
        
        # Normalize
        s = np.sqrt(gx**2 + gy**2 + gz**2)
        if s > 1e-6:
            gx, gy, gz = gx/s, gy/s, gz/s
            
            # Roll and pitch from gravity vector
            phi_meas = np.arctan2(gy, gz)
            theta_meas = -np.arcsin(np.clip(gx, -1, 1))
            
            z_tilt = np.array([phi_meas, theta_meas])
            H = np.zeros((2, 9))
            H[0:2, 6:8] = np.eye(2)  # measure [phi; theta]
            
            # Use tuned tilt noise
            rdeg = getattr(self.params, 'Tilt_R_deg', 3.0)
            Rtilt = np.diag([np.deg2rad(rdeg)**2, np.deg2rad(rdeg)**2])
            
            # Ensure x_pred is a 1D array of length 9
            if x_pred.size == 81:  # 9x9 matrix
                x_pred_flat = np.diag(x_pred)  # Take diagonal elements
            else:
                x_pred_flat = x_pred.flatten() if x_pred.ndim > 1 else x_pred
            
            # Ensure we have exactly 9 elements
            if len(x_pred_flat) != 9:
                x_pred_flat = x_pred_flat[:9]  # Take first 9 elements
            
            y = z_tilt - (H @ x_pred_flat)
            
            # Innovation covariance
            S = H @ P_pred @ H.T + Rtilt + 1e-9 * np.eye(2)
            
            # Kalman gain
            K = P_pred @ H.T @ np.linalg.inv(S)
            
            # State update
            x_est = x_pred + (K @ y).flatten()
            
            # Covariance update
            I = np.eye(9)
            P = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ Rtilt @ K.T
            
            return x_est, P
        else:
            return x_pred, P_pred
    
    def step(self, imu: np.ndarray, dt: float, 
             gps_meas: Optional[np.ndarray] = None,
             baro_meas: Optional[float] = None,
             mag_meas: Optional[float] = None,
             use_accel_tilt: bool = False) -> Tuple[np.ndarray, np.ndarray]:
        """
        Complete EKF step: predict and update with available measurements.
        
        Args:
            imu: IMU measurements [accel(3); gyro(3)]
            dt: time step
            gps_meas: GPS position measurement (optional)
            baro_meas: Barometer altitude measurement (optional)
            mag_meas: Magnetometer yaw measurement (optional)
            use_accel_tilt: Whether to use accelerometer for tilt correction
        
        Returns:
            x_est: updated state estimate
            P: updated covariance
        """
        # Update adaptive noise scaling
        self._update_adaptive_noise(imu, dt)
        
        # Prediction step
        x_pred, P_pred = self.predict(imu, dt)
        
        # Update steps - ensure x_pred is a 9-element vector
        x_est = x_pred.flatten()[:9] if x_pred.size > 9 else x_pred.copy()
        P = P_pred.copy()
        
        # GPS update
        if gps_meas is not None:
            x_est, P = self.update_gps(x_est, P, gps_meas)
            x_est = x_est.flatten() if x_est.ndim > 1 else x_est
        
        # Barometer update
        if baro_meas is not None:
            x_est, P = self.update_baro(x_est, P, baro_meas)
            x_est = x_est.flatten() if x_est.ndim > 1 else x_est
        
        # Magnetometer update
        if mag_meas is not None:
            x_est, P = self.update_mag(x_est, P, mag_meas)
            x_est = x_est.flatten() if x_est.ndim > 1 else x_est
        
        # Accelerometer tilt update
        if use_accel_tilt:
            # Ensure x_est is a 9-element vector before calling update_accel_tilt
            x_est_flat = x_est.flatten()[:9] if x_est.size > 9 else x_est
            x_est, P = self.update_accel_tilt(x_est_flat, P, imu)
            x_est = x_est.flatten() if x_est.ndim > 1 else x_est
        
        # Final validation and clamping
        if not self._validate_state(x_est) or not self._validate_covariance(P):
            print("Warning: EKF: Final state or covariance contains NaN/Inf. Resetting to prediction.")
            x_est = x_pred.copy()
            P = P_pred.copy()
        
        self._clamp_state(x_est)
        P = self._ensure_positive_definite(P)
        
        # Update internal state
        self.x_est = x_est
        self.P = P
        
        return x_est, P
    
    def _validate_state(self, x: np.ndarray) -> bool:
        """Validate state vector"""
        return np.all(np.isfinite(x)) and len(x) == 9
    
    def _validate_covariance(self, P: np.ndarray) -> bool:
        """Validate covariance matrix"""
        return (np.all(np.isfinite(P)) and 
                P.shape == (9, 9) and 
                np.allclose(P, P.T) and
                np.all(np.linalg.eigvals(P) > 0))
    
    def _clamp_state(self, x: np.ndarray):
        """Clamp state to reasonable bounds"""
        x[0:3] = np.clip(x[0:3], -self.pos_lim, self.pos_lim)  # Position
        x[3:6] = np.clip(x[3:6], -self.vel_lim, self.vel_lim)  # Velocity
        x[6:8] = np.clip(x[6:8], -self.roll_pitch_lim, self.roll_pitch_lim)  # Roll/pitch
        x[8] = wrap_to_pi(x[8])  # Wrap yaw
    
    def _ensure_positive_definite(self, M: np.ndarray) -> np.ndarray:
        """Ensure matrix is positive definite"""
        U, S, Vt = np.linalg.svd(M)
        S = np.maximum(S, 1e-12)  # Prevent singular values from becoming too small
        S = np.minimum(S, 1e6)    # Prevent singular values from becoming too large
        return U @ np.diag(S) @ Vt
    
    def _update_innovation_stats(self, sensor_type: str, innovation: np.ndarray, rejected: bool = False):
        """Update innovation statistics"""
        if sensor_type in self.innovation_stats:
            stats = self.innovation_stats[sensor_type]
            if not rejected:
                stats['count'] += 1
                stats['sum_sq'] += np.sum(innovation**2)
            else:
                stats['rejected'] += 1
    
    def _update_adaptive_noise(self, imu: np.ndarray, dt: float):
        """Update adaptive noise scaling based on motion"""
        # Calculate angular rate magnitude
        angular_rate = np.linalg.norm(imu[3:6])
        
        # Update GPS outage duration
        self.gps_outage_duration += dt
        
        # Get adaptive noise scale
        if self.params.adaptive_noise:
            if angular_rate > self.params.turn_threshold:
                self.current_noise_scale = self.params.noise_scale_turn
            else:
                self.current_noise_scale = self.params.noise_scale_normal
            
            # Additional scaling during GPS outage
            if self.gps_outage_duration > 1.0:  # 1 second GPS outage
                gps_scale = min(1.0 + self.gps_outage_duration * 0.1, 
                               self.params.gps_outage_max_scale)
                self.current_noise_scale *= gps_scale
        else:
            self.current_noise_scale = 1.0
    
    def get_state(self) -> np.ndarray:
        """Get current state estimate"""
        return self.x_est.copy()
    
    def get_covariance(self) -> np.ndarray:
        """Get current covariance matrix"""
        return self.P.copy()
    
    def get_innovation_stats(self) -> Dict[str, Dict[str, Any]]:
        """Get innovation statistics"""
        stats = {}
        for sensor_type, data in self.innovation_stats.items():
            stats[sensor_type] = {
                'count': data['count'],
                'rejected': data['rejected'],
                'rejection_rate': data['rejected'] / max(1, data['count'] + data['rejected']),
                'rms_innovation': np.sqrt(data['sum_sq'] / max(1, data['count']))
            }
        return stats

if __name__ == '__main__':
    # Test EKF
    from ekf_parameters import EKFParameters
    
    params = EKFParameters()
    ekf = ExtendedKalmanFilter(params)
    
    # Initialize with some state
    x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
    ekf.initialize(x0)
    
    # Test prediction
    imu = np.array([0, 0, -9.81, 0, 0, 0])  # At rest
    dt = 0.01
    
    x_pred, P_pred = ekf.predict(imu, dt)
    print(f"Predicted state: {x_pred}")
    print(f"Predicted covariance shape: {P_pred.shape}")
    
    # Test GPS update
    gps_meas = np.array([1.0, 2.0, 3.0])
    x_est, P = ekf.update_gps(x_pred, P_pred, gps_meas)
    print(f"Updated state after GPS: {x_est}")
    
    # Test complete step
    x_est, P = ekf.step(imu, dt, gps_meas=gps_meas, baro_meas=3.0, mag_meas=0.1)
    print(f"Final state: {x_est}")
    
    # Get statistics
    stats = ekf.get_innovation_stats()
    print(f"Innovation stats: {stats}")
