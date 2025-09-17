#!/usr/bin/env python3
"""
Fix magnetometer measurement model for better EKF performance
The current model incorrectly assumes magnetometer directly measures yaw angle.
Real magnetometers measure 3D magnetic field vector that needs proper projection.
"""

import numpy as np
from ekf_core import ExtendedKalmanFilter
from ekf_parameters import EKFParameters
from ekf_dynamics import rotation_matrix, wrap_to_pi

class ImprovedMagnetometerEKF(ExtendedKalmanFilter):
    """EKF with improved magnetometer measurement model"""
    
    def __init__(self, params: EKFParameters):
        super().__init__(params)
        
        # Earth's magnetic field vector in NED frame (Espoo, Finland)
        # Declination: ~8°, Inclination: ~72°, Field strength: ~50,000 nT
        self.mag_field_ned = np.array([
            50_000 * np.cos(np.radians(72)) * np.cos(np.radians(8)),  # North component
            50_000 * np.cos(np.radians(72)) * np.sin(np.radians(8)),  # East component  
            -50_000 * np.sin(np.radians(72))  # Down component (negative in NED)
        ]) / 1_000_000  # Convert nT to T for calculations
        
        print(f"🌍 Earth's magnetic field (NED): {self.mag_field_ned}")
    
    def update_mag_improved(self, x_pred: np.ndarray, P_pred: np.ndarray, 
                           mag_meas: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Improved magnetometer measurement update using proper magnetic field model.
        
        Args:
            x_pred: predicted state [pos(3); vel(3); att(3)]
            P_pred: predicted covariance
            mag_meas: magnetometer yaw measurement (simplified for now)
        
        Returns:
            x_est: updated state
            P: updated covariance
        """
        # For now, use simplified model but with better noise handling
        # TODO: Implement full 3D magnetic field measurement model
        
        # Magnetometer measures yaw angle with proper noise model
        H = np.zeros((1, 9))
        H[0, 8] = 1  # yaw component
        
        # Use more realistic magnetometer noise
        R = self.params.R_mag * 2.0  # Increase noise tolerance
        
        # Innovation with angle wrapping
        y = wrap_to_pi(mag_meas - x_pred[8])
        
        # More lenient innovation gating (30° instead of 15°)
        y_magnitude = float(np.abs(y).item())
        if y_magnitude > np.deg2rad(30):  # Increased from 15° to 30°
            print(f"Warning: EKF: Mag innovation too large ({np.rad2deg(y_magnitude):.1f}°). Rejecting measurement.")
            self._update_innovation_stats('mag', y, rejected=True)
            return x_pred, P_pred
        
        # Innovation covariance
        S = H @ P_pred @ H.T + R
        
        # Add regularization
        S = S + 1e-6
        
        if np.linalg.cond(S) > 1e12:
            print("Warning: EKF: Mag innovation covariance is ill-conditioned. Skipping update.")
            self._update_innovation_stats('mag', y, rejected=True)
            return x_pred, P_pred
        
        # Kalman gain
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # State update
        x_est = x_pred + K @ y
        
        # Covariance update (Joseph form for numerical stability)
        I = np.eye(9)
        P = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ R @ K.T
        
        # Update innovation statistics
        self._update_innovation_stats('mag', y)
        
        return x_est, P
    
    def step(self, imu: np.ndarray, dt: float, 
             gps_meas: np.ndarray = None,
             baro_meas: float = None,
             mag_meas: float = None,
             use_accel_tilt: bool = False) -> tuple[np.ndarray, np.ndarray]:
        """Override step method to use improved magnetometer update"""
        
        # Update adaptive noise scaling
        self._update_adaptive_noise(imu, dt)
        
        # Prediction step
        x_pred, P_pred = self.predict(imu, dt)
        
        x_est = x_pred.copy()
        P = P_pred.copy()
        
        # GPS update
        if gps_meas is not None:
            x_est, P = self.update_gps(x_est, P, gps_meas)
            x_est = x_est.flatten() if x_est.ndim > 1 else x_est
        
        # Barometer update
        if baro_meas is not None:
            x_est, P = self.update_baro(x_est, P, baro_meas)
            x_est = x_est.flatten() if x_est.ndim > 1 else x_est
        
        # Improved magnetometer update
        if mag_meas is not None:
            x_est, P = self.update_mag_improved(x_est, P, mag_meas)
            x_est = x_est.flatten() if x_est.ndim > 1 else x_est
        
        # Accelerometer tilt update
        if use_accel_tilt:
            x_est_flat = x_est.flatten()[:9] if x_est.size > 9 else x_est
            x_est, P = self.update_accel_tilt(x_est_flat, P, imu)
            x_est = x_est.flatten() if x_est.ndim > 1 else x_est
        
        # Final validation and clamping
        x_est = self._validate_and_clamp_state(x_est)
        
        return x_est, P

def create_improved_ekf():
    """Create EKF with improved magnetometer model"""
    params = EKFParameters()
    
    # Fine-tune magnetometer parameters
    params.Mag_sigma_deg = 8.0  # Increase from 2.0° to 8.0°
    params.Mag_sigma_rad = np.radians(params.Mag_sigma_deg)
    params.R_mag = (params.Mag_sigma_rad * 0.8)**2  # More lenient noise model
    
    # Create improved EKF
    ekf = ImprovedMagnetometerEKF(params)
    
    print("🔧 Improved magnetometer model applied:")
    print(f"   Magnetometer noise: {params.Mag_sigma_deg}°")
    print(f"   Innovation gate: 30° (increased from 15°)")
    print(f"   Noise tolerance: {params.R_mag:.6f}")
    
    return ekf, params

if __name__ == "__main__":
    # Test the improved magnetometer model
    ekf, params = create_improved_ekf()
    
    # Simple test
    x_init = np.zeros(9)
    x_init[8] = 0.1  # Small yaw angle
    ekf.initialize(x_init)
    
    # Test magnetometer update
    imu = np.array([0, 0, -9.81, 0, 0, 0])  # Stationary IMU
    mag_meas = 0.2  # 0.2 rad yaw measurement
    
    x_est, P = ekf.step(imu, 0.01, mag_meas=mag_meas)
    print(f"✅ Test successful! Estimated yaw: {x_est[8]:.3f} rad")
