#!/usr/bin/env python3

import numpy as np
from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class EKFParameters:
    """EKF Parameters Configuration - Python version of parameters.m"""
    
    # General Simulation & Timing Parameters
    solver_type: str = 'Fixed-step'
    Ts_physics: float = 0.002    # Physics sim rate (500 Hz)
    Ts_IMU: float = 0.004        # IMU sample time (250 Hz)
    Ts_GPS: float = 0.1          # GPS sample time (10 Hz)
    Ts_Baro: float = 0.02        # Barometer sample time (50 Hz)
    Ts_Mag: float = 0.02         # Magnetometer sample time (50 Hz)
    sim_duration: float = 60.0   # seconds
    
    # Location-Specific Parameters (Espoo, Finland)
    g: np.ndarray = None         # Gravity vector (m/s^2), NED positive down
    mag_NED: np.ndarray = None   # Magnetic field (uT)
    
    # Drone-Specific Physical Parameters
    profile: str = 'QAV250'      # 'QAV250' or 'S500'
    arm_length: float = 0.125    # meters
    prop_radius: float = 0.0635  # meters
    CT: float = 1.2e-6          # Thrust coefficient
    CQ: float = 2.1e-8          # Torque coefficient
    Q_vel: float = 0.4          # EKF process noise (velocity)
    Q_att: float = 0.02         # EKF process noise (attitude)
    mass: float = 0.5           # kg
    I: np.ndarray = None        # Inertia matrix (kg*m^2)
    
    # Sensor Noise & Error Parameters (Pixhawk 6C & M10 GPS)
    IMU_accel_noise_density: float = 6.9e-4    # (m/s^2)/sqrt(Hz)
    IMU_accel_bias_instab: float = 0.008       # m/s^2
    IMU_gyro_noise_density: float = 4.9e-5     # (rad/s)/sqrt(Hz)
    IMU_gyro_bias_instab: float = 8.7e-5       # rad/s
    
    # GPS noise parameters
    GPS_sigma_xy: float = 0.8   # meters (horizontal)
    GPS_sigma_z: float = 1.6    # meters (vertical)
    
    # Barometer noise
    Baro_sigma_z: float = 0.35  # meters
    
    # Magnetometer noise
    Mag_sigma_deg: float = 2.0  # degrees
    Mag_sigma_rad: float = None # radians (computed)
    
    # EKF Tuning Parameters (Q & R Matrices)
    Q: np.ndarray = None        # Process noise matrix
    R_gps: np.ndarray = None    # GPS measurement noise
    R_baro: float = None        # Barometer measurement noise
    R_mag: float = None         # Magnetometer measurement noise
    
    # Adaptive noise scaling parameters
    adaptive_noise: bool = True
    turn_threshold: float = 0.2  # rad/s
    noise_scale_turn: float = 1.1
    noise_scale_normal: float = 0.5
    gps_outage_max_scale: float = 1.5
    
    # Innovation gate thresholds
    innovation_gate_gps: float = 8.0      # meters
    innovation_gate_baro: float = 4.0     # meters
    innovation_gate_mag: float = np.deg2rad(30)  # radians
    innovation_gate_imu: float = 15.0     # m/s²
    max_angular_rate: float = np.deg2rad(120)  # rad/s
    
    # Cross-Coupling Parameters
    drag_coeff: float = 0.1
    gyro_momentum: float = 0.1
    coriolis_enabled: bool = True
    
    def __post_init__(self):
        """Initialize computed parameters after dataclass creation"""
        
        # Initialize gravity vector (NED frame, positive down)
        if self.g is None:
            self.g = np.array([0.0, 0.0, 9.819])
        
        # Initialize magnetic field (Espoo, Finland)
        if self.mag_NED is None:
            self.mag_NED = np.array([15.16, 2.65, 49.78])
        
        # Initialize magnetometer noise in radians
        if self.Mag_sigma_rad is None:
            self.Mag_sigma_rad = np.deg2rad(self.Mag_sigma_deg)
        
        # Set drone-specific parameters based on profile
        self._set_drone_parameters()
        
        # Initialize EKF matrices
        self._initialize_ekf_matrices()
    
    def _set_drone_parameters(self):
        """Set drone-specific parameters based on profile"""
        if self.profile == 'QAV250':
            self.arm_length = 0.125
            self.prop_radius = 0.0635
            self.CT = 1.2e-6
            self.CQ = 2.1e-8
            self.Q_vel = 0.4
            self.Q_att = 0.02
            self.mass = 0.5
            self.I = np.diag([0.0023, 0.0023, 0.004])
        elif self.profile == 'S500':
            self.arm_length = 0.25
            self.prop_radius = 0.127
            self.CT = 9.5e-6
            self.CQ = 1.8e-7
            self.Q_vel = 0.4
            self.Q_att = 0.01
            self.mass = 1.2
            self.I = np.diag([0.011, 0.011, 0.021])
        else:
            raise ValueError(f"Unknown drone profile: {self.profile}")
    
    def _initialize_ekf_matrices(self):
        """Initialize EKF Q and R matrices"""
        
        # Process noise matrix Q (9x9 for [pos(3); vel(3); att(3)])
        # Optimized for enhanced performance
        self.Q = np.diag([
            0.02, 0.02, 0.02,  # Position process noise
            0.10, 0.10, 0.12,  # Velocity process noise
            0.05, 0.05, 0.06   # Attitude process noise
        ])
        
        # Measurement noise matrices
        self.R_gps = np.diag([
            self.GPS_sigma_xy**2,
            self.GPS_sigma_xy**2,
            self.GPS_sigma_z**2
        ])
        
        self.R_baro = self.Baro_sigma_z**2
        self.R_mag = self.Mag_sigma_rad**2
    
    def get_imu_noise(self, dt: float) -> Dict[str, np.ndarray]:
        """Get IMU noise for given time step"""
        return {
            'accel_noise': self.IMU_accel_noise_density / np.sqrt(dt) * np.random.randn(3),
            'gyro_noise': self.IMU_gyro_noise_density / np.sqrt(dt) * np.random.randn(3),
            'accel_bias': self.IMU_accel_bias_instab * np.random.randn(3),
            'gyro_bias': self.IMU_gyro_bias_instab * np.random.randn(3)
        }
    
    def get_gps_noise(self) -> np.ndarray:
        """Get GPS noise vector"""
        return np.array([
            self.GPS_sigma_xy * np.random.randn(),
            self.GPS_sigma_xy * np.random.randn(),
            self.GPS_sigma_z * np.random.randn()
        ])
    
    def get_baro_noise(self) -> float:
        """Get barometer noise"""
        return self.Baro_sigma_z * np.random.randn()
    
    def get_mag_noise(self) -> float:
        """Get magnetometer noise"""
        return self.Mag_sigma_rad * np.random.randn()
    
    def get_adaptive_noise_scale(self, angular_rate: float) -> float:
        """Get adaptive noise scaling factor based on angular rate"""
        if not self.adaptive_noise:
            return self.noise_scale_normal
        
        if abs(angular_rate) > self.turn_threshold:
            return self.noise_scale_turn
        else:
            return self.noise_scale_normal
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert parameters to dictionary for easy access"""
        return {
            'solver_type': self.solver_type,
            'Ts_physics': self.Ts_physics,
            'Ts_IMU': self.Ts_IMU,
            'Ts_GPS': self.Ts_GPS,
            'Ts_Baro': self.Ts_Baro,
            'Ts_Mag': self.Ts_Mag,
            'sim_duration': self.sim_duration,
            'g': self.g,
            'mag_NED': self.mag_NED,
            'profile': self.profile,
            'arm_length': self.arm_length,
            'prop_radius': self.prop_radius,
            'CT': self.CT,
            'CQ': self.CQ,
            'Q_vel': self.Q_vel,
            'Q_att': self.Q_att,
            'mass': self.mass,
            'I': self.I,
            'IMU_accel_noise_density': self.IMU_accel_noise_density,
            'IMU_accel_bias_instab': self.IMU_accel_bias_instab,
            'IMU_gyro_noise_density': self.IMU_gyro_noise_density,
            'IMU_gyro_bias_instab': self.IMU_gyro_bias_instab,
            'GPS_sigma_xy': self.GPS_sigma_xy,
            'GPS_sigma_z': self.GPS_sigma_z,
            'Baro_sigma_z': self.Baro_sigma_z,
            'Mag_sigma_deg': self.Mag_sigma_deg,
            'Mag_sigma_rad': self.Mag_sigma_rad,
            'Q': self.Q,
            'R_gps': self.R_gps,
            'R_baro': self.R_baro,
            'R_mag': self.R_mag,
            'adaptive_noise': self.adaptive_noise,
            'turn_threshold': self.turn_threshold,
            'noise_scale_turn': self.noise_scale_turn,
            'noise_scale_normal': self.noise_scale_normal,
            'gps_outage_max_scale': self.gps_outage_max_scale,
            'innovation_gate_gps': self.innovation_gate_gps,
            'innovation_gate_baro': self.innovation_gate_baro,
            'innovation_gate_mag': self.innovation_gate_mag,
            'innovation_gate_imu': self.innovation_gate_imu,
            'max_angular_rate': self.max_angular_rate,
            'drag_coeff': self.drag_coeff,
            'gyro_momentum': self.gyro_momentum,
            'coriolis_enabled': self.coriolis_enabled
        }

# Global parameters instance
params = EKFParameters()

def get_params() -> EKFParameters:
    """Get global parameters instance"""
    return params

def set_profile(profile: str):
    """Set drone profile and update parameters"""
    global params
    params.profile = profile
    params._set_drone_parameters()
    params._initialize_ekf_matrices()

def update_ekf_tuning(Q: np.ndarray = None, R_gps: np.ndarray = None, 
                      R_baro: float = None, R_mag: float = None):
    """Update EKF tuning parameters"""
    global params
    if Q is not None:
        params.Q = Q
    if R_gps is not None:
        params.R_gps = R_gps
    if R_baro is not None:
        params.R_baro = R_baro
    if R_mag is not None:
        params.R_mag = R_mag

if __name__ == '__main__':
    # Test parameters
    params = EKFParameters()
    print("EKF Parameters initialized:")
    print(f"Profile: {params.profile}")
    print(f"Mass: {params.mass} kg")
    print(f"Inertia: {params.I}")
    print(f"Q matrix shape: {params.Q.shape}")
    print(f"R_gps shape: {params.R_gps.shape}")
    print(f"R_baro: {params.R_baro}")
    print(f"R_mag: {params.R_mag}")
