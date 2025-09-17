#!/usr/bin/env python3
"""
Fine-tuning script for waypoint-based EKF simulation
Based on performance analysis of waypoint_based_ekf_data_20250903_130135.json
"""

import numpy as np
from ekf_parameters import EKFParameters
from waypoint_based_ekf_simulation import WaypointBasedEKFSimulation

def create_fine_tuned_parameters():
    """Create fine-tuned EKF parameters based on performance analysis"""
    params = EKFParameters()
    
    # 1. MAGNETOMETER TUNING (Major issue: 22° innovation rejection)
    print("🔧 Fine-tuning magnetometer parameters...")
    params.Mag_sigma_deg = 5.0  # Increase from 2.0° to 5.0°
    params.Mag_sigma_rad = np.radians(params.Mag_sigma_deg)
    params.R_mag = (params.Mag_sigma_rad * 0.5)**2  # Increase tolerance
    
    # 2. ATTITUDE ESTIMATION IMPROVEMENT (131.3° RMSE is high)
    print("🔧 Fine-tuning attitude process noise...")
    params.Q_att = 0.05  # Increase from 0.02 for better tracking
    
    # 3. GPS TUNING (Current performance is good, minor optimization)
    print("🔧 Fine-tuning GPS parameters...")
    params.GPS_sigma_xy = 1.0  # Slightly increase from 0.8m
    params.GPS_sigma_z = 2.0   # Slightly increase from 1.6m
    params.R_gps = np.diag([
        (params.GPS_sigma_xy * 0.6)**2,  # Increase tolerance
        (params.GPS_sigma_xy * 0.6)**2,
        (params.GPS_sigma_z * 0.6)**2
    ])
    
    # 4. BAROMETER TUNING
    print("🔧 Fine-tuning barometer parameters...")
    params.Baro_sigma_z = 0.5  # Increase from 0.35m
    params.R_baro = (params.Baro_sigma_z * 0.4)**2
    
    # 5. VELOCITY PROCESS NOISE (Good performance, minor optimization)
    print("🔧 Fine-tuning velocity process noise...")
    params.Q_vel = 0.3  # Slightly decrease from 0.4
    
    # 6. ADAPTIVE NOISE SCALING
    print("🔧 Enabling adaptive noise scaling...")
    params.adaptive_noise = True
    params.turn_threshold = 0.3  # rad/s
    params.noise_scale_turn = 2.0
    
    return params

def run_fine_tuned_simulation():
    """Run simulation with fine-tuned parameters"""
    print("🚁 FINE-TUNED WAYPOINT-BASED EKF SIMULATION")
    print("=============================================")
    
    # Create fine-tuned parameters
    fine_tuned_params = create_fine_tuned_parameters()
    
    # Create simulation with custom parameters
    sim = WaypointBasedEKFSimulation(duration=120.0, dt=0.01)
    sim.params = fine_tuned_params
    sim.ekf = sim.ekf.__class__(fine_tuned_params)  # Reinitialize with new params
    sim.sensor_model = sim.sensor_model.__class__(fine_tuned_params)
    
    print("✅ Fine-tuned parameters applied:")
    print(f"   Magnetometer noise: {fine_tuned_params.Mag_sigma_deg}°")
    print(f"   Attitude process noise: {fine_tuned_params.Q_att}")
    print(f"   GPS noise: {fine_tuned_params.GPS_sigma_xy}m")
    print(f"   Barometer noise: {fine_tuned_params.Baro_sigma_z}m")
    print(f"   Velocity process noise: {fine_tuned_params.Q_vel}")
    print(f"   Adaptive noise: {fine_tuned_params.adaptive_noise}")
    
    # Run simulation
    sim.run_simulation()
    
    return sim

if __name__ == "__main__":
    sim = run_fine_tuned_simulation()
    
    print("\n🎯 FINE-TUNING COMPLETE!")
    print("Check the generated plots and data files for improved performance.")
