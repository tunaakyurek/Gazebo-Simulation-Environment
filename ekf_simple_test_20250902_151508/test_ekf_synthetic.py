#!/usr/bin/env python3

import sys
import os
sys.path.append('/u/12/akyuret1/unix/drone_sim')

import numpy as np
import matplotlib.pyplot as plt
from ekf_parameters import EKFParameters
from ekf_core import ExtendedKalmanFilter
from ekf_sensor_model import SensorModel

def test_ekf_synthetic():
    """Test EKF with synthetic trajectory data"""
    print("Testing EKF with synthetic data...")
    
    # Initialize components
    params = EKFParameters()
    ekf = ExtendedKalmanFilter(params)
    sensor_model = SensorModel(params)
    
    # Simulation parameters
    dt = 0.01
    duration = 30.0
    n_steps = int(duration / dt)
    
    # Generate synthetic trajectory (figure-8 pattern)
    t = np.linspace(0, duration, n_steps)
    
    # Simple figure-8 trajectory
    x_true = np.zeros((n_steps, 9))
    x_true[:, 0] = 5 * np.sin(0.2 * t)  # X position
    x_true[:, 1] = 5 * np.sin(0.4 * t)  # Y position
    x_true[:, 2] = 2 * np.sin(0.1 * t)  # Z position (altitude variation)
    x_true[:, 3] = 5 * 0.2 * np.cos(0.2 * t)  # X velocity
    x_true[:, 4] = 5 * 0.4 * np.cos(0.4 * t)  # Y velocity
    x_true[:, 5] = 2 * 0.1 * np.cos(0.1 * t)  # Z velocity
    x_true[:, 6] = 0.1 * np.sin(0.3 * t)  # Roll
    x_true[:, 7] = 0.1 * np.cos(0.3 * t)  # Pitch
    x_true[:, 8] = 0.2 * t  # Yaw (slow rotation)
    
    # Initialize EKF
    ekf.initialize(x_true[0])
    
    # Storage for results
    x_est = np.zeros((n_steps, 9))
    P_est = np.zeros((n_steps, 9, 9))
    innovation_stats = []
    
    print(f"Running simulation for {duration} seconds with {n_steps} steps...")
    
    # Run EKF simulation
    for i in range(n_steps):
        # Generate sensor measurements
        sensors = sensor_model.generate_sensors(x_true[i], t[i])
        
        # EKF step
        x_est[i], P_est[i] = ekf.step(
            sensors['imu'], dt,
            gps_meas=sensors['gps'],
            baro_meas=sensors['baro'],
            mag_meas=sensors['mag'],
            use_accel_tilt=True
        )
        
        # Collect innovation statistics
        if i % 100 == 0:  # Every second
            stats = ekf.get_innovation_stats()
            innovation_stats.append(stats)
        
        if i % 1000 == 0:
            print(f"Step {i}/{n_steps} ({i/n_steps*100:.1f}%)")
    
    print("✅ EKF simulation completed")
    
    # Calculate performance metrics
    pos_error = np.linalg.norm(x_est[:, 0:3] - x_true[:, 0:3], axis=1)
    vel_error = np.linalg.norm(x_est[:, 3:6] - x_true[:, 3:6], axis=1)
    att_error = np.linalg.norm(x_est[:, 6:9] - x_true[:, 6:9], axis=1)
    
    print(f"Position RMSE: {np.sqrt(np.mean(pos_error**2)):.3f} m")
    print(f"Velocity RMSE: {np.sqrt(np.mean(vel_error**2)):.3f} m/s")
    print(f"Attitude RMSE: {np.sqrt(np.mean(att_error**2)):.3f} rad ({np.rad2deg(np.sqrt(np.mean(att_error**2))):.1f}°)")
    
    # Plot results
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    
    # Position plots
    axes[0, 0].plot(t, x_true[:, 0], 'b-', label='True')
    axes[0, 0].plot(t, x_est[:, 0], 'r--', label='EKF')
    axes[0, 0].set_title('X Position')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(t, x_true[:, 1], 'b-', label='True')
    axes[0, 1].plot(t, x_est[:, 1], 'r--', label='EKF')
    axes[0, 1].set_title('Y Position')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    axes[0, 2].plot(t, x_true[:, 2], 'b-', label='True')
    axes[0, 2].plot(t, x_est[:, 2], 'r--', label='EKF')
    axes[0, 2].set_title('Z Position')
    axes[0, 2].legend()
    axes[0, 2].grid(True)
    
    # Velocity plots
    axes[1, 0].plot(t, x_true[:, 3], 'b-', label='True')
    axes[1, 0].plot(t, x_est[:, 3], 'r--', label='EKF')
    axes[1, 0].set_title('X Velocity')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(t, x_true[:, 4], 'b-', label='True')
    axes[1, 1].plot(t, x_est[:, 4], 'r--', label='EKF')
    axes[1, 1].set_title('Y Velocity')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    axes[1, 2].plot(t, x_true[:, 5], 'b-', label='True')
    axes[1, 2].plot(t, x_est[:, 5], 'r--', label='EKF')
    axes[1, 2].set_title('Z Velocity')
    axes[1, 2].legend()
    axes[1, 2].grid(True)
    
    # Attitude plots
    axes[2, 0].plot(t, np.rad2deg(x_true[:, 6]), 'b-', label='True')
    axes[2, 0].plot(t, np.rad2deg(x_est[:, 6]), 'r--', label='EKF')
    axes[2, 0].set_title('Roll (deg)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    axes[2, 1].plot(t, np.rad2deg(x_true[:, 7]), 'b-', label='True')
    axes[2, 1].plot(t, np.rad2deg(x_est[:, 7]), 'r--', label='EKF')
    axes[2, 1].set_title('Pitch (deg)')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    
    axes[2, 2].plot(t, np.rad2deg(x_true[:, 8]), 'b-', label='True')
    axes[2, 2].plot(t, np.rad2deg(x_est[:, 8]), 'r--', label='EKF')
    axes[2, 2].set_title('Yaw (deg)')
    axes[2, 2].legend()
    axes[2, 2].grid(True)
    
    plt.tight_layout()
    plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_synthetic_test.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Plot error analysis
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    axes[0].plot(t, pos_error)
    axes[0].set_title('Position Error (m)')
    axes[0].set_ylabel('Error (m)')
    axes[0].grid(True)
    
    axes[1].plot(t, vel_error)
    axes[1].set_title('Velocity Error (m/s)')
    axes[1].set_ylabel('Error (m/s)')
    axes[1].grid(True)
    
    axes[2].plot(t, np.rad2deg(att_error))
    axes[2].set_title('Attitude Error (deg)')
    axes[2].set_ylabel('Error (deg)')
    axes[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_error_analysis.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("✅ Plots saved: ekf_synthetic_test.png, ekf_error_analysis.png")
    
    # Generate performance report
    report = f"""
EKF SYNTHETIC DATA TEST REPORT
==============================

Test Duration: {duration} seconds
Sample Rate: {1/dt} Hz
Total Steps: {n_steps}

PERFORMANCE METRICS:
- Position RMSE: {np.sqrt(np.mean(pos_error**2)):.3f} m
- Velocity RMSE: {np.sqrt(np.mean(vel_error**2)):.3f} m/s
- Attitude RMSE: {np.sqrt(np.mean(att_error**2)):.3f} rad ({np.rad2deg(np.sqrt(np.mean(att_error**2))):.1f}°)

MAXIMUM ERRORS:
- Max Position Error: {np.max(pos_error):.3f} m
- Max Velocity Error: {np.max(vel_error):.3f} m/s
- Max Attitude Error: {np.max(att_error):.3f} rad ({np.rad2deg(np.max(att_error)):.1f}°)

PERFORMANCE ASSESSMENT:
- Position Estimation: {'EXCELLENT' if np.sqrt(np.mean(pos_error**2)) < 0.1 else 'GOOD' if np.sqrt(np.mean(pos_error**2)) < 0.5 else 'FAIR'}
- Velocity Estimation: {'EXCELLENT' if np.sqrt(np.mean(vel_error**2)) < 0.05 else 'GOOD' if np.sqrt(np.mean(vel_error**2)) < 0.2 else 'FAIR'}
- Attitude Estimation: {'EXCELLENT' if np.sqrt(np.mean(att_error**2)) < 0.05 else 'GOOD' if np.sqrt(np.mean(att_error**2)) < 0.1 else 'FAIR'}

INNOVATION STATISTICS:
"""
    
    if innovation_stats:
        latest_stats = innovation_stats[-1]
        for sensor_type, stats in latest_stats.items():
            if stats['count'] > 0:
                report += f"- {sensor_type.upper()}: {stats['count']} updates, RMS: {stats['rms_innovation']:.3f}, Rejection: {stats['rejection_rate']:.1%}\n"
    
    print(report)
    
    # Save report
    with open('/u/12/akyuret1/unix/drone_sim/ekf_synthetic_test_report.txt', 'w') as f:
        f.write(report)
    
    print("✅ Report saved: ekf_synthetic_test_report.txt")
    
    return True

if __name__ == '__main__':
    try:
        test_ekf_synthetic()
    except Exception as e:
        print(f"❌ EKF synthetic test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
