#!/usr/bin/env python3

import sys
import os
sys.path.append('/u/12/akyuret1/unix/drone_sim')

import numpy as np
import matplotlib.pyplot as plt
from ekf_parameters import EKFParameters
from ekf_core import ExtendedKalmanFilter
from ekf_sensor_model import SensorModel

def test_parameter_sensitivity():
    """Test EKF sensitivity to different parameter settings"""
    print("Testing EKF parameter sensitivity...")
    
    # Test different Q matrix settings
    q_scales = [0.1, 0.5, 1.0, 2.0, 5.0]
    results = []
    
    for q_scale in q_scales:
        print(f"Testing Q scale: {q_scale}")
        
        # Create parameters with scaled Q matrix
        params = EKFParameters()
        params.Q = params.Q * q_scale
        
        # Initialize EKF and sensor model
        ekf = ExtendedKalmanFilter(params)
        sensor_model = SensorModel(params)
        
        # Simple test trajectory
        dt = 0.01
        duration = 10.0
        n_steps = int(duration / dt)
        
        t = np.linspace(0, duration, n_steps)
        x_true = np.zeros((n_steps, 9))
        x_true[:, 0] = 2 * np.sin(0.5 * t)  # X position
        x_true[:, 1] = 2 * np.cos(0.5 * t)  # Y position
        x_true[:, 2] = 1 * np.sin(0.2 * t)  # Z position
        x_true[:, 3] = 2 * 0.5 * np.cos(0.5 * t)  # X velocity
        x_true[:, 4] = -2 * 0.5 * np.sin(0.5 * t)  # Y velocity
        x_true[:, 5] = 1 * 0.2 * np.cos(0.2 * t)  # Z velocity
        
        # Initialize EKF
        ekf.initialize(x_true[0])
        
        # Run simulation
        x_est = np.zeros((n_steps, 9))
        for i in range(n_steps):
            sensors = sensor_model.generate_sensors(x_true[i], t[i])
            x_est[i], _ = ekf.step(
                sensors['imu'], dt,
                gps_meas=sensors['gps'],
                baro_meas=sensors['baro'],
                mag_meas=sensors['mag']
            )
        
        # Calculate performance metrics
        pos_error = np.linalg.norm(x_est[:, 0:3] - x_true[:, 0:3], axis=1)
        vel_error = np.linalg.norm(x_est[:, 3:6] - x_true[:, 3:6], axis=1)
        
        pos_rmse = np.sqrt(np.mean(pos_error**2))
        vel_rmse = np.sqrt(np.mean(vel_error**2))
        
        results.append({
            'q_scale': q_scale,
            'pos_rmse': pos_rmse,
            'vel_rmse': vel_rmse
        })
        
        print(f"  Position RMSE: {pos_rmse:.3f} m")
        print(f"  Velocity RMSE: {vel_rmse:.3f} m/s")
    
    # Plot results
    q_scales = [r['q_scale'] for r in results]
    pos_rmses = [r['pos_rmse'] for r in results]
    vel_rmses = [r['vel_rmse'] for r in results]
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    axes[0].semilogx(q_scales, pos_rmses, 'bo-', linewidth=2, markersize=8)
    axes[0].set_xlabel('Q Scale Factor')
    axes[0].set_ylabel('Position RMSE (m)')
    axes[0].set_title('Position Estimation vs Q Scale')
    axes[0].grid(True)
    
    axes[1].semilogx(q_scales, vel_rmses, 'ro-', linewidth=2, markersize=8)
    axes[1].set_xlabel('Q Scale Factor')
    axes[1].set_ylabel('Velocity RMSE (m/s)')
    axes[1].set_title('Velocity Estimation vs Q Scale')
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_parameter_sensitivity.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Find optimal Q scale
    best_idx = np.argmin(pos_rmses)
    best_q_scale = q_scales[best_idx]
    
    print(f"\nOptimal Q scale: {best_q_scale}")
    print(f"Best position RMSE: {pos_rmses[best_idx]:.3f} m")
    print(f"Best velocity RMSE: {vel_rmses[best_idx]:.3f} m/s")
    
    # Generate tuning report
    report = f"""
EKF PARAMETER TUNING TEST REPORT
================================

Q Scale Sensitivity Analysis:
"""
    
    for result in results:
        report += f"Q Scale {result['q_scale']}: Position RMSE {result['pos_rmse']:.3f} m, Velocity RMSE {result['vel_rmse']:.3f} m/s\n"
    
    report += f"\nOptimal Q Scale: {best_q_scale}\n"
    report += f"Best Position RMSE: {pos_rmses[best_idx]:.3f} m\n"
    report += f"Best Velocity RMSE: {vel_rmses[best_idx]:.3f} m/s\n"
    
    print(report)
    
    # Save report
    with open('/u/12/akyuret1/unix/drone_sim/ekf_tuning_test_report.txt', 'w') as f:
        f.write(report)
    
    print("✅ Parameter sensitivity analysis completed")
    print("✅ Plots saved: ekf_parameter_sensitivity.png")
    print("✅ Report saved: ekf_tuning_test_report.txt")
    
    return best_q_scale

if __name__ == '__main__':
    try:
        optimal_q_scale = test_parameter_sensitivity()
        print(f"\n🎉 EKF parameter tuning test completed!")
        print(f"Recommended Q scale factor: {optimal_q_scale}")
    except Exception as e:
        print(f"❌ EKF tuning test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
