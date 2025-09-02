#!/usr/bin/env python3

import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from scipy import signal
from scipy.spatial.transform import Rotation as R
import os
import glob
from typing import Dict, Any, List, Optional

class RealTimeEKFAnalyzer:
    """Analyzer for real-time EKF simulation data"""
    
    def __init__(self, log_file: str):
        """Initialize analyzer with EKF log data"""
        self.log_file = log_file
        self.data = self.load_log_data()
        self.analyze_data()
    
    def load_log_data(self) -> Dict[str, Any]:
        """Load EKF log data from JSON file"""
        with open(self.log_file, 'r') as f:
            return json.load(f)
    
    def analyze_data(self):
        """Analyze the loaded data"""
        # Convert to numpy arrays
        self.timestamps = np.array(self.data['timestamps'])
        self.true_states = np.array(self.data['true_states'])
        self.estimated_states = np.array(self.data['estimated_states'])
        self.covariances = np.array(self.data['covariances'])
        
        # Extract state components
        self.true_positions = self.true_states[:, 0:3]
        self.true_velocities = self.true_states[:, 3:6]
        self.true_attitudes = self.true_states[:, 6:9]
        
        self.est_positions = self.estimated_states[:, 0:3]
        self.est_velocities = self.estimated_states[:, 3:6]
        self.est_attitudes = self.estimated_states[:, 6:9]
        
        # Convert attitudes to degrees
        self.true_attitudes_deg = np.rad2deg(self.true_attitudes)
        self.est_attitudes_deg = np.rad2deg(self.est_attitudes)
        
        # Calculate errors
        self.position_errors = np.linalg.norm(self.est_positions - self.true_positions, axis=1)
        self.velocity_errors = np.linalg.norm(self.est_velocities - self.true_velocities, axis=1)
        self.attitude_errors = np.linalg.norm(self.est_attitudes - self.true_attitudes, axis=1)
        
        # Calculate derived quantities
        self.calculate_derived_quantities()
        
        # Analyze sensor data
        self.analyze_sensor_data()
        
        # Analyze innovation statistics
        self.analyze_innovation_stats()
    
    def calculate_derived_quantities(self):
        """Calculate derived quantities from state estimates"""
        # Time differences
        self.dt = np.diff(self.timestamps)
        
        # Velocity magnitude
        self.true_velocity_magnitude = np.linalg.norm(self.true_velocities, axis=1)
        self.est_velocity_magnitude = np.linalg.norm(self.est_velocities, axis=1)
        
        # Acceleration from velocity differences
        if len(self.true_velocities) > 1:
            self.true_accelerations = np.diff(self.true_velocities, axis=0) / self.dt[:, np.newaxis]
            self.est_accelerations = np.diff(self.est_velocities, axis=0) / self.dt[:, np.newaxis]
        else:
            self.true_accelerations = np.array([])
            self.est_accelerations = np.array([])
        
        # Angular rates from attitude differences
        if len(self.true_attitudes) > 1:
            # Handle angle wrapping
            true_att_diff = np.diff(self.true_attitudes, axis=0)
            true_att_diff = np.arctan2(np.sin(true_att_diff), np.cos(true_att_diff))
            self.true_angular_rates = true_att_diff / self.dt[:, np.newaxis]
            
            est_att_diff = np.diff(self.est_attitudes, axis=0)
            est_att_diff = np.arctan2(np.sin(est_att_diff), np.cos(est_att_diff))
            self.est_angular_rates = est_att_diff / self.dt[:, np.newaxis]
        else:
            self.true_angular_rates = np.array([])
            self.est_angular_rates = np.array([])
        
        # Position uncertainty (diagonal of covariance)
        self.position_uncertainty = np.sqrt(np.diagonal(self.covariances[:, 0:3, 0:3], axis1=1, axis2=2))
        self.velocity_uncertainty = np.sqrt(np.diagonal(self.covariances[:, 3:6, 3:6], axis1=1, axis2=2))
        self.attitude_uncertainty = np.sqrt(np.diagonal(self.covariances[:, 6:9, 6:9], axis1=1, axis2=2))
        
        # Total uncertainty
        self.total_position_uncertainty = np.linalg.norm(self.position_uncertainty, axis=1)
        self.total_velocity_uncertainty = np.linalg.norm(self.velocity_uncertainty, axis=1)
        self.total_attitude_uncertainty = np.linalg.norm(self.attitude_uncertainty, axis=1)
    
    def analyze_sensor_data(self):
        """Analyze sensor measurement data"""
        # IMU data
        self.imu_data = []
        self.gps_data = []
        self.baro_data = []
        self.mag_data = []
        
        for measurement in self.data['sensor_measurements']:
            self.imu_data.append(measurement['imu'])
            self.gps_data.append(measurement['gps'])
            self.baro_data.append(measurement['baro'])
            self.mag_data.append(measurement['mag'])
        
        self.imu_data = np.array(self.imu_data)
        self.gps_data = np.array(self.gps_data)
        self.baro_data = np.array(self.baro_data)
        self.mag_data = np.array(self.mag_data)
        
        if len(self.imu_data) > 0:
            self.accel_measurements = self.imu_data[:, 0:3]
            self.gyro_measurements = self.imu_data[:, 3:6]
    
    def analyze_innovation_stats(self):
        """Analyze innovation statistics"""
        self.innovation_stats = self.data['innovation_stats']
        
        # Extract statistics for each sensor
        self.gps_stats = []
        self.baro_stats = []
        self.mag_stats = []
        
        for stats in self.innovation_stats:
            self.gps_stats.append(stats.get('gps', {}))
            self.baro_stats.append(stats.get('baro', {}))
            self.mag_stats.append(stats.get('mag', {}))
    
    def plot_comprehensive_analysis(self):
        """Plot comprehensive analysis results"""
        print("📊 Generating comprehensive analysis plots...")
        
        # Create comprehensive analysis plots
        fig, axes = plt.subplots(4, 3, figsize=(20, 16))
        
        time_axis = self.timestamps - self.timestamps[0]
        
        # Position comparison
        axes[0, 0].plot(time_axis, self.true_positions[:, 0], 'b-', linewidth=2, label='True')
        axes[0, 0].plot(time_axis, self.est_positions[:, 0], 'r--', linewidth=2, label='EKF')
        axes[0, 0].fill_between(time_axis, 
                               self.est_positions[:, 0] - 2*self.position_uncertainty[:, 0],
                               self.est_positions[:, 0] + 2*self.position_uncertainty[:, 0],
                               alpha=0.3, color='red')
        axes[0, 0].set_title('X Position (m)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        axes[0, 1].plot(time_axis, self.true_positions[:, 1], 'b-', linewidth=2, label='True')
        axes[0, 1].plot(time_axis, self.est_positions[:, 1], 'r--', linewidth=2, label='EKF')
        axes[0, 1].fill_between(time_axis, 
                               self.est_positions[:, 1] - 2*self.position_uncertainty[:, 1],
                               self.est_positions[:, 1] + 2*self.position_uncertainty[:, 1],
                               alpha=0.3, color='red')
        axes[0, 1].set_title('Y Position (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        axes[0, 2].plot(time_axis, self.true_positions[:, 2], 'b-', linewidth=2, label='True')
        axes[0, 2].plot(time_axis, self.est_positions[:, 2], 'r--', linewidth=2, label='EKF')
        axes[0, 2].fill_between(time_axis, 
                               self.est_positions[:, 2] - 2*self.position_uncertainty[:, 2],
                               self.est_positions[:, 2] + 2*self.position_uncertainty[:, 2],
                               alpha=0.3, color='red')
        axes[0, 2].set_title('Z Position (m)')
        axes[0, 2].legend()
        axes[0, 2].grid(True)
        
        # Velocity comparison
        axes[1, 0].plot(time_axis, self.true_velocities[:, 0], 'b-', linewidth=2, label='True')
        axes[1, 0].plot(time_axis, self.est_velocities[:, 0], 'r--', linewidth=2, label='EKF')
        axes[1, 0].fill_between(time_axis, 
                               self.est_velocities[:, 0] - 2*self.velocity_uncertainty[:, 0],
                               self.est_velocities[:, 0] + 2*self.velocity_uncertainty[:, 0],
                               alpha=0.3, color='red')
        axes[1, 0].set_title('X Velocity (m/s)')
        axes[1, 0].set_ylabel('Velocity (m/s)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        axes[1, 1].plot(time_axis, self.true_velocities[:, 1], 'b-', linewidth=2, label='True')
        axes[1, 1].plot(time_axis, self.est_velocities[:, 1], 'r--', linewidth=2, label='EKF')
        axes[1, 1].fill_between(time_axis, 
                               self.est_velocities[:, 1] - 2*self.velocity_uncertainty[:, 1],
                               self.est_velocities[:, 1] + 2*self.velocity_uncertainty[:, 1],
                               alpha=0.3, color='red')
        axes[1, 1].set_title('Y Velocity (m/s)')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        axes[1, 2].plot(time_axis, self.true_velocities[:, 2], 'b-', linewidth=2, label='True')
        axes[1, 2].plot(time_axis, self.est_velocities[:, 2], 'r--', linewidth=2, label='EKF')
        axes[1, 2].fill_between(time_axis, 
                               self.est_velocities[:, 2] - 2*self.velocity_uncertainty[:, 2],
                               self.est_velocities[:, 2] + 2*self.velocity_uncertainty[:, 2],
                               alpha=0.3, color='red')
        axes[1, 2].set_title('Z Velocity (m/s)')
        axes[1, 2].legend()
        axes[1, 2].grid(True)
        
        # Attitude comparison
        axes[2, 0].plot(time_axis, self.true_attitudes_deg[:, 0], 'b-', linewidth=2, label='True')
        axes[2, 0].plot(time_axis, self.est_attitudes_deg[:, 0], 'r--', linewidth=2, label='EKF')
        axes[2, 0].fill_between(time_axis, 
                               self.est_attitudes_deg[:, 0] - 2*np.rad2deg(self.attitude_uncertainty[:, 0]),
                               self.est_attitudes_deg[:, 0] + 2*np.rad2deg(self.attitude_uncertainty[:, 0]),
                               alpha=0.3, color='red')
        axes[2, 0].set_title('Roll (deg)')
        axes[2, 0].set_ylabel('Angle (deg)')
        axes[2, 0].legend()
        axes[2, 0].grid(True)
        
        axes[2, 1].plot(time_axis, self.true_attitudes_deg[:, 1], 'b-', linewidth=2, label='True')
        axes[2, 1].plot(time_axis, self.est_attitudes_deg[:, 1], 'r--', linewidth=2, label='EKF')
        axes[2, 1].fill_between(time_axis, 
                               self.est_attitudes_deg[:, 1] - 2*np.rad2deg(self.attitude_uncertainty[:, 1]),
                               self.est_attitudes_deg[:, 1] + 2*np.rad2deg(self.attitude_uncertainty[:, 1]),
                               alpha=0.3, color='red')
        axes[2, 1].set_title('Pitch (deg)')
        axes[2, 1].legend()
        axes[2, 1].grid(True)
        
        axes[2, 2].plot(time_axis, self.true_attitudes_deg[:, 2], 'b-', linewidth=2, label='True')
        axes[2, 2].plot(time_axis, self.est_attitudes_deg[:, 2], 'r--', linewidth=2, label='EKF')
        axes[2, 2].fill_between(time_axis, 
                               self.est_attitudes_deg[:, 2] - 2*np.rad2deg(self.attitude_uncertainty[:, 2]),
                               self.est_attitudes_deg[:, 2] + 2*np.rad2deg(self.attitude_uncertainty[:, 2]),
                               alpha=0.3, color='red')
        axes[2, 2].set_title('Yaw (deg)')
        axes[2, 2].legend()
        axes[2, 2].grid(True)
        
        # Error analysis
        axes[3, 0].plot(time_axis, self.position_errors, 'g-', linewidth=2)
        axes[3, 0].set_title('Position Error (m)')
        axes[3, 0].set_xlabel('Time (s)')
        axes[3, 0].set_ylabel('Error (m)')
        axes[3, 0].grid(True)
        
        axes[3, 1].plot(time_axis, self.velocity_errors, 'g-', linewidth=2)
        axes[3, 1].set_title('Velocity Error (m/s)')
        axes[3, 1].set_xlabel('Time (s)')
        axes[3, 1].set_ylabel('Error (m/s)')
        axes[3, 1].grid(True)
        
        axes[3, 2].plot(time_axis, np.rad2deg(self.attitude_errors), 'g-', linewidth=2)
        axes[3, 2].set_title('Attitude Error (deg)')
        axes[3, 2].set_xlabel('Time (s)')
        axes[3, 2].set_ylabel('Error (deg)')
        axes[3, 2].grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/comprehensive_ekf_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # 3D trajectory comparison
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.plot(self.true_positions[:, 0], self.true_positions[:, 1], self.true_positions[:, 2], 
                'b-', linewidth=2, label='True Trajectory')
        ax.plot(self.est_positions[:, 0], self.est_positions[:, 1], self.est_positions[:, 2], 
                'r--', linewidth=2, label='EKF Estimate')
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('3D Trajectory Comparison')
        ax.legend()
        ax.grid(True)
        
        plt.savefig('/u/12/akyuret1/unix/drone_sim/3d_trajectory_comparison.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        print("✅ Comprehensive analysis plots generated")
    
    def generate_detailed_report(self):
        """Generate detailed analysis report"""
        duration = self.timestamps[-1] - self.timestamps[0]
        
        # Calculate comprehensive statistics
        pos_stats = {
            'rmse': np.sqrt(np.mean(self.position_errors**2)),
            'mean_error': np.mean(self.position_errors),
            'std_error': np.std(self.position_errors),
            'max_error': np.max(self.position_errors),
            'min_error': np.min(self.position_errors)
        }
        
        vel_stats = {
            'rmse': np.sqrt(np.mean(self.velocity_errors**2)),
            'mean_error': np.mean(self.velocity_errors),
            'std_error': np.std(self.velocity_errors),
            'max_error': np.max(self.velocity_errors),
            'min_error': np.min(self.velocity_errors)
        }
        
        att_stats = {
            'rmse': np.sqrt(np.mean(self.attitude_errors**2)),
            'mean_error': np.mean(self.attitude_errors),
            'std_error': np.std(self.attitude_errors),
            'max_error': np.max(self.attitude_errors),
            'min_error': np.min(self.attitude_errors)
        }
        
        uncertainty_stats = {
            'position_mean': np.mean(self.total_position_uncertainty),
            'position_max': np.max(self.total_position_uncertainty),
            'velocity_mean': np.mean(self.total_velocity_uncertainty),
            'velocity_max': np.max(self.total_velocity_uncertainty),
            'attitude_mean': np.mean(self.total_attitude_uncertainty),
            'attitude_max': np.max(self.total_attitude_uncertainty)
        }
        
        # Sensor statistics
        sensor_stats = {
            'imu_samples': len(self.imu_data),
            'gps_samples': len(self.gps_data),
            'baro_samples': len(self.baro_data),
            'mag_samples': len(self.mag_data)
        }
        
        report = f"""
REAL-TIME EKF COMPREHENSIVE ANALYSIS REPORT
==========================================

Simulation Parameters:
- Duration: {duration:.2f} seconds
- Sample Rate: {len(self.timestamps)/duration:.1f} Hz
- Total Samples: {len(self.timestamps)}
- Drone Profile: {self.data['simulation_params']['drone_profile']}

POSITION ESTIMATION ANALYSIS:
- RMSE: {pos_stats['rmse']:.3f} m
- Mean Error: {pos_stats['mean_error']:.3f} m
- Std Error: {pos_stats['std_error']:.3f} m
- Max Error: {pos_stats['max_error']:.3f} m
- Min Error: {pos_stats['min_error']:.3f} m

VELOCITY ESTIMATION ANALYSIS:
- RMSE: {vel_stats['rmse']:.3f} m/s
- Mean Error: {vel_stats['mean_error']:.3f} m/s
- Std Error: {vel_stats['std_error']:.3f} m/s
- Max Error: {vel_stats['max_error']:.3f} m/s
- Min Error: {vel_stats['min_error']:.3f} m/s

ATTITUDE ESTIMATION ANALYSIS:
- RMSE: {att_stats['rmse']:.3f} rad ({np.rad2deg(att_stats['rmse']):.1f}°)
- Mean Error: {att_stats['mean_error']:.3f} rad ({np.rad2deg(att_stats['mean_error']):.1f}°)
- Std Error: {att_stats['std_error']:.3f} rad ({np.rad2deg(att_stats['std_error']):.1f}°)
- Max Error: {att_stats['max_error']:.3f} rad ({np.rad2deg(att_stats['max_error']):.1f}°)
- Min Error: {att_stats['min_error']:.3f} rad ({np.rad2deg(att_stats['min_error']):.1f}°)

UNCERTAINTY ANALYSIS:
- Mean Position Uncertainty: {uncertainty_stats['position_mean']:.3f} m
- Max Position Uncertainty: {uncertainty_stats['position_max']:.3f} m
- Mean Velocity Uncertainty: {uncertainty_stats['velocity_mean']:.3f} m/s
- Max Velocity Uncertainty: {uncertainty_stats['velocity_max']:.3f} m/s
- Mean Attitude Uncertainty: {uncertainty_stats['attitude_mean']:.3f} rad ({np.rad2deg(uncertainty_stats['attitude_mean']):.1f}°)
- Max Attitude Uncertainty: {uncertainty_stats['attitude_max']:.3f} rad ({np.rad2deg(uncertainty_stats['attitude_max']):.1f}°)

SENSOR DATA ANALYSIS:
- IMU Samples: {sensor_stats['imu_samples']}
- GPS Samples: {sensor_stats['gps_samples']}
- Barometer Samples: {sensor_stats['baro_samples']}
- Magnetometer Samples: {sensor_stats['mag_samples']}

PERFORMANCE ASSESSMENT:
- Position Estimation: {'EXCELLENT' if pos_stats['rmse'] < 0.1 else 'GOOD' if pos_stats['rmse'] < 0.5 else 'FAIR'}
- Velocity Estimation: {'EXCELLENT' if vel_stats['rmse'] < 0.05 else 'GOOD' if vel_stats['rmse'] < 0.2 else 'FAIR'}
- Attitude Estimation: {'EXCELLENT' if att_stats['rmse'] < 0.05 else 'GOOD' if att_stats['rmse'] < 0.1 else 'FAIR'}

INNOVATION STATISTICS:
"""
        
        if self.innovation_stats:
            latest_stats = self.innovation_stats[-1]
            for sensor_type, stats in latest_stats.items():
                if stats and 'count' in stats and stats['count'] > 0:
                    report += f"- {sensor_type.upper()}: {stats['count']} updates, RMS: {stats['rms_innovation']:.3f}, Rejection: {stats['rejection_rate']:.1%}\n"
        
        report += f"""
RECOMMENDATIONS:
1. {'Position estimation performance is excellent.' if pos_stats['rmse'] < 0.1 else 'Consider tuning Q matrix for better position estimation.' if pos_stats['rmse'] > 0.5 else 'Position estimation performance is acceptable.'}
2. {'Velocity estimation performance is excellent.' if vel_stats['rmse'] < 0.05 else 'Consider tuning process noise for better velocity estimation.' if vel_stats['rmse'] > 0.2 else 'Velocity estimation performance is acceptable.'}
3. {'Attitude estimation performance is excellent.' if att_stats['rmse'] < 0.05 else 'Consider improving magnetometer calibration or tuning.' if att_stats['rmse'] > 0.1 else 'Attitude estimation performance is acceptable.'}
4. Monitor innovation statistics for sensor health.
5. Consider adaptive noise scaling for different flight regimes.

---
Report generated by Real-Time EKF Analysis System
"""
        
        print(report)
        
        # Save report to file
        with open('/u/12/akyuret1/unix/drone_sim/real_time_ekf_analysis_report.txt', 'w') as f:
            f.write(report)
        
        return report
    
    def run_complete_analysis(self):
        """Run complete EKF data analysis"""
        print("🚁 REAL-TIME EKF COMPREHENSIVE ANALYSIS")
        print("=" * 50)
        
        self.plot_comprehensive_analysis()
        report = self.generate_detailed_report()
        
        print("\n🎉 Real-time EKF analysis completed!")
        print("📊 Analysis plots saved:")
        print("   - comprehensive_ekf_analysis.png")
        print("   - 3d_trajectory_comparison.png")
        print("📋 Detailed report saved: real_time_ekf_analysis_report.txt")

def main():
    """Main function"""
    # Find the most recent real-time EKF log file
    log_files = glob.glob('/u/12/akyuret1/unix/drone_sim/real_time_ekf_log_*.json')
    
    if not log_files:
        print("No real-time EKF log files found. Please run the real-time simulation first.")
        return
    
    # Use the most recent file
    latest_file = max(log_files, key=os.path.getctime)
    print(f"Analyzing real-time EKF data from: {latest_file}")
    
    # Create analyzer and run analysis
    analyzer = RealTimeEKFAnalyzer(latest_file)
    analyzer.run_complete_analysis()

if __name__ == '__main__':
    main()
