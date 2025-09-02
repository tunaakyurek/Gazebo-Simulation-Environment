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

class EKFDataAnalyzer:
    """Analyzer for EKF log data from Gazebo integration"""
    
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
        self.estimated_states = np.array(self.data['estimated_states'])
        self.covariances = np.array(self.data['covariances'])
        
        # Extract state components
        self.positions = self.estimated_states[:, 0:3]
        self.velocities = self.estimated_states[:, 3:6]
        self.attitudes = self.estimated_states[:, 6:9]
        
        # Convert attitudes to degrees
        self.attitudes_deg = np.rad2deg(self.attitudes)
        
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
        self.velocity_magnitude = np.linalg.norm(self.velocities, axis=1)
        
        # Acceleration from velocity differences
        if len(self.velocities) > 1:
            self.accelerations = np.diff(self.velocities, axis=0) / self.dt[:, np.newaxis]
        else:
            self.accelerations = np.array([])
        
        # Angular rates from attitude differences
        if len(self.attitudes) > 1:
            # Handle angle wrapping
            att_diff = np.diff(self.attitudes, axis=0)
            att_diff = np.arctan2(np.sin(att_diff), np.cos(att_diff))  # Wrap to [-pi, pi]
            self.angular_rates = att_diff / self.dt[:, np.newaxis]
        else:
            self.angular_rates = np.array([])
        
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
        self.imu_data = np.array(self.data['imu_measurements'])
        if len(self.imu_data) > 0:
            self.accel_measurements = self.imu_data[:, 0:3]
            self.gyro_measurements = self.imu_data[:, 3:6]
        
        # GPS data
        self.gps_data = np.array(self.data['gps_measurements'])
        self.gps_valid = ~np.isnan(self.gps_data).any(axis=1)
        
        # Barometer data
        self.baro_data = np.array(self.data['baro_measurements'])
        self.baro_valid = ~np.isnan(self.baro_data)
        
        # Magnetometer data
        self.mag_data = np.array(self.data['mag_measurements'])
        self.mag_valid = ~np.isnan(self.mag_data)
    
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
    
    def plot_3d_trajectory(self):
        """Plot 3D trajectory with uncertainty"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory
        ax.plot(self.positions[:, 0], self.positions[:, 1], self.positions[:, 2], 
                'b-', linewidth=2, label='EKF Trajectory')
        
        # Plot uncertainty ellipsoids at selected points
        n_points = min(20, len(self.positions))
        indices = np.linspace(0, len(self.positions)-1, n_points, dtype=int)
        
        for i in indices:
            pos = self.positions[i]
            cov_pos = self.covariances[i, 0:3, 0:3]
            
            # Create uncertainty ellipsoid
            eigenvals, eigenvecs = np.linalg.eigh(cov_pos)
            eigenvals = np.maximum(eigenvals, 1e-6)  # Avoid zero eigenvalues
            
            # Scale by 2-sigma
            scale = 2.0
            radii = scale * np.sqrt(eigenvals)
            
            # Create ellipsoid
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            x = radii[0] * np.outer(np.cos(u), np.sin(v))
            y = radii[1] * np.outer(np.sin(u), np.sin(v))
            z = radii[2] * np.outer(np.ones(np.size(u)), np.cos(v))
            
            # Rotate and translate
            for j in range(len(x)):
                for k in range(len(x[0])):
                    point = np.array([x[j, k], y[j, k], z[j, k]])
                    point = eigenvecs @ point + pos
                    x[j, k], y[j, k], z[j, k] = point[0], point[1], point[2]
            
            ax.plot_surface(x, y, z, alpha=0.1, color='red')
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('EKF 3D Trajectory with Uncertainty Ellipsoids')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_3d_trajectory.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_state_estimates(self):
        """Plot state estimates over time"""
        fig, axes = plt.subplots(3, 3, figsize=(15, 12))
        
        time_axis = self.timestamps - self.timestamps[0]
        
        # Position estimates
        axes[0, 0].plot(time_axis, self.positions[:, 0], 'b-', linewidth=2, label='X')
        axes[0, 0].fill_between(time_axis, 
                               self.positions[:, 0] - 2*self.position_uncertainty[:, 0],
                               self.positions[:, 0] + 2*self.position_uncertainty[:, 0],
                               alpha=0.3, color='blue')
        axes[0, 0].set_title('Position X (m)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].grid(True)
        axes[0, 0].legend()
        
        axes[0, 1].plot(time_axis, self.positions[:, 1], 'g-', linewidth=2, label='Y')
        axes[0, 1].fill_between(time_axis, 
                               self.positions[:, 1] - 2*self.position_uncertainty[:, 1],
                               self.positions[:, 1] + 2*self.position_uncertainty[:, 1],
                               alpha=0.3, color='green')
        axes[0, 1].set_title('Position Y (m)')
        axes[0, 1].grid(True)
        axes[0, 1].legend()
        
        axes[0, 2].plot(time_axis, self.positions[:, 2], 'r-', linewidth=2, label='Z')
        axes[0, 2].fill_between(time_axis, 
                               self.positions[:, 2] - 2*self.position_uncertainty[:, 2],
                               self.positions[:, 2] + 2*self.position_uncertainty[:, 2],
                               alpha=0.3, color='red')
        axes[0, 2].set_title('Position Z (m)')
        axes[0, 2].grid(True)
        axes[0, 2].legend()
        
        # Velocity estimates
        axes[1, 0].plot(time_axis, self.velocities[:, 0], 'b-', linewidth=2, label='Vx')
        axes[1, 0].fill_between(time_axis, 
                               self.velocities[:, 0] - 2*self.velocity_uncertainty[:, 0],
                               self.velocities[:, 0] + 2*self.velocity_uncertainty[:, 0],
                               alpha=0.3, color='blue')
        axes[1, 0].set_title('Velocity X (m/s)')
        axes[1, 0].set_ylabel('Velocity (m/s)')
        axes[1, 0].grid(True)
        axes[1, 0].legend()
        
        axes[1, 1].plot(time_axis, self.velocities[:, 1], 'g-', linewidth=2, label='Vy')
        axes[1, 1].fill_between(time_axis, 
                               self.velocities[:, 1] - 2*self.velocity_uncertainty[:, 1],
                               self.velocities[:, 1] + 2*self.velocity_uncertainty[:, 1],
                               alpha=0.3, color='green')
        axes[1, 1].set_title('Velocity Y (m/s)')
        axes[1, 1].grid(True)
        axes[1, 1].legend()
        
        axes[1, 2].plot(time_axis, self.velocities[:, 2], 'r-', linewidth=2, label='Vz')
        axes[1, 2].fill_between(time_axis, 
                               self.velocities[:, 2] - 2*self.velocity_uncertainty[:, 2],
                               self.velocities[:, 2] + 2*self.velocity_uncertainty[:, 2],
                               alpha=0.3, color='red')
        axes[1, 2].set_title('Velocity Z (m/s)')
        axes[1, 2].grid(True)
        axes[1, 2].legend()
        
        # Attitude estimates
        axes[2, 0].plot(time_axis, self.attitudes_deg[:, 0], 'b-', linewidth=2, label='Roll')
        axes[2, 0].fill_between(time_axis, 
                               self.attitudes_deg[:, 0] - 2*np.rad2deg(self.attitude_uncertainty[:, 0]),
                               self.attitudes_deg[:, 0] + 2*np.rad2deg(self.attitude_uncertainty[:, 0]),
                               alpha=0.3, color='blue')
        axes[2, 0].set_title('Roll (deg)')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_ylabel('Angle (deg)')
        axes[2, 0].grid(True)
        axes[2, 0].legend()
        
        axes[2, 1].plot(time_axis, self.attitudes_deg[:, 1], 'g-', linewidth=2, label='Pitch')
        axes[2, 1].fill_between(time_axis, 
                               self.attitudes_deg[:, 1] - 2*np.rad2deg(self.attitude_uncertainty[:, 1]),
                               self.attitudes_deg[:, 1] + 2*np.rad2deg(self.attitude_uncertainty[:, 1]),
                               alpha=0.3, color='green')
        axes[2, 1].set_title('Pitch (deg)')
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].grid(True)
        axes[2, 1].legend()
        
        axes[2, 2].plot(time_axis, self.attitudes_deg[:, 2], 'r-', linewidth=2, label='Yaw')
        axes[2, 2].fill_between(time_axis, 
                               self.attitudes_deg[:, 2] - 2*np.rad2deg(self.attitude_uncertainty[:, 2]),
                               self.attitudes_deg[:, 2] + 2*np.rad2deg(self.attitude_uncertainty[:, 2]),
                               alpha=0.3, color='red')
        axes[2, 2].set_title('Yaw (deg)')
        axes[2, 2].set_xlabel('Time (s)')
        axes[2, 2].grid(True)
        axes[2, 2].legend()
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_state_estimates.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_uncertainty_evolution(self):
        """Plot uncertainty evolution over time"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        time_axis = self.timestamps - self.timestamps[0]
        
        # Position uncertainty
        axes[0, 0].plot(time_axis, self.total_position_uncertainty, 'b-', linewidth=2)
        axes[0, 0].set_title('Total Position Uncertainty')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Uncertainty (m)')
        axes[0, 0].grid(True)
        
        # Velocity uncertainty
        axes[0, 1].plot(time_axis, self.total_velocity_uncertainty, 'g-', linewidth=2)
        axes[0, 1].set_title('Total Velocity Uncertainty')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Uncertainty (m/s)')
        axes[0, 1].grid(True)
        
        # Attitude uncertainty
        axes[1, 0].plot(time_axis, np.rad2deg(self.total_attitude_uncertainty), 'r-', linewidth=2)
        axes[1, 0].set_title('Total Attitude Uncertainty')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Uncertainty (deg)')
        axes[1, 0].grid(True)
        
        # Velocity magnitude
        axes[1, 1].plot(time_axis, self.velocity_magnitude, 'purple', linewidth=2)
        axes[1, 1].set_title('Velocity Magnitude')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Speed (m/s)')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_uncertainty_evolution.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_sensor_data(self):
        """Plot sensor measurement data"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        time_axis = self.timestamps - self.timestamps[0]
        
        # IMU accelerometer data
        if len(self.accel_measurements) > 0:
            axes[0, 0].plot(time_axis, self.accel_measurements[:, 0], 'r-', linewidth=1, label='Ax')
            axes[0, 0].plot(time_axis, self.accel_measurements[:, 1], 'g-', linewidth=1, label='Ay')
            axes[0, 0].plot(time_axis, self.accel_measurements[:, 2], 'b-', linewidth=1, label='Az')
            axes[0, 0].set_title('IMU Accelerometer (m/s²)')
            axes[0, 0].set_ylabel('Acceleration (m/s²)')
            axes[0, 0].legend()
            axes[0, 0].grid(True)
        
        # IMU gyroscope data
        if len(self.gyro_measurements) > 0:
            axes[0, 1].plot(time_axis, self.gyro_measurements[:, 0], 'r-', linewidth=1, label='ωx')
            axes[0, 1].plot(time_axis, self.gyro_measurements[:, 1], 'g-', linewidth=1, label='ωy')
            axes[0, 1].plot(time_axis, self.gyro_measurements[:, 2], 'b-', linewidth=1, label='ωz')
            axes[0, 1].set_title('IMU Gyroscope (rad/s)')
            axes[0, 1].set_ylabel('Angular Velocity (rad/s)')
            axes[0, 1].legend()
            axes[0, 1].grid(True)
        
        # GPS data
        if np.any(self.gps_valid):
            gps_time = time_axis[self.gps_valid]
            gps_pos = self.gps_data[self.gps_valid]
            axes[1, 0].plot(gps_time, gps_pos[:, 0], 'ro', markersize=3, label='GPS X')
            axes[1, 0].plot(gps_time, gps_pos[:, 1], 'go', markersize=3, label='GPS Y')
            axes[1, 0].plot(gps_time, gps_pos[:, 2], 'bo', markersize=3, label='GPS Z')
            axes[1, 0].set_title('GPS Measurements')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Position (m)')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
        
        # Barometer data
        if np.any(self.baro_valid):
            baro_time = time_axis[self.baro_valid]
            baro_alt = self.baro_data[self.baro_valid]
            axes[1, 1].plot(baro_time, baro_alt, 'mo', markersize=3, label='Baro Altitude')
            axes[1, 1].set_title('Barometer Measurements')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Altitude (m)')
            axes[1, 1].legend()
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_sensor_data.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def generate_summary_report(self):
        """Generate a summary report of the EKF analysis"""
        duration = self.timestamps[-1] - self.timestamps[0]
        
        # Calculate statistics
        pos_stats = {
            'mean': np.mean(self.positions, axis=0),
            'std': np.std(self.positions, axis=0),
            'range': np.ptp(self.positions, axis=0)
        }
        
        vel_stats = {
            'mean': np.mean(self.velocities, axis=0),
            'std': np.std(self.velocities, axis=0),
            'max': np.max(np.abs(self.velocities), axis=0)
        }
        
        att_stats = {
            'mean': np.mean(self.attitudes_deg, axis=0),
            'std': np.std(self.attitudes_deg, axis=0),
            'range': np.ptp(self.attitudes_deg, axis=0)
        }
        
        uncertainty_stats = {
            'position_mean': np.mean(self.total_position_uncertainty),
            'position_max': np.max(self.total_position_uncertainty),
            'velocity_mean': np.mean(self.total_velocity_uncertainty),
            'velocity_max': np.max(self.total_velocity_uncertainty),
            'attitude_mean': np.mean(np.rad2deg(self.total_attitude_uncertainty)),
            'attitude_max': np.max(np.rad2deg(self.total_attitude_uncertainty))
        }
        
        # Sensor statistics
        sensor_stats = {
            'gps_updates': np.sum(self.gps_valid),
            'baro_updates': np.sum(self.baro_valid),
            'mag_updates': np.sum(self.mag_valid),
            'imu_samples': len(self.imu_data)
        }
        
        report = f"""
        EKF GAZEBO INTEGRATION ANALYSIS REPORT
        =====================================
        
        Flight Duration: {duration:.2f} seconds
        Total Samples: {len(self.timestamps)}
        Sample Rate: {len(self.timestamps)/duration:.1f} Hz
        
        POSITION STATISTICS:
        - Mean Position: [{pos_stats['mean'][0]:.2f}, {pos_stats['mean'][1]:.2f}, {pos_stats['mean'][2]:.2f}] m
        - Position Std: [{pos_stats['std'][0]:.2f}, {pos_stats['std'][1]:.2f}, {pos_stats['std'][2]:.2f}] m
        - Position Range: [{pos_stats['range'][0]:.2f}, {pos_stats['range'][1]:.2f}, {pos_stats['range'][2]:.2f}] m
        
        VELOCITY STATISTICS:
        - Mean Velocity: [{vel_stats['mean'][0]:.2f}, {vel_stats['mean'][1]:.2f}, {vel_stats['mean'][2]:.2f}] m/s
        - Velocity Std: [{vel_stats['std'][0]:.2f}, {vel_stats['std'][1]:.2f}, {vel_stats['std'][2]:.2f}] m/s
        - Max Velocity: [{vel_stats['max'][0]:.2f}, {vel_stats['max'][1]:.2f}, {vel_stats['max'][2]:.2f}] m/s
        
        ATTITUDE STATISTICS:
        - Mean Attitude: [{att_stats['mean'][0]:.1f}, {att_stats['mean'][1]:.1f}, {att_stats['mean'][2]:.1f}] deg
        - Attitude Std: [{att_stats['std'][0]:.1f}, {att_stats['std'][1]:.1f}, {att_stats['std'][2]:.1f}] deg
        - Attitude Range: [{att_stats['range'][0]:.1f}, {att_stats['range'][1]:.1f}, {att_stats['range'][2]:.1f}] deg
        
        UNCERTAINTY STATISTICS:
        - Mean Position Uncertainty: {uncertainty_stats['position_mean']:.3f} m
        - Max Position Uncertainty: {uncertainty_stats['position_max']:.3f} m
        - Mean Velocity Uncertainty: {uncertainty_stats['velocity_mean']:.3f} m/s
        - Max Velocity Uncertainty: {uncertainty_stats['velocity_max']:.3f} m/s
        - Mean Attitude Uncertainty: {uncertainty_stats['attitude_mean']:.2f} deg
        - Max Attitude Uncertainty: {uncertainty_stats['attitude_max']:.2f} deg
        
        SENSOR STATISTICS:
        - GPS Updates: {sensor_stats['gps_updates']} ({sensor_stats['gps_updates']/len(self.timestamps)*100:.1f}%)
        - Barometer Updates: {sensor_stats['baro_updates']} ({sensor_stats['baro_updates']/len(self.timestamps)*100:.1f}%)
        - Magnetometer Updates: {sensor_stats['mag_updates']} ({sensor_stats['mag_updates']/len(self.timestamps)*100:.1f}%)
        - IMU Samples: {sensor_stats['imu_samples']}
        
        EKF PERFORMANCE ASSESSMENT:
        - Position Estimation: {'EXCELLENT' if uncertainty_stats['position_mean'] < 0.1 else 'GOOD' if uncertainty_stats['position_mean'] < 0.5 else 'FAIR'}
        - Velocity Estimation: {'EXCELLENT' if uncertainty_stats['velocity_mean'] < 0.05 else 'GOOD' if uncertainty_stats['velocity_mean'] < 0.2 else 'FAIR'}
        - Attitude Estimation: {'EXCELLENT' if uncertainty_stats['attitude_mean'] < 1.0 else 'GOOD' if uncertainty_stats['attitude_mean'] < 3.0 else 'FAIR'}
        - Overall Performance: {'EXCELLENT' if uncertainty_stats['position_mean'] < 0.2 and uncertainty_stats['velocity_mean'] < 0.1 else 'GOOD' if uncertainty_stats['position_mean'] < 0.5 and uncertainty_stats['velocity_mean'] < 0.3 else 'FAIR'}
        """
        
        print(report)
        
        # Save report to file
        with open('/u/12/akyuret1/unix/drone_sim/ekf_analysis_report.txt', 'w') as f:
            f.write(report)
    
    def run_complete_analysis(self):
        """Run complete EKF data analysis"""
        print("Starting complete EKF data analysis...")
        
        self.plot_3d_trajectory()
        self.plot_state_estimates()
        self.plot_uncertainty_evolution()
        self.plot_sensor_data()
        self.generate_summary_report()
        
        print("EKF analysis complete! All plots saved to /u/12/akyuret1/unix/drone_sim/")

def main():
    """Main function"""
    # Find the most recent EKF log file
    log_files = glob.glob('/u/12/akyuret1/unix/drone_sim/ekf_log_*.json')
    
    if not log_files:
        print("No EKF log files found. Please run the EKF Gazebo integration first.")
        return
    
    # Use the most recent file
    latest_file = max(log_files, key=os.path.getctime)
    print(f"Analyzing EKF data from: {latest_file}")
    
    # Create analyzer and run analysis
    analyzer = EKFDataAnalyzer(latest_file)
    analyzer.run_complete_analysis()

if __name__ == '__main__':
    main()
