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

class EKFAnalyzer:
    def __init__(self, flight_data_file):
        """Initialize EKF analyzer with flight data"""
        self.flight_data = self.load_flight_data(flight_data_file)
        self.positions = np.array(self.flight_data['positions'])
        self.orientations = np.array(self.flight_data['orientations'])
        self.targets = np.array(self.flight_data['target_positions'])
        self.timestamps = np.array(self.flight_data['timestamps'])
        
        # Calculate derived data
        self.calculate_derived_data()
        
    def load_flight_data(self, filename):
        """Load flight data from JSON file"""
        with open(filename, 'r') as f:
            return json.load(f)
    
    def calculate_derived_data(self):
        """Calculate derived data for analysis"""
        # Calculate velocities from positions
        dt = np.diff(self.timestamps)
        self.velocities = np.diff(self.positions, axis=0) / dt[:, np.newaxis]
        
        # Calculate accelerations from velocities
        if len(self.velocities) > 1:
            dt_vel = np.diff(self.timestamps[1:])
            self.accelerations = np.diff(self.velocities, axis=0) / dt_vel[:, np.newaxis]
        else:
            self.accelerations = np.array([])
        
        # Calculate Euler angles from quaternions
        self.euler_angles = np.zeros((len(self.orientations), 3))
        for i, quat in enumerate(self.orientations):
            r = R.from_quat(quat)
            self.euler_angles[i] = r.as_euler('xyz', degrees=True)
        
        # Calculate estimation errors (difference from target)
        self.position_errors = self.positions - self.targets
        self.position_error_magnitude = np.linalg.norm(self.position_errors, axis=1)
        
        # Calculate trajectory smoothness (second derivative)
        if len(self.positions) > 2:
            self.trajectory_curvature = self.calculate_curvature()
        else:
            self.trajectory_curvature = np.array([])
    
    def calculate_curvature(self):
        """Calculate trajectory curvature"""
        if len(self.positions) < 3:
            return np.array([])
            
        # Calculate first and second derivatives
        dt = np.diff(self.timestamps)
        velocity = np.diff(self.positions, axis=0) / dt[:, np.newaxis]
        
        if len(velocity) < 2:
            return np.array([])
            
        dt_vel = np.diff(self.timestamps[1:])
        acceleration = np.diff(velocity, axis=0) / dt_vel[:, np.newaxis]
        
        # Calculate curvature using the formula: |v x a| / |v|^3
        velocity_mag = np.linalg.norm(velocity[1:], axis=1)
        cross_product = np.cross(velocity[1:], acceleration)
        cross_product_mag = np.linalg.norm(cross_product, axis=1)
        
        # Avoid division by zero
        curvature = np.zeros_like(velocity_mag)
        mask = velocity_mag > 1e-6
        curvature[mask] = cross_product_mag[mask] / (velocity_mag[mask] ** 3)
        
        return curvature
    
    def plot_3d_trajectory(self):
        """Plot 3D trajectory"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot actual trajectory
        ax.plot(self.positions[:, 0], self.positions[:, 1], self.positions[:, 2], 
                'b-', linewidth=2, label='Actual Trajectory')
        
        # Plot target waypoints
        ax.scatter(self.targets[:, 0], self.targets[:, 1], self.targets[:, 2], 
                  c='red', s=100, marker='o', label='Target Waypoints')
        
        # Plot start and end points
        ax.scatter(self.positions[0, 0], self.positions[0, 1], self.positions[0, 2], 
                  c='green', s=150, marker='^', label='Start')
        ax.scatter(self.positions[-1, 0], self.positions[-1, 1], self.positions[-1, 2], 
                  c='orange', s=150, marker='v', label='End')
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('3D Flight Trajectory')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/3d_trajectory.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_position_errors(self):
        """Plot position estimation errors"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        time_axis = self.timestamps - self.timestamps[0]
        
        # X position error
        axes[0, 0].plot(time_axis, self.position_errors[:, 0], 'r-', linewidth=2)
        axes[0, 0].set_title('X Position Error')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Error (m)')
        axes[0, 0].grid(True)
        
        # Y position error
        axes[0, 1].plot(time_axis, self.position_errors[:, 1], 'g-', linewidth=2)
        axes[0, 1].set_title('Y Position Error')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Error (m)')
        axes[0, 1].grid(True)
        
        # Z position error
        axes[1, 0].plot(time_axis, self.position_errors[:, 2], 'b-', linewidth=2)
        axes[1, 0].set_title('Z Position Error')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Error (m)')
        axes[1, 0].grid(True)
        
        # Total position error magnitude
        axes[1, 1].plot(time_axis, self.position_error_magnitude, 'k-', linewidth=2)
        axes[1, 1].set_title('Total Position Error Magnitude')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Error Magnitude (m)')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/position_errors.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_velocity_analysis(self):
        """Plot velocity analysis"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        time_axis = self.timestamps[1:] - self.timestamps[0]
        
        # Velocity components
        axes[0, 0].plot(time_axis, self.velocities[:, 0], 'r-', linewidth=2, label='Vx')
        axes[0, 0].plot(time_axis, self.velocities[:, 1], 'g-', linewidth=2, label='Vy')
        axes[0, 0].plot(time_axis, self.velocities[:, 2], 'b-', linewidth=2, label='Vz')
        axes[0, 0].set_title('Velocity Components')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Velocity (m/s)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Velocity magnitude
        velocity_magnitude = np.linalg.norm(self.velocities, axis=1)
        axes[0, 1].plot(time_axis, velocity_magnitude, 'k-', linewidth=2)
        axes[0, 1].set_title('Velocity Magnitude')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Speed (m/s)')
        axes[0, 1].grid(True)
        
        # Acceleration components
        if len(self.accelerations) > 0:
            time_acc = self.timestamps[2:] - self.timestamps[0]
            axes[1, 0].plot(time_acc, self.accelerations[:, 0], 'r-', linewidth=2, label='Ax')
            axes[1, 0].plot(time_acc, self.accelerations[:, 1], 'g-', linewidth=2, label='Ay')
            axes[1, 0].plot(time_acc, self.accelerations[:, 2], 'b-', linewidth=2, label='Az')
            axes[1, 0].set_title('Acceleration Components')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Acceleration (m/s²)')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
        
        # Trajectory curvature
        if len(self.trajectory_curvature) > 0:
            time_curv = self.timestamps[2:] - self.timestamps[0]
            axes[1, 1].plot(time_curv, self.trajectory_curvature, 'purple', linewidth=2)
            axes[1, 1].set_title('Trajectory Curvature')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Curvature (1/m)')
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/velocity_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_orientation_analysis(self):
        """Plot orientation analysis"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        time_axis = self.timestamps - self.timestamps[0]
        
        # Euler angles
        axes[0, 0].plot(time_axis, self.euler_angles[:, 0], 'r-', linewidth=2, label='Roll')
        axes[0, 0].plot(time_axis, self.euler_angles[:, 1], 'g-', linewidth=2, label='Pitch')
        axes[0, 0].plot(time_axis, self.euler_angles[:, 2], 'b-', linewidth=2, label='Yaw')
        axes[0, 0].set_title('Euler Angles')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Angle (degrees)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Quaternion components
        axes[0, 1].plot(time_axis, self.orientations[:, 0], 'r-', linewidth=2, label='qx')
        axes[0, 1].plot(time_axis, self.orientations[:, 1], 'g-', linewidth=2, label='qy')
        axes[0, 1].plot(time_axis, self.orientations[:, 2], 'b-', linewidth=2, label='qz')
        axes[0, 1].plot(time_axis, self.orientations[:, 3], 'k-', linewidth=2, label='qw')
        axes[0, 1].set_title('Quaternion Components')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Quaternion Value')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # Quaternion magnitude (should be 1)
        quat_magnitude = np.linalg.norm(self.orientations, axis=1)
        axes[1, 0].plot(time_axis, quat_magnitude, 'purple', linewidth=2)
        axes[1, 0].set_title('Quaternion Magnitude')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Magnitude')
        axes[1, 0].axhline(y=1.0, color='r', linestyle='--', alpha=0.7)
        axes[1, 0].grid(True)
        
        # Angular velocity (if available from IMU)
        if len(self.flight_data['imu_data']) > 0:
            angular_vel = np.array([imu['angular_velocity'] for imu in self.flight_data['imu_data']])
            axes[1, 1].plot(time_axis, angular_vel[:, 0], 'r-', linewidth=2, label='ωx')
            axes[1, 1].plot(time_axis, angular_vel[:, 1], 'g-', linewidth=2, label='ωy')
            axes[1, 1].plot(time_axis, angular_vel[:, 2], 'b-', linewidth=2, label='ωz')
            axes[1, 1].set_title('Angular Velocity (IMU)')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Angular Velocity (rad/s)')
            axes[1, 1].legend()
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/orientation_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_ekf_performance_metrics(self):
        """Plot EKF performance metrics"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        time_axis = self.timestamps - self.timestamps[0]
        
        # Position error statistics
        axes[0, 0].plot(time_axis, self.position_error_magnitude, 'b-', linewidth=2)
        axes[0, 0].axhline(y=np.mean(self.position_error_magnitude), color='r', linestyle='--', 
                          label=f'Mean: {np.mean(self.position_error_magnitude):.3f}m')
        axes[0, 0].axhline(y=np.std(self.position_error_magnitude), color='g', linestyle='--', 
                          label=f'Std: {np.std(self.position_error_magnitude):.3f}m')
        axes[0, 0].set_title('Position Error Magnitude')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Error (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Error distribution
        axes[0, 1].hist(self.position_error_magnitude, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
        axes[0, 1].set_title('Position Error Distribution')
        axes[0, 1].set_xlabel('Error Magnitude (m)')
        axes[0, 1].set_ylabel('Frequency')
        axes[0, 1].grid(True)
        
        # Cumulative error
        cumulative_error = np.cumsum(self.position_error_magnitude)
        axes[1, 0].plot(time_axis, cumulative_error, 'purple', linewidth=2)
        axes[1, 0].set_title('Cumulative Position Error')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Cumulative Error (m)')
        axes[1, 0].grid(True)
        
        # Error rate (derivative of error)
        if len(self.position_error_magnitude) > 1:
            error_rate = np.diff(self.position_error_magnitude) / np.diff(time_axis)
            time_rate = time_axis[1:]
            axes[1, 1].plot(time_rate, error_rate, 'orange', linewidth=2)
            axes[1, 1].set_title('Position Error Rate')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Error Rate (m/s)')
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_performance.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def generate_summary_report(self):
        """Generate a summary report of the flight analysis"""
        report = f"""
        AUTONOMOUS FLIGHT ANALYSIS REPORT
        =================================
        
        Flight Duration: {self.timestamps[-1] - self.timestamps[0]:.2f} seconds
        Total Waypoints: {len(self.waypoints)}
        Data Points: {len(self.positions)}
        
        POSITION ACCURACY:
        - Mean Position Error: {np.mean(self.position_error_magnitude):.3f} m
        - Std Position Error: {np.std(self.position_error_magnitude):.3f} m
        - Max Position Error: {np.max(self.position_error_magnitude):.3f} m
        - Min Position Error: {np.min(self.position_error_magnitude):.3f} m
        
        VELOCITY ANALYSIS:
        - Mean Speed: {np.mean(np.linalg.norm(self.velocities, axis=1)):.3f} m/s
        - Max Speed: {np.max(np.linalg.norm(self.velocities, axis=1)):.3f} m/s
        - Mean Acceleration: {np.mean(np.linalg.norm(self.accelerations, axis=1)):.3f} m/s²
        
        TRAJECTORY SMOOTHNESS:
        - Mean Curvature: {np.mean(self.trajectory_curvature):.6f} 1/m
        - Max Curvature: {np.max(self.trajectory_curvature):.6f} 1/m
        
        ORIENTATION STABILITY:
        - Quaternion Magnitude Mean: {np.mean(np.linalg.norm(self.orientations, axis=1)):.6f}
        - Quaternion Magnitude Std: {np.std(np.linalg.norm(self.orientations, axis=1)):.6f}
        
        EKF PERFORMANCE:
        - Position Estimation: {'GOOD' if np.mean(self.position_error_magnitude) < 0.5 else 'NEEDS IMPROVEMENT'}
        - Trajectory Tracking: {'SMOOTH' if np.mean(self.trajectory_curvature) < 0.1 else 'JERKY'}
        - Overall Performance: {'EXCELLENT' if np.mean(self.position_error_magnitude) < 0.3 else 'GOOD' if np.mean(self.position_error_magnitude) < 0.8 else 'FAIR'}
        """
        
        print(report)
        
        # Save report to file
        with open('/u/12/akyuret1/unix/drone_sim/flight_analysis_report.txt', 'w') as f:
            f.write(report)
    
    def run_complete_analysis(self):
        """Run complete EKF analysis and generate all plots"""
        print("Starting complete EKF analysis...")
        
        self.plot_3d_trajectory()
        self.plot_position_errors()
        self.plot_velocity_analysis()
        self.plot_orientation_analysis()
        self.plot_ekf_performance_metrics()
        self.generate_summary_report()
        
        print("Analysis complete! All plots saved to /u/12/akyuret1/unix/drone_sim/")

def main():
    # Find the most recent flight data file
    data_files = glob.glob('/u/12/akyuret1/unix/drone_sim/flight_data_*.json')
    
    if not data_files:
        print("No flight data files found. Please run the autonomous flight controller first.")
        return
    
    # Use the most recent file
    latest_file = max(data_files, key=os.path.getctime)
    print(f"Analyzing flight data from: {latest_file}")
    
    # Create analyzer and run analysis
    analyzer = EKFAnalyzer(latest_file)
    analyzer.run_complete_analysis()

if __name__ == '__main__':
    main()
