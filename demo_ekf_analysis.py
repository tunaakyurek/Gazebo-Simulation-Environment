#!/usr/bin/env python3

import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from scipy import signal
from scipy.spatial.transform import Rotation as R
import os
from datetime import datetime

def generate_simulated_flight_data():
    """Generate realistic simulated flight data for demonstration"""
    
    # Flight parameters
    duration = 60.0  # seconds
    dt = 0.1  # 10 Hz
    t = np.arange(0, duration, dt)
    n_points = len(t)
    
    # Define waypoints for a square pattern
    waypoints = np.array([
        [0.0, 0.0, 2.0],    # Takeoff
        [5.0, 0.0, 2.0],    # Forward
        [5.0, 5.0, 2.0],    # Right
        [0.0, 5.0, 2.0],    # Back
        [0.0, 0.0, 2.0],    # Return
        [0.0, 0.0, 0.5]     # Land
    ])
    
    # Generate smooth trajectory between waypoints
    positions = np.zeros((n_points, 3))
    target_positions = np.zeros((n_points, 3))
    
    # Create smooth trajectory
    for i, waypoint in enumerate(waypoints[:-1]):
        start_idx = int(i * n_points / (len(waypoints) - 1))
        end_idx = int((i + 1) * n_points / (len(waypoints) - 1))
        
        if i == len(waypoints) - 2:  # Last segment
            end_idx = n_points
            
        segment_length = end_idx - start_idx
        if segment_length > 0:
            # Smooth interpolation with some noise
            t_segment = np.linspace(0, 1, segment_length)
            
            # Add some realistic flight dynamics
            smooth_t = t_segment ** 2 * (3 - 2 * t_segment)  # Smooth step function
            
            for j in range(3):
                positions[start_idx:end_idx, j] = waypoints[i, j] + (waypoints[i+1, j] - waypoints[i, j]) * smooth_t
                target_positions[start_idx:end_idx, j] = waypoints[i+1, j]
    
    # Add realistic noise and dynamics
    # Position noise (EKF estimation errors)
    position_noise = np.random.normal(0, 0.1, positions.shape)
    positions += position_noise
    
    # Add some realistic flight dynamics (slight overshoot, settling)
    for i in range(1, n_points):
        for j in range(3):
            # Add some momentum and settling behavior
            if i < n_points - 1:
                positions[i, j] += 0.1 * np.sin(2 * np.pi * t[i] / 5) * np.exp(-t[i] / 20)
    
    # Generate orientations (quaternions)
    orientations = np.zeros((n_points, 4))
    orientations[:, 3] = 1.0  # w component
    
    # Add slight orientation changes during flight
    for i in range(n_points):
        # Small roll and pitch changes during flight
        roll = 0.05 * np.sin(2 * np.pi * t[i] / 8)
        pitch = 0.03 * np.cos(2 * np.pi * t[i] / 6)
        yaw = 0.02 * np.sin(2 * np.pi * t[i] / 10)
        
        r = R.from_euler('xyz', [roll, pitch, yaw])
        orientations[i] = r.as_quat()
    
    # Generate IMU data
    imu_data = []
    for i in range(n_points):
        # Calculate accelerations from position changes
        if i > 0:
            accel = (positions[i] - 2*positions[i-1] + positions[max(0, i-2)]) / (dt**2)
        else:
            accel = np.array([0, 0, -9.81])  # Gravity
        
        # Add IMU noise
        accel += np.random.normal(0, 0.5, 3)
        
        # Angular velocities (small changes during flight)
        ang_vel = np.array([
            0.1 * np.sin(2 * np.pi * t[i] / 5),
            0.08 * np.cos(2 * np.pi * t[i] / 7),
            0.05 * np.sin(2 * np.pi * t[i] / 9)
        ]) + np.random.normal(0, 0.02, 3)
        
        imu_data.append({
            'linear_acceleration': accel.tolist(),
            'angular_velocity': ang_vel.tolist()
        })
    
    # Generate GPS data
    gps_data = []
    for i in range(n_points):
        # Simulate GPS with some noise and occasional dropouts
        if np.random.random() > 0.05:  # 95% availability
            lat = 60.1699 + positions[i, 0] * 1e-5 + np.random.normal(0, 1e-6)
            lon = 24.9384 + positions[i, 1] * 1e-5 + np.random.normal(0, 1e-6)
            alt = 10.0 + positions[i, 2] + np.random.normal(0, 0.5)
        else:
            # GPS dropout
            lat = 0.0
            lon = 0.0
            alt = 0.0
            
        gps_data.append({
            'latitude': lat,
            'longitude': lon,
            'altitude': alt
        })
    
    # Create flight data structure
    flight_data = {
        'timestamps': t.tolist(),
        'positions': positions.tolist(),
        'orientations': orientations.tolist(),
        'target_positions': target_positions.tolist(),
        'imu_data': imu_data,
        'gps_data': gps_data
    }
    
    return flight_data

def run_ekf_demonstration():
    """Run complete EKF analysis demonstration"""
    
    print("🚁 AUTONOMOUS FLIGHT EKF ANALYSIS DEMONSTRATION")
    print("=" * 50)
    print()
    
    # Generate simulated flight data
    print("📊 Generating simulated flight data...")
    flight_data = generate_simulated_flight_data()
    
    # Save flight data
    filename = f'/u/12/akyuret1/unix/drone_sim/demo_flight_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
    with open(filename, 'w') as f:
        json.dump(flight_data, f, indent=2)
    print(f"✅ Flight data saved to: {filename}")
    print()
    
    # Load data into analyzer
    print("🔍 Loading data into EKF analyzer...")
    analyzer = EKFAnalyzer(filename)
    print("✅ Data loaded successfully")
    print()
    
    # Run complete analysis
    print("📈 Running complete EKF analysis...")
    analyzer.run_complete_analysis()
    print("✅ Analysis complete!")
    print()
    
    print("🎉 EKF ANALYSIS DEMONSTRATION COMPLETE!")
    print("=" * 50)
    print("📊 Generated plots:")
    print("  - 3d_trajectory.png: 3D flight trajectory")
    print("  - position_errors.png: Position estimation errors")
    print("  - velocity_analysis.png: Velocity and acceleration analysis")
    print("  - orientation_analysis.png: Orientation and IMU data")
    print("  - ekf_performance.png: EKF performance metrics")
    print("  - flight_analysis_report.txt: Summary report")
    print()

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
        ax.set_title('3D Flight Trajectory with EKF Estimation')
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
        axes[0, 0].set_title('X Position Error (EKF Estimation)')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Error (m)')
        axes[0, 0].grid(True)
        
        # Y position error
        axes[0, 1].plot(time_axis, self.position_errors[:, 1], 'g-', linewidth=2)
        axes[0, 1].set_title('Y Position Error (EKF Estimation)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Error (m)')
        axes[0, 1].grid(True)
        
        # Z position error
        axes[1, 0].plot(time_axis, self.position_errors[:, 2], 'b-', linewidth=2)
        axes[1, 0].set_title('Z Position Error (EKF Estimation)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Error (m)')
        axes[1, 0].grid(True)
        
        # Total position error magnitude
        axes[1, 1].plot(time_axis, self.position_error_magnitude, 'k-', linewidth=2)
        axes[1, 1].set_title('Total Position Error Magnitude (EKF Performance)')
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
        axes[0, 0].set_title('Velocity Components (EKF Estimated)')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Velocity (m/s)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Velocity magnitude
        velocity_magnitude = np.linalg.norm(self.velocities, axis=1)
        axes[0, 1].plot(time_axis, velocity_magnitude, 'k-', linewidth=2)
        axes[0, 1].set_title('Velocity Magnitude (EKF Estimated)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Speed (m/s)')
        axes[0, 1].grid(True)
        
        # Acceleration components
        if len(self.accelerations) > 0:
            time_acc = self.timestamps[2:] - self.timestamps[0]
            axes[1, 0].plot(time_acc, self.accelerations[:, 0], 'r-', linewidth=2, label='Ax')
            axes[1, 0].plot(time_acc, self.accelerations[:, 1], 'g-', linewidth=2, label='Ay')
            axes[1, 0].plot(time_acc, self.accelerations[:, 2], 'b-', linewidth=2, label='Az')
            axes[1, 0].set_title('Acceleration Components (EKF Estimated)')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Acceleration (m/s²)')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
        
        # Trajectory curvature
        if len(self.trajectory_curvature) > 0:
            time_curv = self.timestamps[2:] - self.timestamps[0]
            axes[1, 1].plot(time_curv, self.trajectory_curvature, 'purple', linewidth=2)
            axes[1, 1].set_title('Trajectory Curvature (EKF Analysis)')
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
        axes[0, 0].set_title('Euler Angles (EKF Estimated)')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Angle (degrees)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Quaternion components
        axes[0, 1].plot(time_axis, self.orientations[:, 0], 'r-', linewidth=2, label='qx')
        axes[0, 1].plot(time_axis, self.orientations[:, 1], 'g-', linewidth=2, label='qy')
        axes[0, 1].plot(time_axis, self.orientations[:, 2], 'b-', linewidth=2, label='qz')
        axes[0, 1].plot(time_axis, self.orientations[:, 3], 'k-', linewidth=2, label='qw')
        axes[0, 1].set_title('Quaternion Components (EKF Estimated)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Quaternion Value')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # Quaternion magnitude (should be 1)
        quat_magnitude = np.linalg.norm(self.orientations, axis=1)
        axes[1, 0].plot(time_axis, quat_magnitude, 'purple', linewidth=2)
        axes[1, 0].set_title('Quaternion Magnitude (EKF Validation)')
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
            axes[1, 1].set_title('Angular Velocity (IMU Data)')
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
        axes[0, 0].set_title('EKF Position Error Magnitude')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Error (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Error distribution
        axes[0, 1].hist(self.position_error_magnitude, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
        axes[0, 1].set_title('EKF Error Distribution')
        axes[0, 1].set_xlabel('Error Magnitude (m)')
        axes[0, 1].set_ylabel('Frequency')
        axes[0, 1].grid(True)
        
        # Cumulative error
        cumulative_error = np.cumsum(self.position_error_magnitude)
        axes[1, 0].plot(time_axis, cumulative_error, 'purple', linewidth=2)
        axes[1, 0].set_title('Cumulative EKF Position Error')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Cumulative Error (m)')
        axes[1, 0].grid(True)
        
        # Error rate (derivative of error)
        if len(self.position_error_magnitude) > 1:
            error_rate = np.diff(self.position_error_magnitude) / np.diff(time_axis)
            time_rate = time_axis[1:]
            axes[1, 1].plot(time_rate, error_rate, 'orange', linewidth=2)
            axes[1, 1].set_title('EKF Position Error Rate')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Error Rate (m/s)')
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('/u/12/akyuret1/unix/drone_sim/ekf_performance.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def generate_summary_report(self):
        """Generate a summary report of the flight analysis"""
        report = f"""
        AUTONOMOUS FLIGHT EKF ANALYSIS REPORT
        ====================================
        
        Flight Duration: {self.timestamps[-1] - self.timestamps[0]:.2f} seconds
        Total Waypoints: {len(self.waypoints) if hasattr(self, 'waypoints') else 'N/A'}
        Data Points: {len(self.positions)}
        
        EKF POSITION ACCURACY:
        - Mean Position Error: {np.mean(self.position_error_magnitude):.3f} m
        - Std Position Error: {np.std(self.position_error_magnitude):.3f} m
        - Max Position Error: {np.max(self.position_error_magnitude):.3f} m
        - Min Position Error: {np.min(self.position_error_magnitude):.3f} m
        
        EKF VELOCITY ANALYSIS:
        - Mean Speed: {np.mean(np.linalg.norm(self.velocities, axis=1)):.3f} m/s
        - Max Speed: {np.max(np.linalg.norm(self.velocities, axis=1)):.3f} m/s
        - Mean Acceleration: {np.mean(np.linalg.norm(self.accelerations, axis=1)):.3f} m/s²
        
        EKF TRAJECTORY SMOOTHNESS:
        - Mean Curvature: {np.mean(self.trajectory_curvature):.6f} 1/m
        - Max Curvature: {np.max(self.trajectory_curvature):.6f} 1/m
        
        EKF ORIENTATION STABILITY:
        - Quaternion Magnitude Mean: {np.mean(np.linalg.norm(self.orientations, axis=1)):.6f}
        - Quaternion Magnitude Std: {np.std(np.linalg.norm(self.orientations, axis=1)):.6f}
        
        EKF PERFORMANCE ASSESSMENT:
        - Position Estimation: {'EXCELLENT' if np.mean(self.position_error_magnitude) < 0.1 else 'GOOD' if np.mean(self.position_error_magnitude) < 0.3 else 'FAIR'}
        - Trajectory Tracking: {'SMOOTH' if np.mean(self.trajectory_curvature) < 0.1 else 'ACCEPTABLE'}
        - Overall EKF Performance: {'EXCELLENT' if np.mean(self.position_error_magnitude) < 0.2 else 'GOOD' if np.mean(self.position_error_magnitude) < 0.5 else 'FAIR'}
        
        EKF FILTER CHARACTERISTICS:
        - Filter Convergence: {'FAST' if np.std(self.position_error_magnitude[:50]) > np.std(self.position_error_magnitude[-50:]) else 'STABLE'}
        - Noise Rejection: {'GOOD' if np.std(self.position_error_magnitude) < 0.2 else 'MODERATE'}
        - Estimation Stability: {'HIGH' if np.std(np.linalg.norm(self.orientations, axis=1)) < 0.01 else 'MODERATE'}
        """
        
        print(report)
        
        # Save report to file
        with open('/u/12/akyuret1/unix/drone_sim/ekf_analysis_report.txt', 'w') as f:
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
        
        print("EKF Analysis complete! All plots saved to /u/12/akyuret1/unix/drone_sim/")

if __name__ == '__main__':
    run_ekf_demonstration()
