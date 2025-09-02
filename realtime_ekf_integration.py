#!/usr/bin/env python3
"""
Real-time EKF Integration with Gazebo Drone Simulation
Processes simulation data and runs our EKF algorithm
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import json
import time
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import queue

# Import our EKF modules
from ekf_core import ExtendedKalmanFilter
from ekf_parameters import EKFParameters
from ekf_sensor_model import SensorModel

# ROS 2 message types
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray

class RealTimeEKFIntegration(Node):
    """
    Real-time EKF integration that processes Gazebo simulation data
    and runs our custom EKF algorithm
    """
    
    def __init__(self):
        super().__init__('realtime_ekf_integration')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize EKF
        self.ekf_params = EKFParameters()
        self.sensor_model = SensorModel(self.ekf_params)
        self.ekf = ExtendedKalmanFilter(self.ekf_params)
        
        # Data storage
        self.true_states = []  # Ground truth from simulation
        self.ekf_states = []   # EKF estimates
        self.measurements = [] # Raw sensor measurements
        self.timestamps = []
        
        # Data queues for thread-safe communication
        self.imu_queue = queue.Queue()
        self.gps_queue = queue.Queue()
        self.pose_queue = queue.Queue()
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/sim/imu_raw',
            self.imu_callback,
            qos_profile
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/sim/gps',
            self.gps_callback,
            qos_profile
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/autonomous_drone/pose',
            self.pose_callback,
            qos_profile
        )
        
        # Publishers for EKF results
        self.ekf_pose_pub = self.create_publisher(
            PoseStamped,
            '/ekf/pose',
            qos_profile
        )
        
        self.ekf_velocity_pub = self.create_publisher(
            TwistStamped,
            '/ekf/velocity',
            qos_profile
        )
        
        # EKF processing timer
        self.ekf_timer = self.create_timer(0.01, self.ekf_processing_loop)  # 100 Hz
        
        # Visualization timer
        self.viz_timer = self.create_timer(0.1, self.update_visualization)  # 10 Hz
        
        # Data logging
        self.ekf_data = []
        self.log_file = f"realtime_ekf_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        # Visualization setup
        self.setup_visualization()
        
        self.get_logger().info("Real-time EKF Integration initialized")
        self.get_logger().info("EKF parameters loaded and ready")
        
    def setup_visualization(self):
        """Setup real-time visualization"""
        plt.ion()  # Interactive mode
        
        # Create figure with subplots
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('Real-time EKF Analysis', fontsize=16)
        
        # Initialize plots
        self.ax_pos = self.axes[0, 0]
        self.ax_vel = self.axes[0, 1]
        self.ax_att = self.axes[1, 0]
        
        # Position plot
        self.ax_pos.set_title('Position Estimation')
        self.ax_pos.set_xlabel('Time (s)')
        self.ax_pos.set_ylabel('Position (m)')
        self.ax_pos.grid(True)
        
        # Velocity plot
        self.ax_vel.set_title('Velocity Estimation')
        self.ax_vel.set_xlabel('Time (s)')
        self.ax_vel.set_ylabel('Velocity (m/s)')
        self.ax_vel.grid(True)
        
        # Attitude plot
        self.ax_att.set_title('Attitude Estimation')
        self.ax_att.set_xlabel('Time (s)')
        self.ax_att.set_ylabel('Angle (rad)')
        self.ax_att.grid(True)
        
        # 3D trajectory plot
        self.ax_3d = self.fig.add_subplot(2, 2, 4, projection='3d')
        self.ax_3d.set_title('3D Trajectory')
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        
        # Initialize plot lines
        self.pos_lines = {}
        self.vel_lines = {}
        self.att_lines = {}
        
        colors = ['red', 'blue', 'green']
        labels = ['X', 'Y', 'Z']
        
        for i, (color, label) in enumerate(zip(colors, labels)):
            self.pos_lines[label] = self.ax_pos.plot([], [], color=color, label=f'{label} (True)')[0]
            self.vel_lines[label] = self.ax_vel.plot([], [], color=color, label=f'{label} (True)')[0]
            self.att_lines[label] = self.ax_att.plot([], [], color=color, label=f'{label} (True)')[0]
        
        # EKF estimate lines
        for i, (color, label) in enumerate(zip(colors, labels)):
            self.ax_pos.plot([], [], color=color, linestyle='--', alpha=0.7, label=f'{label} (EKF)')
            self.ax_vel.plot([], [], color=color, linestyle='--', alpha=0.7, label=f'{label} (EKF)')
            self.ax_att.plot([], [], color=color, linestyle='--', alpha=0.7, label=f'{label} (EKF)')
        
        self.ax_pos.legend()
        self.ax_vel.legend()
        self.ax_att.legend()
        
        # 3D trajectory
        self.traj_true = self.ax_3d.plot([], [], [], 'b-', label='True Trajectory')[0]
        self.traj_ekf = self.ax_3d.plot([], [], [], 'r--', label='EKF Estimate')[0]
        self.ax_3d.legend()
        
        plt.tight_layout()
        plt.show(block=False)
        
    def imu_callback(self, msg: Imu):
        """Handle IMU data"""
        imu_data = {
            'timestamp': time.time(),
            'linear_acceleration': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ],
            'angular_velocity': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'orientation': [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
        }
        
        try:
            self.imu_queue.put_nowait(imu_data)
        except queue.Full:
            pass  # Skip if queue is full
    
    def gps_callback(self, msg: NavSatFix):
        """Handle GPS data"""
        gps_data = {
            'timestamp': time.time(),
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': list(msg.position_covariance)
        }
        
        try:
            self.gps_queue.put_nowait(gps_data)
        except queue.Full:
            pass
    
    def pose_callback(self, msg: PoseStamped):
        """Handle ground truth pose data"""
        pose_data = {
            'timestamp': time.time(),
            'position': [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ],
            'orientation': [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
        }
        
        try:
            self.pose_queue.put_nowait(pose_data)
        except queue.Full:
            pass
    
    def ekf_processing_loop(self):
        """Main EKF processing loop"""
        current_time = time.time()
        
        # Process IMU data
        while not self.imu_queue.empty():
            try:
                imu_data = self.imu_queue.get_nowait()
                self.process_imu_data(imu_data)
            except queue.Empty:
                break
        
        # Process GPS data
        while not self.gps_queue.empty():
            try:
                gps_data = self.gps_queue.get_nowait()
                self.process_gps_data(gps_data)
            except queue.Empty:
                break
        
        # Process ground truth pose
        while not self.pose_queue.empty():
            try:
                pose_data = self.pose_queue.get_nowait()
                self.process_ground_truth(pose_data)
            except queue.Empty:
                break
        
        # Run EKF prediction step
        if len(self.timestamps) > 0:
            dt = 0.01  # 100 Hz
            self.ekf.predict(dt)
            
            # Get current state estimate
            state = self.ekf.get_state()
            self.ekf_states.append(state.copy())
            
            # Publish EKF results
            self.publish_ekf_results(state)
            
            # Log data
            self.log_ekf_data(current_time, state)
    
    def process_imu_data(self, imu_data: dict):
        """Process IMU measurement"""
        # Convert to numpy arrays
        accel = np.array(imu_data['linear_acceleration'])
        gyro = np.array(imu_data['angular_velocity'])
        
        # Create measurement vector
        measurement = np.concatenate([accel, gyro])
        
        # Add to EKF
        self.ekf.update_imu(measurement, imu_data['timestamp'])
        
        # Store measurement
        self.measurements.append({
            'type': 'imu',
            'timestamp': imu_data['timestamp'],
            'data': measurement.tolist()
        })
    
    def process_gps_data(self, gps_data: dict):
        """Process GPS measurement"""
        # Convert lat/lon to local coordinates (simplified)
        # In a real implementation, you would use proper coordinate transformation
        lat, lon, alt = gps_data['latitude'], gps_data['longitude'], gps_data['altitude']
        
        # Simple conversion (not accurate, just for demonstration)
        x = (lon - 24.0) * 111320 * np.cos(np.radians(lat))
        y = (lat - 60.0) * 111320
        z = alt
        
        position = np.array([x, y, z])
        
        # Create measurement vector
        measurement = position
        
        # Add to EKF
        self.ekf.update_gps(measurement, gps_data['timestamp'])
        
        # Store measurement
        self.measurements.append({
            'type': 'gps',
            'timestamp': gps_data['timestamp'],
            'data': measurement.tolist()
        })
    
    def process_ground_truth(self, pose_data: dict):
        """Process ground truth pose for comparison"""
        position = np.array(pose_data['position'])
        
        # Convert quaternion to Euler angles (simplified)
        qx, qy, qz, qw = pose_data['orientation']
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        orientation = np.array([roll, pitch, yaw])
        
        # Store ground truth
        true_state = np.concatenate([position, np.zeros(3), orientation, np.zeros(3)])
        self.true_states.append(true_state)
        self.timestamps.append(pose_data['timestamp'])
    
    def publish_ekf_results(self, state: np.ndarray):
        """Publish EKF results"""
        # Extract position and velocity
        position = state[:3]
        velocity = state[3:6]
        orientation = state[6:9]
        
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        
        # Convert Euler to quaternion
        roll, pitch, yaw = orientation
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        self.ekf_pose_pub.publish(pose_msg)
        
        # Publish velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "world"
        
        vel_msg.twist.linear.x = float(velocity[0])
        vel_msg.twist.linear.y = float(velocity[1])
        vel_msg.twist.linear.z = float(velocity[2])
        
        vel_msg.twist.angular.x = float(state[9])
        vel_msg.twist.angular.y = float(state[10])
        vel_msg.twist.angular.z = float(state[11])
        
        self.ekf_velocity_pub.publish(vel_msg)
    
    def log_ekf_data(self, timestamp: float, state: np.ndarray):
        """Log EKF data"""
        data_point = {
            'timestamp': timestamp,
            'ekf_state': state.tolist(),
            'ekf_covariance': self.ekf.get_covariance().tolist()
        }
        self.ekf_data.append(data_point)
        
        # Save data periodically
        if len(self.ekf_data) % 100 == 0:
            with open(self.log_file, 'w') as f:
                json.dump(self.ekf_data, f, indent=2)
    
    def update_visualization(self):
        """Update real-time visualization"""
        if len(self.timestamps) < 2:
            return
        
        # Convert timestamps to relative time
        start_time = self.timestamps[0]
        times = [(t - start_time) for t in self.timestamps]
        
        # Limit data for visualization
        max_points = 1000
        if len(times) > max_points:
            times = times[-max_points:]
            true_states = self.true_states[-max_points:]
            ekf_states = self.ekf_states[-max_points:]
        else:
            true_states = self.true_states
            ekf_states = self.ekf_states
        
        # Update position plot
        if len(true_states) > 0 and len(ekf_states) > 0:
            # Position
            true_pos = np.array(true_states)[:, :3]
            ekf_pos = np.array(ekf_states)[:, :3]
            
            for i, label in enumerate(['X', 'Y', 'Z']):
                self.pos_lines[label].set_data(times, true_pos[:, i])
            
            # Velocity
            true_vel = np.array(true_states)[:, 3:6]
            ekf_vel = np.array(ekf_states)[:, 3:6]
            
            for i, label in enumerate(['X', 'Y', 'Z']):
                self.vel_lines[label].set_data(times, true_vel[:, i])
            
            # Attitude
            true_att = np.array(true_states)[:, 6:9]
            ekf_att = np.array(ekf_states)[:, 6:9]
            
            for i, label in enumerate(['X', 'Y', 'Z']):
                self.att_lines[label].set_data(times, true_att[:, i])
            
            # 3D trajectory
            self.traj_true.set_data_3d(true_pos[:, 0], true_pos[:, 1], true_pos[:, 2])
            self.traj_ekf.set_data_3d(ekf_pos[:, 0], ekf_pos[:, 1], ekf_pos[:, 2])
            
            # Update plot limits
            self.ax_pos.relim()
            self.ax_pos.autoscale_view()
            self.ax_vel.relim()
            self.ax_vel.autoscale_view()
            self.ax_att.relim()
            self.ax_att.autoscale_view()
            
            # Update 3D plot limits
            if len(true_pos) > 0:
                self.ax_3d.set_xlim(true_pos[:, 0].min()-1, true_pos[:, 0].max()+1)
                self.ax_3d.set_ylim(true_pos[:, 1].min()-1, true_pos[:, 1].max()+1)
                self.ax_3d.set_zlim(true_pos[:, 2].min()-1, true_pos[:, 2].max()+1)
            
            # Refresh plot
            plt.draw()
            plt.pause(0.001)
    
    def save_data(self):
        """Save all data"""
        # Save EKF data
        with open(self.log_file, 'w') as f:
            json.dump(self.ekf_data, f, indent=2)
        
        # Save complete analysis
        analysis_data = {
            'timestamps': self.timestamps,
            'true_states': [state.tolist() for state in self.true_states],
            'ekf_states': [state.tolist() for state in self.ekf_states],
            'measurements': self.measurements,
            'ekf_parameters': {
                'Q_scale': self.ekf_params.Q_scale,
                'R_scale': self.ekf_params.R_scale,
                'initial_covariance': self.ekf_params.P0.tolist()
            }
        }
        
        analysis_file = f"complete_ekf_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(analysis_file, 'w') as f:
            json.dump(analysis_data, f, indent=2)
        
        self.get_logger().info(f"Data saved to {self.log_file} and {analysis_file}")
        
        # Generate final plots
        self.generate_final_plots()

def main(args=None):
    rclpy.init(args=args)
    
    ekf_integration = RealTimeEKFIntegration()
    
    try:
        rclpy.spin(ekf_integration)
    except KeyboardInterrupt:
        ekf_integration.get_logger().info("Shutting down real-time EKF integration")
        ekf_integration.save_data()
    finally:
        ekf_integration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
