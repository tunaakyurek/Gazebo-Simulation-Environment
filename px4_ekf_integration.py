#!/usr/bin/env python3
"""
Real-time EKF Integration with PX4 Autonomous Flight System
This processes the ACTUAL autonomous flight data from PX4/Gazebo
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import json
from datetime import datetime
from collections import deque

# Import our EKF modules
from ekf_core import ExtendedKalmanFilter
from ekf_parameters import EKFParameters
from ekf_sensor_model import SensorModel

# ROS 2 message types for PX4/MAVROS
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget, PositionTarget
from std_msgs.msg import Header

class PX4EKFIntegration(Node):
    """Real-time EKF integration with PX4 autonomous flight system"""
    
    def __init__(self):
        super().__init__('px4_ekf_integration')
        
        # QoS profiles
        # Use sensor data QoS for MAVROS subscriptions (BEST_EFFORT, low latency)
        sensor_qos: QoSProfile = qos_profile_sensor_data
        # Use reliable QoS for EKF publishers
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize EKF
        self.ekf_params = EKFParameters()
        self.ekf = ExtendedKalmanFilter(self.ekf_params)
        self.sensor_model = SensorModel(self.ekf_params)
        
        # Data storage
        self.max_points = 1000
        self.true_positions = deque(maxlen=self.max_points)
        self.true_velocities = deque(maxlen=self.max_points)
        self.true_attitudes = deque(maxlen=self.max_points)
        self.ekf_positions = deque(maxlen=self.max_points)
        self.ekf_velocities = deque(maxlen=self.max_points)
        self.ekf_attitudes = deque(maxlen=self.max_points)
        self.timestamps = deque(maxlen=self.max_points)
        self.imu_data = deque(maxlen=self.max_points)
        self.gps_data = deque(maxlen=self.max_points)
        
        # PX4/MAVROS subscribers
        self.true_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.true_pose_callback,
            sensor_qos
        )
        
        self.true_velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.true_velocity_callback,
            sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/gps/fix',
            self.gps_callback,
            sensor_qos
        )
        
        self.px4_state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.px4_state_callback,
            sensor_qos
        )
        
        # EKF publishers
        self.ekf_pose_pub = self.create_publisher(
            PoseStamped,
            '/ekf/pose',
            pub_qos
        )
        
        self.ekf_velocity_pub = self.create_publisher(
            TwistStamped,
            '/ekf/velocity',
            pub_qos
        )
        
        # Data logging
        self.log_data = []
        self.start_time = time.time()
        
        # Setup visualization
        self.setup_visualization()
        
        # Timer for EKF updates
        self.create_timer(0.01, self.ekf_update_loop)  # 100 Hz
        # Periodic timer to persist logs regularly even if run is short
        self.create_timer(5.0, self.save_data)
        
        self.get_logger().info("PX4 EKF Integration initialized")
        self.get_logger().info("Processing REAL PX4 autonomous flight data")
        
    def setup_visualization(self):
        """Setup real-time visualization"""
        plt.ion()
        
        # Create figure with subplots
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('PX4 Autonomous Flight + EKF Analysis', fontsize=16)
        
        # Initialize plots
        self.ax_pos = self.axes[0, 0]
        self.ax_vel = self.axes[0, 1]
        self.ax_att = self.axes[1, 0]
        self.ax_3d = self.fig.add_subplot(2, 2, 4, projection='3d')
        
        # Position plot
        self.ax_pos.set_title('Position: True vs EKF')
        self.ax_pos.set_xlabel('Time (s)')
        self.ax_pos.set_ylabel('Position (m)')
        self.ax_pos.grid(True)
        
        # Velocity plot
        self.ax_vel.set_title('Velocity: True vs EKF')
        self.ax_vel.set_xlabel('Time (s)')
        self.ax_vel.set_ylabel('Velocity (m/s)')
        self.ax_vel.grid(True)
        
        # Attitude plot
        self.ax_att.set_title('Attitude: True vs EKF')
        self.ax_att.set_xlabel('Time (s)')
        self.ax_att.set_ylabel('Angle (rad)')
        self.ax_att.grid(True)
        
        # 3D trajectory plot
        self.ax_3d.set_title('3D Flight Trajectory')
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        
        # Initialize plot lines
        self.pos_lines = {}
        self.vel_lines = {}
        self.att_lines = {}
        
        colors = ['red', 'green', 'blue']
        labels = ['X', 'Y', 'Z']
        
        for i, (color, label) in enumerate(zip(colors, labels)):
            self.pos_lines[f'true_{label}'] = self.ax_pos.plot([], [], color=color, linestyle='-', label=f'True {label}')[0]
            self.pos_lines[f'ekf_{label}'] = self.ax_pos.plot([], [], color=color, linestyle='--', alpha=0.7)[0]
            
            self.vel_lines[f'true_{label}'] = self.ax_vel.plot([], [], color=color, linestyle='-', label=f'True {label}')[0]
            self.vel_lines[f'ekf_{label}'] = self.ax_vel.plot([], [], color=color, linestyle='--', alpha=0.7)[0]
            
            self.att_lines[f'true_{label}'] = self.ax_att.plot([], [], color=color, linestyle='-', label=f'True {label}')[0]
            self.att_lines[f'ekf_{label}'] = self.ax_att.plot([], [], color=color, linestyle='--', alpha=0.7)[0]
        
        self.ax_pos.legend()
        self.ax_vel.legend()
        self.ax_att.legend()
        
        # 3D trajectory lines
        self.traj_true = self.ax_3d.plot([], [], [], 'b-', linewidth=2, label='True Trajectory')[0]
        self.traj_ekf = self.ax_3d.plot([], [], [], 'r--', linewidth=2, label='EKF Estimate')[0]
        self.ax_3d.legend()
        
        plt.tight_layout()
        plt.show(block=False)
        
    def true_pose_callback(self, msg):
        """Handle true pose from PX4"""
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        
        # Convert quaternion to Euler angles
        euler = self.quaternion_to_euler(orientation)
        
        self.true_positions.append(position)
        self.true_attitudes.append(euler)
        self.timestamps.append(time.time())
        
    def true_velocity_callback(self, msg):
        """Handle true velocity from PX4"""
        velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        self.true_velocities.append(velocity)
        
    def imu_callback(self, msg):
        """Handle IMU data from PX4"""
        imu = {
            'accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'gyro': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'timestamp': time.time()
        }
        self.imu_data.append(imu)
        
    def gps_callback(self, msg):
        """Handle GPS data from PX4"""
        gps = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'timestamp': time.time()
        }
        self.gps_data.append(gps)
        
    def px4_state_callback(self, msg):
        """Handle PX4 state information"""
        if len(self.timestamps) % 100 == 0:  # Log every 100 messages
            self.get_logger().info(
                f"PX4 State: Mode={msg.mode}, Armed={msg.armed}, "
                f"Connected={msg.connected}, Guided={msg.guided}"
            )
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles"""
        x, y, z, w = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
    
    def ekf_update_loop(self):
        """Main EKF update loop"""
        if len(self.imu_data) < 2:
            return
        
        # Get latest IMU data
        latest_imu = self.imu_data[-1]
        
        # Run EKF prediction step
        dt = 0.01  # 100 Hz
        self.ekf.predict(dt)
        
        # Run EKF update step with IMU
        if len(self.imu_data) > 0:
            imu_measurement = np.array(latest_imu['accel'] + latest_imu['gyro'])
            self.ekf.update_imu(imu_measurement, dt)
        
        # Get EKF state estimate
        ekf_state = self.ekf.get_state()
        
        # Store EKF estimates
        self.ekf_positions.append(ekf_state[:3].tolist())
        self.ekf_velocities.append(ekf_state[3:6].tolist())
        self.ekf_attitudes.append(ekf_state[6:9].tolist())
        
        # Publish EKF estimates
        self.publish_ekf_estimates(ekf_state)
        
        # Log data
        self.log_data.append({
            'timestamp': time.time(),
            'true_position': self.true_positions[-1] if self.true_positions else None,
            'true_velocity': self.true_velocities[-1] if self.true_velocities else None,
            'true_attitude': self.true_attitudes[-1] if self.true_attitudes else None,
            'ekf_position': ekf_state[:3].tolist(),
            'ekf_velocity': ekf_state[3:6].tolist(),
            'ekf_attitude': ekf_state[6:9].tolist(),
            'imu_data': latest_imu
        })
        
        # Update visualization
        if len(self.timestamps) % 10 == 0:  # Update every 10 iterations
            self.update_plots()
        
        # Save data periodically (more frequent to ensure plots are available)
        if len(self.log_data) > 0 and len(self.log_data) % 50 == 0:
            self.save_data()
    
    def publish_ekf_estimates(self, ekf_state):
        """Publish EKF state estimates"""
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(ekf_state[0])
        pose_msg.pose.position.y = float(ekf_state[1])
        pose_msg.pose.position.z = float(ekf_state[2])
        self.ekf_pose_pub.publish(pose_msg)
        
        # Publish velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'map'
        vel_msg.twist.linear.x = float(ekf_state[3])
        vel_msg.twist.linear.y = float(ekf_state[4])
        vel_msg.twist.linear.z = float(ekf_state[5])
        self.ekf_velocity_pub.publish(vel_msg)
    
    def update_plots(self):
        """Update real-time plots"""
        if len(self.timestamps) < 2:
            return
        
        # Convert timestamps to relative time
        start_time = self.timestamps[0]
        times = [(t - start_time) for t in self.timestamps]
        
        # Update position plot
        if len(self.true_positions) > 0 and len(self.ekf_positions) > 0:
            true_pos = np.array(self.true_positions)
            ekf_pos = np.array(self.ekf_positions)
            
            for i, label in enumerate(['X', 'Y', 'Z']):
                self.pos_lines[f'true_{label}'].set_data(times, true_pos[:, i])
                self.pos_lines[f'ekf_{label}'].set_data(times, ekf_pos[:, i])
        
        # Update velocity plot
        if len(self.true_velocities) > 0 and len(self.ekf_velocities) > 0:
            true_vel = np.array(self.true_velocities)
            ekf_vel = np.array(self.ekf_velocities)
            
            for i, label in enumerate(['X', 'Y', 'Z']):
                self.vel_lines[f'true_{label}'].set_data(times, true_vel[:, i])
                self.vel_lines[f'ekf_{label}'].set_data(times, ekf_vel[:, i])
        
        # Update attitude plot
        if len(self.true_attitudes) > 0 and len(self.ekf_attitudes) > 0:
            true_att = np.array(self.true_attitudes)
            ekf_att = np.array(self.ekf_attitudes)
            
            for i, label in enumerate(['X', 'Y', 'Z']):
                self.att_lines[f'true_{label}'].set_data(times, true_att[:, i])
                self.att_lines[f'ekf_{label}'].set_data(times, ekf_att[:, i])
        
        # Update 3D trajectory
        if len(self.true_positions) > 0:
            true_pos = np.array(self.true_positions)
            self.traj_true.set_data_3d(true_pos[:, 0], true_pos[:, 1], true_pos[:, 2])
            
            if len(self.ekf_positions) > 0:
                ekf_pos = np.array(self.ekf_positions)
                self.traj_ekf.set_data_3d(ekf_pos[:, 0], ekf_pos[:, 1], ekf_pos[:, 2])
        
        # Update plot limits
        self.ax_pos.relim()
        self.ax_pos.autoscale_view()
        self.ax_vel.relim()
        self.ax_vel.autoscale_view()
        self.ax_att.relim()
        self.ax_att.autoscale_view()
        
        # Update 3D plot limits
        if len(self.true_positions) > 0:
            true_pos = np.array(self.true_positions)
            self.ax_3d.set_xlim(true_pos[:, 0].min()-1, true_pos[:, 0].max()+1)
            self.ax_3d.set_ylim(true_pos[:, 1].min()-1, true_pos[:, 1].max()+1)
            self.ax_3d.set_zlim(true_pos[:, 2].min()-1, true_pos[:, 2].max()+1)
        
        # Refresh plot
        plt.draw()
        plt.pause(0.001)
    
    def save_data(self):
        """Save collected data to file"""
        if not self.log_data:
            return
        
        filename = f"px4_ekf_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, 'w') as f:
            json.dump(self.log_data, f, indent=2)
        
        self.get_logger().info(f"Data saved to {filename}")
        self.log_data = []  # Clear logged data

    def flush_remaining(self):
        """Flush any remaining log data to disk on shutdown."""
        if self.log_data:
            filename = f"px4_ekf_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            try:
                with open(filename, 'w') as f:
                    json.dump(self.log_data, f, indent=2)
                self.get_logger().info(f"Flushed remaining data to {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to flush data: {e}")
            finally:
                self.log_data = []

def main():
    rclpy.init()
    
    ekf_integration = PX4EKFIntegration()
    
    try:
        print("🎯 PX4 EKF Integration started!")
        print("📊 Processing REAL PX4 autonomous flight data")
        print("🧠 Running EKF on actual sensor measurements")
        print("📈 Real-time visualization active")
        print("\nPress Ctrl+C to stop")
        
        rclpy.spin(ekf_integration)
        
    except KeyboardInterrupt:
        ekf_integration.get_logger().info("Stopping PX4 EKF integration")
    finally:
        # Ensure we flush any remaining data to disk
        try:
            ekf_integration.flush_remaining()
        except Exception:
            pass
        ekf_integration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
