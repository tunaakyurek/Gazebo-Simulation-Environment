#!/usr/bin/env python3
"""
Waypoint-Based EKF Integration with Realistic Gazebo Dynamics
This script uses PX4's autonomous flight control with waypoint navigation
while running our custom EKF on realistic sensor data from Gazebo.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import matplotlib.pyplot as plt
import time
import json
from datetime import datetime
from collections import deque

# Import our fine-tuned EKF modules
from ekf_core import ExtendedKalmanFilter
from ekf_parameters import EKFParameters
from ekf_sensor_model import SensorModel

# ROS 2 message types
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from mavros_msgs.msg import State, WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, CommandBool, SetMode
from std_msgs.msg import Header

class WaypointBasedEKFIntegration(Node):
    """EKF integration with waypoint-based autonomous flight using realistic Gazebo dynamics"""
    
    def __init__(self, simulation_duration: float = 120.0):
        super().__init__('waypoint_based_ekf_integration')
        
        # Simulation parameters
        self.simulation_duration = simulation_duration  # 2 minutes for full waypoint mission
        self.start_time = time.time()
        self.simulation_active = True
        
        # QoS profiles for MAVROS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize our fine-tuned EKF
        self.ekf_params = EKFParameters()
        self.ekf = ExtendedKalmanFilter(self.ekf_params)
        self.sensor_model = SensorModel(self.ekf_params)
        
        # Data storage for analysis
        self.max_points = 10000  # Increased for longer mission
        self.true_positions = deque(maxlen=self.max_points)
        self.true_velocities = deque(maxlen=self.max_points)
        self.true_attitudes = deque(maxlen=self.max_points)
        self.ekf_positions = deque(maxlen=self.max_points)
        self.ekf_velocities = deque(maxlen=self.max_points)
        self.ekf_attitudes = deque(maxlen=self.max_points)
        self.timestamps = deque(maxlen=self.max_points)
        self.sensor_data = deque(maxlen=self.max_points)
        
        # Flight state tracking
        self.ekf_initialized = False
        self.px4_connected = False
        self.mission_uploaded = False
        self.flight_started = False
        self.current_waypoint = 0
        
        # MAVROS subscribers for sensor data
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            sensor_qos
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
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
            '/mavros/global_position/global',
            self.gps_callback,
            sensor_qos
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            sensor_qos
        )
        
        # Publishers for our EKF results
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
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
        
        # MAVROS service clients for mission control
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Processing timer
        self.timer = self.create_timer(0.01, self.process_ekf)  # 100 Hz
        
        # Mission management timer
        self.mission_timer = self.create_timer(1.0, self.manage_mission)  # 1 Hz
        
        # Data logging
        self.flight_data = []
        self.log_file = f"waypoint_ekf_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        # Define waypoint mission (figure-8 pattern with altitude variation)
        self.waypoints = self.generate_figure8_waypoints()
        
        # Visualization setup
        self.setup_visualization()
        
        self.get_logger().info("Waypoint-Based EKF Integration initialized")
        self.get_logger().info("Waiting for PX4 connection...")
        
        print("🚁 WAYPOINT-BASED EKF INTEGRATION WITH REALISTIC DYNAMICS")
        print("=========================================================")
        print("✅ Using Gazebo realistic drone physics")
        print("✅ PX4 autonomous flight control")
        print("✅ Waypoint-based mission execution")
        print("✅ Fine-tuned EKF for state estimation")
        print(f"⏱️  Mission duration: {self.simulation_duration} seconds")
        print(f"🎯 {len(self.waypoints)} waypoints planned")
    
    def generate_figure8_waypoints(self):
        """Generate a figure-8 waypoint mission with altitude variation"""
        waypoints = []
        
        # Starting position
        waypoints.append({'x': 0, 'y': 0, 'z': 5, 'yaw': 0})
        
        # Figure-8 pattern waypoints
        t_points = np.linspace(0, 4*np.pi, 16)  # 16 waypoints for smooth figure-8
        
        for t in t_points:
            x = 8 * np.sin(t)  # Figure-8 X component (8m amplitude)
            y = 4 * np.sin(2*t)  # Figure-8 Y component (4m amplitude)
            z = 5 + 2 * np.sin(t/2)  # Altitude variation (3-7m)
            yaw = np.arctan2(np.cos(2*t), np.cos(t))  # Point in direction of travel
            
            waypoints.append({'x': x, 'y': y, 'z': z, 'yaw': yaw})
        
        # Return to start
        waypoints.append({'x': 0, 'y': 0, 'z': 5, 'yaw': 0})
        
        return waypoints
    
    def upload_mission(self):
        """Upload waypoint mission to PX4"""
        if not self.waypoint_push_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Waypoint service not available")
            return False
        
        # Create waypoint list
        waypoint_list = WaypointList()
        
        for i, wp in enumerate(self.waypoints):
            waypoint = Waypoint()
            waypoint.frame = Waypoint.FRAME_LOCAL_NED
            waypoint.command = Waypoint.NAV_WAYPOINT
            waypoint.is_current = (i == 0)
            waypoint.autocontinue = True
            waypoint.param1 = 0.0  # Hold time
            waypoint.param2 = 2.0  # Acceptance radius
            waypoint.param3 = 0.0  # Pass radius
            waypoint.param4 = wp['yaw']
            waypoint.x_lat = wp['x']
            waypoint.y_long = wp['y']
            waypoint.z_alt = wp['z']
            
            waypoint_list.waypoints.append(waypoint)
        
        # Upload mission
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoint_list.waypoints
        
        future = self.waypoint_push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.get_logger().info(f"Mission uploaded successfully! {len(self.waypoints)} waypoints")
            self.mission_uploaded = True
            return True
        else:
            self.get_logger().error("Failed to upload mission")
            return False
    
    def arm_and_start_mission(self):
        """Arm the drone and start autonomous mission"""
        if not self.px4_connected:
            return False
        
        # Set AUTO mode
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            mode_req = SetMode.Request()
            mode_req.custom_mode = "AUTO.MISSION"
            future = self.set_mode_client.call_async(mode_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not (future.result() and future.result().mode_sent):
                self.get_logger().error("Failed to set AUTO mode")
                return False
        
        # Arm the drone
        if self.arming_client.wait_for_service(timeout_sec=5.0):
            arm_req = CommandBool.Request()
            arm_req.value = True
            future = self.arming_client.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().success:
                self.get_logger().info("Drone armed and mission started!")
                self.flight_started = True
                return True
            else:
                self.get_logger().error("Failed to arm drone")
                return False
        
        return False
    
    def manage_mission(self):
        """Manage mission state and progression"""
        if not self.simulation_active:
            return
        
        # Check if mission time is up
        if time.time() - self.start_time > self.simulation_duration:
            self.get_logger().info("Mission duration completed, landing...")
            self.simulation_active = False
            self.save_results()
            return
        
        # Mission state machine
        if self.px4_connected and not self.mission_uploaded:
            self.get_logger().info("Uploading waypoint mission...")
            self.upload_mission()
        
        elif self.mission_uploaded and not self.flight_started:
            self.get_logger().info("Starting autonomous mission...")
            self.arm_and_start_mission()
        
        elif self.flight_started:
            # Monitor mission progress
            progress = min(100, (time.time() - self.start_time) / self.simulation_duration * 100)
            if int(time.time()) % 10 == 0:  # Log every 10 seconds
                self.get_logger().info(f"Mission progress: {progress:.1f}% | Data points: {len(self.timestamps)}")
    
    def state_callback(self, msg):
        """MAVROS state callback"""
        self.px4_connected = msg.connected
        if msg.connected and not self.flight_started:
            self.get_logger().info(f"PX4 connected! Mode: {msg.mode}, Armed: {msg.armed}")
    
    def pose_callback(self, msg):
        """True pose from Gazebo/PX4 (for comparison)"""
        current_time = time.time() - self.start_time
        
        # Store true position
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.true_positions.append(pos)
        
        # Store true attitude (quaternion to euler)
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        # Convert quaternion to euler angles
        roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = np.arcsin(2*(qw*qy - qz*qx))
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        self.true_attitudes.append([roll, pitch, yaw])
        
        if len(self.timestamps) == 0 or current_time > self.timestamps[-1] + 0.008:  # ~100Hz
            self.timestamps.append(current_time)
    
    def velocity_callback(self, msg):
        """True velocity from Gazebo/PX4 (for comparison)"""
        vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        self.true_velocities.append(vel)
    
    def imu_callback(self, msg):
        """IMU data for EKF processing"""
        # Store IMU data for EKF
        imu_data = {
            'timestamp': time.time() - self.start_time,
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        }
        self.sensor_data.append(imu_data)
    
    def gps_callback(self, msg):
        """GPS data for EKF processing"""
        # Convert GPS to local coordinates for EKF
        # This would typically use a geographic projection
        # For simulation, we can use the pose data as reference
        pass
    
    def process_ekf(self):
        """Process EKF estimation at 100Hz"""
        if not self.simulation_active or len(self.true_positions) == 0:
            return
        
        current_time = time.time() - self.start_time
        
        # Initialize EKF with first true state
        if not self.ekf_initialized and len(self.true_positions) > 0:
            initial_state = np.zeros(9)
            initial_state[0:3] = self.true_positions[0]  # Position
            if len(self.true_velocities) > 0:
                initial_state[3:6] = self.true_velocities[0]  # Velocity
            if len(self.true_attitudes) > 0:
                initial_state[6:9] = self.true_attitudes[0]  # Attitude
            
            self.ekf.initialize(initial_state)
            self.ekf_initialized = True
            self.get_logger().info("EKF initialized with realistic Gazebo state")
            return
        
        # Run EKF prediction and update
        if self.ekf_initialized and len(self.sensor_data) > 0:
            dt = 0.01  # 100Hz
            
            # Get latest sensor data
            latest_sensors = self.sensor_data[-1]
            
            # Create sensor measurements for EKF
            imu = {
                'accel': np.array(latest_sensors['linear_acceleration']),
                'gyro': np.array(latest_sensors['angular_velocity'])
            }
            
            # Run EKF step
            x_est, P = self.ekf.step(imu, dt)
            
            # Store EKF estimates
            self.ekf_positions.append(x_est[0:3].tolist())
            self.ekf_velocities.append(x_est[3:6].tolist())
            self.ekf_attitudes.append(x_est[6:9].tolist())
            
            # Publish EKF results
            self.publish_ekf_results(x_est, current_time)
    
    def publish_ekf_results(self, state, timestamp):
        """Publish EKF estimation results"""
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(state[0])
        pose_msg.pose.position.y = float(state[1])
        pose_msg.pose.position.z = float(state[2])
        
        # Convert euler to quaternion
        roll, pitch, yaw = state[6], state[7], state[8]
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        self.ekf_pose_pub.publish(pose_msg)
        
        # Publish velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "map"
        vel_msg.twist.linear.x = float(state[3])
        vel_msg.twist.linear.y = float(state[4])
        vel_msg.twist.linear.z = float(state[5])
        
        self.ekf_velocity_pub.publish(vel_msg)
    
    def setup_visualization(self):
        """Setup real-time visualization"""
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('Waypoint-Based EKF Analysis (Realistic Dynamics)', fontsize=16)
        
        # Will be updated in real-time
        self.position_lines = {}
        self.velocity_lines = {}
    
    def save_results(self):
        """Save simulation results and generate analysis"""
        if len(self.timestamps) == 0:
            self.get_logger().warn("No data to save")
            return
        
        # Convert to numpy arrays
        timestamps = np.array(list(self.timestamps))
        true_pos = np.array(list(self.true_positions))
        true_vel = np.array(list(self.true_velocities))
        true_att = np.array(list(self.true_attitudes))
        ekf_pos = np.array(list(self.ekf_positions))
        ekf_vel = np.array(list(self.ekf_velocities))
        ekf_att = np.array(list(self.ekf_attitudes))
        
        # Ensure arrays are same length
        min_len = min(len(timestamps), len(true_pos), len(ekf_pos))
        timestamps = timestamps[:min_len]
        true_pos = true_pos[:min_len]
        true_vel = true_vel[:min_len]
        true_att = true_att[:min_len]
        ekf_pos = ekf_pos[:min_len]
        ekf_vel = ekf_vel[:min_len]
        ekf_att = ekf_att[:min_len]
        
        # Calculate performance metrics
        pos_errors = np.linalg.norm(true_pos - ekf_pos, axis=1)
        vel_errors = np.linalg.norm(true_vel - ekf_vel, axis=1)
        att_errors = np.linalg.norm(true_att - ekf_att, axis=1)
        
        pos_rmse = np.sqrt(np.mean(pos_errors**2))
        vel_rmse = np.sqrt(np.mean(vel_errors**2))
        att_rmse = np.sqrt(np.mean(att_errors**2))
        
        # Generate comprehensive plots
        self.generate_analysis_plots(timestamps, true_pos, true_vel, true_att, 
                                   ekf_pos, ekf_vel, ekf_att, 
                                   pos_errors, vel_errors, att_errors)
        
        # Save data
        results = {
            'simulation_type': 'waypoint_based_realistic_dynamics',
            'duration': float(timestamps[-1]),
            'sample_rate': len(timestamps) / timestamps[-1],
            'waypoints': self.waypoints,
            'performance_metrics': {
                'position_rmse': float(pos_rmse),
                'velocity_rmse': float(vel_rmse),
                'attitude_rmse': float(att_rmse),
                'max_position_error': float(np.max(pos_errors)),
                'max_velocity_error': float(np.max(vel_errors)),
                'max_attitude_error': float(np.max(att_errors))
            },
            'timestamps': timestamps.tolist(),
            'true_trajectory': {
                'positions': true_pos.tolist(),
                'velocities': true_vel.tolist(),
                'attitudes': true_att.tolist()
            },
            'ekf_estimates': {
                'positions': ekf_pos.tolist(),
                'velocities': ekf_vel.tolist(),
                'attitudes': ekf_att.tolist()
            }
        }
        
        with open(self.log_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\n🎉 WAYPOINT-BASED EKF SIMULATION COMPLETE!")
        print(f"📊 Performance Metrics:")
        print(f"   Position RMSE: {pos_rmse:.3f} m")
        print(f"   Velocity RMSE: {vel_rmse:.3f} m/s")
        print(f"   Attitude RMSE: {np.degrees(att_rmse):.1f}°")
        print(f"📁 Results saved: {self.log_file}")
        print(f"📈 Analysis plots generated")
    
    def generate_analysis_plots(self, timestamps, true_pos, true_vel, true_att,
                               ekf_pos, ekf_vel, ekf_att, pos_errors, vel_errors, att_errors):
        """Generate comprehensive analysis plots"""
        
        # Create comprehensive analysis figure
        fig, axes = plt.subplots(3, 3, figsize=(20, 15))
        fig.suptitle('Waypoint-Based EKF Analysis with Realistic Gazebo Dynamics', fontsize=16)
        
        # Position plots
        axes[0, 0].plot(timestamps, true_pos[:, 0], 'b-', linewidth=2, label='True')
        axes[0, 0].plot(timestamps, ekf_pos[:, 0], 'r--', linewidth=2, label='EKF')
        axes[0, 0].set_title('X Position')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        axes[0, 1].plot(timestamps, true_pos[:, 1], 'b-', linewidth=2, label='True')
        axes[0, 1].plot(timestamps, ekf_pos[:, 1], 'r--', linewidth=2, label='EKF')
        axes[0, 1].set_title('Y Position')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        axes[0, 2].plot(timestamps, true_pos[:, 2], 'b-', linewidth=2, label='True')
        axes[0, 2].plot(timestamps, ekf_pos[:, 2], 'r--', linewidth=2, label='EKF')
        axes[0, 2].set_title('Z Position')
        axes[0, 2].legend()
        axes[0, 2].grid(True)
        
        # Velocity plots
        axes[1, 0].plot(timestamps, true_vel[:, 0], 'b-', linewidth=2, label='True')
        axes[1, 0].plot(timestamps, ekf_vel[:, 0], 'r--', linewidth=2, label='EKF')
        axes[1, 0].set_title('X Velocity')
        axes[1, 0].set_ylabel('Velocity (m/s)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        axes[1, 1].plot(timestamps, true_vel[:, 1], 'b-', linewidth=2, label='True')
        axes[1, 1].plot(timestamps, ekf_vel[:, 1], 'r--', linewidth=2, label='EKF')
        axes[1, 1].set_title('Y Velocity')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        axes[1, 2].plot(timestamps, true_vel[:, 2], 'b-', linewidth=2, label='True')
        axes[1, 2].plot(timestamps, ekf_vel[:, 2], 'r--', linewidth=2, label='EKF')
        axes[1, 2].set_title('Z Velocity')
        axes[1, 2].legend()
        axes[1, 2].grid(True)
        
        # Error plots
        axes[2, 0].plot(timestamps, pos_errors, 'r-', linewidth=2)
        axes[2, 0].set_title('Position Error')
        axes[2, 0].set_ylabel('Error (m)')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].grid(True)
        
        axes[2, 1].plot(timestamps, vel_errors, 'r-', linewidth=2)
        axes[2, 1].set_title('Velocity Error')
        axes[2, 1].set_ylabel('Error (m/s)')
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].grid(True)
        
        axes[2, 2].plot(timestamps, np.degrees(att_errors), 'r-', linewidth=2)
        axes[2, 2].set_title('Attitude Error')
        axes[2, 2].set_ylabel('Error (degrees)')
        axes[2, 2].set_xlabel('Time (s)')
        axes[2, 2].grid(True)
        
        plt.tight_layout()
        plt.savefig('waypoint_based_ekf_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # 3D trajectory plot
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot true and estimated trajectories
        ax.plot(true_pos[:, 0], true_pos[:, 1], true_pos[:, 2], 
                'b-', linewidth=3, label='True Trajectory (Gazebo)')
        ax.plot(ekf_pos[:, 0], ekf_pos[:, 1], ekf_pos[:, 2], 
                'r--', linewidth=2, label='EKF Estimate')
        
        # Plot waypoints
        wp_x = [wp['x'] for wp in self.waypoints]
        wp_y = [wp['y'] for wp in self.waypoints]
        wp_z = [wp['z'] for wp in self.waypoints]
        ax.scatter(wp_x, wp_y, wp_z, c='green', s=100, marker='o', label='Waypoints')
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('3D Trajectory: Waypoint-Based Autonomous Flight\n(Realistic Gazebo Dynamics)')
        ax.legend()
        ax.grid(True)
        
        plt.savefig('waypoint_based_3d_trajectory.png', dpi=300, bbox_inches='tight')
        plt.close()


def main():
    rclpy.init()
    
    # Create and run the waypoint-based EKF integration
    node = WaypointBasedEKFIntegration(simulation_duration=120.0)  # 2 minutes
    
    try:
        print("🚀 Starting waypoint-based EKF integration...")
        print("   Waiting for PX4 connection...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️  Simulation interrupted by user")
    finally:
        if node.simulation_active:
            node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
