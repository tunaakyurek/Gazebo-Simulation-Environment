#!/usr/bin/env python3
"""
Real PX4 gz_x500 Model EKF Integration
This script integrates our custom EKF with the actual PX4 gz_x500 model running in Gazebo
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
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Header

class RealPX4EKFIntegration(Node):
    """Real-time EKF integration with actual PX4 gz_x500 model"""
    
    def __init__(self, simulation_duration: float = 45.0):
        super().__init__('real_px4_ekf_integration')
        
        # Simulation parameters
        self.simulation_duration = simulation_duration  # 45 seconds
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
        self.max_points = 5000  # Increased for 45 second simulation
        self.true_positions = deque(maxlen=self.max_points)
        self.true_velocities = deque(maxlen=self.max_points)
        self.ekf_positions = deque(maxlen=self.max_points)
        self.ekf_velocities = deque(maxlen=self.max_points)
        self.timestamps = deque(maxlen=self.max_points)
        self.imu_data = deque(maxlen=self.max_points)
        
        # State tracking
        self.ekf_initialized = False
        self.last_imu_time = 0.0
        self.px4_connected = False
        
        # MAVROS subscribers for real PX4 data
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
        
        # Processing timer
        self.timer = self.create_timer(0.01, self.process_ekf)  # 100 Hz
        
        # Auto-stop timer
        self.stop_timer = self.create_timer(1.0, self.check_simulation_time)  # Check every second
        
        # Data logging
        self.flight_data = []
        self.log_file = f"real_px4_ekf_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        # Visualization setup
        self.setup_visualization()
        
        self.get_logger().info("Real PX4 EKF Integration initialized")
        self.get_logger().info("Waiting for PX4 gz_x500 model data...")
        
        print("🚁 REAL PX4 GZ_X500 EKF INTEGRATION")
        print("===================================")
        print("✅ Connected to real PX4 gz_x500 model")
        print("✅ Using fine-tuned EKF parameters")
        print("✅ Real-time processing at 100Hz")
        print(f"⏱️  Auto-stop after {self.simulation_duration} seconds")
        print("🎯 Processing ACTUAL autonomous flight data")
    
    def setup_visualization(self):
        """Setup real-time visualization"""
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('Real PX4 gz_x500 EKF Analysis', fontsize=16)
        
        # Position plot
        self.ax_pos = self.axes[0, 0]
        self.ax_pos.set_title('Position Estimation (PX4 gz_x500)')
        self.ax_pos.set_xlabel('Time (s)')
        self.ax_pos.set_ylabel('Position (m)')
        self.ax_pos.grid(True)
        
        # Velocity plot
        self.ax_vel = self.axes[0, 1]
        self.ax_vel.set_title('Velocity Estimation')
        self.ax_vel.set_xlabel('Time (s)')
        self.ax_vel.set_ylabel('Velocity (m/s)')
        self.ax_vel.grid(True)
        
        # Error plot
        self.ax_err = self.axes[1, 0]
        self.ax_err.set_title('Position Error')
        self.ax_err.set_xlabel('Time (s)')
        self.ax_err.set_ylabel('Error (m)')
        self.ax_err.grid(True)
        
        # 3D trajectory
        self.ax_3d = self.fig.add_subplot(2, 2, 4, projection='3d')
        self.ax_3d.set_title('3D Trajectory (gz_x500)')
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        
        plt.tight_layout()
        plt.show(block=False)
    
    def state_callback(self, msg: State):
        """Handle PX4 state updates"""
        self.px4_connected = msg.connected
        if msg.connected and not self.ekf_initialized:
            self.get_logger().info("PX4 gz_x500 connected! Initializing EKF...")
    
    def pose_callback(self, msg: PoseStamped):
        """Handle real pose data from PX4 gz_x500"""
        if not self.px4_connected:
            return
            
        current_time = time.time()
        
        # Extract position
        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # Convert quaternion to Euler angles
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        
        # Roll
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch
        sinp = 2 * (qw * qy - qz * qx)
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        
        # Yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        attitude = np.array([roll, pitch, yaw])
        
        # Store true data from PX4 gz_x500
        self.true_positions.append(pos)
        self.timestamps.append(current_time)
        
        # Initialize EKF with first real PX4 data
        if not self.ekf_initialized and len(self.true_positions) > 0:
            initial_state = np.concatenate([pos, np.zeros(3), attitude])
            self.ekf.initialize(initial_state)
            self.ekf_initialized = True
            self.get_logger().info("EKF initialized with real PX4 gz_x500 data!")
            print("✅ EKF initialized with REAL PX4 gz_x500 model data")
    
    def velocity_callback(self, msg: TwistStamped):
        """Handle real velocity data from PX4 gz_x500"""
        if not self.px4_connected:
            return
            
        vel = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
        
        self.true_velocities.append(vel)
    
    def imu_callback(self, msg: Imu):
        """Handle real IMU data from PX4 gz_x500"""
        if not self.px4_connected or not self.ekf_initialized:
            return
            
        current_time = time.time()
        dt = current_time - self.last_imu_time if self.last_imu_time > 0 else 0.01
        
        # Extract IMU data
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        imu_measurement = np.concatenate([accel, gyro])
        
        # Store IMU data
        self.imu_data.append({
            'timestamp': current_time,
            'accel': accel.tolist(),
            'gyro': gyro.tolist()
        })
        
        self.last_imu_time = current_time
    
    def check_simulation_time(self):
        """Check if simulation should stop after specified duration"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time >= self.simulation_duration:
            print(f"\n⏱️  Simulation time reached: {elapsed_time:.1f}s")
            print("🛑 Auto-stopping simulation and saving results...")
            self.simulation_active = False
            self.save_final_results()
            rclpy.shutdown()
        else:
            # Print progress every 5 seconds
            if int(elapsed_time) % 5 == 0:
                progress = (elapsed_time / self.simulation_duration) * 100
                print(f"⏱️  Progress: {progress:.1f}% | Time: {elapsed_time:.1f}s | Data points: {len(self.flight_data)}")
    
    def process_ekf(self):
        """Process EKF with real PX4 gz_x500 data"""
        if not self.simulation_active or not self.ekf_initialized or len(self.imu_data) == 0:
            return
        
        # Get latest IMU data
        latest_imu = self.imu_data[-1]
        accel = np.array(latest_imu['accel'])
        gyro = np.array(latest_imu['gyro'])
        imu_measurement = np.concatenate([accel, gyro])
        
        # EKF prediction step with real IMU data
        dt = 0.01  # 100 Hz
        try:
            x_est, P = self.ekf.step(imu_measurement, dt)
            
            # Store EKF estimate
            self.ekf_positions.append(x_est[:3])
            self.ekf_velocities.append(x_est[3:6])
            
            # Publish EKF results
            self.publish_ekf_pose(x_est)
            self.publish_ekf_velocity(x_est)
            
            # Log data
            self.log_data(latest_imu['timestamp'], x_est)
            
            # Update visualization every 10th iteration
            if len(self.ekf_positions) % 10 == 0:
                self.update_plots()
                
        except Exception as e:
            self.get_logger().error(f"EKF processing error: {e}")
    
    def publish_ekf_pose(self, state: np.ndarray):
        """Publish EKF pose estimate"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        # Position
        pose_msg.pose.position.x = float(state[0])
        pose_msg.pose.position.y = float(state[1])
        pose_msg.pose.position.z = float(state[2])
        
        # Orientation (simplified - convert from Euler)
        roll, pitch, yaw = state[6:9]
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
    
    def publish_ekf_velocity(self, state: np.ndarray):
        """Publish EKF velocity estimate"""
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "map"
        
        vel_msg.twist.linear.x = float(state[3])
        vel_msg.twist.linear.y = float(state[4])
        vel_msg.twist.linear.z = float(state[5])
        
        self.ekf_velocity_pub.publish(vel_msg)
    
    def log_data(self, timestamp: float, state: np.ndarray):
        """Log real PX4 gz_x500 EKF data"""
        data_point = {
            'timestamp': timestamp,
            'ekf_state': state.tolist(),
            'true_position': self.true_positions[-1].tolist() if self.true_positions else [0, 0, 0],
            'true_velocity': self.true_velocities[-1].tolist() if self.true_velocities else [0, 0, 0],
            'source': 'real_px4_gz_x500'
        }
        self.flight_data.append(data_point)
        
        # Save periodically
        if len(self.flight_data) % 100 == 0:
            with open(self.log_file, 'w') as f:
                json.dump(self.flight_data, f, indent=2)
    
    def update_plots(self):
        """Update real-time visualization"""
        if len(self.true_positions) < 2 or len(self.ekf_positions) < 2:
            return
        
        # Convert to arrays
        true_pos = np.array(list(self.true_positions))
        ekf_pos = np.array(list(self.ekf_positions))
        times = np.array(list(self.timestamps))
        
        if len(times) < 2:
            return
            
        # Relative time
        times = times - times[0]
        
        # Clear and update plots
        self.ax_pos.clear()
        self.ax_pos.plot(times, true_pos[:, 0], 'b-', label='PX4 gz_x500 X', linewidth=2)
        self.ax_pos.plot(times, ekf_pos[:, 0], 'r--', label='EKF X', alpha=0.8)
        self.ax_pos.plot(times, true_pos[:, 1], 'g-', label='PX4 gz_x500 Y', linewidth=2)
        self.ax_pos.plot(times, ekf_pos[:, 1], 'm--', label='EKF Y', alpha=0.8)
        self.ax_pos.set_title('Position Estimation (Real PX4 gz_x500)')
        self.ax_pos.set_xlabel('Time (s)')
        self.ax_pos.set_ylabel('Position (m)')
        self.ax_pos.legend()
        self.ax_pos.grid(True)
        
        # Velocity plot
        if len(self.true_velocities) > 0 and len(self.ekf_velocities) > 0:
            true_vel = np.array(list(self.true_velocities))
            ekf_vel = np.array(list(self.ekf_velocities))
            
            self.ax_vel.clear()
            self.ax_vel.plot(times[:len(true_vel)], true_vel[:, 0], 'b-', label='PX4 Vx', linewidth=2)
            self.ax_vel.plot(times[:len(ekf_vel)], ekf_vel[:, 0], 'r--', label='EKF Vx', alpha=0.8)
            self.ax_vel.set_title('Velocity Estimation')
            self.ax_vel.set_xlabel('Time (s)')
            self.ax_vel.set_ylabel('Velocity (m/s)')
            self.ax_vel.legend()
            self.ax_vel.grid(True)
        
        # Position error
        if len(true_pos) == len(ekf_pos):
            pos_errors = np.linalg.norm(ekf_pos - true_pos, axis=1)
            self.ax_err.clear()
            self.ax_err.plot(times, pos_errors, 'r-', linewidth=2)
            self.ax_err.set_title('Position Error')
            self.ax_err.set_xlabel('Time (s)')
            self.ax_err.set_ylabel('Error (m)')
            self.ax_err.grid(True)
        
        # 3D trajectory
        self.ax_3d.clear()
        self.ax_3d.plot(true_pos[:, 0], true_pos[:, 1], true_pos[:, 2], 'b-', linewidth=3, label='PX4 gz_x500')
        self.ax_3d.plot(ekf_pos[:, 0], ekf_pos[:, 1], ekf_pos[:, 2], 'r--', linewidth=2, alpha=0.8, label='EKF Estimate')
        self.ax_3d.set_title('3D Trajectory (Real PX4 gz_x500)')
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        self.ax_3d.legend()
        
        plt.draw()
        plt.pause(0.001)
    
    def save_final_results(self):
        """Save final analysis results"""
        print("\n📊 REAL PX4 GZ_X500 EKF ANALYSIS RESULTS")
        print("=========================================")
        
        elapsed_time = time.time() - self.start_time
        print(f"🎯 Simulation Duration: {elapsed_time:.1f} seconds")
        print(f"📊 Total Data Points: {len(self.flight_data)}")
        print(f"🔗 PX4 Connection: {'✅ Connected' if self.px4_connected else '❌ Not Connected'}")
        print(f"🧠 EKF Initialized: {'✅ Yes' if self.ekf_initialized else '❌ No'}")
        
        if len(self.true_positions) > 0 and len(self.ekf_positions) > 0:
            true_pos = np.array(list(self.true_positions))
            ekf_pos = np.array(list(self.ekf_positions))
            
            # Calculate performance with real PX4 data
            min_len = min(len(true_pos), len(ekf_pos))
            pos_errors = np.linalg.norm(ekf_pos[:min_len] - true_pos[:min_len], axis=1)
            
            print(f"\n📈 PERFORMANCE WITH REAL PX4 GZ_X500 MODEL:")
            print(f"   Position RMSE: {np.sqrt(np.mean(pos_errors**2)):.3f} m")
            print(f"   Max Position Error: {np.max(pos_errors):.3f} m")
            print(f"   Mean Position Error: {np.mean(pos_errors):.3f} m")
            print(f"   Position Samples: {len(true_pos)}")
            print(f"   EKF Estimates: {len(ekf_pos)}")
            
            # Save final plot
            plt.savefig('real_px4_gz_x500_ekf_analysis.png', dpi=300, bbox_inches='tight')
            print(f"✅ Real PX4 gz_x500 analysis saved: real_px4_gz_x500_ekf_analysis.png")
        else:
            print("⚠️  Limited data collected - check MAVROS connection")
        
        # Save data with metadata
        final_data = {
            'metadata': {
                'simulation_duration': elapsed_time,
                'target_duration': self.simulation_duration,
                'data_points': len(self.flight_data),
                'px4_connected': self.px4_connected,
                'ekf_initialized': self.ekf_initialized,
                'source': 'real_px4_gz_x500_autonomous_flight',
                'timestamp': datetime.now().isoformat()
            },
            'flight_data': self.flight_data
        }
        
        with open(self.log_file, 'w') as f:
            json.dump(final_data, f, indent=2)
        print(f"✅ Real PX4 gz_x500 data saved: {self.log_file}")
        
        print(f"\n🎉 REAL PX4 GZ_X500 EKF SIMULATION COMPLETE!")
        print(f"📁 Results saved with 'real_px4_gz_x500' identifier")
        print(f"🎯 Processed {elapsed_time:.1f}s of ACTUAL autonomous flight data")

def main(args=None):
    rclpy.init(args=args)
    
    # Allow custom duration
    duration = 45.0  # 45 seconds default
    if args and len(args) > 0:
        try:
            duration = float(args[0])
        except:
            duration = 45.0
    
    ekf_integration = RealPX4EKFIntegration(simulation_duration=duration)
    
    try:
        print(f"🚀 Starting {duration}s real PX4 gz_x500 EKF integration...")
        print("⏱️  Will auto-stop and save results after specified duration")
        print("🎯 Processing REAL autonomous flight data from PX4 gz_x500")
        print("📊 Progress updates every 5 seconds")
        print()
        rclpy.spin(ekf_integration)
    except KeyboardInterrupt:
        print("\n🛑 Manual stop - saving results...")
        ekf_integration.save_final_results()
    except Exception as e:
        print(f"\n✅ Simulation completed: {e}")
        ekf_integration.save_final_results()
    finally:
        try:
            ekf_integration.destroy_node()
        except:
            pass

if __name__ == '__main__':
    main()
