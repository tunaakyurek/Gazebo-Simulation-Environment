#!/usr/bin/env python3
"""
Comprehensive Data Pipeline Diagnostic Tool
Identifies exactly where the data flow breaks between PX4 gz_x500 -> MAVROS -> EKF
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import subprocess
import signal
import sys

# ROS 2 message types
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Header

class DataPipelineDiagnostic(Node):
    """Comprehensive diagnostic for PX4 -> MAVROS -> EKF data pipeline"""
    
    def __init__(self):
        super().__init__('data_pipeline_diagnostic')
        
        # Diagnostic counters
        self.mavros_state_count = 0
        self.mavros_pose_count = 0
        self.mavros_velocity_count = 0
        self.mavros_imu_count = 0
        self.mavros_gps_count = 0
        
        # Connection status
        self.px4_connected = False
        self.px4_armed = False
        self.px4_mode = ""
        
        # Data samples for analysis
        self.latest_pose = None
        self.latest_velocity = None
        self.latest_imu = None
        self.latest_state = None
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers with diagnostic callbacks
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.diagnostic_state_callback, sensor_qos)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.diagnostic_pose_callback, sensor_qos)
        
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.diagnostic_velocity_callback, sensor_qos)
        
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.diagnostic_imu_callback, sensor_qos)
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.diagnostic_gps_callback, sensor_qos)
        
        # Diagnostic timer
        self.diagnostic_timer = self.create_timer(2.0, self.print_diagnostics)
        
        # Auto-stop timer
        self.start_time = time.time()
        self.stop_timer = self.create_timer(1.0, self.check_stop_condition)
        
        print("🔍 DATA PIPELINE DIAGNOSTIC STARTED")
        print("===================================")
        print("Monitoring for 30 seconds...")
        print("Checking: PX4 gz_x500 -> MAVROS -> EKF data flow")
        print()
    
    def diagnostic_state_callback(self, msg: State):
        """Monitor MAVROS state"""
        self.mavros_state_count += 1
        self.px4_connected = msg.connected
        self.px4_armed = msg.armed
        self.px4_mode = msg.mode
        self.latest_state = msg
    
    def diagnostic_pose_callback(self, msg: PoseStamped):
        """Monitor MAVROS pose data"""
        self.mavros_pose_count += 1
        self.latest_pose = msg
    
    def diagnostic_velocity_callback(self, msg: TwistStamped):
        """Monitor MAVROS velocity data"""
        self.mavros_velocity_count += 1
        self.latest_velocity = msg
    
    def diagnostic_imu_callback(self, msg: Imu):
        """Monitor MAVROS IMU data"""
        self.mavros_imu_count += 1
        self.latest_imu = msg
    
    def diagnostic_gps_callback(self, msg: NavSatFix):
        """Monitor MAVROS GPS data"""
        self.mavros_gps_count += 1
    
    def check_gazebo_processes(self):
        """Check Gazebo and PX4 processes"""
        try:
            # Check PX4 gz_x500 processes
            result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
            px4_processes = [line for line in result.stdout.split('\n') if 'px4' in line and 'gz_x500' in line]
            gazebo_processes = [line for line in result.stdout.split('\n') if 'gz sim' in line]
            mavros_processes = [line for line in result.stdout.split('\n') if 'mavros_node' in line]
            
            return {
                'px4_count': len(px4_processes),
                'gazebo_count': len(gazebo_processes),
                'mavros_count': len(mavros_processes),
                'px4_details': px4_processes[:2],  # First 2 processes
                'gazebo_details': gazebo_processes[:2]
            }
        except:
            return {'error': 'Could not check processes'}
    
    def check_ros_topics(self):
        """Check available ROS topics"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
            topics = result.stdout.split('\n')
            mavros_topics = [t for t in topics if 'mavros' in t]
            ekf_topics = [t for t in topics if 'ekf' in t]
            
            return {
                'total_topics': len(topics),
                'mavros_topics': len(mavros_topics),
                'ekf_topics': len(ekf_topics),
                'key_topics_present': {
                    'mavros_state': '/mavros/state' in topics,
                    'mavros_pose': '/mavros/local_position/pose' in topics,
                    'mavros_imu': '/mavros/imu/data' in topics,
                    'ekf_pose': '/ekf/pose' in topics
                }
            }
        except:
            return {'error': 'Could not check topics'}
    
    def print_diagnostics(self):
        """Print comprehensive diagnostic information"""
        elapsed = time.time() - self.start_time
        
        print(f"\n🔍 DIAGNOSTIC REPORT - {elapsed:.1f}s elapsed")
        print("=" * 50)
        
        # Process status
        process_info = self.check_gazebo_processes()
        print(f"🖥️  PROCESS STATUS:")
        print(f"   PX4 gz_x500 processes: {process_info.get('px4_count', 0)}")
        print(f"   Gazebo processes: {process_info.get('gazebo_count', 0)}")
        print(f"   MAVROS processes: {process_info.get('mavros_count', 0)}")
        
        # ROS topics status
        topic_info = self.check_ros_topics()
        print(f"\n📡 ROS TOPICS STATUS:")
        print(f"   Total topics: {topic_info.get('total_topics', 0)}")
        print(f"   MAVROS topics: {topic_info.get('mavros_topics', 0)}")
        print(f"   EKF topics: {topic_info.get('ekf_topics', 0)}")
        
        key_topics = topic_info.get('key_topics_present', {})
        print(f"   Key topics present:")
        for topic, present in key_topics.items():
            status = "✅" if present else "❌"
            print(f"     {status} {topic}")
        
        # MAVROS data flow
        print(f"\n📊 MAVROS DATA FLOW:")
        print(f"   State messages: {self.mavros_state_count}")
        print(f"   Pose messages: {self.mavros_pose_count}")
        print(f"   Velocity messages: {self.mavros_velocity_count}")
        print(f"   IMU messages: {self.mavros_imu_count}")
        print(f"   GPS messages: {self.mavros_gps_count}")
        
        # Connection status
        print(f"\n🔗 PX4 CONNECTION STATUS:")
        print(f"   Connected: {'✅ Yes' if self.px4_connected else '❌ No'}")
        print(f"   Armed: {'✅ Yes' if self.px4_armed else '❌ No'}")
        print(f"   Mode: {self.px4_mode or 'Unknown'}")
        
        # Data quality check
        print(f"\n📈 DATA QUALITY:")
        if self.latest_pose:
            pos = self.latest_pose.pose.position
            print(f"   Latest position: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]")
        else:
            print(f"   ❌ No pose data received")
            
        if self.latest_imu:
            acc = self.latest_imu.linear_acceleration
            print(f"   Latest IMU accel: [{acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f}]")
        else:
            print(f"   ❌ No IMU data received")
        
        # Problem identification
        print(f"\n🚨 PROBLEM IDENTIFICATION:")
        if self.mavros_state_count == 0:
            print("   ❌ CRITICAL: No MAVROS state messages - MAVROS not working")
        elif not self.px4_connected:
            print("   ❌ CRITICAL: MAVROS not connected to PX4")
        elif self.mavros_pose_count == 0:
            print("   ❌ CRITICAL: No pose data - PX4 not publishing position")
        elif self.mavros_imu_count == 0:
            print("   ❌ CRITICAL: No IMU data - PX4 sensors not working")
        elif not self.px4_armed:
            print("   ⚠️  WARNING: PX4 not armed - limited data expected")
        else:
            print("   ✅ Data pipeline appears healthy")
        
        print("-" * 50)
    
    def check_stop_condition(self):
        """Check if diagnostic should stop"""
        elapsed = time.time() - self.start_time
        if elapsed >= 30.0:  # Run diagnostic for 30 seconds
            print(f"\n🎯 DIAGNOSTIC COMPLETE - {elapsed:.1f}s")
            print("=" * 50)
            
            # Final recommendations
            print("🔧 RECOMMENDATIONS:")
            if self.mavros_state_count == 0:
                print("   1. Check MAVROS is running: ros2 run mavros mavros_node")
                print("   2. Check correct FCU URL: fcu_url:=udp://:14540@127.0.0.1:14580")
            elif not self.px4_connected:
                print("   1. Check PX4 is running and listening on port 14580")
                print("   2. Try different MAVROS FCU URL: 14557 instead of 14580")
                print("   3. Check firewall/network settings")
            elif self.mavros_pose_count == 0:
                print("   1. Check PX4 EKF2 is running: 'ekf2 status' in PX4 shell")
                print("   2. Arm PX4: 'commander arm -f' in PX4 shell")
                print("   3. Check sensor data: 'sensor_accel status' in PX4 shell")
            else:
                print("   ✅ Data pipeline is working!")
                print(f"   📊 Collected {self.mavros_pose_count} pose samples")
                print(f"   📊 Collected {self.mavros_imu_count} IMU samples")
            
            rclpy.shutdown()

def main():
    print("🚁 PX4 GZ_X500 DATA PIPELINE DIAGNOSTIC")
    print("=======================================")
    print("This tool will identify exactly where data flow breaks")
    print("Monitoring: PX4 gz_x500 -> MAVROS -> EKF pipeline")
    print()
    
    rclpy.init()
    diagnostic = DataPipelineDiagnostic()
    
    try:
        rclpy.spin(diagnostic)
    except KeyboardInterrupt:
        print("\n🛑 Diagnostic stopped by user")
    except Exception as e:
        print(f"\n✅ Diagnostic completed: {e}")
    finally:
        try:
            diagnostic.destroy_node()
        except:
            pass

if __name__ == '__main__':
    main()
