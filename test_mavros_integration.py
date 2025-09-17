#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import State
import time

class TestMAVROSIntegration(Node):
    def __init__(self):
        super().__init__('test_mavros_integration')
        
        self.data_received = {}
        self.start_time = time.time()
        
        # Create subscribers for key topics
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
            
        self.get_logger().info("🧪 MAVROS Integration Test Started")
        self.get_logger().info("Testing data flow from key topics...")
        
        # Timer to check results
        self.timer = self.create_timer(1.0, self.check_data)
        
    def state_callback(self, msg):
        if 'state' not in self.data_received:
            self.data_received['state'] = 0
            self.get_logger().info(f"✅ STATE data received! Connected: {msg.connected}")
        self.data_received['state'] += 1
        
    def pose_callback(self, msg):
        if 'pose' not in self.data_received:
            self.data_received['pose'] = 0
            self.get_logger().info("✅ POSE data received!")
        self.data_received['pose'] += 1
        
    def imu_callback(self, msg):
        if 'imu' not in self.data_received:
            self.data_received['imu'] = 0
            self.get_logger().info("✅ IMU data received!")
        self.data_received['imu'] += 1
        
    def gps_callback(self, msg):
        if 'gps' not in self.data_received:
            self.data_received['gps'] = 0
            self.get_logger().info("✅ GPS data received!")
        self.data_received['gps'] += 1
        
    def check_data(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"📊 After {elapsed:.1f}s - Data counts: {self.data_received}")
        
        if elapsed > 30:  # Test for 30 seconds
            self.get_logger().info("🏁 Test completed!")
            if any(self.data_received.values()):
                self.get_logger().info("✅ SUCCESS: MAVROS data is flowing!")
            else:
                self.get_logger().info("❌ FAILURE: No MAVROS data detected")
            rclpy.shutdown()

def main():
    rclpy.init()
    test_node = TestMAVROSIntegration()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
