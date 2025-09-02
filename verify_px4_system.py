#!/usr/bin/env python3
"""
Verify PX4 Autonomous System Status
"""

import rclpy
from rclpy.node import Node
import time

class SystemVerifier(Node):
    def __init__(self):
        super().__init__('system_verifier')
        
        print("🎯 PX4 AUTONOMOUS SYSTEM VERIFICATION")
        print("=====================================")
        
        # Check ROS topics
        self.check_ros_topics()
        
    def check_ros_topics(self):
        """Check available ROS topics"""
        try:
            topics = self.get_topic_names_and_types()
            
            print("\n📡 ROS Topics Available:")
            print("----------------------")
            
            # Check for MAVROS topics
            mavros_topics = [t[0] for t in topics if 'mavros' in t[0]]
            if mavros_topics:
                print(f"✅ MAVROS Topics: {len(mavros_topics)} found")
                print("   Key topics:")
                for topic in mavros_topics[:5]:  # Show first 5
                    print(f"   - {topic}")
            else:
                print("❌ No MAVROS topics found")
            
            # Check for EKF topics
            ekf_topics = [t[0] for t in topics if 'ekf' in t[0]]
            if ekf_topics:
                print(f"✅ EKF Topics: {len(ekf_topics)} found")
                for topic in ekf_topics:
                    print(f"   - {topic}")
            else:
                print("⚠️  No EKF topics found yet")
            
            # Check for sensor topics
            sensor_topics = [t[0] for t in topics if any(s in t[0] for s in ['imu', 'gps', 'pose', 'velocity'])]
            if sensor_topics:
                print(f"✅ Sensor Topics: {len(sensor_topics)} found")
                for topic in sensor_topics[:5]:  # Show first 5
                    print(f"   - {topic}")
            
            print(f"\n📊 Total ROS Topics: {len(topics)}")
            
        except Exception as e:
            print(f"❌ Error checking topics: {e}")

def main():
    rclpy.init()
    
    verifier = SystemVerifier()
    
    try:
        rclpy.spin_once(verifier, timeout_sec=2)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
