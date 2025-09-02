#!/usr/bin/env python3
"""
Show current system status and generate summary plots
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import matplotlib.pyplot as plt
import time
import json
import os
import glob

# ROS 2 message types
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

class SystemStatusMonitor(Node):
    """Monitor system status and generate summary plots"""
    
    def __init__(self):
        super().__init__('system_status_monitor')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Data collection
        self.data_points = []
        self.start_time = time.time()
        
        # Subscribers
        self.true_pose_sub = self.create_subscription(
            PoseStamped,
            '/autonomous_drone/pose',
            self.true_pose_callback,
            qos_profile
        )
        
        self.motor_speed_sub = self.create_subscription(
            Float32MultiArray,
            '/autonomous_drone/command/motor_speed',
            self.motor_speed_callback,
            qos_profile
        )
        
        # Timer for data collection
        self.create_timer(0.1, self.collect_data)
        
        self.get_logger().info("System status monitor initialized")
        
    def true_pose_callback(self, msg):
        """Handle true pose data"""
        self.current_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'timestamp': time.time()
        }
        
    def motor_speed_callback(self, msg):
        """Handle motor speed data"""
        self.current_motor_speeds = list(msg.data)
        
    def collect_data(self):
        """Collect data for analysis"""
        if hasattr(self, 'current_pose') and hasattr(self, 'current_motor_speeds'):
            data_point = {
                'timestamp': time.time(),
                'position': [self.current_pose['x'], self.current_pose['y'], self.current_pose['z']],
                'motor_speeds': self.current_motor_speeds
            }
            self.data_points.append(data_point)
            
            # Log status every 50 points
            if len(self.data_points) % 50 == 0:
                self.get_logger().info(
                    f"Collected {len(self.data_points)} data points. "
                    f"Current position: [{self.current_pose['x']:.2f}, {self.current_pose['y']:.2f}, {self.current_pose['z']:.2f}]"
                )
    
    def generate_summary_plot(self):
        """Generate a summary plot of the collected data"""
        if len(self.data_points) < 10:
            self.get_logger().warn("Not enough data points for plotting")
            return
        
        # Convert to numpy arrays
        timestamps = np.array([p['timestamp'] for p in self.data_points])
        positions = np.array([p['position'] for p in self.data_points])
        motor_speeds = np.array([p['motor_speeds'] for p in self.data_points])
        
        # Convert to relative time
        times = timestamps - timestamps[0]
        
        # Create plots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Autonomous Drone System - Live Data Summary', fontsize=16)
        
        # Position vs time
        ax1 = axes[0, 0]
        ax1.plot(times, positions[:, 0], 'r-', label='X Position')
        ax1.plot(times, positions[:, 1], 'g-', label='Y Position')
        ax1.plot(times, positions[:, 2], 'b-', label='Z Position')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position (m)')
        ax1.set_title('Position vs Time')
        ax1.legend()
        ax1.grid(True)
        
        # 3D trajectory
        ax2 = fig.add_subplot(2, 2, 2, projection='3d')
        ax2.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2)
        ax2.scatter(positions[0, 0], positions[0, 1], positions[0, 2], color='green', s=100, label='Start')
        ax2.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], color='red', s=100, label='End')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Z (m)')
        ax2.set_title('3D Flight Trajectory')
        ax2.legend()
        ax2.grid(True)
        
        # Motor speeds
        ax3 = axes[1, 0]
        for i in range(4):
            ax3.plot(times, motor_speeds[:, i], label=f'Motor {i+1}')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Motor Speed')
        ax3.set_title('Motor Speeds vs Time')
        ax3.legend()
        ax3.grid(True)
        
        # System statistics
        ax4 = axes[1, 1]
        ax4.axis('off')
        
        # Calculate statistics
        flight_time = times[-1]
        total_distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
        avg_speed = total_distance / flight_time if flight_time > 0 else 0
        max_altitude = np.max(positions[:, 2])
        min_altitude = np.min(positions[:, 2])
        
        stats_text = f"""
System Statistics:
• Flight Time: {flight_time:.1f} seconds
• Total Distance: {total_distance:.1f} meters
• Average Speed: {avg_speed:.2f} m/s
• Max Altitude: {max_altitude:.2f} m
• Min Altitude: {min_altitude:.2f} m
• Data Points: {len(self.data_points)}
• Current Position: [{positions[-1, 0]:.2f}, {positions[-1, 1]:.2f}, {positions[-1, 2]:.2f}]
• Current Motor Speeds: {motor_speeds[-1]}
        """
        
        ax4.text(0.1, 0.9, stats_text, transform=ax4.transAxes, fontsize=12,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = f"system_status_summary_{int(time.time())}.png"
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"Summary plot saved as: {plot_filename}")
        
        plt.show()
        
        return fig

def main():
    rclpy.init()
    
    monitor = SystemStatusMonitor()
    
    try:
        print("🎯 System Status Monitor started!")
        print("📊 Collecting data for 30 seconds...")
        print("   - Monitoring position data")
        print("   - Monitoring motor commands")
        print("   - Will generate summary plot")
        
        # Run for 30 seconds to collect data
        rclpy.spin_once(monitor, timeout_sec=30)
        
        print("📈 Generating summary plot...")
        monitor.generate_summary_plot()
        
        print("✅ System status monitoring complete!")
        
    except KeyboardInterrupt:
        monitor.get_logger().info("Stopping system status monitor")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
