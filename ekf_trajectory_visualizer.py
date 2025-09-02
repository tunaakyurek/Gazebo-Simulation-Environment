#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
from typing import List, Tuple
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

class EKFTrajectoryVisualizer(Node):
    """Real-time EKF trajectory visualizer for Gazebo"""
    
    def __init__(self):
        super().__init__('ekf_trajectory_visualizer')
        
        # Trajectory storage
        self.trajectory_points: List[Tuple[float, float, float]] = []
        self.uncertainty_ellipsoids: List[Tuple[float, float, float, float, float, float]] = []
        self.max_trajectory_points = 1000
        
        # Publishers
        self.trajectory_pub = self.create_publisher(Marker, '/ekf/trajectory', 10)
        self.uncertainty_pub = self.create_publisher(MarkerArray, '/ekf/uncertainty', 10)
        self.current_pose_pub = self.create_publisher(Marker, '/ekf/current_pose', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, '/ekf/waypoints', 10)
        
        # Subscribers
        self.ekf_pose_sub = self.create_subscription(
            PoseStamped, '/ekf/pose', self.ekf_pose_callback, 10)
        self.ekf_state_sub = self.create_subscription(
            PoseStamped, '/ekf/state', self.ekf_state_callback, 10)
        
        # Timer for publishing visualization
        self.viz_timer = self.create_timer(0.1, self.publish_visualization)
        
        # Current EKF state
        self.current_pose = None
        self.current_uncertainty = None
        
        # Waypoints for mission visualization
        self.waypoints = [
            [0.0, 0.0, 2.0],    # Takeoff
            [5.0, 0.0, 2.0],    # Forward
            [5.0, 5.0, 2.0],    # Right
            [0.0, 5.0, 2.0],    # Back
            [0.0, 0.0, 2.0],    # Return
            [0.0, 0.0, 0.5]     # Land
        ]
        
        self.get_logger().info('EKF Trajectory Visualizer initialized')
        self.get_logger().info('Publishing to RViz-compatible topics')
    
    def ekf_pose_callback(self, msg: PoseStamped):
        """Callback for EKF pose estimates"""
        self.current_pose = {
            'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            'orientation': [msg.pose.orientation.x, msg.pose.orientation.y, 
                          msg.pose.orientation.z, msg.pose.orientation.w],
            'timestamp': time.time()
        }
        
        # Add to trajectory
        pos = self.current_pose['position']
        self.trajectory_points.append((pos[0], pos[1], pos[2]))
        
        # Limit trajectory length
        if len(self.trajectory_points) > self.max_trajectory_points:
            self.trajectory_points.pop(0)
    
    def ekf_state_callback(self, msg: PoseStamped):
        """Callback for EKF state estimates (includes uncertainty)"""
        # Extract uncertainty from orientation field (covariance norm)
        uncertainty_norm = msg.pose.orientation.w
        self.current_uncertainty = uncertainty_norm
    
    def create_trajectory_marker(self) -> Marker:
        """Create trajectory line marker"""
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        
        marker.ns = 'ekf_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set trajectory points
        marker.points = []
        for point in self.trajectory_points:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(point[2])
            marker.points.append(p)
        
        # Set marker properties
        marker.scale.x = 0.05  # Line width
        marker.color = ColorRGBA()
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.pose.orientation.w = 1.0
        
        return marker
    
    def create_uncertainty_markers(self) -> MarkerArray:
        """Create uncertainty ellipsoid markers"""
        marker_array = MarkerArray()
        
        if self.current_pose is None or self.current_uncertainty is None:
            return marker_array
        
        # Create uncertainty ellipsoid at current position
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        
        marker.ns = 'ekf_uncertainty'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        pos = self.current_pose['position']
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2])
        
        # Scale based on uncertainty
        uncertainty_scale = min(max(self.current_uncertainty * 2.0, 0.1), 2.0)
        marker.scale.x = uncertainty_scale
        marker.scale.y = uncertainty_scale
        marker.scale.z = uncertainty_scale
        
        # Color based on uncertainty level
        marker.color = ColorRGBA()
        if self.current_uncertainty < 0.1:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif self.current_uncertainty < 0.5:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.color.a = 0.3
        
        marker.pose.orientation.w = 1.0
        
        marker_array.markers.append(marker)
        
        return marker_array
    
    def create_current_pose_marker(self) -> Marker:
        """Create current pose marker"""
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        
        marker.ns = 'ekf_current_pose'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        if self.current_pose is not None:
            # Position
            pos = self.current_pose['position']
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            
            # Orientation
            marker.pose.orientation.x = self.current_pose['orientation'][0]
            marker.pose.orientation.y = self.current_pose['orientation'][1]
            marker.pose.orientation.z = self.current_pose['orientation'][2]
            marker.pose.orientation.w = self.current_pose['orientation'][3]
            
            # Scale
            marker.scale.x = 1.0  # Arrow length
            marker.scale.y = 0.1  # Arrow width
            marker.scale.z = 0.1  # Arrow height
            
            # Color
            marker.color = ColorRGBA()
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        
        return marker
    
    def create_waypoint_markers(self) -> MarkerArray:
        """Create waypoint markers"""
        marker_array = MarkerArray()
        
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            
            marker.ns = 'ekf_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = float(waypoint[0])
            marker.pose.position.y = float(waypoint[1])
            marker.pose.position.z = float(waypoint[2])
            
            # Scale
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color
            marker.color = ColorRGBA()
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker.pose.orientation.w = 1.0
            
            marker_array.markers.append(marker)
        
        return marker_array
    
    def publish_visualization(self):
        """Publish all visualization markers"""
        # Publish trajectory
        if len(self.trajectory_points) > 1:
            trajectory_marker = self.create_trajectory_marker()
            self.trajectory_pub.publish(trajectory_marker)
        
        # Publish uncertainty
        uncertainty_markers = self.create_uncertainty_markers()
        if len(uncertainty_markers.markers) > 0:
            self.uncertainty_pub.publish(uncertainty_markers)
        
        # Publish current pose
        current_pose_marker = self.create_current_pose_marker()
        self.current_pose_pub.publish(current_pose_marker)
        
        # Publish waypoints
        waypoint_markers = self.create_waypoint_markers()
        self.waypoint_pub.publish(waypoint_markers)

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        visualizer = EKFTrajectoryVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        print('EKF Trajectory Visualizer shutting down...')
    finally:
        if 'visualizer' in locals():
            visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
