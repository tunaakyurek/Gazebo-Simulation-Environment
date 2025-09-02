#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import time
import json
from datetime import datetime
from typing import Dict, Any, Optional

# EKF modules
from ekf_parameters import EKFParameters, get_params
from ekf_core import ExtendedKalmanFilter
from ekf_sensor_model import GazeboSensorInterface
from ekf_dynamics import wrap_to_pi

# ROS2 imports
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import State
from std_msgs.msg import Header

class EKFGazeboIntegration(Node):
    """EKF integration with Gazebo simulation through ROS2"""
    
    def __init__(self):
        super().__init__('ekf_gazebo_integration')
        
        # EKF components
        self.params = get_params()
        self.ekf = ExtendedKalmanFilter(self.params)
        self.sensor_interface = GazeboSensorInterface()
        
        # State tracking
        self.initialized = False
        self.last_imu_time = 0.0
        self.last_gps_time = 0.0
        self.last_baro_time = 0.0
        self.last_mag_time = 0.0
        
        # Data logging
        self.log_data = {
            'timestamps': [],
            'true_states': [],
            'estimated_states': [],
            'covariances': [],
            'imu_measurements': [],
            'gps_measurements': [],
            'baro_measurements': [],
            'mag_measurements': [],
            'innovation_stats': []
        }
        
        # Publishers for EKF estimates
        self.ekf_pose_pub = self.create_publisher(PoseStamped, '/ekf/pose', 10)
        self.ekf_vel_pub = self.create_publisher(TwistStamped, '/ekf/velocity', 10)
        self.ekf_state_pub = self.create_publisher(PoseStamped, '/ekf/state', 10)
        
        # Timer for EKF processing
        self.ekf_timer = self.create_timer(self.params.Ts_IMU, self.ekf_processing_loop)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('EKF Gazebo Integration initialized')
        self.get_logger().info(f'Using drone profile: {self.params.profile}')
        self.get_logger().info(f'IMU rate: {1/self.params.Ts_IMU:.1f} Hz')
        self.get_logger().info(f'GPS rate: {1/self.params.Ts_GPS:.1f} Hz')
    
    def initialize_ekf(self, initial_pose: Optional[np.ndarray] = None):
        """
        Initialize EKF with initial state.
        
        Args:
            initial_pose: Initial pose [x, y, z, roll, pitch, yaw] (optional)
        """
        if initial_pose is None:
            # Default initialization at origin
            x0 = np.zeros(9)
        else:
            # Initialize with provided pose
            x0 = np.zeros(9)
            x0[0:3] = initial_pose[0:3]  # Position
            x0[6:9] = initial_pose[3:6]  # Attitude
        
        # Initialize EKF
        self.ekf.initialize(x0)
        self.initialized = True
        
        self.get_logger().info('EKF initialized with state:')
        self.get_logger().info(f'Position: {x0[0:3]}')
        self.get_logger().info(f'Velocity: {x0[3:6]}')
        self.get_logger().info(f'Attitude: {np.rad2deg(x0[6:9])} deg')
    
    def ekf_processing_loop(self):
        """Main EKF processing loop"""
        if not self.initialized:
            # Try to initialize with current sensor data
            self._try_initialize_from_sensors()
            return
        
        # Spin sensor interface to get latest data
        self.sensor_interface.spin_once()
        
        # Get latest sensor data
        sensors = self.sensor_interface.get_latest_sensors()
        
        if 'imu' not in sensors:
            return  # No IMU data available
        
        current_time = time.time()
        dt = current_time - self.last_imu_time if self.last_imu_time > 0 else self.params.Ts_IMU
        
        # IMU data (always available)
        imu_meas = sensors['imu']
        
        # Optional sensor measurements
        gps_meas = sensors.get('gps', None)
        baro_meas = sensors.get('baro', None)
        mag_meas = sensors.get('mag', None)
        
        # Check if we should use each sensor based on timing
        use_gps = (gps_meas is not None and 
                  current_time - self.last_gps_time >= self.params.Ts_GPS)
        use_baro = (baro_meas is not None and 
                   current_time - self.last_baro_time >= self.params.Ts_Baro)
        use_mag = (mag_meas is not None and 
                  current_time - self.last_mag_time >= self.params.Ts_Mag)
        
        # EKF step
        try:
            x_est, P = self.ekf.step(
                imu_meas, dt,
                gps_meas=gps_meas if use_gps else None,
                baro_meas=baro_meas if use_baro else None,
                mag_meas=mag_meas if use_mag else None,
                use_accel_tilt=True
            )
            
            # Update timing
            self.last_imu_time = current_time
            if use_gps:
                self.last_gps_time = current_time
            if use_baro:
                self.last_baro_time = current_time
            if use_mag:
                self.last_mag_time = current_time
            
            # Publish estimates
            self._publish_estimates(x_est, P)
            
            # Log data
            self._log_data(current_time, x_est, P, sensors)
            
        except Exception as e:
            self.get_logger().error(f'EKF step failed: {e}')
    
    def _try_initialize_from_sensors(self):
        """Try to initialize EKF from available sensor data"""
        self.sensor_interface.spin_once()
        sensors = self.sensor_interface.get_latest_sensors()
        
        if 'pose' in sensors and 'velocity' in sensors:
            # Initialize from Gazebo pose and velocity
            pos = sensors['pose']
            vel = sensors['velocity']
            
            # Get orientation from IMU or pose
            if 'orientation' in sensors:
                # Convert quaternion to Euler angles
                from scipy.spatial.transform import Rotation as R
                quat = sensors['orientation']
                r = R.from_quat(quat)
                euler = r.as_euler('xyz')
            else:
                euler = np.zeros(3)
            
            initial_state = np.concatenate([pos, vel, euler])
            self.initialize_ekf(initial_state)
            
            self.get_logger().info('EKF initialized from Gazebo sensors')
        elif 'imu' in sensors:
            # Initialize with zero state but start processing
            self.initialize_ekf()
            self.get_logger().info('EKF initialized with zero state')
    
    def _publish_estimates(self, x_est: np.ndarray, P: np.ndarray):
        """Publish EKF estimates to ROS topics"""
        current_time = self.get_clock().now()
        
        # Publish pose estimate
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(x_est[0])
        pose_msg.pose.position.y = float(x_est[1])
        pose_msg.pose.position.z = float(x_est[2])
        
        # Convert Euler angles to quaternion
        from scipy.spatial.transform import Rotation as R
        r = R.from_euler('xyz', x_est[6:9])
        quat = r.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        self.ekf_pose_pub.publish(pose_msg)
        
        # Publish velocity estimate
        vel_msg = TwistStamped()
        vel_msg.header = Header()
        vel_msg.header.stamp = current_time.to_msg()
        vel_msg.header.frame_id = 'map'
        vel_msg.twist.linear.x = float(x_est[3])
        vel_msg.twist.linear.y = float(x_est[4])
        vel_msg.twist.linear.z = float(x_est[5])
        vel_msg.twist.angular.x = 0.0  # Not estimated
        vel_msg.twist.angular.y = 0.0
        vel_msg.twist.angular.z = 0.0
        
        self.ekf_vel_pub.publish(vel_msg)
        
        # Publish full state estimate
        state_msg = PoseStamped()
        state_msg.header = Header()
        state_msg.header.stamp = current_time.to_msg()
        state_msg.header.frame_id = 'ekf'
        state_msg.pose.position.x = float(x_est[0])
        state_msg.pose.position.y = float(x_est[1])
        state_msg.pose.position.z = float(x_est[2])
        state_msg.pose.orientation.x = float(x_est[3])  # Velocity in orientation
        state_msg.pose.orientation.y = float(x_est[4])
        state_msg.pose.orientation.z = float(x_est[5])
        state_msg.pose.orientation.w = float(np.linalg.norm(P))  # Covariance norm
        
        self.ekf_state_pub.publish(state_msg)
    
    def _log_data(self, timestamp: float, x_est: np.ndarray, P: np.ndarray, sensors: Dict[str, Any]):
        """Log EKF data for analysis"""
        self.log_data['timestamps'].append(timestamp)
        self.log_data['estimated_states'].append(x_est.tolist())
        self.log_data['covariances'].append(P.tolist())
        
        # Log sensor measurements
        self.log_data['imu_measurements'].append(sensors.get('imu', []).tolist())
        self.log_data['gps_measurements'].append(sensors.get('gps', []).tolist())
        self.log_data['baro_measurements'].append(sensors.get('baro', None))
        self.log_data['mag_measurements'].append(sensors.get('mag', None))
        
        # Log innovation statistics
        stats = self.ekf.get_innovation_stats()
        self.log_data['innovation_stats'].append(stats)
    
    def print_status(self):
        """Print EKF status periodically"""
        if not self.initialized:
            return
        
        x_est = self.ekf.get_state()
        stats = self.ekf.get_innovation_stats()
        
        self.get_logger().info('EKF Status:')
        self.get_logger().info(f'Position: [{x_est[0]:.2f}, {x_est[1]:.2f}, {x_est[2]:.2f}] m')
        self.get_logger().info(f'Velocity: [{x_est[3]:.2f}, {x_est[4]:.2f}, {x_est[5]:.2f}] m/s')
        self.get_logger().info(f'Attitude: [{np.rad2deg(x_est[6]):.1f}, {np.rad2deg(x_est[7]):.1f}, {np.rad2deg(x_est[8]):.1f}] deg')
        
        for sensor_type, data in stats.items():
            if data['count'] > 0:
                self.get_logger().info(f'{sensor_type.upper()}: {data["count"]} updates, '
                                     f'RMS: {data["rms_innovation"]:.3f}, '
                                     f'Rejection: {data["rejection_rate"]:.1%}')
    
    def save_log_data(self, filename: Optional[str] = None):
        """Save logged data to file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f'/u/12/akyuret1/unix/drone_sim/ekf_log_{timestamp}.json'
        
        with open(filename, 'w') as f:
            json.dump(self.log_data, f, indent=2)
        
        self.get_logger().info(f'EKF log data saved to: {filename}')
        return filename
    
    def get_ekf_state(self) -> np.ndarray:
        """Get current EKF state estimate"""
        return self.ekf.get_state()
    
    def get_ekf_covariance(self) -> np.ndarray:
        """Get current EKF covariance matrix"""
        return self.ekf.get_covariance()
    
    def get_innovation_stats(self) -> Dict[str, Dict[str, Any]]:
        """Get innovation statistics"""
        return self.ekf.get_innovation_stats()

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        # Create EKF integration node
        ekf_node = EKFGazeboIntegration()
        
        # Run the node
        rclpy.spin(ekf_node)
        
    except KeyboardInterrupt:
        print('EKF integration shutting down...')
        
        # Save log data
        if hasattr(ekf_node, 'log_data') and ekf_node.log_data['timestamps']:
            filename = ekf_node.save_log_data()
            print(f'Log data saved to: {filename}')
        
    finally:
        if 'ekf_node' in locals():
            ekf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
