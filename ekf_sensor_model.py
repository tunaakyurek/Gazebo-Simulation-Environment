#!/usr/bin/env python3

import numpy as np
from typing import Dict, Any, Optional
from ekf_parameters import EKFParameters
from ekf_dynamics import rotation_matrix, wrap_to_pi

class SensorModel:
    """Sensor model for generating synthetic sensor readings from true state"""
    
    def __init__(self, params: EKFParameters):
        self.params = params
        
        # Persistent biases and previous values
        self.accel_bias: Optional[np.ndarray] = None
        self.gyro_bias: Optional[np.ndarray] = None
        self.prev_t: Optional[float] = None
        self.prev_vel: Optional[np.ndarray] = None
        self.prev_att: Optional[np.ndarray] = None
        
        # Initialize biases
        self._initialize_biases()
    
    def _initialize_biases(self):
        """Initialize sensor biases"""
        self.accel_bias = self.params.IMU_accel_bias_instab * np.random.randn(3)
        self.gyro_bias = self.params.IMU_gyro_bias_instab * np.random.randn(3)
    
    def generate_sensors(self, x: np.ndarray, t: float) -> Dict[str, Any]:
        """
        Generate synthetic sensor readings from true state.
        
        Args:
            x: true state [pos(3); vel(3); att(3)]
            t: current time (s)
        
        Returns:
            sensors: dict with fields accel, gyro, gps, baro, mag
        """
        # Initialize persistent variables on first call
        if self.prev_t is None:
            self.prev_t = t
            self.prev_vel = x[3:6].copy()
            self.prev_att = x[6:9].copy()
        
        # Unpack true values
        pos = x[0:3]
        vel = x[3:6]
        att = x[6:9]
        g = self.params.g
        
        # Finite-difference step aligned to IMU period
        dt_fd = self.params.Ts_IMU
        
        # IMU (body frame) mechanization
        R = rotation_matrix(att[0], att[1], att[2])
        
        # True linear acceleration in NED via finite difference
        a_true_ned = (vel - self.prev_vel) / dt_fd
        
        # Specific force in body frame: f_b = R' * (a - g)
        f_body = R.T @ (a_true_ned - g)
        
        # True Euler angle rates via finite difference (with angle wrapping)
        ang_diff = np.array([
            wrap_to_pi(att[0] - self.prev_att[0]),
            wrap_to_pi(att[1] - self.prev_att[1]),
            wrap_to_pi(att[2] - self.prev_att[2])
        ])
        att_dot = ang_diff / dt_fd
        
        # Map Euler angle rates to body rates: att_dot = E * omega_body
        phi, theta = att[0], att[1]
        E = np.array([
            [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])
        
        if np.linalg.cond(E) < 1e6:
            omega_body = np.linalg.solve(E, att_dot)
        else:
            omega_body = np.zeros(3)
        
        # Add sensor noises and biases - REDUCED for stability
        # IMU noise per sample (discrete-time) using noise densities - reduced by 50%
        accel_noise = 0.5 * self.params.IMU_accel_noise_density / np.sqrt(self.params.Ts_IMU) * np.random.randn(3)
        gyro_noise = 0.5 * self.params.IMU_gyro_noise_density / np.sqrt(self.params.Ts_IMU) * np.random.randn(3)
        
        # Compose IMU measurements with reduced bias
        accel_meas = f_body + 0.3 * self.accel_bias + accel_noise  # Reduced bias
        gyro_meas = omega_body + 0.3 * self.gyro_bias + gyro_noise  # Reduced bias
        
        # GPS (position) - reduced noise for stability
        gps_noise = np.array([
            0.5 * self.params.GPS_sigma_xy * np.random.randn(),  # Reduced GPS noise
            0.5 * self.params.GPS_sigma_xy * np.random.randn(),
            0.5 * self.params.GPS_sigma_z * np.random.randn()
        ])
        gps_meas = pos + gps_noise
        
        # Barometer (altitude = -z in NED) - reduced noise
        baro_noise = 0.3 * self.params.Baro_sigma_z * np.random.randn()  # Much less baro noise
        baro_meas = -pos[2] + baro_noise
        
        # Magnetometer (yaw/heading) - significantly reduced noise for stable heading
        mag_noise = 0.2 * self.params.Mag_sigma_rad * np.random.randn()  # Much less mag noise
        mag_meas = att[2] + mag_noise
        
        # Update persistence
        self.prev_t = t
        self.prev_vel = vel.copy()
        self.prev_att = att.copy()
        
        return {
            'accel': accel_meas,
            'gyro': gyro_meas,
            'gps': gps_meas,
            'baro': baro_meas,
            'mag': mag_meas,
            'imu': np.concatenate([accel_meas, gyro_meas])
        }

class GazeboSensorInterface:
    """Interface to Gazebo sensors through ROS2 topics"""
    
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Imu, NavSatFix
        from geometry_msgs.msg import PoseStamped, TwistStamped
        
        self.node = Node('gazebo_sensor_interface')
        
        # QoS compatible with MAVROS (often best-effort sensor data)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Subscribers for Gazebo/MAVROS sensor data
        self.imu_sub = self.node.create_subscription(
            Imu, '/mavros/imu/data_raw', self.imu_callback, sensor_qos)
        self.gps_sub = self.node.create_subscription(
            NavSatFix, '/mavros/global_position/raw/fix', self.gps_callback, sensor_qos)
        self.pose_sub = self.node.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, sensor_qos)
        self.vel_sub = self.node.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.vel_callback, sensor_qos)
        
        # Latest sensor data
        self.latest_imu = None
        self.latest_gps = None
        self.latest_pose = None
        self.latest_vel = None
        
        # Sensor data timestamps
        self.imu_timestamp = None
        self.gps_timestamp = None
        self.pose_timestamp = None
        self.vel_timestamp = None
    
    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.latest_imu = {
            'accel': np.array([msg.linear_acceleration.x, 
                              msg.linear_acceleration.y, 
                              msg.linear_acceleration.z]),
            'gyro': np.array([msg.angular_velocity.x,
                             msg.angular_velocity.y,
                             msg.angular_velocity.z]),
            'orientation': np.array([msg.orientation.x,
                                   msg.orientation.y,
                                   msg.orientation.z,
                                   msg.orientation.w])
        }
        self.imu_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    def gps_callback(self, msg):
        """Callback for GPS data"""
        self.latest_gps = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': np.array(msg.position_covariance).reshape(3, 3)
        }
        self.gps_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    def pose_callback(self, msg):
        """Callback for pose data"""
        self.latest_pose = {
            'position': np.array([msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z]),
            'orientation': np.array([msg.pose.orientation.x,
                                   msg.pose.orientation.y,
                                   msg.pose.orientation.z,
                                   msg.pose.orientation.w])
        }
        self.pose_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    def vel_callback(self, msg):
        """Callback for velocity data"""
        self.latest_vel = {
            'linear': np.array([msg.twist.linear.x,
                               msg.twist.linear.y,
                               msg.twist.linear.z]),
            'angular': np.array([msg.twist.angular.x,
                                msg.twist.angular.y,
                                msg.twist.angular.z])
        }
        self.vel_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    def get_latest_sensors(self) -> Dict[str, Any]:
        """Get latest sensor data from Gazebo"""
        sensors = {}
        
        if self.latest_imu is not None:
            sensors['imu'] = np.concatenate([
                self.latest_imu['accel'],
                self.latest_imu['gyro']
            ])
            sensors['accel'] = self.latest_imu['accel']
            sensors['gyro'] = self.latest_imu['gyro']
            sensors['imu_timestamp'] = self.imu_timestamp
        
        if self.latest_gps is not None:
            # Convert GPS to local position (simplified)
            sensors['gps'] = np.array([
                self.latest_gps['longitude'] * 111320 * np.cos(np.deg2rad(self.latest_gps['latitude'])),
                self.latest_gps['latitude'] * 111320,
                self.latest_gps['altitude']
            ])
            sensors['gps_timestamp'] = self.gps_timestamp
        
        if self.latest_pose is not None:
            sensors['pose'] = self.latest_pose['position']
            sensors['orientation'] = self.latest_pose['orientation']
            sensors['pose_timestamp'] = self.pose_timestamp
        
        if self.latest_vel is not None:
            sensors['velocity'] = self.latest_vel['linear']
            sensors['angular_velocity'] = self.latest_vel['angular']
            sensors['vel_timestamp'] = self.vel_timestamp
        
        return sensors
    
    def spin_once(self):
        """Spin ROS2 node once"""
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.001)
    
    def destroy(self):
        """Destroy ROS2 node"""
        self.node.destroy_node()

def create_sensor_model(params: EKFParameters, use_gazebo: bool = False):
    """
    Create sensor model instance.
    
    Args:
        params: EKF parameters
        use_gazebo: If True, use Gazebo sensor interface; otherwise use synthetic model
    
    Returns:
        Sensor model instance
    """
    if use_gazebo:
        return GazeboSensorInterface()
    else:
        return SensorModel(params)

if __name__ == '__main__':
    # Test sensor model
    from ekf_parameters import EKFParameters
    
    params = EKFParameters()
    sensor_model = SensorModel(params)
    
    # Test with sample state
    x = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.05, 0.1, 0.2])
    t = 0.0
    
    sensors = sensor_model.generate_sensors(x, t)
    print("Generated sensors:")
    for key, value in sensors.items():
        if isinstance(value, np.ndarray):
            print(f"{key}: {value}")
        else:
            print(f"{key}: {value}")
    
    # Test with another state
    x2 = x + np.array([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
    t2 = 0.01
    
    sensors2 = sensor_model.generate_sensors(x2, t2)
    print("\nGenerated sensors (second call):")
    for key, value in sensors2.items():
        if isinstance(value, np.ndarray):
            print(f"{key}: {value}")
        else:
            print(f"{key}: {value}")
