#!/usr/bin/env python3
"""
Standalone EKF Demonstration
Simulates autonomous drone flight with real-time EKF processing
This demonstrates the EKF system without requiring Gazebo or ROS
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import json
from datetime import datetime
from typing import Tuple, Dict, Any

# Import our EKF modules
from ekf_core import ExtendedKalmanFilter
from ekf_parameters import EKFParameters
from ekf_sensor_model import SensorModel
from ekf_dynamics import wrap_to_pi

class StandaloneDroneSimulation:
    """
    Standalone drone simulation with EKF integration
    Simulates realistic drone dynamics and sensor data
    """
    
    def __init__(self, duration: float = 60.0, dt: float = 0.01):
        self.duration = duration
        self.dt = dt
        self.n_steps = int(duration / dt)
        
        # Drone parameters
        self.mass = 1.5  # kg
        self.g = 9.81   # m/s^2
        
        # Initialize EKF
        self.ekf_params = EKFParameters()
        self.ekf = ExtendedKalmanFilter(self.ekf_params)
        self.sensor_model = SensorModel(self.ekf_params)
        
        # Flight trajectory parameters
        self.trajectory_scale = 5.0
        self.trajectory_height = 3.0
        self.trajectory_period = 30.0
        
        # Data storage
        self.timestamps = []
        self.true_states = []
        self.estimated_states = []
        self.sensor_data = []
        self.performance_metrics = []
        
        print("🚁 Standalone EKF Drone Simulation")
        print("==================================")
        print(f"Duration: {duration}s | Sample Rate: {1/dt}Hz | Steps: {self.n_steps}")
        print(f"Trajectory: Figure-8 | Scale: {self.trajectory_scale}m | Height: {self.trajectory_height}m")
    
    def generate_figure8_trajectory(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generate figure-8 trajectory with smooth transitions"""
        omega = 2 * np.pi / self.trajectory_period
        
        # Figure-8 trajectory
        x = self.trajectory_scale * np.sin(omega * t)
        y = self.trajectory_scale * np.sin(2 * omega * t) / 2
        z = self.trajectory_height
        
        # Velocities
        vx = self.trajectory_scale * omega * np.cos(omega * t)
        vy = self.trajectory_scale * omega * np.cos(2 * omega * t)
        vz = 0.0
        
        # Accelerations
        ax = -self.trajectory_scale * omega**2 * np.sin(omega * t)
        ay = -2 * self.trajectory_scale * omega**2 * np.sin(2 * omega * t)
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def simulate_drone_dynamics(self, pos: np.ndarray, vel: np.ndarray, 
                               att: np.ndarray, motor_commands: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Simulate realistic drone dynamics"""
        # Extract attitude
        roll, pitch, yaw = att
        
        # Rotation matrix (body to world)
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R_bw = np.array([
            [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
            [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
            [-sp,   sr*cp,            cr*cp]
        ])
        
        # Total thrust from motors
        total_thrust = np.sum(motor_commands) * 15.0  # Simplified thrust model
        thrust_body = np.array([0, 0, total_thrust])
        thrust_world = R_bw @ thrust_body
        
        # Acceleration in world frame
        gravity = np.array([0, 0, -self.g])
        acc = thrust_world / self.mass + gravity
        
        # Add some dynamics noise
        acc += np.random.normal(0, 0.1, 3)
        
        # Integrate motion
        new_vel = vel + acc * dt
        new_pos = pos + vel * dt + 0.5 * acc * dt**2
        
        # Simplified attitude dynamics (for demonstration)
        # In reality, this would be much more complex
        roll_torque = (motor_commands[2] + motor_commands[0] - motor_commands[1] - motor_commands[3]) * 0.1
        pitch_torque = (motor_commands[2] + motor_commands[3] - motor_commands[0] - motor_commands[1]) * 0.1
        yaw_torque = (motor_commands[1] + motor_commands[2] - motor_commands[0] - motor_commands[3]) * 0.05
        
        # Angular acceleration (simplified)
        angular_acc = np.array([roll_torque, pitch_torque, yaw_torque])
        angular_acc += np.random.normal(0, 0.01, 3)  # Dynamics noise
        
        # Integrate attitude (simplified)
        new_att = att + angular_acc * dt
        new_att[2] = wrap_to_pi(new_att[2])  # Wrap yaw
        
        return new_pos, new_vel, new_att
    
    def generate_sensor_measurements(self, pos: np.ndarray, vel: np.ndarray, 
                                   att: np.ndarray, acc: np.ndarray) -> Dict[str, np.ndarray]:
        """Generate realistic sensor measurements with noise"""
        # IMU measurements
        roll, pitch, yaw = att
        
        # Rotation matrix (world to body)
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R_wb = np.array([
            [cp*cy, cp*sy, -sp],
            [sr*sp*cy - cr*sy, sr*sp*sy + cr*cy, sr*cp],
            [cr*sp*cy + sr*sy, cr*sp*sy - sr*cy, cr*cp]
        ])
        
        # Accelerometer (specific force in body frame)
        gravity_world = np.array([0, 0, -self.g])
        specific_force_world = acc - gravity_world
        accel_body = R_wb @ specific_force_world
        
        # Add IMU noise
        accel_noise = np.random.normal(0, 0.02, 3)
        gyro_noise = np.random.normal(0, 0.001, 3)
        
        imu = np.concatenate([
            accel_body + accel_noise,
            np.random.normal(0, 0.01, 3) + gyro_noise  # Simplified gyro
        ])
        
        # GPS measurements (with noise)
        gps_noise = np.random.normal(0, 1.0, 3)
        gps = pos + gps_noise
        
        # Barometer (altitude with noise)
        baro_noise = np.random.normal(0, 0.1)
        baro = -pos[2] + baro_noise  # NED convention: altitude = -z
        
        # Magnetometer (yaw with noise)
        mag_noise = np.random.normal(0, np.deg2rad(2))
        mag = yaw + mag_noise
        
        return {
            'imu': imu,
            'gps': gps,
            'baro': baro,
            'mag': mag
        }
    
    def compute_motor_commands(self, pos_des: np.ndarray, vel_des: np.ndarray,
                              acc_des: np.ndarray, pos_curr: np.ndarray,
                              vel_curr: np.ndarray, att_curr: np.ndarray) -> np.ndarray:
        """Compute motor commands for trajectory following"""
        # Control gains
        kp_pos = 2.0
        kd_pos = 1.0
        kp_att = 3.0
        
        # Position control
        pos_error = pos_des - pos_curr
        vel_error = vel_des - vel_curr
        
        # Desired acceleration
        acc_cmd = acc_des + kp_pos * pos_error + kd_pos * vel_error
        acc_cmd[2] += self.g  # Add gravity compensation
        
        # Convert to thrust and attitude
        thrust_magnitude = np.linalg.norm(acc_cmd)
        thrust_normalized = np.clip(thrust_magnitude / (self.mass * self.g), 0.2, 2.0)
        
        # Desired attitude from acceleration command
        if thrust_magnitude > 1e-6:
            roll_des = np.arcsin(np.clip(acc_cmd[1] / thrust_magnitude, -1, 1))
            pitch_des = -np.arcsin(np.clip(acc_cmd[0] / thrust_magnitude, -1, 1))
        else:
            roll_des = 0.0
            pitch_des = 0.0
        yaw_des = 0.0
        
        # Attitude control
        roll_error = roll_des - att_curr[0]
        pitch_error = pitch_des - att_curr[1]
        yaw_error = wrap_to_pi(yaw_des - att_curr[2])
        
        # Motor mixing (quadrotor X configuration)
        base_thrust = thrust_normalized / 4
        roll_correction = kp_att * roll_error * 0.1
        pitch_correction = kp_att * pitch_error * 0.1
        yaw_correction = kp_att * yaw_error * 0.05
        
        motor_speeds = np.array([
            base_thrust + roll_correction - pitch_correction - yaw_correction,  # Front left
            base_thrust - roll_correction - pitch_correction + yaw_correction,  # Front right
            base_thrust + roll_correction + pitch_correction + yaw_correction,  # Rear left
            base_thrust - roll_correction + pitch_correction - yaw_correction   # Rear right
        ])
        
        return np.clip(motor_speeds, 0.1, 1.0)
    
    def run_simulation(self):
        """Run the complete simulation with EKF processing"""
        print("\n🚀 Starting autonomous flight simulation with EKF...")
        
        # Initialize states
        pos = np.array([0.0, 0.0, 0.0])  # Start at origin
        vel = np.array([0.0, 0.0, 0.0])
        att = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # Initialize EKF
        x0 = np.concatenate([pos, vel, att])
        self.ekf.initialize(x0)
        
        start_time = time.time()
        
        for step in range(self.n_steps):
            t = step * self.dt
            
            # Generate desired trajectory
            pos_des, vel_des, acc_des = self.generate_figure8_trajectory(t)
            
            # Compute motor commands
            motor_commands = self.compute_motor_commands(pos_des, vel_des, acc_des, pos, vel, att)
            
            # Simulate drone dynamics
            pos, vel, att = self.simulate_drone_dynamics(pos, vel, att, motor_commands, self.dt)
            
            # Generate sensor measurements
            acc_world = (vel - (self.timestamps[-1] if self.timestamps else vel)) / self.dt if self.timestamps else np.zeros(3)
            sensors = self.generate_sensor_measurements(pos, vel, att, acc_world)
            
            # EKF processing
            ekf_state, ekf_cov = self.ekf.step(
                sensors['imu'], self.dt,
                gps_meas=sensors['gps'],
                baro_meas=sensors['baro'],
                mag_meas=sensors['mag']
            )
            
            # Store data
            true_state = np.concatenate([pos, vel, att])
            self.timestamps.append(t)
            self.true_states.append(true_state)
            self.estimated_states.append(ekf_state)
            self.sensor_data.append(sensors)
            
            # Progress update
            if step % 1000 == 0:
                pos_error = np.linalg.norm(ekf_state[:3] - true_state[:3])
                vel_error = np.linalg.norm(ekf_state[3:6] - true_state[3:6])
                progress = (step / self.n_steps) * 100
                print(f"⏱️  Progress: {progress:.1f}% | Time: {t:.1f}s | Pos Error: {pos_error:.3f}m | Vel Error: {vel_error:.3f}m/s")
        
        simulation_time = time.time() - start_time
        real_time_factor = self.duration / simulation_time
        
        print(f"\n✅ Simulation completed in {simulation_time:.2f} seconds")
        print(f"   Real-time factor: {real_time_factor:.2f}x")
        
        return self.analyze_performance()
    
    def analyze_performance(self) -> Dict[str, Any]:
        """Analyze EKF performance"""
        print("\n📊 Analyzing EKF performance...")
        
        # Convert to numpy arrays
        true_states = np.array(self.true_states)
        estimated_states = np.array(self.estimated_states)
        
        # Calculate errors
        pos_errors = np.linalg.norm(estimated_states[:, :3] - true_states[:, :3], axis=1)
        vel_errors = np.linalg.norm(estimated_states[:, 3:6] - true_states[:, 3:6], axis=1)
        att_errors = np.linalg.norm(estimated_states[:, 6:9] - true_states[:, 6:9], axis=1)
        
        # Performance metrics
        metrics = {
            'position_rmse': np.sqrt(np.mean(pos_errors**2)),
            'velocity_rmse': np.sqrt(np.mean(vel_errors**2)),
            'attitude_rmse': np.sqrt(np.mean(att_errors**2)),
            'max_pos_error': np.max(pos_errors),
            'max_vel_error': np.max(vel_errors),
            'max_att_error': np.max(att_errors),
            'avg_pos_error': np.mean(pos_errors),
            'avg_vel_error': np.mean(vel_errors),
            'avg_att_error': np.mean(att_errors)
        }
        
        print(f"\n🎯 PERFORMANCE METRICS:")
        print(f"   Position RMSE: {metrics['position_rmse']:.3f} m")
        print(f"   Velocity RMSE: {metrics['velocity_rmse']:.3f} m/s")
        print(f"   Attitude RMSE: {metrics['attitude_rmse']:.3f} rad ({np.rad2deg(metrics['attitude_rmse']):.1f}°)")
        print(f"   Max Position Error: {metrics['max_pos_error']:.3f} m")
        print(f"   Max Velocity Error: {metrics['max_vel_error']:.3f} m/s")
        print(f"   Max Attitude Error: {metrics['max_att_error']:.3f} rad ({np.rad2deg(metrics['max_att_error']):.1f}°)")
        
        return metrics
    
    def generate_plots(self):
        """Generate comprehensive analysis plots"""
        print("\n📈 Generating analysis plots...")
        
        # Convert to numpy arrays
        true_states = np.array(self.true_states)
        estimated_states = np.array(self.estimated_states)
        times = np.array(self.timestamps)
        
        # Create comprehensive plot
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('Standalone EKF Drone Simulation Analysis', fontsize=16)
        
        # Position comparison
        axes[0, 0].plot(times, true_states[:, 0], 'b-', label='True X', linewidth=2)
        axes[0, 0].plot(times, estimated_states[:, 0], 'r--', label='EKF X', alpha=0.8)
        axes[0, 0].plot(times, true_states[:, 1], 'g-', label='True Y', linewidth=2)
        axes[0, 0].plot(times, estimated_states[:, 1], 'm--', label='EKF Y', alpha=0.8)
        axes[0, 0].set_title('Position Estimation')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Velocity comparison
        axes[0, 1].plot(times, true_states[:, 3], 'b-', label='True Vx', linewidth=2)
        axes[0, 1].plot(times, estimated_states[:, 3], 'r--', label='EKF Vx', alpha=0.8)
        axes[0, 1].plot(times, true_states[:, 4], 'g-', label='True Vy', linewidth=2)
        axes[0, 1].plot(times, estimated_states[:, 4], 'm--', label='EKF Vy', alpha=0.8)
        axes[0, 1].set_title('Velocity Estimation')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Velocity (m/s)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # Attitude comparison
        axes[0, 2].plot(times, np.rad2deg(true_states[:, 6]), 'b-', label='True Roll', linewidth=2)
        axes[0, 2].plot(times, np.rad2deg(estimated_states[:, 6]), 'r--', label='EKF Roll', alpha=0.8)
        axes[0, 2].plot(times, np.rad2deg(true_states[:, 7]), 'g-', label='True Pitch', linewidth=2)
        axes[0, 2].plot(times, np.rad2deg(estimated_states[:, 7]), 'm--', label='EKF Pitch', alpha=0.8)
        axes[0, 2].set_title('Attitude Estimation')
        axes[0, 2].set_xlabel('Time (s)')
        axes[0, 2].set_ylabel('Angle (degrees)')
        axes[0, 2].legend()
        axes[0, 2].grid(True)
        
        # Position errors
        pos_errors = np.linalg.norm(estimated_states[:, :3] - true_states[:, :3], axis=1)
        axes[1, 0].plot(times, pos_errors, 'r-', linewidth=2)
        axes[1, 0].set_title('Position Error')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Error (m)')
        axes[1, 0].grid(True)
        
        # Velocity errors
        vel_errors = np.linalg.norm(estimated_states[:, 3:6] - true_states[:, 3:6], axis=1)
        axes[1, 1].plot(times, vel_errors, 'g-', linewidth=2)
        axes[1, 1].set_title('Velocity Error')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Error (m/s)')
        axes[1, 1].grid(True)
        
        # 3D trajectory
        axes[1, 2].remove()  # Remove 2D subplot
        ax_3d = fig.add_subplot(2, 3, 6, projection='3d')
        ax_3d.plot(true_states[:, 0], true_states[:, 1], true_states[:, 2], 'b-', linewidth=3, label='True Trajectory')
        ax_3d.plot(estimated_states[:, 0], estimated_states[:, 1], estimated_states[:, 2], 'r--', linewidth=2, alpha=0.8, label='EKF Estimate')
        ax_3d.scatter(true_states[0, 0], true_states[0, 1], true_states[0, 2], color='green', s=100, label='Start')
        ax_3d.scatter(true_states[-1, 0], true_states[-1, 1], true_states[-1, 2], color='red', s=100, label='End')
        ax_3d.set_xlabel('X (m)')
        ax_3d.set_ylabel('Y (m)')
        ax_3d.set_zlabel('Z (m)')
        ax_3d.set_title('3D Flight Trajectory')
        ax_3d.legend()
        
        plt.tight_layout()
        plt.savefig('/workspace/standalone_ekf_analysis.png', dpi=300, bbox_inches='tight')
        plt.show(block=False)
        plt.pause(2)  # Show for 2 seconds
        plt.close()
        
        print("✅ Analysis plots saved to: standalone_ekf_analysis.png")
    
    def save_results(self, metrics: Dict[str, Any]):
        """Save simulation results"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save flight data
        flight_data = {
            'metadata': {
                'timestamp': timestamp,
                'duration': self.duration,
                'dt': self.dt,
                'trajectory_type': 'figure8',
                'ekf_parameters': {
                    'Q_vel': self.ekf_params.Q_vel,
                    'Q_att': self.ekf_params.Q_att,
                    'profile': self.ekf_params.profile
                }
            },
            'timestamps': self.timestamps,
            'true_states': [state.tolist() for state in self.true_states],
            'estimated_states': [state.tolist() for state in self.estimated_states],
            'sensor_data': self.sensor_data,
            'performance_metrics': metrics
        }
        
        filename = f'/workspace/standalone_ekf_demo_{timestamp}.json'
        with open(filename, 'w') as f:
            json.dump(flight_data, f, indent=2)
        
        print(f"✅ Simulation data saved to: {filename}")
        return filename

def main():
    """Main simulation function"""
    print("🚁 STANDALONE EKF DRONE SIMULATION")
    print("==================================")
    print("This demonstrates the autonomous drone EKF system")
    print("without requiring Gazebo or ROS dependencies")
    print()
    
    # Create simulation
    simulation = StandaloneDroneSimulation(duration=30.0, dt=0.01)
    
    try:
        # Run simulation
        metrics = simulation.run_simulation()
        
        # Generate plots
        simulation.generate_plots()
        
        # Save results
        filename = simulation.save_results(metrics)
        
        print("\n🎉 SIMULATION COMPLETE!")
        print("======================")
        print(f"📁 Data saved to: {filename}")
        print(f"📊 Plots saved to: standalone_ekf_analysis.png")
        print("\nThe simulation demonstrates:")
        print("✅ Autonomous figure-8 flight trajectory")
        print("✅ Real-time EKF state estimation")
        print("✅ Sensor fusion (IMU, GPS, Barometer, Magnetometer)")
        print("✅ Performance analysis and visualization")
        
    except Exception as e:
        print(f"❌ Simulation error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()