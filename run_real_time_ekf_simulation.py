#!/usr/bin/env python3

import sys
import os
sys.path.append('/u/12/akyuret1/unix/drone_sim')

import numpy as np
import matplotlib.pyplot as plt
import time
import json
from datetime import datetime
from typing import Dict, Any, List, Optional

# EKF modules
from ekf_parameters import EKFParameters
from ekf_core import ExtendedKalmanFilter
from ekf_sensor_model import SensorModel
from ekf_dynamics import wrap_to_pi

class RealTimeEKFSimulation:
    """Real-time EKF simulation with realistic sensor data"""
    
    def __init__(self, duration: float = 60.0, dt: float = 0.01):
        self.duration = duration
        self.dt = dt
        self.n_steps = int(duration / dt)
        
        # Initialize EKF components
        self.params = EKFParameters()
        self.ekf = ExtendedKalmanFilter(self.params)
        self.sensor_model = SensorModel(self.params)
        
        # Data storage
        self.timestamps = []
        self.true_states = []
        self.estimated_states = []
        self.covariances = []
        self.sensor_measurements = []
        self.innovation_stats = []
        
        # Performance metrics
        self.position_errors = []
        self.velocity_errors = []
        self.attitude_errors = []
        
        print(f"🚁 Real-time EKF Simulation initialized")
        print(f"   Duration: {duration} seconds")
        print(f"   Sample rate: {1/dt} Hz")
        print(f"   Total steps: {self.n_steps}")
        print(f"   Drone profile: {self.params.profile}")
    
    def generate_realistic_trajectory(self) -> np.ndarray:
        """Generate a realistic drone flight trajectory"""
        t = np.linspace(0, self.duration, self.n_steps)
        
        # Create a complex 3D trajectory
        x_true = np.zeros((self.n_steps, 9))
        
        # Position: Figure-8 pattern with altitude changes
        x_true[:, 0] = 10 * np.sin(0.3 * t)  # X: Figure-8
        x_true[:, 1] = 10 * np.sin(0.6 * t)  # Y: Figure-8
        x_true[:, 2] = 5 + 3 * np.sin(0.2 * t)  # Z: Altitude variation
        
        # Velocity: Derivative of position
        x_true[:, 3] = 10 * 0.3 * np.cos(0.3 * t)  # Vx
        x_true[:, 4] = 10 * 0.6 * np.cos(0.6 * t)  # Vy
        x_true[:, 5] = 3 * 0.2 * np.cos(0.2 * t)   # Vz
        
        # Attitude: Realistic drone attitude changes
        x_true[:, 6] = 0.1 * np.sin(0.4 * t)  # Roll
        x_true[:, 7] = 0.1 * np.cos(0.4 * t)  # Pitch
        x_true[:, 8] = 0.1 * t  # Yaw (slow rotation)
        
        return x_true
    
    def run_simulation(self):
        """Run the real-time EKF simulation"""
        print("\n🎯 Starting real-time EKF simulation...")
        
        # Generate true trajectory
        x_true = self.generate_realistic_trajectory()
        
        # Initialize EKF
        self.ekf.initialize(x_true[0])
        print("✅ EKF initialized with true initial state")
        
        # Run simulation
        start_time = time.time()
        
        for i in range(self.n_steps):
            current_time = i * self.dt
            
            # Generate sensor measurements
            sensors = self.sensor_model.generate_sensors(x_true[i], current_time)
            
            # EKF step
            x_est, P = self.ekf.step(
                sensors['imu'], self.dt,
                gps_meas=sensors['gps'],
                baro_meas=sensors['baro'],
                mag_meas=sensors['mag'],
                use_accel_tilt=True
            )
            
            # Store data
            self.timestamps.append(current_time)
            self.true_states.append(x_true[i].tolist())
            self.estimated_states.append(x_est.tolist())
            self.covariances.append(P.tolist())
            self.sensor_measurements.append({
                'imu': sensors['imu'].tolist(),
                'gps': sensors['gps'].tolist(),
                'baro': sensors['baro'],
                'mag': sensors['mag']
            })
            
            # Calculate errors
            pos_error = np.linalg.norm(x_est[0:3] - x_true[i, 0:3])
            vel_error = np.linalg.norm(x_est[3:6] - x_true[i, 3:6])
            att_error = np.linalg.norm(x_est[6:9] - x_true[i, 6:9])
            
            self.position_errors.append(pos_error)
            self.velocity_errors.append(vel_error)
            self.attitude_errors.append(att_error)
            
            # Store innovation statistics
            if i % 100 == 0:  # Every second
                stats = self.ekf.get_innovation_stats()
                self.innovation_stats.append(stats)
            
            # Real-time display
            if i % 1000 == 0:  # Every 10 seconds
                elapsed = time.time() - start_time
                progress = (i / self.n_steps) * 100
                print(f"⏱️  Progress: {progress:.1f}% | Time: {current_time:.1f}s | "
                      f"Pos Error: {pos_error:.3f}m | Vel Error: {vel_error:.3f}m/s")
        
        total_time = time.time() - start_time
        print(f"\n✅ Simulation completed in {total_time:.2f} seconds")
        print(f"   Real-time factor: {self.duration/total_time:.2f}x")
    
    def analyze_performance(self):
        """Analyze EKF performance"""
        print("\n📊 Analyzing EKF performance...")
        
        # Convert to numpy arrays
        true_states = np.array(self.true_states)
        est_states = np.array(self.estimated_states)
        covariances = np.array(self.covariances)
        
        # Calculate performance metrics
        pos_rmse = np.sqrt(np.mean(self.position_errors))
        vel_rmse = np.sqrt(np.mean(self.velocity_errors))
        att_rmse = np.sqrt(np.mean(self.attitude_errors))
        
        pos_max_error = np.max(self.position_errors)
        vel_max_error = np.max(self.velocity_errors)
        att_max_error = np.max(self.attitude_errors)
        
        # Calculate uncertainty metrics
        pos_uncertainty = np.sqrt(np.diagonal(covariances[:, 0:3, 0:3], axis1=1, axis2=2))
        vel_uncertainty = np.sqrt(np.diagonal(covariances[:, 3:6, 3:6], axis1=1, axis2=2))
        att_uncertainty = np.sqrt(np.diagonal(covariances[:, 6:9, 6:9], axis1=1, axis2=2))
        
        avg_pos_uncertainty = np.mean(np.linalg.norm(pos_uncertainty, axis=1))
        avg_vel_uncertainty = np.mean(np.linalg.norm(vel_uncertainty, axis=1))
        avg_att_uncertainty = np.mean(np.linalg.norm(att_uncertainty, axis=1))
        
        # Print results
        print(f"\n🎯 PERFORMANCE METRICS:")
        print(f"   Position RMSE: {pos_rmse:.3f} m")
        print(f"   Velocity RMSE: {vel_rmse:.3f} m/s")
        print(f"   Attitude RMSE: {att_rmse:.3f} rad ({np.rad2deg(att_rmse):.1f}°)")
        print(f"\n   Max Position Error: {pos_max_error:.3f} m")
        print(f"   Max Velocity Error: {vel_max_error:.3f} m/s")
        print(f"   Max Attitude Error: {att_max_error:.3f} rad ({np.rad2deg(att_max_error):.1f}°)")
        
        print(f"\n📈 UNCERTAINTY METRICS:")
        print(f"   Avg Position Uncertainty: {avg_pos_uncertainty:.3f} m")
        print(f"   Avg Velocity Uncertainty: {avg_vel_uncertainty:.3f} m/s")
        print(f"   Avg Attitude Uncertainty: {avg_att_uncertainty:.3f} rad ({np.rad2deg(avg_att_uncertainty):.1f}°)")
        
        # Performance assessment
        pos_perf = "EXCELLENT" if pos_rmse < 0.1 else "GOOD" if pos_rmse < 0.5 else "FAIR"
        vel_perf = "EXCELLENT" if vel_rmse < 0.05 else "GOOD" if vel_rmse < 0.2 else "FAIR"
        att_perf = "EXCELLENT" if att_rmse < 0.05 else "GOOD" if att_rmse < 0.1 else "FAIR"
        
        print(f"\n🏆 PERFORMANCE ASSESSMENT:")
        print(f"   Position Estimation: {pos_perf}")
        print(f"   Velocity Estimation: {vel_perf}")
        print(f"   Attitude Estimation: {att_perf}")
        
        return {
            'position_rmse': pos_rmse,
            'velocity_rmse': vel_rmse,
            'attitude_rmse': att_rmse,
            'position_max_error': pos_max_error,
            'velocity_max_error': vel_max_error,
            'attitude_max_error': att_max_error,
            'avg_position_uncertainty': avg_pos_uncertainty,
            'avg_velocity_uncertainty': avg_vel_uncertainty,
            'avg_attitude_uncertainty': avg_att_uncertainty
        }
    
    def plot_results(self):
        """Plot simulation results"""
        print("\n📊 Generating performance plots...")
        
        # Convert to numpy arrays
        timestamps = np.array(self.timestamps)
        true_states = np.array(self.true_states)
        est_states = np.array(self.estimated_states)
        covariances = np.array(self.covariances)
        
        # Create comprehensive plots
        fig, axes = plt.subplots(3, 3, figsize=(18, 12))
        
        # Position plots
        axes[0, 0].plot(timestamps, true_states[:, 0], 'b-', linewidth=2, label='True')
        axes[0, 0].plot(timestamps, est_states[:, 0], 'r--', linewidth=2, label='EKF')
        axes[0, 0].set_title('X Position (m)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        axes[0, 1].plot(timestamps, true_states[:, 1], 'b-', linewidth=2, label='True')
        axes[0, 1].plot(timestamps, est_states[:, 1], 'r--', linewidth=2, label='EKF')
        axes[0, 1].set_title('Y Position (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        axes[0, 2].plot(timestamps, true_states[:, 2], 'b-', linewidth=2, label='True')
        axes[0, 2].plot(timestamps, est_states[:, 2], 'r--', linewidth=2, label='EKF')
        axes[0, 2].set_title('Z Position (m)')
        axes[0, 2].legend()
        axes[0, 2].grid(True)
        
        # Velocity plots
        axes[1, 0].plot(timestamps, true_states[:, 3], 'b-', linewidth=2, label='True')
        axes[1, 0].plot(timestamps, est_states[:, 3], 'r--', linewidth=2, label='EKF')
        axes[1, 0].set_title('X Velocity (m/s)')
        axes[1, 0].set_ylabel('Velocity (m/s)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        axes[1, 1].plot(timestamps, true_states[:, 4], 'b-', linewidth=2, label='True')
        axes[1, 1].plot(timestamps, est_states[:, 4], 'r--', linewidth=2, label='EKF')
        axes[1, 1].set_title('Y Velocity (m/s)')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        axes[1, 2].plot(timestamps, true_states[:, 5], 'b-', linewidth=2, label='True')
        axes[1, 2].plot(timestamps, est_states[:, 5], 'r--', linewidth=2, label='EKF')
        axes[1, 2].set_title('Z Velocity (m/s)')
        axes[1, 2].legend()
        axes[1, 2].grid(True)
        
        # Attitude plots
        axes[2, 0].plot(timestamps, np.rad2deg(true_states[:, 6]), 'b-', linewidth=2, label='True')
        axes[2, 0].plot(timestamps, np.rad2deg(est_states[:, 6]), 'r--', linewidth=2, label='EKF')
        axes[2, 0].set_title('Roll (deg)')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_ylabel('Angle (deg)')
        axes[2, 0].legend()
        axes[2, 0].grid(True)
        
        axes[2, 1].plot(timestamps, np.rad2deg(true_states[:, 7]), 'b-', linewidth=2, label='True')
        axes[2, 1].plot(timestamps, np.rad2deg(est_states[:, 7]), 'r--', linewidth=2, label='EKF')
        axes[2, 1].set_title('Pitch (deg)')
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].legend()
        axes[2, 1].grid(True)
        
        axes[2, 2].plot(timestamps, np.rad2deg(true_states[:, 8]), 'b-', linewidth=2, label='True')
        axes[2, 2].plot(timestamps, np.rad2deg(est_states[:, 8]), 'r--', linewidth=2, label='EKF')
        axes[2, 2].set_title('Yaw (deg)')
        axes[2, 2].set_xlabel('Time (s)')
        axes[2, 2].legend()
        axes[2, 2].grid(True)
        
        plt.tight_layout()
        plt.savefig('/workspace/real_time_ekf_performance.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # Error analysis plot
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        axes[0].plot(timestamps, self.position_errors)
        axes[0].set_title('Position Error (m)')
        axes[0].set_ylabel('Error (m)')
        axes[0].grid(True)
        
        axes[1].plot(timestamps, self.velocity_errors)
        axes[1].set_title('Velocity Error (m/s)')
        axes[1].set_ylabel('Error (m/s)')
        axes[1].grid(True)
        
        axes[2].plot(timestamps, np.rad2deg(self.attitude_errors))
        axes[2].set_title('Attitude Error (deg)')
        axes[2].set_ylabel('Error (deg)')
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.savefig('/workspace/real_time_ekf_errors.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # 3D trajectory plot
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.plot(true_states[:, 0], true_states[:, 1], true_states[:, 2], 
                'b-', linewidth=2, label='True Trajectory')
        ax.plot(est_states[:, 0], est_states[:, 1], est_states[:, 2], 
                'r--', linewidth=2, label='EKF Estimate')
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('3D Trajectory Comparison')
        ax.legend()
        ax.grid(True)
        
        plt.savefig('/workspace/real_time_ekf_3d_trajectory.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        print("✅ Plots saved:")
        print("   - real_time_ekf_performance.png")
        print("   - real_time_ekf_errors.png")
        print("   - real_time_ekf_3d_trajectory.png")
    
    def save_data(self):
        """Save simulation data for analysis"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'/workspace/real_time_ekf_log_{timestamp}.json'
        
        data = {
            'timestamps': self.timestamps,
            'true_states': self.true_states,
            'estimated_states': self.estimated_states,
            'covariances': self.covariances,
            'sensor_measurements': self.sensor_measurements,
            'innovation_stats': self.innovation_stats,
            'position_errors': self.position_errors,
            'velocity_errors': self.velocity_errors,
            'attitude_errors': self.attitude_errors,
            'simulation_params': {
                'duration': self.duration,
                'dt': self.dt,
                'n_steps': self.n_steps,
                'drone_profile': self.params.profile
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"✅ Simulation data saved to: {filename}")
        return filename
    
    def run_complete_simulation(self):
        """Run complete real-time EKF simulation with analysis"""
        print("🚁 REAL-TIME EKF SIMULATION")
        print("=" * 50)
        
        # Run simulation
        self.run_simulation()
        
        # Analyze performance
        metrics = self.analyze_performance()
        
        # Generate plots
        self.plot_results()
        
        # Save data
        log_file = self.save_data()
        
        print(f"\n🎉 Real-time EKF simulation completed successfully!")
        print(f"📁 Log file: {log_file}")
        print(f"📊 Performance plots generated")
        
        return metrics, log_file

def main():
    """Main function"""
    # Create and run simulation
    simulation = RealTimeEKFSimulation(duration=60.0, dt=0.01)
    metrics, log_file = simulation.run_complete_simulation()
    
    return metrics, log_file

if __name__ == '__main__':
    main()
