#!/usr/bin/env python3
"""
Waypoint-based EKF simulation with FIXED magnetometer model
This version uses the improved magnetometer measurement model to reduce innovation warnings.
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import json
from datetime import datetime
from typing import Dict, Any, List, Optional

# Import improved EKF with fixed magnetometer model
from fix_magnetometer_model import ImprovedMagnetometerEKF, create_improved_ekf
from ekf_sensor_model import SensorModel
from ekf_dynamics import wrap_to_pi

class WaypointBasedEKFFixedMag:
    """Waypoint-based EKF simulation with fixed magnetometer model"""
    
    def __init__(self, duration: float = 120.0, dt: float = 0.01):
        self.duration = duration
        self.dt = dt
        self.n_steps = int(duration / dt)
        
        # Initialize improved EKF with fixed magnetometer model
        self.ekf, self.params = create_improved_ekf()
        self.sensor_model = SensorModel(self.params)
        
        # Generate waypoint mission
        self.waypoints = self.generate_figure8_waypoints()
        
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
        
        print(f"🚁 Waypoint-Based EKF with FIXED Magnetometer Model")
        print(f"==================================================")
        print(f"✅ Improved magnetometer measurement model")
        print(f"✅ 30° innovation gate (increased from 15°)")
        print(f"✅ Realistic magnetic field modeling")
        print(f"✅ {len(self.waypoints)} waypoints planned")
    
    def generate_figure8_waypoints(self):
        """Generate a figure-8 waypoint mission with altitude variation"""
        waypoints = []
        
        # Starting position
        waypoints.append({'x': 0, 'y': 0, 'z': 5, 'yaw': 0, 'hold_time': 2.0})
        
        # Figure-8 pattern waypoints
        t_points = np.linspace(0, 4*np.pi, 16)  # 16 waypoints for smooth figure-8
        
        for t in t_points:
            x = 8 * np.sin(t)  # Figure-8 X component (8m amplitude)
            y = 4 * np.sin(2*t)  # Figure-8 Y component (4m amplitude)
            z = 5 + 2 * np.sin(t/2)  # Altitude variation (3-7m)
            yaw = np.arctan2(np.cos(2*t), np.cos(t))  # Point in direction of travel
            
            waypoints.append({
                'x': x, 'y': y, 'z': z, 'yaw': yaw, 
                'hold_time': 0.5 + 0.5 * np.random.rand()  # Random hold time
            })
        
        # Return to start
        waypoints.append({'x': 0, 'y': 0, 'z': 5, 'yaw': 0, 'hold_time': 2.0})
        
        return waypoints
    
    def generate_realistic_waypoint_trajectory(self):
        """Generate realistic trajectory following waypoints with proper dynamics"""
        x_true = np.zeros((self.n_steps, 9))
        
        # Calculate waypoint timing
        total_hold_time = sum(wp['hold_time'] for wp in self.waypoints)
        waypoint_times = []
        current_time = 0
        
        for wp in self.waypoints:
            current_time += wp['hold_time']
            waypoint_times.append(current_time)
        
        # Normalize to simulation duration
        time_scale = self.duration / total_hold_time
        waypoint_times = [t * time_scale for t in waypoint_times]
        
        print(f"🚁 Autonomous flight following realistic waypoint mission")
        
        for i in range(self.n_steps):
            current_time = i * self.dt
            
            # Find current waypoint segment
            wp_idx = 0
            for j, wp_time in enumerate(waypoint_times):
                if current_time <= wp_time:
                    wp_idx = j
                    break
            
            if wp_idx == 0:
                # Before first waypoint
                wp1 = self.waypoints[0]
                x_true[i, 0] = wp1['x']
                x_true[i, 1] = wp1['y'] 
                x_true[i, 2] = wp1['z']
                x_true[i, 8] = wp1['yaw']
            elif wp_idx >= len(waypoint_times) - 1:
                # After last waypoint
                wp1 = self.waypoints[-1]
                x_true[i, 0] = wp1['x']
                x_true[i, 1] = wp1['y']
                x_true[i, 2] = wp1['z']
                x_true[i, 8] = wp1['yaw']
            else:
                # Interpolate between waypoints
                wp1 = self.waypoints[wp_idx]
                wp2 = self.waypoints[wp_idx + 1]
                
                t1 = waypoint_times[wp_idx - 1] if wp_idx > 0 else 0
                t2 = waypoint_times[wp_idx]
                
                if t2 > t1:
                    alpha = (current_time - t1) / (t2 - t1)
                    alpha = np.clip(alpha, 0, 1)
                    
                    # Smooth interpolation using cubic spline-like curve
                    alpha_smooth = 3*alpha**2 - 2*alpha**3
                    
                    x_true[i, 0] = wp1['x'] + alpha_smooth * (wp2['x'] - wp1['x'])
                    x_true[i, 1] = wp1['y'] + alpha_smooth * (wp2['y'] - wp1['y'])
                    x_true[i, 2] = wp1['z'] + alpha_smooth * (wp2['z'] - wp1['z'])
                    x_true[i, 8] = wp1['yaw'] + alpha_smooth * wrap_to_pi(wp2['yaw'] - wp1['yaw'])
                else:
                    x_true[i, 0] = wp1['x']
                    x_true[i, 1] = wp1['y']
                    x_true[i, 2] = wp1['z']
                    x_true[i, 8] = wp1['yaw']
            
            # Generate realistic velocities and accelerations
            if i > 0:
                dt = self.dt
                # Velocity from position difference
                x_true[i, 3] = (x_true[i, 0] - x_true[i-1, 0]) / dt
                x_true[i, 4] = (x_true[i, 1] - x_true[i-1, 1]) / dt
                x_true[i, 5] = (x_true[i, 2] - x_true[i-1, 2]) / dt
                
                # Attitude rates from attitude difference
                x_true[i, 6] = wrap_to_pi(x_true[i, 8] - x_true[i-1, 8]) / dt  # Roll rate
                x_true[i, 7] = 0.1 * np.sin(0.1 * current_time)  # Pitch rate
                x_true[i, 8] = wrap_to_pi(x_true[i, 8] - x_true[i-1, 8]) / dt  # Yaw rate
        
        return x_true
    
    def run_simulation(self):
        """Run the waypoint-based EKF simulation with fixed magnetometer model"""
        print("\n🎯 Starting waypoint-based EKF simulation with FIXED magnetometer...")
        
        # Generate true trajectory based on waypoints
        x_true = self.generate_realistic_waypoint_trajectory()
        
        # Initialize EKF
        self.ekf.initialize(x_true[0])
        print("✅ EKF initialized with realistic waypoint navigation")
        
        # Run simulation
        start_time = time.time()
        
        for i in range(self.n_steps):
            current_time = i * self.dt
            
            # Generate sensor measurements from realistic trajectory
            sensors = self.sensor_model.generate_sensors(x_true[i], current_time)
            
            # EKF step with improved magnetometer model
            x_est, P = self.ekf.step(
                sensors['imu'], self.dt,
                gps_meas=sensors.get('gps'),
                baro_meas=sensors.get('baro'),
                mag_meas=sensors.get('mag'),
                use_accel_tilt=True
            )
            
            # Store data
            self.timestamps.append(current_time)
            self.true_states.append(x_true[i].copy())
            self.estimated_states.append(x_est.copy())
            self.covariances.append(np.diag(P).copy())
            self.sensor_measurements.append(sensors.copy())
            
            # Calculate errors
            pos_error = np.linalg.norm(x_true[i, 0:3] - x_est[0:3])
            vel_error = np.linalg.norm(x_true[i, 3:6] - x_est[3:6])
            att_error = np.linalg.norm(x_true[i, 6:9] - x_est[6:9])
            
            self.position_errors.append(pos_error)
            self.velocity_errors.append(vel_error)
            self.attitude_errors.append(att_error)
            
            # Progress updates
            if i % 1000 == 0 and i > 0:
                progress = i / self.n_steps * 100
                print(f"⏱️  Progress: {progress:.1f}% | Time: {current_time:.1f}s | Pos Error: {pos_error:.3f}m")
        
        end_time = time.time()
        sim_time = end_time - start_time
        real_time_factor = self.duration / sim_time
        
        print(f"\n✅ Simulation completed in {sim_time:.2f} seconds")
        print(f"   Real-time factor: {real_time_factor:.2f}x")
        
        # Calculate performance metrics
        self.calculate_performance_metrics()
        self.generate_plots()
        self.save_data()
    
    def calculate_performance_metrics(self):
        """Calculate and display performance metrics"""
        pos_errors = np.array(self.position_errors)
        vel_errors = np.array(self.velocity_errors)
        att_errors = np.array(self.attitude_errors)
        
        pos_rmse = np.sqrt(np.mean(pos_errors**2))
        vel_rmse = np.sqrt(np.mean(vel_errors**2))
        att_rmse = np.sqrt(np.mean(att_errors**2))
        
        print(f"\n🎯 PERFORMANCE METRICS:")
        print(f"   Position RMSE: {pos_rmse:.3f} m")
        print(f"   Velocity RMSE: {vel_rmse:.3f} m/s")
        print(f"   Attitude RMSE: {att_rmse:.3f} rad ({np.degrees(att_rmse):.1f}°)")
        print(f"\n   Max Position Error: {np.max(pos_errors):.3f} m")
        print(f"   Max Velocity Error: {np.max(vel_errors):.3f} m/s")
        print(f"   Max Attitude Error: {np.max(att_errors):.3f} rad ({np.degrees(np.max(att_errors)):.1f}°)")
        
        # Uncertainty metrics
        covariances = np.array(self.covariances)
        avg_pos_uncertainty = np.mean(np.sqrt(covariances[:, 0:3]), axis=0)
        avg_vel_uncertainty = np.mean(np.sqrt(covariances[:, 3:6]), axis=0)
        avg_att_uncertainty = np.mean(np.sqrt(covariances[:, 6:9]), axis=0)
        
        print(f"\n📈 UNCERTAINTY METRICS:")
        print(f"   Avg Position Uncertainty: {np.mean(avg_pos_uncertainty):.3f} m")
        print(f"   Avg Velocity Uncertainty: {np.mean(avg_vel_uncertainty):.3f} m/s")
        print(f"   Avg Attitude Uncertainty: {np.mean(avg_att_uncertainty):.3f} rad ({np.degrees(np.mean(avg_att_uncertainty)):.1f}°)")
        
        # Performance assessment
        pos_perf = "EXCELLENT" if pos_rmse < 0.5 else "GOOD" if pos_rmse < 1.0 else "FAIR" if pos_rmse < 2.0 else "POOR"
        vel_perf = "EXCELLENT" if vel_rmse < 0.5 else "GOOD" if vel_rmse < 1.0 else "FAIR" if vel_rmse < 2.0 else "POOR"
        att_perf = "EXCELLENT" if att_rmse < 0.5 else "GOOD" if att_rmse < 1.0 else "FAIR" if att_rmse < 2.0 else "POOR"
        
        print(f"\n🏆 PERFORMANCE ASSESSMENT:")
        print(f"   Position Estimation: {pos_perf}")
        print(f"   Velocity Estimation: {vel_perf}")
        print(f"   Attitude Estimation: {att_perf}")
    
    def generate_plots(self):
        """Generate analysis plots"""
        print(f"\n📊 Generating waypoint-based analysis plots...")
        
        # Create comprehensive analysis plot
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('Waypoint-Based EKF Analysis (Fixed Magnetometer Model)', fontsize=16)
        
        timestamps = np.array(self.timestamps)
        true_states = np.array(self.true_states)
        est_states = np.array(self.estimated_states)
        
        # Position tracking
        axes[0, 0].plot(timestamps, true_states[:, 0], 'b-', label='True X', linewidth=2)
        axes[0, 0].plot(timestamps, est_states[:, 0], 'r--', label='Estimated X', linewidth=2)
        axes[0, 0].set_title('Position X Tracking')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        axes[0, 1].plot(timestamps, true_states[:, 1], 'b-', label='True Y', linewidth=2)
        axes[0, 1].plot(timestamps, est_states[:, 1], 'r--', label='Estimated Y', linewidth=2)
        axes[0, 1].set_title('Position Y Tracking')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Position (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        axes[0, 2].plot(timestamps, true_states[:, 2], 'b-', label='True Z', linewidth=2)
        axes[0, 2].plot(timestamps, est_states[:, 2], 'r--', label='Estimated Z', linewidth=2)
        axes[0, 2].set_title('Position Z Tracking')
        axes[0, 2].set_xlabel('Time (s)')
        axes[0, 2].set_ylabel('Position (m)')
        axes[0, 2].legend()
        axes[0, 2].grid(True)
        
        # Error plots
        pos_errors = np.array(self.position_errors)
        vel_errors = np.array(self.velocity_errors)
        att_errors = np.array(self.attitude_errors)
        
        axes[1, 0].plot(timestamps, pos_errors, 'g-', linewidth=2)
        axes[1, 0].set_title('Position Error')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Error (m)')
        axes[1, 0].grid(True)
        
        axes[1, 1].plot(timestamps, vel_errors, 'g-', linewidth=2)
        axes[1, 1].set_title('Velocity Error')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Error (m/s)')
        axes[1, 1].grid(True)
        
        axes[1, 2].plot(timestamps, np.degrees(att_errors), 'g-', linewidth=2)
        axes[1, 2].set_title('Attitude Error')
        axes[1, 2].set_xlabel('Time (s)')
        axes[1, 2].set_ylabel('Error (degrees)')
        axes[1, 2].grid(True)
        
        plt.tight_layout()
        plt.savefig('waypoint_based_ekf_fixed_mag_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # 3D trajectory plot
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot true trajectory
        ax.plot(true_states[:, 0], true_states[:, 1], true_states[:, 2], 
                'b-', label='True Trajectory', linewidth=3)
        
        # Plot estimated trajectory
        ax.plot(est_states[:, 0], est_states[:, 1], est_states[:, 2], 
                'r--', label='EKF Estimate', linewidth=2)
        
        # Plot waypoints
        wp_x = [wp['x'] for wp in self.waypoints]
        wp_y = [wp['y'] for wp in self.waypoints]
        wp_z = [wp['z'] for wp in self.waypoints]
        ax.scatter(wp_x, wp_y, wp_z, c='red', s=100, marker='o', label='Waypoints')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Trajectory with Fixed Magnetometer Model')
        ax.legend()
        
        plt.savefig('waypoint_based_3d_trajectory_fixed_mag.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        print("✅ Plots saved:")
        print("   - waypoint_based_ekf_fixed_mag_analysis.png")
        print("   - waypoint_based_3d_trajectory_fixed_mag.png")
    
    def save_data(self):
        """Save simulation data to JSON file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"waypoint_based_ekf_fixed_mag_data_{timestamp}.json"
        
        data = {
            'simulation_type': 'waypoint_based_autonomous_flight_fixed_mag',
            'duration': self.duration,
            'sample_rate': len(self.timestamps) / self.duration,
            'waypoints': self.waypoints,
            'timestamps': self.timestamps,
            'true_states': [state.tolist() for state in self.true_states],
            'estimated_states': [state.tolist() for state in self.estimated_states],
            'covariances': [cov.tolist() for cov in self.covariances],
            'position_errors': self.position_errors,
            'velocity_errors': self.velocity_errors,
            'attitude_errors': self.attitude_errors,
            'performance_metrics': {
                'position_rmse': np.sqrt(np.mean(np.array(self.position_errors)**2)),
                'velocity_rmse': np.sqrt(np.mean(np.array(self.velocity_errors)**2)),
                'attitude_rmse': np.sqrt(np.mean(np.array(self.attitude_errors)**2))
            },
            'ekf_parameters': {
                'mag_sigma_deg': self.params.Mag_sigma_deg,
                'mag_sigma_rad': self.params.Mag_sigma_rad,
                'R_mag': self.params.R_mag,
                'Q_att': self.params.Q_att,
                'Q_vel': self.params.Q_vel
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"✅ Simulation data saved to: {filename}")

def main():
    """Main function to run the fixed magnetometer simulation"""
    sim = WaypointBasedEKFFixedMag(duration=120.0, dt=0.01)
    sim.run_simulation()
    
    print(f"\n🎉 Waypoint-based EKF simulation with FIXED magnetometer completed successfully!")
    print(f"📁 Check the generated plots and data files for improved performance.")
    print(f"🔧 Magnetometer innovation warnings should be significantly reduced!")

if __name__ == "__main__":
    main()
