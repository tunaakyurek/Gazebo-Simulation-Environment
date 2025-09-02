#!/usr/bin/env python3
"""
Autonomous Flight Controller for Gazebo Drone Simulation
Generates figure-8 trajectory and publishes motor commands
"""

import numpy as np
import time
import json
from datetime import datetime
from typing import Tuple, List
import threading
import queue

class AutonomousFlightController:
    """
    Autonomous flight controller that generates figure-8 trajectories
    and publishes motor speed commands for the quadrotor drone
    """
    
    def __init__(self):
        self.running = False
        self.flight_data = []
        self.start_time = time.time()
        
        # Flight parameters
        self.hover_thrust = 0.6  # Normalized thrust for hovering
        self.max_thrust = 1.0
        self.min_thrust = 0.0
        
        # Figure-8 trajectory parameters
        self.trajectory_scale = 3.0  # Size of figure-8
        self.trajectory_height = 2.0  # Flight altitude
        self.trajectory_period = 20.0  # Time to complete one figure-8
        
        # Control gains
        self.kp_pos = 1.0
        self.kd_pos = 0.5
        self.kp_att = 2.0
        self.kd_att = 0.8
        
        # Drone parameters
        self.mass = 1.5  # kg
        self.arm_length = 0.2  # m
        self.thrust_coeff = 8.54858e-06
        self.moment_coeff = 0.016
        
        print("🚁 Autonomous Flight Controller initialized")
        print(f"   Trajectory: Figure-8 pattern")
        print(f"   Scale: {self.trajectory_scale}m")
        print(f"   Height: {self.trajectory_height}m")
        print(f"   Period: {self.trajectory_period}s")
    
    def generate_figure8_trajectory(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate figure-8 trajectory waypoints
        
        Args:
            t: Current time in seconds
        
        Returns:
            position: [x, y, z] desired position
            velocity: [vx, vy, vz] desired velocity
            acceleration: [ax, ay, az] desired acceleration
        """
        # Figure-8 parameters
        omega = 2 * np.pi / self.trajectory_period
        
        # Figure-8 trajectory equations
        x = self.trajectory_scale * np.sin(omega * t)
        y = self.trajectory_scale * np.sin(2 * omega * t) / 2
        z = self.trajectory_height
        
        # Velocities (derivatives)
        vx = self.trajectory_scale * omega * np.cos(omega * t)
        vy = self.trajectory_scale * omega * np.cos(2 * omega * t)
        vz = 0.0
        
        # Accelerations (second derivatives)
        ax = -self.trajectory_scale * omega**2 * np.sin(omega * t)
        ay = -2 * self.trajectory_scale * omega**2 * np.sin(2 * omega * t)
        az = 0.0
        
        position = np.array([x, y, z])
        velocity = np.array([vx, vy, vz])
        acceleration = np.array([ax, ay, az])
        
        return position, velocity, acceleration
    
    def compute_motor_commands(self, pos_des: np.ndarray, vel_des: np.ndarray, 
                              acc_des: np.ndarray, pos_curr: np.ndarray, 
                              vel_curr: np.ndarray, att_curr: np.ndarray) -> np.ndarray:
        """
        Compute motor speed commands for desired trajectory
        
        Args:
            pos_des: Desired position [x, y, z]
            vel_des: Desired velocity [vx, vy, vz]
            acc_des: Desired acceleration [ax, ay, az]
            pos_curr: Current position [x, y, z]
            vel_curr: Current velocity [vx, vy, vz]
            att_curr: Current attitude [roll, pitch, yaw]
        
        Returns:
            motor_speeds: [m1, m2, m3, m4] normalized motor speeds (0-1)
        """
        # Position control
        pos_error = pos_des - pos_curr
        vel_error = vel_des - vel_curr
        
        # Desired acceleration from position control
        acc_cmd = acc_des + self.kp_pos * pos_error + self.kd_pos * vel_error
        
        # Add gravity compensation
        g = 9.81
        acc_cmd[2] += g
        
        # Convert to thrust and attitude commands
        thrust_cmd = self.mass * np.linalg.norm(acc_cmd)
        
        # Desired attitude from acceleration command
        if thrust_cmd > 1e-6:
            roll_des = np.arcsin(np.clip(acc_cmd[1] / thrust_cmd, -1, 1))
            pitch_des = -np.arcsin(np.clip(acc_cmd[0] / thrust_cmd, -1, 1))
        else:
            roll_des = 0.0
            pitch_des = 0.0
        
        yaw_des = 0.0  # Keep yaw at 0
        
        # Attitude control
        roll_curr, pitch_curr, yaw_curr = att_curr
        
        roll_error = roll_des - roll_curr
        pitch_error = pitch_des - pitch_curr
        yaw_error = yaw_des - yaw_curr
        
        # Wrap yaw error
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        
        # Moment commands
        M_roll = self.kp_att * roll_error
        M_pitch = self.kp_att * pitch_error
        M_yaw = self.kp_att * yaw_error
        
        # Normalize thrust command
        thrust_normalized = np.clip(thrust_cmd / (self.mass * g), 0, 1)
        
        # Motor mixing for quadrotor in X configuration
        # Motor layout: 1(FL), 2(FR), 3(RL), 4(RR)
        motor1 = thrust_normalized + M_roll - M_pitch - M_yaw
        motor2 = thrust_normalized - M_roll - M_pitch + M_yaw
        motor3 = thrust_normalized + M_roll + M_pitch + M_yaw
        motor4 = thrust_normalized - M_roll + M_pitch - M_yaw
        
        # Clip motor commands
        motor_speeds = np.array([motor1, motor2, motor3, motor4])
        motor_speeds = np.clip(motor_speeds, self.min_thrust, self.max_thrust)
        
        return motor_speeds
    
    def log_flight_data(self, t: float, pos_des: np.ndarray, pos_curr: np.ndarray, 
                       vel_curr: np.ndarray, att_curr: np.ndarray, motor_speeds: np.ndarray):
        """Log flight data for analysis"""
        data_point = {
            'timestamp': t,
            'desired_position': pos_des.tolist(),
            'current_position': pos_curr.tolist(),
            'current_velocity': vel_curr.tolist(),
            'current_attitude': att_curr.tolist(),
            'motor_speeds': motor_speeds.tolist()
        }
        self.flight_data.append(data_point)
    
    def save_flight_data(self):
        """Save flight data to file"""
        filename = f"autonomous_flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, 'w') as f:
            json.dump(self.flight_data, f, indent=2)
        print(f"✅ Flight data saved to {filename}")
        return filename
    
    def run_autonomous_flight(self, duration: float = 60.0):
        """
        Run autonomous flight mission
        
        Args:
            duration: Flight duration in seconds
        """
        print("🚁 Starting autonomous flight mission...")
        print(f"   Duration: {duration} seconds")
        print(f"   Pattern: Figure-8")
        
        self.running = True
        dt = 0.02  # 50 Hz control loop
        
        # Simulated current state (in real system this would come from sensors/EKF)
        pos_curr = np.array([0.0, 0.0, 0.0])  # Start at origin
        vel_curr = np.array([0.0, 0.0, 0.0])
        att_curr = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        start_time = time.time()
        
        try:
            while self.running and (time.time() - start_time) < duration:
                current_time = time.time() - start_time
                
                # Generate desired trajectory
                pos_des, vel_des, acc_des = self.generate_figure8_trajectory(current_time)
                
                # Compute motor commands
                motor_speeds = self.compute_motor_commands(
                    pos_des, vel_des, acc_des, pos_curr, vel_curr, att_curr
                )
                
                # Simulate drone response (simplified)
                # In real system, this would be handled by Gazebo physics
                self.simulate_drone_response(motor_speeds, dt, pos_curr, vel_curr, att_curr)
                
                # Log data
                self.log_flight_data(current_time, pos_des, pos_curr, vel_curr, att_curr, motor_speeds)
                
                # Publish motor commands (placeholder - would use ROS topics in real system)
                self.publish_motor_commands(motor_speeds)
                
                # Control loop timing
                time.sleep(dt)
                
                # Print status every 5 seconds
                if int(current_time) % 5 == 0 and current_time - int(current_time) < dt:
                    print(f"Flight time: {current_time:.1f}s, Position: [{pos_curr[0]:.2f}, {pos_curr[1]:.2f}, {pos_curr[2]:.2f}]")
        
        except KeyboardInterrupt:
            print("\n🛑 Flight interrupted by user")
        
        finally:
            self.running = False
            print("🛬 Landing and saving flight data...")
            filename = self.save_flight_data()
            print(f"✅ Autonomous flight mission completed!")
            print(f"📊 Flight data saved to: {filename}")
    
    def simulate_drone_response(self, motor_speeds: np.ndarray, dt: float,
                               pos_curr: np.ndarray, vel_curr: np.ndarray, 
                               att_curr: np.ndarray):
        """
        Simplified drone dynamics simulation
        (In real system, this is handled by Gazebo)
        """
        # Simple integration for position
        acc_body = np.array([0, 0, np.mean(motor_speeds) * 15 - 9.81])  # Simplified thrust model
        
        # Rotate to world frame (simplified)
        roll, pitch, yaw = att_curr
        R = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        acc_world = R @ acc_body
        
        # Update velocity and position
        vel_curr += acc_world * dt
        pos_curr += vel_curr * dt
        
        # Simple attitude dynamics (very simplified)
        att_curr[0] += (motor_speeds[2] + motor_speeds[0] - motor_speeds[1] - motor_speeds[3]) * 0.1 * dt
        att_curr[1] += (motor_speeds[2] + motor_speeds[3] - motor_speeds[0] - motor_speeds[1]) * 0.1 * dt
        att_curr[2] += (motor_speeds[1] + motor_speeds[2] - motor_speeds[0] - motor_speeds[3]) * 0.05 * dt
    
    def publish_motor_commands(self, motor_speeds: np.ndarray):
        """
        Publish motor commands to simulation
        (In real system, this would use ROS topics)
        """
        # This is a placeholder - in the real system this would publish to
        # /autonomous_drone/command/motor_speed topic
        pass

def main():
    """Main function to run autonomous flight"""
    print("🚁 AUTONOMOUS FLIGHT CONTROLLER")
    print("===============================")
    
    controller = AutonomousFlightController()
    
    try:
        # Run autonomous flight for 60 seconds
        controller.run_autonomous_flight(duration=60.0)
    except Exception as e:
        print(f"❌ Error during flight: {e}")
    finally:
        print("🛬 Flight controller shutting down")

if __name__ == '__main__':
    main()