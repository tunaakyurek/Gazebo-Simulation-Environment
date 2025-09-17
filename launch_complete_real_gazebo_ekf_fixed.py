#!/usr/bin/env python3
"""
Complete Real Gazebo EKF Integration Launcher
This script launches the complete real Gazebo+PX4+MAVROS+EKF integration
with automatic connection fixing and improved magnetometer model.
"""

import subprocess
import sys
import time
import os
import signal
from pathlib import Path

def run_command(cmd, background=False, cwd=None):
    """Run a command with proper error handling"""
    print(f"🔧 Running: {' '.join(cmd)}")
    if cwd:
        print(f"   Working directory: {cwd}")
    
    try:
        if background:
            return subprocess.Popen(cmd, cwd=cwd, preexec_fn=os.setsid)
        else:
            result = subprocess.run(cmd, cwd=cwd, check=True, capture_output=True, text=True)
            return result
    except subprocess.CalledProcessError as e:
        print(f"❌ Command failed: {e}")
        if hasattr(e, 'stdout') and e.stdout:
            print(f"STDOUT: {e.stdout}")
        if hasattr(e, 'stderr') and e.stderr:
            print(f"STDERR: {e.stderr}")
        return None
    except Exception as e:
        print(f"❌ Error: {e}")
        return None

def check_dependencies():
    """Check if all required dependencies are available"""
    print("🔍 Checking dependencies...")
    
    required_commands = ['ros2', 'python3']
    missing = []
    
    for cmd in required_commands:
        if not subprocess.run(['which', cmd], capture_output=True).returncode == 0:
            missing.append(cmd)
    
    if missing:
        print(f"❌ Missing required commands: {', '.join(missing)}")
        return False
    
    # Check Python packages
    required_packages = ['rclpy', 'numpy', 'matplotlib', 'psutil']
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            print(f"❌ Missing Python package: {package}")
            return False
    
    print("✅ All dependencies available")
    return True

def check_ekf_files():
    """Check if all EKF files are present"""
    print("📁 Checking EKF files...")
    
    required_files = [
        'ekf_core.py',
        'ekf_parameters.py',
        'ekf_sensor_model.py',
        'ekf_dynamics.py',
        'waypoint_based_ekf_integration.py',
        'fix_mavros_px4_connection.py'
    ]
    
    missing = []
    for file in required_files:
        if not Path(file).exists():
            missing.append(file)
    
    if missing:
        print(f"❌ Missing required files: {', '.join(missing)}")
        return False
    
    print("✅ All EKF files present")
    return True

def main():
    """Main launcher function"""
    print("🚀 COMPLETE REAL GAZEBO EKF INTEGRATION LAUNCHER")
    print("================================================")
    print("🔧 Features:")
    print("   ✅ Automatic MAVROS-PX4 connection fixing")
    print("   ✅ Improved magnetometer model (reduced warnings)")
    print("   ✅ Real Gazebo physics simulation")
    print("   ✅ 18-waypoint autonomous mission")
    print("   ✅ 100Hz EKF processing")
    print("   ✅ Real-time performance analysis")
    print()
    
    # Check dependencies
    if not check_dependencies():
        print("❌ Dependency check failed")
        return 1
    
    # Check files
    if not check_ekf_files():
        print("❌ File check failed")
        return 1
    
    print("🔧 Starting connection fix procedure...")
    
    # Step 1: Fix MAVROS-PX4 connection
    try:
        # Import and run the connection fixer
        from fix_mavros_px4_connection import MAVROSPx4ConnectionFixer
        
        print("\n🔧 STEP 1: FIXING MAVROS-PX4 CONNECTION")
        print("========================================")
        
        with MAVROSPx4ConnectionFixer() as fixer:
            connection_success = fixer.fix_connection()
            
            if not connection_success:
                print("❌ Failed to establish MAVROS-PX4 connection")
                print("   Please check the error messages and try again.")
                return 1
            
            print("\n🎯 STEP 2: LAUNCHING EKF INTEGRATION")
            print("====================================")
            
            # Step 2: Launch the improved EKF integration
            try:
                print("🚁 Starting waypoint-based EKF integration...")
                
                # Run the enhanced EKF integration in the same process
                # Import here to ensure connection is established first
                import rclpy
                from waypoint_based_ekf_integration import WaypointBasedEKFIntegration
                
                # Initialize ROS 2
                rclpy.init()
                
                # Create and run the EKF integration node
                ekf_node = WaypointBasedEKFIntegration(simulation_duration=120.0)
                
                print("\n🎉 EKF INTEGRATION STARTED SUCCESSFULLY!")
                print("🔄 Running autonomous mission...")
                print("📊 Monitor the output for EKF performance")
                print("\nPress Ctrl+C to stop the simulation...")
                
                # Spin the node
                try:
                    rclpy.spin(ekf_node)
                except KeyboardInterrupt:
                    print("\n👋 Simulation interrupted by user")
                finally:
                    ekf_node.destroy_node()
                    rclpy.shutdown()
                
                print("\n🎯 SIMULATION COMPLETED SUCCESSFULLY!")
                print("📁 Check for generated analysis files and plots")
                
            except ImportError as e:
                print(f"❌ Import error: {e}")
                print("   Make sure all EKF modules are available")
                return 1
            except Exception as e:
                print(f"❌ Error during EKF integration: {e}")
                return 1
        
        print("\n✅ COMPLETE REAL GAZEBO EKF INTEGRATION FINISHED")
        return 0
        
    except ImportError as e:
        print(f"❌ Failed to import connection fixer: {e}")
        print("   Make sure fix_mavros_px4_connection.py is available")
        return 1
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return 1

def launch_manual_mode():
    """Launch in manual mode for debugging"""
    print("🔧 MANUAL LAUNCH MODE")
    print("=====================")
    print("This mode allows you to manually control each step.")
    print()
    
    steps = [
        ("1. Fix MAVROS-PX4 connection", "python3 fix_mavros_px4_connection.py"),
        ("2. Launch EKF integration", "python3 -c 'import rclpy; from waypoint_based_ekf_integration import WaypointBasedEKFIntegration; rclpy.init(); node=WaypointBasedEKFIntegration(); rclpy.spin(node)'")
    ]
    
    for i, (description, command) in enumerate(steps, 1):
        print(f"Step {i}: {description}")
        print(f"   Command: {command}")
        
        response = input(f"\nRun step {i}? (y/n/q): ").lower()
        if response == 'q':
            print("Exiting manual mode")
            return
        elif response == 'y':
            print(f"\n🔧 Running step {i}...")
            if i == 1:
                os.system(command)
            else:
                print("Please run this command in a new terminal:")
                print(f"   {command}")
                input("Press Enter when done...")
        else:
            print(f"Skipping step {i}")
    
    print("\n✅ Manual launch mode completed")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Launch complete real Gazebo EKF integration")
    parser.add_argument('--manual', action='store_true', help='Launch in manual mode for debugging')
    args = parser.parse_args()
    
    try:
        if args.manual:
            launch_manual_mode()
        else:
            exit(main())
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
        exit(0)
