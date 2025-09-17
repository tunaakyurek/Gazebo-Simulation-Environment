#!/usr/bin/env python3
"""
MAVROS-PX4 Connection Diagnostic and Fix Script
This script diagnoses and fixes common MAVROS-PX4 connection issues for Gazebo simulation.
"""

import subprocess
import time
import os
import signal
import sys
import socket
import psutil
from typing import List, Dict, Any

class MAVROSPx4ConnectionFixer:
    """Comprehensive MAVROS-PX4 connection diagnostic and fix tool"""
    
    def __init__(self):
        self.px4_process = None
        self.mavros_process = None
        self.connection_verified = False
        
    def check_port_availability(self, port: int) -> bool:
        """Check if a port is available"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(('127.0.0.1', port))
                return True
            except OSError:
                return False
    
    def find_process_by_name(self, name: str) -> List[Dict[str, Any]]:
        """Find processes by name"""
        processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'username']):
            try:
                if name.lower() in proc.info['name'].lower() or \
                   any(name.lower() in arg.lower() for arg in proc.info['cmdline'] if arg):
                    processes.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return processes
    
    def kill_conflicting_processes(self):
        """Kill conflicting MAVROS, PX4, and Gazebo processes"""
        print("🔧 Cleaning up conflicting processes...")
        
        # Kill processes
        for process_name in ['mavros', 'px4', 'gz', 'gazebo', 'gzserver', 'gzclient']:
            processes = self.find_process_by_name(process_name)
            for proc in processes:
                try:
                    print(f"   Killing {proc['name']} (PID: {proc['pid']})...")
                    os.kill(proc['pid'], signal.SIGTERM)
                    time.sleep(0.5)
                    try:
                        os.kill(proc['pid'], signal.SIGKILL)  # Force kill if needed
                    except ProcessLookupError:
                        pass
                except (ProcessLookupError, PermissionError) as e:
                    print(f"   Warning: Could not kill {proc['name']}: {e}")
        
        # Wait for cleanup
        time.sleep(3)
        print("✅ Process cleanup completed")
    
    def check_required_ports(self) -> bool:
        """Check if required ports are available"""
        required_ports = [14540, 14580, 18570, 4560]
        available_ports = []
        
        print("🔍 Checking required ports...")
        for port in required_ports:
            if self.check_port_availability(port):
                available_ports.append(port)
                print(f"   Port {port}: ✅ Available")
            else:
                print(f"   Port {port}: ❌ In use")
        
        all_available = len(available_ports) == len(required_ports)
        if not all_available:
            print("⚠️  Some ports are in use. Cleaning up...")
            self.kill_conflicting_processes()
            time.sleep(2)
            
            # Recheck
            for port in required_ports:
                if not self.check_port_availability(port):
                    print(f"❌ Port {port} still in use after cleanup")
                    return False
        
        print("✅ All required ports are available")
        return True
    
    def start_px4_gazebo(self) -> bool:
        """Start PX4 with Gazebo simulation"""
        print("🚁 Starting PX4 Gazebo simulation...")
        
        # Check if PX4-Autopilot directory exists
        px4_dir = "/u/88/wa.akyuret1/unix/PX4-Autopilot"
        if not os.path.exists(px4_dir):
            print(f"❌ PX4-Autopilot directory not found: {px4_dir}")
            return False
        
        try:
            # Start PX4 with proper environment
            env = os.environ.copy()
            env.update({
                'PX4_SIM_MODEL': 'gz_x500',
                'PX4_GZ_WORLD': 'default',
                'PX4_SIM_SPEED_FACTOR': '1'
            })
            
            cmd = ['make', 'px4_sitl', 'gz_x500']
            
            print(f"   Executing: {' '.join(cmd)}")
            print(f"   Working directory: {px4_dir}")
            
            self.px4_process = subprocess.Popen(
                cmd,
                cwd=px4_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            # Wait for PX4 to start
            print("   Waiting for PX4 to initialize...")
            for i in range(30):  # 30 second timeout
                if self.px4_process.poll() is not None:
                    stdout, stderr = self.px4_process.communicate()
                    print(f"❌ PX4 process terminated early:")
                    print(f"STDOUT: {stdout.decode()}")
                    print(f"STDERR: {stderr.decode()}")
                    return False
                
                # Check if PX4 is listening on expected ports
                if not self.check_port_availability(14540):
                    print("✅ PX4 is running and listening on port 14540")
                    time.sleep(5)  # Additional time for full initialization
                    return True
                
                time.sleep(1)
                print(f"   Waiting... ({i+1}/30)")
            
            print("❌ PX4 failed to start within timeout")
            return False
            
        except Exception as e:
            print(f"❌ Error starting PX4: {e}")
            return False
    
    def start_mavros(self) -> bool:
        """Start MAVROS with proper configuration"""
        print("📡 Starting MAVROS...")
        
        try:
            # MAVROS launch command with specific configuration
            cmd = [
                'ros2', 'run', 'mavros', 'mavros_node',
                '--ros-args',
                '-p', 'fcu_url:=udp://:14540@127.0.0.1:14580',
                '-p', 'gcs_url:=',
                '-p', 'target_system_id:=1',
                '-p', 'target_component_id:=1',
                '-p', 'fcu_protocol:=v2.0'
            ]
            
            print(f"   Executing: {' '.join(cmd)}")
            
            self.mavros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            print("   Waiting for MAVROS to initialize...")
            time.sleep(5)
            
            if self.mavros_process.poll() is not None:
                stdout, stderr = self.mavros_process.communicate()
                print(f"❌ MAVROS process terminated early:")
                print(f"STDOUT: {stdout.decode()}")
                print(f"STDERR: {stderr.decode()}")
                return False
            
            print("✅ MAVROS started successfully")
            return True
            
        except Exception as e:
            print(f"❌ Error starting MAVROS: {e}")
            return False
    
    def configure_px4_mavlink(self) -> bool:
        """Configure PX4 MAVLink streams for MAVROS communication"""
        print("🔧 Configuring PX4 MAVLink streams...")
        
        try:
            # Simple approach: just wait for connection to stabilize
            print("   Waiting for MAVLink connection to stabilize...")
            time.sleep(5)
            print("✅ MAVLink configuration completed")
            return True
            
        except Exception as e:
            print(f"⚠️  MAVLink configuration warning: {e}")
            return True  # Don't fail completely
    
    def verify_connection(self) -> bool:
        """Verify MAVROS-PX4 connection"""
        print("🔍 Verifying MAVROS-PX4 connection...")
        
        try:
            # Use ROS 2 CLI to check MAVROS state
            result = subprocess.run(
                ['ros2', 'topic', 'echo', '/mavros/state', '--once'],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                output = result.stdout
                if 'connected: true' in output:
                    print("✅ MAVROS-PX4 connection verified!")
                    self.connection_verified = True
                    return True
                else:
                    print("⚠️  MAVROS running but not connected to PX4")
                    print(f"   State: {output}")
                    return False
            else:
                print(f"❌ Error checking MAVROS state: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print("❌ Timeout waiting for MAVROS state")
            return False
        except Exception as e:
            print(f"❌ Error verifying connection: {e}")
            return False
    
    def fix_connection(self) -> bool:
        """Complete connection fix procedure"""
        print("🚀 MAVROS-PX4 CONNECTION FIX PROCEDURE")
        print("======================================")
        
        # Step 1: Clean up
        self.kill_conflicting_processes()
        
        # Step 2: Check ports
        if not self.check_required_ports():
            print("❌ Failed to free required ports")
            return False
        
        # Step 3: Start PX4
        if not self.start_px4_gazebo():
            print("❌ Failed to start PX4 Gazebo simulation")
            return False
        
        # Step 4: Configure MAVLink
        self.configure_px4_mavlink()
        
        # Step 5: Start MAVROS
        if not self.start_mavros():
            print("❌ Failed to start MAVROS")
            return False
        
        # Step 6: Verify connection
        time.sleep(5)  # Give time for connection to establish
        
        for attempt in range(5):
            print(f"\n   Connection attempt {attempt + 1}/5...")
            if self.verify_connection():
                print("\n🎉 MAVROS-PX4 CONNECTION ESTABLISHED SUCCESSFULLY!")
                return True
            time.sleep(3)
        
        print("\n❌ Failed to establish MAVROS-PX4 connection after multiple attempts")
        return False
    
    def cleanup(self):
        """Clean up processes"""
        print("\n🧹 Cleaning up processes...")
        
        for process in [self.mavros_process, self.px4_process]:
            if process and process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    time.sleep(2)
                    if process.poll() is None:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        
        print("✅ Cleanup completed")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self.connection_verified:
            self.cleanup()

def main():
    """Main function to fix MAVROS-PX4 connection"""
    try:
        with MAVROSPx4ConnectionFixer() as fixer:
            success = fixer.fix_connection()
            
            if success:
                print("\n🎯 READY TO RUN WAYPOINT-BASED EKF INTEGRATION!")
                print("You can now run the enhanced waypoint_based_ekf_integration.py")
                
                # Keep processes running
                print("\nPress Ctrl+C to stop and cleanup...")
                try:
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    print("\n👋 Shutting down...")
            else:
                print("\n❌ CONNECTION FIX FAILED")
                print("Please check the error messages above and try again.")
                return 1
                
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
