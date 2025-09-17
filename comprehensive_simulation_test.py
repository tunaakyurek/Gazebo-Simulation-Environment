#!/usr/bin/env python3

"""
Comprehensive PX4+Gazebo+MAVROS Simulation Test
Tests all approaches and validates sensor data flow
"""

import subprocess
import time
import sys
import signal
import os

class SimulationTester:
    
    def __init__(self):
        self.test_results = {}
        self.current_processes = []
        
    def run_command(self, cmd, timeout=15, background=False):
        """Run command with proper error handling"""
        try:
            if background:
                proc = subprocess.Popen(cmd, shell=True, 
                                      stdout=subprocess.PIPE, 
                                      stderr=subprocess.PIPE)
                self.current_processes.append(proc)
                return True, "", ""
            else:
                result = subprocess.run(cmd, shell=True, capture_output=True, 
                                      text=True, timeout=timeout)
                return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Command timed out"
        except Exception as e:
            return False, "", str(e)
            
    def cleanup_processes(self):
        """Clean up all running processes"""
        print("🧹 Cleaning up processes...")
        
        # Kill our spawned processes
        for proc in self.current_processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except:
                try:
                    proc.kill()
                except:
                    pass
        
        # Kill system processes
        cleanup_cmds = [
            "sudo pkill -f 'px4' || true",
            "sudo pkill -f 'gz sim' || true", 
            "sudo pkill -f 'mavros' || true",
            "sudo pkill -f 'gazebo' || true"
        ]
        
        for cmd in cleanup_cmds:
            self.run_command(cmd, timeout=5)
            
        time.sleep(3)
        print("✅ Cleanup completed")
        
    def test_approach_1_custom_mavlink(self):
        """Test Approach 1: Custom MAVLink Streams"""
        
        print("\n🧪 TEST 1: Custom MAVLink Stream Configuration")
        print("=" * 60)
        
        self.cleanup_processes()
        
        # Run custom startup script
        success, stdout, stderr = self.run_command(
            "chmod +x px4_custom_startup.sh && ./px4_custom_startup.sh", 
            timeout=45
        )
        
        if not success:
            print(f"❌ Custom startup failed: {stderr}")
            self.test_results['approach_1'] = False
            return False
            
        # Wait for initialization
        time.sleep(10)
        
        # Test sensor data flow
        sensor_test = self.test_sensor_data_flow()
        self.test_results['approach_1'] = sensor_test
        
        return sensor_test
        
    def test_approach_2_gazebo_classic(self):
        """Test Approach 2: Gazebo Classic Integration"""
        
        print("\n🧪 TEST 2: Gazebo Classic Integration")
        print("=" * 60)
        
        self.cleanup_processes()
        
        # Run Gazebo Classic setup
        success, stdout, stderr = self.run_command(
            "chmod +x px4_gazebo_classic_setup.sh && ./px4_gazebo_classic_setup.sh",
            timeout=45
        )
        
        if not success:
            print(f"❌ Gazebo Classic setup failed: {stderr}")
            self.test_results['approach_2'] = False
            return False
            
        # Wait for initialization  
        time.sleep(15)
        
        # Test sensor data flow
        sensor_test = self.test_sensor_data_flow()
        self.test_results['approach_2'] = sensor_test
        
        return sensor_test
        
    def test_approach_3_ekf2_fix(self):
        """Test Approach 3: EKF2 Configuration Fix"""
        
        print("\n🧪 TEST 3: EKF2 Configuration Fix")
        print("=" * 60)
        
        # Apply EKF2 fixes
        success, stdout, stderr = self.run_command(
            "python3 fix_px4_ekf2_config.py",
            timeout=30
        )
        
        if not success:
            print(f"❌ EKF2 configuration failed: {stderr}")
            self.test_results['approach_3'] = False
            return False
            
        print("✅ EKF2 configuration applied")
        
        # Restart simulation with EKF2 fixes
        success = self.test_approach_1_custom_mavlink()
        self.test_results['approach_3'] = success
        
        return success
        
    def test_sensor_data_flow(self):
        """Test if sensor data is flowing through MAVROS"""
        
        print("📊 Testing sensor data flow...")
        
        # Check MAVROS topics
        topics_cmd = """
sudo -u wa.akyuret1 bash -c "
source /opt/ros/jazzy/setup.bash
timeout 3s ros2 topic list | grep mavros | head -5
"
"""
        
        success, stdout, stderr = self.run_command(topics_cmd, timeout=10)
        
        if not success or not stdout.strip():
            print("❌ No MAVROS topics found")
            return False
            
        print("✅ MAVROS topics available:")
        print(stdout.strip())
        
        # Test specific sensor topics
        sensor_topics = [
            "/mavros/imu/data",
            "/mavros/local_position/pose", 
            "/mavros/global_position/global",
            "/mavros/state"
        ]
        
        working_topics = 0
        
        for topic in sensor_topics:
            test_cmd = f"""
sudo -u wa.akyuret1 bash -c "
source /opt/ros/jazzy/setup.bash
timeout 2s ros2 topic echo {topic} --once --timeout-sec 2
"
"""
            
            success, stdout, stderr = self.run_command(test_cmd, timeout=5)
            
            if success and stdout.strip():
                print(f"✅ {topic}: Data flowing")
                working_topics += 1
            else:
                print(f"❌ {topic}: No data")
                
        # Consider success if at least 2 topics have data
        return working_topics >= 2
        
    def run_final_integration_test(self):
        """Run the actual waypoint EKF integration"""
        
        print("\n🚀 FINAL INTEGRATION TEST")
        print("=" * 60)
        
        # Run waypoint integration for 30 seconds
        integration_cmd = """
sudo -u wa.akyuret1 bash -c "
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
cd /u/88/wa.akyuret1/unix/drone_sim
timeout 30s python3 waypoint_based_ekf_integration.py
"
"""
        
        print("🛫 Running waypoint-based EKF integration...")
        success, stdout, stderr = self.run_command(integration_cmd, timeout=35)
        
        # Check for data collection
        data_check_cmd = """
sudo -u wa.akyuret1 bash -c "
cd /u/88/wa.akyuret1/unix/drone_sim
ls -la real_px4_ekf_data_*.json 2>/dev/null | tail -1
"
"""
        
        data_success, data_stdout, data_stderr = self.run_command(data_check_cmd, timeout=5)
        
        if data_success and data_stdout.strip():
            print("✅ Data files created:")
            print(data_stdout.strip())
            
            # Check file content
            latest_file = data_stdout.strip().split()[-1]
            content_cmd = f"""
sudo -u wa.akyuret1 bash -c "
cd /u/88/wa.akyuret1/unix/drone_sim
grep 'data_points' {latest_file} || echo 'File check failed'
"
"""
            
            content_success, content_stdout, content_stderr = self.run_command(content_cmd, timeout=5)
            
            if "data_points" in content_stdout and '"data_points": 0' not in content_stdout:
                print("🎉 SUCCESS: Real sensor data collected!")
                return True
            else:
                print("⚠️  Data file exists but no sensor data collected")
                return False
        else:
            print("❌ No data files created")
            return False
            
    def run_comprehensive_test(self):
        """Run all test approaches systematically"""
        
        print("🔬 COMPREHENSIVE PX4+GAZEBO+MAVROS SIMULATION TEST")
        print("=" * 70)
        
        try:
            # Test all approaches
            approaches = [
                ("Custom MAVLink Streams", self.test_approach_1_custom_mavlink),
                ("Gazebo Classic", self.test_approach_2_gazebo_classic), 
                ("EKF2 Configuration Fix", self.test_approach_3_ekf2_fix)
            ]
            
            successful_approach = None
            
            for name, test_func in approaches:
                print(f"\n🧪 Testing: {name}")
                if test_func():
                    print(f"✅ {name}: SUCCESS")
                    successful_approach = name
                    break
                else:
                    print(f"❌ {name}: FAILED")
                    
            # If any approach worked, run final integration
            if successful_approach:
                print(f"\n🎯 Using successful approach: {successful_approach}")
                if self.run_final_integration_test():
                    print("\n🏆 COMPLETE SUCCESS: Waypoint-based EKF integration working!")
                    return True
                else:
                    print("\n⚠️  Approach worked but integration failed")
                    
            print("\n📊 FINAL RESULTS:")
            print("=" * 40)
            for approach, result in self.test_results.items():
                status = "✅ SUCCESS" if result else "❌ FAILED"
                print(f"{approach}: {status}")
                
            return False
            
        except KeyboardInterrupt:
            print("\n⏹️  Test interrupted by user")
            return False
        finally:
            self.cleanup_processes()

def main():
    """Main test execution"""
    
    tester = SimulationTester()
    
    # Setup signal handler for cleanup
    def signal_handler(sig, frame):
        print("\n🛑 Received interrupt signal")
        tester.cleanup_processes()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Run comprehensive test
    success = tester.run_comprehensive_test()
    
    if success:
        print("\n🎉 MISSION ACCOMPLISHED!")
        print("✅ Real PX4+Gazebo+MAVROS waypoint simulation is working!")
        sys.exit(0)
    else:
        print("\n❌ SIMULATION SETUP REQUIRES ADDITIONAL WORK")
        sys.exit(1)

if __name__ == "__main__":
    main()

