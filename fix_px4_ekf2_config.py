#!/usr/bin/env python3

"""
PX4 EKF2 Configuration Fix Script
Addresses the "Preflight Fail: ekf2 missing data" warning
"""

import subprocess
import time
import sys

def run_command(cmd, timeout=10):
    """Run shell command with timeout"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, 
                              text=True, timeout=timeout)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)

def configure_ekf2_parameters():
    """Configure EKF2 parameters to accept simulated sensor data"""
    
    print("=== CONFIGURING EKF2 PARAMETERS ===")
    
    ekf2_configs = [
        # Enable EKF2 and set sensor timeouts
        "param set EKF2_ENABLE 1",
        "param set EKF2_GPS_DELAY 110.0",
        "param set EKF2_IMU_DELAY 10.0",
        "param set EKF2_BARO_DELAY 0.0",
        "param set EKF2_MAG_DELAY 0.0",
        
        # Set sensor noise parameters for simulation
        "param set EKF2_GPS_P_NOISE 0.5",
        "param set EKF2_GPS_V_NOISE 0.3", 
        "param set EKF2_BARO_NOISE 2.0",
        "param set EKF2_ACC_NOISE 0.35",
        "param set EKF2_GYR_NOISE 0.015",
        "param set EKF2_MAG_NOISE 0.05",
        
        # Enable all sensor fusion
        "param set EKF2_AID_MASK 1",  # GPS
        "param set EKF2_HGT_MODE 0",  # Barometer height
        "param set EKF2_GPS_CHECK 245",  # GPS checks
        
        # Simulation-specific parameters
        "param set SYS_MC_EST_GROUP 2",  # Use EKF2
        "param set SENS_BOARD_X_OFF 0.0",
        "param set SENS_BOARD_Y_OFF 0.0", 
        "param set SENS_BOARD_Z_OFF 0.0",
        
        # Relaxed arming checks for simulation
        "param set COM_ARM_WO_GPS 1",
        "param set NAV_RCL_ACT 0",
        "param set NAV_DLL_ACT 0",
        
        # Save parameters
        "param save"
    ]
    
    # Apply configurations via PX4 shell
    config_script = """
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
./build/px4_sitl_default/bin/px4 -d << 'EKF_CONFIG'
{}
quit
EKF_CONFIG
""".format('\n'.join(ekf2_configs))
    
    success, stdout, stderr = run_command(f"sudo -u wa.akyuret1 bash -c \"{config_script}\"", 30)
    
    if success:
        print("✅ EKF2 parameters configured successfully")
    else:
        print(f"❌ EKF2 configuration failed: {stderr}")
    
    return success

def verify_sensor_availability():
    """Check if sensors are available in Gazebo model"""
    
    print("=== VERIFYING SENSOR AVAILABILITY ===")
    
    # Check x500 model sensors
    model_check = """
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
find . -name "model.sdf" -path "*x500*" -exec grep -l "imu_sensor\|navsat_sensor\|air_pressure_sensor\|magnetometer_sensor" {} \;
"""
    
    success, stdout, stderr = run_command(f"sudo -u wa.akyuret1 bash -c \"{model_check}\"")
    
    if success and stdout.strip():
        print("✅ Sensors found in Gazebo model:")
        print(stdout.strip())
        return True
    else:
        print("❌ No sensor definitions found in Gazebo model")
        return False

def check_px4_logs():
    """Check recent PX4 logs for EKF2 status"""
    
    print("=== CHECKING PX4 LOGS FOR EKF2 STATUS ===")
    
    log_check = """
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
if [ -f /tmp/px4_custom.log ]; then
    echo "=== Recent PX4 Log ==="
    tail -20 /tmp/px4_custom.log | grep -E "(EKF|ekf|sensor|missing|fail)"
else
    echo "No PX4 log found"
fi
"""
    
    success, stdout, stderr = run_command(f"sudo -u wa.akyuret1 bash -c \"{log_check}\"")
    
    if stdout:
        print(stdout)
    else:
        print("No EKF2-related log entries found")

def main():
    """Main EKF2 configuration workflow"""
    
    print("🔧 PX4 EKF2 Configuration Fix")
    print("=" * 50)
    
    # Step 1: Verify sensor availability
    if not verify_sensor_availability():
        print("⚠️  Warning: Sensor issues detected in Gazebo model")
    
    # Step 2: Configure EKF2 parameters
    if configure_ekf2_parameters():
        print("✅ EKF2 configuration applied")
        time.sleep(2)
    else:
        print("❌ EKF2 configuration failed")
        return False
    
    # Step 3: Check logs
    check_px4_logs()
    
    print("🏁 EKF2 configuration process completed")
    return True

if __name__ == "__main__":
    main()

