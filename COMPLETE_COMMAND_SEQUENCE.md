# Complete Gazebo EKF Implementation - Command Sequence

## 🎯 **EXACT COMMAND SEQUENCE FOR ADMIN/USER**

### **Prerequisites:**
- User: `akyuret1` or `wa.akyuret1` 
- Working directory: `/u/12/akyuret1/unix/drone_sim`
- ROS 2 Jazzy installed and sourced

---

## **STEP 1: Clean Up Any Existing Processes**
```bash
# Kill any existing processes
pkill -f "autonomous_flight_controller"
pkill -f "realtime_ekf_integration" 
pkill -f "show_live_plots"
pkill -f "px4_ekf_integration"
pkill -f "mavros"
```

---

## **STEP 2: Start PX4 Gazebo Simulation**
```bash
# Navigate to PX4 directory and start simulation
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
make px4_sitl gz_x500 &
```

**Alternative (if PX4 make fails):**
```bash
# Use existing Gazebo simulation
gz sim -s -r -v 4 /u/88/wa.akyuret1/unix/PX4-Autopilot/Tools/simulation/gz/worlds/minimal_nocam.sdf &
```

---

## **STEP 3: Start MAVROS Bridge**
```bash
# Source ROS and start MAVROS
source /opt/ros/jazzy/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557 &
```

---

## **STEP 4: Start EKF Integration**
```bash
# Navigate to project directory
cd /u/12/akyuret1/unix/drone_sim

# Source ROS and start EKF integration
source /opt/ros/jazzy/setup.bash
python3 px4_ekf_integration.py &
```

---

## **STEP 5: Verify System Status**
```bash
# Check all processes are running
ps aux | grep -E "(gz sim|mavros|px4_ekf)" | grep -v grep

# Check ROS topics
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep mavros

# Check EKF topics
ros2 topic list | grep ekf
```

---

## **STEP 6: Monitor Real-time Data**
```bash
# Monitor PX4 position data
source /opt/ros/jazzy/setup.bash
ros2 topic echo /mavros/local_position/pose --once

# Monitor EKF estimates
ros2 topic echo /ekf/pose --once

# Monitor IMU data
ros2 topic echo /mavros/imu/data --once
```

---

## **STEP 7: Generate Analysis Plots**
```bash
# Generate plots from collected data
python3 create_demo_plots.py

# Or run live plotting
source /opt/ros/jazzy/setup.bash
python3 show_live_plots.py &
```

---

## **STEP 8: Check Data Files**
```bash
# List generated data files
ls -la *ekf*.json *autonomous*.json *.png

# Check file sizes
du -h *.json *.png
```

---

## **COMPLETE ONE-LINE LAUNCH SCRIPT**

Create this script for easy execution:

```bash
#!/bin/bash
# File: launch_px4_ekf_complete.sh

echo "🚁 LAUNCHING COMPLETE PX4 EKF SYSTEM"
echo "===================================="

# Cleanup
pkill -f "autonomous_flight_controller"
pkill -f "realtime_ekf_integration" 
pkill -f "px4_ekf_integration"
pkill -f "mavros"

# Start PX4 Gazebo
echo "🌍 Starting PX4 Gazebo..."
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
make px4_sitl gz_x500 &
sleep 5

# Start MAVROS
echo "🌉 Starting MAVROS..."
source /opt/ros/jazzy/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557 &
sleep 3

# Start EKF Integration
echo "🧠 Starting EKF Integration..."
cd /u/12/akyuret1/unix/drone_sim
source /opt/ros/jazzy/setup.bash
python3 px4_ekf_integration.py &

echo "✅ System launched! Check processes with: ps aux | grep -E '(gz|mavros|px4_ekf)'"
```

---

## **VERIFICATION COMMANDS**

```bash
# Check system status
ps aux | grep -E "(gz sim|mavros|px4_ekf)" | grep -v grep

# Check ROS topics
source /opt/ros/jazzy/setup.bash
ros2 topic list | wc -l

# Check MAVROS topics
ros2 topic list | grep mavros | wc -l

# Check EKF topics  
ros2 topic list | grep ekf
```

---

## **CURRENT RUNNING SYSTEM STATUS**

Based on terminal output, you currently have:

✅ **PX4 Gazebo Simulation**: Running (PID 163392)
✅ **MAVROS Bridge**: Connected and initialized
✅ **EKF Integration**: Running and processing data
✅ **Real-time Visualization**: Active

**Key Topics Available:**
- `/mavros/local_position/pose`
- `/mavros/local_position/velocity_local` 
- `/mavros/imu/data`
- `/mavros/gps/fix`
- `/ekf/pose`
- `/ekf/velocity`

---

## **STOP SYSTEM**

```bash
# Stop all components
pkill -f "gz sim"
pkill -f "mavros"
pkill -f "px4_ekf_integration"
pkill -f "px4"
```

---

## **TROUBLESHOOTING**

If MAVROS connection fails:
```bash
# Check PX4 is running
ps aux | grep px4

# Check UDP connection
netstat -u | grep 14540

# Restart MAVROS with different port
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557
```

If EKF integration fails:
```bash
# Check ROS topics
source /opt/ros/jazzy/setup.bash
ros2 topic list

# Check MAVROS topics
ros2 topic list | grep mavros
```

---

## **EXPECTED RESULTS**

After running the complete sequence:

1. **Gazebo GUI** showing PX4 drone flying autonomously
2. **MAVROS topics** publishing real flight data
3. **EKF integration** processing sensor data
4. **Real-time plots** showing trajectory and estimates
5. **Data files** containing logged flight data
6. **Analysis plots** showing performance metrics

This is the **authentic** PX4 autonomous flight system with EKF processing!
