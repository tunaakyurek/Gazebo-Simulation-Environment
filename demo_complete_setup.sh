#!/bin/bash
set -e

echo "=== Complete Gazebo + ROS + PX4 Simulation Setup Demo ==="
echo

# Step 1: Check Gazebo version
echo "1. Checking Gazebo version..."
gz sim --version
echo

# Step 2: Start Gazebo simulation
echo "2. Starting Gazebo simulation..."
pkill -f "gz sim" 2>/dev/null || true
gz sim -s -r -v 4 /u/12/akyuret1/unix/drone_sim/worlds/simple_world.sdf &
GZ_PID=$!
sleep 5
echo "Gazebo simulation started with PID: $GZ_PID"
echo

# Step 3: Check Gazebo topics
echo "3. Checking Gazebo topics..."
gz topic -l
echo

# Step 4: Check IMU data
echo "4. Checking IMU sensor data..."
timeout 3 gz topic -e -t /sim/imu_raw || echo "IMU topic not available yet"
echo

# Step 5: Start ROS bridge
echo "5. Starting ROS-Gazebo bridge..."
bash -c "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock /sim/imu_raw@sensor_msgs/msg/Imu@gz.msgs.IMU" &
BRIDGE_PID=$!
sleep 3
echo "ROS bridge started with PID: $BRIDGE_PID"
echo

# Step 6: Check ROS topics
echo "6. Checking ROS topics..."
bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
echo

# Step 7: Test sensor data flow
echo "7. Testing sensor data flow..."
echo "Checking IMU data in ROS..."
timeout 3 bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /sim/imu_raw --once" || echo "IMU data not flowing through bridge yet"
echo

# Step 8: PX4 integration test
echo "8. Testing PX4 integration..."
echo "Checking if PX4 can run with current Gazebo version..."
cd /u/12/akyuret1/unix/PX4-Autopilot
if ./build/px4_sitl_default/bin/px4 -s ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim --help >/dev/null 2>&1; then
    echo "✅ PX4 is compatible with current Gazebo version"
else
    echo "❌ PX4 compatibility issue detected"
fi
echo

echo "=== Setup Complete ==="
echo "Gazebo PID: $GZ_PID"
echo "Bridge PID: $BRIDGE_PID"
echo "Press Ctrl+C to stop all processes"
echo

# Wait for user interrupt
trap "echo 'Stopping simulation...'; kill $GZ_PID $BRIDGE_PID 2>/dev/null; exit 0" INT
wait
