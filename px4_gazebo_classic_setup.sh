#!/bin/bash

# Alternative PX4+Gazebo Classic+ROS2 Setup
# This approach uses ROS wrappers for more reliable sensor data flow

echo "=== PX4 + GAZEBO CLASSIC + ROS2 INTEGRATION ==="

# Clean environment
sudo pkill -f "px4\|gazebo\|mavros\|gz" || true
sleep 3

# Setup ROS environment and PX4 paths
sudo -u wa.akyuret1 bash -c "
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
source /opt/ros/jazzy/setup.bash

# Setup Gazebo environment
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds

# Build PX4 for Gazebo Classic
DONT_RUN=1 make px4_sitl_default gazebo-classic

# Export ROS package paths
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd)
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

echo 'Environment configured for Gazebo Classic'
"

echo "=== STARTING PX4 WITH GAZEBO CLASSIC ==="
sudo -u wa.akyuret1 bash -c "
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd):\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
nohup ros2 launch px4 posix_sitl.launch > /tmp/px4_classic.log 2>&1 &
echo 'PX4 Gazebo Classic launched'
"

sleep 10

echo "=== STARTING MAVROS FOR CLASSIC ==="
sudo -u wa.akyuret1 bash -c "
source /opt/ros/jazzy/setup.bash
cd /u/88/wa.akyuret1/unix/drone_sim
nohup ros2 launch mavros px4.launch fcu_url:='udp://:14540@192.168.1.36:14557' > /tmp/mavros_classic.log 2>&1 &
echo 'MAVROS Classic started'
"

echo "=== CLASSIC SETUP COMPLETE ==="

