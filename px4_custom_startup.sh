#!/bin/bash

# PX4 Custom Startup Script with MAVLink Stream Configuration
# This script resolves the sensor data publishing issue

echo "=== CUSTOM PX4 STARTUP WITH MAVLINK STREAMS ==="

# Clean any existing PX4 processes
sudo pkill -f "px4" || true
sudo pkill -f "gz sim" || true
sleep 2

# Start PX4 SITL with Gazebo in background
sudo -u wa.akyuret1 bash -c "
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLD=default
nohup make px4_sitl gz_x500 > /tmp/px4_custom.log 2>&1 &
echo 'PX4+Gazebo started in background'
"

# Wait for PX4 to initialize
echo "Waiting for PX4 initialization..."
sleep 10

# Configure MAVLink streams via direct commands
echo "=== CONFIGURING MAVLINK STREAMS ==="
sudo -u wa.akyuret1 bash -c "
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
./build/px4_sitl_default/bin/px4 -d << 'MAVLINK_CONFIG'
mavlink start -u 14556 -r 4000000 -m onboard -o 14540
mavlink stream -u 14556 -s HIGHRES_IMU -r 50
mavlink stream -u 14556 -s SCALED_PRESSURE -r 20  
mavlink stream -u 14556 -s GPS_RAW_INT -r 10
mavlink stream -u 14556 -s GLOBAL_POSITION_INT -r 10
mavlink stream -u 14556 -s LOCAL_POSITION_NED -r 20
mavlink stream -u 14556 -s ATTITUDE -r 50
mavlink stream -u 14556 -s HEARTBEAT -r 1
mavlink stream -u 14556 -s SYS_STATUS -r 1
mavlink stream -u 14556 -s RAW_IMU -r 50
mavlink stream -u 14556 -s SCALED_IMU -r 50
quit
MAVLINK_CONFIG
echo 'MAVLink streams configured'
"

# Start MAVROS with correct configuration
echo "=== STARTING MAVROS ==="
sudo -u wa.akyuret1 bash -c "
source /opt/ros/jazzy/setup.bash
cd /u/88/wa.akyuret1/unix/drone_sim
nohup ros2 launch mavros px4.launch fcu_url:='udp://:14540@127.0.0.1:14580' > /tmp/mavros_custom.log 2>&1 &
echo 'MAVROS started'
"

echo "=== SETUP COMPLETE ==="
echo "PX4 Log: /tmp/px4_custom.log"
echo "MAVROS Log: /tmp/mavros_custom.log"
echo "Waiting 5 seconds for connections..."
sleep 5

echo "=== VERIFYING SETUP ==="
ps aux | grep -E "(px4|mavros|gz sim)" | grep -v grep

