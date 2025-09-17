#!/bin/bash

echo "=== FIXING PX4-GAZEBO CONNECTION ISSUES ==="
echo "Run with: sudo -u wa.akyuret1 bash fix_px4_gazebo_connection.sh"
echo ""

echo "=== STEP 1: STOP ALL RUNNING PROCESSES ==="
pkill -f "px4" || true
pkill -f "gz sim" || true
pkill -f "mavros" || true
sleep 3

echo "=== STEP 2: FIX GZ_BRIDGE PATH ==="
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
ln -sf /u/88/wa.akyuret1/unix/PX4-Autopilot/build/px4_sitl_default/bin/px4-gz_bridge /usr/local/bin/gz_bridge
echo "gz_bridge symlink created"

echo "=== STEP 3: VERIFY GZ_BRIDGE ==="
which gz_bridge
gz_bridge --help

echo "=== STEP 4: RESTART PX4 WITH GAZEBO ==="
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLD=default
make px4_sitl gz_x500 &

echo "=== STEP 5: WAIT FOR INITIALIZATION ==="
sleep 10

echo "=== STEP 6: CHECK PX4 STATUS ==="
timeout 3s ./build/px4_sitl_default/bin/px4 -d << 'EOF'
status
sensor status
quit
EOF

echo "=== STEP 7: START MAVROS ==="
cd /u/88/wa.akyuret1/unix/drone_sim
source /opt/ros/jazzy/setup.bash
ros2 launch mavros px4.launch fcu_url:='udp://:14540@127.0.0.1:14580' &

echo "=== STEP 8: WAIT FOR MAVROS ==="
sleep 5

echo "=== STEP 9: TEST SENSOR DATA ==="
timeout 3s ros2 topic echo /mavros/imu/data --once

echo "=== FIX COMPLETE ==="
