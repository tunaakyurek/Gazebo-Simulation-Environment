#!/bin/bash

echo "=== COMPLETE PX4+GAZEBO+MAVROS SYSTEM DIAGNOSIS ==="
echo "Run with: sudo -u wa.akyuret1 bash complete_system_diagnosis.sh"
echo ""

echo "=== STEP 1: SYSTEM PROCESSES ==="
ps aux | grep -E "(px4|gazebo|gz|mavros|ros)" | grep -v grep

echo -e "\n=== STEP 2: NETWORK PORTS ==="
ss -tulpn | grep -E "(14540|14550|14557|14580|4560)"

echo -e "\n=== STEP 3: PX4 INSTALLATION ==="
ls -la /u/88/wa.akyuret1/unix/PX4-Autopilot/build/px4_sitl_default/bin/px4

echo -e "\n=== STEP 4: GZ_BRIDGE STATUS ==="
ls -la /u/88/wa.akyuret1/unix/PX4-Autopilot/build/px4_sitl_default/bin/px4-gz_bridge
which gz_bridge || echo "gz_bridge not in PATH"

echo -e "\n=== STEP 5: GAZEBO INSTALLATION ==="
which gz
gz --version

echo -e "\n=== STEP 6: ROS2 ENVIRONMENT ==="
source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO: $ROS_DISTRO"
which ros2

echo -e "\n=== STEP 7: PX4 CONSOLE TEST ==="
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
timeout 3s ./build/px4_sitl_default/bin/px4 -d << 'EOF'
status
quit
EOF

echo -e "\n=== STEP 8: SENSOR DATA TEST ==="
timeout 3s ./build/px4_sitl_default/bin/px4 -d << 'EOF'
listener sensor_accel
quit
EOF

echo -e "\n=== STEP 9: MAVROS TOPICS TEST ==="
cd /u/88/wa.akyuret1/unix/drone_sim
source /opt/ros/jazzy/setup.bash
timeout 3s ros2 topic list | grep mavros | head -5

echo -e "\n=== DIAGNOSIS COMPLETE ==="
