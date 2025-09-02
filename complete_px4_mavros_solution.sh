#!/bin/bash

echo "🚁 COMPLETE PX4-MAVROS CONNECTION SOLUTION"
echo "==========================================="

# Step 1: Kill existing processes cleanly
echo "1. Cleaning up existing processes..."
pkill -f mavros_node
pkill -f px4
pkill -f gz
sleep 3

# Step 2: Restart PX4 with proper MAVLink configuration
echo "2. Starting PX4 gz_x500 with MAVLink configuration..."

# Go to PX4 directory and start PX4
cd /u/88/wa.akyuret1/unix/PX4-Autopilot

# Start PX4 in background with specific MAVLink setup
echo "   Starting PX4 SITL with gz_x500..."
timeout 30 make px4_sitl gz_x500 &
PX4_PID=$!

echo "   Waiting for PX4 to initialize..."
sleep 15

# Check if PX4 started successfully
if ps -p $PX4_PID > /dev/null; then
    echo "   ✅ PX4 process is running (PID: $PX4_PID)"
else
    echo "   ❌ PX4 failed to start"
    exit 1
fi

# Step 3: Configure MAVLink in PX4 (this would normally be done in PX4 shell)
echo "3. Configuring MAVLink..."
echo "   NOTE: MAVLink should be auto-configured in PX4 startup"

# Wait a bit more for full initialization
sleep 10

# Step 4: Test MAVLink output
echo "4. Testing MAVLink output..."
cd /u/12/akyuret1/unix/drone_sim

# Check for MAVLink traffic
timeout 5 ss -ulpn | grep 145 || echo "   Checking ports..."

# Step 5: Start clean MAVROS
echo "5. Starting MAVROS..."
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580 &
MAVROS_PID=$!

echo "   Waiting for MAVROS to initialize..."
sleep 8

# Step 6: Test connection
echo "6. Testing PX4-MAVROS connection..."
CONNECTED=$(timeout 5 ros2 topic echo /mavros/state --once | grep "connected:" | awk '{print $2}')

if [ "$CONNECTED" = "true" ]; then
    echo "   ✅ SUCCESS: MAVROS connected to PX4!"
    
    # Step 7: Test data flow
    echo "7. Testing data flow..."
    IMU_MESSAGES=$(timeout 3 ros2 topic hz /mavros/imu/data 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
    POSE_MESSAGES=$(timeout 3 ros2 topic hz /mavros/local_position/pose 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
    
    echo "   IMU messages: ${IMU_MESSAGES} Hz"
    echo "   Pose messages: ${POSE_MESSAGES} Hz"
    
    if [ "${IMU_MESSAGES}" != "0" ]; then
        echo "   ✅ Data pipeline is working!"
        echo ""
        echo "🎯 READY TO RUN EKF SIMULATION!"
        echo "==============================="
        echo "Use: python3 real_px4_ekf_integration.py"
    else
        echo "   ⚠️  Connection established but no data flow"
    fi
else
    echo "   ❌ MAVROS still not connected to PX4"
    echo ""
    echo "🔧 MANUAL STEPS REQUIRED:"
    echo "========================="
    echo "1. In PX4 shell (pxh>), run:"
    echo "   mavlink start -u 14580 -r 4000000"
    echo "   mavlink stream -u 14580 -s ATTITUDE -r 50"
    echo "   mavlink stream -u 14580 -s LOCAL_POSITION_NED -r 50"
    echo "   mavlink stream -u 14580 -s SCALED_IMU -r 50"
    echo ""
    echo "2. Then restart MAVROS:"
    echo "   pkill mavros_node"
    echo "   ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580"
fi

echo ""
echo "Background processes:"
echo "PX4 PID: $PX4_PID"
echo "MAVROS PID: $MAVROS_PID"
