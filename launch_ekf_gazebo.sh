#!/bin/bash
set -e

echo "🚁 EKF GAZEBO INTEGRATION LAUNCHER"
echo "=================================="
echo

# Check if MAVROS is running
echo "🔍 Checking MAVROS status..."
if ! pgrep -f "mavros_node" > /dev/null; then
    echo "❌ MAVROS is not running. Please start MAVROS first."
    echo "Run: ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580"
    exit 1
fi
echo "✅ MAVROS is running"
echo

# Check if Gazebo simulation is running
echo "🔍 Checking Gazebo simulation..."
if ! sudo -u wa.akyuret1 gz topic -l > /dev/null 2>&1; then
    echo "❌ Gazebo simulation is not running. Please start the simulation first."
    exit 1
fi
echo "✅ Gazebo simulation is running"
echo

# Install required Python packages if needed
echo "📦 Checking Python dependencies..."
python3 -c "import scipy" 2>/dev/null || {
    echo "Installing scipy..."
    sudo apt install -y python3-scipy
}
echo "✅ Python dependencies OK"
echo

# Make scripts executable
chmod +x /u/12/akyuret1/unix/drone_sim/ekf_*.py
echo "✅ EKF scripts made executable"
echo

# Source ROS environment
unset ROS_DISTRO
source /opt/ros/jazzy/setup.bash

echo "🚁 Starting EKF Gazebo Integration..."
echo "Features:"
echo "  - Real-time EKF state estimation"
echo "  - IMU, GPS, Barometer, Magnetometer fusion"
echo "  - Adaptive noise scaling"
echo "  - Innovation monitoring"
echo "  - Data logging for analysis"
echo

# Start EKF integration
echo "🎮 Starting EKF node..."
python3 /u/12/akyuret1/unix/drone_sim/ekf_gazebo_integration.py &
EKF_PID=$!

echo "✅ EKF node started with PID: $EKF_PID"
echo "📊 EKF estimates published to:"
echo "  - /ekf/pose: Position and orientation estimates"
echo "  - /ekf/velocity: Velocity estimates"
echo "  - /ekf/state: Full state estimates"
echo

# Wait for EKF to initialize
echo "⏳ Waiting for EKF initialization..."
sleep 5

# Check if EKF is running
if ! ps -p $EKF_PID > /dev/null; then
    echo "❌ EKF node failed to start"
    exit 1
fi

echo "✅ EKF integration running successfully!"
echo

# Monitor EKF topics
echo "📡 Monitoring EKF topics..."
echo "Press Ctrl+C to stop and save data"
echo

# Function to handle cleanup
cleanup() {
    echo
    echo "🛑 Stopping EKF integration..."
    kill $EKF_PID 2>/dev/null || true
    echo "✅ EKF integration stopped"
    echo
    
    # Check for log files
    if ls /u/12/akyuret1/unix/drone_sim/ekf_log_*.json 1> /dev/null 2>&1; then
        echo "📊 EKF log files available:"
        ls -la /u/12/akyuret1/unix/drone_sim/ekf_log_*.json
        echo
        echo "🎯 To analyze the data, run:"
        echo "python3 /u/12/akyuret1/unix/drone_sim/analyze_ekf_data.py"
    fi
    
    echo "🎉 EKF integration complete!"
}

# Set up signal handling
trap cleanup INT TERM

# Monitor loop
while ps -p $EKF_PID > /dev/null; do
    sleep 1
done

# If we get here, the process ended naturally
cleanup
