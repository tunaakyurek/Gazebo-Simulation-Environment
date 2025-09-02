#!/bin/bash
set -e

echo "🚁 COMPLETE GUI SIMULATION WITH EKF TRAJECTORY VISUALIZATION"
echo "============================================================="
echo

# Function to handle cleanup
cleanup() {
    echo
    echo "🛑 Stopping all processes..."
    
    # Kill all background processes
    jobs -p | xargs -r kill 2>/dev/null || true
    
    # Kill specific processes
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ekf_gazebo_integration" 2>/dev/null || true
    pkill -f "ekf_trajectory_visualizer" 2>/dev/null || true
    pkill -f "mavros" 2>/dev/null || true
    
    echo "✅ All processes stopped"
    echo "🎉 GUI simulation complete!"
}

# Set up signal handling
trap cleanup INT TERM

# Check prerequisites
echo "🔍 Checking prerequisites..."

# Check if MAVROS is running
if ! pgrep -f "mavros_node" > /dev/null; then
    echo "❌ MAVROS is not running. Starting MAVROS..."
    unset ROS_DISTRO
    source /opt/ros/jazzy/setup.bash
    ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580 &
    MAVROS_PID=$!
    echo "✅ MAVROS started with PID: $MAVROS_PID"
    sleep 5
else
    echo "✅ MAVROS is already running"
fi

# Make scripts executable
chmod +x /u/12/akyuret1/unix/drone_sim/*.py
chmod +x /u/12/akyuret1/unix/drone_sim/*.sh
echo "✅ Scripts made executable"

# Source ROS environment
unset ROS_DISTRO
source /opt/ros/jazzy/setup.bash

echo
echo "🚀 Starting complete GUI simulation..."
echo "Components:"
echo "  1. Gazebo GUI simulation"
echo "  2. EKF state estimation"
echo "  3. Real-time trajectory visualization"
echo "  4. MAVROS bridge"
echo

# Step 1: Launch Gazebo GUI
echo "🎮 Step 1: Launching Gazebo GUI..."
/u/12/akyuret1/unix/drone_sim/launch_gazebo_gui.sh &
GAZEBO_PID=$!
echo "✅ Gazebo GUI started with PID: $GAZEBO_PID"

# Wait for Gazebo to initialize
echo "⏳ Waiting for Gazebo to initialize..."
sleep 10

# Check if Gazebo is running
if ! ps -p $GAZEBO_PID > /dev/null; then
    echo "❌ Gazebo failed to start"
    exit 1
fi

echo "✅ Gazebo GUI is running successfully!"
echo

# Step 2: Launch EKF integration
echo "🧠 Step 2: Starting EKF integration..."
python3 /u/12/akyuret1/unix/drone_sim/ekf_gazebo_integration.py &
EKF_PID=$!
echo "✅ EKF integration started with PID: $EKF_PID"

# Wait for EKF to initialize
echo "⏳ Waiting for EKF to initialize..."
sleep 5

# Step 3: Launch trajectory visualizer
echo "📊 Step 3: Starting trajectory visualizer..."
python3 /u/12/akyuret1/unix/drone_sim/ekf_trajectory_visualizer.py &
VIZ_PID=$!
echo "✅ Trajectory visualizer started with PID: $VIZ_PID"

echo
echo "🎉 COMPLETE GUI SIMULATION IS RUNNING!"
echo "======================================"
echo
echo "📱 What you should see:"
echo "  - Gazebo GUI window with 3D simulation"
echo "  - Blue drone model on ground plane"
echo "  - Real-time EKF trajectory visualization"
echo "  - Uncertainty ellipsoids around drone"
echo "  - Mission waypoints displayed"
echo
echo "🎮 Controls:"
echo "  - Mouse: Rotate, zoom, pan the 3D view"
echo "  - Middle mouse: Pan"
echo "  - Right mouse: Zoom"
echo "  - Left mouse: Rotate"
echo
echo "📊 Visualization topics:"
echo "  - /ekf/trajectory: Green trajectory line"
echo "  - /ekf/uncertainty: Uncertainty ellipsoids"
echo "  - /ekf/current_pose: Red arrow showing current pose"
echo "  - /ekf/waypoints: Blue spheres for waypoints"
echo
echo "🔍 To view in RViz (optional):"
echo "  ros2 run rviz2 rviz2"
echo "  Add Marker displays for the topics above"
echo
echo "Press Ctrl+C to stop all components"

# Monitor processes
while true; do
    # Check if any process died
    if ! ps -p $GAZEBO_PID > /dev/null; then
        echo "❌ Gazebo process died"
        break
    fi
    
    if ! ps -p $EKF_PID > /dev/null; then
        echo "❌ EKF process died"
        break
    fi
    
    if ! ps -p $VIZ_PID > /dev/null; then
        echo "❌ Visualizer process died"
        break
    fi
    
    sleep 1
done

# Cleanup will be called by signal handler
