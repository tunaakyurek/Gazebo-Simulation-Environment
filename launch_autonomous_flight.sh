#!/bin/bash
set -e

echo "🚁 AUTONOMOUS FLIGHT WITH EKF ANALYSIS"
echo "======================================"
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

# Install required Python packages
echo "📦 Installing required Python packages..."
pip install matplotlib scipy pandas numpy --quiet
echo "✅ Python packages installed"
echo

# Make scripts executable
chmod +x /u/12/akyuret1/unix/drone_sim/autonomous_flight_controller.py
chmod +x /u/12/akyuret1/unix/drone_sim/ekf_analysis_plotter.py
echo "✅ Scripts made executable"
echo

# Start autonomous flight
echo "🚁 Starting autonomous flight mission..."
echo "Mission waypoints:"
echo "  1. Takeoff to 2m"
echo "  2. Move forward 5m"
echo "  3. Move right 5m"
echo "  4. Move back 5m"
echo "  5. Return to start"
echo "  6. Land"
echo

# Source ROS environment and start flight controller
unset ROS_DISTRO
source /opt/ros/jazzy/setup.bash

echo "🎮 Starting flight controller..."
python3 /u/12/akyuret1/unix/drone_sim/autonomous_flight_controller.py &
FLIGHT_PID=$!

echo "✅ Flight controller started with PID: $FLIGHT_PID"
echo "📊 Flight data will be recorded automatically"
echo

# Wait for flight to complete (or user interrupt)
echo "Press Ctrl+C to stop the flight and run analysis..."
trap "echo 'Stopping flight...'; kill $FLIGHT_PID 2>/dev/null; echo 'Flight stopped.'" INT

wait $FLIGHT_PID

echo
echo "🎯 Flight completed! Running EKF analysis..."
echo

# Run EKF analysis
python3 /u/12/akyuret1/unix/drone_sim/ekf_analysis_plotter.py

echo
echo "🎉 AUTONOMOUS FLIGHT AND EKF ANALYSIS COMPLETE!"
echo "=============================================="
echo "📊 Analysis results saved to:"
echo "  - 3d_trajectory.png"
echo "  - position_errors.png"
echo "  - velocity_analysis.png"
echo "  - orientation_analysis.png"
echo "  - ekf_performance.png"
echo "  - flight_analysis_report.txt"
echo
echo "🚁 Ready for next autonomous mission!"
