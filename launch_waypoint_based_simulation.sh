#!/bin/bash
set -e

echo "🚁 LAUNCHING WAYPOINT-BASED EKF SIMULATION WITH REALISTIC DYNAMICS"
echo "=================================================================="
echo "This simulation uses:"
echo "✅ Gazebo realistic drone physics and dynamics"
echo "✅ PX4 autonomous flight control with waypoints"
echo "✅ Fine-tuned custom EKF for state estimation"
echo "✅ True trajectory data from Gazebo for comparison"
echo

# Check if PX4 is running
check_px4() {
    if ps aux | grep "PX4_SIM_MODEL=gz_x500" | grep -v grep > /dev/null; then
        echo "✅ PX4 gz_x500 simulation is running"
        return 0
    else
        echo "❌ PX4 gz_x500 simulation not found"
        echo "Please start PX4 first:"
        echo "  cd /u/88/wa.akyuret1/unix/PX4-Autopilot"
        echo "  make px4_sitl gz_x500"
        return 1
    fi
}

# Check if MAVROS is running
check_mavros() {
    if pgrep -f "mavros_node" > /dev/null; then
        echo "✅ MAVROS is running"
        return 0
    else
        echo "❌ MAVROS not running"
        echo "Starting MAVROS..."
        ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580 &
        sleep 5
        echo "✅ MAVROS started"
        return 0
    fi
}

# Main execution
echo "🔍 CHECKING SYSTEM REQUIREMENTS"
echo "==============================="

# Check PX4
if ! check_px4; then
    exit 1
fi

echo

# Check MAVROS
check_mavros

echo
echo "🚀 STARTING WAYPOINT-BASED EKF SIMULATION"
echo "========================================="
echo "Mission Profile:"
echo "  - Figure-8 pattern with altitude variation"
echo "  - 17 waypoints total"
echo "  - Duration: 2 minutes"
echo "  - EKF processing: 100Hz"
echo "  - Using realistic Gazebo dynamics"
echo

python3 waypoint_based_ekf_integration.py

echo
echo "🎉 WAYPOINT-BASED SIMULATION COMPLETE!"
echo "======================================"
echo "📊 Check the generated analysis files:"
echo "  - waypoint_based_ekf_analysis.png"
echo "  - waypoint_based_3d_trajectory.png"
echo "  - waypoint_ekf_data_YYYYMMDD_HHMMSS.json"
