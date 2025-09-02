#!/bin/bash
set -e

echo "🚁 QUICK SIMULATION SETUP SCRIPT"
echo "================================="
echo "This script will quickly set up the simulation environment for EKF testing"
echo

# Function to cleanup processes
cleanup() {
    echo
    echo "🛑 Cleaning up processes..."
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ros2 run ros_gz_bridge" 2>/dev/null || true
    pkill -f "mavros" 2>/dev/null || true
    pkill -f "px4" 2>/dev/null || true
    sleep 2
    echo "✅ Cleanup complete"
}

# Function to check if processes are running
check_processes() {
    echo "🔍 Checking running processes..."
    
    if pgrep -f "gz sim" > /dev/null; then
        echo "✅ Gazebo simulation is running"
        return 0
    else
        echo "❌ Gazebo simulation is not running"
        return 1
    fi
}

# Function to start minimal simulation
start_minimal_simulation() {
    echo "🚀 Starting minimal simulation environment..."
    
    # Clean up any existing processes
    cleanup
    
    # Start Gazebo simulation
    echo "Starting Gazebo simulation..."
    gz sim -s -r -v 4 /u/12/akyuret1/unix/drone_sim/worlds/simple_world.sdf &
    GZ_PID=$!
    sleep 5
    
    # Check if Gazebo started successfully
    if ! ps -p $GZ_PID > /dev/null; then
        echo "❌ Gazebo failed to start"
        exit 1
    fi
    echo "✅ Gazebo started with PID: $GZ_PID"
    
    # Start ROS bridge
    echo "Starting ROS-Gazebo bridge..."
    unset ROS_DISTRO
    bash -c "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock" &
    BRIDGE_PID=$!
    sleep 3
    echo "✅ ROS bridge started with PID: $BRIDGE_PID"
    
    # Wait for system to stabilize
    echo "⏳ Waiting for system to stabilize..."
    sleep 5
    
    # Check if all processes are running
    if ps -p $GZ_PID > /dev/null && ps -p $BRIDGE_PID > /dev/null; then
        echo "✅ Minimal simulation environment running successfully"
        echo "📡 Gazebo topics available:"
        gz topic -l | head -10
        echo
        echo "🎮 Simulation ready for EKF testing!"
        echo "   - Gazebo PID: $GZ_PID"
        echo "   - Bridge PID: $BRIDGE_PID"
        echo
        echo "Press Ctrl+C to stop the simulation"
        
        # Keep simulation running
        trap "echo 'Stopping simulation...'; kill $GZ_PID $BRIDGE_PID 2>/dev/null; exit 0" INT
        wait
    else
        echo "❌ Some simulation components failed to start"
        cleanup
        exit 1
    fi
}

# Function to start full simulation
start_full_simulation() {
    echo "🚀 Starting full simulation environment (PX4 spawns Gazebo)..."
    
    # Clean up any existing processes
    cleanup
    
    # Start PX4 SITL with Gazebo x500 via environment variables
    echo "Starting PX4 SITL (gz_x500 model)..."
    cd /u/12/akyuret1/unix/PX4-Autopilot
    PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=default PX4_GZ_MODEL_POSE="0,0,2,0,0,0" make px4_sitl &
    PX4_PID=$!
    sleep 10
    echo "✅ PX4 SITL started with PID: $PX4_PID"
    
    # Start MAVROS
    echo "Starting MAVROS..."
    unset ROS_DISTRO
    bash -c "source /opt/ros/jazzy/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580" &
    MAVROS_PID=$!
    sleep 5
    echo "✅ MAVROS started with PID: $MAVROS_PID"
    
    # Wait for system to stabilize
    echo "⏳ Waiting for system to stabilize..."
    sleep 10
    
    # Check if PX4 and MAVROS are running
    if ps -p $PX4_PID > /dev/null && ps -p $MAVROS_PID > /dev/null; then
        echo "✅ PX4 + MAVROS running successfully"
        echo "📡 MAVROS topics (subset):"
        unset ROS_DISTRO
        bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list | grep -E '(mavros|uas1)' | head -20"
        echo
        echo "Press Ctrl+C to stop"
        
        # Keep running
        trap "echo 'Stopping...'; kill $PX4_PID $MAVROS_PID 2>/dev/null; exit 0" INT
        wait
    else
        echo "❌ Some components failed to start (PX4 or MAVROS)"
        cleanup
        exit 1
    fi
}

# Main execution
case "${1:-minimal}" in
    "minimal")
        start_minimal_simulation
        ;;
    "full")
        start_full_simulation
        ;;
    "check")
        check_processes
        ;;
    "cleanup")
        cleanup
        ;;
    *)
        echo "Usage: $0 [minimal|full|check|cleanup]"
        echo "  minimal: Start minimal simulation (Gazebo + ROS bridge)"
        echo "  full:    Start full simulation (Gazebo + ROS bridge + PX4 + MAVROS)"
        echo "  check:   Check if simulation is running"
        echo "  cleanup: Stop all simulation processes"
        exit 1
        ;;
esac
