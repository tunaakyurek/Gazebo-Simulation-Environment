#!/bin/bash
set -e

echo "🚁 COMPLETE AUTONOMOUS DRONE SYSTEM LAUNCHER"
echo "============================================="
echo "This script will launch:"
echo "1. Gazebo simulation with autonomous drone"
echo "2. ROS-Gazebo bridge"
echo "3. Autonomous flight controller"
echo "4. Real-time EKF integration"
echo "5. Real-time visualization"
echo

# Function to cleanup processes
cleanup() {
    echo
    echo "🛑 Cleaning up processes..."
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ros2 run ros_gz_bridge" 2>/dev/null || true
    pkill -f "autonomous_flight_controller" 2>/dev/null || true
    pkill -f "realtime_ekf_integration" 2>/dev/null || true
    pkill -f "python3.*autonomous_flight_controller" 2>/dev/null || true
    pkill -f "python3.*realtime_ekf_integration" 2>/dev/null || true
    sleep 2
    echo "✅ Cleanup complete"
}

# Function to check if processes are running
check_processes() {
    echo "🔍 Checking running processes..."
    
    local all_running=true
    
    if pgrep -f "gz sim" > /dev/null; then
        echo "✅ Gazebo simulation is running"
    else
        echo "❌ Gazebo simulation is not running"
        all_running=false
    fi
    
    if pgrep -f "ros_gz_bridge" > /dev/null; then
        echo "✅ ROS-Gazebo bridge is running"
    else
        echo "❌ ROS-Gazebo bridge is not running"
        all_running=false
    fi
    
    if pgrep -f "autonomous_flight_controller" > /dev/null; then
        echo "✅ Autonomous flight controller is running"
    else
        echo "❌ Autonomous flight controller is not running"
        all_running=false
    fi
    
    if pgrep -f "realtime_ekf_integration" > /dev/null; then
        echo "✅ Real-time EKF integration is running"
    else
        echo "❌ Real-time EKF integration is not running"
        all_running=false
    fi
    
    return $all_running
}

# Function to start the complete system
start_complete_system() {
    echo "🚀 Starting complete autonomous drone system..."
    
    # Clean up any existing processes
    cleanup
    
    # Set up environment
    export GZ_SIM_RESOURCE_PATH="/u/12/akyuret1/unix/drone_sim:$GZ_SIM_RESOURCE_PATH"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"
    
    # Start Gazebo simulation
    echo "🌍 Starting Gazebo simulation..."
    gz sim -s -r -v 4 /u/12/akyuret1/unix/drone_sim/worlds/autonomous_drone_world.sdf &
    GZ_PID=$!
    sleep 5
    
    # Check if Gazebo started successfully
    if ! ps -p $GZ_PID > /dev/null; then
        echo "❌ Gazebo failed to start"
        exit 1
    fi
    echo "✅ Gazebo started with PID: $GZ_PID"
    
    # Start ROS-Gazebo bridge
    echo "🌉 Starting ROS-Gazebo bridge..."
    bash -c "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock /sim/imu_raw@sensor_msgs/msg/Imu@gz.msgs.IMU /sim/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat /autonomous_drone/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose /autonomous_drone/velocity@geometry_msgs/msg/TwistStamped@gz.msgs.Twist /autonomous_drone/command/motor_speed@gz_msgs/msg/Float32Array@gz.msgs.Float32Array" &
    BRIDGE_PID=$!
    sleep 3
    echo "✅ ROS-Gazebo bridge started with PID: $BRIDGE_PID"
    
    # Wait for system to stabilize
    echo "⏳ Waiting for system to stabilize..."
    sleep 5
    
    # Start autonomous flight controller
    echo "🎮 Starting autonomous flight controller..."
    bash -c "source /opt/ros/jazzy/setup.bash && cd /u/12/akyuret1/unix/drone_sim && python3 autonomous_flight_controller.py" &
    FLIGHT_PID=$!
    sleep 2
    echo "✅ Autonomous flight controller started with PID: $FLIGHT_PID"
    
    # Start real-time EKF integration
    echo "🧠 Starting real-time EKF integration..."
    bash -c "source /opt/ros/jazzy/setup.bash && cd /u/12/akyuret1/unix/drone_sim && python3 realtime_ekf_integration.py" &
    EKF_PID=$!
    sleep 2
    echo "✅ Real-time EKF integration started with PID: $EKF_PID"
    
    # Wait for all components to start
    echo "⏳ Waiting for all components to initialize..."
    sleep 5
    
    # Check if all processes are running
    if ps -p $GZ_PID > /dev/null && ps -p $BRIDGE_PID > /dev/null && ps -p $FLIGHT_PID > /dev/null && ps -p $EKF_PID > /dev/null; then
        echo "✅ Complete autonomous drone system running successfully!"
        echo
        echo "📊 System Components:"
        echo "   - Gazebo simulation: PID $GZ_PID"
        echo "   - ROS-Gazebo bridge: PID $BRIDGE_PID"
        echo "   - Flight controller: PID $FLIGHT_PID"
        echo "   - EKF integration: PID $EKF_PID"
        echo
        echo "📡 Available ROS topics:"
        bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list" | grep -E "(sim|ekf|autonomous_drone)" | head -10
        echo
        echo "🎯 System Features:"
        echo "   - Autonomous flight with figure-8 trajectory"
        echo "   - Real-time sensor data processing"
        echo "   - Custom EKF state estimation"
        echo "   - Real-time visualization and analysis"
        echo "   - Data logging for post-processing"
        echo
        echo "📈 Real-time plots should be visible showing:"
        echo "   - Position estimation (true vs EKF)"
        echo "   - Velocity estimation"
        echo "   - Attitude estimation"
        echo "   - 3D trajectory visualization"
        echo
        echo "Press Ctrl+C to stop the complete system"
        
        # Keep system running
        trap "echo 'Stopping complete system...'; kill $GZ_PID $BRIDGE_PID $FLIGHT_PID $EKF_PID 2>/dev/null; cleanup; exit 0" INT
        wait
    else
        echo "❌ Some system components failed to start"
        cleanup
        exit 1
    fi
}

# Function to start minimal system (for testing)
start_minimal_system() {
    echo "🚀 Starting minimal system for testing..."
    
    cleanup
    
    # Start only Gazebo and bridge
    echo "🌍 Starting Gazebo simulation..."
    gz sim -s -r -v 4 /u/12/akyuret1/unix/drone_sim/worlds/autonomous_drone_world.sdf &
    GZ_PID=$!
    sleep 5
    
    echo "🌉 Starting ROS-Gazebo bridge..."
    bash -c "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock" &
    BRIDGE_PID=$!
    sleep 3
    
    echo "✅ Minimal system running"
    echo "   - Gazebo PID: $GZ_PID"
    echo "   - Bridge PID: $BRIDGE_PID"
    
    trap "echo 'Stopping minimal system...'; kill $GZ_PID $BRIDGE_PID 2>/dev/null; cleanup; exit 0" INT
    wait
}

# Function to show system status
show_status() {
    echo "📊 System Status:"
    check_processes
    echo
    echo "📡 ROS Topics:"
    bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list" | grep -E "(sim|ekf|autonomous_drone)" || echo "No relevant topics found"
    echo
    echo "📁 Data Files:"
    ls -la *.json 2>/dev/null | tail -5 || echo "No data files found"
}

# Main execution
case "${1:-complete}" in
    "complete")
        start_complete_system
        ;;
    "minimal")
        start_minimal_system
        ;;
    "status")
        show_status
        ;;
    "check")
        check_processes
        ;;
    "cleanup")
        cleanup
        ;;
    *)
        echo "Usage: $0 [complete|minimal|status|check|cleanup]"
        echo "  complete: Start complete autonomous drone system (default)"
        echo "  minimal:  Start minimal system (Gazebo + bridge only)"
        echo "  status:   Show system status and topics"
        echo "  check:    Check if processes are running"
        echo "  cleanup:  Stop all processes"
        exit 1
        ;;
esac
