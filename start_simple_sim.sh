#!/bin/bash
set -e

echo "Starting simple drone simulation..."

# Kill any existing processes
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ros2 launch" 2>/dev/null || true

# Start Gazebo simulation with simple world
echo "Starting Gazebo simulation..."
gz sim -s -r -v 4 /u/12/akyuret1/unix/drone_sim/worlds/simple_world.sdf &
GZ_PID=$!

# Wait for simulation to start
sleep 5

# Check if simulation is running
if ! ps -p $GZ_PID > /dev/null; then
    echo "Failed to start Gazebo simulation"
    exit 1
fi

echo "Gazebo simulation started with PID: $GZ_PID"

# Wait a bit more for topics to be available
sleep 3

# List available topics
echo "Available Gazebo topics:"
gz topic -l

echo "Simulation is running. Press Ctrl+C to stop."
wait $GZ_PID
