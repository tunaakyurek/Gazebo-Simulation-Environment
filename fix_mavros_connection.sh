#!/bin/bash

echo "🔧 FIXING MAVROS CONNECTION ISSUE"
echo "=================================="

# Step 1: Clean up existing processes
echo "1. Cleaning up existing MAVROS processes..."
pkill -f mavros_node
sleep 2
sudo kill -9 $(ps aux | grep mavros_node | grep -v grep | awk '{print $2}') 2>/dev/null || true

echo "2. Checking PX4 status..."
PX4_COUNT=$(ps aux | grep px4 | grep -v grep | wc -l)
echo "   PX4 processes found: $PX4_COUNT"

# Step 3: Check ports
echo "3. Checking port status..."
ss -ulpn | grep -E "145(40|57|80)" || echo "   No MAVLink ports in use"

# Step 4: Start clean MAVROS with correct port
echo "4. Starting MAVROS with proper configuration..."
echo "   Using FCU URL: udp://:14540@127.0.0.1:14580"

# Start MAVROS in background
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580 &
MAVROS_PID=$!
echo "   MAVROS started with PID: $MAVROS_PID"

# Step 5: Wait and test connection
echo "5. Testing connection..."
sleep 5

echo "6. Checking MAVROS state..."
timeout 3 ros2 topic echo /mavros/state --once || echo "   No state received"

echo "7. Final port check..."
ss -ulpn | grep -E "145(40|57|80)"

echo ""
echo "🎯 CONNECTION FIX COMPLETE"
echo "=========================="
echo "Check the state above. If 'connected: true', MAVROS is working!"
