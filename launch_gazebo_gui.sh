#!/bin/bash
set -e

echo "🚁 GAZEBO GUI SIMULATION LAUNCHER"
echo "=================================="
echo

# Check if we're in the right environment
echo "🔍 Checking environment..."

# Kill any existing Gazebo processes
echo "🧹 Cleaning up existing Gazebo processes..."
pkill -f "gz sim" 2>/dev/null || true
pkill -f "gazebo" 2>/dev/null || true
sleep 2

# Check if we have the necessary models and worlds
echo "📁 Checking simulation assets..."
if [ ! -d "/u/12/akyuret1/unix/drone_sim/models" ]; then
    echo "❌ Models directory not found. Creating basic models..."
    mkdir -p /u/12/akyuret1/unix/drone_sim/models
fi

if [ ! -d "/u/12/akyuret1/unix/drone_sim/worlds" ]; then
    echo "❌ Worlds directory not found. Creating basic world..."
    mkdir -p /u/12/akyuret1/unix/drone_sim/worlds
fi

# Create a simple world file for GUI simulation
echo "🌍 Creating simulation world..."
cat > /u/12/akyuret1/unix/drone_sim/worlds/ekf_demo_world.sdf << 'EOF'
<?xml version='1.0'?>
<sdf version='1.8'>
  <world name='ekf_demo_world'>
    <!-- Ground plane -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Lighting -->
    <light name='sun' type='directional'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Physics -->
    <physics name='1ms' type='ignored'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- GUI -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>5 -5 2 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
EOF

echo "✅ Simulation world created"
echo

# Create a simple drone model for visualization
echo "🚁 Creating drone model..."
mkdir -p /u/12/akyuret1/unix/drone_sim/models/simple_drone

cat > /u/12/akyuret1/unix/drone_sim/models/simple_drone/model.sdf << 'EOF'
<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='simple_drone'>
    <pose>0 0 1 0 0 0</pose>
    <static>false</static>
    
    <link name='base_link'>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.5 0.5 0.1</size>
          </box>
        </geometry>
      </collision>
      
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.5 0.5 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      
      <!-- IMU sensor -->
      <sensor name='imu_sensor' type='imu'>
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type='gaussian'>
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type='gaussian'>
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type='gaussian'>
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type='gaussian'>
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type='gaussian'>
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type='gaussian'>
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
        <plugin filename='gz-sim-imu-system' name='gz::sim::systems::Imu'>
          <topic>/sim/imu</topic>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
EOF

echo "✅ Drone model created"
echo

# Set up Gazebo environment
export GZ_SIM_RESOURCE_PATH="/u/12/akyuret1/unix/drone_sim:$GZ_SIM_RESOURCE_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"

echo "🚀 Starting Gazebo GUI simulation..."
echo "Features:"
echo "  - Interactive 3D visualization"
echo "  - Real-time physics simulation"
echo "  - Drone model with IMU sensor"
echo "  - Ground plane and lighting"
echo

# Launch Gazebo with GUI
echo "🎮 Launching Gazebo GUI..."
gz sim -v 4 -r /u/12/akyuret1/unix/drone_sim/worlds/ekf_demo_world.sdf &
GAZEBO_PID=$!

echo "✅ Gazebo GUI started with PID: $GAZEBO_PID"
echo "📱 GUI should be opening in a new window..."
echo

# Wait for Gazebo to start
echo "⏳ Waiting for Gazebo to initialize..."
sleep 5

# Check if Gazebo is running
if ! ps -p $GAZEBO_PID > /dev/null; then
    echo "❌ Gazebo failed to start"
    exit 1
fi

echo "✅ Gazebo GUI is running successfully!"
echo
echo "🎯 Next steps:"
echo "1. The Gazebo GUI window should be open"
echo "2. You should see a ground plane and a blue drone model"
echo "3. Run the EKF integration to see trajectory visualization"
echo "4. Use the mouse to navigate the 3D view"
echo

# Function to handle cleanup
cleanup() {
    echo
    echo "🛑 Stopping Gazebo GUI..."
    kill $GAZEBO_PID 2>/dev/null || true
    echo "✅ Gazebo GUI stopped"
}

# Set up signal handling
trap cleanup INT TERM

echo "Press Ctrl+C to stop Gazebo GUI"
echo "The GUI window will remain open for interaction"

# Keep the script running
wait $GAZEBO_PID
