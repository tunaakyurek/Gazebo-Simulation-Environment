# 🚁 MULTI-TERMINAL SOLUTION FOR PX4-MAVROS CONNECTION

## Problem Identified
- **MAVROS is running but cannot connect to PX4** (`connected: false`)
- **PX4 gz_x500 is running but MAVLink is not properly configured**
- **Multiple user processes** (wa.akyuret1 vs akyuret1) causing permission issues

## Solution: Run in 3 Terminals

### Terminal 1: Start PX4 gz_x500 (User: wa.akyuret1 - admin)
```bash
# Clean up any existing PX4 processes
cd /u/88/wa.akyuret1/unix/PX4-Autopilot
sudo pkill -f px4
sudo pkill -f gz
sleep 3

# Start PX4 SITL with gz_x500
make px4_sitl gz_x500
```

**Wait for PX4 shell prompt `pxh>`**, then configure MAVLink:
```bash
pxh> mavlink start -u 14580 -r 4000000
pxh> mavlink stream -u 14580 -s ATTITUDE -r 50
pxh> mavlink stream -u 14580 -s LOCAL_POSITION_NED -r 50
pxh> mavlink stream -u 14580 -s SCALED_IMU -r 50
pxh> mavlink stream -u 14580 -s GPS_RAW_INT -r 10
pxh> mavlink status
```

### Terminal 2: Start MAVROS (User: akyuret1 - normal)
**Wait 30 seconds after PX4 is fully started**, then:
```bash
cd /u/12/akyuret1/unix/drone_sim

# Clean up any existing MAVROS
pkill -f mavros_node
sleep 2

# Start MAVROS with correct FCU URL
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580
```

**Look for this message**: `[INFO] [mavros]: MAVROS UAS started`

### Terminal 3: Test Connection (User: akyuret1 - normal)
**Wait 10 seconds after MAVROS starts**, then:
```bash
cd /u/12/akyuret1/unix/drone_sim

# Test MAVROS connection
echo "Testing MAVROS state..."
ros2 topic echo /mavros/state --once

# If connected: true, test data flow
echo "Testing IMU data..."
timeout 5 ros2 topic hz /mavros/imu/data

echo "Testing pose data..."
timeout 5 ros2 topic hz /mavros/local_position/pose

# If data is flowing, run the EKF integration
python3 real_px4_ekf_integration.py 45
```

## Expected Results

### Terminal 1 (PX4):
```
pxh> mavlink status
instance #0:
        GCS heartbeat: 789654 us ago
        mavlink chan: #0
        type: UDP Server
        flow_control: ON
        rates:
        tx: 5.2 KB/s
        rx: 1.1 KB/s
```

### Terminal 2 (MAVROS):
```
[INFO] [mavros]: MAVROS UAS started
[INFO] [mavros.gps]: CON: GPS fix acquired
[INFO] [mavros.sys]: CON: Got HEARTBEAT, connected.
```

### Terminal 3 (Test):
```
connected: true
armed: false
guided: false
```

## If Still Not Working

If you still get `connected: false`, try these alternative ports:

### In Terminal 1 (PX4):
```bash
pxh> mavlink start -u 14557 -r 4000000
```

### In Terminal 2 (MAVROS):
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557
```

## Success Criteria
✅ `connected: true` in MAVROS state  
✅ IMU data flowing at ~50 Hz  
✅ Pose data flowing at ~50 Hz  
✅ EKF integration runs without "0 data points"
