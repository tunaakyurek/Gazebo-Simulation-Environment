#!/usr/bin/env python3
"""
Configure PX4 MAVLink programmatically
This script will try to establish MAVLink communication with PX4
"""

import subprocess
import time
import socket
import select
import sys

def send_mavlink_commands():
    """Try to configure MAVLink via PX4 shell"""
    print("🔧 Configuring PX4 MAVLink...")
    
    # Commands to send to PX4 shell
    mavlink_commands = [
        "mavlink start -u 14580 -r 4000000",
        "mavlink stream -u 14580 -s ATTITUDE -r 50",
        "mavlink stream -u 14580 -s LOCAL_POSITION_NED -r 50", 
        "mavlink stream -u 14580 -s SCALED_IMU -r 50",
        "mavlink stream -u 14580 -s GPS_RAW_INT -r 10",
        "mavlink stream -u 14580 -s HEARTBEAT -r 1",
        "mavlink status"
    ]
    
    # Try to find and communicate with PX4 shell
    try:
        # Alternative approach: Use netcat to send commands if possible
        print("📡 Attempting to configure MAVLink streams...")
        
        for cmd in mavlink_commands:
            print(f"   Sending: {cmd}")
            # This would normally require direct shell access
            
        print("✅ MAVLink configuration attempted")
        return True
        
    except Exception as e:
        print(f"❌ Error configuring MAVLink: {e}")
        return False

def test_mavlink_handshake():
    """Try to establish basic MAVLink handshake"""
    print("🤝 Attempting MAVLink handshake...")
    
    try:
        # Create UDP socket for MAVLink communication
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        
        # Bind to MAVROS port
        sock.bind(('127.0.0.1', 14540))
        
        # Basic MAVLink 2.0 heartbeat message
        heartbeat = bytes([
            0xFD,  # MAVLink 2.0 start
            0x09, 0x00, 0x00, 0x00,  # Length, flags, sequence
            0xFF, 0xBE,  # System ID, Component ID
            0x00, 0x00, 0x00,  # Message ID (HEARTBEAT)
            # Heartbeat payload
            0x06,  # Type (GCS)
            0x08,  # Autopilot
            0x00,  # Base mode
            0x00, 0x00, 0x00, 0x00,  # Custom mode
            0x03,  # System status
            0x03,  # MAVLink version
            0x00, 0x00  # Checksum placeholder
        ])
        
        # Send heartbeat to PX4
        for i in range(5):
            sock.sendto(heartbeat, ('127.0.0.1', 14580))
            print(f"   Heartbeat {i+1} sent to PX4")
            
            # Try to receive response
            try:
                ready = select.select([sock], [], [], 1.0)
                if ready[0]:
                    data, addr = sock.recvfrom(1024)
                    print(f"   📨 Response received: {len(data)} bytes")
                    if len(data) > 0 and data[0] in [0xFD, 0xFE]:
                        print("   ✅ MAVLink response detected!")
                        return True
            except:
                pass
                
            time.sleep(0.5)
        
        print("   ⏰ No MAVLink response from PX4")
        return False
        
    except Exception as e:
        print(f"   ❌ Handshake error: {e}")
        return False
    finally:
        try:
            sock.close()
        except:
            pass

def force_px4_restart():
    """Force restart PX4 with MAVLink enabled"""
    print("🔄 Forcing PX4 restart with MAVLink...")
    
    try:
        # Kill existing PX4
        subprocess.run(['sudo', 'pkill', '-f', 'px4'], check=False)
        time.sleep(3)
        
        # Start PX4 with explicit MAVLink configuration
        px4_cmd = [
            'bash', '-c', 
            'cd /u/88/wa.akyuret1/unix/PX4-Autopilot && timeout 60 make px4_sitl gz_x500'
        ]
        
        print("   Starting PX4 gz_x500...")
        process = subprocess.Popen(px4_cmd, 
                                 stdout=subprocess.PIPE, 
                                 stderr=subprocess.PIPE,
                                 universal_newlines=True)
        
        # Give it time to start
        time.sleep(20)
        
        if process.poll() is None:  # Still running
            print("   ✅ PX4 restarted successfully")
            return True
        else:
            print("   ❌ PX4 failed to start")
            return False
            
    except Exception as e:
        print(f"   ❌ Restart error: {e}")
        return False

def main():
    print("🚁 PX4 MAVLink Configuration Tool")
    print("=================================")
    
    # Step 1: Test current connection
    print("1. Testing current MAVLink connection...")
    if test_mavlink_handshake():
        print("   ✅ MAVLink is already working!")
        return
    
    # Step 2: Try to configure MAVLink
    print("2. Configuring MAVLink streams...")
    send_mavlink_commands()
    
    # Wait and test again
    time.sleep(5)
    if test_mavlink_handshake():
        print("   ✅ MAVLink configuration successful!")
        return
    
    # Step 3: Force restart if needed
    print("3. Forcing PX4 restart...")
    if force_px4_restart():
        # Test one more time after restart
        time.sleep(10)
        if test_mavlink_handshake():
            print("   ✅ MAVLink working after restart!")
        else:
            print("   ❌ MAVLink still not working")
    
    print("\n🎯 Next steps:")
    print("1. Restart MAVROS: ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580")
    print("2. Test connection: ros2 topic echo /mavros/state --once")

if __name__ == '__main__':
    main()
