#!/usr/bin/env python3
"""
Test PX4 MAVLink connection directly
This will help us understand if PX4 is actually sending MAVLink messages
"""

import socket
import time
import struct

def test_px4_mavlink_port():
    """Test if PX4 is sending MAVLink messages on port 14580"""
    print("🔍 Testing PX4 MAVLink on port 14580...")
    
    try:
        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(5.0)  # 5 second timeout
        
        # Bind to port 14540 (where MAVROS listens)
        sock.bind(('127.0.0.1', 14540))
        print("✅ Successfully bound to port 14540")
        
        # Try to receive data
        print("🔄 Listening for MAVLink messages for 10 seconds...")
        start_time = time.time()
        message_count = 0
        
        while time.time() - start_time < 10.0:
            try:
                data, addr = sock.recvfrom(1024)
                message_count += 1
                
                if message_count <= 3:  # Show first 3 messages
                    print(f"📨 Received {len(data)} bytes from {addr}: {data[:20].hex()}")
                
                # Check if it looks like MAVLink (starts with 0xFD for MAVLink 2.0)
                if len(data) > 0 and data[0] == 0xFD:
                    print(f"✅ MAVLink 2.0 message detected!")
                elif len(data) > 0 and data[0] == 0xFE:
                    print(f"✅ MAVLink 1.0 message detected!")
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"❌ Error receiving: {e}")
                break
        
        print(f"📊 Total messages received: {message_count}")
        
        if message_count > 0:
            print("✅ PX4 is sending MAVLink messages!")
            return True
        else:
            print("❌ No MAVLink messages received from PX4")
            return False
            
    except Exception as e:
        print(f"❌ Error testing MAVLink: {e}")
        return False
    finally:
        try:
            sock.close()
        except:
            pass

def test_send_mavlink_request():
    """Send a heartbeat request to PX4"""
    print("\n🔍 Sending heartbeat request to PX4...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3.0)
        
        # MAVLink 2.0 heartbeat request (simplified)
        # This is a basic heartbeat message
        heartbeat = bytes([
            0xFD,  # MAVLink 2.0 start byte
            0x09,  # Payload length
            0x00,  # Incompatible flags
            0x00,  # Compatible flags
            0x00,  # Sequence
            0xFF,  # System ID (255 = ground station)
            0xBE,  # Component ID (190 = ground control station)
            0x00, 0x00, 0x00,  # Message ID (0 = heartbeat)
            # Payload (9 bytes for heartbeat)
            0x06,  # Type (MAV_TYPE_GCS)
            0x08,  # Autopilot (MAV_AUTOPILOT_INVALID)
            0x00,  # Base mode
            0x00, 0x00, 0x00, 0x00,  # Custom mode
            0x03,  # System status
            0x03   # MAVLink version
        ])
        
        # Add checksum (simplified - would normally be calculated)
        heartbeat += bytes([0x00, 0x00])  # Placeholder checksum
        
        # Send to PX4
        sock.sendto(heartbeat, ('127.0.0.1', 14580))
        print("📤 Heartbeat sent to PX4")
        
        # Listen for response
        try:
            data, addr = sock.recvfrom(1024)
            print(f"📨 Response received: {len(data)} bytes from {addr}")
            return True
        except socket.timeout:
            print("⏰ No response from PX4 (timeout)")
            return False
            
    except Exception as e:
        print(f"❌ Error sending heartbeat: {e}")
        return False
    finally:
        try:
            sock.close()
        except:
            pass

def main():
    print("🚁 PX4 MAVLink Connection Test")
    print("==============================")
    
    # Test 1: Listen for PX4 messages
    px4_sending = test_px4_mavlink_port()
    
    # Test 2: Try to send a request to PX4
    px4_responding = test_send_mavlink_request()
    
    print("\n🎯 Test Results:")
    print("================")
    print(f"PX4 sending MAVLink: {'✅ Yes' if px4_sending else '❌ No'}")
    print(f"PX4 responding to requests: {'✅ Yes' if px4_responding else '❌ No'}")
    
    if not px4_sending:
        print("\n🔧 Possible Solutions:")
        print("1. PX4 MAVLink might not be started")
        print("2. PX4 might be using a different port")
        print("3. PX4 might need MAVLink configuration")
        print("4. Check PX4 startup scripts for MAVLink setup")

if __name__ == '__main__':
    main()
