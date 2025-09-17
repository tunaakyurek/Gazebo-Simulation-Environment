#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import HilGPS

class HilGpsSpoofer(Node):
    def __init__(self):
        super().__init__('hil_gps_spoofer')
        self.pub = self.create_publisher(HilGPS, '/mavros/hil/gps', 10)
        self.lat, self.lon, self.alt = 47.397742, 8.545594, 488.0
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
    def tick(self):
        msg = HilGPS()
        msg.header.frame_id = 'map'
        msg.fix_type = 3
        msg.geo.latitude  = self.lat
        msg.geo.longitude = self.lon
        msg.geo.altitude  = self.alt
        msg.eph = 50   # ~0.5 m (cm in MAVLink units)
        msg.epv = 80   # ~0.8 m
        msg.vel = 0
        msg.vn = msg.ve = msg.vd = 0
        msg.cog = 0
        msg.satellites_visible = 12
        self.pub.publish(msg)

def main():
    rclpy.init(); n = HilGpsSpoofer()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
