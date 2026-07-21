#!/usr/bin/env python3
"""YDLIDAR X4 driver v4 — raw 2-byte pairs as range data"""
import rclpy, struct, time, math, serial, threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class X4Driver(Node):
    def __init__(self):
        super().__init__("x4_driver")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(LaserScan, "/scan", qos)
        
        self.ser = serial.Serial("/dev/rplidar", 128000, timeout=0.5)
        self.ser.dtr = False
        time.sleep(0.2)
        
        self.ser.write(bytes([0xA5, 0x65])); time.sleep(0.2)
        self.ser.read(500)
        self.ser.write(bytes([0xA5, 0x60])); time.sleep(0.3)
        
        self.get_logger().info("X4 started @ 128000 baud")
        
        self._count = 0
        self._ranges = []
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        self.create_timer(3.0, self._status)
    
    def _status(self):
        self.get_logger().info("published {} scans".format(self._count))
    
    def _loop(self):
        buf = b""
        last_cmd = time.time()
        while self._running:
            try:
                if time.time() - last_cmd > 2:
                    self.ser.write(bytes([0xA5, 0x60]))
                    last_cmd = time.time()
                
                chunk = self.ser.read(2048)
                if not chunk: continue
                buf += chunk
                
                # Find 5AA5 headers
                while len(buf) >= 200:
                    idx = buf.find(b"\x5A\xA5")
                    if idx < 0:
                        buf = buf[-100:]
                        break
                    
                    if idx > 0:
                        # Data between last header and this one
                        if self._ranges:
                            self._publish_scan()
                        self._ranges = []
                        buf = buf[idx:]
                    
                    # Skip header (7 bytes)
                    if len(buf) < 8: break
                    buf = buf[7:]
                    
                    # Read data until next 5AA5 or buffer exhausted
                    next_idx = buf.find(b"\x5A\xA5")
                    if next_idx < 0:
                        # No next header yet — store what we have
                        for i in range(0, len(buf) - 1, 2):
                            if i + 2 <= len(buf):
                                val = struct.unpack("<H", buf[i:i+2])[0]
                                if val > 0:
                                    self._ranges.append(val / 1000.0)  # assume mm
                        break
                    else:
                        # Process up to next header
                        seg = buf[:next_idx]
                        for i in range(0, len(seg) - 1, 2):
                            if i + 2 <= len(seg):
                                val = struct.unpack("<H", seg[i:i+2])[0]
                                if val > 0:
                                    self._ranges.append(val / 1000.0)
                        buf = buf[next_idx:]
                    
            except Exception:
                time.sleep(0.1)
    
    def _publish_scan(self):
        if len(self._ranges) < 10:
            return
        msg = LaserScan()
        msg.header.frame_id = "laser_frame"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        n = len(self._ranges)
        msg.angle_increment = 2 * math.pi / n
        msg.angle_min = 0.0
        msg.angle_max = msg.angle_min + msg.angle_increment * n
        msg.range_min = 0.05
        msg.range_max = 8.0
        msg.ranges = [r if 0.05 < r < 8.0 else float("inf") for r in self._ranges]
        
        self.pub.publish(msg)
        self._count += 1

def main():
    rclpy.init()
    rclpy.spin(X4Driver())

if __name__ == "__main__":
    main()
