#!/usr/bin/env python3
"""bare_minimum: serial read -> LaserScan publish, no fancy parsing"""
import rclpy, serial, struct, time, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

rclpy.init()
n = rclpy.create_node("bare")
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
pub = n.create_publisher(LaserScan, "/scan", qos)

ser = serial.Serial("/dev/rplidar", 128000, timeout=0.5)
ser.dtr = False; time.sleep(0.2)
ser.write(bytes([0xA5, 0x65])); time.sleep(0.2)
ser.read(500)
ser.write(bytes([0xA5, 0x60])); time.sleep(1.0)

count = 0; t0 = time.time()
while time.time() - t0 < 5:
    data = ser.read(2048)
    if len(data) > 100:
        ranges = []
        for i in range(0, len(data)-1, 2):
            val = struct.unpack("<H", data[i:i+2])[0]
            ranges.append(val / 1000.0)
        if len(ranges) > 10:
            msg = LaserScan()
            msg.header.frame_id = "laser_frame"
            msg.header.stamp = n.get_clock().now().to_msg()
            msg.angle_increment = 2*math.pi/len(ranges)
            msg.angle_min = 0.0; msg.angle_max = 2*math.pi
            msg.range_min = 0.05; msg.range_max = 8.0
            msg.ranges = [r if 0.05<r<8.0 else float("inf") for r in ranges]
            pub.publish(msg)
            count += 1

ser.close()
print("PUBLISHED:", count)
n.destroy_node(); rclpy.shutdown()
