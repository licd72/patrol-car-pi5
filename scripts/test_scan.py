#!/usr/bin/env python3
"""test_scan: 验证 /scan 数据是否可接收"""
import rclpy, time
from sensor_msgs.msg import LaserScan

rclpy.init()
n = rclpy.create_node("test_scan")
c = [0]
def cb(m):
    c[0] += 1
    if c[0] <= 2:
        print(f"SCAN: ranges={len(m.ranges)} ang_min={m.angle_min:.2f} frame={m.header.frame_id}")

n.create_subscription(LaserScan, "/scan", cb, 10)
t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
print(f"TOTAL: {c[0]} scans received")
n.destroy_node()
rclpy.shutdown()
