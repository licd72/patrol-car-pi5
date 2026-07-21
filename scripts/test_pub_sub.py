#!/usr/bin/env python3
"""test_pub: 用 Python rclpy 发 /scan, 同时订阅验证"""
import rclpy, time
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init()
n = rclpy.create_node("test_pub")

# 发布 (和驱动一样)
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
pub = n.create_publisher(LaserScan, "/test_scan", qos)

# 订阅同一个 topic
count = [0]
def cb(m):
    count[0] += 1
    if count[0] <= 2:
        print(f"RECEIVED: {len(m.ranges)} ranges")

n.create_subscription(LaserScan, "/test_scan", cb, qos)

# 持续发布
for i in range(10):
    msg = LaserScan()
    msg.header.frame_id = "test"
    msg.header.stamp = n.get_clock().now().to_msg()
    msg.angle_min = 0.0; msg.angle_max = 6.28
    msg.angle_increment = 0.0174
    msg.range_min = 0.1; msg.range_max = 8.0
    msg.ranges = [float(i+1)] * 10
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.2)
    time.sleep(0.1)

print(f"TOTAL received: {count[0]}")
n.destroy_node()
rclpy.shutdown()
