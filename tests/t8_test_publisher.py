#!/usr/bin/env python3
"""桥+发布者端到端测试 (类型修复)"""
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestPub(Node):
    def __init__(self):
        super().__init__("t8_test_publisher")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

def send(pub, vx, vy, wz, duration, rate=10):
    t = Twist()
    t.linear.x = float(vx); t.linear.y = float(vy); t.linear.z = 0.0
    t.angular.x = 0.0; t.angular.y = 0.0; t.angular.z = float(wz)
    n = int(duration*rate)
    for _ in range(n):
        pub.publish(t)
        time.sleep(1.0/rate)

def main():
    rclpy.init()
    n = TestPub()
    time.sleep(1.0)
    print(">>> 前进 0.2 m/s × 3s"); send(n.pub, 0.2, 0, 0, 3.0)
    print(">>> 停车 1s"); send(n.pub, 0, 0, 0, 1.0)
    print(">>> 原地右转 0.5 rad/s × 3s"); send(n.pub, 0, 0, 0.5, 3.0)
    print(">>> 停车 1s"); send(n.pub, 0, 0, 0, 1.0)
    print(">>> 后退 0.2 m/s × 3s"); send(n.pub, -0.2, 0, 0, 3.0)
    print(">>> 全停"); send(n.pub, 0, 0, 0, 0.5)
    print("完成 ✅")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
