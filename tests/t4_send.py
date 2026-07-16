#!/usr/bin/env python3
"""测试4: 发一次 Twist 到 /cmd_vel (用来配合 test3 观察)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdSender(Node):
    def __init__(self):
        super().__init__("cmd_vel_test_sender")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

def main():
    rclpy.init()
    n = CmdSender()
    time.sleep(1)
    print("→ 发布 forward 0.15 m/s")
    t = Twist(); t.linear.x = 0.15
    for _ in range(10):    # 发 1 秒 (10Hz)
        n.pub.publish(t)
        time.sleep(0.1)
    print("→ 发布 stop")
    n.pub.publish(Twist())
    time.sleep(0.3)
    print("完成")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
