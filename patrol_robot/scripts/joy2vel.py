#!/usr/bin/env python3
"""Joy→/cmd_vel_joy — 只在摇杆动时发布"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

DEAD = 0.15
SCALE_LIN = 0.3
SCALE_ANG = 0.5
TIMEOUT = 0.3  # twist_mux joy 超时

class Joy2Vel(Node):
    def __init__(self):
        super().__init__("joy2vel")
        self.sub = self.create_subscription(Joy, "/joy", self.cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel_joy", 10)
        self._last = (0.0, 0.0)  # (vx, wz)
        self._active = False
        self.create_timer(TIMEOUT, self._check)

    def cb(self, msg):
        vx = msg.axes[1] * SCALE_LIN if abs(msg.axes[1]) > DEAD else 0.0
        wz = -msg.axes[0] * SCALE_ANG if abs(msg.axes[0]) > DEAD else 0.0
        self._last = (vx, wz)
        if vx != 0.0 or wz != 0.0:
            self._active = True
            tw = Twist(); tw.linear.x = vx; tw.angular.z = wz
            self.pub.publish(tw)

    def _check(self):
        if self._active:
            self._active = False
            # 停发一次零速让 twist_mux 超时切换到 others
            tw = Twist()
            self.pub.publish(tw)

rclpy.init()
n = Joy2Vel()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
n.destroy_node()
rclpy.shutdown()
