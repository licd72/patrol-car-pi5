#!/usr/bin/env python3
"""cmd_vel_bridge — /cmd_vel => FIFO => serial_driver"""
import rclpy, time, json, os
from rclpy.node import Node
from geometry_msgs.msg import Twist

FIFO = '/tmp/cmd_fifo'

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self._last_cmd = (0.0, 0.0, 0.0)
        self._last_time = time.time()
        self._recv_count = 0
        self.sub = self.create_subscription(Twist, '/cmd_vel_out', self._on_cmd, 10)
        self._hb = self.create_timer(0.05, self._heartbeat)
        self._stat = self.create_timer(1.0, self._stat_tick)
        self.get_logger().info('bridge ready | FIFO mode')

    def _on_cmd(self, msg):
        self._last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_time = time.time()
        self._recv_count += 1

    def _heartbeat(self):
        vx, vy, wz = self._last_cmd
        if time.time() - self._last_time > 0.5:
            vx = vy = wz = 0.0
        try:
            with open(FIFO, 'w') as f:
                f.write(json.dumps({'vx': vx, 'vy': vy, 'wz': wz}) + '\n')
        except:
            pass

    def _stat_tick(self):
        self.get_logger().info(f'recv={self._recv_count}')
        self._recv_count = 0

def main():
    rclpy.init()
    rclpy.spin(CmdVelBridge())

if __name__ == '__main__':
    main()
