#!/usr/bin/env python3
"""cmd_vel_bridge — /cmd_vel → STM32 底盘驱动 (Fast-DDS, DOMAIN=42)"""
import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist

sys.path.insert(0, '/home/pi/patrol_robot')
from Rosmaster_Lib import Rosmaster


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.bot = Rosmaster(car_type=1, com='/dev/myserial')
        time.sleep(0.3)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self._last_cmd = (0.0, 0.0, 0.0)
        self._last_time = time.time()
        time.sleep(0.3)
        self._hb = self.create_timer(0.05, self._heartbeat)
        self._stat = self.create_timer(1.0, self._stat_tick)
        self._recv_count = 0
        self.get_logger().info('bridge ready | Fast-DDS | DOMAIN=42')

    def _on_cmd(self, msg):
        self._last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_time = time.time()
        self._recv_count += 1

    def _heartbeat(self):
        vx, vy, wz = self._last_cmd
        if time.time() - self._last_time > 0.5:
            vx = vy = wz = 0.0
        self.bot.set_car_motion(vx, vy, wz)

    def _stat_tick(self):
        enc = self.bot.get_motor_encoder()
        v = self._last_cmd[0]
        self.get_logger().info(f'recv={self._recv_count} vx={v:.2f} enc=({enc[0]},{enc[1]},{enc[2]},{enc[3]})')
        self._recv_count = 0


def main():
    rclpy.init()
    rclpy.spin(CmdVelBridge())


if __name__ == '__main__':
    main()
