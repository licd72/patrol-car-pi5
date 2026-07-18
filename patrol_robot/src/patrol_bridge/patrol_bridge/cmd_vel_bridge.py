#!/usr/bin/env python3
"""cmd_vel_bridge — 订阅 /cmd_vel, 驱动 STM32 底盘"""
import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sys.path.insert(0, '/home/pi/patrol_robot')
from Rosmaster_Lib import Rosmaster

_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST, depth=1)


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        # 初始化底盘
        self.bot = Rosmaster(car_type=1, com='/dev/myserial')
        time.sleep(0.3)
        
        # 订阅 /cmd_vel (BEST_EFFORT 减少 DDS 流量)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd, _qos)
        self.voltage_pub = self.create_publisher(Float32, '/patrol/voltage', 10)
        
        # 心跳 (20Hz)
        self._last_cmd = (0.0, 0.0, 0.0)
        self._last_time = time.time()
        self._timeout = 0.5
        self._hb = self.create_timer(0.05, self._heartbeat)
        
        # 状态统计 (1Hz)
        self._recv_count = 0
        self._stat = self.create_timer(1.0, self._stat_tick)
        
        self.get_logger().info('bridge ready | QoS=BEST_EFFORT | ROS_DOMAIN_ID=' + 
            __import__('os').environ.get('ROS_DOMAIN_ID', '0'))

    def _on_cmd(self, msg):
        self._last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_time = time.time()
        self._recv_count += 1

    def _heartbeat(self):
        vx, vy, wz = self._last_cmd
        if time.time() - self._last_time > self._timeout:
            vx = vy = wz = 0.0
        try:
            self.bot.set_car_motion(vx, vy, wz)
        except:
            pass

    def _stat_tick(self):
        vx = self._last_cmd[0]
        enc = self.bot.get_motor_encoder() if hasattr(self.bot, 'get_motor_encoder') else (0,0,0,0)
        self.get_logger().info(
            f'recv={self._recv_count} vx={vx:.2f} enc=({enc[0]},{enc[1]},{enc[2]},{enc[3]})')
        self._recv_count = 0


def main():
    rclpy.init()
    rclpy.spin(CmdVelBridge())


if __name__ == '__main__':
    main()
