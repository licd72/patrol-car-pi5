#!/usr/bin/env python3
"""cmd_vel_bridge - /cmd_vel => Rosmaster X3 (car_type=1, 4麦轮)
   用独立线程发 set_car_motion，不依赖 rclpy timer 精度"""
import rclpy, sys, time, math, threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
sys.path.insert(0, '/home/pi/patrol_robot')
from Rosmaster_Lib import Rosmaster

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self._last_cmd = (0.0, 0.0, 0.0)
        self._last_time = time.time()
        self._recv = 0
        self._bat = 0.0
        self._running = True

        self.bot = Rosmaster(car_type=1, com='/dev/myserial')
        self.bot.create_receive_threading()
        self.bot.set_auto_report_state(enable=True, forever=False)
        time.sleep(0.3)
        self.get_logger().info('Rosmaster ready (car_type=1)')

        self.sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.voltage_pub = self.create_publisher(Float32, '/patrol/voltage', 10)

        # 独立线程发 heartbeat (10Hz, 不受 rclpy executor 影响)
        self._hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_thread.start()
        # stat 线程
        self._stat_thread = threading.Thread(target=self._stat_loop, daemon=True)
        self._stat_thread.start()

    def _on_cmd(self, msg):
        self._last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_time = time.time()
        self._recv += 1

    def _hb_loop(self):
        while self._running:
            vx, vy, wz = self._last_cmd
            if time.time() - self._last_time > 0.5:
                vx = vy = wz = 0.0
            # 运动补偿
            if abs(vx) < 0.02: vx_c = 0.0
            else: vx_c = vx * 1.05
            if abs(wz) < 0.02: wz_c = 0.0
            else: wz_c = math.copysign(2.33 * abs(wz) + 0.4, wz)
            vx_c = max(-1, min(1, vx_c))
            wz_c = max(-5, min(5, wz_c))
            try:
                self.bot.set_car_motion(vx_c, 0.0, wz_c)
            except:
                pass
            time.sleep(0.05)

    def _stat_loop(self):
        while self._running:
            try:
                bat = self.bot.get_battery_voltage()
                if bat and bat > 0.1:
                    self._bat = bat
                    self.voltage_pub.publish(Float32(data=float(bat)))
            except:
                pass
            self.get_logger().info(
                'recv=%d bat=%.1fV vx=%.2f' % (self._recv, self._bat, self._last_cmd[0])
            )
            self._recv = 0
            time.sleep(1.0)

    def destroy_node(self):
        self._running = False
        time.sleep(0.1)
        try:
            self.bot.set_car_motion(0, 0, 0)
        except:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
