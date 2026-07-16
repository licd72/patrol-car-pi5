#!/usr/bin/env python3
"""cmd_vel_bridge_v2 — 只写不读版
避免与 web_server 竞争串口读缓冲
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Rosmaster_Lib import Rosmaster
import time

CMD_TIMEOUT = 0.5
HEARTBEAT_HZ = 10

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge_v2")
        self.get_logger().info("初始化 Rosmaster @ /dev/myserial (只写模式)")
        self.car = Rosmaster(com="/dev/myserial", debug=False)
        self.car.set_car_type(1)
        # ⚠️ 不调 create_receive_threading, 避免与 web_server 抢读

        self.last_vx = self.last_vy = self.last_wz = 0.0
        self.last_cmd_time = 0.0

        self.sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.timer = self.create_timer(1.0/HEARTBEAT_HZ, self._heartbeat)
        self.get_logger().info(f"✅ 桥就绪 (只写) timeout={CMD_TIMEOUT}s hb={HEARTBEAT_HZ}Hz")

    def _on_cmd(self, msg: Twist):
        self.last_vx = float(msg.linear.x)
        self.last_vy = float(msg.linear.y)
        self.last_wz = float(msg.angular.z)
        self.last_cmd_time = time.time()
        self.car.set_car_motion(self.last_vx, self.last_vy, self.last_wz)

    def _heartbeat(self):
        now = time.time()
        if now - self.last_cmd_time > CMD_TIMEOUT:
            if abs(self.last_vx)+abs(self.last_vy)+abs(self.last_wz) > 1e-6:
                self.get_logger().info(f"超时 → 停车")
                self.last_vx=self.last_vy=self.last_wz=0.0
            self.car.set_car_motion(0, 0, 0)
        else:
            self.car.set_car_motion(self.last_vx, self.last_vy, self.last_wz)

    def destroy_node(self):
        try:
            self.car.set_car_motion(0, 0, 0)
        except: pass
        super().destroy_node()

def main():
    rclpy.init()
    n = CmdVelBridge()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
