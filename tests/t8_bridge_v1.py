#!/usr/bin/env python3
"""
cmd_vel_bridge_v1 —— 测试版

目的:
  订阅 /cmd_vel → Rosmaster.set_car_motion()
  10Hz heartbeat: 只要 0.5s 内收到过 Twist, 就以 10Hz 重发, 绕过 STM32 watchdog
  超过 0.5s 未收到 → 自动停车

用法:
  python3 t8_bridge_v1.py

⚠️ 独立测试脚本, 不修改任何现有节点
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Rosmaster_Lib import Rosmaster
import time

CMD_TIMEOUT = 0.5   # 0.5秒没新指令就停车
HEARTBEAT_HZ = 10   # 以10Hz重发

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge_v1")
        self.get_logger().info("初始化 Rosmaster @ /dev/myserial")
        self.car = Rosmaster(com="/dev/myserial", debug=False)
        self.car.set_car_type(1)
        self.car.create_receive_threading()
        time.sleep(0.5)

        v = self.car.get_battery_voltage()
        self.get_logger().info(f"底盘就绪 电压={v}V")

        # 状态
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0
        self.last_cmd_time = 0.0

        # /cmd_vel 订阅 (best_effort 减少延迟)
        self.sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        # 10Hz heartbeat timer
        self.timer = self.create_timer(1.0/HEARTBEAT_HZ, self._heartbeat)

        self.get_logger().info(f"✅ 桥就绪 timeout={CMD_TIMEOUT}s heartbeat={HEARTBEAT_HZ}Hz")

    def _on_cmd(self, msg: Twist):
        self.last_vx = msg.linear.x
        self.last_vy = msg.linear.y
        self.last_wz = msg.angular.z
        self.last_cmd_time = time.time()
        # 立即下发一次
        self.car.set_car_motion(self.last_vx, self.last_vy, self.last_wz)

    def _heartbeat(self):
        now = time.time()
        if now - self.last_cmd_time > CMD_TIMEOUT:
            # 超时 → 停车 (只发一次即可, 但每次心跳都发保险)
            if abs(self.last_vx)+abs(self.last_vy)+abs(self.last_wz) > 1e-6:
                self.get_logger().info(f"cmd 超时 {CMD_TIMEOUT}s → 停车")
                self.last_vx=self.last_vy=self.last_wz=0.0
            self.car.set_car_motion(0, 0, 0)
        else:
            # 心跳期内 → 持续重发上一个 twist
            self.car.set_car_motion(self.last_vx, self.last_vy, self.last_wz)

    def destroy_node(self):
        try:
            self.car.set_car_motion(0, 0, 0)
        except: pass
        super().destroy_node()

def main():
    rclpy.init()
    n = CmdVelBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
