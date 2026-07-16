#!/usr/bin/env python3
"""测试3: 临时 /cmd_vel 桥, 订阅 Twist → Rosmaster_Lib
运行期间可让语音/巡逻状态机/web的 /cmd_vel 发布真正生效
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Rosmaster_Lib import Rosmaster

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge_TEST")
        self.get_logger().info("初始化 Rosmaster /dev/myserial ...")
        self.car = Rosmaster(com="/dev/myserial", debug=False)
        self.car.set_car_type(1)
        self.sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd, 10
        )
        self.get_logger().info("✅ 桥接就绪: /cmd_vel → Rosmaster.set_car_motion()")
        self._last = (0.0, 0.0, 0.0)

    def _on_cmd(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        if (vx, vy, wz) != self._last:
            self.get_logger().info(f"/cmd_vel → vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
            self._last = (vx, vy, wz)
        self.car.set_car_motion(vx, vy, wz)

def main():
    rclpy.init()
    n = CmdVelBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        n.car.set_car_motion(0, 0, 0)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
