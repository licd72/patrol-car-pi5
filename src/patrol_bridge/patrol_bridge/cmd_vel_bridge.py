#!/usr/bin/env python3
"""
cmd_vel_bridge (产品版 v1.0)

设计目的:
  1. 系统内唯一 /cmd_vel 订阅者 → 唯一电机 owner (避免多进程写 STM32 串口)
  2. 10Hz heartbeat 持续重发最后一条 Twist → 对抗 STM32 底盘 ~100ms watchdog
  3. 0.5s 未收到新命令 → 自动停车
  4. 只写模式 (不调 create_receive_threading) → 状态查询由其他节点/web 负责

ROS2 接口:
  Subscribers:
    /cmd_vel  (geometry_msgs/Twist)
  Publishers:  (无, 只写 STM32)

依赖:
  Rosmaster_Lib (镜像内自带)
  串口: /dev/myserial (udev 绑定 STM32 CH340)

由 container_init.sh 在其他节点之前启动 (作为底盘接入层)
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")

        # 参数
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("cmd_timeout", 0.5)      # 无新指令超时秒
        self.declare_parameter("heartbeat_hz", 10.0)     # STM32 watchdog ~100ms

        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.cmd_timeout = self.get_parameter("cmd_timeout").get_parameter_value().double_value
        self.heartbeat_hz = self.get_parameter("heartbeat_hz").get_parameter_value().double_value

        # 初始化底盘
        try:
            from Rosmaster_Lib import Rosmaster
        except ImportError as e:
            self.get_logger().fatal(f"Rosmaster_Lib 缺失: {e}")
            raise

        self.get_logger().info(f"初始化 Rosmaster @ {self.serial_port} (只写)")
        self.car = Rosmaster(com=self.serial_port, debug=False)
        self.car.set_car_type(1)   # X3 麦克纳姆
        time.sleep(0.3)

        # ⚠️ 不启动接收线程 (避免与 web_server 状态查询抢串口读缓冲)
        # 状态查询由 web_server._get_chassis 单独打开只读实例负责

        # 状态
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0
        self.last_cmd_time = 0.0
        self._stopped = True     # 用于优化: 已经停了就不重发停车

        # 统计 (每秒打印一次)
        self._recv_count = 0
        self._hb_count = 0
        self._last_stat_time = time.time()


        # 订阅
        self.sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        # heartbeat
        period = 1.0 / self.heartbeat_hz
        self.timer = self.create_timer(period, self._heartbeat)

        self.get_logger().info(
            f"桥就绪 timeout={self.cmd_timeout}s hb={self.heartbeat_hz}Hz "
            f"port={self.serial_port}"
        )

    def _on_cmd(self, msg: Twist):
        self._recv_count += 1
        self.last_vx = float(msg.linear.x)
        self.last_vy = float(msg.linear.y)
        self.last_wz = float(msg.angular.z)
        self.last_cmd_time = time.time()
        # 立即下发一次
        try:
            self.car.set_car_motion(self.last_vx, self.last_vy, self.last_wz)
            self._stopped = (
                abs(self.last_vx) < 1e-6 and
                abs(self.last_vy) < 1e-6 and
                abs(self.last_wz) < 1e-6
            )
        except Exception as e:
            self.get_logger().warn(f"set_car_motion 失败: {e}")

    def _heartbeat(self):
        # 每秒 stat
        self._hb_count += 1
        now = time.time()
        if now - self._last_stat_time >= 1.0:
            self.get_logger().info(
                f"stat: recv={self._recv_count} hb={self._hb_count} "
                f"vx={self.last_vx:.2f} vy={self.last_vy:.2f} wz={self.last_wz:.2f} "
                f"stopped={self._stopped}"
            )
            self._recv_count = 0
            self._hb_count = 0
            self._last_stat_time = now

        if now - self.last_cmd_time > self.cmd_timeout:
            # 超时: 强制清零 (若未停)
            if not self._stopped:
                self.get_logger().info(
                    f"cmd_vel 超时 {self.cmd_timeout}s → 停车"
                )
                self.last_vx = self.last_vy = self.last_wz = 0.0
                self._stopped = True
            try:
                self.car.set_car_motion(0.0, 0.0, 0.0)
            except Exception:
                pass
        else:
            # 心跳内: 持续重发最新 twist
            try:
                self.car.set_car_motion(self.last_vx, self.last_vy, self.last_wz)
            except Exception:
                pass

    def destroy_node(self):
        try:
            self.car.set_car_motion(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
