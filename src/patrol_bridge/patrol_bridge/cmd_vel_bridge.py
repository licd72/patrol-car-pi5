#!/usr/bin/env python3
"""
cmd_vel_bridge (产品版 v2.0)

v2 增加:
  - 编码器读取 + 陀螺仪 yaw 积分
  - 发布 /odom (nav_msgs/Odometry)  
  - 发布 tf odom → base_footprint (动态)
  - 关键: 与写共用同一 Rosmaster 实例 (同进程读写不冲突)

设计目的:
  1. 系统内唯一 /cmd_vel 订阅者 → 唯一电机 owner
  2. 10Hz heartbeat 持续重发最后一条 Twist → 对抗 STM32 ~100ms watchdog
  3. 0.5s 未收到新命令 → 自动停车
  4. 读+写共同进程 (create_receive_threading 只在 bridge 里)
  5. 20Hz 从编码器计算 odom + IMU yaw → /odom + tf
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

TICK_PER_M = 1.0 / 1.843e-4  # 标定: 891 ticks / 0.225m = 3961 tick/m
# 校准值 (2026-07-17): 编码器 0.15 m/s × 1.5 s -> 平均 delta ≈ 891 ticks
M_PER_TICK = 1.843e-4


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")

        # 参数
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("heartbeat_hz", 10.0)
        self.declare_parameter("odom_hz", 20.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.serial_port = self.get_parameter("serial_port").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.heartbeat_hz = self.get_parameter("heartbeat_hz").value
        self.odom_hz = self.get_parameter("odom_hz").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # 打开 Rosmaster
        from Rosmaster_Lib import Rosmaster
        self.bot = Rosmaster(car_type=5, com=self.serial_port)
        self.bot.create_receive_threading()
        self.bot.set_auto_report_state(enable=True, forever=False)
        time.sleep(1.0)
        self.get_logger().info(f"Rosmaster 就绪 (port={self.serial_port})")

        # 订阅
        self.sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # 发布
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_br = TransformBroadcaster(self)

        # 状态
        self._last_cmd = (0.0, 0.0, 0.0)     # vx, vy, wz
        self._last_cmd_time = time.time()
        self._stat_recv = 0
        self._stat_hb = 0

        # Odom 累积
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._last_encoders = None
        self._last_odom_time = time.time()
        self._th_offset = None  # yaw 初始偏移
        self._n_odom = 0

        # 定时器
        self.create_timer(1.0 / self.heartbeat_hz, self._heartbeat)
        self.create_timer(1.0 / self.odom_hz, self._odom_tick)
        self.create_timer(1.0, self._stat_log)

    # ---------- cmd_vel ----------
    def _on_cmd_vel(self, msg: Twist):
        self._last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd_time = time.time()
        self._stat_recv += 1

    def _heartbeat(self):
        vx, vy, wz = self._last_cmd
        stopped = (time.time() - self._last_cmd_time) > self.cmd_timeout
        if stopped:
            vx = vy = wz = 0.0
            self._last_cmd = (0.0, 0.0, 0.0)
        try:
            t0 = time.time()
            self.bot.set_car_motion(vx, vy, wz)
            dt = time.time() - t0
            if dt > 0.05:
                self.get_logger().warn(f"set_car_motion 耗时 {dt*1000:.1f}ms!", throttle_duration_sec=1)
        except Exception as e:
            self.get_logger().error(f"set_car_motion err: {e}", throttle_duration_sec=2)
        self._stat_hb += 1

    def _stat_log(self):
        vx, vy, wz = self._last_cmd
        stopped = (time.time() - self._last_cmd_time) > self.cmd_timeout
        try:
            enc_now = self.bot.get_motor_encoder()
        except Exception:
            enc_now = None
        self.get_logger().info(
            f"stat: recv={self._stat_recv} hb={self._stat_hb} vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} stopped={stopped} odom={self._n_odom} x={self._x:.2f} y={self._y:.2f} th={math.degrees(self._th):.0f}deg enc={enc_now}"
        )
        self._stat_recv = 0
        self._stat_hb = 0
        self._n_odom = 0

    # ---------- odom ----------
    def _odom_tick(self):
        try:
            enc = self.bot.get_motor_encoder()  # (m1, m2, m3, m4)
            imu = self.bot.get_imu_attitude_data()  # (roll, pitch, yaw) 度
        except Exception as e:
            self.get_logger().error(f"read err: {e}", throttle_duration_sec=2)
            return

        if enc is None or len(enc) < 4:
            return

        now = time.time()
        dt = now - self._last_odom_time
        if dt <= 0:
            return
        self._last_odom_time = now

        # 首次: 记初值
        if self._last_encoders is None:
            self._last_encoders = enc
            if imu:
                self._th_offset = math.radians(imu[2])
            return

        # 编码器差 (mecanum: 4轮平均前后运动)
        # M1 前左, M2 后左, M3 前右, M4 后右
        # 前进: 4 轮同符号增
        # 侧移: 对角同符号
        # 简化: 只用前后 (对 SLAM 建图够)
        d_ticks = [enc[i] - self._last_encoders[i] for i in range(4)]
        self._last_encoders = enc
        # 平均 4 轮 = 前进距离
        d_forward = sum(d_ticks) / 4.0 * M_PER_TICK

        # yaw 用 IMU (更稳)
        if imu and self._th_offset is not None:
            self._th = math.radians(imu[2]) - self._th_offset
            # normalize
            while self._th > math.pi: self._th -= 2 * math.pi
            while self._th < -math.pi: self._th += 2 * math.pi

        # 位置积分 (只考虑前进方向)
        self._x += d_forward * math.cos(self._th)
        self._y += d_forward * math.sin(self._th)

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        # yaw -> quaternion
        cy = math.cos(self._th * 0.5)
        sy = math.sin(self._th * 0.5)
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        odom.twist.twist.linear.x = d_forward / dt if dt > 0 else 0.0
        vx_cmd, vy_cmd, wz_cmd = self._last_cmd
        odom.twist.twist.angular.z = wz_cmd  # 用 cmd (真实需要陀螺仪差分)
        self.odom_pub.publish(odom)

        # 发布 tf
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = sy
            t.transform.rotation.w = cy
            self.tf_br.sendTransform(t)

        self._n_odom += 1

    def destroy_node(self):
        try:
            self.bot.set_car_motion(0, 0, 0)
            self.bot.set_auto_report_state(enable=False, forever=False)
        except Exception:
            pass
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
