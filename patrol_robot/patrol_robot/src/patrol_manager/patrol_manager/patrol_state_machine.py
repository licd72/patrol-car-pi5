#!/usr/bin/env python3
"""
巡逻状态机节点

状态流转:
  IDLE → NAVIGATING → SCANNING → (检测到异常?) → TRACKING
                                                    │
                                      ┌─ 确认异常 ──┤
                                      ▼              │
                                   ALERTING ←── 未确认
                                      │
                                      ▼
                                   (冷却) → NAVIGATING

ROS2 接口:
  Subscribers:
    /patrol/detections    (vision_msgs/Detection2DArray) — YOLO 检测结果
  Publishers:
    /patrol/state          (std_msgs/String)             — 当前状态
    /patrol/alert_trigger  (std_msgs/String)             — 报警触发信号
  Action Clients:
    /navigate_to_pose      (nav2_msgs/NavigateToPose)    — Nav2 导航
"""

import time
import math
import threading
from enum import Enum
from pathlib import Path
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import math

def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> list:
    """内置四元数转换，避免依赖 tf_transformations"""
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    ]


# ── 状态枚举 ──
class PatrolState(Enum):
    IDLE = "IDLE"                # 待机
    NAVIGATING = "NAVIGATING"    # 前往预置点
    SCANNING = "SCANNING"         # 到达后 360° 扫描
    TRACKING = "TRACKING"         # 异常目标跟踪确认
    ALERTING = "ALERTING"         # 报警中


class PatrolStateMachine(Node):
    """巡逻状态机主节点"""

    def __init__(self):
        super().__init__("patrol_state_machine")

        # ── 加载参数 ──
        self.declare_parameter("routes_file", "")
        self.declare_parameter("patrol_mode", "sequential")
        self.declare_parameter("loop_count", 0)
        self.declare_parameter("track_duration", 10.0)
        self.declare_parameter("confirm_ratio", 0.6)
        self.declare_parameter("alert_cooldown", 30.0)
        self.declare_parameter("scan_rotation_speed", 0.5)     # rad/s
        self.declare_parameter("scan_duration", 15.0)          # 秒
        self.declare_parameter("waypoint_tolerance", 0.3)      # 到达容差(m)

        routes_file = self.get_parameter("routes_file").get_parameter_value().string_value
        self.patrol_mode = self.get_parameter("patrol_mode").get_parameter_value().string_value
        self.loop_count = self.get_parameter("loop_count").get_parameter_value().integer_value
        self.track_duration = self.get_parameter("track_duration").get_parameter_value().double_value
        self.confirm_ratio = self.get_parameter("confirm_ratio").get_parameter_value().double_value
        self.alert_cooldown = self.get_parameter("alert_cooldown").get_parameter_value().double_value
        self.scan_rotation_speed = self.get_parameter("scan_rotation_speed").get_parameter_value().double_value
        self.scan_duration = self.get_parameter("scan_duration").get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter("waypoint_tolerance").get_parameter_value().double_value

        # ── 加载巡逻路线 ──
        if not routes_file:
            routes_file = str(Path(__file__).parent.parent.parent.parent.parent /
                              "config" / "patrol_routes.yaml")

        self.waypoints = self._load_routes(routes_file)
        if not self.waypoints:
            self.get_logger().error(f"未找到巡逻路线: {routes_file}")
            raise RuntimeError("无巡逻路线")

        self.get_logger().info(f"加载 {len(self.waypoints)} 个预置点")

        # ── 状态变量 ──
        self._lock = threading.Lock()
        self.state = PatrolState.IDLE
        self.current_waypoint_idx = 0
        self.loop_idx = 0

        # 跟踪确认窗口
        self.track_start_time = 0.0
        self.track_total_frames = 0
        self.track_detect_frames = 0

        # 报警冷却
        self.last_alert_time = 0.0

        # Timer 引用 (防止堆积)
        self._track_timer = None

        # ── Nav2 导航客户端 ──
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._nav_goal_handle = None
        self._nav_done = threading.Event()

        # ── cmd_vel 发布器 (用于原地旋转扫描) ──
        from geometry_msgs.msg import Twist
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── 话题 ──
        self.state_pub = self.create_publisher(String, "/patrol/state", 10)
        self.alert_trigger_pub = self.create_publisher(String, "/patrol/alert_trigger", 10)

        # 订阅 YOLO 检测结果
        self.detection_sub = self.create_subscription(
            Detection2DArray, "/patrol/detections", self._on_detection, 10
        )

        # ── 定时器: 主循环 10Hz ──
        self.main_timer = self.create_timer(0.1, self._main_loop)
        # ── 状态重发: 每5秒发布当前状态(确保迟到订阅者收到) ──
        self._state_repub_timer = self.create_timer(5.0, self._republish_state)

    def _republish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

        self.get_logger().info("巡逻状态机启动")

    # ═══════════════════════════════════════════════
    #  主循环
    # ═══════════════════════════════════════════════

    def _main_loop(self):
        """10Hz 主循环, 执行当前状态的动作"""
        with self._lock:
            if self.state == PatrolState.IDLE and not getattr(self, '_nav_unavailable', False):
                self._enter_navigating()

    def _set_state(self, new_state: PatrolState):
        """状态切换 + 日志 + 发布"""
        old = self.state
        self.state = new_state
        self.get_logger().info(f"状态: {old.value} → {new_state.value}")

        msg = String()
        msg.data = new_state.value
        self.state_pub.publish(msg)

    # ═══════════════════════════════════════════════
    #  NAVIGATING — 导航到预置点
    # ═══════════════════════════════════════════════

    def _enter_navigating(self):
        """开始导航到下一个预置点"""
        if not self.waypoints:
            self._set_state(PatrolState.IDLE)
            return

        wp = self.waypoints[self.current_waypoint_idx]
        self._set_state(PatrolState.NAVIGATING)

        # 构造导航目标
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(wp["x"], wp["y"], wp["yaw"])
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"导航 → [{wp['name']}] ({wp['x']:.1f}, {wp['y']:.1f}, yaw={wp['yaw']:.2f})"
        )

        # 等待 action server
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("Nav2 不可用, 降级为纯检测模式 (无导航)")
            self._nav_unavailable = True
            self._set_state(PatrolState.IDLE)
            return

        # 发送导航目标 (异步)
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback
        )
        send_future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        """导航目标已接受"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Nav2 不可用时静默 IDLE (不打断 TRACKING)
            if getattr(self, '_nav_unavailable', False):
                if self.state != PatrolState.TRACKING:
                    self._set_state(PatrolState.IDLE)
                return
            self.get_logger().warn("导航目标被拒绝, 跳到下一个预置点")
            if self.state == PatrolState.NAVIGATING:
                self._advance_waypoint()
            return

        self.get_logger().info("导航目标已接受, 行进中...")
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_feedback(self, feedback_msg):
        """导航中反馈 (当前距离目标)"""
        dist = feedback_msg.feedback.distance_remaining
        if int(dist * 10) % 10 == 0:  # 每米打印一次
            self.get_logger().debug(f"  剩余距离: {dist:.1f}m")

    def _nav_result(self, future):
        """导航到达 (成功或失败)"""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"到达预置点 [{self.waypoints[self.current_waypoint_idx]['name']}]")
            if self.state == PatrolState.NAVIGATING:
                self._change_state_later(PatrolState.SCANNING, 0.0)
        else:
            self.get_logger().warn(f"导航失败 (status={result.status}), 跳到下一个")
            if self.state == PatrolState.NAVIGATING:
                self._advance_waypoint()

    # ═══════════════════════════════════════════════
    #  SCANNING — 到达后 360° 扫描
    # ═══════════════════════════════════════════════

    def _enter_scanning(self):
        """到达预置点, 开始 360° 缓慢旋转扫描"""
        wp = self.waypoints[self.current_waypoint_idx]
        self._set_state(PatrolState.SCANNING)
        self.get_logger().info(f"扫描中 [{wp['name']}] — 旋转 {self.scan_duration}s")

        # 原地旋转 (通过 cmd_vel)
        self._rotate_async(self.scan_rotation_speed, self.scan_duration)

        # 停留时间到后, 如果没有检测到异常 → 下一个点
        dwell = wp.get("dwell", 3)
        total_wait = self.scan_duration + dwell
        self._change_state_later(PatrolState.NAVIGATING, total_wait,
                                 callback=self._advance_waypoint)

    def _rotate_async(self, angular_speed: float, duration: float):
        """异步旋转 (非阻塞) — 通过定时器发送 cmd_vel"""
        from geometry_msgs.msg import Twist

        # 取消旧的旋转 timer (防止堆积)
        if hasattr(self, '_rotate_timers'):
            for t in self._rotate_timers:
                try:
                    t.cancel()
                except Exception:
                    pass
        self._rotate_timers = []

        def _send_rotate():
            twist = Twist()
            twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist)

        def _stop_rotate():
            self.cmd_vel_pub.publish(Twist())  # 零速度

        # 每 100ms 发送旋转指令
        steps = int(duration / 0.1)
        for i in range(steps):
            t = self.create_timer(i * 0.1, _send_rotate)
            self._rotate_timers.append(t)
        # 最后停下
        t = self.create_timer(duration, _stop_rotate)
        self._rotate_timers.append(t)

    # ═══════════════════════════════════════════════
    #  TRACKING — 异常目标跟踪确认
    # ═══════════════════════════════════════════════

    def _on_detection(self, msg: Detection2DArray):
        """接收 YOLO 检测结果"""
        with self._lock:
            now = time.time()

            if self.state == PatrolState.TRACKING:
                # 已经在跟踪中, 累积帧
                self.track_total_frames += 1
                if len(msg.detections) > 0:
                    self.track_detect_frames += 1

            elif self.state in (PatrolState.IDLE, PatrolState.SCANNING, PatrolState.NAVIGATING):
                # 处于待机/巡逻中, 检测到异常 → 开始跟踪
                if len(msg.detections) > 0:
                    self._start_tracking()

    def _start_tracking(self):
        """开始跟踪确认窗口 (跟踪中不重复启动)"""
        now = time.time()
        # 已在跟踪中: 不重启 (防止 timer 被无限取消)
        if self.state == PatrolState.TRACKING:
            return
        # 冷却检查: 距上次跟踪结束不到 5 秒则跳过
        if hasattr(self, '_last_track_end') and now - self._last_track_end < 5.0:
            return
        self._set_state(PatrolState.TRACKING)
        self.track_start_time = now
        self.track_total_frames = 0
        self.track_detect_frames = 0
        self.get_logger().info(f"检测到异常目标, 开始跟踪 ({self.track_duration}s)")

        # 创建一次性 timer (oneshot), 10秒后评估
        if self._track_timer is not None:
            self._track_timer.cancel()
        self._track_timer = self.create_timer(
            self.track_duration, self._evaluate_tracking
        )

    def _evaluate_tracking(self):
        """跟踪窗口结束, 评估是否触发报警"""
        with self._lock:
            if self.state != PatrolState.TRACKING:
                return

            total = max(self.track_total_frames, 1)
            ratio = self.track_detect_frames / total

            self.get_logger().info(
                f"跟踪评估: {self.track_detect_frames}/{total} 帧 = {ratio:.1%} "
                f"(阈值 {self.confirm_ratio:.0%})"
            )

            if ratio >= self.confirm_ratio:
                self._trigger_alert()
            else:
                self.get_logger().info("未确认异常, 继续巡逻")
                self._last_track_end = time.time()  # 记录跟踪结束时间, 用于冷却
                self._advance_waypoint()

    # ═══════════════════════════════════════════════
    #  ALERTING — 触发报警
    # ═══════════════════════════════════════════════

    def _trigger_alert(self):
        """触发报警 (冷却检查)"""
        now = time.time()

        # 冷却检查
        if now - self.last_alert_time < self.alert_cooldown:
            self.get_logger().warn(
                f"报警冷却中 ({(self.alert_cooldown - (now - self.last_alert_time)):.0f}s 剩余)"
            )
            self._advance_waypoint()
            return

        self._set_state(PatrolState.ALERTING)
        self.last_alert_time = now

        # 发布报警触发信号 → alert_dispatcher 处理
        alert_msg = String()
        wp = self.waypoints[self.current_waypoint_idx] if self.waypoints else {"name": "unknown"}
        alert_msg.data = (
            f"location={wp.get('name', 'unknown')}|"
            f"detections={self.track_detect_frames}|"
            f"confidence={self.track_detect_frames / max(self.track_total_frames, 1):.1%}|"
            f"time={now}"
        )
        self.alert_trigger_pub.publish(alert_msg)
        self.get_logger().error(f"🚨 报警触发! {alert_msg.data}")

        # 报警结束后恢复巡逻
        self._change_state_later(PatrolState.NAVIGATING, self.alert_cooldown,
                                 callback=self._advance_waypoint)

    # ═══════════════════════════════════════════════
    #  辅助方法
    # ═══════════════════════════════════════════════

    def _advance_waypoint(self):
        """前进到下一个预置点"""
        self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
        if self.current_waypoint_idx == 0:
            self.loop_idx += 1
            self.get_logger().info(f"巡逻周期 #{self.loop_idx} 完成")
        self._set_state(PatrolState.NAVIGATING)

    def _change_state_later(self, state: PatrolState, delay: float, callback=None):
        """延迟切换状态 (非阻塞, 取消旧 timer 防堆积, TRACKING 中不打断)"""
        def _switch():
            # 跟踪中不切换状态 (防止打断报警确认)
            if self.state == PatrolState.TRACKING:
                return
            self._set_state(state)
            if state == PatrolState.SCANNING:
                self._enter_scanning()
            if callback:
                callback()
        # 取消旧延迟切换 timer
        if hasattr(self, '_delay_timer') and self._delay_timer is not None:
            self._delay_timer.cancel()
        self._delay_timer = self.create_timer(delay, _switch)

    @staticmethod
    def _make_pose(x: float, y: float, yaw: float) -> PoseStamped:
        """构造 PoseStamped"""
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        q = _quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def _load_routes(self, routes_file: str) -> list:
        """从 YAML 加载巡逻路线"""
        path = Path(routes_file)
        if not path.exists():
            self.get_logger().error(f"路线文件不存在: {routes_file}")
            return []

        with open(path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        routes = config.get("routes", {})
        # 使用第一条 route (可扩展为多路线切换)
        if routes:
            route_name = list(routes.keys())[0]
            waypoints = routes[route_name].get("waypoints", [])
            self.get_logger().info(f"加载路线: {route_name} ({len(waypoints)} 点)")
            return waypoints

        return []


def main(args=None):
    rclpy.init(args=args)
    node = PatrolStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
