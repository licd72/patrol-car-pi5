#!/usr/bin/env python3
"""
巡逻车语音交互节点

功能:
  1. 状态播报 — 状态变化时自动语音播报
  2. 报警语音 — 检测到异常时播报警告
  3. 语音指令 — 接收语音命令并执行底盘控制

ROS2 接口:
  Subscribers: /patrol/state, /patrol/alert_trigger, /patrol/alert_status
  Publishers:  /patrol/voice_cmd, /cmd_vel
"""

import time
import threading
from pathlib import Path
import sys

SPEECH_LIB_PATH = str(Path(__file__).parent.parent.parent.parent.parent / "src")
if SPEECH_LIB_PATH not in sys.path:
    sys.path.insert(0, SPEECH_LIB_PATH)

try:
    from Speech_Lib import Speech
    HAS_SPEECH = True
except ImportError:
    HAS_SPEECH = False

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests


VOICE_MAP = {
    "boot": 1, "idle": 2, "patrol_start": 3,
    "navigating": 4, "scanning": 5, "tracking": 6,
    "alert": 7, "alert_end": 8, "report": 20, "cmd_ok": 21,
}

CMD_MAP = {
    # ── 运动控制 ──
    1: "stop", 2: "stop",
    4: "forward", 5: "backward",
    6: "turn_left", 7: "turn_right",
    8: "spin_left", 9: "spin_right",
    # ── 灯光 ──
    10: "lights_off", 11: "red_light", 12: "green_light",
    13: "blue_light", 14: "yellow_light",
    15: "flow_light", 16: "gradient_light", 17: "breath_light",
    # ── 导航 ──
    18: "show_battery", 
    19: "goto_1", 20: "goto_2", 21: "goto_3", 32: "goto_4",
    33: "go_home",
    # ── 巡线 ──
    22: "line_off", 23: "red_line", 24: "green_line",
    25: "blue_line", 26: "yellow_line",
    # ── 跟随 ──
    27: "follow_on", 28: "follow_off",
    # ── 机械臂/舵机 ──
    41: "arm_left", 42: "arm_right",
    43: "grip_close", 44: "grip_open",
    # ── 其他 ──
    38: "alert_on",
}


class PatrolVoice(Node):
    """巡逻车语音交互 ROS2 节点"""

    def __init__(self):
        super().__init__("patrol_voice")

        self.declare_parameter("serial_port", "/dev/ttyAMA0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("voice_enabled", True)
        self.declare_parameter("command_enabled", True)
        self.declare_parameter("voice_interval", 5.0)
        self.declare_parameter("command_poll_rate", 10.0)

        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.voice_enabled = self.get_parameter("voice_enabled").get_parameter_value().bool_value
        self.command_enabled = self.get_parameter("command_enabled").get_parameter_value().bool_value
        self.voice_interval = self.get_parameter("voice_interval").get_parameter_value().double_value
        poll_rate = self.get_parameter("command_poll_rate").get_parameter_value().double_value

        self._last_state = "OFFLINE"
        self._last_voice_time = 0.0
        self._lock = threading.Lock()
        self._move_timer = None

        # 初始化语音模块
        self.speech = None
        self.speech_cmd = None
        if HAS_SPEECH and self.voice_enabled:
            try:
                self.speech = Speech(com=serial_port)
                self.get_logger().info(f"语音模块就绪: {serial_port}")
            except Exception as e:
                self.get_logger().warn(f"语音模块连接失败: {e}")

        if self.command_enabled:
            self.speech_cmd = self.speech  # 共用同一个串口
            self.get_logger().info("命令监听: 共用语音串口")

        if self.speech:
            self._speak("boot")

        # 订阅
        self.state_sub = self.create_subscription(String, "/patrol/state", self._on_state_change, 10)
        self.alert_sub = self.create_subscription(String, "/patrol/alert_trigger", self._on_alert, 10)
        self.status_sub = self.create_subscription(String, "/patrol/alert_status", self._on_alert_status, 10)

        # 发布
        self.cmd_pub = self.create_publisher(String, "/patrol/voice_cmd", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 命令轮询
        if self.command_enabled:
            self.cmd_timer = self.create_timer(1.0 / poll_rate, self._poll_command)

        self.get_logger().info(f"语音节点启动 | 播报={'Y' if self.speech else 'N'} | 命令={'Y' if self.command_enabled else 'N'}")

    # ═══════════════════════════════════════════════
    #  状态播报
    # ═══════════════════════════════════════════════

    def _on_state_change(self, msg: String):
        new_state = msg.data
        if new_state == self._last_state:
            return
        self._last_state = new_state
        voice_id = {"IDLE": "idle", "NAVIGATING": "navigating", "SCANNING": "scanning",
                    "TRACKING": "tracking", "ALERTING": "alert"}.get(new_state)
        if voice_id:
            self._speak(voice_id)

    def _on_alert(self, msg: String):
        self._speak("alert")

    def _on_alert_status(self, msg: String):
        if "解除" in msg.data or "恢复" in msg.data:
            self._speak("alert_end")

    # ═══════════════════════════════════════════════
    #  语音指令接收
    # ═══════════════════════════════════════════════

    def _poll_command(self):
        """轮询语音模块"""
        if not self.speech_cmd:
            return
        try:
            cmd_id = self.speech_cmd.speech_read()
        except Exception as e:
            return

        if cmd_id == 999 or cmd_id not in CMD_MAP:
            return

        cmd_name = CMD_MAP[cmd_id]
        self.get_logger().info(f"🎤 语音命令: {cmd_name} (id={cmd_id})")

        # 发布到 /patrol/voice_cmd (日志用)
        cmd_msg = String()
        cmd_msg.data = cmd_name
        self.cmd_pub.publish(cmd_msg)

        # 执行底盘运动
        self._execute_move(cmd_name)

        # 确认回复
        self._speak("cmd_ok")

    # ═══════════════════════════════════════════════
    #  底盘控制
    # ═══════════════════════════════════════════════

    def _execute_move(self, cmd_name: str):
        """将语音命令转换为底盘运动"""
        twist = Twist()
        duration = 2.0

        # ── 平移 ──
        if cmd_name == "forward":
            twist.linear.x = 0.2
        elif cmd_name == "backward":
            twist.linear.x = -0.2
        # ── 转向 ──
        elif cmd_name == "turn_left":
            twist.angular.z = 0.8
        elif cmd_name == "turn_right":
            twist.angular.z = -0.8
        # ── 原地旋转 ──
        elif cmd_name == "spin_left":
            twist.angular.z = 1.2
        elif cmd_name == "spin_right":
            twist.angular.z = -1.2
        # ── 停止 ──
        elif cmd_name == "stop":
            duration = 0.1
        # ── 灯光 (通过 Rosmaster 协议的 /RGBLight 话题) ──
        elif cmd_name == "lights_off":
            self.cmd_vel_pub.publish(Twist())  # 先停
            self._publish_rgb(0)
            return
        elif cmd_name == "red_light":
            self._publish_rgb(1)
            return
        elif cmd_name == "green_light":
            self._publish_rgb(2)
            return
        elif cmd_name == "blue_light":
            self._publish_rgb(3)
            return
        elif cmd_name == "yellow_light":
            self._publish_rgb(4)
            return
        elif cmd_name == "flow_light":
            self._publish_rgb(5)
            return
        elif cmd_name == "gradient_light":
            self._publish_rgb(6)
            return
        elif cmd_name == "breath_light":
            self._publish_rgb(7)
            return
        # ── 巡线/跟随/导航/报警 ──
        elif cmd_name in ("goto_1", "goto_2", "goto_3", "goto_4", "go_home",
                          "line_off", "red_line", "green_line", "blue_line", "yellow_line",
                          "follow_on", "follow_off", "show_battery", "alert_on",
                          "arm_left", "arm_right", "grip_close", "grip_open"):
            self.get_logger().info(f"🎤 命令已记录: {cmd_name} (待状态机支持)")
            return
        else:
            return

        self.get_logger().info(f"🎤→🚗 {cmd_name} (duration={duration}s @ 10Hz)")
        # 取消上一次持续发布
        if self._move_timer:
            try: self._move_timer.cancel()
            except: pass
        # 10Hz 持续发布 duration 秒 (对抗 STM32 底盘 100ms watchdog)
        # cmd_vel_bridge 也有 heartbeat, 这里保险起见客户端也维持
        def _keep_publish():
            import time as _t
            end = _t.time() + duration
            while _t.time() < end:
                self.cmd_vel_pub.publish(twist)
                _t.sleep(0.1)
            self.cmd_vel_pub.publish(Twist())   # 结束后发一次零速
        t = threading.Thread(target=_keep_publish, daemon=True)
        self._move_timer = t
        t.start()

    def _publish_rgb(self, effect: int):
        """发布 RGB 灯光指令"""
        from std_msgs.msg import Int32
        msg = Int32()
        msg.data = effect
        # 确保 RGB 发布器存在
        if not hasattr(self, '_rgb_pub'):
            self._rgb_pub = self.create_publisher(Int32, "/RGBLight", 10)
        self._rgb_pub.publish(msg)
        self.get_logger().info(f"💡 灯光: {effect}")

    def _auto_stop(self):
        """自动停止"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


    # ═══════════════════════════════════════════════
    #  播报方法
    # ═══════════════════════════════════════════════

    def _speak(self, voice_name: str):
        """播报指定语音"""
        if not self.speech or not self.voice_enabled:
            return

        voice_id = VOICE_MAP.get(voice_name)
        if voice_id is None:
            return

        now = time.time()
        if now - self._last_voice_time < self.voice_interval:
            return
        self._last_voice_time = now

        def _do_speak():
            try:
                self.speech.speech_write(voice_id)
            except Exception as e:
                self.get_logger().debug(f"播报失败: {e}")

        t = threading.Thread(target=_do_speak, daemon=True)
        t.start()


def main():
    rclpy.init()
    node = PatrolVoice()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
