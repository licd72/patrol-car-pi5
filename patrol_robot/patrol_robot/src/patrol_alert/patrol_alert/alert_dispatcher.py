#!/usr/bin/env python3
"""
报警联动调度节点

接收巡逻状态机的报警触发信号, 执行多通道报警:
  1. 本地声光 (GPIO 蜂鸣器 + LED)
  2. 钉钉机器人推送 (Webhook)
  3. 飞书机器人推送 (Webhook, 可选)
  4. 抓拍图像存证 (JPEG → SSD)

ROS2 接口:
  Subscribers:
    /patrol/alert_trigger  (std_msgs/String)  — 报警触发信号
    /patrol/state          (std_msgs/String)  — 巡逻状态
    /patrol/alert_image    (sensor_msgs/Image) — YOLO 抓拍图
"""

import time
import json
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import requests
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ── GPIO 导入 (仅在 RPi 上) ──
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class AlertDispatcher(Node):
    """多通道报警联动调度节点"""

    def __init__(self):
        super().__init__("alert_dispatcher")

        # ── 加载配置 ──
        self.declare_parameter("config_file", "")
        self.declare_parameter("snapshot_dir", "/home/pi/patrol_robot/snapshots/")
        self.declare_parameter("max_snapshots_per_alert", 20)

        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_file:
            config_file = str(
                Path(__file__).parent.parent.parent.parent.parent
                / "config" / "alert_rules.yaml"
            )

        self.config = self._load_config(config_file)
        self.snapshot_dir = Path(
            self.get_parameter("snapshot_dir").get_parameter_value().string_value
        )
        self.max_snapshots = self.get_parameter(
            "max_snapshots_per_alert"
        ).get_parameter_value().integer_value

        self.snapshot_dir.mkdir(parents=True, exist_ok=True)

        # ── 状态 ──
        self._lock = threading.Lock()
        self._alert_active = False
        self._alert_start_time = 0.0
        self._snapshot_count = 0
        self._last_snapshot = None

        # ── 环形缓冲区: 缓存最近 N 帧，报警触发时回写 ──
        self._image_buffer = deque(maxlen=60)  # 60帧 ≈ 30秒

        # ── CV Bridge ──
        self.bridge = CvBridge()

        # ── GPIO 初始化 ──
        self._gpio_ready = False
        if HAS_GPIO and self.config.get("channels", {}).get("local", {}).get("enabled"):
            self._init_gpio()

        # ── 订阅 ──
        self.alert_sub = self.create_subscription(
            String, "/patrol/alert_trigger", self._on_alert_trigger, 10
        )
        self.state_sub = self.create_subscription(
            String, "/patrol/state", self._on_state_change, 10
        )
        self.image_sub = self.create_subscription(
            Image, "/patrol/alert_image", self._on_alert_image, 10
        )

        # ── 发布 ──
        self.status_pub = self.create_publisher(String, "/patrol/alert_status", 10)

        self.get_logger().info("报警调度器启动")

    # ═══════════════════════════════════════════════
    #  报警触发
    # ═══════════════════════════════════════════════

    def _on_alert_trigger(self, msg: String):
        """接收报警触发信号, 执行全部报警通道"""
        data = msg.data
        self.get_logger().error(f"🚨 收到报警触发: {data}")

        with self._lock:
            self._alert_active = True
            self._alert_start_time = time.time()
            self._snapshot_count = 0

        # ── 回写环形缓冲区: 保存检测到人那一刻的画面 ──
        self._dump_buffer()

        # 解析触发数据
        info = {}
        for part in data.split("|"):
            if "=" in part:
                k, v = part.split("=", 1)
                info[k] = v

        location = info.get("location", "未知位置")
        detections = info.get("detections", "?")
        confidence = info.get("confidence", "?")

        # ── 并行执行所有报警通道 ──
        threads = []

        # 通道 1: 本地声光
        t1 = threading.Thread(target=self._channel_local, args=(location,))
        threads.append(t1)

        # 通道 2: 钉钉推送
        if self.config.get("channels", {}).get("dingtalk", {}).get("enabled"):
            t2 = threading.Thread(target=self._channel_dingtalk,
                                  args=(location, detections, confidence))
            threads.append(t2)

        # 通道 3: 飞书推送
        if self.config.get("channels", {}).get("feishu", {}).get("enabled"):
            t3 = threading.Thread(target=self._channel_feishu,
                                  args=(location, detections, confidence))
            threads.append(t3)

        for t in threads:
            t.start()

    # ═══════════════════════════════════════════════
    #  通道 1: 本地声光报警 (GPIO)
    # ═══════════════════════════════════════════════

    def _init_gpio(self):
        """初始化 GPIO 引脚"""
        local_cfg = self.config.get("channels", {}).get("local", {})
        buzzer_pin = local_cfg.get("buzzer_pin", 18)
        led_pin = local_cfg.get("led_pin", 23)

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(buzzer_pin, GPIO.OUT)
            GPIO.setup(led_pin, GPIO.OUT)
            GPIO.output(buzzer_pin, GPIO.LOW)
            GPIO.output(led_pin, GPIO.LOW)
            self._gpio_ready = True
            self.get_logger().info(
                f"GPIO 就绪: 蜂鸣器 GPIO{buzzer_pin}, LED GPIO{led_pin}"
            )
        except Exception as e:
            self.get_logger().warn(f"GPIO 初始化失败: {e}")

    def _channel_local(self, location: str):
        """执行本地声光报警"""
        if not self._gpio_ready:
            self.get_logger().warn("GPIO 未就绪, 跳过本地报警")
            return

        local_cfg = self.config.get("channels", {}).get("local", {})
        buzzer_pin = local_cfg.get("buzzer_pin", 18)
        led_pin = local_cfg.get("led_pin", 23)
        pattern = local_cfg.get("pattern", "flash")

        self.get_logger().info(f"🔊 本地报警启动 (模式: {pattern})")

        try:
            duration = self.config.get("alert", {}).get("cooldown", 30.0)
            end_time = time.time() + duration

            while time.time() < end_time and self._alert_active:
                if pattern == "flash":
                    GPIO.output(led_pin, GPIO.HIGH)
                    GPIO.output(buzzer_pin, GPIO.HIGH)
                    time.sleep(0.5)
                    GPIO.output(led_pin, GPIO.LOW)
                    GPIO.output(buzzer_pin, GPIO.LOW)
                    time.sleep(0.5)
                elif pattern == "siren":
                    GPIO.output(led_pin, GPIO.HIGH)
                    GPIO.output(buzzer_pin, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(buzzer_pin, GPIO.LOW)
                    time.sleep(0.1)
                else:  # steady
                    GPIO.output(led_pin, GPIO.HIGH)
                    GPIO.output(buzzer_pin, GPIO.HIGH)
                    time.sleep(1.0)

            # 关闭
            GPIO.output(buzzer_pin, GPIO.LOW)
            GPIO.output(led_pin, GPIO.LOW)
        except Exception as e:
            self.get_logger().error(f"本地报警异常: {e}")

    # ═══════════════════════════════════════════════
    #  通道 2: 钉钉机器人推送
    # ═══════════════════════════════════════════════

    def _channel_dingtalk(self, location: str, detections: str, confidence: str):
        """钉钉 Webhook 报警推送"""
        cfg = self.config.get("channels", {}).get("dingtalk", {})
        webhook_url = cfg.get("webhook_url", "")

        if "YOUR_TOKEN" in webhook_url:
            self.get_logger().warn("钉钉 Webhook URL 未配置 (仍为 YOUR_TOKEN)")
            return

        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # 构造 ActionCard 消息
        payload = {
            "msgtype": "actionCard",
            "actionCard": {
                "title": f"🚨 巡逻报警 — 人员闯入",
                "text": (
                    f"## 巡逻报警\n\n"
                    f"**位置**: {location}\n\n"
                    f"**检测目标**: {detections} 帧命中\n\n"
                    f"**置信度**: {confidence}\n\n"
                    f"**时间**: {now}\n\n"
                    f"---\n"
                    f"[查看实时画面](http://{self._get_ip()}:5000)"
                ),
                "btnOrientation": "0",
                "singleTitle": "查看详情",
                "singleURL": f"http://{self._get_ip()}:5000",
            },
        }

        try:
            resp = requests.post(
                webhook_url, json=payload,
                headers={"Content-Type": "application/json"},
                timeout=5,
            )
            result = resp.json()
            if result.get("errcode") == 0:
                self.get_logger().info("✅ 钉钉推送成功")
            else:
                self.get_logger().warn(f"钉钉推送返回: {result}")
        except Exception as e:
            self.get_logger().error(f"钉钉推送失败: {e}")

    # ═══════════════════════════════════════════════
    #  通道 3: 飞书机器人推送 (备用)
    # ═══════════════════════════════════════════════

    def _channel_feishu(self, location: str, detections: str, confidence: str):
        """飞书 Webhook 报警推送"""
        cfg = self.config.get("channels", {}).get("feishu", {})
        webhook_url = cfg.get("webhook_url", "")

        if not webhook_url:
            self.get_logger().warn("飞书 Webhook URL 未配置")
            return

        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        payload = {
            "msg_type": "interactive",
            "card": {
                "header": {
                    "title": {"tag": "plain_text", "content": "🚨 巡逻报警"},
                    "template": "red",
                },
                "elements": [
                    {"tag": "div", "text": {"tag": "lark_md",
                        "content": f"**位置**: {location}\n**置信度**: {confidence}\n**时间**: {now}"}},
                ],
            },
        }

        try:
            resp = requests.post(webhook_url, json=payload, timeout=5)
            self.get_logger().info(f"飞书推送: {resp.status_code}")
        except Exception as e:
            self.get_logger().error(f"飞书推送失败: {e}")

    # ═══════════════════════════════════════════════
    #  抓拍存证
    # ═══════════════════════════════════════════════

    def _on_alert_image(self, msg: Image):
        """接收 YOLO 抓拍图, 始终缓存到环形缓冲区 (报警触发时回写)"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            timestamp = time.time()
            self._image_buffer.append((timestamp, cv_img))
        except Exception as e:
            self.get_logger().error(f"抓拍缓存失败: {e}")

    # ═══════════════════════════════════════════════
    #  环形缓冲区回写
    # ═══════════════════════════════════════════════

    def _dump_buffer(self):
        """将环形缓冲区中的帧写入磁盘 (报警确认时调用)"""
        buf_len = len(self._image_buffer)
        if buf_len == 0:
            self.get_logger().warn("抓拍缓冲区为空")
            return

        # 从缓冲区中均匀采样 max_snapshots 张
        step = max(1, buf_len // self.max_snapshots)
        count = 0
        for i in range(0, buf_len, step):
            if count >= self.max_snapshots:
                break
            ts, cv_img = self._image_buffer[i]
            timestamp = datetime.fromtimestamp(ts, tz=CST).strftime("%Y%m%d_%H%M%S")
            filename = self.snapshot_dir / f"alert_{timestamp}_{count:02d}.jpg"
            cv2.imwrite(str(filename), cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            count += 1
            self._last_snapshot = str(filename)

        self._snapshot_count = count
        self.get_logger().info(
            f"📸 抓拍完成: {count}/{buf_len} 帧写入磁盘 (缓冲区 {buf_len} 帧)"
        )

        # 清空缓冲区, 避免下次触发重复保存
        self._image_buffer.clear()

    # ═══════════════════════════════════════════════
    #  状态跟踪
    # ═══════════════════════════════════════════════

    def _on_state_change(self, msg: String):
        """跟踪巡逻状态变化"""
        if msg.data == "IDLE" or msg.data == "NAVIGATING":
            with self._lock:
                if self._alert_active:
                    self.get_logger().info("巡逻恢复, 报警结束")
                self._alert_active = False
                self._image_buffer.clear()  # 清空未触发的缓冲

        # 定期清理过期快照
        if msg.data == "IDLE":
            self._cleanup_old_snapshots()

    def _cleanup_old_snapshots(self):
        """清理过期抓拍 (retention_days 天前)"""
        retention = self.config.get("snapshot", {}).get("retention_days", 30)
        cutoff = time.time() - retention * 86400

        try:
            for f in self.snapshot_dir.glob("alert_*.jpg"):
                if f.stat().st_mtime < cutoff:
                    f.unlink()
                    self.get_logger().debug(f"清理过期: {f.name}")
        except Exception:
            pass  # 静默处理

    # ═══════════════════════════════════════════════
    #  辅助
    # ═══════════════════════════════════════════════

    @staticmethod
    def _get_ip() -> str:
        """获取本机 IP (用于报警消息中的链接)"""
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "192.168.1.100"

    def _load_config(self, config_file: str) -> dict:
        """加载 YAML 配置"""
        path = Path(config_file)
        if not path.exists():
            self.get_logger().warn(f"配置文件不存在: {config_file}, 使用默认值")
            return {
                "alert": {"cooldown": 30.0},
                "channels": {
                    "local": {"enabled": True, "buzzer_pin": 18, "led_pin": 23},
                    "dingtalk": {"enabled": False},
                    "feishu": {"enabled": False},
                },
                "snapshot": {"retention_days": 30},
            }

        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def destroy_node(self):
        """清理 GPIO"""
        if self._gpio_ready:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AlertDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
