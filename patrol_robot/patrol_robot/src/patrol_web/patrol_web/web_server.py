#!/usr/bin/env python3
"""
巡逻小车 Web 监控面板

Flask 后端 + ROS2 数据源, 浏览器访问 http://<rpi_ip>:5000

功能:
  - 实时巡逻状态 (IDLE/NAVIGATING/SCANNING/TRACKING/ALERTING)
  - 最近检测记录 + 报警历史
  - 抓拍图片查看 (最近 50 张)
  - 预置点列表与当前导航目标
  - 健康检查 API

ROS2 接口 (内部订阅):
  /patrol/state          → 巡逻状态
  /patrol/detections     → YOLO 检测结果
  /patrol/alert_status   → 报警状态

HTTP 接口:
  GET  /                     — 仪表盘 HTML
  GET  /api/state            — JSON: 当前状态
  GET  /api/detections       — JSON: 最近检测
  GET  /api/alerts           — JSON: 报警历史
  GET  /api/snapshots        — JSON: 抓拍文件列表
  GET  /snapshots/<filename> — 抓拍图片文件
"""

import json
import time
import threading
from datetime import datetime, timezone, timedelta
CST = timezone(timedelta(hours=8))
from pathlib import Path
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

from flask import Flask, render_template, jsonify, send_from_directory, request, Response


# ── Flask 应用 ──
app = Flask(__name__)

# 全局数据存储 (ROS2 线程写入, Flask 线程读取)
_store = {
    "state": "OFFLINE",
    "state_since": "",
    "waypoint": {"name": "—", "x": 0, "y": 0, "completed": 0, "total": 0},
    "detections": deque(maxlen=20),
    "alerts": deque(maxlen=50),
    "voice_cmds": deque(maxlen=20),
    "snapshot_dir": "",
    "start_time": datetime.now(CST).isoformat(),
}


# ═══════════════════════════════════════════════
#  Flask 路由
# ═══════════════════════════════════════════════

@app.route("/")
def dashboard():
    """仪表盘主页"""
    return render_template("dashboard.html")


@app.route("/api/state")
def api_state():
    """当前巡逻状态"""
    det_count = len([d for d in _store["detections"] if d])
    return jsonify({
        "state": _store["state"],
        "state_since": _store["state_since"],
        "waypoint": _store["waypoint"],
        "total_detections": det_count,
        "total_alerts": len(_store["alerts"]),
        "uptime_seconds": round(
            time.time() - datetime.fromisoformat(_store["start_time"]).timestamp()
        ),
    })


@app.route("/api/detections")
def api_detections():
    """最近检测结果"""
    return jsonify(list(_store["detections"]))


@app.route("/api/alerts")
def api_alerts():
    """报警历史"""
    return jsonify(list(_store["alerts"]))


@app.route("/api/voice_cmds")
def api_voice_cmds():
    """语音命令历史"""
    return jsonify(list(_store["voice_cmds"]))


# ── 小车控制 API ──
import threading

@app.route("/api/control/move", methods=["POST"])
def api_control_move():
    """移动控制: {direction: forward|backward|left|right|strafe_left|strafe_right|stop, duration: 0.5, speed: 0.15}"""
    data = request.get_json() or {}
    direction = data.get("direction", "stop")
    # 旋转需要更长时间才能看到效果
    rot_dirs = ["left", "right"]
    default_dur = 1.5 if direction in rot_dirs else 0.4
    duration = min(float(data.get("duration", default_dur)), 3.0)
    speed = min(float(data.get("speed", _store.get("_speed", 0.15))), 0.5)

    _store["_speed"] = speed  # 持久化速度

    pub = _store.get("cmd_vel_pub")
    if pub is None:
        return jsonify({"error": "publisher not ready"}), 503

    twist = Twist()
    if direction in ["left", "right"]:
        _speed = min(speed * 5.0, 2.5)
    else:
        _speed = speed
    if direction == "forward":
        twist.linear.x = _speed
    elif direction == "backward":
        twist.linear.x = -_speed
    elif direction == "left":
        twist.angular.z = _speed
    elif direction == "right":
        twist.angular.z = -_speed
    elif direction == "strafe_left":
        twist.linear.y = _speed
    elif direction == "strafe_right":
        twist.linear.y = -_speed

    pub.publish(twist)

    def _stop():
        import time
        time.sleep(duration)
        pub.publish(Twist())
    threading.Thread(target=_stop, daemon=True).start()

    return jsonify({"ok": True, "direction": direction, "speed": speed, "duration": duration})


@app.route("/api/control/speed", methods=["POST"])
def api_control_speed():
    """速度调节: {speed: 0.1-0.5} 或 {delta: ±0.05}"""
    data = request.get_json() or {}
    if "delta" in data:
        _store["_speed"] = min(0.5, max(0.05, _store.get("_speed", 0.15) + float(data["delta"])))
    else:
        _store["_speed"] = min(0.5, max(0.05, float(data.get("speed", 0.15))))
    return jsonify({"speed": round(_store["_speed"], 2)})


@app.route("/api/control/patrol", methods=["POST"])
def api_control_patrol():
    """巡逻控制: {action: start|stop}"""
    data = request.get_json() or {}
    action = data.get("action", "start")

    pub = _store.get("patrol_state_pub")
    if pub is None:
        return jsonify({"error": "not ready"}), 503

    msg = String()
    msg.data = action
    pub.publish(msg)
    return jsonify({"ok": True, "action": action})


@app.route("/api/snapshots")
def api_snapshots():
    """抓拍文件列表"""
    snap_dir = Path(_store["snapshot_dir"])
    if not snap_dir.exists():
        return jsonify([])

    files = sorted(snap_dir.glob("alert_*.jpg"), reverse=True)[:50]
    return jsonify([
        {
            "filename": f.name,
            "time": datetime.fromtimestamp(f.stat().st_mtime).isoformat(),
            "size_kb": round(f.stat().st_size / 1024, 1),
        }
        for f in files
    ])


@app.route("/snapshots/<filename>")
def serve_snapshot(filename: str):
    """提供抓拍图片"""
    snap_dir = Path(_store["snapshot_dir"])
    return send_from_directory(str(snap_dir), filename)


# ── MJPEG 实时视频流 ──
import cv2, numpy as np, threading
_store["_latest_frame"] = None
_store["_frame_lock"] = threading.Lock()

@app.route("/video_feed")
def video_feed():
    """MJPEG 视频流 — 浏览器 <img src> 直出"""
    def generate():
        while True:
            with _store["_frame_lock"]:
                frame = _store.get("_latest_frame")
            if frame is not None:
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n")
            else:
                import time
                time.sleep(0.1)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ═══════════════════════════════════════════════
#  ROS2 节点
# ═══════════════════════════════════════════════

class PatrolWebNode(Node):
    """封装数据订阅, 与 Flask 共享 _store"""

    def __init__(self):
        super().__init__("patrol_web")

        # ── 读取参数 ──
        self.declare_parameter("snapshot_dir", "/home/pi/patrol_robot/snapshots/")
        _store["snapshot_dir"] = self.get_parameter("snapshot_dir").get_parameter_value().string_value

        # ── 订阅 ──
        self.state_sub = self.create_subscription(
            String, "/patrol/state", self._on_state, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, "/patrol/detections", self._on_detection, 10
        )
        self.alert_sub = self.create_subscription(
            String, "/patrol/alert_status", self._on_alert, 10
        )
        # 订阅语音命令
        self.voice_cmd_sub = self.create_subscription(
            String, "/patrol/voice_cmd", self._on_voice_cmd, 10
        )
        # 订阅相机图像用于 Web 直播
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, "/camera/rgb/image_raw", self._on_image, 10
        )

        # ── 控制发布器 ──
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        _store["cmd_vel_pub"] = self.cmd_vel_pub
        _store["patrol_state_pub"] = self.create_publisher(String, "/patrol/state_control", 10)

        self.get_logger().info("Web 面板数据采集启动")

    def _on_state(self, msg: String):
        _store["state"] = msg.data
        _store["state_since"] = datetime.now(CST).strftime("%H:%M:%S")

    def _on_detection(self, msg: Detection2DArray):
        if not msg.detections:
            return

        entry = {
            "time": datetime.now(CST).strftime("%H:%M:%S"),
            "count": len(msg.detections),
            "objects": [],
        }
        for det in msg.detections:
            if det.results:
                entry["objects"].append({
                    "id": det.results[0].id if det.results else "unknown",
                    "score": round(det.results[0].score, 2),
                    "bbox": {
                        "x": round(det.bbox.center.x, 1),
                        "y": round(det.bbox.center.y, 1),
                        "w": round(det.bbox.size_x, 1),
                        "h": round(det.bbox.size_y, 1),
                    },
                })
        _store["detections"].appendleft(entry)

    def _on_alert(self, msg: String):
        _store["alerts"].appendleft({
            "time": datetime.now(CST).strftime("%H:%M:%S"),
            "message": msg.data,
        })

    def _on_voice_cmd(self, msg: String):
        """接收语音命令"""
        _store["voice_cmds"].appendleft({
            "time": datetime.now(CST).strftime("%H:%M:%S"),
            "cmd": msg.data,
        })

    def _on_image(self, msg):
        """接收相机图像, 存入 _store 供 Web 直播"""
        try:
            # 尝试 bgr8，失败则用原始编码
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                cv_img = self.bridge.imgmsg_to_cv2(msg)
            with _store["_frame_lock"]:
                _store["_latest_frame"] = cv_img
        except Exception as e:
            self.get_logger().debug(f"image error: {e}")


# ═══════════════════════════════════════════════
#  主入口
# ═══════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    # 创建 ROS2 节点
    ros_node = PatrolWebNode()

    # Flask 在独立线程运行
    # Flask 模板路径: install/share/patrol_web/templates/
    template_dir = str(
        Path(__file__).parent.parent.parent.parent
        / "install" / "patrol_web" / "share" / "patrol_web" / "templates"
    )
    app.template_folder = template_dir

    flask_thread = threading.Thread(
        target=app.run,
        kwargs={"host": "0.0.0.0", "port": 5000, "debug": False, "use_reloader": False},
        daemon=True,
    )
    flask_thread.start()

    ros_node.get_logger().info("🌐 Web 面板: http://0.0.0.0:5000")

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
