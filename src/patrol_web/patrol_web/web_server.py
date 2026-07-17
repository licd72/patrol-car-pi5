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
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import io
import math as _math

from flask import Flask, render_template, jsonify, send_from_directory, request, Response

app = Flask(__name__)

# 全局数据存储 (ROS2 线程写入, Flask 线程读取)
_store = {
    "map_data": None,      # {w, h, res, ox, oy, cells: bytes}
    "map_stamp": 0,        # 更新时间
    "pose": {"x": 0.0, "y": 0.0, "theta": 0.0, "stamp": 0},
    "state": "IDLE",  # 默认 IDLE (Nav2不可用时状态机降级为IDLE)
    "state_since": "",
    "voltage": 0.0,
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


# ─── 地图 + 位姿 API ──────────────────────────
@app.route("/api/pose")
def api_pose():
    return jsonify(_store.get("pose") or {"x": 0, "y": 0, "theta": 0, "stamp": 0})


@app.route("/api/map/meta")
def api_map_meta():
    m = _store.get("map_data")
    if not m:
        return jsonify({"ok": False, "reason": "no map yet"})
    return jsonify({
        "ok": True,
        "width": m["w"],
        "height": m["h"],
        "resolution": m["res"],
        "origin_x": m["ox"],
        "origin_y": m["oy"],
        "stamp": _store.get("map_stamp", 0),
    })


@app.route("/api/map.png")
def api_map_png():
    """把 OccupancyGrid 编码成 PNG (dark theme: 空闲=浅灰, 障碍=红, 未知=深底)"""
    m = _store.get("map_data")
    if not m:
        return Response(b"", mimetype="image/png", status=204)
    try:
        from PIL import Image
    except ImportError:
        return Response(b"pillow not installed", status=500)

    w, h = m["w"], m["h"]
    cells = m["cells"]  # list[int]

    # RGB: 深色主题
    rgb = bytearray(w * h * 3)
    for i, c in enumerate(cells):
        if c == -1:      # 未知 → 深灰
            r, g, b = 30, 41, 59
        elif c > 65:      # 障碍 → 亮红
            r, g, b = 239, 68, 68
        elif c < 25:      # 空闲 → 浅灰
            r, g, b = 226, 232, 240
        else:             # 边缘 → 中灰
            r, g, b = 100, 116, 139
        rgb[i*3] = r
        rgb[i*3+1] = g
        rgb[i*3+2] = b

    # PGM/OccupancyGrid 是 bottom-left origin, 图像是 top-left → 垂直翻转行
    row = w * 3
    rows = [bytes(rgb[i*row:(i+1)*row]) for i in range(h)]
    rgb_flipped = b"".join(reversed(rows))

    img = Image.frombytes("RGB", (w, h), rgb_flipped)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)
    resp = Response(buf.read(), mimetype="image/png")
    resp.headers["Cache-Control"] = "no-cache, max-age=0"
    return resp


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
    pub_raw = _store.get("vel_raw_pub")
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

    # 10Hz 持续发布 duration 秒 → patrol_bridge 会转发到 STM32
    # (旧版本 _get_chassis 直驱已移除, patrol_bridge 是唯一 STM32 写入者)
    _store["_last_cmd_time"] = time.time()
    _store["_cmd_active"] = True

    def _publish_loop():
        import time as _t
        end = _t.time() + duration
        while _t.time() < end:
            pub.publish(twist)
            if pub_raw:
                pub_raw.publish(twist)
            _t.sleep(0.1)
        zero = Twist()
        pub.publish(zero)
        if pub_raw:
            pub_raw.publish(zero)
        _store["_cmd_active"] = False
        _store["_last_cmd_time"] = _t.time()
    threading.Thread(target=_publish_loop, daemon=True).start()

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
    """抓拍文件列表 (首页缩略图, 默认最近20张)"""
    snap_dir = Path(_store["snapshot_dir"])
    if not snap_dir.exists():
        return jsonify([])
    limit = request.args.get("limit", 10, type=int)
    files = sorted(snap_dir.glob("alert_*.jpg"), reverse=True)[:limit]
    return jsonify([
        {
            "filename": f.name,
            "time": datetime.fromtimestamp(f.stat().st_mtime).isoformat(),
            "size_kb": round(f.stat().st_size / 1024, 1),
        }
        for f in files
    ])

@app.route("/api/snapshots/log")
def api_snapshots_log():
    """抓拍日志: 分页查询, 支持日期筛选"""
    snap_dir = Path(_store["snapshot_dir"])
    if not snap_dir.exists():
        return jsonify({"files": [], "total": 0, "page": 1, "pages": 0, "dates": []})

    page = request.args.get("page", 1, type=int)
    per_page = request.args.get("per_page", 30, type=int)
    date_filter = request.args.get("date", "")

    all_files = sorted(snap_dir.glob("alert_*.jpg"), reverse=True)
    if date_filter:
        all_files = [f for f in all_files if f.name.startswith("alert_" + date_filter)]

    total = len(all_files)
    pages = max(1, (total + per_page - 1) // per_page)
    start = (page - 1) * per_page
    files = all_files[start:start + per_page]

    dates = sorted(set(f.name[6:14] for f in snap_dir.glob("alert_*.jpg")), reverse=True)

    return jsonify({
        "files": [{
            "filename": f.name,
            "time": datetime.fromtimestamp(f.stat().st_mtime).isoformat(),
            "size_kb": round(f.stat().st_size / 1024, 1),
        } for f in files],
        "total": total, "page": page, "pages": pages, "dates": dates,
    })


@app.route("/snapshots/<filename>")
def serve_snapshot(filename: str):
    """提供抓拍图片"""
    snap_dir = Path(_store["snapshot_dir"])
    return send_from_directory(str(snap_dir), filename)

@app.route("/api/snapshots/<filename>", methods=["DELETE"])
def delete_snapshot(filename: str):
    """删除单张抓拍"""
    snap_dir = Path(_store["snapshot_dir"])
    filepath = snap_dir / filename
    if filepath.exists():
        filepath.unlink()
        return jsonify({"ok": True, "deleted": filename})
    return jsonify({"error": "not found"}), 404

@app.route("/api/snapshots/delete_all", methods=["POST"])
def delete_all_snapshots():
    """批量删除: 全部 或 按日期"""
    snap_dir = Path(_store["snapshot_dir"])
    if not snap_dir.exists():
        return jsonify({"deleted": 0})
    req = request.get_json() or {}
    date_filter = req.get("date", "")
    if date_filter:
        files = [f for f in snap_dir.glob("alert_*.jpg") if f.name.startswith("alert_" + date_filter)]
    else:
        files = list(snap_dir.glob("alert_*.jpg"))
    count = 0
    for f in files:
        f.unlink()
        count += 1
    return jsonify({"deleted": count})


# ── 系统健康 API ──

@app.route("/api/health")
def api_health():
    """系统健康状态: 电压/温度/内存/磁盘"""
    import os as _os
    health = {"voltage": None, "cpu_temp": None, "mem": None, "disk": None}

    # 电池电压 (从 bridge 发布的 /patrol/voltage 话题获取)
    health["voltage"] = _store.get("voltage", 0.0) or None

    # CPU 温度
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            health["cpu_temp"] = round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        pass

    # 内存
    try:
        with open("/proc/meminfo") as f:
            lines = f.read().split("\n")
        mem = {}
        for l in lines:
            if "MemTotal" in l: mem["total"] = int(l.split()[1])
            if "MemAvailable" in l: mem["avail"] = int(l.split()[1])
        if mem:
            health["mem"] = {
                "total_mb": round(mem["total"] / 1024, 1),
                "avail_mb": round(mem["avail"] / 1024, 1),
                "used_pct": round((1 - mem["avail"]/mem["total"]) * 100, 1),
            }
    except Exception:
        pass

    # 磁盘 (根分区)
    try:
        st = _os.statvfs("/")
        total_gb = round(st.f_blocks * st.f_frsize / 1073741824, 1)
        avail_gb = round(st.f_bavail * st.f_frsize / 1073741824, 1)
        health["disk"] = {
            "total_gb": total_gb,
            "avail_gb": avail_gb,
            "used_pct": round((1 - st.f_bavail / st.f_blocks) * 100, 1),
        }
    except Exception:
        pass

    # CPU 负载 (1分钟平均)
    try:
        with open("/proc/loadavg") as f:
            health["load"] = float(f.read().split()[0])
    except Exception:
        pass

    return jsonify(health)

# ── 巡逻路线编辑 API ──

import yaml as _yaml
from pathlib import Path as _Path

ROUTES_FILE = str(_Path(__file__).parent.parent.parent.parent.parent / "config" / "patrol_routes.yaml")

def _load_routes():
    if not _Path(ROUTES_FILE).exists():
        return {"routes": {}}
    with open(ROUTES_FILE, "r", encoding="utf-8") as f:
        return _yaml.safe_load(f) or {"routes": {}}

def _save_routes(data):
    with open(ROUTES_FILE, "w", encoding="utf-8") as f:
        _yaml.dump(data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)


@app.route("/api/routes")
def api_routes():
    data = _load_routes()
    return jsonify(data)

@app.route("/api/routes/waypoint", methods=["POST"])
def api_add_waypoint():
    req = request.get_json() or {}
    route_name = req.get("route", "campus_1f")
    data = _load_routes()
    routes = data.setdefault("routes", {})
    route = routes.setdefault(route_name, {"description": "", "waypoints": []})
    wp = {
        "name": req.get("name", "P" + str(len(route["waypoints"]) + 1)),
        "x": float(req.get("x", 0)),
        "y": float(req.get("y", 0)),
        "yaw": float(req.get("yaw", 0)),
        "dwell": int(req.get("dwell", 3)),
    }
    route["waypoints"].append(wp)
    _save_routes(data)
    return jsonify({"ok": True, "waypoint": wp})

@app.route("/api/routes/waypoint/<int:idx>", methods=["PUT", "DELETE"])
def api_edit_waypoint(idx):
    route_name = request.args.get("route", "campus_1f")
    data = _load_routes()
    route = data.get("routes", {}).get(route_name, {})
    wps = route.get("waypoints", [])
    if idx < 0 or idx >= len(wps):
        return jsonify({"error": "index out of range"}), 400
    if request.method == "DELETE":
        removed = wps.pop(idx)
        _save_routes(data)
        return jsonify({"ok": True, "removed": removed})
    else:
        req = request.get_json() or {}
        wp = wps[idx]
        if "name" in req: wp["name"] = req["name"]
        if "x" in req: wp["x"] = float(req["x"])
        if "y" in req: wp["y"] = float(req["y"])
        if "yaw" in req: wp["yaw"] = float(req["yaw"])
        if "dwell" in req: wp["dwell"] = int(req["dwell"])
        _save_routes(data)
        return jsonify({"ok": True, "waypoint": wp})

@app.route("/api/routes/record_position", methods=["POST"])
def api_record_position():
    req = request.get_json() or {}
    route_name = req.get("route", "campus_1f")
    name = req.get("name", "P" + str(int(__import__("time").time()) % 10000))
    x = float(req.get("x", 0))
    y = float(req.get("y", 0))
    yaw = float(req.get("yaw", 0))
    data = _load_routes()
    route = data.setdefault("routes", {}).setdefault(route_name, {"description": "", "waypoints": []})
    route["waypoints"].append({"name": name, "x": x, "y": y, "yaw": yaw, "dwell": 3})
    _save_routes(data)
    return jsonify({"ok": True, "name": name, "x": x, "y": y})


# ── MJPEG 实时视频流 ──
import cv2, numpy as np, threading
_store["_latest_frame"] = None
_store["_frame_lock"] = threading.Lock()

@app.route("/video_feed")
def video_feed():
    """MJPEG 视频流 — 浏览器 <img src> 直出"""
    def generate():
        import time as _t
        while True:
            with _store["_frame_lock"]:
                frame = _store.get("_latest_frame")
            if frame is not None:
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n")
                _t.sleep(0.1)  # 10fps 限速, 降低 CPU
            else:
                _t.sleep(0.2)
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

        # ── 地图 (transient_local, RELIABLE) ──
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        # ── 里程计 (odom → pose) ──
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # ── 电池电压 (bridge 发布) ──
        self.voltage_sub = self.create_subscription(Float32, "/patrol/voltage", self._on_voltage, 10)

        # ── 控制发布器 ──
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.vel_raw_pub = self.create_publisher(Twist, "/vel_raw", 10)
        _store["cmd_vel_pub"] = self.cmd_vel_pub
        _store["vel_raw_pub"] = self.vel_raw_pub
        _store["patrol_state_pub"] = self.create_publisher(String, "/patrol/state_control", 10)

        self.get_logger().info("Web 面板数据采集启动")

    def _on_map(self, msg: OccupancyGrid):
        _store["map_data"] = {
            "w": msg.info.width,
            "h": msg.info.height,
            "res": msg.info.resolution,
            "ox": msg.info.origin.position.x,
            "oy": msg.info.origin.position.y,
            "cells": list(msg.data),
        }
        _store["map_stamp"] = time.time()

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw from quaternion (z,w 主分量, roll/pitch 忽略)
        theta = _math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        _store["pose"] = {
            "x": round(msg.pose.pose.position.x, 3),
            "y": round(msg.pose.pose.position.y, 3),
            "theta": round(theta, 3),
            "stamp": time.time(),
        }

    def _on_voltage(self, msg: Float32):
        """接收 bridge 发布的电池电压"""
        _store["voltage"] = round(msg.data, 2)

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
    # Flask 模板路径: 优先 install/share (colcon 布局), fallback src/
    _candidates = [
        Path("/home/pi/patrol_robot/patrol_robot/install/patrol_web/share/patrol_web/templates"),
        Path("/home/pi/patrol_robot/patrol_robot/src/patrol_web/templates"),
        Path(__file__).resolve().parent.parent.parent.parent.parent / "install" / "patrol_web" / "share" / "patrol_web" / "templates",
    ]
    template_dir = None
    for _c in _candidates:
        if (_c / "dashboard.html").exists():
            template_dir = str(_c)
            break
    if template_dir is None:
        template_dir = str(_candidates[0])
    app.template_folder = template_dir
    ros_node.get_logger().info(f"template_folder = {template_dir}")

    flask_thread = threading.Thread(
        target=app.run,
        kwargs={"host": "0.0.0.0", "port": 5000, "debug": False, "use_reloader": False},
        daemon=True,
    )
    flask_thread.start()

    # ── 安全 Watchdog: 持续发零速度，防止 STM32 残留指令 ──
    # 原理: base_node_X3 不自动超时，需要持续收到零速度来覆盖旧指令
    _store["_last_cmd_time"] = 0.0
    _store["_cmd_active"] = False

    def _cmd_watchdog():
        import time as _time
        from geometry_msgs.msg import Twist as _Twist
        pub = _store.get("cmd_vel_pub")
        while pub is not None:
            _time.sleep(0.2)
            now = _time.time()
            active = _store.get("_cmd_active", False)
            last = _store.get("_last_cmd_time", 0)
            # 超过 0.8 秒没有新命令 → 强制归零
            if not active or now - last > 0.8:
                zero = _Twist()
                pub.publish(zero)
                pub_raw = _store.get("vel_raw_pub")
                if pub_raw:
                    pub_raw.publish(zero)
                # 停车走 /cmd_vel → patrol_bridge, 不再直接操作 STM32 串口
                _store["_cmd_active"] = False

    threading.Thread(target=_cmd_watchdog, daemon=True).start()

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
