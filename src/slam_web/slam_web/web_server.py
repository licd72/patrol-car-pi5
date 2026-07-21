#!/usr/bin/env python3
"""
SLAM 建图 Web 控制面板 — 独立模块，不修改任何 patrol 系统代码

Flask (port 5001) + ROS2 订阅 /scan, /map, /odom, /tf
控制: 发布 /cmd_vel → bridge → STM32
SLAM管理: docker compose 启停 slam_nav 容器
地图保存: 调用 Humble 容器内 map_saver_cli

架构原则: 只读现有话题 + 发布 /cmd_vel, 不修改 patrol 节点
"""

import json
import math
import time
import io
import base64
import subprocess
import threading
import os
import sys
import queue
from datetime import datetime, timezone, timedelta
from pathlib import Path
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan, Image as RosImage
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

from flask import Flask, render_template, jsonify, request, Response

# Pillow for map image generation
from PIL import Image
import numpy as np

CST = timezone(timedelta(hours=8))

app = Flask(__name__)

# ─── 全局状态 store ─────────────────────────────────
_store = {
    # 地图 (OccupancyGrid 转 base64 PNG)
    "map_b64": None,
    "map_info": {"width": 0, "height": 0, "resolution": 0.05, "origin_x": 0, "origin_y": 0},
    "map_stamp": 0,

    # 激光扫描 (最近一帧)
    "scan_ranges": [],
    "scan_angle_min": 0.0,
    "scan_angle_max": 0.0,
    "scan_stamp": 0,

    # 里程计位姿
    "pose": {"x": 0.0, "y": 0.0, "theta": 0.0, "stamp": 0},

    # SLAM 状态
    "slam_status": "idle",       # idle | starting | running | stopping
    "slam_msg": "",
    "map_count": 0,

    # 控制
    "last_cmd_time": 0,
    "active_cmd": None,

    # 日志
    "log": deque(maxlen=50),
}

_store_lock = threading.Lock()

def add_log(msg):
    ts = datetime.now(CST).strftime("%H:%M:%S")
    _store["log"].append(f"[{ts}] {msg}")
    print(f"[SLAM_WEB] {msg}")


# ═══════════════════════════════════════════════════
#  ROS2 节点 (在独立线程中运行)
# ═══════════════════════════════════════════════════

class SlamWebNode(Node):
    def __init__(self):
        super().__init__('slam_web_node')

        # QoS
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 订阅 /scan (ydlidar_driver 用 BEST_EFFORT 发布，必须一致)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, sensor_qos)

        # 订阅 /map (跨容器 DDS, 来自 Humble slam_toolbox)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, reliable_qos)

        # 订阅 /odom
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, sensor_qos)

        # 订阅 /tf
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self._on_tf, reliable_qos)

        # 订阅摄像头
        self._latest_jpeg = None
        self._jpeg_lock = threading.Lock()
        self.cam_sub = self.create_subscription(
            RosImage, '/camera/rgb/image_raw', self._on_image, 10)

        # 🔴 延迟初始化 Rosmaster (在单独线程中，避免阻塞 DDS 发现)
        self.bot = None
        self._rosmaster_ready = threading.Event()
        threading.Thread(target=self._init_rosmaster, daemon=True).start()

        # 🔴 线程安全命令队列 + 持续发布
        self._cmd_queue = queue.Queue(maxsize=5)
        self._current_twist = Twist()
        self._cmd_expire = 0

        # 定时器: 20Hz 持续控车 (等 Rosmaster 就绪后再启动)
        self._cmd_timer = None

        add_log("ROS2 节点初始化完成, 监听 /scan /map /odom /tf")

    def _init_rosmaster(self):
        """延迟初始化 Rosmaster"""
        try:
            sys.path.insert(0, '/home/pi/patrol_robot')
            from Rosmaster_Lib import Rosmaster
            self.bot = Rosmaster(car_type=1, com='/dev/myserial')
            time.sleep(0.3)
            self.bot.create_receive_threading()
            time.sleep(0.2)
            self.bot.set_auto_report_state(True)
            time.sleep(0.2)
            add_log(f"底盘直驱就绪, 电池: {self.bot.get_battery_voltage():.1f}V")
            self._rosmaster_ready.set()
            # 启动控车定时器
            self._cmd_timer = self.create_timer(0.05, self._cmd_tick)
        except Exception as e:
            add_log(f"❌ Rosmaster 初始化失败: {e}")

    def push_command(self, twist, duration=0.3):
        """线程安全: Flask→队列"""
        try:
            self._cmd_queue.put_nowait((twist, duration))
        except queue.Full:
            pass

    def _cmd_tick(self):
        """20Hz: 队列取命令 → Rosmaster 直驱底盘"""
        if self.bot is None:
            return  # Rosmaster 未就绪
        now = time.time()
        try:
            twist, duration = self._cmd_queue.get_nowait()
            self._current_twist = twist
            self._cmd_expire = now + duration
        except queue.Empty:
            pass
        if now > self._cmd_expire:
            self._current_twist = Twist()
        self.bot.set_car_motion(self._current_twist.linear.x,
                                self._current_twist.linear.y,
                                self._current_twist.angular.z)

    def _on_scan(self, msg: LaserScan):
        with _store_lock:
            # 降采样: 360 → 180 点
            step = max(1, len(msg.ranges) // 180)
            _store["scan_ranges"] = [msg.ranges[i] for i in range(0, len(msg.ranges), step)]
            _store["scan_angle_min"] = msg.angle_min
            _store["scan_angle_max"] = msg.angle_max
            _store["scan_stamp"] = time.time()

    def _on_map(self, msg: OccupancyGrid):
        """将 OccupancyGrid 转为 PNG base64"""
        try:
            w, h = msg.info.width, msg.info.height
            if w == 0 or h == 0:
                return

            # OccupancyGrid: 0=free, 100=occupied, -1=unknown
            # 转为灰度图像: free=255(白), occupied=0(黑), unknown=128(灰)
            img = Image.new('L', (w, h))
            pixels = img.load()
            for y in range(h):
                for x in range(w):
                    v = msg.data[y * w + x]
                    if v == -1:
                        pixels[x, h - 1 - y] = 128  # unknown → 灰
                    elif v == 0:
                        pixels[x, h - 1 - y] = 255  # free → 白
                    else:
                        pixels[x, h - 1 - y] = max(0, 255 - int(v * 2.55))  # occupied → 深色

            # 缩放到合理尺寸
            max_size = 600
            if w > max_size or h > max_size:
                ratio = max_size / max(w, h)
                img = img.resize((int(w * ratio), int(h * ratio)), Image.NEAREST)

            buf = io.BytesIO()
            img.save(buf, format='PNG')
            b64 = base64.b64encode(buf.getvalue()).decode()

            with _store_lock:
                _store["map_b64"] = b64
                _store["map_info"] = {
                    "width": msg.info.width,
                    "height": msg.info.height,
                    "resolution": msg.info.resolution,
                    "origin_x": msg.info.origin.position.x,
                    "origin_y": msg.info.origin.position.y,
                }
                _store["map_stamp"] = time.time()
        except Exception as e:
            self.get_logger().warn(f"Map conversion: {e}")

    def _on_odom(self, msg: Odometry):
        with _store_lock:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            _, _, yaw = self._quat_to_euler(ori.x, ori.y, ori.z, ori.w)
            _store["pose"] = {"x": pos.x, "y": pos.y, "theta": yaw, "stamp": time.time()}

    def _on_tf(self, msg: TFMessage):
        for tf in msg.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
                pos = tf.transform.translation
                ori = tf.transform.rotation
                _, _, yaw = self._quat_to_euler(ori.x, ori.y, ori.z, ori.w)
                with _store_lock:
                    _store["pose"] = {"x": pos.x, "y": pos.y, "theta": yaw, "stamp": time.time()}

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)
        return 0.0, 0.0, yaw

    def _on_image(self, msg: RosImage):
        """摄像头回调: BGR8 → JPEG (不用cv_bridge, ARM64不兼容)"""
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            _, jpeg = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 60])
            with self._jpeg_lock:
                self._latest_jpeg = jpeg.tobytes()
        except Exception:
            pass


# ═══════════════════════════════════════════════════
#  Docker / SLAM 生命周期管理
# ═══════════════════════════════════════════════════

COMPOSE_FILE = "/home/pi/patrol_robot/docker-compose.yml"
MAPS_DIR = "/home/pi/patrol_robot/maps"
HUMBLE_CONTAINER = "slam_nav"


# ─── Docker 操作 (容器内已安装 docker.io) ───

def _run_docker(cmd, timeout=30):
    """运行 docker 命令 (容器内 docker CLI)"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"
    except Exception as e:
        return -1, "", str(e)


def _docker_exec(container, exec_cmd, timeout=30):
    """在容器中执行命令"""
    safe_cmd = exec_cmd.replace('"', '\\"')
    return _run_docker(f'docker exec {container} bash -c "{safe_cmd}"', timeout=timeout)


def start_slam():
    """启动 SLAM: 停巡逻节点 → docker run 启动 Humble 容器"""
    if _store["slam_status"] in ("running", "starting"):
        return False, "SLAM 已运行中"

    _store["slam_status"] = "starting"
    _store["slam_msg"] = "正在停止巡逻节点..."
    add_log("🛑 停止巡逻节点 (释放内存)")

    # 停止巡逻节点 (不要杀 web_server — 会误杀 slam_web 自己!)
    # slam_web 在 :5001, patrol_web 在 :5000, 用端口区分
    patrol_nodes = [
        ("yolo_detector", "yolo_detector"),
        ("alert_dispatcher", "alert_dispatcher"),
        ("voice_node", "voice_node"),
        ("patrol_state_machine", "patrol_state_machine"),
        ("web_server", "patrol_web.web_server"),  # 精确匹配 patrol_web, 不误杀 slam_web
    ]
    for _, pattern in patrol_nodes:
        os.system(f"pkill -f '{pattern}' 2>/dev/null; true")

    time.sleep(3)
    add_log("✅ 巡逻节点已停止, 释放 ~660MB")

    # 清理旧容器 (通过 docker CLI, 连到宿主机 daemon)
    os.system("docker rm -f slam_nav 2>/dev/null; true")
    time.sleep(1)

    # 启动 Humble SLAM 容器
    _store["slam_msg"] = "正在启动 SLAM 容器..."
    add_log("🚀 启动 slam_nav 容器 (Humble + slam_toolbox)")

    ret = os.system(
        f"docker run -d --name slam_nav "
        f"--network host --privileged "
        f"--memory 2048m "
        f"-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp "
        f"-e ROS_DOMAIN_ID=42 "
        f"-e TZ=Asia/Shanghai "
        f"-v /home/pi/patrol_robot:/home/pi/patrol_robot "
        f"-v /dev:/dev -v /tmp:/tmp "
        f"--device /dev/myserial:/dev/myserial "
        f"--device /dev/rplidar:/dev/rplidar "
        f"-w /home/pi/patrol_robot/patrol_robot "
        f"patrol-nav2:humble "
        f"bash /home/pi/patrol_robot/scripts/slam_nav_init.sh")

    if ret != 0:
        _store["slam_status"] = "idle"
        msg = f"容器启动失败 (exit={ret})"
        _store["slam_msg"] = msg
        add_log(f"❌ {msg}")
        return False, msg

    add_log("📦 容器已启动, 等待 /map...")

    # 等 /map 话题
    for i in range(30):
        time.sleep(1)
        ret2 = os.system(
            "docker exec slam_nav bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null' | grep -q /map")
        if ret2 == 0:
            _store["slam_status"] = "running"
            _store["slam_msg"] = "建图中 - 使用方向键控制小车"
            add_log("✅ SLAM 建图已启动! /map 话题就绪")
            return True, "SLAM 启动成功"

    _store["slam_status"] = "running"
    _store["slam_msg"] = "建图中 (等待地图...)"
    return True, "SLAM 启动成功 (等待地图数据)"


def stop_slam():
    """停止 SLAM: 关闭 Humble 容器"""
    if _store["slam_status"] == "idle":
        return False, "SLAM 未运行"

    _store["slam_status"] = "stopping"
    _store["slam_msg"] = "正在停止..."
    add_log("🛑 停止 SLAM 建图")

    os.system("docker stop slam_nav 2>/dev/null; docker rm slam_nav 2>/dev/null; true")
    time.sleep(2)
    add_log("✅ slam_nav 容器已关闭")

    # 不自动重启 patrol — 让用户手动点击"恢复巡逻"
    _store["slam_status"] = "idle"
    _store["slam_msg"] = ""
    _store["map_b64"] = None
    add_log("✅ SLAM 已停止")
    return True, "已停止"


@app.route("/api/patrol/restore", methods=["POST"])
def api_patrol_restore():
    """恢复巡逻系统 (只在需要时手动调用)"""
    add_log("🔄 恢复巡逻节点...")
    ret = os.system("bash /home/pi/patrol_robot/scripts/container_init.sh &")
    if ret == 0:
        add_log("✅ 巡逻系统启动中...")
        return jsonify({"ok": True, "msg": "巡逻系统正在恢复"})
    else:
        return jsonify({"ok": False, "msg": f"启动失败 (exit={ret})"})


def save_map(name=None):
    """保存地图"""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    map_name = name or f"map_{ts}"

    add_log(f"💾 保存地图: {map_name}")

    ret = os.system(
        f"docker exec slam_nav bash -c '"
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 run nav2_map_server map_saver_cli -f /home/pi/patrol_robot/maps/{map_name}'")

    if ret != 0:
        add_log(f"❌ 保存失败 (exit={ret})")
        return False, f"保存失败"

    add_log(f"✅ 地图已保存: maps/{map_name}.pgm + .yaml")
    _store["map_count"] += 1
    return True, map_name


# ═══════════════════════════════════════════════════
#  Flask API 路由
# ═══════════════════════════════════════════════════

@app.route("/")
def index():
    """SLAM 控制面板主页"""
    return render_template("slam.html")


@app.route("/api/status")
def api_status():
    """获取完整状态"""
    with _store_lock:
        return jsonify({
            "slam_status": _store["slam_status"],
            "slam_msg": _store["slam_msg"],
            "map_info": _store["map_info"],
            "pose": _store["pose"],
            "map_count": _store["map_count"],
            "has_map": _store["map_b64"] is not None,
            "has_scan": len(_store["scan_ranges"]) > 0,
            "scan_age": round(time.time() - _store["scan_stamp"], 1) if _store["scan_stamp"] else -1,
            "log": list(_store["log"])[-10:],
        })


@app.route("/api/map")
def api_map():
    """地图 PNG (base64) — 文件桥优先, 回退ROS2"""
    # 文件桥: 读 /tmp/map_data.json 原始数据 → 转PNG
    try:
        if os.path.exists("/tmp/map_data.json"):
            with open("/tmp/map_data.json") as f:
                d = json.load(f)
            w, h = d["width"], d["height"]
            cells = d["cells"]
            img = Image.new('L', (w, h))
            px = img.load()
            for y in range(h):
                for x in range(w):
                    v = cells[y * w + x]
                    if v == -1: px[x, h-1-y] = 128
                    elif v == 0: px[x, h-1-y] = 255
                    else: px[x, h-1-y] = max(0, 255 - int(v*2.55))
            mw = 600
            if w > mw or h > mw:
                r = mw / max(w,h)
                img = img.resize((int(w*r), int(h*r)), Image.NEAREST)
            buf = io.BytesIO()
            img.save(buf, format='PNG')
            b64 = base64.b64encode(buf.getvalue()).decode()
            return jsonify({"b64": b64, "info": {
                "width": w, "height": h,
                "resolution": d["resolution"],
                "origin_x": d["origin_x"], "origin_y": d["origin_y"]
            }, "stamp": d["stamp"]})
    except:
        pass
    # 回退
    with _store_lock:
        return jsonify({
            "b64": _store["map_b64"],
            "info": _store["map_info"],
            "stamp": _store["map_stamp"],
        })


@app.route("/api/scan")
def api_scan():
    """激光扫描数据"""
    with _store_lock:
        return jsonify({
            "ranges": _store["scan_ranges"],
            "angle_min": _store["scan_angle_min"],
            "angle_max": _store["scan_angle_max"],
            "stamp": _store["scan_stamp"],
        })


@app.route("/api/control", methods=["POST"])
def api_control():
    """手动方向控制 → /cmd_vel"""
    data = request.get_json(force=True)
    direction = data.get("direction", "stop")
    speed = float(data.get("speed", 0.2))
    duration = float(data.get("duration", 0.3))

    # 找 ROS2 节点
    node = _ros_node
    if node is None:
        return jsonify({"ok": False, "msg": "ROS2 未就绪"})

    twist = Twist()

    if direction == "forward":
        twist.linear.x = speed
    elif direction == "backward":
        twist.linear.x = -speed
    elif direction == "left":
        twist.angular.z = speed * 3.0  # 放大角速度
    elif direction == "right":
        twist.angular.z = -speed * 3.0
    elif direction == "left_strafe":
        twist.linear.y = speed  # 麦轮左横移
    elif direction == "right_strafe":
        twist.linear.y = -speed  # 麦轮右横移
    else:  # stop
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0

    node.push_command(twist, duration)
    return jsonify({"ok": True, "direction": direction})


@app.route("/api/slam/start", methods=["POST"])
def api_slam_start():
    """启动 SLAM 建图"""
    ok, msg = start_slam()
    return jsonify({"ok": ok, "msg": msg})


@app.route("/api/slam/stop", methods=["POST"])
def api_slam_stop():
    """停止 SLAM 建图"""
    ok, msg = stop_slam()
    return jsonify({"ok": ok, "msg": msg})


@app.route("/api/save_map", methods=["POST"])
def api_save_map():
    """保存地图"""
    data = request.get_json(force=True) if request.data else {}
    name = data.get("name", "").strip() or None
    ok, result = save_map(name)
    return jsonify({"ok": ok, "name": result if ok else None, "msg": result if not ok else ""})


@app.route("/api/log")
def api_log():
    """获取日志"""
    with _store_lock:
        return jsonify({"log": list(_store["log"])})


@app.route("/video_feed")
def video_feed():
    """MJPEG 摄像头实时流"""
    def generate():
        while True:
            node = _ros_node
            if node:
                with node._jpeg_lock:
                    jpeg = node._latest_jpeg
                if jpeg:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
            time.sleep(0.1)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/api/camera")
def api_camera():
    """单帧JPEG (base64), 用于JS轮询"""
    node = _ros_node
    if node:
        with node._jpeg_lock:
            jpeg = node._latest_jpeg
        if jpeg:
            b64 = base64.b64encode(jpeg).decode()
            return jsonify({"ok": True, "jpeg": b64})
    return jsonify({"ok": False})


# ═══════════════════════════════════════════════════
#  启动入口
# ═══════════════════════════════════════════════════

_ros_node = None


def ros2_spin():
    """ROS2 主循环 (在独立线程中) - 使用 MultiThreadedExecutor"""
    global _ros_node
    rclpy.init()
    _ros_node = SlamWebNode()
    add_log("ROS2 spinning...")
    
    # 使用 MultiThreadedExecutor 避免死锁
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(_ros_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        _ros_node.destroy_node()
        rclpy.shutdown()


def main():
    """主入口"""
    # 确保 maps 目录存在
    os.makedirs(MAPS_DIR, exist_ok=True)

    # 启动 ROS2 线程
    ros_thread = threading.Thread(target=ros2_spin, daemon=True)
    ros_thread.start()

    # 等待 ROS2 就绪
    time.sleep(3)

    add_log("=" * 50)
    add_log("SLAM Web 控制面板已启动")
    add_log("浏览器访问: http://192.168.31.75:5001")
    add_log("=" * 50)

    # Flask (关闭 reloader 避免双进程)
    app.run(host="0.0.0.0", port=5001, debug=False, threaded=True)


if __name__ == "__main__":
    main()
