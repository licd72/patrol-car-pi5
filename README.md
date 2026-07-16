# 树莓派5 巡逻小车 — 容器与程序架构总结

> **日期**: 2026-07-16  
> **设备**: Raspberry Pi 5 (arm64, Debian 12 bookworm, 内核 6.6.62)  
> **IP**: 192.168.31.75  
> **ROS2**: Foxy Fitzroy (Docker 容器 `yahboomtechnology/ros-foxy:4.0.9R2`)  
> **硬件**: YDLIDAR X3 Pro, USB 2.0 Camera (Astra), YB-MAE01 语音模块, YB-ERF01 STM32 底盘

---

## 一、容器架构

```
宿主机 (Raspberry Pi 5)
└── Docker
    └── patrol_car (patrol-robot:foxy)
        ├── 镜像: yahboomtechnology/ros-foxy:4.0.9R2
        ├── 挂载: /home/pi/patrol_robot:/home/pi/patrol_robot
        │         /dev:/dev
        ├── 时区: TZ=Asia/Shanghai
        ├── 网络: host 模式
        └── restart: unless-stopped

已清理: orbbec_ros_foxy (闲置占用摄像头, 已 stop + restart=no)
```

**启动方式**: `docker compose -f ~/patrol_robot/docker-compose.yml up -d`

---

## 二、程序架构

### 2.1 ROS2 节点图 (8 节点)

```
┌─────────────────────────────────────────────────────────────┐
│ 硬件驱动层                                                    │
│                                                              │
│  /dev/video0 ──→ simple_camera ──→ /camera/rgb/image_raw    │
│  /dev/ttyUSB0 ─→ ydlidar_driver ─→ /scan                    │
│  /dev/myspeech ─→ patrol_voice  ←→ YB-MAE01 语音模块         │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ AI 推理层                                                     │
│                                                              │
│  /camera/rgb/image_raw ──→ patrol_yolo (YOLOv5n + ONNX)     │
│       ├──→ /patrol/detections   (vision_msgs/Detection2DArray)│
│       └──→ /patrol/alert_image  (sensor_msgs/Image)          │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 决策层                                                        │
│                                                              │
│  /patrol/detections ──→ patrol_state_machine                 │
│       ├──→ /patrol/state          (IDLE/NAVIGATING/TRACKING…)│
│       └──→ /patrol/alert_trigger  (触发报警)                  │
│                                                              │
│  /patrol/alert_trigger ──→ alert_dispatcher                  │
│       ├──→ 钉钉机器人推送 (Webhook)                            │
│       ├──→ 飞书机器人推送 (备用)                                │
│       ├──→ GPIO 声光报警 (蜂鸣器+LED)                           │
│       └──→ 抓拍存证 (环形缓冲区 → snapshots/*.jpg)             │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 展示层                                                        │
│                                                              │
│  patrol_web (Flask :5000)                                    │
│       ├── GET  /                   仪表盘 HTML                │
│       ├── GET  /api/state          巡逻状态 JSON              │
│       ├── GET  /api/snapshots      抓拍文件列表               │
│       ├── GET  /snapshots/<fn>     抓拍图片                   │
│       ├── GET  /video_feed         MJPEG 实时视频流            │
│       └── POST /api/control/move   遥控底盘                   │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 话题清单

| 话题 | 类型 | 发布者 → 订阅者 |
|:-----|:-----|:-----|
| `/camera/rgb/image_raw` | Image | simple_camera → patrol_yolo, patrol_web |
| `/scan` | LaserScan | ydlidar_driver |
| `/patrol/detections` | Detection2DArray | patrol_yolo → patrol_state_machine, patrol_web |
| `/patrol/alert_image` | Image | patrol_yolo → alert_dispatcher |
| `/patrol/state` | String | patrol_state_machine → patrol_web, patrol_voice, alert_dispatcher |
| `/patrol/alert_trigger` | String | patrol_state_machine → alert_dispatcher, patrol_voice |
| `/patrol/alert_status` | String | alert_dispatcher → patrol_web, patrol_voice |
| `/patrol/voice_cmd` | String | patrol_voice → patrol_web |
| `/cmd_vel` | Twist | patrol_voice / patrol_web / patrol_state_machine |

### 2.3 状态机流转

```
IDLE → NAVIGATING → SCANNING → (YOLO检测到人?) → TRACKING
                                                      │
                                        ┌─ 确认(≥60%) ──┤
                                        ▼                │
                                     ALERTING ←── 未确认  │
                                        │                │
                                        ▼                ▼
                                     (冷却30s) → NAVIGATING
```

---

## 三、文件结构

```
/home/pi/patrol_robot/
│
├── docker-compose.yml              # 容器编排
├── Dockerfile                      # 镜像定义 (基于 yahboomtechnology/ros-foxy:4.0.9R2)
│
├── docs/
│   ├── architecture.md             # 架构设计文档
│   └── hardware_bom.md             # 硬件物料清单
│
├── config/
│   ├── nav2_params.yaml            # Nav2 导航参数
│   ├── patrol_routes.yaml          # 巡逻路线 (6个预置点)
│   ├── alert_rules.yaml            # 报警规则 (max_per_alert=20)
│   └── slam_params.yaml            # SLAM 参数
│
├── models/
│   └── yolov5n.onnx                # YOLOv5n ONNX 模型 (~10MB)
│
├── scripts/
│   ├── container_init.sh           # ★ 容器启动入口
│   ├── deploy_rpi5.sh              # 部署脚本
│   ├── patrol_start.sh             # 宿主机启动 (备用)
│   ├── patrol_start_docker.sh      # Docker内启动 (备用)
│   ├── start_drivers.sh            # 驱动启动
│   └── download_model.py           # 模型下载
│
├── snapshots/                      # 抓拍存储 (~100+ jpg)
│
└── patrol_robot/                   # ROS2 工作空间
    ├── build/                      # colcon build (symlink-install)
    ├── install/                    # colcon install
    │
    └── src/                        # ★ 源代码
        ├── _cam_check.py           # 摄像头验证脚本 (NEW)
        │
        ├── patrol_bringup/         # 总启动 launch
        │   └── launch/patrol_all.launch.py
        │
        ├── patrol_yolo/            # YOLO 检测节点
        │   └── patrol_yolo/yolo_detector.py
        │
        ├── patrol_manager/         # ★ 状态机核心
        │   └── patrol_manager/patrol_state_machine.py
        │
        ├── patrol_alert/           # ★ 报警调度 (含环形缓冲区)
        │   └── patrol_alert/alert_dispatcher.py
        │
        ├── patrol_voice/           # 语音交互
        │   └── patrol_voice/voice_node.py
        │
        ├── patrol_web/             # Web 监控面板
        │   ├── patrol_web/web_server.py
        │   └── templates/dashboard.html
        │
        ├── simple_camera.py        # 相机驱动
        ├── ydlidar_driver.py       # 激光雷达驱动
        └── Speech_Lib/             # 语音库
```

---

## 四、启动流程

```bash
# 1. 停掉冲突容器
docker stop orbbec_ros_foxy

# 2. 启动巡逻系统
cd ~/patrol_robot && docker compose up -d

# 3. 访问 Web 面板
# http://192.168.31.75:5000
```

**container_init.sh 启动顺序** (共 8 步):

| 步骤 | 节点 | 作用 |
|:--:|:-----|:-----|
| [0] | — | 释放 /dev/video0 + 摄像头验证 |
| [1] | Mcnamu_driver_X3 | 麦轮底盘驱动 |
| [2] | simple_camera | 摄像头 → /camera/rgb/image_raw |
| [3] | ydlidar_driver | 激光雷达 → /scan |
| [4] | patrol_yolo | YOLOv5 检测 → /patrol/detections |
| [5] | patrol_state_machine | 状态机 → /patrol/state |
| [6] | alert_dispatcher | 报警调度 + 抓拍 |
| [7] | patrol_voice | 语音交互 |
| [8] | patrol_web | Web 面板 :5000 |

---

## 五、调试中遇到的问题与修复

### 🔴 P0: 抓拍画面为空场景

**现象**: Web 面板能看到抓拍图片，但画面是空场景，不是检测到人的瞬间。

**根因**: YOLO 检测到人时立即发布 `/patrol/alert_image`，但 `alert_dispatcher` 要等状态机 10 秒跟踪确认后才开始保存。10 秒后人已离开画面。

**修复**: `alert_dispatcher.py` 添加 `deque(maxlen=60)` 环形缓冲区，始终缓存最近 60 帧 (~30秒)。报警触发时从缓冲区回写"检测到人的那一刻"的帧。

**修改文件**: `patrol_alert/patrol_alert/alert_dispatcher.py`

---

### 🔴 P1: 状态机 TRACKING 振荡

**现象**: 状态机在 `TRACKING ↔ NAVIGATING` 之间快速切换，永远达不到 60% 确认率，报警无法触发。

**根因**: `_start_tracking` 每次被 YOLO 检测触发都创建新的 `create_timer`，不取消旧 timer。旧 timer 残留堆积，各自触发 `_evaluate_tracking`，导致状态混乱。

**修复**:
1. 保存 timer 引用，新前 cancel 旧 (`_track_timer`, `_rotate_timers`, `_delay_timer`)
2. `_start_tracking` 加 5 秒冷却，避免频繁重置跟踪窗口

**修改文件**: `patrol_manager/patrol_manager/patrol_state_machine.py`

---

### 🟡 P2: 摄像头被占用

**现象**: `simple_camera` 启动成功但 `cap.read()` 始终返回空帧，YOLO 无输入，系统静默失效。

**根因**: 闲置容器 `orbbec_ros_foxy` 先占用了 `/dev/video0`，两个容器竞争同一设备。

**修复**:
1. `container_init.sh` 启动前执行 `fuser -k /dev/video0` 强制释放
2. 新增 `_cam_check.py` 循环验证摄像头可读 (最多 5 次重试)
3. `docker update --restart=no orbbec_ros_foxy && docker stop orbbec_ros_foxy`

**修改文件**: `scripts/container_init.sh`, 新增 `src/_cam_check.py`

---

### 🟡 P3: 容器反复重启

**现象**: 容器 `Restarting (1)` 循环，`set -e` 导致脚本遇错即退。

**根因**: `container_init.sh` 中 `source /home/pi/yahboomcar_ws/install/setup.bash` 文件在容器内不存在。

**修复**:
1. 去掉 `set -e`，改为关键步骤单独容错 `|| true`
2. source 路径加 `2>/dev/null || true`
3. 重写启动脚本，步骤编号清晰，每步独立

**修改文件**: `scripts/container_init.sh`

---

### 🟡 P4: 抓拍文件时间戳错误

**现象**: 抓拍文件名中的时间比北京时间少 8 小时 (UTC vs CST)。

**根因**: Docker 容器默认 UTC 时区。

**修复**:
1. `container_init.sh` 加 `export TZ='Asia/Shanghai'`
2. `docker-compose.yml` 加 `environment: TZ=Asia/Shanghai`
3. `alert_dispatcher._dump_buffer` 中 `datetime.fromtimestamp(ts, tz=CST)`

**修改文件**: `container_init.sh`, `docker-compose.yml`, `alert_dispatcher.py`

---

### 🟢 P5: 抓拍数量不足

**现象**: 每次报警只保存 5 张抓拍，可能错过关键画面。

**修复**: `max_snapshots_per_alert` 默认值 5 → 20，`alert_rules.yaml` 中 `max_per_alert: 5 → 20`

---

## 六、关键代码片段

### 环形缓冲区 (alert_dispatcher.py)

```python
# 初始化
self._image_buffer = deque(maxlen=60)  # 缓存60帧 ≈ 30秒

# 始终缓存 (不检查 _alert_active)
def _on_alert_image(self, msg: Image):
    cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    self._image_buffer.append((time.time(), cv_img))

# 报警触发时回写
def _dump_buffer(self):
    buf_len = len(self._image_buffer)
    step = max(1, buf_len // self.max_snapshots)
    for i in range(0, buf_len, step):
        ts, cv_img = self._image_buffer[i]
        timestamp = datetime.fromtimestamp(ts, tz=CST).strftime(...)
        cv2.imwrite(filename, cv_img)
    self._image_buffer.clear()
```

### Timer 管理 (patrol_state_machine.py)

```python
# 修复前: timer 堆积
self.create_timer(self.track_duration, self._evaluate_tracking)

# 修复后: 取消旧 timer
if self._track_timer is not None:
    self._track_timer.cancel()
self._track_timer = self.create_timer(self.track_duration, self._evaluate_tracking)
```

---

## 七、备份文件清单

| 文件 | 备份 |
|:-----|:-----|
| `alert_dispatcher.py` | `alert_dispatcher.py.bak` |
| `patrol_state_machine.py` | `patrol_state_machine.py.bak2`, `.bak3` |
| `container_init.sh` | `container_init.sh.bak` |
| `alert_rules.yaml` | 仅改一个数字, 无备份 |

---

## 八、已知遗留问题

1. **Nav2 不可用时状态机退化**: 无导航栈时状态机在 NAVIGATING 循环重试，不影响检测和抓拍，但无法实际移动巡逻
2. **Timer 残留**: 修复前创建的旧 timer 在 ROS2 队列中无法取消，随运行逐渐消耗
3. **Docker Compose version 警告**: `docker-compose.yml` 中 `version: "3.8"` 已废弃，不影响功能
4. **YOLO 模型固定**: `yolov5n.onnx` 的 5 类目标 (person/car/motorcycle/bus/truck)，如需检测其他类型需更换模型
