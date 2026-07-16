# 树莓派巡逻小车 — 可靠性架构 (v4 稳定版)

> 2026-07-17 最终稳定版本

---

## 架构总览

```
树莓派启动
  │
  ├─ @reboot cron (15s) → pkill rosmaster → reload uvcvideo
  │
  ├─ Docker daemon 启动
  │   └─ patrol_car (restart=always) → container_init.sh
  │       ├─ [0] fuser -k /dev/video0
  │       ├─ [1] simple_camera (V4L2+MJPG)
  │       ├─ [2] ydlidar_driver (/dev/rplidar)
  │       ├─ [3] patrol_yolo (YOLOv5n+ONNX)
  │       ├─ [4] patrol_state_machine (IDLE, 纯检测)
  │       ├─ [5] alert_dispatcher (环形缓冲区抓拍)
  │       ├─ [6] patrol_voice (HTTP→web_server)
  │       └─ [7] patrol_web (Flask :5000 + Rosmaster_Lib 直驱底盘)
  │
  └─ Web 面板 http://192.168.31.75:5000
```

---

## 已清理的冲突源

| 组件 | 问题 | 处理 |
|:-----|:-----|:-----|
| `rosmaster.service` | systemd 开机自启，抢占摄像头+串口 | 停用+禁用 |
| `orbbec_ros_foxy` | 闲置容器，曾自动启动 | 删除 |
| `Mcnamu_driver_X3` | 不执行运动指令，占用串口 | 移除 |
| `base_node_X3` | 不响应 vel_raw | 移除 |
| `yahboomcar_ws` 依赖 | container_init.sh source 失败导致崩溃 | 移除 |
| 8个 `start_*.sh` | 可能被误执行 | 归档 |
| `_cam_check.py` | 与 simple_camera 竞争摄像头 | 移除 |

## 关键修复

| 问题 | 修复 | 文件 |
|:-----|:-----|:-----|
| 摄像头 obsensor 抢占 | `cv2.CAP_V4L2` + MJPG | simple_camera.py |
| 状态机 Nav2 循环 | 降级为 IDLE 纯检测 | patrol_state_machine.py |
| IDLE 不触发检测 | 加入 IDLE 状态处理 | patrol_state_machine.py |
| 抓拍时序滞后 | 环形缓冲区 dequeue(60) | alert_dispatcher.py |
| 车轮失控 | 0.2s watchdog 归零 | web_server.py |
| 底盘不响应 | Rosmaster_Lib 直驱 | web_server.py |
| 语音失效 | HTTP→web_server API | voice_node.py |
| USB 枚举变化 | udev ID_PATH 绑定 | 99-yahboomcar.rules |
| 时区错误 | TZ=Asia/Shanghai | docker-compose.yml |
| 环境变量缺失 | ROBOT_TYPE=x3 | docker-compose.yml |

## 启动可靠性保障

| 层级 | 机制 |
|:-----|:-----|
| 宿主机 | @reboot cron: kill rosmaster + reload uvcvideo |
| Docker | restart: always (daemon 启动自动拉起) |
| 容器内 | fuser -k /dev/video0 (释放残留占用) |
| 应用层 | V4L2+MJPG (不被 obsensor 抢占) |
| 设备层 | udev ID_PATH 绑定 (/dev/rplidar 等) |

## 文件清单

```
patrol_robot/
├── docker-compose.yml              # restart=always, 精简
├── Dockerfile
├── scripts/container_init.sh       # 7步启动, 无外部依赖
├── config/
│   ├── alert_rules.yaml
│   ├── patrol_routes.yaml
│   └── nav2_params.yaml
├── models/yolov5n.onnx
├── snapshots/                      # 抓拍存储
├── docs/
│   ├── architecture.md
│   ├── debug-summary.md
│   └── debug-rules.md
└── patrol_robot/src/
    ├── simple_camera.py            # V4L2+MJPG
    ├── ydlidar_driver.py           # /dev/rplidar
    ├── patrol_alert/alert_dispatcher.py  # 环形缓冲区
    ├── patrol_manager/patrol_state_machine.py  # IDLE降级
    ├── patrol_web/web_server.py    # Rosmaster直驱+watchdog
    ├── patrol_voice/voice_node.py  # HTTP→web_server
    └── patrol_yolo/yolo_detector.py

/etc/udev/rules.d/99-yahboomcar.rules  # ID_PATH绑定
/etc/systemd/system/rosmaster.service  # 已禁用
```

## 如果需要添加导航(SLAM+Nav2)

在 `container_init.sh` 中添加:
```bash
echo "[8] SLAM..."
ros2 launch slam_toolbox online_async_launch.py &
echo "[9] Nav2..."
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/pi/patrol_robot/config/nav2_params.yaml &
```
状态机会自动检测到 Nav2 可用→恢复全功能巡逻模式。
