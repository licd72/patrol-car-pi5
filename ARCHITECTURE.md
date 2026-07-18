# X3 巡逻小车 — 完整架构与部署文档

> 固化日期: 2026-07-18
> GitHub: https://github.com/licd72/patrol-car-pi5
> 硬件: 树莓派5 + 亚博智能 Rosmaster X3 麦克纳姆轮小车

---

## 1. USB 物理拓扑（树莓派5 四个 USB 端口）

```
Pi5 物理端口                          USB Hub          芯片/设备         ttyUSB    符号链接
═══════════════════════════════════════════════════════════════════════════════════
USB 2.0 端口1 (上左) ─── Bus 001 ─┬─ Port 2 ── CH341 (QinHeng 1a86:7522)  ttyUSB0 → /dev/myspeech (语音)
                                  └─ Port 1 ── (空)

USB 3.0 端口2 (上右) ─── Bus 002 ─── Port 1 ── USB 存储 (Lenovo 128GB)

USB 2.0 端口3 (下左) ─── Bus 003 ─┬─ Port 1 ── VIA USB Hub ─┬─ Port 2 ── CP2102 (Silicon Labs 10c4:ea60)  ttyUSB1 → /dev/rplidar (激光雷达)
                                  │                          ├─ Port 3 ── CH341 (QinHeng 1a86:7523)        ttyUSB4 → /dev/myserial (STM32)  ← 关键!
                                  │                          ├─ Port 4 ── Genesys Hub ─┬─ Port 1 ── USB 音频
                                  │                          │                        └─ Port 2 ── 摄像头     video0
                                  │                          └─ Port 5 ── Billboard
                                  │
                                  └─ Port 2 ── CH341 (QinHeng 1a86:7523)              ttyUSB2 → (未使用/备用)

USB 3.0 端口4 (下右) ─── Bus 004 ─── Port 1 ── VIA USB 3.0 Hub (空)
```

### 1.1 设备详细信息

| ttyUSB | 总线:端口.子端口 | USB VID:PID | 芯片 | 驱动 | 速度 | udev 符号链接 | 硬件用途 |
|:--|:--|:--|:--|:--|:--|:--|:--|
| **ttyUSB0** | Bus1:Port2 | `1a86:7522` | CH340 | ch341 | 12M | `/dev/myspeech` | 语音识别 ASR 模块 |
| **ttyUSB1** | Bus3:Port1.Port2 | `10c4:ea60` | **CP2102** | cp210x | 12M | `/dev/rplidar` | YDLIDAR X4 激光雷达 |
| ttyUSB2 | Bus3:Port2 | `1a86:7523` | CH340 | ch341 | 12M | — | 未使用 (备用 CH340) |
| **ttyUSB4** | Bus3:Port1.Port3 | `1a86:7523` | **CH340** | ch341 | 12M | `/dev/myserial` | **STM32 底盘控制板** |

### 1.2 摄像头

| 设备 | 总线路径 | 驱动 | 分辨率 |
|:--|:--|:--|:--|
| **/dev/video0** | Bus3:Port1.Port4.Port2 | uvcvideo | 320×240 MJPG |

### 1.3 备注

- Pi5 有 4 个物理 USB 口：2个 USB 2.0 (上层), 2个 USB 3.0 (下层/蓝色)
- Bus 003 通过一个 **VIA Labs USB 2.0 Hub** (VID `2109:2817`) 扩展出 5 个端口
- 激光雷达 (CP2102) 和 STM32 (CH340) 共享同一个 VIA Hub，在 **Bus 003 Port 1** 下
- 语音模块 (CH340) 独占 **Bus 001 Port 2**
- udev 符号链接文件: `/etc/udev/rules.d/99-mydevices.rules`

---

## 2. 依赖清单

### 2.1 底盘驱动库

| 项目 | 详情 |
|:--|:--|
| **库名** | `Rosmaster_Lib.py` |
| **版本** | V1.5.8 |
| **行数** | 1139 行 |
| **来源** | 亚博智能 Yahboom 原厂 |
| **路径** | `/home/pi/patrol_robot/Rosmaster_Lib.py` |
| **类** | `class Rosmaster(car_type=1, com="/dev/myserial", delay=.002, debug=False)` |

**核心方法与协议参数**:

```python
# 运动控制 (全向麦轮)
bot.set_car_motion(v_x, v_y, v_z)        # v_x/v_y: -1.0~1.0 m/s, v_z: -5~5 rad/s

# 传感器
bot.create_receive_threading()            # 启动接收线程 (必须!)
bot.set_auto_report_state(True)           # 开启自动上报 (必须!)
bot.get_battery_voltage()                 # 电池电压 (V)
bot.get_motor_encoder()                   # 4轮编码器 (M1,M2,M3,M4)
bot.get_imu_attitude_data()              # IMU (roll, pitch, yaw)
bot.get_motion_data()                    # 速度反馈 (vx, vy, vz)

# STM32 协议参数
HEAD = 0xFF                               # 帧头
DEVICE_ID = 0xFC                          # 发送帧设备ID
DEVICE_ID - 1 = 0xFB                      # 接收帧匹配ID
FUNC_MOTION = 0x12                        # 运动控制指令
FUNC_MOTOR = 0x10                         # 独立电机指令
CAR_ADJUST = 0x80                         # 车型偏移
CARTYPE_MECANUM_MINI = 0x01              # X3 麦轮类型
# 实际发送 car_type = 0x01 | 0x80 = 0x81
```

### 2.2 Python 依赖

| 包 | 版本 | 用途 |
|:--|:--|:--|
| `pyserial` | 3.4 | 串口通信 |
| `Flask` | 1.1.1 | Web 控制面板 |
| `numpy` | 1.24.4 | YOLO 数值计算 |
| `rclpy` | Foxy | ROS2 Python API |

### 2.3 ROS2 系统包

| 包 | 版本 | 用途 |
|:--|:--|:--|
| `ros-foxy-ros-base` | Foxy (Ubuntu 20.04) | ROS2 基础 |
| `ros-foxy-rmw-fastrtps-cpp` | 1.3.2 | **Fast-DDS** (生产使用) |
| `ros-foxy-rmw-cyclonedds-cpp` | — | CycloneDDS (禁止使用,ARM+Docker有bug) |
| `ros-foxy-geometry-msgs` | — | Twist 消息类型 |
| `ros-foxy-std-msgs` | — | String/Float32 |
| `ros-foxy-nav-msgs` | — | Odometry |
| `ros-foxy-cv-bridge` | — | ROS↔OpenCV 图像转换 |

### 2.4 语音依赖

| 模块 | 路径 |
|:--|:--|
| `Speech_Lib` | `patrol_robot/src/Speech_Lib/Speech_Lib.py` |
| `PYTHONPATH` | `export PYTHONPATH="/home/pi/patrol_robot/patrol_robot/src/Speech_Lib:$PYTHONPATH"` |

---

## 3. 目录结构

```
/home/pi/patrol_robot/                         ← 顶层仓库
│
├── Rosmaster_Lib.py                          ← ★ 底盘驱动库 (原厂 V1.5.8, 1139行)
├── docker-compose.yml                        ← 容器编排
├── Dockerfile                                ← Foxy 镜像构建
├── ARCHITECTURE.md                           ← 本文档
│
├── scripts/
│   ├── container_init.sh                     ← patrol_car 容器启动脚本
│   ├── serial_worker.py                      ← 串口驱动子进程 (FIFO→STM32, 0 rclpy)
│   ├── deploy_rpi5.sh
│   ├── download_model.py
│   └── start_slam.sh
│
├── patrol_robot/
│   ├── src/                                  ← 源码 (开发用, 需同步到 install/)
│   │   ├── Rosmaster_Lib.py                  ← 底盘库副本
│   │   ├── simple_camera.py                  ← 摄像头驱动
│   │   ├── ydlidar_driver.py                 ← 激光雷达驱动
│   │   ├── Speech_Lib/
│   │   │   ├── __init__.py
│   │   │   └── Speech_Lib.py
│   │   ├── patrol_bridge/                    ← ★ 底盘桥节点
│   │   │   └── patrol_bridge/cmd_vel_bridge.py
│   │   ├── patrol_web/                       ← ★ Web 面板
│   │   │   ├── patrol_web/web_server.py
│   │   │   └── templates/
│   │   ├── patrol_voice/                     ← ★ 语音控制
│   │   │   └── patrol_voice/voice_node.py
│   │   ├── patrol_yolo/
│   │   │   └── patrol_yolo/yolo_detector.py
│   │   ├── patrol_manager/
│   │   │   └── patrol_manager/patrol_state_machine.py
│   │   └── patrol_alert/
│   │       └── patrol_alert/alert_dispatcher.py
│   │
│   ├── install/                              ← colcon build 产物 (容器加载此目录)
│   │   ├── setup.bash
│   │   ├── share/                            ← 模板/launch 文件
│   │   └── lib/python3.8/site-packages/      ← ★ Python 包 (容器实际运行)
│   │       ├── patrol_bridge/cmd_vel_bridge.py
│   │       ├── patrol_web/web_server.py
│   │       └── patrol_voice/voice_node.py
│   │
│   └── build/                                ← colcon 中间产物 (可删除)
│
├── models/                                   ← YOLOv5 模型
└── snapshots/                                ← 抓拍存储
```

---

## 4. Docker 信息

### 4.1 镜像

| 镜像 | 大小 | 用途 |
|:--|:--|:--|
| `patrol-robot:foxy` | 1.49 GB | 生产容器 (ROS2 Foxy + 所有节点) |
| `patrol-robot:humble` | 2.4 GB | SLAM/Nav2 容器 (按需启动) |
| `ros:foxy-ros-base` | 662 MB | 基础 Foxy 镜像 (构建依赖) |
| `ros:humble-ros-base` | 720 MB | 基础 Humble 镜像 |

### 4.2 容器

| 容器 | 镜像 | 网络 | 内存 |
|:--|:--|:--|:--|
| `serial_driver` | `patrol-robot:foxy` | **`--network none`** | 默认 |
| `patrol_car` | `patrol-robot:foxy` | `host` | **2048m** |
| `slam_nav` | `patrol-robot:humble` | `host` | 2048m |

### 4.3 docker-compose.yml (patrol_car + slam_nav)

```yaml
services:
  patrol_car:
    image: patrol-robot:foxy
    container_name: patrol_car
    mem_limit: 2048m
    network_mode: host
    privileged: true
    restart: always
    environment:
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_DOMAIN_ID=42
      - TZ=Asia/Shanghai
      - ROBOT_TYPE=x3
    volumes:
      - /home/pi/patrol_robot:/home/pi/patrol_robot
      - /dev:/dev
      - /tmp:/tmp
    devices:
      - /dev/myserial:/dev/myserial
      - /dev/rplidar:/dev/rplidar
      - /dev/video0:/dev/video0
    working_dir: /home/pi/patrol_robot/patrol_robot
    command: bash /home/pi/patrol_robot/scripts/container_init.sh
```

### 4.4 serial_driver 启动命令

```bash
docker rm -f serial_driver 2>/dev/null
docker run -d --name serial_driver --network none \
    --device /dev/ttyUSB4:/dev/ttyUSB4 \
    -v /home/pi/patrol_robot:/host \
    -v /tmp:/tmp \
    --entrypoint python3 \
    patrol-robot:foxy -u /host/patrol_robot/src/serial_worker.py
```

---

## 5. ROS2 信息

### 5.1 节点列表 (8 个)

| 节点 | 包 | 功能 |
|:--|:--|:--|
| `/cmd_vel_bridge` | patrol_bridge | 订阅 /cmd_vel → 写 FIFO 给串口 |
| `/patrol_web` | patrol_web | Web 面板 + /api/control/move |
| `/patrol_voice` | patrol_voice | 语音识别 → /cmd_vel |
| `/patrol_state_machine` | patrol_manager | 巡逻状态机 → /cmd_vel |
| `/patrol_yolo` | patrol_yolo | YOLOv5 目标检测 |
| `/alert_dispatcher` | patrol_alert | 报警分发 |
| `/simple_camera` | — | 摄像头驱动 (纯 Python) |
| `/ydlidar_driver` | — | 激光雷达驱动 (纯 Python) |

### 5.2 话题列表

| 话题 | 类型 | 发布者 | 订阅者 |
|:--|:--|:--|:--|
| `/cmd_vel` | `Twist` | web, voice, state_machine | **bridge** |
| `/camera/rgb/image_raw` | `Image` | simple_camera | web |
| `/scan` | `LaserScan` | ydlidar_driver | — |
| `/patrol/detections` | `Detection2DArray` | patrol_yolo | web |
| `/patrol/state` | `String` | state_machine | web |
| `/patrol/alert_status` | `String` | alert_dispatcher | web |
| `/patrol/voice_cmd` | `String` | voice | web |
| `/patrol/voltage` | `Float32` | bridge | web |
| `/patrol/state_control` | `String` | web | state_machine |
| `/odom` | `Odometry` | bridge | — |
| `/vel_raw` | `Twist` | web | — (调试用) |
| `/tf` | `TFMessage` | — | — |

### 5.3 QoS 配置

- 所有 `/cmd_vel` 发布者和订阅者: **默认 RELIABLE** (depth=10)
- ROS_DOMAIN_ID=42 (全网隔离)
- DDS: Fast-DDS (`rmw_fastrtps_cpp`)

---

## 6. 架构数据流

```
Web 方向键 / 语音 "前进"
        │
        ▼
┌──────────────────────────────────────┐
│  patrol_web / patrol_voice           │
│  publish Twist(linear.x=0.3)         │
│  → /cmd_vel (RELIABLE, DOMAIN=42)    │
└──────────────┬───────────────────────┘
               │ Fast-DDS (wlan0)
               ▼
┌──────────────────────────────────────┐
│  cmd_vel_bridge                      │
│  _on_cmd() → _last_cmd = (0.3,0,0)  │
│  _heartbeat(20Hz) → 写 FIFO          │
│  {"vx":0.3,"vy":0,"wz":0}            │
└──────────────┬───────────────────────┘
               │ FIFO (/tmp/cmd_fifo)
               ▼
┌──────────────────────────────────────┐  ← --network none (0 DDS!)
│  serial_driver 容器                  │
│  serial_worker.py                    │
│  set_car_motion(0.3, 0, 0)          │
│  → STM32 (FUNC_MOTION=0x12)          │
└──────────────┬───────────────────────┘
               │ /dev/ttyUSB4 (CH340)
               ▼
┌──────────────────────────────────────┐
│  STM32 控制板                        │
│  DEVICE_ID=0xFC, car_type=0x81      │
│  → 4 路电机 PWM                      │
└──────────────────────────────────────┘
```

---

## 7. 启动流程

```bash
# 步骤 1: 启动串口隔离容器 (必须先启)
docker rm -f serial_driver 2>/dev/null
sudo fuser -k /dev/ttyUSB4 2>/dev/null
docker run -d --name serial_driver --network none \
    --device /dev/ttyUSB4:/dev/ttyUSB4 \
    -v /home/pi/patrol_robot:/host \
    -v /tmp:/tmp \
    --entrypoint python3 \
    patrol-robot:foxy -u /host/patrol_robot/src/serial_worker.py

# 步骤 2: 等 FIFO 就绪 + 启动 patrol_car
sleep 3
cd ~/patrol_robot && docker compose up -d patrol_car

# 停止
cd ~/patrol_robot && docker compose down
docker stop serial_driver && docker rm serial_driver
```

---

## 8. 核心驱动库备份与依赖关系

### 8.1 驱动库清单

| 库名 | 版本 | 大小 | MD5 (Pi) | 备份位置 |
|:--|:--|:--|:--|:--|
| **Rosmaster_Lib.py** | V1.5.8 原厂 | 49,803 字节 | `65161b7c...` | Pi: `/home/pi/patrol_robot/Rosmaster_Lib.py`<br>Pi: `patrol_robot/src/Rosmaster_Lib.py`<br>GitHub: 同路径×2<br>Windows: `D:\小车-ros\Rosmaster-X3资料\ROS1\...` |
| **Speech_Lib.py** | — | 1,413 字节 | — | Pi: `patrol_robot/src/Speech_Lib/Speech_Lib.py`<br>GitHub: 同路径 |

### 8.2 依赖关系图

```
serial_worker.py / cmd_vel_bridge.py
  ├── import Rosmaster_Lib.Rosmaster ← /home/pi/patrol_robot/Rosmaster_Lib.py
  │     ├── import serial (pyserial 3.4)
  │     ├── import struct (标准库)
  │     └── import threading (标准库)
  │
  ├── 串口: /dev/myserial → ttyUSB2 (KERNELS="3-1.3*")
  │
voice_node.py
  ├── import Speech_Lib.Speech ← PYTHONPATH=patrol_robot/src/Speech_Lib/
  ├── import rclpy
  └── /dev/myspeech → ttyUSB0 (KERNELS="1-2*")

web_server.py
  ├── from flask import Flask (1.1.1)
  ├── import rclpy
  └── publish /cmd_vel
    
ydlidar_driver.py
  ├── /dev/rplidar → ttyUSB3 (KERNELS="3-1.2*")
  └── 纯 Python, 无 ROS2

simple_camera.py
  └── /dev/video0, 320×240 MJPG
```

### 8.3 备份策略

- **Rosmaster_Lib**：Pi 上 2 份 + GitHub 2 份 + Windows 原厂资料 1 份 = **5 份**
- **Speech_Lib**：Pi 1 份 + GitHub 1 份 = **2 份**
- **恢复方法**：`git clone https://github.com/licd72/patrol-car-pi5` 即可恢复全部

---

## 9. 铁律

| # | 规则 |
|:--|:--|
| 1 | STM32 只能被 serial_driver 容器独占 (`--network none`) |
| 2 | 改代码后必须 `down + up` (restart 不更新 volume 代码) |
| 3 | 改 install 后必须 `sudo find install -name __pycache__ -exec rm -rf {} +` |
| 4 | 禁止 CycloneDDS (ARM+Docker 有 `IP_MULTICAST_IF` bug) |
| 5 | `ROS_DOMAIN_ID=42` 全局一致 |
| 6 | Exit 137 = OOM → 加 `mem_limit` |
| 7 | `Rosmaster_Lib` 用原版 1139 行, 参数 `DEVICE_ID=0xFC` |
| 8 | `/dev/myserial` → `/dev/ttyUSB4` (CH340, Bus003) |
| 9 | 摄像头 `/dev/video0` 在用前 `fuser -k /dev/video0` |
| 10 | 部署到 GitHub 前 `git add -A && git commit && git push` |
