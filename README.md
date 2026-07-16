# 树莓派5 巡逻小车 — 完整架构与调试总结

> **日期**: 2026-07-16/17  
> **设备**: Raspberry Pi 5 + STM32 (YB-ERF01) + YDLIDAR X3 Pro + USB 摄像头  
> **IP**: 192.168.31.75  
> **GitHub**: https://github.com/licd72/patrol-car-pi5

---

## 🎯 最终架构 (v3 — 简化直驱)

```
┌──────────────────────────────────────────────────────────┐
│ patrol_car (Docker: yahboomtechnology/ros-foxy:4.0.9R2)  │
│                                                          │
│  /dev/video0 → simple_camera → /camera/rgb/image_raw     │
│  /dev/ttyUSB0 → ydlidar_driver → /scan                   │
│  /dev/myserial ←→ web_server (Rosmaster_Lib 直驱底盘)     │
│                                                          │
│  patrol_yolo (YOLOv5n+ONNX) → /patrol/detections         │
│  patrol_state_machine → IDLE/NAVIGATING/TRACKING/ALERTING│
│  alert_dispatcher (环形缓冲区抓拍) → snapshots/*.jpg       │
│  patrol_web (Flask :5000) → Web遥控 + MJPEG视频流         │
└──────────────────────────────────────────────────────────┘
```

**关键简化**: 去掉了 `Mcnamu_driver_X3` 和 `base_node_X3`——这两个 ROS2 节点不执行运动指令。改为 `web_server` 直接通过 `Rosmaster_Lib.set_car_motion()` 控制 STM32 底盘。

---

## 🐛 本次调试修复的所有问题

### 1. 🔴 抓拍画面为空 (P0)
- **根因**: YOLO检测到人→立即发图→alert_dispatcher丢弃→10秒确认后保存→人已走
- **修复**: 环形缓冲区 `deque(maxlen=60)`，始终缓存，报警触发时回写
- **文件**: `alert_dispatcher.py`

### 2. 🔴 状态机 TRACKING 死循环 (P0)
- **根因**: ① timer 堆积(create_timer不取消旧timer) ② Nav2回调在TRACKING中触发_advance_waypoint ③ _change_state_later的5秒延迟timer打断TRACKING
- **修复**: 三处加固—timer引用管理+导航回调状态检查+_change_state_later TRACKING保护
- **文件**: `patrol_state_machine.py`

### 3. 🟡 摄像头被占用 (P1)
- **根因**: orbbec_ros_foxy容器抢占/dev/video0
- **修复**: fuser -k释放 + _cam_check.py验证
- **文件**: `container_init.sh`

### 4. 🟡 容器反复重启 (P1)
- **根因**: set -e遇错退出 + yahboomcar_ws路径不存在
- **修复**: 去掉set -e，容错处理
- **文件**: `container_init.sh`

### 5. 🟡 时区错误 (P1)
- **根因**: 容器默认UTC
- **修复**: TZ=Asia/Shanghai (docker-compose + container_init + 代码)
- **文件**: `docker-compose.yml`, `container_init.sh`, `alert_dispatcher.py`

### 6. 🔴 底盘不响应运动指令 (P0)
- **根因**: Mcnamu_driver_X3只读传感器不执行运动控制；base_node_X3订阅vel_raw但没有正确接收
- **修复**: web_server直接用Rosmaster_Lib.set_car_motion()控制底盘，移除无用的driver_node/base_node
- **文件**: `web_server.py`, `container_init.sh`

### 7. 🔴 车轮失控持续转动 (P0)
- **根因**: STM32固件会持续执行最后收到的非零指令，不会自动停止
- **修复**: watchdog每0.2秒发零速度，超过0.8秒无新指令自动归零
- **文件**: `web_server.py`

### 8. 🟡 环境变量缺失 (P1)
- **根因**: ROBOT_TYPE=x3未设置，默认r2导致参数不匹配
- **修复**: docker-compose + container_init 设置环境变量
- **文件**: `docker-compose.yml`, `container_init.sh`

---

## 📦 核心经验教训

1. **不要假设 ROS2 节点行为** — Mcnamu_driver_X3/basic_node_X3 看起来在线但不执行运动指令，最终绕过它们直接用 Rosmaster_Lib 解决问题
2. **timer 管理是 ROS2 状态机的头号杀手** — 不做 cancel 必定堆积，导致状态混乱
3. **STM32 固件不会自动停止** — 必须持续发送零速度指令
4. **环境变量是隐藏炸弹** — ROBOT_TYPE 未设导致所有参数用错
5. **2>/dev/null 是调试的天敌** — 屏蔽了无数错误信息
