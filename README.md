# 树莓派5 巡逻小车 — 完整架构与调试总结

> **日期**: 2026-07-16/17  
> **设备**: Raspberry Pi 5 + STM32 (YB-ERF01) + YDLIDAR X3 Pro + USB 摄像头 + YB-MAE01 语音模块  
> **IP**: 192.168.31.75  
> **GitHub**: https://github.com/licd72/patrol-car-pi5

---

## 🎯 最终架构 (v3 — 简化直驱)

```
┌──────────────────────────────────────────────────────────┐
│ patrol_car (Docker: yahboomtechnology/ros-foxy:4.0.9R2)  │
│                                                          │
│  /dev/video0  → simple_camera → /camera/rgb/image_raw    │
│  /dev/rplidar → ydlidar_driver → /scan                   │
│  /dev/myserial ←→ web_server (Rosmaster_Lib 直驱底盘)    │
│  /dev/myspeech ←→ patrol_voice (Speech_Lib 语音识别)      │
│                                                          │
│  patrol_yolo (YOLOv5n+ONNX) → /patrol/detections         │
│  patrol_state_machine → IDLE/TRACKING/ALERTING            │
│  alert_dispatcher (环形缓冲区抓拍) → snapshots/*.jpg       │
│  patrol_web (Flask :5000) → Web遥控 + MJPEG视频流         │
│                                                          │
│  ★ patrol_voice → HTTP POST → patrol_web → 底盘控制       │
└──────────────────────────────────────────────────────────┘
```

**关键简化**: 
- 去掉 `Mcnamu_driver_X3` 和 `base_node_X3`（ROS2 节点不执行运动指令）
- `web_server` 直接通过 `Rosmaster_Lib.set_car_motion()` 控制 STM32
- `patrol_voice` 通过 HTTP 调用 `web_server` API 控制底盘（避免串口竞争）

---

## 🐛 本次调试修复的所有问题 (8项)

### 1. 🔴 抓拍画面为空
- **根因**: YOLO检测到人→立即发图→alert_dispatcher丢弃→10秒确认后保存→人已走
- **修复**: 环形缓冲区 `deque(maxlen=60)`，始终缓存，报警触发时回写
- **文件**: `alert_dispatcher.py`

### 2. 🔴 状态机 TRACKING 死循环
- **根因**: ① timer堆积(create_timer不cancel) ② Nav2回调在TRACKING中触发_advance_waypoint ③ _change_state_later的5秒timer打断TRACKING
- **修复**: timer引用管理 + 导航回调状态检查 + _change_state_later TRACKING保护
- **文件**: `patrol_state_machine.py`

### 3. 🟡 摄像头被占用
- **根因**: orbbec_ros_foxy容器抢占/dev/video0
- **修复**: fuser -k释放 + _cam_check.py验证
- **文件**: `container_init.sh`

### 4. 🟡 容器反复重启
- **根因**: set -e遇错退出 + yahboomcar_ws路径不存在
- **修复**: 去掉set -e，容错处理
- **文件**: `container_init.sh`

### 5. 🟡 时区错误
- **根因**: 容器默认UTC
- **修复**: TZ=Asia/Shanghai
- **文件**: `docker-compose.yml`, `container_init.sh`, `alert_dispatcher.py`

### 6. 🔴 底盘不响应运动指令
- **根因**: Mcnamu_driver_X3只读传感器不执行运动控制
- **修复**: web_server直接用Rosmaster_Lib.set_car_motion()控制底盘
- **文件**: `web_server.py`, `container_init.sh`

### 7. 🔴 车轮失控持续转动
- **根因**: STM32固件持续执行最后收到的非零指令
- **修复**: watchdog每0.2秒发零速度，0.8秒无新指令自动归零
- **文件**: `web_server.py`

### 8. 🟡 树莓派重启后语音失效 / USB枚举变化
- **根因**: USB设备枚举顺序变了，ydlidar_driver硬编码的/dev/ttyUSB0抢了语音模块串口
- **修复**: ydlidar改用 `/dev/rplidar`（按设备属性绑定的符号链接）
- **文件**: `ydlidar_driver.py`

---

## 📦 核心经验教训

1. **不要假设 ROS2 节点行为** — 节点在线 ≠ 功能正常，最终绕过 driver_node/base_node 直接用 Rosmaster_Lib
2. **timer 管理是状态机头号杀手** — 不 cancel 必定堆积
3. **STM32 固件不会自动停止** — 必须持续发零速度
4. **环境变量是隐藏炸弹** — ROBOT_TYPE 未设导致参数全错
5. **2>/dev/null 是调试天敌** — 屏蔽了无数错误
6. **不要硬编码 /dev/ttyUSBx** — 树莓派重启后枚举顺序会变，用 udev 符号链接 `/dev/rplidar` `/dev/myserial` 等
7. **ROS2 串口节点与直驱不能共存** — web_server 和 voice_node 都需串口时，voice 通过 HTTP 调 web API 避免竞争
