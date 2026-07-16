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

---

## 🐛 修复的所有问题 (8项)

| # | 问题 | 根因 | 修复 |
|:--|:-----|:-----|:-----|
| 1 | 抓拍为空 | 报警确认后才保存，人已走 | 环形缓冲区 dequeue(maxlen=60) |
| 2 | TRACKING 死循环 | timer 堆积 + Nav2 回调打断 | timer引用管理 + 状态保护 |
| 3 | 摄像头占用 | orbbec 容器抢设备 | fuser -k + _cam_check.py |
| 4 | 容器重启 | set -e 遇错退出 | 容错处理 |
| 5 | 时区错误 | 容器 UTC | TZ=Asia/Shanghai |
| 6 | 底盘不响应 | driver_node 不执行运动指令 | web_server Rosmaster_Lib 直驱 |
| 7 | 车轮失控 | STM32 不自动停止 | 0.2s watchdog |
| 8 | 重启后异常 | USB 枚举变化 | **udev 规则改用 ID_PATH** |

---

## ⚠️ 核心教训：设备路径问题 → 先查 udev，不要改程序！

**问题 6 和 8 其实有同一个根因——设备路径不对**。但花了大量时间改代码（换 Rosmaster 直驱、加 HTTP fallback），其实只需要：

```bash
# 检查符号链接
ls -la /dev/rplidar /dev/myserial /dev/myspeech

# 更新 udev 规则
sudo nano /etc/udev/rules.d/99-yahboomcar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

详见 [`docs/debug-rules.md`](docs/debug-rules.md) —— **下次调试前先看这个**。

---

## 📦 udev 设备映射 (v2 — ID_PATH 绑定)

| 符号链接 | 设备 | 绑定方式 |
|:-----|:-----|:-----|
| `/dev/rplidar` | YDLIDAR X3 (CP2102) | ID_SERIAL (唯一) |
| `/dev/myserial` | STM32 扩展板 (CH340) | ID_PATH (物理拓扑) |
| `/dev/myspeech` | 语音模块 (CH340) | ID_PATH |
| `/dev/voice` | GPS/备用 (CH340) | ID_PATH |

重启稳定，前提：**不换 USB 插口**。
