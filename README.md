# 树莓派5 巡逻小车 — v4 稳定版

> **日期**: 2026-07-17  
> **IP**: 192.168.31.75  
> **GitHub**: https://github.com/licd72/patrol-car-pi5  
> **Web 面板**: http://192.168.31.75:5000

---

## 架构

```
树莓派启动
  ├─ @reboot cron(15s) → kill rosmaster → reload uvcvideo
  ├─ Docker(restart=always) → patrol_car 容器
  │   ├─ [0] fuser -k /dev/video0
  │   ├─ [1] simple_camera   (V4L2+MJPG, 10次重试)
  │   ├─ [2] ydlidar_driver  (/dev/rplidar)
  │   ├─ [3] patrol_yolo     (YOLOv5n+ONNX)
  │   ├─ [4] patrol_state_machine (IDLE纯检测, Nav2可用自动切换)
  │   ├─ [5] alert_dispatcher (deque环形缓冲区抓拍)
  │   ├─ [6] patrol_voice    (HTTP→web_server)
  │   └─ [7] patrol_web      (Flask :5000 + Rosmaster_Lib直驱)
  └─ /dev/myserial ← Rosmaster_Lib ← web_server (底盘控制)
     /dev/myspeech ← Speech_Lib    ← patrol_voice (语音识别)
```

## 本次全部修复 (9项)

| # | 问题 | 修复 | 文件 |
|:--|:-----|:-----|:-----|
| 1 | 抓拍为空(时机滞后) | deque环形缓冲区(60帧) | alert_dispatcher.py |
| 2 | TRACKING死循环 | timer管理+状态保护+IDLE触发 | patrol_state_machine.py |
| 3 | 摄像头obsensor抢占 | V4L2+MJPG + 10次重试 | simple_camera.py |
| 4 | 底盘不响应 | Rosmaster_Lib直驱(去Mcnamu/base_node) | web_server.py + container_init.sh |
| 5 | 车轮失控 | 0.2s watchdog自动归零 | web_server.py |
| 6 | 语音失效 | HTTP→web_server API | voice_node.py |
| 7 | USB枚举变化 | udev ID_PATH绑定 | 99-yahboomcar.rules |
| 8 | rosmaster抢占设备 | 停用systemd服务 | — |
| 9 | Nav2不可用时日志轰炸 | IDLE降级(不循环) | patrol_state_machine.py |

## 冷启动可靠性

| 层级 | 保障 |
|:-----|:-----|
| 宿主机 | @reboot cron: rosmaster已禁用, uvcvideo重载 |
| Docker | restart: always |
| 容器 | fuser -k /dev/video0 |
| 相机 | V4L2+MJPG + 10次重试(最多等20秒) |
| 设备 | udev ID_PATH绑定(/dev/rplidar等) |

## 清理清单

- ❌ rosmaster.service (systemd)
- ❌ orbbec_ros_foxy 容器
- ❌ Mcnamu_driver_X3 + base_node_X3
- ❌ yahboomcar_ws 依赖
- ❌ 8个 start_*.sh → ~/archived_scripts/
- ❌ _cam_check.py (与相机竞争)
