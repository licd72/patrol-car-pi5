# 硬件 BOM 清单（RPi5 版）

| 模块 | 型号 | 用途 | 数量 | 来源 | 单价(¥) |
|:-----|:-----|:-----|:---:|:-----|:------:|
| 主控 | Raspberry Pi 5 8GB | AI 推理 + ROS2 运行 | 1 | 已有 | 0 |
| 底盘 | ROSMASTER X3 | Ackerman 转向平台 | 1 | 已有 | 0 |
| 扩展板 | ERF01 v3.0 (STM32F407) | 电机/传感器 | 1 | 随底盘 | 0 |
| 激光雷达 | YDLIDAR X4 | SLAM + 避障 | 1 | 已有 | 0 |
| 深度相机 | Astra Pro | RGB + 深度 | 1 | 已有 | 0 |
| NVMe SSD | 128GB + M.2 HAT | 系统 + 存储 | 1 | 淘宝 | ~200 |
| 4G 模块 | EC200T | 远程通信 | 1 | 淘宝 | ~150 |
| 蜂鸣器 | 有源 5V | 本地报警 | 1 | 淘宝 | ~5 |
| LED 警示灯 | 12V 红蓝双闪 | 本地报警 | 1 | 淘宝 | ~25 |
| 散热套件 | 官方 Active Cooler | 防止降频 | 1 | 淘宝 | ~40 |

**新增总费用：约 ¥420**

## 硬件接线

```
RPi5 GPIO
├── [5V/GND]    → 蜂鸣器 (PWM 控制, GPIO18)
├── [5V/GND]    → LED 警示灯 (GPIO23 继电器)
├── [UART TX/RX] → ERF01 扩展板串口
├── [USB 3.0]   → Astra Pro 深度相机
├── [USB 2.0]   → YDLIDAR X4
├── [USB 2.0]   → EC200T 4G 模块
└── [PCIe M.2]  → NVMe SSD (M.2 HAT)
```
