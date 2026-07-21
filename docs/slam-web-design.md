# SLAM 建图 Web 控制面板 — 架构设计

> 版本: 1.0 | 日期: 2026-07-18 | 原则: 用户全程在浏览器操作，不碰终端

---

## 1. 设计目标

新建一个独立的 Web 界面（端口 **5001**），用于 SLAM 建图。与现有的巡逻监控面板（`:5000`）完全独立。

**核心要求:**
- 实时显示地图构建过程
- 手动方向键控制小车
- 一键保存地图
- 建图时自动释放巡逻节点内存
- 建图完成后一键恢复巡逻系统

---

## 2. 整体架构

```
┌──────────────────────────────────────────────────────────┐
│  patrol_car 容器 (Foxy, 常驻运行)                         │
│                                                          │
│  slam_web :5001  ← 新建，常驻                            │
│  ├── 手动控制 ──→ /cmd_vel ──→ cmd_vel_bridge ──→ STM32  │
│  ├── 激光显示 ←── /scan ←──── ydlidar_driver             │
│  ├── 地图显示 ←── /map  ←──── (跨容器 DDS)               │
│  ├── POST /api/slam/start                                │
│  │   ├── ① _patrol_stop()    释放 ~660MB                 │
│  │   └── ② docker compose up slam_nav                    │
│  ├── POST /api/slam/stop                                 │
│  │   ├── ① docker compose stop slam_nav                  │
│  │   └── ② _patrol_start()   恢复巡逻节点                │
│  └── POST /api/save_map                                  │
│       └── ros2 run map_saver_cli                         │
│                                                          │
│  保留的核心节点: bridge(57MB) + lidar(55MB) + camera*    │
│  建图时停掉: YOLO(238MB) + web(155) + alert(141)         │
│             + voice(65) + state(65) = 释放 ~660MB        │
└──────────────────────────────────────────────────────────┘
                         │
         跨容器 DDS (host 网络, ROS_DOMAIN_ID=42)
                         │
┌──────────────────────────────────────────────────────────┐
│  slam_nav 容器 (Humble, 按需启动)                         │
│  └── slam_toolbox ──→ /map                               │
│       online_async 模式, 参数见 slam_toolbox_params.yaml  │
└──────────────────────────────────────────────────────────┘
```

## 3. 用户操作流程

```
手机/电脑 浏览器
     │
     ▼
http://192.168.31.75:5001

┌──────────────────────────────────────┐
│  🗺️ SLAM 建图            ⏳ 未启动  │
│                                      │
│  [🔴 开始建图]  [💾 保存]  [⬛ 停止] │
│                                      │
│       ┌──────────────────┐           │
│       │                  │           │
│       │  实时地图 Canvas  │           │
│       │                  │           │
│       └──────────────────┘           │
│                                      │
│           ▲                          │
│       ◄   ●   ►     方向键           │
│           ▼                          │
└──────────────────────────────────────┘

操作步骤:
  ① 点击 [开始建图]
     → 自动停掉巡逻节点(释放内存)
     → 启动 Humble 容器 + slam_toolbox
     → 等待 5 秒，地图开始实时显示

  ② 方向键控制小车走一圈
     → 支持移动端触摸长按
     → 地图 Canvas 每秒自动刷新

  ③ 点击 [保存地图]
     → 输入名称 (可选)
     → 调用 map_saver_cli
     → 生成 ~/maps/<name>.pgm + .yaml

  ④ 点击 [停止建图]
     → 关闭 Humble 容器
     → 自动重启巡逻节点
     → 系统恢复巡逻模式
```

## 4. 节点启停管理

### 建图时停掉的节点（释放内存）

| 节点 | 进程匹配 | 内存 | 重启命令 |
|:--|:--|:--|:--|
| patrol_yolo | `yolo_detector` | ~238MB | `ros2 run patrol_yolo yolo_detector ...` |
| patrol_web | `web_server` | ~155MB | `ros2 run patrol_web web_server` |
| alert_dispatcher | `alert_dispatcher` | ~141MB | `ros2 run patrol_alert alert_dispatcher` |
| patrol_voice | `voice_node` | ~65MB | `ros2 run patrol_voice voice_node ...` |
| patrol_state | `patrol_state_machine` | ~65MB | `ros2 run patrol_manager patrol_state_machine` |

**总释放: ~660MB**（对 8GB Pi5 足够运行 SLAM）

### 始终保留的节点

| 节点 | 作用 | 内存 |
|:--|:--|:--|
| cmd_vel_bridge | 底盘控制 | ~57MB |
| ydlidar_driver | 激光雷达 /scan | ~55MB |
| simple_camera | 摄像头 | ~133MB |

## 5. 文件结构

```
patrol-car-pi5/
├── src/slam_web/                         ← 新建
│   ├── setup.py                          # ROS2 包配置
│   ├── resource/slam_web                 # ament 索引
│   ├── slam_web/
│   │   ├── __init__.py
│   │   └── web_server.py                 # Flask + ROS2 核心
│   └── templates/
│       └── slam.html                     # 前端控制面板
│
├── scripts/
│   ├── container_init.sh                 ← 修改: 增加 slam_web 启动
│   └── slam_init.sh                      ← 新建: Humble SLAM 启动
│
├── config/
│   └── slam_toolbox_params.yaml          ← 新建: SLAM 参数
│
└── docker-compose.yml                    ← 修改: slam_nav 容器
```

## 6. API 设计

| 方法 | 端点 | 功能 | 返回 |
|:--|:--|:--|:--|
| GET | `/` | 建图控制面板 HTML | HTML |
| GET | `/api/map` | 地图 PNG (base64) + 元信息 | JSON `{png, info, status}` |
| GET | `/api/scan` | 最近激光数据 | JSON `{ranges: [...]}` |
| POST | `/api/control` | 手动方向控制 | JSON `{ok, direction}` |
| POST | `/api/slam/start` | 启动建图 (停 patrol + 启 Humble) | JSON `{ok, status}` |
| POST | `/api/slam/stop` | 停止建图 (停 Humble + 启 patrol) | JSON `{ok}` |
| POST | `/api/save_map` | 保存地图 | JSON `{ok, name, path}` |

### 控制请求格式

```json
POST /api/control
{
  "direction": "forward|backward|left|right|stop",
  "duration": 0.3,    // 秒 (最大2.0)
  "speed": 0.2         // m/s
}
```

### 地图保存请求格式

```json
POST /api/save_map
{
  "name": "office_1f"  // 可选, 默认 map_YYYYMMDD_HHMMSS
}
```

## 7. 跨容器 DDS 通信

```
Foxy (patrol_car)              Humble (slam_nav)
═════════════════              ═════════════════
network_mode: host             network_mode: host
ROS_DOMAIN_ID=42               ROS_DOMAIN_ID=42
rmw_fastrtps_cpp               rmw_fastrtps_cpp
     │                              │
     ├── /scan ──────────────────→ slam_toolbox (订阅)
     │                              │
     │         slam_toolbox ───→ /map (发布)
     │                              │
slam_web ←── /map ─────────────────┘
```

**备选方案:** 如果 Fast-DDS 跨容器不通，两个容器都切换为 `rmw_cyclonedds_cpp`。

## 8. 前端页面结构

```html
slam.html
├── #header        → 标题 + 状态指示
├── #map-area      → Canvas (实时地图, 自适应缩放)
├── #info          → 地图尺寸 + 分辨率
├── #controls      → 5键方向盘 (▲◄●►▼) 支持触摸长按
└── #save-bar      → 名称输入 + 保存按钮
```

**刷新策略:** 地图每秒拉取一次 `/api/map`，避免频繁请求。

## 9. SLAM 参数要点

```yaml
slam_toolbox:
  mode: mapping              # 在线异步建图
  map_update_interval: 3.0   # 3秒更新一次地图
  resolution: 0.05           # 5cm/像素
  max_laser_range: 8.0       # 激光最大距离
  transform_timeout: 0.2     # TF 超时
```

完整参数见 `config/slam_toolbox_params.yaml`。

## 10. 部署清单

| # | 步骤 | 命令 |
|:--|:--|:--|
| 1 | 装 pillow | `docker exec patrol_car pip3 install pillow` |
| 2 | 同步代码到 Pi | `scp -r src/slam_web scripts/slam_init.sh config/ pi@192.168.31.75:/home/pi/patrol_robot/` |
| 3 | 编译 slam_web | `docker exec patrol_car bash -c 'source /opt/ros/foxy/setup.bash && cd ... && colcon build --packages-select slam_web --merge-install'` |
| 4 | 修改启动脚本 | `container_init.sh` 末尾加 `ros2 run slam_web web_server` |
| 5 | 重启容器 | `docker compose down && docker compose up -d` |
| 6 | 验证 | 浏览器打开 `http://192.168.31.75:5001` |

## 11. 风险与缓解

| 风险 | 缓解 |
|:--|:--|
| 跨容器 DDS 不通 (/map 收不到) | 两容器同 host 网络 + DOMAIN_ID=42；备选 CycloneDDS |
| Humble 镜像缺少 slam_toolbox | 已有 `patrol-robot:humble` 镜像含完整 ROS2 Humble |
| /cmd_vel 跨容器被 bridge 接收 | slam_web 在 Foxy 容器中，/cmd_vel 本地发布，不走跨容器 |
| Pi5 内存不足 | 建图前释放 ~660MB，剩余 ~5.4GB 空闲足够 |
| 地图保存失败 | map_saver_cli 是 nav2 自带工具，Humble 镜像已包含 |
