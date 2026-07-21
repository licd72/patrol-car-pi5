# SLAM 建图 Web 控制面板 — 完整技术文档

> 版本: 1.0 | 日期: 2026-07-18 | 项目: patrol-car-pi5

---

## 一、硬件环境

| 组件 | 型号 | 接口 | 设备路径 | 备注 |
|:--|:--|:--|:--|:--|
| 主机 | 树莓派 5 (8GB) | — | IP: 192.168.31.75 | Raspberry Pi OS 64-bit |
| 底盘 | 亚博 Rosmaster X3 | 串口 | `/dev/myserial` → ttyUSB4 | 麦克纳姆轮, STM32 控制板 |
| 激光雷达 | YDLIDAR X4 | 串口 | `/dev/rplidar` → ttyUSB1 | 10Hz, 8m 测距 |
| 摄像头 | USB 摄像头 | USB | `/dev/video0` | 320×240 MJPG, 15fps |
| 语音模块 | YB-MAE01 (CI1302) | 串口 | `/dev/myspeech` → ttyUSB0 | 离线语音识别 |

### USB 拓扑

```
Pi5 USB 端口:
├── USB 2.0 (上左): CH340 → /dev/myspeech (语音)
├── USB 3.0 (上右): USB 存储 128GB
├── USB 2.0 (下左): VIA Hub
│   ├── CP2102 → /dev/rplidar (激光)
│   ├── CH340  → /dev/myserial (STM32)
│   ├── Genesys Hub → USB 音频 + 摄像头(/dev/video0)
│   └── Billboard
└── USB 3.0 (下右): VIA USB 3.0 Hub (空)
```

---

## 二、Docker 环境

### 容器架构

```
┌─────────────────────────────────────────────┐
│ patrol_car (Foxy, 常驻, restart: always)     │
│ ├── cmd_vel_bridge  → FIFO                   │
│ ├── ydlidar_driver  → /scan                  │
│ ├── simple_camera   → /camera/rgb/image_raw  │
│ ├── patrol_yolo, state_machine, alert, voice │
│ ├── patrol_web      → :5000 (巡逻面板)       │
│ └── slam_web        → :5001 (建图面板) ← 新增│
│     └── Rosmaster_Lib 直驱 STM32 (不经过FIFO)│
└─────────────────────────────────────────────┘
         │ host network, ROS_DOMAIN_ID=42
         ▼
┌─────────────────────────────────────────────┐
│ slam_nav (Humble, 按需启动)                  │
│ ├── robot_state_publisher (全链 TF)          │
│ └── slam_toolbox → /map                      │
└─────────────────────────────────────────────┘
```

### 关键配置

```yaml
# docker-compose.yml
patrol_car:
  image: patrol-robot:foxy
  network_mode: host
  privileged: true
  restart: always
  environment:
    - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    - ROS_DOMAIN_ID=42
  volumes:
    - /var/run/docker.sock:/var/run/docker.sock  # slam_web 通过此操作宿主机 Docker
    - /home/pi/patrol_robot:/home/pi/patrol_robot
    - /dev:/dev

slam_nav:
  image: patrol-nav2:humble
  network_mode: host
  restart: "no"     # 手动启停
  profiles: [nav]   # 不随 compose up 自动启动
  environment:
    - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    - ROS_DOMAIN_ID=42
```

### 容器内额外安装

```bash
# patrol_car 容器内 (down+up 后需重装):
apt-get install -y docker.io        # slam_web 操作宿主机 Docker
pip3 install cv-bridge              # (后来发现 ARM64 不兼容, 改用 numpy)
```

---

## 三、软件架构

### 文件结构

```
patrol-car-pi5/
├── src/slam_web/                    ← 新建, 完全独立
│   ├── package.xml
│   ├── setup.py
│   ├── resource/slam_web
│   ├── slam_web/
│   │   ├── __init__.py
│   │   └── web_server.py           ← 核心: Flask + ROS2 + Rosmaster 直驱
│   └── templates/
│       └── slam.html               ← 前端控制面板
├── scripts/
│   ├── check_slam.sh               ← 系统重启后一键启动
│   ├── slam_nav_init.sh            ← Humble 容器内 SLAM 启动脚本
│   └── odom_tf.py                  ← 已废弃 (TF 链移至 slam_nav 内部)
├── config/
│   └── slam_params.yaml            ← slam_toolbox 参数
└── docker-compose.yml              ← 仅加了一行 docker.sock 挂载
```

### 核心架构：slam_web → Rosmaster 直驱

```
浏览器 ← HTTP → slam_web (Flask :5001)
                   │
                   ├── Rosmaster_Lib.set_car_motion() → /dev/myserial → STM32
                   ├── 订阅 /camera/rgb/image_raw → JPEG base64 → 前端
                   ├── 订阅 /map (跨容器) → PNG base64 → 前端 Canvas
                   ├── 订阅 /scan, /odom, /tf
                   └── Docker API → 启停 slam_nav 容器
```

**关键设计决策：不用 FIFO，不用 serial_worker，不用 ROS2 DDS 做控制。**

原因：
- FIFO 需要 serial_worker 进程读+写串口，多进程抢 `/dev/myserial` 导致串口冲突
- ROS2 DDS (Fast-DDS) 在容器内长期运行后退化，`/cmd_vel` 发布者存在但订阅者收不到
- Rosmaster_Lib 直驱最简单可靠

### API 接口

| 方法 | 端点 | 功能 |
|:--|:--|:--|
| GET | `/` | 建图控制面板 HTML |
| GET | `/api/status` | 完整状态 JSON |
| GET | `/api/map` | 地图 PNG (base64) |
| GET | `/api/camera` | 摄像头单帧 JPEG (base64) |
| POST | `/api/control` | 方向控制 `{direction, speed, duration}` |
| POST | `/api/slam/start` | 启动建图 (停巡逻 → 启 Humble) |
| POST | `/api/slam/stop` | 停止建图 |
| POST | `/api/save_map` | 保存地图到 `~/maps/` |
| POST | `/api/patrol/restore` | 恢复巡逻面板 (:5000) |
| GET | `/video_feed` | MJPEG 流 (备用, 前端用 `/api/camera` 轮询) |

### 前端技术方案

- 纯 HTML/CSS/JS，无外部依赖
- 暗色主题，响应式布局
- 摄像头：JS `setInterval(200ms)` 轮询 `/api/camera` 获取 base64 JPEG
- 地图：`setInterval(1500ms)` 轮询 `/api/map` 获取 base64 PNG
- 方向键：`pointerdown/pointerup` 长按持续发送 + 键盘 WASD/QE
- 速度滑块可调 0.05~0.5 m/s

---

## 四、遇到的问题及解决

### 问题 1：控制按钮车不动

**根因**: Flask werkzeug 线程调用 `rclpy.Publisher.publish()` 被静默丢弃。

**解决**: 使用线程安全队列 `queue.Queue` + ROS2 Timer。
```python
# Flask 线程 → push_command() → Queue
# ROS2 Timer(20Hz) → _cmd_tick() → 从 Queue 取 → set_car_motion()
```

### 问题 2：DDS 退化，/cmd_vel 发布者存在但桥收不到

**根因**: Fast-DDS 在 ARM64 Docker 内长期运行后参与者失联。`recv=0` 但 `Publisher count=1`。

**解决**: 放弃 ROS2 /cmd_vel，改用 Rosmaster_Lib 直驱 STM32。
```
slam_web → bot.set_car_motion(vx, vy, wz) → /dev/myserial → STM32
```

### 问题 3：串口冲突，车不动

**根因**: container_init.sh 启动的 `cmd_vel_bridge` 和 `serial_worker` 与 slam_web 的 Rosmaster 共享 `/dev/myserial`，导致 `read failed: multiple access on port`。

**解决**: check_slam.sh 启动前先 `pkill -9 cmd_vel_bridge` 和 `pkill -9 serial_worker`。

### 问题 4：SLAM 丢帧 (queue is full)

**根因**: slam_toolbox 的 MessageFilter 无法解析 `odom→laser_frame` 的 TF 链。
- patrol_car 的 odom_tf 发 `odom→base_footprint` 到 `/tf`
- slam_nav 的 robot_state_publisher 发 `base_footprint→laser_frame` 到 `/tf_static`
- 跨容器 TF 时序不匹配导致查找失败

**解决**: 把完整 TF 链（含 `odom` link）全部放入 slam_nav 的 URDF，由 robot_state_publisher 发布为 `/tf_static`。
```xml
<link name="odom"/>
<joint name="odom_joint" type="fixed">
  <parent link="odom"/><child link="base_footprint"/>
</joint>
```

### 问题 5：摄像头画面不显示

**根因 1**: `cv_bridge` 在 ARM64 上不兼容 (`SystemError: initialization of cv_bridge_boost`)。

**解决**: 用 `np.frombuffer(msg.data)` 手动 BGR8→JPEG 转换。

**根因 2**: HTML 模板引号被 patch 工具转义成 `\"`。

**解决**: Python 脚本 `replace(chr(92)+chr(34), chr(34))` 修复。

**根因 3**: `<img src="/video_feed">` MJPEG 流在部分浏览器不稳定。

**解决**: 改用 JS `setInterval(200ms)` 轮询 `/api/camera` 获取 base64 JPEG。

### 问题 6：地图不更新

**根因**: 用户没有点「开始建图」；或 SLAM 启动后 patrol_car 的 odom_tf 干扰了 slam_nav 的 TF。

**解决**: 
- 停止 patrol_car 中的 odom_tf 进程
- slam_nav 内部自带完整 TF 链
- 用户需点击「开始建图」才能看到地图

### 问题 7：停止 SLAM 后巡逻系统堆积进程

**根因**: 早期 `stop_slam()` 自动调用 `container_init.sh` 重启巡逻，每次产生新的 bridge/camera/lidar 副本。

**解决**: 移除自动重启。改为独立的「🔄 恢复巡逻」按钮，用户手动触发。

### 问题 8：.pyc 缓存导致代码更新不生效

**根因**: Python 加载 `__pycache__/*.pyc` 优先于 `.py` 源文件。

**解决**: 每次部署后强制清除：
```bash
find install -path '*slam_web*' \( -name '*.pyc' -o -name __pycache__ \) -exec rm -rf {} +
```

### 问题 9：容器重启后 slam_web 不自动启动

**根因**: slam_web 是手动启动的后台进程，容器重启后消失。

**解决**: 提供 `check_slam.sh` 一键脚本，重启后运行即可。

---

## 五、操作手册

### 树莓派重启后

```bash
ssh pi@192.168.31.75    # 密码 yahboom
bash /home/pi/patrol_robot/scripts/check_slam.sh
```

### 建图流程

1. 浏览器打开 `http://192.168.31.75:5001`
2. 点 **「▶ 开始建图」**（自动停巡逻节点释放内存）
3. 用 **W/A/S/D** 或屏幕方向键控车走一圈
4. 地图区域实时绘制
5. 点 **「💾 保存地图」**（可选输入名称）
6. 点 **「⬛ 停止」** 结束建图
7. 需要恢复巡逻时点 **「🔄 恢复巡逻」**

### 代码部署

```bash
# 本地修改后:
cd patrol-car-pi5
tar czf /tmp/deploy.tar.gz src/slam_web/
scp /tmp/deploy.tar.gz pi@192.168.31.75:/tmp/

# Pi 上:
cd /home/pi/patrol_robot/patrol_robot && tar xzf /tmp/deploy.tar.gz
docker exec patrol_car bash -c "
  cp src/slam_web/slam_web/web_server.py install/lib/python3.8/site-packages/slam_web/
  cp -r src/slam_web/templates/* install/lib/python3.8/site-packages/slam_web/templates/
  find install -path '*slam_web*' \( -name '*.pyc' -o -name __pycache__ \) -exec rm -rf {} +
  pkill -9 -f slam_web
"
sleep 2
docker exec -d patrol_car bash -c "
  export PYTHONPATH=/home/pi/patrol_robot/patrol_robot/install/lib/python3.8/site-packages:\$PYTHONPATH
  source /opt/ros/foxy/setup.bash
  python3 -m slam_web.web_server > /tmp/slam_web.log 2>&1
"
```

---

## 六、关键命令速查

```bash
# 启动 slam_web
bash /home/pi/patrol_robot/scripts/check_slam.sh

# 查看 slam_web 状态
curl http://192.168.31.75:5001/api/status

# 诊断
python3 /tmp/diag.py

# 看日志
docker exec patrol_car tail -f /tmp/slam_web.log

# 查看 SLAM 容器
docker logs slam_nav

# 强制停止 SLAM
docker rm -f slam_nav
curl -X POST http://192.168.31.75:5001/api/slam/stop

# 直接测试底盘
docker exec patrol_car python3 /tmp/test_motor.py
```
