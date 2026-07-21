# SLAM 建图程序 — 当前进度总结

> 日期: 2026-07-19 | 项目: patrol-car-pi5

---

## 一、整体架构（最终定案）

```
patrol_car 容器 (Foxy)
├── slam_web (:5001)    Flask + ROS2 → 摄像头/控制/地图 API
├── slam_aio_v3         雷达读取 + 栅格建图 + /map 发布 + map_data.json 写入
└── Rosmaster_Lib       底盘直驱 (/dev/myserial, 115200)
```

**核心原则**：所有组件必须在同一进程/容器内。ARM64 Docker 上跨进程 DDS 不可靠——同进程 pub+sub 全部 10/10 通过，跨进程 0/0。

---

## 二、硬件参数（YDLIDAR X4）

| 参数 | 值 | 发现过程 |
|:--|:--|:--|
| 串口芯片 | CP2102 (10c4:ea60) | udev 规则 |
| 符号链接 | `/dev/rplidar` | udev → ttyUSB1 |
| 波特率 | **128000** | 试了 230400/512000/128000，仅 128000 有 5A A5 响应头 |
| 协议帧头 | `5A A5`（非 AA 55）| 开发手册写 A55A，字节序为 5A A5 |
| 帧头格式 | 非标准 AA55 数据包 | 实际数据用 2-byte raw pairs，非手册描述的 FSA/LSA/CS 格式 |
| 驱动 SDK | YDLidar-SDK v1.0.6 | 已编译安装但无法初始化——SDK 版本与固件不兼容 |

---

## 三、✅ 已完成

| 功能 | 状态 | 说明 |
|:--|:--|:--|
| 底盘控制 | ✅ | Rosmaster_Lib 直驱，杀 bridge 后独占串口 |
| 摄像头 | ✅ | JS 轮询 `/api/camera_jpeg`，200ms/帧 |
| Web 面板 (:5001) | ✅ | 控制方向键 + 速度滑块 + 状态轮询 |
| 激光雷达数据读取 | ✅ | 128000 波特率，raw 2-byte pairs |
| `/scan` 发布 | ✅ | 同进程 pub（20+ 帧/秒） |
| 栅格建图 | ✅ | 400×400 格网 (20m×20m)，射线投影 |
| `/map` 发布 | ✅ | OccupancyGrid，同进程 pub |
| map_data.json 写入 | ✅ | 格式匹配 slam_web 的 `/api/map` |
| 地图 API | ✅ | 返回 base64 PNG，MD5 每次不同（地图持续更新） |
| 速度共享 | ✅ | slap_web → /tmp/vel.json → slam_aio 读取，更新位姿 |
| rviz2 | ✅ | VNC 模式可用 (`-platform vnc`, port 5900) |

---

## 四、❌ 待修复

| 问题 | 症状 | 根因 | 优先级 |
|:--|:--|:--|:--|
| **距离单位不准** | 地图显示黑圈（障碍物全在近处）| raw 2-byte 值除以 400 只是估算，实际距离单位未知 | 🔴 P0 |
| 位姿跟踪不准 | 地图随车移动不明显 | 速度来自控制命令（非编码器），累积误差大 | 🟡 P1 |
| 地图保存 | 未实现 | Web 面板无保存按钮对接 | 🟡 P2 |
| 恢复 ydlidar 驱动 | 重启后需手动禁自带驱动才能启动 slam_aio | container_init.sh 的 ydlidar_driver.py 和 slam_aio 抢串口 | 🟢 P3 |
| slam_web 丢失后需重传 | 容器重启清空 /tmp | slam_aio_v3.py 不在镜像内 | 🟢 P3 |

---

## 五、距离校准方案（P0）

当前 `val / 400.0` 只是猜测。正确的校准方法：

### 方案 A：rviz2 对照法
1. Pi 上启动 rviz2 VNC：`docker exec -e DISPLAY=:0 patrol_car bash -c 'source /opt/ros/foxy/setup.bash && rviz2 -platform vnc'`
2. VNC 连 192.168.31.75:5900，添加 LaserScan(`/scan`)
3. 在车前方 1m 处放障碍物，看 rviz2 中 /scan 显示的距离
4. 调整 `slam_aio_v3.py` 中的除数，使 rviz2 显示 1m 对应实际 1m

### 方案 B：实物测距法
1. 车前 1m 处放纸箱
2. 查看 grid 中障碍物离中心多少格（每格 5cm）
3. 调整除数直到 20 格 = 1m

### 方案 C：从 5A A5 响应头正确解析
1. 研究 5A A5 响应头后的 mode=1 连续数据
2. 找到 AA 55 数据包的实际字节位置
3. 按手册协议正确解析 FSA/LSA/CS/距离

---

## 六、重启后操作流程

```bash
# 1. SSH
ssh pi@192.168.31.75

# 2. 禁自带雷达驱动（避免串口冲突）
docker exec patrol_car mv /home/pi/patrol_robot/patrol_robot/src/ydlidar_driver.py \
  /home/pi/patrol_robot/patrol_robot/src/ydlidar_driver.py.bak
docker exec patrol_car pkill -9 -f ydlidar

# 3. 重传 slam_aio（/tmp 被清空）
scp slam_aio_v3.py pi@192.168.31.75:/tmp/

# 4. 杀桥 + 启 slam_web
docker exec patrol_car pkill -9 -f cmd_vel_bridge
docker exec -d patrol_car bash -c "source /opt/ros/foxy/setup.bash; \
  PYTHONPATH=... python3 -m slam_web.web_server > /tmp/slam_web.log 2>&1"

# 5. 启 slam_aio
docker exec -d patrol_car bash -c "source /opt/ros/foxy/setup.bash; \
  python3 /tmp/slam_aio_v3.py > /tmp/slam_aio.log 2>&1"

# 6. 浏览器
http://192.168.31.75:5001
```

---

## 七、文件清单

| 文件 | 位置 | 用途 |
|:--|:--|:--|
| `slam_aio_v3.py` | `patrol-car-pi5/scripts/` | 雷达+建图一体化 |
| `web_server.py` | `patrol-car-pi5/src/slam_web/` | Web 面板后端 |
| `slam_params.yaml` | `patrol-car-pi5/config/` | slam_toolbox 参数（参考） |
| `x4_driver.py` | `patrol-car-pi5/scripts/` | X4 纯驱动（测试用） |
| `test_scan.py` | `patrol-car-pi5/scripts/` | /scan 接收验证 |
| `test_pub_sub.py` | `patrol-car-pi5/scripts/` | 同进程 pub+sub 验证 |
| `diag.py` | `patrol-car-pi5/scripts/` | 诊断脚本 |

---

## 八、关键教训

1. **不要用双容器架构** — ARM64 Docker 跨容器 DDS 不通
2. **同进程 pub+sub 100% 可靠** — 跨进程 0%
3. **X4 协议和手册不一致** — 128000 baud，5A A5 头，raw pairs 格式
4. **YDLidar-SDK v1.0.6 不兼容此固件** — 虽然编译成功但无法初始化
5. **重启后 /tmp 清空** — slam_aio 需要固化到镜像或持久化路径
