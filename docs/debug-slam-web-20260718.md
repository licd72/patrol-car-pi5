# SLAM 建图问题诊断 — 2026-07-18

**症状**：摄像头图像不显示 + 实时地图不更新
**诊断者**：Hermes Agent + 用户协同
**耗时**：~30 分钟

---

## 一、症状表现

用户访问 `http://192.168.31.75:5001`：
- 地图区显示"点击开始建图"灰色占位
- 摄像头区显示黑色 camera 裂图框
- 点击「开始建图」后 slam_nav 容器启动，但前端仍然无图无相机

## 二、根因（两层叠加）

### 🔴 根因 A：YDLIDAR X4 雷达电机停转（已修复）

**证据**：
```
docker exec patrol_car cat /dev/rplidar  # 2 秒 0 字节
ros2 topic info /scan                    # Publisher count: 0
```

**根因**：雷达内部电机/扫描头停转，不吐数据。可能原因：
- ydlidar_driver 进程异常退出后没发 0xA5 0x65 停止命令，雷达处于混乱状态
- 或雷达自身固件异常

**修复**（临时）：
```python
import serial
ser = serial.Serial("/dev/rplidar", 128000, timeout=1)
ser.write(bytes([0xA5, 0x40]))  # RESET
time.sleep(2)
ser.write(bytes([0xA5, 0x60]))  # START SCAN
# 立刻涌出 2048 字节数据
```

### 🔴 根因 B：Fast-DDS 发现机制全面退化

**证据**：
```
ros2 node list  → 只有 /simple_camera, /robot_state_publisher, /slam_toolbox
                  ❌ 没有 /ydlidar_driver (进程在跑但节点没注册)
                  ❌ 没有 /slam_web_node  (Flask 工作但 ROS2 订阅消失)
                  ❌ 没有 /cmd_vel_bridge

ros2 topic info /scan               → Publisher count: 0 (ydlidar 在发但没人看见)
ros2 topic info /camera/rgb/image_raw → Subscription count: 0 (slam_web 订阅了但不可见)
```

**对比**：`simple_camera` 发布能被看到（它启动早），后期启动的节点全部不可见 → 典型的 **Fast-DDS 在 ARM64 Docker 长期运行后的参与者失联**。

**文档证据**：`docs/slam-web-build-log.md` 问题 2 已经记录过这个：  
> "Fast-DDS 在 ARM64 Docker 内长期运行后退化，`/cmd_vel` 发布者存在但订阅者收不到"

当时是放弃 /cmd_vel 改用 Rosmaster 直驱绕开。但这次 /scan、/camera 是**只读订阅**，绕不开。

## 三、修复方案

### 短期（立即可做）：重启 patrol_car 容器

```bash
docker restart patrol_car
sleep 15

# 拉起 ydlidar
docker exec -d patrol_car bash -c '
  source /opt/ros/foxy/setup.bash
  cd /home/pi/patrol_robot/patrol_robot/src
  python3 ydlidar_driver.py > /tmp/ydlidar.log 2>&1'

# 拉起 slam_web
docker exec -d patrol_car bash -c '
  export PYTHONPATH=/home/pi/patrol_robot/patrol_robot/install/lib/python3.8/site-packages:$PYTHONPATH
  source /opt/ros/foxy/setup.bash
  python3 -m slam_web.web_server > /tmp/slam_web.log 2>&1'
```

### 长期（治本）：切换到 CycloneDDS

Fast-DDS 在 ARM64 Docker 长期运行下的退化是已知问题。`slam-web-design.md` 第 7 节已经提到备选：

> **备选方案:** 如果 Fast-DDS 跨容器不通，两个容器都切换为 `rmw_cyclonedds_cpp`。

修改 `docker-compose.yml`：
```yaml
patrol_car:
  environment:
    - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # 改这里
slam_nav:
  environment:
    - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # 改这里
```

容器内可能需要 `apt-get install -y ros-foxy-rmw-cyclonedds-cpp`（Foxy）和 `ros-humble-rmw-cyclonedds-cpp`（Humble）。

## 四、本次新学到的经验

1. **YDLIDAR 雷达软复位命令**：`0xA5 0x40` (RESET) + `0xA5 0x60` (START) —— 比重启进程可靠
2. **DDS 退化的诊断特征**：进程在跑、日志正常、但 `ros2 node list` 看不到节点 → 必然是 DDS 发现问题
3. **多个节点同时消失 ≠ 多个独立 bug** → 大概率是中间件层统一故障
4. **先验证硬件层（cat /dev/xxx），再验证 ROS 层（ros2 topic）** → 自底向上排查最快

## 五、待用户确认

执行 `docker restart patrol_car` + 后续重拉命令（见上文短期方案）。
