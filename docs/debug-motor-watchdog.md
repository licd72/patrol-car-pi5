# 巡逻小车 /cmd_vel 无法驱动 —— 根因诊断与修复方案

**日期**: 2026-07-17
**症状**: 悬空测试时，向 `/cmd_vel` 发布 Twist，轮子不动
**排查耗时**: 8 个独立测试脚本，未修改任何现有生产代码

---

## 一、症状表现

```
$ ros2 topic info /cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 3        # web / voice / state_machine 都在发
Subscription count: 0     # ⚠️ 无订阅者
```

- 悬空小车 —— 轮子理论上无阻力
- Web `/api/control/move` **能让车动**（HTTP 直接 curl 一次即可）
- 但用 `ros2 topic pub /cmd_vel ...` **不动**
- 语音 / 巡逻状态机发的 `/cmd_vel` **也不动**

---

## 二、8 步诊断（`tests/t1_serial.py` ~ `tests/t8_*`）

### T1: 串口层
```python
serial.Serial("/dev/myserial", 115200)  # ✅ 可打开
```
Linux tty 允许多进程 open，但同时读会互抢字节。

### T2: `Rosmaster_Lib` 层
```python
car.set_car_motion(0.15, 0, 0)  # 发出 [255,252,10,18,...] motion帧
# 但编码器几乎无变化: Δ=[4,3,2,0]
```
表象："收指令但不动"

### T3+T4: ROS2 桥（首个模型）
```python
# t3_bridge.py 订阅 /cmd_vel → set_car_motion()
# t4_send.py  ros2 topic pub -r 10 /cmd_vel ...
```
桥日志显示 `/cmd_vel → vx=0.15` 被正确接收 ✅
串口通信正确 ✅

### T5: 完整驱动序列
`set_motor(50,50,50,50)` 4s → Δenc=`[60,65,60,68]`（几乎不动）

### T6: 多 API 交叉测试 ⭐ 突破口
| 阶段 | 命令 | Δ编码器 |
|:----|:----|:------|
| A | set_motor(100×4) 单次 | `[52, 67, 53, 74]` ❌ |
| B | **set_motor + auto_report on** | `[484, -555, 508, -509]` ✅ |
| C~F | 每个电机单独测试 | 全部有 200~1300 变化 ✅ |

**四个电机全部工作正常！**

### T7: 决定性对比 —— 单次 vs 循环
| 发送模式 | 4秒内 Δ编码器 |
|:--------|:------------|
| **单次 + sleep 4s** | `[41, 38, 47, 46]`   ❌ 几乎不动 |
| **10Hz × 4s** | `[685, 238, 771, 737]` ✅ 连续转 |
| **20Hz × 4s** | `[1183, 854, 1328, 1326]` ✅ 更连贯 |

### T8: cmd_vel_bridge + heartbeat
桥 v1（含读线程）→ 与 web_server 抢串口读缓冲 →
```
serial.SerialException: device reports readiness to read but returned no data
```
桥 v2（只写模式）→ ✅ 稳定运行 11.5s

---

## 三、根因（两层叠加）

### 🔴 Bug A: `/cmd_vel` 无订阅者
- v4 精简版清理时移除了 `Mcnamu_driver_X3` / `base_node_X3`
- 用 `Rosmaster_Lib` 直驱替代，但**只在 web_server.py 的 `/api/control/move` 里绕过 ROS**
- 语音 / 状态机发的 `/cmd_vel` **消息掉进虚空**

### 🔴 Bug B: STM32 底盘 watchdog
- STM32 固件收到 `set_car_motion` 后 **~100ms 自动停车**（安全机制）
- 系统里所有事件式发布（发一次就 sleep）都会被 watchdog 立即停掉
- 只有**持续 ≥10Hz 发送**才能让轮子连续转

## 完整故障链

```
① 程序单次发 /cmd_vel
    ↓
② 无订阅者 (Bug A)
    ↓ 假设有订阅者
③ 桥调用一次 set_car_motion
    ↓
④ STM32 watchdog ~100ms 后自动停车 (Bug B)
    ↓
⑤ 轮子瞬间转一下→肉眼不可见
```

---

## 四、解决方案

### 独立测试脚本 `tests/t8_bridge_v2.py`（不改任何现有代码）

```python
class CmdVelBridge(Node):
    # 关键点：
    # 1. 订阅 /cmd_vel → 转发到 Rosmaster
    # 2. 10Hz heartbeat 持续重发最后一条 Twist (对抗 STM32 watchdog)
    # 3. 0.5s 无新命令→自动停车
    # 4. 只写不读 (避免与 web_server 抢串口读缓冲)
    ...
```

启动方式（独立进程，不改容器）：
```bash
docker exec -d patrol_car bash -c 'source /opt/ros/foxy/setup.bash && python3 /home/pi/patrol_robot/tests/t8_bridge_v2.py > /tmp/bridge.log 2>&1'
```

---

## 五、正式合入建议（下一步，需批准）

1. **cmd_vel_bridge 独立 ROS2 包**
   - `src/patrol_bridge/patrol_bridge/cmd_vel_bridge.py`
   - 加入 `container_init.sh` 启动清单（在 web_server 之前）
   - Web 面板保持 HTTP 直驱功能不变（做 fallback）

2. **修改 web_server 里的 `_stop` 逻辑**
   - 现在: `sleep(duration)` 期间只 publish 一次
   - 改成: `sleep` 时间内以 10Hz 持续 publish（依赖桥的 heartbeat 会更简洁）

3. **在 `patrol_voice._execute_move` 加持续发布**
   - 目前语音识别发一次就走
   - 改成 duration 秒内 10Hz 持续发布

4. **docs/debug-rules.md 追加法则六**
   > 法则六：STM32 底盘 watchdog ~100ms，任何 /cmd_vel 发布必须持续 ≥10Hz

---

## 六、涉及文件（本次调试只写不改）

```
tests/
├── t1_serial.py         # 串口 open 测试
├── t2_rosmaster.py      # Rosmaster_Lib 直驱
├── t3_bridge.py         # 首个桥模型
├── t4_send.py           # Twist 发送测试
├── t5_deep.py           # 完整驱动序列
├── t6_diag.py           # 单电机隔离测试 (突破)
├── t7_hz.py             # 频率对比 (决定性)
├── t8_bridge_v1.py      # 桥+heartbeat (读崩)
├── t8_bridge_v2.py      # 桥 只写版 ✅
└── t8_test_publisher.py # 11.5s 端到端

/tmp/bridge_v2.log       # 桥运行日志
```

**未修改文件**: 0 个（现有生产代码零改动）
