#!/usr/bin/env python3
"""测试2: Rosmaster_Lib 直驱, 不涉及 ROS2
"""
import sys, time
try:
    from Rosmaster_Lib import Rosmaster
except ImportError as e:
    print(f"[T2] ❌ 找不到 Rosmaster_Lib: {e}")
    sys.exit(1)

print("[T2] 初始化 Rosmaster (/dev/myserial)...")
try:
    car = Rosmaster(com="/dev/myserial", debug=True)
    car.set_car_type(1)   # 1=X3 麦克纳姆轮
    print("[T2] ✅ 底盘对象就绪")
except Exception as e:
    print(f"[T2] ❌ 初始化失败: {e}")
    sys.exit(1)

# 读电压看通信 OK
try:
    v = car.get_battery_voltage()
    print(f"[T2] 电池电压: {v} V")
except Exception as e:
    print(f"[T2] ⚠️ 读电压失败: {e}")

# 前进 1.0 秒 (慢速)
print("[T2] 🚗 前进 0.15 m/s × 1.0 秒 ...")
car.set_car_motion(0.15, 0, 0)   # vx, vy, wz
time.sleep(1.0)

print("[T2] ⏹️ 停止")
car.set_car_motion(0, 0, 0)
time.sleep(0.5)

# 右转 1.0 秒
print("[T2] 🔄 原地右转 0.5 rad/s × 1.0 秒 ...")
car.set_car_motion(0, 0, 0.5)
time.sleep(1.0)

print("[T2] ⏹️ 停止")
car.set_car_motion(0, 0, 0)

print("[T2] 完成 ✅")
