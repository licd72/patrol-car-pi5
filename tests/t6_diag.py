#!/usr/bin/env python3
"""高PWM冲击测试 - 判定是硬件还是软件问题"""
from Rosmaster_Lib import Rosmaster
import time

car = Rosmaster(com="/dev/myserial", debug=False)
car.set_car_type(1)
car.create_receive_threading()
time.sleep(1.2)

def snapshot(label):
    v = car.get_battery_voltage()
    e = car.get_motor_encoder()
    print(f"  [{label}] voltage={v}V, encoder={e}")
    return e

# baseline
e0 = snapshot("baseline")

# ── 试 A: reset_car_state（重置 STM32 内部状态，可能恢复电机使能）
print("\n>>> A. reset_car_state() 后立即 set_motor(100,100,100,100) 4 秒")
car.reset_car_state()
time.sleep(0.3)
car.set_motor(100, 100, 100, 100)   # 满 PWM
time.sleep(4.0)
eA = snapshot("A-100%PWM 4s后")
car.set_motor(0, 0, 0, 0)
print(f"  Δencoder = {[eA[i]-e0[i] for i in range(4)]}")

# ── 试 B: set_auto_report_state(True) 打开自动上报
print("\n>>> B. set_auto_report_state(True) 再冲击")
time.sleep(0.5)
try:
    car.set_auto_report_state(True, forever=False)
    print("  auto_report on ✅")
except Exception as e:
    print(f"  {e}")
time.sleep(0.5)
eB0 = car.get_motor_encoder()
car.set_motor(100, -100, 100, -100)   # 原地自转（麦轮：左右反相）
time.sleep(3.0)
eB = snapshot("B-原地自转 3s")
car.set_motor(0, 0, 0, 0)
print(f"  Δencoder = {[eB[i]-eB0[i] for i in range(4)]}")

# ── 试 C: 单独驱动 M1 一个电机
print("\n>>> C. 只驱动 M1: set_motor(100,0,0,0) 3秒")
time.sleep(0.5)
eC0 = car.get_motor_encoder()
car.set_motor(100, 0, 0, 0)
time.sleep(3.0)
eC = snapshot("C-单独M1")
car.set_motor(0, 0, 0, 0)
print(f"  Δencoder = {[eC[i]-eC0[i] for i in range(4)]}")

# ── 试 D: 单独驱动 M2
print("\n>>> D. 只驱动 M2: set_motor(0,100,0,0) 3秒")
time.sleep(0.5)
eD0 = car.get_motor_encoder()
car.set_motor(0, 100, 0, 0)
time.sleep(3.0)
eD = snapshot("D-单独M2")
car.set_motor(0, 0, 0, 0)
print(f"  Δencoder = {[eD[i]-eD0[i] for i in range(4)]}")

# ── 试 E: 单独驱动 M3、M4
print("\n>>> E. 只驱动 M3: set_motor(0,0,100,0) 3秒")
time.sleep(0.5)
eE0 = car.get_motor_encoder()
car.set_motor(0, 0, 100, 0)
time.sleep(3.0)
eE = snapshot("E-单独M3")
car.set_motor(0, 0, 0, 0)
print(f"  Δencoder = {[eE[i]-eE0[i] for i in range(4)]}")

print("\n>>> F. 只驱动 M4: set_motor(0,0,0,100) 3秒")
time.sleep(0.5)
eF0 = car.get_motor_encoder()
car.set_motor(0, 0, 0, 100)
time.sleep(3.0)
eF = snapshot("F-单独M4")
car.set_motor(0, 0, 0, 0)
print(f"  Δencoder = {[eF[i]-eF0[i] for i in range(4)]}")

print("\n>>> G. 蜂鸣器（验证 STM32 仍响应）")
car.set_beep(1); time.sleep(0.4); car.set_beep(0)

# 结论
print("\n===== 结论 =====")
totals = [eF[i]-e0[i] for i in range(4)]
print(f"22秒累计 Δencoder = {totals}")
if all(abs(x) < 500 for x in totals):
    print("❌ 电机功率驱动被关闭（硬件/使能层）")
else:
    print("✅ 电机能被驱动")
