#!/usr/bin/env python3
"""决定性测试: 单次 vs 10Hz 循环发送"""
from Rosmaster_Lib import Rosmaster
import time

car = Rosmaster(com="/dev/myserial", debug=False)
car.set_car_type(1)
car.create_receive_threading()
time.sleep(1.0)

# 测试1: 单次发送 前进 → 等 4 秒
print(">>> 测试1: 单次 set_car_motion(0.2,0,0) + sleep(4)")
e0 = car.get_motor_encoder()
car.set_car_motion(0.2, 0, 0)
time.sleep(4.0)
e1 = car.get_motor_encoder()
car.set_car_motion(0, 0, 0)
print(f"  Δ = {[e1[i]-e0[i] for i in range(4)]}")
time.sleep(1.0)

# 测试2: 10Hz 循环发送 4 秒
print("\n>>> 测试2: 循环 10Hz set_car_motion(0.2,0,0) × 40 次 (4秒)")
e0 = car.get_motor_encoder()
for _ in range(40):
    car.set_car_motion(0.2, 0, 0)
    time.sleep(0.1)
e1 = car.get_motor_encoder()
car.set_car_motion(0, 0, 0)
print(f"  Δ = {[e1[i]-e0[i] for i in range(4)]}")
time.sleep(1.0)

# 测试3: 20Hz 循环发送 4 秒
print("\n>>> 测试3: 循环 20Hz set_car_motion(0.2,0,0) × 80 次 (4秒)")
e0 = car.get_motor_encoder()
for _ in range(80):
    car.set_car_motion(0.2, 0, 0)
    time.sleep(0.05)
e1 = car.get_motor_encoder()
car.set_car_motion(0, 0, 0)
print(f"  Δ = {[e1[i]-e0[i] for i in range(4)]}")
time.sleep(1.0)

# 测试4: 10Hz 右转
print("\n>>> 测试4: 循环 10Hz set_car_motion(0,0,0.5) × 40 次 原地右转")
e0 = car.get_motor_encoder()
for _ in range(40):
    car.set_car_motion(0, 0, 0.5)
    time.sleep(0.1)
e1 = car.get_motor_encoder()
car.set_car_motion(0, 0, 0)
print(f"  Δ = {[e1[i]-e0[i] for i in range(4)]}")

print("\n完成 ✅")
