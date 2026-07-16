#!/usr/bin/env python3
from Rosmaster_Lib import Rosmaster
import time

print("Step1: 打开串口 + 类型")
car = Rosmaster(com="/dev/myserial", debug=False)
car.set_car_type(1)
time.sleep(0.5)

print("Step2: 启动接收线程")
car.create_receive_threading()
time.sleep(1.0)

print(f"  电压: {car.get_battery_voltage()}V")
print(f"  编码器 t0: {car.get_motor_encoder()}")

# 底层 set_motor 直接 4 轮 PWM
print("\nStep3: set_motor(50,50,50,50) 4 秒 (⚠️ 轮子应该转)")
car.set_motor(50, 50, 50, 50)
time.sleep(4.0)
print(f"  编码器 t1: {car.get_motor_encoder()}")

print("Step4: set_motor(0,0,0,0) 停")
car.set_motor(0, 0, 0, 0)
time.sleep(1.0)
print(f"  编码器 t2: {car.get_motor_encoder()}")

# set_car_motion (含运动学)
print("\nStep5: set_car_motion(0.3,0,0) 4 秒 (⚠️ 轮子应该转)")
car.set_car_motion(0.3, 0, 0)
time.sleep(4.0)
print(f"  编码器 t3: {car.get_motor_encoder()}")
car.set_car_motion(0, 0, 0)
print(f"  编码器 t4 (停车): {car.get_motor_encoder()}")

# set_car_run
print("\nStep7: set_car_run(1, 50) 前进 4 秒")
try:
    car.set_car_run(1, 50)
    time.sleep(4.0)
    print(f"  编码器 t5: {car.get_motor_encoder()}")
    car.set_car_run(0, 0)
except Exception as e:
    print(f"  失败: {e}")

# 蜂鸣器（无关电机，验证 STM32 响应）
print("\nStep8: 蜂鸣器 3 声 (STM32 响应验证)")
for _ in range(3):
    car.set_beep(1)
    time.sleep(0.3)
    car.set_beep(0)
    time.sleep(0.2)

# LED
print("Step9: 彩灯效果测试")
try:
    car.set_colorful_effect(3, 6, parm=1)
    time.sleep(2)
    car.set_colorful_effect(0, 6, parm=1)
except Exception as e:
    print(f"  灯失败: {e}")

print("=== 完成 ===")
