#!/usr/bin/env python3
"""测试1: 直接串口，不用 Rosmaster_Lib
STM32 ERF01 协议: 0xFF 0xFC ... 校验 0xFF
这里用 Rosmaster_Lib 底层的 tx_data
"""
import sys, time, serial

PORT = "/dev/myserial"

print(f"[T1] 尝试打开 {PORT}")
try:
    ser = serial.Serial(PORT, 115200, timeout=1)
    print(f"[T1] ✅ 打开成功: {ser}")
except Exception as e:
    print(f"[T1] ❌ 无法打开串口: {e}")
    sys.exit(1)

print("[T1] 保持打开 1 秒后关闭...")
time.sleep(1)
ser.close()
print("[T1] 关闭成功")
