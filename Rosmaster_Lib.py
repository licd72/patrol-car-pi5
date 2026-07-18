
#!/usr/bin/env python3
"""
Rosmaster_Lib - 精简版 (patrol-car-pi5)
仅实现底盘控制所需的核心功能，兼容 Yahboom STM32 扩展板协议。

协议: 0xFF + DEVICE_ID + LEN + FUNC + DATA + CHECKSUM
串口: 115200 8N1
"""

import struct
import time
import serial
import threading


class Rosmaster:
    """Yahboom ROSMASTER 底盘驱动 (X3/R2 兼容)"""
    
    # 常量
    __HEAD = 0xFF
    __DEVICE_ID = 0xFC
    __COMPLEMENT = 257 - 0xFC  # = 1
    
    # 功能码
    FUNC_MOTOR = 0x10
    FUNC_CAR_RUN = 0x11
    FUNC_MOTION = 0x12
    FUNC_SET_MOTOR_PID = 0x13
    FUNC_SET_YAW_PID = 0x14
    FUNC_SET_CAR_TYPE = 0x15
    FUNC_AUTO_REPORT = 0x01
    FUNC_BEEP = 0x02
    FUNC_PWM_SERVO = 0x03
    FUNC_RGB = 0x05
    FUNC_RGB_EFFECT = 0x06
    FUNC_RESET_STATE = 0x0F
    FUNC_VERSION = 0x51
    FUNC_RESET_FLASH = 0xA0
    
    # 车型
    CARTYPE_X3 = 0x01
    CARTYPE_X3_PLUS = 0x02
    CARTYPE_X1 = 0x04
    CARTYPE_R2 = 0x05
    
    def __init__(self, car_type=1, com="/dev/myserial", delay=0.002, debug=False):
        self.ser = serial.Serial(com, 115200, timeout=0.1)
        self._delay = delay
        self._debug = debug
        self._car_type = car_type
        self._running = True
        
        # 传感器数据
        self._encoder_m1 = 0
        self._encoder_m2 = 0
        self._encoder_m3 = 0
        self._encoder_m4 = 0
        self._imu_roll = 0.0
        self._imu_pitch = 0.0
        self._imu_yaw = 0.0
        self._gyro_x = 0.0
        self._gyro_y = 0.0
        self._gyro_z = 0.0
        self._battery = 0.0
        
        self._read_thread = None
        self._auto_report = False
    
    # ── 协议层 ──
    def _checksum(self, data):
        """计算校验和"""
        return (sum(data, self.__COMPLEMENT) & 0xFF)
    
    def _send_cmd(self, func, params=b''):
        """发送命令帧: HEAD + DEV_ID + LEN + FUNC + PARAMS + CSUM"""
        cmd = bytearray([self.__HEAD, self.__DEVICE_ID, 0, func])
        cmd.extend(params)
        cmd[2] = len(cmd) - 1  # 长度
        cmd.append(self._checksum(cmd))
        
        if self._debug:
            print(f"[Rosmaster] TX: {' '.join(f'{b:02X}' for b in cmd)}")
        
        try:
            self.ser.write(bytes(cmd))
            self.ser.flush()
        except Exception as e:
            if self._debug:
                print(f"[Rosmaster] write error: {e}")
        
        time.sleep(self._delay)
    
    # ── 底盘运动 ──
    def set_car_motion(self, v_x, v_y, v_z):
        """全向运动控制 (X3麦轮: vx/vy ±1.0, vz ±5.0)"""
        try:
            vx_bytes = struct.pack('h', int(v_x * 1000))
            vy_bytes = struct.pack('h', int(v_y * 1000))
            vz_bytes = struct.pack('h', int(v_z * 1000))
            params = bytes([self._car_type]) + vx_bytes + vy_bytes + vz_bytes
            self._send_cmd(self.FUNC_MOTION, params)
        except Exception as e:
            if self._debug:
                print(f"[Rosmaster] set_car_motion error: {e}")
    
    def set_car_type(self, car_type):
        """设置车型 (1=X3, 5=R2)"""
        self._car_type = car_type
        self._send_cmd(self.FUNC_SET_CAR_TYPE, bytes([car_type]))
    
    def set_auto_report_state(self, enable=True, forever=False):
        """启用/禁用自动上报 (编码器+IMU+电压)"""
        self._auto_report = enable
        self._send_cmd(self.FUNC_AUTO_REPORT, bytes([1 if enable else 0, 1 if forever else 0]))
    
    def create_receive_threading(self):
        """启动后台接收线程"""
        if self._read_thread and self._read_thread.is_alive():
            return
        self._running = True
        self._read_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._read_thread.start()
    
    def set_beep(self, on=True):
        """蜂鸣器"""
        self._send_cmd(self.FUNC_BEEP, bytes([1 if on else 0]))
    
    def set_colorful_lamps(self, led_id, r, g, b):
        """RGB灯"""
        self._send_cmd(self.FUNC_RGB, bytes([led_id, r, g, b]))
    
    # ── 传感器读取 ──
    def get_motor_encoder(self):
        """返回 (m1, m2, m3, m4) 编码器计数"""
        return (self._encoder_m1, self._encoder_m2, self._encoder_m3, self._encoder_m4)
    
    def get_imu_attitude_data(self):
        """返回 (roll, pitch, yaw) 单位: 度"""
        return (self._imu_roll, self._imu_pitch, self._imu_yaw)
    
    def get_gyroscope_data(self):
        """返回 (gx, gy, gz) 单位: rad/s"""
        return (self._gyro_x, self._gyro_y, self._gyro_z)
    
    def get_battery_voltage(self):
        """返回电池电压 (V)"""
        return self._battery
    
    def reset_state(self):
        """复位 STM32 状态"""
        self._send_cmd(self.FUNC_RESET_STATE)
    
    # ── 后台接收 ──
    def _receive_loop(self):
        """后台串口接收线程 — 解析自动上报数据"""
        buf = bytearray()
        
        while self._running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    buf.extend(data)
                    
                    # 解析帧
                    while len(buf) >= 5:
                        if buf[0] != self.__HEAD:
                            buf.pop(0)
                            continue
                        
                        if len(buf) < 3:
                            break
                        
                        frame_len = buf[2] + 2  # 数据长度 + HEAD + LEN
                        if len(buf) < frame_len:
                            break
                        
                        frame = buf[:frame_len]
                        buf = buf[frame_len:]
                        
                        # 校验
                        expected = self._checksum(frame[:-1])
                        if frame[-1] != expected:
                            continue
                        
                        self._parse_frame(frame)
                else:
                    time.sleep(0.001)
            except Exception as e:
                if self._debug:
                    print(f"[Rosmaster] recv error: {e}")
                time.sleep(0.01)
    
    def _parse_frame(self, frame):
        """解析接收帧"""
        func = frame[3]
        data = frame[4:-1]  # 去掉校验和
        
        try:
            if func == self.FUNC_MOTOR:  # 编码器上报
                if len(data) >= 8:
                    self._encoder_m1, self._encoder_m2, \
                    self._encoder_m3, self._encoder_m4 = \
                        struct.unpack('hhhh', data[:8])
            
            elif func == 0x61:  # IMU 姿态 (自动上报格式)
                if len(data) >= 6:
                    self._imu_roll, self._imu_pitch, self._imu_yaw = \
                        struct.unpack('hhh', data[:6])
                    self._imu_roll /= 100.0
                    self._imu_pitch /= 100.0
                    self._imu_yaw /= 100.0
            
            elif func == 0x62:  # 陀螺仪
                if len(data) >= 6:
                    self._gyro_x, self._gyro_y, self._gyro_z = \
                        struct.unpack('hhh', data[:6])
                    self._gyro_x /= 1000.0
                    self._gyro_y /= 1000.0
                    self._gyro_z /= 1000.0
            
            elif func == 0x63:  # 电池电压
                if len(data) >= 2:
                    self._battery = struct.unpack('H', data[:2])[0] / 1000.0
            
            elif func == 0x51:  # 版本号
                pass  # 忽略
            
        except Exception as e:
            if self._debug:
                print(f"[Rosmaster] parse error: {e}")
    
    def close(self):
        """关闭串口"""
        self._running = False
        try:
            self.set_car_motion(0, 0, 0)
        except:
            pass
        if self._read_thread:
            self._read_thread.join(timeout=1)
        self.ser.close()
    
    def __del__(self):
        self.close()


# 兼容旧接口
Rosmaster_Lib = Rosmaster

