#!/usr/bin/env python3
import sys,time
sys.path.insert(0,'/home/pi/patrol_robot')
from Rosmaster_Lib import Rosmaster
for tty in ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3']:
    try:
        b=Rosmaster(car_type=1, com=tty)
        b.create_receive_threading()
        time.sleep(0.5)
        b.set_auto_report_state(True)
        time.sleep(0.5)
        v=b.get_battery_voltage()
        print(f'{tty} => BATTERY: {v}V')
    except Exception as e:
        print(f'{tty} => ERROR: {e}')
