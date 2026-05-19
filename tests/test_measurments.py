# !/usr/bin/env python3
import os
import sys
import time
# tests_path = os.path.dirname(os.path.abspath(__file__))
# project_path = os.path.join(tests_path, "..")
# sys.path.insert(0, project_path)
# from can_example_2026 import ODriveCAN, ODriveMotor
from src.drivers.odrive_can import ODriveCAN, ODriveMotor
# from src.adapters.actuator import ODriveActuator
# from opensourceleg.actuators import MOTOR_CONSTANTS

def velocity_read():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    knee.calibrate(10)
    knee.idle()
    knee.follow_velocity()
    time.sleep(10)
    knee.idle()

if __name__ == "__main__":
    CAN_CH = 'can0'
    BITRATE = 1000000  # Default is 1Mb/s 
    os.system(f"source /home/enable-lab/Desktop/OSL-Control/.venv/bin/activate")
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE}")
    velocity_read()