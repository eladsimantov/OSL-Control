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



def test_position_control():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    knee.calibrate(10)
    knee.idle()
    print("\n Position Control Example \n")
    knee.closed_loop()
    time.sleep(3)
    knee.position_deg(0)
    time.sleep(10)
    print("\n ------------------------ \n")
    knee.idle()

def test_velocity_control():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    knee.calibrate(10)
    knee.idle()
    print("\n Velocity Control Example \n")
    knee.closed_loop()
    time.sleep(3)
    knee.velocity_deg_s(3)
    
    time.sleep(10)
    print("\n ------------------------ \n")
    knee.idle()torque_e

def test_torque_control():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    knee.calibrate(10)
    knee.idle()
    print("\n Torque Control Example \n")
    knee.closed_loop()
    time.sleep(3)
    #knee.set_limit_current(10,10)
    knee.torque_nm(0.2)
    
    time.sleep(20)
    print("\n ------------------------ \n")
    knee.idle()

def test_impedance_control():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    knee.calibrate(10)
    knee.idle()
    print("\n Impedance Control Example \n")
    knee.closed_loop()
    time.sleep(3)
    #knee.set_limit_current(10,10)
    #for refrence: impedance_control(self, kp=0.1, kd=0, pos_eq_deg=30.0, stop_time=10,torque_eq_nm=0)
    knee.impedance_control()
    time.sleep(20)
    print("\n ------------------------ \n")
    knee.idle()

if __name__ == "__main__":
    CAN_CH = 'can0'
    BITRATE = 1000000  # Default is 1Mb/s 
    os.system(f"source /home/enable-lab/Desktop/OSL-Control/.venv/bin/activate")
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE}")
    #test_position_control()
    #test_velocity_control()
    #test_torque_control()
    #test_impedance_control()
    
