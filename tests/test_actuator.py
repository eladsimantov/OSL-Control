# !/usr/bin/env python3
import os
import sys
import time
import math
import numpy as np
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
    
    knee.idle()
    print("\n Position Control Example \n")
    knee.closed_loop()
    knee.set_limit_current(10,15)
    time.sleep(10)
    knee.position_deg(5)
    time.sleep(2)
    knee.position_deg(45)
    time.sleep(10)
    knee.get_velocity()
    knee.read_position()
    knee.read_current()
    knee.get_velocity()
    time.sleep(10)
    print("\n ------------------------ \n")
    knee.idle()

def test_velocity_control():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    #knee.calibrate(10)
    knee.idle()
    print("\n Velocity Control Example \n")
    knee.set_limit_current(max_current=10, max_velocity=5)
    knee.closed_loop()
    time.sleep(3)
    knee.velocity_deg_s(1)
    knee.follow_velocity()
    time.sleep(10)
    print("\n ------------------------ \n")
    knee.idle()

def test_torque_control(tau_command=0.2):
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    #knee.calibrate(10)
    knee.idle()
    print("\n Torque Control Example \n")
    knee.closed_loop()
    time.sleep(3)
    knee.set_limit_current(max_current=10, max_velocity=30)
    knee.torque_nm(tau_command)
    knee.follow_current()

    
    print("\n ------------------------ \n")
    knee.idle()

def test_impedance_control():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    # knee.calibrate(10)
    knee.idle()
    print("\n Impedance Control Example \n")
    knee.set_limit_current(10,30)
    knee.closed_loop()
    time.sleep(3)
    
    #for refrence: impedance_control(self, kp=0.1, kd=0, pos_eq_deg=30.0, stop_time=10,torque_eq_nm=0)
    knee.impedance_control(kp=0.025, kd=0.00005,pos_eq_deg=25,stop_time=20)
    
    print("\n ------------------------ \n")
    knee.idle()

def test_sine_movement():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    knee.set_limit_current(10,10)
    knee.set_state(1)
    print("idle")
    time.sleep(2)
    knee.set_state(8)
    # knee.calibrate(15)
    knee.position_deg(0)
    tNow = time.time()
    DeltaT = 20
    sine = lambda p, Amp, N: Amp * math.sin(2 * math.pi * N * p)  # 30 degrees amplitude, 0.1 Hz frequency
    ramp = lambda p, Amp: Amp * p  # Ramp from 0 to 10 degrees over the duration of the test
    while time.time() - tNow < DeltaT:
        phase = ( time.time() - tNow ) / DeltaT
        knee.position_deg(
            ramp(phase,Amp=20) + sine(phase, Amp=20, N=5) 
            )
    knee.position_deg(0)
    knee.set_state(1)

def test_idle():
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    can1 = ODriveCAN(node_id=0,dbc_path=dbc_path)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)
    knee.idle()


if __name__ == "__main__":
    CAN_CH = 'can0'
    BITRATE = 1000000  # Default is 1Mb/s 
    # os.system(f"source /home/enable-lab/Desktop/OSL-Control/.venv/bin/activate")
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE}")
    try:
        # test_position_control()
        # test_velocity_control()
        # test_torque_control(0.2)
        # test_sine_movement()
        test_impedance_control()
    
    except KeyboardInterrupt:
        print("Moving Knee to Idle mode")
        test_idle()    
        os.system(f"sudo ip link set {CAN_CH} down")