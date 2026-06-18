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


def test_position_control(knee):
    knee.idle()
    print("\n Position Control Example \n")
    knee.closed_loop()
    knee.set_limit_current(10,15)
    time.sleep(2)
    print("\n Moving to 5 Degrees \n")
    knee.position_deg(5)
    time.sleep(2)
    print("\n Moving to 45 Degrees \n")
    knee.position_deg(45)
    time.sleep(2)
    print("\n Moving to 15 Degrees \n")
    knee.position_deg(15)
    time.sleep(2)
    print("\n Moving to 60 Degrees \n")
    knee.position_deg(60)
    time.sleep(2)
    knee.get_velocity()
    knee.read_position()
    knee.read_current()
    knee.get_velocity()
    time.sleep(2)
    print("\n ------------------------ \n")
    knee.idle()

def test_velocity_control(knee):
    #knee.calibrate(10)
    knee.idle()
    print("\n Velocity Control Example \n")
    knee.set_limit_current(max_current=10, max_velocity=5)
    knee.closed_loop()
    time.sleep(3)
    knee.velocity_deg_s(1)
    knee.follow_velocity()
    time.sleep(2)
    print("\n ------------------------ \n")
    knee.idle()

def test_torque_control(knee):
    #knee.calibrate(10)
    tau_command=0.165 # Nm
    knee.idle()
    print("\n Torque Control Example \n")
    knee.closed_loop()
    time.sleep(1.5)
    knee.set_limit_current(max_current=10, max_velocity=20)
    knee.torque_nm(tau_command)
    knee.follow_current()

    
    print("\n ------------------------ \n")
    knee.idle()

def test_torque_sin(knee):
    tau_command=0.075 # Nm
    knee.idle()
    knee.set_limit_current(max_current=10, max_velocity=20)

    print("\n Torque Control Example \n")
    knee.closed_loop()
    tNow = time.time()
    DeltaT = 20
    sine = lambda p, Amp, N: Amp * math.sin(2 * math.pi * N * p)  # 30 degrees amplitude, 0.1 Hz frequency
    ramp = lambda p, Amp: Amp * p  # Ramp from 0 to 10 degrees over the duration of the test

    while time.time() - tNow < 3:
        phase = ( time.time() - tNow ) / 3
        knee.torque_nm(0.1+ramp(phase, tau_command))
    while time.time() - tNow < 3 + 2:
        phase = ( time.time() - tNow ) / 2
        knee.torque_nm(0.0)
    while time.time() - tNow < 3 + 2 + 3:
        phase = ( time.time() - tNow ) / 3
        knee.torque_nm(0.05 + sine(phase, tau_command, N=1))
    while time.time() - tNow < 3 + 2 + 3 + 3:
        phase = ( time.time() - tNow ) / 3
        knee.torque_nm(0.1+ramp(phase, tau_command))




    
        # try:
        #     current_pos = knee.read_position() 
        #     if current_pos < 5:
        #         knee.torque_nm(0.0)
        # except Exception as e:
        #     print(f"Error reading position: {e}")

    knee.idle()
    return

def test_impedance_control(knee):
    # knee.calibrate(10)
    knee.idle()
    print("\n Impedance Control Example \n")
    knee.set_limit_current(10,30)
    knee.closed_loop()
    time.sleep(3)
    
    #for refrence: impedance_control(self, kp=0.1, kd=0, pos_eq_deg=30.0, stop_time=10,torque_eq_nm=0)
    knee.impedance_control(kp=0.020, kd=0.00006,pos_eq_deg=40,stop_time=20)
    
    print("\n ------------------------ \n")
    knee.idle()

def test_sine_movement(knee):
    knee.set_limit_current(10,10)
    knee.set_state(1)
    print("idle")
    time.sleep(2)
    knee.set_state(8)
    # knee.calibrate(15)
    knee.position_deg(10)
    tNow = time.time()
    DeltaT = 20
    sine = lambda p, Amp, N: Amp * math.sin(2 * math.pi * N * p)  # 30 degrees amplitude, 0.1 Hz frequency
    ramp = lambda p, Amp: Amp * p  # Ramp from 0 to 10 degrees over the duration of the test
    while time.time() - tNow < DeltaT:
        phase = ( time.time() - tNow ) / DeltaT
        knee.position_deg(
            ramp(phase,Amp=20) + sine(phase, Amp=20, N=5) + 10
            )
    knee.position_deg(0)
    knee.set_state(1)


if __name__ == "__main__":
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    node_id = 0
    CAN_CH = 'can0'
    BITRATE = 1000000  # Default is 1Mb/s 
    can1 = ODriveCAN(node_id=node_id,dbc_path=dbc_path,bus_name=CAN_CH)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40)

    # os.system(f"source /home/enable-lab/Desktop/OSL-Control/.venv/bin/activate")

    # # CAN SHIELD SETTINGS
    # os.system(f"sudo ip link set {CAN_CH} down")
    # os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE}")

    # USB2CAN adapter settings
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE} sample-point 0.750")
    os.system(f"sudo ip link set {CAN_CH} txqueuelen 1000") # Set the queue length to 1000 so the USB buffer doesn't overflow

    try:
        
        test_position_control(knee)
        # test_velocity_control(knee)
        # test_torque_control(knee)
        # test_sine_movement(knee)
        # test_impedance_control(knee)
        # knee.idle()
        # test_torque_sin(knee)
    
    except KeyboardInterrupt:
        print("Moving Knee to Idle mode")
        knee.idle()  
        os.system(f"sudo ip link set {CAN_CH} down")