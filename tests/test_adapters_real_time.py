# !/usr/bin/env python3
"""
test_adapters.py - This module tests the adapters library. This library is meant to provide an interface between the OSL API and our custom hardware drivers, allowing us to use OSL's architecture and control modes while interfacing with our specific hardware.
The tests in this module will focus on verifying that all classes are properly initialized, that the methods are correctly implemented, and that the integration with the OSL API is seamless. 
We will also include some basic functionality tests to ensure that the actuators can be controlled and that sensors give readings.

Author: Elad Siman Tov
Date: 2026-01-01
"""

import os
import sys
tests_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.join(tests_path, "..")
sys.path.insert(0, project_path)
from src.drivers.odrive_can import ODriveCAN, ODriveMotor
# from opensourceleg.actuators import ActuatorBase, MOTOR_CONSTANTS, CONTROL_MODES
from tests.test_actuator import test_impedance_control
# from src.adapters.imu import IMUAdapter
# from opensourceleg.sensors.imu import BNO055
from src.adapters.loadcell import SRILoadCell_M8123B2

from opensourceleg.utilities import SoftRealtimeLoop


if __name__ == "__main__":
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    motor_node_id = 0
    CAN_CH = 'can2'
    BITRATE = 1000000  # Default is 1Mb/s 
    can1 = ODriveCAN(node_id=motor_node_id,dbc_path=dbc_path,bus_name=CAN_CH)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40) 
    knee.idle()

    loadcell = SRILoadCell_M8123B2(tag="Shank LC", channel=CAN_CH)
    loadcell.start()
    loadcell.calibrate()

    loop = SoftRealtimeLoop(dt=0.01)
    for t in loop:
        print("\n Impedance Control Example \n")
        knee.set_limit_current(10,30)
        knee.closed_loop()
    
        knee.impedance_control(kp=0.025, kd=0.00005,pos_eq_deg=25,stop_time=20)
        loadcell.update()     
        # Print Force (N) and Moments (Nm)
        # Moments use properties mx, my, mz from your class
        print(f"F(xyz) N: [{loadcell.fx:6.2f}, {loadcell.fy:6.2f}, {loadcell.fz:6.2f}] | "
                f"M(xyz) Nm: [{loadcell.mx:6.2f}, {loadcell.my:6.2f}, {loadcell.mz:6.2f}]", end='\n')

        if t >= 20:
            loop.stop()

    # thighIMU= BNO055(tag="Timu", addr=40, offline=False)
    # thighIMU = IMUAdapter(tag="Timu", address=40, offline=True)
    # from opensourceleg.actuators import ActuatorBase, MOTOR_CONSTANTS, CONTROL_MODES
    # from src.drivers.odrive_can import ODriveMotor, ODriveCAN
    # import numpy as np

    # can1 = ODriveCAN(node_id=1)
    # kneeActuator = ODriveActuator(
    #             tag="act1",
    #             can_interface=can1,
    #             gear_ratio=100,
    #             motor_constants=MOTOR_CONSTANTS(2048, 0.02, 0.001, 0.0001, 80.0, 120.0)
    # )




    # imu = BNO055(tag="imu1", bus=1, address=0x28)
    # encoder = AS5048B(tag="encoder1", bus=1, address=0x40)

    # OSK("OSK", actuators={"knee": kneeActuator}, sensors={  "loadcell": loadcell, "imu": imu, "encoder": encoder })

