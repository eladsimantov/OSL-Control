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
from src.drivers.odrive_can import ODriveMotor, ODriveCAN
from opensourceleg.actuators import ActuatorBase, MOTOR_CONSTANTS, CONTROL_MODES
from src.adapters.actuator import ODriveActuator

from src.adapters.imu import IMUAdapter
from opensourceleg.sensors.imu import BNO055

if __name__ == "__main__":
    # can1 = ODriveCAN(node_id=1)
    # kneeActuator = ODriveActuator(
    #             tag="act1",
    #             can_interface=can1,
    #             gear_ratio=100,
    #             motor_constants=MOTOR_CONSTANTS(2048, 0.02, 0.001, 0.0001, 80.0, 120.0)
    # )
    # TODO: add test cases based on the examples.
    print(".")
    thighIMU= BNO055(tag="Timu", addr=40, offline=False)
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
    # loadcell = SRILoadCell_M8123B2(
    #     tag="loadcell1",
    #     calibration_matrix = np.eye(6),  # Placeholder, should be replaced with actual calibration matrix
    #     amp_gain=125.0,
    #     exc=5.0,
    #     bus=1
    # )
    # imu = BNO055(tag="imu1", bus=1, address=0x28)
    # encoder = AS5048B(tag="encoder1", bus=1, address=0x40)

    # OSK("OSK", actuators={"knee": kneeActuator}, sensors={  "loadcell": loadcell, "imu": imu, "encoder": encoder })

