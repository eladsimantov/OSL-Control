# !/usr/bin/env python3
import os
import sys
# tests_path = os.path.dirname(os.path.abspath(__file__))
# project_path = os.path.join(tests_path, "..")
# sys.path.insert(0, project_path)

from src.drivers.odrive_can import ODriveCAN
from src.adapters.actuator import ODriveActuator
from opensourceleg.actuators import MOTOR_CONSTANTS

def test_actuator():
    can1 = ODriveCAN(node_id=1)
    kneeActuator = ODriveActuator(
                tag="act1",
                can_interface=can1,
                gear_ratio=100,
                motor_constants=MOTOR_CONSTANTS(2048, 0.02, 0.001, 0.0001, 80.0, 120.0)
    )
    print("\n ------------------------ \n")


if __name__ == "__main__":
    test_actuator()
