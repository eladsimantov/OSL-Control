"""
This module implements an adapter for ODrive drivers + BLDC motors using the OSL (Open Source Leg) framework.
It defines the ODriveActuator class that extends the ActuatorBase class from OSL, and uses our 
custom ODrive module and CAN to interface with the actuators.
The goal is to provide an integration of ODrive motors into the OSL API, allowing 
us to leverage OSL's architecture while using our ODrive hardware.

Author: Elad Siman Tov
Date: 2026-01-01
"""
import math
import os
import sys
import logging
from typing import Any, Optional

# Import OSL components
from opensourceleg.actuators import ActuatorBase, CONTROL_MODES, MOTOR_CONSTANTS, CONTROL_MODE_CONFIGS, ControlModeConfig

# Import our Custom Driver and interfaces code
project_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, project_path)
from drivers.odrive_can import ODriveMotor, ODriveCAN

LOGGER = logging.getLogger(__name__)

class ODriveActuator(ActuatorBase):
    """
    Impelementation of the OSL actuators class based on Odrive drivers and can interface.
    """
    # TODO: Add actual implementation of methods to interface with ODrive motors via CAN.
    # We may need to override ODriveMotor class as it is unnecessary middleware.
    def __init__(
        self, 
        can_interface: ODriveCAN, 
        tag: str, 
        gear_ratio: float = 1.0, 
        motor_constants = MOTOR_CONSTANTS(
             MOTOR_COUNT_PER_REV=2048,
             NM_PER_AMP=0.02,
             NM_PER_RAD_TO_K=0.001,
             NM_S_PER_RAD_TO_B=0.0001,
             MAX_CASE_TEMPERATURE=80.0,
             MAX_WINDING_TEMPERATURE=120.0
            ),
        frequency: int = 1000,
        **kwargs
    ):

        # 2. Initialize Base Class
        super().__init__(
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=motor_constants,
            frequency=frequency,
            **kwargs
        )

        # 3. Initialize Hardware Driver
        self.driver = ODriveMotor(can_interface, gear_ratio = 1, name=tag)
        
        
    @property
    def _CONTROL_MODE_CONFIGS(self):
        return CONTROL_MODE_CONFIGS()
    def start(self):
        print("Started")
    def stop(self):
        print("Stopped")
    def update(self):
        print("Updated")
    def set_motor_voltage(self, value: float) -> None:
        print(f"Motor voltage set to {value}")
    def set_motor_current(self, value: float) -> None:
        print(f"Motor current set to {value}")
    def set_motor_position(self, value: float) -> None:
        print(f"Motor position set to {value}")
    def set_motor_torque(self, value: float) -> None:
        print(f"Motor torque set to {value}")
    def set_output_torque(self, value: float) -> None:
        print(f"Output torque set to {value}")
    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        print("Current gains set")
    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        print("Position gains set")
    def _set_impedance_gains(self, k: float, b: float) -> None:
        print("Impedance gains set")
    def home(self) -> None:
        print("Homed")
    @property
    def motor_position(self) -> float:
        return 100.0
    @property
    def motor_velocity(self) -> float:
        return 10.0
    @property
    def motor_voltage(self) -> float:
        return 24.0
    @property
    def motor_current(self) -> float:
        return 0.5
    @property
    def motor_torque(self) -> float:
        return 2.0
    @property
    def case_temperature(self) -> float:
        return 70.0
    @property
    def winding_temperature(self) -> float:
        return 90.0
