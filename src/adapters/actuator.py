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
    def __init__(
        self, 
        can_interface: Optional[ODriveCAN], 
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
        offline: bool = False,
        **kwargs
    ):

        # 1. Initialize Base Class
        super().__init__(
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=motor_constants,
            frequency=frequency,
            offline=offline,
            **kwargs
        )

        self._mock_pos = 0.0
        self._is_homed = False
        self._is_streaming = False

        # 2. Initialize Hardware Driver if online
        if not self.is_offline:
            if can_interface is None:
                raise ValueError("can_interface must be provided when offline=False")
            self.driver = ODriveMotor(can_interface, gear_ratio=gear_ratio, name=tag)
        else:
            self.driver = None
        
        
    @property
    def _CONTROL_MODE_CONFIGS(self):
        return CONTROL_MODE_CONFIGS()
    def start(self):
        if self.is_offline:
            print(f"[Offline Mock {self.tag}] Closed loop active.")
            self._is_streaming = True
            return
        
        self.driver.closed_loop()
        self._is_streaming = True

    def stop(self):
        if self.is_offline:
            print(f"[Offline Mock {self.tag}] Motor idled.")
            self._is_streaming = False
            return
        
        self.driver.idle()
        self._is_streaming = False

    def update(self):
        pass

    def set_motor_position(self, value: float) -> None:
        """Set target position in radians (absolute)."""
        if self.is_offline:
            self._mock_pos = value
            print(f"[Offline Mock {self.tag}] Target position: {math.degrees(value):.2f}°")
            return
        
        # Convert radians to output degrees
        deg = math.degrees(value)
        self.driver.position_deg(deg, absolute=True)

    def set_motor_torque(self, value: float) -> None:
        """Set target torque in Nm."""
        if self.is_offline:
            print(f"[Offline Mock {self.tag}] Target torque: {value:.2f} Nm")
            return
        
        self.driver.torque_nm(value)

    def set_motor_current(self, value: float) -> None:
        """Set max current limit in Amps."""
        if self.is_offline:
            print(f"[Offline Mock {self.tag}] Target current limit: {value:.2f} A")
            return
        
        self.driver.set_limit_current(value)

    def set_motor_voltage(self, value: float) -> None:
        pass

    def set_output_torque(self, value: float) -> None:
        self.set_motor_torque(value / self.gear_ratio)

    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        pass

    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        pass

    def _set_impedance_gains(self, k: float, b: float) -> None:
        pass

    def home(self) -> None:
        if self.is_offline:
            self._mock_pos = 0.0
            self._is_homed = True
            print(f"[Offline Mock {self.tag}] Homing complete.")
            return

        # Perform zero-jump homing
        mname, sname, turns = self.driver._read_current_turns()
        if turns is not None:
            self.driver.can.send_dbc("Axis0_Set_Input_Pos", {
                "Input_Pos": float(turns),
                "Vel_FF": 0.0,
                "Torque_FF": 0.0
            })
            self.driver._home_deg = (turns / self.driver.gear_ratio) * 360.0
            print(f"[{self.tag}] Homed ODrive to: {self.driver._home_deg:.2f} degrees")
        else:
            self.driver._home_deg = 0.0
            print(f"[{self.tag}] Warning: Could not read encoder turns, default homing to 0.0")
        
        self._is_homed = True

    @property
    def motor_position(self) -> float:
        """Returns the motor position in radians."""
        if self.is_offline:
            return self._mock_pos
        
        mname, sname, turns = self.driver._read_current_turns()
        if turns is None:
            turns = 0.0
        return float(turns * 2.0 * math.pi)

    @property
    def motor_velocity(self) -> float:
        """Returns the motor velocity in radians per second."""
        if self.is_offline:
            return 0.0
        
        candidates = ["vel_estimate", "velocity", "encoder_vel", "velocity_estimate"]
        mname, sname, val = self.driver.can.find_signal(self.driver.can.axisID, candidates)
        if val is None:
            return 0.0
        try:
            turns_s = float(val)
            return float(turns_s * 2.0 * math.pi)
        except Exception:
            return 0.0

    @property
    def motor_voltage(self) -> float:
        return 24.0

    @property
    def motor_current(self) -> float:
        if self.is_offline:
            return 0.0
        current = self.driver.read_current()
        return float(current) if current is not None else 0.0

    @property
    def motor_torque(self) -> float:
        return self.motor_current * self.MOTOR_CONSTANTS.NM_PER_AMP

    @property
    def case_temperature(self) -> float:
        return 35.0

    @property
    def winding_temperature(self) -> float:
        return 40.0

