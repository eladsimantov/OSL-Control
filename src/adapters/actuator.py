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
from typing import Any, Optional

# Import OSL components
from opensourceleg.actuators import ActuatorBase, CONTROL_MODES, MOTOR_CONSTANTS, CONTROL_MODE_CONFIGS, ControlModeConfig
from opensourceleg.logging import LOGGER

# Import our Custom Driver and interfaces code
project_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, project_path)
from drivers.odrive_can import ODriveMotor, ODriveCAN

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

        # 2. Instantaneous Actuator States
        self.q = 0.0          # Current position (rad)
        self.qdot = 0.0       # Current velocity (rad/s)
        self.q_des = 0.0      # Desired position (rad)
        self.qdot_des = 0.0   # Desired velocity (rad/s)
        
        self.kp = 0.0         # Proportional gain / Stiffness (Kp / K)
        self.kd = 0.0         # Derivative gain / Damping (Kd / B)
        self.pos_eq = 0.0     # Equilibrium position (same as q_des in impedance mode)
        
        self._motor_torque = 0.0
        self._motor_current = 0.0

        self._is_homed = False
        self._is_streaming = False

        # 3. Initialize Hardware Driver if online
        if not self.is_offline:
            if can_interface is None:
                raise ValueError("can_interface must be provided when offline=False")
            self.driver = ODriveMotor(can_interface, gear_ratio=gear_ratio, name=tag)
        else:
            self.driver = None

    @property
    def _CONTROL_MODE_CONFIGS(self):
        return CONTROL_MODE_CONFIGS()

    def set_control_mode(self, mode: CONTROL_MODES) -> None:
        super().set_control_mode(mode)
        if self.is_offline:
            LOGGER.info(f"[{self.tag}] Switched to {mode.name} mode (offline).")
            return

        # Map to ODrive controller mode
        if mode == CONTROL_MODES.POSITION:
            self.driver.position_control()
        elif mode == CONTROL_MODES.VELOCITY:
            self.driver.velocity_control()
        elif mode == CONTROL_MODES.TORQUE:
            self.driver.torque_control()
        elif mode == CONTROL_MODES.CURRENT:
            self.driver.torque_control()
        elif mode == CONTROL_MODES.IMPEDANCE:
            self.driver.torque_control()
        elif mode == CONTROL_MODES.IDLE:
            self.driver.idle()

    def start(self):
        if self.is_offline:
            LOGGER.info(f"[{self.tag}] Closed loop active (offline).")
            self._is_streaming = True
            return
        
        self.driver.closed_loop()
        self._is_streaming = True

    def stop(self):
        if self.is_offline:
            LOGGER.info(f"[{self.tag}] Motor idled (offline).")
            self._is_streaming = False
            return
        
        self.driver.idle()
        self._is_streaming = False

    def update(self):
        if not self.is_offline:
            # 1. Read instantaneous states from hardware driver
            mname, sname, turns = self.driver._read_current_turns()
            if turns is not None:
                self.q = float(turns * 2.0 * math.pi)
                
            vel_deg_s = self.driver.get_velocity()
            if vel_deg_s is not None:
                self.qdot = float(math.radians(vel_deg_s))
                    
            curr = self.driver.read_current()
            if curr is not None:
                self._motor_current = float(curr)
                self._motor_torque = self._motor_current * self.MOTOR_CONSTANTS.NM_PER_AMP

            # 2. If in IMPEDANCE mode, compute torque and send it to ODrive in real-time
            if self.mode == CONTROL_MODES.IMPEDANCE:
                spring_err = self.q - self.pos_eq
                self._motor_torque = - self.kp * spring_err - self.kd * self.qdot
                self._motor_current = self._motor_torque / self.MOTOR_CONSTANTS.NM_PER_AMP
                self.driver.torque_nm(self._motor_torque)
        else:
            # 2. Simulate states for offline mock
            dt = 1.0 / self.frequency
            
            if self.mode == CONTROL_MODES.POSITION:
                self.q = self.q_des
                self.qdot = 0.0
                self._motor_torque = 0.0

            elif self.mode == CONTROL_MODES.VELOCITY:
                self.qdot = self.qdot_des
                self.q += self.qdot * dt
                self._motor_torque = 0.0

            elif self.mode == CONTROL_MODES.TORQUE:
                self._motor_current = self._motor_torque / self.MOTOR_CONSTANTS.NM_PER_AMP
                # Accelerate simulated rotor inertia
                acceleration = self._motor_torque / 0.05
                self.qdot += acceleration * dt
                self.qdot = max(-15.0, min(15.0, self.qdot))
                self.q += self.qdot * dt

            elif self.mode == CONTROL_MODES.CURRENT:
                self._motor_torque = self._motor_current * self.MOTOR_CONSTANTS.NM_PER_AMP
                # Accelerate simulated rotor inertia
                acceleration = self._motor_torque / 0.05
                self.qdot += acceleration * dt
                self.qdot = max(-15.0, min(15.0, self.qdot))
                self.q += self.qdot * dt

            elif self.mode == CONTROL_MODES.IMPEDANCE:
                # Spring-damper torque: T = - Kp * (q - pos_eq) - Kd * qdot
                spring_err = self.q - self.pos_eq
                self._motor_torque = - self.kp * spring_err - self.kd * self.qdot
                self._motor_current = self._motor_torque / self.MOTOR_CONSTANTS.NM_PER_AMP
                
                # Accelerate simulated rotor inertia
                acceleration = self._motor_torque / 0.05
                self.qdot += acceleration * dt
                self.qdot = max(-15.0, min(15.0, self.qdot))
                self.q += self.qdot * dt

            elif self.mode == CONTROL_MODES.IDLE:
                self.qdot *= 0.8
                self.q += self.qdot * dt
                self._motor_torque = 0.0
                self._motor_current = 0.0

    def set_motor_position(self, value: float) -> None:
        """Set target position in radians (absolute)."""
        self.q_des = value
        if not self.is_offline:
            deg = math.degrees(value)
            self.driver.position_deg(deg, absolute=True)

    def set_motor_velocity(self, value: float) -> None:
        """Set target velocity in radians per second."""
        self.qdot_des = value
        if not self.is_offline:
            deg_s = math.degrees(value)
            self.driver.velocity_deg_s(deg_s)

    def set_motor_torque(self, value: float) -> None:
        """Set target torque in Nm."""
        self._motor_torque = value
        self._motor_current = value / self.MOTOR_CONSTANTS.NM_PER_AMP
        if not self.is_offline:
            self.driver.torque_nm(value)

    def set_motor_current(self, value: float) -> None:
        """Set target current in Amps."""
        self._motor_current = value
        self._motor_torque = value * self.MOTOR_CONSTANTS.NM_PER_AMP
        if not self.is_offline:
            self.driver.torque_nm(self._motor_torque)

    def set_motor_voltage(self, value: float) -> None:
        pass

    def set_output_torque(self, value: float) -> None:
        self.set_motor_torque(value / self.gear_ratio)

    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        pass

    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        pass

    def _set_impedance_gains(self, k: float, b: float) -> None:
        self.kp = k
        self.kd = b

    def home(self) -> None:
        if self.is_offline:
            self.q = 0.0
            self._is_homed = True
            LOGGER.info(f"[{self.tag}] Homing complete (offline).")
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
            LOGGER.info(f"[{self.tag}] Homed ODrive to: {self.driver._home_deg:.2f} degrees")
        else:
            self.driver._home_deg = 0.0
            LOGGER.warning(f"[{self.tag}] Could not read encoder turns, default homing to 0.0")
        
        self._is_homed = True

    def calibrate(self, wait_time: float = 18.0) -> None:
        """Run the ODrive full calibration sequence."""
        if self.is_offline:
            LOGGER.info(f"[{self.tag}] Calibration skipped (offline).")
            return
        self.driver.calibrate(wait_time)

    def set_current_limit(self, max_current: float, max_velocity: float = 5.0) -> None:
        """Set the ODrive safety current and velocity limits."""
        if self.is_offline:
            LOGGER.debug(f"[{self.tag}] Limits not applied (offline).")
            return
        self.driver.set_limit_current(max_current, max_velocity)

    @property
    def motor_position(self) -> float:
        """Returns the motor position in radians."""
        return self.q

    @property
    def motor_velocity(self) -> float:
        """Returns the motor velocity in radians per second."""
        return self.qdot

    @property
    def motor_voltage(self) -> float:
        return 24.0

    @property
    def motor_current(self) -> float:
        return self._motor_current

    @property
    def motor_torque(self) -> float:
        return self._motor_torque

    @property
    def case_temperature(self) -> float:
        return 35.0

    @property
    def winding_temperature(self) -> float:
        return 40.0
