import math
import logging
from typing import Any, Optional

# Import OSL components
from opensourceleg.actuators import ActuatorBase, CONTROL_MODES, CONTROL_MODE_CONFIGS, ControlModeConfig
from opensourceleg.constants import MOTOR_CONSTANTS

# Import your Custom Driver
from drivers.odrive_can import ODriveMotor

LOGGER = logging.getLogger(__name__)

class ODriveActuator(ActuatorBase):
    """
    Implementation of the OSL ActuatorBase for ODrive motors via CAN.
    """

    def __init__(
        self, 
        can_interface, 
        axis_id: int, 
        tag: str, 
        gear_ratio: float = 1.0, 
        motor_constants: MOTOR_CONSTANTS = None,
        frequency: int = 1000,
        **kwargs
    ):
        # 1. Setup Defaults if constants aren't provided
        if motor_constants is None:
            # Placeholder constants - adjust to your specific motor (e.g., T-Motor AK80-9)
            motor_constants = MOTOR_CONSTANTS(
                encoder_ticks_per_revolution=4096,
                torque_constant=0.02, # Nm/A
                terminal_resistance=0.1,
                terminal_inductance=0.001,
                max_case_temperature=80.0,
                max_winding_temperature=120.0
            )

        # 2. Initialize Base Class
        super().__init__(
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=motor_constants,
            frequency=frequency,
            **kwargs
        )

        # 3. Initialize Hardware Driver
        self.driver = ODriveMotor(can_interface, axis_id=axis_id, gear_ratio=gear_ratio, name=tag)
        
        # Internal Cache for sensors (updated in self.update())
        self._cache_pos = 0.0
        self._cache_vel = 0.0
        self._cache_current = 0.0
        self._cache_voltage = 24.0 # Default voltage assumption

        # Define Mode Configs (How to handle switching modes)
        self._mode_configs = CONTROL_MODE_CONFIGS(
            IDLE=ControlModeConfig(
                entry_callback=lambda a: self.driver.set_state(1), # 1 = Idle
                exit_callback=lambda a: None
            ),
            TORQUE=ControlModeConfig(
                entry_callback=lambda a: self.driver.set_state(8), # 8 = Closed Loop
                exit_callback=lambda a: None
            ),
            POSITION=ControlModeConfig(
                entry_callback=lambda a: self.driver.set_state(8), # 8 = Closed Loop
                exit_callback=lambda a: None
            ),
            CURRENT=ControlModeConfig(
                entry_callback=lambda a: self.driver.set_state(8),
                exit_callback=lambda a: None
            ),
            IMPEDANCE=ControlModeConfig(
                entry_callback=lambda a: self.driver.set_state(8),
                exit_callback=lambda a: None
            ),
            VOLTAGE=ControlModeConfig( # ODrive doesn't support direct voltage mode easily via CAN
                entry_callback=lambda a: self.driver.set_state(1), 
                exit_callback=lambda a: None
            )
        )

    # ----------------------------------------------------------------
    # MANDATORY PROPERTIES (From ActuatorBase)
    # ----------------------------------------------------------------

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return self._mode_configs

    @property
    def motor_position(self) -> float:
        """Returns motor position in Radians."""
        return self._cache_pos

    @property
    def motor_velocity(self) -> float:
        """Returns motor velocity in Radians/sec."""
        return self._cache_vel

    @property
    def motor_current(self) -> float:
        return self._cache_current

    @property
    def motor_torque(self) -> float:
        """Estimated torque based on current * torque_constant."""
        # Note: Or use self.driver.get_feedback()['torque'] if available
        return self._cache_current * self._MOTOR_CONSTANTS.TORQUE_CONSTANT

    @property
    def motor_voltage(self) -> float:
        return self._cache_voltage

    @property
    def case_temperature(self) -> float:
        # Placeholder: ODrive CAN simple protocol doesn't always send temp
        return 25.0 

    @property
    def winding_temperature(self) -> float:
        return 25.0

    # ----------------------------------------------------------------
    # CONTROL METHODS
    # ----------------------------------------------------------------

    def start(self) -> None:
        """Enable the motor."""
        print(f"[{self.tag}] Starting...")
        self.driver.set_state(8) # Closed Loop Control

    def stop(self) -> None:
        """Disable the motor."""
        print(f"[{self.tag}] Stopping...")
        self.driver.set_state(1) # Idle

    def update(self) -> None:
        """
        Called every loop cycle. 
        1. Reads latest CAN data from driver.
        2. Converts units (Degrees -> Radians).
        """
        # 1. Fetch data from low-level driver
        data = self.driver.get_feedback() # Expects dict {'pos': turns, 'vel': turns/s, 'current': A}
        
        if data:
            # 2. Convert Turns -> Radians
            # (2 * pi)
            self._cache_pos = data['pos'] * 2 * math.pi
            self._cache_vel = data['vel'] * 2 * math.pi
            self._cache_current = data['current']

    def home(self, **kwargs) -> None:
        """
        Trigger ODrive homing sequence.
        """
        print(f"[{self.tag}] Homing...")
        # State 3 is typically Homing in ODrive
        self.driver.set_state(3)

    # ----------------------------------------------------------------
    # ACTUATION METHODS (Rad/s, Nm, Amps)
    # ----------------------------------------------------------------

    def set_motor_position(self, value: float) -> None:
        """
        Args:
            value: Target position in Radians
        """
        # Convert Radians -> Turns
        target_turns = value / (2 * math.pi)
        self.driver.set_position(target_turns)

    def set_motor_torque(self, value: float) -> None:
        """
        Args:
            value: Target torque in Nm (at the motor)
        """
        # ODrive takes Nm directly if configured, or we convert to Amps?
        # Assuming your ODrive is configured for Torque Control in Nm:
        self.driver.set_torque(value)

    def set_output_torque(self, value: float) -> None:
        """
        Args:
            value: Target torque in Nm (at the output/joint)
        """
        # Motor Torque = Output Torque / Gear Ratio
        motor_torque = value / self.gear_ratio
        self.set_motor_torque(motor_torque)

    def set_motor_current(self, value: float) -> None:
        """
        Args:
            value: Target current in Amps
        """
        # Calculate Torque from Current and send as torque command
        # (Since standard ODrive CAN simplified usually exposes Input_Torque)
        torque_req = value * self._MOTOR_CONSTANTS.TORQUE_CONSTANT
        self.driver.set_torque(torque_req)

    # ----------------------------------------------------------------
    # GAIN SETTING (Optional / Advanced)
    # ----------------------------------------------------------------
    
    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        # Not implemented in simple CAN driver
        pass

    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        # Not implemented in simple CAN driver
        pass

    def _set_impedance_gains(self, k: float, b: float) -> None:
        # Not implemented in simple CAN driver
        pass

    def set_motor_voltage(self, value: float) -> None:
        pass