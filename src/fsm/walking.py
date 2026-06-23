import os
import sys
import time
import math
from typing import Optional
import numpy as np

# Include project path for imports
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if project_path not in sys.path:
    sys.path.insert(0, project_path)

from opensourceleg.control.fsm import State, StateMachine
from opensourceleg.utilities import SoftRealtimeLoop
from opensourceleg.logging import LOGGER, LogLevel
from src.drivers.odrive_can import ODriveCAN, ODriveMotor
from src.adapters.loadcell import SRILoadCell_M8123B2
from src.adapters.imu import WitMotionIMUAdapter

# ------------- COMPATIBILITY WRAPPERS FOR OSL API ---------------- #

class KneeWrapper:
    def __init__(self, motor: ODriveMotor):
        self.motor = motor
        self.k = 0.02
        self.b = 0.0006
        self.desired_position_deg = 0.0

    @property
    def output_position(self) -> float:
        """Knee position in radians."""
        pos = self.motor.get_position()
        return np.deg2rad(pos) if pos is not None else 0.0

    @property
    def output_velocity(self) -> float:
        """Knee velocity in rad/s."""
        vel = self.motor.get_velocity()
        return np.deg2rad(vel) if vel is not None else 0.0

    def set_output_impedance(self, k: float = None, b: float = None):
        """Set virtual impedance stiffness and damping."""
        if k is not None:
            self.k = k
        if b is not None:
            self.b = b
        self.motor.set_impedance(
            kp=self.k, 
            kd=self.b, 
            deg_eq=self.desired_position_deg
        )

    def set_output_position(self, rad: float):
        """Set target equilibrium position in radians."""
        self.desired_position_deg = float(np.rad2deg(rad))
        self.motor.set_impedance(
            kp=self.k, 
            kd=self.b, 
            deg_eq=self.desired_position_deg
        )

    def closed_loop(self):
        self.motor.closed_loop()

    def idle(self):
        self.motor.idle()

    def torque_control(self):
        self.motor.torque_control()

    def set_limit_current(self, max_current, max_velocity):
        self.motor.set_limit_current(max_current, max_velocity)


class LoadcellWrapper:
    def __init__(self, loadcell: SRILoadCell_M8123B2):
        self.loadcell = loadcell

    @property
    def fz(self) -> float:
        """Force in Newton along the z-axis."""
        return self.loadcell.fz

    def reset(self):
        pass

    def calibrate(self):
        self.loadcell.calibrate()


class OpenSourceLeg:
    def __init__(self, tag: str, knee_motor: ODriveMotor, loadcell: SRILoadCell_M8123B2,
                 thigh_imu: Optional[WitMotionIMUAdapter] = None, foot_imu: Optional[WitMotionIMUAdapter] = None):
        self.tag = tag
        self.knee = KneeWrapper(knee_motor)
        self.loadcell = LoadcellWrapper(loadcell)
        self.thigh_imu = thigh_imu
        self.foot_imu = foot_imu

        # Compatibility dictionary lookup support
        self.sensors = {
            "joint_encoder_knee": self.knee,
            "loadcell": self.loadcell
        }

    def update(self):
        self.loadcell.update()
        if self.thigh_imu:
            self.thigh_imu.update()
        if self.foot_imu:
            self.foot_imu.update()

    def home(self, callbacks=None):
        if callbacks:
            for name, cb in callbacks.items():
                if cb:
                    cb()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

# ------------- TUNABLE FSM PARAMETERS ---------------- #
GEAR_RATIO = 40.0
FREQUENCY = 100.0
BODY_WEIGHT = 10 * 9.8  # 98.0 N

# STATE 1: EARLY STANCE
KNEE_K_ESTANCE = 0.02
KNEE_B_ESTANCE = 0.0006
KNEE_THETA_ESTANCE = 5.0
LOAD_LSTANCE: float = 1.0 * BODY_WEIGHT * 0.25

# STATE 2: LATE STANCE
KNEE_K_LSTANCE = 0.02
KNEE_B_LSTANCE = 0.0006
KNEE_THETA_LSTANCE = 8.0
LOAD_ESWING: float = 1.0 * BODY_WEIGHT * 0.15

# STATE 3: EARLY SWING
KNEE_K_ESWING = 0.02
KNEE_B_ESWING = 0.0006
KNEE_THETA_ESWING = 60.0
KNEE_THETA_ESWING_TO_LSWING = np.deg2rad(50.0)
KNEE_DTHETA_ESWING_TO_LSWING = np.deg2rad(170.0)  # rad/s (~3.0 rad/s converted from original)

# STATE 4: LATE SWING
KNEE_K_LSWING = 0.02
KNEE_B_LSWING = 0.0006
KNEE_THETA_LSWING = 5.0
LOAD_ESTANCE: float = 1.0 * BODY_WEIGHT * 0.4
KNEE_THETA_LSWING_TO_ESTANCE = np.deg2rad(30.0)

# ---------------------------------------------------- #

def create_simple_walking_fsm(osl: OpenSourceLeg) -> StateMachine:
    e_stance = State(
        name="e_stance",
        knee_theta=KNEE_THETA_ESTANCE,
        knee_stiffness=KNEE_K_ESTANCE,
        knee_damping=KNEE_B_ESTANCE,
    )

    l_stance = State(
        name="l_stance",
        knee_theta=KNEE_THETA_LSTANCE,
        knee_stiffness=KNEE_K_LSTANCE,
        knee_damping=KNEE_B_LSTANCE,
    )

    e_swing = State(
        name="e_swing",
        knee_theta=KNEE_THETA_ESWING,
        knee_stiffness=KNEE_K_ESWING,
        knee_damping=KNEE_B_ESWING,
    )

    l_swing = State(
        name="l_swing",
        knee_theta=KNEE_THETA_LSWING,
        knee_stiffness=KNEE_K_LSWING,
        knee_damping=KNEE_B_LSWING,
    )

    def estance_to_lstance(osl: OpenSourceLeg) -> bool:
        # Get sensor readings
        fz = osl.loadcell.fz
        thigh_pitch = osl.thigh_imu.euler_y
        foot_pitch = osl.foot_imu.euler_y
        
        # Return True only if all conditions are met
        return bool(fz > 30.0 and thigh_pitch > 5.0 and foot_pitch < -2.0)

    def lstance_to_eswing(osl: OpenSourceLeg) -> bool:
        if osl.loadcell is None:
            raise ValueError("Loadcell is not connected")
        return bool(osl.loadcell.fz < LOAD_ESWING)

    def eswing_to_lswing(osl: OpenSourceLeg) -> bool:
        if osl.knee is None:
            raise ValueError("Knee is not connected")
        return bool(
            osl.knee.output_position > KNEE_THETA_ESWING_TO_LSWING
            and osl.knee.output_velocity < KNEE_DTHETA_ESWING_TO_LSWING
        )

    def lswing_to_estance(osl: OpenSourceLeg) -> bool:
        if osl.knee is None:
            raise ValueError("Knee is not connected")
        if osl.loadcell is None:
            raise ValueError("Loadcell is not connected")
        return bool(osl.loadcell.fz > LOAD_ESTANCE or osl.knee.output_position < KNEE_THETA_LSWING_TO_ESTANCE)

    fsm = StateMachine(
        states=[e_stance, l_stance, e_swing, l_swing],
        initial_state_name="e_stance"
    )

    fsm.add_transition(
        source=e_stance,
        destination=l_stance,
        event_name="foot_flat",
        criteria=estance_to_lstance,
    )
    fsm.add_transition(
        source=l_stance,
        destination=e_swing,
        event_name="heel_off",
        criteria=lstance_to_eswing,
    )
    fsm.add_transition(
        source=e_swing,
        destination=l_swing,
        event_name="toe_off",
        criteria=eswing_to_lswing,
    )
    fsm.add_transition(
        source=l_swing,
        destination=e_stance,
        event_name="heel_strike",
        criteria=lswing_to_estance,
    )
    return fsm


def run_walking_fsm(max_duration: float = None):
    # Setup parameters
    can_interface = "can0"
    bitrate = 1000000
    node_id_odrive = 0
    thigh_imu_mac = "EF:D5:AC:1A:0D:21"
    foot_imu_mac = "EC:8E:70:CE:63:24"

    LOGGER.set_stream_level(LogLevel.DEBUG)
    LOGGER.info("--- Starting Knee-Only Walking FSM ---")

    # 1. Bring up CAN Link
    LOGGER.info(f"Setting up CAN link {can_interface} at {bitrate} bps...")
    os.system(f"sudo ip link set {can_interface} down")
    os.system(f"sudo ip link set {can_interface} up type can bitrate {bitrate} sample-point 0.750")
    os.system(f"sudo ip link set {can_interface} txqueuelen 1000")

    # 2. Initialize CAN bus and adapters
    LOGGER.info(f"Initializing ODrive CAN on {can_interface}...")
    dir_path = os.path.dirname(os.path.abspath(__file__))
    dbc_path = os.path.abspath(os.path.join(dir_path, "..", "drivers", "odrive-cansimple.dbc"))
    can_bus = ODriveCAN(bus_name=can_interface, node_id=node_id_odrive, dbc_path=dbc_path)

    # Initialize adapters
    knee_motor = ODriveMotor(can_interface=can_bus, name="knee", gear_ratio=GEAR_RATIO)

    loadcell_adapter = SRILoadCell_M8123B2(
        tag="loadcell",
        channel=can_interface,
        offline=False
    )

    thigh_imu = WitMotionIMUAdapter(
        tag="Thigh IMU",
        mac_address=thigh_imu_mac,
        connection_type="ble",
        offline=False
    )
    
    foot_imu = WitMotionIMUAdapter(
        tag="Foot IMU",
        mac_address=foot_imu_mac,
        connection_type="ble",
        offline=False
    )

    # Start sensor streams
    loadcell_adapter.start()
    thigh_imu.start()
    foot_imu.start()

    # ODrive closed-loop setup
    knee_motor.idle()
    knee_motor.set_limit_current(10, 30)
    knee_motor.closed_loop()
    knee_motor.torque_control()

    # Wrap inside OpenSourceLeg compatibility class
    osl = OpenSourceLeg(
        tag="osl",
        knee_motor=knee_motor,
        loadcell=loadcell_adapter,
        thigh_imu=thigh_imu,
        foot_imu=foot_imu
    )

    osl_fsm = create_simple_walking_fsm(osl)

    # Homing callbacks
    def knee_homing_complete():
        LOGGER.info("Knee homing complete!")

    def ankle_homing_complete():
        LOGGER.info("Ankle homing complete!")

    callbacks = {"knee": knee_homing_complete, "ankle": ankle_homing_complete}

    clock = SoftRealtimeLoop(dt=1.0 / FREQUENCY, report=True)

    try:
        with osl, osl_fsm:
            osl.update()
            osl.home(callbacks=callbacks)
            
            # Initial setup of closed loop settings on wrappers
            osl.knee.closed_loop()
            osl.knee.torque_control()
            osl.loadcell.calibrate()

            LOGGER.info("Starting real-time FSM loop... Press Ctrl+C to stop cleanly.")
            for t in clock:
                # Stop if max duration is reached
                if max_duration is not None and t >= max_duration:
                    LOGGER.info(f"Max duration of {max_duration}s reached. Stopping FSM loop.")
                    clock.stop()
                    break

                # Read sensors (updates underlying adapters)
                osl.update()

                # Evaluate state transitions
                osl_fsm.update(osl=osl)

                # Set knee impedance target based on FSM active state
                osl.knee.set_output_impedance(
                    k=osl_fsm.current_state.knee_stiffness,
                    b=osl_fsm.current_state.knee_damping,
                )
                osl.knee.set_output_position(np.deg2rad(osl_fsm.current_state.knee_theta))

                # Print telemetry periodically
                if int(t * FREQUENCY) % 20 == 0:
                    knee_pos_deg = np.rad2deg(osl.knee.output_position)
                    print(f"\r t={t:5.2f}s | State: {osl_fsm.current_state.name:11} | Fz: {osl.loadcell.fz:6.1f}N | "
                          f"Knee Pos: {knee_pos_deg:6.1f}°", end='', flush=True)

    except KeyboardInterrupt:
        LOGGER.info("KeyboardInterrupt detected. Shutting down cleanly...")
    finally:
        LOGGER.info("\nCleaning up and idling motor...")
        try:
            knee_motor.idle()
        except Exception:
            pass
        try:
            loadcell_adapter.stop()
        except Exception:
            pass
        try:
            thigh_imu.stop()
        except Exception:
            pass
        try:
            foot_imu.stop()
        except Exception:
            pass
        os.system(f"sudo ip link set {can_interface} down")
        LOGGER.close()
        clock.stop()
        LOGGER.info("Shutdown complete.")

if __name__ == "__main__":
    run_walking_fsm()
