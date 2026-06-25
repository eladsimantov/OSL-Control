import os
import sys
import time
import math
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
from src.enabletools.control_laws import cvp_controller

# Helper for non-blocking console input
import select
def get_keypress():
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    if dr:
        return sys.stdin.readline().strip()
    return None

# ------------- TUNABLE FSM PARAMETERS ---------------- #
GEAR_RATIO = 41.5
FREQUENCY = 100.0
BODY_WEIGHT = 25 * 9.8  # 245.0 N

# STATE 1: EARLY STANCE
KNEE_K_ESTANCE = 0.025
KNEE_B_ESTANCE = 0.0006
KNEE_THETA_ESTANCE = 0.0
LOAD_LSTANCE: float = 1.0 * BODY_WEIGHT * 0.25
THIGH_ELEVATION_ESTANCE_TO_LSTANCE: float = 0.00

# STATE 2: LATE STANCE
KNEE_K_LSTANCE = 0.02
KNEE_B_LSTANCE = 0.00024
KNEE_THETA_LSTANCE = 8.0
LOAD_ESWING: float = 1.0 * BODY_WEIGHT * 0.1
THIGH_ELEVATION_LSTANCE_TO_ESWING: float = 0.00

# STATE 3: EARLY SWING
KNEE_K_ESWING = 0.008
KNEE_B_ESWING = 0.000012
KNEE_THETA_ESWING = 60.0
KNEE_THETA_ESWING_TO_LSWING = 50.0  # degrees at least reached for late swing
KNEE_DTHETA_ESWING_TO_LSWING = 170.0  # deg/s (~3.0 rad/s)

# STATE 4: LATE SWING
KNEE_K_LSWING = 0.008
KNEE_B_LSWING = 0.00072
KNEE_THETA_LSWING = 5.0
LOAD_ESTANCE: float = 1.0 * BODY_WEIGHT * 0.15
KNEE_THETA_LSWING_TO_ESTANCE = 30.0  # degrees

# ---------------------------------------------------- #

def create_simple_walking_fsm() -> StateMachine:
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

    # Transition criteria functions receiving kwargs directly
    def estance_to_lstance(fz: float, thigh_elevation: float, foot_elevation: float) -> bool:
        """
        Transition from early stance to late stance when the loadcell
        reads a force greater than a threshold (LOAD_LSTANCE) and
        the thigh elevation angle is below a threshold (THIGH_ELEVATION_ESTANCE_TO_LSTANCE).
        """
        return bool(fz > LOAD_LSTANCE and thigh_elevation < THIGH_ELEVATION_ESTANCE_TO_LSTANCE)

    def lstance_to_eswing(fz: float, thigh_elevation: float) -> bool:
        """
        Transition from late stance to early swing when the loadcell
        reads a force less than a threshold (LOAD_ESWING) and
        the thigh elevation angle is below a threshold (THIGH_ELEVATION_LSTANCE_TO_ESWING).
        """
        return bool(fz < LOAD_ESWING and thigh_elevation < THIGH_ELEVATION_LSTANCE_TO_ESWING)

    def eswing_to_lswing(knee_pos: float, knee_vel: float) -> bool:
        return bool(
            knee_pos > KNEE_THETA_ESWING_TO_LSWING
            and knee_vel < KNEE_DTHETA_ESWING_TO_LSWING
        )

    def lswing_to_estance(fz: float, knee_pos: float) -> bool:
        """
        Transition from late swing to early stance (heel strike) if
        the loadcell reads a force greater than a threshold (LOAD_ESTANCE) or
        the knee position is below a threshold (KNEE_THETA_LSWING_TO_ESTANCE).
        """
        return bool(fz > LOAD_ESTANCE or knee_pos < KNEE_THETA_LSWING_TO_ESTANCE)

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

    # LOGGER.set_stream_level(LogLevel.DEBUG)
    LOGGER.set_stream_level(LogLevel.ERROR)
    LOGGER.info("--- Starting Knee-Only Walking FSM (Direct Data Flow) ---")

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

    loadcell = SRILoadCell_M8123B2(
        tag="loadcell",
        channel=can_interface,
        offline=False
    )

    os.system(f"sudo rfkill block bluetooth") 
    os.system(f"sudo rfkill unblock bluetooth") # restart Bluetooth on Pi

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
    loadcell.start()
    loadcell.calibrate()

    thigh_imu.start()
    thigh_imu.calibrate()
    time.sleep(3)
    foot_imu.start()
    foot_imu.calibrate()

    # ODrive closed-loop setup
    knee_motor.idle()
    time.sleep(0.2)  # Allow time for motor to idle
    knee_motor.set_limit_current(10, 30)
    knee_motor.closed_loop()
    time.sleep(0.2)  # Allow time for motor to enter closed-loop
    knee_motor.torque_control()

    osl_fsm = create_simple_walking_fsm()

    clock = SoftRealtimeLoop(dt=1.0 / FREQUENCY, report=True)

    try:
        with osl_fsm:
            LOGGER.info("Starting real-time FSM loop... Press Ctrl+C to stop cleanly.")
            for t in clock:
                # Stop if max duration is reached
                if max_duration is not None and t >= max_duration:
                    LOGGER.info(f"Max duration of {max_duration}s reached. Stopping FSM loop.")
                    clock.stop()
                    break

                # Update sensors
                loadcell.update()
                thigh_imu.update()
                foot_imu.update()

                # Read parameters directly
                fz = loadcell.fz
                thigh_elevation = thigh_imu.euler_y
                foot_elevation = foot_imu.euler_y
                knee_pos = knee_motor.get_position()
                knee_vel = knee_motor.get_velocity()
                

                # Evaluate state transitions via kwargs
                osl_fsm.update(
                    fz=fz,
                    thigh_elevation=thigh_elevation,
                    foot_elevation=foot_elevation,
                    knee_pos=knee_pos,
                    knee_vel=knee_vel
                )

                gamma = 0.50 # apply 50% CVP controller versus baseline impedance controller
                V = np.array([[0.1617,  0.9616, -0.2220],
                              [0.6546,  0.0638,  0.7533],
                              [0.7385, -0.2671, -0.6191]])
                mu = np.array([6.4536, -16.3486, 77.4091])
                shank_cvp = cvp_controller(thigh_elevation, foot_elevation + 90, V, mu) 
                knee_CVP = thigh_elevation - shank_cvp # The simplest 2D transformation in the sagittal plane.
                knee_effective_eq = osl_fsm.current_state.knee_theta*gamma + knee_CVP * (1-gamma)

                # Set knee impedance target based on current FSM active state
                knee_motor.set_impedance(
                    kp=osl_fsm.current_state.knee_stiffness,
                    kd=osl_fsm.current_state.knee_damping,
                    deg_eq=knee_effective_eq
                )

                # Print telemetry periodically
                if int(t * FREQUENCY) % 20 == 0:
                    print(f"\r t={t:5.2f}s | State: {osl_fsm.current_state.name:11} | Fz: {fz:6.1f}N | "
                          f"Knee Pos: {knee_pos:6.1f}° | Thigh: {thigh_elevation:5.1f}° | Foot: {foot_elevation:5.1f}° | "
                          f"CVP-Eq distance: {knee_CVP-knee_pos:6.1f}° | ", end='', flush=True)

    except KeyboardInterrupt:
        LOGGER.info("KeyboardInterrupt detected. Shutting down cleanly...")
    finally:
        LOGGER.info("\nCleaning up and idling motor...")
        try:
            knee_motor.idle()
            time.sleep(0.2)  # Give CAN bus time to transmit the idle command before shutdown
        except Exception:
            pass
        try:
            loadcell.stop()
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
