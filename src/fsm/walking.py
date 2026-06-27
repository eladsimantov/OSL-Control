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

try:
    from opensourceleg.control.compiled_controller import CompiledController
    HAS_PHASE_VAR = True
    print("Compiled controlller library loaded")
except ImportError as e:
    HAS_PHASE_VAR = False
    print(e)
    print("No compiled controlller library loaded")


# Helper for non-blocking console input
import select
def get_keypress():
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    if dr:
        return sys.stdin.readline().strip()
    return None

import json

# ------------- LOAD CONFIGURATION ---------------- #
config_path = os.path.join(os.path.dirname(__file__), "config_walking.json")
try:
    with open(config_path, "r") as f:
        config = json.load(f)
except Exception as e:
    print(f"Error loading configuration from {config_path}: {e}")
    sys.exit(1)

GEAR_RATIO = config.get("GEAR_RATIO", 41.5)
FREQUENCY = config.get("FREQUENCY", 100.0)
BODY_WEIGHT = config.get("BODY_WEIGHT", 245.0)
WALKING_SPEED = config.get("WALKING_SPEED", 1.0)
WALKING_INCLINE = config.get("WALKING_INCLINE", 0.0)
PHASE_VAR_LIB_PATH = config.get("PHASE_VAR_LIB_PATH", None)

# STATE 1: EARLY STANCE
KNEE_K_ESTANCE = config.get("KNEE_K_ESTANCE", 0.025)
KNEE_B_ESTANCE = config.get("KNEE_B_ESTANCE", 0.0006)
KNEE_THETA_ESTANCE = config.get("KNEE_THETA_ESTANCE", 0.0)
LOAD_LSTANCE = config.get("LOAD_LSTANCE", 61.25)
THIGH_ELEVATION_ESTANCE_TO_LSTANCE = config.get("THIGH_ELEVATION_ESTANCE_TO_LSTANCE", 0.00)

# STATE 2: LATE STANCE
KNEE_K_LSTANCE = config.get("KNEE_K_LSTANCE", 0.02)
KNEE_B_LSTANCE = config.get("KNEE_B_LSTANCE", 0.00024)
KNEE_THETA_LSTANCE = config.get("KNEE_THETA_LSTANCE", 8.0)
LOAD_ESWING = config.get("LOAD_ESWING", 24.5)
THIGH_ELEVATION_LSTANCE_TO_ESWING = config.get("THIGH_ELEVATION_LSTANCE_TO_ESWING", 0.00)

# STATE 3: EARLY SWING
KNEE_K_ESWING = config.get("KNEE_K_ESWING", 0.008)
KNEE_B_ESWING = config.get("KNEE_B_ESWING", 0.000012)
KNEE_THETA_ESWING = config.get("KNEE_THETA_ESWING", 60.0)
KNEE_THETA_ESWING_TO_LSWING = config.get("KNEE_THETA_ESWING_TO_LSWING", 50.0)
KNEE_DTHETA_ESWING_TO_LSWING = config.get("KNEE_DTHETA_ESWING_TO_LSWING", 170.0)

# STATE 4: LATE SWING
KNEE_K_LSWING = config.get("KNEE_K_LSWING", 0.008)
KNEE_B_LSWING = config.get("KNEE_B_LSWING", 0.00072)
KNEE_THETA_LSWING = config.get("KNEE_THETA_LSWING", 5.0)
LOAD_ESTANCE = config.get("LOAD_ESTANCE", 36.75)
KNEE_THETA_LSWING_TO_ESTANCE = config.get("KNEE_THETA_LSWING_TO_ESTANCE", 30.0)

# CVP controller parameters
gamma = config.get("baseline_versus_cvp_gamma", 1.0)
V = np.array(config.get("cvp_V_matrix", [
    [0.1617, 0.9616, -0.2220],
    [0.6546, 0.0638, 0.7533],
    [0.7385, -0.2671, -0.6191]
]))
mu = np.array(config.get("cvp_means_vector", [6.4536, -16.3486, 77.4091]))

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
    can_interface = config.get("can_interface", "can0")
    bitrate = config.get("bitrate", 1000000)
    node_id_odrive = config.get("node_id_odrive", 0)
    thigh_imu_mac = config.get("thigh_imu_mac", "EF:D5:AC:1A:0D:21")
    foot_imu_mac = config.get("foot_imu_mac", "EC:8E:70:CE:63:24")

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

    phase_var = None
    if HAS_PHASE_VAR:
        try:
            if PHASE_VAR_LIB_PATH:
                lib_path = os.path.abspath(PHASE_VAR_LIB_PATH)
            else:
                fsm_dir = os.path.dirname(os.path.abspath(__file__))
                lib_path = os.path.abspath(os.path.join(fsm_dir, "..", "locolabtools", "phaseVar"))
            
            phase_var = CompiledController(
                library_name="LocolabPhaseVariable",
                library_path=lib_path,
                main_function_name="LocolabPhaseVariable",
                initialization_function_name="LocolabPhaseVariable_initialize",
                cleanup_function_name="LocolabPhaseVariable_terminate",
            )
            phase_var.define_inputs([
                ("thighAngle_deg", phase_var.types.c_double),
                ("thighVelocity_dps", phase_var.types.c_double),
                ("Fz", phase_var.types.c_double),
                ("time", phase_var.types.c_double),
                ("incline", phase_var.types.c_double),
                ("speed", phase_var.types.c_double),
            ])
            phase_var.define_outputs([
                ("phase", phase_var.types.c_double),
                ("stancePhase", phase_var.types.c_double),
                ("swingPhase", phase_var.types.c_double),
                ("state", phase_var.types.c_double),
            ])
            phase_var.inputs.speed = WALKING_SPEED
            phase_var.inputs.incline = WALKING_INCLINE
            LOGGER.info("LocoLab Phase Variable Controller successfully initialized.")
        except Exception as e:
            print(f"\n[Warning] Could not load LocolabPhaseVariable library: {e}. Running walking FSM without phase detection.")
            phase_var = None

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

                shank_cvp = cvp_controller(thigh_elevation, foot_elevation + 90, V, mu) 
                knee_CVP = thigh_elevation - shank_cvp # The simplest 2D transformation in the sagittal plane.
                knee_effective_eq = osl_fsm.current_state.knee_theta*gamma + knee_CVP * (1-gamma)

                # Set knee impedance target based on current FSM active state
                knee_motor.set_impedance(
                    kp=osl_fsm.current_state.knee_stiffness,
                    kd=osl_fsm.current_state.knee_damping,
                    deg_eq=knee_effective_eq
                )

                # Evaluate phase variable if available
                phase_val = 0.0
                stance_phase_val = 0.0
                swing_phase_val = 0.0
                state_val = 0.0

                if phase_var is not None:
                    try:
                        phase_var.inputs.thighAngle_deg = thigh_elevation
                        # Convert rad/s to deg/s for the Locolab algorithm
                        phase_var.inputs.thighVelocity_dps = math.degrees(thigh_imu.gyro_y)
                        phase_var.inputs.Fz = fz
                        phase_var.inputs.time = t
                        
                        outputs = phase_var.run()
                        phase_val = outputs.phase
                        stance_phase_val = outputs.stancePhase
                        swing_phase_val = outputs.swingPhase
                        state_val = outputs.state
                    except Exception as e:
                        pass

                # Print telemetry periodically
                if int(t * FREQUENCY) % 20 == 0:
                    phase_str = f" | Phase: {phase_val:5.2f} (Stance: {stance_phase_val:5.2f}, Swing: {swing_phase_val:5.2f}, State: {state_val:1.0f})" if phase_var is not None else ""
                    print(f"\r t={t:5.2f}s | State: {osl_fsm.current_state.name:11} | Fz: {fz:6.1f}N | "
                          f"Knee Pos: {knee_pos:6.1f}° | Thigh: {thigh_elevation:5.1f}° | Foot: {foot_elevation:5.1f}° | "
                          f"CVP-Eq distance: {knee_CVP-knee_pos:6.1f}°{phase_str} | ", end='', flush=True)

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
