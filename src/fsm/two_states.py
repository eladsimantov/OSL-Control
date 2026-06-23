import time
import sys
import os
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
from src.enabletools import HoldImpedanceController, SwingTrajectoryController, CVPController

# Helper for non-blocking console input
import select
def get_keypress():
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    if dr:
        return sys.stdin.readline().strip()
    return None

# Default configurations - modify directly in this file
CONFIG = {
    # Hardware configurations
    "can_interface": "can0",
    "bitrate": 1000000,
    "node_id_odrive": 0,
    "gear_ratio": 40.0,
    "loop_hz": 100.0,
    "calibrate_time": 2.0,  # One-time motor configuration state time
    "thigh_imu_mac": "EF:D5:AC:1A:0D:21",
    "foot_imu_mac": "EC:8E:70:CE:63:24",

    # Gait parameters
    "unload_threshold": 20.0,      # N (Fz below this triggers Swing)
    "load_threshold": 60.0,        # N (Fz above this triggers Stance)
    "stance_knee_angle": 5.0,      # degrees (slight flexion for support)
    "peak_flexion_angle": 20.0,    # degrees (peak flexion during swing)
    "peak_flexion_time": 0.25,     # seconds into swing to reach peak flexion
    "max_swing_time": 0.55,        # seconds (safety swing timeout)
}

class TwoStatesFSM:
    def __init__(self, offline: bool = False, max_duration: float = None, 
                 stance_control: str = "impedance", stance_offset: float = 0.0):
        self.max_duration = max_duration
        self.config = CONFIG
        self.stance_control_type = stance_control.lower()
        self.stance_offset = stance_offset

        self.can_interface = self.config["can_interface"]
        self.bitrate = self.config["bitrate"]
        node_id_odrive = self.config["node_id_odrive"]
        gear_ratio = self.config["gear_ratio"]
        self.loop_hz = self.config["loop_hz"]
        self.calibrate_time = self.config["calibrate_time"]
        self.unload_threshold = self.config["unload_threshold"]
        self.load_threshold = self.config["load_threshold"]
        
        self.stance_knee_angle = self.config["stance_knee_angle"]
        self.peak_flexion_angle = self.config["peak_flexion_angle"]
        self.peak_flexion_time = self.config["peak_flexion_time"]
        self.max_swing_time = self.config["max_swing_time"]

        LOGGER.set_stream_level(LogLevel.DEBUG)
        LOGGER.info("--- Starting Two-States Gait FSM ---")
        LOGGER.info(f"Stance Control: {self.stance_control_type.upper()} | Initial Offset: {self.stance_offset}°")
        LOGGER.info(f"Parameters:\n  Unload Thr: {self.unload_threshold}N | Load Thr: {self.load_threshold}N")
        LOGGER.info(f"  Stance Ang: {self.stance_knee_angle}° | Swing Peak: {self.peak_flexion_angle}° in {self.peak_flexion_time}s")

        # 1. Bring up CAN Link
        LOGGER.info(f"Setting up CAN link {self.can_interface} at {self.bitrate} bps...")
        os.system(f"sudo ip link set {self.can_interface} down")
        os.system(f"sudo ip link set {self.can_interface} up type can bitrate {self.bitrate} sample-point 0.750")
        os.system(f"sudo ip link set {self.can_interface} txqueuelen 1000")

        # 2. Initialize CAN bus and adapters
        LOGGER.info(f"Initializing ODrive CAN on {self.can_interface}...")
        dir_path = os.path.dirname(os.path.abspath(__file__))
        dbc_path = os.path.abspath(os.path.join(dir_path, "..", "drivers", "odrive-cansimple.dbc"))
        can_bus = ODriveCAN(bus_name=self.can_interface, node_id=node_id_odrive, dbc_path=dbc_path)

        # Initialize adapters (ODriveMotor used directly in degrees)
        self.knee = ODriveMotor(can_interface=can_bus, name="knee", gear_ratio=gear_ratio)

        self.loadcell = SRILoadCell_M8123B2(
            tag="loadcell",
            channel=self.can_interface,
            offline=False
        )

        self.thigh_imu = WitMotionIMUAdapter(
            tag="Thigh IMU",
            mac_address=self.config["thigh_imu_mac"],
            connection_type="ble",
            offline=False
        )
        
        self.foot_imu = WitMotionIMUAdapter(
            tag="Foot IMU",
            mac_address=self.config["foot_imu_mac"],
            connection_type="ble",
            offline=False
        )

        # Start sensor streams
        self.loadcell.start()
        self.loadcell.calibrate()

        self.thigh_imu.start()
        self.foot_imu.start()

        # 3. Setup Controllers
        if self.stance_control_type == "cvp":
            self.stance_controller = CVPController(
                kp=0.02,
                kd=0.0006,
                offset_deg=self.stance_offset
            )
        else:
            self.stance_controller = HoldImpedanceController(
                kp=0.02,
                kd=0.0006,
                deg_eq=self.stance_knee_angle
            )

        self.swing_controller = SwingTrajectoryController(
            kp=0.02,
            kd=0.0006,
            stance_angle=self.stance_knee_angle,
            peak_flexion_angle=self.peak_flexion_angle,
            peak_flexion_time=self.peak_flexion_time,
            max_swing_time=self.max_swing_time
        )

        # 4. Define FSM States
        # State: CALIBRATING
        def enter_calibrating(*args, **kwargs):
            LOGGER.info("[FSM STATE] -> CALIBRATING: Configuring ODrive loop states...")
            self.knee.idle()
            self.knee.set_limit_current(10, 30)
            self.knee.closed_loop()
            self.knee.torque_control()

        self.calibrating_state = State(
            name="CALIBRATING",
            entry_callbacks=[enter_calibrating]
        )

        # State: STANCE
        def enter_stance(*args, **kwargs):
            LOGGER.info("[FSM STATE] -> STANCE: Stance support active.")

        self.stance_state = State(
            name="STANCE",
            entry_callbacks=[enter_stance]
        )

        # State: SWING
        def enter_swing(*args, **kwargs):
            LOGGER.info("[FSM STATE] -> SWING: Flexion trajectory active.")

        self.swing_state = State(
            name="SWING",
            entry_callbacks=[enter_swing]
        )

        # State: IDLE
        def enter_idle(*args, **kwargs):
            LOGGER.info("[FSM STATE] -> IDLE: Motor idled.")
            self.knee.idle()

        self.idle_state = State(
            name="IDLE",
            entry_callbacks=[enter_idle]
        )

        # 5. Create StateMachine
        self.sm = StateMachine()
        self.sm.add_states([self.calibrating_state, self.stance_state, self.swing_state, self.idle_state], initial_state_name="CALIBRATING")
        self.sm.previous_state = None

        # 6. Define Transitions
        def calibration_criteria(state_time: float) -> bool:
            return state_time >= self.calibrate_time

        self.sm.add_transition(
            source=self.calibrating_state,
            destination=self.stance_state,
            event_name="calibration_complete",
            criteria=calibration_criteria
        )

        def stance_to_swing_criteria(fz: float) -> bool:
            return fz < self.unload_threshold

        self.sm.add_transition(
            source=self.stance_state,
            destination=self.swing_state,
            event_name="toe_off",
            criteria=stance_to_swing_criteria
        )

        def swing_to_stance_criteria(fz: float, state_time: float) -> bool:
            return fz > self.load_threshold or state_time >= self.max_swing_time

        self.sm.add_transition(
            source=self.swing_state,
            destination=self.stance_state,
            event_name="heel_strike",
            criteria=swing_to_stance_criteria
        )

        # 7. Configure Telemetry Log
        LOGGER.track_function(self.knee.get_position, "knee_pos")
        LOGGER.track_function(self.knee.get_velocity, "knee_vel")
        LOGGER.track_function(self.knee.get_current, "knee_curr")
        LOGGER.track_function(lambda: self.loadcell.fz, "fz")
        LOGGER.track_function(lambda: self.sm.current_state.name, "state")
        if self.stance_control_type == "cvp":
            LOGGER.track_function(lambda: self.stance_offset, "cvp_offset")

        self.loop_dt = 1.0 / self.loop_hz
        self.clock = SoftRealtimeLoop(dt=self.loop_dt, report=True)

    def update(self) -> int:
        t = self.clock.time_since_start

        # Stop iteration if max duration is reached
        if self.max_duration is not None and t >= self.max_duration:
            LOGGER.info(f"Max duration of {self.max_duration}s reached. Stopping FSM loop.")
            return 0

        current_state_obj = self.sm.current_state
        self.sm.previous_state = current_state_obj

        # Check keypress inputs to adjust CVP offset dynamically
        key_input = get_keypress()
        if key_input:
            key_input = key_input.strip()
            if key_input in ['u', '+']:
                self.stance_offset += 1.0
                LOGGER.info(f"Adjusted CVP Offset to {self.stance_offset:.1f}°")
            elif key_input in ['d', '-']:
                self.stance_offset -= 1.0
                LOGGER.info(f"Adjusted CVP Offset to {self.stance_offset:.1f}°")
            else:
                try:
                    self.stance_offset = float(key_input)
                    LOGGER.info(f"Set CVP Offset to {self.stance_offset:.1f}°")
                except ValueError:
                    LOGGER.warning(f"Invalid input: '{key_input}'. Use '+' or '-' to step, or type a number and hit Enter.")

            if hasattr(self.stance_controller, 'offset_deg'):
                self.stance_controller.offset_deg = self.stance_offset

        # Update sensor readings
        self.loadcell.update()
        self.thigh_imu.update()
        self.foot_imu.update()
        fz_val = self.loadcell.fz

        # Update current state timing
        state_time = current_state_obj.current_time_in_state

        # Evaluate transitions (only when not in terminal IDLE state)
        if self.sm.current_state != self.idle_state:
            self.sm.update(fz=fz_val, state_time=state_time)

        # Execute active controller logic
        if self.sm.current_state == self.stance_state:
            self.stance_controller.update(self.knee, self.thigh_imu, self.foot_imu, self.loadcell, t, state_time)
        elif self.sm.current_state == self.swing_state:
            self.swing_controller.update(self.knee, self.thigh_imu, self.foot_imu, self.loadcell, t, state_time)

        # Update logs
        LOGGER.update()

        # Telemetry print in-place
        if int(t * 10) % 2 == 0:
            pos_deg = self.knee.get_position()
            offset_str = f" | Offset: {self.stance_offset:+.1f}°" if self.stance_control_type == "cvp" else ""
            print(f"\r t={t:5.2f}s | State: {self.sm.current_state.name:11} | Fz: {fz_val:6.1f}N | "
                  f"Knee Pos: {pos_deg:6.1f}°{offset_str} | "
                  f"Thigh Y: {self.thigh_imu.euler_y:5.1f}° | Foot Y: {self.foot_imu.euler_y:5.1f}°", end='', flush=True)

        return 1

    def run(self):
        LOGGER.info("Starting real-time FSM loop... Press Ctrl+C to stop cleanly.")
        LOGGER.info("Type offset adjustment (e.g. '+', '-', or a custom number like '-5.0') and hit Enter to change CVP in real time.")
        try:
            with self.sm:
                self.clock.run(self.update)
        except KeyboardInterrupt:
            LOGGER.info("KeyboardInterrupt detected. Shutting down cleanly...")
        finally:
            self.cleanup()

    def cleanup(self):
        LOGGER.info("\nCleaning up ODrive and sensors...")
        try:
            self.knee.idle()
        except Exception:
            pass
        try:
            self.loadcell.stop()
        except Exception:
            pass
        try:
            self.thigh_imu.stop()
        except Exception:
            pass
        try:
            self.foot_imu.stop()
        except Exception:
            pass
        os.system(f"sudo ip link set {self.can_interface} down")
        LOGGER.close()
        self.clock.stop()
        LOGGER.info("Shutdown complete.")

def run_two_states_fsm(offline: bool = None, max_duration: float = None, 
                       stance_control: str = "impedance", stance_offset: float = 0.0):
    fsm = TwoStatesFSM(offline=offline, max_duration=max_duration, 
                       stance_control=stance_control, stance_offset=stance_offset)
    fsm.run()

if __name__ == "__main__":
    run_two_states_fsm()
