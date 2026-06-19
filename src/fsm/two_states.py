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
if sys.platform == "win32":
    import msvcrt
    def get_keypress():
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            try:
                return ch.decode('utf-8', errors='ignore')
            except Exception:
                return None
        return None
else:
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
    "offline": False,
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
        self.offline = offline
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
        LOGGER.info(f"--- Starting Two-States Gait FSM (Offline={self.offline}) ---")
        LOGGER.info(f"Stance Control: {self.stance_control_type.upper()} | Initial Offset: {self.stance_offset}°")
        LOGGER.info(f"Parameters:\n  Unload Thr: {self.unload_threshold}N | Load Thr: {self.load_threshold}N")
        LOGGER.info(f"  Stance Ang: {self.stance_knee_angle}° | Swing Peak: {self.peak_flexion_angle}° in {self.peak_flexion_time}s")

        # 1. Bring up CAN Link (only if online and Linux)
        if not self.offline and sys.platform.startswith("linux"):
            LOGGER.info(f"Setting up CAN link {self.can_interface} at {self.bitrate} bps...")
            os.system(f"sudo ip link set {self.can_interface} down")
            os.system(f"sudo ip link set {self.can_interface} up type can bitrate {self.bitrate} sample-point 0.750")
            os.system(f"sudo ip link set {self.can_interface} txqueuelen 1000")

        # 2. Initialize CAN bus (if online) and adapters
        if self.offline:
            can_bus = None
        else:
            try:
                LOGGER.info(f"Initializing ODrive CAN on {self.can_interface}...")
                dir_path = os.path.dirname(os.path.abspath(__file__))
                dbc_path = os.path.abspath(os.path.join(dir_path, "..", "drivers", "odrive-cansimple.dbc"))
                can_bus = ODriveCAN(bus_name=self.can_interface, node_id=node_id_odrive, dbc_path=dbc_path)
            except Exception as e:
                LOGGER.error(f"Failed to connect to ODrive: {e}. Switching to offline.")
                can_bus = None
                self.offline = True

        # Initialize adapters (ODriveMotor used directly in degrees)
        if self.offline:
            self.knee = ODriveMotor(can_interface=None, name="knee", gear_ratio=gear_ratio)
            # Mock variables for offline simulation
            self.knee.alive = True
            self._mock_pos = 0.0
            self._mock_vel = 0.0
            self._mock_curr = 0.0
            self.knee.get_position = lambda: self._mock_pos
            self.knee.get_velocity = lambda: self._mock_vel
            self.knee.get_current = lambda: self._mock_curr
            self.knee.set_impedance = lambda kp, kd, deg_eq, pos_deg=None, vel_dps=None: setattr(self, '_mock_pos', deg_eq)
        else:
            self.knee = ODriveMotor(can_interface=can_bus, name="knee", gear_ratio=gear_ratio)

        self.loadcell = SRILoadCell_M8123B2(
            tag="loadcell",
            channel=self.can_interface,
            offline=self.offline
        )

        self.thigh_imu = WitMotionIMUAdapter(
            tag="Thigh IMU",
            mac_address=self.config["thigh_imu_mac"],
            connection_type="ble",
            offline=self.offline
        )
        
        self.foot_imu = WitMotionIMUAdapter(
            tag="Foot IMU",
            mac_address=self.config["foot_imu_mac"],
            connection_type="ble",
            offline=self.offline
        )

        # Start sensor streams
        self.loadcell.start()
        if not self.offline:
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
            if not self.offline:
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
            if not self.offline:
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
        self.sim_stance_start = 0.0

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

        # Offline simulation updates
        if self.offline:
            if current_state_obj == self.calibrating_state:
                self.loadcell._data[2] = 120.0
            elif current_state_obj == self.stance_state:
                if self.sm.previous_state == self.calibrating_state or self.sm.previous_state == self.swing_state:
                    self.sim_stance_start = t
                elapsed_stance = t - self.sim_stance_start
                self.loadcell._data[2] = max(10.0, 150.0 - (140.0 * (elapsed_stance / 1.2)))
            elif current_state_obj == self.swing_state:
                self.loadcell._data[2] = 5.0
                state_time = current_state_obj.current_time_in_state
                if state_time >= 0.45:
                    self.loadcell._data[2] = 130.0

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
        if not self.offline and sys.platform.startswith("linux"):
            os.system(f"sudo ip link set {self.can_interface} down")
        LOGGER.close()
        self.clock.stop()
        LOGGER.info("Shutdown complete.")

def run_two_states_fsm(offline: bool = None, max_duration: float = None, 
                       stance_control: str = "impedance", stance_offset: float = 0.0):
    if offline is None:
        offline = CONFIG["offline"]
    fsm = TwoStatesFSM(offline=offline, max_duration=max_duration, 
                       stance_control=stance_control, stance_offset=stance_offset)
    fsm.run()

if __name__ == "__main__":
    run_two_states_fsm(offline=True)
