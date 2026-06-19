import time
import sys
import os
import math
from typing import Optional

# Include project path for imports
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if project_path not in sys.path:
    sys.path.insert(0, project_path)

from opensourceleg.control.fsm import State, StateMachine
from opensourceleg.utilities import SoftRealtimeLoop
from opensourceleg.logging import LOGGER, LogLevel
from src.drivers.odrive_can import ODriveCAN, ODriveMotor
from src.enabletools import HoldImpedanceController

# Default configurations - modify directly in this file
CONFIG = {
    "can_interface": "can0",
    "bitrate": 1000000,
    "node_id": 0,
    "gear_ratio": 40.0,
    "loop_hz": 100.0,
    "calibrate_time": 2.0,  # Configuration state time
    "target_angle": 0.0,    # Target hold angle in degrees
    "offline": False
}

class BaselineFSM:
    def __init__(self, offline: bool = False, max_duration: float = None):
        self.offline = offline
        self.max_duration = max_duration
        self.config = CONFIG

        self.can_interface = self.config["can_interface"]
        self.bitrate = self.config["bitrate"]
        node_id = self.config["node"] if "node" in self.config else self.config["node_id"]
        gear_ratio = self.config["gear_ratio"]
        self.loop_hz = self.config["loop_hz"]
        self.calibrate_time = self.config["calibrate_time"]
        self.target_angle = self.config["target_angle"]

        LOGGER.set_stream_level(LogLevel.DEBUG)
        LOGGER.info(f"--- Starting Baseline FSM (Offline={self.offline}) ---")
        LOGGER.info(f"Target Hold Angle: {self.target_angle}°")

        # 1. Bring up CAN Link (only if online and Linux)
        if not self.offline and sys.platform.startswith("linux"):
            LOGGER.info(f"Setting up CAN link {self.can_interface} at {self.bitrate} bps...")
            os.system(f"sudo ip link set {self.can_interface} down")
            os.system(f"sudo ip link set {self.can_interface} up type can bitrate {self.bitrate} sample-point 0.750")
            os.system(f"sudo ip link set {self.can_interface} txqueuelen 1000")

        # 2. Initialize CAN Bus (if online)
        if self.offline:
            can_bus = None
        else:
            try:
                LOGGER.info(f"Initializing CAN interface {self.can_interface}...")
                dir_path = os.path.dirname(os.path.abspath(__file__))
                dbc_path = os.path.abspath(os.path.join(dir_path, "..", "drivers", "odrive-cansimple.dbc"))
                can_bus = ODriveCAN(bus_name=self.can_interface, node_id=node_id, dbc_path=dbc_path)
            except Exception as e:
                LOGGER.error(f"Failed to connect to hardware CAN: {e}. Switching to offline mode.")
                can_bus = None
                self.offline = True

        # Initialize ODriveMotor driver
        if self.offline:
            self.knee = ODriveMotor(can_interface=None, name="knee", gear_ratio=gear_ratio)
            self.knee.alive = True
            self._mock_pos = 0.0
            self._mock_vel = 0.0
            self._mock_curr = 0.0
            self.knee.get_position = lambda: self._mock_pos
            self.knee.get_velocity = lambda: self._mock_vel
            self.knee.get_current = lambda: self._mock_curr
            self.knee.set_impedance = lambda kp, kd, deg_eq, pos_deg=None, vel_dps=None: setattr(self, '_mock_pos', deg_eq)
        else:
            self.knee = ODriveMotor(can_bus, name="knee", gear_ratio=gear_ratio)

        # 3. Setup Controller
        self.controller = HoldImpedanceController(
            kp=0.02,
            kd=0.0006,
            deg_eq=self.target_angle
        )

        # 4. Define FSM States
        # State: CALIBRATING (Handles configuration state)
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

        # State: HOLDING
        def enter_holding(*args, **kwargs):
            LOGGER.info("[FSM STATE] -> HOLDING: Closed-loop control active.")

        self.holding_state = State(
            name="HOLDING",
            entry_callbacks=[enter_holding]
        )

        # State: IDLE
        def enter_idle(*args, **kwargs):
            LOGGER.info("[FSM STATE] -> IDLE: Motor entered idle state.")
            if not self.offline:
                self.knee.idle()

        self.idle_state = State(
            name="IDLE",
            entry_callbacks=[enter_idle]
        )

        # 5. Create StateMachine
        self.sm = StateMachine()
        self.sm.add_states([self.calibrating_state, self.holding_state, self.idle_state], initial_state_name="CALIBRATING")

        # 6. Define Transitions
        def calibration_criteria(state_time: float) -> bool:
            return state_time >= self.calibrate_time

        self.sm.add_transition(
            source=self.calibrating_state,
            destination=self.holding_state,
            event_name="calibration_done",
            criteria=calibration_criteria
        )

        # 7. Configure Telemetry Log
        LOGGER.track_function(self.knee.get_position, "knee_pos")
        LOGGER.track_function(self.knee.get_velocity, "knee_vel")
        LOGGER.track_function(self.knee.get_current, "knee_curr")
        LOGGER.track_function(lambda: self.sm.current_state.name, "state")

        self.loop_dt = 1.0 / self.loop_hz
        self.clock = SoftRealtimeLoop(dt=self.loop_dt, report=True)

    def update(self) -> int:
        t = self.clock.time_since_start

        # Stop iteration if max duration is reached
        if self.max_duration is not None and t >= self.max_duration:
            LOGGER.info(f"Max duration of {self.max_duration}s reached. Stopping FSM loop.")
            return 0

        # Update current state timing
        state_time = self.sm.current_state.current_time_in_state
        
        # Check transitions
        self.sm.update(state_time=state_time)
        
        # Execute active state logic
        if self.sm.current_state == self.holding_state:
            self.controller.update(self.knee, None, None, None, t, state_time)

        # Log current variables
        LOGGER.update()
        
        # Print status info periodically
        if int(t * 10) % 5 == 0:
            pos_deg = self.knee.get_position()
            print(f"\r t={t:5.2f}s | State: {self.sm.current_state.name:11} | Knee Pos: {pos_deg:6.1f}° | Knee Curr: {self.knee.get_current():5.2f}A", end='', flush=True)
            
        return 1

    def run(self):
        LOGGER.info("Starting real-time FSM loop... Press Ctrl+C to stop cleanly.")
        try:
            with self.sm:
                self.clock.run(self.update)
        except KeyboardInterrupt:
            LOGGER.info("KeyboardInterrupt detected. Shutting down cleanly...")
        finally:
            self.cleanup()

    def cleanup(self):
        LOGGER.info("\nCleaning up ODrive and shutting down...")
        try:
            self.knee.idle()
        except Exception:
            pass
        if not self.offline and sys.platform.startswith("linux"):
            os.system(f"sudo ip link set {self.can_interface} down")
        LOGGER.close()
        self.clock.stop()
        LOGGER.info("Shutdown complete.")


def run_baseline_fsm(offline: bool = None, max_duration: float = None):
    if offline is None:
        offline = CONFIG["offline"]
    fsm = BaselineFSM(offline=offline, max_duration=max_duration)
    fsm.run()


if __name__ == "__main__":
    run_baseline_fsm(offline=True)
