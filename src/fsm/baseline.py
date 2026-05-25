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
from src.drivers.odrive_can import ODriveCAN
from src.adapters.actuator import ODriveActuator

# Default configurations - modify directly in this file
CONFIG = {
    "can_interface": "can0",
    "node_id": 0,
    "gear_ratio": 40.0,
    "loop_hz": 100.0,
    "calibrate_time": 5.0,
    "offline": False
}

class BaselineFSM:
    def __init__(self, offline: bool = False, max_duration: float = None):
        self.offline = offline
        self.max_duration = max_duration
        self.config = CONFIG

        can_interface = self.config["can_interface"]
        node_id = self.config["node_id"]
        gear_ratio = self.config["gear_ratio"]
        self.loop_hz = self.config["loop_hz"]
        self.calibrate_time = self.config["calibrate_time"]

        print(f"--- Starting Baseline FSM (Offline={self.offline}) ---")

        # 1. Initialize CAN Bus (if online) and Actuator Adapter
        if self.offline:
            can_bus = None
        else:
            try:
                print(f"Initializing CAN interface {can_interface}...")
                # Resolve DBC path dynamically relative to the driver file location
                dir_path = os.path.dirname(os.path.abspath(__file__))
                dbc_path = os.path.abspath(os.path.join(dir_path, "..", "drivers", "odrive-cansimple.dbc"))
                can_bus = ODriveCAN(bus_name=can_interface, node_id=node_id, dbc_path=dbc_path)
            except Exception as e:
                print(f"Failed to connect to hardware CAN: {e}. Switching to offline mode.")
                can_bus = None
                self.offline = True

        # Initialize ODriveActuator adapter
        self.knee = ODriveActuator(
            can_interface=can_bus,
            tag="knee",
            gear_ratio=gear_ratio,
            offline=self.offline
        )

        # 2. Define FSM States
        # State: CALIBRATING
        def enter_calibrating(*args, **kwargs):
            print("[FSM STATE] -> CALIBRATING: Initiating motor calibration...")
            # For ODriveMotor driver calibration mode:
            if not self.knee.is_offline:
                self.knee.driver.set_state(3)

        self.calibrating_state = State(
            name="CALIBRATING",
            entry_callbacks=[enter_calibrating]
        )

        # State: HOLDING
        def enter_holding(*args, **kwargs):
            print("[FSM STATE] -> HOLDING: Closed-loop control active.")
            self.knee.home()
            self.knee.start()

        self.holding_state = State(
            name="HOLDING",
            entry_callbacks=[enter_holding]
        )

        # State: IDLE
        def enter_idle(*args, **kwargs):
            print("[FSM STATE] -> IDLE: Motor entering idle state.")
            self.knee.stop()

        self.idle_state = State(
            name="IDLE",
            entry_callbacks=[enter_idle]
        )

        # 3. Create StateMachine
        self.sm = StateMachine()
        self.sm.add_states([self.calibrating_state, self.holding_state, self.idle_state], initial_state_name="CALIBRATING")

        # 4. Define Transitions
        # Transition: CALIBRATING -> HOLDING after calibrate_time
        def calibration_criteria(state_time: float) -> bool:
            return state_time >= self.calibrate_time

        self.sm.add_transition(
            source=self.calibrating_state,
            destination=self.holding_state,
            event_name="calibration_done",
            criteria=calibration_criteria
        )

        # 5. Execute FSM in SoftRealtimeLoop
        self.loop_dt = 1.0 / self.loop_hz
        self.clock = SoftRealtimeLoop(dt=self.loop_dt, report=True)

    def update(self) -> int:
        t = self.clock.time_since_start

        # Stop iteration if max duration is reached
        if self.max_duration is not None and t >= self.max_duration:
            print(f"Max duration of {self.max_duration}s reached. Stopping FSM loop.")
            return 0

        # Update current state's internal variables
        state_time = self.sm.current_state.current_time_in_state
        
        # Check transitions
        self.sm.update(state_time=state_time)
        
        # Execute active state logic
        if self.sm.current_state == self.holding_state:
            # Keep commanding 0 radians (absolute position control at home angle)
            self.knee.set_motor_position(0.0)
        
        # Print status info periodically
        if int(t * 10) % 20 == 0:
            pos_deg = math.degrees(self.knee.motor_position)
            print(f"Time: {t:.2f}s | Current State: {self.sm.current_state.name} | State Time: {state_time:.1f}s")
            print(f"Knee Position: {pos_deg:.2f}°")
            
        return 1

    def run(self):
        print("FSM setup complete. Starting real-time control loop...")
        try:
            with self.sm:
                self.clock.run(self.update)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt detected. Shutting down cleanly...")
        finally:
            self.cleanup()

    def cleanup(self):
        # Graceful cleanup
        print("Cleaning up ODrive and shutting down CAN...")
        try:
            self.knee.stop()
        except Exception:
            pass
        if not self.offline and hasattr(self.knee, 'driver') and self.knee.driver and hasattr(self.knee.driver, 'can') and self.knee.driver.can.bus:
            try:
                self.knee.driver.can.bus.shutdown()
            except Exception:
                pass
        self.clock.stop()
        print("Shutdown complete.")


def run_baseline_fsm(offline: bool = None, max_duration: float = None):
    if offline is None:
        offline = CONFIG["offline"]
    fsm = BaselineFSM(offline=offline, max_duration=max_duration)
    fsm.run()


if __name__ == "__main__":
    run_baseline_fsm(offline=True)
