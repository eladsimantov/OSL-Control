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
from src.adapters.loadcell import SRILoadCell_M8123B2

# Default configurations - modify directly in this file
CONFIG = {
    # Hardware configurations
    "can_interface_odrive": "can0",
    "can_interface_loadcell": "can1",
    "node_id_odrive": 0,
    "gear_ratio": 40.0,
    "loop_hz": 100.0,
    "calibrate_time": 5.0,
    "offline": False,

    # Gait parameters
    "unload_threshold": 20.0,      # N (Fz below this triggers Swing)
    "load_threshold": 60.0,        # N (Fz above this triggers Stance)
    "stance_knee_angle": 5.0,      # degrees (slight flexion for support)
    "peak_flexion_angle": 40.0,    # degrees (peak flexion during swing)
    "peak_flexion_time": 0.25,     # seconds into swing to reach peak flexion
    "max_swing_time": 0.55,        # seconds (safety swing timeout)
}

class TwoStatesFSM:
    def __init__(self, offline: bool = False, max_duration: float = None):
        self.offline = offline
        self.max_duration = max_duration
        self.config = CONFIG

        can_interface_odrive = self.config["can_interface_odrive"]
        can_interface_loadcell = self.config["can_interface_loadcell"]
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

        # Convert degree target angles to radians for actuator adapter
        self.stance_knee_angle_rad = math.radians(self.stance_knee_angle)
        self.peak_flexion_angle_rad = math.radians(self.peak_flexion_angle)

        print(f"--- Starting Two-States Gait FSM (Offline={self.offline}) ---")
        print(f"Parameters:\n  Unload Thr: {self.unload_threshold}N | Load Thr: {self.load_threshold}N")
        print(f"  Stance Ang: {self.stance_knee_angle}° | Swing Peak: {self.peak_flexion_angle}° in {self.peak_flexion_time}s")

        # 1. Initialize CAN bus (if online) and adapters
        if self.offline:
            can_bus = None
        else:
            try:
                print(f"Initializing ODrive CAN on {can_interface_odrive}...")
                # Resolve DBC path dynamically relative to the driver file location
                dir_path = os.path.dirname(os.path.abspath(__file__))
                dbc_path = os.path.abspath(os.path.join(dir_path, "..", "drivers", "odrive-cansimple.dbc"))
                can_bus = ODriveCAN(bus_name=can_interface_odrive, node_id=node_id_odrive, dbc_path=dbc_path)
            except Exception as e:
                print(f"Failed to connect to ODrive: {e}. Switching to offline.")
                can_bus = None
                self.offline = True

        # Initialize adapters
        self.knee = ODriveActuator(
            can_interface=can_bus,
            tag="knee",
            gear_ratio=gear_ratio,
            offline=self.offline
        )

        self.loadcell = SRILoadCell_M8123B2(
            tag="loadcell",
            channel=can_interface_loadcell,
            offline=self.offline
        )

        # Start sensor stream
        self.loadcell.start()

        # 2. Define FSM States
        # State: CALIBRATING
        def enter_calibrating(*args, **kwargs):
            print("[FSM STATE] -> CALIBRATING: Initiating motor calibration...")
            if not self.knee.is_offline:
                self.knee.driver.set_state(3)

        self.calibrating_state = State(
            name="CALIBRATING",
            entry_callbacks=[enter_calibrating]
        )

        # State: STANCE
        def enter_stance(*args, **kwargs):
            print("[FSM STATE] -> STANCE: Weight support active.")
            self.knee.home()
            self.knee.start()

        self.stance_state = State(
            name="STANCE",
            entry_callbacks=[enter_stance]
        )

        # State: SWING
        def enter_swing(*args, **kwargs):
            print("[FSM STATE] -> SWING: Flexion trajectory initiated.")

        self.swing_state = State(
            name="SWING",
            entry_callbacks=[enter_swing]
        )

        # State: IDLE
        def enter_idle(*args, **kwargs):
            print("[FSM STATE] -> IDLE: Motor entered idle state.")
            self.knee.stop()

        self.idle_state = State(
            name="IDLE",
            entry_callbacks=[enter_idle]
        )

        # 3. Create StateMachine
        self.sm = StateMachine()
        self.sm.add_states([self.calibrating_state, self.stance_state, self.swing_state, self.idle_state], initial_state_name="CALIBRATING")

        # Track state history to determine transitions
        self.sm.previous_state = None

        # 4. Define Transitions
        # Transition 1: CALIBRATING -> STANCE
        def calibration_criteria(state_time: float) -> bool:
            return state_time >= self.calibrate_time

        self.sm.add_transition(
            source=self.calibrating_state,
            destination=self.stance_state,
            event_name="calibration_complete",
            criteria=calibration_criteria
        )

        # Transition 2: STANCE -> SWING (when leg unloads)
        def stance_to_swing_criteria(fz: float) -> bool:
            return fz < self.unload_threshold

        self.sm.add_transition(
            source=self.stance_state,
            destination=self.swing_state,
            event_name="toe_off",
            criteria=stance_to_swing_criteria
        )

        # Transition 3: SWING -> STANCE (heel strike or timeout)
        def swing_to_stance_criteria(fz: float, state_time: float) -> bool:
            return fz > self.load_threshold or state_time >= self.max_swing_time

        self.sm.add_transition(
            source=self.swing_state,
            destination=self.stance_state,
            event_name="heel_strike",
            criteria=swing_to_stance_criteria
        )

        # 5. Execute FSM inside SoftRealtimeLoop
        self.loop_dt = 1.0 / self.loop_hz
        self.clock = SoftRealtimeLoop(dt=self.loop_dt, report=True)
        
        # Store simulated stance start time for offline mode
        self.sim_stance_start = 0.0

    def update(self) -> int:
        t = self.clock.time_since_start

        # Stop iteration if max duration is reached
        if self.max_duration is not None and t >= self.max_duration:
            print(f"Max duration of {self.max_duration}s reached. Stopping FSM loop.")
            return 0

        current_state_obj = self.sm.current_state

        # Cache previous state before update
        self.sm.previous_state = current_state_obj

        # Offline simulation updates
        if self.offline:
            if current_state_obj == self.calibrating_state:
                self.loadcell._data[2] = 120.0
            elif current_state_obj == self.stance_state:
                # Force decreases over time simulating leg unloading
                if self.sm.previous_state == self.calibrating_state or self.sm.previous_state == self.swing_state:
                    self.sim_stance_start = t
                elapsed_stance = t - self.sim_stance_start
                # Decrease Fz from 150N down to 10N in 1.2s
                self.loadcell._data[2] = max(10.0, 150.0 - (140.0 * (elapsed_stance / 1.2)))
            elif current_state_obj == self.swing_state:
                # Loadcell shows minimal ground force during swing
                self.loadcell._data[2] = 5.0
                # Simulate heel strike after 0.45 seconds (before timeout)
                state_time = current_state_obj.current_time_in_state
                if state_time >= 0.45:
                    self.loadcell._data[2] = 130.0

        # Update sensor readings
        self.loadcell.update()
        fz_val = self.loadcell.fz

        # Update current state timing
        state_time = current_state_obj.current_time_in_state

        # Evaluate state machine transitions
        self.sm.update(fz=fz_val, state_time=state_time)

        # Execute active state control logic
        if self.sm.current_state == self.stance_state:
            # Hold knee at stance angle (in radians)
            self.knee.set_motor_position(self.stance_knee_angle_rad)
        
        elif self.sm.current_state == self.swing_state:
            # Trajectory: flex to target, then return to full extension (in radians)
            if state_time < self.peak_flexion_time:
                # Interpolate from stance angle to peak flexion
                alpha = state_time / self.peak_flexion_time
                target_ang = self.stance_knee_angle_rad + alpha * (self.peak_flexion_angle_rad - self.stance_knee_angle_rad)
            else:
                # Interpolate back to 0.0 rad (full extension)
                time_left = self.max_swing_time - self.peak_flexion_time
                alpha = (state_time - self.peak_flexion_time) / time_left
                target_ang = self.peak_flexion_angle_rad - alpha * self.peak_flexion_angle_rad
                target_ang = max(0.0, target_ang)
            
            self.knee.set_motor_position(target_ang)

        # Periodic prints
        if int(t * 10) % 20 == 0:
            pos_deg = math.degrees(self.knee.motor_position)
            print(f"Time: {t:.2f}s | State: {self.sm.current_state.name} ({state_time:.2f}s) | Fz: {fz_val:.1f}N")
            print(f"Knee Position: {pos_deg:.2f}°")

        return 1

    def run(self):
        print("Two-states FSM setup complete. Starting gait loop...")
        try:
            with self.sm:
                self.clock.run(self.update)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt detected. Shutting down cleanly...")
        finally:
            self.cleanup()

    def cleanup(self):
        # Graceful cleanup
        print("Cleaning up ODrive and shutting down sensors...")
        try:
            self.knee.stop()
        except Exception:
            pass
        try:
            self.loadcell.stop()
        except Exception:
            pass
        if not self.offline and hasattr(self.knee, 'driver') and self.knee.driver and hasattr(self.knee.driver, 'can') and self.knee.driver.can.bus:
            try:
                self.knee.driver.can.bus.shutdown()
            except Exception:
                pass
        self.clock.stop()
        print("Shutdown complete.")


def run_two_states_fsm(offline: bool = None, max_duration: float = None):
    if offline is None:
        offline = CONFIG["offline"]
    fsm = TwoStatesFSM(offline=offline, max_duration=max_duration)
    fsm.run()


if __name__ == "__main__":
    run_two_states_fsm(offline=True)
