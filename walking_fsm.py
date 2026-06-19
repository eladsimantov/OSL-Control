#!/usr/bin/env python3
"""
walking_fsm.py - A clean, simple real-time gait FSM for the Raspberry Pi.

This script executes a 2-state walking machine (STANCE/SWING) using real hardware.
You can directly edit the parameters, gains, and controllers inside this file.
"""

import os
import sys
import math
import select
import numpy as np

# Ensure project paths are resolved
project_path = os.path.dirname(os.path.abspath(__file__))
if project_path not in sys.path:
    sys.path.insert(0, project_path)

from src.drivers.odrive_can import ODriveCAN, ODriveMotor
from src.adapters.loadcell import SRILoadCell_M8123B2
from src.adapters.imu import WitMotionIMUAdapter
from src.enabletools import HoldImpedanceController, SwingTrajectoryController, CVPController
from opensourceleg.utilities import SoftRealtimeLoop

# ==================== TUNABLE PARAMETERS ====================
CAN_CH = "can0"
BITRATE = 1000000
MOTOR_ID = 0
GEAR_RATIO = 40.0
LOOP_HZ = 100.0

# Gait triggers
UNLOAD_THRESHOLD = 20.0  # N (Fz below this triggers Swing)
LOAD_THRESHOLD = 60.0    # N (Fz above this triggers Stance)
MAX_SWING_TIME = 0.55    # seconds (safety timeout to return to Stance)

# Impedance gains
KP = 0.02
KD = 0.0006

# Controller settings
STANCE_CONTROL = "cvp"   # "impedance" or "cvp"
STANCE_OFFSET = 0.0      # initial CVP offset in degrees
STANCE_KNEE_ANGLE = 5.0  # degrees (target angle for HoldImpedanceController)

# Swing trajectory settings
PEAK_FLEXION_ANGLE = 40.0
PEAK_FLEXION_TIME = 0.25
# ============================================================

def get_keypress():
    """Non-blocking keyboard read from console."""
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    if dr:
        return sys.stdin.readline().strip()
    return None

def main():
    print(f"--- Starting Real-Time Walking FSM on RPi ({CAN_CH}) ---")
    
    # 1. Initialize hardware CAN
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE} sample-point 0.750")
    os.system(f"sudo ip link set {CAN_CH} txqueuelen 1000")

    dir_path = os.path.dirname(os.path.abspath(__file__))
    dbc_path = os.path.join(dir_path, "src", "drivers", "odrive-cansimple.dbc")
    can_bus = ODriveCAN(bus_name=CAN_CH, node_id=MOTOR_ID, dbc_path=dbc_path)
    
    # Initialize adapters
    knee = ODriveMotor(can_bus, name="knee", gear_ratio=GEAR_RATIO)
    loadcell = SRILoadCell_M8123B2(tag="Shank LC", channel=CAN_CH)
    thigh_imu = WitMotionIMUAdapter(tag="Thigh IMU", mac_address="EF:D5:AC:1A:0D:21", connection_type="ble")
    foot_imu = WitMotionIMUAdapter(tag="Foot IMU", mac_address="EC:8E:70:CE:63:24", connection_type="ble")

    # Start streams
    loadcell.start()
    loadcell.calibrate()
    thigh_imu.start()
    foot_imu.start()

    # ODrive closed-loop setup
    knee.idle()
    knee.set_limit_current(10, 30)
    knee.closed_loop()
    knee.torque_control()

    # Initialize controllers
    global STANCE_OFFSET
    if STANCE_CONTROL == "cvp":
        stance_controller = CVPController(kp=KP, kd=KD, offset_deg=STANCE_OFFSET)
    else:
        stance_controller = HoldImpedanceController(kp=KP, kd=KD, deg_eq=STANCE_KNEE_ANGLE)

    swing_controller = SwingTrajectoryController(
        kp=KP, kd=KD,
        stance_angle=STANCE_KNEE_ANGLE,
        peak_flexion_angle=PEAK_FLEXION_ANGLE,
        peak_flexion_time=PEAK_FLEXION_TIME,
        max_swing_time=MAX_SWING_TIME
    )

    # State tracking
    state = "STANCE"
    state_start_time = 0.0

    print("\nFSM ready. Starting control loop. Press Ctrl+C to exit.")
    print("Type '+' or '-' followed by Enter to step the CVP offset, or type a new float offset.")

    loop = SoftRealtimeLoop(dt=1.0 / LOOP_HZ)
    try:
        for t in loop:
            # Read sensors (non-blocking updates)
            loadcell.update()
            thigh_imu.update()
            foot_imu.update()
            
            fz = loadcell.fz
            state_time = t - state_start_time

            # Check keyboard input for adjusting CVP offset
            key = get_keypress()
            if key:
                if key in ['u', '+']:
                    STANCE_OFFSET += 1.0
                    print(f"\nOffset: {STANCE_OFFSET:.1f}°")
                elif key in ['d', '-']:
                    STANCE_OFFSET -= 1.0
                    print(f"\nOffset: {STANCE_OFFSET:.1f}°")
                else:
                    try:
                        STANCE_OFFSET = float(key)
                        print(f"\nOffset set to: {STANCE_OFFSET:.1f}°")
                    except ValueError:
                        pass
                if STANCE_CONTROL == "cvp":
                    stance_controller.offset_deg = STANCE_OFFSET

            # Evaluate transitions (STANCE <-> SWING)
            if state == "STANCE":
                if fz < UNLOAD_THRESHOLD:
                    state = "SWING"
                    state_start_time = t
                    state_time = 0.0
                    print("\n[TRANSITION] -> SWING (Toe Off)")
            elif state == "SWING":
                if fz > LOAD_THRESHOLD or state_time >= MAX_SWING_TIME:
                    state = "STANCE"
                    state_start_time = t
                    state_time = 0.0
                    print("\n[TRANSITION] -> STANCE (Heel Strike)")

            # Execute active controller
            if state == "STANCE":
                stance_controller.update(knee, thigh_imu, foot_imu, loadcell, t, state_time)
            elif state == "SWING":
                swing_controller.update(knee, thigh_imu, foot_imu, loadcell, t, state_time)

            # Print telemetry periodically
            if round(t * LOOP_HZ) % 10 == 0:
                pos = knee.get_position()
                offset_str = f" | Offset: {STANCE_OFFSET:+.1f}°" if STANCE_CONTROL == "cvp" else ""
                print(f"\r t={t:6.2f}s | State: {state:8} | Fz: {fz:6.1f}N | "
                      f"Knee Pos: {pos:5.1f}°{offset_str} | "
                      f"Thigh Y: {thigh_imu.euler_y:5.1f}° | Foot Y: {foot_imu.euler_y:5.1f}°", end="", flush=True)

    finally:
        print("\nCleaning up and idling motor...")
        knee.idle()
        loadcell.stop()
        thigh_imu.stop()
        foot_imu.stop()
        os.system(f"sudo ip link set {CAN_CH} down")

if __name__ == "__main__":
    main()
