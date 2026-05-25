"""
test_actuator_adapter.py - Tests the ODriveActuator adapter for all control modes.

Mirrors the functionality in test_actuator.py but uses:
  - ODriveActuator adapter instead of raw ODriveMotor
  - SoftRealtimeLoop instead of time.sleep
  - LOGGER instead of print

Usage:
  Offline:  python tests/test_actuator_adapter.py
  Online:   python tests/test_actuator_adapter.py --online --node-id 0

Author: Elad Siman Tov
Date: 2026-01-01
"""

import sys
import os
import math

# Include project path for imports
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_path not in sys.path:
    sys.path.insert(0, project_path)

from opensourceleg.actuators import CONTROL_MODES
from opensourceleg.utilities import SoftRealtimeLoop
from opensourceleg.logging import LOGGER
from opensourceleg.logging.logger import LogLevel

from src.adapters.actuator import ODriveActuator


# ─── Position Control ────────────────────────────────────────────────
def test_position_control(actuator, duration=5.0, dt=0.01):
    """
    Position control test - commands a sine wave trajectory.
    Replaces: test_position_control + test_sine_movement from test_actuator.py
    """
    LOGGER.info("=== Position Control Test ===")
    actuator.set_control_mode(CONTROL_MODES.POSITION)

    loop = SoftRealtimeLoop(dt=dt)
    for t in loop:
        # Sine wave: 30 deg amplitude, 0.5 Hz
        target_deg = 30.0 * math.sin(2 * math.pi * 0.5 * t)
        actuator.set_motor_position(math.radians(target_deg))
        actuator.update()

        actual_deg = math.degrees(actuator.motor_position)
        LOGGER.debug(
            f"t={t:.3f}s | target={target_deg:7.2f}° | actual={actual_deg:7.2f}°"
        )

        if t >= duration:
            loop.stop()

    LOGGER.info("Position control test complete.\n")


# ─── Velocity Control ────────────────────────────────────────────────
def test_velocity_control(actuator, duration=5.0, dt=0.01):
    """
    Velocity control test - commands a constant velocity and reads back position.
    Replaces: test_velocity_control from test_actuator.py
    """
    LOGGER.info("=== Velocity Control Test ===")
    actuator.set_control_mode(CONTROL_MODES.VELOCITY)

    target_vel_dps = 45.0  # deg/s
    actuator.set_motor_velocity(math.radians(target_vel_dps))

    loop = SoftRealtimeLoop(dt=dt)
    for t in loop:
        actuator.update()

        pos_deg = math.degrees(actuator.motor_position)
        vel_dps = math.degrees(actuator.motor_velocity)
        LOGGER.debug(
            f"t={t:.3f}s | cmd_vel={target_vel_dps:.1f}°/s | pos={pos_deg:7.2f}° | vel={vel_dps:7.2f}°/s"
        )

        if t >= duration:
            loop.stop()

    LOGGER.info("Velocity control test complete.\n")


# ─── Torque Control ──────────────────────────────────────────────────
def test_torque_control(actuator, duration=5.0, dt=0.01):
    """
    Torque control test - commands a constant torque and monitors current + velocity.
    Replaces: test_torque_control from test_actuator.py
    """
    LOGGER.info("=== Torque Control Test ===")
    actuator.set_control_mode(CONTROL_MODES.TORQUE)

    target_torque = 0.2  # Nm
    actuator.set_motor_torque(target_torque)

    loop = SoftRealtimeLoop(dt=dt)
    for t in loop:
        actuator.update()

        vel_dps = math.degrees(actuator.motor_velocity)
        current = actuator.motor_current
        LOGGER.debug(
            f"t={t:.3f}s | torque={target_torque:.2f}Nm | vel={vel_dps:7.2f}°/s | current={current:.4f}A"
        )

        if t >= duration:
            loop.stop()

    LOGGER.info("Torque control test complete.\n")


# ─── Impedance Control ───────────────────────────────────────────────
def test_impedance_control(actuator, duration=5.0, dt=0.01):
    """
    Impedance control test - spring-damper behavior around an equilibrium.
    Replaces: test_impedance_control from test_actuator.py
    The update() loop computes torque = -Kp*(q - pos_eq) - Kd*qdot and sends it.
    """
    LOGGER.info("=== Impedance Control Test ===")
    actuator.set_control_mode(CONTROL_MODES.IMPEDANCE)

    actuator._set_impedance_gains(k=10.0, b=0.5)
    actuator.pos_eq = math.radians(30.0)

    loop = SoftRealtimeLoop(dt=dt)
    for t in loop:
        actuator.update()

        pos_deg = math.degrees(actuator.motor_position)
        eq_deg = math.degrees(actuator.pos_eq)
        torque = actuator.motor_torque
        LOGGER.debug(
            f"t={t:.3f}s | pos={pos_deg:7.2f}° | eq={eq_deg:.1f}° | torque={torque:.4f}Nm"
        )

        if t >= duration:
            loop.stop()

    LOGGER.info("Impedance control test complete.\n")


# ─── Main ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Raspberry Pi settings for can bus communications to motor
    CAN_CH = 'can0'
    BITRATE = 1000000  # Default is 1Mb/s 
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE}")

    import argparse

    parser = argparse.ArgumentParser(description="ODriveActuator adapter test")
    parser.add_argument("--online", action="store_true", help="Run on real hardware")
    parser.add_argument("--node-id", type=int, default=0, help="CAN node ID")
    parser.add_argument("--gear-ratio", type=float, default=40.0)
    parser.add_argument("--dt", type=float, default=0.01, help="Loop period in seconds")
    parser.add_argument("--duration", type=float, default=5.0, help="Test duration in seconds")
    parser.add_argument("--calibrate", action="store_true", help="Run calibration first")
    args = parser.parse_args()

    # ── Logger setup ──
    LOGGER.set_stream_level(LogLevel.DEBUG)

    # ── Actuator setup ──
    if args.online:
        from src.drivers.odrive_can import ODriveCAN
        dbc_path = os.path.join(project_path, "src", "drivers", "odrive-cansimple.dbc")
        can = ODriveCAN(node_id=args.node_id, dbc_path=dbc_path)
        actuator = ODriveActuator(
            can_interface=can, tag="knee", gear_ratio=args.gear_ratio
        )
    else:
        actuator = ODriveActuator(
            can_interface=None, tag="knee", gear_ratio=args.gear_ratio, offline=True
        )

    # ── Track variables with LOGGER ──
    LOGGER.track_attributes(actuator, [
        "motor_position", "motor_velocity", "motor_current", "motor_torque"
    ])

    # ── Calibrate & set limits (like test_actuator.py) ──
    if args.calibrate:
        actuator.calibrate(wait_time=10.0)

    actuator.set_current_limit(max_current=10.0, max_velocity=10.0)
    actuator.home()
    actuator.start()

    # ── Run tests ──
    LOGGER.info(f"Running tests (dt={args.dt}s, duration={args.duration}s)\n")

    test_position_control(actuator, duration=args.duration, dt=args.dt)
    test_velocity_control(actuator, duration=args.duration, dt=args.dt)
    test_torque_control(actuator, duration=args.duration, dt=args.dt)
    test_impedance_control(actuator, duration=args.duration, dt=args.dt)

    # ── Cleanup ──
    actuator.stop()
    LOGGER.info("All tests completed.")
