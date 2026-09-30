"""
Author: Sarit Nagel
Date: 09/07/2026

This script performs position control on a motor using an ODrive S1 controller 
connected to a Raspberry Pi. The hardware chain consists of a USB-to-CAN 
adapter plugged into the Raspberry Pi, which connects to the ODrive S1 
via its USB-C port. 

The code handles hardware initialization, motor calibration, and executes 
a smooth move using a Trapezoidal Trajectory profile to ensure controlled 
acceleration and deceleration.
"""

import odrive
from odrive.enums import *
import time
import math
from odrive.utils import dump_errors

def setup_and_move():
    """
    Connects to the ODrive, configures motor and encoder parameters, 
    calibrates the hardware, and performs a smooth position move 
    using trapezoidal trajectory tracking.
    """
    print("Searching for ODrive...")
    my_drive = odrive.find_any()
    print("Connected!")

    # 1. Clear Errors and Start Fresh
    my_drive.axis0.requested_state = AxisState.IDLE
    my_drive.clear_errors()
    time.sleep(0.5)

    # 2. Apply Hardware Parameters
    print("Applying hardware parameters...")
    my_drive.axis0.config.motor.pole_pairs = 21
    my_drive.axis0.config.motor.torque_constant = 0.0827
    my_drive.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
    my_drive.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0

    # Limits & Calibration Currents
    my_drive.axis0.config.motor.calibration_current = 10
    my_drive.axis0.config.motor.resistance_calib_max_voltage = 2
    my_drive.axis0.config.motor.current_soft_max = 10
    my_drive.axis0.config.motor.current_hard_max = 23
    my_drive.axis0.config.torque_soft_min = -math.inf
    my_drive.axis0.config.torque_soft_max = math.inf
    print("Searching for ODrive...")
    my_drive = odrive.find_any()
    print("Connected!")


    # 3. Apply Smooth Tracking (Trapezoidal Trajectory) Configuration
    print("Setting up Position Control with Trajectory Tracking...")
    my_drive.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
    my_drive.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ

    # Control Gains (High Stiffness): Configures the motor to behave like a rigid muscle, 
    # ensuring high positional accuracy and resistance to external loads on the ankle.
    my_drive.axis0.controller.config.pos_gain = 100.0 
    my_drive.axis0.controller.config.vel_gain = 0.1
    my_drive.axis0.controller.config.vel_integrator_gain = 0.3
    my_drive.axis0.controller.config.vel_limit = 10.0

    # Trajectory Path Smoothness
    my_drive.axis0.trap_traj.config.vel_limit = 2.0     # Max speed during move
    my_drive.axis0.trap_traj.config.accel_limit = 1.0   # Speed up rate
    my_drive.axis0.trap_traj.config.decel_limit = 1.0   # Slow down rate

    # 4. Calibration Sequence
    print("Starting Calibration (Motor should move/beep)...")
    my_drive.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    while my_drive.axis0.current_state != AxisState.IDLE:
        time.sleep(0.1)
    print("Calibration Complete.")

    # 5. Engage Motor
    print("Engaging CLOSED_LOOP_CONTROL...")
    my_drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(0.5)


    # 6. Execute Smooth Move
    if my_drive.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
        start_pos = my_drive.axis0.pos_estimate
        if start_pos >=3:
            target = start_pos - 2.0
        else:
            target = start_pos + 2.0
        print(f"Moving from {start_pos:.3f} to {target:.3f}")
        my_drive.axis0.controller.input_pos = target
        
        for i in range(50):
            curr_pos = my_drive.axis0.pos_estimate
            # This is the most reliable way to see the 'push'
            measured_iq = my_drive.axis0.motor.foc.Iq_measured 
            calc_torque = measured_iq * 0.0827
            
            print(f"Step {i}: Pos: {curr_pos:.3f} | Measured Torque: {calc_torque:.4f} Nm")
            
            if abs(curr_pos - target) < 0.05:
                print("Target reached!")
                break
            time.sleep(0.2)
    # 7. Disengage
    print("Disengaging motor.")
    my_drive.axis0.requested_state = AxisState.IDLE
    print("Done.")

if __name__ == "__main__":
    """
    Main entry point of the script. It triggers the ODrive test sequence.
    """
    setup_and_move()