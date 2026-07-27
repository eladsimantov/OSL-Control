"""
Author: Sarit Nagel
Date: 27/07/2026

This script implements a simple impedance-control loop for an ankle
actuator with an AS5048A absolute encoder and an ODrive motor controller.
It alternates between STANCE, SWING, and ASSIST phases, smoothing gains
and torque outputs to avoid jolting behavior. The various states are activated 
by the user physically manipulating the ankle, and the system responds with a virtual spring-damper
behavior based on the measured ankle position and velocity.

Pin Cpnnections: (AS5048A to Raspberry Pi)
- GND -> Pin 6 (GND)
- VDD5V -> Pin 1 (3.3V)
- MISO -> Pin 35 (SPI0 MISO)
- MOSI -> Pin 38 (SPI0 MOSI)
- CLK -> Pin 40 (SPI0 SCLK)
- CSn -> Pin 36 (GPIO16)
"""
import odrive
from odrive.enums import *
import time
import math
import spidev

# --- Impedance Configuration ---
G = -41.667                   # Gear Ratio (Ankle to Motor)
K_SPRING = 51.3127 * 2        # Physical stiffness of the 2 torsion springs (Nm/rad)
TARGET_POS_RAD = 0.0          # Neutral equilibrium point (rad)

# --- GAIT & COMPLIANCE CONFIGURATION ---
PUSH_OFF_THRESHOLD = 0.25   # Activation threshold for push-off (rad)
ASSIST_TARGET_FWD = 0.20   # Forward assist target ankle position (rad)
ASSIST_TARGET_BWD = -0.20    # Backward assist target ankle position (rad)
RAMP_SPEED = 0.0005          # Rate at which the target equilibrium position moves toward the assist target (rad per loop)
K_SMOOTH = 0.05             # Smoothing factor for gain transitions (Lower = smoother)

ALPHA = 0.02                # Velocity filter strength

# Virtual spring-damper parameters for the ASSIST phase
ASSIST_K = 15.0             # Virtual spring stiffness for assist phase (Nm/rad)
ASSIST_D = 2.0              # Damping for assist phase (Nm/(rad/s))

# Virtual spring-damper parameters for the STANCE phase
STANCE_K = 10.0             # Virtual spring stiffness for stance phase (Nm/rad)
STANCE_D = 5.0              # Damping for stance phase (Nm/(rad/s))

# --- SPI Setup for AS5048A ---
spi = spidev.SpiDev()
spi.open(1, 2) # Corresponds to the above mentioned SPI pin connections 
spi.max_speed_hz = 100000 
spi.mode = 1

def get_raw_encoder_rad(): 
    """
    Reads the actual raw value from the AS5048A.
    Returns:
        float: The raw ankle position in radians.
    """
    try:
        spi.xfer2([0xFF, 0xFF])
        time.sleep(0.0001)
        resp = spi.xfer2([0xFF, 0xFF])
        raw_val = ((resp[0] & 0x3F) << 8) | resp[1]
        return (raw_val * 2.0 * math.pi) / 16384.0
    except:
        return 0.0

def run_impedance_control():
    """
    Main loop that handles ODrive communication, state transitions, 
    and the calculation of the impedance control law.
    """
     
    TORQUE_CONSTANT = 0.0827

    print("Searching for ODrive...")
    my_drive = odrive.find_any()
    print("Connected!")

    # 1. Setup hardware parameters
    my_drive.axis0.config.motor.pole_pairs = 21
    my_drive.axis0.config.motor.torque_constant = TORQUE_CONSTANT

    # 2. Configure for Impedance (Using Position Control as a Virtual Spring)
    my_drive.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
    my_drive.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
    my_drive.axis0.controller.config.pos_gain = 25.0 
    my_drive.axis0.controller.config.vel_gain = 0.02
    my_drive.axis0.controller.config.vel_limit = 20.0 # Safety limit

    # 3. Calibration
    print("Calibrating...")
    my_drive.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    while my_drive.axis0.current_state != AxisState.IDLE:
        time.sleep(0.1)

    # --- 4. Startup Synchronization ---
    print("Syncing Encoder and Motor...")
    encoder_start_val = get_raw_encoder_rad()
    motor_start_offset = my_drive.axis0.pos_estimate

    print("Engaging CLOSED_LOOP_CONTROL...")
    my_drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    # --- Initialize State Variables ---
    last_theta_a = 0.0                # Check for new breakout

    last_time = time.time()
    filtered_omega = 0.0
    equilibrium = 0.0
    current_phase = "STANCE"
    current_k = STANCE_K
    current_d = STANCE_D
    assist_start_time = None
    filtered_torque = 0.0
    TORQUE_ALPHA = 0.1  # Smoothing factor for the output command


    print("-" * 85)
    print(f"{'Phase':<8} | {'Target':<6} | {'Ankle (Rad)':<12} | {'Torque (Nm)':<12}")
    print("-" * 85)

    try:
       while True:
            now = time.time()
            dt = now - last_time
            if dt < 0.001: continue # 1kHz loop
            
            # 1. Read Ankle Position
            theta_a = get_raw_encoder_rad() - encoder_start_val
            # Wrap to -pi to pi
            theta_a = (theta_a + math.pi) % (2.0 * math.pi) - math.pi
            
            # 2. Filter Velocity (for Damping)
            raw_omega = (theta_a - last_theta_a) / dt
            filtered_omega = (ALPHA * raw_omega) + ((1 - ALPHA) * filtered_omega)

            # --- 3. Phase Logic & Parameter Selection ---
            if abs(filtered_omega) > 1.0:
                current_phase = "SWING"

            if current_phase == "SWING":
                target_k, target_d = 3.0, 0.2
                equilibrium = 0.0 # Return to neutral in swing
                
                if abs(filtered_omega) < 1.0:
                    current_phase = "STANCE"
                    equilibrium = theta_a 

            elif current_phase == "ASSIST":
                target_k, target_d = ASSIST_K, ASSIST_D

                # RAMP equilibrium toward the current assist target position.
                if equilibrium < ASSIST_TARGET:
                    equilibrium = min(ASSIST_TARGET, equilibrium + RAMP_SPEED)
                elif equilibrium > ASSIST_TARGET:
                    equilibrium = max(ASSIST_TARGET, equilibrium - RAMP_SPEED)

                # Completion Logic
                reached_positive = (ASSIST_TARGET > 0 and theta_a >= ASSIST_TARGET - 0.02)
                reached_negative = (ASSIST_TARGET < 0 and theta_a <= ASSIST_TARGET + 0.02)
                user_resisting = (ASSIST_TARGET < 0 and filtered_omega > 0.5) or \
                                 (ASSIST_TARGET > 0 and filtered_omega < -0.5)
                
                if assist_start_time is None: assist_start_time = now
                if reached_positive or reached_negative or user_resisting or (now - assist_start_time > 1.5):
                    current_phase = "STANCE"
                    equilibrium = 0.0  # Return home after push
                    assist_start_time = None
            
            elif current_phase == "STANCE":
                target_k, target_d = STANCE_K, STANCE_D
                
                # SMOOTH RETURN LOGIC:
                if equilibrium > 0.01:
                    equilibrium -= RAMP_SPEED
                elif equilibrium < -0.01:
                    equilibrium += RAMP_SPEED
                else:
                    equilibrium = theta_a 
                
                # Check for new breakout
                if theta_a > PUSH_OFF_THRESHOLD:
                    current_phase = "ASSIST"
                    ASSIST_TARGET = ASSIST_TARGET_FWD
                    assist_start_time = now
                elif theta_a < -PUSH_OFF_THRESHOLD:
                    current_phase = "ASSIST"
                    ASSIST_TARGET = ASSIST_TARGET_BWD
                    assist_start_time = now

            # --- 4. Gain Smoothing Logic ---
            # Prevents the motor from "snapping" when phases change
            current_k = (K_SMOOTH * target_k) + ((1 - K_SMOOTH) * current_k)
            current_d = (K_SMOOTH * target_d) + ((1 - K_SMOOTH) * current_d)

            # --- 6. The Impedance Law ---
            raw_torque = current_k * (equilibrium - theta_a) - current_d * filtered_omega

            # Smooth the torque command to kill high-frequency noise
            filtered_torque = (TORQUE_ALPHA * raw_torque) + (1 - TORQUE_ALPHA) * filtered_torque

            # --- 7. Calculate Motor Position ---
            theta_m_rad = G * (theta_a + (filtered_torque / K_SPRING))
            
            # Convert to turns
            motor_rel_turns = theta_m_rad / (2.0 * math.pi)
        
            # 8. Send Command
            my_drive.axis0.controller.input_pos = motor_start_offset + motor_rel_turns

            # Logging every 0.2 seconds
            if int(now * 5) != int(last_time * 5): 
                print(f"{current_phase:<8} | {equilibrium:<6.2f} | {theta_a:<12.3f} | {filtered_torque:<12.2f}")

            last_theta_a = theta_a
            last_time = now

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Safety Shutdown: Setting Motor to IDLE.")
        my_drive.axis0.requested_state = AxisState.IDLE
        spi.close()

if __name__ == "__main__":
    run_impedance_control()