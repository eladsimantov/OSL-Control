"""
Author: Sarit Nagel
Date: 27/07/2026

This script implements a biomimetic impedance-control loop for an ankle actuator using an AS5048A absolute 
encoder and an ODrive motor controller. It alternates between STANCE, SWING, and ASSIST phases, utilizing 
bidirectional triggers and stiffness scaling based on Rouse et al. (2014). The system responds to physical 
manipulation with a virtual spring-damper behavior, employing gain and torque smoothing to ensure seamless 
transitions and avoid jolting.


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
G = -37.5                   # Gear Ratio (Ankle to Motor)
K_SPRING = 51.3127 * 2        # Physical stiffness of the 2 torsion springs (Nm/rad)

# --- GAIT & COMPLIANCE CONFIGURATION ---
# These parameters control the state machine transitions and equilibrium position ramping
PUSH_OFF_THRESHOLD = 0.25   # Activation threshold for push-off (rad) - triggers assist when ankle dorsiflexes beyond this
ASSIST_TARGET_FWD = 0.20   # Forward assist target ankle position (rad) - assists plantarflexion recovery
ASSIST_TARGET_BWD = -0.20    # Backward assist target ankle position (rad) - assists dorsiflexion if needed
# RAMP_SPEED controls how smoothly the equilibrium position transitions between states.
# Instead of snapping to a new position, it gradually moves toward the target at this rate.
# This prevents jerky motor responses and allows for smooth force transitions.
RAMP_SPEED = 0.0001          # Rate at which the target equilibrium position moves toward the assist target (rad per loop)
# K_SMOOTH smooths the stiffness and damping gains during phase transitions to avoid "snapping" behavior
K_SMOOTH = 0.01             # Smoothing factor for gain transitions (Lower = smoother)

ALPHA = 0.02                # Velocity filter strength

# Virtual spring-damper parameters for the ASSIST phase
ASSIST_K = 15.0             # Virtual spring stiffness for assist phase (Nm/rad)
ASSIST_D = 2.0              # Damping for assist phase (Nm/(rad/s))

# Constants Implemented Based on Rouse et al. (2014) "Estimation of Human Ankle Impedance During the Stance Phase of Walking"
# Source: https://ieeexplore.ieee.org/abstract/document/6750034

TARGET_POS_RAD = 0.075          # Neutral equilibrium point (rad) according to Rouse et. al. 
USER_WEIGHT = 2.0                 # Set to actual user weight in kg 
K_SLOPE = 13.6                  # Stiffness slope (Nm/rad/kg) based on Rouse et. al. (2014)
K_INTERCEPT = 1.6               # Stiffness intercept (Nm/rad) based on Rouse et. al. (2014)
STANCE_D_NORM = 0.03            # Damping normalized to user weight (Nm/(rad/s)/kg) based on Rouse et. al. (2014)


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
    # Initialize STANCE phase with the baseline biomimetic values
    current_k = USER_WEIGHT * K_INTERCEPT
    current_d = USER_WEIGHT * STANCE_D_NORM
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
            # The system detects swing phase by high angular velocity
            if abs(filtered_omega) > 2.0:
                current_phase = "SWING"

            if current_phase == "SWING":
                # During swing: low stiffness and damping for easier leg movement
                target_k, target_d = 3.0, 0.2

                # EQUILIBRIUM RAMPING IN SWING PHASE:
                # Gradually move the equilibrium position back toward ankle center during swing.
                # This prevents the motor from maintaining a fixed dorsiflexed/plantarflexed position.
                # The ramp allows the leg to swing freely without fighting against a biased spring.
                if equilibrium > 0.005:
                    equilibrium -= RAMP_SPEED  # Ramp down from positive (plantarflex) toward equilibrium
                elif equilibrium < -0.005:
                    equilibrium += RAMP_SPEED  # Ramp up from negative (dorsiflex) toward equilibrium
                
                # Exit swing phase when velocity drops below threshold (beginning of loading response)
                if abs(filtered_omega) < 2.0:
                    current_phase = "STANCE"

            elif current_phase == "ASSIST":
                # ASSIST PHASE: Active motor support to help the user move in a specific direction
                # This is triggered when user force exceeds PUSH_OFF_THRESHOLD
                
                # 1. Biomimetic Scaling for Assist
                # Use a higher baseline stiffness for assist than swing, but still scaled by user mass
                # Higher stiffness provides more positive guidance toward the assist target
                target_k = USER_WEIGHT * 5.0  # Normalized assist stiffness (higher than swing)
                
                # 2. Use the same damping found at 70% stance (0.03 Nms/rad/kg) for stability
                target_d = USER_WEIGHT * STANCE_D_NORM 

                # EQUILIBRIUM RAMPING IN ASSIST PHASE:
                # Smoothly move the virtual spring equilibrium point toward the assist target.
                # This gently guides the ankle toward the target position.
                # The ramp prevents sudden jerky movements and allows the user to resist if needed.
                if equilibrium < ASSIST_TARGET:
                    equilibrium = min(ASSIST_TARGET, equilibrium + RAMP_SPEED)  # Ramp toward target
                elif equilibrium > ASSIST_TARGET:
                    equilibrium = max(ASSIST_TARGET, equilibrium - RAMP_SPEED)  # Ramp toward target

                # ASSIST PHASE COMPLETION LOGIC:
                # Multiple exit conditions allow the phase to terminate naturally
                reached_positive = (ASSIST_TARGET > 0 and theta_a >= ASSIST_TARGET - 0.02)
                reached_negative = (ASSIST_TARGET < 0 and theta_a <= ASSIST_TARGET + 0.02)
                # User can break out of assist by applying counter force (resisting)
                user_resisting = (ASSIST_TARGET < 0 and filtered_omega > 0.5) or \
                                 (ASSIST_TARGET > 0 and filtered_omega < -0.5)
                
                if assist_start_time is None: assist_start_time = now
                
                # 3. Transition back to Biomimetic Stance
                # Exit assist when: target reached, user resists, or timeout (1.5s) exceeded
                if reached_positive or reached_negative or user_resisting or (now - assist_start_time > 1.5):
                    current_phase = "STANCE"
                    # Reset equilibrium to the natural biomimetic stance neutral point
                    equilibrium = TARGET_POS_RAD  
                    assist_start_time = None
            
            elif current_phase == "STANCE":
                # STANCE PHASE: Main walking phase with biomimetic impedance based on Rouse et al. (2014)
                # This implements human-like ankle behavior during weight-bearing
                
                # 1. BIOMIMETIC STIFFNESS CALCULATION (Nm/rad):
                # Stiffness increases with ankle angle (position-dependent gain), creating a natural bracing effect.
                # Formula from Rouse et al. (2014): K = USER_WEIGHT * (K_SLOPE * angle + K_INTERCEPT)
                target_k = USER_WEIGHT * (K_SLOPE * theta_a + K_INTERCEPT)
                
                # 2. EQUILIBRIUM POINT RAMPING IN STANCE:
                # The ankle naturally wants to return to TARGET_POS_RAD (0.075 rad plantarflex)
                # Instead of snapping there instantly,  ramp smoothly to avoid jerky behavior.
                # This simulates a gradual muscle activation and allows for smooth load distribution.
                if equilibrium < TARGET_POS_RAD:
                    equilibrium = min(TARGET_POS_RAD, equilibrium + RAMP_SPEED)  # Ramp toward natural position
                elif equilibrium > TARGET_POS_RAD:
                    equilibrium = max(TARGET_POS_RAD, equilibrium - RAMP_SPEED)  # Ramp toward natural position
                
                # 3. DAMPING: Proportional to user weight for realistic energy dissipation
                # Higher damping in stance prevents oscillation and provides stability
                target_d = USER_WEIGHT * STANCE_D_NORM 

                # PUSH-OFF DETECTION & ASSIST TRIGGER:
                # When the user actively pushes (dorsiflexes beyond threshold), activate assist
                # This detects intentional effort and provides motor support
                if theta_a > PUSH_OFF_THRESHOLD:
                    current_phase = "ASSIST"
                    ASSIST_TARGET = ASSIST_TARGET_FWD  # Assist in plantarflexion (push-off direction)
                    assist_start_time = now
                
                # Backward Push-off (for bidirectional assist/dorsiflexion assistance)
                # Allows assist in the opposite direction if user dorsiflexes beyond threshold
                elif theta_a < -PUSH_OFF_THRESHOLD:
                    current_phase = "ASSIST"
                    ASSIST_TARGET = ASSIST_TARGET_BWD  # Assist in dorsiflexion direction
                    assist_start_time = now 


            # --- 4. GAIN SMOOTHING LOGIC (Low-Pass Filter) ---
            # Prevents the motor from "snapping" when phases change
            # This is a first-order low-pass filter that smoothly transitions gains between phases.
            # current_k = K_SMOOTH * target_k + (1 - K_SMOOTH) * previous_k
            # Lower K_SMOOTH = smoother, slower transitions; Higher K_SMOOTH = faster tracking
            current_k = (K_SMOOTH * target_k) + ((1 - K_SMOOTH) * current_k)
            current_d = (K_SMOOTH * target_d) + ((1 - K_SMOOTH) * current_d)

            # --- 6. THE IMPEDANCE CONTROL LAW ---
            # This implements a virtual spring-damper:
            #   Torque = K * (equilibrium_pos - actual_pos) - D * velocity
            # - Spring term: K * (eq - θ_a) creates a restoring force toward equilibrium
            # - Damper term: D * ω provides energy dissipation (viscous damping)
            # This simulates how human ankle muscles behave: proportional to position error and velocity
            raw_torque = current_k * (equilibrium - theta_a) - current_d * filtered_omega

            # TORQUE SMOOTHING (Low-Pass Filter):
            # Smooth the torque command to eliminate high-frequency noise and prevent chattering
            # The filtered torque responds smoothly to changes in the commanded torque
            filtered_torque = (TORQUE_ALPHA * raw_torque) + (1 - TORQUE_ALPHA) * filtered_torque

            # --- 7. MOTOR COMMAND CALCULATION ---
            # Convert ankle torque to motor position using the transmission mechanics
            # The motor must compensate for:
            #   1. Ankle angle (θ_a) scaled by gear ratio
            #   2. Spring deflection caused by the required torque
            # θ_motor = G * (θ_ankle + τ/K_spring)
            # where G is gear ratio, K_spring is torsion bar stiffness
            theta_m_rad = G * (theta_a + (filtered_torque / K_SPRING))
            
            # Convert motor angle (radians) to motor turns for ODrive
            # ODrive expects position in units of complete revolutions
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
    """
    Main function to run the impedance control algorithm.
    """
    run_impedance_control()
