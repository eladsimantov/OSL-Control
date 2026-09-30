"""
Author: Sarit Nagel
Date: 27/07/2026

This script is used to test the gear ratio between the ODrive motor and the AS5048A encoder on the ankle. 
It moves the motor a known number of rotations and measures the resulting movement.
"""
import odrive
from odrive.enums import *
import spidev
import time
import math

# --- SPI SETUP ---
spi = spidev.SpiDev()
spi.open(1, 2)
spi.max_speed_hz = 100000
spi.mode = 1

def get_ext_rad():
    """
    Reads the actual raw value from the AS5048A encoder and converts it to radians.
    Returns:
        float: The external ankle position in radians."""
    spi.xfer2([0xFF, 0xFF])
    time.sleep(0.0001)
    resp = spi.xfer2([0xFF, 0xFF])
    raw = ((resp[0] & 0x3F) << 8) | resp[1]
    return (raw * 2 * math.pi) / 16384.0

if __name__ == "__main__":
    """
    Main function to test the gear ratio. It initializes the ODrive, moves the motor a specified number of rotations, 
    and measures the resulting movement of the ankle.
    """
    odrv = odrive.find_any()
    axis = odrv.axis0
    
    print("Zeroing... Do not touch the ankle.")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    
    # Starting positions
    m_start = axis.pos_vel_mapper.pos_rel
    e_start_raw = get_ext_rad()
    last_e_raw = e_start_raw
    e_wrap_accumulator = 0

    # Commands
    test_rotations = 4 #move motor 4 full circles
    print(f"Moving motor {test_rotations} rotations...")
    axis.controller.input_pos = m_start + test_rotations
    
    # Wait for movement to finish
    time.sleep(5) 

    # Final measurements
    m_end = axis.pos_vel_mapper.pos_rel
    e_final_raw = get_ext_rad()
    
    # Calculate external distance with a single manual wrap check 
    # (Assuming 10 motor turns doesn't rotate ankle more than 2pi)
    # If it does, use the accumulator logic:
    e_rel = (e_final_raw - e_start_raw)
    # If ankle moved more than half a turn, it might have wrapped
    if e_rel < -math.pi: e_rel += 2 * math.pi
    elif e_rel > math.pi: e_rel -= 2 * math.pi
    
    m_delta_rad = (m_end - m_start) * 2 * math.pi
    
    print("-" * 30)
    print(f"Motor Moved: {m_delta_rad:.4f} rad")
    print(f"Ankle Moved: {e_rel:.4f} rad")
    
    if abs(e_rel) > 0.01:
        actual_ratio = abs(m_delta_rad / e_rel)
        print(f"ACTUAL MEASURED RATIO: {actual_ratio:.4f}")
        print(f"EXPECTED (50/12): 41.667")
    else:
        print("Ankle didn't move enough to measure.")

    axis.requested_state = AXIS_STATE_IDLE