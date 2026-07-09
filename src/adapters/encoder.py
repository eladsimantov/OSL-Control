"""
Author: Sarit Nagel
Date: 09/07/2026

This module implements an adapter for the AS5048A absolute encoder using the OSL (Open Source Leg) framework.
It synchronizes with an ODrive motor to perform a calibration and trapezoidal trajectory move.

Pin Cpnnections: (AS5048A to Raspberry Pi)
- GND -> Pin 6 (GND)
- VDD5V -> Pin 1 (3.3V)
- MISO -> Pin 35 (SPI0 MISO)
- MOSI -> Pin 38 (SPI0 MOSI)
- CLK -> Pin 40 (SPI0 SCLK)
- CSn -> Pin 36 (GPIO16)
"""

import math
import time
import spidev
import odrive
from odrive.enums import *
from typing import Optional, Any, Dict

# OSL Imports
from opensourceleg.sensors.base import EncoderBase
from opensourceleg.actuators import CONTROL_MODES
from opensourceleg.logging import LOGGER

class AS5048AEncoder(EncoderBase):
    """
    Implementation of the OSL Encoder class for the AS5048A magnetic encoder via SPI.
    """
    def __init__(
        self,
        tag: str,
        bus: int = 1,
        device: int = 2,
        gear_ratio: float = 1.0,
        frequency: int = 1000,
        offline: bool = False,
        **kwargs
    ):
        super().__init__(
            tag=tag,
            gear_ratio=gear_ratio,
            frequency=frequency,
            offline=offline,
            **kwargs
        )
        self._bus = bus
        self._device = device
        self._spi = None
        self._streaming = False

        self._raw_angle_rad = 0.0
        self._velocity_rad_s = 0.0
        self._last_angle_rad = 0.0
        self._last_timestamp = time.time()

        if not self.is_offline:
            try:
                self._spi = spidev.SpiDev()
                self._spi.open(self._bus, self._device)
                self._spi.max_speed_hz = 100000 
                self._spi.mode = 1
                LOGGER.info(f"[{self.tag}] AS5048A SPI Initialized.")
            except Exception as e:
                LOGGER.error(f"[{self.tag}] Failed to initialize SPI: {e}")
                self._offline = True

    def start(self):
        # Enable data streaming from the encoder
        self._streaming = True

    def stop(self):
        # Disable data streaming from the encoder
        self._streaming = False

    # Check if the encoder is currently streaming data
    @property
    def is_streaming(self) -> bool:
        return self._streaming

    # Get the current angular position in radians
    @property
    def position(self) -> float:
        return self._raw_angle_rad

    # Get the current angular velocity in radians per second
    @property
    def velocity(self) -> float:
        return self._velocity_rad_s

    # Returns a dictionary containing the current position and velocity
    @property
    def data(self) -> Dict[str, Any]:
        return {"position": self.position, "velocity": self.velocity}

    # Alias for position
    @property
    def q(self) -> float:
        return self.position

    # Alias for velocity
    @property
    def qdot(self) -> float:
        return self.velocity

    # Updates the encoder's position and velocity by reading from the AS5048A via SPI
    def update(self) -> None:
        """Reads the absolute angular position from the AS5048A."""
        current_time = time.time()
        dt = current_time - self._last_timestamp

        if not self.is_offline:
            try:
                self._spi.xfer2([0xFF, 0xFF])
                time.sleep(0.0001) 
                resp = self._spi.xfer2([0xFF, 0xFF])
                raw_val = ((resp[0] & 0x3F) << 8) | resp[1]
                
                # Convert to Radians (14-bit resolution)
                self._raw_angle_rad = (raw_val * 2.0 * math.pi) / 16384.0
                
                if dt > 0:
                    diff = self._raw_angle_rad - self._last_angle_rad
                    if diff > math.pi: diff -= 2.0 * math.pi
                    elif diff < -math.pi: diff += 2.0 * math.pi
                    self._velocity_rad_s = diff / dt

                self._last_angle_rad = self._raw_angle_rad
                self._last_timestamp = current_time
            except Exception as e:
                LOGGER.error(f"[{self.tag}] SPI Read Error: {e}")

    # Placeholder for zeroing the encoder, currently does nothing
    def zero(self) -> None:
        pass

    # Safely closes the SPI connection when the encoder is no longer needed
    def close(self) -> None:
        if self._spi is not None:
            self._spi.close()

# --- Main Test Execution  ---
# Run the control loop to sync ODrive motion with OSL encoder feedback

def run_odrive_with_osl_encoder():
    """
    Synchronizes ODrive and AS5048A using the OSL class structure.
    """
    TORQUE_CONSTANT = 0.0827
    
    # 1. Initialize the Encoder Class
    encoder = AS5048AEncoder(tag="Ankle_Angle", bus=1, device=2)
    encoder.start()

    # 2. ODrive Setup 
    print("Searching for ODrive...")
    my_drive = odrive.find_any()
    print("Connected!")

    my_drive.axis0.requested_state = AxisState.IDLE
    my_drive.clear_errors()
    
    my_drive.axis0.config.motor.pole_pairs = 21
    my_drive.axis0.config.motor.torque_constant = TORQUE_CONSTANT
    my_drive.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
    my_drive.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ

    # Path Smoothness
    my_drive.axis0.trap_traj.config.vel_limit = 1.0     
    my_drive.axis0.trap_traj.config.accel_limit = 0.5   
    my_drive.axis0.trap_traj.config.decel_limit = 0.5

    # 3. Calibration
    print("Starting Calibration...")
    my_drive.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    while my_drive.axis0.current_state != AxisState.IDLE:
        time.sleep(0.1)
    print("Calibration Complete.")

    # 4. Engage
    my_drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    # 5. Move and Log
    if my_drive.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
        # Determine target position based on current position
        start_pos = my_drive.axis0.pos_estimate
        if start_pos > 0.0:
            target = start_pos - 2.0
        else:
            target = start_pos + 2.0
        print(f"Moving from {start_pos:.3f} to {target:.3f}")
        
        # Log Header
        print("-" * 85)
        print(f"{'Time (s)':<10} | {'Position (Turns)':<15} | {'Angle (Deg)':<15} | {'Motor Torque (Nm)':<15}")
        print("-" * 85)
        
        my_drive.axis0.controller.input_pos = target
        start_t = time.time()
        
        try:
            for _ in range(150):
                # Update Encoder Object
                encoder.update()
                
                # Get ODrive Data
                motor_pos = my_drive.axis0.pos_estimate
                measured_iq = my_drive.axis0.motor.foc.Iq_measured 
                calc_torque = measured_iq * TORQUE_CONSTANT
                
                # Get Encoder Data (Converted back to degrees for the display)
                ankle_deg = math.degrees(encoder.q)
                
                elapsed = time.time() - start_t
                print(f"{elapsed:<10.2f} | {motor_pos:<15.3f} | {ankle_deg:<15.2f} | {calc_torque:<15.4f}")
                
                if abs(motor_pos - target) < 0.05:
                    print("-" * 85)
                    print("Target reached!")
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    # 6. Cleanup
    my_drive.axis0.requested_state = AxisState.IDLE
    encoder.stop()
    encoder.close()
    print("Done.")

# Entry point for script execution
if __name__ == "__main__":
    run_odrive_with_external_encoder = run_odrive_with_osl_encoder # Alias for consistency
    run_odrive_with_osl_encoder()

    

    