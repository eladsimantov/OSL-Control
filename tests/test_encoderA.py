"""
Author: Sarit Nagel
Date: 09/07/2026

This script is used to interface with an AS5048A absolute rotary encoder 
using SPI communication on a Raspberry Pi. 
It reads the 14-bit angular position from the encoder and converts 
it into degrees for real-time monitoring. 

Pin Cpnnections: (AS5048A to Raspberry Pi)
- GND -> Pin 6 (GND)
- VDD5V -> Pin 1 (3.3V)
- MISO -> Pin 35 (SPI0 MISO)
- MOSI -> Pin 38 (SPI0 MOSI)
- CLK -> Pin 40 (SPI0 SCLK)
- CSn -> Pin 36 (GPIO16)
"""
import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
# Use (1, 2) which matches your CSN Pin 36 setup
spi.open(1, 2) 
spi.max_speed_hz = 100000 
spi.mode = 1

def get_angle():
    """
    Communicates with the AS5048A over SPI to retrieve the raw 14-bit 
    angle value. It sends a read request for the angle register (0x3FFF) 
    and processes the parity and error bits.
    """
    # Step 1: Send request for angle (Register 0x3FFF)
    # The AS5048A requires the Parity bit to be 1 for a read command, 
    # so 0x3FFF becomes 0xFFFF.
    spi.xfer2([0xFF, 0xFF])
    
    # Very short pause for the chip to prepare the data
    time.sleep(0.0001)
    
    # Step 2: Read the result
    resp = spi.xfer2([0xFF, 0xFF])
    
    # Combine bytes: 
    # resp[0] is High Byte, resp[1] is Low Byte
    # We use & 0x3F to remove the Parity (bit 15) and Error (bit 14) flags
    raw_val = ((resp[0] & 0x3F) << 8) | resp[1]
    return raw_val

if __name__ == "__main__":
    """
    Main loop that continuously reads data from the encoder, calculates 
    the angle in degrees based on the 16,384 step resolution, and 
    displays the status until interrupted by the user.
    """
    try:
        print("AS5048A Reading Started...")
        print("Rotate the magnet to see the angle change.")
        while True:
            raw = get_angle()
            
            # Calculate degrees: 14-bit resolution is 16,384 steps
            degrees = (raw * 360.0) / 16384.0
            
            if raw == 0:
                status = "Check Magnet/Wiring"
            else:
                status = "OK"

            print(f"Raw: {raw:5} | Angle: {degrees:6.2f}° | Status: {status}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        spi.close()
        print("\nStopped by user.")

