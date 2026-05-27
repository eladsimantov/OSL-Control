import asyncio
import struct
from bleak import BleakClient

# Configuration constants
MAC_ADDRESS = "EF:D5:AC:1A:0D:21"
DATA_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

def notification_handler(sender, data):
    """
    Direct interrupt callback. Unpacks the data immediately 
    without string handling, regex, or external libraries.
    """
    # 0x55 0x61 indicates WitMotion high-speed combined broadcast frame
    if len(data) >= 20 and data[0] == 0x55 and data[1] == 0x61:
        try:
            # Struct reads 9 signed 16-bit integers (<hhhhhhhhh) from memory slice 2 to 20
            ax, ay, az, gx, gy, gz, roll_raw, pitch_raw, yaw_raw = struct.unpack('<hhhhhhhhh', data[2:20])
            
            # Apply official hardware multiplication/shift coefficients 
            accel_x = ax / 32768.0 * 16.0
            accel_y = ay / 32768.0 * 16.0
            accel_z = az / 32768.0 * 16.0
            
            gyro_x = gx / 32768.0 * 2000.0
            gyro_y = gy / 32768.0 * 2000.0
            gyro_z = gz / 32768.0 * 2000.0
            
            roll  = roll_raw / 32768.0 * 180.0
            pitch = pitch_raw / 32768.0 * 180.0
            yaw   = yaw_raw / 32768.0 * 180.0
            
            # Print string formatting template optimized to maintain low terminal latency
            print(
                f"[ACCEL] X:{accel_x:+.2f}g Y:{accel_y:+.2f}g Z:{accel_z:+.2f}g | "
                f"[GYRO] X:{gyro_x:+.1f}°/s Y:{gyro_y:+.1f}°/s Z:{gyro_z:+.1f}°/s | "
                f"[ANGLE] R:{roll:+.1f}° P:{pitch:+.1f}° Y:{yaw:+.1f}°      ", 
                end="\r"
            )
            
        except struct.error:
            pass

async def main():
    print(f"Direct connection request sent to: {MAC_ADDRESS}")
    async with BleakClient(MAC_ADDRESS) as client:
        if client.is_connected:
            print("Connected! Streaming 9-axis state fields at max speed...\n")
            
            # Register hardware handler profile
            await client.start_notify(DATA_UUID, notification_handler)
            
            # Simple non-blocking sleep loop to yield thread time to incoming BLE signals
            while True:
                await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting real-time display loop.")
    except Exception as e:
        print(f"\nBLE Connection Error: {e}")
