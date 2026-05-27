import asyncio
import struct
from bleak import BleakClient
import pywitmotion as wit

MAC_ADDRESS = "EF:D5:AC:1A:0D:21"
DATA_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

def notification_handler(sender, data):
    # 1. Print the raw hexadecimal bytes so you can see the active traffic stream
    hex_string = " ".join(f"{b:02X}" for b in data)
    print(f"Raw Packet ({len(data)} bytes): {hex_string}")

    data = data.split(b'U') 
    print(data)
    for msg in data:
        q = wit.get_quaternion(msg)
        print(q)
    
    # 2. Extract and decode the bytes if it follows the 11-byte WitMotion format
    if len(data) >= 11 and data[0] == 0x55:
        packet_type = data[1]
        
        # Unpack payload bytes (bytes 2 to 7) into 3 signed 16-bit integers
        try:
            raw_x, raw_y, raw_z = struct.unpack('<hhh', data[2:8])
            
            if packet_type == 0x51:    # Acceleration
                ax = raw_x / 32768.0 * 16.0
                ay = raw_y / 32768.0 * 16.0
                az = raw_z / 32768.0 * 16.0
                print(f" -> [ACCEL] X: {ax:+.3f}g | Y: {ay:+.3f}g | Z: {az:+.3f}g\n")
                
            elif packet_type == 0x52:  # Gyroscope
                gx = raw_x / 32768.0 * 2000.0
                gy = raw_y / 32768.0 * 2000.0
                gz = raw_z / 32768.0 * 2000.0
                print(f" -> [GYRO]  X: {gx:+.2f}°/s | Y: {gy:+.2f}°/s | Z: {gz:+.2f}°/s\n")
                
            elif packet_type == 0x53:  # Orientation Angles
                roll  = raw_x / 32768.0 * 180.0
                pitch = raw_y / 32768.0 * 180.0
                yaw   = raw_z / 32768.0 * 180.0
                print(f" -> [ANGLE] Roll: {roll:+.2f}° | Pitch: {pitch:+.2f}° | Yaw: {yaw:+.2f}°\n")
                
        except struct.error:
            print(" -> Error unpacking packet data structure.")
    else:
        print(" -> Data received, but does not match standard 11-byte 0x55 header.\n")

async def main():
    print(f"Connecting to IMU at {MAC_ADDRESS}...")
    async with BleakClient(MAC_ADDRESS) as client:
        if client.is_connected:
            print("Connected! Listening for live data streams...\n")
            await client.start_notify(DATA_UUID, notification_handler)
            
            # Keep running to listen for streams
            while True:
                await asyncio.sleep(1)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("\nExiting script.")
except Exception as e:
    print(f"\nConnection Error: {e}")
