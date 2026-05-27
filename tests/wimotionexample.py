import asyncio
import pywitmotion as wit
from bleak import BleakClient

MAC_ADDRESS = "EF:D5:AC:1A:0D:21"
DATA_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

def notification_handler(sender, data):
    # Split incoming stream by the 'U' (0x55) header flag
    messages = data.split(b'U')
    
    for msg in messages:
        if not msg or len(msg) < 2:
            continue
            
        # If it is the high-speed BLE block (starts with 0x61 / b'a')
        if msg[0] == 0x61:
            # Strip the 0x61 flag to get the raw payload
            payload = msg[1:]
            
            if len(payload) >= 18:
                # 1. Reconstruct Acceleration Packet (Header 0x51 + first 8 bytes)
                accel_packet = b'\x51' + payload[0:8]
                
                # 2. Reconstruct Gyroscope Packet (Header 0x52 + next 8 bytes)
                gyro_packet = b'\x52' + payload[8:16]
                
                # Pass your newly formatted blocks into pywitmotion!
                accel = wit.get_acceleration(accel_packet)
                gyro = wit.get_gyro(gyro_packet)
                
                if accel is not None:
                    print(f"[ACCEL] X: {accel[0]:+.3f}g | Y: {accel[1]:+.3f}g | Z: {accel[2]:+.3f}g")
                if gyro is not None:
                    print(f"[GYRO]  X: {gyro[0]:+.2f}°/s | Y: {gyro[1]:+.2f}°/s | Z: {gyro[2]:+.2f}°/s")
                    print("-" * 50)
        else:
            # If the sensor drops out of high-speed mode and sends standard 9-byte packets
            if len(msg) >= 9:
                accel = wit.get_acceleration(msg)
                if accel is not None:
                    print(f"[REGULAR ACCEL] {accel}")

async def main():
    print(f"Connecting to IMU via Bleak...")
    async with BleakClient(MAC_ADDRESS) as client:
        if client.is_connected:
            print("Connected! Feeding processed streams to pywitmotion...\n")
            await client.start_notify(DATA_UUID, notification_handler)
            while True:
                await asyncio.sleep(1)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("\nExiting script.")
except Exception as e:
    print(f"\nBLE Error: {e}")
