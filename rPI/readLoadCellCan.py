import can

# Initialize the bus on can0 (standard for most RPi shields)
bus = can.interface.Bus(channel='can1', bustype='socketcan')

try:
    print("Listening for messages...")
    while True:
        msg = bus.recv()
        if msg:
            print(f"ID: {msg.arbitration_id:X} Data: {msg.data.hex()}")
except KeyboardInterrupt:
    bus.shutdown()