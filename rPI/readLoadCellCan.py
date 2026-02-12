import os
import can
import struct
import time

# --- Configuration ---
CAN_CH = 'can1'
BITRATE = 1000000  # Default is 1Mb/s 
ID_QUERY = 0x80    # ID #1: Master to Sensor [cite: 483]
ID_REPLY = [0x291, 0x292, 0x293] # IDs #2, #3, #4 [cite: 485, 487, 489]

def setup_can():
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE}")
    return can.interface.Bus(channel=CAN_CH, bustype='socketcan')

def main():
    bus = setup_can()
    
    # Send 0x02 to ID #1 to start continuous data [cite: 494, 495]
    start_msg = can.Message(arbitration_id=ID_QUERY, data=[0x02], is_extended_id=False)
    bus.send(start_msg)
    
    data = {"FX": 0.0, "FY": 0.0, "FZ": 0.0, "MX": 0.0, "MY": 0.0, "MZ": 0.0}
    
    try:
        while True:
            msg = bus.recv()
            if msg is None: continue
            
            # Unpack 4-byte Floats [cite: 501, 502]
            if msg.arbitration_id == 0x291:
                data["FX"], data["FY"] = struct.unpack('<ff', msg.data)
            elif msg.arbitration_id == 0x292:
                data["FZ"], data["MX"] = struct.unpack('<ff', msg.data)
            elif msg.arbitration_id == 0x293:
                data["MY"], data["MZ"] = struct.unpack('<ff', msg.data)
                
                # Print full set once ID #4 is received [cite: 498]
                print(f"Force: {data['FX']:7.2f} {data['FY']:7.2f} {data['FZ']:7.2f} | "
                      f"Torque: {data['MX']:7.2f} {data['MY']:7.2f} {data['MZ']:7.2f}")

    except KeyboardInterrupt:
        # Send 0x00 to stop data [cite: 496]
        bus.send(can.Message(arbitration_id=ID_QUERY, data=[0x00], is_extended_id=False))
        bus.shutdown()

if __name__ == "__main__":
    main()