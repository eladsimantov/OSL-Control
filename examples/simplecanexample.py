import can
import struct

node_id = 0  # Update if necessary
bus = can.interface.Bus("can0", interface="socketcan")

# Send position command
target_position = 10.0  # Target position in turns
velocity_ff = 0.0       # Velocity feedforward
torque_ff = 0.0         # Torque feedforward

print(f"Sending position command: {target_position} turns")
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0C),  # 0x0C: Set_Input_Pos
    data=struct.pack('<fhh', target_position, int(velocity_ff * 1000), int(torque_ff * 1000)),
    is_extended_id=False
))