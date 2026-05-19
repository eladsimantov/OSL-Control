"""
Modified example for controlling an ODrive via the CANSimple protocol.

Moves the motor to a specific position using the Set_Input_Pos message.
"""

import can
import time
import struct

node_id = 0  # Must match `<odrv>.axis0.config.can.node_id`. Default is 0.

bus = can.interface.Bus("can0", interface="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

# Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
    data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

# Wait for axis to enter closed loop control by scanning heartbeat messages
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
            print("Axis is in CLOSED_LOOP_CONTROL state.")
            break

# Move motor to a specific position
target_position = 10.0  # Target position in turns
velocity_ff = 0.0       # Velocity feedforward
torque_ff = 0.0         # Torque feedforward

print(f"Sending position command: {target_position} turns")
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0C),  # 0x0C: Set_Input_Pos
    data=struct.pack('<fhh', target_position, int(velocity_ff * 1000), int(torque_ff * 1000)),
    is_extended_id=False
))

# Wait for a few seconds to allow the motor to reach the position
time.sleep(5)

print("Position command sent. Check motor movement.")