import os
import can
import time
import struct

node_id = 2 # Must match `<odrv>.axis0.config.can.node_id`. Default is 0.

# Ensure CAN interface is up
os.system("sudo ip link set can0 up type can bitrate 1000000")

try:
    bus = can.interface.Bus("can0", interface="socketcan")
    # Put axis into closed loop control state
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
        data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    print("Axis set to CLOSED_LOOP_CONTROL state.")

    # # Move motor to a specific position
    # target_position = 30.0  # Target position in turns
    # velocity_ff = 1.0       # Velocity feedforward
    # torque_ff = 2.0         # Torque feedforward

    # print(f"Sending position command: {target_position} turns")
    # bus.send(can.Message(
    #     arbitration_id=(node_id << 5 | 0x0C),  # 0x0C: Set_Input_Pos
    #     data=struct.pack('<fhh', target_position, int(velocity_ff * 1000), int(torque_ff * 1000)),
    #     is_extended_id=False
    # ))

    # Wait for a few seconds to allow the motor to reach the position
    time.sleep(15)

    # print("Position command sent. Check motor movement.")

finally:
    if 'bus' in locals():
        bus.shutdown()
        print("CAN bus shut down properly.")