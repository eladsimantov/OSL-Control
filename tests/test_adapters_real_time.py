# !/usr/bin/env python3
"""
test_adapters.py - This module tests the adapters library. This library is meant to provide an interface between the OSL API and our custom hardware drivers, allowing us to use OSL's architecture and control modes while interfacing with our specific hardware.
The tests in this module will focus on verifying that all classes are properly initialized, that the methods are correctly implemented, and that the integration with the OSL API is seamless. 
We will also include some basic functionality tests to ensure that the actuators can be controlled and that sensors give readings.

Author: Elad Siman Tov
Date: 2026-01-01
"""

import os
import sys
import time
tests_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.join(tests_path, "..")
sys.path.insert(0, project_path)
from src.drivers.odrive_can import ODriveCAN, ODriveMotor
# from opensourceleg.actuators import ActuatorBase, MOTOR_CONSTANTS, CONTROL_MODES
from src.adapters.imu import WitMotionIMUAdapter
# from opensourceleg.sensors.imu import BNO055
from src.adapters.loadcell import SRILoadCell_M8123B2

from opensourceleg.utilities import SoftRealtimeLoop


if __name__ == "__main__":
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    motor_node_id = 0
    CAN_CH = 'can0'
    BITRATE = 1000000  # Default is 1Mb/s 
    # USB2CAN adapter settings (Must be done before ODriveCAN connects)
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE} sample-point 0.750")
    os.system(f"sudo ip link set {CAN_CH} txqueuelen 1000") # Set the queue length to 1000 so the USB buffer doesn't overflow

    can1 = ODriveCAN(node_id=motor_node_id,dbc_path=dbc_path,bus_name=CAN_CH)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40) 

    # Initialize loadcell (uses CAN filters so ODrive traffic is ignored)
    loadcell = SRILoadCell_M8123B2(tag="Shank LC", channel=CAN_CH)
    loadcell.start()
    loadcell.calibrate()

    # Initialize WitMotion IMU adapters
    # Mode 1: Direct BLE connection (Recommended - no manual or sudo rfcomm commands needed)
    os.system(f"sudo rfkill block bluetooth") # restart Bluetooth on Pi
    os.system(f"sudo rfkill unblock bluetooth") # restart Bluetooth on Pi
    thigh_imu = WitMotionIMUAdapter(tag="Thigh IMU", mac_address="EF:D5:AC:1A:0D:21", connection_type="ble")
    foot_imu = WitMotionIMUAdapter(tag="Foot IMU", mac_address="EC:8E:70:CE:63:24", connection_type="ble")

    try:
        print("[STATUS] Connecting to Bluetooth IMU on /dev/rfcomm0...")
        thigh_imu.start()
        foot_imu.start()
        if not thigh_imu.is_streaming or not foot_imu.is_streaming:
            raise ConnectionError("IMU started but is not streaming.")
        print("[STATUS] SUCCESS: Connected to WitMotion IMU via Bluetooth.")
    except (ImportError, NotImplementedError, Exception) as e:
        print(f"[STATUS] HARDWARE/OS ERROR: {e}")
        print("[STATUS] FALLBACK: Starting in OFFLINE MODE (Simulation).")
        thigh_imu._offline = True
        foot_imu._offline = True
        thigh_imu.start()
        foot_imu.start()

    print("[STATUS] Calibrating IMUs... Keep them stationary.")
    thigh_imu.calibrate()
    foot_imu.calibrate()
    print("[STATUS] IMU calibration complete.")

    # --- One-time motor setup (before the loop) ---
    knee.idle()
    time.sleep(0.3)
    knee.set_limit_current(10, 30)
    knee.closed_loop()
    knee.torque_control()

    loop = SoftRealtimeLoop(dt=0.01)
    try: 
        for t in loop:
            # Read loadcell and IMU (non-blocking)
            loadcell.update()
            thigh_imu.update()
            foot_imu.update()

            # Send torque command every iteration (ready for impedance control)
            # knee.set_motor_torque(-0.1)
            
            knee.set_impedance(kp=0.02, kd=0.0006, deg_eq=0.0)
            # knee.set_impedance(kp=0.02, kd=0.0006, deg_eq=0.0, pos_deg=knee.get_position(), vel_dps=knee.get_position())

            # Print loadcell and IMU data in-place (carriage return avoids scroll overhead)
            if round(t/0.01)%10 == 0:
                print(f"\r t={t:6.2f}s | "
                      f"F(xyz) N: [{loadcell.fx:7.2f}, {loadcell.fy:7.2f}, {loadcell.fz:7.2f}] | "
                      f"M(xyz) Nm: [{loadcell.mx:7.3f}, {loadcell.my:7.3f}, {loadcell.mz:7.3f}] | "
                      f"Thigh (deg): [{thigh_imu.euler_x:6.2f}, {thigh_imu.euler_y:6.2f}, {thigh_imu.euler_z:6.2f}] | "
                      f"Foot (deg): [{foot_imu.euler_x:6.2f}, {foot_imu.euler_y:6.2f}, {foot_imu.euler_z:6.2f}] | "
                      f"CAN rx: {loadcell._last_update_count}", end='   ')

    finally:
        # This block ALWAYS runs — whether loop exits via:
        #   - LoopKiller signal (Ctrl+C → SoftRealtimeLoop swallows SIGINT)
        #   - loop.stop()
        #   - Any exception
        print("\n\nStopping loop and moving Knee to Idle mode")
        knee.idle()
        import time
        time.sleep(0.2)  # Give CAN bus time to transmit the idle command before shutdown
        loadcell.stop()
        thigh_imu.stop()
        foot_imu.stop()
        os.system(f"sudo ip link set {CAN_CH} down")    
        
    # thighIMU= BNO055(tag="Timu", addr=40, offline=False)
    # thighIMU = IMUAdapter(tag="Timu", address=40, offline=True)
    # from opensourceleg.actuators import ActuatorBase, MOTOR_CONSTANTS, CONTROL_MODES
    # from src.drivers.odrive_can import ODriveMotor, ODriveCAN
    # import numpy as np

    # can1 = ODriveCAN(node_id=1)
    # kneeActuator = ODriveActuator(
    #             tag="act1"
    #             can_interface=can1,
    #             gear_ratio=100,
    #             motor_constants=MOTOR_CONSTANTS(2048, 0.02, 0.001, 0.0001, 80.0, 120.0)
    # )




    # imu = BNO055(tag="imu1", bus=1, address=0x28)
    # encoder = AS5048B(tag="encoder1", bus=1, address=0x40)

    # OSK("OSK", actuators={"knee": kneeActuator}, sensors={  "loadcell": loadcell, "imu": imu, "encoder": encoder })

