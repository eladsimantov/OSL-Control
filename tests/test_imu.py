#!/usr/bin/env python3
import os
import sys
import time

# # Path setup to include project root
# tests_path = os.path.dirname(os.path.abspath(__file__))
# project_path = os.path.join(tests_path, "..")
# sys.path.insert(0, project_path)

from src.adapters.imu import BNO055Adapter

def test_imu():
    print("\n" + "="*50)
    print("      BNO055 IMU ADAPTER TEST (6-AXIS) ")
    print("="*50 + "\n")

    # 1. Hardware Connection Attempt
    # We initialize it with hardware-intent first
    imu = BNO055Adapter(tag="Ankle IMU", addr=0x28, offline=False)
    
    try:
        # In our new adapter, .start() contains the dangerous 'board' imports.
        # It will fail here on Windows, and we catch that failure.
        imu.start()
        
        if not imu.is_streaming:
            raise ConnectionError("IMU started but is not streaming.")
            
        print("[STATUS] SUCCESS: Connected to BNO055 IMU via I2C.")

    except (ImportError, NotImplementedError, Exception) as e:
        # 2. Fallback to Offline Mode for Windows/Teammates
        print(f"[STATUS] HARDWARE/OS ERROR: {e}")
        print("[STATUS] FALLBACK: Starting in OFFLINE MODE (Simulation).")
        
        # We don't need to re-init; just set the offline flag and start again
        imu._offline = True
        imu.start()

    print(f"\n-> Sensor: {imu.tag} | Mode: {'Offline' if imu.is_offline else 'Online'}")
    print("\n" + "-"*50)
    print("  LIVE IMU MONITORING (Ctrl+C to stop) ")
    print("-"*50)

    try:
        while True:
            # Poll the sensor (online reads hardware, offline does nothing)
            imu.update() 
            
            # Display 6-Axis Data
            print(f"Acc  (m/s^2): [{imu.acc_x:6.2f}, {imu.acc_y:6.2f}, {imu.acc_z:6.2f}] | "
                f"Gyro (rad/s): [{imu.gyro_x:6.2f}, {imu.gyro_y:6.2f}, {imu.gyro_z:6.2f}]", end='\r')
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n[INFO] Test stopped by user.")
    finally:
        # 3. Safe Shutdown
        imu.stop()
        print("[INFO] Cleanup complete. Sensor stopped.")

if __name__ == "__main__":
    test_imu()