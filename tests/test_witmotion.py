#!/usr/bin/env python3
import os
import sys
import time

# Path setup to include project root
tests_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.join(tests_path, "..")
sys.path.insert(0, project_path)

from src.adapters.imu import WitMotionIMUAdapter

def test_witmotion():
    print("\n" + "="*50)
    print("      WITMOTION BLUETOOTH IMU TEST ")
    print("="*50 + "\n")

    # Initialize the Bluetooth adapter (connects to /dev/rfcomm0 by default)
    # We initialize it with hardware-intent first (online mode)
    imu = WitMotionIMUAdapter(tag="Body IMU", port="/dev/rfcomm0", offline=False)
    
    try:
        print("[STATUS] Connecting to Bluetooth IMU on /dev/rfcomm0...")
        imu.start()
        
        if not imu.is_streaming:
            raise ConnectionError("IMU started but is not streaming.")
            
        print("[STATUS] SUCCESS: Connected to WitMotion IMU via Bluetooth.")

    except (ImportError, NotImplementedError, Exception) as e:
        # Fallback to Offline Mode for Windows/Teammates
        print(f"[STATUS] HARDWARE/OS ERROR: {e}")
        print("[STATUS] FALLBACK: Starting in OFFLINE MODE (Simulation).")
        
        # Set offline flag and start again
        imu._offline = True
        imu.start()

    print(f"\n-> Sensor: {imu.tag} | Mode: {'Offline' if imu.is_offline else 'Online'}")
    print("\n" + "-"*50)
    print("  LIVE MONITORING (Ctrl+C to stop) ")
    print("-"*50)

    try:
        while True:
            # Poll sensor data
            imu.update() 
            
            # Print live orientation data (quaternions and angles)
            print(f"Euler (deg): [{imu.euler_x:6.2f}, {imu.euler_y:6.2f}, {imu.euler_z:6.2f}] | "
                  f"Quat: [{imu.quat_w:5.2f}, {imu.quat_x:5.2f}, {imu.quat_y:5.2f}, {imu.quat_z:5.2f}] | "
                  f"Acc: [{imu.acc_x:5.2f}, {imu.acc_y:5.2f}, {imu.acc_z:5.2f}]", end='\r')
            
            time.sleep(0.02)  # Poll at 50Hz

    except KeyboardInterrupt:
        print("\n\n[INFO] Test stopped by user.")
    finally:
        imu.stop()
        print("[INFO] Cleanup complete. Sensor stopped.")

if __name__ == "__main__":
    test_witmotion()
