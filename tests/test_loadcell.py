#!/usr/bin/env python3
import os
import sys

# Path setup to include project root
tests_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.join(tests_path, "..")
sys.path.insert(0, project_path)

from src.adapters.loadcell import SRILoadCell_M8123B2
from opensourceleg.utilities import SoftRealtimeLoop

if __name__ == "__main__":
    CAN_CH = 'can0'
    BITRATE = 1000000  # Default is 1Mb/s 

    # USB2CAN adapter settings
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE} sample-point 0.750")
    os.system(f"sudo ip link set {CAN_CH} txqueuelen 1000") # Set the queue length to 1000 so the USB buffer doesn't overflow

    # Initialize loadcell
    loadcell = SRILoadCell_M8123B2(tag="Shank LC", channel=CAN_CH)
    loadcell.start()
    loadcell.calibrate()

    print(f"\n-> Sensor: {loadcell.tag}")
    print("\n" + "-"*50)
    print("  LIVE 6-AXIS MONITORING (Ctrl+C to stop) ")
    print("-"*50)

    loop = SoftRealtimeLoop(dt=0.01)
    try:
        for t in loop:
            # Refresh data from CAN
            loadcell.update() 
            
            # Print Force (N) and Moments (Nm) and update count every 10 loops
            if round(t/0.01)%10 == 0:
                print(f"\r t={t:6.2f}s | "
                      f"F(xyz) N: [{loadcell.fx:7.2f}, {loadcell.fy:7.2f}, {loadcell.fz:7.2f}] | "
                      f"M(xyz) Nm: [{loadcell.mx:7.3f}, {loadcell.my:7.3f}, {loadcell.mz:7.3f}] | "
                      f"CAN rx: {loadcell._last_update_count}", end='   ')

    finally:
        # Safe Shutdown
        loadcell.stop()
        os.system(f"sudo ip link set {CAN_CH} down")
        print("\n[INFO] Bus closed. Cleanup complete.")