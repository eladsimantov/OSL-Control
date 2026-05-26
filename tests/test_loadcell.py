#!/usr/bin/env python3
import os
import sys
import time

# # Path setup to include project root
# tests_path = os.path.dirname(os.path.abspath(__file__))
# project_path = os.path.join(tests_path, "..")
# sys.path.insert(0, project_path)

from src.adapters.loadcell import SRILoadCell_M8123B2


def setup_can():
    # --- Configurat2ion ---
    CAN_CH = 'can2'
    BITRATE = 1000000  # Default is 1Mb/s 
    ID_QUERY = 0x80    # ID #1: Master to Sensor [cite: 483]
    ID_REPLY = [0x291, 0x292, 0x293] # IDs #2, #3, #4 [cite: 485, 487, 489]

    # CRATE, CFIDL and CTXIDL
    # # CAN SHIELD SETTINGS
    # os.system(f"sudo ip link set {CAN_CH} down")
    # os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE}")

    # USB2CAN adapter settings
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE} sample-point 0.750")
    os.system(f"sudo ip link set {CAN_CH} txqueuelen 1000") # Set the queue length to 1000 so the USB buffer doesn't overflow

    return 

def test_loadcell():
    print("\n" + "="*50)
    print("  SRI M8123B2 ADAPTER TEST (FORCE & MOMENT) ")
    print("="*50 + "\n")

    # 1. Hardware Connection Attempt
    try:
        # Defaulting to online mode (offline=False)
        loadcell = SRILoadCell_M8123B2(tag="Shank LC", channel="can2")
        loadcell.start()
        
        # Check if the SocketCAN bus actually opened
        if not loadcell.is_streaming:
            raise ConnectionError("CAN bus not responding.")
            
        print("[STATUS] SUCCESS: Connected to SRI Load Cell via CAN.")

    except Exception as e:
        # 2. Fallback to Offline Mode
        print(f"[STATUS] HARDWARE ERROR: {e}")
        print("[STATUS] FALLBACK: Starting in OFFLINE MODE (Simulation).")
        
        # Re-initialize as an OSL offline sensor
        loadcell = SRILoadCell_M8123B2(tag="Shank LC", offline=True)
        loadcell.start()

    # 3. Calibration (Zeroing)
    # n_samples is reduced for a quicker offline check
    samples = 1000 if not loadcell.is_offline else 10
    loadcell.calibrate(n_samples=samples)

    print(f"\n-> Sensor: {loadcell.tag} | Offline: {loadcell.is_offline}")
    print("\n" + "-"*50)
    print("  LIVE 6-AXIS MONITORING (Ctrl+C to stop) ")
    print("-"*50)

    try:
        while True:
            # Refresh data from CAN
            loadcell.update() 
            
            # Print Force (N) and Moments (Nm)
            # Moments use properties mx, my, mz from your class
            print(f"F(xyz) N: [{loadcell.fx:6.2f}, {loadcell.fy:6.2f}, {loadcell.fz:6.2f}] | "
                  f"M(xyz) Nm: [{loadcell.mx:6.2f}, {loadcell.my:6.2f}, {loadcell.mz:6.2f}]", end='\n')
            

    except KeyboardInterrupt:
        print("\n\n[INFO] Stop signal received.")
    finally:
        # 4. Safe Shutdown
        loadcell.stop()
        print("[INFO] Bus closed. Cleanup complete.")

if __name__ == "__main__":
    setup_can()
    test_loadcell()