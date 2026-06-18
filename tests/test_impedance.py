
import os
import sys
import time
tests_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.join(tests_path, "..")
sys.path.insert(0, project_path)
from src.drivers.odrive_can_updated import ODriveCAN, ODriveMotor
# from opensourceleg.actuators import ActuatorBase, MOTOR_CONSTANTS, CONTROL_MODES
# from src.adapters.imu import IMUAdapter
# from opensourceleg.sensors.imu import BNO055
from src.adapters.loadcell import SRILoadCell_M8123B2

from opensourceleg.utilities import SoftRealtimeLoop


if __name__ == "__main__":
    dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
    motor_node_id = 0
    CAN_CH = 'can0'
    BITRATE = 1000000  # Default is 1Mb/s 
    can1 = ODriveCAN(node_id=motor_node_id,dbc_path=dbc_path,bus_name=CAN_CH)
    knee = ODriveMotor(can1, name="knee", gear_ratio=40) 

    # USB2CAN adapter settings
    os.system(f"sudo ip link set {CAN_CH} down")
    os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE} sample-point 0.750")
    os.system(f"sudo ip link set {CAN_CH} txqueuelen 1000") # Set the queue length to 1000 so the USB buffer doesn't overflow

    # Initialize loadcell (uses CAN filters so ODrive traffic is ignored)
    loadcell = SRILoadCell_M8123B2(tag="Shank LC", channel=CAN_CH)
    loadcell.start()
    loadcell.calibrate()


    # --- One-time motor setup (before the loop) ---
    knee.calibrate()
    knee.idle()
    knee.set_limit_current(10, 30)
    knee.closed_loop()
    knee.torque_control()

    loop = SoftRealtimeLoop(dt=0.01)

    state = "free"

    try: 
       
        for t in loop:
            loadcell.update()

            if loadcell.fz > 30:
                state = "stance"
            
            if state == "stance":
                if loadcell.fz <10:
                    state = "free"

            if state == "stance":
                    print(f"impedance, F={loadcell.fz}")
                    knee.set_impedance()

            if state =="free":
                print(f"non_loop, F ={loadcell.fz}")                   
                    
           
            # Print loadcell data in-place (carriage return avoids scroll overhead)
            # if round(t/0.01)%10 == 0:
            #     print(f"\r t={t:6.2f}s | "
            #           f"F(xyz) N: [{loadcell.fx:7.2f}, {loadcell.fy:7.2f}, {loadcell.fz:7.2f}] | "
            #           f"M(xyz) Nm: [{loadcell.mx:7.3f}, {loadcell.my:7.3f}, {loadcell.mz:7.3f}] | "
            #           f"CAN rx: {loadcell._last_update_count}", end='   ')


    except KeyboardInterrupt:
        print("Moving Knee to Idle mode")
        knee.idle()  
        os.system(f"sudo ip link set {CAN_CH} down")
        
    finally:
        # This block ALWAYS runs — whether loop exits via:
        #   - LoopKiller signal (Ctrl+C → SoftRealtimeLoop swallows SIGINT)
        #   - loop.stop()
        #   - Any exception
        print("\n\nStopping loop and moving Knee to Idle mode")
        knee.idle()
        loadcell.stop()
        os.system(f"sudo ip link set {CAN_CH} down")