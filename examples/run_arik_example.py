"""
This example will activate the knee and ankle joints by position 
commands using the our custom drivers library. 
It will not use the CLI, only a 1 minute example of 
activation, calibration, and positions.
"""

#it seems that we can break the leg easely if we control torque or velocity,
#the code doesnt care about the limits of the leg 
#do i need to import all the libraries? 
import time
import os
import sys
import threading
import math
import can
import cantools

project_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, project_path)
from src.drivers.odrive_can import ODriveMotor
from src.drivers.odrive_can import ODriveCAN

os.system("sudo ip link set can0 up type can bitrate 250000")

#probably we need the second class also, maybe even the third 
#we want to move to one command for initiation, not 2 or more 


def main():
    #initiation 
    can1 = ODriveCAN(node_id=1)
    knee = ODriveMotor(can1, name="knee")
    can2 = ODriveCAN(node_id=2)
    ankle = ODriveMotor(can2, name="ankle")
    
    # # calibration - make new funciton to reset_position to zero 
    # ankle.calibrate(5) # for 5 seconds..
    # knee.calibrate(5)

    # Statte 3 - closed loop control
    ankle.closed_loop()
    knee.closed_loop()
    time.sleep(3)

    #i dont understand the calibration process arik wrote 
    #set state 3 - calibrate the motor (internal things, not the home angle)
    #home deg - what we use to tell the position we want. i need to check if it is absolute or relative to the turning on 
    #according to gemini the endcoder is encramental- so we define the home deg, maybe easier to change the code so the 0 changes 
    # why fo we need idle in the calibration?

    #oriention 
    degKnee = 10
    #need to check for float or integar 
    knee.position_deg(degKnee)
    degAnkle = 10 
    ankle.position_deg(degAnkle)
    delay = 3 # seconds
    time.sleep(delay)
    degKnee = 0
    knee.position_deg(degKnee)
    degAnkle = 0
    ankle.position_deg(degAnkle)

#finishing the example
    ankle.idle()
    knee.idle()
    return


if __name__ == "__main__":
    main()

