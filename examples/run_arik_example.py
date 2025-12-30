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
import threading
import math
import can
import cantools

from src.drivers.odrive_can import ODriveMotor
from src.drivers.odrive_can import ODriveCAN

#probably we need the second class also, maybe even the third 

#we want to move to one command for initiation, not 2 or more 


def main():
    #initiation 
    can1 = ODriveCAN(node_id=1)
    knee = ODriveMotor(can1, name="knee")
    can2 = ODriveCAN(node_id=2)
    ankle = ODriveMotor(can2, name="ankle")
    #calibration 
    
    #i dont understand the calibration process arik wrote 
    #set state 3 - calibrate the motor (internal things, not the home angle)
    #home deg - what we use to tell the position we want. i need to check if it is absolute or relative to the turning on 
    #according to gemini the endcoder is encramental- so we define the home deg, maybe easier to change the code so the 0 changes 
    # why fo we need idle in the calibration?



    #oriention 
    degKnee = 30
    #need to check for float or integar 
    knee.position_deg(degKnee)
    degAnkle = 10 
    ankle.position_deg(degAnkle)
    delay = 10 
    time.sleep(10)
    degKnee = 0
    knee.position_deg(degKnee)
    degAnkle = 0
    ankle.position_deg(degAnkle)

    return


if __name__ == "__main__":
    main()

