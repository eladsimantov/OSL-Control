import os
import sys
import time


from src.adapters.loadcell import SRILoadCell_M8123B2

def read_force_z():
    z = 0 
    sensor = SRILoadCell_M8123B2(channel="can1",bitrate=100000)
    try:
        sensor.start()
        sensor.calibrate(n_samples=500)

        while True:
            sensor.update
            time.sleep(0.05)


    except KeyboardInterrupt:
        print("stopped")
    finally:
        sensor.stop()
    # This test should identify contact based on z axis measured load cell force from a threshold. 

    if __name__ == "__main__":
     read_force_z()