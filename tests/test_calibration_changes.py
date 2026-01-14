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

