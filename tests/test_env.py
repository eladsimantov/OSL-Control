import os
import sys

if __name__ == "__main__":
    tests_path = os.path.dirname(os.path.abspath(__file__))
    project_path = os.path.join(tests_path, "..")
    # src_path = os.path.join(tests_path, "..", "src")
    sys.path.insert(0, project_path)
    # os.chdir(project_path)
    # print(os.getcwd())
    import opensourceleg as osl
    from opensourceleg.actuators import ActuatorBase, MOTOR_CONSTANTS, CONTROL_MODES
    from src.adapters.osl_lib_adapter import ODriveActuator
    from src.drivers.odrive_can import *
    dbc_path = "/home/enable-lab/OSL-Control/src/drivers/odrive-cansimple.dbc"
    db = cantools.database.load_file(dbc_path)
    # db.encode_message("", 0)
    print("Environment is set up.")
