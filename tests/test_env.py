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
    from opensourceleg.actuators import ActuatorBase
    from src.drivers.odrive_can import *
    print("Environment is set up.")
