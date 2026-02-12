from tests.test_actuator import test_actuator
from tests.test_imu import test_imu
from tests.test_loadcell import test_loadcell

if __name__ == "__main__":
    print("\n" + "="*50)
    print("      ROBOT SENSOR/ACTUATOR TESTS ")
    print("="*50 + "\n")
    input("Press Enter to start the IMU test...")
    test_imu()
    
    input("Press Enter to start the Load Cell test...")
    test_loadcell()

    input("Press Enter to start the Actuator test...")
    test_actuator()

    print("\n" + "="*50)
    print("      ALL TESTS COMPLETED ")
    print("="*50 + "\n")