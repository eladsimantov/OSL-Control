"""
This module contains the implementation of the sensor/actuator classes for our application using OSL architecture.
It is called "adapters" because it serves as an adapter layer between our hardware and the OSL API.
The classes inherit from the sensor/actuator classes defined in opensourceleg modules.
- IMU: use the BNO055 IMU sensor, which is already implemented by the great Dr. Kevin Best.
- Loadcell: use the base loadcell class, and implement for SRI load cell.
- Encoder: use the AS5048B encoder, which is already implemented by Senthur Ayyappan.
- Actuator: use the base actuator class, and implement our class using ODrive and our custom CAN interface.
"""