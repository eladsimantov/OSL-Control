import numpy as np
from opensourceleg.sensors.base import IMUBase
from opensourceleg.logging import LOGGER
from typing import Any

class BNO055Adapter(IMUBase): # Inherit from IMUBase directly to bypass BNO055's init crash
    """
    Local adapter for the Bosch BNO055 IMU.
    Wraps hardware logic to allow 'Offline Mode' on Windows/MacOS.

    To check the adapter address, run the command on your PI: 
        ```bash 
        i2cdetect -y 1
        ```
        If you see 29, change the "addr" to 0x29 instead of 0x28.
    """

    def __init__(
        self, 
        tag: str = "BNO055", 
        addr: int = 0x28, 
        offline: bool = False
    ) -> None:
        super().__init__(tag=tag, offline=offline)
        self._address = addr
        self._gyro_data = [0.0, 0.0, 0.0]
        self._acc_data = [0.0, 0.0, 0.0]
        self._euler_data = [0.0, 0.0, 0.0]
        self._quat_data = [1.0, 0.0, 0.0, 0.0]  # Initialize default identity quaternion
        self._is_streaming = False
        self._adafruit_imu = None

    def start(self) -> None:
        """
        Initializes hardware only if NOT in offline mode.
        This prevents 'import board' from crashing on Windows.
        """
        if self.is_offline:
            self._is_streaming = True
            LOGGER.info(f"[{self.tag}] Started in OFFLINE mode (Simulation).")
            return

        try:
            # Hardware-only imports happen here
            import adafruit_bno055
            import board
            import busio
            
            i2c = busio.I2C(board.SCL, board.SDA)
            self._adafruit_imu = adafruit_bno055.BNO055_I2C(i2c, address=self._address)
            
            # OSL Config: Use IMUPLUS_MODE (accelerometer + gyro fusion) to enable
            # onboard orientation calculation (Euler angles & Quaternions).
            # If absolute compass heading is needed, use NDOF_MODE instead.
            # Avoid NDOF_MODE if close to high-current motor wires to prevent magnetic distortion.
            # See in the adafruit library all modes and their descriptions in: adafruit_bno055.BNO055
            self._adafruit_imu.use_external_crystal = True
            self._adafruit_imu.mode = adafruit_bno055.IMUPLUS_MODE
            
            self._is_streaming = True
            LOGGER.info(f"[{self.tag}] Hardware connected and streaming.")
        except Exception as e:
            LOGGER.error(f"[{self.tag}] Hardware initialization failed: {e}")
            self._is_streaming = False

    def update(self) -> None:
        """Reads data from hardware if available."""
        if self.is_offline or not self.is_streaming or self._adafruit_imu is None:
            return
            
        self._acc_data = list(self._adafruit_imu.acceleration)
        self._gyro_data = list(self._adafruit_imu.gyro)
        
        # Read orientation data if available (using value comparison instead of identity is not)
        euler_val = self._adafruit_imu.euler
        if euler_val != (None, None, None):
            self._euler_data = list(euler_val)
            
        quat_val = self._adafruit_imu.quaternion
        if quat_val != (None, None, None, None):
            self._quat_data = list(quat_val)

    @property
    def data(self) -> dict[str, Any]:
        return {
            "acc": self._acc_data,
            "gyro": self._gyro_data,
            "euler": self._euler_data,
            "quat": self._quat_data
        }

    # API Properties for OSL compatibility
    @property
    def acc_x(self) -> float: return self._acc_data[0]
    @property
    def acc_y(self) -> float: return self._acc_data[1]
    @property
    def acc_z(self) -> float: return self._acc_data[2]
    @property
    def gyro_x(self) -> float: return self._gyro_data[0]
    @property
    def gyro_y(self) -> float: return self._gyro_data[1]
    @property
    def gyro_z(self) -> float: return self._gyro_data[2]
    @property
    def euler_x(self) -> float: return self._euler_data[0]
    @property
    def euler_y(self) -> float: return self._euler_data[1]
    @property
    def euler_z(self) -> float: return self._euler_data[2]
    @property
    def quat_w(self) -> float: return self._quat_data[0]
    @property
    def quat_x(self) -> float: return self._quat_data[1]
    @property
    def quat_y(self) -> float: return self._quat_data[2]
    @property
    def quat_z(self) -> float: return self._quat_data[3]
    
    @property
    def is_streaming(self) -> bool: return self._is_streaming

    def stop(self) -> None:
        self._is_streaming = False