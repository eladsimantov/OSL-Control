import numpy as np
from opensourceleg.sensors.base import IMUBase
from opensourceleg.logging import LOGGER
from typing import Any

class BNO055Adapter(IMUBase): # Inherit from IMUBase directly to bypass BNO055's init crash
    """
    Local adapter for the Bosch BNO055 IMU.
    Wraps hardware logic to allow 'Offline Mode' on Windows/MacOS.
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
            
            # Standard OSL Config
            self._adafruit_imu.use_external_crystal = True
            self._adafruit_imu.mode = adafruit_bno055.ACCGYRO_MODE
            
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

    @property
    def data(self) -> dict[str, Any]:
        return {
            "acc": self._acc_data,
            "gyro": self._gyro_data
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
    def is_streaming(self) -> bool: return self._is_streaming

    def stop(self) -> None:
        self._is_streaming = False