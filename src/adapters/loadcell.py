import can
import struct
import numpy as np
import numpy.typing as npt
from opensourceleg.sensors.base import LoadcellBase
from opensourceleg.logging import LOGGER

class SRILoadCell_M8123B2(LoadcellBase):
    """
    Standardized OSL Adapter for the Sunrise Instruments (SRI) 6-Axis Loadcell.

    Hardware Overview:
    ------------------
    This class interfaces with an SRI 6-axis sensor connected to an M8123B2 OEM 
    circuit board. The board handles the complex analog-to-digital conversion 
    and applies a factory-embedded 6x6 decoupling matrix.

    Communication Details:
    ----------------------
    - Protocol: SocketCAN (CAN Bus).
    - Interface: Requires a CAN-FD or Standard CAN HAT (typically 'can1' on RPi).
    - Data Format: Receives three sequential 8-byte UDP packets containing 
      32-bit IEEE 754 floating-point numbers.
    - Sample IDs: ID 0x291 (FX, FY), ID 0x292 (FZ, MX), ID 0x293 (MY, MZ).

    Processing Logic:
    -----------------
    1. Internal Decoupling: It is assumed the board is pre-calibrated by SRI. 
       Forces (F) and Moments (M) arrive as final engineering units (N and Nm).
    2. Software Tare: Since the sensor often carries the weight of a foot or 
       attachment, the `calibrate()` method measures a static baseline (bias) 
       and subtracts it from all live readings to ensure a 'Zero' starting point.
    
    Author: Elad Siman Tov
    Date: 2026-01-01
    """

    def __init__(
        self,
        tag: str = "SRILoadcell",
        channel: str = "can1",
        bitrate: int = 1000000,
        id_query: int = 0x80,
        offline: bool = False,
    ) -> None:
        """
        Initialize the SRI loadcell adapter.
        
        Args:
            tag: Identifier for the sensor.
            channel: The CAN interface name on the RPi (e.g., 'can1').
            bitrate: Speed of the CAN bus (Default 1Mb/s for M8123B2).
            id_query: The command ID for the sensor (Default 0x80). 
            offline: If True, operates without hardware.
        """
        super().__init__(tag=tag, offline=offline)

        self._channel = channel
        self._bitrate = bitrate
        self._id_query = id_query
        
        # M8123B2 uses sequential IDs for the 6 axes 
        self._id_replies = [0x291, 0x292, 0x293] 
        
        self._data: npt.NDArray[np.double] = np.zeros(6)
        self._offset: npt.NDArray[np.double] = np.zeros(6)
        self._is_streaming: bool = False
        self._is_calibrated: bool = False
        self._bus = None

    def start(self) -> None:
        """Connects to the CAN bus and sends the continuous measurement trigger."""
        if self.is_offline:
            self._is_streaming = True
            return

        try:
            # Initialize SocketCAN bus
            self._bus = can.interface.Bus(channel=self._channel, bustype='socketcan')
            
            # Send '2' (0x02) to ID #1 to start continuous data 
            start_msg = can.Message(arbitration_id=self._id_query, data=[0x02], is_extended_id=False)
            self._bus.send(start_msg)
            
            self._is_streaming = True
            LOGGER.info(f"[{self.tag}] SRI M8123B2 stream started on {self._channel}")
        except Exception as e:
            LOGGER.error(f"[{self.tag}] Failed to initialize CAN: {e}")
            self._is_streaming = False

    def update(self) -> None:
        """Collects the three CAN packets and unpacks the 6-axis floats."""
        if self.is_offline or not self._is_streaming:
            return

        # Attempt to capture all three data frames sequentially 
        received_ids = set()
        while len(received_ids) < 3:
            msg = self._bus.recv(timeout=0.01)
            if msg is None:
                break
            
            # Packets are Little-Endian 4-byte floats 
            if msg.arbitration_id == 0x291:
                self._data[0], self._data[1] = struct.unpack('<ff', msg.data) # FX, FY
                received_ids.add(0x291)
            elif msg.arbitration_id == 0x292:
                self._data[2], self._data[3] = struct.unpack('<ff', msg.data) # FZ, MX
                received_ids.add(0x292)
            elif msg.arbitration_id == 0x293:
                self._data[4], self._data[5] = struct.unpack('<ff', msg.data) # MY, MZ
                received_ids.add(0x293)

    def stop(self) -> None:
        """Stops the sensor stream and closes the bus."""
        if self._bus:
            # Send '0' (0x00) to ID #1 to stop 
            stop_msg = can.Message(arbitration_id=self._id_query, data=[0x00], is_extended_id=False)
            self._bus.send(stop_msg)
            self._bus.shutdown()
        self._is_streaming = False

    def calibrate(self, n_samples: int = 2000) -> None:
        """Perform a software zeroing routine by averaging samples."""
        LOGGER.info(f"[{self.tag}] Starting zeroing routine. Ensure the sensor is unloaded.")
        input("Press Enter to start...")
        
        samples = []
        for _ in range(n_samples):
            self.update()
            samples.append(self._data.copy())
            
        self._offset = np.mean(samples, axis=0)
        self._is_calibrated = True
        LOGGER.info(f"[{self.tag}] Zeroing complete.")

    def reset(self) -> None:
        """Resets the local data and offsets."""
        self._data = np.zeros(6)
        self._offset = np.zeros(6)
        self._is_calibrated = False

    @property
    def data(self) -> list[float]:
        """Returns zeroed [Fx, Fy, Fz, Mx, My, Mz]."""
        return (self._data - self._offset).tolist()

    @property
    def is_streaming(self) -> bool: return self._is_streaming

    @property
    def is_calibrated(self) -> bool: return self._is_calibrated

    # Individual Axis Properties with offset subtraction
    @property
    def fx(self) -> float: return float(self._data[0] - self._offset[0])
    @property
    def fy(self) -> float: return float(self._data[1] - self._offset[1])
    @property
    def fz(self) -> float: return float(self._data[2] - self._offset[2])
    @property
    def mx(self) -> float: return float(self._data[3] - self._offset[3])
    @property
    def my(self) -> float: return float(self._data[4] - self._offset[4])
    @property
    def mz(self) -> float: return float(self._data[5] - self._offset[5])