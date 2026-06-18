import numpy as np

# Witmotion Relevant 
import threading
import serial
import struct
import math
import time

# OSL Imports
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


class WitMotionIMUAdapter(IMUBase):
    """
    OSL Adapter for the WitMotion Bluetooth IMU (e.g. BWT901CL / BWT61CL).
    
    Threading and Loop Execution Model:
    -----------------------------------
    1. Online Mode (Hardware Connected):
       - A background thread (`_read_loop`) runs continuously to read serial bytes,
         verify 11-byte checksums, parse incoming packets, and update the internal states.
       - The properties (`euler_x`, `quat_w`, etc.) return these updated states instantly.
       - You do NOT need to call `update()` to get new hardware readings, as they update in the background.
    
    2. Offline Mode (Simulation):
       - No background thread runs. Calling `update()` in your control loop generates
         sinusoidal simulated values for testing logic.
         
    Recommendation: Always call `update()` in your real-time loop so that your code
    behaves identically in both online (production) and offline (simulation) modes.
    """

    def __init__(
        self,
        tag: str = "WitMotion",
        mac_address: str = None,
        port: str = "/dev/rfcomm0",
        baudrate: int = 115200,
        offline: bool = False
    ) -> None:
        super().__init__(tag=tag, offline=offline)
        self._mac_address = mac_address
        self._port = port
        self._baudrate = baudrate
        self._gyro_data = [0.0, 0.0, 0.0]
        self._acc_data = [0.0, 0.0, 0.0]
        self._euler_data = [0.0, 0.0, 0.0]
        self._quat_data = [1.0, 0.0, 0.0, 0.0]
        
        self._is_streaming = False
        self._serial = None
        self._socket = None
        self._using_socket = False
        self._thread = None
        self._running = False
        self._lock = threading.Lock()

    def start(self) -> None:
        """Connects to the Bluetooth device (directly via socket or serial port) and starts the background listener."""
        if self.is_offline:
            self._is_streaming = True
            LOGGER.info(f"[{self.tag}] Started in OFFLINE mode (Simulation).")
            return

        try:
            if self._mac_address:
                import socket
                if not hasattr(socket, 'AF_BLUETOOTH'):
                    raise NotImplementedError(
                        f"Direct Bluetooth connection is not supported on this OS. "
                        f"Please pair the device at OS-level and use serial `port` parameter instead."
                    )
                
                LOGGER.info(f"[{self.tag}] Connecting directly via Bluetooth RFCOMM socket to {self._mac_address}...")
                self._socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self._socket.settimeout(5.0)
                # Channel 1 is standard for WitMotion SPP Bluetooth connection
                self._socket.connect((self._mac_address, 1))
                self._socket.settimeout(0.1)
                self._using_socket = True
                LOGGER.info(f"[{self.tag}] Bluetooth socket connection established.")
            else:
                # Open RFCOMM serial port
                self._serial = serial.Serial(self._port, baudrate=self._baudrate, timeout=0.1)
                self._using_socket = False
                LOGGER.info(f"[{self.tag}] Bluetooth Serial opened on {self._port}")
            
            self._running = True
            # Start background reading thread
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            
            self._is_streaming = True
        except Exception as e:
            LOGGER.error(f"[{self.tag}] Failed to open Bluetooth connection: {e}")
            if self._socket:
                try:
                    self._socket.close()
                except Exception:
                    pass
                self._socket = None
            self._is_streaming = False

    def _read_loop(self) -> None:
        """Background thread reading and parsing WitMotion packets continuously."""
        buffer = bytearray()
        import socket
        while self._running:
            try:
                if self._using_socket:
                    try:
                        data = self._socket.recv(1024)
                        if not data:
                            LOGGER.warning(f"[{self.tag}] Bluetooth socket connection closed by peer.")
                            break
                        buffer.extend(data)
                    except socket.timeout:
                        continue
                else:
                    if self._serial.in_waiting > 0:
                        data = self._serial.read(self._serial.in_waiting)
                        buffer.extend(data)
                    else:
                        # Small sleep to prevent 100% CPU thread thrashing
                        time.sleep(0.001)
                        continue

                # WitMotion standard packets are exactly 11 bytes
                while len(buffer) >= 11:
                    if buffer[0] == 0x55:
                        packet = buffer[:11]
                        
                        # Verify WitMotion checksum: sum of first 10 bytes masked with 0xFF
                        checksum = sum(packet[0:10]) & 0xFF
                        if checksum == packet[10]:
                            self._parse_packet(packet)
                            del buffer[:11]
                        else:
                            # Checksum failed: shift by 1 byte to find next header alignment
                            buffer.pop(0)
                    else:
                        buffer.pop(0)
            except Exception as e:
                LOGGER.warning(f"[{self.tag}] Error in Bluetooth read loop: {e}")
                time.sleep(0.1)

    def _parse_packet(self, packet: bytearray) -> None:
        """Parse standard 11-byte packet based on flag byte."""
        flag = packet[1]
        
        with self._lock:
            if flag == 0x51:  # Acceleration packet
                # Raw acceleration range is typically +/- 16g
                ax = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 16.0 * 9.80665
                ay = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 16.0 * 9.80665
                az = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 16.0 * 9.80665
                self._acc_data = [ax, ay, az]
                
            elif flag == 0x52:  # Angular Velocity packet
                # Raw gyro range is typically +/- 2000 deg/s. Convert to rad/s for OSL
                gx = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 2000.0 * (math.pi / 180.0)
                gy = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 2000.0 * (math.pi / 180.0)
                gz = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 2000.0 * (math.pi / 180.0)
                self._gyro_data = [gx, gy, gz]
                
            elif flag == 0x53:  # Euler Angles packet (Roll, Pitch, Yaw)
                roll = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 180.0
                pitch = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 180.0
                yaw = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 180.0
                self._euler_data = [yaw, pitch, roll]  # Match OSL property convention (yaw, pitch, roll)
                
            elif flag == 0x59:  # Quaternion packet (q0, q1, q2, q3)
                q0 = struct.unpack('<h', packet[2:4])[0] / 32768.0  # w
                q1 = struct.unpack('<h', packet[4:6])[0] / 32768.0  # x
                q2 = struct.unpack('<h', packet[6:8])[0] / 32768.0  # y
                q3 = struct.unpack('<h', packet[8:10])[0] / 32768.0 # z
                self._quat_data = [q0, q1, q2, q3]

    def update(self) -> None:
        """Polls sensor state.
        
        - In Online Mode: This method is a no-op (returns immediately) because updates 
          are handled asynchronously in the background reading thread.
        - In Offline Mode: Generates fake simulated sinusoidal wave data based on time.
        """
        if self.is_offline:
            # Simulated fake data in offline mode
            print("FAKE DATA!!!!")
            t = time.time() if hasattr(time, 'time') else 0.0
            self._euler_data = [10.0 * math.sin(t), 5.0 * math.cos(t), 20.0 * math.sin(0.5 * t)]
            
    def stop(self) -> None:
        """Safely stops the thread and closes connections."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
        self._is_streaming = False

    @property
    def data(self) -> dict[str, Any]:
        with self._lock:
            return {
                "acc": list(self._acc_data),
                "gyro": list(self._gyro_data),
                "euler": list(self._euler_data),
                "quat": list(self._quat_data)
            }

    # API Properties for OSL compatibility
    @property
    def acc_x(self) -> float:
        with self._lock: return self._acc_data[0]
    @property
    def acc_y(self) -> float:
        with self._lock: return self._acc_data[1]
    @property
    def acc_z(self) -> float:
        with self._lock: return self._acc_data[2]
        
    @property
    def gyro_x(self) -> float:
        with self._lock: return self._gyro_data[0]
    @property
    def gyro_y(self) -> float:
        with self._lock: return self._gyro_data[1]
    @property
    def gyro_z(self) -> float:
        with self._lock: return self._gyro_data[2]
        
    @property
    def euler_x(self) -> float:
        with self._lock: return self._euler_data[0]
    @property
    def euler_y(self) -> float:
        with self._lock: return self._euler_data[1]
    @property
    def euler_z(self) -> float:
        with self._lock: return self._euler_data[2]
        
    @property
    def quat_w(self) -> float:
        with self._lock: return self._quat_data[0]
    @property
    def quat_x(self) -> float:
        with self._lock: return self._quat_data[1]
    @property
    def quat_y(self) -> float:
        with self._lock: return self._quat_data[2]
    @property
    def quat_z(self) -> float:
        with self._lock: return self._quat_data[3]
        
    @property
    def is_streaming(self) -> bool: return self._is_streaming