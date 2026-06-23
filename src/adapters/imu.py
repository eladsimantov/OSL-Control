import numpy as np

# Witmotion Relevant 
import threading
import serial
import struct
import math
import time
import asyncio
import subprocess

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


class _BLEManager:
    _instance = None
    _lock = threading.Lock()
    
    @classmethod
    def get_instance(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        self._devices = {}  # mac_address -> WitMotionIMUAdapter instance
        self._tasks = {}    # mac_address -> asyncio.Task
        self._thread = None
        self._loop = None
        self._running = False
        self._lock = threading.Lock()

    def register_device(self, adapter):
        mac = adapter._mac_address
        if not mac:
            return
        
        with self._lock:
            self._devices[mac] = adapter
            
            # Start the background thread if not already running
            if not self._thread or not self._thread.is_alive():
                self._running = True
                self._thread = threading.Thread(target=self._run_event_loop, daemon=True)
                self._thread.start()
            else:
                # If loop is already running, schedule the new connection task on it
                if self._loop and self._loop.is_running():
                    def schedule_task():
                        task = self._loop.create_task(self._connect_and_stream(mac))
                        with self._lock:
                            self._tasks[mac] = task
                    self._loop.call_soon_threadsafe(schedule_task)

    def unregister_device(self, adapter):
        mac = adapter._mac_address
        with self._lock:
            if mac in self._devices:
                del self._devices[mac]
            
            # Cancel the task associated with this device
            task = self._tasks.pop(mac, None)
            if task and self._loop and self._loop.is_running():
                self._loop.call_soon_threadsafe(task.cancel)
                
            if not self._devices:
                self._running = False

    def _run_event_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        with self._lock:
            # Schedule connection for all currently registered devices
            for mac in list(self._devices.keys()):
                task = self._loop.create_task(self._connect_and_stream(mac))
                self._tasks[mac] = task
            
        try:
            self._loop.run_until_complete(self._keep_alive())
            
            # Now self._running is False. Wait for tasks to exit normally to disconnect clients cleanly.
            with self._lock:
                active_tasks = list(self._tasks.values())
            
            if active_tasks:
                LOGGER.info(f"[BLEManager] Waiting for {len(active_tasks)} BLE task(s) to exit normally...")
                done, pending = self._loop.run_until_complete(
                    asyncio.wait(active_tasks, timeout=3.0)
                )
                
                if pending:
                    LOGGER.warning(f"[BLEManager] {len(pending)} BLE task(s) did not exit in time. Cancelling them...")
                    for task in pending:
                        task.cancel()
                    self._loop.run_until_complete(
                        asyncio.wait(pending, timeout=2.0)
                    )
        except Exception as e:
            LOGGER.error(f"[BLEManager] Event loop exception: {e}")
        finally:
            with self._lock:
                self._tasks.clear()
            self._loop.close()
            self._loop = None

    async def _keep_alive(self):
        while self._running:
            await asyncio.sleep(0.1)

    async def _connect_and_stream(self, mac):
        from bleak import BleakClient
        
        DATA_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"
        
        def notification_handler(sender, data):
            # Fetch under lock to avoid race conditions
            with self._lock:
                adapter = self._devices.get(mac)
            if adapter:
                adapter._parse_ble_data(data)

        try:
            while self._running:
                with self._lock:
                    if mac not in self._devices:
                        break
                    adapter = self._devices[mac]
                    
                try:
                    LOGGER.info(f"[BLEManager] BLE Connecting to {mac}...")
                    async with BleakClient(mac) as client:
                        if client.is_connected:
                            LOGGER.info(f"[BLEManager] BLE Connected to {mac}.")
                            with adapter._lock:
                                adapter._is_streaming = True
                            await client.start_notify(DATA_UUID, notification_handler)
                            while self._running and client.is_connected:
                                with self._lock:
                                    if mac not in self._devices:
                                        break
                                await asyncio.sleep(0.1)
                            if client.is_connected:
                                await client.stop_notify(DATA_UUID)
                    LOGGER.warning(f"[BLEManager] BLE Connection lost/closed for {mac}.")
                except Exception as e:
                    LOGGER.error(f"[BLEManager] BLE Connection error for {mac}: {e}. Retrying in 4 seconds...")
                    # Sleep in small steps to remain responsive to shutdown
                    for _ in range(40):
                        if not self._running:
                            break
                        await asyncio.sleep(0.1)
        finally:
            LOGGER.info(f"[BLEManager] BLE cleaning up stream for {mac}...")
            with self._lock:
                adapter = self._devices.get(mac)
                self._tasks.pop(mac, None)
            if adapter:
                with adapter._lock:
                    adapter._is_streaming = False


class WitMotionIMUAdapter(IMUBase):
    """
    OSL Adapter for the WitMotion Bluetooth IMU (e.g. BWT901CL / BWT61CL).
    
    Threading and Loop Execution Model:
    -----------------------------------
    1. Online Mode (Hardware Connected):
       - A background thread (`_read_loop` or BLE notifier) runs continuously to read
         bytes (or receive BLE notifications), verify checksums/headers, parse incoming
         packets, and update the internal states.
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
        connection_type: str = "serial",
        offline: bool = False
    ) -> None:
        super().__init__(tag=tag, offline=offline)
        self._mac_address = mac_address
        self._port = port
        self._baudrate = baudrate
        self._connection_type = connection_type.lower()
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
        self._offset = [0.0, 0.0, 0.0]
        self._is_calibrated = False

    def start(self) -> None:
        """Connects to the Bluetooth device (directly via BLE, socket, or serial port) and starts the background listener."""
        if self.is_offline:
            self._is_streaming = True
            LOGGER.info(f"[{self.tag}] Started in OFFLINE mode (Simulation).")
            return

        try:
            self._running = True
            
            if self._connection_type == "ble":
                if not self._mac_address:
                    raise ValueError(f"[{self.tag}] MAC address is required for BLE connection.")
                
                # Register with the shared BLE manager
                _BLEManager.get_instance().register_device(self)
                
                # Wait for connection to establish (up to 5 seconds) so that is_streaming becomes True
                start_wait = time.time()
                while not self.is_streaming and (time.time() - start_wait) < 5.0:
                    time.sleep(0.1)
                
            else: # Serial mode
                if self._mac_address and self._port and "rfcomm" in self._port:
                    # Extract the RFCOMM device number, e.g., "/dev/rfcomm0" -> "0"
                    rfcomm_num = "".join(filter(str.isdigit, self._port))
                    if rfcomm_num:
                        LOGGER.info(f"[{self.tag}] Ensuring RFCOMM binding for {self._port} to {self._mac_address}...")
                        # Release first
                        subprocess.run(["sudo", "rfcomm", "release", rfcomm_num], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        # Bind
                        res = subprocess.run(["sudo", "rfcomm", "bind", rfcomm_num, self._mac_address], capture_output=True, text=True)
                        if res.returncode != 0:
                            LOGGER.warning(f"[{self.tag}] rfcomm bind command output: {res.stderr.strip()}")
                
                # Try to open serial port (or RFCOMM socket if mac_address is provided but no port is specified/not rfcomm)
                if self._mac_address and (not self._port or "rfcomm" not in self._port):
                    import socket
                    if not hasattr(socket, 'AF_BLUETOOTH'):
                        raise NotImplementedError(
                            f"Direct RFCOMM socket connection is not supported on this OS. "
                            f"Please pair the device at OS-level and use serial `port` parameter instead."
                        )
                    
                    LOGGER.info(f"[{self.tag}] Connecting directly via RFCOMM socket to {self._mac_address}...")
                    self._socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                    self._socket.settimeout(5.0)
                    self._socket.connect((self._mac_address, 1))
                    self._socket.settimeout(0.1)
                    self._using_socket = True
                    LOGGER.info(f"[{self.tag}] RFCOMM socket connection established.")
                else:
                    LOGGER.info(f"[{self.tag}] Opening Serial port on {self._port}...")
                    self._serial = serial.Serial(self._port, baudrate=self._baudrate, timeout=0.1)
                    self._using_socket = False
                    LOGGER.info(f"[{self.tag}] Serial port opened on {self._port}")
                
                # Start background reading thread for Serial/Socket
                self._thread = threading.Thread(target=self._read_loop, daemon=True)
                self._thread.start()
                self._is_streaming = True
                
        except Exception as e:
            LOGGER.error(f"[{self.tag}] Failed to start connection: {e}")
            self._running = False
            if self._socket:
                try:
                    self._socket.close()
                except Exception:
                    pass
                self._socket = None
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._is_streaming = False

    def _parse_ble_data(self, data: bytes) -> None:
        """Parse raw incoming bytes from BLE notification channel."""
        idx = 0
        while idx < len(data):
            # Check for 20-byte high-speed combined broadcast frame: 0x55 0x61
            if idx + 20 <= len(data) and data[idx] == 0x55 and data[idx+1] == 0x61:
                packet = data[idx:idx+20]
                try:
                    ax, ay, az, gx, gy, gz, roll_raw, pitch_raw, yaw_raw = struct.unpack('<hhhhhhhhh', packet[2:20])
                    
                    # Convert raw values to standard OSL units (m/s^2, rad/s, degrees)
                    ax_val = ax / 32768.0 * 16.0 * 9.80665
                    ay_val = ay / 32768.0 * 16.0 * 9.80665
                    az_val = az / 32768.0 * 16.0 * 9.80665
                    
                    gx_val = gx / 32768.0 * 2000.0 * (math.pi / 180.0)
                    gy_val = gy / 32768.0 * 2000.0 * (math.pi / 180.0)
                    gz_val = gz / 32768.0 * 2000.0 * (math.pi / 180.0)
                    
                    roll = roll_raw / 32768.0 * 180.0
                    pitch = pitch_raw / 32768.0 * 180.0
                    yaw = yaw_raw / 32768.0 * 180.0
                    
                    with self._lock:
                        self._acc_data = [ax_val, ay_val, az_val]
                        self._gyro_data = [gx_val, gy_val, gz_val]
                        self._euler_data = [roll, pitch, yaw] # (roll, pitch, yaw)
                except Exception as e:
                    LOGGER.warning(f"[{self.tag}] BLE parse 0x55 0x61 frame error: {e}")
                idx += 20
            # Check for standard 11-byte serial-style packet: starts with 0x55
            elif idx + 11 <= len(data) and data[idx] == 0x55:
                packet = data[idx:idx+11]
                checksum = sum(packet[0:10]) & 0xFF
                if checksum == packet[10]:
                    try:
                        self._parse_packet(packet)
                    except Exception as e:
                        LOGGER.warning(f"[{self.tag}] BLE parse 11-byte frame error: {e}")
                    idx += 11
                else:
                    idx += 1
            else:
                idx += 1

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

                # Process buffer
                while len(buffer) >= 11:
                    if buffer[0] == 0x55:
                        # Check for 20-byte high-speed combined broadcast frame: 0x55 0x61
                        if len(buffer) >= 20 and buffer[1] == 0x61:
                            packet = buffer[:20]
                            try:
                                ax, ay, az, gx, gy, gz, roll_raw, pitch_raw, yaw_raw = struct.unpack('<hhhhhhhhh', packet[2:20])
                                
                                ax_val = ax / 32768.0 * 16.0 * 9.80665
                                ay_val = ay / 32768.0 * 16.0 * 9.80665
                                az_val = az / 32768.0 * 16.0 * 9.80665
                                
                                gx_val = gx / 32768.0 * 2000.0 * (math.pi / 180.0)
                                gy_val = gy / 32768.0 * 2000.0 * (math.pi / 180.0)
                                gz_val = gz / 32768.0 * 2000.0 * (math.pi / 180.0)
                                
                                roll = roll_raw / 32768.0 * 180.0
                                pitch = pitch_raw / 32768.0 * 180.0
                                yaw = yaw_raw / 32768.0 * 180.0
                                
                                with self._lock:
                                    self._acc_data = [ax_val, ay_val, az_val]
                                    self._gyro_data = [gx_val, gy_val, gz_val]
                                    self._euler_data = [roll, pitch, yaw]
                            except Exception as e:
                                LOGGER.warning(f"[{self.tag}] Serial parse 0x55 0x61 error: {e}")
                            del buffer[:20]
                        else:
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
                ax = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 16.0 * 9.80665
                ay = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 16.0 * 9.80665
                az = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 16.0 * 9.80665
                self._acc_data = [ax, ay, az]
                
            elif flag == 0x52:  # Angular Velocity packet
                gx = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 2000.0 * (math.pi / 180.0)
                gy = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 2000.0 * (math.pi / 180.0)
                gz = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 2000.0 * (math.pi / 180.0)
                self._gyro_data = [gx, gy, gz]
                
                roll = struct.unpack('<h', packet[2:4])[0] / 32768.0 * 180.0
                pitch = struct.unpack('<h', packet[4:6])[0] / 32768.0 * 180.0
                yaw = struct.unpack('<h', packet[6:8])[0] / 32768.0 * 180.0
                self._euler_data = [roll, pitch, yaw]  # Match standard roll, pitch, yaw order
                
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
            t = time.time() if hasattr(time, 'time') else 0.0
            self._euler_data = [10.0 * math.sin(t), 5.0 * math.cos(t), 20.0 * math.sin(0.5 * t)]
            
    def stop(self) -> None:
        """Safely stops the thread and closes connections."""
        self._running = False
        
        if self._connection_type == "ble":
            _BLEManager.get_instance().unregister_device(self)
        else:
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

            # Clean up RFCOMM binding if we created one
            if not self.is_offline and self._connection_type == "serial" and self._mac_address and self._port and "rfcomm" in self._port:
                rfcomm_num = "".join(filter(str.isdigit, self._port))
                if rfcomm_num:
                    LOGGER.info(f"[{self.tag}] Releasing RFCOMM binding for {self._port}...")
                    subprocess.run(["sudo", "rfcomm", "release", rfcomm_num], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
        self._is_streaming = False

    def calibrate(self, n_samples: int = 500) -> None:
        """Perform a software zeroing routine by averaging samples."""
        LOGGER.info(f"[{self.tag}] Starting zeroing routine. Ensure the sensor is stationary.")
        
        samples = []
        for _ in range(n_samples):
            if self.is_offline:
                self.update()
            
            with self._lock:
                samples.append(list(self._euler_data))
            
            time.sleep(0.005)
            
        with self._lock:
            self._offset = np.mean(samples, axis=0).tolist()
            self._is_calibrated = True
        LOGGER.info(f"[{self.tag}] Zeroing complete. Offsets: {self._offset}")

    def reset(self) -> None:
        """Resets the offsets."""
        with self._lock:
            self._offset = [0.0, 0.0, 0.0]
            self._is_calibrated = False

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    @property
    def data(self) -> dict[str, Any]:
        with self._lock:
            return {
                "acc": list(self._acc_data),
                "gyro": list(self._gyro_data),
                "euler": [
                    self._euler_data[0] - self._offset[0],
                    self._euler_data[1] - self._offset[1],
                    self._euler_data[2] - self._offset[2]
                ],
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
        with self._lock: return self._euler_data[0] - self._offset[0]
    @property
    def euler_y(self) -> float:
        with self._lock: return self._euler_data[1] - self._offset[1]
    @property
    def euler_z(self) -> float:
        with self._lock: return self._euler_data[2] - self._offset[2]
        
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