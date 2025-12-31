import time
import threading
import can
import cantools
import os

# CAN Initialization (Linux/Raspberry Pi)
# os.system("sudo ip link set can0 up type can bitrate 250000")

# ===========================================================
# CLASS: ODriveCAN
# - background listener decodes incoming CAN frames and stores
#   the latest decoded signals per (axis, message_name).
# ===========================================================
class ODriveCAN:
    def __init__(self, bus_name="can0", node_id=1, dbc_path=os.path.join(os.getcwd(), "src/drives/odrive-cansimple.dbc")):
        self.node_id = node_id
        self.axisID = node_id
        # open CAN and DBC
        self.bus = can.Bus(bus_name, bustype="socketcan")
        try:
            self.db = cantools.database.load_file(dbc_path)
        except Exception:
            self.db = None
        self.latest = {}  # {(axis, message_name): signals_dict}
        self._listener_thread = threading.Thread(target=self._listener, daemon=True)
        self._listener_thread.start()

    def send(self, arb_id, data):
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        msg = can.Message(arbitration_id=arb_id, is_extended_id=False, data=data)
        try:
            self.bus.send(msg)
        except Exception:
            pass

    def send_dbc(self, message_name, signals):
        if not self.db:
            raise RuntimeError("DBC not loaded")
        data = self.db.encode_message(message_name, signals)
        message = self.db.get_message_by_name(message_name)
        arb_id = (self.axisID << 5) | message.frame_id
        self.send(arb_id, data)

    def _listener(self):
        while True:
            try:
                msg = self.bus.recv(timeout=1.0)
                if not msg:
                    continue
                frame_id = msg.arbitration_id & 0x1F
                axis = msg.arbitration_id >> 5
                if not self.db:
                    self.latest[(axis, f"RAW_{frame_id:#x}")] = {"raw": msg.data}
                    continue
                try:
                    message = self.db.get_message_by_frame_id(frame_id)
                except Exception:
                    self.latest[(axis, f"RAW_{frame_id:#x}")] = {"raw": msg.data}
                    continue
                try:
                    signals = message.decode(msg.data)
                except Exception:
                    self.latest[(axis, message.name)] = {"raw": msg.data}
                    continue
                self.latest[(axis, message.name)] = signals
            except Exception:
                time.sleep(0.1)

    def get_latest_for_axis(self, axis):
        out = {}
        for (a, name), signals in self.latest.items():
            if a == axis:
                out[name] = signals
        return out

    def find_signal(self, axis, candidates):
        msgs = self.get_latest_for_axis(axis)
        for mname, signals in msgs.items():
            for sname, val in signals.items():
                for cand in candidates:
                    if sname.lower() == cand.lower():
                        return (mname, sname, val)
        return (None, None, None)

    def dump_axis(self, axis):
        msgs = self.get_latest_for_axis(axis)
        if not msgs:
            return f"axis {axis}: no decoded messages"
        out = [f"axis {axis}:"]
        for mname, signals in msgs.items():
            out.append(f"  {mname}:")
            for sname, val in signals.items():
                out.append(f"    {sname} = {val}")
        return "\n".join(out)




class ODriveMotor:
    def __init__(self, can_interface: ODriveCAN, gear_ratio=20, name="motor"):
        self.can = can_interface
        self.gear_ratio = gear_ratio
        self.name = name
        self.alive = True
        self._home_deg = 0.0

    def set_state(self, state_id):
        # 8 = Closed Loop, 1 = Idle
        self.can.send_dbc("Axis0_Set_Axis_State", {"Axis_Requested_State": state_id})

    def get_position_deg(self):
        """Returns position in degrees relative to home."""
        candidates = ["pos_estimate", "pos", "encoder_pos", "encoder_count", "position"]
        mname, sname, val = self.can.find_signal(self.can.axisID, candidates)
        
        if val is None: return 0.0
        
        try:
            v = float(val)
            # Logic from your original code regarding counts vs turns
            if abs(v) > 1000: turns = v / 4096.0
            else: turns = v
            
            deg = (turns * 360.0 / self.gear_ratio) - self._home_deg
            return deg
        except:
            return 0.0

    def set_position_deg(self, deg):
        """Send Position Command."""
        target_deg = deg + self._home_deg
        target_turns = (target_deg / 360.0) * self.gear_ratio
        
        self.can.send_dbc("Axis0_Set_Input_Pos", {
            "Input_Pos": float(target_turns),
            "Vel_FF": 0.0,
            "Torque_FF": 0.0
        })

    def set_torque(self, torque_nm):
        """Send Torque Command (Added for OSL Compatibility)."""
        # Note: ODrive typically expects torque at the motor shaft.
        # If your gear ratio is 20, 1Nm at output = 0.05Nm at motor.
        motor_torque = torque_nm / self.gear_ratio 
        
        self.can.send_dbc("Axis0_Set_Input_Torque", {
            "Input_Torque": float(motor_torque)
        })