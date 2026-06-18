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
    def __init__(self, bus_name="can0", node_id=1, dbc_path="/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"):
        self.node_id = node_id
        self.axisID = node_id
        # open CAN and DBC
        try:
            self.bus = can.Bus(bus_name, bustype="socketcan")
        except Exception as e:
            try:
                self.bus = can.Bus(bus_name, bustype="virtual")
                print(f"ODriveCAN: socketcan failed ({e}). Fell back to virtual CAN bus.")
            except Exception as ve:
                self.bus = None
                print(f"ODriveCAN: failed to open CAN bus (socketcan & virtual failed): {ve}")
        try:
            self.db = cantools.database.load_file(dbc_path)
        except Exception:
            self.db = None
        self.latest = {}  # {(axis, message_name): signals_dict}
        self._listener_thread = threading.Thread(target=self._listener, daemon=True)
        self._listener_thread.start()

    def send(self, arb_id, data):
        if not self.bus:
            return
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
                if not self.bus:
                    time.sleep(0.5)
                    continue
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


# ===========================================================
# CLASS: ODriveMotor (Single Motor)
# ===========================================================
class ODriveMotor:
    def __init__(self, can_interface: ODriveCAN, gear_ratio=40, name="motor"):
        self.can = can_interface
        self.gear_ratio = gear_ratio
        self.name = name
        self.alive = True
        self.home_deg = 0.0
        self.degrees = 0.0
        self.Vel_FF = 0.0
        self.Torque_FF = 0.0
        self.mode = 'idle'

    def set_state(self, state_id):
        try:
            self.can.send_dbc("Axis0_Set_Axis_State", {"Axis_Requested_State": state_id})
        except Exception:
            self.alive = False

    def closed_loop(self):
        #print(f"→ {self.name}: CLOSED_LOOP_CONTROL")
        self.set_state(8)

    def idle(self):
        #print(f"→ {self.name}: IDLE")
        self.mode = 'idle'
        self.set_state(1)

    def control_mode(self, mode_id, input_id):
        try:
            self.can.send_dbc("Axis0_Set_Controller_Mode", {
                "Input_Mode": input_id,
                "Control_Mode": mode_id})
            
            #input mode means we can use ramp or trajectory, all the options are in the dbc 
            #control mode is the mode itself. 
            #input mode is neccesary for the control mode to work.
            #input mode 1 is without filtering 

        except Exception:
            self.alive = False

    def torque_control(self):   
        print(f"→ {self.name}: TORQUE_CONTROL")
        self.mode = 'torque'
        self.control_mode(1, 1)

    def velocity_control(self):
        print(f"→ {self.name}: VELOCITY_CONTROL")
        self.mode = 'velocity'
        self.control_mode(2, 1)
        
    def position_control(self):  
        print(f"→ {self.name}: POSITION_CONTROL")
        self.mode = 'position'
        self.control_mode(3, 1)

    def get_turns(self):
        candidates = ["pos_estimate", "pos", "encoder_pos", "encoder_count", "position"]
        mname, sname, val = self.can.find_signal(self.can.axisID, candidates)
        if val is None:
            return None, None, None
        try:
            v = float(val)
        except Exception:
            return mname, sname, None
        if abs(v) > 1000:
            turns = v / 4096.0
            return mname, sname, turns
        else:
            return mname, sname, v

    def get_state(self):
        """
        Reads position (deg), velocity (deg/s), and current (A) in a single unified state.
        This ensures synchronized and consistent state readings.
        """
        # --- 1. Position ---
        pos_deg = None
        msg_name = f"Axis{self.can.axisID}_Get_Encoder_Estimates"
        signals = self.can.latest.get((self.can.axisID, msg_name))
        if signals and "Pos_Estimate" in signals:
            try:
                turns = float(signals["Pos_Estimate"])
                pos_deg = 360.0 * (turns / self.gear_ratio)
                self.degrees = pos_deg
            except (ValueError, TypeError):
                pass

        if pos_deg is None:
            mname, sname, turns = self.get_turns()
            if turns is not None:
                pos_deg = 360.0 * (turns / self.gear_ratio)
                self.degrees = pos_deg
            else:
                pos_deg = self.degrees

        # --- 2. Velocity ---
        vel_dps = None
        if signals and "Vel_Estimate" in signals:
            try:
                turns_s = float(signals["Vel_Estimate"])
                vel_dps = (turns_s / self.gear_ratio) * 360.0
            except (ValueError, TypeError):
                pass

        if vel_dps is None:
            candidates = ["vel_estimate", "velocity", "encoder_vel", "velocity_estimate"]
            mname, sname, val = self.can.find_signal(self.can.axisID, candidates)
            if val is not None:
                try:
                    vel_dps = (float(val) / self.gear_ratio) * 360.0
                except (ValueError, TypeError):
                    vel_dps = 0.0
            else:
                vel_dps = 0.0

        # --- 3. Current ---
        current_amp = None
        msg_iq = f"Axis{self.can.axisID}_Get_Iq"
        signals_iq = self.can.latest.get((self.can.axisID, msg_iq))
        if signals_iq and "Iq_Measured" in signals_iq:
            try:
                current_amp = float(signals_iq["Iq_Measured"])
            except (ValueError, TypeError):
                pass

        if current_amp is None:
            candidates = ["Iq_Measured", "motor_current", "current"]
            mname, sname, val = self.can.find_signal(self.can.axisID, candidates)
            if val is not None:
                try:
                    current_amp = float(val)
                except (ValueError, TypeError):
                    current_amp = 0.0
            else:
                current_amp = 0.0

        return pos_deg, vel_dps, current_amp

    def get_position(self):
        pos, _, _ = self.get_state()
        return pos
        
    def get_velocity(self):
        _, vel, _ = self.get_state()
        return vel

    def get_current(self):
        _, _, curr = self.get_state()
        return curr
        
    # the follow function are ment for live graph creating. and we will not use them

    def follow_position(self, stop_time=30):
        self.start_time = time.time()
        self.current_time = 0
        
        while self.current_time - self.start_time < stop_time:
            self.current_time = time.time()
            self.get_position()
            time.sleep(0.1)
    
    def follow_velocity(self, stop_time=30):
        self.start_time = time.time()
        self.current_time = 0
        
        while self.current_time - self.start_time < stop_time:
            self.current_time = time.time()
            self.get_velocity()
            time.sleep(0.1)

    def follow_current(self, stop_time=20):
        start_time = time.time()
        current_time = 0
        last_time = 0
        dt = 0.1 # seconds

        while current_time - start_time < stop_time:
            if current_time - last_time >= dt:
                last_time = current_time
                val = self.get_current()
                if val is not None:
                    print(f"Current reading: {val} A")
            current_time = time.time()

    def set_limit_current(self, max_current, max_velocity=5.0):
        try:
            self.can.send_dbc("Axis0_Set_Limits", {
                "Current_Limit": float(max_current),
                "Velocity_Limit": float(max_velocity)})
            print(f"{self.name} -> set current limit: {max_current} A")
        except Exception:
            self.alive = False

    # basic commands, for use in tests for the motor. those commands are not optimized for real time control, they are more for testing and debugging.

    def position_deg(self, deg, absolute=True):
        self.position_control()
        """Command position in degrees.

        If absolute==False (default) deg is treated as a relative delta (old behavior).
        If absolute==True deg is treated as an absolute output angle (degrees)
        relative to stored home baseline (home_deg).
        """
        if not self.alive:
            return

        if absolute:
            # absolute target in output degrees -> convert to motor turns
            target_deg = deg + self.home_deg
            target_turns = (target_deg / 360.0) * self.gear_ratio
            debug_src = "absolute"
        else:
            # relative: compute delta in motor turns and add to current reading or home baseline
            delta_turns = (deg / 360.0) * self.gear_ratio
            mname, sname, cur_turns = self.get_turns()
            if cur_turns is None:
                home_turns = (self.home_deg / 360.0) * self.gear_ratio
                target_turns = home_turns + delta_turns
                debug_src = "home_baseline"
            else:
                target_turns = cur_turns + delta_turns
                debug_src = f"{mname}.{sname}"

        try:
            self.can.send_dbc("Axis0_Set_Input_Pos", {
                "Input_Pos": float(target_turns),
                "Vel_FF": self.Vel_FF,
                "Torque_FF": self.Torque_FF
            })
            print(f"{self.name} -> pos cmd: target_turns={target_turns:.6f} (src={debug_src})")
        except Exception:
            self.alive = False

    def velocity_deg_s(self, vel):
        self.velocity_control()
        if not self.alive:
            return
        turns_s = (vel / 360.0) * self.gear_ratio
        try:
            self.can.send_dbc("Axis0_Set_Input_Vel", {
                "Input_Vel": turns_s,
                "Input_Torque_FF": self.Torque_FF
            })
        except Exception:
            self.alive = False
  
    def torque_nm(self, torque):
        self.torque_control()
        if not self.alive:
            return
        try:
            self.can.send_dbc("Axis0_Set_Input_Torque", {
                "Input_Torque": float(torque)
            })
            print(f"torque_nm command: {torque} Nm")
        except Exception:
            self.alive = False

    # this commands are for real use. they are faster and simpler

    def set_motor_position(self, deg):
        if not self.alive:
            return
        
        target_turns = (deg / 360.0) * self.gear_ratio

        try:
            self.can.send_dbc("Axis0_Set_Input_Pos", {
                "Input_Pos": float(target_turns),
                "Vel_FF": self.Vel_FF,
                "Torque_FF": self.Torque_FF
            })
        except Exception:
            self.alive = False

    def set_motor_velocity(self, vel):
        if not self.alive:
            return
        turns_s = (vel / 360.0) * self.gear_ratio

        try:
            self.can.send_dbc("Axis0_Set_Input_Vel", {
                "Input_Vel": turns_s,
                "Input_Torque_FF": self.Torque_FF
            })
        except Exception:
            self.alive = False

    def set_motor_torque(self, torque):
        if not self.alive:
            return
        try:
            self.can.send_dbc("Axis0_Set_Input_Torque", {
                "Input_Torque": float(torque)
            })
        except Exception:
            self.alive = False    

    def set_impedance(self, kp=0.01, kd=0.0, deg_eq=0.0, pos_deg=None, vel_dps=None):
        if not self.alive:
            return

        if pos_deg is None or vel_dps is None:
            pos_val, vel_val, _ = self.get_state()
            if pos_deg is None:
                pos_deg = pos_val
            if vel_dps is None:
                vel_dps = vel_val

        if pos_deg is None or vel_dps is None:
            return

        try:
            desired_torque = - kp * (pos_deg - deg_eq) - kd * vel_dps
            self.set_motor_torque(desired_torque)
        except Exception:
            self.alive = False

    def encoder_reset(self):
        if not self.alive:
            return
        try:
            self.can.send_dbc("Axis0_Set_Encoder_Pos", {"Encoder_Pos": 0.0})
        except Exception:
            self.alive = False

    def calibrate(self, wait_time=18.0):
        if not self.alive:
            print(f"→ {self.name}: not available for calibration")
            return
        print(f"→ {self.name}: STARTING FULL CALIBRATION (approx {int(wait_time)}s)")
        try:
            self.set_state(3)

            #state 3 is full calibration sequence
            start = time.time()
            while time.time() - start < wait_time:
                remaining = int(wait_time - (time.time() - start))
                print(f"  calibrating... {remaining:2d}s remaining", end="\r")
                time.sleep(0.5)
            print(" " * 60, end="\r")
            self.set_state(1)
            #state 1 is idle 
            time.sleep(0.8)
            self.set_state(8)
            #state 8 is closed loop control
            time.sleep(0.2)
            mname, sname, val = self.can.find_signal(self.can.axisID, ["pos_estimate", "pos", "encoder_pos", "encoder_count", "position"])
            if val is not None:
                try:
                    v = float(val)
                    if abs(v) > 1000:
                        turns = v / 4096.0
                    else:
                        turns = v
                    self.home_deg = turns * 360.0 / self.gear_ratio
                    print(f"→ {self.name}: Set home baseline from {mname}.{sname} = {val} -> home_deg={self.home_deg:.2f}")
                    
                    #a try to solve the jumping problem
                    self.set_state(1)
                    self.position_deg(self.home_deg)
                    time.sleep(1)
                    self.set_state(8)
                    time.sleep(0.2)
                except Exception:
                    self.home_deg = 0.0
            else:
                print(f"→ {self.name}: No encoder message seen yet; home baseline left at {self.home_deg:.2f} deg")
            print(f"→ {self.name}: CALIBRATION COMPLETE (approx)")
        except Exception:
            self.alive = False
            print(f"→ {self.name}: CALIBRATION FAILED")
       

    def save_config(self):
        if not self.alive:
            print(f"→ {self.name}: not available to save config")
            return
        try:
            arb = (self.can.axisID << 5) | 0x001
            self.can.send(arb, [0] * 8)
            print(f"→ {self.name}: SAVE CONFIG SENT")
        except Exception:
            self.alive = False
            print(f"→ {self.name}: SAVE CONFIG FAILED")

    def save_calibration_params(self):
        self.save_config()

    def reboot(self):
        if not self.alive:
            return
        try:
            self.can.send((self.can.axisID << 5) | 0x016, [0] * 8)
        except Exception:
            self.alive = False

    def print_encoder_position(self):
        try:
            candidates = ["pos_estimate", "pos", "encoder_pos", "encoder_count", "position"]
            mname, sname, val = self.can.find_signal(self.can.axisID, candidates)
            if val is not None:
                try:
                    v = float(val)
                    if abs(v) > 1000:
                        turns = v / 4096.0
                        note = "counts->turns(4096)"
                    else:
                        turns = v
                        note = "turns"
                    deg = turns * 360.0 / self.gear_ratio
                    print(f"{self.name} [{mname}.{sname}] raw={v} ({note}) -> {deg:.2f} deg (home {self.home_deg:.2f})")
                except Exception:
                    print(f"{self.name} [{mname}.{sname}] = {val}")
            else:
                dump = self.can.dump_axis(self.can.axisID)
                print(f"{self.name} encoder: no matched signal. Dump:\n{dump}")
        except Exception as e:
            print(f"{self.name} encoder: Error reading position ({e})")
