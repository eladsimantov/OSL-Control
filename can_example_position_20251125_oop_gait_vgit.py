#!/usr/bin/env python3
import os
import time
import threading
import math
import can
import cantools

# -----------------------------------------------------------
# CAN initialization
# -----------------------------------------------------------
os.system("sudo ip link set can0 up type can bitrate 250000")

# ===========================================================
# CLASS: ODriveCAN
# - background listener decodes incoming CAN frames and stores
#   the latest decoded signals per (axis, message_name).
# ===========================================================
class ODriveCAN:
    def __init__(self, bus_name="can0", node_id=1, dbc_path="/home/nuc/Downloads/odrive-cansimple.dbc"):
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


# ===========================================================
# CLASS: ODriveMotor (Single Motor)
# ===========================================================
class ODriveMotor:
    def __init__(self, can_interface: ODriveCAN, gear_ratio=20, name="motor"):
        self.can = can_interface
        self.gear_ratio = gear_ratio
        self.name = name
        self.alive = True
        self._home_deg = 0.0

    def set_state(self, state_id):
        try:
            self.can.send_dbc("Axis0_Set_Axis_State", {"Axis_Requested_State": state_id})
        except Exception:
            self.alive = False

    def closed_loop(self):
        print(f"→ {self.name}: CLOSED_LOOP_CONTROL")
        self.set_state(8)

    def idle(self):
        print(f"→ {self.name}: IDLE")
        self.set_state(1)

    def _read_current_turns(self):
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

    def position_deg(self, deg, absolute=True):
        """Command position in degrees.

        If absolute==False (default) deg is treated as a relative delta (old behavior).
        If absolute==True deg is treated as an absolute output angle (degrees)
        relative to stored home baseline (_home_deg).
        """
        if not self.alive:
            return

        if absolute:
            # absolute target in output degrees -> convert to motor turns
            target_deg = deg + self._home_deg
            target_turns = (target_deg / 360.0) * self.gear_ratio
            debug_src = "absolute"
        else:
            # relative: compute delta in motor turns and add to current reading or home baseline
            delta_turns = (deg / 360.0) * self.gear_ratio
            mname, sname, cur_turns = self._read_current_turns()
            if cur_turns is None:
                home_turns = (self._home_deg / 360.0) * self.gear_ratio
                target_turns = home_turns + delta_turns
                debug_src = "home_baseline"
            else:
                target_turns = cur_turns + delta_turns
                debug_src = f"{mname}.{sname}"

        try:
            self.can.send_dbc("Axis0_Set_Input_Pos", {
                "Input_Pos": float(target_turns),
                "Vel_FF": 30.0,
                "Torque_FF": 1.0
            })
            print(f"{self.name} -> pos cmd: target_turns={target_turns:.6f} (src={debug_src})")
        except Exception:
            self.alive = False

    def velocity_deg_s(self, vel):
        if not self.alive:
            return
        turns_s = (vel / 360.0) * self.gear_ratio
        try:
            self.can.send_dbc("Axis0_Set_Input_Vel", {
                "Input_Vel": turns_s,
                "Input_Torque_FF": 1.0
            })
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
            start = time.time()
            while time.time() - start < wait_time:
                remaining = int(wait_time - (time.time() - start))
                print(f"  calibrating... {remaining:2d}s remaining", end="\r")
                time.sleep(0.5)
            print(" " * 60, end="\r")
            self.set_state(1)
            time.sleep(0.8)
            self.set_state(8)
            time.sleep(0.2)
            mname, sname, val = self.can.find_signal(self.can.axisID, ["pos_estimate", "pos", "encoder_pos", "encoder_count", "position"])
            if val is not None:
                try:
                    v = float(val)
                    if abs(v) > 1000:
                        turns = v / 4096.0
                    else:
                        turns = v
                    self._home_deg = turns * 360.0 / self.gear_ratio
                    print(f"→ {self.name}: Set home baseline from {mname}.{sname} = {val} -> home_deg={self._home_deg:.2f}")
                except Exception:
                    self._home_deg = 0.0
            else:
                print(f"→ {self.name}: No encoder message seen yet; home baseline left at {self._home_deg:.2f} deg")
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
                    print(f"{self.name} [{mname}.{sname}] raw={v} ({note}) -> {deg:.2f} deg (home {self._home_deg:.2f})")
                except Exception:
                    print(f"{self.name} [{mname}.{sname}] = {val}")
            else:
                dump = self.can.dump_axis(self.can.axisID)
                print(f"{self.name} encoder: no matched signal. Dump:\n{dump}")
        except Exception as e:
            print(f"{self.name} encoder: Error reading position ({e})")


# ===========================================================
# CLASS: Dual Motor Walking Controller
# - supports adjustable phase offsets for knee and ankle
# ===========================================================
class DualMotorController:
    def __init__(self, motor_knee: ODriveMotor, motor_ankle: ODriveMotor,
                 knee_phase_deg=-30.0, ankle_phase_deg=0.0):
        self.knee = motor_knee
        motor_knee.gear_ratio = 20
        self.ankle = motor_ankle
        motor_ankle.gear_ratio = 20
        self.walking = False
        self.walk_thread = None
        self.knee_amp = 30.0
        self.ankle_amp = 30.0
        self.step_frequency = 0.8
        self.loop_dt = 0.02
        # phases stored in radians
        self.knee_phase = math.radians(knee_phase_deg)
        self.ankle_phase = math.radians(ankle_phase_deg)

    def set_phase_between(self, ankle_phase_deg):
        """Set ankle phase relative to knee (knee phase kept 0)."""
        self.knee_phase = 0.0
        self.ankle_phase = math.radians(ankle_phase_deg)
        print(f"→ phases set: knee {math.degrees(self.knee_phase):.1f}°, ankle {math.degrees(self.ankle_phase):.1f}°")

    def set_phases(self, knee_phase_deg, ankle_phase_deg):
        self.knee_phase = math.radians(knee_phase_deg)
        self.ankle_phase = math.radians(ankle_phase_deg)
        print(f"→ phases set: knee {knee_phase_deg:.1f}°, ankle {ankle_phase_deg:.1f}°")

    def inc_phases(self, knee_inc_deg=0.0, ankle_inc_deg=0.0):
        self.knee_phase += math.radians(knee_inc_deg)
        self.ankle_phase += math.radians(ankle_inc_deg)
        print(f"→ phases incremented: knee {math.degrees(self.knee_phase):.1f}°, ankle {math.degrees(self.ankle_phase):.1f}°")

    def reset_phases(self):
        self.knee_phase = 0.0
        self.ankle_phase = math.pi
        print("→ phases reset to defaults (knee 0°, ankle 180°)")

    def walking_loop(self):
        print("→ Walking cycle started")
        t_start = time.time()
        last_print = 0.0
        while self.walking:
            t = time.time() - t_start
            base = 2.0 * math.pi * self.step_frequency * t
            knee_pos = self.knee_amp * math.sin(base - self.knee_phase)
            ankle_pos = self.ankle_amp * math.sin(base - self.ankle_phase)
            if self.knee.alive:
                self.knee.position_deg(knee_pos)
            if self.ankle.alive:
                self.ankle.position_deg(ankle_pos)
            # occasional status print
            if time.time() - last_print > 2.0:
                last_print = time.time()
                print(f"  gait: knee_phase={math.degrees(self.knee_phase):.1f}°, ankle_phase={math.degrees(self.ankle_phase):.1f}°")
            time.sleep(self.loop_dt)
        print("→ Walking cycle stopped")

    def start_walking(self):
        if self.walking:
            print("Already walking.")
            return
        print("→ Starting WALK mode...")
        if self.knee.alive:
            self.knee.closed_loop()
        if self.ankle.alive:
            self.ankle.closed_loop()
        time.sleep(0.2)
        self.walking = True
        self.walk_thread = threading.Thread(target=self.walking_loop, daemon=True)
        self.walk_thread.start()

    def stop_walking(self):
        self.walking = False
        time.sleep(self.loop_dt * 2)
        if self.knee.alive:
            self.knee.idle()
        if self.ankle.alive:
            self.ankle.idle()


# ===========================================================
# CLASS: CommandInterface
# - adds commands to manipulate phase offsets
# ===========================================================
class CommandInterface:
    def __init__(self, motor1: ODriveMotor, motor2: ODriveMotor, walker: DualMotorController):
        self.motor1 = motor1
        self.motor2 = motor2
        self.walker = walker

    def print_menu(self):
        print("\n===== Dual-Motor Walking Controller =====")
        print("o              - closed loop both motors")
        print("f              - idle both motors")
        print("p1<p>          - knee by <p> degrees (relative)")
        print("p2<p>          - ankle by <p> degrees (relative)")
        print("v1<v>          - knee velocity")
        print("v2<v>          - ankle velocity")
        print("walk           - start walking")
        print("stop           - stop walking")
        print("ep             - print encoder positions (both)")
        print("dump1/dump2    - dump raw/decoded messages for axis")
        print("phase <deg>    - set ankle phase relative to knee (deg)")
        print("phaseboth k a  - set both phases (deg)")
        print("phaseinc k a   - increment phases by k,a degrees")
        print("phasereset     - reset phases to defaults (knee 0°, ankle 180°)")
        print("cal1 <s>       - calibrate knee (optional seconds)")
        print("cal2 <s>       - calibrate ankle (optional seconds)")
        print("save1          - save knee calibration params")
        print("save2          - save ankle calibration params")
        print("x              - exit\n")

    def process(self, cmd):
        if not cmd:
            return
        if cmd == "o":
            self.motor1.closed_loop()
            self.motor2.closed_loop()
        elif cmd == "f":
            self.motor1.idle()
            self.motor2.idle()
        elif cmd.startswith("p1"):
            try:
                deg = float(cmd[2:])
                self.motor1.position_deg(deg)
            except Exception:
                print("Invalid p1")
        elif cmd.startswith("p2"):
            try:
                deg = float(cmd[2:])
                self.motor2.position_deg(deg)
            except Exception:
                print("Invalid p2")
        elif cmd.startswith("v1"):
            try:
                vel = float(cmd[2:])
                self.motor1.velocity_deg_s(vel)
            except Exception:
                print("Invalid v1")
        elif cmd.startswith("v2"):
            try:
                vel = float(cmd[2:])
                self.motor2.velocity_deg_s(vel)
            except Exception:
                print("Invalid v2")
        elif cmd == "walk":
            self.walker.start_walking()
        elif cmd == "stop":
            self.walker.stop_walking()
        elif cmd == "ep":
            self.motor1.print_encoder_position()
            self.motor2.print_encoder_position()
        elif cmd == "dump1":
            print(self.motor1.can.dump_axis(self.motor1.can.axisID))
        elif cmd == "dump2":
            print(self.motor2.can.dump_axis(self.motor2.can.axisID))
        elif cmd.startswith("phaseboth"):
            parts = cmd.split()
            if len(parts) >= 3:
                try:
                    k = float(parts[1]); a = float(parts[2])
                    self.walker.set_phases(k, a)
                except Exception:
                    print("Invalid phaseboth args")
            else:
                print("Usage: phaseboth <k_deg> <a_deg>")
        elif cmd.startswith("phaseinc"):
            parts = cmd.split()
            try:
                k = float(parts[1]) if len(parts) > 1 else 0.0
                a = float(parts[2]) if len(parts) > 2 else 0.0
                self.walker.inc_phases(k, a)
            except Exception:
                print("Invalid phaseinc args")
        elif cmd.startswith("phase "):
            parts = cmd.split()
            if len(parts) >= 2:
                try:
                    a = float(parts[1])
                    self.walker.set_phase_between(a)
                except Exception:
                    print("Invalid phase arg")
            else:
                print("Usage: phase <ankle_deg>")
        elif cmd == "phasereset":
            self.walker.reset_phases()
        elif cmd.startswith("cal1"):
            parts = cmd.split()
            try:
                t = float(parts[1]) if len(parts) > 1 else 18.0
                self.motor1.calibrate(wait_time=t)
            except Exception:
                print("Invalid cal1 time")
        elif cmd.startswith("cal2"):
            parts = cmd.split()
            try:
                t = float(parts[1]) if len(parts) > 1 else 18.0
                self.motor2.calibrate(wait_time=t)
            except Exception:
                print("Invalid cal2 time")
        elif cmd == "save1":
            self.motor1.save_calibration_params()
        elif cmd == "save2":
            self.motor2.save_calibration_params()
        elif cmd == "?":
            self.print_menu()
        elif cmd == "x":
            print("Exiting safely...")
            self.walker.stop_walking()
            self.motor1.idle()
            self.motor2.idle()
            try:
                self.motor1.can.bus.shutdown()
            except Exception:
                pass
            exit()
        else:
            print("Unknown command")


# ===========================================================
# MAIN ENTRY
# ===========================================================
def main():
    can1 = ODriveCAN(node_id=1)
    knee = ODriveMotor(can1, name="knee")
    can2 = ODriveCAN(node_id=2)
    ankle = ODriveMotor(can2, name="ankle")
    walker = DualMotorController(knee, ankle)
    ui = CommandInterface(knee, ankle, walker)
    ui.print_menu()
    while True:
        try:
            cmd = input("→ ").strip().lower()
            if cmd:
                ui.process(cmd)
        except KeyboardInterrupt:
            print("Stopping...")
            walker.stop_walking()
            knee.idle()
            ankle.idle()
            try:
                knee.can.bus.shutdown()
            except Exception:
                pass
            break

if __name__ == "__main__":
    main()