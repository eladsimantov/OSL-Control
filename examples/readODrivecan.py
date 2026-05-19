#!/usr/bin/env python3
import os, time, struct
import can, cantools

CAN_CH = "can0"
BITRATE = 1000000
DBC = "/home/enable-lab/Desktop/OSL-Control/src/drivers/odrive-cansimple.dbc"
NODE_ID = 0          # axis node id (0..7)
POLL_INTERVAL = 0.02 # seconds between polls (50 Hz)
RUN_SECONDS = 0      # 0 = run forever
CALIBRATE = True     # set False to only poll

def setup_iface():
    os.system(f"sudo ip link set {CAN_CH} down")
    res = os.system(f"sudo ip link set {CAN_CH} up type can bitrate {BITRATE}")
    if res != 0:
        raise RuntimeError("Failed to set up CAN interface; ensure socketcan/can-utils installed and you have privileges")
    return can.interface.Bus(channel=CAN_CH, interface="socketcan")

def get_message_by_prefix(db, prefix):
    for m in db.messages:
        if m.name.startswith(prefix):
            return m
    return None

def get_get_encoder_frame_id(db, node):
    name = f"Axis{node}_Get_Encoder_Estimates"
    try:
        return db.get_message_by_name(name).frame_id
    except Exception:
        for m in db.messages:
            if "Get_Encoder_Estimates" in m.name:
                return m.frame_id
    return None

def make_poll_arbs(frame_id, node):
    return list(dict.fromkeys([frame_id, (frame_id << 5) | node, (node << 5) | frame_id]))

def decode_with_db(db, rx):
    rx_id = rx.arbitration_id & 0x7FF
    candidates = [rx_id, rx_id >> 5, rx_id & 0x1F]
    for fid in candidates:
        try:
            msg = db.get_message_by_frame_id(fid)
        except Exception:
            msg = None
        if not msg:
            continue
        try:
            decoded = msg.decode(rx.data)
            return msg.name, decoded
        except Exception:
            continue
    return None, None

def print_pos_vel(decoded):
    pos = None; vel = None
    for k in ("Pos_Estimate", "Pos", "Position", "Pos_turns"):
        if k in decoded:
            pos = decoded[k]; break
    for k in ("Vel_Estimate", "Vel", "Velocity"):
        if k in decoded:
            vel = decoded[k]; break
    return pos, vel

def find_set_state_message(db, node):
    candidates = [f"Axis{node}_Set_State", f"Axis{node}_Set_Axis_State", "Axis_Set_State", "Set_Axis_State", "Set_State"]
    for name in candidates:
        try:
            m = db.get_message_by_name(name)
            return m
        except Exception:
            continue
    for m in db.messages:
        if "set" in m.name.lower():
            sig_names = [s.name.lower() for s in m.signals]
            if any("state" in s for s in sig_names) or any(getattr(s, 'length', 0) in (8,16,32) for s in m.signals):
                return m
    return None

def send_calibrate_command(bus, db, node):
    target_state = 3  # FULL_CALIBRATION_SEQUENCE
    msg_def = find_set_state_message(db, node)
    if not msg_def:
        print("No suitable Set_State message found in DBC; cannot send calibration command.")
        return False

    state_sig = None
    for s in msg_def.signals:
        lname = s.name.lower()
        if "state" in lname or "axis_requested" in lname or "axisrequested" in lname or "axis_requested_state" in lname:
            state_sig = s.name
            break

    try:
        if state_sig:
            # try integer first; if DBC has enum you may also pass string like "FULL_CALIBRATION_SEQUENCE"
            payload = msg_def.encode({state_sig: target_state})
        else:
            payload = struct.pack("<I", target_state)
    except Exception as e:
        print("Encoding via DBC failed, fallback to raw pack:", e)
        payload = struct.pack("<I", target_state)

    arb = (msg_def.frame_id << 5) | node
    print(f"Sending calibration/state set to axis {node} arb=0x{arb:X} payload={payload.hex()}")
    try:
        bus.send(can.Message(arbitration_id=arb, data=payload, is_extended_id=False))
        return True
    except Exception as e:
        print("Send failed:", e)
        return False

def watch_state_transition(bus, db, node, timeout=10.0):
    end = time.time() + timeout
    state_msgs = []
    while time.time() < end:
        rx = bus.recv(timeout=0.2)
        if rx is None:
            continue
        name, decoded = decode_with_db(db, rx)
        if not name:
            continue
        # heuristic: messages that include 'State' or 'Get_State' or 'Axis{N}_' + 'State'
        if "state" in name.lower() or "get_state" in name.lower() or ("axis" in name.lower() and "state" in name.lower()):
            ts = time.time()
            print(f"{ts:.3f} {name} {decoded}")
            state_msgs.append((ts, name, decoded))
    return state_msgs

def main():
    bus = setup_iface()
    db = cantools.database.load_file(DBC)

    frame_id = get_get_encoder_frame_id(db, NODE_ID)
    if frame_id is None:
        print("Get_Encoder_Estimates frame id not found in DBC.")
        return

    poll_arbs = make_poll_arbs(frame_id, NODE_ID)
    print("Polling arbitration IDs:", [hex(a) for a in poll_arbs])
    print("Listening and printing Pos (turns) and Vel (turns/s)... Ctrl-C to stop")

    print("Sending test zero-length frames to poll arbs...")
    for arb in poll_arbs:
        try:
            bus.send(can.Message(arbitration_id=arb, data=b"", is_extended_id=False))
        except Exception as e:
            print("Send error:", e)

    if CALIBRATE:
        ok = send_calibrate_command(bus, db, NODE_ID)
        if not ok:
            print("Calibration command not sent (see message above).")
        else:
            print("Watching for state transitions for 10s...")
            watch_state_transition(bus, db, NODE_ID, timeout=10.0)

    start = time.time()
    try:
        while True:
            window_end = time.time() + POLL_INTERVAL
            for arb in poll_arbs:
                try:
                    bus.send(can.Message(arbitration_id=arb, data=b"", is_extended_id=False))
                except Exception:
                    pass

            while time.time() < window_end:
                rx = bus.recv(timeout=0.01)
                if rx is None:
                    continue
                name, decoded = decode_with_db(db, rx)
                if name is None:
                    print(f"RAW ID=0x{rx.arbitration_id:X} data={rx.data.hex()}")
                    continue
                pos, vel = print_pos_vel(decoded)
                ts = time.time()
                print(f"{ts:.3f} ID=0x{rx.arbitration_id:X} {name} pos={pos} vel={vel} raw={decoded}")

            if RUN_SECONDS and (time.time() - start) >= RUN_SECONDS:
                break

    except KeyboardInterrupt:
        pass
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
