#!/usr/bin/env python3
"""
test_phase_variable.py - Step-by-step test bench for the LocoLab Phase Variable
(src/locolabtools/phaseVar/LocolabPhaseVariable.so) using our own sensors:
the WitMotion thigh IMU (BLE) and the SRI M8123B2 loadcell (CAN).

The knee motor is NEVER commanded by this script.

Modes (run them in this order):

  1) selftest  - No hardware. Loads the .so and feeds it a synthetic gait.
                 Proves the library loads on this Pi and shows what "working" looks like.
        python3 tests/test_phase_variable.py selftest

  2) sensors   - Guided sign / axis / rate check of the IMU + loadcell (prompts you).
                 Prints the exact flags to use for the next two modes.
        sudo python3 tests/test_phase_variable.py sensors

  3) live      - Runs the phase variable live on the sensors, prints it, logs a CSV.
        sudo python3 tests/test_phase_variable.py live --duration 60 [flags from step 2]

  4) replay    - Re-runs a logged CSV through the .so offline (on the Pi), optionally with
                 different axis/sign/offset flags, so you can tune without re-walking.
        python3 tests/test_phase_variable.py replay logs/phase_var_XXXX.csv --plot [flags]

Input conventions expected by the LocoLab module (verified in selftest):
  - thighAngle_deg   : global thigh angle, 0 = vertical, POSITIVE = hip flexion (thigh forward)
  - thighVelocity_dps: its time derivative in deg/s (same sign convention)
  - Fz               : vertical load in N, NEGATIVE when the leg is loaded (OSL loadcell convention)
  - time             : seconds, monotonic
  - speed [0.8..1.2 m/s], incline [-10..+10 deg]
Our SRI loadcell reads Fz POSITIVE when loaded (see walking.py: fz > LOAD_LSTANCE), hence
the default --fz-sign -1.

Author: generated for the eNaBLe lab OSL-Control repo
"""

import argparse
import csv
import ctypes
import json
import math
import os
import platform
import struct
import sys
import time

import numpy as np

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(TESTS_DIR, ".."))
sys.path.insert(0, PROJECT_DIR)

DEFAULT_LIB_DIR = os.path.join(PROJECT_DIR, "src", "locolabtools", "phaseVar")
CONFIG_PATH = os.path.join(PROJECT_DIR, "src", "fsm", "config_walking.json")

try:
    with open(CONFIG_PATH, "r") as f:
        CONFIG = json.load(f)
except Exception:
    CONFIG = {}

AXES = {"x": 0, "y": 1, "z": 2}
LOG_FIELDS = [
    "t", "loop_dt", "imu_new",
    "euler_x", "euler_y", "euler_z",          # deg (after IMU zeroing)
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",  # deg/s
    "fz_raw",                                  # N, loadcell as read (after tare)
    "thigh_deg", "thigh_vel_dps", "Fz_in",     # what was fed to the library
    "phase", "stance_phase", "swing_phase", "state",
]


# =============================================================================
# Library wrapper
# =============================================================================
class _Inputs(ctypes.Structure):
    _fields_ = [
        ("thighAngle_deg", ctypes.c_double),
        ("thighVelocity_dps", ctypes.c_double),
        ("Fz", ctypes.c_double),
        ("time", ctypes.c_double),
        ("incline", ctypes.c_double),
        ("speed", ctypes.c_double),
    ]


class _Outputs(ctypes.Structure):
    _fields_ = [
        ("phase", ctypes.c_double),
        ("stancePhase", ctypes.c_double),
        ("swingPhase", ctypes.c_double),
        ("state", ctypes.c_double),
    ]


class PhaseVariable:
    """Thin ctypes wrapper - identical calling convention to opensourceleg's CompiledController
    (main(&inputs, &outputs)), but with a reset() so replays start from a clean state."""

    def __init__(self, lib_dir: str = DEFAULT_LIB_DIR, speed: float = 1.0, incline: float = 0.0):
        so_path = os.path.join(lib_dir, "LocolabPhaseVariable.so")
        if not os.path.isfile(so_path):
            raise FileNotFoundError(f"Library not found: {so_path}")
        try:
            self.lib = ctypes.CDLL(so_path)
        except OSError as e:
            raise OSError(
                f"{e}\n\n  LocolabPhaseVariable.so is a 32-bit ARM (armhf) library.\n"
                f"  This Python is {platform.machine()} / {struct.calcsize('P') * 8}-bit.\n"
                f"  It only loads from a 32-bit ARM Python (e.g. 32-bit Raspberry Pi OS).\n"
                f"  On 64-bit Pi OS (aarch64) or on a PC it will NOT load - that also means\n"
                f"  walking.py silently runs without the phase variable."
            ) from e
        self._main = self.lib.LocolabPhaseVariable
        self._main.argtypes = [ctypes.POINTER(_Inputs), ctypes.POINTER(_Outputs)]
        self._main.restype = None
        self.inputs = _Inputs()
        self.outputs = _Outputs()
        self.inputs.speed = speed
        self.inputs.incline = incline
        self._initialized = False
        self.reset()

    def reset(self):
        """The .so keeps its filter/FSM state in globals -> terminate + initialize to start fresh."""
        if self._initialized:
            self.lib.LocolabPhaseVariable_terminate()
        self.lib.LocolabPhaseVariable_initialize()
        self._initialized = True

    def step(self, thigh_deg, thigh_vel_dps, fz, t):
        self.inputs.thighAngle_deg = thigh_deg
        self.inputs.thighVelocity_dps = thigh_vel_dps
        self.inputs.Fz = fz
        self.inputs.time = t
        self._main(ctypes.byref(self.inputs), ctypes.byref(self.outputs))
        o = self.outputs
        return o.phase, o.stancePhase, o.swingPhase, o.state

    def close(self):
        if self._initialized:
            self.lib.LocolabPhaseVariable_terminate()
            self._initialized = False


# =============================================================================
# Signal conditioning (shared by live and replay so replay reproduces live exactly)
# =============================================================================
class Conditioner:
    def __init__(self, args):
        self.a_idx = AXES[args.thigh_axis]
        self.g_idx = AXES[args.gyro_axis or args.thigh_axis]
        self.thigh_sign = args.thigh_sign
        self.gyro_sign = args.gyro_sign if args.gyro_sign is not None else args.thigh_sign
        self.offset = args.thigh_offset
        self.fz_sign = args.fz_sign
        self.vel_source = args.vel_source
        self._prev_angle = None
        self._prev_t = None
        self._vel_lp = 0.0

    def __call__(self, t, euler_deg, gyro_dps, fz_raw):
        angle = self.thigh_sign * euler_deg[self.a_idx] + self.offset
        if self.vel_source == "gyro":
            vel = self.gyro_sign * gyro_dps[self.g_idx]
        else:  # numerical derivative + 1st-order low-pass (~10 Hz)
            if self._prev_t is None or t <= self._prev_t:
                raw = 0.0
            else:
                raw = (angle - self._prev_angle) / (t - self._prev_t)
            dt = (t - self._prev_t) if self._prev_t is not None else 0.01
            alpha = dt / (dt + 1.0 / (2 * math.pi * 10.0))
            self._vel_lp += alpha * (raw - self._vel_lp)
            vel = self._vel_lp
        self._prev_angle, self._prev_t = angle, t
        return angle, vel, self.fz_sign * fz_raw


# =============================================================================
# Analysis
# =============================================================================
def summarize(t, phase, state, imu_new=None, loop_dt=None, freq=None, title="SUMMARY"):
    t = np.asarray(t); phase = np.asarray(phase); state = np.round(np.asarray(state)).astype(int)
    print("\n" + "=" * 64 + f"\n  {title}\n" + "=" * 64)
    if len(t) < 10:
        print("  Not enough samples.")
        return {}
    dur = t[-1] - t[0]
    hs_idx = np.where((state[1:] <= 3) & (state[:-1] >= 4))[0] + 1   # swing -> stance
    to_idx = np.where((state[1:] >= 4) & (state[:-1] <= 3))[0] + 1   # stance -> swing
    stride_T = np.diff(t[hs_idx]) if len(hs_idx) > 1 else np.array([])
    dphi = np.diff(phase)
    backward = int(np.sum((dphi < -0.02) & (dphi > -0.5)))           # ignore the 1->0 wrap
    uniq, cnt = np.unique(state, return_counts=True)
    occupancy = {int(u): 100.0 * c / len(state) for u, c in zip(uniq, cnt)}

    print(f"  Duration            : {dur:.1f} s  ({len(t)} samples)")
    if loop_dt is not None and freq:
        ld = np.asarray(loop_dt)[1:]
        over = np.mean(ld > 1.5 / freq) * 100 if len(ld) else 0
        print(f"  Loop rate           : {1.0 / np.mean(ld):.1f} Hz (target {freq:.0f}), "
              f"{over:.1f}% iterations > 1.5*dt")
    if imu_new is not None:
        n_new = int(np.sum(imu_new))
        print(f"  IMU new samples     : {n_new / max(dur, 1e-6):.1f} Hz effective")
    print(f"  Heel strikes (6/5->1..3): {len(hs_idx)}   toe-offs (3->4): {len(to_idx)}")
    if len(stride_T):
        print(f"  Stride time         : {np.mean(stride_T):.2f} +/- {np.std(stride_T):.2f} s")
        if len(to_idx) and len(hs_idx) > 1:
            st_frac = []
            for a, b in zip(hs_idx[:-1], hs_idx[1:]):
                tos = to_idx[(to_idx > a) & (to_idx < b)]
                if len(tos):
                    st_frac.append((t[tos[0]] - t[a]) / (t[b] - t[a]))
            if st_frac:
                print(f"  Stance fraction     : {100 * np.mean(st_frac):.0f}% of stride (normal ~60%)")
    print(f"  Backward phase jumps: {backward}  (should be ~0)")
    print("  State occupancy     : " + ", ".join(f"S{k}:{v:.0f}%" for k, v in occupancy.items()))

    verdict = []
    if len(uniq) == 1:
        s = int(uniq[0])
        verdict.append(f"STUCK in state {s}. " + (
            "State 6 only -> the library never sees load: Fz sign is probably wrong "
            "(needs NEGATIVE when loaded -> flip --fz-sign) or loadcell not streaming."
            if s >= 4 else
            "Stance only -> never unloads / thigh never extends: check --thigh-sign and the loadcell tare."))
    elif not (set(uniq) & {1, 2, 3}) or not (set(uniq) & {4, 5, 6}):
        verdict.append("Only stance OR only swing states seen -> check Fz sign and thigh sign.")
    elif len(hs_idx) < 2:
        verdict.append("Fewer than 2 strides detected - walk longer / check signals.")
    else:
        verdict.append("Phase variable is cycling through stance and swing. Looks healthy." if backward < 0.1 * len(hs_idx) + 1
                       else "Cycling, but phase often runs backwards -> noisy/low-rate thigh signal.")
    print("\n  VERDICT: " + " ".join(verdict))
    print("=" * 64)
    return {"strides": len(hs_idx), "states": sorted(occupancy), "backward": backward}


def save_csv(path, rows):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=LOG_FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"\n[LOG] Saved {len(rows)} rows -> {path}")


def plot_rows(rows, png_path, title=""):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[PLOT] matplotlib not installed (pip install matplotlib) - skipping plot.")
        return
    t = np.array([r["t"] for r in rows], dtype=float)
    g = lambda k: np.array([r[k] for r in rows], dtype=float)
    fig, ax = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    ax[0].plot(t, g("thigh_deg")); ax[0].set_ylabel("thigh [deg]\n(+ = flexion)"); ax[0].grid(alpha=.3)
    ax[1].plot(t, g("thigh_vel_dps"), color="tab:orange"); ax[1].set_ylabel("thigh vel\n[deg/s]"); ax[1].grid(alpha=.3)
    ax[2].plot(t, g("Fz_in"), color="tab:green"); ax[2].set_ylabel("Fz into lib [N]\n(- = loaded)"); ax[2].grid(alpha=.3)
    ax[3].plot(t, g("phase"), label="phase", lw=2)
    ax[3].plot(t, g("stance_phase"), label="stance", alpha=.6)
    ax[3].plot(t, g("swing_phase"), label="swing", alpha=.6)
    ax3b = ax[3].twinx(); ax3b.step(t, g("state"), color="k", alpha=.35, where="post"); ax3b.set_ylabel("state"); ax3b.set_ylim(0, 7)
    ax[3].set_ylabel("phase"); ax[3].set_xlabel("time [s]"); ax[3].legend(loc="upper left"); ax[3].grid(alpha=.3)
    os.makedirs(os.path.dirname(os.path.abspath(png_path)), exist_ok=True)
    fig.suptitle(title); fig.tight_layout(); fig.savefig(png_path, dpi=110)
    print(f"[PLOT] Saved -> {png_path}")


# =============================================================================
# Mode 1: selftest (no hardware)
# =============================================================================
def synthetic_gait(fs=100.0, stride_T=1.1, n_strides=15, body_load=700.0, noise_N=3.0, seed=0):
    """Rough able-bodied walking at ~1 m/s: thigh +25 deg at heel strike, ~-13 deg at
    push-off, toe-off at 60%. Fz returned POSITIVE when loaded (like our SRI loadcell)."""
    rng = np.random.default_rng(seed)
    t = np.arange(0, n_strides * stride_T, 1 / fs)
    ph = (t / stride_T) % 1.0
    th = 8 + 18 * np.cos(2 * np.pi * ph) + 5 * np.sin(2 * np.pi * ph) - 3 * np.cos(4 * np.pi * ph)
    dth = np.gradient(th, 1 / fs)
    s = np.clip(ph / 0.6, 0, 1)
    fz = np.where(ph < 0.6, body_load * np.sin(np.pi * s) ** 0.6 * (1 - 0.25 * np.sin(2 * np.pi * s) ** 2), 0.0)
    fz = fz + rng.normal(0, noise_N, len(t))
    return t, ph, th, dth, fz


def run_selftest(args):
    print("=" * 64 + "\n  SELFTEST - library + synthetic gait (no hardware)\n" + "=" * 64)
    print(f"  Python   : {sys.version.split()[0]}  {platform.machine()}  {struct.calcsize('P') * 8}-bit")
    print(f"  Lib dir  : {args.lib_path}")
    pv = PhaseVariable(args.lib_path, args.speed, args.incline)
    print("  [OK] LocolabPhaseVariable.so loaded and initialized.")

    t, ph_true, th, dth, fz_sensor = synthetic_gait(fs=args.freq)

    def run_case(fz_sign, thigh_sign, label):
        pv.reset()
        rows = []
        for i in range(len(t)):
            a, v, f = thigh_sign * th[i], thigh_sign * dth[i], fz_sign * fz_sensor[i]
            p, sp, swp, st = pv.step(a, v, f, t[i])
            rows.append(dict(t=t[i], loop_dt=1 / args.freq, imu_new=1, euler_x=0, euler_y=a, euler_z=0,
                             gyro_x_dps=0, gyro_y_dps=v, gyro_z_dps=0, fz_raw=fz_sensor[i],
                             thigh_deg=a, thigh_vel_dps=v, Fz_in=f, phase=p, stance_phase=sp,
                             swing_phase=swp, state=st))
        phase = np.array([r["phase"] for r in rows]); state = np.array([r["state"] for r in rows])
        m = t > 5.0  # skip the first strides while it adapts
        err = np.angle(np.exp(1j * 2 * np.pi * (phase[m] - ph_true[m]))) / (2 * np.pi)
        res = summarize(t, phase, state, title=label)
        print(f"  Phase RMS error vs synthetic truth: {np.sqrt(np.mean(err ** 2)):.3f}  "
              f"(synthetic thigh curve is approximate - < 0.15 is fine)")
        return rows, res

    rows, good = run_case(-1, +1, "CASE A: correct conventions (Fz<0 loaded, thigh + flexion)")
    _, bad_fz = run_case(+1, +1, "CASE B (negative control): Fz POSITIVE when loaded")

    ok = good.get("strides", 0) >= 10 and {1, 2, 3} & set(good["states"]) and {4, 5, 6} & set(good["states"])
    print("\n  RESULT: " + ("PASS - library works on this machine." if ok else "FAIL - see summaries above."))
    if bad_fz.get("strides", 0) == 0:
        print("  Case B confirms: with Fz positive-when-loaded the module never enters stance.\n"
              "  -> our SRI loadcell must be fed with --fz-sign -1 (walking.py currently passes it unflipped).")
    if args.plot:
        plot_rows(rows, os.path.join(args.log_dir, "phase_var_selftest.png"), "Selftest - synthetic gait")
    pv.close()


# =============================================================================
# Hardware helpers
# =============================================================================
def setup_hardware(args, need_loadcell=True):
    from src.adapters.imu import WitMotionIMUAdapter
    from src.adapters.loadcell import SRILoadCell_M8123B2
    try:
        from opensourceleg.logging import LOGGER, LogLevel
        LOGGER.set_stream_level(LogLevel.ERROR)
    except Exception:
        pass

    loadcell = None
    if need_loadcell:
        os.system(f"sudo ip link set {args.can} down")
        os.system(f"sudo ip link set {args.can} up type can bitrate {args.bitrate} sample-point 0.750")
        os.system(f"sudo ip link set {args.can} txqueuelen 1000")
        loadcell = SRILoadCell_M8123B2(tag="Shank LC", channel=args.can)
        loadcell.start()
        if not loadcell.is_streaming:
            raise RuntimeError("Loadcell failed to start (check CAN / candump).")

    os.system("sudo rfkill block bluetooth")
    os.system("sudo rfkill unblock bluetooth")
    time.sleep(1.0)
    imu = WitMotionIMUAdapter(tag="Thigh IMU", mac_address=args.thigh_mac, connection_type="ble")
    imu.start()
    if not imu.is_streaming:
        raise RuntimeError(f"Thigh IMU {args.thigh_mac} not streaming (BLE).")

    if loadcell is not None:
        input("\n>> Lift the foot OFF the ground (loadcell unloaded), then press Enter to tare... ")
        loadcell.calibrate()
    input(">> Stand upright, THIGH VERTICAL and still, then press Enter to zero the IMU... ")
    imu.calibrate()
    print("[OK] Sensors ready.\n")
    return imu, loadcell


def read_sensors(imu, loadcell):
    if loadcell is not None:
        loadcell.update()
    imu.update()
    d = imu.data  # single locked read -> consistent snapshot
    euler = d["euler"]
    gyro = [math.degrees(g) for g in d["gyro"]]  # adapter gives rad/s
    fz = loadcell.fz if loadcell is not None else 0.0
    sig = tuple(d["acc"]) + tuple(d["gyro"])   # changes only when a new IMU packet arrived
    return euler, gyro, fz, sig


def shutdown_hardware(args, imu, loadcell):
    try:
        if imu is not None:
            imu.stop()
    except Exception:
        pass
    try:
        if loadcell is not None:
            loadcell.stop()
            os.system(f"sudo ip link set {args.can} down")
    except Exception:
        pass
    print("[INFO] Sensors stopped.")


def capture(imu, loadcell, seconds, rate=100.0):
    E, G, F, T, new = [], [], [], [], 0
    last, t0 = None, time.perf_counter()
    while (now := time.perf_counter()) - t0 < seconds:
        e, g, f, sig = read_sensors(imu, loadcell)
        new += sig != last; last = sig
        E.append(e); G.append(g); F.append(f); T.append(now - t0)
        time.sleep(max(0.0, 1.0 / rate - (time.perf_counter() - now)))
    return np.array(E), np.array(G), np.array(F), np.array(T), new


# =============================================================================
# Mode 2: guided sensor check
# =============================================================================
def run_sensors(args):
    imu = loadcell = None
    try:
        imu, loadcell = setup_hardware(args, need_loadcell=True)
        print("--- Step 0: data rates (keep still, 3 s) ---")
        E, G, F, T, new = capture(imu, loadcell, 3.0)
        imu_hz = new / T[-1]
        print(f"  IMU effective update rate : {imu_hz:.0f} Hz "
              + ("[OK]" if imu_hz >= 50 else "[WARN] < 50 Hz - raise the WitMotion output rate (WitMotion app, 100-200 Hz)"))
        print(f"  Thigh at rest (deg) x/y/z : {E.mean(0).round(2)}   (should be ~0 after zeroing)")
        print(f"  Gyro at rest (dps)  x/y/z : {G.mean(0).round(2)}   noise std {G.std(0).round(2)}")
        print(f"  Fz at rest (N)            : {F.mean():.1f} +/- {F.std():.1f}")

        input("\n--- Step 1: swing the thigh FORWARD (hip flexion) ~30 deg and HOLD it. Press Enter, hold 2 s ---")
        E1, *_ = capture(imu, loadcell, 2.0)
        delta = E1.mean(0) - E.mean(0)
        ax = int(np.argmax(np.abs(delta)))
        thigh_axis, thigh_sign = "xyz"[ax], int(np.sign(delta[ax]) or 1)
        print(f"  Change per axis (deg): {delta.round(1)}  -> thigh axis = {thigh_axis}, sign = {thigh_sign:+d}")
        if abs(delta[ax]) < 10:
            print("  [WARN] Less than 10 deg change - did the thigh actually move?")

        input("\n--- Step 2: swing the thigh back and forth continuously. Press Enter, swing for 5 s ---")
        E2, G2, _, T2, _ = capture(imu, loadcell, 5.0)
        ang = thigh_sign * E2[:, ax]
        dang = np.gradient(ang, T2)
        best, best_c = 0, 0.0
        for k in range(3):
            c = np.corrcoef(dang, G2[:, k])[0, 1] if np.std(G2[:, k]) > 1e-6 else 0.0
            print(f"  corr(d(angle)/dt, gyro_{'xyz'[k]}) = {c:+.2f}")
            if abs(c) > abs(best_c):
                best, best_c = k, c
        # dang is already in the corrected (+ = flexion) frame, so the gyro sign is just sign(corr)
        gyro_axis, gyro_sign = "xyz"[best], int(np.sign(best_c) or 1)
        scale = np.std(G2[:, best]) / max(np.std(dang), 1e-6)
        print(f"  -> gyro axis = {gyro_axis}, sign = {gyro_sign:+d}  (|corr| {abs(best_c):.2f}, "
              f"gyro/derivative amplitude ratio {scale:.2f} - should be ~1)")
        print(f"  Thigh range during swing: {ang.min():.1f} .. {ang.max():.1f} deg")
        if abs(best_c) < 0.8:
            print("  [WARN] weak correlation - IMU rate too low or axes not aligned; consider --vel-source diff")

        input("\n--- Step 3: put WEIGHT on the leg / press the foot down firmly. Press Enter, hold 2 s ---")
        _, _, F3, _, _ = capture(imu, loadcell, 2.0)
        dF = F3.mean() - F.mean()
        fz_sign = -int(np.sign(dF) or 1)
        print(f"  Fz change: {dF:+.1f} N  -> raw loadcell is {'POSITIVE' if dF > 0 else 'NEGATIVE'} when loaded "
              f"-> --fz-sign {fz_sign:+d} (library needs NEGATIVE when loaded)")
        if abs(dF) < 50:
            print("  [WARN] < 50 N change - is the loadcell streaming / really loaded?")

        print("\n" + "=" * 64 + "\n  RECOMMENDED FLAGS for live / replay:\n" + "=" * 64)
        print(f"  --thigh-axis {thigh_axis} --thigh-sign {thigh_sign} "
              f"--gyro-axis {gyro_axis} --gyro-sign {gyro_sign} --fz-sign {fz_sign}")
        print("  In walking.py this means:\n"
              f"    thighAngle_deg    = {thigh_sign:+d} * thigh_imu.euler_{thigh_axis}\n"
              f"    thighVelocity_dps = {gyro_sign:+d} * math.degrees(thigh_imu.gyro_{gyro_axis})\n"
              f"    Fz                = {fz_sign:+d} * loadcell.fz")
    finally:
        shutdown_hardware(args, imu, loadcell)


# =============================================================================
# Mode 3: live
# =============================================================================
def run_live(args):
    from opensourceleg.utilities import SoftRealtimeLoop
    pv = PhaseVariable(args.lib_path, args.speed, args.incline)
    cond = Conditioner(args)
    imu = loadcell = None
    rows = []
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.log_dir, f"phase_var_{stamp}.csv")
    try:
        imu, loadcell = setup_hardware(args, need_loadcell=not args.no_loadcell)
        print("Streaming phase variable (knee motor is NOT driven). Ctrl+C to stop.\n")
        last_sig, t_prev = None, None
        pv.reset()
        loop = SoftRealtimeLoop(dt=1.0 / args.freq)
        for t in loop:
            if args.duration and t >= args.duration:
                loop.stop(); break
            euler, gyro, fz_raw, sig = read_sensors(imu, loadcell)
            new = int(sig != last_sig); last_sig = sig
            a, v, f = cond(t, euler, gyro, fz_raw)
            p, sp, swp, st = pv.step(a, v, f, t)
            rows.append(dict(t=t, loop_dt=(t - t_prev) if t_prev is not None else 0.0, imu_new=new,
                             euler_x=euler[0], euler_y=euler[1], euler_z=euler[2],
                             gyro_x_dps=gyro[0], gyro_y_dps=gyro[1], gyro_z_dps=gyro[2], fz_raw=fz_raw,
                             thigh_deg=a, thigh_vel_dps=v, Fz_in=f, phase=p, stance_phase=sp,
                             swing_phase=swp, state=st))
            t_prev = t
            if len(rows) % max(1, int(args.freq / 10)) == 0:
                bar = "#" * int(p * 30)
                print(f"\r t={t:6.1f}s | thigh {a:6.1f} deg {v:7.1f} dps | Fz {f:7.1f} N | "
                      f"phase {p:4.2f} [{bar:<30}] st {sp:4.2f} sw {swp:4.2f} S{int(st)}", end="", flush=True)
    finally:
        print()
        shutdown_hardware(args, imu, loadcell)
        pv.close()
        if rows:
            save_csv(csv_path, rows)
            summarize([r["t"] for r in rows], [r["phase"] for r in rows], [r["state"] for r in rows],
                      imu_new=[r["imu_new"] for r in rows], loop_dt=[r["loop_dt"] for r in rows],
                      freq=args.freq, title="LIVE SUMMARY")
            if args.plot:
                plot_rows(rows, csv_path.replace(".csv", ".png"), f"Live - {os.path.basename(csv_path)}")
            print(f"\nTip: re-tune offline with\n  python3 {os.path.relpath(__file__)} replay {csv_path} --plot [flags]")


# =============================================================================
# Mode 4: replay
# =============================================================================
def run_replay(args):
    with open(args.csv) as f:
        src = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    print(f"[REPLAY] {len(src)} rows from {args.csv}")
    pv = PhaseVariable(args.lib_path, args.speed, args.incline)
    cond = Conditioner(args)
    rows = []
    for r in src:
        euler = [r["euler_x"], r["euler_y"], r["euler_z"]]
        gyro = [r["gyro_x_dps"], r["gyro_y_dps"], r["gyro_z_dps"]]
        a, v, fz = cond(r["t"], euler, gyro, r["fz_raw"])
        p, sp, swp, st = pv.step(a, v, fz, r["t"])
        out = dict(r); out.update(thigh_deg=a, thigh_vel_dps=v, Fz_in=fz, phase=p,
                                  stance_phase=sp, swing_phase=swp, state=st)
        rows.append(out)
    pv.close()
    out_path = args.csv.replace(".csv", "_replay.csv")
    save_csv(out_path, rows)
    summarize([r["t"] for r in rows], [r["phase"] for r in rows], [r["state"] for r in rows],
              imu_new=[r["imu_new"] for r in rows], loop_dt=[r["loop_dt"] for r in rows],
              freq=args.freq, title="REPLAY SUMMARY")
    if args.plot:
        plot_rows(rows, out_path.replace(".csv", ".png"), f"Replay - {os.path.basename(args.csv)}")


# =============================================================================
# CLI
# =============================================================================
def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["selftest", "sensors", "live", "replay"])
    p.add_argument("csv", nargs="?", help="CSV log to replay (replay mode)")
    p.add_argument("--lib-path", default=DEFAULT_LIB_DIR)
    p.add_argument("--freq", type=float, default=float(CONFIG.get("FREQUENCY", 100.0)), help="loop Hz")
    p.add_argument("--speed", type=float, default=float(CONFIG.get("WALKING_SPEED", 1.0)))
    p.add_argument("--incline", type=float, default=float(CONFIG.get("WALKING_INCLINE", 0.0)))
    p.add_argument("--duration", type=float, default=0.0, help="live: stop after N s (0 = Ctrl+C)")
    # signal mapping
    p.add_argument("--thigh-axis", choices="xyz", default="y", help="IMU euler axis for thigh angle")
    p.add_argument("--thigh-sign", type=int, choices=[-1, 1], default=1)
    p.add_argument("--thigh-offset", type=float, default=0.0, help="deg added after sign (mounting offset)")
    p.add_argument("--gyro-axis", choices="xyz", default=None, help="default: same as --thigh-axis")
    p.add_argument("--gyro-sign", type=int, choices=[-1, 1], default=None, help="default: same as --thigh-sign")
    p.add_argument("--vel-source", choices=["gyro", "diff"], default="gyro")
    p.add_argument("--fz-sign", type=int, choices=[-1, 1], default=-1,
                   help="multiplier on loadcell.fz so that loaded = NEGATIVE (SRI reads + when loaded)")
    # hardware
    p.add_argument("--can", default=CONFIG.get("can_interface", "can0"))
    p.add_argument("--bitrate", type=int, default=int(CONFIG.get("bitrate", 1000000)))
    p.add_argument("--thigh-mac", default=CONFIG.get("thigh_imu_mac", "EF:D5:AC:1A:0D:21"))
    p.add_argument("--no-loadcell", action="store_true", help="live: IMU only (Fz = 0, will not cycle)")
    # output
    p.add_argument("--log-dir", default=os.path.join(PROJECT_DIR, "logs"))
    p.add_argument("--plot", action="store_true", help="save a PNG (needs matplotlib)")
    return p


def main():
    args = build_parser().parse_args()
    os.makedirs(args.log_dir, exist_ok=True)
    if args.mode == "selftest":
        run_selftest(args)
    elif args.mode == "sensors":
        run_sensors(args)
    elif args.mode == "live":
        run_live(args)
    elif args.mode == "replay":
        if not args.csv:
            sys.exit("replay needs a CSV path")
        run_replay(args)


if __name__ == "__main__":
    main()
