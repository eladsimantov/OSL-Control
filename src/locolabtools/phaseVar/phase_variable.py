"""
phase_variable.py - Load the LocoLab phase variable on any Raspberry Pi OS (32- or 64-bit).

LocolabPhaseVariable.so is a 32-bit ARM (armhf) library. A 64-bit Python (aarch64) cannot
load it with ctypes ("wrong ELF class: ELFCLASS32"). Here's how this module handles that:

  1. It tries ctypes first. That works on 32-bit Pi OS.
  2. Otherwise it starts `pv_helper`, a tiny 32-bit program that links the .so and serves
     it over a pipe (binary protocol, ~tens of microseconds per call natively). The helper
     is started, in order of preference:
       a) directly                         (needs armhf multiarch libc: libc6:armhf)
       b) via the armhf loader from the cross packages, natively on the CPU
       c) under qemu-arm                   (works even when the kernel can't run 32-bit code,
                                            e.g. the Pi 5 default 16K-page kernel)

One-time setup on 64-bit Pi OS (Debian bookworm):
    sudo apt install libc6-armhf-cross libstdc++6-armhf-cross libgomp1-armhf-cross qemu-user
    # optional, to (re)build the helper on the Pi:
    sudo apt install gcc-arm-linux-gnueabihf

Two APIs are available:
    pv = PhaseVariable(lib_dir, speed=1.0, incline=0.0)
    phase, stance, swing, state = pv.step(thigh_deg, thigh_vel_dps, fz, t)

    # or drop-in style for code that used opensourceleg's CompiledController:
    pv.inputs.thighAngle_deg = ...; pv.inputs.thighVelocity_dps = ...
    pv.inputs.Fz = ...; pv.inputs.time = t
    out = pv.run();  out.phase, out.stancePhase, out.swingPhase, out.state

Input conventions: thigh angle global (0 = vertical, + = flexion) in deg; velocity deg/s;
Fz in N, NEGATIVE when the leg is loaded; time in s.
"""

import atexit
import ctypes
import os
import select
import shutil
import stat
import subprocess

LIB_NAME = "LocolabPhaseVariable.so"
HELPER_NAME = "pv_helper"
HELPER_SRC = "pv_helper.c"
ARMHF_ROOT = "/usr/arm-linux-gnueabihf"
ARMHF_LOADER = os.path.join(ARMHF_ROOT, "lib", "ld-linux-armhf.so.3")
# 64K segment alignment so the helper also loads on 16K-page kernels (Pi 5 default)
PAGE_FLAGS = ["-Wl,-z,max-page-size=0x10000", "-Wl,-z,common-page-size=0x10000"]


class Inputs(ctypes.Structure):
    _fields_ = [
        ("thighAngle_deg", ctypes.c_double),
        ("thighVelocity_dps", ctypes.c_double),
        ("Fz", ctypes.c_double),
        ("time", ctypes.c_double),
        ("incline", ctypes.c_double),
        ("speed", ctypes.c_double),
    ]


class Outputs(ctypes.Structure):
    _fields_ = [
        ("phase", ctypes.c_double),
        ("stancePhase", ctypes.c_double),
        ("swingPhase", ctypes.c_double),
        ("state", ctypes.c_double),
    ]


class _CtypesBackend:
    name = "ctypes (in-process)"

    def __init__(self, lib_dir):
        self.lib = ctypes.CDLL(os.path.join(lib_dir, LIB_NAME))
        self.lib.LocolabPhaseVariable.argtypes = [ctypes.POINTER(Inputs), ctypes.POINTER(Outputs)]
        self.lib.LocolabPhaseVariable.restype = None
        self.lib.LocolabPhaseVariable_initialize()
        self._alive = True

    def step(self, inputs, outputs):
        self.lib.LocolabPhaseVariable(ctypes.byref(inputs), ctypes.byref(outputs))

    def reset(self):
        self.lib.LocolabPhaseVariable_terminate()
        self.lib.LocolabPhaseVariable_initialize()

    def close(self):
        if self._alive:
            self.lib.LocolabPhaseVariable_terminate()
            self._alive = False


class _HelperBackend:
    """Runs pv_helper (32-bit) as a child process and talks to it over stdin/stdout."""

    def __init__(self, lib_dir, timeout=5.0):
        helper = os.path.join(lib_dir, HELPER_NAME)
        _ensure_helper(lib_dir, helper)
        lib_path = f"{lib_dir}:{os.path.join(ARMHF_ROOT, 'lib')}"
        candidates = [
            ("armhf helper (native, multiarch - optional, needs libc6:armhf)", [helper]),
            ("armhf helper (native, cross loader)", [ARMHF_LOADER, "--library-path", lib_path, helper]),
            ("armhf helper (qemu-arm emulation)", [shutil.which("qemu-arm") or "qemu-arm", "-L", ARMHF_ROOT, helper]),
        ]
        errors = []
        for name, cmd in candidates:
            if not os.path.exists(cmd[0]):
                errors.append(f"  - {name}: {cmd[0]} not found")
                continue
            try:
                proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, bufsize=0, cwd=lib_dir)
            except OSError as e:
                errors.append(f"  - {name}: {e}")
                continue
            self.proc = proc
            try:
                if self._request(b"P", 1, timeout) == b"p":
                    self.name = name
                    atexit.register(self.close)
                    return
            except Exception as e:
                err = ""
                try:
                    proc.kill()
                    err = proc.stderr.read().decode(errors="ignore").strip()
                except Exception:
                    pass
                errors.append(f"  - {name}: {err or e}")
        hint = ("\n\nOn 64-bit Pi OS run once:\n"
                "  sudo apt install libc6-armhf-cross libstdc++6-armhf-cross libgomp1-armhf-cross qemu-user")
        page = os.sysconf("SC_PAGE_SIZE")
        if page > 4096 or any("page-aligned" in e or "map segment" in e for e in errors):
            hint += (f"\n\nThis kernel uses {page // 1024} KB memory pages (Pi 5 default kernel_2712).\n"
                     "Some 32-bit libraries are only 4 KB aligned and cannot be loaded on it, and\n"
                     "qemu-user cannot emulate them either. Fix: switch to the 4 KB-page kernel:\n"
                     "  echo 'kernel=kernel8.img' | sudo tee -a /boot/firmware/config.txt && sudo reboot\n"
                     "  (check afterwards: getconf PAGESIZE  -> 4096)")
        raise RuntimeError("Could not start the 32-bit phase-variable helper:\n" + "\n".join(errors) + hint)

    def _read_exact(self, n, timeout):
        buf = b""
        fd = self.proc.stdout
        while len(buf) < n:
            if timeout is not None:
                r, _, _ = select.select([fd], [], [], timeout)
                if not r:
                    raise TimeoutError("phase-variable helper did not answer")
            chunk = fd.read(n - len(buf))
            if not chunk:
                raise RuntimeError("phase-variable helper exited unexpectedly")
            buf += chunk
        return buf

    def _request(self, payload, n_reply, timeout=None):
        self.proc.stdin.write(payload)
        return self._read_exact(n_reply, timeout)

    def step(self, inputs, outputs):
        reply = self._request(b"S" + bytes(inputs), ctypes.sizeof(Outputs), timeout=1.0)
        ctypes.memmove(ctypes.byref(outputs), reply, len(reply))

    def reset(self):
        if self._request(b"R", 1, timeout=2.0) != b"r":
            raise RuntimeError("phase-variable helper: bad reset reply")

    def close(self):
        proc = getattr(self, "proc", None)
        if proc is None or proc.poll() is not None:
            return
        try:
            proc.stdin.write(b"Q")
            proc.stdin.close()
            proc.wait(timeout=1.0)
        except Exception:
            proc.kill()


def _ensure_helper(lib_dir, helper):
    """Make sure pv_helper exists and is executable (git/OneDrive often drop the +x bit)."""
    if not os.path.isfile(helper):
        src = os.path.join(lib_dir, HELPER_SRC)
        cc = shutil.which("arm-linux-gnueabihf-gcc")
        if not (cc and os.path.isfile(src)):
            raise FileNotFoundError(
                f"{helper} missing. Build it with:\n  cd {lib_dir} && arm-linux-gnueabihf-gcc -O2 -o pv_helper "
                f"pv_helper.c -L. -l:{LIB_NAME} -Wl,-rpath,'$ORIGIN' {' '.join(PAGE_FLAGS)}\n"
                "(sudo apt install gcc-arm-linux-gnueabihf)")
        subprocess.run([cc, "-O2", "-o", helper, src, "-L", lib_dir, f"-l:{LIB_NAME}",
                        "-Wl,-rpath,$ORIGIN", *PAGE_FLAGS], check=True)
    mode = os.stat(helper).st_mode
    if not mode & stat.S_IXUSR:
        try:
            os.chmod(helper, mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        except OSError:
            pass


class PhaseVariable:
    def __init__(self, lib_dir=None, speed=1.0, incline=0.0, backend="auto"):
        """backend: 'auto' (ctypes, else helper), 'ctypes', or 'helper'."""
        lib_dir = os.path.abspath(lib_dir or os.path.dirname(os.path.abspath(__file__)))
        if not os.path.isfile(os.path.join(lib_dir, LIB_NAME)):
            raise FileNotFoundError(f"{LIB_NAME} not found in {lib_dir}")
        self._backend = None
        if backend in ("auto", "ctypes"):
            try:
                self._backend = _CtypesBackend(lib_dir)
            except OSError as e:
                if backend == "ctypes":
                    raise
                self.ctypes_error = str(e)
        if self._backend is None:
            self._backend = _HelperBackend(lib_dir)
        self.backend = self._backend.name
        self.inputs = Inputs()
        self.outputs = Outputs()
        self.inputs.speed = speed
        self.inputs.incline = incline

    def run(self):
        """CompiledController-style call: uses self.inputs, returns self.outputs."""
        self._backend.step(self.inputs, self.outputs)
        return self.outputs

    def step(self, thigh_deg, thigh_vel_dps, fz, t):
        self.inputs.thighAngle_deg = thigh_deg
        self.inputs.thighVelocity_dps = thigh_vel_dps
        self.inputs.Fz = fz
        self.inputs.time = t
        o = self.run()
        return o.phase, o.stancePhase, o.swingPhase, o.state

    def reset(self):
        """The library keeps its filter/FSM state in globals; this starts it fresh."""
        self._backend.reset()

    def close(self):
        if self._backend is not None:
            self._backend.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
