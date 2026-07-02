#!/usr/bin/env python
"""First-run setup helper for the servo aligner.

Run it and pick an action from the menu (or "Full guided setup" to do them all,
in order). The individual options are:

* **Python dependencies** -- check ``requirements.txt`` and offer to
  ``pip install`` any that are missing.
* **machine.yaml** -- create it from the template and edit it interactively:
  hardware, the ``servo.channels`` map (built from a live bus scan) and the
  channel grouping ``masks``. Comments are preserved by round-tripping through
  ``ruyaml``, which is required for this step.
* **calibration.yaml** -- same, for the optics / optimizer-tuning values.
* **Servo bus** -- the register steps doc/motor.md otherwise asks you to do by
  hand in FEETECH's FD tool: assign a unique bus **ID** to a servo (factory
  default is 1), enable hardware multi-turn feedback (**Register 18 -> 124**)
  and set the min/max angle limits (**Registers 9 & 11**, 0..4095; both 0
  disables the travel limit for multi-turn absolute positioning).
* **Scan the bus** -- just list the servo IDs currently responding.
* **Identify a servo** -- briefly jog one selected servo back and forth so you
  can see which physical motor a given id is.

Run it from the repo root before anything else:

    python app/init_helper.py

This module talks to the bus directly via ``scservo_sdk``; it deliberately does
NOT import ``servodriver``/``config``, so it is safe to run before the YAML
files exist. The only action that moves a servo is "Identify a servo", which
jogs the one you pick and returns it to where it started; every other step only
reads, or writes the ID / phase / angle-limit / EEPROM-lock registers.
"""

import argparse
import contextlib
import glob
import importlib
import io
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import _bootstrap  # noqa: F401 -- prepend ../src on sys.path (for the lazily-imported scservo_sdk)

# Shared curses TUI toolkit (item language, theme, status line, input helpers).
from tui import (
    curses, tui_available, _quiet, _init_theme, _status, _flash_status,
    _it_action, _it_run, _it_toggle, _it_edit, _to_int, _tui_page, _tui_message,
    _tui_input, _tui_input_int, _tui_confirm,
)

# --- YAML backend ------------------------------------------------------------
# The wizard round-trips the config files so the rich comments in the templates
# survive a load->edit->dump. That REQUIRES a ruamel-family library: ``ruyaml``
# (the community fork this project standardises on) or upstream ``ruamel.yaml``,
# which is API-identical. There is deliberately no PyYAML fallback -- PyYAML
# would silently drop every comment on write, so we force the round-tripper and
# error if none is present. The backend is *optional at import time* so the
# wizard can bootstrap a bare environment: the dependency step below pip-installs
# ruyaml, after which _resolve_yaml_backend() re-detects it. _load_text/_dump
# raise only if used with no round-tripper at all.
_RUAMEL = False       # True when a ruamel-family (comment-preserving) backend is active
_yaml = None          # the configured round-trip YAML() instance, when available
# ruyaml/ruamel symbols used by the node builders (None until resolved):
YAML = CommentedMap = CommentedSeq = HexInt = SingleQuotedScalarString = None

# Round-trip backends to try, in order; ruyaml is preferred. ruyaml exposes YAML
# at ``ruyaml`` with submodules ``ruyaml.comments`` etc.; ruamel.yaml mirrors the
# same layout rooted at ``ruamel.yaml``. So the package name doubles as the
# submodule prefix.
_RT_BACKENDS = ("ruyaml", "ruamel.yaml")


def _resolve_yaml_backend():
    """(Re)detect the best available round-trip backend and set the module globals."""
    global _RUAMEL, _yaml
    global YAML, CommentedMap, CommentedSeq, HexInt, SingleQuotedScalarString
    for base in _RT_BACKENDS:
        try:
            root = importlib.import_module(base)
            comments = importlib.import_module(base + ".comments")
            scalarint = importlib.import_module(base + ".scalarint")
            scalarstring = importlib.import_module(base + ".scalarstring")
        except Exception:  # pragma: no cover - environment dependent
            continue
        YAML = root.YAML
        CommentedMap, CommentedSeq = comments.CommentedMap, comments.CommentedSeq
        HexInt = scalarint.HexInt
        SingleQuotedScalarString = scalarstring.SingleQuotedScalarString
        y = YAML()
        y.preserve_quotes = True
        y.width = 4096  # don't wrap long lists (e.g. masks) across lines
        y.indent(mapping=2, sequence=4, offset=2)
        _yaml, _RUAMEL = y, True
        return
    _yaml, _RUAMEL = None, False


_NO_BACKEND = "ruyaml is required to read/write the config files -- install it (pip install ruyaml)."


def _load_text(text):
    if not _RUAMEL:
        raise RuntimeError(_NO_BACKEND)
    return _yaml.load(text)


def _dump(obj, fh):
    if not _RUAMEL:
        raise RuntimeError(_NO_BACKEND)
    _yaml.dump(obj, fh)


_resolve_yaml_backend()


# --- STS control-table addresses used for first-time setup -------------------
ADDR_ID = 5         # bus ID (EEPROM)
ADDR_MIN_ANGLE = 9  # min angle limit (EEPROM, 2 bytes)
ADDR_MAX_ANGLE = 11  # max angle limit (EEPROM, 2 bytes)
ADDR_PHASE = 18     # phase setting / feedback mode (EEPROM)
ADDR_TORQUE_ENABLE = 40  # RAM; 1 = hold. Used only by the "identify" jog.
PHASE_SINGLE_TURN = 108  # factory default (single-turn feedback)
PHASE_MULTI_TURN = 124   # 108 + BIT4 -> report full multi-turn angle
# Valid range for the angle-limit registers (9 & 11): one encoder turn is
# 0..4095. Setting both limits to 0 disables the travel limit, which the STS
# memory table requires for multi-turn absolute position control (factory
# default is 0 / 4095).
ANGLE_LIMIT_LO = 0
ANGLE_LIMIT_HI = 4095

INTERACTIVE = sys.stdin.isatty() and sys.stdout.isatty()


# =============================================================================
# Small interactive-prompt helpers
# =============================================================================
def _prompt(label, default):
    """Show ``label [default]:`` and return the raw (stripped) reply."""
    d = "" if default is None else str(default)
    suffix = f" [{d}]" if d != "" else ""
    try:
        return input(f"  {label}{suffix}: ").strip()
    except EOFError:
        return ""


def confirm(question, default=True):
    d = "Y/n" if default else "y/N"
    try:
        raw = input(f"{question} [{d}]: ").strip().lower()
    except EOFError:
        return default
    if raw == "":
        return default
    return raw in ("y", "yes")


def ask_str(label, current):
    raw = _prompt(label, current)
    return raw if raw != "" else current


def ask_int(label, current, lo=None, hi=None):
    while True:
        raw = _prompt(label, current)
        if raw == "":
            return current
        try:
            val = int(raw, 0)  # base 0 also accepts 0x.. / 0b..
        except ValueError:
            print("    not an integer, try again")
            continue
        if (lo is not None and val < lo) or (hi is not None and val > hi):
            print(f"    out of range ({lo}..{hi}), try again")
            continue
        return val


def ask_float(label, current):
    while True:
        raw = _prompt(label, current)
        if raw == "":
            return current
        try:
            return float(raw)
        except ValueError:
            print("    not a number, try again")


def ask_bool(label, current):
    while True:
        raw = _prompt(label + " (y/n)", "y" if current else "n").lower()
        if raw == "":
            return current
        if raw in ("y", "yes", "true", "1"):
            return True
        if raw in ("n", "no", "false", "0"):
            return False
        print("    answer y or n")


def ask_hex(label, current):
    cur = current if isinstance(current, str) else hex(int(current))
    raw = _prompt(label + " (hex ok, e.g. 0x68)", cur)
    if raw == "":
        return current
    try:
        return _hexint(int(raw, 0))
    except ValueError:
        print("    not a number, keeping current")
        return current


def ask_choice(label, options, default):
    while True:
        raw = _prompt(label, default).lower()
        if raw == "":
            return default
        if raw in options:
            return raw
        print(f"    choose one of {sorted(options)}")


def section(title):
    bar = "=" * max(8, len(title))
    print(f"\n{bar}\n{title}\n{bar}")


# =============================================================================
# YAML node builders (keep the template's inline styling where it matters)
# =============================================================================
def _hexint(v):
    if _RUAMEL:
        try:
            return HexInt(int(v))
        except Exception:
            return int(v)
    return int(v)


def _seq(values, flow=False):
    if _RUAMEL:
        s = CommentedSeq(values)
        if flow:
            s.fa.set_flow_style()
        return s
    return list(values)


def _channel_node(sid, name):
    """A single inline ``{id: , name: }`` channel entry (flow style under ruamel)."""
    if _RUAMEL:
        m = CommentedMap()
        m["id"] = int(sid)
        m["name"] = SingleQuotedScalarString(str(name))
        m.fa.set_flow_style()
        return m
    return {"id": int(sid), "name": str(name)}


def _build_channels(rows):
    """rows = [(id, name), ...] -> block list of inline {id: , name: } maps."""
    out = CommentedSeq() if _RUAMEL else []
    for sid, name in rows:
        out.append(_channel_node(sid, name))
    return out


# Channel-map lookups/mutators keyed by servo id, used by the TUI to persist
# edits back into machine.yaml's servo.channels list.
def _channel_for_id(channels, sid):
    for c in channels:
        if int(c["id"]) == int(sid):
            return c
    return None


def _channel_name(channels, sid):
    c = _channel_for_id(channels, sid)
    return str(c["name"]) if c is not None else ""


def _set_channel_id(channels, old_id, new_id):
    c = _channel_for_id(channels, old_id)
    if c is not None:
        c["id"] = int(new_id)
    else:
        channels.append(_channel_node(new_id, ""))


def _set_channel_name(channels, sid, name):
    c = _channel_for_id(channels, sid)
    if c is not None:
        c["name"] = SingleQuotedScalarString(str(name)) if _RUAMEL else str(name)
    else:
        channels.append(_channel_node(sid, name))


# =============================================================================
# Generic interactive editor over a loaded YAML tree
# =============================================================================
# Big structural sections that a first-time user almost never edits by hand;
# default the "edit this section?" gate to No for these.
_SECTION_GATE_DEFAULT = {
    "masks": False,
    "accept_functions": False,
    "coupling_vectors": False,
}


def ask_scalar(key, val):
    if key == "address":  # ADC I2C address reads best in hex
        return ask_hex(key, val)
    if isinstance(val, bool):
        return ask_bool(key, val)
    if isinstance(val, int):
        return ask_int(key, val)
    if isinstance(val, float):
        return ask_float(key, val)
    return ask_str(key, val)


def _fmt_nums(values):
    return ", ".join(str(x) for x in values)


def _parse_nums(raw, as_int):
    parts = [p for chunk in raw.split(",") for p in chunk.split()]
    return [int(p) if as_int else float(p) for p in parts]


def edit_list(key, val):
    if len(val) == 0:
        return val
    # list of numbers (a mask, an optimizer vector, ...)
    if all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in val):
        sample = val[0]
        as_int = isinstance(sample, int)
        raw = _prompt(f"{key} ({len(val)} values, space/comma separated)", _fmt_nums(val))
        if raw == "":
            return val
        try:
            return _seq(_parse_nums(raw, as_int), flow=True)
        except ValueError:
            print("    couldn't parse numbers, keeping current")
            return val
    # list of rows (e.g. coupling_vectors: list of vectors)
    if all(isinstance(x, list) for x in val):
        print(f"  {key}: {len(val)} rows")
        return [edit_list(f"{key}[{i}]", row) for i, row in enumerate(val)]
    # list of mappings
    if all(isinstance(x, dict) for x in val):
        for i, item in enumerate(val):
            print(f"  {key}[{i}]:")
            edit_mapping(item, f"{key}[{i}]", gate=False)
        return val
    return val


def edit_mapping(node, name=None, gate=True):
    for key in list(node.keys()):
        val = node[key]
        label = f"{name}.{key}" if name else key
        if isinstance(val, dict):
            default = _SECTION_GATE_DEFAULT.get(key, True)
            if gate and not confirm(f"  edit section '{label}'?", default=default):
                continue
            print(f"  -- {label} --")
            edit_mapping(val, label, gate=False)
        elif isinstance(val, list):
            node[key] = edit_list(label, val)
        else:
            node[key] = ask_scalar(key, val)


# =============================================================================
# Serial bus access (lazy SDK import so config editing works without pyserial)
# =============================================================================
class Bus:
    """Thin wrapper over the FEETECH SDK for scanning and EEPROM writes."""

    def __init__(self, port, ph, comm_success):
        self.port = port
        self.ph = ph
        self.OK = comm_success
        self.device = None   # serial device path, set by open()

    @classmethod
    def open(cls, devices, baudrate):
        try:
            from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts
        except Exception as e:
            print(f"  serial SDK unavailable ({e}); skipping bus operations")
            return None
        for dev in devices:
            try:
                port = PortHandler(str(dev))
                port.baudrate = int(baudrate)
                if not port.openPort():
                    continue
                if not port.setBaudRate(int(baudrate)):
                    port.closePort()
                    continue
                print(f"  opened {dev} @ {baudrate} baud")
                inst = cls(port, sms_sts(port), COMM_SUCCESS)
                inst.device = str(dev)
                return inst
            except Exception as e:
                print(f"  {dev}: {e}")
        print("  could not open any serial device")
        return None

    def ping(self, sid):
        _, comm, _ = self.ph.ping(sid)
        return comm == self.OK

    def scan(self, id_range):
        found = []
        for sid in id_range:
            if self.ping(sid):
                found.append(sid)
        return found

    def read1(self, sid, addr):
        val, comm, _ = self.ph.read1ByteTxRx(sid, addr)
        return val if comm == self.OK else None

    def read2(self, sid, addr):
        val, comm, _ = self.ph.read2ByteTxRx(sid, addr)
        return val if comm == self.OK else None

    def write_eeprom1(self, sid, addr, value, lock_id=None):
        """Unlock EEPROM, write one byte, re-lock. ``lock_id`` defaults to ``sid``
        but must be the *new* id when the write changes the servo's own id."""
        self.ph.unLockEprom(sid)
        res, _ = self.ph.write1ByteTxRx(sid, addr, value)
        self.ph.LockEprom(sid if lock_id is None else lock_id)
        return res == self.OK

    def write_eeprom2(self, sid, addr, value, lock_id=None):
        """Unlock EEPROM, write one 2-byte word, re-lock. Used for the angle
        limit registers (9 / 11), which are 2 bytes each."""
        self.ph.unLockEprom(sid)
        res, _ = self.ph.write2ByteTxRx(sid, addr, value)
        self.ph.LockEprom(sid if lock_id is None else lock_id)
        return res == self.OK

    def identify(self, sid, sweep=350, speed=1500, acc=30, cycles=3, settle=0.6):
        """Jog one servo back and forth so you can see which motor it is.

        Reads the present position, enables torque, wiggles +/- ``sweep`` counts
        around it a few times, returns to the start, then disables torque. This
        is the ONLY operation in this module that moves a servo.
        """
        pos, comm, _ = self.ph.ReadPos(sid)
        if comm != self.OK:
            print(f"    id {sid}: not responding")
            return False
        base = pos if 0 <= pos <= 4095 else 2048   # keep the wiggle inside one turn
        lo, hi = max(0, base - sweep), min(4095, base + sweep)
        self.ph.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)
        try:
            for _ in range(cycles):
                self.ph.WritePosEx(sid, hi, speed, acc)
                time.sleep(settle)
                self.ph.WritePosEx(sid, lo, speed, acc)
                time.sleep(settle)
            self.ph.WritePosEx(sid, base, speed, acc)   # back to where it started
            time.sleep(settle)
        finally:
            self.ph.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
        return True

    def close(self):
        try:
            self.port.closePort()
        except Exception:
            pass


def scan_for_channels(devices, baudrate, id_max):
    bus = Bus.open(devices, baudrate)
    if bus is None:
        return None
    try:
        print(f"  scanning ids 1..{id_max} ...")
        return bus.scan(range(1, id_max + 1))
    finally:
        bus.close()


# =============================================================================
# Servo register operations (the "do it in FD by hand" steps)
# =============================================================================
def assign_ids(bus, id_max):
    print(
        "\n  Assign servo IDs. Ideally connect ONE un-IDed servo at a time so it\n"
        "  can be detected unambiguously (new servos ship as id 1)."
    )
    while True:
        present = bus.scan(range(0, id_max + 1))
        if len(present) == 1:
            cur = present[0]
            print(f"  one servo present: current id = {cur}")
        elif present:
            print(f"  servos present: {present}")
            cur = ask_int("which current id do you want to change?", present[0])
        else:
            print("  no servo detected on the bus")
            cur = ask_int("enter the current id to change", 1)

        new = ask_int("new id (1-252)", cur)
        if not (0 <= new <= 252):
            print("    id out of range (0-252)")
            continue
        if new != cur and new in present:
            if not confirm(
                f"  id {new} is already on the bus -- writing it will collide. continue?",
                default=False,
            ):
                continue
        if confirm(f"  write id {cur} -> {new}?", default=True):
            ok = bus.write_eeprom1(cur, ADDR_ID, new, lock_id=new)
            if ok and bus.ping(new):
                print(f"    servo now responds as id {new}  ✓")
            else:
                print("    write failed -- check wiring/power and that only the target is connected")
        if not confirm("  assign another id?", default=False):
            break


def setup_multiturn(bus, ids):
    ids = sorted(set(ids))
    if not ids:
        print("  no servos to configure")
        return
    print(f"  enabling multi-turn feedback (Register 18 -> {PHASE_MULTI_TURN}) on ids: {ids}")
    for sid in ids:
        cur = bus.read1(sid, ADDR_PHASE)
        if cur is None:
            print(f"    id {sid}: not responding, skipped")
            continue
        if cur == PHASE_MULTI_TURN:
            print(f"    id {sid}: already {cur}, ok")
            continue
        if not confirm(f"  id {sid}: set register 18 {cur} -> {PHASE_MULTI_TURN}?", default=True):
            continue
        bus.write_eeprom1(sid, ADDR_PHASE, PHASE_MULTI_TURN)
        chk = bus.read1(sid, ADDR_PHASE)
        if chk == PHASE_MULTI_TURN:
            print(f"    id {sid}: {cur} -> {chk}  ✓")
        else:
            print(f"    id {sid}: write failed (reads {chk})")
    print(
        "\n  Note: per doc/motor.md, a driver-board power cycle resets the turn\n"
        "  count -- re-zero / re-home before trusting absolute angles."
    )


def setup_angle_limits(bus, ids):
    """Set the min/max angle limit registers (9 & 11) on the given servos.

    Both are plain integers in ``ANGLE_LIMIT_LO..ANGLE_LIMIT_HI`` (one encoder
    turn, 0..4095). Setting both to 0 disables the hardware travel limit, which
    the STS memory table requires for multi-turn absolute position control; the
    factory default (0 / 4095) otherwise clamps every command into a single turn.
    """
    ids = sorted(set(ids))
    if not ids:
        print("  no servos to configure")
        return
    print("  current angle limits (min/max):")
    for sid in ids:
        lo, hi = bus.read2(sid, ADDR_MIN_ANGLE), bus.read2(sid, ADDR_MAX_ANGLE)
        print(f"    id {sid}: {lo}/{hi}" if lo is not None else f"    id {sid}: no response")

    new_lo = ask_int("min angle limit (register 9)", ANGLE_LIMIT_LO,
                     lo=ANGLE_LIMIT_LO, hi=ANGLE_LIMIT_HI)
    new_hi = ask_int("max angle limit (register 11)", ANGLE_LIMIT_LO,
                     lo=ANGLE_LIMIT_LO, hi=ANGLE_LIMIT_HI)
    if new_hi < new_lo:
        print("    max limit is below min limit; nothing written")
        return
    print(f"  writing angle limits {new_lo}/{new_hi} on ids: {ids}")
    for sid in ids:
        lo, hi = bus.read2(sid, ADDR_MIN_ANGLE), bus.read2(sid, ADDR_MAX_ANGLE)
        if lo is None or hi is None:
            print(f"    id {sid}: not responding, skipped")
            continue
        if lo == new_lo and hi == new_hi:
            print(f"    id {sid}: already {lo}/{hi}, ok")
            continue
        bus.write_eeprom2(sid, ADDR_MIN_ANGLE, new_lo)
        bus.write_eeprom2(sid, ADDR_MAX_ANGLE, new_hi)
        clo, chi = bus.read2(sid, ADDR_MIN_ANGLE), bus.read2(sid, ADDR_MAX_ANGLE)
        if clo == new_lo and chi == new_hi:
            print(f"    id {sid}: {lo}/{hi} -> {clo}/{chi}  ✓")
        else:
            print(f"    id {sid}: write failed (reads {clo}/{chi})")


def register_setup(devices, baudrate, channels, id_max):
    bus = Bus.open(devices, baudrate)
    if bus is None:
        print("  could not open the bus; skipping register setup")
        return
    try:
        found = bus.scan(range(1, id_max + 1))
        print(f"  servos found on the bus: {found if found else 'none'}")

        if confirm("assign / change a servo bus id?", default=False):
            assign_ids(bus, id_max)
            found = bus.scan(range(1, id_max + 1))
            print(f"  servos now on the bus: {found if found else 'none'}")

        # Read Register 18 and the angle limits first so you can see which
        # servos still need configuring.
        targets = found or [int(c["id"]) for c in channels]
        if targets:
            print(f"  current Register 18 ({PHASE_SINGLE_TURN}=single-turn, "
                  f"{PHASE_MULTI_TURN}=multi-turn)  +  angle limits (registers 9/11):")
            for sid in targets:
                cur = bus.read1(sid, ADDR_PHASE)
                if cur is None:
                    print(f"    id {sid}: no response")
                    continue
                tag = {PHASE_MULTI_TURN: "  (multi-turn, already enabled)",
                       PHASE_SINGLE_TURN: "  (single-turn)"}.get(cur, "")
                lo, hi = bus.read2(sid, ADDR_MIN_ANGLE), bus.read2(sid, ADDR_MAX_ANGLE)
                print(f"    id {sid}: {cur}{tag}    limits {lo}/{hi}")

        if confirm(
            "enable hardware multi-turn (Register 18 -> 124) on the servos?",
            default=True,
        ):
            setup_multiturn(bus, targets)

        if confirm(
            "set the min/max angle limits (Registers 9 & 11)?",
            default=True,
        ):
            setup_angle_limits(bus, targets)
    finally:
        bus.close()


# =============================================================================
# Python dependencies
# =============================================================================
# requirements.txt is the single source of truth for *which* packages and which
# versions. The only thing we hardcode is the handful of distributions whose
# import name differs from the pip name -- needed to probe "is it importable?"
# *before* installing (that mapping can't be derived from the dist name alone).
_IMPORT_NAME = {
    "PyYAML": "yaml",
    "pyserial": "serial",
    "pyzmq": "zmq",
}


def _find_requirements():
    here = Path(__file__).resolve().parent
    for cand in (here.parent / "requirements.txt", here / "requirements.txt"):
        if cand.exists():
            return cand
    return None


def _parse_requirements(req_path):
    """Parse requirements.txt -> list of (dist_name, import_name).

    Strips comments, version specifiers, extras and environment markers; the
    file itself stays the source of truth for the names and version floors.
    """
    pkgs = []
    for raw in req_path.read_text().splitlines():
        spec = raw.split("#", 1)[0].strip()
        if not spec:
            continue
        dist = re.split(r"[<>=!~;@ \[]", spec, maxsplit=1)[0].strip()
        if dist:
            pkgs.append((dist, _IMPORT_NAME.get(dist, dist)))
    return pkgs


def _missing_packages(pkgs):
    """Subset of (dist, import) pairs whose import name is not importable."""
    missing = []
    for dist, imp in pkgs:
        try:
            importlib.import_module(imp)
        except Exception:
            missing.append(dist)
    return missing


def install_dependencies():
    """Check requirements.txt and offer to pip-install whatever is missing."""
    req = _find_requirements()
    if req is None:
        print("  requirements.txt not found; skipping dependency check")
        return
    pkgs = _parse_requirements(req)
    missing = _missing_packages(pkgs)
    print(f"  {len(pkgs) - len(missing)}/{len(pkgs)} packages from {req.name} already importable")
    if not missing:
        print("  all dependencies satisfied ✓")
        return
    print("  missing: " + ", ".join(missing))

    if not confirm(f"  pip-install the missing packages now (into {sys.executable})?", default=True):
        print(f"  skipped. Install later with:  {sys.executable} -m pip install -r {req}")
        return

    if confirm(f"  install everything from {req.name} rather than just the missing ones?", default=True):
        cmd = [sys.executable, "-m", "pip", "install", "-r", str(req)]
    else:
        cmd = [sys.executable, "-m", "pip", "install", *missing]
    print(f"  running: {' '.join(cmd)}")
    try:
        rc = subprocess.call(cmd)
    except Exception as e:
        print(f"  pip install could not run ({e}); install manually and re-run.")
        return
    if rc != 0:
        print("  pip install failed; resolve manually and re-run.")
        return

    # Pick up anything we just installed (notably the YAML backend this wizard
    # uses). invalidate_caches() lets the running interpreter see the new dists.
    importlib.invalidate_caches()
    was_rt = _RUAMEL
    _resolve_yaml_backend()
    still = _missing_packages(pkgs)
    if still:
        print("  still missing after install: " + ", ".join(still))
    else:
        print("  all dependencies satisfied ✓")
    if (not was_rt) and _RUAMEL:
        print("  (comment-preserving YAML now active -- the config files will keep their comments)")


# =============================================================================
# Config-file creation / editing
# =============================================================================
def find_config_dir(override=None):
    if override:
        return Path(override)
    here = Path(__file__).resolve().parent
    candidates = []
    env = os.environ.get("SERVO_ALIGNER_CONFIG_DIR")
    if env:
        candidates.append(Path(env))
    candidates += [here.parent / "config", here]
    for d in candidates:
        if (d / "machine.template.yaml").exists():
            return d
    return here.parent / "config"


def _backup(path):
    bak = path.with_name(path.name + ".bak")
    shutil.copy2(path, bak)
    print(f"  backed up existing {path.name} -> {bak.name}")


def prepare_config(name, config_dir):
    """Return (cfg, target_path); cfg is None if the user chooses to skip."""
    template = config_dir / f"{name}.template.yaml"
    target = config_dir / f"{name}.yaml"
    if not template.exists():
        print(f"  template {template} not found; skipping {name}.yaml")
        return None, target
    if target.exists():
        print(f"  {target.name} already exists.")
        choice = ask_choice(
            "[e]dit existing / [o]verwrite from template / [s]kip",
            options={"e", "o", "s"},
            default="e",
        )
        if choice == "s":
            print(f"  keeping {target.name} as-is")
            return None, target
        text = (template if choice == "o" else target).read_text()
        return _load_text(text), target
    print(f"  creating {target.name} from template")
    return _load_text(template.read_text()), target


def write_config(cfg, target):
    if target.exists():
        _backup(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    with open(target, "w") as fh:
        _dump(cfg, fh)
    print(f"  wrote {target}")


def _print_channels(channels):
    print("  current channel map (index: id, name):")
    for i, c in enumerate(channels):
        print(f"    {i}: id={c['id']}  name={c['name']}")


def edit_devices(devices):
    detected = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if detected:
        print(f"  detected serial ports: {', '.join(detected)}")
    raw = _prompt(
        "serial device(s), comma-separated (first that opens wins)",
        ", ".join(str(d) for d in devices),
    )
    if raw == "":
        return devices
    items = [s.strip() for s in raw.split(",") if s.strip()]
    return _seq(items)


def edit_channels(channels, devices, baudrate, scan_max):
    _print_channels(channels)
    if confirm("scan the servo bus now to discover ids?", default=True):
        found = scan_for_channels(devices, baudrate, scan_max)
        if found:
            print(f"  discovered ids (in bus order): {found}")
            if confirm("build the channel map from these ids?", default=True):
                rows = []
                for i, sid in enumerate(found):
                    default_name = str(channels[i]["name"]) if i < len(channels) else f"s{sid}"
                    name = ask_str(f"channel {i}: name for servo id {sid}", default_name)
                    rows.append((sid, name))
                return _build_channels(rows)
        elif found is not None:
            print("  no servos found on the bus")

    if confirm("edit the channel map manually?", default=not channels):
        n = ask_int("number of channels", len(channels))
        rows = []
        for i in range(n):
            cur_id = channels[i]["id"] if i < len(channels) else i + 1
            cur_name = channels[i]["name"] if i < len(channels) else f"s{i + 1}"
            sid = ask_int(f"channel {i}: servo id", cur_id)
            name = ask_str(f"channel {i}: name", cur_name)
            rows.append((sid, name))
        return _build_channels(rows)
    return channels


def edit_machine(cfg, scan_max):
    serial = cfg["serial"]
    print("  -- serial bus --")
    serial["devices"] = edit_devices(serial["devices"])
    serial["baudrate"] = ask_int("baudrate", serial["baudrate"])

    servo = cfg["servo"]
    print("  -- servo defaults --")
    servo["speed"] = ask_int("speed", servo["speed"])
    servo["acc"] = ask_int("acc", servo["acc"])
    if confirm("  edit de-hysteresis settings?", default=False):
        edit_mapping(servo["de_hysteresis"], "de_hysteresis", gate=False)

    print("  -- channel map --")
    servo["channels"] = edit_channels(
        servo["channels"], serial["devices"], serial["baudrate"], scan_max
    )
    # Channel grouping masks live alongside the channel map; rarely hand-edited
    # (they follow the wiring), so default the gate to No.
    if "masks" in servo and confirm("  edit channel grouping masks?", default=False):
        edit_mapping(servo["masks"], "masks", gate=False)

    if confirm("  edit ADC (photodiode) settings?", default=True):
        edit_mapping(cfg["adc"], "adc", gate=False)
    if confirm("  edit filesystem paths?", default=False):
        edit_mapping(cfg["paths"], "paths", gate=False)
    if confirm("  edit ZMQ server identity?", default=True):
        edit_mapping(cfg["server"], "server", gate=False)


def _bus_params(machine_cfg, machine_path):
    """Serial device list, baudrate and channels, from memory or disk."""
    cfg = machine_cfg
    if cfg is None and machine_path.exists():
        cfg = _load_text(machine_path.read_text())
    if cfg is None:
        return [], 1000000, []
    devices = [str(d) for d in cfg["serial"]["devices"]]
    baud = int(cfg["serial"]["baudrate"])
    channels = list(cfg["servo"]["channels"])
    return devices, baud, channels


# =============================================================================
# CLI arguments
# =============================================================================
def parse_args(argv=None):
    p = argparse.ArgumentParser(description="First-run setup wizard for the servo aligner.")
    p.add_argument("--config-dir", help="directory holding the *.template.yaml files")
    p.add_argument("--scan-max", type=int, default=20, help="highest servo id to scan (default 20)")
    p.add_argument("--full-scan", action="store_true", help="scan the full id range 1..252")
    p.add_argument("--no-deps", action="store_true", help="skip the dependency check / install step")
    p.add_argument("--no-bus", action="store_true", help="skip all serial-bus / register steps")
    return p.parse_args(argv)


# =============================================================================
# Menu actions -- each is self-contained so it can run on its own or as part of
# the guided flow. Config-editing actions need the YAML round-tripper; bus
# actions need a configured machine.yaml.
# =============================================================================
def _require_backend():
    """True if the comment-preserving round-tripper is active; else explain and False."""
    if _RUAMEL:
        return True
    print(
        "  ruyaml is not available, so the config files can't be read or written.\n"
        f"  run the dependencies option first, or: {sys.executable} -m pip install ruyaml"
    )
    return False


def do_dependencies(args):
    section("Python dependencies")
    if args.no_deps:
        print("  --no-deps given; skipping dependency check")
        return
    install_dependencies()


def do_machine(config_dir, scan_max):
    section("machine.yaml  -- hardware, channel map & masks")
    if not _require_backend():
        return
    machine, machine_path = prepare_config("machine", config_dir)
    if machine is not None:
        edit_machine(machine, scan_max)
        write_config(machine, machine_path)


def do_calibration(config_dir):
    section("calibration.yaml  -- optics & optimizer tuning")
    if not _require_backend():
        return
    calib, calib_path = prepare_config("calibration", config_dir)
    if calib is not None:
        if confirm(
            "walk through calibration.yaml now? (most values can keep their template\n"
            "defaults and be tuned later during calibration)",
            default=True,
        ):
            edit_mapping(calib, gate=True)
        write_config(calib, calib_path)


def do_bus(config_dir, scan_max, args):
    section("servo bus  -- ids, multi-turn (Reg 18) & angle limits (Reg 9/11)")
    if args.no_bus:
        print("  --no-bus given; skipping bus / register setup")
        return
    if not _require_backend():
        return
    devices, baud, channels = _bus_params(None, config_dir / "machine.yaml")
    if not devices:
        print("  no serial devices configured -- set up machine.yaml first; skipping")
        return
    register_setup(devices, baud, channels, scan_max)


def do_scan_bus(config_dir, scan_max):
    section("scan the bus  -- list connected servos")
    if not _require_backend():
        return
    devices, baud, _ = _bus_params(None, config_dir / "machine.yaml")
    if not devices:
        print("  no serial devices configured -- set up machine.yaml first; skipping")
        return
    found = scan_for_channels(devices, baud, scan_max)
    if found:
        print(f"  servos on the bus (id order): {found}")
    elif found is not None:
        print("  no servos found on the bus")


def do_identify(config_dir, scan_max):
    section("identify a servo  -- jog the selected motor so you can spot it")
    if not _require_backend():
        return
    devices, baud, channels = _bus_params(None, config_dir / "machine.yaml")
    if not devices:
        print("  no serial devices configured -- set up machine.yaml first; skipping")
        return
    bus = Bus.open(devices, baud)
    if bus is None:
        return
    try:
        found = bus.scan(range(1, scan_max + 1))
        if not found:
            print("  no servos found on the bus")
            return
        names = {int(c["id"]): str(c["name"]) for c in channels}
        print("  servos on the bus:")
        for sid in found:
            label = f"  ({names[sid]})" if sid in names else ""
            print(f"    id {sid}{label}")
        while True:
            sid = ask_int("which id to jog? (0 to stop)", found[0])
            if sid == 0:
                break
            if sid not in found and not confirm(
                f"  id {sid} wasn't detected; try anyway?", default=False
            ):
                continue
            print(f"  jogging id {sid} -- watch which motor moves ...")
            bus.identify(sid)
            if not confirm("  identify another?", default=True):
                break
    finally:
        bus.close()


def guided_setup(args, config_dir, scan_max):
    """The original end-to-end flow: dependencies -> machine -> calibration -> bus."""
    do_dependencies(args)
    if not _RUAMEL:
        print(
            "\n  ruyaml is not available, so the config files can't be created/edited.\n"
            f"  install it and re-run:  {sys.executable} -m pip install ruyaml"
        )
        return
    do_machine(config_dir, scan_max)
    do_calibration(config_dir)
    do_bus(config_dir, scan_max, args)
    print(
        "\nGuided setup done. Next steps:\n"
        "  - sanity-check config/machine.yaml and config/calibration.yaml\n"
        "  - run `python servo_server.py set_zero` (or `home`) to set the servo zeros\n"
        "  - see doc/motor.md for the register details and CLAUDE.md for the workflow"
    )


# =============================================================================
# Curses TUI (a menuconfig/raspi-config-style keyboard-driven page)
#
# Navigation: up/down (or k/j) move, typing a row's "[#]" number jumps to it,
# left/right change a toggle, Enter selects, q/ESC (or the "Back"/"Exit" row)
# backs out. Every screen -- main menu, servo
# pages, config-file editors, dependencies -- is built from the same item
# language (_it_action / _it_run / _it_toggle) and drawn by _tui_page. The bus
# helpers and config writers print to stdout, which would corrupt the screen, so
# those calls run under _quiet(); only streaming pip output drops out of curses
# (_with_suspend) and comes back.
# =============================================================================
def _tui_available():
    # tui.tui_available() checks curses + a real terminal; allow forcing the
    # plain text menu with SERVO_INIT_NOTUI=1.
    return tui_available() and not os.environ.get("SERVO_INIT_NOTUI")


def servo_config_tui(stdscr, config_dir, scan_max):
    """Scan the bus, list servos, edit each, and persist the channel map back to
    machine.yaml (id / name changes are written to servo.channels)."""
    if not _RUAMEL:
        _tui_message(stdscr, ["ruyaml is not available, so machine.yaml can't be read/written.",
                              "Run the 'Python dependencies' option first."])
        return
    machine_path = config_dir / "machine.yaml"
    if not machine_path.exists():
        _tui_message(stdscr, ["machine.yaml not found.",
                              "Create it with the 'machine.yaml' option first."])
        return
    with _quiet():
        cfg = _load_text(machine_path.read_text())
    try:
        devices = [str(d) for d in cfg["serial"]["devices"]]
        baud = int(cfg["serial"]["baudrate"])
        channels = cfg["servo"]["channels"]
    except (KeyError, TypeError):
        _tui_message(stdscr, ["machine.yaml is missing its serial / servo sections."])
        return
    if not devices:
        _tui_message(stdscr, ["No serial devices configured in machine.yaml."])
        return
    sink = io.StringIO()
    with contextlib.redirect_stdout(sink):
        bus = Bus.open(devices, baud)
    if bus is None:
        _tui_message(stdscr, ["Could not open the serial bus:",
                              sink.getvalue().strip() or "(no device opened)"])
        return

    dirty = [False]
    def mark_dirty():
        dirty[0] = True

    # Cache the scan so navigating the list doesn't re-ping the bus on every key;
    # refresh only on entry, on "Rescan", and after editing a servo.
    cache = {"found": [], "phase": {}}

    def rescan():
        _flash_status(stdscr, "scanning the bus ...")
        with _quiet():
            cache["found"] = bus.scan(range(1, scan_max + 1))
            cache["phase"] = {sid: bus.read1(sid, ADDR_PHASE) for sid in cache["found"]}
        _status(f"found {len(cache['found'])} servo(s): {cache['found'] or 'none'}")

    def build():
        items = [_it_action("Rescan bus", "rescan"),
                 _it_action("Add a new servo  (park existing at +100, add one, restore)", "add")]
        for sid in cache["found"]:
            nm = _channel_name(channels, sid)
            phase = {PHASE_SINGLE_TURN: "single-turn",
                     PHASE_MULTI_TURN: "multi-turn"}.get(cache["phase"].get(sid), "?")
            tag = f"  ({nm})" if nm else "  (unmapped)"
            items.append(_it_action(f"id {sid}{tag}    [{phase}]", ("servo", sid)))
        if not cache["found"]:
            items.append(_it_action("(no servos detected -- check wiring/power, then Rescan)", "rescan"))
        items.append(_it_action("Back", None))
        return items

    def subtitle():
        return f"bus: {bus.device} @ {baud}    {len(cache['found'])} servo(s) found"

    rescan()
    try:
        while True:
            choice = _tui_page(stdscr, "Servo configuration", build, subtitle=subtitle,
                               footer="up/down move - [#] jump - Enter open - q back")
            if choice is None:
                break
            if choice == "rescan":
                rescan()
            elif choice == "add":
                _add_servo_tui(stdscr, bus, channels, scan_max, mark_dirty)
                rescan()   # reflect the new servo (and restored ids) in the list
            elif isinstance(choice, tuple) and choice[0] == "servo":
                _servo_submenu(stdscr, bus, choice[1], channels, mark_dirty)
                rescan()   # reflect any id / feedback change back in the list
    finally:
        with _quiet():
            bus.close()

    if dirty[0]:
        with _quiet():
            write_config(cfg, machine_path)
        _status(f"saved channel map to {machine_path.name}")


_PHASE_FMT = {PHASE_SINGLE_TURN: "108 (single-turn)", PHASE_MULTI_TURN: "124 (multi-turn)"}

# A new FEETECH servo ships as id 1, which collides with the servos already on
# the bus. To add one unambiguously we temporarily park every existing servo out
# of the low id range by this offset, so the freshly connected servo is the only
# thing responding down low.
_ID_BUMP = 100


def _smallest_free_id(taken, hi):
    """Smallest id in 1..hi-1 that is not in ``taken`` (a set); None if all taken."""
    for i in range(1, hi):
        if i not in taken:
            return i
    return None


def _add_servo_tui(stdscr, bus, channels, scan_max, mark_dirty):
    """Guided 'add a new servo' flow.

    A new servo ships as id 1 and can't be told apart from an existing id-1
    servo. So: park every existing servo out of the way (id += _ID_BUMP), have
    the user connect the new servo, scan the now-empty low range to find it, give
    it a final id + name, then move the parked servos back down. The park is
    always undone in a ``finally`` so a cancel/error can't strand the existing
    servos at 100+; the new servo is forced onto a free id (never one of the
    parked-down ids) so restoring can't collide.
    """
    with _quiet():
        existing = sorted(bus.scan(range(1, scan_max + 1)))
        # Exact target ids we'd park onto must be free, or the park would collide.
        conflicts = [e + _ID_BUMP for e in existing if bus.ping(e + _ID_BUMP)]
    if conflicts:
        _tui_message(stdscr, [f"Parking targets already in use: {conflicts}.",
                              "Resolve those ids before adding a servo."])
        return
    bad = [e for e in existing if e + _ID_BUMP > 252]
    if bad:
        _tui_message(stdscr, [f"Can't park ids {bad} (+{_ID_BUMP} exceeds the 252 id limit).",
                              "Reassign them lower first."])
        return

    if existing and not _tui_confirm(
        stdscr,
        f"Add a servo? Parks {len(existing)} servo(s) at +{_ID_BUMP}, then restores them.",
    ):
        return

    parked = []      # original ids successfully parked at id + _ID_BUMP
    added = None     # final id of the newly configured servo, once assigned
    try:
        # 1. Park existing servos out of the low range.
        for e in existing:
            _flash_status(stdscr, f"parking id {e} -> {e + _ID_BUMP} ...")
            with _quiet():
                ok = (bus.write_eeprom1(e, ADDR_ID, e + _ID_BUMP, lock_id=e + _ID_BUMP)
                      and bus.ping(e + _ID_BUMP))
            if not ok:
                _tui_message(stdscr, [f"Failed to park id {e} -> {e + _ID_BUMP}.",
                                      "Aborting; parked servos will be restored."])
                return
            parked.append(e)

        # 2. Wait for the user to physically connect the new servo.
        lines = []
        if parked:
            lines += [f"Existing servos parked at ids {[e + _ID_BUMP for e in parked]}.", ""]
        lines += ["Connect the NEW servo now (it ships as id 1) and power it.",
                  "Then press any key to scan for it."]
        _tui_message(stdscr, lines)

        # 3. Scan the (now empty) low range for the newcomer.
        _flash_status(stdscr, "scanning for the new servo ...")
        with _quiet():
            newfound = bus.scan(range(1, _ID_BUMP))
        if not newfound:
            _tui_message(stdscr, ["No new servo found in the low id range.",
                                  "Check wiring/power. Parked servos will be restored."])
            return
        if len(newfound) > 1:
            _tui_message(stdscr, [f"More than one servo in the low range: {newfound}.",
                                  "Connect only ONE new servo at a time.",
                                  "Parked servos will be restored; try again."])
            return
        newcur = newfound[0]

        # 4. Give the new servo a final id (never a parked-down id, so restoring
        #    can't collide) and a channel-map name.
        default_id = _smallest_free_id(set(existing), _ID_BUMP) or newcur
        while True:
            new_id = _tui_input_int(stdscr, f"final id for the new servo (found at {newcur})",
                                    default_id)
            if not (1 <= new_id < _ID_BUMP):
                _tui_message(stdscr, [f"id must be in 1..{_ID_BUMP - 1}."])
                continue
            if new_id in existing:
                _tui_message(stdscr, [f"id {new_id} is a parked servo's id -- pick a free one.",
                                      f"suggested free id: {default_id}"])
                continue
            break
        if new_id != newcur:
            with _quiet():
                ok = bus.write_eeprom1(newcur, ADDR_ID, new_id, lock_id=new_id) and bus.ping(new_id)
            if not ok:
                _tui_message(stdscr, [f"Failed to set the new servo id {newcur} -> {new_id}.",
                                      "Parked servos will be restored."])
                return
        name = _tui_input(stdscr, f"name for servo id {new_id}", _channel_name(channels, new_id))
        _set_channel_name(channels, new_id, name)
        mark_dirty()
        added = new_id
        _status(f"added servo id {new_id}" + (f" ({name})" if name else ""))
    finally:
        # 5. Always move the parked servos back down to their original ids.
        for e in parked:
            _flash_status(stdscr, f"restoring id {e + _ID_BUMP} -> {e} ...")
            with _quiet():
                ok = (bus.write_eeprom1(e + _ID_BUMP, ADDR_ID, e, lock_id=e)
                      and bus.ping(e))
            if not ok:
                _status(f"WARNING: could not restore id {e + _ID_BUMP} -> {e} -- fix it manually")

    # 6. With every servo back in place, open the new one for full configuration
    #    (name / feedback / angle limits / jog).
    if added is not None:
        _servo_submenu(stdscr, bus, added, channels, mark_dirty)


def _servo_submenu(stdscr, bus, sid, channels, mark_dirty):
    """Per-servo page built from the shared item language. Bus ID and the
    Register-18 feedback mode are inline edit fields (Enter to edit, arrows/typing
    to change, Enter to confirm); name is a prompt, jog is an action. Editing
    id updates the channel map (mark_dirty). Every action reports on the status line."""
    state = {"cur": sid}

    def set_id(new):
        cur = state["cur"]
        if new == cur:
            _status("id unchanged")
            return
        with _quiet():
            ok = bus.write_eeprom1(cur, ADDR_ID, new, lock_id=new) and bus.ping(new)
        if ok:
            _set_channel_id(channels, cur, new)
            mark_dirty()
            state["cur"] = new
            _status(f"id {cur} -> {new}  (changed; saves on exit)")
        else:
            _status(f"id {cur} -> {new} FAILED (check wiring/power)")

    def set_phase(value):
        cur = state["cur"]
        with _quiet():
            bus.write_eeprom1(cur, ADDR_PHASE, value)
            chk = bus.read1(cur, ADDR_PHASE)
        mode = {PHASE_MULTI_TURN: "multi-turn", PHASE_SINGLE_TURN: "single-turn"}.get(chk, "?")
        _status(f"id {cur} Register 18 -> {chk} ({mode})" + ("" if chk == value else "  WRITE FAILED"))

    def change_name():
        cur = state["cur"]
        name = _channel_name(channels, cur)
        new = _tui_input(stdscr, "name", name)
        if new == name:
            _status("name unchanged")
        else:
            _set_channel_name(channels, cur, new)
            mark_dirty()
            _status(f"id {cur} name -> '{new}'  (saves on exit)")

    def set_limit(addr, name):
        def f(new):
            cur = state["cur"]
            with _quiet():
                bus.write_eeprom2(cur, addr, new)
                chk = bus.read2(cur, addr)
            _status(f"id {cur} {name} angle limit -> {chk}" + ("" if chk == new else "  WRITE FAILED"))
        return f

    def jog():
        cur = state["cur"]
        _flash_status(stdscr, f"jogging id {cur} -- watch which motor moves ...")
        with _quiet():
            bus.identify(cur)
        _status(f"id {cur}: done jogging")

    def build():
        cur = state["cur"]
        name = _channel_name(channels, cur)
        with _quiet():
            ph = bus.read1(cur, ADDR_PHASE)
            lo = bus.read2(cur, ADDR_MIN_ANGLE)
            hi = bus.read2(cur, ADDR_MAX_ANGLE)
        return [
            _it_edit("Bus ID    : ", cur, set_id, step=1, lo=1, hi=252),
            _it_run(f"Name      : {name or '(unnamed)'}", change_name),
            _it_edit("Feedback  : ", ph if ph in _PHASE_FMT else PHASE_SINGLE_TURN, set_phase,
                     options=[PHASE_SINGLE_TURN, PHASE_MULTI_TURN], fmt=lambda v: _PHASE_FMT.get(v, str(v))),
            _it_edit("Min limit : ", lo if lo is not None else 0, set_limit(ADDR_MIN_ANGLE, "min"),
                     step=1, lo=ANGLE_LIMIT_LO, hi=ANGLE_LIMIT_HI),
            _it_edit("Max limit : ", hi if hi is not None else 0, set_limit(ADDR_MAX_ANGLE, "max"),
                     step=1, lo=ANGLE_LIMIT_LO, hi=ANGLE_LIMIT_HI),
            _it_run("Identify (jog this servo)", jog),
            _it_action("Back", "back"),
        ]

    def title():
        cur = state["cur"]
        name = _channel_name(channels, cur)
        return f"Servo id {cur}" + (f"  ({name})" if name else "")

    _tui_page(stdscr, title, build)


def _with_suspend(stdscr, fn):
    """Run fn() with curses paused -- for streaming subprocesses like pip."""
    curses.def_prog_mode()
    curses.endwin()
    try:
        fn()
        try:
            input("\n[ press Enter to return to the menu ] ")
        except EOFError:
            pass
    finally:
        curses.reset_prog_mode()
        _init_theme(stdscr)
        stdscr.clear()
        stdscr.refresh()


# --- Generic curses editor for a loaded YAML tree (same item language) -------
def _coerce_like(old, raw):
    """Parse raw text to old's type, keeping ruamel hex/quote styling where easy."""
    if isinstance(old, bool):
        return raw.strip().lower() in ("y", "yes", "true", "1")
    if isinstance(old, int):  # bool already handled above
        val = int(raw, 0)
        return _hexint(val) if (_RUAMEL and HexInt is not None and isinstance(old, HexInt)) else val
    if isinstance(old, float):
        return float(raw)
    if _RUAMEL and SingleQuotedScalarString is not None and isinstance(old, SingleQuotedScalarString):
        return SingleQuotedScalarString(raw)
    return raw


def _is_num(x):
    return not isinstance(x, bool) and isinstance(x, (int, float))


def _edit_mapping_tui(stdscr, mapping, title, mark_dirty):
    """Edit a dict in place: scalars prompt, bools toggle, dicts/lists recurse.
    Every actual change calls mark_dirty() so the caller can offer save/discard."""
    def edit_scalar(k):  # strings / None: bottom-line prompt
        def f():
            old = mapping[k]
            raw = _tui_input(stdscr, str(k), "" if old is None else old)
            try:
                new = _coerce_like(old, raw)
                if new != old:
                    mapping[k] = new
                    mark_dirty()
                _status(f"{k} = {mapping[k]}")
            except ValueError:
                _status(f"{k}: '{raw}' is not a valid {type(old).__name__}")
        return f

    def set_number(k):  # numbers: inline edit-mode (arrows / typing)
        def f(v):
            old = mapping[k]
            try:
                new = _coerce_like(old, str(v))   # keep hex/int/float ruamel styling
            except ValueError:
                _status(f"{k}: invalid value")
                return
            if new != old:
                mapping[k] = new
                mark_dirty()
            _status(f"{k} = {mapping[k]}")
        return f

    def toggle_bool(k):
        def f(_d):
            mapping[k] = not bool(mapping[k])
            mark_dirty()
            _status(f"{k} = {mapping[k]}")
        return f

    def edit_numlist(k):
        def f():
            old = mapping[k]
            raw = _tui_input(stdscr, f"{k} ({len(old)} numbers, space/comma sep)", _fmt_nums(old))
            try:
                old[:] = _parse_nums(raw, isinstance(old[0], int))   # keep the list object + flow style
                mark_dirty()
                _status(f"{k} = [{_fmt_nums(old)}]")
            except (ValueError, IndexError):
                _status(f"{k}: couldn't parse numbers")
        return f

    def edit_strlist(k):
        def f():
            old = mapping[k]
            raw = _tui_input(stdscr, f"{k} (comma separated)", ", ".join(str(x) for x in old))
            old[:] = [s.strip() for s in raw.split(",") if s.strip()]
            mark_dirty()
            _status(f"{k}: {len(old)} item(s)")
        return f

    def dive_map(k):
        return lambda: _edit_mapping_tui(stdscr, mapping[k], f"{title} / {k}", mark_dirty)

    def dive_entries(k):
        return lambda: _edit_seq_tui(stdscr, mapping[k], f"{title} / {k}", mark_dirty)

    def dive_rows(k):
        return lambda: _edit_rows_tui(stdscr, mapping[k], f"{title} / {k}", mark_dirty)

    def build():
        items = []
        for k in list(mapping.keys()):
            v = mapping[k]
            kl = f"{str(k):<16}"
            if isinstance(v, bool):
                items.append(_it_toggle(f"{kl}: < {v} >", toggle_bool(k)))
            elif _is_num(v):
                is_hex = _RUAMEL and HexInt is not None and isinstance(v, HexInt)
                items.append(_it_edit(
                    f"{kl}: ", v, set_number(k),
                    step=1 if isinstance(v, int) else 1.0,
                    cast=_to_int if isinstance(v, int) else float,
                    fmt=hex if is_hex else None))
            elif isinstance(v, str) or v is None:
                items.append(_it_run(f"{kl}: {v}", edit_scalar(k)))
            elif isinstance(v, dict):
                items.append(_it_run(f"{kl}: ... ({len(v)} keys)", dive_map(k)))
            elif isinstance(v, list):
                if v and all(_is_num(x) for x in v):
                    items.append(_it_run(f"{kl}: [{_fmt_nums(v)}]", edit_numlist(k)))
                elif v and all(isinstance(x, str) for x in v):
                    items.append(_it_run(f"{kl}: [{', '.join(map(str, v))}]", edit_strlist(k)))
                elif v and all(isinstance(x, dict) for x in v):
                    items.append(_it_run(f"{kl}: ({len(v)} entries)", dive_entries(k)))
                elif v and all(isinstance(x, list) for x in v):
                    items.append(_it_run(f"{kl}: ({len(v)} rows)", dive_rows(k)))
                else:
                    items.append(_it_run(f"{kl}: ({len(v)} items)",
                                         lambda kk=k: _status(f"{kk}: not editable here")))
            else:
                items.append(_it_run(f"{kl}: (unsupported)", lambda kk=k: _status(f"{kk}: not editable")))
        items.append(_it_action("Back", None))
        return items

    _tui_page(stdscr, title, build, footer="up/down move - [#] jump - Enter edit - q back")


def _edit_seq_tui(stdscr, seq, title, mark_dirty):
    """Edit a list of mappings (e.g. servo.channels): pick an entry to edit."""
    def build():
        items = []
        for i, entry in enumerate(seq):
            summary = ", ".join(f"{kk}={vv}" for kk, vv in entry.items())
            items.append(_it_run(f"[{i}] {summary}",
                                  (lambda j: lambda: _edit_mapping_tui(
                                      stdscr, seq[j], f"{title}[{j}]", mark_dirty))(i)))
        items.append(_it_action("Back", None))
        return items

    _tui_page(stdscr, title, build, footer="up/down move - [#] jump - Enter edit - q back")


def _edit_rows_tui(stdscr, seq, title, mark_dirty):
    """Edit a list of numeric rows (e.g. coupling vectors)."""
    def edit_row(i):
        def f():
            row = seq[i]
            raw = _tui_input(stdscr, f"row {i} ({len(row)} numbers)", _fmt_nums(row))
            try:
                row[:] = _parse_nums(raw, all(isinstance(x, int) for x in row))
                mark_dirty()
                _status(f"row {i} = [{_fmt_nums(row)}]")
            except ValueError:
                _status(f"row {i}: couldn't parse numbers")
        return f

    def build():
        items = [_it_run(f"[{i}] [{_fmt_nums(row)}]", edit_row(i)) for i, row in enumerate(seq)]
        items.append(_it_action("Back", None))
        return items

    _tui_page(stdscr, title, build, footer="up/down move - [#] jump - Enter edit - q back")


def _prepare_config_tui(stdscr, name, config_dir):
    """Load name.yaml for editing in curses (creating it from the template).
    Returns (cfg, target, overwrote) -- overwrote is True when the user chose to
    start from the template, so the caller treats the buffer as already-changed."""
    template = config_dir / f"{name}.template.yaml"
    target = config_dir / f"{name}.yaml"
    if not template.exists():
        _tui_message(stdscr, [f"template {template.name} not found; cannot create {target.name}."])
        return None, target, False
    src, overwrote = template, False
    if target.exists():
        choice = _tui_page(stdscr, f"{target.name} already exists", lambda: [
            _it_action("Edit the existing file", "edit"),
            _it_action("Overwrite from the template", "overwrite"),
            _it_action("Leave it unchanged (cancel)", None),
        ], footer="up/down move - [#] jump - Enter select - q cancel")
        if choice is None:
            return None, target, False
        src = template if choice == "overwrite" else target
        overwrote = choice == "overwrite"
    with _quiet():
        return _load_text(src.read_text()), target, overwrote


def _edit_config_tui(stdscr, name, config_dir, title):
    if not _RUAMEL:
        _tui_message(stdscr, ["ruyaml is required to edit the config files.",
                              "Use 'Python dependencies' to install it first."])
        return
    cfg, target, overwrote = _prepare_config_tui(stdscr, name, config_dir)
    if cfg is None:
        return
    dirty = [overwrote]
    _edit_mapping_tui(stdscr, cfg, title, lambda: dirty.__setitem__(0, True))
    if not dirty[0]:
        _status(f"{target.name}: no changes made")
        return
    # Offer an explicit way to leave without changing the file on disk.
    choice = _tui_page(stdscr, f"Save changes to {target.name}?", lambda: [
        _it_action("Save changes", "save"),
        _it_action("Discard changes (leave the file unchanged)", "discard"),
    ], footer="up/down move - [#] jump - Enter select")
    if choice == "save":
        with _quiet():
            write_config(cfg, target)
        _status(f"saved {target.name}")
    else:
        _status(f"{target.name}: changes discarded -- file left unchanged")


def deps_tui(stdscr):
    req = _find_requirements()
    if req is None:
        _tui_message(stdscr, ["requirements.txt not found next to the project."])
        return
    pkgs = _parse_requirements(req)

    def pip_install(full):
        missing = _missing_packages(pkgs)
        if not full and not missing:
            _status("nothing to install -- all present")
            return
        cmd = [sys.executable, "-m", "pip", "install"] + (["-r", str(req)] if full else list(missing))

        def run():
            print(f"running: {' '.join(cmd)}\n")
            try:
                rc = subprocess.call(cmd)
            except Exception as e:  # pragma: no cover
                print(f"pip could not run: {e}")
                rc = 1
            importlib.invalidate_caches()
            _resolve_yaml_backend()   # pick up a freshly-installed ruyaml
            print("\npip finished" if rc == 0 else "\npip finished WITH ERRORS")

        _with_suspend(stdscr, run)
        still = _missing_packages(pkgs)
        _status("all dependencies satisfied" if not still else f"still missing: {', '.join(still)}")

    def build():
        missing = set(_missing_packages(pkgs))
        items = [_it_run(f"{dist:<18} [{'MISSING' if dist in missing else 'ok'}]",
                         lambda: _status("(status only -- use the install actions below)"))
                 for dist, _imp in pkgs]
        if missing:
            items.append(_it_run(f"Install the {len(missing)} missing package(s)", lambda: pip_install(False)))
        items.append(_it_run("Install everything from requirements.txt", lambda: pip_install(True)))
        items.append(_it_action("Back", None))
        return items

    _tui_page(stdscr, "Python dependencies", build,
              subtitle=f"requirements: {req.name}", footer="up/down move - [#] jump - Enter select - q back")


def guided_tui(stdscr, config_dir, scan_max):
    deps_tui(stdscr)
    if not _RUAMEL:
        _tui_message(stdscr, ["ruyaml is still missing -- install it, then edit the configs."])
        return
    _edit_config_tui(stdscr, "machine", config_dir, "machine.yaml")
    _edit_config_tui(stdscr, "calibration", config_dir, "calibration.yaml")
    servo_config_tui(stdscr, config_dir, scan_max)
    _status("guided setup complete")


def run_tui(config_dir, scan_max):
    def app(stdscr):
        curses.curs_set(0)
        _init_theme(stdscr)

        def build():
            return [
                _it_action("Full guided setup  (dependencies, machine, calibration, bus)", "guided"),
                _it_action("Python dependencies", "deps"),
                _it_action("machine.yaml  (hardware, channel map, masks)", "machine"),
                _it_action("calibration.yaml  (optics / optimizer tuning)", "calib"),
                _it_action("Servo configuration  (scan bus, IDs, names, feedback, jog)", "servo"),
                _it_action("Exit", "exit"),
            ]

        dispatch = {
            "guided": lambda: guided_tui(stdscr, config_dir, scan_max),
            "deps": lambda: deps_tui(stdscr),
            "machine": lambda: _edit_config_tui(stdscr, "machine", config_dir, "machine.yaml"),
            "calib": lambda: _edit_config_tui(stdscr, "calibration", config_dir, "calibration.yaml"),
            "servo": lambda: servo_config_tui(stdscr, config_dir, scan_max),
        }
        while True:
            choice = _tui_page(stdscr, "Servo-aligner setup", build,
                               subtitle=f"config: {config_dir}",
                               footer="up/down move - [#] jump - Enter select - q quit")
            if choice in (None, "exit"):
                break
            dispatch[choice]()

    curses.wrapper(app)


# =============================================================================
# Entry point
# =============================================================================
def run_menu(args, config_dir, scan_max):
    options = [
        ("Full guided setup (dependencies -> machine -> calibration -> bus)",
         lambda: guided_setup(args, config_dir, scan_max)),
        ("Check / install Python dependencies",
         lambda: do_dependencies(args)),
        ("Create or edit machine.yaml (hardware, channel map, masks)",
         lambda: do_machine(config_dir, scan_max)),
        ("Create or edit calibration.yaml (optics / optimizer tuning)",
         lambda: do_calibration(config_dir)),
        ("Servo bus: assign IDs, multi-turn (Reg 18) & angle limits (Reg 9/11)",
         lambda: do_bus(config_dir, scan_max, args)),
        ("Scan the bus (list connected servo IDs)",
         lambda: do_scan_bus(config_dir, scan_max)),
        ("Identify a servo (jog the selected motor to spot it)",
         lambda: do_identify(config_dir, scan_max)),
    ]
    while True:
        print("\nWhat would you like to do?")
        for i, (label, _) in enumerate(options, 1):
            print(f"  {i}) {label}")
        print("  q) Quit")
        try:
            raw = input("select [q]: ").strip().lower()
        except EOFError:
            break
        if raw in ("", "q", "quit", "exit"):
            break
        try:
            idx = int(raw)
        except ValueError:
            print(f"  enter a number 1-{len(options)}, or q to quit")
            continue
        if 1 <= idx <= len(options):
            options[idx - 1][1]()
        else:
            print(f"  out of range (1-{len(options)})")


def main(argv=None):
    args = parse_args(argv)
    scan_max = 252 if args.full_scan else args.scan_max
    config_dir = find_config_dir(args.config_dir)

    if not INTERACTIVE:
        for name in ("machine", "calibration"):
            template = config_dir / f"{name}.template.yaml"
            target = config_dir / f"{name}.yaml"
            if template.exists() and not target.exists():
                shutil.copy2(template, target)
                print(f"created {target} from template")
        print("non-interactive shell: copied templates only; edit the files by hand.")
        return

    # Preferred UI: a keyboard-driven curses page. Fall back to the plain text
    # menu when curses isn't available or the terminal can't support it (set
    # SERVO_INIT_NOTUI=1 to force the text menu).
    if _tui_available():
        run_tui(config_dir, scan_max)
        return

    print(
        "Servo-aligner setup\n"
        "-------------------\n"
        "Pick an action below: install dependencies, create/edit the config files,\n"
        "or set up the servo bus. 'Full guided setup' runs them all in order."
    )
    print(f"config directory: {config_dir}")
    run_menu(args, config_dir, scan_max)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\naborted.")
        sys.exit(130)
