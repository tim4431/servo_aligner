#!/usr/bin/env python
"""First-run setup wizard for the servo aligner.

For someone setting up a new machine from scratch, this does four things:

1. Installs the project's Python dependencies (the packages in
   ``requirements.txt``), offering to ``pip install`` any that are missing.
2. Creates the two gitignored config files from the checked-in templates --
   ``config/machine.yaml`` and ``config/calibration.yaml`` -- and walks you
   through editing the values interactively (the template comments are preserved
   by round-tripping through ``ruyaml``, which is required for this step).
3. Connects to the serial bus and scans for servos so you can build the
   ``servo.channels`` map from what is actually wired up.
4. Performs the servo register steps that doc/motor.md otherwise asks you to do
   by hand in FEETECH's FD tool:
     * assign a unique bus **ID** to a servo (factory default is 1), and
     * enable hardware multi-turn feedback by setting **Register 18 -> 124**.

Run it from ``src/`` before anything else:

    python init_helper.py

or, in production (installed under expctl):

    python -m expctl.servers.servoaligner.init_helper

This module talks to the bus directly via ``scservo_sdk``; it deliberately does
NOT import ``servodriver``/``config``, so it is safe to run before the YAML
files exist. It never enables torque and never commands a position, so no servo
moves -- the only writes are to the ID / phase / EEPROM-lock registers.
"""

import argparse
import glob
import importlib
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

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
ADDR_PHASE = 18     # phase setting / feedback mode (EEPROM)
PHASE_SINGLE_TURN = 108  # factory default (single-turn feedback)
PHASE_MULTI_TURN = 124   # 108 + BIT4 -> report full multi-turn angle

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


def ask_int(label, current):
    while True:
        raw = _prompt(label, current)
        if raw == "":
            return current
        try:
            return int(raw, 0)  # base 0 also accepts 0x.. / 0b..
        except ValueError:
            print("    not an integer, try again")


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


def _build_channels(rows):
    """rows = [(id, name), ...] -> block list of inline {id: , name: } maps."""
    out = CommentedSeq() if _RUAMEL else []
    for sid, name in rows:
        if _RUAMEL:
            m = CommentedMap()
            m["id"] = int(sid)
            m["name"] = SingleQuotedScalarString(str(name))
            m.fa.set_flow_style()
            out.append(m)
        else:
            out.append({"id": int(sid), "name": str(name)})
    return out


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
                return cls(port, sms_sts(port), COMM_SUCCESS)
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

    def write_eeprom1(self, sid, addr, value, lock_id=None):
        """Unlock EEPROM, write one byte, re-lock. ``lock_id`` defaults to ``sid``
        but must be the *new* id when the write changes the servo's own id."""
        self.ph.unLockEprom(sid)
        res, _ = self.ph.write1ByteTxRx(sid, addr, value)
        self.ph.LockEprom(sid if lock_id is None else lock_id)
        return res == self.OK

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

        if confirm(
            "enable hardware multi-turn (Register 18 -> 124) on the servos?",
            default=True,
        ):
            targets = found or [int(c["id"]) for c in channels]
            setup_multiturn(bus, targets)
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
# Entry point
# =============================================================================
def parse_args(argv=None):
    p = argparse.ArgumentParser(description="First-run setup wizard for the servo aligner.")
    p.add_argument("--config-dir", help="directory holding the *.template.yaml files")
    p.add_argument("--scan-max", type=int, default=20, help="highest servo id to scan (default 20)")
    p.add_argument("--full-scan", action="store_true", help="scan the full id range 1..252")
    p.add_argument("--no-deps", action="store_true", help="skip the dependency check / install step")
    p.add_argument("--no-bus", action="store_true", help="skip all serial-bus / register steps")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    scan_max = 252 if args.full_scan else args.scan_max

    print(
        "Servo-aligner setup wizard\n"
        "--------------------------\n"
        "Installs dependencies, creates config/machine.yaml + config/calibration.yaml\n"
        "from the templates, then helps connect to the servos and set up their IDs and\n"
        "multi-turn mode."
    )
    config_dir = find_config_dir(args.config_dir)
    print(f"config directory: {config_dir}")

    if not INTERACTIVE:
        for name in ("machine", "calibration"):
            template = config_dir / f"{name}.template.yaml"
            target = config_dir / f"{name}.yaml"
            if template.exists() and not target.exists():
                shutil.copy2(template, target)
                print(f"created {target} from template")
        print("non-interactive shell: copied templates only; edit the files by hand.")
        return

    # 1) Python dependencies --------------------------------------------------
    section("1/4  Python dependencies")
    if args.no_deps:
        print("  --no-deps given; skipping dependency check")
    else:
        install_dependencies()
    if not _RUAMEL:
        print(
            "\nERROR: ruyaml is not available, so the config files cannot be created\n"
            "or edited without dropping the template comments. Install it and re-run:\n"
            f"    {sys.executable} -m pip install ruyaml"
        )
        return

    # 2) machine.yaml ---------------------------------------------------------
    section("2/4  machine.yaml  -- hardware & connection")
    machine, machine_path = prepare_config("machine", config_dir)
    if machine is not None:
        edit_machine(machine, scan_max)
        write_config(machine, machine_path)

    # 3) calibration.yaml -----------------------------------------------------
    section("3/4  calibration.yaml  -- optics & optimizer tuning")
    calib, calib_path = prepare_config("calibration", config_dir)
    if calib is not None:
        if confirm(
            "walk through calibration.yaml now? (most values can keep their template\n"
            "defaults and be tuned later during calibration)",
            default=True,
        ):
            edit_mapping(calib, gate=True)
        write_config(calib, calib_path)

    # 4) servo bus: ids + multi-turn -----------------------------------------
    section("4/4  servo bus  -- ids & multi-turn (Register 18)")
    if args.no_bus:
        print("  --no-bus given; skipping bus / register setup")
    else:
        devices, baud, channels = _bus_params(machine, machine_path)
        if not devices:
            print("  no serial devices configured; skipping")
        elif confirm("do the servo register setup over the bus now?", default=True):
            register_setup(devices, baud, channels, scan_max)

    print(
        "\nDone. Next steps:\n"
        "  - sanity-check config/machine.yaml and config/calibration.yaml\n"
        "  - run `python STSServer.py set_zero` (or `home`) to set the servo zeros\n"
        "  - see doc/motor.md for the register details and CLAUDE.md for the workflow"
    )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\naborted.")
        sys.exit(130)
