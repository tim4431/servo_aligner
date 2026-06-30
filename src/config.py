"""Central configuration for the servo aligner.

Everything that differs between installations lives in two YAML files so that
moving to a new machine (or a new optical setup) means editing YAML, not code:

* ``machine.yaml``     - hardware / software settings tied to a particular
  Raspberry Pi: serial devices, baudrate, the servo channel map and the channel
  grouping masks, ADC wiring, filesystem paths and the ZMQ server identity.
* ``calibration.yaml`` - optics-setup / calibration settings: the beam-clip
  accept functions, Jacobian coupling vectors and the optimizer tuning
  parameters.

Both files are gitignored; copy the checked-in ``*.template.yaml`` files and
edit them when setting up a machine. The file locations default to this
directory and may be overridden with the ``SERVO_ALIGNER_MACHINE_CONFIG`` /
``SERVO_ALIGNER_CALIB_CONFIG`` environment variables.

This module is pure (no hardware imports) and safe to import anywhere. It does,
however, require PyYAML and the two YAML files to be present.
"""

import os
from pathlib import Path

try:
    import yaml
except ModuleNotFoundError as e:  # pragma: no cover - environment dependent
    raise ModuleNotFoundError(
        "PyYAML is required to load the servo-aligner configuration. "
        "Install it with `pip install pyyaml` (or `sudo apt install python3-yaml`)."
    ) from e

_HERE = Path(__file__).resolve().parent   # the src/ dir (or installed package dir)
_REPO_ROOT = _HERE.parent                 # repo root (parent of src/)

# Directories searched, in order, for the YAML config files; the first that
# contains a given file wins. Standalone runs use <repo>/config; production can
# point SERVO_ALIGNER_CONFIG_DIR at an absolute location, or drop the files next
# to this module.
_CONFIG_DIRS = [Path(d) for d in (
    os.environ.get("SERVO_ALIGNER_CONFIG_DIR"),
    _REPO_ROOT / "config",
    _HERE,
) if d]


def _load(filename, file_env):
    """Locate and parse a YAML config file.

    Resolution order: the per-file env override, then each entry of
    ``_CONFIG_DIRS``. Raises a helpful error if found nowhere.
    """
    override = os.environ.get(file_env)
    if override:
        path = Path(override)
    else:
        path = next((d / filename for d in _CONFIG_DIRS if (d / filename).exists()),
                    _CONFIG_DIRS[0] / filename)
    if not path.exists():
        template = path.with_name(filename.replace(".yaml", ".template.yaml"))
        searched = ", ".join(str(d) for d in _CONFIG_DIRS)
        raise FileNotFoundError(
            f"Config file '{filename}' not found (searched: {searched}). "
            f"Copy the template and edit it, e.g.:\n    cp {template} {path}"
        )
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _resolve_path(p):
    """Absolute paths are used as-is; relative paths resolve against the repo root."""
    p = Path(p)
    return p if p.is_absolute() else (_REPO_ROOT / p)


MACHINE = _load("machine.yaml", "SERVO_ALIGNER_MACHINE_CONFIG")
CALIB = _load("calibration.yaml", "SERVO_ALIGNER_CALIB_CONFIG")

# --- machine.yaml: serial bus -------------------------------------------------
DEVICENAME_LIST = list(MACHINE["serial"]["devices"])
BAUDRATE = int(MACHINE["serial"]["baudrate"])

# --- machine.yaml: servo defaults + channel layout ---------------------------
SERVO_SPEED = int(MACHINE["servo"]["speed"])
SERVO_ACC = int(MACHINE["servo"]["acc"])

_channels = MACHINE["servo"]["channels"]
# {channel_index: [servo_ID, "name"]} - list order defines the N-element vectors
# (masks, angle lists, ...) used throughout the code.
sts3032_dict = {i: [int(c["id"]), str(c["name"])] for i, c in enumerate(_channels)}
SERVO_CHANNEL_LIST = list(range(len(_channels)))
N_CHANNELS = len(_channels)

# Channel grouping masks: {mask_name: [0/1, ...]} over the channels above. Each
# mask selects which channel indices form a knob group (e.g.
# A_X_XDOT = [1,0,1,0,0,0,0,0]). Which servo drives which knob is wiring, so the
# masks live in machine.yaml alongside the channel map. Per-path masks follow the
# convention "<path>_<group>" (path "A"=upper, "B"=lower), e.g. "A_X_XDOT" or
# "B_POS_ALL"; "POS_ALL" spans both paths.
MASKS = {name: list(m) for name, m in MACHINE["servo"]["masks"].items()}


def knob_mask(path, group=None):
    """Channel mask for a knob group on a beam path, e.g. knob_mask("A", "X_XDOT").

    Looks up MASKS["<path>_<group>"]; ``path`` is "A" (upper) or "B" (lower).
    """
    if group == None:
        return MASKS[f"{path}"]
    return MASKS[f"{path}_{group}"]


def posmask2str(posmask):
    """Reverse lookup: the mask's name (e.g. "A_X_XDOT"), or None if unknown."""
    return next((name for name, mask in MASKS.items() if posmask == mask), None)

# de-hysteresis: depends on the physical (3D-printed) mount, so it is tunable.
_dehys = MACHINE["servo"]["de_hysteresis"]
DE_HYSTERESIS = bool(_dehys["enabled"])
DEHYS_OVERSHOOT = int(_dehys["overshoot"])  # encoder steps to overshoot
DEHYS_THRESHOLD = int(_dehys["threshold"])  # min |delta| before de-hys engages

# --- machine.yaml: photodiode ADC (MCP3424 over I2C) -------------------------
ADC = dict(MACHINE["adc"])

# --- machine.yaml: filesystem paths ------------------------------------------
# state_folder holds the runtime servos_<board>.json (servo positions), kept out
# of the source tree. data_folder is the scan/calibration output base. Relative
# paths resolve against the repo root.
STATE_FOLDER = str(_resolve_path(MACHINE["paths"]["state_folder"]))
DATA_FOLDER = str(_resolve_path(MACHINE["paths"]["data_folder"]))

# --- machine.yaml: ZMQ server -------------------------------------------------
SERVER = dict(MACHINE["server"])

# --- calibration.yaml: beam-clip accept functions ----------------------------
# {mask_name: {slope, b, tol}}
ACCEPT_FUNCTIONS = dict(CALIB.get("accept_functions", {}))

# --- calibration.yaml: Jacobian coupling vectors -----------------------------
# {master_path: [[...], [...]]}
COUPLING_VECTORS = dict(CALIB.get("coupling_vectors", {}))

# --- calibration.yaml: optimizer tuning --------------------------------------
SPIRAL_PARAMS = dict(CALIB["spiral"])
BFGS_PARAMS = dict(CALIB["bfgs"])
CLIP_SCAN = dict(CALIB.get("clip_scan", {}))
JACOBIAN = dict(CALIB.get("jacobian", {}))
