"""Central configuration for the servo aligner.

Everything that differs between installations lives in two YAML files so that
moving to a new machine (or a new optical setup) means editing YAML, not code:

* ``machine.yaml``     - hardware / software settings tied to a particular
  Raspberry Pi: serial devices, baudrate, the servo channel map, ADC wiring,
  filesystem paths and the ZMQ server identity.
* ``calibration.yaml`` - optics-setup / calibration settings: the channel
  grouping masks, beam-clip accept functions, Jacobian coupling vectors and the
  optimizer tuning parameters.

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

_HERE = Path(__file__).resolve().parent


def _load(filename, env_var):
    """Load a YAML config file, honouring an env-var path override."""
    override = os.environ.get(env_var)
    path = Path(override) if override else _HERE / filename
    if not path.exists():
        template = path.with_name(filename.replace(".yaml", ".template.yaml"))
        raise FileNotFoundError(
            f"Config file '{path}' not found. Copy the template and edit it:\n"
            f"    cp {template} {path}"
        )
    with open(path, "r") as f:
        return yaml.safe_load(f)


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

# de-hysteresis: depends on the physical (3D-printed) mount, so it is tunable.
_dehys = MACHINE["servo"]["de_hysteresis"]
DE_HYSTERESIS = bool(_dehys["enabled"])
DEHYS_OVERSHOOT = int(_dehys["overshoot"])  # encoder steps to overshoot
DEHYS_THRESHOLD = int(_dehys["threshold"])  # min |delta| before de-hys engages

# --- machine.yaml: photodiode ADC (MCP3424 over I2C) -------------------------
ADC = dict(MACHINE["adc"])

# --- machine.yaml: filesystem paths ------------------------------------------
HOME_FOLDER = str(MACHINE["paths"]["home_folder"])
DATA_FOLDER = str(MACHINE["paths"]["data_folder"])

# --- machine.yaml: ZMQ server -------------------------------------------------
SERVER = dict(MACHINE["server"])

# --- calibration.yaml: channel grouping masks --------------------------------
# {mask_name: [0/1, ...]} over the channels defined in machine.yaml.
MASKS = {name: list(m) for name, m in CALIB["masks"].items()}

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
