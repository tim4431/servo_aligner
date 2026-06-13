# Setup & Usage

How to install the package, configure a machine, drive servos from the CLI,
run the alignment routines, and start the ZMQ server.

## 1. Install

```bash
pip install -e .                         # core: hardware-free (simulation backend works)
pip install -e ".[hardware,plot]"        # on the Pi: serial + I2C drivers, figures
pip install -e ".[hardware,server,plot]" # …plus the expctl ZMQ server
pip install -e ".[dev]"                  # tests
```

Nothing touches hardware at import time; motors only move on explicit CLI
commands or API calls. The FEETECH serial SDK is vendored at
`src/servo_aligner/scservo_sdk/` — nothing to install.

Before first hardware run, set the servos' **Register 18 → 124** with the FD
tool so they report multi-turn angle (see [motor.md](motor.md); the tool and
manual are in [`files/`](files/)).

## 2. Configure

One YAML file per machine holds every setting (serial ports, channel map,
ADC, optimizer tuning, scan stages, output dirs):

```bash
cp config/example_config.yaml ~/servo_aligner.yaml
export SERVO_ALIGNER_CONFIG=~/servo_aligner.yaml   # or pass -c per command
```

The example file is fully commented and mirrors the rydpiservo production
setup. Key sections:

- `actuator.channels` — ordered list of `{name, servo_id, label}`; the order
  defines the channel index used in vectors and masks.
- `groups` — named channel groups (replaces the old hardcoded masks).
- `sensor` — MCP3424 address/channel/gain and the readings-per-measurement.
- `optimize` / `clip_scan` / `jacobian` — the lab-validated tuning constants.
- `actuator.backend: simulation` + `sensor.backend: simulation` — run the
  whole stack hardware-free against the beam-clip physics model
  ([simulation.md](simulation.md)).

Servo positions persist in `state_dir/servos_<board>.json` (written after
each move and at exit) so software turn-counting survives restarts — but not
a driver-board power loss.

## 3. CLI

```bash
servo-aligner status                      # read all channel angles
servo-aligner home                        # all channels to 0° (count 2048)
servo-aligner set-zero                    # define current pose as zero
servo-aligner set-angle 10 -5 0 0 0 0 0 0 # absolute angles (deg), all channels
servo-aligner set-single 3 12.5           # one channel
servo-aligner clip-scan [--no-plot]       # raster knob pairs, fit clip center, re-zero
servo-aligner calibrate-jacobian [--master B] [--offset-type pm] [--norm 20] [-n 5]
servo-aligner server [--dehys 0|1]        # expctl ZMQ server (server extra)
```

`python -m servo_aligner …` is equivalent. Long unattended scans on the Pi:

```bash
nohup servo-aligner clip-scan --no-plot > clip.log 2>&1 &
```

What the routines do and the physics behind them:
[application.md](application.md) (beam centering & MOT alignment),
[spiral.md](spiral.md) (the 2D optimizer), [optimize.md](optimize.md)
(staged rounds), [jacobian.md](jacobian.md) (coupling calibration).

## 4. The ZMQ server (expctl)

`servo-aligner server` listens on a ZMQ REP socket (port from
`server.port`, default **60627**), receives pickled `Sequence` objects from
the `expctl` framework, and moves the servos to the requested angles during
the `QUEUE` phase. The wire protocol is unchanged from the pre-refactor
server, so expctl clients need no changes. Unpickling `Sequence` payloads
requires expctl's `utilities` package importable on the server side.

## 5. Using the library directly

```python
from servo_aligner.config import load_config
from servo_aligner.factory import build_stack
from servo_aligner.optimize.step import step_optimize
import numpy as np

cfg = load_config("~/servo_aligner.yaml")
with build_stack(cfg) as stack:
    group = stack.layout.group("A_X_XDOT")
    zero = step_optimize(stack.measurement, group, np.zeros(stack.layout.n),
                         opt=cfg.optimize)
```

Porting to different hardware = implementing the two small protocols in
`servo_aligner/hal/interfaces.py` (`Actuator`, `IntensitySensor`) and wiring
them into `factory.py`; channel count and grouping are entirely
config-driven.

## 6. Tests

```bash
pip install -e ".[dev]" && pytest
```

The suite runs hardware-free: golden-value parity tests against the
pre-refactor numerics, a fake serial bus for the de-hysteresis/multi-turn
logic, and end-to-end runs of both routines on the simulation backend.
