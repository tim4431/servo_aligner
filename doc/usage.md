# Setup & Usage

How to configure the program, start the ZMQ server, drive servos from the CLI,
and run the standalone alignment/calibration scripts.

## 1. Prerequisites

This software talks to **real hardware** and is meant to run on the Raspberry Pi
(`rydpiservo`): Pi → URT/serial driver board → daisy-chained STS3032 servos, plus
an MCP3424 ADC on I2C reading the photodiode. Most modules **touch hardware at
import time**, so they cannot be imported on a dev machine.

Python dependencies: `numpy`, `scipy`, `matplotlib`, `tqdm`, `pyzmq`,
`coloredlogs`, `pyyaml` (config files), `smbus2`, `MCP342x` (I2C ADC). The
FEETECH serial SDK is vendored in `src/scservo_sdk/` — nothing to install.

Before first run, set the servos' **Register 18 → 124** with the FD tool so they
report multi-turn angle (see [motor.md](motor.md); the tool and a Chinese
getting-started PDF are in [`files/`](files/)).

## 2. Configure the YAML files

All machine- and setup-specific values live in **two gitignored YAML files** in
`config/` (a sibling of `src/`, kept out of the source tree), loaded once by
[`config.py`](../src/config.py). Create them from the checked-in templates:

```bash
cp config/machine.template.yaml     config/machine.yaml
cp config/calibration.template.yaml config/calibration.yaml
```

### [`machine.yaml`](../config/machine.template.yaml) — hardware / software (per machine)

| Section | Meaning |
|---------|---------|
| `serial.devices` | Candidate serial devices, tried in order (e.g. `/dev/ttyUSB0…2`). The first one that opens wins. |
| `serial.baudrate` | Serial baudrate; STS3032 default is `1000000`. |
| `servo.speed`, `servo.acc` | Default move speed / acceleration applied to every servo. |
| `servo.de_hysteresis` | Backlash compensation tuned to the physical mount: `enabled`, `overshoot` (steps), `threshold`. |
| `servo.channels` | The channel map — a list of `{id, name}`. **List order defines the channel index** and thus the 8-element vectors used everywhere (masks, angles). |
| `adc` | MCP3424 I2C wiring: `i2c_bus`, `address`, `channel`, `gain`, `resolution`. |
| `paths.state_folder` | Folder where the runtime `servos_<board>.json` (saved positions) lives — kept out of `src/`, created automatically. Relative paths resolve against the repo root; use an absolute path in production. |
| `paths.data_folder` | Base folder for scan/calibration output (scripts create subfolders under it). Same relative/absolute rule. |
| `server` | ZMQ server identity: `name`, `port`, `board_id`. |

> `servo.channels` is the source of truth for the channel layout. The default maps
> indices 0–7 to the eight alignment knobs; the commented-out entries (UV servos)
> show how to extend it. Channel **index** (list position) is distinct from servo
> **ID** (bus address) — keep both correct.

### [`calibration.yaml`](../config/calibration.template.yaml) — optics / calibration (per setup)

| Section | Meaning |
|---------|---------|
| `masks` | The channel grouping masks (`A_X_XDOT`, …): which channel indices form each knob group. Depends on which servo drives which knob. |
| `accept_functions` | Beam-clip raster region per mask: `slope`, `b`, `tol`. |
| `coupling_vectors` | Per-path (`A`/`B`) Jacobian coupling directions used to seed offsets. |
| `spiral`, `bfgs` | Optimizer tuning (spiral-descent + L-BFGS-B). |
| `clip_scan`, `jacobian` | Scan/calibration knobs: output subfolders, point counts, scan ranges, probe magnitude. |

`config.py` looks for the files in `$SERVO_ALIGNER_CONFIG_DIR`, then `<repo>/config`,
then next to `config.py` (a production fallback); individual paths can be overridden
with the `SERVO_ALIGNER_MACHINE_CONFIG` / `SERVO_ALIGNER_CALIB_CONFIG` environment
variables. Constants fixed by the STS3032 itself (control-table addresses, the
`2048`/`4096` encoder geometry) are **not** in YAML — they stay in code.

`servos_<board>.json` (under `state_folder`) is created automatically on first run
and rewritten on every move and at exit; it lets software turn-counting survive a
program restart (but not a driver-board power loss). It records `board_id`, a
timestamp, the servo **IDs** (bus addresses) and the positions; on load, a
mismatching servo count or ID list is reported and stale positions are ignored. A
template is in [`servos_template.json`](servos_template.json).

## 3. Two ways to run

The same `src/` tree runs in two contexts, which **changes how you launch it**:

- **Standalone** — run scripts directly *from inside `src/`* so the flat imports
  (`from servodriver import Servoset`) resolve.
- **Installed under expctl** — in production the tree is dropped into the lab's
  experiment-control package as `expctl.servers.servoaligner`, and you launch
  with `python -m expctl.servers.servoaligner.<module>`.

### Start the ZMQ server

[`STSServer.py`](../src/STSServer.py) listens on a ZMQ REP socket (**port 60627**), receives pickled
`Sequence` objects from the `expctl` framework, and moves the servos to the
requested angles during the `QUEUE` phase.

```bash
# standalone, from src/
python STSServer.py

# installed
python -m expctl.servers.servoaligner.STSServer
```

On startup it constructs a `Servoset` for channels `[0..7]`, enables torque, and
turns **de-hysteresis on** before entering the message loop.

### Server CLI subcommands

[`STSServer.py`](../src/STSServer.py) also parses an argparse CLI **once at startup**, before the
message loop — handy for manual setup without the experiment framework:

```bash
python STSServer.py set_zero              # set current pose as the zero/center
python STSServer.py home                  # move all servos to 0° (position 2048)
python STSServer.py set_angle 10 -5 0 ... # absolute angles (deg) for all channels
python STSServer.py set_single 3 12.5     # move one channel (index 3) to 12.5°
python STSServer.py dehys 0|1             # de-hysteresis off (0) / on (1)
```

(Replace `python STSServer.py` with the `python -m expctl.…STSServer` form when
installed.)

## 4. Standalone alignment / calibration scripts

These construct their own `Servoset` and **start moving motors immediately** when
run — they are scripts, not libraries. Run from `src/` (or via `-m`):

```bash
python clip_scan.py            # 2D raster scan of a knob pair; fit the beam-clip center
python calibrate_jacobian.py   # spiral + L-BFGS-B coupling optimization; collect Jacobian data
```

For a long unattended scan on the Pi, detach it:

```bash
nohup python -m expctl.servers.servoaligner.clip_scan > clip.log 2>&1 &
```

What they do and the physics behind them: [application.md](application.md)
(beam centering & MOT alignment), [spiral.md](spiral.md) (the 2D optimizer),
[optimize.md](optimize.md) (how spiral stages chain into a full round), and
[jacobian.md](jacobian.md) (the coupling calibration). Output folders are derived
from `paths.data_folder` (machine.yaml) plus the `output_subdir` in
calibration.yaml (`clip_scan.output_subdir`, `jacobian.output_subdir`), and are
created automatically — point `data_folder` at a writable location.

## 5. Safe-to-import vs hardware modules

If you are reading/editing code off the Pi, only these import without hardware:
[`config.py`](../src/config.py), [`servo_util.py`](../src/servo_util.py), [`spiral.py`](../src/spiral.py), [`fit_gaussian.py`](../src/fit_gaussian.py),
[`numeric_sim.py`](../src/numeric_sim.py). Everything else opens a serial port or the I2C ADC at import.
[`spiral.py`](../src/spiral.py) and [`numeric_sim.py`](../src/numeric_sim.py) have `__main__` matplotlib demos that are the
only things you can actually *run* on a dev machine.
