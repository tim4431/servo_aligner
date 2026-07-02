# Setup & Usage

How to configure the program, start the ZMQ server, drive servos from the CLI,
and run the standalone alignment/calibration scripts.

The tree is split in two: [`app/`](../app/) holds the entry-point scripts you
run (`python app/<script>.py` from the repo root — each one starts with
`import _bootstrap`, which puts the sibling `src/` on `sys.path`), and
[`src/`](../src/) holds the importable library.

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
`config/` (a sibling of `src/` and `app/`), loaded once by
[`config.py`](../src/config.py). The easiest way to create them is the setup
wizard, `python app/init_helper.py` (it also installs missing dependencies,
scans the bus to build the channel map, and does the servo register steps).
Or create them from the checked-in templates by hand:

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
| `servo.masks` | Channel grouping masks (`A_X_XDOT`, …): which channel indices form each knob group. Lives with the channel map because it depends on the servo→knob wiring. |
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
and rewritten on every move and at exit; it lets the last-known multi-turn
positions survive a program restart (but not a driver-board power loss, which
resets the servos' hardware turn count). It records `board_id`, a
timestamp, the servo **IDs** (bus addresses) and the positions; on load, a
mismatching servo count or ID list is reported and stale positions are ignored. A
template is in [`servos_template.json`](servos_template.json).

## 3. Running the servers and the console

### Start the ZMQ server

[`app/zmq_server.py`](../app/zmq_server.py) (class `STSServer`) listens on a ZMQ REP socket (**port
60627**), receives pickled `Sequence` objects from the lab's `expctl`
experiment-control framework, and moves the servos to the requested angles
during the `QUEUE` phase. Its `Servoset` is injected and its
`main_loop(cond_fn)` polls every 10 ms, so the interactive console can run it in
a background thread and stop it on demand.

```bash
python app/zmq_server.py          # run the server directly (no console)

# or turn it on/off from the interactive console
python app/servo_server.py        # control panel -> "ZMQ server: [open]"
```

### Manual servo controls (the console + one-shot CLI)

[`app/servo_server.py`](../app/servo_server.py) is the interactive console — a curses control panel with
the live servo monitor, the objective page, the manual controls, and the
ZMQ-server switch. The manual controls are also available as one-shot commands,
handy for setup scripts without the experiment framework:

```bash
python app/servo_server.py set_zero              # set current pose as the zero/center
python app/servo_server.py home                  # move all servos to 0° (position 2048)
python app/servo_server.py set_angle 10 -5 0 ... # absolute angles (deg) for all channels
python app/servo_server.py set_single 3 12.5     # move one channel (index 3) to 12.5°
```

(De-hysteresis is a runtime toggle — flip it in the control panel, or set its
default in `machine.yaml`.)

## 4. Standalone alignment / calibration scripts

These construct their own `Servoset` and **start moving motors immediately** when
run — they are scripts, not libraries:

```bash
python app/clip_scan.py            # 2D raster scan of a knob pair; fit the beam-clip center
python app/calibrate_jacobian.py   # spiral + L-BFGS-B coupling optimization; collect Jacobian data
```

For a long unattended scan on the Pi, detach it:

```bash
nohup python app/clip_scan.py > clip.log 2>&1 &
```

What they do and the physics behind them: [application.md](application.md)
(beam centering & MOT alignment), [spiral.md](spiral.md) (the 2D optimizer),
[optimize.md](optimize.md) (how spiral stages chain into a full round), and
[jacobian.md](jacobian.md) (the coupling calibration). Output folders are derived
from `paths.data_folder` (machine.yaml) plus the `output_subdir` in
calibration.yaml (`clip_scan.output_subdir`, `jacobian.output_subdir`), and are
created automatically — point `data_folder` at a writable location.

## 5. Safe-to-import vs hardware modules

If you are reading/editing code off the Pi: every `src/` module *imports*
without hardware (`callback_functions.py` opens the I2C ADC defensively — reads
return NaN when it is absent), but real I/O starts at use: constructing a
`Servoset` opens the serial bus and enables torque, and the `app/` scripts
`clip_scan.py` / `calibrate_jacobian.py` do that (and start moving motors) at
module level — don't import them to "check" them. The pure modules —
[`config.py`](../src/config.py), [`servo_util.py`](../src/servo_util.py),
[`spiral.py`](../src/spiral.py), [`fit_gaussian.py`](../src/fit_gaussian.py),
[`numeric_sim.py`](../src/numeric_sim.py), [`step_optimize.py`](../src/step_optimize.py),
[`optimize.py`](../src/optimize.py), [`datastore.py`](../src/datastore.py),
[`app/tui.py`](../app/tui.py) — never touch hardware at all.
[`spiral.py`](../src/spiral.py) and [`numeric_sim.py`](../src/numeric_sim.py)
have `__main__` matplotlib demos that are the only things you can actually
*run* on a dev machine.
