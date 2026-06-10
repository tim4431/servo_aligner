# Setup & Usage

How to configure the program, start the ZMQ server, drive servos from the CLI,
and run the standalone alignment/calibration scripts.

## 1. Prerequisites

This software talks to **real hardware** and is meant to run on the Raspberry Pi
(`rydpiservo`): Pi → URT/serial driver board → daisy-chained STS3032 servos, plus
an MCP3424 ADC on I2C reading the photodiode. Most modules **touch hardware at
import time**, so they cannot be imported on a dev machine.

Python dependencies: `numpy`, `scipy`, `matplotlib`, `tqdm`, `pyzmq`,
`coloredlogs`, `smbus2`, `MCP342x` (I2C ADC). The FEETECH serial SDK is vendored
in `src/scservo_sdk/` — nothing to install.

Before first run, set the servos' **Register 18 → 124** with the FD tool so they
report multi-turn angle (see [motor.md](motor.md); the tool and a Chinese
getting-started PDF are in [`files/`](files/)).

## 2. Configure `customize.py`

`customize.py` holds all machine-specific settings and is **gitignored** — create
it from the template:

```bash
cp doc/customize.template.py src/customize.py
```

Then edit `src/customize.py`:

| Setting | Meaning |
|---------|---------|
| `HOME_FOLDER` | Absolute folder where `servos_<board>.json` (saved positions) lives. In production this is the installed package path, e.g. `…/expctl/servers/servoaligner/`. |
| `DEVICENAME_LIST` | Candidate serial devices, tried in order (e.g. `['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2']`). The first one that opens wins. |
| `BAUDRATE` | Serial baudrate; STS3032 default is `1000000`. |
| `SERVO_SPEED`, `SERVO_ACC` | Default move speed / acceleration applied to every servo. |
| `sts3032_dict` | The channel map: `{channel_index: [servo_ID, "name"]}`. **Order defines the 8-element vectors** used everywhere (masks, angles). |

> `sts3032_dict` is the source of truth for the channel layout. The default maps
> indices 0–7 to the eight alignment knobs; the commented-out entries (UV servos)
> show how to extend it. Channel **index** (position in the vector) is distinct
> from servo **ID** (bus address) — keep both correct.

`servos_0.json` is created automatically on first run and rewritten on every move
and at exit; it lets software turn-counting survive a program restart (but not a
driver-board power loss). A template is in `doc/servos_template.json`.

## 3. Two ways to run

The same `src/` tree runs in two contexts, which **changes how you launch it**:

- **Standalone** — run scripts directly *from inside `src/`* so the flat imports
  (`from servodriver import Servoset`) resolve.
- **Installed under expctl** — in production the tree is dropped into the lab's
  experiment-control package as `expctl.servers.servoaligner`, and you launch
  with `python -m expctl.servers.servoaligner.<module>`.

### Start the ZMQ server

`STSServer.py` listens on a ZMQ REP socket (**port 60627**), receives pickled
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

`STSServer.py` also parses an argparse CLI **once at startup**, before the
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
[jacobian.md](jacobian.md) (the coupling calibration). Output folders
(`FOLDER`, dataset paths) are hard-coded near the top of each script — point them
at a writable location before running.

## 5. Safe-to-import vs hardware modules

If you are reading/editing code off the Pi, only these import without hardware:
`servo_util.py`, `servo_const.py`, `spiral.py`, `fit_gaussian.py`,
`numeric_sim.py`. Everything else opens a serial port or the I2C ADC at import.
`spiral.py` and `numeric_sim.py` have `__main__` matplotlib demos that are the
only things you can actually *run* on a dev machine.
