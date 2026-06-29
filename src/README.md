# servo_aligner — `src/`

Control software for FEETECH **STS3032** serial-bus servo motors driving optical mirror-mount knobs, used for automated laser-beam alignment in a Rydberg-atom cavity experiment. Runs on a Raspberry Pi: Pi → serial driver board → daisy-chained servos, with a photodiode read over I2C (MCP3424) as the optimization feedback.

> Most modules here touch real hardware (serial port, I2C) **at import time** and cannot run on a machine without the servos and ADC attached.

## Setup

Configuration lives in two gitignored YAML files under [`../config/`](../config/) (a sibling of `src/`), loaded by [`config.py`](config.py) (needs PyYAML). Python dependencies are listed in [`../requirements.txt`](../requirements.txt).

**First time on a new machine, run the setup wizard** — it installs the dependencies, creates both config files from the templates, walks you through the values, scans the bus to build the channel map, and does the FD-tool register steps (assign servo IDs, enable multi-turn `Register 18 → 124`):

```bash
python init_helper.py           # interactive; --no-deps / --no-bus skip those steps
```

Or do it by hand — install deps and copy the templates:

```bash
pip install -r ../requirements.txt
cp ../config/machine.template.yaml     ../config/machine.yaml       # serial bus, channel map + masks, ADC, paths, server
cp ../config/calibration.template.yaml ../config/calibration.yaml   # accept functions, coupling vectors, optimizer tuning
```

`machine.yaml` holds per-machine hardware/software settings; `calibration.yaml` holds optics-setup/calibration values. `config.py` finds them via `$SERVO_ALIGNER_CONFIG_DIR` → `<repo>/config` → next to `config.py`; per-file overrides via `SERVO_ALIGNER_MACHINE_CONFIG` / `SERVO_ALIGNER_CALIB_CONFIG`. Runtime `servos_<board>.json` is written under `state_folder` (outside `src/`), persisting encoder positions between runs.

## Layout

| File | Role |
|------|------|
| `init_helper.py` | First-run setup wizard: installs the deps in `../requirements.txt`, creates `../config/*.yaml` from the templates (comments preserved), prompts for each value, scans the bus to build the channel map, and assigns servo IDs / enables multi-turn (`Register 18 → 124`). Talks to the bus directly via `scservo_sdk`; safe to run before the YAML files exist, and never moves a servo. |
| `config.py` (+ `../config/*.yaml`) | Loads the two YAML config files (machine hardware/software vs. optics/calibration) from `../config/` and exposes them as constants. Edit the YAML, not the code, when migrating. |
| `servodriver.py` | `sts3032` (single motor) and `Servoset` (the 8-motor set): connect, move, multi-turn tracking, de-hysteresis, position persistence. Clean hardware library — no app/TUI. |
| `servo_server.py` / `tui.py` | Interactive console (curses control panel): live servo monitor, manual controls (set-zero/home/dehys, also one-shot via CLI), and a switch to turn the ZMQ server on/off. `tui.py` is the shared curses toolkit. |
| `callback_functions.py` | The optimizer objectives: the MCP3424 photodiode ADC reader (over I2C), an `OBJECTIVES` registry (`intensity_adc`, …), and `make_callback_func(servos, objective)` that builds the move-then-measure `callback_func`. |
| `scservo_sdk/` | Vendored FEETECH serial-servo SDK (do not edit). |
| `zmq_server.py` / `ServerClass.py` | ZMQ server (class `STSServer`) that plugs into the lab `expctl` framework and drives servos from received sequences. Its `Servoset` is injected so `servo_server.py` can run it in a background thread. |
| `sequence.py` / `SequenceProcessor.py` | Vendored expctl classes so the server can unpickle `Sequence` objects. |
| `clip_scan.py` | 2D raster scan of knob pairs; fit beam-clip ellipse to find the center. |
| `calibrate_jacobian.py` | Spiral + L-BFGS-B coupling optimization; derive the knob Jacobian. |
| `step_optimize.py` / `spiral.py` | Optimization engine: `step_optimize` (one stage) and `pts_iterator` (optimizer dispatch) live in `step_optimize.py`; `spiral.py` is the custom spiral-descent search. |
| `servo_util.py` | The `r`/`nr`/`nd` vector helpers (`compose_para`, etc.); channel masks live in `config.py`. |
| `fit_gaussian.py` / `motor_scan.py` / `numeric_sim.py` / `plot_csv.py` | Fitting, scanning, simulation, and plotting helpers. |

## Running

```bash
# Standalone (from this dir, on the Pi):
python clip_scan.py
python calibrate_jacobian.py

# Interactive console / one-shot manual commands:
python servo_server.py                 # control panel (monitor + manual controls + ZMQ on/off)
python servo_server.py set_zero        # or home / set_angle / set_single / dehys

# As an installed expctl server:
python -m expctl.servers.servoaligner.zmq_server
```

See [`../CLAUDE.md`](../CLAUDE.md) for the architecture and core concepts, and the developer notes under `../tmp/` for the physics and algorithm rationale.
