# servo_aligner — `src/`

Control software for FEETECH **STS3032** serial-bus servo motors driving optical mirror-mount knobs, used for automated laser-beam alignment in a Rydberg-atom cavity experiment. Runs on a Raspberry Pi: Pi → serial driver board → daisy-chained servos, with a photodiode read over I2C (MCP3424) as the optimization feedback.

> Most modules here touch real hardware (serial port, I2C) **at import time** and cannot run on a machine without the servos and ADC attached.

## Setup

Configuration lives in two gitignored YAML files, loaded by [`config.py`](config.py) (needs PyYAML). Create them from the checked-in templates:

```bash
cp machine.template.yaml     machine.yaml       # serial bus, channel map, ADC, paths, server
cp calibration.template.yaml calibration.yaml   # masks, accept functions, coupling vectors, optimizer tuning
```

`machine.yaml` holds per-machine hardware/software settings; `calibration.yaml` holds optics-setup/calibration values. Override their paths with `SERVO_ALIGNER_MACHINE_CONFIG` / `SERVO_ALIGNER_CALIB_CONFIG` if needed. `servos_0.json` persists the last encoder positions between runs.

## Layout

| File | Role |
|------|------|
| `config.py` + `machine.yaml` / `calibration.yaml` | Loads the two YAML config files (machine hardware/software vs. optics/calibration) and exposes them as constants. Edit the YAML, not the code, when migrating. |
| `servodriver.py` | `sts3032` (single motor) and `Servoset` (the 8-motor set): connect, move, multi-turn tracking, de-hysteresis, position persistence. |
| `pd.py` | Photodiode ADC reader (MCP3424 over I2C) — the optimization signal. |
| `scservo_sdk/` | Vendored FEETECH serial-servo SDK (do not edit). |
| `STSServer.py` / `ServerClass.py` | ZMQ server that plugs into the lab `expctl` framework and drives servos from received sequences; also a small CLI. |
| `sequence.py` / `SequenceProcessor.py` | Vendored expctl classes so the server can unpickle `Sequence` objects. |
| `clip_scan.py` | 2D raster scan of knob pairs; fit beam-clip ellipse to find the center. |
| `calibrate_jacobian.py` | Spiral + L-BFGS-B coupling optimization; derive the knob Jacobian. |
| `step_optimize.py` / `pts_iterator.py` / `spiral.py` | Optimization engine (custom spiral descent + scipy minimizers). |
| `servo_util.py` / `servo_const.py` | Channel masks and the `r`/`nr`/`nd` vector helpers (`compose_para`, etc.). |
| `fit_gaussian.py` / `motor_scan.py` / `numeric_sim.py` / `plot_csv.py` | Fitting, scanning, simulation, and plotting helpers. |

## Running

```bash
# Standalone (from this dir, on the Pi):
python clip_scan.py
python calibrate_jacobian.py

# As an installed expctl server:
python -m expctl.servers.servoaligner.STSServer
```

See [`../CLAUDE.md`](../CLAUDE.md) for the architecture and core concepts, and the developer notes under `../tmp/` for the physics and algorithm rationale.
