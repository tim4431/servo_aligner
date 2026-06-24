# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Controls FEETECH **STS3032** serial-bus servo motors that turn the adjuster knobs of optical mirror mounts, for **automated laser-beam alignment** in a Rydberg-atom cavity experiment. It runs on a Raspberry Pi (`rydpiservo`) wired Pi → URT/serial driver board → daisy-chained servos (addressed by ID), with a photodiode read over I2C (MCP3424 ADC) as the optimization feedback signal.

The physics background, alignment procedure, de-hysteresis rationale, Jacobian/optimization theory, and the "fat tail" beam-clipping model are documented in the developer notes at `tmp/ExportBlock-*/Motorized Mirror Mount *.md` (exported from Notion). Read that note before touching the optimization or scan logic — the algorithms encode hard-won lab findings.

## Hardware dependency

Most of `src/` cannot run on a dev machine: it imports `smbus2`, `MCP342x`, and opens a serial port to real servos. Modules like `pd.py`, `servodriver.py`, `clip_scan.py`, `calibrate_jacobian.py` execute hardware I/O **at import time** (e.g. `pd.py` reads the ADC; `calibrate_jacobian.py` constructs a `Servoset` and enables torque). Don't import these to "check" them — they will fail or move motors. Pure, safe-to-import modules: `config.py`, `servo_util.py`, `servo_const.py`, `spiral.py`, `fit_gaussian.py`, `numeric_sim.py` (the first two need PyYAML and the `machine.yaml`/`calibration.yaml` files present, since `config.py` reads them at import and `servo_const.py` pulls its masks from `config`).

## Two ways the code runs

1. **ZMQ server** — `STSServer.py` subclasses `ServerClass.Server` and plugs into the lab's external `expctl` experiment-control framework. It listens on a ZMQ REP socket (port 60627), receives pickled `Sequence` objects (`sequence.py` is a vendored copy of the expctl class), and drives servos to the requested angles during the `QUEUE` phase. It also exposes an argparse CLI (`set_zero`, `home`, `set_angle`, `set_single`, `dehys`) parsed once at startup before entering the message loop.
2. **Standalone scripts** — `clip_scan.py` and `calibrate_jacobian.py` are run directly as alignment/calibration routines (see Commands).

In production the whole `src/` tree is dropped into the expctl package as `expctl.servers.servoaligner` (note the `python -m expctl.servers.servoaligner.X` invocations); the `config/` dir and `state_folder` are set per deployment via `config/machine.yaml` (or `$SERVO_ALIGNER_CONFIG_DIR`). Run standalone, scripts use flat imports (`from servodriver import Servoset`) and must be launched from inside `src/`.

## Setup

All machine- and setup-specific values live in **two gitignored YAML files** under `config/` (sibling of `src/`, not inside it), loaded once by `config.py` (requires PyYAML). Create them from the checked-in templates:

```bash
cp config/machine.template.yaml     config/machine.yaml       # hardware / software
cp config/calibration.template.yaml config/calibration.yaml   # optics / calibration
```

- **`machine.yaml`** — per-Pi hardware/software: serial `devices`/`baudrate`, servo `speed`/`acc`, the `de_hysteresis` tuning, the `channels` map (list order = channel index, each `{id, name}`), the `adc` I2C wiring (MCP3424 bus/address/channel/gain), filesystem `paths` (`state_folder`, `data_folder`), and the ZMQ `server` (`name`/`port`/`board_id`).
- **`calibration.yaml`** — optics-setup/calibration-dependent: the channel grouping `masks`, beam-clip `accept_functions`, Jacobian `coupling_vectors`, and optimizer tuning (`spiral`, `bfgs`, `clip_scan`, `jacobian`).

`config.py` searches for the files in `$SERVO_ALIGNER_CONFIG_DIR`, then `<repo>/config`, then next to `config.py` (production fallback); per-file paths can also be overridden with `SERVO_ALIGNER_MACHINE_CONFIG` / `SERVO_ALIGNER_CALIB_CONFIG`. It exposes the values as module constants (`DEVICENAME_LIST`, `sts3032_dict`, `STATE_FOLDER`, `MASKS`, …); relative `paths` resolve against the repo root. STS3032-fixed constants (control-table addresses; the `2048`/`4096` encoder geometry in `servo_util.py`) stay in code, not YAML. Runtime state `servos_<board>.json` lives under `state_folder` (outside `src/`), persisting encoder positions across restarts; it is rewritten on every move and at exit (`atexit`).

## Commands

```bash
# Run from src/ (standalone) — these touch hardware:
python clip_scan.py            # 2D raster scan of knob pairs, fit beam-clip center
python calibrate_jacobian.py   # spiral + L-BFGS-B coupling optimization, derive Jacobian

# In production (installed under expctl on rydpiservo):
python -m expctl.servers.servoaligner.STSServer          # start the ZMQ server
python -m expctl.servers.servoaligner.STSServer set_zero # CLI subcommands
python -m expctl.servers.servoaligner.calibrate_jacobian
# long-running scan, detached:
nohup python -m expctl.servers.servoaligner.clip_scan > clip.log 2>&1 &
```

There is no build, lint, or test suite. The `example/notebooks/*.ipynb` are exploratory (simulation, servo comparison); `numeric_sim.py` / `spiral.py` have `__main__` blocks that produce matplotlib demos and are the only things runnable without hardware.

## Core concepts (needed to read the optimization code)

- **Channel layout**: 8 servos = two beam paths × two mirrors × two knobs. Naming convention `A` = upper path, `B` = lower path; per path the four knobs are `x, y, xdot, ydot` (a position knob and an angle knob on each of two mirrors). Geometrically the knob pairs are coupled, which is the whole reason for numeric optimization.
- **`pos_mask`** (`servo_const.py`): an 8-element 0/1 list selecting which channels a given step acts on (e.g. `A_X_XDOT_MASK = [1,0,1,0,0,0,0,0]`). `posmask2str` / `posmask2acceptfunc` look up behavior by identity-comparing against the known masks, so pass the actual constant objects, not copies.
- **Vector notations in `servo_util.py`** — three representations the helpers convert between:
  - `r` = *reduced* vector, only the masked entries (length = `sum(mask)`)
  - `nr` = full-length **angle** vector in degrees
  - `nd` = full-length **encoder-count** position (`a2p`: angle→count; the `d` is encoder counts, not "digital"; `2048` = 0°, `4096`/turn)
  - Helpers: `r2nr`/`r2nd` (embed reduced into full), `nrselr` (extract masked), `nraddr`/`nrmodr`/`ndmodr` (add/overwrite masked entries).
- **`compose_para`** (`servo_util.py`): the heart of every objective. Builds a full angle command from a reduced parameter `para` on top of a `zero` offset, and — if a Jacobian is supplied — sets the *slave* knobs to follow the *master* knobs via `dB = J·(dA − offset)`. `jac_master_mask` picks which channels are master.
- **`callback_func`**: the optimizer objective — calls `compose_para`, moves the servos, reads the photodiode twice and averages, returns `(para, intensity)`. **It is duplicated** (copy-pasted) in `clip_scan.py`, `calibrate_jacobian.py`, and `callback_func.py`; the standalone module version references `servos`/`MCP3424_fiber` globals it doesn't define and only works inside a namespace that does.
- **Optimization stack**: `step_optimize.py` holds two functions — `step_optimize` (one optimization stage; only commits the new origin if the resulting intensity stays ≥70% of the best seen) and `pts_iterator` (dispatches to `spiral`, `L-BFGS-B`, or `Powell`; spiral is 2D-only) — calling down to `spiral.py` (`SpiralPath`: a space-filling spiral whose center is dragged toward higher intensity — the custom "spiral descent" algorithm from the notes).
- **De-hysteresis** (`Servoset.set_position`): because the 3D-printed mount frame flexes, moves in the negative direction overshoot by 100 encoder steps then return, so backlash is always taken up from the same side. Toggle via `servos.de_hysterisis` (also the `dehys` CLI command). Calibration accuracy depends on this being set correctly.
- **Multi-turn encoder**: a single turn is 0–4095; `Servoset` tracks turn count in software (`turn_num`) by watching for large position jumps. The notes describe setting servo **register 18 → 124** in FEETECH's FD tool to get hardware multi-turn reporting (±7 turns); driver-board power loss resets the turn count.

## Vendored / external

- `src/scservo_sdk/` — FEETECH's serial servo SDK, vendored unmodified (`doc/scservo_sdk_README.md`). Don't edit; it's upstream.
- `sequence.py` / `SequenceProcessor.py` — copies of expctl framework code so the server can unpickle `Sequence` objects; `sequence.py` imports `utilities.util` from expctl and isn't importable standalone.
