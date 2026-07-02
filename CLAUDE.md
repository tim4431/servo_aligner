# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Controls FEETECH **STS3032** serial-bus servo motors that turn the adjuster knobs of optical mirror mounts, for **automated laser-beam alignment** in a Rydberg-atom cavity experiment. It runs on a Raspberry Pi (`rydpiservo`) wired Pi → URT/serial driver board → daisy-chained servos (addressed by ID), with a photodiode read over I2C (MCP3424 ADC) as the optimization feedback signal.

The physics background, alignment procedure, de-hysteresis rationale, Jacobian/optimization theory, and the "fat tail" beam-clipping model are documented in the developer notes at `tmp/ExportBlock-*/Motorized Mirror Mount *.md` (exported from Notion). Read that note before touching the optimization or scan logic — the algorithms encode hard-won lab findings.

## Repository layout

The tree is split into an **app layer** and an **importable library**:

- **`app/`** — the entry-point scripts you run directly: `servo_server.py` (interactive console), `init_helper.py` (setup wizard), `calibrate_jacobian.py` / `clip_scan.py` (calibration & scan routines), `zmq_server.py` (the ZMQ server), plus `tui.py` (the shared curses toolkit) and `_bootstrap.py`. Every entry script does `import _bootstrap` **first**, which prepends the sibling `src/` dir to `sys.path` so the flat library imports (`from servodriver import Servoset`, …) resolve. Run them as `python app/<script>.py` (or from inside `app/`). `tui.py` needs no bootstrap — it has no project imports.
- **`src/`** — the importable library: `config.py`, `servodriver.py`, `servo_util.py`, `callback_functions.py`, `optimize.py`, `step_optimize.py`, `spiral.py`, `datastore.py`, `fit_gaussian.py`, `motor_scan.py`, `numeric_sim.py`, plus vendored code (`scservo_sdk/`, and the expctl stubs `sequence.py` / `ServerClass.py` / `utilities/`). No library module imports an `app/` script — dependencies point app → src only, so the library stays importable on its own.

## Hardware dependency

Most of the code cannot run on a dev machine: it imports `smbus2`, `MCP342x`, and opens a serial port to real servos. Modules like `src/servodriver.py`, `app/clip_scan.py`, `app/calibrate_jacobian.py` execute hardware I/O **at import time** (e.g. `app/calibrate_jacobian.py` constructs a `Servoset` and enables torque). Don't import these to "check" them — they will fail or move motors. `callback_functions.py` opens the I2C ADC at import too, but **defensively** now (its `_open_adc` falls back to `(None, None)`, and `read_objective` yields NaN, when the ADC or the `smbus2`/`MCP342x` libs are absent), so it is import-safe — though on real hardware it still talks to the ADC. Pure, safe-to-import modules (all in `src/` except `app/tui.py`): `config.py`, `servo_util.py`, `spiral.py`, `fit_gaussian.py`, `numeric_sim.py`, `app/tui.py`, `step_optimize.py`, `optimize.py`, `datastore.py` (`config.py`, and the modules that import it — `step_optimize.py`, `optimize.py`, `datastore.py` — need PyYAML and the `machine.yaml`/`calibration.yaml` files present, since `config.py` reads them at import; none of them touch hardware, though constructing a `datastore.DataStore` creates its run folder under `data/`). `app/tui.py` is a self-contained curses TUI toolkit (item language, theme, status line, input helpers) shared by `app/init_helper.py` and `app/servo_server.py`; it has no project or hardware imports. `src/servodriver.py` is now a clean hardware library (the `sts3032` / `Servoset` classes only — no app, no TUI, no `__main__`); `app/servo_server.py` is the interactive console on top of it. Running `servo_server.py` opens a curses control panel: a live servo monitor (select a servo + field with left/right to set its angle, toggle torque, or zero it), the manual controls (home / set-zero / de-hysteresis), an **objective page** (lists `callback_functions.OBJECTIVES` with their live, NaN-safe values and lets you pick the active one), and a **ZMQ server page** (open from the menu) that starts/stops the `zmq_server.py` server in a background thread and shows its live log at the bottom — you can leave the page and the server keeps running (its logs/prints are captured into a ring buffer, not the screen). Set-zero is double-confirmed. The server and the console share one `Servoset`; `Servoset` serialises every bus transaction with an `RLock` (per-transaction, not per-move), so the monitor and manual controls work **while the server is running** without their serial packets interleaving.

## Two ways the code runs

1. **ZMQ server** — `zmq_server.py` (class `STSServer`) subclasses `ServerClass.Server` and plugs into the lab's external `expctl` experiment-control framework. It listens on a ZMQ REP socket (port 60627), receives pickled `Sequence` objects (`sequence.py` is a vendored copy of the expctl class), and drives servos to the requested angles during the `QUEUE` phase. Its `Servoset` is *injected*, and `main_loop(cond_fn)` polls every 10 ms re-checking `cond_fn`, so `servo_server.py` can run it in a background thread (sharing one serial connection) and stop it cleanly.
2. **Interactive console** — `app/servo_server.py` is the app: a curses control panel with the live servo monitor, the manual servo controls that used to be the ZMQ server's CLI (`set_zero`, `home`, `set_angle`, `set_single`, `dehys` — also usable one-shot as `python app/servo_server.py <cmd>`), and a switch to turn the ZMQ server on/off.
3. **Standalone scripts** — `app/clip_scan.py` and `app/calibrate_jacobian.py` are run directly as alignment/calibration routines (see Commands).

Everything runs **standalone** now: launch the `app/` entry scripts directly (`python app/<script>.py`), and their `import _bootstrap` puts the sibling `src/` on `sys.path` for the flat library imports (`from servodriver import Servoset`). The `config/` dir and `state_folder` are set per machine via `config/machine.yaml` (or `$SERVO_ALIGNER_CONFIG_DIR`). (The code was previously deployed into the lab's `expctl` framework as `expctl.servers.servoaligner`, run via `python -m expctl.servers.servoaligner.X`; the vendored `sequence.py` / `ServerClass.py` / `utilities/` stubs under `src/` that back the ZMQ server are what remain of that path.)

## Setup

All machine- and setup-specific values live in **two gitignored YAML files** under `config/` (sibling of `src/` and `app/`, not inside them), loaded once by `config.py` (requires PyYAML).

First-time setup is easiest via the wizard `app/init_helper.py` (`python app/init_helper.py`): it (1) installs the Python deps from `requirements.txt` (offers to `pip install` any missing into the running interpreter), (2) creates both YAML files from the templates (preserving the template comments via `ruamel.yaml`, falling back to PyYAML), prompting for each value, (3) scans the serial bus to build the `servo.channels` map, and (4) performs the steps doc/motor.md otherwise does by hand in FEETECH's FD tool — assigning servo IDs and enabling hardware multi-turn (`Register 18 → 124`, via the SDK's EEPROM unlock/lock). It talks to the bus directly through `scservo_sdk` (it does **not** import `config.py`/`servodriver`, enable torque, or command any motion), so it is safe to run before the config files exist; `--no-deps` / `--no-bus` skip the dependency and serial/register steps. Or just copy the templates by hand:

```bash
cp config/machine.template.yaml     config/machine.yaml       # hardware / software
cp config/calibration.template.yaml config/calibration.yaml   # optics / calibration
```

- **`machine.yaml`** — per-Pi hardware/software: serial `devices`/`baudrate`, servo `speed`/`acc`, the `de_hysteresis` tuning, the `channels` map (list order = channel index, each `{id, name}`) and the channel grouping `masks` that go with it (under `servo`), the `adc` I2C wiring (MCP3424 bus/address/channel/gain), filesystem `paths` (`state_folder`, `data_folder`), and the ZMQ `server` (`name`/`port`/`board_id`).
- **`calibration.yaml`** — optics-setup/calibration-dependent: the beam-clip `accept_functions`, Jacobian `coupling_vectors`, and optimizer tuning (`spiral`, `bfgs`, `clip_scan`, `jacobian`).

`config.py` searches for the files in `$SERVO_ALIGNER_CONFIG_DIR`, then `<repo>/config`, then next to `config.py` (production fallback); per-file paths can also be overridden with `SERVO_ALIGNER_MACHINE_CONFIG` / `SERVO_ALIGNER_CALIB_CONFIG`. It exposes the values as module constants (`DEVICENAME_LIST`, `sts3032_dict`, `STATE_FOLDER`, `MASKS`, …); relative `paths` resolve against the repo root. STS3032-fixed constants (control-table addresses; the `2048`/`4096` encoder geometry in `servo_util.py`) stay in code, not YAML. Runtime state `servos_<board>.json` lives under `state_folder` (outside `src/`), persisting encoder positions across restarts; it is rewritten on every move and at exit (`atexit`). Task **output** (scans, Jacobian datasets, plots) instead goes under `data_folder` (the `data/` root) through `datastore.py`: a `DataStore("<name>")` is one named run folder under `data/` — it **warns if that folder already exists** (a re-run may overwrite earlier results; pass `unique=True` for a fresh, auto-numbered `name-2`/`name-3`/… folder) and exposes uniform `save_npz`/`save_npy`/`save_fig`/`load_npz`/`load_npy`/`path`/`exists` helpers, so each task decides *what* to save while *where/how* it lands is centralized. `clip_scan.py` and `calibrate_jacobian.py` build their store from `output_subdir` (`calibration.yaml`); input paths (e.g. the assumed-Jacobian `.npz`) resolve via `datastore.resolve_under_data` (absolute as-is, relative under `data/`). The state file is a separate concern and stays under `state_folder`.

## Commands

Run the entry scripts from the repo root as `python app/<script>.py` (each does `import _bootstrap` to put `src/` on the path); or `cd app` and run them by name.

```bash
# First-time setup wizard: writes config/*.yaml, builds the channel map by
# scanning the bus, assigns servo IDs, enables multi-turn (register 18):
python app/init_helper.py      # installs deps, writes config, sets ids/multi-turn
                               #   --no-deps / --no-bus skip those steps

# Standalone routines — these touch hardware:
python app/clip_scan.py            # 2D raster scan of knob pairs, fit beam-clip center
python app/calibrate_jacobian.py   # spiral + L-BFGS-B coupling optimization, derive Jacobian
# long-running scan, detached:
nohup python app/clip_scan.py > clip.log 2>&1 &

# Interactive console (control panel: monitor + manual controls + ZMQ on/off):
python app/servo_server.py                # control panel (TUI)
python app/servo_server.py set_zero       # one-shot manual command (set_zero/home/set_angle/set_single/dehys)
python app/zmq_server.py                   # run the ZMQ server directly (no console)
```

There is no build, lint, or test suite. The `example/notebooks/*.ipynb` are exploratory (simulation, servo comparison); `src/numeric_sim.py` / `src/spiral.py` have `__main__` blocks that produce matplotlib demos and are the only things runnable without hardware.

## Core concepts (needed to read the optimization code)

- **Channel layout**: 8 servos = two beam paths × two mirrors × two knobs. Naming convention `A` = upper path, `B` = lower path; per path the four knobs are `x, y, xdot, ydot` (a position knob and an angle knob on each of two mirrors). Geometrically the knob pairs are coupled, which is the whole reason for numeric optimization.
- **`pos_mask`** (`config.py`): an 8-element 0/1 list selecting which channels a given step acts on (e.g. `A_X_XDOT_MASK = [1,0,1,0,0,0,0,0]`). The masks and the `posmask2str` reverse lookup live in `config.py` (loaded from `machine.yaml`'s `servo.masks`, since they follow the channel wiring); `posmask2acceptfunc` (in `clip_scan.py`) looks up the accept function by mask name.
- **Vector notations in `servo_util.py`** — three representations the helpers convert between:
  - `r` = *reduced* vector, only the masked entries (length = `sum(mask)`)
  - `nr` = full-length **angle** vector in degrees
  - `nd` = full-length **encoder-count** position (`a2p`: angle→count; the `d` is encoder counts, not "digital"; `2048` = 0°, `4096`/turn)
  - Helpers: `r2nr`/`r2nd` (embed reduced into full), `nrselr` (extract masked), `nraddr`/`nrmodr`/`ndmodr` (add/overwrite masked entries).
- **`compose_para`** (`servo_util.py`): the heart of every objective. Builds a full angle command from a reduced parameter `para` on top of a `zero` offset, and — if a Jacobian is supplied — sets the *slave* knobs to follow the *master* knobs via `dB = J·(dA − offset)`. `jac_master_mask` picks which channels are master.
- **`callback_func`**: the optimizer objective — composes a full angle command (`compose_para`), moves the servos, reads an objective function, returns `(para, intensity)`. It is built by `make_callback_func(servos, objective)` in `callback_functions.py`, which also holds the MCP3424 ADC setup (merged from the old `pd.py`) and an `OBJECTIVES` registry of selectable objective functions (`intensity_adc` is the default; `get_objective(name=None)` picks one by name or at random). `read_objective(func)` evaluates one defensively — NaN if the ADC is absent or a read fails — which the servo console's objective page uses to show each objective's live value. `clip_scan.py` / `calibrate_jacobian.py` just call `make_callback_func(servos)`.
- **Optimization stack**: `step_optimize.py` holds two functions — `step_optimize` (one optimization stage; only commits the new origin if a re-measure confirms the gain) and `pts_iterator` (dispatches to `spiral`, `L-BFGS-B`, or `Powell`; spiral is 2D-only) — calling down to `spiral.py` (`SpiralPath`: a space-filling spiral whose center is dragged toward higher intensity — the custom "spiral descent" algorithm from the notes). Both take an `opt_type` (`"max"` / `"min"` / `"zero"`); `score_of` turns it into the value the optimizers always maximize (`v` / `-v` / `-abs(v)`, so `"zero"` drives the objective toward 0). `"max"` keeps the original positive-intensity ≥70%-of-best ratio guard exactly; `"min"`/`"zero"` use a sign-safe score-space variant (commit if a re-read retains ≥`accept_frac` of the gain over the start). `optimize.py` sits on top: `optimize_knobs(servos, callback_func, zero, masks, opt_type=…)` is a reusable **template** that runs one `step_optimize` per knob mask (auto-picking `spiral` for 2-knob groups, `L-BFGS-B` otherwise), threading the origin through, and returns `(best_zero, value)`; `fiber_coupling(servos, callback_func, zero, path=…)` is the concrete fiber-coupling recipe (`X_Y → X_XDOT → Y_YDOT → joint L-BFGS-B over POS_ALL`) that `calibrate_jacobian.py` calls.
- **De-hysteresis** (`Servoset.set_position`): because the 3D-printed mount frame flexes, moves in the negative direction overshoot by 100 encoder steps then return, so backlash is always taken up from the same side. Toggle via `servos.de_hysterisis` (also the `dehys` CLI command). Calibration accuracy depends on this being set correctly.
- **Multi-turn encoder**: a single turn is 0–4095; `Servoset` tracks turn count in software (`turn_num`) by watching for large position jumps. The notes describe setting servo **register 18 → 124** in FEETECH's FD tool to get hardware multi-turn reporting (±7 turns); driver-board power loss resets the turn count.

## Vendored / external

- `src/scservo_sdk/` — FEETECH's serial servo SDK, vendored unmodified (`doc/scservo_sdk_README.md`). Don't edit; it's upstream.
- `src/sequence.py` / `src/ServerClass.py` — copies of expctl framework code so the server can unpickle `Sequence` objects; `sequence.py` does `from utilities.util import *`. `src/utilities/util.py` is a minimal vendored stub of that (just the coloured-print helpers, routed to logging) so the ZMQ server (`app/zmq_server.py`, which reaches these via the `_bootstrap` path) can **start** standalone; in the old expctl deployment the real top-level `utilities` package shadowed it (absolute import), so the stub only applies to standalone runs.
