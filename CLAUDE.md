# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Controls FEETECH **STS3032** serial-bus servo motors that turn the adjuster knobs of optical mirror mounts, for **automated laser-beam alignment** in a Rydberg-atom cavity experiment. It runs on a Raspberry Pi, wired Pi → URT/serial driver board → daisy-chained servos (addressed by ID), with a photodiode read over I2C (MCP3424 ADC) as the optimization feedback signal.

The physics background, alignment procedure, de-hysteresis rationale, Jacobian/optimization theory, and the "fat tail" beam-clipping model are documented under `doc/` (start at `doc/README.md`, which gives a reading order). Read the relevant page before touching the optimization or scan logic — the algorithms encode hard-won lab findings.

## Repository layout

- **`app/`** — the entry-point scripts you run directly, as `python app/<script>.py` from the repo root (or by name from inside `app/`):
  - `servo_server.py` — interactive console (also takes one-shot commands)
  - `init_helper.py` — first-time setup wizard
  - `clip_scan.py` / `calibrate_jacobian.py` — scan & calibration routines
  - `zmq_server.py` — the ZMQ server
  - `tui.py` — shared curses toolkit (no project imports), `_bootstrap.py` — path shim

  Every entry script does `import _bootstrap` **first**, which prepends the sibling `src/` dir to `sys.path` so the flat library imports (`from servodriver import Servoset`, …) resolve.
- **`src/`** — the importable library: `config.py`, `servodriver.py`, `servo_util.py`, `callback_functions.py`, `optimize.py`, `step_optimize.py`, `spiral.py`, `datastore.py`, `fit_gaussian.py`, `motor_scan.py`, `numeric_sim.py`, plus vendored code (`scservo_sdk/`, and the expctl stubs `ServerClass.py` / `sequence.py`). No library module imports an `app/` script — dependencies point app → src only, so the library stays importable on its own.
- **`config/`** — the two gitignored YAML config files and their checked-in templates. **`doc/`** — developer documentation. **`state/`** / **`data/`** — runtime state and task output (gitignored). **`example/`** — exploratory notebooks and FEETECH SDK examples.

## Hardware & import safety

Most of this code needs the real hardware to *do* anything, but importing is safe with two caveats:

- Every `src/` module imports cleanly without hardware. Real I/O starts at use: **constructing a `Servoset` opens the serial port and enables torque.**
- `app/clip_scan.py` and `app/calibrate_jacobian.py` construct a `Servoset` **at module level** — importing or running them moves motors. Don't import them to "check" them.
- `callback_functions.py` opens the I2C ADC at import, but **defensively**: when the ADC or the `smbus2`/`MCP342x` libs are absent, `_open_adc` falls back to `(None, None)` and `read_objective` yields NaN, so the module imports anywhere (on real hardware it does talk to the ADC).
- Pure modules that never touch hardware: `config.py`, `servo_util.py`, `spiral.py`, `fit_gaussian.py`, `numeric_sim.py`, `step_optimize.py`, `optimize.py`, `datastore.py`, `app/tui.py`. Note `config.py` (and everything importing it) needs PyYAML and the two YAML files present, and constructing a `datastore.DataStore` creates its run folder under `data/`.

There is no build, lint, or test suite. `src/spiral.py` and `src/numeric_sim.py` have `__main__` matplotlib demos — the only things runnable without hardware. The `example/notebooks/*.ipynb` are exploratory. On this Pi the project runs under the `expctl3` conda env (`~/anaconda3/envs/expctl3/bin/python`).

## Configuration

All machine- and setup-specific values live in **two gitignored YAML files** under `config/`, loaded once by `config.py` (exposed as module constants: `DEVICENAME_LIST`, `sts3032_dict`, `MASKS`, `STATE_FOLDER`, …):

- **`machine.yaml`** — per-Pi hardware/software: serial `devices`/`baudrate`, servo `speed`/`acc`, the `de_hysteresis` tuning, the `channels` map (list order = channel index, each `{id, name}`) and the channel-grouping `masks` that go with it, the `adc` I2C wiring (MCP3424), filesystem `paths` (`state_folder`, `data_folder`), and the ZMQ `server` (`name`/`port`/`board_id`; default port 60627).
- **`calibration.yaml`** — optics-setup/calibration-dependent: the beam-clip `accept_functions`, Jacobian `coupling_vectors`, and optimizer tuning (`spiral`, `bfgs`, `clip_scan`, `jacobian`).

`config.py` searches `$SERVO_ALIGNER_CONFIG_DIR`, then `<repo>/config`, then next to `config.py`; per-file overrides via `SERVO_ALIGNER_MACHINE_CONFIG` / `SERVO_ALIGNER_CALIB_CONFIG`. Relative `paths` resolve against the repo root. STS3032-fixed constants (control-table addresses; the `2048`/`4096` encoder geometry in `servo_util.py`) stay in code, not YAML.

**First-time setup** is easiest via the wizard `python app/init_helper.py`: it installs missing Python deps from `requirements.txt`, creates both YAML files from the templates (comment-preserving round-trip; needs `ruyaml` or `ruamel.yaml`), scans the serial bus to build the `servo.channels` map, and performs the register steps `doc/motor.md` otherwise does by hand in FEETECH's FD tool — assigning servo IDs, enabling hardware multi-turn (`Register 18 → 124`), setting the angle-limit registers. It talks to the bus directly through `scservo_sdk` (it does **not** import `config.py`/`servodriver`), so it is safe to run before the config files exist; the only action that moves a servo is "Identify a servo". `--no-deps` / `--no-bus` skip those steps. Or just copy the templates by hand:

```bash
cp config/machine.template.yaml     config/machine.yaml       # hardware / software
cp config/calibration.template.yaml config/calibration.yaml   # optics / calibration
```

**State vs data** (two separate concerns):

- Runtime state `servos_<board>.json` lives under `state_folder`, persisting the last-known encoder positions across restarts; it is rewritten on every move and at exit (`atexit`). `set_zero` also appends the pre-zero pose (with a `revert_position`) to `set_zero_history_<board>.jsonl`.
- Task **output** (scans, Jacobian datasets, plots) goes under `data_folder` through `datastore.py`: a `DataStore("<name>")` is one named run folder under `data/` — it **warns if that folder already exists** (a re-run may overwrite earlier results; pass `unique=True` for a fresh auto-numbered folder) and exposes uniform `save_npz`/`save_npy`/`save_fig`/`load_npz`/`load_npy`/`path`/`exists` helpers. `clip_scan.py` and `calibrate_jacobian.py` name their store from `output_subdir` (`calibration.yaml`); input paths (e.g. the assumed-Jacobian `.npz`) resolve via `datastore.resolve_under_data`.

## Commands

Run the entry scripts from the repo root as `python app/<script>.py`.

```bash
python app/init_helper.py          # first-time setup wizard (--no-deps / --no-bus skip steps)

# Standalone routines — these move motors immediately:
python app/clip_scan.py            # 2D raster scan of knob pairs, fit beam-clip center
python app/calibrate_jacobian.py   # spiral + L-BFGS-B coupling optimization, derive Jacobian
nohup python app/clip_scan.py > clip.log 2>&1 &    # long-running scan, detached

# Interactive console (control panel: monitor + objectives + manual controls + ZMQ on/off):
python app/servo_server.py
# one-shot manual commands (no TUI):
python app/servo_server.py set_zero                # set current pose as zero for all servos
python app/servo_server.py home                    # all servos to 0 deg
python app/servo_server.py set_angle 10 -5 0 0 0 0 0 0   # absolute angles, one per channel
python app/servo_server.py set_single 3 12.5       # one channel to an angle

python app/zmq_server.py           # run the ZMQ server directly (no console)
```

## How the code runs

1. **ZMQ server** — `app/zmq_server.py` (class `STSServer`) subclasses the vendored `ServerClass.Server` and plugs into the lab's external `expctl` experiment-control framework: it listens on a ZMQ REP socket, receives pickled `Sequence` objects (unpickled onto the minimal data shells in `ServerClass.py`), and drives servos to the requested angles during the `QUEUE` phase. Its `Servoset` is *injected*, and `main_loop(cond_fn)` polls every 10 ms re-checking `cond_fn`, so the console can run it in a background thread (sharing one serial connection) and stop it cleanly.
2. **Interactive console** — `app/servo_server.py`, a curses control panel:
   - a live **servo monitor** (select a servo and field with the arrow keys to set its angle or toggle torque);
   - an **objective page** listing `callback_functions.OBJECTIVES` with their live, NaN-safe values, where the active objective is selected;
   - **manual controls**: home, set-zero (double-confirmed), de-hysteresis toggle;
   - a **ZMQ server page** that starts/stops the server in a background thread — leaving the page keeps it running, and its logs/prints are captured into a ring buffer shown in a shared bottom log pane, never on the curses screen.

   The server and the console share one `Servoset`, which serialises every bus transaction with an `RLock` (per transaction, not per move), so the monitor and manual controls work **while the server is running** without their serial packets interleaving.
3. **Standalone scripts** — `app/clip_scan.py` and `app/calibrate_jacobian.py`, run directly (see Commands).

Everything runs standalone. (The code was previously deployed inside the lab's `expctl` framework; the trimmed `ServerClass.py` / `sequence.py` stubs under `src/` that back the ZMQ server are what remain of that path.)

## Core concepts (needed to read the optimization code)

- **Channel layout**: 8 servos = two beam paths × two mirrors × two knobs. Naming convention `A` = upper path, `B` = lower path; per path the four knobs are `x, y, xdot, ydot` (a position knob and an angle knob on each of two mirrors). Geometrically the knob pairs are coupled, which is the whole reason for numeric optimization.
- **`pos_mask`**: an 8-element 0/1 list selecting which channels a step acts on (e.g. `A_X_XDOT = [1,0,1,0,0,0,0,0]`). The masks and the `posmask2str` reverse lookup live in `config.py` (loaded from `machine.yaml`'s `servo.masks`, since they follow the channel wiring); `knob_mask(path, group)` looks one up; `posmask2acceptfunc` (in `clip_scan.py`) maps a mask to its accept function.
- **Vector notations in `servo_util.py`** — three representations the helpers convert between:
  - `r` = *reduced* vector, only the masked entries (length = `sum(mask)`)
  - `nr` = full-length **angle** vector in degrees
  - `nd` = full-length **encoder-count** position (`a2p`: angle→count; the `d` is encoder counts, not "digital"; `2048` = 0°, `4096`/turn)
  - Helpers: `r2nr`/`r2nd` (embed reduced into full), `nrselr` (extract masked), `nraddr`/`nrmodr`/`ndmodr` (add/overwrite masked entries).
- **`compose_para`** (`servo_util.py`): the heart of every objective. Builds a full angle command from a reduced parameter `para` on top of a `zero` offset, and — if a Jacobian is supplied — sets the *slave* knobs to follow the *master* knobs via `dB = J·(dA − offset)`. `jac_master_mask` picks which channels are master.
- **`callback_func`**: the optimizer objective — composes a full angle command (`compose_para`), moves the servos, reads an objective function, returns `(para, intensity)`. It is built by `make_callback_func(servos, objective)` in `callback_functions.py`, which also holds the MCP3424 ADC setup and the `OBJECTIVES` registry of selectable objective functions (`intensity_adc` is the hardware default; `dummy_gaussian` is a hardware-free synthetic landscape; `get_objective(name=None)` picks one by name or at random). `read_objective(func)` evaluates one defensively — NaN if the ADC is absent or a read fails — which the console's objective page uses for live values. `clip_scan.py` / `calibrate_jacobian.py` call `make_callback_func(servos, intensity_adc)`.
- **Optimization stack**: `step_optimize.py` holds two functions — `step_optimize` (one optimization stage; only commits the new origin if a re-measure confirms the gain) and `pts_iterator` (dispatches to `spiral`, `L-BFGS-B`, or `Powell`; spiral is 2D-only) — calling down to `spiral.py` (`SpiralPath`: a space-filling spiral whose center is dragged toward higher intensity — the custom "spiral descent" algorithm, see `doc/spiral.md`). Both take an `opt_type` (`"max"` / `"min"` / `"zero"`); `score_of` turns it into the value the optimizers always maximize (`v` / `-v` / `-abs(v)`, so `"zero"` drives the objective toward 0). `"max"` keeps the original positive-intensity ≥70%-of-best re-measure guard exactly; `"min"`/`"zero"` use a sign-safe score-space variant (commit if a re-read retains ≥`accept_frac` of the gain over the start). `optimize.py` sits on top: `optimize_knobs(servos, callback_func, zero, masks, opt_type=…)` is a reusable **template** that runs one `step_optimize` per knob mask (auto-picking `spiral` for 2-knob groups, `L-BFGS-B` otherwise), threading the origin through, and returns `(best_zero, value)`; `fiber_coupling(servos, callback_func, zero, path=…)` is the concrete fiber-coupling recipe (`X_Y → X_XDOT → Y_YDOT → joint L-BFGS-B over POS_ALL`) that `calibrate_jacobian.py` calls.
- **De-hysteresis** (`Servoset.set_position`): because the 3D-printed mount frame flexes, moves in the negative direction overshoot (default 100 encoder steps) then return, so backlash is always taken up from the same side. Toggle via `servos.de_hysterisis` (control-panel toggle; default from `machine.yaml`). Calibration accuracy depends on this being set correctly.
- **Multi-turn encoder**: a single turn is 0–4095. The servos are configured for **hardware multi-turn reporting** (register 18 → 124, done by `init_helper.py` or FEETECH's FD tool; ±~7 turns); `Servoset.get_position` decodes the reported position as bit-15 sign-magnitude, and the last-known positions persist in `servos_<board>.json`. Driver-board power loss resets the servo's turn count — re-zero/re-home before trusting absolute angles after a power cycle.

## Vendored / external

- `src/scservo_sdk/` — FEETECH's serial servo SDK, vendored. Don't edit; it's upstream.
- `src/ServerClass.py` — trimmed expctl framework code: the ZMQ `Server` base class plus the minimal `Sequence`/`Channel`/`Interval` data shells needed to unpickle the sequences clients send. `src/sequence.py` is only a re-export shim: clients pickle these classes from a top-level module named `sequence`, so that module name must stay importable — don't delete or rename it.
