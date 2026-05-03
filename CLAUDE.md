# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repo layout

```
servo_aligner/
├── CLAUDE.md, LICENSE, .gitignore
├── src/                            # flat module set; src/ goes on PYTHONPATH
│   ├── STSServer.py, servodriver.py, customize.py, servos_0.json
│   ├── ServerClass.py              # base ZMQ REP server
│   ├── sequence.py                 # imported by ServerClass for unpickling
│   ├── SequenceProcessor.py        # FPGA sequence post-processor (now unused by STSServer)
│   ├── scservo_sdk/                # vendored Feetech SDK — only real subpackage in src/
│   └── (alignment scripts: clip_scan, calibrate_jacobian, motor_scan,
│        pts_iterator, spiral, step_optimize, fit_gaussian, servo_util,
│        servo_const, pd, callback_func, numeric_sim, plot_csv, sync_read_write)
├── doc/      README.md, instructions.txt, customize.template.py, servos_template.json, setup.py
├── example/  scsservo_sdk_example/, scsservo_sdk_source/, notebooks/
└── data/     *.log, *.csv, *.png — runtime/output artifacts
```

There is no top-level package — every `.py` in `src/` is imported by bare name (`from servodriver import Servoset`, `from scservo_sdk import *`). The only nested package is the vendored [src/scservo_sdk/](src/scservo_sdk/), which keeps its internal `from .xxx import *` style.

## Common commands

`src/` must be on `PYTHONPATH`. Either:

```bash
cd src && python STSServer.py
# or
PYTHONPATH=src python -m STSServer
```

Documented entry points (from [doc/instructions.txt](doc/instructions.txt) — note the old `python -m expctl.servers.servoaligner.*` paths there are stale, modules are top-level now):

```bash
# Calibrate the Jacobian linking master/slave servo axes
cd src && python calibrate_jacobian.py

# 2-D scan + Gaussian fit to recover the beam centroid
cd src && python clip_scan.py

# Backgrounded scan with logging
cd src && nohup python clip_scan.py > ../data/clip_cooldown_YYYYMMDD.log 2>&1 &
```

ZMQ server CLI (from [src/STSServer.py:91-135](src/STSServer.py#L91-L135)):

```bash
cd src && python STSServer.py            # bind tcp://*:60627, then enter main_loop
cd src && python STSServer.py set_zero
cd src && python STSServer.py home
cd src && python STSServer.py set_angle 10 -5 0 0 0 0 0 0
cd src && python STSServer.py set_single 3 12.5
cd src && python STSServer.py dehys 0|1
```

There are no tests to run.

## Important: `sequence.py` still references a missing `utilities` package

[src/sequence.py:1,9](src/sequence.py#L1) starts with `from utilities.util import *` and `from utilities import jGlobals`. There is no `utilities/` here — these were originally `..utilities` (climbing into a wider `expctl` repo) and are now bare absolute imports, but the module is still missing. `sequence.py` will not import on its own.

`ServerClass.py` does `import sequence`, which is needed only so pickled `Sequence` objects can be deserialized when a `SEQ` ZMQ message arrives. If you only use the argparse CLI subcommands (or only `PING`/`RUN`), you can remove the `import sequence` line in [src/ServerClass.py:8](src/ServerClass.py#L8) and delete `sequence.py` entirely. Otherwise, supply a `utilities/` package on the `PYTHONPATH`.

## Architecture

### Two cooperating layers

1. **Server layer**
   - [src/ServerClass.py](src/ServerClass.py) — base `Server` with the REP socket, command dispatcher, and four protocol verbs: `PING`, `SEQ` (load a `Sequence`), `QUEUE` (arm/execute), `RUN`, `GETPLOTDATA`. Subclasses override `cmd_seq`, `queue`, `run`, `plotdata`.
   - [src/STSServer.py](src/STSServer.py) — concrete `STSServer` for the 8-channel Feetech STS3032 servo set. `cmd_seq` stores the sequence; `queue` calls `set_angle`, which pulls the first transformed value from each channel and writes it via `Servoset.set_angle`. The same script is also the CLI front-end (argparse subcommands wired before `main_loop`).

2. **Sequence layer**
   - [src/sequence.py](src/sequence.py) — `Sequence` holds an `allChannels` array; channels carry `_UserValues` and `_TransValues` (4-tuples of `[startTime, startVal, stopTime, stopVal]`). Currently won't import on its own (see note above).
   - [src/SequenceProcessor.py](src/SequenceProcessor.py) — post-processes a sorted interval list (`FixHoles` → `FixImplicitSteps` → `FixExplicitSteps` → `ImposeIF`) to feed an FPGA. The flatten removed the dead `from ..util.SequenceProcessor import *` line in `STSServer.py`, so this file is now genuinely orphan — keep only if a client still imports it, otherwise delete.

### Servo driver internals

[src/servodriver.py](src/servodriver.py) is the heart of the runtime:

- `sts3032` wraps a single servo (set_position polls `ADDR_STS_PRESENT_POSITION` + `ADDR_STS_MOVING_STATUS`, tracks `turn_num` for multi-revolution motion).
- `Servoset` opens the USB port (tries each entry in `DEVICENAME_LIST` from [src/customize.py](src/customize.py) until one works), creates one `sts3032` per channel, registers `GroupSyncRead`/`GroupSyncWrite` for batched I/O, and persists the last commanded position to `servos_<board_id>.json` via `atexit.register(self.save)`.
- **De-hysteresis logic** ([servodriver.py:355-381](src/servodriver.py#L355-L381)): when `de_hysterisis=True` and any motor moves in the negative direction by more than 2 ticks, it first commands `goal-100` and then `goal`. This is part of the alignment contract — turn it off only knowing the consequences.
- Position units: 4096 ticks per 360°, with 2048 = 0°. The `angle_to_position`/`position_to_angle` helpers are the single source of truth for the conversion.

Channel-to-hardware-ID mapping lives in `sts3032_dict` in [src/customize.py](src/customize.py) (paired stages named `1x/1y … 4y` for two optical paths "A" and "B").

### Vendored SDK

[src/scservo_sdk/](src/scservo_sdk/) is the upstream Feetech Python SDK (its README is at [doc/README.md](doc/README.md); the original archive and reference examples live under [example/scsservo_sdk_example/](example/scsservo_sdk_example/) and [example/scsservo_sdk_source/](example/scsservo_sdk_source/)). Don't edit the vendored SDK — `servodriver.py` is the wrapper. Only `from scservo_sdk import *` ([servodriver.py:6](src/servodriver.py#L6)) is the supported entry point.

### Alignment / scan stack

The optimization scripts share the same coordinate plumbing in [src/servo_util.py](src/servo_util.py):

- `compose_para(para, pos_mask, zero, jac, jac_master_mask, ...)` — turns a low-dimensional optimization vector into the full 8-element angle command, applying a `zero` offset and (optionally) a Jacobian that drives slave motors from the masters.
- `r2nd / r2nr / nrselr / nrmodr / nraddr / ndmodr` — embed/extract values into the masked positions. The mask vocabulary (`A_X_XDOT_MASK`, `B_POS_ALL_MASK`, etc.) is enumerated in [src/servo_const.py](src/servo_const.py).

Pipeline:

- [src/pd.py](src/pd.py) provides `MCP3424_fiber`, an I²C ADC photodiode reader on `SMBus(1)` at `0x68` — this is the cost function's input.
- [src/motor_scan.py](src/motor_scan.py) walks a 1-D / 2-D grid in zigzag order, calling a user `callback_func(para)` that internally calls `servos.set_angle(...)` and returns `(para, intensity)`.
- [src/pts_iterator.py](src/pts_iterator.py) wraps `scipy.optimize.minimize` (L-BFGS-B / Powell) and a custom spiral search ([src/spiral.py](src/spiral.py)).
- [src/step_optimize.py](src/step_optimize.py) glues a `Servoset` + `callback_func` + a chosen pos_mask into a single optimization step that returns an updated `zero`.
- [src/calibrate_jacobian.py](src/calibrate_jacobian.py) sweeps offsets and calls `step_optimize` repeatedly to recover the master→slave Jacobian for one optical path (`MASTER = "A"` or `"B"`).
- [src/clip_scan.py](src/clip_scan.py) is the production scan: 2-D scan over a pos_mask, fit Gaussian via [src/fit_gaussian.py](src/fit_gaussian.py), update zero, repeat across masks.

### Configuration that has to exist

- [src/customize.py](src/customize.py) is required (`from customize import *` in [servodriver.py:14](src/servodriver.py#L14)). It defines `HOME_FOLDER`, `DEVICENAME_LIST`, `BAUDRATE`, `SERVO_SPEED`, `SERVO_ACC`, `sts3032_dict`. The canonical schema is in [doc/customize.template.py](doc/customize.template.py) — never delete `customize.py` without first preserving the values, and never check in edits that hard-code one machine's paths.
- `servos_<board_id>.json` is auto-created next to `HOME_FOLDER`; only [doc/servos_template.json](doc/servos_template.json) should be tracked.

## Things worth knowing before changing code

- **`customize.py` and `servos_0.json` are committed despite being listed in [.gitignore](.gitignore).** Treat them as machine-local — don't propagate one workstation's USB ports / baud rate into the repo. Note that the live [src/customize.py](src/customize.py) uses `BAUDRATE = 115200` while [doc/customize.template.py](doc/customize.template.py) has the SDK default `1000000`; this divergence is intentional for this rig but confuses fresh checkouts.
- **Notebooks and large data artifacts.** Multi-MB logs/CSVs/PNGs and exploratory notebooks (`servocomp.ipynb` ≈ 3 MB, `clip_heatup_*.log` ≈ 6 MB) used to live next to the source; they're now under [data/](data/) and [example/notebooks/](example/notebooks/). Treat them as output, not source.
- **Orphan modules.** [src/callback_func.py](src/callback_func.py) references undefined `servos` and `MCP3424_fiber` at module scope — it's dead code; the working `callback_func` is duplicated inline in [clip_scan.py:29](src/clip_scan.py#L29) and [calibrate_jacobian.py:26](src/calibrate_jacobian.py#L26). [pd.py](src/pd.py) also runs `convert_and_read()` and `print`s at import time — importing it touches the I²C bus. [SequenceProcessor.py](src/SequenceProcessor.py) is no longer imported by anything in `src/` after the flatten.
- **Redundant torque enable.** `STSServer.__init__` ([STSServer.py:19](src/STSServer.py#L19)) calls `torques_enable()` after `Servoset.__init__` already called `torque_enable()` per servo inside `refresh()`. Harmless but a hint not to add similar layered-init logic.
- **Blocking polls.** `sts3032.set_position` ([servodriver.py:111-138](src/servodriver.py#L111-L138)) loops up to 5000 times reading status registers; `Servoset._set_position` uses a `self.timeout`-bounded loop instead. Be aware which one you're calling.
- **Bare `except: pass`** in [STSServer.py:130-131](src/STSServer.py#L130-L131) swallows argparse errors — a real argparse failure (e.g., an unknown subcommand) silently falls through to `main_loop`.
- **Hardware assumptions.** The servo path requires running on the rig's Raspberry Pi (Linux `/dev/ttyUSB*`, `smbus2`, `MCP342x`). On Windows you can still load and reason about the code but `import servodriver` will fail at the `smbus2` / `MCP342x` import.
- **Stale paths in [doc/instructions.txt](doc/instructions.txt).** The file still lists `python -m expctl.servers.servoaligner.*` invocations from before the flatten — see "Common commands" above for the current form.
