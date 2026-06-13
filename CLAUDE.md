# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Controls FEETECH **STS3032** serial-bus servo motors that turn the adjuster knobs of optical mirror mounts, for **automated laser-beam alignment** in a Rydberg-atom cavity experiment. In production it runs on a Raspberry Pi (`rydpiservo`) wired Pi → URT/serial driver board → daisy-chained servos (addressed by ID), with a photodiode read over I2C (MCP3424 ADC) as the optimization feedback signal.

The physics background, alignment procedure, de-hysteresis rationale, Jacobian/optimization theory, and the "fat tail" beam-clipping model are documented in `doc/` (see `doc/README.md` for reading order). **The algorithms encode hard-won lab findings — restructure around them if needed, never "improve" the numerics.** The test suite pins them: `tests/golden/golden_values.json` holds outputs captured from the pre-refactor code (tag `pre-refactor`) and the parity tests assert exact equality.

## Architecture (post-2026 refactor)

Installable src-layout package, `pip install -e ".[dev]"`, entry point `servo-aligner` (= `python -m servo_aligner`). **Nothing performs hardware I/O at import time**; hardware drivers are imported lazily by `factory.py` based on the YAML config.

- `src/servo_aligner/config.py` — YAML → frozen dataclasses. One file per machine (`-c` flag → `$SERVO_ALIGNER_CONFIG` → `./servo_aligner.yaml`); template at `config/example_config.yaml`. Every tuning constant lives here: optimizer params, accept lines, scan stages, de-hysteresis counts, ADC settings, paths.
- `src/servo_aligner/channels.py` — `ChannelLayout`/`ChannelGroup`: named channel groups from config replace the old hardcoded 8-element masks; the group's `mask` tuple feeds the same vector helpers. N-channel generic. **The B-path naming quirk is intentional**: `B_X_Y` selects `B_xdot, B_ydot` and `B_XDOT_YDOT` selects `B_x, B_y` (mirrored vs. the A path) — faithful to the original wiring; do not "fix" without lab confirmation.
- `src/servo_aligner/hal/` — hardware abstraction. `interfaces.py` defines the `Actuator` and `IntensitySensor` protocols (the API speaks **degrees**). Backends: `sts3032.py` (`ScsBus` owns all scservo_sdk/serial usage; `Sts3032Servo` single servo incl. software multi-turn tracking; `Sts3032Actuator` group moves + de-hysteresis + persistence), `mcp3424.py` (ADC), `simulation.py` (pure-state actuator + photodiode simulated via the beam-clip physics — the whole stack runs hardware-free with `backend: simulation`).
- `src/servo_aligner/vectors.py` — the `r`/`nr`/`nd` vector helpers and `compose_para` (Jacobian master→slave composition, `dB = J(dA − offset) + x0`). Numerics verbatim from the legacy code.
- `src/servo_aligner/measurement.py` — `Measurement.measure/objective`: the single move-then-read callback (was duplicated per script); `JacobianCoupling` bundles the coupling arguments.
- `src/servo_aligner/optimize/` — `spiral.py` (`SpiralPath` spiral descent, 2D only), `iterate.py` (`iterate_points` dispatch to spiral/L-BFGS-B/Powell, returns `OptimizationTrace`), `step.py` (`step_optimize`: one stage, commits the new zero only if re-measured intensity ≥ `optimize.accept_ratio` (0.7) of the best seen).
- `src/servo_aligner/scan/raster.py` — serpentine (zigzag) 1D/2D scans; `fitting.py` — Gaussian/smooth-Heaviside fits; `plotting.py` — all matplotlib, lazy-imported (`plot` extra); `sim/beam_model.py` — `BeamClipModel` physics + legacy-parity `calc_data`.
- `src/servo_aligner/routines/` — `clip_scan.py` and `calibrate_jacobian.py` as functions (`run_clip_scan`, `run_jacobian_calibration`); output `.npz`/`.npy` formats and filenames unchanged from the legacy scripts.
- `src/servo_aligner/server/` — the optional expctl adapter (`server` extra): ZMQ REP on port 60627, protocol unchanged; `_vendor/sequence.py` is the vendored expctl module (needs expctl's `utilities` at runtime; `compat.py` maps pickled module aliases onto it). Core code never imports this package.
- `src/servo_aligner/scservo_sdk/` — FEETECH's SDK, vendored unmodified. Don't edit.

## Commands

```bash
.venv/bin/python -m pytest tests/        # hardware-free test suite (use MPLBACKEND=Agg)
servo-aligner -c <cfg.yaml> status|home|set-zero|set-angle …|set-single I A
servo-aligner -c <cfg.yaml> clip-scan [--no-plot]
servo-aligner -c <cfg.yaml> calibrate-jacobian [--master B] [--offset-type pm]
servo-aligner -c <cfg.yaml> server [--dehys 0|1]   # requires [server] extra
```

For dev work use a simulation-backend config (see `tests/conftest.py::write_sim_config` or set both backends to `simulation`); real-hardware behavior can only be smoke-tested on the Pi (`doc/migration.md` §6 has the checklist).

## Core concepts (needed to read the optimization code)

- **Channel layout**: in the lab, 8 servos = two beam paths (`A` upper, `B` lower) × two mirrors × two knobs (`x, y, xdot, ydot` per path). Geometric coupling between knob pairs is the reason for the numeric optimization. Channel order in `actuator.channels` defines the vector index everywhere.
- **Vector notations** (`vectors.py`): `r` = reduced vector (masked entries only), `nr` = full-length angles in degrees, `nd` = full-length encoder counts (`a2p`: angle→position, 2048 = 0°, 4096/turn). Reduced-vector order is channel-index order, regardless of how a group lists its members.
- **De-hysteresis** (`Sts3032Actuator.set_positions`): the 3D-printed mount frame flexes, so negative moves overshoot by `overshoot_counts` (100) then return — backlash is always taken up from the same side. Toggle via `actuator.de_hysteresis`; clip scans run with it off, Jacobian calibration with it on. Calibration accuracy depends on this.
- **Multi-turn encoder**: a single turn is 0–4095; `Sts3032Servo` tracks turn count in software (jump > 3500 ⇒ wraparound). Servo register 18 → 124 (set via FEETECH's FD tool) gives hardware multi-turn reporting (±7 turns); driver-board power loss resets the turn count.
- **Acceptance logic**: both `step_optimize` and the clip scan only commit a new zero after re-measuring it (`accept_ratio`, `clip_scan.I_meaningful`) — the beam signal is noisy and flat-topped; never bypass these checks.

## Gotchas

- `pyproject.toml` extras: `hardware` (smbus2/MCP342x/pyserial), `plot`, `server` (pyzmq/coloredlogs), `dev`. The test venv (`.venv/`) deliberately has **no** hardware/server extras — `tests/test_imports.py` and `tests/test_server_adapter.py` rely on their absence.
- Machine-local files are gitignored: the YAML config, `state_dir` contents, `servos_*.json`. Legacy `src/customize.py`/`src/servos_0.json` may still exist locally as untracked leftovers.
- `tests/golden/_capture_from_legacy.py` documents how the goldens were made; it can no longer run (the legacy modules are deleted) — never regenerate goldens from the *new* code, that would defeat their purpose.
- The simulator's `smooth_transition` option is a **test-only** knob (gives the optimizer gradients); the lab signal is the hard-edged variant (`null`).
