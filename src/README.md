# servo_aligner — `src/`

The **importable library**: everything here is imported by flat name (`from
servodriver import Servoset`) by the entry-point scripts in [`../app/`](../app/),
which prepend this directory to `sys.path` via `app/_bootstrap.py`. Dependencies
point one way only — `app/` imports `src/`, never the reverse — so this tree
stays importable on its own.

Configuration lives in two gitignored YAML files under [`../config/`](../config/)
(copied from the checked-in templates), loaded once by [`config.py`](config.py).
First-time setup: `python app/init_helper.py`. See [`../CLAUDE.md`](../CLAUDE.md)
for the architecture and core concepts, and [`../doc/`](../doc/README.md) for the
physics and algorithm rationale.

## Layout

| File | Role |
|------|------|
| `config.py` (+ `../config/*.yaml`) | Loads the two YAML config files (machine hardware/software vs. optics/calibration) and exposes them as constants. Edit the YAML, not the code, when migrating. |
| `servodriver.py` | `sts3032` (single motor) and `Servoset` (the motor set): connect, move, de-hysteresis, position persistence. Clean hardware library — no app, no TUI. Import-safe; **constructing a `Servoset` opens the serial port and enables torque.** |
| `servo_util.py` | The `r`/`nr`/`nd` vector helpers and `compose_para`; channel masks live in `config.py`. Pure. |
| `callback_functions.py` | Optimizer objectives: the MCP3424 photodiode ADC (over I2C), the `OBJECTIVES` registry (`intensity_adc`, `dummy_gaussian`, …), and `make_callback_func(servos, objective)` that builds the move-then-measure `callback_func`. Imports defensively — reads return NaN when the ADC is absent. |
| `optimize.py` / `step_optimize.py` / `spiral.py` | Optimization stack, top down: `optimize_knobs` / `fiber_coupling` (staged sequences) → `step_optimize` / `pts_iterator` (one stage; optimizer dispatch) → `SpiralPath` (the custom 2D spiral-descent search). |
| `motor_scan.py` / `fit_gaussian.py` | 1D/2D raster scans of the objective; Gaussian & beam-clip (smooth-heaviside) fits of the resulting maps. |
| `datastore.py` | `DataStore`: one named run folder per task under `data/`, with uniform save/load helpers. |
| `numeric_sim.py` | Hardware-free geometric simulation of the coupled-knob clip scans (see `doc/simulation.md`). |
| `scservo_sdk/` | Vendored FEETECH serial-servo SDK (do not edit). |
| `ServerClass.py` / `sequence.py` | Trimmed expctl framework code: the ZMQ `Server` base class plus the minimal `Sequence`/`Channel`/`Interval` data shells needed to unpickle client sequences. `sequence.py` is only a re-export shim so pickles referencing module `sequence` resolve — don't delete or rename it. |

## Hardware caveat

Every module here **imports** cleanly without hardware, but real I/O starts at
object construction / use: `Servoset(...)` opens the serial bus and enables
torque; the ADC objective reads I2C (NaN without it). The pure modules —
`config`, `servo_util`, `spiral`, `fit_gaussian`, `numeric_sim`, `step_optimize`,
`optimize`, `datastore` — never touch hardware at all (`spiral.py` and
`numeric_sim.py` have `__main__` matplotlib demos runnable on any machine).
