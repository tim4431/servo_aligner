# Examples

- `notebooks/` — exploratory Jupyter notebooks (simulation, servo
  comparison). They predate the 2026 package refactor and use the old flat
  module names; the table below maps the imports if you want to re-run them.
- `scsservo_sdk_example/` — FEETECH's upstream SDK examples. The SDK is now
  vendored at `servo_aligner.scservo_sdk` (`from servo_aligner.scservo_sdk
  import *` instead of `from scservo_sdk import *`).
- `scsservo_sdk_source/` — the original vendor archive.

| Old import | New import |
|------------|------------|
| `import numeric_sim` | `from servo_aligner.sim import beam_model` (`calc_data`, `BeamClipModel`) |
| `from spiral import SpiralPath` | `from servo_aligner.optimize.spiral import SpiralPath` |
| `from fit_gaussian import …` | `from servo_aligner.fitting import …` |
| `from servo_util import …` | `from servo_aligner.vectors import …` |
| `from servodriver import Servoset` | `from servo_aligner.hal.sts3032 import Sts3032Actuator` (built via `servo_aligner.factory`) |
