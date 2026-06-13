# optimize — the optimization engine

The lab-validated search algorithms. Tuning constants live in the YAML config
(`optimize.spiral`, `optimize.lbfgsb`, `optimize.accept_ratio`), not here.

| Module | Role |
|--------|------|
| [spiral.py](spiral.py) | `SpiralPath`: the custom 2D "spiral descent" maximizer — an outward space-filling spiral whose center is dragged toward higher intensity, restarting when a sample beats the running max. 2D only. See [../../doc/spiral.md](../../doc/spiral.md). |
| [iterate.py](iterate.py) | `iterate_points`: dispatch one search to `spiral` / `L-BFGS-B` / `Powell`, recording every sample into an `OptimizationTrace` (plotting is handled separately by [../plotting.py](../plotting.py)). |
| [step.py](step.py) | `step_optimize`: run one stage on a channel group from the current `zero`, then commit the result as the new zero only if the re-measured intensity stays ≥ `accept_ratio` (default 0.7) of the best seen. |

How stages chain into a full alignment round: [../../doc/optimize.md](../../doc/optimize.md).

Unlike the legacy `pts_iterator`, exceptions propagate to the caller (which
homes/closes hardware in a `finally`) rather than being swallowed.
