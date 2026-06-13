# sim — beam-clip physics model

| Module | Role |
|--------|------|
| [beam_model.py](beam_model.py) | `BeamClipModel`: transmitted intensity of a laser beam steered by two mirrors (four knobs: x, y, xdot, ydot) through a pair of apertures, including the geometric knob-pair crosstalk that motivates the Jacobian optimization. `calc_data(...)` is a byte-for-byte port of the legacy `numeric_sim` 2D-slice scan (keeps the notebooks reproducible). |

This is the physics the simulated photodiode
([../hal/simulation.py](../hal/simulation.py)) evaluates, so the entire
alignment stack runs hardware-free. Constructor parameters (aperture size,
lever arms, knob pitch, crosstalk matrix) are exposed via the YAML
`simulation.model` section.

The `smooth_transition` option replaces the hard aperture edge with an
erfc-smoothed one so gradient-based tests converge — it is a **test-only**
knob, not lab physics (the real signal is hard-edged). Background:
[../../doc/simulation.md](../../doc/simulation.md).
