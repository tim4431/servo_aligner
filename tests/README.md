# tests

Hardware-free pytest suite. Run with `pip install -e ".[dev]" && pytest`
(set `MPLBACKEND=Agg` to keep matplotlib headless).

## What's covered

| Test file | Focus |
|-----------|-------|
| `test_vectors.py`, `test_compose_para.py` | masked-vector helpers + `compose_para`, asserted equal to the legacy numerics |
| `test_channels.py`, `test_config.py` | channel-group masks vs. the legacy constants; YAML loading + validation |
| `test_spiral.py`, `test_optimize_stack.py` | spiral trajectory + `iterate_points`/`step_optimize` (spiral & L-BFGS-B), accept/reject logic |
| `test_fitting.py`, `test_beam_model.py` | Gaussian/heaviside fits and the beam-clip physics, vs. legacy |
| `test_sts3032_actuator.py` | de-hysteresis sequencing, masking, multi-turn wraparound, persistence (via `fake_bus.py`) |
| `test_raster.py` | zigzag un-shuffle and accept-filter |
| `test_simulation_e2e.py`, `test_routines_cli.py` | config → factory → simulated stack; both routines and the CLI end-to-end |
| `test_imports.py`, `test_server_adapter.py` | import hygiene (no hardware libs pulled in) and server-adapter isolation |

## Support files

- `conftest.py` — the `golden` fixture and `write_sim_config()` helper (a
  simulation-backend config builder).
- `fake_bus.py` — `FakeScsBus`, a serial-bus double that records every write
  and teleports servos to their goal (no hardware).
- [`golden/`](golden/) — golden values captured from the pre-refactor code.

The test venv deliberately has **no** `hardware`/`server` extras installed —
`test_imports.py` and `test_server_adapter.py` rely on their absence.
