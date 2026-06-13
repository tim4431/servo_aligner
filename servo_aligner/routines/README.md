# routines — alignment procedures

The end-to-end procedures, exposed as functions (no import-time hardware I/O)
and wired to the CLI. Each builds its stack via [../factory.py](../factory.py)
and wraps the work in `try/finally` so the actuator is homed/closed even on
error.

| Module | Entry point | Role |
|--------|-------------|------|
| [clip_scan.py](clip_scan.py) | `run_clip_scan(cfg)` → `servo-aligner clip-scan` | Raster a knob pair, fit the beam-clip center (smooth-Heaviside), drive to it, and re-zero. Output `.npz`/`.png` filenames match the legacy scripts. See [../../doc/application.md](../../doc/application.md). |
| [calibrate_jacobian.py](calibrate_jacobian.py) | `run_jacobian_calibration(cfg)` → `servo-aligner calibrate-jacobian` | Offset one beam path, re-optimize the other through the configured stage sequence, and append `(offset, zero, intensity)` to a dataset whose finite differences give the knob-coupling Jacobian. See [../../doc/jacobian.md](../../doc/jacobian.md). |

All behavior is config-driven: scan stages / accept lines from
`cfg.clip_scan`, master path / offset type / stage sequence from
`cfg.jacobian`, optimizer tuning from `cfg.optimize`.
