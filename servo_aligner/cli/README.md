# cli — command-line interface

| Module | Role |
|--------|------|
| [main.py](main.py) | The `servo-aligner` entry point (also reachable as `python -m servo_aligner`). |

Every command loads the YAML config (`-c` / `$SERVO_ALIGNER_CONFIG` /
`./servo_aligner.yaml`) and builds the hardware — or simulation — stack
explicitly; nothing moves at import time.

```
servo-aligner [-c CONFIG] [-v] <command>

  status              read and print all channel angles
  home                move all channels to 0°
  set-zero            define the current pose as zero
  set-angle A1 …      absolute angles (deg) for all channels
  set-single I A      move one channel
  clip-scan           raster knob pairs, fit clip center, re-zero  [--no-plot]
  calibrate-jacobian  offset one path, re-optimize the other       [--master, --offset-type, --norm, -n]
  server              run the expctl ZMQ server (requires [server] extra) [--dehys 0|1]
```

The heavy routine/server imports are deferred into the command handlers, so
`servo-aligner status` on a sim config never imports matplotlib or pyzmq.
