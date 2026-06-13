# scan — raster scanning

| Module | Role |
|--------|------|
| [raster.py](raster.py) | `raster_1d` / `raster_2d`: scan an objective over knob coordinates. The 2D scan walks the grid in serpentine (zigzag) order to minimize knob travel, then un-shuffles the samples back into grid order. An optional `accept_func` skips points outside the knob-coupling band. Returns a `ScanResult(X, Y, Z)`. |

The scan only calls the objective — homing the actuator and closing on error
is the calling routine's responsibility, and plotting lives in
[../plotting.py](../plotting.py). Used by the clip-scan routine
([../routines/clip_scan.py](../routines/clip_scan.py)); see
[../../doc/application.md](../../doc/application.md) for the beam-clip method.
