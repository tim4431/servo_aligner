# servo_aligner

Algorithm and designs for controlling STS3032 servo motors, for automatic beam alignment.

<img src="doc/figs/servo_aligner_assembly.png" alt="servo aligner assembly" width="400">

**Features:**
- Packaged servo control [api](src/servo_driver.py).
- [Spiral search](doc/spiral.md) based low-dimensional walking of coupled mirror knobs.
- Robust automated alignment with a model-free jacobian.



Full documentation (setup, hardware, the spiral/Jacobian optimization stack, applications) lives in [doc/](doc/README.md).