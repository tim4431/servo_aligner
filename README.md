# servo_aligner

Automated laser-beam alignment: FEETECH **STS3032** serial-bus servos turn
the adjuster knobs of optical mirror mounts, with a photodiode (MCP3424 ADC)
as the optimization feedback. Built for a Rydberg-atom cavity experiment on
a Raspberry Pi; structured as a hardware-abstracted Python package so it
ports to other devices.

## Install & run

```bash
pip install -e ".[hardware,plot]"        # on the Pi (add [server] for expctl)
cp config/example_config.yaml ~/servo_aligner.yaml   # one YAML per machine
export SERVO_ALIGNER_CONFIG=~/servo_aligner.yaml

servo-aligner status
servo-aligner clip-scan                  # beam-centering scan + fit + re-zero
servo-aligner calibrate-jacobian         # knob-coupling calibration
servo-aligner server                     # ZMQ server for the expctl framework
```

No hardware? Set `actuator.backend: simulation` and `sensor.backend:
simulation` in the config and the entire stack runs against a beam-clip
physics model — which is also how the test suite works:

```bash
pip install -e ".[dev]" && pytest
```

## Layout

| Path | Role |
|------|------|
| `servo_aligner/hal/` | Hardware abstraction: `Actuator`/`IntensitySensor` protocols; STS3032, MCP3424, and simulation backends |
| `servo_aligner/optimize/`, `scan/` | The lab-validated algorithms: spiral descent, staged optimization, serpentine raster |
| `servo_aligner/routines/` | Clip-scan and Jacobian-calibration procedures |
| `servo_aligner/server/` | Optional ZMQ adapter for the expctl lab framework |
| `servo_aligner/scservo_sdk/` | Vendored FEETECH serial SDK (do not edit) |
| `config/example_config.yaml` | Fully commented machine config template |
| `tests/` | Hardware-free suite incl. golden-value parity with the pre-refactor code |

Full documentation — setup, hardware, the spiral/Jacobian optimization
stack, physics — lives in [doc/](doc/README.md). Migrating an old
deployment: [doc/migration.md](doc/migration.md).
