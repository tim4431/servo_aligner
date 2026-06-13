# Servo Aligner — Documentation

STS3032 servos turn mirror-mount knobs to automate laser-beam alignment, with a
photodiode as feedback. Suggested reading order:

1. [usage.md](usage.md) — install, YAML config, CLI, ZMQ server
2. [motor.md](motor.md) — servo wiring, registers, encoder, de-hysteresis
3. [spiral.md](spiral.md) — the 2D spiral-descent search
4. [optimize.md](optimize.md) — chaining spirals into a full round: 2D knob pairs, then 4D L-BFGS-B
5. [jacobian.md](jacobian.md) — calibrating the 4×4 coupling between the two beam paths
6. [application.md](application.md) — beam centering (clip scan) & MOT alignment
7. [simulation.md](simulation.md) — hardware-free models and the optimizer test bed
8. [migration.md](migration.md) — moving a pre-refactor Pi deployment to the package layout

Also here: [servos_template.json](servos_template.json) (state-file template;
the machine config template is [../config/example_config.yaml](../config/example_config.yaml)),
[files/](files/) (FEETECH vendor tools/manual).
