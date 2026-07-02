# Servo Aligner — Documentation

STS3032 servos turn mirror-mount knobs to automate laser-beam alignment, with a
photodiode as feedback. Suggested reading order:

1. [usage.md](usage.md) — setup, ZMQ server, CLI, running the scripts
2. [mirror_mounts.md](mirror_mounts.md) — the optics premise: why two mirrors, and the two knob couplings to fight
3. [motor.md](motor.md) — servo wiring, registers, encoder, de-hysteresis
4. [spiral.md](spiral.md) — the 2D spiral-descent search
5. [optimize.md](optimize.md) — chaining spirals into a full round: 2D knob pairs, then 4D L-BFGS-B
6. [jacobian.md](jacobian.md) — calibrating the 4×4 coupling between the two beam paths
7. [application.md](application.md) — beam centering (clip scan) & MOT alignment
8. [simulation.md](simulation.md) — hardware-free models and the optimizer test bed

Component reference: [servo_server.md](servo_server.md) — the interactive
console (servo monitor, objective page, ZMQ server switch, one-shot CLI).

Config templates live in [`../config/`](../config/): [`machine.template.yaml`](../config/machine.template.yaml)
and [`calibration.template.yaml`](../config/calibration.template.yaml) (copy to
`config/machine.yaml` / `config/calibration.yaml`). Also here:
[servos_template.json](servos_template.json) (saved-position template) and
[files/](files/) (FEETECH vendor tools/manual).
