# hal — hardware abstraction layer

The alignment stack talks to hardware only through two protocols, so backends
are swappable (real servos/ADC on the Pi, the simulator on a dev machine, or
a future device). The API speaks **degrees**; encoder counts are an STS3032
implementation detail.

| Module | Role |
|--------|------|
| [interfaces.py](interfaces.py) | `Actuator` and `IntensitySensor` `typing.Protocol`s — the contract every backend implements. |
| [persistence.py](persistence.py) | `PositionStore`: load/save servo encoder positions as JSON (format unchanged from the legacy `servos_<board>.json`). |
| [sts3032.py](sts3032.py) | FEETECH backend: `ScsBus` (owns the serial port + all `scservo_sdk` calls), `Sts3032Servo` (one servo, software multi-turn tracking), `Sts3032Actuator` (group moves, de-hysteresis sequence, persistence). |
| [mcp3424.py](mcp3424.py) | `Mcp3424Sensor`: photodiode behind an MCP3424 I2C ADC (bus opened in the constructor, not at import). |
| [simulation.py](simulation.py) | `SimulatedActuator` (pure state + command log) and `SimulatedIntensitySensor` (reads the [../sim/](../sim/) beam-clip physics) — the whole stack runs hardware-free. |

`sts3032` and `mcp3424` need the `hardware` extra (smbus2 / MCP342x /
pyserial), imported lazily so this package stays importable everywhere.

**Porting to other hardware:** implement the two protocols in
[interfaces.py](interfaces.py) and add a branch in
[../factory.py](../factory.py). Channel count and grouping are entirely
config-driven, so no algorithm code changes.

The motion numerics (de-hysteresis overshoot, multi-turn wraparound, goal
encoding) are lab-validated — do not change them.
