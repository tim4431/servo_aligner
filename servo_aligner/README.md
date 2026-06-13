# servo_aligner

The application package. Layered so nothing here touches hardware at import
time — hardware drivers are imported lazily by [factory.py](factory.py)
according to the YAML config.

## Top-level modules

| Module | Role |
|--------|------|
| [config.py](config.py) | YAML → typed frozen dataclasses; resolution via `-c` / `$SERVO_ALIGNER_CONFIG` / `./servo_aligner.yaml`. |
| [channels.py](channels.py) | `ChannelLayout` / `ChannelGroup`: named channel groups + masks derived from config (replaces the old hardcoded `*_MASK` constants). |
| [vectors.py](vectors.py) | The `r`/`nr`/`nd` masked-vector helpers and `compose_para` (Jacobian master→slave composition). |
| [measurement.py](measurement.py) | `Measurement` — the single move-then-read callback; `JacobianCoupling`. |
| [fitting.py](fitting.py) | 2D Gaussian / smooth-Heaviside fits for scan data. |
| [factory.py](factory.py) | Build the actuator / sensor / measurement stack from a `Config`. |
| [plotting.py](plotting.py) | All matplotlib rendering (lazy import; `plot` extra). |

## Subpackages

| Package | Role |
|---------|------|
| [hal/](hal/) | Hardware abstraction: `Actuator`/`IntensitySensor` protocols, STS3032 + MCP3424 + simulation backends. |
| [optimize/](optimize/) | Spiral descent, optimizer dispatch, staged step-optimize. |
| [scan/](scan/) | Serpentine 1D/2D raster scans. |
| [sim/](sim/) | Beam-clip physics model used by the simulation sensor. |
| [routines/](routines/) | The clip-scan and Jacobian-calibration procedures. |
| [cli/](cli/) | The `servo-aligner` command-line interface. |
| [server/](server/) | Optional ZMQ adapter for the expctl lab framework (`server` extra). |
| [scservo_sdk/](scservo_sdk/) | Vendored FEETECH serial SDK (unmodified). |

Dependency direction: `routines`/`cli` → `factory` → `hal`; the algorithm
packages (`optimize`, `scan`, `fitting`, `sim`) depend only on numpy/scipy
and never import `hal`. See [../CLAUDE.md](../CLAUDE.md) for the architecture
and core concepts, and [../doc/](../doc/README.md) for the physics.
