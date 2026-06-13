# config — machine configuration

One YAML file per machine holds every setting (serial ports, channel→servo
map, ADC, named channel groups, optimizer tuning, scan stages, output dirs).
It replaces the legacy gitignored `customize.py`.

| File | Purpose |
|------|---------|
| [example_config.yaml](example_config.yaml) | Fully annotated reference mirroring the rydpiservo production setup — read this to understand every key. |
| [servo_aligner.template.yaml](servo_aligner.template.yaml) | Terse, copy-ready template with `FIXME` markers on the machine-specific values. |

## Usage

```bash
cp config/servo_aligner.template.yaml ~/servo_aligner.yaml   # then edit the FIXMEs
export SERVO_ALIGNER_CONFIG=~/servo_aligner.yaml             # or pass -c per command
```

Resolution order: `--config` flag → `$SERVO_ALIGNER_CONFIG` →
`./servo_aligner.yaml`. The loader ([../servo_aligner/config.py](../servo_aligner/config.py))
validates the file into typed dataclasses and rejects unknown keys.

Your actual machine config (`servo_aligner.yaml`) is gitignored — only the
example and template are tracked. For the full key reference and the
old→new mapping from `customize.py`, see
[../doc/usage.md](../doc/usage.md) and [../doc/migration.md](../doc/migration.md).
