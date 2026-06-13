# Migrating the Pi deployment to the refactored package

The 2026 refactor turned the flat `src/` script tree into an installable
package (`servo_aligner`) with a YAML config, a `servo-aligner` CLI, and a
hardware abstraction layer. The alignment algorithms (spiral descent,
de-hysteresis, Jacobian composition, acceptance thresholds, fits) are
**numerically unchanged** — the test suite asserts exact parity against
golden values captured from the old code. The last commit of the old layout
is tagged **`pre-refactor`** (rollback: `git checkout pre-refactor`).

## 1. Install on the Pi

```bash
git clone <repo> servo_aligner && cd servo_aligner
python -m venv ~/.venvs/servo-aligner
~/.venvs/servo-aligner/bin/pip install -e ".[hardware,server,plot]"
```

The package no longer needs to live inside the expctl tree. The expctl
*client* needs no changes (same ZMQ port and message protocol), but the
*server* still needs expctl's `utilities` package importable to unpickle
`Sequence` payloads — run it in an environment where expctl is installed.

## 2. Build your config

```bash
cp config/example_config.yaml /home/rydpiservo/servo_aligner.yaml
# then either pass -c on every call or set, in ~/.profile / the systemd unit:
export SERVO_ALIGNER_CONFIG=/home/rydpiservo/servo_aligner.yaml
```

Mapping from the old `customize.py` (and the constants that used to be
hardcoded in the scripts):

| Old | New (YAML) |
|-----|------------|
| `HOME_FOLDER` (servos_0.json location) | `state_dir` |
| `DEVICENAME_LIST` | `actuator.ports` |
| `BAUDRATE` | `actuator.baudrate` — **the deployed customize.py said `115200`, the template said `1000000`; copy the value the driver board actually uses** |
| `SERVO_SPEED` / `SERVO_ACC` | `actuator.speed` / `actuator.acc` |
| `sts3032_dict` `{idx: [ID, name]}` | `actuator.channels` (ordered list; order = channel index) |
| `servo_const.py` masks | `groups` (named channel lists; B-path quirk preserved) |
| clip_scan `FOLDER = /home/rydpiservo/servodata/servorecover5/` | `clip_scan.output_dir` |
| clip_scan accept-line slopes/tolerances | `clip_scan.accept_lines` |
| clip_scan `__main__` stage list / `I_meaningful` | `clip_scan.stages` / `clip_scan.I_meaningful` |
| jacobian dataset path `/home/rydpiservo/servodata/servosetup5/` | `jacobian.output_dir` |
| `N`, `normd`, `offset_type`, `MASTER` in calibrate_jacobian.py | `jacobian.n_iterations` / `norm` / `offset_type` / `master` |
| `spiral_params` / `BFGS_params` in step_optimize.py | `optimize.spiral` / `optimize.lbfgsb` |
| 70% commit threshold | `optimize.accept_ratio` |
| de-hysteresis −100 overshoot / threshold 2 | `actuator.de_hysteresis.*` |
| MCP3424 address/channel/gain/resolution, 2-read average | `sensor.*` |

## 3. Migrate state

The JSON format is unchanged — copy the last-known positions file:

```bash
mkdir -p ~/servo_aligner/state   # whatever state_dir you configured
cp /home/rydpiservo/expctl/src/expctl/servers/servoaligner/servos_0.json \
   ~/servo_aligner/state/
```

## 4. Command translation

| Old | New |
|-----|-----|
| `python -m expctl.servers.servoaligner.STSServer` | `servo-aligner server` |
| `python ...STSServer set_zero` | `servo-aligner set-zero` |
| `python ...STSServer home` | `servo-aligner home` |
| `python ...STSServer set_angle 10 -5 ...` | `servo-aligner set-angle 10 -5 ...` |
| `python ...STSServer set_single 3 12.5` | `servo-aligner set-single 3 12.5` |
| `python ...STSServer dehys 0\|1` | `servo-aligner server --dehys 0\|1` (or `server.de_hysteresis_on_start` in YAML) |
| `python clip_scan.py` | `servo-aligner clip-scan` (`--no-plot` headless) |
| `python calibrate_jacobian.py` | `servo-aligner calibrate-jacobian [--master B] [--offset-type zero]` |
| `nohup python -m ...clip_scan > clip.log 2>&1 &` | `nohup servo-aligner clip-scan --no-plot > clip.log 2>&1 &` |
| (new) read positions | `servo-aligner status` |

`python -m servo_aligner <command>` is equivalent to `servo-aligner <command>`.

## 5. Behavior changes to be aware of

- **Position JSON is saved once per completed move** (plus at exit/close),
  not on every poll read — the old code rewrote the file hundreds of times
  per move (SD-card wear). Power-loss semantics are unchanged.
- **Errors raise instead of hanging or being swallowed**: serial connect
  failure raises (the old code prompted for a key press), optimizer
  exceptions propagate (routines still home/close in `finally`), and a
  wrong-length angle list raises `ValueError`.
- **Plots**: scans fit and save results regardless; figures only with
  matplotlib installed and without `--no-plot`.
- **Spelling**: `de_hysterisis` → `de_hysteresis` everywhere in code.
- **No import-time hardware I/O anywhere** — importing any module is safe;
  motors only move via CLI commands or explicit API calls.

## 6. Pi smoke-test checklist (hardware required)

1. `servo-aligner status` — positions read back, names/IDs match the wiring.
2. `servo-aligner home`, then `servo-aligner set-single 0 10` — watch the
   right knob move; degree convention unchanged.
3. Negative move with de-hysteresis on: knob dips ~100 counts then returns.
4. Old `servos_0.json` loads from the new `state_dir` (step 3 above).
5. Short clip-scan stage (small `n_pts` in a test config) — `.npz`/`.png`
   appear in `clip_scan.output_dir`, fitted center sane.
6. `servo-aligner server` + PING from the expctl client; then a real
   sequence through QUEUE — this validates Sequence unpickling, the one
   thing untestable off the Pi. If unpickling fails with a module error,
   set `server.sequence_module_aliases` to the module path in the error.
7. Detached run: `nohup servo-aligner clip-scan --no-plot > clip.log 2>&1 &`.

## 7. Cleanup

After verifying, remove the old copy at
`expctl/src/expctl/servers/servoaligner/` so nothing imports it by accident.
