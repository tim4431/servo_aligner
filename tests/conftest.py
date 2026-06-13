import json
from pathlib import Path

import pytest

GOLDEN_PATH = Path(__file__).parent / "golden" / "golden_values.json"


@pytest.fixture(scope="session")
def golden():
    """Golden values captured from the legacy flat modules (pre-refactor).

    See tests/golden/_capture_from_legacy.py for provenance.
    """
    return json.loads(GOLDEN_PATH.read_text())


SIM_CONFIG_TEMPLATE = """
state_dir: {tmp}/state
data_dir: {tmp}/data

actuator:
  backend: simulation
  channels:
    - {{name: A_x,    servo_id: 3}}
    - {{name: A_y,    servo_id: 4}}
    - {{name: A_xdot, servo_id: 1}}
    - {{name: A_ydot, servo_id: 2}}
    - {{name: B_x,    servo_id: 5}}
    - {{name: B_y,    servo_id: 6}}
    - {{name: B_xdot, servo_id: 7}}
    - {{name: B_ydot, servo_id: 8}}

sensor:
  backend: simulation
  samples_per_read: 2

groups:
  A_X_XDOT:    [A_x, A_xdot]
  A_Y_YDOT:    [A_y, A_ydot]
  A_X_Y:       [A_x, A_y]
  A_POS_ALL:   [A_x, A_y, A_xdot, A_ydot]
  B_X_XDOT:    [B_x, B_xdot]
  B_Y_YDOT:    [B_y, B_ydot]
  B_X_Y:       [B_xdot, B_ydot]
  B_POS_ALL:   [B_x, B_y, B_xdot, B_ydot]
  ALL: "*"

simulation:
  noise_rms: 1.0e-6
  seed: 1234
  smooth_transition: {smooth}
  paths:
    - {{channels: [A_x, A_y, A_xdot, A_ydot], weight: 1.0}}
  true_zero: {true_zero}
"""


def write_sim_config(
    tmp_path, true_zero="[0, 0, 0, 0, 0, 0, 0, 0]", smooth="0.3", extra=""
):
    """Write a simulation-backend config and return its path."""
    text = SIM_CONFIG_TEMPLATE.format(
        tmp=tmp_path, smooth=smooth, true_zero=true_zero
    )
    path = tmp_path / "sim.yaml"
    path.write_text(text + extra)
    return path
