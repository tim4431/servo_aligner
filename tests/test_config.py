"""Config loading, placeholder expansion, and validation."""

from pathlib import Path

import pytest
import yaml

from servo_aligner.config import ConfigError, load_config

EXAMPLE = Path(__file__).parents[1] / "config" / "example_config.yaml"

MINIMAL = """
actuator:
  channels:
    - {name: x, servo_id: 1}
    - {name: y, servo_id: 2}
groups:
  XY: [x, y]
"""


def _write(tmp_path, text):
    p = tmp_path / "cfg.yaml"
    p.write_text(text)
    return p


def test_example_config_loads():
    cfg = load_config(EXAMPLE)
    assert cfg.actuator.backend == "sts3032"
    assert len(cfg.actuator.channels) == 8
    assert cfg.sensor.address == 0x68
    assert cfg.optimize.accept_ratio == 0.7
    assert cfg.optimize.spiral.SPIRAL_RESOLUTION == 14
    assert cfg.clip_scan.accept_lines["A_X_XDOT"].slope == 1.35
    assert [s.group for s in cfg.clip_scan.stages] == ["A_X_XDOT", "A_Y_YDOT", "A_X_Y"]
    assert cfg.jacobian.master == "B"
    assert cfg.server.port == 60627
    # masks derived from the example config match the legacy layout
    layout = cfg.layout()
    assert list(layout.group("A_X_XDOT").mask) == [1, 0, 1, 0, 0, 0, 0, 0]
    assert list(layout.group("B_X_Y").mask) == [0, 0, 0, 0, 0, 0, 1, 1]


def test_minimal_config(tmp_path):
    cfg = load_config(_write(tmp_path, MINIMAL))
    assert cfg.channel_names == ("x", "y")
    assert cfg.layout().group("XY").mask == (1, 1)
    # defaults
    assert cfg.actuator.de_hysteresis.overshoot_counts == 100
    assert cfg.sensor.samples_per_read == 2


def test_resolve_dir_placeholders(tmp_path):
    cfg = load_config(_write(tmp_path, MINIMAL + "\ndata_dir: /tmp/sa_data\n"))
    out = cfg.resolve_dir("{data_dir}/clip_scan")
    assert out == Path("/tmp/sa_data/clip_scan")


def test_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        load_config(tmp_path / "nope.yaml")


def test_env_var_resolution(tmp_path, monkeypatch):
    p = _write(tmp_path, MINIMAL)
    monkeypatch.setenv("SERVO_ALIGNER_CONFIG", str(p))
    cfg = load_config()
    assert cfg.source_path == p


def test_unknown_key_rejected(tmp_path):
    with pytest.raises(ConfigError, match="unknown"):
        load_config(_write(tmp_path, MINIMAL + "\ntypo_section: 1\n"))


def test_duplicate_servo_ids_rejected(tmp_path):
    bad = """
actuator:
  channels:
    - {name: x, servo_id: 1}
    - {name: y, servo_id: 1}
"""
    with pytest.raises(ConfigError, match="duplicate servo ids"):
        load_config(_write(tmp_path, bad))


def test_group_with_unknown_channel_rejected(tmp_path):
    bad = MINIMAL + "\n  BAD: [x, ghost]\n"
    with pytest.raises(KeyError, match="ghost"):
        load_config(_write(tmp_path, bad))


def test_clip_stage_group_must_have_two_channels(tmp_path):
    bad = MINIMAL + """
clip_scan:
  stages:
    - {group: XYZ, n_pts: 5, range: 10}
"""
    with pytest.raises(ConfigError, match="unknown group"):
        load_config(_write(tmp_path, bad))


def test_spiral_params_as_options_match_legacy():
    cfg = load_config(EXAMPLE)
    opts = cfg.optimize.spiral.as_options()
    assert opts == {
        "I_meaningful": 0.005,
        "D": 2.4,
        "SPIRAL_RESOLUTION": 14,
        "SPIRAL_SPAN": 6,
        "SINGLE_SPIRAL_SPAN": 3.5,
        "N_LOOPS_BEFORE_RESET_ORIGIN": 0.5,
        "MAX_X0Y0_DISPLACEMENT": 10,
        "COEF_I_RESET_ORIGIN": 1.4,
        "alpha": 0.03,
        "COEF_I_DECAY": 0.995,
    }


def test_yaml_example_is_valid_yaml():
    # guard against editing the example into invalid YAML
    assert yaml.safe_load(EXAMPLE.read_text())
