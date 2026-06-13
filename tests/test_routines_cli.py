"""Routines + CLI running end-to-end against the simulation backend."""

import numpy as np
import pytest

from servo_aligner.cli.main import main
from servo_aligner.config import load_config
from servo_aligner.factory import build_stack
from servo_aligner.routines.calibrate_jacobian import run_jacobian_calibration
from servo_aligner.routines.clip_scan import build_accept_func, run_clip_scan

from .conftest import write_sim_config

CLIP_EXTRA = """
clip_scan:
  output_dir: "{data_dir}/clip"
  I_meaningful: 0.1
  accept_lines:
    A_X_XDOT: {slope: -0.75, intercept: 0, tol: 80}
  stages:
    - {group: A_X_XDOT, n_pts: 15, range: 120, accept: true, plot_type: 0}
  iterations: [0]
"""

JAC_EXTRA = """
jacobian:
  output_dir: "{data_dir}/jac"
  paths: [A, B]
  master: B
  offset_type: zero
  n_iterations: 1
  stages:
    - {group_suffix: X_XDOT, method: spiral}
"""


def test_accept_func_matches_legacy_lambda():
    from servo_aligner.config import AcceptLine

    f = build_accept_func(AcceptLine(slope=1.35, intercept=0, tol=80))
    legacy = lambda xy: np.abs(xy[1] - xy[0] * 1.35 - 0) < 80  # noqa: E731
    for xy in [(0, 0), (100, 135), (100, 300), (-50, -67.5), (-50, 50)]:
        assert f(xy) == legacy(xy)


def test_run_clip_scan_on_simulator(tmp_path):
    cfg_path = write_sim_config(
        tmp_path, true_zero="[20, 0, -15, 0, 0, 0, 0, 0]", smooth="null",
        extra=CLIP_EXTRA,
    )
    cfg = load_config(cfg_path)
    zero = run_clip_scan(cfg, plot=False)

    out = cfg.resolve_dir(cfg.clip_scan.output_dir)
    scan_file = out / "clip_A_X_XDOT_0.npz"
    popt_file = out / "clip_A_X_XDOT_0_popt.npz"
    assert scan_file.exists() and popt_file.exists()

    # legacy npz keys preserved
    d = np.load(scan_file, allow_pickle=True)
    assert {"X", "Y", "Z", "posmaskstr", "ITER_NUM"} <= set(d.keys())
    assert str(d["posmaskstr"]) == "A_X_XDOT"
    dp = np.load(popt_file, allow_pickle=True)
    assert {"popt", "mu", "cov", "sigmas", "zero"} <= set(dp.keys())
    np.testing.assert_array_equal(dp["zero"], zero)

    # only the scanned pair may have moved; other channels stay zeroed
    assert zero[1] == 0 and all(zero[4:] == 0)

    # the accepted zero must put the beam back on the detector
    stack = build_stack(cfg)
    _, intensity = stack.measurement.measure(list(zero), stack.layout.all)
    assert intensity > 0.5


def test_run_jacobian_calibration_zero_offset(tmp_path):
    cfg_path = write_sim_config(
        tmp_path,
        true_zero="[15, 0, 12, 0, 0, 0, 0, 0]",
        extra=JAC_EXTRA,
    )
    cfg = load_config(cfg_path)
    # master B, zero offset -> stages optimize the A path; must not raise
    run_jacobian_calibration(cfg)


def test_run_jacobian_calibration_pm_dataset(tmp_path):
    cfg_path = write_sim_config(tmp_path, extra=JAC_EXTRA)
    cfg = load_config(cfg_path)
    run_jacobian_calibration(cfg, offset_type="pm", norm=5, n_iterations=2)
    out = cfg.resolve_dir(cfg.jacobian.output_dir)
    dataset = np.load(out / "jacobian_pm_5.npy", allow_pickle=True).item()
    # two pm offsets recorded, each with one (zero, intensity) entry
    assert len(dataset) == 2
    for offset, entries in dataset.items():
        assert len(offset) == 4
        zero, intensity = entries[0]
        assert len(zero) == 8
        assert isinstance(intensity, float)


def test_cli_status_and_moves(tmp_path, capsys):
    cfg = write_sim_config(tmp_path)
    assert main(["-c", str(cfg), "status"]) == 0
    out = capsys.readouterr().out
    assert "A_x" in out and "backend: simulation" in out

    assert main(["-c", str(cfg), "set-angle", "1", "2", "3", "4", "5", "6", "7", "8"]) == 0
    assert main(["-c", str(cfg), "home"]) == 0
    assert main(["-c", str(cfg), "set-single", "0", "12.5"]) == 0


def test_cli_clip_scan(tmp_path):
    cfg = write_sim_config(
        tmp_path, true_zero="[20, 0, -15, 0, 0, 0, 0, 0]", smooth="null",
        extra=CLIP_EXTRA,
    )
    assert main(["-c", str(cfg), "clip-scan", "--no-plot"]) == 0
    assert (tmp_path / "data" / "clip" / "clip_A_X_XDOT_0.npz").exists()


def test_cli_calibrate_jacobian(tmp_path):
    cfg = write_sim_config(tmp_path, extra=JAC_EXTRA)
    assert main(["-c", str(cfg), "calibrate-jacobian", "--offset-type", "zero"]) == 0


def test_cli_missing_config_errors(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv("SERVO_ALIGNER_CONFIG", raising=False)
    with pytest.raises(Exception, match="config file not found"):
        main(["status"])
