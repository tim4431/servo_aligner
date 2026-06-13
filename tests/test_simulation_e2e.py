"""End-to-end: config → factory → simulated stack → alignment improves."""

import numpy as np
import pytest

from servo_aligner.config import load_config
from servo_aligner.factory import build_stack
from servo_aligner.fitting import fit_gaussian_2d_smooth_heaviside, popt_get_mu_cov
from servo_aligner.hal.interfaces import Actuator, IntensitySensor
from servo_aligner.optimize.step import step_optimize
from servo_aligner.scan.raster import raster_2d

from .conftest import write_sim_config


def test_factory_builds_simulation_stack(tmp_path):
    stack = build_stack(load_config(write_sim_config(tmp_path)))
    assert isinstance(stack.actuator, Actuator)
    assert isinstance(stack.sensor, IntensitySensor)
    assert stack.actuator.n_channels == 8
    assert stack.layout.group("A_X_XDOT").mask == (1, 0, 1, 0, 0, 0, 0, 0)


def test_aligned_pose_reads_peak_intensity(tmp_path):
    cfg = load_config(write_sim_config(tmp_path, true_zero="[0,0,0,0,0,0,0,0]"))
    stack = build_stack(cfg)
    assert stack.sensor.read_averaged() == pytest.approx(0.9817, abs=0.01)
    # large misalignment kills the signal
    stack.actuator.set_angles([200, 0, 0, 0, 0, 0, 0, 0])
    assert stack.sensor.read_averaged() == pytest.approx(0.0, abs=0.01)


def test_set_zero_keeps_physical_pose(tmp_path):
    cfg = load_config(write_sim_config(tmp_path, true_zero="[15,0,0,0,0,0,0,0]"))
    stack = build_stack(cfg)
    stack.actuator.set_angles([15, 0, 0, 0, 0, 0, 0, 0])
    before = stack.sensor.read_averaged()
    stack.actuator.set_zero()
    np.testing.assert_array_equal(stack.actuator.get_angles(), np.zeros(8))
    assert stack.sensor.read_averaged() == pytest.approx(before, abs=0.01)


def test_measurement_moves_then_reads(tmp_path):
    cfg = load_config(write_sim_config(tmp_path))
    stack = build_stack(cfg)
    group = stack.layout.group("A_X_XDOT")
    para, z = stack.measurement.measure(
        [5.0, -5.0], group, zero=np.zeros(8)
    )
    assert para == (5.0, -5.0)
    np.testing.assert_array_equal(
        stack.actuator.command_log[-1], [5.0, 0, -5.0, 0, 0, 0, 0, 0]
    )
    assert 0 < z <= 1.1


def test_step_optimize_recovers_misalignment(tmp_path):
    """The spiral, with lab-default tuning, must climb onto the passband.

    The (x, xdot) transmission is a long diagonal ridge (the coupling line),
    so the optimizer can land anywhere along it — the invariant is that the
    intensity recovers, not that a specific point is found. The start is
    misaligned ACROSS the ridge (I ≈ 0.2).
    """
    cfg = load_config(
        write_sim_config(tmp_path, true_zero="[25, 0, 20, 0, 0, 0, 0, 0]")
    )
    stack = build_stack(cfg)
    group = stack.layout.group("A_X_XDOT")
    zero0 = np.zeros(8)

    i_start = stack.measurement.objective(group, zero=zero0)([0, 0])[1]
    assert i_start < 0.3
    zero_new = step_optimize(
        stack.measurement, group, zero0, opt=cfg.optimize, method="spiral"
    )
    i_end = stack.measurement.objective(group, zero=zero_new)([0, 0])[1]

    assert i_end > 0.6
    assert i_end > 2 * i_start
    # untouched channels stay zero
    assert zero_new[1] == 0 and all(zero_new[4:] == 0)


def test_scan_fit_roundtrip_lands_in_passband(tmp_path):
    """Raster + smooth-Heaviside fit must put the fitted center on the beam.

    The passband is an extended diagonal band, so the fitted center is not a
    unique point — the lab-relevant invariant (and the legacy acceptance
    logic) is that moving to the fitted mu recovers high intensity.
    """
    cfg = load_config(
        write_sim_config(
            tmp_path, true_zero="[30, 0, -25, 0, 0, 0, 0, 0]", smooth="null"
        )
    )
    stack = build_stack(cfg)
    group = stack.layout.group("A_X_XDOT")
    objective = stack.measurement.objective(group, zero=np.zeros(8))

    result = raster_2d(objective, n_pts=25, scan_range=150, progress=False)
    assert result.Z.max() > 0.5  # the scan actually crossed the passband

    popt = fit_gaussian_2d_smooth_heaviside(result.X, result.Y, result.Z)
    mu, _ = popt_get_mu_cov(popt)
    # the fitted center must lie inside the scan window and on the beam
    assert abs(mu[0]) < 150 and abs(mu[1]) < 150
    _, i_at_mu = objective(list(mu))
    assert i_at_mu > 0.8


def test_command_log_and_dehysteresis_flag(tmp_path):
    cfg = load_config(write_sim_config(tmp_path))
    stack = build_stack(cfg)
    stack.actuator.de_hysteresis = False
    stack.actuator.set_angles([1, 2, 3, 4, 5, 6, 7, 8])
    stack.actuator.home()
    assert len(stack.actuator.command_log) == 2
    np.testing.assert_array_equal(stack.actuator.command_log[-1], np.zeros(8))
    assert stack.actuator.de_hysteresis is False
