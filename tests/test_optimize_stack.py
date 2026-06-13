"""Parity + behavior tests for iterate_points and step_optimize."""

import numpy as np
import pytest

from servo_aligner.channels import ChannelLayout
from servo_aligner.config import OptimizeConfig, SpiralParams
from servo_aligner.optimize.iterate import iterate_points
from servo_aligner.optimize.step import step_optimize
from servo_aligner.vectors import compose_para, nrselr

LAYOUT = ChannelLayout(
    ["A_x", "A_y", "A_xdot", "A_ydot", "B_x", "B_y", "B_xdot", "B_ydot"],
    {"A_X_XDOT": ["A_x", "A_xdot"]},
)

LEGACY_SPIRAL = SpiralParams().as_options()
LEGACY_LBFGSB = {"disp": True, "maxiter": 10, "eps": 5}


def smooth_objective(para):
    z = float(np.exp(-((para[0] - 3.0) ** 2 + (para[1] + 2.0) ** 2) / 50.0))
    return tuple(para), z


# ------------------------------------------------------------- iterate_points


def test_iterate_spiral_matches_legacy(golden):
    trace = iterate_points(
        smooth_objective,
        n_var=2,
        p0=[0.0, 0.0],
        bounds=[(-50, 50), (-50, 50)],
        method="spiral",
        options=dict(LEGACY_SPIRAL),
    )
    g = golden["pts_iterator_spiral"]
    np.testing.assert_allclose(trace.best_para, g["best_para"], rtol=0, atol=0)
    assert trace.best_value == g["best_I"]
    assert len(trace.samples) == g["n_evals"]
    assert trace.spiral_centers is not None


def test_iterate_lbfgsb_matches_legacy(golden):
    trace = iterate_points(
        smooth_objective,
        n_var=2,
        p0=[0.0, 0.0],
        bounds=[(-50, 50), (-50, 50)],
        method="L-BFGS-B",
        options=dict(LEGACY_LBFGSB),
    )
    g = golden["pts_iterator_lbfgsb"]
    np.testing.assert_allclose(trace.best_para, g["best_para"], rtol=1e-12)
    assert trace.best_value == pytest.approx(g["best_I"], rel=1e-12)
    assert len(trace.samples) == g["n_evals"]


def test_iterate_rejects_spiral_above_2d():
    with pytest.raises(AssertionError):
        iterate_points(smooth_objective, n_var=3, method="spiral")


def test_iterate_propagates_exceptions():
    # legacy swallowed exceptions and returned None; they must propagate now
    def broken(para):
        raise RuntimeError("sensor died")

    with pytest.raises(RuntimeError, match="sensor died"):
        iterate_points(broken, n_var=2, method="spiral", options=dict(LEGACY_SPIRAL))


# -------------------------------------------------------------- step_optimize


class StubMeasurement:
    """Deterministic objective matching the legacy golden capture."""

    def __init__(self, fn=None):
        self.fn = fn

    def objective(self, group, zero=None, coupling=None):
        def cb(para):
            if self.fn is not None:
                return self.fn(para)
            nr = compose_para(para, list(group.mask), zero)
            sel = nrselr(nr, list(group.mask))
            z = float(np.exp(-((sel[0] - 5.0) ** 2 + (sel[1] - 4.0) ** 2) / 200.0))
            return tuple(para), z

        return cb


def test_step_optimize_accept_matches_legacy(golden):
    zero_new = step_optimize(
        StubMeasurement(),
        LAYOUT.group("A_X_XDOT"),
        zero=np.zeros(8),
        method="spiral",
        bounds_single=(-100, 100),
        opt=OptimizeConfig(),
    )
    np.testing.assert_allclose(
        zero_new, golden["step_optimize_accept"]["zero_new"], rtol=0, atol=0
    )
    # only the masked channels moved
    assert zero_new[1] == 0 and zero_new[3] == 0
    assert all(zero_new[4:] == 0)


def test_step_optimize_reject_keeps_zero():
    # objective collapses when the best point is re-measured -> ratio < 0.7
    state = {"calls": 0}

    def flaky(para):
        state["calls"] += 1
        good = abs(para[0] - 5) < 1 and abs(para[1] - 5) < 1
        if good and state.setdefault("spiked", 0) == 0:
            state["spiked"] = 1
            return tuple(para), 100.0  # one-off spike
        return tuple(para), 0.01

    zero0 = np.arange(8, dtype=float)
    zero_new = step_optimize(
        StubMeasurement(flaky),
        LAYOUT.group("A_X_XDOT"),
        zero=zero0,
        method="spiral",
        opt=OptimizeConfig(),
    )
    np.testing.assert_array_equal(zero_new, zero0)


def test_step_optimize_trace_sink_called():
    seen = []
    step_optimize(
        StubMeasurement(),
        LAYOUT.group("A_X_XDOT"),
        zero=np.zeros(8),
        method="spiral",
        opt=OptimizeConfig(),
        trace_sink=seen.append,
    )
    assert len(seen) == 1
    assert seen[0].best_para is not None


def test_step_optimize_unknown_method():
    with pytest.raises(ValueError, match="unknown method"):
        step_optimize(
            StubMeasurement(),
            LAYOUT.group("A_X_XDOT"),
            zero=np.zeros(8),
            method="genetic",
        )
