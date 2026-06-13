"""Parity tests for the beam-clip simulation against legacy numeric_sim."""

import numpy as np

from servo_aligner.sim.beam_model import (
    BeamClipModel,
    calc_data,
    crosstalk_matrix,
)


def test_calc_data_xxdot_matches_legacy(golden):
    Z, xZ, tZ = calc_data(
        crosstalk_matrix=crosstalk_matrix, zero=np.zeros(4), scan_type="xxdot"
    )
    g = golden["numeric_sim"]
    assert float(Z.sum()) == g["Z_sum"]
    assert float(xZ.sum()) == g["xZ_sum"]
    assert float(tZ.sum()) == g["tZ_sum"]
    for i, j, val in g["Z_samples"]:
        assert float(Z[i, j]) == val


def test_calc_data_yydot_matches_legacy(golden):
    Z, _, _ = calc_data(
        crosstalk_matrix=crosstalk_matrix,
        zero=np.array([10.0, -5.0, 3.0, 2.0]) * np.pi / 180,
        scan_type="yydot",
    )
    assert float(Z.sum()) == golden["numeric_sim_yydot"]["Z_sum"]


def test_transmission_consistent_with_calc_data_slice():
    """The 4-knob transmission must reproduce the xxdot slice of calc_data."""
    model = BeamClipModel()
    zero = np.array([0.05, -0.02, 0.01, 0.03])
    # transmission([kx, ky, kxdot, kydot]) with y knobs at 0 + zero must equal
    # the calc_data xxdot computation at (tknob1, tknob2) = (kx, kxdot)
    Z, _, _ = calc_data(
        crosstalk_matrix=crosstalk_matrix, zero=zero, scan_type="xxdot", n=5
    )
    from servo_aligner.sim.beam_model import make_grid

    X, Y = make_grid(n=5)
    for i in range(5):
        for j in range(5):
            t = model.transmission(
                [X[i, j], 0.0, Y[i, j], 0.0],
                zero_rad=[zero[0], zero[1], zero[2], zero[3]],
            )
            assert t == Z[i, j]


def test_smooth_transition_gives_gradients():
    model = BeamClipModel(smooth_transition=0.3)
    t_center = model.transmission([0.0, 0.0, 0.0, 0.0])
    t_off = model.transmission([0.5, 0.0, 0.0, 0.0])
    assert 0 < t_off < t_center <= 1.0
