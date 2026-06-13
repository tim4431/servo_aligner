"""Parity tests for compose_para — the core of every alignment objective."""

import numpy as np

from servo_aligner.vectors import compose_para

A_X_XDOT_MASK = [1, 0, 1, 0, 0, 0, 0, 0]
B_POS_ALL_MASK = [0, 0, 0, 0, 1, 1, 1, 1]

JAC = np.array(
    [
        [0.5, -0.1, 0.0, 0.2],
        [0.0, 0.3, -0.2, 0.1],
        [0.1, 0.0, 0.4, -0.3],
        [-0.2, 0.1, 0.0, 0.6],
    ]
)


def test_compose_plain(golden):
    out = compose_para(
        [3.0, -7.0],
        A_X_XDOT_MASK,
        zero=np.array([1, 2, 3, 4, 5, 6, 7, 8], dtype=float),
    )
    np.testing.assert_array_equal(out, golden["compose_plain"])


def test_compose_jac(golden):
    out = compose_para(
        [10.0, -20.0, 5.0, 2.5],
        B_POS_ALL_MASK,
        zero=np.array([1, 2, 3, 4, 5, 6, 7, 8], dtype=float),
        jac=JAC,
        jac_master_mask=B_POS_ALL_MASK,
        jac_x0=np.array([0.1, 0.2, 0.3, 0.4]),
    )
    np.testing.assert_allclose(out, golden["compose_jac"], rtol=0, atol=0)


def test_compose_jac_with_master_offset(golden):
    out = compose_para(
        [10.0, -20.0, 5.0, 2.5],
        B_POS_ALL_MASK,
        zero=np.zeros(8),
        jac=JAC,
        jac_master_mask=B_POS_ALL_MASK,
        jac_master_offset=np.array([1.0, 1.0, -1.0, -1.0]),
        jac_x0=np.array([0.1, 0.2, 0.3, 0.4]),
    )
    np.testing.assert_allclose(out, golden["compose_jac_offset"], rtol=0, atol=0)


def test_compose_defaults():
    # para=None / zero=None default to zeros (legacy supports this only for
    # an all-ones mask, since the default para has full length)
    out = compose_para(None, [1] * 8)
    np.testing.assert_array_equal(out, np.zeros(8))
