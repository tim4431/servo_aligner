"""Parity tests: servo_aligner.vectors must reproduce the legacy servo_util."""

import numpy as np
import pytest

from servo_aligner import vectors


def test_a2p(golden):
    for angle_str, expected in golden["a2p"].items():
        assert vectors.a2p(float(angle_str)) == expected


def test_r2nd(golden):
    np.testing.assert_array_equal(vectors.r2nd([-5, 4], [1, 0, 1, 0]), golden["r2nd"])


def test_r2nr(golden):
    np.testing.assert_array_equal(
        vectors.r2nr([-5.5, 4.25], [1, 0, 1, 0]), golden["r2nr"]
    )


def test_nrselr(golden):
    np.testing.assert_array_equal(
        vectors.nrselr([10.0, 20.0, 30.0, 40.0], [0, 1, 0, 1]), golden["nrselr"]
    )


def test_nrmodr(golden):
    np.testing.assert_array_equal(
        vectors.nrmodr([1.0, 2.0, 3.0, 4.0], [9.5, -9.5], [1, 0, 0, 1]),
        golden["nrmodr"],
    )


def test_nraddr(golden):
    np.testing.assert_array_equal(
        vectors.nraddr([1.0, 2.0, 3.0, 4.0], [0.5, -0.5], [0, 1, 1, 0]),
        golden["nraddr"],
    )


def test_ndmodr(golden):
    np.testing.assert_array_equal(
        vectors.ndmodr([2048, 2048, 2048, 2048], [45.0, -45.0], [1, 0, 0, 1]),
        golden["ndmodr"],
    )


def test_format_para(golden):
    assert vectors.format_para([1.234, -5.678]) == golden["format_para"]


def test_zigzag(golden):
    X = np.arange(12, dtype=float).reshape(3, 4)
    X_zig, index_map = vectors.create_zigzag_X(X)
    np.testing.assert_array_equal(X_zig, golden["zigzag"]["X_zig"])
    np.testing.assert_array_equal(index_map, golden["zigzag"]["index_map"])


def test_mask_length_mismatch_raises():
    with pytest.raises(AssertionError):
        vectors.r2nr([1.0, 2.0, 3.0], [1, 0, 1, 0])
