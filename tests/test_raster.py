"""Raster scan: zigzag un-shuffling and acceptance filtering."""

import numpy as np

from servo_aligner.scan.raster import raster_1d, raster_2d


def quadratic(para):
    x, y = para
    return tuple(para), float(-(x**2) - 2 * y**2 + 0.1 * x * y)


def test_raster_2d_grid_order_independent_of_zigzag():
    result = raster_2d(quadratic, n_pts=7, scan_range=10, progress=False)
    # direct (non-serpentine) evaluation must land at the same grid slots
    expected = np.zeros_like(result.Z)
    for i in range(7):
        for j in range(7):
            expected[i, j] = quadratic([result.X[i, j], result.Y[i, j]])[1]
    np.testing.assert_allclose(result.Z, expected, rtol=0, atol=0)


def test_raster_2d_rectangular_grid():
    result = raster_2d(quadratic, n_pts=(4, 6), scan_range=(10, 20), progress=False)
    assert result.Z.shape == (6, 4)
    assert result.X.min() == -10 and result.Y.max() == 20


def test_raster_2d_accept_func_skips_points():
    accept = lambda xy: abs(xy[1] - xy[0]) < 1e-9  # noqa: E731 — diagonal only
    result = raster_2d(quadratic, n_pts=5, scan_range=4, progress=False)
    filtered = raster_2d(
        quadratic, n_pts=5, scan_range=4, accept_func=accept, progress=False
    )
    diag = np.diag(filtered.Z)
    np.testing.assert_allclose(diag, np.diag(result.Z))
    off_diag = filtered.Z - np.diag(diag)
    np.testing.assert_array_equal(off_diag, np.zeros_like(off_diag))


def test_raster_1d():
    L, Z = raster_1d(
        quadratic, n_pts=11, scan_range=5, scan_vec=[1.0, 0.0], progress=False
    )
    assert len(L) == len(Z) == 11
    np.testing.assert_allclose(Z, [quadratic([x, 0.0])[1] for x in L])
