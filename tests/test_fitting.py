"""Parity tests for the Gaussian / smooth-Heaviside fitting."""

import numpy as np

from servo_aligner import fitting


def _grid():
    x = np.linspace(-5, 5, 25)
    y = np.linspace(-5, 5, 25)
    return np.meshgrid(x, y)


MU_T = np.array([0.7, -1.1])
COV_T = np.array([[2.0, 0.3], [0.3, 1.2]])


def test_fit_smooth_heaviside_matches_legacy(golden):
    GX, GY = _grid()
    GZ = np.array(
        [
            [
                fitting.gaussian_2d_smooth_heaviside(xi, yi, MU_T, COV_T, 0.5, 0.2)
                for xi, yi in zip(xrow, yrow)
            ]
            for xrow, yrow in zip(GX, GY)
        ]
    )
    popt = fitting.fit_gaussian_2d_smooth_heaviside(GX, GY, GZ)
    np.testing.assert_allclose(popt, golden["fit_heaviside"]["popt"], rtol=1e-9)


def test_fit_gaussian_matches_legacy(golden):
    GX, GY = _grid()
    GZ = np.array(
        [
            [fitting.gaussian_2d(xi, yi, MU_T, COV_T) for xi, yi in zip(xr, yr)]
            for xr, yr in zip(GX, GY)
        ]
    )
    popt = fitting.fit_gaussian_2d(GX, GY, GZ)
    np.testing.assert_allclose(popt, golden["fit_gaussian"]["popt"], rtol=1e-9)


def test_popt_get_mu_cov_roundtrip():
    popt = np.array([1.0, 2.0, 3.0, 0.5, 4.0])
    mu, cov = fitting.popt_get_mu_cov(popt)
    np.testing.assert_array_equal(mu, [1.0, 2.0])
    np.testing.assert_array_equal(cov, [[3.0, 0.5], [0.5, 4.0]])
