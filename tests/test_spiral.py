"""Parity test: SpiralPath must reproduce the legacy trajectory exactly."""

import numpy as np

from servo_aligner.optimize.spiral import SpiralPath

# the legacy step_optimize.spiral_params used for the golden capture
LEGACY_SPIRAL_PARAMS = {
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


def covariance_matrix(sigma_a, sigma_b, theta):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    Lambda = np.array([[sigma_a**2, 0], [0, sigma_b**2]])
    return np.dot(R, np.dot(Lambda, R.T))


def noisy_gaussian2d(x, y, mu, cov):
    # byte-for-byte the legacy spiral.gaussian2d demo objective
    inv_cov = np.linalg.inv(cov)
    det_cov = float(np.linalg.det(cov))
    r = np.array([x, y]).T - mu
    z = np.exp(-0.5 * (r @ inv_cov @ r.T))
    coeff = 1 / (2 * np.pi * np.sqrt(det_cov))
    return coeff * z + 0.00000001 * np.random.randn()


def test_spiral_trajectory_matches_legacy(golden):
    np.random.seed(42)
    mu = np.array([8.0, 15.0])
    cov = covariance_matrix(4, 0.2, np.pi / 3)
    sp = SpiralPath()
    result = sp.maximize(
        lambda xy: noisy_gaussian2d(xy[0], xy[1], mu, cov),
        x0=(0.0, 0.0),
        bounds=[(-20, 20), (-20, 20)],
        options=dict(LEGACY_SPIRAL_PARAMS),
    )
    g = golden["spiral"]
    assert len(sp.pts_x) == g["n_pts"]
    np.testing.assert_allclose(result, g["result"], rtol=0, atol=0)
    np.testing.assert_allclose(sp.pts_x, g["pts_x"], rtol=0, atol=0)
    np.testing.assert_allclose(sp.pts_y, g["pts_y"], rtol=0, atol=0)
    np.testing.assert_allclose(sp.pts_I, g["pts_I"], rtol=0, atol=0)
    np.testing.assert_allclose(sp.pts_x0, g["pts_x0"], rtol=0, atol=0)
    np.testing.assert_allclose(sp.pts_y0, g["pts_y0"], rtol=0, atol=0)
