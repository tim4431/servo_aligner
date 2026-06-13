"""2D Gaussian and smooth-Heaviside fitting for scan data.

The smooth-Heaviside model fits the flat-topped, sharp-edged intensity
profile produced by beam clipping (see ``doc/application.md``); the plain
Gaussian fits unclipped beams. These numerics are lab-validated.
"""

from __future__ import annotations

import logging

import numpy as np
from scipy.optimize import least_squares
from scipy.special import erfc

logger = logging.getLogger(__name__)


def gaussian_2d(x: float, y: float, mu, cov) -> float:
    """Evaluate a normalized 2D Gaussian at ``(x, y)``."""
    inv_cov = np.linalg.inv(cov)
    det_cov = float(np.linalg.det(cov))
    r = np.array([x, y]).T - mu
    z = np.exp(-0.5 * (r @ inv_cov @ r.T))
    coeff = 1 / (2 * np.pi * np.sqrt(det_cov))
    return coeff * z


def gaussian_2d_offset(x: float, y: float, mu, cov, scale, offset) -> float:
    """2D Gaussian with linear scale and constant offset."""
    return gaussian_2d(x, y, mu, cov) * scale + offset


def gaussian_2d_smooth_heaviside(
    x: float, y: float, mu, cov, amp, transition_width
) -> float:
    """Flat-topped profile: ``amp * erfc((r' Σ⁻¹ r − 1) / transition_width)``."""
    inv_cov = np.linalg.inv(cov)
    r = np.array([x, y]) - mu
    quadratic_form = r.T @ inv_cov @ r
    smooth_transition = amp * erfc((quadratic_form - 1) / transition_width)
    return smooth_transition


def statistics_for_gaussian2d(xdata, ydata, Idata):
    """Intensity-weighted mean and covariance of scan data (fit seed)."""
    xdata = xdata.flatten()
    ydata = ydata.flatten()
    Idata = Idata.flatten()

    sum_I = np.sum(Idata)

    mu_x = np.sum(xdata * Idata) / sum_I
    mu_y = np.sum(ydata * Idata) / sum_I
    mu = np.array([mu_x, mu_y])

    x_centered = xdata - mu_x
    y_centered = ydata - mu_y

    sigma_xx = np.sum(Idata * x_centered * x_centered) / sum_I
    sigma_xy = np.sum(Idata * x_centered * y_centered) / sum_I
    sigma_yy = np.sum(Idata * y_centered * y_centered) / sum_I

    cov = np.array([[sigma_xx, sigma_xy], [sigma_xy, sigma_yy]])
    return mu, cov


def popt_get_mu_cov(popt):
    """Extract ``(mu, cov)`` from a fit parameter vector."""
    mu = popt[:2]
    cov = np.array([[popt[2], popt[3]], [popt[3], popt[4]]])
    return mu, cov


def fit_gaussian_2d(X, Y, Z, p0=None, offset=False):
    """Least-squares fit of a 2D Gaussian (optionally with scale + offset).

    Returns the parameter vector ``[mu_x, mu_y, cov_xx, cov_xy, cov_yy]``
    (plus ``[scale, offset]`` when ``offset=True``).
    """
    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)

    if offset is False:

        def _residuals(p, x, y, z):
            mu, cov = popt_get_mu_cov(p)
            z_fit = np.array([gaussian_2d(x_, y_, mu, cov) for x_, y_ in zip(x, y)])
            return z - z_fit

    else:

        def _residuals(p, x, y, z):
            mu, cov = popt_get_mu_cov(p)
            z_fit = np.array(
                [gaussian_2d_offset(x_, y_, mu, cov, p[5], p[6]) for x_, y_ in zip(x, y)]
            )
            return z - z_fit

    xdata = np.array(X).flatten()
    ydata = np.array(Y).flatten()
    zdata = np.array(Z).flatten()

    if p0 is None:
        mu, cov = statistics_for_gaussian2d(X, Y, Z)
        if offset is False:
            p0 = np.array([mu[0], mu[1], cov[0, 0], cov[0, 1], cov[1, 1]])
        else:
            # seed scale/offset from the value nearest the weighted center
            xidx = np.unravel_index(np.argmin(np.abs(X - mu[0])), X.shape)[0]
            yidx = np.unravel_index(np.argmin(np.abs(Y - mu[1])), Y.shape)[1]
            Z_center = Z[xidx, yidx]
            logger.debug(
                "X,Y,Z_center: %s %s %s", X[xidx, yidx], Y[xidx, yidx], Z_center
            )
            if np.abs(Z_center - np.min(Z)) > np.abs(Z_center - np.max(Z)):
                scale = +(np.max(Z) - np.min(Z))
                offset = np.min(Z)
            else:
                scale = -(np.max(Z) - np.min(Z))
                offset = np.max(Z)
            p0 = np.array([mu[0], mu[1], cov[0, 0], cov[0, 1], cov[1, 1], scale, offset])
        logger.debug("Initial guess for p0: %s", p0)

    res = least_squares(_residuals, p0, args=(xdata, ydata, zdata))
    return res.x


def fit_gaussian_2d_smooth_heaviside(X, Y, Z, p0=None):
    """Least-squares fit of the smooth-Heaviside (clipped-beam) model.

    Returns ``[mu_x, mu_y, cov_xx, cov_xy, cov_yy, amp, transition_width]``.
    """
    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)

    def _residuals(p, x, y, z):
        mu, cov = popt_get_mu_cov(p)
        z_fit = np.array(
            [
                gaussian_2d_smooth_heaviside(x_, y_, mu, cov, p[5], p[6])
                for x_, y_ in zip(x, y)
            ]
        )
        return z - z_fit

    xdata = np.array(X).flatten()
    ydata = np.array(Y).flatten()
    zdata = np.array(Z).flatten()

    if p0 is None:
        mu, cov = statistics_for_gaussian2d(X, Y, Z)
        p0 = np.array([mu[0], mu[1], cov[0, 0], cov[0, 1], cov[1, 1], np.max(Z) / 2, 0.1])
        logger.debug("Initial guess for p0: %s", p0)

    res = least_squares(_residuals, p0, args=(xdata, ydata, zdata))
    return res.x


def ZX(X0, Z_row):
    """Average of ``X0`` with weights ``Z_row``."""
    return np.dot(X0, Z_row) / np.sum(Z_row)


def ZZX(X0, Z_row):
    """Average of ``(X0 - mu)^2`` with weights ``Z_row``."""
    mu = ZX(X0, Z_row)
    return np.dot((X0 - mu) ** 2, Z_row) / np.sum(Z_row)


def ZZZX(X0, Z_row):
    """Average of ``(X0 - mu)^3`` with weights ``Z_row``."""
    mu = ZX(X0, Z_row)
    return np.dot((X0 - mu) ** 3, Z_row) / np.sum(Z_row)


def statistics_skewness(X0, Z_row):
    """Skewness of a weighted 1D profile."""
    return ZZZX(X0, Z_row) / ZZX(X0, Z_row) ** 1.5
