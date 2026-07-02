"""Least-squares fits of a 2D intensity map ``(X, Y, Z)`` to beam models.

Two model families, each with a fit + plot helper:

* plain 2D Gaussian (optionally with scale/offset) — ``fit_gaussian_2d`` /
  ``fit_and_plot``;
* "smooth heaviside": a Gaussian-shaped plateau with an erfc edge, the
  beam-clip model ``clip_scan.py`` fits to find the clip center —
  ``fit_gaussian_2d_smooth_heaviside`` / ``fit_and_plot_smooth_heaviside``.

The fit parameter vector ``popt`` is ``[mu_x, mu_y, cov_xx, cov_xy, cov_yy,
...model extras]``; ``popt_get_mu_cov`` unpacks the shared part. Initial
guesses come from intensity-weighted moments (``statistics_for_gaussian2d``).
Pure numpy/scipy/matplotlib — no hardware imports, safe anywhere.
"""

import logging

from scipy.optimize import least_squares
import numpy as np
import matplotlib.pyplot as plt
from scipy.special import erfc


def gaussian_2d(x: float, y: float, mu, cov) -> float:
    inv_cov = np.linalg.inv(cov)
    det_cov = float(np.linalg.det(cov))
    r = np.array([x, y]).T - mu
    z = np.exp(-0.5 * (r @ inv_cov @ r.T))
    coeff = 1 / (2 * np.pi * np.sqrt(det_cov))
    return coeff * z

def gaussian_2d_offset(x: float, y: float, mu, cov, scale, offset) -> float:
    return gaussian_2d(x, y, mu, cov) * scale + offset

def gaussian_2d_smooth_heaviside(x: float, y: float,mu, cov,  amp ,transition_width) -> float:
    inv_cov = np.linalg.inv(cov)
    r = np.array([x, y]) - mu
    quadratic_form = r.T @ inv_cov @ r

    # Define a smooth transition using a sigmoid-like function
    # smooth_transition = 1 / (1 + np.exp((quadratic_form - 1) / transition_width))
    # using scipy.erfc
    smooth_transition = amp * erfc((quadratic_form - 1) / transition_width)

    return smooth_transition


def statistics_for_gaussian2d(xdata, ydata, Idata):
    # Flatten the arrays in case they are not 1D
    xdata = xdata.flatten()
    ydata = ydata.flatten()
    Idata = Idata.flatten()

    # Calculate the sum of the data values
    sum_I = np.sum(Idata)

    # Calculate the weighted mean (mu)
    mu_x = np.sum(xdata * Idata) / sum_I
    mu_y = np.sum(ydata * Idata) / sum_I
    mu = np.array([mu_x, mu_y])

    # Center the coordinates by subtracting the mean
    x_centered = xdata - mu_x
    y_centered = ydata - mu_y

    # Calculate the elements of the covariance matrix
    sigma_xx = np.sum(Idata * x_centered * x_centered) / sum_I
    sigma_xy = np.sum(Idata * x_centered * y_centered) / sum_I
    sigma_yy = np.sum(Idata * y_centered * y_centered) / sum_I

    # Assemble the covariance matrix (cov)
    cov = np.array([[sigma_xx, sigma_xy],
                    [sigma_xy, sigma_yy]])

    return mu, cov

def popt_get_mu_cov(popt):
    mu = popt[:2]
    cov = np.array([[popt[2], popt[3]], [popt[3], popt[4]]])
    return mu, cov

def fit_gaussian_2d(X,Y,Z,p0=None,offset=False):
    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)

    if offset == False:
        # fit to gaussian_2d_cov using least square
        def _residuals(p, x, y, z):
            mu, cov = popt_get_mu_cov(p)
            z_fit = np.array([gaussian_2d(x_, y_, mu, cov) for x_, y_ in zip(x, y)])
            return z - z_fit
    else:
        def _residuals(p, x, y, z):
            mu, cov = popt_get_mu_cov(p)
            z_fit = np.array([gaussian_2d_offset(x_, y_, mu, cov, p[5], p[6]) for x_, y_ in zip(x, y)])
            return z - z_fit

    xdata = np.array(X).flatten()
    ydata = np.array(Y).flatten()
    zdata = np.array(Z).flatten()

    if p0 is None:
        mu, cov = statistics_for_gaussian2d(X, Y, Z)
        if offset == False:
            p0 = np.array([mu[0], mu[1], cov[0, 0], cov[0, 1], cov[1, 1]])
        else:
            # sign the scale by whether the peak at (mu_x, mu_y) sits nearer the
            # max (a bright peak) or the min (a dip)
            xidx = np.unravel_index(np.argmin(np.abs(X-mu[0])),X.shape)[0]
            yidx = np.unravel_index(np.argmin(np.abs(Y-mu[1])),Y.shape)[1]
            Z_center = Z[xidx,yidx]
            logging.info("X,Y,Z_center: %s %s %s", X[xidx,yidx], Y[xidx,yidx], Z_center)
            if np.abs(Z_center-np.min(Z))>np.abs(Z_center-np.max(Z)):
                scale = +(np.max(Z)-np.min(Z))
                z_offset = np.min(Z)
            else:
                scale = -(np.max(Z)-np.min(Z))
                z_offset = np.max(Z)
            p0 = np.array([mu[0], mu[1], cov[0, 0], cov[0, 1], cov[1, 1], scale, z_offset])
        logging.info("Initial guess for p0: %s", p0)

    res = least_squares(_residuals, p0, args=(xdata, ydata, zdata))
    popt = res.x
    return popt

def fit_gaussian_2d_smooth_heaviside(X,Y,Z,p0=None):
    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)
    # fit to gaussian_2d_cov using least square
    def _residuals(p, x, y, z):
        mu, cov = popt_get_mu_cov(p)
        z_fit = np.array([gaussian_2d_smooth_heaviside(x_, y_, mu, cov, p[5], p[6]) for x_, y_ in zip(x, y)])
        return z - z_fit

    xdata = np.array(X).flatten()
    ydata = np.array(Y).flatten()
    zdata = np.array(Z).flatten()

    if p0 is None:
        mu, cov = statistics_for_gaussian2d(X, Y, Z)
        p0 = np.array([mu[0], mu[1], cov[0, 0], cov[0, 1], cov[1, 1], np.max(Z)/2, 0.1])
        logging.info("Initial guess for p0: %s", p0)

    res = least_squares(_residuals, p0, args=(xdata, ydata, zdata))
    popt = res.x
    return popt


def fit_and_plot(X,Y,Z,p0=None,ax=None,offset=False):
    popt = fit_gaussian_2d(X,Y,Z,p0=p0,offset=offset)
    bounds_x = (np.min(X),np.max(X))
    bounds_y = (np.min(Y),np.max(Y))
    X_new = np.linspace(*bounds_x,100)
    Y_new = np.linspace(*bounds_y,100)
    X_new,Y_new = np.meshgrid(X_new,Y_new)
    mu, cov = popt_get_mu_cov(popt)
    Z_new = np.array([gaussian_2d(x_,y_,mu,cov) for x_,y_ in zip(X_new.ravel(),Y_new.ravel())])
    #
    if ax is None:
        fig, ax = plt.subplots()
    ax.imshow(Z.reshape(X.shape)/np.max(Z),origin="lower",extent=[bounds_x[0],bounds_x[1],bounds_y[0],bounds_y[1]])
    ax.contour(X_new,Y_new,Z_new.reshape(X_new.shape),cmap="jet")
    return popt

def fit_and_plot_smooth_heaviside(X,Y,Z,p0=None,ax=None):
    popt = fit_gaussian_2d_smooth_heaviside(X,Y,Z,p0=p0)
    bounds_x = (np.min(X),np.max(X))
    bounds_y = (np.min(Y),np.max(Y))
    X_new = np.linspace(*bounds_x,100)
    Y_new = np.linspace(*bounds_y,100)
    X_new,Y_new = np.meshgrid(X_new,Y_new)
    mu, cov = popt_get_mu_cov(popt)
    Z_new = np.array([gaussian_2d_smooth_heaviside(x_,y_,mu,cov, popt[5], popt[6]) for x_,y_ in zip(X_new.ravel(),Y_new.ravel())])
    #
    if ax is None:
        fig, ax = plt.subplots()
    ax.imshow(Z.reshape(X.shape)/np.max(Z),origin="lower",extent=[bounds_x[0],bounds_x[1],bounds_y[0],bounds_y[1]])
    ax.contour(X_new,Y_new,Z_new.reshape(X_new.shape),cmap="jet")
    return popt

def ZX(X0,Z_row):
    """ average X with weights Z_row """
    return np.dot(X0,Z_row)/np.sum(Z_row)

def ZZX(X0,Z_row):
    """ average (X0-mu)^2 with weights Z_row """
    mu = ZX(X0,Z_row)
    return np.dot((X0-mu)**2,Z_row)/np.sum(Z_row)

def ZZZX(X0,Z_row):
    """ average (X0-mu)^3 with weights Z_row """
    mu = ZX(X0,Z_row)
    return np.dot((X0-mu)**3,Z_row)/np.sum(Z_row)

def statistics_skewness(X0,Z_row):
    return ZZZX(X0,Z_row)/ZZX(X0,Z_row)**1.5

if __name__ == "__main__":
    # round-trip test: generate a 2D Gaussian, then fit it back
    x = np.linspace(-1, 1, 20)
    y = np.linspace(-1, 1, 20)
    X, Y = np.meshgrid(x, y)
    mu_true = np.array([0.1, -0.2])
    cov_true = np.array([[0.2, 0.05], [0.05, 0.1]])
    Z = np.array([[gaussian_2d(xi, yi, mu_true, cov_true) for xi, yi in zip(xr, yr)]
                  for xr, yr in zip(X, Y)])
    popt = fit_gaussian_2d(X, Y, Z)
    print("true mu, cov:    ", mu_true, cov_true.ravel())
    print("recovered mu, cov:", popt_get_mu_cov(popt))
