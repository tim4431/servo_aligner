"""All matplotlib rendering, decoupled from the algorithms.

Requires the ``plot`` extra. Imports matplotlib lazily so headless
deployments (the Pi runs scans with ``--no-plot``) never load it.
The figures reproduce the legacy inline plots of ``pts_iterator``,
``motor_scan`` and ``clip_scan``.
"""

from __future__ import annotations

from typing import Callable, Optional, Sequence

import numpy as np

from .fitting import gaussian_2d_smooth_heaviside, popt_get_mu_cov
from .optimize.iterate import OptimizationTrace
from .scan.raster import ScanResult


def _plt():
    import matplotlib.pyplot as plt

    return plt


def plot_optimization_trace(trace: OptimizationTrace, show: bool = False):
    """Visited points + convergence curve (legacy pts_iterator figure)."""
    plt = _plt()
    paras, values = zip(*trace.samples)
    fig, (ax0, ax1) = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))

    if len(paras[0]) == 2:
        boundx, boundy = trace.bounds
        xs, ys = zip(*paras)
        ax0.scatter(ys, xs, c=values, cmap="jet", label="Visited points")
        xlst, ylst = trace.final_para
        ax0.scatter(ylst, xlst, marker="x", c="black", s=50, label="Last point")
        xbst, ybst = trace.best_para
        ax0.scatter(ybst, xbst, marker="*", c="red", s=100, label="Best point")
        if trace.spiral_centers is not None:
            pts_x0, pts_y0 = trace.spiral_centers
            ax0.plot(pts_y0, pts_x0, "b--", label="(x0,y0)")
        ax0.set(xlim=boundx, ylim=boundy, xlabel="y(deg)", ylabel="x(deg)")
        ax0.legend()
        ax0.set_aspect("equal")
        ax0.figure.colorbar(ax0.collections[0], ax=ax0)

    ax1.plot(values)
    ax1.set(xlabel="n_iter", ylabel="value")
    if show:
        plt.show()
    return fig


def plot_scan(result: ScanResult, show: bool = False):
    """Heatmap of a 2D raster scan (legacy motor_2d_scan figure)."""
    plt = _plt()
    fig = plt.matshow(
        result.Z,
        extent=[
            np.min(result.X),
            np.max(result.X),
            np.min(result.Y),
            np.max(result.Y),
        ],
        origin="lower",
    ).figure
    plt.colorbar()
    if show:
        plt.show()
    return fig


def render_heaviside_fit(ax, X, Y, Z, popt):
    """Scan image + fitted smooth-Heaviside contours (legacy fit_and_plot)."""
    bounds_x = (np.min(X), np.max(X))
    bounds_y = (np.min(Y), np.max(Y))
    X_new = np.linspace(*bounds_x, 100)
    Y_new = np.linspace(*bounds_y, 100)
    X_new, Y_new = np.meshgrid(X_new, Y_new)
    mu, cov = popt_get_mu_cov(popt)
    Z_new = np.array(
        [
            gaussian_2d_smooth_heaviside(x_, y_, mu, cov, popt[5], popt[6])
            for x_, y_ in zip(X_new.ravel(), Y_new.ravel())
        ]
    )
    ax.imshow(
        Z.reshape(X.shape) / np.max(Z),
        origin="lower",
        extent=[bounds_x[0], bounds_x[1], bounds_y[0], bounds_y[1]],
    )
    ax.contour(X_new, Y_new, Z_new.reshape(X_new.shape), cmap="jet")


def plot_clip_fit(
    result: ScanResult,
    popt: np.ndarray,
    mu: np.ndarray,
    sigmas: np.ndarray,
    file_name: str,
    accept_func: Optional[Callable[[Sequence[float]], np.ndarray]] = None,
    plot_type: int = 0,
    save_path: Optional[str] = None,
):
    """The clip-scan figure (legacy scan_and_analyze plot branches).

    ``plot_type`` 0: four panels (acceptance overlay, fit, zoomed center,
    row sum); 1: a single fit panel.
    """
    plt = _plt()
    X, Y, Z = result.X, result.Y, result.Z
    Z_norm = Z / np.max(Z)
    title = "{}, mu = ({:.3f},{:.3f}), sigmas = ({:.3f},{:.3f})".format(
        file_name, mu[0], mu[1], sigmas[0], sigmas[1]
    )

    if plot_type == 0:
        fig, ax = plt.subplots(1, 4, figsize=(20, 5))

        # ax0: acceptance-region overlay
        ax0 = ax[0]
        if accept_func is not None:
            xy = np.array([X, Y])
            Zacc = accept_func(xy).astype(int)
            ax0.imshow(
                Zacc * 0.2 + Z_norm,
                extent=[X.min(), X.max(), Y.min(), Y.max()],
                origin="lower",
            )
        else:
            ax0.imshow(
                Z_norm, extent=[X.min(), X.max(), Y.min(), Y.max()], origin="lower"
            )

        # ax1: smooth-Heaviside fit
        render_heaviside_fit(ax[1], X, Y, Z, popt)
        ax[1].scatter(mu[0], mu[1], marker="x", color="black")

        # ax2: central zoom
        ax2 = ax[2]
        ax2.imshow(Z_norm, extent=[X.min(), X.max(), Y.min(), Y.max()], origin="lower")
        ax2.scatter(mu[0], mu[1], marker="x", color="black")
        ax2.set_xlim([mu[0] - 100, mu[0] + 100])
        ax2.set_ylim([mu[1] - 100, mu[1] + 100])

        # ax3: row sum
        ax3 = ax[3]
        Z_row = np.sum(Z, axis=0)
        ax3.plot(X[0, :], Z_row)

        fig.suptitle(title)
    else:
        fig, ax = plt.subplots(figsize=(10, 5))
        render_heaviside_fit(ax, X, Y, Z, popt)
        ax.scatter(mu[0], mu[1], marker="x", color="black")
        ax.set_title(title)

    if save_path is not None:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
    return fig
