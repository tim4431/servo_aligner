from typing import List, Tuple, Callable
import logging
import numpy as np
import scipy.optimize
import matplotlib.pyplot as plt

from servo_util import format_para, nraddr
from spiral import SpiralPath, SpiralPathConfig
from config import SPIRAL_PARAMS, BFGS_PARAMS

# Optimizer tuning comes from calibration.yaml (the `spiral:` / `bfgs:` sections).
spiral_params = SpiralPathConfig(**SPIRAL_PARAMS)
BFGS_params = dict(BFGS_PARAMS)


def pts_iterator(
    N_var: int,
    callback_func: Callable,
    p0: List[float] = [0, 0],
    bounds: List[Tuple[float, float]] = [(-50, 50), (-50, 50)],
    method: str = "spiral",
    options: dict = {},
):
    """Maximize a noisy objective over `N_var` knobs and return the best point seen.

    Dispatches to one of three optimizers, records every evaluated
    ``(para, intensity)`` sample, and plots the search trace and convergence curve.

    Args:
        N_var: Number of free parameters (must match ``len(p0)`` and ``len(bounds)``).
        callback_func: Objective ``para -> (para, intensity)``; intensity is maximized.
        p0: Starting point.
        bounds: Per-parameter ``(min, max)`` limits.
        method: ``"spiral"`` (custom spiral descent, 2D only), ``"L-BFGS-B"``, or ``"Powell"``.
        options: Optimizer options dict (e.g. ``spiral_params`` / ``BFGS_params``).

    Returns:
        ``(best_para, best_intensity)`` — the highest-intensity sample observed.
    """
    assert len(p0) == N_var, ValueError(f"len(p0) should be {N_var}")
    assert len(bounds) == N_var, ValueError(f"len(bounds) should be {N_var}")
    if method == "spiral":
        assert N_var == 2, ValueError("Only 2D spiral path is supported")

    datas = []

    def _optimize_wrapper(datas, callback_func, paras, multiplier: int = 1):
        para, T = callback_func(paras)
        datas.append((para, T))
        logging.debug(f"{format_para(para)}; T={T}")
        return multiplier * T

    sp = SpiralPath()

    try:
        para = p0
        if method == "L-BFGS-B":
            para = scipy.optimize.minimize(
                lambda para: _optimize_wrapper(datas, callback_func, para, multiplier=-1),
                x0=p0,
                method="L-BFGS-B",
                bounds=bounds,
                options=options,
            ).x
        elif method == "Powell":
            para = scipy.optimize.minimize(
                lambda para: _optimize_wrapper(datas, callback_func, para, multiplier=-1),
                x0=p0,
                method="Powell",
                bounds=bounds,
                options=options,
            ).x
        elif method == "spiral":
            para = sp.maximize(
                lambda para: _optimize_wrapper(datas, callback_func, para, multiplier=1),
                x0=para,
                bounds=bounds,
                options=options,
            )

        logging.info("Converged position: {:s}".format(format_para(para)))

        # >>> statistics: best point <<<
        paras, Ts = zip(*datas)
        idx = np.argmax(Ts)
        Tbst = Ts[idx]
        parabst = paras[idx]
        logging.info(f"Best point: {format_para(parabst)}; T={Tbst}")

        # >>> plot trace (ax0) + convergence curve (ax1) <<<
        fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))
        ax0, ax1 = axs
        if N_var == 2:
            boundx, boundy = bounds
            # trace, coloured by intensity
            xs, ys = zip(*paras)
            ax0.scatter(ys, xs, c=Ts, cmap="jet", label="Visited points")
            # last point
            xlst, ylst = para
            ax0.scatter(ylst, xlst, marker="x", c="black", s=50, label="Last point")
            # best point
            xbst, ybst = parabst
            ax0.scatter(ybst, xbst, marker="*", c="red", s=100, label="Best point")
            # spiral-centre trajectory
            if method == "spiral":
                ax0.plot(sp.pts_y0, sp.pts_x0, "b--", label="(x0,y0)")
            ax0.set(xlim=boundx, ylim=boundy, xlabel="y(deg)", ylabel="x(deg)")
            ax0.legend()
            ax0.set_aspect("equal")
            ax0.figure.colorbar(ax0.collections[0], ax=ax0)  # 0 stands for trace
        ax1.plot(Ts)
        ax1.set(xlabel="n_iter", ylabel="value")

        return (parabst, Tbst)

    except Exception as e:
        logging.error(e)


def step_optimize(servos,
                  callback_func,
                  pos_mask=None,
                  p0=None,
                  zero=None,
                  method="spiral",
                  bounds_single = (-100,100),
                  )->np.ndarray:
    #
    if method == 'L-BFGS-B':
        # servos.set_precision(1)
        options = BFGS_params
    elif method == 'spiral':
        # servos.set_precision(5)
        options = spiral_params
    else:
        raise ValueError(f"unknown method: {method}")
    #
    N_var = np.sum(pos_mask)
    if p0 is None:
        p0 = np.zeros(N_var)
    bounds = [bounds_single for i in range(N_var)]
    cf = lambda x: callback_func(x,pos_mask,zero=zero)
    Istart = cf(p0)[1]
    logging.info(f"Start position: {format_para(p0)}, start I: {Istart}")
    #
    para, Ibst = pts_iterator(N_var=N_var,callback_func=cf, p0=p0, bounds = bounds, options=options, method = method)
    #
    para = list(para)
    Inow = cf(para)[1]
    logging.info(f"Best position: {format_para(para)}, now I: {Inow}")
    #
    if Inow/Ibst > 0.7:
        logging.info(f"New Origin set to be {format_para(para)}")
        zero_fullnd = nraddr(zero,para,pos_mask)
    else:
        logging.info("The intensity is not high enough, operation cancelled.")
        zero_fullnd = np.array(zero)
    #
    logging.info(f"Zero = {zero_fullnd}")
    return zero_fullnd
