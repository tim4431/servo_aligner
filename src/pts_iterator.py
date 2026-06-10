from typing import List, Tuple, Callable
import scipy.optimize
import matplotlib.pyplot as plt
import logging
import numpy as np
from spiral import SpiralPath


def pts_iterator(
    N_var:int,
    callback_func: Callable,
    p0: List[float] = [0, 0],
    bounds: List[Tuple[float, float]] = [(-50, 50), (-50, 50)],
    method: str = "spiral",
    options: dict = {}
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

    datas=[]

    def _format_para(para):
        str_para = " ".join(["x_{:d}={:.2f}".format(i,para[i]) for i in range(len(para))])
        return str_para


    def _optimize_wrapper(datas, callback_func, paras,multiplier:int=1):
        para, T = callback_func(paras)
        datas.append((para, T))
        log_str = _format_para(para)
        logging.debug(f"{log_str}; T={T}")
        return multiplier*T

    sp=SpiralPath()

    #
    try:
        para = p0
        if method == "L-BFGS-B":
            para = scipy.optimize.minimize(
                lambda para: _optimize_wrapper(datas, callback_func, para, multiplier=-1),
                x0=p0,
                method="L-BFGS-B",
                bounds=bounds,
                options=options,
            )
            para = para.x
        #
        elif method == "Powell":
            para = scipy.optimize.minimize(
                lambda para: _optimize_wrapper(datas, callback_func, para, multiplier=-1),
                x0=p0,
                method="Powell",
                bounds=bounds,
                options=options,
            )
            para = para.x
        elif method == "spiral":
            # use spiral path
            para = sp.maximize(
                lambda para: _optimize_wrapper(datas, callback_func, para, multiplier=1),
                x0=para,
                bounds=bounds,
                options=options,
            )
        #
        logging.info("Converged position: {:s}".format(_format_para(para)))
        #
        # >>> statistics <<<
        # best point
        paras, Ts = zip(*datas)
        idx = np.argmax(Ts)
        Tbst = Ts[idx]
        parabst = paras[idx]
        logging.info(f"Best point: {_format_para(parabst)}; T={Tbst}")

        # >>> plot datas <<<
        fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))
        ax0, ax1 = axs
        # ax0
        if N_var == 2:
            boundx, boundy = bounds
            # trace
            xs, ys = zip(*paras)
            ax0.scatter(
                ys, xs, c=Ts, cmap="jet", label="Visited points"
            )
            # last point
            xlst, ylst = para
            ax0.scatter(ylst, xlst, marker="x", c="black", s=50, label="Last point")
            # best point
            xbst, ybst = parabst
            ax0.scatter(ybst, xbst, marker="*", c="red", s=100, label="Best point")
            # plot x0 y0 trajectory
            if method == "spiral":
                pts_x0 = sp.pts_x0
                pts_y0 = sp.pts_y0
                ax0.plot(pts_y0, pts_x0, "b--", label="(x0,y0)")
            #
            ax0.set(xlim=boundx, ylim=boundy, xlabel="y(deg)", ylabel="x(deg)")
            ax0.legend()
            ax0.set_aspect("equal")
            ax0.figure.colorbar(ax0.collections[0], ax=ax0) # 0 stands for trace
        # ax1
        # >>> plot convergence curve <<<
        ax1.plot(Ts)
        ax1.set(xlabel="n_iter", ylabel="value")
        #
        return (parabst, Tbst)

    except Exception as e:
        logging.error(e)