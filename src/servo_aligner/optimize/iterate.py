"""Optimizer dispatch: spiral descent, L-BFGS-B, or Powell.

Port of the legacy ``pts_iterator`` minus the inline plotting: every
evaluated sample is recorded into an :class:`OptimizationTrace` which the
caller can hand to :func:`servo_aligner.plotting.plot_optimization_trace`.

Unlike the legacy code, exceptions propagate to the caller (which is
responsible for homing/closing hardware in a ``finally``) instead of being
swallowed and returning ``None``.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
import scipy.optimize

from ..vectors import format_para
from .spiral import SpiralPath

logger = logging.getLogger(__name__)

METHODS = ("spiral", "L-BFGS-B", "Powell")


@dataclass
class OptimizationTrace:
    """Everything one optimizer run produced."""

    method: str
    bounds: Sequence[Tuple[float, float]]
    samples: List[Tuple[tuple, float]] = field(default_factory=list)
    final_para: Optional[Sequence[float]] = None
    best_para: Optional[tuple] = None
    best_value: Optional[float] = None
    #: spiral-center trajectory (pts_x0, pts_y0), spiral method only
    spiral_centers: Optional[Tuple[List[float], List[float]]] = None


def iterate_points(
    objective: Callable[[Sequence[float]], Tuple[tuple, float]],
    n_var: int,
    p0: Optional[Sequence[float]] = None,
    bounds: Optional[Sequence[Tuple[float, float]]] = None,
    method: str = "spiral",
    options: Optional[dict] = None,
) -> OptimizationTrace:
    """Maximize a noisy objective over ``n_var`` knobs.

    Args:
        objective: ``para -> (para, intensity)``; intensity is maximized.
        n_var: Number of free parameters.
        p0: Starting point (defaults to the origin).
        bounds: Per-parameter ``(min, max)``; defaults to ``(-50, 50)`` each.
        method: ``"spiral"`` (2D only), ``"L-BFGS-B"``, or ``"Powell"``.
        options: Optimizer options (spiral attribute overrides / scipy options).

    Returns:
        The trace; ``best_para``/``best_value`` is the highest-intensity
        sample observed (not necessarily the final optimizer point).
    """
    if p0 is None:
        p0 = [0.0] * n_var
    if bounds is None:
        bounds = [(-50, 50)] * n_var
    options = options or {}
    assert len(p0) == n_var, ValueError(f"len(p0) should be {n_var}")
    assert len(bounds) == n_var, ValueError(f"len(bounds) should be {n_var}")
    if method == "spiral":
        assert n_var == 2, ValueError("Only 2D spiral path is supported")
    if method not in METHODS:
        raise ValueError(f"unknown method: {method}")

    trace = OptimizationTrace(method=method, bounds=list(bounds))

    def _wrapper(paras, multiplier: int = 1):
        para, value = objective(paras)
        trace.samples.append((para, value))
        logger.debug("%s; T=%s", format_para(para), value)
        return multiplier * value

    sp = SpiralPath()

    para = p0
    if method == "L-BFGS-B":
        res = scipy.optimize.minimize(
            lambda paras: _wrapper(paras, multiplier=-1),
            x0=p0,
            method="L-BFGS-B",
            bounds=bounds,
            options=options,
        )
        para = res.x
    elif method == "Powell":
        res = scipy.optimize.minimize(
            lambda paras: _wrapper(paras, multiplier=-1),
            x0=p0,
            method="Powell",
            bounds=bounds,
            options=options,
        )
        para = res.x
    elif method == "spiral":
        para = sp.maximize(
            lambda paras: _wrapper(paras, multiplier=1),
            x0=para,
            bounds=bounds,
            options=options,
        )
        trace.spiral_centers = (sp.pts_x0, sp.pts_y0)

    logger.info("Converged position: %s", format_para(para))

    paras, values = zip(*trace.samples)
    idx = int(np.argmax(values))
    trace.final_para = para
    trace.best_para = paras[idx]
    trace.best_value = values[idx]
    logger.info("Best point: %s; T=%s", format_para(trace.best_para), trace.best_value)
    return trace
