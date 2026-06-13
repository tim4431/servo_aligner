"""One optimization stage: search a channel group, conditionally commit.

Port of the legacy ``step_optimize``: run the optimizer from the current
``zero``, re-measure the best point, and only commit it as the new origin if
the re-measured intensity stays above ``accept_ratio`` (default 70%) of the
best seen — protecting against noise spikes and drift.
"""

from __future__ import annotations

from typing import Callable, Optional, Tuple

import numpy as np
import logging

from ..channels import ChannelGroup
from ..config import OptimizeConfig
from ..measurement import JacobianCoupling, Measurement
from ..vectors import format_para, nraddr
from .iterate import OptimizationTrace, iterate_points

logger = logging.getLogger(__name__)


def step_optimize(
    measurement: Measurement,
    group: ChannelGroup,
    zero,
    *,
    opt: Optional[OptimizeConfig] = None,
    p0=None,
    method: str = "spiral",
    bounds_single: Optional[Tuple[float, float]] = None,
    coupling: Optional[JacobianCoupling] = None,
    trace_sink: Optional[Callable[[OptimizationTrace], None]] = None,
) -> np.ndarray:
    """Optimize the channels of ``group`` and return the (maybe) updated zero.

    Args:
        measurement: The move-and-measure bundle.
        group: Channel group to optimize (2 channels for the spiral method).
        zero: Current full-length zero offset (degrees).
        opt: Optimizer tuning; defaults to :class:`OptimizeConfig` defaults.
        p0: Start point in reduced coordinates (defaults to the origin).
        method: ``"spiral"`` or ``"L-BFGS-B"``.
        bounds_single: Per-knob bounds; defaults to ``opt.default_bounds``.
        coupling: Optional Jacobian coupling applied inside the objective.
        trace_sink: Called with the :class:`OptimizationTrace` (e.g. plotting).

    Returns:
        The new full-length zero: moved to the best point if accepted,
        unchanged otherwise.
    """
    if opt is None:
        opt = OptimizeConfig()
    if method == "L-BFGS-B":
        options = dict(opt.lbfgsb)
    elif method == "spiral":
        options = opt.spiral.as_options()
    else:
        raise ValueError(f"unknown method: {method}")
    if bounds_single is None:
        bounds_single = tuple(opt.default_bounds)

    n_var = group.n
    if p0 is None:
        p0 = np.zeros(n_var)
    bounds = [bounds_single for _ in range(n_var)]
    objective = measurement.objective(group, zero=zero, coupling=coupling)

    i_start = objective(p0)[1]
    logger.info("Start position: %s, start I: %s", format_para(p0), i_start)

    trace = iterate_points(
        objective, n_var=n_var, p0=p0, bounds=bounds, options=options, method=method
    )
    if trace_sink is not None:
        trace_sink(trace)

    para = list(trace.best_para)
    i_now = objective(para)[1]
    logger.info("Best position: %s, now I: %s", format_para(para), i_now)

    if i_now / trace.best_value > opt.accept_ratio:
        logger.info("New Origin set to be %s", format_para(para))
        zero_full = nraddr(zero, para, list(group.mask))
    else:
        logger.info("The intensity is not high enough, operation cancelled.")
        zero_full = np.array(zero)

    logger.info("Zero = %s", zero_full)
    return zero_full
