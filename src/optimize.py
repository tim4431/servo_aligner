"""Composable optimization sequences over servo knob groups.

A single alignment is a *sequence* of optimizer stages: optimize one knob group,
commit the improved origin, move on to the next group, and finish with a joint
multi-knob refinement. ``calibrate_jacobian.py`` used to spell such a sequence
out inline; this module factors it into a reusable template so the same shape can
drive **any** objective, in **any** direction, over **any** choice of knob groups.

* :func:`optimize_knobs` — the template. Given a start origin ``zero``, an
  optimizer ``callback_func`` (from :func:`callback_functions.make_callback_func`),
  an optimization ``opt_type`` (``"max"`` / ``"min"`` / ``"zero"``) and an ordered
  list of knob ``masks``, it runs one :func:`step_optimize.step_optimize` per mask,
  threading the improving origin through, and returns ``(best_zero, value)`` — the
  final full-length angle origin and the objective re-evaluated there.
* :func:`fiber_coupling` — a concrete recipe built on the template: the
  fiber-coupling alignment of one beam path (coarse X/Y, then the X/Xdot and
  Y/Ydot knob pairs by spiral descent, then a joint L-BFGS-B over all four
  position knobs).

``opt_type`` meanings (each is turned into a maximization inside the optimizer;
see :func:`step_optimize.score_of`):

===========  =====================================================
``"max"``    maximize the objective (e.g. coupled power) — default
``"min"``    minimize the objective
``"zero"``   drive the objective toward 0 (minimize its magnitude)
===========  =====================================================

The objective itself is whatever ``callback_func`` reads — swap it by building the
callback with a different objective, e.g.::

    from callback_functions import make_callback_func, get_objective
    cb = make_callback_func(servos, get_objective("intensity_adc"))
    best_zero, value = fiber_coupling(servos, cb, zero, path="A")

This module imports :mod:`step_optimize` and :mod:`config` only (both pure); it
touches no hardware itself — motion happens through the injected ``callback_func``
and ``servos``.
"""

import logging

import numpy as np

from step_optimize import step_optimize
from config import knob_mask


def optimize_knobs(servos, callback_func, zero, masks,
                   opt_type="max", methods=None, bounds=None):
    """Optimize a list of knob groups in turn, threading the origin through.

    One :func:`step_optimize.step_optimize` is run per entry in ``masks``; each
    stage starts from the origin the previous stage committed, so the sequence
    progressively refines a single full-length ``zero``.

    Args:
        servos: the live ``Servoset`` (forwarded to ``step_optimize``).
        callback_func: objective ``(para, pos_mask, zero=...) -> (para, value)``
            from :func:`callback_functions.make_callback_func`.
        zero: full-length angle origin (degrees) to start from.
        masks: ordered list of full-length 0/1 ``pos_mask``s; each is one stage.
        opt_type: ``"max"`` / ``"min"`` / ``"zero"`` (see module docstring).
        methods: optional per-mask method override, same length as ``masks``. A
            ``None`` entry (or ``methods=None``) auto-picks ``"spiral"`` for a
            2-knob group (spiral is 2D-only) and ``"L-BFGS-B"`` otherwise.
        bounds: optional per-mask ``(lo, hi)`` ``bounds_single`` override, same
            length as ``masks``; ``None`` entries use ``step_optimize``'s default.

    Returns:
        ``(best_zero, value)`` — the final full-length angle origin and the raw
        objective re-evaluated there (not the ``opt_type``-transformed score).
    """
    masks = [list(m) for m in masks]
    n = len(masks)
    if methods is None:
        methods = [None] * n
    if bounds is None:
        bounds = [None] * n
    if not (len(methods) == len(bounds) == n):
        raise ValueError("masks, methods and bounds must have equal length")

    zero = np.array(zero, dtype=float)
    for i, mask in enumerate(masks):
        n_var = int(np.sum(mask))
        method = methods[i] or ("spiral" if n_var == 2 else "L-BFGS-B")
        kw = {} if bounds[i] is None else {"bounds_single": tuple(bounds[i])}
        logging.info("[optimize_knobs] stage %d/%d: mask=%s method=%s opt_type=%s",
                     i + 1, n, mask, method, opt_type)
        zero = step_optimize(servos, callback_func, pos_mask=mask, zero=zero,
                             method=method, opt_type=opt_type, **kw)

    # Re-evaluate the objective at the final origin: pass the full origin as the
    # step over an all-channels mask (zero origin), i.e. move there and read.
    full_mask = [1] * len(zero)
    _, value = callback_func(zero, pos_mask=full_mask)
    logging.info("[optimize_knobs] final value = %s", value)
    return zero, value


def fiber_coupling(servos, callback_func, zero, path="A", opt_type="max"):
    """Fiber-coupling alignment recipe for one beam path.

    Runs the sequence ``calibrate_jacobian.py`` performs on the slave path:
    coarse X/Y, then the two position/angle knob pairs (X/Xdot, Y/Ydot) by spiral
    descent, then a joint L-BFGS-B refinement over all four position knobs.

    Args:
        servos, callback_func, zero: as in :func:`optimize_knobs`.
        path: beam path to align, ``"A"`` (upper) or ``"B"`` (lower).
        opt_type: optimization sense; ``"max"`` (couple the most power) by default.

    Returns:
        ``(best_zero, value)`` from :func:`optimize_knobs`.
    """
    masks = [knob_mask(path, "X_Y"),
             knob_mask(path, "X_XDOT"),
             knob_mask(path, "Y_YDOT"),
             knob_mask(path, "POS_ALL")]
    methods = ["spiral", "spiral", "spiral", "L-BFGS-B"]
    return optimize_knobs(servos, callback_func, zero, masks,
                          opt_type=opt_type, methods=methods)
