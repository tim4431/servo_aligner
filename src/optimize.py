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
* :func:`optimize_mask` — the fallback recipe: a single joint L-BFGS-B over an
  arbitrary set of knobs, for when the caller hand-picks the changeable channels
  rather than choosing a named recipe.

For interactive use (the servo console's "Optimize" subpage) the recipes are also
exposed as **named, selectable templates**: :class:`OptTemplate` wraps a stage
sequence and can report which channels it moves (:meth:`OptTemplate.channel_mask`)
and run itself; :data:`OPT_TEMPLATES` (built by :func:`build_templates`) is the
``{name: OptTemplate}`` registry, holding a ``fiber_coupling_<path>`` entry for
each beam path whose knob masks are configured. :func:`optimize_knobs` takes an
optional ``on_stage`` progress hook so a UI can show which knobs are active.

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
from config import knob_mask, MASKS


def optimize_knobs(servos, callback_func, zero, masks,
                   opt_type="max", methods=None, bounds=None, on_stage=None):
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
        on_stage: optional progress hook ``on_stage(mask, i, n)`` called just
            before each stage ``i`` (0-based) of ``n`` starts with that stage's
            full-length ``mask``, and once more at the end with ``(None, n, n)``.
            Lets a UI show which knobs are being optimized right now (the servo
            console uses it to mark the active knobs); errors from it are ignored.

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

    def _notify(mask, i):
        if on_stage is not None:
            try:
                on_stage(mask, i, n)
            except Exception:
                pass   # a progress hook must never break the optimization

    zero = np.array(zero, dtype=float)
    for i, mask in enumerate(masks):
        n_var = int(np.sum(mask))
        method = methods[i] or ("spiral" if n_var == 2 else "L-BFGS-B")
        kw = {} if bounds[i] is None else {"bounds_single": tuple(bounds[i])}
        logging.info("[optimize_knobs] stage %d/%d: mask=%s method=%s opt_type=%s",
                     i + 1, n, mask, method, opt_type)
        _notify(mask, i)
        zero = step_optimize(servos, callback_func, pos_mask=mask, zero=zero,
                             method=method, opt_type=opt_type, **kw)
    _notify(None, n)   # signal "no stage active" so the UI clears its markers

    # Re-evaluate the objective at the final origin: pass the full origin as the
    # step over an all-channels mask (zero origin), i.e. move there and read.
    full_mask = [1] * len(zero)
    _, value = callback_func(zero, pos_mask=full_mask)
    logging.info("[optimize_knobs] final value = %s", value)
    return zero, value


# The knob groups a fiber-coupling recipe runs through, in stage order. A beam
# path is "wired" for the recipe only if all four masks are configured for it.
_FIBER_COUPLING_GROUPS = ["X_Y", "X_XDOT", "Y_YDOT", "POS_ALL"]


def _fiber_coupling_masks(path):
    """The X_Y -> X_XDOT -> Y_YDOT -> POS_ALL stage masks for ``path``.

    Raises ``KeyError`` (from :func:`config.knob_mask`) if any of the four masks
    is not defined for ``path`` -- :func:`build_templates` only builds templates
    for paths :func:`_fiber_coupling_paths` has already confirmed are complete.
    """
    return [knob_mask(path, group) for group in _FIBER_COUPLING_GROUPS]


def fiber_coupling(servos, callback_func, zero, path="A", opt_type="max", on_stage=None):
    """Fiber-coupling alignment recipe for one beam path.

    Runs the sequence ``calibrate_jacobian.py`` performs on the slave path:
    coarse X/Y, then the two position/angle knob pairs (X/Xdot, Y/Ydot) by spiral
    descent, then a joint L-BFGS-B refinement over all four position knobs. The
    per-stage method isn't spelled out -- :func:`optimize_knobs` auto-picks
    ``spiral`` for the 2-knob pair stages and ``L-BFGS-B`` for the 4-knob POS_ALL
    stage, which is exactly that sequence.

    Args:
        servos, callback_func, zero: as in :func:`optimize_knobs`.
        path: beam path to align; any configured path prefix (e.g. ``"A"``).
        opt_type: optimization sense; ``"max"`` (couple the most power) by default.
        on_stage: optional progress hook, forwarded to :func:`optimize_knobs`.

    Returns:
        ``(best_zero, value)`` from :func:`optimize_knobs`.
    """
    return optimize_knobs(servos, callback_func, zero, _fiber_coupling_masks(path),
                          opt_type=opt_type, on_stage=on_stage)


def optimize_mask(servos, callback_func, zero, mask, opt_type="max", on_stage=None):
    """Default 'just optimize these knobs' recipe: one L-BFGS-B stage over ``mask``.

    This is the fallback the console uses when the user hand-picks which knobs are
    optimizable (instead of selecting a named template): a single joint L-BFGS-B
    optimization over exactly the chosen channels. ``mask`` is a full-length 0/1
    ``pos_mask``. Returns ``(best_zero, value)`` from :func:`optimize_knobs`.
    """
    return optimize_knobs(servos, callback_func, zero, [mask],
                          opt_type=opt_type, methods=["L-BFGS-B"], on_stage=on_stage)


class OptTemplate:
    """A named, selectable optimization recipe for the servo console.

    A template is just an ordered list of stage ``masks`` (+ optional per-stage
    ``methods``) plus a display ``name``/``description``. It knows which channels
    it will move (:meth:`channel_mask`, the union of its stage masks) so the
    console can preview that, and it runs itself through :func:`optimize_knobs`.
    """

    def __init__(self, name, masks, methods=None, description=""):
        self.name = name
        self.masks = [list(m) for m in masks]
        self.methods = methods
        self.description = description

    def channel_mask(self):
        """Full-length 0/1 mask of every channel this template moves (union of stages)."""
        n = len(self.masks[0])
        return [1 if any(m[i] for m in self.masks) else 0 for i in range(n)]

    def run(self, servos, callback_func, zero, opt_type="max", on_stage=None):
        return optimize_knobs(servos, callback_func, zero, self.masks,
                              opt_type=opt_type, methods=self.methods, on_stage=on_stage)


def _fiber_coupling_paths():
    """Beam-path prefixes that have a full set of fiber-coupling knob masks.

    Discovered from the configured mask names (:data:`config.MASKS`) rather than a
    fixed ``A``/``B`` list, so any path naming works: a mask ``<P>_X_Y`` marks a
    candidate path ``<P>``, kept only if every group in
    :data:`_FIBER_COUPLING_GROUPS` is also defined for it. Returned in the order
    the masks are defined in the config.
    """
    suffix = "_X_Y"
    paths = []
    for name in MASKS:
        if name.endswith(suffix):
            path = name[:-len(suffix)]
            if path and path not in paths and \
                    all(f"{path}_{g}" in MASKS for g in _FIBER_COUPLING_GROUPS):
                paths.append(path)
    return paths


def build_templates():
    """Discover the optimization templates available on this machine.

    Returns an ordered ``{name: OptTemplate}`` dict with a ``fiber_coupling_<path>``
    template for each path :func:`_fiber_coupling_paths` reports, so a single-path
    rig gets only that path's template (and a rig with paths named anything, not
    just ``A``/``B``, gets templates for all of them).
    """
    templates = {}
    for path in _fiber_coupling_paths():
        name = f"fiber_coupling_{path}"
        templates[name] = OptTemplate(
            name=name, masks=_fiber_coupling_masks(path),
            description=f"fiber coupling of beam path {path}")
    return templates


# The templates the console offers; built once from the machine's configured masks.
OPT_TEMPLATES = build_templates()
