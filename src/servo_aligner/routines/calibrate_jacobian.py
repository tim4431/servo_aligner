"""Jacobian calibration: offset one beam path, re-optimize the other.

Port of the legacy ``calibrate_jacobian.py`` — the module-level loop now
lives in :func:`run_jacobian_calibration`. For each iteration an offset is
applied on the *master* path, the optimizer re-aligns the other path through
the configured stage sequence, and ``(offset, final_zero, intensity)`` is
appended to a dataset keyed by offset — the finite differences of which
yield the knob-coupling Jacobian (see ``doc/jacobian.md``).
"""

from __future__ import annotations

import logging
import os
from collections import defaultdict
from typing import Optional

import numpy as np

from ..config import Config
from ..factory import AlignerStack, build_stack
from ..optimize.step import step_optimize
from ..vectors import compose_para

logger = logging.getLogger(__name__)


def cord_pm_offset(n_dims: int, normd: float, i: int) -> np.ndarray:
    """Cycle a single coordinate through ±normd (legacy ``pm`` offsets)."""
    j = i % n_dims
    pm = 1 if i // n_dims % 2 == 0 else -1
    offset = np.zeros(n_dims)
    offset[j] = pm * normd
    return offset


def random_norm_offset(n_dims: int, normd: float) -> np.ndarray:
    """Random direction with fixed norm."""
    offset = np.random.randn(n_dims)
    return offset / np.linalg.norm(offset) * normd


def lin_comb_offset(normd: float, vecs) -> np.ndarray:
    """Random nonnegative combination of the coupling vectors, normalized."""
    vecs_normed = [np.asarray(v) / np.linalg.norm(v) for v in vecs]
    distrib = np.random.rand(len(vecs))
    distrib = distrib / np.linalg.norm(distrib)
    vec = np.sum([distrib[k] * vecs_normed[k] for k in range(len(vecs))], axis=0)
    return vec * normd


def load_assumed_jac(path):
    """Load a previously-fit Jacobian for extrapolation bootstrapping.

    Returns ``(jac, jac_x0)``; with ``path=None`` returns ``(None, None)`` so
    the calibration starts from the plain origin.
    """
    if path is None:
        return None, None
    d = np.load(path, allow_pickle=True)
    jac = np.array(d["jac"])
    jac_x0 = np.array(d["x0"]) if "x0" in d else None
    return jac, jac_x0


def run_jacobian_calibration(
    cfg: Config,
    master: Optional[str] = None,
    offset_type: Optional[str] = None,
    norm: Optional[float] = None,
    n_iterations: Optional[int] = None,
    plot: bool = False,
    stack: Optional[AlignerStack] = None,
) -> None:
    """Run the calibration loop (config values, optionally overridden)."""
    jac_cfg = cfg.jacobian
    master = master if master is not None else jac_cfg.master
    offset_type = offset_type if offset_type is not None else jac_cfg.offset_type
    normd = norm if norm is not None else jac_cfg.norm
    n = n_iterations if n_iterations is not None else jac_cfg.n_iterations

    if master not in jac_cfg.paths:
        raise ValueError(f"master {master!r} not in jacobian.paths {jac_cfg.paths}")
    others = [p for p in jac_cfg.paths if p != master]
    if len(others) != 1:
        raise ValueError(
            f"jacobian.paths must contain exactly two paths, got {jac_cfg.paths}"
        )
    slave = others[0]  # the path that gets re-optimized

    own_stack = stack is None
    if own_stack:
        stack = build_stack(cfg)
    layout = stack.layout
    measurement = stack.measurement

    output_dir = cfg.resolve_dir(jac_cfg.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    stack.actuator.de_hysteresis = True  # calibration accuracy depends on it
    stack.actuator.torque_enable()

    jac_assume, jac_x0 = load_assumed_jac(jac_cfg.assumed_jacobian)

    trace_sink = None
    if plot:
        from .. import plotting

        counter = {"n": 0}

        def trace_sink(trace):  # noqa: F811
            import matplotlib.pyplot as plt

            fig = plotting.plot_optimization_trace(trace)
            fig.savefig(output_dir / f"trace_{counter['n']:03d}.png", dpi=150)
            plt.close(fig)
            counter["n"] += 1

    try:
        baseline = measurement.measure(
            [0.0] * layout.n, layout.all, zero=np.zeros(layout.n)
        )
        logger.info("baseline: %s", (baseline,))

        offset_group = layout.group(f"{master}_POS_ALL")
        offset_mask = list(offset_group.mask)

        dataset = None
        filename = None
        if offset_type in ("pm", "rand", "lin"):
            filename = output_dir / f"jacobian_{offset_type}_{normd:g}.npy"
            if not os.path.exists(filename):
                np.save(filename, defaultdict(list))
            dataset = np.load(filename, allow_pickle=True).item()

        for i in range(n):
            zero = np.zeros(layout.n, dtype=float)

            if offset_type == "pm":
                offset = cord_pm_offset(offset_group.n, normd, i)
            elif offset_type == "rand":
                offset = random_norm_offset(offset_group.n, normd)
            elif offset_type == "lin":
                vecs = jac_cfg.lin_comb_vectors.get(master)
                if not vecs:
                    raise ValueError(
                        f"jacobian.lin_comb_vectors has no entry for path {master!r}"
                    )
                offset = lin_comb_offset(normd, vecs)
            elif offset_type == "zero":
                offset = np.zeros(offset_group.n)
            elif offset_type == "spec":
                offset = np.array(jac_cfg.spec_offset, dtype=float)
            else:
                raise ValueError(f"unknown offset_type: {offset_type!r}")

            zero = compose_para(
                para=offset,
                pos_mask=offset_mask,
                zero=zero,
                jac=jac_assume,
                jac_master_mask=offset_mask,
                jac_x0=jac_x0,
            )
            logger.info("Offset = %s, Zero = %s", offset, zero)
            logger.info("Start optimization with zero = %s", zero)

            for st in jac_cfg.stages:
                group = layout.group(f"{slave}_{st.group_suffix}")
                zero = step_optimize(
                    measurement,
                    group,
                    zero,
                    opt=cfg.optimize,
                    method=st.method,
                    bounds_single=st.bounds,
                    trace_sink=trace_sink,
                )

            _, intensity = measurement.measure(list(zero), layout.all)
            logger.info("final I: %s (i=%d)", intensity, i)

            if dataset is not None:
                dataset[tuple(offset)].append((list(zero), intensity))
                np.save(filename, dataset)
    finally:
        if own_stack:
            stack.close()
