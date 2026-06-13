"""Clip scan: raster a knob pair, fit the beam-clip profile, re-center.

Port of the legacy ``clip_scan.py`` with all module-level hardware code
moved into :func:`run_clip_scan`. The scan/fit/accept numerics and the
output file naming (``clip_<group>_<iter>.npz`` / ``_popt.npz``) are
unchanged.
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Optional, Sequence

import numpy as np

from ..channels import ChannelGroup, ChannelLayout
from ..config import AcceptLine, ClipScanStage, Config
from ..factory import AlignerStack, build_stack
from ..fitting import fit_gaussian_2d_smooth_heaviside, popt_get_mu_cov
from ..measurement import Measurement
from ..scan.raster import raster_2d
from ..vectors import nraddr

logger = logging.getLogger(__name__)


def build_accept_func(line: AcceptLine) -> Callable:
    """Accept points within ``tol`` of the knob-coupling line (legacy)."""
    return lambda xy: np.abs(xy[1] - xy[0] * line.slope - line.intercept) < line.tol


def scan_and_analyze(
    measurement: Measurement,
    group: ChannelGroup,
    zero: np.ndarray,
    stage: ClipScanStage,
    iter_num: int,
    cfg: Config,
    plot: bool = True,
) -> np.ndarray:
    """One scan stage: raster, fit, move to the fitted center, update zero."""
    output_dir = cfg.resolve_dir(cfg.clip_scan.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    time.sleep(1)
    logger.info("SCANNING: %s %s", group.name, iter_num)
    objective = measurement.objective(group, zero=zero)

    accept_line = cfg.clip_scan.accept_lines.get(group.name)
    accept_func = None
    if stage.accept:
        if accept_line is None:
            raise KeyError(
                f"no clip_scan.accept_lines entry for group {group.name!r}"
            )
        accept_func = build_accept_func(accept_line)

    scan_start_time = time.time()
    result = raster_2d(objective, stage.n_pts, stage.range, accept_func=accept_func)
    scan_stop_time = time.time()
    measurement.actuator.home()  # the legacy scan homed after every raster

    file_name = "clip_{}_{}".format(group.name, iter_num)
    np.savez(
        output_dir / f"{file_name}.npz",
        X=result.X,
        Y=result.Y,
        Z=result.Z,
        posmaskstr=group.name,
        ITER_NUM=iter_num,
        scan_start_time=scan_start_time,
        scan_stop_time=scan_stop_time,
    )

    popt = fit_gaussian_2d_smooth_heaviside(result.X, result.Y, result.Z)
    mu, cov = popt_get_mu_cov(popt)
    eigvals, _ = np.linalg.eig(cov)
    sigmas = np.sqrt(eigvals)
    logger.info("popt: %s", popt)
    logger.info("sigmas: %s", sigmas)

    if plot:
        from .. import plotting

        plotting.plot_clip_fit(
            result,
            popt,
            mu,
            sigmas,
            file_name,
            accept_func=(
                build_accept_func(accept_line) if accept_line is not None else None
            ),
            plot_type=stage.plot_type,
            save_path=str(output_dir / f"{file_name}.png"),
        )

    # drive to the fitted center and verify the intensity there
    objective(list(mu))
    time.sleep(1)
    logger.info("going to mu: %s", mu)
    _, intensity = objective(list(mu))
    logger.info("I: %s", intensity)

    if intensity > cfg.clip_scan.I_meaningful:
        logger.info("calculating new zero point")
        zero_new = nraddr(zero, np.array(list(mu)), list(group.mask))
        logger.info("zero_new: %s", zero_new)
    else:
        logger.info("I too small, not setting zero point")
        zero_new = zero
    np.savez(
        output_dir / f"{file_name}_popt.npz",
        popt=popt,
        mu=mu,
        cov=cov,
        sigmas=sigmas,
        zero=zero_new,
    )

    logger.info("DONE SCANNING: %s %s", group.name, iter_num)
    return zero_new


def run_clip_scan(
    cfg: Config,
    plot: bool = True,
    iterations: Optional[Sequence[int]] = None,
    stack: Optional[AlignerStack] = None,
) -> np.ndarray:
    """Run the configured clip-scan stages; returns the final zero."""
    own_stack = stack is None
    if own_stack:
        stack = build_stack(cfg)
    layout: ChannelLayout = stack.layout
    actuator = stack.actuator
    measurement = stack.measurement

    actuator.de_hysteresis = False  # legacy clip scans run without it
    actuator.torque_enable()

    if iterations is None:
        iterations = cfg.clip_scan.iterations

    zero = np.zeros(layout.n, dtype=float)
    try:
        baseline = measurement.measure([0.0] * layout.n, layout.all, zero=zero)
        logger.info("baseline: %s", (baseline,))
        for iter_num in iterations:
            for stage in cfg.clip_scan.stages:
                group = layout.group(stage.group)
                zero = scan_and_analyze(
                    measurement, group, zero, stage, iter_num, cfg, plot=plot
                )
        return zero
    finally:
        actuator.home()
        if own_stack:
            stack.close()
