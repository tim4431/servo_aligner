"""1D / 2D raster scans of an objective over knob coordinates.

Port of the legacy ``motor_scan`` minus hardware concerns: the scan only
calls the objective. Homing the actuator and closing on error is the
calling routine's responsibility (in a ``finally``), and plotting moved to
:mod:`servo_aligner.plotting`.

The 2D scan walks the grid in a serpentine (zigzag) order to minimize knob
travel, then un-shuffles the samples back into grid order.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple, Union

import numpy as np
import tqdm

from ..vectors import create_zigzag_X

logger = logging.getLogger(__name__)

Objective = Callable[[Sequence[float]], Tuple[tuple, float]]
AcceptFunc = Callable[[Sequence[float]], bool]


@dataclass
class ScanResult:
    X: np.ndarray
    Y: np.ndarray
    Z: np.ndarray


def raster_1d(
    objective: Objective,
    n_pts: int,
    scan_range: float,
    scan_vec: Sequence[float],
    accept_func: Optional[AcceptFunc] = None,
    progress: bool = True,
):
    """Scan along the direction ``scan_vec``; returns ``(L, Z)``."""
    L = np.linspace(-scan_range, scan_range, n_pts)
    X = L * scan_vec[0]
    Y = L * scan_vec[1]
    Z = np.zeros_like(L)

    z0 = objective([0, 0])
    logger.info("z0: %s", z0)

    with tqdm.tqdm(total=len(L), disable=not progress) as pbar:
        for i in range(len(L)):
            x, y = X[i], Y[i]
            if (accept_func is None) or accept_func([x, y]):
                _, z = objective([x, y])
                Z[i] = z
            pbar.update(1)
    return L, Z


def raster_2d(
    objective: Objective,
    n_pts: Union[int, Tuple[int, int]],
    scan_range: Union[float, Tuple[float, float]],
    accept_func: Optional[AcceptFunc] = None,
    progress: bool = True,
) -> ScanResult:
    """Serpentine 2D scan over ``[-scan_range, scan_range]²``.

    Points rejected by ``accept_func`` are skipped (their Z stays 0) —
    used to restrict scans to the band around the knob-coupling line.
    """
    if isinstance(n_pts, tuple):
        n_pts_x, n_pts_y = n_pts
        if isinstance(scan_range, tuple):
            scan_range_x, scan_range_y = scan_range
        else:
            scan_range_x, scan_range_y = scan_range, scan_range
        Xs = np.linspace(-scan_range_x, scan_range_x, n_pts_x)
        Ys = np.linspace(-scan_range_y, scan_range_y, n_pts_y)
    else:
        Xs = np.linspace(-scan_range, scan_range, n_pts)
        Ys = np.linspace(-scan_range, scan_range, n_pts)

    X, Y = np.meshgrid(Xs, Ys)
    Z = np.zeros_like(X)
    X_zig, index_map = create_zigzag_X(X)
    z0 = objective([0, 0])
    logger.info("z0: %s", z0)

    with tqdm.tqdm(total=len(Xs) * len(Ys), disable=not progress) as pbar:
        for i in range(len(Ys)):
            for j in range(len(Xs)):
                x, y = X_zig[i, j], Y[i, j]
                idx = index_map[i, j]  # original index of X
                idx_i, idx_j = np.unravel_index(idx, X.shape)
                #
                if (accept_func is None) or accept_func([x, y]):
                    _, z = objective([x, y])
                    Z[idx_i, idx_j] = z
                pbar.update(1)

    logger.info("Z range: %s .. %s", np.min(Z), np.max(Z))
    return ScanResult(X=X, Y=Y, Z=Z)
