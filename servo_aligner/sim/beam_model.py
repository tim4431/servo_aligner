"""Physics model of beam clipping through a double aperture.

Simulates the transmitted intensity of a laser beam steered by two mirrors
(four knobs: x, y, xdot, ydot) through a pair of apertures, including the
geometric crosstalk between knob pairs that motivates the Jacobian
optimization (see ``doc/simulation.md``).

``BeamClipModel`` parametrizes what used to be module-level constants in the
legacy ``numeric_sim.py``; :func:`calc_data` reproduces the legacy 2D-slice
scan exactly and keeps the exploratory notebooks runnable.
"""

from __future__ import annotations

from typing import Callable, Optional, Sequence, Union

import numpy as np
from scipy.special import erfc

CrosstalkFn = Callable[[float, float], np.ndarray]


def crosstalk_matrix(tknob1: float, tknob2: float) -> np.ndarray:
    """Legacy default constant crosstalk matrix."""
    return np.array([[0.1, 0], [0, 0.3]])


def crosstalk_matrix_rot(tknob1: float, tknob2: float) -> np.ndarray:
    """Legacy knob-dependent (rotating) crosstalk matrix."""
    return np.array([[0.1 + tknob1 / 6, 0], [0, 0.3 + tknob1 / 10]])


def make_grid(range_deg: float = 400.0, n: int = 80):
    """Knob-angle scan grid matching the legacy module-level ``X, Y``."""
    rng = range_deg * (np.pi / 180)
    t1 = np.linspace(-rng, rng, n)
    t2 = np.linspace(-rng, rng, n)
    return np.meshgrid(t1, t2)


class BeamClipModel:
    """Beam transmission through two apertures, steered by 4 mirror knobs.

    Args:
        L: Separation of the two test planes (m).
        d: Aperture diameter (m).
        L1: Lever arm of the first mirror (m).
        L2: Lever arm of the second mirror (m).
        knob_pitch: Mirror displacement per knob turn (m / 2π rad of knob).
        crosstalk: ``(tknob1, tknob2) -> 2x2`` coupling of the x-knob pair
            into the y mirror angles (or a constant 2x2 array). Defaults to
            the legacy constant matrix.
        smooth_transition: ``None`` (default) for the physical hard-edged
            aperture; a positive number substitutes an erfc-smoothed edge
            (relative width) so gradient-based tests converge. **The smooth
            variant is a test-only knob, not lab physics.**
    """

    def __init__(
        self,
        L: float = 16e-3,
        d: float = 1.4e-3,
        L1: float = 0.2,
        L2: float = 0.2 * 2.35,  # legacy expression, not 0.47: bit-parity matters

        knob_pitch: float = 8e-3,
        crosstalk: Optional[Union[CrosstalkFn, Sequence[Sequence[float]]]] = None,
        smooth_transition: Optional[float] = None,
    ):
        self.L = L
        self.d = d
        self.L1 = L1
        self.L2 = L2
        self.knob_pitch = knob_pitch
        self.smooth_transition = smooth_transition
        if crosstalk is None:
            self.crosstalk: CrosstalkFn = crosstalk_matrix
        elif callable(crosstalk):
            self.crosstalk = crosstalk
        else:
            cm = np.array(crosstalk, dtype=float)
            self.crosstalk = lambda tknob1, tknob2: cm

    # ------------------------------------------------------------- geometry

    def _window(self, r):
        """Aperture transmission for a radial offset ``r`` (legacy boxcar)."""
        if self.smooth_transition is None:
            return np.where(np.abs(r) < self.d / 2, 1, 0)
        return 0.5 * erfc((np.abs(r) - self.d / 2) / (self.smooth_transition * self.d))

    def t_mirror(self, t):
        """Knob angle (rad) to mirror tilt via the knob pitch."""
        return self.knob_pitch * (t / (2 * np.pi))

    def g(self, x, y, tx, ty):
        """Transmission of a beam at ``(x, y)`` with tilts ``(tx, ty)``."""
        x1 = x + (self.L / 2) * np.tan(tx)
        x2 = x - (self.L / 2) * np.tan(tx)
        y1 = y + (self.L / 2) * np.tan(ty)
        y2 = y - (self.L / 2) * np.tan(ty)
        return self._window(np.sqrt(x1**2 + y1**2)) * self._window(
            np.sqrt(x2**2 + y2**2)
        )

    def _beam(self, tx1, tx2, ty1, ty2):
        """Beam position and transmission from the four mirror tilts."""
        tx = tx1 + tx2
        ty = ty1 + ty2
        x = -self.L1 * np.tan(tx1 * 2) + self.L2 * np.tan(tx * 2)
        y = -self.L1 * np.tan(ty1 * 2) + self.L2 * np.tan(ty * 2)
        return x, y, tx, ty

    # --------------------------------------------------------- transmission

    def transmission(self, knobs_rad, zero_rad=None) -> float:
        """Transmitted intensity for knob angles ``[x, y, xdot, ydot]`` (rad).

        Generalizes the legacy ``calc_data`` x/xdot slice to all four knobs:
        the x-knob pair sets the x mirror tilts, the crosstalk matrix couples
        them into the y tilts on top of the y-knob pair.
        """
        k = np.asarray(knobs_rad, dtype=float)
        if zero_rad is not None:
            k = k + np.asarray(zero_rad, dtype=float)
        kx, ky, kxdot, kydot = k

        tx1 = self.t_mirror(kx)
        tx2 = self.t_mirror(kxdot)
        cm = self.crosstalk(kx, kxdot)
        ty_extra = np.dot(cm, np.array([tx1, tx2]))
        ty1 = self.t_mirror(ky) + ty_extra[0]
        ty2 = self.t_mirror(kydot) + ty_extra[1]

        x, y, tx, ty = self._beam(tx1, tx2, ty1, ty2)
        return float(self.g(x, y, tx, ty))


def calc_data(
    crosstalk_matrix: CrosstalkFn,
    zero,
    scan_type: str = "xxdot",
    range_deg: float = 400.0,
    n: int = 80,
    model: Optional[BeamClipModel] = None,
):
    """Legacy 2D-slice scan: returns ``(Z, xZ, tZ)`` over the knob grid.

    Byte-for-byte port of the legacy ``numeric_sim.calc_data`` (including its
    convention of calling ``crosstalk_matrix`` with the *scan* coordinates,
    excluding ``zero``). ``zero`` is the 4-vector ``[x, y, xdot, ydot]`` of
    knob offsets in radians.
    """
    if model is None:
        model = BeamClipModel(crosstalk=crosstalk_matrix)
    else:
        model = BeamClipModel(
            L=model.L,
            d=model.d,
            L1=model.L1,
            L2=model.L2,
            knob_pitch=model.knob_pitch,
            crosstalk=crosstalk_matrix,
            smooth_transition=model.smooth_transition,
        )
    X, Y = make_grid(range_deg, n)
    Z = np.zeros((n, n))
    xZ = np.zeros((n, n))
    tZ = np.zeros((n, n))

    for i in range(n):
        for j in range(n):
            tknob1 = X[i, j]
            tknob2 = Y[i, j]
            if scan_type == "xxdot":
                tx1 = model.t_mirror(tknob1 + zero[0])
                tx2 = model.t_mirror(tknob2 + zero[2])
                tx_vec = np.array([tx1, tx2])
                cm = crosstalk_matrix(tknob1, tknob2)
                ty_vec = np.dot(cm, tx_vec)
                ty1 = model.t_mirror(zero[1]) + ty_vec[0]
                ty2 = model.t_mirror(zero[3]) + ty_vec[1]
            elif scan_type == "yydot":
                ty1 = model.t_mirror(tknob1 + zero[1])
                ty2 = model.t_mirror(tknob2 + zero[3])
                ty_vec = np.array([ty1, ty2])
                cm = crosstalk_matrix(tknob1, tknob2)
                tx_vec = np.dot(cm, ty_vec)
                tx1 = model.t_mirror(zero[0]) + tx_vec[0]
                tx2 = model.t_mirror(zero[2]) + tx_vec[1]
            else:
                raise ValueError(f"unknown scan_type: {scan_type}")
            #
            x, y, tx, ty = model._beam(tx1, tx2, ty1, ty2)
            transmission = model.g(x, y, tx, ty)
            Z[i, j] = transmission
            xZ[i, j] = x * transmission
            tZ[i, j] = tx * transmission
    return Z, xZ, tZ
