"""The shared move-and-measure callback at the heart of every routine.

Replaces the ``callback_func`` that was copy-pasted across the legacy
``clip_scan.py`` and ``calibrate_jacobian.py``: compose the full angle
command (optionally with Jacobian master→slave coupling), move the actuator,
read the intensity sensor, and return ``(para, intensity)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

from .channels import ChannelGroup
from .hal.interfaces import Actuator, IntensitySensor
from .vectors import compose_para


@dataclass(frozen=True)
class JacobianCoupling:
    """A calibrated master→slave knob coupling: ``dB = J(dA − offset) + x0``."""

    matrix: np.ndarray
    master: ChannelGroup
    master_offset: Optional[np.ndarray] = None
    x0: Optional[np.ndarray] = None


class Measurement:
    """Bundles an actuator and a sensor into objective functions."""

    def __init__(self, actuator: Actuator, sensor: IntensitySensor):
        self.actuator = actuator
        self.sensor = sensor

    def compose(
        self,
        para: Sequence[float],
        group: ChannelGroup,
        zero=None,
        coupling: Optional[JacobianCoupling] = None,
        debug: bool = False,
    ) -> np.ndarray:
        """The full angle command this measurement would move to."""
        return compose_para(
            para,
            list(group.mask),
            zero,
            jac=coupling.matrix if coupling else None,
            jac_master_mask=list(coupling.master.mask) if coupling else None,
            jac_master_offset=coupling.master_offset if coupling else None,
            jac_x0=coupling.x0 if coupling else None,
            debug=debug,
        )

    def measure(
        self,
        para: Sequence[float],
        group: ChannelGroup,
        zero=None,
        coupling: Optional[JacobianCoupling] = None,
    ) -> Tuple[tuple, float]:
        """Move to ``compose(para, ...)``, read intensity, return ``(para, I)``."""
        angles = self.compose(para, group, zero, coupling)
        self.actuator.set_angles(list(angles))
        z = self.sensor.read_averaged()
        return tuple(para), z

    def objective(
        self,
        group: ChannelGroup,
        zero=None,
        coupling: Optional[JacobianCoupling] = None,
    ) -> Callable[[Sequence[float]], Tuple[tuple, float]]:
        """An optimizer-ready closure over (group, zero, coupling)."""
        return lambda para: self.measure(para, group, zero=zero, coupling=coupling)
