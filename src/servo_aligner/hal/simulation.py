"""Hardware-free backend: a simulated actuator and photodiode.

Lets the entire alignment stack (scans, spiral descent, Jacobian
calibration, CLI) run on a dev machine. The sensor evaluates the
:class:`~servo_aligner.sim.beam_model.BeamClipModel` physics on the
simulated pose, so the optimization problem has the same structure as the
lab signal (flat-topped clip profile, knob crosstalk).
"""

from __future__ import annotations

import logging
from typing import List, Optional, Sequence

import numpy as np

from ..channels import ChannelLayout
from ..config import ActuatorConfig, SimulationConfig
from ..sim.beam_model import BeamClipModel

logger = logging.getLogger(__name__)

DEG2RAD = np.pi / 180


class SimulatedActuator:
    """Pure-state actuator; records every command for test assertions."""

    def __init__(self, cfg: ActuatorConfig):
        self.cfg = cfg
        self.de_hysteresis = cfg.de_hysteresis.enabled  # recorded, no effect
        self._names = tuple(ch.name for ch in cfg.channels)
        self.angles = np.zeros(len(self._names))
        #: coordinate offset accumulated by set_zero(): the physical pose is
        #: ``angles + zero_offset`` while reported angles restart at 0
        self.zero_offset = np.zeros(len(self._names))
        self.torque_on = False
        self.closed = False
        self.command_log: List[np.ndarray] = []

    @property
    def n_channels(self) -> int:
        return len(self._names)

    @property
    def channel_names(self):
        return self._names

    @property
    def physical_angles(self) -> np.ndarray:
        return self.angles + self.zero_offset

    def set_angles(
        self, angles_deg: Sequence[float], mask: Optional[Sequence[int]] = None
    ) -> None:
        goal = np.array(angles_deg, dtype=float)
        if len(goal) != self.n_channels:
            raise ValueError(
                f"Goal list length {len(goal)} does not match the number of "
                f"channels {self.n_channels}"
            )
        if mask is not None:
            goal = np.where(np.array(mask, dtype=bool), goal, self.angles)
        self.angles = goal
        self.command_log.append(goal.copy())

    def get_angles(self) -> np.ndarray:
        return self.angles.copy()

    def set_single(self, index: int, angle_deg: float) -> None:
        mask = [0] * self.n_channels
        mask[index] = 1
        angles = [0.0] * self.n_channels
        angles[index] = angle_deg
        self.set_angles(angles, mask)

    def home(self) -> None:
        self.set_angles([0.0] * self.n_channels)

    def set_zero(self) -> None:
        # the pose stays physically where it is; the coordinates shift
        self.zero_offset = self.zero_offset + self.angles
        self.angles = np.zeros(self.n_channels)
        logger.info("Simulated zero set; physical offset now %s", self.zero_offset)

    def torque_enable(self) -> None:
        self.torque_on = True

    def torque_disable(self) -> None:
        self.torque_on = False

    def close(self) -> None:
        self.closed = True

    def __enter__(self) -> "SimulatedActuator":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


class SimulatedIntensitySensor:
    """Evaluates the beam-clip physics on the simulated pose.

    Each configured beam path maps a channel quadruple — in
    ``(x, y, xdot, ydot)`` order — through a :class:`BeamClipModel`; the
    weighted transmissions are summed and Gaussian noise is added. Intensity
    peaks where the pose equals ``simulation.true_zero``.
    """

    def __init__(
        self,
        actuator: SimulatedActuator,
        layout: ChannelLayout,
        sim: SimulationConfig,
        samples_per_read: int = 2,
    ):
        self.actuator = actuator
        self.samples_per_read = samples_per_read
        self.noise_rms = sim.noise_rms
        self._rng = np.random.default_rng(sim.seed)
        self.model = BeamClipModel(
            **dict(sim.model), smooth_transition=sim.smooth_transition
        )
        if not sim.paths:
            raise ValueError(
                "simulation.paths must define at least one channel quadruple"
            )
        self._paths = [
            ([layout.index(ch) for ch in path.channels], path.weight)
            for path in sim.paths
        ]
        if sim.true_zero:
            self.true_zero = np.array(sim.true_zero, dtype=float)
        else:
            self.true_zero = np.zeros(layout.n)

    def read(self) -> float:
        offset_deg = self.actuator.physical_angles - self.true_zero
        total = 0.0
        for indices, weight in self._paths:
            knobs_rad = offset_deg[indices] * DEG2RAD
            total += weight * self.model.transmission(knobs_rad)
        return float(total + self.noise_rms * self._rng.standard_normal())

    def read_averaged(self, n: Optional[int] = None) -> float:
        if n is None:
            n = self.samples_per_read
        return float(np.mean([self.read() for _ in range(n)]))

    def close(self) -> None:
        pass
