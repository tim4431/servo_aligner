"""Hardware abstraction interfaces.

The alignment stack talks to hardware exclusively through these two
protocols, so backends are swappable: real servos/ADC on the Pi, the
simulation backend on a dev machine, or any future device. The API speaks
**degrees** — encoder counts are an implementation detail of the STS3032
backend.

Backends do not need to inherit from these classes; any object with the
right methods satisfies the protocol (``typing.Protocol``).
"""

from __future__ import annotations

from typing import Optional, Protocol, Sequence, runtime_checkable

import numpy as np


@runtime_checkable
class Actuator(Protocol):
    """A set of positioning channels (e.g. servo-driven mirror knobs)."""

    #: When True, negative moves take up backlash from a consistent side
    #: (implementation-specific; a no-op flag for backends without backlash).
    de_hysteresis: bool

    @property
    def n_channels(self) -> int: ...

    @property
    def channel_names(self) -> Sequence[str]: ...

    def set_angles(
        self, angles_deg: Sequence[float], mask: Optional[Sequence[int]] = None
    ) -> None:
        """Move channels to absolute angles; unmasked channels hold position."""
        ...

    def get_angles(self) -> np.ndarray:
        """Current angles of all channels, in degrees."""
        ...

    def set_single(self, index: int, angle_deg: float) -> None:
        """Move one channel, holding all others."""
        ...

    def home(self) -> None:
        """Move all channels to 0 degrees."""
        ...

    def set_zero(self) -> None:
        """Define the current pose as 0 degrees on all channels."""
        ...

    def torque_enable(self) -> None: ...

    def torque_disable(self) -> None: ...

    def close(self) -> None: ...

    def __enter__(self) -> "Actuator": ...

    def __exit__(self, exc_type, exc, tb) -> None: ...


@runtime_checkable
class IntensitySensor(Protocol):
    """A scalar intensity readout (e.g. a photodiode behind an ADC)."""

    def read(self) -> float:
        """One conversion."""
        ...

    def read_averaged(self, n: Optional[int] = None) -> float:
        """Mean of ``n`` conversions (default: the configured samples_per_read)."""
        ...

    def close(self) -> None: ...
