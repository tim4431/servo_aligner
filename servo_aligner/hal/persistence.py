"""Persistence of servo encoder positions across restarts.

The JSON shape is unchanged from the legacy ``servos_<board>.json`` so
existing state files migrate by copying them into the configured
``state_dir``.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import List, Optional, Sequence

logger = logging.getLogger(__name__)


class PositionStore:
    """Stores the last-known encoder positions of one servo board."""

    def __init__(self, path: Path):
        self.path = Path(path)

    def load(self) -> Optional[List[int]]:
        """Return the stored positions, or None if there is no usable file."""
        if not self.path.exists():
            logger.info("No position data found on disk (%s)", self.path)
            return None
        try:
            dct = json.loads(self.path.read_text())
            return [int(p) for p in dct["position"]]
        except Exception as e:
            logger.error("Error loading position from disk: %s", e)
            return None

    def save(self, positions: Sequence[int], angles_deg: Sequence[float]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        dct = {
            "position": [int(p) for p in positions],
            "angles_deg": [float(a) for a in angles_deg],
        }
        self.path.write_text(json.dumps(dct))
