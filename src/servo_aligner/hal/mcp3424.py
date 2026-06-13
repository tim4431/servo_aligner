"""MCP3424 I2C ADC photodiode sensor.

Unlike the legacy ``pd.py``, the I2C bus is opened in the constructor — not
at import time — so this module is importable everywhere. Requires the
``hardware`` extra (smbus2 + MCP342x).
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from ..config import SensorConfig


class Mcp3424Sensor:
    """Photodiode behind an MCP3424 ADC (the optimization feedback signal)."""

    def __init__(self, cfg: SensorConfig):
        from smbus2 import SMBus

        import MCP342x

        self._bus = SMBus(cfg.i2c_bus)
        self._adc = MCP342x.MCP342x(
            self._bus,
            cfg.address,
            device="MCP3424",
            channel=cfg.channel,
            gain=cfg.gain,
            resolution=cfg.resolution,
            continuous_mode=False,
            scale_factor=1.0,
            offset=0.0,
        )
        self.samples_per_read = cfg.samples_per_read

    def read(self) -> float:
        return float(self._adc.convert_and_read())

    def read_averaged(self, n: Optional[int] = None) -> float:
        if n is None:
            n = self.samples_per_read
        data_cache = [self._adc.convert_and_read() for _ in range(n)]
        return float(np.mean(np.array(data_cache)))

    def close(self) -> None:
        self._bus.close()
