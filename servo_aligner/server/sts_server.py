"""The servo-aligner ZMQ server for the expctl lab framework.

Port of the legacy ``STSServer.py``: during the QUEUE phase the requested
channel values from the pickled ``Sequence`` are applied as servo angles.
The actuator is injected (built by the factory) instead of constructed here;
the legacy one-shot startup subcommands (set_zero/home/...) are now regular
``servo-aligner`` CLI commands.
"""

from __future__ import annotations

import logging

from ..config import Config
from ..factory import build_actuator
from ..hal.interfaces import Actuator
from .base import Server

logger = logging.getLogger(__name__)

BANNER = """
===========================================
==           Servo Aligner Server        ==
==                STS3032                ==
===========================================
"""


class STSServer(Server):
    def __init__(self, cfg: Config, actuator: Actuator, message: str = BANNER):
        super().__init__(
            cfg.server.name,
            cfg.server.port,
            message,
            sequence_aliases=cfg.server.sequence_module_aliases,
        )
        self.actuator = actuator
        self.actuator.torque_enable()

    def set_angle(self):
        value_list = []
        mask = [0 for _ in range(self.actuator.n_channels)]
        if self.actuator.n_channels != len(self.seq.allChannels):
            logger.error(
                "Servoaligner received sequence with %s channels, but only %s "
                "channels are connected",
                len(self.seq.allChannels),
                self.actuator.n_channels,
            )
            return
        for i in range(self.actuator.n_channels):
            chan = self.seq.allChannels[i]
            if chan is not None:
                mask[i] = 1
                set_val = chan._TransValues[0]
                val = set_val[1]  # the first value
                value_list.append(float(val))
                if set_val[3] != val:  # non-identical values in the sequence
                    logger.warning(
                        "Servoaligner given multiple settings in same sequence, "
                        "but only takes the first!!"
                    )
            else:
                value_list.append(0)

        logger.debug("mask: %s values: %s", mask, value_list)
        # legacy behavior: the mask is logged but NOT applied — channels with
        # no sequence entry are driven to 0 degrees
        self.actuator.set_angles(value_list)

    def queue(self):
        self.set_angle()  # move the stage during the QUEUE phase
        return 1

    def run(self):
        return 1

    def plotdata(self):
        return [0], [0]


def serve(cfg: Config, de_hysteresis: bool = True) -> None:
    """Build the actuator, start the server, run until interrupted."""
    actuator = build_actuator(cfg)
    actuator.de_hysteresis = de_hysteresis
    server = STSServer(cfg, actuator)
    try:
        server.main_loop()
    finally:
        actuator.close()
