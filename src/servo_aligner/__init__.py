"""Automated laser-beam alignment with serial-bus servos and photodiode feedback.

The package is layered so that nothing here touches hardware at import time:

- :mod:`servo_aligner.hal` — hardware abstraction (STS3032 servos, MCP3424 ADC,
  and a hardware-free simulation backend).
- :mod:`servo_aligner.optimize` / :mod:`servo_aligner.scan` — the lab-validated
  alignment algorithms (spiral descent, raster scanning, step optimization).
- :mod:`servo_aligner.routines` — the clip-scan and Jacobian-calibration
  procedures, runnable via the ``servo-aligner`` CLI.
- :mod:`servo_aligner.server` — optional ZMQ adapter for the expctl lab
  framework (requires the ``server`` extra).

Hardware drivers are imported lazily by :mod:`servo_aligner.factory`, so
``import servo_aligner`` works on machines without smbus2/pyserial installed.
"""

__version__ = "1.0.0.dev0"
