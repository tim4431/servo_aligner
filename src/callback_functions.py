"""Objective (callback) functions for the optimizers, plus the photodiode ADC.

Every optimizer maximizes a scalar *intensity* returned by a ``callback_func``,
which does two things on each evaluation:

1. compose the reduced step ``para`` into a full angle command and move the
   servos (common to every objective), then
2. read an **objective function** — the signal being maximized.

Objective functions are registered in :data:`OBJECTIVES` so one can be selected
by name or at random; ``intensity_adc`` reads the photodiode over I2C. The
MCP3424 ADC wiring (formerly ``pd.py``) lives here too, since the objective is
the only thing that touches it.

The MCP3424 ADC is opened at import, but defensively (see :func:`_open_adc`): if
the ADC — or even the I2C libraries — are absent, the module still imports and
reads return NaN via :func:`read_objective`, so the objective list can be shown
(e.g. on the servo console's objective page) on a machine without the hardware.
"""

import random
import numpy as np

try:  # the I2C libs are present on the Pi; absent on a dev machine
    from smbus2 import SMBus
    import MCP342x
except Exception:
    SMBus = None
    MCP342x = None

from config import ADC
from servo_util import compose_para


# --- Photodiode ADC (MCP3424 over I2C); wiring from machine.yaml (adc:) -------
def _open_adc():
    """Open the MCP3424 photodiode ADC.

    Returns ``(bus, device)``, or ``(None, None)`` when the I2C libraries or the
    ADC hardware are absent — so importing this module never fails on that
    account; reads of a ``None`` device then surface as NaN via
    :func:`read_objective` rather than crashing at import time.
    """
    if SMBus is None:
        return None, None
    try:
        bus = SMBus(ADC["i2c_bus"])
        device = MCP342x.MCP342x(
            bus, ADC["address"],
            device=ADC.get("device", "MCP3424"),
            channel=ADC.get("channel", 0),
            gain=ADC.get("gain", 1),
            resolution=ADC.get("resolution", 16),
            continuous_mode=False, scale_factor=1.0, offset=0.0,
        )
        return bus, device
    except Exception:
        return None, None


i2cbus, MCP3424_fiber = _open_adc()
# Extra MCP3424 channels, if wired (e.g. pinhole / reference photodiodes):
# MCP3424_pinhole = MCP342x.MCP342x(i2cbus, ADC["address"], device="MCP3424", channel=1, gain=1, resolution=16, continuous_mode=False)
# MCP3424_ref     = MCP342x.MCP342x(i2cbus, ADC["address"], device="MCP3424", channel=2, gain=4, resolution=16, continuous_mode=False)


# --- Objective functions: the signal to maximize -----------------------------
# Each returns a float intensity. Register new ones with @objective(name) so they
# can be picked by name or at random (e.g. a second sensor, or a ratio of two).
OBJECTIVES = {}


def objective(name):
    """Decorator registering an objective function under ``name`` in :data:`OBJECTIVES`."""
    def register(func):
        OBJECTIVES[name] = func
        return func
    return register


@objective("intensity_adc")
def intensity_adc(n_avg=2):
    """Photodiode intensity: the mean of ``n_avg`` MCP3424 reads."""
    samples = [MCP3424_fiber.convert_and_read() for _ in range(n_avg)]
    return float(np.mean(samples))


def get_objective(name=None):
    """Return the objective registered as ``name``; ``name=None`` picks one at random."""
    if name is None:
        name = random.choice(list(OBJECTIVES))
    return OBJECTIVES[name]


def read_objective(objective_func, *args, **kwargs):
    """Evaluate an objective, returning ``float('nan')`` on any failure.

    Objectives read hardware (the photodiode ADC over I2C); when the ADC is not
    installed or a read errors, callers that only want to *display* the current
    value — the servo console's objective monitor — get NaN instead of an
    exception. The optimizer's ``callback_func`` deliberately does **not** route
    through this: a failed read during optimization should surface, not silently
    poison the objective with NaN.
    """
    try:
        return float(objective_func(*args, **kwargs))
    except Exception:
        return float("nan")


# --- The optimizer callback: move the servos, then read the chosen objective --
def make_callback_func(servos, objective_func):
    """Build the optimizer callback for ``servos`` using ``objective_func``.

    ``objective_func`` is required (no default) so every caller states explicitly
    which signal it is maximizing.

    Returns ``callback_func(para, pos_mask, zero=None, jac=None,
    jac_master_mask=None, debug=False, **kwargs)``, which composes ``para`` into a
    full angle command (:func:`servo_util.compose_para`), moves the servos, and
    returns ``(tuple(para), intensity)`` with ``intensity = objective_func()``.
    With ``debug=True`` it only composes the command (no motion, no read) and
    returns ``None`` — preserving the original behavior.

    Pass a specific objective (``make_callback_func(servos, intensity_adc)``), one
    by name (``get_objective("intensity_adc")``), or a random one
    (``get_objective()``).
    """
    def callback_func(para, pos_mask, zero=None, jac=None, jac_master_mask=None, debug=False, **kwargs):
        para_nr_move = compose_para(para, pos_mask, zero, jac, jac_master_mask, debug=debug, **kwargs)
        if not debug:
            servos.set_angle(list(para_nr_move))
            return tuple(para), objective_func()
    return callback_func
