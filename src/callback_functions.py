"""Objective (callback) functions for the optimizers, plus the photodiode ADC.

Every optimizer maximizes a scalar *intensity* returned by a ``callback_func``,
which does two things on each evaluation:

1. compose the reduced step ``para`` into a full angle command and move the
   servos (common to every objective), then
2. read an **objective function** — the signal being maximized.

Every objective is called with the full commanded **angle** vector
``para_nr_move`` (the pose just sent to the servos) as an optional keyword, so an
objective may depend on the commanded pose — e.g. the hardware-free
:func:`dummy_gaussian`, a Gaussian landscape in encoder-count space. Objectives
that read a real sensor (``intensity_adc``) simply ignore it.

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

from config import ADC, SERVO_CHANNEL_LIST
from servo_util import a2p, compose_para


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
    returns ``(tuple(para), intensity)`` with
    ``intensity = objective_func(para_nr_move=para_nr_move)`` — the composed
    full-angle command is handed to the objective so it can depend on the pose.
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
            return tuple(para), objective_func(para_nr_move=para_nr_move)
    return callback_func



@objective("intensity_adc")
def intensity_adc(para_nr_move=None, n_avg=2):
    """Photodiode intensity: the mean of ``n_avg`` MCP3424 reads.

    ``para_nr_move`` is accepted (the common objective signature) but ignored --
    the photodiode reading doesn't depend on the commanded pose directly.
    """
    samples = [MCP3424_fiber.convert_and_read() for _ in range(n_avg)]
    return float(np.mean(samples))


# dummy_gaussian's synthetic landscape: it peaks when the commanded pose (in
# encoder counts) matches DUMMY_GAUSSIAN_TARGET_ND. Edit the target/width to move
# or reshape the optimum. The target is a full-length nd vector (one per channel).
DUMMY_GAUSSIAN_TARGET_DEG = 10.0        # peak at +10 deg on every channel
DUMMY_GAUSSIAN_TARGET_ND = np.array(
    [a2p(DUMMY_GAUSSIAN_TARGET_DEG)] * len(SERVO_CHANNEL_LIST), dtype=float)
DUMMY_GAUSSIAN_SIGMA_COUNTS = 400.0     # Gaussian width, in encoder counts


@objective("dummy_gaussian")
def dummy_gaussian(para_nr_move=None, sigma=DUMMY_GAUSSIAN_SIGMA_COUNTS,
                   amplitude=1.0, noise=0.01):
    """Hardware-free objective: a Gaussian beam in encoder-count space (no ADC).

    The signal peaks when the commanded pose matches ``DUMMY_GAUSSIAN_TARGET_ND``:
    it converts the full commanded **angle** vector ``para_nr_move`` to encoder
    counts (``nd``), takes the difference ``r = nd - DUMMY_GAUSSIAN_TARGET_ND``, and
    returns ``amplitude * exp(-|r|^2 / (2 sigma^2))`` plus a little read noise --
    a smooth, deterministic optimization landscape with a known optimum, so the
    optimizer stack (and the console's objective displays) can be exercised without
    the photodiode. ``para_nr_move=None`` (a bare display read, no move) evaluates
    at the origin, 0 deg on every channel.
    """
    if para_nr_move is None:
        para_nr_move = np.zeros(len(SERVO_CHANNEL_LIST))
    nd = np.array([a2p(a) for a in para_nr_move], dtype=float)
    r = nd - DUMMY_GAUSSIAN_TARGET_ND
    r2 = float(np.dot(r, r))
    return float(amplitude * np.exp(-r2 / (2.0 * sigma ** 2)) + random.gauss(0.0, noise))

