"""Vector helpers for masked servo-channel arithmetic.

Three vector representations are used throughout the alignment code
(see the channel-group docs in ``doc/optimize.md``):

- ``r``  — *reduced* vector: only the entries selected by a 0/1 mask
  (length ``sum(mask)``).
- ``nr`` — full-length **angle** vector in degrees (length = number of
  channels).
- ``nd`` — full-length **digital** encoder-position vector
  (``2048`` = 0 degrees, ``4096`` counts per turn).

The function names encode the conversion: ``r2nr`` embeds a reduced vector
into a full angle vector, ``nrselr`` extracts the masked entries, ``nraddr``
adds a reduced vector onto the masked entries, etc.

These numerics are lab-validated; do not change behavior.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

#: Encoder counts per full turn of an STS3032 servo.
COUNTS_PER_TURN = 4096
#: Encoder count corresponding to 0 degrees.
ZERO_COUNT = 2048


def a2p(angle: float) -> int:
    """Convert an angle in degrees to a digital encoder position."""
    return int(angle * (COUNTS_PER_TURN / 360) + ZERO_COUNT)


def create_zigzag_X(X: np.ndarray):
    """Reverse every other row of a scan grid for serpentine scanning.

    Returns ``(X_zigzag, index_map)`` where ``index_map[i, j]`` is the flat
    index into the original ``X`` of the element now at ``X_zigzag[i, j]``.
    """
    X_zigzag = np.copy(X)
    # track the original flat index of every element in X_zigzag
    index_map = np.zeros_like(X, dtype=int)

    for i in range(X_zigzag.shape[0]):
        if i % 2 == 0:
            X_zigzag[i, :] = X[i, :]
            index_map[i, :] = np.arange(X_zigzag.shape[1]) + i * X_zigzag.shape[1]
        else:
            X_zigzag[i, :] = np.flip(X[i, :])
            index_map[i, :] = np.flip(np.arange(X_zigzag.shape[1])) + i * X_zigzag.shape[1]
    return X_zigzag, index_map


def r2nd(r: Sequence[float], r_mask: Optional[Sequence[int]] = None) -> np.ndarray:
    """Embed a reduced angle vector into a full digital-position vector.

    Unmasked entries are set to ``a2p(0)`` (= 2048).
    """
    if r_mask is None:
        r_mask = np.ones_like(r, dtype=np.bool_)
    r = list(r)
    assert np.sum(r_mask) == len(r), ValueError(
        "r2nd: r_mask should have the same 1 as len(r)"
    )
    pos = np.ones_like(r_mask, dtype=np.int_) * a2p(0)
    for i in range(len(r_mask)):
        if r_mask[i]:
            pos[i] = a2p(r.pop(0))
    return pos.astype(np.int_)


def r2nr(r: Sequence[float], r_mask: Optional[Sequence[int]] = None) -> np.ndarray:
    """Embed a reduced vector into a full angle vector (zero-filled)."""
    if r_mask is None:
        r_mask = np.ones_like(r, dtype=np.bool_)
    r = list(r)
    assert np.sum(r_mask) == len(r), ValueError(
        "r2nr: r_mask should have the same 1 as len(r)"
    )
    pos = np.zeros_like(r_mask, dtype=np.float64)
    for i in range(len(r_mask)):
        if r_mask[i]:
            pos[i] = r.pop(0)
    return pos


def nrselr(pos: Sequence[float], r_mask: Sequence[int]) -> np.ndarray:
    """Extract the masked entries of a full vector as a reduced vector."""
    r = []
    for i in range(len(r_mask)):
        if r_mask[i]:
            r.append(pos[i])
    return np.array(r)


def nrmodr(
    r_origin: Sequence[float],
    r_mod: Sequence[float],
    r_mask: Optional[Sequence[int]],
) -> np.ndarray:
    """Overwrite the masked entries of ``r_origin`` with the reduced ``r_mod``."""
    if r_mask is None:
        r_mask = np.ones_like(r_origin, dtype=np.bool_)
    r_mod = list(r_mod)
    r_origin = np.array(r_origin)
    assert np.sum(r_mask) == len(r_mod), ValueError(
        "nrmodr: r_mask should have the same length as r_mod"
    )
    for i in range(len(r_mask)):
        if r_mask[i]:
            r_origin[i] = r_mod.pop(0)
    return r_origin


def nraddr(
    r_origin: Sequence[float],
    r_add: Sequence[float],
    r_mask: Optional[Sequence[int]] = None,
) -> np.ndarray:
    """Add the reduced ``r_add`` onto the masked entries of ``r_origin``."""
    if r_mask is None:
        r_mask = np.ones_like(r_origin, dtype=np.bool_)
    r_add = list(r_add)
    r_origin = np.array(r_origin)
    assert np.sum(r_mask) == len(r_add), ValueError(
        "nraddr: r_mask should have the same length as r_add"
    )
    for i in range(len(r_mask)):
        if r_mask[i]:
            r_origin[i] += r_add.pop(0)
    return r_origin


def ndmodr(
    pos_origin: Sequence[int],
    r_mod: Sequence[float],
    r_mask: Sequence[int],
) -> np.ndarray:
    """Overwrite masked entries of a digital-position vector with ``a2p(r_mod)``."""
    r_mod = list(r_mod)
    pos_origin = np.array(pos_origin)
    assert np.sum(r_mask) == len(r_mod), ValueError(
        "r_mask should have the same length as r_mod"
    )
    for i in range(len(r_mask)):
        if r_mask[i]:
            pos_origin[i] = a2p(r_mod.pop(0))
    return pos_origin.astype(np.int_)


def format_para(para: Sequence[float]) -> str:
    """Format a parameter vector as ``x_0=...  x_1=...`` for log messages."""
    return " ".join("x_{:d}={:.2f}".format(i, para[i]) for i in range(len(para)))


def compose_para(
    para,
    pos_mask,
    zero=None,
    jac=None,
    jac_master_mask=None,
    jac_master_offset=None,
    jac_x0=None,
    debug: bool = False,
) -> np.ndarray:
    """Build a full angle command from a reduced parameter on top of ``zero``.

    The heart of every alignment objective. Starting from the full-length
    ``zero`` offset, the reduced ``para`` is added onto the channels selected
    by ``pos_mask``. If a Jacobian ``jac`` is supplied, the *slave* channels
    (complement of ``jac_master_mask``) are additionally displaced by
    ``dB = J @ (dA - jac_master_offset) + jac_x0`` where ``dA`` is the master
    part of ``para``.

    Args:
        para: Reduced parameter vector (length ``sum(pos_mask)``), or ``None``
            for all-zeros.
        pos_mask: 0/1 channel mask selecting which channels ``para`` acts on.
        zero: Full-length angle offset; defaults to zeros.
        jac: Master→slave coupling matrix, or ``None`` for no coupling.
        jac_master_mask: 0/1 mask of the master channels (required with ``jac``).
        jac_master_offset: Reduced offset subtracted from the master displacement.
        jac_x0: Constant added to the slave displacement.
        debug: Print intermediate vectors.

    Returns:
        Full-length angle command vector.
    """
    if para is None:
        para = np.zeros(len(pos_mask))
    if zero is None:
        zero = np.zeros(len(pos_mask))
    # dB = J (dA - jac_master_offset)
    if jac_master_offset is None:
        jac_master_offset = np.zeros(len(pos_mask))
    else:
        jac_master_offset = r2nr(jac_master_offset, jac_master_mask)

    para_nr_move = nraddr(zero, para, pos_mask)
    # set slave knobs according to jac
    if jac is not None:
        assert jac_master_mask is not None, "jac_master_mask is not provided"
        dr = r2nr(para, r_mask=pos_mask) - jac_master_offset
        dr = nrselr(dr, jac_master_mask)
        d_slave_r = np.dot(jac, dr)
        if jac_x0 is not None:
            d_slave_r = d_slave_r + jac_x0
        jac_slave_mask = 1 - np.array(jac_master_mask)
        para_nr_move = nraddr(para_nr_move, d_slave_r, jac_slave_mask)
        if debug:
            print(dr)
            print(d_slave_r)
            print(para_nr_move)
    return para_nr_move
