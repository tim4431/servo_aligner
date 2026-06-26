"""Vector helpers mapping between *reduced* and *full* knob vectors.

One optimizer step acts on only a subset of the servo channels, picked by a 0/1
``mask`` (the channel grouping masks in ``config``). These helpers convert
between three vector
representations (names match the developer notes / CLAUDE.md):

* ``r``  — *reduced* vector: just the masked entries, length ``sum(mask)``.
* ``nr`` — full-length **angle** vector, in degrees (one entry per channel).
* ``nd`` — full-length **encoder-count** vector (the ``d`` is encoder counts, not
  "digital"). ``a2p`` maps an angle in degrees to a count (``ENCODER_CENTER`` =
  0 deg, ``COUNTS_PER_TURN`` / rev).

Helper naming convention:

===========  ===============================================================
``r2nr``     embed a reduced vector into a full **angle** vector
``r2nd``     embed a reduced vector into a full **encoder-count** vector
``nrselr``   extract the masked entries of a full vector back to reduced
``nrmodr``   overwrite the masked entries of a full vector
``nraddr``   add onto the masked entries of a full vector
``ndmodr``   overwrite the masked entries of a full **encoder-count** vector
===========  ===============================================================

This module is pure (no hardware imports) and safe to import.
"""

import numpy as np

# STS3032 12-bit encoder geometry. These are fixed by the servo hardware (not
# the machine), so they live in code rather than in the YAML config.
ENCODER_CENTER = 2048    # encoder count reported at 0 deg
COUNTS_PER_TURN = 4096   # encoder counts per full revolution
DEGREES_PER_TURN = 360


def _a2p_array(angles) -> np.ndarray:
    """Vectorised angle(deg) -> encoder count, truncated toward zero."""
    counts = np.asarray(angles, dtype=float) * (COUNTS_PER_TURN / DEGREES_PER_TURN) + ENCODER_CENTER
    return counts.astype(np.int_)


def a2p(angle) -> int:
    """Convert a single angle in degrees to an encoder count.

    ``ENCODER_CENTER`` (2048) is 0 deg and a full turn is ``COUNTS_PER_TURN``.
    """
    return int(_a2p_array(angle))


def _as_bool_mask(mask, like) -> np.ndarray:
    """Return ``mask`` as a boolean array; ``None`` selects every entry of ``like``."""
    if mask is None:
        return np.ones(len(like), dtype=bool)
    return np.asarray(mask, dtype=bool)


def _check_reduced_len(mask, values, who):
    """Raise if a reduced vector's length doesn't match the number of masked slots."""
    n_slots = int(np.count_nonzero(mask))
    if n_slots != len(values):
        raise ValueError(f"{who}: mask selects {n_slots} entries but got {len(values)} reduced values")


def create_zigzag_X(X):
    """Boustrophedon (zigzag) reordering of a 2D raster grid.

    Reverses every odd row of ``X`` so a raster scan can sweep back and forth
    instead of jumping back to the start of each row — minimising motor travel,
    the dominant cost on the hardware.

    Returns ``(X_zigzag, index_map)`` where ``index_map[i, j]`` is the flat index
    in the original ``X`` of the value now sitting at ``X_zigzag[i, j]`` (so a
    scan can write results back to the un-zigzagged grid).
    """
    X_zigzag = np.copy(X)
    index_map = np.zeros_like(X, dtype=int)
    n_cols = X.shape[1]
    for i in range(X.shape[0]):
        if i % 2 == 0:                                  # even rows: keep order
            X_zigzag[i, :] = X[i, :]
            index_map[i, :] = np.arange(n_cols) + i * n_cols
        else:                                           # odd rows: reverse order
            X_zigzag[i, :] = np.flip(X[i, :])
            index_map[i, :] = np.flip(np.arange(n_cols)) + i * n_cols
    return X_zigzag, index_map


def r2nd(r, r_mask=None) -> np.ndarray:
    """Embed reduced angles ``r`` into a full **encoder-count** vector.

    Masked positions become ``a2p(value)``; the rest stay at the 0-deg count
    ``ENCODER_CENTER``. Returns an int array of length ``len(r_mask)``.
    """
    mask = _as_bool_mask(r_mask, r)
    _check_reduced_len(mask, r, "r2nd")
    nd = np.full(len(mask), ENCODER_CENTER, dtype=np.int_)
    nd[mask] = _a2p_array(r)
    return nd


def r2nr(r, r_mask=None) -> np.ndarray:
    """Embed reduced angles ``r`` into a full **angle** (degrees) vector.

    Masked positions take their value; the rest are 0. Returns a float array of
    length ``len(r_mask)``.
    """
    mask = _as_bool_mask(r_mask, r)
    _check_reduced_len(mask, r, "r2nr")
    nr = np.zeros(len(mask), dtype=np.float64)
    nr[mask] = np.asarray(r, dtype=np.float64)
    return nr


def nrselr(nr, r_mask) -> np.ndarray:
    """Extract the masked entries of full vector ``nr`` into a reduced vector."""
    return np.asarray(nr)[_as_bool_mask(r_mask, nr)]


def nrmodr(nr, r_mod, r_mask=None) -> np.ndarray:
    """Return a copy of full vector ``nr`` with its masked entries overwritten by ``r_mod``."""
    mask = _as_bool_mask(r_mask, nr)
    _check_reduced_len(mask, r_mod, "nrmodr")
    out = np.array(nr)
    out[mask] = r_mod
    return out


def nraddr(nr, r_add, r_mask=None) -> np.ndarray:
    """Return a copy of full vector ``nr`` with reduced ``r_add`` added onto its masked entries."""
    mask = _as_bool_mask(r_mask, nr)
    _check_reduced_len(mask, r_add, "nraddr")
    out = np.array(nr)
    out[mask] += np.asarray(r_add)
    return out


def ndmodr(nd, r_mod, r_mask) -> np.ndarray:
    """Return a copy of full **encoder-count** vector ``nd`` with its masked entries set to ``a2p(r_mod)``."""
    mask = _as_bool_mask(r_mask, nd)
    _check_reduced_len(mask, r_mod, "ndmodr")
    out = np.array(nd)
    out[mask] = _a2p_array(r_mod)
    return out.astype(np.int_)


def format_para(para) -> str:
    """Format a parameter vector as ``"x_0=.. x_1=.."`` for log messages."""
    return " ".join("x_{:d}={:.2f}".format(i, para[i]) for i in range(len(para)))


def compose_para(para,
                 pos_mask,
                 zero=None,
                 jac=None,
                 jac_master_mask=None,
                 jac_master_offset=None,
                 jac_x0=None,
                 debug=False) -> np.ndarray:
    """Build the full-channel angle command for one optimizer step.

    Starts from the ``zero`` origin and adds the reduced step ``para`` on the
    channels picked by ``pos_mask``. If a Jacobian ``jac`` is given, the *slave*
    knobs follow the *master* knobs via ``d_slave = jac . (dA - jac_master_offset)``
    (plus an optional constant ``jac_x0``), where ``jac_master_mask`` selects the
    master channels and the slaves are everything else.

    Args:
        para: Reduced step over the ``pos_mask`` channels (``None`` -> no step).
        pos_mask: 0/1 mask of the channels this step drives.
        zero: Full-length angle origin to step from (``None`` -> zeros).
        jac: Coupling matrix mapping master deltas to slave deltas (``None`` -> no coupling).
        jac_master_mask: 0/1 mask of the master channels (required when ``jac`` is set).
        jac_master_offset: Reduced offset subtracted from the master delta before applying ``jac``.
        jac_x0: Constant slave offset added after ``jac``.
        debug: If True, print the intermediate master/slave deltas.

    Returns:
        Full-length angle command (degrees), one entry per channel.
    """
    if para is None:
        para = np.zeros(len(pos_mask))
    if zero is None:
        zero = np.zeros(len(pos_mask))
    # dB = jac . (dA - jac_master_offset)
    if jac_master_offset is None:
        jac_master_offset = np.zeros(len(pos_mask))
    else:
        jac_master_offset = r2nr(jac_master_offset, jac_master_mask)

    # step the masked (master) knobs off the origin
    para_nr_move = nraddr(zero, para, pos_mask)

    # drive the slave knobs to follow the masters according to the Jacobian
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


if __name__ == "__main__":
    # reduced [-5, 4] over mask [1,0,1,0] -> full encoder-count vector
    print(r2nd([-5, 4], [1, 0, 1, 0]))
