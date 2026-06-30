"""Centralized data storage for task output (scans, calibrations, ...).

Every task writes its output under the single ``data/`` root (``paths.data_folder``
in machine.yaml, resolved to ``<repo>/data`` by :mod:`config`). A
:class:`DataStore` is one named *run folder* under that root. It:

* creates the folder (``data/<name>/``),
* **hints when the folder already exists**, so a re-run does not silently
  overwrite an earlier one (the reason the configured subdirs used to carry a
  hand-bumped number), and
* offers consistent helpers for the file types the tasks use — ``.npz``,
  ``.npy`` and Matplotlib figures — with one place that owns naming and paths.

What each task saves stays in the task; *where* and *how* it lands lives here::

    store = DataStore("clip_scan")          # -> <repo>/data/clip_scan/
    store.save_npz("clip_A_X_Y_3", X=X, Y=Y, Z=Z)
    store.save_fig("clip_A_X_Y_3")          # current Matplotlib figure -> .png
    d = store.load_npz("clip_A_X_Y_3")

Pass ``unique=True`` for a fresh, non-colliding folder (``name``, ``name-2``,
``name-3``, ...) instead of reusing an existing one.

The runtime servo-position state (``servos_<board>.json``) is deliberately *not*
stored here: it lives under ``paths.state_folder`` because it is live state, not
task output.

Import-safe: importing this module only resolves the data root and imports
numpy/config (both pure). The filesystem is touched on :class:`DataStore`
construction; Matplotlib is imported lazily inside :meth:`DataStore.save_fig`.
"""

import logging
from pathlib import Path

import numpy as np

from config import DATA_FOLDER

# The one data root for all task output: <repo>/data (machine.yaml paths.data_folder).
DATA_ROOT = Path(DATA_FOLDER)


def data_path(*parts) -> Path:
    """A path under the ``data/`` root (does not create anything)."""
    return DATA_ROOT.joinpath(*parts)


def resolve_under_data(path) -> Path:
    """Resolve an input path: absolute as-is, relative against the ``data/`` root."""
    p = Path(path)
    return p if p.is_absolute() else DATA_ROOT / p


def _next_free(root: Path, name: str) -> Path:
    """``root/name`` if free, else the first free ``root/name-2``, ``name-3`` ..."""
    candidate = root / name
    i = 2
    while candidate.exists():
        candidate = root / f"{name}-{i}"
        i += 1
    return candidate


class DataStore:
    """One named run folder under the ``data/`` root.

    Args:
        name: folder name under ``data/`` (e.g. the configured ``output_subdir``).
        unique: if True, never reuse an existing folder — pick the next free
            ``name`` / ``name-2`` / ``name-3`` ... instead.

    Attributes:
        dir: the resolved run-folder :class:`~pathlib.Path`.
        existed: True if the folder already existed (a re-run may overwrite files).
    """

    def __init__(self, name: str, unique: bool = False):
        self.dir = _next_free(DATA_ROOT, name) if unique else (DATA_ROOT / name)
        self.existed = self.dir.exists()
        self.dir.mkdir(parents=True, exist_ok=True)
        if self.existed:
            logging.warning(
                "DataStore: '%s' already exists; new files may overwrite earlier "
                "results (pass unique=True for a fresh folder).", self.dir,
            )
        else:
            logging.info("DataStore: created %s", self.dir)

    # --- paths ---------------------------------------------------------------
    def path(self, filename, suffix: str = None) -> Path:
        """Resolve ``filename`` inside this run folder, appending ``suffix`` if absent."""
        p = self.dir / filename
        if suffix and p.suffix != suffix:
            p = p.with_name(p.name + suffix)
        return p

    def exists(self, filename, suffix: str = None) -> bool:
        """True if ``filename`` (with optional ``suffix``) exists in this folder."""
        return self.path(filename, suffix).exists()

    # --- save ----------------------------------------------------------------
    def save_npz(self, filename, **arrays) -> Path:
        """Save a compressed-key ``.npz`` of named arrays; returns the path."""
        p = self.path(filename, ".npz")
        np.savez(p, **arrays)
        logging.info("saved %s", p)
        return p

    def save_npy(self, filename, obj) -> Path:
        """Save a single object as ``.npy`` (``allow_pickle`` for dict datasets)."""
        p = self.path(filename, ".npy")
        np.save(p, obj)
        logging.info("saved %s", p)
        return p

    def save_fig(self, filename, fig=None, dpi: int = 300, **savefig_kwargs) -> Path:
        """Save a Matplotlib figure (``fig`` or the current one) as ``.png``."""
        import matplotlib.pyplot as plt
        p = self.path(filename, ".png")
        (fig if fig is not None else plt).savefig(p, dpi=dpi, **savefig_kwargs)
        logging.info("saved %s", p)
        return p

    # --- load ----------------------------------------------------------------
    def load_npz(self, filename):
        """Load an ``.npz`` archive from this folder (``allow_pickle=True``)."""
        return np.load(self.path(filename, ".npz"), allow_pickle=True)

    def load_npy(self, filename):
        """Load an ``.npy`` object from this folder (``allow_pickle=True``)."""
        return np.load(self.path(filename, ".npy"), allow_pickle=True)
