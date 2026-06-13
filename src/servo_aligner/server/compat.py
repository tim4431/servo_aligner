"""Unpickling compatibility for expctl Sequence payloads.

The expctl client pickles ``Sequence`` objects whose classes were defined in
a module the server may not have under the same name. Before unpickling, the
configured aliases (default: ``sequence``) are mapped onto the vendored copy
so the lookup resolves. This module has no zmq dependency so it is testable
without the ``server`` extra.
"""

from __future__ import annotations

import importlib
import sys
from typing import Sequence as Seq

VENDORED_MODULE = "servo_aligner.server._vendor.sequence"


def install_sequence_aliases(aliases: Seq[str] = ("sequence",)) -> None:
    """Make pickled module references resolve to the vendored sequence module.

    Raises a clear error when expctl's ``utilities`` package (a dependency of
    the vendored module) is not importable — the server extra only works
    inside an expctl environment.
    """
    missing = [a for a in aliases if a not in sys.modules]
    if not missing:
        return
    try:
        module = importlib.import_module(VENDORED_MODULE)
    except ImportError as e:
        raise RuntimeError(
            "cannot unpickle expctl Sequence payloads: the vendored sequence "
            "module needs expctl's 'utilities' package importable (run the "
            f"server inside an expctl environment). Underlying error: {e}"
        ) from e
    for alias in missing:
        sys.modules[alias] = module
