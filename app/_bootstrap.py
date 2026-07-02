"""Import-path bootstrap for the standalone app scripts.

The entry-point scripts live in ``app/`` but import the library modules that stay
in ``src/`` -- ``config``, ``servodriver``, ``servo_util``, ``callback_functions``,
``optimize``, ``datastore``, ``fit_gaussian``, ``motor_scan`` -- plus the vendored
``scservo_sdk`` and the expctl stubs (``ServerClass``, ``sequence``, ``utilities``)
that also live under ``src/``. All of these are imported by flat name.

Running ``python app/<script>.py`` only puts ``app/`` on ``sys.path`` (so the
sibling app imports like ``tui`` / ``zmq_server`` resolve). Importing this module
first prepends the sibling ``src/`` directory, so the ``src/`` library imports
resolve too. Import it before any project import::

    import _bootstrap  # noqa: F401  -- prepend ../src on sys.path for library imports
    from config import SERVER

It is idempotent (safe to import from several scripts in one process).
"""

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent / "src"
if _SRC.is_dir():
    _src = str(_SRC)
    if _src not in sys.path:
        sys.path.insert(0, _src)
