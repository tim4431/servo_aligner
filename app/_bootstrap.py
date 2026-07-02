"""Import-path bootstrap + environment self-activation for the app scripts.

Two jobs, in order:

1. **Self-activate the project venv.** If a project-local ``.venv`` exists (built
   by ``app/init_helper.py`` with uv) and this process isn't already running
   inside it, re-exec the same command with the venv's interpreter. That makes
   ``python app/<script>.py`` "just work" from any interpreter -- system Python,
   conda base, etc. -- without a manual ``activate``. It is deliberately skipped
   when no ``.venv`` exists yet (so init_helper can bootstrap one) and guarded
   against re-exec loops. Set ``SERVO_ALIGNER_NO_VENV=1`` to disable it.

2. **Prepend ``../src`` on ``sys.path``**, so the flat library imports
   (``from config import ...``) that live in ``src/`` -- ``config``,
   ``servodriver``, ``servo_util``, ``callback_functions``, ``optimize``,
   ``datastore``, ``fit_gaussian``, ``motor_scan``, plus the vendored
   ``scservo_sdk`` and the expctl stubs (``ServerClass``, ``sequence``) -- resolve
   for scripts in ``app/``. Idempotent; safe to import from several scripts in
   one process.

Import it before any project import::

    import _bootstrap  # noqa: F401  -- self-activate the venv + prepend ../src
    from config import SERVER
"""

import os
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_VENV_DIR = _REPO_ROOT / ".venv"
_VENV_PY = _VENV_DIR / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
_REEXEC_GUARD = "SERVO_ALIGNER_VENV_REEXEC"


def _should_reexec():
    """True when we should re-exec into the project venv (see module docstring)."""
    if os.environ.get("SERVO_ALIGNER_NO_VENV"):      # explicit opt-out
        return False
    if os.environ.get(_REEXEC_GUARD):                # already re-execed once
        return False
    if not _VENV_PY.exists():                        # no venv yet -> run as-is
        return False
    try:
        if Path(sys.prefix).resolve() == _VENV_DIR.resolve():    # already inside it
            return False
        if Path(sys.executable).resolve() == _VENV_PY.resolve():
            return False
    except OSError:
        return False
    # Only self-activate for `python <file>.py` launches of one of *our* scripts,
    # never for a REPL, `python -c`, or an unrelated script that imports this shim.
    argv0 = sys.argv[0] if sys.argv else ""
    if not argv0.endswith(".py"):
        return False
    try:
        script = Path(argv0).resolve()
    except OSError:
        return False
    return script.is_file() and _REPO_ROOT in script.parents


if _should_reexec():
    os.environ[_REEXEC_GUARD] = "1"
    try:
        os.execv(str(_VENV_PY), [str(_VENV_PY), *sys.argv])
    except OSError:
        pass   # couldn't re-exec (e.g. permissions) -- fall through and run here

# --- prepend ../src so the flat library imports resolve -----------------------
_SRC = _REPO_ROOT / "src"
if _SRC.is_dir():
    _src = str(_SRC)
    if _src not in sys.path:
        sys.path.insert(0, _src)
