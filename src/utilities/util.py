"""Minimal standalone stub of expctl's ``utilities.util``.

The vendored ``sequence.py`` starts with ``from utilities.util import *`` and uses
a handful of coloured print helpers (``printGreen``, ``printError`` ...). In the
expctl deployment the real top-level ``utilities`` package provides them; this
stub lets the ZMQ server (``zmq_server.py`` / ``servo_server.py``) **start when
run standalone from ``src/``**, where the real package isn't on the path.

The helpers route to ``logging`` (not ``print``) so they never corrupt a curses
screen -- they're silent under the interactive console (which disables logging)
and visible when ``zmq_server.py`` is run directly. In production this subpackage
is shadowed by the real top-level ``utilities`` (absolute import), so it only ever
applies to a standalone run.
"""

import logging

_log = logging.getLogger("utilities")


def _emit(*args, **_kwargs):
    if args:
        _log.info(" ".join(str(a) for a in args))


# The colour helpers sequence.py imports via ``*`` -- all no-op-ish log lines.
printGreen = printYellow = printRed = printBlue = printPurple = _emit
printCyan = printWhite = printBold = printComment = printError = _emit
printGray = printGrey = printOrange = _emit
