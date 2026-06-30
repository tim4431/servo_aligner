"""Interactive servo console for the servo aligner.

This is the *app* on top of the clean hardware library ``servodriver.py`` and the
shared curses toolkit ``tui.py``. Run it with no arguments for an interactive
control panel:

  * a live **servo monitor** (position / angle / load / torque you can edit),
  * the manual servo controls that used to live in the ZMQ server's CLI
    (set-zero, home, de-hysteresis on/off),
  * a switch to turn the **ZMQ server** (``zmq_server.py``) on and off -- it runs
    in a background thread sharing this console's single serial connection.

In production (installed under expctl): ``python -m expctl.servers.servoaligner.servo_server``.
"""

import collections
import logging
import sys
import threading

from config import SERVER, SERVO_CHANNEL_LIST, sts3032_dict
from servodriver import Servoset
from tui import (
    curses, tui_available, _init_theme, _safe_addstr, _attr_normal, _attr_select,
    _status, _draw_status, _flash_status, _NumberEditor, _quiet,
    _it_action, _it_run, _tui_page, _tui_confirm, _tui_message,
)

# Ring buffer of the ZMQ server's log/print lines, shown on the ZMQ page. While
# the control panel is up, stdout/stderr and logging are redirected here so the
# server's output never lands on the curses screen.
_LOG_BUF = collections.deque(maxlen=1000)


class _BufStream:
    """File-like sink that splits writes into lines and appends them to a deque."""

    def __init__(self, buf):
        self.buf = buf
        self._partial = ""

    def write(self, s):
        self._partial += s
        while "\n" in self._partial:
            line, self._partial = self._partial.split("\n", 1)
            line = line.rstrip()
            if line:
                self.buf.append(line)
        return len(s)

    def flush(self):
        pass

    def isatty(self):
        return False


def _servo_name(servos, i):
    return sts3032_dict[servos.servo_channel_list[i]][1]


def servo_monitor_tui(servos, stdscr=None):
    """Live curses table of every servo's position / angle / load / torque.

    The table refreshes on a timer (~0.6 s) so the values stay live. Two-axis
    selection: up/down (or a digit) pick the servo row, left/right pick which
    field in that row to change (angle or torque -- the active field is
    highlighted). Enter edits the selected field: angle -> type / nudge a number,
    torque -> cycle off/on; Enter confirms (and moves, for angle), Esc cancels.
    't' is a quick torque toggle, 'q' quits.

    Pass ``stdscr`` to run inside an existing curses session (the control panel);
    otherwise it opens its own ``curses.wrapper``.
    """
    n = len(servos.servo_list)
    data = {"pos": [0] * n, "ang": [0.0] * n, "load": [0] * n, "torque": [False] * n}
    # Query the real torque state up front (refresh() no longer force-enables it).
    try:
        data["torque"] = list(servos.get_torque())
    except Exception:
        pass
    EDITABLE_FIELD = ["angle", "torque"]   # the editable fields a row's cursor moves between

    def refresh():
        try:
            with _quiet():
                pos = servos.get_position()
                data["pos"] = list(pos)
                data["ang"] = list(servos.position_to_angle(pos))
                data["load"] = list(servos.multi_load_list)
                # keep the torque column honest if it is toggled out of band
                data["torque"] = list(servos.get_torque())
        except Exception as e:
            _status(f"read error: {e}")

    def field_text(i, field, editing):
        if editing is not None:
            return f"[ {editing} ]"
        if field == "angle":
            return f"{float(data['ang'][i]):.2f}"
        return "on" if data["torque"][i] else "off"

    def draw(stdscr, sel, col, edit_text=None):
        stdscr.erase()
        h, _w = stdscr.getmaxyx()
        _safe_addstr(stdscr, 0, 1, "Servo monitor", _attr_normal() | curses.A_BOLD)
        _safe_addstr(stdscr, 1, 1, f"board {servos.board_id}    {n} servo(s)",
                     _attr_normal() | curses.A_DIM)
        header = (f"{'idx':>3}  {'name':<8} {'id':>3}  {'position':>9}  "
                  f"{'angle(deg)':>13}  {'load':>6}  {'torque':>9}")
        _safe_addstr(stdscr, 3, 3, header, _attr_normal() | curses.A_BOLD)
        for i in range(n):
            # ">" marks the selected servo row
            _safe_addstr(stdscr, 4 + i, 1, ">" if i == sel else " ", _attr_normal() | curses.A_BOLD)
            ang = field_text(i, "angle", edit_text if (i == sel and EDITABLE_FIELD[col] == "angle") else None)
            tq = field_text(i, "torque", edit_text if (i == sel and EDITABLE_FIELD[col] == "torque") else None)
            segs = [
                (f"{i:>3}  {_servo_name(servos, i):<8} {servos.servo_list[i].SCS_ID:>3}  "
                 f"{int(data['pos'][i]):>9}  ", None),
                (f"{ang:>13}", "angle"),
                (f"  {int(data['load'][i]):>6}  ", None),
                (f"{tq:>9}", "torque"),
            ]
            x = 3
            for text, field in segs:
                active = i == sel and field is not None and field == EDITABLE_FIELD[col]
                _safe_addstr(stdscr, 4 + i, x, text, _attr_select() if active else _attr_normal())
                x += len(text)
        foot = ("left/right or type to change - Enter confirm - Esc cancel" if edit_text is not None
                else "up/down servo - left/right field - Enter edit - q quit")
        _safe_addstr(stdscr, h - 2, 1, foot, _attr_normal() | curses.A_DIM)
        _draw_status(stdscr)
        stdscr.refresh()

    def set_torque(i, on):
        try:
            with _quiet():
                servos.set_torque(i, on)
            data["torque"][i] = on
            _status(f"servo {i}: torque {'on' if on else 'off'}")
        except Exception as e:
            _status(f"servo {i}: torque change failed ({e})")

    def move_to(i, deg):
        _status(f"moving servo {i} to {deg} deg ...")
        try:
            with _quiet():
                servos.set_single(i, deg)
            _status(f"servo {i} -> {deg} deg")
        except Exception as e:
            _status(f"servo {i}: move failed ({e})")

    def edit_field(stdscr, sel, col):
        field = EDITABLE_FIELD[col]
        if field == "angle":
            ed = _NumberEditor(round(float(data["ang"][sel]), 2), step=1.0, cast=float,
                               fmt=lambda v: f"{v:.2f}")
        else:  # torque: cycle off/on
            ed = _NumberEditor(bool(data["torque"][sel]), options=[False, True],
                               fmt=lambda v: "on" if v else "off")
        stdscr.timeout(-1)   # block (no refresh ticks) while editing one field
        try:
            while True:
                draw(stdscr, sel, col, edit_text=ed.display())
                res = ed.handle(stdscr.getch())
                if res == "cancel":
                    _status("edit cancelled")
                    return
                if res == "commit":
                    if field == "angle":
                        draw(stdscr, sel, col)
                        move_to(sel, ed.value)
                    else:
                        set_torque(sel, bool(ed.value))
                    return
        finally:
            stdscr.timeout(600)

    def app(stdscr):
        curses.curs_set(0)
        _init_theme(stdscr)
        stdscr.timeout(600)   # getch returns -1 after 0.6 s so the table refreshes
        sel, col = 0, 0
        while True:
            refresh()
            draw(stdscr, sel, col)
            c = stdscr.getch()
            if c == -1:
                continue   # timer tick: just refresh the table
            if c in (curses.KEY_UP, ord("k")):
                sel = (sel - 1) % n
            elif c in (curses.KEY_DOWN, ord("j")):
                sel = (sel + 1) % n
            elif c == curses.KEY_LEFT:
                col = max(0, col - 1)            # clamp -- don't wrap to the rightmost
            elif c == curses.KEY_RIGHT:
                col = min(len(EDITABLE_FIELD) - 1, col + 1)  # clamp -- don't wrap to the leftmost
            elif ord("0") <= c <= ord("9") and (c - ord("0")) < n:
                sel = c - ord("0")
            elif c in (ord("q"), 27):
                break
            elif c in (curses.KEY_ENTER, 10, 13):
                edit_field(stdscr, sel, col)
            elif c in (ord("t"), ord("T")):   # quick torque toggle shortcut
                set_torque(sel, not data["torque"][sel])

    if stdscr is not None:        # run inside the control panel's curses session
        try:
            app(stdscr)
        finally:
            stdscr.timeout(-1)    # restore blocking getch for the caller's menu
        return

    # Standalone: own session. Silence library logging (it writes to stderr).
    prev = logging.root.manager.disable
    logging.disable(logging.CRITICAL)
    try:
        curses.wrapper(app)
    finally:
        logging.disable(prev)


class ZmqController:
    """Turns the ZMQ server on/off in a background thread that shares the
    console's ``Servoset`` (one serial connection, no second owner).

    ``Server.main_loop(cond_fn)`` polls every 10 ms and re-checks ``cond_fn``, so
    setting the stop event makes the thread exit cleanly. ``zmq_server`` is
    imported lazily because it pulls in the expctl framework
    (``sequence`` -> ``utilities.util``), which only exists in the deployment.
    """

    def __init__(self, servos):
        self.servos = servos
        self.thread = None
        self.stop_event = None

    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self):
        if self.running():
            return
        from zmq_server import STSServer   # lazy: needs the expctl framework
        # ServerClass installs a coloredlogs handler on its own logger at import;
        # drop it and let records propagate to root (which the panel captures into
        # _LOG_BUF), so server logs reach the log page, not the screen.
        sc_logger = logging.getLogger("ServerClass")
        sc_logger.handlers = []
        sc_logger.propagate = True
        server = STSServer(SERVER["name"], SERVER["port"], "", self.servos, SERVO_CHANNEL_LIST)
        self.stop_event = threading.Event()
        ev = self.stop_event
        self.thread = threading.Thread(
            target=lambda: server.main_loop(cond_fn=lambda: not ev.is_set()),
            daemon=True,
        )
        self.thread.start()

    def stop(self):
        if self.running():
            self.stop_event.set()
            self.thread.join(timeout=5)
        self.thread = None
        self.stop_event = None


def zmq_page(stdscr, zmq):
    """The ZMQ server's own page: start/stop it and watch its log at the bottom.

    The server runs in a background thread, so leaving this page (Back / q) keeps
    it running -- come back any time and it's still here, with its log. Refreshes
    on a timer so the log stays live.
    """
    stdscr.timeout(500)
    sel = 0
    try:
        while True:
            on = zmq.running()
            actions = ["Stop server" if on else "Start server", "Back (leave server running)"]
            stdscr.erase()
            h, w = stdscr.getmaxyx()
            _safe_addstr(stdscr, 0, 1, "ZMQ server", _attr_normal() | curses.A_BOLD)
            status = f"RUNNING on port {SERVER['port']}" if on else "stopped"
            _safe_addstr(stdscr, 1, 1, f"status: {status}", _attr_normal() | curses.A_DIM)
            for i, a in enumerate(actions):
                _safe_addstr(stdscr, 3 + i, 1, ">" if i == sel else " ", _attr_normal() | curses.A_BOLD)
                _safe_addstr(stdscr, 3 + i, 3, a, _attr_select() if i == sel else _attr_normal())
            _safe_addstr(stdscr, 6, 1, "--- server log ---", _attr_normal() | curses.A_BOLD)
            top = 7
            avail = max(0, (h - 2) - top)
            for j, line in enumerate(list(_LOG_BUF)[-avail:] if avail else []):
                _safe_addstr(stdscr, top + j, 1, line, _attr_normal())
            _safe_addstr(stdscr, h - 2, 1, "up/down - Enter select - q back (server keeps running)",
                         _attr_normal() | curses.A_DIM)
            _draw_status(stdscr)
            stdscr.refresh()
            c = stdscr.getch()
            if c == -1:
                continue   # timer tick: refresh the log
            if c in (curses.KEY_UP, ord("k")):
                sel = (sel - 1) % len(actions)
            elif c in (curses.KEY_DOWN, ord("j")):
                sel = (sel + 1) % len(actions)
            elif c in (ord("q"), 27):
                break
            elif c in (curses.KEY_ENTER, 10, 13):
                if sel == 1:
                    break
                if on:
                    _flash_status(stdscr, "stopping ZMQ server ...")
                    zmq.stop()
                    _status("ZMQ server stopped")
                else:
                    try:
                        zmq.start()
                        _status(f"ZMQ server listening on port {SERVER['port']}")
                    except Exception as e:
                        _status(f"ZMQ server failed to start: {e}")
    finally:
        stdscr.timeout(-1)


def objectives_page(stdscr, state):
    """List the objective (callback) functions with their live values; pick one.

    The objectives are ``callback_functions.OBJECTIVES`` -- the scalar signals the
    optimizers maximize (e.g. photodiode intensity over the ADC). Each value is
    read through ``read_objective``, which returns NaN if the hardware is absent
    or a read fails, so a row shows "NaN" instead of crashing the panel. The
    active objective (``state["objective"]``) is marked with "*"; Enter on a row
    makes it active. Refreshes on a timer so the values stay live.

    ``callback_functions`` is imported lazily (it pulls in the I2C ADC libs), so
    the rest of the console still works if that import is unavailable.
    """
    try:
        from callback_functions import OBJECTIVES, read_objective
    except Exception as e:
        _tui_message(stdscr, [f"objective functions unavailable: {e}"])
        return
    names = list(OBJECTIVES)
    if not names:
        _tui_message(stdscr, ["no objective functions are registered"])
        return
    sel = names.index(state["objective"]) if state.get("objective") in names else 0
    stdscr.timeout(700)   # getch returns -1 after 0.7 s so the values refresh
    try:
        while True:
            stdscr.erase()
            h, _w = stdscr.getmaxyx()
            _safe_addstr(stdscr, 0, 1, "Objective (callback) functions",
                         _attr_normal() | curses.A_BOLD)
            _safe_addstr(stdscr, 1, 1, "live sensor reads -- '*' marks the active objective",
                         _attr_normal() | curses.A_DIM)
            _safe_addstr(stdscr, 3, 3, f"{'name':<22}  {'value':>14}",
                         _attr_normal() | curses.A_BOLD)
            for i, name in enumerate(names):
                val = read_objective(OBJECTIVES[name])
                shown = "NaN" if val != val else f"{val:.4f}"   # val != val is True only for NaN
                active = "*" if name == state.get("objective") else " "
                _safe_addstr(stdscr, 4 + i, 1, ">" if i == sel else " ",
                             _attr_normal() | curses.A_BOLD)
                _safe_addstr(stdscr, 4 + i, 3, f"{active} {name:<22}  {shown:>14}",
                             _attr_select() if i == sel else _attr_normal())
            _safe_addstr(stdscr, h - 2, 1, "up/down move - Enter toggle active - q back",
                         _attr_normal() | curses.A_DIM)
            _draw_status(stdscr)
            stdscr.refresh()
            c = stdscr.getch()
            if c == -1:
                continue   # timer tick: just re-read the values
            if c in (curses.KEY_UP, ord("k")):
                sel = (sel - 1) % len(names)
            elif c in (curses.KEY_DOWN, ord("j")):
                sel = (sel + 1) % len(names)
            elif c in (ord("q"), 27):
                break
            elif c in (curses.KEY_ENTER, 10, 13):
                # Enter on the active row de-selects it (toggle), so you can clear
                # the choice -- handy when none of the objectives read a value.
                if state.get("objective") == names[sel]:
                    state["objective"] = None
                    _status(f"objective deselected ({names[sel]})")
                else:
                    state["objective"] = names[sel]
                    _status(f"active objective: {names[sel]}")
    finally:
        stdscr.timeout(-1)


def control_panel(servos):
    """Interactive control panel: the ZMQ server page (run it in the background +
    watch its log), the live servo monitor, and the manual controls. The server
    and the console share one ``Servoset``, whose bus lock serialises access, so
    the monitor / manual controls work even while the server is running."""
    zmq = ZmqController(servos)
    state = {"objective": None}   # active objective function, selected on its page

    def panel(stdscr):
        curses.curs_set(0)
        _init_theme(stdscr)

        # Actions run in place (via _it_run) so the cursor keeps its position;
        # only "Quit" / q exits the page.
        def open_zmq():
            zmq_page(stdscr, zmq)
            _init_theme(stdscr)

        def open_objectives():
            objectives_page(stdscr, state)
            _init_theme(stdscr)

        # The manual controls and the monitor work even while the server is
        # running -- Servoset serialises bus access with a lock, so the console
        # and the server thread share the one serial port safely.
        def run_monitor():
            servo_monitor_tui(servos, stdscr)
            _init_theme(stdscr)

        def do_home():
            _flash_status(stdscr, "homing all servos ...")
            try:
                with _quiet():
                    servos.home()
                _status("homed all servos to 0 deg")
            except Exception as e:
                _status(f"home failed: {e}")

        def do_zero():
            if not _tui_confirm(stdscr, "Set the CURRENT pose as zero for ALL servos?", double=True):
                _status("set-zero cancelled")
                return
            _flash_status(stdscr, "setting zero ...")
            try:
                with _quiet():
                    servos.set_zero()
                _status("set current pose as zero for all servos")
            except Exception as e:
                _status(f"set-zero failed: {e}")

        def do_dehys():
            servos.de_hysterisis = not servos.de_hysterisis
            _status(f"de-hysteresis turned {'on' if servos.de_hysterisis else 'off'}")

        def build():
            zmq_state = f"RUNNING (port {SERVER['port']})" if zmq.running() else "stopped"
            return [
                _it_run(f"ZMQ server: {zmq_state}   [open]", open_zmq),
                _it_run("Servo monitor  (live table)", run_monitor),
                _it_run(f"Objective: {state['objective'] or '-'}   [select]", open_objectives),
                _it_run("Home all servos (0 deg)", do_home),
                _it_run("Set current pose as zero (all servos)", do_zero),
                _it_run(f"De-hysteresis: {'on' if servos.de_hysterisis else 'off'}   [toggle]", do_dehys),
                _it_action("Quit", "quit"),
            ]

        _tui_page(
            stdscr, "Servo server", build,
            subtitle=f"board {SERVER['board_id']}  {len(servos.servo_list)} servo(s)  zmq port {SERVER['port']}",
            footer="up/down move - Enter select - q quit")
        zmq.stop()   # don't leave the server thread running after the panel exits

    # Capture the server's logs/prints into _LOG_BUF (shown on the ZMQ page) and
    # keep them off the curses screen: redirect stdout/stderr to the buffer and
    # route root logging there. Handlers created later (the lazy zmq_server
    # import) bind to the redirected stderr, so they're captured too.
    old_out, old_err = sys.stdout, sys.stderr
    root = logging.getLogger()
    saved_handlers, saved_level, saved_disable = root.handlers[:], root.level, root.manager.disable
    sink = _BufStream(_LOG_BUF)
    handler = logging.StreamHandler(sink)
    handler.setFormatter(logging.Formatter("%(levelname)s %(name)s: %(message)s"))
    sys.stdout = sys.stderr = sink
    root.handlers = [handler]
    root.setLevel(logging.INFO)
    logging.disable(logging.NOTSET)
    try:
        curses.wrapper(panel)
    finally:
        sys.stdout, sys.stderr = old_out, old_err
        root.handlers, root.level = saved_handlers, saved_level
        logging.disable(saved_disable)


if __name__ == "__main__":
    servos = Servoset(SERVER["board_id"], SERVO_CHANNEL_LIST)
    try:
        if tui_available():
            control_panel(servos)
        else:
            # Non-interactive (piped / no terminal): print a one-shot snapshot.
            pos = servos.get_position()
            ang = servos.position_to_angle(pos)
            for i, servo in enumerate(servos.servo_list):
                print(f"[{i}] {_servo_name(servos, i):<8} id={servo.SCS_ID:>3}  "
                      f"pos={int(pos[i]):>6}  angle={float(ang[i]):>8.2f} deg  load={int(servos.multi_load_list[i])}")
    finally:
        servos.close()
