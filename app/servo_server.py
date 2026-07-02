"""Interactive servo console for the servo aligner.

This is the *app* on top of the clean hardware library ``servodriver.py`` and the
shared curses toolkit ``tui.py``. Run it with no arguments for an interactive
control panel:

  * a live **servo monitor** (position / angle / load / torque you can edit),
  * the manual servo controls that used to live in the ZMQ server's CLI
    (set-zero, home, de-hysteresis on/off),
  * a switch to turn the **ZMQ server** (``zmq_server.py``) on and off -- it runs
    in a background thread sharing this console's single serial connection.

The manual controls are also usable one-shot, e.g. for setup scripts::

    python app/servo_server.py set_zero
    python app/servo_server.py home
    python app/servo_server.py set_angle 10 -5 0 0 0 0 0 0
    python app/servo_server.py set_single 3 12.5
"""

import collections
import logging
import sys
import threading

import _bootstrap  # noqa: F401 -- prepend ../src on sys.path for the library imports below
from config import SERVER, SERVO_CHANNEL_LIST, sts3032_dict
from servodriver import Servoset
from tui import (
    curses, tui_available, _init_theme, _safe_addstr, _attr_normal, _attr_select,
    _status, _flash_status, _NumberEditor, _quiet,
    _it_action, _it_run, _tui_page, _tui_confirm, _tui_message,
    _log_enable, _log_disable, _log_handle_key, _content_dims, _draw_chrome,
)

# Ring buffer of all log/print output. While the control panel is up, stdout,
# stderr and logging are redirected here so output never lands on the curses
# screen; instead it streams into the shared bottom log pane shown on every page
# (see tui._log_enable / _draw_chrome).
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


def _fmt_objective(val):
    """Format an objective reading for display: ``-`` when unset, ``NaN`` for a
    failed/absent read, else 4 decimals (``val != val`` is True only for NaN)."""
    if val is None:
        return "-"
    return "NaN" if val != val else f"{val:.4f}"


def _current_pose(servos):
    """Current full-length angle vector (degrees), or ``None`` if the read fails.

    Feeds pose-dependent objectives (``dummy_gaussian``) their live pose on pages
    that don't otherwise read the servos. Wrapped in ``_quiet()`` so any helper
    prints can't corrupt the curses screen; returns ``None`` on a bus error so the
    caller falls back to a pose-free read.
    """
    try:
        with _quiet():
            return list(servos.position_to_angle(servos.get_position()))
    except Exception:
        return None


def _read_active_objective(state, para_nr_move=None):
    """Live value of the active objective (``state['objective']``).

    ``para_nr_move`` (the current full-length angle vector) is forwarded to the
    objective, so pose-dependent objectives -- e.g. ``dummy_gaussian`` -- track the
    live pose; pass it where a pose is on hand (the servo monitor), or leave it
    ``None`` (objectives that read a real sensor ignore it, and ``dummy_gaussian``
    then evaluates at the origin).

    Returns ``None`` when no objective is selected, or ``float('nan')`` when
    ``callback_functions`` (or its ADC) is unavailable or the read fails -- so the
    value can be shown on any page without crashing on a machine that lacks the
    hardware. ``callback_functions`` is imported lazily because it pulls in the
    I2C ADC libraries.
    """
    name = state.get("objective") if state else None
    if not name:
        return None
    try:
        from callback_functions import OBJECTIVES, read_objective
        return read_objective(OBJECTIVES[name], para_nr_move=para_nr_move)
    except Exception:
        return float("nan")


def servo_monitor_tui(servos, stdscr=None, state=None):
    """Live curses table of every servo's position / angle / load / torque.

    The table refreshes on a timer (~0.6 s) so the values stay live. Two-axis
    selection: up/down (or a digit) pick the servo row, left/right pick which
    field in that row to change (angle or torque -- the active field is
    highlighted). Enter edits the selected field: angle -> type / nudge a number,
    torque -> cycle off/on; Enter confirms (and moves, for angle), Esc cancels.
    't' is a quick torque toggle, 'q' quits.

    Pass ``stdscr`` to run inside an existing curses session (the control panel);
    otherwise it opens its own ``curses.wrapper``. ``state`` is the control panel's
    shared state dict -- when given, the chosen objective (callback) function and
    its live value are shown above the table.
    """
    n = len(servos.servo_list)
    data = {"pos": [0] * n, "ang": [0.0] * n, "load": [0] * n, "torque": [False] * n}
    obj = {"name": None, "val": None}   # active objective + its last live reading
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
        # Objective read is independent of the servo serial bus (I2C ADC) and is
        # NaN-safe, so keep it outside the try above; None when no state/objective.
        # Pass the current pose so a pose-dependent objective (dummy_gaussian)
        # tracks the servos live as they are jogged here.
        obj["name"] = state.get("objective") if state else None
        obj["val"] = _read_active_objective(state, para_nr_move=data["ang"])

    def field_text(i, field, editing):
        if editing is not None:
            return f"[ {editing} ]"
        if field == "angle":
            return f"{float(data['ang'][i]):.2f}"
        return "on" if data["torque"][i] else "off"

    def draw(stdscr, sel, col, edit_text=None):
        stdscr.erase()
        h, _w = stdscr.getmaxyx()
        rows, _cols = _content_dims(stdscr)   # leave room for the shared log pane
        _safe_addstr(stdscr, 0, 1, "Servo monitor", _attr_normal() | curses.A_BOLD)
        _safe_addstr(stdscr, 1, 1, f"board {servos.board_id}    {n} servo(s)",
                     _attr_normal() | curses.A_DIM)
        if state is not None:
            obj_line = (f"objective: {obj['name']} = {_fmt_objective(obj['val'])}"
                        if obj["name"] else "objective: -  (none selected)")
            _safe_addstr(stdscr, 2, 1, obj_line, _attr_normal() | curses.A_DIM)
        header = (f"{'idx':>3}  {'name':<8} {'id':>3}  {'position':>9}  "
                  f"{'angle(deg)':>13}  {'load':>6}  {'torque':>9}")
        _safe_addstr(stdscr, 3, 3, header, _attr_normal() | curses.A_BOLD)
        for i in range(n):
            if 4 + i >= rows:           # stop above a bottom log pane
                break
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
                else "up/down servo - left/right field - Enter edit - l log - q quit")
        _draw_chrome(stdscr, foot)   # footer + status, then the log pane below them
        stdscr.refresh()

    def set_torque(i, on):
        try:
            pos_mask = [1 if j == i else 0 for j in range(n)]
            with _quiet():
                servos.set_torque(on, pos_mask)
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
            if _log_handle_key(stdscr, c):
                continue   # log pane took the key (focus / move / scroll)
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
    imported lazily so the rest of the console still works if the vendored
    expctl stub (``ServerClass``) or ``pyzmq`` is unavailable.
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
        # _LOG_BUF), so server logs reach the log pane, not the screen.
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
    """The ZMQ server's own page: start/stop it. Its log output streams into the
    shared bottom log pane (same as every page) -- press 'l' to focus and scroll it.

    The server runs in a background thread, so leaving this page (Back / q) keeps
    it running. Refreshes on a timer so the live values stay current.
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
            _draw_chrome(stdscr, "up/down - Enter select - l log - q back (server keeps running)")
            stdscr.refresh()
            c = stdscr.getch()
            if c == -1:
                continue   # timer tick: refresh live values
            if _log_handle_key(stdscr, c):
                continue   # log pane took the key (focus / move / scroll)
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


def objectives_page(stdscr, state, servos):
    """List the objective (callback) functions with their live values; pick one.

    The objectives are ``callback_functions.OBJECTIVES`` -- the scalar signals the
    optimizers maximize (e.g. photodiode intensity over the ADC). Each value is
    read through ``read_objective``, which returns NaN if the hardware is absent
    or a read fails, so a row shows "NaN" instead of crashing the panel. Each read
    is passed the current servo pose (read once per refresh), so a pose-dependent
    objective (``dummy_gaussian``) shows its value at the live pose. The active
    objective (``state["objective"]``) is marked with "*"; Enter on a row makes it
    active. Refreshes on a timer so the values stay live.

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
            rows, _cols = _content_dims(stdscr)   # leave room for the shared log pane
            _safe_addstr(stdscr, 0, 1, "Objective (callback) functions",
                         _attr_normal() | curses.A_BOLD)
            _safe_addstr(stdscr, 1, 1, "live sensor reads -- '*' marks the active objective",
                         _attr_normal() | curses.A_DIM)
            _safe_addstr(stdscr, 3, 3, f"{'name':<22}  {'value':>14}",
                         _attr_normal() | curses.A_BOLD)
            pose = _current_pose(servos)   # one bus read per refresh, shared by every row
            for i, name in enumerate(names):
                if 4 + i >= rows:           # stop above a bottom log pane
                    break
                val = read_objective(OBJECTIVES[name], para_nr_move=pose)
                shown = "NaN" if val != val else f"{val:.4f}"   # val != val is True only for NaN
                active = "*" if name == state.get("objective") else " "
                _safe_addstr(stdscr, 4 + i, 1, ">" if i == sel else " ",
                             _attr_normal() | curses.A_BOLD)
                _safe_addstr(stdscr, 4 + i, 3, f"{active} {name:<22}  {shown:>14}",
                             _attr_select() if i == sel else _attr_normal())
            _draw_chrome(stdscr, "up/down move - Enter toggle active - l log - q back")
            stdscr.refresh()
            c = stdscr.getch()
            if c == -1:
                continue   # timer tick: just re-read the values
            if _log_handle_key(stdscr, c):
                continue   # log pane took the key (focus / move / scroll)
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
    """Interactive control panel: the ZMQ server page (run it in the background),
    the live servo monitor, and the manual controls. A shared log pane is docked at
    the bottom of every page (press 'l' to focus and scroll it). The server and the
    console share one ``Servoset``, whose bus lock serialises access, so the
    monitor / manual controls work even while the server is running."""
    zmq = ZmqController(servos)
    state = {"objective": None}   # active objective function, selected on its page

    def panel(stdscr):
        curses.curs_set(0)
        _init_theme(stdscr)
        # Pin the captured server/print/log output as a live, scrollable pane at
        # the bottom of the panel; 'l' moves focus into it to scroll back.
        _log_enable(_LOG_BUF, height=8, title="server log")

        # Actions run in place (via _it_run) so the cursor keeps its position;
        # only "Quit" / q exits the page.
        def open_zmq():
            zmq_page(stdscr, zmq)
            _init_theme(stdscr)

        def open_objectives():
            objectives_page(stdscr, state, servos)
            _init_theme(stdscr)

        # The manual controls and the monitor work even while the server is
        # running -- Servoset serialises bus access with a lock, so the console
        # and the server thread share the one serial port safely.
        def run_monitor():
            servo_monitor_tui(servos, stdscr, state)
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
            if state["objective"]:
                val = _read_active_objective(state, para_nr_move=_current_pose(servos))
                obj_label = f"{state['objective']} = {_fmt_objective(val)}"
            else:
                obj_label = "-"
            return [
                _it_run(f"ZMQ server: {zmq_state}   [open]", open_zmq),
                _it_run("Servo monitor  (live table)", run_monitor),
                _it_run(f"Objective: {obj_label}   [select]", open_objectives),
                _it_run("Home all servos (0 deg)", do_home),
                _it_run("Set current pose as zero (all servos)", do_zero),
                _it_run(f"De-hysteresis: {'on' if servos.de_hysterisis else 'off'}   [toggle]", do_dehys),
                _it_action("Quit", "quit"),
            ]

        _tui_page(
            stdscr, "Servo server", build,
            subtitle=f"board {SERVER['board_id']}  {len(servos.servo_list)} servo(s)  zmq port {SERVER['port']}",
            footer="up/down move - Enter select - l log - q quit")
        _log_disable()
        zmq.stop()   # don't leave the server thread running after the panel exits

    # Capture the server's logs/prints into _LOG_BUF (shown in the log pane) and
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


_COMMANDS = ("set_zero", "home", "set_angle", "set_single")


def _run_command(servos, cmd, args):
    """One-shot manual command dispatch (see the module docstring)."""
    if cmd == "set_zero":
        servos.set_zero()
    elif cmd == "home":
        servos.home()
    elif cmd == "set_angle":
        n = len(servos.servo_list)
        if len(args) != n:
            raise SystemExit(f"set_angle needs {n} angles (deg), got {len(args)}")
        servos.set_angle([float(a) for a in args])
    elif cmd == "set_single":
        if len(args) != 2:
            raise SystemExit("usage: set_single <channel index> <angle deg>")
        servos.set_single(int(args[0]), float(args[1]))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] not in _COMMANDS:
        raise SystemExit(f"unknown command {sys.argv[1]!r} (expected one of: {', '.join(_COMMANDS)})")
    if len(sys.argv) > 1:
        logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    servos = Servoset(SERVER["board_id"], SERVO_CHANNEL_LIST)
    try:
        if len(sys.argv) > 1:
            _run_command(servos, sys.argv[1], sys.argv[2:])
        elif tui_available():
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
