"""A tiny curses TUI toolkit shared by the interactive tools (init_helper,
servo_server).

Everything is built from one small "item language" so every screen looks and
behaves the same:

* ``_it_action(text, value)`` -- Enter returns ``value`` to the caller (navigation)
* ``_it_run(text, fn)``       -- Enter runs ``fn()`` in place (edit a field, jog, ...)
* ``_it_toggle(text, fn)``    -- left/right (or Enter) runs ``fn(delta)``, delta -1/+1

``_tui_page(stdscr, title, build, ...)`` renders such a list and drives it.
A white-on-black theme, a persistent bottom status line and small input/message
helpers round it out. An optional scrollable **log pane** (``_log_enable`` with a
``deque`` of lines) is docked at the bottom of every page: a page calls
``_content_dims`` to size its content above the pane, ``_draw_chrome`` to paint
the footer + status + log, and ``_log_handle_key`` to give the log first crack at
each key. ``l`` moves keyboard focus into the pane (up/down, PageUp/Down,
Home/End scroll). This module is pure (no hardware, no project imports) and
degrades gracefully when curses is unavailable (``curses is None``).
"""

import contextlib
import io
import sys

try:
    import curses
except Exception:  # pragma: no cover - curses is absent on some platforms (e.g. Windows)
    curses = None


def tui_available():
    """True when curses is importable and we have a real interactive terminal."""
    return curses is not None and sys.stdin.isatty() and sys.stdout.isatty()


@contextlib.contextmanager
def _quiet():
    """Swallow stdout (helpers that print) so it can't corrupt the curses screen."""
    with contextlib.redirect_stdout(io.StringIO()):
        yield


def _safe_addstr(win, y, x, text, attr=0):
    h, w = win.getmaxyx()
    if 0 <= y < h and 0 <= x < w:
        try:
            win.addstr(y, x, str(text)[: max(0, w - x - 1)], attr)
        except curses.error:
            pass


# --- Theme: white background, black-on-white text, black highlight bar -------
def _init_theme(stdscr):
    if curses.has_colors():
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)  # normal
        curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLACK)  # selected
        stdscr.bkgd(" ", curses.color_pair(1))


def _attr_normal():
    return curses.color_pair(1) if curses.has_colors() else curses.A_NORMAL


def _attr_select():
    return curses.color_pair(2) if curses.has_colors() else curses.A_REVERSE


# --- Status line: shows the last action's feedback, just above the log pane ---
# (or on the bottom row when no log pane is shown -- see _status_row / _layout).
_STATUS = {"text": ""}


def _status(msg):
    _STATUS["text"] = str(msg)


def _draw_status(stdscr):
    y = _status_row(stdscr)
    try:
        stdscr.move(y, 0)
        stdscr.clrtoeol()
    except curses.error:
        pass
    if _STATUS["text"]:
        _safe_addstr(stdscr, y, 1, _STATUS["text"], _attr_normal() | curses.A_BOLD)


def _flash_status(stdscr, msg):
    """Set the status line and paint it now (for feedback before a blocking call)."""
    _status(msg)
    _draw_status(stdscr)
    stdscr.refresh()


# --- Log pane: one scrollable log window docked at the bottom of every page ----
# A run pins a live log to the screen by handing _log_enable a deque of lines.
# Then EVERY page draws it the same way: _content_dims(stdscr) gives the rows left
# for the page's own content; the page draws within that, then calls
# _draw_chrome(stdscr, footer) to lay down the footer + status + log, and in its
# input loop lets _log_handle_key(stdscr, c) intercept keys first. The pane fills
# the bottom rows; the footer (description) and status (info) lines sit just above
# it. 'l' moves keyboard focus into the pane so arrows / PageUp/Down / Home/End
# scroll the log. State is module-global, like _STATUS.
_LOG = {
    "buf": None,        # deque of log lines to show, or None -> pane hidden
    "height": 8,        # desired number of log lines in the pane
    "title": "log",     # shown in the pane's header bar
    "focused": False,   # True while keystrokes scroll the log, not the page
    "offset": 0,        # lines scrolled back from the newest (0 = pinned to newest)
}


def _log_enable(buf, height=8, title="log"):
    """Show ``buf`` (a deque of strings) as the scrollable log pane on every page."""
    _LOG.update(buf=buf, height=height, title=title, focused=False, offset=0)


def _log_disable():
    """Hide the log pane and release any keyboard focus it held."""
    _LOG.update(buf=None, focused=False, offset=0)


def _log_active():
    return _LOG["buf"] is not None


def _log_focused():
    return _log_active() and _LOG["focused"]


def _log_region(h):
    """Bottom log pane on a screen ``h`` tall: ``(header_y, n_lines)``, or None when
    off / too short. The pane fills the bottom rows; the footer and status line go
    just above its header. Capped so the page above always keeps some rows."""
    if not _log_active():
        return None
    n = min(_LOG["height"], max(3, (h - 8) // 2))
    header_y = h - n - 1               # header row, then n lines reaching the last row
    if header_y < 5:                   # need rows for content + footer + status above
        return None
    return header_y, n


def _layout(h):
    """Row layout for the shared bottom chrome: a dict with ``rows`` (content rows
    the page may use), ``footer_y``, ``status_y`` and ``log`` (the ``_log_region``
    tuple or None). With a log pane the footer/status sit just above it; otherwise
    on the last two rows as before."""
    region = _log_region(h)
    if region:
        header_y, _n = region
        return {"rows": header_y - 2, "footer_y": header_y - 2,
                "status_y": header_y - 1, "log": region}
    return {"rows": h - 2, "footer_y": h - 2, "status_y": h - 1, "log": None}


def _content_dims(stdscr):
    """(rows, cols) a page may use for its own content, leaving room for the footer,
    status line and bottom log pane. Every page sizes itself against this so the
    log always lands in the same place."""
    h, w = stdscr.getmaxyx()
    return _layout(h)["rows"], w


def _footer_row(stdscr):
    return _layout(stdscr.getmaxyx()[0])["footer_y"]


def _status_row(stdscr):
    return _layout(stdscr.getmaxyx()[0])["status_y"]


def _log_scroll(delta, page):
    """Move the view by ``delta`` lines (+ = older / up, - = newer / down); ``page``
    is the number of visible lines (so PageUp/Down move by a screenful)."""
    if not _log_active():
        return
    max_off = max(0, len(_LOG["buf"]) - page)
    _LOG["offset"] = max(0, min(max_off, _LOG["offset"] + delta))


def _log_handle_key(stdscr, c):
    """First crack at a key for any page. Returns True if the log consumed it (the
    page should then ignore it). 'l' grabs focus; once focused, arrows / PageUp /
    PageDown / Home / End scroll and 'l'/'q'/Esc release focus back to the page."""
    if not _log_active():
        return False
    region = _log_region(stdscr.getmaxyx()[0])
    page = region[1] if region else _LOG["height"]
    if not _log_focused():
        if c == ord("l"):
            _LOG.update(focused=True, offset=0)
            return True
        return False
    if c in (ord("l"), ord("q"), 27):
        _LOG["focused"] = False
    elif c in (curses.KEY_UP, ord("k")):
        _log_scroll(+1, page)
    elif c in (curses.KEY_DOWN, ord("j")):
        _log_scroll(-1, page)
    elif c == curses.KEY_PPAGE:
        _log_scroll(+page, page)
    elif c == curses.KEY_NPAGE:
        _log_scroll(-page, page)
    elif c == curses.KEY_HOME:
        _log_scroll(+len(_LOG["buf"]), page)   # jump to the oldest line
    elif c == curses.KEY_END:
        _log_scroll(-len(_LOG["buf"]), page)   # back to the newest line
    return True


def _clear_eol(stdscr, y, x=0):
    try:
        stdscr.move(y, x)
        stdscr.clrtoeol()
    except curses.error:
        pass


def _log_window(region):
    """The slice of log lines currently visible for ``region`` (newest at bottom)."""
    n = region[1]
    lines = list(_LOG["buf"])
    end = len(lines) - _LOG["offset"]
    start = max(0, end - n)
    return lines[start:end]


def _draw_log(stdscr):
    """Draw the bottom log pane (header + the visible window of lines), if enabled
    and it fits. Over-paints its rows so any content bleed is cleared."""
    h, w = stdscr.getmaxyx()
    region = _log_region(h)
    if region is None:
        return
    header_y, _n = region
    focused = _LOG["focused"]
    hattr = (_attr_select() if focused else _attr_normal()) | curses.A_BOLD
    hint = ("up/down scroll - PgUp/Dn page - l/q back" if focused
            else "l to focus & scroll")
    _clear_eol(stdscr, header_y)
    bar = f"--- {_LOG['title']} ({hint}) ".ljust(max(0, w - 2), "-")
    _safe_addstr(stdscr, header_y, 1, bar, hattr)
    for j, line in enumerate(_log_window(region)):
        _clear_eol(stdscr, header_y + 1 + j)
        _safe_addstr(stdscr, header_y + 1 + j, 1, line, _attr_normal())


def _draw_chrome(stdscr, footer):
    """Lay down the bottom chrome every page shares: the footer (description) line,
    the status (info) line, then the log pane below them. Call after the page has
    drawn its own content."""
    lay = _layout(stdscr.getmaxyx()[0])
    _clear_eol(stdscr, lay["footer_y"])
    _safe_addstr(stdscr, lay["footer_y"], 1, footer, _attr_normal() | curses.A_DIM)
    _draw_status(stdscr)
    _draw_log(stdscr)


def _to_int(s):
    return int(str(s), 0)   # base 0 also accepts 0x.. / 0b..


class _NumberEditor:
    """Tracks the working value while a number field is in edit mode.

    Left/Down nudge it down by ``step``, Right/Up nudge it up (or cycle through
    ``options`` when given); digits/'.'/'-' type a fresh value; Enter commits,
    Esc cancels. ``handle(key)`` returns "commit", "cancel", or None (still
    editing). Reusable by any curses loop, not just _tui_page.
    """

    def __init__(self, value, step=1, cast=_to_int, options=None, lo=None, hi=None, fmt=None):
        self.value = value
        self.buf = ""
        self.step = step
        self.cast = cast
        self.options = options
        self.lo = lo
        self.hi = hi
        self.fmt = fmt or (lambda v: f"{v}")

    def display(self):
        return self.buf if self.buf != "" else self.fmt(self.value)

    def _cur(self):
        if self.buf != "":
            try:
                return self.cast(self.buf)
            except ValueError:
                return self.value
        return self.value

    def _clamp(self, v):
        if self.lo is not None:
            v = max(self.lo, v)
        if self.hi is not None:
            v = min(self.hi, v)
        return v

    def _nudge(self, d):
        if self.options:
            try:
                i = self.options.index(self._cur())
            except ValueError:
                i = 0
            self.value = self.options[(i + d) % len(self.options)]
        else:
            self.value = self._clamp(self._cur() + d * self.step)
        self.buf = ""

    def handle(self, c):
        if c in (curses.KEY_LEFT, curses.KEY_DOWN):
            self._nudge(-1)
        elif c in (curses.KEY_RIGHT, curses.KEY_UP):
            self._nudge(+1)
        elif c in (curses.KEY_ENTER, 10, 13):
            self.value = self._clamp(self._cur())
            return "commit"
        elif c == 27:  # Esc
            return "cancel"
        elif c in (curses.KEY_BACKSPACE, 127, 8):
            self.buf = self.buf[:-1]
        elif 0 <= c < 256 and chr(c) in "0123456789.+-":
            self.buf += chr(c)
        return None


# --- One menu "language" shared by every page --------------------------------
def _it_action(text, value):
    return {"kind": "action", "text": text, "value": value}


def _it_run(text, on_enter):
    return {"kind": "run", "text": text, "on_enter": on_enter}


def _it_toggle(text, on_change):
    return {"kind": "toggle", "text": text, "on_change": on_change}


def _it_edit(prefix, value, on_set, step=1, cast=_to_int, options=None, lo=None, hi=None, fmt=None):
    """An inline-editable number/register field. Navigate to it, press Enter to
    enter edit mode (arrows nudge / cycle ``options``, digits type a value), then
    Enter commits via ``on_set(new_value)`` and Esc cancels. ``prefix`` is the
    label up to the value, e.g. ``"Bus ID    : "``."""
    fmt = fmt or (lambda v: f"{v}")
    return {"kind": "edit", "prefix": prefix, "value": value, "on_set": on_set,
            "text": f"{prefix}{fmt(value)}",
            "opts": {"step": step, "cast": cast, "options": options, "lo": lo, "hi": hi, "fmt": fmt}}


def _render_page(stdscr, title, subtitle, items, sel, footer):
    stdscr.erase()
    h, w = stdscr.getmaxyx()
    _safe_addstr(stdscr, 0, 1, title, _attr_normal() | curses.A_BOLD)
    top = 2
    if subtitle:
        _safe_addstr(stdscr, 1, 1, subtitle, _attr_normal() | curses.A_DIM)
        top = 3
    # Size the menu to the area the log pane leaves free (rows for a bottom pane,
    # cols for a right pane), so the log always lands in the same place.
    rows, cols = _content_dims(stdscr)
    num_w = len(str(len(items) - 1)) if items else 1
    for i, item in enumerate(items):
        y = top + i
        if y >= rows:
            break
        selected = i == sel
        # ">" marks the selected row, plus a highlight bar on the item itself
        _safe_addstr(stdscr, y, 1, ">" if selected else " ", _attr_normal() | curses.A_BOLD)
        attr = _attr_select() if selected else _attr_normal()
        # Prefix every row with its index ("[0] ...") so it can be jumped to by
        # typing the number (see _tui_page); width-pad so multi-digit lists line up.
        label = f"[{i:>{num_w}}] {item['text']}"
        _safe_addstr(stdscr, y, 3, f"{label:<{max(0, cols - 5)}}", attr)
    _draw_chrome(stdscr, footer)   # footer + status, then the log pane below them
    stdscr.refresh()


def _tui_page(stdscr, title, build, subtitle=None,
              footer="up/down move - [#] jump - Enter select - q back",
              edit_footer="left/right or type to change - Enter confirm - Esc cancel"):
    """Render a page and drive it. ``build()`` returns a fresh item list each loop
    so any values shown stay live. Returns the chosen action's value, or None on
    q/ESC. An ``_it_edit`` field enters inline edit mode on Enter; while editing,
    arrows/typing change the working value and Enter/Esc commit/cancel. Each row
    is prefixed with its index ("[0] ..."); typing that number jumps the
    selection to it (digits accumulate for two-digit lists, clearing after a
    brief pause)."""
    sel = 0
    ed = None   # active _NumberEditor while a field is being edited, else None
    jump = ""   # line-number digits typed so far; self-clears on a getch timeout
    try:
        while True:
            items = build()
            if not items:
                return None
            sel = max(0, min(sel, len(items) - 1))
            t = title() if callable(title) else title
            sub = subtitle() if callable(subtitle) else subtitle
            if ed is not None:
                shown = list(items)
                row = dict(items[sel])
                row["text"] = f"{row['prefix']}[ {ed.display()} ]"
                shown[sel] = row
                _render_page(stdscr, t, sub, shown, sel, edit_footer)
            else:
                _render_page(stdscr, t, sub, items, sel, footer)
            # Time out (so the loop redraws) while a jump number is pending OR the
            # log pane is shown -- the latter keeps the streamed log live without a
            # keypress. Otherwise block waiting for a key as usual.
            stdscr.timeout(600 if (jump or _log_active()) else -1)
            c = stdscr.getch()
            if c == -1:            # timed out -> clear any pending jump, redraw
                jump = ""
                continue
            item = items[sel]
            # The log pane gets first crack at the key: 'l' focus, 'L' move it,
            # and while focused the scroll keys drive the log, not the menu.
            if ed is None and _log_handle_key(stdscr, c):
                continue
            if ed is not None:
                res = ed.handle(c)
                if res == "commit":
                    item["on_set"](ed.value)
                    ed = None
                elif res == "cancel":
                    ed = None
                continue
            kind = item["kind"]
            # Type a line number to jump straight to that row. Digits accumulate
            # so two-digit indices work; a digit that would overflow starts a
            # fresh number. Any non-digit key below ends the pending jump.
            if 0 <= c < 256 and chr(c) in "0123456789":
                cand = jump + chr(c)
                if int(cand) < len(items):
                    jump = cand
                elif int(chr(c)) < len(items):
                    jump = chr(c)
                else:
                    continue
                sel = int(jump)
                continue
            jump = ""
            if c in (curses.KEY_UP, ord("k")):
                sel = (sel - 1) % len(items)
            elif c in (curses.KEY_DOWN, ord("j")):
                sel = (sel + 1) % len(items)
            elif c in (ord("q"), 27):
                return None
            elif c == curses.KEY_LEFT and kind == "toggle":
                item["on_change"](-1)
            elif c == curses.KEY_RIGHT and kind == "toggle":
                item["on_change"](+1)
            elif c in (curses.KEY_ENTER, 10, 13):
                if kind == "action":
                    return item["value"]
                if kind == "edit":
                    ed = _NumberEditor(item["value"], **item["opts"])
                elif kind == "toggle":
                    item["on_change"](+1)
                else:  # "run"
                    item["on_enter"]()
    finally:
        stdscr.timeout(-1)   # restore blocking getch for other curses readers


def _tui_confirm(stdscr, question, double=False):
    """Yes/No dialog for a destructive action. Defaults to **No** (the cursor
    starts on "No", and q/Esc also mean No), so an accidental Enter is safe. With
    ``double=True`` the user must confirm a second time. Returns True only on full
    confirmation. Reusable by any curses tool."""
    def ask(q):
        return _tui_page(stdscr, q, lambda: [
            _it_action("No  (cancel)", False),
            _it_action("Yes", True),
        ], footer="up/down move - Enter select - q = No") is True

    if not ask(question):
        return False
    if double:
        return ask("Are you sure? This re-defines the servo zero and cannot be undone.")
    return True


def _tui_message(stdscr, lines, wait=True):
    if isinstance(lines, str):
        lines = [lines]
    stdscr.erase()
    for i, ln in enumerate(lines):
        _safe_addstr(stdscr, 1 + i, 2, ln, _attr_normal())
    # footer + status + log pane below the message (the log pane stays visible)
    _draw_chrome(stdscr, "Press any key to continue ..." if wait else "")
    stdscr.refresh()
    if wait:
        stdscr.getch()


def _tui_input(stdscr, prompt, default=""):
    """Read a line at the bottom of the screen; empty reply keeps the default."""
    h, w = stdscr.getmaxyx()
    y = h - 2
    msg = f"{prompt} [{default}]: " if str(default) != "" else f"{prompt}: "
    try:
        stdscr.move(y, 0)
        stdscr.clrtoeol()
    except curses.error:
        pass
    _safe_addstr(stdscr, y, 1, msg, _attr_normal())
    stdscr.refresh()
    curses.echo()
    curses.curs_set(1)
    try:
        raw = stdscr.getstr(y, min(1 + len(msg), w - 2), 64)
        s = raw.decode("utf-8", "ignore").strip() if raw is not None else ""
    except Exception:
        s = ""
    finally:
        curses.noecho()
        curses.curs_set(0)
    return s if s != "" else str(default)


def _tui_input_int(stdscr, prompt, default):
    while True:
        s = _tui_input(stdscr, prompt, default)
        try:
            return int(str(s), 0)   # base 0 also accepts 0x.. / 0b..
        except ValueError:
            _tui_message(stdscr, [f"'{s}' is not an integer -- try again."])


def _tui_input_float(stdscr, prompt, default):
    while True:
        s = _tui_input(stdscr, prompt, default)
        try:
            return float(s)
        except ValueError:
            _tui_message(stdscr, [f"'{s}' is not a number -- try again."])
