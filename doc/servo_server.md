# The Interactive Console — `servo_server.py`

[`app/servo_server.py`](../app/servo_server.py) is the day-to-day cockpit for the
aligner: a curses **control panel** on top of the hardware library
([`servodriver.py`](../src/servodriver.py)) and the shared TUI toolkit
([`app/tui.py`](../app/tui.py)). One process gives you

* a live **servo monitor** (position / angle / load / torque, editable in place),
* an **objective page** showing every registered objective function with its
  live sensor value, and a way to pick the active one,
* the **manual controls** (home, set-zero, de-hysteresis toggle),
* an on/off switch for the **ZMQ server**
  ([`zmq_server.py`](../app/zmq_server.py)), which then runs in a background
  thread *inside this process*, sharing the console's single serial connection.

The manual controls double as **one-shot CLI commands** for setup scripts, so
the same file is also the scriptable entry point for "move the servos by hand".

## 1. Three ways to run it

```bash
python app/servo_server.py                         # interactive control panel
python app/servo_server.py set_zero                # one-shot command (see §5)
python app/servo_server.py | cat                   # no TTY: print a status snapshot
```

On startup it constructs the process's one `Servoset` (which **opens the serial
port and enables torque** — see [motor.md](motor.md)) and always closes it on
exit. With no arguments it opens the control panel if a real terminal is
available (`tui_available()`); without one (piped, cron, no TTY) it degrades to
a one-shot table of every servo's position, angle and load.

## 2. The control panel

The main menu (title "Servo server") shows the live state in its own labels —
ZMQ server running/stopped, the active objective and its current value, the
de-hysteresis state — and every page docks the same scrollable **log pane** at
the bottom (§4). Common keys on every page:

| Key | Action |
|-----|--------|
| up/down (or `j`/`k`) | move the selection |
| Enter | select / edit / run the item |
| `l` | focus the log pane (up/down, PageUp/Down, Home/End scroll; `l`/Esc to leave) |
| `q` / Esc | leave the page (quit, from the main menu) |

### Servo monitor

A live table of every channel — index, name, bus ID, raw encoder position,
angle in degrees, load, torque — refreshed on a ~0.6 s timer. Two-axis
selection: up/down (or a digit `0`–`9`) picks the servo row, left/right picks
the editable field (**angle** or **torque**). Enter edits the selected field:
angle opens a number editor (type a value or nudge with left/right; Enter
confirms *and moves the servo*, Esc cancels), torque cycles off/on. `t` is a
quick torque toggle for the selected servo. If an objective is active it is
shown above the table, evaluated at the live pose — so jogging a knob shows the
objective respond in real time.

### Objective page

Lists `callback_functions.OBJECTIVES` — the scalar feedback signals the
optimizers maximize (see [optimize.md](optimize.md)); `intensity_adc` is the
photodiode over the MCP3424 ADC, `dummy_gaussian` a hardware-free synthetic
landscape. Every row shows a live reading through `read_objective`, which is
**NaN-safe**: a missing ADC or a failed read shows `NaN` instead of crashing the
panel, so the page works on a machine without the sensor. The current servo
pose is read once per refresh and passed to each objective so pose-dependent
ones (`dummy_gaussian`) track the knobs live. Enter marks a row as the active
objective (`*`); Enter on the active row deselects it. The choice is panel
state only — it feeds the live displays, not the standalone optimizer scripts.

### Manual controls

* **Home all servos** — every channel to 0° (encoder 2048).
* **Set current pose as zero** — double-confirmed, since it redefines the
  reference for every angle; the pre-zero pose is appended to
  `set_zero_history_<board>.jsonl` for recovery.
* **De-hysteresis toggle** — runtime on/off of the backlash compensation
  ([motor.md](motor.md)); the default comes from `machine.yaml`.

### ZMQ server page

Starts/stops the [`zmq_server.py`](../app/zmq_server.py) `STSServer` (port from
`machine.yaml`'s `server` section, default 60627) in a **daemon background
thread**. Leaving the page keeps it running — the main menu keeps showing
`RUNNING (port …)` — and quitting the whole panel stops it. Two design points
make this safe:

* The server's `Servoset` is **injected**: console and server share the one
  serial connection, and `Servoset` serialises every bus transaction with an
  `RLock`, so the monitor and manual controls work *while* the server is
  serving without interleaved packets.
* `Server.main_loop(cond_fn)` polls the ZMQ socket every 10 ms and re-checks
  `cond_fn` each pass, so "stop" is just setting a `threading.Event` and
  joining the thread (class `ZmqController`).

`zmq_server` is imported lazily on first start, so the rest of the console
works even when `pyzmq` or the vendored expctl stubs are unavailable.

## 3. Panel structure (for hacking on it)

```
control_panel(servos)                       # owns the shared state + log capture
 ├─ state = {"objective": None}             # active objective, shared by all pages
 ├─ ZmqController(servos)                   # server thread on/off
 └─ _tui_page(...)  main menu, items rebuilt every pass (live labels)
     ├─ servo_monitor_tui(servos, stdscr, state)   # also runnable standalone
     ├─ objectives_page(stdscr, state, servos)
     ├─ zmq_page(stdscr, zmq)
     └─ do_home / do_zero / do_dehys        # thin wrappers with status messages
```

All pages are built from `tui.py`'s item language (`_it_run` / `_it_action`)
and its chrome helpers (`_content_dims`, `_draw_chrome`, `_log_handle_key`), so
they share look, keys and the log pane for free. Live pages set a
`stdscr.timeout(...)` so `getch` doubles as the refresh timer; every hardware
call is wrapped in `try/except` + `_quiet()` so a bus error becomes a status
line, and a stray `print` inside a helper can't corrupt the curses screen.

## 4. Log capture — why the server can't scribble on the screen

A background server that prints would corrupt any curses display. While the
panel is up, `control_panel` therefore redirects **stdout, stderr and root
logging** into a 1000-line ring buffer (`_LOG_BUF`, fed by the line-splitting
`_BufStream`), and the ring buffer is what the bottom **log pane** on every page
renders (`_log_enable`). Two subtleties:

* `ServerClass` installs its own `coloredlogs` handler at import; on server
  start it is removed and the logger set to propagate, so server logs land in
  the buffer with everything else.
* Handlers created after the redirect bind to the redirected `sys.stderr`, so
  even late imports are captured.

Everything is restored on exit — one-shot commands and the no-TTY snapshot log
normally to the terminal.

## 5. One-shot commands

With arguments, no TUI is started — the command runs against the freshly
constructed `Servoset` and the process exits (INFO logging on, so you see the
moves):

```bash
python app/servo_server.py set_zero                     # current pose becomes 0° for all servos
python app/servo_server.py home                         # all servos to 0°
python app/servo_server.py set_angle 10 -5 0 0 0 0 0 0  # absolute angles (deg), one per channel
python app/servo_server.py set_single 3 12.5            # channel index 3 to 12.5°
```

`set_angle` requires exactly one angle per configured channel; an unknown
command name exits with the list of valid ones. Note the one-shot `set_zero`
does **not** ask for confirmation — that guard exists only in the panel.

## 6. Related pages

* [usage.md](usage.md) — setup, config files, and where this fits among the scripts
* [motor.md](motor.md) — encoder geometry, multi-turn registers, de-hysteresis
* [optimize.md](optimize.md) — what the objective functions are for
