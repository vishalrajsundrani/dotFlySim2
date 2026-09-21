"""
The dashboard: walker's main screen.

DESIGN RULE, ENFORCED THROUGHOUT
================================
THE KEYBOARD IS NEVER BLOCKED. Every call that could take longer than a frame
-- starting a unit, composing, probing -- runs on a worker thread. The render
loop only ever reads an in-memory mirror that walkerd's pushed events keep up
to date. A simulation that takes 20 s to come up must not make the UI feel
like it has hung for 20 s; it shows `starting` and keeps responding to keys.

That mirror is why walker never polls. walkerd pushes a `unit` event the
instant a state changes (§6.3), so the dashboard is current without asking.
"""

from __future__ import annotations

import curses
import threading
import time
from collections import deque

from . import paths
from .client import Client, WalkerdError

# Colour pairs, allocated once in main().
C_OK, C_WARN, C_FAIL, C_DIM, C_HEAD, C_KEY = 1, 2, 3, 4, 5, 6

STATE_COLOUR = {
    "running": C_OK, "starting": C_WARN, "stopping": C_WARN,
    "failed": C_FAIL, "stopped": C_DIM,
}
STATE_MARK = {
    "running": "●", "starting": "◐", "stopping": "◑", "failed": "✖", "stopped": "○",
}

# unit -> the key that starts/stops it. Mirrors §7 so the help screen and the
# dashboard cannot disagree.
UNIT_KEYS = {"sim": "s", "bridge": "b", "project": "p", "qgc": "q",
             "rviz": "v", "record": "r", "replay": "R"}


class App:
    def __init__(self, stdscr, client: Client):
        self.scr = stdscr
        self.c = client
        self.units: dict[str, dict] = {}
        self.selection: dict = {}
        self.log: deque[tuple[float, str, str, str]] = deque(maxlen=500)
        self.status = ""
        self.status_until = 0.0
        self.links: list[dict] = []
        self.busy: set[str] = set()
        self.quit = False
        self._lock = threading.RLock()

    # ── events from walkerd ──────────────────────────────────────────────────

    def on_event(self, ev: dict) -> None:
        kind = ev.get("ev")
        with self._lock:
            if kind == "unit":
                self.units[ev["name"]] = ev
            elif kind == "log":
                self.log.append((ev.get("t", time.time()), ev.get("unit", "?"),
                                 ev.get("level", "info"), ev.get("text", "")))
            elif kind == "selection":
                self.selection.update({k: v for k, v in ev.items() if k != "ev"})

    def say(self, text: str, seconds: float = 6.0) -> None:
        self.status, self.status_until = text, time.time() + seconds

    # ── background work ──────────────────────────────────────────────────────

    def bg(self, name: str, fn) -> None:
        """Run fn on a worker thread, marking `name` busy while it runs."""
        if name in self.busy:
            self.say(f"{name} is already busy")
            return

        def wrap():
            self.busy.add(name)
            try:
                fn()
            except WalkerdError as e:
                self.say(str(e).splitlines()[0])
            except Exception as e:
                self.say(f"{type(e).__name__}: {e}")
            finally:
                self.busy.discard(name)

        threading.Thread(target=wrap, daemon=True).start()

    def toggle_unit(self, name: str) -> None:
        state = self.units.get(name, {}).get("state", "stopped")
        if state in ("running", "starting"):
            self.bg(name, lambda: (self.c.stop(name), self.say(f"stopping {name}")))
        else:
            def go():
                r = self.c.start(name)
                if r.get("ok"):
                    for a in r.get("advisories", []):
                        self.say(a, 10)
                else:
                    # A refusal always names the holder and the way out; that
                    # is the whole point of the constraint table.
                    self.say(f"{name} refused: {r.get('reason','?')}  "
                             f"[{r.get('hint','')}]", 12)
            self.bg(name, go)

    def refresh_state(self) -> None:
        def go():
            st = self.c.state()
            with self._lock:
                self.units = {u["name"]: u for u in st["units"]}
                self.selection = st.get("selection", {})
            pr = self.c.probe()
            with self._lock:
                self.links = pr.get("links", [])
        self.bg("state", go)


    # ── safe drawing ─────────────────────────────────────────────────────────

    def put(self, y: int, x: int, text: str, attr: int = 0) -> None:
        """
        addstr that cannot throw.

        curses raises on a write to the last cell of the last line -- it tries
        to advance the cursor past the end of the screen -- so a full-width
        status bar at the bottom crashes the whole UI. It also raises for any
        write starting off-screen, which happens the moment someone makes the
        window narrow. Neither is worth taking the dashboard down for, so every
        write goes through here and is clipped instead.
        """
        h, w = self.scr.getmaxyx()
        if y < 0 or y >= h or x < 0 or x >= w:
            return
        room = w - x
        # The bottom-right cell is the one that cannot be written at all.
        if y == h - 1:
            room -= 1
        if room <= 0:
            return
        try:
            self.scr.addnstr(y, x, text, room, attr)
        except curses.error:
            pass

    # ── drawing ──────────────────────────────────────────────────────────────

    def draw(self) -> None:
        s = self.scr
        s.erase()
        h, w = s.getmaxyx()
        if h < 18 or w < 70:
            self.put(0, 0, "terminal too small (need 70x18)"[:w - 1])
            s.refresh()
            return

        sel = self.selection
        title = " dotFlySim2 · walker "
        self.put(0, 0, title.ljust(w), curses.color_pair(C_HEAD) | curses.A_BOLD)

        row = 2
        for label, key, value in (
            ("WORLD", "w", sel.get("world", "-")),
            ("DRONE", "d", sel.get("drone", "-")),
            ("CAMERAS", "c", sel.get("cameras", "-")),
        ):
            self.put(row, 2, label, curses.color_pair(C_DIM))
            self.put(row, 12, str(value)[:28], curses.A_BOLD)
            self._key(row, w - 14, key, "change")
            row += 1

        row += 1
        self.put(row, 2, "units", curses.color_pair(C_DIM)); row += 1
        for name in ("sim", "bridge", "project", "qgc", "rviz", "record", "replay"):
            u = self.units.get(name)
            state = u["state"] if u else "stopped"
            known = u is not None
            colour = curses.color_pair(STATE_COLOUR.get(state, C_DIM))
            self.put(row, 4, STATE_MARK.get(state, "○"), colour)
            self.put(row, 6, name.ljust(10),
                     curses.A_BOLD if state == "running" else curses.A_NORMAL)
            label = state if known else "not yet implemented"
            self.put(row, 17, label.ljust(12)[:12],
                     colour if known else curses.color_pair(C_DIM))
            if name in self.busy:
                self.put(row, 30, "working…", curses.color_pair(C_WARN))
            elif u and u.get("uptime"):
                self.put(row, 30, f"{u['uptime']:>5.0f}s", curses.color_pair(C_DIM))
            detail = (u or {}).get("detail", "")
            if detail:
                self.put(row, 38, detail[:max(0, w - 52)],
                         curses.color_pair(C_FAIL if state == "failed" else C_DIM))
            if known:
                self._key(row, w - 14, UNIT_KEYS.get(name, "?"),
                          "stop" if state in ("running", "starting") else "start")
            row += 1

        # Links: the chain, as the persistent watcher sees it right now.
        if self.links and row < h - 8:
            row += 1
            self.put(row, 2, "links", curses.color_pair(C_DIM)); row += 1
            for l in self.links[:6]:
                if row >= h - 6:
                    break
                ok = l["state"] == "ok"
                self.put(row, 4, "✔" if ok else "✖",
                         curses.color_pair(C_OK if ok else C_FAIL))
                self.put(row, 6, l["name"].ljust(24)[:24])
                self.put(row, 31, l["detail"][:14],
                         curses.color_pair(C_OK if ok else C_FAIL))
                row += 1

        # Log pane fills whatever is left.
        log_top = row + 1
        avail = h - log_top - 2
        if avail > 1:
            self.put(log_top - 1, 2, "log", curses.color_pair(C_DIM))
            with self._lock:
                lines = list(self.log)[-avail:]
            for i, (t, unit, level, text) in enumerate(lines):
                y = log_top + i
                self.put(y, 2, time.strftime("%H:%M:%S", time.localtime(t)),
                         curses.color_pair(C_DIM))
                self.put(y, 11, unit[:8].ljust(9), curses.color_pair(C_DIM))
                colour = curses.color_pair(C_WARN if level == "warn" else 0)
                self.put(y, 20, text[:max(0, w - 22)], colour)

        # Status line, then the key bar.
        if self.status and time.time() < self.status_until:
            self.put(h - 2, 2, self.status, curses.color_pair(C_WARN))
        bar = " [s]im  [w]orld  [d]rone  [c]ameras  [t]erminal  [P]robe  [?]help  [Q]uit "
        self.put(h - 1, 0, bar.ljust(w), curses.color_pair(C_HEAD))
        s.refresh()

    def _key(self, y: int, x: int, key: str, label: str) -> None:
        if x < 4:
            return
        self.put(y, x, f"[{key}]", curses.color_pair(C_KEY) | curses.A_BOLD)
        self.put(y, x + 4, label[:9], curses.color_pair(C_DIM))

    # ── input ────────────────────────────────────────────────────────────────

    def key(self, ch: int) -> None:
        try:
            k = chr(ch)
        except ValueError:
            return
        if k in ("Q",):
            self.quit = True
        elif k == "s":
            self.toggle_unit("sim")
        elif k in UNIT_KEYS.values():
            name = next((n for n, kk in UNIT_KEYS.items() if kk == k), None)
            if name in self.units:
                self.toggle_unit(name)
            else:
                self.say(f"'{name}' arrives in a later milestone")
        elif k == "P":
            self.refresh_state()
            self.say("probing…", 2)
        elif k == "t":
            self.open_terminal("sim")
        elif k == "?":
            self.say("keys: s sim · P probe · t terminal · Q quit   "
                     "(full help screen lands with the other units)", 10)
        elif k in ("w", "d", "c"):
            self.say(f"the {'world' if k=='w' else 'drone' if k=='d' else 'cameras'}"
                     f" screen lands in M3", 6)

    def open_terminal(self, unit: str) -> None:
        from .terminal import open_window
        cmd = (f"docker exec -it {paths.CONTAINER} walker-attach {unit}")
        ok, note = open_window(f"walker · {unit}", cmd)
        self.say(note.splitlines()[0], 10)

    # ── loop ─────────────────────────────────────────────────────────────────

    def run(self) -> None:
        self.scr.nodelay(True)
        self.refresh_state()
        last_refresh = time.time()
        while not self.quit:
            ch = self.scr.getch()
            if ch != -1:
                self.key(ch)
            # A periodic resync: events keep the mirror current, but a probe
            # has to be asked for, and a reconnect needs a full snapshot.
            if time.time() - last_refresh > 2.0:
                self.refresh_state()
                last_refresh = time.time()
            self.draw()
            time.sleep(0.05)


def dump(stdscr, settle: float = 6.0) -> str:
    """
    Draw one settled frame and return the screen as text.

    WHY THIS EXISTS. A curses layout cannot be checked by reading the code, and
    capturing a real terminal means re-implementing one -- incremental updates
    superimpose frames and the result is unreadable. Asking curses itself what
    is on screen (`instr`) is exact, and it makes the dashboard's layout
    testable in CI, where there is no terminal at all.
    """
    _init_colours()
    app = App(stdscr, None)
    client = Client(paths.socket_path(), on_event=app.on_event)
    client.connect()
    app.c = client
    app.refresh_state()
    deadline = time.time() + settle
    while time.time() < deadline:
        app.draw()
        time.sleep(0.1)
    h, w = stdscr.getmaxyx()
    rows = []
    for y in range(h):
        try:
            rows.append(stdscr.instr(y, 0, w - 1).decode("utf-8", "replace").rstrip())
        except curses.error:
            rows.append("")
    client.close()
    return "\n".join(rows)


def _init_colours() -> None:
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(C_OK, curses.COLOR_GREEN, -1)
    curses.init_pair(C_WARN, curses.COLOR_YELLOW, -1)
    curses.init_pair(C_FAIL, curses.COLOR_RED, -1)
    curses.init_pair(C_DIM, curses.COLOR_WHITE, -1)
    curses.init_pair(C_HEAD, curses.COLOR_BLACK, curses.COLOR_CYAN)
    curses.init_pair(C_KEY, curses.COLOR_CYAN, -1)


def main(stdscr) -> int:
    _init_colours()
    app = App(stdscr, None)
    client = Client(paths.socket_path(), on_event=app.on_event)
    client.connect()
    app.c = client
    try:
        app.run()
    finally:
        client.close()
    return 0


def run() -> int:
    return curses.wrapper(main)


def run_dump(settle: float = 6.0) -> int:
    print(curses.wrapper(dump, settle))
    return 0
