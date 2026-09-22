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

from . import paths, scan
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

# The four camera profiles, with what each costs. The megapixel figures are
# measured for the m4e's 12 payload + 2 fisheye sensors (SPIKE-1); walker
# recomputes them per drone from its manifest once more drones exist.
_CAMERA_TITLES = {
    "none":    ("no cameras",      "0 MP/s — fastest; flight work, CI"),
    "fisheye": ("fisheye only",    "~20 MP/s — obstacle avoidance, stereo, SLAM"),
    "payload": ("payload only",    "~1074 MP/s — gimbal/inspection, QGC video"),
    "all":     ("fisheye + payload", "~1094 MP/s — needs a real GPU"),
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
        # The picker overlays the dashboard rather than replacing the app's
        # state, so events keep arriving and the simulation keeps running
        # while you browse. None means the dashboard is showing.
        self.picker: dict | None = None
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

    # ── the picker ───────────────────────────────────────────────────────────

    def open_picker(self, kind: str) -> None:
        """
        Open a chooser over the dashboard.

        Rescans on open, always. A cached list is fine for redrawing, but the
        moment you PRESS the key you are usually asking because you just put
        something in the directory -- so that press is the one time the cache
        must not be trusted.
        """
        if kind == "cameras":
            items = [scan.Entry(name=p, path=paths.REPO, kind="camera",
                                title=_CAMERA_TITLES[p][0],
                                detail=_CAMERA_TITLES[p][1])
                     for p in scan.CAMERA_PROFILES]
            current = self.selection.get("cameras", "none")
        else:
            items = scan.get("worlds" if kind == "world" else "drones", force=True)
            current = self.selection.get(kind, "")

        if not items:
            where = {"world": "worlds/", "drone": "models/"}.get(kind, "")
            self.say(f"nothing to choose: no {kind}s found in {where}", 8)
            return
        idx = next((i for i, e in enumerate(items) if e.name == current), 0)
        self.picker = {"kind": kind, "items": items, "idx": idx}

    def picker_key(self, k: str, ch: int) -> None:
        pk = self.picker
        n = len(pk["items"])
        if ch in (curses.KEY_UP,) or k == "k":
            pk["idx"] = (pk["idx"] - 1) % n
        elif ch in (curses.KEY_DOWN,) or k == "j":
            pk["idx"] = (pk["idx"] + 1) % n
        elif ch in (curses.KEY_ENTER, 10, 13):
            self.commit_picker()
        elif k in ("\x1b", "q"):
            self.picker = None
        elif ch == curses.KEY_F5 or k == "5":
            scan.invalidate()
            self.open_picker(pk["kind"])
            self.say("rescanned")

    def commit_picker(self) -> None:
        pk = self.picker
        entry = pk["items"][pk["idx"]]
        if not entry.usable:
            # Refusing here, with the reason, beats composing something that
            # will fail later inside Gazebo with a URI error.
            self.say(f"{entry.name} cannot be used: {entry.error}", 12)
            return
        field = pk["kind"]
        self.picker = None

        sim_state = self.units.get("sim", {}).get("state", "stopped")

        def go():
            self.c.select(**{field: entry.name})
            with self._lock:
                self.selection[field] = entry.name
            if sim_state in ("running", "starting"):
                # Honest about what a selection does and does not do: the
                # composition is built when the simulation STARTS, so changing
                # the drone or world under a running one changes nothing until
                # it is restarted. V1 had the same property and did not say so.
                self.say(f"{field} = {entry.name} — restart the sim (s) for it "
                         f"to take effect", 12)
            else:
                self.say(f"{field} = {entry.name}", 5)
        self.bg("select", go)

    def draw_picker(self) -> None:
        s, pk = self.scr, self.picker
        h, w = s.getmaxyx()
        kind = pk["kind"]
        heading = {"world": "Worlds — worlds/", "drone": "Drones — models/",
                   "cameras": "Camera profile"}[kind]
        self.put(0, 0, f" {heading} ".ljust(w),
                 curses.color_pair(C_HEAD) | curses.A_BOLD)

        top = 2
        room = h - top - 3
        items = pk["items"]
        # Keep the cursor in view without scrolling for short lists.
        first = max(0, min(pk["idx"] - room // 2, len(items) - room)) if len(items) > room else 0
        for i in range(first, min(len(items), first + room)):
            e = items[i]
            y = top + (i - first)
            sel = (i == pk["idx"])
            attr = curses.A_REVERSE if sel else 0
            mark = "›" if sel else " "
            self.put(y, 1, mark, curses.color_pair(C_KEY) | curses.A_BOLD)
            name = e.name[:20].ljust(21)
            self.put(y, 3, name, attr | (curses.A_BOLD if e.usable else 0))
            self.put(y, 25, (e.title or "")[:30].ljust(31),
                     0 if e.usable else curses.color_pair(C_DIM))
            if e.usable:
                self.put(y, 57, e.detail[:max(0, w - 59)], curses.color_pair(C_DIM))
            else:
                self.put(y, 57, e.error[:max(0, w - 59)], curses.color_pair(C_FAIL))
            if e.name == self.selection.get(kind):
                self.put(y, 0, "•", curses.color_pair(C_OK) | curses.A_BOLD)

        if self.status and time.time() < self.status_until:
            self.put(h - 2, 2, self.status, curses.color_pair(C_WARN))
        bar = (" [↑↓/jk] move   [enter] choose   [F5] rescan   [esc] back "
               "   • = current ")
        self.put(h - 1, 0, bar.ljust(w), curses.color_pair(C_HEAD))
        s.refresh()

    # ── drawing ──────────────────────────────────────────────────────────────

    def draw(self) -> None:
        s = self.scr
        s.erase()
        if self.picker is not None:
            self.draw_picker()
            return
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
            k = chr(ch) if 0 <= ch < 0x110000 else ""
        except ValueError:
            k = ""
        if self.picker is not None:
            self.picker_key(k, ch)
            return
        if ch == curses.KEY_F5:
            scan.invalidate()
            self.refresh_state()
            self.say("rescanned models/, worlds/, bags/, projects/")
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
            self.say("keys: s sim · w world · d drone · c cameras · F5 rescan · "
                     "P probe · t terminal · Q quit", 10)
        elif k == "w":
            self.open_picker("world")
        elif k == "d":
            self.open_picker("drone")
        elif k == "c":
            self.open_picker("cameras")

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


def dump(stdscr, settle: float = 6.0, keys: str = "") -> str:
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
    # Let state arrive first, then apply the keystrokes, then let the result
    # settle -- so a capture shows the screen a person would actually see
    # after pressing those keys, not a half-populated one.
    deadline = time.time() + settle
    while time.time() < deadline:
        app.draw()
        time.sleep(0.1)
    for ch in keys:
        app.key(ord(ch))
        for _ in range(8):
            app.draw()
            time.sleep(0.05)
    for _ in range(12):
        app.draw()
        time.sleep(0.05)
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


def run_dump(settle: float = 6.0, keys: str = "") -> int:
    print(curses.wrapper(dump, settle, keys))
    return 0
