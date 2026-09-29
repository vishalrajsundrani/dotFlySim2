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

RUNNING = "running"
STARTING = "starting"

# Units that get their own terminal window the moment they start.
#
# These are the ones whose output is worth watching live and far too long for
# the dashboard's log pane: Gazebo and PX4's console, the bridge's route table,
# RViz's complaints, QGC's link messages, a mission's step transitions, a bag's
# progress line. Opening the window when the unit starts is what an operator
# would do by hand a second later anyway.
#
# `cameras` is deliberately absent -- it is a quiet relay that says almost
# nothing -- and so is `record`, whose one useful number (elapsed, size) is
# already on the dashboard. A window per unit only helps while every window
# earns its place.
TERMINAL_UNITS = ("sim", "bridge", "project", "rviz", "qgc", "replay")

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
        self.editable: dict = {}
        self.cam_enabled: set[str] = set()
        self.busy: set[str] = set()
        # The picker overlays the dashboard rather than replacing the app's
        # state, so events keep arriving and the simulation keeps running
        # while you browse. None means the dashboard is showing.
        self.picker: dict | None = None
        # A modal question drawn over everything: {"text", "options", "on"}.
        self.confirm: dict | None = None
        # A modal text field: {"text", "hint", "value", "on", "check"}.
        self.prompt: dict | None = None
        # A bag's full detail, shown before replaying it.
        self.bag: dict | None = None
        # Units whose terminal window walker has already opened, so a unit that
        # flaps between states does not spawn a window every time.
        self.opened: set[str] = set()
        self.quit = False
        self.shutdown_on_quit = False
        self._lock = threading.RLock()

    # ── events from walkerd ──────────────────────────────────────────────────

    def on_event(self, ev: dict) -> None:
        kind = ev.get("ev")
        with self._lock:
            if kind == "unit":
                name, state = ev["name"], ev.get("state")
                self.units[name] = ev
                # A window per noisy unit (see TERMINAL_UNITS), opened at
                # STARTING rather than at running. The difference is the
                # whole point for a project: its unit spends its first minute
                # compiling, and a window that appears only once the mission is
                # running shows none of the build. The same holds for the
                # simulation, whose most interesting output -- Gazebo loading
                # the world, PX4 booting, which airframe it chose -- all happens
                # before it is ready.
                if (state in (STARTING, RUNNING) and name in TERMINAL_UNITS
                        and name not in self.opened):
                    self.opened.add(name)
                    threading.Thread(target=self.open_terminal, args=(name,),
                                     daemon=True).start()
                elif state in ("stopped", "failed"):
                    # Let it open a fresh window next time it runs.
                    self.opened.discard(name)
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
                self.editable = st.get("editable", {})
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

    # ── a modal question ─────────────────────────────────────────────────────

    def ask(self, text: str, options: list[tuple[str, str]], on) -> None:
        """
        Put a question on screen. `options` is [(key, label)], `on(key)` acts.

        Used for the one decision walker must not guess: what to do with the
        running stack when you leave. Quitting silently either way is wrong --
        tearing down a simulation someone wanted to keep, or leaving a container
        and a Gazebo running for someone who thought they had closed it.
        """
        self.confirm = {"text": text, "options": options, "on": on}

    def draw_confirm(self) -> None:
        h, w = self.scr.getmaxyx()
        cf = self.confirm
        lines = [cf["text"], ""] + [f"  [{k}]  {label}" for k, label in cf["options"]]
        box_w = min(w - 4, max(len(l) for l in lines) + 6)
        box_h = len(lines) + 4
        top = max(0, (h - box_h) // 2)
        left = max(0, (w - box_w) // 2)
        for i in range(box_h):
            self.put(top + i, left, " " * box_w, curses.color_pair(C_HEAD))
        self.put(top + 1, left + 3, cf["text"],
                 curses.color_pair(C_HEAD) | curses.A_BOLD)
        for i, (k, label) in enumerate(cf["options"]):
            self.put(top + 3 + i, left + 3, f"[{k}]  {label}",
                     curses.color_pair(C_HEAD))
        self.scr.refresh()

    # ── a modal text field ───────────────────────────────────────────────────

    def ask_text(self, text: str, hint: str, on, check=None,
                 value: str = "") -> None:
        """
        Ask for a line of text. `on(value)` acts; `check(value)` returns "" or
        a complaint, shown live as you type.

        Used for naming a recording. A name is asked for BEFORE the recorder
        starts, not afterwards, so a flight is never captured into something
        called `run_2026-09-29_20-45-13` that nobody can identify a week later.
        """
        self.prompt = {"text": text, "hint": hint, "value": value,
                       "on": on, "check": check, "problem": ""}

    def draw_prompt(self) -> None:
        h, w = self.scr.getmaxyx()
        pr = self.prompt
        box_w = min(w - 6, 74)
        left = max(0, (w - box_w) // 2)
        top = max(0, h // 2 - 4)
        for i in range(8):
            self.put(top + i, left, " " * box_w, curses.color_pair(C_HEAD))
        self.put(top + 1, left + 3, pr["text"],
                 curses.color_pair(C_HEAD) | curses.A_BOLD)
        self.put(top + 2, left + 3, pr["hint"], curses.color_pair(C_HEAD))
        field = (pr["value"] + "_")[:box_w - 8]
        self.put(top + 4, left + 3, "> " + field.ljust(box_w - 8),
                 curses.color_pair(C_HEAD) | curses.A_BOLD)
        if pr["problem"]:
            self.put(top + 5, left + 3, pr["problem"][:box_w - 6],
                     curses.color_pair(C_FAIL) | curses.A_BOLD)
        self.put(top + 6, left + 3, "[enter] confirm   [esc] cancel",
                 curses.color_pair(C_HEAD))
        self.scr.refresh()

    def prompt_key(self, k: str, ch: int) -> None:
        pr = self.prompt
        if ch == 27:
            self.prompt = None
            return
        if ch in (curses.KEY_ENTER, 10, 13):
            if pr["problem"]:
                return                      # refuse to submit a bad value
            value = pr["value"].strip()
            if pr["check"]:
                problem = pr["check"](value)
                if problem:
                    pr["problem"] = problem
                    return
            self.prompt = None
            pr["on"](value)
            return
        if ch in (curses.KEY_BACKSPACE, 127, 8):
            pr["value"] = pr["value"][:-1]
        elif k and k.isprintable():
            pr["value"] += k
        # Validate as you type, so a clash is visible before you commit.
        pr["problem"] = pr["check"](pr["value"].strip()) if pr["check"] else ""

    # ── the picker ───────────────────────────────────────────────────────────

    def _locked(self, setting: str) -> bool:
        """True (and says so) when this setting cannot be changed right now."""
        if self.editable.get(setting, True):
            return False
        sim = self.units.get("sim", {}).get("state", "stopped")
        if sim in ("running", "starting"):
            self.say(f"{setting} is fixed while the simulation runs — "
                     f"it is read when the simulation is composed. Stop it (s) first.", 10)
        else:
            self.say(f"{setting} needs a running simulation (s)", 8)
        return True

    def open_picker(self, kind: str) -> None:
        """
        Open a chooser over the dashboard.

        Rescans on open, always. A cached list is fine for redrawing, but the
        moment you PRESS the key you are usually asking because you just put
        something in the directory -- so that press is the one time the cache
        must not be trusted.
        """
        if kind == "project":
            # Built from walkerd's view, not the host scan: only walkerd knows
            # whether a binary is up to date with its sources, and running a
            # stale mission is the failure that wastes the most time.
            try:
                rows = self.c.call("projects").get("projects", [])
            except WalkerdError as e:
                self.say(str(e).splitlines()[0]); return
            items = []
            for r in rows:
                state = r.get("state", "?")
                items.append(scan.Entry(
                    name=r["name"], path=paths.REPO, kind="project",
                    title={"built": "ready", "stale": "needs rebuild",
                           "unbuilt": "never built", "broken": "broken",
                           "missing": "missing"}.get(state, state),
                    detail=r.get("wants", "") or r.get("detail", ""),
                    error="" if state in ("built", "stale", "unbuilt") else r.get("detail", "")))
            current = self.selection.get("project", "")
        elif kind == "bag":
            try:
                rows = self.c.call("bags").get("bags", [])
            except WalkerdError as e:
                self.say(str(e).splitlines()[0]); return
            if not rows:
                self.say("no recordings in bags/ yet — press r to make one", 8)
                return
            items = []
            for b in rows:
                mins, secs = divmod(int(b["duration_s"]), 60)
                items.append(scan.Entry(
                    name=b["name"], path=paths.REPO, kind="bag",
                    title=f"{mins}:{secs:02d} · {b['messages']:,} msgs",
                    detail=(f"{b['n_topics']} topics · {b['n_services']} services · "
                            f"{b['size_bytes']/1e6:.0f} MB · "
                            + ("can fly back" if b["can_fly"] else "telemetry only")),
                    error=b.get("error", "")))
            current = ""
        elif kind == "cameras":
            # EVERY LENS, INDIVIDUALLY. The four profiles remain as one-key
            # shortcuts because that is how people talk about the cameras, but
            # they cannot express "tele at preview plus the downward fisheye",
            # which is a perfectly reasonable thing to want.
            try:
                r = self.c.call("camera_list")
            except WalkerdError as e:
                self.say(str(e).splitlines()[0]); return
            self.cam_enabled = set(r.get("enabled", []))
            items = [scan.Entry(name=cam["topic"], path=paths.REPO, kind="camera",
                                title=cam["label"], detail=cam["detail"],
                                meta={"group": cam["group"]})
                     for cam in r.get("cameras", [])]
            current = ""
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
            # Enter ALWAYS rebuilds. For a mission you are iterating on, that
            # is what you want every time: the alternative is running yesterday's
            # binary because a timestamp comparison disagreed with you.
            self.commit_picker(rebuild=True)
        elif ch == -2 or k == "b":
            # Shift+Enter (or `b`, which always works -- see the note on
            # modifyOtherKeys) reuses an existing build and only compiles when
            # there is nothing to run.
            self.commit_picker(rebuild=False)
        elif k in ("\x1b", "q"):
            self.picker = None
        elif pk["kind"] == "cameras" and k == " ":
            self.toggle_camera(pk["items"][pk["idx"]].name)
        elif pk["kind"] == "cameras" and k in "1234":
            self.apply_camera_profile(["none", "fisheye", "payload", "all"][int(k) - 1])
        elif ch == curses.KEY_F5 or k == "5":
            scan.invalidate()
            self.open_picker(pk["kind"])
            self.say("rescanned")

    def start_recording(self) -> None:
        """Ask for a name, then record. The name comes first, always."""
        def check(value: str) -> str:
            if not value:
                return ""
            try:
                return self.c.call("check_name", name=value).get("problem", "")
            except WalkerdError:
                return ""

        def begin(name: str) -> None:
            def go():
                r = self.c.start("record", name=name, scope="wrapper")
                if r.get("ok"):
                    self.say(f"recording into bags/{name}  —  r again to stop", 10)
                else:
                    self.say(f"record refused: {r.get('reason','?')}", 12)
            self.bg("record", go)

        self.ask_text("Name this recording",
                      "it lands in bags/<name>/ — letters, digits, dot, dash, underscore",
                      begin, check)

    def toggle_camera(self, topic: str) -> None:
        """Flip one lens. Takes effect immediately on a running simulation."""
        def go():
            r = self.c.call("cameras", args={"toggle": topic})
            if r.get("ok"):
                with self._lock:
                    self.cam_enabled = set(r.get("enabled", []))
                    self.selection["cameras"] = r.get("profile", "custom")
                on = topic in self.cam_enabled
                self.say(f"{'on ' if on else 'off'} {topic}"
                         f"   ({len(self.cam_enabled)} rendering)", 5)
            else:
                self.say(f"cameras: {r.get('reason','?')}  [{r.get('hint','')}]", 10)
        self.bg("cameras", go)

    def apply_camera_profile(self, profile: str) -> None:
        def go():
            r = self.c.call("cameras", args={"profile": profile})
            if r.get("ok"):
                with self._lock:
                    self.cam_enabled = set(r.get("enabled", []))
                    self.selection["cameras"] = profile
                self.say(f"{profile}: {len(self.cam_enabled)} camera(s) rendering", 6)
            else:
                self.say(f"cameras: {r.get('reason','?')}  [{r.get('hint','')}]", 10)
        self.bg("cameras", go)

    def commit_picker(self, rebuild: bool = True) -> None:
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

        live_sim = sim_state in ("running", "starting")

        def go():
            # CAMERAS APPLY IMMEDIATELY; everything else is read when the
            # simulation is composed. That difference is the whole point of
            # subscription-driven rendering, and hiding it would make the two
            # cases look alike when they behave nothing alike.
            if field == "bag":
                # Look before you replay: the detail screen, not straight to
                # playing something you have not seen.
                self.open_bag(entry.name)
                return
            if field == "cameras":
                self.toggle_camera(entry.name)
                return
            if field == "project":
                r = self.c.start("project", package=entry.name, rebuild=rebuild)
                if r.get("ok"):
                    with self._lock:
                        self.selection["project"] = entry.name
                    self.say(f"{entry.name}: "
                             + ("rebuilding, then flying" if rebuild
                                else "reusing the existing build" ) 
                             + " — watch its terminal", 8)
                    for a in r.get("advisories", []):
                        self.say(a, 10)
                else:
                    self.say(f"{entry.name} refused: {r.get('reason','?')}  "
                             f"[{r.get('hint','')}]", 12)
                return

            if field == "cameras" and live_sim:
                r = self.c.call("cameras", args={"profile": entry.name})
                if r.get("ok"):
                    with self._lock:
                        self.selection["cameras"] = entry.name
                    self.say("cameras: " + ", ".join(r.get("applied", [])), 6)
                else:
                    self.say(f"cameras: {r.get('reason','?')}  [{r.get('hint','')}]", 10)
                return

            r = self.c.select(**{field: entry.name})
            if not r.get("ok"):
                # walkerd owns the gating rules; a refusal is shown verbatim
                # rather than second-guessed here.
                self.say(f"{field} is locked: {r.get('reason','?')}  "
                         f"[{r.get('hint','')}]", 12)
                return
            with self._lock:
                self.selection[field] = entry.name
            if live_sim:
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
                   "cameras": ("Cameras — each lens switches on its own; "
                               "RViz can then look through it"),
                   "project": "Projects — projects/   (enter builds if needed, then flies)",
                   "bag": "Recordings — bags/   (enter shows what is inside)"}[kind]
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
            if kind == "cameras":
                on = e.name in self.cam_enabled
                self.put(y, 3, "[x]" if on else "[ ]",
                         curses.color_pair(C_OK if on else C_DIM) | curses.A_BOLD)
                self.put(y, 7, (e.title or "")[:24].ljust(25), attr)
                self.put(y, 33, e.detail[:20].ljust(21), curses.color_pair(C_DIM))
                self.put(y, 55, e.name[:max(0, w - 57)], curses.color_pair(C_DIM))
                continue
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
        if pk["kind"] == "cameras":
            bar = (f" [space/enter] toggle   [1] none [2] fisheye [3] payload [4] all"
                   f"   [esc] back      {len(self.cam_enabled)} rendering ")
        elif pk["kind"] == "project":
            bar = (" [↑↓/jk] move   [enter] rebuild + fly   [shift+enter / b] fly "
                   "existing   [F5] rescan   [esc] back ")
        else:
            bar = (" [↑↓/jk] move   [enter] choose   [F5] rescan   [esc] back "
                   "   • = current ")
        self.put(h - 1, 0, bar.ljust(w), curses.color_pair(C_HEAD))

    # ── a bag, in detail ─────────────────────────────────────────────────────

    def open_bag(self, name: str) -> None:
        """
        Show everything about a recording before replaying it.

        Replaying a bag you have not looked at is how you discover, two minutes
        in, that it holds no command topics and the aircraft was never going to
        move. Everything needed to know that is in the metadata; this screen
        shows it.
        """
        def go():
            r = self.c.call("bag_detail", bag=name)
            if r.get("ok"):
                with self._lock:
                    self.bag = r["bag"]
            else:
                self.say(f"{name}: {r.get('reason','not found')}", 8)
        self.bg("bag", go)

    def draw_bag(self) -> None:
        b = self.bag
        h, w = self.scr.getmaxyx()
        self.put(0, 0, f" Bag — {b['name']} ".ljust(w),
                 curses.color_pair(C_HEAD) | curses.A_BOLD)

        mins, secs = divmod(int(b["duration_s"]), 60)
        kind = "can fly the aircraft back" if b["can_fly"] else "telemetry only — nothing will move"
        stats = [
            ("duration", f"{mins}:{secs:02d}"),
            ("messages", f"{b['messages']:,}"),
            ("size", f"{b['size_bytes']/1e6:.1f} MB"),
            ("storage", b["storage"]),
            ("topics", str(b["n_topics"])),
            ("service calls", str(b["n_services"])),
            ("camera streams", str(b["n_cameras"])),
            ("command topics", f"{b['n_commands']}  ({kind})"),
        ]
        row = 2
        for label, value in stats:
            self.put(row, 3, label, curses.color_pair(C_DIM))
            colour = 0
            if label == "command topics":
                colour = curses.color_pair(C_OK if b["can_fly"] else C_WARN)
            self.put(row, 20, value, colour | curses.A_BOLD)
            row += 1

        if b.get("error"):
            self.put(row + 1, 3, b["error"][:w - 6], curses.color_pair(C_FAIL))
            row += 2

        row += 1
        self.put(row, 3, "topics, by message count", curses.color_pair(C_DIM))
        row += 1
        self.put(row, 3, "topic", curses.color_pair(C_DIM))
        self.put(row, 52, "msgs", curses.color_pair(C_DIM))
        self.put(row, 62, "Hz", curses.color_pair(C_DIM))
        self.put(row, 70, "type", curses.color_pair(C_DIM))
        row += 1

        # Services are counted above but not listed one by one: 59 rows of
        # _service_event would bury the topics, and the count is the useful
        # part (it says whether the takeoff that started the flight is in here).
        shown = [t for t in b["topics"] if not t["name"].endswith("/_service_event")]
        room = h - row - 3
        for t_ in shown[:room]:
            name = t_["name"].replace("/wrapper/psdk_ros2/", "")
            self.put(row, 3, name[:47])
            self.put(row, 52, f"{t_['count']:,}"[:9])
            self.put(row, 62, f"{t_['hz']:.1f}")
            self.put(row, 70, t_["type"].split("/")[-1][:max(0, w - 72)],
                     curses.color_pair(C_DIM))
            row += 1
        if len(shown) > room > 0:
            self.put(row, 3, f"... and {len(shown) - room} more",
                     curses.color_pair(C_DIM))

        if self.status and time.time() < self.status_until:
            self.put(h - 2, 2, self.status, curses.color_pair(C_WARN))
        self.put(h - 1, 0, " [enter] replay this bag   [esc] back ".ljust(w),
                 curses.color_pair(C_HEAD))
        self.scr.refresh()

    def bag_key(self, k: str, ch: int) -> None:
        if ch == 27 or k == "q":
            self.bag = None
            return
        if ch in (curses.KEY_ENTER, 10, 13):
            name = self.bag["name"]
            self.bag = None

            def go():
                r = self.c.start("replay", bag=name)
                if r.get("ok"):
                    self.say(f"replaying {name} — watch its terminal", 8)
                else:
                    self.say(f"replay refused: {r.get('reason','?')}  "
                             f"[{r.get('hint','')}]", 12)
            self.bg("replay", go)

    # ── drawing ──────────────────────────────────────────────────────────────

    def draw(self) -> None:
        self.scr.erase()
        if self.prompt is not None:
            self._draw_dashboard()
            self.draw_prompt()
            return
        if self.bag is not None:
            self.draw_bag()
            return
        if self.confirm is not None:
            # Drawn over the dashboard rather than instead of it, so the state
            # you are deciding about stays visible behind the question.
            self._draw_dashboard()
            self.draw_confirm()
            return
        if self.picker is not None:
            self.draw_picker()
            return
        self._draw_dashboard()
        self.scr.refresh()

    def _draw_dashboard(self) -> None:
        s = self.scr
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
            ("WINDOW", "g", "gazebo window" if sel.get("gui", True) else "headless"),
        ):
            setting = {"WORLD": "world", "DRONE": "drone",
                       "CAMERAS": "cameras", "WINDOW": "gui"}[label]
            # `editable` comes from walkerd, which is also what enforces it.
            can = self.editable.get(setting, True)
            self.put(row, 2, label, curses.color_pair(C_DIM))
            self.put(row, 12, str(value)[:28],
                     curses.A_BOLD if can else curses.color_pair(C_DIM))
            if can:
                self._key(row, w - 14, key, "change")
            else:
                # A locked setting shows WHY at a glance, so nobody presses the
                # key three times wondering if the keyboard is broken.
                self.put(row, w - 14, "  ⨯ ", curses.color_pair(C_DIM))
                self.put(row, w - 10, "sim up", curses.color_pair(C_DIM))
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
        bar = (" [s]im [b]ridge [p]roject [r]ec [R]eplay [c]ams [v]iz [q]gc "
               "[t]erm [?]help [Q]uit ")
        self.put(h - 1, 0, bar.ljust(w), curses.color_pair(C_HEAD))
        s.refresh()

    def _key(self, y: int, x: int, key: str, label: str) -> None:
        if x < 4:
            return
        self.put(y, x, f"[{key}]", curses.color_pair(C_KEY) | curses.A_BOLD)
        self.put(y, x + 4, label[:9], curses.color_pair(C_DIM))

    # ── input ────────────────────────────────────────────────────────────────

    def _read_modified(self) -> tuple[int, int] | None:
        """
        After an ESC, try to read a modifyOtherKeys sequence: ESC [ 27;m;k ~

        Returns (modifier, key) or None, having consumed only what it read. A
        bare ESC (the cancel key) arrives as an ESC with nothing behind it, and
        getch returning -1 is how that is told apart.
        """
        buf = ""
        for _ in range(12):
            c = self.scr.getch()
            if c == -1:
                break
            buf += chr(c)
            if c == ord("~"):
                break
        if buf.startswith("[27;") and buf.endswith("~"):
            try:
                _, mod, key = buf[1:-1].split(";")
                return int(mod), int(key)
            except ValueError:
                return None
        return None

    def key(self, ch: int) -> None:
        try:
            k = chr(ch) if 0 <= ch < 0x110000 else ""
        except ValueError:
            k = ""
        if ch == 27:
            mod_key = self._read_modified()
            if mod_key == SHIFT_ENTER[1:]:
                ch, k = -2, ""          # -2 is walker's "Shift+Enter"
            elif mod_key is not None:
                return                  # some other modified key: ignore
        if self.prompt is not None:
            self.prompt_key(k, ch)
            return
        if self.bag is not None:
            self.bag_key(k, ch)
            return
        if self.confirm is not None:
            cf = self.confirm
            if k in [o[0] for o in cf["options"]]:
                self.confirm = None
                cf["on"](k)
            elif ch == 27:                      # esc cancels
                self.confirm = None
            return
        if self.picker is not None:
            self.picker_key(k, ch)
            return
        if ch == curses.KEY_F5:
            scan.invalidate()
            self.refresh_state()
            self.say("rescanned models/, worlds/, bags/, projects/")
            return
        if k in ("Q",):
            def decide(choice: str) -> None:
                if choice == "l":
                    self.quit = True
                elif choice == "s":
                    self.shutdown_on_quit = True
                    self.quit = True
            self.ask("Leave walker — what about the running stack?",
                     [("l", "leave it running (walker reopens instantly)"),
                      ("s", "stop everything: units, then the container"),
                      ("esc", "cancel")],
                     decide)
        elif k == "s":
            self.toggle_unit("sim")
        elif k == "p":
            state = self.units.get("project", {}).get("state", "stopped")
            if state in ("running", "starting"):
                self.toggle_unit("project")          # stop the running mission
            else:
                self.open_picker("project")          # choose which one to fly
        elif k == "r":
            state = self.units.get("record", {}).get("state", "stopped")
            if state in ("running", "starting"):
                self.bg("record", lambda: (self.c.stop("record"),
                                           self.say("closing the recording…", 8)))
            else:
                self.start_recording()
        elif k == "R":
            state = self.units.get("replay", {}).get("state", "stopped")
            if state in ("running", "starting"):
                self.bg("replay", lambda: (self.c.stop("replay"),
                                           self.say("stopping the replay", 6)))
            else:
                self.open_picker("bag")
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
            self.say("keys: s sim · w world · d drone · c cameras · g window · "
                     "F5 rescan · P probe · t terminal · Q quit", 10)
        elif k == "w":
            if not self._locked("world"):
                self.open_picker("world")
        elif k == "d":
            if not self._locked("drone"):
                self.open_picker("drone")
        elif k == "c":
            if not self._locked("cameras"):
                self.open_picker("cameras")
        elif k == "g":
            if self._locked("gui"):
                return
            want = not self.selection.get("gui", True)

            def go():
                self.c.select(gui=want)
                with self._lock:
                    self.selection["gui"] = want
                running = self.units.get("sim", {}).get("state") in ("running", "starting")
                self.say(("gazebo window on" if want else "headless")
                         + (" — restart the sim (s) for it to take effect" if running else ""),
                         10 if running else 5)
            self.bg("select", go)

    def open_terminal(self, unit: str) -> None:
        from .terminal import open_window
        cmd = (f"docker exec -it {paths.CONTAINER} walker-attach {unit}")
        ok, note = open_window(f"walker · {unit}", cmd)
        self.say(note.splitlines()[0], 10)

    # ── loop ─────────────────────────────────────────────────────────────────

    def run(self) -> None:
        self.scr.nodelay(True)
        # Ask for modified keys; harmless where unsupported.
        try:
            import sys as _sys
            _sys.stdout.write(MODIFY_OTHER_KEYS_ON)
            _sys.stdout.flush()
        except Exception:
            pass
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
        try:
            import sys as _sys
            _sys.stdout.write(MODIFY_OTHER_KEYS_OFF)
            _sys.stdout.flush()
        except Exception:
            pass


def dump(stdscr, settle: float = 6.0, keys: str = "", hold: float = 0.6) -> str:
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
    # `hold` keeps the app alive after the keystrokes so slow consequences --
    # a simulation coming up, a terminal window opening -- actually happen
    # while something is there to react to them.
    end = time.time() + hold
    while time.time() < end:
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


# ── telling Shift+Enter from Enter ───────────────────────────────────────────
#
# A terminal sends the SAME byte (13) for Enter and Shift+Enter by default:
# the modifier is simply not encoded, so no program can tell them apart. It is
# not a curses limitation, it is what is on the wire.
#
# xterm's "modifyOtherKeys" mode changes that. Mode 2 makes modified keys arrive
# as a CSI sequence carrying the modifier, so Shift+Enter becomes
#
#     ESC [ 27 ; 2 ; 13 ~
#
# VTE terminals (ptyxis, gnome-terminal) and xterm support it; others ignore
# the request harmlessly. Where it is ignored, Shift+Enter is indistinguishable
# from Enter, so `b` is documented as doing the same thing and always works.
#
# The mode is turned off again on exit -- leaving a terminal in it makes other
# programs see escape sequences where they expect plain keys.
MODIFY_OTHER_KEYS_ON = "\033[>4;2m"
MODIFY_OTHER_KEYS_OFF = "\033[>4;0m"

# ESC [ 27 ; <mod> ; <key> ~   -- mod 2 is Shift, key 13 is Enter.
SHIFT_ENTER = (27, 2, 13)


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
    # Reported back to the CLI, which owns the container: tearing it down from
    # inside curses would mean drawing a teardown log onto a screen that is
    # about to be restored.
    return 10 if app.shutdown_on_quit else 0


def run() -> int:
    return curses.wrapper(main)


def run_dump(settle: float = 6.0, keys: str = "", hold: float = 0.6) -> int:
    print(curses.wrapper(dump, settle, keys, hold))
    return 0
