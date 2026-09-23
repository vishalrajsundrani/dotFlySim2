"""
What may run at the same time as what.

THIS TABLE IS THE ONLY COPY. walker's help screen renders it, README.md quotes
it, and walkerd enforces it -- all three read `RULES` below, so the
documentation cannot drift away from the behaviour. That drift is exactly what
happened in Version 1, where the rules lived as scattered `if` statements and
the README described an older set of them.

THE RULES, AS THE PROJECT OWNER STATED THEM
-------------------------------------------
  * one C++ project at a time
  * a replay locks out projects and other replays -- the bag is publishing the
    wrapper surface, and a project publishing setpoints into that same surface
    gives two writers on one topic with no way to tell whose message won
  * a replay REQUIRES the simulation (decision D3); there is no
    simulation-less replay in V2
  * recording is never locked out: during a project, during a replay, during
    nothing at all. One recorder at a time is the only limit
  * QGC may run alongside anything, because the MAVLink guard makes it
    read-only whenever something else holds the flight lock (decision D4)
"""

from __future__ import annotations

from dataclasses import dataclass

# Units that, while running, hold the right to command the aircraft. walkerd
# locks QGC's command traffic for as long as any of these is up.
FLIGHT_LOCK_HOLDERS = ("project", "replay")


@dataclass(frozen=True)
class Rule:
    """`want` may not start while `blocker` runs. `hint` names the way out."""
    want: str
    blocker: str
    reason: str
    hint: str


RULES: tuple[Rule, ...] = (
    Rule("project", "project",
         "one C++ project at a time",
         "stop the running project first (p)"),
    Rule("project", "replay",
         "a replay owns the wrapper surface; a project would be a second writer "
         "on the same command topics",
         "stop the replay first (R)"),
    Rule("replay", "project",
         "a project is steering the aircraft; a bag would fight it for the same "
         "command topics",
         "stop the project first (p)"),
    Rule("replay", "replay",
         "one replay at a time",
         "stop the running replay first (R)"),
    Rule("record", "record",
         "one recorder at a time; two would write two bags of one flight",
         "stop the running recording first (r)"),
)

# What must already be running before a unit may start.
REQUIRES: dict[str, tuple[str, ...]] = {
    "project": ("sim", "bridge"),
    "qgc": ("sim",),
    # D3: every replay needs a live simulation, including a telemetry-only bag.
    "replay": ("sim", "bridge"),
    "bridge": ("sim",),
    # rviz and record deliberately require nothing: watching an empty graph or
    # recording one is useless but harmless, and refusing it would be a
    # surprise with no upside.
}

# Warnings, not refusals: allowed, but the operator should know.
ADVISORIES: dict[tuple[str, str], str] = {
    ("qgc", "project"):
        "QGC is now an OBSERVER: the MAVLink guard is dropping its commands "
        "while the project holds the flight lock",
    ("qgc", "replay"):
        "QGC is now an OBSERVER: the MAVLink guard is dropping its commands "
        "while the replay holds the flight lock",
    ("project", "qgc"):
        "QGC is open; it has been switched to observer for this project",
    ("replay", "qgc"):
        "QGC is open; it has been switched to observer for this replay",
}


@dataclass
class Verdict:
    allowed: bool
    reason: str = ""
    held_by: str = ""
    hint: str = ""
    advisories: tuple[str, ...] = ()


def check(want: str, running: set[str]) -> Verdict:
    """May `want` start, given the set of units currently running?"""
    for rule in RULES:
        if rule.want == want and rule.blocker in running:
            return Verdict(False, rule.reason, rule.blocker, rule.hint)

    missing = [u for u in REQUIRES.get(want, ()) if u not in running]
    if missing:
        first = missing[0]
        return Verdict(
            False,
            f"{want} needs {' and '.join(missing)}",
            first,
            {"sim": "start the simulation first (s)",
             "bridge": "the bridge starts with the simulation; check its unit"}
            .get(first, f"start {first} first"))

    notes = tuple(msg for (w, r), msg in ADVISORIES.items()
                  if w == want and r in running)
    return Verdict(True, advisories=notes)


def flight_lock_holder(running: set[str]) -> str | None:
    """Which unit, if any, currently owns the right to command the aircraft."""
    for name in FLIGHT_LOCK_HOLDERS:
        if name in running:
            return name
    return None


def as_table() -> list[list[str]]:
    """The matrix, rendered for walker's help screen and for README.md."""
    units = ["sim", "project", "replay", "record", "qgc", "rviz"]
    rows = [["start \\ running"] + units]
    for want in units:
        row = [want]
        for other in units:
            if want == other and want not in {r.want for r in RULES if r.want == r.blocker}:
                row.append("--")
                continue
            v = check(want, {other} | set(REQUIRES.get(want, ())))
            if not v.allowed and v.held_by == other:
                row.append("BLOCKED")
            elif any(other == r for (w, r) in ADVISORIES if w == want):
                row.append("warn")
            else:
                row.append("ok")
        rows.append(row)
    return rows


# ═══════════════════════════════════════════════════════════════════════════
# WHEN A SETTING MAY BE CHANGED
# ═══════════════════════════════════════════════════════════════════════════
#
# The rules above say what may RUN alongside what. These say what may be
# CHANGED, and when. They are a different question and they were missing: a
# setting that is read once, when the simulation is composed, can be edited
# freely while it runs and will appear to have been accepted while changing
# nothing at all.
#
# That is the worst kind of failure this project keeps finding: the interface
# says yes, and the system does not change. So each setting declares when it is
# editable, walker greys out the key when it is not, and walkerd refuses the
# change with the reason rather than accepting it quietly.
#
# THREE KINDS
#
#   COMPOSE TIME   read by tools/compose_sim.py when the simulation starts, so
#                  changing it under a running simulation is meaningless until
#                  the next start. Editable only while `sim` is stopped.
#
#   LIVE           takes effect immediately on a running simulation, and means
#                  nothing without one. Editable only while `sim` runs.
#
#   ANYTIME        safe either way.

ANYTIME = "anytime"
SIM_STOPPED = "sim_stopped"
SIM_RUNNING = "sim_running"


@dataclass(frozen=True)
class SettingRule:
    name: str
    when: str
    what: str        # what the setting does, for the UI
    reason: str      # why it is gated, in the operator's terms


SETTINGS: tuple[SettingRule, ...] = (
    SettingRule(
        "world", SIM_STOPPED, "which world to fly in",
        "the world is composed and symlinked into PX4's path when the "
        "simulation starts; changing it now would change nothing until it is "
        "restarted"),
    SettingRule(
        "drone", SIM_STOPPED, "which aircraft to fly",
        "the airframe file and the model are generated from the drone's "
        "manifest at start, and PX4 reads the airframe once, at boot"),
    SettingRule(
        "gui", SIM_STOPPED, "open a Gazebo window",
        "the Gazebo client is started alongside the server; a running "
        "simulation already has a window or does not"),
    SettingRule(
        "qgc_video", SIM_STOPPED, "load the video plugin for QGC",
        "GstCameraSystem is written into the gz server config at compose "
        "time, and loading it later is not possible. It also holds camera "
        "subscriptions open, which is why it is not simply always on"),
    SettingRule(
        # The headline capability of V2: proved in SPIKE-1, where a camera
        # group switched on mid-flight took CPU from 18.8% to 106% and back.
        "cameras", ANYTIME, "which lenses render",
        "cameras are subscription-driven, so any lens can be switched during a "
        "flight; and with the simulation down the choice is recorded and "
        "applied the moment it starts"),
    SettingRule(
        "lens", SIM_RUNNING, "which payload lens feeds the stream",
        "there is nothing to switch between until the cameras exist"),
    SettingRule(
        "tier", SIM_RUNNING, "photo, 4K, FHD or preview",
        "there is nothing to switch between until the cameras exist"),
)

_BY_NAME = {r.name: r for r in SETTINGS}


def can_edit(setting: str, running: set[str]) -> Verdict:
    """May `setting` be changed right now?"""
    rule = _BY_NAME.get(setting)
    if rule is None:
        # Unknown settings are allowed: refusing something this table has not
        # heard of would make adding one a two-file change.
        return Verdict(True)

    sim_up = "sim" in running
    if rule.when == SIM_STOPPED and sim_up:
        return Verdict(False, rule.reason, "sim",
                       "stop the simulation first (s), then change it")
    if rule.when == SIM_RUNNING and not sim_up:
        return Verdict(False, rule.reason, "",
                       "start the simulation first (s)")
    return Verdict(True)


def editable(running: set[str]) -> dict[str, bool]:
    """{setting: may be changed now} -- what walker greys out."""
    return {r.name: can_edit(r.name, running).allowed for r in SETTINGS}
