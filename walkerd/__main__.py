#!/usr/bin/env python3
"""
walkerd — the supervisor, inside the container.

It owns every process the simulation is made of, enforces what may run
alongside what, and answers walker over a unix socket. Nothing else starts or
stops a unit; that single ownership is what makes the constraint rules real
rather than advisory.

    walkerd                 run the supervisor (walker starts it this way)
    walkerd status          one-shot status of a running walkerd, as text
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import constraints                      # noqa: E402
import probes                           # noqa: E402
import bags as bagreg                   # noqa: E402
import cameras as camreg                # noqa: E402
import projects                         # noqa: E402
from mavguard import MavGuard           # noqa: E402
from rosnode import RosWatcher          # noqa: E402
from server import Server               # noqa: E402
from units import (FAILED, LogRule, RUNNING, STARTING, Unit, UnitSpec)  # noqa: E402

# project.conf spells the setpoint frame the way a mission author thinks about
# it; the bridge names the same routes slightly differently.
_SETPOINT_ALIASES = {
    "velocity": "velocity", "enu_velocity": "velocity",
    "position": "position", "enu_position": "position",
    "body": "body", "body_velocity": "body", "flu": "body",
    "rc": "rc",
}

HOME = os.path.expanduser("~")
WS = os.path.join(HOME, "ws")
TOOLS = os.path.join(WS, "src", "tools")
BRIDGE = os.path.join(WS, "src", "bridge")
SIMSUPPORT = os.path.join(WS, "src", "simsupport")

# Which bridge routes carry each camera group onto the wrapper surface.
# Enabling a route makes the bridge SUBSCRIBE, which makes ros_gz_bridge
# subscribe (lazy:true), which makes Gazebo render the sensor (always_on=false).
# That chain is the whole reason a camera group can be switched mid-flight --
# see SPIKE-1.
# Which bridge route carries which INDIVIDUAL camera onto the wrapper surface.
# Taken from the routes' own sim_topic in bridge/simty/registry.py; if a route
# is repointed there, change it here too.
VIDEO_ROUTE_TOPICS = {
    "stereo_left_out":  "/drone/perception/front/left/image_raw",
    "stereo_right_out": "/drone/perception/front/right/image_raw",
}

CAMERA_ROUTES = {
    "fisheye": ("stereo_left_out", "stereo_right_out"),
    "payload": ("main_camera_out",),
}
RUNTIME = os.path.join(HOME, "gz_runtime")
SOCKET = os.environ.get("WALKERD_SOCKET", "/run/walker/walkerd.sock")


# ── readiness probes, per unit ───────────────────────────────────────────────
# Each returns True when the unit is USABLE, not merely alive. The difference
# matters: "Gazebo is running but has not finished loading the world" and
# "Gazebo is running" look identical to a process check and completely
# different to anything that then tries to spawn a model into it.

# Set once the Daemon exists; the probe closures read it.
WATCH: RosWatcher | None = None


def _sim_ready() -> bool:
    """
    Usable, not merely alive.

    Both halves matter and they fail differently: a missing PROCESS means
    something crashed or never started, while processes-up-but-no-telemetry
    means a link in the chain is not forwarding -- most often the XRCE agent.
    Reporting them as one thing is what makes "the simulation won't start"
    undiagnosable.
    """
    p = probes.process_links()
    if not (p["gazebo"] and p["px4"] and p["xrce_agent"] and p["ros_gz_bridge"]):
        return False
    # Read the persistent watcher's counters -- no new participant, no
    # discovery delay, no repeated rclpy init. See rosnode.py.
    return bool(WATCH and WATCH.ready(("clock", "imu")))


def _bridge_ready() -> bool:
    """
    The bridge is ready when the WRAPPER SURFACE is live, not when its process
    is. Those are far apart in time: the process starts in under a second and
    then spends several building ~46 routes and 56 service servers, and a
    project that starts in between finds half a surface.

    flight_status is the right thing to wait on because it is what every
    mission's first step blocks on.
    """
    return bool(WATCH and WATCH.ready(("w_status",)))


class Daemon:
    def __init__(self) -> None:
        global WATCH
        self.watch = RosWatcher()
        WATCH = self.watch
        # Always in PX4's MAVLink path; the lock only changes what it forwards.
        self.guard = MavGuard()
        self.units: dict[str, Unit] = {}
        self.server = Server(SOCKET, self.handle)
        self.selection = {"drone": "m4e", "world": "powerline",
                          "cameras": "none", "qgc_video": False,
                          # A Gazebo window by default: the usual reason to run
                          # a simulation is to watch it. CI and remote shells
                          # turn it off, and start_sim.sh falls back to
                          # headless by itself when DISPLAY does not answer.
                          "gui": True}
        # Topics of the cameras currently switched on. The set, not a profile:
        # every lens is individually controllable and a profile is only a
        # shortcut for a particular set.
        self.enabled_cameras: set[str] = set()
        self._define_units()

    # ── the unit table ───────────────────────────────────────────────────────

    def _define_units(self) -> None:
        def add(spec: UnitSpec) -> None:
            self.units[spec.name] = Unit(spec, self.server.broadcast)

        add(UnitSpec(
            name="sim",
            argv=["/bin/bash", "-lc", os.path.join(TOOLS, "start_sim.sh")],
            cwd=HOME,
            ready=_sim_ready,
            ready_timeout=180.0,
            # PX4 is not a program that stops politely on SIGINT from a script;
            # SIGTERM to the whole group is what actually ends the simulation.
            stop_signal=signal.SIGTERM,
            stop_timeout=25.0,
            description="Gazebo + PX4 SITL + XRCE agent + ros_gz_bridge",
            log_rules=[
                LogRule(r"^\[sim\]", "info"),
                LogRule(r"SYS_AUTOSTART|gz_bridge.*model:", "info"),
                LogRule(r"Preflight Fail|ERROR|Unknown model", "warn"),
                LogRule(r"\[commander\].*(Takeoff|Landing|Armed|Disarmed)", "info"),
            ],
        ))

        add(UnitSpec(
            name="bridge",
            argv=["python3", "-B", os.path.join(BRIDGE, "ROS_Bridge_Simty.py")],
            cwd=BRIDGE,
            ready=_bridge_ready,
            ready_timeout=120.0,
            # SIGINT: the bridge tears its endpoints down and says so. Killed,
            # it leaves the wrapper names on the graph for a few seconds and
            # the next start races its own corpse.
            stop_signal=signal.SIGINT,
            stop_timeout=20.0,
            description="PSDK wrapper surface: 41 telemetry + 5 command topics, 56 services",
            log_rules=[
                LogRule(r"^\[simty\]!", "warn"),
                LogRule(r"serving /wrapper|shutting down", "info"),
                LogRule(r"surface:|routes|services", "info"),
            ],
        ))

        # The project unit's argv is rewritten each time one is started -- see
        # op_start. It is declared here with a placeholder so the unit exists
        # in the table (and on the dashboard) before anything has been chosen.
        add(UnitSpec(
            name="project",
            argv=["/bin/true"],
            cwd=projects.PROJECTS,
            ready=None,          # a mission is "ready" as soon as it is running
            stop_signal=signal.SIGINT,
            stop_timeout=15.0,
            description="one C++ mission, talking only to the wrapper surface",
            log_rules=[
                # A mission's step transitions are the whole story, and
                # demo_orbit_mission's own logging convention is what these
                # match: "[STEP 12s] status=... mode=... height=..."
                LogRule(r"^\[[A-Z_]+ ", "info"),
                LogRule(r"telemetry is flowing|reached|finished|landed|starting the circle",
                        "info"),
                LogRule(r"ok$|: ok", "info"),
                LogRule(r"refused|not available|ERROR|could not|never switched", "warn"),
            ],
        ))

        add(UnitSpec(
            name="cameras",
            argv=["/bin/bash", "-c",
                  "set +u; source /opt/ros/jazzy/setup.bash; "
                  f"source {WS}/install/setup.bash 2>/dev/null; "
                  f"exec python3 -u {SIMSUPPORT}/camera_manager.py"],
            cwd=SIMSUPPORT,
            ready=None,
            stop_signal=signal.SIGINT,
            stop_timeout=10.0,
            description="holds one subscription per switched-on camera, which is "
                        "what makes Gazebo render it",
            log_rules=[LogRule(r"lens|mode|zoom|active", "info")],
        ))

        add(UnitSpec(
            name="rviz",
            argv=["/bin/bash", "-c",
                  "set +u; source /opt/ros/jazzy/setup.bash; "
                  f"source {WS}/install/setup.bash 2>/dev/null; "
                  f"exec rviz2 -d {WS}/config/rviz/flight.rviz"],
            cwd=HOME,
            ready=None,
            stop_signal=signal.SIGTERM,
            stop_timeout=12.0,
            description="RViz: TF, odometry, obstacle markers and any enabled camera",
            log_rules=[LogRule(r"error|Error|failed", "warn")],
        ))

        add(UnitSpec(
            name="qgc",
            argv=["/bin/bash", "-c",
                  "cd $HOME && exec ./QGroundControl-x86_64.AppImage "
                  "--appimage-extract-and-run"],
            cwd=HOME,
            env={"APPIMAGE_EXTRACT_AND_RUN": "1"},
            ready=None,
            stop_signal=signal.SIGTERM,
            stop_timeout=15.0,
            description="QGroundControl on MAVLink UDP 14550, behind the guard",
            log_rules=[LogRule(r"error|Error|failed|Connected", "info")],
        ))

        add(UnitSpec(
            name="record",
            argv=["/bin/true"],          # rewritten per recording, see op_start
            cwd=HOME,
            ready=None,
            # SIGINT, NOT SIGTERM. rosbag2 finalises the bag on Ctrl-C: it
            # writes metadata.yaml and closes the mcap. A killed recorder
            # leaves a directory `ros2 bag info` cannot read and `ros2 bag
            # play` will not open -- the whole flight, lost at the last step.
            stop_signal=signal.SIGINT,
            stop_timeout=30.0,
            description="ros2 bag record, writing straight into bags/",
            log_rules=[
                LogRule(r"^===", "info"),
                LogRule(r"Recording|Listening|error|Error", "info"),
            ],
        ))

        add(UnitSpec(
            name="replay",
            argv=["/bin/true"],          # rewritten per replay
            cwd=HOME,
            ready=None,
            stop_signal=signal.SIGINT,
            stop_timeout=15.0,
            description="ros2 bag play onto the wrapper surface",
            log_rules=[
                LogRule(r"^===", "info"),
                LogRule(r"error|Error|failed", "warn"),
            ],
        ))

    # ── state ────────────────────────────────────────────────────────────────

    # ── the RViz panel ───────────────────────────────────────────────────────

    def panel_state(self) -> dict:
        """
        What the RViz panel draws, digested so the panel does no work.

        Deliberately small and flat. Every value here is something a person
        glances at while watching the aircraft; anything needing a scroll bar
        belongs in walker, not in a dock inside RViz.
        """
        rates = self.watch.rates()
        units = {n: u.state for n, u in self.units.items()}
        return {
            "t": time.time(),
            "drone": self.selection.get("drone", "?"),
            "world": self.selection.get("world", "?"),
            "cameras": self.selection.get("cameras", "none"),
            "units": units,
            "flight_lock": constraints.flight_lock_holder(self.running()) or "",
            "guard_locked": self.guard.locked,
            "telemetry": round(rates.get("w_status", 0.0), 1),
            "height": round(self.watch.last_height, 2),
            "armed": self.watch.last_armed,
            "mode": self.watch.last_mode,
        }

    def on_panel_command(self, verb: str) -> None:
        """
        Act on a button in the RViz panel.

        The panel owns no service clients and no policy: it sends a verb and
        walkerd does what walker's own keys would do. One place decides what
        "land" means.
        """
        verb = (verb or "").strip().lower()
        self.server.broadcast({"ev": "log", "unit": "rviz", "level": "info",
                               "t": time.time(), "text": f"panel: {verb}"})
        if verb in ("takeoff", "land", "estop"):
            service = {"takeoff": "takeoff", "land": "land",
                       "estop": "start_force_landing"}[verb]
            self.watch.call_wrapper_service(service)
        elif verb.startswith("cameras "):
            self.op_cameras({"profile": verb.split(None, 1)[1].strip()})

    def _sync_flight_lock(self) -> None:
        """
        Keep the MAVLink guard in step with who holds the flight lock.

        Called after every unit transition rather than only at start and stop,
        so a mission that EXITS ON ITS OWN releases QGC as surely as one that
        is stopped from the dashboard. A lock that is only released on the
        tidy path is a lock that eventually sticks.
        """
        holder = constraints.flight_lock_holder(self.running())
        was = self.guard.locked
        self.guard.set_locked(holder is not None, holder or "")
        if bool(holder) != was:
            self.server.broadcast({
                "ev": "log", "unit": "qgc", "level": "warn" if holder else "info",
                "t": time.time(),
                "text": (f"QGC is now an OBSERVER: {holder} holds the flight lock"
                         if holder else
                         "QGC has full control again: nothing holds the flight lock")})

    def running(self) -> set[str]:
        return {n for n, u in self.units.items() if u.state in (RUNNING, STARTING)}

    def snapshot(self) -> dict:
        return {
            "units": [u.snapshot() for u in self.units.values()],
            "selection": dict(self.selection),
            "flight_lock": constraints.flight_lock_holder(self.running()),
            # walker greys out keys from THIS, rather than re-deriving the
            # rules on the host: one table, enforced and displayed from the
            # same place, so what the UI offers and what walkerd accepts can
            # never disagree.
            "editable": constraints.editable(self.running()),
            "guard": self.guard.status(),
            "composed": os.path.isfile(os.path.join(RUNTIME, "compose.json")),
        }

    # ── operations ───────────────────────────────────────────────────────────

    def op_start(self, req: dict) -> dict:
        name = req.get("unit", "")
        unit = self.units.get(name)
        if unit is None:
            # Naming the unit alone reads as if the unit were the reason for a
            # refusal, which is how a not-yet-implemented feature looks exactly
            # like a constraint violation on the dashboard.
            return {"ok": False, "error": "no such unit",
                    "reason": f"'{name}' is not a unit this walkerd knows",
                    "hint": f"units: {', '.join(self.units)}"}

        verdict = constraints.check(name, self.running())
        if not verdict.allowed:
            return {"ok": False, "error": "refused", "reason": verdict.reason,
                    "held_by": verdict.held_by, "hint": verdict.hint}

        if name == "record":
            err = self._prepare_record(req.get("args", {}) or {})
            if err:
                return {"ok": False, "error": "record", "reason": err}

        if name == "replay":
            err = self._prepare_replay(req.get("args", {}) or {})
            if err:
                return {"ok": False, "error": "replay", "reason": err}

        if name == "project":
            # PREPARING A PROJECT CAN TAKE MINUTES, so it does not happen
            # inside the request. A stale mission is rebuilt first (57 s is
            # typical), then the bridge is put into project mode, then the unit
            # starts -- and a request that blocked for all of that would time
            # out on the client, which is exactly what happened the first time.
            #
            # The reply says "accepted" and everything after it is reported the
            # way every other unit transition is: as events. The unit goes
            # STARTING -> RUNNING, or STARTING -> FAILED with the reason in its
            # detail, and walker's dashboard shows either without waiting.
            args = req.get("args", {}) or {}
            unit = self.units["project"]

            def prepare_and_start():
                err = self._prepare_project(args)
                if err:
                    unit.mark(FAILED, err)
                    return
                unit.start()

            unit.mark(STARTING, "building and starting — see its terminal")
            threading.Thread(target=prepare_and_start, daemon=True,
                             name="prepare-project").start()
            return {"ok": True, "accepted": True,
                    "advisories": list(verdict.advisories)}

        # The simulation is COMPOSED before it starts: which drone, which world
        # and which camera profile are decisions made on walker's screens, and
        # compose_sim.py turns them into the files PX4 and Gazebo will read.
        if name == "sim":
            # The window is a per-run choice, so it is passed at start rather
            # than baked into the unit.
            self.units["sim"].spec.env["SIM_HEADLESS"] = (
                "0" if self.selection.get("gui", True) else "1")
            err = self.compose()
            if err:
                return {"ok": False, "error": "compose failed", "reason": err}

        unit.start()
        return {"ok": True, "advisories": list(verdict.advisories)}

    def _prepare_record(self, args: dict) -> str:
        """Point the recorder at a named bag. Returns "" or a reason."""
        name = (args.get("name") or "").strip()
        problem = bagreg.valid_name(name)
        if problem:
            return problem
        scope = args.get("scope", "wrapper")
        if scope not in bagreg.SCOPES:
            return f"unknown scope '{scope}'"

        # The wrapper service names, for --services. Asked of the live graph
        # rather than hard-coded, so a service added to the bridge is recorded
        # without anyone remembering to update a list here.
        services: list[str] = []
        if scope != "all":
            services = self.watch.wrapper_service_names()
            if not services:
                self.server.broadcast({
                    "ev": "log", "unit": "record", "level": "warn",
                    "t": time.time(),
                    "text": "no wrapper services on the graph - service CALLS "
                            "will not be recorded, only topics"})

        unit = self.units["record"]
        unit.spec.argv = bagreg.record_argv(name, scope, services)
        unit.spec.description = f"recording bags/{name} ({scope})"
        self.selection["recording"] = name
        return ""

    def _prepare_replay(self, args: dict) -> str:
        """Point the player at a bag, and get the bridge out of its way."""
        name = (args.get("bag") or "").strip()
        bag = bagreg.detail(name)
        if not bag:
            return f"no bag called '{name}' under bags/"
        if bag.get("error"):
            return bag["error"]

        # THE BAG OWNS THE WRAPPER SURFACE WHILE IT PLAYS.
        #
        # Without this the bridge keeps publishing its own telemetry onto the
        # same topics the bag is replaying, and a subscriber gets the two
        # interleaved with nothing saying which is which. Replay mode switches
        # the bridge's own routes off and restores them afterwards; the
        # services stay served, so a caller still gets an answer.
        if not self.watch.send_control("replay value=on"):
            return ("the bridge did not accept replay mode (is it running?) - "
                    "without it the bridge would publish over the recording")
        time.sleep(6.0)    # one endpoint mutation per tick; replay touches all

        unit = self.units["replay"]
        unit.spec.argv = bagreg.play_argv(bag["path"],
                                          float(args.get("rate", 1.0)),
                                          bool(args.get("loop", False)))
        kind = "fly-back" if bag["can_fly"] else "telemetry-only"
        unit.spec.description = f"replaying {name} ({kind})"
        self.selection["replaying"] = name
        self.server.broadcast({
            "ev": "log", "unit": "replay", "level": "info", "t": time.time(),
            "text": (f"{name}: {kind}; bridge routes off so the bag owns the "
                     f"surface")})
        return ""

    def _prepare_project(self, args: dict) -> str:
        """
        Point the project unit at a mission, building it if need be.
        Returns "" or an error meant for a human.
        """
        name = args.get("package", "")
        if not name:
            return "no project named"
        st = projects.status(name)
        if st["state"] in ("missing", "broken"):
            return f"{name}: {st['detail']}"

        # Build when it has never been built, or when the source is newer than
        # the binary. Running a stale binary is the failure that wastes the
        # most time: the mission runs, behaves like the old code, and nothing
        # says why.
        # THE BUILD RUNS INSIDE THE UNIT, not here.
        #
        # Building in walkerd sent compiler output to the socket as log events,
        # so the mission's own terminal opened only after the build had
        # finished and showed none of it. A build failure was then a one-line
        # "failed to build; see the log" with the actual error somewhere else.
        #
        # Putting the build in the unit's command means its terminal carries
        # the whole story in order: what is being built, the compiler's output,
        # and then the mission's own logs -- which is what you want open when a
        # mission misbehaves.
        # WHAT "REBUILD" MEANS, from the two ways a mission is started.
        #
        #   enter          rebuild always. You are iterating on the code, and
        #                  the alternative is running yesterday's binary because
        #                  a timestamp comparison disagreed with you.
        #   shift+enter    reuse what is built. Skips ~60 s when you are
        #                  re-flying an unchanged mission with new parameters.
        #
        # Either way a project with NO binary is built, because there is
        # nothing else to run.
        if args.get("rebuild", True):
            needs_build = True
        else:
            needs_build = st["state"] in ("unbuilt", "missing")

        unit = self.units["project"]
        unit.spec.argv = projects.run_argv(name, args.get("params"),
                                           build=needs_build,
                                           why=st.get("detail", ""))
        unit.spec.description = f"{name} — wrapper surface only"
        self.selection["project"] = name

        # NO PROJECT MODE SWITCHING. The bridge is ready for missions the
        # moment it is up.
        #
        # V1 needed a per-session "project testing mode" because the same
        # bridge also served a real Manifold: it enabled the telemetry routes,
        # set the services to SERVE, picked one setpoint route and turned
        # auto_offboard on. In V2 the surface policy already enables every
        # route and serves every service (finding F-1), and auto_offboard is
        # the bridge's default (settings.py) -- so the whole dance reduced to a
        # four-second pause that could only fail.
        #
        # Starting a project is therefore exactly: build it if it needs
        # building, then run it.
        return ""

    def op_cameras(self, args: dict) -> dict:
        """
        Switch cameras -- individually, or by profile shortcut. Live.

        Accepts any of:
            {"profile": "fisheye"}            the four shortcuts
            {"on": [topic, ...]}              switch these on
            {"off": [topic, ...]}             switch these off
            {"toggle": topic}                 flip one
            {"active": topic}                 which payload lens feeds
                                              the wrapper's main_camera_stream

        Nothing restarts. A camera goes on by making the manager subscribe and
        off by making it stop; Gazebo follows within a sensor period.
        """
        verdict = constraints.can_edit("cameras", self.running())
        if not verdict.allowed:
            return {"ok": False, "error": "locked", "reason": verdict.reason,
                    "hint": verdict.hint}
        # The gate above has already refused this if the simulation is
        # running: the camera set is chosen before a run and fixed for its
        # duration, so a run's rendering cost cannot change underneath a
        # mission that is being measured.
        #
        # `live` stays, because a set chosen while stopped still has to be
        # applied the moment the simulation comes up.
        live = "sim" in self.running()
        drone = self.selection.get("drone", "")
        known = {c["topic"] for c in camreg.catalogue(drone)}
        before = set(self.enabled_cameras)

        if "profile" in args:
            profile = args["profile"]
            if profile not in ("none", "fisheye", "payload", "all"):
                return {"ok": False, "error": "bad profile", "reason": profile}
            self.enabled_cameras = set(camreg.profile_topics(profile, drone))
            self.selection["cameras"] = profile
        else:
            for topic in args.get("on", []):
                if topic in known:
                    self.enabled_cameras.add(topic)
            for topic in args.get("off", []):
                self.enabled_cameras.discard(topic)
            if args.get("toggle"):
                topic = args["toggle"]
                if topic in self.enabled_cameras:
                    self.enabled_cameras.discard(topic)
                elif topic in known:
                    self.enabled_cameras.add(topic)
            # A hand-picked set is no longer one of the four shortcuts, and
            # saying "fisheye" when three lenses are on would be a lie.
            self.selection["cameras"] = self._profile_name()

        if args.get("active"):
            self.selection["active_lens"] = args["active"]
            # Something has to be rendering for it to feed anything.
            self.enabled_cameras.add(args["active"])

        if live:
            self._apply_cameras()

        added = sorted(self.enabled_cameras - before)
        removed = sorted(before - self.enabled_cameras)
        self.server.broadcast({"ev": "selection", **self.selection})
        self.server.broadcast({
            "ev": "log", "unit": "cameras", "level": "info", "t": time.time(),
            "text": (f"{len(self.enabled_cameras)} camera(s) on"
                     + (f"; +{len(added)}" if added else "")
                     + (f"; -{len(removed)}" if removed else ""))})
        return {"ok": True, "enabled": sorted(self.enabled_cameras),
                "added": added, "removed": removed,
                "profile": self.selection["cameras"]}

    def _report_recording(self) -> None:
        """
        Say what was actually captured, once the recorder has closed the bag.

        rosbag2 writes metadata.yaml when it finalises, so this runs after the
        unit has stopped -- and if the file is missing, the recording is not
        readable and saying so now is far better than discovering it on the
        Bags screen a week later.
        """
        name = self.selection.get("recording", "")
        if not name:
            return
        time.sleep(1.5)                       # let the finaliser land
        bag = bagreg.detail(name)
        if not bag or bag.get("error"):
            self.server.broadcast({
                "ev": "log", "unit": "record", "level": "warn", "t": time.time(),
                "text": f"bags/{name} has no metadata.yaml - it may not be "
                        f"readable. Try: ros2 bag reindex -s mcap bags/{name}"})
            return
        mins, secs = divmod(int(bag["duration_s"]), 60)
        self.server.broadcast({
            "ev": "log", "unit": "record", "level": "info", "t": time.time(),
            "text": (f"bags/{name}: {mins}:{secs:02d}, {bag['messages']:,} msgs, "
                     f"{bag['n_topics']} topics, {bag['n_services']} services, "
                     f"{bag['size_bytes']/1e6:.1f} MB - "
                     f"{'can fly back' if bag['can_fly'] else 'telemetry only'}")})

    def _apply_cameras(self) -> None:
        """
        Make the running simulation render exactly the chosen set.

        Called when a camera is switched AND when the simulation reaches
        running, so a set chosen while it was down is honoured on start rather
        than silently forgotten -- which is what "enable cameras before running"
        has to mean to be worth anything.
        """
        unit = self.units["cameras"]
        if self.enabled_cameras and unit.state not in (RUNNING, STARTING):
            unit.start()
        elif not self.enabled_cameras and unit.alive:
            threading.Thread(target=unit.stop, daemon=True).start()

        # Published twice on purpose: once now, and once shortly after, because
        # the manager may only just have been started above and a latched
        # sample is the belt to that brace.
        self.watch.publish_cameras(list(self.enabled_cameras),
                                   self.selection.get("active_lens", ""))
        threading.Timer(3.0, lambda: self.watch.publish_cameras(
            list(self.enabled_cameras),
            self.selection.get("active_lens", ""))).start()

        # EACH BRIDGE ROUTE FOLLOWS ITS OWN CAMERA, not a group flag.
        #
        # This was a real bug. Both stereo routes were driven from one
        # "any front fisheye is on" flag, but they subscribe to DIFFERENT
        # topics -- stereo_left_out to front/left, stereo_right_out to
        # front/right. Switching on front-left therefore enabled the route
        # that subscribes to front-RIGHT, which made Gazebo render a camera
        # walker was showing as off. And it could not be switched off again,
        # because the flag only went false once BOTH were off.
        #
        # The symptom was "a camera, once on, is always on".
        for key, topic in VIDEO_ROUTE_TOPICS.items():
            on = topic in self.enabled_cameras
            self.watch.send_control(
                f"enable route={key} value={1 if on else 0}")

        # main_camera_out is the exception: it reads the camera manager's
        # /drone/camera/active/image_raw rather than a lens directly, so it
        # follows "is any payload lens on" rather than one topic.
        payload_on = any("/camera/" in t for t in self.enabled_cameras)
        self.watch.send_control(
            f"enable route=main_camera_out value={1 if payload_on else 0}")

    def _profile_name(self) -> str:
        """Name the current set if it happens to be one of the shortcuts."""
        drone = self.selection.get("drone", "")
        for profile in ("none", "fisheye", "payload", "all"):
            if set(camreg.profile_topics(profile, drone)) == self.enabled_cameras:
                return profile
        return f"custom ({len(self.enabled_cameras)})"

    def op_stop(self, req: dict) -> dict:
        name = req.get("unit", "")
        unit = self.units.get(name)
        if unit is None:
            return {"ok": False, "error": "no such unit"}

        def go():
            unit.stop()
            # Hand the bridge its route table back. Leaving it in project mode
            # would leave the simulation quietly wide open -- auto_offboard on,
            # setpoint routes live -- for whatever runs next.
            if name == "replay":
                # Give the bridge its own routes back. Left in replay mode it
                # publishes nothing at all, and the next mission would sit
                # waiting for telemetry that no longer comes.
                self.watch.send_control("replay value=off", wait_for_bridge=2.0)
            if name == "record":
                self._report_recording()

        threading.Thread(target=go, daemon=True).start()
        return {"ok": True}

    def compose(self) -> str:
        """Run the composer for the current selection. Returns "" or an error."""
        s = self.selection
        # The camera label is recorded, not acted on: every sensor is composed
        # either way and which ones render is decided at run time. The ENABLED
        # SET is what matters, and it is applied when the simulation comes up
        # (see _apply_cameras).
        cmd = [sys.executable, os.path.join(TOOLS, "compose_sim.py"),
               "--drone", s["drone"], "--world", s["world"],
               "--cameras", str(s.get("cameras", "none"))]
        if s.get("qgc_video"):
            cmd.append("--qgc-video")
        cp = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        if cp.returncode != 0:
            return (cp.stderr or cp.stdout).strip()[-500:]
        for line in (cp.stdout or "").splitlines():
            self.server.broadcast({"ev": "log", "unit": "compose",
                                   "level": "info", "text": line.strip(),
                                   "t": time.time()})
        return ""

    def handle(self, req: dict) -> dict:
        op = req.get("op", "")
        if op == "ping":
            return {"ok": True, "t": time.time()}
        if op == "state":
            return {"ok": True, **self.snapshot()}
        if op == "start":
            return self.op_start(req)
        if op == "stop":
            return self.op_stop(req)
        if op == "select":
            args = req.get("args", {}) or {}
            running = self.running()
            # Check EVERY requested change before applying ANY of them: a
            # partially applied selection is worse than a refused one, because
            # the screen then shows a mixture of what you asked for and what
            # you had.
            for k in args:
                verdict = constraints.can_edit(k, running)
                if not verdict.allowed:
                    return {"ok": False, "error": "locked", "reason": verdict.reason,
                            "held_by": verdict.held_by, "hint": verdict.hint,
                            "setting": k}
            for k in ("drone", "world", "cameras", "qgc_video", "gui"):
                if k in args:
                    self.selection[k] = args[k]
            self.server.broadcast({"ev": "selection", **self.selection})
            return {"ok": True, "selection": dict(self.selection)}
        if op == "probe":
            # Instant: the watcher has been counting since walkerd started, so
            # this is a subtraction rather than a subscription.
            return {"ok": True,
                    "processes": probes.process_links(),
                    "links": self.watch.links(wrapper=req.get("wrapper", True)),
                    "watcher_error": self.watch.error}
        if op == "cameras":
            return self.op_cameras(req.get("args", {}) or {})
        if op == "camera_list":
            return {"ok": True,
                    "cameras": camreg.catalogue(self.selection.get("drone", "")),
                    "enabled": sorted(self.enabled_cameras),
                    "active": self.selection.get("active_lens", "")}
        if op == "bags":
            return {"ok": True, "bags": bagreg.listing(),
                    "scopes": bagreg.SCOPES}
        if op == "bag_detail":
            bag = bagreg.detail(req.get("bag", ""))
            return ({"ok": True, "bag": bag} if bag else
                    {"ok": False, "error": "no such bag",
                     "reason": req.get("bag", "")})
        if op == "check_name":
            return {"ok": True, "problem": bagreg.valid_name(req.get("name", ""))}
        if op == "projects":
            return {"ok": True, "projects": projects.listing()}
        if op == "build":
            name = req.get("package", "")
            ok, out = projects.build(name)
            return {"ok": ok, "output": out.splitlines()[-20:],
                    "status": projects.status(name)}
        if op == "constraints":
            return {"ok": True, "table": constraints.as_table(),
                    "rules": [{"want": r.want, "blocker": r.blocker,
                               "reason": r.reason, "hint": r.hint}
                              for r in constraints.RULES]}
        if op == "attach":
            # Output events already go to every client, so attaching is really
            # just an acknowledgement: attach.py filters by unit name. Keeping
            # it as an explicit op means walkerd can refuse a name that does
            # not exist, rather than leaving a terminal silently showing
            # nothing.
            if req.get("unit") not in self.units:
                return {"ok": False, "error": "no such unit",
                        "reason": f"'{req.get('unit')}' is not a unit",
                        "hint": f"units: {', '.join(self.units)}"}
            return {"ok": True, "attached": req.get("unit")}
        if op == "logs":
            unit = self.units.get(req.get("unit", ""))
            if unit is None:
                return {"ok": False, "error": "no such unit"}
            n = int(req.get("lines", 200))
            return {"ok": True, "lines": list(unit.ring)[-n:]}
        if op == "input":
            unit = self.units.get(req.get("unit", ""))
            if unit is None:
                return {"ok": False, "error": "no such unit"}
            unit.write(req.get("data", ""))
            return {"ok": True}
        if op == "resize":
            unit = self.units.get(req.get("unit", ""))
            if unit:
                unit.resize(int(req.get("rows", 50)), int(req.get("cols", 200)))
            return {"ok": True}
        if op == "shutdown":
            threading.Thread(target=self.shutdown, daemon=True).start()
            return {"ok": True}
        return {"ok": False, "error": "unknown op", "reason": op}

    # ── run ──────────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        # Stop units in reverse dependency order so nothing is left talking to
        # a graph that has gone away.
        for name in reversed(list(self.units)):
            u = self.units[name]
            if u.alive:
                u.stop()
        self.watch.stop()
        self.guard.stop()
        self.server.stop()
        os._exit(0)

    def run(self) -> int:
        self.watch.start()
        self.guard.start()
        self.server.start()

        # The guard must track the flight lock even when nothing calls
        # start/stop -- a mission that ends by itself is the common case.
        self.watch.on_panel_command = self.on_panel_command

        def lock_watch():
            tick = 0
            sim_was = bridge_was = ""
            while True:
                try:
                    self._sync_flight_lock()
                    # A camera set chosen while the simulation was down is
                    # applied the moment it comes up.
                    # Re-apply the camera set when EITHER the simulation or
                    # the bridge comes up.
                    #
                    # The bridge matters as much as the simulation: the route
                    # commands that put a camera's frames onto the wrapper
                    # surface go to the bridge, and one started after the
                    # simulation would never receive them. The symptom was a
                    # lens rendering happily while
                    # /wrapper/psdk_ros2/perception_stereo_left_stream stayed
                    # silent, which reads as a broken bridge and is not.
                    sim_now = self.units["sim"].state
                    bridge_now = self.units["bridge"].state
                    came_up = ((sim_now == RUNNING and sim_was != RUNNING) or
                               (bridge_now == RUNNING and bridge_was != RUNNING))
                    if came_up and self.enabled_cameras:
                        self._apply_cameras()
                        self.server.broadcast({
                            "ev": "log", "unit": "cameras", "level": "info",
                            "t": time.time(),
                            "text": f"applied {len(self.enabled_cameras)} "
                                    "camera(s) chosen before start"})
                    sim_was, bridge_was = sim_now, bridge_now
                    # 5 Hz to the panel: fast enough to feel live, slow enough
                    # that the panel never repaints faster than a person reads.
                    if tick % 1 == 0:
                        self.watch.publish_panel_state(self.panel_state())
                except Exception:
                    pass
                tick += 1
                time.sleep(0.2)
        threading.Thread(target=lock_watch, daemon=True, name="flightlock").start()
        print(f"walkerd: listening on {SOCKET}", flush=True)
        print(f"walkerd: units: {', '.join(self.units)}", flush=True)

        stop = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())
        while not stop.is_set():
            stop.wait(0.5)
        self.shutdown()
        return 0


def cmd_status() -> int:
    """Ask a running walkerd for its state, as text. Used by the `units` alias."""
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect(SOCKET)
        s.sendall(b'{"id":1,"op":"state"}\n')
        buf = b""
        while b"\n" not in buf:
            buf += s.recv(65536)
        data = json.loads(buf.split(b"\n")[0])
    except OSError as e:
        print(f"walkerd is not running ({e})", file=sys.stderr)
        return 1
    sel = data.get("selection", {})
    print(f"\n  selection: {sel.get('drone')} in {sel.get('world')}, "
          f"cameras={sel.get('cameras')}\n")
    for u in data.get("units", []):
        up = f"{u['uptime']:.0f}s" if u["uptime"] else ""
        print(f"  {u['state']:<9} {u['name']:<10} {up:<8} {u['detail'][:60]}")
    print()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(prog="walkerd")
    p.add_argument("command", nargs="?", default="run", choices=["run", "status"])
    a = p.parse_args()
    if a.command == "status":
        return cmd_status()
    return Daemon().run()


if __name__ == "__main__":
    sys.exit(main())
