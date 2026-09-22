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
from rosnode import RosWatcher          # noqa: E402
from server import Server               # noqa: E402
from units import (FAILED, LogRule, RUNNING, STARTING, Unit, UnitSpec)  # noqa: E402

HOME = os.path.expanduser("~")
WS = os.path.join(HOME, "ws")
TOOLS = os.path.join(WS, "src", "tools")
BRIDGE = os.path.join(WS, "src", "bridge")
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
        self.units: dict[str, Unit] = {}
        self.server = Server(SOCKET, self.handle)
        self.selection = {"drone": "m4e", "world": "powerline",
                          "cameras": "none", "qgc_video": False}
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

    # ── state ────────────────────────────────────────────────────────────────

    def running(self) -> set[str]:
        return {n for n, u in self.units.items() if u.state in (RUNNING, STARTING)}

    def snapshot(self) -> dict:
        return {
            "units": [u.snapshot() for u in self.units.values()],
            "selection": dict(self.selection),
            "flight_lock": constraints.flight_lock_holder(self.running()),
            "composed": os.path.isfile(os.path.join(RUNTIME, "compose.json")),
        }

    # ── operations ───────────────────────────────────────────────────────────

    def op_start(self, req: dict) -> dict:
        name = req.get("unit", "")
        unit = self.units.get(name)
        if unit is None:
            return {"ok": False, "error": "no such unit", "reason": name}

        verdict = constraints.check(name, self.running())
        if not verdict.allowed:
            return {"ok": False, "error": "refused", "reason": verdict.reason,
                    "held_by": verdict.held_by, "hint": verdict.hint}

        # The simulation is COMPOSED before it starts: which drone, which world
        # and which camera profile are decisions made on walker's screens, and
        # compose_sim.py turns them into the files PX4 and Gazebo will read.
        if name == "sim":
            err = self.compose()
            if err:
                return {"ok": False, "error": "compose failed", "reason": err}

        unit.start()
        return {"ok": True, "advisories": list(verdict.advisories)}

    def op_stop(self, req: dict) -> dict:
        unit = self.units.get(req.get("unit", ""))
        if unit is None:
            return {"ok": False, "error": "no such unit"}
        threading.Thread(target=unit.stop, daemon=True).start()
        return {"ok": True}

    def compose(self) -> str:
        """Run the composer for the current selection. Returns "" or an error."""
        s = self.selection
        cmd = [sys.executable, os.path.join(TOOLS, "compose_sim.py"),
               "--drone", s["drone"], "--world", s["world"],
               "--cameras", s["cameras"]]
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
            for k in ("drone", "world", "cameras", "qgc_video"):
                if k in req.get("args", {}):
                    self.selection[k] = req["args"][k]
            self.server.broadcast({"ev": "selection", **self.selection})
            return {"ok": True, "selection": dict(self.selection)}
        if op == "probe":
            # Instant: the watcher has been counting since walkerd started, so
            # this is a subtraction rather than a subscription.
            return {"ok": True,
                    "processes": probes.process_links(),
                    "links": self.watch.links(wrapper=req.get("wrapper", True)),
                    "watcher_error": self.watch.error}
        if op == "constraints":
            return {"ok": True, "table": constraints.as_table(),
                    "rules": [{"want": r.want, "blocker": r.blocker,
                               "reason": r.reason, "hint": r.hint}
                              for r in constraints.RULES]}
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
        self.server.stop()
        os._exit(0)

    def run(self) -> int:
        self.watch.start()
        self.server.start()
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
