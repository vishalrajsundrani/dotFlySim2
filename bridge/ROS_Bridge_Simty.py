#!/usr/bin/env python3
"""
ROS_Bridge_Simty.py — the PSDK wrapper surface, served from the simulation.

THIS PROCESS IS WHAT MAKES /wrapper/psdk_ros2 EXIST. Nothing else in the stack
translates between DJI's interface and PX4/Gazebo. With it running, a C++
mission written against a real Matrice 4E flies the simulation unchanged; with
it stopped, the wrapper surface is simply absent and every project waits
forever for telemetry that no one is producing.

WHAT CHANGED FROM VERSION 1
===========================
V1's entry point was an interactive curses console, and most of what it did was
about a DJI Manifold 3 that is not part of Version 2:

    scanning for the Manifold over multicast, starting its wrapper over SSH,
    faking one locally when none answered, escalating between those three,
    and a 2 000-line TUI to drive it all.

None of that applies to a simulation that runs entirely inside one container,
so it is gone -- about 3 500 lines, with no loss of simulated behaviour. What
is left is the part that was always the point: the route tables, the
converters, and the service handlers.

The console is gone too, and deliberately: in V2 walker is the operator
surface. This process is a UNIT, supervised by walkerd, which starts it with
the simulation and stops it with the simulation. It writes to stdout, walkerd
captures that, and walker shows the lines that matter. Everything the old
console could change live is reachable over /simty/control, which walkerd
drives.

    bridge_psdk                run it (walkerd does this)
    bridge_psdk --once-report  print one readiness report and exit

WHY THE IMPORTS ARE WHERE THEY ARE
==================================
Middleware selection has to happen before rclpy and any message typesupport are
dlopen'd. An earlier revision set RMW_IMPLEMENTATION after `import px4_msgs.msg`
at module scope had already bound typesupport, and the override was then
ignored or half-applied depending on the build. Hence: stdlib only, then the
environment, then ROS.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

DEFAULT_RMW = "rmw_fastrtps_cpp"


def configure_middleware() -> list:
    """
    Settle the DDS environment before ROS is imported. Returns notes to log
    once a logger exists.

    Policy: respect what the container already set, fill in what is missing,
    and WARN rather than silently override when the environment disagrees with
    the project's choice. Silently switching middleware under the operator is
    how an earlier revision ended up on a different vendor from the rest of the
    stack, with symptoms that looked like a QoS bug.
    """
    notes = []

    # Domain 0 and localhost-only discovery are the V2 defaults, set as real
    # ENV in the Dockerfile. They are not asserted here -- only reported --
    # because a deliberate override should work.
    if "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = "0"
        notes.append(("info", "ROS_DOMAIN_ID was unset; using 0"))

    current = os.environ.get("RMW_IMPLEMENTATION")
    if not current:
        os.environ["RMW_IMPLEMENTATION"] = DEFAULT_RMW
        notes.append(("info", f"RMW_IMPLEMENTATION was unset; using {DEFAULT_RMW}"))
    elif not current.startswith("rmw_fastrtps"):
        notes.append((
            "warn",
            f"RMW_IMPLEMENTATION is {current}, not Fast DDS. The rest of this "
            "stack (PX4's XRCE agent, ros_gz_bridge) is built against Fast DDS; "
            "mixing vendors is not supported by ROS 2 and shows up as endpoints "
            "that discover each other and then never exchange a message."))

    rng = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE")
    if rng and rng != "LOCALHOST":
        notes.append((
            "warn",
            f"ROS_AUTOMATIC_DISCOVERY_RANGE is {rng}, not LOCALHOST. V2 expects "
            "the whole graph to be confined to this container."))

    profiles = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if profiles and not os.path.isfile(profiles):
        notes.append((
            "warn",
            f"FASTRTPS_DEFAULT_PROFILES_FILE points at {profiles}, which does "
            "not exist; Fast DDS will fall back to its built-in transports."))
    return notes


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="bridge_psdk", description=__doc__)
    parser.add_argument("--once-report", action="store_true",
                        help="print one readiness report and exit")
    parser.add_argument("--settle", type=float, default=6.0,
                        help="seconds to let discovery settle before reporting")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    env_notes = configure_middleware()

    # --- ROS and package imports only after the environment is settled ---
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    from simty.node import SimtyBridge
    from simty.settings import DEFAULT_PROFILE_PATH, Settings
    from simty.state import BridgeState, INFO, WARN

    settings, profile_note = Settings.load(DEFAULT_PROFILE_PATH)
    settings.domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))

    state = BridgeState()
    for level, message in env_notes:
        state.log.add(WARN if level == "warn" else INFO, "env", message)
    state.log.add(INFO, "profile", profile_note)

    rclpy.init(args=None)
    bridge = SimtyBridge(settings, state)
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)

    # Everything the bridge says goes to stdout, where walkerd's pty pump is
    # reading. Printing from the log ring rather than wiring a second logger
    # keeps one source of truth for what the bridge has said.
    # LogRing exposes a monotonic `seq` and a `tail(n)`; tracking the sequence
    # is how we print each record exactly once without holding a second copy of
    # the ring. Capped at 200 per drain so a burst cannot monopolise the loop.
    seen = {"seq": 0}

    def drain_log() -> None:
        seq = state.log.seq
        new = seq - seen["seq"]
        if new <= 0:
            return
        for rec in state.log.tail(min(new, 200)):
            mark = "!" if getattr(rec, "level", INFO) >= WARN else " "
            print(f"[simty]{mark} {getattr(rec, 'source', '?')}: "
                  f"{getattr(rec, 'message', rec)}", flush=True)
        seen["seq"] = seq

    stop = threading.Event()

    def on_signal(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    spinner = threading.Thread(target=executor.spin, daemon=True, name="simty-spin")
    spinner.start()

    if args.once_report:
        time.sleep(args.settle)
        drain_log()
        report = bridge.report()
        print(f"[simty] surface: {report.surface.label}")
        print(f"[simty] {report.summary()}")
        for missing in report.expected_missing[:10]:
            print(f"[simty]! expected but absent: {missing}")
        stop.set()
    else:
        print("[simty] serving /wrapper/psdk_ros2 — "
              "this is what a C++ mission talks to", flush=True)

    while not stop.is_set():
        drain_log()
        stop.wait(0.5)

    drain_log()
    print("[simty] shutting down", flush=True)
    try:
        executor.shutdown(timeout_sec=5.0)
    except Exception:
        pass
    try:
        bridge.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
