#!/usr/bin/env python3
"""
Compare the LIVE wrapper surface against the PDF.

This is M4's acceptance test, kept as a tool because it is the check you want
again after every change to registry.py: it answers "does this simulation still
present the same interface as a real Matrice 4E" in one command.

Run it inside the container, with the bridge up:
    python3 tools/check_surface.py
    python3 tools/check_surface.py --rates      # also measure publish rates
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from psdk_surface import (COMMAND, COMMAND_UNCONVERTIBLE, EXPECTED_HZ,  # noqa: E402
                          NO_DATA, SERVICES, SYNTHESIS_GAP, TELEMETRY,
                          VIDEO_OPT_IN)

PREFIX = "/wrapper/psdk_ros2/"


# Deliberately NOT `ros2 topic list`.
#
# Built on the CLI, this check reported 0/102 against a bridge that was
# demonstrably publishing -- the CLI daemon was answering from a cache
# populated before the bridge started. That is the same failure already
# recorded as finding D-4 (`ros2 topic hz` claiming silence on a 250 Hz topic,
# `ros2 topic info` disagreeing with `ros2 topic list`), and it is worth
# stating twice: ANY check the project relies on talks to the graph through
# rclpy, never through the CLI.
def _graph() -> tuple[set[str], set[str], int]:
    """Return (topic names, service names, service-event topic count)."""
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("walker_surface_check")
    # Discovery is not instantaneous. Spin briefly so the graph is populated
    # before it is read -- a fresh participant that reads immediately sees a
    # partial graph and reports missing things that are there.
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    topics = {n for n, _ in node.get_topic_names_and_types()}
    services = {n for n, _ in node.get_service_names_and_types()}
    node.destroy_node()
    rclpy.shutdown()

    events = sum(1 for n in topics
                 if n.startswith(PREFIX) and n.endswith("/_service_event"))
    strip = lambda s: {n[len(PREFIX):] for n in s if n.startswith(PREFIX)}
    return ({n for n in strip(topics) if not n.startswith("_")},
            {n for n in strip(services) if not n.startswith("_")},
            events)


def report(label: str, want: list[str], have: set[str]) -> tuple[int, int, int]:
    """
    Compare, and classify what is absent.

    A flat "missing" list is the wrong shape here, because three very different
    things produce it and only one of them is a defect:

      opt-in      video, off by design until a camera group is enabled
      gap         never synthesised; on real hardware it came from the aircraft
      UNEXPECTED  a route that should be live and is not -- the only failure

    Reporting them together is how a known, documented design decision gets
    mistaken for a regression on every run.
    """
    want_set = set(want)
    present = want_set & have
    missing = want_set - have
    opt_in = sorted(missing & set(VIDEO_OPT_IN))
    gap = sorted(missing & (set(SYNTHESIS_GAP) | set(COMMAND_UNCONVERTIBLE)))
    bad = sorted(missing - set(opt_in) - set(gap))

    pct = 100.0 * len(present) / len(want_set) if want_set else 100.0
    print(f"\n{label}: {len(present)}/{len(want_set)} live ({pct:.0f}%)")
    if opt_in:
        print(f"  opt-in, appears when cameras are enabled ({len(opt_in)}): "
              f"{', '.join(opt_in)}")
    if gap:
        print(f"  not synthesised by this simulation ({len(gap)}): {', '.join(gap)}")
    if bad:
        print(f"  UNEXPECTEDLY ABSENT ({len(bad)}):")
        for m in bad:
            print(f"    - {m}")
    return len(present), len(want_set), len(bad)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rates", action="store_true")
    a = ap.parse_args()

    topics, services, n_events = _graph()
    score = []
    score.append(report("TELEMETRY topics", TELEMETRY, topics))
    score.append(report("COMMAND topics", COMMAND, topics))
    report("topics a real M4E does not source (simulation-only here)",
           NO_DATA, topics)
    score.append(report("SERVICES", SERVICES, services))

    have = sum(s[0] for s in score)
    want = sum(s[1] for s in score)
    unexpected = sum(s[2] for s in score)
    print(f"\n=== {have}/{want} of the required surface is live; "
          f"{unexpected} unexpectedly absent ===")

    # Service introspection: rosbag2 records a service CALL through its
    # <service>/_service_event topic, and the server only publishes that when
    # introspection is on. Without it a bag can replay a sortie's setpoints but
    # never the takeoff that started it -- V1 learned this and V2 keeps the fix.
    print(f"service introspection: {n_events} /_service_event topics "
          f"({'on' if n_events else 'OFF -- service calls will not be recorded'})")
    if a.rates:
        check_rates(topics)
    # The exit code tracks only UNEXPECTED absence. Opt-in video and the
    # documented synthesis gap are decisions, not failures, and a check that
    # fails on a decision gets ignored.
    return 0 if unexpected == 0 else 1


def check_rates(topics: set[str]) -> None:
    """Measure publish rates and compare with the PDF, within +/-20%."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)

    names = [n for n in EXPECTED_HZ if n in topics]
    if not names:
        print("\nrates: no measurable topics are live")
        return

    rclpy.init(args=None)
    node = Node("walker_rate_check")

    # DISCOVERY IS NOT INSTANTANEOUS, and a fresh participant knows nothing
    # about the graph the moment it is created. Querying the topic list
    # immediately returns almost nothing, every lookup below then finds no
    # type, no subscription is created, and the result is a confident table of
    # 0.0 Hz for topics that are publishing at 50 Hz. That is exactly how this
    # function failed the first time it ran.
    settle_end = time.monotonic() + 3.0
    while time.monotonic() < settle_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    # BEST_EFFORT reader: it matches a best-effort writer AND a reliable one,
    # so one profile covers the whole surface. The reverse (a reliable reader)
    # silently fails to match PX4's best-effort publishers -- finding D-4.
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    counts = {n: 0 for n in names}
    # Subscribe generically: the type does not matter for counting, and
    # resolving 23 message classes would add failure modes to a rate check.
    from rclpy.subscription import Subscription  # noqa: F401
    import rosidl_runtime_py.utilities as ru
    subs = []
    graph = dict(node.get_topic_names_and_types())
    skipped = []
    for n in names:
        types = graph.get(PREFIX + n, [])
        if not types:
            skipped.append(n)
            continue
        try:
            cls = ru.get_message(types[0])
        except Exception:
            continue

        def cb(_m, k=n):
            counts[k] += 1
        subs.append(node.create_subscription(cls, PREFIX + n, cb, qos))

    window = 6.0
    end = time.monotonic() + window
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    print(f"\nRATES vs the PDF (+/-20%, {window:.0f}s window)")
    if skipped:
        print(f"  (no type resolved, not measured: {', '.join(skipped)})")
    ok = bad = 0
    for n in sorted(names):
        want = EXPECTED_HZ[n]
        got = counts[n] / window
        within = abs(got - want) <= 0.2 * want
        if counts[n] == 0:
            mark, note = "  --", "no messages"
        elif within:
            mark, note = "  ok", ""
        else:
            mark, note = " OFF", f"expected ~{want:.0f}"
        ok += int(within)
        bad += int(not within and counts[n] > 0)
        print(f"  {mark}  {n:<26} {got:6.1f} Hz   {note}")
    print(f"  {ok} within tolerance, {bad} outside")


if __name__ == "__main__":
    sys.exit(main())
