#!/usr/bin/env python3
"""
probes.py — answer "is the chain up?", link by link, by name.

WHY NOT JUST CALL THE ros2 CLI
==============================
Because it lies, slowly. Measured on this stack while the simulation was
demonstrably healthy and publishing:

  * `ros2 topic hz /fmu/out/sensor_combined`   -> "no data"
        It subscribes RELIABLE by default. PX4's topics are BEST_EFFORT, so
        the subscription never matches and the tool reports silence rather
        than incompatibility. The same trap bites RViz image displays.

  * `ros2 topic info /fmu/out/sensor_combined` -> "Unknown topic"
        ...while `ros2 topic list` listed that exact topic, because the two
        take different paths through the CLI daemon's cache.

  * every invocation costs 4-5 s of Python import and daemon round-trip
        before discovery even begins.

A readiness check built on that would be slow AND wrong, and the failure it
reports would send someone to debug a simulation that was working. So probes
open one rclpy node, subscribe with the QoS the publisher actually uses, and
measure for a fixed window.

WHAT A PROBE IS FOR
===================
A C++ project that cannot reach the wrapper surface sits in WAIT_FOR_DATA
forever, and every possible cause looks identical from inside it. These checks
distinguish them, so the answer is "the XRCE agent is not forwarding" and not
"something is wrong".

USAGE
    python3 probes.py --json            # machine-readable, for walkerd
    python3 probes.py                   # human-readable table
    python3 probes.py --window 3.0
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

# The links, in the order data actually travels. Each entry is
#   (key, human name, topic, type string, qos, why it matters)
#
# QoS: "sensor" is BEST_EFFORT/VOLATILE/depth 5 -- what PX4 and every image
# stream publish with. "default" is RELIABLE. Getting this wrong is the single
# most common reason a probe reports a healthy link as dead.
CHAIN = [
    ("clock", "gazebo clock", "/clock", "rosgraph_msgs/msg/Clock", "default",
     "Gazebo is stepping and ros_gz_bridge is up"),
    ("imu", "px4 -> xrce -> ros", "/fmu/out/sensor_combined",
     "px4_msgs/msg/SensorCombined", "sensor",
     "PX4 is running and the XRCE agent is forwarding uORB to DDS"),
    ("attitude", "px4 attitude", "/fmu/out/vehicle_attitude",
     "px4_msgs/msg/VehicleAttitude", "sensor",
     "the estimator is producing an attitude solution"),
]

# The wrapper surface. Only meaningful once the bridge unit is up; absent
# before that, which is not an error, just a later stage.
WRAPPER = [
    ("w_status", "wrapper flight_status", "/wrapper/psdk_ros2/flight_status",
     "psdk_interfaces/msg/FlightStatus", "sensor",
     "the PSDK bridge is translating PX4 state to the DJI surface"),
    ("w_height", "wrapper height", "/wrapper/psdk_ros2/height_above_ground",
     "std_msgs/msg/Float32", "sensor",
     "what every C++ mission's CLIMB step reads"),
]


def _qos(kind: str):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    if kind == "sensor":
        return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)


def _import(type_str: str):
    """'px4_msgs/msg/SensorCombined' -> the class, or None if unavailable."""
    pkg, kind, name = type_str.split("/")
    try:
        mod = __import__(f"{pkg}.{kind}", fromlist=[name])
        return getattr(mod, name)
    except Exception:
        return None


def measure(entries, window: float = 2.5) -> dict:
    """Subscribe to everything at once and count messages for `window` seconds."""
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("walker_probe")
    counts = {k: 0 for k, *_ in entries}
    subs = []
    for key, _name, topic, type_str, qos, _why in entries:
        cls = _import(type_str)
        if cls is None:
            counts[key] = -1          # type unavailable: a build problem
            continue

        def cb(_msg, k=key):
            counts[k] += 1

        subs.append(node.create_subscription(cls, topic, cb, _qos(qos)))

    # One spin loop for every subscription, rather than a probe per topic:
    # discovery is paid once and the whole picture is from the same instant.
    end = time.monotonic() + window
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return {k: (v / window if v > 0 else v) for k, v in counts.items()}


def process_links() -> dict:
    """
    The non-ROS half: are the processes that should exist actually alive?

    THE BRACKET TRICK IS NOT DECORATION. `pgrep -f "gz sim"` matches the
    command line of the shell that is running the probe, because that command
    line contains the string "gz sim". The result is a probe that reports
    Gazebo as healthy seconds after Gazebo died -- observed here, and it is
    exactly the kind of false-OK that sends someone to debug the wrong layer.

    "[g]z sim" is a regex matching the literal text "gz sim", but the pattern
    as written does not contain that text, so it cannot match itself.

    Zombies do not count as alive: gz's launcher is a ruby wrapper that leaves
    a defunct entry behind, and `pgrep` happily reports it. -r excludes any
    state but running/sleeping.
    """
    def alive(pattern: str) -> bool:
        cp = subprocess.run(["pgrep", "-r", "DRSW", "-f", pattern],
                            capture_output=True)
        return cp.returncode == 0

    home = os.path.expanduser("~")
    return {
        "gazebo": alive(r"[g]z sim"),
        "xrce_agent": alive("[M]icroXRCEAgent"),
        "ros_gz_bridge": alive("[p]arameter_bridge"),
        "px4": alive("[b]in/px4"),
        "px4_socket": os.path.exists("/tmp/px4-sock-0"),
        "composed": os.path.isfile(os.path.join(home, "gz_runtime", "compose.json")),
    }


def run(window: float = 2.5, wrapper: bool = True) -> dict:
    procs = process_links()
    entries = CHAIN + (WRAPPER if wrapper else [])
    rates = measure(entries, window)

    links = []
    for key, name, topic, type_str, _qos_kind, why in entries:
        hz = rates.get(key, 0)
        if hz == -1:
            state, detail = "fail", f"{type_str} is not importable"
        elif hz > 0:
            state, detail = "ok", f"{hz:.1f} Hz"
        else:
            state, detail = "fail", "no messages"
        links.append({"key": key, "name": name, "topic": topic,
                      "state": state, "detail": detail, "why": why})

    return {"processes": procs, "links": links,
            "ok": all(l["state"] == "ok" for l in links if not l["key"].startswith("w_"))}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--json", action="store_true")
    p.add_argument("--window", type=float, default=2.5)
    p.add_argument("--no-wrapper", action="store_true")
    a = p.parse_args()

    result = run(a.window, wrapper=not a.no_wrapper)
    if a.json:
        print(json.dumps(result, indent=2))
        return 0 if result["ok"] else 1

    print("\n  processes")
    for k, v in result["processes"].items():
        print(f"    {'[ ok ]' if v else '[fail]'} {k}")
    print("\n  links")
    for l in result["links"]:
        mark = "[ ok ]" if l["state"] == "ok" else "[fail]"
        print(f"    {mark} {l['name']:<26} {l['detail']:<14} {l['topic']}")
        if l["state"] != "ok":
            print(f"           ^ {l['why']}")
    print()
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
