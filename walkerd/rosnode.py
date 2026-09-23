"""
One long-lived rclpy node, for the whole life of walkerd.

WHY NOT A NODE PER PROBE
========================
`probes.py` creates a node, measures, and shuts rclpy down. That is right for a
one-shot command and wrong inside a daemon, for two reasons observed here:

  * REPEATED init/shutdown DEGRADES. walkerd's readiness probe runs once a
    second during start-up; after a few dozen init/shutdown cycles a fresh
    participant stopped seeing /clock that was demonstrably publishing at
    250 Hz. One context, created once, does not have that problem.

  * EVERY NEW PARTICIPANT PAYS DISCOVERY AGAIN. A node that has been up for a
    minute already knows every publisher; a node created for a 2.5 s
    measurement spends most of that window discovering. That is why a probe
    built this way can report "no messages" about a healthy topic.

So: subscribe once, count forever, and answer a probe by reading counters.
A measurement becomes a subtraction rather than a subscription, which is both
instant and correct.

WHAT IT WATCHES
===============
The chain from PX4 to the wrapper surface, plus whatever cameras the current
composition has. Rates are computed over a rolling window so "what is the rate
right now" is always available without waiting.
"""

from __future__ import annotations

import threading
import time
from collections import deque

# (key, human name, topic(s), type string, qos kind, why it matters)
#
# A topic entry may be a LIST of candidates, tried together.
#
# WHY: PX4 now versions its uORB topics, so the graph carries both
# /fmu/out/vehicle_status_v1 and /fmu/out/vehicle_status_v4 -- and on this
# build only v4 actually publishes. Subscribing to the wrong one gives a topic
# that exists, shows a publisher, and never delivers a message: precisely the
# silent failure these probes exist to catch, and one that would otherwise
# reappear with every PX4 upgrade. Subscribing to all candidates and counting
# them together makes the probe survive the version churn.
WATCH = [
    ("clock", "gazebo clock", "/clock", "rosgraph_msgs/msg/Clock", "default",
     "Gazebo is stepping and ros_gz_bridge is up"),
    ("imu", "px4 -> xrce -> ros", "/fmu/out/sensor_combined",
     "px4_msgs/msg/SensorCombined", "sensor",
     "PX4 is running and the XRCE agent is forwarding uORB to DDS"),
    ("attitude", "px4 attitude", "/fmu/out/vehicle_attitude",
     "px4_msgs/msg/VehicleAttitude", "sensor",
     "the estimator is producing an attitude solution"),
    ("status", "px4 status",
     ["/fmu/out/vehicle_status_v4", "/fmu/out/vehicle_status_v1",
      "/fmu/out/vehicle_status"],
     "px4_msgs/msg/VehicleStatus", "sensor",
     "arming state and nav mode, which the bridge turns into FlightStatus"),
    ("w_status", "wrapper flight_status", "/wrapper/psdk_ros2/flight_status",
     "psdk_interfaces/msg/FlightStatus", "sensor",
     "the PSDK bridge is translating PX4 state to the DJI surface"),
    ("w_height", "wrapper height", "/wrapper/psdk_ros2/height_above_ground",
     "std_msgs/msg/Float32", "sensor",
     "what every C++ mission's CLIMB step reads"),
]

WINDOW = 5.0        # seconds of history kept per topic


class RosWatcher:
    """Runs rclpy in its own thread; everything public is thread-safe."""

    def __init__(self) -> None:
        self._stamps: dict[str, deque] = {k: deque() for k, *_ in WATCH}
        self._lock = threading.Lock()
        self._node = None
        self._control = None      # publisher onto /simty/control
        self._panel_state = None  # publisher onto /walker/state (the RViz panel)
        self._panel_cmd = None    # subscription on /walker/control
        self._cameras = None      # publisher onto /walker/cameras/enabled
        self.on_panel_command = None   # set by the daemon
        self._exec = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.error = ""
        self.missing_types: list[str] = []
        # Last-known values the RViz panel shows. Kept here because this is the
        # one place already subscribed to the wrapper surface.
        self.last_height = 0.0
        self.last_armed = False
        self.last_mode = ""

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True, name="roswatch")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _qos(self, kind: str):
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        if kind == "sensor":
            # BEST_EFFORT, because that is what PX4 and every image stream
            # publish with. A RELIABLE subscription simply never matches them
            # and reports silence -- the trap that makes `ros2 topic hz` lie.
            return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)

    def _run(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
        except Exception as e:                       # pragma: no cover
            self.error = f"rclpy unavailable: {e}"
            return

        try:
            rclpy.init(args=None)
            self._node = Node("walkerd_watch")

            # The bridge's command surface. In V1 this was driven by the
            # bridge's own curses console; in V2 walker is the operator
            # surface, so walkerd holds the publisher and walker's keys reach
            # it over the socket. One verb per message -- see simty/control.py.
            from std_msgs.msg import String
            self._control = self._node.create_publisher(String, "/simty/control", 10)

            # THE RVIZ PANEL'S ENTIRE INTERFACE: one state topic out, one
            # command topic in.
            #
            # V1's panels each held their own subscriptions to raw wrapper
            # topics and repainted from ROS callbacks arriving at 50 Hz. This
            # publishes ONE pre-digested document at 5 Hz, so the panel does no
            # parsing, no rate limiting and no repaint faster than a person can
            # read. See walker_rviz_panel/.
            #
            # TRANSIENT_LOCAL so a panel that opens later gets the current
            # state immediately instead of a blank widget until the next tick.
            from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                                   ReliabilityPolicy)
            latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
            self._panel_state = self._node.create_publisher(
                String, "/walker/state", latched)
            # The camera manager's whole interface: the complete desired set,
            # latched so a manager that (re)starts adopts it immediately
            # instead of rendering nothing until the next change.
            self._cameras = self._node.create_publisher(
                String, "/walker/cameras/enabled", latched)

            def _on_cmd(msg):
                cb = self.on_panel_command
                if cb:
                    try:
                        cb(msg.data)
                    except Exception:
                        pass

            self._panel_cmd = self._node.create_subscription(
                String, "/walker/control", _on_cmd, 10)
            for key, _n, topics, type_str, qos, _w in WATCH:
                cls = self._import(type_str)
                if cls is None:
                    self.missing_types.append(type_str)
                    continue

                def cb(msg, k=key):
                    with self._lock:
                        self._stamps[k].append(time.monotonic())
                    if k == "w_height":
                        self.last_height = float(getattr(msg, "data", 0.0))
                    elif k == "w_status":
                        # FlightStatus: 0 stopped, 1 on ground, 2 in air.
                        fs = int(getattr(msg, "flight_status", 0))
                        self.last_armed = fs != 0
                        self.last_mode = {0: "STOPPED", 1: "ON_GROUND",
                                          2: "ON_AIR"}.get(fs, "?")

                for topic in ([topics] if isinstance(topics, str) else topics):
                    self._node.create_subscription(cls, topic, cb, self._qos(qos))

            while not self._stop.is_set():
                rclpy.spin_once(self._node, timeout_sec=0.1)
                self._trim()
        except Exception as e:                       # pragma: no cover
            self.error = f"{type(e).__name__}: {e}"
        finally:
            try:
                if self._node:
                    self._node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass

    @staticmethod
    def _import(type_str: str):
        pkg, kind, name = type_str.split("/")
        try:
            mod = __import__(f"{pkg}.{kind}", fromlist=[name])
            return getattr(mod, name)
        except Exception:
            return None

    def _trim(self) -> None:
        cutoff = time.monotonic() - WINDOW
        with self._lock:
            for dq in self._stamps.values():
                while dq and dq[0] < cutoff:
                    dq.popleft()

    def send_control(self, text: str, wait_for_bridge: float = 5.0) -> bool:
        """
        Send one verb to the bridge. True if it was published.

        Waits briefly for the bridge's subscription to exist first. Publishing
        into a topic with no subscriber succeeds silently and the command
        simply vanishes -- which is how V1's launcher used to report "project
        testing mode requested" about a bridge that never heard it, leaving
        every service the project was about to call unserved.
        """
        if self._control is None:
            return False
        from std_msgs.msg import String
        deadline = time.monotonic() + wait_for_bridge
        while time.monotonic() < deadline:
            if self._control.get_subscription_count() > 0:
                break
            time.sleep(0.2)
        else:
            return False
        msg = String()
        msg.data = text
        self._control.publish(msg)
        return True

    def call_wrapper_service(self, name: str, timeout: float = 5.0) -> bool:
        """
        Call a std_srvs/Trigger on the wrapper surface, fire and forget.

        Used by the RViz panel's buttons. Not waited on: a button that freezes
        RViz until a service answers is worse than one whose effect shows up in
        the telemetry a moment later.
        """
        if self._node is None:
            return False
        try:
            from std_srvs.srv import Trigger
            client = self._node.create_client(
                Trigger, f"/wrapper/psdk_ros2/{name}")
            if not client.wait_for_service(timeout_sec=timeout):
                return False
            client.call_async(Trigger.Request())
            return True
        except Exception:
            return False

    def publish_cameras(self, enabled: list[str], active: str = "") -> None:
        """Tell the camera manager exactly which lenses should be rendering."""
        if self._cameras is None:
            return
        import json
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps({"enabled": sorted(enabled), "active": active})
        self._cameras.publish(msg)

    def publish_panel_state(self, doc: dict) -> None:
        """Push one state document to the RViz panel. Cheap; called at 5 Hz."""
        if self._panel_state is None:
            return
        import json
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps(doc, separators=(",", ":"))
        self._panel_state.publish(msg)

    # ── reading ──────────────────────────────────────────────────────────────

    def rates(self) -> dict[str, float]:
        """Current rate per key, over the rolling window. Instant; no waiting."""
        self._trim()
        with self._lock:
            out = {}
            for k, dq in self._stamps.items():
                if len(dq) < 2:
                    out[k] = 0.0
                else:
                    span = dq[-1] - dq[0]
                    out[k] = (len(dq) - 1) / span if span > 0 else 0.0
            return out

    def links(self, wrapper: bool = True) -> list[dict]:
        r = self.rates()
        out = []
        for key, name, topics, type_str, _q, why in WATCH:
            if not wrapper and key.startswith("w_"):
                continue
            topic = topics if isinstance(topics, str) else topics[0]
            hz = r.get(key, 0.0)
            if type_str in self.missing_types:
                state, detail = "fail", f"{type_str} is not importable"
            elif hz > 0:
                state, detail = "ok", f"{hz:.1f} Hz"
            else:
                state, detail = "fail", "no messages"
            out.append({"key": key, "name": name, "topic": topic,
                        "state": state, "detail": detail, "why": why})
        return out

    def ready(self, keys: tuple[str, ...]) -> bool:
        r = self.rates()
        return all(r.get(k, 0.0) > 0 for k in keys)
