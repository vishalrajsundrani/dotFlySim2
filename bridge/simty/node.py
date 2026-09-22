"""
The bridge node: turns the route tables into live DDS endpoints.

Three design decisions carry most of the weight here.

1. ENDPOINT SHARING
   Four routes read /fmu/out/vehicle_status_v4 (flight status, display mode,
   anomaly, RC link) and two read /wrapper/psdk_ros2/rc. Creating one
   subscription per route would mean N deserialisations of every sample. Routes
   that agree on (topic, type, QoS) share a single endpoint and fan out to
   handlers in Python instead. Publishers are reference-counted the same way,
   so tearing down one route cannot yank a publisher another route still uses.

2. MUTATION HAPPENS ON THE EXECUTOR THREAD
   rclpy is not thread-safe for entity creation. The TUI never touches the node
   directly -- it pushes a callable onto a queue, and a timer drains that queue
   on the executor thread. That is what makes "toggle a route while data is
   flowing" safe rather than a race.

3. NOTHING ARMS ITSELF
   auto_offboard and auto_arm both default to False. The bridge publishes an
   offboard heartbeat (PX4 rejects offboard without one at >2 Hz) whenever RC
   authority is live, but changing flight mode or arming is an explicit
   operator action from the TUI or an explicit setting.
"""

import os
import queue
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool

from . import converters as cv
from . import inspect as inspect_mod
from . import discovery as disc
from . import qos as qos_mod
from .journal import ConversionJournal
from .registry import (
    Direction,
    ServiceRole,
    ServiceRoute,
    TopicRoute,
    build_service_routes,
    build_topic_routes,
    missing_types,
)
from .state import BridgeState, DEBUG, ERROR, INFO, WARN

# PX4 MAV_CMD numbers, used when the installed px4_msgs lacks the constant.
_CMD_FALLBACK = {
    "NAV_TAKEOFF": 22,
    "NAV_LAND": 21,
    "NAV_RETURN_TO_LAUNCH": 20,
    "COMPONENT_ARM_DISARM": 400,
    "DO_SET_MODE": 176,
    "DO_SET_HOME": 179,
}

# Every route that writes /fmu/in/trajectory_setpoint, and the OffboardControlMode
# flags PX4 needs for the kind of setpoint each one produces. PX4 ignores a
# setpoint whose matching flag is not set in the heartbeat, so this mapping is
# what makes a position route behave differently from a velocity one -- and the
# keys are also the set the single-writer rule arbitrates between.
# THE ONE TAKEOFF HEIGHT IN THIS SIMULATION.
#
# DJI's takeoff service carries no altitude -- it is a Trigger -- so the height
# is a property of the aircraft, not of the call, and every C++ project in
# demo/ is written against 1.8 m. It is set in three places that must agree:
#
#   here                     what the bridge asks PX4 for on `takeoff`
#   MIS_TAKEOFF_ALT          what PX4's own auto-takeoff climbs to, set in
#                            config/launch_sim.sh
#   kFlightHeight/takeoff_altitude in each demo
#
# Change one and change all three, or a project will spend its CLIMB step
# fighting an auto-takeoff that stopped somewhere else.
TAKEOFF_ALTITUDE_M = 1.8

SETPOINT_ROUTES = {
    # key                          -> (position, velocity)
    "rc_setpoint": (False, True),
    "psdk_position_setpoint": (True, False),
    "psdk_velocity_setpoint": (False, True),
    "psdk_body_velocity_setpoint": (False, True),
}

# PX4 custom main mode for OFFBOARD, used with DO_SET_MODE.
_PX4_MAIN_MODE_OFFBOARD = 6.0


class _SharedSub:
    __slots__ = ("subscription", "handlers")

    def __init__(self, subscription):
        self.subscription = subscription
        self.handlers: Dict[str, Callable] = {}


class _SharedPub:
    __slots__ = ("publisher", "refs")

    def __init__(self, publisher):
        self.publisher = publisher
        self.refs: Set[str] = set()


class SimtyBridge(Node):
    """The bridge. One node, many routes, all of them re-configurable live."""

    def __init__(self, settings, state: Optional[BridgeState] = None):
        super().__init__("simty_bridge")
        self.settings = settings
        self.state = state or BridgeState()
        self.shared: Dict[str, Any] = {}

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()

        # endpoint pools
        self._subs: Dict[str, _SharedSub] = {}
        self._pubs: Dict[str, _SharedPub] = {}
        self._route_subs: Dict[str, str] = {}  # route key -> sub pool key
        self._route_pubs: Dict[str, List[str]] = {}  # route key -> pub pool keys

        # rate limiting
        self._last_pass: Dict[str, float] = {}
        # Next time each route is allowed to publish; see the deadline note above.
        self._next_due: dict = {}

        # The one route key whose messages are exploded field-by-field for the
        # INSPECT screen, or "" for none. Written by the TUI thread and read
        # here; a bare string assignment is atomic under the GIL and a stale
        # read costs one frame of a debug view, so this needs no lock.
        self.inspect_key: str = ""

        # service endpoints
        self._servers: Dict[str, Any] = {}
        self._clients: Dict[str, Any] = {}

        # The conversion journal: every clamp, substitution and inference a
        # converter makes, aggregated by (route, code). Its text output goes both
        # to the TUI's log ring and -- for WARN and above -- to the ROS logger,
        # which puts it on /rosout where rqt_console and `ros2 topic echo` can
        # see it without this process having to be the only window on the bridge.
        self.journal = ConversionJournal(
            sink=self._journal_sink,
            min_interval_s=settings.journal_min_interval_s,
        )

        # conversion context shares one dict with everything else
        from .clock import Px4Clock, SimClock

        self.clock_px4 = Px4Clock()
        # Gazebo's /clock. Separate from clock_px4 on purpose: PX4 boots long
        # after the world does, so the two domains are offset by that startup
        # gap, and only this one matches the stamps on bridged camera frames.
        self.clock_sim = SimClock()

        self._ctx = cv.ConversionContext(
            clock=self.clock_px4,
            params=settings.params,
            shared=self.shared,
            sim_clock=self.clock_sim,
            log=lambda source, message: self.state.log.add(DEBUG, source, message),
            journal=self.journal,
            route="bridge",
        )

        # command queue: the only way in from other threads
        self._commands: "queue.Queue[Callable[[SimtyBridge], None]]" = queue.Queue()
        # ENDPOINT MUTATIONS GO HERE, and are applied ONE PER TICK.
        #
        # Realising a route destroys and recreates DDS endpoints. Doing dozens
        # of those inside a single callback -- which is what project testing
        # mode did when it pointed thirty routes at once -- wedged the whole
        # node: it took four samples on each new route and then stopped
        # dispatching anything, alive but deaf, with its own render loop
        # stopped too. rclpy makes no promise that destroying an entity while
        # the executor dispatches others is safe, and a burst of sixty makes
        # hitting that a certainty rather than a risk. Spread over the 20 Hz
        # drain, the same reconfiguration takes a second or two and holds.
        self._slow_commands: "queue.Queue[Callable[[SimtyBridge], None]]" = queue.Queue()

        # routes
        self.routes: List[TopicRoute] = build_topic_routes(settings)
        self.service_routes: List[ServiceRoute] = build_service_routes(settings)
        for problem in missing_types(self.routes, self.service_routes):
            self.state.log.add(WARN, "registry", problem)
        for route in self.routes:
            if not route.available:
                self.journal.report(
                    route.key, route.direction.value, "type_unavailable",
                    f"host={getattr(route.host_type, '__name__', route.host_type)} "
                    f"sim={getattr(route.sim_type, '__name__', route.sim_type)}",
                )

        # Filled in by the launcher, which owns the executor and therefore the
        # only safe way to add or remove the extra participants. The control
        # channel exposes them to the panels; everything here tolerates None so
        # a caller that wires up only the bridge still works.
        self.provisioner = None
        self.mock_control = None  # callable(enable: bool) -> str
        self.standin_control = None  # callable(enable: bool) -> str
        self.profile_path = ""

        # offboard bookkeeping
        self._offboard_stream_since = 0.0
        self._offboard_engaged = False
        self._offboard_pub_key: Optional[str] = None

        # Guards the single-writer rule on /fmu/in/trajectory_setpoint against
        # re-entering itself: switching a loser off is itself a direction change.
        self._arbitrating = False
        # Project testing mode: the route/service/setting state it replaced, so
        # turning it off puts the bridge back rather than leaving it wide open.
        self.project_mode = False
        self._project_restore: Optional[Dict[str, Any]] = None
        self.replay_mode = False
        self._replay_restore: Optional[Dict[str, Any]] = None

        # When an arm (or takeoff, which implies one) was last asked for, so
        # housekeeping can notice that it did not take and say why.
        self._arm_requested_at = 0.0

        # discovery
        self.scanner = disc.GraphScanner(
            self, settings, own_node_names={"simty_bridge"}
        )
        self._report = disc.DiscoveryReport()
        self._prev_surface = None

        # bring everything up
        self._realise_all_routes()
        self._realise_all_services()
        self._subscribe_clock()
        self._subscribe_sim_clock()
        self._subscribe_failsafe_flags()
        self._subscribe_sim_attitude()
        self._subscribe_host_attitude()
        self._subscribe_gimbal_joints()

        # The control room. viz and diagnostics are publish-only and
        # subscriber-gated, so constructing them is not a commitment to any
        # traffic; control adds the one inbound path.
        self.viz = None
        self.diagnostics = None
        self.control = None
        self._start_control_room()

        # timers
        self.create_timer(0.05, self._drain_commands)  # 20 Hz
        self.create_timer(max(0.02, 1.0 / max(2.0, settings.offboard_hz)), self._offboard_tick)
        self.create_timer(max(0.5, settings.scan_period_s), self._scan_tick)
        self.create_timer(0.5, self._housekeeping_tick)

        facts = disc.middleware_facts(settings.domain)
        self.state.log.add(
            INFO,
            "bridge",
            f"started on domain {facts['domain']} via {facts['rmw']}; "
            f"SPDP multicast UDP {facts['multicast_port']}",
        )
        if not str(facts["rmw"]).startswith("rmw_fastrtps"):
            self.state.log.add(
                WARN,
                "bridge",
                f"RMW is {facts['rmw']} but this project is configured for Fast DDS. "
                "Mixed RMWs do not interoperate reliably -- set "
                "RMW_IMPLEMENTATION=rmw_fastrtps_cpp everywhere.",
            )
        if facts["profiles_file"] and not facts["profiles_ok"]:
            self.state.log.add(
                WARN,
                "bridge",
                f"FASTRTPS_DEFAULT_PROFILES_FILE={facts['profiles_file']} does not exist; "
                "Fast DDS is running on defaults (64 KB datagrams, no MTU capping)",
            )

    # ── the control room's publish-only layers ───────────────────────────────

    def _journal_sink(self, level: int, source: str, message: str) -> None:
        """
        One journal line -> the TUI log ring, and for anything at WARN or above
        also the ROS logger. The second half matters: it puts conversion
        warnings on /rosout, so `ros2 topic echo /rosout` or rqt_console shows
        them from another terminal, another container, or a CI job -- none of
        which can read this process's TUI.
        """
        self.state.log.add(level, source, message)
        if level >= ERROR:
            self.get_logger().error(f"[{source}] {message}")
        elif level >= WARN:
            self.get_logger().warning(f"[{source}] {message}")

    def _start_control_room(self) -> None:
        """
        Bring up the RViz marker layer and the diagnostics/event publishers.

        Both are optional in the strong sense: a workspace missing
        visualization_msgs or tf2_ros should cost the control room, not the
        bridge, so an import failure is logged and the rest keeps running.
        """
        try:
            from .viz import ControlRoomViz

            self.viz = ControlRoomViz(self)
            self.create_timer(
                1.0 / max(0.5, float(self.settings.viz_hz)), self._viz_tick
            )
            self.state.log.add(
                INFO,
                "viz",
                "RViz control room on /simty/viz/{flow,services,status,vehicle,path}"
                " (published only while something is subscribed)",
            )
        except Exception as exc:
            self.state.log.add(
                WARN, "viz", f"RViz marker layer unavailable: {type(exc).__name__}: {exc}"
            )

        try:
            from .diagnostics import DiagnosticsPublisher

            self.diagnostics = DiagnosticsPublisher(self)
            self.create_timer(1.0, self._diagnostics_tick)
            self.state.log.add(
                INFO,
                "diagnostics",
                "/simty/diagnostics and /simty/conversion_events are live",
            )
        except Exception as exc:
            self.state.log.add(
                WARN,
                "diagnostics",
                f"diagnostics publisher unavailable: {type(exc).__name__}: {exc}",
            )

        if not self.settings.enable_control:
            self.state.log.add(
                WARN,
                "control",
                "remote control disabled (enable_control off); the console is the "
                "only way to change anything this session",
            )
            return
        try:
            from .control import BridgeControl

            self.control = BridgeControl(self)
            self.create_timer(0.25, self._control_tick)
            self.state.log.add(
                INFO,
                "control",
                "control channel up: /simty/control (commands), "
                "/simty/control_result, /simty/state",
            )
        except Exception as exc:
            self.state.log.add(
                WARN, "control", f"control channel unavailable: {type(exc).__name__}: {exc}"
            )

    def _viz_tick(self) -> None:
        if self.viz is None:
            return
        try:
            self.viz.publish()
        except Exception as exc:
            # A rendering bug must never stop the bridge translating messages.
            self.state.log.add(ERROR, "viz", f"{type(exc).__name__}: {exc}")

    def _diagnostics_tick(self) -> None:
        if self.diagnostics is None:
            return
        try:
            self.diagnostics.publish()
        except Exception as exc:
            self.state.log.add(ERROR, "diagnostics", f"{type(exc).__name__}: {exc}")

    def _control_tick(self) -> None:
        if self.control is None:
            return
        try:
            self.control.publish()
        except Exception as exc:
            self.state.log.add(ERROR, "control", f"{type(exc).__name__}: {exc}")

    # ── thread-safe entry point ──────────────────────────────────────────────

    def submit_slow(self, action: Callable[["SimtyBridge"], None]) -> None:
        """Queue one endpoint mutation, applied a tick after the one before."""
        self._slow_commands.put(action)

    def submit(self, action: Callable[["SimtyBridge"], None]) -> None:
        """Queue a mutation to run on the executor thread. Safe from any thread."""
        self._commands.put(action)

    def _drain_commands(self) -> None:
        for _ in range(64):  # bounded, so one burst cannot starve callbacks
            try:
                action = self._commands.get_nowait()
            except queue.Empty:
                break
            try:
                action(self)
            except Exception as exc:
                self.state.log.add(ERROR, "command", f"{type(exc).__name__}: {exc}")

        # Exactly one endpoint mutation per tick. See _slow_commands.
        try:
            action = self._slow_commands.get_nowait()
        except queue.Empty:
            return
        try:
            action(self)
        except Exception as exc:
            self.state.log.add(ERROR, "command", f"{type(exc).__name__}: {exc}")

    # ── endpoint pools ───────────────────────────────────────────────────────

    @staticmethod
    def _key(topic: str, msg_type, qos_name: str) -> str:
        return f"{topic}|{getattr(msg_type, '__name__', msg_type)}|{qos_name}"

    def _acquire_sub(self, topic, msg_type, qos_name, owner, handler) -> str:
        key = self._key(topic, msg_type, qos_name)
        shared = self._subs.get(key)
        if shared is None:
            warning = qos_mod.warn_if_unsafe_subscription(qos_name)
            if warning and topic.startswith("/fmu/"):
                self.state.log.add(WARN, "qos", f"{topic}: {warning}")
                self.journal.report(owner, "", "qos_subscribe_risk", f"{topic} [{qos_name}]")
            subscription = self.create_subscription(
                msg_type,
                topic,
                self._make_dispatch(key),
                qos_mod.build(qos_name),
                callback_group=self._cb_group,
            )
            shared = _SharedSub(subscription)
            self._subs[key] = shared
            self.state.log.add(DEBUG, "sub", f"+ {topic} [{qos_name}]")
        shared.handlers[owner] = handler
        self._route_subs[owner] = key
        return key

    def _release_sub(self, owner) -> None:
        key = self._route_subs.pop(owner, None)
        if key is None:
            return
        shared = self._subs.get(key)
        if shared is None:
            return
        shared.handlers.pop(owner, None)
        if not shared.handlers:
            try:
                self.destroy_subscription(shared.subscription)
            except Exception:
                pass
            self._subs.pop(key, None)
            self.state.log.add(DEBUG, "sub", f"- {key.split('|')[0]}")

    def _acquire_pub(self, topic, msg_type, qos_name, owner) -> Tuple[str, Any]:
        key = self._key(topic, msg_type, qos_name)
        shared = self._pubs.get(key)
        if shared is None:
            publisher = self.create_publisher(msg_type, topic, qos_mod.build(qos_name))
            shared = _SharedPub(publisher)
            self._pubs[key] = shared
            self.state.log.add(DEBUG, "pub", f"+ {topic} [{qos_name}]")
        shared.refs.add(owner)
        self._route_pubs.setdefault(owner, [])
        if key not in self._route_pubs[owner]:
            self._route_pubs[owner].append(key)
        return key, shared.publisher

    def _release_pubs(self, owner) -> None:
        for key in self._route_pubs.pop(owner, []):
            shared = self._pubs.get(key)
            if shared is None:
                continue
            shared.refs.discard(owner)
            if not shared.refs:
                try:
                    self.destroy_publisher(shared.publisher)
                except Exception:
                    pass
                self._pubs.pop(key, None)
                self.state.log.add(DEBUG, "pub", f"- {key.split('|')[0]}")

    def _publisher_for(self, key: str):
        shared = self._pubs.get(key)
        return shared.publisher if shared else None

    def _make_dispatch(self, key: str):
        """One DDS callback per shared subscription; fans out to route handlers."""

        def dispatch(msg):
            shared = self._subs.get(key)
            if shared is None:
                return
            for handler in list(shared.handlers.values()):
                try:
                    handler(msg)
                except Exception as exc:
                    self.state.log.add(ERROR, "dispatch", f"{type(exc).__name__}: {exc}")

        return dispatch

    def _subscribe_clock(self) -> None:
        """
        Keep the PX4 clock synced regardless of which routes are on.

        Timestamps on outgoing TrajectorySetpoint, OffboardControlMode and
        VehicleCommand must be in PX4's simulated-time domain. Learning that
        offset only from enabled telemetry routes would mean an operator who
        disables all of them silently reverts every outgoing timestamp to wall
        clock -- decades out, and enough to break the offboard freshness check.
        This owns a subscription of its own (pooled with any route that happens
        to use the same topic and QoS, so it usually costs nothing).
        """
        try:
            from px4_msgs.msg import VehicleStatus
        except ImportError:
            self.state.log.add(WARN, "clock", "px4_msgs unavailable; using wall clock")
            return

        topic = self.settings.sim_topics["vehicle_status"]

        def on_status(msg):
            self.clock_px4.feed(getattr(msg, "timestamp", 0))

        self._acquire_sub(topic, VehicleStatus, "compat", "clock", on_status)
        self.state.log.add(DEBUG, "clock", f"tracking PX4 time from {topic}")

    def _clock_summary(self) -> str:
        """
        Both time domains in one short line, plus which one is reaching
        header.stamp.

        This rides in the existing `px4_clock` snapshot field rather than
        adding a new one: that field is serialised positionally on /simty/state
        (control.py) and read back by index in the C++ RViz panel
        (bridge_link.cpp field(parts, 9)), so a new column would have to be
        added in three places at once. The value is tab-cleaned downstream and
        this string contains no tabs.
        """
        px4 = self.clock_px4
        px4_text = f"PX4 t={px4.now_us() / 1e6:.1f}s" if px4.synced else "PX4 unsynced"

        sim_ns = self.clock_sim.now_ns()
        mode = str(self.settings.params.get("stamp_source", "auto"))
        if sim_ns is None:
            gz_text = "gz none"
            stamping = "wall"
        else:
            gz_text = f"gz t={sim_ns / 1e9:.1f}s"
            stamping = "wall" if mode == "wall" else "gz"
        return f"{px4_text} · {gz_text} · stamps={stamping}"

    def _subscribe_sim_clock(self) -> None:
        """
        Track Gazebo's world clock, which is what every bridged camera frame is
        stamped in.

        This is NOT the same domain as _subscribe_clock above. gz sim starts
        first and PX4 comes up last (config/launch_sim.sh), so PX4's boot-
        relative microseconds sit some tens of seconds behind the world's. A
        stamp meant to line up with an image has to come from here.

        /clock is bridged from Gazebo by ros_gz_bridge (config/bridge.yaml) and
        is RELIABLE on the gz side, so "compat" (best-effort) matches it. On a
        real Manifold nothing publishes /clock at all, this subscription simply
        never fires, and ctx.ros_stamp() stays on the wall clock -- which is
        the correct behaviour there and needs no configuration.
        """
        try:
            from rosgraph_msgs.msg import Clock
        except ImportError:
            self.state.log.add(WARN, "clock", "rosgraph_msgs unavailable; wall stamps only")
            return

        def on_clock(msg):
            stamp = getattr(msg, "clock", None)
            if stamp is not None:
                self.clock_sim.feed(getattr(stamp, "sec", 0), getattr(stamp, "nanosec", 0))

        self._acquire_sub("/clock", Clock, "compat", "sim_clock", on_clock)
        self.state.log.add(DEBUG, "clock", "tracking Gazebo sim time from /clock")

    def _subscribe_failsafe_flags(self) -> None:
        """
        Track why PX4 would refuse to arm.

        When commander says "Arming denied: Resolve system health failures first"
        it does not say which failure. FailsafeFlags does: every boolean in it is
        a named problem, so reading them back is the difference between "arming
        does not work" and "the accelerometer has timed out".

        The flag names are not hard-coded. The message's own field list is walked
        and every true boolean is reported, so a PX4 release that adds, removes or
        renames a flag is handled without an edit here -- the same tolerance _g()
        gives the converters.
        """
        try:
            from px4_msgs.msg import FailsafeFlags
        except ImportError:
            self.state.log.add(
                WARN,
                "preflight",
                "px4_msgs has no FailsafeFlags; arming refusals cannot be explained "
                "(PX4 older than v1.14?)",
            )
            return

        topic = self.settings.sim_topics.get("failsafe_flags", "/fmu/out/failsafe_flags")

        def on_flags(msg):
            blockers = []
            try:
                fields = msg.get_fields_and_field_types()
            except Exception:
                fields = {}
            for name, kind in fields.items():
                if kind != "boolean" or name.startswith("mode_req"):
                    continue
                if bool(getattr(msg, name, False)):
                    blockers.append(name)
            self.shared["arm_blockers"] = blockers
            self.shared["arm_blockers_at"] = time.time()
            # Not a blocker in itself: a bitmask of nav_states that refuse an
            # arm request, indexed by nav_state. See _clear_prevent_arming_mode.
            self.shared["mode_req_prevent_arming"] = int(
                getattr(msg, "mode_req_prevent_arming", 0)
            )
            self.shared["mode_req_offboard_signal"] = int(
                getattr(msg, "mode_req_offboard_signal", 0)
            )
            self.shared["offboard_signal_lost"] = bool(
                getattr(msg, "offboard_control_signal_lost", False)
            )

        self._acquire_sub(topic, FailsafeFlags, "compat", "preflight", on_flags)
        self.state.log.add(DEBUG, "preflight", f"watching {topic} for arming blockers")

    def _subscribe_sim_attitude(self) -> None:
        """
        Keep the simulated vehicle's orientation in shared state regardless of
        which routes are on.

        Three things read it: joy_to_trajectory_setpoint rotates body-frame stick
        input into NED by the heading, viz.py orients the `vehicle` TF frame, and
        the gimbal converter can use it as its yaw reference. All three used to be
        fed as a side effect of the gimbal_angles route running sim->host. That
        made driving the gimbal from the wrapper -- which needs that same route
        pointed host->sim -- silently revert stick control to head-north and
        freeze the RViz vehicle upright, with nothing anywhere saying why. Owning
        the subscription here decouples the heading from the route table
        completely (and pools with any route on the same topic and QoS, so it
        usually costs nothing).
        """
        try:
            from px4_msgs.msg import VehicleAttitude
        except ImportError:
            self.state.log.add(
                WARN, "attitude", "px4_msgs has no VehicleAttitude; heading unavailable"
            )
            return

        topic = self.settings.sim_topics.get("attitude", "/fmu/out/vehicle_attitude")

        def on_attitude(msg):
            quaternion = getattr(msg, "q", None)
            if quaternion is None or len(quaternion) < 4:
                return
            roll, pitch, yaw = cv.quat_to_euler(quaternion)
            now = time.time()
            self.shared["yaw"] = yaw
            self.shared["yaw_at"] = now
            self.shared["rpy"] = (roll, pitch, yaw)
            # The raw PX4 quaternion, kept unconverted so consumers apply their
            # own frame change once. sensor_combined_to_imu splices this into
            # the Imu message's orientation, which SensorCombined itself does
            # not carry but a real M4E's imu topic does.
            self.shared["q_ned_frd"] = tuple(float(v) for v in quaternion[:4])
            self.shared["q_ned_frd_at"] = now

        self._acquire_sub(topic, VehicleAttitude, "compat", "sim_attitude", on_attitude)
        self.state.log.add(DEBUG, "attitude", f"tracking simulated heading from {topic}")

    def _subscribe_host_attitude(self) -> None:
        """
        Track the REAL aircraft's heading, straight from the wrapper.

        gimbal_angles_to_joint_cmds needs it to turn DJI's ground-frame gimbal
        yaw into the body-relative angle the simulated pan joint must hold. It
        has to be the real aircraft's heading and not the simulator's: the two
        point different ways whenever the simulation is not flying the same track,
        and substituting the sim's heading would feed that whole difference into
        the pan joint.

        Not a route because it produces no output message -- it is shared state,
        like the PX4 clock, and an operator turning routes off should not change
        what the gimbal conversion means.
        """
        from geometry_msgs.msg import QuaternionStamped

        topic = f"{self.settings.wrapper_prefix.rstrip('/')}/attitude"

        def on_attitude(msg):
            yaw = cv.quat_msg_to_yaw(getattr(msg, "quaternion", None))
            if yaw is None:
                return
            self.shared["host_yaw"] = yaw
            self.shared["host_yaw_at"] = time.time()

        self._acquire_sub(topic, QuaternionStamped, "compat", "host_attitude", on_attitude)
        self.state.log.add(DEBUG, "attitude", f"tracking aircraft heading from {topic}")

    def _subscribe_gimbal_joints(self) -> None:
        """
        Track the simulated gimbal's three joint setpoints in shared state.

        These are the topics drone_controller.py publishes when the operator
        drags the pan/roll/tilt sliders, and the same ones ros_gz_bridge forwards
        into Gazebo's JointPositionControllers (config/bridge.yaml). They are
        therefore the only sim-side record of where the simulated gimbal is
        pointing: bridge.yaml maps ROS -> Gz for these topics and nothing maps
        joint state back, so there is no /joint_states to read instead.

        gimbal_joints_to_gimbal_angles needs all three to build one
        Vector3Stamped, but a route only ever has one source topic. Pan is the
        route's source; roll and tilt are picked up here, the same way the PX4
        clock and the two headings are, so turning routes on and off cannot
        change what the reverse gimbal conversion means.
        """
        from std_msgs.msg import Float64

        for axis in ("pan", "roll", "tilt"):
            topic = f"/drone/gimbal/cmd/{axis}"

            def on_value(msg, axis=axis):
                value = getattr(msg, "data", None)
                if value is None:
                    return
                self.shared[f"gimbal_joint_{axis}"] = float(value)
                self.shared[f"gimbal_joint_{axis}_at"] = time.time()

            self._acquire_sub(topic, Float64, "compat", f"gimbal_{axis}", on_value)
        self.state.log.add(
            DEBUG, "gimbal", "tracking /drone/gimbal/cmd/{pan,roll,tilt} joint setpoints"
        )

    # ── arming preconditions ─────────────────────────────────────────────────

    def arming_blockers(self) -> List[str]:
        """
        PX4's own reasons that arming would be refused, freshest first. Empty when
        the flags say nothing is wrong -- or when they have not arrived, which is
        itself reported as a blocker so an operator is never told "all clear" on
        the strength of no data at all.
        """
        seen_at = float(self.shared.get("arm_blockers_at", 0.0))
        if seen_at == 0.0:
            return ["no failsafe_flags from PX4 (is /fmu/out/failsafe_flags there?)"]
        if (time.time() - seen_at) > 5.0:
            return [f"failsafe_flags stale by {time.time() - seen_at:.0f}s"]
        blockers = list(self.shared.get("arm_blockers", []))
        if self.shared.get("preflight_ok") is False and not blockers:
            blockers.append("pre_flight_checks_pass is false")
        return blockers

    def active_sink_topics(self) -> Set[str]:
        """Topics this bridge currently publishes. Used by the mock to stay clear."""
        with self._lock:
            return {key.split("|")[0] for key in self._pubs}

    def served_service_names(self) -> Set[str]:
        """
        Relative service names this bridge is answering right now, e.g.
        {"takeoff", "camera_get_type"}. The wrapper stand-in subtracts these
        from the surface it stubs, so one name never has two servers.
        """
        with self._lock:
            return {
                service.service
                for service in self.service_routes
                if service.enabled
                and service.available
                and service.role is ServiceRole.SERVE
            }

    # ── route realisation ────────────────────────────────────────────────────

    def _realise_all_routes(self) -> None:
        for route in self.routes:
            self._realise_route(route)

    def _teardown_route(self, route: TopicRoute) -> None:
        self._release_sub(route.key)
        self._release_pubs(route.key)
        self._last_pass.pop(route.key, None)

    def _realise_route(self, route: TopicRoute) -> None:
        """(Re)create the endpoints for one route to match its current config."""
        with self._lock:
            self._teardown_route(route)

            if not route.enabled or route.direction is Direction.OFF:
                return
            if not route.available:
                self.state.log.add(
                    WARN, "route", f"{route.key}: message type unavailable, skipped"
                )
                return

            # Sink publisher(s) first, so the callback always finds them.
            sink_keys: Dict[str, str] = {}
            key, _ = self._acquire_pub(
                route.sink_topic, route.sink_type, route.pub_qos, route.key
            )
            sink_keys[route.sink_topic] = key
            for topic, msg_type in route.fanout:
                extra_key, _ = self._acquire_pub(topic, msg_type, route.pub_qos, route.key)
                sink_keys[topic] = extra_key

            self._acquire_sub(
                route.source_topic,
                route.source_type,
                route.sub_qos,
                route.key,
                self._make_route_handler(route, sink_keys),
            )

    def _make_route_handler(self, route: TopicRoute, sink_keys: Dict[str, str]):
        stats = self.state.stats(route.key)
        is_from_sim = route.direction is Direction.SIM_TO_HOST

        # One context per route, sharing the same params/shared dicts. The
        # executor is multi-threaded with a reentrant callback group, so two
        # converters really do run at once -- a single shared context whose
        # `route` field is rewritten before each call would mis-attribute
        # warnings under load.
        ctx = cv.ConversionContext(
            clock=self.clock_px4,
            params=self.settings.params,
            shared=self.shared,
            sim_clock=self.clock_sim,
            log=lambda source, message: self.state.log.add(DEBUG, source, message),
            journal=self.journal,
            route=route.key,
            direction=route.direction.value,
        )

        def handle(msg):
            now = time.monotonic()

            # PX4 messages carry the simulated clock; learn from every one.
            if is_from_sim:
                timestamp = getattr(msg, "timestamp", None)
                if timestamp is not None:
                    self.clock_px4.feed(timestamp)
                    stats.latency_ms = self.clock_px4.latency_ms(timestamp)

            stats.note_rx(now)

            if route.max_hz > 0.0:
                # DEADLINE, NOT "TIME SINCE LAST".
                #
                # The obvious test -- drop if `now - last < 1/max_hz` -- is
                # wrong whenever the source rate is close to the cap, and it
                # fails in the most misleading direction: it UNDER-delivers.
                # A 50 Hz source under a 50 Hz cap does not arrive on a perfect
                # 20 ms grid; ordinary jitter puts perhaps a third of samples a
                # fraction early, each of those is dropped, and the measured
                # output settles around 33 Hz. Observed exactly that here:
                # attitude fell from 51.7 Hz to 33.8 Hz the moment a 50 Hz cap
                # was applied to a 50 Hz stream.
                #
                # Tracking the next DUE time and advancing it by one whole
                # period lets an early sample through and keeps the long-run
                # average at the cap. The max() clamp stops a burst after a
                # quiet spell from being released all at once, which is what
                # pure period-advancing would do.
                period = 1.0 / route.max_hz
                due = self._next_due.get(route.key, 0.0)
                if now < due:
                    stats.dropped_rate += 1
                    return
                self._next_due[route.key] = max(due + period, now)
            self._last_pass[route.key] = now

            if self.settings.show_live_data:
                stats.last_sample = cv.summarise(msg)
            if self.settings.log_rx_traffic:
                self.state.log.add(
                    DEBUG, route.key, f"{route.source_topic} {cv.summarise(msg)}"
                )

            try:
                produced = route.converter(msg, ctx)
            except Exception as exc:
                stats.dropped_error += 1
                stats.last_error = f"{type(exc).__name__}: {exc}"
                self.journal.report(
                    route.key, route.direction.value, "converter_raised", stats.last_error
                )
                if self.inspect_key == route.key:
                    self._capture_inspect(
                        route, stats, msg, None,
                        note=f"converter raised {type(exc).__name__}: {exc}",
                    )
                return

            if self.inspect_key == route.key:
                self._capture_inspect(route, stats, msg, produced)

            if produced is None:
                return

            try:
                if isinstance(produced, list):
                    for topic, out_msg in produced:
                        publisher = self._publisher_for(sink_keys.get(topic, ""))
                        if publisher is None:
                            stats.last_error = f"no publisher for fan-out topic {topic}"
                            self.journal.report(
                                route.key, route.direction.value, "sink_missing", topic
                            )
                            continue
                        publisher.publish(out_msg)
                    stats.note_tx()
                else:
                    publisher = self._publisher_for(sink_keys.get(route.sink_topic, ""))
                    if publisher is None:
                        stats.last_error = "sink publisher missing"
                        self.journal.report(
                            route.key, route.direction.value, "sink_missing", route.sink_topic
                        )
                        return
                    publisher.publish(produced)
                    stats.note_tx()
                stats.last_error = ""
            except Exception as exc:
                stats.dropped_error += 1
                stats.last_error = f"publish: {type(exc).__name__}: {exc}"
                self.journal.report(
                    route.key, route.direction.value, "publish_failed", stats.last_error
                )

            # Liveness of the control stream, and ONLY of the route that is
            # actually steering. rc_passthrough used to count too, which meant a
            # Manifold (or the mock) publishing sticks the bridge was no longer
            # converting still looked like live control: the offboard heartbeat
            # ran, auto_offboard put PX4 into OFFBOARD, and nothing sent it a
            # setpoint. A disarmed aircraft then refuses to arm, because OFFBOARD
            # requires an offboard signal it is not getting -- which is a very
            # long way from "the display route is receiving".
            if (
                route.key in SETPOINT_ROUTES
                and route.enabled
                and route.direction is Direction.HOST_TO_SIM
            ):
                self.shared["last_rc_time"] = time.time()

        return handle

    def _capture_inspect(self, route, stats, incoming, produced, note: str = "") -> None:
        """
        Record one before/after pair for the INSPECT screen.

        Only ever called for the single route being inspected (see
        `inspect_key`), because exploding a message costs far more than
        converting it -- and on a 200 Hz route that cost would land squarely in
        the bridging path.

        Never raises: a failure to render a debug view must not drop a message
        that was already converted successfully.
        """
        try:
            stats.sample_in = tuple(inspect_mod.explode(incoming))
            stats.sample_in_type = inspect_mod.summarise_type(incoming)

            if isinstance(produced, list):
                # Fan-out converters emit [(topic, msg), ...]. EVERY output has
                # to be shown: on the gimbal routes the three outputs are pan,
                # roll and tilt, and showing only the first made the conversion
                # look as though it read one input field and ignored the rest.
                if produced:
                    fields = []
                    for topic, out_msg in produced:
                        leaf = topic.rsplit("/", 1)[-1]
                        exploded = inspect_mod.explode(out_msg)
                        # A Float64 joint setpoint explodes to a single field
                        # called "data", which is useless as a label and pairs
                        # with nothing. Name the row after the topic instead
                        # ("pan", "roll", "tilt") so the axis mapping lines up
                        # against the source's x/y/z.
                        if len(exploded) == 1 and exploded[0][0] == "data":
                            fields.append((leaf, exploded[0][1]))
                        else:
                            for path, value in exploded:
                                fields.append((f"{leaf}.{path}", value))
                    stats.sample_out = tuple(fields)
                    stats.sample_out_type = " + ".join(
                        sorted({inspect_mod.summarise_type(m) for _t, m in produced})
                    )
                    stats.sample_out_topic = ", ".join(t for t, _m in produced)
                    note = note or (
                        f"fan-out: {len(produced)} topics, each row prefixed with its "
                        "topic's last segment"
                    )
                else:
                    stats.sample_out = ()
                    stats.sample_out_type = "-"
                    stats.sample_out_topic = route.sink_topic
                    note = note or "converter returned an empty fan-out list"
            else:
                if produced is None and not note:
                    note = (
                        "converter returned None -- this message was deliberately "
                        "dropped"
                    )
                stats.sample_out = tuple(inspect_mod.explode(produced))
                stats.sample_out_type = inspect_mod.summarise_type(produced)
                stats.sample_out_topic = route.sink_topic

            stats.sample_note = note
            stats.sample_when = time.time()
        except Exception:
            pass

    def set_inspect(self, key: str) -> None:
        """Point the field-level capture at one route, or "" to switch it off."""
        self.inspect_key = key or ""

    # ── live reconfiguration (called via submit()) ────────────────────────────

    def route_by_key(self, key: str) -> Optional[TopicRoute]:
        for route in self.routes:
            if route.key == key:
                return route
        return None

    def set_route_enabled(self, key: str, enabled: bool) -> None:
        route = self.route_by_key(key)
        if route is None:
            return
        if route.enabled == enabled:
            # No-op, and it matters that it stays one: this is called in bulk by
            # project testing mode, and re-realising a route that is already
            # right means tearing down healthy endpoints for nothing.
            return
        route.enabled = enabled
        self._realise_route(route)
        self.settings.override(key, **route.as_override())
        self.state.log.add(INFO, "route", f"{key} {'enabled' if enabled else 'disabled'}")
        if enabled:
            self._enforce_single_setpoint_writer(key)

    def toggle_route(self, key: str) -> None:
        route = self.route_by_key(key)
        if route is not None:
            self.set_route_enabled(key, not route.enabled)

    def active_setpoint_route(self) -> Optional[TopicRoute]:
        """The route currently writing /fmu/in/trajectory_setpoint, if any."""
        for key in SETPOINT_ROUTES:
            route = self.route_by_key(key)
            if (
                route is not None
                and route.enabled
                and route.available
                and route.direction is Direction.HOST_TO_SIM
            ):
                return route
        return None

    def _enforce_single_setpoint_writer(self, winner_key: str) -> None:
        """
        One writer on /fmu/in/trajectory_setpoint, always.

        rc_setpoint and the three psdk_*_setpoint routes all publish there, and
        two of them running at once does not blend: PX4 acts on whichever sample
        arrived last, so the aircraft alternates between two intentions at the
        rate the slower one publishes. That is a genuinely dangerous failure and
        an almost impossible one to read from the outside, so enabling one of
        them switches the others off and says so in the log.

        Silently, rather than refusing, because the operator's last action is the
        one they meant -- and a refusal would leave them staring at a route that
        will not turn on with no obvious reason why.
        """
        if self._arbitrating:
            return
        winner = self.route_by_key(winner_key)
        if winner is None or winner_key not in SETPOINT_ROUTES:
            return
        if not (winner.enabled and winner.direction is Direction.HOST_TO_SIM):
            return

        losers = [
            route
            for key in SETPOINT_ROUTES
            for route in [self.route_by_key(key)]
            if route is not None
            and route.key != winner_key
            and route.enabled
            and route.direction is Direction.HOST_TO_SIM
        ]
        if not losers:
            return

        self._arbitrating = True
        try:
            for route in losers:
                self.set_route_direction(route.key, Direction.OFF)
                self.state.log.add(
                    WARN,
                    "control",
                    f"{route.key} switched off: {winner_key} now owns "
                    f"{winner.sim_topic} (one writer only)",
                )
        finally:
            self._arbitrating = False

    def set_route_direction(self, key: str, direction: Direction) -> None:
        """
        Point one conversion explicitly: wrapper->sim, sim->host, or neither.

        Distinct from cycle_route_direction because "set it to sim->host" and
        "advance it one step" are different intentions, and only the first one
        is safe to expose to a GUI or a script -- a cycling control has to be
        pressed a variable number of times to reach a known state, which is
        exactly the kind of thing that gets pressed once too often.
        """
        route = self.route_by_key(key)
        if route is None:
            return
        if route.direction is direction:
            return
        was = route.direction
        route.direction = direction
        self._realise_route(route)
        self.settings.override(key, **route.as_override())
        if direction is Direction.OFF:
            self.state.log.add(
                INFO, "route", f"{key} conversion OFF (was {was.value}); endpoints released"
            )
        else:
            self.state.log.add(
                INFO,
                "route",
                f"{key} direction -> {direction.value}: "
                f"{route.source_topic} -> {route.sink_topic}",
            )
            self._enforce_single_setpoint_writer(key)

    def cycle_route_direction(self, key: str) -> None:
        route = self.route_by_key(key)
        if route is None:
            return
        self.set_route_direction(key, route.direction.cycle())

    def set_all_directions(self, direction: Direction, group: Optional[str] = None) -> int:
        """
        Point every route (or every route in one group) the same way. Routes whose
        type is unavailable are skipped -- they cannot be realised either way.
        Returns how many were changed.
        """
        changed = 0
        for route in self.routes:
            if group and route.group != group:
                continue
            if not route.available or route.direction is direction:
                continue
            self.set_route_direction(route.key, direction)
            changed += 1
        scope = group or "all"
        self.state.log.add(
            INFO, "route", f"{scope}: {changed} route(s) -> {direction.value}"
        )
        return changed

    def set_route_qos(self, key: str, side: str, preset: str) -> None:
        route = self.route_by_key(key)
        if route is None or preset not in qos_mod.PRESETS:
            return
        if side == "sub":
            route.sub_qos = preset
        else:
            route.pub_qos = preset
        self._realise_route(route)
        self.settings.override(key, **route.as_override())
        self.state.log.add(INFO, "route", f"{key} {side} QoS -> {preset}")

    def cycle_route_qos(self, key: str, side: str = "sub") -> None:
        route = self.route_by_key(key)
        if route is None:
            return
        current = route.sub_qos if side == "sub" else route.pub_qos
        self.set_route_qos(key, side, qos_mod.cycle(current))

    def set_route_rate(self, key: str, hz: float) -> None:
        route = self.route_by_key(key)
        if route is None:
            return
        route.max_hz = max(0.0, float(hz))
        self.settings.override(key, **route.as_override())
        cap = "unlimited" if route.max_hz == 0.0 else f"{route.max_hz:g} Hz"
        self.state.log.add(INFO, "route", f"{key} rate cap -> {cap}")

    def nudge_route_rate(self, key: str, up: bool) -> None:
        route = self.route_by_key(key)
        if route is None:
            return
        steps = [0.0, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 50.0, 100.0, 200.0]
        current = route.max_hz
        if up:
            nxt = next((s for s in steps if s > current), steps[-1])
        else:
            lower = [s for s in steps if s < current]
            nxt = lower[-1] if lower else 0.0
        self.set_route_rate(key, nxt)

    def set_all_routes(self, enabled: bool, group: Optional[str] = None) -> None:
        for route in self.routes:
            if group and route.group != group:
                continue
            route.enabled = enabled
            self._realise_route(route)
            self.settings.override(route.key, **route.as_override())
        scope = group or "all"
        self.state.log.add(
            INFO, "route", f"{scope} routes {'enabled' if enabled else 'disabled'}"
        )

    # ── services ─────────────────────────────────────────────────────────────

    def _service_name(self, service: ServiceRoute) -> str:
        return f"{self.settings.wrapper_prefix.rstrip('/')}/{service.service}"

    def _realise_all_services(self) -> None:
        for service in self.service_routes:
            self._realise_service(service)

    def _teardown_service(self, service: ServiceRoute) -> None:
        server = self._servers.pop(service.key, None)
        if server is not None:
            try:
                self.destroy_service(server)
            except Exception:
                pass
        client = self._clients.pop(service.key, None)
        if client is not None:
            try:
                self.destroy_client(client)
            except Exception:
                pass

    def _realise_service(self, service: ServiceRoute) -> None:
        with self._lock:
            self._teardown_service(service)
            if not service.enabled or service.role is ServiceRole.OFF:
                return
            if not service.available:
                self.state.log.add(
                    WARN, "service", f"{service.key}: srv type unavailable, skipped"
                )
                return

            name = self._service_name(service)
            if service.role is ServiceRole.SERVE:
                server = self.create_service(
                    service.srv_type,
                    name,
                    self._make_service_handler(service),
                    callback_group=self._cb_group,
                )
                self._servers[service.key] = server
                self._introspect_service(server, name)
                self.state.log.add(DEBUG, "service", f"serving {name}")
            else:  # PROXY
                self._clients[service.key] = self.create_client(
                    service.srv_type, name, callback_group=self._cb_group
                )
                self.state.log.add(DEBUG, "service", f"client for {name}")

    # SERVICE CALLS ARE INVISIBLE TO rosbag2 UNLESS THE SERVER PUBLISHES THEM.
    # A recording of a flight is a recording of what made it happen, and for
    # this surface that is half topics and half service calls: takeoff, land,
    # obtain_ctrl_authority. ROS 2 exposes those to a recorder through service
    # introspection -- with it on, the server also publishes every request and
    # response on <service>/_service_event, which is what
    # `ros2 bag record` stores and what `ros2 bag play
    # --publish-service-requests` sends back as real calls. Without it,
    # ./run.sh replay can re-fly the setpoints of a sortie but never the
    # takeoff that started it.
    #
    # CONTENTS (the default here) carries the payload as well as the metadata.
    # The flight services are std_srvs/Trigger -- an empty request and a bool
    # plus a string back -- and they are called a handful of times per sortie,
    # so the cost is nil. SIMTY_SERVICE_INTROSPECTION=metadata|off changes or
    # disables it. Failure is never fatal: an rclpy without introspection
    # should cost the bridge nothing but a debug line.
    def _introspect_service(self, server, name: str) -> None:
        wanted = os.environ.get("SIMTY_SERVICE_INTROSPECTION", "contents").lower()
        if wanted in ("off", "0", "none", "false"):
            return
        try:
            from rclpy.service_introspection import ServiceIntrospectionState
            from rclpy.qos import qos_profile_system_default

            state = (
                ServiceIntrospectionState.METADATA
                if wanted in ("metadata", "meta")
                else ServiceIntrospectionState.CONTENTS
            )
            server.configure_introspection(
                self.get_clock(), qos_profile_system_default, state
            )
        except Exception as exc:  # noqa: BLE001 - never break serving over this
            self.state.log.add(
                DEBUG,
                "service",
                f"{name}: no introspection ({type(exc).__name__}: {exc})",
            )

    def _make_service_handler(self, service: ServiceRoute):
        def handle(request, response):
            try:
                response = service.handler(request, response, self)
                self.state.note_service(service.key, True, "ok")
                self.state.log.add(
                    INFO, "service", f"{self._service_name(service)} handled"
                )
            except Exception as exc:
                detail = f"{type(exc).__name__}: {exc}"
                self.state.note_service(service.key, False, detail)
                self.journal.report(
                    f"srv:{service.key}", "host->sim", "service_failed", detail
                )
                if hasattr(response, "success"):
                    response.success = False
            return response

        return handle

    def service_by_key(self, key: str) -> Optional[ServiceRoute]:
        for service in self.service_routes:
            if service.key == key:
                return service
        return None

    def service_name(self, service: ServiceRoute) -> str:
        """Full DDS name of a bridged service. Public for the panels."""
        return self._service_name(service)

    def service_client(self, key: str):
        """The PROXY-role client for one service, or None."""
        with self._lock:
            return self._clients.get(key)

    def set_service_enabled(self, key: str, enabled: bool) -> None:
        service = self.service_by_key(key)
        if service is None or service.enabled == enabled:
            return
        service.enabled = enabled
        self._realise_service(service)
        self.settings.override(f"srv:{key}", **service.as_override())
        self.state.log.add(
            INFO, "service", f"{key} {'enabled' if enabled else 'disabled'}"
        )

    def toggle_service(self, key: str) -> None:
        service = self.service_by_key(key)
        if service is not None:
            self.set_service_enabled(key, not service.enabled)

    def set_service_role(self, key: str, role: ServiceRole) -> None:
        """Explicit counterpart to cycle_service_role, for GUIs and scripts."""
        service = self.service_by_key(key)
        if service is None or service.role is role:
            return
        service.role = role
        self._realise_service(service)
        self.settings.override(f"srv:{key}", **service.as_override())
        self.state.log.add(INFO, "service", f"{key} role -> {role.value}")

    def cycle_service_role(self, key: str) -> None:
        service = self.service_by_key(key)
        if service is None:
            return
        self.set_service_role(key, service.role.cycle())

    def set_all_services(self, enabled: bool) -> None:
        for service in self.service_routes:
            service.enabled = enabled
            self._realise_service(service)
            self.settings.override(f"srv:{service.key}", **service.as_override())
        self.state.log.add(
            INFO, "service", f"all services {'enabled' if enabled else 'disabled'}"
        )

    # ── simulator actuation, used by service handlers and the TUI ─────────────

    def publish_sim(self, topic: str, message, msg_type) -> None:
        """Publish onto a sim topic, creating a long-lived publisher on demand."""
        owner = f"adhoc:{topic}"
        _, publisher = self._acquire_pub(topic, msg_type, "reliable", owner)
        publisher.publish(message)

    def send_vehicle_command(self, name: str, **params) -> None:
        """
        Publish a PX4 VehicleCommand. `name` is the MAV_CMD without the
        VEHICLE_CMD_ prefix, e.g. "NAV_TAKEOFF".
        """
        try:
            from px4_msgs.msg import VehicleCommand
        except ImportError:
            self.state.log.add(ERROR, "command", "px4_msgs unavailable")
            return

        command = getattr(VehicleCommand, f"VEHICLE_CMD_{name}", None)
        if command is None:
            command = _CMD_FALLBACK.get(name)
        if command is None:
            self.state.log.add(ERROR, "command", f"unknown MAV_CMD {name}")
            return

        message = VehicleCommand()
        message.timestamp = self.clock_px4.now_us()
        message.command = int(command)
        for index in range(1, 8):
            setattr(message, f"param{index}", float(params.get(f"param{index}", 0.0)))
        message.target_system = int(params.get("target_system", 1))
        message.target_component = int(params.get("target_component", 1))
        message.source_system = int(params.get("source_system", 1))
        message.source_component = int(params.get("source_component", 1))
        message.from_external = True

        self.publish_sim(
            self.settings.sim_topics["vehicle_command"], message, VehicleCommand
        )
        self.state.log.add(INFO, "command", f"VehicleCommand {name} {params or ''}".strip())

    def set_rc_authority(self, granted: bool, source: str = "operator") -> None:
        self.shared["rc_authority"] = granted
        self.publish_sim("/drone/rc/authority", Bool(data=granted), Bool)
        self.state.log.add(
            INFO, "authority", f"RC authority {'granted' if granted else 'released'} ({source})"
        )

    def engage_offboard(self) -> None:
        self.send_vehicle_command(
            "DO_SET_MODE", param1=1.0, param2=_PX4_MAIN_MODE_OFFBOARD
        )
        self._offboard_engaged = True

    # PX4 custom main modes for DO_SET_MODE (param2), with AUTO's sub-mode in
    # param3. Only the two this bridge ever needs are named.
    _PX4_MAIN_MODE_AUTO = 4.0
    _PX4_SUB_MODE_LOITER = 3.0

    def _clear_prevent_arming_mode(self) -> bool:
        """
        Get out of a flight mode that refuses to arm, so an arm request can land.

        After a landing PX4 stays in AUTO_LAND, and AUTO_LAND is one of the
        nav_states in `mode_req_prevent_arming` -- so the next takeoff is
        refused, with no message a DJI client can see: FlightStatus just keeps
        saying STOPPED. It looks exactly like a broken bridge, and it is the one
        thing a wrapper-only client cannot fix for itself, because the PSDK
        surface has no "set mode".

        So the bridge does it: when the vehicle is disarmed in a mode that
        prevents arming, ask for AUTO_LOITER first. The mask comes from PX4
        itself (failsafe_flags.mode_req_prevent_arming) rather than a hard-coded
        list of mode numbers, so a PX4 release that changes the rule changes this
        behaviour with it.

        Deliberately narrow: modes that do NOT prevent arming are left alone.
        OFFBOARD is one of them, and the GUI's Pilot Mode arms from offboard --
        clearing that would break it.
        """
        nav_state = self.shared.get("nav_state")
        if nav_state is None:
            return False
        nav_state = int(nav_state)

        prevents = (int(self.shared.get("mode_req_prevent_arming", 0)) >> nav_state) & 1
        # The second case: a mode whose requirement cannot currently be met.
        # OFFBOARD is the one that bites -- PX4 leaves the aircraft in it after a
        # mission ends, and arming there is refused while no setpoints arrive.
        needs_offboard = (int(self.shared.get("mode_req_offboard_signal", 0)) >> nav_state) & 1
        starved = bool(needs_offboard and self.shared.get("offboard_signal_lost", False))
        if not (prevents or starved):
            return False
        reason = "refuses arming" if prevents else "needs an offboard signal it is not getting"

        self.send_vehicle_command(
            "DO_SET_MODE",
            param1=1.0,  # custom mode
            param2=self._PX4_MAIN_MODE_AUTO,
            param3=self._PX4_SUB_MODE_LOITER,
        )
        self.state.log.add(
            INFO,
            "command",
            f"nav_state {nav_state} {reason}; asked for AUTO_LOITER first",
        )
        self.journal.report(
            "command", "host->sim", "arm_mode_cleared",
            f"nav_state {nav_state} {reason}; sent DO_SET_MODE AUTO_LOITER before the arm",
        )
        return True

    def arm(self) -> None:
        """
        Ask PX4 to arm, and say in advance if it is going to refuse.

        The command is sent either way: the operator pressed the key, and PX4 is
        the authority on whether arming is allowed. What changes is that a refusal
        is no longer silent from this side -- the blockers are logged before the
        command and the outcome is checked afterwards.
        """
        blockers = self.arming_blockers()
        if blockers:
            self.journal.report(
                "command", "host->sim", "arm_blocked", ", ".join(blockers[:6])
            )
        # A mode that refuses arming has to be left first, and PX4 processes the
        # two commands in order, so both go out now. If the mode change has not
        # taken effect by the time the arm is evaluated, the caller's retry (the
        # console repeats on a keypress, the demos every two seconds) lands it.
        self._clear_prevent_arming_mode()
        self._arm_requested_at = time.monotonic()
        self.send_vehicle_command("COMPONENT_ARM_DISARM", param1=1.0)

    def disarm(self) -> None:
        self._arm_requested_at = 0.0
        self.send_vehicle_command("COMPONENT_ARM_DISARM", param1=0.0)

    def takeoff(self, altitude: float = TAKEOFF_ALTITUDE_M) -> None:
        """
        Arm, then NAV_TAKEOFF, with the altitude PX4 actually expects.

        THIS ARMS THE VEHICLE ITSELF, and that is a deliberate change. DJI's
        takeoff is one call that spins the motors and lifts off; a payload
        written against a real M4E calls takeoff and nothing else. PX4 here
        does not auto-arm on NAV_TAKEOFF, so the old behaviour was that
        takeoff was accepted, nothing moved, and the only way to find out was
        to know to call turn_on_motors first -- which no DJI payload does.
        So the arm goes out here, immediately before the takeoff, and the two
        are processed in order.

        turn_on_motors still exists and still works; it is simply no longer
        something a caller has to know about.

        MAV_CMD_NAV_TAKEOFF's param7 is altitude *AMSL*, and param5/param6 are the
        takeoff latitude and longitude. This used to send param7=5 and zeros for
        the rest, which asks PX4 to take off to 5 m above mean sea level at
        latitude 0, longitude 0 -- and since this world's home is 8 m AMSL
        (PX4_HOME_ALT=8, Karachi), 5 m AMSL is three metres *underground*.
        Commander rejected it and nothing happened.

        So: `altitude` is metres above the current position, converted to AMSL
        using the global position the bridge already receives, and lat/lon/yaw go
        out as NaN, which is how MAVLink spells "keep the current one".
        """
        nan = float("nan")
        self.shared["takeoff_alt"] = float(altitude)
        current = self.shared.get("alt")
        try:
            amsl = float(current) + float(altitude)
        except (TypeError, ValueError):
            amsl = nan
        if amsl != amsl:  # NaN: PX4 falls back to MIS_TAKEOFF_ALT above home
            self.journal.report(
                "command", "host->sim", "takeoff_alt_unknown",
                f"requested +{altitude:g} m but no global altitude has arrived",
            )

        blockers = self.arming_blockers()
        if blockers:
            self.journal.report(
                "command", "host->sim", "arm_blocked",
                "takeoff implies arming: " + ", ".join(blockers[:6]),
            )

        # Order matters and PX4 honours it: leave any mode that refuses arming,
        # arm, then take off. All three go out now rather than being spread
        # over a timer, because PX4 queues VehicleCommands and evaluates them
        # in the order they arrive -- and a takeoff that arrives before the arm
        # has been evaluated is simply rejected, which is the failure this
        # method exists to remove.
        self._clear_prevent_arming_mode()
        self._arm_requested_at = time.monotonic()
        self.send_vehicle_command("COMPONENT_ARM_DISARM", param1=1.0)
        self.send_vehicle_command(
            "NAV_TAKEOFF",
            param1=nan,  # minimum pitch: unchanged
            param4=nan,  # yaw: keep the current heading
            param5=nan,  # latitude: take off where we are
            param6=nan,  # longitude
            param7=amsl,  # altitude AMSL, not relative
        )
        self.state.log.add(
            INFO,
            "command",
            f"arm + takeoff to +{altitude:g} m"
            + (
                f" (= {amsl:.1f} m AMSL from {float(current):.1f})"
                if amsl == amsl
                else " (altitude unknown; PX4 will use MIS_TAKEOFF_ALT)"
            ),
        )

    def land(self) -> None:
        self.send_vehicle_command("NAV_LAND")

    def return_to_launch(self) -> None:
        self.send_vehicle_command("NAV_RETURN_TO_LAUNCH")

    # ── periodic work ────────────────────────────────────────────────────────

    def _rc_is_live(self) -> bool:
        last = float(self.shared.get("last_rc_time", 0.0))
        if last == 0.0:
            return False
        return (time.time() - last) < max(0.05, self.settings.rc_timeout_s)

    def _offboard_tick(self) -> None:
        """
        Offboard heartbeat.

        PX4 will not accept offboard mode, and will fall out of it, unless an
        OffboardControlMode stream arrives faster than 2 Hz. A trajectory
        setpoint alone is not enough -- which is why RC control never actually
        flew the vehicle before.
        """
        route = self.active_setpoint_route()
        if route is None:
            self._offboard_stream_since = 0.0
            return
        if not self._rc_is_live() or not bool(self.shared.get("rc_authority", False)):
            self._offboard_stream_since = 0.0
            self._offboard_engaged = False
            return

        try:
            from px4_msgs.msg import OffboardControlMode
        except ImportError:
            return

        # The flags must match the kind of setpoint the live route produces:
        # PX4 ignores a position setpoint unless the heartbeat says position,
        # and a velocity one unless it says velocity. Getting this wrong looks
        # like the aircraft ignoring perfectly good setpoints.
        position, velocity = SETPOINT_ROUTES.get(route.key, (False, True))
        heartbeat = OffboardControlMode()
        heartbeat.timestamp = self.clock_px4.now_us()
        heartbeat.position = position
        heartbeat.velocity = velocity
        heartbeat.acceleration = False
        heartbeat.attitude = False
        heartbeat.body_rate = False
        self.publish_sim(
            self.settings.sim_topics["offboard_control_mode"], heartbeat, OffboardControlMode
        )

        now = time.monotonic()
        if self._offboard_stream_since == 0.0:
            self._offboard_stream_since = now
            self.state.log.add(
                INFO,
                "offboard",
                f"heartbeat started for {route.key} "
                f"({'position' if position else 'velocity'} setpoints, authority held)",
            )
            return

        # PX4 wants the stream established before the mode switch.
        if (
            self.settings.auto_offboard
            and not self._offboard_engaged
            and (now - self._offboard_stream_since) > 1.5
        ):
            self.engage_offboard()
            self.state.log.add(INFO, "offboard", "auto-engaged OFFBOARD")
            if self.settings.auto_arm:
                self.arm()
                self.state.log.add(WARN, "offboard", "auto-ARM issued")

    def _scan_tick(self) -> None:
        try:
            report = self.scanner.scan(self.routes, self.service_routes)
        except Exception as exc:
            self.state.log.add(ERROR, "discovery", f"{type(exc).__name__}: {exc}")
            return
        with self._lock:
            self._report = report

        if report.surface is not self._prev_surface:
            self._prev_surface = report.surface
            if report.surface is disc.SurfaceState.FOREIGN:
                self.state.log.add(
                    INFO,
                    "discovery",
                    f"MANIFOLD CONNECTED via multicast: {report.summary()}",
                )
            elif report.surface is disc.SurfaceState.OURS:
                self.state.log.add(
                    INFO, "discovery", f"synthetic Manifold only: {report.summary()}"
                )
            else:
                self.state.log.add(WARN, "discovery", "no /wrapper endpoints found")

        for name in report.conflicting_services:
            self.journal.report(
                "discovery", "", "service_collision",
                f"{name} already has a remote server -- switch that row to PROXY "
                "or disable it",
            )

        # Keep the journal's throttle in step with the setting; the SETTINGS
        # screen can change it while the bridge runs.
        self.journal.set_min_interval(self.settings.journal_min_interval_s)

    def _check_arm_outcome(self) -> None:
        """
        Did the arm command take? PX4 answers on its own log, not to us, so the
        only honest check is whether arming_state changed. Two seconds is long
        enough for commander to have decided and short enough to still be next to
        the keypress in the log.
        """
        if self._arm_requested_at == 0.0:
            return
        if bool(self.shared.get("armed", False)):
            self._arm_requested_at = 0.0
            self.state.log.add(INFO, "command", "vehicle is ARMED")
            return
        if (time.monotonic() - self._arm_requested_at) < 2.0:
            return
        self._arm_requested_at = 0.0
        blockers = self.arming_blockers()
        self.journal.report(
            "command", "host->sim", "arm_denied",
            ", ".join(blockers[:6]) if blockers
            else "PX4 gave no failsafe reason -- check its console output",
        )

    def _housekeeping_tick(self) -> None:
        self.state.decay_all(self.settings.stale_after_s)
        self._check_arm_outcome()
        self.state.set_flight(
            armed=bool(self.shared.get("armed", False)),
            nav_state=self.shared.get("nav_state", "-"),
            landed=self.shared.get("landed", None),
            battery=self.shared.get("battery_pct", None),
            lat=self.shared.get("lat", None),
            lon=self.shared.get("lon", None),
            alt=self.shared.get("alt", None),
            ned=self.shared.get("ned", None),
            yaw=self.shared.get("yaw", None),
            sats=self.shared.get("sats", None),
            rc_authority=bool(self.shared.get("rc_authority", False)),
            rc_live=self._rc_is_live(),
            offboard_stream=self._offboard_stream_since > 0.0,
            px4_clock=self._clock_summary(),
            preflight_ok=self.shared.get("preflight_ok", None),
            arm_blockers=self.arming_blockers(),
        )

    # ── project testing mode ─────────────────────────────────────────────────
    #
    # One switch that puts the bridge into the shape a project under test needs,
    # and one that puts it back.
    #
    # Getting there by hand is eight or nine separate acts on three different
    # screens -- point six telemetry routes sim->host, enable the setpoint route
    # the project publishes to, enable the services it calls, turn on
    # auto_offboard -- and every one of them is easy to forget. Forgetting any
    # single one fails the same way: the project runs, sees nothing or commands
    # nothing, and looks broken. So this is a mode, not a checklist.
    #
    # It is NOT "make everything live": it enables exactly the routes and
    # services the projects in demo/ use, records what each one was, and restores
    # that on the way out. Nothing here arms anything, and auto_arm is left
    # alone -- takeoff arms implicitly, which is enough for a mission to run and
    # still leaves "arm the aircraft" as its own explicit act.

    # Telemetry a project reads. Every one goes sim->host and enabled.
    PROJECT_TELEMETRY = (
        "flight_status",
        "display_mode",
        "flight_anomaly",
        "rc_connection",
        "battery",
        "position_fused",
        "velocity_ground_fused",
        "height_above_ground",
        "altitude_sea_level",
        "gps_position_fused",
        "home_position",
        "home_point",
        "home_point_altitude",
        "home_point_status",
        "land_detected",
        "attitude",
        # A localization project needs the raw fix and the inertial stream, not
        # just the fused position above: gps_position_fused is PX4's EKF output
        # and already contains the IMU, so fusing it is circular. Both are on by
        # default, but project mode should not depend on an operator not having
        # turned them off.
        "gps_position",
        "imu",
    )

    # Services a project calls. SERVE, because in project testing the simulation
    # is the aircraft -- a PROXY row would send the call to hardware instead.
    PROJECT_SERVICES = (
        "takeoff",
        "land",
        "return_home",
        "obtain_authority",
        "release_authority",
        "motors_on",
        "motors_off",
        "get_go_home_alt",
        "set_go_home_alt",
        "set_home_from_gps",
        # Obstacle avoidance is five directional pairs on the aircraft, not
        # one switch; registry.py explains why. A project that queries
        # avoidance at start-up wants them served.
        "set_oa_downwards_vo", "get_oa_downwards_vo",
        "set_oa_horizontal_vo", "get_oa_horizontal_vo",
        "set_oa_horizontal_radar", "get_oa_horizontal_radar",
        "set_oa_upwards_vo", "get_oa_upwards_vo",
        "set_oa_upwards_radar", "get_oa_upwards_radar",
    )

    # What a project may steer with. The default is the ENU velocity setpoint:
    # it is a real psdk_ros2 control topic (so a project written against it also
    # runs against hardware), and being a ground-frame command it needs no
    # heading, which removes the one piece of state a stick-driven project has
    # to guess at.
    PROJECT_SETPOINTS = {
        "velocity": "psdk_velocity_setpoint",
        "position": "psdk_position_setpoint",
        "body": "psdk_body_velocity_setpoint",
        "flu": "psdk_body_velocity_setpoint",
        "rc": "rc_setpoint",
    }

    def set_project_mode(self, enable: bool, setpoint: str = "velocity") -> str:
        """Turn project testing mode on or off. Returns a line for the operator."""
        if enable:
            return self._enter_project_mode(setpoint)
        return self._leave_project_mode()

    def _enter_project_mode(self, setpoint: str) -> str:
        wanted = self.PROJECT_SETPOINTS.get((setpoint or "velocity").lower())
        if wanted is None:
            return (
                f"unknown setpoint '{setpoint}'; use "
                f"{', '.join(sorted(set(self.PROJECT_SETPOINTS)))}"
            )

        # Restoring matters more than entering: an operator who flips this on to
        # try something must get their route table back, not a bridge that is
        # quietly wide open afterwards.
        #
        # Entering twice must therefore NOT re-snapshot: the second snapshot
        # would record the state this mode itself installed, and leaving would
        # then "restore" the bridge to project-testing configuration for ever.
        # Tools call this idempotently (demo/run_demo.sh sends it before every
        # run), so this is the normal path, not an edge case.
        first_entry = self._project_restore is None
        restore: Dict[str, Any] = self._project_restore or {
            "routes": {},
            "services": {},
            "auto_offboard": self.settings.auto_offboard,
        }

        # The wrapper's attitude is the one route that can do harm on real
        # hardware: node.py reads that topic for the REAL aircraft's heading
        # (the gimbal conversion needs it), so publishing the simulator's
        # attitude onto it while a Manifold is connected replaces the reference
        # with our own. Skip it in that case and say why.
        connected = self.report().surface is disc.SurfaceState.FOREIGN
        skipped: List[str] = []

        for key in self.PROJECT_TELEMETRY:
            route = self.route_by_key(key)
            if route is None or not route.available:
                continue
            if key == "attitude" and connected:
                skipped.append("attitude (a real Manifold owns it)")
                continue
            if first_entry:
                restore["routes"][key] = (route.enabled, route.direction)
            # Queued, not applied: one endpoint mutation per tick. Applying the
            # whole set inside this call is what used to wedge the node.
            self.submit_slow(
                lambda bridge, name=key: (
                    bridge.set_route_direction(name, Direction.SIM_TO_HOST),
                    bridge.set_route_enabled(name, True),
                )
            )

        control_route = self.route_by_key(wanted)
        if control_route is None or not control_route.available:
            return f"{wanted} is not available in this workspace"

        # Record ALL of the setpoint routes, not just the chosen one: enabling
        # it switches whichever was steering off, and a mode that cannot put
        # that one back is a mode that quietly breaks the operator's setup.
        if first_entry:
            for key in SETPOINT_ROUTES:
                route = self.route_by_key(key)
                if route is not None and key not in restore["routes"]:
                    restore["routes"][key] = (route.enabled, route.direction)

        self.submit_slow(
            lambda bridge, name=wanted: (
                bridge.set_route_direction(name, Direction.HOST_TO_SIM),
                # Switches the other writers off; see the single-writer rule.
                bridge.set_route_enabled(name, True),
            )
        )

        for key in self.PROJECT_SERVICES:
            service = self.service_by_key(key)
            if service is None or not service.available:
                continue
            if first_entry:
                restore["services"][key] = (service.enabled, service.role)
            self.submit_slow(
                lambda bridge, name=key: (
                    bridge.set_service_role(name, ServiceRole.SERVE),
                    bridge.set_service_enabled(name, True),
                )
            )

        # Without this the setpoints arrive, the heartbeat runs, and PX4 stays
        # in whatever mode it was in -- the single most confusing way for a
        # project to fail, because everything looks connected.
        self.settings.auto_offboard = True

        self._project_restore = restore
        self.project_mode = True
        telemetry_count = max(0, len(restore["routes"]) - len(SETPOINT_ROUTES))
        message = (
            f"project testing mode {'ON' if first_entry else 'RE-APPLIED'}: "
            f"{telemetry_count} telemetry routes sim->host, "
            f"{len(restore['services'])} services SERVE, {wanted} host->sim, "
            "auto_offboard on (applied one route per tick, ~2 s)"
        )
        if skipped:
            message += f" (skipped {', '.join(skipped)})"
        self.state.log.add(WARN, "project", message)
        return message

    def _leave_project_mode(self) -> str:
        if not self._project_restore:
            self.project_mode = False
            return "project testing mode was not on"
        restore = self._project_restore
        for key, (enabled, direction) in restore["routes"].items():
            self.submit_slow(
                lambda bridge, name=key, want=enabled, way=direction: (
                    bridge.set_route_direction(name, way),
                    bridge.set_route_enabled(name, want),
                )
            )
        for key, (enabled, role) in restore["services"].items():
            self.submit_slow(
                lambda bridge, name=key, want=enabled, which=role: (
                    bridge.set_service_role(name, which),
                    bridge.set_service_enabled(name, want),
                )
            )
        self.settings.auto_offboard = bool(restore.get("auto_offboard", False))
        self._project_restore = None
        self.project_mode = False
        message = "project testing mode OFF: routes, services and auto_offboard restored"
        self.state.log.add(INFO, "project", message)
        return message

    # ── bag replay mode ─────────────────────────────────────────────────────
    #
    # For `./run.sh replay`: a recorded flight is played back onto the wrapper
    # surface by `ros2 bag play`, and everything downstream -- a project under
    # test, an RViz panel, a perception node -- reads it as if a Manifold were
    # publishing it.
    #
    # WHY THE BRIDGE HAS TO GET OUT OF THE WAY
    # ----------------------------------------
    # The bag publishes /wrapper/psdk_ros2/*. Those are exactly the names the
    # bridge publishes in the sim->host direction. Two writers on one topic does
    # not fail, it INTERLEAVES: a subscriber gets alternating samples from the
    # bag and from the simulation, timestamps jump backwards, and every
    # consumer sees a flight that teleports. That is close enough to "working"
    # to be believed for a while, which is what makes it worth a mode of its
    # own rather than a note in the docs.
    #
    # So this turns every topic route OFF -- not just the telemetry ones. A bag
    # holds whatever was recorded, and the moment a route republishes a name the
    # bag also carries, the same interleaving is back.
    #
    # WHAT IT LEAVES ON, AND WHY THAT IS THE POINT
    # --------------------------------------------
    # The services. A bag cannot answer a service call: rosbag2 records service
    # traffic as events and can replay REQUESTS, but nothing in a bag is a
    # server. So a project replaying a flight can read telemetry all day and
    # then hang the first time it calls `takeoff`. The bridge stays as the
    # server for the flight services, exactly as in project testing mode.
    #
    # Those calls reach a simulation that replay mode does not start, so they
    # answer and nothing flies. That is the honest behaviour for a replay: the
    # past does not take instructions.
    REPLAY_SERVICES = PROJECT_SERVICES

    def set_replay_mode(self, enable: bool) -> str:
        """Turn bag replay mode on or off. Returns a line for the operator."""
        if enable:
            return self._enter_replay_mode()
        return self._leave_replay_mode()

    def _enter_replay_mode(self) -> str:
        # Project testing mode points routes at the simulation; replay mode
        # points them nowhere. Holding both would mean each one's restore
        # snapshot recording the other's work, so the second to be asked for
        # takes over and says so.
        if self.project_mode:
            self.state.log.add(
                WARN, "replay",
                "leaving project testing mode first: it publishes the same wrapper topics "
                "the bag does",
            )
            self._leave_project_mode()

        first_entry = self._replay_restore is None
        restore: Dict[str, Any] = self._replay_restore or {
            "routes": {},
            "services": {},
            "auto_offboard": self.settings.auto_offboard,
        }

        silenced = 0
        for route in self.routes:
            if not route.enabled:
                continue
            if first_entry:
                restore["routes"][route.key] = (route.enabled, route.direction)
            silenced += 1
            self.submit_slow(
                lambda bridge, name=route.key: bridge.set_route_enabled(name, False)
            )

        served = 0
        for key in self.REPLAY_SERVICES:
            service = self.service_by_key(key)
            if service is None or not service.available:
                continue
            if first_entry:
                restore["services"][key] = (service.enabled, service.role)
            served += 1
            self.submit_slow(
                lambda bridge, name=key: (
                    bridge.set_service_role(name, ServiceRole.SERVE),
                    bridge.set_service_enabled(name, True),
                )
            )

        # Nothing is steering during a replay, so the offboard heartbeat has
        # nothing to be a heartbeat for. Leaving auto-engage on would have the
        # bridge try to put a PX4 that is not running into OFFBOARD.
        self.settings.auto_offboard = False

        # The one conflict this mode CANNOT fix from here. The synthetic
        # Manifold is not a route -- it is a set of 27 real publishers on the
        # wrapper topics -- so switching routes off does not silence it, and it
        # will interleave with the bag sample for sample.
        #
        # Stopping it is deliberately not done here: standin_control and
        # mock_control hand their work to the executor and WAIT for it, and
        # this method already runs on that executor's command queue, so calling
        # either one from here would block until its own timeout. Reporting it
        # is honest and cannot deadlock; ./run.sh replay avoids the situation
        # by starting the bag before the bridge, and the console's MANIFOLD
        # screen (key T) stops it in one keystroke.
        # settings.mock_enabled is set by the launcher when it actually starts
        # one, which is a different question from discovery's MOCK
        # classification -- that only means "wrapper names exist with nothing
        # remote behind them", which is also what a gap between two players
        # looks like.
        if getattr(self.settings, "mock_enabled", False):
            self.state.log.add(
                WARN, "replay",
                "the synthetic Manifold is running: its ~27 publishers on "
                f"{self.settings.wrapper_prefix}/* will INTERLEAVE with the bag. "
                "Stop it on the MANIFOLD screen (key T) -- replay mode cannot, "
                "it is a fake aircraft rather than a route",
            )

        self._replay_restore = restore
        self.replay_mode = True
        message = (
            f"bag replay mode {'ON' if first_entry else 'RE-APPLIED'}: "
            f"{silenced} topic routes off (the bag owns {self.settings.wrapper_prefix}/*), "
            f"{served} services SERVE, auto_offboard off"
        )
        self.state.log.add(WARN, "replay", message)
        return message

    def _leave_replay_mode(self) -> str:
        if not self._replay_restore:
            self.replay_mode = False
            return "bag replay mode was not on"
        restore = self._replay_restore
        for key, (enabled, direction) in restore["routes"].items():
            self.submit_slow(
                lambda bridge, name=key, want=enabled, way=direction: (
                    bridge.set_route_direction(name, way),
                    bridge.set_route_enabled(name, want),
                )
            )
        for key, (enabled, role) in restore["services"].items():
            self.submit_slow(
                lambda bridge, name=key, want=enabled, which=role: (
                    bridge.set_service_role(name, which),
                    bridge.set_service_enabled(name, want),
                )
            )
        self.settings.auto_offboard = bool(restore.get("auto_offboard", False))
        self._replay_restore = None
        self.replay_mode = False
        message = "bag replay mode OFF: routes, services and auto_offboard restored"
        self.state.log.add(INFO, "replay", message)
        return message

    # ── accessors for the TUI ────────────────────────────────────────────────

    def report(self) -> disc.DiscoveryReport:
        with self._lock:
            return self._report

    def endpoint_counts(self) -> Tuple[int, int]:
        with self._lock:
            return len(self._subs), len(self._pubs)
