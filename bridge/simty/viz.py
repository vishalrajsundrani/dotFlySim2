"""
The RViz half of the control room.

WHY MARKERS AND NOT AN RVIZ PLUGIN
----------------------------------
The thing an operator needs to see is not a 3D scene -- it is *which conversion
is running, in which direction, how fast, and what it is complaining about*.
RViz has no display for that, and writing a C++ panel plugin would mean a
package, a build and a Dockerfile change. Marker text and arrows in a dedicated
TF frame give the same board with zero build steps and work in any RViz 2, so
the whole control room ships as a mounted Python file plus a .rviz config.

Three boards, each in its own frame so a saved RViz view can look straight at
one of them:

  simty_flow_board      one row per topic route: wrapper endpoint, an arrow in
                        the direction data is actually flowing (scaled by rate,
                        coloured by health), the sim endpoint, counters, and the
                        live conversion warnings for that row
  simty_service_board   one row per bridged service: name, SERVE/PROXY, call and
                        failure counts, collision flag
  simty_status_board    the link verdict, middleware facts, provisioning state,
                        and the tail of the conversion journal

plus the vehicle itself in `map`: pose from PositionFused, gimbal frame from
attitude, the downward LRF ray, home, the commanded RC velocity vector, and a
flown trail on /simty/viz/path.

SUBSCRIBER-GATED
----------------
Every publish is skipped when nothing is subscribed, the same trick the camera
bridges use with lazy:true. Leaving viz_enabled on therefore costs nothing while
RViz is closed, which is what makes it safe to default to on.

FRAME CONVENTIONS
-----------------
PX4 is NED/FRD, ROS and RViz are ENU/FLU. Position converts as
(east, north, up) = (y, x, -z) and orientation as (roll, -pitch, pi/2 - yaw).
Getting this wrong yields a drone that banks the wrong way and a heading arrow
mirrored about north -- easy to miss on a symmetric world, which is exactly why
it is written down here.
"""

import math
import os
from typing import List, Optional, Tuple

from builtin_interfaces.msg import Duration as RosDuration
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .registry import Direction, ServiceRole
from .state import DEBUG, ERROR, INFO, WARN

# ── palette ──────────────────────────────────────────────────────────────────

GREEN = (0.30, 0.85, 0.35)
YELLOW = (0.95, 0.80, 0.20)
RED = (0.92, 0.28, 0.24)
GREY = (0.45, 0.45, 0.48)
BLUE = (0.35, 0.65, 0.95)
WHITE = (0.90, 0.92, 0.95)
CYAN = (0.30, 0.85, 0.90)

LEVEL_COLOUR = {DEBUG: GREY, INFO: WHITE, WARN: YELLOW, ERROR: RED}

# board geometry, metres
ROW = 1.7
TEXT = 0.62
HEAD = 0.85
COL_HOST = 0.0
COL_ARROW_A = 15.0
COL_ARROW_B = 22.0
COL_SIM = 30.0
COL_STATS = 46.0
COL_WARN = 62.0

MAX_FLOW_WARNINGS = 2
MAX_JOURNAL_LINES = 14


def _colour(rgb, alpha: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(alpha))


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def _ned_to_enu(ned) -> Tuple[float, float, float]:
    north, east, down = float(ned[0]), float(ned[1]), float(ned[2])
    return east, north, -down


def _short(topic: str, width: int = 30) -> str:
    if len(topic) <= width:
        return topic
    return "..." + topic[-(width - 3):]


class ControlRoomViz:
    """
    Owns the marker publishers and the TF broadcaster. Created by SimtyBridge;
    publish() runs on the executor thread from a bridge timer, so it may read
    rclpy state freely but must never block.
    """

    FLOW = "/simty/viz/flow"
    SERVICES = "/simty/viz/services"
    STATUS = "/simty/viz/status"
    VEHICLE = "/simty/viz/vehicle"
    PATH = "/simty/viz/path"

    def __init__(self, bridge):
        self._bridge = bridge
        self._settings = bridge.settings
        self._state = bridge.state
        self._journal = bridge.journal
        self._trail: List[PoseStamped] = []
        self._last_trail_point: Optional[Tuple[float, float, float]] = None
        self._mesh_checked = False
        self._mesh_ok = False

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pubs = {
            self.FLOW: bridge.create_publisher(MarkerArray, self.FLOW, profile),
            self.SERVICES: bridge.create_publisher(MarkerArray, self.SERVICES, profile),
            self.STATUS: bridge.create_publisher(MarkerArray, self.STATUS, profile),
            self.VEHICLE: bridge.create_publisher(MarkerArray, self.VEHICLE, profile),
        }
        self._path_pub = bridge.create_publisher(Path, self.PATH, profile)

        from tf2_ros import TransformBroadcaster

        self._tf = TransformBroadcaster(bridge)

    # ── frames ───────────────────────────────────────────────────────────────

    @property
    def _prefix(self) -> str:
        return self._settings.viz_board_frame_prefix.strip("/") or "simty"

    @property
    def flow_frame(self) -> str:
        return f"{self._prefix}_flow_board"

    @property
    def service_frame(self) -> str:
        return f"{self._prefix}_service_board"

    @property
    def status_frame(self) -> str:
        return f"{self._prefix}_status_board"

    # ── entry point ──────────────────────────────────────────────────────────

    def publish(self) -> None:
        if not self._settings.viz_enabled:
            return

        wanted = {
            topic: self._bridge.count_subscribers(topic) > 0
            for topic in self._pubs
        }
        path_wanted = self._bridge.count_subscribers(self.PATH) > 0
        if not any(wanted.values()) and not path_wanted:
            return  # nobody is looking; do not build a single marker

        stamp = self._bridge.get_clock().now().to_msg()
        self._broadcast_board_frames(stamp)

        if wanted[self.FLOW]:
            self._pubs[self.FLOW].publish(self._flow_board(stamp))
        if wanted[self.SERVICES]:
            self._pubs[self.SERVICES].publish(self._service_board(stamp))
        if wanted[self.STATUS]:
            self._pubs[self.STATUS].publish(self._status_board(stamp))
        if wanted[self.VEHICLE] or path_wanted:
            self._broadcast_vehicle_frames(stamp)
        if wanted[self.VEHICLE]:
            self._pubs[self.VEHICLE].publish(self._vehicle_markers(stamp))
        if path_wanted:
            self._path_pub.publish(self._path(stamp))

    # ── TF ───────────────────────────────────────────────────────────────────

    def _transform(self, stamp, parent: str, child: str, x, y, z, rotation=None):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)
        transform.transform.rotation = rotation or Quaternion(w=1.0)
        return transform

    def _broadcast_board_frames(self, stamp) -> None:
        """
        The boards hang off `map` at fixed offsets. Broadcast every tick rather
        than once as a static transform: RViz started after the bridge would
        otherwise never see them, since a StaticTransformBroadcaster's latched
        sample is only replayed to subscribers that were present.
        """
        boards = (
            (self.flow_frame, self._settings.viz_flow_board_at),
            (self.service_frame, self._settings.viz_service_board_at),
            (self.status_frame, self._settings.viz_status_board_at),
        )
        for frame, at in boards:
            east = float(at[0]) if len(at) > 0 else 0.0
            north = float(at[1]) if len(at) > 1 else 0.0
            self._tf.sendTransform(self._transform(stamp, "map", frame, east, north, 0.0))

    def _broadcast_vehicle_frames(self, stamp) -> None:
        shared = self._bridge.shared
        ned = shared.get("ned") or (0.0, 0.0, 0.0)
        east, north, up = _ned_to_enu(ned)
        roll, pitch, yaw = shared.get("rpy") or (0.0, 0.0, float(shared.get("yaw", 0.0)))
        self._tf.sendTransform(
            self._transform(
                stamp,
                "map",
                "base_link",
                east,
                north,
                up,
                _euler_to_quat(roll, -pitch, math.pi / 2.0 - yaw),
            )
        )
        pan, gimbal_roll, tilt = shared.get("gimbal_cmd") or (0.0, 0.0, 0.0)
        self._tf.sendTransform(
            self._transform(
                stamp,
                "base_link",
                "gimbal_link",
                0.12,
                0.0,
                -0.08,
                _euler_to_quat(gimbal_roll, -tilt, -pan),
            )
        )

    # ── marker helpers ───────────────────────────────────────────────────────

    def _lifetime(self) -> RosDuration:
        # Three publish periods: a stale row disappears on its own instead of
        # being cleaned up with a DELETEALL, which would make RViz flicker.
        period = 1.0 / max(0.5, float(self._settings.viz_hz))
        total = period * 3.0
        return RosDuration(sec=int(total), nanosec=int((total % 1.0) * 1e9))

    def _base(self, stamp, frame: str, namespace: str, ident: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = namespace
        marker.id = ident
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = self._lifetime()
        marker.frame_locked = True
        return marker

    def _text(self, stamp, frame, namespace, ident, x, y, text, rgb=WHITE, height=TEXT, z=0.0):
        marker = self._base(stamp, frame, namespace, ident, Marker.TEXT_VIEW_FACING)
        marker.pose.position = Point(x=float(x), y=float(y), z=float(z))
        marker.scale.z = float(height)
        marker.color = _colour(rgb)
        marker.text = text
        return marker

    def _arrow(self, stamp, frame, namespace, ident, start, end, rgb, shaft=0.25):
        marker = self._base(stamp, frame, namespace, ident, Marker.ARROW)
        marker.points = [
            Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
            Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
        ]
        marker.scale.x = float(shaft)  # shaft diameter
        marker.scale.y = float(shaft) * 2.2  # head diameter
        marker.scale.z = float(shaft) * 2.5  # head length
        marker.color = _colour(rgb)
        return marker

    def _line(self, stamp, frame, namespace, ident, points, rgb, width=0.08, alpha=1.0):
        marker = self._base(stamp, frame, namespace, ident, Marker.LINE_STRIP)
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        marker.scale.x = float(width)
        marker.color = _colour(rgb, alpha)
        return marker

    # ── the flow board ───────────────────────────────────────────────────────

    def _route_health(self, route, stats) -> Tuple[Tuple[float, float, float], str]:
        """Colour and one-word verdict for a route row."""
        if not route.available:
            return RED, "no type"
        if not route.enabled or route.direction is Direction.OFF:
            return GREY, "off"
        if stats is None or stats.last_rx == 0.0:
            return YELLOW, "waiting"
        if stats.age() > self._settings.stale_after_s:
            return YELLOW, "stale"
        if self._journal.worst_level(route.key, self._settings.journal_active_window_s) >= ERROR:
            return RED, "errors"
        return GREEN, "flowing"

    def _flow_board(self, stamp) -> MarkerArray:
        frame = self.flow_frame
        array = MarkerArray()
        stats_all = self._state.snapshot_stats()
        window = self._settings.journal_active_window_s
        active = self._journal.issues_by_route(window)

        totals = self._journal.totals()
        h2s = totals.get(Direction.HOST_TO_SIM.value, {})
        s2h = totals.get(Direction.SIM_TO_HOST.value, {})
        array.markers.append(
            self._text(
                stamp, frame, "flow_title", 0, COL_SIM * 0.5, 2.6 * ROW,
                "CONVERSION FLOW   wrapper <-> simulation",
                CYAN, HEAD * 1.1,
            )
        )
        array.markers.append(
            self._text(
                stamp, frame, "flow_title", 1, COL_SIM * 0.5, 1.7 * ROW,
                f"host->sim warnings {h2s.get(WARN, 0)} / errors {h2s.get(ERROR, 0)}"
                f"      sim->host warnings {s2h.get(WARN, 0)} / errors {s2h.get(ERROR, 0)}",
                YELLOW if (h2s.get(WARN, 0) or s2h.get(WARN, 0)) else GREY,
                TEXT,
            )
        )
        for column, label in (
            (COL_HOST, self._settings.wrapper_prefix),
            (COL_SIM, "simulation (PX4 / Gazebo)"),
            (COL_STATS, "rx / tx / rate"),
            (COL_WARN, "conversion warnings"),
        ):
            array.markers.append(
                self._text(stamp, frame, "flow_head", int(column), column, 0.8 * ROW, label, BLUE, TEXT)
            )

        for index, route in enumerate(self._bridge.routes):
            stats = stats_all.get(route.key)
            rgb, verdict = self._route_health(route, stats)
            y = -index * ROW
            ident = index * 10

            array.markers.append(
                self._text(
                    stamp, frame, "flow_host", ident, COL_HOST, y,
                    _short(route.host_topic.replace(self._settings.wrapper_prefix, "~")),
                    rgb,
                )
            )
            array.markers.append(
                self._text(
                    stamp, frame, "flow_key", ident + 1, (COL_ARROW_A + COL_ARROW_B) / 2.0,
                    y + 0.55, f"{route.key}  [{verdict}]", rgb, TEXT * 0.8,
                )
            )
            array.markers.append(
                self._text(
                    stamp, frame, "flow_sim", ident + 2, COL_SIM, y,
                    _short(route.sim_topic) + (f"  +{len(route.fanout)}" if route.fanout else ""),
                    rgb,
                )
            )

            hz = stats.hz if stats else 0.0
            shaft = 0.18 + 0.5 * min(1.0, hz / 25.0)
            if route.direction is Direction.HOST_TO_SIM:
                start, end = (COL_ARROW_A, y, 0.0), (COL_ARROW_B, y, 0.0)
            elif route.direction is Direction.SIM_TO_HOST:
                start, end = (COL_ARROW_B, y, 0.0), (COL_ARROW_A, y, 0.0)
            else:
                array.markers.append(
                    self._line(
                        stamp, frame, "flow_arrow", ident + 3,
                        [(COL_ARROW_A, y, 0.0), (COL_ARROW_B, y, 0.0)], GREY, 0.06, 0.6,
                    )
                )
                start = end = None
            if start is not None:
                array.markers.append(
                    self._arrow(stamp, frame, "flow_arrow", ident + 3, start, end, rgb, shaft)
                )

            if stats:
                text = (
                    f"rx {stats.rx}  tx {stats.tx}  {hz:.1f}Hz"
                    f"  cap {'-' if route.max_hz <= 0 else f'{route.max_hz:g}'}"
                )
                if stats.dropped_rate:
                    text += f"  capped {stats.dropped_rate}"
                if stats.dropped_error:
                    text += f"  dropped {stats.dropped_error}"
            else:
                text = "no traffic"
            array.markers.append(
                self._text(stamp, frame, "flow_stats", ident + 4, COL_STATS, y, text, rgb, TEXT * 0.85)
            )

            issues = active.get(route.key, [])
            if issues:
                lines = [
                    f"{issue.code} x{issue.count}"
                    for issue in issues[:MAX_FLOW_WARNINGS]
                ]
                if len(issues) > MAX_FLOW_WARNINGS:
                    lines.append(f"+{len(issues) - MAX_FLOW_WARNINGS} more")
                worst = max(issue.level for issue in issues)
                array.markers.append(
                    self._text(
                        stamp, frame, "flow_warn", ident + 5, COL_WARN, y,
                        "  ".join(lines), LEVEL_COLOUR.get(worst, YELLOW), TEXT * 0.85,
                    )
                )

        return array

    # ── the service board ────────────────────────────────────────────────────

    def _service_board(self, stamp) -> MarkerArray:
        frame = self.service_frame
        array = MarkerArray()
        calls = self._state.service_calls()

        array.markers.append(
            self._text(
                stamp, frame, "srv_title", 0, COL_SIM * 0.4, 2.0 * ROW,
                f"BRIDGED SERVICES   {self._settings.wrapper_prefix}/*",
                CYAN, HEAD * 1.1,
            )
        )
        array.markers.append(
            self._text(
                stamp, frame, "srv_title", 1, COL_SIM * 0.4, 1.1 * ROW,
                "SERVE = this bridge answers and drives the sim   "
                "PROXY = the Manifold answers, we forward",
                GREY, TEXT * 0.9,
            )
        )

        for index, service in enumerate(self._bridge.service_routes):
            good, bad, last = calls.get(service.key, (0, 0, ""))
            y = -index * ROW
            ident = index * 10
            if not service.available:
                rgb, verdict = RED, "no type"
            elif service.remote_server_seen:
                rgb, verdict = YELLOW, "name collision"
            elif not service.enabled or service.role is ServiceRole.OFF:
                rgb, verdict = GREY, "off"
            elif bad:
                rgb, verdict = RED, "failing"
            else:
                rgb, verdict = GREEN, service.role.value
            array.markers.append(
                self._text(
                    stamp, frame, "srv_name", ident, COL_HOST, y,
                    f"~/{service.service}", rgb,
                )
            )
            array.markers.append(
                self._text(
                    stamp, frame, "srv_role", ident + 1, COL_ARROW_B, y,
                    f"{verdict}   ok {good}  fail {bad}", rgb, TEXT * 0.85,
                )
            )
            if last and last != "ok":
                array.markers.append(
                    self._text(
                        stamp, frame, "srv_last", ident + 2, COL_STATS, y,
                        last[:48], RED if bad else GREY, TEXT * 0.8,
                    )
                )
        return array

    # ── the status board ─────────────────────────────────────────────────────

    def _status_board(self, stamp) -> MarkerArray:
        frame = self.status_frame
        array = MarkerArray()
        report = self._bridge.report()
        subs, pubs = self._bridge.endpoint_counts()
        flight = self._state.flight()

        if report.manifold is ManifoldState.CONNECTED:
            verdict, rgb = "MANIFOLD CONNECTED", GREEN
        elif report.manifold is ManifoldState.MOCK:
            verdict, rgb = "SYNTHETIC / STAND-IN MANIFOLD", YELLOW
        else:
            verdict, rgb = "SEARCHING FOR MANIFOLD", RED

        array.markers.append(
            self._text(stamp, frame, "status", 0, 0.0, 2.0 * ROW, verdict, rgb, HEAD * 1.3)
        )

        provision = getattr(self._bridge, "provisioner", None)
        lines: List[Tuple[str, Tuple[float, float, float]]] = [
            (report.summary(), rgb),
            (f"{report.rmw}   domain {report.domain}   SPDP mcast udp/{report.multicast_port}", WHITE),
            (
                f"dds profile {'ok' if report.profiles_ok else 'MISSING'}"
                f"   endpoints {subs} sub / {pubs} pub"
                f"   {flight.get('px4_clock', '-')}",
                WHITE if report.profiles_ok else YELLOW,
            ),
            (f"remote nodes: {', '.join(report.remote_nodes) or '-'}", WHITE),
        ]
        if provision is not None:
            lines.append((f"provisioning: {provision.describe()}", CYAN))
        armed = bool(flight.get("armed"))
        lines.append(
            (
                f"armed {'YES' if armed else 'no'}   nav {flight.get('nav_state', '-')}   "
                f"RC {'live' if flight.get('rc_live') else 'stale'}   "
                f"authority {'granted' if flight.get('rc_authority') else 'released'}   "
                f"offboard hb {'streaming' if flight.get('offboard_stream') else 'idle'}",
                RED if armed else WHITE,
            )
        )

        for index, (text, colour) in enumerate(lines):
            array.markers.append(
                self._text(stamp, frame, "status", 10 + index, 0.0, 0.9 * ROW - index * ROW * 0.8, text, colour, TEXT)
            )

        base_y = 0.9 * ROW - len(lines) * ROW * 0.8 - ROW
        array.markers.append(
            self._text(stamp, frame, "journal_title", 0, 0.0, base_y, "CONVERSION JOURNAL", CYAN, HEAD)
        )
        events = self._journal.recent(MAX_JOURNAL_LINES, min_level=INFO)
        if not events:
            array.markers.append(
                self._text(
                    stamp, frame, "journal", 0, 0.0, base_y - ROW * 0.8,
                    "no conversion compromises reported", GREEN, TEXT,
                )
            )
        for index, event in enumerate(reversed(events)):
            suffix = f"  (+{event.suppressed} suppressed)" if event.suppressed else ""
            array.markers.append(
                self._text(
                    stamp, frame, "journal", index + 1, 0.0,
                    base_y - (index + 1) * ROW * 0.8,
                    f"{event.clock}  {event.route}  {event.code}{suffix}",
                    LEVEL_COLOUR.get(event.level, WHITE), TEXT * 0.9,
                )
            )
        return array

    # ── the vehicle ──────────────────────────────────────────────────────────

    def _mesh_available(self) -> bool:
        if not self._mesh_checked:
            self._mesh_checked = True
            path = self._settings.viz_drone_mesh
            local = path[7:] if path.startswith("file://") else path
            self._mesh_ok = bool(path) and os.path.exists(local)
        return self._mesh_ok

    def _vehicle_markers(self, stamp) -> MarkerArray:
        array = MarkerArray()
        shared = self._bridge.shared
        flight = self._state.flight()

        if self._mesh_available():
            body = self._base(stamp, "base_link", "vehicle", 0, Marker.MESH_RESOURCE)
            body.mesh_resource = self._settings.viz_drone_mesh
            body.mesh_use_embedded_materials = True
            body.scale.x = body.scale.y = body.scale.z = 1.0
            body.color = _colour(WHITE, 1.0)
        else:
            body = self._base(stamp, "base_link", "vehicle", 0, Marker.CUBE)
            body.scale.x, body.scale.y, body.scale.z = 0.45, 0.45, 0.12
            body.color = _colour(GREEN if flight.get("armed") else GREY, 0.9)
        array.markers.append(body)

        # Nose direction, so a mirrored heading conversion is visible at a glance.
        array.markers.append(
            self._arrow(stamp, "base_link", "vehicle", 1, (0.0, 0.0, 0.0), (1.2, 0.0, 0.0), BLUE, 0.06)
        )
        # Gimbal line of sight.
        array.markers.append(
            self._arrow(stamp, "gimbal_link", "vehicle", 2, (0.0, 0.0, 0.0), (1.5, 0.0, 0.0), CYAN, 0.05)
        )

        # Downward LRF: the value the bridge actually sent as RelativeObstacleInfo.down.
        lrf = float(shared.get("lrf_range", 0.0) or 0.0)
        if lrf > 0.0:
            array.markers.append(
                self._line(
                    stamp, "base_link", "lrf", 0,
                    [(0.0, 0.0, 0.0), (0.0, 0.0, -lrf)], YELLOW, 0.04,
                )
            )
            array.markers.append(
                self._text(
                    stamp, "base_link", "lrf", 1, 0.25, 0.0,
                    f"LRF {lrf:.2f} m", YELLOW, 0.3, z=-lrf * 0.5,
                )
            )

        # Commanded RC velocity, in NED as published to PX4.
        command = shared.get("rc_cmd")
        if command:
            north, east, down = float(command[0]), float(command[1]), float(command[2])
            if abs(north) + abs(east) + abs(down) > 0.05:
                array.markers.append(
                    self._arrow(
                        stamp, "map", "rc_cmd", 0,
                        _ned_to_enu(shared.get("ned") or (0.0, 0.0, 0.0)),
                        tuple(
                            a + b
                            for a, b in zip(
                                _ned_to_enu(shared.get("ned") or (0.0, 0.0, 0.0)),
                                _ned_to_enu((north, east, down)),
                            )
                        ),
                        RED, 0.12,
                    )
                )

        home = self._base(stamp, "map", "home", 0, Marker.CYLINDER)
        home.scale.x = home.scale.y = 1.2
        home.scale.z = 0.1
        home.color = _colour(BLUE, 0.6)
        array.markers.append(home)

        label = (
            f"{'ARMED' if flight.get('armed') else 'disarmed'}   "
            f"nav {flight.get('nav_state', '-')}   "
            f"batt {flight.get('battery') if flight.get('battery') is not None else '-'}"
        )
        array.markers.append(
            self._text(
                stamp, "base_link", "vehicle_label", 0, 0.0, 0.0, label,
                RED if flight.get("armed") else WHITE, 0.35, z=0.9,
            )
        )
        return array

    def _path(self, stamp) -> Path:
        shared = self._bridge.shared
        ned = shared.get("ned")
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "map"
        if ned is not None:
            east, north, up = _ned_to_enu(ned)
            previous = self._last_trail_point
            moved = (
                previous is None
                or math.dist((east, north, up), previous) > 0.25
            )
            if moved:
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = "map"
                pose.pose.position = Point(x=east, y=north, z=up)
                pose.pose.orientation.w = 1.0
                self._trail.append(pose)
                self._last_trail_point = (east, north, up)
                limit = max(2, int(self._settings.viz_trail_length))
                if len(self._trail) > limit:
                    del self._trail[: len(self._trail) - limit]
        path.poses = list(self._trail)
        return path

    def clear_trail(self) -> None:
        self._trail.clear()
        self._last_trail_point = None


__all__ = ["ControlRoomViz"]
