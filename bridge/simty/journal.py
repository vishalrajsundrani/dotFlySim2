"""
The conversion journal: what each translation had to compromise on, and why.

WHY THIS EXISTS SEPARATELY FROM state.LogRing
---------------------------------------------
LogRing is a flat stream of text. It answers "what happened at 12:41:07". It
cannot answer the question an operator actually has in front of a bridge that
is running but behaving oddly:

    "which conversions are quietly lying to the other side, and how often?"

Almost every converter in this bridge has to make a decision the source data
does not fully justify -- clamp a gimbal command to the m4e's mechanical
limits, rotate RC sticks by a heading that has not arrived yet, report a DJI
battery percentage when PX4 says "unknown", substitute P_GPS for a PX4
nav_state with no DJI analogue. Each of those is correct behaviour and each of
them is a lossy step. Made invisible, they turn into a support ticket that
reads "the gimbal does not go past 60 degrees" three weeks later.

So converters report them, by code, and this module aggregates:

  * first-seen / last-seen and a count per (route, code)
  * a bounded ring of recent events for the RViz ticker and the JSON topic
  * per-direction WARN/ERROR totals for the dashboard and diagnostics

RATE LIMITING IS NOT OPTIONAL
-----------------------------
rc_setpoint runs at 50 Hz. A converter that warns on every sample would push
3000 lines a minute into the log ring and evict everything else -- the exact
failure mode where the warning system destroys the information it exists to
provide. So the first occurrence of a (route, code) pair is logged immediately,
subsequent ones are counted and re-logged at most once per `min_interval_s`,
carrying the suppressed count with them. Nothing is lost: the counters are
exact, only the text output is thinned.
"""

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Optional, Tuple

from .state import DEBUG, ERROR, INFO, WARN

# ── the code catalogue ───────────────────────────────────────────────────────
#
# code -> (default level, what the conversion DID about it)
#
# The second element is written as the action taken, not as the problem, because
# that is the part an operator cannot deduce from the topic data: seeing
# "gps_accuracy_unknown" in a log tells you nothing; seeing "eph/epv are 0, so
# covariance is published as UNKNOWN rather than claiming perfect accuracy"
# tells you exactly what the far side received.

CODES: Dict[str, Tuple[int, str]] = {
    # ── host -> sim ──────────────────────────────────────────────────────────
    "rc_axes_short": (
        WARN,
        "Joy carried fewer than 4 axes; the missing ones were treated as centred",
    ),
    "rc_axis_clipped": (
        WARN,
        "a stick exceeded rc_max full scale and was clamped; check the rc_max "
        "tunable against what the wrapper actually publishes",
    ),
    "rc_heading_unknown": (
        WARN,
        "no attitude has arrived, so body-frame sticks were rotated by yaw=0 "
        "-- control degrades to head-north instead of nose-forward",
    ),
    "rc_heading_stale": (
        WARN,
        "the heading used to rotate sticks into NED is older than "
        "rc_heading_stale_s; forward is where the nose USED to point",
    ),
    "rc_no_authority": (
        INFO,
        "sticks are arriving without RC authority; the setpoint is still "
        "published but PX4 ignores it until offboard is engaged",
    ),
    "gimbal_axis_clamped": (
        WARN,
        "commanded angle is outside the m4e's mechanical range and was clamped "
        "to the limit in models/m4e/model.sdf",
    ),
    "gimbal_mode_unsupported": (
        WARN,
        "rotation_mode is incremental; the simulated gimbal implements absolute "
        "positioning only, so the value was applied as absolute",
    ),
    "gimbal_nonfinite": (ERROR, "a non-finite gimbal angle was dropped"),
    "gimbal_joint_malformed": (
        ERROR,
        "the /drone/gimbal/cmd/pan setpoint arrived without a `data` field, so "
        "the reverse gimbal conversion had no pan angle to report and dropped "
        "the sample",
    ),
    "gimbal_angles_malformed": (
        ERROR,
        "gimbal_angles arrived without a `vector` field, so there was nothing to "
        "convert -- check that the topic really carries geometry_msgs/Vector3Stamped",
    ),
    "gimbal_heading_unknown": (
        WARN,
        "no aircraft heading has arrived, so DJI's ground-frame gimbal yaw was "
        "used as a body-relative joint angle. Expect the pan joint to sit on its "
        "limit until the aircraft's attitude topic appears, because ground-frame "
        "yaw is measured from North and the joint is measured from the airframe",
    ),
    "gimbal_heading_stale": (
        WARN,
        "the heading used to make gimbal yaw body-relative is older than "
        "gimbal_heading_stale_s; the pan joint is following where the nose USED "
        "to point",
    ),
    "gimbal_yaw_reference_unknown": (
        WARN,
        "gimbal_yaw_reference is not aircraft|sim|none, so it was treated as "
        "none and ground-frame yaw was copied through untouched",
    ),
    # ── images ───────────────────────────────────────────────────────────────
    "image_empty": (
        WARN,
        "an Image arrived with zero step or height, so there was no payload to "
        "forward -- usually a camera that has not produced its first frame",
    ),
    "image_frame_large": (
        INFO,
        "a single video frame is larger than image_warn_bytes. Uncompressed "
        "frames fragment into thousands of datagrams at the 1400-byte cap in "
        "config/dds.xml; if telemetry starts stuttering, lower the route's max_hz "
        "or switch to a preview-tier feed",
    ),
    # ── the wider telemetry surface ──────────────────────────────────────────
    "altitude_invalid": (WARN, "PX4 reports alt_valid=false; the altitude was forwarded anyway"),
    "altitude_nonfinite": (ERROR, "a non-finite altitude was dropped"),
    "altitude_baro_substituted": (
        INFO,
        "barometric altitude is standing in for the EKF's fused altitude: the "
        "simulation has no separate barometer, so this value lacks the drift and "
        "the weather offset a real baro shows",
    ),
    "hagl_from_ekf": (
        WARN,
        "no valid rangefinder return, so height above ground fell back to -z "
        "above the EKF origin -- height above the takeoff point, not the terrain",
    ),
    "hagl_nonfinite": (ERROR, "a non-finite height above ground was dropped"),
    "velocity_invalid": (WARN, "PX4 reports the velocity estimate invalid; forwarded anyway"),
    "velocity_nonfinite": (ERROR, "a non-finite velocity was dropped"),
    "angular_rate_missing": (ERROR, "VehicleAngularVelocity carried no xyz field"),
    "angular_rate_nonfinite": (ERROR, "a non-finite body rate was dropped"),
    "imu_missing_fields": (
        ERROR,
        "SensorCombined carried no accelerometer or gyro array, so no Imu could "
        "be built",
    ),
    "imu_nonfinite": (ERROR, "a non-finite IMU sample was dropped"),
    "angular_rate_no_attitude": (
        WARN,
        "angular_rate_ground_fused went out carrying BODY rates, not ground "
        "rates, because no attitude had arrived to rotate them with. The topic "
        "name promises a ground frame, so treat these samples as unusable "
        "rather than merely late",
    ),
    "gps_control_level_copied": (
        DEBUG,
        "gps_control_level is reported as a copy of gps_signal_level. A real "
        "aircraft distinguishes 'the GPS looks good' from 'the flight "
        "controller trusts it'; this simulation models no such distrust",
    ),
    "motor_start_error_generic": (
        WARN,
        "the motors would not start and the reason was reported as the generic "
        "code 1, because DJI's numbering does not map onto PX4's failsafe "
        "flags. The flag names are in the detail text and are the real answer",
    ),
    "imu_no_orientation": (
        WARN,
        "the Imu message went out with no orientation (REP-145 covariance[0] = -1) "
        "because no fresh VehicleAttitude was available to splice in. A real M4E "
        "always carries orientation on this topic, so a consumer written against "
        "hardware may not handle its absence",
    ),
    "home_nonfinite": (ERROR, "a non-finite home position was dropped"),
    "odometry_missing_fields": (ERROR, "VehicleOdometry lacked position or orientation"),
    "odometry_nonfinite": (ERROR, "a non-finite odometry sample was dropped"),
    "odometry_no_velocity": (
        WARN,
        "odometry velocity was absent or non-finite and is reported as zero, "
        "which a consumer cannot distinguish from genuinely standing still",
    ),
    "authority_denied": (
        INFO,
        "device_mode does not match rc_device_mode, so RC authority was "
        "published as false",
    ),
    "authority_flag_conflict": (
        WARN,
        "device_mode says RC but control_auth is not 1; with "
        "require_control_auth on, authority is withheld",
    ),
    "clock_unsynced": (
        WARN,
        "no PX4 timestamp has been seen, so outgoing timestamps are wall clock "
        "-- decades out in PX4's simulated-time domain, and PX4 will reject "
        "them as stale",
    ),
    # ── sim -> host ──────────────────────────────────────────────────────────
    "battery_remaining_unknown": (
        WARN,
        "PX4 reports remaining=-1 (unknown); DJI capacity_percentage was sent "
        "as 0, which a client cannot distinguish from a flat battery",
    ),
    "battery_temp_invalid": (
        INFO,
        "battery temperature was non-finite or below -100 C; 25 C substituted",
    ),
    "battery_voltage_zero": (
        WARN,
        "neither voltage_v nor voltage_filtered_v carried a value; 0 V sent",
    ),
    "gps_no_fix": (
        WARN,
        "fix_type < 2, so NavSatFix.status reports NO_FIX",
    ),
    "gps_origin_zero": (
        WARN,
        "lat and lon are both exactly 0 -- PX4's global origin is not set yet, "
        "so this is not a position off West Africa",
    ),
    "gps_nonfinite": (ERROR, "a non-finite coordinate was dropped"),
    "gps_accuracy_unknown": (
        INFO,
        "eph/epv are 0, so covariance_type is published as UNKNOWN rather than "
        "DIAGONAL_KNOWN with a zero diagonal claiming perfect accuracy",
    ),
    "local_pos_invalid": (
        WARN,
        "PX4 marks this local position estimate invalid; values were forwarded "
        "with the matching PositionFused health flag set to 0",
    ),
    "heading_nonfinite": (
        WARN,
        "heading was NaN; the previous heading is still in use for RC rotation",
    ),
    "landed_unknown": (
        INFO,
        "the land_detected route is not running, so ON_GROUND vs ON_AIR is "
        "inferred from arming state alone",
    ),
    "nav_state_unmapped": (
        INFO,
        "this PX4 nav_state has no DJI analogue; P_GPS was substituted, which "
        "is what a DJI operator sees in normal GPS flight",
    ),
    "home_invalid": (
        WARN,
        "PX4 reports the home position invalid; HomePosition went out with "
        "status FAILED",
    ),
    "lrf_no_returns": (
        WARN,
        "no usable range in the scan; down_health was set to 0 rather than "
        "reporting a fabricated clear path below the aircraft",
    ),
    "quat_denormalised": (
        WARN,
        "attitude quaternion is not unit length; Euler angles from it are "
        "approximate",
    ),
    "attitude_missing_q": (ERROR, "VehicleAttitude carried no usable quaternion"),
    # ── psdk_ros2 flight control setpoints ──────────────────────────────────
    "setpoint_axes_short": (
        WARN,
        "a flight_control_setpoint message carried fewer than the four axes "
        "psdk_ros2 defines; the sample was dropped",
    ),
    "setpoint_nonfinite": (
        ERROR,
        "a setpoint axis was NaN or infinite; the sample was dropped rather "
        "than passed to PX4, which would latch it",
    ),
    "setpoint_needs_position": (
        WARN,
        "DJI's position setpoint is an OFFSET from where the aircraft is, so "
        "it cannot be resolved while the position_fused route is off -- turn "
        "that route on (project testing mode does)",
    ),
    "setpoint_clipped": (
        WARN,
        "the commanded offset, speed, climb rate or altitude exceeded this "
        "bridge's setpoint_* limit and was clamped",
    ),
    "setpoint_heading_unknown": (
        WARN,
        "a body-frame (FLU) velocity setpoint arrived before any attitude did, "
        "so 'forward' was taken to mean north",
    ),
    "setpoint_yaw_frame": (
        INFO,
        "yaw is being read as ENU (0 = east, counter-clockwise) rather than "
        "DJI's clockwise-from-north; set setpoint_yaw_frame to change it",
    ),
    # ── route plumbing, reported by node.py rather than a converter ──────────
    "type_unavailable": (
        ERROR,
        "a message type for this route is not in the workspace, so the route "
        "cannot be realised",
    ),
    "sink_missing": (ERROR, "the publisher for a fan-out topic was not found"),
    "converter_raised": (ERROR, "the converter raised; the sample was dropped"),
    "publish_failed": (ERROR, "publishing the converted sample failed"),
    "qos_subscribe_risk": (
        WARN,
        "a RELIABLE subscription cannot match a BEST_EFFORT publisher, and "
        "every /fmu/out topic is best-effort -- this route will receive nothing",
    ),
    "service_collision": (
        WARN,
        "a remote server already owns this name; two servers on one name means "
        "clients reach whichever DDS matched first",
    ),
    "service_failed": (ERROR, "a bridged service handler raised"),
    # ── commands out to PX4 ──────────────────────────────────────────────────
    "arm_mode_cleared": (
        INFO,
        "the vehicle was in a flight mode that refuses to arm (PX4 stays in "
        "AUTO_LAND after a landing), so AUTO_LOITER was requested first -- "
        "without it the next takeoff is refused with nothing said on the DJI "
        "surface, which carries no arming reason",
    ),
    "arm_blocked": (
        WARN,
        "PX4's failsafe flags say arming will be refused; the command was sent "
        "anyway because the operator asked for it, but expect 'Arming denied'",
    ),
    "arm_denied": (
        ERROR,
        "the arm command was sent and the vehicle did not arm -- PX4 refused it. "
        "The listed failsafe flags are why; they are sim-side, not bridge-side",
    ),
    "takeoff_alt_unknown": (
        INFO,
        "no global altitude has arrived, so NAV_TAKEOFF went out with a "
        "non-finite altitude and PX4 will use MIS_TAKEOFF_ALT above home",
    ),
}


def describe(code: str) -> str:
    """What the conversion did about `code`. Empty string for unknown codes."""
    entry = CODES.get(code)
    return entry[1] if entry else ""


def default_level(code: str) -> int:
    entry = CODES.get(code)
    return entry[0] if entry else WARN


# ── records ──────────────────────────────────────────────────────────────────


@dataclass
class Issue:
    """Aggregate state for one (route, code) pair."""

    route: str
    direction: str
    code: str
    level: int
    count: int = 0
    suppressed: int = 0
    first_at: float = 0.0
    last_at: float = 0.0
    detail: str = ""

    @property
    def age(self) -> float:
        return time.time() - self.last_at if self.last_at else 1e9

    def is_active(self, within_s: float) -> bool:
        """True when this issue is still happening, not merely historical."""
        return self.age <= within_s


@dataclass
class Event:
    """One journal line, for the RViz ticker and the machine-readable topic."""

    when: float
    route: str
    direction: str
    code: str
    level: int
    detail: str
    suppressed: int = 0
    seq: int = 0  # monotonic, so a publisher can ask for "everything after N"

    @property
    def clock(self) -> str:
        return time.strftime("%H:%M:%S", time.localtime(self.when))


# ── the journal ──────────────────────────────────────────────────────────────


class ConversionJournal:
    """
    Thread-safe. Written from every executor thread that runs a converter, read
    by the TUI thread, the viz timer and the diagnostics timer.

    `sink` receives (level, source, message) for the text log -- in practice
    state.LogRing.add, optionally also mirrored to the ROS logger by node.py.
    """

    def __init__(
        self,
        sink: Optional[Callable[[int, str, str], None]] = None,
        min_interval_s: float = 5.0,
        event_capacity: int = 400,
    ):
        self._lock = threading.Lock()
        self._sink = sink
        self._min_interval = max(0.0, float(min_interval_s))
        self._issues: Dict[Tuple[str, str], Issue] = {}
        self._events: Deque[Event] = deque(maxlen=event_capacity)
        self._last_logged: Dict[Tuple[str, str], float] = {}
        self._totals: Dict[str, Dict[int, int]] = {}
        self._seq = 0
        self._event_seq = 0

    # -- write side --

    def report(
        self,
        route: str,
        direction: str,
        code: str,
        detail: str = "",
        level: Optional[int] = None,
    ) -> None:
        """
        Record one conversion compromise. Cheap enough for a 50 Hz path: two
        dict lookups and some integer arithmetic on the hot repeat path.
        """
        now = time.time()
        level = default_level(code) if level is None else level
        key = (route, code)

        with self._lock:
            issue = self._issues.get(key)
            if issue is None:
                issue = Issue(
                    route=route,
                    direction=direction,
                    code=code,
                    level=level,
                    first_at=now,
                )
                self._issues[key] = issue
            issue.count += 1
            issue.last_at = now
            issue.level = level
            if detail:
                issue.detail = detail
            self._totals.setdefault(direction, {}).setdefault(level, 0)
            self._totals[direction][level] += 1
            self._seq += 1

            last = self._last_logged.get(key, 0.0)
            due = (now - last) >= self._min_interval
            if due:
                self._last_logged[key] = now
                suppressed = issue.suppressed
                issue.suppressed = 0
            else:
                issue.suppressed += 1
                suppressed = -1

            if due:
                self._event_seq += 1
                self._events.append(
                    Event(
                        when=now,
                        route=route,
                        direction=direction,
                        code=code,
                        level=level,
                        detail=detail or describe(code),
                        suppressed=max(0, suppressed),
                        seq=self._event_seq,
                    )
                )

        if due and self._sink is not None:
            text = f"{code}: {detail or describe(code)}"
            if suppressed > 0:
                text += f"  (+{suppressed} more since the last line)"
            self._sink(level, route, text)

    # -- read side --

    def issues(self, active_within_s: float = 0.0) -> List[Issue]:
        """
        Snapshot, worst and most frequent first. With `active_within_s` > 0 only
        issues still occurring within that window are returned, which is what
        the dashboard wants -- a clamp that stopped an hour ago is history, not
        a current problem.
        """
        with self._lock:
            items = list(self._issues.values())
        if active_within_s > 0.0:
            items = [i for i in items if i.is_active(active_within_s)]
        items.sort(key=lambda i: (-i.level, -i.count))
        return items

    def for_route(self, route: str) -> List[Issue]:
        with self._lock:
            items = [i for i in self._issues.values() if i.route == route]
        items.sort(key=lambda i: (-i.level, -i.count))
        return items

    def issues_by_route(self, active_within_s: float = 0.0) -> Dict[str, List[Issue]]:
        """
        Every route's issues in one pass, worst first within each route.

        for_route() takes the lock and scans every issue, so calling it once per
        row is O(routes x issues) with a lock acquisition per row -- fine in a
        script, not fine in a render loop that runs on every keypress. Renderers
        (the console, the RViz boards, diagnostics, /simty/state) use this
        instead: one lock, one scan.
        """
        grouped: Dict[str, List[Issue]] = {}
        with self._lock:
            items = list(self._issues.values())
        for issue in items:
            if active_within_s > 0.0 and not issue.is_active(active_within_s):
                continue
            grouped.setdefault(issue.route, []).append(issue)
        for issues in grouped.values():
            issues.sort(key=lambda i: (-i.level, -i.count))
        return grouped

    def worst_level(self, route: str, active_within_s: float = 10.0) -> int:
        """Highest severity currently active on one route, or 0 when clean."""
        worst = 0
        for issue in self.for_route(route):
            if issue.is_active(active_within_s) and issue.level > worst:
                worst = issue.level
        return worst

    def recent(self, count: int = 20, min_level: int = DEBUG) -> List[Event]:
        with self._lock:
            items = [e for e in self._events if e.level >= min_level]
        return items[-count:]

    def events_since(self, seq: int, limit: int = 50) -> List[Event]:
        """
        Events with seq > `seq`, oldest first. Lets diagnostics.py publish each
        journal line exactly once without keeping its own copy of the ring.
        """
        with self._lock:
            items = [e for e in self._events if e.seq > seq]
        return items[:limit]

    @property
    def event_seq(self) -> int:
        with self._lock:
            return self._event_seq

    def totals(self) -> Dict[str, Dict[int, int]]:
        """{direction: {level: count}} -- for the per-direction health boxes."""
        with self._lock:
            return {d: dict(levels) for d, levels in self._totals.items()}

    def counts_by_level(self) -> Dict[int, int]:
        out = {DEBUG: 0, INFO: 0, WARN: 0, ERROR: 0}
        with self._lock:
            for levels in self._totals.values():
                for level, count in levels.items():
                    out[level] = out.get(level, 0) + count
        return out

    @property
    def seq(self) -> int:
        with self._lock:
            return self._seq

    def set_min_interval(self, seconds: float) -> None:
        with self._lock:
            self._min_interval = max(0.0, float(seconds))

    def clear(self) -> None:
        with self._lock:
            self._issues.clear()
            self._events.clear()
            self._last_logged.clear()
            self._totals.clear()


__all__ = [
    "CODES",
    "ConversionJournal",
    "Event",
    "Issue",
    "default_level",
    "describe",
]
