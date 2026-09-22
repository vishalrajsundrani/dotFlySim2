"""
Message translation functions.

CONTRACT
--------
Every converter has the signature

    f(msg, ctx) -> None | Message | list[tuple[str, Message]]

  * None                       -> drop this sample
  * Message                    -> publish on the route's opposite topic
  * [(topic, Message), ...]    -> fan out to explicit topics (see the gimbal
                                  converter, which turns one DJI command into
                                  three joint setpoints)

`ctx` (see ConversionContext) carries the PX4 clock, the tunable parameter
dict, a shared scratch space for cross-converter state such as the current
heading, a logger, and the conversion journal.

Converters are pure with respect to ROS: they never publish, never subscribe,
and never touch the node. That makes them trivially unit-testable and keeps
all lifecycle concerns in node.py.

EVERY LOSSY STEP IS REPORTED
----------------------------
Translating between two vendors' models of a drone is not a field rename. A DJI
gimbal command can ask for an angle the m4e cannot reach; PX4 can report a
battery level of "unknown" where DJI's field has no such value; RC sticks are
body-frame and PX4's setpoint is NED, so a heading that has not arrived yet
silently changes what "forward" means. Each converter below therefore calls
ctx.warn(<code>) whenever it clamps, substitutes, infers or drops -- see
journal.py for the code catalogue and what each one means. The FLOW screen and
the RViz warning board are rendered straight off those calls, so a conversion
compromise is visible while it is happening rather than inferred from odd
behaviour days later.

FIELD NAMES ARE VERIFIED
------------------------
Every psdk_interfaces field used here was read out of psdk_interfaces/msg/*.msg
in this repository. This matters because the previous revision guessed, and
guessed wrong nearly every time:

    FlightStatus  has `flight_status` (0/1/2), NOT `flight_mode`/`arming_state`
    DisplayMode   has `display_mode`,          NOT `mode`
    GPSDetails    has DOP/accuracy/sat-counts, and NO lat/lon/alt whatsoever
    GimbalStatus  is health flags only,        and has NO pitch/roll/yaw
    SingleBatteryInfo uses `capacity_percentage` (0-100) and `capacity_remain`

Lat/lon belongs on sensor_msgs/NavSatFix (which is what psdk_ros2 actually
publishes for gps_position/rtk_position) and gimbal angles on
geometry_msgs/Vector3Stamped. Those are the types used below.

PX4 field names drift between releases, so PX4 messages are read through _g()
with a default rather than by direct attribute access.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, QuaternionStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image, Imu, Joy, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Float64, String, UInt8, UInt16

from psdk_interfaces.msg import (
    ControlMode,
    DisplayMode,
    FlightAnomaly,
    FlightStatus,
    GimbalStatus,
    GPSDetails,
    HomePosition as PsdkHomePosition,
    PositionFused,
    RCConnectionStatus,
    RelativeObstacleInfo,
    SingleBatteryInfo,
)

# ── context ──────────────────────────────────────────────────────────────────


@dataclass
class ConversionContext:
    """
    Everything a converter is allowed to see.

    node.py builds one of these per route, all sharing the same `params` and
    `shared` dicts. Per-route rather than one global instance because the
    executor is multi-threaded with a reentrant callback group: two converters
    genuinely run at once, so `route`/`direction` cannot be fields that the
    caller rewrites before each call without racing.
    """

    clock: Any  # simty.clock.Px4Clock
    params: Dict[str, Any] = field(default_factory=dict)
    shared: Dict[str, Any] = field(default_factory=dict)
    # simty.clock.SimClock -- Gazebo's /clock, the domain every bridged camera
    # frame is stamped in. None on a build with no simulator attached.
    sim_clock: Any = None
    log: Optional[Callable[[str, str], None]] = None
    journal: Any = None  # simty.journal.ConversionJournal
    route: str = "convert"
    direction: str = ""

    def p(self, name: str, default):
        """Read a tunable parameter, falling back to a compiled-in default."""
        value = self.params.get(name, default)
        return default if value is None else value

    def ros_stamp(self) -> RosTime:
        """
        Stamp for std_msgs/Header on host-bound messages.

        Gazebo's /clock when it is arriving and stamp_source allows it, wall
        clock otherwise. This used to be unconditionally wall clock, on the
        reasoning that "the Manifold side lives in wall time" -- true of real
        hardware, and wrong here: every image ros_gz_bridge delivers carries
        Gazebo sim time, so a wall-stamped IMU could not be paired with a frame
        at all once the real-time factor fell below 1. simty/clock.py has the
        long version.

        PX4's own `timestamp` fields are unaffected and still use ctx.clock
        (Px4Clock), which is a third, offset domain -- see _new_setpoint.
        """
        if self.p("stamp_source", "auto") != "wall" and self.sim_clock is not None:
            sim_ns = self.sim_clock.now_ns()
            if sim_ns is not None:
                return RosTime(sec=int(sim_ns // 1_000_000_000),
                               nanosec=int(sim_ns % 1_000_000_000))
        now = time.time()
        return RosTime(sec=int(now), nanosec=int((now % 1.0) * 1e9))

    def note(self, message: str) -> None:
        if self.log:
            self.log("convert", message)

    def warn(self, code: str, detail: str = "", level: Optional[int] = None) -> None:
        """
        Report a conversion compromise by code. Rate limiting, counting and
        text output all happen in the journal, so a converter on a 50 Hz path
        can call this unconditionally.
        """
        if self.journal is not None:
            self.journal.report(self.route, self.direction, code, detail, level)


def _g(msg, name, default=0.0):
    """getattr for PX4 messages, tolerant of field renames across releases."""
    value = getattr(msg, name, None)
    return default if value is None else value


def _finite(value) -> bool:
    """True for a real number that is neither NaN nor infinite."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def _deadzone(value, width):
    """Remove stick slop, then rescale so full deflection still reaches 1.0."""
    if width <= 0.0:
        return value
    if abs(value) <= width:
        return 0.0
    return (value - math.copysign(width, value)) / (1.0 - width)


def _wrap_pi(angle: float) -> float:
    """Fold an angle in radians into [-pi, pi]."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler(q):
    """PX4 quaternion [w, x, y, z] -> (roll, pitch, yaw) in radians, NED/FRD."""
    try:
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    except (TypeError, IndexError, ValueError):
        return 0.0, 0.0, 0.0
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(_clamp(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quat_msg_to_yaw(quaternion) -> Optional[float]:
    """
    geometry_msgs/Quaternion -> yaw in radians, or None when unusable.

    Separate from quat_to_euler because that one takes PX4's [w, x, y, z] array
    while ROS message fields are named x/y/z/w -- reading one with the other's
    convention swaps the axes and produces a plausible, wrong heading. node.py
    uses this for the wrapper's attitude topic.
    """
    if quaternion is None:
        return None
    try:
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
    except (AttributeError, TypeError, ValueError):
        return None
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not _finite(norm) or norm == 0.0:
        return None
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ── HOST -> SIM ──────────────────────────────────────────────────────────────


def joy_to_trajectory_setpoint(joy: Joy, ctx: ConversionContext):
    """
    DJI RC sticks -> PX4 TrajectorySetpoint.

    DJI axis order (psdk_wrapper telemetry.cpp): [roll, pitch, yaw, throttle],
    each in [-10000, 10000].

    PX4's TrajectorySetpoint.velocity is in the LOCAL NED frame, not the body
    frame. Pilots expect "forward" to mean "where the nose points", so we
    rotate the body-frame stick vector by the current heading. The heading is
    published into ctx.shared by node.py's own attitude subscription, which is
    deliberately independent of the route table -- it used to be a side effect of
    the gimbal_angles route, so pointing that route host->sim silently reverted
    stick control to head-north. If attitude has not arrived yet we fall back to
    yaw=0, which degrades to head-north control rather than to nonsense.
    """
    from px4_msgs.msg import TrajectorySetpoint

    raw = list(joy.axes)
    if len(raw) < 4:
        ctx.warn("rc_axes_short", f"Joy carried {len(raw)} axes, expected 4")
    axes = raw + [0.0] * 4
    rc_max = float(ctx.p("rc_max", 10000.0)) or 1.0
    dz = float(ctx.p("deadzone", 0.02))

    scaled = []
    for index, name in enumerate(("roll", "pitch", "yaw", "throttle")):
        value = axes[index] / rc_max
        if abs(value) > 1.0:
            ctx.warn(
                "rc_axis_clipped",
                f"{name} stick at {axes[index]:+.0f} exceeds rc_max={rc_max:g}",
            )
        scaled.append(_deadzone(_clamp(value, -1.0, 1.0), dz))
    roll_n, pitch_n, yaw_n, thr_n = scaled

    max_speed = float(ctx.p("max_speed", 15.0))
    max_climb = float(ctx.p("max_climb", 5.0))
    max_yaw_rate = float(ctx.p("max_yaw_rate", 1.5))

    v_fwd = pitch_n * max_speed
    v_right = roll_n * max_speed

    # The rotation from body frame into NED is only as good as the heading. Both
    # ways it can be wrong are silent by nature, so both are reported: no
    # attitude at all degrades to head-north control, and a stale attitude
    # rotates the sticks by where the nose used to point.
    yaw = ctx.shared.get("yaw")
    if yaw is None:
        ctx.warn("rc_heading_unknown")
        yaw = 0.0
    else:
        yaw = float(yaw)
        heading_at = float(ctx.shared.get("yaw_at", 0.0))
        stale_after = float(ctx.p("rc_heading_stale_s", 2.0))
        if heading_at and (time.time() - heading_at) > stale_after:
            ctx.warn(
                "rc_heading_stale",
                f"heading is {time.time() - heading_at:.1f}s old "
                f"(limit {stale_after:g}s)",
            )
    if not bool(ctx.shared.get("rc_authority", False)):
        ctx.warn("rc_no_authority")
    if ctx.clock is not None and not getattr(ctx.clock, "synced", True):
        ctx.warn("clock_unsynced")

    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    msg = TrajectorySetpoint()
    msg.timestamp = ctx.clock.now_us()
    nan = float("nan")
    msg.position = [nan, nan, nan]
    msg.velocity = [
        v_fwd * cos_y - v_right * sin_y,  # north
        v_fwd * sin_y + v_right * cos_y,  # east
        -thr_n * max_climb,  # down (NED): stick up -> negative
    ]
    msg.acceleration = [nan, nan, nan]
    msg.yaw = nan
    msg.yawspeed = yaw_n * max_yaw_rate
    # Kept for the control room: the RViz vehicle view draws this as an arrow, so
    # a stick input that produces the wrong NED vector is visible immediately.
    ctx.shared["rc_cmd"] = (
        msg.velocity[0],
        msg.velocity[1],
        msg.velocity[2],
        msg.yawspeed,
    )
    return msg


def joy_passthrough(joy: Joy, ctx: ConversionContext):
    """
    Republish the RC sticks on /drone/rc unchanged.

    drone_controller.py subscribes to /drone/rc for its stick display, and
    nothing published it before, so that display was permanently dead.
    """
    out = Joy()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = "dji_rc"
    out.axes = list(joy.axes)
    out.buttons = list(joy.buttons)
    return out


# ── psdk_ros2 flight control setpoints -> PX4 ────────────────────────────────
#
# These three are what a PSDK application actually publishes to fly an aircraft:
# psdk_ros2 hands them straight to DJI's joystick API. Until they were bridged,
# the only input that moved this simulation was the RC stick channel, so every
# project had to pretend to be a radio.
#
# DJI's joystick semantics are NOT "a setpoint in a world frame", and copying
# the numbers across would fly the wrong path:
#
#   * horizontal position axes are OFFSETS from where the aircraft is now,
#   * the vertical axis is an ABSOLUTE height above the takeoff point,
#   * yaw is an ABSOLUTE angle, in degrees.
#
# psdk_ros2 names the horizontal pair ENU, so axes[0] is east and axes[1] is
# north. PX4 wants NED metres and radians. Yaw is the one genuinely ambiguous
# field -- DJI documents its ground-frame yaw as clockwise from north, which is
# PX4's convention, while the ENU naming of the topic suggests counter-clockwise
# from east. `setpoint_yaw_frame` picks one ("ned", the default, or "enu") and
# the conversion is reported the first time it runs, so a project that assumed
# the other one finds out from the journal instead of from the flight path.


def _setpoint_axes(joy: Joy, ctx: ConversionContext, what: str):
    """The four axes psdk_ros2 puts in these messages, or None if short."""
    raw = list(joy.axes)
    if len(raw) < 4:
        ctx.warn("setpoint_axes_short", f"{what} carried {len(raw)} axes, expected 4")
        return None
    axes = [float(value) for value in raw[:4]]
    if not all(_finite(value) for value in axes):
        ctx.warn("setpoint_nonfinite", f"{what} axes={axes}")
        return None
    return axes


def _setpoint_yaw_rad(value_deg: float, ctx: ConversionContext) -> float:
    """DJI ground-frame yaw in degrees -> PX4 NED yaw in radians."""
    frame = str(ctx.p("setpoint_yaw_frame", "ned")).lower()
    if frame == "enu":
        ctx.warn(
            "setpoint_yaw_frame",
            "reading yaw as ENU (0 = east, counter-clockwise) and converting to "
            "NED; set setpoint_yaw_frame=ned if the project sends DJI's "
            "clockwise-from-north angle",
        )
        return math.radians(90.0 - value_deg)
    return math.radians(value_deg)


def _setpoint_yaw_rate_rad(value_deg: float, ctx: ConversionContext) -> float:
    frame = str(ctx.p("setpoint_yaw_frame", "ned")).lower()
    rate = math.radians(value_deg)
    return -rate if frame == "enu" else rate


def _new_setpoint(ctx: ConversionContext):
    from px4_msgs.msg import TrajectorySetpoint

    nan = float("nan")
    msg = TrajectorySetpoint()
    msg.timestamp = ctx.clock.now_us()
    msg.position = [nan, nan, nan]
    msg.velocity = [nan, nan, nan]
    msg.acceleration = [nan, nan, nan]
    msg.yaw = nan
    msg.yawspeed = nan
    return msg


def psdk_position_yaw_to_setpoint(joy: Joy, ctx: ConversionContext):
    """
    flight_control_setpoint_ENUposition_yaw -> PX4 TrajectorySetpoint (position).

    axes = [east offset (m), north offset (m), height above takeoff (m), yaw (deg)]

    The horizontal pair is relative, so this needs to know where the aircraft
    is: it reads the NED position the position_fused route publishes into shared
    state. With that route off there is nothing to add the offset to, and the
    sample is dropped rather than guessed at -- which is why project testing
    mode turns position_fused on with these.
    """
    axes = _setpoint_axes(joy, ctx, "ENUposition_yaw")
    if axes is None:
        return None
    east_offset, north_offset, height_up, yaw_deg = axes

    ned = ctx.shared.get("ned")
    if ned is None:
        ctx.warn("setpoint_needs_position")
        return None
    north_now, east_now, _ = ned

    limit = float(ctx.p("setpoint_max_offset", 200.0))
    if max(abs(east_offset), abs(north_offset)) > limit:
        ctx.warn(
            "setpoint_clipped",
            f"offset ({north_offset:+.1f} N, {east_offset:+.1f} E) m exceeds "
            f"setpoint_max_offset={limit:g} m and was clamped",
        )
        north_offset = _clamp(north_offset, -limit, limit)
        east_offset = _clamp(east_offset, -limit, limit)

    ceiling = float(ctx.p("setpoint_max_altitude", 120.0))
    if height_up > ceiling:
        ctx.warn(
            "setpoint_clipped",
            f"height {height_up:.1f} m exceeds setpoint_max_altitude={ceiling:g} m",
        )
        height_up = ceiling

    msg = _new_setpoint(ctx)
    # NED: z is DOWN, and PX4's local origin is the takeoff point -- which is
    # exactly the reference DJI's absolute height uses.
    msg.position = [
        north_now + north_offset,
        east_now + east_offset,
        -height_up,
    ]
    msg.yaw = _setpoint_yaw_rad(yaw_deg, ctx)
    return msg


def psdk_enu_velocity_to_setpoint(joy: Joy, ctx: ConversionContext):
    """
    flight_control_setpoint_ENUvelocity_yawrate -> TrajectorySetpoint (velocity).

    axes = [v east (m/s), v north (m/s), v up (m/s), yaw rate (deg/s)]

    Ground-frame velocity, so no heading is needed -- this is the setpoint topic
    to prefer when the aircraft's yaw is unknown or unreliable.
    """
    axes = _setpoint_axes(joy, ctx, "ENUvelocity_yawrate")
    if axes is None:
        return None
    v_east, v_north, v_up, yaw_rate_deg = axes

    limit = float(ctx.p("setpoint_max_speed", 15.0))
    speed = math.hypot(v_north, v_east)
    if speed > limit:
        ctx.warn(
            "setpoint_clipped",
            f"{speed:.1f} m/s exceeds setpoint_max_speed={limit:g} m/s and was scaled back",
        )
        scale = limit / speed
        v_north *= scale
        v_east *= scale

    climb_limit = float(ctx.p("setpoint_max_climb", 5.0))
    if abs(v_up) > climb_limit:
        ctx.warn(
            "setpoint_clipped",
            f"climb {v_up:+.1f} m/s exceeds setpoint_max_climb={climb_limit:g} m/s",
        )
        v_up = _clamp(v_up, -climb_limit, climb_limit)

    msg = _new_setpoint(ctx)
    msg.velocity = [v_north, v_east, -v_up]   # ENU -> NED
    msg.yawspeed = _setpoint_yaw_rate_rad(yaw_rate_deg, ctx)
    return msg


def psdk_flu_velocity_to_setpoint(joy: Joy, ctx: ConversionContext):
    """
    flight_control_setpoint_FLUvelocity_yawrate -> TrajectorySetpoint (velocity).

    axes = [v forward (m/s), v left (m/s), v up (m/s), yaw rate (deg/s)]

    Body frame, so the heading is load-bearing exactly as it is for the RC
    sticks: without it "forward" silently becomes "north". node.py owns that
    subscription, independently of the route table, so this cannot be broken by
    pointing some other route the wrong way -- but it can still be too old to
    trust, and a stale heading is reported rather than used quietly.
    """
    axes = _setpoint_axes(joy, ctx, "FLUvelocity_yawrate")
    if axes is None:
        return None
    v_forward, v_left, v_up, yaw_rate_deg = axes

    yaw = ctx.shared.get("yaw")
    if yaw is None:
        ctx.warn("setpoint_heading_unknown")
        yaw = 0.0
    else:
        yaw = float(yaw)
        heading_at = float(ctx.shared.get("yaw_at", 0.0))
        stale_after = float(ctx.p("rc_heading_stale_s", 2.0))
        if heading_at and (time.time() - heading_at) > stale_after:
            ctx.warn(
                "rc_heading_stale",
                f"heading is {time.time() - heading_at:.1f}s old (limit {stale_after:g}s)",
            )

    limit = float(ctx.p("setpoint_max_speed", 15.0))
    speed = math.hypot(v_forward, v_left)
    if speed > limit:
        ctx.warn(
            "setpoint_clipped",
            f"{speed:.1f} m/s exceeds setpoint_max_speed={limit:g} m/s and was scaled back",
        )
        scale = limit / speed
        v_forward *= scale
        v_left *= scale

    climb_limit = float(ctx.p("setpoint_max_climb", 5.0))
    v_up = _clamp(v_up, -climb_limit, climb_limit)

    # FLU -> FRD, then body -> NED by the heading.
    v_right = -v_left
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    msg = _new_setpoint(ctx)
    msg.velocity = [
        v_forward * cos_y - v_right * sin_y,
        v_forward * sin_y + v_right * cos_y,
        -v_up,
    ]
    msg.yawspeed = _setpoint_yaw_rate_rad(yaw_rate_deg, ctx)
    return msg


def control_mode_to_authority(msg: ControlMode, ctx: ConversionContext):
    """
    ControlMode -> /drone/rc/authority.

    device_mode: 0=RC, 1=MSDK, 4=PSDK. RC holds authority at 0. control_auth
    is a separate "has authority" flag; when the operator asks for it to be
    honoured, both must agree.
    """
    device_mode = int(_g(msg, "device_mode", 0))
    control_auth = int(_g(msg, "control_auth", 1))
    rc_mode = device_mode == int(ctx.p("rc_device_mode", 0))
    if rc_mode and control_auth != 1:
        ctx.warn(
            "authority_flag_conflict",
            f"device_mode={device_mode} (RC) but control_auth={control_auth}",
        )
    if bool(ctx.p("require_control_auth", False)):
        rc_mode = rc_mode and control_auth == 1
    if not rc_mode:
        ctx.warn(
            "authority_denied",
            f"device_mode={device_mode} "
            f"(0=RC 1=MSDK 4=PSDK), rc_device_mode={int(ctx.p('rc_device_mode', 0))}",
        )
    ctx.shared["rc_authority"] = rc_mode
    ctx.shared["device_mode"] = int(_g(msg, "device_mode", 0))
    return Bool(data=rc_mode)


def _gimbal_joint_outputs(ctx: ConversionContext, yaw, roll, pitch):
    """
    Clamp a (yaw, roll, pitch) request in radians to the m4e's mechanical limits
    from models/m4e/model.sdf and turn it into the three joint setpoints.

    Shared by both gimbal converters so the limits, the warning code and the
    RViz feedback are identical no matter which wrapper topic drove the gimbal.
    The Gazebo joints are `gimbal_yaw_joint` (pan), `gimbal_roll_joint` and
    `gimbal_tilt_joint`, each driven by a JointPositionController listening on
    the topic named below.
    """
    pan_lim = math.radians(float(ctx.p("gimbal_pan_deg", 60.0)))
    roll_lim = math.radians(float(ctx.p("gimbal_roll_deg", 47.0)))
    tilt_min = math.radians(float(ctx.p("gimbal_tilt_min_deg", -90.0)))
    tilt_max = math.radians(float(ctx.p("gimbal_tilt_max_deg", 35.0)))

    axes = (
        ("pan", yaw, -pan_lim, pan_lim),
        ("roll", roll, -roll_lim, roll_lim),
        ("tilt", pitch, tilt_min, tilt_max),
    )
    values = {}
    for axis, requested, low, high in axes:
        requested = float(requested)
        clamped = _clamp(requested, low, high)
        if clamped != requested:
            ctx.warn(
                "gimbal_axis_clamped",
                f"{axis} asked for {math.degrees(requested):+.1f}deg, "
                f"limit is {math.degrees(low):+.1f}..{math.degrees(high):+.1f}deg",
            )
        values[axis] = clamped

    # drives the RViz gimbal_link frame (viz.py)
    ctx.shared["gimbal_cmd"] = (values["pan"], values["roll"], values["tilt"])
    ctx.shared["gimbal_cmd_at"] = time.time()

    return [
        ("/drone/gimbal/cmd/pan", Float64(data=values["pan"])),
        ("/drone/gimbal/cmd/roll", Float64(data=values["roll"])),
        ("/drone/gimbal/cmd/tilt", Float64(data=values["tilt"])),
    ]


def gimbal_rotation_to_joint_cmds(msg, ctx: ConversionContext):
    """
    DJI GimbalRotation -> the three Gazebo joint setpoints.

    Fan-out converter: one input, three outputs on three different topics.

    NOTE ON REAL HARDWARE: gimbal_rotation is an *input* to psdk_ros2 -- the
    real wrapper subscribes to it in order to command the payload and never
    publishes it. Against a live Manifold this route therefore has no source and
    stays silent; it is useful when something sim-side (or the mock) publishes
    gimbal commands. To make the simulated gimbal follow the real one, use the
    gimbal_angles route and gimbal_angles_to_joint_cmds below.
    """
    # rotation_mode 1 is absolute, 0 is incremental. Gazebo's joint position
    # controllers take absolute setpoints, so an incremental request is applied
    # as absolute -- workable, but the far side must know it happened.
    if int(_g(msg, "rotation_mode", 1)) != 1:
        ctx.warn("gimbal_mode_unsupported", f"rotation_mode={int(_g(msg, 'rotation_mode', 1))}")

    requested = (("yaw", _g(msg, "yaw", 0.0)),
                 ("roll", _g(msg, "roll", 0.0)),
                 ("pitch", _g(msg, "pitch", 0.0)))
    for field_name, value in requested:
        if not _finite(value):
            ctx.warn("gimbal_nonfinite", f"{field_name}={value}")
            return None

    return _gimbal_joint_outputs(
        ctx, requested[0][1], requested[1][1], requested[2][1]
    )


def gimbal_angles_to_joint_cmds(msg, ctx: ConversionContext):
    """
    Wrapper gimbal_angles (geometry_msgs/Vector3Stamped, radians) -> the three
    Gazebo joint setpoints. This is the route that makes the simulated gimbal
    actually follow the real one.

    WHY THIS TOPIC AND NOT gimbal_rotation
    --------------------------------------
    gimbal_rotation is a command psdk_ros2 *consumes*; a live Manifold reports
    zero publishers on it, so a host->sim route from it can never fire however
    correct it looks. gimbal_angles is the measured payload attitude the wrapper
    does publish (~35 Hz on the m4e), which makes it the only topic that can
    drive the simulated gimbal.

    FRAMES -- THE PART THAT MATTERS
    ------------------------------
    DJI reports gimbal attitude in the GROUND frame: roll and pitch relative to
    the horizon, yaw relative to North. Gazebo's gimbal_yaw_joint is relative to
    the AIRFRAME. Copying yaw straight through therefore pins the joint at its
    +-60 deg limit whenever the aircraft is not pointing north -- which is how a
    "working" conversion produces a gimbal that never moves and a clamp warning
    on every sample. The default subtracts the aircraft's own heading, giving the
    joint angle the real gimbal is actually holding.

    `gimbal_yaw_reference` picks the subtrahend:

      aircraft -- the wrapper's own attitude (default; the only choice that is
                  right when the real and simulated aircraft face different ways)
      sim      -- the simulated vehicle's heading
      none     -- copy yaw through untouched, for a wrapper already reporting
                  body-relative yaw

    Roll and pitch are passed through: both are horizon-relative on the DJI side
    and the simulated airframe is close enough to level for the difference not to
    be worth inventing state for. A tilted airframe makes the simulated camera
    point a few degrees off, and that is a smaller lie than dropping the sample.
    """
    vector = getattr(msg, "vector", None)
    if vector is None:
        ctx.warn("gimbal_angles_malformed", "message has no `vector` field")
        return None

    roll_in = _g(vector, "x", 0.0)
    pitch_in = _g(vector, "y", 0.0)
    yaw_in = _g(vector, "z", 0.0)
    for name, value in (("roll", roll_in), ("pitch", pitch_in), ("yaw", yaw_in)):
        if not _finite(value):
            ctx.warn("gimbal_nonfinite", f"{name}={value}")
            return None
    yaw_in = float(yaw_in)

    yaw_in = _gimbal_yaw_shift(ctx, yaw_in, sign=-1.0)

    ctx.shared["gimbal_measured"] = (float(roll_in), float(pitch_in), float(yaw_in))
    return _gimbal_joint_outputs(ctx, yaw_in, roll_in, -pitch_in)


def _gimbal_yaw_shift(ctx: ConversionContext, yaw: float, sign: float) -> float:
    """
    Move a gimbal yaw between the ground frame and the airframe.

    sign = -1  ground -> body   (host->sim: subtract the aircraft heading)
    sign = +1  body   -> ground (sim->host: add it back)

    Both directions MUST come through here. They used to be written separately
    and the sim->host side never shifted at all, so a value could go out to the
    wrapper and come back as a different angle -- the round trip was not the
    identity, which is exactly the symptom of a gimbal that drifts or sits on a
    limit. One function with a sign makes the two provably inverse.

    `gimbal_yaw_reference` picks the heading to shift by:

      aircraft -- the wrapper's own attitude (default; the only choice that is
                  right when the real and simulated aircraft face different ways)
      sim      -- the simulated vehicle's heading
      none     -- no shift, for a wrapper already reporting body-relative yaw
    """
    reference = str(ctx.p("gimbal_yaw_reference", "aircraft")).strip().lower()
    if reference == "aircraft":
        heading = ctx.shared.get("host_yaw")
        heading_at = float(ctx.shared.get("host_yaw_at", 0.0) or 0.0)
        source = "the wrapper's attitude topic"
    elif reference == "sim":
        heading = ctx.shared.get("yaw")
        heading_at = float(ctx.shared.get("yaw_at", 0.0) or 0.0)
        source = "simulated attitude"
    else:
        if reference != "none":
            ctx.warn(
                "gimbal_yaw_reference_unknown",
                f"gimbal_yaw_reference={reference!r}; expected aircraft|sim|none, "
                "treating as none",
            )
        return float(yaw)

    if heading is None:
        # Raw ground-frame yaw is nearly certain to sit on the pan limit, so say
        # why rather than leaving the operator to infer it from the clamp
        # warning that follows.
        ctx.warn(
            "gimbal_heading_unknown",
            f"no heading from {source} yet; using yaw unshifted, which will clamp "
            "to the pan limit unless the aircraft faces north",
        )
        return float(yaw)

    age = time.time() - heading_at if heading_at else 0.0
    if heading_at and age > float(ctx.p("gimbal_heading_stale_s", 2.0)):
        ctx.warn("gimbal_heading_stale", f"heading from {source} is {age:.1f}s old")
    return _wrap_pi(float(yaw) + sign * float(heading))


# ── SIM -> HOST ──────────────────────────────────────────────────────────────

# PX4 nav_state -> DJI display mode. Values that have no sensible DJI analogue
# fall through to P_GPS, which is what a DJI operator sees during normal
# GPS-assisted flight.
_NAV_TO_DISPLAY = {
    0: DisplayMode.DISPLAY_MODE_MANUAL_CTRL,  # MANUAL
    1: DisplayMode.DISPLAY_MODE_ATTITUDE,  # ALTCTL
    2: DisplayMode.DISPLAY_MODE_P_GPS,  # POSCTL
    3: DisplayMode.DISPLAY_MODE_NAVI_SDK_CTRL,  # AUTO_MISSION
    4: DisplayMode.DISPLAY_MODE_P_GPS,  # AUTO_LOITER
    5: DisplayMode.DISPLAY_MODE_NAVI_GO_HOME,  # AUTO_RTL
    10: DisplayMode.DISPLAY_MODE_ATTITUDE,  # ACRO
    12: DisplayMode.DISPLAY_MODE_AUTO_LANDING,  # DESCEND
    13: DisplayMode.DISPLAY_MODE_FORCE_AUTO_LANDING,  # TERMINATION
    14: DisplayMode.DISPLAY_MODE_NAVI_SDK_CTRL,  # OFFBOARD
    15: DisplayMode.DISPLAY_MODE_ATTITUDE,  # STAB
    17: DisplayMode.DISPLAY_MODE_AUTO_TAKEOFF,  # AUTO_TAKEOFF
    18: DisplayMode.DISPLAY_MODE_AUTO_LANDING,  # AUTO_LAND
}

_ARMED = 2  # VehicleStatus.ARMING_STATE_ARMED


def vehicle_status_to_flight_status(msg, ctx: ConversionContext):
    """
    VehicleStatus -> FlightStatus.

    FlightStatus is a three-state enum: STOPED(0) / ON_GROUND(1) / ON_AIR(2).
    Disarmed means motors stopped; armed on the ground means ON_GROUND; armed
    and airborne means ON_AIR. "Airborne" comes from the land detector when
    the land-detected route is running, otherwise from arming alone.
    """
    armed = int(_g(msg, "arming_state", 1)) == _ARMED
    ctx.shared["armed"] = armed
    ctx.shared["nav_state"] = int(_g(msg, "nav_state", 0))
    ctx.shared["failsafe"] = bool(_g(msg, "failsafe", False))
    # PX4 v1.14+ only. None means "this release does not report it", which is not
    # the same as False and must not be shown as a failure.
    preflight = _g(msg, "pre_flight_checks_pass", None)
    ctx.shared["preflight_ok"] = None if preflight is None else bool(preflight)

    out = FlightStatus()
    out.header.stamp = ctx.ros_stamp()
    if armed and "landed" not in ctx.shared:
        ctx.warn("landed_unknown")
    if not armed:
        out.flight_status = FlightStatus.FLIGHT_STATUS_STOPED
    elif ctx.shared.get("landed", not armed):
        out.flight_status = FlightStatus.FLIGHT_STATUS_ON_GROUND
    else:
        out.flight_status = FlightStatus.FLIGHT_STATUS_ON_AIR
    return out


def vehicle_status_to_display_mode(msg, ctx: ConversionContext):
    """VehicleStatus.nav_state -> DJI display mode."""
    nav = int(_g(msg, "nav_state", 0))
    out = DisplayMode()
    out.header.stamp = ctx.ros_stamp()
    if nav not in _NAV_TO_DISPLAY:
        ctx.warn("nav_state_unmapped", f"PX4 nav_state={nav}")
    out.display_mode = _NAV_TO_DISPLAY.get(nav, DisplayMode.DISPLAY_MODE_P_GPS)
    return out


def land_detected_to_shared(msg, ctx: ConversionContext):
    """
    Feed the land detector into shared state for flight_status, and mirror it
    to /drone/landed so the rest of the sim can use it. Not a DJI message --
    this route exists purely to make FlightStatus honest.
    """
    landed = bool(_g(msg, "landed", False))
    ctx.shared["landed"] = landed
    return Bool(data=landed)


def battery_status_to_single_battery(msg, ctx: ConversionContext):
    """BatteryStatus -> SingleBatteryInfo (percentages are 0-100 on the DJI side)."""
    out = SingleBatteryInfo()
    out.header.stamp = ctx.ros_stamp()
    out.battery_index = int(ctx.p("battery_index", 0))

    # voltage_v is current PX4; voltage_filtered_v was the pre-1.14 name.
    voltage = float(_g(msg, "voltage_v", 0.0)) or float(_g(msg, "voltage_filtered_v", 0.0))
    if voltage == 0.0:
        ctx.warn("battery_voltage_zero")
    out.voltage = voltage
    out.current = float(_g(msg, "current_a", 0.0))

    remaining = float(_g(msg, "remaining", -1.0))  # PX4: 0..1, -1 when unknown
    if remaining < 0.0:
        # DJI's capacity_percentage has no "unknown" value, so 0 is the only
        # option -- and 0 is indistinguishable from a flat pack to the client.
        ctx.warn("battery_remaining_unknown")
    out.capacity_percentage = _clamp(remaining * 100.0, 0.0, 100.0) if remaining >= 0.0 else 0.0

    design = float(ctx.p("battery_capacity_mah", 5000.0))
    discharged = float(_g(msg, "discharged_mah", 0.0))
    out.full_capacity = design
    out.capacity_remain = max(0.0, design - discharged)

    temperature = float(_g(msg, "temperature", float("nan")))
    if not (_finite(temperature) and temperature > -100.0):
        ctx.warn("battery_temp_invalid", f"temperature={temperature}")
        temperature = 25.0
    out.temperature = temperature
    out.cell_count = int(_g(msg, "cell_count", 0))
    ctx.shared["battery_pct"] = out.capacity_percentage
    return out


def global_position_to_navsatfix(msg, ctx: ConversionContext):
    """
    VehicleGlobalPosition -> sensor_msgs/NavSatFix.

    This is the correct type for psdk_ros2's gps_position and rtk_position.
    eph/epv are 1-sigma metres, so the covariance diagonal is their squares.
    """
    out = NavSatFix()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("gps_frame_id", "map"))

    latitude = _g(msg, "lat", 0.0)
    longitude = _g(msg, "lon", 0.0)
    altitude = _g(msg, "alt", 0.0)
    if not (_finite(latitude) and _finite(longitude) and _finite(altitude)):
        ctx.warn("gps_nonfinite", f"lat={latitude} lon={longitude} alt={altitude}")
        return None
    out.latitude = float(latitude)
    out.longitude = float(longitude)
    out.altitude = float(altitude)
    # Exactly (0, 0) is PX4 saying "no global origin yet", not a position in the
    # Gulf of Guinea. A client that plots it will fly the map to Null Island.
    if out.latitude == 0.0 and out.longitude == 0.0:
        ctx.warn("gps_origin_zero")
    ctx.shared["lat"] = out.latitude
    ctx.shared["lon"] = out.longitude
    ctx.shared["alt"] = out.altitude

    eph = float(_g(msg, "eph", 0.0))
    epv = float(_g(msg, "epv", 0.0))
    out.position_covariance = [
        eph * eph, 0.0, 0.0,
        0.0, eph * eph, 0.0,
        0.0, 0.0, epv * epv,
    ]
    if eph > 0.0 and epv > 0.0:
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    else:
        # A zero diagonal tagged DIAGONAL_KNOWN is a claim of perfect accuracy.
        # UNKNOWN is the honest encoding of "PX4 did not tell us".
        ctx.warn("gps_accuracy_unknown", f"eph={eph:g} epv={epv:g}")
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    out.status.service = NavSatStatus.SERVICE_GPS
    fix = int(ctx.shared.get("gps_fix_type", 3))
    if fix < 2:
        ctx.warn("gps_no_fix", f"fix_type={fix}")
    out.status.status = NavSatStatus.STATUS_FIX if fix >= 2 else NavSatStatus.STATUS_NO_FIX
    return out


def sensor_gps_to_navsatfix(msg, ctx: ConversionContext):
    """
    SensorGps -> sensor_msgs/NavSatFix. The RAW receiver fix.

    WHY THIS EXISTS ALONGSIDE global_position_to_navsatfix
    ------------------------------------------------------
    DJI publishes two different things and psdk_ros2 keeps them apart:
    `gps_position` is what the GNSS receiver reported, and
    `gps_position_fused` is the flight controller's fused estimate. Both routes
    here used to read VehicleGlobalPosition, so `gps_position` was in fact the
    EKF output wearing the raw topic's name.

    That is not merely imprecise, it is circular for anything that fuses GNSS
    with inertial data: PX4's EKF has already folded the IMU into that number,
    so an estimator "fusing" it is re-using its own prior and an evaluation
    against it measures nothing. VehicleGlobalPosition is also smooth, which
    hides exactly the scatter a GNSS factor's covariance is meant to model.

    PX4 FIELD DRIFT
    ---------------
    SensorGps changed encoding: it used to carry lat/lon as int32 in units of
    1e-7 deg and alt as int32 millimetres; newer releases carry latitude_deg /
    longitude_deg / altitude_msl_m as float64 degrees and metres. PX4 is cloned
    unpinned at image build time, so both are read and whichever exists wins.

    This deliberately does NOT write ctx.shared["lat"/"lon"/"alt"]. Those feed
    the status displays and set_home_from_gps, and they should keep showing the
    best available position, which is the fused one. The raw fix lands on its
    own keys so it stays inspectable.
    """
    out = NavSatFix()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("gps_frame_id", "map"))

    # Newer float64 degrees/metres first, then the legacy scaled integers.
    latitude = _g(msg, "latitude_deg", None)
    longitude = _g(msg, "longitude_deg", None)
    altitude = _g(msg, "altitude_msl_m", None)
    if latitude is None or longitude is None:
        latitude = float(_g(msg, "lat", 0)) * 1e-7
        longitude = float(_g(msg, "lon", 0)) * 1e-7
        altitude = float(_g(msg, "alt", 0)) * 1e-3  # mm -> m
    if altitude is None:
        altitude = float(_g(msg, "alt", 0)) * 1e-3

    if not (_finite(latitude) and _finite(longitude) and _finite(altitude)):
        ctx.warn("gps_nonfinite", f"lat={latitude} lon={longitude} alt={altitude}")
        return None
    out.latitude = float(latitude)
    out.longitude = float(longitude)
    out.altitude = float(altitude)
    if out.latitude == 0.0 and out.longitude == 0.0:
        ctx.warn("gps_origin_zero")
    ctx.shared["gps_raw_lat"] = out.latitude
    ctx.shared["gps_raw_lon"] = out.longitude
    ctx.shared["gps_raw_alt"] = out.altitude

    eph = float(_g(msg, "eph", 0.0))
    epv = float(_g(msg, "epv", 0.0))
    out.position_covariance = [
        eph * eph, 0.0, 0.0,
        0.0, eph * eph, 0.0,
        0.0, 0.0, epv * epv,
    ]
    if eph > 0.0 and epv > 0.0:
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    else:
        ctx.warn("gps_accuracy_unknown", f"eph={eph:g} epv={epv:g}")
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    # Read fix_type off the message rather than ctx.shared, so this route does
    # not silently depend on gps_details being enabled.
    fix = int(_g(msg, "fix_type", 3))
    ctx.shared["gps_fix_type"] = fix
    out.status.service = NavSatStatus.SERVICE_GPS
    if fix >= 4:
        # 4 RTCM code differential, 5 RTK float, 6 RTK fixed. All are corrected
        # from a ground station, which is what GBAS denotes in REP-hood terms.
        out.status.status = NavSatStatus.STATUS_GBAS_FIX
    elif fix >= 2:
        out.status.status = NavSatStatus.STATUS_FIX
    else:
        ctx.warn("gps_no_fix", f"fix_type={fix}")
        out.status.status = NavSatStatus.STATUS_NO_FIX
    return out


def sensor_gps_to_gps_details(msg, ctx: ConversionContext):
    """
    SensorGps -> GPSDetails.

    GPSDetails carries quality metrics only: DOP (unit 0.01, so 1.00 -> 100),
    accuracies in mm (cm/s for speed), and satellite counts. It has no
    position fields at all -- position rides on NavSatFix.
    """
    out = GPSDetails()
    out.header.stamp = ctx.ros_stamp()

    hdop = float(_g(msg, "hdop", 0.0))
    vdop = float(_g(msg, "vdop", 0.0))
    out.horizontal_dop = hdop * 100.0
    out.position_dop = math.hypot(hdop, vdop) * 100.0

    fix = int(_g(msg, "fix_type", 3))
    ctx.shared["gps_fix_type"] = fix
    out.fix_state = float(fix)
    if fix < 2:
        ctx.warn("gps_no_fix", f"fix_type={fix}")

    out.horizontal_accuracy = float(_g(msg, "eph", 0.0)) * 1000.0  # m -> mm
    out.vertical_accuracy = float(_g(msg, "epv", 0.0)) * 1000.0
    out.speed_accuracy = float(_g(msg, "s_variance_m_s", 0.0)) * 100.0  # m/s -> cm/s

    sats = int(_g(msg, "satellites_used", 0))
    out.num_gps_satellites_used = sats
    out.num_glonass_satellites_used = 0
    out.num_total_satellites_used = sats
    ctx.shared["gps_counter"] = int(ctx.shared.get("gps_counter", 0)) + 1
    out.gps_counter = ctx.shared["gps_counter"] & 0xFFFF
    ctx.shared["sats"] = sats
    return out


def local_position_to_position_fused(msg, ctx: ConversionContext):
    """VehicleLocalPosition -> PositionFused (NED metres + per-axis validity)."""
    out = PositionFused()
    out.header.stamp = ctx.ros_stamp()
    out.position = Point(
        x=float(_g(msg, "x", 0.0)),
        y=float(_g(msg, "y", 0.0)),
        z=float(_g(msg, "z", 0.0)),
    )
    xy_valid = bool(_g(msg, "xy_valid", False))
    z_valid = bool(_g(msg, "z_valid", False))
    xy_ok = 1 if xy_valid else 0
    out.x_health = xy_ok
    out.y_health = xy_ok
    out.z_health = 1 if z_valid else 0
    if not (xy_valid and z_valid):
        ctx.warn(
            "local_pos_invalid",
            f"xy_valid={xy_valid} z_valid={z_valid}; values forwarded with "
            "health 0",
        )

    ctx.shared["ned"] = (out.position.x, out.position.y, out.position.z)
    heading = _g(msg, "heading", None)
    if heading is None:
        pass  # this PX4 release does not carry heading here; attitude supplies it
    elif _finite(heading):
        ctx.shared["yaw"] = float(heading)
        ctx.shared["yaw_at"] = time.time()
    else:
        ctx.warn("heading_nonfinite", f"heading={heading}")
    return out


def gimbal_joints_to_gimbal_angles(msg, ctx: ConversionContext):
    """
    The simulated gimbal's joint setpoints -> geometry_msgs/Vector3Stamped, the
    type psdk_ros2 uses for gimbal_angles. The exact inverse of
    gimbal_angles_to_joint_cmds.

    The source topic is /drone/gimbal/cmd/pan and `msg` is that Float64. Roll
    and tilt come from shared state, which node.py fills from the other two
    joint topics (_subscribe_gimbal_joints) -- a route has only one source, and
    all three axes are needed to build one Vector3Stamped.

    WHAT THIS REPLACES, AND WHY IT WAS WRONG
    ----------------------------------------
    This route used to run attitude_to_gimbal_angles, which read
    /fmu/out/vehicle_attitude -- the AIRFRAME's quaternion -- and published the
    aircraft's own roll/pitch/yaw as though they were gimbal angles. Two
    consequences, both of which look like "the shift is not working":

      * the simulated gimbal's pan never reached gimbal_angles at all, because
        the pan joint was not an input to the conversion. Moving the gimbal in
        drone_controller.py changed nothing on the wrapper side.
      * no heading shift was applied. The forward path subtracts the aircraft
        heading to turn DJI's ground-frame yaw into a body-relative joint angle,
        so the reverse path has to add it back. It did not, so a round trip
        ground -> joint -> ground returned a different angle whenever the
        aircraft was not facing north.

    AXES
    ----
    Matching gimbal_angles_to_joint_cmds exactly, so the two compose to the
    identity: vector.x is roll, vector.y is pitch (the tilt joint), vector.z is
    yaw (the pan joint, shifted back into the ground frame).
    """
    pan = getattr(msg, "data", None)
    if pan is None:
        ctx.warn("gimbal_joint_malformed", "pan setpoint has no `data` field")
        return None

    roll = ctx.shared.get("gimbal_joint_roll", 0.0)
    tilt = ctx.shared.get("gimbal_joint_tilt", 0.0)
    for name, value in (("pan", pan), ("roll", roll), ("tilt", tilt)):
        if not _finite(value):
            ctx.warn("gimbal_nonfinite", f"{name}={value}")
            return None

    # Body-relative joint angle back into DJI's ground frame. Same helper as the
    # forward path with the sign flipped, so the two cannot drift apart.
    yaw = _gimbal_yaw_shift(ctx, float(pan), sign=+1.0)

    out = Vector3Stamped()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("gimbal_frame_id", "gimbal"))
    out.vector.x = float(roll)
    out.vector.y = -float(tilt)
    out.vector.z = float(yaw)
    return out


def attitude_to_gimbal_status(msg, ctx: ConversionContext):
    """
    Synthesise a healthy GimbalStatus.

    GimbalStatus is entirely flags -- mounted, busy, per-axis limit hit,
    calibration, ESC health. The simulated gimbal is always mounted and never
    faults, so this reports a nominal payload. Angles do NOT belong here; see
    gimbal_joints_to_gimbal_angles.
    """
    out = GimbalStatus()
    out.header.stamp = ctx.ros_stamp()
    out.mount_status = 1
    out.is_busy = 0
    out.pitch_limited = 0
    out.roll_limited = 0
    out.yaw_limited = 0
    out.calibrating = 0
    out.prev_calibration_result = 1
    out.installed_direction = 0
    out.disabled_mvo = 0
    out.gear_show_unable = 0
    out.gyro_falut = 0
    out.esc_pitch_status = 1
    out.esc_roll_status = 1
    out.esc_yaw_status = 1
    out.drone_data_recv = 1
    out.init_unfinished = 1
    out.fw_updating = 0
    return out


def laserscan_to_obstacle_info(msg, ctx: ConversionContext):
    """
    The m4e's downward laser range finder -> RelativeObstacleInfo.

    Only the `down` sector is real; the other five directions are reported as
    not-working (health 0) rather than as a fabricated clear path, which would
    be actively dangerous to trust.
    """
    ranges = [r for r in getattr(msg, "ranges", []) if _finite(r) and r > 0.0]
    down = min(ranges) if ranges else 0.0
    if not ranges:
        ctx.warn(
            "lrf_no_returns",
            f"{len(getattr(msg, 'ranges', []))} samples, none usable",
        )
    ctx.shared["lrf_range"] = down

    out = RelativeObstacleInfo()
    out.header.stamp = ctx.ros_stamp()
    out.down = float(down)
    out.front = 0.0
    out.right = 0.0
    out.back = 0.0
    out.left = 0.0
    out.up = 0.0
    out.down_health = 1 if ranges else 0
    out.front_health = 0
    out.right_health = 0
    out.back_health = 0
    out.left_health = 0
    out.up_health = 0
    out.reserved = 0
    return out


def px4_home_to_psdk_home(msg, ctx: ConversionContext):
    """px4_msgs/HomePosition -> psdk_interfaces/HomePosition (lat/lon only)."""
    out = PsdkHomePosition()
    out.header.stamp = ctx.ros_stamp()
    valid = bool(_g(msg, "valid_hpos", True))
    if not valid:
        ctx.warn("home_invalid")
    out.home_position_status = (
        PsdkHomePosition.HOME_POSITION_STATUS_SUCCESS
        if valid
        else PsdkHomePosition.HOME_POSITION_STATUS_FAILED
    )
    out.latitude = float(_g(msg, "lat", 0.0))
    out.longitude = float(_g(msg, "lon", 0.0))
    ctx.shared["home"] = (out.latitude, out.longitude)
    return out


def vehicle_status_to_rc_connection(msg, ctx: ConversionContext):
    """
    Report the RC link as seen from the sim.

    "Connected" here means the bridge is actually receiving stick data, which
    is the only thing the simulation can honestly assert about the RC link.
    """
    fresh_for = float(ctx.p("rc_fresh_s", 2.0))
    last_rc = float(ctx.shared.get("last_rc_time", 0.0))
    linked = (time.time() - last_rc) < fresh_for if last_rc else False

    out = RCConnectionStatus()
    out.header.stamp = ctx.ros_stamp()
    out.air_connection = 1 if linked else 0
    out.ground_connection = 1 if linked else 0
    out.app_connection = 0
    out.air_or_ground_disconnected = 0 if linked else 1
    return out


def vehicle_status_to_flight_anomaly(msg, ctx: ConversionContext):
    """VehicleStatus failsafe flags -> FlightAnomaly (all-zero when nominal)."""
    out = FlightAnomaly()
    out.header.stamp = ctx.ros_stamp()
    failsafe = bool(_g(msg, "failsafe", False))
    out.impact_in_air = 0
    out.random_fly = 0
    out.height_ctrl_fail = 0
    out.roll_pitch_ctrl_fail = 0
    out.yaw_ctrl_fail = 0
    out.aircraft_is_falling = 1 if failsafe else 0
    out.strong_wind_level1 = 0
    out.strong_wind_level2 = 0
    out.compass_installation_error = 0
    out.imu_installation_error = 0
    out.esc_temperature_high = 0
    out.at_least_one_esc_disconnected = 0
    out.gps_yaw_error = 0
    out.reserved = 0
    return out


# ── frame conventions used by the converters below ───────────────────────────
#
# PX4 works in NED for position/velocity and FRD (forward-right-down) for body
# rates and accelerations. psdk_ros2 follows REP-103: ENU for anything in the
# world frame and FLU (forward-left-up) for body frames. Every conversion below
# that crosses that boundary goes through one of these two helpers rather than
# open-coding the sign flips, because getting one axis wrong produces data that
# looks entirely plausible until something tries to fly on it.


def _ned_to_enu(north, east, down):
    """(N, E, D) -> (E, N, U). World-frame vectors: PX4 -> REP-103."""
    return float(east), float(north), -float(down)


def _frd_to_flu(x, y, z):
    """(F, R, D) -> (F, L, U). Body-frame vectors: PX4 -> REP-103."""
    return float(x), -float(y), -float(z)


_SQRT_HALF = math.sqrt(2.0) / 2.0


def _px4_quat_to_ros(quaternion):
    """
    PX4's [w, x, y, z] (NED->FRD) -> ROS (x, y, z, w) (ENU->FLU), or None.

    This is the rotation the real wrapper applies (psdk_ros2 telemetry.cpp):

        R_FLU2ENU = R_NED2ENU * R_FRD2NED * R_FLU2FRD

    As quaternions that is  q_A (x) q (x) q_B,  where q_A is the 180-degree
    turn about (1,1,0)/sqrt(2) taking NED to ENU and q_B is the 180-degree turn
    about x taking FLU to FRD. Multiplying the two out gives the four terms
    below, with a = sqrt(2)/2:

        w' = a(w + z)   x' = a(x + y)   y' = a(x - y)   z' = a(w - z)

    THIS IS NOT THE COMPONENT SWAP (y, x, -z, w) THAT USED TO BE HERE. That
    swap is the NED/ENU rule for *vectors*, and applying it to a quaternion is
    not the same operation -- it does not reproduce R_FLU2ENU for any
    orientation with a nonzero heading. Concretely it left a vehicle pointing
    north reporting an ENU yaw of 0 instead of +90 degrees, i.e. it reported
    north as east. The check that catches this: for level flight the two
    conventions must satisfy yaw_enu = 90 degrees - yaw_ned.

    One copy, because there were two and they disagreed -- odometry_to_nav_-
    odometry used the swap and attitude_to_quaternion_stamped did not convert
    at all, so the simulated `attitude` topic reported a heading in a different
    frame from the one a real Manifold reports under the same name.
    """
    if quaternion is None or len(quaternion) < 4:
        return None
    try:
        w, x, y, z = (float(quaternion[index]) for index in range(4))
    except (TypeError, ValueError, IndexError):
        return None
    if not all(_finite(component) for component in (w, x, y, z)):
        return None
    a = _SQRT_HALF
    return a * (x + y), a * (x - y), a * (w - z), a * (w + z)


# ── passthrough / mirror ─────────────────────────────────────────────────────


def passthrough(msg, ctx: ConversionContext):
    """
    Hand a message across unchanged.

    Used by the `mirror` route group, where both endpoints carry the same type
    and the only thing the bridge adds is the crossing itself plus the route's
    rate limit. Returning the very same object is safe: rclpy serialises on
    publish and the converter contract forbids mutation.
    """
    return msg


def image_passthrough(msg, ctx: ConversionContext):
    """
    sensor_msgs/Image across the bridge, in either direction.

    Works both ways because both endpoints are Image: pointed host->sim it makes
    the real payload video available inside the simulation graph; pointed
    sim->host it publishes simulated video under the wrapper's camera name, which
    is what lets a PSDK or MSDK client see a picture from the simulation.

    WHY THESE ROUTES DEFAULT TO OFF
    -------------------------------
    An uncompressed 1080p frame is ~6 MB, which is ~4500 fragments at the
    1400-byte cap in config/dds.xml. Two of these enabled at once will saturate
    the wireless link to the Manifold and starve the telemetry and control routes
    that share it. Enable one at a time, keep max_hz low, and prefer the
    `preview` tier. The frame budget below reports when a single frame is large
    enough that the rate limit is doing the real work.
    """
    step = int(_g(msg, "step", 0))
    height = int(_g(msg, "height", 0))
    payload = step * height
    if payload <= 0:
        ctx.warn("image_empty", f"step={step} height={height}; nothing to forward")
        return None

    budget = int(float(ctx.p("image_warn_bytes", 2_000_000)))
    if payload > budget:
        ctx.warn(
            "image_frame_large",
            f"{payload / 1e6:.1f} MB per frame "
            f"({int(_g(msg, 'width', 0))}x{height} {_g(msg, 'encoding', '?')}) "
            f"exceeds image_warn_bytes={budget / 1e6:.1f} MB",
        )
    return msg


# ── SIM -> HOST: the rest of the wrapper's telemetry surface ─────────────────


def attitude_to_quaternion_stamped(msg, ctx: ConversionContext):
    """
    VehicleAttitude -> geometry_msgs/QuaternionStamped, as psdk_ros2 publishes
    `attitude`.

    FRAME: ENU->FLU, per REP-103, which is what the real wrapper publishes on
    this topic. This used to emit PX4's NED->FRD quaternion unconverted -- only
    the [w,x,y,z] to x/y/z/w field reordering was applied -- so the simulated
    `attitude` topic and a real Manifold's disagreed by the NED/ENU flip while
    wearing the same name. node._subscribe_host_attitude reads this topic back
    to derive the gimbal's yaw reference, so in a sim-only session that heading
    was 90 degrees out from the same code path on hardware.
    """
    converted = _px4_quat_to_ros(_g(msg, "q", None))
    if converted is None:
        ctx.warn("attitude_missing_q")
        return None
    x, y, z, w = converted

    out = QuaternionStamped()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("body_frame_id", "base_link"))
    out.quaternion.x, out.quaternion.y = x, y
    out.quaternion.z, out.quaternion.w = z, w
    return out


def global_position_to_altitude_msl(msg, ctx: ConversionContext):
    """VehicleGlobalPosition.alt -> Float32 metres above mean sea level."""
    altitude = _g(msg, "alt", 0.0)
    if not _finite(altitude):
        ctx.warn("altitude_nonfinite", f"alt={altitude}")
        return None
    if not bool(_g(msg, "alt_valid", True)):
        ctx.warn("altitude_invalid", "PX4 reports alt_valid=false")
    return Float32(data=float(altitude))


def global_position_to_altitude_barometric(msg, ctx: ConversionContext):
    """
    VehicleGlobalPosition.alt -> Float32, standing in for the barometer.

    The EKF's fused altitude is not a raw barometric reading: it has GPS and
    (on the m4e) rangefinder folded in, so it lacks the drift and the
    weather-driven offset a real barometer shows. Reported once per route so a
    consumer testing barometer behaviour knows it is looking at a substitute.
    """
    ctx.warn("altitude_baro_substituted")
    return global_position_to_altitude_msl(msg, ctx)


def local_position_to_height_above_ground(msg, ctx: ConversionContext):
    """
    VehicleLocalPosition -> Float32 height above ground.

    Prefers `dist_bottom` (the downward rangefinder, which is what DJI's
    height_above_ground actually reports) and falls back to -z, the EKF's height
    above its local origin. Those differ over any terrain that is not flat, so
    the fallback is reported rather than blended.
    """
    if bool(_g(msg, "dist_bottom_valid", False)):
        distance = _g(msg, "dist_bottom", 0.0)
        if _finite(distance):
            return Float32(data=float(distance))
        ctx.warn("hagl_nonfinite", f"dist_bottom={distance}")

    down = _g(msg, "z", 0.0)
    if not _finite(down):
        ctx.warn("hagl_nonfinite", f"z={down}")
        return None
    ctx.warn(
        "hagl_from_ekf",
        "no valid rangefinder return; using -z above the EKF origin, which is "
        "height above the takeoff point rather than above the terrain",
    )
    return Float32(data=-float(down))


def local_position_to_velocity_ground(msg, ctx: ConversionContext):
    """VehicleLocalPosition -> Vector3Stamped ground velocity in ENU (REP-103)."""
    if not bool(_g(msg, "v_xy_valid", True)):
        ctx.warn("velocity_invalid", "PX4 reports v_xy_valid=false")
    north, east, down = _g(msg, "vx", 0.0), _g(msg, "vy", 0.0), _g(msg, "vz", 0.0)
    if not all(_finite(value) for value in (north, east, down)):
        ctx.warn("velocity_nonfinite", f"vx={north} vy={east} vz={down}")
        return None

    out = Vector3Stamped()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("gps_frame_id", "map"))
    out.vector.x, out.vector.y, out.vector.z = _ned_to_enu(north, east, down)
    return out


def local_position_to_gps_velocity(msg, ctx: ConversionContext):
    """VehicleLocalPosition -> TwistStamped, the type psdk_ros2 uses for
    gps_velocity. Linear only: PX4's local position carries no body rates."""
    if not bool(_g(msg, "v_xy_valid", True)):
        ctx.warn("velocity_invalid", "PX4 reports v_xy_valid=false")
    north, east, down = _g(msg, "vx", 0.0), _g(msg, "vy", 0.0), _g(msg, "vz", 0.0)
    if not all(_finite(value) for value in (north, east, down)):
        ctx.warn("velocity_nonfinite", f"vx={north} vy={east} vz={down}")
        return None

    out = TwistStamped()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("gps_frame_id", "map"))
    east_e, north_n, up = _ned_to_enu(north, east, down)
    out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = east_e, north_n, up
    return out


def angular_velocity_to_vector3(msg, ctx: ConversionContext):
    """
    Body angular rates -> Vector3Stamped, FRD -> FLU.

    Reads `gyro_rad` (SensorCombined) or `xyz` (VehicleAngularVelocity),
    whichever the message has. SensorCombined is the source that matters in
    practice: /fmu/out/vehicle_angular_velocity is commented out in PX4's
    dds_topics.yaml, so a route pointed at it would sit at zero messages forever
    with nothing to say why. Accepting both means uncommenting that line is all
    it takes to switch.
    """
    rates = _g(msg, "gyro_rad", None)
    if rates is None:
        rates = _g(msg, "xyz", None)
    if rates is None or len(rates) < 3:
        ctx.warn("angular_rate_missing", "neither gyro_rad nor xyz is present")
        return None
    if not all(_finite(value) for value in rates[:3]):
        ctx.warn("angular_rate_nonfinite", f"xyz={list(rates[:3])}")
        return None

    out = Vector3Stamped()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("body_frame_id", "base_link"))
    out.vector.x, out.vector.y, out.vector.z = _frd_to_flu(*rates[:3])
    return out


def sensor_combined_to_imu(msg, ctx: ConversionContext):
    """
    SensorCombined (+ VehicleAttitude) -> sensor_msgs/Imu, FRD -> FLU.

    ORIENTATION IS SPLICED IN, and the previous revision deliberately did not
    do that. Its reasoning was sound in the abstract -- SensorCombined is raw
    gyro and accelerometer, it carries no orientation, and REP-145 says to
    advertise that with orientation_covariance[0] = -1 rather than invent one.

    It is nonetheless wrong for THIS interface. psdk_ros2's `imu` topic is fed
    from DJI's HARD_SYNC subscription, which delivers quaternion, acceleration
    and angular rate together as one sample, and the wrapper populates
    orientation from it (telemetry.cpp, imu_callback). So on a real M4E this
    topic has an orientation and here it did not: code that reads it works on
    hardware and silently sees an identity quaternion in the simulator, which
    is the exact failure this rig exists to prevent. Matching the interface
    beats matching the provenance of the underlying uORB topic.

    The splice is honest about being one: orientation comes from the most
    recent VehicleAttitude, and if that is missing or older than
    imu_attitude_stale_s the message reverts to the REP-145 "no orientation"
    encoding rather than shipping a stale attitude as if it were current.

    COVARIANCES ARE LEFT AT ZERO, deliberately, because the real wrapper leaves
    them at zero (all three arrays, verified in telemetry.cpp). The simulator
    knows its own noise -- it is in models/m4e/model.sdf -- and filling it in
    here would hand consumers weights that simply do not exist on the aircraft.
    An estimator that read them would then divide by zero, or take zero to mean
    infinite confidence, the first time it ran on hardware. The SDF values are
    carried in the estimator's own config instead; see
    demo_gnss_stereo_inertial.
    """
    accel = _g(msg, "accelerometer_m_s2", None)
    gyro = _g(msg, "gyro_rad", None)
    if accel is None or gyro is None or len(accel) < 3 or len(gyro) < 3:
        ctx.warn("imu_missing_fields", "SensorCombined lacks accelerometer or gyro")
        return None
    if not all(_finite(value) for value in list(accel[:3]) + list(gyro[:3])):
        ctx.warn("imu_nonfinite")
        return None

    out = Imu()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("body_frame_id", "base_link"))
    out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = (
        _frd_to_flu(*accel[:3])
    )
    out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = (
        _frd_to_flu(*gyro[:3])
    )
    quaternion = ctx.shared.get("q_ned_frd")
    quaternion_at = float(ctx.shared.get("q_ned_frd_at", 0.0) or 0.0)
    age = (time.time() - quaternion_at) if quaternion_at else None
    stale_after = float(ctx.p("imu_attitude_stale_s", 0.2))

    converted = _px4_quat_to_ros(quaternion) if quaternion is not None else None
    if converted is None:
        ctx.warn("imu_no_orientation", "no VehicleAttitude seen yet")
    elif age is not None and age > stale_after:
        ctx.warn("imu_no_orientation", f"attitude is {age:.2f}s old (> {stale_after:g}s)")
        converted = None

    if converted is None:
        out.orientation.w = 1.0
        out.orientation_covariance[0] = -1.0  # REP-145 "no orientation here"
    else:
        (out.orientation.x, out.orientation.y,
         out.orientation.z, out.orientation.w) = converted
    return out


# ── the rest of the aircraft's telemetry surface ─────────────────────────────
#
# Eight topics a Matrice 4E publishes that this bridge did not. None of them is
# hard to produce -- they are mostly PX4 fields the simulation already carries,
# reshaped. What matters is that they EXIST: a payload that subscribes to
# gps_signal_level on the aircraft and gets nothing here has been silently
# lied to about what the simulation can stand in for.
#
# Where the simulation genuinely has no equivalent (radar-based RTK yaw, a
# landing gear the m4e does not have) the converter says so through the journal
# and publishes the honest resting value, rather than leaving the topic absent.


def local_position_to_angular_rate_ground(msg, ctx: ConversionContext):
    """
    VehicleAngularVelocity or SensorCombined -> Vector3Stamped, GROUND frame.

    The ground-frame twin of angular_rate_body_raw. Rotating body rates into the
    world needs the attitude, which node.py keeps in shared state; without a
    recent one the rates are passed through unrotated and the journal says so,
    because a silently body-framed "ground" rate is worse than a gap.
    """
    rates = _g(msg, "gyro_rad", None)
    if rates is None:
        rates = _g(msg, "xyz", None)
    if rates is None or len(rates) < 3 or not all(_finite(v) for v in rates[:3]):
        ctx.warn("angular_rate_missing", "no usable body rates")
        return None

    forward, left, up = _frd_to_flu(*rates[:3])
    quaternion = ctx.shared.get("q_ned_frd")
    converted = _px4_quat_to_ros(quaternion) if quaternion is not None else None

    out = Vector3Stamped()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("gps_frame_id", "map"))
    if converted is None:
        ctx.warn("angular_rate_no_attitude",
                 "no attitude yet, so body rates are reported unrotated")
        out.vector.x, out.vector.y, out.vector.z = forward, left, up
        return out

    x, y, z, w = converted
    # Rotate the body-frame rate vector into the world by the body orientation.
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        out.vector.x, out.vector.y, out.vector.z = forward, left, up
        return out
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    r00 = 1 - 2 * (y * y + z * z); r01 = 2 * (x * y - w * z); r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z); r11 = 1 - 2 * (x * x + z * z); r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y); r21 = 2 * (y * z + w * x); r22 = 1 - 2 * (x * x + y * y)
    out.vector.x = r00 * forward + r01 * left + r02 * up
    out.vector.y = r10 * forward + r11 * left + r12 * up
    out.vector.z = r20 * forward + r21 * left + r22 * up
    return out


def sensor_gps_to_signal_level(msg, ctx: ConversionContext):
    """
    SensorGps -> UInt8, DJI's 0..5 GPS signal level.

    DJI reports signal quality as a single number a pilot can read off a screen;
    PX4 reports a fix type and a satellite count. The mapping below is a
    judgement, not a conversion -- there is no published table -- so it is
    written where it can be argued with:

        0  no fix at all
        1  a fix, but fewer than 6 satellites: unusable for position hold
        2  6-8 satellites
        3  9-11 satellites
        4  12+ satellites
        5  12+ satellites AND an RTK or differential fix
    """
    fix = int(_g(msg, "fix_type", 0))
    satellites = int(_g(msg, "satellites_used", 0))
    if fix < 2:
        level = 0
    elif fix >= 5:
        level = 5 if satellites >= 12 else 4
    elif satellites >= 12:
        level = 4
    elif satellites >= 9:
        level = 3
    elif satellites >= 6:
        level = 2
    else:
        level = 1
    ctx.shared["gps_signal_level"] = level
    return UInt8(data=level)


def sensor_gps_to_control_level(msg, ctx: ConversionContext):
    """
    SensorGps -> UInt8, DJI's "is the GPS good enough to fly on" level.

    Same 0..5 scale as the signal level and, on a real aircraft, usually the
    same number -- it differs only when the flight controller distrusts a
    signal it can nonetheless see. This simulation has no such distrust model,
    so it reports the signal level and says as much.
    """
    level = int(ctx.shared.get("gps_signal_level", 0))
    ctx.warn("gps_control_level_copied",
             "control level is reported as the signal level; the simulation has "
             "no separate notion of GPS trustworthiness")
    return UInt8(data=level)


def vehicle_status_to_landing_gear(msg, ctx: ConversionContext):
    """
    VehicleStatus -> UInt8 landing gear state.

    THE M4E HAS FIXED LANDING GEAR. It does not retract, so the only honest
    answer is "deployed", always. The topic exists because the aircraft
    publishes it and a payload may subscribe; the value is constant because the
    hardware is.
    """
    return UInt8(data=0)   # 0 = deployed, and it never changes


def vehicle_status_to_motor_start_error(msg, ctx: ConversionContext):
    """
    VehicleStatus -> UInt16 motor start error code.

    DJI publishes a numbered reason the motors would not start. PX4 publishes a
    set of failsafe booleans instead, which node.py already collects into
    shared["arm_blockers"]. There is no mapping between the two numbering
    schemes, so rather than invent DJI codes this reports 0 (no error) when PX4
    has no blockers and 1 (a generic "cannot start") when it does, and names the
    real PX4 reasons in the journal where they can actually be read.
    """
    blockers = ctx.shared.get("arm_blockers") or []
    if not blockers:
        return UInt16(data=0)
    ctx.warn("motor_start_error_generic",
             "PX4 blocks arming for: " + ", ".join(blockers[:4]) +
             " -- reported as generic code 1, DJI's own codes do not map")
    return UInt16(data=1)


def sensor_gps_to_rtk_connection(msg, ctx: ConversionContext):
    """
    SensorGps -> UInt16, "is an RTK base station connected" (1 or 0).

    True only for PX4 fix types that require corrections from a base station
    (4 = RTCM code differential, 5 = RTK float, 6 = RTK fixed). The simulated
    GNSS never reaches those, so this is False in practice -- which is the
    truthful answer for a simulation with no base station in it.
    """
    # UInt16 rather than Bool, because that is the type the mirror route for
    # this same topic already declares -- the two must not disagree about what
    # a payload will receive.
    return UInt16(data=1 if int(_g(msg, "fix_type", 0)) >= 4 else 0)


def sensor_gps_to_rtk_position_info(msg, ctx: ConversionContext):
    """
    SensorGps -> UInt8, RTK solution quality.

    DJI's scale: 0 none, 1 single, 2 float, 3 fixed. Derived from PX4's fix
    type, which tops out at 3 (plain 3D) in this simulation, so this reports 1.
    """
    fix = int(_g(msg, "fix_type", 0))
    if fix >= 6:
        return UInt8(data=3)
    if fix == 5:
        return UInt8(data=2)
    if fix >= 2:
        return UInt8(data=1)
    return UInt8(data=0)


def sensor_gps_to_rtk_yaw_info(msg, ctx: ConversionContext):
    """
    SensorGps -> UInt8, quality of the RTK dual-antenna heading.

    ALWAYS 0, AND THAT IS NOT A PLACEHOLDER. RTK yaw comes from the baseline
    between two antennas. There is one simulated GNSS antenna, so there is no
    baseline and no heading to report. A payload that needs RTK yaw cannot be
    developed against this simulation, and saying 0 here is how it finds that
    out immediately rather than after a field trip.
    """
    return UInt8(data=0)


def battery_status_to_battery_state(msg, ctx: ConversionContext):
    """
    BatteryStatus -> sensor_msgs/BatteryState, which is what psdk_ros2 publishes
    on `battery` (the psdk_interfaces SingleBatteryInfo form goes out on
    single_battery_index2 instead; see battery_status_to_single_battery).
    """
    out = BatteryState()
    out.header.stamp = ctx.ros_stamp()
    out.voltage = float(_g(msg, "voltage_v", 0.0))
    out.current = -float(_g(msg, "current_a", 0.0))  # ROS: discharge is negative
    out.temperature = float(_g(msg, "temperature", 0.0)) or float("nan")

    remaining = float(_g(msg, "remaining", -1.0))
    if remaining < 0.0:
        ctx.warn("battery_remaining_unknown", "PX4 reports remaining<0")
        out.percentage = float("nan")
    else:
        out.percentage = _clamp(remaining, 0.0, 1.0)

    capacity_mah = float(ctx.p("battery_capacity_mah", 5000.0))
    out.design_capacity = capacity_mah / 1000.0  # Ah
    discharged_mah = float(_g(msg, "discharged_mah", 0.0))
    out.charge = max(0.0, (capacity_mah - discharged_mah) / 1000.0)
    out.capacity = float("nan")
    out.present = bool(_g(msg, "connected", True))
    cells = int(_g(msg, "cell_count", 0))
    if cells > 0:
        # PX4 array fields arrive as numpy arrays, so this cannot be written as
        # `_g(...) or []`: numpy raises on the truth test of a multi-element array.
        voltages = _g(msg, "voltage_cell_v", None)
        if voltages is not None:
            out.cell_voltage = [float(value) for value in list(voltages)[:cells]]
    out.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    out.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
    out.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
    return out


def px4_home_to_navsatfix(msg, ctx: ConversionContext):
    """HomePosition -> NavSatFix, the type psdk_ros2 uses for home_point."""
    if not bool(_g(msg, "valid_hpos", False)):
        ctx.warn("home_invalid", "PX4 reports valid_hpos=false")
        return None
    latitude, longitude = _g(msg, "lat", 0.0), _g(msg, "lon", 0.0)
    if not (_finite(latitude) and _finite(longitude)):
        ctx.warn("home_nonfinite", f"lat={latitude} lon={longitude}")
        return None

    out = NavSatFix()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("gps_frame_id", "map"))
    out.status.status = NavSatStatus.STATUS_FIX
    out.status.service = NavSatStatus.SERVICE_GPS
    out.latitude = float(latitude)
    out.longitude = float(longitude)
    out.altitude = float(_g(msg, "alt", 0.0))
    return out


def px4_home_to_altitude(msg, ctx: ConversionContext):
    """HomePosition.alt -> Float32, psdk_ros2's home_point_altitude."""
    if not bool(_g(msg, "valid_alt", False)):
        ctx.warn("home_invalid", "PX4 reports valid_alt=false")
        return None
    altitude = _g(msg, "alt", 0.0)
    if not _finite(altitude):
        ctx.warn("home_nonfinite", f"alt={altitude}")
        return None
    return Float32(data=float(altitude))


def px4_home_to_status(msg, ctx: ConversionContext):
    """HomePosition -> Bool, psdk_ros2's home_point_status (is home set)."""
    return Bool(data=bool(_g(msg, "valid_hpos", False)) and bool(_g(msg, "valid_alt", False)))


def odometry_to_nav_odometry(msg, ctx: ConversionContext):
    """
    VehicleOdometry -> nav_msgs/Odometry, psdk_ros2's visual_odometry.

    PX4 position and velocity are NED and the orientation quaternion is
    NED->FRD, so both are converted to REP-103. The covariance blocks are left
    at zero: PX4 publishes per-axis variances rather than a 6x6, and spreading
    three numbers across a matrix would invent correlations that were never
    estimated. Consumers that need covariance should read the variances from the
    PX4 topic directly.
    """
    position = _g(msg, "position", None)
    velocity = _g(msg, "velocity", None)
    quaternion = _g(msg, "q", None)
    if position is None or len(position) < 3 or quaternion is None or len(quaternion) < 4:
        ctx.warn("odometry_missing_fields")
        return None
    if not all(_finite(value) for value in list(position[:3]) + list(quaternion[:4])):
        ctx.warn("odometry_nonfinite")
        return None

    out = Odometry()
    out.header.stamp = ctx.ros_stamp()
    out.header.frame_id = str(ctx.p("gps_frame_id", "map"))
    out.child_frame_id = str(ctx.p("body_frame_id", "base_link"))

    east, north, up = _ned_to_enu(position[0], position[1], position[2])
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (
        east, north, up
    )

    x, y, z, w = _px4_quat_to_ros(quaternion)
    out.pose.pose.orientation.x = x
    out.pose.pose.orientation.y = y
    out.pose.pose.orientation.z = z
    out.pose.pose.orientation.w = w

    if velocity is not None and len(velocity) >= 3 and all(
        _finite(value) for value in velocity[:3]
    ):
        vx, vy, vz = _ned_to_enu(velocity[0], velocity[1], velocity[2])
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.linear.z = vz
    else:
        ctx.warn("odometry_no_velocity", "velocity absent or non-finite; reported as zero")

    rates = _g(msg, "angular_velocity", None)
    if rates is not None and len(rates) >= 3 and all(_finite(value) for value in rates[:3]):
        wx, wy, wz = _frd_to_flu(*rates[:3])
        out.twist.twist.angular.x = wx
        out.twist.twist.angular.y = wy
        out.twist.twist.angular.z = wz
    return out


# ── payload summaries, for the live-data debug view ───────────────────────────


def summarise(msg) -> str:
    """
    One-line human-readable digest of a message, shown in the TUI's live-data
    column. Deliberately best-effort: never raises, never blocks.
    """
    try:
        name = type(msg).__name__
        if isinstance(msg, Joy):
            axes = " ".join(f"{a:+.0f}" for a in list(msg.axes)[:4])
            return f"axes[{axes}]"
        if isinstance(msg, Bool):
            return f"{msg.data}"
        if isinstance(msg, (Float64, Float32)):
            return f"{msg.data:+.3f}"
        if isinstance(msg, Image):
            size = int(_g(msg, "step", 0)) * int(_g(msg, "height", 0))
            return (f"{int(_g(msg, 'width', 0))}x{int(_g(msg, 'height', 0))} "
                    f"{_g(msg, 'encoding', '?')} {size / 1e6:.1f}MB")
        if isinstance(msg, QuaternionStamped):
            yaw = quat_msg_to_yaw(msg.quaternion)
            return "yaw=?" if yaw is None else f"yaw={math.degrees(yaw):+.1f}"
        if isinstance(msg, Imu):
            a, g = msg.linear_acceleration, msg.angular_velocity
            return (f"a=[{a.x:+.1f} {a.y:+.1f} {a.z:+.1f}] "
                    f"w=[{g.x:+.2f} {g.y:+.2f} {g.z:+.2f}]")
        if isinstance(msg, TwistStamped):
            v = msg.twist.linear
            return f"v=[{v.x:+.1f} {v.y:+.1f} {v.z:+.1f}]"
        if isinstance(msg, BatteryState):
            percent = msg.percentage * 100.0
            shown = "?" if math.isnan(percent) else f"{percent:.0f}%"
            return f"{msg.voltage:.1f}V {msg.current:+.1f}A {shown}"
        if isinstance(msg, Odometry):
            p_ = msg.pose.pose.position
            return f"enu=[{p_.x:+.1f} {p_.y:+.1f} {p_.z:+.1f}]"
        if isinstance(msg, String):
            return msg.data[:40]
        if isinstance(msg, NavSatFix):
            return f"{msg.latitude:.6f},{msg.longitude:.6f} alt={msg.altitude:.1f}"
        if isinstance(msg, Vector3Stamped):
            v = msg.vector
            return f"r={math.degrees(v.x):+.1f} p={math.degrees(v.y):+.1f} y={math.degrees(v.z):+.1f}"
        if isinstance(msg, FlightStatus):
            return {0: "STOPPED", 1: "ON_GROUND", 2: "ON_AIR"}.get(msg.flight_status, "?")
        if isinstance(msg, DisplayMode):
            return f"display_mode={msg.display_mode}"
        if isinstance(msg, SingleBatteryInfo):
            return f"{msg.voltage:.1f}V {msg.current:+.1f}A {msg.capacity_percentage:.0f}%"
        if isinstance(msg, ControlMode):
            return f"device_mode={msg.device_mode} auth={msg.control_auth}"
        if isinstance(msg, GPSDetails):
            return f"fix={msg.fix_state:.0f} sats={msg.num_total_satellites_used} hdop={msg.horizontal_dop / 100:.2f}"
        if isinstance(msg, PositionFused):
            p = msg.position
            return f"n={p.x:+.1f} e={p.y:+.1f} d={p.z:+.1f}"
        if isinstance(msg, RelativeObstacleInfo):
            return f"down={msg.down:.2f}m"

        # PX4 and anything else: pick a few interesting scalars.
        for candidates in (
            ("arming_state", "nav_state"),
            ("lat", "lon", "alt"),
            ("x", "y", "z"),
            ("velocity",),
            ("landed",),
        ):
            if all(hasattr(msg, c) for c in candidates):
                parts = []
                for c in candidates:
                    value = getattr(msg, c)
                    if isinstance(value, (list, tuple)) or hasattr(value, "__len__"):
                        parts.append(f"{c}=[" + ",".join(f"{float(v):+.1f}" for v in value) + "]")
                    elif isinstance(value, float):
                        parts.append(f"{c}={value:+.2f}")
                    else:
                        parts.append(f"{c}={value}")
                return " ".join(parts)
        return name
    except Exception:  # a debug view must never take the bridge down
        return "<unsummarisable>"
