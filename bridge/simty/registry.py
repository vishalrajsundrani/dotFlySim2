"""
The route tables. This is the file you edit to add a bridge.

A TopicRoute row fully describes one bridged topic: both endpoints, both types,
which way it flows, how to translate, what QoS to use on each side and how hard
to rate-limit it. node.py turns rows into live endpoints; tui.py renders and
edits them. Neither needs to know what any particular row means.

A ServiceRoute row describes one bridged service. Two roles:

  SERVE  -- we stand up the /wrapper/psdk_ros2/<name> server ourselves and
            translate each request into simulator actions. This is the mode
            that lets PSDK/MSDK clients fly the simulation.
  PROXY  -- the real Manifold owns the server; we expose a sim-side client and
            forward. Use when you want the simulation to command real hardware.

SERVE and the real wrapper cannot both own a name on one DDS domain. node.py
detects a remote server during discovery and warns; the operator flips the role
from the SERVICES screen.

TYPE AVAILABILITY
-----------------
px4_msgs contents vary by PX4 release, so message classes are resolved through
_px4() and a row whose type is missing is reported as unavailable instead of
crashing the process at import.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, List, Tuple

from . import converters as cv
from .state import INFO, WARN


# ── direction ────────────────────────────────────────────────────────────────


class Direction(Enum):
    OFF = "off"
    HOST_TO_SIM = "host->sim"
    SIM_TO_HOST = "sim->host"

    @property
    def arrow(self) -> str:
        return {"off": "--", "host->sim": "H->S", "sim->host": "S->H"}[self.value]

    def cycle(self) -> "Direction":
        order = [Direction.HOST_TO_SIM, Direction.SIM_TO_HOST, Direction.OFF]
        return order[(order.index(self) + 1) % len(order)]


# ── topic routes ─────────────────────────────────────────────────────────────


@dataclass
class TopicRoute:
    key: str
    host_topic: str
    host_type: Any
    sim_topic: str
    sim_type: Any
    direction: Direction
    converter: Callable
    group: str = "misc"
    sub_qos: str = "compat"
    pub_qos: str = "px4"
    max_hz: float = 0.0  # 0 = unlimited
    enabled: bool = True
    notes: str = ""
    # Extra sim-side topics a fan-out converter emits: (topic, type) pairs.
    fanout: Tuple[Tuple[str, Any], ...] = ()

    @property
    def available(self) -> bool:
        return self.host_type is not None and self.sim_type is not None

    @property
    def source_topic(self) -> str:
        return self.host_topic if self.direction is Direction.HOST_TO_SIM else self.sim_topic

    @property
    def sink_topic(self) -> str:
        return self.sim_topic if self.direction is Direction.HOST_TO_SIM else self.host_topic

    @property
    def source_type(self):
        return self.host_type if self.direction is Direction.HOST_TO_SIM else self.sim_type

    @property
    def sink_type(self):
        return self.sim_type if self.direction is Direction.HOST_TO_SIM else self.host_type

    def apply_overrides(self, overrides: dict) -> None:
        if "enabled" in overrides:
            self.enabled = bool(overrides["enabled"])
        if "direction" in overrides:
            try:
                self.direction = Direction(overrides["direction"])
            except ValueError:
                pass
        if "sub_qos" in overrides:
            self.sub_qos = str(overrides["sub_qos"])
        if "pub_qos" in overrides:
            self.pub_qos = str(overrides["pub_qos"])
        if "max_hz" in overrides:
            try:
                self.max_hz = max(0.0, float(overrides["max_hz"]))
            except (TypeError, ValueError):
                pass

    def as_override(self) -> dict:
        return {
            "enabled": self.enabled,
            "direction": self.direction.value,
            "sub_qos": self.sub_qos,
            "pub_qos": self.pub_qos,
            "max_hz": self.max_hz,
        }


# ── service routes ───────────────────────────────────────────────────────────


class ServiceRole(Enum):
    OFF = "off"
    SERVE = "serve"  # we answer /wrapper/... and drive the sim
    PROXY = "proxy"  # we call the Manifold's /wrapper/... from a sim-side name

    def cycle(self) -> "ServiceRole":
        order = [ServiceRole.SERVE, ServiceRole.PROXY, ServiceRole.OFF]
        return order[(order.index(self) + 1) % len(order)]


@dataclass
class ServiceRoute:
    key: str
    service: str  # relative to the wrapper prefix, e.g. "takeoff"
    srv_type: Any
    handler: Callable  # (request, response, bridge) -> response
    role: ServiceRole = ServiceRole.SERVE
    group: str = "misc"
    enabled: bool = True
    notes: str = ""
    remote_server_seen: bool = False

    @property
    def available(self) -> bool:
        return self.srv_type is not None

    def apply_overrides(self, overrides: dict) -> None:
        if "enabled" in overrides:
            self.enabled = bool(overrides["enabled"])
        if "role" in overrides:
            try:
                self.role = ServiceRole(overrides["role"])
            except ValueError:
                pass

    def as_override(self) -> dict:
        return {"enabled": self.enabled, "role": self.role.value}


# ── type resolution ──────────────────────────────────────────────────────────


def _px4(name: str):
    """Resolve a px4_msgs message class, or None when this PX4 lacks it."""
    try:
        import px4_msgs.msg as px4msg
    except ImportError:
        return None
    return getattr(px4msg, name, None)


def _psdk_srv(name: str):
    """Resolve a psdk_interfaces service class, or None when absent."""
    try:
        import psdk_interfaces.srv as psdksrv
    except ImportError:
        return None
    return getattr(psdksrv, name, None)


def missing_types(routes, services) -> List[str]:
    """Human-readable list of rows disabled because a type is unavailable."""
    problems = []
    for route in routes:
        if route.host_type is None:
            problems.append(f"{route.key}: host type missing")
        elif route.sim_type is None:
            problems.append(f"{route.key}: sim type missing (PX4 message not in px4_msgs)")
    for service in services:
        if service.srv_type is None:
            problems.append(f"{service.key}: srv type missing")
    return problems


# ── the topic table ──────────────────────────────────────────────────────────


def build_topic_routes(settings) -> List[TopicRoute]:
    """Construct the route table, resolving topic names from settings."""
    from geometry_msgs.msg import (
        AccelStamped,
        QuaternionStamped,
        TwistStamped,
        Vector3Stamped,
    )
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import (
        BatteryState,
        Image,
        Imu,
        Joy,
        LaserScan,
        MagneticField,
        NavSatFix,
    )
    from std_msgs.msg import Bool, Float32, Float64, UInt8, UInt16

    from psdk_interfaces.msg import (
        ControlMode,
        DisplayMode,
        EscData,
        FlightAnomaly,
        FlightStatus,
        GimbalRotation,
        GimbalStatus,
        GPSDetails,
        PerceptionCameraParameters,
        PositionFused,
        RCConnectionStatus,
        RelativeObstacleInfo,
        RTKYaw,
        SingleBatteryInfo,
    )

    prefix = settings.wrapper_prefix.rstrip("/")
    sim = settings.sim_topics

    def host(name: str) -> str:
        return f"{prefix}/{name}"

    routes: List[TopicRoute] = [
        # ── control: Manifold drives the simulation ───────────────────────
        TopicRoute(
            key="rc_setpoint",
            host_topic=host("rc"),
            host_type=Joy,
            sim_topic=sim["trajectory_setpoint"],
            sim_type=_px4("TrajectorySetpoint"),
            direction=Direction.HOST_TO_SIM,
            converter=cv.joy_to_trajectory_setpoint,
            group="control",
            sub_qos="compat",
            pub_qos="px4",
            max_hz=50.0,
            notes="RC sticks -> body-frame velocity, rotated into NED by heading",
        ),
        TopicRoute(
            key="rc_passthrough",
            host_topic=host("rc"),
            host_type=Joy,
            sim_topic="/drone/rc",
            sim_type=Joy,
            direction=Direction.HOST_TO_SIM,
            converter=cv.joy_passthrough,
            group="control",
            sub_qos="compat",
            # RELIABLE, not px4/BEST_EFFORT: drone_controller.py subscribes to
            # /drone/rc with the rclpy default (RELIABLE), and a RELIABLE reader
            # never matches a BEST_EFFORT writer. Publishing best-effort here
            # would leave the GUI's stick display dead with no error anywhere.
            pub_qos="reliable",
            max_hz=30.0,
            notes="raw sticks for the GUI stick display",
        ),
        # The setpoint topics a PSDK application actually publishes. Until these
        # existed the only input that moved the simulation was the RC stick
        # channel, so every project had to pretend to be a radio.
        #
        # All three are OFF by default and only one may run at a time: they and
        # rc_setpoint write the same /fmu/in/trajectory_setpoint, and two writers
        # on it fight. node.py enforces that -- pointing one of these host->sim
        # switches the others off and says so.
        TopicRoute(
            key="psdk_position_setpoint",
            host_topic=host("flight_control_setpoint_ENUposition_yaw"),
            host_type=Joy,
            sim_topic=sim["trajectory_setpoint"],
            sim_type=_px4("TrajectorySetpoint"),
            direction=Direction.HOST_TO_SIM,
            converter=cv.psdk_position_yaw_to_setpoint,
            group="control",
            sub_qos="compat",
            pub_qos="px4",
            max_hz=50.0,
            enabled=False,
            notes="OFF: ENU offsets + absolute height/yaw -> NED position; "
                  "needs position_fused for the offset reference",
        ),
        TopicRoute(
            key="psdk_velocity_setpoint",
            host_topic=host("flight_control_setpoint_ENUvelocity_yawrate"),
            host_type=Joy,
            sim_topic=sim["trajectory_setpoint"],
            sim_type=_px4("TrajectorySetpoint"),
            direction=Direction.HOST_TO_SIM,
            converter=cv.psdk_enu_velocity_to_setpoint,
            group="control",
            sub_qos="compat",
            pub_qos="px4",
            max_hz=50.0,
            enabled=False,
            notes="OFF: ENU ground velocity -> NED velocity; needs no heading",
        ),
        TopicRoute(
            key="psdk_body_velocity_setpoint",
            host_topic=host("flight_control_setpoint_FLUvelocity_yawrate"),
            host_type=Joy,
            sim_topic=sim["trajectory_setpoint"],
            sim_type=_px4("TrajectorySetpoint"),
            direction=Direction.HOST_TO_SIM,
            converter=cv.psdk_flu_velocity_to_setpoint,
            group="control",
            sub_qos="compat",
            pub_qos="px4",
            max_hz=50.0,
            enabled=False,
            notes="OFF: body FLU velocity -> NED, rotated by the heading",
        ),
        TopicRoute(
            key="rc_authority",
            host_topic=host("control_mode"),
            host_type=ControlMode,
            sim_topic="/drone/rc/authority",
            sim_type=Bool,
            direction=Direction.HOST_TO_SIM,
            converter=cv.control_mode_to_authority,
            group="control",
            sub_qos="compat",
            pub_qos="latched",
            max_hz=10.0,
            notes="device_mode==0 -> RC holds authority; latched for late joiners",
        ),
        TopicRoute(
            key="gimbal_command",
            host_topic=host("gimbal_rotation"),
            host_type=GimbalRotation,
            sim_topic="/drone/gimbal/cmd/pan",
            sim_type=Float64,
            direction=Direction.HOST_TO_SIM,
            converter=cv.gimbal_rotation_to_joint_cmds,
            group="control",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=30.0,
            enabled=False,
            # OFF because gimbal_angles now owns the joint topics. A live wrapper
            # never publishes gimbal_rotation (it subscribes to it), so this route
            # is silent against real hardware -- but the mock DOES publish it, and
            # two enabled routes writing /drone/gimbal/cmd/* would fight each other
            # at 30 Hz. Enable it only if something sim-side issues gimbal commands,
            # and point gimbal_angles elsewhere first.
            notes="OFF: gimbal_angles drives the joints. DJI command topic, "
                  "fans out to pan/roll/tilt",
            fanout=(
                ("/drone/gimbal/cmd/roll", Float64),
                ("/drone/gimbal/cmd/tilt", Float64),
            ),
        ),
        # ── telemetry: simulation reports back to the Manifold ────────────
        TopicRoute(
            key="flight_status",
            host_topic=host("flight_status"),
            host_type=FlightStatus,
            sim_topic=sim["vehicle_status"],
            sim_type=_px4("VehicleStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.vehicle_status_to_flight_status,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=5.0,
        ),
        TopicRoute(
            key="display_mode",
            host_topic=host("display_mode"),
            host_type=DisplayMode,
            sim_topic=sim["vehicle_status"],
            sim_type=_px4("VehicleStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.vehicle_status_to_display_mode,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=5.0,
        ),
        TopicRoute(
            key="flight_anomaly",
            host_topic=host("flight_anomaly"),
            host_type=FlightAnomaly,
            sim_topic=sim["vehicle_status"],
            sim_type=_px4("VehicleStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.vehicle_status_to_flight_anomaly,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=2.0,
        ),
        TopicRoute(
            key="rc_connection",
            host_topic=host("rc_connection_status"),
            host_type=RCConnectionStatus,
            sim_topic=sim["vehicle_status"],
            sim_type=_px4("VehicleStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.vehicle_status_to_rc_connection,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=2.0,
            notes="reports whether stick data is actually arriving",
        ),
        TopicRoute(
            key="battery",
            # Was "aircraft_battery", a name no M4E publishes. The aircraft
            # has single_battery_index1 and index2; index1 already has a route
            # below, so this one becomes index2. The simulation models one
            # pack, so both indices report the same cells -- a real aircraft
            # would not.
            host_topic=host("single_battery_index2"),
            host_type=SingleBatteryInfo,
            sim_topic=sim["battery"],
            sim_type=_px4("BatteryStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.battery_status_to_single_battery,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=2.0,
        ),
        TopicRoute(
            key="gps_position",
            host_topic=host("gps_position"),
            host_type=NavSatFix,
            sim_topic=sim["sensor_gps"],
            sim_type=_px4("SensorGps"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.sensor_gps_to_navsatfix,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            # The M4E caps gps_data at 5 Hz (psdk_params.yaml); asking for more
            # on real hardware fails the subscription with NONSUPPORT, so the
            # simulation offers exactly what the aircraft can.
            max_hz=5.0,
            notes="RAW receiver fix from SensorGps, not the EKF solution -- "
                  "see gps_position_fused for that",
        ),
        TopicRoute(
            key="rtk_position",
            host_topic=host("rtk_position"),
            host_type=NavSatFix,
            sim_topic=sim["global_position"],
            sim_type=_px4("VehicleGlobalPosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.global_position_to_navsatfix,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=5.0,
            enabled=False,
            # Deliberately NOT the same source as gps_position any more. The
            # M4E's RTK is a second, centimetre-accurate receiver; the
            # simulation has exactly one GNSS, so the closest honest stand-in
            # for "the accurate one" is PX4's fused solution, which is smooth
            # and tracks truth, while gps_position carries the raw scatter.
            # Neither is a real RTK model -- there is no fix-type transition,
            # no baseline, no correction dropout.
            notes="fused solution standing in for RTK; gps_position is raw. "
                  "Off by default to halve traffic",
        ),
        TopicRoute(
            key="gps_details",
            host_topic=host("gps_details"),
            host_type=GPSDetails,
            sim_topic=sim["sensor_gps"],
            sim_type=_px4("SensorGps"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.sensor_gps_to_gps_details,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=2.0,
            notes="DOP/accuracy/sat counts only - GPSDetails carries no position",
        ),
        TopicRoute(
            key="position_fused",
            host_topic=host("position_fused"),
            host_type=PositionFused,
            sim_topic=sim["local_position"],
            sim_type=_px4("VehicleLocalPosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.local_position_to_position_fused,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=20.0,
        ),
        # The route that makes the simulated gimbal follow the real one. It is
        # host->sim and in the control group because gimbal_angles is what the
        # wrapper actually publishes -- gimbal_rotation above is a command the
        # wrapper consumes, so it has no publisher on a live Manifold and cannot
        # drive anything. See gimbal_angles_to_joint_cmds for the ground-frame
        # to airframe yaw correction, which is the part that decides whether the
        # gimbal moves at all or sits on its pan limit.
        TopicRoute(
            key="gimbal_angles",
            host_topic=host("gimbal_angles"),
            host_type=Vector3Stamped,
            sim_topic="/drone/gimbal/cmd/pan",
            sim_type=Float64,
            direction=Direction.HOST_TO_SIM,
            converter=cv.gimbal_angles_to_joint_cmds,
            group="control",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=30.0,
            notes="real gimbal attitude -> pan/roll/tilt joints, yaw made body-relative",
            fanout=(
                ("/drone/gimbal/cmd/roll", Float64),
                ("/drone/gimbal/cmd/tilt", Float64),
            ),
        ),
        # The inverse of the route above: report where the SIMULATED gimbal is
        # pointing back out as gimbal_angles. The source is the pan joint topic
        # that drone_controller.py drives; roll and tilt are read from shared
        # state, which node.py fills from the other two joint topics. It used to
        # read /fmu/out/vehicle_attitude and publish the airframe's own attitude,
        # which meant the simulated pan never reached the wrapper and no heading
        # shift was applied in this direction -- see
        # gimbal_joints_to_gimbal_angles for the full account.
        TopicRoute(
            key="gimbal_feedback",
            host_topic=host("gimbal_angles"),
            host_type=Vector3Stamped,
            sim_topic="/drone/gimbal/cmd/pan",
            sim_type=Float64,
            direction=Direction.SIM_TO_HOST,
            converter=cv.gimbal_joints_to_gimbal_angles,
            group="control",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=20.0,
            enabled=False,
            notes="OFF: sim gimbal joints -> gimbal_angles, yaw shifted back to "
                  "the ground frame. Enabling this while the gimbal_angles route "
                  "is on closes a sim->host->sim loop",
        ),
        TopicRoute(
            key="gimbal_status",
            host_topic=host("gimbal_status"),
            host_type=GimbalStatus,
            sim_topic=sim["attitude"],
            sim_type=_px4("VehicleAttitude"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.attitude_to_gimbal_status,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=1.0,
            notes="health flags only; angles go out on gimbal_angles",
        ),
        # THERE IS NO "home_position" TOPIC ON A MATRICE 4E. This route used to
        # publish psdk_interfaces/HomePosition on that name; a live aircraft's
        # topic list has home_point (NavSatFix), home_point_altitude (Float32)
        # and home_point_status (Bool) instead, and all three already have
        # routes further down carrying exactly the same information from the
        # same PX4 source. So this row was a fourth copy under a name nothing
        # would ever subscribe to, and it has been removed rather than renamed.
        #
        # psdk_interfaces/msg/HomePosition.msg stays in the package: it is part
        # of the vendored interface definition, not ours to delete.
        TopicRoute(
            key="obstacle_info",
            host_topic=host("relative_obstacle_info"),
            host_type=RelativeObstacleInfo,
            sim_topic=sim["lrf_range"],
            sim_type=LaserScan,
            direction=Direction.SIM_TO_HOST,
            converter=cv.laserscan_to_obstacle_info,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=10.0,
            notes="downward LRF only; other sectors report health=0",
        ),
        TopicRoute(
            key="land_detected",
            host_topic="/drone/landed",
            host_type=Bool,
            sim_topic=sim["land_detected"],
            sim_type=_px4("VehicleLandDetected"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.land_detected_to_shared,
            group="internal",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=5.0,
            notes="feeds ON_GROUND/ON_AIR in flight_status",
        ),
    ]

    routes += _video_routes(host, sim, Image)
    routes += _extended_telemetry_routes(
        host,
        sim,
        types=dict(
            QuaternionStamped=QuaternionStamped,
            TwistStamped=TwistStamped,
            Vector3Stamped=Vector3Stamped,
            NavSatFix=NavSatFix,
            BatteryState=BatteryState,
            SingleBatteryInfo=SingleBatteryInfo,
            Imu=Imu,
            Odometry=Odometry,
            Float32=Float32,
            Bool=Bool,
            UInt8=UInt8,
            UInt16=UInt16,
        ),
    )
    routes += _mirror_routes(
        host,
        (
            # (wrapper topic, type) -- everything the Manifold publishes that has
            # no simulated counterpart to convert from. See _mirror_routes.
            ("acceleration_body_fused", AccelStamped),
            ("acceleration_body_raw", AccelStamped),
            ("acceleration_ground_fused", AccelStamped),
            ("angular_rate_ground_fused", Vector3Stamped),
            ("esc_data", EscData),
            ("gps_control_level", UInt8),
            ("gps_signal_level", UInt8),
            ("landing_gear_status", UInt8),
            ("magnetic_field", MagneticField),
            ("motor_start_error", UInt16),
            ("perception_camera_parameters", PerceptionCameraParameters),
            ("rtk_connection_status", UInt16),
            ("rtk_position_info", UInt8),
            ("rtk_velocity", TwistStamped),
            ("rtk_yaw", RTKYaw),
            ("rtk_yaw_info", UInt8),
            ("single_battery_index2", SingleBatteryInfo),
            # The two setpoint inputs that are NOT converted. The position and
            # the two velocity forms now have real routes above
            # (psdk_position_setpoint, psdk_velocity_setpoint,
            # psdk_body_velocity_setpoint) and are deliberately not mirrored as
            # well -- one source topic, one meaning. These two remain mirrors
            # because PX4 takes attitude and thrust on a different message
            # entirely, so there is nothing honest to convert them into: the
            # mirror at least makes what the client asked for visible.
            ("flight_control_setpoint_generic", Joy),
            ("flight_control_setpoint_rollpitch_yawrate_thrust", Joy),
        ),
    )

    routes = _apply_v2_surface_policy(routes)

    for route in routes:
        route.apply_overrides(settings.overrides_for(route.key))
    return routes


# ── the Version 2 surface policy ─────────────────────────────────────────────

# Publish rates a real Matrice 4E produces, from psdk_ros2_topics_services.pdf.
#
# These are used as CAPS, not targets. The point is fidelity in both
# directions: a simulation that publishes imu at 88 Hz where the aircraft does
# 50 is just as wrong as one that publishes at 17, and a consumer tuned against
# the simulation would then misbehave on hardware. PX4's source rates do not
# match DJI's, so the wrapper surface has to be shaped to DJI's.
#
# A topic absent from this table runs uncapped.
PDF_RATE_HZ = {
    "attitude": 50.0, "imu": 50.0, "position_fused": 50.0,
    "velocity_ground_fused": 50.0, "angular_rate_body_raw": 50.0,
    "angular_rate_ground_fused": 50.0, "visual_odometry": 50.0,
    "acceleration_body_fused": 50.0, "acceleration_body_raw": 50.0,
    "acceleration_ground_fused": 50.0, "magnetic_field": 50.0,
    "altitude_barometric": 70.0, "altitude_sea_level": 70.0,
    "gps_position_fused": 30.0,
    "flight_status": 25.0, "display_mode": 25.0, "control_mode": 24.0,
    "height_above_ground": 25.0, "gps_position": 25.0, "gps_details": 25.0,
    "gps_signal_level": 25.0, "rc": 25.0, "rc_connection_status": 25.0,
    "relative_obstacle_info": 25.0, "home_point": 25.0,
    "home_point_status": 24.0, "flight_anomaly": 25.0, "gps_velocity": 25.0,
    "esc_data": 21.0, "single_battery_index1": 21.0,
    "single_battery_index2": 21.0, "motor_start_error": 20.0,
    "rtk_connection_status": 5.0, "rtk_position": 5.0,
    "rtk_position_info": 5.0, "rtk_velocity": 5.0, "rtk_yaw": 5.0,
    "rtk_yaw_info": 5.0,
    "main_camera_stream": 11.0,
    "perception_stereo_left_stream": 18.0,
    "perception_stereo_right_stream": 19.0,
}


def pdf_rate_for(host_topic: str) -> float:
    """The documented aircraft rate for a wrapper topic, or 0 if uncapped."""
    return PDF_RATE_HZ.get(host_topic.rsplit("/", 1)[-1], 0.0)


def _apply_v2_surface_policy(routes: List[TopicRoute]) -> List[TopicRoute]:
    """
    Decide which routes are live by default, for a simulation with no Manifold.

    WHY THE V1 DEFAULTS ARE WRONG HERE
    ----------------------------------
    In Version 1 most telemetry routes shipped DISABLED, and that was correct:
    a real Manifold was publishing those same wrapper topics, so a bridge that
    also published them would have put two writers on one topic. Enabling a
    route was therefore a deliberate act, done from the console, once you knew
    the Manifold was not already covering it.

    Version 2 has no Manifold. The bridge is the ONLY thing that can publish
    the wrapper surface, so a route left off is simply a topic a C++ mission
    will wait for forever. The default inverts: everything on, except where
    there is a specific reason.

    THE ONE EXCEPTION: VIDEO
    ------------------------
    Video routes stay opt-in, as in V1 and as §12.4 of the plan requires. An
    uncompressed 1080p frame is ~6 MB; two feeds saturate the transport and
    starve the control and telemetry routes sharing it. A project that wants a
    picture asks for one route, at a rate cap.

    MIRROR ROUTES ARE DROPPED ENTIRELY
    ----------------------------------
    A mirror route copies a topic the MANIFOLD publishes into /manifold/<name>
    so the simulation side can read it. With no Manifold there is nothing to
    copy: the route would subscribe to a wrapper topic nobody publishes and
    republish silence. Worse, it makes the wrapper topic *appear* in the graph
    with a subscriber and no publisher, which reads as "the bridge is handling
    this" when it is doing the opposite.

    So they are removed here rather than disabled, and the topics they covered
    are tracked honestly as a gap -- see SYNTHESIS_GAP below.
    """
    kept: List[TopicRoute] = []
    for route in routes:
        if route.group == "mirror":
            continue
        if route.group != "video":
            route.enabled = True
            # THE RATE CAPS MEASURED A CONSTRAINT THAT NO LONGER EXISTS.
            #
            # V1's caps (1-50 Hz depending on the route) were sized for the
            # wireless link to a Manifold 3, where config/dds.xml capped
            # datagrams at 1400 bytes and a saturated link starved the control
            # routes. V2's entire graph is shared memory inside one container:
            # there is no radio, no fragmentation, and nothing to starve.
            #
            # Leaving the caps in place made the simulation LESS faithful than
            # the aircraft it imitates -- attitude arrived at 17 Hz where the
            # PDF documents 50 -- which is a strange way to fail. So non-video
            # routes run at whatever rate their PX4 source produces.
            #
            # Video keeps its own cap: 6 MB a frame is a real constraint even
            # over shared memory, and that cap is chosen by whoever turns the
            # feed on.
            #
            # Everything else is capped at the rate the REAL AIRCRAFT produces,
            # which is the honest target -- see PDF_RATE_HZ. Uncapping entirely
            # was the first attempt and it overshot: imu came out at 88 Hz
            # against the documented 50, because PX4's sensor_combined is
            # faster than DJI's surface. Too fast is as wrong as too slow: a
            # consumer tuned against the simulation would then misbehave on
            # hardware.
            route.max_hz = pdf_rate_for(route.host_topic)
        kept.append(route)
    return kept


# Wrapper topics a real Matrice 4E publishes that this simulation does NOT yet
# produce, because Version 1 never synthesised them either -- they arrived from
# the aircraft's own hardware and the bridge merely mirrored them.
#
# Listed here rather than left implicit so that `tools/check_surface.py` can
# report the gap precisely, and so nobody re-discovers it by watching a mission
# wait on a topic that was never going to arrive.
#
#   name                          what it would have to come from
SYNTHESIS_GAP = {
    "acceleration_body_fused": "sensor_combined accel, low-pass filtered",
    "acceleration_body_raw": "sensor_combined accelerometer_m_s2",
    "acceleration_ground_fused": "vehicle_local_position ax/ay/az",
    "angular_rate_ground_fused": "sensor_combined gyro, rotated to ENU",
    "esc_data": "PX4 esc_status, which is not in dds_topics.yaml by default",
    "gps_control_level": "vehicle_gps_position fix type",
    "gps_signal_level": "vehicle_gps_position satellites_used",
    "landing_gear_status": "nothing: the M4E has no retractable gear",
    "magnetic_field": "the Gazebo magnetometer, which is not bridged to ROS",
    "motor_start_error": "PX4 arming rejection reason",
    "perception_camera_parameters": "the fisheye pair's camera_info",
    "rtk_connection_status": "nothing: no RTK is simulated",
    "rtk_position_info": "nothing: no RTK is simulated",
    "rtk_velocity": "nothing: no RTK is simulated",
    "rtk_yaw": "nothing: no RTK is simulated",
    "rtk_yaw_info": "nothing: no RTK is simulated",
    "flight_control_setpoint_generic": "accepted but not convertible to a PX4 setpoint",
    "flight_control_setpoint_rollpitch_yawrate_thrust":
        "PX4 takes attitude+thrust on a different message entirely",
}


# ── the rest of the wrapper's surface ────────────────────────────────────────
#
# The tables above are the routes that existed when the bridge only had to fly
# the aircraft. A live Manifold publishes far more than that -- 62 topics on the
# m4e -- and none of the rest could be pointed at the simulation at all until it
# had a row here. The three builders below add the remainder, split by how much
# translation each one actually needs.


def _video_routes(host, sim, Image) -> List[TopicRoute]:
    """
    The camera streams, in both directions.

    Each feed gets two rows rather than one switchable row, because the two
    directions are not the same job and cannot share a sim-side topic:

      *_in   wrapper -> `/manifold/<name>`. Real payload video inside the
             simulation graph: watchable in RViz, recordable, or feedable to a
             perception node under test. A separate sim-side name so it never
             collides with camera_switcher.py's own publishers.
      *_out  simulated camera -> the wrapper's own topic name, so a PSDK or MSDK
             client sees a picture from the simulation.

    EVERY ONE OF THESE DEFAULTS TO OFF, and that is not timidity. An
    uncompressed 1080p frame is ~6 MB, which at the 1400-byte datagram cap in
    config/dds.xml is ~4500 fragments; losing any one of them costs the whole
    frame. Two feeds at once will saturate the link to the Manifold and starve
    the control and telemetry routes sharing it. Turn on one, at a low max_hz,
    and prefer a preview tier.

    The *_out rows additionally collide with a live wrapper, which already
    publishes these names -- two writers, and a subscriber gets interleaved
    frames from both.
    """
    return [
        TopicRoute(
            key="main_camera_in",
            host_topic=host("main_camera_stream"),
            host_type=Image,
            sim_topic="/manifold/main_camera_stream",
            sim_type=Image,
            direction=Direction.HOST_TO_SIM,
            converter=cv.image_passthrough,
            group="video",
            sub_qos="compat",
            pub_qos="video",
            max_hz=5.0,
            enabled=False,
            notes="OFF: real main camera into the sim graph as "
                  "/manifold/main_camera_stream",
        ),
        TopicRoute(
            key="main_camera_out",
            host_topic=host("main_camera_stream"),
            host_type=Image,
            sim_topic=sim["camera_active"],
            sim_type=Image,
            direction=Direction.SIM_TO_HOST,
            converter=cv.image_passthrough,
            group="video",
            sub_qos="compat",
            pub_qos="video",
            max_hz=5.0,
            enabled=False,
            notes="OFF: simulated active lens out as main_camera_stream. "
                  "Collides with a live wrapper, which publishes this name too",
        ),
        TopicRoute(
            key="fpv_camera_in",
            host_topic=host("fpv_camera_stream"),
            host_type=Image,
            sim_topic="/manifold/fpv_camera_stream",
            sim_type=Image,
            direction=Direction.HOST_TO_SIM,
            converter=cv.image_passthrough,
            group="video",
            sub_qos="compat",
            pub_qos="video",
            max_hz=5.0,
            enabled=False,
            notes="OFF: real FPV camera into the sim graph",
        ),
        TopicRoute(
            key="fpv_camera_out",
            host_topic=host("fpv_camera_stream"),
            host_type=Image,
            sim_topic=sim["camera_preview"],
            sim_type=Image,
            direction=Direction.SIM_TO_HOST,
            converter=cv.image_passthrough,
            group="video",
            sub_qos="compat",
            pub_qos="video",
            max_hz=5.0,
            enabled=False,
            notes="OFF: simulated preview tier out as fpv_camera_stream. "
                  "Collides with a live wrapper",
        ),
        TopicRoute(
            key="stereo_left_in",
            host_topic=host("perception_stereo_left_stream"),
            host_type=Image,
            sim_topic="/manifold/perception_stereo_left_stream",
            sim_type=Image,
            direction=Direction.HOST_TO_SIM,
            converter=cv.image_passthrough,
            group="video",
            sub_qos="compat",
            pub_qos="video",
            max_hz=5.0,
            enabled=False,
            notes="OFF: the m4e has no simulated stereo pair, so inbound only",
        ),
        TopicRoute(
            key="stereo_right_in",
            host_topic=host("perception_stereo_right_stream"),
            host_type=Image,
            sim_topic="/manifold/perception_stereo_right_stream",
            sim_type=Image,
            direction=Direction.HOST_TO_SIM,
            converter=cv.image_passthrough,
            group="video",
            sub_qos="compat",
            pub_qos="video",
            max_hz=5.0,
            enabled=False,
            notes="OFF: real right stereo frames into the sim graph",
        ),
        # The simulated pair, going the other way. These are what make a PSDK
        # client see stereo from the simulation, and they are the input the
        # localization project reads.
        #
        # NOT OPERATOR-TOGGLED IN THE NORMAL CASE: start_perception owns them,
        # because that is how the aircraft works -- a payload calls the service
        # and the stream appears. Flipping them by hand from the FLOW screen
        # works and is occasionally useful, but it bypasses the direction
        # bookkeeping in shared["perception_direction"].
        TopicRoute(
            key="stereo_left_out",
            host_topic=host("perception_stereo_left_stream"),
            host_type=Image,
            sim_topic="/drone/perception/front/left/image_raw",
            sim_type=Image,
            direction=Direction.SIM_TO_HOST,
            converter=cv.image_passthrough,
            group="video",
            sub_qos="compat",
            pub_qos="video",
            # The aircraft's own rate. No cap below it: a stereo pair with one
            # frame dropped out of each second is worse than useless to a
            # matcher, and mono8 704x704 is an eighth of a 1080p RGB frame.
            max_hz=20.0,
            enabled=False,
            notes="OFF until start_perception FRONT; mono8 704x704 @20 Hz",
        ),
        TopicRoute(
            key="stereo_right_out",
            host_topic=host("perception_stereo_right_stream"),
            host_type=Image,
            sim_topic="/drone/perception/front/right/image_raw",
            sim_type=Image,
            direction=Direction.SIM_TO_HOST,
            converter=cv.image_passthrough,
            group="video",
            sub_qos="compat",
            pub_qos="video",
            max_hz=20.0,
            enabled=False,
            notes="OFF until start_perception FRONT; mono8 704x704 @20 Hz",
        ),
    ]


def _extended_telemetry_routes(host, sim, types) -> List[TopicRoute]:
    """
    Wrapper telemetry the simulation can genuinely produce, sim->host.

    These are real conversions, not mirrors: each one reads a PX4 topic and
    builds the psdk_ros2 message a PSDK client expects, including the NED->ENU
    and FRD->FLU frame changes (see the frame helpers in converters.py). The
    cheap, low-rate ones are on by default; the high-rate ones are off so that
    turning the bridge on does not by itself put tens of megabits onto a
    wireless link.
    """
    return [
        TopicRoute(
            key="attitude",
            host_topic=host("attitude"),
            host_type=types["QuaternionStamped"],
            sim_topic=sim["attitude"],
            sim_type=_px4("VehicleAttitude"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.attitude_to_quaternion_stamped,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=20.0,
            enabled=False,
            # node.py subscribes to this same topic to learn the REAL aircraft's
            # heading, which the gimbal conversion needs. Publishing simulated
            # attitude onto it would feed our own sim heading back in as though
            # it came from the Manifold, and the gimbal's yaw reference would
            # quietly become the simulator's own yaw.
            notes="OFF: would collide with the wrapper's own attitude, which "
                  "node.py reads as the gimbal yaw reference",
        ),
        TopicRoute(
            key="altitude_sea_level",
            host_topic=host("altitude_sea_level"),
            host_type=types["Float32"],
            sim_topic=sim["global_position"],
            sim_type=_px4("VehicleGlobalPosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.global_position_to_altitude_msl,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=5.0,
        ),
        TopicRoute(
            key="altitude_barometric",
            host_topic=host("altitude_barometric"),
            host_type=types["Float32"],
            sim_topic=sim["global_position"],
            sim_type=_px4("VehicleGlobalPosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.global_position_to_altitude_barometric,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=5.0,
            enabled=False,
            notes="OFF: the EKF's fused altitude standing in for a barometer",
        ),
        TopicRoute(
            key="height_above_ground",
            host_topic=host("height_above_ground"),
            host_type=types["Float32"],
            sim_topic=sim["local_position"],
            sim_type=_px4("VehicleLocalPosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.local_position_to_height_above_ground,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=10.0,
            notes="downward rangefinder, falling back to -z above the EKF origin",
        ),
        TopicRoute(
            key="gps_position_fused",
            host_topic=host("gps_position_fused"),
            host_type=types["NavSatFix"],
            sim_topic=sim["global_position"],
            sim_type=_px4("VehicleGlobalPosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.global_position_to_navsatfix,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=10.0,
        ),
        TopicRoute(
            key="velocity_ground_fused",
            host_topic=host("velocity_ground_fused"),
            host_type=types["Vector3Stamped"],
            sim_topic=sim["local_position"],
            sim_type=_px4("VehicleLocalPosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.local_position_to_velocity_ground,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=10.0,
            notes="NED -> ENU",
        ),
        TopicRoute(
            key="gps_velocity",
            host_topic=host("gps_velocity"),
            host_type=types["TwistStamped"],
            sim_topic=sim["local_position"],
            sim_type=_px4("VehicleLocalPosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.local_position_to_gps_velocity,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=10.0,
            notes="NED -> ENU, linear only",
        ),
        TopicRoute(
            key="angular_rate_body_raw",
            host_topic=host("angular_rate_body_raw"),
            host_type=types["Vector3Stamped"],
            sim_topic=sim["sensor_combined"],
            sim_type=_px4("SensorCombined"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.angular_velocity_to_vector3,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=20.0,
            notes="gyro from SensorCombined, FRD -> FLU",
        ),
        TopicRoute(
            key="imu",
            host_topic=host("imu"),
            host_type=types["Imu"],
            sim_topic=sim["sensor_combined"],
            sim_type=_px4("SensorCombined"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.sensor_combined_to_imu,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            # 50 Hz is the M4E's ceiling for this topic: psdk_params.yaml notes
            # that asking for 100 Hz fails the subscription with NONSUPPORT.
            # SensorCombined itself runs at ~250 Hz (models/m4e/model.sdf), so
            # the cap is what makes the simulated rate match the aircraft's
            # rather than flattering it.
            max_hz=50.0,
            notes="50 Hz (the M4E's cap). FRD -> FLU, orientation spliced from "
                  "VehicleAttitude, covariances zero as on hardware",
        ),
        TopicRoute(
            key="battery_state",
            host_topic=host("battery"),
            host_type=types["BatteryState"],
            sim_topic=sim["battery"],
            sim_type=_px4("BatteryStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.battery_status_to_battery_state,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=1.0,
            notes="sensor_msgs form; the psdk_interfaces form goes out on "
                  "single_battery_index2",
        ),
        # ── the rest of the aircraft's telemetry surface ─────────────────
        #
        # Eight topics a live M4E publishes that this bridge did not. Verified
        # against a real aircraft's `ros2 topic list`. Several are constants or
        # judgements rather than measurements -- see each converter, which says
        # plainly which is which. All are OFF by default: they exist so that a
        # payload subscribing to them finds a publisher, not so that every
        # session pays for eight more conversions.
        #
        # EACH OF THESE HAS A MIRROR TWIN further down (mirror_<name>), and that
        # is deliberate, not duplication: the mirror carries a REAL Manifold's
        # value INTO the simulation, while the row here makes the SIMULATION
        # produce the topic for a payload to read. Opposite directions, same
        # name, and both off by default -- turning on both at once would have
        # the bridge publishing and subscribing the same topic, so do not.
        TopicRoute(
            key="angular_rate_ground_fused",
            host_topic=host("angular_rate_ground_fused"),
            host_type=types["Vector3Stamped"],
            sim_topic=sim["sensor_combined"],
            sim_type=_px4("SensorCombined"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.local_position_to_angular_rate_ground,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=20.0,
            enabled=False,
            notes="OFF: body rates rotated into the world by the attitude",
        ),
        TopicRoute(
            key="gps_signal_level",
            host_topic=host("gps_signal_level"),
            host_type=types["UInt8"],
            sim_topic=sim["sensor_gps"],
            sim_type=_px4("SensorGps"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.sensor_gps_to_signal_level,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=2.0,
            enabled=False,
            notes="OFF: DJI 0-5 quality, mapped from fix type and satellite count",
        ),
        TopicRoute(
            key="gps_control_level",
            host_topic=host("gps_control_level"),
            host_type=types["UInt8"],
            sim_topic=sim["sensor_gps"],
            sim_type=_px4("SensorGps"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.sensor_gps_to_control_level,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=2.0,
            enabled=False,
            notes="OFF: copies the signal level; no trust model in simulation",
        ),
        TopicRoute(
            key="landing_gear_status",
            host_topic=host("landing_gear_status"),
            host_type=types["UInt8"],
            sim_topic=sim["vehicle_status"],
            sim_type=_px4("VehicleStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.vehicle_status_to_landing_gear,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=1.0,
            enabled=False,
            notes="OFF: always 'deployed' -- the m4e's gear does not retract",
        ),
        TopicRoute(
            key="motor_start_error",
            host_topic=host("motor_start_error"),
            host_type=types["UInt16"],
            sim_topic=sim["vehicle_status"],
            sim_type=_px4("VehicleStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.vehicle_status_to_motor_start_error,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=1.0,
            enabled=False,
            notes="OFF: 0 or a generic 1; PX4's real reasons go to the journal",
        ),
        TopicRoute(
            key="rtk_connection_status",
            host_topic=host("rtk_connection_status"),
            host_type=types["UInt16"],
            sim_topic=sim["sensor_gps"],
            sim_type=_px4("SensorGps"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.sensor_gps_to_rtk_connection,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=1.0,
            enabled=False,
            notes="OFF: false in simulation -- there is no base station",
        ),
        TopicRoute(
            key="rtk_position_info",
            host_topic=host("rtk_position_info"),
            host_type=types["UInt8"],
            sim_topic=sim["sensor_gps"],
            sim_type=_px4("SensorGps"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.sensor_gps_to_rtk_position_info,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=2.0,
            enabled=False,
            notes="OFF: 0 none, 1 single, 2 float, 3 fixed; reaches 1 here",
        ),
        TopicRoute(
            key="rtk_yaw_info",
            host_topic=host("rtk_yaw_info"),
            host_type=types["UInt8"],
            sim_topic=sim["sensor_gps"],
            sim_type=_px4("SensorGps"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.sensor_gps_to_rtk_yaw_info,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=1.0,
            enabled=False,
            notes="OFF: always 0 -- one antenna, so no dual-antenna heading",
        ),
        TopicRoute(
            key="single_battery_index1",
            host_topic=host("single_battery_index1"),
            host_type=types["SingleBatteryInfo"],
            sim_topic=sim["battery"],
            sim_type=_px4("BatteryStatus"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.battery_status_to_single_battery,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=1.0,
            enabled=False,
            notes="OFF: the m4e is simulated with one pack, already reported on "
                  "single_battery_index2",
        ),
        TopicRoute(
            key="home_point",
            host_topic=host("home_point"),
            host_type=types["NavSatFix"],
            sim_topic=sim["home_position"],
            sim_type=_px4("HomePosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.px4_home_to_navsatfix,
            group="telemetry",
            sub_qos="compat",
            pub_qos="latched",
            max_hz=1.0,
        ),
        TopicRoute(
            key="home_point_altitude",
            host_topic=host("home_point_altitude"),
            host_type=types["Float32"],
            sim_topic=sim["home_position"],
            sim_type=_px4("HomePosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.px4_home_to_altitude,
            group="telemetry",
            sub_qos="compat",
            pub_qos="latched",
            max_hz=1.0,
        ),
        TopicRoute(
            key="home_point_status",
            host_topic=host("home_point_status"),
            host_type=types["Bool"],
            sim_topic=sim["home_position"],
            sim_type=_px4("HomePosition"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.px4_home_to_status,
            group="telemetry",
            sub_qos="compat",
            pub_qos="latched",
            max_hz=1.0,
        ),
        TopicRoute(
            key="visual_odometry",
            host_topic=host("visual_odometry"),
            host_type=types["Odometry"],
            sim_topic=sim["odometry"],
            sim_type=_px4("VehicleOdometry"),
            direction=Direction.SIM_TO_HOST,
            converter=cv.odometry_to_nav_odometry,
            group="telemetry",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=10.0,
            enabled=False,
            notes="OFF: high rate. NED -> ENU pose and velocity, no covariance",
        ),
    ]


def _mirror_routes(host, table) -> List[TopicRoute]:
    """
    One passthrough row per remaining wrapper topic, host->sim.

    These are the topics the simulation has nothing to synthesise from -- ESC
    telemetry, RTK, the perception camera's intrinsics, DJI's own setpoint
    inputs. There is nothing to convert, so the row exists to make the data
    *reachable* from the simulation side: each one republishes onto
    `/manifold/<name>`, where RViz, a rosbag or a node under test can pick it up
    without having to know the wrapper's prefix or share its QoS.

    `/manifold/` rather than the wrapper's own name so a mirror can never be
    confused with, or collide with, what the Manifold itself publishes.

    All off by default: 24 idle subscriptions cost nothing, 24 live ones are a
    surprise. Flip the ones you want from the FLOW screen or the RViz panel.
    """
    return [
        TopicRoute(
            key=f"mirror_{name}",
            host_topic=host(name),
            host_type=msg_type,
            sim_topic=f"/manifold/{name}",
            sim_type=msg_type,
            direction=Direction.HOST_TO_SIM,
            converter=cv.passthrough,
            group="mirror",
            sub_qos="compat",
            pub_qos="reliable",
            max_hz=10.0,
            enabled=False,
            notes=f"OFF: verbatim copy to /manifold/{name}",
        )
        for name, msg_type in table
    ]


# ── service handlers ─────────────────────────────────────────────────────────
#
# Signature: (request, response, bridge) -> response
# `bridge` is the SimtyBridge node, which exposes send_vehicle_command(),
# publish_sim() and .shared.

def _ok(response, value=True):
    if hasattr(response, "success"):
        response.success = value
    return response


def _fail(response, reason):
    """
    Refuse, and say why in the response where the caller will actually see it.

    Used where the simulation genuinely cannot stand in for the aircraft --
    formatting a card that does not exist, listing files never written. An
    empty success is worse than a refusal in those cases: it reads as "the card
    is empty" rather than "there is no card", and sends a payload looking for
    something that will never appear.
    """
    if hasattr(response, "success"):
        response.success = False
    if hasattr(response, "message"):
        response.message = reason
    return response


def _h_takeoff(request, response, bridge):
    # bridge.takeoff() owns the AMSL conversion and the arming pre-check; calling
    # send_vehicle_command directly here is how this handler used to send a
    # takeoff altitude PX4 read as metres above sea level.
    # 1.8 m unless something has already asked for a different height this
    # session. See TAKEOFF_ALTITUDE_M in node.py for why that number and where
    # else it has to match.
    from .node import TAKEOFF_ALTITUDE_M
    bridge.takeoff(float(bridge.shared.get("takeoff_alt", TAKEOFF_ALTITUDE_M)))
    return _ok(response)


def _h_land(request, response, bridge):
    bridge.send_vehicle_command("NAV_LAND")
    return _ok(response)


def _h_return_home(request, response, bridge):
    bridge.send_vehicle_command("NAV_RETURN_TO_LAUNCH")
    return _ok(response)


def _h_obtain_authority(request, response, bridge):
    bridge.set_rc_authority(True, source="obtain_ctrl_authority")
    return _ok(response)


def _h_release_authority(request, response, bridge):
    bridge.set_rc_authority(False, source="release_ctrl_authority")
    return _ok(response)


def _h_motors_on(request, response, bridge):
    # bridge.arm() rather than a bare COMPONENT_ARM_DISARM: it logs why PX4 is
    # about to refuse, notices two seconds later that the vehicle did not arm,
    # and -- the part that matters here -- leaves a flight mode that refuses to
    # arm before asking. PX4 stays in AUTO_LAND after a landing, and this handler
    # sending the command directly is why the second takeoff of a session used
    # to be refused with nothing said on the DJI surface.
    bridge.arm()
    return _ok(response)


def _h_motors_off(request, response, bridge):
    bridge.disarm()
    return _ok(response)


def _h_gimbal_set_mode(request, response, bridge):
    bridge.shared["gimbal_mode"] = int(getattr(request, "gimbal_mode", 0))
    return _ok(response)


def _h_gimbal_reset(request, response, bridge):
    from std_msgs.msg import Float64

    for axis in ("pan", "roll", "tilt"):
        bridge.publish_sim(f"/drone/gimbal/cmd/{axis}", Float64(data=0.0), Float64)
    return _ok(response)


def _h_camera_set_zoom(request, response, bridge):
    from std_msgs.msg import Float64, String

    factor = float(getattr(request, "zoom_factor", 1.0))
    bridge.shared["zoom"] = factor
    bridge.publish_sim("/drone/camera/zoom", Float64(data=factor), Float64)
    # Mirror drone_controller.py's lens breakpoints so the active feed follows.
    lens = "wide" if factor < 3.0 else ("medium_tele" if factor <= 7.0 else "tele")
    bridge.publish_sim("/drone/camera/select", String(data=lens), String)
    return _ok(response)


def _h_camera_get_zoom(request, response, bridge):
    response = _ok(response)
    if hasattr(response, "zoom_factor"):
        response.zoom_factor = float(bridge.shared.get("zoom", 1.0))
    if hasattr(response, "max_zoom_factor"):
        response.max_zoom_factor = float(bridge.shared.get("max_zoom", 168.0))
    return response


def _h_camera_shoot_photo(request, response, bridge):
    from std_msgs.msg import String

    bridge.publish_sim("/drone/camera/shoot", String(data="single"), String)
    bridge.shared["photo_count"] = int(bridge.shared.get("photo_count", 0)) + 1
    return _ok(response)


def _h_camera_record(request, response, bridge):
    from std_msgs.msg import String

    start = bool(getattr(request, "start_stop", False))
    bridge.shared["recording"] = start
    bridge.publish_sim(
        "/drone/camera/record", String(data="start" if start else "stop"), String
    )
    return _ok(response)


def _h_camera_stream_source(request, response, bridge):
    from std_msgs.msg import String

    # E_DjiCameraManagerStreamSource: 1 = WIDE, 2 = ZOOM.
    source = int(getattr(request, "stream_source", 1))
    lens = "wide" if source == 1 else "tele"
    bridge.publish_sim("/drone/camera/select", String(data=lens), String)
    bridge.shared["stream_source"] = lens
    return _ok(response)


def _h_camera_get_type(request, response, bridge):
    response = _ok(response)
    if hasattr(response, "camera_type"):
        response.camera_type = str(bridge.shared.get("camera_type", "M4E_TRIPLE_LENS"))
    return response


def _h_camera_laser_ranging(request, response, bridge):
    response = _ok(response)
    distance = float(bridge.shared.get("lrf_range", 0.0))
    if hasattr(response, "distance"):
        response.distance = int(distance * 10.0)  # DJI reports decimetres
    if hasattr(response, "latitude"):
        response.latitude = float(bridge.shared.get("lat", 0.0))
    if hasattr(response, "longitude"):
        response.longitude = float(bridge.shared.get("lon", 0.0))
    if hasattr(response, "altitude"):
        response.altitude = int(float(bridge.shared.get("alt", 0.0)))
    if hasattr(response, "enable_lidar"):
        response.enable_lidar = distance > 0.0
    if hasattr(response, "exception"):
        response.exception = 0
    return response


# ── the rest of the aircraft's service surface ───────────────────────────────
#
# Twenty-nine services a live M4E answers that this bridge did not. Verified
# against a real aircraft's `ros2 service list`.
#
# WHAT THESE CAN AND CANNOT DO, because it matters more than the code.
#
# The simulation has no camera MODULE. It has render targets in Gazebo: twelve
# pictures at fixed focal lengths, with no aperture, no shutter, no ISO, no
# focus motor and no SD card. So the settings services below cannot change an
# image. They remember what they were told and hand it back, which is enough
# for the thing payload code actually does at start-up -- read a setting,
# change it, read it back, check it took -- and is not enough to test that a
# photo came out darker.
#
# THAT DISTINCTION IS THE WHOLE POINT OF THEM EXISTING. Before this, a payload
# calling camera_set_iso against the simulation got "service not available" and
# usually crashed at start-up, long before the part worth testing. Now it gets
# a plausible answer and runs, and the journal says the setting went nowhere.
#
# Where the simulation cannot even pretend -- formatting an SD card that does
# not exist, listing files never written -- the service answers success=false
# with a reason, rather than an empty success that sends a payload looking for
# files it will never find.


def _remembered(bridge, key, default):
    return bridge.shared.get(f"cam_{key}", default)


def _make_camera_setter(key, field, default, unit=""):
    """
    A camera setting that is stored and can be read back, but changes no pixel.

    `field` is the request field carrying the value; it is read tolerantly
    because the psdk_interfaces srv definitions name these differently from one
    another (some `value`, some after the setting itself).
    """
    def handler(request, response, bridge):
        value = getattr(request, field, None)
        if value is None:
            # Some srv files name the field after the setting; try that too
            # before giving up, so a definition change does not silently
            # start storing None.
            value = getattr(request, key, default)
        bridge.shared[f"cam_{key}"] = value
        bridge.journal and bridge.journal.report(
            f"srv:{key}", "host->sim", "camera_setting_ignored",
            f"{key}={value}{unit} was stored but changes nothing: the simulated "
            f"cameras have fixed optics", None)
        return _ok(response)
    return handler


def _make_camera_getter(key, field, default):
    def handler(request, response, bridge):
        response = _ok(response)
        value = _remembered(bridge, key, default)
        for name in (field, key):
            if hasattr(response, name):
                setattr(response, name, value)
                break
        return response
    return handler


def _h_camera_get_focus_ring_range(request, response, bridge):
    """The focus ring's travel. Reported as a plausible fixed range."""
    response = _ok(response)
    for name, value in (("min_focus_ring_value", 0), ("max_focus_ring_value", 1000)):
        if hasattr(response, name):
            setattr(response, name, value)
    return response


def _h_camera_get_sd_storage(request, response, bridge):
    """
    SD card capacity. THERE IS NO CARD. Reported as a card with space on it so
    that a payload's "is there room to record" check passes, because the
    alternative -- refusing -- stops start-up code that has nothing to do with
    storage. Nothing is ever written to it.
    """
    response = _ok(response)
    for name, value in (("total_space", 64000), ("remain_space", 64000),
                        ("available_capture_count", 99999),
                        ("available_recording_time", 36000)):
        if hasattr(response, name):
            setattr(response, name, value)
    return response


def _h_camera_format_sd_card(request, response, bridge):
    """Refused: there is no card to format, and pretending would be a lie."""
    return _fail(response, "no SD card in the simulation; nothing to format")


def _h_camera_get_file_list(request, response, bridge):
    """
    Refused: photos taken here are counted, not stored. Returning an empty list
    with success=true would read as "the card is empty", which is a different
    and wrong answer from "this simulation has no filesystem".
    """
    return _fail(response,
                 "the simulation stores no media; "
                 f"{int(bridge.shared.get('photo_count', 0))} photo(s) were counted only")


def _h_camera_setup_streaming(request, response, bridge):
    """
    Start or stop the main camera feed, the way psdk_ros2 does it.

    This is the one camera service with a real effect: it decides whether the
    simulated lens reaches the wrapper's main_camera_stream, by turning the
    route that carries it on or off.
    """
    start = bool(getattr(request, "start_stop", False))
    source = int(getattr(request, "camera_source", 0))
    bridge.shared["camera_streaming"] = start
    bridge.shared["camera_source"] = source
    for key in ("main_camera_out",):
        bridge.submit_slow(lambda b, name=key, on=start: b.set_route_enabled(name, on))
    return _ok(response, True)


def _h_camera_shoot_burst(request, response, bridge):
    from std_msgs.msg import String
    count = int(getattr(request, "photo_burst_count", 3))
    bridge.publish_sim("/drone/camera/shoot", String(data=f"burst:{count}"), String)
    bridge.shared["photo_count"] = int(bridge.shared.get("photo_count", 0)) + count
    return _ok(response)


def _h_camera_shoot_interval(request, response, bridge):
    from std_msgs.msg import String
    count = int(getattr(request, "photo_num_conticap", 0))
    interval = int(getattr(request, "time_interval", 1))
    bridge.shared["interval_shooting"] = (count, interval)
    bridge.publish_sim("/drone/camera/shoot", String(data=f"interval:{interval}"), String)
    return _ok(response)


def _h_camera_stop_shoot(request, response, bridge):
    from std_msgs.msg import String
    bridge.shared["interval_shooting"] = None
    bridge.publish_sim("/drone/camera/shoot", String(data="stop"), String)
    return _ok(response)


# ── flight services the aircraft has and this bridge did not ────────────────


def _h_cancel_go_home(request, response, bridge):
    """Stop a return-to-home. PX4's equivalent is simply leaving the mode."""
    bridge.send_vehicle_command("DO_SET_MODE", param1=1.0, param2=4.0, param3=3.0)
    return _ok(response)


def _h_cancel_landing(request, response, bridge):
    """Abort a descent by switching back to loiter, which stops the descent."""
    bridge.send_vehicle_command("DO_SET_MODE", param1=1.0, param2=4.0, param3=3.0)
    return _ok(response)


def _h_start_confirm_landing(request, response, bridge):
    """
    DJI asks for confirmation when it cannot see the ground properly, and the
    landing stops a few metres up until this is called. PX4 has no such pause,
    so there is never anything to confirm -- the landing already completes on
    its own. Reported as success so a payload's landing sequence is not left
    waiting for a confirmation that will never be needed.
    """
    return _ok(response)


def _h_start_force_landing(request, response, bridge):
    """Land now, ignoring the obstacle checks. PX4: the normal land command."""
    bridge.send_vehicle_command("NAV_LAND")
    return _ok(response)


def _h_set_home_from_current_location(request, response, bridge):
    """Set home to where the aircraft is now, from the fused position."""
    latitude = float(bridge.shared.get("lat", 0.0))
    longitude = float(bridge.shared.get("lon", 0.0))
    if latitude == 0.0 and longitude == 0.0:
        return _fail(response, "no position fix yet, so home would be set to Null Island")
    bridge.send_vehicle_command("DO_SET_HOME", param1=1.0)
    bridge.shared["home_lat"], bridge.shared["home_lon"] = latitude, longitude
    return _ok(response)


def _h_set_local_position_ref(request, response, bridge):
    """
    Make the current position the origin of position_fused.
    
    PX4's local frame origin is set at EKF initialisation and is not re-settable
    from outside, so this records the offset the bridge would have to subtract
    rather than moving PX4's origin. Honest about it in the journal: a payload
    that expects position_fused to jump to zero will not see that happen.
    """
    ned = bridge.shared.get("ned")
    if ned is None:
        return _fail(response, "no local position yet")
    bridge.shared["local_ref"] = tuple(ned)
    return _ok(response)


def _h_set_go_home_alt(request, response, bridge):
    bridge.shared["go_home_alt"] = int(getattr(request, "altitude", 50))
    return _ok(response)


def _h_get_go_home_alt(request, response, bridge):
    response = _ok(response)
    if hasattr(response, "altitude"):
        response.altitude = int(bridge.shared.get("go_home_alt", 50))
    return response


def _h_set_home_from_gps(request, response, bridge):
    import math as _math

    # SetHomeFromGPS carries radians; PX4's DO_SET_HOME wants degrees.
    lat = _math.degrees(float(getattr(request, "latitude", 0.0)))
    lon = _math.degrees(float(getattr(request, "longitude", 0.0)))
    bridge.shared["home"] = (lat, lon)
    bridge.send_vehicle_command("DO_SET_HOME", param1=0.0, param5=lat, param6=lon)
    return _ok(response)


# The directions the aircraft has hardware for, and the one it does not. The
# M4E has no upward stereo pair, so PSDK fails an UP request -- worth refusing
# here in the same way rather than accepting it and streaming nothing.
_PERCEPTION_DIRECTIONS = ("FRONT", "REAR", "LEFT", "RIGHT", "DOWN")
# ...and the subset this simulation actually renders. Adding another means two
# sensors in models/m4e/model.sdf, two image rows plus two camera_info rows in
# config/bridge.yaml, and a pair of routes below.
_PERCEPTION_SIMULATED = ("FRONT",)


def _h_start_perception(request, response, bridge):
    """
    start_perception -- switch the stereo pair on, off, or to another direction.

    Mirrors the aircraft's behaviour rather than the simulation's convenience:

      * exactly one direction streams at a time. Calling again with a different
        direction switches the pair over; it does not add a second one.
      * UP is refused, because the M4E has no upward stereo pair and PSDK
        refuses it there too.
      * a direction the aircraft supports but this world does not render is
        refused explicitly, and says which it is. Returning success and then
        publishing nothing would be the one failure mode that looks like a bug
        in the caller.

    Note what is NOT here: perception_camera_parameters stays unpublished. The
    call behind it segfaults on a real M4E, so a consumer must read
    config/perception_calib.yaml instead, and the simulation refuses to be the
    reason someone writes code that depends on a topic the aircraft cannot
    serve.
    """
    direction = str(getattr(request, "stereo_cameras_direction", "") or "").strip().upper()
    start = bool(getattr(request, "start_stop", False))

    def reply(success, message):
        if hasattr(response, "success"):
            response.success = success
        if hasattr(response, "message"):
            response.message = message
        bridge.state.log.add(INFO if success else WARN, "perception", message)
        return response

    if not start:
        for key in ("stereo_left_out", "stereo_right_out"):
            bridge.submit_slow(lambda b, name=key: b.set_route_enabled(name, False))
        bridge.shared["perception_direction"] = ""
        return reply(True, "perception stereo stopped")

    if direction == "UP":
        return reply(False, "UP is not available: the M4E has no upward stereo pair")
    if direction not in _PERCEPTION_DIRECTIONS:
        return reply(
            False,
            f"unknown direction {direction!r}; expected one of "
            f"{', '.join(_PERCEPTION_DIRECTIONS)}",
        )
    if direction not in _PERCEPTION_SIMULATED:
        return reply(
            False,
            f"{direction} is supported by the aircraft but not rendered in this "
            f"simulation; only {', '.join(_PERCEPTION_SIMULATED)} is available",
        )

    # THE SENSORS MAY NOT BE IN THE WORLD AT ALL. tools/compose_sim.py leaves
    # the perception pair out when the simulation is started with
    # --no-fisheye-cameras (or SIM_FISHEYE_CAMERAS=0), because rendering them
    # costs ~20 megapixels/second that most sessions do not need. Enabling the
    # routes anyway would report success and then publish nothing, which is the
    # one failure mode that looks like a bug in the caller.
    missing = [
        route.sim_topic
        for route in (bridge.route_by_key("stereo_left_out"),
                      bridge.route_by_key("stereo_right_out"))
        if route is not None and bridge.count_publishers(route.sim_topic) == 0
    ]
    if missing:
        return reply(
            False,
            "the perception cameras are not in this world (nothing publishes "
            f"{', '.join(missing)}). Restart with fisheye cameras on: "
            "./run.sh up  (or ./simulation.sh sim --no-cameras to go the other way)",
        )

    for key in ("stereo_left_out", "stereo_right_out"):
        bridge.submit_slow(lambda b, name=key: b.set_route_enabled(name, True))
    bridge.shared["perception_direction"] = direction
    return reply(True, f"perception stereo streaming {direction}")


# OBSTACLE AVOIDANCE IS FIVE SWITCHES ON THE AIRCRAFT, NOT ONE.
#
# This bridge used to serve set_obstacle_avoidance and get_obstacle_avoidance.
# Neither name exists on a Matrice 4E. Its service list has one pair per
# sensing direction and per sensor kind:
#
#   downwards_vo        the downward vision sensors
#   horizontal_vo       the forward/backward/sideways vision sensors
#   horizontal_radar    the horizontal radar, if fitted
#   upwards_vo          the upward vision sensors
#   upwards_radar       the upward radar, if fitted
#
# A payload written against the single switch would fail on the aircraft with
# "service not available", so the single switch is gone and all five pairs are
# served here.
#
# WHAT THEY ACTUALLY DO HERE: nothing but remember the answer. PX4 in this
# simulation has no obstacle avoidance to turn on or off -- there is no APAS,
# no collision prevention configured, and the fisheye pair feeds a demo rather
# than the flight controller. So these are honest state holders: a client can
# set a switch and read back what it set, which is what most start-up code
# checks, and nothing in the simulation changes behaviour as a result. Anything
# relying on avoidance ACTUALLY happening must be tested on hardware.
_OBSTACLE_AVOIDANCE_DIRECTIONS = (
    "downwards_vo",
    "horizontal_vo",
    "horizontal_radar",
    "upwards_vo",
    "upwards_radar",
)


def _make_oa_setter(direction):
    def handler(request, response, bridge):
        on = bool(getattr(request, "obstacle_avoidance_on", False))
        bridge.shared[f"oa_{direction}"] = on
        bridge.state.log.add(
            INFO, "avoidance",
            f"{direction} obstacle avoidance set {'on' if on else 'off'} "
            f"(remembered only; this simulation has no avoidance to engage)")
        return _ok(response)
    return handler


def _make_oa_getter(direction):
    def handler(request, response, bridge):
        if hasattr(response, "obstacle_avoidance_on"):
            response.obstacle_avoidance_on = bool(bridge.shared.get(f"oa_{direction}", False))
        return _ok(response)
    return handler


def _h_set_obstacle_avoidance(request, response, bridge):
    bridge.shared["oa_on"] = bool(getattr(request, "obstacle_avoidance_on", False))
    return _ok(response)


def _h_get_obstacle_avoidance(request, response, bridge):
    response = _ok(response)
    if hasattr(response, "obstacle_avoidance_on"):
        response.obstacle_avoidance_on = bool(bridge.shared.get("oa_on", False))
    return response


def build_service_routes(settings) -> List[ServiceRoute]:
    """
    Construct the service table.

    Flight-control services are std_srvs/Trigger, matching psdk_ros2. Camera
    and gimbal services use the vendored psdk_interfaces types.
    """
    from std_srvs.srv import Trigger

    services: List[ServiceRoute] = [
        # ── flight control ────────────────────────────────────────────────
        ServiceRoute("takeoff", "takeoff", Trigger, _h_takeoff, group="flight",
                     notes="-> VehicleCommand NAV_TAKEOFF"),
        ServiceRoute("land", "land", Trigger, _h_land, group="flight",
                     notes="-> VehicleCommand NAV_LAND"),
        # The KEY stays "return_home" because control.py, node.py's
        # PROJECT_SERVICES and the RViz panel all address it by that name. Only
        # the WIRE name changes, and it had to: the aircraft has no
        # return_home, it has start_go_home (verified against a live M4E's
        # service list).
        ServiceRoute("return_home", "start_go_home", Trigger, _h_return_home, group="flight",
                     notes="-> VehicleCommand NAV_RETURN_TO_LAUNCH"),
        ServiceRoute("obtain_authority", "obtain_ctrl_authority", Trigger,
                     _h_obtain_authority, group="flight",
                     notes="latches /drone/rc/authority true"),
        ServiceRoute("release_authority", "release_ctrl_authority", Trigger,
                     _h_release_authority, group="flight",
                     notes="latches /drone/rc/authority false"),
        ServiceRoute("motors_on", "turn_on_motors", Trigger, _h_motors_on, group="flight",
                     enabled=False, notes="ARMS the vehicle - off by default"),
        ServiceRoute("motors_off", "turn_off_motors", Trigger, _h_motors_off, group="flight",
                     notes="-> DISARM"),
        # ── gimbal ────────────────────────────────────────────────────────
        ServiceRoute("gimbal_set_mode", "gimbal_set_mode", _psdk_srv("GimbalSetMode"),
                     _h_gimbal_set_mode, group="gimbal"),
        ServiceRoute("gimbal_reset", "gimbal_reset", _psdk_srv("GimbalReset"),
                     _h_gimbal_reset, group="gimbal", notes="all three joints -> 0 rad"),
        # ── camera ────────────────────────────────────────────────────────
        ServiceRoute("camera_set_zoom", "camera_set_optical_zoom",
                     _psdk_srv("CameraSetOpticalZoom"), _h_camera_set_zoom, group="camera",
                     notes="-> /drone/camera/zoom + lens select"),
        ServiceRoute("camera_get_zoom", "camera_get_optical_zoom",
                     _psdk_srv("CameraGetOpticalZoom"), _h_camera_get_zoom, group="camera"),
        ServiceRoute("camera_shoot_photo", "camera_shoot_single_photo",
                     _psdk_srv("CameraShootSinglePhoto"), _h_camera_shoot_photo, group="camera"),
        ServiceRoute("camera_record", "camera_record_video",
                     _psdk_srv("CameraRecordVideo"), _h_camera_record, group="camera"),
        ServiceRoute("camera_stream_source", "camera_set_stream_source",
                     _psdk_srv("CameraSetStreamSource"), _h_camera_stream_source, group="camera"),
        ServiceRoute("camera_get_type", "camera_get_type",
                     _psdk_srv("CameraGetType"), _h_camera_get_type, group="camera"),
        ServiceRoute("camera_laser_ranging", "camera_get_laser_ranging_info",
                     _psdk_srv("CameraGetLaserRangingInfo"), _h_camera_laser_ranging,
                     group="camera", notes="answers from the m4e's downward LRF"),
        # ── flight configuration ──────────────────────────────────────────
        ServiceRoute("set_home_from_gps", "set_home_from_gps",
                     _psdk_srv("SetHomeFromGPS"), _h_set_home_from_gps, group="config",
                     notes="request is in radians; converted to degrees for PX4"),
        ServiceRoute("set_go_home_alt", "set_go_home_altitude",
                     _psdk_srv("SetGoHomeAltitude"), _h_set_go_home_alt, group="config"),
        ServiceRoute("get_go_home_alt", "get_go_home_altitude",
                     _psdk_srv("GetGoHomeAltitude"), _h_get_go_home_alt, group="config"),
        # Five directions x {set, get}. See the note above _make_oa_setter for
        # why this is ten rows and not two.
        *[
            ServiceRoute(f"set_oa_{direction}", f"set_{direction}_obstacle_avoidance",
                         _psdk_srv("SetObstacleAvoidance"), _make_oa_setter(direction),
                         group="config",
                         notes="remembered only; no avoidance runs in simulation")
            for direction in _OBSTACLE_AVOIDANCE_DIRECTIONS
        ],
        *[
            ServiceRoute(f"get_oa_{direction}", f"get_{direction}_obstacle_avoidance",
                         _psdk_srv("GetObstacleAvoidance"), _make_oa_getter(direction),
                         group="config",
                         notes="reads back what set_ was given")
            for direction in _OBSTACLE_AVOIDANCE_DIRECTIONS
        ],
        # ── the rest of the aircraft's service surface ───────────────────
        #
        # See the long note above _make_camera_setter for what these can and
        # cannot do. Short version: the settings ones remember and read back,
        # they change no pixel, and that is enough to stop a payload dying at
        # start-up on a missing service.
        #
        # Camera settings: set/get pairs, stored only.
        *[
            row
            for key, service_set, service_get, srv_set, srv_get, field, default in (
                ("aperture", "camera_set_aperture", "camera_get_aperture",
                 "CameraSetAperture", "CameraGetAperture", "aperture", 400),
                ("iso", "camera_set_iso", "camera_get_iso",
                 "CameraSetISO", "CameraGetISO", "iso", 3),
                ("shutter_speed", "camera_set_shutter_speed", "camera_get_shutter_speed",
                 "CameraSetShutterSpeed", "CameraGetShutterSpeed", "shutter_speed", 20),
                ("exposure_mode_ev", "camera_set_exposure_mode_ev",
                 "camera_get_exposure_mode_ev", "CameraSetExposureModeEV",
                 "CameraGetExposureModeEV", "exposure_mode", 1),
                ("focus_mode", "camera_set_focus_mode", "camera_get_focus_mode",
                 "CameraSetFocusMode", "CameraGetFocusMode", "focus_mode", 0),
                ("focus_target", "camera_set_focus_target", "camera_get_focus_target",
                 "CameraSetFocusTarget", "CameraGetFocusTarget", "x_target", 0.5),
                ("focus_ring_value", "camera_set_focus_ring_value",
                 "camera_get_focus_ring_value", "CameraSetFocusRingValue",
                 "CameraGetFocusRingValue", "focus_ring_value", 500),
            )
            for row in (
                ServiceRoute(f"camera_set_{key}", service_set, _psdk_srv(srv_set),
                             _make_camera_setter(key, field, default),
                             group="camera", enabled=False,
                             notes="OFF: stored only; the simulated optics are fixed"),
                ServiceRoute(f"camera_get_{key}", service_get, _psdk_srv(srv_get),
                             _make_camera_getter(key, field, default),
                             group="camera", enabled=False,
                             notes="OFF: reads back what set_ was given"),
            )
        ],
        ServiceRoute("camera_set_infrared_zoom", "camera_set_infrared_zoom",
                     _psdk_srv("CameraSetInfraredZoom"),
                     _make_camera_setter("infrared_zoom", "zoom_factor", 1.0),
                     group="camera", enabled=False,
                     notes="OFF: the m4e has no infrared camera; stored only"),
        ServiceRoute("camera_get_focus_ring_range", "camera_get_focus_ring_range",
                     _psdk_srv("CameraGetFocusRingRange"), _h_camera_get_focus_ring_range,
                     group="camera", enabled=False, notes="OFF: a fixed 0-1000 range"),
        ServiceRoute("camera_get_sd_storage", "camera_get_sd_storage_info",
                     _psdk_srv("CameraGetSDStorageInfo"), _h_camera_get_sd_storage,
                     group="camera", enabled=False,
                     notes="OFF: reports a card with room; nothing is written"),
        ServiceRoute("camera_format_sd_card", "camera_format_sd_card",
                     _psdk_srv("CameraFormatSdCard"), _h_camera_format_sd_card,
                     group="camera", enabled=False, notes="OFF: refuses; there is no card"),
        ServiceRoute("camera_get_file_list", "camera_get_file_list_info",
                     _psdk_srv("CameraGetFileListInfo"), _h_camera_get_file_list,
                     group="camera", enabled=False, notes="OFF: refuses; no media is stored"),
        ServiceRoute("camera_setup_streaming", "camera_setup_streaming",
                     _psdk_srv("CameraSetupStreaming"), _h_camera_setup_streaming,
                     group="camera", enabled=False,
                     notes="OFF: the one camera service with a real effect -- "
                           "turns main_camera_out on or off"),
        ServiceRoute("camera_shoot_burst", "camera_shoot_burst_photo",
                     _psdk_srv("CameraShootBurstPhoto"), _h_camera_shoot_burst,
                     group="camera", enabled=False, notes="OFF: counted, not stored"),
        ServiceRoute("camera_shoot_interval", "camera_shoot_interval_photo",
                     _psdk_srv("CameraShootIntervalPhoto"), _h_camera_shoot_interval,
                     group="camera", enabled=False, notes="OFF: counted, not stored"),
        ServiceRoute("camera_stop_shoot", "camera_stop_shoot_photo",
                     _psdk_srv("CameraStopShootPhoto"), _h_camera_stop_shoot,
                     group="camera", enabled=False, notes="OFF: stops interval shooting"),

        # Flight services. These DO something.
        ServiceRoute("cancel_go_home", "cancel_go_home", Trigger, _h_cancel_go_home,
                     group="flight", notes="back to loiter, which ends the return"),
        ServiceRoute("cancel_landing", "cancel_landing", Trigger, _h_cancel_landing,
                     group="flight", notes="back to loiter, which stops the descent"),
        ServiceRoute("start_confirm_landing", "start_confirm_landing", Trigger,
                     _h_start_confirm_landing, group="flight",
                     notes="PX4 never pauses for confirmation, so this always succeeds"),
        ServiceRoute("start_force_landing", "start_force_landing", Trigger,
                     _h_start_force_landing, group="flight", notes="land now"),
        ServiceRoute("set_home_from_current_location", "set_home_from_current_location",
                     Trigger, _h_set_home_from_current_location, group="config",
                     notes="home at the current fused position"),
        ServiceRoute("set_local_position_ref", "set_local_position_ref", Trigger,
                     _h_set_local_position_ref, group="config",
                     notes="records the offset; PX4's own origin cannot be moved"),

        # Perception. Until this existed the stand-in served an auto-derived
        # stub for the name (standin.snake_case turns
        # PerceptionStereoVisionSetup into perception_stereo_vision_setup),
        # which answered success=True and streamed nothing. The real wrapper
        # calls it start_perception, which is the name used here.
        ServiceRoute("start_perception", "start_perception",
                     _psdk_srv("PerceptionStereoVisionSetup"), _h_start_perception,
                     group="config",
                     notes="FRONT only in simulation; UP refused as on the M4E"),
    ]

    # THE POLICY IS A DEFAULT, SO IT GOES FIRST. With no Manifold there is no
    # other server to collide with, so every service this bridge knows how to
    # answer is served; a service left off is a mission that hangs on a client
    # call with no indication why.
    #
    # Applying this AFTER apply_overrides would silently discard whatever the
    # operator had saved in their profile, which is the opposite of what an
    # override is for.
    for service in services:
        if service.available:
            service.enabled = True
            service.role = ServiceRole.SERVE

    for service in services:
        service.apply_overrides(settings.overrides_for(f"srv:{service.key}"))
    return services
