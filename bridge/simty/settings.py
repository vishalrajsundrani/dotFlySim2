"""
Runtime settings and JSON profile persistence.

Two kinds of state live here:

  * Settings   -- global knobs the operator edits from the TUI's SETTINGS
                  screen. SETTING_SPECS drives that screen generically, so
                  adding a knob means adding one spec row.
  * overrides  -- per-route deltas (enabled / direction / QoS / rate cap).
                  Saved alongside the settings so a tuned session survives a
                  restart.

The profile is a plain JSON file. Default location honours XDG and falls back
to ~/.dotflysim/simty_profile.json.
"""

import json
import os
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Tuple

from .state import DEBUG, ERROR, INFO, LEVEL_ORDER, WARN

DEFAULT_PROFILE_PATH = os.path.join(
    os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config")),
    "dotflysim",
    "simty_profile.json",
)

WRAPPER_PREFIX = "/wrapper/psdk_ros2"


def _default_params() -> Dict[str, Any]:
    """Converter tunables. Everything a converter reads via ctx.p() lives here."""
    return {
        # RC stick shaping
        "rc_max": 10000.0,
        "deadzone": 0.02,
        "max_speed": 15.0,
        "max_climb": 5.0,
        "max_yaw_rate": 1.5,
        # RC authority interpretation
        "rc_device_mode": 0,
        "require_control_auth": False,
        "rc_fresh_s": 2.0,
        # How old the heading may be before rotating sticks into NED is called
        # out as using a stale attitude.
        "rc_heading_stale_s": 2.0,
        # gimbal mechanical limits (models/m4e/model.sdf)
        "gimbal_pan_deg": 60.0,
        "gimbal_roll_deg": 47.0,
        "gimbal_tilt_min_deg": -90.0,
        "gimbal_tilt_max_deg": 35.0,
        # What to subtract from DJI's ground-frame gimbal yaw to get the
        # body-relative pan joint angle: aircraft (the wrapper's own attitude),
        # sim (the simulated vehicle's heading) or none (copy yaw through).
        # "aircraft" is the only value that stays correct when the real and
        # simulated aircraft face different ways. See
        # converters.gimbal_angles_to_joint_cmds.
        "gimbal_yaw_reference": "aircraft",
        "gimbal_heading_stale_s": 2.0,
        # battery model
        "battery_capacity_mah": 5000.0,
        "battery_index": 0,
        # Which clock goes into header.stamp on host-bound messages.
        #
        #   auto  sim time whenever /clock is arriving, wall time otherwise. A
        #         real Manifold publishes no /clock, so hardware keeps wall
        #         time without anyone having to remember to set this.
        #   sim   same as auto today; kept distinct so the intent is recorded
        #         in the profile rather than inferred from behaviour.
        #   wall  always wall time -- what this did before the knob existed.
        #
        # auto is the default because the alternative is silent: images bridged
        # out of Gazebo carry sim time regardless of this setting, so under a
        # real-time factor below 1 a wall-stamped IMU and a sim-stamped frame
        # drift apart without bound and nothing reports it. It lives in params
        # rather than in Settings so that changing it takes effect on routes
        # that are already running -- params is one dict shared by every live
        # ConversionContext. simty/clock.py has the full argument.
        "stamp_source": "auto",
        # How stale the last VehicleAttitude may be before sensor_combined_to_imu
        # stops splicing it into the Imu message and falls back to REP-145's
        # "no orientation" encoding. PX4 publishes attitude far faster than the
        # 50 Hz this route runs at, so anything approaching this is a dropout,
        # not jitter.
        "imu_attitude_stale_s": 0.2,
        # frames
        "gps_frame_id": "map",
        "gimbal_frame_id": "gimbal",
        "body_frame_id": "base_link",
        # Video: per-frame size above which a route says so in the journal. An
        # uncompressed 1080p RGB frame is ~6 MB; 2 MB catches the tiers that will
        # hurt the wireless link without nagging about preview-sized frames.
        "image_warn_bytes": 2_000_000,
    }


def _default_sim_topics() -> Dict[str, str]:
    """
    PX4 publishes versioned topic names and they change between releases, so
    they are settings rather than constants. The DISCOVERY screen flags any
    route whose sim topic is absent from the live graph, which is how you catch
    a rename without reading PX4's dds_topics.yaml.
    """
    return {
        "vehicle_status": "/fmu/out/vehicle_status_v4",
        "local_position": "/fmu/out/vehicle_local_position_v1",
        # NOT vehicle_global_position_v1. PX4's message versioning is per
        # message, not global: VehicleStatus and VehicleLocalPosition carry a
        # MESSAGE_VERSION and are published with the _v4/_v1 suffix, while
        # VehicleGlobalPosition in this release carries none and is published
        # unversioned. Subscribing to the _v1 name matched nothing -- silently,
        # because a topic with no publisher still appears in `ros2 topic list`
        # once something subscribes to it. The visible symptom was a takeoff
        # that always went to MIS_TAKEOFF_ALT: no global altitude means no AMSL
        # conversion, so the altitude went out as NaN. Verify with
        # `ros2 topic info /fmu/out/<name>` and look at Publisher count.
        "global_position": "/fmu/out/vehicle_global_position",
        "attitude": "/fmu/out/vehicle_attitude",
        "battery": "/fmu/out/battery_status_v1",
        "land_detected": "/fmu/out/vehicle_land_detected",
        "sensor_gps": "/fmu/out/vehicle_gps_position",
        # _v2 here, unversioned above: same reason, opposite direction.
        "home_position": "/fmu/out/home_position_v2",
        # Not a route: node.py subscribes to this directly to explain why PX4
        # refuses to arm. "Arming denied: resolve system health failures first"
        # is the whole message PX4 sends; the reason is only in these flags.
        "failsafe_flags": "/fmu/out/failsafe_flags",
        "trajectory_setpoint": "/fmu/in/trajectory_setpoint",
        "offboard_control_mode": "/fmu/in/offboard_control_mode",
        "vehicle_command": "/fmu/in/vehicle_command",
        "lrf_range": "/drone/lrf/range",
        # Sources for the wider wrapper telemetry surface. Unversioned because
        # neither message is versioned in PX4's dds_topics.yaml, unlike the
        # _vN topics above.
        #
        # sensor_combined is also where body angular rates come from: PX4's
        # /fmu/out/vehicle_angular_velocity is commented out in dds_topics.yaml,
        # so it publishes nothing unless you uncomment it and rebuild.
        "sensor_combined": "/fmu/out/sensor_combined",
        "odometry": "/fmu/out/vehicle_odometry",
        # Simulated camera feeds, for the video routes. camera_active is whatever
        # lens camera_switcher.py has selected; the preview tier is an order of
        # magnitude cheaper on the wire than the 4K one.
        "camera_active": "/drone/camera/active/image_raw",
        "camera_preview": "/drone/camera/wide/preview/image_raw",
    }


@dataclass
class Settings:
    # -- identity / graph --
    domain: int = 42
    wrapper_prefix: str = WRAPPER_PREFIX

    # -- safety: both default OFF. Arming an aircraft, even a simulated one,
    #    is an explicit operator action, never a side effect of starting a
    #    bridge.
    auto_offboard: bool = False
    auto_arm: bool = False
    offboard_hz: float = 10.0
    rc_timeout_s: float = 1.0

    # -- project testing mode --
    #    Which control topic a project under test steers with when the mode is
    #    switched on (node.SimtyBridge.PROJECT_SETPOINTS names them). The ENU
    #    velocity setpoint is the default because it is a real psdk_ros2 control
    #    topic and, being ground-frame, needs no heading -- so a project written
    #    against it has one less thing to get wrong. The mode itself is NOT
    #    persisted as a setting: it is entered deliberately, per session, and a
    #    bridge that came up already wired for a test would be a trap.
    project_setpoint: str = "velocity"

    # -- display --
    refresh_hz: float = 5.0
    show_live_data: bool = True
    log_level: int = INFO
    stale_after_s: float = 2.0
    log_rx_traffic: bool = False  # per-message DEBUG logging; very noisy

    # -- discovery --
    scan_period_s: float = 2.0

    # -- synthetic Manifold --
    mock_enabled: bool = False
    mock_rc_hz: float = 20.0
    mock_fast_hz: float = 10.0
    mock_slow_hz: float = 1.0
    mock_fly_circle: bool = True

    # -- conversion journal --
    #
    # journal_min_interval_s throttles repeat text output for one (route, code)
    # pair. Counters stay exact regardless; only the log lines are thinned.
    # journal_active_window_s decides how long after its last occurrence an
    # issue still counts as "happening now" on the dashboard and in diagnostics.
    journal_min_interval_s: float = 5.0
    journal_active_window_s: float = 10.0

    # -- control room: RViz + diagnostics --
    #
    # Marker publication is subscriber-gated in viz.py, so leaving this on costs
    # nothing while RViz is closed. Same reasoning as the lazy camera bridges.
    viz_enabled: bool = True
    viz_hz: float = 4.0
    viz_trail_length: int = 400  # poses kept in the /simty/viz/path trail
    viz_board_frame_prefix: str = "simty"
    # Where each RViz board sits relative to map, in metres (east, north).
    # Far enough from the aircraft that the world view stays uncluttered; the
    # saved views in gui/rviz/*.rviz target these frames directly.
    viz_flow_board_at: List[float] = field(default_factory=lambda: [60.0, 0.0])
    viz_service_board_at: List[float] = field(default_factory=lambda: [60.0, -40.0])
    viz_status_board_at: List[float] = field(default_factory=lambda: [60.0, 30.0])
    viz_drone_mesh: str = "file:///home/developer/gz_models/m4e/meshes/m4_faceless.dae"
    publish_diagnostics: bool = True
    publish_events: bool = True  # JSON conversion events on /simty/conversion_events

    # -- remote control (/simty/control, /simty/state) --
    #
    # This is what the RViz panels and gui/bridge_panel.py drive. Turning it off
    # makes the console the only writable surface -- worth doing on a shared
    # domain where you do not want another machine redirecting your conversions.
    enable_control: bool = True
    control_state_hz: float = 2.0

    # -- Manifold provisioning (SSH) and the local stand-in --
    #
    # Everything here defaults to inert. Starting a wrapper on real hardware
    # over SSH is an outward-facing action, and standing in for the Manifold
    # locally changes what "connected" means -- neither should happen because a
    # bridge was launched.
    auto_provision: bool = False
    provision_policy: str = "ssh_then_standin"  # ssh | standin | ssh_then_standin
    provision_after_s: float = 20.0  # how long SEARCHING must persist first
    provision_retry_s: float = 120.0  # gap between SSH attempts after a failure
    manifold_host: str = ""
    manifold_user: str = "dji"
    manifold_port: int = 22
    manifold_ssh_key: str = ""  # empty: agent / default keys
    manifold_ssh_timeout_s: float = 10.0
    # Ran through `bash -lc` on the Manifold. env_setup exists because ROS lives
    # in ~/.bashrc on most Jetson images and a non-interactive shell skips it.
    manifold_env_setup: str = "source ~/.bashrc"
    manifold_launch_cmd: str = "ros2 launch psdk_wrapper psdk_wrapper.launch.py"
    manifold_stop_cmd: str = "pkill -f psdk_wrapper"
    manifold_probe_cmd: str = "ros2 topic list"
    standin_services: bool = True  # stand-in also serves the un-routed services

    # -- nested --
    params: Dict[str, Any] = field(default_factory=_default_params)
    sim_topics: Dict[str, str] = field(default_factory=_default_sim_topics)
    overrides: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    # ---- persistence ----

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True)

    @classmethod
    def load(cls, path: str) -> Tuple["Settings", str]:
        """
        Returns (settings, note). `note` describes what happened so the caller
        can log it -- a missing profile is normal, a corrupt one is not.
        """
        if not path or not os.path.exists(path):
            return cls(), f"no profile at {path}; using defaults"
        try:
            with open(path, "r", encoding="utf-8") as handle:
                raw = json.load(handle)
        except (OSError, ValueError) as exc:
            return cls(), f"profile {path} unreadable ({exc}); using defaults"

        settings = cls()
        known = set(asdict(settings).keys())
        for key, value in raw.items():
            if key not in known:
                continue
            if key in ("params", "sim_topics"):
                # Merge, so a profile written by an older build does not drop
                # knobs added since.
                current = getattr(settings, key)
                if isinstance(value, dict):
                    current.update(value)
            elif key == "overrides":
                if isinstance(value, dict):
                    settings.overrides = value
            else:
                setattr(settings, key, value)
        return settings, f"loaded profile {path}"

    def save(self, path: str) -> str:
        try:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(self.to_json())
            return f"saved profile {path}"
        except OSError as exc:
            return f"could not save profile {path}: {exc}"

    # ---- per-route overrides ----

    def override(self, key: str, **kwargs) -> None:
        self.overrides.setdefault(key, {}).update(kwargs)

    def overrides_for(self, key: str) -> Dict[str, Any]:
        return dict(self.overrides.get(key, {}))

    def cycle_log_level(self) -> int:
        index = LEVEL_ORDER.index(self.log_level) if self.log_level in LEVEL_ORDER else 1
        self.log_level = LEVEL_ORDER[(index + 1) % len(LEVEL_ORDER)]
        return self.log_level

    # ---- Manifold provisioning helpers ----

    @property
    def ssh_target(self) -> str:
        """user@host, or "" when no host has been configured."""
        if not self.manifold_host:
            return ""
        return f"{self.manifold_user}@{self.manifold_host}" if self.manifold_user else self.manifold_host

    @property
    def ssh_configured(self) -> bool:
        return bool(self.manifold_host)

    def cycle_stamp_source(self) -> str:
        order = ["auto", "sim", "wall"]
        current = self.params.get("stamp_source", "auto")
        try:
            index = order.index(current)
        except ValueError:
            index = -1
        chosen = order[(index + 1) % len(order)]
        self.params["stamp_source"] = chosen
        return chosen

    def cycle_provision_policy(self) -> str:
        order = ["ssh_then_standin", "ssh", "standin"]
        try:
            index = order.index(self.provision_policy)
        except ValueError:
            index = -1
        self.provision_policy = order[(index + 1) % len(order)]
        return self.provision_policy


# ── generic editor description, consumed by tui.py ───────────────────────────
#
# (attribute, label, kind, lo, hi, step)
#   kind: "bool" | "float" | "int" | "level"
SETTING_SPECS: List[Tuple[str, str, str, float, float, float]] = [
    ("auto_offboard", "Auto-engage OFFBOARD on RC authority", "bool", 0, 0, 0),
    ("auto_arm", "Auto-ARM when offboard engages  (DANGEROUS)", "bool", 0, 0, 0),
    ("offboard_hz", "Offboard heartbeat rate (Hz)", "float", 2.0, 50.0, 1.0),
    ("rc_timeout_s", "RC stale timeout (s)", "float", 0.1, 10.0, 0.1),
    ("show_live_data", "Show live payload column", "bool", 0, 0, 0),
    ("log_rx_traffic", "Log every message (very noisy)", "bool", 0, 0, 0),
    ("log_level", "Log level", "level", 0, 0, 0),
    ("refresh_hz", "TUI refresh rate (Hz)", "float", 1.0, 20.0, 1.0),
    ("stale_after_s", "Mark route stale after (s)", "float", 0.5, 30.0, 0.5),
    ("scan_period_s", "Graph rescan period (s)", "float", 0.5, 30.0, 0.5),
    ("mock_rc_hz", "Mock RC rate (Hz)", "float", 1.0, 100.0, 1.0),
    ("mock_fast_hz", "Mock telemetry rate (Hz)", "float", 1.0, 100.0, 1.0),
    ("mock_slow_hz", "Mock status rate (Hz)", "float", 0.2, 20.0, 0.2),
    ("mock_fly_circle", "Mock flies a circle", "bool", 0, 0, 0),
    # -- control room --
    ("viz_enabled", "Publish RViz control-room markers", "bool", 0, 0, 0),
    ("viz_hz", "RViz marker rate (Hz)", "float", 1.0, 20.0, 1.0),
    ("publish_diagnostics", "Publish /simty/diagnostics", "bool", 0, 0, 0),
    ("publish_events", "Publish /simty/conversion_events (JSON)", "bool", 0, 0, 0),
    ("enable_control", "Accept commands on /simty/control", "bool", 0, 0, 0),
    ("control_state_hz", "/simty/state publish rate (Hz)", "float", 0.2, 10.0, 0.2),
    ("journal_min_interval_s", "Warning repeat interval (s)", "float", 0.0, 60.0, 1.0),
    ("journal_active_window_s", "Warning 'active' window (s)", "float", 1.0, 120.0, 1.0),
    # -- provisioning --
    ("auto_provision", "Auto-provision Manifold while SEARCHING", "bool", 0, 0, 0),
    ("provision_after_s", "Wait before provisioning (s)", "float", 5.0, 300.0, 5.0),
    ("provision_retry_s", "Retry SSH after (s)", "float", 30.0, 900.0, 30.0),
    ("standin_services", "Stand-in serves un-routed services too", "bool", 0, 0, 0),
]

# Converter tunables get the same treatment.
PARAM_SPECS: List[Tuple[str, str, str, float, float, float]] = [
    ("max_speed", "RC max horizontal speed (m/s)", "float", 0.5, 30.0, 0.5),
    ("max_climb", "RC max climb rate (m/s)", "float", 0.5, 15.0, 0.5),
    ("max_yaw_rate", "RC max yaw rate (rad/s)", "float", 0.1, 4.0, 0.1),
    ("deadzone", "RC stick deadzone (fraction)", "float", 0.0, 0.3, 0.01),
    ("rc_max", "RC full-scale value", "float", 100.0, 40000.0, 100.0),
    ("require_control_auth", "Require control_auth==1 for authority", "bool", 0, 0, 0),
    ("rc_heading_stale_s", "Warn when heading older than (s)", "float", 0.2, 30.0, 0.2),
    ("gimbal_pan_deg", "Gimbal pan limit (deg)", "float", 5.0, 180.0, 1.0),
    ("gimbal_roll_deg", "Gimbal roll limit (deg)", "float", 5.0, 90.0, 1.0),
    ("battery_capacity_mah", "Battery design capacity (mAh)", "float", 500.0, 40000.0, 500.0),
]

__all__ = [
    "DEFAULT_PROFILE_PATH",
    "PARAM_SPECS",
    "SETTING_SPECS",
    "Settings",
    "WRAPPER_PREFIX",
    "DEBUG",
    "INFO",
    "WARN",
    "ERROR",
]
