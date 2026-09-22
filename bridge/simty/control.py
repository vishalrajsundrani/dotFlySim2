"""
The bridge's remote control surface: two topics that let anything drive it.

WHY A TOPIC PAIR AND NOT SERVICES WITH CUSTOM TYPES
--------------------------------------------------
"Set route X to sim->host" wants a service with a string request. std_srvs has
Trigger and SetBool and nothing with a string in it, and a custom .srv would mean
editing psdk_interfaces, a colcon build, and a Dockerfile change -- the whole
point of this work is that it drops into a mounted directory. So the control
surface is:

  /simty/control          std_msgs/String  one command per message
  /simty/control_result   std_msgs/String  ok/err + explanation + the echo
  /simty/state            std_msgs/String  a full snapshot, TRANSIENT_LOCAL

The wire format is line/word oriented rather than JSON for one specific reason:
it has to be parseable in three places -- Python (the Tkinter panel), C++ (the
RViz panels, with no JSON library in the image), and a human's shell:

  ros2 topic pub --once /simty/control std_msgs/String \\
      "data: 'direction route=rc_setpoint value=sim->host'"
  ros2 topic echo /simty/state

A command is `verb key=value key=value`. State is one record per line, fields
separated by tabs, first field a single-letter record type. Splitting on tabs is
three lines of C++ and one of Python; a JSON parser is neither.

/simty/conversion_events stays JSON: it is consumed by scripts, where JSON is
the right answer. Different audience, different format, on purpose.

EVERYTHING GOES THROUGH bridge.submit()
---------------------------------------
Commands arrive on a subscription callback, which runs on the executor thread --
but realising a route destroys and recreates endpoints, and doing that from
inside a callback while the same executor is dispatching is how you get a
half-built subscription. So each command is applied through the same command
queue the TUI uses. One mutation path, one place where entity lifetime is
handled.

SERVICE INVOCATION
------------------
`call route=takeoff` runs a bridged service. For a SERVE-role row the handler is
invoked directly -- that is exactly what a DDS client's call would do, minus the
round trip, and it means the panel needs no psdk_interfaces types of its own. For
a PROXY row the request goes out through the existing client to the real
Manifold, asynchronously, and the result arrives on /simty/control_result when it
arrives.

Services that command the aircraft (arm, disarm, takeoff, land, RTL) require
`confirm=1`. A one-click ARM button on a panel that also has a camera preview is
exactly the sort of thing that gets pressed by accident.
"""

import json
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from std_msgs.msg import Float64, String

from . import qos as qos_mod
from .registry import Direction, ServiceRole
from .state import ERROR, INFO, LEVEL_NAMES, WARN

CONTROL_TOPIC = "/simty/control"
RESULT_TOPIC = "/simty/control_result"
STATE_TOPIC = "/simty/state"

STATE_SCHEMA = 1

# Bridged services that move the aircraft. `call` refuses these without
# confirm=1, so a mis-click on a panel cannot arm a vehicle.
DANGEROUS_SERVICES = frozenset(
    {"takeoff", "land", "return_home", "motors_on", "motors_off"}
)

CAMERA_LENSES = ("wide", "medium_tele", "tele")
CAMERA_MODES = ("photo", "video_4k", "video_fhd", "preview")
CAMERA_SELECT_TOPIC = "/drone/camera/select"
CAMERA_MODE_TOPIC = "/drone/camera/mode"
CAMERA_ZOOM_TOPIC = "/drone/camera/zoom"
CAMERA_ACTIVE_TOPIC = "/drone/camera/active/image_raw"

# What a panel should offer as feed choices, in preference order. The compressed
# active feed is first because it is the only one that is cheap: camera_switcher
# JPEGs it to well under 1 MB, while a raw frame is ~3.7 MB and the per-lens
# topics are lazy -- subscribing to one makes Gazebo start rendering that sensor.
CAMERA_FEEDS: Tuple[Tuple[str, str, str], ...] = (
    (CAMERA_ACTIVE_TOPIC + "/compressed", "compressed", "active lens, JPEG (cheapest)"),
    (CAMERA_ACTIVE_TOPIC, "raw", "active lens, raw"),
    ("/drone/camera/wide/image_raw", "raw", "wide, raw (renders on subscribe)"),
    ("/drone/camera/medium_tele/image_raw", "raw", "medium_tele, raw (renders on subscribe)"),
    ("/drone/camera/tele/image_raw", "raw", "tele, raw (renders on subscribe)"),
    ("/drone/camera/wide/preview/image_raw", "raw", "wide preview 640x360"),
    ("/drone/camera/medium_tele/preview/image_raw", "raw", "medium_tele preview"),
    ("/drone/camera/tele/preview/image_raw", "raw", "tele preview"),
)


def _clean(value) -> str:
    """
    Tabs and newlines are the record separators, so no field may contain one.
    """
    text = "-" if value is None else str(value)
    return text.replace("\t", " ").replace("\n", " ").replace("\r", " ") or "-"


def _flag(value) -> str:
    return "1" if value else "0"


def parse_command(text: str) -> Tuple[str, Dict[str, str]]:
    """
    "direction route=rc_setpoint value=off" -> ("direction", {...}).

    Bare words after the verb become flags with value "1", so `call
    route=takeoff confirm` works as well as `confirm=1` -- a human typing this
    into `ros2 topic pub` should not have to remember which.
    """
    parts = text.strip().split()
    if not parts:
        return "", {}
    verb = parts[0].lower()
    args: Dict[str, str] = {}
    for part in parts[1:]:
        if "=" in part:
            key, _, value = part.partition("=")
            args[key.strip().lower()] = value.strip()
        else:
            args[part.strip().lower()] = "1"
    return verb, args


def _truthy(value: str, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on", "enable", "enabled")


class BridgeControl:
    """
    Created by SimtyBridge. Owns the two publishers, the command subscription and
    the camera-state mirror.
    """

    def __init__(self, bridge):
        self._bridge = bridge
        self._settings = bridge.settings
        self._state = bridge.state
        self._journal = bridge.journal
        self._camera = {"lens": "wide", "mode": "photo", "zoom": 1.0, "at": 0.0}
        self._commands_seen = 0
        self._last_state_at = 0.0
        self._dirty = True

        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # TRANSIENT_LOCAL: a panel that opens after the bridge gets the current
        # snapshot immediately instead of showing an empty table for half a
        # second, which reads as "the bridge is not running".
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._result_pub = bridge.create_publisher(String, RESULT_TOPIC, reliable)
        self._state_pub = bridge.create_publisher(String, STATE_TOPIC, latched)
        bridge.create_subscription(String, CONTROL_TOPIC, self._on_command, reliable)

        # Mirror the camera switcher's own inputs so the panels can show what is
        # selected rather than what they last asked for.
        for topic, msg_type, key in (
            (CAMERA_SELECT_TOPIC, String, "lens"),
            (CAMERA_MODE_TOPIC, String, "mode"),
            (CAMERA_ZOOM_TOPIC, Float64, "zoom"),
        ):
            bridge.create_subscription(
                msg_type, topic, self._make_camera_watch(key), qos_mod.build("compat")
            )

    # ── camera mirror ────────────────────────────────────────────────────────

    def _make_camera_watch(self, key: str):
        def watch(msg):
            value = getattr(msg, "data", None)
            if value is None:
                return
            self._camera[key] = float(value) if key == "zoom" else str(value)
            self._camera["at"] = time.time()
            self._dirty = True

        return watch

    # ── inbound commands ─────────────────────────────────────────────────────

    def _reply(self, ok: bool, message: str, command: str) -> None:
        self._result_pub.publish(
            String(data=f"{'ok' if ok else 'err'}\t{_clean(message)}\t{_clean(command)}")
        )
        self._state.log.add(
            INFO if ok else WARN, "control", f"{command} -> {message}"
        )
        self._dirty = True

    def _on_command(self, msg) -> None:
        command = (getattr(msg, "data", "") or "").strip()
        if not command:
            return
        self._commands_seen += 1
        verb, args = parse_command(command)
        handler = getattr(self, f"_cmd_{verb}", None)
        if handler is None:
            self._reply(
                False,
                f"unknown command '{verb}'; try one of "
                f"{', '.join(self.verbs())}",
                command,
            )
            return

        # Apply on the executor's command queue, never straight from this
        # callback: realising a route tears down and rebuilds endpoints.
        def apply(_bridge):
            try:
                ok, message = handler(args)
            except Exception as exc:
                ok, message = False, f"{type(exc).__name__}: {exc}"
            self._reply(ok, message, command)

        self._bridge.submit(apply)

    def verbs(self) -> List[str]:
        return sorted(
            name[len("_cmd_"):] for name in dir(self) if name.startswith("_cmd_")
        )

    # ── route commands ───────────────────────────────────────────────────────

    def _route(self, args) -> Tuple[Optional[Any], str]:
        key = args.get("route") or args.get("key") or ""
        if not key:
            return None, "no route= given"
        route = self._bridge.route_by_key(key)
        if route is None:
            return None, f"no route '{key}'"
        return route, ""

    def _cmd_direction(self, args):
        """direction route=<key> value=host->sim|sim->host|off"""
        route, problem = self._route(args)
        if route is None:
            return False, problem
        wanted = (args.get("value") or args.get("dir") or "").lower()
        aliases = {
            "host->sim": Direction.HOST_TO_SIM,
            "h2s": Direction.HOST_TO_SIM,
            "wrapper->sim": Direction.HOST_TO_SIM,
            "sim->host": Direction.SIM_TO_HOST,
            "s2h": Direction.SIM_TO_HOST,
            "sim->wrapper": Direction.SIM_TO_HOST,
            "off": Direction.OFF,
            "none": Direction.OFF,
        }
        if wanted not in aliases:
            return False, (
                "value must be host->sim, sim->host or off "
                f"(got '{wanted}')"
            )
        direction = aliases[wanted]
        self._bridge.set_route_direction(route.key, direction)
        if direction is Direction.OFF:
            return True, f"{route.key}: conversion off, no endpoints"
        return True, (
            f"{route.key}: {route.source_topic} -> {route.sink_topic}"
        )

    def _cmd_enable(self, args):
        """enable route=<key> value=1|0|toggle"""
        route, problem = self._route(args)
        if route is None:
            return False, problem
        wanted = (args.get("value") or "toggle").lower()
        if wanted == "toggle":
            self._bridge.toggle_route(route.key)
        else:
            self._bridge.set_route_enabled(route.key, _truthy(wanted))
        return True, f"{route.key} {'enabled' if route.enabled else 'disabled'}"

    def _cmd_qos(self, args):
        """qos route=<key> side=sub|pub [value=<preset>]"""
        route, problem = self._route(args)
        if route is None:
            return False, problem
        side = (args.get("side") or "sub").lower()
        if side not in ("sub", "pub"):
            return False, "side must be sub or pub"
        preset = args.get("value")
        if preset:
            if preset not in qos_mod.PRESETS:
                return False, (
                    f"unknown QoS preset '{preset}'; have "
                    f"{', '.join(qos_mod.ORDER)}"
                )
            self._bridge.set_route_qos(route.key, side, preset)
        else:
            self._bridge.cycle_route_qos(route.key, side)
        current = route.sub_qos if side == "sub" else route.pub_qos
        return True, f"{route.key} {side} QoS -> {current}"

    def _cmd_rate(self, args):
        """rate route=<key> value=<hz>   (0 = unlimited)"""
        route, problem = self._route(args)
        if route is None:
            return False, problem
        try:
            hz = max(0.0, float(args.get("value", "0")))
        except ValueError:
            return False, f"value must be a number of Hz (got '{args.get('value')}')"
        self._bridge.set_route_rate(route.key, hz)
        return True, f"{route.key} rate cap -> {'unlimited' if hz == 0 else f'{hz:g} Hz'}"

    def _cmd_all(self, args):
        """all value=on|off|host->sim|sim->host [group=<group>]"""
        wanted = (args.get("value") or "").lower()
        group = args.get("group") or None
        if wanted in ("on", "off", "1", "0"):
            self._bridge.set_all_routes(_truthy(wanted) or wanted == "on", group)
            return True, f"{group or 'all'} routes {'enabled' if wanted in ('on', '1') else 'disabled'}"
        aliases = {
            "host->sim": Direction.HOST_TO_SIM,
            "sim->host": Direction.SIM_TO_HOST,
            "none": Direction.OFF,
        }
        if wanted in aliases:
            changed = self._bridge.set_all_directions(aliases[wanted], group)
            return True, f"{changed} route(s) -> {wanted}"
        return False, "value must be on, off, host->sim, sim->host or none"

    def _cmd_project(self, args):
        """project value=on|off|toggle [setpoint=velocity|position|body|rc]

        Project testing mode: point every route and service a project under test
        needs, and remember what they were. See node.set_project_mode -- the
        interesting part is that turning it off restores the route table rather
        than leaving the bridge wide open.
        """
        wanted = (args.get("value") or "toggle").lower()
        if wanted in ("toggle", ""):
            enable = not bool(getattr(self._bridge, "project_mode", False))
        elif wanted in ("on", "off", "1", "0", "true", "false", "yes", "no"):
            enable = wanted in ("on", "1", "true", "yes")
        else:
            return False, f"value must be on, off or toggle (got '{wanted}')"
        setpoint = args.get("setpoint") or args.get("control") or "velocity"
        message = self._bridge.set_project_mode(enable, setpoint)
        # set_project_mode returns its own refusal text when the setpoint name or
        # the route is unusable, and in that case nothing was changed.
        ok = enable is False or bool(getattr(self._bridge, "project_mode", False))
        return ok, message

    def _cmd_replay(self, args):
        """replay value=on|off|toggle

        Bag replay mode: switch every topic route OFF so a `ros2 bag play` owns
        the wrapper surface, and keep the flight services SERVE so a caller
        still gets an answer. See node.set_replay_mode.

        The point of the mode is the thing it prevents: the bag and the bridge
        publish the same /wrapper/psdk_ros2 names, and two writers on one topic
        interleave rather than fail.
        """
        wanted = (args.get("value") or "toggle").lower()
        if wanted in ("toggle", ""):
            enable = not bool(getattr(self._bridge, "replay_mode", False))
        elif wanted in ("on", "off", "1", "0", "true", "false", "yes", "no"):
            enable = wanted in ("on", "1", "true", "yes")
        else:
            return False, f"value must be on, off or toggle (got '{wanted}')"
        message = self._bridge.set_replay_mode(enable)
        ok = enable is False or bool(getattr(self._bridge, "replay_mode", False))
        return ok, message

    # ── service commands ─────────────────────────────────────────────────────

    def _service(self, args) -> Tuple[Optional[Any], str]:
        key = args.get("route") or args.get("service") or args.get("key") or ""
        if not key:
            return None, "no route= given"
        service = self._bridge.service_by_key(key)
        if service is None:
            return None, f"no service '{key}'"
        return service, ""

    def _cmd_service(self, args):
        """service route=<key> [role=serve|proxy|off] [enable=1|0]"""
        service, problem = self._service(args)
        if service is None:
            return False, problem
        acted = []
        if "role" in args:
            wanted = args["role"].lower()
            roles = {r.value: r for r in ServiceRole}
            if wanted not in roles:
                return False, f"role must be serve, proxy or off (got '{wanted}')"
            self._bridge.set_service_role(service.key, roles[wanted])
            acted.append(f"role {service.role.value}")
        if "enable" in args:
            self._bridge.set_service_enabled(service.key, _truthy(args["enable"]))
            acted.append("enabled" if service.enabled else "disabled")
        if not acted:
            self._bridge.toggle_service(service.key)
            acted.append("enabled" if service.enabled else "disabled")
        return True, f"{service.key}: {', '.join(acted)}"

    def _cmd_call(self, args):
        """call route=<key> [confirm=1] [arg.<field>=<value> ...]"""
        service, problem = self._service(args)
        if service is None:
            return False, problem
        if not service.available:
            return False, f"{service.key}: srv type unavailable in this workspace"
        if not service.enabled or service.role is ServiceRole.OFF:
            return False, (
                f"{service.key} is disabled; enable it first "
                "(SERVICES screen, or service route=... enable=1)"
            )
        if service.key in DANGEROUS_SERVICES and not _truthy(args.get("confirm")):
            return False, (
                f"{service.key} commands the aircraft -- repeat with confirm=1 "
                "if that is what you meant"
            )

        request = service.srv_type.Request()
        applied = self._fill_request(request, args)

        if service.role is ServiceRole.SERVE:
            response = service.srv_type.Response()
            response = service.handler(request, response, self._bridge)
            self._state.note_service(service.key, True, "ok (control)")
            return True, (
                f"{service.key} handled locally{applied}: {self._summarise(response)}"
            )

        # PROXY: the real Manifold owns this name. Fire and report on arrival.
        client = self._bridge.service_client(service.key)
        if client is None:
            return False, f"{service.key}: no client (role is proxy but none was built)"
        if not client.service_is_ready():
            return False, (
                f"{service.key}: no server on "
                f"{self._bridge.service_name(service)} -- is the Manifold running it?"
            )
        future = client.call_async(request)

        def arrived(done):
            try:
                response = done.result()
                self._state.note_service(service.key, True, "ok (proxy)")
                self._reply(
                    True,
                    f"{service.key} answered by the Manifold: {self._summarise(response)}",
                    f"call route={service.key}",
                )
            except Exception as exc:
                self._state.note_service(service.key, False, str(exc))
                self._reply(
                    False, f"{service.key} proxy call failed: {exc}",
                    f"call route={service.key}",
                )

        future.add_done_callback(arrived)
        return True, f"{service.key} forwarded to the Manifold{applied}, waiting"

    @staticmethod
    def _fill_request(request, args) -> str:
        """
        Apply `arg.<field>=<value>` pairs, coercing to the field's current type.
        Unknown or unconvertible fields are reported rather than ignored: a panel
        that silently drops a zoom factor is worse than one that says it cannot.
        """
        applied, problems = [], []
        for key, value in args.items():
            if not key.startswith("arg."):
                continue
            field = key[4:]
            if not hasattr(request, field):
                problems.append(f"no field '{field}'")
                continue
            current = getattr(request, field)
            try:
                if isinstance(current, bool):
                    setattr(request, field, _truthy(value))
                elif isinstance(current, int):
                    setattr(request, field, int(float(value)))
                elif isinstance(current, float):
                    setattr(request, field, float(value))
                elif isinstance(current, str):
                    setattr(request, field, value)
                else:
                    problems.append(f"'{field}' is not a scalar")
                    continue
                applied.append(f"{field}={value}")
            except (TypeError, ValueError) as exc:
                problems.append(f"{field}: {exc}")
        note = ""
        if applied:
            note += " (" + ", ".join(applied) + ")"
        if problems:
            note += " [ignored: " + "; ".join(problems) + "]"
        return note

    @staticmethod
    def _summarise(response) -> str:
        if response is None:
            return "no response"
        fields = getattr(response, "get_fields_and_field_types", None)
        names = list(fields().keys()) if callable(fields) else []
        if not names:
            return type(response).__name__
        parts = []
        for name in names[:6]:
            value = getattr(response, name, None)
            if isinstance(value, float):
                parts.append(f"{name}={value:g}")
            elif isinstance(value, (bool, int, str)):
                parts.append(f"{name}={value}")
        return " ".join(parts) or type(response).__name__

    # ── camera commands ──────────────────────────────────────────────────────

    def _cmd_camera(self, args):
        """camera [lens=wide|medium_tele|tele] [mode=photo|video_4k|video_fhd|preview] [zoom=<x>]"""
        acted = []
        if "lens" in args:
            lens = args["lens"].lower()
            if lens not in CAMERA_LENSES:
                return False, f"lens must be one of {', '.join(CAMERA_LENSES)}"
            self._bridge.publish_sim(CAMERA_SELECT_TOPIC, String(data=lens), String)
            self._camera["lens"] = lens
            acted.append(f"lens {lens}")
        if "mode" in args:
            mode = args["mode"].lower()
            if mode not in CAMERA_MODES:
                return False, f"mode must be one of {', '.join(CAMERA_MODES)}"
            self._bridge.publish_sim(CAMERA_MODE_TOPIC, String(data=mode), String)
            self._camera["mode"] = mode
            acted.append(f"mode {mode}")
        if "zoom" in args:
            try:
                zoom = float(args["zoom"])
            except ValueError:
                return False, f"zoom must be a number (got '{args['zoom']}')"
            zoom = min(168.0, max(1.0, zoom))
            self._bridge.publish_sim(CAMERA_ZOOM_TOPIC, Float64(data=zoom), Float64)
            self._camera["zoom"] = zoom
            acted.append(f"zoom {zoom:g}x")
        if not acted:
            return False, "nothing to do: give lens=, mode= or zoom="
        return True, "camera " + ", ".join(acted)

    # ── bridge-level commands ────────────────────────────────────────────────

    def _cmd_mock(self, args):
        """mock value=1|0"""
        control = getattr(self._bridge, "mock_control", None)
        if control is None:
            return False, "this session has no mock control"
        return True, control(_truthy(args.get("value", "1")))

    def _cmd_standin(self, args):
        """standin value=1|0"""
        control = getattr(self._bridge, "standin_control", None)
        if control is None:
            return False, "this session has no stand-in control"
        return True, control(_truthy(args.get("value", "1")))

    def _cmd_ssh(self, args):
        """ssh action=start|stop|probe|tail"""
        provisioner = getattr(self._bridge, "provisioner", None)
        if provisioner is None:
            return False, "this session has no provisioner"
        action = (args.get("action") or "probe").lower()
        if action not in ("start", "stop", "probe", "tail"):
            return False, "action must be start, stop, probe or tail"
        return True, provisioner.request_ssh(action)

    def _cmd_journal(self, args):
        """journal action=clear"""
        if (args.get("action") or "clear").lower() != "clear":
            return False, "action must be clear"
        self._journal.clear()
        return True, "conversion journal cleared"

    def _cmd_profile(self, args):
        """profile action=save"""
        if (args.get("action") or "save").lower() != "save":
            return False, "action must be save"
        return True, self._settings.save(getattr(self._bridge, "profile_path", "") or "")

    def _cmd_state(self, args):
        """state action=publish"""
        self._dirty = True
        self.publish(force=True)
        return True, "state published"

    def _cmd_help(self, args):
        """help"""
        lines = []
        for verb in self.verbs():
            doc = (getattr(self, f"_cmd_{verb}").__doc__ or "").strip()
            lines.append(doc.splitlines()[0] if doc else verb)
        return True, " | ".join(lines)

    # ── outbound state ───────────────────────────────────────────────────────

    def publish(self, force: bool = False) -> None:
        """
        Called from a bridge timer. Rate-limited, and skipped entirely when
        nobody is subscribed -- same reasoning as the viz layer, except the
        TRANSIENT_LOCAL snapshot means a late panel still gets one immediately.
        """
        now = time.monotonic()
        period = 1.0 / max(0.2, float(self._settings.control_state_hz))
        if not force and (now - self._last_state_at) < period:
            return
        if not force and not self._bridge.count_subscribers(STATE_TOPIC):
            self._last_state_at = now
            return
        self._last_state_at = now
        self._dirty = False
        self._state_pub.publish(String(data=self.snapshot()))

    def snapshot(self) -> str:
        bridge = self._bridge
        settings = self._settings
        report = bridge.report()
        stats = self._state.snapshot_stats()
        calls = self._state.service_calls()
        flight = self._state.flight()
        window = settings.journal_active_window_s
        active = self._journal.issues_by_route(window)
        subs, pubs = bridge.endpoint_counts()
        provisioner = getattr(bridge, "provisioner", None)

        lines = [f"V\t{STATE_SCHEMA}\t{time.time():.3f}"]
        lines.append(
            "L\t"
            + "\t".join(
                _clean(value)
                for value in (
                    report.manifold.value,
                    report.rmw,
                    report.domain,
                    report.multicast_port,
                    _flag(report.profiles_ok),
                    ", ".join(report.remote_nodes),
                    subs,
                    pubs,
                    flight.get("px4_clock", "-"),
                    provisioner.describe() if provisioner else "not configured",
                    report.summary(),
                )
            )
        )
        lines.append(
            "F\t"
            + "\t".join(
                _clean(value)
                for value in (
                    _flag(flight.get("armed")),
                    flight.get("nav_state", "-"),
                    "-" if flight.get("landed") is None else _flag(flight.get("landed")),
                    flight.get("battery", "-"),
                    flight.get("lat", "-"),
                    flight.get("lon", "-"),
                    flight.get("alt", "-"),
                    _flag(flight.get("rc_live")),
                    _flag(flight.get("rc_authority")),
                    _flag(flight.get("offboard_stream")),
                    # Appended, never inserted: the panels read this record by
                    # index, so a new field may only ever go on the end.
                    _flag(getattr(bridge, "project_mode", False)),
                )
            )
        )

        for route in bridge.routes:
            st = stats.get(route.key)
            issues = active.get(route.key, [])
            worst = max((i.level for i in issues), default=0)
            top = max(issues, key=lambda i: (i.level, i.count), default=None)
            lines.append(
                "R\t"
                + "\t".join(
                    _clean(value)
                    for value in (
                        route.key,
                        route.group,
                        route.host_topic,
                        route.sim_topic,
                        route.direction.value,
                        _flag(route.enabled),
                        _flag(route.available),
                        route.sub_qos,
                        route.pub_qos,
                        f"{route.max_hz:g}",
                        f"{st.hz:.2f}" if st else "0",
                        st.rx if st else 0,
                        st.tx if st else 0,
                        st.dropped_rate if st else 0,
                        st.dropped_error if st else 0,
                        LEVEL_NAMES.get(worst, "-") if worst else "-",
                        sum(i.count for i in issues),
                        top.code if top else "-",
                        route.notes,
                    )
                )
            )

        for service in bridge.service_routes:
            good, bad, last = calls.get(service.key, (0, 0, ""))
            lines.append(
                "S\t"
                + "\t".join(
                    _clean(value)
                    for value in (
                        service.key,
                        bridge.service_name(service),
                        service.role.value,
                        _flag(service.enabled),
                        _flag(service.available),
                        service.srv_type.__name__ if service.srv_type else "-",
                        good,
                        bad,
                        _flag(service.remote_server_seen),
                        _flag(service.key in DANGEROUS_SERVICES),
                        last,
                        service.notes,
                    )
                )
            )

        lines.append(
            "C\t"
            + "\t".join(
                _clean(value)
                for value in (
                    self._camera["lens"],
                    self._camera["mode"],
                    f"{float(self._camera['zoom']):g}",
                    CAMERA_ACTIVE_TOPIC,
                )
            )
        )
        for topic, encoding, description in CAMERA_FEEDS:
            lines.append(f"E\t{_clean(topic)}\t{encoding}\t{_clean(description)}")
        for preset in qos_mod.ORDER:
            lines.append(f"Q\t{preset}\t{_clean(qos_mod.describe(preset))}")
        for name in ("host->sim", "sim->host", "off"):
            lines.append(f"D\t{name}")
        return "\n".join(lines)

    # ── for the TUI ──────────────────────────────────────────────────────────

    @property
    def commands_seen(self) -> int:
        return self._commands_seen

    def camera_state(self) -> Dict[str, Any]:
        return dict(self._camera)


__all__ = [
    "CAMERA_FEEDS",
    "CAMERA_LENSES",
    "CAMERA_MODES",
    "CONTROL_TOPIC",
    "DANGEROUS_SERVICES",
    "RESULT_TOPIC",
    "STATE_TOPIC",
    "BridgeControl",
    "parse_command",
]
