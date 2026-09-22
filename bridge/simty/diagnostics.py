"""
The machine-readable half of the control room.

The TUI is for a person at a terminal and the markers are for a person in RViz.
Neither is any use to a script, a CI job, or `ros2 topic echo` over SSH -- so the
same state also goes out on two standard topics:

  /simty/diagnostics          diagnostic_msgs/DiagnosticArray, one status per
                              route, per service, plus the link and the clock.
                              Readable by rqt_robot_monitor and by anything that
                              already speaks diagnostics.
  /simty/conversion_events    std_msgs/String carrying one JSON object per
                              journal line. Deliberately a String and not a
                              custom type: a custom .msg would mean touching
                              psdk_interfaces, a colcon build and a Dockerfile
                              change, and this needs to work off a mounted file.

WHY DIAGNOSTIC LEVELS AND NOT JUST THE TEXT
-------------------------------------------
A route can be simultaneously healthy at the transport level and wrong at the
semantic level: samples arriving at 20 Hz while every one of them is being
clamped. Transport health comes from the counters, semantic health from the
conversion journal, and the published level is the worse of the two -- so a
monitor cannot show green on a bridge that is silently distorting commands.

Human-facing log lines are not duplicated here. node.py already mirrors WARN and
ERROR journal lines into the node logger, which puts them on /rosout for
rqt_console.
"""

import json
import time
from typing import List

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String

from .journal import describe
from .registry import Direction, ServiceRole
from .state import ERROR, LEVEL_NAMES, WARN

OK = DiagnosticStatus.OK
DIAG_WARN = DiagnosticStatus.WARN
DIAG_ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

DIAGNOSTICS_TOPIC = "/simty/diagnostics"
EVENTS_TOPIC = "/simty/conversion_events"


def _level_to_diag(level: int) -> int:
    if level >= ERROR:
        return DIAG_ERROR
    if level >= WARN:
        return DIAG_WARN
    return OK


class DiagnosticsPublisher:
    """
    Created by SimtyBridge. publish() runs from a bridge timer on the executor
    thread; like the viz layer it is subscriber-gated, so an unwatched bridge
    pays only for the two subscriber counts.
    """

    def __init__(self, bridge):
        self._bridge = bridge
        self._settings = bridge.settings
        self._state = bridge.state
        self._journal = bridge.journal
        self._last_event_seq = self._journal.event_seq

        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._diag_pub = bridge.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC, profile)
        self._event_pub = bridge.create_publisher(String, EVENTS_TOPIC, profile)

    # ── entry point ──────────────────────────────────────────────────────────

    def publish(self) -> None:
        if self._settings.publish_diagnostics and self._bridge.count_subscribers(DIAGNOSTICS_TOPIC):
            self._diag_pub.publish(self._array())
        if self._settings.publish_events:
            self._publish_events()
        else:
            # Stay caught up while muted, so re-enabling does not dump history.
            self._last_event_seq = self._journal.event_seq

    # ── diagnostics ──────────────────────────────────────────────────────────

    def _array(self) -> DiagnosticArray:
        array = DiagnosticArray()
        array.header.stamp = self._bridge.get_clock().now().to_msg()
        array.status.append(self._link_status())
        array.status.extend(self._route_statuses())
        array.status.extend(self._service_statuses())
        return array

    def _link_status(self) -> DiagnosticStatus:
        report = self._bridge.report()
        status = DiagnosticStatus()
        status.name = "simty: manifold link"
        status.hardware_id = report.rmw or "unknown-rmw"
        if report.manifold is ManifoldState.CONNECTED:
            status.level = OK
            status.message = f"connected -- {report.summary()}"
        elif report.manifold is ManifoldState.MOCK:
            status.level = DIAG_WARN
            status.message = (
                "synthetic or stand-in Manifold only; this proves nothing about "
                "the network"
            )
        else:
            status.level = DIAG_ERROR
            status.message = "no /wrapper endpoints in the DDS graph"

        subs, pubs = self._bridge.endpoint_counts()
        counts = self._journal.counts_by_level()
        provision = getattr(self._bridge, "provisioner", None)
        pairs = [
            ("manifold", report.manifold.label),
            ("rmw", report.rmw),
            ("domain", report.domain),
            ("spdp_multicast_port", str(report.multicast_port)),
            ("dds_profile", report.profiles_file or "(none)"),
            ("dds_profile_ok", str(report.profiles_ok)),
            ("remote_nodes", ", ".join(report.remote_nodes) or "-"),
            ("wrapper_topics", str(len(report.wrapper_topics))),
            ("wrapper_services", str(len(report.wrapper_services))),
            ("endpoints_sub", str(subs)),
            ("endpoints_pub", str(pubs)),
            ("px4_clock", str(self._state.flight().get("px4_clock", "-"))),
            ("journal_warnings", str(counts.get(WARN, 0))),
            ("journal_errors", str(counts.get(ERROR, 0))),
        ]
        if provision is not None:
            pairs.append(("provisioning", provision.describe()))
        status.values = [KeyValue(key=key, value=value) for key, value in pairs]
        return status

    def _route_statuses(self) -> List[DiagnosticStatus]:
        statuses = []
        stats_all = self._state.snapshot_stats()
        window = self._settings.journal_active_window_s
        active = self._journal.issues_by_route(window)
        for route in self._bridge.routes:
            stats = stats_all.get(route.key)
            status = DiagnosticStatus()
            status.name = f"simty: route/{route.key}"
            status.hardware_id = route.group

            if not route.available:
                level, message = DIAG_ERROR, "message type unavailable in this workspace"
            elif not route.enabled or route.direction is Direction.OFF:
                level, message = OK, "disabled by the operator"
            elif stats is None or stats.last_rx == 0.0:
                level, message = DIAG_WARN, "no sample has arrived yet"
            elif stats.age() > self._settings.stale_after_s:
                level, message = DIAG_WARN, f"stale: last sample {stats.age():.1f}s ago"
            else:
                level, message = OK, f"flowing at {stats.hz:.1f} Hz"

            # Semantic health from the journal, worse-of-the-two.
            issues = active.get(route.key, [])
            if issues:
                worst = max(issue.level for issue in issues)
                semantic = _level_to_diag(worst)
                if semantic > level:
                    level = semantic
                headline = max(issues, key=lambda i: (i.level, i.count))
                message = f"{message}; {headline.code} x{headline.count}"

            status.level = level
            status.message = message
            pairs = [
                ("direction", route.direction.value),
                ("source", route.source_topic),
                ("sink", route.sink_topic),
                ("qos_sub", route.sub_qos),
                ("qos_pub", route.pub_qos),
                ("rate_cap_hz", f"{route.max_hz:g}"),
                ("rx", str(stats.rx if stats else 0)),
                ("tx", str(stats.tx if stats else 0)),
                ("hz", f"{stats.hz:.2f}" if stats else "0"),
                ("dropped_rate_limited", str(stats.dropped_rate if stats else 0)),
                ("dropped_converter_error", str(stats.dropped_error if stats else 0)),
                ("latency_ms", f"{stats.latency_ms:.1f}" if stats and stats.latency_ms >= 0 else "n/a"),
                ("last_error", (stats.last_error if stats else "") or "-"),
            ]
            for issue in issues[:6]:
                pairs.append(
                    (
                        f"issue.{issue.code}",
                        f"{issue.count} since {time.strftime('%H:%M:%S', time.localtime(issue.first_at))}"
                        f" [{LEVEL_NAMES.get(issue.level, '?')}] {issue.detail or describe(issue.code)}",
                    )
                )
            status.values = [KeyValue(key=key, value=value) for key, value in pairs]
            statuses.append(status)
        return statuses

    def _service_statuses(self) -> List[DiagnosticStatus]:
        statuses = []
        calls = self._state.service_calls()
        for service in self._bridge.service_routes:
            good, bad, last = calls.get(service.key, (0, 0, ""))
            status = DiagnosticStatus()
            status.name = f"simty: service/{service.key}"
            status.hardware_id = service.group
            if not service.available:
                status.level, status.message = DIAG_ERROR, "srv type unavailable"
            elif service.remote_server_seen:
                status.level, status.message = (
                    DIAG_WARN,
                    "a remote server already owns this name -- switch to PROXY or disable",
                )
            elif not service.enabled or service.role is ServiceRole.OFF:
                status.level, status.message = OK, "disabled by the operator"
            elif bad:
                status.level, status.message = DIAG_ERROR, f"{bad} failed call(s): {last}"
            else:
                status.level, status.message = OK, f"{service.role.value}, {good} call(s)"
            status.values = [
                KeyValue(key="service", value=f"{self._settings.wrapper_prefix}/{service.service}"),
                KeyValue(key="role", value=service.role.value),
                KeyValue(key="type", value=service.srv_type.__name__ if service.srv_type else "-"),
                KeyValue(key="ok", value=str(good)),
                KeyValue(key="fail", value=str(bad)),
                KeyValue(key="last", value=last or "-"),
            ]
            statuses.append(status)
        return statuses

    # ── events ───────────────────────────────────────────────────────────────

    def _publish_events(self) -> None:
        events = self._journal.events_since(self._last_event_seq)
        if not events:
            return
        if not self._bridge.count_subscribers(EVENTS_TOPIC):
            self._last_event_seq = events[-1].seq
            return
        for event in events:
            payload = {
                "t": round(event.when, 3),
                "clock": event.clock,
                "route": event.route,
                "direction": event.direction,
                "code": event.code,
                "level": LEVEL_NAMES.get(event.level, str(event.level)),
                "detail": event.detail,
                "suppressed": event.suppressed,
                "meaning": describe(event.code),
            }
            self._event_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
            self._last_event_seq = event.seq


__all__ = ["DIAGNOSTICS_TOPIC", "EVENTS_TOPIC", "DiagnosticsPublisher"]
