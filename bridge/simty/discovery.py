"""
Graph discovery: what is on the wrapper surface, and is it only us?

WHAT THIS WAS, AND WHAT IT IS NOW
---------------------------------
In Version 1 this module answered "is the DJI Manifold actually there?" -- it
scanned for a remote participant over multicast SPDP, distinguished a real
Manifold from our own synthetic one, and explained why discovery had failed
when it did. Version 2 has no Manifold, no multicast and no remote anything, so
all of that is gone.

What remains is the half that was never really about the Manifold: a periodic
read of the DDS graph that answers three questions the bridge genuinely needs.

  1. IS THE SURFACE UP?  Which /wrapper/psdk_ros2 topics and services exist,
     against what the registry says should exist. A route that is enabled but
     whose endpoint never appeared is a bug you otherwise find by watching a
     C++ project wait forever.

  2. DOES ANYTHING ELSE PUBLISH ON IT?  See SurfaceState below. In V2 the
     honest answer is almost always "a bag replay", and two writers on one
     wrapper topic is the failure that is invisible from the inside: whichever
     message arrives last wins and nothing says so.

  3. ARE THE SIM-SIDE INPUTS THERE?  Which /fmu topics the converters depend on
     are actually publishing, so a dead link is named rather than inferred.

It owns no ROS resources of its own -- it only queries the node it is given --
so it is safe to call from the executor thread.
"""

import os
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple


class SurfaceState(Enum):
    """
    Who is publishing on /wrapper/psdk_ros2 right now.

    In Version 1 this enum tracked a DJI Manifold: SEARCHING until multicast
    discovery found one, CONNECTED when it did, MOCK when we were faking it.
    Version 2 has no Manifold and no multicast, but the question the enum
    answers is still the one that matters -- it is just a different question:

        QUIET    nothing publishes the wrapper surface yet. Normal before the
                 bridge finishes coming up; a problem if it persists.

        OURS     the surface exists and WE are the only publisher. This is the
                 healthy steady state.

        FOREIGN  something else is publishing on it too. In V2 that is almost
                 always a bag replay, and it is the condition worth shouting
                 about: two writers on one topic means whichever message
                 arrives last wins, with nothing anywhere saying so. The
                 constraint matrix (walkerd/constraints.py) exists to prevent
                 it; this is the check that catches it when prevention fails.
    """

    QUIET = "quiet"
    OURS = "ours"
    FOREIGN = "foreign"

    @property
    def label(self) -> str:
        return {
            "quiet": "QUIET",
            "ours": "OURS",
            "foreign": "FOREIGN (someone else is publishing)",
        }[self.value]


@dataclass
class Endpoint:
    topic: str
    types: List[str] = field(default_factory=list)
    publishers: int = 0
    subscribers: int = 0
    foreign_publishers: List[str] = field(default_factory=list)
    local_publishers: List[str] = field(default_factory=list)

    @property
    def is_foreign(self) -> bool:
        return bool(self.foreign_publishers)

    @property
    def type_name(self) -> str:
        return self.types[0] if self.types else "?"


@dataclass
class DiscoveryReport:
    scanned_at: float = 0.0
    scan_count: int = 0

    # middleware facts
    rmw: str = ""
    domain: str = ""
    profiles_file: str = ""
    profiles_ok: bool = False
    discovery_range: str = ""
    multicast_port: int = 0

    # graph
    wrapper_topics: List[Endpoint] = field(default_factory=list)
    wrapper_services: List[Tuple[str, List[str]]] = field(default_factory=list)
    fmu_topics: List[str] = field(default_factory=list)
    foreign_nodes: List[str] = field(default_factory=list)
    local_nodes: List[str] = field(default_factory=list)

    # registry cross-check
    expected_present: List[str] = field(default_factory=list)
    expected_missing: List[str] = field(default_factory=list)
    unmapped_wrapper: List[str] = field(default_factory=list)
    missing_sim_topics: List[str] = field(default_factory=list)
    conflicting_services: List[str] = field(default_factory=list)

    surface: SurfaceState = SurfaceState.QUIET
    first_connected_at: float = 0.0
    note: str = ""

    @property
    def foreign_publisher_count(self) -> int:
        return sum(len(e.foreign_publishers) for e in self.wrapper_topics)

    def summary(self) -> str:
        if self.surface is SurfaceState.FOREIGN:
            return (
                f"{len(self.wrapper_topics)} /wrapper topics, "
                f"{len(self.wrapper_services)} services, "
                f"CONTESTED by {len(self.foreign_nodes)} other publisher(s): "
                f"{', '.join(self.foreign_nodes[:3])}"
            )
        if self.surface is SurfaceState.OURS:
            return (f"{len(self.wrapper_topics)} /wrapper topics, "
                    f"{len(self.wrapper_services)} services, ours alone")
        return "no /wrapper endpoints in the DDS graph"


def middleware_facts(domain: Optional[int] = None) -> Dict[str, object]:
    """
    Read the middleware configuration actually in force, plus the multicast
    port the domain implies. Fast DDS/RTPS puts SPDP multicast discovery on
    7400 + 250*domain, which is the port to watch with tcpdump when discovery
    is not working.
    """
    domain_env = os.environ.get("ROS_DOMAIN_ID", "0")
    try:
        domain_id = int(domain if domain is not None else domain_env)
    except (TypeError, ValueError):
        domain_id = 0

    profiles = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    return {
        "rmw": os.environ.get("RMW_IMPLEMENTATION", "(default)"),
        "domain": str(domain_id),
        "profiles_file": profiles,
        "profiles_ok": bool(profiles) and os.path.exists(profiles),
        "discovery_range": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET (default)"),
        "multicast_port": 7400 + 250 * domain_id,
    }


class GraphScanner:
    """
    Periodic graph scanner. Owns no ROS resources of its own; it only queries
    the node it is given, so it is safe to call from the executor thread.
    """

    def __init__(self, node, settings, own_node_names: Set[str]):
        self._node = node
        self._settings = settings
        self._own = set(own_node_names)
        self._scan_count = 0
        self._first_connected_at = 0.0

    def register_own_node(self, name: str) -> None:
        """Tell the scanner about a node of ours (e.g. the mock Manifold)."""
        self._own.add(name)

    def unregister_own_node(self, name: str) -> None:
        self._own.discard(name)

    # -- helpers --

    def _endpoint(self, topic: str, types: List[str]) -> Endpoint:
        endpoint = Endpoint(topic=topic, types=list(types))
        try:
            infos = self._node.get_publishers_info_by_topic(topic)
        except Exception:
            infos = []
        for info in infos:
            name = getattr(info, "node_name", "") or "?"
            namespace = getattr(info, "node_namespace", "") or "/"
            full = f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            if name in self._own:
                endpoint.local_publishers.append(full)
            else:
                endpoint.foreign_publishers.append(full)
        endpoint.publishers = len(infos)
        try:
            endpoint.subscribers = self._node.count_subscribers(topic)
        except Exception:
            endpoint.subscribers = 0
        return endpoint

    # -- the scan --

    def scan(self, routes, services) -> DiscoveryReport:
        report = DiscoveryReport()
        self._scan_count += 1
        report.scanned_at = time.time()
        report.scan_count = self._scan_count

        facts = middleware_facts(self._settings.domain)
        report.rmw = str(facts["rmw"])
        report.domain = str(facts["domain"])
        report.profiles_file = str(facts["profiles_file"])
        report.profiles_ok = bool(facts["profiles_ok"])
        report.discovery_range = str(facts["discovery_range"])
        report.multicast_port = int(facts["multicast_port"])

        prefix = self._settings.wrapper_prefix.rstrip("/")

        try:
            all_topics = self._node.get_topic_names_and_types()
        except Exception as exc:
            report.note = f"graph query failed: {exc}"
            return report

        present_topics = {name for name, _ in all_topics}

        for name, types in sorted(all_topics):
            if name.startswith(prefix):
                report.wrapper_topics.append(self._endpoint(name, types))
            elif name.startswith("/fmu/"):
                report.fmu_topics.append(name)

        try:
            for name, types in sorted(self._node.get_service_names_and_types()):
                if name.startswith(prefix):
                    report.wrapper_services.append((name, list(types)))
        except Exception:
            pass

        # Node inventory, split by ownership. Keep the (name, namespace) pairs
        # for remote nodes: they are needed to ask each one what it serves.
        remote_pairs: List[Tuple[str, str]] = []
        try:
            for name, namespace in sorted(self._node.get_node_names_and_namespaces()):
                full = f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
                if name in self._own:
                    report.local_nodes.append(full)
                else:
                    report.foreign_nodes.append(full)
                    remote_pairs.append((name, namespace))
        except Exception:
            pass

        # -- cross-check against the route table --
        wrapper_by_name = {e.topic: e for e in report.wrapper_topics}
        expected: Set[str] = set()
        for route in routes:
            if not route.available:
                continue
            # Only /wrapper topics are expected to come from the Manifold. A few
            # routes have a sim-side "host" topic (e.g. land_detected publishes
            # /drone/landed); counting those as missing Manifold endpoints would
            # be a permanent false alarm.
            if route.host_topic.startswith(prefix):
                expected.add(route.host_topic)
            if route.sim_topic not in present_topics and route.sim_topic.startswith("/fmu/"):
                if route.sim_topic not in report.missing_sim_topics:
                    report.missing_sim_topics.append(route.sim_topic)

        for topic in sorted(expected):
            if topic in wrapper_by_name:
                report.expected_present.append(topic)
            else:
                report.expected_missing.append(topic)

        for topic in sorted(wrapper_by_name):
            if topic not in expected:
                report.unmapped_wrapper.append(topic)

        # A SERVE-role service whose name is already served by someone else is a
        # name collision: two servers on one name, and a client reaches whichever
        # one DDS happens to match.
        #
        # get_service_names_and_types() cannot say WHO serves a name -- and once
        # we serve it ourselves it appears there too, so that list cannot
        # distinguish "the Manifold owns this" from "we own this". Asking each
        # remote node what it serves can.
        served_remotely: Set[str] = set()
        for name, namespace in remote_pairs:
            try:
                for service_name, _types in self._node.get_service_names_and_types_by_node(
                    name, namespace
                ):
                    if service_name.startswith(prefix):
                        served_remotely.add(service_name)
            except Exception:
                # Node vanished between listing and querying, or the RMW does
                # not support the query. Not fatal: we simply cannot warn.
                continue

        for service in services:
            if not service.available or service.role.value != "serve":
                service.remote_server_seen = False
                continue
            full = f"{prefix}/{service.service}"
            service.remote_server_seen = full in served_remotely
            if service.remote_server_seen:
                report.conflicting_services.append(full)

        # -- classify --
        if not report.wrapper_topics:
            report.surface = SurfaceState.QUIET
        elif report.foreign_publisher_count > 0:
            report.surface = SurfaceState.FOREIGN
            if self._first_connected_at == 0.0:
                self._first_connected_at = report.scanned_at
        else:
            report.surface = SurfaceState.OURS
        report.first_connected_at = self._first_connected_at

        return report


def explain_failure(report: DiscoveryReport) -> List[str]:
    """
    Actionable checks for a SEARCHING state, ordered by how often each one is
    the actual cause.
    """
    hints = [
        "Is the Manifold's psdk_wrapper actually running, under namespace 'wrapper'? "
        "Check on the Manifold: ros2 topic list | grep wrapper",
        f"Do both sides use ROS_DOMAIN_ID={report.domain}? "
        f"(SPDP multicast then lands on UDP {report.multicast_port})",
        f"Do both sides use the same RMW? This side is {report.rmw}. "
        "Mixing Fast DDS and CycloneDDS breaks type hashes and services.",
        "Is the Manifold's podman container on the HOST network? A bridge "
        "network does not forward multicast.",
        "Same L2 subnet? Multicast does not route between subnets, and cannot "
        "be tunnelled over SSH.",
        f"Verify traffic: tcpdump -ni any udp port {report.multicast_port}",
        "Verify from the shell: ros2 topic list | grep wrapper",
    ]
    if not report.profiles_ok and report.profiles_file:
        hints.insert(
            0,
            f"FASTRTPS_DEFAULT_PROFILES_FILE points at {report.profiles_file}, "
            "which does not exist -- Fast DDS is running on defaults.",
        )
    return hints
