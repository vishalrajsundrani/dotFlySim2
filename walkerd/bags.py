"""
Recorded flights: finding them, describing them, and naming a new one.

A BAG IS A DIRECTORY, not a file: one `metadata.yaml` plus one or more `.mcap`
files. One level of nesting is allowed, so a folder of recordings can be copied
into `bags/` whole.

WHY metadata.yaml IS READ DIRECTLY
==================================
`ros2 bag info` costs 4-5 seconds per bag before it prints anything, which
makes a screen listing a dozen recordings unusable. Everything the screen needs
-- duration, message counts, the topic list with per-topic counts, the
serialisation format -- is already in `metadata.yaml` as plain text.

It is parsed with a small reader rather than PyYAML because walkerd should not
gain a dependency to read six scalars and a list whose shape rosbag2 fixes.
"""

from __future__ import annotations

import os
import re

BAGS = os.path.expanduser("~/bags")

# The five topics that let a bag re-fly the aircraft rather than only be
# watched. A bag holding none of them can still be replayed, but nothing moves.
COMMAND_TOPICS = (
    "flight_control_setpoint_ENUvelocity_yawrate",
    "flight_control_setpoint_ENUposition_yaw",
    "flight_control_setpoint_FLUvelocity_yawrate",
    "flight_control_setpoint_rollpitch_yawrate_thrust",
    "flight_control_setpoint_generic",
)

# A recording scope decides what the filter passes. Named here so walker's
# screen and walkerd's command cannot disagree about what "wrapper" means.
SCOPES = {
    "wrapper": "the wrapper surface: commands, telemetry and service calls",
    "wrapper-nocam": "the wrapper surface without the camera streams",
    "all": "every topic on the graph, including /fmu and raw Gazebo",
}

_NAME_OK = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


def valid_name(name: str) -> str:
    """Return "" if the name is usable, else why it is not."""
    name = (name or "").strip()
    if not name:
        return "a recording needs a name"
    if not _NAME_OK.match(name):
        return ("letters, digits, dot, dash and underscore only, starting with "
                "a letter or digit (max 64)")
    if os.path.exists(os.path.join(BAGS, name)):
        return f"bags/{name} already exists"
    return ""


# ── reading metadata.yaml ────────────────────────────────────────────────────


def _read_metadata(path: str) -> dict:
    """
    Pull what the screens need out of rosbag2's metadata.yaml.

    Deliberately tolerant: a bag recorded by a different rosbag2 version, or
    one still being written, should degrade to "fewer fields" rather than
    raise. A recording in progress has no metadata.yaml at all -- rosbag2
    writes it when it closes -- which is itself useful information.
    """
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    except OSError:
        return {}

    def num(pattern: str, default: float = 0.0) -> float:
        m = re.search(pattern, text)
        return float(m.group(1)) if m else default

    duration = num(r"duration:\s*\n\s*nanoseconds:\s*(\d+)") / 1e9
    started = num(r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)") / 1e9
    total = int(num(r"^\s*message_count:\s*(\d+)", 0))

    # Per-topic blocks: name, type, then a message_count for that topic.
    topics = []
    for block in re.finditer(
            r"-\s*topic_metadata:\s*\n(.*?)(?=\n\s*-\s*topic_metadata:|\Z)",
            text, re.S):
        body = block.group(1)
        name = re.search(r"name:\s*(\S+)", body)
        typ = re.search(r"type:\s*(\S+)", body)
        cnt = re.search(r"message_count:\s*(\d+)", body)
        if not name:
            continue
        n = int(cnt.group(1)) if cnt else 0
        topics.append({
            "name": name.group(1).strip("\"'"),
            "type": (typ.group(1).strip("\"'") if typ else "?"),
            "count": n,
            "hz": (n / duration) if duration > 0 else 0.0,
        })

    fmt = re.search(r"storage_identifier:\s*(\S+)", text)
    return {
        "duration_s": duration,
        "started_at": started,
        "messages": total or sum(t["count"] for t in topics),
        "topics": topics,
        "storage": fmt.group(1).strip("\"'") if fmt else "?",
    }


def _size_bytes(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def _summarise(directory: str, label: str) -> dict:
    meta = _read_metadata(os.path.join(directory, "metadata.yaml"))
    topics = meta.get("topics", [])
    names = [t["name"] for t in topics]

    # Service calls are recorded through a hidden <service>/_service_event
    # topic, which is why they look like topics here and are separated out.
    services = [n for n in names if n.endswith("/_service_event")]
    plain = [n for n in names if not n.endswith("/_service_event")]
    commands = [n for n in plain if any(n.endswith(c) for c in COMMAND_TOPICS)]
    cameras = [n for n in plain if "image" in n or "camera_stream" in n]

    return {
        "name": label,
        "path": directory,
        "duration_s": meta.get("duration_s", 0.0),
        "messages": meta.get("messages", 0),
        "size_bytes": _size_bytes(directory),
        "storage": meta.get("storage", "?"),
        "topics": topics,
        "n_topics": len(plain),
        "n_services": len(services),
        "n_commands": len(commands),
        "n_cameras": len(cameras),
        "commands": commands,
        "can_fly": bool(commands),
    }


def listing() -> list[dict]:
    """Every bag under bags/, one level of nesting allowed."""
    out = []
    if not os.path.isdir(BAGS):
        return out

    def consider(d: str, label: str) -> None:
        if os.path.isfile(os.path.join(d, "metadata.yaml")):
            out.append(_summarise(d, label))
        elif any(f.endswith(".mcap") for f in os.listdir(d)):
            # mcap present but no metadata: either still recording, or a
            # recorder that was killed rather than asked to stop. Both are
            # worth showing, because `ros2 bag play` will refuse it.
            out.append({"name": label, "path": d, "duration_s": 0.0,
                        "messages": 0, "size_bytes": _size_bytes(d),
                        "storage": "mcap", "topics": [], "n_topics": 0,
                        "n_services": 0, "n_commands": 0, "n_cameras": 0,
                        "commands": [], "can_fly": False,
                        "error": "no metadata.yaml - still recording, or the "
                                 "recorder was killed. `ros2 bag reindex` may "
                                 "rebuild it."})

    for entry in sorted(os.listdir(BAGS)):
        d = os.path.join(BAGS, entry)
        if not os.path.isdir(d):
            continue
        consider(d, entry)
        for sub in sorted(os.listdir(d)):
            s = os.path.join(d, sub)
            if os.path.isdir(s):
                consider(s, f"{entry}/{sub}")
    return out


def detail(name: str) -> dict:
    """Everything about one bag, for the detail screen."""
    for bag in listing():
        if bag["name"] == name:
            bag = dict(bag)
            bag["topics"] = sorted(bag["topics"],
                                   key=lambda t: (-t["count"], t["name"]))
            return bag
    return {}


# ── the commands ─────────────────────────────────────────────────────────────

WRAPPER = "/wrapper/psdk_ros2"


def record_argv(name: str, scope: str, services: list[str] | None = None) -> list[str]:
    """
    `ros2 bag record`, filtered by scope, writing straight into bags/.

    SERVICE CALLS ARE HALF OF WHAT MADE A FLIGHT HAPPEN. takeoff, land and
    obtain_ctrl_authority are not topics, and a bag without them can re-fly a
    sortie's setpoints but never the takeoff that started it. rosbag2 records a
    service through its <service>/_service_event topic, which is HIDDEN (a path
    segment starting with _), so --regex does not reach it and the names have
    to be passed explicitly.
    """
    out = os.path.join(BAGS, name)
    if scope == "all":
        filt = "--all"
    elif scope == "wrapper-nocam":
        filt = (f"--regex '^{WRAPPER}/' "
                f"--exclude-regex 'camera_stream|image_raw'")
    else:
        filt = f"--regex '^{WRAPPER}/'"

    svc = ""
    if scope != "all" and services:
        svc = " --services " + " ".join(services)

    return ["/bin/bash", "-c",
            "set +u; source /opt/ros/jazzy/setup.bash; "
            "source $HOME/ws/install/setup.bash 2>/dev/null; "
            f'echo "=== recording bags/{name} ({SCOPES.get(scope, scope)}) ==="; '
            f"exec ros2 bag record {filt}{svc} --storage mcap -o {out}"]


def play_argv(path: str, rate: float = 1.0, loop: bool = False) -> list[str]:
    """`ros2 bag play`, in its own terminal so the progress line is readable."""
    extra = f" --rate {rate}" if rate and rate != 1.0 else ""
    if loop:
        extra += " --loop"
    return ["/bin/bash", "-c",
            "set +u; source /opt/ros/jazzy/setup.bash; "
            "source $HOME/ws/install/setup.bash 2>/dev/null; "
            f'echo "=== replaying {path} ==="; '
            f"exec ros2 bag play {path}{extra} --clock"]
