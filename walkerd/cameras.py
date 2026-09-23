"""
The camera registry: every lens the current composition has, individually.

WHY THIS IS BUILT FROM THE COMPOSITION AND NOT A LIST
=====================================================
tools/compose_sim.py already walks the composed models and records every
sensor's name, type and Gazebo topic in compose.json. Reading that means the
camera screen is correct for whatever drone is loaded, including one added
tomorrow, with no list here to fall out of date.

GROUPS ARE A CONVENIENCE, NOT THE UNIT OF CONTROL
=================================================
"fisheye" and "payload" remain as one-key shortcuts because they are how people
talk about the cameras, but every lens is switchable on its own -- all three
payload lenses at each of their four resolutions, and each of the seven vision
cameras. A tele lens at preview resolution and a downward fisheye is a
perfectly reasonable thing to want, and no pair of group switches can express
it.
"""

from __future__ import annotations

import json
import os

COMPOSE = os.path.expanduser("~/gz_runtime/compose.json")

# Pretty names and ordering. A camera absent from here still appears -- it just
# sorts last and shows its raw sensor name, which is better than hiding a lens
# because this table was not updated.
_PAYLOAD_LENS = {"wide": "wide", "medium_tele": "medium tele", "tele": "tele"}
_TIERS = {"": "photo", "video4k": "4K", "videofhd": "FHD", "preview": "preview"}
_VISION = {
    "perception_front_left":  ("vision", "forward left",  "90° · 0.4-200 m"),
    "perception_front_right": ("vision", "forward right", "90° · 0.4-200 m"),
    "perception_back_left":   ("vision", "back left",     "90° · 0.4-200 m"),
    "perception_back_right":  ("vision", "back right",    "90° · 0.4-200 m"),
    "perception_left":        ("vision", "left",          "90° · 0.5-200 m"),
    "perception_right":       ("vision", "right",         "90° · 0.5-200 m"),
    "perception_down":        ("vision", "downward",      "160° · 0.3-18.8 m"),
}


def _load() -> dict:
    try:
        with open(COMPOSE, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def catalogue() -> list[dict]:
    """
    Every camera in the current composition, described for the screen.

    Each entry: sensor, topic, group, label, detail, resolution.
    """
    comp = _load()
    out: list[dict] = []
    for sensor, info in sorted(comp.get("sensor_topics", {}).items()):
        if info.get("type") != "camera":
            continue
        topic = info["topic"]
        entry = {"sensor": sensor, "topic": topic}

        if sensor in _VISION:
            group, label, detail = _VISION[sensor]
            entry.update(group=group, label=label, detail=detail, order=(1, sensor))
        elif sensor.startswith("m4e_") or "/camera/" in topic:
            # /drone/camera/<lens>/<tier>/image_raw, or .../<lens>/image_raw
            parts = topic.strip("/").split("/")
            lens = parts[2] if len(parts) > 2 else "?"
            tier = parts[3] if len(parts) > 4 else ""
            entry.update(
                group="payload",
                label=f"{_PAYLOAD_LENS.get(lens, lens)} · {_TIERS.get(tier, tier)}",
                detail={"": "full sensor, 1 Hz", "video4k": "3840x2160 @30",
                        "videofhd": "1920x1080 @30",
                        "preview": "640x360 @30"}.get(tier, ""),
                order=(0, lens, tier))
        else:
            entry.update(group="other", label=sensor, detail="", order=(2, sensor))
        out.append(entry)

    out.sort(key=lambda e: (e["order"][0], str(e["order"][1:])))
    for e in out:
        e.pop("order", None)
    return out


def topics_for_group(group: str) -> list[str]:
    return [c["topic"] for c in catalogue() if c["group"] == group]


def profile_topics(profile: str) -> list[str]:
    """
    The old four profiles, expressed as sets of individual cameras.

    `payload` means one lens at one tier, not all twelve: switching on every
    payload sensor at once costs about 1074 MP/s and is never what anybody
    wanted. The wide FHD stream is the sensible default and the one the wrapper
    surface carries.
    """
    cat = catalogue()
    if profile == "none":
        return []
    vision = [c["topic"] for c in cat if c["group"] == "vision"
              and c["sensor"] in ("perception_front_left", "perception_front_right")]
    payload = [c["topic"] for c in cat
               if c["topic"].endswith("/wide/videofhd/image_raw")]
    if profile == "fisheye":
        return vision
    if profile == "payload":
        return payload
    if profile == "all":
        return vision + payload
    return []
