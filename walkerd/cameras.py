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
import sys

COMPOSE = os.path.expanduser("~/gz_runtime/compose.json")
MODELS = os.path.expanduser("~/gz_models")

# tools/ holds the composer, whose SDF scanning this reuses rather than
# reimplementing. Two copies of "find the camera sensors in a model" would
# drift, and the screen would then offer a lens the simulation does not have.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "tools"))

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


def _from_models(drone: str) -> dict:
    """
    Read the camera sensors straight out of a drone's models.

    WHY THIS EXISTS ALONGSIDE compose.json
    ======================================
    compose.json is written when a simulation STARTS, so before the first run
    there is none -- and after a run with a different drone it describes the
    wrong aircraft. Either way the Cameras screen would be empty or wrong at
    exactly the moment somebody wants to choose what the next run will render.

    So when a drone is named, the catalogue comes from that drone's own model
    files, which exist as soon as the folder does. The composer's own scanner is
    reused, so what the screen lists and what the simulation will carry cannot
    disagree.
    """
    try:
        import compose_sim
    except ImportError:
        return {}
    try:
        manifest = compose_sim.load_toml(os.path.join(MODELS, drone, "walker.toml"))
    except SystemExit:
        return {}
    names = [drone] + [a["model"] for a in manifest.get("attachments", [])
                       if a.get("model")]
    topics: dict[str, dict] = {}
    for name in names:
        path = os.path.join(MODELS, name, "model.sdf")
        try:
            with open(path, encoding="utf-8") as fh:
                found = compose_sim.sensor_topics(fh.read())
        except OSError:
            continue
        for sensor, (kind, topic) in found.items():
            topics[sensor] = {"type": kind, "topic": topic}
    return {"sensor_topics": topics, "drone": drone}


def catalogue(drone: str = "") -> list[dict]:
    """
    Every camera the chosen drone carries, described for the screen.

    Prefers the live composition when it matches the drone asked about, and
    falls back to reading the model files -- so the screen works before the
    first simulation has ever been started.
    """
    comp = _load()
    if drone and comp.get("drone") != drone:
        comp = _from_models(drone)
    elif not comp and drone:
        comp = _from_models(drone)
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


def topics_for_group(group: str, drone: str = "") -> list[str]:
    return [c["topic"] for c in catalogue(drone) if c["group"] == group]


def profile_topics(profile: str, drone: str = "") -> list[str]:
    """
    The old four profiles, expressed as sets of individual cameras.

    `payload` means one lens at one tier, not all twelve: switching on every
    payload sensor at once costs about 1074 MP/s and is never what anybody
    wanted. The wide FHD stream is the sensible default and the one the wrapper
    surface carries.
    """
    cat = catalogue(drone)
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
