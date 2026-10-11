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

# WHAT A DIRECTION COSTS IS NOT TWO CAMERAS.
#
# The six fisheyes are shared between five directions: fl is an eye of FRONT
# and also an eye of LEFT. So enabling LEFT when FRONT and REAR are already on
# costs nothing -- both of its lenses are already rendering. The screen says
# so, because otherwise the obvious reading of "5 directions x 2 cameras" is
# 10 cameras, and there are six.
CALIB = os.path.expanduser("~/ws/config/perception_calib.yaml")

_RANGE = {
    "FRONT": "0.4-22.5 m", "REAR": "0.4-22.5 m",
    "LEFT": "0.5-32 m", "RIGHT": "0.5-32 m",
    "DOWN": "0.3-18.8 m",
}


def _calib() -> dict:
    for path in (os.environ.get("PERCEPTION_CALIB", ""), CALIB):
        if path and os.path.isfile(path):
            try:
                import yaml
                with open(path, encoding="utf-8") as fh:
                    return yaml.safe_load(fh) or {}
            except Exception:
                return {}
    return {}


def _specs(drone: str) -> dict:
    """Sensor geometry from the model files, for labels and for cost.

    Read rather than written down: a lens that moves or changes resolution in
    model.sdf used to leave this screen quietly describing the old one.
    """
    try:
        import sdf_cameras
        import compose_sim
    except ImportError:
        return {}
    try:
        manifest = compose_sim.load_toml(os.path.join(MODELS, drone, "walker.toml"))
    except (SystemExit, OSError):
        return {}
    names = [drone] + [a["model"] for a in manifest.get("attachments", [])
                       if a.get("model")]
    out = {}
    for name in names:
        try:
            out.update(sdf_cameras.read(os.path.join(MODELS, name, "model.sdf")))
        except OSError:
            continue
    return out


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

    Labels and costs are DERIVED from the model, not written here. The old
    table said "90 deg" for the forward pair and "1920x1080 @30" for the FHD
    tier; both became wrong the moment the model changed, and a screen that
    confidently describes a lens the simulation does not have is worse than
    one that says nothing.
    """
    comp = _load()
    if drone and comp.get("drone") != drone:
        comp = _from_models(drone)
    elif not comp and drone:
        comp = _from_models(drone)
    specs = _specs(drone or comp.get("drone", ""))

    out: list[dict] = []
    for sensor, info in sorted(comp.get("sensor_topics", {}).items()):
        # "camera" is a rectilinear sensor, "wideanglecamera" a fisheye; the
        # perception lenses are the latter and were invisible here until they
        # were counted as cameras too.
        if info.get("type") not in ("camera", "wideanglecamera"):
            continue
        topic = info["topic"]
        spec = specs.get(sensor, {})
        entry = {"sensor": sensor, "topic": topic,
                 "mpix_s": round(spec.get("mpix_s", 0.0), 1)}

        if sensor.startswith("perception_"):
            entry.update(group="vision", label=sensor[len("perception_"):],
                         detail=_describe(spec), order=(1, sensor))
        elif sensor.startswith("m4e_") or "/camera/" in topic:
            # /drone/camera/<lens>/<tier>/image_raw, or .../<lens>/image_raw
            parts = topic.strip("/").split("/")
            lens = parts[2] if len(parts) > 2 else "?"
            tier = parts[3] if len(parts) > 4 else ""
            entry.update(
                group="payload",
                label=f"{_PAYLOAD_LENS.get(lens, lens)} \u00b7 {_TIERS.get(tier, tier)}",
                detail=_describe(spec), order=(0, lens, tier))
        else:
            entry.update(group="other", label=sensor, detail=_describe(spec),
                         order=(2, sensor))
        out.append(entry)

    out.sort(key=lambda e: (e["order"][0], str(e["order"][1:])))
    for e in out:
        e.pop("order", None)
    return out


def _describe(spec: dict) -> str:
    try:
        import sdf_cameras
    except ImportError:
        return ""
    return sdf_cameras.describe(spec) if spec else ""


def directions(drone: str = "", enabled: set | None = None) -> list[dict]:
    """
    The perception pairs, as directions rather than as six loose lenses.

    A direction is the unit an operator actually chooses: start_perception
    takes FRONT, not a pair of topics. Each entry carries both its lenses, what
    it costs to add GIVEN WHAT IS ALREADY ON, and whether it is ready to use.

    `marginal_mpix_s` is the honest number for the screen. Enabling LEFT after
    FRONT and REAR costs nothing, because fl and rl are already rendering, and
    a screen that charged for them twice would push people away from exactly
    the configuration that is cheapest.
    """
    enabled = enabled or set()
    cal = _calib()
    specs = _specs(drone)
    by_topic = {c["topic"]: c for c in catalogue(drone)}
    # sensor name -> topic, via the catalogue so a renamed topic follows
    topic_of = {c["sensor"]: c["topic"] for c in catalogue(drone)}

    out = []
    for name, d in (cal.get("directions") or {}).items():
        left, right = d.get("left", ""), d.get("right", "")
        topics = [topic_of.get(left, ""), topic_of.get(right, "")]
        present = all(topics)
        new_topics = [t for t in topics if t and t not in enabled]
        marginal = sum(specs.get(s, {}).get("mpix_s", 0.0)
                       for s, t in ((left, topics[0]), (right, topics[1]))
                       if t and t not in enabled)
        out.append({
            "direction": name,
            "left": left, "right": right,
            "topics": [t for t in topics if t],
            "baseline_m": d.get("baseline_m"),
            "range": _RANGE.get(name, ""),
            "detail": _describe(specs.get(left, {})),
            "on": present and all(t in enabled for t in topics),
            "available": present,
            "marginal_mpix_s": round(marginal, 1),
            "shares": sorted({s for s in (left, right)
                              if topic_of.get(s, "") in enabled}),
        })
    order = {"FRONT": 0, "DOWN": 1, "REAR": 2, "LEFT": 3, "RIGHT": 4}
    out.sort(key=lambda e: order.get(e["direction"], 9))
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
    # "fisheye" means the FORWARD pair, not all six: it is the direction a
    # localisation demo starts from, and six lenses is 60 MP/s for nothing if
    # only one direction is ever streamed.
    front = {d["direction"]: d for d in directions(drone)}.get("FRONT", {})
    vision = list(front.get("topics", []))
    payload = [c["topic"] for c in cat
               if c["topic"].endswith("/wide/videofhd/image_raw")]
    if profile == "fisheye":
        return vision
    if profile == "payload":
        return payload
    if profile == "all":
        return vision + payload
    return []
