#!/usr/bin/env python3
"""
compose_sim.py — build the simulation that is about to run.

WHAT IT PRODUCES, AND WHY EACH PIECE EXISTS
===========================================
Given a drone, a world and a camera profile, this writes a complete, disposable
staging directory (default ~/gz_runtime) and installs it where PX4 and Gazebo
will find it:

  <out>/<drone>/model.sdf        every camera sensor set to always_on=false
  <out>/<attachment>/model.sdf   the same, for the gimbal/payload model
  <out>/worlds/<world>.sdf       the source world with drone-specific includes
                                 stripped and THIS drone's attachments injected
  <out>/airframe/<id>_gz_<drone> the PX4 airframe, generated from the manifest
  <out>/server.config            the gz system plugin list for this run
  <out>/bridge.yaml              ros_gz_bridge rows for the sensors that exist
  <out>/compose.json             a machine-readable record of all of the above

  ...plus symlinks into PX4's own model/world directories, and the airframe
  copied into PX4's runtime airframes directory.

THE SOURCES UNDER models/ AND worlds/ ARE NEVER MODIFIED. The staging directory
is placed FIRST on GZ_SIM_RESOURCE_PATH, so Gazebo resolves a name to the
composed copy. Deleting <out> is always safe.

FOUR THINGS HERE ARE LOAD-BEARING, EACH LEARNED THE HARD WAY
============================================================

1. always_on=false, NOT sensor removal.
   Version 1 deleted camera sensors from the SDF to make them cheap, which
   meant the camera profile could only change at start-up. Measured (SPIKE-1):
   with always_on=false and nothing subscribed, 12 payload cameras cost 18.9%
   CPU against a 17.1% floor — i.e. essentially nothing — and a subscriber
   brings one back to life in under a second. So the sensors always stay, and
   the profile is purely a question of who subscribes. That is what makes
   cameras switchable DURING a flight.

2. PX4 spawns models by ABSOLUTE PATH.
   px4-rc.gzsim builds <include><uri> from ${PX4_GZ_MODELS}/<name>/model.sdf
   and does NOT consult GZ_SIM_RESOURCE_PATH. Version 1 satisfied this with a
   build-time symlink, which is precisely why its aircraft was a constant.
   We symlink at run time instead. (Finding C1.)

3. GstCameraSystem is an always-subscriber.
   It subscribes to camera topics to read camera_info and never unsubscribes,
   which pins those sensors awake and defeats point 1 — measured as the gap
   between 77.8% and 18.9%. So the server config is generated per run and that
   plugin is included only when QGC video is actually wanted. (Finding C2.)

4. Text editing, not an XML library.
   Round-tripping these files through ElementTree drops every comment and
   reorders attributes, and models/m4e/model.sdf is more comment than XML --
   the note about gz-sensors issue #325 next to the navsat noise is
   load-bearing documentation. Sensor blocks are therefore patched textually
   and the rest of the file is copied through byte for byte.

USAGE
    compose_sim.py --drone m4e --world powerline --cameras fisheye
    compose_sim.py --drone m4e --world powerline --cameras all --qgc-video
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import tomllib

HOME = os.path.expanduser("~")
DEFAULT_MODELS = os.path.join(HOME, "gz_models")
DEFAULT_WORLDS = os.path.join(HOME, "gz_worlds")
DEFAULT_OUT = os.path.join(HOME, "gz_runtime")
PX4 = os.path.join(HOME, "PX4-Autopilot")
PX4_GZ_MODELS = os.path.join(PX4, "Tools/simulation/gz/models")
PX4_GZ_WORLDS = os.path.join(PX4, "Tools/simulation/gz/worlds")
PX4_AIRFRAMES = os.path.join(PX4, "build/px4_sitl_default/etc/init.d-posix/airframes")
PX4_SERVER_CONFIG = os.path.join(PX4, "src/modules/simulation/gz_bridge/server.config")

CAMERA_PROFILES = ("none", "fisheye", "payload", "all")


# ── manifests ────────────────────────────────────────────────────────────────


def load_toml(path: str) -> dict:
    """
    Read a manifest, or fail with a message a person can act on.

    An unhandled TOMLDecodeError here prints a traceback whose most prominent
    line is a frame in this file -- which reads as "compose_sim.py is broken"
    rather than "your manifest has a typo on line 3". The file and the parser's
    own complaint are what matter, so that is what gets printed.
    """
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "rb") as fh:
            return tomllib.load(fh)
    except tomllib.TOMLDecodeError as e:
        raise SystemExit(f"compose: {path} is not valid TOML.\n  {e}")
    except OSError as e:
        raise SystemExit(f"compose: cannot read {path}: {e}")


def drone_manifest(models_dir: str, name: str) -> dict:
    d = load_toml(os.path.join(models_dir, name, "walker.toml"))
    if d.get("kind") != "drone":
        raise SystemExit(
            f"compose: '{name}' is not a flyable drone.\n"
            f"  models/{name}/walker.toml must exist and say kind = \"drone\".\n"
            f"  A model without one is scenery: a world can include it, but it "
            f"cannot be selected as the aircraft.")
    return d


def world_manifest(worlds_dir: str, name: str) -> tuple[str, dict]:
    sdf = os.path.join(worlds_dir, f"{name}.sdf")
    if not os.path.isfile(sdf):
        raise SystemExit(f"compose: no such world: {sdf}")
    man = load_toml(os.path.join(worlds_dir, f"{name}.walker.toml"))
    if "gz_world_name" not in man:
        # Infer it, because getting this wrong yields a simulation that starts
        # and never becomes ready -- every gz service path is built from it.
        with open(sdf, encoding="utf-8") as fh:
            m = re.search(r'<world\s+name=["\']([^"\']+)["\']', fh.read())
        man["gz_world_name"] = m.group(1) if m else "default"
    return sdf, man


# ── SDF sensor surgery ───────────────────────────────────────────────────────


def sensor_blocks(text: str):
    """Yield (start, end, name, type) for every <sensor>...</sensor>."""
    for m in re.finditer(r'<sensor\b[^>]*>', text):
        tag = m.group(0)
        name = re.search(r'name=["\']([^"\']+)["\']', tag)
        typ = re.search(r'type=["\']([^"\']+)["\']', tag)
        close = text.find("</sensor>", m.end())
        if close == -1:
            continue
        yield m.start(), close + len("</sensor>"), (name.group(1) if name else ""), \
              (typ.group(1) if typ else "")


def sensor_topics(text: str) -> dict[str, tuple[str, str]]:
    """{sensor name: (type, gz topic)} for every sensor that declares a <topic>."""
    out = {}
    for s, e, name, typ in sensor_blocks(text):
        t = re.search(r"<topic>([^<]+)</topic>", text[s:e])
        if name and t:
            out[name] = (typ, t.group(1).strip())
    return out


def relax_cameras(text: str) -> tuple[str, list[str]]:
    """
    Set <always_on>false</always_on> on every CAMERA sensor. See note 1 above.

    Depth/lidar/IMU/baro/mag/navsat are left alone: PX4's estimator needs them
    continuously, and they are cheap.
    """
    pieces, touched, last = [], [], 0
    for s, e, name, typ in sensor_blocks(text):
        block = text[s:e]
        if typ == "camera" and "<always_on>true</always_on>" in block:
            block = block.replace("<always_on>true</always_on>",
                                  "<always_on>false</always_on>", 1)
            touched.append(name)
        pieces.append(text[last:s])
        pieces.append(block)
        last = e
    pieces.append(text[last:])
    return "".join(pieces), touched


def stage_model(models_dir: str, out: str, name: str, transform) -> list[str]:
    """
    Copy one model into the staging directory, transforming model.sdf.

    Everything except model.sdf is SYMLINKED, so the meshes (tens of megabytes
    of .dae) are not duplicated on every start.
    """
    src, dst = os.path.join(models_dir, name), os.path.join(out, name)
    if not os.path.isdir(src):
        raise SystemExit(f"compose: model '{name}' not found under {models_dir}")
    os.makedirs(dst, exist_ok=True)
    for entry in os.listdir(src):
        if entry == "model.sdf":
            continue
        link = os.path.join(dst, entry)
        if not os.path.lexists(link):
            os.symlink(os.path.join(src, entry), link)
    with open(os.path.join(src, "model.sdf"), encoding="utf-8") as fh:
        text = fh.read()
    text, touched = transform(text)
    with open(os.path.join(dst, "model.sdf"), "w", encoding="utf-8") as fh:
        fh.write(text)
    return touched


# ── the world ────────────────────────────────────────────────────────────────


def compose_world(sdf_path: str, out_dir: str, world_name: str,
                  drop_models: set[str], attachments: list[dict]) -> str:
    """
    Copy the world, remove includes of drone-specific models, inject this
    drone's attachments.

    WHY THIS IS NECESSARY. In V1, worlds/powerline.sdf contained
    `<include><uri>model://m4e_camera</uri></include>` -- the world knew which
    aircraft was going to fly in it. That makes "any drone in any world"
    impossible, so the include is stripped here and re-injected from the
    DRONE's manifest instead. A world that never mentioned a drone is
    unaffected.
    """
    with open(sdf_path, encoding="utf-8") as fh:
        text = fh.read()

    removed = []
    for m in reversed(list(re.finditer(r"[ \t]*<include>.*?</include>\s*", text, re.S))):
        uri = re.search(r"<uri>\s*model://([^<\s]+)\s*</uri>", m.group(0))
        if uri and uri.group(1) in drop_models:
            removed.append(uri.group(1))
            text = text[:m.start()] + text[m.end():]

    inject = []
    for att in attachments:
        pose = att.get("pose", [0, 0, 0, 0, 0, 0])
        inject.append(
            "    <!-- injected by compose_sim.py from the drone's walker.toml -->\n"
            "    <include>\n"
            f"      <uri>model://{att['model']}</uri>\n"
            f"      <name>{att['model']}</name>\n"
            f"      <pose>{' '.join(str(float(v)) for v in pose)}</pose>\n"
            "    </include>\n")
    if inject:
        text = text.replace("</world>", "".join(inject) + "</world>", 1)

    os.makedirs(out_dir, exist_ok=True)
    dst = os.path.join(out_dir, os.path.basename(sdf_path))
    with open(dst, "w", encoding="utf-8") as fh:
        fh.write(text)
    return dst


# ── the airframe ─────────────────────────────────────────────────────────────


def write_airframe(out: str, drone: str, man: dict) -> tuple[str, int]:
    """
    Generate the PX4 airframe file. SPIKE-2 established that PX4 finds this by
    scanning its runtime airframes directory at boot, so no rebuild is needed.
    """
    af = man.get("airframe", {})
    aid = int(af.get("id", 4900))
    base = af.get("base", "rc.mc_defaults")
    params = af.get("params", {})

    lines = [
        "#!/bin/sh",
        "#",
        f"# @name Gazebo {man.get('title', drone)}",
        f"# @type {af.get('type', 'Quadrotor').title()}",
        "#",
        "# GENERATED by tools/compose_sim.py from models/%s/walker.toml." % drone,
        "# Do not edit here: this file is rewritten on every simulation start.",
        "#",
        f". ${{R}}etc/init.d/{base}",
        "",
        "PX4_SIMULATOR=${PX4_SIMULATOR:=gz}",
        "PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}",
        f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={drone}}}",
        "",
    ]
    for k, v in params.items():
        if isinstance(v, bool):
            v = int(v)
        lines.append(f"param set-default {k} {v}")
    lines.append("")

    d = os.path.join(out, "airframe")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, f"{aid}_gz_{drone}")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    os.chmod(path, 0o755)

    # The .post fragment. px4-rc.mavlink binds the GCS link to 18570 and relies
    # on UDP broadcast for QGC to find PX4; Linux loopback does not forward
    # broadcast, so QGC on the same machine never hears it. This second link
    # targets 127.0.0.1:14550 explicitly.
    #
    # NOTE it targets the MAVLINK GUARD's port, not QGC's, when one is in use --
    # walkerd rewrites this line when it starts the guard (§16.2).
    with open(path + ".post", "w", encoding="utf-8") as fh:
        fh.write("\nmavlink start -x -u 14541 -r 4000000 -t 127.0.0.1 -o 14550 -f\n")
    return path, aid


# ── the gz server config (Finding C2) ────────────────────────────────────────


def write_server_config(out: str, qgc_video: bool) -> tuple[str, bool]:
    """
    Copy PX4's server.config, dropping GstCameraSystem unless QGC video is
    wanted.

    That plugin subscribes to camera topics for their camera_info and never
    unsubscribes, which keeps those sensors rendering even when walker has
    switched the group off -- measured as 77.8% CPU instead of 18.9%. Loading
    it only when its output is actually wanted is a config change, not a C++
    change.
    """
    with open(PX4_SERVER_CONFIG, encoding="utf-8") as fh:
        text = fh.read()
    dropped = False
    if not qgc_video:
        text, n = re.subn(r"<plugin[^>]*libGstCameraSystem\.so.*?</plugin>\s*",
                          "", text, flags=re.S)
        dropped = n > 0
    path = os.path.join(out, "server.config")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)
    return path, dropped


# ── the ros_gz bridge config ─────────────────────────────────────────────────

_IMG = ("sensor_msgs/msg/Image", "gz.msgs.Image")
_TYPES = {
    "camera": _IMG,
    "gpu_lidar": ("sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
    "boundingbox_camera": _IMG,
}


def write_bridge_yaml(out: str, topics: dict[str, tuple[str, str]]) -> tuple[str, int]:
    """
    Generate ros_gz_bridge rows for exactly the sensors this composition has.

    Every camera row is lazy:true -- ros_gz_bridge then creates its gz-transport
    subscription only when a ROS subscriber appears, which is the second half of
    the demand-driven chain (the first half is always_on=false in the SDF).
    """
    rows = [
        "# GENERATED by tools/compose_sim.py. Do not edit; rewritten every start.",
        "#",
        "# lazy:true on every camera is load-bearing: together with",
        "# always_on=false in the model SDF it is what makes a switched-off",
        "# camera cost nothing. See SPIKE-1 in VERSION2_PLAN.md.",
        "",
        "# Simulation clock.",
        '- ros_topic_name: "/clock"',
        '  gz_topic_name: "/clock"',
        '  ros_type_name: "rosgraph_msgs/msg/Clock"',
        '  gz_type_name: "gz.msgs.Clock"',
        "  direction: GZ_TO_ROS",
        "",
    ]
    n = 0
    for name, (typ, topic) in sorted(topics.items(), key=lambda kv: kv[1][1]):
        if typ not in _TYPES:
            continue
        ros_t, gz_t = _TYPES[typ]
        lazy = typ != "gpu_lidar"   # the LRF is not lazy: PX4's EKF needs range
        rows += [
            f"# sensor: {name}",
            f'- ros_topic_name: "{topic}"',
            f'  gz_topic_name: "{topic}"',
            f'  ros_type_name: "{ros_t}"',
            f'  gz_type_name: "{gz_t}"',
            "  direction: GZ_TO_ROS",
        ]
        if lazy:
            rows.append("  lazy: true")
        rows.append("")
        n += 1
        if typ == "camera":
            rows += [
                f'- ros_topic_name: "{topic.rsplit("/", 1)[0]}/camera_info"',
                f'  gz_topic_name: "{topic}/camera_info"',
                '  ros_type_name: "sensor_msgs/msg/CameraInfo"',
                '  gz_type_name: "gz.msgs.CameraInfo"',
                "  direction: GZ_TO_ROS",
                "  lazy: true",
                "",
            ]

    # Drives GstCameraSystem's selective mode: whichever gz camera topic is
    # published here becomes the single stream QGC receives on UDP 5600.
    rows += [
        "# QGC video lens selection.",
        '- ros_topic_name: "/drone/camera/gst_select"',
        '  gz_topic_name: "/drone/camera/gst_select"',
        '  ros_type_name: "std_msgs/msg/String"',
        '  gz_type_name: "gz.msgs.StringMsg"',
        "  direction: ROS_TO_GZ",
        "",
    ]
    path = os.path.join(out, "bridge.yaml")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(rows))
    return path, n


# ── installing into PX4's absolute paths (Finding C1) ────────────────────────


def link(src: str, dst: str) -> None:
    if os.path.lexists(dst):
        if os.path.islink(dst):
            os.unlink(dst)
        else:
            shutil.rmtree(dst) if os.path.isdir(dst) else os.unlink(dst)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    os.symlink(src, dst)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Build the simulation about to run.")
    p.add_argument("--drone", required=True)
    p.add_argument("--world", required=True)
    p.add_argument("--cameras", default="none", choices=CAMERA_PROFILES,
                   help="which camera groups stream at start-up (all sensors "
                        "are present either way; this only sets who subscribes)")
    p.add_argument("--qgc-video", action="store_true",
                   help="load GstCameraSystem so QGC can receive a stream")
    p.add_argument("--models", default=DEFAULT_MODELS)
    p.add_argument("--worlds", default=DEFAULT_WORLDS)
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--no-install", action="store_true",
                   help="compose only; do not touch PX4's directories")
    a = p.parse_args(argv)

    dman = drone_manifest(a.models, a.drone)
    wsdf, wman = world_manifest(a.worlds, a.world)

    # A fresh staging directory every time: leftovers from a previous, different
    # composition are exactly the kind of thing that produces a simulation that
    # is subtly not what you asked for.
    if os.path.isdir(a.out):
        shutil.rmtree(a.out)
    os.makedirs(a.out, exist_ok=True)

    attachments = dman.get("attachments", [])
    staged, relaxed = [], []
    for name in [a.drone] + [x["model"] for x in attachments]:
        touched = stage_model(a.models, a.out, name, relax_cameras)
        staged.append(name)
        relaxed += [f"{name}:{t}" for t in touched]

    # Scenery: symlinked through so a world's includes still resolve against
    # the staging directory, which is first on GZ_SIM_RESOURCE_PATH.
    for entry in sorted(os.listdir(a.models)):
        if entry in staged:
            continue
        dst = os.path.join(a.out, entry)
        if not os.path.lexists(dst):
            os.symlink(os.path.join(a.models, entry), dst)

    # Any model that is a drone or an attachment must not be left in the world
    # by its own include -- this drone's attachments are injected instead.
    drop = set()
    for entry in os.listdir(a.models):
        k = load_toml(os.path.join(a.models, entry, "walker.toml")).get("kind")
        if k in ("drone", "attachment"):
            drop.add(entry)
    world_out = compose_world(wsdf, os.path.join(a.out, "worlds"),
                              wman["gz_world_name"], drop, attachments)

    topics: dict[str, tuple[str, str]] = {}
    for name in staged:
        with open(os.path.join(a.out, name, "model.sdf"), encoding="utf-8") as fh:
            topics.update(sensor_topics(fh.read()))

    af_path, af_id = write_airframe(a.out, a.drone, dman)
    sc_path, gst_dropped = write_server_config(a.out, a.qgc_video)
    br_path, n_sensors = write_bridge_yaml(a.out, topics)

    installed = []
    if not a.no_install:
        # THE SYMLINKS THAT MAKE DRONE SELECTION WORK AT ALL (Finding C1).
        for name in staged:
            link(os.path.join(a.out, name), os.path.join(PX4_GZ_MODELS, name))
            installed.append(os.path.join(PX4_GZ_MODELS, name))
        link(world_out, os.path.join(PX4_GZ_WORLDS, os.path.basename(world_out)))
        installed.append(os.path.join(PX4_GZ_WORLDS, os.path.basename(world_out)))
        # The airframe is COPIED, not linked: PX4 execs it as a shell script
        # and a dangling link here is a boot failure with an obscure message.
        os.makedirs(PX4_AIRFRAMES, exist_ok=True)
        for suffix in ("", ".post"):
            shutil.copy2(af_path + suffix,
                         os.path.join(PX4_AIRFRAMES, os.path.basename(af_path) + suffix))
        os.chmod(os.path.join(PX4_AIRFRAMES, os.path.basename(af_path)), 0o755)
        installed.append(os.path.join(PX4_AIRFRAMES, os.path.basename(af_path)))

    record = {
        "drone": a.drone, "world": a.world,
        "gz_world_name": wman["gz_world_name"],
        "world_sdf": world_out,
        "px4_sim_model": f"gz_{a.drone}",
        "airframe_id": af_id,
        "spawn": wman.get("spawn", {}),
        "cameras_profile": a.cameras,
        "qgc_video": bool(a.qgc_video),
        "gst_camera_system": not gst_dropped,
        "staged_models": staged,
        "cameras_relaxed": relaxed,
        "sensor_topics": {k: {"type": v[0], "topic": v[1]} for k, v in topics.items()},
        "server_config": sc_path,
        "bridge_yaml": br_path,
        "resource_path": [a.out, a.models],
        "installed": installed,
    }
    with open(os.path.join(a.out, "compose.json"), "w", encoding="utf-8") as fh:
        json.dump(record, fh, indent=2)

    print(f"composed {a.drone} in {a.world}  ->  {a.out}")
    print(f"  models staged     : {', '.join(staged)}")
    print(f"  cameras relaxed   : {len(relaxed)} sensor(s) set always_on=false")
    print(f"  camera profile    : {a.cameras} (subscription-driven; all sensors present)")
    print(f"  airframe          : {af_id}_gz_{a.drone}  ({len(dman.get('airframe', {}).get('params', {}))} params)")
    print(f"  world             : {world_out}  (gz world name: {wman['gz_world_name']})")
    print(f"  GstCameraSystem   : {'loaded (QGC video)' if not gst_dropped else 'NOT loaded (keeps switched-off cameras free)'}")
    print(f"  bridge rows       : {n_sensors} sensor(s)")
    if installed:
        print(f"  installed         : {len(installed)} path(s) into PX4's tree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
