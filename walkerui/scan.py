"""
Scanning the repository for the things an operator can choose.

WHAT MAKES THIS DIFFERENT FROM VERSION 1
========================================
V1 had no scan. The world was `worlds/powerline.sdf` because a shell script
said so, and the drone was `m4e` because the Dockerfile had baked its airframe
in. Adding either meant editing files and rebuilding an image.

Here, everything selectable is a directory on disk that is also a mount, so
"add a world" is "put a .sdf in worlds/ and press F5". That only works if
scanning is cheap enough to do on every keystroke, and honest enough to show
you the thing you just added even when its manifest is wrong.

CHEAP
-----
`os.scandir` on the host, with an mtime-keyed cache. The whole repository
scans in single-digit milliseconds, so walker can rescan on demand without a
spinner. Nothing here talks to docker or to ROS: a scan works before the
container is even up, which is exactly when you are choosing what to run.

HONEST
------
A model with a broken `walker.toml` is LISTED, with the parse error as its
subtitle -- not silently dropped. A world whose scenery is missing is listed
and marked. Hiding a broken thing is how you end up staring at a menu
wondering why the drone you just added is not in it.
"""

from __future__ import annotations

import os
import re
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from . import paths

CAMERA_PROFILES = ("none", "fisheye", "payload", "all")


@dataclass
class Entry:
    """Something selectable, or something that wanted to be."""
    name: str
    path: Path
    kind: str                     # drone | attachment | scenery | world | bag | project
    title: str = ""
    detail: str = ""
    error: str = ""               # non-empty => listed but not selectable
    meta: dict = field(default_factory=dict)

    @property
    def usable(self) -> bool:
        return not self.error


def _load_toml(path: Path) -> tuple[dict, str]:
    if not path.is_file():
        return {}, ""
    try:
        with path.open("rb") as fh:
            return tomllib.load(fh), ""
    except (tomllib.TOMLDecodeError, OSError) as e:
        # Reported, not swallowed: a typo in a manifest must be visible in the
        # menu where you would look for the thing it describes.
        return {}, f"{path.name}: {e}"


# ── models ───────────────────────────────────────────────────────────────────


def models() -> list[Entry]:
    """Every model directory, classified. Drones first, then attachments, then scenery."""
    out: list[Entry] = []
    root = paths.REPO / "models"
    if not root.is_dir():
        return out
    for d in sorted(root.iterdir(), key=lambda p: p.name):
        if not d.is_dir():
            continue
        sdf = d / "model.sdf"
        if not sdf.is_file():
            continue                       # not a Gazebo model at all
        man, err = _load_toml(d / "walker.toml")
        # A manifest that FAILED TO PARSE tells us nothing -- including whether
        # this was meant to be a drone. Defaulting to "scenery" there is how a
        # drone with a typo in its manifest disappears from the Drones screen
        # entirely, leaving you to wonder why the thing you just added is not
        # listed. So a broken manifest is classified "broken", and the Drones
        # screen shows it: something that tried to declare itself and failed is
        # far more likely to be the drone you are looking for than scenery.
        has_manifest = (d / "walker.toml").is_file()
        if err:
            kind = "broken"
        else:
            kind = man.get("kind", "scenery")
        e = Entry(name=d.name, path=d, kind=kind,
                  title=man.get("title", d.name), meta=man, error=err)

        if kind == "broken":
            e.detail = "manifest could not be read"
        elif kind == "drone":
            af = man.get("airframe", {})
            cams = _count_cameras(man)
            if "id" not in af:
                e.error = e.error or "walker.toml has no [airframe].id"
            e.detail = (f"airframe {af.get('id','?')} · "
                        f"{af.get('type','?')} · "
                        f"{cams} camera group(s)")
        elif kind == "attachment":
            e.detail = "payload/gimbal, spawned with its drone"
        else:
            e.detail = "scenery"
        out.append(e)

    order = {"broken": 0, "drone": 0, "attachment": 1, "scenery": 2}
    out.sort(key=lambda e: (order.get(e.kind, 3), e.name))
    return out


def _count_cameras(man: dict) -> int:
    cams = man.get("cameras", {})
    n = 0
    if "fisheye" in cams:
        n += 1
    lenses = cams.get("payload", {}).get("lenses", {})
    n += len(lenses)
    return n


def drones() -> list[Entry]:
    """Selectable aircraft, plus anything whose manifest is broken enough that
    we cannot rule it out -- listed, not selectable, with the reason."""
    return [e for e in models() if e.kind in ("drone", "broken")]


# ── worlds ───────────────────────────────────────────────────────────────────


def worlds() -> list[Entry]:
    """
    Every .sdf in worlds/. A manifest is optional -- a bare .sdf is selectable,
    with its gz world name read out of the file.
    """
    out: list[Entry] = []
    root = paths.REPO / "worlds"
    if not root.is_dir():
        return out
    known = {m.name for m in models()}
    for sdf in sorted(root.glob("*.sdf")):
        name = sdf.stem
        man, err = _load_toml(root / f"{name}.walker.toml")
        e = Entry(name=name, path=sdf, kind="world",
                  title=man.get("title", name), meta=man, error=err)

        gz_name = man.get("gz_world_name")
        if not gz_name:
            try:
                m = re.search(r'<world\s+name=["\']([^"\']+)["\']',
                              sdf.read_text(encoding="utf-8", errors="replace"))
                gz_name = m.group(1) if m else "default"
                e.meta.setdefault("gz_world_name", gz_name)
            except OSError as ex:
                e.error = e.error or f"unreadable: {ex}"
                gz_name = "?"

        # Scenery the world says it needs. Checking here means the menu can say
        # "missing hv_tower_220kv" instead of Gazebo failing on a URI later.
        missing = [m for m in man.get("requires_models", []) if m not in known]
        if missing:
            e.error = e.error or f"missing scenery: {', '.join(missing)}"

        size_mb = sdf.stat().st_size / 1e6
        bits = [f"gz world '{gz_name}'", f"{size_mb:.1f} MB"]
        if man.get("heavy"):
            bits.append("heavy")
        sp = man.get("spawn", {})
        if sp:
            bits.append(f"{sp.get('lat','?'):.4f},{sp.get('lon','?'):.4f}"
                        if isinstance(sp.get("lat"), float) else "spawn set")
        e.detail = " · ".join(bits)
        out.append(e)
    return out


# ── bags ─────────────────────────────────────────────────────────────────────


def bags() -> list[Entry]:
    """
    Recorded flights. A bag is a DIRECTORY holding metadata.yaml plus one or
    more .mcap files; one level of nesting is allowed so a folder of recordings
    can be copied in whole.

    metadata.yaml is read directly rather than by shelling out to
    `ros2 bag info`, which costs 4-5 s per bag and would make the screen
    unusable with a dozen of them.
    """
    out: list[Entry] = []
    root = paths.REPO / "bags"
    if not root.is_dir():
        return out

    def consider(d: Path, label: str) -> None:
        meta = d / "metadata.yaml"
        if not meta.is_file():
            return
        e = Entry(name=label, path=d, kind="bag")
        try:
            text = meta.read_text(encoding="utf-8", errors="replace")
        except OSError as ex:
            e.error = f"unreadable: {ex}"
            out.append(e)
            return
        info = _bag_summary(text)
        e.meta = info
        cmds = info.get("command_topics", 0)
        e.detail = (f"{_hms(info.get('duration_s', 0))} · "
                    f"{info.get('messages', 0):,} msgs · "
                    f"{info.get('topics', 0)} topics · "
                    f"{'can fly back' if cmds else 'telemetry only'}")
        out.append(e)

    for d in sorted(root.iterdir(), key=lambda p: p.name):
        if not d.is_dir():
            continue
        consider(d, d.name)
        for sub in sorted(d.iterdir(), key=lambda p: p.name):
            if sub.is_dir():
                consider(sub, f"{d.name}/{sub.name}")
    return out


# The five topics that make a bag able to re-fly the aircraft.
_COMMAND_TOPICS = (
    "flight_control_setpoint_ENUvelocity_yawrate",
    "flight_control_setpoint_ENUposition_yaw",
    "flight_control_setpoint_FLUvelocity_yawrate",
    "flight_control_setpoint_rollpitch_yawrate_thrust",
    "flight_control_setpoint_generic",
)


def _bag_summary(text: str) -> dict:
    """
    Pull what the Bags screen needs out of metadata.yaml, without a YAML parser.

    Deliberately regex rather than PyYAML: walker is stdlib-only (D2), and the
    handful of scalars needed here are unambiguous in rosbag2's output.
    """
    def num(pattern: str) -> int:
        m = re.search(pattern, text)
        return int(m.group(1)) if m else 0

    topics = re.findall(r"^\s*name:\s*(\S+)", text, re.M)
    cmds = sum(1 for t in topics if any(t.endswith(c) for c in _COMMAND_TOPICS))
    return {
        "duration_s": num(r"duration:\s*\n\s*nanoseconds:\s*(\d+)") / 1e9,
        "messages": num(r"message_count:\s*(\d+)"),
        "topics": len(set(topics)),
        "command_topics": cmds,
        "topic_names": sorted(set(topics)),
    }


def _hms(seconds: float) -> str:
    s = int(seconds)
    return f"{s // 60}:{s % 60:02d}"


# ── projects ─────────────────────────────────────────────────────────────────


def projects() -> list[Entry]:
    """
    C++ missions. A project is a directory with package.xml, CMakeLists.txt
    containing an add_executable, and src/.

    V2 requires the package name to EQUAL the directory name. V1 allowed them
    to differ (demo_arm_takeoff_logs held package demo_arm_takeoff) and every
    tool then needed both names; the constraint costs nothing and removes a
    whole class of "which one do I type" confusion.
    """
    out: list[Entry] = []
    root = paths.REPO / "projects"
    if not root.is_dir():
        return out
    for d in sorted(root.iterdir(), key=lambda p: p.name):
        if not d.is_dir():
            continue
        pkg_xml, cmake = d / "package.xml", d / "CMakeLists.txt"
        if not (pkg_xml.is_file() and cmake.is_file() and (d / "src").is_dir()):
            continue
        e = Entry(name=d.name, path=d, kind="project")
        try:
            pkg = re.search(r"<name>\s*([^<\s]+)\s*</name>",
                            pkg_xml.read_text(encoding="utf-8", errors="replace"))
            pkg_name = pkg.group(1) if pkg else ""
        except OSError:
            pkg_name = ""
        if pkg_name and pkg_name != d.name:
            e.error = f"package.xml says '{pkg_name}' but the directory is '{d.name}'"
        try:
            if "add_executable" not in cmake.read_text(encoding="utf-8", errors="replace"):
                e.error = e.error or "CMakeLists.txt has no add_executable"
        except OSError:
            pass
        conf = _project_conf(d / "project.conf")
        e.meta = conf
        e.detail = " · ".join(
            filter(None, [f"cameras={conf.get('CAMERAS', 'none')}",
                          f"setpoint={conf.get('SETPOINT', 'velocity')}",
                          "rviz" if conf.get("RVIZ") else ""]))
        out.append(e)
    return out


def _project_conf(path: Path) -> dict:
    """
    KEY=value, read as DATA and never sourced.

    `source <project>/project.conf` would execute whatever a project directory
    happens to contain -- a project you cloned from a colleague is not
    something to run as a shell script just to find out which camera profile it
    wants.
    """
    conf: dict[str, str] = {}
    if not path.is_file():
        return conf
    try:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            line = line.split("#", 1)[0].strip()
            if "=" in line:
                k, _, v = line.partition("=")
                conf[k.strip()] = v.strip()
    except OSError:
        pass
    return conf


# ── everything, cached on mtime ──────────────────────────────────────────────

_CACHE: dict[str, tuple[float, list[Entry]]] = {}


def _dir_stamp(*dirs: Path) -> float:
    """
    A cheap change signal: the newest mtime among the directories themselves
    and their immediate children. Adding, removing or renaming anything at that
    level changes it, which is exactly what a rescan needs to notice.
    """
    newest = 0.0
    for d in dirs:
        if not d.exists():
            continue
        try:
            newest = max(newest, d.stat().st_mtime)
            for child in os.scandir(d):
                newest = max(newest, child.stat(follow_symlinks=False).st_mtime)
        except OSError:
            pass
    return newest


_SOURCES = {
    "models": ("models",),
    "drones": ("models",),
    "worlds": ("worlds",),
    "bags": ("bags",),
    "projects": ("projects",),
}
_FUNCS = {"models": models, "drones": drones, "worlds": worlds,
          "bags": bags, "projects": projects}


def get(kind: str, force: bool = False) -> list[Entry]:
    """Cached scan. `force=True` is what F5 does."""
    stamp = _dir_stamp(*(paths.REPO / d for d in _SOURCES[kind]))
    hit = _CACHE.get(kind)
    if hit and not force and hit[0] == stamp:
        return hit[1]
    result = _FUNCS[kind]()
    _CACHE[kind] = (stamp, result)
    return result


def invalidate() -> None:
    _CACHE.clear()
