"""
C++ missions: finding them, building them, and knowing when a build is stale.

WHAT A PROJECT IS
=================
A directory under projects/ holding package.xml, CMakeLists.txt with exactly
one add_executable, and src/. The package name must EQUAL the directory name.

V1 allowed them to differ -- demo_arm_takeoff_logs held package
demo_arm_takeoff, and demo_square_mission's package.xml actually declared
demo_orbit_mission, so two directories built one package name and colcon picked
whichever it saw last. Every tool then needed to carry both names around. The
constraint costs nothing and removes the whole class of confusion; walker's
scanner reports a mismatch rather than letting it through.

STALENESS
=========
"Built" is not "built from what is on disk now". A mission whose source is
newer than its binary will run the OLD code, silently, and the symptom is a
change that appears to have had no effect. So the binary's mtime is compared
against every source file, and walker says `stale` rather than `built`.
"""

from __future__ import annotations

import os
import re
import subprocess

HOME = os.path.expanduser("~")
WS = os.path.join(HOME, "ws")
PROJECTS = os.path.join(WS, "src", "projects")
INSTALL = os.path.join(WS, "install")

SOURCE_SUFFIXES = (".cpp", ".hpp", ".h", ".cc", ".hh", ".txt", ".xml", ".conf")


def project_dir(name: str) -> str:
    return os.path.join(PROJECTS, name)


def executable_of(name: str) -> str:
    """The single add_executable target. Empty string if the project has none."""
    cmake = os.path.join(project_dir(name), "CMakeLists.txt")
    try:
        with open(cmake, encoding="utf-8", errors="replace") as fh:
            m = re.search(r"add_executable\s*\(\s*([A-Za-z0-9_]+)", fh.read())
    except OSError:
        return ""
    return m.group(1) if m else ""


def binary_path(name: str) -> str:
    exe = executable_of(name)
    return os.path.join(INSTALL, name, "lib", name, exe) if exe else ""


def newest_source(name: str) -> float:
    newest = 0.0
    for root, _dirs, files in os.walk(project_dir(name)):
        if "/build" in root or "/install" in root:
            continue
        for f in files:
            if f.endswith(SOURCE_SUFFIXES):
                try:
                    newest = max(newest, os.stat(os.path.join(root, f)).st_mtime)
                except OSError:
                    pass
    return newest


def status(name: str) -> dict:
    """One project's build state: missing | never built | stale | built."""
    d = project_dir(name)
    if not os.path.isdir(d):
        return {"name": name, "state": "missing",
                "detail": f"no such directory: {d}"}
    exe = executable_of(name)
    if not exe:
        return {"name": name, "state": "broken",
                "detail": "CMakeLists.txt has no add_executable"}
    binary = binary_path(name)
    if not os.path.isfile(binary):
        return {"name": name, "state": "unbuilt", "executable": exe,
                "detail": "never built"}
    built_at = os.stat(binary).st_mtime
    src_at = newest_source(name)
    if src_at > built_at:
        age = (src_at - built_at) / 60.0
        return {"name": name, "state": "stale", "executable": exe,
                "detail": f"source is {age:.0f} min newer than the binary"}
    return {"name": name, "state": "built", "executable": exe,
            "detail": "up to date"}


def build(name: str, timeout: float = 600.0) -> tuple[bool, str]:
    """
    colcon build for one package. Returns (ok, output tail).

    --symlink-install so a rebuild of a project that only changed its launch or
    config files does not need a reinstall, and --packages-select so building
    one mission does not rebuild px4_msgs.
    """
    cmd = (
        "set -eo pipefail; "
        "source /opt/ros/jazzy/setup.bash; "
        f"cd {WS} && colcon build --packages-select {name} "
        "--symlink-install "
        "--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1"
    )
    cp = subprocess.run(["/bin/bash", "-c", cmd], capture_output=True,
                        text=True, timeout=timeout)
    out = (cp.stdout or "") + (cp.stderr or "")
    return cp.returncode == 0, out.strip()[-1500:]


def run_argv(name: str, params: dict | None = None,
             build: bool = False, why: str = "") -> list[str]:
    """
    The whole life of a mission, as one shell command: build it, then fly it.

    WHY THE BUILD IS HERE AND NOT IN THE SUPERVISOR
    -----------------------------------------------
    Building in walkerd sent the compiler's output to the socket as log events,
    which meant the mission's terminal opened *after* the build and showed none
    of it. A failure became a one-line "failed to build; see the log", with the
    actual error in a different place from the window you were looking at.

    Run here, the unit's terminal carries the story in order: what is being
    built and why, every line the compiler said, then the mission's own output.
    That is the window you want open when something misbehaves.

    THE OVERLAY IS SOURCED AT SPAWN, NOT INHERITED. walkerd sources
    ~/ws/install/setup.bash when IT starts; a mission built afterwards is not
    in that environment, and `ros2 run` then answers "Package not found" about
    a package that compiled thirty seconds earlier.

    `set +u` around the sourcing for the usual reason: ROS's setup.bash reads
    $AMENT_TRACE_SETUP_FILES with no default.
    """
    args = ""
    if params:
        parts = []
        for k, v in params.items():
            if isinstance(v, bool):
                v = "true" if v else "false"
            parts.append(f"-p {k}:={v}")
        args = " --ros-args " + " ".join(parts)

    lines = [
        "set +u",
        "source /opt/ros/jazzy/setup.bash",
        f"cd {WS}",
    ]
    if build:
        lines += [
            f'echo "=== building {name} ({why}) ==="',
            # No `set -e`: the explicit check below reports a build failure in
            # words, where a bare non-zero exit would leave the terminal
            # showing compiler errors with no statement of what just happened.
            f"colcon build --packages-select {name} --symlink-install "
            "--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo",
            "rc=$?",
            'if [ "$rc" != "0" ]; then',
            f'    echo "=== {name} FAILED TO BUILD (exit $rc) ==="',
            '    echo "    The compiler output is above. Fix it and press p again."',
            "    exit $rc",
            "fi",
            f'echo "=== built {name} ==="',
        ]
    else:
        lines.append(f'echo "=== {name} is up to date, not rebuilding ==="')

    lines += [
        f"source {WS}/install/setup.bash",
        f'echo "=== running {name}{args} ==="',
        f"exec ros2 run {name} {executable_of(name)}{args}",
    ]
    return ["/bin/bash", "-c", "\n".join(lines)]

def conf(name: str) -> dict:
    """project.conf, read as DATA. Never sourced -- see walkerui/scan.py."""
    path = os.path.join(project_dir(name), "project.conf")
    out: dict[str, str] = {}
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                line = line.split("#", 1)[0].strip()
                if "=" in line:
                    k, _, v = line.partition("=")
                    out[k.strip()] = v.strip()
    except OSError:
        pass
    return out


def listing() -> list[dict]:
    """
    Every project, with its build state AND what it asks the simulation for.

    The build state alone is not enough to choose from: "never built" tells you
    nothing about whether a mission needs cameras, which setpoint frame it
    steers in, or how high it climbs. Those come from project.conf and are what
    you actually pick between.
    """
    if not os.path.isdir(PROJECTS):
        return []
    out = []
    for name in sorted(os.listdir(PROJECTS)):
        if not os.path.isdir(os.path.join(PROJECTS, name)):
            continue
        st = status(name)
        c = conf(name)
        wants = [f"cameras={c.get('CAMERAS', 'none')}",
                 f"setpoint={c.get('SETPOINT', 'velocity')}"]
        if c.get("RVIZ"):
            wants.append("rviz")
        st["wants"] = " · ".join(wants)
        out.append(st)
    return out
