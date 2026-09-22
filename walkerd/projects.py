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


def run_argv(name: str, params: dict | None = None) -> list[str]:
    """
    How to launch the mission.

    `ros2 run` rather than the binary directly: it resolves the package's
    environment, and a project that loads a config or an RViz file from its own
    share/ directory then finds it.

    Parameters are passed the ROS way (--ros-args -p k:=v) so a mission's
    declare_parameter defaults stay in the code, where CPP_DESIGN.md says they
    belong, and walker only overrides them.
    """
    args = ""
    if params:
        parts = []
        for k, v in params.items():
            if isinstance(v, bool):
                v = "true" if v else "false"
            parts.append(f"-p {k}:={v}")
        args = " --ros-args " + " ".join(parts)

    # THE OVERLAY IS SOURCED AT SPAWN, NOT INHERITED.
    #
    # walkerd sources ~/ws/install/setup.bash when IT starts. A mission built
    # afterwards is not in that environment, and `ros2 run` answers
    # "Package 'demo_orbit_mission' not found" about a package that was
    # compiled successfully thirty seconds earlier -- which reads as a build
    # failure and is not one.
    #
    # Sourcing inside the unit's own shell means a project built at any point
    # is runnable immediately, with no walkerd restart.
    #
    # `set +u` around it for the usual reason: ROS's setup.bash reads
    # $AMENT_TRACE_SETUP_FILES with no default.
    return ["/bin/bash", "-c",
            "set +u; source /opt/ros/jazzy/setup.bash; "
            f"source {WS}/install/setup.bash; "
            f"exec ros2 run {name} {executable_of(name)}{args}"]


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
    if not os.path.isdir(PROJECTS):
        return []
    return [status(n) for n in sorted(os.listdir(PROJECTS))
            if os.path.isdir(os.path.join(PROJECTS, n))]
