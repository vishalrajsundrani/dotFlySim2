"""
Where everything lives, and what gets mounted where.

THIS IS THE SINGLE SOURCE OF TRUTH FOR THE CONTAINER'S SHAPE. The mount table
below is read by the container launcher, by `walker doctor`, and by the
documentation generator. If a path moves, it moves here and nowhere else.

WHY A TABLE AND NOT A `docker run` STRING
-----------------------------------------
Version 1 built its `docker run` as a 20-line shell command with the mounts
inline. Three things then had to agree by hand: the command, the readiness
checks that looked for those paths, and the README that told you what was
mounted. They drifted. Here the launcher, the checker and the docs all read
this list, so drifting is not possible.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

# ── the repository ───────────────────────────────────────────────────────────
# walker/paths.py -> walker/ -> the repo root.
REPO = Path(__file__).resolve().parent.parent

# ── container identity ───────────────────────────────────────────────────────
# Overridable so two checkouts can run side by side without fighting over one
# container name; walker prints which it is using whenever it is not the default.
IMAGE = os.environ.get("WALKER_IMAGE", "dotflysim2:latest")
CONTAINER = os.environ.get("WALKER_CONTAINER", "dotflysim2")
CONTAINER_USER = "developer"
HOME = f"/home/{CONTAINER_USER}"
WORKSPACE = f"{HOME}/ws"

# The PSDK surface's namespace. Every wrapper topic and service hangs off this.
WRAPPER_PREFIX = "/wrapper/psdk_ros2"

# Where walker keeps its runtime state on the host. Gitignored; safe to delete.
RUNTIME = REPO / ".walker"
SOCKET_NAME = "walkerd.sock"
# The same directory as the container sees it. walkerd binds the socket here.
CONTAINER_RUNTIME = "/run/walker"


@dataclass(frozen=True)
class Mount:
    """One bind mount, plus why it exists — the 'why' is printed by `doctor`."""

    host: Path
    container: str
    mode: str  # "rw" or "ro"
    why: str
    # Must this exist before the container can start? Directories that walker
    # creates itself (bags, .walker) are False; content directories are True.
    required: bool = True

    def spec(self) -> str:
        return f"{self.host}:{self.container}:{self.mode}"


# ── the mount table ──────────────────────────────────────────────────────────
# EVERYTHING AN OPERATOR CAN CHOOSE OR EDIT IS HERE. The image holds only what
# is expensive and stable (see the Dockerfile header); this is the other half
# of that bargain. A world, a drone, a mission or a bag is added by putting a
# folder in one of these directories — never by rebuilding.
MOUNTS: tuple[Mount, ...] = (
    Mount(
        REPO / "models",
        f"{HOME}/gz_models",
        "rw",
        "drone and scenery models; scanned by walker, composed at run time",
    ),
    Mount(
        REPO / "worlds",
        f"{HOME}/gz_worlds",
        "rw",
        "worlds; scanned by walker, symlinked into PX4's world path at run time",
    ),
    Mount(
        REPO / "projects",
        f"{WORKSPACE}/src/projects",
        "rw",
        "C++ missions; inside the colcon workspace so a build needs no image change",
    ),
    Mount(
        REPO / "bridge",
        f"{WORKSPACE}/src/bridge",
        "rw",
        "the PSDK bridge (simty); editable without a rebuild",
    ),
    Mount(
        REPO / "walkerd",
        f"{WORKSPACE}/src/walkerd",
        "rw",
        "the supervisor itself; the image only carries shims that exec this",
    ),
    Mount(
        REPO / "walker_rviz_panel",
        f"{WORKSPACE}/src/walker_rviz_panel",
        "rw",
        "the RViz panel package, built on demand",
    ),
    Mount(
        REPO / "simsupport",
        f"{WORKSPACE}/src/simsupport",
        "rw",
        "the gimbal rig and sensor models that make the payload follow the aircraft",
    ),
    Mount(
        REPO / "tools",
        f"{WORKSPACE}/src/tools",
        "rw",
        "compose_sim.py and the probes",
    ),
    Mount(
        REPO / "config",
        f"{WORKSPACE}/config",
        "rw",
        "dds.xml, the bridge.yaml template, rviz configs",
    ),
    Mount(
        REPO / "bags",
        f"{HOME}/bags",
        # READ-WRITE, unlike V1. V1 mounted this read-only and then had to
        # record to a container path and `docker cp` the bag out afterwards --
        # a whole code path (RECORDING_SPILLED) that existed only because of
        # this flag. Recording writes straight to the host now.
        "rw",
        "recorded flights; rosbag writes here directly",
        required=False,
    ),
    Mount(
        RUNTIME,
        CONTAINER_RUNTIME,
        "rw",
        "walkerd's socket and unit state",
        required=False,
    ),
)

# Directories walker creates rather than demanding. Docker would create a
# missing bind-mount source itself, but as ROOT, which then cannot be written
# from the host -- a confusing permission error much later. So they are made
# here, as this user, before the container starts.
AUTOCREATE = tuple(m.host for m in MOUNTS if not m.required)


def socket_path() -> Path:
    """The walkerd socket, as the host sees it."""
    return RUNTIME / SOCKET_NAME


def container_socket() -> str:
    """The walkerd socket, as the container sees it."""
    return f"{CONTAINER_RUNTIME}/{SOCKET_NAME}"


def missing_mounts() -> list[Mount]:
    """Required mount sources that are not present. Empty list means good."""
    return [m for m in MOUNTS if m.required and not m.host.is_dir()]
