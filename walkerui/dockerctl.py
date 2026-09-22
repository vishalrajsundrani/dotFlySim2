"""
The container's lifecycle, from the host.

Everything `docker` in walker goes through here. Two rules the rest of walker
depends on:

  1. NOTHING HERE BLOCKS FOREVER. Every call has a timeout, because this code
     runs on walker's worker threads and a wedged docker daemon must degrade
     into a red status line, not a frozen TUI.

  2. EVERY FAILURE SAYS WHAT TO DO. "docker: permission denied" is a dead end;
     "your user is not in the docker group -- run `sudo usermod -aG docker
     $USER` and log out" is an answer.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from dataclasses import dataclass

from . import paths


class DockerError(RuntimeError):
    """A docker operation failed, with a message meant for a human."""


# ── engine detection ─────────────────────────────────────────────────────────
# Resolved once. Docker without group membership needs sudo, and discovering
# that through a permission error halfway into a container start is worse than
# probing for it up front.
_ENGINE: list[str] | None = None


def engine() -> list[str]:
    global _ENGINE
    if _ENGINE is not None:
        return _ENGINE

    exe = shutil.which("docker") or shutil.which("podman")
    if not exe:
        raise DockerError(
            "neither docker nor podman is installed.\n"
            "  Install docker:  sudo apt install docker.io  (then log out and back in)"
        )

    # Plain first: a user in the docker group needs no sudo, and asking for a
    # password when none is required is its own kind of wrong.
    probe = subprocess.run(
        [exe, "version", "--format", "{{.Server.Version}}"],
        capture_output=True, text=True, timeout=20,
    )
    if probe.returncode == 0:
        _ENGINE = [exe]
        return _ENGINE

    err = (probe.stderr or "").lower()
    if "permission denied" in err or "connect" in err:
        if shutil.which("sudo"):
            _ENGINE = ["sudo", exe]
            return _ENGINE
        raise DockerError(
            f"{exe} needs privileges this user does not have, and sudo is missing.\n"
            f"  Fix:  sudo usermod -aG docker $USER   (then log out and back in)"
        )
    raise DockerError(
        f"{exe} is installed but not responding:\n  {(probe.stderr or '').strip()}\n"
        f"  Is the daemon running?  systemctl status docker"
    )


def _run(args: list[str], timeout: float = 60, check: bool = True) -> str:
    cp = subprocess.run(engine() + args, capture_output=True, text=True, timeout=timeout)
    if check and cp.returncode != 0:
        raise DockerError((cp.stderr or cp.stdout or "").strip() or f"docker {args[0]} failed")
    return cp.stdout.strip()


# ── image ────────────────────────────────────────────────────────────────────


def image_present() -> bool:
    try:
        _run(["image", "inspect", paths.IMAGE], timeout=30)
        return True
    except DockerError:
        return False


# ── container ────────────────────────────────────────────────────────────────


@dataclass
class ContainerState:
    exists: bool
    running: bool
    status: str = ""

    @property
    def label(self) -> str:
        if not self.exists:
            return "absent"
        return "up" if self.running else f"stopped ({self.status})"


def state() -> ContainerState:
    out = _run(
        ["ps", "-a", "--filter", f"name=^{paths.CONTAINER}$",
         "--format", "{{.State}}\t{{.Status}}"],
        timeout=30, check=False,
    )
    if not out:
        return ContainerState(exists=False, running=False)
    st, _, status = out.partition("\t")
    return ContainerState(exists=True, running=(st == "running"), status=status)


def _gpu_args() -> list[str]:
    """
    Render-group access, but only when the render node actually exists.

    Passing a group id for a missing device fails the run outright, which is
    how V1 broke on GPU-less machines. Without it Gazebo falls back to software
    rendering: slow, but it flies.
    """
    node = "/dev/dri/renderD128"
    try:
        import os
        gid = os.stat(node).st_gid
    except OSError:
        return []
    return ["--group-add", str(gid)]


def allow_x11() -> str | None:
    """
    Let the container's GUI clients (Gazebo, RViz, QGC) reach this display.

    Returns a note for the log, or None if nothing was needed. On this host
    that is XWayland: DISPLAY=:0 with the X11 socket bind-mounted. `xhost` is
    best-effort -- many setups already allow local connections, and a machine
    without xhost installed should not fail to start a simulation over it.
    """
    import os
    if not os.environ.get("DISPLAY"):
        return "DISPLAY is unset -- Gazebo, RViz and QGC will have nowhere to draw"
    if not shutil.which("xhost"):
        return None
    cp = subprocess.run(["xhost", "+local:"], capture_output=True, text=True, timeout=15)
    return None if cp.returncode == 0 else "xhost failed; GUI windows may be refused"


def start(force_recreate: bool = False) -> list[str]:
    """
    Bring the container up, idempotently. Returns notes for the log.

    Detached with `sleep infinity` as PID 1, exactly as V1 settled on: the
    container must outlive any one terminal, so that closing the window a
    simulation was started from does not kill the simulation.
    """
    notes: list[str] = []

    if not image_present():
        raise DockerError(
            f"image '{paths.IMAGE}' is not built.\n"
            f"  Build it (~40 min):  cd {paths.REPO} && docker build -t {paths.IMAGE} ."
        )

    missing = paths.missing_mounts()
    if missing:
        raise DockerError(
            "these mount sources are missing from the repository:\n"
            + "\n".join(f"    {m.host}   ({m.why})" for m in missing)
        )

    # Make the directories docker would otherwise create as root.
    for d in paths.AUTOCREATE:
        d.mkdir(parents=True, exist_ok=True)

    st = state()
    if st.running and not force_recreate:
        notes.append(f"container '{paths.CONTAINER}' already running -- reusing it")
        return notes
    if st.exists:
        notes.append(f"removing the existing '{paths.CONTAINER}' container")
        _run(["rm", "-f", paths.CONTAINER], timeout=60)

    note = allow_x11()
    if note:
        notes.append(note)

    import os
    args = [
        "run", "-d", "--name", paths.CONTAINER,
        # Gazebo's renderer and the GPU nodes.
        "--privileged",
        # Fast DDS allocates a 32 MB SHM segment PER participant and V2 runs
        # 8+ of them. Docker's default /dev/shm is 64 MB, at which the fourth
        # participant fails with "RTPS_TRANSPORT_SHM Error: Failed to create
        # segment" and then silently fails to discover its peers.
        "--shm-size=2g",
        # NOTE: no --network host. V1 needed it for DDS multicast to a Manifold
        # on the LAN. V2's entire graph is in this one container, so the default
        # bridge network is correct AND safer: nothing can discover the
        # simulation from outside, and two engineers on one office network
        # cannot see each other's aircraft.
        *_gpu_args(),
        "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
        "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
    ]
    for m in paths.MOUNTS:
        args += ["-v", m.spec()]
    args += [paths.IMAGE, "sleep", "infinity"]

    _run(args, timeout=180)

    # `docker run -d` returns as soon as the container is created; give the
    # daemon a moment to actually mark it running before the first exec.
    import time
    for _ in range(20):
        if state().running:
            notes.append("container up")
            return notes
        time.sleep(1)
    raise DockerError(f"container '{paths.CONTAINER}' did not reach the running state")


def stop(remove: bool = False) -> list[str]:
    st = state()
    if not st.exists:
        return ["no container to stop"]
    if remove:
        _run(["rm", "-f", paths.CONTAINER], timeout=120)
        return ["container removed"]
    if st.running:
        _run(["stop", paths.CONTAINER], timeout=120)
        return ["container stopped (it still exists; `walker up` reuses it)"]
    return ["container was already stopped"]


# ── running things inside ────────────────────────────────────────────────────


def exec_(cmd: str, timeout: float = 60, ros: bool = True, check: bool = False) -> tuple[int, str]:
    """
    One command inside the container. Returns (returncode, combined output).

    `docker exec ... bash -c` is a NON-interactive shell: it does not read
    .bashrc, so none of the image's aliases or `source` lines are in effect.
    The load-bearing variables (RMW, domain, discovery range, DDS profile) are
    real ENV in the Dockerfile and do survive; what has to be added here is the
    ROS overlay, and nothing else.

    `set +u` around the sourcing is deliberate: ROS's own setup.bash reads
    $AMENT_TRACE_SETUP_FILES with no default, which under `set -u` is a fatal
    unbound-variable error naming a file nobody here wrote.
    """
    prelude = (
        "set +u; source /opt/ros/jazzy/setup.bash; "
        f"[ -f {paths.WORKSPACE}/install/setup.bash ] && source {paths.WORKSPACE}/install/setup.bash; "
    ) if ros else ""
    cp = subprocess.run(
        engine() + ["exec", paths.CONTAINER, "bash", "-c", prelude + cmd],
        capture_output=True, text=True, timeout=timeout,
    )
    out = (cp.stdout or "") + (cp.stderr or "")
    if check and cp.returncode != 0:
        raise DockerError(out.strip() or f"command failed inside the container: {cmd}")
    return cp.returncode, out.strip()


def walkerd_running() -> bool:
    rc, _ = exec_("pgrep -r DRSW -f '[w]alkerd/__main__.py' >/dev/null", ros=False, timeout=30)
    return rc == 0


def start_walkerd(wait: float = 15.0) -> list[str]:
    """
    Start the supervisor inside the container, if it is not already up.

    Detached and in its own session, so it outlives the `docker exec` that
    started it. It is walkerd, not this exec, that must own the units -- that
    ownership is the whole reason the orphan problems of V1 do not recur.
    """
    import time
    if walkerd_running():
        return ["walkerd already running"]
    # Invoked directly rather than through the image's /usr/local/bin/walkerd
    # shim. The shim is a convenience for a human in `walker shell`; walker
    # itself must not depend on it, because the shim is baked into the image
    # and an older image would silently have an older one. (It did: the first
    # build's shim carried `set -u`, which is fatal the moment ROS's setup.bash
    # reads $AMENT_TRACE_SETUP_FILES.) Spelling the command out here means
    # walker works against any image that has the mounts.
    cmd = (
        "set -eo pipefail; "
        "source /opt/ros/jazzy/setup.bash; "
        f"[ -f {paths.WORKSPACE}/install/setup.bash ] && "
        f"source {paths.WORKSPACE}/install/setup.bash; "
        f"exec python3 -B {paths.WORKSPACE}/src/walkerd/__main__.py "
        "> /tmp/walkerd.log 2>&1"
    )
    subprocess.Popen(
        engine() + ["exec", "-d", paths.CONTAINER, "bash", "-c", cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + wait
    while time.time() < deadline:
        if paths.socket_path().exists() and walkerd_running():
            return ["walkerd up"]
        time.sleep(0.5)
    rc, out = exec_("tail -5 /tmp/walkerd.log", ros=False, timeout=30)
    raise DockerError("walkerd did not start. Its last words:\n    " +
                      out.replace("\n", "\n    "))
