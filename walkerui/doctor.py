"""
`walker doctor` — answer, by name, every question the stack depends on.

WHY THIS EXISTS
---------------
When a simulation does not come up, the symptom is almost always the same
regardless of the cause: a C++ project sits waiting for telemetry, or Gazebo
shows an empty world. Version 1's answer was a set of probes buried inside a
3 000-line bash script, run only as part of starting a mode. Here they are a
command you can run at any time, and each one names the layer it tested.

Every check returns a (state, detail, fix) triple. A failing check must always
carry a `fix` -- a line you can actually type. A check that can only say "no"
is a check that sends someone to guess.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass, field

from . import dockerctl, paths

OK, WARN, FAIL, SKIP = "ok", "warn", "fail", "skip"


@dataclass
class Check:
    name: str
    state: str
    detail: str = ""
    fix: str = ""


@dataclass
class Report:
    checks: list[Check] = field(default_factory=list)

    def add(self, *a, **kw) -> Check:
        c = Check(*a, **kw)
        self.checks.append(c)
        return c

    @property
    def failed(self) -> list[Check]:
        return [c for c in self.checks if c.state == FAIL]

    @property
    def ok(self) -> bool:
        return not self.failed


# ── the host ─────────────────────────────────────────────────────────────────


def _check_host(r: Report) -> None:
    try:
        eng = dockerctl.engine()
        ver = subprocess.run(eng + ["version", "--format", "{{.Server.Version}}"],
                             capture_output=True, text=True, timeout=20).stdout.strip()
        r.add("container engine", OK, f"{' '.join(eng)} {ver}")
    except dockerctl.DockerError as e:
        r.add("container engine", FAIL, str(e).splitlines()[0],
              fix=str(e).split("\n", 1)[1].strip() if "\n" in str(e) else "")
        return

    r.add("image", OK if dockerctl.image_present() else FAIL,
          paths.IMAGE if dockerctl.image_present() else f"'{paths.IMAGE}' not built",
          fix=f"cd {paths.REPO} && docker build -t {paths.IMAGE} .")

    # The display. Gazebo, RViz and QGC are all X11 clients; on this host that
    # is XWayland behind a Wayland session, which is fine -- what matters is
    # that DISPLAY is set and the socket directory exists to be mounted.
    disp = os.environ.get("DISPLAY")
    if not disp:
        r.add("display", FAIL, "DISPLAY is unset",
              fix="run walker from a graphical session, or export DISPLAY=:0")
    elif not os.path.isdir("/tmp/.X11-unix"):
        r.add("display", WARN, f"DISPLAY={disp} but /tmp/.X11-unix is missing",
              fix="GUI units (gazebo gui, rviz, qgc) will not be able to open a window")
    else:
        session = os.environ.get("XDG_SESSION_TYPE", "?")
        r.add("display", OK, f"DISPLAY={disp} ({session}; GUI clients use XWayland)")

    # The GPU. Absent is survivable -- Gazebo falls back to software rendering
    # -- but it changes what camera profile is sensible, so it is reported.
    if os.path.exists("/dev/dri/renderD128"):
        r.add("gpu", OK, "/dev/dri/renderD128 present; render group will be passed in")
    else:
        r.add("gpu", WARN, "no /dev/dri/renderD128",
              fix="Gazebo will software-render: prefer the 'none' camera profile")

    # A terminal emulator, for the per-unit windows. Not fatal: walker shows
    # every unit's log itself, so a missing emulator costs the separate window
    # and nothing else.
    from .terminal import detect
    term = detect()
    if term:
        r.add("terminal emulator", OK, f"{term.name} -- units get their own window")
    else:
        r.add("terminal emulator", WARN, "none of the known emulators is installed",
              fix="units will run without a separate window; walker still shows their logs. "
                  "Override with SIM_TERMINAL='<cmd template>'")


# ── the repository ───────────────────────────────────────────────────────────


def _check_repo(r: Report) -> None:
    missing = paths.missing_mounts()
    if missing:
        r.add("repository layout", FAIL,
              "missing: " + ", ".join(m.host.name for m in missing),
              fix="this checkout is incomplete -- these directories are mount sources")
    else:
        r.add("repository layout", OK, f"{len(paths.MOUNTS)} mount sources present")

    dds = paths.REPO / "config" / "dds.xml"
    r.add("dds profile", OK if dds.is_file() else FAIL,
          str(dds.relative_to(paths.REPO)) if dds.is_file() else "config/dds.xml is missing",
          fix="without it Fast DDS falls back to multicast on every interface")


# ── the container ────────────────────────────────────────────────────────────


def _check_container(r: Report) -> None:
    st = dockerctl.state()
    if not st.running:
        r.add("container", WARN if st.exists else SKIP, f"'{paths.CONTAINER}' is {st.label}",
              fix="walker up")
        return
    r.add("container", OK, f"'{paths.CONTAINER}' {st.status}")

    # Every mount, checked from INSIDE -- the only place the answer is real.
    # A path can exist on the host and still be absent, or read-only, in the
    # container, and that difference is exactly what breaks a recording.
    bad: list[str] = []
    for m in paths.MOUNTS:
        want_w = m.mode == "rw"
        probe = f"test -d {m.container}" + (f" && test -w {m.container}" if want_w else "")
        rc, _ = dockerctl.exec_(probe, ros=False, timeout=30)
        if rc != 0:
            bad.append(f"{m.container} ({'not writable' if want_w else 'missing'})")
    if bad:
        r.add("mounts", FAIL, "; ".join(bad),
              fix="walker down --rm && walker up   (mounts are fixed when a container is created)")
    else:
        r.add("mounts", OK, f"all {len(paths.MOUNTS)} present and writable as declared")

    # Shared memory. The single most confusing failure in V1: too small, and
    # Fast DDS participants fail to register SHM transport and then silently
    # fail to discover each other.
    rc, out = dockerctl.exec_("df -m /dev/shm | awk 'NR==2{print $2\" \"$4}'", ros=False, timeout=30)
    if rc == 0 and out:
        total, free = (int(x) for x in out.split()[:2])
        if total < 1024:
            r.add("/dev/shm", FAIL, f"{total} MB total -- too small for this stack",
                  fix="walker down --rm && walker up   (walker starts it with --shm-size=2g)")
        elif free < 512:
            r.add("/dev/shm", WARN, f"{free} MB free of {total} MB",
                  fix="stale participants may be holding segments; restart the stack")
        else:
            r.add("/dev/shm", OK, f"{free} MB free of {total} MB")


# ── discovery isolation: the no-multicast rule ───────────────────────────────


def _check_isolation(r: Report) -> None:
    """
    The V2 promise that the simulation cannot be seen from, or reach, the LAN.

    Three independent things have to hold, and each is checked separately so a
    failure names which one broke.
    """
    if not dockerctl.state().running:
        r.add("discovery isolation", SKIP, "container not running")
        return

    # 1. Not on the host network. If this regresses, everything else is moot.
    rc, mode = dockerctl.exec_("true", ros=False, timeout=20)
    net = subprocess.run(
        dockerctl.engine() + ["inspect", paths.CONTAINER,
                              "--format", "{{.HostConfig.NetworkMode}}"],
        capture_output=True, text=True, timeout=30).stdout.strip()
    if net == "host":
        r.add("network mode", FAIL, "container is on --network host",
              fix="V2 must run on the default bridge network; recreate with walker down --rm && walker up")
    else:
        r.add("network mode", OK, f"{net} (not host -- DDS cannot reach the LAN)")

    # 2. No published ports. Everything (QGC, MAVLink, XRCE, video) is internal.
    ports = subprocess.run(
        dockerctl.engine() + ["port", paths.CONTAINER],
        capture_output=True, text=True, timeout=30).stdout.strip()
    if ports:
        r.add("published ports", WARN, ports.replace("\n", "; "),
              fix="nothing in V2 needs a published port")
    else:
        r.add("published ports", OK, "none -- nothing is reachable from outside")

    # 3. The environment the middleware actually reads.
    rc, out = dockerctl.exec_(
        "echo \"$ROS_DOMAIN_ID|$ROS_AUTOMATIC_DISCOVERY_RANGE|"
        "$RMW_IMPLEMENTATION|$FASTRTPS_DEFAULT_PROFILES_FILE\"", ros=False, timeout=30)
    dom, rng, rmw, prof = (out.split("|") + ["", "", "", ""])[:4]
    if rng != "LOCALHOST":
        r.add("discovery range", FAIL, f"ROS_AUTOMATIC_DISCOVERY_RANGE={rng or 'unset'}",
              fix="must be LOCALHOST; it is set as ENV in the Dockerfile")
    else:
        r.add("discovery range", OK, f"LOCALHOST (domain {dom}, {rmw})")

    rc, _ = dockerctl.exec_(f"test -f {prof}", ros=False, timeout=30) if prof else (1, "")
    if rc != 0:
        r.add("dds profile in container", FAIL, f"{prof or 'unset'} is not readable",
              fix="config/ is mounted at " + f"{paths.WORKSPACE}/config")
    else:
        rc2, has_mc = dockerctl.exec_(
            f"grep -c 'metatrafficUnicastLocatorList' {prof} || true", ros=False, timeout=30)
        r.add("dds profile in container", OK,
              f"{prof} (explicit unicast locator: {'yes' if has_mc.strip() not in ('', '0') else 'NO'})")


# ── the ROS graph ────────────────────────────────────────────────────────────


def _check_ros(r: Report) -> None:
    if not dockerctl.state().running:
        r.add("ros 2", SKIP, "container not running")
        return
    rc, out = dockerctl.exec_(
        'echo "$ROS_DISTRO $(ros2 pkg list 2>/dev/null | wc -l) packages"', timeout=90)
    r.add("ros 2", OK if (rc == 0 and out and not out.startswith(" 0")) else FAIL,
          out.strip(), fix="the ROS overlay failed to source inside the container")

    # The two message packages that must be built into the image; everything
    # else compiles against them.
    rc, out = dockerctl.exec_(
        "ros2 pkg list 2>/dev/null | grep -cE '^(px4_msgs|psdk_interfaces)$'", timeout=90)
    n = out.strip() or "0"
    r.add("message packages", OK if n == "2" else FAIL,
          f"px4_msgs + psdk_interfaces: {n}/2 present",
          fix="rebuild the image -- these are built in at STEP 8")

    rc, out = dockerctl.exec_("test -x $HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo yes",
                              ros=False, timeout=30)
    r.add("px4 sitl binary", OK if "yes" in out else FAIL,
          "prebuilt in the image" if "yes" in out else "missing",
          fix="rebuild the image -- FIX LAYER 4 compiles it")

    # C1: PX4 spawns models by absolute path, so these directories must exist
    # and be writable for the composer to symlink into at run time.
    rc, out = dockerctl.exec_(
        "for d in $HOME/PX4-Autopilot/Tools/simulation/gz/models "
        "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds $HOME/gz_runtime; do "
        "test -w $d || echo $d; done", ros=False, timeout=30)
    if out.strip():
        r.add("runtime model dirs", FAIL, "not writable: " + out.replace("\n", " "),
              fix="the composer symlinks the chosen drone and world into these; rebuild the image")
    else:
        r.add("runtime model dirs", OK, "writable -- composer can install the chosen drone/world")


def _check_isolation_deep(r: Report) -> None:
    """
    PROVE the isolation claim rather than asserting it from an env var.

    A publisher is started inside our container; a second, throwaway container
    on the same default bridge network then looks for it. If V2's discovery
    confinement works, the second container sees NOTHING -- and that is the
    whole promise: another process on this machine, let alone another machine,
    cannot see or fly your aircraft.

    Slow (it pays two ROS CLI start-ups), so it is behind --deep.
    """
    if not dockerctl.state().running:
        r.add("isolation (proof)", SKIP, "container not running")
        return

    talker = ("nohup ros2 topic pub -r 5 /walker_isolation_probe std_msgs/String "
              "'{data: probe}' >/tmp/probe.log 2>&1 &")
    dockerctl.exec_(talker, timeout=60)
    import time
    time.sleep(12)

    # Is it actually publishing where it should be visible -- inside?
    rc, inside = dockerctl.exec_(
        "timeout 25 ros2 topic list 2>/dev/null | grep -c walker_isolation_probe || true",
        timeout=60)
    seen_inside = inside.strip() not in ("", "0")

    # Now from a separate container on the same bridge network.
    probe_cmd = (
        "set +u; source /opt/ros/jazzy/setup.bash; "
        "timeout 25 ros2 topic list 2>/dev/null | grep -c walker_isolation_probe || true")
    out = subprocess.run(
        dockerctl.engine() + ["run", "--rm", "--shm-size=256m",
                              "-e", "ROS_DOMAIN_ID=0",
                              "-e", "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST",
                              paths.IMAGE, "bash", "-c", probe_cmd],
        capture_output=True, text=True, timeout=180).stdout.strip()
    seen_outside = out.splitlines()[-1].strip() not in ("", "0") if out else False

    dockerctl.exec_("pkill -f walker_isolation_probe || true", ros=False, timeout=30)

    if not seen_inside:
        r.add("isolation (proof)", WARN,
              "the probe publisher never appeared even inside; test inconclusive",
              fix="re-run `walker doctor --deep` once the container has settled")
    elif seen_outside:
        r.add("isolation (proof)", FAIL,
              "a SEPARATE container discovered this simulation's topics",
              fix="discovery is leaking: check ROS_AUTOMATIC_DISCOVERY_RANGE and config/dds.xml")
    else:
        r.add("isolation (proof)", OK,
              "visible inside, invisible to a separate container -- confinement holds")


def run(deep: bool = False) -> Report:
    r = Report()
    _check_host(r)
    _check_repo(r)
    _check_container(r)
    _check_isolation(r)
    _check_ros(r)
    if deep:
        _check_isolation_deep(r)
    return r
