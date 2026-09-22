# dotFlySim **Version 2** — Design & Implementation Plan

**Status:** proposal, awaiting review
**Date:** 2026-09-21
**Inherits from:** `/home/xenon/Documents/dotFly/Simulation/dotFlySim-latest/dotFlySim` (“Version 1”)
**Target repo:** `/home/xenon/Documents/dotFly/Simulation/dotFlySim2`
**Authoritative interface reference:** `/home/xenon/Downloads/psdk_ros2_topics_services.pdf`
(41 live telemetry topics · 5 command topics · 8 dead-on-M4E topics · 56 services)

> Read §1 and §2 first. §2 is the short list of things I want a yes/no on before any code is
> written; everything after it is the design those answers lock in.

---

## Table of contents

1. [What Version 2 is](#1-what-version-2-is)
2. [Decisions — RESOLVED](#2-decisions--resolved)
3. [What is inherited, changed and dropped](#3-what-is-inherited-changed-and-dropped)
4. [System architecture](#4-system-architecture)
5. [Walker — the TUI](#5-walker--the-tui)
6. [walkerd — the in-container supervisor](#6-walkerd--the-in-container-supervisor)
7. [The run-state machine and its constraints](#7-the-run-state-machine-and-its-constraints)
8. [Terminals: how a unit gets its own window](#8-terminals-how-a-unit-gets-its-own-window)
9. [Docker: image, mounts, discovery](#9-docker-image-mounts-discovery)
10. [Scanning: worlds, drone models, bags](#10-scanning-worlds-drone-models-bags)
11. [The camera system](#11-the-camera-system)
12. [The bridge (simty v2)](#12-the-bridge-simty-v2)
13. [The PSDK / wrapper surface](#13-the-psdk--wrapper-surface)
14. [RViz](#14-rviz)
15. [rosbag: record and replay](#15-rosbag-record-and-replay)
16. [QGroundControl](#16-qgroundcontrol)
17. [The C++ project contract](#17-the-c-project-contract)
18. [Repository layout](#18-repository-layout)
19. [Documentation deliverables](#19-documentation-deliverables)
20. [Milestones and acceptance criteria](#20-milestones-and-acceptance-criteria)
21. [Risks and de-risking spikes](#21-risks-and-de-risking-spikes)
22. [Explicit non-goals](#22-explicit-non-goals)

---

## 1. What Version 2 is

Version 1 is a simulator you drive with two large bash scripts (`run.sh`, 3 000 lines;
`simulation.sh`, 1 100 lines) that between them own a container, four operating modes, a
readiness-probe suite and a recording system. It works, and the parts that work best — the
PSDK↔PX4 bridge, the `demo_orbit_mission` flight pattern, the camera-composition trick, the
preflight probes — are exactly what Version 2 keeps.

What Version 2 changes is the **operator surface and the process model**:

- **One program to drive everything: `walker`**, a keyboard-driven TUI on the host. No mode
  flags, no remembering which script does what. You pick a world, a drone and a camera
  profile, press a key, and the simulation comes up.
- **Every long-lived, noisy thing gets its own terminal window** — the simulation, each C++
  project, QGC, a bag replay, RViz — while walker itself stays a compact dashboard that
  shows the important lines from all of them at once.
- **Nothing is compiled into the image that an operator should be able to choose.** Worlds,
  drone models and bags are scanned from disk on every start and mounted into the container,
  so dropping a folder in is the whole of “adding a world”.
- **No Manifold, no multicast, no remote anything.** The whole graph lives inside one
  container on one host. DDS discovery is pinned to localhost.
- **The PSDK/wrapper surface is the only control path.** It comes up with the simulation,
  always, and a C++ project that talks to it runs unchanged against a real aircraft.

### The one-sentence version

> *Walker starts a Dockerised PX4+Gazebo simulation of a chosen drone in a chosen world,
> exposes the full DJI PSDK ROS 2 surface over an always-on bridge, and lets you run C++
> missions, QGC, RViz and rosbag record/replay against it from one keyboard.*

---

## 2. Decisions — RESOLVED

Answered by the project owner on 2026-09-21. These are now constraints, not options.

| # | Question | **Decision** |
|---|---|---|
| **D1** | Keep V1's Tkinter flight panel (`gui/drone_controller.py`)? | **NO — removed.** QGC plus walker are sufficient for manual control. The 943-line panel is not ported and has no V2 counterpart. |
| **D2** | Walker's implementation language | **Python 3 + stdlib `curses`.** No third-party TUI dependency, ever. (Host is Python 3.14; `curses` is in the stdlib and `rich`/`textual` are *not* installed — see §5.4.) |
| **D3** | Does a bag replay need the simulation running? | **YES — always.** A replay may not start unless the `sim` unit is up, and stopping the simulation stops the replay. This removes V1's "no simulation at all" replay path entirely (§15.2). |
| **D4** | C++ project and QGC at the same time? | **Yes — and the warning is backed by a real lock.** Walker inserts a MAVLink guard between QGC and PX4 that *drops QGC's command traffic* while a project holds the flight lock, so QGC physically cannot intervene. Telemetry still flows up, so QGC stays usable as an instrument panel. Design in §16.2. |
| **D5** | Port `demo_gnss_stereo_inertial` (factor-graph SLAM)? | **Yes — M9, committed, not a stretch goal.** It is the proof that the perception stream is real and the only consumer of the fisheye pair. Scheduled for the next iteration immediately after the docs land. |
| **D6** | Keep V1's RViz C++ panel plugins (`gui/rviz_panels/`)? | **NO — V1's are deleted, and a new minimal panel is written from scratch** on an efficient design: one 5 Hz state message, one control topic, zero per-frame ROS work. Design in §14.3. |

### What these decisions changed elsewhere in this plan

| Decision | Sections rewritten |
|---|---|
| D1 | §3 (dropped list) |
| D3 | §7 constraint matrix, §15.2 replay |
| D4 | §7, §16.2 (new: the MAVLink guard), §18, §20 M6, §21 R11 |
| D5 | §20 M9 |
| D6 | §3, §14.3 (new: the walker RViz panel), §18, §20 M6 |

---

## 3. What is inherited, changed and dropped

### Inherited essentially as-is

| From V1 | Why it survives |
|---|---|
| `Dockerfile` (Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · PX4 SITL · Micro XRCE-DDS Agent) | It is correct and it took a long time to get correct. V2 edits it, does not restart it. See §9 for the diffs. |
| `gui/simty/` — converters, registry, node, QoS, clock, journal | This *is* the PSDK bridge. ~10 000 lines of message translation that already matches the PDF surface. |
| `config/bridge.yaml` (ros_gz_bridge routes) | Regenerated per-run by the composer, same schema. |
| `tools/compose_sim.py` | The camera-composition trick. Extended in §11. |
| `tools/preflight_probe.py` | One in-container process that answers “is the chain up” without paying the 4–5 s `ros2` CLI start-up per question. Becomes the health probe inside walkerd. |
| `models/` (m4e, m4e_camera, hv_tower_220kv, hv_span_220kv, transmission_tower), `worlds/powerline.sdf`, `tools/gen_powerline.py` | The content. Gains manifests (§10). |
| `psdk_interfaces/` (23 msgs, 38 srvs, 2 actions) | The message package the C++ projects and the bridge share. |
| `px4-patches/` (`server.config`, `GstCameraSystem`) | Needed for the QGC video stream and for gz-sim’s selective camera mode. |
| `demo/demo_orbit_mission` | **The reference project.** Every other C++ project in V2 is written in its shape (§17). |
| `demo/demo_arm_takeoff`, `demo_square_mission`, `demo_camera_track` | Ported to the V2 project contract as the worked examples for `CPP_DESIGN.md`. |

### Changed

| Thing | V1 | V2 |
|---|---|---|
| Operator entry point | `./run.sh <mode>` + interactive menus inside bash | `walker` TUI, one process, always the same screen |
| Process supervision | bash background jobs + `pkill -f` pattern matching | `walkerd`, a real supervisor with a unit table, PTYs and exit codes (§6) |
| Mutual exclusion | implicit; two modes could fight over the same topic | explicit state machine, enforced in one place (§7) |
| World & model selection | hard-coded `powerline.sdf` + `PX4_SIM_MODEL=gz_m4e`, airframe baked into the image | scanned from disk each start, airframe generated at runtime (§10) |
| Camera selection | 4 choices, applied at compose time, **requires a restart to change** | same 4 choices at boot **plus live on/off during flight** (§11) |
| Recording | a global on/off flag file; the *mode* decides the bag name | an independent unit you start and stop at any time, with a name you type (§15) |
| Bridge lifetime | started per mode, in three different ways | one unit, up whenever the simulation is up, never restarted by a mode |
| DDS | `--network host`, multicast SPDP, domain 42, shared with a Manifold | container-private, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, SHM-first |
| Logs | four log files in `/tmp` inside the container, read with `docker exec tail` | every unit writes a structured log; walker tails all of them into one pane |

### Dropped

| Dropped | Reason |
|---|---|
| **Mode 3 — Manifold RC** (`mode_rc`) | No Manifold in V2. |
| **`simty/discovery.py`, `remote.py`, `provision.py`, `mock.py`, `standin.py`** | All of these exist to find, start or fake a remote wrapper over multicast. There is no remote. (`mock.py` may return later as a test fixture — see M9.) |
| **Multicast DDS / `config/dds.xml` transport section** | Replaced by a localhost-only profile. |
| **`gui/bridge_panel.py`** | Superseded by walker. |
| **`gui/rviz_panels/`** (V1's C++ Qt panels) | **D6:** deleted, not ported. A new, much smaller panel is written from scratch in §14.3 — V1's version re-implemented half the bridge console inside Qt widgets and rebuilt a Qt toolchain on every image build. |
| **`gui/drone_controller.py`** (Tkinter flight panel) | **D1:** removed. QGC covers manual flight; walker covers everything else. |
| **`run.sh` / `simulation.sh`** | Replaced by walker + walkerd. A thin `./walker` launcher script is all that remains. |
| **`gui/control_room.sh`** | Walker starts RViz and the bridge itself. |

---

## 4. System architecture

```
 HOST (Ubuntu, GNOME/Wayland)                          CONTAINER  dotflysim2
┌────────────────────────────────────┐                ┌──────────────────────────────────────┐
│                                    │                │                                      │
│   walker  (curses TUI)             │  unix socket   │  walkerd  (supervisor, Python)       │
│   ├ scans models/ worlds/ bags/    │◄──────────────►│  ├ unit table + state machine        │
│   ├ owns the docker lifecycle      │  JSON-lines    │  ├ PTY per unit, log ring per unit   │
│   ├ renders state + merged logs    │  .walker/sock  │  ├ health probes (one rclpy node)    │
│   └ spawns terminal windows ───────┼──┐             │  └ constraint enforcement            │
│                                    │  │             │                                      │
└────────────────────────────────────┘  │             │   ── units it supervises ──          │
                                        │             │   sim       gz sim + PX4 SITL + XRCE  │
   ptyxis / gnome-terminal / xterm ◄────┘             │             + ros_gz_bridge           │
        each runs:                                    │   bridge    ROS_Bridge_Simty (always) │
        docker exec -it dotflysim2 \                  │   cameras   camera relay/switcher     │
              walker-attach <unit>                    │   project   one C++ node at a time    │
                                                      │   qgc       QGroundControl            │
                                                      │   rviz      rviz2                     │
                                                      │   record    ros2 bag record           │
                                                      │   replay    ros2 bag play             │
                                                      └──────────────────────────────────────┘
```

### The ROS 2 graph inside the container

```
      ┌─────────┐  uORB   ┌──────────────┐  DDS   ┌──────────────┐  DDS   ┌──────────────┐
      │  PX4    │────────►│ MicroXRCE    │───────►│   /fmu/out/* │───────►│              │
      │  SITL   │◄────────│ DDS Agent    │◄───────│   /fmu/in/*  │◄───────│  ROS_Bridge  │
      └────┬────┘         └──────────────┘        └──────────────┘        │    _Simty    │
           │ gz-transport                                                 │  (the bridge)│
      ┌────▼────┐         ┌──────────────┐        ┌──────────────┐        └──────┬───────┘
      │ gz sim  │────────►│ ros_gz_bridge│───────►│ /drone/...   │───────────────┘
      │ Harmonic│         │ (bridge.yaml)│        │ cameras, lrf │               │
      └─────────┘         └──────────────┘        └──────────────┘               │
                                                                                 ▼
                                                              ┌──────────────────────────────┐
                                                              │  /wrapper/psdk_ros2/*        │
                                                              │  41 telemetry · 5 command    │
                                                              │  56 services                 │
                                                              └───┬──────────┬───────────┬───┘
                                                                  │          │           │
                                                            C++ project    RViz     ros2 bag
```

**Design rule, stated once and enforced everywhere:** a C++ project sees *only*
`/wrapper/psdk_ros2/*`. It never subscribes to `/fmu/*`, never to `/drone/*`, never calls a
Gazebo service. That is what makes the same binary run against a real Matrice 4E.

---

## 5. Walker — the TUI

### 5.1 Principles

1. **Never block the keyboard.** Every docker call, every ROS query, every filesystem scan
   runs on a worker thread. The render loop only ever reads an in-memory state snapshot.
   Target: <16 ms per frame, keypress → visible acknowledgement <50 ms even when the
   container is busy.
2. **No `docker exec` on the hot path.** V1 paid 4–5 s for each `ros2` CLI invocation.
   Walker talks to walkerd over a unix socket; walkerd holds one long-lived rclpy node and
   answers from cached state.
3. **One screen for the common case.** Starting a simulation and running a mission should
   not require leaving the main screen.
4. **Every refusal explains itself.** If a key is disabled, the status line says which unit
   is holding the lock and which key stops it.

### 5.1b Toolchain, locked (D2)

Python 3 + **stdlib `curses` only**. No `rich`, no `textual`, no `blessed` — none of them are
installed on this host and adding one would put a pip dependency between an engineer and a
simulation. Everything walker needs is in the stdlib: `curses` (render), `threading` +
`queue` (workers), `socket` (walkerd), `json`, `subprocess` (docker, terminals), `os.scandir`
(scanning). `walker/` must stay importable on a bare Python 3.10+ interpreter, and CI asserts
that by importing it with `-I` (isolated mode).

### 5.2 Screens

```
┌ dotFlySim2 · walker ─────────────────────────────────── container: up · 00:14:21 ─┐
│                                                                                    │
│  WORLD    powerline            5 towers · 800 m corridor            [w] change     │
│  DRONE    m4e                  DJI Matrice 4E · quad · 14 cameras   [d] change     │
│  CAMERAS  fisheye only         ~20 MP/s                             [c] change     │
│                                                                                    │
│ ── units ─────────────────────────────────────────────────────────────────────────│
│   sim        ● running   px4 armed=no  gz 62 fps  clock 00:03:12       [s] stop    │
│   bridge     ● running   46 routes · 56 services · 1.2 kmsg/s          [b] restart │
│   project    ● running   demo_orbit_mission  ORBIT 14s  h=1.8m         [p] stop    │
│   qgc        ○ stopped                                                 [q] start   │
│   rviz       ○ stopped                                                 [v] start   │
│   record     ● recording bags/tower_pass_2  00:01:44  118 MB           [r] stop    │
│   replay     ○ stopped                                                 [R] start   │
│                                                                                    │
│ ── log ───────────────────────────────────────────────────────── [f] filter ──────│
│  12:04:11 project  [ORBIT 14s] status=ON_AIR mode=SDK_CTRL height=1.8 m            │
│  12:04:11 bridge   psdk_velocity_setpoint  20.0 Hz → /fmu/in/trajectory_setpoint   │
│  12:04:09 sim      INFO  [commander] Takeoff detected                              │
│  12:04:02 record   bags/tower_pass_2  41 topics · 8 services                       │
│                                                                                    │
│ [enter] unit detail  [t] terminal  [l] logs  [?] help  [Q] quit                     │
└────────────────────────────────────────────────────────────────────────────────────┘
```

| Screen | Key | Contents |
|---|---|---|
| **Dashboard** | `esc` | the screen above — selection, units, merged log |
| **Worlds** | `w` | scanned worlds, size, model count, preview note, `enter` selects |
| **Drones** | `d` | scanned drone models, airframe, rotor count, camera inventory |
| **Cameras** | `c` | the four profiles + per-group live toggles (§11) |
| **Projects** | `p` | scanned C++ projects: built / stale / never built, `b` build, `enter` run, parameter editor |
| **Bags** | `R` | scanned bags: duration, message count, topics, whether it can fly |
| **Record** | `r` | name prompt, scope (wrapper / wrapper-no-camera / all), start/stop |
| **Topics** | `T` | live topic table from walkerd’s node: name, type, Hz, publisher count |
| **Logs** | `l` | full-screen log with per-unit filter and search |
| **Help** | `?` | keymap + the constraint matrix, rendered from the same table the code enforces |

### 5.3 Performance budget

| Operation | Budget | How |
|---|---|---|
| keypress → redraw | 50 ms | all work off-thread |
| scan worlds/models/bags | 200 ms | host-side `os.scandir`, cached with mtime invalidation |
| unit state refresh | 500 ms tick | walkerd pushes deltas; walker never polls docker |
| topic rate table | 1 s tick | walkerd’s rclpy node, not `ros2 topic hz` |
| start simulation → wrapper telemetry flowing | ≤ 45 s cold, ≤ 25 s warm | measured and shown as a progress bar with named steps |

---

## 6. walkerd — the in-container supervisor

Everything that runs inside the container is a **unit**. walkerd owns the unit table; nothing
else starts or stops a process.

### 6.1 Unit definition

```python
@dataclass
class Unit:
    name: str                    # sim, bridge, project, qgc, rviz, record, replay, cameras
    argv: list[str]              # what to exec
    env: dict[str, str]
    cwd: str
    pty: bool = True             # give it a pty so it behaves as if on a terminal
    ready_probe: Callable        # returns True when the unit is actually usable
    ready_timeout: float
    log_path: str                # /var/log/walker/<name>.log, also a 5 000-line ring in RAM
    log_rules: list[LogRule]     # regex → (level, short form) for the walker dashboard
    stop_signal: int = SIGINT    # SIGINT for the recorder (it must finalise the mcap)
    stop_timeout: float = 20.0
    depends_on: list[str]
    conflicts_with: list[str]
```

The **`log_rules`** field is what makes “walker shows major logs of it on itself” cheap: each
unit declares which of its lines matter (PX4 `[commander]`, a project’s step transitions, the
bridge’s route errors, rosbag2’s topic count). Everything else goes to the file and is only
seen in the full log screen or in that unit’s terminal.

### 6.2 Lifecycle

```
  request ──► guard(constraints) ──► spawn(pty) ──► ready_probe ──► RUNNING
                     │                                   │
                     │ refused                           │ timeout
                     ▼                                   ▼
                 REFUSED(reason)                    FAILED(last 20 log lines)

  stop ──► stop_signal ──► wait(stop_timeout) ──► SIGKILL ──► STOPPED(exit code)
```

Two details V1 got right and V2 keeps:

- **`SIGINT` to the recorder, never `SIGTERM`/`SIGKILL`.** rosbag2 writes `metadata.yaml` on
  Ctrl-C; a killed recorder leaves a bag that `ros2 bag play` refuses to open. If the
  metadata is missing anyway, walkerd runs `ros2 bag reindex` before reporting.
- **Ready probes are per-unit and named.** “The simulation is up” means: the `gz sim` process
  is alive **and** `/world/<w>/control` answers **and** `/tmp/px4-sock-0` exists **and**
  `/fmu/out/vehicle_status` is publishing. A failure names the link that is down, because
  from inside a C++ project every one of them looks like “no telemetry”.

### 6.3 The wire protocol

Unix socket at `<repo>/.walker/walkerd.sock`, mounted into the container. Newline-delimited
JSON, request/response plus a server-push event stream.

```jsonc
// walker → walkerd
{"id":7,"op":"start","unit":"project","args":{"package":"demo_orbit_mission",
   "params":{"radius":8.0,"laps":3}}}
{"id":8,"op":"cameras","args":{"payload":false,"fisheye":true}}
{"id":9,"op":"record.start","args":{"name":"tower_pass_2","scope":"wrapper"}}

// walkerd → walker  (responses)
{"id":7,"ok":false,"error":"refused","reason":"replay is running",
   "held_by":"replay","hint":"stop it with R, or press ! to force"}

// walkerd → walker  (events, unsolicited)
{"ev":"unit","unit":"sim","state":"running","since":1790000000.0}
{"ev":"log","unit":"project","level":"info","t":1790000123.4,
   "text":"[ORBIT 14s] status=ON_AIR mode=SDK_CTRL height=1.8 m"}
{"ev":"telemetry","height":1.83,"armed":true,"mode":"SDK_CTRL","batt":0.67}
```

Why a socket and not `docker exec`: a `docker exec` costs ~150 ms of daemon round-trip before
the process even starts, and a `ros2` CLI call inside it costs 4–5 s more. The socket costs
microseconds and lets walkerd push state instead of walker polling for it.

---

## 7. The run-state machine and its constraints

This is the table you described, written once, enforced in `walkerd/constraints.py`, and
rendered verbatim into walker’s help screen and into `README.md` so the three can never drift.

### 7.1 Matrix

Rows = what you are asking to start. Columns = what is already running.

|  start ↓ / running → | **sim** | **project** | **replay** | **record** | **qgc** | **rviz** |
|---|---|---|---|---|---|---|
| **sim**      | — already up | n/a | ⛔ stop the replay first | ✅ (recording continues) | ✅ | ✅ |
| **project**  | ✅ required | ⛔ **one project at a time** | ⛔ **replay owns the surface** | ✅ | ✅ QGC auto-locked² | ✅ |
| **replay**   | ✅ **required**¹ | ⛔ stop the project first | ⛔ one replay at a time | ✅ | ✅ QGC auto-locked² | ✅ |
| **record**   | ✅ | ✅ **explicitly allowed** | ✅ **explicitly allowed** | ⛔ one recorder at a time | ✅ | ✅ |
| **qgc**      | ✅ required | ✅ starts locked² | ✅ starts locked² | ✅ | — | ✅ |
| **rviz**     | ✅ or replay | ✅ | ✅ | ✅ | ✅ | — |

✅ allowed  ⚠️ allowed with a banner  ⛔ refused with a reason and the key that clears it
¹ **D3: every replay requires a running simulation**, including a telemetry-only bag. Stopping
the simulation stops the replay.
² **D4:** QGC and a project/replay may run together, and walker enforces it rather than
warning about it — the MAVLink guard (§16.2) drops QGC's command traffic for as long as
something else holds the flight lock. The dashboard shows `qgc ● running (observer — locked
by project)`.

### 7.2 The rules in words, as you stated them

- **One C++ project at a time.** Starting a second refuses and names the first.
- **A replay needs the simulation.** There is no simulation-less replay in V2 (D3): the bag
  drives the aircraft and the graph the aircraft lives in must exist.
- **A replay locks out projects and other replays.** The bag is publishing the wrapper
  surface; a project publishing setpoints into the same surface, or a second bag, produces a
  graph with two writers on one topic and no way to tell whose message won.
- **Recording is never locked out.** You may start a recording while a project flies, while a
  bag replays, while nothing at all is happening, and stop it at any moment. The only limit
  is one recorder at a time.
- **A recording is named by you**, defaulting to `<context>_<timestamp>`, and lands in
  `bags/<name>/` on the host through the read-write mount.
- **QGC never fights a project.** It may be open at any time; whenever a project or a replay
  holds the flight lock, the guard makes QGC read-only and says so on both screens.

### 7.3 Forced override

`!` before a refused key runs it anyway, after a confirmation that spells out the expected
failure mode (“two publishers on `/wrapper/psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate`;
PX4 will follow whichever arrives last”). Recorded in the log as `FORCED`. This exists because
during debugging you sometimes *want* the broken configuration.

---

## 8. Terminals: how a unit gets its own window

> **You get a terminal per unit for: sim, project, qgc, replay, rviz.** Plus `t` on any unit
> in the dashboard opens one on demand.

### 8.1 The host reality on this machine

Checked on 2026-09-21: GNOME on **Wayland**, and the only terminal emulator installed is
**`ptyxis`**. No `gnome-terminal`, no `xterm`, no `tmux`, no `screen`. So the plan cannot
assume `gnome-terminal -- cmd`.

### 8.2 Launcher chain

`walker/terminal.py` tries, in order, and remembers what worked in `.walker/config.json`:

| Emulator | Invocation |
|---|---|
| `ptyxis` | `ptyxis --new-window --title "<unit>" -- <cmd>` |
| `gnome-terminal` | `gnome-terminal --title=… -- <cmd>` |
| `konsole` | `konsole -p tabtitle=… -e <cmd>` |
| `xfce4-terminal`, `alacritty`, `kitty`, `wezterm`, `foot` | their own forms |
| `x-terminal-emulator` | Debian alternatives |
| `xterm` | last resort |
| **none found** | walker prints the exact command to paste, and offers its own full-screen log view for that unit instead — **the simulation still runs**; only the separate window is missing |

`SIM_TERMINAL="<cmd template>"` in the environment overrides the chain entirely, so a machine
with an unusual setup is one env var away from working.

### 8.3 What runs inside the window

```bash
docker exec -it dotflysim2 walker-attach <unit>
```

`walker-attach` connects to walkerd, attaches to that unit’s PTY, replays the last 200 lines
of its ring buffer so the window is not empty, and then streams live. Closing the window
**detaches**; it does not kill the unit (Ctrl-C inside it does, after a confirmation — that
is the one place where a keystroke in a terminal changes unit state).

Consequences worth stating:
- The unit’s output is in **two** places at once — its window, and walker’s merged log — and
  they are the same bytes.
- Closing every window loses nothing.
- Wayland: the `--privileged` container reaches the display through the mounted
  `/tmp/.X11-unix` socket and `DISPLAY=:0` (XWayland). Gazebo GUI, RViz and QGC are all X11
  clients under XWayland; this already works in V1 and is kept unchanged.

---

## 9. Docker: image, mounts, discovery

### 9.1 Image

Start from V1’s `Dockerfile`, with these edits:

| Edit | Reason |
|---|---|
| Remove the baked `4900_gz_m4e` airframe layers (`FIX LAYER 2`) | airframes are generated at runtime from model manifests (§10.3) |
| Remove `COPY models/… /home/developer/gz_models/…` | models are mounted, not copied |
| Add `walkerd` + `walker-attach` into `/opt/walker/` | the supervisor |
| Set sensor `always_on` policy hook | see §11 / SPIKE-1 |
| Keep Qt5 toolchain only if D6 says keep the RViz panels | otherwise drops a build layer |
| Add `ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | the no-multicast rule, enforced by the middleware itself |
| Replace `config/dds.xml` with a localhost profile | SHM transport + UDPv4 on loopback only; no `builtin` multicast locators |

Tag: `dotflysim2:latest`. Container name: `dotflysim2`.

### 9.2 Container flags

```bash
docker run -d --name dotflysim2 \
  --privileged \
  --shm-size=2g \
  --group-add "$(stat -c '%g' /dev/dri/renderD128)"   # only if it exists
  -e DISPLAY="${DISPLAY:-:0}" \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/developer/ws/config/dds.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ...mounts (below)... \
  dotflysim2:latest sleep infinity
```

**`--network host` is gone.** V1 needed it because DDS multicast does not cross a docker
bridge and the Manifold lived on the LAN. V2’s entire graph is inside one container, so the
default bridge network is correct and is *safer*: no simulation traffic can leak onto your
LAN, and two engineers running V2 on the same office network cannot discover each other’s
drones. QGC’s MAVLink (UDP 14550) is container-internal and unaffected.

**`--shm-size=2g`**, up from V1’s 1 g: Fast DDS allocates a large SHM segment per participant
and V2 runs more of them (sim, bridge, walkerd, camera relay, rviz, project, recorder,
replayer). Under-sizing this produces `RTPS_TRANSPORT_SHM Error: Failed to create segment`
and silent discovery failures — the single most confusing failure mode in V1.

### 9.3 Mounts

| Host | Container | Mode | Why |
|---|---|---|---|
| `models/` | `/home/developer/gz_models` | rw | scanned; edits apply without a rebuild |
| `worlds/` | `/home/developer/gz_worlds` | rw | same |
| `projects/` | `/home/developer/ws/src/projects` | rw | colcon finds C++ projects; a project may write logs next to itself |
| `bridge/` | `/home/developer/ws/src/bridge` | rw | simty, live-editable |
| `tools/` | `/home/developer/ws/src/tools` | rw | composer, probes |
| `config/` | `/home/developer/ws/config` | rw | dds.xml, bridge.yaml, rviz configs |
| `bags/` | `/home/developer/bags` | **rw** | record writes here directly; replay reads here. (V1 mounted it ro and had to copy bags out of the container — that whole `RECORDING_SPILLED` code path disappears.) |
| `.walker/` | `/run/walker` | rw | the socket, pid files, unit state |

Everything the operator can add — a world, a model, a project, a bag — is a **mounted
directory**, so “add” means “put a folder there and press `F5`”.

---

## 10. Scanning: worlds, drone models, bags

### 10.1 When

- On walker start.
- On `F5`.
- On opening the Worlds / Drones / Projects / Bags screen (cached, mtime-invalidated).

Scanning is host-side (`os.scandir` on the mounted directories), so it costs nothing and
works before the container is even up.

### 10.2 What makes something selectable

**A world** = a `.sdf` in `worlds/`, optionally with a sibling `<name>.walker.yaml`:

```yaml
# worlds/powerline.walker.yaml
name: powerline
title: 220 kV powerline corridor
description: 5 towers, 4 spans, 800 m
spawn:            { lat: 24.867412, lon: 67.057352, alt: 8 }   # PX4_HOME_*
gz_world_name: default          # the <world name="..."> inside the sdf
requires_models: [hv_tower_220kv, hv_span_220kv]
heavy: true                     # walker warns on a GPU-less machine
preview: preview/corridor.png
```

Without the manifest, walker infers: name from the filename, `gz_world_name` by grepping
`<world name=`, spawn from PX4 defaults. **A bare `.sdf` still works** — the manifest only
adds polish.

**A drone model** = a directory in `models/` with `model.config` + `model.sdf`, plus
`walker.yaml` if it is *flyable*:

```yaml
# models/m4e/walker.yaml
kind: drone                     # drone | scenery
title: DJI Matrice 4E
airframe:
  id: 4900                      # unique per model; walker checks for collisions
  type: quadrotor
  params:                       # verbatim into the generated PX4 airframe file
    CA_ROTOR_COUNT: 4
    CA_ROTOR0_PX: 0.11551
    # ... (the full set from V1's 4900_gz_m4e)
    MPC_THR_HOVER: 0.60
    NAV_DLL_ACT: 0
    MPC_XY_VEL_MAX: 21.0
    MIS_TAKEOFF_ALT: 1.8        # the demos are written against this
attachments:
  - model: m4e_camera           # the payload/gimbal model spawned with it
    pose: [0, 0, -0.06, 0, 0, 0]
cameras:
  fisheye: [perception_front_left, perception_front_right]
  payload:
    wide:        [m4e_wide, m4e_wide_video4k, m4e_wide_videofhd, m4e_wide_preview]
    medium_tele: [...]
    tele:        [...]
  main_stream_default: wide/videofhd     # what feeds /wrapper/…/main_camera_stream
lrf: { sensor: lrf, topic: /drone/lrf/range }
```

A directory without `walker.yaml`, or with `kind: scenery`, appears under **Scenery** and is
mounted for worlds to reference but cannot be selected as the aircraft.

**A bag** = a directory under `bags/` (one level of nesting allowed) containing
`metadata.yaml` + `*.mcap`. Walker reads `metadata.yaml` directly — no `ros2 bag info`
subprocess — for duration, message count, topic list, and whether it holds any of the five
command topics (which is what decides *fly-back* vs *passive*).

### 10.3 Runtime airframe generation — the key enabler

V1 bakes `4900_gz_m4e` into the image at build time, which is why the drone model is not
selectable. V2 generates it at simulation start:

```
models/<drone>/walker.yaml
        │
        ▼  tools/compose_sim.py --drone <drone> --world <world> --cameras <profile>
  /home/developer/gz_runtime/
        ├─ <drone>/model.sdf            camera sensors per the chosen profile
        ├─ <attachments>/model.sdf
        ├─ bridge.yaml                  ros_gz_bridge routes for exactly these sensors
        └─ airframe/<id>_gz_<drone>     generated from walker.yaml `params`
                │
                ▼  copied into  ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/
                   then          PX4_SIM_MODEL=gz_<drone> PX4_GZ_WORLD=<gz_world_name> ./bin/px4
```

PX4 SITL resolves `PX4_SIM_MODEL` against the **runtime** `etc/init.d-posix/airframes`
directory in the build tree, not against compiled-in data, so a new airframe needs no PX4
rebuild. **This is SPIKE-2 in §21** — it is the single assumption that makes drone selection
possible, and it gets verified before anything else is built on it.

`gz_runtime/` is regenerated on every start and is always safe to delete. `models/` and
`worlds/` are never modified.

---

## 11. The camera system

### 11.1 The four boot profiles

Exactly as you specified, chosen on the Cameras screen before the simulation starts:

| Profile | Fisheye pair | Payload lenses | Render cost | Use it for |
|---|---|---|---|---|
| **none** | off | off | 0 MP/s | flight-dynamics work, missions that never look at a picture |
| **fisheye** | on | off | ~20 MP/s | obstacle avoidance, stereo perception, SLAM |
| **payload** | off | on | ~1074 MP/s | gimbal/inspection work, QGC video |
| **all** | on | on | ~1094 MP/s | everything, on a machine with a real GPU |

The numbers are V1’s measurements for the m4e (12 payload + 2 fisheye sensors); for another
model they are computed from its `walker.yaml` sensor list and shown before you commit.

### 11.2 Live on/off during a flight — how

This is the part V1 could not do (`apply_camera_choice` literally prints “restart it for this
choice to take effect”), and the reason is precise and fixable:

> V1’s SDF sensors are `<always_on>true</always_on>`. gz-sim renders an always-on sensor
> whether or not anyone is subscribed, so `lazy: true` in `bridge.yaml` saves nothing, and the
> only way to stop the rendering cost is to remove the sensor from the SDF — which means a
> restart.

**V2 sets `<always_on>false</always_on>` on every camera and lidar sensor** (composer rewrites
this, sources untouched). gz-sim’s `Sensors` system then updates a rendering sensor only while
it has gz-transport connections. That gives a demand-driven chain:

```
  walker toggles group ON
        │
        ▼
  camera relay node subscribes to /drone/camera/<lens>/<tier>/image_raw
        │
        ▼
  ros_gz_bridge (lazy:true) creates its gz-transport subscription
        │
        ▼
  gz-sim sees a connection → starts rendering that sensor      ← cost appears here
        │
        ▼
  frames flow → /wrapper/psdk_ros2/main_camera_stream (and RViz)
```

Toggling off reverses it and the GPU cost disappears within one sensor period. **No restart.**

The four boot profiles then mean “which groups the relay subscribes to at start-up”, and the
per-group toggles are live for the whole session. The IMU / barometer / magnetometer / navsat
sensors stay `always_on` — PX4’s EKF needs them continuously — as does the LRF, for the same
reason V1 gives.

**Fallback if SPIKE-1 fails** (i.e. gz Harmonic renders `always_on:false` sensors anyway):
keep V1’s compose-time stripping as the boot profile, and restrict live toggling to *streaming*
of the groups that were composed in. Walker then greys out the groups that need a restart and
says so. The feature degrades; it does not disappear.

### 11.3 What “camera on” publishes

| Group | Gazebo topic | Wrapper topic | Notes |
|---|---|---|---|
| fisheye left | `/drone/perception/front/left/image_raw` | `/wrapper/psdk_ros2/perception_stereo_left_stream` | mono8 704×704 @20 Hz; gated by the `start_perception` service, as on the real aircraft |
| fisheye right | `/drone/perception/front/right/image_raw` | `…/perception_stereo_right_stream` | + `camera_info` siblings |
| payload (selected lens/tier) | `/drone/camera/<lens>/<tier>/image_raw` | `…/main_camera_stream` | rgb8 1440×1080 @~11 Hz to match the PDF |
| payload (preview tier) | `/drone/camera/<lens>/preview/image_raw` | `…/fpv_camera_stream` | the PDF marks this dead on the M4E; V2 publishes it because the simulator can |

Lens (`wide` / `medium_tele` / `tele`) and tier (`video4k` / `videofhd` / `preview`) are
selected live from the Cameras screen and through `camera_set_stream_source` /
`camera_setup_streaming`, so a C++ project can change them the same way it would on hardware.

---

## 12. The bridge (simty v2)

### 12.1 What it is

`ROS_Bridge_Simty` is the only thing that translates between the DJI PSDK interface and
PX4/Gazebo. In V2 it is **a unit that comes up with the simulation and stays up** — not a mode,
not something a project starts. If the simulation is running, the wrapper surface exists.

### 12.2 What changes

| Module | V1 | V2 |
|---|---|---|
| `converters.py` (1 995 lines) | keep, unchanged | the message translation, and it is correct |
| `registry.py` (1 988 lines) | keep; strip PROXY role | `SERVE` only — there is no remote wrapper to proxy to |
| `node.py` (2 134 lines) | keep; remove discovery hooks | |
| `clock.py`, `qos.py`, `state.py`, `journal.py`, `converters` | keep | |
| `tui.py` (2 042 lines) | the interactive bridge console | **removed**; walker is the console. Its control verbs move to the walkerd socket |
| `control.py` | `/simty/control` String verbs | keep as the mechanism; walker drives it through walkerd |
| `discovery.py`, `remote.py`, `provision.py`, `mock.py`, `standin.py` | find/start/fake a Manifold | **removed** |
| `viz.py`, `diagnostics.py` | RViz marker boards, `/simty/diagnostics` | keep `diagnostics`; `viz` optional (it is what makes the RViz “boards” layout useful) |

Net: roughly 6 000 of ~14 000 lines removed, with no loss of simulated behaviour.

### 12.3 Bridge modes

Three, and walkerd switches between them — never the operator, never a project:

| Mode | When | What it does |
|---|---|---|
| **normal** | simulation up, nothing special | all routes live, all services `SERVE` |
| **project** | a C++ project is running | selects the setpoint route the project declared (`ENUvelocity` / `ENUposition` / `FLUvelocity`), raises its rate cap, and makes sure the offboard path is armed for use |
| **replay** | a bag is playing | every telemetry route **off** so the bag owns the wrapper surface; services still `SERVE` so a caller gets an answer; snapshot is restored on exit |

V1 discovered the hard way that two writers on one wrapper topic is the failure worth a whole
mode; that lesson is preserved.

### 12.4 Video routes stay opt-in

Every video route ships disabled. An uncompressed 1080p frame is ~6 MB; enabling two at once
starves control and telemetry. A project that wants a picture asks for one route, at a rate
cap, and walker shows which routes are open and what they cost.

---

## 13. The PSDK / wrapper surface

All names below are under `/wrapper/psdk_ros2/`. Sourced from
`psdk_ros2_topics_services.pdf`. Image and perception streams use **best-effort** QoS
(`SensorDataQoS`) — in RViz set *Reliability = Best Effort* or you will see nothing.

### 13.1 Telemetry topics (41)

| Group | Topics |
|---|---|
| **Attitude & rates** | `attitude` (QuaternionStamped, 50 Hz), `angular_rate_body_raw`, `angular_rate_ground_fused` (Vector3Stamped, 50 Hz), `imu` (Imu, 50 Hz) |
| **Acceleration** | `acceleration_body_fused`, `acceleration_body_raw`, `acceleration_ground_fused` (AccelStamped, 50 Hz) |
| **Position & velocity** | `position_fused` (PositionFused, 50 Hz), `visual_odometry` (Odometry, 50 Hz), `velocity_ground_fused` (Vector3Stamped, 50 Hz), `gps_velocity` (TwistStamped, 25 Hz) |
| **Altitude** | `altitude_barometric`, `altitude_sea_level` (Float32, 70 Hz), `height_above_ground` (Float32, 25 Hz) |
| **GNSS / RTK** | `gps_position`, `gps_position_fused` (NavSatFix), `gps_details` (GPSDetails), `gps_signal_level` (UInt8), `rtk_position`, `rtk_velocity`, `rtk_yaw`, `rtk_yaw_info`, `rtk_position_info`, `rtk_connection_status` (5 Hz) |
| **Home point** | `home_point` (NavSatFix), `home_point_status` (Bool) |
| **Flight state** | `flight_status` (FlightStatus), `display_mode` (DisplayMode), `control_mode` (ControlMode), `flight_anomaly` (FlightAnomaly), `motor_start_error` (UInt16) |
| **Power** | `single_battery_index1`, `single_battery_index2` (SingleBatteryInfo, 21 Hz), `esc_data` (EscData, 21 Hz) |
| **Sensing** | `magnetic_field` (MagneticField, 50 Hz), `relative_obstacle_info` (RelativeObstacleInfo, 25 Hz) |
| **RC** | `rc` (Joy, 25 Hz), `rc_connection_status` (RCConnectionStatus, 25 Hz) |
| **Vision** | `main_camera_stream` (Image rgb8 1440×1080, 11 Hz), `perception_stereo_left_stream`, `perception_stereo_right_stream` (Image mono8 704×704, ~18 Hz) |

### 13.2 Command topics (5) — you publish these

All `sensor_msgs/msg/Joy`. **Stream continuously at ≥ 50 Hz** (20 Hz works in practice and is
what the demos use; below that PX4 drops out of offboard). Preconditions:
`obtain_ctrl_authority` **and** airborne.

| Topic | `axes` |
|---|---|
| `flight_control_setpoint_ENUvelocity_yawrate` | `[v_east, v_north, v_up, yaw_rate]` m/s, m/s, m/s, **deg/s** |
| `flight_control_setpoint_ENUposition_yaw` | `[east, north, up, yaw]` m, m, m, deg |
| `flight_control_setpoint_FLUvelocity_yawrate` | body frame: `[v_forward, v_left, v_up, yaw_rate]` |
| `flight_control_setpoint_rollpitch_yawrate_thrust` | attitude-rate control |
| `flight_control_setpoint_generic` | raw PSDK flight-control flag byte + 4 axes |

> ⚠️ **The yaw-rate unit trap, carried over from V1.** The psdk_ros2 documentation table says
> radians/s. The converter that actually reads these messages
> (`psdk_enu_velocity_to_setpoint` in `converters.py`) takes **degrees/s**. Believe the code.
> `ROS2_ARCHITECTURE.md` will state this in bold, and V2 will additionally publish the unit in
> the route’s `notes` field so walker can display it.

### 13.3 Topics with no data on a real M4E (8)

`battery`, `fpv_camera_stream`, `gimbal_angles`, `gimbal_status`, `gps_control_level`,
`home_point_altitude`, `landing_gear_status`, `perception_camera_parameters`.

**V2 policy:** the simulator *can* source several of these (the gimbal is real in Gazebo, the
preview tier is a real camera). They are published, and every one of them is **flagged in
walker and in `ROS2_ARCHITECTURE.md` as “simulation-only — a real M4E does not publish this”**,
so a project cannot accidentally depend on something the aircraft will not give it.

### 13.4 Services (56)

| Group | Count | Services |
|---|---|---|
| **Flight** | 11 | `takeoff`, `land`, `start_go_home`, `cancel_go_home`, `cancel_landing`, `start_confirm_landing`, `start_force_landing`, `turn_on_motors`, `turn_off_motors`, `obtain_ctrl_authority`, `release_ctrl_authority` |
| **Home & reference** | 5 | `set_home_from_gps`, `set_home_from_current_location`, `set_go_home_altitude`, `get_go_home_altitude`, `set_local_position_ref` |
| **Obstacle avoidance** | 10 | `{set,get}_{horizontal_radar,horizontal_vo,upwards_radar,upwards_vo,downwards_vo}_obstacle_avoidance` |
| **Perception** | 1 | `start_perception` (`PerceptionStereoVisionSetup`) — gates the stereo streams |
| **Camera — capture** | 5 | `camera_shoot_single_photo`, `camera_shoot_burst_photo`, `camera_shoot_interval_photo`, `camera_stop_shoot_photo`, `camera_record_video` |
| **Camera — optics** | 14 | `{set,get}_{aperture,iso,shutter_speed,exposure_mode_ev,focus_mode,focus_target,focus_ring_value}` |
| **Camera — zoom & focus range** | 4 | `camera_set_optical_zoom`, `camera_get_optical_zoom`, `camera_set_infrared_zoom`, `camera_get_focus_ring_range` |
| **Camera — media & streaming** | 6 | `camera_setup_streaming`, `camera_get_type`, `camera_get_laser_ranging_info`, `camera_get_sd_storage_info`, `camera_format_sd_card`, `camera_get_file_list_info` |

Plus the two file actions (`CameraDownloadFileByIndex`, `CameraDeleteFileByIndex`) that
`psdk_interfaces` already carries.

**Delta against V1’s registry, to be resolved during M4:** V1 also serves
`camera_set_stream_source`, `gimbal_set_mode`, `gimbal_reset` and the `gimbal_rotation`
command topic, which the PDF does not list. These are kept and marked *extension* — useful in
simulation, absent on this aircraft.

**Every served service gets ROS 2 service introspection turned on**, because rosbag2 records a
service call through its `<service>/_service_event` topic. Without it a bag can replay a
sortie’s setpoints but never the `takeoff` that started it — V1 learned this and V2 keeps the
fix.

---

## 14. RViz

RViz is a unit like any other: `v` starts it, in its own window, with a config chosen for the
context. It is **never started automatically** — it is expensive and most runs do not need it.

### 14.1 Camera viewing — the workflow you described

```
  Cameras screen  →  enable the group you want          (c, then space on the group)
         │
         ▼
  Dashboard       →  start RViz                          (v)
         │
         ▼
  RViz            →  the Image displays for the enabled cameras are already in the config;
                     the disabled ones show "no messages received" and cost nothing
```

Walker’s Cameras screen shows, per group, **whether anything is subscribed** — so “I enabled
it and RViz shows nothing” is answered on the screen rather than by guessing.

### 14.2 Shipped configs

| Config | Shows |
|---|---|
| `flight.rviz` | TF tree, odometry trail, the aircraft model, `relative_obstacle_info` markers, LRF range |
| `cameras.rviz` | an Image display per camera group (`main_camera_stream`, both `perception_stereo_*`, `fpv_camera_stream`), **all with Reliability = Best Effort** |
| `perception.rviz` | the stereo pair + `camera_info` + point cloud, for perception work |
| `replay.rviz` | what a bag typically contains |

### 14.3 The walker RViz panel — re-implemented (D6)

V1's `gui/rviz_panels/` is deleted. It re-implemented much of the bridge console inside Qt
widgets, each panel doing its own ROS subscriptions and its own formatting, and it dragged a
Qt5 widget toolchain into every image build. The replacement is one small plugin package built
on a deliberately cheap design:

```
   walkerd ──── /walker/state  (std_msgs/String, one JSON doc, 5 Hz, latched)
                      │
                      ▼
            ┌──────────────────────┐
            │  WalkerPanel (Qt)    │   ONE subscription, ONE publisher.
            │  ├ flight state      │   Repaints from a cached struct on a 5 Hz
            │  ├ height / battery  │   QTimer — never from the ROS callback thread.
            │  ├ camera groups     │   No per-frame allocation, no widget rebuild.
            │  └ [takeoff][land]   │
            │    [E-STOP] [cams]   │
            └──────────┬───────────┘
                       │
                  /walker/control  (std_msgs/String, verb per message)
                       │
                       ▼
                    walkerd ── applies it through the same path walker's keys use
```

**Why this is efficient where V1's was not:**

| V1 panels | V2 panel |
|---|---|
| N panels × M subscriptions each, every panel parsing raw wrapper messages | 1 subscription to one pre-digested JSON state message |
| repaint driven by ROS callbacks (at 50 Hz telemetry rates, on the ROS thread) | repaint on a 5 Hz Qt timer from a cached struct; ROS callback only swaps a pointer |
| each panel owned its own service clients and its own notion of state | every action is one verb on `/walker/control`; walkerd is the only thing that acts |
| Qt widget trees rebuilt on state change | fixed widget tree, values only |
| a Qt5 toolchain layer in the Dockerfile for the panels alone | one package, built with the same colcon pass as everything else |

Scope is deliberately small: **observe, plus the four actions you want without leaving RViz**
(takeoff, land, E-STOP, camera group toggle). Anything more belongs in walker, which is where
the keyboard already is. The panel is optional — RViz runs fine without it, and the `flight`
and `cameras` configs each load it in a docked side pane.

### 14.4 Per-project configs

A C++ project may declare its own config in `project.conf` (`RVIZ=rviz/mine.rviz`) and walker
offers it when that project runs — V1’s `demo_camera_track` and `demo_gnss_stereo_inertial`
both do this and both are ported.

---

## 15. rosbag: record and replay

### 15.1 Recording

- **Start/stop at any time**, from anywhere in walker, with `r`.
- **You name it.** Walker prompts, defaults to `<context>_<YYYY-MM-DD_HH-MM-SS>` where context
  is the running project name, `replay`, `qgc` or `idle`. Invalid characters are rejected at
  the prompt, not after the flight.
- **Scope**, chosen at start:

  | Scope | Filter | Typical size |
  |---|---|---|
  | `wrapper` *(default)* | `--regex '^/wrapper/psdk_ros2/'` + all wrapper service-event topics | ~2 MB/min without cameras |
  | `wrapper-nocam` | as above, minus image topics | ~1.5 MB/min |
  | `all` | `--all` | large; includes `/fmu/*` and raw gazebo topics |

- Writes **directly** to `bags/<name>/` through the rw mount. No copy-out step.
- Closed with **SIGINT**, then `metadata.yaml` is verified, then `ros2 bag reindex` if it is
  missing, then walker reports message count, duration and whether the bag can fly one back.
- Storage: **mcap**.
- Live in the dashboard: elapsed time, size on disk, messages/s, and a warning when the host
  filesystem drops below 5 GB free.

### 15.2 Replay

Walker’s Bags screen lists every bag under `bags/` (one nesting level), reading
`metadata.yaml` directly:

```
  name                      duration   msgs     topics  cmds  can fly
  tower_pass_2               4:12      301 442     44     1    yes  (ENUvelocity)
  arm_takeoff_2026-09-14     0:58       71 004     41     0    no   (passive only)
```

**The simulation must be running for any replay (D3).** Walker refuses `R` with
`"replay needs the simulation — press s to start it"` when `sim` is down, and stopping the
simulation stops an in-flight replay rather than leaving a bag publishing into a dead graph.
V1's simulation-less replay path is gone.

“cmds” is how many of the five command topics the bag holds — that is what decides *how* it
replays, not *whether* it needs the simulation:

| Kind | When | What happens |
|---|---|---|
| **fly-back** | the bag holds ≥1 command topic | bridge → replay mode for telemetry, command topics routed **into** the bridge; walker takes authority and the aircraft re-flies the sortie in the live simulation |
| **telemetry-only** | the bag holds none | bridge → replay mode, every route off, the bag owns the wrapper surface; the simulation is up and idle underneath, and whatever reads that surface (RViz, a perception node, a recorder) sees the recorded flight. Walker says plainly that the aircraft will not move. |

Replay runs in **its own terminal** so `ros2 bag play`’s progress line is readable, with
pause/step (`space`/`s`) available there and rate control from walker.

While a replay is running: **no C++ project, no second replay** (§7). Recording is allowed —
re-recording a replay into a filtered bag is a legitimate and useful thing to do.

---

## 16. QGroundControl

### 16.1 How it connects

- Runs in the container, X11 to the host display, its own window (it is a GUI, so “its own
  terminal” means its own window plus a log pane in walker).
- MAVLink over UDP 14550 inside the container; PX4’s second MAVLink instance targets
  `127.0.0.1:14550` explicitly, because Linux loopback does not forward the broadcast QGC
  normally auto-discovers on. (V1’s `.post` airframe fragment; V2 generates the same lines.)
- **Failsafe parameter handling**, carried over: `NAV_DLL_ACT=0` when there is no GCS (C++
  projects, replay), `NAV_DLL_ACT=1` while QGC is connected, back to `0` when QGC exits.
  walkerd owns this transition so it can never be left in the wrong state by a crash.
- Video: QGC receives one stream on UDP 5600, selected through `/drone/camera/gst_select`, and
  walker’s Cameras screen is where you pick which lens QGC sees.

### 16.2 The MAVLink guard — QGC cannot intervene (D4)

You asked for a lock rather than a warning, and there is a clean place to put one. PX4 already
talks to QGC over a dedicated MAVLink instance on a known pair of UDP ports (V1's airframe
`.post` fragment: `mavlink start -x -u 14541 -r 4000000 -t 127.0.0.1 -o 14550 -f`). V2 puts a
~200-line UDP relay in that gap:

```
            ── telemetry, always ──────────────────────────────────────►
   PX4 :14541 ────────────► mavguard :14551 ────────────► QGC :14550
            ◄──────────── commands, only when unlocked ────────────────
                               │
                               │ drops, counts and logs while locked:
                               │   COMMAND_LONG / COMMAND_INT
                               │   SET_MODE
                               │   MANUAL_CONTROL / RC_CHANNELS_OVERRIDE
                               │   SET_POSITION_TARGET_* / SET_ATTITUDE_TARGET
                               │   MISSION_* (write side)
                               │   PARAM_SET
                               ▼
                           walkerd  ── owns the lock, shows the drop count
```

**Semantics**

| Flight lock held by | QGC can | QGC cannot |
|---|---|---|
| nobody | everything — fly it manually, upload missions, set parameters | — |
| a C++ project | see every instrument, watch the mission, view video | arm, disarm, change mode, take manual control, upload a mission, set a parameter |
| a replay | same | same |

- The lock is taken and released by **walkerd**, at exactly the moments it starts and stops a
  project or a replay. It is never a manual step and cannot be left stuck: if the holder dies,
  the lock dies with its unit.
- **Telemetry is never filtered.** A locked QGC is a full instrument panel; that is the point.
- **Every blocked message is counted and logged** (`qgc blocked SET_MODE ×3`), so an operator
  jabbing at a greyed-out control gets an explanation on walker's dashboard instead of silence.
- `!` force-unlock exists for debugging, with the same confirmation any other forced action
  gets (§7.3).
- **Fallback if the relay proves awkward** (e.g. a MAVLink v2 signing or sequence-number
  problem): fall back to PX4-side parameter enforcement — `COM_RC_IN_MODE=4` to disable stick
  input plus refusing QGC's mode changes at the commander — and downgrade D4 to the warning
  banner. The relay is tried first because it is the only option that is total.

Implementation note: this is plain `socket` + a MAVLink v1/v2 frame parser (message ID is at a
fixed offset; no payload parsing needed to filter by ID), so it adds **no dependency** — it
does not need pymavlink.

---

## 17. The C++ project contract

`demo_orbit_mission` is the tested reference. Every project follows it. This section is the
skeleton of `CPP_DESIGN.md`.

### 17.1 Hard rules

1. **PSDK surface only.** Subscribe and publish `/wrapper/psdk_ros2/*`; call
   `/wrapper/psdk_ros2/*` services. Never `/fmu/*`, never `/drone/*`, never a `gz` service.
   *This is what makes the binary run on the real aircraft unchanged.*
2. **Authority, then takeoff. Never arm.** Call `obtain_ctrl_authority`, wait ~2 s, call
   `takeoff`. **Do not call `turn_on_motors`** — `takeoff` arms the aircraft itself, and
   calling `turn_on_motors` leaves a drone spinning on the ground with nothing to do. Release
   authority when the mission ends.
3. **Send no setpoints during the automatic takeoff.** Publishing a velocity setpoint mid-climb
   switches PX4 to offboard and cuts the automatic takeoff short. Wait until
   `display_mode != DISPLAY_MODE_AUTO_TAKEOFF` and `height_above_ground` has settled.
4. **The automatic takeoff does not reach your altitude.** `takeoff` is a `Trigger`; it carries
   no altitude, and it stops at `MIS_TAKEOFF_ALT` (1.8 m here, ~1.18 m in practice). Every
   metre after that you fly yourself, with a `CLIMB` step.
5. **Never block in a callback.** Use `async_send_request` with a completion lambda. A
   `spin_until_future_complete` inside a timer callback deadlocks on the single-threaded
   executor — the answer arrives on the thread that is waiting for it.
6. **Stream setpoints at a fixed tick, 20 Hz minimum.** PX4 leaves offboard when they stop.
   Count ticks, not seconds.
7. **Best-effort QoS on telemetry, reliable on commands.**
8. **A step machine, not a sequence of sleeps.** `enum Step` + `goToStep()` + a per-step
   timeout that lands the aircraft rather than hanging. Every step logs once a second.
9. **Land, then release authority.** Send zero velocity for ~2 s first — landing while still
   sliding is how an aircraft tips over on touchdown — then stop publishing entirely so
   offboard does not fight the landing.

### 17.2 The template

```
projects/<my_project>/
├── package.xml            <name> must equal the directory name in V2 (V1 allowed drift)
├── CMakeLists.txt         exactly one add_executable
├── project.conf           launcher settings, read as data, never sourced
├── src/main.cpp
└── rviz/<optional>.rviz
```

`project.conf` keys:

| Key | Values | Meaning |
|---|---|---|
| `CAMERAS` | `none` `fisheye` `payload` `all` | the boot profile walker suggests for this project |
| `SETPOINT` | `velocity` `position` `body_velocity` | which command topic the bridge should prioritise |
| `RVIZ` | path | config to offer when this project runs |
| `VIDEO_ROUTE` / `VIDEO_RATE` / `CAMERA_LENS` / `CAMERA_MODE` | | if the project reads pictures |
| `TAKEOFF_ALT` | metres | checked against the airframe’s `MIS_TAKEOFF_ALT` |
| `TIMEOUT` | seconds | walker stops the project and reports if it overruns |

### 17.3 The seven-step skeleton

```cpp
enum Step { WAIT_FOR_DATA, GET_AUTHORITY, TAKEOFF, CLIMB, /* YOUR MISSION */, LAND, DONE };
```

`CPP_DESIGN.md` walks through each step with the code from `demo_orbit_mission`, explains why
each guard exists, and then shows the three shipped variants:

| Project | Adds |
|---|---|
| `demo_arm_takeoff` | the skeleton alone — takeoff, hold, land |
| `demo_orbit_mission` | an open-loop circle on ENU velocity **(the reference)** |
| `demo_square_mission` | waypoints on ENU position |
| `demo_camera_track` | body-frame FLU velocity + reads `main_camera_stream` |

### 17.4 Preflight, before a project is allowed to start

A project that cannot reach the wrapper surface sits in `WAIT_FOR_DATA` forever with nothing
saying why. So walkerd tests the chain by name first and refuses with the failing link:

```
  [ ok ] container                 dotflysim2, up 14m
  [ ok ] px4                       /tmp/px4-sock-0, armed=no
  [ ok ] xrce agent                /fmu/out/vehicle_status @ 5.1 Hz
  [ ok ] ros_gz_bridge             /clock @ 250 Hz
  [ ok ] bridge                    46 routes, mode=project(velocity)
  [ ok ] wrapper telemetry         flight_status @ 24.9 Hz, height_above_ground @ 25.2 Hz
  [ ok ] wrapper services          takeoff, land, obtain_ctrl_authority, release_ctrl_authority
  [ ok ] position estimate         healthy for 3.2 s
  [ ok ] failsafe                  NAV_DLL_ACT=0 (no GCS required)
  [ ok ] project                   demo_orbit_mission, built 2m ago, not stale
```

---

## 18. Repository layout

```
dotFlySim2/
├── walker                        launcher (bash): finds a Python, execs walkerui
├── README.md                     ← deliverable
├── CPP_DESIGN.md                 ← deliverable
├── ROS2_ARCHITECTURE.md          ← deliverable
├── VERSION2_PLAN.md              this file
├── Dockerfile
│
├── walkerui/                     HOST side (Python, stdlib only)
│   │                             NOTE: named walkerui, not walker, because a
│   │                             launcher file named `walker` and a package
│   │                             directory named `walker/` cannot coexist.
│   │                             The three names: ./walker (launcher),
│   │                             walkerui/ (host), walkerd/ (container).
│   ├── __main__.py               entry point
│   ├── __main__.py               CLI: up / down / status / doctor / shell, and the TUI
│   ├── app.py                    event loop, screen stack                     [M2]
│   ├── screens/                  dashboard, worlds, drones, cameras, bags     [M2]
│   ├── client.py                 walkerd socket client                        [M2]
│   ├── paths.py                  repo layout + THE mount table (single source of truth)
│   ├── dockerctl.py              engine detection, container lifecycle, exec
│   ├── doctor.py                 every readiness check, each with a fix
│   ├── scan.py                   worlds, models, bags, projects               [M3]
│   ├── terminal.py               emulator detection + spawn (§8)
│   └── ui.py                     colours and status marks
│
├── walkerd/                      CONTAINER side (Python + rclpy)
│   ├── __main__.py
│   ├── units.py                  unit table + PTY supervision
│   ├── constraints.py            THE matrix from §7, single source of truth
│   ├── probes.py                 ready probes / preflight (from tools/preflight_probe.py)
│   ├── rosnode.py                one long-lived rclpy node: topic rates, telemetry snapshot
│   ├── server.py                 JSON-lines unix socket
│   ├── mavguard.py               MAVLink relay: the QGC intervene lock (§16.2)
│   ├── flightlock.py             who holds control; drives mavguard + the bridge mode
│   └── attach.py                 walker-attach
│
├── bridge/                       the PSDK bridge (simty, trimmed)
│   ├── ROS_Bridge_Simty.py
│   └── simty/                    converters, registry, node, clock, qos, state, journal,
│                                 control, diagnostics, viz
│
├── walker_rviz_panel/            NEW, small: one Qt panel on /walker/state + /walker/control
│   ├── CMakeLists.txt  package.xml  plugin_description.xml
│   └── src/walker_panel.cpp
│
├── psdk_interfaces/              msgs / srvs / actions (unchanged from V1)
│
├── projects/                     C++ projects — scanned, mounted, built with colcon
│   ├── demo_arm_takeoff/
│   ├── demo_orbit_mission/       ← the reference
│   ├── demo_square_mission/
│   └── demo_camera_track/
│
├── models/                       scanned + mounted; each flyable one has walker.yaml
│   ├── m4e/ m4e_camera/
│   └── hv_tower_220kv/ hv_span_220kv/ transmission_tower/     (kind: scenery)
│
├── worlds/                       scanned + mounted
│   ├── powerline.sdf + powerline.walker.yaml
│   └── preview/
│
├── bags/                         scanned + mounted rw
├── config/                       dds.xml (localhost), bridge.yaml template, rviz/
├── tools/                        compose_sim.py, gen_powerline.py, probes
├── px4-patches/                  server.config, GstCameraSystem
└── .walker/                      runtime: socket, config.json, unit state   (gitignored)
```

---

## 19. Documentation deliverables

Three files, written last (M7) so they describe what was actually built, but outlined now so
the code is written to be describable.

### `README.md`

1. What this simulation is — one page, one diagram, the honest scope (what is simulated
   faithfully, what is approximated, what is absent).
2. **Quick start** — prerequisites, `docker build`, `./walker`, and a first flight in under ten
   commands, with what each screen looks like at each step.
3. **Walker, feature by feature** — every screen, every key, with a screenshot-style block.
4. The constraint matrix (§7), rendered from the same source as the code.
5. Choosing a world, a drone, a camera profile — and what each costs.
6. Recording and replaying a flight.
7. Troubleshooting: the ten failures that actually happen, each with the symptom, the cause and
   the fix. (Shared-memory segment errors, no `/dev/dri`, XWayland permission, stale colcon
   build, a bag with no command topics, PX4 refusing to arm, RViz showing nothing because of
   reliable-vs-best-effort QoS.)
8. Adding a world / a model / a project — the three “drop a folder in” recipes.

### `CPP_DESIGN.md`

1. Why the PSDK surface is the only API, and what it buys you (the same binary flies the real
   M4E).
2. The nine hard rules from §17.1, each with the failure it prevents.
3. The project skeleton, file by file, with `project.conf` documented key by key.
4. `demo_orbit_mission` walked through end to end — the reference implementation, annotated.
5. The full command-topic reference: axes, frames, **units** (including the degrees/radians
   trap), rates, preconditions.
6. The service reference a mission actually needs: authority, takeoff, land, go-home, camera.
7. Telemetry: what to trust, what is simulation-only, what a real M4E does not publish.
8. Building and running: colcon, the workspace layout, what “stale” means, how walker rebuilds.
9. Debugging a mission: the preflight report, the bridge’s route table, `ros2 topic hz`, RViz,
   recording a bag and replaying it into your own node.
10. Porting to hardware: the checklist of everything that differs between this simulator and a
    Manifold 3 running the real wrapper.

### `ROS2_ARCHITECTURE.md`

1. The node graph — every node, what it publishes, what it subscribes to, drawn as ASCII and as
   a Mermaid diagram.
2. The full topic table: all 41 telemetry topics + 5 command topics + the 8 no-data topics, with
   type, rate, frame_id, QoS and **what the simulator sources each one from**.
3. The full service table: all 56, with type, what it does, what it does in *this* simulator,
   and its preconditions.
4. The translation layer: for each bridged route, `PX4/Gazebo topic → converter → wrapper
   topic`, and every compromise the converter makes (the `journal.CODES` table — this already
   exists in V1 and is genuinely good documentation).
5. Frames and conventions: `psdk_base_link`, `psdk_map_enu`, ENU vs FLU vs NED, where PX4’s NED
   becomes DJI’s ENU and what that costs.
6. QoS: why images are best-effort, why commands are reliable, and the compatibility rules.
7. Time: simulated clock, `/clock`, and what `use_sim_time` means for your node.
8. Discovery: localhost-only, no multicast, why, and how to verify it.
9. What a real Matrice 4E + Manifold 3 does differently.

---

## 20. Milestones and acceptance criteria

Each milestone is independently demonstrable. Nothing later starts before its predecessor’s
acceptance test passes.

| # | Milestone | Acceptance test |
|---|---|---|
| **M0** | **Spikes** (§21) | SPIKE-1 and SPIKE-2 answered in writing, with the commands that answered them |
| **M1** ✅ | **Image + container + mounts** | **DONE 2026-09-21.** `docker build` succeeds; `./walker up` starts the container; all 10 mounts present and writable (checked from inside); isolation **proved**, not asserted — `walker doctor --deep` publishes a topic in the stack's container and confirms a separate container on the same bridge network cannot see it. `walker doctor` is the executable form of this test and exits non-zero in CI. |
| **M2** ✅ | **walkerd + walker skeleton** | **DONE 2026-09-21.** Dashboard renders with live link rates. Socket round-trip **0.064 ms median / 0.83 ms max** over 200 calls (budget was <1 ms). The `sim` unit starts (20 s to ready), runs, and stops in 1 s **leaving no orphans**, all through walkerd. The readiness probe names the failing layer when Gazebo is killed. Constraint matrix enforced from one table. `walker dump` renders a frame as text so the layout is testable in CI. |
| **M3** ✅ | **World + drone selection** | **DONE 2026-09-22.** Full scan of models/worlds/bags/projects in **4.7 ms** (budget 200 ms). Worlds, Drones and Cameras pickers render and commit to walkerd. A second world (`flat`) written from scratch was scanned, selected and **flew** — no rebuild, no code change. Generated airframe vs V1's: see Appendix E — numerically identical, not byte-identical. |
| **M4** 🔶 | **Bridge always-on + full PSDK surface** | **Substantially done 2026-09-22.** ✅ Bridge is a walkerd unit, up with the simulation. ✅ **56/56 services (100%)** with service introspection on (59 `_service_event` topics). ✅ 90/102 of the surface live, **0 unexpectedly absent**. ✅ 11/19 measurable rates within ±20% of the PDF (was 3). ⬜ Remaining: 8 source-limited rates need periodic republish; 7 telemetry topics need converters V1 never had. See Appendix F. |
| **M5** ✅ | **C++ projects + terminals + constraints** | **DONE 2026-09-22.** `demo_orbit_mission` flies authority → takeoff → climb → a clean orbit → land → release, **exits 0 by itself**, and frees the flight lock. A second project is refused by name with the way out. `walker-attach` gives a live PX4 console in its own ptyxis window. Step transitions appear in walker's log pane. See Appendix G. |
| **M6** 🔶 | **Cameras + RViz + QGC + the guard** | **Mostly done 2026-09-23.** ✅ Live camera toggling **measured through walker**: 41.9% → 101.2% (fisheye) → 41.9%, and 58.4% (payload wide FHD) → 41.3%, mid-flight, no restart. ✅ Camera intrinsics corrected against DJI's published specs (Appendix H). ✅ RViz unit with best-effort image displays, real window. ✅ **MAVLink guard proven**: with a mission flying, QGC's COMMAND_LONG and SET_MODE are dropped while 171 telemetry datagrams keep flowing; the lock releases when the mission ends. ⬜ Remaining: the walker RViz panel (D6), and QGC flown manually end to end. |
| **M7** | **rosbag record + replay** | record during a project; record during a replay; named bag lands in `bags/`; `ros2 bag info` is clean; fly-back replay re-flies the recorded orbit; passive replay drives RViz with no simulation running |
| **M8** | **Documentation** | the three files, reviewed against the built system, every command in them executed once |
| **M9** | **`demo_gnss_stereo_inertial` port — committed (D5)**, plus `simty.mock` as a test fixture and a CI smoke test | the SLAM project builds and runs on the `fisheye` profile, consuming `perception_stereo_{left,right}_stream` + `camera_info`, and its trajectory error against ground truth is reported; headless orbit flight in CI producing a bag |

---

## 21. Risks and de-risking spikes

| ID | Risk | Impact | Spike / mitigation |
|---|---|---|---|
| **SPIKE-1** | `always_on=false` may not stop gz Harmonic from rendering a camera, which would kill live camera toggling | High — it is the headline new feature | **ANSWERED — YES, it works, once a hidden subscriber is dealt with.** See Appendix C. Closed. |
| **SPIKE-2** | PX4 SITL may not resolve `PX4_SIM_MODEL` against a runtime-dropped airframe file, which would kill drone selection | High — it is the other headline feature | **ANSWERED — YES, it works.** See Appendix C. Closed. |
| **R3** | Wayland/XWayland: a container GUI (Gazebo, RViz, QGC) fails to reach the display | Medium | V1 already does this on this machine; keep `allow_x11` (xhost) and the `/tmp/.X11-unix` mount. Detect and report clearly if `DISPLAY` is unset. |
| **R4** | Only `ptyxis` is installed; a teammate’s machine has something else | Low | Detection chain + `SIM_TERMINAL` override + in-walker log fallback (§8) |
| **R5** | Dropping `--network host` breaks something that silently relied on it | Medium | M1’s acceptance test is exactly this. QGC, PX4 MAVLink and XRCE are all container-internal. |
| **R6** | `--shm-size` too small → Fast DDS SHM segment failures that look like discovery bugs | Medium | 2 g, and walkerd checks `df /dev/shm` at start and warns below 512 MB free |
| **R7** | Removing 6 000 lines from simty breaks a converter that quietly depended on a removed module | Medium | Remove module by module, running the full wrapper-surface rate check (M4’s test) after each |
| **R8** | Two writers on one wrapper topic during replay | High (silent) | The bridge’s replay mode, plus the §7 constraints, plus walkerd refusing the combination |
| **R9** | GPU-less machine: the `all` camera profile makes the simulation unflyable | Low | Walker computes and displays the MP/s cost before you commit, warns when `/dev/dri/renderD128` is absent, and defaults to `none` |
| **R10** | Scope creep — V2 quietly regrows V1’s Manifold/RC/mock machinery | Medium | §22 |
| **R11** | The MAVLink guard (§16.2) mangles frames — MAVLink v2 signing, sequence numbers or fragmentation — and QGC loses the link | Medium | The relay forwards frames **byte-for-byte**; it only ever decides forward-or-drop on the message ID, never rewrites. Tested in M6 by leaving QGC connected for 10 minutes under lock and confirming zero link drops. Fallback to PX4-parameter enforcement in §16.2. |
| **R12** | The new RViz panel repeats V1's mistake and grows into a second control surface | Low | Its scope is fixed at four actions and one state subscription (§14.3); anything else is a walker feature. Reviewed at M6. |

---

## 22. Explicit non-goals

Version 2 does **not**:

- talk to a Manifold 3, a real aircraft, or anything off this host;
- use multicast DDS, or share a ROS domain with another machine;
- support multiple simultaneous drones in one world;
- support a distributed or multi-container deployment;
- reimplement DJI’s MSDK, Cloud API or waypoint-mission (v2/v3) services — the mission logic
  lives in your C++ project, on top of the flight-control topics;
- replace QGroundControl with its own map/mission UI;
- ship a “fly it like a game” panel of its own — D1 removed V1's, and QGC is the manual-flight
  answer;
- replay a bag without a simulation behind it (D3);
- guarantee real-time performance — it is SITL on a laptop, and the sim clock says so;
- keep V1 runnable from the same checkout. V2 is a new repository; V1 stays where it is.

---

## Appendix A — Source material read for this plan

| File | What it settled |
|---|---|
| `run.sh` (3 000 lines) | the four modes, camera choice, recording, replay decision logic, preflight |
| `simulation.sh` (1 100 lines) | container flags and why each is load-bearing, mounts, probes, bridge modes |
| `config/launch_sim.sh` | the exact simulation boot order and the PX4 parameter pokes |
| `config/bridge.yaml` | every ros_gz_bridge route and the `lazy:true` pattern |
| `Dockerfile` (656 lines) | the image, the baked airframe, the PX4/QGC acquisition, the MAVLink loopback fix |
| `tools/compose_sim.py` | camera composition, and why it edits text rather than XML |
| `gui/simty/*` (~14 000 lines) | the bridge: routes, converters, services, QoS, the control surface |
| `models/m4e/model.sdf`, `models/m4e_camera/model.sdf` | 14 camera sensors, all `always_on=true` — the fact SPIKE-1 turns on |
| `demo/demo_orbit_mission/src/main.cpp` | the reference flight pattern, step by step |
| `demo/*/demo.conf` | the per-project launcher settings mechanism |
| `bags/README.md`, `worlds/README.md` | the bag folder contract; the world, in numbers |
| `psdk_ros2_topics_services.pdf` | the authoritative 41 / 5 / 8 / 56 surface |
| host probe | GNOME/Wayland, `ptyxis` only, Python 3.14, Docker 29.5.3, no tmux |

---

---

## Appendix C — M0 spike results

### SPIKE-2 — runtime airframe + runtime drone model — **PASSED**

**Question:** can a drone model and a PX4 airframe that were never built into the image be
created at run time and flown, or does drone selection require an image rebuild?

**Answer: it works, and the mechanism is read directly out of PX4's own boot scripts.**

Two independent confirmations:

1. **By inspection.** `build/px4_sitl_default/etc/init.d-posix/rcS` resolves the airframe by
   *scanning the runtime directory at boot*:

   ```sh
   # Find the matching Autostart ID (file name has the form: [0-9]+_${PX4_SIM_MODEL})
   REQUESTED_AUTOSTART=$(ls "${R}etc/init.d-posix/airframes" \
       | sed -n 's/^\([0-9][0-9]*\)_'${PX4_SIM_MODEL}'$/\1/p')
   ```

   Nothing is compiled in. A file dropped into that directory is found on the next boot.
   `px4-rc.gzsim` then derives the Gazebo model name the same way:
   `MODEL_NAME="${PX4_SIM_MODEL#*gz_}"`.

2. **Empirically.** A drone called `m4x` — a name that has never existed in the image — was
   created at run time together with a `4950_gz_m4x` airframe, and PX4 booted against it:

   ```
   INFO  [init] found model autostart file as SYS_AUTOSTART=4950
     SYS_AUTOSTART: curr: 0 -> new: 4950
   INFO  [init] Spawning Gazebo model
   INFO  [gz_bridge] world: default, model: m4x_0
   ```

   The model appeared in the world with its full sensor set:

   ```
   /world/default/model/m4x_0/link/base_link/sensor/imu_sensor/imu
   /world/default/model/m4x_0/link/base_link/sensor/navsat_sensor/navsat
   /world/default/model/m4x_0/link/base_link/sensor/magnetometer_sensor/magnetometer
   /world/default/model/m4x_0/link/base_link/sensor/air_pressure_sensor/air_pressure
   ```

**Consequence:** §10.3 stands as designed. No fallback needed, and the "pre-baked airframe
slots" contingency is dropped.

### Finding C1 — PX4 spawns models by ABSOLUTE PATH, not by resource path (**new requirement**)

The same run surfaced something the plan had not accounted for, and it is the reason V1's
aircraft is not selectable:

```
[Err] [UserCommands.cc:927] Error Code 14: [/sdf/include[0]/uri]:
      Unable to find uri[file:///home/developer/PX4-Autopilot/Tools/simulation/gz/models/m4x/model.sdf]
```

`px4-rc.gzsim` does **not** honour `GZ_SIM_RESOURCE_PATH` when it spawns the vehicle. It
builds an absolute URI:

```
${PX4_GZ_MODELS}/${PX4_SIM_MODEL#*gz_}/model.sdf
```

V1 satisfied this by symlinking its models into that directory **at image build time**
(`Dockerfile`: `ln -sf /home/developer/gz_models/m4e .../gz/models/m4e`) — which is exactly
what makes V1's drone a build-time constant.

**Requirement added to `tools/compose_sim.py`:** after composing, and before PX4 starts,
symlink the composed model directory (and each of its attachments) into `$PX4_GZ_MODELS`, and
the chosen world into `$PX4_GZ_WORLDS`. `GZ_SIM_RESOURCE_PATH` is still set with
`gz_runtime` first — Gazebo itself does use it — but PX4's spawn needs the symlink.

The V2 Dockerfile creates both directories empty and `developer`-owned for this purpose
(see its "RUNTIME MODEL AND WORLD DIRECTORIES" layer), and bakes no model into either.

### SPIKE-1 — live camera toggling — **PASSED**

**Question:** does `always_on=false` make gz Harmonic render a camera only while something is
subscribed, so camera groups can be switched on and off during a flight without a restart?

**Method.** A minimal world containing only `m4e_camera` (12 payload camera sensors + the
LRF), run headless under PX4's real `server.config`, measuring the `gz sim` server process's
own CPU (utime+stime from `/proc/<pid>/stat`) over 15–20 s windows. Four variants, one
variable at a time.

| Variant | Subscribers | gz sim CPU |
|---|---|---|
| `always_on=true` (V1's models, as shipped) | none | **133.5 %** |
| `always_on=false` | none | 77.8 % |
| `always_on=false`, `GstCameraSystem` not loaded | none | **18.9 %** |
| camera sensors stripped from the SDF entirely (V1's compose-time approach) | none | **17.1 %** (the floor) |

`always_on=false` plus no hidden subscriber lands within 1.8 points of the floor — that is,
**it costs essentially nothing to have a camera present but switched off.**

**The toggle, both directions**, on one 30 Hz 1080p sensor:

```
PHASE 1  no subscriber          cpu=18.8%
PHASE 2  one camera subscribed  cpu=106.3%    frames received: 162
PHASE 3  subscriber gone        cpu=17.0%
```

Rendering starts on subscribe, stops on unsubscribe, and returns to the floor. **No restart.**
§11.2 stands as designed and the compose-time fallback is not needed.

### Finding C2 — `GstCameraSystem` is a hidden always-subscriber (**new requirement**)

The gap between 77.8 % and 18.9 % is entirely `libGstCameraSystem.so`, the video-streaming
plugin that feeds QGC. It subscribes to camera topics to read their `camera_info`
(`GstCameraSystem.cpp:341`) and, having subscribed, keeps those sensors rendering even in
selective mode where only one stream is actually maintained. A camera group that walker has
switched "off" would keep costing GPU time for as long as that plugin is loaded.

**Requirement added:** walkerd composes the **gz server config per run**, the same way it
composes the model set, and loads `GstCameraSystem` only when QGC video is actually wanted
(QGC enabled *and* a payload group enabled). Everything else in `server.config` — Physics,
Sensors, Imu, NavSat, Magnetometer, AirPressure, UserCommands, SceneBroadcaster — is always
loaded. This needs no C++ change and is one more generated file alongside the composed model
and the generated airframe.

*Follow-up (not blocking, logged for M6):* the plugin's `camera_info` subscription should be
closed once the resolution has been read. Fixing that would let the plugin stay loaded
permanently without holding sensors awake.

---

## Appendix D — Findings from M2 (the composer and the sim unit)

### D-1 — Manifests are TOML, not YAML

`tomllib` has been in the standard library since Python 3.11 (host 3.14,
container 3.12). YAML would mean PyYAML, which means pip, which is the thing
decision D2 exists to prevent. TOML also has comments, which JSON does not, and
these files are mostly explanation. So a drone declares itself in
`models/<name>/walker.toml` and a world in `worlds/<name>.walker.toml`.

### D-2 — A world must not name a drone (**fixed in the composer**)

V1's `worlds/powerline.sdf` contained
`<include><uri>model://m4e_camera</uri></include>` — the world knew which
aircraft would fly in it, which makes "any drone in any world" impossible.

`compose_sim.py` now strips every include whose model is `kind = "drone"` or
`kind = "attachment"`, and injects the **selected** drone's attachments from its
own manifest. A world that never mentioned a drone is untouched. This is what
lets the powerline corridor host a different aircraft tomorrow.

### D-3 — PX4's `pxh` console costs 22 MB/min without a TTY (**fixed**)

Started with its output on a pipe, PX4 redraws its `pxh> ` prompt continuously:
measured at **22 MB of escape sequences in under a minute**, burning CPU the
simulation needs. `tools/start_sim.sh` now checks `[ -t 0 ]` and passes `-d`
(daemon mode, no console) when there is no terminal — log dropped to 64 KB.

This also settles how walkerd must run units: **give the `sim` unit a real
PTY**, which both keeps the log sane and makes `walker-attach sim` genuinely
useful — an attached terminal gets a live PX4 console where `commander status`
and `param show` work.

### D-4 — The ROS 2 CLI is unfit for readiness probes (**shaped `walkerd/probes.py`**)

Measured against a stack that was demonstrably healthy and publishing:

| CLI call | Said | Truth |
|---|---|---|
| `ros2 topic hz /fmu/out/sensor_combined` | "no data" | 99.3 Hz — it subscribes **RELIABLE** and PX4 publishes **BEST_EFFORT**, so it never matched |
| `ros2 topic info <same topic>` | "Unknown topic" | `ros2 topic list` listed it; the two take different paths through the daemon cache |
| any of them | — | 4–5 s each before discovery even starts |

A readiness check built on that is slow *and* wrong, and its failures send
people to debug a working simulation. `walkerd/probes.py` therefore opens **one
rclpy node**, subscribes to every link at once with the QoS the publisher
actually uses, and measures for one fixed window. The whole chain is answered in
~2.5 s from a single instant.

### D-5 — `pgrep -f` matches its own shell (**fixed**)

`pgrep -f "gz sim"` matches the command line of the shell running the probe,
because that command line contains `gz sim`. The probe reported Gazebo healthy
seconds after it was killed. Fixed with V1's bracket trick (`[g]z sim` — a regex
matching text the pattern itself does not contain) plus `pgrep -r DRSW`, since
gz's ruby launcher leaves a defunct entry that `pgrep` otherwise counts as
alive.

### End-to-end result

A simulation composed entirely from manifests — generated airframe, composed
world, composed models, generated bridge config and server config — boots and
runs:

```
  processes
    [ ok ] gazebo      [ ok ] xrce_agent   [ ok ] ros_gz_bridge
    [ ok ] px4         [ ok ] px4_socket   [ ok ] composed
  links
    [ ok ] gazebo clock          238.7 Hz    /clock
    [ ok ] px4 -> xrce -> ros     99.3 Hz    /fmu/out/sensor_combined
    [ ok ] px4 attitude           19.7 Hz    /fmu/out/vehicle_attitude
```

and killing Gazebo names the layer that died rather than reporting a generic
failure.

### D-6 — PX4 versions its uORB topics, and the old name still appears (**shaped the watcher**)

The graph carries **both** `/fmu/out/vehicle_status_v1` and
`/fmu/out/vehicle_status_v4`; on this build only `v4` publishes. Subscribing to
`v1` gives a topic that exists, reports a publisher, and never delivers a
message — the silent failure these probes exist to catch, and one that would
return with every PX4 upgrade.

`walkerd/rosnode.py` therefore takes a **list of candidate topics** per link and
counts them together, so a version bump does not silently blind the dashboard.

### D-7 — The persistent watcher was not just faster, it was more accurate

Replacing the per-call probe with one long-lived rclpy node changed the numbers
it reported, because a node created for a 2.5 s measurement spends most of that
window on discovery:

| link | per-call node | persistent node | truth |
|---|---|---|---|
| `/clock` | "no messages" | 250.0 Hz | 250 Hz |
| `/fmu/out/sensor_combined` | 121.6 Hz | 250.0 Hz | 250 Hz |
| `/fmu/out/vehicle_attitude` | 24.4 Hz | 50.0 Hz | 50 Hz |

Everything was being undercounted by roughly half, and `/clock` was being
reported as dead on a healthy simulation. Time to "sim ready" also fell from
36 s to 20 s, because readiness no longer waits for a fresh participant to
discover the graph on every poll. Probe latency is now ~20 ms.

### D-8 — Bounds-safe curses writes

`addstr` raises on a write to the bottom-right cell — it tries to advance the
cursor past the end of the screen — so a full-width status bar crashed the whole
UI on its first paint. It also raises for any write starting off-screen, which
happens as soon as the window is made narrow. Every write now goes through one
clipping helper; the dashboard degrades instead of dying.

`walker dump` was added for the same reason: a curses layout cannot be checked
by reading the code, and capturing a real terminal means re-implementing one.
Asking curses itself what is on screen (`instr`) is exact and works in CI.

---

## Appendix E — M3: selection, and what the airframe comparison actually showed

### The acceptance criterion was wrong as written

The plan asked for the generated airframe to match V1's **byte for byte**. That
is not achievable and should not be: the generated file carries a provenance
header saying where it came from and that editing it is pointless. What matters
is that the aircraft flies identically, so the comparison made was of the
`param set-default` set, whitespace-normalised:

| | V1 (baked into the image) | V2 (generated from `walker.toml`) |
|---|---|---|
| params | 33 | 34 |
| differences | — | `-0.16500` vs `-0.165`, `0.60` vs `0.6` — the same numbers |
| extra in V2 | — | `MIS_TAKEOFF_ALT 1.8` |

`MIS_TAKEOFF_ALT` is not new behaviour: V1 set it at run time from
`launch_sim.sh` with `px4-param set`. Moving it into the airframe puts it with
the other 33 params, per drone, in one place — the whole point of the manifest.

**Verdict: a superset of V1's, numerically identical where they overlap.**

### Finding E-1 — a broken manifest was silently hiding the thing it described

`scan.py` claimed to list broken entries rather than drop them. It did not. A
drone whose `walker.toml` failed to parse returned an empty dict, so `kind`
defaulted to `"scenery"`, so it vanished from the Drones screen — leaving you
to wonder why the aircraft you just added is not listed. Precisely the failure
the module's own docstring promised to avoid.

Fixed: a manifest that fails to parse classifies as `broken`, and the Drones
screen lists it with the parser's own complaint, line and column included.
Something that tried to declare itself and failed is far more likely to be what
you are looking for than scenery.

### Finding E-2 — the composer tracebacked on the same input

An unhandled `TOMLDecodeError` printed a stack whose most prominent line was a
frame inside `compose_sim.py`, which reads as "the composer is broken" rather
than "your manifest has a typo". It now names the file and quotes the parser.

### The new world

`worlds/flat.sdf` — bare ground, 2 km square, same origin as the corridor so
coordinates mean the same thing in both and bags can be compared across them.
It exists because the powerline world renders five towers and four catenary
spans that a velocity controller has no opinion about; `flat` is what a
mission's step machine, a climb controller, or CI should be tested against.

Its header also documents the one mistake a new world is most likely to make:
declaring system plugins. PX4's `server.config` already loads Physics, Sensors,
Imu and the rest, and declaring them again in the world loads each twice —
which applies forces twice and makes the drone spin on the spot.

---

## Appendix F — M4: the bridge, and what the surface really looks like

### What came across

`gui/simty/` → `bridge/simty/`, with the Manifold machinery removed:

| removed | lines | why |
|---|---|---|
| `tui.py` | 2 042 | walker is the operator surface now |
| `mock.py` | 456 | a synthetic Manifold to talk to |
| `remote.py` | 353 | started the real wrapper over SSH |
| `provision.py` | 347 | escalation policy between those |
| `standin.py` | 291 | service stubs for what we did not serve |
| **total** | **3 489** | ~35% of the bridge, with no loss of simulated behaviour |

`discovery.py` survived but was reframed. It used to ask "is the Manifold
there?"; it now asks **"is the wrapper surface up, and is it only ours?"** —
`SurfaceState.QUIET / OURS / FOREIGN`. That is not a cosmetic rename: FOREIGN
is the two-writers condition, which in V2 means a bag replay has started
publishing over the bridge, and it is invisible from inside a mission.

### Finding F-1 — V1's defaults were inverted for V2

71 topic routes existed and only **26 were enabled**. That was correct in V1: a
real Manifold published those wrapper topics, so the bridge had to stay off
them. With no Manifold the bridge is the only possible publisher, and a route
left off is simply a topic a mission waits on forever.

The V2 policy (`_apply_v2_surface_policy`) inverts the default: everything on
except video, which stays opt-in because 6 MB a frame is a real constraint. The
24 `mirror_*` routes are **deleted** rather than disabled — they copied what a
Manifold published into `/manifold/*`, so in V2 they would subscribe to a topic
nobody publishes and republish silence, while making the topic *appear* handled.

Services went from 35 to **56/56**.

### Finding F-2 — the rate caps measured a constraint that no longer exists

V1 capped routes between 1 and 50 Hz, sized for the **wireless link to a
Manifold** where `dds.xml` capped datagrams at 1400 bytes. V2's whole graph is
shared memory in one container. Leaving the caps made the simulation *less*
faithful than the aircraft it imitates — attitude arrived at 17 Hz against a
documented 50.

Removing them entirely overshot in the other direction (imu 88 Hz vs 50), which
is equally wrong: a consumer tuned against the simulation would misbehave on
hardware. Caps are now set from `PDF_RATE_HZ`, the documented aircraft rates.

### Finding F-3 — the rate limiter under-delivered by a third

Applying a 50 Hz cap to a 50 Hz stream produced **33.8 Hz**. The limiter tested
`now - last < 1/max_hz`, and a real 50 Hz source does not arrive on a perfect
20 ms grid: ordinary jitter puts about a third of samples a fraction early, and
each of those was dropped.

Replaced with a deadline that advances by one whole period (clamped so a burst
after a quiet spell is not released at once). Result: **3 → 11 of 19 rates
within ±20%**.

### What is still not right, precisely

**8 rates are source-limited.** The bridge cannot publish faster than PX4 feeds
it, and PX4's rates are not DJI's:

| wrapper topic | live | PDF | PX4 source |
|---|---|---|---|
| `flight_status`, `display_mode`, `flight_anomaly`, `rc_connection_status` | 2 Hz | 25 Hz | `vehicle_status_v4` at 2 Hz |
| `altitude_barometric`, `altitude_sea_level` | 50 Hz | 70 Hz | `vehicle_global_position` at 50 Hz |
| `single_battery_index2` | 1 Hz | 21 Hz | `battery_status_v1` at 1 Hz |
| `relative_obstacle_info` | 1 Hz | 25 Hz | the LRF sensor's own `update_rate` is 1 in the model SDF |

The fix is the one real DJI firmware uses: **republish from last-known state on
a timer** at the surface's documented rate, rather than only on source change.
`relative_obstacle_info` is cheaper still — raise the LRF `update_rate` in
`models/m4e_camera/model.sdf`.

**7 telemetry topics are not synthesised at all** — `acceleration_body_{fused,
raw}`, `acceleration_ground_fused`, `esc_data`, `magnetic_field`, `rtk_velocity`,
`rtk_yaw`, plus `perception_camera_parameters`. V1 never synthesised these
either: they arrived from the aircraft's hardware and the bridge merely mirrored
them, so there was no conversion to inherit. They are listed in
`registry.SYNTHESIS_GAP` and reported by `tools/check_surface.py` as a gap
rather than a failure, so nobody rediscovers them by watching a mission wait.

**2 command topics have no honest conversion**: PX4 takes attitude and thrust on
a different message entirely, and `flight_control_setpoint_generic` is a raw
PSDK flag byte. A route that accepted them and quietly did nothing would be
worse than their absence.

### Finding F-4 — building the conformance check on the ROS CLI (again)

`tools/check_surface.py` first reported **0/102** against a bridge that was
demonstrably publishing: the CLI daemon answered from a cache populated before
the bridge started. This is finding D-4 for the third time, so it is now a rule
rather than an observation: **anything this project relies on reads the graph
through rclpy, never through the `ros2` CLI.** The tool was rewritten
accordingly, and it also has to spin for a few seconds before reading, because
a fresh participant knows nothing about the graph the instant it is created.

---

## Appendix G — M5: the first real flight, and six bugs it found

### The flight

```
telemetry is flowing, starting
obtain_ctrl_authority: ok
takeoff: ok
automatic takeoff finished at 1.10 m, taking over to reach 3.0 m
reached 2.9 m, starting the circle
circle finished after 40 s, landing
landed and disarmed
release_ctrl_authority: ok
flight finished, shutting down
```

Two laps of 20 s each, landing exactly on schedule — flown entirely through
`/wrapper/psdk_ros2`, with the same binary that would fly a real Matrice 4E.

### G-1 — the mission flew but never engaged offboard

The first attempt reached `automatic takeoff finished at 0.75 m` and then:
`PX4 never switched to SDK_CTRL after 15 s`. Every service worked; the
setpoints went nowhere.

`auto_offboard` defaults to **False**, and only `set_project_mode` turns it on
— in V1 that was the operator pressing `P` on the bridge console. The mission's
own error message names neither the bridge nor the setting, so from inside the
mission this is indistinguishable from a broken simulation.

walkerd now sends `project value=on setpoint=<from project.conf>` before
starting a mission, and `project value=off` when it stops, so the bridge is
never left quietly wide open. The setpoint frame comes from the project, so
`demo_camera_track` gets the body-frame route rather than the ground-frame
default.

### G-2 — a finished mission held the simulation hostage

`demo_orbit_mission` reached `DONE` and kept spinning. walkerd treats a running
process as a running mission, and a running mission holds the flight lock — so
a *completed* flight locked out the next project, locked out replay, and would
have kept QGC in observer mode, until somebody noticed and stopped it by hand.

Every ported mission now calls `rclcpp::shutdown()` on `DONE`. This is a
contract item for `CPP_DESIGN.md`: **a mission that has finished exits.**

### G-3 — a clean exit was reported as a failure

With G-2 fixed, the dashboard said `failed — exited with code 0`. A mission that
flies its pattern, lands, releases authority and exits 0 has *succeeded*;
calling that a failure tells the operator their flight went wrong when it went
exactly right. There are three outcomes, not two: asked to stop, finished its
work (`stopped — completed`), and fell over.

### G-4 — a 57 s build inside a request

Starting a stale project rebuilt it first, and the request blocked for the whole
build until the client gave up at 60 s. The same "never block" rule that governs
the keyboard governs the protocol: the reply is now immediate, and the build,
the bridge mode change and the start are reported as ordinary unit events.
The unit shows `starting — preparing (build + bridge project mode)`.

### G-5 — walkerd kept dying with an empty log

`docker exec -d` returns immediately, but the process it starts stays in that
exec session's process group. When the docker client was killed — a script
timing out, a terminal closing — the group got SIGTERM and walkerd shut down
cleanly and invisibly. The container had not restarted and the log held no
error, which makes this very hard to read.

Fixed with `setsid nohup`, and verified: walkerd now survives its client being
killed. `walker up --restart-walkerd` was added for the case that kept biting
during development — editing `walkerd/` or `bridge/` has no effect until the
supervisor is restarted, because Python holds the old module in memory.

### G-6 — the scanner caught a V1 bug on sight

`demo_square_mission/package.xml` declared `<name>demo_orbit_mission</name>` —
two directories building one package name, so colcon installed whichever it saw
last. This is exactly what V2's "package name equals directory name" rule exists
to catch, and it was caught the first time the projects were scanned.

### A note on what `ros2 run` needs

A mission built after walkerd started was not in walkerd's environment, and
`ros2 run` answered *"Package 'demo_orbit_mission' not found"* about a package
that had compiled successfully thirty seconds earlier. Units now source the
workspace overlay in their own shell at spawn, so a project built at any moment
is runnable immediately.

---

## Appendix H — M6 part 1: the window, the optics, and when a setting may change

### H-1 — the simulation had no window at all

`start_sim.sh` defaulted to `SIM_HEADLESS=1`. Correct for CI, wrong for a
person, and it went unnoticed because every development test passed
`SIM_HEADLESS=1` explicitly. The default now follows the environment: a window
when `DISPLAY` is set **and answers**, headless otherwise. The probe matters —
`DISPLAY` can be set and unusable, and Gazebo's GUI then dies seconds later with
an Ogre error long after the "starting" line scrolled past, so the simulation
looks like it came up fine. `[g]` toggles it.

### H-2 — every payload camera had the wrong field of view

DJI publishes a **diagonal** FOV; Gazebo's `<horizontal_fov>` is horizontal. The
model fed the quoted numbers straight in, so every lens saw a much wider scene
than the real aircraft — which quietly invalidates anything measured through it:
a detection's bearing, a pixel-to-metre scale, a photogrammetry overlap
estimate.

That the figures are diagonal is **checkable, not assumed**: each matches the
diagonal FOV implied by its own quoted 35 mm-equivalent focal length.

| lens | equiv f | DFOV quoted | DFOV from f | HFOV (4:3) | was | now |
|---|---|---|---|---|---|---|
| wide | 24 mm | 84.0° | 84.1° | **71.5°** | 84.0° | 71.5° |
| medium tele | 70 mm | 35.0° | 34.3° | **28.3°** | 35.0° | 28.3° |
| tele | 168 mm | 15.0° | 14.7° | **12.0°** | 15.0° | 12.0° |

All tiers of a lens share its HFOV: the 16:9 video modes are a vertical crop of
the same 4:3 sensor, so they keep full sensor width and Gazebo derives each
tier's vertical FOV from the image aspect (wide 4K: 71.5° × 44.1°).

The LRF's `update_rate` was also raised from 1 Hz to 25 Hz, which was the cause
of `relative_obstacle_info` arriving at 1 Hz against the PDF's 25 (finding F-2's
loose end).

**Fisheye pair**, against the published spec — faithful in horizontal FOV (90°),
image (704×704 mono8) and rate (20 Hz); **not** faithful in vertical FOV, since
the real forward pair is 90°×135° and a square image cannot express that.
Matching the published stream was judged more important than matching the lens.
The 90 mm baseline is a plausible value for the airframe, not a measured one,
and it is load-bearing for stereo depth — a node calibrated here must be
recalibrated against the aircraft. Only the forward pair is modelled, because
only the forward pair reaches the wrapper surface.

### H-3 — the payload was filming the wrong place

V1's `gimbal_pose_relay.py` and `gimbal_stabilizer.py` had not been ported. The
payload is a **separate** Gazebo model (gz's `set_pose` only sticks on a
top-level model, so a stabilised gimbal cannot be a link of the drone) and
nothing was moving it — it sat at the world origin while the aircraft flew away.

The simulation looks completely healthy in that state: the drone flies, the
cameras render, frames arrive on the wrapper surface, and every payload image
shows the patch of ground the aircraft took off from.

Ported into `simsupport/`, parameterised from `compose.json` rather than
hard-coded to `default`/`m4e_0`, and verified in flight — a constant 0.122 m
nose-mount offset held through x, y and z while the aircraft climbed and
orbited.

### H-4 — when a setting may be changed

The constraint table said what may RUN alongside what. It said nothing about
what may be CHANGED, and when — so a setting read once at compose time could be
edited freely under a running simulation and would appear to have been accepted
while changing nothing. That is the failure mode this project keeps finding:
**the interface says yes and the system does not change.**

`constraints.SETTINGS` now classifies every setting:

| setting | when | why |
|---|---|---|
| `world` | sim stopped | composed and symlinked into PX4's path at start |
| `drone` | sim stopped | airframe and model generated at start; PX4 reads the airframe once, at boot |
| `gui` | sim stopped | the Gazebo client starts alongside the server |
| `qgc_video` | sim stopped | decides whether `GstCameraSystem` is written into the server config (Finding C2) |
| `cameras` | **anytime** | subscription-driven — the SPIKE-1 capability |
| `lens`, `tier` | sim running | nothing to switch between until the cameras exist |

walkerd refuses a locked change with the reason and the way out, checking
**every** requested change before applying any (a partly applied selection is
worse than a refused one). walker greys the key out and shows `⨯ sim up`, so
nobody presses it three times wondering whether the keyboard is broken. Both
read the same table.

---

## Appendix I — M6 part 2: cameras on demand, and a lock that is real

### Live camera switching, measured end to end

SPIKE-1 established the mechanism; this is the capability, driven from walker
on a running simulation:

| phase | gz server CPU |
|---|---|
| all camera groups off | 41.9 % |
| **fisheye on** | **101.2 %** |
| off again | 42.0 % |
| **payload on (wide, FHD)** | **58.4 %** |
| off again | 41.3 % |

Nothing restarts. A group goes on by making something subscribe and off by
making it stop; Gazebo follows within a sensor period. The chain is
`always_on=false` in the SDF → `lazy:true` in ros_gz_bridge → the bridge's video
route, with `camera_switcher` holding exactly one payload lens at a time.

`tools/gz_cpu.sh` is kept as a tool because rendering cost is the **only**
honest way to tell a camera that is genuinely off from one that is merely not
being looked at — topic rates cannot distinguish them, since an unsubscribed
sensor publishes nothing either way.

### I-1 — the MAVLink guard (decision D4, delivered)

You asked for a lock rather than a warning, and a warning is not a lock: an
operator who clicks Disarm mid-mission gets a disarmed aircraft whatever the
banner said. So PX4's GCS link points at the guard, always:

```
                telemetry, always
    PX4 :14541  ───────────────────────────────►  QGC :14550
                ◄───────────────────────────────
                  commands, only while unlocked
```

Measured with a mission flying:

```
flight lock: project        guard locked: True (holder=project)
QGC commands BLOCKED:       {'COMMAND_LONG': 5, 'SET_MODE': 5}
telemetry still flowing:    171 more datagrams to QGC
after the mission stops:    lock released, 0 further blocks
```

Fifteen message ids are blocked while locked — arm/disarm/takeoff/RTL
(`COMMAND_LONG`/`COMMAND_INT`), `SET_MODE`, joystick and RC override, the
offboard setpoint messages, the mission **write** side, and `PARAM_SET`.
Mission *reads* pass, so QGC can still display the current plan.

Three decisions worth stating:

* **The guard is always in the path**, even unlocked. Inserting a relay only
  when needed means re-pointing PX4's MAVLink instance mid-flight, which drops
  the ground-station link at the exact moment someone is most likely watching
  it.
* **Frames are forwarded byte for byte.** The only decision is forward-or-drop,
  on the message id; nothing is re-encoded, so v2 signing, sequence numbers and
  CRCs are untouched. A relay that rewrote frames would have to re-sign them.
* **The lock follows the flight lock on a 1 Hz watch**, not only on start and
  stop — because a mission that exits by itself (G-2) must release QGC just as
  surely as one that is stopped from the dashboard. A lock released only on the
  tidy path is a lock that eventually sticks.

### I-2 — RViz image displays are best-effort, and that is not optional

`config/rviz/flight.rviz` sets `Reliability Policy: Best Effort` on every image
display. Image streams are published with `SensorDataQoS`; a reliable subscriber
never matches them and RViz then shows an empty panel **with no error at all**.
That is the most common "the cameras are broken" report in a project like this,
and the cameras are fine. Finding D-4, third form.

---

## Appendix B — Decision log

| Date | Decision | Effect |
|---|---|---|
| 2026-09-21 | D1 no Tkinter panel | `gui/drone_controller.py` not ported |
| 2026-09-21 | D2 stdlib curses | no third-party TUI dependency, enforced in CI |
| 2026-09-21 | D3 replay requires the simulation | V1's simulation-less replay path removed |
| 2026-09-21 | D4 QGC allowed, locked by a MAVLink guard | new component `walkerd/mavguard.py` + `flightlock.py` |
| 2026-09-21 | D5 SLAM project committed to M9 | fisheye profile becomes a first-class requirement |
| 2026-09-21 | D6 RViz panels re-implemented | V1's deleted; new `walker_rviz_panel/` on a 5 Hz state message |

---

**Plan approved 2026-09-21. Work starts at M0 (the two spikes).**
