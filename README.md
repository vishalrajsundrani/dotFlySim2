# dotFlySim2

A simulator for DJI Matrice 4E inspection flights, driven from one keyboard.

You pick a world, an aircraft and which cameras should render; press a key; and
a PX4 + Gazebo simulation comes up inside Docker presenting the **full DJI PSDK
ROS 2 interface**. A C++ mission written against that interface flies the
simulation today and a real Matrice 4E tomorrow, unchanged.

```
┌ dotFlySim2 · walker ───────────────────────────────────────────────────────┐
│  WORLD     powerline                                          ⨯ sim up     │
│  DRONE     m4e                                                ⨯ sim up     │
│  CAMERAS   fisheye                                          [c] change     │
│  WINDOW    gazebo window                                      ⨯ sim up     │
│ ── units ──────────────────────────────────────────────────────────────────│
│    ● sim        running     308s                             [s] stop      │
│    ● bridge     running     288s                             [b] stop      │
│    ● project    running     demo_orbit_mission  ORBIT 14s    [p] stop      │
│    ● rviz       running     279s                             [v] stop      │
│ ── links ──────────────────────────────────────────────────────────────────│
│    ✔ gazebo clock   250.0 Hz      ✔ px4 attitude          50.0 Hz          │
│    ✔ px4 → xrce → ros 250.0 Hz    ✔ wrapper flight_status  2.0 Hz          │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## Contents

1. [What this is](#1-what-this-is)
2. [Quick start](#2-quick-start)
3. [Walker, feature by feature](#3-walker-feature-by-feature)
4. [What may run alongside what](#4-what-may-run-alongside-what)
5. [When a setting may be changed](#5-when-a-setting-may-be-changed)
6. [Cameras](#6-cameras)
7. [Adding a world, a drone, a mission](#7-adding-a-world-a-drone-a-mission)
8. [Troubleshooting](#8-troubleshooting)
9. [What is not here yet](#9-what-is-not-here-yet)

---

## 1. What this is

A **Docker image** holding Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, a
prebuilt PX4 SITL and QGroundControl — everything expensive and stable.

Everything you can *choose* is mounted from this repository and scanned on
every start: `models/`, `worlds/`, `projects/`, `bridge/`, `config/`. Adding a
world means putting a `.sdf` in `worlds/` and pressing `F5`. Nothing is baked
into the image, so nothing needs a rebuild.

**What is simulated faithfully**

| | |
|---|---|
| Flight dynamics | PX4 SITL, the same autopilot firmware family as a real airframe |
| The DJI interface | 56/56 published services, 31 of 41 telemetry topics live, all 3 convertible command topics |
| Aircraft scale | Body footprint within 2% of DJI's published dimensions; takeoff mass exact. See `reports/dotFlySim2_scale_audit.xlsx` |
| Camera optics | Field of view derived from DJI's published figures, per lens and per resolution tier |
| The inspection world | A 220 kV double-circuit line built from real engineering figures — ACSR Zebra conductor, 5 m phase spacing, 13 m ground clearance |

**What is approximated, and said so plainly**

- 7 wrapper telemetry topics are **not synthesised** (accelerations, `esc_data`,
  `magnetic_field`, some RTK). They came from the aircraft's own hardware on a
  real M4E and there was never a conversion to inherit. `tools/check_surface.py`
  reports them as a gap rather than a failure.
- 2 command topics (`..._generic`, `..._rollpitch_yawrate_thrust`) have no
  honest PX4 equivalent. A route that accepted them and did nothing would be
  worse than their absence.
- The forward fisheye pair is 90°×90°, not the real 90°×135°: the published
  stream is a square 704×704 image and matching that was judged more important
  than matching the lens.
- It is SITL on a laptop. Under a full load the simulation runs slower than
  real time — which is why missions count ticks, not seconds.

---

## 2. Quick start

**Prerequisites:** Docker, a graphical session, ~20 GB of disk. A GPU is
optional; without one Gazebo software-renders and you should keep cameras off.

```bash
# 1. build the image (~40 min, once)
cd dotFlySim2
docker build -t dotflysim2:latest .

# 2. check the machine is ready — every failure names its own fix
./walker doctor

# 3. run it
./walker
```

`./walker` starts the container and the supervisor itself; there is no separate
setup step. On exit it asks whether to leave the stack running or stop it.

**Your first flight, from the dashboard:**

| press | what happens |
|---|---|
| `w` | choose a world — `flat` is bare ground and fast; `powerline` is the inspection corridor |
| `d` | choose an aircraft — `m4e` |
| `s` | start the simulation. A Gazebo window opens, and a terminal showing PX4's console |
| `b` | start the bridge. The `/wrapper/psdk_ros2` surface appears; it is ready for missions immediately |
| `p` | choose a mission. **Enter** builds it and flies it |

The mission's terminal shows the build, then the flight:

```
=== building demo_orbit_mission (never built) ===
=== built demo_orbit_mission ===
=== running demo_orbit_mission ===
[INFO] telemetry is flowing, starting
[INFO] obtain_ctrl_authority: ok
[INFO] takeoff: ok
[INFO] reached 2.9 m, starting the circle
[INFO] circle finished after 40 s, landing
[INFO] landed and disarmed
[INFO] flight finished, shutting down
```

**Offline builds.** Drop `PX4-Autopilot.zip` and/or
`QGroundControl-x86_64.AppImage` into `vendor/` and the build uses them instead
of reaching out to GitHub. The zip must come from a *recursive clone with
`.git` kept* — GitHub's "Download ZIP" has neither submodules nor
`git describe` and cannot build. The Dockerfile detects a bad archive and falls
back to a clone rather than failing ten minutes into the compile.

---

## 3. Walker, feature by feature

### The command line

```
./walker              the dashboard (and it starts everything)
./walker up           start the container and supervisor without the TUI
   --restart-walkerd  stop the supervisor first. USE THIS after editing
                      walkerd/ or bridge/ — a running supervisor holds the
                      old code in memory
./walker down [--rm]  stop the container (--rm also removes it)
./walker doctor       every readiness check, each failure with its fix
   --deep             also PROVE discovery isolation, by publishing a topic
                      and confirming a separate container cannot see it
./walker status       container and mounts
./walker shell        an interactive shell inside the container
./walker dump         render one dashboard frame as text (for tests and CI)
```

### The dashboard

| key | action |
|---|---|
| `s` | start / stop the simulation |
| `b` | start / stop the bridge |
| `p` | choose a mission (or stop the running one) |
| `q` | QGroundControl |
| `v` | RViz |
| `c` | cameras — every lens individually |
| `w` `d` | world, drone |
| `g` | Gazebo window on/off |
| `t` | open a terminal on a unit |
| `P` | probe every link now |
| `F5` | rescan `models/ worlds/ projects/` |
| `?` | keys |
| `Q` | quit — asks whether to leave the stack running |

In the **project picker**: **Enter** rebuilds and flies; **Shift+Enter** (or
`b`) reuses an existing build and only compiles when there is no binary.

> Most terminals cannot tell Enter from Shift+Enter — both send byte 13 and the
> modifier is not on the wire. Walker asks the terminal for xterm's
> `modifyOtherKeys` mode, which VTE terminals (ptyxis, gnome-terminal) and
> xterm support. Where it is ignored, **`b` does the same thing**.

### A terminal per unit

`sim`, `bridge`, `project`, `rviz` and `qgc` each open their own window the
moment they start — PX4's live console, the bridge's route table, a mission's
build and step transitions. Closing a window **detaches**; it does not stop the
unit. Walker keeps showing the important lines from all of them in its own log
pane.

Walker tries `ptyxis`, `gnome-terminal`, `konsole`, `xfce4-terminal`, `kitty`,
`alacritty`, `wezterm`, `foot`, `x-terminal-emulator`, then `xterm`.
`SIM_TERMINAL='myterm -e {cmd}'` overrides the lot. With none of them, the
simulation still runs — you just read its output in walker.

---

## 4. What may run alongside what

Rows = what you are starting. Columns = what is already running.

| start ↓ / running → | sim | project | qgc | rviz |
|---|---|---|---|---|
| **sim** | — | n/a | ✅ | ✅ |
| **project** | ✅ required | ⛔ one at a time | ✅ QGC auto-locked | ✅ |
| **qgc** | ✅ required | ✅ starts locked | — | ✅ |
| **rviz** | ✅ | ✅ | ✅ | — |

A refusal always names what is holding the lock and the key that clears it:

```
project refused: one C++ project at a time   [stop the running project first (p)]
```

**QGC is locked, not warned.** While a mission holds the flight lock, a MAVLink
relay drops QGC's arm, disarm, mode-change, joystick, mission-upload and
parameter traffic before it reaches PX4 — while telemetry keeps flowing, so QGC
stays a complete instrument panel. The lock releases the moment the mission
ends, including when it ends by itself.

The table above lives in `walkerd/constraints.py`. Walker's help screen and
walkerd's enforcement both read it, so they cannot drift.

---

## 5. When a setting may be changed

Some settings are read when the simulation is **composed**, so changing them
under a running simulation would change nothing. Walker refuses those with a
reason instead of accepting them silently.

| setting | when | why |
|---|---|---|
| world, drone | sim **stopped** | composed at start; PX4 reads the airframe once, at boot |
| Gazebo window | sim **stopped** | the client starts alongside the server |
| QGC video | sim **stopped** | decides whether the video plugin is loaded |
| **cameras** | **any time** | subscription-driven — switchable mid-flight, and a set chosen while stopped is applied at start |
| lens, tier | sim **running** | nothing to switch between until the cameras exist |

A locked setting shows `⨯ sim up` where its key would be.

---

## 6. Cameras

**Rendering is demand-driven.** Every camera is `always_on=false` in the
composed model and its bridge row is lazy, so Gazebo renders a lens only while
something is subscribed. Switching a camera on *is* subscribing to it.

Measured on a running simulation:

| what is on | Gazebo CPU |
|---|---|
| nothing | ~41% |
| one fisheye | ~40% |
| two fisheyes | 99.8% |
| four cameras | 103.0% |
| all off again | 41.0% |

Press `c` for **all 19 lenses individually** — 3 payload lenses × 4 tiers
(photo, 4K, FHD, preview), plus 7 vision cameras (forward, backward and lateral
pairs, and downward). `1`–`4` apply the shortcuts `none`, `fisheye`, `payload`,
`all`; space toggles one.

**RViz is a camera switch too.** `config/rviz/cameras.rviz` carries one Image
display per camera, all disabled. Enabling a display subscribes, and
subscribing is what makes the lens render — so ticking a box turns that camera
on and unticking it gives the GPU time back.

> Every image display must be **Best Effort**. Image streams are
> `SensorDataQoS`; a reliable subscriber never matches them and shows an empty
> panel *with no error at all*. This is the single most common "the cameras are
> broken" report, and the cameras are fine.

---

## 7. Adding a world, a drone, a mission

### A world

Put a `.sdf` in `worlds/` and press `F5`. Optionally add
`worlds/<name>.walker.toml` for a title, the spawn coordinates and the scenery
it needs.

> **Do not declare system plugins in a world.** PX4's `server.config` already
> loads Physics, Sensors, Imu, NavSat and the rest; declaring them again loads
> each twice, which applies forces twice and makes the drone spin on the spot.
> This is the mistake a new world is most likely to make. See the header of
> `worlds/flat.sdf`.

### A drone

A directory in `models/` with `model.sdf`, `model.config` and a
`walker.toml` declaring `kind = "drone"`, its PX4 airframe parameters, its
attachments and its cameras. The airframe file is **generated at start** from
that manifest, so no image rebuild is involved. See `models/m4e/walker.toml`.

A model whose manifest fails to parse is still **listed**, with the parser's
line and column — hiding a broken thing is how you end up staring at a menu
wondering why the drone you just added is not in it.

### A mission

A directory in `projects/` with `package.xml`, `CMakeLists.txt` (exactly one
`add_executable`), `src/`, and optionally `project.conf`. **The package name
must equal the directory name.** See [CPP_DESIGN.md](CPP_DESIGN.md).

### Changing sizes

`reports/dotFlySim2_scale_audit.xlsx` compares every dimension against
real-world figures and names the file and parameter behind each one. The short
version: the whole powerline is regenerated from `tools/gen_powerline.py`, and
`<scale>` is silently ignored inside an SDF `<include>`, so a tower cannot be
resized from the world file.

---

## 8. Troubleshooting

**Run `./walker doctor` first.** Every check names its own fix.

| symptom | cause | fix |
|---|---|---|
| `RTPS_TRANSPORT_SHM Error: Failed to create segment`, or peers that never discover each other | `/dev/shm` too small; Fast DDS takes 32 MB per participant and V2 runs 8+ | `./walker down --rm && ./walker up` — walker starts it with `--shm-size=2g` |
| No Gazebo window | `DISPLAY` unset or not answering | `./walker doctor` reports it; `xhost +local:` on the host |
| Gazebo software-renders, everything crawls | no `/dev/dri/renderD128` | keep cameras off; `doctor` warns about this |
| RViz shows an empty image panel, no error | the display is Reliable; image streams are Best Effort | set **Reliability = Best Effort**, or use `cameras.rviz` |
| `Package 'x' not found` right after it compiled | a shell that sourced the overlay before the package existed | units source at spawn, so this should not happen; `./walker up --restart-walkerd` |
| A mission takes off then never steers | the bridge is not running | press `b`. The bridge is ready for missions the moment it is up — there is no mode to set |
| Editing `walkerd/` or `bridge/` changes nothing | the supervisor holds the old code in memory | `./walker up --restart-walkerd` |
| `PX4 server already running for instance 0` | a stale `/tmp/px4-sock-0` from a PX4 that was killed | the launcher removes it automatically; if a PX4 really is running, stop the sim unit |
| A mission runs the old code | you pressed Shift+Enter, which reuses the build | press **Enter**, which always rebuilds |

**A note on the ROS 2 CLI.** `ros2 topic hz` reports "no data" on topics
publishing at 250 Hz, because it subscribes Reliable and PX4 publishes Best
Effort. `ros2 topic info` disagrees with `ros2 topic list` because they take
different paths through the daemon cache. **Trust walker's link panel and
`tools/check_surface.py`**, which read the graph through rclpy.

---

## 9. What is not here yet

Stated plainly so nobody hunts for them:

- **rosbag recording and replay.** The `record` and `replay` keys appear in the
  keymap but the units are not implemented. Planned as milestone M7.
- **`demo_gnss_stereo_inertial`**, the factor-graph SLAM project that consumes
  the fisheye pair. Planned as milestone M9.

Everything else described above is built and verified. `VERSION2_PLAN.md`
carries the full design, every milestone's acceptance evidence, and the
findings behind each decision.

---

## Where things live

```
walker                  the launcher — start here
walkerui/               the host program: TUI, docker control, scanning, checks
walkerd/                the supervisor, inside the container, owning every process
bridge/                 the PSDK bridge (simty): routes, converters, services
simsupport/             the gimbal rig and sensor models
models/  worlds/        content, scanned every start
projects/               C++ missions
config/                 dds.xml, rviz configs
tools/                  the composer, the surface checker, the powerline generator
reports/                the scale audit
```

- [CPP_DESIGN.md](CPP_DESIGN.md) — writing a mission that also flies real hardware
- [ROS2_ARCHITECTURE.md](ROS2_ARCHITECTURE.md) — the node graph, every topic and service
- [VERSION2_PLAN.md](VERSION2_PLAN.md) — the design, and why each decision was made
