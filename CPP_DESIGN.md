# Writing a mission

How to write a C++ project that flies this simulation — and flies a real DJI
Matrice 4E, unchanged.

---

## Contents

1. [The one rule everything else follows from](#1-the-one-rule-everything-else-follows-from)
2. [The nine rules](#2-the-nine-rules)
3. [The skeleton](#3-the-skeleton)
4. [`project.conf`](#4-projectconf)
5. [demo_orbit_mission, walked through](#5-demo_orbit_mission-walked-through)
6. [Command topics](#6-command-topics)
7. [Services](#7-services)
8. [Telemetry: what to trust](#8-telemetry-what-to-trust)
9. [Building and running](#9-building-and-running)
10. [Debugging a mission](#10-debugging-a-mission)
11. [Porting to hardware](#11-porting-to-hardware)

---

## 1. The one rule everything else follows from

> **A mission reads and writes `/wrapper/psdk_ros2/*` and nothing else.**
> Never `/fmu/*`. Never `/drone/*`. Never a Gazebo service.

That is the whole reason this simulator exists. `/wrapper/psdk_ros2` is the
interface DJI's `psdk_ros2` wrapper presents on a real Matrice 4E's Manifold.
A binary that talks only to it does not know whether PX4 and Gazebo are
underneath, or an actual aircraft — so the mission you debug on your desk is
the mission that flies.

Reach past it — subscribe to `/fmu/out/vehicle_local_position` because it is
right there and has better data — and you have written something that works
beautifully in simulation and cannot be deployed. There is no `/fmu` on an
aircraft.

---

## 2. The nine rules

Each of these exists because breaking it produces a specific, confusing
failure. The failure is named.

**1. PSDK surface only.** See above.

**2. Authority, then takeoff. Never arm.**
```cpp
obtain_ctrl_authority   →  wait ~2 s  →  takeoff
```
`takeoff` arms the aircraft itself. **Do not call `turn_on_motors`** — that
leaves a drone spinning on the ground with nothing to do. Release authority
when the mission ends.

**3. Send no setpoints during the automatic takeoff.** Publishing a velocity
setpoint mid-climb switches PX4 to offboard and cuts the automatic takeoff
short. Wait until `display_mode` is no longer `AUTO_TAKEOFF` *and*
`height_above_ground` has settled.

**4. The automatic takeoff does not reach your altitude.** `takeoff` is a
`Trigger` carrying no altitude; it stops at `MIS_TAKEOFF_ALT` (1.8 m for the
m4e, often ~1.1 m in practice). Every metre above that is yours to fly, with a
`CLIMB` step.

**5. Never block in a callback.** Use `async_send_request` with a completion
lambda. `spin_until_future_complete` inside a timer callback **deadlocks** —
the answer arrives on the thread that is waiting for it.

**6. Stream setpoints at a fixed tick, 20 Hz minimum. Count ticks, not
seconds.** PX4 leaves offboard when setpoints stop. And under load this
simulation runs slower than real time, so wall-clock timing drifts against the
aircraft's actual behaviour; tick counts do not.

**7. Best-effort QoS on telemetry, reliable on commands.** A reliable
subscription **never matches** PX4's best-effort publishers and you will see
silence with no error.

**8. A step machine, not a sequence of sleeps.** `enum Step` + `goToStep()` +
a per-step timeout that *lands the aircraft* rather than hanging. Log once a
second so a stuck step is visible.

**9. Land, then release authority — and then exit.** Send zero velocity for
~2 s first: landing while still sliding is how an aircraft tips over on
touchdown. Then stop publishing entirely so offboard does not fight the
landing. Finally call `rclcpp::shutdown()`.

> **Rule 9's last clause is not tidiness.** walkerd treats a running process as
> a running mission, and a running mission holds the flight lock: it locks out
> the next project, locks out replay, and keeps QGC in observer mode. A mission
> that reaches DONE and keeps spinning holds the simulation hostage until
> somebody notices.

---

## 3. The skeleton

```
projects/my_mission/
├── package.xml          <name> MUST equal the directory name
├── CMakeLists.txt       exactly one add_executable
├── project.conf         launcher settings, read as data, never sourced
├── src/main.cpp
└── rviz/my.rviz         optional
```

> **The package name must equal the directory name.** V1 allowed them to
> differ, and one project's `package.xml` actually declared *another
> project's* name — so two directories built one package and colcon installed
> whichever it saw last. Walker's scanner reports a mismatch rather than
> letting it through.

The seven steps every mission shares:

```cpp
enum Step { WAIT_FOR_DATA, GET_AUTHORITY, TAKEOFF, CLIMB,
            /* YOUR MISSION */, LAND, DONE };
```

Copy `projects/demo_arm_takeoff` for the skeleton alone, or
`projects/demo_orbit_mission` for the skeleton plus a worked flight pattern.

---

## 4. `project.conf`

`KEY=value`, one per line, `#` starts a comment. **Read as data, never
sourced** — a project you cloned from a colleague should not run as a shell
script just to reveal which camera profile it wants.

| key | values | meaning |
|---|---|---|
| `CAMERAS` | `none` `fisheye` `payload` `all` | the profile walker suggests for this mission |
| `SETPOINT` | `velocity` `position` `body` | which frame this mission steers in |
| `RVIZ` | path | an RViz config to offer when it runs |
| `TAKEOFF_ALT` | metres | checked against the airframe's `MIS_TAKEOFF_ALT` |
| `TIMEOUT` | seconds | walker stops the mission and reports if it overruns |

Walker shows these on the project picker, because "never built" tells you
nothing about which mission you actually want:

```
 › demo_arm_takeoff     never built   cameras=none · setpoint=velocity
   demo_camera_track    never built   cameras=payload · setpoint=body · rviz
   demo_orbit_mission   never built   cameras=none · setpoint=velocity
```

Unknown keys are ignored on purpose: an older checkout must not choke on a key
a newer project uses.

---

## 5. `demo_orbit_mission`, walked through

The reference implementation. Every other mission is written in its shape.

### Subscribing — note the QoS

```cpp
rclcpp::QoS qos(10);
qos.best_effort();                          // rule 7

flight_status_sub_ = create_subscription<FlightStatus>(
  "/wrapper/psdk_ros2/flight_status", qos,
  [this](const FlightStatus::SharedPtr msg) {
    flight_status_ = msg->flight_status;
    got_telemetry_ = true;                  // gates WAIT_FOR_DATA
  });
```

### Commanding — note that it does not wait

```cpp
void sendCommand(rclcpp::Client<Trigger>::SharedPtr client, std::string name)
{
  if (!client->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is not available -- is the bridge running?",
                name.c_str());
    return;
  }
  client->async_send_request(                       // rule 5
    std::make_shared<Trigger::Request>(),
    [this, name](rclcpp::Client<Trigger>::SharedFuture future) {
      auto reply = future.get();
      RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(),
                  reply->success ? "ok" : reply->message.c_str());
    });
}
```

### The tick — 20 Hz, counting ticks

```cpp
const double TICK_HZ = 20.0;                        // rule 6
double secondsInStep() { return ticks_in_step_ / TICK_HZ; }
```

### TAKEOFF — the step that sends nothing

```cpp
case TAKEOFF:
  // Send NO velocity commands here: they would switch PX4 to OFFBOARD in the
  // middle of the automatic climb and cut it short.              // rule 3
  if (isFlying() && height_ >= min_takeoff_height_ &&
      display_mode_ != DisplayMode::DISPLAY_MODE_AUTO_TAKEOFF &&
      secondsInStep() > 5.0) {
    goToStep(CLIMB);
  } else if (secondsInStep() >= takeoff_timeout_) {
    goToStep(LAND);                                              // rule 8
  }
  break;
```

### CLIMB — where offboard actually engages

```cpp
case CLIMB:
  publishVelocity(0.0, 0.0, climbSpeed(), 0.0);
  if (weAreSteering() && settled_ticks_ >= 2 * TICK_HZ) {
    goToStep(ORBIT);
  } else if (!weAreSteering() && secondsInStep() >= offboard_timeout_) {
    RCLCPP_ERROR(get_logger(), "PX4 never switched to SDK_CTRL after %.0f s",
                 offboard_timeout_);
    goToStep(LAND);
  }
```

`weAreSteering()` is `display_mode == DISPLAY_MODE_NAVI_SDK_CTRL`. It becomes
true **about 1.5 s after you start sending**, so it is something to check for
*after* commanding, never a precondition before.

> If you see "PX4 never switched to SDK_CTRL", **the bridge is not running**.
> Press `b`. In this version the bridge is ready for missions the moment it
> starts — there is no mode to set.

### ORBIT — the actual work

```cpp
double angle = 2.0 * M_PI * (seconds / lap_seconds_);
double speed = 2.0 * M_PI * radius_ / lap_seconds_;
publishVelocity(speed * -std::sin(angle),      // east
                speed *  std::cos(angle),      // north
                climbSpeed(),                  // height stays closed-loop
                yaw_rate);
```

Open loop sideways, closed loop vertically. Drifting up or down is not
acceptable; a circle that does not quite close is fine for a demo. Closing the
horizontal loop against `position_fused` is the natural next exercise.

### LAND and DONE

```cpp
case LAND:
  if (secondsInStep() < 2.0) {
    publishVelocity(0.0, 0.0, 0.0, 0.0);      // settle first — rule 9
  } else if (!isArmed() && secondsInStep() > 4.0) {
    sendCommand(release_client_, "release_ctrl_authority");
    goToStep(DONE);
  } else if (!isComingDown() && ticks_in_step_ % 100 == 1) {
    sendCommand(land_client_, "land");
  }
  break;

case DONE:
  if (ticks_in_step_ == 1) {
    RCLCPP_INFO(get_logger(), "flight finished, shutting down");
    rclcpp::shutdown();                       // rule 9 — frees the flight lock
  }
  break;
```

---

## 6. Command topics

All five are `sensor_msgs/msg/Joy`. Stream continuously; PX4 leaves offboard
when they stop.

**Preconditions:** `obtain_ctrl_authority` succeeded, and the aircraft is
airborne.

| topic | `axes` | frame |
|---|---|---|
| `flight_control_setpoint_ENUvelocity_yawrate` | `[v_east, v_north, v_up, yaw_rate]` | ground (ENU) |
| `flight_control_setpoint_ENUposition_yaw` | `[east, north, up, yaw]` | ground (ENU) |
| `flight_control_setpoint_FLUvelocity_yawrate` | `[v_forward, v_left, v_up, yaw_rate]` | body (FLU) |
| `flight_control_setpoint_rollpitch_yawrate_thrust` | attitude-rate control | — |
| `flight_control_setpoint_generic` | raw PSDK flag byte + 4 axes | — |

> ### ⚠ The yaw-rate unit trap
>
> The psdk_ros2 documentation says **radians**/s. The converter that actually
> reads these messages (`psdk_enu_velocity_to_setpoint` in
> `bridge/simty/converters.py`) takes **degrees**/s.
>
> **Believe the code.** A mission written against the documentation yaws
> roughly 57× too slowly and looks like a tuning problem.

> **The last two are not converted in this simulator.** PX4 takes attitude and
> thrust on a different message entirely, and `generic` is a raw PSDK flag
> byte. A route that accepted them and quietly did nothing would be worse than
> their absence, so they are absent. Use one of the first three.

Choose the frame that matches what you measure. A camera measures everything
relative to the airframe, so a visual-tracking mission should steer in FLU
(`demo_camera_track` does). A pattern defined over the ground should steer in
ENU.

---

## 7. Services

All 56 published services are served. The ones a mission actually needs:

| service | type | notes |
|---|---|---|
| `obtain_ctrl_authority` | `Trigger` | first, always |
| `release_ctrl_authority` | `Trigger` | last, always |
| `takeoff` | `Trigger` | arms the aircraft itself |
| `land` | `Trigger` | |
| `start_go_home` / `cancel_go_home` | `Trigger` | |
| `start_force_landing` | `Trigger` | the E-STOP path |
| `turn_on_motors` / `turn_off_motors` | `Trigger` | **do not use `turn_on_motors`** — rule 2 |
| `start_perception` | `PerceptionStereoVisionSetup` | gates the stereo streams |
| `camera_*` | various | 29 camera services: capture, optics, zoom, storage |
| `set_*_obstacle_avoidance` | `SetObstacleAvoidance` | five directional pairs, not one switch |

Every served service has **ROS 2 service introspection on**, so a call is
recorded through its `<service>/_service_event` topic. Without that, a
recording could replay a sortie's setpoints but never the `takeoff` that
started it.

---

## 8. Telemetry: what to trust

**Live and faithful** — attitude, IMU, `position_fused`, `visual_odometry`,
velocities, altitudes, GNSS, `height_above_ground`, flight and display mode,
battery, `relative_obstacle_info`, RC.

**Live but simulation-only** — a real M4E does **not** publish these; do not
build a mission that depends on them: `gimbal_angles`, `gimbal_status`,
`home_point_altitude`, `battery`, `gps_control_level`, `landing_gear_status`.

**Not produced at all** — `tools/check_surface.py` reports these as a gap:
`acceleration_body_fused`, `acceleration_body_raw`,
`acceleration_ground_fused`, `esc_data`, `magnetic_field`, `rtk_velocity`,
`rtk_yaw`, `perception_camera_parameters`. They came from the aircraft's own
hardware on a real M4E, so there was never a conversion to inherit.

**Only while a camera is on** — `main_camera_stream`, `fpv_camera_stream`,
`perception_stereo_left_stream`, `perception_stereo_right_stream`. Enable the
lens from walker's `c` screen, and set `CAMERAS=` in your `project.conf` so
walker suggests the right profile.

Check the live surface at any time:

```bash
./walker shell
python3 /home/developer/ws/src/tools/check_surface.py --rates
```

---

## 9. Building and running

From walker: press `p`, pick your mission.

- **Enter** always rebuilds. When you are iterating that is what you want; the
  alternative is running yesterday's binary because a timestamp comparison
  disagreed with you.
- **Shift+Enter** (or `b`) reuses the existing build, compiling only when there
  is no binary.

The build runs **in the mission's own terminal**, so one window carries the
whole story — what is being built, the compiler's output, then the flight. A
failure says so in words:

```
=== demo_orbit_mission FAILED TO BUILD (exit 1) ===
    The compiler output is above. Fix it and press p again.
```

By hand:

```bash
./walker shell
cd ~/ws && colcon build --packages-select my_mission --symlink-install
source install/setup.bash
ros2 run my_mission my_mission --ros-args -p radius:=8.0
```

Walker marks a project **stale** when any source is newer than the binary,
because running a stale binary is the failure that wastes the most time: the
mission runs, behaves like the old code, and nothing says why.

---

## 10. Debugging a mission

**Read its terminal first** — it opens automatically and carries the build and
every log line.

| symptom | cause |
|---|---|
| stuck in `WAIT_FOR_DATA` | the bridge is not running — press `b` |
| takes off, then never steers | again the bridge; `display_mode` never reaches `SDK_CTRL` |
| yaws ~57× too slowly | you used radians. The converter takes **degrees** |
| a subscription that never fires | Reliable QoS against a Best Effort publisher — rule 7 |
| service call hangs the node | `spin_until_future_complete` in a callback — rule 5 |
| climbs to ~1.1 m and stops | you expected `takeoff` to reach your altitude — rule 4 |
| the next mission is refused | the last one never exited — rule 9 |
| code changes have no effect | you pressed Shift+Enter. Press **Enter** |

Then:

```bash
./walker doctor          # is the chain up, layer by layer
# press P in walker      # live rate of every link
```

Watch the flight in RViz (`v`), and enable a camera (`c`) to see what the
aircraft sees.

> **Do not debug with `ros2 topic hz`.** It subscribes Reliable, so it reports
> "no data" about topics publishing at 250 Hz. `ros2 topic info` disagrees with
> `ros2 topic list` for cache reasons. Use walker's link panel and
> `tools/check_surface.py`, which read the graph through rclpy.

---

## 11. Porting to hardware

If you followed rule 1, the binary is already portable. What differs:

| | simulation | real M4E |
|---|---|---|
| what runs the wrapper | `bridge/` in this container | DJI's `psdk_ros2` on a Manifold 3 |
| discovery | localhost only, domain 0 | the Manifold's network and domain |
| `MIS_TAKEOFF_ALT` | 1.8 m, from `walker.toml` | a property of the aircraft |
| the 7 missing topics | absent | **present** — your mission may use them |
| the simulation-only topics | present | **absent** — a mission that needs them will hang |
| timing | slower than real time under load | real time |
| failure | a container restart | an aircraft |

**Checklist before flying hardware:**

1. Does the mission read any topic in the "simulation-only" list? It will hang.
2. Does it assume the takeoff altitude? Read it, do not hard-code it.
3. Does every step have a timeout that **lands**, not one that hangs?
4. Does it release authority on every exit path, including failure?
5. Have you flown the same binary in `flat` and in `powerline`? Different
   timing shakes out wall-clock assumptions.
6. Does it exit when finished?

---

## The shipped missions

| project | adds |
|---|---|
| `demo_arm_takeoff` | the skeleton alone — takeoff, hold, land |
| `demo_orbit_mission` | an open-loop circle on ENU velocity — **the reference** |
| `demo_square_mission` | waypoints on ENU position |
| `demo_camera_track` | body-frame FLU velocity, and reads `main_camera_stream` |
