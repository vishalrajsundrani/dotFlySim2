# Camera architecture: how the simulation models the aircraft's cameras

Living document. Section 1 is the checklist and its state; section 2 onward
describes the architecture as it now stands and why each piece is the way it
is. Updated as each item lands.

Companion documents: `CAMERA_FIDELITY.md` (the survey that motivated this) and
`dotFlySim2_surface_comparison.xlsx` (the generated three-way comparison).

---

## 1. Checklist

### Phase 0 — unknowns to resolve
- [x] 0.1 Reconcile DJI's "six fisheye lenses" with the seven sensors modelled
- [ ] 0.2 Decode `/tf_static` from the aircraft bag for the real frame tree *(blocked: USB stick keeps unmounting)*

### Phase 1 — payload / main camera
- [x] 1.1 `camera_setup_streaming` honours all four request fields
- [x] 1.2 `frame_id` matches psdk_ros2's real behaviour
- [x] 1.3 Remove `camera_set_stream_source` (no upstream counterpart)
- [x] 1.4 Main-camera rate: reconcile the 5 / 11 / 23.5 Hz disagreement
- [x] 1.5 `camera_source` M4E_4K vs VIS selects the streamed tier
- [x] 1.6 Model the mid-stream resolution change

### Phase 2 — fisheye / perception
- [x] 2.1 Six lenses in the shared-corner layout, replacing seven
- [x] 2.2 `wideanglecamera` + equidistant projection
- [x] 2.3 Correct per-direction fields of view
- [x] 2.4 Direction pairs wired through `bridge.yaml` and `walker.toml`
- [x] 2.5 `config/perception_calib.yaml`, read by both sides
- [x] 2.6 `_PERCEPTION_SIMULATED` derived from what is actually rendered

### Phase 3 — walker
- [x] 3.1 Camera descriptions derived from the SDF, not hand-written
- [x] 3.2 Profile cost figures computed, not transcribed
- [x] 3.3 New camera-selection screen
- [x] 3.4 Show the live stereo direction
- [x] 3.5 Constraint: direction switching at run time, within what was enabled

### Phase 4 — verification
- [x] 4.1 `surface_compare.py` fails on surface drift
- [x] 4.2 Record a simulated bag and compare against the aircraft's
- [x] 4.3 This document completed

---

## 2. Findings that shaped the design

### 2.1 Six lenses, not seven sensors

DJI publishes "six HD low-light fisheye vision sensors" and an "omnidirectional
binocular vision system" with five sensing directions. Six lenses cannot
provide five directions at two lenses each, so the lenses must be **shared
between directions**. The published ranges confirm it:

| direction | range | implied baseline |
| --- | --- | --- |
| forward / backward | 0.4 – 22.5 m | shorter |
| lateral | 0.5 – 32 m | **longer** |

A longer range means a longer baseline. That is only possible if the lateral
pair is formed from a front lens and a rear lens, whose separation along the
airframe is larger than the side-to-side separation of the two front lenses.

So the layout is four lenses at the corners plus two below:

```
          front
   FL  o-----------o  FR        FRONT pair = FL + FR   (lateral baseline)
       |           |            REAR  pair = RL + RR   (lateral baseline)
       |           |            LEFT  pair = FL + RL   (longitudinal, longer)
       |           |            RIGHT pair = FR + RR   (longitudinal, longer)
   RL  o-----------o  RR        DOWN  pair = DL + DR
           rear
              DL  DR  (underside)
```

The previous model had seven sensors: two forward, two rear, **one** left,
**one** right and **one** down. The three singles could never have served
`start_perception`, which must publish a left *and* a right stream for the
chosen direction. This is the single most consequential correction in this
work: adding directions was never a wiring job.

This layout is an inference from the published specification, not from DJI
documentation of the physical placement, which does not state it. It is
recorded here as an inference. Decoding `/tf_static` from an aircraft
recording would confirm or refute it directly (item 0.2).

---

## 3. The architecture as it now stands

### 3.1 Three layers, and what each one decides

```
  models/m4e/model.sdf          six fisheyes + twelve payload sensors
        |                       geometry, projection, resolution, rate
        v
  tools/compose_sim.py          stages the model, makes every camera lazy,
        |                       writes ~/gz_runtime/bridge.yaml
        v
  ros_gz_bridge                 gz topic -> ROS topic, only while subscribed
        |
        v  /drone/perception/<s>/image_raw   /drone/camera/active/image_raw
        |
  bridge/simty (simty)          the PSDK wrapper surface
        |                       routes, converters, services
        v
  /wrapper/psdk_ros2/perception_stereo_{left,right}_stream   (mono8 704x704)
  /wrapper/psdk_ros2/main_camera_stream                      (rgb8, size varies)
```

Three different things choose what renders, and they are easy to confuse:

| decision | made by | when |
| --- | --- | --- |
| which lenses **exist** | `models/m4e/model.sdf` | authoring time |
| which lenses **render** | walker's camera screen | before a run, fixed for its duration |
| which direction **streams** | `start_perception`, a PSDK service | any time during a run |

The second is locked for a run so a mission's rendering cost cannot change
underneath it. The third must stay open because a payload switches direction
mid-flight on real hardware. They meet in one place: `start_perception`
refuses a direction whose lenses this run is not rendering, and says so by
name.

### 3.2 The perception lenses

Six `wideanglecamera` sensors with an `equidistant` lens, on `base_link` with
a pose (not on links of their own, which would add mass the airframe is tuned
without). Four at the corners yawed 45 degrees outward, two underneath.

| | resolution | rate | hfov | f (equidistant) |
| --- | --- | --- | --- | --- |
| `perception_fl/fr/rl/rr` | 704x704 | 20 Hz | 190 deg | 212.3 px |
| `perception_dl/dr` | 704x704 | 20 Hz | 160 deg | 252.1 px |

Each costs 9.9 MP/s to render. All six is 59.5 MP/s, but the sharing means a
session rarely needs all six: FRONT and REAR together are four lenses, and
they give LEFT and RIGHT for free.

**They render RGB and the wrapper publishes mono8.** `WideAngleCameraSensor`
supports exactly one pixel format, `RGB_INT8`; asking for `L8` logs
`Unsupported pixel format [1]` and produces nothing. The fisheye projection is
worth more than the conversion, so `converters.stereo_mono_image` applies
BT.601 luma on the way out and a consumer sees the mono8 the aircraft sends.

**Which sensor is the left eye is data, not a naming convention.** It lives in
`config/perception_calib.yaml`, generated from the model by
`tools/gen_perception_calib.py`, together with intrinsics, distortion and
`T_cam_imu` for each lens. Looking aft, the airframe's rear-right corner is on
your left: inferring the pairing from names gives negative disparity, which
reads as bad data rather than as a swapped pair.

### 3.3 The payload camera

`camera_setup_streaming` is the switch, and all four of its fields now do what
psdk_ros2 does with them:

| field | effect |
| --- | --- |
| `payload_index` | 1 (NO_1) streams on `main_camera_stream`, 7 (FPV) on `fpv_camera_stream`; 2 and 3 are refused |
| `camera_source` | 0 DEFAULT and 1 M4E_VIS stream the FHD tier, 3 M4E_4K the 4K tier; anything else is refused by name |
| `start_stop` | enables or disables that route |
| `decoded_output` | true only; encoded is refused with a message rather than served as RGB under an `h264` label |

`frame_id` reproduces psdk_ros2's own behaviour, including a bug. Its
`camera_source_str` is a `std::map` whose keys collide, because the enum
reuses small integers across aircraft, so the surviving names are an H20's
whatever is flying. Verified live:

| source | frame_id published | width |
| --- | --- | --- |
| 0 DEFAULT | `default_optical_link` | 1920 |
| 1 M4E_VIS | `h20_wide_optical_link` | 1920 |
| 3 M4E_4K | `h20t_ir_optical_link` | 3840 |

The first row matches `bag_000026` exactly. Set
`main_camera_true_frame_id` to emit M4E names instead, for a rig with no
hardware to match.

**The stream changes size at run time**, as it does on the aircraft, which
changed between 1440x1080 and 1920x1080 five times in 184 s with `frame_id`
constant throughout. A consumer that caches geometry from the first frame is
wrong on hardware and is now wrong here too.

**Rate is 23.5 Hz**, measured from the aircraft, replacing a 5 Hz cap that
made the simulation 4.7x slower than the thing it imitates.

---

## 4. What was wrong before, and how it was found

Each of these was a real defect, and the last three were found only by running
the simulation rather than by reading it.

**Three of five directions could never have worked.** The model had seven
sensors: a forward pair, a rear pair, and a single camera each for left, right
and down. `start_perception` must publish a left *and* a right stream, so
LEFT, RIGHT and DOWN were unreachable by construction. Adding directions was
never the wiring job it looked like.

**The fisheyes were not fisheyes.** All seven used Gazebo's default
rectilinear projection, including the 160 degree downward camera, where a
rectilinear projection is not merely inaccurate but undefined.

**`camera_set_stream_source` did not exist upstream.** There is no
`CameraSetStreamSource.srv` in psdk_ros2 and no handler for it. A mission
written against it would fail on the aircraft with "service not available".
`tools/surface_compare.py --check` now fails the build on that class of drift,
in that direction only: a service the simulation lacks is a visible
inconvenience, one it invents is a trap.

**Three hand-written tables had drifted from the model.** `cameras.py` said
"90 deg" for a pair that is 90 by 135; the payload tier detail said
"1920x1080 @30" for a stream that is about to change size at run time; and the
`payload` profile was advertised at ~1074 MP/s when the profile enables one
lens at 62 MP/s, a 17x overstatement that steers people away from the profile
they want. All three are now derived from the model through
`tools/sdf_cameras.py`.

**A fisheye is not a `camera`, in three places.** gz calls the sensor
`wideanglecamera`, and three separate pieces of code tested for the string
`"camera"`: `relax_cameras` (so the six lenses silently stayed `always_on`,
rendering 60 MP/s nobody asked for), the `camera_info` row emitter, and
`_TYPES` in the bridge-config generator. The third was the worst: it dropped
the six sensors from the generated bridge config entirely, so the ROS topics
never existed and `start_perception` refused every direction with a message
that looked like a configuration problem. A `grep` for `== "camera"` found two
of the three; the dict lookup was found only by running it.

**`config/bridge.yaml` is not read at run time.** `compose_sim.py` writes the
operative file from scratch every start. The repository copy is a reference
example, and now says so at the top, because a maintained-looking file that
nothing loads is where the next person will make a change that does nothing.

---

## 5. Reproducing and extending

```bash
# after any change to the lenses in models/m4e/model.sdf
tools/gen_perception_calib.py                 # regenerate the calibration

# check the wrapper surface has not drifted from psdk_ros2
tools/surface_compare.py --bag <aircraft bag> --check
```

Adding a direction now means adding a pair of sensors to `models/m4e/model.sdf`
and re-running `gen_perception_calib.py`. The bridge config, walker's camera
screen, the cost figures and `start_perception`'s notion of what is available
all follow from the model; none of them has a list to update.

## 6. Still open

- **0.2** `/tf_static` from an aircraft recording, to confirm or refute the
  corner layout against the real frame tree. The USB stick holding
  `bag_000026` unmounted itself three times during this work, which also
  explains the three multi-second recorder stalls inside the bag.
- **Camera-to-IMU time offset.** The aircraft stamps
  `main_camera_stream` with `get_clock()->now()` in the decode callback, so
  the stamp is publication time, not capture time, and carries unmodelled
  latency. The simulation stamps at render. Any visual-inertial work must
  estimate this offset rather than assume it is zero.
- **The accelerometer gravity difference** from `IMU_CALIBRATION.md` 7a:
  the aircraft removes gravity from `imu.linear_acceleration`, the simulation
  does not. Unchanged here, and it matters more to a visual-inertial system
  than anything in this document.
