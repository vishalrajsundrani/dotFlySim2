# IMU calibration — matching the simulated Matrice 4E to a real one

The simulation's IMU noise was inherited from PX4's x500 reference airframe:
a 2 kg research quadcopter with whatever MEMS part its author had to hand. The
Matrice 4E is not that aircraft. This document records how those numbers were
replaced with ones measured from a real M4E, what went wrong on the way, and
what the result is and is not worth.

Everything here is reproducible from two commands, listed at the end.

---

## 1. The question, stated precisely

"Make the simulated IMU match the real one" can mean two different things, and
they do not have the same answer:

- make `models/m4e/model.sdf` contain the physical sensor's noise figures, or
- make `/wrapper/psdk_ros2/imu` **publish** what the aircraft's topic publishes.

The second is the one that matters. Nothing in this project consumes the SDF;
missions, SLAM and QGC consume the wrapper topic. And the two are not the same
number, because between them sit Gazebo's sensor model, PX4's filtering, and
the bridge's 50 Hz rate cap. Section 7 shows that the chain multiplies the
noise by about 1.9x, so writing the measured value into the SDF would have
left the simulation nearly twice as noisy as the aircraft.

So the target is the published topic, and the SDF is only the knob.

---

## 2. Source data

`/run/media/xenon/7981-2572/bag_000026/` — a 15 GB rosbag2/MCAP recording from
a physical Matrice 4E.

| property | value |
| --- | --- |
| duration | 184.1 s |
| messages | 193,277 across 58 topics |
| files | 4 `.mcap` + `metadata.yaml`, all one recording |
| IMU channels | `angular_rate_body_raw`, `acceleration_body_raw`, `imu`, and 3 fused |
| gyro / accel messages | 7,989 / 7,981 |

The bulk of the 15 GB is camera data. The IMU is a few megabytes inside it.

---

## 3. What was built

| file | role |
| --- | --- |
| `tools/imu_from_bag.py` | reads a bag, finds a stationary segment, characterises the IMU, writes `model.sdf` |
| `tools/imu_report_xlsx.py` | turns the tool's JSON into `reports/dotFlySim2_imu_calibration.xlsx` |

They are separate on purpose: re-rendering the report must never require
re-reading 15 GB off a USB stick, and the spreadsheet's numbers are then
provably the same numbers the tool printed.

---

## 4. The process, step by step

1. **Read the bag.** MCAP is parsed directly (see error E1). The summary
   section yields a chunk index; each chunk's per-channel message index gives
   byte offsets within the decompressed chunk, so only IMU messages are
   deserialised and the camera payloads are never decoded.
2. **Establish the true publication rate** from the mean of non-dropout
   intervals (E3).
3. **Find a segment where the aircraft was genuinely still** (E4).
4. **Estimate the white-noise density** from the flat top of the spectrum
   rather than from the segment's variance (E6).
5. **Convert to the simulation's sample rate.** A standard deviation is
   meaningless without the rate it was sampled at:
   `density = sigma / sqrt(rate)`, then `sigma_sim = density * sqrt(250)`.
6. **Check that the conversion is legal** by testing the spectrum for roll-off
   (E7).
7. **Write `model.sdf`**, run the simulation, record its published IMU, and
   measure it with the same tool.
8. **Correct for the chain gain** and repeat until the published topics agree
   (section 7).

---

## 5. Errors encountered, and what each one cost

These are recorded because several of them produced *confident wrong answers*
rather than failures, which is the dangerous kind.

### E1 — No MCAP library, and no way to install one
`import mcap` failed; `pip install mcap` is refused by PEP 668 on this host.
**Fixed** by writing an MCAP reader against the format specification (~150
lines). Python 3.14 supplies `compression.zstd`, which the chunks need.

### E2 — Chunk-level filtering saved nothing
The plan was to skip chunks that contain no IMU channel. In practice rosbag2
writes chunks by time, so each mixes ~28 topics and **105 of 106 chunks
contained IMU data**. Skipping saved 1%.
**Fixed** by changing the goal: the chunk still gets decompressed, but the
per-channel message index is used to slice out only IMU messages, so no image
is ever deserialised. Full 15 GB read: ~100 s.

### E3 — Three sample-rate estimators, three different answers
| estimator | result | why it is wrong |
| --- | --- | --- |
| `message_count / duration` | 43.4 Hz | 23.6 s of the recording is lost to three write stalls |
| median interval | 52.5 Hz | the interval distribution is skewed; a hard floor at the sensor period and a long tail of late packets pull the median below the mean |
| **mean of non-dropout intervals** | **49.75 Hz** | correct |

I had earlier reported 43.4 Hz to the user and suggested the bridge's 50 Hz cap
should be lowered to match. That was wrong: the aircraft runs at 50 Hz and the
existing cap was right. The dropouts are three gaps of 6.1 s, 7.1 s and 7.6 s —
almost certainly the recorder stalling while writing 81 MB/s of camera data to
a USB stick.
**Fixed** in `sample_rate()`, which now excludes intervals longer than 3x the
rough period and takes the mean of the rest.

### E4 — The stationary search returned a flying aircraft
The first implementation took a fixed 10 s window and returned the least-bad
one. It reported a gyro sigma of 0.0344 rad/s — 70x the true figure — with a
lag-1 autocorrelation of 0.95 and no complaint.

The cause: this bag's only still moments are **6.3 s before takeoff** and
**1.9 s after landing**. No 10 s window fits inside either, so every candidate
contained climb-out, and "least bad" was still flight.

**Fixed** by removing the assumption. A 1 s probe window now sweeps the whole
flight, "still" is defined relative to the quietest probe seen, the longest
contiguous run of still probes becomes the segment, and if that run is shorter
than the caller will accept the tool **refuses and says what it found**:

```
longest stationary run is 6.3s but 10.0s was required.
  stationary runs found: 6.3s at t+0.0s, 1.9s at t+181.8s
  re-run with --window 6 to accept the longest run, or --segment START:END
```

This was the most valuable fix in the exercise. A tool that returns a
confidently wrong number is worse than one that stops.

### E5 — `acceleration_body_raw` is not raw
Its mean magnitude over a stationary segment is 0.17 m/s², not 9.81. DJI
subtracts gravity before publishing. (`acceleration_body_fused`, by contrast,
reads -1.0008 on z: it is in **g**, and includes gravity. The two topics do not
even share units.)

**Consequence, not fixable:** accelerometer bias cannot be measured from this
bag. The residual mean mixes sensor bias with the flight controller's
gravity-removal error, and nothing separates them. The accelerometer `<mean>`
values are therefore left at zero and the file says why. Only a six-position
calibration on a known-level fixture would give them.

### E6 — Variance is the wrong estimator for the accelerometer
Using the stationary segment's standard deviation assumes everything in it is
sensor noise. For the gyro that holds. For the accelerometer it does not:

| axis | power below 5 Hz | power above 15 Hz |
| --- | --- | --- |
| gyro x/y/z | 23–28% | 25–30% |
| accel x | **67%** | 9% |
| accel y | 56% | 14% |
| accel z | 17% | 54% |

A parked aircraft sways on its legs in the wind. That is real motion of a real
airframe, and counting it as sensor noise would have made the simulated
accelerometer 2.4x noisier than the part actually is.

**Fixed** by `noise_density_psd()`, which reads the density off the flat part
of the spectrum between 35% and 95% of Nyquist. The tool also reports a
"redness" ratio — how much total power exceeds the white-noise level — so the
contamination is visible rather than silently absorbed.

### E7 — The first legality check blamed the wrong thing
I used lag-1 autocorrelation to decide whether rescaling to 250 Hz was valid,
and it flagged accel x (0.87) as "low-pass filtered before recording". That
conflates two different phenomena: a red-but-unfiltered signal (an airframe
swaying) also has high lag-1 correlation, and a perfectly flat plateau.

**Fixed** by testing the spectrum for actual roll-off — median power in the top
band against the middle band. This correctly reports that accel x and accel y
*are* filtered (top/mid 0.27 and 0.30) while the gyro is not, which is a
different and more defensible conclusion than lag-1 gave.

### E8 — A claim in a code comment that was not true
I wrote that the aircraft's `imu` topic is "byte-identical" to the two raw
topics, having seen their means agree. Checked properly, it is not: the topics
have different message counts (7,988 / 7,989 / 7,981) and stamps a median 5 ms
apart. They are separate samplings of the sensor, not copies.

**Fixed** by checking the property the tool actually relies on — that the
*noise densities* agree, which they do to within 2% on all six axes — and
rewriting the comment to claim only that, with the numbers and an explicit
warning not to use the substitution for anything time-aligned.

### E9 — `pkill -f "gz sim"` killed its own shell, and invalidated a whole run
After writing corrected values I restarted the simulation and re-measured. The
published noise was **identical** to before — the correction appeared to have
no effect whatsoever.

The cause was not the physics. The command

```bash
docker exec dotflysim2 bash -lc 'pkill -f ROS_Bridge_Simty; ...; pkill -f "gz sim"; ...'
```

has the string `gz sim` in *its own* command line, so `pkill -f "gz sim"`
matched the shell running it and killed the chain mid-way. Gazebo survived.
`start_sim.sh` then found a running world and attached to it, so the "restarted"
simulation was the **old world with the old model still loaded**. The proof was
in the Gazebo IMU's own covariance field: 1.5697e-06, whose square root is
0.0012529 — the previous iteration's value, six minutes older than the staged
model file.

**Fixed** with the bracket trick already used elsewhere in this repo:
`pkill -f "[g]z sim"`. The regex matches the process but not the literal string
in the shell's own command line. The invalid recording was discarded and the
run redone.

This is the second time this project has been bitten by a `-f` self-match, the
first being a `pgrep` that reported a dead Gazebo as healthy. It is worth
treating `pkill -f` / `pgrep -f` with a bare pattern as a bug on sight.

### E10 — The report compared against its own output
The "before" column of the spreadsheet read `model.sdf` at measurement time —
which, after the first `--emit`, was no longer the x500 baseline but the tool's
own first iteration. The comparison looked reassuringly small and meant
nothing.
**Fixed** with `--baseline-sdf`, pointed at the file as it was at `git HEAD`.

### E11 — Minor
`docs/IMU_CALIBRATION.md` was referenced in `model.sdf` before noticing this
repo keeps its markdown at the root. Corrected. The decoded-series cache was
also built before the `imu` topic was added to the reader, so it had to be
rebuilt once (hence `--refresh`).

---

## 6. Results

Measured from 6.3 s of stationary data at 49.75 Hz, converted to the sensor's
250 Hz, then corrected for the chain gain of section 7.

| parameter | before (x500) | after (measured M4E) | factor |
| --- | --- | --- | --- |
| `angular_velocity.x` stddev | 1.12512e-03 | 6.63256e-04 | 0.59 |
| `angular_velocity.y` stddev | 1.12512e-03 | 6.05421e-04 | 0.54 |
| `angular_velocity.z` stddev | 1.12512e-03 | 6.08367e-04 | 0.54 |
| `linear_acceleration.x` stddev | 1.58306e-02 | 2.26754e-03 | **0.14** |
| `linear_acceleration.y` stddev | 1.37570e-02 | 2.50024e-03 | **0.18** |
| `linear_acceleration.z` stddev | 1.86445e-02 | 1.13418e-02 | 0.61 |
| `angular_velocity.x` mean | 0 | -5.651e-04 | measured bias |
| `angular_velocity.y` mean | 0 | +8.837e-04 | measured bias |
| `angular_velocity.z` mean | 0 | -4.249e-04 | measured bias |

Underlying noise densities of the real aircraft:

| axis | density | unit |
| --- | --- | --- |
| gyro x / y / z | 7.92e-05 / 6.94e-05 / 7.27e-05 | rad/s/√Hz |
| accel x / y / z | 2.78e-04 / 2.97e-04 / 1.33e-03 | m/s²/√Hz |

**The inherited gyro figures were nearly right** — within 11% before the chain
correction. **The accelerometer figures were wrong by up to 7x.** The real
part is far quieter laterally than the x500's, and its z axis is ~4.7x noisier
than x and y, which is ordinary for MEMS accelerometers.

---

## 7. The chain gain, and why open-loop would have failed

What is written into the SDF is not what a consumer receives. Measured on a
147 s simulated recording, with the drone sitting on the ground:

```
SDF stddev  ->  Gazebo @250 Hz  ->  PX4 SensorCombined  ->  bridge @50 Hz  ->  published
```

the published noise density came out **1.85x to 1.94x higher** than the SDF
value implied, uniformly across all six axes. The dominant mechanism is
aliasing: Gazebo generates noise across the sensor's full 125 Hz bandwidth, and
the decimation to 50 Hz is not anti-aliased, so that power folds into 25 Hz.
The theoretical ceiling for pure aliasing is sqrt(250/49.3) = 2.25; the observed
1.9 is that, partly undone by PX4's own low-pass.

Guessing this factor from theory would be fragile — it depends on PX4 filter
parameters and the bridge's rate cap, both of which can change. So it is
measured, by `--calibrate`, and the loop is closed.

**Verification.** After applying the correction, the simulation was restarted
(properly, this time — see E9) and re-recorded for 148 s:

| axis | aircraft | simulation | ratio |
| --- | --- | --- | --- |
| gyro.x | 7.924e-05 | 7.808e-05 | 0.99 |
| gyro.y | 6.941e-05 | 6.995e-05 | 1.01 |
| gyro.z | 7.269e-05 | 7.111e-05 | 0.98 |
| accel.x | 2.776e-04 | 2.802e-04 | 1.01 |
| accel.y | 2.973e-04 | 2.980e-04 | 1.00 |
| accel.z | 1.328e-03 | 1.370e-03 | 1.03 |

All six axes agree within 3%. The chain gain measured on the second run
(1.83–1.95) matched the first (1.85–1.94), confirming the chain is linear and
one correction suffices.

---

## 8. What was deliberately not determined

A calibration is only as honest as its list of things it could not measure.

- **Bias instability and correlation time** stay at their inherited x500
  values. Reading them off an Allan curve requires the curve to turn over,
  which needs averaging times of tens of seconds to minutes; 6.3 s of stillness
  cannot reach that. On four of the six axes (all three gyro, accel y) the
  Allan minimum sits at the right-hand edge of the curve — the signature of a
  measurement that has not converged. On the other two it sits at tau = 0.42 s
  and 1.61 s, which are not bias-instability floors but noise in a curve
  computed from 313 samples. Either way a fitted number would have been
  invention, and the tool refuses to adopt one below a 600 s segment regardless
  of how convincing the fit looks. **To fix: 30+ minutes of the aircraft
  powered on and stationary.**
- **Accelerometer bias** — unmeasurable from this bag (E5).
- **Gyro bias is measured but weakly constrained in time.** Within the 6.3 s
  window the standard error is ~6% of the value, but a gyro bias is not a
  constant. Measured over successive 30 s windows of a simulated run, the
  published mean wandered by several times the injected offset. The static
  bias is transmitted — a 150 s average reproduced the injected values to
  within 3–12% — but no short window should be read as *the* bias.
- **Temperature dependence** — not addressed. The bag is one flight at one
  ambient temperature.
- **Scale factor, axis misalignment, g-sensitivity** — not addressed. These
  need a rate table, not a flight recording.

---

## 9. Reproducing this

```bash
# characterise the aircraft and write models/m4e/model.sdf
tools/imu_from_bag.py /path/to/bag_000026 \
    --window 6 --cache /tmp/phys.npz --json /tmp/phys.json --emit

# ... run the simulation, record /wrapper/psdk_ros2/imu for ~150 s on the
# ground, then measure it the same way ...
tools/imu_from_bag.py bags/imu_verify --window 5 --json /tmp/sim.json

# close the loop: correct the SDF for what the chain does to it
tools/imu_from_bag.py /path/to/bag_000026 \
    --window 6 --cache /tmp/phys.npz --calibrate /tmp/sim.json \
    --json /tmp/phys_cal.json --emit

# build the spreadsheet
tools/imu_report_xlsx.py /tmp/phys_cal.json --sim-json /tmp/sim3.json \
    --baseline-sdf <old model.sdf> -o reports/dotFlySim2_imu_calibration.xlsx
```

`--cache` keeps the decoded IMU series so that iterating on the analysis does
not re-read the bag.

## 10. Files touched

| file | change |
| --- | --- |
| `models/m4e/model.sdf` | IMU noise replaced with measured values; provenance comment added |
| `tools/imu_from_bag.py` | new |
| `tools/imu_report_xlsx.py` | new |
| `reports/dotFlySim2_imu_calibration.xlsx` | new, 7 sheets, 12 charts |
| `bags/imu_verify`, `bags/imu_verify3` | the two valid verification recordings — **not committed**, `bags/*` is gitignored as data rather than source |

Nothing in `bridge/` changed. The 50 Hz rate cap in `registry.PDF_RATE_HZ` was
investigated and found **correct** — see E3.
