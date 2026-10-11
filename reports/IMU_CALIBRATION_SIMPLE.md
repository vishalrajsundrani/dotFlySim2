# IMU calibration

This file explains, in plain words, how we made the simulated drone's IMU
behave like the IMU on the real aircraft. 

---

## What an IMU is, and what "calibrating" means here

An **IMU** (Inertial Measurement Unit) is the small sensor inside a drone that
measures two things:

- the **gyroscope** measures how fast the drone is turning
- the **accelerometer** measures how fast its speed is changing

No sensor is perfect. Even when a drone sits completely still on the ground,
its IMU does not report exactly zero. The numbers jump around a little. That
jumping is called **noise**.

Every IMU has its own amount of noise. A good simulation must copy it. If the
simulated IMU is too quiet, software tested in the simulation will look better
than it really is, and may fail on the real drone.

Our simulation had the wrong numbers. They had been copied from a different
aircraft (a small research drone called the x500), not from a Matrice 4E. Our
job was to replace them with real measurements.

---

## Step 1: Get data from the real aircraft

We started with a **rosbag**. A rosbag is a recording file. While the real
drone flies, it saves everything its sensors said, with the time of each
reading.

Our file was `bag_000026`. It was 15 gigabytes, held 184 seconds of flight,
and contained 58 different streams of data. Most of that size was camera
video. The IMU part was only a few megabytes hidden inside it.

We wrote a tool, `tools/imu_from_bag.py`, to pull out only the IMU readings
and ignore the video.

---

## Step 2: Find a moment when the drone was standing still

This is the most important step, and it is easy to get wrong.

To measure noise, the drone must not be moving. If it is flying, the sensor is
reporting real movement, and we would mistake that movement for noise.

So we looked through the whole flight for a quiet period. We used the
gyroscope to decide, not the accelerometer. The reason is simple: a drone
sitting on a slope still feels gravity pulling sideways, so the accelerometer
can look busy while nothing moves. A gyroscope cannot be fooled this way. If
the drone is not turning, it reads near zero.

We found two quiet periods:

- **6.3 seconds at the very start**, before the drone took off
- **1.9 seconds at the end**, after it landed

Everything in between was flight. We used the 6.3 second window at the start.

Our first attempt asked for a 10 second window. No 10 second window fits
inside 6.3 seconds, so the tool picked the "least bad" window instead, which
was half full of the drone climbing away. It then reported a noise figure
about 70 times too large and said nothing was wrong. We changed the tool so
that it **refuses and explains** instead of guessing. A tool that returns a
confident wrong answer is worse than one that stops.

---

## Step 3: Take out the raw numbers

The drone publishes several versions of the same measurement. Some are
"fused", meaning the flight computer has already cleaned them up and mixed
them with other sensors. Those are not useful to us, because we want the
sensor itself, not the flight computer's opinion of it.

We used the two **raw** streams:

- `angular_rate_body_raw` — the gyroscope
- `acceleration_body_raw` — the accelerometer

We also had to work out how fast the sensor actually reports. Three different
methods gave three different answers:

| method | answer |
| --- | --- |
| messages divided by recording time | 43.4 Hz |
| the middle value of the gaps between readings | 52.5 Hz |
| the average gap, ignoring dropped data | **49.75 Hz** |

The last one is correct. The first is too low because the recorder stalled
three times and lost 23 seconds of data. The second is too high because the
gaps are lopsided. The real sensor runs at 50 Hz, and 49.75 is that.

This matters for the next step.

---

## Step 4: Measure the noise with standard deviation

**Standard deviation** is a number that says how much a set of readings
spreads out around its average. A small standard deviation means the readings
are tightly grouped. A large one means they jump around a lot. It is the
normal way to put a number on noise.

We calculated the standard deviation of each axis (x, y and z) separately, for
both the gyroscope and the accelerometer, over the 6.3 quiet seconds.

But a standard deviation on its own is not enough, and this is the part people
usually miss. **The same sensor gives a different standard deviation depending
on how fast you sample it.** Sample faster and each reading is noisier.

So we converted it into a **noise density**, which does not depend on the
sample rate:

```
noise density = standard deviation / square root of the sample rate
```

This conversion, and the reason it is necessary, is explained in **Woodman
(2007), "An introduction to inertial navigation", section 4.2**, in
`papers/woodman2007_intro_inertial_navigation.pdf`. The same section explains
how noise density relates to the other names manufacturers use, such as
"angle random walk".

Our simulated IMU runs at 250 Hz, not 50 Hz. So to get the number the
simulation needs, we converted back the other way:

```
simulation value = noise density x square root of 250
```

---

## Step 5: A problem — not all the shaking was noise

When we looked closely, the accelerometer did not behave like pure noise.

Pure sensor noise is spread evenly across all frequencies. It is like radio
static. When we examined the accelerometer's x axis, **two thirds of its
movement was at very low frequencies**, below 5 Hz.

That is not sensor noise. That is the real aircraft gently rocking on its
landing legs in the wind. It is real movement of a real drone.

If we had counted it as noise, we would have made the simulated accelerometer
about 2.4 times noisier than the real part actually is.

So instead of using the standard deviation of everything, we looked at the
**spectrum** — a breakdown of the signal by frequency — and read the noise
level from the flat, high-frequency part, where the rocking has died away and
only sensor noise is left. Reading noise off the flat part of a spectrum is
standard practice and is described in **Woodman (2007) section 4.2** and in
**Hou (2004) chapter 4**.

The gyroscope did not have this problem. Its energy was spread evenly, which
is what real noise looks like, so its numbers were trustworthy either way.

---

## Step 6: The things we measured but did not use

An IMU has more than one kind of error. Besides the fast, random jumping,
there is a slow **drift**: the sensor's zero point wanders over minutes.

The standard way to measure drift is the **Allan variance**. You take longer
and longer stretches of data, average each one, and see how the averages
disagree. Plotted on a chart, different kinds of error show up as different
slopes, so one chart separates them.

The papers behind this:

- **Allan (1966), "Statistics of Atomic Frequency Standards"** — the original
  method. It was invented for atomic clocks, not drones, but every later use
  comes from it. In `papers/allan1966_statistics_atomic_frequency_standards.pdf`.
- **Hou (2004), "Modeling Inertial Sensors Errors Using Allan Variance",
  chapter 4** — applies it to IMUs, and shows how each slope on the chart maps
  to a specific kind of error. Section 4.3.3 gives the factor 0.664 used to
  read drift off the chart. In
  `papers/hou2004_modeling_inertial_sensor_errors_allan_variance.pdf`.
- **Riley (2008), NIST Special Publication 1065, section 5.2.4** — the exact
  calculation we used, called the *overlapping* Allan deviation. In
  `papers/riley2008_nist_sp1065_frequency_stability.pdf`.

We ran this calculation. **Then we threw the result away**, on purpose.

Drift takes minutes to show itself. We had 6.3 seconds. The chart never
curved the way it must for the answer to be readable. Fitting a number to it
anyway would have been inventing data.

**Lethander and Taylor (2023), "Conservative Estimation of Inertial Sensor
Errors Using Allan Variance Data"** is about exactly this situation: what to
do when the recording is too short to support the answer. Their advice is to
report a conservative bound rather than a confident number. We took the
simpler version of that advice and kept the old values, clearly labelled as
not measured. In `papers/navi2023_conservative_estimation_inertial_errors_allan.pdf`.

The tool now refuses to fit drift from anything shorter than 10 minutes.

---

## Step 7: Find out what settings the simulator actually has

Next we needed to know which numbers the simulator will even accept. The
simulated IMU is described in a file called `models/m4e/model.sdf`. Its noise
settings are:

| setting | what it means |
| --- | --- |
| `mean` | a fixed offset added to every reading |
| `stddev` | how much each reading jumps around |
| `dynamic_bias_stddev` | how much the slow drift wanders |
| `dynamic_bias_correlation_time` | how slowly that drift changes |

Rather than guess what these do, we read the simulator's own source code,
`gz-sensors`, and saved a copy in
`papers/gz-sensors_GaussianNoiseModel.cc`. Reading it told us three useful
things.

First, `stddev` is a per-reading value, not a noise density. That confirmed our
conversion in step 4 was the right one.

Second, the drift is modelled as something called a **Gauss-Markov process**: a
value that wanders randomly but is always pulled back toward zero.
**Hou (2004) section 4.3.6** describes this kind of error and how it appears on
an Allan chart.

Third, and most useful, `dynamic_bias_stddev` **is not** the size of the drift.
Working through the formula in the source code, the drift actually wanders by:

```
real drift size = dynamic_bias_stddev x square root of (correlation time / 2)
```

With the inherited correlation time of 300 seconds, that is about **12 times
larger** than the setting suggests. We checked this by simulation and it was
correct to within 5 percent.

This mattered. It meant the inherited drift setting was making the simulated
gyroscope's zero point wander by about as much as the real bias we had just
measured. The drift was swamping the measurement.

---

## Step 8: Record the simulation and compare

Measuring the real drone was only half the job. We then did exactly the same
thing to the simulation:

1. started the simulation with the drone sitting on the ground
2. recorded its IMU into a rosbag, the same way the real drone's was recorded
3. ran the **same tool** over it, with the same settings

Using the same tool on both sides matters. If we had measured the real drone
one way and the simulation another, any difference might have come from the
measuring rather than from the drones.

---

## Step 9: The surprise — the simulation multiplies the noise

The number you write in the model file is **not** the number that comes out
the other end.

In between sit three things: the Gazebo simulator generates the noise, the
PX4 flight software filters it, and our bridge publishes it at 50 Hz to match
the aircraft.

That last step is the problem. Gazebo creates noise across a wide frequency
range, and cutting the rate to 50 Hz folds all of it into a narrow range. The
noise does not disappear; it piles up.

We measured the effect: the published noise came out **1.85 to 1.94 times
larger** than the value we had written, consistently across all six axes.

So we divided our values by that measured factor and wrote them again. We did
not calculate this factor from theory, because it depends on filter settings
that could change. We measured it, and the tool can re-measure it at any time.

---

## Step 10: Check that it worked

We restarted the simulation with the corrected values and recorded it again.
Here is what the two drones published, side by side:

| axis | real aircraft | simulation | difference |
| --- | --- | --- | --- |
| gyroscope x | 7.92e-05 | 7.81e-05 | 1% |
| gyroscope y | 6.94e-05 | 7.00e-05 | 1% |
| gyroscope z | 7.27e-05 | 7.11e-05 | 2% |
| accelerometer x | 2.78e-04 | 2.80e-04 | 1% |
| accelerometer y | 2.97e-04 | 2.98e-04 | 0% |
| accelerometer z | 1.33e-03 | 1.37e-03 | 3% |

All six within 3 percent. The simulation now sounds like the real aircraft.

A side note worth knowing: the old gyroscope values were nearly right already,
within about 11 percent. The accelerometer values were wrong by up to 7 times.
The real Matrice 4E accelerometer is much quieter sideways than the research
drone whose numbers we had been using.

---

## What we still cannot do

Being honest about the limits is part of the result.

**We cannot measure drift.** That needs the drone switched on and completely
still for 30 minutes or more. We had 6.3 seconds. The drift settings in the
model are still the old inherited ones.

**We cannot measure accelerometer bias.** The flight computer subtracts
gravity before publishing, so what is left mixes the sensor's own offset with
the error in its gravity estimate, and nothing separates the two.
**Woodman (2007) sections 4.1 and 4.3** explains why telling a sensor offset
apart from a slight tilt needs a proper calibration rig, not a flight
recording. We set those values to zero and said why in the model file.

**We did not measure temperature effects.** We had one flight at one
temperature.

If you can get a recording of the aircraft switched on and sitting still for
half an hour, both of the first two gaps can be closed by running the same
tool again. No code needs changing.

---

