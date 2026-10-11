# References for the IMU calibration

Every method and every term used in `../IMU_CALIBRATION.md` is defined in one
of the works below. The table in section 1 is the useful part: it maps each
thing the tool does to the source that defines it, so a reviewer can check the
method rather than take it on trust.

PDFs in this directory were downloaded from the publishers' own open-access
copies. Four works could not be included because they are behind a paywall;
they are listed in section 3 with DOIs, and in every case an open substitute
that covers the same material is named.

---

## 1. Which reference defines what

| term / method used | defined in | where (verified) |
| --- | --- | --- |
| **Allan variance**, and the σ(τ) log–log plot | Allan 1966 | the original derivation |
| **Overlapping Allan deviation** (the estimator the tool computes) | Riley 2008 (NIST SP 1065) | §5.2.4 "Overlapping Allan Variance" |
| **Quantization noise** (Allan slope −1) | Hou 2004 | §4.3.1 |
| **Angle / velocity random walk** (`N`, slope −½, read at τ = 1 s) | Hou 2004; Woodman 2007 | Hou §4.3.2; Woodman §4.2 "Thermo-Mechanical White Noise / Angle Random Walk" |
| **Bias instability**, and the 0.664 factor relating the Allan floor to `B` | Hou 2004 | §4.3.3 |
| **Rate random walk** (slope +½) | Hou 2004 | §4.3.4 |
| **Exponentially correlated (Gauss-Markov) noise** — the process Gazebo's `dynamic_bias_correlation_time` selects | Hou 2004 | §4.3.6 |
| **Noise density**, and why a standard deviation is meaningless without its sample rate | Woodman 2007 | §4.2, incl. the conversion between PSD, FFT noise density and ARW |
| **Why bias cannot be separated from a tilt** without a fixture | Woodman 2007 | §4.1, §4.3 |
| **Random-walk bias model** and the IMU error state it sits in | Trawny & Roumeliotis 2005 | eqs. (111)–(116) |
| **Exactly what Gazebo does** with `stddev`, `dynamic_bias_stddev`, `dynamic_bias_correlation_time` | `gz-sensors_GaussianNoiseModel.cc` | `ApplyImpl()` |
| **Steady-state variance `σ_b²·τ/2`** of that process | derived in §5 below from the `gz-sensors` update; verified numerically in the workbook's *Bias model* sheet |  |
| **Conservative estimation from short Allan data** — the formal version of this project's "refuse rather than fit" rule | Lethander & Taylor 2023 | throughout; cites IEEE 647-2006 and 952-2020 |
| **Fitting a stochastic model statistically** (GMWM) rather than reading slopes off a plot | Balamuta et al. 2016 | §II–III |

### One distinction the sources disagree on, which matters here

Trawny & Roumeliotis (eq. 114) and the Kalibr noise model treat the bias as a
**pure random walk**, `ḃ = n_w`, which is unbounded — it wanders further the
longer you run. Gazebo does not: `gz-sensors` implements an **exponentially
correlated (first-order Gauss-Markov)** bias with a finite correlation time,
which is bounded and reverts toward zero. The random walk is the `τ → ∞` limit
of the Gauss-Markov process. Hou §4.3.6 is the reference that covers the form
Gazebo actually uses, which is why it, and not Trawny, is cited for it above.
This distinction is the reason `dynamic_bias_stddev` cannot be read as "the
bias standard deviation" — see §5.

---

## 2. Included PDFs

**Allan, D. W. (1966).** *Statistics of Atomic Frequency Standards.*
Proceedings of the IEEE 54(2), 221–230.
`allan1966_statistics_atomic_frequency_standards.pdf`
The paper the Allan variance is named after. Written about caesium clocks, not
gyroscopes, but every inertial use descends from it.

**Hou, H. (2004).** *Modeling Inertial Sensors Errors Using Allan Variance.*
MSc thesis, University of Calgary (UCGE Report 20201). DOI 10.11575/prism/21729.
`hou2004_modeling_inertial_sensor_errors_allan_variance.pdf`
The most directly useful document here. It is the long-form version of the
widely cited El-Sheimy, Hou & Niu (2008) paper, with the derivations that the
journal version compresses: how each noise process maps to a slope on the Allan
plot, and how to read the coefficients off. 35 MB, ~140 pages.

**Woodman, O. J. (2007).** *An introduction to inertial navigation.*
University of Cambridge Computer Laboratory, UCAM-CL-TR-696.
`woodman2007_intro_inertial_navigation.pdf`
The clearest explanation of noise density and of why bias and tilt are
confounded. §4 is the relevant part and is about fifteen pages.

**Riley, W. J. (2008).** *Handbook of Frequency Stability Analysis.*
NIST Special Publication 1065.
`riley2008_nist_sp1065_frequency_stability.pdf`
The reference implementation of every Allan-family estimator, including the
overlapping form used here, and the standard treatment of how much confidence
an Allan point deserves given how few clusters support it.

**Trawny, N. & Roumeliotis, S. I. (2005).** *Indirect Kalman Filter for 3D
Attitude Estimation.* University of Minnesota, MARS Lab TR 2005-002.
`trawny2005_indirect_kalman_filter_3d_attitude.pdf`
Defines the IMU error state and the random-walk bias model, eqs. (111)-(116).
Note that it uses a pure random walk, not the finite-correlation-time
Gauss-Markov process Gazebo implements; see the note at the end of section 1.
It is the document the Kalibr IMU noise model page follows.

**Lethander, K. A. & Taylor, C. N. (2023).** *Conservative Estimation of
Inertial Sensor Errors Using Allan Variance Data.* NAVIGATION 70(3).
DOI 10.33012/navi.563.
`navi2023_conservative_estimation_inertial_errors_allan.pdf`
Directly relevant to this project's central difficulty: what to do when the
Allan data is too short to determine the parameters. Argues for conservative
bounds rather than point estimates, and cites IEEE 647-2006 and 952-2020 as
the standard procedure.

**Balamuta, J., Guerrier, S., Molinari, R. & Yang, W. (2016).** *A
Computationally Efficient Framework for Automatic Inertial Sensor
Calibration.* arXiv:1603.05297.
`balamuta2016_automatic_inertial_sensor_calibration_gmwm.pdf`
The Generalised Method of Wavelet Moments: fits a composite stochastic model
to the data statistically instead of reading slopes off a plot by eye. The
principled upgrade path if this calibration is ever redone properly.

**`gz-sensors_GaussianNoiseModel.cc`** — not a paper, but the authoritative
definition of what the numbers in `model.sdf` mean. Fetched from
`gazebosim/gz-sensors`, branch `gz-sensors8`. Its own comments cite RotorS and
the Kalibr IMU noise model as the derivation.

---

## 3. Paywalled, with open substitutes

**El-Sheimy, N., Hou, H. & Niu, X. (2008).** *Analysis and Modeling of
Inertial Sensors Using Allan Variance.* IEEE Trans. Instrumentation and
Measurement 57(1), 140–149. DOI 10.1109/TIM.2007.908635.
The standard citation for Allan-variance IMU characterisation. **Substitute:
Hou 2004**, included above — same author, same work, more detail.

**IEEE Std 952-2020** — *Guide and Test Procedure for Single-Axis
Interferometric Fiber Optic Gyros*, and **IEEE Std 647-2006** for ring laser
gyros. These are where "angle random walk", "bias instability" and "rate
random walk" are formally defined for instrument datasheets.
**Substitutes: Hou 2004 §3 and Lethander & Taylor 2023**, which restate the
definitions and cite the standards directly.

**Farrell, J. A., Silva, F. O., Rahman, F. & Wendel, J. (2022).** *IMU Error
Modeling Tutorial.* IEEE Control Systems Magazine 42(6), 40–66.
DOI 10.1109/MCS.2022.3209059. An author preprint exists at
<https://escholarship.org/uc/item/1vf7j52p> but eScholarship returned HTTP 403
to this machine; it may be reachable from another network.
**Substitute: Trawny & Roumeliotis 2005** for the state-space bias model, and
Hou 2004 §4.3.6 for the exponentially correlated form.
Companion code: <https://github.com/jaffarrell/AV-Matlab-SW>.

**Furrer, F., Burri, M., Achtelik, M. & Siegwart, R. (2016).** *RotorS — A
Modular Gazebo MAV Simulator Framework.* In *Robot Operating System (ROS):
The Complete Reference, Vol. 1*, 595–625, Springer. DOI 10.1007/978-3-319-26054-9_23.
The origin of the dynamic-bias model Gazebo inherited. Springer-locked.
**Substitute: the source file included here**, whose comment names RotorS as
its source, plus the Kalibr wiki below.

**Rehder, J., Nikolic, J., Schneider, T., Hinzmann, T. & Siegwart, R. (2016).**
*Extending kalibr: Calibrating the extrinsics of multiple IMUs and of
individual axes.* ICRA 2016. DOI 10.1109/ICRA.2016.7487628.

---

## 4. Non-paper references

- **Kalibr IMU Noise Model** —
  <https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model>
  The engineering reference most widely used in robotics for the relationship
  between a datasheet's noise density and a discrete-time simulation
  parameter. Cited by name in the Gazebo source.
- **SDFormat `<noise>` specification** — <http://sdformat.org/spec?elem=sensor>
- **`gazebosim/gz-sensors`** — <https://github.com/gazebosim/gz-sensors>
- **RotorS IMU plugin**, the ancestor of the Gazebo implementation —
  <https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/src/gazebo_imu_plugin.cpp>

---

## 5. One thing the literature settles that mattered here

`dynamic_bias_stddev` is **not** the standard deviation of the simulated
wandering bias. Substituting the discrete Gauss-Markov update that
`gz-sensors` implements,

```
phi     = exp(-dt/tau)
sigma_d = sqrt(-sigma_b^2 * tau/2 * expm1(-2*dt/tau))
bias   <- phi*bias + N(0, sigma_d)
```

into the steady-state relation `var = sigma_d^2 / (1 - phi^2)` gives

```
var_steady_state = sigma_b^2 * tau / 2
```

exactly. So the bias actually wanders by `sigma_b * sqrt(tau/2)`, which for the
inherited `tau = 300 s` is **12x larger** than the parameter reads. The
consequence for this calibration is in the workbook's *Bias model* sheet: the
inherited dynamic bias wanders by about as much as the static bias measured
from the aircraft, and therefore swamps it.
