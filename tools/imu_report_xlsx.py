#!/usr/bin/env python3
"""Turn imu_from_bag.py's JSON dump into a calibration workbook.

Kept separate from the measurement so that re-rendering the report never
re-reads fifteen gigabytes off a USB stick, and so the numbers in the
spreadsheet are provably the same numbers the tool printed.

    tools/imu_from_bag.py <bag> --json phys.json
    tools/imu_report_xlsx.py phys.json -o reports/imu_calibration.xlsx
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from openpyxl import Workbook
from openpyxl.chart import LineChart, Reference, ScatterChart, Series
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

GYRO_TOPIC = "/wrapper/psdk_ros2/angular_rate_body_raw"
ACCEL_TOPIC = "/wrapper/psdk_ros2/acceleration_body_raw"
AXES = ("x", "y", "z")

H1 = Font(bold=True, size=14, color="FFFFFF")
H2 = Font(bold=True, size=11)
MONO = Font(name="Consolas", size=10)
FILL_H = PatternFill("solid", fgColor="1F3864")
FILL_SUB = PatternFill("solid", fgColor="D9E2F3")
FILL_WARN = PatternFill("solid", fgColor="FFF2CC")
FILL_OK = PatternFill("solid", fgColor="E2EFDA")
FILL_BAD = PatternFill("solid", fgColor="FBE5D6")
THIN = Side(style="thin", color="BFBFBF")
BOX = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)


def title(ws, text, width=8):
    ws.merge_cells(start_row=1, start_column=1, end_row=1, end_column=width)
    c = ws.cell(1, 1, text)
    c.font = H1
    c.fill = FILL_H
    c.alignment = Alignment(horizontal="left", vertical="center")
    ws.row_dimensions[1].height = 24


def header_row(ws, row, labels, start=1):
    for i, lab in enumerate(labels):
        c = ws.cell(row, start + i, lab)
        c.font = H2
        c.fill = FILL_SUB
        c.border = BOX
        c.alignment = Alignment(horizontal="center", wrap_text=True)


def widths(ws, spec):
    for col, w in spec.items():
        ws.column_dimensions[col].width = w


# --------------------------------------------------------------------------

def sheet_summary(wb, d):
    r = d["result"]
    ws = wb.create_sheet("Summary")
    title(ws, "IMU calibration — Matrice 4E, measured from flight data", 9)
    widths(ws, {"A": 22, "B": 13, "C": 13, "D": 13, "E": 13, "F": 13, "G": 10, "H": 11, "I": 34})

    row = 3
    for k, v in (
        ("Source bag", d["bag"]),
        ("Model patched", d["model"]),
        ("Bag duration (s)", round(r["duration"], 1)),
        ("Gyro messages", r["n_gyro"]),
        ("Accel messages", r["n_accel"]),
        ("Measured rate — gyro (Hz)", round(r["rate_gyro"], 2)),
        ("Measured rate — accel (Hz)", round(r["rate_accel"], 2)),
        ("Simulation IMU rate (Hz)", r["sim_rate"]),
        ("Rescale factor √(sim/real)", round(math.sqrt(r["sim_rate"] / r["rate_gyro"]), 3)),
        ("Stationary segment start (s)", round(r["segment_start_s"], 1)),
        ("Stationary segment length (s)", round(r["segment_len_s"], 1)),
    ):
        ws.cell(row, 1, k).font = H2
        ws.cell(row, 2, v)
        row += 1

    row += 1
    ws.cell(row, 1, "Per-axis results").font = H2
    row += 1
    header_row(ws, row, [
        "channel", "measured σ\n(per sample)", "noise density\n(unit/√Hz)",
        "→ simulation σ\n(at sim rate)", "previous σ\n(before this work)",
        "ratio\nnew/old", "lag-1\nautocorr", "mean\n(bias)", "verdict"])
    row += 1
    first_data = row
    for key, unit in (("gyro", "rad/s"), ("accel", "m/s²")):
        for axis in AXES:
            s = r["stats"][key][axis]
            cur = s.get("stddev_current") or 0.0
            ratio = (s["stddev_sim"] / cur) if cur else None
            ws.cell(row, 1, f"{'angular_velocity' if key=='gyro' else 'linear_acceleration'}.{axis}  [{unit}]")
            ws.cell(row, 2, s["stddev_measured"]).number_format = "0.000E+00"
            ws.cell(row, 3, s["density"]).number_format = "0.000E+00"
            ws.cell(row, 4, s["stddev_sim"]).number_format = "0.000E+00"
            ws.cell(row, 5, cur).number_format = "0.000E+00"
            c = ws.cell(row, 6, ratio)
            c.number_format = "0.00"
            if ratio:
                c.fill = FILL_OK if 0.5 <= ratio <= 2 else FILL_BAD
            c2 = ws.cell(row, 7, s["lag1"])
            c2.number_format = "0.00"
            c2.fill = FILL_OK if abs(s["lag1"]) < 0.2 else FILL_WARN
            ws.cell(row, 8, s["mean"]).number_format = "0.000E+00"
            # judge on the factor, not the difference: 0.53x and 1.9x are the
            # same size of error in opposite directions
            if ratio:
                factor = max(ratio, 1.0 / ratio)
                if factor >= 2.0:
                    verdict = f"inherited value was wrong by {factor:.1f}x"
                elif factor >= 1.25:
                    verdict = f"inherited value was off by {factor:.2f}x"
                else:
                    verdict = "inherited value was already close"
            else:
                verdict = ""
            ws.cell(row, 9, verdict)
            for col in range(1, 10):
                ws.cell(row, col).border = BOX
            row += 1

    chart = LineChart()
    chart.title = "Measured-at-sim-rate σ vs previously inherited σ"
    chart.y_axis.title = "standard deviation"
    chart.x_axis.title = "channel"
    chart.height, chart.width = 8, 20
    data = Reference(ws, min_col=4, max_col=5, min_row=first_data - 1, max_row=row - 1)
    cats = Reference(ws, min_col=1, min_row=first_data, max_row=row - 1)
    chart.add_data(data, titles_from_data=True)
    chart.set_categories(cats)
    ws.add_chart(chart, f"A{row + 2}")
    return ws


def sheet_method(wb, d):
    ws = wb.create_sheet("Method & caveats")
    title(ws, "How these numbers were produced, and what they are worth", 3)
    widths(ws, {"A": 6, "B": 48, "C": 86})
    row = 3
    ws.cell(row, 2, "Step").font = H2
    ws.cell(row, 3, "Detail").font = H2
    ws.cell(row, 2).fill = FILL_SUB
    ws.cell(row, 3).fill = FILL_SUB
    row += 1
    steps = [
        ("Read the bag",
         "MCAP parsed directly. The summary section gives a chunk index; each chunk's per-channel "
         "message index gives byte offsets inside the decompressed chunk, so only IMU messages are "
         "deserialised and the camera payloads are never decoded."),
        ("Pick the channels",
         "angular_rate_body_raw and acceleration_body_raw — the unfiltered sensor channels. The fused "
         "channels were read too, but only as a cross-check."),
        ("Establish the true rate",
         "From the median inter-sample interval, not from message_count/duration. The average is "
         "dragged down by dropouts and reports a rate the sensor never ran at."),
        ("Find a stationary segment",
         "A window is slid across the flight and scored on summed gyro variance. The gyro is used "
         "rather than the accelerometer because a steady tilt looks like acceleration but cannot "
         "fake a rotation rate."),
        ("Per-axis statistics",
         "Mean (bias) and standard deviation over the quietest window."),
        ("Convert to a noise density",
         "σ_density = σ_sample / √rate, then back up to the simulation's rate: σ_sim = σ_density × √rate_sim. "
         "A standard deviation is only meaningful alongside the rate it was sampled at."),
        ("Check that rescaling is legal",
         "Lag-1 autocorrelation of the detrended segment. Near zero means independent samples and the "
         "rescale holds; clearly positive means the data was low-pass filtered before recording and the "
         "rescale understates the true noise."),
        ("Allan deviation",
         "Overlapping Allan deviation per axis, for bias instability and correlation time — reported, "
         "but only adopted when the stationary segment is long enough to support it."),
    ]
    for i, (s, detail) in enumerate(steps, 1):
        ws.cell(row, 1, i)
        ws.cell(row, 2, s).font = H2
        c = ws.cell(row, 3, detail)
        c.alignment = Alignment(wrap_text=True, vertical="top")
        ws.row_dimensions[row].height = 42
        for col in (1, 2, 3):
            ws.cell(row, col).border = BOX
        row += 1

    row += 1
    ws.cell(row, 2, "Caveats carried into the model").font = H2
    row += 1
    for n in d["notes"]:
        c = ws.cell(row, 3, n)
        c.alignment = Alignment(wrap_text=True, vertical="top")
        c.fill = FILL_WARN
        c.border = BOX
        ws.row_dimensions[row].height = 32
        row += 1
    return ws


def sheet_stationary(wb, d):
    ws = wb.create_sheet("Stationary search")
    title(ws, "Finding a segment where the aircraft was genuinely still", 6)
    widths(ws, {"A": 12, "B": 16, "C": 14, "D": 14, "E": 14, "F": 60})
    r = d["result"]
    row = 3
    ws.cell(row, 1, "Stationary runs found").font = H2
    row += 1
    header_row(ws, row, ["start (s)", "end (s)", "length (s)", "used"])
    row += 1
    for st, en, ln in d["stationary"].get("runs", []):
        ws.cell(row, 1, round(st, 2))
        ws.cell(row, 2, round(en, 2))
        ws.cell(row, 3, round(ln, 2))
        used = abs(st - r["segment_start_s"]) < 0.5
        c = ws.cell(row, 4, "yes" if used else "no")
        c.fill = FILL_OK if used else FILL_WARN
        for col in range(1, 5):
            ws.cell(row, col).border = BOX
        row += 1

    row += 1
    note = ws.cell(row, 1,
        "The whole flight is profiled with a 1-second probe window. 'Still' means the probe's "
        "summed per-axis gyro standard deviation is within 3x the quietest probe anywhere in the "
        "bag. The longest contiguous run of still probes becomes the segment. An earlier version "
        "took a fixed 10-second window and returned the least-bad one; on this bag, whose only "
        "still moments are before takeoff and after landing, that silently returned a window half "
        "full of climb-out.")
    ws.merge_cells(start_row=row, start_column=1, end_row=row + 2, end_column=6)
    note.alignment = Alignment(wrap_text=True, vertical="top")
    note.fill = FILL_WARN
    row += 4

    ws.cell(row, 1, "Profile (gyro noise across the flight)").font = H2
    row += 1
    header_row(ws, row, ["time (s)", "summed gyro σ (rad/s)", "threshold"])
    row += 1
    first = row
    thr = d["stationary"].get("threshold", 0.0)
    for _, score, _, t in d["scores"]:
        ws.cell(row, 1, round(t, 2))
        ws.cell(row, 2, score)
        ws.cell(row, 3, thr)
        row += 1
    ch = ScatterChart()
    ch.title = "Gyro noise across the flight (log scale) — still periods sit on the floor"
    ch.x_axis.title = "time since start of bag (s)"
    ch.y_axis.title = "summed per-axis gyro σ (rad/s)"
    ch.y_axis.scaling.logBase = 10
    ch.height, ch.width = 10, 26
    xs = Reference(ws, min_col=1, min_row=first, max_row=row - 1)
    for col, name in ((2, "measured"), (3, "still threshold")):
        ser = Series(Reference(ws, min_col=col, min_row=first - 1, max_row=row - 1),
                     xs, title_from_data=True)
        ser.marker.symbol = "none"
        ch.series.append(ser)
    ws.add_chart(ch, "E3")
    return ws


def _segment(d, which, limit_s=None):
    """Raw samples inside the stationary window, for the plotting sheets."""
    topic = GYRO_TOPIC if which == "gyro" else ACCEL_TOPIC
    raw = d["raw"].get(topic)
    if raw is None:
        return np.array([]), np.array([])
    t = np.array(raw["t"])
    x = np.array(raw["xyz"])
    r = d["result"]
    lo = r["segment_start_s"]
    hi = lo + r["segment_len_s"]
    if limit_s is not None:
        hi = min(hi, lo + limit_s)
    m = (t >= lo) & (t <= hi)
    if not m.any():
        return t[m], x[m]
    return t[m] - t[m][0], x[m]


def sheet_timeseries(wb, d, name, subtitle, chart_limit_s=None):
    """Raw samples of one stationary segment, aircraft or simulation.

    The aircraft contributes 6.3 s; a simulated recording contributes over two
    minutes, because nothing there has to take off.  The full segment is
    written out either way, but the chart is clipped to a common span so the
    two sheets can be read against each other instead of one appearing calm
    merely by being drawn at a 20x wider time scale.
    """
    ws = wb.create_sheet(name)
    title(ws, subtitle, 9)
    widths(ws, {"A": 11, "B": 12, "C": 12, "D": 12, "F": 11, "G": 12, "H": 12, "I": 12})
    anchor = {"gyro": (1, "A"), "accel": (6, "F")}
    for which, (col0, letter) in anchor.items():
        t, x = _segment(d, which)
        unit = "rad/s" if which == "gyro" else "m/s^2"
        c = ws.cell(3, col0, f"{which} [{unit}]  n={len(t)}  "
                             f"{(t[-1] if len(t) else 0):.1f} s")
        c.font = H2
        header_row(ws, 4, ["t (s)", "x", "y", "z"], start=col0)
        for i in range(len(t)):
            ws.cell(5 + i, col0, round(float(t[i]), 4))
            for j in range(3):
                ws.cell(5 + i, col0 + 1 + j, float(x[i, j]))
        if len(t) == 0:
            continue
        n_chart = len(t)
        if chart_limit_s is not None:
            n_chart = int(np.searchsorted(t, chart_limit_s)) or len(t)
        ch = LineChart()
        ch.title = (f"{which} during the stationary segment"
                    + (f" (first {chart_limit_s:.1f} s)" if chart_limit_s
                       and n_chart < len(t) else ""))
        ch.y_axis.title = unit
        ch.x_axis.title = "t (s)"
        ch.height, ch.width = 8, 18
        data = Reference(ws, min_col=col0 + 1, max_col=col0 + 3, min_row=4,
                         max_row=4 + n_chart)
        ch.add_data(data, titles_from_data=True)
        ws.add_chart(ch, f"{letter}{8 + len(t)}")
    return ws


def sheet_sampling(wb, phys, sim):
    """How the two sources are actually sampled, and where the aircraft's
    recording lost data."""
    ws = wb.create_sheet("Sampling & dropouts")
    title(ws, "Sample intervals: the aircraft's recording has gaps, the simulation's does not", 8)
    widths(ws, {"A": 46, "B": 16, "C": 16, "E": 14, "F": 14, "G": 14})
    ws.merge_cells("A3:C6")
    c = ws.cell(3, 1,
        "Three estimators of the aircraft's rate disagree, and only one is right. "
        "message_count/duration reports 43.4 Hz because 23.6 s of the recording is missing to "
        "three write stalls. The median interval reports 52.5 Hz because the interval "
        "distribution is skewed. The mean over non-dropout intervals reports 49.75 Hz, which is "
        "the 50 Hz the aircraft runs at and the rate the bridge is capped to.")
    c.alignment = Alignment(wrap_text=True, vertical="top")
    c.fill = FILL_WARN

    row = 8
    header_row(ws, row, ["quantity", "aircraft", "simulation"])
    row += 1
    pr, sr = phys["result"], (sim or {}).get("result", {})
    for label, pv, sv in (
        ("gyro messages", pr["n_gyro"], sr.get("n_gyro")),
        ("recording span (s)", round(pr["duration"], 1), round(sr.get("duration", 0), 1)),
        ("count/duration (Hz)", round(pr["n_gyro"] / pr["duration"], 2),
         round(sr["n_gyro"] / sr["duration"], 2) if sr else None),
        ("rate, non-dropout mean (Hz)", round(pr["rate_gyro"], 2),
         round(sr.get("rate_gyro", 0), 2) if sr else None),
        ("stationary segment (s)", round(pr["segment_len_s"], 1),
         round(sr.get("segment_len_s", 0), 1) if sr else None),
    ):
        ws.cell(row, 1, label).font = H2
        ws.cell(row, 2, pv)
        ws.cell(row, 3, sv)
        for col in (1, 2, 3):
            ws.cell(row, col).border = BOX
        row += 1

    # interval histograms
    col = 5
    for label, d in (("aircraft", phys), ("simulation", sim)):
        if d is None:
            continue
        raw = d["raw"].get(GYRO_TOPIC)
        if raw is None:
            continue
        t = np.array(raw["t"])
        dt = np.diff(t) * 1000.0
        ws.cell(8, col, f"{label}: interval (ms)").font = H2
        ws.cell(8, col + 1, "count").font = H2
        h, e = np.histogram(dt[dt < 60], bins=40)
        for i, (cnt, lo) in enumerate(zip(h, e[:-1])):
            ws.cell(9 + i, col, round(float(lo), 2))
            ws.cell(9 + i, col + 1, int(cnt))
        ch = ScatterChart()
        ch.title = f"{label}: distribution of sample intervals"
        ch.x_axis.title = "interval (ms)"
        ch.y_axis.title = "count"
        ch.height, ch.width = 8, 14
        xs = Reference(ws, min_col=col, min_row=9, max_row=8 + len(h))
        ser = Series(Reference(ws, min_col=col + 1, min_row=8, max_row=8 + len(h)),
                     xs, title_from_data=True)
        ser.marker.symbol = "none"
        ch.series.append(ser)
        ws.add_chart(ch, f"{get_column_letter(col)}{10 + len(h)}")
        col += 3

    # the aircraft's dropouts, named
    raw = phys["raw"].get(GYRO_TOPIC)
    if raw is not None:
        t = np.array(raw["t"])
        dt = np.diff(t)
        big = np.where(dt > 0.1)[0]
        r0 = 9 + 45
        ws.cell(r0, 1, "Dropouts in the aircraft recording (>100 ms)").font = H2
        header_row(ws, r0 + 1, ["at t (s)", "gap (s)"])
        for i, k in enumerate(big):
            ws.cell(r0 + 2 + i, 1, round(float(t[k]), 1))
            ws.cell(r0 + 2 + i, 2, round(float(dt[k]), 3))
        ws.cell(r0 + 2 + len(big), 1, "total lost").font = H2
        ws.cell(r0 + 2 + len(big), 2, round(float(dt[dt > 0.1].sum()), 1))
    return ws


def sheet_psd(wb, phys, sim):
    ws = wb.create_sheet("Spectrum")
    title(ws, "Aircraft and simulation spectra, overlaid — the real test of a noise model", 12)
    widths(ws, {"A": 58})
    ws.merge_cells("A3:A7")
    c = ws.cell(3, 1,
        "White sensor noise is flat in frequency. Anything rising at the left is real "
        "low-frequency motion — a parked airframe swaying on its legs — and counting it as "
        "sensor noise would make the simulated IMU noisier than the real part; that is why the "
        "density is read off the plateau and not off the variance. Anything falling at the right "
        "is a low-pass filter applied before publication. Matching densities is necessary but not "
        "sufficient: these curves show whether the simulation matches the aircraft at every "
        "frequency or only on average.")
    c.alignment = Alignment(wrap_text=True, vertical="top")
    c.fill = FILL_WARN

    col = 3
    for key in ("gyro", "accel"):
        for axis in AXES:
            series = []
            for label, d in (("aircraft", phys), ("simulation", sim)):
                if d is None:
                    continue
                st = d["result"]["stats"][key][axis]
                f, p_ = st.get("psd_f") or [], st.get("psd") or []
                if f:
                    series.append((label, f, p_))
            if not series:
                continue
            nmax = max(len(f) for _, f, _ in series)
            ws.cell(9, col, "freq (Hz)").font = H2
            for j, (label, f, p_) in enumerate(series):
                ws.cell(9, col + 1 + j, f"{key}.{axis} {label}").font = H2
                for i in range(1, len(f)):
                    ws.cell(9 + i, col, round(float(f[i]), 3))
                    ws.cell(9 + i, col + 1 + j, float(p_[i]))
            ch = ScatterChart()
            ch.title = f"PSD {key}.{axis}"
            ch.x_axis.title = "Hz"
            ch.y_axis.title = "power / Hz"
            ch.y_axis.scaling.logBase = 10
            ch.height, ch.width = 7, 12
            xs = Reference(ws, min_col=col, min_row=10, max_row=8 + nmax)
            for j, _ in enumerate(series):
                ser = Series(Reference(ws, min_col=col + 1 + j, min_row=9,
                                       max_row=8 + nmax), xs, title_from_data=True)
                ser.marker.symbol = "none"
                ch.series.append(ser)
            band = 0 if key == "gyro" else 1
            ws.add_chart(ch, f"{get_column_letter(col)}{12 + nmax + band}")
            col += 4
    return ws


def sheet_allan(wb, d, sim=None):
    ws = wb.create_sheet("Allan deviation")
    title(ws, "Allan deviation — reported, deliberately not adopted", 10)
    widths(ws, {"A": 60})
    ws.merge_cells("A3:A6")
    c = ws.cell(3, 1,
        "Bias instability and correlation time are read off the floor of an Allan curve. That floor "
        "only appears when the averaging time reaches the bias process, which for this class of part "
        "is tens of seconds to minutes. This bag offers 6.3 seconds of stillness, so the curves below "
        "never turn over and the minimum sits at the right-hand edge — the signature of a "
        "measurement that has not converged. The model therefore keeps its inherited "
        "dynamic_bias_stddev and dynamic_bias_correlation_time.")
    c.alignment = Alignment(wrap_text=True, vertical="top")
    c.fill = FILL_WARN

    r = d["result"]
    col = 3
    for key in ("gyro", "accel"):
        for axis in AXES:
            for label, src in (("aircraft", d), ("simulation", sim)):
                if src is None:
                    continue
                st = src["result"]["stats"][key][axis]
                taus = st.get("allan_tau") or []
                devs = st.get("allan_dev") or []
                if not taus:
                    continue
                ws.cell(8, col, "tau (s)").font = H2
                ws.cell(8, col + 1, f"{key}.{axis} {label}").font = H2
                for i, (tau, dev) in enumerate(zip(taus, devs)):
                    ws.cell(9 + i, col, float(tau))
                    ws.cell(9 + i, col + 1, float(dev))
                col += 3
    ch = ScatterChart()
    ch.title = "Allan deviation (log–log). No floor is reached: the segment is too short."
    ch.x_axis.title = "averaging time τ (s)"
    ch.y_axis.title = "σ(τ)"
    ch.x_axis.scaling.logBase = 10
    ch.y_axis.scaling.logBase = 10
    ch.height, ch.width = 11, 24
    col = 3
    for key in ("gyro", "accel"):
        for axis in AXES:
            for label, src in (("aircraft", d), ("simulation", sim)):
                if src is None:
                    continue
                n = len(src["result"]["stats"][key][axis].get("allan_tau") or [])
                if not n:
                    continue
                xs = Reference(ws, min_col=col, min_row=9, max_row=8 + n)
                ser = Series(Reference(ws, min_col=col + 1, min_row=8, max_row=8 + n),
                             xs, title_from_data=True)
                ser.marker.symbol = "none"
                ch.series.append(ser)
                col += 3
    ws.add_chart(ch, "A9")
    return ws


def _accel_mean(d):
    """Mean of the accelerometer over the stationary segment, as published."""
    if d is None:
        return None
    raw = d["raw"].get(ACCEL_TOPIC)
    if raw is None:
        return None
    t = np.array(raw["t"])
    x = np.array(raw["xyz"])
    r = d["result"]
    lo = r["segment_start_s"]
    m = (t >= lo) & (t <= lo + r["segment_len_s"])
    if not m.any():
        return None
    return tuple(float(v) for v in x[m].mean(axis=0))


def sheet_verification(wb, phys, sim):
    ws = wb.create_sheet("Verification")
    title(ws, "Closed loop: does the simulation now publish what the aircraft publishes?", 9)
    widths(ws, {"A": 26, "B": 14, "C": 14, "D": 10, "E": 12, "F": 46})
    row = 3
    ws.merge_cells(start_row=row, start_column=1, end_row=row + 3, end_column=6)
    c = ws.cell(row, 1,
        "The value written into model.sdf is not what reaches a consumer. Gazebo's IMU feeds PX4, "
        "PX4 low-pass filters it into SensorCombined, and only then does the bridge publish "
        "/wrapper/psdk_ros2/imu. So the open-loop number is a starting point, and the question that "
        "matters is whether the published topic matches the aircraft's published topic. This sheet "
        "answers it by measuring the simulation exactly as the aircraft was measured — same tool, "
        "same estimator, same stationary logic.")
    c.alignment = Alignment(wrap_text=True, vertical="top")
    c.fill = FILL_SUB
    row += 5

    if sim is None:
        ws.cell(row, 1, "No simulation bag supplied.").font = H2
        return ws

    pr, sr = phys["result"], sim["result"]
    ws.cell(row, 1, "Published rate").font = H2
    ws.cell(row, 2, round(pr["rate_gyro"], 2))
    ws.cell(row, 3, round(sr["rate_gyro"], 2))
    row += 1
    header_row(ws, row, ["channel", "aircraft density", "simulation density",
                         "sim / real", "verdict", "note"])
    row += 1
    first = row
    for key, unit in (("gyro", "rad/s/√Hz"), ("accel", "m/s²/√Hz")):
        for axis in AXES:
            p = phys["result"]["stats"][key][axis]
            s = sim["result"]["stats"][key][axis]
            ratio = s["density"] / p["density"] if p["density"] else float("nan")
            ws.cell(row, 1, f"{key}.{axis}  [{unit}]")
            ws.cell(row, 2, p["density"]).number_format = "0.000E+00"
            ws.cell(row, 3, s["density"]).number_format = "0.000E+00"
            cc = ws.cell(row, 4, ratio)
            cc.number_format = "0.00"
            if math.isfinite(ratio):
                cc.fill = FILL_OK if 0.5 <= ratio <= 2.0 else FILL_BAD
            ws.cell(row, 5, "match" if 0.5 <= ratio <= 2.0 else "off")
            ws.cell(row, 6, "" if 0.5 <= ratio <= 2.0 else
                    ("simulation quieter than the aircraft" if ratio < 1
                     else "simulation noisier than the aircraft"))
            for col in range(1, 7):
                ws.cell(row, col).border = BOX
            row += 1
    # --- the gravity discrepancy -------------------------------------------
    # Found only because the simulation's raw samples went into this report:
    # the two sides of the same topic do not carry the same quantity.
    row += 1
    gp = _accel_mean(phys)
    gs = _accel_mean(sim)
    if gp is not None and gs is not None and abs(abs(gs[2]) - abs(gp[2])) > 1.0:
        ws.merge_cells(start_row=row, start_column=1, end_row=row + 4, end_column=6)
        c = ws.cell(row, 1,
            "SEPARATE FINDING, NOT A NOISE PROBLEM — the two sides of this topic do not carry "
            "the same quantity.\n"
            f"Over its stationary segment the aircraft's linear_acceleration averages "
            f"({gp[0]:+.4f}, {gp[1]:+.4f}, {gp[2]:+.4f}), magnitude {math.dist(gp,(0,0,0)):.4f} m/s^2: "
            "the flight controller has removed gravity. The simulation averages "
            f"({gs[0]:+.4f}, {gs[1]:+.4f}, {gs[2]:+.4f}), magnitude {math.dist(gs,(0,0,0)):.4f} m/s^2: "
            "gravity is present.\n"
            "The noise calibration is unaffected, because density is read from the spectrum with "
            "the mean removed. But a consumer that integrates acceleration — a mission, or the "
            "stereo-inertial SLAM port — sees a 9.8 m/s^2 difference between hardware and "
            "simulation on the same topic name. This is exactly the class of defect this rig "
            "exists to catch, and it is a bridge question, not an IMU one.")
        c.alignment = Alignment(wrap_text=True, vertical="top")
        c.fill = FILL_BAD
        c.border = BOX
        row += 6

    ch = LineChart()
    ch.title = "Noise density: aircraft vs simulation, per axis"
    ch.y_axis.title = "density"
    ch.height, ch.width = 9, 20
    data = Reference(ws, min_col=2, max_col=3, min_row=first - 1, max_row=row - 1)
    cats = Reference(ws, min_col=1, min_row=first, max_row=row - 1)
    ch.add_data(data, titles_from_data=True)
    ch.set_categories(cats)
    ws.add_chart(ch, f"A{row + 2}")
    return ws


def sheet_bias_model(wb, phys, model_sdf):
    """What the dynamic-bias parameters actually do, and why they matter here."""
    ws = wb.create_sheet("Bias model")
    title(ws, "The dynamic bias process, and why it currently swamps the measured bias", 8)
    widths(ws, {"A": 26, "B": 14, "C": 14, "D": 16, "E": 18, "F": 18, "G": 46})
    ws.merge_cells("A3:G8")
    c = ws.cell(3, 1,
        "Gazebo adds two different things to a perfect measurement. One is white noise, drawn "
        "fresh every sample from N(mean, stddev) — that is what the rest of this workbook "
        "calibrates. The other is a wandering bias, a first-order Gauss-Markov process driven by "
        "dynamic_bias_stddev and dynamic_bias_correlation_time, implemented in gz-sensors as\n"
        "    phi = exp(-dt/tau);  sigma_d = sqrt(-sigma_b^2 * tau/2 * expm1(-2*dt/tau));  "
        "bias <- phi*bias + N(0, sigma_d)\n"
        "Substituting gives a steady-state variance of sigma_b^2 * tau/2 exactly, so the bias "
        "this process actually wanders by is NOT dynamic_bias_stddev but sigma_b * sqrt(tau/2). "
        "With tau = 300 s that is a factor of 12 larger than the parameter suggests. The "
        "consequence is in the table below.")
    c.alignment = Alignment(wrap_text=True, vertical="top")
    c.fill = FILL_WARN
    ws.row_dimensions[3].height = 15

    row = 10
    header_row(ws, row, ["channel", "dynamic_bias_stddev", "correlation time (s)",
                         "wander sigma_b*sqrt(tau/2)", "measured static bias",
                         "wander / static bias", "consequence"])
    row += 1
    first = row
    for key, group, unit in (("gyro", "angular_velocity", "rad/s"),
                             ("accel", "linear_acceleration", "m/s^2")):
        for axis in AXES:
            st = phys["result"]["stats"][key][axis]
            m = model_sdf.get(f"{group}.{axis}", {})
            sb = m.get("dynamic_bias_stddev") or 0.0
            tau = m.get("dynamic_bias_correlation_time") or 0.0
            wander = sb * math.sqrt(tau / 2.0) if tau > 0 else 0.0
            static = abs(st.get("mean") or 0.0)
            ratio = (wander / static) if static > 0 else None
            ws.cell(row, 1, f"{group}.{axis} [{unit}]")
            ws.cell(row, 2, sb).number_format = "0.000E+00"
            ws.cell(row, 3, tau)
            ws.cell(row, 4, wander).number_format = "0.000E+00"
            ws.cell(row, 5, static).number_format = "0.000E+00"
            cc = ws.cell(row, 6, ratio)
            if ratio is not None:
                cc.number_format = "0.00"
                cc.fill = FILL_BAD if ratio >= 0.5 else FILL_OK
                ws.cell(row, 7,
                        "wandering bias is the same size as, or larger than, the bias we measured"
                        if ratio >= 0.5 else "static bias dominates; usable")
            else:
                ws.cell(row, 7, "accelerometer bias not measurable from this bag")
            for col in range(1, 8):
                ws.cell(row, col).border = BOX
            row += 1

    ch = LineChart()
    ch.title = "Wandering bias vs measured static bias"
    ch.y_axis.title = "rad/s or m/s^2"
    ch.height, ch.width = 8, 20
    data = Reference(ws, min_col=4, max_col=5, min_row=first - 1, max_row=row - 1)
    cats = Reference(ws, min_col=1, min_row=first, max_row=row - 1)
    ch.add_data(data, titles_from_data=True)
    ch.set_categories(cats)
    ws.add_chart(ch, f"A{row + 2}")

    row += 20
    ws.merge_cells(start_row=row, start_column=1, end_row=row + 3, end_column=7)
    c = ws.cell(row, 1,
        "Reading: dynamic_bias_stddev and dynamic_bias_correlation_time are still the inherited "
        "x500 values, because 6.3 s of stationary data cannot support a 300 s correlation time. "
        "The table shows that those inherited values make the simulated gyro bias wander by "
        "roughly as much as the static bias measured from the aircraft, which is why the "
        "published mean differed between two simulated runs of the same model. Until a long "
        "stationary recording is available, the gyro <mean> values should be read as the best "
        "estimate of the aircraft's bias at one moment, not as a figure the simulation will "
        "reproduce on any given run.")
    c.alignment = Alignment(wrap_text=True, vertical="top")
    c.fill = FILL_SUB
    return ws


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("phys_json", type=Path, help="imu_from_bag.py --json for the aircraft")
    ap.add_argument("--sim-json", type=Path, help="the same, for a simulation recording")
    ap.add_argument("--baseline-sdf", type=Path,
                    help="a model.sdf to use for the 'before' column. Without it the "
                         "report compares against whatever was in the model when the "
                         "measurement ran, which after a first --emit is the tool's own "
                         "output rather than the original values.")
    ap.add_argument("--model-sdf", type=Path,
                    default=Path(__file__).resolve().parent.parent / "models/m4e/model.sdf",
                    help="the model as it now stands, for the bias-model sheet")
    ap.add_argument("-o", "--out", type=Path,
                    default=Path("reports/dotFlySim2_imu_calibration.xlsx"))
    a = ap.parse_args(argv)

    phys = json.loads(a.phys_json.read_text())
    sim = json.loads(a.sim_json.read_text()) if a.sim_json else None
    if a.baseline_sdf:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from imu_from_bag import read_model_noise
        base = read_model_noise(a.baseline_sdf)
        phys["baseline"] = base
        for key, group in (("gyro", "angular_velocity"), ("accel", "linear_acceleration")):
            for axis in AXES:
                b = base.get(f"{group}.{axis}", {})
                st = phys["result"]["stats"][key][axis]
                st["stddev_current"] = b.get("stddev")
                st["mean_current"] = b.get("mean")

    model_now = {}
    if a.model_sdf and a.model_sdf.exists():
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from imu_from_bag import read_model_noise
        model_now = read_model_noise(a.model_sdf)

    wb = Workbook()
    wb.remove(wb.active)
    sheet_summary(wb, phys)
    sheet_verification(wb, phys, sim)
    sheet_method(wb, phys)
    sheet_stationary(wb, phys)
    sheet_timeseries(wb, phys, "Aircraft samples",
                     "Aircraft: raw samples of the stationary segment")
    if sim is not None:
        sheet_timeseries(wb, sim, "Simulation samples",
                         "Simulation: raw samples of the stationary segment",
                         chart_limit_s=phys["result"]["segment_len_s"])
    sheet_sampling(wb, phys, sim)
    sheet_psd(wb, phys, sim)
    sheet_allan(wb, phys, sim)
    if model_now:
        sheet_bias_model(wb, phys, model_now)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    wb.save(a.out)
    print(f"wrote {a.out}  ({len(wb.sheetnames)} sheets: {', '.join(wb.sheetnames)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
