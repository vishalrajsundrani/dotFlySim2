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


def _segment(d, which):
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
    m = (t >= lo) & (t <= hi)
    return t[m] - t[m][0] if m.any() else t[m], x[m]


def sheet_timeseries(wb, d):
    ws = wb.create_sheet("Stationary samples")
    title(ws, "The stationary segment, sample by sample", 9)
    widths(ws, {"A": 11, "B": 12, "C": 12, "D": 12, "F": 11, "G": 12, "H": 12, "I": 12})
    anchor = {"gyro": (1, "A"), "accel": (6, "F")}
    for which, (col0, letter) in anchor.items():
        t, x = _segment(d, which)
        unit = "rad/s" if which == "gyro" else "m/s²"
        c = ws.cell(3, col0, f"{which} [{unit}]")
        c.font = H2
        header_row(ws, 4, ["t (s)", "x", "y", "z"], start=col0)
        for i in range(len(t)):
            ws.cell(5 + i, col0, round(float(t[i]), 4))
            for j in range(3):
                ws.cell(5 + i, col0 + 1 + j, float(x[i, j]))
        if len(t) == 0:
            continue
        ch = LineChart()
        ch.title = f"{which} during the stationary segment"
        ch.y_axis.title = unit
        ch.x_axis.title = "t (s)"
        ch.height, ch.width = 8, 18
        data = Reference(ws, min_col=col0 + 1, max_col=col0 + 3, min_row=4,
                         max_row=4 + len(t))
        ch.add_data(data, titles_from_data=True)
        ws.add_chart(ch, f"{letter}{8 + len(t)}")
    return ws


def sheet_psd(wb, d):
    ws = wb.create_sheet("Spectrum")
    title(ws, "Why the density is read off the spectrum, not off the variance", 10)
    widths(ws, {"A": 60})
    ws.merge_cells("A3:A6")
    c = ws.cell(3, 1,
        "White sensor noise is flat in frequency. Anything rising at the left of these curves is "
        "real low-frequency motion — a parked airframe swaying on its legs — and counting it as "
        "sensor noise would make the simulated IMU noisier than the real part. Anything falling at "
        "the right is a low-pass filter applied before the data was published, which makes the "
        "measured density a lower bound.")
    c.alignment = Alignment(wrap_text=True, vertical="top")
    c.fill = FILL_WARN

    r = d["result"]
    col = 3
    for key in ("gyro", "accel"):
        for axis in AXES:
            s = r["stats"][key][axis]
            f = s.get("psd_f") or []
            p = s.get("psd") or []
            if not f:
                continue
            ws.cell(3, col, "freq (Hz)").font = H2
            ws.cell(3, col + 1, f"{key}.{axis}").font = H2
            for i in range(1, len(f)):
                ws.cell(3 + i, col, round(float(f[i]), 3))
                ws.cell(3 + i, col + 1, float(p[i]))
            ch = ScatterChart()
            ch.title = f"PSD {key}.{axis}"
            ch.x_axis.title = "Hz"
            ch.y_axis.title = "power / Hz"
            ch.y_axis.scaling.logBase = 10
            ch.height, ch.width = 7, 12
            xs = Reference(ws, min_col=col, min_row=4, max_row=3 + len(f) - 1)
            ser = Series(Reference(ws, min_col=col + 1, min_row=3, max_row=3 + len(f) - 1),
                         xs, title_from_data=True)
            ser.marker.symbol = "none"
            ch.series.append(ser)
            anchor_row = 8 + 15 * (0 if key == "gyro" else 1)
            ws.add_chart(ch, f"{get_column_letter(col)}{anchor_row + 30}")
            col += 3
    return ws


def sheet_allan(wb, d):
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
            s = r["stats"][key][axis]
            taus, devs = s.get("allan_tau") or [], s.get("allan_dev") or []
            if not taus:
                continue
            ws.cell(8, col, "τ (s)").font = H2
            ws.cell(8, col + 1, f"{key}.{axis}").font = H2
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
            s = r["stats"][key][axis]
            n = len(s.get("allan_tau") or [])
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

    wb = Workbook()
    wb.remove(wb.active)
    sheet_summary(wb, phys)
    sheet_verification(wb, phys, sim)
    sheet_method(wb, phys)
    sheet_stationary(wb, phys)
    sheet_timeseries(wb, phys)
    sheet_psd(wb, phys)
    sheet_allan(wb, phys)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    wb.save(a.out)
    print(f"wrote {a.out}  ({len(wb.sheetnames)} sheets: {', '.join(wb.sheetnames)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
