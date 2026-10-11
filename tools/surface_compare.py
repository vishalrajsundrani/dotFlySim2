#!/usr/bin/env python3
"""Compare three descriptions of the same aircraft interface.

  documented   what psdk_ros2 says a Matrice 4E publishes and serves
  recorded     what a real M4E actually put on the wire (a rosbag)
  simulated    what this simulation publishes and serves

They disagree in ways that matter, and the disagreements are not symmetric:
the documentation lists topics the aircraft never populates, the recording
contains only what was switched on that day, and the simulation renders only
what someone has built. A gap in one column means something different from a
gap in another, so the report keeps all three rather than reducing them to a
pass/fail.

Written for the visual-localisation work, so the camera sheets carry the real
detail: sensor geometry, projection model, resolution and rate, against what
the aircraft's own specification says.

    tools/surface_compare.py --bag /path/to/bag_000026 \
        -o reports/dotFlySim2_surface_comparison.xlsx
"""

from __future__ import annotations

import argparse
import importlib.util
import re
import sys
from pathlib import Path

from openpyxl import Workbook
from openpyxl.chart import BarChart, Reference
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side

REPO = Path(__file__).resolve().parent.parent
PREFIX = "/wrapper/psdk_ros2/"

H1 = Font(bold=True, size=14, color="FFFFFF")
H2 = Font(bold=True, size=11)
FILL_H = PatternFill("solid", fgColor="1F3864")
FILL_SUB = PatternFill("solid", fgColor="D9E2F3")
FILL_OK = PatternFill("solid", fgColor="E2EFDA")
FILL_WARN = PatternFill("solid", fgColor="FFF2CC")
FILL_BAD = PatternFill("solid", fgColor="FBE5D6")
THIN = Side(style="thin", color="BFBFBF")
BOX = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)


def title(ws, text, width=8):
    ws.merge_cells(start_row=1, start_column=1, end_row=1, end_column=width)
    c = ws.cell(1, 1, text)
    c.font, c.fill = H1, FILL_H
    c.alignment = Alignment(horizontal="left", vertical="center")
    ws.row_dimensions[1].height = 24


def header_row(ws, row, labels, start=1):
    for i, lab in enumerate(labels):
        c = ws.cell(row, start + i, lab)
        c.font, c.fill, c.border = H2, FILL_SUB, BOX
        c.alignment = Alignment(horizontal="center", wrap_text=True)


def widths(ws, spec):
    for col, w in spec.items():
        ws.column_dimensions[col].width = w


def note(ws, row, col, span, text, fill=FILL_WARN, height=5):
    ws.merge_cells(start_row=row, start_column=col, end_row=row + height - 1,
                   end_column=col + span - 1)
    c = ws.cell(row, col, text)
    c.alignment = Alignment(wrap_text=True, vertical="top")
    c.fill, c.border = fill, BOX
    return row + height


# --------------------------------------------------------------------------
# the three sources
# --------------------------------------------------------------------------

def documented() -> dict:
    spec = importlib.util.spec_from_file_location("ps", REPO / "tools/psdk_surface.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return {
        "telemetry": set(m.TELEMETRY),
        "no_data": set(m.NO_DATA),
        "command": set(m.COMMAND),
        "services": set(m.SERVICES),
    }


def recorded(bag: Path) -> dict:
    """Topics, types and rates straight out of the bag's own metadata."""
    meta = bag / "metadata.yaml"
    if not meta.is_file():
        raise SystemExit(f"{meta}: not found")
    text = meta.read_text()
    pairs = re.findall(r"name:\s*(\S+)\s*\n\s*type:\s*(\S+)", text)
    counts = [int(c) for c in re.findall(r"message_count:\s*(\d+)", text)]
    dur = int(re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text).group(1)) / 1e9
    out = {}
    for (name, typ), count in zip(pairs, counts[1:]):
        out[name] = {"type": typ, "count": count, "hz": count / dur if dur else 0.0}
    return {"topics": out, "duration": dur}


def simulated() -> dict:
    """What the bridge exposes, read from the registry rather than guessed.

    SERVICES ARE NOT ALL DECLARED THE SAME WAY, and an earlier version of this
    function assumed they were. It matched only literal
    `ServiceRoute("key", "name", ...)` calls, which missed every route built in
    a loop: the ten obstacle-avoidance pairs come from an f-string over five
    directions, and the fourteen camera-settings rows from a comprehension
    over a table of tuples. The report then announced 24 documented services
    as unserved when all 56 are in fact defined. Matching string literals
    against the documented set is cruder but cannot miss a route for being
    built cleverly.

    DEFINED IS NOT THE SAME AS ADVERTISED. A ServiceRoute with enabled=False
    is never created (node._realise_service returns early), so a caller gets
    "service not available" exactly as if it did not exist. The camera
    settings pairs ship that way, deliberately: they store a value and change
    no pixel. They are reported separately rather than counted as served,
    because what a mission meets is what is advertised.
    """
    src = (REPO / "bridge/simty/registry.py").read_text()
    topics = set(re.findall(r'host\(\s*"([a-z0-9_]+)"\s*\)', src))

    # Three ways a route is declared, and all three have to be read. Matching
    # every string literal instead would be simpler and wrong: it sweeps up
    # internal keys, shared-state names and topic names that are not services
    # at all, and then reports them as surface the aircraft lacks.
    #
    #   1. the common case, two literals
    exposed = set(re.findall(
        r'ServiceRoute\(\s*"[a-z0-9_]+"\s*,\s*"([a-z0-9_]+)"', src))
    #   2. obstacle avoidance: an f-string over five directions
    directions = re.search(r"_OBSTACLE_AVOIDANCE_DIRECTIONS\s*=\s*\(([^)]*)\)", src, re.S)
    if directions:
        for d in re.findall(r'"([a-z_]+)"', directions.group(1)):
            exposed |= {f"set_{d}_obstacle_avoidance", f"get_{d}_obstacle_avoidance"}
    #   3. camera settings: a comprehension over a table of tuples. The same
    #      block is the one that ships disabled, so it is read once for both.
    disabled: set[str] = set()
    try:
        a = src.index("# Camera settings: set/get pairs, stored only.")
        b = src.index('ServiceRoute("camera_set_infrared_zoom"', a)
        disabled = set(re.findall(r'"(camera_(?:set|get)_\w+)"', src[a:b]))
    except ValueError:
        pass
    defined = exposed | disabled

    gap = set(re.findall(r'^\s*"([a-z0-9_]+)":\s*"',
                         src[src.index("SYNTHESIS_GAP = {"):
                             src.index("SYNTHESIS_GAP = {") + 2000], re.M))
    return {"topics": topics, "services": defined, "disabled": disabled, "gap": gap}


def sdf_cameras(path: Path, pattern: str = "") -> list:
    """Camera sensors from a model, via the shared parser in sdf_cameras.py."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import sdf_cameras as _s
    out = []
    for name, spec in sorted(_s.read(path).items()):
        out.append({"name": name, "type": spec["kind"], "w": spec["width"],
                    "h": spec["height"], "format": spec["format"],
                    "hfov": spec["hfov_rad"], "rate": spec["rate_hz"],
                    "lens": spec["lens"], "mpix_s": spec["mpix_s"]})
    return out


# --------------------------------------------------------------------------
# what the aircraft's own specification says
# --------------------------------------------------------------------------
# From DJI's published Matrice 4E sensing specification. Recorded here because
# the simulation's fisheye geometry has to be checked against something, and
# neither the bag nor psdk_ros2 exposes it.
M4E_VISION = [
    # direction, horizontal FOV, vertical FOV, range, note
    ("FRONT", 90, 135, "0.4-22.5 m binocular, 0.4-200 m measurement",
     "stereo pair; vertical FOV is 135, not 90"),
    ("REAR", 90, 135, "0.4-22.5 m binocular, 0.4-200 m measurement",
     "stereo pair; vertical FOV is 135, not 90"),
    ("LEFT", 90, 90, "0.5-32 m", "lateral stereo"),
    ("RIGHT", 90, 90, "0.5-32 m", "lateral stereo"),
    ("DOWN", 160, 160, "0.3-18.8 m", "plus a 3D infrared sensor"),
    ("UP", None, None, "-",
     "no dedicated stereo pair; DJI's FOV table lists no upward row"),
]
M4E_VISION_SOURCE = ("DJI Matrice 4 Series sensing specification: six HD low-light "
                     "fisheye lenses, omnidirectional binocular vision plus a 3D "
                     "infrared sensor at the bottom.")


def sheet_topics(wb, doc, rec, sim):
    ws = wb.create_sheet("Topics")
    title(ws, "Topics: documented vs recorded on the aircraft vs published by the simulation", 8)
    widths(ws, {"A": 36, "B": 13, "C": 13, "D": 13, "E": 11, "F": 10, "G": 44})
    row = 3
    row = note(ws, row, 1, 7,
        "A blank in 'recorded' does not mean the aircraft cannot publish it. The bag is one "
        "184-second flight: anything that was switched off, or that the flight controller only "
        "populates in another mode, is simply absent. A blank in 'simulated' does mean the "
        "simulation does not produce it. Read the two columns differently.", height=4)
    row += 1

    names = sorted(doc["telemetry"] | doc["no_data"] | doc["command"] | sim["topics"]
                   | {t[len(PREFIX):] for t in rec["topics"] if t.startswith(PREFIX)})
    header_row(ws, row, ["wrapper topic", "documented", "recorded", "msgs", "Hz",
                         "simulated", "reading"])
    row += 1
    first = row
    for n in names:
        full = PREFIX + n
        r = rec["topics"].get(full)
        is_doc = n in doc["telemetry"] or n in doc["command"]
        is_nodata = n in doc["no_data"]
        in_sim = n in sim["topics"] and n not in sim["gap"]
        ws.cell(row, 1, n)
        ws.cell(row, 2, "yes" if is_doc else ("listed, no data" if is_nodata else ""))
        ws.cell(row, 3, "yes" if r else "")
        ws.cell(row, 4, r["count"] if r else None)
        ws.cell(row, 5, round(r["hz"], 2) if r else None)
        c = ws.cell(row, 6, "yes" if in_sim else ("gap" if n in sim["gap"] else ""))
        reading = ""
        if r and r["count"] == 0:
            reading = "topic advertised by the aircraft but never populated"
        if r and r["count"] and not in_sim:
            reading = "AIRCRAFT PUBLISHES IT, SIMULATION DOES NOT"
            c.fill = FILL_BAD
        elif in_sim and not r:
            reading = "simulation publishes it; not present in this recording"
            c.fill = FILL_WARN
        elif in_sim and r and r["count"]:
            c.fill = FILL_OK
        ws.cell(row, 7, reading)
        for col in range(1, 8):
            ws.cell(row, col).border = BOX
        row += 1

    ws.cell(row + 1, 1, f"{len(names)} distinct wrapper topics considered").font = H2
    ws.cell(row + 2, 1, f"recorded with data: "
                        f"{sum(1 for t,v in rec['topics'].items() if t.startswith(PREFIX) and v['count'])}")
    ws.cell(row + 3, 1, f"published by the simulation: {len(sim['topics'] - sim['gap'])}")
    return ws


def sheet_services(wb, doc, sim):
    ws = wb.create_sheet("Services")
    title(ws, "Services: documented vs served by the simulation", 5)
    widths(ws, {"A": 44, "B": 13, "C": 13, "D": 16, "E": 52})
    row = 3
    row = note(ws, row, 1, 5,
        "A rosbag records topics, not service calls, so the aircraft column cannot exist here: "
        "nothing in the recording says which services the aircraft would have answered. The "
        "comparison is therefore documentation against simulation only.", height=3)
    row += 1
    header_row(ws, row, ["service", "documented", "simulated", "group", "note"])
    row += 1
    for s in sorted(doc["services"] | sim["services"]):
        in_doc, in_sim = s in doc["services"], s in sim["services"]
        ws.cell(row, 1, s)
        ws.cell(row, 2, "yes" if in_doc else "")
        c = ws.cell(row, 3, "yes" if in_sim else "")
        grp = ("camera" if s.startswith("camera") else
               "perception" if "perception" in s else
               "gimbal" if s.startswith("gimbal") else
               "obstacle avoidance" if "obstacle_avoidance" in s else "flight")
        ws.cell(row, 4, grp)
        if in_doc and s in sim.get("disabled", ()):
            c.fill = FILL_WARN
            ws.cell(row, 5, "defined but shipped disabled: it stores a value and "
                            "changes no pixel, so it is not advertised and a caller "
                            "gets 'service not available' until it is enabled")
        elif in_doc and not in_sim:
            c.fill = FILL_BAD
            ws.cell(row, 5, "documented, not defined by the simulation")
        elif in_sim and not in_doc:
            c.fill = FILL_WARN
            ws.cell(row, 5, "served by the simulation; absent from the transcribed PDF list "
                            "(the online documentation does list gimbal services)")
        else:
            c.fill = FILL_OK
        for col in range(1, 6):
            ws.cell(row, col).border = BOX
        row += 1
    served = doc["services"] & sim["services"]
    ws.cell(row + 1, 1,
            f"documented {len(doc['services'])} \u00b7 defined here {len(served)} \u00b7 "
            f"shipped disabled {len(sim.get('disabled', ()))} \u00b7 "
            f"advertised by default {len(served) - len(sim.get('disabled', ()))} \u00b7 "
            f"not defined {len(doc['services'] - sim['services'])}").font = H2
    return ws


def sheet_vision(wb, percep):
    """The fisheye/perception cameras: aircraft specification vs what is modelled."""
    ws = wb.create_sheet("Vision system")
    title(ws, "Perception cameras: what the aircraft has vs what the simulation models", 9)
    widths(ws, {"A": 13, "B": 10, "C": 10, "D": 30, "E": 24, "F": 11, "G": 11,
                "H": 20, "I": 50})
    row = 3
    row = note(ws, row, 1, 9, M4E_VISION_SOURCE +
        "  The simulation's sensors are read directly from models/m4e/model.sdf, so this table "
        "is generated, not transcribed.", fill=FILL_SUB, height=3)
    row += 1

    header_row(ws, row, ["direction", "real H FOV", "real V FOV", "real range",
                         "simulated sensor(s)", "sim res", "sim FOV",
                         "sim projection", "verdict"])
    row += 1
    by_dir = {
        "FRONT": [c for c in percep if "front" in c["name"]],
        "REAR": [c for c in percep if "back" in c["name"] or "rear" in c["name"]],
        "LEFT": [c for c in percep if c["name"].endswith("_left") and "front" not in c["name"]
                 and "back" not in c["name"]],
        "RIGHT": [c for c in percep if c["name"].endswith("_right") and "front" not in c["name"]
                  and "back" not in c["name"]],
        "DOWN": [c for c in percep if "down" in c["name"]],
        "UP": [],
    }
    import math
    for direction, hfov, vfov, rng, remark in M4E_VISION:
        cams = by_dir.get(direction, [])
        ws.cell(row, 1, direction).font = H2
        ws.cell(row, 2, f"{hfov} deg" if hfov else "n/a")
        ws.cell(row, 3, f"{vfov} deg" if vfov else "n/a")
        ws.cell(row, 4, rng)
        ws.cell(row, 5, ", ".join(c["name"] for c in cams) or "not modelled")
        if cams:
            c0 = cams[0]
            sim_h = math.degrees(c0["hfov"]) if c0["hfov"] else 0
            # a square sensor under a pinhole projection has V FOV == H FOV
            sim_v = sim_h if c0["w"] == c0["h"] else None
            ws.cell(row, 6, f"{c0['w']}x{c0['h']}")
            ws.cell(row, 7, f"{sim_h:.0f} deg")
            pc = ws.cell(row, 8, c0["lens"])
            verdict = []
            if hfov and abs(sim_h - hfov) > 5:
                verdict.append(f"H FOV {sim_h:.0f} vs {hfov}")
            if vfov and sim_v is not None and abs(sim_v - vfov) > 5:
                verdict.append(f"V FOV {sim_v:.0f} vs {vfov} (square sensor cannot give {vfov})")
            if "pinhole" in c0["lens"] and (hfov or 0) >= 120:
                verdict.append("PINHOLE AT A WIDE FOV IS GEOMETRICALLY WRONG")
                pc.fill = FILL_BAD
            elif "pinhole" in c0["lens"]:
                verdict.append("pinhole, not a fisheye projection")
                pc.fill = FILL_WARN
            cc = ws.cell(row, 9, "; ".join(verdict) or "matches")
            cc.fill = FILL_BAD if any("WRONG" in v for v in verdict) else (
                FILL_WARN if verdict else FILL_OK)
        else:
            ws.cell(row, 9, remark).fill = FILL_BAD if direction != "UP" else FILL_OK
        for col in range(1, 10):
            ws.cell(row, col).border = BOX
        row += 1

    row += 1
    note(ws, row, 1, 9,
        "Only FRONT is wired to ROS topics. models/m4e/walker.toml lists two sensors "
        "(perception_front_left, perception_front_right); registry._PERCEPTION_SIMULATED is "
        "('FRONT',). The other five sensors exist in the SDF but reach nothing, and "
        "start_perception refuses those directions explicitly rather than succeeding and "
        "publishing silence — which is the right behaviour, but it does mean omnidirectional "
        "visual localisation cannot be developed against this simulation today.", height=5)
    return ws


def sheet_streams(wb, rec, payload):
    """The image streams that actually reach ROS."""
    ws = wb.create_sheet("Camera streams")
    title(ws, "Image streams: what the recording contains vs what the simulation renders", 7)
    widths(ws, {"A": 32, "B": 26, "C": 16, "D": 11, "E": 13, "F": 54})
    row = 3
    header_row(ws, row, ["stream", "recorded format", "recorded rate", "msgs",
                         "simulated", "note"])
    row += 1
    main = rec["topics"].get(PREFIX + "main_camera_stream")
    rows = [
        ("main_camera_stream",
         "rgb8, 1440x1080 AND 1920x1080" if main else "",
         "23.5 Hz mean, 28.6 Hz median" if main else "",
         main["count"] if main else 0,
         "yes, fixed 1920x1080 @30 Hz",
         "THE RECORDING CHANGES ASPECT RATIO FIVE TIMES mid-flight (4:3 <-> 16:9) at "
         "t = 0, 2.7, 3.8, 29.5 and 139.0 s. The simulation emits one fixed size, so a "
         "consumer that assumes constant intrinsics passes in simulation and breaks on "
         "the aircraft."),
        ("fpv_camera_stream", "", "", 0, "route exists",
         "absent from the recording; the wrapper lists it under 'no data'"),
        ("perception_stereo_left_stream", "", "", 0, "FRONT only, mono8 704x704 @20 Hz",
         "NOT PRESENT IN THE RECORDING AT ALL - not even as an empty topic. The perception "
         "node was never started, so this flight carries no stereo imagery whatsoever."),
        ("perception_stereo_right_stream", "", "", 0, "FRONT only, mono8 704x704 @20 Hz",
         "as above"),
        ("perception_camera_parameters", "declared, 0 messages", "", 0,
         "deliberately unpublished",
         "the aircraft advertises it and never populates it; the bridge notes the underlying "
         "PSDK call segfaults on a real M4E, so intrinsics must come from a calibration file"),
    ]
    for name, fmt, rate, cnt, sim_s, nt in rows:
        ws.cell(row, 1, name)
        ws.cell(row, 2, fmt)
        ws.cell(row, 3, rate)
        ws.cell(row, 4, cnt)
        ws.cell(row, 5, sim_s)
        c = ws.cell(row, 6, nt)
        c.alignment = Alignment(wrap_text=True, vertical="top")
        c.fill = FILL_BAD if ("NOT PRESENT" in nt or "CHANGES ASPECT" in nt) else FILL_WARN
        for col in range(1, 7):
            ws.cell(row, col).border = BOX
        ws.row_dimensions[row].height = 46
        row += 1

    row += 1
    ws.cell(row, 1, "Payload sensors modelled in models/m4e_camera/model.sdf").font = H2
    row += 1
    header_row(ws, row, ["sensor", "resolution", "format", "H FOV (deg)", "Hz"])
    row += 1
    import math
    for c in payload:
        ws.cell(row, 1, c["name"])
        ws.cell(row, 2, f"{c['w']}x{c['h']}")
        ws.cell(row, 3, c["format"])
        ws.cell(row, 4, round(math.degrees(c["hfov"]), 1) if c["hfov"] else None)
        ws.cell(row, 5, c["rate"])
        for col in range(1, 6):
            ws.cell(row, col).border = BOX
        row += 1
    return ws


ACTIONS = [
    (1, "Replace the perception pinhole with a real fisheye projection",
     "models/m4e/model.sdf",
     "All seven perception sensors are plain pinhole cameras. A pinhole cannot represent a "
     "160-degree lens at all: the projection diverges towards 180 and the edge pixels are "
     "stretched without bound. Gazebo's wideanglecamera with <lens><type>equidistant</type> "
     "(or equisolid_angle) is the right sensor. Until this changes, any descriptor, "
     "undistortion or depth estimate tuned in simulation is tuned against a lens the "
     "aircraft does not have.",
     "highest: it invalidates transfer of anything geometric"),
    (2, "Give FRONT and REAR their real 135-degree vertical field of view",
     "models/m4e/model.sdf",
     "The sensors are square (704x704) with a 90-degree horizontal FOV, so their vertical FOV "
     "is also 90. DJI specifies 90 horizontal by 135 vertical for the forward and backward "
     "pairs. A non-square sensor, or an explicit vertical FOV, is needed. The lateral pairs "
     "at 90x90 are already correct.",
     "high: vertical coverage drives how much structure a VIO front end sees"),
    (3, "Render more than the FRONT pair",
     "models/m4e/model.sdf, models/m4e/walker.toml, config/bridge.yaml, registry",
     "Five of the seven sensors exist in the SDF but are wired to nothing, and "
     "_PERCEPTION_SIMULATED is ('FRONT',). start_perception correctly refuses the others "
     "rather than publishing silence, but that means omnidirectional localisation cannot be "
     "exercised. DOWN matters most for a localisation system: it is the pair with the widest "
     "FOV and the most texture during an inspection hover.",
     "high, and the cheapest of the three to do"),
    (4, "Publish camera_info for the perception pair",
     "config/perception_calib.yaml, bridge",
     "perception_camera_parameters is advertised by the aircraft and never populated - the "
     "underlying PSDK call segfaults - so on hardware the intrinsics must come from a "
     "calibration file. The simulation should publish the same file through the same path, so "
     "the localisation system reads its calibration identically in both places.",
     "high: without it the sim and the aircraft are calibrated differently"),
    (5, "Decide what main_camera_stream does about resolution",
     "bridge, models/m4e_camera/model.sdf",
     "The recording switches between 1440x1080 and 1920x1080 five times in 184 seconds. The "
     "simulation emits one fixed size forever. A consumer that caches intrinsics on the first "
     "frame works in simulation and is wrong on the aircraft the first time the operator "
     "changes mode.",
     "medium for localisation, high for anything using the payload"),
    (6, "Resolve the accelerometer gravity difference",
     "bridge/simty/converters.py",
     "Carried over from the IMU work: the aircraft's imu.linear_acceleration has gravity "
     "removed (mean 0.0036 m/s2), the simulation's does not (9.7978 m/s2). A visual-inertial "
     "system integrates this term directly, so the two environments will not agree until it "
     "is settled.",
     "highest for VIO specifically - it is a 9.8 m/s2 bias"),
]


def sheet_actions(wb):
    ws = wb.create_sheet("What to change")
    title(ws, "Ordered by what blocks a visual-localisation system first", 5)
    widths(ws, {"A": 5, "B": 44, "C": 40, "D": 66, "E": 34})
    row = 3
    header_row(ws, row, ["#", "change", "where", "why", "priority"])
    row += 1
    for n, what, where, why, prio in ACTIONS:
        ws.cell(row, 1, n)
        ws.cell(row, 2, what).font = H2
        ws.cell(row, 3, where)
        for col, val in ((4, why), (5, prio)):
            c = ws.cell(row, col, val)
            c.alignment = Alignment(wrap_text=True, vertical="top")
        ws.cell(row, 5).fill = FILL_BAD if "highest" in prio else (
            FILL_WARN if prio.startswith("high") else FILL_SUB)
        for col in range(1, 6):
            ws.cell(row, col).border = BOX
        ws.row_dimensions[row].height = 84
        row += 1
    return ws


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True,
                    help="the physical-aircraft rosbag directory")
    ap.add_argument("-o", "--out", type=Path,
                    default=REPO / "reports/dotFlySim2_surface_comparison.xlsx")
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if the simulation serves a service psdk_ros2 "
                         "does not, which is the drift that silently breaks a mission "
                         "when it meets the aircraft")
    a = ap.parse_args(argv)

    doc = documented()
    rec = recorded(a.bag)
    sim = simulated()
    percep = [c for c in sdf_cameras(REPO / "models/m4e/model.sdf")
              if c["name"].startswith("perception")]
    payload = sdf_cameras(REPO / "models/m4e_camera/model.sdf")

    wb = Workbook()
    wb.remove(wb.active)
    sheet_actions(wb)
    sheet_vision(wb, percep)
    sheet_streams(wb, rec, payload)
    sheet_topics(wb, doc, rec, sim)
    sheet_services(wb, doc, sim)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    wb.save(a.out)

    print(f"wrote {a.out}")
    print(f"  bag {a.bag.name}: {len(rec['topics'])} topics over {rec['duration']:.1f} s")
    print(f"  simulation: {len(sim['topics'] - sim['gap'])} wrapper topics, "
          f"{len(sim['services'])} services defined "
          f"({len(sim.get('disabled', ()))} shipped disabled)")
    print(f"  documented: {len(doc['telemetry'] | doc['no_data'] | doc['command'])} topics, "
          f"{len(doc['services'])} services")
    print(f"  perception sensors modelled: {len(percep)}; payload sensors: {len(payload)}")

    # DRIFT IN ONE DIRECTION IS WORSE THAN THE OTHER. A documented service the
    # simulation lacks is a gap: a mission fails here and works on the
    # aircraft, which is annoying but visible. A service the simulation serves
    # and the aircraft does not is a trap: the mission works here and fails in
    # the field. Only the second fails this check.
    invented = sorted(sim["services"] - doc["services"])
    known_extra = {"gimbal_reset", "gimbal_set_mode"}   # real, absent from the PDF list
    bad = [s for s in invented if s not in known_extra]
    if bad:
        print()
        print("  SERVED HERE AND NOT BY psdk_ros2 -- a mission using these would "
              "fail on the aircraft:")
        for s in bad:
            print(f"    {s}")
        if a.check:
            return 1
    elif a.check:
        print("  check: no invented services")
    return 0


if __name__ == "__main__":
    sys.exit(main())
