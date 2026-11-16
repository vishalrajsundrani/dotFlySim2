#!/usr/bin/env python3
"""Generate config/perception_calib.yaml from the drone model.

The aircraft advertises perception_camera_parameters and never populates it;
the PSDK call behind it segfaults on a real M4E. So on hardware the intrinsics
and extrinsics have to come from a file, and the simulation must publish the
same shape of file through the same path or the two are calibrated
differently by construction.

Generated rather than hand written, because every number in it is already
stated in models/m4e/model.sdf. A hand copy would drift the first time a lens
moved, and a stereo rig whose calibration quietly disagrees with its geometry
fails in a way that looks like bad data rather than bad configuration.

Kalibr's layout, because that is what a visual-inertial system expects:
  camera_model     pinhole
  distortion_model equidistant   (Kannala-Brandt)
  intrinsics       [fu, fv, cu, cv]
  distortion       [k1, k2, k3, k4]

The intrinsics come from the aircraft calibrations in simsupport/. There are
two front fisheye calibrations; because the six perception lenses are the same
lens module, the front-left calibration is applied to fl, rl and dl and the
front-right calibration to fr, rr and dr. fx is also what sets the SDF's
horizontal_fov: the simulated lens is an ideal equidistant projection,
f = width / hfov, so the renderer and this file agree.

The simulated lens is ideal, so all four distortion coefficients are zero
even though the OpenCV calibration carries small radial/tangential terms.
Gazebo's WideAngleCamera has no intrinsics block and no principal-point
offset; it renders a radially symmetric lens centred on the image. fy, cx and
cy are still reported here because a consumer should not have to guess them.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# Which calibration belongs to which sensor. "left" in a sensor name is a
# position on the airframe; the calibration file names use the same sense
# (FRONT_LEFT is the lens on the left of the nose, not the left eye of a
# stereo pair). The two front calibrations are reused for the rear and down
# lenses of the same side.
CALIBRATION_FILES = {
    "perception_fl": "camera_calibration_FISHEYE_FRONT_LEFT.json",
    "perception_rl": "camera_calibration_FISHEYE_FRONT_LEFT.json",
    "perception_dl": "camera_calibration_FISHEYE_FRONT_LEFT.json",
    "perception_fr": "camera_calibration_FISHEYE_FRONT_RIGHT.json",
    "perception_rr": "camera_calibration_FISHEYE_FRONT_RIGHT.json",
    "perception_dr": "camera_calibration_FISHEYE_FRONT_RIGHT.json",
}

# direction -> (sensor feeding perception_stereo_LEFT_stream,
#               sensor feeding perception_stereo_RIGHT_stream)
#
# NOT inferred from the sensor names, deliberately. "left" in a sensor name is
# a position on the airframe; "left" in the topic name is an eye of a stereo
# pair, and for a rearward or sideways pair the two do not agree. Getting this
# backwards yields negative disparity, which reads as bad data rather than as
# a swapped pair. The rule is: the LEFT stream is the left eye as seen looking
# ALONG the direction of view.
PAIRS = {
    #            left eye         right eye
    "FRONT": ("perception_fl", "perception_fr"),
    # looking aft, the airframe's rear-right corner is on your left
    "REAR":  ("perception_rr", "perception_rl"),
    # looking to port, the nose is on your right
    "LEFT":  ("perception_rl", "perception_fl"),
    # looking to starboard, the nose is on your left
    "RIGHT": ("perception_fr", "perception_rr"),
    # looking down with the nose forward, the airframe's left is on your left
    "DOWN":  ("perception_dl", "perception_dr"),
}


def calibration(path: Path) -> dict:
    """Read the intrinsics out of an OpenCV calibration JSON."""
    d = json.loads(path.read_text())
    K = d["camera_matrix"]
    try:
        source = path.relative_to(REPO).as_posix()
    except ValueError:
        source = str(path)
    return {
        "source": source,
        "width": int(d["image_size"]["width"]),
        "height": int(d["image_size"]["height"]),
        "fx": float(K[0][0]),
        "fy": float(K[1][1]),
        "cx": float(K[0][2]),
        "cy": float(K[1][2]),
        "distortion": [float(v) for v in d["distortion_coefficients"][:4]],
    }


def sensors(sdf: Path, calibration_dir: Path) -> dict:
    text = sdf.read_text()
    out = {}
    for m in re.finditer(
            r'<sensor name="(perception_\w+)" type="(\w+)"(.*?)</sensor>', text, re.S):
        name, kind, body = m.group(1), m.group(2), m.group(3)

        def g(tag, cast=str, default=None):
            mm = re.search(rf"<{tag}>([^<]+)</{tag}>", body)
            return cast(mm.group(1)) if mm else default

        pose = [float(v) for v in g("pose").split()]
        w, h = g("width", int), g("height", int)
        hfov = g("horizontal_fov", float)
        out[name] = {
            "pose": pose, "w": w, "h": h, "hfov": hfov,
            "lens": g("type", str, "pinhole"),
            "rate": g("update_rate", float),
            "topic": g("topic"),
            "f": w / hfov,            # equidistant: r = f * theta
        }
        filename = CALIBRATION_FILES.get(name)
        if filename:
            path = calibration_dir / filename
            if not path.is_file():
                raise SystemExit(
                    f"{name}: calibration {path} not found. Restore it or pass "
                    f"--calibration-dir pointing at the directory that holds it.")
            out[name]["calibration"] = calibration(path)
    return out


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def invert(R, t):
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    ti = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    return Rt, ti


def matrix_rows(R, t):
    return ([[*R[i], t[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]])


def fmt(rows) -> str:
    return "\n".join(
        "      - [" + ", ".join(f"{v: .8f}" for v in row) + "]" for row in rows)


def build(sdf: Path, calibration_dir: Path) -> str:
    S = sensors(sdf, calibration_dir)
    missing = {s for pair in PAIRS.values() for s in pair} - set(S)
    if missing:
        raise SystemExit(f"{sdf}: model is missing {', '.join(sorted(missing))}")

    L = []
    L.append("# GENERATED by tools/gen_perception_calib.py from models/m4e/model.sdf")
    L.append("# and the front-fisheye calibrations in simsupport/.")
    L.append("# Do not hand edit: re-run the tool. See reports/CAMERA_ARCHITECTURE.md")
    L.append("#")
    L.append("# The aircraft never populates perception_camera_parameters (the PSDK call")
    L.append("# behind it segfaults), so this file is what BOTH the simulation and")
    L.append("# on-aircraft code read. Replace the simsupport JSONs with a calibration")
    L.append("# of the aircraft and re-run the tool; nothing else has to change.")
    L.append("")
    L.append("# Which sensor feeds which stereo stream, per direction. The LEFT stream is")
    L.append("# the left eye as seen looking ALONG the direction of view, which for the")
    L.append("# rear and lateral pairs is NOT the sensor whose name contains 'left'.")
    L.append("directions:")
    for d, (left, right) in PAIRS.items():
        a, b = S[left], S[right]
        base = math.dist(a["pose"][:3], b["pose"][:3])
        L.append(f"  {d}:")
        L.append(f"    left:  {left}")
        L.append(f"    right: {right}")
        L.append(f"    baseline_m: {base:.4f}")
    L.append("")
    L.append("cameras:")
    for name in sorted(S):
        s = S[name]
        R = rpy_to_matrix(*s["pose"][3:6])
        t = s["pose"][0:3]
        # T_cam_body: body (FLU, base_link) expressed in the camera frame
        Rc, tc = invert(R, t)
        L.append(f"  {name}:")
        L.append(f"    topic: {s['topic']}")
        L.append(f"    camera_model: pinhole")
        L.append(f"    distortion_model: equidistant")
        L.append(f"    resolution: [{s['w']}, {s['h']}]")
        L.append(f"    rate_hz: {s['rate']:.0f}")
        L.append(f"    hfov_deg: {math.degrees(s['hfov']):.4f}")
        cal = s.get("calibration")
        if cal:
            if (cal["width"], cal["height"]) != (s["w"], s["h"]):
                raise SystemExit(
                    f"{cal['source']}: image size {cal['width']}x{cal['height']} "
                    f"does not match {name}'s {s['w']}x{s['h']}")
            # The SDF's hfov and the calibrated fx are two statements of the
            # same lens. Letting them drift would make the rendered image and
            # the published calibration disagree in a way nothing else checks.
            expected = s["w"] / cal["fx"]
            if abs(expected - s["hfov"]) > 1e-5:
                raise SystemExit(
                    f"{sdf}: {name} has horizontal_fov {s['hfov']:.8f} rad but "
                    f"{cal['source']} implies {expected:.8f} rad (width / fx). "
                    f"Update the model's hfov or the calibration.")
            fx, fy = cal["fx"], cal["fy"]
            cx, cy = cal["cx"], cal["cy"]
            L.append(f"    # fx, fy, cx, cy from {cal['source']}.")
            L.append(f"    # fx is fixed by horizontal_fov: {s['w']} / "
                     f"{s['hfov']:.8f} = {s['f']:.4f} px.")
            L.append(f"    intrinsics: [{fx:.6f}, {fy:.6f}, {cx:.6f}, {cy:.6f}]")
            L.append(f"    # The calibration's OpenCV radial/tangential distortion is")
            L.append(f"    # not applied: the simulated lens is an ideal equidistant")
            L.append(f"    # projection.")
        else:
            L.append(f"    # fu = fv = width / hfov; an ideal equidistant lens")
            L.append(f"    intrinsics: [{s['f']:.4f}, {s['f']:.4f}, "
                     f"{s['w'] / 2:.1f}, {s['h'] / 2:.1f}]")
        L.append(f"    distortion_coeffs: [0.0, 0.0, 0.0, 0.0]")
        L.append(f"    # the IMU is at base_link; this is T_cam_imu, metres and radians")
        L.append(f"    T_cam_imu:")
        L.append(fmt(matrix_rows(Rc, tc)))
    L.append("")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, default=REPO / "models/m4e/model.sdf")
    ap.add_argument("--calibration-dir", type=Path, default=REPO / "simsupport",
                    help="directory holding camera_calibration_FISHEYE_FRONT_*.json")
    ap.add_argument("-o", "--out", type=Path,
                    default=REPO / "config/perception_calib.yaml")
    a = ap.parse_args(argv)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(build(a.model, a.calibration_dir))
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
