"""Read camera sensors out of an SDF model. Stdlib only.

One parser, three readers: the surface comparison report, walker's camera
registry, and the perception calibration generator all need the same facts
(resolution, rate, field of view, projection) and used to get them from
hand-written tables that drifted from the model. A lens moves in model.sdf and
every one of them is then quietly wrong.

Deliberately regex rather than ElementTree: models/m4e/model.sdf is more
comment than XML, and the comments carry the reasoning. A parse-and-rewrite
round trip would lose them, and anything that reads this file must not be
tempted to write it.
"""

from __future__ import annotations

import math
import re

_SENSOR = re.compile(
    r'<sensor name="([^"]+)"[^>]*type="(camera|wideanglecamera|depth_camera)"(.*?)</sensor>',
    re.S)


def _tag(body: str, name: str, cast=str, default=None):
    m = re.search(rf"<{name}>([^<]+)</{name}>", body)
    return cast(m.group(1)) if m else default


def cameras(sdf_text: str) -> dict[str, dict]:
    """{sensor name: spec} for every camera sensor in the model."""
    out: dict[str, dict] = {}
    for m in _SENSOR.finditer(sdf_text):
        name, kind, body = m.group(1), m.group(2), m.group(3)
        w = _tag(body, "width", int, 0)
        h = _tag(body, "height", int, 0)
        rate = _tag(body, "update_rate", float, 0.0)
        hfov = _tag(body, "horizontal_fov", float, 0.0)
        lens = re.search(r"<lens>.*?<type>(\w+)</type>", body, re.S)
        pose = _tag(body, "pose", str, "0 0 0 0 0 0")
        out[name] = {
            "sensor": name,
            "kind": kind,
            "topic": _tag(body, "topic", str, ""),
            "width": w,
            "height": h,
            "format": _tag(body, "format", str, ""),
            "rate_hz": rate,
            "hfov_rad": hfov,
            "hfov_deg": math.degrees(hfov) if hfov else 0.0,
            "lens": lens.group(1) if lens else "rectilinear",
            "pose": [float(v) for v in pose.split()],
            # What this sensor costs to render, which is the number walker's
            # camera screen is really choosing between.
            "mpix_s": (w * h * rate) / 1e6 if w and h and rate else 0.0,
        }
    return out


def read(path) -> dict[str, dict]:
    with open(path, encoding="utf-8") as fh:
        return cameras(fh.read())


def describe(spec: dict) -> str:
    """One line a human can read, derived rather than written down."""
    if not spec.get("width"):
        return spec.get("sensor", "?")
    bits = [f"{spec['width']}x{spec['height']}"]
    if spec.get("rate_hz"):
        bits.append(f"@{spec['rate_hz']:g} Hz")
    if spec.get("hfov_deg"):
        bits.append(f"{spec['hfov_deg']:.0f}°")
    if spec.get("lens") and spec["lens"] != "rectilinear":
        bits.append(spec["lens"])
    if spec.get("mpix_s"):
        bits.append(f"{spec['mpix_s']:.1f} MP/s")
    return " · ".join(bits)
