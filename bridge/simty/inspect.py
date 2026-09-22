"""
Field-level inspection of a single conversion: what went in, what came out.

The FLOW and ROUTES screens tell you a conversion is running and whether it is
complaining. They do not tell you what it actually *did* to the numbers -- and
that is the question that comes up when a client sees a plausible-looking value
that is nonetheless wrong: a metre read as a foot, a NED axis published as ENU,
an angle in degrees where radians were meant.

So: explode() flattens a live ROS message into (path, value) pairs, and pair()
lines the source message up against the message the converter produced, so the
two can be shown side by side with the changed rows marked.

Everything here is best-effort and must never raise. It runs on the executor
thread inside the message handler (capture) and on the render thread (display);
a debug view that can take the bridge down is worse than no debug view.
"""

import math
from typing import Any, List, Optional, Sequence, Tuple

# One row of a side-by-side comparison.
#   left  : (path, value) on the source side, or None when nothing corresponds
#   right : (path, value) on the sink side, or None
#   kind  : "same" | "changed" | "only-in" | "only-out"
Row = Tuple[Optional[Tuple[str, str]], Optional[Tuple[str, str]], str]

MAX_FIELDS = 80
MAX_DEPTH = 4
MAX_SEQUENCE = 6


# ── formatting ───────────────────────────────────────────────────────────────


def _format_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "+Inf" if value > 0 else "-Inf"
        # 10 significant digits, not 6. This view exists to catch conversions
        # that quietly change a number, and a degree of latitude is ~111 km:
        # %.6g renders 51.477928 as "51.4779" and throws away 3 metres of
        # position, which is precisely the class of bug being hunted. %.10g
        # still collapses float noise (0.1+0.2 prints as 0.3) and still keeps
        # 1e-09 short.
        return f"{value:.10g}"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, (bytes, bytearray)):
        return f"<{len(value)} bytes>"
    if isinstance(value, str):
        return value if len(value) <= 28 else value[:27] + "…"
    return str(value)


def _is_sequence(value: Any) -> bool:
    if isinstance(value, (str, bytes, bytearray)):
        return False
    return isinstance(value, (list, tuple)) or (
        hasattr(value, "__len__") and hasattr(value, "__getitem__")
    )


def _format_sequence(value: Sequence) -> str:
    try:
        length = len(value)
    except Exception:
        return "<sequence>"
    if length == 0:
        return "[]"
    shown = [_format_scalar(v) for v in list(value[:MAX_SEQUENCE])]
    tail = f" +{length - MAX_SEQUENCE}" if length > MAX_SEQUENCE else ""
    return "[" + " ".join(shown) + tail + "]"


def _fields_of(msg: Any):
    """
    The field names of a ROS message, in declaration order.

    rosidl generates get_fields_and_field_types(); __slots__ is the fallback for
    anything hand-rolled, and it carries a leading underscore per field that has
    to be stripped before getattr.
    """
    getter = getattr(msg, "get_fields_and_field_types", None)
    if callable(getter):
        try:
            return list(getter().keys())
        except Exception:
            pass
    slots = getattr(msg, "__slots__", None)
    if slots:
        return [name[1:] if name.startswith("_") else name for name in slots]
    return []


def explode(msg: Any, prefix: str = "", depth: int = 0) -> List[Tuple[str, str]]:
    """
    Flatten a ROS message into ordered (dotted.path, formatted value) pairs.

    Nested messages recurse; sequences are summarised rather than expanded, so a
    36-element covariance or a 2 MB image buffer costs one row instead of
    swamping the screen.
    """
    out: List[Tuple[str, str]] = []
    if msg is None:
        return out
    try:
        names = _fields_of(msg)
        if not names:
            return [(prefix.rstrip(".") or "value", _format_scalar(msg))]
        for name in names:
            if len(out) >= MAX_FIELDS:
                out.append(("…", "truncated"))
                break
            try:
                value = getattr(msg, name)
            except Exception:
                continue
            path = f"{prefix}{name}"
            if _fields_of(value) and depth < MAX_DEPTH:
                out.extend(explode(value, f"{path}.", depth + 1))
            elif _is_sequence(value):
                out.append((path, _format_sequence(value)))
            else:
                out.append((path, _format_scalar(value)))
    except Exception:
        out.append(("<error>", "could not read message"))
    return out


def summarise_type(msg: Any) -> str:
    if msg is None:
        return "-"
    return type(msg).__name__


# ── pairing the two sides ────────────────────────────────────────────────────

# Field renames this bridge performs. Left is what PX4 (or the source side)
# calls it, right is the PSDK / ROS-standard name it lands on. Only the leaf
# name is compared, so "position.x" matches "x".
ALIASES = {
    "lat": ("latitude",),
    "lon": ("longitude",),
    "alt": ("altitude",),
    "eph": ("position_covariance",),
    "epv": ("position_covariance",),
    "timestamp": ("stamp", "sec", "nanosec"),
    "voltage_v": ("voltage",),
    "current_a": ("current",),
    "remaining": ("percentage", "capacity_percentage"),
    "discharged_mah": ("charge",),
    "capacity": ("capacity_percentage",),
    "satellites_used": ("num_total_satellites_used",),
    "hdop": ("horizontal_dop",),
    "vdop": ("vertical_dop",),
    "fix_type": ("fix_state", "status"),
    "heading": ("yaw",),
    "landed": ("flight_status",),
    "arming_state": ("flight_status", "display_mode"),
    "nav_state": ("display_mode",),
    "dist_bottom": ("down",),
    "xy_valid": ("status",),
    # Gimbal axis mapping. A Vector3Stamped on the wrapper side carries
    # (x=roll, y=pitch, z=yaw); the simulated gimbal's three joints are named
    # pan/roll/tilt. Without these the comparison shows six unpaired rows and
    # the reader cannot see that y drives tilt and z drives pan.
    "x": ("roll",),
    "y": ("pitch", "tilt"),
    "z": ("yaw", "pan"),
}

# Axis-frame renames: PX4 speaks NED/FRD, the PSDK surface and ROS speak
# ENU/FLU. Pairing x->x is right, but the row deserves to be marked as a frame
# change rather than looking like an unexplained sign flip.
FRAME_PAIRS = {("x", "y"), ("y", "x"), ("z", "z")}


def _leaf(path: str) -> str:
    return path.rsplit(".", 1)[-1]


def _candidates(name: str) -> Tuple[str, ...]:
    return (name,) + ALIASES.get(name, ())


def pair(
    source: Sequence[Tuple[str, str]],
    sink: Sequence[Tuple[str, str]],
) -> List[Row]:
    """
    Line the source fields up against the sink fields.

    Matching is by leaf name, then by the ALIASES table. Unmatched fields are
    still shown -- a field that appears on only one side is exactly the kind of
    thing worth seeing (it means the converter invented it, or dropped it).
    """
    rows: List[Row] = []
    remaining = list(sink)
    # Two different notions of "taken". `consumed` blocks a second exact-name
    # match, because two identically named fields really are two fields.
    # `paired` only keeps a field out of the leftovers list. They differ for
    # alias matches: PX4's eph AND epv both land in NavSatFix's single
    # position_covariance, and showing epv as "read but not published" would be
    # a lie. So an alias match pairs without consuming.
    consumed = [False] * len(remaining)
    paired = [False] * len(remaining)

    def take(name: str) -> Tuple[Optional[int], bool]:
        for index, (path, _value) in enumerate(remaining):
            if not consumed[index] and _leaf(path) == name:
                return index, True
        for wanted in ALIASES.get(name, ()):
            for index, (path, _value) in enumerate(remaining):
                if _leaf(path) == wanted:
                    return index, False
        return None, False

    for path, value in source:
        index, exact = take(_leaf(path))
        if index is None:
            rows.append(((path, value), None, "only-in"))
            continue
        if exact:
            consumed[index] = True
        paired[index] = True
        other = remaining[index]
        kind = "same" if other[1] == value else "changed"
        rows.append(((path, value), other, kind))

    for index, (path, value) in enumerate(remaining):
        if not paired[index]:
            rows.append((None, (path, value), "only-out"))

    return rows


def relation(row: Row) -> str:
    """The gutter glyph between the two columns."""
    left, right, kind = row
    if kind == "same":
        return "="
    if kind == "changed":
        return "→"
    if kind == "only-in":
        return "✗"
    return "+"


def describe_kind(kind: str) -> str:
    return {
        "same": "carried through unchanged",
        "changed": "value rewritten by the converter",
        "only-in": "read from the source but not published",
        "only-out": "produced by the converter; no direct source field",
    }.get(kind, kind)


# ── converter documentation ──────────────────────────────────────────────────


def converter_doc(converter: Any) -> List[str]:
    """
    The converter's own docstring, wrapped into display lines.

    These docstrings are where the intent lives -- units, frames, why a value is
    substituted -- so the inspection view shows them verbatim rather than
    paraphrasing.
    """
    doc = (getattr(converter, "__doc__", "") or "").strip()
    if not doc:
        return ["(this converter has no docstring)"]
    lines: List[str] = []
    for raw in doc.splitlines():
        lines.append(raw.strip())
    while lines and not lines[-1]:
        lines.pop()
    return lines


def converter_name(converter: Any) -> str:
    return getattr(converter, "__name__", type(converter).__name__)
