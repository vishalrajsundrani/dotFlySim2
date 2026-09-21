#!/usr/bin/env python3
"""
gen_powerline.py — build the powerline models for worlds/powerline.sdf.

Generates a 220 kV double-circuit lattice tower and the conductor span that
hangs between two of them, as OBJ meshes plus the SDF models that wrap them,
from real line dimensions rather than from a downloaded mesh.

    python3 tools/gen_powerline.py                 # defaults, straight into models/
    python3 tools/gen_powerline.py --height 45 --span 300 --sag 7.0
    python3 tools/gen_powerline.py --list          # print the geometry and exit

WHY A GENERATOR AND NOT A MESH
------------------------------
The mesh this replaces (models/transmission_tower) is one COLLADA file holding
TWO towers and the wires between them, baked at a fixed 14.4 m tower height and
a 63 m span — half the height of a real transmission tower and a fifth of a real
span. It cannot be rescaled from the world file either: `<scale>` is not a child
of `<include>` in SDF, so the `<scale>3 3 3</scale>` that used to sit there was
silently dropped by libsdformat. And because both towers live in one mesh, there
is no way to place five of them along a line.

Everything here is computed from the numbers at the top of Config instead, so
"proper scale" is a parameter rather than a modelling job: change --span and the
catenary is re-solved, change --height and the arms, insulators and earth wire
peaks move with it.

WHAT IT WRITES
--------------
    models/hv_tower_220kv/
        meshes/tower_steel.obj      the lattice: legs, bracing, arms, peaks
        meshes/insulators.obj       6 suspension strings of 14 porcelain discs
        meshes/foundations.obj      4 concrete stubs
        model.sdf  model.config     visuals + PRIMITIVE collision boxes
    models/hv_span_220kv/
        meshes/conductors.obj       6 phase conductors, true catenary sag
        meshes/earthwires.obj       2 shield wires, sagged less, as in reality
        meshes/dampers.obj          Stockbridge dampers near every attachment
        model.sdf  model.config     visuals + inflated cylinder collisions

One tower mesh is reused by all five includes and one span mesh by all four
spans, so the whole line is 2 meshes on disk and ~9 draw calls, not 5 copies of
a 2.5 MB COLLADA file.

MATERIALS LIVE IN THE SDF, NOT IN THE MESH
------------------------------------------
Each mesh holds one material's worth of geometry — steel, porcelain, concrete,
aluminium conductor, galvanised earth wire — so the colours are `<material>`
tags you can edit in model.sdf without regenerating anything, and no .mtl file
has to resolve at load time.
"""

import argparse
import math
import os
import re

# ─────────────────────────────────────────────────────────────────────────────
# Geometry — a 220 kV double-circuit "Danube" suspension tower
# ─────────────────────────────────────────────────────────────────────────────
# Defaults are ordinary numbers for this class of line: 36 m to the earth wire
# peak, a 7.2 m square base narrowing to a 2.6 m waist, three cross-arms a side
# at ~5 m vertical phase spacing, 2 m insulator strings, 200 m spans with 4.5 m
# of sag. A real 220 kV tower is 35-45 m and real spans are 200-400 m; the mesh
# this replaces was 14.4 m and 63 m.


class Config:
    def __init__(self, args):
        self.height = args.height              # to the earth wire peak, m
        self.base = args.base                  # leg spread at ground, m square
        self.waist = args.waist                # body width above the waist, m
        self.waist_h = args.waist * 0 + args.waist_height
        self.span = args.span                  # tower to tower, m
        self.sag = args.sag                    # conductor sag at mid-span, m
        # A shield wire is strung tighter than the phases it protects, so it
        # sags less — that is what keeps its shielding angle over the whole
        # span instead of only at the towers.
        self.earth_sag = args.sag * args.earth_sag_ratio

        # Cross-arms, as fractions of tower height. Middle arm longest: that is
        # what makes a Danube tower a Danube tower.
        h = self.height
        self.arms = [                          # (height, half-length)
            (0.556 * h, 5.6),                  # lower  ~20.0 m
            (0.708 * h, 6.4),                  # middle ~25.5 m
            (0.847 * h, 5.2),                  # upper  ~30.5 m
        ]
        self.body_top = 0.889 * h              # ~32.0 m, where the peak starts
        self.peak_arm_h = 0.931 * h            # ~33.5 m, the earth wire crossarm
        self.earth_y = 2.2                     # earth wire offset from centre
        self.insulator_len = args.insulator    # arm tip to conductor, m

        self.conductor_r = args.conductor / 2000.0    # mm diameter -> m radius
        self.earthwire_r = args.earthwire / 2000.0

        # Collision radius is deliberately NOT the visual radius: see model.sdf.
        self.wire_collision_r = args.wire_collision

    def attach_points(self):
        """(y, z) of every conductor attachment on one tower, both circuits."""
        points = []
        for arm_h, arm_len in self.arms:
            for side in (-1.0, 1.0):
                points.append((side * arm_len, arm_h - self.insulator_len))
        return points

    def earth_points(self):
        return [(-self.earth_y, self.height), (self.earth_y, self.height)]

    def half_width(self, z):
        """Body half-width at height z — the taper that gives the silhouette."""
        if z >= self.waist_h:
            return self.waist / 2.0
        t = z / self.waist_h
        return (self.base / 2.0) * (1.0 - t) + (self.waist / 2.0) * t


# ─────────────────────────────────────────────────────────────────────────────
# Mesh building
# ─────────────────────────────────────────────────────────────────────────────
# A tiny OBJ writer. Vertices, normals and triangles, nothing else: no UVs (the
# materials are flat colours from the SDF) and no material library (one mesh per
# material, so there is nothing to reference).


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    length = math.sqrt(_dot(a, a))
    if length < 1e-12:
        return (0.0, 0.0, 1.0)
    return (a[0] / length, a[1] / length, a[2] / length)


class Mesh:
    def __init__(self, name):
        self.name = name
        self.v = []
        self.vn = []
        self.f = []            # (vi, ni) triples, 1-based, already offset

    # ── primitives ──────────────────────────────────────────────────────────
    def quad(self, p0, p1, p2, p3, normal=None):
        if normal is None:
            normal = _norm(_cross(_sub(p1, p0), _sub(p3, p0)))
        base = len(self.v) + 1
        self.v += [p0, p1, p2, p3]
        self.vn.append(normal)
        n = len(self.vn)
        self.f.append(((base, n), (base + 1, n), (base + 2, n)))
        self.f.append(((base, n), (base + 2, n), (base + 3, n)))

    def beam(self, p0, p1, width, depth=None):
        """
        One structural member: a rectangular bar from p0 to p1.

        Real towers are built from steel angle sections; at any distance a bar
        of the right thickness reads exactly the same, and costs 12 triangles.
        """
        depth = width if depth is None else depth
        axis = _norm(_sub(p1, p0))
        ref = (0.0, 0.0, 1.0)
        if abs(_dot(axis, ref)) > 0.95:
            ref = (1.0, 0.0, 0.0)
        u = _norm(_cross(ref, axis))
        w = _norm(_cross(axis, u))
        hu, hw = _mul(u, width / 2.0), _mul(w, depth / 2.0)

        corners = []
        for end in (p0, p1):
            corners.append([
                _add(_add(end, _mul(hu, -1)), _mul(hw, -1)),
                _add(_add(end, hu), _mul(hw, -1)),
                _add(_add(end, hu), hw),
                _add(_add(end, _mul(hu, -1)), hw),
            ])
        a, b = corners
        self.quad(a[0], a[1], a[2], a[3], _mul(axis, -1))          # cap
        self.quad(b[3], b[2], b[1], b[0], axis)                    # cap
        for i in range(4):
            j = (i + 1) % 4
            self.quad(a[i], b[i], b[j], a[j])

    def box(self, centre, size):
        cx, cy, cz = centre
        sx, sy, sz = (s / 2.0 for s in size)
        p = [(cx - sx, cy - sy, cz - sz), (cx + sx, cy - sy, cz - sz),
             (cx + sx, cy + sy, cz - sz), (cx - sx, cy + sy, cz - sz),
             (cx - sx, cy - sy, cz + sz), (cx + sx, cy - sy, cz + sz),
             (cx + sx, cy + sy, cz + sz), (cx - sx, cy + sy, cz + sz)]
        self.quad(p[3], p[2], p[1], p[0], (0, 0, -1))
        self.quad(p[4], p[5], p[6], p[7], (0, 0, 1))
        self.quad(p[0], p[1], p[5], p[4], (0, -1, 0))
        self.quad(p[2], p[3], p[7], p[6], (0, 1, 0))
        self.quad(p[1], p[2], p[6], p[5], (1, 0, 0))
        self.quad(p[3], p[0], p[4], p[7], (-1, 0, 0))

    def cylinder(self, p0, p1, radius, sides=10, cap=True):
        self.tube([p0, p1], radius, sides=sides, cap=cap)

    def tube(self, points, radius, sides=8, cap=True):
        """
        A round tube swept along a polyline — every wire in this world.

        The cross-section frame is parallel-transported from ring to ring
        rather than rebuilt from a fixed reference, so a wire that changes
        direction along its length (which a catenary does, continuously) has no
        twist and no seam.
        """
        if len(points) < 2:
            return
        axis = _norm(_sub(points[1], points[0]))
        ref = (0.0, 0.0, 1.0)
        if abs(_dot(axis, ref)) > 0.95:
            ref = (1.0, 0.0, 0.0)
        u = _norm(_cross(ref, axis))

        rings = []
        for i, p in enumerate(points):
            if i == 0:
                a = _norm(_sub(points[1], points[0]))
            elif i == len(points) - 1:
                a = _norm(_sub(points[-1], points[-2]))
            else:
                a = _norm(_sub(points[i + 1], points[i - 1]))
            # Re-orthogonalise the carried frame against the new axis.
            u = _norm(_sub(u, _mul(a, _dot(u, a))))
            w = _norm(_cross(a, u))
            ring = []
            for s in range(sides):
                ang = 2.0 * math.pi * s / sides
                direction = _add(_mul(u, math.cos(ang)), _mul(w, math.sin(ang)))
                ring.append((_add(p, _mul(direction, radius)), direction))
            rings.append(ring)

        for i in range(len(rings) - 1):
            r0, r1 = rings[i], rings[i + 1]
            for s in range(sides):
                t = (s + 1) % sides
                base = len(self.v) + 1
                self.v += [r0[s][0], r1[s][0], r1[t][0], r0[t][0]]
                nb = len(self.vn) + 1
                self.vn += [r0[s][1], r1[s][1], r1[t][1], r0[t][1]]
                self.f.append(((base, nb), (base + 1, nb + 1), (base + 2, nb + 2)))
                self.f.append(((base, nb), (base + 2, nb + 2), (base + 3, nb + 3)))

        if cap:
            for ring, p, sign in ((rings[0], points[0], -1.0),
                                  (rings[-1], points[-1], 1.0)):
                a = _mul(_norm(_sub(points[1], points[0])) if sign < 0
                         else _norm(_sub(points[-1], points[-2])), sign)
                base = len(self.v) + 1
                self.v.append(p)
                self.vn.append(a)
                n = len(self.vn)
                centre = base
                for s in range(sides):
                    t = (s + 1) % sides
                    self.v += [ring[s][0], ring[t][0]]
                    i0 = len(self.v) - 1
                    self.f.append(((centre, n), (i0, n), (i0 + 1, n)))

    # ── output ──────────────────────────────────────────────────────────────
    def write(self, path, header=""):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as out:
            out.write(f"# {self.name}\n")
            for line in header.strip().splitlines():
                out.write(f"# {line.strip()}\n")
            out.write(f"# {len(self.v)} vertices, {len(self.f)} triangles\n")
            out.write("# generated by tools/gen_powerline.py -- do not hand-edit\n")
            out.write(f"o {self.name}\n")
            for x, y, z in self.v:
                out.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
            for x, y, z in self.vn:
                out.write(f"vn {x:.4f} {y:.4f} {z:.4f}\n")
            for tri in self.f:
                out.write("f " + " ".join(f"{vi}//{ni}" for vi, ni in tri) + "\n")
        return path


# ─────────────────────────────────────────────────────────────────────────────
# The catenary
# ─────────────────────────────────────────────────────────────────────────────
# A hanging wire is a catenary, not a parabola, and at inspection distances the
# difference is visible: the parabola is too full near the towers. y = a*cosh
# (x/a) with a solved from the sag the operator asked for.


def catenary_a(span, sag):
    """Solve a in sag = a*(cosh(span/(2a)) - 1) by bisection."""
    if sag <= 1e-6:
        return None
    lo, hi = 1e-3, 1e7
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        try:
            value = mid * (math.cosh(span / (2.0 * mid)) - 1.0)
        except OverflowError:
            value = float("inf")
        if value > sag:                 # too much sag -> stiffer wire
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def catenary_points(start, end, sag, segments=48):
    """
    Sample the wire hanging between two attachment points.

    The two ends are at the same height here (level spans), so the low point is
    the middle; the code carries the linear interpolation anyway so an uneven
    span keeps working if someone changes a tower's height.
    """
    span = math.dist(start[:2], end[:2])
    a = catenary_a(span, sag)
    points = []
    for i in range(segments + 1):
        t = i / segments
        x = start[0] + (end[0] - start[0]) * t
        y = start[1] + (end[1] - start[1]) * t
        z = start[2] + (end[2] - start[2]) * t
        if a is not None:
            s = (t - 0.5) * span
            z -= a * (math.cosh(span / (2.0 * a)) - math.cosh(s / a))
        points.append((x, y, z))
    return points


# ─────────────────────────────────────────────────────────────────────────────
# The tower
# ─────────────────────────────────────────────────────────────────────────────


def build_tower(cfg):
    steel = Mesh("tower_steel")
    porcelain = Mesh("insulators")
    concrete = Mesh("foundations")

    leg = 0.22          # main leg chord, m
    brace = 0.11        # bracing member
    small = 0.07        # redundant/secondary member

    # Panel joints: close together low down where the legs are still splayed
    # and the wind load is highest, wider up the body. These are the heights
    # where a real tower has a horizontal belt and the bracing changes over.
    joints = [0.0, 2.4, 5.2, 8.4, 12.0, 15.6, cfg.waist_h]
    z = cfg.waist_h
    while z < cfg.body_top - 0.1:
        joints.append(min(z + 3.2, cfg.body_top))
        z += 3.2
    joints = sorted(set(round(j, 3) for j in joints))

    def corner(z, sx, sy):
        h = cfg.half_width(z)
        return (sx * h, sy * h, z)

    signs = [(-1, -1), (1, -1), (1, 1), (-1, 1)]

    # ── legs, belts and bracing ─────────────────────────────────────────────
    for i in range(len(joints) - 1):
        z0, z1 = joints[i], joints[i + 1]
        for sx, sy in signs:
            steel.beam(corner(z0, sx, sy), corner(z1, sx, sy), leg)

        # Horizontal belt at the top of the panel, on all four faces.
        for k in range(4):
            a, b = signs[k], signs[(k + 1) % 4]
            steel.beam(corner(z1, *a), corner(z1, *b), brace)

        # X bracing on each face, plus the short redundant members that make a
        # lattice tower read as a lattice rather than as a wireframe box.
        for k in range(4):
            a, b = signs[k], signs[(k + 1) % 4]
            p00, p01 = corner(z0, *a), corner(z0, *b)
            p10, p11 = corner(z1, *a), corner(z1, *b)
            steel.beam(p00, p11, brace)
            steel.beam(p01, p10, brace)
            mid = _mul(_add(p00, p11), 0.5)
            steel.beam(_mul(_add(p00, p01), 0.5), mid, small)
            steel.beam(_mul(_add(p10, p11), 0.5), mid, small)

    # ── cross-arms ──────────────────────────────────────────────────────────
    # Each arm is a cantilever truss: two bottom chords out to the tip, a top
    # chord sloping down to meet them, and webs between. Built twice per level,
    # once each side, extending along Y — the line itself runs along X, so the
    # arms stick out sideways from it, which is what puts the six conductors in
    # two vertical circuits.
    for arm_h, arm_len in cfg.arms:
        hw = cfg.half_width(arm_h)
        top_h = arm_h + 2.6
        for side in (-1.0, 1.0):
            tip = (0.0, side * arm_len, arm_h)
            tip_out = (0.0, side * (arm_len + 0.35), arm_h)
            bottoms = []
            for sx in (-1.0, 1.0):
                root = (sx * hw, side * hw, arm_h)
                steel.beam(root, tip, 0.16)
                bottoms.append(root)
            steel.beam(bottoms[0], bottoms[1], brace)
            steel.beam(tip, tip_out, 0.16)

            # Top chord from higher up the body down to the tip, and its webs.
            apex = (0.0, side * hw, top_h)
            steel.beam((-hw, side * hw, top_h), apex, brace)
            steel.beam((hw, side * hw, top_h), apex, brace)
            steel.beam(apex, tip, 0.14)
            for t in (0.3, 0.55, 0.8):
                on_top = _add(apex, _mul(_sub(tip, apex), t))
                on_bottom = (0.0, side * (hw + (arm_len - hw) * t), arm_h)
                steel.beam(on_bottom, on_top, small)
                for sx in (-1.0, 1.0):
                    chord = (sx * hw * (1.0 - t), side * (hw + (arm_len - hw) * t), arm_h)
                    steel.beam(chord, on_top, small)

    # ── the peak and the earth wire crossarm ────────────────────────────────
    top_hw = cfg.half_width(cfg.body_top)
    peak_hw = 0.45
    for sx, sy in signs:
        steel.beam((sx * top_hw, sy * top_hw, cfg.body_top),
                   (sx * peak_hw, sy * peak_hw, cfg.peak_arm_h), leg * 0.8)
    for k in range(4):
        a, b = signs[k], signs[(k + 1) % 4]
        steel.beam((a[0] * peak_hw, a[1] * peak_hw, cfg.peak_arm_h),
                   (b[0] * peak_hw, b[1] * peak_hw, cfg.peak_arm_h), brace)

    for y, z in cfg.earth_points():
        side = 1.0 if y > 0 else -1.0
        for sx in (-1.0, 1.0):
            steel.beam((sx * peak_hw, side * peak_hw, cfg.peak_arm_h),
                       (0.0, y, z), 0.13)
        steel.beam((0.0, y * 0.5, cfg.peak_arm_h), (0.0, y, z), small)

    # ── insulator strings ───────────────────────────────────────────────────
    # 14 cap-and-pin discs on a suspension string, which is what 220 kV wears.
    # Drawn as discs on a short steel link so the string reads as a string and
    # not as a rod: the alternating profile is most of what you recognise.
    discs = 14
    pitch = (cfg.insulator_len - 0.5) / discs
    for y, z_attach in cfg.attach_points():
        arm_h = z_attach + cfg.insulator_len
        steel.beam((0.0, y, arm_h), (0.0, y, arm_h - 0.25), 0.10)
        for d in range(discs):
            z = arm_h - 0.3 - d * pitch
            porcelain.cylinder((0.0, y, z), (0.0, y, z - pitch * 0.45), 0.14, sides=10)
            steel.beam((0.0, y, z - pitch * 0.45), (0.0, y, z - pitch), 0.045)
        steel.beam((0.0, y, z_attach + 0.22), (0.0, y, z_attach - 0.05), 0.09)

    # ── foundations ─────────────────────────────────────────────────────────
    for sx, sy in signs:
        x, y, _ = corner(0.0, sx, sy)
        concrete.box((x, y, 0.35), (1.25, 1.25, 1.6))

    return steel, porcelain, concrete


# ─────────────────────────────────────────────────────────────────────────────
# The span
# ─────────────────────────────────────────────────────────────────────────────


def build_span(cfg, segments):
    conductors = Mesh("conductors")
    earthwires = Mesh("earthwires")
    dampers = Mesh("dampers")

    for y, z in cfg.attach_points():
        points = catenary_points((0.0, y, z), (cfg.span, y, z), cfg.sag, segments)
        conductors.tube(points, cfg.conductor_r, sides=8)

        # Stockbridge dampers, a few metres in from each end, where a real line
        # puts them: two bell weights on a short messenger clamped to the wire.
        for base_x in (3.0, cfg.span - 3.0):
            t = base_x / cfg.span
            zz = catenary_points((0.0, y, z), (cfg.span, y, z), cfg.sag,
                                 segments)[min(int(t * segments), segments)][2]
            dampers.cylinder((base_x - 0.35, y, zz - 0.05),
                             (base_x + 0.35, y, zz - 0.05), 0.025, sides=6)
            for dx in (-0.38, 0.38):
                dampers.cylinder((base_x + dx - 0.09, y, zz - 0.05),
                                 (base_x + dx + 0.09, y, zz - 0.05), 0.055, sides=8)
            dampers.cylinder((base_x, y, zz), (base_x, y, zz - 0.05), 0.03, sides=6)

    for y, z in cfg.earth_points():
        points = catenary_points((0.0, y, z), (cfg.span, y, z), cfg.earth_sag, segments)
        earthwires.tube(points, cfg.earthwire_r, sides=6)

    return conductors, earthwires, dampers


# ─────────────────────────────────────────────────────────────────────────────
# SDF
# ─────────────────────────────────────────────────────────────────────────────
# Colours are the interesting part of these files and the reason the meshes
# carry no materials of their own: every one of them is a line you can edit and
# reload without running this script again.

MATERIALS = {
    # Hot-dip galvanised steel, weathered to matt light grey. Deliberately
    # unsaturated: demo_camera_track's default "structure" colour window finds
    # a target by low saturation against a saturated sky and ground.
    "steel":     ("0.30 0.31 0.33 1", "0.62 0.64 0.66 1", "0.25 0.25 0.26 1"),
    # ACSR conductor: aluminium strands that oxidise to a dull mid-grey within
    # months. Shiny silver is what a conductor looks like on the drum only.
    "conductor": ("0.16 0.16 0.17 1", "0.34 0.35 0.36 1", "0.12 0.12 0.12 1"),
    # Galvanised steel earth wire, brighter than the phases it shields.
    "earthwire": ("0.26 0.27 0.28 1", "0.56 0.57 0.59 1", "0.30 0.30 0.32 1"),
    # Brown-glazed porcelain discs.
    "porcelain": ("0.20 0.15 0.11 1", "0.46 0.34 0.25 1", "0.35 0.30 0.25 1"),
    "concrete":  ("0.28 0.27 0.25 1", "0.60 0.58 0.54 1", "0.05 0.05 0.05 1"),
    "damper":    ("0.18 0.18 0.19 1", "0.38 0.38 0.40 1", "0.20 0.20 0.20 1"),
}


def xml_comment(text):
    """
    Wrap prose as an XML comment.

    XML forbids "--" ANYWHERE inside a comment, which is a trap when the
    comment is prose: a perfectly ordinary dashed aside makes libsdformat
    refuse the whole model with "not well-formed (invalid token)" and a line
    number pointing at the geometry rather than at the sentence. Em dashes are
    what the rest of this repo's prose uses anyway.
    """
    body = re.sub(r"-{2,}", "\u2014", text)
    return "<!--" + body + "-->"


def visual(name, mesh_uri, material, cast_shadows=True):
    ambient, diffuse, specular = MATERIALS[material]
    return f"""      <visual name="{name}">
        <cast_shadows>{'true' if cast_shadows else 'false'}</cast_shadows>
        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{diffuse}</diffuse>
          <specular>{specular}</specular>
        </material>
      </visual>
"""


def model_config(name, description):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>dotFlySim</name></author>
  <description>{description}</description>
</model>
"""


def tower_sdf(cfg):
    collisions = []

    # COLLISION IS PRIMITIVES, NOT THE MESH — the same rule the model this
    # replaces learned the hard way. A COLLADA/OBJ trimesh is either dropped
    # silently by DART or collapsed to a convex hull, and the drone then flies
    # through the tower it is supposed to be inspecting.
    steps = 6
    for i in range(steps):
        z0 = cfg.body_top * i / steps
        z1 = cfg.body_top * (i + 1) / steps
        w = cfg.half_width(0.5 * (z0 + z1)) * 2.0
        collisions.append(
            f'      <collision name="body_{i}"><pose>0 0 {0.5 * (z0 + z1):.2f} 0 0 0</pose>\n'
            f'        <geometry><box><size>{w:.2f} {w:.2f} {z1 - z0:.2f}</size></box></geometry></collision>')

    for idx, (arm_h, arm_len) in enumerate(cfg.arms):
        collisions.append(
            f'      <collision name="arm_{idx}"><pose>0 0 {arm_h + 1.0:.2f} 0 0 0</pose>\n'
            f'        <geometry><box><size>1.6 {2 * arm_len + 0.7:.2f} 3.4</size></box></geometry></collision>')

    collisions.append(
        f'      <collision name="peak"><pose>0 0 {0.5 * (cfg.body_top + cfg.height):.2f} 0 0 0</pose>\n'
        f'        <geometry><box><size>1.2 {2 * cfg.earth_y + 0.4:.2f} {cfg.height - cfg.body_top:.2f}</size></box></geometry></collision>')

    # The insulator strings hang in the flight path, so they are solid too.
    for idx, (y, z) in enumerate(cfg.attach_points()):
        collisions.append(
            f'      <collision name="insulator_{idx}"><pose>0 {y:.2f} {z + cfg.insulator_len / 2:.2f} 0 0 0</pose>\n'
            f'        <geometry><cylinder><radius>0.22</radius><length>{cfg.insulator_len:.2f}</length></cylinder></geometry></collision>')

    visuals = (visual("steel", "meshes/tower_steel.obj", "steel")
               + visual("insulators", "meshes/insulators.obj", "porcelain")
               + visual("foundations", "meshes/foundations.obj", "concrete"))

    comment = xml_comment(f""" 220 kV double-circuit suspension tower, {cfg.height:.0f} m to the earth wire peak.

       GENERATED by tools/gen_powerline.py -- rerun it rather than editing the
       geometry here. The <material> blocks are the exception: they are meant to
       be edited, and nothing regenerates from them.

       Origin is at the centre of the base, +Z up, and the line runs along +X --
       so the cross-arms extend along +/-Y and the six conductors leave this
       model in two vertical circuits of three.

       Conductor attachment points (y, z), for anything that needs to fly to one:
{chr(10).join(f'         {y:+7.2f}  {z:6.2f} m' for y, z in cfg.attach_points())}
       Earth wires:
{chr(10).join(f'         {y:+7.2f}  {z:6.2f} m' for y, z in cfg.earth_points())}
  """)
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  {comment}
  <model name="hv_tower_220kv">
    <static>true</static>
    <link name="link">
{visuals}
{chr(10).join(collisions)}
    </link>
  </model>
</sdf>
"""


def span_sdf(cfg, segments):
    collisions = []

    # WHY THE COLLISION WIRE IS FATTER THAN THE VISIBLE ONE
    # A {cfg.conductor_r * 2000:.0f} mm conductor is about the width of a finger. At 250 Hz
    # physics and a few m/s of closing speed the drone moves ~2 cm a step, so a
    # 14 mm cylinder is a target it can pass clean through between two steps
    # while the camera watched it happen. The collision radius is inflated to
    # {cfg.wire_collision_r * 100:.0f} cm -- a compromise: still thin enough to fly between two
    # phases, thick enough that flying INTO one is an event.
    pieces = 6
    for idx, (y, z) in enumerate(cfg.attach_points()):
        points = catenary_points((0.0, y, z), (cfg.span, y, z), cfg.sag, pieces)
        for k in range(pieces):
            p0, p1 = points[k], points[k + 1]
            mid = _mul(_add(p0, p1), 0.5)
            length = math.dist(p0, p1)
            pitch = math.atan2(p1[2] - p0[2], p1[0] - p0[0])
            collisions.append(
                f'      <collision name="c{idx}_{k}">'
                f'<pose>{mid[0]:.2f} {mid[1]:.2f} {mid[2]:.2f} 0 {math.pi / 2 - pitch:.4f} 0</pose>\n'
                f'        <geometry><cylinder><radius>{cfg.wire_collision_r}</radius>'
                f'<length>{length:.2f}</length></cylinder></geometry></collision>')

    for idx, (y, z) in enumerate(cfg.earth_points()):
        points = catenary_points((0.0, y, z), (cfg.span, y, z), cfg.earth_sag, pieces)
        for k in range(pieces):
            p0, p1 = points[k], points[k + 1]
            mid = _mul(_add(p0, p1), 0.5)
            length = math.dist(p0, p1)
            pitch = math.atan2(p1[2] - p0[2], p1[0] - p0[0])
            collisions.append(
                f'      <collision name="e{idx}_{k}">'
                f'<pose>{mid[0]:.2f} {mid[1]:.2f} {mid[2]:.2f} 0 {math.pi / 2 - pitch:.4f} 0</pose>\n'
                f'        <geometry><cylinder><radius>{cfg.wire_collision_r}</radius>'
                f'<length>{length:.2f}</length></cylinder></geometry></collision>')

    visuals = (visual("conductors", "meshes/conductors.obj", "conductor", cast_shadows=False)
               + visual("earthwires", "meshes/earthwires.obj", "earthwire", cast_shadows=False)
               + visual("dampers", "meshes/dampers.obj", "damper", cast_shadows=False))

    comment = xml_comment(f""" One {cfg.span:.0f} m span of 220 kV double-circuit line: 6 phase conductors and
       2 earth wires, each a real catenary ({cfg.sag:.1f} m of sag on the phases,
       {cfg.earth_sag:.1f} m on the earth wires, which are strung tighter).

       GENERATED by tools/gen_powerline.py. Place it at the position of the
       tower the span STARTS at: the mesh runs from x=0 to x={cfg.span:.0f}, at the same
       attachment heights the tower model hangs its insulators at, so
       tower_k and span_k share a pose.

       Conductor diameter is {cfg.conductor_r * 2000:.0f} mm and the earth wire {cfg.earthwire_r * 2000:.0f} mm -- real ACSR
       and OPGW sizes for this class. At 5 m the conductor is ~2 px wide in the
       m4e's 640x360 preview tier, which is the point: if it were drawn fatter,
       an inspection algorithm tuned in here would not survive contact with a
       real frame.

       Shadows are off on the wires. A {cfg.conductor_r * 2000:.0f} mm tube casting a shadow map
       costs a full extra pass to produce a line thinner than one shadow texel.
  """)
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  {comment}
  <model name="hv_span_220kv">
    <static>true</static>
    <link name="link">
{visuals}
{chr(10).join(collisions)}
    </link>
  </model>
</sdf>
"""


# ─────────────────────────────────────────────────────────────────────────────


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--height", type=float, default=36.0, help="tower height to the earth wire peak, m (default 36)")
    p.add_argument("--base", type=float, default=7.2, help="leg spread at ground level, m square (default 7.2)")
    p.add_argument("--waist", type=float, default=2.6, help="body width above the waist, m (default 2.6)")
    p.add_argument("--waist-height", type=float, default=18.0, help="height at which the taper ends, m (default 18)")
    p.add_argument("--span", type=float, default=200.0, help="tower spacing, m (default 200)")
    p.add_argument("--sag", type=float, default=4.5, help="conductor sag at mid-span, m (default 4.5)")
    p.add_argument("--earth-sag-ratio", type=float, default=0.78, help="earth wire sag as a fraction of conductor sag")
    p.add_argument("--insulator", type=float, default=2.4, help="suspension string length, arm tip to conductor, m")
    p.add_argument("--conductor", type=float, default=28.6, help="conductor diameter, mm (default 28.6, ACSR Zebra)")
    p.add_argument("--earthwire", type=float, default=14.0, help="earth wire diameter, mm (default 14, OPGW)")
    p.add_argument("--wire-collision", type=float, default=0.08, help="collision radius for a wire, m (see model.sdf)")
    p.add_argument("--segments", type=int, default=48, help="catenary segments per wire per span (default 48)")
    p.add_argument("--models", default="models", help="output directory for the models (default models/)")
    p.add_argument("--list", action="store_true", help="print the resulting geometry and exit")
    args = p.parse_args()

    cfg = Config(args)

    if args.list:
        print(f"tower      {cfg.height:.1f} m tall, {cfg.base:.1f} m base -> {cfg.waist:.1f} m waist at {cfg.waist_h:.1f} m")
        for i, (h, l) in enumerate(cfg.arms):
            print(f"  arm {i}    {h:6.2f} m, half-length {l:.2f} m")
        print(f"  peak     body top {cfg.body_top:.2f} m, earth wires at +/-{cfg.earth_y:.1f} m, {cfg.height:.1f} m")
        print(f"span       {cfg.span:.0f} m, sag {cfg.sag:.2f} m (earth {cfg.earth_sag:.2f} m)")
        a = catenary_a(cfg.span, cfg.sag)
        print(f"  catenary a = {a:.1f} m")
        low = min(z for _, z in cfg.attach_points()) - cfg.sag
        print(f"  lowest conductor at mid-span: {low:.2f} m above ground")
        for y, z in cfg.attach_points():
            print(f"  conductor  y={y:+6.2f}  z={z:6.2f}")
        return

    tower_dir = os.path.join(args.models, "hv_tower_220kv")
    span_dir = os.path.join(args.models, "hv_span_220kv")

    steel, porcelain, concrete = build_tower(cfg)
    conductors, earthwires, dampers = build_span(cfg, args.segments)

    header = (f"generated for a {cfg.height:.0f} m 220 kV double-circuit tower, "
              f"{cfg.span:.0f} m span, {cfg.sag:.1f} m sag")
    written = [
        steel.write(os.path.join(tower_dir, "meshes", "tower_steel.obj"), header),
        porcelain.write(os.path.join(tower_dir, "meshes", "insulators.obj"), header),
        concrete.write(os.path.join(tower_dir, "meshes", "foundations.obj"), header),
        conductors.write(os.path.join(span_dir, "meshes", "conductors.obj"), header),
        earthwires.write(os.path.join(span_dir, "meshes", "earthwires.obj"), header),
        dampers.write(os.path.join(span_dir, "meshes", "dampers.obj"), header),
    ]

    for path, text in (
        (os.path.join(tower_dir, "model.sdf"), tower_sdf(cfg)),
        (os.path.join(tower_dir, "model.config"),
         model_config("hv_tower_220kv",
                      f"{cfg.height:.0f} m 220 kV double-circuit lattice suspension tower, "
                      "generated by tools/gen_powerline.py")),
        (os.path.join(span_dir, "model.sdf"), span_sdf(cfg, args.segments)),
        (os.path.join(span_dir, "model.config"),
         model_config("hv_span_220kv",
                      f"{cfg.span:.0f} m span of 6 conductors and 2 earth wires with "
                      f"{cfg.sag:.1f} m catenary sag, generated by tools/gen_powerline.py")),
    ):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as out:
            out.write(text)
        written.append(path)

    for path in written:
        size = os.path.getsize(path)
        print(f"  {path:60s} {size / 1024:8.1f} KiB")


if __name__ == "__main__":
    main()
