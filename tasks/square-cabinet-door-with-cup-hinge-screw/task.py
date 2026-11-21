"""Square a crooked wall-cabinet door by turning its lower cup hinge's side-adjustment screw with a PH2 screwdriver.

Two 400 mm tall frameless wall cabinets hang side by side on a wall section behind the bench: ours (450 mm wide) and its
left-hand neighbour (300 mm wide), each with a full-overlay 19 mm door. Ours hangs on two Blum CLIP top 110-degree hinges
(71T3550) on its right-hand side (the robot's right) and stands open 50 degrees. Shut, it is crooked: the gap to the
neighbour's door is 1.6 mm at 20 mm above the bottom and 2.8 mm at 20 mm below the top. The fix is to seat a PH2
screwdriver in the lower hinge arm's side-adjustment screw (inside the cabinet, on the side panel), push it in and turn it
clockwise about 0.5 to 1.1 turns, which draws the bottom of the door towards the hinge side, and shut the door.

Mechanism (mechkit):
- the door hangs on a swing hinge (shut to 110 degrees open; the hinge springs pull it shut over the last 25 degrees and it
  stays where it is put above that), carried by a sideways slide and a tilt about the door's normal through the lower
  hinge. Those two follow the two side screws: 1 mm of sideways movement at a hinge per turn, +-2 mm of travel to a stop.
- each side screw's PH2 recess (four flanks sloping into a 6 mm mouth, 3 mm deep) sits on a 20 N/mm spring along the
  screw's axis, so its give reads how hard the screwdriver is pushed in.
- the screw turns with the screwdriver only while the tip is in the recess (within 0.8 mm of its axis), the screwdriver
  within 6 degrees of the screw's axis, and pushed in with at least 10 N (the recess down 0.5 mm or more). Then the
  screwdriver's own turning drives the screw one for one, and the screw resists with 0.2 N m (3 N m at the end of its
  travel), applied to the screwdriver as a torque, so the grip carries the turning effort. Pushed too lightly, tilted or
  off the recess, the screwdriver turns without the screw (cam-out).

The scorer reads the gaps from where the door's free edge actually is, the door's angle and speed, and the grippers'
fingertips. The record of the screws' turns is cleared only in on_reset.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
import yam_arena.demo as _harness_demo  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("The right-hand cabinet door hangs crooked, so the gap between the two doors is uneven. "
               "Make the gap even, 2 to 3 mm wide from top to bottom, and leave the door shut.")
SUCCESS_HOLD_S = 1.0

# -- the cabinets (metres; x forward from the arm bases, y left, z above the bench) ----------------------------------------
XF, XB = 0.420, 0.720  # carcass front and back faces
YR, YL = -0.250, 0.200  # our carcass: right and left outer faces (450 mm wide)
YN = 0.500  # the neighbour's left outer face (300 mm wide)
ZB, ZT = 0.030, 0.430  # carcass underside and top (400 mm tall), hung 30 mm above the bench
T = 0.018  # carcass panels (18 mm melamine-faced particleboard)
T_BACK = 0.008
Y_IN = YR + T  # inside face of our right-hand side, where the hinge plates are: -0.232

# -- the doors (shut) -----------------------------------------------------------------------------------------------------
DT = 0.019  # door thickness
D_IN = XF - 0.0015  # inner face (1.5 mm off the carcass front, on the hinges' stop)
D_OUT = D_IN - DT  # outer face
D_Z0, D_Z1 = ZB + 0.002, ZT - 0.002  # 396 mm tall
Y_NEDGE = YL + 0.0012  # the neighbour door's right-hand edge, fixed: the line our gap is read against
GAP_NOM = 2.5  # mm: our door's gap when square and centred
Y_FREE = Y_NEDGE - GAP_NOM / 1000  # our door's free edge, square: 0.1987
DW = 0.446
Y_HEDGE = Y_FREE - DW  # its hinge edge: -0.2473
DOOR_MASS = 2.2  # kg: 446 x 396 x 19 mm particleboard at about 650 kg/m3, plus the knob and hinge cups
PIVOT = (D_IN, Y_HEDGE)  # the swing axis, vertical, at the door's inner hinge-side edge
Z_H = {"b": D_Z0 + 0.080, "t": D_Z1 - 0.080}  # hinge centres, 80 mm from the door's ends: 0.112 and 0.348
H_SPAN = Z_H["t"] - Z_H["b"]  # 0.236
Z_GAP = {"t": D_Z1 - 0.020, "b": D_Z0 + 0.020}  # where the gap is read: 20 mm below the top, 20 mm above the bottom
KNOB = (D_OUT - 0.020, Y_FREE - 0.035, D_Z0 + 0.060)  # 32 mm knob head centre: 35 mm in from the free edge, 60 mm up
KNOB_R, KNOB_STEM_R = 0.016, 0.0065

# -- the hinges' side adjustment -----------------------------------------------------------------------------------------
PITCH = 1.0e-3  # m of sideways movement at a hinge per turn of its side screw (assumed; see README)
TRAVEL_TURNS = 2.0  # +-2 mm (Blum: side adjustment +-2 mm), from mid-travel
GAP0 = {"t": 2.8, "b": 1.6}  # mm: the gaps as found


def _f(z: float) -> float:
    return (z - Z_H["b"]) / H_SPAN


def _offsets_for(gaps: dict) -> tuple[float, float]:
    """(delta_b, delta_t), metres: how far each hinge sits towards the hinge side of square for the given gaps (mm)."""
    ft, fb = _f(Z_GAP["t"]), _f(Z_GAP["b"])
    a = np.array([[1 - ft, ft], [1 - fb, fb]])
    s = np.array([gaps["t"] - GAP_NOM, gaps["b"] - GAP_NOM]) / 1000
    db, dt = np.linalg.solve(a, s)
    return float(db), float(dt)


DELTA0_B, DELTA0_T = _offsets_for(GAP0)  # -0.698 mm and +0.098 mm
ALPHA0 = math.atan((DELTA0_T - DELTA0_B) / H_SPAN)
Y_TILT = Y_HEDGE  # the tilt axis: along x through the lower hinge's height, at the door's hinge edge


def _crook(v: np.ndarray) -> np.ndarray:
    """Points of the square door moved to where the door hangs as found: tilted about x through the lower hinge, shifted."""
    v = np.array(v, float)
    dy, dz = v[:, 1] - Y_TILT, v[:, 2] - Z_H["b"]
    c, s = math.cos(ALPHA0), math.sin(ALPHA0)
    out = v.copy()
    out[:, 1] = Y_TILT + dy * c - dz * s - DELTA0_B
    out[:, 2] = Z_H["b"] + dy * s + dz * c
    return out


START_DEG = 50.0  # the door is found standing open this far; it is modelled in that pose (joint zero)


def _opened(v: np.ndarray) -> np.ndarray:
    """Points of the shut door turned open START_DEG about the swing axis (vertical, through PIVOT)."""
    v = np.array(v, float)
    a = math.radians(START_DEG)
    dx, dy = v[:, 0] - PIVOT[0], v[:, 1] - PIVOT[1]
    out = v.copy()
    out[:, 0] = PIVOT[0] + dx * math.cos(a) - dy * math.sin(a)
    out[:, 1] = PIVOT[1] + dx * math.sin(a) + dy * math.cos(a)
    return out


def _unopened(v: np.ndarray) -> np.ndarray:
    """The inverse of _opened: points in the door's (open) frame back in its shut pose."""
    v = np.array(v, float)
    a = -math.radians(START_DEG)
    dx, dy = v[:, 0] - PIVOT[0], v[:, 1] - PIVOT[1]
    out = v.copy()
    out[:, 0] = PIVOT[0] + dx * math.cos(a) - dy * math.sin(a)
    out[:, 1] = PIVOT[1] + dx * math.sin(a) + dy * math.cos(a)
    return out


def _crooked(mesh: m.Mesh) -> m.Mesh:
    """A door mesh as it hangs: crooked, and standing open START_DEG."""
    return m.Mesh(_opened(_crook(mesh.v)), mesh.f)


# -- the hinge arms on the side panel -----------------------------------------------------------------------------------
ARM_X = (XF + 0.003, XF + 0.066)  # the arm lies along the side from just behind the front edge, over its plate
ARM_HALF_W = 0.010  # 20 mm wide
Y_FACE = Y_IN + 0.016  # the arm's inner face, where the screw heads are: -0.216
XS = XF + 0.024  # the side-adjustment screw (front); the depth screw is behind it
XD = XF + 0.052
HEAD_R = 0.0045
HOLE = 0.0047  # half-width of the opening round the head in the arm's collider
# the PH2 recess, on the sprung seat: a four-flank funnel
REC_W0, REC_W1, REC_D, REC_R = 0.0030, 0.0005, 0.0030, 0.0044
K_SEAT = 20000.0  # N/m
SEAT_TRAVEL = 0.0015
SEAT_MIN = 0.0005  # 10 N
RADIAL_MAX = 0.0008
ANGLE_MAX_DEG = 6.0
T_TURN = 0.2  # N m to turn a side screw (estimate)
T_STOP = 3.0  # N m against the end of its travel
OMEGA0 = 0.15  # rad/s: the turning torque is fully on above about this speed (tanh)

# -- self-closing hinge springs -----------------------------------------------------------------------------------------
SELF_CLOSE_DEG = 25.0
CLOSE_K = 0.6  # N m/rad towards a rest 10 degrees past shut
CLOSE_PAST_DEG = 10.0
SHUT_TOL_DEG = 0.5

# -- the screwdriver and its stand ----------------------------------------------------------------------------------------
DRV_FILE_TIP = np.array([0.1375, 0.0, 0.0164])  # in the SimReady file's frame: the tip; the axis runs along x at z 0.0164
DRV_TIP = np.array([0.0946, 0.0, 0.0])  # the tip from the body's reported origin (the middle of its 188 mm length, on its axis)
STAND = (0.240, 0.240)  # centre of the stand's hole
STAND_W, STAND_H = 0.070, 0.045
RING_LO = (0.028, 0.0028)  # top of the lower (narrow) bore, and its half-width: the shaft's cone seats here
RING_HI = 0.0053  # half-width of the upper bore
TIP_SEATED_Z = 0.0059  # where the shaft seats the tip in the bore (measured on the settled scene)
GRASP_FROM_TIP = 0.1475  # TCP from the tip along the axis: the pads on the handle 25-55 mm from its end

GREYS = {
    "carcass": m.Look(color=m.rgb(0.93, 0.93, 0.91), roughness=0.45),
    "edge": m.Look(color=m.rgb(0.86, 0.86, 0.84), roughness=0.5),
    "wall": m.Look(color=m.rgb(0.80, 0.77, 0.70), roughness=0.85),
    "door": m.Look(color=m.rgb(0.55, 0.60, 0.62), roughness=0.4),
    "steel": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.9, roughness=0.25),
    "zinc": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.8, roughness=0.35),
    "cup": m.Look(color=m.rgb(0.70, 0.71, 0.72), metallic=0.85, roughness=0.3),
    "screw": m.Look(color=m.rgb(0.50, 0.50, 0.52), metallic=0.9, roughness=0.3),
    "slot": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.8),
    "paint": m.Look(color=m.rgb(0.95, 0.85, 0.10), roughness=0.5),
    "knob": m.Look(color=m.rgb(0.20, 0.20, 0.21), metallic=0.6, roughness=0.35),
    "pine": m.Look(color=m.rgb(0.80, 0.66, 0.45), roughness=0.75),
    "hole": m.Look(color=m.rgb(0.25, 0.18, 0.10), roughness=0.9),
}


def _span(x, y, z):
    size = (x[1] - x[0], y[1] - y[0], z[1] - z[0])
    return size, ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2)


def _knob_parts(y_knob: float, crooked: bool):
    stem_len = 0.013
    stem = m.cylinder(KNOB_STEM_R, stem_len, (KNOB[0] + 0.0135 - stem_len / 2 + 0.0005, y_knob, KNOB[2]), axis="x", segments=20)
    head = m.cylinder(KNOB_R, 0.014, (KNOB[0], y_knob, KNOB[2]), axis="x", segments=28)
    if crooked:
        stem, head = _crooked(stem), _crooked(head)
    return stem, head


def _cabinets() -> m.Part:
    parts, cols = [], []

    def add(name, x, y, z, look="carcass", collide=True):
        size, c = _span(x, y, z)
        parts.append((name, m.box(size, c), look))
        if collide:
            cols.append(m.col_box(f"{name}_c", size, c))

    for tag, (y0, y1) in (("our", (YR, YL)), ("nb", (YL, YN))):
        add(f"side_r_{tag}", (XF, XB), (y0, y0 + T), (ZB, ZT))
        add(f"side_l_{tag}", (XF, XB), (y1 - T, y1), (ZB, ZT))
        add(f"bottom_{tag}", (XF, XB - T_BACK), (y0 + T, y1 - T), (ZB, ZB + T))
        add(f"top_{tag}", (XF, XB - T_BACK), (y0 + T, y1 - T), (ZT - T, ZT))
        add(f"back_{tag}", (XB - T_BACK, XB), (y0 + T, y1 - T), (ZB + T, ZT - T), "edge")
    add("wall", (XB, XB + 0.0125), (-0.32, 0.56), (0.0, 0.62), "wall")
    # the neighbour's door, shut and square (its own hinges are not modelled: it is the fixed reference for the gap)
    add("nb_door", (D_OUT, D_IN), (Y_NEDGE, YN - 0.0015), (D_Z0, D_Z1), "door")
    stem, head = _knob_parts(Y_NEDGE + 0.035, False)
    parts += [("nb_knob_stem", stem, "knob"), ("nb_knob_head", head, "knob")]
    cols += [m.col_hull("nb_knob_stem_c", stem), m.col_hull("nb_knob_head_c", head)]
    return m.fixed("cabinets", (0.0, 0.0, TOP), parts, cols, GREYS, friction=(0.6, 0.5))


def _stand() -> m.Part:
    """A 70 x 70 x 45 mm pine block screwed to the bench, with a stepped bore: 5.6 mm across up to 28 mm, 10.6 mm above."""
    cx, cy = STAND
    hw = STAND_W / 2
    parts, cols = [], []

    def ring(tag, z0, z1, a):
        for k, (x, y) in enumerate((((cx - hw, cx - a), (cy - hw, cy + hw)), ((cx + a, cx + hw), (cy - hw, cy + hw)),
                                    ((cx - a, cx + a), (cy - hw, cy - a)), ((cx - a, cx + a), (cy + a, cy + hw)))):
            size, c = _span(x, y, (z0, z1))
            parts.append((f"{tag}{k}", m.box(size, c), "pine"))
            cols.append(m.col_box(f"{tag}{k}_c", size, c))

    ring("lo", 0.0, RING_LO[0], RING_LO[1])
    ring("hi", RING_LO[0], STAND_H, RING_HI)
    size, c = _span((cx - RING_LO[1], cx + RING_LO[1]), (cy - RING_LO[1], cy + RING_LO[1]), (0.0, 0.004))
    parts.append(("bore_floor", m.box(size, c), "hole"))
    cols.append(m.col_box("bore_floor_c", size, c))
    parts.append(("bore_shadow", m.box((2 * RING_HI, 2 * RING_HI, 0.001), (cx, cy, RING_LO[0] - 0.0005)), "hole"))
    return m.fixed("stand", (0.0, 0.0, TOP), parts, cols, GREYS, friction=(0.6, 0.5))


def _arm_frame(tag: str, z: float):
    """A mounting plate and the hinge arm clipped on it, along the side panel's inside face; colliders leave room round the screw head."""
    parts, cols = [], []
    plate = _span((XF + 0.017, XF + 0.057), (Y_IN, Y_IN + 0.006), (z - 0.022, z + 0.022))
    parts.append((f"plate_{tag}", m.box(*plate), "zinc"))
    cols.append(m.col_box(f"plate_{tag}_c", *plate))
    y0 = Y_IN + 0.006
    body = [((ARM_X[0], XS - HOLE), (z - ARM_HALF_W, z + ARM_HALF_W)), ((XS + HOLE, ARM_X[1]), (z - ARM_HALF_W, z + ARM_HALF_W)),
            ((XS - HOLE, XS + HOLE), (z + HOLE, z + ARM_HALF_W)), ((XS - HOLE, XS + HOLE), (z - ARM_HALF_W, z - HOLE))]
    for k, (x, zz) in enumerate(body):
        size, c = _span(x, (y0, Y_FACE), zz)
        parts.append((f"arm_{tag}{k}", m.box(size, c), "steel"))
        cols.append(m.col_box(f"arm_{tag}{k}_c", size, c))
    # the floor of the opening round the side screw's head, and the depth screw's head
    parts.append((f"well_{tag}", m.box(*_span((XS - HOLE, XS + HOLE), (y0, Y_FACE - REC_D - 0.002), (z - HOLE, z + HOLE))), "steel"))
    parts.append((f"dhead_{tag}", m.cylinder(HEAD_R, 0.0012, (XD, Y_FACE + 0.0006, z), axis="y", segments=24), "screw"))
    for k, (sx, sz) in enumerate(((0.0045, 0.0009), (0.0009, 0.0045))):
        parts.append((f"dslot_{tag}{k}", m.box((sx, 0.0004, sz), (XD, Y_FACE + 0.0013, z)), "slot"))
    # the hinge's cranked front end, round the carcass's front edge into the door's cup (it is part of the arm, fixed)
    parts.append((f"crank_{tag}", m.box(*_span((D_IN + 0.0005, ARM_X[0] + 0.004), (Y_IN - 0.012, Y_FACE), (z - 0.009, z + 0.009))), "steel"))
    return parts, cols


def _hull_mesh(pts: np.ndarray) -> m.Mesh:
    from scipy.spatial import ConvexHull

    h = ConvexHull(pts)
    faces = []
    centre = pts.mean(axis=0)
    for f in h.simplices:
        a, b, c = pts[f]
        faces.append(f if np.dot(np.cross(b - a, c - a), a - centre) > 0 else f[::-1])
    return m.Mesh(pts, np.array(faces, int))


def _seat_link(tag: str, z: float) -> m.Link:
    """The PH2 recess under a side screw's head, on a stiff spring along the screw's axis: the screwdriver's tip bears on
    it, its four sloping flanks centre the tip, and its give is how hard the screwdriver is pushed."""
    c = np.array([XS, Y_FACE, z])
    cols = []
    ex, ez = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])
    for k, (u, v) in enumerate(((ex, ez), (-ex, ez), (ez, ex), (-ez, ex))):
        pts = []
        for depth, w in ((0.0, REC_W0), (REC_D, REC_W1)):
            for sgn in (-1, 1):
                pts.append(c + u * w + v * sgn * w - np.array([0, depth, 0]))
                pts.append(c + u * REC_R + v * sgn * REC_R - np.array([0, depth, 0]))
        cols.append(m.col_hull(f"flank_{tag}{k}", _hull_mesh(np.array(pts))))
    size, cc = _span((XS - REC_R, XS + REC_R), (Y_FACE - REC_D - 0.001, Y_FACE - REC_D), (z - REC_R, z + REC_R))
    cols.append(m.col_box(f"floor_{tag}", size, cc))
    return m.slider(f"seat_{tag}", "Y", -SEAT_TRAVEL, 0.0, (), cols, at=tuple(c), mass=0.02, spring=K_SEAT, damping=20.0, friction=0.0,
                    effort=500.0, armature=0.001, inertia=(1e-6, 1e-6, 1e-6))


def _screw_link(tag: str, z: float) -> m.Link:
    """The side screw's head, seen: it turns as the rule turns it (its recess is the seat's)."""
    parts = [(f"head_{tag}", m.cylinder(HEAD_R, 0.0010, (XS, Y_FACE + 0.0003, z), axis="y", segments=28), "screw")]
    for k, (sx, sz) in enumerate(((2 * REC_W0 + 0.0006, 0.0011), (0.0011, 2 * REC_W0 + 0.0006))):
        parts.append((f"slot_{tag}{k}", m.box((sx, 0.0004, sz), (XS, Y_FACE + 0.0009, z)), "slot"))
    parts.append((f"mark_{tag}", m.box((0.0007, 0.0004, HEAD_R - REC_W0 - 0.0004), (XS, Y_FACE + 0.0009, z + (HEAD_R + REC_W0) / 2 + 0.0002)), "paint"))
    return m.hinge(f"screw_{tag}", "Y", (XS, Y_FACE, z), float("-inf"), float("inf"), parts, (), mass=0.005, spring=0.5, damping=0.005,
                   friction=0.0, effort=5.0, armature=1e-4, inertia=(1e-6, 1e-6, 1e-6))


def _door_parts():
    slab_size, slab_c = _span((D_OUT, D_IN), (Y_HEDGE, Y_FREE), (D_Z0, D_Z1))
    slab = _crooked(m.box(slab_size, slab_c))
    parts = [("slab", slab, "door")]
    cols = [m.col_hull("slab_c", slab)]
    stem, head = _knob_parts(KNOB[1], True)
    parts += [("knob_stem", stem, "knob"), ("knob_head", head, "knob")]
    cols += [m.col_hull("knob_stem_c", stem), m.col_hull("knob_head_c", head)]
    cup_y = Y_HEDGE + 0.0225
    for tag, z in Z_H.items():
        parts.append((f"cup_{tag}", _crooked(m.cylinder(0.0175, 0.0015, (D_IN + 0.0006, cup_y, z), axis="x", segments=28)), "cup"))
        parts.append((f"boss_{tag}", _crooked(m.box(*_span((D_IN, D_IN + 0.004), (cup_y - 0.012, Y_HEDGE + 0.003), (z - 0.009, z + 0.009)))), "steel"))
    return parts, cols


def _hinges() -> m.Part:
    frame_parts, frame_cols = [], []
    for tag, z in Z_H.items():
        p, c = _arm_frame(tag, z)
        frame_parts += p
        frame_cols += c
    shift = m.slider("shift", "Y", -0.004, 0.004, (), (), at=(XF, Y_TILT, Z_H["b"]), mass=0.3, spring=2.0e6, damping=4.0e3, friction=0.0,
                     effort=1.0e5, armature=0.01, inertia=(1e-3, 1e-3, 1e-3))
    tilt = m.hinge("tilt", "X", (XF, Y_TILT, Z_H["b"]), -2.0, 2.0, (), (), mass=0.3, spring=2.0e5, damping=400.0, friction=0.0, effort=1.0e5,
                   armature=0.01, inertia=(1e-3, 1e-3, 1e-3), parent="shift")
    door_parts, door_cols = _door_parts()
    com = tuple(_opened(np.array([[(D_OUT + D_IN) / 2, (Y_HEDGE + Y_FREE) / 2, (D_Z0 + D_Z1) / 2]]))[0])
    dh = D_Z1 - D_Z0
    i_side = DOOR_MASS * (DW**2 + DT**2 + 2 * dh**2) / 24  # about horizontal axes through the centre, averaged: the door stands turned
    inertia = (i_side, i_side, DOOR_MASS * (DW**2 + DT**2) / 12)
    door = m.hinge("door", "Z", (PIVOT[0], PIVOT[1], com[2]), -START_DEG, 110.0 - START_DEG, door_parts, door_cols, mass=DOOR_MASS, com=com, inertia=inertia,
                   spring=CLOSE_K, damping=0.3, friction=0.03, parent="tilt")
    links = [shift, tilt, door]
    for tag, z in Z_H.items():
        links += [_seat_link(tag, z), _screw_link(tag, z)]
    return m.mechanism("hinges", (0.0, 0.0, TOP), links, GREYS, frame_parts=frame_parts, frame_colliders=frame_cols, friction=(0.6, 0.5),
                       solver_iters=64)


_Q_STAND = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # the file's +x (towards the tip) turned to point down

OBJECTS = [
    _cabinets(),
    _hinges(),
    _stand(),
    SimReady("screwdriver", pos=(STAND[0] - DRV_FILE_TIP[2], STAND[1], TOP + TIP_SEATED_Z + 0.002 + DRV_FILE_TIP[0]), size=(0.188, 0.034, 0.032), mass=0.10,
             quat_xyzw=_Q_STAND, package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd"),
    # Distractors (SimReady props), loose on the bench clear of the door's swing and the arms' paths.
    SimReady("mug", pos=(0.28, -0.37, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=2.0,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("utility_knife", pos=(0.12, -0.28, TOP), size=(0.160, 0.061, 0.026), mass=0.15, yaw=0.6,
             package="handTool_knife_retractableBlade_a01", usd="sm_handTool_knife_retractableBlade_a01_01.usd"),
    SimReady("marker", pos=(0.12, 0.25, TOP), size=(0.109, 0.008, 0.007), mass=0.01, yaw=1.2,
             package="stationery_marker_thin_blue_a01", usd="sm_stationery_marker_thin_blue_a01_01.usd"),
]


# -- live rules -----------------------------------------------------------------------------------------------------------


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _box_gap(local: np.ndarray, lo, hi) -> np.ndarray:
    out = np.maximum(np.maximum(np.asarray(lo) - local, local - np.asarray(hi)), 0.0)
    return np.linalg.norm(out, axis=1)


def _rotvec(r: np.ndarray) -> np.ndarray:
    c = max(-1.0, min(1.0, (np.trace(r) - 1) / 2))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w / (2 * math.sin(ang)) * ang


INWARD = np.array([0.0, -1.0, 0.0])  # a side screw's axis, from the cabinet's inside into the side panel
HANDS_OFF = 0.03
STILL_RAD_S = 0.02  # the door's free edge moving under 9 mm/s
HOLD_STEPS = round(SUCCESS_HOLD_S * 20)
CARCASS_BOX = ((XF, YR, ZB), (XB, YN, ZT))
DOOR_BOX = ((D_OUT - 0.036, Y_HEDGE - 0.002, D_Z0 - 0.002), (D_IN + 0.005, Y_FREE + 0.002, D_Z1 + 0.002))  # with its knob
NB_DOOR_BOX = ((D_OUT - 0.036, Y_NEDGE, D_Z0), (D_IN, YN, D_Z1))
TURN_SIGN = 1.0  # clockwise seen from the handle (right-handed about INWARD) moves a hinge towards the hinge side


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the screws' turns and the record are cleared
        self.phi = {"b": 0.0, "t": 0.0}  # turns of each side screw from where it was found
        self.prev_rot = None
        self.prev_eng = {"b": False, "t": False}
        self.good = 0
        self.last: dict = {}
        self.n = 0

    def _driver(self, env):
        from yam_arena.scene import object_states

        s = object_states(env, ("screwdriver",))["screwdriver"]
        r = m.quat_matrix(s.quat_xyzw)
        return np.asarray(s.pos, float), r, np.asarray(s.ang_vel, float)

    def _wrench(self, env, torque: np.ndarray, centre) -> None:
        """The screw's reaction on the screwdriver: its turning resistance (a torque), and the recess's centring push at the tip."""
        import torch

        base = env.unwrapped
        asset = base.scene["screwdriver"]
        force = np.zeros(3)
        torque = np.asarray(torque, float)
        if centre is not None:  # a push at the tip: the same push at the centre of mass, plus its moment about it
            origin = _np(base.scene.env_origins[0])
            com = _np(asset.data.root_com_pos_w)[0] - origin - np.array([0.0, 0.0, TOP])
            force = np.asarray(centre[0], float)
            torque = torque + np.cross(np.asarray(centre[1], float) - com, force)
        t = torch.tensor([[[float(v) for v in torque]]], device=asset.device)
        f = torch.tensor([[[float(v) for v in force]]], device=asset.device)
        asset.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=t, positions=None, body_ids=[0], is_global=True)
        m._PUSHED.add(("screwdriver", 0))  # so the kit clears it before a reset's settling steps

    def step(self, env):
        st = m.state(env, "hinges")
        centre = None
        pos, rot, w = self._driver(env)
        pos = pos - np.array([0.0, 0.0, TOP])  # heights above the bench, as everything here
        axis = rot @ np.array([1.0, 0.0, 0.0])
        tip = pos + rot @ DRV_TIP
        twist = 0.0 if self.prev_rot is None else float(_rotvec(rot @ self.prev_rot.T) @ INWARD)
        self.prev_rot = rot
        angle = math.degrees(math.acos(max(-1.0, min(1.0, float(axis @ INWARD)))))
        eng, info = {}, {}
        for tag, z in Z_H.items():
            press = -st.joint(f"seat_{tag}")
            radial = math.hypot(tip[0] - XS, tip[2] - z)
            eng[tag] = bool(press >= SEAT_MIN and radial <= RADIAL_MAX and angle <= ANGLE_MAX_DEG)
            info[tag] = (press, radial)
        torque = np.zeros(3)
        for tag in Z_H:
            if not eng[tag]:
                continue
            if self.prev_eng[tag]:
                self.phi[tag] = float(np.clip(self.phi[tag] + TURN_SIGN * twist / (2 * math.pi), -TRAVEL_TURNS, TRAVEL_TURNS))
            omega = float(w @ INWARD)
            at_stop = (self.phi[tag] >= TRAVEL_TURNS and omega > 0) or (self.phi[tag] <= -TRAVEL_TURNS and omega < 0)
            torque = -(T_STOP if at_stop else T_TURN) * math.tanh(omega / OMEGA0) * INWARD
        self.prev_eng = eng
        self._wrench(env, torque, centre)
        # the screws move the door: each hinge sideways by PITCH per turn
        db = DELTA0_B + PITCH * self.phi["b"]
        dt = DELTA0_T + PITCH * self.phi["t"]
        m.set_target(env, "hinges", "shift", -(db - DELTA0_B))
        m.set_target(env, "hinges", "tilt", math.atan((dt - db) / H_SPAN) - ALPHA0)
        for tag in Z_H:
            m.set_target(env, "hinges", f"screw_{tag}", -TURN_SIGN * 2 * math.pi * self.phi[tag])  # the joint is about +y
        # the hinges' springs shut the door over the last 25 degrees; above that it stays where it is put
        q = st.joint("door")
        open_deg = START_DEG + math.degrees(q)
        m.set_target(env, "hinges", "door", math.radians(-CLOSE_PAST_DEG - START_DEG) if open_deg < SELF_CLOSE_DEG else q)
        # what the scorer reads: the gaps, from where the door's free edge actually is
        base = env.unwrapped
        asset = base.scene["hinges"]
        origin = _np(base.scene.env_origins[0])
        names = list(asset.body_names)
        p = _np(asset.data.body_link_pose_w)[0][names.index("door")]
        r_door = m.quat_matrix(p[3:7])
        gaps = {}
        for tag, z in Z_GAP.items():
            edge = _opened(_crook(np.array([[(D_OUT + D_IN) / 2, Y_FREE, z]])))[0]
            world = p[:3] - origin + r_door @ edge
            gaps[tag] = (Y_NEDGE - world[1]) * 1000
        tips = m.fingertips(env)
        tips_b = tips - np.array([0.0, 0.0, TOP])
        local = _unopened((tips - (p[:3] - origin)) @ r_door)  # the fingers in the shut, square door's frame
        clear = min(float(_box_gap(local, *DOOR_BOX).min()), float(_box_gap(tips_b, *CARCASS_BOX).min()), float(_box_gap(tips_b, *NB_DOOR_BOX).min()))
        qd = _np(asset.data.joint_vel)[0]
        q_vel = float(qd[list(asset.joint_names).index("door_joint")])
        self.last = {"open_deg": open_deg, "gap_t": gaps["t"], "gap_b": gaps["b"], "clear": clear, "q_vel": q_vel,
                     "phi_b": self.phi["b"], "phi_t": self.phi["t"]}
        self.good = self.good + 1 if self._full() else 0
        self.n += 1
        if os.environ.get("SQUARE_TRACE") and self.n % 10 == 0:  # a line every 0.5 s: what the rules saw
            print(f"[square] step {self.n} door {open_deg:.2f} deg, gaps top {gaps['t']:.2f} bottom {gaps['b']:.2f} mm, "
                  f"lower screw {self.phi['b']:+.3f} turns (seat {info['b'][0] * 1000:.2f} mm, tip {info['b'][1] * 1000:.2f} mm off axis, "
                  f"driver {angle:.1f} deg, engaged {eng['b']}), top screw {self.phi['t']:+.3f}, hands {clear * 100:.1f} cm", flush=True)

    def _window(self) -> bool:
        r = self.last
        return bool(r and r["open_deg"] <= SHUT_TOL_DEG and 2.0 <= r["gap_t"] <= 3.0 and 2.0 <= r["gap_b"] <= 3.0
                    and abs(r["gap_t"] - r["gap_b"]) <= 0.5)

    def _full(self) -> bool:
        r = self.last
        return bool(self._window() and r["clear"] >= HANDS_OFF and abs(r["q_vel"]) < STILL_RAD_S)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    if not TRIAL.last:
        return 0.0
    if TRIAL.good >= HOLD_STEPS:
        return 1.0
    if TRIAL._window():
        return 0.5
    if TRIAL.phi["b"] >= 0.25:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return bool(TRIAL._full())


# -- the scripted demo: an open-loop plan from the published layout -------------------------------------------------------


class Grip(float):
    """A waypoint's yaw that carries a whole TCP rotation (the harness's ArmPlan passes it to grip_rotation, extended below)."""

    rot: np.ndarray = np.eye(3)


def _install_grips() -> None:
    if getattr(_harness_demo.grip_rotation, "_takes_grips", False):
        return
    plain = _harness_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return plain(yaw) if rot is None else rot

    grip_rotation._takes_grips = True
    grip_rotation._plain = plain
    _harness_demo.grip_rotation = grip_rotation


_install_grips()


def _grip(rot: np.ndarray) -> Grip:
    g = Grip(0.0)
    g.rot = rot
    return g


def tilted(theta_deg: float, phi_deg: float) -> Grip:
    """Tilted theta_deg from straight down towards the direction phi_deg from +x, the fingers closing across it."""
    phi = math.radians(phi_deg)
    plain = _harness_demo.grip_rotation._plain(phi)
    return _grip(axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ plain)


def _level_rot(ahead, psi_deg: float) -> np.ndarray:
    """Fingers level, pointing along `ahead`, rolled psi_deg about it (right-handed)."""
    z = np.asarray(ahead, float) / np.linalg.norm(ahead)
    x = np.cross([0.0, 0.0, 1.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return axis_angle_matrix(z, math.radians(psi_deg)) @ np.stack([x, y, z], 1)


def _rx(deg: float) -> np.ndarray:
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(deg))


def _rot_z(deg: float, p, about):
    a = math.radians(deg)
    d = np.asarray(p, float) - np.asarray(about, float)
    return np.asarray(about, float) + np.array([d[0] * math.cos(a) - d[1] * math.sin(a), d[0] * math.sin(a) + d[1] * math.cos(a), d[2]])


def _w(t, p, finger, label, grip):
    return Waypoint(t, (float(p[0]), float(p[1]), float(p[2]) + TOP), float(finger), label, grip)


OPEN_TO = START_DEG
PSI_A, PSI_B = -295.0, -75.0  # the wrist's roll at each end of a stroke (clockwise from the handle is increasing)
PUSH_OVER = 0.035  # commanded past the recess's floor
TRIM = np.array([0.0006, 0.0029, 0.0067])  # the arm's droop holding the screwdriver level at the lower hinge (measured)
TIP_OFF = np.array([0.0011, -0.1509, -0.0053])  # the tip from the TCP as the screwdriver hangs in the grip, level (measured)
PARK = np.array([0.20, 0.30, 0.20])
# where to aim the TCP, beyond the screw's axis, so that the soft wrist, bent by the push, ends up in line with it
# (measured on the settled pushes at each roll; one entry per roll from PSI_A to PSI_B)
LEAN = [np.array([dx, 0.0, dz]) for dx, dz in ((-0.017, 0.009), (-0.017, -0.007), (-0.017, -0.009), (-0.017, 0.011), (-0.018, 0.010), (-0.021, -0.008))]
STROKES = (5, 2)  # a full stroke of the wrist (220 degrees), then two-fifths of one
MODE = "full"  # the red-team variants (README) are copies of this file with another MODE
PINCH_OPEN = -0.028  # fingers opened to about 56 mm round the 34 mm handle


def _left_plan(mode: str = "full") -> list:
    out = []
    lvl_a = _level_rot(INWARD, PSI_A)
    g_down = _grip(_rx(90.0) @ lvl_a)  # straight down; tilting it 90 degrees about x makes it level, pointing into the screw
    g_half = _grip(_rx(45.0) @ lvl_a)
    g_lvl = _grip(lvl_a)
    ax = np.array([STAND[0], STAND[1], 0.0])
    z_grasp = TIP_SEATED_Z + GRASP_FROM_TIP
    out += [
        _w(2.0, ax + [0, 0, 0.22], PINCH_OPEN, "above the screwdriver", g_down),
        _w(1.2, ax + [0, 0, z_grasp], PINCH_OPEN, "fingers round its handle", g_down),
        _w(0.6, ax + [0, 0, z_grasp], FINGER_CLOSED, "grip the handle", g_down),
        _w(1.2, ax + [0, 0, 0.215], FINGER_CLOSED, "lift it out of its stand", g_down),
        _w(1.5, (0.32, 0.10, 0.195), FINGER_CLOSED, "behind the open door", g_down),
        _w(1.0, (0.32, 0.10, 0.19), FINGER_CLOSED, "tip it", g_half),
        _w(1.0, (0.32, 0.10, 0.185), FINGER_CLOSED, "level, pointing at the right-hand side", g_lvl),
    ]
    zb = Z_H["b"]
    tcp = lambda tip, roll=None: np.asarray(tip, float) - TIP_OFF + TRIM  # noqa: E731  where to send the TCP to put the tip at `tip`
    out += [
        _w(1.5, (0.39, 0.0, zb + 0.01), FINGER_CLOSED, "down to the lower hinge's height", g_lvl),
        _w(1.0, (0.41, -0.040, zb + 0.01), FINGER_CLOSED, "into the opening", g_lvl),
        _w(1.2, tcp((XS, Y_FACE + 0.010, zb)), FINGER_CLOSED, "tip in line with the side screw", g_lvl),
        _w(1.0, tcp((XS, Y_FACE - 0.005, zb)), FINGER_CLOSED, "tip into the recess", g_lvl),
    ]
    psis = np.linspace(PSI_A, PSI_B, 6)  # the wrist's roll through a full stroke, PSI_A first
    pushed = tcp((XS, Y_FACE - 0.005 - PUSH_OVER, zb))
    relaxed = tcp((XS, Y_FACE - 0.015, zb))
    last = 0
    strokes = {"nopush": (), "ccw": (), "over": (5, 5, 5)}.get(mode, STROKES)
    tip0 = np.array([XS, Y_FACE - 0.003, zb])

    def turned(pos, rot):  # red team "tilted": the screwdriver swung 20 degrees off the screw's axis about its tip
        if mode != "tilted":
            return pos, _grip(rot)
        rz = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(20.0))
        return tip0 + rz @ (np.asarray(pos) - tip0), _grip(rz @ rot)

    for k, n in enumerate(strokes):  # n: how many of the stroke's five segments to turn through
        pos, grip = turned(pushed + LEAN[0], _level_rot(INWARD, PSI_A))
        out += [_w(1.0, pos, FINGER_CLOSED, "push it in", grip), _w(0.4, pos, FINGER_CLOSED, "push", grip)]
        for i in range(1, n + 1):
            pos, grip = turned(pushed + LEAN[i], _level_rot(INWARD, psis[i]))
            out.append(_w(1.1, pos, FINGER_CLOSED, "turn clockwise, pushing", grip))
        last = n
        if k == len(strokes) - 1:
            break
        out.append(_w(0.8, relaxed + 0.4 * LEAN[n], FINGER_CLOSED, "ease the push, tip still in the recess", _grip(_level_rot(INWARD, psis[n]))))
        for i in range(n - 1, -1, -1):
            out.append(_w(0.8, relaxed + 0.4 * LEAN[i], FINGER_CLOSED, "wind the wrist back, pushing lightly", _grip(_level_rot(INWARD, psis[i]))))
    if mode == "nopush":  # red team: the tip in the recess, turned without pushing
        for i in range(1, 6):
            out.append(_w(1.1, relaxed, FINGER_CLOSED, "turn without pushing", _grip(_level_rot(INWARD, psis[i]))))
        last = 5
    if mode == "ccw":  # red team: the right screw, turned the wrong way (anticlockwise), pushed properly
        for i in range(1, 6):
            out.append(_w(0.8, relaxed + 0.4 * LEAN[i], FINGER_CLOSED, "wind the wrist round first", _grip(_level_rot(INWARD, psis[i]))))
        out += [_w(1.0, pushed + LEAN[5], FINGER_CLOSED, "push it in", _grip(_level_rot(INWARD, PSI_B))),
                _w(0.4, pushed + LEAN[5], FINGER_CLOSED, "push", _grip(_level_rot(INWARD, PSI_B)))]
        for i in range(4, -1, -1):
            out.append(_w(1.1, pushed + LEAN[i], FINGER_CLOSED, "turn anticlockwise, pushing", _grip(_level_rot(INWARD, psis[i]))))
        last = 0
    g_end = _grip(_level_rot(INWARD, psis[last]))
    out += [
        _w(0.8, tcp((XS, Y_FACE + 0.02, zb)), FINGER_CLOSED, "off the screw", g_end),
        _w(1.0, (0.41, -0.040, zb + 0.01), FINGER_CLOSED, "out of the hinge", g_end),
        _w(1.0, (0.39, 0.0, zb + 0.01), FINGER_CLOSED, "out of the cabinet", g_end),
        _w(1.5, (0.32, 0.10, 0.185), FINGER_CLOSED, "up", g_end),
        _w(1.0, (0.32, 0.10, 0.185), FINGER_CLOSED, "wind the wrist back", _grip(_level_rot(INWARD, (PSI_A + psis[last]) / 2))),
        _w(1.0, (0.32, 0.10, 0.185), FINGER_CLOSED, "wind the wrist back", g_lvl),
        _w(2.0, PARK, FINGER_CLOSED, "hold it upright, clear of the door's swing", g_down),
        _w(0.5, PARK, FINGER_CLOSED, "hold", g_down),
    ]
    return out


def _right_plan(wait: float, mode: str = "full") -> list:
    pivot = np.array([PIVOT[0], PIVOT[1], 0.0])
    r_push, z_push = 0.30, 0.25
    out = [_w(2.0, (0.20, -0.25, 0.20), FINGER_CLOSED, "wait", tilted(0.0, 0.0)), _w(wait, (0.20, -0.25, 0.20), FINGER_CLOSED, "wait", tilted(0.0, 0.0))]

    def contact(deg):  # a point on the door's outer face, r_push from the hinge edge, and the inward normal there
        p = _rot_z(deg, np.array([D_OUT, Y_HEDGE + r_push, z_push]), pivot)
        n = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0])
        return p, n

    p, n = contact(OPEN_TO)
    tcp = lambda p, n, gap: p - n * (0.013 + gap)  # noqa: E731  fingertips 13 mm past the TCP
    out += [
        _w(2.0, tcp(p, n, 0.03), FINGER_CLOSED, "fingertips facing the door", tilted(90.0, OPEN_TO)),
        _w(1.0, tcp(p, n, 0.002), FINGER_CLOSED, "fingertips on the door", tilted(90.0, OPEN_TO)),
    ]
    for deg in np.arange(OPEN_TO - 4.0, 11.9, -4.0):
        p, n = contact(deg)
        out.append(_w(0.5, tcp(p, n, -0.002), FINGER_CLOSED, "push it towards shut", tilted(90.0, deg)))
    if mode == "hold":  # red team: push it all the way shut and keep the hand on it
        out.append(_w(0.4, tcp(*contact(0.0), -0.004), FINGER_CLOSED, "push it shut", tilted(90.0, 0.0)))
        out.append(_w(6.0, tcp(*contact(0.0), -0.004), FINGER_CLOSED, "keep the hand on it", tilted(90.0, 0.0)))
        return out
    p, n = contact(12.0)
    out += [
        _w(0.6, tcp(p, n, 0.04), FINGER_CLOSED, "let it go: the hinge springs shut it", tilted(90.0, 12.0)),
        _w(1.5, (0.20, -0.25, 0.20), FINGER_CLOSED, "away", tilted(0.0, 0.0)),
    ]
    return out


def demo():
    if MODE == "none":  # red team: do nothing
        return {"left": [_w(40.0, PARK, FINGER_CLOSED, "wait", _grip(_rx(90.0) @ _level_rot(INWARD, PSI_A)))],
                "right": [_w(40.0, (0.20, -0.25, 0.20), FINGER_CLOSED, "wait", tilted(0.0, 0.0))]}
    left = _left_plan(MODE)
    t_left = sum(w.duration for w in left)
    right = _right_plan(t_left - 2.0, MODE)
    if MODE == "onearm":  # red team: one arm turns the screw; nothing shuts the door
        right = right[:2]
    return {"left": left, "right": right}
