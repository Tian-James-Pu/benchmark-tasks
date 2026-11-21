"""Fit a hinge-pin door stop on the top hinge of a hung interior door.

A 28 x 80 in (711 x 2032 mm), 1-3/8 in hollow-core interior door hangs on three 3-1/2 x 3-1/2 in five-knuckle butt hinges (0.234 in pin).
Its top hinge pin has already been tapped up 40 mm (the step done with a nail set and hammer). An adjustable hinge-pin door stop (zinc
body, a ring for the pin, a rubber tip on a stud and a rubber tip on the end boss) lies on the work surface in front of the door. The
robot must pull the pin, set the stop's ring on the top knuckle, put the pin back through the ring and the knuckles until its head sits
on the stop, and let go.

Model (declared in code; every state change is contact):
- Door ("door", articulation fixed to the world): the frame carries the stud-wall section, the jambs, the head jamb, the casing on this
  side and the top hinge's jamb leaf with knuckles 1, 3 and 5, each bored 6.36 mm for the 5.94 mm pin (12 convex sectors per knuckle,
  the top knuckle's bore chamfered 1.0 mm). The door leaf is a link on a hinge about the pin's axis (the axis the middle and bottom
  hinges keep), 10 kg, with the inertia of the whole leaf; it carries knuckles 2 and 4 (seen; they turn about the pin's own axis). It
  starts 2 degrees open and swings from shut to 170 degrees open.
- Pin ("pin", free, 22 g): 5.94 mm steel shank 88 mm long with a rounded tip, a 10.4 mm button head. Steel-on-steel friction. Its
  hand-loose fit in the knuckles is sliding friction along its axis of up to 3 N while its tip is in the top knuckle (class Fit).
- Stop ("stop", free, 35 g): a 3.5 mm ring (6.5 mm hole, its mouth chamfered 1.2 mm) that sits on the top knuckle under the pin's head, a 6 mm zinc bar to a
  12 mm boss 64 mm out, a rubber tip on a stud near the ring that bears on the casing and a rubber tip on the boss that bears on the
  door's face: set to stop the door at about 103 degrees. Its weight hangs well out from the ring, so set on the knuckle without the
  pin through it, it tips off.
- Not modelled: a tight pin (it is hand-loose), the middle and bottom hinges and the lower 1.75 m of the door (the table top stands in
  for a plane 1.75 m above the floor), the stop's adjusting thread (fixed).

Scoring (end state, read every control step by the tracker; `score()` returns it):
- Stop seated: its ring's centre within 1.0 mm of the hinge axis, its underside -0.5 to +2.0 mm from the top knuckle's top, tilted at
  most 10 degrees.
- Pin home: head and tip within 1.0 mm of the hinge axis, within 3 degrees of vertical, its head's underside -0.5 to +1.5 mm from the
  top of the stop's ring.
- 1.0: stop seated and pin home, both still (no tracked point moving more than 0.5 mm between steps), and every fingertip point at
  least 3 cm from the pin and from the stop, continuously for 1 s.
- 0.75: stop seated and pin home, but not yet 1.0 (held, or not yet for 1 s).
- 0.5: stop seated with the pin through its ring and at least the top knuckle (head and tip within 1.0 mm of the axis, the tip at
  least 18 mm below the knuckle's top).
- 0.25: stop seated, the pin not through it.
- 0.0 otherwise (the pin back without the stop, the stop off the hinge, the starting state).
Full credit lasts only while the state that earned it does.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Fit the hinge-pin door stop on this door."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the hinge: 3-1/2 x 3-1/2 in, five knuckles, 0.234 in pin; the pin's axis is the door mechanism's origin --------------------------
PIN_XY = (0.420, 0.080)  # world, the hinge axis
Z_K = 0.100  # the top knuckle's top above the table (the table stands in for a plane 1.75 m up)
H_HINGE = 0.0889
KH = H_HINGE / 5  # one knuckle, 17.8 mm
R_PIN = 0.00297  # 0.234 in
R_BORE = 0.00318  # 1/4 in knuckle bore
R_KN = 0.00525  # the barrel: the leaf rolled round the pin
CHAMFER = 0.0010
# -- the door: 28 x 80 x 1-3/8 in hollow core, its face 16 mm behind the pin's axis (3-1/2 in hinge, 1/4 in backset) ----------------
T_DOOR, W_DOOR = 0.035, 0.711
FACE = 0.016  # the door's face (this side) and the jamb's edge, from the pin's axis, toward the wall (+x)
GAP = 0.0016  # half the hinge-side gap
Z_DT = Z_K + 0.178  # the door's top: the top hinge's top is 7 in below it
OPEN0 = 2.0  # degrees open at the start: closed most of the way, as the stop's instructions say
JAMB_T, JAMB_D = 0.019, 0.1143  # 3/4 in pine, 4-1/2 in deep (2x4 wall, 1/2 in drywall)
CASING_T, CASING_W, REVEAL = 0.011, 0.0572, 0.0048  # 2-1/4 in colonial casing, 7/16 in at its inner edge; 3/16 in reveal
Z_WALL = 0.45  # the wall section is cut here
Y_WALL = 0.30  # and here, beyond the hinge jamb

# -- the stop: zinc body, ring at its origin (ring's underside on the axis), bar out along -x -------------------------------------------
R_RING_IN, R_RING_OUT, T_RING = 0.00325, 0.0059, 0.0035
BAR_X0, BAR_X1, BAR_W, BAR_H = -0.064, -0.006, 0.012, 0.006  # the bar: x from -64 to -6 mm, 12 mm wide, 6 mm tall
BOSS_X0, BOSS_X1, BOSS_H = -0.064, -0.044, 0.012  # the boss at the bar's end that carries the end tip
C_TIP = (0.0, 0.016)  # the stud tip's face (on the casing at the set angle), its centre in xy; points +x
D_TIP = (-0.054, -0.0105)  # the end tip's face, its centre; points -y (on the door's face at the set angle)
R_TIP, L_TIP, Z_TIP = 0.006, 0.010, 0.0065
M_STOP = 0.035
M_PIN = 0.022
L_PIN, R_HEAD, T_HEAD = 0.088, 0.0052, 0.003

LOOKS = {
    "paint": m.Look(color=m.rgb(0.93, 0.92, 0.88), roughness=0.55),
    "trim": m.Look(color=m.rgb(0.97, 0.97, 0.95), roughness=0.4),
    "pine": m.Look(color=m.rgb(0.86, 0.80, 0.68), roughness=0.6),
    "door": m.Look(color=m.rgb(0.96, 0.96, 0.94), roughness=0.35),
    "hinge": m.Look(color=m.rgb(0.62, 0.60, 0.56), metallic=0.85, roughness=0.35),
    "pin": m.Look(color=m.rgb(0.70, 0.68, 0.64), metallic=0.9, roughness=0.25),
    "zinc": m.Look(color=m.rgb(0.55, 0.53, 0.50), metallic=0.85, roughness=0.35),
    "rubber": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.8),
    "stud": m.Look(color=m.rgb(0.75, 0.72, 0.62), metallic=0.8, roughness=0.3),
}


def _sector(r0_bot, r1, z0, z1, a0, a1, r0_top=None):
    """A convex sector of an annulus (inner radius r0 at the bottom, r0_top at the top), from angle a0 to a1."""
    r0_top = r0_bot if r0_top is None else r0_top
    pts = []
    for a in (a0, a1):
        for r, z in ((r0_bot, z0), (r1, z0), (r0_top, z1), (r1, z1)):
            pts.append((r * math.cos(a), r * math.sin(a), z))
    v = np.array(pts)
    f = np.array([(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)])
    return m.Mesh(v, f)


def _bore(prefix, r_in, r_out, z0, z1, n=12, chamfer=0.0):
    """Colliders of a bored knuckle or ring: n sectors, the top `chamfer` of the bore opened out by 45 degrees."""
    cols = []
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        if chamfer > 0:
            cols.append(m.col_hull(f"{prefix}_{k:02d}", _sector(r_in, r_out, z0, z1 - chamfer, a0, a1)))
            cols.append(m.col_hull(f"{prefix}c_{k:02d}", _sector(r_in, r_out, z1 - chamfer, z1, a0, a1, r_in + chamfer)))
        else:
            cols.append(m.col_hull(f"{prefix}_{k:02d}", _sector(r_in, r_out, z0, z1, a0, a1)))
    return cols


def _knuckle_z(i):
    """Knuckle i (1 the top) from z0 to z1 above the table."""
    return Z_K - i * KH, Z_K - (i - 1) * KH


def _rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _turned(mesh, deg):
    return m.Mesh(mesh.v @ _rot_z(deg).T, mesh.f)


# -- the door: frame (wall, jambs, casing, jamb leaf and knuckles 1, 3, 5) and the door leaf --------------------------------------------
def _frame():
    parts, cols = [], []
    x0, x1 = FACE, FACE + JAMB_D  # the wall's faces, from the pin's axis
    yj0, yj1 = GAP, GAP + JAMB_T  # the hinge jamb
    y_latch = -GAP - W_DOOR - 0.003  # the latch jamb's face
    z_head = Z_DT + 0.003  # the head jamb's underside
    # hinge-side wall, header and the latch jamb's wall stub
    walls = [("wall_h", (x1 - x0, Y_WALL - yj1, Z_WALL), ((x0 + x1) / 2, (yj1 + Y_WALL) / 2, Z_WALL / 2)),
             ("wall_top", (x1 - x0, Y_WALL - (y_latch - JAMB_T - 0.05), Z_WALL - (z_head + JAMB_T)),
              ((x0 + x1) / 2, (Y_WALL + y_latch - JAMB_T - 0.05) / 2, (Z_WALL + z_head + JAMB_T) / 2)),
             ("wall_l", (x1 - x0, 0.05, Z_WALL), ((x0 + x1) / 2, y_latch - JAMB_T - 0.025, Z_WALL / 2))]
    for name, size, c in walls:
        parts.append((name, m.box(size, c), "paint"))
        cols.append(m.col_box(f"{name}_c", size, c))
    jambs = [("jamb_h", (x1 - x0, JAMB_T, z_head + JAMB_T), ((x0 + x1) / 2, (yj0 + yj1) / 2, (z_head + JAMB_T) / 2)),
             ("jamb_l", (x1 - x0, JAMB_T, z_head + JAMB_T), ((x0 + x1) / 2, y_latch - JAMB_T / 2, (z_head + JAMB_T) / 2)),
             ("jamb_t", (x1 - x0, yj1 - (y_latch - JAMB_T), JAMB_T), ((x0 + x1) / 2, (yj1 + y_latch - JAMB_T) / 2, z_head + JAMB_T / 2))]
    for name, size, c in jambs:
        parts.append((name, m.box(size, c), "pine"))
        cols.append(m.col_box(f"{name}_c", size, c))
    # the door stop moulding on the head and latch jambs, behind the door (seen only)
    parts.append(("stop_t", m.box((0.0095, abs(y_latch) + GAP, 0.0095), (FACE + T_DOOR + 0.0048, (y_latch + GAP) / 2, z_head + 0.0048)), "pine"))
    parts.append(("stop_l", m.box((0.0095, 0.0095, z_head), (FACE + T_DOOR + 0.0048, y_latch + 0.0048, z_head / 2)), "pine"))
    # casing on this side: hinge leg, head, latch leg (inner edges 3/16 in back from the jambs' faces)
    xc = FACE - CASING_T / 2
    zc_top = z_head + REVEAL + CASING_W
    y_h0 = GAP + REVEAL
    y_l0 = y_latch - REVEAL
    casing = [("casing_h", (CASING_T, CASING_W, zc_top), (xc, y_h0 + CASING_W / 2, zc_top / 2)),
              ("casing_l", (CASING_T, CASING_W, zc_top), (xc, y_l0 - CASING_W / 2, zc_top / 2)),
              ("casing_t", (CASING_T, (y_h0 + CASING_W) - (y_l0 - CASING_W), CASING_W),
               (xc, (y_h0 + CASING_W + y_l0 - CASING_W) / 2, z_head + REVEAL + CASING_W / 2))]
    for name, size, c in casing:
        parts.append((name, m.box(size, c), "trim"))
        cols.append(m.col_box(f"{name}_c", size, c))
    # the top hinge's jamb leaf (in its mortise, flush with the jamb's face) and knuckles 1, 3, 5
    z_h0 = Z_K - H_HINGE
    parts.append(("leaf_j", m.box((0.0445 - 0.004, 0.00216, H_HINGE), ((0.0445 + 0.004) / 2, GAP + 0.00108, (z_h0 + Z_K) / 2)), "hinge"))
    for i in (1, 3, 5):
        z0, z1 = _knuckle_z(i)
        z0, z1 = z0 + 0.0002, z1 - (0.0 if i == 1 else 0.0002)
        prof = [(R_BORE, z0), (R_KN, z0), (R_KN, z1), (R_BORE + (CHAMFER if i == 1 else 0.0), z1)]
        parts.append((f"knuckle{i}", m.lathe(prof + [prof[0]], 32), "hinge"))
        cols += _bore(f"k{i}", R_BORE, R_KN, z0, z1, chamfer=CHAMFER if i == 1 else 0.0)
    return parts, cols


def _door_leaf():
    """The door leaf at joint zero: OPEN0 degrees open (turned clockwise seen from above, toward the robot), in the frame's axes."""
    parts, cols = [], []
    slab = m.box((T_DOOR, W_DOOR, Z_DT - 0.005), (FACE + T_DOOR / 2, -GAP - W_DOOR / 2, (Z_DT + 0.005) / 2))
    parts.append(("slab", _turned(slab, -OPEN0), "door"))
    cols.append(m.col_hull("slab_c", _turned(slab, -OPEN0)))
    leaf = m.box((0.0445 - 0.004, 0.00216, H_HINGE), ((0.0445 + 0.004) / 2, -GAP - 0.00108, Z_K - H_HINGE / 2))
    parts.append(("leaf_d", _turned(leaf, -OPEN0), "hinge"))
    for i in (2, 4):
        z0, z1 = _knuckle_z(i)
        prof = [(R_BORE + 0.0002, z0 + 0.0002), (R_KN, z0 + 0.0002), (R_KN, z1 - 0.0002), (R_BORE + 0.0002, z1 - 0.0002)]
        parts.append((f"knuckle{i}", m.lathe(prof + [prof[0]], 32), "hinge"))
    return parts, cols


M_DOOR = 10.0  # kg: a 28 x 80 in hollow-core leaf; the joint carries the whole leaf's mass and inertia
I_DOOR = (M_DOOR * (W_DOOR ** 2 + 2.032 ** 2) / 12, M_DOOR * (T_DOOR ** 2 + 2.032 ** 2) / 12, M_DOOR * (W_DOOR ** 2 + T_DOOR ** 2) / 12)  # about its centre
_fp, _fc = _frame()
_dp, _dc = _door_leaf()
LEAF = m.hinge("leaf", "Z", (0.0, 0.0, 0.0), -(170.0 - OPEN0), OPEN0, _dp, _dc, mass=M_DOOR, damping=2.0, friction=0.05, effort=50.0,
               inertia=I_DOOR, com=(FACE + T_DOOR / 2, -GAP - W_DOOR / 2, Z_DT / 2))
DOOR = m.mechanism("door", (PIN_XY[0], PIN_XY[1], TOP), [LEAF], LOOKS, frame_parts=_fp, frame_colliders=_fc, friction=(0.5, 0.4), solver_iters=64)


# -- the pin: origin at its head's underside, on its axis ---------------------------------------------------------------------------
def _pin():
    shank = m.lathe([(0.0, -L_PIN), (R_PIN - 0.0008, -L_PIN), (R_PIN, -L_PIN + 0.0012), (R_PIN, 0.0), (0.0, 0.0)], 16)
    head = m.lathe([(0.0, 0.0), (R_HEAD, 0.0), (R_HEAD, T_HEAD - 0.0008), (R_HEAD - 0.0008, T_HEAD), (0.0, T_HEAD)], 24)
    tip = m.lathe([(0.0, T_HEAD), (R_HEAD - 0.0008, T_HEAD), (R_HEAD - 0.0018, T_HEAD + 0.0012), (0.0, T_HEAD + 0.0016)], 24)
    parts = [("shank", shank, "pin"), ("head", head, "pin"), ("tip", tip, "pin")]
    cols = [m.col_hull("shank_c", shank), m.col_hull("head_c", head)]
    return parts, cols


PIN_UP = 0.040  # how far the pin has been tapped up
_pp, _pc = _pin()
PIN = m.free("pin", (PIN_XY[0], PIN_XY[1], TOP + Z_K + PIN_UP), _pp, _pc, LOOKS, mass=M_PIN, friction=(0.5, 0.4), torsional_patch=0.004,
             solver_iters=64, com=(0.0, 0.0, -L_PIN / 2))


# -- the stop: origin at the ring's underside, on its axis -------------------------------------------------------------------------
def _stop():
    y1 = BAR_W / 2
    parts, cols = [], []
    ring = m.lathe([(R_RING_IN, 0.0), (R_RING_OUT, 0.0), (R_RING_OUT, T_RING), (R_RING_IN + 0.0012, T_RING), (R_RING_IN, T_RING - 0.0012), (R_RING_IN, 0.0)], 32)
    parts.append(("ring", ring, "zinc"))
    cols += _bore("ring", R_RING_IN, R_RING_OUT, 0.0, T_RING, chamfer=0.0012)
    bar = m.box((BAR_X1 - BAR_X0, BAR_W, BAR_H), ((BAR_X1 + BAR_X0) / 2, 0.0, BAR_H / 2))
    parts.append(("bar", bar, "zinc"))
    cols.append(m.col_hull("bar_c", bar))
    # the stud arm out to the casing tip, and the tip
    arm = m.box((0.010, C_TIP[1] - y1 + 0.002, 0.005), (-0.010, (y1 + C_TIP[1]) / 2 - 0.001, 0.0025))
    parts.append(("arm", arm, "zinc"))
    cols.append(m.col_hull("arm_c", arm))
    stud = m.cylinder(0.0024, 0.012, (-0.010, C_TIP[1], Z_TIP), axis="x")
    parts.append(("stud", stud, "stud"))
    tip_c = m.cylinder(R_TIP, L_TIP, (C_TIP[0] - L_TIP / 2, C_TIP[1], Z_TIP), axis="x")
    parts.append(("tip_c", tip_c, "rubber"))
    cols.append(m.col_hull("tip_c_c", tip_c))
    boss = m.box((BOSS_X1 - BOSS_X0, BAR_W, BOSS_H), ((BOSS_X1 + BOSS_X0) / 2, 0.0, BOSS_H / 2))
    parts.append(("boss", boss, "zinc"))
    cols.append(m.col_hull("boss_c", boss))
    l_d = -BAR_W / 2 - D_TIP[1]  # the end tip stands this far out of the boss's side
    tip_d = m.cylinder(R_TIP, l_d, (D_TIP[0], D_TIP[1] + l_d / 2, Z_TIP), axis="y")
    parts.append(("tip_d", tip_d, "rubber"))
    cols.append(m.col_hull("tip_d_c", tip_d))
    return parts, cols


STOP_START = (0.345, 0.220)  # ring's centre on the work surface
STOP_YAW = 0.0  # the bar points at the robot
_sp, _sc = _stop()
STOP = m.free("stop", (STOP_START[0], STOP_START[1], TOP + 0.0003), _sp, _sc, LOOKS, mass=M_STOP, friction=(0.9, 0.8), torsional_patch=0.01,
              solver_iters=64, yaw_deg=STOP_YAW)

# -- distractors (SimReady): a mug, a notebook, a roll of tape -------------------------------------------------------------------------
MUG = SimReady("mug", pos=(0.15, -0.49, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
               usd="sm_dishware_mug_a01_01.usda", yaw=0.6)
NOTEBOOK = SimReady("notebook", pos=(0.12, 0.47, TOP), size=(0.229, 0.170, 0.010), mass=0.12, yaw=0.2,
                    package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd")
TAPE = SimReady("tape", pos=(0.33, 0.42, TOP), size=(0.10, 0.10, 0.05), mass=0.10, package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd")

OBJECTS = [DOOR, PIN, STOP, MUG, NOTEBOOK, TAPE]
DISTRACTORS = (("mug", MUG), ("notebook", NOTEBOOK), ("tape", TAPE))


# -- the scorer ----------------------------------------------------------------------------------------------------------------------
AXIS = np.array([PIN_XY[0], PIN_XY[1]])
Z_KNUCKLE = TOP + Z_K


def _seg_dist(p, a, b):
    """Distance from points p to the segment a-b."""
    ab = b - a
    t = np.clip(((p - a) @ ab) / float(ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def _box_dist(q, lo, hi):
    """Distance from local points q to an axis-aligned box."""
    d = np.maximum(np.maximum(lo - q, q - hi), 0.0)
    return np.linalg.norm(d, axis=1)


STOP_BOX = (np.array([-0.065, -0.0110, 0.0]), np.array([0.0060, 0.0225, 0.0130]))  # the stop's bounds in its own frame


def measure(pin, stop, tips=None) -> dict:
    rp, rs = pin.rotation(), stop.rotation()
    ring_top = stop.pos + rs[:, 2] * T_RING
    pin_tip = pin.pos - rp[:, 2] * L_PIN
    out = {
        "stop_off": float(np.linalg.norm(stop.pos[:2] - AXIS)),
        "stop_dz": float(stop.pos[2] - Z_KNUCKLE),
        "stop_tilt": stop.tilt_deg(),
        "stop_yaw": float(np.degrees(math.atan2(rs[1, 0], rs[0, 0]))),
        "pin_off": float(np.linalg.norm(pin.pos[:2] - AXIS)),
        "pin_tip_off": float(np.linalg.norm(pin_tip[:2] - AXIS)),
        "pin_tilt": pin.tilt_deg(),
        "pin_dz": float(pin.pos[2] - ring_top[2]),  # the head's underside above the ring's top
        "pin_depth": float(Z_KNUCKLE - pin_tip[2]),  # the pin's tip below the top knuckle's top
    }
    if tips is not None:
        d_pin = _seg_dist(tips, pin_tip, pin.pos + rp[:, 2] * (T_HEAD + 0.002)) - R_HEAD
        d_stop = _box_dist(stop.to_local(tips), *STOP_BOX)
        out["hand_gap"] = float(min(d_pin.min(), d_stop.min()))
    return out


def level(c: dict) -> float:
    seated = c["stop_off"] <= 0.0010 and -0.0005 <= c["stop_dz"] <= 0.0020 and c["stop_tilt"] <= 10.0
    if not seated:
        return 0.0
    through = c["pin_off"] <= 0.0010 and c["pin_tip_off"] <= 0.0010 and c["pin_depth"] >= 0.018
    home = through and c["pin_tilt"] <= 3.0 and -0.0005 <= c["pin_dz"] <= 0.0015
    if home:
        return 1.0
    if through:
        return 0.5
    return 0.25


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the tracker's history is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        self.prev = None

    def step(self, env):
        pin, stop = m.state(env, "pin"), m.state(env, "stop")
        c = measure(pin, stop, m.fingertips(env))
        now = np.r_[pin.pos, pin.to_world(np.array([0.0, 0.0, -L_PIN]))[0], stop.pos, stop.to_world(np.array([BAR_X0, 0.0, 0.0]))[0]]
        prev = self.prev if self.prev is not None else now
        c["still"] = bool(np.max(np.abs(now - prev)) < 0.0005)
        self.prev = now
        self.last = c
        lv = level(c)
        ok = lv == 1.0 and c["still"] and c["hand_gap"] >= 0.03
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= HOLD_STEPS:
            self.full = True
        if self.full and lv < 1.0:  # credit does not outlast the state that earned it
            self.full = False
        self.score = 1.0 if self.full else (0.75 if lv == 1.0 else lv)


F_FIT = 3.0  # N: the most the pin's snug fit in the knuckles resists sliding with


class Fit:
    """The pin's snug fit in the knuckles, as sliding friction along its axis while its tip is in the top knuckle's bore and it is on
    the hinge's axis: up to F_FIT, never more than stops it within a control step (so it holds where it is put, and is pushed or pulled
    through against F_FIT). Off the hinge, the pin is a plain rigid body."""

    def __init__(self):
        self.on = False

    def reset(self, env=None):
        """After the reset's settle (when no hook runs, so the pin slides down its bore), put the pin back where it was tapped up to,
        at rest, and hold it there by its fit."""
        self.on = False
        if env is None:
            return
        import torch

        base = env.unwrapped
        asset = base.scene["pin"]
        origin = m._np(base.scene.env_origins[0])
        q = m._np(asset.data.root_link_pose_w)[0, 3:7]
        pose = torch.tensor([[PIN.pos[0] + origin[0], PIN.pos[1] + origin[1], PIN.pos[2] + origin[2], *q]], dtype=torch.float32, device=asset.device)
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=asset.device))
        m.push(env, "pin", (0.0, 0.0, M_PIN * 9.81))
        self.on = True

    def step(self, env):
        p = m.state(env, "pin")
        ax = p.rotation()[:, 2]
        tip = p.pos - ax * L_PIN
        engaged = tip[2] < Z_KNUCKLE - 0.001 and np.linalg.norm(p.pos[:2] - AXIS) < 0.0015 and p.tilt_deg() < 5.0
        if not engaged:
            if self.on:
                m.push(env, "pin", (0.0, 0.0, 0.0))
                self.on = False
            return
        v = float(p.lin_vel @ ax)
        f = float(np.clip(M_PIN * 9.81 * ax[2] - M_PIN * v / env.unwrapped.step_dt, -F_FIT, F_FIT))
        m.push(env, "pin", tuple(float(x) for x in ax * f))
        self.on = True


FIT = Fit()
m.on_step(FIT.step)
m.on_reset(FIT.reset)
TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration ---------------------------------------------------------------------------------------------------------------
def _report(ctx, label=""):
    c = TRIAL.last
    if label.startswith("end"):
        print("[distractors] " + ", ".join(f"{n} {np.round(ctx.pos(n)[:2] - np.array(o.pos[:2]), 4).tolist()}" for n, o in DISTRACTORS), flush=True)
    if not c:
        return
    print(f"[hinge] t {ctx.t:5.1f} {label} stop off {c['stop_off'] * 1000:5.2f} dz {c['stop_dz'] * 1000:+6.2f} tilt {c['stop_tilt']:4.1f} yaw {c['stop_yaw']:+6.1f} | "
          f"pin off {c['pin_off'] * 1000:5.2f}/{c['pin_tip_off'] * 1000:5.2f} tilt {c['pin_tilt']:4.1f} dz {c['pin_dz'] * 1000:+6.1f} depth {c['pin_depth'] * 1000:+6.1f} | "
          f"door {ctx.deg('door', 'leaf'):+6.1f} | hands {c.get('hand_gap', 0) * 1000:5.0f} mm still {c['still']} | score {TRIAL.score:.2f} full {TRIAL.full}", flush=True)


def demo_script(ctx):
    """Right: takes the raised pin in its fingertips (fingers level, pointing past the pin toward the casing) and pulls it up out of the
    hinge, then holds it aside. Left: picks the stop off the work surface by the boss at its end (fingers tilted 60 degrees), sets its
    ring on the top knuckle and holds it there, centred on the hinge's axis. Right: brings the pin's tip over the ring, stands the pin
    upright, and pushes it down through the ring and into the knuckles as far as its fingers clear the stop, then lets go and moves
    away. Left lets go of the stop (the pin now holds it), closes its fingers and presses the pin's head down with their tips until it
    sits on the stop, then moves away.
    Red-team modes: none (nothing), onearm (the stop set on the knuckle and let go before the pin goes in), careless (the pin pushed
    back without the stop), placed (the stop held on the knuckle, the pin left out), held (as full, but the left hand stays on the pin)."""
    import demokit as d

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "placed", "held"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for k in range(60):
            yield
        _report(ctx, "end:")
        return
    up = np.array([0.0, 0.0, 1.0])
    axis = np.array([AXIS[0], AXIS[1], Z_KNUCKLE])

    def pin_tip():
        st = ctx.state("pin")
        return np.asarray(st.pos) - m.quat_matrix(st.quat_xyzw)[:, 2] * L_PIN

    # 1. right: pull the pin, held in the fingertips
    gp = d.level(60.0)
    ap = gp.rot[:, 2]
    TIP_IN = 0.0115  # the pin 1.5 mm in from the fingertips
    g0 = ctx.pos("pin") - up * 0.010 - ap * TIP_IN
    yield from r.move(g0 - ap * 0.06 + up * 0.01, grip=gp, finger=-0.008, t=2.5, label="right in front of the pin")
    yield from r.servo(g0 - ap * 0.06 + up * 0.01, timeout=1.0, label="right in front of the pin")
    yield from r.move(g0, t=1.2, label="right round the pin")
    yield from r.servo(g0, tol=0.0005, timeout=1.5, label="right round the pin")
    yield from r.close(2 * R_PIN, 20.0, t=0.5, label="grip the pin")
    z_p = ctx.pos("pin")[2]
    yield from r.servo(lambda t: np.array([AXIS[0], AXIS[1], z_p + min(0.075, 0.03 * t)]), read=lambda: ctx.pos("pin"), tol=0.001, timeout=3.5,
                       label="pull the pin")  # straight up its axis, so that it does not bind
    _report(ctx, "pin out:")
    yield from r.move(r.at() - ap * 0.05 + up * 0.02, t=1.0, label="pin back")
    yield from r.move(np.array([0.30, -0.16, TOP + 0.22]), t=1.5, label="pin aside")
    hold_stop = False
    gs = d.tilted(60.0, STOP_YAW)
    as_ = gs.rot[:, 2]
    if mode != "careless":
        # 2. left: pick the stop by its end boss (with the rubber tip) and set its ring on the top knuckle
        st = ctx.state("stop")
        boss = st.to_world(np.array([D_TIP[0], (BAR_W / 2 + D_TIP[1]) / 2, BOSS_H / 2]))[0]
        grip_s = boss - as_ * 0.013
        yield from l.move(grip_s - as_ * 0.06 + up * 0.02, grip=gs, finger=-0.016, t=2.5, label="left over the stop")
        yield from l.servo(grip_s - as_ * 0.06 + up * 0.02, timeout=1.0, label="left over the stop")
        yield from l.move(grip_s, t=1.2, label="left round the boss")
        yield from l.servo(grip_s, tol=0.0007, timeout=1.0, label="left round the boss")
        yield from l.close(BAR_W / 2 - D_TIP[1], 20.0, t=0.5, label="grip the stop")
        yield from l.move(l.at() + up * 0.06, t=1.0, label="lift the stop")
        off = ctx.pos("stop") - l.tcp()  # the ring from the hand
        above = axis + up * 0.025
        yield from l.move(above - off - as_ * 0.03, t=2.0, label="stop toward the hinge")
        yield from l.move(above - off, t=1.0, label="stop over the hinge")
        yield from l.servo(above, read=lambda: ctx.pos("stop"), tol=0.0005, timeout=1.5, label="stop over the hinge")
        _report(ctx, "stop over:")
        yield from l.servo(lambda t: axis + up * max(0.0005, 0.025 - 0.02 * t), read=lambda: ctx.pos("stop"), tol=0.0004, timeout=3.0, label="stop onto the knuckle")
        _report(ctx, "stop on:")
        hold_stop = mode != "onearm"
        if not hold_stop:
            yield from l.open(t=0.5, label="let go of the stop")
            yield from l.move(l.at() - as_ * 0.06 + up * 0.03, t=1.0, label="left back")
            yield from l.wait(1.0)
            _report(ctx, "stop let go:")
        if mode == "placed":
            for k in range(60):
                yield
                if k % 20 == 19:
                    _report(ctx, "end:")
            return
    # 3. right: the pin's tip over the ring (or the bare knuckle), upright, then down through the ring and the knuckles
    z_entry = Z_KNUCKLE + T_RING + 0.004
    target = np.array([AXIS[0], AXIS[1], z_entry])
    yield from r.move(r.at() + (target - pin_tip()) + up * 0.03 - ap * 0.03, t=2.0, label="pin toward the ring")
    yield from r.move(r.at() + (target - pin_tip()) + up * 0.01, t=1.2, label="pin over the ring")
    yield from r.servo(target, read=pin_tip, tol=0.0004, timeout=2.0, label="pin over the ring")
    for _ in range(3):  # stand the pin upright: turn the hand about the tilt's axis, through the grip
        v = m.quat_matrix(ctx.state("pin").quat_xyzw)[:, 2]
        k = np.cross(v, up)
        ang = math.degrees(math.asin(min(1.0, float(np.linalg.norm(k)))))
        if ang < 0.2:
            break
        yield from r.turn(ang, axis=k / np.linalg.norm(k), t=max(0.4, ang / 20.0), label="stand the pin upright")
        yield from r.servo(target, read=pin_tip, tol=0.0004, timeout=1.5, label="pin over the ring")
    _report(ctx, "pin over:")
    depth_goal = 0.045  # as far as the hand clears the stop and stays in reach
    depth = lambda: Z_KNUCKLE - pin_tip()[2]
    stuck = 0
    for _ in range(80):  # push down in 2 mm strokes, keeping the pin over the axis; if it stops going in, back off, re-centre, go again
        d0 = depth()
        if d0 >= depth_goal or stuck > 6:
            break
        head_xy = ctx.pos("pin")[:2]
        lead = pin_tip()[:2] if d0 < -T_RING + 0.0005 else head_xy  # the tip until it is in the ring, then the head
        fix = np.r_[(AXIS - lead) * 0.6, 0.0]
        yield from r.move(r.at() + fix - up * min(0.002, depth_goal - d0 + 0.0005), t=0.25, label="pin down")
        yield from r.wait(0.1)
        if depth() - d0 < 0.0004:
            stuck += 1
            yield from r.move(r.at() + up * 0.003, t=0.3, label="ease the pin up")
            yield from r.servo(np.r_[AXIS, pin_tip()[2]], read=pin_tip, tol=0.0003, timeout=1.0, label="re-centre the pin")
    _report(ctx, "pin down:")
    yield from r.open(t=0.4, label="let go of the pin")
    yield from r.move(r.at() + up * 0.03, t=0.8, label="right up")
    yield from r.move(r.at() - ap * 0.05, t=0.8, label="right back")
    yield from r.move(np.array([0.28, -0.15, TOP + 0.22]), t=1.5, label="right away")
    # 4. left: let go of the stop, close, and press the head home with the fingertips
    if hold_stop:
        yield from l.open(t=0.5, label="let go of the stop")
        yield from l.move(l.at() - as_ * 0.04 + up * 0.03, t=1.0, label="left back")
    else:
        yield from l.move(np.array([AXIS[0] - 0.08, AXIS[1], Z_KNUCKLE + 0.05]), grip=gs, t=2.0, label="left over")
    yield from l.close(None, t=0.4, label="close")
    head = lambda: ctx.pos("pin") + up * (T_HEAD + 0.0016)
    w = np.cross(as_, np.cross(up, as_) / np.linalg.norm(np.cross(up, as_)))
    low = -w if w[2] > 0 else w
    edge = lambda: l.tcp() + as_ * 0.0135 + low * 0.005  # the lowest point of the closed fingertips
    yield from l.move(l.at() + (head() + up * 0.008 - edge()), t=1.2, label="tips over the head")
    yield from l.servo(lambda t: head() + up * 0.008, read=edge, tol=0.0008, timeout=1.0, label="tips over the head")
    z0 = head()[2]
    seat = Z_KNUCKLE + (T_RING if mode != "careless" else 0.0) + T_HEAD + 0.0016
    yield from l.servo(lambda t: np.array([AXIS[0], AXIS[1], max(seat - 0.0015, z0 + 0.008 - 0.015 * t)]), read=edge, tol=0.0008, timeout=6.0,
                       label="press the pin home")
    _report(ctx, "pressed:")
    if mode == "held":
        for k in range(60):
            yield
            if k % 20 == 19:
                _report(ctx, "end:")
        return
    yield from l.move(l.at() + up * 0.02 - as_ * 0.03, t=0.8, label="left off the pin")
    yield from l.move(np.array([0.20, 0.32, TOP + 0.20]), grip=d.down(0.0), t=1.5, label="left away")
    _report(ctx, "left clear:")
    for k in range(60):
        yield
        if k % 20 == 19:
            _report(ctx, "end:")
