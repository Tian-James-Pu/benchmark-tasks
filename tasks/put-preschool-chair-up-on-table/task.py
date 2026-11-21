"""End-of-day "chairs up" in a preschool classroom: put a 12 in stacking chair up on its 20 in activity table, upside down
with its seat flat on the top, so the floor can be mopped.

The station table stands for the classroom floor. On it stand a Correll 24 x 48 in adjustable activity table, its legs set
at 20 in, its long side across the arms, and, between the arms and the table, a Regency Andy 12 in stack chair (model
4500), facing the arms with its back towards the table. The chair is a polypropylene seat-and-back shell on a tubular
steel frame: a 12 x 12 in seat at 12 in, with a 22 mm rolled rim under its front and side edges, a back rising 10 in
above the seat and raked 10 degrees, and four splayed steel legs on nylon glides; 22 x 14 x 16 in overall, 6 lb. A
notebook, a paper cup and a plastic crayon basket sit at the table's far side.

The chair is one rigid body (free, with its centre of mass from its parts). The table is fixed. Nothing is random.
Scoring is read each control step by a tracker (mechkit hooks): see SCORING below and task.json.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

from dataclasses import replace

from yam_arena import demo as _demo
from yam_arena import station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # the classroom floor
IN = 0.0254


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)

# The top camera is moved up and back so that the 20 in table and the chair turned over above it stay in view; the station
# builds the camera's body and stand from this same rig, for every policy.
CAMERA_POS = (-0.32, -0.62, 1.85)
CAMERA_TARGET = (0.45, 0.0, 1.02)
_base_rig = getattr(station.rig, "_chair_up_base", station.rig)


def _chair_up_rig(top_cam="calibrated"):
    rig = _base_rig(top_cam)
    forward = _unit(np.asarray(CAMERA_TARGET) - np.asarray(CAMERA_POS))
    right = _unit(np.cross(forward, (0.0, 0.0, 1.0)))
    up = np.cross(right, forward)
    rot = np.column_stack((right, up, -forward))
    camera = replace(rig.top_cam, pos=CAMERA_POS, rot_xyzw=tuple(station.matrix_to_quat_xyzw(rot)), convention="opengl")
    return replace(rig, name="station-chair-up", top_cam=camera)


_chair_up_rig._chair_up_base = _base_rig
station.rig = _chair_up_rig

# -- the chair: Regency Andy 4500, in its own frame: origin at the middle of the seat's top face, +x towards the back
# (rear), +y to the chair's left, +z up. The front faces -x. --------------------------------------------------------
SEAT_W = SEAT_D = 12 * IN  # 0.3048: seat 12 x 12 in (maker's listing)
SEAT_H = 12 * IN  # seat height 12 in
SHELL_T = 0.006  # polypropylene shell thickness (estimate)
RIM_D, RIM_T = 0.028, 0.005  # the rolled rim under the front and side edges: 28 mm deep below the top face, 5 mm wall
BACK_RISE, BACK_RAKE, BACK_W = 10 * IN, 10.0, 0.290  # the back: 10 in above the seat (22 in overall), raked 10 deg
TUBE_R = 0.0095  # 3/4 in steel tube (15 gauge, per the dealer's listing; the diameter is our estimate)
GLIDE_R, GLIDE_H = 0.011, 0.012
LEG_TOP_X, LEG_TOP_Y, LEG_TOP_Z = 0.115, 0.090, -0.022  # where the legs meet the under-seat rails
FOOT_FRONT_X, FOOT_REAR_X, FOOT_Y = -0.192, 0.175, 0.167  # glide centres: 14 in wide, 16 in deep overall with the back
FLOOR_Z = -SEAT_H  # the floor in the chair frame
HX = SEAT_D / 2

LOOKS = {
    "shell": m.Look(m.rgb(0.13, 0.30, 0.62), roughness=0.5),  # blue polypropylene
    "chrome": m.Look(m.rgb(0.78, 0.79, 0.81), metallic=0.9, roughness=0.25),
    "glide": m.Look(m.rgb(0.12, 0.12, 0.13), roughness=0.8),
    "slot": m.Look(m.rgb(0.05, 0.05, 0.06), roughness=0.9),
    "top": m.Look(m.rgb(0.86, 0.84, 0.78), roughness=0.55),  # laminate
    "tmold": m.Look(m.rgb(0.20, 0.45, 0.25), roughness=0.6),  # green T-mold edge
    "leg": m.Look(m.rgb(0.72, 0.73, 0.75), metallic=0.8, roughness=0.3),
}


def _rod(a, b, r, segments=16):
    """A cylinder mesh from point a to point b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = float(np.linalg.norm(d))
    z = d / L
    x = _unit(np.cross(z, [0.0, 0.0, 1.0]) if abs(z[2]) < 0.95 else np.cross(z, [1.0, 0.0, 0.0]))
    R = np.column_stack([x, np.cross(z, x), z])
    c = m.cylinder(r, L, (0, 0, L / 2), segments=segments)
    return m.Mesh(c.v @ R.T + a, c.f)


def _back_mesh(thick=SHELL_T, width=BACK_W, inset=0.0):
    L = BACK_RISE / math.cos(math.radians(BACK_RAKE))
    b = m.box((thick, width, L), (0, 0, L / 2)).turned(BACK_RAKE, "y")
    return b.moved((HX + thick / 2 + inset, 0.0, 0.0))


LEGS = {
    "front_left": ((-LEG_TOP_X, LEG_TOP_Y, LEG_TOP_Z), (FOOT_FRONT_X, FOOT_Y, FLOOR_Z + GLIDE_H)),
    "front_right": ((-LEG_TOP_X, -LEG_TOP_Y, LEG_TOP_Z), (FOOT_FRONT_X, -FOOT_Y, FLOOR_Z + GLIDE_H)),
    "rear_left": ((LEG_TOP_X, LEG_TOP_Y, LEG_TOP_Z), (FOOT_REAR_X, FOOT_Y, FLOOR_Z + GLIDE_H)),
    "rear_right": ((LEG_TOP_X, -LEG_TOP_Y, LEG_TOP_Z), (FOOT_REAR_X, -FOOT_Y, FLOOR_Z + GLIDE_H)),
}
RAILS = {s: ((-LEG_TOP_X - 0.01, s * LEG_TOP_Y, LEG_TOP_Z), (LEG_TOP_X + 0.01, s * LEG_TOP_Y, LEG_TOP_Z)) for s in (1, -1)}

# boxes in the chair frame: (size, centre) for the seat plate and its rim
SEAT_BOX = ((SEAT_D, SEAT_W, SHELL_T), (0.0, 0.0, -SHELL_T / 2))
RIM_BOXES = {
    "rim_left": ((SEAT_D, RIM_T, RIM_D - SHELL_T), (0.0, SEAT_W / 2 - RIM_T / 2, -(RIM_D + SHELL_T) / 2)),
    "rim_right": ((SEAT_D, RIM_T, RIM_D - SHELL_T), (0.0, -SEAT_W / 2 + RIM_T / 2, -(RIM_D + SHELL_T) / 2)),
    "rim_front": ((RIM_T, SEAT_W - 2 * RIM_T, RIM_D - SHELL_T), (-HX + RIM_T / 2, 0.0, -(RIM_D + SHELL_T) / 2)),
}

# masses (kg): 6 lb in all (2.72 kg). The split between shell and frame is our estimate (steel tube at 0.78 kg/m).
M_SEAT, M_BACK, M_LEG, M_RAIL, M_GLIDE = 0.76, 0.62, 0.235, 0.18, 0.01
CHAIR_MASS = M_SEAT + M_BACK + 4 * M_LEG + 2 * M_RAIL + 4 * M_GLIDE  # 2.72


def _chair_com():
    parts = [(M_SEAT, np.array([0.0, 0.0, -0.007])), (M_BACK, _back_mesh().v.mean(axis=0))]
    for top, foot in LEGS.values():
        parts.append((M_LEG, (np.array(top) + np.array(foot)) / 2))
        parts.append((M_GLIDE, np.array([foot[0], foot[1], FLOOR_Z + GLIDE_H / 2])))
    for a, b in RAILS.values():
        parts.append((M_RAIL, (np.array(a) + np.array(b)) / 2))
    w = sum(p[0] for p in parts)
    return sum(p[0] * p[1] for p in parts) / w


CHAIR_COM = _chair_com()


def _chair_parts():
    parts, cols = [], []
    seat = m.box(*SEAT_BOX)
    parts.append(("seat", seat, "shell"))
    cols.append(m.col_box("seat_c", *SEAT_BOX))
    for n, (size, c) in RIM_BOXES.items():
        parts.append((n, m.box(size, c), "shell"))
        cols.append(m.col_box(f"{n}_c", size, c))
    back = _back_mesh()
    parts.append(("back", back, "shell"))
    cols.append(m.col_hull("back_c", back))
    # the junction between seat and back, and the carry slot near the back's top (seen from both faces)
    parts.append(("bend", _rod((HX, -BACK_W / 2, -0.001), (HX, BACK_W / 2, -0.001), 0.0045), "shell"))
    for k, side in enumerate((-1, 1)):
        slot = m.box((0.0012, 0.10, 0.025), (0, 0, BACK_RISE / math.cos(math.radians(BACK_RAKE)) - 0.045))
        slot = slot.turned(BACK_RAKE, "y").moved((HX + SHELL_T / 2 + side * (SHELL_T / 2 + 0.0004), 0.0, 0.0))
        parts.append((f"slot{k}", slot, "slot"))
    for n, (top, foot) in LEGS.items():
        rod = _rod(top, foot, TUBE_R)
        parts.append((n, rod, "chrome"))
        cols.append(m.col_hull(f"{n}_c", _rod(top, foot, TUBE_R, segments=10)))
        g = (foot[0], foot[1], FLOOR_Z + GLIDE_H / 2)
        parts.append((f"{n}_glide", m.cylinder(GLIDE_R, GLIDE_H, g, segments=20), "glide"))
        cols.append(m.col_cylinder(f"{n}_glide_c", GLIDE_R, GLIDE_H, g))
    for s, (a, b) in RAILS.items():
        nm = "rail_left" if s > 0 else "rail_right"
        parts.append((nm, _rod(a, b, TUBE_R), "chrome"))
        cols.append(m.col_hull(f"{nm}_c", _rod(a, b, TUBE_R, segments=10)))
        for x in (-0.07, 0.07):  # mounting blocks between rail and shell
            parts.append((f"{nm}_mount{int(x > 0)}", m.box((0.02, 0.02, 0.012), (x, s * LEG_TOP_Y, -0.0115)), "glide"))
    return parts, cols


# -- the table: Correll 24 x 48 in activity table (A2448-REC family), legs at 20 in, 1 1/4 in top -------------------------
TABLE_H = 20 * IN  # 0.508: the top face above the floor
TABLE_D, TABLE_W, TABLE_T = 24 * IN, 48 * IN, 1.25 * IN
X_EDGE = 0.400  # the table's near edge (x); it spans x X_EDGE .. X_EDGE + 0.61 and y -0.61 .. 0.61
TABLE_Z = TOP + TABLE_H
TLEG = 0.032  # square leg (estimate)


def _table_parts():
    cx = X_EDGE + TABLE_D / 2
    top = ((TABLE_D, TABLE_W, TABLE_T), (cx, 0.0, TABLE_H - TABLE_T / 2))
    parts = [("top", m.box(*top), "top")]
    cols = [m.col_box("top_c", *top)]
    parts.append(("tmold_near", m.box((0.003, TABLE_W, TABLE_T), (X_EDGE - 0.0015, 0.0, TABLE_H - TABLE_T / 2)), "tmold"))
    parts.append(("tmold_far", m.box((0.003, TABLE_W, TABLE_T), (X_EDGE + TABLE_D + 0.0015, 0.0, TABLE_H - TABLE_T / 2)), "tmold"))
    for sy, nm in ((1, "l"), (-1, "r")):
        parts.append((f"tmold_{nm}", m.box((TABLE_D, 0.003, TABLE_T), (cx, sy * (TABLE_W / 2 + 0.0015), TABLE_H - TABLE_T / 2)), "tmold"))
    for k, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        lx, ly = cx + sx * (TABLE_D / 2 - 0.06), sy * (TABLE_W / 2 - 0.08)
        leg = ((TLEG, TLEG, TABLE_H - TABLE_T), (lx, ly, (TABLE_H - TABLE_T) / 2))
        parts.append((f"leg{k}", m.box(*leg), "leg"))
        cols.append(m.col_box(f"leg{k}_c", *leg))
    return parts, cols


# -- layout --------------------------------------------------------------------------------------------------------------
GRIP_X = 0.100  # where the hands take the seat's side edges: 10 cm behind the seat's middle (chair frame x)
GRIP_Z = -RIM_D / 2  # the hands' centre line: half-way between the seat's top face and the rim's lower edge
HAND_Y = 0.122  # the hands' TCPs, either side of the midline (3 cm in from the seat's side edges)
CHAIR_GAP = 0.12  # the back's top stands 12 cm from the table's near edge
CHAIR_X0 = X_EDGE - CHAIR_GAP - (HX + SHELL_T + BACK_RISE * math.tan(math.radians(BACK_RAKE)))
CHAIR_POS = (CHAIR_X0, 0.0, TOP + SEAT_H)

_cp, _cc = _chair_parts()
_tp, _tc = _table_parts()
OBJECTS = [
    m.free("chair", CHAIR_POS, _cp, _cc, LOOKS, mass=CHAIR_MASS, friction=(0.9, 0.8), solver_iters=48,
           com=tuple(float(v) for v in CHAIR_COM)),
    m.fixed("activity_table", (0.0, 0.0, TOP), _tp, _tc, LOOKS, friction=(0.9, 0.8)),
    # distractors (SimReady props) on the table's far side; scoring ignores them
    SimReady("notebook", pos=(0.84, 0.36, TABLE_Z), size=(0.229, 0.170, 0.010), mass=0.12, yaw=0.3,
             package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd"),
    SimReady("cup", pos=(0.88, 0.10, TABLE_Z), size=(0.080, 0.080, 0.110), mass=0.01,
             package="dishware_cup_paper_a01", usd="sm_dishware_cup_paper_a01_01.usd"),
    SimReady("basket", pos=(0.86, -0.30, TABLE_Z), size=(0.201, 0.271, 0.114), mass=0.15,
             package="misc_basket_plastic_a01", usd="sm_misc_basket_plastic_a01_01.usd"),
]

# -- scoring -------------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 1.0  # on top of the tracker's own 2 s hold
HOLD_FULL, HOLD_PART = 40, 20  # control steps at 20 Hz
TOUCH, CLEAR = 0.012, 0.03  # a fingertip point this close to the chair holds it; this far from it is clear
DROP_GAP = 0.015  # let go with every part of it more than 1.5 cm above what is under it: dropped
LIFT_GAP = 0.05
LEG_STRIKE = 0.003
OVERHANG_MAX = 0.04
FLAT_DEG = 5.0
TURNED_DEG = 30.0
UPSIDE_DEG = 60.0
REST_V, REST_W = 0.01, 0.05


def _chair_points():
    """Points on the chair (chair frame): seat-face and rim corners, the back's corners, and along every leg and rail."""
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts.append((sx * HX, sy * SEAT_W / 2, 0.0))
            pts.append((sx * HX, sy * SEAT_W / 2, -RIM_D))
    for v in _back_mesh().v:
        pts.append(tuple(v))
    for top, foot in list(LEGS.values()) + list(RAILS.values()):
        for s in np.linspace(0.0, 1.0, 12):
            pts.append(tuple(np.array(top) + s * (np.array(foot) - np.array(top))))
    for top, foot in LEGS.values():
        pts.append((foot[0], foot[1], FLOOR_Z))
    return np.unique(np.round(np.array(pts), 6), axis=0)


CHAIR_PTS = _chair_points()
SEAT_FACE = np.array([(sx * HX, sy * SEAT_W / 2, 0.0) for sx in (-1, 1) for sy in (-1, 1)])
LEG_PTS = np.vstack([np.array(t) + s[:, None] * (np.array(f) - np.array(t)) for t, f in LEGS.values()
                     for s in [np.linspace(0.0, 1.0, 40)]] + [np.array([[f[0], f[1], FLOOR_Z + 0.002]]) for _, f in LEGS.values()])


def _seg_dist(p, a, b):
    a, b = np.asarray(a), np.asarray(b)
    d = b - a
    t = np.clip(((p - a) @ d) / (d @ d), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * d), axis=1)


def _box_dist(p, size, centre):
    return np.linalg.norm(np.maximum(np.abs(p - np.asarray(centre)) - np.asarray(size) / 2, 0.0), axis=1)


_BACK_R = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(BACK_RAKE))
_BACK_L = BACK_RISE / math.cos(math.radians(BACK_RAKE))
_BACK_C = np.array([HX + SHELL_T / 2, 0.0, 0.0]) + _BACK_R @ np.array([0.0, 0.0, _BACK_L / 2])


def chair_distance(local):
    """Distance (m) from points in the chair frame to the chair's surfaces (seat, rim, back, legs, rails)."""
    d = _box_dist(local, *SEAT_BOX)
    for size, c in RIM_BOXES.values():
        d = np.minimum(d, _box_dist(local, size, c))
    d = np.minimum(d, _box_dist((local - _BACK_C) @ _BACK_R, (SHELL_T, BACK_W, _BACK_L), (0, 0, 0)))
    for a, b in list(LEGS.values()) + list(RAILS.values()):
        d = np.minimum(d, np.maximum(_seg_dist(local, a, b) - TUBE_R, 0.0))
    return d


def surface_under(p):
    """Height of what is under each world point: the tabletop if over it (and not below it), else the floor."""
    over = (p[:, 0] >= X_EDGE) & (p[:, 0] <= X_EDGE + TABLE_D) & (np.abs(p[:, 1]) <= TABLE_W / 2) & (p[:, 2] >= TABLE_Z - 0.01)
    return np.where(over, TABLE_Z, TOP)


def outside_table(p):
    """Horizontal distance of each world point outside the tabletop's outline (0 if over it)."""
    dx = np.maximum(np.maximum(X_EDGE - p[:, 0], p[:, 0] - (X_EDGE + TABLE_D)), 0.0)
    dy = np.maximum(np.abs(p[:, 1]) - TABLE_W / 2, 0.0)
    return np.hypot(dx, dy)


def table_top_distance(p):
    return _box_dist(p, (TABLE_D, TABLE_W, TABLE_T), (X_EDGE + TABLE_D / 2, 0.0, TABLE_Z - TABLE_T / 2))


class Trial:
    """Per-step tracker. Recorded harm (a drop, a leg striking the table) is cleared only by reset()."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):  # the ONLY place a recorded harm or achievement is cleared
        self.dropped = self.struck = False
        self.lifted = self.turned = False
        self.hold_part = self.hold_full = 0
        self.full = False
        self.score = 0.0
        self.why = ""

    def update(self, st, tips):
        R = st.rotation()
        world = st.to_world(CHAIR_PTS)
        gaps = world[:, 2] - surface_under(world)
        gap = float(gaps.min())
        dist = chair_distance(st.to_local(tips)) if tips is not None else np.full(1, 1.0)
        held = bool(dist.min() <= TOUCH)
        clear = bool(dist.min() >= CLEAR)
        up_z = float(R[2, 2])  # the chair's up axis, world z component: +1 upright, -1 upside down
        legs = st.to_world(LEG_PTS)
        if float((table_top_distance(legs) - TUBE_R).min()) <= LEG_STRIKE:
            self.struck = True
        if not held and gap > DROP_GAP:
            self.dropped = True
        airborne = gap >= LIFT_GAP
        if held and airborne:
            self.lifted = True
            if up_z <= -math.cos(math.radians(TURNED_DEG)):
                self.turned = True
        still = float(np.linalg.norm(st.lin_vel)) < REST_V and float(np.linalg.norm(st.ang_vel)) < REST_W
        on_floor = bool(((world[:, 2] - TOP) < 0.02).any())
        on_table = bool((table_top_distance(world) < 0.005).any()) and not on_floor
        upside = up_z <= -math.cos(math.radians(UPSIDE_DEG))
        part_ok = still and clear and on_table and upside and gap <= 0.005
        face = st.to_world(SEAT_FACE)
        flat = up_z <= -math.cos(math.radians(FLAT_DEG))
        seated = bool((np.abs(face[:, 2] - TABLE_Z) <= 0.005).all())
        overhang = float(outside_table(face).max())
        full_ok = part_ok and self.turned and flat and seated and overhang <= OVERHANG_MAX
        self.hold_part = self.hold_part + 1 if part_ok else 0
        self.hold_full = self.hold_full + 1 if full_ok else 0
        harm = self.dropped or self.struck
        self.full = (not harm) and self.hold_full >= HOLD_FULL
        if harm:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif self.hold_part >= HOLD_PART and self.turned:
            self.score = 0.75
        elif self.turned:
            self.score = 0.5
        elif self.lifted:
            self.score = 0.25
        else:
            self.score = 0.0
        self.why = (f"gap={gap:.3f} held={held} clear={clear} up={up_z:.3f} on_table={on_table} flat={flat} "
                    f"seated={seated} overhang={overhang:.3f} still={still} dropped={self.dropped} struck={self.struck}")


TRIAL = Trial()


@m.on_reset
def _reset(environment=None):
    TRIAL.reset()


@m.on_step
def _step(environment):
    TRIAL.update(m.state(environment, "chair"), m.fingertips(environment))


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full)


# -- scripted demo -------------------------------------------------------------------------------------------------------
# Hand orientations are set through a float subclass carrying a rotation matrix (yam_arena's Waypoint takes a yaw).
_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)
    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


class Orient(float):
    rot = np.eye(3)


def _frame(finger, closing):
    f = _unit(finger)
    c = np.asarray(closing, float)
    c = _unit(c - f * (c @ f))
    return np.column_stack([c, np.cross(f, c), f])


def hand(arm, theta_deg=0.0, yaw_deg=0.0, roll_deg=0.0):
    """A hand's TCP orientation: fingers pointing in across the seat (left arm -y, right arm +y), closing vertically,
    the chair turned theta about +y, the hand then turned yaw about the vertical and rolled about its fingers."""
    s = 1.0 if arm == "left" else -1.0
    r0 = _frame((0.0, -s, 0.0), (0.0, 0.0, -s))
    r = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(theta_deg)) @ r0
    if roll_deg:
        r = r @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(roll_deg))
    if yaw_deg:
        r = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(s * yaw_deg)) @ r
    o = Orient(0.0)
    o.rot = r
    return o


def grip_world(x, z, theta_deg):
    """The chair's pose when the hands' centre line is at (x, z) and the chair has turned theta about +y."""
    R = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(theta_deg))
    pos = np.array([x, 0.0, z]) - R @ np.array([GRIP_X, 0.0, GRIP_Z])
    return R, pos


X_G0 = CHAIR_X0 + GRIP_X  # the hands' x at the grasp
Z_G0 = TOP + SEAT_H + GRIP_Z
X_SET = X_EDGE - 0.045  # the hands' x when the chair is set on the table (seat 9.5 cm over the near edge, hands clear of it)
Z_SET = TABLE_Z - GRIP_Z  # seat face on the top
PUSH_Z = TABLE_Z - 0.03  # the hands push the back 3 cm below the top
X_BEHIND = X_SET - 0.12  # behind the hanging back
X_PUSHED = X_EDGE - 0.040  # the hands' x once the chair is home

# (duration, hands x, hands z, chair turn theta, finger) for the carry; both hands mirror each other
FLIP_PATH = [
    (2.0, X_G0, TOP + 0.42, 0.0),
    (1.5, X_G0, TOP + 0.42, 0.0),
    (2.5, 0.10, TOP + 0.42, 40.0),
    (2.5, 0.10, TOP + 0.42, 90.0),
    (2.5, 0.11, TOP + 0.47, 130.0),
    (1.5, 0.11, TOP + 0.47, 130.0),
    (2.5, 0.22, TOP + 0.58, 130.0),
    (2.5, X_SET - 0.025, TOP + 0.58, 130.0),
    (1.5, X_SET - 0.025, TOP + 0.58, 130.0),
    (2.5, X_SET - 0.025, Z_SET + 0.03, 165.0),
    (2.0, X_SET - 0.025, Z_SET - 0.01, 180.0),
    (1.0, X_SET - 0.025, Z_SET - 0.01, 180.0),
]


def _arm_plan(arm):
    s = 1.0 if arm == "left" else -1.0
    y = s * HAND_Y
    out = s * 0.175
    w = [
        Waypoint(2.0, (X_G0 - 0.01, out, Z_G0 + 0.06), FINGER_OPEN, "out by the seat", hand(arm, 0.0, 30.0)),
        Waypoint(1.5, (X_G0 - 0.01, out, Z_G0), FINGER_OPEN, "level with the seat edge", hand(arm, 0.0, 30.0)),
        Waypoint(2.0, (X_G0, y, Z_G0), FINGER_OPEN, "in over the edge", hand(arm, 0.0)),
        Waypoint(1.0, (X_G0, y, Z_G0), FINGER_CLOSED, "grip", hand(arm, 0.0)),
        Waypoint(0.5, (X_G0, y, Z_G0), FINGER_CLOSED, "settle", hand(arm, 0.0)),
    ]
    for d, x, z, th in FLIP_PATH:
        w.append(Waypoint(d, (x, y, z), FINGER_CLOSED, f"carry {th:.0f}", hand(arm, th)))
    w += [
        Waypoint(1.0, (X_SET - 0.025, y, Z_SET - 0.01), FINGER_OPEN, "let go", hand(arm, 180.0)),
        Waypoint(1.5, (X_SET - 0.03, s * 0.150, Z_SET - 0.01), FINGER_OPEN, "turn out", hand(arm, 180.0, 30.0)),
        Waypoint(1.5, (X_SET - 0.035, out, Z_SET - 0.01), FINGER_OPEN, "out from the seat", hand(arm, 180.0, 30.0)),
        Waypoint(2.0, (X_BEHIND, out, PUSH_Z), FINGER_CLOSED, "round the back", hand(arm, 180.0, 30.0)),
        Waypoint(1.5, (X_BEHIND, y, PUSH_Z), FINGER_CLOSED, "behind the back", hand(arm, 180.0)),
        Waypoint(4.0, (X_PUSHED, y, PUSH_Z), FINGER_CLOSED, "push it home", hand(arm, 180.0)),
        Waypoint(1.0, (X_PUSHED, y, PUSH_Z), FINGER_CLOSED, "hold", hand(arm, 180.0)),
        Waypoint(1.5, (X_BEHIND, y, PUSH_Z), FINGER_CLOSED, "back off", hand(arm, 180.0)),
        Waypoint(2.0, (0.17, s * 0.17, PUSH_Z - 0.08), FINGER_CLOSED, "away", hand(arm, 180.0, 30.0)),
        Waypoint(5.0, (0.17, s * 0.17, PUSH_Z - 0.08), FINGER_CLOSED, "wait", hand(arm, 180.0, 30.0)),
    ]
    return w


def demo():
    return {"left": _arm_plan("left"), "right": _arm_plan("right")}
