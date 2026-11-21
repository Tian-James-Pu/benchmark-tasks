"""Take the pin out of a closed interior door's bottom hinge: tap it up from below with a nail set and a claw hammer,
then pull it out by its head and put it down.

The floor is the station table top (z 0.7475): the wall, the door and everything on them are full size and stand on it.
World frame, metres: x forward from the arm bases, y to the robot's left, z up.

- doorway (one fixed part): a painted drywall partition (4-1/2 in. thick) facing the robot, its face at x 0.4700, with a
  30 x 80 in. door opening whose hinge jamb is at y 0. The door is a 30 x 80 in. (762 x 2032 mm), 1-3/8 in. (35 mm)
  hollow-core flush door slab, closed and latched, 1/2 in. (12.7 mm) above the floor, its swing face flush with the
  wall face, hung on three 3-1/2 x 3-1/2 in. five-knuckle steel butt hinges (Hager RC1842 class: 0.085 in. leaves,
  0.234 in. = 5.94 mm loose pin, button tips). The bottom hinge's barrel (88.9 mm, 10.8 mm across) is the only one in
  reach: its axis at x 0.4541, y 0.0, 15.9 mm in front of the door face, from 292.1 mm to 381.0 mm above the floor.
  The hinge-side casing (2-1/4 x 11/16 in. pine, painted) has a 3/16 in. reveal: its edge is 6.4 mm from the pin's axis
  and its face 1.6 mm in front of it, so the barrel sits beside it. A 3-1/2 in. baseboard runs along the wall.
  The door, held by its latch and two upper hinges, does not move: it is part of the fixed doorway.
- pin (loose part): the bottom hinge's pin, 5.94 x 88.9 mm steel with a 10.8 x 4.8 mm button head, 23 g, its lower end
  flush with the bottom of the lowest knuckle, its head down on the top knuckle. It is painted in and loaded sideways by
  the door's weight (see "the bound pin" below).
- nail set (loose part): a 3/32 in. nail set, 4 in. (102 mm) long, 3/8 in. (9.5 mm) knurled body tapering over 25 mm to
  a 2.4 mm cupped tip, 50 g, lying on the floor at the front right.
- hammer (SimReady tool_hammer_a02): a 16 oz claw hammer, 304 mm, 0.567 kg, lying on its side at the front left.
- distractors (loose SimReady props the scorer ignores): a cotton rag, a can of multi-purpose oil, a utility knife.

The bound pin (the model). Until it has been driven up LOOSE (4 mm), the pin is held where it is (its pose is written
back after every 240 Hz physics step): no pull, push or lever on it moves it. It moves only when struck through the nail
set: a blow counts when the hammer's face meets the nail set's head moving toward it along the nail set's axis at
V_MIN (0.3 m/s) or more (a new blow once the face has drawn 5 mm back), with the nail set's tip on the pin's lower end
(within 2.0 mm of the pin's axis, between 1.0 mm into and 1.5 mm below its end) and its axis within 20 deg of the pin's.
Each blow raises the pin by E / F_BIND, E = 0.5 m v^2 of the hammer (m 0.567 kg, v that closing speed) and F_BIND = 60 N
the bond of the paint and the door's weight. Once it is up 4 mm it is loose: from then on it is an ordinary rigid body
in the knuckles' 6.1 mm bore, pressed sideways toward the jamb by F_SIDE (2 N, the door's weight on it), so friction
keeps it where it is left and a pull of a few newtons draws it out. That sideways load stops once its lower end has
left the top knuckle.

Damage: any part of the hammer touching the door, the casing, the wall, the baseboard or the hinge while moving at
0.10 m/s or more is a blow on the door or its trim (a hinge-pin-tool patent: "It is also easy to miss the nail and strike
the molding around the doorframe, or even strike the door itself"). It is recorded and cleared only by mechkit's
on_reset hook.

Score (task.json "scoring" says the same); r is how far the pin's head stands above the top knuckle:
- A blow on the door or its trim caps the score at 0.25 for the rest of the run.
- 1.0 (success, held 3 s): the pin out of the hinge and lying on the floor (its middle within 12 mm of it) at least 3 cm
  from the hinge's axis, at rest, every fingertip of both grippers at least 5 cm from it, no blow on the door or trim.
- 0.75: the pin out of all five knuckles (its lower end at least 2 mm above the top knuckle, or 15 mm off the axis).
- 0.5: r >= 30 mm.
- 0.25: r >= 4 mm (the pin broken loose).
- 0: otherwise (doing nothing leaves r = 0).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402


def _allow_matrix_grips() -> None:
    """Let a demo Waypoint's ``yaw`` be a full 3x3 TCP rotation (world frame), for the level grips the nail set, the
    hammer and the pin need (the same wrapper as the accepted nail-driving task's bodykit.allow_matrix_grips)."""
    from yam_arena import demo

    if getattr(demo, "_matrix_grips", False):
        return
    plain = demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    demo.grip_rotation = grip_rotation
    demo._matrix_grips = True


_allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 3.0
DEBUG = os.environ.get("HINGE_DEBUG") == "1"
MODE = os.environ.get("HINGE_MODE", "full")  # red-team variants: none, pull, straight, onearm

IN = 0.0254
# -- the doorway (metres; x, y in the world, z above the floor) ----------------------------------------------------------
AX, AY = 0.4541, 0.0  # the bottom hinge's pin axis
DF = AX + 0.0159  # the door's swing face and the wall's face
GAP = 0.125 * IN  # door edge to hinge jamb
DOOR_W, DOOR_T, DOOR_H, UNDERCUT = 30 * IN, 1.375 * IN, 80 * IN, 0.5 * IN
WALL_T = 4.5 * IN
JAMB_T = 0.6875 * IN
REVEAL = 0.1875 * IN
CAS_W, CAS_T = 2.25 * IN, 0.6875 * IN
BASE_H, BASE_T = 3.5 * IN, 0.5 * IN
WALL_H = 2.30
Y_DOOR_EDGE = AY - GAP / 2  # the door's hinge edge
Y_JAMB = AY + GAP / 2  # the hinge jamb's face
CE = Y_JAMB + REVEAL  # the hinge-side casing's edge (6.4 mm from the axis)
Y_LATCH = Y_DOOR_EDGE - DOOR_W  # the door's latch edge
Y_LJAMB = Y_LATCH - GAP  # the latch jamb's face
LCE = Y_LJAMB - REVEAL  # the latch-side casing's edge
HEAD_Z = UNDERCUT + DOOR_H + GAP  # the head jamb's face

# -- the bottom hinge -----------------------------------------------------------------------------------------------------
H0 = UNDERCUT + 11.0 * IN  # bottom of the bottom hinge: 11 in. above the door's bottom edge
HL = 3.5 * IN  # 88.9 mm
KN = 5  # knuckles
KN_R = 0.0054  # barrel radius (10.8 mm across)
BORE_R = 0.00305
PIN_R = 0.234 * IN / 2  # 2.97 mm
HEAD_R, HEAD_H = 0.0054, 0.0048
PIN_MASS = 0.023
LEAF_T = 0.085 * IN

# -- the model ------------------------------------------------------------------------------------------------------------
LOOSE = 0.004
F_BIND = 60.0
F_SIDE = 2.0
V_MIN = 0.3
REARM = 0.005
TILT_MAX = 20.0
SEAT_R, SEAT_GAP_HI, SEAT_GAP_LO = 0.0020, 0.0015, -0.0010
V_HARM = 0.10
R_HALF, R_OUT_GAP, OFF_AXIS = 0.030, 0.002, 0.015
FLOOR_MID, AWAY, CLEAR_HANDS = 0.012, 0.03, 0.05

# -- the nail set (local frame: origin at the tip, +z toward the head) ----------------------------------------------------
NS_TIP_R, NS_BODY_R, NS_LEN, NS_TAPER = 0.0012, 0.00475, 0.102, 0.027
NS_MASS = 0.05
NS_GRIP = 0.070  # the right gripper holds it this far below its tip
NS_TILT = 0.0 if MODE == "straight" else 10.0  # deg: held leaning toward the front right, out of the corner of door face and casing
TILT_DIR = np.array([math.cos(math.radians(45.0)), math.sin(math.radians(45.0)), 0.0])  # tip leans this way (back left)

# -- the hammer (SimReady tool_hammer_a02; its body frame: the file's frame moved up to the middle of its 26 mm thickness,
# handle along +y toward the head, face toward +x; measured in this simulation) --------------------------------------------
H_MASS = 0.567
BODY_DZ = 0.0136  # the body frame sits this far above the file's origin
FACE_L = np.array([0.052, 0.137, 0.0135 - BODY_DZ])  # the face's centre
FACE_N_L = np.array([1.0, 0.0, 0.0])  # its outward normal
HANDLE_Y = 0.087  # the left gripper takes the handle here, 50 mm from the head's axis
HANDLE_AXIS_L = np.array([0.001, 0.0, 0.0135 - BODY_DZ])  # the handle's centre line (x, -, z)

LOOKS = {
    "wall": m.Look(color=m.rgb(0.90, 0.89, 0.86), roughness=0.85),
    "trim": m.Look(color=m.rgb(0.97, 0.97, 0.95), roughness=0.45),
    "door": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.5),
    "hinge": m.Look(color=m.rgb(0.62, 0.62, 0.60), metallic=0.85, roughness=0.35),
    "pin": m.Look(color=m.rgb(0.70, 0.70, 0.68), metallic=0.9, roughness=0.3),
    "blued": m.Look(color=m.rgb(0.18, 0.19, 0.22), metallic=0.75, roughness=0.4),
    "dark": m.Look(color=m.rgb(0.12, 0.12, 0.12), roughness=0.7),
}


def _bx(name, x0, x1, y0, y1, z0, z1, look):
    """A box between bounds (doorway frame = world x, y; z above the floor)."""
    size = (x1 - x0, y1 - y0, z1 - z0)
    c = ((x0 + x1) / 2 - AX, (y0 + y1) / 2 - AY, (z0 + z1) / 2)
    return (name, m.box(size, c), look), m.col_box(name + "_c", size, c), (x0, x1, y0, y1, z0, z1)


def _ring_wedges(z0, z1, n=12):
    """Convex wedges making the barrel's 6.1 mm bore (flats at BORE_R) out to its 10.8 mm outside."""
    out = []
    ri = BORE_R / math.cos(math.pi / n)
    ro = KN_R / math.cos(math.pi / n)
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = []
        for z in (z0, z1):
            for r, a in ((ri, a0), (ri, a1), (ro, a0), (ro, a1)):
                pts.append((r * math.cos(a), r * math.sin(a), z))
        pts = np.array(pts)
        f = np.array([[0, i, i + 1] for i in range(1, len(pts) - 1)])
        out.append(m.col_hull(f"bore_{k}", m.Mesh(pts, f)))
    return out


BOXES = []  # (x0, x1, y0, y1, z0, z1): the doorway's solid boxes, for the hammer check


def _doorway() -> m.Part:
    parts, cols = [], []

    def add(*a):
        p, c, b = _bx(*a)
        parts.append(p)
        cols.append(c)
        BOXES.append(b)

    xw0, xw1 = DF, DF + WALL_T
    add("wall_left", xw0, xw1, Y_JAMB + JAMB_T, 1.0, 0.0, WALL_H, "wall")
    add("wall_right", xw0, xw1, -1.3, Y_LJAMB - JAMB_T, 0.0, WALL_H, "wall")
    add("wall_head", xw0, xw1, Y_LJAMB - JAMB_T, Y_JAMB + JAMB_T, HEAD_Z + JAMB_T, WALL_H, "wall")
    add("jamb_hinge", xw0, xw1, Y_JAMB, Y_JAMB + JAMB_T, 0.0, HEAD_Z + JAMB_T, "trim")
    add("jamb_latch", xw0, xw1, Y_LJAMB - JAMB_T, Y_LJAMB, 0.0, HEAD_Z + JAMB_T, "trim")
    add("jamb_head", xw0, xw1, Y_LJAMB, Y_JAMB, HEAD_Z, HEAD_Z + JAMB_T, "trim")
    add("door", DF, DF + DOOR_T, Y_LATCH, Y_DOOR_EDGE, UNDERCUT, UNDERCUT + DOOR_H, "door")
    add("casing_hinge", DF - CAS_T, DF, CE, CE + CAS_W, 0.0, HEAD_Z + REVEAL + CAS_W, "trim")
    add("casing_latch", DF - CAS_T, DF, LCE - CAS_W, LCE, 0.0, HEAD_Z + REVEAL + CAS_W, "trim")
    add("casing_head", DF - CAS_T, DF, LCE - CAS_W, CE + CAS_W, HEAD_Z + REVEAL, HEAD_Z + REVEAL + CAS_W, "trim")
    add("base_left", DF - BASE_T, DF, CE + CAS_W, 1.0, 0.0, BASE_H, "trim")
    add("base_right", DF - BASE_T, DF, -1.3, LCE - CAS_W, 0.0, BASE_H, "trim")
    # the bottom hinge: leaves in the gap, the five-knuckle barrel round the bore (one collider ring)
    leaf_x1 = DF + DOOR_T - 0.25 * IN
    for nm, y0, y1 in (("leaf_door", Y_DOOR_EDGE - LEAF_T, Y_DOOR_EDGE), ("leaf_jamb", Y_JAMB, Y_JAMB + LEAF_T)):
        p, c, b = _bx(nm, AX + KN_R * 0.6, leaf_x1, y0, y1, H0, H0 + HL, "hinge")
        parts.append(p)
        cols.append(c)
    seg = HL / KN
    for k in range(KN):
        parts.append((f"knuckle_{k}", m.cylinder(KN_R - 0.0001, seg - 0.0006, (0.0, 0.0, H0 + seg * (k + 0.5)), segments=32),
                      "hinge"))
    cols += _ring_wedges(H0, H0 + HL)
    # the two upper hinges (out of reach; seen only)
    for zb in (UNDERCUT + DOOR_H - 7.0 * IN - HL, UNDERCUT + (DOOR_H - HL) / 2):
        parts.append((f"barrel_{int(zb * 1000)}", m.cylinder(KN_R, HL, (0.0, 0.0, zb + HL / 2)), "hinge"))
        parts.append((f"button_{int(zb * 1000)}", m.cylinder(HEAD_R, HEAD_H, (0.0, 0.0, zb + HL + HEAD_H / 2)), "hinge"))
    return m.fixed("doorway", (AX, AY, TOP), parts, cols, LOOKS, friction=(0.6, 0.5))


def _pin() -> m.Part:
    shank = m.cylinder(PIN_R, HL, (0.0, 0.0, HL / 2), segments=24)
    head = m.lathe([(0.0, HL), (HEAD_R, HL), (HEAD_R, HL + HEAD_H - 0.0012), (HEAD_R - 0.0012, HL + HEAD_H),
                    (0.0, HL + HEAD_H)], 32)
    cols = [m.col_cylinder("shank_c", PIN_R, HL, (0.0, 0.0, HL / 2)),
            m.col_cylinder("head_c", HEAD_R, HEAD_H, (0.0, 0.0, HL + HEAD_H / 2))]
    return m.free("pin", (AX, AY, TOP + H0), [("shank", shank, "pin"), ("head", head, "hinge")], cols, LOOKS,
                  mass=PIN_MASS, friction=(0.6, 0.5), solver_iters=48)


def _quat(r) -> tuple:
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


def _axis_rot(axis, deg) -> np.ndarray:
    a = math.radians(deg)
    u = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def _level(approach, closing) -> np.ndarray:
    zz = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float) - np.dot(closing, zz) * zz
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(zz, x), zz])


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# -- working geometry (also fixes where the tools lie) --------------------------------------------------------------------
Z = np.array([0.0, 0.0, 1.0])
PIN_END0 = np.array([AX, AY, TOP + H0])  # the pin's lower end at the start
NS_AX = _unit(math.cos(math.radians(NS_TILT)) * Z + math.sin(math.radians(NS_TILT)) * TILT_DIR)  # head -> tip, held
A_NS = _unit(TILT_DIR - np.dot(TILT_DIR, NS_AX) * NS_AX)  # the right gripper's approach while holding it
R_NS = _level(A_NS, -np.cross(NS_AX, A_NS))  # level, fingers closing square to the nail set
A_NS_FLAT = np.array([TILT_DIR[0], TILT_DIR[1], 0.0])
R_NS_PICK = _level(-Z, -np.cross(Z, A_NS_FLAT))  # straight down, the same fingers, over the nail set lying tip-forward
NS_AXIS_OFF = 0.009  # the nail set's axis lies this far beyond the TCP along the approach (fingertip hold)

# hammer, while striking: face up along the nail set's axis, handle running from the head toward the front left
U_H = _unit(np.array([0.30, -0.95, 0.0]) - np.dot([0.30, -0.95, 0.0], NS_AX) * NS_AX)  # grip -> head along the handle
AP_H = _unit(np.cross(U_H, NS_AX))  # the left gripper's approach while striking
if AP_H[0] < 0:
    AP_H = -AP_H
R_STRIKE = _level(AP_H, -NS_AX)  # level, fingers closing along the head's axis
R_HAM_PICK = _level(-Z, -(AP_H - np.dot(AP_H, Z) * Z))  # straight down, fingers along the lying head's axis


def _ham_rot(face_dir, handle_dir) -> np.ndarray:
    """Rotation of the hammer's file frame: +x (face) to face_dir, +y (toward the head) to handle_dir."""
    x = _unit(face_dir)
    y = _unit(np.asarray(handle_dir, float) - np.dot(handle_dir, x) * x)
    return np.column_stack([x, y, np.cross(x, y)])


# lying positions
NS_PICK = np.array([0.255, -0.175])  # the TCP over the nail set's body when it is taken
NS_TIP0 = NS_PICK + NS_GRIP * A_NS_FLAT[:2]
U_LYING = _unit(np.array([U_H[0], U_H[1], 0.0]))
AP_LYING = _unit(np.array([AP_H[0], AP_H[1], 0.0]))
HAM_PICK = np.array([0.215, 0.215])  # the TCP over the handle when it is taken
R_HAM_LYING = _ham_rot(AP_LYING, U_LYING)  # face toward AP, handle toward U: lies on its other side (z down)


def _ham_origin_lying() -> np.ndarray:
    """File origin of the lying hammer that puts its handle point (HANDLE_Y) under HAM_PICK."""
    grip_l = np.array([HANDLE_AXIS_L[0], HANDLE_Y, HANDLE_AXIS_L[2]])
    o = np.array([HAM_PICK[0], HAM_PICK[1], 0.0]) - R_HAM_LYING @ grip_l
    o[2] = TOP + 0.0265  # on its side, z of the file frame pointing down: the file's top (0.026) rests on the floor
    return o


def _nailset() -> m.Part:
    prof = [(0.0, 0.0006), (NS_TIP_R * 0.5, 0.0), (NS_TIP_R, 0.0), (NS_TIP_R, 0.002), (NS_BODY_R, NS_TAPER),
            (NS_BODY_R, NS_LEN - 0.003), (NS_BODY_R - 0.0008, NS_LEN), (0.0, NS_LEN)]
    cols = [m.col_cylinder("tip", NS_TIP_R, 0.002, (0.0, 0.0, 0.001)),
            m.col_hull("taper", m.lathe([(NS_TIP_R, 0.002), (NS_BODY_R, NS_TAPER)], 16)),
            m.col_cylinder("body", NS_BODY_R, NS_LEN - NS_TAPER, (0.0, 0.0, (NS_LEN + NS_TAPER) / 2))]
    q = _quat(_ns_lying_rot())
    return m.free("nailset", (float(NS_TIP0[0]), float(NS_TIP0[1]), TOP + NS_BODY_R + 0.0003),
                  [("nailset", m.lathe(prof, 24), "blued")], cols, LOOKS, mass=NS_MASS, start_quat=q,
                  friction=(0.9, 0.8))


def _ns_lying_rot() -> np.ndarray:
    """Local +z (tip -> head) points back along -A_NS_FLAT: the tip forward (toward the door), the head toward the arm."""
    zl = -A_NS_FLAT
    xl = np.array([0.0, 0.0, 1.0])
    yl = np.cross(zl, xl)
    return np.column_stack([xl, yl, zl])


DOORWAY = _doorway()
PIN = _pin()
NAILSET = _nailset()
HAMMER = SimReady("hammer", pos=tuple(float(v) for v in _ham_origin_lying()), size=(0.104, 0.304, 0.026), mass=H_MASS,
                  package="tool_hammer_a02", usd="sm_tool_hammer_a02_01.usd", quat_xyzw=_quat(R_HAM_LYING))
OIL = SimReady("oil", pos=(0.40, -0.33, TOP), size=(0.052, 0.104, 0.272), mass=0.40,
               package="consumable_lubricant_oil_multi_purpose_a01",
               usd="sm_consumable_lubricant_oil_multi_purpose_a01_01.usd", yaw=0.4)
KNIFE = SimReady("knife", pos=(0.40, 0.36, TOP), size=(0.16, 0.061, 0.026), mass=0.12,
                 package="handTool_knife_retractableBlade_a01", usd="sm_handTool_knife_retractableBlade_a01_01.usd",
                 yaw=1.2)
OBJECTS = [DOORWAY, PIN, NAILSET, HAMMER, OIL, KNIFE]

def _hammer_points() -> np.ndarray:
    """Points on the hammer's head and handle boxes (its file frame), for the damage check."""
    pts = []
    for x in np.linspace(-0.052, 0.052, 9):
        for y in (0.118, 0.137, 0.150):
            for z in (0.001, 0.0135, 0.026):
                pts.append((x, y, z))
    for y in np.linspace(-0.152, 0.11, 12):
        for x in (-0.013, 0.015):
            for z in (0.003, 0.024):
                pts.append((x, y, z))
    return np.array(pts) - np.array([0.0, 0.0, BODY_DZ])


_HPTS = _hammer_points()


def _doorway_gap(pts, which=None) -> float:
    """Smallest distance (m) from world points to the doorway's boxes and the bottom hinge's barrel (negative inside)."""
    p = np.asarray(pts, float) - np.array([0.0, 0.0, TOP])
    best = 1.0
    for i, (x0, x1, y0, y1, z0, z1) in enumerate(BOXES):
        c = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        h = np.array([(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2])
        q = np.abs(p - c) - h
        d = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
        if which is not None and float(d.min()) < best:
            which[:] = [i, int(np.argmin(d))]
        best = min(best, float(d.min()))
    rad = np.hypot(p[:, 0] - AX, p[:, 1] - AY)
    dz = np.maximum(np.maximum(H0 - p[:, 2], p[:, 2] - (H0 + HL)), 0.0)
    best = min(best, float(np.min(np.hypot(np.maximum(rad - KN_R, 0.0), dz))))
    return best


# -- the trial ------------------------------------------------------------------------------------------------------------
class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.raised = 0.0
        self.loose = False
        self.side_on = False
        self.damage = ""
        self.armed = True
        self.prev = None
        self.blows = []
        self.score = 0.0
        self.ok = False
        self.step_n = 0
        self.info = {}
        if env is not None:
            self._env = env
            self._wrap(env)
            self._hold(getattr(env, "unwrapped", env))

    def _wrap(self, env):
        base = getattr(env, "unwrapped", env)
        if getattr(base.scene, "_hinge_wrapped", False):
            return
        orig = base.scene.update

        def update(dt, _orig=orig, _base=base):
            _orig(dt)
            self._substep(_base)

        base.scene.update = update
        base.scene._hinge_wrapped = True

    def _pose(self, base, name):
        d = base.scene[name].data
        p = m._np(d.root_link_pose_w)[0]
        v = m._np(d.root_link_vel_w)[0]
        return p[:3] - m._np(base.scene.env_origins[0]), m.quat_matrix(p[3:7]), v

    def _hold(self, base):
        """Write the bound pin back to where the blows have left it."""
        import torch

        asset = base.scene["pin"]
        origin = m._np(base.scene.env_origins[0])
        pose = torch.tensor([[AX + origin[0], AY + origin[1], TOP + H0 + self.raised + origin[2], 0.0, 0.0, 0.0, 1.0]],
                            dtype=torch.float32, device=asset.device)
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=asset.device))

    def _read(self, base):
        hp, hr, hv = self._pose(base, "hammer")
        np_, nr, nv = self._pose(base, "nailset")
        pp, pr, pv = self._pose(base, "pin")
        face = hp + hr @ FACE_L
        vface = hv[:3] + np.cross(hv[3:6], hr @ FACE_L)
        head_l = np.array([0.0, 0.0, NS_LEN])
        nhead = np_ + nr @ head_l
        vnhead = nv[:3] + np.cross(nv[3:6], nr @ head_l)
        return dict(hp=hp, hr=hr, hv=hv, face=face, n=hr @ FACE_N_L, vface=vface, tip=np_, naxis=-nr[:, 2], nhead=nhead,
                    vnhead=vnhead, pp=pp, pr=pr, pv=pv)

    def _substep(self, base):
        if not self.loose:
            self._hold(base)
        s = self._read(base)
        prev = self.prev
        self.prev = s
        if prev is None:
            return
        # blows of the face on the nail set's head
        rel = s["nhead"] - s["face"]
        sep = float(rel @ s["n"])
        lat = float(np.linalg.norm(rel - sep * s["n"]))
        if sep > REARM or lat > 0.02:
            self.armed = True
        if self.armed and sep < 0.0008 and lat < 0.016:
            self.armed = False
            v_rel = prev["vface"] - prev["vnhead"]
            v_in = float(v_rel @ prev["naxis"])  # face moving toward the tip along the nail set's axis
            if v_in >= V_MIN:
                self._blow(prev, v_in)
        # the hammer on the door, its trim or the hinge, moving
        if not self.damage:
            speed = max(float(np.linalg.norm(s["hv"][:3])), float(np.linalg.norm(s["vface"])))
            if speed >= V_HARM:
                pts = s["hp"] + _HPTS @ s["hr"].T
                which = []
                g = _doorway_gap(pts, which)
                if g < 0.001:
                    self.damage = f"the hammer struck the door or its trim at {speed:.2f} m/s"
                    if DEBUG:
                        print("HINGEDBG DAMAGE", self.damage, self.step_n, "box", which, "pt",
                              np.round(pts[which[1]] - [AX, AY, TOP], 4) if which else "barrel", flush=True)
        # the door's weight on a loose pin, until it has left the top knuckle
        if self.loose:
            bottom = s["pp"][2] - TOP
            inside = bottom < H0 + HL and np.hypot(s["pp"][0] - AX, s["pp"][1] - AY) < 0.004
            if inside and not self.side_on:
                m.push(self._env, "pin", (0.0, F_SIDE, 0.0))
                self.side_on = True
            elif not inside and self.side_on:
                m.push(self._env, "pin", (0.0, 0.0, 0.0))
                self.side_on = False

    def _blow(self, prev, v_in):
        tip = prev["tip"]
        end = np.array([AX, AY, TOP + H0 + self.raised])
        off = float(np.hypot(tip[0] - AX, tip[1] - AY))
        gap = float(end[2] - tip[2])  # positive: the tip below the pin's end
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(prev["naxis"][2])))))
        e = 0.5 * H_MASS * v_in * v_in
        rec = dict(step=self.step_n, v=round(v_in, 3), e=round(e, 4), off=round(off * 1000, 2), gap=round(gap * 1000, 2),
                   tilt=round(tilt, 1), r0=round(self.raised * 1000, 2))
        if self.loose:
            rec["out"] = "pin already loose"
        elif off <= SEAT_R and SEAT_GAP_LO <= gap <= SEAT_GAP_HI and tilt <= TILT_MAX:
            self.raised += e / F_BIND
            rec["out"] = "driven"
            if self.raised >= LOOSE:
                self.loose = True
                rec["out"] = "driven, loose"
        else:
            rec["out"] = "nail set not on the pin's end"
        rec["r1"] = round(self.raised * 1000, 2)
        self.blows.append(rec)
        if DEBUG:
            print("HINGEDBG BLOW", rec, flush=True)

    def step(self, env):
        base = getattr(env, "unwrapped", env)
        self.step_n += 1
        s = self._read(base)
        bottom = s["pp"] + s["pr"] @ np.array([0.0, 0.0, 0.0])
        top_end = s["pp"] + s["pr"] @ np.array([0.0, 0.0, HL + HEAD_H])
        mid = s["pp"] + s["pr"] @ np.array([0.0, 0.0, (HL + HEAD_H) / 2])
        off_axis = float(np.hypot(mid[0] - AX, mid[1] - AY))
        r = float(s["pp"][2] - (TOP + H0)) if off_axis < 0.004 else 1.0
        low_end = min(bottom[2], top_end[2]) - TOP
        out = low_end >= H0 + HL + R_OUT_GAP or off_axis >= OFF_AXIS
        on_floor = (mid[2] - TOP) <= FLOOR_MID and off_axis >= AWAY
        tips = m.fingertips(env)
        pin_pts = np.array([s["pp"] + s["pr"] @ np.array([0.0, 0.0, z]) for z in np.linspace(0.0, HL + HEAD_H, 7)])
        hands = float(np.min(np.linalg.norm(tips[:, None, :] - pin_pts[None, :, :], axis=2)))
        still = float(np.linalg.norm(s["pv"][:3])) < 0.01 and float(np.linalg.norm(s["pv"][3:6])) < 0.5
        level = 0.75 if out else 0.5 if r >= R_HALF else 0.25 if (self.loose or r >= LOOSE) else 0.0
        if self.damage:
            level = min(level, 0.25)
        self.ok = bool(out and on_floor and still and hands >= CLEAR_HANDS and not self.damage)
        self.score = float(level)
        self.info = dict(r=r, out=out, on_floor=on_floor, hands=hands, damage=self.damage, loose=self.loose)
        if DEBUG and self.step_n % 10 == 0:
            ntilt = math.degrees(math.acos(max(-1.0, min(1.0, float(s["naxis"][2])))))
            print(f"HINGEDBG step={self.step_n} r={r * 1000:.2f} raised={self.raised * 1000:.2f} loose={self.loose} "
                  f"tip=({(s['tip'][0] - AX) * 1000:.1f},{(s['tip'][1] - AY) * 1000:.1f},{(s['tip'][2] - TOP - H0) * 1000:.1f}) "
                  f"ntilt={ntilt:.1f} nhead=({(s['nhead'][0] - AX) * 1000:.1f},{(s['nhead'][1] - AY) * 1000:.1f},"
                  f"{(s['nhead'][2] - TOP) * 1000:.1f}) face=({(s['face'][0] - AX) * 1000:.1f},{(s['face'][1] - AY) * 1000:.1f},"
                  f"{(s['face'][2] - TOP) * 1000:.1f}) n=({s['n'][0]:.2f},{s['n'][1]:.2f},{s['n'][2]:.2f}) "
                  f"pin=({(s['pp'][0] - AX) * 1000:.1f},{(s['pp'][1] - AY) * 1000:.1f},{(s['pp'][2] - TOP) * 1000:.1f}) "
                  f"out={out} floor={on_floor} hands={hands:.3f} dmg={self.damage or '-'} score={self.score} ok={self.ok}",
                  flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.ok)


# -- scripted demo ----------------------------------------------------------------------------------------------------------
# The right gripper takes the nail set from the floor by its body, turns it tip-up and sets its tip on the pin's lower end,
# leaning 10 deg out of the corner between the door face and the casing; the left takes the hammer by its handle near the
# head, turns it face-up under the nail set and strikes the nail set's head with short upward strokes of the whole arm.
# Once the pin is loose the left arm takes the hammer away, the right lays the nail set on the floor, pinches the pin's
# head from the front right, draws the pin up out of the knuckles and lays it on the floor.
OPEN, CLOSED = -0.040, 0.0


def _good_ik(arm, pos, rot, seeds=None):
    from yam_arena import ik, station

    best = None
    rng = np.random.default_rng(1)
    pool = list(seeds or []) + [np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])] + \
        [ik.LOWER + rng.random(6) * (ik.UPPER - ik.LOWER) for _ in range(40)]
    for sd in pool:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=100)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _joint_path(arm, q0, q1, finger, label, seconds, steps=15) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, rot)


def _stay(wps, seconds, label="wait") -> list:
    last = wps[-1]
    return [Waypoint(float(seconds), last.pos, last.finger, label, last.yaw)]


def _dur(wps) -> float:
    return sum(w.duration for w in wps)


Q_HOME = np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])
T_SET = 16.0  # by then the nail set stands on the pin's end
N_BLOWS = 3
BLOW_S = 0.9
DRAW = 0.035  # the face draws this far back from the nail set's head along its axis before a stroke
DRIVE = 0.030  # ... and is sent this far past it
SAG = 0.022  # the hammer arm sags about this far under the hammer's weight
NS_SAG = 0.005  # the nail set sits about this far lower in the fingers than sent (measured in this simulation)
NS_PRESS = 0.0008  # the tip is sent this far past the pin's end, to keep it pressed on
NS_GUIDE = -0.0035  # light hold on the 9.5 mm body (about 6 N a finger) while it is struck
RISE_PLAN = 0.003  # the pin's planned rise per blow (E / F_BIND at the demo's measured blow energy)


def _ns_tcp(tip):
    """The right TCP that puts the held nail set's tip at ``tip``."""
    return np.asarray(tip, float) - NS_GRIP * NS_AX - NS_AXIS_OFF * A_NS + NS_SAG * Z


def _right() -> list:
    pick = np.array([NS_PICK[0], NS_PICK[1], TOP + NS_BODY_R + 0.002])
    out = _joint_path("right", Q_HOME, _good_ik("right", pick + [0, 0, 0.07], R_NS_PICK), OPEN, "above the nail set", 3.0)
    out += [_w(1.0, pick, OPEN, "fingers either side of its body", R_NS_PICK),
            _w(0.8, pick, CLOSED, "take the nail set", R_NS_PICK),
            _w(0.4, pick, CLOSED, "settle", R_NS_PICK),
            _w(1.0, pick + [0, 0, 0.12], CLOSED, "lift it", R_NS_PICK)]
    end0 = PIN_END0
    below = end0 - 0.06 * Z
    turn_at = _ns_tcp(below) - 0.12 * A_NS_FLAT
    q_lift = _good_ik("right", turn_at, R_NS_PICK)
    q_turn = _good_ik("right", turn_at, R_NS)
    out += [_w(1.2, turn_at, CLOSED, "carry it short of the hinge", R_NS_PICK)]
    out += _joint_path("right", q_lift, q_turn, CLOSED, "turn it tip-up", 2.0, 16)
    out += [_w(1.4, _ns_tcp(below), CLOSED, "tip under the pin, low", R_NS),
            _w(1.2, _ns_tcp(end0 - 0.008 * Z), CLOSED, "tip just under the pin's end", R_NS),
            _w(0.8, _ns_tcp(end0 + NS_PRESS * Z), CLOSED, "tip on the pin's end", R_NS)]
    if _dur(out) < T_SET:
        out += _stay(out, T_SET - _dur(out), "hold the nail set on the pin")
    # through the blows the grip is eased to a light hold, so that each blow carries the nail set up after the pin
    out += [_w(0.3, _ns_tcp(end0 + NS_PRESS * Z), NS_GUIDE, "ease the grip", R_NS)]
    t0 = T_SET + 4.0
    out += _stay(out, t0 - _dur(out), "keep the nail set on the pin")
    for k in range(N_BLOWS):
        up = min((k + 1) * RISE_PLAN, LOOSE + 0.001)
        out += [_w(BLOW_S - 0.2, _ns_tcp(end0 + (min(k * RISE_PLAN, LOOSE + 0.001) + NS_PRESS) * Z), NS_GUIDE,
                   f"hold through blow {k + 1}", R_NS),
                _w(0.2, _ns_tcp(end0 + (up + NS_PRESS) * Z), NS_GUIDE, "follow the pin up", R_NS)]
    out += [_w(0.3, out[-1].pos, CLOSED, "close on the nail set", R_NS)]
    out += _stay(out, 1.5, "hold while the hammer goes away")
    # lay the nail set down at the front right
    lay = np.array([0.34, -0.20, TOP + 0.10])
    q_now = _good_ik("right", _ns_tcp(end0 - 0.06 * Z), R_NS)
    out += [_w(1.0, _ns_tcp(end0 - 0.06 * Z), CLOSED, "draw the nail set down off the pin", R_NS),
            _w(1.2, _ns_tcp(end0 - 0.06 * Z) - 0.08 * A_NS_FLAT, CLOSED, "back out of the corner", R_NS),
            _w(1.5, lay, CLOSED, "over the floor", R_NS),
            _w(1.0, lay - [0, 0, 0.045], CLOSED, "low", R_NS),
            _w(0.6, lay - [0, 0, 0.045], OPEN, "let go of it", R_NS),
            _w(0.8, lay + [0, 0, 0.05], OPEN, "away", R_NS)]
    # pinch the pin's head from the front right
    head_c = np.array([AX, AY, TOP + H0 + HL + LOOSE + HEAD_H / 2])
    a_p = A_NS_FLAT
    r_pin = _level(a_p, -np.cross(Z, a_p))
    grip = np.array([AX, AY, TOP + H0 + HL + 0.0195]) - NS_AXIS_OFF * a_p  # fingers' lower edge 1.5 mm above the knuckle
    out += [_w(1.6, grip - 0.06 * a_p + [0, 0, 0.02], OPEN, "toward the pin's head", r_pin),
            _w(1.2, grip, OPEN, "fingers either side of the head", r_pin),
            _w(0.8, grip, CLOSED, "pinch the head", r_pin),
            _w(0.4, grip, CLOSED, "settle", r_pin),
            _w(2.5, grip + [0, 0, HL + 0.02], CLOSED, "draw the pin up out of the knuckles", r_pin),
            _w(1.2, grip + [0, 0, HL + 0.02] - 0.10 * a_p, CLOSED, "out of the corner", r_pin)]
    drop = np.array([0.395, -0.10, TOP + 0.125])
    out += [_w(1.5, drop, CLOSED, "over the floor", r_pin),
            _w(0.8, drop - [0, 0, 0.012], CLOSED, "its end on the floor", r_pin),
            _w(0.6, drop - [0.02, 0, 0.012], CLOSED, "tip it toward the robot", r_pin),
            _w(0.5, drop - [0.02, 0, 0.012], OPEN, "let it fall", r_pin),
            _w(1.0, drop + [-0.04, 0, 0.06], OPEN, "away", r_pin)]
    return out + _stay(out, 12.0, "clear")


def _face_tcp(face_c):
    """The left TCP that puts the held hammer's face centre at ``face_c`` (handle centre 51 mm below the face)."""
    grip_l = np.array([HANDLE_AXIS_L[0], HANDLE_Y, HANDLE_AXIS_L[2]])
    return np.asarray(face_c, float) - R_STRIKE_HAM @ (FACE_L - grip_l)


# the hammer's file frame while striking: face along NS_AX, handle toward U_H
R_STRIKE_HAM = _ham_rot(NS_AX, U_H)


def _left() -> list:
    pick = np.array([HAM_PICK[0], HAM_PICK[1], TOP + 0.017])
    out = _joint_path("left", Q_HOME, _good_ik("left", pick + [0, 0, 0.08], R_HAM_PICK), OPEN, "above the handle", 3.0)
    out += [_w(1.0, pick, OPEN, "fingers either side of the handle", R_HAM_PICK),
            _w(0.8, pick, CLOSED, "take the handle near the head", R_HAM_PICK),
            _w(0.4, pick, CLOSED, "settle", R_HAM_PICK),
            _w(1.0, pick + [0, 0, 0.12], CLOSED, "lift it", R_HAM_PICK)]
    nhead0 = PIN_END0 if MODE == "onearm" else PIN_END0 - NS_LEN * NS_AX
    stage_face = nhead0 - 0.035 * NS_AX
    stage = _face_tcp(stage_face) - 0.10 * AP_LYING + [0, 0, SAG + 0.03]
    q_lift = _good_ik("left", pick + [0, 0, 0.12], R_HAM_PICK)
    q_stage = _good_ik("left", stage, R_STRIKE)
    if _dur(out) < T_SET + 0.5:
        out += _stay(out, T_SET + 0.5 - _dur(out), "wait for the nail set")
    out += _joint_path("left", q_lift, q_stage, CLOSED, "turn it face-up", 1.8, 15)
    out += [_w(1.2, _face_tcp(stage_face) + [0, 0, SAG], CLOSED, "face under the nail set", R_STRIKE)]
    t0 = T_SET + 4.0
    if _dur(out) < t0:
        out += _stay(out, t0 - _dur(out), "aim")
    raised = 0.0
    for k in range(N_BLOWS):
        nhead = nhead0 + min(raised, LOOSE) * Z
        low = _face_tcp(nhead - DRAW * NS_AX) + [0, 0, SAG]
        hit = _face_tcp(nhead + DRIVE * NS_AX) + [0, 0, SAG]
        rest = _face_tcp(nhead - 0.02 * NS_AX) + [0, 0, SAG]
        out += [_w(BLOW_S - 0.37, low, CLOSED, f"draw back (blow {k + 1})", R_STRIKE),
                _w(0.15, low, CLOSED, "hold", R_STRIKE),
                _w(0.07, hit, CLOSED, f"strike (blow {k + 1})", R_STRIKE),
                _w(0.15, rest, CLOSED, "drop back", R_STRIKE)]
        raised += RISE_PLAN
    away = _face_tcp(nhead0 - 0.04 * NS_AX) - 0.12 * AP_LYING + [0, 0, SAG + 0.03]
    out += [_w(1.0, _face_tcp(nhead0 - 0.04 * NS_AX) + [0, 0, SAG], CLOSED, "hammer down, clear of the nail set", R_STRIKE),
            _w(1.5, away, CLOSED, "take the hammer away", R_STRIKE),
            _w(1.5, away + [0, 0.05, 0.10], CLOSED, "hold it clear", R_STRIKE)]
    return out + _stay(out, 40.0, "hold it clear")


def _idle(arm):
    from yam_arena import ik, station

    p, r = ik.tcp_pose(Q_HOME, station.BASES[arm])
    return [Waypoint(80.0, tuple(float(v) for v in p), OPEN, "stay", r)]


def _pull_only() -> list:
    """Red team: pinch the bound pin's head and pull, no tools."""
    head_c = np.array([AX, AY, TOP + H0 + HL + HEAD_H / 2])
    a_p = A_NS_FLAT
    r_pin = _level(a_p, -np.cross(Z, a_p))
    grip = head_c - NS_AXIS_OFF * a_p + np.array([0.0, 0.0, 0.006])
    out = _joint_path("right", Q_HOME, _good_ik("right", grip - 0.06 * a_p + [0, 0, 0.02], r_pin), OPEN, "toward the head", 3.0)
    out += [_w(1.2, grip, OPEN, "fingers either side of the head", r_pin),
            _w(0.8, grip, CLOSED, "pinch the head", r_pin),
            _w(2.5, grip + [0, 0, 0.06], CLOSED, "pull", r_pin),
            _w(1.2, grip + [0, 0, 0.06] - 0.10 * a_p, CLOSED, "away", r_pin)]
    return out + _stay(out, 30.0, "wait")


def demo():
    if MODE == "none":
        return {"left": _idle("left"), "right": _idle("right")}
    if MODE == "pull":
        return {"left": _idle("left"), "right": _pull_only()}
    if MODE == "onearm":
        return {"left": _left(), "right": _idle("right")}
    return {"left": _left(), "right": _right()}


def demo_script(ctx):
    """The same scripted plan, played through demokit (for its CPU debug runs): each arm follows demo()'s waypoints."""
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    plans = {side: ArmPlan(wps, station.BASES[side]) for side, wps in demo().items()}
    steps = max(round(p.duration * 20) for p in plans.values())
    for k in range(1, steps + 1):
        for side, plan in plans.items():
            getattr(ctx, side).cmd = plan.action7(min(k / 20.0, plan.duration))[0]
        yield
