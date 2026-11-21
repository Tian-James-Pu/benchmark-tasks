"""Pin a shaft to a clevis: drive a started 4 x 22 mm spring pin through the clevis's hub and the shaft with a pin punch
and an 8 oz ball-peen hammer until neither end of the pin stands proud of the hub.

On the station table (world frame, metres: x forward from the arm bases, y to the robot's left, z up; table top at
z 0.7475):
- clevis (one fixed mechanism): a steel V-block 75 mm long (x), 60 mm wide and high, with a 90 deg V 40 mm across and a
  4 mm relief slot (x 0.263 to 0.338, y -0.03 to 0.03); in the V the 24 mm hub (30 mm long, 12 mm bore) of a steel
  clevis, its fork pointing along -x; a 12 mm steel shaft in the bore, running 120 mm out along +x. Hub and shaft are
  cross-drilled 4 mm on one vertical axis at x 0.280, y 0.000; the hub's top is 69.0 mm above the table. The pin is the
  mechanism's one joint: a 4 x 22 mm slotted spring pin (ISO 8752) started 3.0 mm into the hole, 19.0 mm proud.
- punch: loose, a 5/32 in roll-pin punch: a 4 mm point 40 mm long ending in a rounded pilot (1.8 mm across, 1.5 mm
  long) that sits in the spring pin's open end, a 9.6 mm body, 100 mm long, 36 g, its struck end crowned. It lies on
  the table on the left, its point toward the left arm. The pin's upper end is modelled open (2.4 mm bore, 2 mm deep).
- hammer: loose, a Vaughan TC308 8 oz ball-peen hammer (85 mm head, 22 mm face, hickory handle, 298 mm, 0.37 kg),
  lying on its side on the right.
- distractors (loose SimReady props, ignored by the scorer): a shop rag, a can of multi-purpose oil and an M16 nut.

The pin in the bores (the model). The pin is held by its spring against the bore walls and moves only when struck: by
the punch with its point on the pin's upper end, or by the hammer's face directly. A blow counts when the face meets the
punch's head (or the pin's end) moving toward it along its axis at 0.3 m/s or more (a new blow once the face has lifted
5 mm off). Each blow drives the pin until the friction of the bores, PIN_K newtons per metre of pin engaged in them, has
taken the hammer's kinetic energy (0.5 m v^2, m the hammer's mass). A steady push never drives it.
- The punch is on the pin when its point is within 1.5 mm of the pin's centre and between 1.0 mm into and 1.5 mm above
  its end. A blow on the punch on the pin drives it; tilted more than 3 deg from the hole's axis, the blow cocks the
  pin instead. A blow with the punch's point 1.5 to 3 mm off centre (half on the pin's edge) also cocks it. A cocked pin
  binds and never moves again.
- A blow on the punch with its point on the hub and not on the pin marks the hub (a blow on the part).
- The hammer's face (22 mm across) cannot follow the pin into the hole: a direct blow takes the pin's upper end no lower
  than 0.5 mm proud, and one that would go further lands the face on the hub (a blow on the part). A direct blow with the
  face more than 3 deg off square to the pin cocks it.
- Any part of the hammer touching the clevis, hub or shaft while moving at 0.1 m/s or more is a blow on the part.
The model runs after every physics step (240 Hz); the record of a blow on the part or a cocked pin is cleared only by
mechkit's on_reset hook.

Score (task.json "scoring" says the same): the depth d is how far the pin's lower end is below the top of the 24 mm hub
(3.0 mm at the start); the upper end then stands 22 - d mm proud of the hub and the lower end d - 24 mm proud of its
underside.
- A blow on the part or a cocked pin caps the score at 0.25 for the rest of the run.
- 1.0 (success, held 3 s): neither end proud by more than 0.25 mm (d 21.75 to 24.25 mm), no blow on the part, the pin
  not cocked, every fingertip of both grippers at least 5 cm from the top of the hole, the punch's point at least 3 cm
  from the pin, no part of the hammer within 2 cm of the hub, the pin at rest.
- 0.75: neither end proud as above, the hands or tools not yet clear.
- 0.5: the pin through the shaft and at least 2 mm into the hub's far wall (d >= 20 mm).
- 0.25: the pin through the hub's near wall and at least 2 mm into the shaft (d >= 8 mm).
- 0: otherwise (doing nothing leaves d = 3 mm).
"""

from __future__ import annotations

import itertools
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
    """Let a demo Waypoint's ``yaw`` be a full 3x3 TCP rotation (world frame), for the level grips the punch and the
    hammer need (the same wrapper as the accepted nail-driving task's bodykit.allow_matrix_grips)."""
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
DEBUG = os.environ.get("PIN_DEBUG") == "1"

# -- the work: V-block, clevis hub, shaft, pin (mm in comments) --------------------------------------------------------
MX, MY = 0.280, 0.000  # the cross-hole's axis
VB_X0, VB_X1, VB_W, VB_H = -0.017, 0.058, 0.060, 0.060  # V-block, in the part's frame (origin MX, MY, TOP)
V_APEX, V_HALF, SLOT = 0.040, 0.020, 0.002  # the V's bottom, its half-width at the top, the relief slot's half-width
HR = 0.012  # hub radius (24 mm hub: walls 6 mm, 1.5 x the pin, as SPIROL recommends)
HZ = V_APEX + HR * math.sqrt(2.0)  # hub axis height: a 22 mm round in a 90 deg V
HT = HZ + HR  # hub top (69.0 mm above the table)
HUB_X = 0.015  # hub half-length (30 mm hub)
SR = 0.006  # shaft radius (12 mm shaft: the 4 mm hole is a third of it, the most SPIROL allows)
SHAFT_X1 = 0.135
PIN_R, PIN_L = 0.002, 0.022  # 4 x 22 mm spring pin
PIN_ID, PIN_CUP = 0.0024, 0.002  # its bore (4 mm heavy-duty pin, 0.8 mm wall) and the open depth modelled
W = 2 * HR  # the bore's length through the hub: 24 mm
D0 = 0.003  # how far the pin is started
OPEN_HOLE = 0.003  # the hub's collision shape is open this far (half a square) around the bore, for the punch's point
PIN_K = 5000.0  # N per metre of pin in the bores (scaled; see the README)
V_MIN = 0.3  # m/s
REARM = 0.005
TILT_MAX = 3.0  # deg
SEAT_R, SEAT_GAP_HI, SEAT_GAP_LO, EDGE_R = 0.0016, 0.0015, -0.001, 0.003
FACE_STOP = 0.0005  # a direct hammer blow leaves the pin's upper end at least this proud
D_SHAFT, D_FAR = 0.008, 0.020
PROUD_TOL = 0.00025
D_MAX = 0.0275  # the pin's lower end reaches the V-block's walls
CLEAR_HANDS, CLEAR_PUNCH, CLEAR_HAMMER = 0.05, 0.03, 0.02

# -- the punch: a 5/32 in roll-pin punch with a pilot point, at the size of a Starrett 565D (5/32 in, 4 in long) -----
P_PT_R, P_PT_L, P_BODY_R, P_LEN = 0.00198, 0.040, 0.0048, 0.100
P_TAPER_TOP = 0.050
P_MASS = 0.036
CROWN_R = 0.010
PILOT = False
PILOT_R0, PILOT_R1, PILOT_L = 0.00025, 0.0006, 0.0015  # the pilot: a rounded cone 1.2 mm across, 1.5 mm long
P_GRIP = 0.060  # the left gripper takes it this far up from its point (its balance point is 66 mm up)
TURN_BACK = 0.07  # ... this far short of the bore
LIFT_Z = 0.20  # the punch is turned point-down with the grip this high over the table

# -- the hammer: 8 oz ball-peen (local frame: origin at the eye's middle, z the head's axis from face to ball, the
# handle along -y) ------------------------------------------------------------------------------------------------------
FACE_R, FACE_D = 0.011, 0.040  # face radius, face below the eye's middle
H_MASS = 0.37  # Vaughan TC308: 0.81 lb in all, 8 oz head
L_H = 0.036  # the right gripper takes the handle this far from the head's axis
HANDLE_W, HANDLE_H, HANDLE_Y0, HANDLE_Y1 = 0.015, 0.024, -0.010, -0.288

LOOKS = {
    "steel": m.Look(color=m.rgb(0.55, 0.57, 0.6), metallic=0.8, roughness=0.35),
    "blued": m.Look(color=m.rgb(0.20, 0.22, 0.26), metallic=0.7, roughness=0.4),
    "bright": m.Look(color=m.rgb(0.78, 0.78, 0.76), metallic=0.9, roughness=0.25),
    "hickory": m.Look(color=m.rgb(0.78, 0.60, 0.38), roughness=0.7),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.8),
    "ground": m.Look(color=m.rgb(0.68, 0.69, 0.70), metallic=0.85, roughness=0.3),
}


def _yz_prism(outline_yz, x0, x1) -> m.Mesh:
    """A prism with an outline in the part's (y, z) plane, from x0 to x1."""
    p = m.prism(outline_yz, x0, x1)
    return m.Mesh(p.v[:, [2, 0, 1]], p.f)


def _hull_mesh(points) -> m.Mesh:
    pts = np.asarray(points, float)
    f = np.array([[0, i, i + 1] for i in range(1, len(pts) - 1)])
    return m.Mesh(pts, f)


def _hub_slice(y_lo: float, x0: float, x1: float, sign: float) -> m.Mesh:
    """The part of the hub's circle beyond |y| = y_lo on one side, from x0 to x1 (for the open hole)."""
    pts = []
    for t in np.linspace(-math.pi / 2, math.pi / 2, 17):
        y, z = HR * math.cos(t), HZ + HR * math.sin(t)
        if y >= y_lo:
            pts.append((y, z))
    h = math.sqrt(HR * HR - y_lo * y_lo)
    pts += [(y_lo, HZ - h), (y_lo, HZ + h)]
    out = []
    for x in (x0, x1):
        out += [(x, sign * y, z) for y, z in pts]
    return _hull_mesh(out)


def _work() -> m.Part:
    side_r = [(SLOT, 0.036), (VB_W / 2, 0.036), (VB_W / 2, VB_H), (V_HALF, VB_H), (SLOT, V_APEX + SLOT)]
    side_l = [(-y, z) for y, z in reversed(side_r)]
    base = m.box((VB_X1 - VB_X0, VB_W, 0.036), ((VB_X0 + VB_X1) / 2, 0.0, 0.018))
    vr, vl = _yz_prism(side_r, VB_X0, VB_X1), _yz_prism(side_l, VB_X0, VB_X1)
    hub = m.cylinder(HR, 2 * HUB_X, (0.0, 0.0, HZ), axis="x", segments=48)
    shaft = m.cylinder(SR, SHAFT_X1 - HUB_X, ((HUB_X + SHAFT_X1) / 2, 0.0, HZ), axis="x")
    ear_l = m.box((0.030, 0.006, 0.024), (-HUB_X - 0.015, 0.009, HZ))
    ear_r = m.box((0.030, 0.006, 0.024), (-HUB_X - 0.015, -0.009, HZ))
    bolt = m.cylinder(0.004, 0.024, (-HUB_X - 0.020, 0.0, HZ), axis="y")
    parts = [("vblock_base", base, "ground"), ("vblock_r", vr, "ground"), ("vblock_l", vl, "ground"),
             ("hub", hub, "blued"), ("shaft", shaft, "bright"), ("ear_l", ear_l, "blued"), ("ear_r", ear_r, "blued"),
             ("clevis_pin", bolt, "steel")]
    cols = [m.col_box("vb_base", (VB_X1 - VB_X0, VB_W, 0.036), ((VB_X0 + VB_X1) / 2, 0.0, 0.018)),
            m.col_hull("vb_r", vr), m.col_hull("vb_l", vl),
            m.col_hull("hub_a", m.cylinder(HR, HUB_X - OPEN_HOLE, (-(HUB_X + OPEN_HOLE) / 2, 0.0, HZ), axis="x", segments=24)),
            m.col_hull("hub_b", m.cylinder(HR, HUB_X - OPEN_HOLE, ((HUB_X + OPEN_HOLE) / 2, 0.0, HZ), axis="x", segments=24)),
            m.col_hull("hub_c", _hub_slice(OPEN_HOLE, -OPEN_HOLE, OPEN_HOLE, 1.0)),
            m.col_hull("hub_d", _hub_slice(OPEN_HOLE, -OPEN_HOLE, OPEN_HOLE, -1.0)),
            m.col_hull("shaft_c", m.cylinder(SR, SHAFT_X1 - HUB_X, ((HUB_X + SHAFT_X1) / 2, 0.0, HZ), axis="x", segments=16)),
            m.col_box("ear_lc", (0.030, 0.006, 0.024), (-HUB_X - 0.015, 0.009, HZ)),
            m.col_box("ear_rc", (0.030, 0.006, 0.024), (-HUB_X - 0.015, -0.009, HZ))]
    zc = HT - D0 + PIN_L / 2
    pin_parts = [("pin", m.lathe([(0.0, zc - PIN_L / 2), (PIN_R - 0.0004, zc - PIN_L / 2), (PIN_R, zc - PIN_L / 2 + 0.0005),
                                  (PIN_R, zc + PIN_L / 2 - 0.0005), (PIN_R - 0.0004, zc + PIN_L / 2), (0.0, zc + PIN_L / 2)]), "bright"),
                 ("slot", m.box((0.0006, 0.0006, PIN_L - 0.001), (0.0, -PIN_R + 0.0002, zc)), "black")]
    if PILOT:
        pin_parts.append(("bore", m.cylinder(PIN_ID / 2, 0.0002, (0.0, 0.0, zc + PIN_L / 2 + 0.0001), segments=16), "black"))
    # The pin is a split tube: solid below, and open at its upper end (2.4 mm inside), where a roll-pin punch's pilot
    # sits. Eight wedges make the open end's wall.
    pin_cols = [m.col_cylinder("pin_c", PIN_R, PIN_L - PIN_CUP, (0.0, 0.0, zc - PIN_CUP / 2))]
    if not PILOT:
        pin_cols = [m.col_cylinder("pin_c", PIN_R, PIN_L, (0.0, 0.0, zc))]
    for k in range(8 if PILOT else 0):
        pts = []
        for a in (math.radians(45.0 * k - 21.0), math.radians(45.0 * k + 21.0)):
            for r in (PIN_ID / 2, PIN_R):
                for z in (zc + PIN_L / 2 - PIN_CUP, zc + PIN_L / 2):
                    pts.append((r * math.cos(a), r * math.sin(a), z))
        pin_cols.append(m.col_hull(f"pin_w{k}", _hull_mesh(pts)))
    pin = m.slider("pin", "Z", -(D_MAX - D0), 0.001, pin_parts, pin_cols,
                   mass=0.01, damping=200.0, effort=3000.0, armature=0.05, friction=0.0)
    return m.mechanism("clevis", (MX, MY, TOP), [pin], LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.6, 0.5))


# -- arm geometry for the demo (also fixes where the tools lie) ---------------------------------------------------------
LEFT_BASE, RIGHT_BASE = np.array([0.0, 0.305]), np.array([0.0, -0.305])
_hl = math.atan2(MY - LEFT_BASE[1], MX - LEFT_BASE[0])  # the left gripper's heading while holding the punch
HL = np.array([math.cos(_hl), math.sin(_hl), 0.0])
CL = np.cross([0.0, 0.0, 1.0], HL)  # horizontal, square to it
_hr = math.atan2(MY - RIGHT_BASE[1], MX - RIGHT_BASE[0])  # the right gripper's heading while striking
AP = np.array([math.cos(_hr), math.sin(_hr), 0.0])
U = np.array([-math.sin(_hr), math.cos(_hr), 0.0])  # along the handle, from the grip toward the head
Z = np.array([0.0, 0.0, 1.0])
P_AXIS_OFF = 0.0092  # the punch's axis lies this far beyond the TCP along the approach (measured in this simulation)
FOLLOW_GAP = 0.0005
DRIFT_XY = np.array([0.29, 0.20])  # see _left
P_SIDE_OFF = 0.0009  # ... and this far toward -CL: the firm grip holds it off centre (measured in this simulation)


def _axis_rot(axis, deg) -> np.ndarray:
    a = math.radians(deg)
    u = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def _quat(r) -> tuple:
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


def _level(approach, closing) -> np.ndarray:
    zz = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float) - np.dot(closing, zz) * zz
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(zz, x), zz])


R_HOLD = _level(HL, -CL)  # level, pointing at the bore from the left base, fingers closing square to that
R_PUNCH_PICK = _axis_rot(CL, 90.0) @ R_HOLD  # straight down, the same fingers
R_STRIKE = _level(AP, -Z)  # level, pointing at the bore from the right base, fingers closing vertically
R_HAMMER_PICK = _axis_rot(U, 90.0) @ R_STRIKE  # straight down, fingers closing along -AP
FACE_TRIM = -6.5  # deg about U: held, the face meets the punch tipped 4 deg toward the right arm unless the strokes are sent turned back (measured in this simulation)
R_STROKE = _axis_rot(U, FACE_TRIM) @ R_STRIKE
R_HAM_USE = np.column_stack([AP, U, Z])  # face down, handle back along -U
R_HAM_LYING = _axis_rot(U, 90.0) @ R_HAM_USE  # on its side
R_PUNCH_LYING = _axis_rot(CL, 90.0)  # point toward the left base, head toward the bore

PUNCH_PICK = LEFT_BASE + 0.22 * HL[:2]  # the TCP over the punch's body when it is taken
PUNCH_TIP0 = PUNCH_PICK - P_GRIP * HL[:2]
HAMMER_GRIP0 = RIGHT_BASE + 0.22 * AP[:2]
HEAD0 = HAMMER_GRIP0 + L_H * U[:2]


def _crown():
    """The struck end: a crown of 10 mm radius over the 9.6 mm body (about 1.2 mm high), as on a dressed punch head."""
    rs = [P_BODY_R, 0.004, 0.003, 0.002, 0.001, 0.0]
    return [(r, P_LEN - CROWN_R + math.sqrt(CROWN_R * CROWN_R - r * r)) for r in rs]


def _punch() -> m.Part:
    crown = _crown()
    prof = ([(0.0, -PILOT_L), (PILOT_R0, -PILOT_L), (PILOT_R1, 0.0)] if PILOT else [(0.0, 0.0)]) + [(P_PT_R - 0.0003, 0.0), (P_PT_R, 0.0004),
            (P_PT_R, P_PT_L), (P_BODY_R, P_TAPER_TOP)] + crown
    z_c = crown[0][1]
    cols = ([m.col_hull("pilot", m.lathe([(PILOT_R0, -PILOT_L), (PILOT_R1, 0.0)], 12))] if PILOT else []) + [
            m.col_cylinder("point", P_PT_R, P_PT_L, (0.0, 0.0, P_PT_L / 2)),
            m.col_hull("taper", m.lathe([(P_PT_R, P_PT_L), (P_BODY_R, P_TAPER_TOP)], 16)),
            m.col_cylinder("body", P_BODY_R, z_c - P_TAPER_TOP, (0.0, 0.0, (P_TAPER_TOP + z_c) / 2)),
            m.col_hull("crown", m.lathe(crown, 16))]
    q = _quat(R_PUNCH_LYING)
    return m.free("punch", (float(PUNCH_TIP0[0]), float(PUNCH_TIP0[1]), TOP + P_BODY_R + 0.0003),
                  [("punch", m.lathe(prof, 24), "blued")], cols, LOOKS, mass=P_MASS, start_quat=q, friction=(0.9, 0.8))


def _hammer() -> m.Part:
    head = m.join(m.lathe([(0.0, -FACE_D), (FACE_R - 0.0012, -FACE_D), (FACE_R, -FACE_D + 0.0012), (FACE_R, -0.026),
                           (0.0093, -0.020), (0.0090, -0.012), (0.0, -0.012)], 32),
                  m.box((0.018, 0.020, 0.024), (0.0, 0.0, 0.0)),
                  m.lathe([(0.0, 0.012), (0.0080, 0.012), (0.0072, 0.028), (0.0, 0.028)], 24),
                  m.sphere(0.0095, (0.0, 0.0, 0.036)))
    handle = m.box((HANDLE_W, HANDLE_Y0 - HANDLE_Y1, HANDLE_H), (0.0, (HANDLE_Y0 + HANDLE_Y1) / 2, 0.0))
    cols = [m.col_cylinder("face", FACE_R, 0.014, (0.0, 0.0, -FACE_D + 0.007)),
            m.col_cylinder("neck", 0.0093, 0.014, (0.0, 0.0, -0.019)),
            m.col_box("eye", (0.018, 0.020, 0.024), (0.0, 0.0, 0.0)),
            m.col_cylinder("peen", 0.0078, 0.016, (0.0, 0.0, 0.020)),
            m.col_hull("ball", m.sphere(0.0095, (0.0, 0.0, 0.036), rings=6, segments=12)),
            m.col_box("handle", (HANDLE_W, HANDLE_Y0 - HANDLE_Y1, HANDLE_H), (0.0, (HANDLE_Y0 + HANDLE_Y1) / 2, 0.0))]
    q = _quat(R_HAM_LYING)
    return m.free("hammer", (float(HEAD0[0]), float(HEAD0[1]), TOP + FACE_R + 0.0005), [("head", head, "steel"),
                  ("handle", handle, "hickory")], cols, LOOKS, mass=H_MASS, start_quat=q, friction=(0.9, 0.8),
                  com=(0.0, -0.056, -0.003))


WORK = _work()
PUNCH = _punch()
HAMMER = _hammer()
RAG = SimReady("rag", pos=(0.47, 0.25, TOP), size=(0.26, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01",
               usd="sm_cleaning_cloth_a01_01.usd", yaw=0.4)
OIL = SimReady("oil", pos=(0.50, -0.27, TOP), size=(0.052, 0.104, 0.272), mass=0.45,
               package="consumable_lubricant_oil_multi_purpose_a01", usd="sm_consumable_lubricant_oil_multi_purpose_a01_01.usd",
               yaw=0.5)
NUT = SimReady("nut", pos=(0.44, 0.08, TOP), size=(0.024, 0.028, 0.013), mass=0.03, package="factory_nut_m16_loose",
               usd="sm_factory_nut_m16_loose_01.usda", yaw=0.3)
OBJECTS = [WORK, PUNCH, HAMMER, RAG, OIL, NUT]


# -- the model ----------------------------------------------------------------------------------------------------------
def _engaged(d: float) -> float:
    """Length of pin in the bores (m) with its lower end d below the top of the hub."""
    return max(0.0, min(d, W) - max(d - PIN_L, 0.0))


def _driven(d: float, energy: float) -> float:
    """The depth after a blow of ``energy`` J: the bore walls' friction K x engaged length takes the energy."""
    x, e, step = d, energy, 2e-5
    while e > 0.0 and x < D_MAX:
        f = PIN_K * max(_engaged(x), 0.0005)
        e -= f * step
        x += step
    return x


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.d = D0
        self.bound = False
        self.marked = ""
        self.armed_p = self.armed_h = True
        self.prev = None
        self.blows = []
        self.wrapped_env = None
        self.score = 0.0
        self.ok = False
        self.step_n = 0
        self.info = {}
        if env is not None:
            m.lock(env, "clevis", "pin", stiffness=200000.0, effort=3000.0)
            self._wrap(env)

    # -- readers --
    def _read(self, base):
        def pose(name):
            p = m._np(base.scene[name].data.root_link_pose_w)[0]
            v = m._np(base.scene[name].data.root_link_vel_w)[0]
            return p[:3] - m._np(base.scene.env_origins[0]), m.quat_matrix(p[3:7]), v
        hp, hr, hv = pose("hammer")
        pp, pr, pv = pose("punch")
        q = float(m._np(base.scene["clevis"].data.joint_pos)[0][0])
        face_l = np.array([0.0, 0.0, -FACE_D])
        face = hp + hr @ face_l
        vface = hv[:3] + np.cross(hv[3:6], hr @ face_l)
        head_l = np.array([0.0, 0.0, P_LEN])
        phead = pp + pr @ head_l
        vphead = pv[:3] + np.cross(pv[3:6], pr @ head_l)
        return dict(hp=hp, hr=hr, hv=hv, face=face, n=-hr[:, 2], vface=vface, pp=pp, pr=pr, axis=pr[:, 2],
                    phead=phead, vphead=vphead, q=q)

    def _wrap(self, env):
        base = getattr(env, "unwrapped", env)
        if getattr(base.scene, "_pin_wrapped", False):
            return
        orig = base.scene.update

        def update(dt, _orig=orig, _base=base):
            _orig(dt)
            self._substep(_base)

        base.scene.update = update
        base.scene._pin_wrapped = True
        self._env = env

    # -- the blow model, after every physics step (240 Hz) --
    def _substep(self, base):
        s = self._read(base)
        prev = self.prev
        self.prev = s
        if prev is None:
            return
        pin_top = np.array([MX, MY, TOP + HT + PIN_L - self.d])
        # blows on the punch's head
        rel = s["phead"] - s["face"]
        sep = float(rel @ s["n"])
        lat = float(np.linalg.norm(rel - sep * s["n"]))
        if sep > REARM or lat > FACE_R + 0.01:
            self.armed_p = True
        if self.armed_p and sep < 0.0008 and lat < FACE_R + 0.002:
            self.armed_p = False
            v_rel = prev["vface"] - prev["vphead"]
            v_in = float(-(v_rel @ prev["axis"]))
            if v_in >= V_MIN:
                self._punch_blow(s, prev, v_in, pin_top)
        # blows of the face directly on the pin's upper end
        rel = pin_top - s["face"]
        sep = float(rel @ s["n"])
        lat = float(np.linalg.norm(rel - sep * s["n"]))
        if sep > REARM or lat > FACE_R + 0.01:
            self.armed_h = True
        if self.armed_h and sep < 0.0008 and lat < FACE_R and self.d < PIN_L:
            self.armed_h = False
            v_in = float(-prev["vface"][2])
            if v_in >= V_MIN:
                self._direct_blow(s, prev, v_in)
        # any part of the hammer on the clevis or shaft, moving
        speed = max(float(np.linalg.norm(s["hv"][:3])), float(np.linalg.norm(s["vface"])))
        if speed >= 0.1 and not self.marked:
            pts = s["hp"] + _HAMMER_PTS @ s["hr"].T
            if _clevis_gap(pts) < 0.001:
                self.marked = f"the hammer struck the clevis at {speed:.2f} m/s"

    def _energy(self, v_in):
        return 0.5 * H_MASS * v_in * v_in

    def _punch_blow(self, s, prev, v_in, pin_top):
        tip = prev["pp"]
        off = float(np.hypot(tip[0] - MX, tip[1] - MY))
        gap = float(tip[2] - pin_top[2])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(prev["axis"][2])))))
        e = self._energy(v_in)
        fo = prev["face"] - prev["phead"]
        rec = dict(kind="punch", step=self.step_n, v=round(v_in, 3), e=round(e, 4), off=round(off * 1000, 2),
                   gap=round(gap * 1000, 2), tilt=round(tilt, 2), d0=round(self.d * 1000, 2),
                   face_ap_u=(round(float(fo @ AP) * 1000, 1), round(float(fo @ U) * 1000, 1)),
                   n_ap_u=(round(math.degrees(math.asin(float(prev["n"] @ AP))), 1), round(math.degrees(math.asin(float(prev["n"] @ U))), 1)),
                   punch_ap_u=(round(math.degrees(math.asin(float(prev["axis"] @ AP))), 1), round(math.degrees(math.asin(float(prev["axis"] @ U))), 1)))
        on_pin = SEAT_GAP_LO <= gap <= SEAT_GAP_HI
        if on_pin and off <= SEAT_R:
            if self.bound:
                rec["out"] = "bound"
            elif tilt > TILT_MAX:
                self.bound = True
                rec["out"] = "cocked (tilt)"
            else:
                self.d = _driven(self.d, e)
                self._set_pin()
                rec["out"] = "driven"
        elif on_pin and off <= EDGE_R:
            if not self.bound:
                self.bound = True
            rec["out"] = "cocked (off-centre)"
        elif _on_hub(tip):
            if not self.marked:
                self.marked = "the punch struck the hub beside the pin"
            rec["out"] = "punch on the hub"
        else:
            rec["out"] = "punch not on the work"
        rec["d1"] = round(self.d * 1000, 2)
        self.blows.append(rec)
        if DEBUG:
            rec["tcpL"] = _tcp(self._env.unwrapped, "left")
            print("PINDBG BLOW", rec, flush=True)

    def _direct_blow(self, s, prev, v_in):
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(-prev["n"][2])))))
        e = self._energy(v_in)
        rec = dict(kind="direct", step=self.step_n, v=round(v_in, 3), e=round(e, 4), tilt=round(tilt, 2),
                   d0=round(self.d * 1000, 2))
        if self.bound:
            rec["out"] = "bound"
        elif tilt > TILT_MAX:
            self.bound = True
            rec["out"] = "cocked (face off square)"
        else:
            d1 = _driven(self.d, e)
            if d1 > PIN_L - FACE_STOP:
                d1 = PIN_L - FACE_STOP
                if not self.marked:
                    self.marked = "the hammer's face landed on the hub"
                rec["out"] = "face on the hub"
            else:
                rec["out"] = "driven"
            self.d = max(self.d, d1)
            self._set_pin()
        rec["d1"] = round(self.d * 1000, 2)
        self.blows.append(rec)
        if DEBUG:
            print("PINDBG BLOW", rec, flush=True)

    def _set_pin(self):
        m.set_target(self._env, "clevis", "pin", -(self.d - D0))

    # -- per control step --
    def step(self, env):
        base = getattr(env, "unwrapped", env)
        self.step_n += 1
        s = self._read(base)
        d = D0 - s["q"]  # the pin's measured depth
        top_proud, bottom_proud = PIN_L - d, d - W
        flush = top_proud <= PROUD_TOL and bottom_proud <= PROUD_TOL
        harm = bool(self.marked) or self.bound
        tips = m.fingertips(env)
        bore_top = np.array([MX, MY, TOP + HT])
        hands = float(np.linalg.norm(tips - bore_top, axis=1).min())
        pin_top = np.array([MX, MY, TOP + HT + PIN_L - d])
        punch_gap = float(np.linalg.norm(s["pp"] - pin_top))
        ham_gap = _clevis_gap(s["hp"] + _HAMMER_PTS @ s["hr"].T, hub_only=True)
        still = abs(float(m._np(base.scene["clevis"].data.joint_vel)[0][0])) < 0.002
        clear = hands >= CLEAR_HANDS and punch_gap >= CLEAR_PUNCH and ham_gap >= CLEAR_HAMMER
        level = 0.75 if flush else 0.5 if d >= D_FAR else 0.25 if d >= D_SHAFT else 0.0
        if harm:
            level = min(level, 0.25)
        self.ok = bool(flush and not harm and clear and still)
        self.score = float(level)
        self.info = dict(d=d, top=top_proud, bottom=bottom_proud, hands=hands, punch_gap=punch_gap, ham_gap=ham_gap,
                         marked=self.marked, bound=self.bound)
        if DEBUG and (self.step_n % 10 == 0):
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(s["axis"][2])))))
            off = float(np.hypot(s["pp"][0] - MX, s["pp"][1] - MY))
            print(f"PINDBG step={self.step_n} d={d * 1000:.2f} cmd={self.d * 1000:.2f} tip=({(s['pp'][0] - MX) * 1000:.1f},"
                  f"{(s['pp'][1] - MY) * 1000:.1f},{(s['pp'][2] - pin_top[2]) * 1000:.1f}) off={off * 1000:.2f} "
                  f"tilt={tilt:.2f} face=({(s['face'][0] - MX) * 1000:.1f},{(s['face'][1] - MY) * 1000:.1f},"
                  f"{(s['face'][2] - TOP) * 1000:.1f}) facetilt={math.degrees(math.acos(max(-1, min(1, float(-s['n'][2]))))):.1f} "
                  f"phead_z={(s['phead'][2] - TOP) * 1000:.1f} hands={hands:.3f} pgap={punch_gap:.3f} hgap={ham_gap:.3f} "
                  f"marked={self.marked or '-'} bound={self.bound} score={self.score} ok={self.ok} "
                  f"fingers={_fingers(base)} tcpL={_tcp(base, 'left')} tcpR={_tcp(base, 'right')}", flush=True)


def _tcp(base, side):
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    r = base.scene[f"{side}_robot"]
    q = m._np(r.data.joint_pos)[0][list(r.find_joints(ARM_JOINTS, preserve_order=True)[0])]
    p, _ = ik.tcp_pose(q, station.BASES[side])
    return tuple(round(float(v) * 1000, 1) for v in (p[0] - MX, p[1] - MY, p[2] - TOP))


def _fingers(base):
    out = []
    for side in ("left", "right"):
        r = base.scene[f"{side}_robot"]
        out.append(round(float(m._np(r.data.joint_pos)[0, r.find_joints("joint7")[0][0]]) * 1000, 2))
    return out


_rim = [(FACE_R * math.cos(a), FACE_R * math.sin(a), -FACE_D) for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
_HAMMER_PTS = np.array(_rim + [(0.0, 0.0, -FACE_D)]
                       + [(sx * 0.009, sy * 0.010, z) for sx in (-1, 1) for sy in (-1, 1) for z in (-0.012, 0.0, 0.012)]
                       + [(sx * 0.0095, 0.0, 0.036) for sx in (-1, 1)] + [(0.0, 0.0, 0.0455)]
                       + [(sx * HANDLE_W / 2, y, sz * HANDLE_H / 2) for y in np.linspace(HANDLE_Y0, HANDLE_Y1, 12)
                          for sx in (-1, 1) for sz in (-1, 1)])


def _clevis_gap(pts, hub_only: bool = False) -> float:
    """Smallest distance (m) from world points to the clevis hub, the shaft and the fork's ears (negative inside)."""
    p = pts - np.array([MX, MY, TOP])
    best = 1.0
    rad = np.hypot(p[:, 1], p[:, 2] - HZ)
    dx = np.maximum(np.abs(p[:, 0]) - HUB_X, 0.0)
    best = min(best, float(np.min(np.hypot(dx, np.maximum(rad - HR, 0.0)) + np.minimum(rad - HR, 0.0) * (dx == 0))))
    if hub_only:
        return best
    dxs = np.maximum(np.maximum(HUB_X - p[:, 0], p[:, 0] - SHAFT_X1), 0.0)
    best = min(best, float(np.min(np.hypot(dxs, np.maximum(rad - SR * 0 - SR, 0.0)))))
    for yc in (0.009, -0.009):
        c = np.array([-HUB_X - 0.015, yc, HZ])
        h = np.array([0.015, 0.003, 0.012])
        q = np.abs(p - c) - h
        best = min(best, float(np.min(np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0))))
    return best


def _on_hub(tip) -> bool:
    """The punch's point rests on the hub (within 1.5 mm of its top surface)."""
    p = np.asarray(tip) - np.array([MX, MY, TOP])
    if abs(p[0]) > HUB_X or abs(p[1]) >= HR:
        return False
    zs = HZ + math.sqrt(HR * HR - p[1] * p[1])
    return p[2] - zs < 0.0015


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.ok)


# -- scripted demo ------------------------------------------------------------------------------------------------------
# The left gripper takes the punch from the table by its body, turns it point-down and sets its point on the pin's
# end, holding it lightly so that it can slide down after the pin; the right takes the hammer by its handle close to
# its head, turns it face-down over the punch and strikes the punch's head with short downward strokes of the whole
# arm. The demo is open loop: the left gripper is lowered after each blow by the depth the model gives for a blow of
# the demo's measured energy, and the last strokes are lighter taps. Then both draw back and hold the tools clear.
OPEN, CLOSED = -0.040, 0.0
P_SQUEEZE = 0.0  # the left fingers hold the punch firmly (about 23 N each; they meet its 9.6 mm body at -4.7 mm)
MODE = os.environ.get("PIN_MODE", "full")  # red-team variants: none, onearm, careless, press


def _good_ik(arm, pos, rot, seeds=None):
    from yam_arena import ik, station

    best = None
    rng = np.random.default_rng(1)
    pool = list(seeds or []) + [ik.LOWER + rng.random(6) * (ik.UPPER - ik.LOWER) for _ in range(60)]
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


# Strokes: (raise of the face above the punch's head, m; how far below the head the grip is driven, m).
E_PLAN = 0.12  # J per full stroke, as measured in this simulation (sets the left gripper's lowering schedule)
E_TAP = 0.025  # J per tap
FULL = (0.050, 0.015)
TAP = (0.030, 0.015)
N_FULL, N_TAP = 11, 0
BLOW_S, TAP_S = 0.55, 0.50
SAG = 0.022  # the arm holding the hammer settles about this far below where it is sent (measured in this simulation)
T_SET = 15.5  # by then the punch stands on the pin


def _blow_plan():
    """(energy planned, raise, below, seconds, label) for each stroke of this mode."""
    out = [(E_PLAN, *FULL, BLOW_S, f"blow {k + 1}") for k in range(N_FULL)]
    out += [(E_TAP, *TAP, TAP_S, f"tap {k + 1}") for k in range(N_TAP)]
    if MODE == "careless":  # keeps on striking after the pin is home
        out += [(E_PLAN, *FULL, BLOW_S, f"extra blow {k + 1}") for k in range(6)]
    return out


def _plan_depths():
    d, out = D0, []
    for e, *_ in _blow_plan():
        d = min(_driven(d, e), D_MAX)
        out.append(d)
    return out


def _idle(arm):
    y = 0.25 if arm == "left" else -0.25
    return [Waypoint(60.0, (0.15, y, TOP + 0.25), OPEN, "stay")]


def _left() -> list:
    pin_top = TOP + HT + PIN_L - D0
    pick = np.array([PUNCH_PICK[0], PUNCH_PICK[1], TOP + P_BODY_R + P_AXIS_OFF])
    out = _joint_path("left", np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]), _good_ik("left", pick + [0, 0, 0.06], R_PUNCH_PICK),
                      OPEN, "above the punch", 3.0)
    out += [_w(1.0, pick, OPEN, "fingers either side of the punch's body", R_PUNCH_PICK),
            _w(0.8, pick, P_SQUEEZE, "take the punch", R_PUNCH_PICK),
            _w(0.4, pick, P_SQUEEZE, "settle", R_PUNCH_PICK),
            _w(1.2, pick + [0, 0, 0.14], P_SQUEEZE, "lift it", R_PUNCH_PICK)]

    def hold_at(tip_z):  # the TCP that puts the punch's point on the bore's axis at tip_z
        return np.array([MX, MY, tip_z + P_GRIP]) - P_AXIS_OFF * HL + P_SIDE_OFF * CL

    above = hold_at(pin_top + 0.012)
    turn_at = hold_at(pin_top + 0.07) - TURN_BACK * HL
    turn_at[2] = TOP + LIFT_Z
    q_lift = _good_ik("left", turn_at, R_PUNCH_PICK)
    q_turned = _good_ik("left", turn_at, R_HOLD)
    out += [_w(1.2, turn_at, P_SQUEEZE, "carry it high, short of the work", R_PUNCH_PICK)]
    out += _joint_path("left", q_lift, q_turned, P_SQUEEZE, "turn it point-down, clear of the work", 2.0, 16)
    out += [_w(1.6, hold_at(pin_top + 0.07), P_SQUEEZE, "over the pin, high", R_HOLD),
            _w(1.0, above, P_SQUEEZE, "lower it over the pin", R_HOLD),
            _w(0.5, above, P_SQUEEZE, "settle", R_HOLD),
            _w(1.2, hold_at(pin_top - 0.0005), P_SQUEEZE, "point onto the pin's end", R_HOLD)]
    if _dur(out) < T_SET:
        out += _stay(out, T_SET - _dur(out), "hold the punch on the pin")
    out += _stay(out, _right_lead_s() - _dur(out), "hold the punch on the pin")
    if MODE == "press":
        return out + _stay(out, 12.0, "hold the punch on the pin")
    # Follow the pin down as it is driven: hold still through each stroke, then lower the punch by the depth the model
    # gives for the measured blow, to 0.5 mm short of where the plan has the pin's end (a blow pushes the punch down
    # through the fingers after the pin). Lowered, the arm's joints lag about 0.2 mm sideways per mm (toward -x, -y,
    # measured in this simulation), so each lowered target is sent that much the other way.
    placed = hold_at(pin_top - 0.0005)

    def follow(d):
        p = hold_at(TOP + HT + PIN_L - d + (-0.0005 if d == D0 else FOLLOW_GAP))
        p[:2] += DRIFT_XY * (placed[2] - p[2])
        return p

    d = D0
    for (e, raise_m, below, s, label), d1 in zip(_blow_plan(), _plan_depths()):
        out += [_w(s - 0.12, follow(d), P_SQUEEZE, f"keep it on the pin ({label})", R_HOLD),
                _w(0.12, follow(d1), P_SQUEEZE, "follow the pin down", R_HOLD)]
        d = d1
    out += _stay(out, 1.0, "hold while the hammer lifts away")
    last = np.array(out[-1].pos)
    out += [_w(1.2, last + [0, 0, 0.06], P_SQUEEZE, "lift the punch off the pin", R_HOLD),
            _w(1.2, last + [0, 0, 0.06] - 0.08 * HL, P_SQUEEZE, "draw it back, clear", R_HOLD)]
    return out + _stay(out, 20.0, "hold it clear")


AIM_AP, AIM_U = -0.0080, 0.0015  # a stroke lands this much further out along AP than it is sent (measured in this simulation)
_STRIKE_XY = np.array([MX, MY]) - L_H * U[:2] + AIM_AP * AP[:2] + AIM_U * U[:2]  # the grip while striking


def _grip_z(head_z: float) -> float:
    """The height to send the grip to for the face to rest on a punch head at head_z (handle level)."""
    return head_z + FACE_D + SAG


def _stroke(head_z, raise_m, below_m, seconds, label) -> list:
    top = np.array([*_STRIKE_XY, _grip_z(head_z) + raise_m])
    low = np.array([*_STRIKE_XY, _grip_z(head_z) - below_m])
    rest = np.array([*_STRIKE_XY, _grip_z(head_z) + 0.015])
    r = R_STROKE
    hold, lift = 0.15, 0.15
    # no follow-through: the arm is sent back up as soon as the face can have met the head, so that it does not lean
    # on the punch after the blow (leaning on it tips the punch in the left gripper's fingers)
    return [_w(seconds - hold - 0.05 - lift, top, CLOSED, f"raise ({label})", r), _w(hold, top, CLOSED, "hold", r),
            _w(0.05, low, CLOSED, f"strike ({label})", r), _w(lift, rest, CLOSED, "lift off", r)]


_LEAD = {}


def _right_lead() -> list:
    if "wps" in _LEAD:
        return _LEAD["wps"]
    pick = np.array([HAMMER_GRIP0[0], HAMMER_GRIP0[1], TOP + HANDLE_W / 2 + 0.0035 + 0.006])
    out = _joint_path("right", np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]), _good_ik("right", pick + [0, 0, 0.07], R_HAMMER_PICK),
                      OPEN, "above the hammer's handle", 3.0)
    out += [_w(1.0, pick, OPEN, "fingers either side of the handle", R_HAMMER_PICK),
            _w(0.8, pick, CLOSED, "take the handle close to the head", R_HAMMER_PICK),
            _w(0.4, pick, CLOSED, "settle", R_HAMMER_PICK),
            _w(1.0, pick + [0, 0, 0.10], CLOSED, "lift it", R_HAMMER_PICK)]
    head0 = TOP + HT + PIN_L - D0 + P_LEN
    high = np.array([*_STRIKE_XY, _grip_z(head0) + 0.08])
    stage = high - 0.10 * AP
    if _dur(out) < T_SET - 0.5:
        out += _stay(out, T_SET - 0.5 - _dur(out), "wait for the punch to be set")
    q_lift = _good_ik("right", pick + [0, 0, 0.10], R_HAMMER_PICK)
    q_stage = _good_ik("right", stage, R_STROKE)
    out += _joint_path("right", q_lift, q_stage, CLOSED, "turn it face-down", 1.8, 15)
    out += [_w(1.2, high, CLOSED, "over the punch, high", R_STROKE),
            _w(1.0, np.array([*_STRIKE_XY, _grip_z(head0) + 0.03]), CLOSED, "face over the punch's head", R_STROKE)]
    _LEAD["wps"] = out
    return out


def _right_lead_s() -> float:
    return _dur(_right_lead())


def _right() -> list:
    out = list(_right_lead())
    head0 = TOP + HT + PIN_L - D0 + P_LEN
    if MODE == "onearm":  # no punch: strike the pin's end directly with the hammer's face
        d = D0
        for k in range(24):
            out += _stroke(TOP + HT + PIN_L - d, FULL[0], FULL[1], BLOW_S, f"direct blow {k + 1}")
            d = min(_driven(d, E_PLAN), PIN_L - FACE_STOP)
        return out + _stay(out, 3.0, "wait")
    if MODE == "press":  # no blows: lower the face slowly onto the punch's head and lean on it
        over = np.array([*_STRIKE_XY, _grip_z(head0) + 0.02])
        press = np.array([*_STRIKE_XY, _grip_z(head0) - 0.02])
        out += [_w(3.0, press, CLOSED, "lower the face onto the punch slowly and press", R_STRIKE),
                _w(3.0, press, CLOSED, "keep pressing", R_STRIKE), _w(1.5, over, CLOSED, "lift", R_STRIKE)]
        return out + _stay(out, 5.0, "wait")
    d = D0
    for (e, raise_m, below, s, label), d1 in zip(_blow_plan(), _plan_depths()):
        out += _stroke(head0 - (d - D0), raise_m, below, s, label)
        d = d1
    high = np.array([*_STRIKE_XY, _grip_z(head0) + 0.08])
    out += [_w(0.8, high, CLOSED, "lift the hammer away", R_STRIKE),
            _w(1.2, high - 0.12 * AP, CLOSED, "draw it back, clear", R_STRIKE)]
    return out + _stay(out, 20.0, "hold it clear")


def demo():
    if MODE == "none":
        return {"left": _idle("left"), "right": _idle("right")}
    if MODE == "onearm":
        return {"left": _idle("left"), "right": _right()}
    return {"left": _left(), "right": _right()}
