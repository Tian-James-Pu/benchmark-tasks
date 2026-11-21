"""A care-training manikin head (adult, life size, rigid vinyl) stands on its neck base on the table, facing the robot's right, so
that its right ear faces the arms. A behind-the-ear (BTE) hearing aid with a slim tube and an open dome stands upright in its open
moulded case on the table to the robot's left, as it is put away at night.

The right ear: the canal opens in the side of the head. Its first 6 mm run inward and 30 degrees toward the back of the head; then it
turns 30 degrees and runs straight in. The bore is 9 mm tall, 8 mm across in the first part and 7.5 mm after the bend. Above the
canal, 24 mm up, the top of the ear's root is a groove 10.5 mm wide between the scalp and the back of the upper ear rim, where a BTE
aid's body rests. The upper rim stands 6 mm above the groove's floor.

The hearing aid: a 28 x 8 x 11 mm body, a slim tube preformed to run from the body's front down in front of the ear and into the
canal, and a soft open dome on its end. Worn correctly, the body rests in the groove on top of the ear and the dome sits 8 to 12 mm
inside the canal, past the bend. The tube is flexible (a six-way sprung joint between the body and the tube's end); the dome is soft
(a sprung slider: pushed hard against the canal wall it folds back).

Model (declared in code):
- Head ("head"): fixed. Colliders: the skull, the canal's walls (two straight segments with the 30 degree bend), the ear root's groove
  floor and the upper rim, and the back of the ear (pinna plate and lobe). The concha's rim and the tragus are left out of the
  colliders, so that fingers can reach the canal entrance (see fidelity).
- Hearing aid ("aid"): a free articulation. Its frame is the body. Links: tx, ty, tz (sliders) and yaw, pitch, tip (hinges about the
  tube's end) carry the tube's end with its grip, the dome adaptor and the dome: the tube's flexibility, a spring toward its preformed
  shape on each of the six. The dome is a slider along the tube's end with a spring (DOME_K): compressed DOME_FOLD or more, it has
  folded (latched for the trial).
- Case ("case"): fixed; the body stands in a well like the ear's groove, and the tube and dome hang into the case's open front.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

from yam_arena import demo as _demo
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001

# -- the head and the ear (head frame H: origin at the canal entrance's centre, x into the head, y toward the back of the head,
#    z up; millimetres in the tables below) ------------------------------------------------------------------------------------

PSI = -5.0  # the head's frame turned this far about the vertical from the world's: x into the head is world x turned -5 degrees
E_WORLD = np.array([0.268, -0.022, TOP + 0.121])  # the canal entrance's centre
CANAL_TURN = 30.0  # the first segment runs this far toward the back of the head; the second runs straight in (H x)
L1 = 6.0  # first segment's length along its axis, mm
L2 = 14.0  # second segment's length (modelled; the real canal goes on to the eardrum at about 25 mm)
HALF_1 = 4.0  # first segment's bore half-width, mm
HALF_2 = 3.75  # second segment's
HALF_H = 4.5  # both segments' bore half-height
D1 = np.array([math.cos(math.radians(CANAL_TURN)), math.sin(math.radians(CANAL_TURN)), 0.0])
D2 = np.array([1.0, 0.0, 0.0])
B1 = L1 * D1  # the bend's point on the centreline
CAP_X = B1[0] + L2
ROOT_Z = 24.0  # the ear root groove's floor, above the canal's centre
GROOVE = (-10.5, 0.0)  # the groove between the upper rim's inner face and the scalp, H x
RIM_TOP = ROOT_Z + 6.0

SKIN = "skin"
LOOKS = {
    SKIN: m.Look(color=m.rgb(0.86, 0.71, 0.60), roughness=0.55),
    "ear": m.Look(color=m.rgb(0.84, 0.66, 0.56), roughness=0.55),
    "canal": m.Look(color=m.rgb(0.25, 0.12, 0.10), roughness=0.8),
    "stand": m.Look(color=m.rgb(0.30, 0.30, 0.32), roughness=0.5),
    "lips": m.Look(color=m.rgb(0.70, 0.45, 0.42), roughness=0.6),
    "lid": m.Look(color=m.rgb(0.72, 0.58, 0.50), roughness=0.6),
    "shell": m.Look(color=m.rgb(0.62, 0.60, 0.57), metallic=0.1, roughness=0.35),
    "tube": m.Look(color=m.rgb(0.92, 0.92, 0.90), roughness=0.3, opacity=0.85),
    "adapter": m.Look(color=m.rgb(0.25, 0.25, 0.26), roughness=0.4),
    "dome": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.3, opacity=0.8),
    "mark": m.Look(color=m.rgb(0.05, 0.25, 0.85), roughness=0.5),
    "case": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.45),
    "foam": m.Look(color=m.rgb(0.35, 0.36, 0.40), roughness=0.9),
}


def _q(p):
    return tuple(float(v) * MM for v in p)


def _ellipsoid(center, semi, rings=16, segments=32):
    s = m.sphere(1.0, (0, 0, 0), rings=rings, segments=segments)
    return m.Mesh(s.v * np.asarray(semi, float) * MM + np.asarray(center, float) * MM, s.f)


def _hbox(name, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return m.col_box(name, tuple((hi - lo) * MM), tuple((hi + lo) / 2 * MM))


def _quad(name, pts, z0, z1):
    """A convex prism over an xy outline (mm, counter-clockwise), from z0 to z1 (mm), as a hull collider."""
    return m.col_hull(name, m.prism([(x * MM, y * MM) for x, y in pts], z0 * MM, z1 * MM))


def _canal_corners():
    t = math.tan(math.radians(CANAL_TURN))
    w1 = HALF_1 / math.cos(math.radians(CANAL_TURN))
    ant_y, post_y = B1[1] - HALF_2, B1[1] + HALF_2
    x_ant = (ant_y + w1) / t  # first segment's front wall meets the second's
    x_post = (post_y - w1) / t
    return w1, ant_y, post_y, x_ant, x_post


def _head():
    w1, ant_y, post_y, x_ant, x_post = _canal_corners()
    h = HALF_H
    cols = [
        _hbox("skull_core", (22, -100, -110), (150, 95, 125)),
        _hbox("skull_upper", (0, -100, h), (22, 95, 125)),
        _hbox("skull_lower", (0, -100, -110), (22, 95, -h)),
        _hbox("skull_front", (0, -100, -h), (22, -30, h)),
        _hbox("skull_back", (0, 30, -h), (22, 95, h)),
        _quad("canal_front_1", [(0, -30), (x_ant, -30), (x_ant, ant_y), (0, -w1)], -h, h),
        _hbox("canal_front_2", (x_ant, -30, -h), (22, ant_y, h)),
        _quad("canal_back_1", [(0, w1), (x_post, post_y), (x_post, 30), (0, 30)], -h, h),
        _hbox("canal_back_2", (x_post, post_y, -h), (22, 30, h)),
        _hbox("canal_cap", (CAP_X, ant_y - 0.5, -h), (22, post_y + 0.5, h)),
        # the top of the ear's root: the groove's floor and the upper rim behind the body's place
        _hbox("root", (-17, 4, ROOT_Z - 4), (0, 40, ROOT_Z)),
        _hbox("rim", (-17, 7, ROOT_Z), (GROOVE[0], 31, RIM_TOP)),
        # the back of the ear: a plate flaring 25 degrees off the head from 16 mm behind the canal, and the lobe
        _quad("pinna", [(0, 16), (0, 20), (-12.7, 47.2), (-16.3, 45.5)], -30, ROOT_Z - 4),
        _hbox("lobe", (-9, 10, -30), (0, 28, -16)),
        # the neck and its base
        _hbox("neck", (25, -45, -121), (125, 60, -60)),
    ]
    parts = [
        ("cranium", _ellipsoid((75, 8, 35), (75, 98, 96)), SKIN),
        ("jaw", _ellipsoid((75, -52, -38), (52, 50, 52)), SKIN),
        ("nose", m.Mesh(np.array([[55, -146, 10], [95, -146, 10], [75, -146, 48], [75, -168, 4]], float) * MM,
                        np.array([[0, 1, 3], [1, 2, 3], [2, 0, 3], [0, 2, 1]])), SKIN),
        ("lips", m.box(_q((34, 6, 6)), _q((75, -101, -28))), "lips"),
        ("lid_r", m.box(_q((20, 4, 6)), _q((45, -138, 30))), "lid"),
        ("lid_l", m.box(_q((20, 4, 6)), _q((105, -138, 30))), "lid"),
        ("neck", m.cylinder(0.048, 0.072, _q((75, 8, -96))), SKIN),
        ("base", m.cylinder(0.070, 0.012, _q((75, 8, -115))), "stand"),
        # the ear: the back plate and lobe, the upper rim, the root, the canal opening
        ("pinna", m.prism([(0.0, 0.016), (0.0, 0.020), (-0.0127, 0.0472), (-0.0163, 0.0455)], -0.030, ROOT_Z * MM - 0.004), "ear"),
        ("helix", m.tube([_q((-13.5, 5, ROOT_Z + 2)), _q((-15, 18, RIM_TOP - 1)), _q((-15, 34, ROOT_Z + 1)),
                           _q((-16, 44, ROOT_Z - 10)), _q((-15, 46, -10)), _q((-11, 34, -26))], 0.003), "ear"),
        ("rim", m.box(_q((6.5, 24, 6)), _q((-13.75, 19, ROOT_Z + 3))), "ear"),
        ("root", m.box(_q((17, 36, 4)), _q((-8.5, 22, ROOT_Z - 2))), "ear"),
        ("lobe", m.box(_q((9, 18, 14)), _q((-4.5, 19, -23))), "ear"),
        ("canal", m.prism([(-0.0003, -w1 * MM), (0.0, -w1 * MM), (0.0, w1 * MM), (-0.0003, w1 * MM)], -h * MM, h * MM), "canal"),
    ]
    return m.fixed("head", tuple(E_WORLD), parts, cols, LOOKS, yaw_deg=PSI, friction=(0.6, 0.5))


def h2w(p_mm):
    """A point in the head's frame (mm) in the world (m)."""
    r = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(PSI))
    return E_WORLD + r @ (np.asarray(p_mm, float) * MM)


def w2h(p):
    r = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(PSI))
    return (r.T @ (np.asarray(p, float) - E_WORLD)) / MM


# -- the hearing aid (aid frame A: the head's axes, origin at the body's bottom centre; authored as worn) ------------------------

BODY = (8.0, 28.0, 11.0)  # mm: across, long, tall
BODY_WORN = np.array([-5.25, 19.0, ROOT_Z])  # the body's bottom centre in H, worn
BODY_MASS = 0.0030
TIP_MASS = 0.0003
DOME_MASS = 0.0001
LINK_MASS = 0.0002
TUBE_K, TUBE_C = 40.0, 0.3  # the tube end's stiffness and damping along each axis, N/m and N s/m
TUBE_KR, TUBE_CR = 0.010, 0.0002  # and about each axis, N m/rad and N m s/rad
DOME_K = 300.0  # the dome's axial spring, N/m
DOME_FOLD = 0.0020  # compressed this far, the dome has folded
DOME_TRAVEL = 0.0030
DOME_R = 3.3  # the dome's collider radius, mm (the 8 mm open dome as it sits compressed)
DOME_TIP = np.array([B1[0] + 4.0, B1[1], 0.0])  # worn: the dome's tip 10 mm in along the canal's centreline
DOME_C = DOME_TIP - np.array([2.5, 0.0, 0.0])
GRIP_X = (-10.0, -1.5)  # the straight tube end outside the canal, H x
TUBE_END = np.array([0.0, B1[1], 0.0])  # the tube's line in the canal and out of it: y = B1 y, z = 0
PIVOT = np.array([-6.0, B1[1], 0.0])  # the tube end's hinges turn about this point
TCP_ON_TIP = np.array([-15.0, B1[1], 0.0])  # where the right hand's TCP sits on the tube end (worn), H


def _a(p_h):
    """H (mm) to the aid's frame (m), as worn."""
    return tuple(float(v) for v in (np.asarray(p_h, float) - BODY_WORN) * MM)


def _tube_path():
    return [(-5.25, 5.0, ROOT_Z + 2.0), (-8.0, 2.6, ROOT_Z - 0.5), (-11.0, 2.2, 18.0), (-12.5, 2.4, 11.0), (-12.0, 2.8, 5.0),
            (-10.0, B1[1], 0.6), (GRIP_X[0], B1[1], 0.0)]


def _aid():
    bx, by, bz = BODY
    shell = m.join(m.box((bx * MM, (by - 6) * MM, bz * MM), (0, 0, bz / 2 * MM)),
                   m.cylinder(bx / 2 * MM, (by - 6) * MM, (0, 0, (bz - bx / 2) * MM), axis="y"),
                   m.sphere(bz / 2 * MM * 0.95, (0, -(by / 2 - 4.5) * MM, bz / 2 * MM)),
                   m.sphere(bz / 2 * MM * 0.95, (0, (by / 2 - 4.5) * MM, bz / 2 * MM)))
    shell = m.Mesh(shell.v * np.array([bx / bz, 1.0, 1.0]) + np.array([0.0, 0.0, 0.0]), shell.f)
    frame_parts = [("shell", shell, "shell"),
                   ("tube_top", m.tube([_a(p) for p in _tube_path()[:3]], 0.00065), "tube")]
    frame_cols = [m.col_box("body_c", (bx * MM, by * MM, bz * MM), (0, 0, bz / 2 * MM))]
    zero = (0.0, 0.0, 0.0)
    piv = _a(PIVOT)
    kt = dict(mass=LINK_MASS, spring=TUBE_K, damping=TUBE_C, friction=0.002, armature=2e-4, effort=20.0)
    kr = dict(mass=LINK_MASS, spring=TUBE_KR, damping=TUBE_CR, friction=1e-6, armature=1e-7, effort=5.0)
    tip_parts = [
        ("tube_mid", m.tube([_a(p) for p in _tube_path()[2:]], 0.00065), "tube"),
        ("tube_in", m.cylinder(0.00065, (TUBE_END[0] + 4.2 - GRIP_X[0]) * MM, _a(((GRIP_X[0] + 4.2) / 2, B1[1], 0)), axis="x"), "tube"),
        ("adapter", m.cylinder(0.0012, 0.004, _a((2.2, B1[1], 0)), axis="x"), "adapter"),
        ("mark", m.cylinder(0.0009, 0.0012, _a((-0.8, B1[1], 0)), axis="x"), "mark"),
    ]
    tip_cols = [m.col_box("grip_c", ((GRIP_X[1] - GRIP_X[0]) * MM, 0.002, 0.002), _a(((GRIP_X[0] + GRIP_X[1]) / 2, B1[1], 0)))]
    dome_mesh = m.lathe([(0.0, 0.0), (0.0012, 0.0), (0.0035, 0.0012), (0.0037, 0.0026), (0.0030, 0.0040), (0.0016, 0.0049), (0.0, 0.0050)])
    dome_mesh = dome_mesh.aimed("+X").moved(_a(DOME_TIP - np.array([5.0, 0.0, 0.0])))
    links = [
        m.slider("tx", "X", -0.03, 0.03, (), (), at=piv, **kt),
        m.slider("ty", "Y", -0.03, 0.03, (), (), at=piv, parent="tx", **kt),
        m.slider("tz", "Z", -0.03, 0.03, (), (), at=piv, parent="ty", **kt),
        m.hinge("yaw", "Z", piv, -75.0, 75.0, (), (), parent="tz", **kr),
        m.hinge("pitch", "Y", piv, -75.0, 75.0, (), (), parent="yaw", **kr),
        m.hinge("tip", "X", piv, -75.0, 75.0, tuple(tip_parts), tuple(tip_cols), parent="pitch",
                **{**kr, "mass": TIP_MASS}),
        m.slider("dome", "X", -DOME_TRAVEL, 0.0, (("dome", dome_mesh, "dome"),), (m.col_hull("dome_c", m.sphere(DOME_R * MM, _a(DOME_C), rings=6, segments=12)),),
                 at=_a(DOME_C), parent="tip", mass=DOME_MASS, spring=DOME_K, damping=0.05, friction=0.0, armature=1e-4, effort=20.0),
    ]
    ix = BODY_MASS / 12 * ((by * MM) ** 2 + (bz * MM) ** 2)
    iy = BODY_MASS / 12 * ((bx * MM) ** 2 + (bz * MM) ** 2)
    iz = BODY_MASS / 12 * ((by * MM) ** 2 + (bx * MM) ** 2)
    pos = CASE_WORLD + _rz(PSI) @ (np.asarray(BODY_IN_CASE) * MM) + np.array([0.0, 0.0, 0.0005])
    return m.mechanism("aid", tuple(pos), links, LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols, yaw_deg=PSI,
                       friction=(0.8, 0.6), solver_iters=96, free_mass=BODY_MASS, frame_inertia=(ix, iy, iz))


def _rz(deg):
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(deg))


# -- the case (same axes as the head; origin at its base's centre on the table) --------------------------------------------------

CASE_WORLD = np.array([0.165, 0.175, TOP])
BODY_IN_CASE = (-10.0, 8.0, 40.0)  # the body's bottom centre in the case, mm
WELL_FLOOR = 40.0


def _case():
    bx, ox, oy, _ = BODY[0], BODY_IN_CASE[0], BODY_IN_CASE[1], BODY_IN_CASE[2]
    half = BODY[1] / 2
    lo_x, hi_x = ox - bx / 2 - 1.25, ox + bx / 2 + 1.25  # the well's walls, 1.25 mm clear of the body
    cols = [
        _hbox("floor", (-35, -28, 0), (35, 28, 8)),
        _hbox("block", (lo_x, oy - half - 1, 8), (hi_x, oy + half + 2, WELL_FLOOR)),
        _hbox("wall_out", (lo_x - 3.5, oy - half - 1, 8), (lo_x, oy + half + 2, WELL_FLOOR + 6)),
        _hbox("wall_in", (hi_x, oy - half - 1, 8), (hi_x + 3.5, oy + half + 2, WELL_FLOOR + 6)),
        _hbox("side_l", (-35, 25, 8), (35, 28, 20)),
        _hbox("side_r", (-35, -28, 8), (35, -25, 20)),
        _hbox("side_b", (32, -28, 8), (35, 28, 20)),
        _hbox("side_f", (-35, -28, 8), (-32, 28, 20)),
    ]
    parts = [
        ("floor", m.box(_q((70, 56, 8)), _q((0, 0, 4))), "case"),
        ("sides", m.join(m.box(_q((70, 3, 12)), _q((0, 26.5, 14))), m.box(_q((70, 3, 12)), _q((0, -26.5, 14))),
                         m.box(_q((3, 56, 12)), _q((33.5, 0, 14))), m.box(_q((3, 56, 12)), _q((-33.5, 0, 14)))), "case"),
        ("foam", m.join(m.box(_q((hi_x - lo_x, BODY[1] + 3, WELL_FLOOR - 8)), _q(((lo_x + hi_x) / 2, oy + 0.5, (WELL_FLOOR + 8) / 2))),
                        m.box(_q((3.5, BODY[1] + 3, WELL_FLOOR - 2)), _q((lo_x - 1.75, oy + 0.5, (WELL_FLOOR + 14) / 2))),
                        m.box(_q((3.5, BODY[1] + 3, WELL_FLOOR - 2)), _q((hi_x + 1.75, oy + 0.5, (WELL_FLOOR + 14) / 2)))), "foam"),
    ]
    return m.fixed("case", tuple(CASE_WORLD), parts, cols, LOOKS, yaw_deg=PSI, friction=(0.8, 0.6))


HEAD = _head()
CASE = _case()
AID = _aid()

OBJECTS = [
    HEAD,
    CASE,
    AID,
    # Distractors on the bedside table (SimReady props), which scoring ignores. pos is each one's origin, on the table top.
    SimReady("glasses", pos=(0.43, -0.25, TOP), size=(0.145, 0.174, 0.057), mass=0.03, yaw=0.5,
             package="accessory_eyeglasses_a01", usd="sm_accessory_eyeglasses_a01_01.usd"),
    SimReady("cup", pos=(0.47, 0.20, TOP), size=(0.064, 0.064, 0.100), mass=0.02,
             package="dishware_plastic_cup_a01", usd="sm_dishware_plastic_cup_a01_01.usd", convex=True),
    SimReady("care_log", pos=(0.13, -0.24, TOP), size=(0.229, 0.170, 0.010), mass=0.12, yaw=0.2,
             package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd"),
]


# -- reading the hearing aid --------------------------------------------------------------------------------------------------

def _body_pose(environment, part, body):
    """A link's world pose (position, rotation matrix) and velocity."""
    base = environment.unwrapped
    asset = base.scene[part]
    origin = m._np(base.scene.env_origins[0])
    i = asset.body_names.index(body)
    pose = m._np(asset.data.body_link_pose_w)[0, i]
    vel = m._np(asset.data.body_link_vel_w)[0, i]
    return pose[:3] - origin, m.quat_matrix(pose[3:7]), vel


def _world(pose, p_h):
    """A point authored in H (mm, as worn) carried by a link with this pose."""
    pos, rot, _ = pose
    return pos + rot @ np.asarray(_a(p_h))


def canal_depth(p_h):
    """Depth (mm) along the canal's centreline of the point nearest to p (H, mm), and p's distance from the centreline.
    Negative depth: outside the entrance, along the first segment's axis."""
    p = np.asarray(p_h, float)
    t1 = min(float(p @ D1), L1)
    r1 = float(np.linalg.norm(p - t1 * D1))
    t2 = float(np.clip((p - B1) @ D2, 0.0, L2))
    r2 = float(np.linalg.norm(p - (B1 + t2 * D2)))
    return (t1, r1) if r1 <= r2 else (L1 + t2, r2)


BODY_CORNERS = np.array([[sx * BODY[0] / 2, sy * BODY[1] / 2, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, BODY[2])])
AID_POINTS_H = np.vstack([BODY_CORNERS + BODY_WORN, BODY_WORN + np.array([0, 0, BODY[2] / 2])])
TIP_POINTS_H = np.array([[GRIP_X[0], B1[1], 0.0], [GRIP_X[1], B1[1], 0.0], PIVOT, DOME_C, _tube_path()[3]])

DEPTH_LO, DEPTH_HI = 8.0, 12.0  # mm: the dome's tip past the entrance, along the centreline
DEPTH_IN = 3.0  # the dome is in the canal
DEPTH_HARM = 15.0  # pushed deeper than this: harm
ON_AXIS = 2.5  # the dome's tip within this of the centreline
ROOT_GAP = 5.0  # the body's lowest corner at most this far above the groove's floor
PATH_TOL = 5.0  # the tube's end within this of its fitted place on the body
CLEAR = 0.030  # every fingertip point this far from the aid
STILL_V, STILL_W = 0.005, 0.10
HOLD_S = 1.5
TILT_MAX = 20.0
OFF_TABLE = 0.03


def measure(environment):
    """This step's readings (all from poses and joints a camera or ruler could check)."""
    frame = _body_pose(environment, "aid", "frame")
    tip = _body_pose(environment, "aid", "tip")
    dome = _body_pose(environment, "aid", "dome")
    corners_h = np.array([w2h(frame[0] + frame[1] @ (c * MM)) for c in BODY_CORNERS])
    centre_h = w2h(frame[0] + frame[1] @ np.array([0, 0, BODY[2] / 2 * MM]))
    tilt = math.degrees(math.acos(float(np.clip(frame[1][2, 2], -1.0, 1.0))))
    on_root = bool(ROOT_Z - 1.0 <= corners_h[:, 2].min() <= ROOT_Z + ROOT_GAP and GROOVE[0] <= centre_h[0] <= GROOVE[1]
                   and 4.0 <= centre_h[1] <= 40.0 and tilt <= TILT_MAX)
    dome_tip_h = w2h(_world(dome, DOME_TIP))
    depth, off = canal_depth(dome_tip_h)
    piv_w = _world(tip, PIVOT)
    piv_in_body = frame[1].T @ (piv_w - frame[0])
    path_mm = float(np.linalg.norm(piv_in_body - np.asarray(_a(PIVOT)))) / MM
    tips = m.fingertips(environment)
    aid_pts = np.vstack([np.array([frame[0] + frame[1] @ np.asarray(_a(p)) for p in AID_POINTS_H]),
                         np.array([_world(tip, p) for p in TIP_POINTS_H])])
    gap = float(np.min(np.linalg.norm(tips[:, None, :] - aid_pts[None, :, :], axis=2)))
    still = bool(np.linalg.norm(frame[2][:3]) < STILL_V and np.linalg.norm(frame[2][3:]) < STILL_W and np.linalg.norm(dome[2][:3]) < STILL_V)
    st = m.state(environment, "aid")
    fold = -st.joint("dome")
    low = float(min(aid_pts[:, 2].min(), dome[0][2]))
    return dict(on_root=on_root, depth=depth, off=off, path=path_mm, gap=gap, still=still, fold=fold, low=low,
                body_low=float(corners_h[:, 2].min()), centre=centre_h, tilt=tilt)


class Trial:
    """Updated once per control step after the physics; score() and success() only read it."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):  # the ONLY place a recorded harm or fold is cleared
        self.harm = None
        self.folded = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}

    def step(self, environment):
        r = measure(environment)
        self.last = r
        if r["fold"] >= DOME_FOLD:
            self.folded = True
        if r["depth"] > DEPTH_HARM and r["off"] < 6.0:
            self.harm = self.harm or "dome pushed deeper than 15 mm"
        if r["low"] < TOP - OFF_TABLE:
            self.harm = self.harm or "hearing aid dropped off the table"
        released = r["gap"] >= CLEAR
        seated = DEPTH_LO <= r["depth"] <= DEPTH_HI and r["off"] <= ON_AXIS and not self.folded
        entered = r["depth"] >= DEPTH_IN and r["off"] <= 3.0
        on_path = r["path"] <= PATH_TOL
        ok = self.harm is None and r["on_root"] and released and seated and on_path and r["still"]
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= round(HOLD_S * 20)
        if self.harm is not None or not (r["on_root"] and released):
            value = 0.0
        elif seated and on_path and self.full:
            value = 1.0
        elif seated:
            value = 0.75
        elif entered:
            value = 0.5
        else:
            value = 0.25
        self.score = value


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)

SUCCESS_HOLD_S = 0.0  # the Trial counts its own 1.5 s hold
SCORE_OVER = "end"


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo -----------------------------------------------------------------------------------------------------

class Orient(float):
    """A waypoint 'yaw' that carries a whole TCP rotation."""

    rot = np.eye(3)


def _orient(r):
    o = Orient(0.0)
    o.rot = r
    return o


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


def tilted(theta_deg, phi_deg):
    """Tilted theta from straight down towards phi (from world +x), the fingers closing across that direction."""
    phi = math.radians(phi_deg)
    r = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain_grip(phi)
    return _orient(r)


LEFT_TILT = 30.0
LEFT_GRIP = tilted(LEFT_TILT, PSI)  # leaning out from the head, the fingertips toward the scalp, closing along the body's length
LEFT_TIP_H = np.array([-8.0, 19.0, ROOT_Z + 6.5])  # where the left fingertips' centre takes the body (worn), H
LEFT_OPEN = -0.020  # 40 mm: 6 mm clear of each end of the body
LEFT_SHUT = -(BODY[1] / 2 * MM - 8.0 / 5000.0)  # 8 N a finger on the body's ends


def _left_tcp(body_offset_h):
    """The left TCP for the body moved by body_offset_h (mm, H) from where it is worn."""
    approach = LEFT_GRIP.rot[:, 2]
    return h2w(LEFT_TIP_H + np.asarray(body_offset_h, float)) - 0.013 * approach


def _tip_pose(depth):
    """The tube end's pose (H) for the dome's tip `depth` mm along the canal: its tip point and the axis's angle toward the back."""
    if depth <= L1:
        p = depth * D1
    else:
        p = B1 + (depth - L1) * D2
    theta = CANAL_TURN * float(np.clip((9.0 - depth) / 4.0, 0.0, 1.0))
    return p, theta


def _right_at(depth, lift_mm=0.0, back_mm=0.0):
    """Right TCP and grip with the dome's tip at `depth` (mm) along the canal, raised lift_mm and drawn back back_mm along the tube."""
    p, theta = _tip_pose(depth)
    axis = np.array([math.cos(math.radians(theta)), math.sin(math.radians(theta)), 0.0])
    tcp_h = p - (DOME_TIP[0] - TCP_ON_TIP[0] + back_mm) * axis + np.array([0, 0, lift_mm])
    return h2w(tcp_h), tilted(90.0, PSI + theta)


def _right_worn_offset(offset_h, back_mm=0.0):
    """Right TCP on the tube end where the tube is at rest on a body moved by offset_h (mm, H)."""
    tcp_h = TCP_ON_TIP + np.asarray(offset_h, float) - np.array([back_mm, 0, 0])
    return h2w(tcp_h), tilted(90.0, PSI)


RIGHT_SHUT = FINGER_CLOSED  # on the 2 mm tube end: 5 N a finger
RIGHT_OPEN_NEAR = -0.008
PRE_DEPTH = -3.0  # the dome's tip 3 mm outside the entrance, on the first segment's axis


def demo():
    """Left: takes the aid's body from its case by its two ends, lifts it out and holds it beside and above the right ear. Right: takes
    the tube's end beside the dome, its fingers either side of it. Together: the left brings the body over the ear's root and sets it
    down in the groove on top of the ear, while the right holds the dome just outside the canal, on the canal's first axis. The left
    keeps the body there; the right feeds the dome in along the first segment and turns it 30 degrees through the bend as it goes on,
    to 10 mm. The right lets go and backs out along the tube; then the left lets go and lifts away."""
    case_body = CASE_WORLD + _rz(PSI) @ (np.asarray(BODY_IN_CASE) * MM)
    worn_body = h2w(BODY_WORN)
    in_case = w2h(case_body) - BODY_WORN  # the body's offset in the case from where it is worn, H mm
    stage = np.array([-38.0, -4.0, 42.0])  # beside and above the ear
    above = np.array([0.0, 0.0, 12.0])
    up = np.array([0.0, 0.0, 1.0])

    left = [
        Waypoint(2.5, tuple(_left_tcp(in_case + np.array([0, 0, 40.0]))), LEFT_OPEN, "over the aid in its case", LEFT_GRIP),
        Waypoint(1.2, tuple(_left_tcp(in_case)), LEFT_OPEN, "fingers either end of the body", LEFT_GRIP),
        Waypoint(0.6, tuple(_left_tcp(in_case)), LEFT_SHUT, "grip the body", LEFT_GRIP),
        Waypoint(1.2, tuple(_left_tcp(in_case + np.array([0, 0, 55.0]))), LEFT_SHUT, "lift it out", LEFT_GRIP),
        Waypoint(2.5, tuple(_left_tcp(stage)), LEFT_SHUT, "beside the ear", LEFT_GRIP),
    ]
    t_left = sum(w.duration for w in left)
    r_stage_back, g0 = _right_worn_offset(stage, back_mm=40.0)
    r_stage, _ = _right_worn_offset(stage)
    right = [
        Waypoint(t_left - 1.0, tuple(r_stage_back + np.array([0, 0, 0.03])), RIGHT_OPEN_NEAR * 2, "out beside the ear", g0),
        Waypoint(1.0, tuple(r_stage_back), RIGHT_OPEN_NEAR * 2, "in line with the tube's end", g0),
        Waypoint(1.5, tuple(r_stage), RIGHT_OPEN_NEAR * 2, "fingers either side of the tube's end", g0),
        Waypoint(0.6, tuple(r_stage), RIGHT_SHUT, "take the tube's end", g0),
    ]
    left += [Waypoint(3.1, tuple(_left_tcp(stage)), LEFT_SHUT, "hold while the right takes the tube", LEFT_GRIP)]
    # together: the body over the groove, the dome beside the canal; then both down
    r_pre_up, g_pre = _right_at(PRE_DEPTH, lift_mm=above[2], back_mm=0.0)
    r_pre, _ = _right_at(PRE_DEPTH)
    left += [
        Waypoint(2.5, tuple(_left_tcp(above)), LEFT_SHUT, "over the ear's root", LEFT_GRIP),
        Waypoint(1.5, tuple(_left_tcp(np.zeros(3))), LEFT_SHUT, "down into the groove on top of the ear", LEFT_GRIP),
        Waypoint(0.5, tuple(_left_tcp(np.zeros(3))), LEFT_SHUT, "hold the body on the ear", LEFT_GRIP),
    ]
    right += [
        Waypoint(2.5, tuple(r_pre_up), RIGHT_SHUT, "the dome above the canal", g_pre),
        Waypoint(1.5, tuple(r_pre), RIGHT_SHUT, "the dome at the entrance, on the first axis", g_pre),
        Waypoint(0.5, tuple(r_pre), RIGHT_SHUT, "", g_pre),
    ]
    # the right feeds the dome in, turning it through the bend; the left holds the body
    t_ins = 0.0
    depths = list(np.arange(PRE_DEPTH + 1.0, 10.01, 1.0))
    for d in depths:
        pos, g = _right_at(float(d))
        dt = 0.35
        right.append(Waypoint(dt, tuple(pos), RIGHT_SHUT, f"feed the dome to {d:.0f} mm", g))
        t_ins += dt
    right.append(Waypoint(0.8, tuple(_right_at(10.0)[0]), RIGHT_SHUT, "hold at depth", _right_at(10.0)[1]))
    right.append(Waypoint(0.6, tuple(_right_at(10.0)[0]), RIGHT_OPEN_NEAR, "let go of the tube", _right_at(10.0)[1]))
    out, g_out = _right_at(10.0, back_mm=40.0)
    right.append(Waypoint(1.2, tuple(out), RIGHT_OPEN_NEAR, "back out along the tube", g_out))
    right.append(Waypoint(1.5, tuple(out + np.array([-0.06, -0.08, 0.06])), RIGHT_OPEN_NEAR, "away", g_out))
    t_right_rest = 0.8 + 0.6 + 1.2 + 1.5
    left.append(Waypoint(t_ins + t_right_rest, tuple(_left_tcp(np.zeros(3))), LEFT_SHUT, "keep the body on the ear", LEFT_GRIP))
    left += [
        Waypoint(0.6, tuple(_left_tcp(np.zeros(3))), LEFT_OPEN, "let go of the body", LEFT_GRIP),
        Waypoint(1.2, tuple(_left_tcp(np.zeros(3)) - 0.045 * LEFT_GRIP.rot[:, 2]), LEFT_OPEN, "back off up the fingers", LEFT_GRIP),
        Waypoint(1.5, tuple(_left_tcp(np.zeros(3)) + np.array([-0.06, 0.08, 0.08])), LEFT_OPEN, "away", LEFT_GRIP),
        Waypoint(4.0, tuple(_left_tcp(np.zeros(3)) + np.array([-0.06, 0.08, 0.08])), LEFT_OPEN, "wait", LEFT_GRIP),
    ]
    right.append(Waypoint(sum(w.duration for w in left) - sum(w.duration for w in right) + 0.1, tuple(out + np.array([-0.06, -0.08, 0.06])),
                          RIGHT_OPEN_NEAR, "wait", g_out))
    return {"left": left, "right": right}
