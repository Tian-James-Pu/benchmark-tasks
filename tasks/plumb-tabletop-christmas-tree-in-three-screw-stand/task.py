"""Straighten a 3 ft tabletop Christmas tree that leans in its three-eye-bolt stand, leaving it plumb and gripped by all three bolts.

The stand is a small live-tree stand with three 5/16"-18 eye bolts in a raised collar at 120 degrees (modelled on the Jack-Post Oasis
313-ST: 13 in across, 7 in tall, 5 in opening). The tree's butt sits on the stand's centre spike and its trunk is caged by the three bolts'
steel end caps. Someone set the tree up with the bolts uneven: the bolt on the tree's left (A) is backed out, the front-right one (B) is run
in too far, and the back one (C) is right. All three caps sit on the bark, not tightened, and the tree leans 2.5 degrees to the left.
Turning A in alone jams (B and C hold the trunk); B has to come out as A goes in, about 3.5 turns each, until the trunk is within 1 degree of
plumb, and then the bolts are snugged so that every cap grips: the scorer pushes the trunk 5 N towards each bolt in turn and the treetop
must move at most 3 mm.

Model (declared in code):
- Stand ("stand", fixed): green polypropylene bowl and bolt collar (custom meshes, segment colliders that leave the trunk's well open).
- Tree ("tree", an articulation fixed at the spike's tip): two hinges through the spike's tip (a gimbal: the trunk pivots on the spike;
  lifting the tree off the spike is not modelled). The trunk collides as a 45 mm cylinder up to the lowest branches; the foliage is seen
  only (real branches flex aside). The whole tree is 1.5 kg (an estimate) with gravity.
- Bolts ("bolt_a", "bolt_b", "bolt_c", one articulation each): a slider along the bolt (the shank and end cap, colliding at the cap) and a
  hinge about the bolt for the eye (colliding as a disc, which a gripper pinches). The thread (code, each control step): the slider is
  driven to the position the eye's angle screws it to (1.411 mm a turn, right hand), stiffly, never with more than F_MAX; the thread does
  not back-drive, so pushing the tree does not unscrew a bolt. When the cap is blocked (it cannot follow the eye by more than LAG), the
  eye is held at the angle the cap allows by a stiff drive: it will not turn further, as a snug bolt won't.

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
INSTRUCTION = "Straighten the little Christmas tree in its stand so that it stands upright and firm."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]

# -- the stand (Jack-Post Oasis 313-ST: 13 in across, 7 in tall, 5 in opening, three eye bolts) -------------------------------------------
SC = np.array([0.34, 0.0])  # the stand's centre on the table
BASE_R = 0.1651  # 13 in across
STAND_H = 0.1778  # 7 in tall
OPEN_R = 0.0635  # the collar's 5 in opening
COLLAR_R = 0.078
DECK_Z = 0.100  # the reservoir's top
Z_BOLT = 0.150  # the bolts' axis above the table
Z_PIVOT = 0.020  # the spike's tip (the trunk's pivot) above the table
H_BOLT = Z_BOLT - Z_PIVOT
WELL_R = 0.050  # the reservoir's well round the trunk (no colliders inside it)

# -- the tree: a 3 ft (91 cm) fresh-cut tabletop fir --------------------------------------------------------------------------------------
TRUNK_R = 0.0225  # 45 mm (1.77 in) trunk
TREE_H = 0.91
TRUNK_COL_H = 0.30  # the bare trunk below the lowest branches, from the butt
TREE_MASS = 1.5  # kg, estimate (README, "Scale and mass")
COM_Z = 0.40
LEAN0 = 2.5  # degrees: the starting lean, to the tree's left (+y)
START_QUAT = (-math.sin(math.radians(LEAN0) / 2), 0.0, 0.0, math.cos(math.radians(LEAN0) / 2))  # about world x: the trunk leans to +y

# -- the eye bolts: 5/16"-18 x 3 in, steel end caps ---------------------------------------------------------------------------------------
PITCH = 0.0254 / 18  # m a turn
BOLTS = {"a": 120.0, "b": 240.0, "c": 0.0}  # where each bolt is round the stand, degrees from +x (A on the tree's left, B front right, C back)
CAP_R, CAP_L = 0.0075, 0.009
SHANK_R = 0.0040
BOLT_LEN = 0.080  # from the cap's face to the eye's base
EYE_RM, EYE_WIRE = 0.010, 0.003  # the eye's mean radius and its wire's radius: 26 mm across outside, 14 mm inside
GAP0 = 0.0002  # authored gap between each cap and the bark (nothing authored in contact)
SLIDE_LO, SLIDE_HI = -0.020, 0.020  # travel from the starting position (m, inward positive)
K_S = 3.0e5  # N/m: the thread's (and the bark's) stiffness along the bolt
F_MAX = 300.0  # N: the most a cap pushes (or holds against a push)
LAG = 0.0003  # m: a cap this far short of where the eye has screwed it (pressing 90 N) is blocked
K_JAM = 2.0  # N m/rad: a blocked bolt's eye resists turning further this stiffly
EYE_EFFORT = 4.0  # N m


def unit(alpha_deg: float) -> np.ndarray:
    a = math.radians(alpha_deg)
    return np.array([math.cos(a), math.sin(a), 0.0])


def _start_offset() -> np.ndarray:
    """The trunk's axis at the bolts' height, from the stand's axis, in the starting layout (m, world xy)."""
    return np.array([0.0, H_BOLT * math.tan(math.radians(LEAN0)), 0.0])


RHO0 = {b: TRUNK_R + float(_start_offset() @ unit(a)) + GAP0 for b, a in BOLTS.items()}  # each cap's face from the stand's axis at the start

LOOKS = {
    "poly": m.Look(color=m.rgb(0.09, 0.36, 0.16), roughness=0.55),
    "poly_dark": m.Look(color=m.rgb(0.06, 0.26, 0.11), roughness=0.6),
    "steel": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.35),
    "zinc": m.Look(color=m.rgb(0.78, 0.78, 0.74), metallic=0.85, roughness=0.3),
    "bark": m.Look(color=m.rgb(0.30, 0.20, 0.12), roughness=0.9),
    "cut": m.Look(color=m.rgb(0.80, 0.68, 0.48), roughness=0.9),
    "fir": m.Look(color=m.rgb(0.07, 0.24, 0.12), roughness=0.85),
    "fir_light": m.Look(color=m.rgb(0.10, 0.30, 0.15), roughness=0.85),
}


# -- the stand ---------------------------------------------------------------------------------------------------------------------------------

BOWL_PROFILE = [(BASE_R, 0.0), (BASE_R, 0.008), (0.158, 0.020), (0.128, 0.085), (0.120, DECK_Z)]


def _hull(points) -> m.Mesh:
    """The convex hull's boundary triangles of a few points (brute force, numpy only: importing scipy before the simulator starts can crash
    its startup). The collider is cooked as the convex hull of the points anyway; the triangles only give the mesh a closed surface."""
    p = np.unique(np.round(np.asarray(points, float), 9), axis=0)
    c = p.mean(axis=0)
    faces = []
    n = len(p)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nrm = np.cross(p[j] - p[i], p[k] - p[i])
                if np.linalg.norm(nrm) < 1e-12:
                    continue
                d = (p - p[i]) @ nrm
                if d.max() <= 1e-12:
                    faces.append((i, j, k) if (c - p[i]) @ nrm < 0 else (i, k, j))
                elif d.min() >= -1e-12:
                    faces.append((i, k, j) if (c - p[i]) @ nrm > 0 else (i, j, k))
    return m.Mesh(p, np.array(faces, int))


def _ring_segments(prefix: str, r_in: float, profile, n: int = 12, skip=()) -> list[str]:
    """Convex segments of a ring: inner radius r_in, outer surface `profile` ((r, z) pairs), as hulls."""
    cols = []
    z0, z1 = profile[0][1], profile[-1][1]
    for k in range(n):
        t0, t1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = []
        for t in (t0, t1):
            c, s = math.cos(t), math.sin(t)
            pts += [(r * c, r * s, z) for r, z in profile]
            pts += [(r_in * c, r_in * s, z0), (r_in * c, r_in * s, z1)]
        cols.append(m.col_hull(f"{prefix}{k}", _hull(pts)))
    return cols


def _stand():
    parts = [("bowl", m.lathe([(0.0, 0.0)] + BOWL_PROFILE + [(0.0, DECK_Z)], 64), "poly"),
             ("deck_ring", m.lathe([(WELL_R + 0.004, DECK_Z), (0.118, DECK_Z), (0.118, DECK_Z + 0.002), (WELL_R + 0.004, DECK_Z + 0.002)], 64), "poly_dark"),
             ("collar", m.lathe([(OPEN_R, DECK_Z), (COLLAR_R, DECK_Z), (COLLAR_R, STAND_H), (OPEN_R, STAND_H), (OPEN_R, DECK_Z)], 64), "poly"),
             ("collar_lip", m.lathe([(OPEN_R - 0.002, STAND_H - 0.004), (COLLAR_R + 0.003, STAND_H - 0.004), (COLLAR_R + 0.003, STAND_H), (OPEN_R - 0.002, STAND_H),
                                     (OPEN_R - 0.002, STAND_H - 0.004)], 64), "poly_dark"),
             ("spike", m.lathe([(0.0, DECK_Z - 0.09), (0.007, DECK_Z - 0.09), (0.0, Z_PIVOT)], 16), "steel")]
    for b, a in BOLTS.items():
        boss = m.cylinder(0.012, 0.022, (0.0, 0.0, 0.0), axis="x").moved((COLLAR_R + 0.005, 0.0, Z_BOLT)).turned(a, "z")
        nut = m.prism([(0.0075 * math.cos(math.radians(60 * k + 30)), 0.0075 * math.sin(math.radians(60 * k + 30))) for k in range(6)], 0.0, 0.007)
        nut = nut.turned(90, "y").moved((COLLAR_R + 0.016 + 0.007, 0.0, Z_BOLT)).turned(a, "z")
        parts += [(f"boss_{b}", boss, "poly"), (f"nut_{b}", nut, "zinc")]
    cols = _ring_segments("bowl_c", WELL_R, BOWL_PROFILE) + _ring_segments("collar_c", OPEN_R, [(COLLAR_R, DECK_Z), (COLLAR_R, STAND_H)])
    cols.append(m.col_cylinder("floor_c", WELL_R + 0.002, 0.008, (0.0, 0.0, 0.004)))
    return parts, cols


_sp, _scol = _stand()
STAND = m.fixed("stand", (SC[0], SC[1], TOP), _sp, _scol, LOOKS, friction=(1.0, 0.8))


# -- the tree --------------------------------------------------------------------------------------------------------------------------------


def _tier(zb: float, rb: float, h: float, n: int, turn_deg: float) -> m.Mesh:
    """One whorl of branches: a cone whose rim is n drooping branch tips with notches between them."""
    v = [(0.0, 0.0, zb + h), (0.0, 0.0, zb + 0.035)]
    for i in range(2 * n):
        a = math.radians(turn_deg) + math.pi * i / n
        r, z = (rb, zb - 0.012) if i % 2 == 0 else (0.66 * rb, zb + 0.022)
        v.append((r * math.cos(a), r * math.sin(a), z))
    f = []
    for i in range(2 * n):
        j = (i + 1) % (2 * n)
        f += [(2 + i, 2 + j, 0), (2 + j, 2 + i, 1)]
    return m.Mesh(np.array(v, float), np.array(f))


def _tree_parts():
    trunk = m.lathe([(0.0, 0.0), (TRUNK_R, 0.0), (TRUNK_R, TRUNK_COL_H), (0.017, 0.45), (0.010, 0.70), (0.004, TREE_H - 0.02), (0.0, TREE_H)], 24)
    parts = [("trunk", trunk, "bark"), ("butt", m.cylinder(TRUNK_R - 0.0005, 0.001, (0.0, 0.0, 0.0006), segments=24), "cut")]
    tiers = 10
    for k in range(tiers):
        f = k / (tiers - 1)
        zb = TRUNK_COL_H + 0.015 + f * 0.50
        rb = 0.200 * (1 - f) + 0.050 * f
        hgt = 0.16 - 0.05 * f
        parts.append((f"tier{k}", _tier(zb, rb, hgt, 13 - k // 2, 137.5 * k), "fir" if k % 2 == 0 else "fir_light"))
    parts.append(("leader", m.lathe([(0.0, TREE_H - 0.12), (0.016, TREE_H - 0.10), (0.0, TREE_H)], 12), "fir_light"))
    return parts


def _tree():
    pitch = m.hinge("pitch", "X", (0.0, 0.0, 0.0), -12.0, 12.0, (), (), mass=0.1, inertia=(1e-3, 1e-3, 1e-3), damping=0.2, friction=0.02)
    cols = [m.col_cylinder("trunk_c", TRUNK_R, TRUNK_COL_H, (0.0, 0.0, TRUNK_COL_H / 2))]
    tree = m.hinge("tree", "Y", (0.0, 0.0, 0.0), -12.0, 12.0, _tree_parts(), cols, mass=TREE_MASS, gravity=True, com=(0.0, 0.0, COM_Z),
                   inertia=(0.030, 0.030, 0.020), damping=0.2, friction=0.02, parent="pitch")
    return m.mechanism("tree", (SC[0], SC[1], TOP + Z_PIVOT), [pitch, tree], LOOKS, start_quat=START_QUAT, friction=(0.6, 0.5))


TREE = _tree()


# -- the eye bolts: one articulation each, its frame's +X along the bolt towards the trunk, origin on the stand's axis -------------------------


def _x_cyl(r, x0, x1, segments=20):
    return m.cylinder(r, x1 - x0, ((x0 + x1) / 2, 0.0, 0.0), axis="x", segments=segments)


def _bolt(b: str, alpha: float):
    rho = RHO0[b]
    x_face, x_base = -rho, -rho - BOLT_LEN
    x_eye = x_base - EYE_RM
    cap = m.join(_x_cyl(CAP_R, x_face - CAP_L, x_face - 0.0015), _x_cyl(CAP_R - 0.0015, x_face - 0.0015, x_face))
    slide_parts = [("cap", cap, "steel"), ("shank", _x_cyl(SHANK_R, x_base + 0.003, x_face - CAP_L + 0.002, 12), "zinc")]
    slide_cols = [m.col_hull(f"cap_{b}_c", _x_cyl(CAP_R, x_face - CAP_L, x_face, 16))]
    slide = m.slider("slide", "X", SLIDE_LO, SLIDE_HI, slide_parts, slide_cols, at=(x_face, 0.0, 0.0), mass=0.04, spring=K_S, damping=200.0,
                     friction=0.0, effort=F_MAX, armature=0.02)
    ring = [(x_eye + EYE_RM * math.cos(t), 0.0, EYE_RM * math.sin(t)) for t in np.linspace(0.0, 2 * math.pi, 33)]
    eye_parts = [("eye", m.tube(ring, EYE_WIRE, sides=10), "zinc"), ("neck", _x_cyl(SHANK_R + 0.0005, x_base, x_base + 0.004, 12), "zinc")]
    eye_cols = [m.col_hull(f"eye_{b}_c", m.cylinder(EYE_RM + EYE_WIRE, 2 * EYE_WIRE, (x_eye, 0.0, 0.0), axis="y", segments=20))]
    eye = m.hinge("eye", "X", (x_face, 0.0, 0.0), -math.inf, math.inf, eye_parts, eye_cols, mass=0.02, friction=0.01, damping=0.004, effort=EYE_EFFORT,
                  armature=0.0005, inertia=(1.5e-6, 2.5e-6, 2.5e-6), parent="slide")
    return m.mechanism(f"bolt_{b}", (SC[0], SC[1], TOP + Z_BOLT), [slide, eye], LOOKS, yaw_deg=alpha + 180.0, friction=(0.5, 0.4))


BOLT_PARTS = {b: _bolt(b, a) for b, a in BOLTS.items()}

# -- distractors (SimReady, loose): a pillar candle (unlit), a mug and a paperback, on the same table ------------------------------------------
CANDLE = SimReady("candle", pos=(0.50, 0.33, TOP), size=(0.074, 0.074, 0.075), mass=0.35, package="decor_candle_a01", usd="sm_decor_candle_a01_01.usd")
MUG = SimReady("mug", pos=(0.48, -0.36, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.4)
BOOK = SimReady("book", pos=(0.66, -0.24, TOP), size=(0.229, 0.170, 0.010), mass=0.25, package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd", yaw=1.75)

OBJECTS = [STAND, TREE, *BOLT_PARTS.values(), CANDLE, MUG, BOOK]


# -- the threads -------------------------------------------------------------------------------------------------------------------------------


def _joint_vel(env, part, link) -> float:
    asset = env.unwrapped.scene[part]
    return float(m._np(asset.data.joint_vel)[0, asset.find_joints(f"{link}_joint")[0][0]])


def _drive(env, part, link, target, stiffness):
    asset, j = m._joint(env, part, link)
    m._TOUCHED.add((part, link))
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


class Threads:
    """Each bolt's thread. `screwed[b]` is how far (m, inward) the eye's turning has screwed the bolt: it follows the eye's angle at 1.411 mm a
    turn while the cap can follow it to within LAG; when the cap is blocked (by the bark going in, or by its stop) the screw position stays where
    the cap allows and the eye is held there by a stiff drive. A push on the cap never changes `screwed` (no back-driving)."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.screwed = {b: 0.0 for b in BOLTS}
        self.jam = {b: False for b in BOLTS}
        self.raw = {b: None for b in BOLTS}  # the eye joint as PhysX reports it (it wraps round), and
        self.angle = {b: 0.0 for b in BOLTS}  # the eye's angle counted through every turn (rad)

    def step(self, env):
        for b in BOLTS:
            part = f"bolt_{b}"
            s = m.state(env, part)
            raw, x = s.joint("eye"), s.joint("slide")
            if self.raw[b] is None:
                self.angle[b] = raw
            else:
                self.angle[b] += (raw - self.raw[b] + math.pi) % (2 * math.pi) - math.pi  # far less than half a turn per step
            self.raw[b] = raw
            q = self.angle[b]
            want = PITCH * q / (2 * math.pi)
            lo, hi = min(x - LAG, self.screwed[b]), max(x + LAG, self.screwed[b])
            got = min(max(want, lo), hi)
            self.screwed[b] = got
            self.jam[b] = abs(want - got) > 1e-9
            _drive(env, part, "eye", raw + (got * 2 * math.pi / PITCH - q), K_JAM if self.jam[b] else 0.0)
            m.set_target(env, part, "slide", min(max(got, SLIDE_LO), SLIDE_HI))


THREADS = Threads()
m.on_step(THREADS.step)
m.on_reset(THREADS.reset)


# -- reading the scene -------------------------------------------------------------------------------------------------------------------------


def trunk_axis(tree_state) -> np.ndarray:
    """The trunk's axis (unit, world) from the frame's pose and the gimbal's two joints."""
    qp, qt = tree_state.joint("pitch"), tree_state.joint("tree")
    cp, sp, ct, st = math.cos(qp), math.sin(qp), math.cos(qt), math.sin(qt)
    rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    ry = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
    return tree_state.rotation() @ rx @ ry @ np.array([0.0, 0.0, 1.0])


def tilt_deg(tree_state) -> float:
    return float(math.degrees(math.acos(min(1.0, max(-1.0, trunk_axis(tree_state)[2])))))


def lean_y_deg(tree_state) -> float:
    """The lean towards +y (the tree's left as the robot faces it), signed, degrees."""
    return float(math.degrees(math.asin(min(1.0, max(-1.0, trunk_axis(tree_state)[1])))))


def offset_at_bolts(tree_state) -> np.ndarray:
    """The trunk's axis at the bolts' height, from the stand's axis (world xy, m)."""
    d = trunk_axis(tree_state)
    return np.array([d[0], d[1], 0.0]) * (H_BOLT / d[2])


def cap_gaps(states: dict) -> dict:
    """Each cap's gap to the bark (m; negative: pressed in), across the bolt's axis at its height."""
    d = trunk_axis(states["tree"])
    off = offset_at_bolts(states["tree"])
    out = {}
    for b, a in BOLTS.items():
        u = unit(a)
        rho = RHO0[b] - states[f"bolt_{b}"].joint("slide")
        surface = float(off @ u) + TRUNK_R * math.sqrt(max(0.0, 1.0 - float(u @ d) ** 2)) + CAP_R * abs(float(u @ d)) / d[2]  # the cap's nearest edge
        out[b] = rho - surface
    return out


def eye_centre(b: str, slide: float) -> np.ndarray:
    r = RHO0[b] - slide + BOLT_LEN + EYE_RM
    return np.array([SC[0], SC[1], TOP + Z_BOLT]) + r * unit(BOLTS[b])


def hands_gap(states: dict, tips: np.ndarray) -> float:
    """The nearest fingertip point's distance (m) from the trunk (below the branches) or from any eye bolt's eye."""
    d = trunk_axis(states["tree"])
    p0 = np.array([SC[0], SC[1], TOP + Z_PIVOT])
    t = np.clip((tips - p0) @ d, 0.0, TRUNK_COL_H)
    trunk = np.linalg.norm(tips - (p0 + np.outer(t, d)), axis=1) - TRUNK_R
    gaps = [float(trunk.min())]
    for b in BOLTS:
        e = eye_centre(b, states[f"bolt_{b}"].joint("slide"))
        gaps.append(float(np.min(np.linalg.norm(tips - e, axis=1))) - (EYE_RM + EYE_WIRE))
    return min(gaps)


PLUMB_DEG, NEAR_DEG, HARM_DEG = 1.0, 2.0, 6.0
BEAR_N = 3.0  # N: a cap "bears on the bark" when the bark pushes it back along its thread at least this hard (real trial: a 0.05 mm feeler blade will not enter)
CLEAR = 0.03
STILL_RAD_S = 0.02

# The firmness probe, run by the scorer once the tree has stood plumb, bearing on all three caps, still and untouched for 2 s: a 5 N horizontal push on the
# bare trunk 25 cm above the butt, towards each bolt in turn (so each cap alone takes it), ramped on over 0.25 s and held 1 s, then released for
# 0.75 s. Read: the treetop's sideways movement under the load and after release. Real trial: a push gauge (or a luggage scale pulling) at the mark,
# and a ruler clamped level beside the treetop, read from a phone video.
PROBE_F = 5.0  # N
PROBE_H = 0.25  # m above the butt, on the bare trunk (the lowest branches start at 30 cm)
PROBE_ORDER = ("a", "b", "c")  # pushed towards bolt A, then B, then C
PROBE_RAMP, PROBE_LOAD, PROBE_REST = 5, 20, 15  # control steps: 0.25 s ramp, 1 s at full load, 0.75 s released
DEFLECT_MAX = 0.003  # m: the treetop moves at most 3 mm under the load
RETURN_MAX = 0.001  # m: and comes back to within 1 mm of where it stood once released


def bearing(env, states: dict) -> dict:
    """How hard (N) the bark pushes each cap back along its bolt: the thread's spring, from where the thread holds the cap and where it is."""
    out = {}
    for b in BOLTS:
        target = min(max(THREADS.screwed[b], SLIDE_LO), SLIDE_HI)
        out[b] = K_S * (target - states[f"bolt_{b}"].joint("slide"))
    return out


def treetop(tree_state) -> np.ndarray:
    """The treetop (station frame, m)."""
    return np.array([SC[0], SC[1], TOP + Z_PIVOT]) + TREE_H * trunk_axis(tree_state)


def push(env, force_xyz=None, tree_state=None) -> None:
    """Apply (or, with None, remove) the probe's horizontal force on the trunk PROBE_H above the butt."""
    import torch

    asset = env.unwrapped.scene["tree"]
    composer = asset.permanent_wrench_composer
    if force_xyz is None:
        composer.reset()
        return
    body = asset.find_bodies("tree")[0][0]
    origin = m._np(env.unwrapped.scene.env_origins[0])
    p = origin + np.array([SC[0], SC[1], TOP + Z_PIVOT]) + PROBE_H * trunk_axis(tree_state)
    f = torch.tensor(np.asarray(force_xyz, float).reshape(1, 1, 3), dtype=torch.float32, device=asset.device)
    pos = torch.tensor(p.reshape(1, 1, 3), dtype=torch.float32, device=asset.device)
    composer.set_forces_and_torques_index(forces=f, positions=pos, body_ids=[body], env_ids=torch.tensor([0], device=asset.device), is_global=True)


def now(env) -> dict:
    names = ["tree", *(f"bolt_{b}" for b in BOLTS)]
    states = {n: m.state(env, n) for n in names}
    tips = m.fingertips(env)
    gaps = cap_gaps(states)
    bear = bearing(env, states)
    still = abs(_joint_vel(env, "tree", "pitch")) < STILL_RAD_S and abs(_joint_vel(env, "tree", "tree")) < STILL_RAD_S
    tilt = tilt_deg(states["tree"])
    hands = hands_gap(states, tips)
    return {"tilt": tilt, "lean_y": lean_y_deg(states["tree"]), "gaps": gaps, "bear": bear, "firm": all(f >= BEAR_N for f in bear.values()),
            "plumb": tilt <= PLUMB_DEG, "near": tilt <= NEAR_DEG, "hands": hands, "clear": hands >= CLEAR, "top": treetop(states["tree"]),
            "tree_state": states["tree"], "still": still, "slides": {b: states[f"bolt_{b}"].joint("slide") for b in BOLTS},
            "eyes": {b: states[f"bolt_{b}"].deg("eye") for b in BOLTS}}


class Trial:
    """The score, the firmness probe, and the record of a tree let flop: the trunk ever more than HARM_DEG from plumb (cleared only at a reset).

    Full credit: plumb, all three caps bearing, both grippers clear and the tree still for HOLD_STEPS; then, with the grippers still clear, the probe
    pushes the trunk towards each bolt in turn and the treetop must move at most DEFLECT_MAX under the load and come back to within RETURN_MAX, with
    the tree plumb and every cap bearing again after each release. A failed or interrupted probe starts the hold over."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.flopped = False
        self.last = {}
        self.probe = None  # None, or [index into PROBE_ORDER, step within that push, the treetop before it, the largest movement under load]
        self.probe_log = []  # one line per push: bolt, deflection under load (m), movement after release (m), passed
        self.probes_run = 0
        if env is not None:
            try:
                push(env, None)
            except Exception:
                pass

    def _abort(self, env, why: str):
        push(env, None)
        if self.probe is not None:
            self.probe_log.append((PROBE_ORDER[self.probe[0]], float("nan"), float("nan"), False, why))
        self.probe = None
        self.hold = 0

    def step(self, env):
        c = now(env)
        self.last = c
        if c["tilt"] > HARM_DEG:
            self.flopped = True
        if self.full or self.flopped:
            if self.probe is not None:
                self._abort(env, "ended")
        elif self.probe is not None:
            self._probe_step(env, c)
        else:
            ok = c["plumb"] and c["firm"] and c["clear"] and c["still"]
            self.hold = self.hold + 1 if ok else 0
            if self.hold >= HOLD_STEPS:
                self.probe = [0, 0, c["top"].copy(), 0.0]
                self.probes_run += 1
        if self.flopped:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["plumb"] and c["clear"]:
            self.score = 0.5
        elif c["near"] and c["firm"] and c["clear"]:
            self.score = 0.25
        else:
            self.score = 0.0

    def _probe_step(self, env, c):
        if not c["clear"]:
            self._abort(env, "a gripper came within 3 cm")
            return
        k, s, ref, worst = self.probe
        b = PROBE_ORDER[k]
        moved = float(np.linalg.norm((c["top"] - ref)[:2]))
        if s < PROBE_RAMP + PROBE_LOAD:
            if s >= PROBE_RAMP:
                worst = max(worst, moved)
            scale = min(1.0, (s + 1) / PROBE_RAMP)
            push(env, PROBE_F * scale * unit(BOLTS[b]), c["tree_state"])
        else:
            push(env, None)
        s += 1
        self.probe = [k, s, ref, worst]
        if s < PROBE_RAMP + PROBE_LOAD + PROBE_REST:
            return
        ok = worst <= DEFLECT_MAX and moved <= RETURN_MAX and c["plumb"] and c["firm"]
        self.probe_log.append((b, worst, moved, ok, ""))
        if not ok:
            self.probe = None
            self.hold = 0
            return
        if k + 1 < len(PROBE_ORDER):
            self.probe = [k + 1, 0, c["top"].copy(), 0.0]
        else:
            self.probe = None
            self.full = True


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration -----------------------------------------------------------------------------------------------------------------------


def _report(ctx, env, label=""):
    c = now(env)
    g = c["gaps"]
    print(f"[tree] t {ctx.t:5.1f} {label}tilt {c['tilt']:.2f} lean_y {c['lean_y']:+.2f} deg, gaps a {g['a'] * 1000:+.2f} b {g['b'] * 1000:+.2f} c {g['c'] * 1000:+.2f} mm, "
          f"slides a {c['slides']['a'] * 1000:+.2f} b {c['slides']['b'] * 1000:+.2f} c {c['slides']['c'] * 1000:+.2f} mm, eyes a {math.degrees(THREADS.angle['a']):+.0f} b {math.degrees(THREADS.angle['b']):+.0f}, "
          f"bearing a {c['bear']['a']:.1f} b {c['bear']['b']:.1f} c {c['bear']['c']:.1f} N, "
          f"jam {THREADS.jam}, hands {c['hands'] * 1000:.0f} mm, still {c['still']}, score {TRIAL.score:.2f} full {TRIAL.full} flopped {TRIAL.flopped}", flush=True)
    log = ", ".join(f"{b}: {w * 1000:.3f} mm loaded / {r * 1000:.3f} mm released {'ok' if ok else 'FAIL'}{(' ' + why) if why else ''}" for b, w, r, ok, why in TRIAL.probe_log[-6:])
    print(f"[tree] probes started {TRIAL.probes_run}; last pushes: {log or 'none'}", flush=True)


def demo_script(ctx):
    """Left arm on bolt A (the tree's left), right arm on bolt B (front right), each gripper level and pointing along its bolt at the trunk, so
    that rolling the wrist turns the bolt. Each pinches its bolt's eye across the wire at the start of the wrist's roll (the eye's angle
    allows a pinch every half turn), turns it as far as the wrist allows, lets go, rolls back and pinches again. B is backed out (anticlockwise
    as seen from outside) a little ahead of A being screwed in (clockwise), so the trunk is never squeezed; both stop when the tree is seen
    upright. Then each screws its bolt in until it bites (the eye stops following the wrist), so that all three caps grip the trunk, and both
    let go and move clear.
    Failure checks (--mode): none does nothing; onearm screws A in without touching B; careless backs B out without touching A; wrongway
    backs A out (the tree's lean side) instead; hold does the full job but keeps the left gripper on A's eye; push pushes the tree's trunk
    upright with the left hand and lets go; short stops both bolts when the tree is seen at 1.5 degrees; loose does the full job, then backs
    A out two more pinches; nosnug straightens the tree but never snugs the bolts (the previous demonstration's end); gap1 and gap05 do the
    full job, and then the test itself (not the arms) backs every cap's thread off to leave it 1 mm or 0.5 mm from the bark."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import axis_angle_matrix

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "wrongway", "hold", "push", "short", "loose", "nosnug", "gap1", "gap05"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(80):
            yield
        _report(ctx, env, "end: ")
        return

    ROLL = 110.0  # the wrist's roll stays within +-ROLL degrees (joint 6 is limited to 120)

    goal_deg = 1.5 if mode == "short" else 0.0

    def lean_mm():
        """How far A must still go in (m) for the trunk to lean only `goal_deg`, from the lean seen."""
        lean = ctx.state("tree")
        return 0.866 * H_BOLT * (math.tan(math.radians(lean_y_deg(lean))) - math.tan(math.radians(goal_deg)))

    class Turner:
        """One arm on one bolt: `sign` +1 screws in (clockwise from outside), -1 backs out."""

        def __init__(self, arm, b, sign):
            self.arm, self.b, self.sign = arm, b, sign
            self.u = unit(BOLTS[b])
            self.phi = 0.0  # the wrist's roll about the bolt, from the level grip
            self.level = d.level(BOLTS[b] + 180.0)
            self.axis = -self.u  # the approach: along the bolt towards the trunk

        def grip(self, phi):
            return d._as_grip(axis_angle_matrix(self.axis, math.radians(phi)) @ self.level.rot)

        def tcp(self, back=0.0):
            e = eye_centre(self.b, ctx.joint(f"bolt_{self.b}", "slide"))
            return e + (0.004 + back) * self.u

        def grasp_phi(self):
            q = ctx.deg(f"bolt_{self.b}", "eye")
            if self.sign > 0:  # start low, turn up to +ROLL; pinch across the wire
                return -ROLL + ((q + ROLL) % 180.0)
            return ROLL - ((ROLL - q) % 180.0)


        def approach(self):
            yield from self.arm.move(self.tcp(0.06), grip=self.grip(0.0), finger=FINGER_OPEN, t=2.5, label=f"before bolt {self.b}")
            yield from self.arm.move(self.tcp(0.035), t=0.6, label=f"near bolt {self.b}")
            self.phi = 0.0

        def cycle(self, limit_deg=None):
            """One pinch-turn-release. Returns the degrees turned."""
            phi = self.grasp_phi()
            if abs(phi - self.phi) > 0.5:
                yield from self.arm.turn(phi - self.phi, axis=self.axis, t=max(0.5, abs(phi - self.phi) / 150.0), label=f"roll back at bolt {self.b}")
            self.phi = phi
            yield from self.arm.move(self.tcp(0.0), t=0.5, label=f"onto eye {self.b}")
            yield from self.arm.close(2 * EYE_WIRE, 20.0, t=0.4, label=f"pinch eye {self.b}")
            span = (ROLL - phi) if self.sign > 0 else (phi + ROLL)
            if limit_deg is not None:
                span = min(span, limit_deg)
            span = max(span, 0.0)
            deg = self.sign * span
            yield from self.arm.turn(deg, axis=self.axis, t=max(0.4, span / 90.0), label=f"turn bolt {self.b}")
            self.phi += deg
            yield from self.arm.open(t=0.3, label=f"let go of eye {self.b}")
            yield from self.arm.move(self.tcp(0.035), t=0.4, label=f"off eye {self.b}")
            return span

    left_a = Turner(l, "a", +1 if mode != "wrongway" else -1)
    right_b = Turner(r, "b", -1)
    use_left = mode in ("full", "onearm", "wrongway", "hold", "short", "loose", "nosnug", "gap1", "gap05")
    use_right = mode in ("full", "careless", "hold", "short", "loose", "nosnug", "gap1", "gap05")

    if mode == "push":  # the left hand pushes the trunk below the branches towards plumb, holds, lets go
        p = np.array([SC[0], SC[1] + 0.06, TOP + Z_PIVOT + 0.25])
        yield from l.move(p, grip=d.level(-90.0), finger=FINGER_OPEN * 0.2, t=2.5, label="beside the trunk")
        yield from l.move(p - np.array([0.0, 0.05, 0.0]), t=2.0, label="push the trunk")
        _report(ctx, env, "pushing: ")
        yield from l.wait(1.0)
        yield from l.move(p + np.array([0.0, 0.06, 0.05]), t=1.5, label="hand away")
        for _ in range(60):
            yield
        _report(ctx, env, "end: ")
        return

    gens = []
    if use_left:
        gens.append(left_a.approach())
    if use_right:
        gens.append(right_b.approach())
    yield from ctx.together(*gens)
    _report(ctx, env, "at the bolts: ")

    def run_bolt(t: Turner, lead_s: float, need=None, cycles: int = 14):
        """Pinch-turn cycles until `need()` (m still to go, from the lean seen) is used up, or for `cycles` cycles if no `need`."""
        for _ in range(round(lead_s * 20)):
            yield
        for k in range(cycles):
            left = None if need is None else need()
            if left is not None and left <= 0.0:
                return
            lim = None if left is None else math.degrees(2 * math.pi * left / PITCH) + 8.0
            yield from t.cycle(lim)

    def still_to_go(extra=0.0):
        return (lean_mm() + extra) if lean_y_deg(ctx.state("tree")) > goal_deg + 0.08 else 0.0

    def snug(t: Turner, tries: int = 4):
        """Screw the bolt in until it bites: pinch-turn cycles until the eye stops following the wrist (the cap is tight on the bark)."""
        t.sign = +1
        for _ in range(tries):
            before = THREADS.angle[t.b]  # the eye's own angle, counted through turns (what a camera on the eye sees)
            span = yield from t.cycle(200.0)
            turned = math.degrees(THREADS.angle[t.b] - before)
            print(f"[tree] snug {t.b}: wrist {span:.0f} deg, eye {turned:.0f} deg", flush=True)
            if span >= 30.0 and turned < 0.6 * span:
                return

    if mode in ("full", "hold", "short", "loose", "nosnug", "gap1", "gap05"):
        yield from ctx.together(run_bolt(right_b, 0.0, lambda: still_to_go(0.0003)), run_bolt(left_a, 1.0, still_to_go))
        _report(ctx, env, "plumbed: ")
        if mode != "nosnug":  # both bolts snugged onto the bark, so that all three caps grip
            yield from ctx.together(snug(right_b), snug(left_a))
            _report(ctx, env, "snugged: ")
        if mode == "loose":
            left_a.sign = -1
            yield from run_bolt(left_a, 0.0, cycles=2)
    elif mode == "onearm" or mode == "wrongway":
        yield from run_bolt(left_a, 0.0, cycles=10 if mode == "wrongway" else 6)
    elif mode == "careless":
        yield from run_bolt(right_b, 0.0, cycles=6)
    _report(ctx, env, "turned: ")

    # both away
    gens = []
    if use_left and mode != "hold":
        gens.append(l.move(left_a.tcp(0.08), t=1.0, label="left back"))
    if use_right:
        gens.append(r.move(right_b.tcp(0.08), t=1.0, label="right back"))
    yield from ctx.together(*gens)
    gens = []
    if use_left and mode != "hold":
        gens.append(l.move(np.array([0.20, 0.28, TOP + 0.22]), grip=d.down(0), finger=FINGER_OPEN, t=1.5, label="left clear"))
    if use_right:
        gens.append(r.move(np.array([0.20, -0.28, TOP + 0.22]), grip=d.down(0), finger=FINGER_OPEN, t=1.5, label="right clear"))
    if mode == "hold":
        def keep():
            yield from l.move(left_a.tcp(0.0), t=0.5, label="back onto eye a")
            yield from l.close(2 * EYE_WIRE, 20.0, t=0.4, label="keep the eye pinched")
        gens.append(keep())
    yield from ctx.together(*gens)
    if mode in ("gap1", "gap05"):  # a scorer check, not a robot action: the test itself backs every cap's thread off to leave it this far from the bark
        want = 0.001 if mode == "gap1" else 0.0005
        g = cap_gaps({n: ctx.state(n) for n in ["tree", *(f"bolt_{b}" for b in BOLTS)]})
        for b in BOLTS:
            back = want - g[b]
            THREADS.screwed[b] -= back
            THREADS.angle[b] -= back * 2 * math.pi / PITCH
        _report(ctx, env, "caps backed off by the test: ")
        for _ in range(20):
            yield
        _report(ctx, env, "settled: ")
    for k in range(300):
        yield
        if k % 30 == 29:
            _report(ctx, env, "end: ")

