"""The front end of a parked 50 cc scooter, its left side towards the robot, and a disc lock on the floor beside its front wheel.

The scooter faces +y (its nose to the robot's left). Its front wheel (10-inch alloy rim, 3.50-10 tyre, 434 mm across) turns on its
axle (along x) in the fork. On the wheel's near side (towards the robot) a 220 mm, 4 mm thick steel brake disc is bolted to the hub,
with nine 9 mm vent holes on a 194 mm circle (40 degrees apart). The left fork leg runs up and back from the axle, outboard of the
disc, and carries the brake caliper over the disc's rear upper part; a fender covers the top of the tyre. At the start, the two vent
holes nearest the front of the disc sit 20 degrees below and 20 degrees above its front point.

The disc lock (a 0.56 kg stapler-class disc lock with a 5.5 mm pin) lies on the floor in front of the wheel, its 7 mm jaw slot facing
the wheel. Its pin is the end of a sprung plunger: pressing the plunger's cap flush drives the pin 10 mm across the jaw gap, where a
catch holds it. The pin stops on the steel if it is not over a vent hole.

Model (declared in code):
- Scooter ("scooter"): a fixed frame (fork legs, axle, caliper, fender, crown and steering stem) with one link, the wheel (tyre, rim,
  hub, disc), on an unlimited hinge about the axle. The wheel turns against WHEEL_DRAG of joint friction (the bearings, the pads and
  the tyre's floor contact on a parked scooter), more than the lock's own weight can turn it with.
- Disc ("wheel" link): the vent holes are real holes in the disc's collider: the disc is built of convex pieces around each hole.
- Lock ("disc_lock"): a free 0.54 kg body (two jaws and the spine) with one link, the plunger (pin and cap, 20 g), on a slider along
  the pin, sprung back to its open position (PLUNGER_K). When the plunger reaches the end of its travel the catch takes it (the
  joint is locked); only a reset frees it, as only the key would.
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
INSTRUCTION = "Lock the scooter's front brake disc with the disc lock."
TOP = TABLE_TOP[SCENE]
DT = 0.05
SUCCESS_HOLD_S = 1.0  # the harness's own hold, after the scorer's 2 s
HOLD_STEPS = 40  # 2 s at 20 Hz

# -- the wheel (frame: origin at the wheel's centre, axes as the world's; x is the axle, -x towards the robot) ------------------
W = np.array([0.40, 0.0, TOP + 0.218])
R_TYRE = 0.217  # 3.50-10: 254 mm rim + 2 x 89 mm section height = 432-434 mm across
R_RIM = 0.127  # 10-inch bead seat
X_D = -0.072  # the disc's mid-plane, outboard of the tyre
T_D = 0.004  # disc thickness
R_D = 0.110  # 220 mm disc
R_HUB = 0.035
R_PITCH = 0.097  # vent-hole pitch radius
HOLE_R = 0.0045  # 9 mm vent holes
N_HOLES = 9
HOLE0_DEG = -20.0  # the first hole's angle at the start, from the front (+y) towards up (+z)
HOLE_DEGS = tuple(HOLE0_DEG + 40.0 * k for k in range(N_HOLES))
WHEEL_HOLD, WHEEL_DRAG = 1.0, 0.8  # N m: the wheel's static and sliding friction (Drag below)
WHEEL_K = 300.0  # N m/rad: the friction's stiffness before it slips (0.2 degrees at WHEEL_HOLD)
R_BAND = 0.088  # the inner edge of the band the hole pieces cover

# -- the fork leg and caliper on the disc side
X_LEG, R_LEG, RAKE, LEG_L = -0.100, 0.017, math.radians(25.0), 0.42
LEG_DIR = np.array([0.0, -math.sin(RAKE), math.cos(RAKE)])  # up and back from the axle
CAL_DEG = 140.0  # the caliper's centre, from the front towards up
CAL_R = 0.095
CAL_SIZE = (0.050, 0.055, 0.042)  # along x (outboard half, gap, inboard half), tangential, radial
CAL_X0, CAL_X1 = X_D - 0.026, X_D + 0.016

# -- the lock (frame: origin on the pin's axis at the jaw gap's mid-plane; x along the pin, into the jaw; y away from the jaw's
#    mouth; z up). It lies on the floor as it will sit on the disc's front: the slot facing the wheel (-y).
GAP = 0.007  # jaw gap: takes discs up to 6 mm
T_OUT, T_IN = 0.022, 0.009  # the plunger side and the far jaw
X_OUT, X_IN = -(GAP / 2 + T_OUT), GAP / 2 + T_IN
Y_TIP, Y_SLOT, Y_BACK = -0.014, 0.016, 0.065  # jaw tips, slot bottom, back
H = 0.032
PIN_R = 0.00275  # 5.5 mm pin
TRAVEL = 0.010
PIN_TIP0 = -(GAP / 2 + 0.0005)  # the pin's tip, open: just inside its jaw
CAP_R, CAP_OUT = 0.008, 0.010  # the cap stands 10 mm proud when open, flush when home
CATCH = 0.0095  # the catch takes the plunger this far in
PLUNGER_K = 400.0  # N/m: 4 N at full press
LOCK_MASS, PLUNGER_MASS = 0.54, 0.02
LOCK_START = np.array([0.27, 0.19, TOP + H / 2])

LOOKS = {
    "rubber": m.Look(color=m.rgb(0.04, 0.04, 0.045), roughness=0.85),
    "alloy": m.Look(color=m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.35),
    "disc": m.Look(color=m.rgb(0.58, 0.59, 0.60), metallic=0.9, roughness=0.3),
    "steel": m.Look(color=m.rgb(0.75, 0.75, 0.77), metallic=0.95, roughness=0.25),
    "fork": m.Look(color=m.rgb(0.10, 0.10, 0.11), metallic=0.4, roughness=0.4),
    "caliper": m.Look(color=m.rgb(0.12, 0.12, 0.13), metallic=0.5, roughness=0.45),
    "body": m.Look(color=m.rgb(0.75, 0.08, 0.07), metallic=0.3, roughness=0.3),
    "lock": m.Look(color=m.rgb(0.95, 0.78, 0.05), roughness=0.45),
    "chrome": m.Look(color=m.rgb(0.85, 0.85, 0.87), metallic=1.0, roughness=0.15),
    "dark": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.6),
}


def _ring_pts(r, a0, a1, n):
    a = np.radians(np.linspace(a0, a1, n))
    return np.c_[r * np.cos(a), r * np.sin(a)]


def _slab(poly_yz, x0, x1) -> m.Mesh:
    """A convex polygon in the wheel's (y, z) plane, from x0 to x1."""
    p = np.asarray(poly_yz, float)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]
    pr = m.prism(p, x0, x1)  # built in (x, y, z) = (y, z, x): turn it so its extrusion runs along x
    return m.Mesh(pr.v[:, [2, 0, 1]], pr.f)


def _hole_pieces(phi_deg, n_side=12, wt=0.0068):
    """Convex pieces round one vent hole: the band from R_BAND to R_D, +-wt along the disc, with a 12-sided hole whose inscribed
    circle is the 9 mm hole. In the hole's own (t, n) axes, then placed."""
    n0, n1 = R_BAND - R_PITCH, R_D - R_PITCH
    rc = HOLE_R / math.cos(math.pi / n_side)
    angs = [2 * math.pi * (i + 0.5) / n_side for i in range(n_side)]
    corners = [(wt, n0), (wt, n1), (-wt, n1), (-wt, n0)]
    corner_angs = [math.atan2(cn, ct) % (2 * math.pi) for ct, cn in corners]

    def ray(a):
        c, s = math.cos(a), math.sin(a)
        ss = []
        if abs(c) > 1e-9:
            ss.append((wt if c > 0 else -wt) / c)
        if abs(s) > 1e-9:
            ss.append((n1 if s > 0 else n0) / s)
        k = min(v for v in ss if v > 0)
        return (k * c, k * s)

    phi = math.radians(phi_deg)
    centre = R_PITCH * np.array([math.cos(phi), math.sin(phi)])
    e_n, e_t = np.array([math.cos(phi), math.sin(phi)]), np.array([-math.sin(phi), math.cos(phi)])
    pieces = []
    for i in range(n_side):
        a, b = angs[i] % (2 * math.pi), angs[(i + 1) % n_side] % (2 * math.pi)
        pts = [(rc * math.cos(a), rc * math.sin(a)), (rc * math.cos(b), rc * math.sin(b)), ray(a), ray(b)]
        span = (b - a) % (2 * math.pi)
        pts += [corners[j] for j, ca in enumerate(corner_angs) if 0 < (ca - a) % (2 * math.pi) < span]
        pieces.append([centre + t * e_t + n * e_n for t, n in pts])
    return pieces


def _disc_pieces():
    pieces = []
    for phi in HOLE_DEGS:
        pieces += _hole_pieces(phi)
    d = math.degrees(math.asin(0.0058 / R_D))
    for k, phi in enumerate(HOLE_DEGS):  # the band between holes, and the inner disc
        a0, a1 = phi + d, phi + 40.0 - d
        pieces.append(np.r_[_ring_pts(R_D, a0, a1, 10), _ring_pts(R_BAND - 0.002, a0, a1, 10)])
        pieces.append(np.r_[_ring_pts(0.090, phi, phi + 40.0, 10), _ring_pts(R_HUB, phi, phi + 40.0, 6)])
    return pieces


def _tyre_pieces(n=24):
    prof = ((0.130, 0.040), (0.175, 0.0445), (0.205, 0.040), (0.217, 0.030))  # (radius, half width)
    out = []
    for k in range(n):
        a0, a1 = 360.0 * k / n, 360.0 * (k + 1) / n
        pts = []
        for r, hw in prof:
            for a in np.radians(np.linspace(a0, a1, 3)):
                for x in (-hw, hw):
                    pts.append((x, r * math.cos(a), r * math.sin(a)))
        out.append(m.Mesh(np.array(pts), np.zeros((1, 3), int)))
    return out


def _x_lathe(profile, segments=48):
    return m.lathe(profile, segments).aimed("+X")


def _wheel_link():
    tyre = _x_lathe([(0.127, -0.038), (0.150, -0.044), (0.175, -0.0445), (0.200, -0.040), (0.212, -0.032), (0.217, -0.018),
                     (0.217, 0.018), (0.212, 0.032), (0.200, 0.040), (0.175, 0.0445), (0.150, 0.044), (0.127, 0.038), (0.127, -0.038)], 64)
    rim = _x_lathe([(0.118, -0.041), (0.129, -0.041), (0.129, -0.036), (0.122, -0.034), (0.122, 0.034), (0.129, 0.036), (0.129, 0.041),
                    (0.118, 0.041), (0.118, -0.041)], 64)
    hub = m.cylinder(R_HUB, 0.115, ((X_D + 0.045) / 2 - 0.0, 0, 0), axis="x")
    spokes = [m.box((0.022, 0.090, 0.020), (0.0, 0.0, 0.0)).moved((0, 0, 0.078)).turned(72.0 * k + 18.0, "x") for k in range(5)]
    disc_parts = [_slab(p, X_D - T_D / 2, X_D + T_D / 2) for p in _disc_pieces()]
    bolts = [m.cylinder(0.005, 0.006, (X_D - 0.004, 0.045 * math.cos(a), 0.045 * math.sin(a)), axis="x", segments=12) for a in np.radians([90, 210, 330])]
    parts = [("tyre", tyre, "rubber"), ("rim", rim, "alloy"), ("hub", hub, "alloy"), ("spokes", m.join(*spokes), "alloy"),
             ("disc", m.join(*disc_parts), "disc"), ("bolts", m.join(*bolts), "steel")]
    cols = [m.col_hull(f"disc_{i}", mesh) for i, mesh in enumerate(disc_parts)]
    cols += [m.col_hull(f"tyre_{i}", mesh) for i, mesh in enumerate(_tyre_pieces())]
    return m.hinge("wheel", "X", (0.0, 0.0, 0.0), float("-inf"), float("inf"), parts, cols, mass=5.5, spring=WHEEL_K, damping=0.3, friction=0.0,
                   effort=200.0, armature=0.01, inertia=(0.13, 0.075, 0.075))


def _caliper_mesh_parts():
    """The caliper as boxes in its own frame (x along the axle, t along the disc, n outward), turned to CAL_DEG."""
    a = math.radians(CAL_DEG)
    c = CAL_R * np.array([math.cos(a), math.sin(a)])
    e_n, e_t = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    t, nn = CAL_SIZE[1] / 2, CAL_SIZE[2] / 2

    def block(x0, x1, n0, n1, t0=-t, t1=t):
        poly = [c + tt * e_t + n * e_n for tt in (t0, t1) for n in (n0, n1)]
        return _slab(poly, x0, x1)

    outer = block(CAL_X0, X_D - T_D / 2 - 0.0015, -nn, nn)
    inner = block(X_D + T_D / 2 + 0.0015, CAL_X1, -nn, nn)
    bridge = block(CAL_X0, CAL_X1, R_D + 0.004 - CAL_R, nn + 0.006)
    return outer, inner, bridge


def _frame():
    parts, cols = [], []
    for name, x in (("leg_l", X_LEG), ("leg_r", -X_LEG)):
        p0 = np.array([x, 0.0, 0.0]) - 0.01 * LEG_DIR
        p1 = np.array([x, 0.0, 0.0]) + LEG_L * LEG_DIR
        mesh = m.tube([p0, (p0 + p1) / 2, p1], R_LEG, sides=16)
        parts.append((name, mesh, "fork"))
        cols.append(m.col_hull(name + "_c", m.tube([p0, p1], R_LEG, sides=12)))
        parts.append((name + "_foot", m.cylinder(0.024, 0.030, (x, 0, 0), axis="x"), "fork"))
        cols.append(m.col_hull(name + "_foot_c", m.cylinder(0.024, 0.030, (x, 0, 0), axis="x", segments=12)))
    parts.append(("axle", m.cylinder(0.007, 0.25, (0, 0, 0), axis="x", segments=16), "steel"))
    parts.append(("axle_nut", m.cylinder(0.011, 0.010, (X_LEG - 0.020, 0, 0), axis="x", segments=6), "steel"))
    cols.append(m.col_hull("axle_nut_c", m.cylinder(0.011, 0.010, (X_LEG - 0.020, 0, 0), axis="x", segments=6)))
    for name, mesh in zip(("cal_out", "cal_in", "cal_bridge"), _caliper_mesh_parts()):
        parts.append((name, mesh, "caliper"))
        cols.append(m.col_hull(name + "_c", mesh))
    # the fender over the tyre, from 35 to 150 degrees
    for k, a in enumerate(np.linspace(35.0, 150.0, 12)):
        poly = [r * np.array([math.cos(math.radians(b)), math.sin(math.radians(b))]) for r in (0.232, 0.236) for b in (a - 5.5, a + 5.5)]
        mesh = _slab(poly, -0.060, 0.060)
        parts.append((f"fender_{k}", mesh, "body"))
        cols.append(m.col_hull(f"fender_{k}_c", mesh))
    top = LEG_L * LEG_DIR
    crown = m.box((0.26, 0.06, 0.045), (0.0, top[1], top[2]))
    parts.append(("crown", crown, "fork"))
    cols.append(m.col_box("crown_c", (0.26, 0.06, 0.045), (0.0, top[1], top[2])))
    stem = m.tube([top, top + 0.18 * LEG_DIR], 0.022, sides=16)
    parts.append(("stem", stem, "fork"))
    cols.append(m.col_hull("stem_c", stem))
    return parts, cols


_fp, _fc = _frame()
SCOOTER = m.mechanism("scooter", tuple(W), [_wheel_link()], LOOKS, frame_parts=_fp, frame_colliders=_fc, friction=(1.0, 0.9), solver_iters=32)


def _lock_body():
    parts, cols = [], []
    ch = 0.0015  # chamfer on the jaws' mouths
    outer = [(X_OUT, Y_TIP), (-GAP / 2 - ch, Y_TIP), (-GAP / 2, Y_TIP + ch), (-GAP / 2, Y_BACK), (X_OUT, Y_BACK)]
    inner = [(GAP / 2, Y_TIP + ch), (GAP / 2 + ch, Y_TIP), (X_IN, Y_TIP), (X_IN, Y_SLOT), (GAP / 2, Y_SLOT)]
    spine = [(-GAP / 2, Y_SLOT), (X_IN, Y_SLOT), (X_IN, Y_BACK), (-GAP / 2, Y_BACK)]
    for name, out in (("jaw_out", outer), ("jaw_in", inner), ("spine", spine)):
        mesh = m.prism(np.array(out, float), -H / 2, H / 2)
        parts.append((name, mesh, "lock"))
        cols.append(m.col_hull(name + "_c", mesh))
    # the key cylinder's face on the back, and the catch housing's bezel round the plunger
    parts.append(("key_face", m.cylinder(0.009, 0.002, ((X_OUT + X_IN) / 2 - 0.004, Y_BACK + 0.001, 0.0), axis="y", segments=24), "chrome"))
    parts.append(("key_slot", m.box((0.0025, 0.0012, 0.009), ((X_OUT + X_IN) / 2 - 0.004, Y_BACK + 0.0022, 0.0)), "dark"))
    parts.append(("bezel", m.cylinder(CAP_R + 0.002, 0.0012, (X_OUT - 0.0006, 0.0, 0.0), axis="x", segments=24), "chrome"))
    return parts, cols


def _plunger():
    pin_x0, pin_x1 = X_OUT - CAP_OUT + 0.002, PIN_TIP0
    pin = m.join(m.cylinder(PIN_R, pin_x1 - pin_x0 - 0.0006, ((pin_x0 + pin_x1 - 0.0006) / 2, 0, 0), axis="x", segments=16),
                 m.cylinder(PIN_R - 0.0006, 0.0006, (pin_x1 - 0.0003, 0, 0), axis="x", segments=16))
    cap = m.cylinder(CAP_R, CAP_OUT, (X_OUT - CAP_OUT / 2, 0, 0), axis="x", segments=24)
    parts = [("pin", pin, "steel"), ("cap", cap, "dark")]
    cols = [m.col_hull("pin_c", m.join(m.cylinder(PIN_R, 0.012, (pin_x1 - 0.0006 - 0.006, 0, 0), axis="x", segments=16),
                                       m.cylinder(PIN_R - 0.0006, 0.0006, (pin_x1 - 0.0003, 0, 0), axis="x", segments=16))),
            m.col_hull("cap_c", cap)]
    return m.slider("plunger", "X", 0.0, TRAVEL, parts, cols, mass=PLUNGER_MASS, spring=PLUNGER_K, damping=2.0, friction=0.3, effort=60.0,
                    armature=0.002, inertia=(2e-6, 4e-6, 4e-6))


_lp, _lc = _lock_body()
LOCK = m.mechanism("disc_lock", tuple(LOCK_START), [_plunger()], LOOKS, frame_parts=_lp, frame_colliders=_lc, free_mass=LOCK_MASS,
                   frame_inertia=(3.3e-4, 1.1e-4, 3.5e-4), friction=(0.8, 0.7), solver_iters=32)

# -- distractors (SimReady): sunglasses and a coffee cup on the floor in front of the scooter, a rag under its nose
SUNGLASSES = SimReady("sunglasses", pos=(0.16, -0.20, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                      usd="sm_accessory_eyeglasses_a01_01.usd", yaw=0.4)
CUP = SimReady("coffee_cup", pos=(0.62, 0.30, TOP), size=(0.085, 0.085, 0.117), mass=0.25, package="food_beverage_coffeeCup_a01",
               usd="sm_food_beverage_coffeeCup_a01_01.usd")
RAG = SimReady("rag", pos=(0.20, 0.38, TOP), size=(0.260, 0.233, 0.021), mass=0.04, package="cleaning_cloth_a01", usd="sm_cleaning_cloth_a01_01.usd",
               yaw=0.3)

OBJECTS = [SCOOTER, LOCK, SUNGLASSES, CUP, RAG]


# -- what the scorer reads -----------------------------------------------------------------------------------------------------


def pin_offset(lock_state, wheel_deg: float) -> float:
    """How far (m) the pin's axis passes, at the disc's mid-plane, from the centre of the nearest vent hole."""
    R = lock_state.rotation()
    p, d = np.asarray(lock_state.pos, float), R[:, 0]
    if abs(d[0]) < 0.3:  # the pin is nowhere near square to the disc
        return 1.0
    s = (W[0] + X_D - p[0]) / d[0]
    q = p + s * d - W
    best = 1.0
    for phi in HOLE_DEGS:
        a = math.radians(phi + wheel_deg)
        best = min(best, float(math.hypot(q[1] - R_PITCH * math.cos(a), q[2] - R_PITCH * math.sin(a))))
    return best


LOCK_BOX = (np.array([X_OUT - CAP_OUT, Y_TIP, -H / 2]), np.array([X_IN, Y_BACK, H / 2]))


def lock_points(lock_state) -> np.ndarray:
    """Points over the lock's box (body and cap), 3 x 9 x 3, no more than 12 mm apart: what can touch the caliper or the fork."""
    lo, hi = LOCK_BOX
    pts = np.array([[x, y, z] for x in np.linspace(lo[0], hi[0], 3) for y in np.linspace(lo[1], hi[1], 9) for z in np.linspace(lo[2], hi[2], 3)])
    return lock_state.to_world(pts)


def hands_gap(lock_state, tips) -> float:
    """The nearest fingertip point's distance (m) from the lock's box (body and cap)."""
    q = lock_state.to_local(tips)
    lo, hi = LOCK_BOX
    d = np.maximum(np.maximum(lo - q, q - hi), 0.0)
    return float(np.min(np.linalg.norm(d, axis=1)))


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + np.outer(t, ab)), axis=1)


def strike_gap(lock_state) -> float:
    """The nearest gap (m) between the lock (points over its box) and the caliper or the fork leg on the disc side."""
    pts = lock_points(lock_state) - W
    leg = float(np.min(_seg_dist(pts, np.array([X_LEG, 0, 0]), np.array([X_LEG, 0, 0]) + LEG_L * LEG_DIR))) - R_LEG
    a = math.radians(CAL_DEG)
    e_n, e_t = np.array([0, math.cos(a), math.sin(a)]), np.array([0, -math.sin(a), math.cos(a)])
    rel = pts - CAL_R * e_n
    local = np.c_[rel[:, 0] - (CAL_X0 + CAL_X1) / 2, rel @ e_t, rel @ e_n]
    half = np.array([(CAL_X1 - CAL_X0) / 2, CAL_SIZE[1] / 2, CAL_SIZE[2] / 2 + 0.003])
    cal = float(np.min(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0), axis=1)))
    return min(leg, cal)


def _joint_vel(env, part, link) -> float:
    asset = env.unwrapped.scene[part]
    return float(m._np(asset.data.joint_vel)[0, asset.find_joints(f"{link}_joint")[0][0]])


class Drag:
    """The wheel's friction (bearings, pads and the tyre on the floor), as an elasto-plastic (Coulomb) model on the wheel's joint drive:
    a stiff spring holds the wheel at an anchor angle; when the wheel is pushed more than WHEEL_HOLD past it, the anchor follows,
    WHEEL_DRAG behind. So the wheel stays put under less than WHEEL_HOLD and turns against WHEEL_DRAG under more."""

    def __init__(self):
        self.anchor = 0.0

    def reset(self, env=None):
        self.anchor = 0.0
        if env is not None:
            self.anchor = m.state(env, "scooter").joint("wheel")
            m.set_target(env, "scooter", "wheel", self.anchor)

    def step(self, env):
        th = m.state(env, "scooter").joint("wheel")
        if abs(th - self.anchor) * WHEEL_K > WHEEL_HOLD:
            self.anchor = th - math.copysign(WHEEL_DRAG / WHEEL_K, th - self.anchor)
            m.set_target(env, "scooter", "wheel", self.anchor)


DRAG = Drag()
m.on_step(DRAG.step)
m.on_reset(DRAG.reset)


class Catch:
    """The lock's catch: once the plunger has gone CATCH in, it holds it there."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.caught = False

    def step(self, env):
        if not self.caught and m.state(env, "disc_lock").joint("plunger") >= CATCH:
            m.lock(env, "disc_lock", "plunger")
            self.caught = True


CATCHER = Catch()
m.on_step(CATCHER.step)
m.on_reset(CATCHER.reset)



class Trial:
    """The score and the record of a struck caliper or fork."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.struck = False
        self.score = 0.0
        self.last = {}

    def step(self, env):
        lk, sc = m.state(env, "disc_lock"), m.state(env, "scooter")
        wheel = sc.deg("wheel")
        plunger = lk.joint("plunger")
        latched = CATCHER.caught and plunger >= CATCH - 0.0005
        off = pin_offset(lk, wheel)
        locked = latched and off <= HOLE_R
        strike = strike_gap(lk)
        if latched and strike <= 0.003:  # the locked lock carried into the caliper or the fork
            self.struck = True
        gap = hands_gap(lk, m.fingertips(env))
        still = float(np.linalg.norm(lk.lin_vel)) < 0.01 and float(np.linalg.norm(lk.ang_vel)) < 0.1 and abs(_joint_vel(env, "scooter", "wheel")) < 0.02
        ok = locked and gap >= 0.03 and still and not self.struck
        self.hold = self.hold + 1 if ok else 0
        if self.struck:
            self.score = 0.0
        elif locked and self.hold >= HOLD_STEPS:
            self.score = 1.0
        elif locked:
            self.score = 0.5
        else:
            self.score = 0.0
        self.last = dict(wheel=wheel, plunger=plunger, latched=latched, off=off, strike=strike, gap=gap, still=still, hold=self.hold)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.hold >= HOLD_STEPS and not TRIAL.struck)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published starting layout) -----------------------------------------------------------

HOLE_AT = HOLE_DEGS[0]  # the demo puts the lock on the vent hole 20 degrees below the disc's front point
LOCK_AT = W + np.array([X_D, R_PITCH * math.cos(math.radians(HOLE_AT)), R_PITCH * math.sin(math.radians(HOLE_AT))])
# open-loop correction, measured in the simulator: where the held lock sags to under the arm's give
LEFT_TRIM = np.array([-0.0005, -0.0071, 0.0283])
PRESS_PAST = 0.010  # the right fingertips are sent this far past flush (the arm gives under the plunger's spring)


def _rot_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _finger(width, force):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER
    return float(np.clip(-(width / 2 - force / GRIPPER["stiffness"]), FINGER_OPEN, FINGER_CLOSED))


def demo():
    return _demo_full()


def _demo_full():
    """Open loop, from the published starting layout. Left: takes the lock off the floor by its back (a 45-degree grip from above and
    behind, fingers across its 38 mm), carries it to the disc, turning it 20 degrees on the way, slides the disc's edge into its jaw
    with the pin over the vent hole 20 degrees below the disc's front point, and holds it there. Right: comes in front of the lock and
    presses the plunger's cap flush with its closed fingertips, then backs away. The left lets go last; the lock hangs on its pin."""
    import demokit as d
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
    from yam_arena.demo import Waypoint as Wp

    g0 = d.tilted(45.0, -90.0)
    a0 = g0.rot[:, 2]  # the fingers' direction, picking up
    grip_local = np.array([(X_OUT + X_IN) / 2, 0.047, 0.006]) - 0.006 * a0  # the TCP in the lock's frame (as it lies on the floor)
    tcp_pick = LOCK_START + grip_local
    R = _rot_x(HOLE_AT)
    gl = d._as_grip(R @ g0.rot)
    tcp_at = LOCK_AT + R @ grip_local + LEFT_TRIM
    out = R @ np.array([0.0, 1.0, 0.0])  # out of the jaw: radially out from the axle
    hold = _finger(X_IN - X_OUT, 30.0)
    t_place = 3.0 + 1.2 + 0.7 + 1.1 + 2.5 + 1.5
    t_press_done = t_place + 0.5 + 2.5 + 1.5 + 1.5 + 0.6 + 1.0
    left = [
        Wp(3.0, tuple(tcp_pick - 0.06 * a0), FINGER_OPEN, "over the lock", g0),
        Wp(1.2, tuple(tcp_pick), FINGER_OPEN, "round the lock's back", g0),
        Wp(0.7, tuple(tcp_pick), hold, "grip", g0),
        Wp(1.1, tuple(tcp_pick + (0, 0, 0.08)), hold, "lift", g0),
        Wp(2.5, tuple(tcp_at + 0.035 * out), hold, "in front of the disc", gl),
        Wp(1.5, tuple(tcp_at), hold, "the disc into the jaw", gl),
        Wp(t_press_done - t_place + 0.3, tuple(tcp_at), hold, "hold the lock", gl),
        Wp(0.6, tuple(tcp_at), FINGER_OPEN, "let go", gl),
        Wp(1.2, tuple(tcp_at + 0.05 * out - (0, 0, 0.01)), FINGER_OPEN, "back off", gl),
        Wp(1.2, tuple(tcp_at + 0.10 * out - (0, 0, 0.03)), FINGER_OPEN, "clear", gl),
    ]
    gp = d.level(0.0)
    cap_face = LOCK_AT[0] + X_OUT - CAP_OUT
    pre = np.array([cap_face - PAD_TIP_PAST_TCP - 0.03, LOCK_AT[1], LOCK_AT[2]])
    home = np.array([LOCK_AT[0] + X_OUT - PAD_TIP_PAST_TCP + PRESS_PAST, LOCK_AT[1], LOCK_AT[2]])
    right = [
        Wp(3.0, (0.18, -0.12, TOP + 0.20), FINGER_CLOSED, "up", gp),
        Wp(t_place - 3.0 + 0.5, (0.18, -0.12, TOP + 0.20), FINGER_CLOSED, "wait", gp),
        Wp(2.5, tuple(pre), FINGER_CLOSED, "in front of the plunger", gp),
        Wp(1.5, tuple(pre + (0.02, 0, 0)), FINGER_CLOSED, "up to the cap", gp),
        Wp(1.5, tuple(home), FINGER_CLOSED, "press the plunger flush", gp),
        Wp(0.6, tuple(home), FINGER_CLOSED, "hold it", gp),
        Wp(1.0, tuple(pre - (0.02, 0, 0)), FINGER_CLOSED, "back off", gp),
        Wp(1.4, (0.20, -0.12, TOP + 0.22), FINGER_CLOSED, "clear", gp),
    ]
    return {"left": left, "right": right}
