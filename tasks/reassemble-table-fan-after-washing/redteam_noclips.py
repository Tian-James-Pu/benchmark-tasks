"""On the station table: a 12-inch oscillating desk fan, switched off and unplugged, lies on its back with its face
up after its grilles and blade were washed. Its motor housing rests on the table with the steel motor shaft pointing
straight up at x 0.26, y 0.00; the rear grille is already on, held by its plastic mounting nut, its rim 12.2 cm above
the table. The pillar and the base stand off behind it (towards +x). The shaft turns freely in its bearings; an iron
cross pin goes through it 10.7 cm up, and its end above that is threaded (a left-hand thread). The washed, dry parts
lie on the table: the three-blade fan blade (30.5 cm across, light grey plastic) face up on the left, its hub at
x 0.20, y 0.29; the white blade knob (3.6 cm across, 2 cm tall) standing on its open end in front on the right at
x 0.14, y -0.20; the white front grille (34.8 cm across, its four grille clips open, its centre badge up) on the right
at x 0.29, y -0.37. A sponge and a mug stand at the back of the table.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 2 s hold
SCORE_OVER = "end"

# ---------------------------------------------------------------------------------------------------------------
# Dimensions (metres). The fan is a Utilitech TP30-1A 12 in oscillating desk fan (12 in = 30.48 cm blade); its manual
# gives the parts and how they go together, not their sizes, so the sizes below are our estimates (README).
# Heights are above the table top, in the fan's frame (origin on the shaft axis, on the table).
FAN_XY = np.array([0.26, 0.0])
Z_MOTOR = 0.070  # the motor's front face (the fan lies on its back)
R_SHAFT = 0.004  # 8 mm motor shaft
PIN_Z, PIN_R, PIN_L = 0.092, 0.001, 0.014  # iron cross pin: 2 mm, 14 mm long, across the shaft along the shaft's x
NOTCH_W, NOTCH_D = 0.0026, 0.004  # the cross slot in the back of the blade hub
HUB_SEATED = PIN_Z + PIN_R - NOTCH_D  # the hub's back face when the pin is in its slot: 0.089
HUB_H_MAIN, HUB_H = 0.016, 0.034  # the hub (blade roots) and its front boss, which the knob bears on
R_BORE, R_HUB, R_BOSS = 0.0044, 0.024, 0.015  # bore drawn as a 32-gon: 0.38 mm clear of the 8 mm shaft
BOSS_TOP = HUB_SEATED + HUB_H  # 0.123
PITCH = 0.0015  # blade-knob thread: left-hand, 1.5 mm pitch (estimate)
TH_MAJ, TH_MIN = 0.0040, 0.00308  # external thread radii on the shaft end
TH_CLEAR = 0.0002  # radial clearance of the knob's internal thread
TH_Z0, TH_Z1 = 0.1185, 0.1280  # threaded length of the shaft end (its top is chamfered)
KNOB_R, KNOB_H, KNOB_BORE = 0.018, 0.020, 0.0080
KNOB_SEATED = BOSS_TOP  # the knob's open end on the hub's boss
Z_RIM = 0.107  # top of the rear grille's rim lip
R_REAR_IN, R_REAR_OUT, R_LIP_OUT = 0.162, 0.166, 0.174
R_FRONT_SKIRT, R_FRONT_IN, R_FRONT_OUT = 0.157, 0.161, 0.174
R_CLIP = 0.1765  # clip hinge radius on the front grille
CLIP_OPEN_DEG = 75.0  # clips start swung out this far; closed is +75 deg about their tangent (joint value)
CLIP_SNAP_DEG = 40.0  # past this the over-centre spring takes the clip shut, short of it back open
BLADE_R = 0.1524  # 12 in
BLADE_PITCH = math.radians(15.0)
BLADE_ZC = 0.0125
SHAFT_FRICTION, SHAFT_DAMPING = 0.004, 0.004  # bearing drag of the motor shaft (estimate; it still turns easily by hand)

LOOKS = {
    "white": m.Look(color=m.rgb(0.93, 0.93, 0.91), roughness=0.5),
    "grille": m.Look(color=m.rgb(0.95, 0.95, 0.94), roughness=0.45),
    "blade": m.Look(color=m.rgb(0.78, 0.80, 0.83), roughness=0.4),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.66), metallic=0.8, roughness=0.35),
    "iron": m.Look(color=m.rgb(0.30, 0.30, 0.32), metallic=0.7, roughness=0.5),
    "grey": m.Look(color=m.rgb(0.55, 0.56, 0.58), roughness=0.6),
    "badge": m.Look(color=m.rgb(0.25, 0.27, 0.30), roughness=0.4),
    "clip": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.5),
}


# ---------------------------------------------------------------------------------------------------------------
# Geometry helpers


def _hull_pts(name: str, pts) -> str:
    """A convex-hull collider round 8 points ordered (end 0|1) x (side -|+) x (face -|+)."""
    return m.col_hull(name, _hull_mesh(np.asarray(pts, float)))


def _clip_half(poly, keep):
    """Sutherland-Hodgman: keep the part of a convex polygon where keep(p) >= 0 (keep is linear)."""
    out = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        fa, fb = keep(a), keep(b)
        if fa >= 0:
            out.append(a)
        if (fa >= 0) != (fb >= 0):
            t = fa / (fa - fb)
            out.append(a + t * (b - a))
    return out


def _area(poly) -> float:
    if len(poly) < 3:
        return 0.0
    p = np.asarray(poly)
    return 0.5 * abs(float(np.dot(p[:, 0], np.roll(p[:, 1], -1)) - np.dot(p[:, 1], np.roll(p[:, 0], -1))))


def _sector(r0, r1, a0, a1):
    am = 0.5 * (a0 + a1)
    return [np.array([r0 * math.cos(a0), r0 * math.sin(a0)]), np.array([r1 * math.cos(a0), r1 * math.sin(a0)]),
            np.array([r1 * math.cos(am), r1 * math.sin(am)]), np.array([r1 * math.cos(a1), r1 * math.sin(a1)]),
            np.array([r0 * math.cos(a1), r0 * math.sin(a1)])]


def _ccw(poly):
    p = np.asarray(poly)
    c = p.mean(axis=0)
    ang = np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])
    return p[np.argsort(ang)]


def _ring(prefix, r0, r1, z0, z1, n, look, parts, cols, a_off=0.0, visual=True):
    """An annulus from n convex sectors: each one collider, and the same shapes drawn."""
    for i in range(n):
        a0, a1 = a_off + 2 * math.pi * i / n, a_off + 2 * math.pi * (i + 1) / n
        poly = _ccw(_sector(r0, r1, a0, a1))
        mesh = m.prism(poly, z0, z1)
        cols.append(m.col_hull(f"{prefix}{i}_c", mesh))
        if visual:
            parts.append((f"{prefix}{i}", mesh, look))


def _rib(p0, p1, w):
    """A straight bar of square section w from p0 to p1 (a convex hull's points)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    u = np.cross(d, [0, 0, 1.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(d, [1.0, 0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    pts = [p + s * u * w / 2 + t * v * w / 2 for p in (p0, p1) for s in (-1, 1) for t in (-1, 1)]
    return np.array(pts)


def _radial(radii, zs, theta, inner=False):
    n = len(theta)
    v = np.concatenate([np.c_[r * np.cos(theta), r * np.sin(theta), np.full(n, z)] for z, r in zip(zs, radii)])
    f = []
    for i in range(len(zs) - 1):
        for j in range(n):
            a, b = i * n + j, i * n + (j + 1) % n
            f += [(a, b, b + n), (a, b + n, a + n)]
    f = np.array(f)
    return v, (f[:, ::-1] if inner else f)


def _profile(u):
    """Trapezoidal thread form across one pitch: 0 at the root radius, 1 at the crest."""
    return np.interp(np.mod(u, 1.0), [0, 0.125, 0.4375, 0.5625, 0.875, 1.0], [0, 0, 1, 1, 0, 0])


def _lh_phase(theta, z):
    # Left-hand thread: a point keeps its phase as it turns anticlockwise (seen from the end) and moves down
    return z / PITCH + theta / (2 * math.pi)


def col_sdf(name: str, mesh: m.Mesh, res: int = 256) -> str:
    """A triangle-mesh collider kept concave as a signed distance field (for the threads)."""
    t = m.col_hull(name, mesh)
    t = t.replace('"PhysicsMeshCollisionAPI"', '"PhysicsMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"')
    t = t.replace('physics:approximation = "convexHull"', f'physics:approximation = "sdf"\n        int physxSDFMeshCollision:sdfResolution = {res}')
    return t.replace("contactOffset = 0.004", "contactOffset = 0.0008")


def male_thread() -> m.Mesh:
    """The shaft's threaded end, in the fan's frame: a solid with a 1.5 mm left-hand thread, its top chamfered."""
    theta = np.linspace(0, 2 * math.pi, 144, endpoint=False)
    zs = np.linspace(TH_Z0, TH_Z1, 128)
    radii = []
    for z in zs:
        r = TH_MIN + (TH_MAJ - TH_MIN) * _profile(_lh_phase(theta, z))
        cap = TH_MAJ - (z - (TH_Z1 - 0.0010)) * (TH_MAJ - TH_MIN) / 0.0010  # 1 mm lead-in chamfer
        radii.append(np.minimum(r, max(cap, TH_MIN - 0.0002)) if z > TH_Z1 - 0.0010 else r)
    v, f = _radial(radii, zs, theta)
    n = len(theta)
    f = f.tolist()
    for i, up in ((0, False), (len(zs) - 1, True)):
        c = len(v)
        v = np.vstack((v, [0, 0, zs[i]]))
        for j in range(n):
            tri = (c, i * n + j, i * n + (j + 1) % n)
            f.append(tri if up else tri[::-1])
    return m.Mesh(v, np.asarray(f))


def female_thread() -> m.Mesh:
    """The knob's threaded bore, in the knob's frame (origin at its open end): a sleeve from the thread out to 5 mm."""
    theta = np.linspace(0, 2 * math.pi, 144, endpoint=False)
    zs = np.linspace(0.0, KNOB_BORE, 112)
    rin = []
    for z in zs:
        r = TH_MIN + TH_CLEAR + (TH_MAJ - TH_MIN) * _profile(_lh_phase(theta, z))
        if z < 0.0010:  # 1 mm lead-in chamfer at the open end
            r = np.maximum(r, TH_MAJ + TH_CLEAR + (0.0010 - z) * 0.5)
        rin.append(r)
    ro = 0.0052
    iv, iff = _radial(rin, zs, theta, inner=True)
    ov, off = _radial([np.full(len(theta), ro)] * 2, [zs[0], zs[-1]], theta)
    n = len(theta)
    v = np.vstack((ov, iv))
    f = np.vstack((off, iff + len(ov))).tolist()
    for o, i, up in ((0, len(ov), False), (n, len(ov) + (len(zs) - 1) * n, True)):
        for j in range(n):
            k = (j + 1) % n
            tris = [(o + j, i + k, o + k), (o + j, i + j, i + k)]
            f.extend([t[::-1] for t in tris] if up else tris)
    return m.Mesh(v, np.asarray(f))


# ---------------------------------------------------------------------------------------------------------------
# The fan: body and rear grille fixed (the frame), the motor shaft on a free bearing (a link)


def _fan():
    parts, cols = [], []
    # motor housing lying on its back, and the pillar and base behind it
    housing = m.lathe([(0.0, 0.0), (0.050, 0.0), (0.058, 0.015), (0.060, 0.070), (0.052, 0.082), (0.040, Z_MOTOR), (0.0, Z_MOTOR)], 48)
    parts.append(("motor_housing", housing, "white"))
    cols.append(m.col_cylinder("motor_c", 0.058, Z_MOTOR - 0.002, (0, 0, (Z_MOTOR - 0.002) / 2)))
    nut = m.cylinder(0.024, 0.015, (0, 0, Z_MOTOR + 0.0075), segments=24)
    parts.append(("grille_nut", nut, "white"))
    cols.append(m.col_cylinder("nut_c", 0.024, 0.015, (0, 0, Z_MOTOR + 0.0075)))
    pillar = m.box((0.130, 0.050, 0.040), (0.058 + 0.065, 0, 0.022))
    parts.append(("pillar", pillar, "white"))
    cols.append(m.col_box("pillar_c", (0.130, 0.050, 0.040), (0.058 + 0.065, 0, 0.022)))
    base = m.cylinder(0.105, 0.030, (0.205, 0, 0.105), axis="x", segments=40)
    parts.append(("base", base, "white"))
    cols.append(m.col_hull("base_c", base))
    # rear grille: centre plate under the nut, 24 radial wires to the rim, three rings, rim band and lip; every
    # wire drawn is also a collider
    parts.append(("rear_plate", m.cylinder(0.034, 0.003, (0, 0, Z_MOTOR + 0.0015)), "grille"))
    rim_z = Z_RIM - 0.021  # where the wires meet the band
    for i in range(24):
        a = 2 * math.pi * (i + 0.5) / 24
        e = np.array([math.cos(a), math.sin(a), 0])
        pts = _rib(e * 0.030 + [0, 0, Z_MOTOR + 0.002], e * 0.163 + [0, 0, rim_z], 0.003)
        cols.append(_hull_pts(f"rear_rib{i}_c", pts))
        parts.append((f"rear_rib{i}", _hull_mesh(pts), "grille"))
    for k, r in enumerate((0.060, 0.100, 0.135)):
        z = Z_MOTOR + 0.002 + (r - 0.030) / 0.133 * (rim_z - Z_MOTOR - 0.002)
        _ring(f"rear_ring{k}_", r - 0.0015, r + 0.0015, z - 0.0015, z + 0.0015, 24, "grille", parts, cols)
    _ring("rear_band", R_REAR_IN, R_REAR_OUT, rim_z - 0.001, Z_RIM, 24, "grille", parts, cols)
    _ring("rear_lip", R_REAR_OUT, R_LIP_OUT, Z_RIM - 0.006, Z_RIM, 24, "grille", parts, cols)

    # the motor shaft: plain from the motor up, the cross pin, the threaded end
    sp, sc = [], []
    shaft = m.cylinder(R_SHAFT, TH_Z0 + 0.0005 - 0.060, (0, 0, (0.060 + TH_Z0 + 0.0005) / 2), segments=24)
    sp.append(("shaft", shaft, "steel"))
    sc.append(m.col_cylinder("shaft_c", R_SHAFT, TH_Z0 + 0.0005 - 0.060, (0, 0, (0.060 + TH_Z0 + 0.0005) / 2)))
    pin = m.cylinder(PIN_R, PIN_L, (0, 0, PIN_Z), axis="x", segments=12)
    sp.append(("cross_pin", pin, "iron"))
    sc.append(m.col_hull("pin_c", pin))
    thread = male_thread()
    sp.append(("thread", thread, "steel"))
    sc.append(col_sdf("thread_c", thread))
    shaft_link = m.hinge("shaft", "Z", (0, 0, 0), float("-inf"), float("inf"), sp, sc, mass=0.30, damping=SHAFT_DAMPING,
                         friction=SHAFT_FRICTION, armature=0.0, inertia=(2e-4, 2e-4, 1.0e-4))
    return m.mechanism("fan", (FAN_XY[0], FAN_XY[1], TOP), [shaft_link], LOOKS, frame_parts=parts, frame_colliders=cols,
                       friction=(0.35, 0.30), solver_iters=48)


# ---------------------------------------------------------------------------------------------------------------
# The blade: hub with a plain bore and a cross slot across its back, a front boss, three pitched blades


BLADE_ANGLES = (90.0, 210.0, 330.0)  # in the blade's own frame; the slot runs along its x
BLADE_START = np.array([0.20, 0.29])
BLADE_START_YAW = 12.0  # deg: no blade points at the fan


def _blade_pts(a_deg, r0, r1, c0, c1, th=0.0025):
    a = math.radians(a_deg)
    er, et, ez = np.array([math.cos(a), math.sin(a), 0]), np.array([-math.sin(a), math.cos(a), 0]), np.array([0, 0, 1.0])
    chord = math.cos(BLADE_PITCH) * et + math.sin(BLADE_PITCH) * ez  # the trailing (anticlockwise) edge is higher
    normal = -math.sin(BLADE_PITCH) * et + math.cos(BLADE_PITCH) * ez
    pts = []
    for r, c in ((r0, c0), (r1, c1)):
        for s in (-c / 2, c / 2):
            for n in (-th / 2, th / 2):
                pts.append(r * er + s * chord + n * normal + BLADE_ZC * ez)
    return np.array(pts)


def _blade():
    parts, cols = [], []
    # hub, upper layer: plain bore
    _ring("hub", R_BORE, R_HUB, NOTCH_D, HUB_H_MAIN, 32, "blade", parts, cols)
    _ring("boss", R_BORE, R_BOSS, HUB_H_MAIN, HUB_H, 32, "blade", parts, cols)
    # hub, back layer: the same sectors with the cross slot (|y| < NOTCH_W / 2) cut out, and a 1 mm bore chamfer
    k = 0
    for i in range(32):
        a0, a1 = 2 * math.pi * i / 32, 2 * math.pi * (i + 1) / 32
        sec = _sector(R_BORE + 0.0008, R_HUB, a0, a1)
        for keep in (lambda p: p[1] - NOTCH_W / 2, lambda p: -p[1] - NOTCH_W / 2):
            poly = _clip_half(sec, keep)
            if _area(poly) > 2e-7:
                mesh = m.prism(_ccw(poly), 0.0, NOTCH_D)
                cols.append(m.col_hull(f"back{k}_c", mesh))
                parts.append((f"back{k}", mesh, "blade"))
                k += 1
    # three blades, each from three convex panels (root, middle, tip)
    for b, a in enumerate(BLADE_ANGLES):
        for j, (r0, r1, c0, c1) in enumerate(((0.020, 0.060, 0.030, 0.080), (0.060, 0.115, 0.080, 0.095), (0.115, BLADE_R, 0.095, 0.060))):
            pts = _blade_pts(a, r0, r1, c0, c1)
            cols.append(_hull_pts(f"blade{b}_{j}_c", pts))
            parts.append((f"blade{b}_{j}", _hull_mesh(pts), "blade"))
    return m.free("blade", (BLADE_START[0], BLADE_START[1], TOP + 0.001), parts, cols, LOOKS, mass=0.075, friction=(0.45, 0.40),
                  yaw_deg=BLADE_START_YAW, solver_iters=48)


def _hull_mesh(pts) -> m.Mesh:
    """A box-like hexahedron from 8 points ordered (r0|r1) x (s-|s+) x (n-|n+)."""
    q = np.asarray(pts)
    faces = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    return m.Mesh(q, np.array(faces))


# ---------------------------------------------------------------------------------------------------------------
# The blade knob: knurled cap with a blind, left-hand threaded bore

KNOB_START = np.array([0.12, -0.23])


def _knob():
    parts, cols = [], []
    _ring("wall", 0.0052, KNOB_R, 0.0, KNOB_BORE, 10, "white", parts, cols)
    top = m.lathe([(0.0, KNOB_BORE), (KNOB_R, KNOB_BORE), (KNOB_R, KNOB_H - 0.003), (KNOB_R - 0.003, KNOB_H), (0.0, KNOB_H)], 40)
    parts.append(("crown", top, "white"))
    cols.append(m.col_hull("crown_c", top))
    sleeve = female_thread()
    parts.append(("thread", sleeve, "white"))
    cols.append(col_sdf("thread_c", sleeve))
    for i in range(24):  # knurl ribs, drawn
        a = 2 * math.pi * i / 24
        parts.append((f"knurl{i}", m.box((0.0012, 0.0016, KNOB_H - 0.006), (0, 0, 0)).turned(math.degrees(a), "z").moved((KNOB_R * math.cos(a), KNOB_R * math.sin(a), (KNOB_H - 0.006) / 2 + 0.001)), "white"))
    return m.free("knob", (KNOB_START[0], KNOB_START[1], TOP + 0.0005), parts, cols, LOOKS, mass=0.012, friction=(0.6, 0.5), solver_iters=48)


# ---------------------------------------------------------------------------------------------------------------
# The front grille: rim with an inner locating skirt, domed wires to a centre badge, four hinged grille clips

GRILLE_START = np.array([0.32, -0.38])
GRILLE_YAW = 45.0  # clips at 45, 135, 225 and 315 deg in the world (reachable on the fan)
Z_SKIRT = 0.010  # the skirt hangs this far below the rim
DOME = 0.040  # the badge's underside above the rim's underside
CLIPS = (("clip_e", 0.0, "Y", 1.0), ("clip_n", 90.0, "X", -1.0), ("clip_w", 180.0, "Y", -1.0), ("clip_s", 270.0, "X", 1.0))


def _clip_link(name, psi_deg, axis, sign):
    """A grille clip at angle psi on the rim, hinged about the rim's tangent. Authored swung out by CLIP_OPEN_DEG (its
    start pose, joint zero); shutting it turns it CLIP_OPEN_DEG back, where its hook sits under the rear grille's lip."""
    hz = 0.008
    # closed pose in the clip's radial frame (u out, t along the tangent, z up)
    arm = np.array([[u, t, z] for u in (0.1755, 0.1785) for t in (-0.007, 0.007) for z in (-0.009, 0.009)])
    hook = np.array([[u, t, z] for u in (0.168, 0.1785) for t in (-0.007, 0.007) for z in (-0.009, -0.0065)])
    a = math.radians(-CLIP_OPEN_DEG)  # opening: a negative turn about +t swings the arm outward

    def open_(p):
        x, z = p[:, 0] - R_CLIP, p[:, 2] - hz
        return np.c_[R_CLIP + x * math.cos(a) + z * math.sin(a), p[:, 1], hz - x * math.sin(a) + z * math.cos(a)]

    psi = math.radians(psi_deg)
    rot = np.array([[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1]])
    arm_w, hook_w = open_(arm) @ rot.T, open_(hook) @ rot.T
    at = np.array([R_CLIP * math.cos(psi), R_CLIP * math.sin(psi), hz])
    lo, hi = (-3.0, CLIP_OPEN_DEG) if sign > 0 else (-CLIP_OPEN_DEG, 3.0)
    return m.hinge(name, axis, tuple(at), lo, hi, [("arm", _hull_mesh(arm_w), "clip"), ("hook", _hull_mesh(hook_w), "clip")],
                   [_hull_pts(f"{name}_arm_c", arm_w), _hull_pts(f"{name}_hook_c", hook_w)], mass=0.006, spring=0.15, damping=0.004, friction=0.0, effort=5.0,
                   inertia=(4e-7, 4e-7, 4e-7))


def _grille():
    parts, cols = [], []
    _ring("rim", R_FRONT_IN, R_FRONT_OUT, 0.0, 0.006, 24, "grille", parts, cols)
    # the locating skirt inside the rear grille's band: 4 mm wide, its outside tapering 3 mm towards its lower edge
    for i in range(24):
        a0, a1 = 2 * math.pi * i / 24, 2 * math.pi * (i + 1) / 24
        pts = [np.array([r * math.cos(a), r * math.sin(a), z]) for a in (a0, a1) for r, z in ((R_FRONT_SKIRT, 0.0), (R_FRONT_SKIRT, -Z_SKIRT), (R_FRONT_IN, 0.0), (R_FRONT_IN - 0.003, -Z_SKIRT))]
        mesh = m.Mesh(np.array(pts), np.array([(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6)]))
        cols.append(m.col_hull(f"skirt{i}_c", mesh))
        parts.append((f"skirt{i}", mesh, "grille"))
    for i in range(24):  # 24 domed wires and three rings, each drawn and colliding
        a = 2 * math.pi * (i + 0.5) / 24
        e = np.array([math.cos(a), math.sin(a), 0])
        pts = _rib(e * 0.162 + [0, 0, 0.009], e * 0.032 + [0, 0, DOME + 0.002], 0.003)
        cols.append(_hull_pts(f"rib{i}_c", pts))
        parts.append((f"rib{i}", _hull_mesh(pts), "grille"))
    for k, r in enumerate((0.060, 0.100, 0.135)):
        z = 0.009 + (0.162 - r) / 0.130 * (DOME + 0.002 - 0.009)
        _ring(f"ring{k}_", r - 0.0015, r + 0.0015, z - 0.0015, z + 0.0015, 24, "grille", parts, cols)
    badge = m.cylinder(0.034, 0.006, (0, 0, DOME + 0.003), segments=40)
    parts.append(("badge", badge, "badge"))
    cols.append(m.col_cylinder("badge_c", 0.034, 0.006, (0, 0, DOME + 0.003)))
    boss = m.cylinder(0.020, 0.012, (0, 0, DOME + 0.012), segments=32)
    parts.append(("grip_boss", boss, "badge"))
    cols.append(m.col_cylinder("boss_c", 0.020, 0.012, (0, 0, DOME + 0.012)))
    links = [_clip_link(n, psi, ax, s) for n, psi, ax, s in CLIPS]
    return m.mechanism("grille", (GRILLE_START[0], GRILLE_START[1], TOP + Z_SKIRT + 0.0005), links, LOOKS, frame_parts=parts, frame_colliders=cols,
                       yaw_deg=GRILLE_YAW, friction=(0.45, 0.40), solver_iters=48, free_mass=0.16, frame_inertia=(0.0025, 0.0025, 0.0048))


FAN, BLADE, KNOB, GRILLE = _fan(), _blade(), _knob(), _grille()
OBJECTS = [
    FAN, BLADE, KNOB, GRILLE,
    # distractors, ignored by the scoring
    SimReady("sponge", pos=(0.53, -0.17, TOP), size=(0.071, 0.101, 0.033), mass=0.02, package="cleaning_sponge_b01",
             usd="sm_cleaning_sponge_b01_01.usd", yaw=0.0, convex=True),
    SimReady("mug", pos=(0.49, 0.26, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
             usd="sm_dishware_mug_a01_01.usda", yaw=-math.pi / 2, convex=True),
]


# ---------------------------------------------------------------------------------------------------------------
# Scoring: the tracker reads the parts every control step


HOLD_STEPS = 40  # 2 s at 20 Hz
AXIS = np.array([FAN_XY[0], FAN_XY[1]])


def _off_axis(p) -> float:
    return float(np.linalg.norm(np.asarray(p[:2]) - AXIS))


def blade_on_pin(s) -> bool:
    """The hub down on the shaft with the pin in its slot: back face within 1.5 mm of its seated height, on the axis
    within 1 mm, square within 3 degrees. Resting on the pin with the slot turned away, it stands 4 mm higher."""
    return bool(abs(s.pos[2] - (TOP + HUB_SEATED)) < 0.0015 and _off_axis(s.pos) < 0.001 and s.tilt_deg() < 3.0)


def knob_down(s) -> bool:
    """The knob screwed down onto the hub's boss: its open end within 0.6 mm of the boss, on the axis within 1 mm,
    square within 3 degrees. It can only get there by turning down the thread."""
    return bool(s.pos[2] - (TOP + KNOB_SEATED) < 0.0006 and abs(s.pos[2] - (TOP + KNOB_SEATED)) < 0.002 and _off_axis(s.pos) < 0.001 and s.tilt_deg() < 3.0)


def grille_on(s) -> bool:
    """The front grille's rim on the rear grille's: within 1.5 mm of its seated height, centred within 2 mm, square
    within 2 degrees."""
    return bool(abs(s.pos[2] - (TOP + Z_RIM)) < 0.0015 and _off_axis(s.pos) < 0.002 and s.tilt_deg() < 2.0)


def clips_shut(s) -> int:
    """Grille clips shut: turned to within 3 degrees of shut (75 degrees from their open start)."""
    return int(sum(1 for n, _, _, sgn in CLIPS if sgn * s.deg(n) > CLIP_OPEN_DEG - 3.0))


def still(*states) -> bool:
    return bool(all(np.linalg.norm(s.lin_vel) < 0.01 and np.linalg.norm(s.ang_vel) < 0.2 for s in states))


def hands_clear(tips) -> bool:
    """Every fingertip point more than 20 cm from the fan's axis or more than 3 cm above the top of the front grille."""
    tips = np.atleast_2d(tips)
    r = np.linalg.norm(tips[:, :2] - AXIS, axis=1)
    return bool(np.all((r > 0.20) | (tips[:, 2] > TOP + Z_RIM + DOME + 0.020 + 0.030)))


def tier(blade, knob, grille) -> float:
    """The score for what is in place now, before the release and the hold."""
    if not blade_on_pin(blade):
        return 0.0
    if not knob_down(knob):
        return 0.25
    if not grille_on(grille):
        return 0.5
    return 0.5 + 0.1 * clips_shut(grille)


class Snap:
    """Physics, not scoring: each grille clip's over-centre spring. A plastic clip resists being swung in (its spring
    pulls it back open) until it is pushed past CLIP_SNAP_DEG; from there the spring pulls it on, against its stop with
    the hook under the rear grille's lip. Pushed back past that angle, it springs open again. The clip itself must be
    turned there by a finger and its hook must clear the rear grille's lip: nothing moves it otherwise."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.closed = {n: False for n, _, _, _ in CLIPS}

    def step(self, env):
        g = m.state(env, "grille")
        for n, _, _, sgn in CLIPS:
            shut = sgn * g.deg(n) > CLIP_SNAP_DEG
            if shut != self.closed[n]:
                self.closed[n] = shut
                m.set_target(env, "grille", n, math.radians(sgn * (CLIP_OPEN_DEG + 4.0)) if shut else 0.0)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared: a reset to the starting layout
        self.hold, self.full, self.score, self.steps = 0, False, 0.0, 0

    def step(self, env):
        self.steps += 1
        blade, knob, grille = m.state(env, "blade"), m.state(env, "knob"), m.state(env, "grille")
        t = tier(blade, knob, grille)
        ok = t >= 0.9 and hands_clear(m.fingertips(env)) and still(blade, knob, grille)
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else float(min(t, 0.9))


DEBUG = pathlib.Path("/tmp/claude-1001/-home-mzhang-code-benchmark-tasks/c5e8a00a-04bc-4d39-bc84-61696031f06f/scratchpad/fan_debug.log")


def _debug(trial, env, blade, knob, grille, t):
    if trial.steps % 5 == 0:
        tips = m.fingertips(env)
        lt, rt = tips[:12], tips[12:]
        with DEBUG.with_suffix(".tips").open("a") as f:
            f.write(f"{trial.steps} {' '.join(f'{v:.4f}' for v in lt.mean(axis=0) - [0, 0, TOP])} {lt[:, 2].min() - TOP:.4f} "
                    f"{' '.join(f'{v:.4f}' for v in rt.mean(axis=0) - [0, 0, TOP])} {rt[:, 2].min() - TOP:.4f} "
                    f"{np.linalg.norm(lt[:6].mean(0) - lt[6:].mean(0)):.4f} {np.linalg.norm(rt[:6].mean(0) - rt[6:].mean(0)):.4f}\n")
    if trial.steps == 1 or trial.steps % 20 == 0:
        sh = m.state(env, "fan")
        with DEBUG.open("a") as f:
            f.write(f"{trial.steps} tier={t:.2f} blade={np.round(blade.pos - [0, 0, TOP], 4).tolist()} tilt={blade.tilt_deg():.1f} "
                    f"q={np.round(blade.quat_xyzw, 3).tolist()} knob={np.round(knob.pos - [0, 0, TOP], 4).tolist()} ktilt={knob.tilt_deg():.1f} "
                    f"kq={np.round(knob.quat_xyzw, 3).tolist()} grille={np.round(grille.pos - [0, 0, TOP], 4).tolist()} gtilt={grille.tilt_deg():.1f} "
                    f"clips={[round(grille.deg(n), 1) for n, _, _, _ in CLIPS]} shaft={sh.deg('shaft'):.1f}\n")


SNAP, TRIAL = Snap(), Trial()
m.on_step(SNAP.step)
m.on_reset(SNAP.reset)
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# ---------------------------------------------------------------------------------------------------------------
# Scripted demo: a fixed plan from the published starting layout (open loop)


def _w(d, x, y, z, finger, label, yaw_deg=0.0):
    return Waypoint(float(d), (float(x), float(y), TOP + float(z)), float(finger), label, math.radians(yaw_deg))


def _f(width, force=25.0):
    """The finger target that closes on something `width` across with about `force` newtons a finger."""
    return float(np.clip(-(width / 2 - force / 5000.0), FINGER_OPEN, FINGER_CLOSED))


TCP_OVER_HUB = HUB_H + 0.002  # the grip on the blade's boss: TCP this far above the hub's back face
TCP_OVER_KNOB = KNOB_H - 0.002
TCP_OVER_GRILLE = DOME + 0.018 - 0.001
SLOT_LEAD_DEG = 8.0  # the blade goes onto the shaft turned this far anticlockwise of the pin
CARRY_DX, CARRY_DY = 0.0011, -0.0013  # TCP offset that puts the carried hub on the shaft axis (measured)
KNOB_DX, KNOB_DY = 0.0002, 0.0040  # TCP offset that puts the carried knob on the shaft axis (measured)
PUSH_R, PUSH_Z = 0.14, HUB_SEATED + 0.021  # the fingers that turn and then hold the blade
PUSH_FROM, PUSH_TO = 128.0, 92.0
PUSH_YAW = -90.0  # the left gripper's long side along x, clear of the right hand working at the shaft  # deg about the shaft, from +x
STROKE_DEG = 210.0


def _pad(plan, until, label="wait"):
    """Hold the last pose until time `until` (s from the start)."""
    t = sum(w.duration for w in plan)
    if until > t + 1e-6:
        last = plan[-1]
        plan.append(Waypoint(until - t, last.pos, last.finger, label, last.yaw))


def _clock(plan):
    return sum(w.duration for w in plan)


def _press(plan, at_deg, yaw_deg):
    """Press a grille clip shut: down on its swung-out arm, 18.6 cm out from the fan's axis."""
    a = math.radians(at_deg)
    x, y = FAN_XY[0] + 0.186 * math.cos(a), FAN_XY[1] + 0.186 * math.sin(a)
    plan += [_w(1.2, x, y, Z_RIM + 0.043, FINGER_CLOSED, f"over the {at_deg:.0f} deg clip", yaw_deg),
             _w(0.4, x, y, Z_RIM + 0.032, FINGER_CLOSED, "onto its arm", yaw_deg),
             _w(1.0, x, y, Z_RIM + 0.006, FINGER_CLOSED, "press it shut", yaw_deg),
             _w(0.3, x, y, Z_RIM + 0.006, FINGER_CLOSED, "hold", yaw_deg),
             _w(0.6, x, y, Z_RIM + 0.043, FINGER_CLOSED, "up", yaw_deg)]


def demo():
    cx, cy = FAN_XY
    bx, by = BLADE_START
    kx, ky = KNOB_START
    gx, gy = GRILLE_START
    L, R = [], []
    # Left hand: the blade, taken by its boss, carried over the fan and lowered down the shaft with its slot turned a
    # little anticlockwise of the pin, so that it comes to rest on the pin; let go there.
    yaw0 = BLADE_START_YAW - 30.0  # fingers 30 deg off the blades on both sides
    yaw_set = yaw0 - BLADE_START_YAW + SLOT_LEAD_DEG  # the blade turned to SLOT_LEAD_DEG over the shaft
    sx, sy = cx + CARRY_DX, cy + CARRY_DY  # the TCP target that puts the hub on the axis (measured sag under load)
    L += [_w(2.0, bx, by, 0.10, FINGER_OPEN, "over the blade's boss", yaw0),
          _w(1.5, bx, by, 0.001 + TCP_OVER_HUB, FINGER_OPEN, "down to the boss", yaw0),
          _w(0.8, bx, by, 0.001 + TCP_OVER_HUB, _f(2 * R_BOSS), "grip the boss", yaw0),
          _w(1.5, bx, by, 0.20, _f(2 * R_BOSS), "lift the blade", yaw0),
          _w(2.0, 0.24, 0.12, 0.20, _f(2 * R_BOSS), "over the fan", yaw_set),
          _w(1.5, sx, sy, 0.175, _f(2 * R_BOSS), "over the shaft", yaw_set),
          _w(1.5, sx, sy, 0.175, _f(2 * R_BOSS), "steady", yaw_set),
          _w(1.5, sx, sy, 0.134 + TCP_OVER_HUB, _f(2 * R_BOSS), "just above the shaft's end", yaw_set),
          _w(1.0, sx, sy, 0.134 + TCP_OVER_HUB, _f(2 * R_BOSS), "steady", yaw_set),
          _w(1.2, sx, sy, TH_Z1 - NOTCH_D + 0.0005 + TCP_OVER_HUB, _f(2 * R_BOSS), "counterbore round the shaft's end", yaw_set),
          _w(0.6, sx, sy, TH_Z1 - NOTCH_D + 0.0005 + TCP_OVER_HUB, FINGER_OPEN, "let it slide down the shaft onto the pin", yaw_set),
          _w(1.0, sx, sy, HUB_SEATED + 0.065, FINGER_OPEN, "up", yaw_set)]
    # then it turns the blade clockwise with closed fingers behind its left-pointing blade until the slot drops over
    # the pin, and stays there: the same fingers keep the blade from turning anticlockwise with the knob's thread
    def ring(theta_deg, z):
        a = math.radians(theta_deg)
        return cx + PUSH_R * math.cos(a), cy + PUSH_R * math.sin(a), z
    x0, y0, _ = ring(PUSH_FROM, 0)
    L += [_w(1.5, x0, y0, 0.15, FINGER_CLOSED, "behind a blade", PUSH_YAW),
          _w(1.0, x0, y0, PUSH_Z, FINGER_CLOSED, "fingers down beside it", PUSH_YAW)]
    th = PUSH_FROM
    while th > PUSH_TO + 1e-6:
        th = max(PUSH_TO, th - 3.0)
        x, y, _ = ring(th, 0)
        L.append(_w(0.3, x, y, PUSH_Z, FINGER_CLOSED, "turn the blade until the slot drops on", PUSH_YAW))
    hold = ring(PUSH_TO, 0)[:2]
    t_blade = _clock(L) - 2.5
    # Right hand meanwhile: the knob, carried to a wait point beside the fan
    ky0, kpick = -60.0, 10.0  # pick it with the fingers' line clear of both grilles; turn the wrist on the way
    R += [_w(2.0, kx, ky, 0.10, FINGER_OPEN, "over the knob", kpick),
          _w(1.5, kx, ky, TCP_OVER_KNOB, FINGER_OPEN, "down to the knob", kpick),
          _w(0.8, kx, ky, TCP_OVER_KNOB, _f(2 * KNOB_R), "grip the knob", kpick),
          _w(1.5, kx, ky, 0.20, _f(2 * KNOB_R), "lift it", kpick),
          _w(2.0, 0.14, -0.26, 0.20, _f(2 * KNOB_R), "wait clear of the blade", ky0)]
    _pad(R, t_blade + 0.5)
    rest = TH_Z1 - 0.0015  # the knob's open end resting on the start of the thread (about)
    R += [_w(1.5, cx + KNOB_DX, cy + KNOB_DY, 0.170, _f(2 * KNOB_R), "over the shaft", ky0),
          _w(1.5, cx + KNOB_DX, cy + KNOB_DY, 0.150, _f(2 * KNOB_R), "steady", ky0),
          _w(1.5, cx + KNOB_DX, cy + KNOB_DY, rest - 0.0005 + TCP_OVER_KNOB, _f(2 * KNOB_R), "onto the shaft's end", ky0)]
    # screw it on anticlockwise (a left-hand thread): five strokes of 210 deg, sinking with the thread
    z = rest - 0.0005
    floor = KNOB_SEATED - 0.0005
    grip, loose = _f(2 * KNOB_R, 30.0), -0.030
    for k in range(5):
        yaws = [ky0, ky0 + 60, ky0 + 120, ky0 + 180, ky0 + STROKE_DEG]
        for a, b in zip(yaws, yaws[1:]):
            z = max(floor, z - PITCH * (b - a) / 360.0)
            R.append(_w((b - a) / 45.0, cx + KNOB_DX, cy + KNOB_DY, z + TCP_OVER_KNOB, grip, f"turn {k + 1}", b))
        R.append(_w(0.4, cx + KNOB_DX, cy + KNOB_DY, z + TCP_OVER_KNOB, loose, "let go", yaws[-1]))
        if k < 4:
            for b in (ky0 + 150, ky0 + 90, ky0 + 30, ky0):
                R.append(_w(0.5, cx + KNOB_DX, cy + KNOB_DY, z + TCP_OVER_KNOB, loose, "turn back", b))
            R.append(_w(0.5, cx + KNOB_DX, cy + KNOB_DY, z + TCP_OVER_KNOB, grip, "grip again", ky0))
    R += [_w(0.8, cx, cy, 0.170, loose, "up", ky0 + STROKE_DEG),
          _w(1.5, 0.22, -0.20, 0.20, FINGER_OPEN, "aside", 0.0)]
    t_knob = _clock(R) - 1.5
    # the left hand lets the blade go and moves aside
    _pad(L, t_knob)
    L += [_w(1.0, hold[0], hold[1], 0.17, FINGER_CLOSED, "up", PUSH_YAW),
          _w(1.5, 0.20, 0.28, 0.20, FINGER_OPEN, "aside", 0.0)]
    # Right hand: the front grille by its badge, carried in from the right, lowered onto the rear grille's rim
    R += [_w(1.5, gx, gy, 0.12, FINGER_OPEN, "over the front grille's badge", 0.0),
          _w(1.2, gx, gy, Z_SKIRT + 0.0005 + TCP_OVER_GRILLE, FINGER_OPEN, "down to the badge", 0.0),
          _w(0.8, gx, gy, Z_SKIRT + 0.0005 + TCP_OVER_GRILLE, _f(0.040), "grip it", 0.0),
          _w(1.5, gx, gy, 0.215, _f(0.040), "lift the grille", 0.0),
          _w(2.0, 0.23, -0.15, 0.22, _f(0.040), "over the knob", 0.0),
          _w(1.5, cx, cy - 0.04, 0.195, _f(0.040), "over the fan", 0.0),
          _w(1.0, cx, cy, 0.178, _f(0.040), "centred", 0.0),
          _w(2.0, cx, cy, Z_RIM + 0.001 + TCP_OVER_GRILLE, _f(0.040), "down onto the rim", 0.0),
          _w(0.6, cx, cy, Z_RIM - 0.0005 + TCP_OVER_GRILLE, _f(0.040), "seat it", 0.0),
          _w(0.6, cx, cy, Z_RIM - 0.0005 + TCP_OVER_GRILLE, FINGER_OPEN, "let go", 0.0),
          _w(0.8, cx, cy, 0.178, FINGER_OPEN, "up", 0.0)]
    t_grille = _clock(R)
    # both hands press the four clips shut, two each
    _pad(L, t_grille)
    L.append(_w(1.5, 0.20, 0.30, 0.20, FINGER_OPEN, "away", 0.0))
    R.append(_w(1.5, 0.20, -0.30, 0.20, FINGER_OPEN, "away", 0.0))
    end = max(_clock(L), _clock(R)) + 4.0
    _pad(L, end, "hands off")
    _pad(R, end, "hands off")
    return {"left": L, "right": R}
