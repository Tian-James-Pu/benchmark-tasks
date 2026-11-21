"""Give a classic Mr. Potato Head his eyes, nose, mouth and both ears, every peg pushed home to its collar, and leave him standing.

The toy (README "The product"): Hasbro's classic Mr. Potato Head (item F3244), a hollow polyethylene potato about 13 cm tall on its
base with feet, 120 g in all. Each face part has one round peg that goes into a round hole in the body. The pegs are an interference
fit: a part set in a hole stops at the hole's lip, and it goes home to its collar only under a straight push of several newtons, far
more than the light figure resists on its own (it slides at about 1 N and tips at well under 1 N when pushed at face height).

Model (README "Fidelity"; every contact is real collision, nothing is triggered by distance or state):
- Body ("body", a free articulation, 120 g with its links): the potato is a thin shell of convex patches (3 mm) over a 72 x 88 x 116 mm
  ellipsoid, open at seven holes. Behind each scored hole a rigid sleeve (10.8 mm bore) runs 16 mm in, and four sprung pads, each on a
  radial slider, stand 0.6 mm into the 10 mm peg's path. A peg going in squeezes all four; their spring load, through ordinary contact
  friction, is the interference fit: it is what a push has to overcome, and it holds a seated part. The feet are one block under it.
- Parts ("eyes", "nose", "mouth", "ear_a", "ear_b"; free rigid bodies, 6-10 g): a face piece, a 15 mm collar (14 mm on the ears) and a
  10 mm peg 16 mm long with a chamfered tip, as convex hulls.
- The scorer reads only poses (and the arms' fingertip points for the hands-off check). Its record of the figure falling over is
  cleared only at a reset (mechkit's on_reset).

Scoring: see `score()` and README "Scoring". Demonstration: `python demokit.py task.py OUT --mode full` (closed loop, demo_script).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

_plain_object = m._arena_object


def _arena_object(spec):
    """mechkit's spawn, with an articulation's reset also writing its joints back to their start (Arena's default restores the root only)."""
    obj = _plain_object(spec)
    if spec.role == "jointed":
        from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
        obj._pose_event_cfg.func = reset_articulation_pose_and_joints
    return obj


m._arena_object = _arena_object
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Give Mr. Potato Head his eyes, nose, mouth and both ears, each pushed all the way in, and keep him upright the whole time."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]
MM = 0.001

# -- the body ------------------------------------------------------------------------------------------------------------------------------
# Body frame: origin at the middle of the feet's underside, on the table; he faces -X (towards the arms), his right is +Y, up is +Z.
BODY_XY = (0.27, 0.0)
A_X, A_Y = 0.036, 0.044  # the potato's half-depth and half-width, at its widest
A_Z_HI, A_Z_LO = 0.082, 0.034  # its height above and below its widest girth: a potato standing on its broad end
Z_C = 0.050  # its widest girth above the table (its underside, 16 mm up, sits 4 mm down into the feet)


def a_z(z):
    return A_Z_HI if z >= Z_C else A_Z_LO
T_SHELL = 0.003  # shell thickness
FEET = (0.062, 0.072, 0.020)  # the base with feet: length (toe to heel), width, height
FEET_X = -0.007  # the toes reach 38 mm in front of the body's axis, the heels 24 mm behind
BODY_MASS = 0.080  # the shell and feet; the 20 pads add 40 g, for 120 g in all
MU = (0.3, 0.25)  # polyethylene body on ABS-type parts, static and dynamic (estimate)

R_PEG, L_PEG, CHAMFER = 0.0050, 0.016, 0.0015  # every part's peg: 10 mm across, 16 mm long, its tip chamfered 1.5 mm
R_BORE, R_SLEEVE = 0.0060, 0.0080  # the rigid sleeve behind each hole: 12 mm bore, 1 mm clear of the peg all round
S_SLEEVE = (0.002, 0.016)  # its depth behind the hole's centre on the surface
R_PAD = 0.0045  # each pad's face at rest: 0.5 mm into the peg's path
R_PAD_OUT = 0.0072
PAD_W = 0.004  # pad width (round the bore)
S_PAD = (0.002, 0.010, 0.014)  # pad: lead-in ramp from 2 to 10 mm deep (13 degrees), then its full face to 14 mm
R_RAMP = 0.0063  # the ramp's mouth stands outside the sleeve's bore, so a peg off centre cannot catch its edge
K_PAD = 4000.0  # N/m per pad: 3 N at 0.5 mm; four pads: about 3 N to keep a peg sliding, 3.6 N to start it, 5 N at the lead-in's end
PAD_TRAVEL = 0.003
PAD_MASS = 0.002


def surface_z_to_theta(z: float) -> float:
    return math.degrees(math.acos((z - Z_C) / a_z(z)))


# Holes: centre on the surface (body frame), outward axis, and the pad slider axes. Front holes go in along +X, the ears along -+Y.
Z_EYES, Z_NOSE, Z_MOUTH = 0.108, 0.081, 0.056
Z_EARS = Z_EYES


def _front(z):
    return np.array([-A_X * math.sqrt(1.0 - ((z - Z_C) / a_z(z)) ** 2), 0.0, z])


def _side(z, sign):
    return np.array([0.0, sign * A_Y * math.sqrt(1.0 - ((z - Z_C) / a_z(z)) ** 2), z])


HOLES = {
    "eyes": dict(c=_front(Z_EYES), u=np.array([-1.0, 0.0, 0.0]), pads=(("Z", 1), ("Z", -1), ("Y", 1), ("Y", -1)), collar=0.0085),
    "nose": dict(c=_front(Z_NOSE), u=np.array([-1.0, 0.0, 0.0]), pads=(("Z", 1), ("Z", -1), ("Y", 1), ("Y", -1)), collar=0.0085),
    "mouth": dict(c=_front(Z_MOUTH), u=np.array([-1.0, 0.0, 0.0]), pads=(("Z", 1), ("Z", -1), ("Y", 1), ("Y", -1)), collar=0.0085),
    "ear_right": dict(c=_side(Z_EARS, 1.0), u=np.array([0.0, 1.0, 0.0]), pads=(("Z", 1), ("Z", -1), ("X", 1), ("X", -1)), collar=0.0080),
    "ear_left": dict(c=_side(Z_EARS, -1.0), u=np.array([0.0, -1.0, 0.0]), pads=(("Z", 1), ("Z", -1), ("X", 1), ("X", -1)), collar=0.0080),
}
# Unscored holes, shown but not modelled: the hat's on top and the arms' low on each side.
SHOW_ONLY = {"hat": (np.array([0.0, 0.0, Z_C + A_Z_HI]), np.array([0.0, 0.0, 1.0])),
             "arm_right": (_side(0.060, 1.0), np.array([0.0, 1.0, 0.0])), "arm_left": (_side(0.060, -1.0), np.array([0.0, -1.0, 0.0]))}

# The shell's patches: a latitude-longitude grid on the ellipsoid, laid out so that each scored hole sits at the middle of a patch.
_TH_EYE, _TH_NOSE, _TH_MOUTH = surface_z_to_theta(Z_EYES), surface_z_to_theta(Z_NOSE), surface_z_to_theta(Z_MOUTH)
_H = 9.0
_TH = sorted([0.0, 16.0, 32.0, _TH_EYE - _H, _TH_EYE + _H, _TH_NOSE - _H, _TH_NOSE + _H, _TH_MOUTH - _H, _TH_MOUTH + _H, 120.0, 150.0, 180.0])
THETAS = [t for k, t in enumerate(_TH) if k == 0 or t - _TH[k - 1] > 0.5]
PHIS = [15.0 + 30.0 * k for k in range(13)]  # patch centres at 0, 30, ... 330 degrees (180: his front, 90: his right)


def _row(th):
    return next(i for i in range(len(THETAS) - 1) if THETAS[i] <= th < THETAS[i + 1])


HOLE_PATCHES = {(_row(_TH_EYE), 5): "eyes", (_row(_TH_NOSE), 5): "nose", (_row(_TH_MOUTH), 5): "mouth", (_row(_TH_EYE), 2): "ear_right",
                (_row(_TH_EYE), 8): "ear_left"}  # (theta row, phi column)


def ell_point(th_deg, ph_deg):
    th, ph = math.radians(th_deg), math.radians(ph_deg)
    az = A_Z_HI if th_deg <= 90.0 else A_Z_LO
    return np.array([A_X * math.sin(th) * math.cos(ph), A_Y * math.sin(th) * math.sin(ph), Z_C + az * math.cos(th)])


def ell_normal(p):
    n = np.array([p[0] / A_X ** 2, p[1] / A_Y ** 2, (p[2] - Z_C) / a_z(p[2]) ** 2])
    return n / np.linalg.norm(n)


def ell_hit(p0, d):
    """The outermost point where the line p0 + t d meets the potato (two half-ellipsoids joined at its widest girth), as t (None if
    it misses)."""
    best = None
    for az, upper in ((A_Z_HI, True), (A_Z_LO, False)):
        s = np.array([A_X, A_Y, az])
        q0 = (np.asarray(p0, float) - np.array([0.0, 0.0, Z_C])) / s
        q1 = np.asarray(d, float) / s
        a, b, c = q1 @ q1, 2 * q0 @ q1, q0 @ q0 - 1.0
        disc = b * b - 4 * a * c
        if disc < 0:
            continue
        for t in ((-b + math.sqrt(disc)) / (2 * a), (-b - math.sqrt(disc)) / (2 * a)):
            z = float(np.asarray(p0, float)[2] + t * np.asarray(d, float)[2])
            if (z >= Z_C) == upper and (best is None or t > best):
                best = t
    return best


def _frame(u):
    """Two unit vectors square to the axis u."""
    e1 = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = e1 - (e1 @ u) * u
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(u, e1)


def _depth_radial(p, h):
    """A point's depth into hole h (along the inward axis, from the hole's centre on the surface) and its distance from the axis."""
    d = np.asarray(p, float) - h["c"]
    s = -float(d @ h["u"])
    return s, float(np.linalg.norm(d + s * h["u"]))


def _clear_of_bores(p, keep=0.0064):
    for h in HOLES.values():
        s, r = _depth_radial(p, h)
        if -0.006 < s < 0.022 and r < keep:
            return False
    return True


def _bore_gap(pts):
    """The least distance of the points' hull from any bore's axis, sampled along every pair of points (inside the bore's depth)."""
    pts = np.asarray(pts, float)
    worst = 1.0
    for h in HOLES.values():
        for i in range(len(pts)):
            for f in np.linspace(0.0, 1.0, 7):
                q = pts[i] * (1 - f) + pts * f
                d = q - h["c"]
                sd = -(d @ h["u"])
                r = np.linalg.norm(d + sd[:, None] * h["u"], axis=1)
                inside = (sd > -0.004) & (sd < 0.020)
                if inside.any():
                    worst = min(worst, float(r[inside].min()))
    return worst


def _trim_clear(pts, keep=0.0062):
    """Drop the points nearest a bore until the hull of the rest stays clear of every bore."""
    pts = [np.asarray(p, float) for p in pts]
    while len(pts) >= 5 and _bore_gap(pts) < keep:
        near = [min(_depth_radial(p, h)[1] for h in HOLES.values()) for p in pts]
        pts.pop(int(np.argmin(near)))
    return pts


def _hull_mesh(points):
    """A Mesh of points for col_hull (the hull takes the points; faces are not used)."""
    pts = np.unique(np.round(np.asarray(points, float), 7), axis=0)
    return m.Mesh(pts, np.array([[0, 1, 2]]))


def shell_hulls():
    """(name, points) for every convex piece of the shell."""
    out = []
    for i in range(len(THETAS) - 1):
        for j in range(len(PHIS) - 1):
            if (i, j) in HOLE_PATCHES:
                continue
            pts = []
            for th in np.linspace(THETAS[i], THETAS[i + 1], 3):
                for ph in np.linspace(PHIS[j], PHIS[j + 1], 3):
                    p = ell_point(th, ph)
                    for q in (p, p - T_SHELL * ell_normal(p)):
                        if _clear_of_bores(q):
                            pts.append(q)
            pts = _trim_clear(pts)
            if len(np.unique(np.round(pts, 6), axis=0)) >= 5:
                out.append((f"sh_{i:02d}_{j:02d}", pts))
    # the patch round each scored hole: ten wedges from the bore's edge out past the patch's corners, cut along the hole's axis
    for name, h in HOLES.items():
        e1, e2 = _frame(h["u"])
        n_w = 10
        for k in range(n_w):
            pts = []
            for a in np.linspace(2 * math.pi * k / n_w, 2 * math.pi * (k + 1) / n_w, 3):
                for rho in (0.0065, 0.0088, 0.011, 0.013, 0.015):
                    p0 = h["c"] + rho * (math.cos(a) * e1 + math.sin(a) * e2)
                    t = ell_hit(p0, h["u"])
                    if t is None:
                        continue
                    p = p0 + t * h["u"]
                    pts += [p, p - T_SHELL * h["u"]]
            out.append((f"w_{name}_{k}", pts))
    return out


def sleeve_hulls():
    out = []
    for name, h in HOLES.items():
        e1, e2 = _frame(h["u"])
        for k in range(8):
            pts = []
            for a in np.linspace(2 * math.pi * k / 8, 2 * math.pi * (k + 1) / 8, 3):
                for r in (R_BORE, R_SLEEVE):
                    for s in S_SLEEVE:
                        pts.append(h["c"] - s * h["u"] + r * (math.cos(a) * e1 + math.sin(a) * e2))
            out.append((f"sl_{name}_{k}", pts))
    return out


def seat_depth(name):
    """How deep (m, along the inward axis from the hole's centre on the surface) the collar face stands when its rim meets the body:
    the most outward point of the surface under the collar's rim (negative: in front of the hole's centre)."""
    h = HOLES[name]
    e1, e2 = _frame(h["u"])
    best = 1.0
    for a in np.linspace(0, 2 * math.pi, 73):
        for rho in np.linspace(0.0066, h["collar"], 5):
            p0 = h["c"] + rho * (math.cos(a) * e1 + math.sin(a) * e2)
            t = ell_hit(p0, h["u"])
            if t is not None:
                best = min(best, -t)
    return best


SEAT = {name: seat_depth(name) for name in HOLES}


def _pad(name, hole, ax, sign):
    h = HOLES[hole]
    e = np.array({"X": [1.0, 0, 0], "Y": [0, 1.0, 0], "Z": [0, 0, 1.0]}[ax]) * sign
    t = np.cross(h["u"], e)
    a = -h["u"]
    prof = [(R_RAMP, S_PAD[0]), (R_PAD, S_PAD[1]), (R_PAD, S_PAD[2]), (R_PAD_OUT, S_PAD[2]), (R_PAD_OUT, S_PAD[0])]
    pts = np.array([h["c"] + s * a + r * e + w * t for r, s in prof for w in (-PAD_W / 2, PAD_W / 2)])
    lower, upper = (0.0, PAD_TRAVEL) if sign > 0 else (-PAD_TRAVEL, 0.0)
    return m.slider(name, ax, lower, upper, (), (m.col_hull(f"{name}_c", _hull_mesh(pts)),), at=tuple(float(v) for v in pts.mean(axis=0)), mass=PAD_MASS,
                    spring=K_PAD, damping=15.0, friction=0.0, effort=60.0, armature=0.002, inertia=(2e-8, 2e-8, 2e-8))


PADS = [(f"p_{hole}_{k}", hole, ax, sg) for hole in HOLES for k, (ax, sg) in enumerate(HOLES[hole]["pads"])]

LOOKS = {
    "potato": m.Look(color=m.rgb(0.70, 0.52, 0.32), roughness=0.75),
    "hole": m.Look(color=m.rgb(0.18, 0.11, 0.06), roughness=0.9),
    "shoe": m.Look(color=m.rgb(0.10, 0.22, 0.62), roughness=0.5),
    "sole": m.Look(color=m.rgb(0.92, 0.92, 0.90), roughness=0.6),
    "white": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.35),
    "iris": m.Look(color=m.rgb(0.15, 0.40, 0.85), roughness=0.3),
    "pupil": m.Look(color=m.rgb(0.03, 0.03, 0.03), roughness=0.3),
    "orange": m.Look(color=m.rgb(0.95, 0.45, 0.08), roughness=0.4),
    "red": m.Look(color=m.rgb(0.80, 0.07, 0.10), roughness=0.4),
    "pink": m.Look(color=m.rgb(0.95, 0.60, 0.65), roughness=0.45),
    "pink_in": m.Look(color=m.rgb(0.85, 0.42, 0.50), roughness=0.5),
    "peg": m.Look(color=m.rgb(0.85, 0.85, 0.82), roughness=0.5),
}


def _spot(c, u, r):
    """A dark disc lying on the surface round an axis: a hole's mouth, as seen."""
    e1, e2 = _frame(u)
    verts, n = [], 24
    t0 = ell_hit(c, u)
    verts.append(c + (t0 + 0.0004) * u)
    for k in range(n):
        a = 2 * math.pi * k / n
        p0 = c + r * (math.cos(a) * e1 + math.sin(a) * e2)
        t = ell_hit(p0, u)
        verts.append(p0 + ((t if t is not None else 0.0) + 0.0004) * u)
    f = [(0, 1 + k, 1 + (k + 1) % n) for k in range(n)]
    f += [(0, 1 + (k + 1) % n, 1 + k) for k in range(n)]
    return m.Mesh(np.array(verts), np.array(f))


def _body():
    ell = m.sphere(1.0, (0, 0, 0), rings=28, segments=48)
    v = ell.v * np.array([A_X, A_Y, 1.0])
    v[:, 2] = np.where(v[:, 2] >= 0, v[:, 2] * A_Z_HI, v[:, 2] * A_Z_LO) + Z_C
    parts = [("potato", m.Mesh(v, ell.f), "potato")]
    for name, h in HOLES.items():
        parts.append((f"hole_{name}", _spot(h["c"], h["u"], R_BORE + 0.0003), "hole"))
    for name, (c, u) in SHOW_ONLY.items():
        parts.append((f"hole_{name}", _spot(c, u, R_BORE + 0.0003), "hole"))
    for sy in (-1.0, 1.0):  # the base's two shoes, toes forward
        shoe = [(0.0, 0.0), (0.014, 0.0), (0.017, 0.006), (0.0175, 0.013), (0.015, 0.018), (0.0, 0.020)]
        toe = m.lathe([(r, z) for r, z in shoe], 24)
        parts.append((f"toe{int(sy > 0)}", m.Mesh(toe.v * np.array([1.25, 1.0, 1.0]) + np.array([-0.020, sy * 0.018, 0.0]), toe.f), "shoe"))
        parts.append((f"heel{int(sy > 0)}", m.box((0.040, 0.034, 0.018), (0.003, sy * 0.018, 0.009)), "shoe"))
        parts.append((f"sole{int(sy > 0)}", m.box((0.060, 0.034, 0.003), (FEET_X, sy * 0.018, 0.0015)), "sole"))
    cols = [m.col_hull(n, _hull_mesh(p)) for n, p in shell_hulls() + sleeve_hulls()]
    cols.append(m.col_box("feet", FEET, (FEET_X, 0.0, FEET[2] / 2)))
    links = [_pad(n, hole, ax, sg) for n, hole, ax, sg in PADS]
    return m.mechanism("body", (BODY_XY[0], BODY_XY[1], TOP), links, LOOKS, frame_parts=parts, frame_colliders=cols, friction=MU, solver_iters=48,
                       free_mass=BODY_MASS, frame_inertia=(2.0e-4, 1.8e-4, 1.3e-4))


# -- the parts: frame with the peg along +X, origin at the middle of the collar's face (the face that meets the body) ---------------------
def _peg_and_collar(r_collar, name):
    peg = m.lathe([(0.0, 0.0), (R_PEG, 0.0), (R_PEG, L_PEG - CHAMFER), (R_PEG - CHAMFER, L_PEG), (0.0, L_PEG)], 24).aimed("+X")
    collar = m.cylinder(r_collar, 0.0025, (-0.00125, 0.0, 0.0), axis="x", segments=32)
    parts = [(f"{name}_peg", peg, "peg"), (f"{name}_collar", collar, "peg")]
    cols = [m.col_hull(f"{name}_peg_c", peg), m.col_hull(f"{name}_collar_c", collar)]
    return parts, cols


def _rounded_rect(w, h, r, n=6):
    """A counter-clockwise outline (a, b): w by h, corners rounded to r, centred."""
    out = []
    for cx, cy, a0 in ((w / 2 - r, -h / 2 + r, -90), (w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180)):
        for a in np.linspace(a0, a0 + 90, n):
            out.append((cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a))))
    return out


def _prism_x(outline_yz, x0, x1, z_off=0.0):
    """A prism whose outline lies in y-z, from x0 to x1."""
    p = m.prism(outline_yz, x0, x1)
    v = p.v[:, [2, 0, 1]] + np.array([0.0, 0.0, z_off])
    return m.Mesh(v, p.f)


EYES_PLATE = (0.056, 0.024, 0.010)  # width, height, thickness
MOUTH_PLATE = (0.050, 0.022, 0.011)
NOSE_AX = (0.012, 0.0105)  # the nose's half-length along the peg, and its radius
EAR = (0.028, 0.038, 0.016)  # how far it stands out, its height, its thickness


def _eyes(pos, yaw):
    w, h, t = EYES_PLATE
    plate = _prism_x(_rounded_rect(w, h, 0.008), -0.002 - t, -0.002)
    parts, cols = _peg_and_collar(0.0085, "eyes")
    parts.append(("plate", plate, "white"))
    balls = []
    for k, sy in enumerate((-1.0, 1.0)):
        c = np.array([-0.002 - t + 0.003, sy * 0.0135, 0.002])
        ball = m.sphere(0.009, c, rings=8, segments=16)
        balls.append(ball)
        parts += [(f"ball{k}", ball, "white"), (f"iris{k}", m.sphere(0.0048, c + np.array([-0.0062, 0.0, 0.0]), rings=6, segments=12), "iris"),
                  (f"pupil{k}", m.sphere(0.0026, c + np.array([-0.0098, 0.0, 0.0]), rings=4, segments=10), "pupil")]
    cols.append(m.col_hull("eyes_c", m.join(plate, *balls)))
    return m.free("eyes", pos, parts, cols, LOOKS, mass=0.010, friction=MU, torsional_patch=0.02, solver_iters=32, yaw_deg=yaw)


def _mouth(pos, yaw):
    w, h, t = MOUTH_PLATE
    lips = _prism_x(_rounded_rect(w, h, 0.006), -0.002 - t, -0.002)
    parts, cols = _peg_and_collar(0.0085, "mouth")
    parts.append(("lips", lips, "red"))
    parts.append(("teeth", m.box((0.0015, 0.026, 0.006), (-0.002 - t - 0.0006, 0.0, 0.0015)), "white"))
    cols.append(m.col_hull("lips_c", lips))
    return m.free("mouth", pos, parts, cols, LOOKS, mass=0.009, friction=MU, torsional_patch=0.02, solver_iters=32, yaw_deg=yaw)


def _nose(pos, yaw, quat=None):
    a, r = NOSE_AX
    s = m.sphere(1.0, (0, 0, 0), rings=10, segments=20)
    bulb = m.Mesh(s.v * np.array([a, r, r]) + np.array([-0.0115, 0.0, 0.0]), s.f)
    parts, cols = _peg_and_collar(0.0085, "nose")
    parts.append(("bulb", bulb, "orange"))
    cols.append(m.col_hull("bulb_c", bulb))
    return m.free("nose", pos, parts, cols, LOOKS, mass=0.006, friction=MU, torsional_patch=0.02, solver_iters=32, yaw_deg=yaw, start_quat=quat)


def _ear(name, pos, yaw):
    d, hgt, t = EAR
    outline = [(-0.002 - d / 2 + a, b) for a, b in _rounded_rect(d, hgt, 0.008)]  # flat-edged along the peg, rounded corners
    ear = m.prism(outline, -t / 2, t / 2)
    inner = [(-0.002 - d / 2 - 0.002 + 0.008 * math.cos(a), 0.011 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 20, endpoint=False)]
    swirl = m.prism(inner, t / 2, t / 2 + 0.0006)
    parts, cols = _peg_and_collar(0.0080, name)
    parts += [("ear", ear, "pink"), ("swirl", swirl, "pink_in")]
    cols.append(m.col_hull(f"{name}_c", ear))
    return m.free(name, pos, parts, cols, LOOKS, mass=0.006, friction=MU, torsional_patch=0.02, solver_iters=32, yaw_deg=yaw)


# Starting layout (README "Setup"): the parts lie on the table to his front right and at his sides, as a child would tip them out.
EYES_START = (0.150, -0.095)  # stood on its flat lower edge, facing the arms
MOUTH_START = (0.150, -0.185)  # likewise
NOSE_START = (0.215, -0.150)  # lying on its side, peg pointing away from the arms
EAR_A_START = (0.300, -0.175)  # lying flat, peg pointing at his left side
EAR_B_START = (0.300, 0.175)  # lying flat, peg pointing at his right side

BODY = _body()
EYES = _eyes((EYES_START[0], EYES_START[1], TOP + EYES_PLATE[1] / 2 + 0.0005), 0.0)
MOUTH = _mouth((MOUTH_START[0], MOUTH_START[1], TOP + MOUTH_PLATE[1] / 2 + 0.0005), 0.0)
NOSE = _nose((NOSE_START[0], NOSE_START[1], TOP + NOSE_AX[1] + 0.002), 0.0)
EAR_A = _ear("ear_a", (EAR_A_START[0], EAR_A_START[1], TOP + EAR[2] / 2 + 0.0005), 90.0)
EAR_B = _ear("ear_b", (EAR_B_START[0], EAR_B_START[1], TOP + EAR[2] / 2 + 0.0005), -90.0)

# -- distractors (SimReady): a child's plastic cup, an apple, a marker -----------------------------------------------------------------------
CUP = SimReady("cup", pos=(0.50, 0.12, TOP), size=(0.064, 0.064, 0.100), mass=0.020, package="dishware_plastic_cup_a01", usd="sm_dishware_plastic_cup_a01_01.usd", convex=True)
APPLE = SimReady("apple", pos=(0.50, -0.14, TOP), size=(0.070, 0.067, 0.068), mass=0.18, package="food_produce_apple_b01", usd="sm_food_produce_apple_b01_01.usd", convex=True)
MARKER = SimReady("marker", pos=(0.46, 0.27, TOP), size=(0.109, 0.008, 0.007), mass=0.01, yaw=1.2, package="stationery_marker_thin_blue_a01",
                  usd="sm_stationery_marker_thin_blue_a01_01.usd", convex=True)

OBJECTS = [BODY, EYES, NOSE, MOUTH, EAR_A, EAR_B, CUP, APPLE, MARKER]
PART_HOLES = {"eyes": ("eyes",), "nose": ("nose",), "mouth": ("mouth",), "ear_a": ("ear_left", "ear_right"), "ear_b": ("ear_left", "ear_right")}
PARTS = tuple(PART_HOLES)

# -- the scorer ------------------------------------------------------------------------------------------------------------------------------
GAP_MAX = 0.001  # seated: the collar within 1 mm of the body (a feeler card)
OFF_MAX = 0.0015  # and its peg in that hole: within 1.5 mm of the hole's axis
ANG_MAX = 6.0  # and square to it within 6 degrees
UPRIGHT_MAX = 10.0  # standing: within 10 degrees of upright ...
FLOOR_MAX = 0.003  # ... on his feet, their underside within 3 mm of the table
FALL_DEG = 45.0  # fallen over (or laid down) at any time: tilted past 45 degrees
CLEAR = 0.03  # hands clear: every fingertip point at least 3 cm from the figure
STILL = 0.0005  # still: nothing moved more than 0.5 mm in a step


def part_in_hole(body, part, hole):
    """(gap, off-axis, angle) of a part in a hole: gap is how far the collar face stands out from where it meets the body (m)."""
    h = HOLES[hole]
    q = body.to_local(part.pos)[0]
    s, r = _depth_radial(q, h)
    ax = body.rotation().T @ part.rotation()[:, 0]
    ang = math.degrees(math.acos(float(np.clip(ax @ -h["u"], -1.0, 1.0))))
    return float(SEAT[hole] - s), float(r), float(ang)


def seated(body, parts) -> dict:
    """For each part, the hole it is seated in, or None. Each hole takes one part."""
    out, used = {}, set()
    for name in PARTS:
        out[name] = None
        for hole in PART_HOLES[name]:
            if hole in used:
                continue
            gap, off, ang = part_in_hole(body, parts[name], hole)
            if -0.0015 <= gap <= GAP_MAX and off <= OFF_MAX and ang <= ANG_MAX:
                out[name] = hole
                used.add(hole)
                break
    return out


def _figure_points(body, parts):
    """Points over the figure's surface, world frame: the body's ellipsoid and feet, and each part's face piece."""
    pts = [ell_point(th, ph) for th in np.linspace(5, 175, 15) for ph in np.linspace(0, 345, 24)]
    pts += [np.array([FEET_X + dx * FEET[0] / 2, dy * FEET[1] / 2, FEET[2]]) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    out = [body.to_world(np.array(pts))]
    local = np.array([[x, y, z] for x in (-0.002, -0.012, -0.024) for y in (-0.025, 0.0, 0.025) for z in (-0.012, 0.0, 0.012)])
    for name in PARTS:
        out.append(parts[name].to_world(local))
    return np.vstack(out)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place a recorded harm is cleared
        self.fell = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.prev = None
        self.last = {}

    def evaluate(self, body, parts, tips=None):
        tilt = body.tilt_deg()
        standing = tilt <= UPRIGHT_MAX and abs(float(body.pos[2]) - TOP) <= FLOOR_MAX
        where = seated(body, parts)
        n = sum(v is not None for v in where.values())
        c = {"tilt": tilt, "standing": standing, "seated": where, "n": n, "floor": float(body.pos[2]) - TOP}
        if tips is not None:
            fp = _figure_points(body, parts)
            c["hand_gap"] = float(np.min(np.linalg.norm(tips[:, None, :] - fp[None, :, :], axis=2)))
        return c

    def step(self, env):
        body = m.state(env, "body")
        parts = {n: m.state(env, n) for n in PARTS}
        c = self.evaluate(body, parts, m.fingertips(env))
        now = np.concatenate([body.pos] + [parts[n].pos for n in PARTS])
        c["still"] = bool(self.prev is not None and np.max(np.abs(now - self.prev)) < STILL)
        self.prev = now
        if c["tilt"] > FALL_DEG:
            self.fell = True
        ok = (not self.fell) and c["standing"] and c["n"] == 5 and c["still"] and c["hand_gap"] >= CLEAR
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= HOLD_STEPS:
            self.full = True
        if self.full and (self.fell or not c["standing"] or c["n"] < 5):  # credit does not outlast the state that earned it
            self.full = False
        if self.fell or not c["standing"]:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["n"] >= 3:
            self.score = 0.5
        elif c["n"] >= 1:
            self.score = 0.25
        else:
            self.score = 0.0
        c["fell"] = self.fell
        self.last = c


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration -----------------------------------------------------------------------------------------------------------------------
DISTRACTORS = {"cup": CUP, "apple": APPLE, "marker": MARKER}
# where each part is gripped (part frame): far enough out along the peg that the fingers (36 mm wide along it) stay clear of the body
GRIP_LOCAL = {"eyes": (-0.020, 0.0, 0.0), "mouth": (-0.020, 0.0, 0.0), "nose": (-0.017, 0.0, 0.0), "ear_a": (-0.018, 0.0, 0.0), "ear_b": (-0.018, 0.0, 0.0)}


def _report(ctx, label=""):
    c = TRIAL.last
    if not c:
        return
    body = ctx.state("body")
    parts = {n: ctx.state(n) for n in PARTS}
    gaps = []
    for n in PARTS:
        best = min((part_in_hole(body, parts[n], h) for h in PART_HOLES[n]), key=lambda g: abs(g[0]) + g[1])
        gaps.append(f"{n} {best[0] * 1000:+.1f}/{best[1] * 1000:.1f}/{best[2]:.0f}")
    print(f"[potato] t {ctx.t:5.1f} {label} tilt {c['tilt']:4.1f} floor {c['floor'] * 1000:+.1f} body {np.round((body.pos[:2] - np.array(BODY_XY)) * 1000, 1).tolist()} | "
          f"{' '.join(gaps)} | n {c['n']} hands {c.get('hand_gap', 0) * 1000:.0f} still {c.get('still')} fell {c.get('fell')} | score {TRIAL.score:.2f} full {TRIAL.full}", flush=True)
    if label.startswith(("end", "clear")):
        print("[distractors] " + ", ".join(f"{n} moved {np.round((ctx.pos(n)[:2] - np.array(o.pos[:2])) * 1000, 1).tolist()} mm" for n, o in DISTRACTORS.items()), flush=True)


def demo_script(ctx):
    """Closed loop, reading only poses. For each front part the right arm picks it from the table (top-down, fingers on its ends),
    brings its peg square to the hole and pushes it straight in until its collar meets the body, while the left arm's closed fingers
    brace the body from behind at the same height; then it lets go and backs off. For his left ear the right arm pushes along +y and the
    left braces his right side; for his right ear the arms swap, the right bracing on the seated left ear. Then both arms move clear.
    Red-team modes: none (nothing), onearm (no brace), started (each part set into its hole's mouth and let go, not pushed home),
    careless (each part gripped and pushed with the hand turned 12 degrees, so its peg goes in at an angle), hold (as full, but the left
    hand stays on his right ear at the end)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "started", "careless", "hold"):
        raise ValueError(mode)
    l, r = ctx.left, ctx.right
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for _ in range(120):
            yield
        _report(ctx, "end:")
        return
    up = np.array([0.0, 0.0, 1.0])

    def body_state():
        return ctx.state("body")

    def hole_world(hole):
        b = body_state()
        h = HOLES[hole]
        return b.to_world(h["c"][None])[0], b.rotation() @ h["u"]

    def collar_point(part):
        return ctx.pos(part)

    def tilt_of(part):
        """How far the part's peg points below level (deg), and its heading (deg from +x)."""
        ax = ctx.state(part).rotation()[:, 0]
        return math.degrees(math.asin(float(np.clip(-ax[2], -1, 1)))), math.degrees(math.atan2(ax[1], ax[0]))

    def pick(arm, part, grip, width, force=25.0):
        """Top-down (or tilted to the part's own lie) over the part's middle, fingers 4 mm off the table, close, lift."""
        mid = ctx.state(part).to_world(np.array([GRIP_LOCAL[part]]))[0]
        top = np.r_[mid[:2], max(mid[2] + 0.004, TOP + 0.016)]
        yield from arm.move(top + up * 0.08, grip=grip, finger=FINGER_OPEN, t=2.0, label=f"over the {part}")
        yield from arm.servo(top + up * 0.08, timeout=1.0, label=f"over the {part}")
        yield from arm.move(top, t=1.0, label=f"down to the {part}")
        yield from arm.servo(top, timeout=1.0, label=f"down to the {part}")
        yield from arm.close(width, force, t=0.5, label=f"grip the {part}")
        yield from arm.move(arm.at() + up * 0.06, t=0.8, label=f"lift the {part}")

    def brace(arm, hole, spread=0.030, gap=0.021, dz=0.013):
        """A fork behind the body, opposite a hole: the gripper open, its two fingers either side of the hole's axis (spread from it)
        and just off the body, so that the push seats the body against both fingers and it can neither slide nor turn away."""
        h = HOLES[hole]
        zc = (h["c"][2] - Z_C) / a_z(h["c"][2])
        if abs(h["u"][0]) > 0.5:  # a front hole: fingers either side in y, behind his back
            far = math.sqrt(max(1.0 - ((spread - 0.006) / A_Y) ** 2 - zc ** 2, 0.0)) * A_X  # at the fingers' inner faces
            local = np.array([far + gap, 0.0, h["c"][2] + dz])
            grip = d.down(0.0)
        else:  # an ear: fingers either side in x, at his other side
            far = math.sqrt(max(1.0 - ((spread - 0.006) / A_X) ** 2 - zc ** 2, 0.0)) * A_Y
            local = np.array([0.0, -np.sign(h["u"][1]) * (far + gap), h["c"][2] + dz])
            grip = d.down(90.0 if arm.side == "right" else -90.0)
        p = body_state().to_world(local[None])[0]
        _, u = hole_world(hole)
        width = 2 * spread - 0.010
        yield from arm.move(p - u * 0.04 + up * 0.05, grip=grip, finger=-width / 2, t=1.8, label=f"brace for the {hole}: over")
        yield from arm.move(p - u * 0.03, t=0.8, label=f"brace for the {hole}: down")
        yield from arm.move(p, t=0.8, label=f"brace for the {hole}: on")
        if os.environ.get("POTATO_TIPS"):
            from yam_arena import ik
            tips = m.fingertips(ctx.emb._env)
            k0 = 0 if arm.side == "left" else 12
            tcp, rot = ik.tcp_pose(ctx.observed(arm.side)[:6], arm.base)
            print(f"[tips] {arm.side} {hole} tcp {np.round(tcp * 1000, 1).tolist()} body-local tips:",
                  np.round(body_state().to_local(tips[k0:k0 + 12]) * 1000, 1).tolist(), flush=True)

    def depth_now(part, hole):
        b = body_state()
        return _depth_radial(b.to_local(ctx.pos(part)[None])[0], HOLES[hole])[0]

    def push_in(arm, part, hole, s_goal, holder=None, speed=0.015, lead=0.045, timeout=6.0):
        """Push the held part straight in along the hole's axis (a line fixed in the world when the push starts) until its collar face
        reaches depth s_goal in the hole, the command running at most `lead` ahead of where the part is: the arm is soft, and that lead
        is what pushes. The bracing arm (`holder`), if any, follows the body back to where it was, so that the soft brace does not let
        him be pushed away. Then both commands ease back to where things are, so letting go does not lunge."""
        from yam_arena import ik
        c0, u0 = hole_world(hole)
        t0 = ik.tcp_pose(arm.q, arm.base)[0]
        h0 = ik.tcp_pose(holder.q, holder.base)[0] if holder is not None else None
        b0 = body_state().pos.copy()
        p0 = collar_point(part)
        n_ok, d_cmd, shift = 0, 0.0, 0.0
        for k in range(round(timeout * 20)):
            moved = float((collar_point(part) - p0) @ -u0)
            d_cmd = min(speed * (k + 1) * 0.05, moved + lead)
            arm._solve(t0 - u0 * d_cmd, arm.grip.rot)
            if holder is not None:
                away = float((body_state().pos - b0) @ -u0)  # how far he has been pushed along the push
                shift = float(np.clip(6.0 * away, 0.0, 0.07))  # a stiff virtual spring: it pushes back only as far as he is pushed
                holder._solve(h0 + u0 * shift, holder.grip.rot)
            yield
            n_ok = n_ok + 1 if depth_now(part, hole) >= s_goal - 0.0003 else 0
            if n_ok >= 6:
                break
        moved = float((collar_point(part) - p0) @ -u0)
        away = float((body_state().pos - b0) @ -u0)
        for j in range(8):
            f = (j + 1) / 8
            arm._solve(t0 - u0 * (d_cmd + (moved - d_cmd) * f), arm.grip.rot)
            if holder is not None:
                holder._solve(h0 + u0 * (shift + (-away - shift) * f), holder.grip.rot)
            yield
        print(f"[push] {part}: depth {depth_now(part, hole) * 1000:+.1f} mm (goal {s_goal * 1000:+.1f}) after {k + 1} steps, part moved {moved * 1000:.1f} mm, "
              f"body moved {away * 1000:.1f} mm, brace shift {shift * 1000:.1f} mm", flush=True)

    def insert(arm, part, hole, push=True, holder=None):
        c, u = hole_world(hole)
        start = c + u * (L_PEG + 0.008)
        yield from arm.move(arm.at() + (start - collar_point(part)) + up * 0.03, t=2.0, label=f"{part} to the {hole}")
        yield from arm.servo(start + up * 0.03, read=lambda: collar_point(part), tol=0.002, timeout=1.5, label=f"{part} over the {hole}")
        yield from arm.servo(start, read=lambda: collar_point(part), tol=0.001, timeout=2.0, label=f"{part} at the {hole}")
        _report(ctx, f"{part} at the {hole}:")
        # straight in: to the collar's seat, or (red team "started") only until the peg's tip is in the pads' lead-in
        yield from push_in(arm, part, hole, SEAT[hole] if push else 0.005 - L_PEG, holder=holder)
        _report(ctx, f"{part} pushed:")

    def release(arm, hole):
        c, u = hole_world(hole)
        yield from arm.open(t=0.4, label="let go")
        yield from arm.move(arm.at() + u * 0.04, t=0.8, label="back off")
        yield from arm.move(arm.at() + up * 0.06, t=0.8, label="up")

    brace_on = mode != "onearm"
    grip_r = d.down(12.0 if mode == "careless" else 0.0)  # fingers close along y
    grip_l = d.down(0.0)
    jobs = [("mouth", "mouth", MOUTH_PLATE[0]), ("nose", "nose", 2 * NOSE_AX[1]), ("eyes", "eyes", EYES_PLATE[0])]
    for part, hole, width in jobs:
        if brace_on:
            yield from ctx.together(pick(r, part, grip_r, width), brace(l, hole))
        else:
            yield from pick(r, part, grip_r, width)
        if part == "nose":  # it lay with its peg tipped down: turn the hand until the peg is level (read twice)
            for _ in range(2):
                tilt, head = tilt_of(part)
                print(f"[nose] peg {tilt:+.1f} deg below level, heading {head:+.1f}", flush=True)
                if abs(tilt) > 1.0:
                    yield from r.turn(-tilt, axis=(0.0, 1.0, 0.0), t=max(0.5, abs(tilt) / 20.0), label="level the nose")
        yield from insert(r, part, hole, push=mode != "started", holder=l if brace_on else None)
        yield from release(r, hole)
    if brace_on:
        yield from l.move(l.at() + up * 0.08, t=1.0, label="left up")
    # his left ear (at -y): the right arm pushes along +y, the left braces his right side
    ear_grip_r = d.down(102.0 if mode == "careless" else 90.0)
    ear_grip_l = d.down(-102.0 if mode == "careless" else -90.0)
    if brace_on:
        yield from ctx.together(pick(r, "ear_a", ear_grip_r, EAR[1]), brace(l, "ear_left", spread=0.022))
    else:
        yield from pick(r, "ear_a", ear_grip_r, EAR[1])
    yield from insert(r, "ear_a", "ear_left", push=mode != "started", holder=l if brace_on else None)
    yield from release(r, "ear_left")
    if brace_on:
        yield from l.move(l.at() + up * 0.08, t=1.0, label="left up")
        yield from ctx.together(pick(l, "ear_b", ear_grip_l, EAR[1]), brace(r, "ear_right", spread=0.028))
    else:
        yield from r.move(np.array([0.20, -0.25, TOP + 0.20]), t=1.5, label="right away")
        yield from pick(l, "ear_b", ear_grip_l, EAR[1])
    yield from insert(l, "ear_b", "ear_right", push=mode != "started", holder=r if brace_on else None)
    if mode == "hold":
        yield from l.open(t=0.4, label="let go")
        yield from l.close(t=0.4, label="rest the hand on the ear")
        _report(ctx, "hand left on:")
        for k in range(80):
            yield
        _report(ctx, "end:")
        return
    yield from release(l, "ear_right")
    yield from ctx.together(r.move(np.array([0.20, -0.28, TOP + 0.22]), grip=d.down(0.0), t=1.5, label="right clear"),
                            l.move(np.array([0.20, 0.28, TOP + 0.22]), grip=d.down(0.0), t=1.5, label="left clear"))
    _report(ctx, "clear:")
    for k in range(60):
        yield
        if k % 10 == 9:
            _report(ctx, "end:")


if __name__ == "__main__":
    # offline checks: the shell keeps clear of every bore, and where each collar seats
    bad = 0
    for name, pts in shell_hulls():
        pts = np.asarray(pts)
        for i in range(len(pts)):
            for j in range(i, len(pts)):
                for f in np.linspace(0, 1, 6):
                    q = pts[i] * (1 - f) + pts[j] * f
                    for hn, h in HOLES.items():
                        s, rr = _depth_radial(q, h)
                        if -0.004 < s < 0.020 and rr < R_BORE:
                            bad += 1
                            if bad < 10:
                                print("bore hit", name, hn, round(s * 1000, 2), round(rr * 1000, 2))
    print("shell hulls", len(shell_hulls()), "sleeves", len(sleeve_hulls()), "bore hits", bad)
    for n in HOLES:
        print(n, "centre", np.round(HOLES[n]["c"] * 1000, 1), "seat depth mm", round(SEAT[n] * 1000, 2))
