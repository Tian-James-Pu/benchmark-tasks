"""Put a five-piece matryoshka back together: the third doll into the second, the second closed and into the first, the first closed.

The set (README "The product"): a five-piece turned-wood nesting doll set, the largest doll 150 mm tall and 71 mm across. Each doll
splits at the waist. The bottom half carries a spigot (a short turned collar) above its shoulder, and the top half's bore is turned to a
snug friction fit over it: a top dropped on its spigot stops a few millimetres up, and it seats down to the shoulder only when it is
pressed (or pressed and twisted) on, square. A seated top holds well enough to lift the whole closed doll by its head.

Model (README "Fidelity"):
- Each bottom half (dolls 1 and 2) is a free articulation: the turned shell is the frame, and four small hidden pads sit in slots round
  its spigot, each on a radial slider with a spring. At rest a pad stands 0.6 mm proud of the top's bore, so a top going over the spigot
  squeezes all four in: that radial load, through ordinary contact friction, is the fit's grip. It resists sliding and twisting alike, and
  it is what a press has to overcome. Nothing is triggered by distance or state: seating is contact and friction only.
- Each top half is a free rigid body: its shell, bore and crown as convex pieces.
- The third doll is closed (dolls 4 and 5 inside it) and is one free rigid body with their combined mass.
- The scorer reads only poses (and fingertip points for the hands-off check); its record of a piece falling off the table is cleared only
  by a reset (on_reset), never by the scene looking right again.
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
INSTRUCTION = "Put the nesting dolls back together, each one closed and facing the same way as its body, with the smaller ones inside the largest."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz, everything assembled, still and hands clear
TOP = TABLE_TOP[SCENE]
MM = 0.001

# -- the doll's outline (the largest doll, 150 mm by 71 mm): (height, radius) in mm, foot to crown. The smaller dolls are this outline
#    scaled. A classic Semenov-style outline: a flat foot, a full belly, the waist split at 62 mm, shoulders, a neck and a round head.
OUTLINE = [(0, 25.5), (1.5, 27.5), (5, 30.0), (12, 33.0), (22, 35.0), (35, 35.5), (48, 35.0), (62, 33.5), (72, 32.3), (82, 30.5), (90, 28.0),
           (96, 24.5), (101, 21.5), (105, 20.6), (110, 21.2), (118, 22.3), (124, 22.4), (131, 21.2), (138, 18.2), (143, 14.5), (147, 9.5),
           (149.3, 4.5), (150, 0.0)]
_OZ = np.array([p[0] for p in OUTLINE], float)
_OR = np.array([p[1] for p in OUTLINE], float)
SPLIT = 62.0  # mm, the waist split of the largest doll
HEAD_EQ = 124.0  # mm, the head's widest point
NECK = 105.0

# per doll: scale of the outline, wall, floor and crown thickness, spigot height and wall, bore clearance over the spigot, pad interference,
# pad spring (N/m) and pad width. Doll 1 and 2 halves are open; doll 3 is closed (dolls 4 and 5 inside).
DOLLS = {
    1: dict(s=1.0, wall=3.5, floor=5.0, crown=5.0, sp_h=10.0, sp_w=2.5, clear=1.0, interf=0.6, k=6700.0, pad_w=8.0),
    2: dict(s=0.690, wall=3.0, floor=4.0, crown=4.0, sp_h=8.0, sp_w=2.2, clear=1.0, interf=0.5, k=5000.0, pad_w=6.0),
    3: dict(s=0.420),
}
RHO = 500.0  # kg/m3: turned linden (lime) with its lacquer (linden 420-530 kg/m3)
MU = (0.5, 0.4)  # lacquered wood on lacquered wood, static and dynamic (estimate)
N_WALL, N_FIT = 16, 24  # sectors round a shell, and round the spigot and bore


def outer_r(z_mm, s=1.0):
    """The outline's radius (mm) at height z_mm of a doll scaled by s."""
    return s * float(np.interp(z_mm / s, _OZ, _OR))


def dims(k):
    """Doll k's dimensions in mm (split height, bore, spigot radii, pad radius, floor, total height)."""
    d = DOLLS[k]
    s = d["s"]
    zs = SPLIT * s
    rw = outer_r(zs, s)  # outer radius at the waist
    bore = rw - d["wall"]
    sp_o = bore - d["clear"]
    sp_i = sp_o - d["sp_w"]
    return dict(s=s, zs=zs, rw=rw, bore=bore, sp_o=sp_o, sp_i=sp_i, pad=bore + d["interf"], floor=d["floor"], H=150.0 * s, sp_h=d["sp_h"],
                wall=d["wall"], crown=d["crown"], k=d["k"], pad_w=d["pad_w"])


# -- looks --------------------------------------------------------------------------------------------------------------------------------

LOOKS = {
    "red": m.Look(color=m.rgb(0.72, 0.07, 0.06), roughness=0.25),  # lacquered red
    "gold": m.Look(color=m.rgb(0.93, 0.70, 0.15), roughness=0.3),
    "cream": m.Look(color=m.rgb(0.97, 0.90, 0.78), roughness=0.3),
    "skin": m.Look(color=m.rgb(0.98, 0.84, 0.72), roughness=0.35),
    "dark": m.Look(color=m.rgb(0.08, 0.06, 0.05), roughness=0.4),
    "rose": m.Look(color=m.rgb(0.90, 0.35, 0.35), roughness=0.4),
    "green": m.Look(color=m.rgb(0.15, 0.50, 0.20), roughness=0.4),
    "blue": m.Look(color=m.rgb(0.15, 0.30, 0.70), roughness=0.4),
    "wood": m.Look(color=m.rgb(0.86, 0.74, 0.55), roughness=0.75),  # bare linden inside
}


def two_sided(mesh: m.Mesh) -> m.Mesh:
    return m.Mesh(mesh.v, np.vstack([mesh.f, mesh.f[:, ::-1]]))


def lathe_mm(profile, segments=48):
    return two_sided(m.lathe([(r * MM, z * MM) for r, z in profile], segments))


def _zs(z0, z1, n):
    return list(np.linspace(z0, z1, n))


# -- shells as convex pieces --------------------------------------------------------------------------------------------------------------


def _sector_hulls(prefix, z_breaks, r_out, r_in, n, z_shift=0.0, phase=0.0):
    """Collider hulls of a turned shell between r_in(z) and r_out(z) (mm), in tiers between z_breaks and n sectors round."""
    out = []
    for t, (z0, z1) in enumerate(zip(z_breaks[:-1], z_breaks[1:])):
        zz = _zs(z0, z1, 3)
        for k in range(n):
            pts = []
            for a in (phase + 2 * math.pi * k / n, phase + 2 * math.pi * (k + 1) / n):
                c, s_ = math.cos(a), math.sin(a)
                for z in zz:
                    for r in (r_out(z), max(r_in(z), 0.5)):
                        pts.append((r * c * MM, r * s_ * MM, (z - z_shift) * MM))
            out.append(m.col_hull(f"{prefix}{t}_{k}", m.Mesh(np.array(pts), np.array([[0, 1, 2]]))))
    return out


def _solid_hull(name, z0, z1, r_of, z_shift=0.0, n=24):
    pts = []
    for z in _zs(z0, z1, 6):
        r = max(r_of(z), 0.3)
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append((r * math.cos(a) * MM, r * math.sin(a) * MM, (z - z_shift) * MM))
    return m.col_hull(name, m.Mesh(np.array(pts), np.array([[0, 1, 2]])))


def _bottom_profile(d):
    """Closed (r, z) outline of a bottom half in mm: outside up, shoulder, spigot, inside down."""
    s, zs = d["s"], d["zs"]
    prof = [(0.0, 0.0)]
    prof += [(outer_r(z, s), z) for z in _OZ * s if z <= zs] + [(d["rw"], zs)]
    top = zs + d["sp_h"]
    prof += [(d["sp_o"], zs), (d["sp_o"], top - 1.0), (d["sp_o"] - 1.0, top), (d["sp_i"] + 0.6, top), (d["sp_i"], top - 0.6), (d["sp_i"], zs - 4.0)]
    inner = [(outer_r(z, s) - d["wall"], z) for z in _OZ * s if d["floor"] + 2 < z < zs - 6][::-1]
    prof += [(max(min(r, 99.0), d["sp_i"]), z) for r, z in inner]
    prof += [(outer_r(d["floor"], s) - d["wall"] - 1.5, d["floor"] + 1.0), (outer_r(d["floor"], s) - d["wall"] - 3.0, d["floor"]), (0.0, d["floor"])]
    return prof


def _top_profile(d):
    """Closed (r, z) outline of a top half in mm, z from its rim: outside up to the crown, inside back down to the bore."""
    s, zs, H = d["s"], d["zs"], d["H"]
    prof = [(d["bore"] + 0.8, 0.0), (d["rw"], 0.0)]
    prof += [(outer_r(z, s), z - zs) for z in _OZ * s if zs < z < H] + [(0.0, H - zs)]
    inner = []
    for z in _OZ[::-1] * s:
        if zs + d["sp_h"] + 1.0 < z < H - d["crown"]:
            r = outer_r(z, s) - d["wall"]
            if r > 1.0:
                inner.append((r, z - zs))
    prof = prof + [(0.0, H - d["crown"] - zs)] + inner + [(d["bore"], d["sp_h"] + 0.5), (d["bore"], 1.5)]
    return prof


def _decor_bottom(d):
    """The apron on the bottom half's front (+x): a cream panel with a flower, visual only."""
    s, zs = d["s"], d["zs"]
    parts = []
    pts = []
    for z in _zs(3 * s, zs - 3 * s, 7):
        r = outer_r(z, s) + 0.3
        for a in np.radians(np.linspace(-38, 38, 9)):
            pts.append((r * math.cos(a), r * math.sin(a), z))
    pts = np.array(pts)
    v = pts * MM
    n = 9
    f = []
    for i in range(6):
        for j in range(n - 1):
            a, b, c, e = i * n + j, i * n + j + 1, (i + 1) * n + j + 1, (i + 1) * n + j
            f += [(a, b, c), (a, c, e)]
    parts.append(("apron", two_sided(m.Mesh(v, np.array(f))), "cream"))
    zc = zs * 0.5
    rc = outer_r(zc, s) + 0.8
    for i, (dy, dz, look, rr) in enumerate([(0, 0, "rose", 5.0), (-7, -7, "green", 3.0), (7, -7, "green", 3.0), (0, 0, "gold", 2.0)]):
        sc = s / 0.85 if s < 0.9 else 1.0
        parts.append((f"flower{i}", m.sphere(rr * s * MM * sc, ((rc + (0.6 if look == "gold" else 0.0)) * MM, dy * s * MM, (zc + dz * s) * MM), rings=4, segments=10), look))
    return parts


def _decor_top(d):
    """The face on the top's head (front, +x), a gold shawl band, visual only. z from the rim."""
    s, zs = d["s"], d["zs"]
    zf = HEAD_EQ * s - zs
    rh = outer_r(HEAD_EQ * s, s)
    parts = []
    # face: a flattened ellipsoid on the head's front
    face = m.sphere(1.0, (0, 0, 0), rings=8, segments=16)
    fv = face.v * np.array([0.25 * rh, 0.62 * rh, 0.72 * rh]) * MM + np.array([(rh * 0.82) * MM, 0, zf * MM])
    parts.append(("face", m.Mesh(fv, face.f), "skin"))
    for i, dy in enumerate((-0.25, 0.25)):
        parts.append((f"eye{i}", m.sphere(0.075 * rh * MM, ((rh * 1.06) * MM, dy * rh * MM, (zf + 0.12 * rh) * MM), rings=4, segments=8), "dark"))
        parts.append((f"cheek{i}", m.sphere(0.11 * rh * MM, ((rh * 1.02) * MM, dy * 1.35 * rh * MM, (zf - 0.25 * rh) * MM), rings=4, segments=8), "rose"))
    parts.append(("mouth", m.sphere(0.07 * rh * MM, ((rh * 1.05) * MM, 0, (zf - 0.42 * rh) * MM), rings=4, segments=8), "rose"))
    # shawl: a gold band round the shoulders, and a gold kerchief rim round the face
    band = [(outer_r(z, s) + 0.35, z - zs) for z in _zs(82 * s, 96 * s, 5)]
    parts.append(("shawl", lathe_mm([(band[0][0] - 0.3, band[0][1])] + band + [(band[-1][0] - 0.3, band[-1][1])], 48), "gold"))
    return parts


def _bottom(name, k, pos, yaw):
    d = dims(k)
    s, zs = d["s"], d["zs"]
    mass_s, inertia = _shell_mass_inertia(_bottom_profile(d))
    look_parts = [("shell", lathe_mm(_bottom_profile(d)), "red")]
    look_parts += _decor_bottom(d)
    # inside face (bare wood): a slightly smaller copy of the inside, seen through the opening
    inner = [(max(outer_r(z, s) - d["wall"] - 0.2, d["sp_i"] - 0.2), z) for z in np.linspace(d["floor"] + 0.2, zs + d["sp_h"] - 0.8, 10)]
    look_parts.append(("inside", lathe_mm([(0.0, d["floor"] + 0.2)] + inner), "wood"))
    look_parts.append(("spigot", lathe_mm([(d["sp_i"] + 0.1, zs + 0.2), (d["sp_o"] + 0.1, zs + 0.2), (d["sp_o"] + 0.1, zs + d["sp_h"] - 1.0),
                                            (d["sp_o"] - 0.9, zs + d["sp_h"] + 0.1), (d["sp_i"] + 0.6, zs + d["sp_h"] + 0.1)]), "wood"))
    top = zs + d["sp_h"]
    cols = [_solid_hull(f"{name}_floor", 0.0, d["floor"], lambda z: outer_r(z, s))]
    zb = [d["floor"] - 0.5] + [z for z in (22 * s, 48 * s) if d["floor"] < z < zs] + [zs]
    cols += _sector_hulls(f"{name}_w", zb, lambda z: outer_r(z, s), lambda z: max(outer_r(z, s) - d["wall"], d["sp_i"]) if z < zs - 6 else d["sp_i"], N_WALL)
    cols += _sector_hulls(f"{name}_sp", [zs - 0.5, top - 1.0], lambda z: d["sp_o"], lambda z: d["sp_i"], N_FIT)
    cols += _sector_hulls(f"{name}_spt", [top - 1.0, top], lambda z: d["sp_o"] - (z - (top - 1.0)), lambda z: d["sp_i"] + 0.6 * (z - (top - 1.0)), N_FIT)
    links = []
    for i, (ax, sign) in enumerate((("X", 1), ("Y", 1), ("X", -1), ("Y", -1))):
        links.append(_pad(f"pad{i}", ax, sign, d))
    return m.mechanism(name, pos, links, LOOKS, frame_parts=look_parts, frame_colliders=cols, yaw_deg=yaw, friction=MU, free_mass=mass_s,
                       frame_inertia=inertia, solver_iters=32)


def _pad(name, ax, sign, d):
    """One spring pad in a slot of the spigot, on a radial slider: at rest its face stands `interf` proud of the top's bore."""
    zs, top = d["zs"], d["zs"] + d["sp_h"]
    w = d["pad_w"] / 2
    z_lo, z_hi = zs + 0.6, top - 0.4
    ramp = 4.0 if d["sp_h"] >= 10 else 3.2
    r_in = d["sp_o"] - 1.2
    r_top = d["sp_o"] - 0.3
    prof = [(r_in, z_lo), (d["pad"], z_lo), (d["pad"], z_hi - ramp), (r_top, z_hi), (r_in, z_hi)]
    pts = []
    for r, z in prof:
        for t in (-w, w):
            pts.append((r, t, z))
    pts = np.array(pts) * MM
    if ax == "Y":
        pts = pts[:, [1, 0, 2]]
    pts = pts * np.array([sign if ax == "X" else 1, sign if ax == "Y" else 1, 1])
    mesh = m.Mesh(pts, np.array([[0, 1, 2]]))
    travel = 0.003
    lower, upper = (-travel, 0.0) if sign > 0 else (0.0, travel)
    at = tuple(float(v) for v in pts.mean(axis=0))
    return m.slider(name, ax, lower, upper, (), (m.col_hull(f"{name}_c", mesh),), at=at, mass=0.003, spring=d["k"], damping=15.0, friction=0.0,
                    effort=60.0, armature=0.002, inertia=(2e-8, 2e-8, 2e-8))


def _top(name, k, pos, yaw):
    d = dims(k)
    s, zs, H = d["s"], d["zs"], d["H"]
    prof = _top_profile(d)
    mass_t, _ = _shell_mass_inertia(prof)
    look_parts = [("shell", lathe_mm(prof), "red")] + _decor_top(d)
    inner = [(outer_r(z, s) - d["wall"] - 0.2, z - zs) for z in np.linspace(zs + d["sp_h"] + 1.5, H - d["crown"] - 3.0, 10)]
    look_parts.append(("inside", lathe_mm([(d["bore"] - 0.2, 1.0), (d["bore"] - 0.2, d["sp_h"] + 0.5)] + inner + [(0.0, H - d["crown"] - zs - 0.2)]), "wood"))
    # colliders: the bore ring (24 sectors), the shell in tiers (16 sectors), the crown solid
    cols = _sector_hulls(f"{name}_b", [0.0, d["sp_h"] + 0.5], lambda z: d["rw"] if z < 0.5 else outer_r(z + zs, s), lambda z: d["bore"] + 0.8 * max(0.0, 1.5 - z) / 1.5, N_FIT)
    zc = H - d["crown"] - zs
    tiers = [d["sp_h"] + 0.5] + [z - zs for z in (82 * s, 96 * s, NECK * s, 116 * s, 131 * s) if z - zs < zc - 2] + [zc]
    cols += _sector_hulls(f"{name}_w", tiers, lambda z: outer_r(z + zs, s), lambda z: outer_r(z + zs, s) - d["wall"], N_WALL)
    cols.append(_solid_hull(f"{name}_crown", zc, H - zs, lambda z: outer_r(z + zs, s)))
    return m.free(name, pos, look_parts, cols, LOOKS, mass=mass_t, friction=MU, torsional_patch=0.01, solver_iters=32, yaw_deg=yaw)


def _closed_doll3(name, pos, yaw):
    s = DOLLS[3]["s"]
    H = 150 * s
    prof = [(0.0, 0.0)] + [(outer_r(z, s), z) for z in _OZ * s][:-1] + [(0.0, H)]
    d = dict(s=s, zs=SPLIT * s, H=H, rw=outer_r(SPLIT * s, s))
    parts = [("body", lathe_mm(prof), "red")]
    for nm, mesh, look in _decor_top(d):
        parts.append((nm, mesh.moved((0, 0, d["zs"] * MM)), look))
    parts += _decor_bottom(d)
    parts.append(("split", lathe_mm([(d["rw"] + 0.25, d["zs"] - 0.4), (d["rw"] + 0.25, d["zs"] + 0.4)]), "dark"))
    cols = [_solid_hull(f"{name}_body", 0.0, 96 * s, lambda z: outer_r(z, s)), _solid_hull(f"{name}_head", 96 * s, H, lambda z: outer_r(z, s))]
    return m.free(name, pos, parts, cols, LOOKS, mass=DOLL3_MASS, friction=MU, torsional_patch=0.008, solver_iters=32, yaw_deg=yaw)


def _shell_mass_inertia(profile):
    """Mass (kg) and principal inertia about the origin's axes (kg m2) of a closed (r, z) mm outline revolved, at density RHO (the frame's
    inertia is given about its centre of mass, which PhysX finds from the colliders; the small offset is ignored here)."""
    p = np.array(profile, float) * MM
    # polygon area integrals of the revolved closed outline (Pappus): V = 2 pi int r dA
    r, z = p[:, 0], p[:, 1]
    r2, z2 = np.roll(r, -1), np.roll(z, -1)
    cross = r * z2 - r2 * z
    area = 0.5 * cross.sum()
    sign = 1.0 if area > 0 else -1.0
    cr = (cross * (r + r2)).sum() / 6.0 * sign  # int r dA
    vol = 2 * math.pi * cr
    # int r^3 dA, int r z^2 dA via sampling the region (simple, adequate for an inertia)
    rr, zz = np.meshgrid(np.linspace(0, r.max(), 160), np.linspace(z.min(), z.max(), 320))
    from matplotlib.path import Path  # noqa: PLC0415  (installed with Isaac Lab)
    inside = Path(p).contains_points(np.c_[rr.ravel(), zz.ravel()]).reshape(rr.shape)
    dA = (r.max() / 159) * ((z.max() - z.min()) / 319)
    w = inside * rr * dA * 2 * math.pi * RHO  # mass per cell ring
    mass = float(w.sum()) if w.sum() > 0 else RHO * vol
    zc = float((w * zz).sum() / max(w.sum(), 1e-12))
    izz = float((w * rr ** 2).sum())
    ixx = float((w * (0.5 * rr ** 2 + (zz - zc) ** 2)).sum())
    return mass, (ixx, ixx, izz)


DOLL3_MASS = 0.010  # kg: dolls 3, 4 and 5 together (estimate: about 70% of a solid doll of doll 3's outline at RHO, 14.4 g; README)

# -- the layout (world, metres; z = table top). Faces and aprons are each half's +x. Bottoms stand open end up with their aprons toward the
#    arms (yaw 180); the tops stand on their rims, each turned a quarter turn off its bottom.
B1_POS, B1_YAW = (0.240, 0.025), 180.0
B2_POS, B2_YAW = (0.230, -0.105), 180.0
T1_POS, T1_YAW = (0.215, 0.200), 270.0
T2_POS, T2_YAW = (0.355, -0.190), 90.0
D3_POS, D3_YAW = (0.190, -0.215), 180.0

OBJECTS = [
    _bottom("bottom1", 1, (*B1_POS, TOP), B1_YAW),
    _top("top1", 1, (*T1_POS, TOP), T1_YAW),
    _bottom("bottom2", 2, (*B2_POS, TOP), B2_YAW),
    _top("top2", 2, (*T2_POS, TOP), T2_YAW),
    _closed_doll3("doll3", (*D3_POS, TOP), D3_YAW),
    # distractors: on a living-room table, ignored by the scorer
    SimReady("candle", pos=(0.530, 0.230, TOP), size=(0.074, 0.074, 0.075), mass=0.35, package="decor_candle_a01", usd="sm_decor_candle_a01_01.usd", convex=True),
    SimReady("mug", pos=(0.540, -0.030, TOP), size=(0.118, 0.135, 0.063), mass=0.30, package="kitchenware_mug_a01", usd="sm_kitchenware_mug_a01_01.usd", yaw=math.radians(30), convex=True),
    SimReady("pen", pos=(0.470, -0.300, TOP), size=(0.14, 0.012, 0.012), mass=0.02, package="stationery_pen_metal_a01", usd="sm_stationery_pen_metal_a01_01.usd", yaw=math.radians(60), convex=True),
]
PIECES = ("bottom1", "top1", "bottom2", "top2", "doll3")

# -- scoring ------------------------------------------------------------------------------------------------------------------------------

SEAT_UP, SEAT_DOWN = 1.0 * MM, 1.5 * MM  # a top's rim within 1 mm above its shoulder (it cannot go below: the shoulder stops it)
SEAT_OFF = 1.5 * MM  # centred on the bottom's axis
SEAT_TILT = 3.0  # degrees, square to it
FACE_TOL = 15.0  # degrees between the face and the apron
IN_UP, IN_OFF, IN_TILT = 2.0 * MM, 10.0 * MM, 8.0  # an inner doll on the floor of the next: within 2 mm of it, 10 mm of its axis, 8 degrees
UPRIGHT = 5.0  # degrees: the assembled doll standing
ON_TABLE = 2.0 * MM
HANDS = 0.030  # every fingertip point 3 cm or more clear of every piece
STILL_D, STILL_A = 1.0 * MM, 1.0  # over the last second no piece moves more than 1 mm or turns more than 1 degree
FALLEN = 0.030  # a piece more than 3 cm below the table top has fallen off it


def _rel(st_in, st_base):
    """Pose of st_in in st_base's frame: position (m), tilt between their axes (deg), turn of in's +x about base's axis (deg)."""
    Rb = st_base.rotation()
    Ri = st_in.rotation()
    p = Rb.T @ (np.asarray(st_in.pos) - np.asarray(st_base.pos))
    R = Rb.T @ Ri
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
    turn = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return p, tilt, turn


def seated(st_top, st_bot, k):
    p, tilt, turn = _rel(st_top, st_bot)
    dz = p[2] - dims(k)["zs"] * MM
    on = -SEAT_DOWN <= dz <= SEAT_UP and math.hypot(p[0], p[1]) <= SEAT_OFF and tilt <= SEAT_TILT
    return on, abs(turn) <= FACE_TOL, dz, turn


def inside(st_in, st_bot, k):
    p, tilt, _ = _rel(st_in, st_bot)
    dz = p[2] - dims(k)["floor"] * MM
    return (-1.0 * MM <= dz <= IN_UP and math.hypot(p[0], p[1]) <= IN_OFF and tilt <= IN_TILT), dz


def _extent(name):
    """(radius, z low, z high) of a piece about its own axis, from its origin (m)."""
    if name.startswith("bottom"):
        d = dims(int(name[-1]))
        return 0.0355 * d["s"], 0.0, (d["zs"] + d["sp_h"]) * MM
    if name.startswith("top"):
        d = dims(int(name[-1]))
        return 0.0355 * d["s"] * 0.95, 0.0, (d["H"] - d["zs"]) * MM
    s = DOLLS[3]["s"]
    return 0.0355 * s, 0.0, 0.150 * s


def hands_clear(states, tips):
    for name in PIECES:
        st = states[name]
        r, z0, z1 = _extent(name)
        loc = st.to_local(tips)
        rad = np.hypot(loc[:, 0], loc[:, 1])
        near = (rad < r + HANDS) & (loc[:, 2] > z0 - HANDS) & (loc[:, 2] < z1 + HANDS)
        if near.any():
            return False
    return True


def levels(states):
    b1, t1, b2, t2, d3 = (states[n] for n in PIECES)
    in3, dz3 = inside(d3, b2, 2)
    on2, face2, dz2, turn2 = seated(t2, b2, 2)
    in2, dzb = inside(b2, b1, 1)
    on1, face1, dz1, turn1 = seated(t1, b1, 1)
    stand = b1.tilt_deg() <= UPRIGHT and b1.pos[2] - TOP <= ON_TABLE
    lv = 0.0
    if in3:
        lv = 0.25
        if on2 and face2:
            lv = 0.5
            if in2:
                lv = 0.75
    full_now = lv == 0.75 and on1 and face1 and stand
    info = dict(in3=in3, dz3=dz3 / MM, on2=on2, face2=face2, dz2=dz2 / MM, turn2=turn2, in2=in2, dzb=dzb / MM, on1=on1, face1=face1, dz1=dz1 / MM,
                turn1=turn1, stand=stand)
    return lv, full_now, info


def _angle(q1, q2) -> float:
    """Degrees between two orientations (xyzw)."""
    d = abs(float(np.dot(q1, q2)))
    return math.degrees(2 * math.acos(min(1.0, d)))


class Trial:
    """Per-step scorer. The fall record and the stillness window are cleared only here, at a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.fallen = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.level = 0.0
        self.info = {}
        self.clear = False
        self.ref = None

    def step(self, env):
        states = {n: m.state(env, n) for n in PIECES}
        if any(st.pos[2] < TOP - FALLEN for st in states.values()):
            self.fallen = True
        lv, full_now, info = levels(states)
        tips = m.fingertips(env)
        self.clear = hands_clear(states, tips)
        ok = full_now and self.clear and not self.fallen
        if ok and self.ref is not None:  # still: every piece within 1 mm and 1 degree of where it was when the hold began
            ok = all(np.linalg.norm(states[n].pos - self.ref[n][0]) <= STILL_D and _angle(states[n].quat_xyzw, self.ref[n][1]) <= STILL_A for n in PIECES)
        if ok:
            if self.ref is None:
                self.ref = {n: (np.array(states[n].pos), np.array(states[n].quat_xyzw)) for n in PIECES}
            self.hold += 1
        else:
            self.hold, self.ref = 0, None
        self.full = self.hold >= HOLD_STEPS  # not sticky: it must hold at the end
        self.level, self.info = lv, info
        self.score = 0.0 if self.fallen else (1.0 if self.full else lv)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)

def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published layout) ------------------------------------------------------------------------------

MODES = ("full", "none", "no_press", "unturned", "hold", "outside")


def demo_plan(mode="full"):
    """The scripted demonstration ("full") and the red-team variants (README "Red team"), chosen with TASK_DEMO.
    Right hand: the third doll into the second's bottom; the second's top, turned to its apron, pressed on; the closed second doll, carried
    by its head, into the first's bottom. Left hand: the first's top, turned to its apron, pressed on over it."""
    if mode not in MODES:
        raise ValueError(f"TASK_DEMO must be one of {MODES}")
    from yam_arena.demo import Waypoint

    OPEN, SHUT = -0.04695, 0.0
    PAD = 0.013  # fingertips below the TCP

    def w(t, p, finger, label, yaw_deg):
        return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, math.radians(yaw_deg))

    park_l = np.array([0.16, 0.24, TOP + 0.20])
    park_r = np.array([0.16, -0.24, TOP + 0.20])
    d1, d2, s3 = dims(1), dims(2), DOLLS[3]["s"]
    # heights (m, above the table) of each head's widest point: the tops standing on their rims, and doll 3 standing
    heq1 = (HEAD_EQ - SPLIT) * MM
    heq2 = (HEAD_EQ - SPLIT) * d2["s"] * MM
    heq3 = HEAD_EQ * s3 * MM
    G = 0.004  # the TCP this far above a head's widest point: the pads straddle it
    # each head's width (m); the fingers open to it plus 14 mm, and close to it less 4 mm, so that both pads press it evenly (a full close
    # lets the stronger finger shove a light half along the table before the other arrives)
    w1, w2, w3 = (2 * outer_r(HEAD_EQ * s, s) * MM for s in (1.0, d2["s"], s3))
    SP = dict(o1=-(w1 + 0.014) / 2, o2=-(w2 + 0.014) / 2, o3=-(w3 + 0.014) / 2)
    SQ1, SQ2, SQ3 = (-(w - 0.004) / 2 for w in (w1, w2, w3))

    R, L = [], []
    yaw_r = 0.0
    # -- right: doll 3 into bottom 2
    p3 = np.array([D3_POS[0], D3_POS[1], TOP + heq3 + G + 0.002])
    b2 = np.array([B2_POS[0], B2_POS[1], TOP])
    drop3 = 0.012  # let go with doll 3's foot this far above bottom 2's floor
    over_b2 = b2 + [0, 0, d2["floor"] * MM + drop3 + heq3 + G + 0.002]
    R += [
        w(2.0, p3 + [0, 0, 0.06], SP["o3"], "above the third doll", 0),
        w(1.5, p3, SP["o3"], "pads either side of its head", 0),
        w(0.8, p3, SQ3, "pinch the head", 0),
        w(1.5, p3 + [0, 0, 0.07], SQ3, "lift it", 0),
        w(2.0, over_b2 + [0, 0, 0.065], SQ3, "over the second doll's bottom half", 0),
        w(2.0, over_b2, SQ3, "down into it", 0),
        w(0.6, over_b2, SP["o3"], "let go", 0),
        w(1.2, over_b2 + [0, 0, 0.06], SP["o3"], "up", 0),
    ]
    # -- right: top 2 turned a quarter turn to its apron and pressed on
    pt2 = np.array([T2_POS[0], T2_POS[1], TOP + heq2 + G])
    turn2 = 0.0 if mode == "unturned" else (B2_YAW - T2_YAW)  # +90: the face from +y to the arms
    seat2 = b2 + [0, 0, d2["zs"] * MM + heq2 + G]
    press2 = 0.0 if mode == "no_press" else 0.030
    R += [
        w(2.0, pt2 + [0, 0, 0.06], SP["o2"], "above the second doll's top", 0),
        w(1.5, pt2, SP["o2"], "pads either side of its head", 0),
        w(0.8, pt2, SQ2, "pinch the head", 0),
        w(1.5, pt2 + [0, 0, 0.07], SQ2, "lift it", 0),
        w(2.5, seat2 + [0, 0, 0.040], SQ2, "over the bottom half, turning the face to the apron", turn2),
        w(2.0, seat2 + [0, 0, d2["sp_h"] * MM + 0.002], SQ2, "down onto the spigot", turn2),
    ]
    if mode == "no_press":  # the careless way: set it on and let go, as if it would fall into place
        R += [w(1.5, seat2 + [0, 0, d2["sp_h"] * MM - 0.004], SQ2, "set it on", turn2),
              w(0.6, seat2 + [0, 0, d2["sp_h"] * MM - 0.004], SP["o2"], "let go", turn2)]
    else:
        R += [
            w(2.0, seat2 - [0, 0, press2], SQ2, "press it down over the spigot to the shoulder", turn2),
            w(0.8, seat2 - [0, 0, press2], SQ2, "pressing, twist it a little one way", turn2 + 8.0),
            w(1.2, seat2 - [0, 0, press2], SQ2, "and the other", turn2 - 8.0),
            w(0.8, seat2 - [0, 0, press2], SQ2, "and back, face to apron", turn2),
            w(0.6, seat2 - [0, 0, press2], SQ2, "hold the press", turn2),
            w(0.8, seat2 + [0, 0, 0.001], SQ2, "ease off", turn2),
            w(0.6, seat2 + [0, 0, 0.001], SP["o2"], "let go", turn2),
        ]
    R += [w(1.2, seat2 + [0, 0, 0.06], SP["o2"], "up", turn2)]
    # -- right: the closed second doll by its head into bottom 1
    b1 = np.array([B1_POS[0], B1_POS[1], TOP])
    head2 = b2 + [0, 0, d2["zs"] * MM + heq2 + G]  # where the head is now, if closed
    drop2 = 0.010
    over_b1 = b1 + [0, 0, d1["floor"] * MM + drop2 + d2["zs"] * MM + heq2 + G]
    if mode == "outside":  # the second doll left closed beside the first, which is then closed empty
        R += [w(2.0, park_r, OPEN, "the right hand away", 0)]
    else:
        R += [
            w(1.5, head2 + [0, 0, 0.02], SP["o2"], "over the closed second doll's head", 0),
            w(1.2, head2, SP["o2"], "pads either side of it", 0),
            w(0.8, head2, SQ2, "pinch the head", 0),
            w(2.0, head2 + [0, 0, 0.085], SQ2, "lift the closed doll by its head", 0),
            w(2.5, over_b1 + [0, 0, 0.07], SQ2, "over the first doll's bottom half", 0),
            w(2.5, over_b1, SQ2, "down into it", 0),
            w(0.6, over_b1, SP["o2"], "let go", 0),
            w(1.5, over_b1 + [0, 0, 0.05], SP["o2"], "up", 0),
            w(2.0, park_r, OPEN, "the right hand away", 0),
        ]
    t_right = sum(x.duration for x in R)
    # -- left: top 1, turned a quarter turn to its apron, pressed on once the second doll is in
    pt1 = np.array([T1_POS[0], T1_POS[1], TOP + heq1 + G])
    turn1 = 0.0 if mode == "unturned" else (B1_YAW - T1_YAW)  # -90
    seat1 = b1 + [0, 0, d1["zs"] * MM + heq1 + G]
    press1 = 0.0 if mode == "no_press" else 0.035
    L += [
        w(3.0, park_l, OPEN, "start", 0),
        w(2.0, pt1 + [0, 0, 0.06], SP["o1"], "above the first doll's top", 0),
        w(1.5, pt1, SP["o1"], "pads either side of its head", 0),
        w(0.8, pt1, SQ1, "pinch the head", 0),
        w(1.5, pt1 + [0, 0, 0.09], SQ1, "lift it", 0),
        w(2.5, pt1 + [0, 0, 0.09], SQ1, "turn it a quarter turn", turn1),
    ]
    t_l = sum(x.duration for x in L)
    L.append(w(max(t_right - 2.0 - t_l, 0.5), pt1 + [0, 0, 0.09], SQ1, "wait until the second doll is in", turn1))
    L += [
        w(2.5, seat1 + [0, 0, 0.065], SQ1, "over the first doll's bottom half, high enough to clear the second doll's head", turn1),
        w(2.0, seat1 + [0, 0, d1["sp_h"] * MM + 0.002], SQ1, "down onto the spigot", turn1),
    ]
    if mode == "no_press":
        L += [w(1.5, seat1 + [0, 0, d1["sp_h"] * MM - 0.004], SQ1, "set it on", turn1),
              w(0.6, seat1 + [0, 0, d1["sp_h"] * MM - 0.004], SP["o1"], "let go", turn1)]
    elif mode == "hold":
        L += [w(2.0, seat1 - [0, 0, press1], SQ1, "press it down", turn1),
              w(1.0, seat1 - [0, 0, press1], SQ1, "hold the press", turn1),
              w(0.8, seat1 + [0, 0, 0.001], SQ1, "ease off", turn1),
              w(4.0, seat1 + [0, 0, 0.001], SQ1, "keep holding the head", turn1)]
    else:
        L += [
            w(2.0, seat1 - [0, 0, press1], SQ1, "press it down over the spigot to the shoulder", turn1),
            w(0.8, seat1 - [0, 0, press1], SQ1, "pressing, twist it a little one way", turn1 + 8.0),
            w(1.2, seat1 - [0, 0, press1], SQ1, "and the other", turn1 - 8.0),
            w(0.8, seat1 - [0, 0, press1], SQ1, "and back, face to apron", turn1),
            w(0.6, seat1 - [0, 0, press1], SQ1, "hold the press", turn1),
            w(0.8, seat1 + [0, 0, 0.001], SQ1, "ease off", turn1),
            w(0.6, seat1 + [0, 0, 0.001], SP["o1"], "let go", turn1),
        ]
    if mode != "hold":
        L += [w(1.2, seat1 + [0, 0, 0.06], SP["o1"], "up", turn1), w(2.0, park_l, OPEN, "the left hand away", 0)]
    t_left = sum(x.duration for x in L)
    T = max(t_left, t_right) + 2.5
    L.append(w(T - t_left, L[-1].pos, L[-1].finger, "wait", math.degrees(L[-1].yaw)))
    R.append(w(T - t_right, R[-1].pos, R[-1].finger, "wait", math.degrees(R[-1].yaw)))
    if mode == "none":
        L = [w(T, park_l, OPEN, "the left hand stays away", 0)]
        R = [w(T, park_r, OPEN, "the right hand stays away", 0)]
    return {"left": L, "right": R}


def demo():
    return demo_plan(os.environ.get("TASK_DEMO", "full"))
