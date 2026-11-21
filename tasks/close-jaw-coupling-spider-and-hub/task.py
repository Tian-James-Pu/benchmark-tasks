"""Close a Lovejoy L100 jaw coupling between a brake motor and a driven shaft.

The motor's hub is set-screwed to its shaft (the brake holds that shaft). The driven shaft turns freely in two UCP202-10 pillow blocks
and carries the second hub on its key, slid back 60 mm and turned 30 degrees out of mesh. A new L099/100 open-centre spider stands in a
wooden cradle. Done: the spider is between the hubs, the loose hub's jaws are meshed into the spider's free gaps, and the hub faces are
the catalogue gap G apart (18.0 mm, scored 17.0 to 19.0 mm), with the hands away.

Model (README "Fidelity"):
- "motor" is a fixed part: the motor, its shaft and its hub (three jaw colliders and the hub body), so it collides with everything.
- "drive" is one articulation: the pillow blocks and their riser (the frame), the driven shaft (an unlimited hinge about x) and the loose
  hub on the shaft's key (a slider along x, child of the shaft: it turns with the shaft and slides along it). Self-collision is off inside
  an articulation, which is right here: the bore's fit on the shaft is the joints'.
- "spider" is a free rigid body: a ring and six parallel-sided legs, which collide with both hubs' jaws and faces by contact.
Nothing is detected or applied by code: the meshing, the stops and the gap all come from contact and the joints.
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
    """mechkit's spawn, with the articulation's reset writing its starting joint values as well as its root (Arena's default restores the root
    only, leaving the joints at the file's zero)."""
    obj = _plain_object(spec)
    if spec.role == "jointed":
        from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
        obj._pose_event_cfg.func = reset_articulation_pose_and_joints
    return obj


m._arena_object = _arena_object
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Close the jaw coupling between the motor and the driven shaft with the new spider, with the hub faces 18 mm apart."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 60  # 3 s at 20 Hz: long enough for the hands to be seen leaving the coupling
TOP = TABLE_TOP[SCENE]
IN = 0.0254

# -- Lovejoy L100 (2025 jaw catalogue, L-type dimensional data): OD 2.54 in, LTB 1.38 in, G 0.71 in, OAL 3.48 in; spider L099/100: A 2.54 in,
#    H (open centre) 1.03 in, W 0.61 in, B 0.43 in (read here as the legs' thickness). Coupling frame: origin on the shafts' axis at the motor
#    hub's face (the jaws' roots), x along the axis toward the motor, z up. Angles about x: phi from +z toward -y (right-handed about +x).
R_O = 2.54 * IN / 2  # hub OD and spider A, radius: 32.26 mm
LTB = 1.38 * IN  # hub length through the bore, 35.05 mm
G = 0.71 * IN  # catalogue gap between hub faces, 18.03 mm (the 2012 installation guide's Table 4 gives 0.750 in)
W = 0.61 * IN  # spider width along the axis, 15.49 mm
B = 0.43 * IN  # spider leg thickness, 10.92 mm
R_HOLE = 1.03 * IN / 2  # spider open centre radius, 13.08 mm
R_RING = 0.0180  # spider ring's outside (estimate: the legs run out from it)
R_J0 = 0.0200  # where the jaws start, radially (estimate: 2 mm outside the spider's ring)
JAW_L = 0.0165  # jaw length past the hub face (estimate: shorter than G, so jaw tips never reach the other hub's face)
PLAY = 0.0005  # clearance each side between a leg and a jaw (estimate: a close fit)
JAW_CH = 0.0010  # 45 degree chamfer on the jaw tips' flanks (estimate)
LEG_CH = 0.0008  # 45 degree break on the legs' edges at both faces (moulded radius, estimate)
FIXED_JAWS = (0.0, 120.0, 240.0)  # the motor hub's jaws (phi, degrees), one straight up
LOOSE_JAWS = (60.0, 180.0, 300.0)  # the loose hub's jaws, at shaft turn 0 (in mesh)
LEGS = tuple(30.0 + 60.0 * k for k in range(6))  # spider legs: two level (90 and 270 degrees)
R_SHAFT = 0.625 * IN / 2
HUB_MASS = 0.70  # sintered iron, 5/8 in bore (catalogue: 3.20 lb a coupling at minimum bore, 2.40 lb at maximum)
SPIDER_MASS = 0.027  # NBR, from its volume (21.6 cm3) at 1.25 g/cm3 (estimate)

X_FF = 0.300  # world x of the motor hub's face
AXZ = TOP + 3.5 * IN  # NEMA 56 shaft height D = 3-1/2 in
ORIGIN = np.array([X_FF, 0.0, AXZ])

FACE_HOME = -G  # loose hub face at the catalogue gap (slide 0)
SLIDE0 = -0.060  # start: slid back 60 mm (face 78.0 mm from the motor hub's face)
TURN0 = -30.0  # degrees: the loose hub's jaws start square on three spider legs' positions
SLIDE_HI = 0.0025  # the slide's stop: gap 15.5 mm (the hub face on the spider's legs comes first)
SHAFT_END = -(JAW_L + W + 0.0055)  # the driven shaft's end, 37.5 mm from the motor hub's face: room to bring the spider in
PB_X = (-0.150, -0.200)  # pillow blocks' centres
SHAFT_X1 = -0.230  # the driven shaft's outer end
PB_H = 1.1875 * IN  # UCP202-10 centre height 1-3/16 in
PB_L = 5.0 * IN  # base length 5 in (across the shaft)
PB_BOLT = 3.75 * IN  # bolt centres 3-3/4 in
PB_W = 0.031  # housing width along the shaft (the UC202 insert's 31 mm; estimate for the housing)
RISER_H = 3.5 * IN - PB_H  # steel riser under the pillow blocks, 58.7 mm

LOOKS = {
    "iron": m.Look(color=m.rgb(0.72, 0.73, 0.74), metallic=0.5, roughness=0.45),  # sintered iron, bright grey
    "iron_dark": m.Look(color=m.rgb(0.20, 0.21, 0.22), metallic=0.4, roughness=0.6),
    "steel": m.Look(color=m.rgb(0.66, 0.67, 0.69), metallic=0.9, roughness=0.3),
    "rubber": m.Look(color=m.rgb(0.05, 0.05, 0.055), roughness=0.85),
    "motor": m.Look(color=m.rgb(0.16, 0.30, 0.52), metallic=0.25, roughness=0.5),
    "motor_dark": m.Look(color=m.rgb(0.10, 0.11, 0.13), metallic=0.2, roughness=0.6),
    "brake": m.Look(color=m.rgb(0.55, 0.12, 0.10), metallic=0.2, roughness=0.5),
    "casting": m.Look(color=m.rgb(0.28, 0.30, 0.33), metallic=0.3, roughness=0.65),
    "riser": m.Look(color=m.rgb(0.35, 0.36, 0.38), metallic=0.5, roughness=0.5),
    "wood": m.Look(color=m.rgb(0.62, 0.45, 0.28), roughness=0.8),
    "hole": m.Look(color=m.rgb(0.06, 0.06, 0.06), roughness=0.9),
    "mark": m.Look(color=m.rgb(0.92, 0.92, 0.88), roughness=0.6),
}


# -- geometry helpers about the coupling's x axis ----------------------------------------------------------------------------------------


def _pt(x, r, phi):
    """A point at axial x, radius r, angle phi (rad) about +x from +z (a positive angle takes +z toward -y)."""
    return (x, -r * math.sin(phi), r * math.cos(phi))


def _uv(x, u, v, phi):
    """A point given in a frame turned phi about x: v radial (along the turned +z), u across it (along the turned -y)."""
    c, s = math.cos(phi), math.sin(phi)
    return (x, -(v * s + u * c), v * c - u * s)


def two_sided(mesh: m.Mesh) -> m.Mesh:
    return m.Mesh(mesh.v, np.vstack([mesh.f, mesh.f[:, ::-1]]))


def revolve(profile, segments=48) -> m.Mesh:
    """A closed (r, x) loop revolved about x."""
    n = len(profile)
    v = [_pt(x, r, 2 * math.pi * k / segments) for k in range(segments) for r, x in profile]
    f = []
    for k in range(segments):
        k2 = (k + 1) % segments
        for i in range(n):
            j = (i + 1) % n
            a, b, c, d = k * n + i, k * n + j, k2 * n + j, k2 * n + i
            f += [(a, b, c), (a, c, d)]
    return two_sided(m.Mesh(np.array(v, float), np.array(f)))


def hull_mesh(pts) -> m.Mesh:
    """The convex hull of points as a closed mesh (for visuals and hull colliders)."""
    pts = np.asarray(pts, float)
    try:
        from scipy.spatial import ConvexHull
        h = ConvexHull(pts)
        return two_sided(m.Mesh(pts, np.array(h.simplices)))
    except Exception:  # noqa: BLE001  colliders only need the points
        f = [(0, i, i + 1) for i in range(1, len(pts) - 1)]
        return two_sided(m.Mesh(pts, np.array(f)))


def x_cyl(r, x0, x1, segments=32) -> m.Mesh:
    return m.cylinder(r, abs(x1 - x0), ((x0 + x1) / 2, 0.0, 0.0), axis="x", segments=segments)


def jaw_points(phi_deg, x_root, x_tip, inset_tip=JAW_CH):
    """Points of one jaw from its root face (x_root) to its tip (x_tip), its flanks chamfered by inset_tip over the last inset_tip of length."""
    phi = math.radians(phi_deg)
    sgn = 1.0 if x_tip > x_root else -1.0
    s = B / 2 + PLAY
    pts = []
    for x, ins in ((x_root, 0.0), (x_tip - sgn * inset_tip, 0.0), (x_tip, inset_tip)):
        # inner chord ends and the outer arc, in the jaw's own frame: u across, v radial
        for r in (R_J0, R_O):
            for side in (-1.0, 1.0):
                # the flank line: points whose distance from the leg's centre line is s + ins
                beta = math.radians(30.0) - math.asin((s + ins) / r)
                pts.append(_uv(x, side * r * math.sin(beta), r * math.cos(beta), phi))
        b_o = math.radians(30.0) - math.asin((s + ins) / R_O)
        for a in np.linspace(-b_o, b_o, 7)[1:-1]:
            pts.append(_uv(x, R_O * math.sin(a), R_O * math.cos(a), phi))
    return pts


def leg_points(phi_deg, x0, x1, ch=LEG_CH):
    """One spider leg (parallel-sided, B thick) from radius R_RING - 1 mm out to its end arc at R_O, faces at x0 < x1, edges broken by ch."""
    phi = math.radians(phi_deg)
    pts = []
    h = B / 2
    for x, ins in ((x0, ch), (x0 + ch, 0.0), (x1 - ch, 0.0), (x1, ch)):
        u_max = h - ins
        for u in np.linspace(-u_max, u_max, 5):
            pts.append(_uv(x, u, R_RING - 0.001, phi))
            pts.append(_uv(x, u, math.sqrt(R_O ** 2 - u ** 2) - ins * 0.5, phi))
    return pts


def ring_points(a0_deg, a1_deg, x0, x1, n=3):
    pts = []
    for a in np.radians(np.linspace(a0_deg, a1_deg, n)):
        for r in (R_HOLE, R_RING):
            for x in (x0, x1):
                pts.append(_pt(x, r, a))
    return pts


# -- the motor and its hub (fixed) ------------------------------------------------------------------------------------------------------

MOTOR_SHOULDER = 1.875 * IN  # NEMA 56 V: usable shaft 1-7/8 in; the hub's face is flush with the shaft's end
FACE_X = (2.0625 - 1.875) * IN  # past the shoulder to the end shield's face (AH - V)
R_FRAME = 6.625 / 2 * IN  # NEMA 56 P
MOTOR_LEN = 0.250  # end shield to the brake's front (estimate)
BRAKE_LEN = 0.085
R_BRAKE = 0.072


def _hub_parts(prefix, x_face, toward, jaws):
    """A hub's visual parts and colliders: body from its face away from the jaws (toward = -1: the body runs to -x), jaws toward the other hub."""
    x_back = x_face + toward * LTB
    x_tip = x_face - toward * JAW_L
    parts = [(f"{prefix}_body", revolve([(R_SHAFT, min(x_face, x_back)), (R_O - 0.0008, min(x_face, x_back)), (R_O, min(x_face, x_back) + 0.0008),
                                          (R_O, max(x_face, x_back) - 0.0008), (R_O - 0.0008, max(x_face, x_back)), (R_SHAFT, max(x_face, x_back))], 64), "iron")]
    # the set screw's tapped hole over the keyway (5/16-18 at SL 0.44 in from the hub's back), and the keyway at the hub's back
    sx = x_back - toward * 0.44 * IN
    parts.append((f"{prefix}_tap", m.cylinder(0.004, 0.0006, (sx, 0.0, R_O + 0.0001)), "hole"))
    parts.append((f"{prefix}_kw", m.box((0.0006, 0.1875 * IN, 0.0024), (x_back + toward * 0.0003, 0.0, R_SHAFT + 0.0012)), "hole"))
    cols = [m.col_hull(f"{prefix}_body_c", x_cyl(R_O, x_face, x_back, segments=32))]
    for k, phi in enumerate(jaws):
        pts = jaw_points(phi, x_face, x_tip)
        parts.append((f"{prefix}_jaw{k}", hull_mesh(pts), "iron"))
        cols.append(m.col_hull(f"{prefix}_jaw{k}_c", hull_mesh(pts)))
    return parts, cols


def _motor():
    hp, hc = _hub_parts("mhub", 0.0, 1.0, FIXED_JAWS)
    xs = MOTOR_SHOULDER
    x_shield = xs + FACE_X
    shield = [(0.0, x_shield), (0.020, x_shield), (0.048, x_shield + 0.014), (R_FRAME, x_shield + 0.032), (R_FRAME, x_shield + 0.040),
              (0.0, x_shield + 0.040)]
    x_frame1 = x_shield + MOTOR_LEN
    parts = hp + [
        ("shaft", m.Mesh(x_cyl(R_SHAFT, LTB, xs + 0.0005, 32).v, x_cyl(R_SHAFT, LTB, xs + 0.0005, 32).f), "steel"),
        ("boss", revolve([(0.0, xs), (0.0085, xs), (0.0085, x_shield), (0.0, x_shield)]), "steel"),
        ("shield", revolve(shield), "motor"),
        ("frame", revolve([(0.0, x_shield + 0.040), (R_FRAME, x_shield + 0.040), (R_FRAME, x_frame1), (0.0, x_frame1)]), "motor"),
        ("brake", revolve([(0.0, x_frame1), (R_BRAKE, x_frame1), (R_BRAKE, x_frame1 + BRAKE_LEN - 0.01), (R_BRAKE - 0.01, x_frame1 + BRAKE_LEN),
                           (0.0, x_frame1 + BRAKE_LEN)]), "brake"),
        ("tbox", m.box((0.080, 0.065, 0.055), (x_shield + 0.14, -0.085, 0.045)), "motor"),
        ("plate", m.box((0.20, 0.004, 0.06), (x_shield + 0.14, -R_FRAME - 0.001, 0.0)), "mark"),
    ]
    base_z = -3.5 * IN
    bx = x_shield + 0.13
    parts += [("base", m.box((0.17, 0.14, 0.008), (bx, 0.0, base_z + 0.004)), "motor_dark"),
              ("cradle", m.box((0.13, 0.09, 3.5 * IN - R_FRAME + 0.006), (bx, 0.0, base_z + (3.5 * IN - R_FRAME) / 2 + 0.006)), "motor_dark")]
    cols = hc + [
        m.col_hull("shaft_c", x_cyl(R_SHAFT, LTB, xs, 16)),
        m.col_hull("shield_c", revolve(shield, 24)),
        m.col_hull("frame_c", x_cyl(R_FRAME, x_shield + 0.040, x_frame1, 24)),
        m.col_hull("brake_c", x_cyl(R_BRAKE, x_frame1, x_frame1 + BRAKE_LEN, 24)),
        m.col_box("base_c", (0.17, 0.14, 3.5 * IN - R_FRAME + 0.008), (bx, 0.0, base_z + (3.5 * IN - R_FRAME + 0.008) / 2)),
    ]
    return parts, cols


_mp, _mc = _motor()
MOTOR = m.fixed("motor", tuple(ORIGIN), _mp, _mc, LOOKS, friction=(0.8, 0.7))


# -- the driven side: pillow blocks on a riser (frame), shaft (hinge), loose hub on its key (slider) -------------------------------------


def _pillow_block(name, x):
    z0 = -PB_H  # the block's foot, on the riser
    parts = [(f"{name}_foot", m.box((PB_W + 0.004, PB_L, 0.014), (x, 0.0, z0 + 0.007)), "casting"),
             (f"{name}_web", m.box((PB_W, 0.070, PB_H - 0.010), (x, 0.0, z0 + 0.014 + (PB_H - 0.014) / 2 - 0.005)), "casting"),
             (f"{name}_house", revolve([(0.0, x - PB_W / 2), (0.032, x - PB_W / 2), (0.032, x + PB_W / 2), (0.0, x + PB_W / 2)], 40), "casting"),
             (f"{name}_insert", revolve([(R_SHAFT + 0.0003, x - 0.0157), (0.0185, x - 0.0157), (0.0185, x + 0.0157), (R_SHAFT + 0.0003, x + 0.0157)], 40), "steel"),
             (f"{name}_bolt_a", m.cylinder(0.008, 0.008, (x, PB_BOLT / 2, z0 + 0.018)), "iron_dark"),
             (f"{name}_bolt_b", m.cylinder(0.008, 0.008, (x, -PB_BOLT / 2, z0 + 0.018)), "iron_dark"),
             (f"{name}_grease", m.cylinder(0.0035, 0.008, (x, 0.0, 0.032 + 0.004)), "steel")]
    cols = [m.col_box(f"{name}_foot_c", (PB_W + 0.004, PB_L, 0.014), (x, 0.0, z0 + 0.007)),
            m.col_box(f"{name}_web_c", (PB_W, 0.070, PB_H - 0.014), (x, 0.0, z0 + 0.014 + (PB_H - 0.014) / 2)),
            m.col_hull(f"{name}_house_c", x_cyl(0.032, x - PB_W / 2, x + PB_W / 2, 20))]
    return parts, cols


def _drive_frame():
    parts, cols = [], []
    for k, x in enumerate(PB_X):
        p, c = _pillow_block(f"pb{k}", x)
        parts += p
        cols += c
    rx0, rx1 = min(PB_X) - 0.030, max(PB_X) + 0.030
    z_top = -PB_H
    parts.append(("riser", m.box((rx1 - rx0, 0.150, RISER_H), ((rx0 + rx1) / 2, 0.0, z_top - RISER_H / 2)), "riser"))
    cols.append(m.col_box("riser_c", (rx1 - rx0, 0.150, RISER_H), ((rx0 + rx1) / 2, 0.0, z_top - RISER_H / 2)))
    return parts, cols


def _shaft_link():
    parts = [("dshaft", two_sided(x_cyl(R_SHAFT, SHAFT_X1, SHAFT_END, 40)), "steel"),
             ("dkey", m.box((0.050, 0.1875 * IN - 0.0002, 0.0024), (SHAFT_END - 0.026, 0.0, R_SHAFT + 0.0006)), "steel"),
             ("dmark", m.box((0.006, 0.0012, 0.0004), (SHAFT_X1 + 0.006, 0.0, R_SHAFT + 0.0001)), "mark")]  # an index mark on the free end
    cols = [m.col_hull("dshaft_c", x_cyl(R_SHAFT, SHAFT_X1, SHAFT_END, 20))]
    return m.hinge("shaft", "X", (0.0, 0.0, 0.0), float("-inf"), float("inf"), parts, cols, mass=0.31, friction=0.02, damping=0.01,
                   armature=0.0002, inertia=(1.0e-5, 1.1e-3, 1.1e-3))


def _hub_link():
    hp, hc = _hub_parts("lhub", FACE_HOME, -1.0, LOOSE_JAWS)
    return m.slider("hub", "X", SLIDE0 - 0.002, SLIDE_HI, hp, hc, at=(0.0, 0.0, 0.0), parent="shaft", mass=HUB_MASS, friction=0.3,
                    damping=4.0, armature=0.01, inertia=(3.6e-4, 2.6e-4, 2.6e-4))


_fp, _fc = _drive_frame()
DRIVE = m.mechanism("drive", tuple(ORIGIN), [_shaft_link(), _hub_link()], LOOKS, frame_parts=_fp, frame_colliders=_fc, friction=(0.8, 0.7),
                    solver_iters=48, start=[("shaft", math.radians(TURN0)), ("hub", SLIDE0)])


# -- the spider (free) and its cradle ---------------------------------------------------------------------------------------------------


def _spider():
    x0, x1 = -W / 2, W / 2
    parts, cols = [], []
    for k, phi in enumerate(LEGS):
        pts = leg_points(phi, x0, x1)
        parts.append((f"leg{k}", hull_mesh(pts), "rubber"))
        cols.append(m.col_hull(f"leg{k}_c", hull_mesh(pts)))
    for k in range(12):
        pts = ring_points(30.0 * k, 30.0 * (k + 1), x0, x1)
        parts.append((f"ring{k}", hull_mesh(pts), "rubber"))
        cols.append(m.col_hull(f"ring{k}_c", hull_mesh(pts)))
    return parts, cols


CRADLE = np.array([0.215, 0.175])  # world x, y of the cradle's centre (the spider's centre above it)
CR_FLOOR = 0.004  # the slot's floor above the bench, where the two lowest legs' corners rest
CR_SLOT = W + 0.0016  # slot width along x
CR_WALL, CR_WALL_H = 0.012, 0.018  # wall thickness along x, height above the bench (below the pinch at the level legs' ends)
CR_LEN = 0.100  # along y
LEG_LOW = math.sqrt(R_O ** 2 - (B / 2) ** 2) * math.cos(math.radians(30.0)) + (B / 2) * math.sin(math.radians(30.0))  # the 150/210 degree
# legs' lower corners below the centre: 30.3 mm
SPIDER0 = (CRADLE[0], CRADLE[1], TOP + CR_FLOOR + LEG_LOW + 0.0004)


def _cradle():
    parts, cols = [], []
    for nm, size, c in (("wall_n", (CR_WALL, CR_LEN, CR_WALL_H), (-(CR_SLOT + CR_WALL) / 2, 0.0, CR_WALL_H / 2)),
                        ("wall_f", (CR_WALL, CR_LEN, CR_WALL_H), ((CR_SLOT + CR_WALL) / 2, 0.0, CR_WALL_H / 2)),
                        ("floor", (CR_SLOT, CR_LEN, CR_FLOOR), (0.0, 0.0, CR_FLOOR / 2))):
        parts.append((nm, m.box(size, c), "wood"))
        cols.append(m.col_box(nm + "_c", size, c))
    return parts, cols


_sp, _sc = _spider()
SPIDER = m.free("spider", SPIDER0, _sp, _sc, LOOKS, mass=SPIDER_MASS, friction=(0.9, 0.8), torsional_patch=0.004, solver_iters=48)
_cp, _cc = _cradle()
CRADLE_PART = m.fixed("cradle", (CRADLE[0], CRADLE[1], TOP), _cp, _cc, LOOKS, friction=(0.8, 0.7))

# -- distractors (SimReady): a shop rag, a combination wrench, a can of multi-purpose oil
RAG = SimReady("rag", pos=(0.48, 0.28, TOP), size=(0.26, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01", usd="sm_cleaning_cloth_a01_01.usd", yaw=0.4)
WRENCH = SimReady("wrench", pos=(0.20, -0.20, TOP), size=(0.26, 0.046, 0.017), mass=0.25, package="handTool_wrench_combination_a01",
                  usd="sm_handTool_wrench_combination_a01_01.usd", yaw=0.15)
OIL = SimReady("oil", pos=(0.60, -0.24, TOP), size=(0.052, 0.104, 0.272), mass=0.45, package="consumable_lubricant_oil_multi_purpose_a01",
               usd="sm_consumable_lubricant_oil_multi_purpose_a01_01.usd", yaw=0.5)

OBJECTS = [MOTOR, DRIVE, SPIDER, CRADLE_PART, RAG, WRENCH, OIL]


# -- reading the coupling ---------------------------------------------------------------------------------------------------------------


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def coupling(env) -> dict:
    """Everything the scorer reads, in the coupling frame (metres, degrees)."""
    d = m.state(env, "drive")
    sp = m.state(env, "spider")
    slide, turn = d.joint("hub"), math.degrees(d.joint("shaft"))
    face = FACE_HOME + slide  # the loose hub's face
    c = np.asarray(sp.pos, float) - ORIGIN
    R = sp.rotation()
    ax = R[:, 0]
    tilt = math.degrees(math.acos(float(np.clip(abs(ax[0]), -1.0, 1.0))))
    off = math.hypot(c[1], c[2])
    up = R[:, 2]  # the spider's leg 0, turned about its axis
    phase = math.degrees(math.atan2(-up[1], up[2]))
    leg_err = abs(_wrap_deg((phase + 30.0) % 60.0 - 30.0))  # to the nearest leg position
    far, near = c[0] + W / 2 * abs(ax[0]), c[0] - W / 2 * abs(ax[0])
    jaw_phase = abs(_wrap_deg(((turn % 120.0) + 60.0) % 120.0 - 60.0))  # loose hub's turn to the nearest meshing position
    return {"slide": slide, "turn": turn, "face": face, "gap": -face, "tips": face + JAW_L, "sp_c": c, "sp_far": far, "sp_near": near,
            "sp_off": off, "sp_tilt": tilt, "sp_leg": leg_err, "sp_v": float(np.linalg.norm(sp.lin_vel)), "jaw_phase": jaw_phase}


def _cyl_gap(pts, x0, x1, r):
    """Each point's distance from a cylinder about the coupling's axis (radius r, from x0 to x1)."""
    q = np.atleast_2d(pts) - ORIGIN
    rho = np.hypot(q[:, 1], q[:, 2])
    dx = np.maximum(np.maximum(x0 - q[:, 0], q[:, 0] - x1), 0.0)
    return np.hypot(dx, np.maximum(rho - r, 0.0))


SP_FAR_TOL = 0.0050  # the spider's far face within 5 mm of the motor hub's face (its legs 11.5 mm or more into the 16.5 mm jaws)
SP_OFF_TOL = 0.0015  # its centre within 1.5 mm of the axis
SP_TILT_TOL = 3.0  # degrees
SP_LEG_TOL = 5.0  # degrees: its legs between the jaws
MESH_DEPTH = 0.005  # the loose hub's jaw tips 5 mm or more past the spider's near face
GAP_LO, GAP_HI = 0.0170, 0.0190  # hub faces 17.0 to 19.0 mm apart
CLEAR = 0.03  # every fingertip 3 cm or more from the spider and both hubs
STILL_SLIDE, STILL_TURN, STILL_V = 0.0003, 0.5, 0.005


def levels(c, tips) -> dict:
    sp_in = (c["sp_far"] > -SP_FAR_TOL and c["sp_off"] < SP_OFF_TOL and c["sp_tilt"] < SP_TILT_TOL and c["sp_leg"] < SP_LEG_TOL)
    sp_clear = float(np.min(_cyl_gap(tips, c["sp_near"], c["sp_far"], R_O))) >= CLEAR
    meshed = c["tips"] >= c["sp_near"] + MESH_DEPTH
    gap_ok = GAP_LO <= c["gap"] <= GAP_HI
    all_clear = float(np.min(_cyl_gap(tips, c["face"] - LTB, LTB, R_O))) >= CLEAR
    return {"sp_in": sp_in, "sp_clear": sp_clear, "meshed": meshed, "gap_ok": gap_ok, "all_clear": all_clear}


class Trial:
    """The score is read from the state at each step; the trial's end state counts. Only the stillness window is kept across steps,
    and it is cleared only here, at a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.level = 0.0
        self.ref = None
        self.c = {}
        self.lv = {}

    def step(self, env):
        c = coupling(env)
        tips = m.fingertips(env)
        lv = levels(c, tips)
        self.c, self.lv = c, lv
        level = 0.0
        # the spider must stay in by itself: hands 3 cm off it, unless the loose hub is in mesh, which traps it between the hubs
        if lv["sp_in"] and (lv["sp_clear"] or lv["meshed"]):
            level = 0.25
            if lv["meshed"]:
                level = 0.5
                if lv["gap_ok"]:
                    level = 0.75
        self.level = level
        ready = level == 0.75 and lv["all_clear"] and c["sp_v"] < STILL_V
        if ready and self.ref is not None:
            ready = abs(c["slide"] - self.ref[0]) < STILL_SLIDE and abs(_wrap_deg(c["turn"] - self.ref[1])) < STILL_TURN
        if ready:
            if self.ref is None:
                self.ref = (c["slide"], c["turn"])
            self.hold += 1
        else:
            self.hold = 0
            self.ref = None
        if self.hold >= HOLD_STEPS:
            self.full = True
        self.score = 1.0 if self.full else level


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def _debug(env):
    path = os.environ.get("JAW_LOG")
    if not path:
        return
    c, lv = TRIAL.c, TRIAL.lv
    tips = m.fingertips(env)
    with open(path, "a") as f:
        f.write(f"slide {c['slide']*1000:.2f} turn {c['turn']:.2f} gap {c['gap']*1000:.2f} sp {np.round(c['sp_c']*1000, 2).tolist()} far {c['sp_far']*1000:.2f} "
                f"near {c['sp_near']*1000:.2f} off {c['sp_off']*1000:.2f} tilt {c['sp_tilt']:.2f} leg {c['sp_leg']:.2f} v {c['sp_v']*1000:.1f} "
                f"{ {k: bool(v) for k, v in lv.items()} } lvl {TRIAL.level} hold {TRIAL.hold} full {TRIAL.full} "
                f"L {np.round(tips[:12].mean(0)*1000, 1).tolist()} R {np.round(tips[12:].mean(0)*1000, 1).tolist()}\n")


m.on_step(_debug)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published layout) ------------------------------------------------------------------------------

PAD_TIP = 0.013  # finger tips below the TCP
# corrections from earlier runs of this plan (0 = nominal)
SP_SAG = 0.0  # the held spider's centre below where the grip puts it
ROLL_FIX = 3.0  # degrees: pressed onto the legs, the hub turns about 3 degrees toward mesh on its own (measured in an earlier run of this plan)
GAP_FIX = 0.0018  # the hub ends up about 1.8 mm short of where the hand sends it, once let go (measured likewise)


MODES = ("full", "none", "no_spider", "drop_spider", "no_turn", "hard_push", "hold", "one_arm")


def demo_plan(mode: str = "full"):
    """The scripted demonstration ("full") and the red-team variants (README "Red team"), chosen with TASK_DEMO."""
    if mode not in MODES:
        raise ValueError(f"TASK_DEMO must be one of {MODES}")
    if mode == "one_arm":
        return _one_arm_plan()
    import demokit as d
    from yam_arena.demo import Waypoint
    from yam_arena.urdf import axis_angle_matrix

    OPEN, SHUT = -0.04695, 0.0

    def w(t, p, finger, label, grip):
        return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, grip)

    def rolled(phi_deg, yaw=0.0):
        return d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(phi_deg)) @ d.down(yaw).rot)

    g0 = d.down(0.0)
    park_l = np.array([0.15, 0.25, TOP + 0.18])
    park_r = np.array([0.15, -0.25, TOP + 0.18])
    x_tip = X_FF - JAW_L  # world x of the motor hub's jaw tips

    # -- left: the spider, pinched across its two level legs' ends (64.5 mm across), the pads at the shafts' height. The fingers stay
    #    outside the hubs' OD, so the spider can go into the jaws in the hand: to 4 mm off the hub's face, where the fingers' wider upper
    #    parts are still short of the hub's body.
    SP_OPEN = -0.0410  # 82 mm between the pads
    sp_c0 = np.array(SPIDER0)
    grip_dz = PAD_TIP - 0.0045  # the fingertips 4.5 mm below the spider's centre: above the jaws beside the level legs (their tops 5.9 mm down)
    pick = np.array([sp_c0[0], sp_c0[1], sp_c0[2] + grip_dz])
    sp_x_out = x_tip - 0.004 - W / 2  # the spider's centre with its far face 4 mm short of the jaw tips
    sp_x_in = X_FF - 0.004 - W / 2  # its far face 4 mm off the hub's face: legs 12.5 mm into the jaws
    held = lambda sx: np.array([sx, 0.0, AXZ + grip_dz + SP_SAG])  # noqa: E731
    left = [
        w(2.5, pick + [0, 0, 0.06], SP_OPEN, "above the spider", g0),
        w(1.5, pick, SP_OPEN, "pads either side, at the level legs' ends", g0),
        w(1.0, pick, SHUT, "pinch the spider across its legs", g0),
        w(1.0, pick, SHUT, "hold", g0),
        w(1.5, pick + [0, 0, 0.07], SHUT, "lift the spider out of its cradle", g0),
        w(2.5, held(sp_x_out) + [0, 0, 0.05], SHUT, "carry it over the gap between the hubs", g0),
        w(2.5, held(sp_x_out), SHUT, "down onto the shafts' line, legs lined up between the motor hub's jaws", g0),
        w(1.0, held(sp_x_out), SHUT, "pause", g0),
        w(3.5, held(sp_x_in), SHUT, "slowly into the motor hub's jaws", g0),
        w(0.8, held(sp_x_in), SHUT, "settle", g0),
        w(0.8, held(sp_x_in), SP_OPEN, "let go", g0),
        w(1.2, held(sp_x_in) + [0, 0, 0.06], SP_OPEN, "up clear of the coupling", g0),
        w(2.5, park_l, SP_OPEN, "the left hand away", g0),
    ]
    if mode == "drop_spider":  # the careless way: let the spider go in the gap between the hubs without putting it into the jaws
        left = left[:8] + [w(0.8, held(sp_x_out), SP_OPEN, "let go between the hubs", g0),
                           w(1.2, held(sp_x_out) + [0, 0, 0.06], SP_OPEN, "up", g0),
                           w(2.5 + 4.3, park_l, SP_OPEN, "the left hand away", g0)]
    if mode in ("no_spider", "none"):
        left = [w(sum(p.duration for p in left), park_l, OPEN, "the left hand stays away", g0)]
    t_left = sum(p.duration for p in left)

    # -- right: pinch the loose hub across its OD, slide it up to the spider, roll it into mesh while pressing, set the gap, let go
    HUB_OPEN = -0.0420  # 84 mm between the pads
    tcp_dz = PAD_TIP - 0.010  # the fingertips 10 mm below the axis: the pads across the OD at the axis
    face0 = X_FF + FACE_HOME + SLIDE0

    def hub_at(face_x, roll=0.0, out=0.0):
        """TCP for a hub face at world x face_x, the gripper rolled `roll` degrees about the axis (the TCP on its circle about the axis),
        `out` metres further out along the gripper's own axis."""
        cx = face_x - LTB / 2
        r = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(roll)) @ np.array([0.0, 0.0, tcp_dz + out])
        return np.array([cx, r[1], AXZ + r[2]])

    contact = X_FF - W - JAW_L  # the loose face where its jaw tips meet the spider's legs (the hub pushes the spider home first)
    press = contact + 0.0025
    roll_end = -TURN0 - ROLL_FIX
    right = [
        w(2.0, park_r, OPEN, "start", g0),
    ]
    t_r = sum(p.duration for p in right)
    right.append(w(max(t_left - 1.5 - t_r, 0.5), park_r, OPEN, "wait until the spider is in", g0))
    right += [
        w(2.5, hub_at(face0) + [0, 0, 0.06], HUB_OPEN, "above the loose hub", g0),
        w(1.5, hub_at(face0), HUB_OPEN, "fingers either side of the hub", g0),
        w(1.0, hub_at(face0), SHUT, "pinch the hub across its OD", g0),
        w(3.0, hub_at(contact - 0.003), SHUT, "slide it along the shaft toward the spider", g0),
        w(1.0, hub_at(contact - 0.003), SHUT, "pause", g0),
        w(1.5, hub_at(press), SHUT, "jaw tips onto the spider's legs, pressing", g0),
        w(1.0, hub_at(press), SHUT, "pressing: out of phase, blocked", g0),
    ]
    n = 8
    if mode == "no_turn":  # pushed straight in without turning: the jaw tips stay on the spider's legs
        roll_end = 0.0
    for k in range(n):  # roll slowly (3.4 degrees a step, 3.4 deg/s), pressing, until the jaws drop into the spider's free gaps
        ph = roll_end * (k + 1) / n
        right.append(w(1.0, hub_at(press, ph), SHUT, "turn the hub, pressing, to find the gaps", rolled(ph)))
    rg = rolled(roll_end)
    target = X_FF + FACE_HOME + GAP_FIX
    if mode == "hard_push":  # the careless way: drive the hub hard against the spider (gap 15.5 mm, no float left)
        target = X_FF - W + 0.002
    right += [
        w(2.0, hub_at(press, roll_end), SHUT, "the jaws drop into the spider's free gaps", rg),
        w(3.0, hub_at(target, roll_end), SHUT, "slide the hub to the catalogue gap", rg),
        w(1.0, hub_at(target, roll_end), SHUT, "hold at the gap", rg),
        w(0.8, hub_at(target, roll_end), HUB_OPEN, "let go of the hub", rg),
        w(1.5, hub_at(target, roll_end, 0.06), HUB_OPEN, "back off the hub along the fingers", rg),
        w(2.5, park_r, HUB_OPEN, "the right hand away", g0),
        w(3.0, park_r, HUB_OPEN, "hands clear", g0),
    ]
    if mode == "hold":  # never lets go of the hub
        right = right[:-4] + [w(7.8, hub_at(target, roll_end), SHUT, "keep holding the hub", rg)]
    if mode == "none":
        right = [w(sum(p.duration for p in right), park_r, OPEN, "the right hand stays away", g0)]
    t_right = sum(p.duration for p in right)
    left.append(w(max(t_right - t_left, 0.5), park_l, SHUT, "hands clear", g0))
    return {"left": left, "right": right}


def _one_arm_plan():
    """Red team, one arm: the left hand seats the spider, then works the hub (its grip pre-rolled -14 deg so the 27 deg turn stays in
    its reach); the right arm never moves."""
    import dataclasses
    import demokit as d
    from yam_arena.demo import Waypoint
    from yam_arena.urdf import axis_angle_matrix
    p = demo_plan("full")
    L, R = p["left"], p["right"]
    SHUT, HUB_OPEN = 0.0, -0.0420
    R0 = -14.0

    def rolled(phi):
        return d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(phi)) @ d.down(0.0).rot)

    tcp_dz = PAD_TIP - 0.010

    def hub_at(face_x, roll=0.0, out=0.0):
        cx = face_x - LTB / 2
        r = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(roll)) @ np.array([0.0, 0.0, tcp_dz + out])
        return tuple(float(v) for v in (cx, r[1], AXZ + r[2]))

    def w(t, pos, f, lab, roll):
        return Waypoint(float(t), pos, float(f), lab, rolled(roll))

    face0 = X_FF + FACE_HOME + SLIDE0
    contact = X_FF - W - JAW_L
    press = contact + 0.0025
    roll_end = -TURN0 - ROLL_FIX
    target = X_FF + FACE_HOME + GAP_FIX
    left = list(L[:12])  # the spider, up to "up clear of the coupling"
    left += [w(2.5, tuple(np.add(hub_at(face0, R0), (0, 0, 0.06))), HUB_OPEN, "above the loose hub", R0),
             w(1.5, hub_at(face0, R0), HUB_OPEN, "fingers either side", R0),
             w(1.0, hub_at(face0, R0), SHUT, "pinch", R0),
             w(3.0, hub_at(contact - 0.003, R0), SHUT, "slide", R0),
             w(1.0, hub_at(contact - 0.003, R0), SHUT, "pause", R0),
             w(1.5, hub_at(press, R0), SHUT, "press", R0),
             w(1.0, hub_at(press, R0), SHUT, "blocked", R0)]
    for k in range(8):
        ph = R0 + roll_end * (k + 1) / 8
        left.append(w(1.0, hub_at(press, ph), SHUT, "turn", ph))
    re = R0 + roll_end
    left += [w(2.0, hub_at(press, re), SHUT, "drop in", re),
             w(3.0, hub_at(target, re), SHUT, "to the gap", re),
             w(1.0, hub_at(target, re), SHUT, "hold", re),
             w(0.8, hub_at(target, re), HUB_OPEN, "let go", re),
             w(1.5, hub_at(target, re, 0.06), HUB_OPEN, "back off", re),
             dataclasses.replace(L[12], duration=2.5),
             dataclasses.replace(L[12], duration=4.0, label="hands clear")]
    T = sum(x.duration for x in left)
    right = [dataclasses.replace(R[0], duration=T, label="the right arm stays parked")]
    return {"left": left, "right": right}


def demo():
    return demo_plan(os.environ.get("TASK_DEMO", "full"))
