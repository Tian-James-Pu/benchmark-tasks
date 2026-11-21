"""Open a 15 oz food can on a countertop lever-type electric can opener, then stand the open can on the counter with its lid left on
the opener's magnet.

The opener is modelled on the Hamilton Beach 76702 Extra-Tall Can Opener (9 x 4.6 x 5.7 in, piercing lever with a magnetic lid holder,
geared wheel, automatic shut-off). Its Use & Care guide: lift the piercing lever, raise the can so its rim sits on the geared wheel under the
cutter, press the lever down until the cutter pierces the lid; "It is not necessary to hold lever down or hold can while can is being
opened"; "The motor will automatically stop when can is open"; then (picture 5) one hand holds the can while the other lifts the lever,
and the lid comes away on the magnet. The can is a No. 300 ("15 oz") three-piece steel can, 3 in across the seam and 4 7/16 in tall.

Model (declared in code; every state change is driven by contact and joint angles, never by the grippers' proximity):
- Opener ("opener", fixed articulation): housing, pivot brackets and one link, the piercing lever (hinge about y, 0 = raised, 31 deg
  pressed past home), carrying the cutting wheel (a 18 mm steel disc, real collider), the magnet and, hidden until the cut closes, the lid
  it will hold. A return spring holds the lever up: about 6 N at the grip lifted and about 14 N pressed home (estimates; not measured).
- Geared wheel ("gear", fixed): a 20 mm wheel standing 8 mm out of the face, its top 150 mm above the counter. Its teeth are not modelled:
  it is smooth and slippery (friction 0.1, combined as the minimum), so a can rests on it only while the cutter clamps the rim.
- Can ("can", free articulation, 0.47 kg): its top (the double seam bead, 76.2 mm across and 3.5 mm tall, and the lid, 3.2 mm down in
  its countersink) is the frame; the body (72.4 mm wall, bottom seam and label, 0.43 kg) turns on a joint about the can's axis. That joint
  is locked until the lid is pierced, then a motor turns it (45 deg/s, at most 0.6 N m: a firm grip on the can stalls it) until the body
  has turned one full turn past the cutter: on the real opener the whole can turns past a fixed cutter, here the top ring stays in the
  jaws and the body turns under it, which is the same thing for an axisymmetric seam.
- Pierced: the lever pressed to within 2.5 deg of home (25 deg) while the cutter's edge is on the lid inside the seam (1.5 mm) and the
  seam's underside is on the geared wheel (within 2 mm below to 1 mm above). Then the lever latches. Seated too high, the cutter lands on
  the bead and the lever stops short; held off the wheel, the cutter meets nothing.
- When the body has turned 360 deg the lid is cut free: it is shown on the magnet, the can shown open, the motor stops, and the latch
  becomes a detent that holds the lever down against the hanging can (up to 1.2 N m) until the lever is lifted 6 deg; then it springs up.
  Lifted with nothing holding the can, the can slides off the wheel and falls.

Scoring (end state, kept by the tracker each control step; `score()` returns it):
- 0.0 if, at any time after the lid was pierced, the can tilts more than 20 degrees from upright (an opened or pierced can spills);
  cleared only at a reset.
- 1.0: the lid cut through (360 deg) and on the magnet with the lever raised (within 10 deg of its stop), the can upright (within 10 deg)
  with its base on the counter (within 3 mm), still, and every fingertip at least 5 cm from the can, all held 1 s.
- 0.75: the lid cut through and on the magnet, without the rest of 1.0.
- 0.5: the cut at least 270 degrees round, not through.
- 0.25: the lid pierced, cut less than 270 degrees.
- 0.0: never pierced.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import dataclasses  # noqa: E402

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Open the can with the electric can opener and set the open can down on the counter, leaving the lid on the magnet."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]
DT = 0.05

# -- the can: No. 300 ("15 oz"), 300 x 407: 3 in over the seam, 4 7/16 in tall. Origin: the centre of its base. -----------------------
R_SEAM = 0.0381  # over the double seam
R_BODY = 0.0362  # the body wall below it: the bead overhangs it by 1.9 mm
R_SEAM_IN = 0.0345  # the seam's inside, where the lid's countersink wall is
H_CAN = 0.1127
SEAM_H = 0.0035
Z_SEAM_BOT = H_CAN - SEAM_H  # 0.1092: the bead's underside, which hooks on the geared wheel
Z_LID = H_CAN - 0.0032  # the lid's top, 3.2 mm down in its countersink
R_CUT = 0.0295  # where the cutter runs on the lid, 5 mm inside the seam's inner wall
MASS_TOP, MASS_BODY = 0.04, 0.43
MOTOR_TORQUE = 0.3  # N m at most at the can (an estimate)

# -- the opener: Hamilton Beach 76702 class, 9 in tall, 4.6 in wide, 5.7 in deep. Origin: the counter under the middle of its front face.
XF, YO = 0.40, 0.0  # its front face (facing the arms, -x) and centre line
W_OP, D_OP, H_OP = 0.117, 0.145, 0.229
GEAR_TOP = 0.150  # the geared wheel's top above the counter: where the bead rests
GEAR_R, GEAR_OUT = 0.010, 0.008  # radius, and how far it stands out of the face
SEAT = np.array([XF - GEAR_OUT - R_BODY - 0.0002, YO, TOP + GEAR_TOP - Z_SEAM_BOT])  # the can's base centre when seated

# the piercing lever, about y. Joint zero is raised; THETA_P (25 deg) pressed home on a seated can's lid.
THETA_P = math.radians(25.0)
THETA_MAX = 31.0  # deg: its stop with no can
CUT_R = 0.009  # cutting wheel radius
CUT_OFF = np.array([0.028, 0.0, 0.0055])  # the cutting wheel's centre from the pivot, lever raised
GRIP_H = 0.068  # where a hand holds the lever: this far up the handle from the pivot
HANDLE_L = 0.085
K_LEVER, PRELOAD = 0.9, -0.30  # N m/rad, and its rest target (rad): 0.27 N m raised (4 N at the grip), 0.66 N m pressed home (10 N)
PRESS_TOL = 3.0  # deg: the lever counts as pressed home within this of 25 deg
K_CLAMP, E_CLAMP = 10.0, 0.5  # once pierced, the latch presses the cutter on: stiffness (N m/rad) toward the lever's stop, at most 0.5 N m (18 N at the cutter)


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


CUT_PRESSED = SEAT + np.array([R_CUT, 0.0, Z_LID + CUT_R])  # the cutter's centre, pressed home on a seated lid (world)
PIVOT = CUT_PRESSED - _ry(THETA_P) @ CUT_OFF  # world
ORIGIN = np.array([XF, YO, TOP])  # the opener's origin (world)
P_LOCAL = PIVOT - ORIGIN


def cutter_low(theta: float) -> np.ndarray:
    """The lowest point of the cutting wheel's edge (world) with the lever at theta (rad)."""
    c = PIVOT + _ry(theta) @ CUT_OFF
    a = theta - THETA_P  # the wheel is vertical (its axis along x) when pressed home
    return c - CUT_R * np.array([math.sin(a), 0.0, math.cos(a)])


GEAR_EDGE = np.array([XF - GEAR_OUT + 0.0005, YO, TOP + GEAR_TOP])  # the geared wheel's top front edge (world)

LOOKS = {
    "tin": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.9, roughness=0.3),
    "label": m.Look(color=m.rgb(0.80, 0.16, 0.10), roughness=0.6),
    "label2": m.Look(color=m.rgb(0.95, 0.86, 0.55), roughness=0.6),
    "stripe": m.Look(color=m.rgb(0.20, 0.45, 0.20), roughness=0.6),
    "cutline": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.5),
    "food": m.Look(color=m.rgb(0.62, 0.20, 0.08), roughness=0.4),
    "body": m.Look(color=m.rgb(0.07, 0.07, 0.08), roughness=0.4),
    "trim": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=0.85, roughness=0.25),
    "steel": m.Look(color=m.rgb(0.55, 0.56, 0.58), metallic=0.9, roughness=0.35),
    "lever": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.35),
    "magnet": m.Look(color=m.rgb(0.30, 0.30, 0.32), metallic=0.6, roughness=0.4),
    "rubber": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.9),
}


def _sector(r0, r1, z0, z1, a0, a1) -> m.Mesh:
    pts = []
    for a in (a0, a1):
        for r in (r0, r1):
            for z in (z0, z1):
                pts.append((r * math.cos(a), r * math.sin(a), z))
    return m.Mesh(np.array(pts), np.array([(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6),
                                           (0, 6, 4), (1, 5, 7), (1, 7, 3)]))


def _band(r, z0, z1, a0, a1, n=16) -> m.Mesh:
    """A curved strip of a cylinder's side (outward faces), from angle a0 to a1."""
    v, f = [], []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        v += [(r * math.cos(a), r * math.sin(a), z0), (r * math.cos(a), r * math.sin(a), z1)]
    for k in range(n):
        i = 2 * k
        f += [(i, i + 2, i + 3), (i, i + 3, i + 1)]
    return m.Mesh(np.array(v), np.array(f))


N_CUT = 36  # the cut line drawn on the lid in 10 degree pieces, on the turning body


def _can():
    # frame: the top seam bead and the lid (they stay in the opener's jaws while the body turns)
    seam = m.lathe([(R_SEAM_IN, Z_SEAM_BOT), (R_SEAM - 0.0004, Z_SEAM_BOT), (R_SEAM, Z_SEAM_BOT + 0.001), (R_SEAM, H_CAN - 0.001),
                    (R_SEAM - 0.0008, H_CAN), (R_SEAM_IN + 0.0006, H_CAN), (R_SEAM_IN, H_CAN - 0.0008), (R_SEAM_IN, Z_LID - 0.0003)], 64)
    lid = m.lathe([(0.0, Z_LID - 0.0008), (R_SEAM_IN, Z_LID - 0.0008), (R_SEAM_IN, Z_LID), (0.0315, Z_LID), (0.0305, Z_LID - 0.0006), (0.024, Z_LID - 0.0006),
                   (0.023, Z_LID), (0.012, Z_LID), (0.011, Z_LID - 0.0005), (0.0, Z_LID - 0.0005)], 64)
    food = m.lathe([(0.0, H_CAN - 0.0110), (R_BODY - 0.0006, H_CAN - 0.0110), (R_BODY - 0.0006, H_CAN - 0.0115), (0.0, H_CAN - 0.0115)], 48)
    rim_left = m.lathe([(R_CUT + 0.0006, Z_LID - 0.0008), (R_SEAM_IN, Z_LID - 0.0008), (R_SEAM_IN, Z_LID), (R_CUT + 0.0006, Z_LID)], 64)
    fparts = [("seam", seam, "tin"), ("lid", lid, "tin"), ("food", food, "food", True), ("rim_left", rim_left, "tin", True)]
    fcols = [m.col_hull(f"seam_{k:02d}", _sector(R_SEAM_IN, R_SEAM, Z_SEAM_BOT, H_CAN, 2 * math.pi * k / 24, 2 * math.pi * (k + 1) / 24 + 0.01))
             for k in range(24)]
    fcols.append(m.col_cylinder("lid_c", R_SEAM_IN, 0.0012, (0.0, 0.0, Z_LID - 0.0006)))
    # the body: wall, bottom seam, label (with a stripe and a panel, so that its turning shows), and the cut line, drawn as it is made
    wall = m.lathe([(0.0, 0.0), (R_SEAM - 0.0006, 0.0), (R_SEAM, 0.0008), (R_SEAM, SEAM_H - 0.0006), (R_SEAM - 0.0006, SEAM_H), (R_BODY, SEAM_H + 0.0004),
                    (R_BODY, Z_SEAM_BOT), (R_SEAM_IN - 0.0004, Z_SEAM_BOT), (0.0, Z_SEAM_BOT)], 64)
    bparts = [("wall", wall, "tin"), ("label", _band(R_BODY + 0.0004, 0.010, 0.101, 0.0, 2 * math.pi, 96), "label"),
              ("panel", _band(R_BODY + 0.0007, 0.030, 0.080, -0.6, 0.6, 24), "label2"),
              ("stripe", _band(R_BODY + 0.0009, 0.010, 0.101, 2.4, 2.75, 8), "stripe")]
    for k in range(N_CUT):
        a0, a1 = 2 * math.pi * k / N_CUT, 2 * math.pi * (k + 1) / N_CUT
        bparts.append((f"cut{k:02d}", _sector(R_CUT - 0.0006, R_CUT + 0.0006, Z_LID + 0.0001, Z_LID + 0.0005, a0, a1), "cutline", True))
    bcols = (m.col_cylinder("bottom_c", R_SEAM, SEAM_H, (0.0, 0.0, SEAM_H / 2)),
             m.col_cylinder("wall_c", R_BODY, Z_SEAM_BOT - SEAM_H, (0.0, 0.0, (Z_SEAM_BOT + SEAM_H) / 2)))
    spin = m.hinge("spin", "Z", (0.0, 0.0, 0.0), float("-inf"), float("inf"), bparts, bcols, mass=MASS_BODY, spring=0.0, damping=0.05, friction=0.0,
                   effort=MOTOR_TORQUE, gravity=True, com=(0.0, 0.0, 0.054), inertia=(5.7e-4, 5.7e-4, 2.9e-4))
    return fparts, fcols, spin


CAN_START = (0.27, -0.13)
_fp, _fc, _spin = _can()
CAN = m.mechanism("can", (CAN_START[0], CAN_START[1], TOP), [_spin], LOOKS, frame_parts=_fp, frame_colliders=_fc, friction=(1.0, 0.9), solver_iters=64,
                  free_mass=MASS_TOP, frame_inertia=(2.6e-5, 2.6e-5, 5.2e-5))


# -- the opener ------------------------------------------------------------------------------------------------------------------------
def _rounded_box(x0, x1, y0, y1, z0, z1, r=0.012, n=5) -> m.Mesh:
    pts = []
    for cx, cy, a0 in ((x1 - r, y1 - r, 0.0), (x0 + r, y1 - r, 90.0), (x0 + r, y0 + r, 180.0), (x1 - r, y0 + r, 270.0)):
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return m.prism(pts, z0, z1)


def _local(p) -> np.ndarray:
    return np.asarray(p, float) - ORIGIN


def _about_pivot(mesh: m.Mesh, theta: float) -> m.Mesh:
    """A mesh authored at lever angle theta, turned back to the lever's joint zero."""
    return mesh.moved(-P_LOCAL).turned(-math.degrees(theta), "y").moved(P_LOCAL)


def _opener():
    hw = W_OP / 2
    px, pz = P_LOCAL[0], P_LOCAL[2]
    fparts = [("pin", m.cylinder(0.003, 0.034, (px, 0.0, pz), axis="y"), "steel")]
    fcols = []
    for s in (-1.0, 1.0):  # the head's two cheeks that carry the lever's pivot
        size = (0.040 - px + 0.0, 0.006, 0.012)
        ctr = ((px - 0.007 + 0.033) / 2, s * 0.0125, pz + 0.0015)
        fparts.append((f"cheek{int(s > 0)}", m.box((0.040 - px, 0.006, 0.012), ((px - 0.007 + 0.033) / 2, s * 0.0125, pz + 0.0015)), "trim"))
        fcols.append(m.col_box(f"cheek{int(s > 0)}_c", size, ctr))
    # the lever, raised (joint zero): a handle up from the pivot, and an arm back to the cutting wheel
    hub = m.cylinder(0.0062, 0.017, (px, 0.0, pz), axis="y")
    handle = m.box((0.010, 0.016, HANDLE_L - 0.006), (px, 0.0, pz + 0.006 + (HANDLE_L - 0.006) / 2))
    knob = m.cylinder(0.008, 0.018, (px, 0.0, pz + HANDLE_L), axis="y")
    c0 = P_LOCAL + CUT_OFF
    arm = m.box((CUT_OFF[0], 0.008, 0.007), (px + CUT_OFF[0] / 2, 0.0, pz + 0.004))
    cut_pressed = _local(CUT_PRESSED)
    wheel = _about_pivot(m.cylinder(CUT_R, 0.0015, cut_pressed, axis="x"), THETA_P)
    axle = _about_pivot(m.cylinder(0.003, 0.004, cut_pressed + np.array([0.0025, 0.0, 0.0]), axis="x"), THETA_P)
    # the magnet sits on the lid between pivot and wheel when pressed home; the lid it will hold is drawn there, hidden
    mag_c = _local(SEAT + np.array([0.0175, 0.0, Z_LID + 0.0005 + 0.002]))
    magnet = _about_pivot(m.cylinder(0.0075, 0.004, mag_c, axis="z"), THETA_P)
    mag_arm = _about_pivot(m.box((0.004, 0.006, 0.012), mag_c + np.array([0.0, 0.0, 0.007])), THETA_P)
    lid_lathe = m.lathe([(0.0, Z_LID - 0.0008), (R_CUT, Z_LID - 0.0008), (R_CUT, Z_LID), (0.024, Z_LID), (0.023, Z_LID - 0.0006), (0.012, Z_LID - 0.0006),
                         (0.011, Z_LID), (0.0, Z_LID)], 48)
    maglid = _about_pivot(lid_lathe.moved(_local(SEAT) + np.array([0.0, 0.0, 0.0002])), THETA_P)
    lparts = [("hub", hub, "lever"), ("handle", handle, "lever"), ("knob", knob, "lever"), ("arm", arm, "lever"), ("wheel", wheel, "steel"),
              ("axle", axle, "steel"), ("magnet", magnet, "magnet"), ("mag_arm", mag_arm, "lever"), ("maglid", maglid, "tin", True)]
    lcols = (m.col_box("handle_c", (0.010, 0.016, HANDLE_L - 0.012), (px, 0.0, pz + 0.012 + (HANDLE_L - 0.012) / 2)),
             m.col_hull("wheel_c", wheel))
    lever = m.hinge("lever", "Y", tuple(P_LOCAL), 0.0, THETA_MAX, lparts, lcols, mass=0.06, spring=K_LEVER, damping=0.05, friction=0.0, effort=50.0,
                    inertia=(1.5e-4, 1.5e-4, 2e-5))
    _ = c0
    return fparts, fcols, lever


def _min_friction(part: m.Part) -> m.Part:
    """This part's friction wins over whatever touches it (PhysX "min" combine), so that a slippery surface stays slippery against
    the can's own higher friction, rather than being averaged with it."""
    build = part.build

    def slippery() -> str:
        return build().replace('prepend apiSchemas = ["PhysicsMaterialAPI"]\n    )\n    {',
                               'prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n    )\n    {\n'
                               '        uniform token physxMaterial:frictionCombineMode = "min"', 1)

    return dataclasses.replace(part, build=slippery)


LOWER_FRONT = -GEAR_OUT + 0.0022  # the body's face below the wheel: the can's bottom seam leans on it when the can hangs in the jaws


def _housing() -> m.Part:
    """The opener's body: glossy plastic, so a separate part with its own (lower) friction. Below the wheel its face stands forward,
    nearly flush with the wheel, and a hanging can's bottom seam leans on it; the chrome band that carries the wheel is set back."""
    hw = W_OP / 2
    pz = P_LOCAL[2]
    parts = [
        ("lower", _rounded_box(LOWER_FRONT, D_OP, -hw, hw, 0.006, GEAR_TOP - 2 * GEAR_R - 0.002, r=0.010), "body"),
        ("band", _rounded_box(0.0, D_OP, -hw, hw, GEAR_TOP - 2 * GEAR_R - 0.002, pz - 0.006, r=0.010), "trim"),
        ("upper", _rounded_box(0.036, D_OP, -hw, hw, pz - 0.006, H_OP, r=0.016), "body"),
        ("foot_l", m.box((D_OP - 0.02, 0.012, 0.006), (D_OP / 2 + 0.005, hw - 0.012, 0.003)), "rubber"),
        ("foot_r", m.box((D_OP - 0.02, 0.012, 0.006), (D_OP / 2 + 0.005, -hw + 0.012, 0.003)), "rubber"),
        ("sharpener", m.box((0.06, 0.004, 0.012), (D_OP - 0.045, 0.0, H_OP + 0.0005)), "rubber"),
    ]
    zl = GEAR_TOP - 2 * GEAR_R - 0.002
    cols = [m.col_box("lower_c", (D_OP - LOWER_FRONT, W_OP, zl - 0.006), ((LOWER_FRONT + D_OP) / 2, 0.0, (zl + 0.006) / 2)),
            m.col_box("band_c", (D_OP, W_OP, pz - 0.006 - zl), (D_OP / 2, 0.0, (zl + pz - 0.006) / 2)),
            m.col_box("upper_c", (D_OP - 0.036, W_OP, H_OP - pz + 0.006), ((0.036 + D_OP) / 2, 0.0, (pz - 0.006 + H_OP) / 2))]
    return _min_friction(m.fixed("housing", tuple(ORIGIN), parts, cols, LOOKS, friction=(0.05, 0.05)))


HOUSING = _housing()
_op, _oc, _lever = _opener()
OPENER = m.mechanism("opener", tuple(ORIGIN), [_lever], LOOKS, frame_parts=_op, frame_colliders=_oc, friction=(1.2, 1.0), solver_iters=64)


def _gear() -> m.Part:
    teeth = []
    for k in range(18):
        a = 2 * math.pi * k / 18
        teeth.append(m.box((GEAR_OUT, 0.0016, 0.0016), (-GEAR_OUT / 2, (GEAR_R - 0.0006) * math.cos(a), GEAR_TOP - GEAR_R + (GEAR_R - 0.0006) * math.sin(a)))
                     .moved((0.0, 0.0, 0.0)))
    parts = [("wheel", m.cylinder(GEAR_R - 0.0008, GEAR_OUT, (-GEAR_OUT / 2, 0.0, GEAR_TOP - GEAR_R), axis="x"), "steel"),
             ("teeth", m.join(*teeth), "steel"), ("hub", m.cylinder(0.004, 0.002, (-GEAR_OUT - 0.001, 0.0, GEAR_TOP - GEAR_R), axis="x"), "trim")]
    cols = [m.col_hull("gear_c", m.cylinder(GEAR_R, GEAR_OUT + 0.004, (-GEAR_OUT / 2 + 0.002, 0.0, GEAR_TOP - GEAR_R), axis="x"))]
    return _min_friction(m.fixed("gear", tuple(ORIGIN), parts, cols, LOOKS, friction=(0.1, 0.1)))


GEAR = _gear()

# -- distractors (SimReady): a mixing bowl, a mug, a dish cloth --------------------------------------------------------------------------
BOWL = SimReady("bowl", pos=(0.53, 0.22, TOP), size=(0.163, 0.163, 0.055), mass=0.35, package="dishware_bowl_a01", usd="sm_dishware_bowl_a01_01.usda")
MUG = SimReady("mug", pos=(0.56, -0.21, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.4)
CLOTH = SimReady("dish_cloth", pos=(0.20, 0.25, TOP), size=(0.260, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01", usd="sm_cleaning_cloth_a01_01.usd",
                 yaw=0.3)

OBJECTS = [HOUSING, OPENER, GEAR, CAN, BOWL, MUG, CLOTH]


# -- the scorer -------------------------------------------------------------------------------------------------------------------------
def seat_check(can, theta: float) -> dict:
    """Where the cutter's edge and the geared wheel's top are, in the can's frame."""
    q = can.to_local(cutter_low(theta))[0]
    g = can.to_local(GEAR_EDGE)[0]
    return {"cut_r": float(math.hypot(q[0], q[1])), "cut_dz": float(q[2] - Z_LID), "cut_ang": float(math.atan2(q[1], q[0])),
            "gear_r": float(math.hypot(g[0], g[1])), "gear_dz": float(g[2] - Z_SEAM_BOT)}


def seated(s: dict, tilt: float, slack: float = 1.0) -> bool:
    return bool(0.026 <= s["cut_r"] <= R_SEAM_IN - 0.0005 and abs(s["cut_dz"]) <= 0.0015 * slack and R_BODY - 0.001 * slack <= s["gear_r"] <= R_SEAM - 0.0003 + 0.0008 * (slack - 1)
                and -0.002 * slack <= s["gear_dz"] <= 0.001 * slack and tilt <= 8.0 * slack)


def hand_gap(can, tips) -> float:
    q = can.to_local(tips)
    dr = np.maximum(np.hypot(q[:, 0], q[:, 1]) - R_SEAM, 0.0)
    dz = np.maximum(q[:, 2] - H_CAN, 0.0) + np.maximum(-q[:, 2], 0.0)
    return float(np.min(np.hypot(dr, dz)))


TILT_HARM = 20.0
DROP = 0.03  # m within two steps (0.1 s): faster than 0.3 m/s down. Falling free out of the opener the can covers its 4 cm in 0.09 s
OMEGA = math.radians(45.0)  # the motor: one turn in 8 s
MOTOR_GAIN = 2.0  # N m per rad/s short of OMEGA: the motor slows under load and stalls at MOTOR_TORQUE


def _motor(env, speed: float) -> None:
    """Drive the can's body about its axis at `speed` (rad/s), as a speed-controlled motor with a torque limit; 0 stops it."""
    asset, j = m._joint(env, "can", "spin")
    asset.write_joint_damping_to_sim(m._tensor(asset, MOTOR_GAIN if speed else 0.05), joint_ids=[j])
    asset.set_joint_velocity_target(m._tensor(asset, speed), joint_ids=[j])


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place a recorded harm is cleared
        self.spilled = False
        self.dropped = False
        self.pierced = False
        self.done = False
        self.released = False
        self.lock_theta = 0.0
        self.s0 = 0.0  # the body's angle when pierced
        self.cut = 0.0  # radians cut
        self.start_ang = 0.0
        self.target = 0.0
        self.shown = 0
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        self.prev = None
        self.zs = []  # the base's height over the last steps
        self.yaw_prev = None
        self.spin_prev = None
        self.motor_on = False
        self.yaw_acc = 0.0  # the body's turn about its axis, as seen from the opener (unwrapped)
        self.yaw0 = 0.0
        self.max_tilt_after = 0.0
        if env is not None:
            m.set_target(env, "opener", "lever", PRELOAD)
            _motor(env, 0.0)
            m.lock(env, "can", "spin")
            self._visuals(0, False)

    def _visuals(self, shown: int, done: bool):
        """The lid on the can (sealed) or on the magnet (cut free), and the cut line hidden: `_cut_marks` draws it while cutting."""
        base = f"{m.ENV_PATH}/can"
        for k in range(N_CUT):
            m.set_visible(f"{base}/spin/visual/cut{k:02d}", False)
        m.set_visible(f"{base}/frame/visual/lid", not done)
        m.set_visible(f"{base}/frame/visual/food", done)
        m.set_visible(f"{base}/frame/visual/rim_left", done)
        m.set_visible(f"{m.ENV_PATH}/opener/lever/visual/maglid", done)

    def _clamp(self, env):
        """The latch: the lever's drive now presses the cutter onto the lid, toward the lever's stop, with at most E_CLAMP."""
        m.lock(env, "opener", "lever", stiffness=K_CLAMP, effort=E_CLAMP)
        m.set_target(env, "opener", "lever", math.radians(THETA_MAX))

    def _cut_marks(self, spin: float):
        """Draw the cut line, on the turning body, from where it was pierced back round as far as the body has turned past the cutter."""
        base = f"{m.ENV_PATH}/can/spin/visual"
        start = self.start_ang  # the body's angle under the cutter when pierced
        for k in range(N_CUT):
            mid = 2 * math.pi * (k + 0.5) / N_CUT
            behind = (start - mid) % (2 * math.pi)  # how far the body had to turn to bring this piece under the cutter
            m.set_visible(f"{base}/cut{k:02d}", (not self.done) and behind <= self.cut)

    def step(self, env):
        can, op = m.state(env, "can"), m.state(env, "opener")
        theta = op.joint("lever")
        spin = can.joint("spin")
        tilt = can.tilt_deg()
        s = seat_check(can, theta)
        c = {"theta": math.degrees(theta), "spin": math.degrees(spin), "tilt": tilt, **s}
        bx = can.rotation() @ np.array([math.cos(spin), math.sin(spin), 0.0])  # the body's x axis in the world
        yaw = math.atan2(bx[1], bx[0])
        if self.yaw_prev is not None:
            self.yaw_acc += (yaw - self.yaw_prev + math.pi) % (2 * math.pi) - math.pi
        self.yaw_prev = yaw
        if self.spin_prev is None:
            self.spin_prev = spin
        if not self.pierced and theta >= THETA_P - math.radians(PRESS_TOL) and seated(s, tilt):
            self.pierced = True
            self._clamp(env)
            self.lock_theta = theta
            self.s0 = spin
            self.yaw0 = self.yaw_acc
            self.start_ang = s["cut_ang"] - spin
            m.unlock(env, "can", "spin")
            _motor(env, OMEGA)
            self.motor_on = True
        if self.pierced and not self.done:
            on = seated(s, tilt, slack=2.0)  # out of the jaws, the motor stops turning it
            if on != self.motor_on:
                _motor(env, OMEGA if on else 0.0)
                self.motor_on = on
            self.cut = max(self.cut, self.yaw_acc - self.yaw0)  # the rim that has passed the cutter: the body's turn as seen from the opener
            self._cut_marks(spin)
            if self.cut >= 2 * math.pi:
                self.done = True
                _motor(env, 0.0)
                m.lock(env, "can", "spin")
                self.lock_theta = theta
                self._visuals(0, True)
                self._cut_marks(spin)
        if self.done and not self.released and theta < self.lock_theta - math.radians(6.0):
            self.released = True
            m.unlock(env, "opener", "lever")
            m.set_target(env, "opener", "lever", PRELOAD)
        if self.pierced:
            self.max_tilt_after = max(self.max_tilt_after, tilt)
            if tilt > TILT_HARM:
                self.spilled = True
            if len(self.zs) >= 2 and self.zs[-2] - can.pos[2] > DROP:  # fell: its base came down 3 cm within 0.1 s
                self.dropped = True
                self.spilled = True
        self.zs = (self.zs + [float(can.pos[2])])[-3:]
        self.spin_prev = spin
        # stillness from poses, step to step
        now = np.r_[can.pos, spin]
        prev = self.prev if self.prev is not None else now
        still = bool(np.max(np.abs(now[:3] - prev[:3])) < 0.0005 and abs((now[3] - prev[3] + math.pi) % (2 * math.pi) - math.pi) < 0.01)
        self.prev = now
        gap = hand_gap(can, m.fingertips(env))
        c.update({"cut": math.degrees(self.cut), "floor": float(can.pos[2] - TOP), "still": still, "hands": gap, "pierced": self.pierced, "done": self.done,
                  "released": self.released, "spilled": self.spilled, "dropped": self.dropped})
        self.last = c
        ok = (not self.spilled and self.done and math.degrees(theta) <= 10.0 and tilt <= 10.0 and c["floor"] <= 0.003 and still and gap >= 0.05)
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= HOLD_STEPS:
            self.full = True
        if self.full and not ok and (self.spilled or not (self.done and tilt <= 10.0 and c["floor"] <= 0.003 and math.degrees(theta) <= 10.0)):
            self.full = False  # credit does not outlast the state that earned it
        if self.spilled or not self.pierced:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif self.done:
            self.score = 0.75
        elif self.cut >= math.radians(270.0):
            self.score = 0.5
        else:
            self.score = 0.25


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration ------------------------------------------------------------------------------------------------------------------
DISTRACTORS = ("bowl", "mug", "dish_cloth")


def _report(ctx, label=""):
    c = TRIAL.last
    if label.startswith(("end", "start")):
        print("[distractors] " + ", ".join(f"{n} {np.round(ctx.pos(n)[:2] - np.array(o.pos[:2]), 4).tolist()}" for n, o in zip(DISTRACTORS, (BOWL, MUG, CLOTH))),
              flush=True)
    if not c:
        return
    print(f"[can] t {ctx.t:5.1f} {label} lever {c['theta']:5.1f} spin {c['spin']:7.1f} cut {c['cut']:6.1f} tilt {c['tilt']:5.1f} floor {c['floor'] * 1000:6.1f} | "
          f"cut_r {c['cut_r'] * 1000:5.1f} cut_dz {c['cut_dz'] * 1000:+5.1f} gear_r {c['gear_r'] * 1000:5.1f} gear_dz {c['gear_dz'] * 1000:+5.1f} | "
          f"hands {c['hands'] * 1000:5.0f} still {c['still']} | pierced {c['pierced']} done {c['done']} released {c['released']} spilled {c['spilled']} dropped {c['dropped']} | "
          f"score {TRIAL.score:.2f} full {TRIAL.full} | can {np.round(ctx.pos('can'), 4).tolist()} R {np.round(ctx.right.tcp(), 4).tolist()} L {np.round(ctx.left.tcp(), 4).tolist()}", flush=True)


def demo_script(ctx):
    """Right: takes the can round its upper body from the front (fingers pointing forward and down, closing across it), lifts it, brings it
    in front of the opener with the bead just above the geared wheel's top, and slides it back under the cutter until the wall is at the
    wheel. Left: holds the lever's grip from the side and swings it down about its pivot; the cutter pushes the bead onto the wheel, pierces
    the lid, and the lever latches. Right lets go and keeps its open hand round the can while the motor turns it (a held can would stall
    it). Once the can has stopped turning, right takes the can again, left lifts the lever (the lid comes up on the magnet), right draws
    the can forward off the wheel, stands it on the counter and lets go, and both arms move clear.
    Red-team modes: none (nothing), onearm (right alone: hooks the can on the wheel, lets go and presses the lever itself), careless (left
    lifts the lever with nothing holding the can), clamp (right keeps hold of the can while it is cut), high (the bead held 5 mm high)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "clamp", "high"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"start {mode}:")
    if mode == "none":
        for k in range(160):
            yield
            if k % 40 == 39:
                _report(ctx, "end:")
        return
    up = np.array([0.0, 0.0, 1.0])
    ex = np.array([1.0, 0.0, 0.0])
    ey = np.array([0.0, 1.0, 0.0])
    g = d.tilted(60.0, 0.0)  # fingers pointing forward and 30 degrees down, closing across the can (y)
    hold_h = 0.075  # TCP above the can's base
    can0 = ctx.pos("can")
    pivot = PIVOT.copy()
    handle = pivot + np.array([0.0, 0.0, GRIP_H])
    gl = d.level(-90.0)  # the left hand on the lever: fingers pointing right, closing fore and aft on the grip
    canp = lambda: ctx.pos("can")  # noqa: E731

    def spin():
        return ctx.joint("can", "spin")

    # 1. take the can; the left hand goes beside the lever's grip
    left_go = l.move(handle + ey * 0.10, grip=gl, finger=FINGER_OPEN, t=2.5, label="left beside the lever") if mode != "onearm" else l.wait(0.1)
    yield from ctx.together(r.move(can0 + up * (hold_h + 0.07) - ex * 0.10, grip=g, finger=FINGER_OPEN, t=2.5, label="right before the can"), left_go)
    yield from r.move(can0 + up * hold_h - ex * 0.06, t=1.0, label="right down")
    yield from r.move(can0 + up * hold_h, t=1.2, label="right round the can")
    yield from r.servo(can0 + up * hold_h, timeout=1.0, label="right round the can")
    yield from r.close(2 * R_BODY, 35.0, t=0.6, label="hold the can")
    yield from r.move(r.at() + up * 0.07, t=1.0, label="lift the can")
    _report(ctx, "can lifted:")

    def lean():
        """The can's lean (deg): its axis's tilt toward +x and toward +y, as seen."""
        z = m.quat_matrix(ctx.state("can").quat_xyzw)[:, 2]
        return math.degrees(math.atan2(z[0], z[2])), math.degrees(math.atan2(z[1], z[2]))

    for _ in range(3):  # a two-finger pinch lets the can swing a little about the line between the pads: turn the hand to stand it upright
        ax, ay = lean()
        if abs(ax) < 0.7 and abs(ay) < 0.7:
            break
        yield from r.turn(-ax, axis=(0.0, 1.0, 0.0), t=max(0.4, abs(ax) / 20.0), label="stand the can upright")
        if abs(ay) >= 0.7:
            yield from r.turn(ay, axis=(1.0, 0.0, 0.0), t=max(0.4, abs(ay) / 20.0), label="stand the can upright")
        yield from r.wait(0.3)
    _report(ctx, "upright:")
    # 2. the bead just above the wheel's top, in front of it; then back under the cutter to the wheel
    rise = 0.001 if mode != "high" else 0.005
    pre = SEAT + np.array([-0.014, 0.0, rise])
    off = r.at() - canp()
    yield from r.move(pre + off - ex * 0.06 + up * 0.02, t=2.0, label="can before the opener")
    yield from r.move(pre + off - ex * 0.06, t=0.8, label="bead level with the wheel")
    yield from r.servo(pre - ex * 0.06, read=canp, timeout=1.5, label="bead level with the wheel")
    _report(ctx, "bead level:")
    yield from r.move(r.at() + ex * 0.06, t=1.5, label="in under the cutter")
    yield from r.servo(pre, read=canp, tol=0.001, timeout=1.5, label="in under the cutter")
    yield from r.move(r.at() + ex * 0.0136, t=1.0, label="wall to the wheel")
    yield from r.servo(SEAT + np.array([0.0006, 0.0, rise]), read=canp, tol=0.0008, timeout=2.0, label="wall to the wheel")
    _report(ctx, "at the wheel:")
    if mode == "onearm":
        # right lets go, hoping the can stays hooked, and goes for the lever itself
        yield from r.move(r.at() - up * (rise + 0.001), t=0.6, label="bead onto the wheel")
        yield from r.open(t=0.4, label="right lets go")
        yield from r.move(r.at() - ex * 0.10, t=1.0, label="right back")
        _report(ctx, "let go:")
        yield from r.move(handle - ex * 0.10, grip=d.level(0.0), t=2.0, label="right before the lever")
        yield from r.move(handle, t=1.2, label="right on the lever")
        yield from r.close(0.016, 15.0, t=0.5, label="right holds the lever")
        yield from r.arc(pivot, (0.0, 1.0, 0.0), 34.0, t=2.0, label="right presses the lever")
        yield from r.wait(1.0)
        _report(ctx, "pressed:")
        yield from r.arc(pivot, (0.0, 1.0, 0.0), -34.0, t=2.0, label="right lets the lever up")
        yield from r.open(t=0.4)
        yield from r.move(r.at() - ex * 0.10, t=1.2, label="right away")
        for k in range(60):
            yield
            if k % 20 == 19:
                _report(ctx, "end:")
        return
    # 3. left takes the lever's grip and presses it home
    yield from l.move(handle, t=1.2, label="left on the lever")
    yield from l.servo(handle, timeout=0.8, label="left on the lever")
    yield from l.close(0.010, 20.0, t=0.5, label="left holds the lever")
    yield from l.arc(pivot, (0.0, 1.0, 0.0), 34.0, t=2.0, label="press the lever")
    yield from l.wait(0.6)
    _report(ctx, "lever pressed:")
    # 4. the motor turns the can: right opens its hand round it, unless it is the clamp cheat
    if mode != "clamp":
        yield from r.open(t=0.4, label="right lets the can turn")
    def turned():
        """The can's label's turn about its axis (the body's joint, as seen)."""
        return ctx.joint("can", "spin")

    acc = [0.0, turned(), 0]

    def stopped():
        now = turned()
        d_ = (now - acc[1] + math.pi) % (2 * math.pi) - math.pi
        acc[0] += d_
        acc[1] = now
        acc[2] = acc[2] + 1 if abs(d_) < 0.002 else 0  # the label has stopped turning
        if int(round(ctx.t * 20)) % 60 == 0:
            _report(ctx, "cutting:")
        return (acc[0] > 5.0 and acc[2] >= 10) or (mode == "clamp" and ctx.t > 40.0)

    yield from ctx.until(stopped, timeout=25.0, label="the motor stops")
    _report(ctx, "stopped:")
    # 5. right takes the can again; left lifts the lever
    if mode != "careless":
        yield from r.close(2 * R_BODY, 40.0, t=0.6, label="right takes the can")
    else:
        yield from r.move(r.at() - ex * 0.10, t=1.0, label="right away from the can")
    yield from l.arc(pivot, (0.0, 1.0, 0.0), -34.0, t=2.0, label="lift the lever")
    yield from l.wait(0.4)
    _report(ctx, "lever lifted:")
    yield from l.open(t=0.4, label="left lets go of the lever")
    yield from l.move(l.at() + ey * 0.10, t=1.0, label="left out to the side")
    yield from l.move(np.array([0.22, 0.22, TOP + 0.20]), grip=d.down(0.0), t=1.5, label="left away")
    if mode == "careless":
        for k in range(60):
            yield
            if k % 20 == 19:
                _report(ctx, "end:")
        return
    # 6. right draws the can off the wheel and stands it on the counter
    yield from r.move(r.at() - ex * 0.02 + up * 0.002, t=0.8, label="can off the wheel")
    _report(ctx, "off the wheel:")
    for _ in range(3):
        ax, ay = lean()
        if abs(ax) < 0.7 and abs(ay) < 0.7:
            break
        yield from r.turn(-ax, axis=(0.0, 1.0, 0.0), t=max(0.4, abs(ax) / 20.0), label="stand the can upright")
        if abs(ay) >= 0.7:
            yield from r.turn(ay, axis=(1.0, 0.0, 0.0), t=max(0.4, abs(ay) / 20.0), label="stand the can upright")
        yield from r.wait(0.3)
    down_at = np.array([0.29, -0.08, TOP])
    off = r.at() - canp()
    yield from r.move(down_at + off + up * 0.02, t=2.0, label="can over the counter")
    yield from r.move(down_at + off + up * 0.003, t=1.0, label="can down")
    yield from r.servo(down_at + up * 0.001, read=canp, timeout=0.8, label="can down")
    yield from r.open(t=0.5, label="right lets go")
    yield from r.move(r.at() + up * 0.07, t=1.2, label="right up off the can")
    yield from r.move(r.at() - ex * 0.08, t=1.0, label="right back")
    yield from r.move(np.array([0.18, -0.22, TOP + 0.18]), grip=d.down(0.0), t=1.5, label="right away")
    _report(ctx, "clear:")
    for k in range(40):
        yield
        if k % 20 == 19:
            _report(ctx, "end:")
