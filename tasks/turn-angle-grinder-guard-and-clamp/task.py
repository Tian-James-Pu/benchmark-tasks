"""A 125 mm cordless angle grinder lies upside down on the bench, battery and disc off, its spindle pointing up: the way its
manual says to set a grinder down for work on the spindle. Its tool-less wheel guard has been put back with the closed side
of the hood towards the front of the tool, away from where the operator holds it, and the clamp lever is shut on it there.
The guard has to be turned half a turn and clamped again before the new disc goes on.

Model (all in this file; dimensions in the README):
- One free articulation, "grinder" (2.0 kg): its frame is the body (gear case, bearing box and spindle, the motor barrel,
  the battery mount, the side handle). Part frame: origin on the bench under the spindle axis, +x towards the front of the
  tool (away from the motor), +z up the spindle, so the body's back rests on the bench.
- "guard": the wheel guard (band, half-annulus plate, half skirt, the clamp's two lugs and screw) on a hinge about the
  spindle axis. Joint zero is the start: the hood's closed side centred on +x (the front). The hood is set for grinding
  when its closed side is centred on -x, towards the motor and the operator's hands: guard angle +-180 degrees.
- "lever": the clamp lever, hinged on the band's lugs about an axis parallel to the spindle, lying along the band when shut
  and swinging outwards to open (negative joint angle). Its free end is bent out into a finger tab.
- The clamp (task code, each control step): while the lever is shut (within LOCK_DEG of the band) the guard joint is
  held where it is, as the tightened band holds it; once the lever is opened past UNLOCK_DEG it is free, against the drag
  of the loose band. The lever is over-centre: a spring of K_LEVER pulls it shut below SNAP_DEG and pushes it open between
  SNAP_DEG and LOOSE_DEG; beyond LOOSE_DEG it is loose.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 60  # 3 s at 20 Hz
FINGER_OPEN, FINGER_CLOSED = -0.04695, 0.0

# -- placement ------------------------------------------------------------------------------------------------------------
SPINDLE_XY = np.array([0.27, 0.02])  # world xy of the spindle axis
YAW_DEG = 90.0  # part +x (front) -> world +y; part -x (motor, rear) -> world -y; part -y (side handle) -> world +x

# -- the body (part frame, metres). Gear case back on the bench at z = 0. --------------------------------------------------
GC_X0, GC_X1, GC_HW, GC_H = -0.045, 0.032, 0.031, 0.064  # gear case: rear and front faces, half width, height
BB_R, BB_TOP = 0.0225, 0.078  # bearing box (the collar the guard band clamps on)
SP_R, SP_TOP = 0.0079, 0.098  # spindle, 5/8"-11
BAR_R, BAR_Z, BAR_X0, BAR_X1 = 0.030, 0.031, -0.235, -0.040  # motor barrel along -x
MNT_X0, MNT_X1, MNT_HW, MNT_H = -0.275, -0.235, 0.0375, 0.058  # battery mount (battery removed)
HDL_R, HDL_Y0, HDL_Y1, HDL_Z = 0.016, -0.031, -0.131, 0.030  # side handle, screwed into the gear case's side

# -- the guard (guard frame = part frame turned by the guard angle about the spindle axis) ---------------------------------
BAND_RI, BAND_RO, BAND_Z0, BAND_Z1 = 0.0225, 0.0237, 0.0655, 0.0775
PLATE_RO, PLATE_Z1 = 0.0700, 0.0790  # plate: BAND_RO..PLATE_RO, BAND_Z1..PLATE_Z1, over the closed half
SKIRT_RI, SKIRT_RO, SKIRT_Z1 = 0.0686, 0.0700, 0.1060  # skirt: PLATE_Z1..SKIRT_Z1
HALF = 90.0  # the hood covers bearings -HALF..HALF of the guard frame (its closed side)
LUG_DEG = (94.0, 101.0)  # the band's two clamp lugs (bearing of each)
PIVOT_DEG, PIVOT_R = 100.0, 0.0300  # lever pivot pin
LEV_RI, LEV_RO, LEV_Z0, LEV_Z1 = 0.0242, 0.0267, 0.0660, 0.0780  # lever strip, lying along the band when shut
LEV_END_DEG = 158.0  # where the strip ends and the tab is bent out
TAB_DEG, TAB_R0, TAB_R1, TAB_T = 159.0, 0.0242, 0.0520, 0.002  # finger tab: radial, 2 mm sheet

# -- the clamp ------------------------------------------------------------------------------------------------------------
LOCK_DEG, UNLOCK_DEG = 3.0, 12.0  # the band holds the guard until the lever passes its over-centre point (12) and grips again once it is back within 3
SNAP_DEG, LOOSE_DEG, OPEN_MAX = 12.0, 30.0, 45.0
K_LEVER = 1.2  # N m/rad: about 0.25 N m to open past the snap, 0.38 N m to close past it (6 to 9 N at the tab)
GUARD_DRAG = 0.12  # N m: the loose band's drag on the bearing box (an estimate)

# -- scoring --------------------------------------------------------------------------------------------------------------
TARGET_DEG = 180.0
FULL_DEG, PART_DEG = 30.0, 60.0
SHUT_DEG = 2.0  # "shut": the lever back against the band
CLEAR_M = 0.03  # every fingertip point this far from the grinder
TILT_MAX = 15.0  # still lying on its back
FELL_DROP = 0.05  # the grinder's origin this far below the bench top: it fell off


def _arc_solid(r0, r1, z0, z1, a0, a1, n=24) -> m.Mesh:
    """A closed annular sector (r0..r1, z0..z1, bearings a0..a1 degrees)."""
    full = abs(a1 - a0) >= 359.999
    t = np.radians(np.linspace(a0, a1, n + 1))
    if full:
        t = t[:-1]
    k = len(t)
    v = []
    for r, z in ((r0, z0), (r1, z0), (r1, z1), (r0, z1)):
        v += [(r * math.cos(a), r * math.sin(a), z) for a in t]
    v = np.array(v)
    f = []
    rings = [0, 1, 2, 3]  # inner-bottom, outer-bottom, outer-top, inner-top
    segs = range(k) if full else range(k - 1)
    for s in segs:
        a, b = s, (s + 1) % k
        for i in range(4):
            p, q = rings[i] * k, rings[(i + 1) % 4] * k
            f += [(p + a, p + b, q + b), (p + a, q + b, q + a)]
    if not full:
        for e, sgn in ((0, -1), (k - 1, 1)):
            idx = [r * k + e for r in rings]
            tri = [(idx[0], idx[1], idx[2]), (idx[0], idx[2], idx[3])]
            f += [t_[::-1] if sgn > 0 else t_ for t_ in tri]
    return m.Mesh(v, np.array(f, dtype=int))


def _turned_box(size, centre_r, bearing_deg, z_mid) -> m.Mesh:
    """A box whose x is radial at `bearing_deg`, centred `centre_r` from the spindle axis."""
    return m.box(size, (centre_r, 0.0, z_mid)).turned(bearing_deg, "z")


def _rot(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


PIVOT = PIVOT_R * np.array([math.cos(math.radians(PIVOT_DEG)), math.sin(math.radians(PIVOT_DEG))])

# -- looks ----------------------------------------------------------------------------------------------------------------
LOOKS = {
    "teal": m.Look(m.rgb(0.0, 0.55, 0.58), roughness=0.5),
    "black": m.Look(m.rgb(0.06, 0.06, 0.06), roughness=0.55),
    "rubber": m.Look(m.rgb(0.03, 0.03, 0.03), roughness=0.85),
    "gearcase": m.Look(m.rgb(0.36, 0.37, 0.39), metallic=0.6, roughness=0.45),
    "alu": m.Look(m.rgb(0.62, 0.63, 0.65), metallic=0.8, roughness=0.35),
    "steel": m.Look(m.rgb(0.55, 0.56, 0.58), metallic=0.9, roughness=0.3),
    "guard": m.Look(m.rgb(0.07, 0.07, 0.08), metallic=0.4, roughness=0.45),
    "zinc": m.Look(m.rgb(0.75, 0.76, 0.78), metallic=0.9, roughness=0.3),
}


def _rounded_rect(x0, x1, hw, rad, n=6):
    pts = []
    for cx, cy, a0 in ((x1 - rad, -hw + rad, -90), (x1 - rad, hw - rad, 0), (x0 + rad, hw - rad, 90), (x0 + rad, -hw + rad, 180)):
        for a in np.linspace(a0, a0 + 90, n):
            pts.append((cx + rad * math.cos(math.radians(a)), cy + rad * math.sin(math.radians(a))))
    return pts


def _body_parts():
    gear = m.prism(_rounded_rect(GC_X0, GC_X1, GC_HW, 0.012), 0.0, GC_H)
    boss = m.cylinder(0.0275, 0.004, (0, 0, GC_H - 0.002), segments=40)  # the bearing box's flange on the gear case
    bbox = m.cylinder(BB_R - 0.0002, BB_TOP - GC_H, (0, 0, (GC_H + BB_TOP) / 2), segments=40)
    spindle = m.cylinder(SP_R, SP_TOP - BB_TOP, (0, 0, (BB_TOP + SP_TOP) / 2), segments=20)
    prof = [(0.0, BAR_X0), (0.027, BAR_X0), (0.030, BAR_X0 + 0.012), (0.030, BAR_X1 - 0.015), (0.028, BAR_X1), (0.0, BAR_X1)]
    barrel = m.lathe(prof, segments=40).aimed("+X").moved((0, 0, BAR_Z))  # lathe runs along +z; aimed turns +z to +x
    # (lathe's z values are BAR_X0..BAR_X1 along x after aiming, which keeps them as x positions)
    vent = m.box((0.06, 0.044, 0.002), (-0.10, 0.0, BAR_Z + BAR_R - 0.0005))
    mount = m.prism(_rounded_rect(MNT_X0, MNT_X1, MNT_HW, 0.008), 0.0, MNT_H)
    rails = m.join(m.box((0.04, 0.004, 0.006), (-0.255, 0.024, MNT_H + 0.003)), m.box((0.04, 0.004, 0.006), (-0.255, -0.024, MNT_H + 0.003)))
    handle_stud = m.cylinder(0.006, 0.012, (0, 0, 0)).aimed("-Y").moved((0, HDL_Y0, HDL_Z))
    handle = m.lathe([(0.0, 0.0), (0.012, 0.0), (0.016, 0.012), (0.017, 0.07), (0.019, 0.09), (0.019, 0.10), (0.0, 0.10)], segments=28)
    handle = handle.aimed("-Y").moved((0, HDL_Y0 - 0.008, HDL_Z))
    return [("gearcase", gear, "gearcase"), ("flange", boss, "alu"), ("bearing_box", bbox, "alu"), ("spindle", spindle, "steel"),
            ("barrel", barrel, "teal"), ("vent", vent, "black"), ("mount", mount, "black"), ("rails", rails, "black"),
            ("handle_stud", handle_stud, "steel"), ("side_handle", handle, "rubber")]


def _body_colliders():
    return [
        m.col_box("gearcase_c", (GC_X1 - GC_X0, 2 * GC_HW, GC_H), ((GC_X0 + GC_X1) / 2, 0, GC_H / 2)),
        m.col_cylinder("bearing_box_c", BB_R, BB_TOP - GC_H, (0, 0, (GC_H + BB_TOP) / 2)),
        m.col_cylinder("spindle_c", SP_R, SP_TOP - BB_TOP, (0, 0, (BB_TOP + SP_TOP) / 2)),
        m.col_hull("barrel_c", m.cylinder(BAR_R, BAR_X1 - BAR_X0, ((BAR_X0 + BAR_X1) / 2, 0, BAR_Z), axis="x", segments=32)),
        m.col_box("mount_c", (MNT_X1 - MNT_X0, 2 * MNT_HW, MNT_H), ((MNT_X0 + MNT_X1) / 2, 0, MNT_H / 2)),
        m.col_hull("handle_c", m.cylinder(HDL_R + 0.002, 0.10, (0, HDL_Y0 - 0.058, HDL_Z), axis="y", segments=24)),
    ]


def _guard_parts():
    band = _arc_solid(BAND_RI, BAND_RO, BAND_Z0, BAND_Z1, LUG_DEG[1] + 1.0, LUG_DEG[0] - 1.0 + 360.0, 60)
    plate = _arc_solid(BAND_RO - 0.0004, PLATE_RO, BAND_Z1, PLATE_Z1, -HALF, HALF, 48)
    skirt = _arc_solid(SKIRT_RI, SKIRT_RO, PLATE_Z1 - 0.0004, SKIRT_Z1, -HALF, HALF, 72)
    zm = (BAND_Z0 + BAND_Z1) / 2
    lugs = m.join(*[_turned_box((0.010, 0.003, BAND_Z1 - BAND_Z0), BAND_RO + 0.004, a, zm) for a in LUG_DEG])
    screw_axis = np.array([-math.sin(math.radians(97.5)), math.cos(math.radians(97.5)), 0.0])
    sc = (BAND_RO + 0.0055) * np.array([math.cos(math.radians(97.5)), math.sin(math.radians(97.5)), 0.0]) + np.array([0, 0, zm])
    screw = m.tube([sc - 0.009 * screw_axis, sc + 0.007 * screw_axis], 0.0022, sides=10)
    head = m.tube([sc - 0.012 * screw_axis, sc - 0.009 * screw_axis], 0.0045, sides=14)
    return [("band", band, "guard"), ("plate", plate, "guard"), ("skirt", skirt, "guard"), ("lugs", lugs, "guard"),
            ("screw", m.join(screw, head), "zinc")]


def _guard_colliders():
    cols = []
    for i, a in enumerate(np.arange(-HALF, HALF, 15.0)):
        cols.append(m.col_hull(f"skirt_c{i}", _arc_solid(SKIRT_RI - 0.0003, SKIRT_RO + 0.0003, PLATE_Z1, SKIRT_Z1, a, a + 15.0, 3)))
    for i, a in enumerate(np.arange(-HALF, HALF, 30.0)):
        cols.append(m.col_hull(f"plate_c{i}", _arc_solid(BAND_RO, PLATE_RO, BAND_Z1, PLATE_Z1, a, a + 30.0, 4)))
    for i, a in enumerate(np.arange(LUG_DEG[1] + 1.0, LUG_DEG[0] - 1.0 + 360.0, 30.0)):
        cols.append(m.col_hull(f"band_c{i}", _arc_solid(BAND_RI, BAND_RO + 0.0005, BAND_Z0, BAND_Z1, a, min(a + 30.0, LUG_DEG[0] + 359.0), 4)))
    return cols


def _lever_parts():
    strip = _arc_solid(LEV_RI, LEV_RO, LEV_Z0, LEV_Z1, PIVOT_DEG, LEV_END_DEG, 16)
    zm = (LEV_Z0 + LEV_Z1) / 2
    tab = _turned_box((TAB_R1 - TAB_R0, TAB_T, LEV_Z1 - LEV_Z0), (TAB_R0 + TAB_R1) / 2, TAB_DEG, zm)
    eye = m.cylinder(0.0042, LEV_Z1 - LEV_Z0, (PIVOT[0], PIVOT[1], zm), segments=16)
    pin = m.cylinder(0.0018, LEV_Z1 - LEV_Z0 + 0.003, (PIVOT[0], PIVOT[1], zm), segments=10)
    return [("strip", m.join(strip, eye), "black"), ("tab", tab, "black"), ("pin", pin, "zinc")]


def _lever_colliders():
    zm = (LEV_Z0 + LEV_Z1) / 2
    cols = [m.col_hull(f"strip_c{i}", _arc_solid(LEV_RI, LEV_RO, LEV_Z0, LEV_Z1, a, a + 20.0 if a + 20.0 < LEV_END_DEG else LEV_END_DEG, 3))
            for i, a in enumerate(np.arange(PIVOT_DEG, LEV_END_DEG, 20.0))]
    cols.append(m.col_hull("tab_c", _turned_box((TAB_R1 - TAB_R0, TAB_T, LEV_Z1 - LEV_Z0), (TAB_R0 + TAB_R1) / 2, TAB_DEG, zm)))
    return cols


GUARD = m.hinge("guard", "Z", (0.0, 0.0, BAND_Z0), -math.inf, math.inf, _guard_parts(), _guard_colliders(), mass=0.20, spring=0.0,
                damping=0.02, friction=GUARD_DRAG, armature=0.002, inertia=(6e-4, 6e-4, 8e-4), com=(0.0, 0.0, 0.085))
LEVER = m.hinge("lever", "Z", (float(PIVOT[0]), float(PIVOT[1]), LEV_Z0), -OPEN_MAX, 0.0, _lever_parts(), _lever_colliders(), mass=0.02,
                spring=K_LEVER, damping=0.02, friction=0.005, effort=20.0, armature=0.001, parent="guard", inertia=(4e-6, 4e-6, 8e-6),
                com=(float(PIVOT[0]) - 0.012, float(PIVOT[1]) - 0.012, 0.072))

GRINDER = m.mechanism("grinder", (float(SPINDLE_XY[0]), float(SPINDLE_XY[1]), TOP), [GUARD, LEVER], LOOKS, frame_parts=_body_parts(),
                      frame_colliders=_body_colliders(), yaw_deg=YAW_DEG, friction=(0.8, 0.7), solver_iters=48, free_mass=2.0,
                      frame_inertia=(0.0012, 0.016, 0.016))

# -- distractors: the protective kit kept at a grinding bench (SimReady, scoring ignores them) ------------------------------
GLASSES = SimReady("safety_glasses", pos=(0.46, -0.20, TOP), size=(0.167, 0.169, 0.060), mass=0.03,
                   package="ppe_eyeProtection_safetyGlasses_a01", usd="sm_ppe_eyeProtection_safetyGlasses_a01_01.usd", yaw=0.4)
GLOVES = SimReady("work_gloves", pos=(0.58, 0.08, TOP), size=(0.247, 0.231, 0.016), mass=0.08,
                  package="ppe_gloves_lightDuty_a01", usd="sm_ppe_gloves_lightDuty_a01_01.usd", yaw=1.2)
EARMUFFS = SimReady("earmuffs", pos=(0.50, 0.27, TOP), size=(0.097, 0.250, 0.212), mass=0.25,
                    package="ppe_hearingProtection_earMuffs_a01", usd="sm_ppe_hearingProtection_earMuffs_a01_01.usd", yaw=0.3)

OBJECTS = [GRINDER, GLASSES, GLOVES, EARMUFFS]


# -- geometry used by the scorer ------------------------------------------------------------------------------------------
def _ring_points(r, z, a0, a1, step):
    return np.array([(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), z) for a in np.arange(a0, a1 + 1e-6, step)])


GUARD_PTS = np.vstack(
    [_ring_points(SKIRT_RO, z, -HALF, HALF, 6.0) for z in (PLATE_Z1, 0.092, SKIRT_Z1)]
    + [_ring_points(r, PLATE_Z1, -HALF, HALF, 10.0) for r in (0.035, 0.050)]
    + [_ring_points(BAND_RO, z, 0.0, 359.0, 12.0) for z in (BAND_Z0, BAND_Z1)]
)
LEVER_PTS = np.vstack(
    [_ring_points(LEV_RO, z, PIVOT_DEG, LEV_END_DEG, 6.0) for z in (LEV_Z0, LEV_Z1)]
    + [np.array([(r * math.cos(math.radians(TAB_DEG)), r * math.sin(math.radians(TAB_DEG)), z) for r in np.linspace(TAB_R0, TAB_R1, 5) for z in (LEV_Z0, LEV_Z1)])]
)


def _body_pts():
    pts = []
    for x in np.linspace(GC_X0, GC_X1, 6):
        for y in np.linspace(-GC_HW, GC_HW, 5):
            pts.append((x, y, GC_H))
        for z in np.linspace(0.0, GC_H, 5):
            pts += [(x, GC_HW, z), (x, -GC_HW, z)]
    for x in np.linspace(BAR_X0, BAR_X1, 14):
        for a in np.arange(0, 360, 30):
            pts.append((x, BAR_R * math.cos(math.radians(a)), BAR_Z + BAR_R * math.sin(math.radians(a))))
    for x in np.linspace(MNT_X0, MNT_X1, 3):
        for y in np.linspace(-MNT_HW, MNT_HW, 4):
            pts.append((x, y, MNT_H))
    for y in np.linspace(HDL_Y0, HDL_Y1, 8):
        for a in np.arange(0, 360, 45):
            pts.append((HDL_R * math.cos(math.radians(a)), y, HDL_Z + HDL_R * math.sin(math.radians(a))))
    pts += [(0, 0, SP_TOP), (BB_R, 0, BB_TOP), (-BB_R, 0, BB_TOP), (0, BB_R, BB_TOP), (0, -BB_R, BB_TOP)]
    return np.array(pts)


BODY_PTS = _body_pts()


def _turn_z(points, deg, about=(0.0, 0.0)):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    p = np.array(points, float).copy()
    x, y = p[:, 0] - about[0], p[:, 1] - about[1]
    p[:, 0], p[:, 1] = about[0] + c * x - s * y, about[1] + s * x + c * y
    return p


def grinder_points(guard_deg: float, lever_deg: float) -> np.ndarray:
    """Sample points on the whole grinder in the part frame, for the guard and lever at these joint angles (degrees)."""
    lev = _turn_z(_turn_z(LEVER_PTS, lever_deg, PIVOT), guard_deg)
    return np.vstack([BODY_PTS, _turn_z(GUARD_PTS, guard_deg), lev])


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def guard_error_deg(guard_deg: float) -> float:
    """Degrees between the hood's closed side and the rear of the tool (towards the operator)."""
    return abs(wrap180(guard_deg - TARGET_DEG))


def tier(err_deg: float, lever_open_deg: float, clear: bool, lying: bool) -> float:
    """The score of one step's state (no history)."""
    if err_deg > PART_DEG or lever_open_deg > SHUT_DEG:
        return 0.0  # wrong way, or turned but left unclamped: no credit for a loose guard
    if clear and lying:
        return 0.5  # 1.0 needs err <= FULL_DEG held for HOLD_STEPS (Trial)
    return 0.25  # clamped within PART_DEG, but still held or not lying on its back


# -- the clamp and the record, each control step --------------------------------------------------------------------------
class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the record is cleared
        self.fell = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.locked = False
        self.lever_target = 0.0
        self.z0 = None
        if env is not None:
            m.lock(env, "grinder", "guard")
            self.locked = True
            m.set_target(env, "grinder", "lever", 0.0)
            self.z0 = float(m.state(env, "grinder").pos[2])

    def _clamp(self, env, s):
        a = -s.deg("lever")  # lever opening, degrees
        if a < SNAP_DEG:
            target = 0.0
        elif a < LOOSE_DEG:
            target = LOOSE_DEG
        else:
            target = a
        m.set_target(env, "grinder", "lever", -math.radians(target))
        if self.locked and a > UNLOCK_DEG:
            m.unlock(env, "grinder", "guard")
            self.locked = False
        elif not self.locked and a <= LOCK_DEG:
            m.lock(env, "grinder", "guard")
            self.locked = True
        return a

    def step(self, env):
        s = m.state(env, "grinder")
        a = self._clamp(env, s)
        g = s.deg("guard")
        if s.pos[2] < TOP - FELL_DROP:
            self.fell = True  # latched: knocked off the bench
        tips = s.to_local(m.fingertips(env))
        pts = grinder_points(g, s.deg("lever"))
        gap = float(np.min(np.linalg.norm(tips[:, None, :] - pts[None, :, :], axis=2)))
        clear = gap >= CLEAR_M
        lying = s.tilt_deg() <= TILT_MAX and (self.z0 is None or abs(float(s.pos[2]) - self.z0) < 0.01)
        err = guard_error_deg(g)
        ok = (not self.fell) and err <= FULL_DEG and a <= SHUT_DEG and self.locked and clear and lying
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.fell:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = tier(err, a, clear, lying)
        self.last = dict(guard=g, err=err, lever=a, locked=self.locked, gap=gap, tilt=s.tilt_deg(), z=float(s.pos[2]))
        import os, json  # DEBUG-REMOVE
        if os.environ.get('GRINDER_DEBUG'):  # DEBUG-REMOVE
            open(os.environ['GRINDER_DEBUG'], 'a').write(json.dumps({k: (float(v) if not isinstance(v, bool) else v) for k, v in self.last.items()} | {'score': self.score, 'pos': [float(x) for x in s.pos], 'ltip': [round(float(x), 4) for x in tips[:12].mean(0)], 'ltips': tips[:12].round(4).tolist(), 'lev_gap': float(np.min(np.linalg.norm(tips[:12, None, :] - _turn_z(_turn_z(LEVER_PTS, s.deg('lever'), PIVOT), g)[None], axis=2)))}) + '\n')  # DEBUG-REMOVE


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- scripted demo (open loop, from the published starting layout) --------------------------------------------------------
HOVER = 0.15  # above the bench top
PAD = 0.013  # pad tips below the TCP


def world_xy(p_xy) -> np.ndarray:
    return SPINDLE_XY + _rot(YAW_DEG) @ np.asarray(p_xy, float)


def wp(t, p_xy, z, finger, label, close_deg):
    """A waypoint from part-frame xy, a height above the bench and the part-frame bearing the fingers close along. World yaw 0
    closes them along world y, which is part bearing 0 (YAW_DEG = 90), so the world yaw is that bearing."""
    x, y = world_xy(p_xy)
    yaw = math.radians(wrap180(close_deg))
    return Waypoint(t, (float(x), float(y), float(TOP + z)), float(finger), label, yaw)


def _polar(r, deg):
    return np.array([r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))])


TAB_GRIP_R = 0.047
SKIRT_R = (SKIRT_RI + SKIRT_RO) / 2
Z_TAB = LEV_Z0 + 0.004 + PAD  # pad tips 4 mm above the tab's lower edge
Z_SKIRT = PLATE_Z1 + 0.006 + PAD  # pad tips 6 mm above the plate
LEVER_SWING = 40.0  # degrees of lever opening the demo drives
STROKE = 61.0  # a little over a third of a half turn: the hood lags the fingers slightly
SHUT_PRESS = 6.0  # degrees past shut the demo presses the lever (the band stops it at shut)
PUSH_FROM = 54.0  # lever angle at which the closed fingers start behind the open tab (it rests near 42)
PUSH_OFF = 0.013  # TCP behind the tab face when pushing
Z_PUSH = LEV_Z1 - 0.004 + PAD  # pad tips 4 mm below the tab's top edge: clear of the gear case top, 2 mm under the tab
PUSH_R = 0.035  # TCP this far out along the tab: the pads reach about 3 cm past the TCP, so they cover the whole tab


def _tab_grip(guard_deg, lever_open_deg, r=TAB_GRIP_R):
    """Part-frame xy of the tab's grip point (r from the spindle when shut) and the yaw that closes the fingers across it."""
    p = _polar(r, TAB_DEG)
    p = PIVOT + _rot(-lever_open_deg) @ (p - PIVOT)
    p = _rot(guard_deg) @ p
    tab_dir = TAB_DEG - lever_open_deg + guard_deg
    return p, tab_dir + 90.0  # the fingers close across the tab, at right angles to it


def wpw(t, xy, z, finger, label, yaw_deg=0.0):
    """A waypoint in world xy (a parked pose)."""
    return Waypoint(t, (float(xy[0]), float(xy[1]), float(TOP + z)), float(finger), label, math.radians(yaw_deg))


# A parallel grip closing along a bearing is the same grip closing along the opposite one; each group of moves takes the
# one that keeps the wrist inside its +-120 degree range (checked offline with yam_arena's IK).
FLIPS = (0.0, 180.0, 180.0, 180.0, 0.0)  # lever opening, the three skirt strokes, lever shutting


def _left_plan(flips=None):
    f_open, f1, f2, f3, f_shut = FLIPS if flips is None else flips
    w = []
    # 1. open the clamp lever: pinch its tab and swing it out past the over-centre point
    p, d = _tab_grip(0.0, 0.0)
    d += f_open
    w += [wp(2.0, p, HOVER, -0.012, "over the lever tab", d), wp(1.5, p, Z_TAB + 0.02, -0.012, "down to the tab", d),
          wp(1.0, p, Z_TAB, -0.012, "at the tab", d), wp(0.8, p, Z_TAB, FINGER_CLOSED, "pinch the tab", d)]
    for k in range(1, 5):
        a = LEVER_SWING * k / 4
        q, dq = _tab_grip(0.0, a)
        w.append(wp(0.7, q, Z_TAB, FINGER_CLOSED, f"swing the lever open {a:.0f}", dq + f_open))
    q, dq = _tab_grip(0.0, LEVER_SWING)
    w += [wp(0.6, q, Z_TAB, -0.012, "let go of the tab", dq + f_open), wp(1.0, q, HOVER, -0.012, "up from the tab", dq + f_open)]
    # 2. turn the hood half a turn by its skirt, in three strokes of STROKE degrees
    g = 0.0
    for n, (grip_rel, fl) in enumerate(zip((30.0, 0.0, -30.0), (f1, f2, f3)), 1):  # where on the hood each stroke takes hold
        start = grip_rel + g
        p = _polar(SKIRT_R, start)
        w += [wp(1.6, p, HOVER, -0.022, f"stroke {n}: over the skirt", start + fl),
              wp(1.2, p, Z_SKIRT + 0.02, -0.022, f"stroke {n}: down beside the skirt", start + fl),
              wp(0.8, p, Z_SKIRT, -0.022, f"stroke {n}: straddle the skirt", start + fl),
              wp(0.8, p, Z_SKIRT, FINGER_CLOSED, f"stroke {n}: pinch the skirt", start + fl)]
        for k in range(1, 7):
            b = start + STROKE * k / 6
            w.append(wp(0.55, _polar(SKIRT_R, b), Z_SKIRT, FINGER_CLOSED, f"stroke {n}: turn the hood", b + fl))
        g += STROKE
        end = start + STROKE
        w += [wp(0.6, _polar(SKIRT_R, end), Z_SKIRT, -0.022, f"stroke {n}: let go of the skirt", end + fl),
              wp(1.0, _polar(SKIRT_R, end), HOVER, -0.022, f"stroke {n}: up", end + fl)]
    # 3. shut the lever: with the fingers closed, push its tab from behind and swing it back against the band, as a thumb
    # does (the hood lags the fingers in the strokes, so it ends near 180)
    g = TARGET_DEG

    def push_at(a):
        q, dq = _tab_grip(g, a, PUSH_R)
        return q + PUSH_OFF * _polar(1.0, TAB_DEG - 90.0 - a + g), dq + f_shut

    p, d = push_at(PUSH_FROM)
    w += [wp(1.8, p, HOVER, FINGER_CLOSED, "over the open lever, behind its tab", d),
          wp(1.5, p, Z_PUSH + 0.02, FINGER_CLOSED, "down behind the open lever's tab", d),
          wp(1.0, p, Z_PUSH, FINGER_CLOSED, "behind the tab", d),
          wp(1.2, p, Z_PUSH, FINGER_CLOSED, "wait for the other hand on the hood", d)]
    for k in range(1, 9):
        a = PUSH_FROM - (PUSH_FROM + SHUT_PRESS) * k / 8  # ends SHUT_PRESS past shut: the band stops the lever there
        q, dq = push_at(a)
        w.append(wp(0.6, q, Z_PUSH, FINGER_CLOSED, f"push the lever shut {a:.0f}", dq))
    q, dq = push_at(-SHUT_PRESS)
    q_back, _ = push_at(8.0)
    w += [wp(0.8, q, Z_PUSH, FINGER_CLOSED, "hold it shut", dq), wp(0.6, q_back, Z_PUSH, FINGER_CLOSED, "let go of the shut lever", dq),
          wp(1.0, q_back, HOVER, FINGER_CLOSED, "up from the shut lever", dq), wpw(1.8, LEFT_PARK, 0.22, -0.012, "park")]
    return w


LEFT_PARK, RIGHT_PARK = (0.30, 0.27), (0.30, -0.27)


HOLD_X = -0.21  # where the right hand holds the motor barrel (part x)
HOOD_HOLD_DEG = 200.0  # where on the turned hood the right hand pinches the skirt while the lever is shut (part bearing)
RIGHT_PARK_YAW = 0.0
R_HOVER = 0.13  # lower than HOVER: the right wrist cannot hold this grip higher up (offline IK)
RIGHT_FLIP = 180.0  # the right wrist's grip on the hood, flipped to stay in its range (checked offline)


def _ends(left):
    """Time at which each labelled left waypoint ends."""
    t, out = 0.0, {}
    for x in left:
        t += x.duration
        out[x.label] = t
    return out


def _right_plan(left):
    """Hold the grinder by its motor barrel while the clamp is opened and the hood turned; then hold the hood still by its skirt
    while the left hand swings the lever shut (a loose hood otherwise turns with the lever)."""
    ends = _ends(left)
    p = np.array([HOLD_X, 0.0])
    z = BAR_Z + PAD + 0.004  # pad tips 4 mm above the barrel's axis
    w = [wp(2.0, p, HOVER, FINGER_OPEN, "over the barrel", 90.0), wp(1.5, p, z + 0.02, FINGER_OPEN, "down beside the barrel", 90.0),
         wp(0.8, p, z, FINGER_OPEN, "round the barrel", 90.0), wp(0.8, p, z, FINGER_CLOSED, "hold the grinder", 90.0)]
    t = sum(x.duration for x in w)
    w.append(wp(ends["stroke 3: up"] - t, p, z, FINGER_CLOSED, "keep holding", 90.0))
    h = _polar(SKIRT_R, HOOD_HOLD_DEG)
    d = HOOD_HOLD_DEG + RIGHT_FLIP
    w += [wp(0.6, p, z, FINGER_OPEN, "let go of the barrel", 90.0), wp(1.0, p, HOVER, FINGER_OPEN, "up from the barrel", 90.0),
          wp(1.0, p, R_HOVER, -0.022, "turn the wrist", d), wp(1.0, h, R_HOVER, -0.022, "over the hood", d), wp(1.2, h, Z_SKIRT + 0.02, -0.022, "down beside the hood's skirt", d),
          wp(0.7, h, Z_SKIRT, -0.022, "straddle the skirt", d), wp(0.6, h, Z_SKIRT, FINGER_CLOSED, "pinch the skirt", d)]
    t = sum(x.duration for x in w)
    w.append(wp(ends["let go of the shut lever"] - t, h, Z_SKIRT, FINGER_CLOSED, "hold the hood while the lever shuts", d))
    w += [wp(0.6, h, Z_SKIRT, -0.022, "let go of the hood", d), wp(1.0, h, R_HOVER, -0.022, "up from the hood", d),
          wpw(1.8, RIGHT_PARK, 0.22, FINGER_OPEN, "park", RIGHT_PARK_YAW)]
    return w


def demo():
    left = _left_plan()
    return {"left": left, "right": _right_plan(left)}
