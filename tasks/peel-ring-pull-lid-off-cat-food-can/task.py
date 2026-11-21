"""On the station table (standing in for a kitchen counter): one unopened 85 g (3 oz) single-serve cat-food can, the
small 211 x 108 size (2 11/16 in across, 1 8/16 in tall: 68 x 38 mm over the seam), with a full-aperture easy-open
steel end. The end's removable panel is 57 mm across, bounded by its score line, and sits 4.5 mm below the top of the
double seam. A ring-pull tab is riveted to the panel 14.5 mm in from the score; its 20 mm-wide ring points at the
panel's centre and is formed slightly upward (6 degrees), so its free end stands about 3 mm off the panel. Beyond the
can: the cat's bowl, a plastic spoon and a plastic fork (distractors).

The ring lifts on its rivet. Lifting the ring's end about 4 mm (8 degrees on the rivet) takes about 15 N at the ring's
end; there the score pops. After the pop the tab folds freely on its rivet, and pulling the ring tears the score round
the panel: the panel is a rigid flap that turns about the far side of the score, where the tear finishes, against a
resistance that never springs back. The last segment of score breaks once the panel has been turned past 125 degrees
and pulled a little harder, and the panel comes away. The filled can weighs 105 g: lifted at the ring, it comes up
with its lid or tips over unless it is held down.

Scoring is in `score()`; the demo is in `demo()`.
"""

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

# -- the can (its own frame: origin on the table under its axis, z up) --------------------------------------------------

CAN_X, CAN_Y = 0.30, 0.03  # where the can stands
CAN_YAW = 0.0  # the rivet on the +y side, the ring pointing -y at the centre: the panel comes away toward -y
R_BODY = 0.0325  # can body radius (65 mm body)
R_SEAM = 0.0340  # outside of the double seam (68 mm)
R_SEAM_IN = 0.0295  # inside of the seam / countersink wall
Z_SEAM = 0.0380  # top of the double seam above the table (38 mm, the "108" height)
Z_PANEL = 0.0335  # top of the end's panel: the countersink is 4.5 mm deep
PANEL_T = 0.0015  # collider thickness of the panel (the steel is 0.2 mm; see fidelity)
R_PANEL = 0.0285  # the score line: the removable panel is 57 mm across
CAN_MASS = 0.100  # filled can without its panel and tab (105 g in all)

# -- the lid (panel + tab; its own frame: origin at the panel's top centre) ---------------------------------------------

LID_MASS = 0.004  # 57 mm steel panel, 0.2 mm thick
RING_MASS = 0.002  # steel ring-pull tab
Y_RIVET = 0.0145  # rivet: 14.5 mm in from the score, toward +y (the ring points back toward -y, at the centre)
RING_LEN = 0.027  # rivet centre to the ring's outer end
RING_W = 0.020  # ring's outer width (x)
RING_T = 0.0015  # ring's thickness (curled edges)
RING_BAND = 0.0035  # width of the ring's band
RING_FORM = 6.0  # the ring is formed upward this many degrees from the panel
PINCH = 0.023  # where the demo pinches the ring: this far from the rivet along it

# the panel turns about the far side of the score: a line along x at the inside top edge of the seam
HINGE_Y = -R_SEAM_IN  # in the can frame (y), at z = Z_SEAM

LOOKS = {
    "tin": m.Look(color=m.rgb(0.80, 0.80, 0.78), metallic=0.9, roughness=0.30),
    "label": m.Look(color=m.rgb(0.42, 0.20, 0.45), metallic=0.3, roughness=0.45),
    "band": m.Look(color=m.rgb(0.85, 0.70, 0.30), metallic=0.6, roughness=0.35),
    "panel": m.Look(color=m.rgb(0.86, 0.86, 0.84), metallic=0.9, roughness=0.25),
    "score": m.Look(color=m.rgb(0.35, 0.35, 0.36), metallic=0.6, roughness=0.5),
    "tab": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=0.9, roughness=0.25),
    "food": m.Look(color=m.rgb(0.42, 0.27, 0.17), metallic=0.0, roughness=0.8),
}


def _ring_turn(mesh, deg):
    """Turn a mesh about the rivet's axis (x through the rivet); positive lowers the ring's free end."""
    return mesh.moved((0.0, -Y_RIVET, 0.0)).turned(deg, "x").moved((0.0, Y_RIVET, 0.0))


def _can():
    body = m.lathe([(0.0, 0.0), (0.030, 0.0), (0.031, 0.0015), (R_BODY, 0.003), (R_BODY, 0.0325),
                    (R_SEAM, 0.0335), (R_SEAM, Z_SEAM - 0.0008), (R_SEAM - 0.0012, Z_SEAM), (R_SEAM_IN + 0.0006, Z_SEAM),
                    (R_SEAM_IN, Z_SEAM - 0.001), (R_SEAM_IN, Z_PANEL - PANEL_T), (0.0, Z_PANEL - PANEL_T)], segments=48)
    label = m.lathe([(0.0, 0.006), (R_BODY + 0.0004, 0.006), (R_BODY + 0.0004, 0.029), (0.0, 0.029)], segments=48)
    band = m.lathe([(0.0, 0.0145), (R_BODY + 0.0007, 0.0145), (R_BODY + 0.0007, 0.0205), (0.0, 0.0205)], segments=48)
    food = m.cylinder(R_SEAM_IN - 0.0003, 0.0004, (0.0, 0.0, Z_PANEL - PANEL_T - 0.0001), segments=48)
    parts = [("body", body, "tin"), ("label", label, "label"), ("band", band, "band"), ("food", food, "food")]
    cols = [m.col_cylinder("body_c", R_BODY, Z_PANEL - PANEL_T, (0.0, 0.0, (Z_PANEL - PANEL_T) / 2))]
    n = 16
    zc, hz = (Z_PANEL - PANEL_T + Z_SEAM) / 2, Z_SEAM - (Z_PANEL - PANEL_T)
    for i in range(n):
        a = 360.0 * i / n
        seg = m.box((R_SEAM - R_SEAM_IN, 2 * math.pi * R_SEAM / n + 0.002, hz), ((R_SEAM + R_SEAM_IN) / 2, 0.0, zc))
        cols.append(m.col_hull(f"seam{i}_c", seg.turned(a, "z")))
    return m.free("can", (CAN_X, CAN_Y, TOP), parts, cols, LOOKS, mass=CAN_MASS, friction=(1.0, 0.8), yaw_deg=CAN_YAW,
                  com=(0.0, 0.0, 0.017))


def _ring_loop():
    """The ring's band as a closed tube round a rounded rectangle, from the rivet toward -y, before its forming."""
    w, r = RING_W / 2 - 0.0008, 0.006
    y0, y1 = Y_RIVET - 0.004, Y_RIVET - RING_LEN + 0.0008
    pts = []
    for cx, cy, a0 in ((w - r, y1 + r, -90), (-(w - r), y1 + r, 180), (-(w - r), y0 - r, 90), (w - r, y0 - r, 0)):
        for k in range(7):
            a = math.radians(a0 + 15 * k)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a), RING_T / 2 + 0.0003))
    pts = pts[::-1]
    pts.append(pts[0])
    return m.tube(pts, 0.0008, sides=8)


def _lid():
    panel = m.cylinder(R_PANEL, 0.0004, (0.0, 0.0, -0.0002), segments=48)
    score = m.lathe([(R_PANEL - 0.0012, -0.00005), (R_PANEL - 0.0004, -0.00005), (R_PANEL - 0.0004, 0.00008),
                     (R_PANEL - 0.0012, 0.00008)], segments=48)
    bead = m.lathe([(0.016, -0.00005), (0.0172, -0.00005), (0.0172, 0.00009), (0.016, 0.00009)], segments=48)
    rivet = m.cylinder(0.0022, 0.0009, (0.0, Y_RIVET, 0.00045), segments=16)
    frame_parts = [("panel", panel, "panel"), ("score", score, "score"), ("bead", bead, "score"), ("rivet", rivet, "tab")]
    frame_cols = [m.col_cylinder("panel_c", R_PANEL, PANEL_T, (0.0, 0.0, -PANEL_T / 2))]
    # the tab: a nose plate from the rivet toward the score, and the ring toward the centre, formed upward
    nose = m.prism([(-0.005, Y_RIVET - 0.004), (0.005, Y_RIVET - 0.004), (0.0045, Y_RIVET + 0.0075),
                    (0.0, Y_RIVET + 0.0095), (-0.0045, Y_RIVET + 0.0075)], 0.0003, 0.0012)
    web = m.box((0.010, 0.006, 0.0008), (0.0, Y_RIVET - 0.006, 0.0009))
    loop = _ring_loop()
    ring_parts = [("nose", _ring_turn(nose, -RING_FORM), "tab"), ("web", _ring_turn(web, -RING_FORM), "tab"),
                  ("loop", _ring_turn(loop, -RING_FORM), "tab")]
    slab = m.box((RING_W, RING_LEN - 0.002, RING_T), (0.0, Y_RIVET - (RING_LEN - 0.002) / 2 - 0.002, RING_T / 2 + 0.0003))
    ring = m.hinge("ring", "X", (0.0, Y_RIVET, 0.0005), -170.0, 5.0, ring_parts,
                   [m.col_hull("ring_c", _ring_turn(slab, -RING_FORM))],
                   mass=RING_MASS, spring=POP_TORQUE / math.radians(POP_DEG), damping=0.002, friction=0.0, effort=3.0,
                   armature=1e-5, inertia=(1e-6, 1e-6, 1e-6))
    return m.mechanism("lid", (CAN_X, CAN_Y, TOP + Z_PANEL), [ring], LOOKS, frame_parts=frame_parts,
                       frame_colliders=frame_cols, friction=(1.0, 0.8), solver_iters=48, free_mass=LID_MASS, yaw_deg=CAN_YAW,
                       frame_inertia=(2.0e-6, 2.0e-6, 3.5e-6))


# -- the score: pop, peel, last segment ---------------------------------------------------------------------------------

POP_DEG = 8.0  # the ring lifted this far on its rivet (about 4 mm at its end) cracks the score
POP_TORQUE = 0.40  # N m on the rivet to get there: about 15 N at the ring's end
PEEL_K = 20.0  # N m/rad: stiffness of the tear front (how far the panel must be pulled past it to tear on)
T_START = 0.30  # N m: tearing the first 20 degrees, round the rivet end of the score
T_PEEL = 0.22  # N m: tearing on round the sides
FINAL_DEG = 105.0  # the last segment of score, at the hinge side, holds from here
T_FINAL = 0.34  # N m: and breaks at this
PEEL_DAMP = 0.01  # N m s/rad


def _tear_strength(front_deg):
    return T_START if front_deg < 20.0 else T_PEEL


LID = _lid()
CAN = _can()

OBJECTS = [
    CAN,
    LID,
    # distractors that belong at a cat's feeding spot (SimReady props); scoring ignores them
    SimReady("bowl", pos=(0.52, 0.17, TOP), size=(0.163, 0.163, 0.055), mass=0.35, yaw=0.0, convex=True,
             package="dishware_bowl_a01", usd="sm_dishware_bowl_a01_01.usda"),
    SimReady("spoon", pos=(0.20, 0.25, TOP), size=(0.157, 0.035, 0.011), mass=0.006, yaw=0.5,
             package="dishware_plastic_spoon_a01", usd="sm_dishware_plastic_spoon_a01_01.usd"),
    SimReady("fork", pos=(0.53, -0.20, TOP), size=(0.158, 0.030, 0.013), mass=0.006, yaw=-0.4,
             package="dishware_plastic_fork_a01", usd="sm_dishware_plastic_fork_a01_01.usd"),
]

# -- the score joints: authored after each reset between the can and the panel -------------------------------------------


def _mat_to_quat(r):
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    if i == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s)


_J = {"gen": 0, "seal": None, "hinge": None}


def _body_paths(stage):
    from pxr import UsdPhysics

    can = f"{m.ENV_PATH}/can"
    lid = f"{m.ENV_PATH}/lid/frame"
    for p in (can, lid):
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(f"peel task: no rigid body at {p}")
    return can, lid


def _remove(key):
    import isaaclab.sim as sim_utils

    path = _J.get(key)
    if path:
        stage = sim_utils.get_current_stage()
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)
    _J[key] = None


def _author_score(environment):
    """The seal (a fixed joint: the uncracked score) and the hinge (the far side of the score, where the tear ends),
    both between the can and the panel, at the settled pose. Removed when the score pops and when the last segment
    breaks; written afresh at every reset."""
    import isaaclab.sim as sim_utils
    from pxr import Gf, Sdf, UsdPhysics

    _remove("seal")
    _remove("hinge")
    stage = sim_utils.get_current_stage()
    can_path, lid_path = _body_paths(stage)
    c, lid = m.state(environment, "can"), m.state(environment, "lid")
    rc, rl = c.rotation(), lid.rotation()
    rel = rl.T @ rc  # can frame in the panel's frame

    def q(r):
        x, y, z, w = _mat_to_quat(r)
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

    def v(p):
        return Gf.Vec3f(*[float(t) for t in p])

    _J["gen"] += 1
    g = _J["gen"]
    hinge_w = c.to_world(np.array([0.0, HINGE_Y, Z_SEAM]))[0]
    lid_w = lid.pos
    for key, kind, point in (("seal", UsdPhysics.FixedJoint, lid_w), ("hinge", UsdPhysics.RevoluteJoint, hinge_w)):
        path = f"{m.ENV_PATH}/score_{key}_{g}"
        joint = kind.Define(stage, path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(can_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(lid_path)])
        joint.CreateLocalPos0Attr().Set(v(c.to_local(point)[0]))
        joint.CreateLocalRot0Attr().Set(q(np.eye(3)))
        joint.CreateLocalPos1Attr().Set(v(lid.to_local(point)[0]))
        joint.CreateLocalRot1Attr().Set(q(rel))
        joint.CreateCollisionEnabledAttr().Set(False)
        joint.CreateExcludeFromArticulationAttr().Set(True)
        if key == "hinge":
            joint.CreateAxisAttr().Set("X")
            joint.CreateLowerLimitAttr().Set(-2.0)
            joint.CreateUpperLimitAttr().Set(178.0)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(float(PEEL_K * math.pi / 180.0))
            drive.CreateDampingAttr().Set(float(PEEL_DAMP * math.pi / 180.0))
            drive.CreateTargetPositionAttr().Set(0.0)
            drive.CreateTargetVelocityAttr().Set(0.0)
            drive.CreateMaxForceAttr().Set(5.0)
        _J[key] = path


def _set_front(deg):
    import isaaclab.sim as sim_utils
    from pxr import UsdPhysics

    if _J["hinge"]:
        prim = sim_utils.get_current_stage().GetPrimAtPath(_J["hinge"])
        UsdPhysics.DriveAPI(prim, "angular").GetTargetPositionAttr().Set(float(deg))


def lid_angle(c, lid):
    """Degrees the panel has turned about the hinge line, read from the can's and the panel's poses."""
    v = c.rotation().T @ lid.rotation() @ np.array([0.0, 1.0, 0.0])
    return float(math.degrees(math.atan2(v[2], v[1])))


def ring_lift(lid):
    """Degrees the ring has been lifted on its rivet from where it was formed."""
    return float(-lid.deg("ring"))


class Score:
    """The state of the score line: popped, how far round it is torn, whether the last segment has broken."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.popped, self.broken, self.front, self.angle = False, False, 0.0, 0.0
        if environment is not None:
            _author_score(environment)

    def step(self, environment):
        c, lid = m.state(environment, "can"), m.state(environment, "lid")
        if not self.broken:
            self.angle = lid_angle(c, lid)
        if not self.popped and ring_lift(lid) >= POP_DEG:
            self.popped = True
            _remove("seal")
            m.set_target(environment, "lid", "ring", 0.0)
            asset, j = m._joint(environment, "lid", "ring")
            asset.write_joint_stiffness_to_sim(m._tensor(asset, 0.0), joint_ids=[j])
        if self.popped and not self.broken:
            if self.front < FINAL_DEG:
                lag = _tear_strength(self.front) / PEEL_K
                if math.radians(self.angle - self.front) > lag:
                    self.front = min(self.angle - math.degrees(lag), FINAL_DEG)
                    _set_front(self.front)
            elif math.radians(self.angle - FINAL_DEG) > T_FINAL / PEEL_K:
                self.broken = True
                _remove("hinge")


SCORE = Score()

# -- scoring ------------------------------------------------------------------------------------------------------------

TIP_DEG = 30.0  # the can tipped past this at any moment: spilled
LIFT_M = 0.020  # or its base lifted this far off the table: it came up with the lid
UPRIGHT_DEG = 5.0
BASE_GAP = 0.003  # resting: base within 3 mm of the table
LID_CLEAR = 0.09  # lid set down with its centre at least 9 cm from the can's axis (3 cm clear of the seam)
LID_DOWN = 0.012  # lid's centre at most 12 mm above the table: lying on it
HANDS_CLEAR = 0.03
STILL = 0.01  # m/s
HOLD_S = 1.0
HALF_DEG = 90.0  # the panel turned 90 degrees: half its score line torn

SUCCESS_HOLD_S = 0.0  # the Trial counts its own hold


def _dist_can(c, pts):
    loc = c.to_local(pts)
    dr = np.maximum(np.hypot(loc[:, 0], loc[:, 1]) - R_SEAM, 0.0)
    dz = np.maximum(np.maximum(loc[:, 2] - Z_SEAM, -loc[:, 2]), 0.0)
    return np.hypot(dr, dz)


def _dist_lid(lid, pts):
    loc = lid.to_local(pts)
    dr = np.maximum(np.hypot(loc[:, 0], loc[:, 1]) - (R_PANEL + 0.012), 0.0)  # the ring can stand past the rim
    return np.hypot(dr, np.abs(loc[:, 2]))


class Trial:
    """Updated once per control step. The harm record is cleared only by reset()."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.spilled, self.hold, self.full, self.value, self.max_angle = False, 0.0, False, 0.0, 0.0
        self.why = ""

    def step(self, environment):
        c, lid = m.state(environment, "can"), m.state(environment, "lid")
        tips = m.fingertips(environment)
        dt = float(environment.unwrapped.step_dt)
        if c.tilt_deg() > TIP_DEG or c.pos[2] - TOP > LIFT_M:
            self.spilled = True
        self.max_angle = max(self.max_angle, SCORE.angle) if SCORE.popped else 0.0
        off = SCORE.broken
        lid_r = float(np.hypot(*(lid.pos[:2] - c.pos[:2])))
        ready = bool(off and not self.spilled
                     and lid_r >= LID_CLEAR and lid.pos[2] - TOP <= LID_DOWN
                     and c.tilt_deg() <= UPRIGHT_DEG and abs(c.pos[2] - TOP) <= BASE_GAP
                     and float(_dist_can(c, tips).min()) >= HANDS_CLEAR
                     and float(_dist_lid(lid, tips).min()) >= HANDS_CLEAR
                     and np.linalg.norm(c.lin_vel) < STILL and np.linalg.norm(lid.lin_vel) < STILL)
        self.hold = self.hold + dt if ready else 0.0
        self.full = bool(ready and self.hold >= HOLD_S - 1e-9)
        if self.spilled:
            self.value = 0.0
        elif self.full:
            self.value = 1.0
        elif off:
            self.value = 0.75
        elif self.max_angle >= HALF_DEG:
            self.value = 0.5
        elif SCORE.popped:
            self.value = 0.25
        else:
            self.value = 0.0


TRIAL = Trial()


@m.on_reset
def _reset(environment):
    SCORE.reset(environment)
    TRIAL.reset(environment)


@m.on_step
def _step(environment):
    SCORE.step(environment)
    TRIAL.step(environment)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo --------------------------------------------------------------------------------------------------


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
    """Tilted theta from straight down towards phi (from +x), fingers closing across that direction."""
    phi = math.radians(phi_deg)
    r = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain_grip(phi)
    return _orient(r)


def _w(p):
    """A point in the can's frame, as the can stands at the start, in world."""
    c, s = math.cos(math.radians(CAN_YAW)), math.sin(math.radians(CAN_YAW))
    return (CAN_X + c * p[0] - s * p[1], CAN_Y + s * p[0] + c * p[1], TOP + p[2])


PIN_GRIP = tilted(60.0, 0.0)  # from the near (-x) side, fingertips leaning toward +x, closing along y round the body below its seam
RING_GRIP = tilted(0.0, 90.0)  # straight down, closing along x across the ring's width
TIP_BELOW = 0.0035  # the fingertips' flat ends go this far below the ring's mid-thickness: onto the panel
PAD_TIP = 0.013  # the fingertips' ends are this far past the TCP
RING_OPEN = -0.015  # the 15 mm-wide fingertips part 30 mm for the 20 mm ring: wide open they would straddle the whole can
SQUEEZE = -0.006  # then close to 12 mm: a 20 N squeeze, light enough for the ring to pivot between the pads
PINCH_R = 0.024  # pinch the ring across its width this far out from its rivet (3 mm short of its end)
LIFT_TO = 84.0  # lift the ring on its rivet until it stands up (the score pops at 8)
RIVET = np.array([0.0, Y_RIVET, Z_PANEL + 0.0005])
HINGE = np.array([0.0, HINGE_Y, Z_SEAM])


def _ring_at(lift_deg, r=PINCH_R):
    """A point r along the ring from its rivet (mid-thickness), the ring lifted lift_deg on the rivet (can frame)."""
    a = math.radians(RING_FORM + lift_deg)
    return RIVET + np.array([0.0, -r * math.cos(a), r * math.sin(a)]) + np.array([0.0, 0.0, RING_T / 2 + 0.0003])


def _arc(rho, alpha_deg):
    a = math.radians(alpha_deg)
    return HINGE + np.array([0.0, rho * math.cos(a), rho * math.sin(a)])


def _wp(duration, contact, finger, label):
    """A ring-hand waypoint with the pinch (between the pads, at the ring's mid-thickness) at `contact`, can frame."""
    return Waypoint(duration, _w(np.asarray(contact) + (PAD_TIP - TIP_BELOW) * np.array([0.0, 0.0, 1.0])), finger, label,
                    RING_GRIP)


def demo():
    """The left arm reaches round the can from the near side and holds it by its body. The right arm pinches the ring
    across its width near its raised end, lifts it on its rivet (the score pops) until it stands up, then pulls it up
    and over toward itself, round the far side of the score, until the last of the score breaks and the panel comes
    away hanging from the ring. It lays the panel on the table beside the can; both arms let go and leave."""
    up = np.array([0.0, 0.0, 1.0])
    ring0 = _ring_at(0.0)
    lift = [_wp(0.4, _ring_at(d), SQUEEZE, f"lift ring {d:.0f}") for d in (3, 6, 9, 12, 16, 22, 30, 40, 52, 64, 76, LIFT_TO)]
    start = _ring_at(LIFT_TO) - HINGE
    rho0, a0 = float(np.hypot(start[1], start[2])), math.degrees(math.atan2(start[2], start[1]))
    peel = []
    n = 18
    for k in range(1, n + 1):
        s = k / n
        rho = rho0 + (0.066 - rho0) * min(1.0, 3 * s)
        alpha = a0 + (128.0 - a0) * s
        peel.append(_wp(0.5, _arc(rho, alpha), SQUEEZE, f"peel {alpha:.0f}"))
    last = np.array(peel[-1].pos)
    away = np.array([CAN_X, CAN_Y - 0.14, TOP + 0.082])
    right = [
        _wp(2.5, ring0 + 0.06 * up, RING_OPEN, "above the ring"),
        _wp(2.0, ring0 + 0.010 * up, RING_OPEN, "down to the ring"),
        _wp(1.2, ring0, RING_OPEN, "fingertips on the panel either side of the ring"),
        _wp(1.0, ring0, SQUEEZE, "pinch the ring"),
        *lift,
        *peel,
        Waypoint(1.0, tuple(last + 0.03 * up), SQUEEZE, "lift the panel clear", RING_GRIP),
        Waypoint(2.0, tuple(away + 0.02 * up), SQUEEZE, "carry it aside", RING_GRIP),
        Waypoint(1.2, tuple(away), SQUEEZE, "lower it to the table", RING_GRIP),
        Waypoint(1.0, tuple(away), RING_OPEN, "let go", RING_GRIP),
        Waypoint(1.5, tuple(away + np.array([0.0, -0.03, 0.10])), RING_OPEN, "away", RING_GRIP),
        Waypoint(3.0, tuple(away + np.array([0.0, -0.03, 0.10])), RING_OPEN, "wait", RING_GRIP),
    ]
    t_right = sum(w.duration for w in right)
    pin = np.array([-0.0093, 0.0, 0.0165])  # TCP placing the fingertips 10 mm up, just past the axis: pads on the body, clear of the seam
    side = np.array([-0.06, 0.0, 0.06])  # back toward the arms and up
    t_hold_until = sum(w.duration for w in right[:-3])
    left = [
        Waypoint(2.5, _w(pin + side), FINGER_OPEN, "beside the can", PIN_GRIP),
        Waypoint(1.5, _w(pin + 0.004 * up), FINGER_OPEN, "round the can", PIN_GRIP),
        Waypoint(1.0, _w(pin), FINGER_CLOSED, "hold it", PIN_GRIP),
        Waypoint(max(0.5, t_hold_until - 5.0), _w(pin), FINGER_CLOSED, "hold while the lid is peeled", PIN_GRIP),
        Waypoint(1.0, _w(pin), FINGER_OPEN, "let go", PIN_GRIP),
        Waypoint(1.5, _w(pin + side), FINGER_OPEN, "out", PIN_GRIP),
        Waypoint(max(0.5, t_right - t_hold_until - 2.5 + 3.0), _w(pin + side + 0.05 * up), FINGER_OPEN, "away", PIN_GRIP),
    ]
    return {"left": left, "right": right}
