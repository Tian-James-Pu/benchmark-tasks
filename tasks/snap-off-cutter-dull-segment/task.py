"""Snap the dull end segment off an 18 mm snap-off cutter's blade into a pocket blade-disposal can, then put the knife down retracted.

The knife is an OLFA-style 18 mm heavy-duty snap-off cutter (152 mm long, about 45 g) with an 18 mm x 100 mm x 0.5 mm snap-off
blade scored every 12.5 mm (8 segments). It lies on a cutting mat with its blade left out about 30 mm (two score lines past the
nose), as after cutting thick card. Next to it stands an OLFA DC-3 pocket blade-disposal can (63 x 32 x 63 mm, about 32 g), its
door open over the snapping slot. OLFA's procedure for the DC-3: "insert the dull tip into the can up to the break line of the
blade, then push the handle down: the blade will snap off and fall into the can".

Model (declared in code; every state change is contact or joint driven, nothing is triggered by proximity):
- Knife ("knife", free articulation, 38 g frame): the handle and steel nose; the blade carrier on a slider along the knife
  ("blade", 5 g) carrying segments 3-8 and the thumb slider. The slider has notches every 2.5 mm: a 1500 N/m spring to the
  nearest notch plus 1 N of joint friction, so it takes about 3 N to move a notch and it stays where it is left.
- The two end segments are separate rigid bodies ("seg2", "seg1", 2 g each, gravity off while attached), joined to the blade
  by PhysX joints authored when the scene is built: seg2 is welded to the carrier at score line 2; seg1 hangs from seg2 at
  score line 1 on a revolute joint across the blade (its bending stiffness, 0.3 N m/rad). A bend of 10 degrees at line 1
  (about 0.05 N m, about 5 N at 10 mm) breaks the blade: the joint at the exposed score line nearest the nose is switched off
  and the freed piece falls under gravity. So with only line 1 past the nose exactly one segment comes off; with line 2 past
  the nose as well the break happens at line 2 and two segments come off. A score line still inside the steel nose cannot break.
- Disposal can ("case", free, 32 g): walls, floor, and a 26 x 3 mm slot in its top with 10 mm guide plates under it, all
  colliders; the snapped segment drops through into the can.
- Cutting mat (fixed), three SimReady distractors (mug, tape roll, marker) that scoring ignores.

Scoring (tracker every control step; harm latches are cleared only in on_reset):
- 0.0: no segment snapped; two segments snapped (break at score line 2); the knife ever fell (its downward speed passed
  0.8 m/s, a free fall of about 3 cm); or the knife's centre is off the mat.
- 1.0: exactly one segment snapped, it lies wholly inside the can (below the can's top face, within its walls), the
  remaining blade is retracted (no steel past the nose: the new tip at least 1 mm inside), the knife rests on the mat
  (lowest point within 3 mm of the mat, still), the can stands upright (within 10 degrees) on the mat, and every fingertip
  point is at least 5 cm from the knife and the can; all held for 1 s.
- 0.5: exactly one segment snapped, the blade retracted and the knife resting on the mat with every fingertip 5 cm clear of
  it, but the 1.0 conditions on the can or the segment not met (the segment loose on the mat, say).
- 0.25: exactly one segment snapped, otherwise (blade left out, knife still held).

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
INSTRUCTION = "Snap the dull end segment off the cutter's blade, put it in the blade-disposal case, and put the knife down with its blade retracted."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 1 s hold
HOLD_STEPS = 20
TOP = TABLE_TOP[SCENE]

# -- cutting mat -------------------------------------------------------------------------------------------------------------------------
MAT_T = 0.003
MAT = TOP + MAT_T  # the mat's surface
MAT_C = (0.285, 0.0)
MAT_SIZE = (0.22, 0.30)  # x, y

# -- knife (part frame: x toward the nose, nose at x = 0; spine on -y; z up from the handle's underside) -------------------------------------
L_HANDLE = 0.152
W_HANDLE = 0.025
T_HANDLE = 0.017
ZB = T_HANDLE / 2  # the blade's mid-plane
W_BLADE = 0.018
T_BLADE = 0.0005  # seen
T_BLADE_C = 0.001  # collider
PITCH_SEG = 0.0125
K_SKEW = 0.010 / W_BLADE  # score lines run 10 mm along the blade across its 18 mm: the point is on the edge (+y) side
E0 = 0.030  # the start: the tip line's centre 30 mm past the nose (lines 1 and 2 both out)
E_MIN, E_MAX = -0.005, 0.060
NOTCH = 0.0025
E_RETRACTED = 0.0115  # after one segment is gone the new tip (old line 1) is >= 1 mm inside the nose
BTN_FRONT = 0.085  # the thumb slider's front face, behind the tip line
KNIFE_POS = (0.20, -0.025, MAT)  # the nose, on the mat; yaw 180 points the nose toward the robot and the spine to +y
KNIFE_YAW = 180.0
GRIP_X = -0.129  # where the right hand holds the handle (its rear 36 mm), part frame

# -- disposal can (part frame: origin at the bottom centre; x its 63 mm length, y its 32 mm width) ------------------------------------------
CASE_L, CASE_W, CASE_H = 0.063, 0.032, 0.063
WALL = 0.002
SLOT_W, SLOT_L = 0.003, 0.026
GUIDE_D, GUIDE_T = 0.010, 0.0015
CASE_POS = (0.268, 0.055, MAT)

BEND_K = 0.3  # N m/rad
BREAK_DEG = 10.0
FALL_V = 0.8

LOOKS = {
    "handle": m.Look(color=m.rgb(0.95, 0.72, 0.05), roughness=0.5),
    "black": m.Look(color=m.rgb(0.06, 0.06, 0.06), roughness=0.6),
    "steel": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.9, roughness=0.25),
    "blade": m.Look(color=m.rgb(0.70, 0.72, 0.74), metallic=0.95, roughness=0.2),
    "dull": m.Look(color=m.rgb(0.45, 0.43, 0.40), metallic=0.8, roughness=0.45),
    "case": m.Look(color=m.rgb(0.93, 0.66, 0.04), roughness=0.45),
    "mat": m.Look(color=m.rgb(0.10, 0.32, 0.22), roughness=0.85),
    "grid": m.Look(color=m.rgb(0.80, 0.86, 0.80), roughness=0.8),
}


def _seg_outline(c_back: float, c_front: float, shrink: float = 0.0):
    """A blade segment between two score lines (line centres at c_back < c_front), counter-clockwise in xy."""
    h = W_BLADE / 2
    b, f = c_back + shrink, c_front - shrink
    return [(b - K_SKEW * h, -h), (f - K_SKEW * h, -h), (f + K_SKEW * h, h), (b + K_SKEW * h, h)]


def _plate(c_back, c_front, thick, shrink=0.0, z=ZB):
    return m.prism(_seg_outline(c_back, c_front, shrink), z - thick / 2, z + thick / 2)


# -- the knife ---------------------------------------------------------------------------------------------------------------------------
def _nose_hull():
    """The steel nose: from the full handle section at x = -0.016 to a thin lip whose face follows the score-line angle."""
    pts = []
    for y in (-W_HANDLE / 2, W_HANDLE / 2):
        for z in (0.0, T_HANDLE):
            pts.append((-0.016, y, z))
    for y in (-0.0105, 0.0105):
        for z in (ZB - 0.004, ZB + 0.004):
            pts.append((K_SKEW * y, y, z))
    v = np.array(pts)
    return m.Mesh(v, _hull_faces(v))


def _hull_faces(v):
    """Faces of the convex hull of a few points (for a visual of a hull collider)."""
    from itertools import combinations

    c = v.mean(axis=0)
    faces = []
    for i, j, k in combinations(range(len(v)), 3):
        n = np.cross(v[j] - v[i], v[k] - v[i])
        if np.linalg.norm(n) < 1e-12:
            continue
        d = (v - v[i]) @ n
        if np.all(d <= 1e-9) or np.all(d >= -1e-9):
            if np.dot(n, v[i] - c) < 0:
                j, k = k, j
            faces.append((i, j, k))
    return np.array(faces)


def _knife():
    hx = -0.016 - (L_HANDLE - 0.016) / 2
    body = m.box((L_HANDLE - 0.016 - 0.012, W_HANDLE, T_HANDLE), (hx + 0.006, 0, T_HANDLE / 2))
    cap = m.box((0.012, W_HANDLE + 0.001, T_HANDLE + 0.001), (-L_HANDLE + 0.006, 0, T_HANDLE / 2))
    grip = m.box((0.070, 0.002, T_HANDLE * 0.8), (-0.105, W_HANDLE / 2 + 0.0005, T_HANDLE / 2))
    grip2 = m.box((0.070, 0.002, T_HANDLE * 0.8), (-0.105, -W_HANDLE / 2 - 0.0005, T_HANDLE / 2))
    slot = m.box((0.072, 0.0012, 0.004), (-0.080, -W_HANDLE / 2 - 0.0002, ZB))
    nose = _nose_hull()
    frame_parts = [("body", body, "handle"), ("cap", cap, "black"), ("grip_l", grip, "black"), ("grip_r", grip2, "black"),
                   ("slot", slot, "black"), ("nose", nose, "steel")]
    frame_cols = [m.col_box("body_c", (L_HANDLE - 0.016, W_HANDLE, T_HANDLE), (hx, 0, T_HANDLE / 2)), m.col_hull("nose_c", nose)]
    # the blade carrier: segments 3-8 (line 8 is the blade's rear end) and the thumb slider on the spine
    lines = [E0 - PITCH_SEG * k for k in range(9)]  # lines[0] is the tip line
    parts = []
    for k in range(3, 9):
        parts.append((f"seg{k}", _plate(lines[k], lines[k - 1], T_BLADE, 0.00012), "blade"))
    x0, x1 = E0 - BTN_FRONT - 0.014, E0 - BTN_FRONT
    btn = m.box((x1 - x0, 0.005, 0.010), ((x0 + x1) / 2, -W_HANDLE / 2 - 0.0025, ZB))
    ridge = [m.box((0.0012, 0.0012, 0.009), (x0 + 0.002 + 0.0025 * i, -W_HANDLE / 2 - 0.0056, ZB)) for i in range(5)]
    stem = m.box((0.008, 0.004, 0.003), ((x0 + x1) / 2, -W_HANDLE / 2 + 0.001, ZB))
    parts += [("button", btn, "black"), ("ridges", m.join(*ridge), "black"), ("stem", stem, "black")]
    cols = [m.col_hull("blade_c", _plate(lines[8], lines[2], T_BLADE_C)), m.col_box("button_c", (x1 - x0, 0.005, 0.010), ((x0 + x1) / 2, -W_HANDLE / 2 - 0.0025, ZB))]
    blade = m.slider("blade", "X", E_MIN - E0, E_MAX - E0, parts, cols, mass=0.005, spring=1500.0, damping=3.0, friction=0.0, effort=50.0,
                     com=(E0 - 0.05, 0.0, ZB))
    inertia = (0.038 / 12 * (W_HANDLE**2 + T_HANDLE**2), 0.038 / 12 * (L_HANDLE**2 + T_HANDLE**2), 0.038 / 12 * (L_HANDLE**2 + W_HANDLE**2))
    return m.mechanism("knife", KNIFE_POS, [blade], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols, yaw_deg=KNIFE_YAW,
                       free_mass=0.038, frame_inertia=inertia, friction=(1.0, 0.8), solver_iters=32)


def _knife_to_world(p):
    """A point of the knife's part frame at its start pose, in the world."""
    a = math.radians(KNIFE_YAW)
    c, s = math.cos(a), math.sin(a)
    x, y, z = p
    return (KNIFE_POS[0] + c * x - s * y, KNIFE_POS[1] + s * x + c * y, KNIFE_POS[2] + z)


def _segment(name: str, c_back_world_line: float, dull: bool):
    """A free blade segment whose origin is its back score line's centre on the blade's mid-plane, posed as the knife."""
    looks = "dull" if dull else "blade"
    vis = [("plate", _plate(0.0, PITCH_SEG, T_BLADE, 0.00012, z=0.0), looks)]
    if dull:  # the worn edge: a darker strip along the cutting edge
        h = W_BLADE / 2
        edge = m.prism([(K_SKEW * (h - 0.002), h - 0.002), (PITCH_SEG + K_SKEW * (h - 0.002), h - 0.002), (PITCH_SEG + K_SKEW * h, h), (K_SKEW * h, h)],
                       -T_BLADE / 2 - 0.0001, T_BLADE / 2 + 0.0001)
        vis.append(("edge", edge, "black"))
    cols = [m.col_hull(f"{name}_c", _plate(0.0, PITCH_SEG, T_BLADE_C, z=0.0))]
    pos = _knife_to_world((c_back_world_line, 0.0, ZB))
    attrs = "\n    bool physxRigidBody:disableGravity = 1"
    return m.Part(name, pos=pos, size=(0.02, 0.02, 0.002), mass=0.002, role="free", friction=(0.9, 0.8), torsional_patch=None, solver_iters=32,
                  yaw_deg=KNIFE_YAW, build=lambda: m._file(m._root_name(name), '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"', attrs,
                                                         m._visual(vis) + "".join(cols), LOOKS, (0.9, 0.8)))


# -- the disposal can --------------------------------------------------------------------------------------------------------------------
def _case():
    hl, hw = CASE_L / 2, CASE_W / 2
    zt = CASE_H - WALL / 2
    boxes = {
        "floor": ((CASE_L, CASE_W, WALL), (0, 0, WALL / 2)),
        "wall_xn": ((WALL, CASE_W, CASE_H), (-hl + WALL / 2, 0, CASE_H / 2)),
        "wall_xp": ((WALL, CASE_W, CASE_H), (hl - WALL / 2, 0, CASE_H / 2)),
        "wall_yn": ((CASE_L, WALL, CASE_H), (0, -hw + WALL / 2, CASE_H / 2)),
        "wall_yp": ((CASE_L, WALL, CASE_H), (0, hw - WALL / 2, CASE_H / 2)),
        "top_n": ((hl - SLOT_W / 2, CASE_W, WALL), (-(hl + SLOT_W / 2) / 2, 0, zt)),
        "top_p": ((hl - SLOT_W / 2, CASE_W, WALL), ((hl + SLOT_W / 2) / 2, 0, zt)),
        "top_en": ((SLOT_W, hw - SLOT_L / 2, WALL), (0, -(hw + SLOT_L / 2) / 2, zt)),
        "top_ep": ((SLOT_W, hw - SLOT_L / 2, WALL), (0, (hw + SLOT_L / 2) / 2, zt)),
        "guide_n": ((GUIDE_T, SLOT_L, GUIDE_D), (-SLOT_W / 2 - GUIDE_T / 2, 0, CASE_H - WALL - GUIDE_D / 2)),
        "guide_p": ((GUIDE_T, SLOT_L, GUIDE_D), (SLOT_W / 2 + GUIDE_T / 2, 0, CASE_H - WALL - GUIDE_D / 2)),
    }
    parts = [(n, m.box(s, c), "case") for n, (s, c) in boxes.items()]
    # the black tambour door, slid open toward the +x end, and a black rim round the slot
    parts.append(("door", m.box((0.024, CASE_W - 0.004, 0.0015), (hl - 0.014, 0, CASE_H + 0.00075)), "black"))
    parts.append(("rim_a", m.box((0.0015, SLOT_L + 0.003, 0.0006), (-SLOT_W / 2 - 0.00075, 0, CASE_H + 0.0003)), "black"))
    parts.append(("rim_b", m.box((0.0015, SLOT_L + 0.003, 0.0006), (SLOT_W / 2 + 0.00075, 0, CASE_H + 0.0003)), "black"))
    cols = [m.col_box(f"{n}_c", s, c) for n, (s, c) in boxes.items()]
    return m.free("case", CASE_POS, parts, cols, LOOKS, mass=0.032, friction=(0.9, 0.8), torsional_patch=0.01)


def _mat():
    parts = [("mat", m.box((MAT_SIZE[0], MAT_SIZE[1], MAT_T), (0, 0, MAT_T / 2)), "mat")]
    lines = []
    for i in range(-10, 11):
        y = i * 0.0127
        if abs(y) < MAT_SIZE[1] / 2 - 0.008:
            lines.append(m.box((MAT_SIZE[0] - 0.016, 0.0006, 0.0002), (0, y, MAT_T + 0.0001)))
    for i in range(-8, 9):
        x = i * 0.0127
        if abs(x) < MAT_SIZE[0] / 2 - 0.008:
            lines.append(m.box((0.0006, MAT_SIZE[1] - 0.016, 0.0002), (x, 0, MAT_T + 0.0001)))
    parts.append(("grid", m.join(*lines), "grid"))
    return m.fixed("mat", (MAT_C[0], MAT_C[1], TOP), parts, [m.col_box("mat_c", (MAT_SIZE[0], MAT_SIZE[1], MAT_T), (0, 0, MAT_T / 2))], LOOKS,
                   friction=(1.0, 0.8))


OBJECTS = [
    _mat(),
    _knife(),
    _segment("seg2", E0 - 2 * PITCH_SEG, False),
    _segment("seg1", E0 - PITCH_SEG, True),
    _case(),
    # distractors: belong on a craft desk, ignored by scoring
    SimReady("mug", pos=(0.46, 0.22, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=-0.6),
    SimReady("tape_roll", pos=(0.45, -0.22, TOP), size=(0.10, 0.10, 0.05), mass=0.06, package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd"),
    SimReady("marker", pos=(0.44, 0.03, TOP), size=(0.109, 0.008, 0.007), mass=0.01, yaw=1.3, package="stationery_marker_thin_blue_a01",
             usd="sm_stationery_marker_thin_blue_a01_01.usd"),
]


# -- runtime joints between the carrier and the two end segments -------------------------------------------------------------------------
_J = {"stage": None, "paths": {}}


def _author_joints(stage, env_path: str) -> None:
    from pxr import Gf, Sdf, UsdPhysics

    blade, s2, s1, frame = f"{env_path}/knife/blade", f"{env_path}/seg2", f"{env_path}/seg1", f"{env_path}/knife/frame"
    for p in (blade, s2, s1, frame):
        if not stage.GetPrimAtPath(p).IsValid():
            kids = [str(c.GetPath()) for c in stage.GetPrimAtPath(env_path).GetChildren()]
            raise RuntimeError(f"snap joints: no prim {p}; {env_path} has {kids}")
    j2 = UsdPhysics.FixedJoint.Define(stage, f"{env_path}/snap_joints/line2")
    j2.CreateBody0Rel().SetTargets([Sdf.Path(blade)])
    j2.CreateBody1Rel().SetTargets([Sdf.Path(s2)])
    j2.CreateLocalPos0Attr().Set(Gf.Vec3f(float(E0 - 2 * PITCH_SEG), 0.0, float(ZB)))
    j2.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    j2.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j2.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    j1 = UsdPhysics.RevoluteJoint.Define(stage, f"{env_path}/snap_joints/line1")
    j1.CreateBody0Rel().SetTargets([Sdf.Path(s2)])
    j1.CreateBody1Rel().SetTargets([Sdf.Path(s1)])
    j1.CreateLocalPos0Attr().Set(Gf.Vec3f(float(PITCH_SEG), 0.0, 0.0))
    j1.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    j1.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j1.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    j1.CreateAxisAttr().Set("Y")
    j1.CreateLowerLimitAttr().Set(-30.0)
    j1.CreateUpperLimitAttr().Set(30.0)
    drive = UsdPhysics.DriveAPI.Apply(j1.GetPrim(), "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(float(BEND_K * math.pi / 180.0))  # USD: per degree
    drive.CreateDampingAttr().Set(float(0.0005 * math.pi / 180.0))
    drive.CreateTargetPositionAttr().Set(0.0)
    drive.CreateMaxForceAttr().Set(10.0)
    for j in (j1, j2):
        j.CreateCollisionEnabledAttr().Set(False)
        j.CreateExcludeFromArticulationAttr().Set(True)
        j.CreateJointEnabledAttr().Set(True)
    for s in (s1, s2):  # the segments slide inside the handle with the carrier: never collide with the knife itself
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(s))
        fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(frame))
        fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(blade))
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(s1))
    fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(s2))
    _J.update(stage=stage, paths={"line1": str(j1.GetPath()), "line2": str(j2.GetPath())})
    print(f"[snap] joints authored: {_J['paths']}", flush=True)


def _set_joint(name: str, on: bool) -> None:
    from pxr import UsdPhysics

    if _J["stage"] is None:
        return
    UsdPhysics.Joint(_J["stage"].GetPrimAtPath(_J["paths"][name])).GetJointEnabledAttr().Set(bool(on))


def _patch_interactive_scene(_module=None) -> None:
    try:
        from isaaclab.scene import interactive_scene as isc
    except Exception:  # offline: no Isaac Lab
        return
    cls = isc.InteractiveScene
    if getattr(cls, "_snap_joints", False):
        return
    init0 = cls.__init__

    def __init__(self, *a, **k):
        init0(self, *a, **k)
        _author_joints(self.stage, self.env_prim_paths[0])

    cls.__init__ = __init__
    cls._snap_joints = True


_scene_patch = m._Finder.targets["yam_arena.scene"]


def _scene_and_joints(module):
    _scene_patch(module)
    _patch_interactive_scene()


m._Finder.targets["yam_arena.scene"] = _scene_and_joints
if "yam_arena.scene" in sys.modules:
    _patch_interactive_scene()


# -- geometry the scorer and the demo both use ---------------------------------------------------------------------------------------------
def exposure(knife) -> float:
    """How far the blade's original tip line stands past the nose (m): E0 plus the carrier's travel."""
    return E0 + knife.joint("blade")


def line_point(knife, k: int):
    """The centre of score line k (0 = the original tip line) on the blade's mid-plane, in the world."""
    return knife.to_world(np.array([[exposure(knife) - PITCH_SEG * k, 0.0, ZB]]))[0]


def _box_corners(lo, hi):
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


HANDLE_LO, HANDLE_HI = np.array([-L_HANDLE, -W_HANDLE / 2 - 0.005, 0.0]), np.array([0.0, W_HANDLE / 2, T_HANDLE])
CASE_LO, CASE_HI = np.array([-CASE_L / 2, -CASE_W / 2, 0.0]), np.array([CASE_L / 2, CASE_W / 2, CASE_H])


def _gap_to_box(state, points, lo, hi) -> float:
    """The least distance from world points to a box in a part's frame (0 inside)."""
    local = state.to_local(points)
    d = np.maximum(np.maximum(lo - local, local - hi), 0.0)
    return float(np.min(np.linalg.norm(d, axis=1)))


def _seg_corners(seg):
    pts = [(x, y, z) for (x, y) in _seg_outline(0.0, PITCH_SEG) for z in (-T_BLADE / 2, T_BLADE / 2)]
    return seg.to_world(np.array(pts))


def _bend_deg(s2, s1) -> float:
    rel = s2.rotation().T @ s1.rotation()
    return math.degrees(math.atan2(rel[0, 2], rel[0, 0]))


def _still(s, v=0.02, w=0.3) -> bool:
    return bool(np.linalg.norm(s.lin_vel) < v and np.linalg.norm(s.ang_vel) < w)


def _on_mat_xy(p) -> bool:
    return bool(abs(p[0] - MAT_C[0]) <= MAT_SIZE[0] / 2 and abs(p[1] - MAT_C[1]) <= MAT_SIZE[1] / 2)


# -- the tracker -------------------------------------------------------------------------------------------------------------------------
class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.one = False  # snapped at score line 1
        self.two = False  # snapped at score line 2 (two segments)
        self.fell = False
        self.hold, self.full, self.score = 0, False, 0.0
        self.notch = None
        self.info = {}
        if env is not None:
            _set_joint("line1", True)
            _set_joint("line2", True)

    def _detent(self, env, knife):
        e = exposure(knife)
        n = round(e / NOTCH) * NOTCH
        if self.notch is None or abs(n - self.notch) > 1e-6:
            self.notch = n
            m.set_target(env, "knife", "blade", n - E0)

    def step(self, env):
        knife, s1, s2, case = (m.state(env, n) for n in ("knife", "seg1", "seg2", "case"))
        self._detent(env, knife)
        e = exposure(knife)
        if not (self.one or self.two):
            bend = _bend_deg(s2, s1)
            self.info["bend"] = bend
            if abs(bend) >= BREAK_DEG and e - PITCH_SEG > 0.001:
                if e - 2 * PITCH_SEG > 0.001:  # line 2 also past the nose: the break comes at line 2
                    self.two = True
                    _set_joint("line2", False)
                    m.push(env, "seg2", (0.0, 0.0, -0.002 * 9.81))
                    m.push(env, "seg1", (0.0, 0.0, -0.002 * 9.81))
                else:
                    self.one = True
                    _set_joint("line1", False)
                    m.push(env, "seg1", (0.0, 0.0, -0.002 * 9.81))
                print(f"[snap] broke at line {2 if self.two else 1}: bend {bend:.1f} deg, exposure {e * 1000:.1f} mm", flush=True)
        if knife.lin_vel[2] < -FALL_V:
            self.fell = True
        corners = knife.to_world(_box_corners(HANDLE_LO, HANDLE_HI))
        centre = knife.to_world(np.array([[-L_HANDLE / 2, 0.0, ZB]]))[0]
        tips = m.fingertips(env)
        knife_rest = bool(abs(corners[:, 2].min() - MAT) <= 0.003 and _still(knife) and _on_mat_xy(centre))
        hands_knife = _gap_to_box(knife, tips, HANDLE_LO, HANDLE_HI) >= 0.05
        retracted = e <= E_RETRACTED
        local = case.to_local(_seg_corners(s1))
        seg_in = bool(np.all(np.abs(local[:, 0]) <= CASE_L / 2) and np.all(np.abs(local[:, 1]) <= CASE_W / 2)
                      and np.all(local[:, 2] >= 0.0) and np.all(local[:, 2] <= CASE_H))
        case_ok = bool(case.tilt_deg() <= 10.0 and abs(case.pos[2] - MAT) <= 0.003 and _still(case))
        hands_case = _gap_to_box(case, tips, CASE_LO, CASE_HI) >= 0.05
        harm = self.two or self.fell or not _on_mat_xy(centre)
        good = self.one and not harm
        full_now = good and retracted and knife_rest and hands_knife and seg_in and case_ok and hands_case
        self.hold = self.hold + 1 if full_now else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if not good:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif retracted and knife_rest and hands_knife:
            self.score = 0.5
        else:
            self.score = 0.25
        self.info.update(e=e, retracted=retracted, knife_rest=knife_rest, hands_knife=hands_knife, seg_in=seg_in, case_ok=case_ok,
                         hands_case=hands_case, harm=harm, one=self.one, two=self.two, fell=self.fell)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- the demonstration -------------------------------------------------------------------------------------------------------------------
def _report(ctx, what):
    k = ctx.state("knife")
    s1 = ctx.state("seg1")
    print(f"[demo] {what} e={exposure(k) * 1000:.1f}mm knife={np.round(k.pos, 4)} tilt={k.tilt_deg():.1f} seg1={np.round(s1.pos, 4)} "
          f"case={np.round(ctx.pos('case'), 4)} trial={ {a: (round(b, 3) if isinstance(b, float) else b) for a, b in TRIAL.info.items()} } score={TRIAL.score}",
          flush=True)


DEBUG = True
STOP = __import__("os").environ.get("SNAP_STOP", "")


def _tips(ctx, what):
    if DEBUG:
        t = m.fingertips(ctx.emb._env)
        for side, tt in (("left", t[:12]), ("right", t[12:])):
            print(f"[tips] {what} {side}: x {tt[:, 0].min():.4f}..{tt[:, 0].max():.4f} y {tt[:, 1].min():.4f}..{tt[:, 1].max():.4f} "
                  f"z {tt[:, 2].min():.4f} knife={np.round(ctx.pos('knife'), 4)}", flush=True)
FINGER_HALF_T = 0.0062  # the closed fingers' half thickness along the jaw axis, within 15 mm of their tips (from the collision meshes)
FINGER_HALF_W = 0.0077  # and their half width
PAD_TIP = 0.013  # pad tips past the TCP


def demo_script(ctx):
    """Right hand: holds the knife by the rear of its handle throughout. Left hand: pushes the thumb slider back until only score
    line 1 stands past the nose, then holds the disposal can; the right lifts the knife, turns it nose down over the can's slot,
    puts the tip in up to line 1 and tilts the handle until the segment snaps off into the can; it lays the knife back on the mat,
    the left pushes the slider back until the blade is inside the nose, and both let go.
    Red-team modes: none (nothing), careless (snaps without first setting the exposure), onearm (the can not held),
    loose (snaps the segment against the mat instead of in the can), extended (puts the knife down without retracting)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "careless", "onearm", "loose", "extended"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for _ in range(80):
            yield
        _report(ctx, "end:")
        return
    up = np.array([0.0, 0.0, 1.0])
    knife = lambda: ctx.state("knife")  # noqa: E731
    kpt = lambda p: knife().to_world(np.array([p]))[0]  # noqa: E731

    # 1. right grips the rear of the handle; left comes over the thumb slider with its fingers closed
    z_grip = 0.017
    grip_pt = kpt((GRIP_X, 0.0, 0.0))
    btn = lambda: kpt((exposure(knife()) - BTN_FRONT, -W_HANDLE / 2, ZB))  # noqa: E731  the slider's front face at the spine

    def push_at(ahead):
        """The left TCP with its closed fingers (12 x 15 mm near the tips) beside the spine, 1.5 mm off it, their
        front face `ahead` m rearward of the slider's front face, pads 3 mm above the mat."""
        k = knife()
        rot = k.rotation()
        rear = -rot[:, 0]
        rear = np.array([rear[0], rear[1], 0.0]) / max(1e-9, math.hypot(rear[0], rear[1]))
        side = -rot[:, 1]  # the spine (part -y) faces this way
        side = np.array([side[0], side[1], 0.0]) / max(1e-9, math.hypot(side[0], side[1]))
        b = btn()
        p = b + rear * (ahead - FINGER_HALF_W) + side * (0.0015 + FINGER_HALF_T)
        return np.r_[p[:2], MAT + PAD_TIP + 0.003]

    yield from ctx.together(
        r.move(np.r_[grip_pt[:2], MAT + 0.08], grip=d.down(0.0), finger=FINGER_OPEN, t=2.5, label="right over the handle"),
        l.move(push_at(-0.006) + np.array([0.0, 0.0, 0.04]), grip=d.down(0.0), finger=0.0, t=2.5, label="left over the slider"),
    )
    yield from r.servo(np.r_[grip_pt[:2], MAT + 0.08], timeout=1.0, tol=0.001, label="right over the handle")
    yield from r.move(np.r_[grip_pt[:2], MAT + z_grip], t=1.0, label="right round the handle")
    _tips(ctx, "right round the handle")
    yield from r.close(W_HANDLE, 25.0, t=0.6, label="right grips the handle")
    _report(ctx, "gripped:")
    if STOP == "grip":
        return

    def push_slider(target_e, label):
        p = push_at(-0.006)
        yield from l.move(p + np.array([0, 0, 0.03]), t=1.0, label=f"{label}: over")
        yield from l.servo(p + np.array([0, 0, 0.03]), timeout=1.0, tol=0.001, label=f"{label}: over")
        yield from l.move(push_at(-0.006), t=0.8, label=f"{label}: down")

        def done():
            k = knife()
            if DEBUG:
                tips = m.fingertips(ctx.emb._env)[:12]
                print(f"[push] e={exposure(k) * 1000:.2f} btn={np.round(btn(), 4)} tipsx={tips[:, 0].min():.4f}..{tips[:, 0].max():.4f} "
                      f"tipsy={tips[:, 1].min():.4f}..{tips[:, 1].max():.4f} tipsz={tips[:, 2].min():.4f} knife={np.round(k.pos, 4)}", flush=True)
            return exposure(k) <= target_e + 0.0006
        yield from l.follow(lambda: push_at(0.004), t=8.0, speed=0.01, until=done, label=f"{label}: push")
        yield from l.move(push_at(-0.006), t=0.4, label=f"{label}: back off")
        yield from l.move(l.at() + np.array([0.0, 0.0, 0.05]), t=0.6, label=f"{label}: up")
        _report(ctx, f"{label}:")

    if mode != "careless":
        yield from push_slider(0.020, "slider to one segment out")
    if STOP == "push":
        return

    case = ctx.pos("case")
    hold_case = mode not in ("onearm",)
    # 2. left takes the can by its two ends, reaching in from the +y side, while the right still holds the knife on the mat
    if hold_case:
        g = np.array([case[0], case[1] + 0.004, MAT + 0.030])
        yield from l.move(g + np.array([0.0, 0.05, 0.05]), grip=d.tilted(60.0, -90.0), finger=FINGER_OPEN, t=1.8, label="left toward the can")
        yield from l.servo(g + np.array([0.0, 0.05, 0.05]), timeout=1.0, tol=0.0015, label="left toward the can")
        yield from l.move(g, t=1.0, label="left round the can")
        yield from l.servo(g, timeout=0.6, tol=0.0015, label="left round the can")
        yield from l.close(CASE_L, 15.0, t=0.6, label="left holds the can")
        print(f"[demo] can held: case={np.round(ctx.pos('case'), 4)} tilt={ctx.state('case').tilt_deg():.1f}", flush=True)
    else:
        yield from l.move(np.array([0.25, 0.20, MAT + 0.12]), t=1.5, label="left away")
    # 3. right lifts the knife and turns it nose down over the slot, the tip a few centimetres above it
    yield from r.move(r.at() + up * 0.12, t=1.2, label="lift the knife")
    if mode == "loose":
        yield from _snap_on_mat(ctx, r, knife, kpt)
    else:
        cs = ctx.state("case")
        slot = cs.to_world(np.array([[0.0, 0.0, CASE_H]]))[0]
        axis = cs.rotation()[:, 1]
        axis = np.array([axis[0], axis[1], 0.0]) / max(1e-9, math.hypot(axis[0], axis[1]))
        if axis[1] < 0:
            axis = -axis
        psi = math.degrees(math.atan2(axis[1], axis[0])) - 90.0
        over = slot + np.array([-ZB * math.cos(math.radians(psi)), -ZB * math.sin(math.radians(psi)), 0.175])
        yield from r.move(over, grip=d.level(psi), t=2.5, label="right turns the knife nose down over the slot")
        _report(ctx, "nose down:")
        line1 = lambda: line_point(knife(), 1)  # noqa: E731
        yield from r.servo(slot + up * 0.025, read=line1, timeout=3.0, tol=0.0015, label="line 1 over the slot")
        _report(ctx, "over the slot:")
        yield from r.servo(lambda t: slot + up * max(0.0006, 0.025 - 0.02 * t), read=line1, timeout=4.0, tol=0.0012, label="tip in to line 1")
        _report(ctx, "tip in:")
        pivot = slot.copy()
        turned = 0.0
        for _ in range(20):
            if _snapped_visibly(ctx):
                break
            yield from r.arc(pivot, axis, 2.0, t=0.25, label="tilt the handle")
            turned += 2.0
        yield from r.wait(0.3)
        _report(ctx, f"after the bend ({turned:.0f} deg):")
        yield from r.arc(pivot, axis, -turned, t=1.0, label="handle upright")
        yield from r.move(r.at() + up * 0.05, t=0.8, label="blade out of the slot")
        if hold_case:
            yield from l.open(t=0.4, label="left lets go of the can")
            yield from l.move(l.at() + np.array([0.0, 0.06, 0.05]), t=0.8, label="left away from the can")
    # 3. right lays the knife back on the mat and holds it; left pushes the slider back until the blade is inside
    home = np.r_[grip_pt[:2], MAT + z_grip]
    yield from r.move(home + up * 0.10, grip=d.down(0.0), t=2.5, label="knife back over the mat")
    yield from r.move(home + up * 0.004, t=1.2, label="knife down onto the mat")
    yield from r.wait(0.3)
    _report(ctx, "knife down:")
    if mode != "extended":
        yield from push_slider(0.0075, "slider fully back")
    yield from l.move(np.array([0.22, 0.22, MAT + 0.15]), t=1.5, label="left clear")
    yield from r.open(t=0.5, label="right lets go")
    yield from r.move(r.at() + up * 0.10, t=1.0, label="right clear")
    yield from r.move(np.array([0.20, -0.22, MAT + 0.15]), t=1.2, label="right away")
    for _ in range(40):
        yield
    _report(ctx, "end:")


def _snapped_visibly(ctx) -> bool:
    """What a person sees: the end segment no longer lines up with the blade (it has dropped or swung free)."""
    k = ctx.state("knife")
    s1 = ctx.state("seg1")
    expect = line_point(k, 1)
    return bool(np.linalg.norm(np.asarray(s1.pos) - expect) > 0.004)


def _snap_on_mat(ctx, r, knife, kpt):
    """Red team: press the tip flat on the mat and lever the handle up (OLFA's 'use the ground' fallback): the segment ends loose."""
    yield from r.move(r.at() + np.array([0.0, 0.0, -0.10 + 0.006]), t=1.0, label="tip back on the mat")
    tip = line_point(knife(), 1)
    for _ in range(15):
        if _snapped_visibly(ctx):
            break
        yield from r.arc(tip, (0.0, 1.0, 0.0), -2.0, t=0.25, label="lever the handle up")
    yield from r.wait(0.3)
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.03]), t=0.6, label="lift")
