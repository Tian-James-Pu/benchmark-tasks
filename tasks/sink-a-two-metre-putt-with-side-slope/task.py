"""A 2.00 m sidehill putt (ball centre to hole centre) on an indoor putting green.

The green is a 3.05 x 0.61 m platform (two-by-four rails on edge, 18 mm plywood, 10 mm putting turf) that runs along
x from the robot table onto a second table of the same height in front of it. One long rail sits on a 21 mm shim, so
the turf falls 2 degrees across the putt, towards -y. A regulation hole (108 mm across, 102 mm deep cup) is cut 2.00 m
straight down the green from the ball. A right-handed 35-inch blade putter lies face down on the green, beside the
ball, its head towards the arms. A golf ball sits on the turf in front of the left arm. A mug, a paper cup and a folded
golf towel stand on the robot table beside the green.

Physics that matters: the ball is a 42.67 mm, 45.9 g rigid sphere; the turf's rolling resistance is a force opposing
the ball's rolling, 0.0785 times its weight (a deceleration of 0.55 m/s^2, a stimpmeter reading of 10 ft), which also
holds a stopped ball on the slope as real turf does. Restitution is 0.75 between ball and putter face. The hole and
cup are real geometry: a ball drops in or lips out by contact alone.
"""

import math
import os
import pathlib
import sys
from dataclasses import replace

import numpy as np

from yam_arena import demo as _demo
from yam_arena import ik as _ik
from yam_arena import station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
import simready as _sr  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the camera: a fixed view from beside the green, so that the whole 2 m putt, the hole and the arms are in frame ----

CAMERA_POS = (1.42, -1.45, 2.00)
CAMERA_TARGET = (1.38, 0.40, 0.84)
_base_rig = getattr(station.rig, "_putt_base", station.rig)


def _putt_rig(top_cam="calibrated"):
    rig = _base_rig(top_cam)
    forward = np.asarray(CAMERA_TARGET, float) - np.asarray(CAMERA_POS, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rot = np.column_stack((right, up, -forward))
    camera = replace(rig.top_cam, pos=CAMERA_POS, rot_xyzw=tuple(float(v) for v in station.matrix_to_quat_xyzw(rot)), convention="opengl")
    return replace(rig, name="station-putting-green", top_cam=camera)


_putt_rig._putt_base = _base_rig
station.rig = _putt_rig

# -- the green: a platform frame with x along the putt, y across it (0 at the low rail), z up from the table -------------

TILT = math.radians(2.0)  # the side slope: the +y rail is shimmed up, so the turf falls towards -y
PLAT = np.array([0.08, 0.05, TOP])  # the low rail's near bottom corner, on the table
LP, WP, TP = 3.05, 0.61, 0.117  # platform length, width, height to the turf's top at the low rail
SLAB = 0.030  # turf and plywood: the top 30 mm
RAIL = 0.038  # a two-by-four's thickness
R_BALL = 0.021335  # 1.68 in
M_BALL = 0.04593  # 1.62 oz
R_HOLE = 0.054  # 4.25 in across
CUP_DEPTH = 0.102  # 4 in
BALL_L = np.array([0.36, 0.305])  # the ball's spot on the green
HOLE_L = BALL_L + np.array([2.00, 0.0])  # the hole, 2.00 m straight down the green
MU_ROLL = 0.0785  # rolling resistance over weight: 0.55 m/s^2 for a rolling ball, a stimp of 10 ft
G = 9.81
DT = 0.05  # one control step


def _tilt(v):
    """A point or direction in the platform frame, turned by the side slope (about the platform's x axis)."""
    c, s = math.cos(TILT), math.sin(TILT)
    v = np.asarray(v, float)
    return np.array([v[..., 0], c * v[..., 1] - s * v[..., 2], s * v[..., 1] + c * v[..., 2]]).T


def to_world(p):
    return _tilt(p) + PLAT


NORMAL = _tilt([0.0, 0.0, 1.0])  # the turf's upward normal
TURF_PT = to_world([0.0, 0.0, TP])  # a point on the turf's plane
HOLE = to_world([HOLE_L[0], HOLE_L[1], TP])  # the hole's centre, on the turf's surface
BALL_START = to_world([BALL_L[0], BALL_L[1], TP]) + R_BALL * NORMAL  # the ball's centre at the start


def turf_z(x, y):
    """The turf's height at world (x, y)."""
    return float(TURF_PT[2] - (NORMAL[0] * (x - TURF_PT[0]) + NORMAL[1] * (y - TURF_PT[1])) / NORMAL[2])


def _tm(mesh):
    """A platform-frame mesh, turned by the slope (the part's origin is PLAT)."""
    return m.Mesh(_tilt(mesh.v), mesh.f)


def _green():
    parts, cols = [], []
    z0, z1 = TP - SLAB, TP
    hx, hy, sq = HOLE_L[0], HOLE_L[1], 0.15
    # the turf slab, in four boards around a 30 cm square about the hole
    for k, (x0, x1, y0, y1) in enumerate([(0.0, hx - sq, 0.0, WP), (hx + sq, LP, 0.0, WP), (hx - sq, hx + sq, 0.0, hy - sq), (hx - sq, hx + sq, hy + sq, WP)]):
        b = m.box((x1 - x0, y1 - y0, SLAB), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
        parts.append((f"turf{k}", _tm(b), "turf"))
        cols.append(m.col_hull(f"turf{k}_c", _tm(b)))
    # inside the square: 24 wedges from the square's edge in to the hole's 24-sided edge (apothem R_HOLE)
    n = 24
    rv = R_HOLE / math.cos(math.pi / n)

    def edge(a):
        d = np.array([math.cos(a), math.sin(a)])
        return d * (sq / max(abs(d[0]), abs(d[1])))

    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        out = [rv * np.array([math.cos(a0), math.sin(a0)]), edge(a0)]
        for corner in ((sq, sq), (-sq, sq), (-sq, -sq), (sq, -sq)):
            ca = math.atan2(corner[1], corner[0]) % (2 * math.pi)
            if a0 < ca < a1:
                out.append(np.array(corner, float))
        out += [edge(a1), rv * np.array([math.cos(a1), math.sin(a1)])]
        w = m.prism(np.array(out) + (hx, hy), z0, z1)
        parts.append((f"lip{i}", _tm(w), "turf"))
        cols.append(m.col_hull(f"lip{i}_c", _tm(w)))
        # the cup's wall below the slab, and its bottom
        mid = (a0 + a1) / 2
        chord = 2 * rv * math.sin(math.pi / n) * 1.08
        wall = m.box((0.004, chord, z0 - (TP - CUP_DEPTH)), (R_HOLE + 0.002, 0.0, (z0 + TP - CUP_DEPTH) / 2)).turned(math.degrees(mid), "z").moved((hx, hy, 0.0))
        parts.append((f"cup{i}", _tm(wall), "cup"))
        cols.append(m.col_hull(f"cup{i}_c", _tm(wall)))
    bottom = m.cylinder(R_HOLE + 0.004, 0.005, (hx, hy, TP - CUP_DEPTH - 0.0025))
    parts.append(("cup_bottom", _tm(bottom), "cup"))
    cols.append(m.col_hull("cup_bottom_c", _tm(bottom)))
    # the frame: two long rails and two end rails, on edge, under the plywood
    for k, (cx, cy, sx, sy) in enumerate([(LP / 2, RAIL / 2, LP, RAIL), (LP / 2, WP - RAIL / 2, LP, RAIL), (RAIL / 2, WP / 2, RAIL, WP - 2 * RAIL), (LP - RAIL / 2, WP / 2, RAIL, WP - 2 * RAIL)]):
        b = m.box((sx, sy, z0), (cx, cy, z0 / 2))
        parts.append((f"rail{k}", _tm(b), "pine"))
        cols.append(m.col_hull(f"rail{k}_c", _tm(b)))
    # the 21 mm shim strip under the high rail
    shim_h = WP * math.tan(TILT)
    shim = m.box((LP, RAIL, shim_h), (LP / 2, WP - RAIL / 2, -shim_h / 2))
    parts.append(("shim", m.Mesh(shim.v, shim.f), "pine"))
    looks = {"turf": m.Look(m.rgb(0.20, 0.45, 0.16), roughness=0.95), "cup": m.Look(m.rgb(0.93, 0.93, 0.92), roughness=0.5),
             "pine": m.Look(m.rgb(0.80, 0.66, 0.45), roughness=0.8)}
    return m.fixed("green", tuple(PLAT), parts, cols, looks, friction=(1.0, 0.8))


def _side_table():
    """The second table, the robot table's height, that carries the far 2.2 m of the green."""
    top = m.box((2.30, 1.20, 0.06), (0.0, 0.0, -0.03))
    legs = [m.box((0.05, 0.05, TOP - 0.06), (sx * 1.08, sy * 0.53, -0.06 - (TOP - 0.06) / 2)) for sx in (-1, 1) for sy in (-1, 1)]
    parts = [("top", top, "oak")] + [(f"leg{i}", g, "oak") for i, g in enumerate(legs)]
    return m.fixed("side_table", (2.07, 0.35, TOP), parts, [m.col_box("top_c", (2.30, 1.20, 0.06), (0.0, 0.0, -0.03))],
                   {"oak": m.Look(m.rgb(0.62, 0.47, 0.32), roughness=0.7)})


# -- the putter: a right-handed 35-inch blade. Its frame: x out of the face (the putt's direction), y towards the heel,
# z up from the sole, origin at the sole's centre, as it sits at address with its 70-degree lie --------------------------

LIE = math.radians(70.0)
HEAD = (0.025, 0.105, 0.024)  # blade head: 25 mm deep, 105 mm heel to toe, 24 mm high
SHAFT_DIR = np.array([0.0, math.cos(LIE), math.sin(LIE)])  # head to grip, leaning towards the heel
P0 = np.array([0.0, 0.045, HEAD[2]])  # where the shaft enters the heel
SHAFT_LEN, GRIP_LEN = 0.60, 0.267  # steel shaft, then a 10.5 in grip: 0.89 m (35 in) from the sole along the shaft
R_SHAFT, R_GRIP = 0.005, 0.0135
M_HEAD, M_SHAFT, M_GRIP = 0.350, 0.120, 0.065
P1 = P0 + SHAFT_LEN * SHAFT_DIR
P2 = P1 + GRIP_LEN * SHAFT_DIR
COM = (M_HEAD * np.array([0.0, 0.0, HEAD[2] / 2]) + M_SHAFT * (P0 + P1) / 2 + M_GRIP * (P1 + P2) / 2) / (M_HEAD + M_SHAFT + M_GRIP)
GRASP = P0 + float((COM - P0) @ SHAFT_DIR) * SHAFT_DIR  # where the demo holds the shaft: level with the putter's centre of mass, 14 cm up from the heel


def _putter_parts():
    head = m.box(HEAD, (0.0, 0.0, HEAD[2] / 2))
    shaft = m.cylinder(R_SHAFT, SHAFT_LEN, (0, 0, 0), segments=16)
    grip = m.cylinder(R_GRIP, GRIP_LEN, (0, 0, 0), segments=16)

    def along(mesh, start, length):
        # a z-aligned mesh centred on the origin, laid along SHAFT_DIR from `start`
        tilt = -(90.0 - math.degrees(LIE))  # about x: z towards +y
        return mesh.turned(tilt, "x").moved(start + 0.5 * length * SHAFT_DIR)

    sh, gr = along(shaft, P0, SHAFT_LEN), along(grip, P1, GRIP_LEN)
    line = m.box((0.002, 0.003, 0.0005), (-HEAD[0] / 2 + 0.006, 0.0, HEAD[2] + 0.00025))  # a white sight line on the top
    parts = [("head", head, "steel"), ("shaft", sh, "chrome"), ("grip", gr, "rubber"), ("sight", line, "white")]
    cols = [m.col_hull("head_c", head), m.col_hull("shaft_c", sh), m.col_hull("grip_c", gr)]
    looks = {"steel": m.Look(m.rgb(0.55, 0.56, 0.58), metallic=0.8, roughness=0.35), "chrome": m.Look(m.rgb(0.80, 0.80, 0.82), metallic=0.9, roughness=0.2),
             "rubber": m.Look(m.rgb(0.08, 0.08, 0.09), roughness=0.85), "white": m.Look(m.rgb(0.95, 0.95, 0.95), roughness=0.5)}
    return parts, cols, looks


def _grip_rot(approach, closing):
    a, c = np.asarray(approach, float), np.asarray(closing, float)
    return np.column_stack([c, np.cross(a, c), a])


# The demo's hold on the shaft at address: fingers forward along the putt (+x), closing across the shaft.
_STROKE_GRIP0 = _grip_rot((1.0, 0.0, 0.0), (0.0, -math.sin(LIE), math.cos(LIE)))
_PICK_GRIP = _demo.grip_rotation(0.0)  # straight down, fingers closing along y
_R_TILT = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(TILT), -math.sin(TILT)], [0.0, math.sin(TILT), math.cos(TILT)]])
_LYING_ROT = _R_TILT @ _PICK_GRIP @ _STROKE_GRIP0.T  # the putter as it lies face down on the green, held as it will be at address
PICK_XY = np.array([0.34, 0.205])  # the shaft's grasp point as the putter lies (world x, y)


def _quat_xyzw(R):
    return tuple(float(v) for v in station.matrix_to_quat_xyzw(R))


def _putter():
    parts, cols, looks = _putter_parts()
    # lying face down: the face plane (x = +HEAD[0]/2) on the turf, so the frame's origin is HEAD[0]/2 above the turf
    grasp_w = np.array([PICK_XY[0], PICK_XY[1], turf_z(*PICK_XY)]) + (HEAD[0] / 2 + 0.002) * NORMAL
    origin = grasp_w - _LYING_ROT @ GRASP
    part = m.free("putter", tuple(origin), parts, cols, looks, mass=M_HEAD + M_SHAFT + M_GRIP, friction=(0.9, 0.8), start_quat=_quat_xyzw(_LYING_ROT), com=tuple(COM))
    build = part.build
    return replace(part, build=lambda: build().replace("float physics:restitution = 0", "float physics:restitution = 0.75"))


# -- the ball: NVIDIA SimReady's golf ball, colliding as an exact sphere -----------------------------------------------

_orig_prepare = _sr._prepare


def _prepare(prim, spec):
    """After the loader's own set-up: the golf ball's 6,146-point SDF collider is switched off and an exact 42.67 mm
    sphere collides instead (a rolling ball must be round), with the ball's mass, a solid sphere's inertia, and a
    material of restitution 0.75 against the putter face."""
    _orig_prepare(prim, spec)
    if spec.name != "ball":
        return
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    while instances := [p for p in Usd.PrimRange(prim) if p.IsInstance()]:
        for p in instances:
            p.SetInstanceable(False)
    body = [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)][0]
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    t = Usd.TimeCode.Default()
    centre_w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(t).Transform(Gf.Vec3d(0.0, 0.0, 0.0215))
    centre_b = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(t).GetInverse().Transform(centre_w)
    sphere = UsdGeom.Sphere.Define(stage, body.GetPath().AppendChild("ball_collider"))
    sphere.CreateRadiusAttr(R_BALL)
    sphere.AddTranslateOp().Set(Gf.Vec3d(centre_b))
    sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    mat = UsdShade.Material.Define(stage, prim.GetPath().AppendChild("BallPhysicsMaterial"))
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(0.4)
    api.CreateDynamicFrictionAttr(0.35)
    api.CreateRestitutionAttr(0.75)
    UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")
    mass = UsdPhysics.MassAPI.Apply(body)
    mass.CreateMassAttr(M_BALL)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*[float(v) for v in centre_b]))
    i = 0.4 * M_BALL * R_BALL**2
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(i, i, i))


_sr._prepare = _prepare

OBJECTS = [
    _green(),
    _side_table(),
    SimReady("ball", pos=tuple(BALL_START - np.array([0.0, 0.0, R_BALL - 0.0015])), size=(0.043, 0.043, 0.043), mass=M_BALL,
             package="misc_golfBall_a01", usd="sm_misc_golfBall_a01_01.usd"),
    _putter(),
    # distractors, on the robot table beside the green: none is in the putt's path or the arms' way
    SimReady("mug", pos=(0.62, -0.30, TOP), size=(0.114, 0.090, 0.080), mass=0.32, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.2),
    SimReady("paper_cup", pos=(0.78, -0.12, TOP), size=(0.08, 0.08, 0.11), mass=0.012, package="dishware_cup_paper_a01", usd="sm_dishware_cup_paper_a01_01.usd"),
    SimReady("towel", pos=(0.45, -0.52, TOP), size=(0.205, 0.212, 0.034), mass=0.06, package="consumable_microfiber_towel_folded_small_b01",
             usd="sm_consumable_microfiber_towel_folded_small_b01_01.usd", yaw=0.4),
]

# -- the turf's rolling resistance, and the trial's record ---------------------------------------------------------------


def _ball_now(env):
    """The ball's centre and velocity (its centre of mass), station frame."""
    a = env.unwrapped.scene["ball"]
    origin = m._np(env.unwrapped.scene.env_origins[0])
    d = a.data
    pos = m._np(getattr(d, "root_com_pos_w", d.root_pos_w))[0] - origin
    vel = m._np(getattr(d, "root_com_lin_vel_w", d.root_lin_vel_w))[0]
    return pos, vel


def _on_plane(c):
    """(height of the ball's centre above the turf's plane, radial distance from the hole's axis in that plane)."""
    rel = c - HOLE
    h = float(rel @ NORMAL)
    radial = float(np.linalg.norm(rel - h * NORMAL))
    return float((c - TURF_PT) @ NORMAL), radial


def _on_green_xy(c):
    p = c - PLAT
    return 0.0 <= p[0] <= LP and 0.0 <= p[1] <= WP * math.cos(TILT)


V_STOP = 0.04  # below this the turf holds the ball, as static rolling resistance does
_BALL = {"prev": None, "v": np.zeros(3), "anchor": None}  # the centre at the last step, the velocity over the step, where the turf holds it


def _hold_force():
    g = np.array([0.0, 0.0, -G * M_BALL])
    return -(g - (g @ NORMAL) * NORMAL)


def _place_ball(env):
    """Reset: the ball on its spot, at rest, held by the turf (the settling steps before the first frame run without
    the turf's resistance, so the ball is put back where the layout says)."""
    import torch

    a = env.unwrapped.scene["ball"]
    c, _ = _ball_now(env)
    pose = a.data.root_link_pose_w.clone() if hasattr(a.data, "root_link_pose_w") else a.data.root_pose_w.clone()
    pose[0, :3] += torch.tensor(BALL_START - c, device=pose.device, dtype=pose.dtype)
    (a.write_root_link_pose_to_sim if hasattr(a, "write_root_link_pose_to_sim") else a.write_root_pose_to_sim)(pose)
    a.write_root_velocity_to_sim(torch.zeros((1, 6), device=pose.device, dtype=pose.dtype))
    _BALL["prev"], _BALL["v"], _BALL["anchor"] = BALL_START.copy(), np.zeros(3), BALL_START.copy()
    m.push(env, "ball", tuple(float(x) for x in _hold_force()))


def _roll(env):
    c, _ = _ball_now(env)
    v = (c - _BALL["prev"]) / DT if _BALL["prev"] is not None else np.zeros(3)
    _BALL["prev"], _BALL["v"] = c, v
    height, radial = _on_plane(c)
    if not (abs(height - R_BALL) < 0.004 and radial > R_HOLE + 0.002 and _on_green_xy(c)):
        m.push(env, "ball", (0.0, 0.0, 0.0))
        return
    vt = v - (v @ NORMAL) * NORMAL
    speed = float(np.linalg.norm(vt))
    fmax = MU_ROLL * M_BALL * G * NORMAL[2]
    if speed > V_STOP:
        f = -fmax * vt / speed
        _BALL["anchor"] = None
    else:
        if _BALL["anchor"] is None:
            _BALL["anchor"] = c.copy()
        off = c - _BALL["anchor"]
        f = _hold_force() - 1.4 * M_BALL * vt / DT - 8.0 * (off - (off @ NORMAL) * NORMAL)  # the ball sits in its own small dent in the turf
        n = float(np.linalg.norm(f))
        if n > fmax:
            f *= fmax / n
    m.push(env, "ball", tuple(float(x) for x in f))


def _putter_gap(env, c):
    """The ball centre's distance to the putter's surface (head block, shaft and grip)."""
    s = m.state(env, "putter")
    p = s.to_local(c)[0]
    half = np.array(HEAD) / 2
    d_head = float(np.linalg.norm(np.maximum(np.abs(p - np.array([0.0, 0.0, half[2]])) - half, 0.0)))

    def seg(a, b, r):
        ab = b - a
        t = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
        return float(np.linalg.norm(p - (a + t * ab))) - r

    return min(d_head, seg(P0, P1, R_SHAFT), seg(P1, P2, R_GRIP))


HOLD_STEPS = 20  # 1 s at rest
FREE_ROLL = 0.30  # once the ball has rolled this far from its spot, any touch of the putter is a second stroke
HAND_GAP = 0.012  # a fingertip point this close to the ball's surface is a touch by hand
CLEAR = 0.05  # full credit needs every fingertip point this far from the ball


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place where a recorded foul is cleared
        self.foul, self.hold, self.full, self.score, self.holed = None, 0, False, 0.0, False

    def step(self, env):
        c, v = _BALL["prev"], _BALL["v"]  # read by _roll this step
        tips = m.fingertips(env)
        hand = float(np.min(np.linalg.norm(tips - c, axis=1))) - R_BALL
        rolled = float(np.linalg.norm((c - BALL_START) - ((c - BALL_START) @ NORMAL) * NORMAL))
        if self.foul is None and hand < HAND_GAP:
            self.foul = "hand"
        if self.foul is None and rolled > FREE_ROLL and _putter_gap(env, c) < R_BALL + 0.004:
            self.foul = "second stroke"
        height, radial = _on_plane(c)
        self.holed = bool(radial < R_HOLE and height < -0.002)  # the whole ball below the turf's surface, inside the hole
        still = float(np.linalg.norm(v)) < 0.005
        ok = self.foul is None and self.holed and still and hand > CLEAR
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        on_turf = abs(height - R_BALL) < 0.004 and _on_green_xy(c)
        if self.foul is not None:
            self.score = 0.0
        elif self.full and self.holed:
            self.score = 1.0
        elif still and on_turf and radial <= 0.30:
            self.score = 0.5
        elif still and on_turf and radial <= 0.60:
            self.score = 0.25
        else:
            self.score = 0.0
        if os.environ.get("PUTT_DEBUG"):
            ps = m.state(env, "putter")
            with open(os.environ["PUTT_DEBUG"], "a") as f:
                f.write(f"{c[0]:.4f} {c[1]:.4f} {c[2]:.4f} {v[0]:.4f} {v[1]:.4f} {v[2]:.4f} | h {height:.4f} r {radial:.4f} | put {ps.pos[0]:.4f} {ps.pos[1]:.4f} {ps.pos[2]:.4f} "
                        f"v {ps.lin_vel[0]:.3f} {ps.lin_vel[1]:.3f} {ps.lin_vel[2]:.3f} | gap {_putter_gap(env, c):.4f} hand {hand:.3f} | foul {self.foul} score {self.score}\n")


TRIAL = Trial()
m.on_step(_roll)
m.on_step(TRIAL.step)
m.on_reset(_place_ball)
m.on_reset(TRIAL.reset)

SUCCESS_HOLD_S = 3.0


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full and TRIAL.holed and TRIAL.foul is None)


# -- the scripted demo: open loop, from the published starting layout ---------------------------------------------------


class Grip(float):
    """A waypoint 'yaw' that carries a whole TCP rotation (the harness passes it to grip_rotation without looking)."""

    rot = np.eye(3)


def _as_grip(R):
    g = Grip(0.0)
    g.rot = np.asarray(R, float)
    return g


if not getattr(_demo.grip_rotation, "_takes_grips", False):
    _plain = _demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return _plain(yaw) if rot is None else rot

    _grip_rotation._takes_grips = True
    _demo.grip_rotation = _grip_rotation


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


AIM_DEG = float(os.environ.get("PUTT_AIM", "8.0"))  # the start line, left of the hole (uphill), degrees
STROKE_T = float(os.environ.get("PUTT_T", "0.45"))  # the forward stroke's time, s
BACK = float(os.environ.get("PUTT_BACK", "0.15"))  # backswing, m
LIFT = float(os.environ.get("PUTT_LIFT", "0.06"))
TURN_UP = float(os.environ.get("PUTT_TURN_UP", "0.16"))


def demo():
    aim = math.radians(AIM_DEG)
    u = np.array([math.cos(aim), math.sin(aim), 0.0])
    R_put = _rz(aim)  # the putter upright at address, its face square to the start line
    R_hand = R_put @ _STROKE_GRIP0
    hand_off = R_put @ GRASP  # the grasp point from the sole's centre
    # address: face 1 cm behind the ball, the sole 4 mm above the turf under the heel
    face_gap = 0.010
    sole_c = BALL_START - (R_BALL + HEAD[0] / 2 + face_gap) * u
    heel = sole_c + R_put @ np.array([0.0, HEAD[1] / 2, 0.0])
    sole_c[2] = turf_z(heel[0], heel[1]) + 0.004
    addr = sole_c + hand_off
    back = addr - BACK * u
    # the forward stroke is symmetric about the moment the face meets the ball, where the hand is fastest
    contact = addr + face_gap * u
    through = contact + (contact - back)
    pick = np.array([PICK_XY[0], PICK_XY[1], turf_z(*PICK_XY) + 0.0155])
    gp, gh = _as_grip(_PICK_GRIP), _as_grip(R_hand)
    open_, shut = FINGER_OPEN, FINGER_CLOSED
    up = np.array([0.0, 0.0, 1.0])
    left = [
        Waypoint(2.0, tuple(pick + 0.05 * up), open_, "over the putter", gp),
        Waypoint(1.2, tuple(pick), open_, "down to the shaft", gp),
        Waypoint(0.8, tuple(pick), shut, "close on the shaft", gp),
        Waypoint(1.2, tuple(pick + LIFT * up), shut, "lift", gp),
        Waypoint(2.0, tuple(0.5 * (pick + addr) + TURN_UP * up), shut, "tip the head down", _as_grip(_ik.slerp_matrix(_PICK_GRIP, R_hand, 0.45))),
        Waypoint(2.0, tuple(addr - 0.04 * u + 0.16 * up), shut, "putter upright", gh),
        Waypoint(1.5, tuple(addr + 0.012 * up), shut, "address", gh),
        Waypoint(0.8, tuple(addr), shut, "sole it", gh),
        Waypoint(1.4, tuple(back), shut, "backswing", gh),
        Waypoint(0.3, tuple(back), shut, "top", gh),
        Waypoint(STROKE_T, tuple(through), shut, "stroke", gh),
        Waypoint(1.0, tuple(through), shut, "hold the finish", gh),
        Waypoint(1.5, tuple(through + 0.20 * up - 0.10 * u), shut, "lift the putter clear", gh),
    ]
    return {"left": left}
