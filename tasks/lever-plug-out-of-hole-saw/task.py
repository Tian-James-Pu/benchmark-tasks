"""Lever a jammed wood plug out of a 2-1/8 in bi-metal hole saw through the slots in its side.

On a bench: a 2-1/8 in (54 mm) bi-metal hole saw with slotted sides for plug removal (modelled on the Milwaukee Hole Dozer
49-56-9668: 41 mm depth of cut) on its arbor, its hex shank stood in a 14 mm hole in the end of a 4x4 offcut screwed to the
bench. It has just bored through a 17.5 mm (11/16 in) pine board, and the
17.5 mm thick pine plug is jammed in the cup, its front face flush with the teeth, its back 23.5 mm clear of the back plate.
A flat screwdriver lies to the right. Each side of the cup has a stepped slot: a low step (ledge 16 mm above the back plate's
underside) and a high step (ledge 28 mm), both open up to 36 mm, the teeth at 44 mm.

Model (declared in code):
- The saw and arbor are one free articulation ("saw", 0.28 kg) standing in the post's hole; it turns in the hole, and can be lifted
  out or knocked over. The plug rides on it on a slider along the saw's axis ("slide", 0 to 30 mm) and a small hinge ("plug",
  +-4 degrees about the axis across the two slots) that shows how cocked it is. The cup's wall, back plate, pilot bit and the
  slots' ledges are colliders. The plug has no collider: its joint guides it.
- The blade's push on the plug is a contact written in code each control step: wherever the screwdriver's blade or shaft is under
  the plug's back face (inside its 51 mm circle), the overlap d gives a force min(K_PUSH d, the jam's hold) along the saw's axis,
  applied down on the screwdriver at that point and, equal and opposite, up on the saw (the plug's grip on the cup). When K_PUSH d
  passes the hold, the plug slides out until the force is back at the hold: a friction law, holding up to
  60 N x (length still in the cup / 17.5 mm) + 1.5 N (the pilot bit).
- Prying from one side only cams the plug crosswise and wedges it ("walking the wedge from side to side"). The code keeps the
  plug's cock, in millimetres of rise of one edge over the other: while the screwdriver's tip is in the cup, plug travel adds to the
  cock on the side its handle comes from. Once the cock reaches 6 mm, travel pushed from that side is locked (wedged) while the
  plug is in the bore; travel pushed from the other side moves it again. The hinge is set to the cock.
  The 60 N, the 6 mm and the slot steps' sizes are estimates.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
import simready  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz

# -- geometry (metres) ----------------------------------------------------------------------------------------------------------------
SX, SY = 0.24, 0.08  # the saw's axis on the bench
SAW_YAW = 90.0  # the saw starts with its slot A facing -y (the right arm)
POST = (0.089, 0.089, 0.100)  # a 4x4 offcut standing on end
HOLE, HOLE_D = 0.014, 0.032  # the hole in its top end (square in the model; a 1/2 in drilled hole is round)
COLLAR_H = 0.015  # the arbor's body under the saw's back plate, resting on the post
Z0 = TOP + POST[2] + COLLAR_H  # the saw frame's origin: the underside of the back plate, on the axis
R_OUT, R_BORE = 0.0270, 0.0257  # 2-1/8 in cup: outside, and bore
PLATE = 0.003  # back plate thickness
TEETH = 0.044  # tooth tips above the frame origin: 41 mm depth of cut over the 3 mm plate
PILOT_R, PILOT_TIP = 0.0032, 0.050  # 1/4 in pilot bit, 6 mm proud of the teeth
LEDGE_LO, LEDGE_HI, SLOT_TOP = 0.016, 0.028, 0.036
SEG = 6  # degrees per wall segment
STEP_IN, STEP_OUT = 3.0, 52.0  # each slot step spans these angles either side of the slot's centre line (about 23 mm wide at the rim)
PLUG_R, PLUG_T = 0.0255, 0.0175  # 11/16 in pine board
PLUG_Z0 = TEETH - PLUG_T  # the plug's back face at the start (26.5 mm above the frame origin)
TRAVEL = 0.030
CLEAR_TEETH, OFF_BIT = TEETH - PLUG_Z0, PILOT_TIP - PLUG_Z0  # 17.5 mm and 23.5 mm of travel

WEDGED_EFFORT = 2000.0  # N: the slider's drive effort (it only holds the plug where the code puts it)
# -- the jam -------------------------------------------------------------------------------------------------------------------------
F_JAM, F_PILOT, F_FREE = 60.0, 1.5, 0.3  # N
BIND = 0.006  # m of cock that wedges the plug
K_HOLD = 2.0e5  # N/m: the slider's drive, which keeps the plug where the jam leaves it
K_PUSH = 3.0e4  # N/m: the code contact's stiffness (2 mm of overlap at the full 60 N)
V_MAX, STEP_S = 0.03, 0.05  # m/s: the fastest the plug slides out (a slip of the blade can't fling it), per 20 Hz step
F_MAX = 150.0  # N: the most the code contact pushes back (a wedged plug)

LOOKS = {
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.85, roughness=0.35),
    "cup": m.Look(color=m.rgb(0.30, 0.31, 0.33), metallic=0.7, roughness=0.45),
    "teeth": m.Look(color=m.rgb(0.72, 0.72, 0.74), metallic=0.9, roughness=0.3),
    "red": m.Look(color=m.rgb(0.70, 0.06, 0.06), roughness=0.5),
    "bit": m.Look(color=m.rgb(0.10, 0.10, 0.11), metallic=0.6, roughness=0.4),
    "pine": m.Look(color=m.rgb(0.86, 0.72, 0.50), roughness=0.8),
    "endgrain": m.Look(color=m.rgb(0.74, 0.58, 0.38), roughness=0.85),
    "post": m.Look(color=m.rgb(0.80, 0.66, 0.46), roughness=0.85),
    "dark": m.Look(color=m.rgb(0.12, 0.10, 0.08), roughness=0.9),
}


def _slot_of(k: int):
    """The slot step a wall segment k (SEG degrees from k*SEG) lies in: its ledge height, or None for a solid segment."""
    a = (k * SEG + SEG / 2) % 360
    for c in (180.0, 0.0):
        d = (a - c + 180) % 360 - 180
        if -STEP_OUT < d < -STEP_IN:
            return LEDGE_LO
        if STEP_IN < d < STEP_OUT:
            return LEDGE_HI
    return None


def _wall_piece(k, z0, z1):
    a = k * SEG + SEG / 2
    chord = 2 * R_OUT * math.sin(math.radians(SEG / 2)) + 0.0006
    rm = (R_OUT + R_BORE) / 2
    return m.box((R_OUT - R_BORE, chord, z1 - z0), (0.0, 0.0, 0.0)).turned(a, "z").moved(
        (rm * math.cos(math.radians(a)), rm * math.sin(math.radians(a)), (z0 + z1) / 2))


def _saw():
    parts, cols = [], []
    hexo = [(0.0064 * math.cos(math.radians(60 * i)), 0.0064 * math.sin(math.radians(60 * i))) for i in range(6)]
    shank = m.prism(hexo, -COLLAR_H - 0.030, -COLLAR_H)
    parts.append(("shank", shank, "steel"))
    cols.append(m.col_hull("shank_c", shank))
    collar = m.cylinder(0.0125, COLLAR_H, (0, 0, -COLLAR_H / 2), segments=6)
    parts.append(("collar", collar, "steel"))
    cols.append(m.col_hull("collar_c", m.cylinder(0.0125, COLLAR_H, (0, 0, -COLLAR_H / 2), segments=12)))
    parts.append(("plate", m.cylinder(R_OUT, PLATE, (0, 0, PLATE / 2), segments=48), "cup"))
    cols.append(m.col_hull("plate_c", m.cylinder(R_OUT, PLATE, (0, 0, PLATE / 2), segments=24)))
    pilot = m.cylinder(PILOT_R, PILOT_TIP - PLATE, (0, 0, (PILOT_TIP + PLATE) / 2), segments=12)
    parts.append(("pilot", pilot, "bit"))
    cols.append(m.col_hull("pilot_c", pilot))
    for k in range(360 // SEG):
        ledge = _slot_of(k)
        spans = [(PLATE, TEETH - 0.004)] if ledge is None else [(PLATE, ledge), (SLOT_TOP, TEETH - 0.004)]
        for i, (z0, z1) in enumerate(spans):
            piece = _wall_piece(k, z0, z1)
            parts.append((f"wall{k}_{i}", piece, "red" if (ledge is None and 0.012 < (z0 + z1) / 2 and k % 15 == 7) else "cup"))
            cols.append(m.col_hull(f"wall{k}_{i}_c", piece))
        teeth = _wall_piece(k, TEETH - 0.004, TEETH)
        parts.append((f"teeth{k}", teeth, "teeth"))
        cols.append(m.col_hull(f"teeth{k}_c", teeth))
    return parts, cols


def _plug_link():
    zc = PLUG_Z0 + PLUG_T / 2
    body = m.cylinder(PLUG_R, PLUG_T, (0, 0, zc), segments=40)
    parts = [("plug", body, "pine"),
             ("face", m.cylinder(PLUG_R - 0.0003, 0.0006, (0, 0, TEETH + 0.0001), segments=40), "endgrain"),
             ("pilot_hole", m.cylinder(PILOT_R, 0.0008, (0, 0, TEETH + 0.0003), segments=12), "dark")]
    cols = []  # the blade's contact is in code (Trial.step); fingers pass through the plug
    slide = m.slider("slide", "Z", 0.0, TRAVEL, (), (), at=(0.0, 0.0, zc), mass=0.005, damping=40.0, friction=0.0, effort=WEDGED_EFFORT,
                     armature=0.001, inertia=(1e-6, 1e-6, 1e-6))
    tilt = m.hinge("plug", "Y", (0.0, 0.0, zc), -4.0, 4.0, parts, cols, parent="slide", mass=0.018, spring=20.0, damping=0.5,
                   friction=0.0, effort=50.0, inertia=(4.6e-6, 4.6e-6, 7.2e-6))
    return [slide, tilt]


_sp, _sc = _saw()
SAW = m.mechanism("saw", (SX, SY, Z0), _plug_link(), LOOKS, frame_parts=_sp, frame_colliders=_sc, free_mass=0.28, yaw_deg=SAW_YAW,
                  frame_inertia=(1.6e-4, 1.6e-4, 1.1e-4), friction=(0.6, 0.5), solver_iters=48)


def _post():
    w, d, h = POST
    hz = h - HOLE_D
    side = (w / 2 - HOLE / 2)
    boxes = [("base", (w, d, hz), (0, 0, hz / 2)),
             ("px", (side, d, HOLE_D), (HOLE / 2 + side / 2, 0, hz + HOLE_D / 2)),
             ("nx", (side, d, HOLE_D), (-HOLE / 2 - side / 2, 0, hz + HOLE_D / 2)),
             ("py", (HOLE, side, HOLE_D), (0, HOLE / 2 + side / 2, hz + HOLE_D / 2)),
             ("ny", (HOLE, side, HOLE_D), (0, -HOLE / 2 - side / 2, hz + HOLE_D / 2))]
    parts = [(n, m.box(s, c), "post") for n, s, c in boxes]
    parts.append(("hole", m.box((HOLE, HOLE, 0.001), (0, 0, hz + 0.0005)), "dark"))
    cols = [m.col_box(n + "_c", s, c) for n, s, c in boxes]
    return parts, cols


_pp, _pc = _post()
STAND = m.fixed("stand", (SX, SY, TOP), _pp, _pc, LOOKS, friction=(0.7, 0.6))  # screwed down to the bench

# The screwdriver's collision shape, in its own frame (x along the shaft from the handle to the tip, the shaft's axis 18.4 mm up):
# boxes fitted to its mesh. The handle's grip is a 3.4 cm square bar (the mesh is round, 3.8 cm at its widest), the shaft 5 mm
# square, the blade 7.8 mm wide and 3.5 mm thick (the real one tapers from 4.4 mm to 1.4 mm at the tip).
SD_AXIS_Z = 0.0184
SD_BOXES = [("grip", (-0.050, 0.014), 0.034, 0.034), ("neck", (0.014, 0.056), 0.024, 0.024),
            ("shaft", (0.056, 0.141), 0.005, 0.005), ("blade", (0.141, 0.1675), 0.0078, 0.0035)]


def _box_colliders(prim) -> None:
    """The screwdriver collides as the boxes above instead of its mesh (a signed distance field), so the blade behaves the same on
    CPU and GPU physics and the square grip doesn't roll between the finger pads. Used through simready's convex option, which only
    the screwdriver sets. The mesh stays as it is for the looks."""
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

    while instances := [p for p in Usd.PrimRange(prim) if p.IsInstance()]:
        for p in instances:
            p.SetInstanceable(False)
    body = None
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and body is None:
            body = p
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    stage = prim.GetStage()
    cache = UsdGeom.XformCache()
    to_body = cache.GetLocalToWorldTransform(body).GetInverse() * Gf.Matrix4d(1.0)
    root = cache.GetLocalToWorldTransform(prim)
    for name, (x0, x1), w, t in SD_BOXES:
        c = UsdGeom.Cube.Define(stage, body.GetPath().AppendChild(f"col_{name}"))
        c.CreateSizeAttr(1.0)
        c.CreatePurposeAttr(UsdGeom.Tokens.guide)
        centre = root.Transform(Gf.Vec3d((x0 + x1) / 2, 0.0, SD_AXIS_Z))  # the prop's frame is its root's
        local = to_body.Transform(centre)
        xf = UsdGeom.Xformable(c)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(local)
        xf.AddScaleOp().Set(Gf.Vec3f(x1 - x0, w, t))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(c.GetPrim())
        contact.CreateContactOffsetAttr(0.002)
        contact.CreateRestOffsetAttr(0.0)


simready._convex_colliders = _box_colliders

SD_POS = (0.14, -0.12, TOP)  # the screwdriver's origin (the underside of its handle's middle), lying along +y, blade towards the saw
SCREWDRIVER = SimReady("screwdriver", pos=SD_POS, size=(0.221, 0.038, 0.036), mass=0.12, package="handTool_screwdriver_slotted_65mm_a01",
                       usd="sm_handTool_screwdriver_slotted_65mm_a01_01.usd", yaw=math.pi / 2, convex=True)
SQUARE = SimReady("speed_square", pos=(0.42, 0.24, TOP), size=(0.290, 0.218, 0.008), mass=0.15, package="measurement_ruler_speedsquare_a02",
                  usd="sm_measurement_ruler_speedsquare_a02_01.usd", yaw=math.pi / 2)
KNIFE = SimReady("utility_knife", pos=(0.45, -0.17, TOP), size=(0.160, 0.061, 0.026), mass=0.12, package="handTool_knife_retractableBlade_a01",
                 usd="sm_handTool_knife_retractableBlade_a01_01.usd", yaw=0.4)
GLASSES = SimReady("safety_glasses", pos=(0.45, -0.04, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                   usd="sm_accessory_eyeglasses_a01_01.usd", yaw=-0.3)
OBJECTS = [STAND, SAW, SCREWDRIVER, SQUARE, KNIFE, GLASSES]

SD_TIP = np.array([0.1675, 0.0, 0.0184])  # the blade's tip, and a point on the handle, in the screwdriver's frame
SD_HANDLE = np.array([0.0, 0.0, 0.0184])
SD_BLADE = np.array([[x, y, 0.0184] for x in (0.1675, 0.1575, 0.1475, 0.1375, 0.1275) for y in (-0.004, 0.0, 0.004)])  # the blade's last 4 cm
# the points that can push the plug: the top face of the blade's box and of the shaft's last 6 cm, in the screwdriver's frame
SD_PUSH = np.array([[x, y, SD_AXIS_Z + 0.00175] for x in np.linspace(0.141, 0.1675, 7) for y in (-0.0039, 0.0, 0.0039)]
                   + [[x, y, SD_AXIS_Z + 0.0025] for x in np.linspace(0.081, 0.141, 8, endpoint=False) for y in (-0.0025, 0.0025)])


def jam_force(q: float) -> float:
    """The slider's holding force (N) at travel q (m)."""
    if q < CLEAR_TEETH:
        return F_JAM * (CLEAR_TEETH - q) / CLEAR_TEETH + F_PILOT
    return F_PILOT if q < OFF_BIT else F_FREE


def _pose(env, name):
    a = env.unwrapped.scene[name]
    p = m._np(a.data.root_link_pose_w)[0]
    return p[:3] - m._np(env.unwrapped.scene.env_origins[0]), m.quat_matrix(p[3:7])


class Trial:
    """The jam, the cock and the score. Cleared only by a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the recorded state is cleared
        self.cock = 0.0
        self.q_prev = 0.0
        self.target = 0.0
        self.pushing = False
        self.force = 0.0
        self.side = 0
        self.bound = False
        self.lift = 0.0
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.shown = 0.0
        if env is not None:
            m.lock(env, "saw", "slide", stiffness=K_HOLD, effort=WEDGED_EFFORT)  # holds the plug where the code puts it
            m.set_target(env, "saw", "plug", 0.0)

    def _side(self, env, s) -> int:
        """+1 when any part of the screwdriver's blade is in the cup or its slots (inside the cup's outside radius, between the back
        plate and just past the teeth) with its handle on the frame's -x side (slot A), -1 on +x (slot B), else 0."""
        p, R = _pose(env, "screwdriver")
        blade = s.to_local(p + SD_BLADE @ R.T)
        handle = s.to_local((p + R @ SD_HANDLE)[None, :])[0]
        inside = (np.hypot(blade[:, 0], blade[:, 1]) < R_OUT + 0.002) & (blade[:, 2] > PLATE) & (blade[:, 2] < TEETH + 0.01)
        if bool(np.any(inside)):
            return 1 if handle[0] < 0 else -1
        return 0

    def step(self, env):
        s = m.state(env, "saw")
        q = s.joint("slide")
        self.side = self._side(env, s)
        dq = q - self.q_prev  # new travel past the furthest out the plug has been (so the joint's jitter adds nothing)
        if dq > 0:
            if self.side:
                self.cock = float(np.clip(self.cock + self.side * dq, -BIND, BIND))
            self.q_prev = q
        # wedged: tipped by BIND and pushed again from the same side, while the plug is still in the bore
        self.bound = q < CLEAR_TEETH and ((self.side == 1 and self.cock >= BIND - 1e-4) or (self.side == -1 and self.cock <= -BIND + 1e-4))
        cap = F_MAX if self.bound else jam_force(q)
        # the code contact: the deepest overlap of the blade's top (or the shaft's) with the plug's back face
        p, R = _pose(env, "screwdriver")
        pts = s.to_local(p + SD_PUSH @ R.T)
        back = PLUG_Z0 + self.target
        d = pts[:, 2] - back
        under = (np.hypot(pts[:, 0], pts[:, 1]) < PLUG_R) & (d > 0.0) & (d < 0.008)
        force = 0.0
        if bool(np.any(under)):
            k = int(np.argmax(np.where(under, d, -1.0)))
            dd = float(d[k])
            if K_PUSH * dd > cap:  # pushed harder than the jam holds: the plug slides out until the force is back at the hold
                self.target = min(TRAVEL, self.target + min(dd - cap / K_PUSH, V_MAX * STEP_S))
                dd = cap / K_PUSH
            force = min(F_MAX, K_PUSH * dd)
            at = s.to_world(pts[k][None, :])[0]
            axis = s.rotation()[:, 2]
            m.push(env, "screwdriver", -force * axis, at=at)
            m.push(env, "saw", force * axis, at=at)
            self.pushing = True
        elif self.pushing:
            m.push(env, "screwdriver", (0.0, 0.0, 0.0))
            m.push(env, "saw", (0.0, 0.0, 0.0))
            self.pushing = False
        self.force = force
        m.set_target(env, "saw", "slide", self.target)
        tilt = 0.5 * math.atan2(self.cock, 2 * PLUG_R)  # shown at half its size, so the tipped plug's low edge stays clear of the slots
        if abs(tilt - self.shown) > 1e-3:
            m.set_target(env, "saw", "plug", tilt)
            self.shown = tilt
        self.lift = q
        # hands off: no fingertip within 2 cm of the plug (a cylinder about the saw's axis)
        tips = s.to_local(m.fingertips(env))
        z0, z1 = PLUG_Z0 + q, PLUG_Z0 + q + PLUG_T
        near = (np.hypot(tips[:, 0], tips[:, 1]) < PLUG_R + 0.02) & (tips[:, 2] > z0 - 0.02) & (tips[:, 2] < z1 + 0.02)
        hands_off = not bool(np.any(near))
        ok = q >= OFF_BIT - 1e-4 and hands_off
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.full:
            self.score = 1.0
        elif q >= CLEAR_TEETH - 1e-4:
            self.score = 0.75
        elif q >= 0.011:
            self.score = 0.5
        elif q >= 0.006:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- the scripted demo ------------------------------------------------------------------------------------------------------------
def demo_script(ctx):
    """Closed loop, for demokit. Reads the screwdriver's and the saw's poses and the plug's height (all visible), never the scorer."""
    from demokit import down
    from yam_arena import ik

    l, r = ctx.left, ctx.right
    OPEN = -0.04695

    def sd():
        p, R = _pose(ctx.emb._env, "screwdriver")  # the prop's origin (ctx.pos gives its bounding box's centre)
        return p + R @ SD_TIP, R

    def tip():
        return sd()[0]

    def plug_q():
        return ctx.joint("saw", "slide")

    def centre():
        return ctx.pos("saw")

    mode = ctx.mode  # red-team modes: none (no motion), onearm (right hand only, never turns the saw), noturn (left holds, never turns)
    if mode == "none":
        yield from ctx.until(lambda: False, timeout=100.0)
        return
    # right: pick up the screwdriver by its handle, fingers across it
    g = np.array(SD_POS) + np.array([0.0, -0.009, 0.020])  # the handle's straight middle (the prop lies along +y)
    yield from r.move(g + [0, 0, 0.08], grip=down(90), finger=OPEN, t=1.5)
    yield from r.move(g, t=1.2)
    yield from r.close()
    yield from r.move(g + [0, 0, 0.16], t=1.2)
    # square the screwdriver up in the hand's yaw so it points along +y
    for _ in range(2):
        u = sd()[1][:, 0]
        err = math.degrees(math.atan2(u[1], u[0])) - 90.0
        if abs(err) > 1.0:
            yield from r.turn(-err, t=0.6)
    # left: hold the saw by the band between the slots' tops and the teeth
    Y0, Y1, YM = -120.0, 60.0, -30.0
    band = 0.040

    def cupgrip():
        c = centre()
        return np.array([c[0], c[1], Z0 + band])

    if mode != "onearm":
        yield from l.move((0.20, 0.15, TOP + 0.20), grip=down(YM), finger=OPEN, t=1.5)
        yield from l.move(cupgrip() + [0, 0, 0.035], t=1.5)
        yield from l.move(cupgrip() + [0, 0, 0.035], grip=down(Y0), t=1.0)
        yield from l.move(cupgrip(), t=1.2)
        yield from l.close()

    def pry(q_goal, label):
        # the higher step once the plug's back face is well above it, else the lower one (the steps are on opposite sides of the slot)
        ledge, off = (LEDGE_HI, OFF) if PLUG_Z0 + plug_q() > LEDGE_HI + 0.007 else (LEDGE_LO, -OFF)
        def aim():
            # in the saw's own frame (it sits a few degrees off square in the post's hole, and is turned by hand): the slot that
            # faces the screwdriver, and across it the side its step is on
            st = ctx.state("saw")
            Rs = st.rotation()
            sdir = np.array([1.0, 0.0, 0.0]) if (Rs @ [1.0, 0.0, 0.0])[1] < 0 else np.array([-1.0, 0.0, 0.0])
            lat = np.array([-sdir[1], sdir[0], 0.0])
            zl = np.array([0.0, 0.0, 1.0])
            world = lambda v: np.asarray(st.to_world(np.asarray(v)[None, :]))[0]
            z = ledge + 0.0055
            outside = world(sdir * (R_OUT + 0.030) + lat * off + zl * z)
            inside = world(sdir * -0.010 + lat * off + zl * z)
            F = world(sdir * (R_OUT - 0.0007) + lat * off + zl * ledge)  # the fulcrum: the ledge's edge under the shaft
            bdir = inside - outside
            axis = Rs @ lat
            axis = axis / np.linalg.norm(axis)
            if axis[0] < 0:
                axis = -axis
            return outside, inside, F, math.degrees(math.atan2(bdir[1], bdir[0])), axis

        outside, inside, F, want_yaw, axis = aim()

        def square_up():
            # level the blade and point it along +y: the round handle pivots between the pads, so turn the hand until it is
            for _ in range(4):
                u = sd()[1][:, 0]
                pitch = math.degrees(math.asin(float(np.clip(u[2], -1, 1))))
                yaw = (math.degrees(math.atan2(u[1], u[0])) - want_yaw + 180.0) % 360.0 - 180.0
                if os.environ.get("PLUG_DEBUG"):
                    print(f"[demo] {label}: square up: pitch {pitch:.1f} yaw {yaw:.1f} grip {ctx.observed('right')[6]:.3f}", flush=True)
                if abs(pitch) < 1.5 and abs(yaw) < 1.5:
                    break
                if abs(pitch) >= 1.5:
                    yield from r.turn(-pitch, axis=tuple(axis), t=0.5, label=label + ": level the blade")
                if abs(yaw) >= 1.5:
                    yield from r.turn(-yaw, t=0.4, label=label + ": square the blade")

        yield from square_up()
        yield from r.servo(outside + [0, -0.03, 0.05], read=tip, tol=0.004, timeout=3.0, label=label + ": over")
        yield from r.servo(outside + [0, 0, 0.01], read=tip, tol=0.002, timeout=3.0, label=label + ": line up")
        outside, inside, F, want_yaw, axis = aim()  # again, now the saw has settled in the left hand
        yield from square_up()
        yield from r.servo(outside, read=tip, tol=0.0012, timeout=2.5, label=label + ": line up")
        hand = lambda p: ik.tcp_pose(r.q, r.base)[0] + (np.asarray(p) - tip())  # the hand's command that brings the tip to p
        print(f"[demo] {label}: tip {np.round(tip() * 1000, 1)} want {np.round(outside * 1000, 1)} yaw {math.degrees(math.atan2(sd()[1][1, 0], sd()[1][0, 0])):.1f}", flush=True)
        yield from r.move(hand(inside), t=1.8, label=label + ": blade in")
        yield from r.wait(0.3)
        print(f"[demo] {label}: in: tip {np.round(tip() * 1000, 1)} want {np.round(inside * 1000, 1)} yaw {math.degrees(math.atan2(sd()[1][1, 0], sd()[1][0, 0])):.1f}", flush=True)
        yield from r.move(hand(inside - [0, 0, 0.003]), t=0.6, label=label + ": onto the ledge")
        done = 0.0
        while plug_q() < q_goal and done < 60.0:
            yield from r.arc(F, tuple(axis), 1.0, t=0.1, label=label + ": lever")
            done += 1.0
            if os.environ.get("PLUG_DEBUG") and int(done) % 5 == 0:
                u = sd()[1][:, 0]
                print(f"[demo] {label} {done:.0f} deg: q {plug_q()*1000:.1f} tip {np.round((tip()-F)*1000,1)} pitch {math.degrees(math.asin(u[2])):.1f} "
                      f"hand {np.round((r.tcp()-F)*1000,1)} cmd {np.round((ik.tcp_pose(r.q, r.base)[0]-F)*1000,1)}", flush=True)
        def note(what):
            if os.environ.get("PLUG_DEBUG"):
                u = sd()[1][:, 0]
                print(f"[demo] {label} {what}: q {plug_q() * 1000:.1f} tip {np.round((tip() - F) * 1000, 1)} pitch "
                      f"{math.degrees(math.asin(float(np.clip(u[2], -1, 1)))):.1f} grip {ctx.observed('right')[6]:.3f}", flush=True)

        yield from r.wait(0.4)
        note("levered")
        yield from r.arc(F, tuple(axis), -done, t=1.0, label=label + ": back level")
        note("back level")
        t1 = tip()
        lift = np.array([t1[0], t1[1], outside[2] + 0.001])  # the blade at its entry height, clear of the ledge
        yield from r.move(hand(lift), t=0.5, label=label + ": up off the ledge")
        note("up")
        yield from r.move(hand(outside + [0, 0, 0.001]), t=2.0, label=label + ": blade out")  # straight back out along the slot
        note("out")
        yield from r.servo(outside + [0, -0.03, 0.05], read=tip, tol=0.005, timeout=2.0, label=label + ": away")
        note("away")

    def turn_saw(first):
        if not first:
            yield from l.open()
            yield from l.move(cupgrip() + [0, 0, 0.03], grip=down(Y1), t=0.8)
            yield from l.move(cupgrip() + [0, 0, 0.03], grip=down(YM), t=0.7)
            yield from l.move(cupgrip() + [0, 0, 0.03], grip=down(Y0), t=0.7)
            yield from l.move(cupgrip(), t=1.0)
            yield from l.close()
        yield from l.move(cupgrip(), grip=down(YM), t=1.5)
        yield from l.move(cupgrip(), grip=down(Y1), t=1.5)
        # check the slot facing the screwdriver: the saw's yaw should be 90 or -90
        yaw = math.degrees(math.atan2(ctx.state("saw").rotation()[1, 0], ctx.state("saw").rotation()[0, 0]))
        print(f"[demo] saw yaw {yaw:.1f}", flush=True)

    OFF = 0.011  # the middle of a slot step, across the saw
    if mode in ("onearm", "noturn"):  # the careless way: pry again and again through the slot that faces the screwdriver
        if mode == "noturn":
            yield from turn_saw(True)  # as in the full demo, so the first pry is the same; then never again
        for n in range(4):
            yield from pry(0.003 + 0.008 * n, f"pry {n + 1}")
        print(f"[demo] plug out {plug_q() * 1000:.1f} mm after the pries", flush=True)
        yield from r.move(g + [0, 0, 0.06], t=1.8)
        yield from r.move(g + [0, 0, 0.004], t=1.0)
        yield from r.open()
        yield from ctx.together(r.move(g + [0, 0, 0.10], t=1.0), l.open())
        yield from l.move((0.30, 0.22, TOP + 0.15), grip=down(0.0), t=1.5)
        yield from ctx.until(lambda: False, timeout=3.0)
        return
    yield from turn_saw(True)
    yield from pry(0.003, "pry 1")
    yield from turn_saw(False)
    yield from pry(0.011, "pry 2")
    yield from turn_saw(False)
    yield from pry(0.019, "pry 3")
    yield from turn_saw(False)
    yield from pry(0.026, "pry 4")
    print(f"[demo] plug out {plug_q() * 1000:.1f} mm after the pries", flush=True)
    # right: put the screwdriver down; left: let go of the saw
    yield from ctx.together(r.move(g + [0, 0, 0.06], t=1.8), l.open())
    yield from l.wait(0.5)
    # the left hand slides off sideways, across its fingers' opening (the post's hole keeps the saw from following), then up
    away = 0.06 * np.array([math.cos(math.radians(Y1)), math.sin(math.radians(Y1)), 0.0])  # the fingers close along Y1 + 90 degrees
    yield from ctx.together(r.move(g + [0, 0, 0.004], t=1.0), l.move(cupgrip() + away, t=1.2))
    yield from r.open()
    yield from ctx.together(r.move(g + [0, 0, 0.10], t=1.0), l.move(cupgrip() + away + [0, 0, 0.06], t=1.2))
    st = ctx.state("saw")
    print(f"[demo] saw at the end: {np.round(st.pos * 1000, 1)} tilt {st.tilt_deg():.1f}", flush=True)
    yield from l.move((0.30, 0.22, TOP + 0.15), grip=down(0.0), t=1.5)
    yield from ctx.until(lambda: False, timeout=3.0)

