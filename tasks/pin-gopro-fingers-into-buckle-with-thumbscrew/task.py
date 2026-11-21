"""On the station table: a helmet (a yellow hard hat, brim towards +x) with a GoPro curved adhesive mount stuck on its crown and a
mounting buckle clipped into it. The buckle's three fingers (prongs) stand up from it, 3.0 mm thick with 3.3 mm gaps, each with a
5.2 mm hole on one axis across the helmet (along y), 20 mm above the crown. An M5 x 0.8 thumb screw (20 mm of thread under a black
plastic neck and paddle head, 54 mm long in all) has been backed out so that its tip sits just inside the right-hand outer finger's
hole; its paddle head sticks out 47 to 59 mm to the right of the buckle. The left outer finger carries the captive M5 nut in a hex
boss. A GoPro HERO (56.6 x 47.7 x 29.4 mm, 86 g) lies on its right side on the table, left of the helmet, lens towards +x and its two
folding mounting fingers (3.0 mm thick, 3.3 mm apart, 5.2 mm holes) folded out, pointing to the left. Sunglasses, a bottle of drink
and a pair of cloth gloves lie on the table.

Mechanism (declared in code):
- Buckle ("mount", an articulation welded to the helmet): the frame is the adhesive base, the buckle body, the middle finger and the
  nut's boss. The two outer fingers are stiff sliders across the axis (100 kN/m, 0.4 mm of travel inward), held at rest; once the
  screw is seated they are driven 0.4 mm inward, which takes up the 0.15 mm clearances and clamps the camera's fingers (25 N each), as
  the screw's tension bends a real buckle's outer fingers in. The screw is a slider along the axis with a hinge on it (its turn).
- The camera's fingers collide with the buckle's fingers and with the screw's shank through real 5.2 mm holes (16 convex pieces
  round each hole). The 4.9 mm shank has a 0.45 mm tip chamfer. So the screw passes a camera finger only when that finger's hole is
  near the axis (in the demo runs it went through at up to about 1 mm off, the chamfer and the holding hand's give leading it in;
  at 2 mm and more its tip stopped on the finger's face: a jam). Nothing about the jam is scripted.
- The thread (code, each control step): the screw slides freely until its tip reaches the nut (13.95 mm of travel). It cannot be
  pushed further: from there its axial position is driven by its turn, 0.8 mm per turn (stiff: 50 kN/m). After 4 turns the head's
  shoulder is against the outer finger: the screw is seated, its turn is held and the outer fingers clamp.
- The scorer reads the end state: the head's standoff from the outer finger (the screw's travel), where the camera's finger holes are,
  the camera's tilt, both grippers' distance and stillness.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
import demokit as d  # noqa: E402  (grips only: the waypoint demo below is played by yam_scripted)
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
INSTRUCTION = "Fasten the GoPro to the helmet's mount with its thumb screw, facing forward and level."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the helmet and the mount: the mount's frame has its origin on the crown, x forward (the brim), y left, z up -------------------------
HAT_X = 0.30
CROWN_Z = 0.1735  # the hard hat's crown above the table (its file's own geometry)
MOUNT_POS = (HAT_X, 0.0, TOP + CROWN_Z)
BASE = (0.042, 0.040, 0.004)  # curved adhesive mount's base plate (x, y, z)
BODY = (0.026, 0.024, 0.008)  # the buckle's body, on the base
Z_ROOT = BASE[2] + BODY[2]  # where the fingers rise from (the slots' floor)
Z_AX = 0.020  # the holes' axis above the crown
R_END = 0.006  # the fingers' rounded ends, about the hole
T_F = 0.0030  # every finger's thickness
GAP = 0.0033  # the buckle's gaps
P1 = (-0.0078, -0.0048)  # the right outer finger (screw head side), y from .. to
P3 = (-0.0015, 0.0015)  # the middle finger
P5 = (0.0048, 0.0078)  # the left outer finger, with the nut
CAM_F = (-0.00315, 0.00315)  # the camera's fingers' mid-planes, when centred in the gaps
R_HOLE = 0.0026  # the fingers' holes (5.2 mm)
R_SHANK = 0.00245  # the screw (M5: 4.9 mm over the thread)
CHAMFER = 0.00045  # the screw tip's chamfer
CLAMP_TRAVEL = 0.0004  # the outer fingers' inward travel when the screw is seated
K_PRONG = 1.0e5  # N/m

# -- the thumb screw (along +y: the tip leads), at the start and as it goes in
L_THREAD, L_NECK, PADDLE = 0.020, 0.022, (0.018, 0.012, 0.006)  # paddle: wide (x at no turn), along the axis (y), thick (z)
R_NECK = 0.0045
TIP0 = P1[1] - 0.00015  # the tip at the start: just inside the outer finger's hole
NUT_IN = P5[1] + 0.00075  # the nut's inner face, in its hex pocket outside the left outer finger
NUT = (0.008, 0.004)  # M5 nut: across flats, thick
PITCH = 0.0008
N_TURNS = 4
ENGAGE_Y = NUT_IN + CHAMFER  # the tip here: the thread starts to catch
S_ENGAGE = ENGAGE_Y - TIP0  # 13.95 mm of free travel
S_SEAT = S_ENGAGE + N_TURNS * PITCH  # the head's shoulder against the outer finger
assert abs((TIP0 + S_SEAT - L_THREAD) - P1[0]) < 1e-9, "seated: the shoulder meets the outer finger"
K_THREAD = 5.0e4  # N/m: the thread's hold on the screw's axial position

# -- the camera: GoPro HERO (2024), 56.6 wide, 47.7 high, 29.4 deep, 86 g with its fingers. Its frame: origin on the fingers' hole axis
#    midway between them, x out of the lens, y to its left, z up. The holes are 10 mm below the body.
CAM = (0.0294, 0.0566, 0.0477)
DROP = 0.010
CAM_MASS = 0.086
CAM_COM = (0.0, 0.0, DROP + CAM[2] / 2)
LENS = (0.011, 0.004, -0.012, DROP + 0.030)  # radius, standing out of the front, y, z
CAM_START = (0.24, 0.20, TOP + CAM[1] / 2 + 0.0002)
CAM_START_QUAT = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # lying on its right side: its y up, its z (top) towards -y

LOOKS = {
    "plastic": m.Look(color=m.rgb(0.06, 0.06, 0.065), roughness=0.55),
    "plastic_grey": m.Look(color=m.rgb(0.22, 0.22, 0.23), roughness=0.5),
    "rubber": m.Look(color=m.rgb(0.035, 0.035, 0.04), roughness=0.85),
    "glass": m.Look(color=m.rgb(0.02, 0.025, 0.035), roughness=0.08, metallic=0.2),
    "screen": m.Look(color=m.rgb(0.04, 0.06, 0.08), roughness=0.1),
    "steel": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.3),
    "button": m.Look(color=m.rgb(0.35, 0.36, 0.38), roughness=0.4),
    "pad": m.Look(color=m.rgb(0.85, 0.85, 0.82), roughness=0.9),
}


def _y_cyl(r, y0, y1, x=0.0, z=0.0, segments=32):
    return m.cylinder(r, y1 - y0, (x, (y0 + y1) / 2, z), axis="y", segments=segments)


def _hull_pts(pts) -> m.Mesh:
    """A closed mesh of the convex hull of `pts` (for a collider: `col_hull` takes the hull of any mesh's points)."""
    pts = np.asarray(pts, float)
    n = len(pts)
    faces = np.array([[0, i, i + 1] for i in range(1, n - 1)])  # only the points matter to a convex-hull collider
    return m.Mesh(pts, faces)


def _stadium(z_root, z_ax, r, n=16):
    """A finger's outline in its plane (x, z): straight sides from `z_root` to the axis, a half circle of radius r above it."""
    pts = [(r, z_root), (r, z_ax)] + [(r * math.cos(a), z_ax + r * math.sin(a)) for a in np.linspace(0, math.pi, n + 1)[1:-1]]
    return pts + [(-r, z_ax), (-r, z_root)]


def _prong_hull(name, y0, y1, chamfer_lo, chamfer_hi):
    """A buckle finger: its stadium outline, y0..y1 thick, its top 2.5 mm chamfered 0.5 mm on the faces that meet a gap."""
    pts = []
    for x, z in _stadium(Z_ROOT, Z_AX, R_END):
        f = max(0.0, (z - (Z_AX + R_END - 0.0025)) / 0.0025)
        pts += [(x, y0 + (0.0005 * f if chamfer_lo else 0.0), z), (x, y1 - (0.0005 * f if chamfer_hi else 0.0), z)]
    return m.col_hull(name, _hull_pts(pts))


def _prong_visual(y0, y1):
    outline = [(x, z) for x, z in _stadium(Z_ROOT, Z_AX, R_END, 24)]
    ring = m.prism(np.array(outline), y0, y1)  # built along z: turn so that its thickness runs along y
    v = ring.v[:, [0, 2, 1]]
    return m.Mesh(v, ring.f[:, ::-1])


# -- the mount ---------------------------------------------------------------------------------------------------------------------------


def _mount_frame():
    parts, cols = [], []
    parts.append(("adhesive", m.box((BASE[0] - 0.002, BASE[1] - 0.002, 0.0012), (0, 0, 0.0006)), "pad"))
    parts.append(("base", m.box((BASE[0], BASE[1], BASE[2] - 0.0012), (0, 0, 0.0012 + (BASE[2] - 0.0012) / 2)), "plastic"))
    parts.append(("buckle", m.box(BODY, (0, 0, BASE[2] + BODY[2] / 2)), "plastic"))
    cols.append(m.col_box("base_c", BASE, (0, 0, BASE[2] / 2)))
    cols.append(m.col_box("buckle_c", BODY, (0, 0, BASE[2] + BODY[2] / 2)))
    parts.append(("finger_mid", _prong_visual(*P3), "plastic"))
    cols.append(_prong_hull("finger_mid_c", *P3, True, True))
    # the nut's hex boss outside the left outer finger, with the nut in it
    hexo = np.array([(0.0055 * math.cos(a), 0.0055 * math.sin(a)) for a in np.radians(np.arange(0, 360, 60) + 30)])
    boss = m.prism(hexo, P5[1], NUT_IN + NUT[1])
    boss = m.Mesh(boss.v[:, [0, 2, 1]] + np.array([0, 0, Z_AX]), boss.f[:, ::-1])
    parts.append(("nut_boss", boss, "plastic"))
    hexn = np.array([(NUT[0] / math.sqrt(3) * math.cos(a), NUT[0] / math.sqrt(3) * math.sin(a)) for a in np.radians(np.arange(0, 360, 60))])
    nut = m.prism(hexn, NUT_IN + NUT[1] - 0.0002, NUT_IN + NUT[1] + 0.0003)
    parts.append(("nut", m.Mesh(nut.v[:, [0, 2, 1]] + np.array([0, 0, Z_AX]), nut.f[:, ::-1]), "steel"))
    cols.append(m.col_hull("nut_boss_c", boss))
    # the boss joins the outer finger's lower part to the body (visual web)
    parts.append(("boss_web", m.box((0.008, NUT_IN + NUT[1] - P5[1], Z_AX - Z_ROOT), (0, (P5[1] + NUT_IN + NUT[1]) / 2, (Z_ROOT + Z_AX) / 2)), "plastic"))
    cols.append(m.col_box("boss_web_c", (0.008, NUT_IN + NUT[1] - P5[1], Z_AX - Z_ROOT), (0, (P5[1] + NUT_IN + NUT[1]) / 2, (Z_ROOT + Z_AX) / 2)))
    return parts, cols


def _outer_finger(name, y0, y1, inward):
    lo, hi = (-0.0001, CLAMP_TRAVEL + 0.0001) if inward > 0 else (-CLAMP_TRAVEL - 0.0001, 0.0001)
    return m.slider(name, "Y", lo, hi, [(name, _prong_visual(y0, y1), "plastic")], [_prong_hull(name + "_c", y0, y1, inward < 0, inward > 0)],
                    at=(0.0, (y0 + y1) / 2, Z_ROOT), mass=0.01, spring=K_PRONG, damping=200.0, friction=0.0, effort=100.0, armature=0.01)


def _screw_parts():
    """The screw at its start (no turn), in the mount's frame: shank and tip along +y, neck and paddle behind."""
    tip, shoulder = TIP0, TIP0 - L_THREAD
    n = 24
    ring = lambda r, y: [(r * math.cos(a), y, Z_AX + r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    shank_pts = ring(R_SHANK, shoulder) + ring(R_SHANK, tip - CHAMFER) + ring(R_SHANK - CHAMFER, tip)
    neck = _y_cyl(R_NECK, shoulder - L_NECK, shoulder, z=Z_AX)
    pad_c = (0.0, shoulder - L_NECK - PADDLE[1] / 2, Z_AX)
    paddle = m.box(PADDLE, pad_c)
    parts = [("shank", _y_cyl(R_SHANK, shoulder, tip - CHAMFER, z=Z_AX, segments=24), "steel"),
             ("tip", m.lathe([(0.0, 0.0), (R_SHANK - CHAMFER, 0.0), (R_SHANK, CHAMFER)], 24).turned(90, "x").moved((0, tip, Z_AX)), "steel"),
             ("neck", neck, "plastic"), ("paddle", paddle, "plastic"),
             ("grip_l", m.box((0.0012, PADDLE[1] * 0.8, PADDLE[2] + 0.0006), (-0.006, pad_c[1], Z_AX)), "plastic_grey"),
             ("grip_r", m.box((0.0012, PADDLE[1] * 0.8, PADDLE[2] + 0.0006), (0.006, pad_c[1], Z_AX)), "plastic_grey")]
    cols = [m.col_hull("shank_c", _hull_pts(shank_pts)), m.col_hull("neck_c", _y_cyl(R_NECK, shoulder - L_NECK, shoulder, z=Z_AX, segments=16)),
            m.col_box("paddle_c", PADDLE, pad_c)]
    return parts, cols


def _mount():
    fp, fc = _mount_frame()
    sp, sc = _screw_parts()
    screw = m.slider("screw", "Y", -0.010, S_SEAT + 0.0002, [], [], at=(0.0, TIP0, Z_AX), mass=0.004, spring=0.0, damping=0.5, friction=0.02,
                     effort=400.0, armature=0.01)
    turn = m.hinge("turn", "Y", (0.0, TIP0, Z_AX), -math.inf, math.inf, sp, sc, mass=0.007, damping=0.002, friction=0.01, effort=50.0,
                   armature=1.0e-4, parent="screw", inertia=(3.0e-6, 3.0e-7, 3.0e-6))
    return m.mechanism("mount", MOUNT_POS, [screw, turn, _outer_finger("p1", *P1, +1), _outer_finger("p5", *P5, -1)], LOOKS,
                       frame_parts=fp, frame_colliders=fc, friction=(0.4, 0.35), solver_iters=48)


MOUNT = _mount()

# -- the camera ----------------------------------------------------------------------------------------------------------------------------


def _finger_pieces(name, yc, n=16):
    """One of the camera's fingers as n convex pieces round its 5.2 mm hole (the finger's outline: 12 mm wide, square up to the body, a
    6 mm half circle below the hole)."""
    cols = []
    y0, y1 = yc - T_F / 2, yc + T_F / 2

    def outer(a):
        c, s = math.cos(a), math.sin(a)
        if s < 0:
            return (R_END * c, R_END * s)
        t = min(R_END / abs(c) if abs(c) > 1e-9 else 1e9, DROP / s if s > 1e-9 else 1e9)
        return (t * c, t * s)

    corners = [math.atan2(DROP, R_END), math.atan2(DROP, -R_END)]
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n + math.pi / n, 2 * math.pi * (k + 1) / n + math.pi / n
        ring = [(R_HOLE * math.cos(a), R_HOLE * math.sin(a)) for a in (a0, a1)]
        out = [outer(a) for a in np.linspace(a0, a1, 4)]
        for c in corners:
            for cc in (c, c + 2 * math.pi):
                if a0 < cc < a1:
                    out.append((R_END if math.cos(c) > 0 else -R_END, DROP))
        pts = [(x, y, z) for x, z in ring + out for y in (y0, y1)]
        cols.append(m.col_hull(f"{name}_{k}", _hull_pts(pts)))
    return cols


def _finger_visual(yc):
    # counter-clockwise in (x, z): the half circle below the hole, then straight up to the body
    outline = [(R_END * math.cos(a), R_END * math.sin(a)) for a in np.linspace(math.pi, 2 * math.pi, 25)] + [(R_END, DROP), (-R_END, DROP)]
    plate = m.prism(np.array(outline), yc - T_F / 2, yc + T_F / 2)
    return m.Mesh(plate.v[:, [0, 2, 1]], plate.f[:, ::-1])


def _rounded_rect(w, h, r, n=6):
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180), (w / 2 - r, -h / 2 + r, 270)):
        for a in np.radians(np.linspace(a0, a0 + 90, n)):
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return np.array(pts)


def _camera():
    body = m.prism(_rounded_rect(CAM[0], CAM[1], 0.004), DROP, DROP + CAM[2])
    lr, lout, ly, lz = LENS
    lens = m.cylinder(lr, lout, (CAM[0] / 2 + lout / 2, ly, lz), axis="x", segments=32)
    glass = m.cylinder(lr * 0.72, 0.0006, (CAM[0] / 2 + lout + 0.0002, ly, lz), axis="x", segments=32)
    screen = m.box((0.0006, 0.034, 0.030), (-CAM[0] / 2 - 0.0002, 0.003, DROP + 0.025))
    shutter = m.cylinder(0.0045, 0.0012, (0.002, -0.010, DROP + CAM[2] + 0.0006), segments=24)
    parts = [("body", body, "rubber"), ("lens", lens, "plastic"), ("glass", glass, "glass"), ("screen", screen, "screen"), ("shutter", shutter, "button"),
             ("finger_a", _finger_visual(CAM_F[0]), "plastic"), ("finger_b", _finger_visual(CAM_F[1]), "plastic"),
             ("recess", m.box((0.016, 0.014, 0.0008), (0, 0, DROP + 0.0003)), "plastic_grey")]
    cols = [m.col_box("body_c", CAM, (0, 0, DROP + CAM[2] / 2)), m.col_hull("lens_c", m.cylinder(lr, lout, (CAM[0] / 2 + lout / 2, ly, lz), axis="x", segments=16))]
    cols += _finger_pieces("fa", CAM_F[0]) + _finger_pieces("fb", CAM_F[1])
    return m.free("camera", CAM_START, parts, cols, LOOKS, mass=CAM_MASS, friction=(0.7, 0.6), start_quat=CAM_START_QUAT, com=CAM_COM, solver_iters=48)


CAMERA = _camera()

# -- the helmet (SimReady hard hat, fixed) and the distractors (SimReady) ----------------------------------------------------------------
HELMET = SimReady("helmet", pos=(HAT_X, 0.0, TOP), size=(0.290, 0.208, 0.173), package="ppe_helmet_hardHat_a01", usd="sm_ppe_helmet_hardHat_a01_01.usd")
SUNGLASSES = SimReady("sunglasses", pos=(0.50, 0.24, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                      usd="sm_accessory_eyeglasses_a01_01.usd", yaw=math.pi / 2)
BOTTLE = SimReady("bottle", pos=(0.47, -0.24, TOP), size=(0.053, 0.053, 0.175), mass=0.55, package="food_beverage_bottledDrink_a01",
                  usd="sm_food_beverage_bottledDrink_a01_01.usd")
GLOVES = SimReady("gloves", pos=(0.58, -0.02, TOP), size=(0.233, 0.252, 0.053), mass=0.12, package="ppe_gloves_cloth_a01",
                  usd="sm_ppe_gloves_cloth_a01_01.usd", yaw=0.0)

OBJECTS = [HELMET, MOUNT, CAMERA, SUNGLASSES, BOTTLE, GLOVES]

# -- the thread ----------------------------------------------------------------------------------------------------------------------------


def _drive(env, part, link, target, stiffness, effort=None):
    asset, j = m._joint(env, part, link)
    m._TOUCHED.add((part, link))
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    if effort is not None:
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


class Thread:
    """The screw's thread in the captive nut: free travel up to the nut, then axial position = 0.8 mm per turn; held at the seat."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.engaged = False
        self.turn0 = 0.0
        self.seated = False
        self.raw = None  # PhysX reports an unlimited hinge wrapped to +-2 pi: the screw's turn is unwrapped here
        self.turn = 0.0
        if env is not None:
            _drive(env, "mount", "screw", 0.0, 0.0)

    def step(self, env):
        st = m.state(env, "mount")
        s, raw = st.joint("screw"), st.joint("turn")
        if self.raw is not None:
            self.turn += (raw - self.raw + math.pi) % (2 * math.pi) - math.pi
        else:
            self.turn = raw
        self.raw = raw
        th = self.turn
        if not self.engaged and s >= S_ENGAGE - 0.0001:
            self.engaged, self.turn0 = True, th  # the thread catches at the screw's turn now
        if self.engaged:
            adv = PITCH * (th - self.turn0) / (2 * math.pi)
            if adv < -0.0003 and not self.seated:  # unscrewed back out of the nut
                self.engaged = False
                _drive(env, "mount", "screw", 0.0, 0.0)
                return
            _drive(env, "mount", "screw", min(S_ENGAGE + max(adv, 0.0), S_SEAT), K_THREAD, 400.0)
            if not self.seated and adv >= N_TURNS * PITCH - 1e-6:
                self.seated = True
                m.lock(env, "mount", "turn")
                m.set_target(env, "mount", "p1", CLAMP_TRAVEL)
                m.set_target(env, "mount", "p5", -CLAMP_TRAVEL)


THREAD = Thread()
m.on_step(THREAD.step)
m.on_reset(THREAD.reset)

# -- the scorer ----------------------------------------------------------------------------------------------------------------------------
CLEAR = 0.03  # m: both grippers this far from the camera, the screw and the buckle
LEVEL_DEG = 5.0
PIN_TOL = 0.001  # m: a camera finger's hole centre this close to the screw's axis
PINNED_STANDOFF = 0.0070  # the head's standoff from the outer finger with the tip through both of the camera's fingers
THREADED_STANDOFF = 0.0030  # at least a quarter turn into the nut (3.2 mm is where the tip meets the nut's thread)
SEATED_STANDOFF = 0.0005
STILL_MM, STILL_DEG = 1.0, 0.5  # during the 2 s hold the camera stays this close to where it was when the hold began


def _box_dist(local, lo, hi):
    d = np.maximum(np.maximum(lo - local, local - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def now(env) -> dict:
    mt, cam = m.state(env, "mount"), m.state(env, "camera")
    s = mt.joint("screw")
    tip = TIP0 + s
    standoff = S_SEAT - s
    holes = mt.to_local(cam.to_world(np.array([[0.0, CAM_F[0], 0.0], [0.0, CAM_F[1], 0.0]])))
    off = np.hypot(holes[:, 0], holes[:, 2] - Z_AX)
    on_axis = bool(np.all(off <= PIN_TOL) and np.all(np.abs(holes[:, 1] - np.array(CAM_F)) <= 0.0015))
    pinned = on_axis and standoff <= PINNED_STANDOFF  # the tip at least 0.55 mm past the camera's second finger
    lens = cam.rotation()[:, 0]
    tilt = math.degrees(math.acos(float(np.clip(lens[0], -1.0, 1.0))))
    tips = m.fingertips(env)
    d_cam = _box_dist(cam.to_local(tips), np.array([-CAM[0] / 2, -CAM[1] / 2, -R_END]), np.array([CAM[0] / 2 + LENS[1], CAM[1] / 2, DROP + CAM[2]]))
    q = mt.to_local(tips)
    a, b = np.array([0.0, tip - L_THREAD - L_NECK - PADDLE[1], Z_AX]), np.array([0.0, tip, Z_AX])
    u = np.clip((q - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0)
    d_screw = np.linalg.norm(q - (a + u[:, None] * (b - a)), axis=1) - PADDLE[0] / 2
    d_mount = _box_dist(q, np.array([-BASE[0] / 2, -BASE[1] / 2, 0.0]), np.array([BASE[0] / 2, BASE[1] / 2, Z_AX + R_END]))
    gap = float(min(d_cam.min(), d_screw.min(), d_mount.min()))
    speed = (float(np.linalg.norm(cam.lin_vel)), float(np.linalg.norm(cam.ang_vel)))
    return {"s": s, "tip": tip, "standoff": standoff, "off": off, "on_axis": on_axis, "pinned": pinned, "tilt": tilt, "gap": gap,
            "clear": gap >= CLEAR, "pos": np.array(cam.pos), "speed": speed, "engaged": THREAD.engaged, "seated": THREAD.seated}


def level_of(c) -> float:
    """The partial credit of an end state (before the 2 s hold that full credit needs)."""
    if not (c["clear"] and c["pinned"]):
        return 0.0
    if c["standoff"] <= SEATED_STANDOFF:
        return 0.75
    if c["standoff"] <= THREADED_STANDOFF:
        return 0.5
    return 0.25


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the trial's record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        self.ref = None  # the camera's position and tilt when the current hold began

    def step(self, env):
        c = now(env)
        self.last = c
        ok = level_of(c) == 0.75 and c["tilt"] <= LEVEL_DEG
        if ok and self.hold > 0:  # still: within 1 mm and 0.5 degrees of where the hold began
            ok = float(np.linalg.norm(c["pos"] - self.ref[0])) <= STILL_MM / 1000 and abs(c["tilt"] - self.ref[1]) <= STILL_DEG
        if ok and self.hold == 0:
            self.ref = (c["pos"], c["tilt"])
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else level_of(c)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)

def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published starting layout) ----------------------------------------------------------------------


def _rot(axis, deg):
    return axis_angle_matrix(np.array(axis, float), math.radians(deg))


AX_W = np.array(MOUNT_POS) + np.array([0.0, 0.0, Z_AX])  # a point on the holes' axis (world)
T_CAM = np.array([0.0, 0.0, DROP + CAM[2] / 2])  # the left hand's TCP in the camera's frame: at its centre of mass, so it turns without slipping
DOWN_L = d.down(-90).rot  # straight down, fingers closing along x
LEVEL_L = _rot((1, 0, 0), -90) @ DOWN_L  # pointing -y, fingers closing along x: the same grip turned with the camera upright
LIFT = 0.15


def _grip(R):
    return d._as_grip(R)


def _screw_grip(a):
    """The right hand level, pointing +y along the screw, turned `a` degrees about it (-90: fingers closing along z)."""
    return _grip(_rot((0, 1, 0), a) @ d.level(90).rot)


# feed-forward offsets measured in earlier runs: the arm sags about 2 to 3 mm holding the camera level, and the front pad bears on the
# lens, so the camera sits about 3.5 mm behind the pinch's centre and pitched 1.5 to 2 degrees
CAM_DZ = 0.0030
CAM_DX = 0.0036
CAM_DY = 0.0


class _HoldServo:
    """The demo's one closed loop (simulator state, which a scripted demo may read; the evaluated model gets only the cameras): while
    the camera is upright with its fingers in the buckle's gaps and before the screw reaches it, the left hand's held position is
    trimmed by the measured offset of the camera's finger holes from the buckle's hole axis (in x and z), and then kept. It plays the
    part of a person's eye lining the holes up."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.corr = np.zeros(3)

    def step(self, env):
        mt, cam = m.state(env, "mount"), m.state(env, "camera")
        if mt.joint("screw") > 0.0015:  # the screw has reached the camera: hold where it is
            return
        holes = mt.to_local(cam.to_world(np.array([[0.0, CAM_F[0], 0.0], [0.0, CAM_F[1], 0.0]]))).mean(axis=0)
        err = np.array([holes[0], 0.0, holes[2] - Z_AX])
        upright = cam.rotation()[2, 2] > math.cos(math.radians(15.0))
        if upright and abs(holes[1]) < 0.003 and abs(err[0]) < 0.008 and abs(err[2]) < 0.004:
            self.corr = np.clip(self.corr - np.clip(0.1 * err, -0.00015, 0.00015), -0.006, 0.006)


SERVO = _HoldServo()
m.on_step(SERVO.step)
m.on_reset(SERVO.reset)


class _Live:
    """A waypoint position read when the plan is played: a fixed point plus the hold servo's correction."""

    def __init__(self, base):
        self.base = np.asarray(base, float)

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.base + SERVO.corr, dtype=dtype or float)


def demo():
    cam0 = np.array(CAM_START) - np.array([0, 0, 0.0002])
    R0 = m.quat_matrix(CAM_START_QUAT)
    grasp = cam0 + R0 @ T_CAM
    up = np.array([0.0, 0.0, 1.0])
    gL0, gL1 = _grip(DOWN_L), _grip(LEVEL_L)
    held = AX_W + T_CAM + np.array([CAM_DX, CAM_DY, CAM_DZ])  # the TCP with the camera's holes on the axis
    left = [
        Waypoint(2.5, tuple(grasp + 0.10 * up), FINGER_OPEN * 0.8, "over the camera", gL0),
        Waypoint(1.5, tuple(grasp), FINGER_OPEN * 0.8, "fingers either side of the camera", gL0),
        Waypoint(0.8, tuple(grasp), FINGER_CLOSED, "pinch it", gL0),
        Waypoint(1.5, tuple(grasp + LIFT * up), FINGER_CLOSED, "lift", gL0),
        Waypoint(2.5, (HAT_X - 0.02, 0.07, TOP + 0.27), FINGER_CLOSED, "turn it upright, clear of the helmet", gL1),
        Waypoint(2.0, tuple(held + 0.035 * up), FINGER_CLOSED, "over the buckle", gL1),
        Waypoint(1.5, tuple(held + 0.012 * up), FINGER_CLOSED, "fingers at the gaps", gL1),
        Waypoint(1.5, _Live(held), FINGER_CLOSED, "down between the buckle's fingers, holes on the axis", gL1),
        Waypoint(3.0, _Live(held), FINGER_CLOSED, "line the holes up", gL1),
    ]
    t_left = sum(w.duration for w in left)
    # the right hand: pinch the paddle (lying flat at the start) from its outer end, push the screw to the nut, then half turns
    pad_out0 = TIP0 - L_THREAD - L_NECK - PADDLE[1]  # the paddle's outer end at the start (mount y)
    def tcp(s_mm):
        return AX_W + np.array([0.0, pad_out0 - 0.002 + s_mm, 0.0])
    right = [
        Waypoint(3.0, tuple(tcp(-0.045)), FINGER_OPEN * 0.3, "beside the screw's head", _screw_grip(-90)),
        Waypoint(1.5, tuple(tcp(0.0)), FINGER_OPEN * 0.3, "fingers above and below the paddle", _screw_grip(-90)),
        Waypoint(0.6, tuple(tcp(0.0)), FINGER_CLOSED, "pinch the paddle", _screw_grip(-90)),
    ]
    wait = t_left + 0.5 - sum(w.duration for w in right)
    right.append(Waypoint(wait, tuple(tcp(0.0)), FINGER_CLOSED, "wait for the camera", _screw_grip(-90)))
    # push the screw through all five fingers to the nut, twisting it 30 degrees to and fro as it goes (it is free until the nut)
    push = S_ENGAGE + 0.0008
    for s_to, a, t in ((0.0015, -90, 0.8), (0.0035, -60, 0.6), (0.0055, -90, 0.6), (0.0080, -60, 0.6), (0.0105, -90, 0.6), (0.0125, -60, 0.6),
                       (push, -90, 0.8)):
        right.append(Waypoint(t, tuple(tcp(s_to)), FINGER_CLOSED, "push the screw through all five fingers to the nut, twisting", _screw_grip(a)))
    right.append(Waypoint(0.4, tuple(tcp(push)), FINGER_CLOSED, "at the nut", _screw_grip(-90)))
    s = push
    halves = 2 * N_TURNS + 1  # one half turn more than the thread needs: turned until it stops, as by hand
    for k in range(halves):
        right.append(Waypoint(1.0, tuple(tcp(min(s + PITCH / 4, S_SEAT + 0.0005))), FINGER_CLOSED, f"half turn {k + 1}", _screw_grip(0)))
        s = min(s + PITCH / 2, S_SEAT + 0.0005)
        right.append(Waypoint(1.0, tuple(tcp(s)), FINGER_CLOSED, f"half turn {k + 1}", _screw_grip(90)))
        right.append(Waypoint(0.3, tuple(tcp(s)), FINGER_CLOSED, "settle", _screw_grip(90)))
        right.append(Waypoint(0.4, tuple(tcp(s)), FINGER_OPEN * 0.35, "let go of the paddle", _screw_grip(90)))
        if k < halves - 1:
            right.append(Waypoint(0.6, tuple(tcp(s)), FINGER_OPEN * 0.35, "turn back", _screw_grip(0)))
            right.append(Waypoint(0.6, tuple(tcp(s)), FINGER_OPEN * 0.35, "turn back", _screw_grip(-90)))
            right.append(Waypoint(0.4, tuple(tcp(s)), FINGER_CLOSED, "pinch again", _screw_grip(-90)))
    right.append(Waypoint(1.2, tuple(tcp(s) - np.array([0.0, 0.05, 0.0])), FINGER_OPEN * 0.35, "back off the paddle", _screw_grip(90)))
    right.append(Waypoint(1.5, tuple(tcp(s) - np.array([0.0, 0.06, -0.03])), FINGER_OPEN * 0.35, "away", _screw_grip(90)))
    t_right_done = sum(w.duration for w in right) - 2.7
    left.append(Waypoint(t_right_done - t_left + 0.3, _Live(held), FINGER_CLOSED, "hold the camera level while the screw goes in", gL1))
    left.append(Waypoint(0.6, _Live(held), FINGER_OPEN * 0.6, "let go", gL1))
    left.append(Waypoint(1.2, _Live(held + np.array([0.0, 0.06, 0.0])), FINGER_OPEN * 0.6, "off the camera", gL1))
    left.append(Waypoint(1.5, _Live(held + np.array([0.0, 0.11, 0.02])), FINGER_OPEN * 0.6, "away", gL1))
    return {"left": left, "right": right}
