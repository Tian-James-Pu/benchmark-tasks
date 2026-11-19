"""Re-clip a wall cabinet's door onto its lower clip-on hinge, holding the sagging door square, then shut it.

A 300 mm wide, 400 mm tall, 300 mm deep wall cabinet (18 mm white melamine-faced particleboard) hangs on a wall section
behind the bench, its underside 30 mm above the bench, its opening towards the robot. Its door (296 x 396 x 16 mm, grey
melamine-faced particleboard, 1.3 kg with its knob) hangs on two 35 mm clip-on cup hinges on the cabinet's right-hand
side (the robot's right). The lower hinge's arm has come off its mounting plate: the door stands open 90 degrees, hangs
from the upper hinge alone and has sagged 4 degrees about it, so its free bottom corner is 21 mm low and the loose arm
sits 15 mm behind its plate and 12 mm proud of it. A hinge arm only clips onto its plate when the door is held square
(within 0.5 degrees), and then needs a firm press until it clicks. Clipped, the door swings on both hinges and the
hinges' springs pull it shut over the last 35 degrees.

Mechanism (mechkit): a "sag" hinge about the door's normal through the upper hinge (-4 to +3 degrees, the door's weight
holds it on -4), carrying the door's swing hinge (closed to 110 degrees open) and the lower arm on a 12 mm slide towards
its plate. The click is a rule: when the arm is pressed to within 1 mm of home with the door within 0.5 degrees of
square, both the sag hinge and the arm's slide lock where they are.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
import yam_arena.demo as _harness_demo  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "This cabinet door has come off its bottom hinge. Hang it back on both hinges and close it."
SUCCESS_HOLD_S = 3.0

# -- the cabinet (metres; x forward from the arm bases, y left, z above the bench) ---------------------------------------
XF, XB = 0.400, 0.700  # carcass front and back faces
YR, YL = -0.100, 0.200  # carcass right and left outer faces (300 mm wide)
ZB, ZT = 0.030, 0.430  # carcass underside and top (400 mm tall), hung 30 mm above the bench
T = 0.018  # carcass panels
T_BACK = 0.008
Y_IN_R = YR + T  # inside face of the right-hand side, where the hinge plates are: -0.082

# -- the door, as authored: open 90 degrees, square -----------------------------------------------------------------------
DW, DH, DT = 0.296, 0.396, 0.016
DOOR_MASS = 1.30
PIVOT = (XF, YR - 0.001)  # the swing axis (vertical), at the carcass's front right corner
D_IN = PIVOT[1] - 0.003  # door's inner face at 90 degrees: y -0.104; shut, it stands 3 mm off the carcass (its bumpers)
D_OUT = D_IN - DT  # outer face: y -0.120
D_X1 = XF - 0.002  # hinge edge
D_X0 = D_X1 - DW  # free edge: x 0.102
D_Z0 = ZB + 0.002
D_Z1 = D_Z0 + DH
HINGE_Z = (D_Z0 + 0.090, D_Z1 - 0.090)  # cup centres, 90 mm from the door's ends: 0.122 and 0.338
SAG_AT = (XF, PIVOT[1], HINGE_Z[1])  # the door hangs from the upper hinge: it sags about the door's normal through it
SAG_DEG = -4.0  # the door's weight holds the sag hinge on this stop (free corner down)
SAG_UP_DEG = 3.0  # over-lifted
SQUARE_TOL_DEG = 0.5  # the arm's hook catches its plate's nose only within this of square (1.9 mm at the arm)
KNOB = (D_X0 + 0.035, D_OUT - 0.028, D_Z0 + 0.060)  # head centre: 35 mm in from the free edge, 60 mm up, 28 mm out
KNOB_R, KNOB_STEM_R = 0.016, 0.0065
CUP_X = D_X1 - 0.0225

# -- the lower hinge's arm and plate --------------------------------------------------------------------------------------
PLATE_X = XF + 0.037  # system-32 line: the plate's centre, 37 mm behind the front edge
ARM_LEN, ARM_W, ARM_H = 0.065, 0.018, 0.015  # along x, z, y
ARM_X = (XF + 0.003, XF + 0.003 + ARM_LEN)
TRAVEL = 0.012  # proud of home when loose
ARM_HOME_Y = (Y_IN_R + 0.0005, Y_IN_R + 0.0005 + ARM_H)  # seated: y -0.0815 to -0.0665
ARM_Y = (ARM_HOME_Y[0] + TRAVEL, ARM_HOME_Y[1] + TRAVEL)  # authored loose (12 mm proud): -0.0695 to -0.0545
ARM_K = 300.0  # N/m: what pushes a loose arm back off its plate
CLICK_DEPTH = TRAVEL - 0.001  # pressed to within 1 mm of home

# -- self-closing hinge springs -----------------------------------------------------------------------------------------
CLOSED_DEG = -90.0  # the swing joint's value when the door is shut (zero is the authored 90 degrees open)
SELF_CLOSE_DEG = 35.0  # the springs act over the last 35 degrees
CLOSE_K = 0.6  # N m/rad, towards a rest 10 degrees past shut: 0.1 N m holds it shut, 0.47 N m at 35 degrees open
CLOSE_PAST_DEG = 10.0
SHUT_TOL_DEG = 1.0  # shut: within 1 degree of the stop, its free edge within 5 mm of closed

GREYS = {
    "carcass": m.Look(color=m.rgb(0.93, 0.93, 0.91), roughness=0.45),
    "edge": m.Look(color=m.rgb(0.86, 0.86, 0.84), roughness=0.5),
    "wall": m.Look(color=m.rgb(0.80, 0.77, 0.70), roughness=0.85),
    "door": m.Look(color=m.rgb(0.58, 0.63, 0.62), roughness=0.4),
    "steel": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.9, roughness=0.25),
    "zinc": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.8, roughness=0.35),
    "cup": m.Look(color=m.rgb(0.70, 0.71, 0.72), metallic=0.85, roughness=0.3),
    "screw": m.Look(color=m.rgb(0.45, 0.45, 0.47), metallic=0.9, roughness=0.3),
    "knob": m.Look(color=m.rgb(0.20, 0.20, 0.21), metallic=0.6, roughness=0.35),
}


def _span_box(x, y, z):
    """A box from (x0, x1), (y0, y1), (z0, z1): its size and centre."""
    size = (x[1] - x[0], y[1] - y[0], z[1] - z[0])
    centre = ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2)
    return size, centre


def _carcass() -> m.Part:
    parts, cols = [], []

    def add(name, x, y, z, look="carcass"):
        size, c = _span_box(x, y, z)
        parts.append((name, m.box(size, c), look))
        cols.append(m.col_box(f"{name}_c", size, c))

    add("side_r", (XF, XB), (YR, YR + T), (ZB, ZT))
    add("side_l", (XF, XB), (YL - T, YL), (ZB, ZT))
    add("bottom", (XF, XB - T_BACK), (YR + T, YL - T), (ZB, ZB + T))
    add("top", (XF, XB - T_BACK), (YR + T, YL - T), (ZT - T, ZT))
    add("back", (XB - T_BACK, XB), (YR + T, YL - T), (ZB + T, ZT - T), "edge")
    add("wall", (XB, XB + 0.0125), (-0.32, 0.42), (0.0, 0.62), "wall")
    return m.fixed("cabinet", (0.0, 0.0, TOP), parts, cols, GREYS, friction=(0.6, 0.5))


def _plate(z: float, tag: str) -> list:
    """A cruciform mounting plate on the right-hand side's inside face: a rail along x under the arm, wings up and down."""
    rail = m.box(*_span_box((PLATE_X - 0.020, PLATE_X + 0.020), (Y_IN_R, Y_IN_R + 0.009), (z - 0.007, z + 0.007)))
    wings = m.box(*_span_box((PLATE_X - 0.007, PLATE_X + 0.007), (Y_IN_R, Y_IN_R + 0.003), (z - 0.026, z + 0.026)))
    out = [(f"rail_{tag}", rail, "zinc"), (f"wings_{tag}", wings, "zinc")]
    for k, dz in enumerate((-0.019, 0.019)):
        head = m.cylinder(0.004, 0.002, (PLATE_X, Y_IN_R + 0.004, z + dz), axis="y", segments=16)
        out.append((f"screw_{tag}{k}", head, "screw"))
    return out


def _arm_mesh(y0: float, y1: float, z: float):
    """A hinge arm lying along the side panel from just behind the front edge, over its plate."""
    body = m.box(*_span_box(ARM_X, (y0, y1), (z - ARM_W / 2, z + ARM_W / 2)))
    boss = m.box(*_span_box((XF + 0.001, XF + 0.014), (y0, y1 + 0.003), (z - ARM_W / 2 - 0.002, z + ARM_W / 2 + 0.002)))
    return body, boss


def _door_parts():
    slab_size, slab_c = _span_box((D_X0, D_X1), (D_OUT, D_IN), (D_Z0, D_Z1))
    parts = [("slab", m.box(slab_size, slab_c), "door")]
    cols = [m.col_box("slab_c", slab_size, slab_c)]
    stem_len = 0.021  # a 32 mm knob, 35 mm proud: a 13 mm stem, then a 14 mm thick head
    stem = m.cylinder(KNOB_STEM_R, stem_len, (KNOB[0], D_OUT - stem_len / 2, KNOB[2]), axis="y", segments=20)
    head = m.cylinder(KNOB_R, 0.014, (KNOB[0], D_OUT - stem_len - 0.007, KNOB[2]), axis="y", segments=28)
    parts += [("knob_stem", stem, "knob"), ("knob_head", head, "knob")]
    cols += [m.col_hull("knob_stem_c", stem), m.col_hull("knob_head_c", head)]
    for k, z in enumerate(HINGE_Z):
        parts.append((f"cup{k}", m.cylinder(0.0175, 0.0015, (CUP_X, D_IN + 0.0005, z), axis="y", segments=28), "cup"))
        # the hinge's body, from the cup round the side's front edge (it turns with the door; shut, it lies inside the side)
        parts.append((f"knuckle{k}", m.box(*_span_box((CUP_X - 0.006, XF - 0.0005), (D_IN, D_IN + 0.0025), (z - 0.009, z + 0.009))), "steel"))
        parts.append((f"link{k}", m.box(*_span_box((XF - 0.008, XF - 0.0005), (D_IN, Y_IN_R + 0.004), (z - 0.008, z + 0.008))), "steel"))
    return parts, cols


def _hinges() -> m.Part:
    frame_parts = _plate(HINGE_Z[0], "low") + _plate(HINGE_Z[1], "up")
    body, boss = _arm_mesh(*ARM_HOME_Y, HINGE_Z[1])  # the upper arm, still on its plate
    frame_parts += [("arm_up", body, "steel"), ("boss_up", boss, "steel")]
    sag = m.hinge("sag", "Y", SAG_AT, SAG_DEG, SAG_UP_DEG, (), (), mass=0.2, damping=0.5, friction=0.0, armature=0.01,
                  inertia=(1e-3, 1e-3, 1e-3))
    door_parts, door_cols = _door_parts()
    com = ((D_X0 + D_X1) / 2, (D_OUT + D_IN) / 2, (D_Z0 + D_Z1) / 2)
    inertia = (DOOR_MASS * (DT**2 + DH**2) / 12, DOOR_MASS * (DW**2 + DH**2) / 12, DOOR_MASS * (DW**2 + DT**2) / 12)
    door = m.hinge("door", "Z", (PIVOT[0], PIVOT[1], com[2]), CLOSED_DEG, 20.0, door_parts, door_cols, mass=DOOR_MASS, gravity=True,
                   com=com, inertia=inertia, spring=CLOSE_K, damping=0.3, friction=0.02, parent="sag")
    body, boss = _arm_mesh(*ARM_Y, HINGE_Z[0])
    arm_size, arm_c = _span_box(ARM_X, ARM_Y, (HINGE_Z[0] - ARM_W / 2, HINGE_Z[0] + ARM_W / 2))
    arm = m.slider("arm", "Y", -TRAVEL, 0.0, [("arm", body, "steel"), ("boss", boss, "steel")], [m.col_box("arm_c", arm_size, arm_c)],
                   at=arm_c, mass=0.08, spring=ARM_K, damping=2.0, friction=0.0, parent="sag")
    return m.mechanism("hinges", (0.0, 0.0, TOP), [sag, door, arm], GREYS, frame_parts=frame_parts, friction=(1.0, 0.9),
                       solver_iters=64, start=(("sag", math.radians(SAG_DEG)),))


OBJECTS = [
    _carcass(),
    _hinges(),
    # Distractors (SimReady props), loose on the bench clear of the cabinet, the door's swing and the arms' paths.
    SimReady("mug", pos=(0.55, 0.31, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=math.pi,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("sponge", pos=(0.62, -0.22, TOP), size=(0.071, 0.101, 0.033), mass=0.02, yaw=0.3,
             package="cleaning_sponge_b01", usd="sm_cleaning_sponge_b01_01.usd"),
    SimReady("screwdriver", pos=(0.43, -0.25, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=0.0, convex=True,
             package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd"),
]


# -- live rules: the click, the closing springs, and what the scorer needs -----------------------------------------------


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _box_gap(local: np.ndarray, lo, hi) -> np.ndarray:
    out = np.maximum(np.maximum(np.asarray(lo) - local, local - np.asarray(hi)), 0.0)
    return np.linalg.norm(out, axis=1)


HANDS_OFF = 0.03  # a gripper's fingers this far or more from the door, its knob and the lower hinge's arm
STILL_RAD_S = 0.034  # the door's free edge moving under 1 cm/s
HOLD_STEPS = round(SUCCESS_HOLD_S * 20)
DOOR_BOX = ((D_X0, D_OUT - 0.036, D_Z0), (D_X1, D_IN + 0.003, D_Z1))  # the slab with its knob and cups
ARM_BOX = ((XF, ARM_Y[0] - 0.001, HINGE_Z[0] - ARM_W / 2 - 0.002), (ARM_X[1], ARM_Y[1] + 0.003, HINGE_Z[0] + ARM_W / 2 + 0.002))


class Tracker:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.clipped = False
        self.last: dict = {}
        self.good = 0  # control steps in a row with every full-score condition met

    def step(self, env):
        st = m.state(env, "hinges")
        sag, q, press = st.joint("sag"), st.joint("door"), st.joint("arm")
        if not self.clipped and press <= -CLICK_DEPTH and abs(math.degrees(sag)) <= SQUARE_TOL_DEG:
            m.lock(env, "hinges", "sag")
            m.lock(env, "hinges", "arm")
            self.clipped = True
        open_deg = math.degrees(q) - CLOSED_DEG
        m.set_target(env, "hinges", "door", math.radians(CLOSED_DEG - CLOSE_PAST_DEG) if open_deg < SELF_CLOSE_DEG else q)
        base = env.unwrapped
        asset = base.scene["hinges"]
        origin = _np(base.scene.env_origins[0])
        poses = _np(asset.data.body_link_pose_w)[0]
        qd = _np(asset.data.joint_vel)[0]
        names = list(asset.body_names)
        tips = m.fingertips(env)
        gaps = []
        for body, (lo, hi) in (("door", DOOR_BOX), ("arm", ARM_BOX)):
            p = poses[names.index(body)]
            local = (tips - (p[:3] - origin)) @ m.quat_matrix(p[3:7])  # in the link's authored frame: z above the bench
            gaps.append(float(_box_gap(local, lo, hi).min()))
        q_vel = float(qd[list(asset.joint_names).index("door_joint")])
        self.last = {"sag_deg": math.degrees(sag), "open_deg": open_deg, "press_mm": -press * 1000, "clipped": self.clipped,
                     "clear_m": min(gaps), "q_vel": q_vel}
        self.good = self.good + 1 if self._full() else 0
        self.n = getattr(self, "n", 0) + 1
        if os.environ.get("RECLIP_DEBUG") and self.n % 5 == 0:  # DEBUG: remove
            print("[reclip]", self.n, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.last.items()}, "good", self.good, flush=True)

    def _full(self) -> bool:
        r = self.last
        return bool(r and r["clipped"] and r["open_deg"] <= SHUT_TOL_DEG and r["clear_m"] >= HANDS_OFF and abs(r["q_vel"]) < STILL_RAD_S)


TRACK = Tracker()
m.on_step(TRACK.step)
m.on_reset(TRACK.reset)


def score(objects) -> float:
    r = TRACK.last
    if not r or not r["clipped"]:
        return 0.0
    if TRACK.good >= HOLD_STEPS:
        return 1.0
    if r["clear_m"] >= HANDS_OFF and abs(r["q_vel"]) < STILL_RAD_S:
        return 0.5
    return 0.25


def success(objects) -> bool:
    return bool(TRACK._full())


# -- the scripted demo: an open-loop plan from the published layout ------------------------------------------------------


class Grip(float):
    """A waypoint's yaw that carries a whole TCP rotation (the trick demokit.py uses): the harness's ArmPlan passes it to
    grip_rotation, which is extended below to return it, so a demo can tilt the gripper."""

    rot: np.ndarray = np.eye(3)


def _install_grips() -> None:
    if getattr(_harness_demo.grip_rotation, "_takes_grips", False):
        return
    plain = _harness_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return plain(yaw) if rot is None else rot

    grip_rotation._takes_grips = True
    grip_rotation._plain = plain
    _harness_demo.grip_rotation = grip_rotation


_install_grips()


def _grip(rot: np.ndarray) -> Grip:
    g = Grip(0.0)
    g.rot = rot
    return g


def down(yaw_deg: float) -> Grip:
    return _grip(_harness_demo.grip_rotation._plain(math.radians(yaw_deg)))


def tilted(theta_deg: float, phi_deg: float) -> Grip:
    """Tilted theta_deg from straight down towards the direction phi_deg from +x, the fingers closing across it."""
    phi = math.radians(phi_deg)
    plain = _harness_demo.grip_rotation._plain(phi)
    return _grip(axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ plain)


def level(phi_deg: float) -> Grip:
    """Fingers level, pointing along phi_deg from +x, closing across it."""
    return tilted(90.0, phi_deg)


def _ahead(theta_deg: float, phi_deg: float) -> np.ndarray:
    """The direction the fingers point in for tilted(theta_deg, phi_deg)."""
    t, p = math.radians(theta_deg), math.radians(phi_deg)
    return np.array([math.cos(p) * math.sin(t), math.sin(p) * math.sin(t), -math.cos(t)])


def _rot_y(deg: float, p, about):
    a = math.radians(deg)
    d = np.asarray(p, float) - np.asarray(about, float)
    return np.asarray(about, float) + np.array([d[0] * math.cos(a) + d[2] * math.sin(a), d[1], -d[0] * math.sin(a) + d[2] * math.cos(a)])


def _rot_z(deg: float, p, about):
    a = math.radians(deg)
    d = np.asarray(p, float) - np.asarray(about, float)
    return np.asarray(about, float) + np.array([d[0] * math.cos(a) - d[1] * math.sin(a), d[0] * math.sin(a) + d[1] * math.cos(a), d[2]])


LIFT_TRIM = np.array([float(v) for v in os.environ.get("RECLIP_TRIM", "-0.0057,0,0.0083").split(",")])  # what the right arm's droop under the door's weight needs on top of the square pose
TILT_KNOB = 45.0  # the right hand reaches down onto the knob at 45 degrees, from outside the door
KNOB_TCP = np.array([0.0, -0.003, 0.003])  # TCP from the knob head's centre: 3 mm out, 3 mm up
PHI_PRESS = float(os.environ.get("RECLIP_PHI", "-55"))
PRESS_OVER = 0.006  # commanded past home


def _w(t, p, finger, label, grip):
    """A waypoint at p, given in metres above the bench."""
    return Waypoint(t, (float(p[0]), float(p[1]), float(p[2]) + TOP), float(finger), label, grip)


def demo():
    sag_at = np.array(SAG_AT)
    knob_sq = np.array(KNOB) + KNOB_TCP
    knob_sag = _rot_y(SAG_DEG, knob_sq, sag_at)
    pinch = FINGER_CLOSED  # commanded shut: the knob's 32 mm head stops the fingers
    tilt = TILT_KNOB
    g = tilted(tilt, 90.0)
    back = -0.06 * _ahead(tilt, 90.0)
    held = knob_sq + LIFT_TRIM
    right = [
        _w(2.5, knob_sag + back + [0, 0, 0.03], FINGER_OPEN, "out from the knob", g),
        _w(1.5, knob_sag + back * 0.3, FINGER_OPEN, "towards the knob", g),
        _w(1.0, knob_sag, FINGER_OPEN, "fingers either side of its head", g),
        _w(0.8, knob_sag, pinch, "pinch the knob", g),
        _w(2.0, held, pinch, "lift the free corner: door square", g),
        _w(6.0, held, pinch, "hold it square while the arm is pressed", g),
    ]
    pivot = np.array([PIVOT[0], PIVOT[1], 0.0])
    for deg in (-10, -20, -30, -40, -50, -58):
        right.append(_w(1.2, _rot_z(deg, held, pivot), pinch, "swing it towards shut", tilted(tilt, 90.0 + deg)))
    end = _rot_z(-58, held, pivot)
    g_end = tilted(tilt, 32.0)
    right += [
        _w(0.6, end, FINGER_OPEN, "let go: the hinge springs shut it", g_end),
        _w(1.0, end - 0.05 * _ahead(tilt, 32.0) + [0, 0, 0.02], FINGER_OPEN, "back off the knob", g_end),
        _w(2.0, (0.22, -0.22, 0.16), FINGER_OPEN, "away", down(60.0)),
    ]

    a = math.radians(PHI_PRESS)
    ahead = np.array([math.cos(a), math.sin(a), 0.0])
    z = HINGE_Z[0]
    touch = np.array([PLATE_X, ARM_Y[1] + 0.004, z]) - 0.013 * ahead  # fingertips 4 mm off the loose arm's inner face
    gl = level(PHI_PRESS)
    left = [
        _w(3.0, (0.30, 0.04, z + 0.03), FINGER_CLOSED, "in front of the opening", gl),
        _w(5.0, (0.30, 0.04, z + 0.03), FINGER_CLOSED, "wait for the door to be square", gl),
        _w(1.5, touch + [-0.02, 0.02, 0.0], FINGER_CLOSED, "into the cabinet", gl),
        _w(1.0, touch, FINGER_CLOSED, "fingertips at the hinge arm", gl),
        _w(1.5, touch - [0.0, TRAVEL + 0.004 + PRESS_OVER, 0.0], FINGER_CLOSED, "press it onto its plate", gl),
        _w(1.0, touch - [0.0, TRAVEL + 0.004 + PRESS_OVER, 0.0], FINGER_CLOSED, "hold the press", gl),
        _w(0.8, touch + [0.0, 0.02, 0.0], FINGER_CLOSED, "off the arm", gl),
        _w(1.5, (0.30, 0.06, z + 0.04), FINGER_CLOSED, "out of the cabinet", gl),
        _w(2.0, (0.25, 0.24, 0.18), FINGER_CLOSED, "back, clear of the door's swing", gl),
    ]
    return {"left": left, "right": right}

# ---- red-team variant: a different demo() ----
from yam_arena import ik as _ik, station as _station
from yam_arena.constants import READY as _READY

_FULL = demo


def _idle(side, t=30.0):
    pos, rot = _ik.tcp_pose(np.array(_READY, float), np.array(_station.BASES[side], float))
    return [Waypoint(t, tuple(float(v) for v in pos), FINGER_OPEN, "idle", _grip(rot))]


def _parts():
    d = _FULL()
    return d["right"], d["left"]
def demo():
    R, L = _parts()
    right = R[:12] + [Waypoint(12.0, R[11].pos, R[11].finger, "keep holding the knob", R[11].yaw)]
    return {"left": L, "right": right}
