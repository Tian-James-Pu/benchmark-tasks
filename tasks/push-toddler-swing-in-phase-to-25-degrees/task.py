"""A toddler's full-bucket swing hangs still in front of the arms, over the station table (the robot's own table, set
behind the swing as in a real trial). The swing hangs from a backyard A-frame swing set: a 2.3 m steel top beam whose
underside is 2.13 m (7 ft) above the floor, with the swing's two hangers 40 cm apart. Two green plastisol-coated chains
run 1.26 m from the hangers' quick links down to the underside of a green moulded full-bucket toddler seat (31 cm wide,
31 cm deep, 31 cm high at its back), which hangs 82 cm above the floor, 6 cm above the table top. Strapped into the seat
is a 15 kg canvas sandbag standing in for a three-year-old. The swing moves towards and away from the arms; the back
of the bucket, where it is pushed, faces the arms, 66.4 cm in front of their bases. On the table, clear of the swing
and the arms: a pair of sunglasses, a coffee cup and a banana.
"""

import math
import pathlib
import sys
from dataclasses import replace

import numpy as np

from yam_arena import demo as _demo
from yam_arena import station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475
FLOOR = station.FLOOR_Z  # the station's floor, -0.0125

# -- the camera: moved back and to the right so it sees the swing's arc and both arms (the station builds the
# matching stand from this rig, as in tasks/reset-shower-chair-short-leg) ------------------------------------------
CAMERA_POS = (-0.12, -0.62, 1.62)
CAMERA_TARGET = (0.62, 0.02, 1.02)
_base_rig = getattr(station.rig, "_swing_base", station.rig)


def _swing_rig(top_cam="calibrated"):
    rig = _base_rig(top_cam)
    forward = np.asarray(CAMERA_TARGET, float) - np.asarray(CAMERA_POS, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rot = np.column_stack((right, up, -forward))
    camera = replace(rig.top_cam, pos=CAMERA_POS, rot_xyzw=tuple(float(v) for v in station.matrix_to_quat_xyzw(rot)), convention="opengl")
    return replace(rig, name="station-toddler-swing", top_cam=camera)


_swing_rig._swing_base = _base_rig
station.rig = _swing_rig

# -- the swing ---------------------------------------------------------------------------------------------------
BEAM_UNDER = FLOOR + 2.13  # 7 ft swing beam
PIVOT_Z = BEAM_UNDER - 0.05  # the hangers' quick-link eyes, the chains' pivot
SEAT_UNDER = FLOOR + 0.82  # the bucket's underside at rest: 6 cm above the table top
X_PIVOT = 0.82  # the hangers, in front of the arms
HANGER_Y = 0.20  # hangers 40 cm apart
BUCKET = (0.311, 0.311, 0.311)  # depth (x), width (y), height of the back (z): 12.25 in
SIDE_H, FRONT_H = 0.22, 0.13  # the side walls' and the front's height above the underside
WALL = 0.02  # collider wall thickness (the moulded wall is about 5 mm; colliders keep its outer surface)
H_REL = SEAT_UNDER - PIVOT_Z  # the underside, below the pivot (-1.26)
BACK_X = BUCKET[0] / 2  # the back's outer face, behind the chain line
PUSH_UP = 0.17  # where the hands push the back: 17 cm above the underside
PUSH_Z = H_REL + PUSH_UP
SAND = (0.22, 0.26, 0.17)  # the sandbag: 15 kg of dry sand in a canvas bag
SAND_C = (0.0, 0.0, H_REL + 0.04 + SAND[2] / 2)
M_SAND, M_BUCKET, M_CHAINS = 15.0, 1.5, 0.7
CHAIN_R = 0.008  # plastisol-coated chain, 16 mm across
SWING_DAMP = 0.6  # N m s/rad at the hangers, as authored
DAMP_EFF = 0.27  # what the simulated joint gives, measured from free swings: 1.3 % of the amplitude lost per swing
LIMIT_DEG = 60.0

ATTACH = {s: ((-0.12, s * BUCKET[1] / 2, H_REL + 0.29), (0.12, s * BUCKET[1] / 2, H_REL + SIDE_H - 0.01)) for s in (1, -1)}
SPLIT = {s: (0.0, s * 0.165, H_REL + 0.45) for s in (1, -1)}
HANGER = {s: (0.0, s * HANGER_Y, -0.01) for s in (1, -1)}


def _mass_props():
    """Total mass, centre of mass and principal inertia about it (link frame), from the sandbag, bucket and chains."""
    items = []  # (mass, centre, own inertia diag)
    sx, sy, sz = SAND
    items.append((M_SAND, np.array(SAND_C), np.array([sy**2 + sz**2, sx**2 + sz**2, sx**2 + sy**2]) * M_SAND / 12))
    bc = np.array([0.0, 0.0, H_REL + 0.13])
    items.append((M_BUCKET, bc, np.full(3, M_BUCKET * 0.15**2 * 0.7)))
    for s in (1, -1):
        a, b = np.array(HANGER[s]), np.array(SPLIT[s])
        L = np.linalg.norm(b - a)
        items.append((M_CHAINS / 2, (a + b) / 2, np.array([1.0, 1.0, 0.0]) * (M_CHAINS / 2) * L**2 / 12))
    mass = sum(i[0] for i in items)
    com = sum(i[0] * i[1] for i in items) / mass
    inertia = np.zeros(3)
    for mi, c, own in items:
        d = c - com
        inertia += own + mi * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
    return mass, com, inertia


M_SWING, COM, INERTIA = _mass_props()
COM = np.array([0.0, 0.0, COM[2]])  # hangs straight down at joint zero
I_PIVOT = INERTIA[1] + M_SWING * COM[2] ** 2 + 0.001  # about the hangers' axis, plus the joint's armature
MGD = M_SWING * 9.81 * -COM[2]

LOOKS = {
    "seat": m.Look(m.rgb(0.13, 0.55, 0.22), roughness=0.45),
    "chain": m.Look(m.rgb(0.10, 0.45, 0.18), roughness=0.5),
    "sand": m.Look(m.rgb(0.70, 0.60, 0.42), roughness=0.9),
    "strap": m.Look(m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "steel": m.Look(m.rgb(0.25, 0.42, 0.62), metallic=0.4, roughness=0.4),
    "hanger": m.Look(m.rgb(0.55, 0.56, 0.58), metallic=0.8, roughness=0.3),
}


def _seat_parts():
    D, W, H = BUCKET
    hz = H_REL
    parts = [
        ("back", m.box((0.012, W, H), (-BACK_X + 0.006, 0.0, hz + H / 2)), "seat"),
        ("back_rim", m.cylinder(0.012, W, (-BACK_X + 0.006, 0.0, hz + H), axis="y"), "seat"),
        ("side_l", m.box((D, 0.012, SIDE_H), (0.0, W / 2 - 0.006, hz + SIDE_H / 2)), "seat"),
        ("side_r", m.box((D, 0.012, SIDE_H), (0.0, -W / 2 + 0.006, hz + SIDE_H / 2)), "seat"),
        ("side_l_top", m.box((0.16, 0.012, H - SIDE_H), (-BACK_X + 0.08, W / 2 - 0.006, hz + SIDE_H + (H - SIDE_H) / 2)), "seat"),
        ("side_r_top", m.box((0.16, 0.012, H - SIDE_H), (-BACK_X + 0.08, -W / 2 + 0.006, hz + SIDE_H + (H - SIDE_H) / 2)), "seat"),
        ("front", m.box((0.012, W, FRONT_H), (BACK_X - 0.006, 0.0, hz + FRONT_H / 2)), "seat"),
        ("floor", m.box((D, W, 0.04), (0.0, 0.0, hz + 0.02)), "seat"),
        ("pommel", m.box((0.06, 0.05, 0.10), (BACK_X - 0.05, 0.0, hz + 0.09)), "seat"),
        ("sandbag", m.box(SAND, SAND_C), "sand"),
        ("strap", m.box((SAND[0] + 0.01, 0.30, 0.03), (SAND_C[0], 0.0, SAND_C[2] + 0.03)), "strap"),
    ]
    for s, tag in ((1, "l"), (-1, "r")):
        parts.append((f"chain_{tag}", m.tube([HANGER[s], SPLIT[s]], CHAIN_R, sides=8), "chain"))
        for i, a in enumerate(ATTACH[s]):
            parts.append((f"bridle_{tag}{i}", m.tube([SPLIT[s], a], CHAIN_R * 0.8, sides=8), "chain"))
    return parts


def _seat_colliders():
    D, W, H = BUCKET
    hz = H_REL
    cols = [
        m.col_box("c_back", (WALL, W, H), (-BACK_X + WALL / 2, 0.0, hz + H / 2)),
        m.col_box("c_side_l", (D, WALL, SIDE_H), (0.0, W / 2 - WALL / 2, hz + SIDE_H / 2)),
        m.col_box("c_side_r", (D, WALL, SIDE_H), (0.0, -W / 2 + WALL / 2, hz + SIDE_H / 2)),
        m.col_box("c_front", (WALL, W, FRONT_H), (BACK_X - WALL / 2, 0.0, hz + FRONT_H / 2)),
        m.col_box("c_floor", (D, W, 0.04), (0.0, 0.0, hz + 0.02)),
        m.col_box("c_sand", SAND, SAND_C),
    ]
    for s, tag in ((1, "l"), (-1, "r")):
        cols.append(m.col_hull(f"c_chain_{tag}", m.tube([HANGER[s], SPLIT[s]], 0.01, sides=8)))
    return cols


# Boxes of the swing in its own frame (centre, half size), for the scorer's distances; chains as segments.
_BOXES = [
    ((-BACK_X + WALL / 2, 0.0, H_REL + BUCKET[2] / 2), (WALL / 2, BUCKET[1] / 2, BUCKET[2] / 2)),
    ((0.0, BUCKET[1] / 2 - WALL / 2, H_REL + SIDE_H / 2), (BUCKET[0] / 2, WALL / 2, SIDE_H / 2)),
    ((0.0, -BUCKET[1] / 2 + WALL / 2, H_REL + SIDE_H / 2), (BUCKET[0] / 2, WALL / 2, SIDE_H / 2)),
    ((BACK_X - WALL / 2, 0.0, H_REL + FRONT_H / 2), (WALL / 2, BUCKET[1] / 2, FRONT_H / 2)),
    ((0.0, 0.0, H_REL + 0.02), (BUCKET[0] / 2, BUCKET[1] / 2, 0.02)),
    (SAND_C, tuple(v / 2 for v in SAND)),
]
_SEGMENTS = [(HANGER[s], SPLIT[s], 0.01) for s in (1, -1)] + [(SPLIT[s], a, 0.008) for s in (1, -1) for a in ATTACH[s]]


def _frame_parts():
    beam_y = 1.15
    parts = [
        ("beam", m.box((0.09, 2 * beam_y, 0.14), (0.0, 0.0, 0.05 + 0.07)), "steel"),
    ]
    for s, tag in ((1, "l"), (-1, "r")):
        parts.append((f"hanger_{tag}", m.box((0.06, 0.035, 0.05), (0.0, s * HANGER_Y, 0.025)), "hanger"))
        for fx, ftag in ((1, "f"), (-1, "b")):
            top = (0.0, s * (beam_y - 0.05), 0.12)
            foot = (fx * 0.80, s * (beam_y + 0.12), FLOOR - PIVOT_Z)
            parts.append((f"leg_{tag}{ftag}", m.tube([top, foot], 0.03, sides=12), "steel"))
    return parts


SWING = m.mechanism(
    "swing",
    (X_PIVOT, 0.0, PIVOT_Z),
    [
        m.hinge("seat", "Y", (0.0, 0.0, 0.0), -LIMIT_DEG, LIMIT_DEG, _seat_parts(), _seat_colliders(),
                mass=float(M_SWING), com=tuple(float(v) for v in COM), inertia=tuple(float(v) for v in INERTIA),
                gravity=True, damping=SWING_DAMP, friction=0.0, armature=0.001, effort=50.0),
    ],
    LOOKS,
    frame_parts=_frame_parts(),
    solver_iters=32,
)

# -- distractors (SimReady, loose; clear of the swing and the arms) ------------------------------------------------
SUNGLASSES = SimReady("sunglasses", pos=(0.52, 0.47, TOP), size=(0.145, 0.174, 0.057), mass=0.03,
                      package="accessory_eyeglasses_a01", usd="sm_accessory_eyeglasses_a01_01.usd", yaw=1.2)
CUP = SimReady("coffee_cup", pos=(0.70, -0.46, TOP), size=(0.085, 0.085, 0.117), mass=0.25,
               package="food_beverage_coffeeCup_a01", usd="sm_food_beverage_coffeeCup_a01_01.usd")
BANANA = SimReady("banana", pos=(0.78, 0.44, TOP), size=(0.039, 0.197, 0.074), mass=0.15,
                  package="food_banana_a01", usd="sm_food_banana_a01_01.usda", yaw=0.4)

OBJECTS = [SWING, SUNGLASSES, CUP, BANANA]

# -- geometry helpers --------------------------------------------------------------------------------------------
PIVOT = np.array([X_PIVOT, 0.0, PIVOT_Z])


def ry(theta: float) -> np.ndarray:
    """Rotation about +y by theta: a positive swing angle moves the seat towards the arms (-x)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def panel_point(theta: float, y: float = 0.0) -> np.ndarray:
    return PIVOT + ry(theta) @ np.array([-BACK_X, y, PUSH_Z])


def panel_normal(theta: float) -> np.ndarray:
    """The back's outward normal (towards the arms when the seat hangs still)."""
    return ry(theta) @ np.array([-1.0, 0.0, 0.0])


R_BACK = float(math.hypot(BACK_X, PUSH_Z))  # the back's push point, from the pivot


def swing_gap(points: np.ndarray, theta: float) -> np.ndarray:
    """Distance from each world point to the swing's seat, sandbag and chains, at swing angle theta."""
    local = (np.asarray(points, float) - PIVOT) @ ry(theta)  # rows: R^T (p - pivot)
    best = np.full(len(local), np.inf)
    for c, h in _BOXES:
        d = np.maximum(np.abs(local - np.asarray(c)) - np.asarray(h), 0.0)
        best = np.minimum(best, np.linalg.norm(d, axis=1))
    for a, b, r in _SEGMENTS:
        a, b = np.asarray(a), np.asarray(b)
        ab = b - a
        t = np.clip(((local - a) @ ab) / (ab @ ab), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(local - (a + t[:, None] * ab), axis=1) - r)
    return best


def free_step(theta: float, omega: float, dt: float, torque: float = 0.0) -> tuple[float, float]:
    """The swing's own motion over dt (RK4): gravity, the hangers' damping and an applied torque."""
    def f(th, om):
        return om, (-MGD * math.sin(th) - DAMP_EFF * om + torque) / I_PIVOT

    k1 = f(theta, omega)
    k2 = f(theta + dt / 2 * k1[0], omega + dt / 2 * k1[1])
    k3 = f(theta + dt / 2 * k2[0], omega + dt / 2 * k2[1])
    k4 = f(theta + dt * k3[0], omega + dt * k3[1])
    return (theta + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
            omega + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]))


# -- scoring -----------------------------------------------------------------------------------------------------
DT = 0.05  # control step
FULL_DEG, HALF_DEG, QUARTER_DEG = 25.0, 20.0, 15.0
TOUCH_GAP = 0.010  # a fingertip point this close to the swing touches it
CLEAR_GAP = 0.05  # hands clear: every fingertip point at least this far from the swing
AWAY_SPEED = 0.05  # m/s at the back's push point: the seat is moving away from the arms
STILL_DEG = 2.0  # the swing hangs still: within this of vertical and slower than AWAY_SPEED
SHOVE_SPEED = 0.6  # m/s: a hand closing on the seat faster than this before it touches
TAU_TOUCH = 3.0  # N m: the swing departs from its own motion by more than this: something touches it


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.harm = None  # "strike" or "shove", latched for the whole trial
        self.t = 0.0
        self.prev = None  # (theta, omega) last step
        self.gaps = [np.inf, np.inf]  # the closest fingertip gap, two steps back and one step back
        self.touching_prev = False
        self.last_unclear = 0.0  # the last time a hand touched or came within CLEAR_GAP
        self.last_rear_t = None
        self.front_since_rear = 0.0  # the furthest forward the seat has gone since its last rear turning point, degrees
        self.best = 0.0  # the highest free swing so far, degrees
        self.full = False
        self.score = 0.0

    def step(self, env):
        self.t += DT
        asset = env.unwrapped.scene["swing"]
        theta = float(m._np(asset.data.joint_pos)[0, 0])
        omega = float(m._np(asset.data.joint_vel)[0, 0])
        tips = m.fingertips(env)
        gap = float(swing_gap(tips, theta).min())
        tau = 0.0
        if self.prev is not None:
            _, om_pred = free_step(self.prev[0], self.prev[1], DT)
            tau = I_PIVOT * (omega - om_pred) / DT
        touching = gap < TOUCH_GAP or abs(tau) > TAU_TOUCH
        away = -omega * R_BACK  # the back's speed away from the arms
        still = abs(math.degrees(theta)) < STILL_DEG and abs(away) < AWAY_SPEED
        if self.harm is None and touching and not (away >= AWAY_SPEED or still):
            self.harm = "strike"
        if self.harm is None and gap < TOUCH_GAP and self.gaps[1] >= TOUCH_GAP and np.isfinite(self.gaps[0]):
            closing = (self.gaps[0] - self.gaps[1]) / DT
            if closing > SHOVE_SPEED:
                self.harm = "shove"
        if touching or gap < CLEAR_GAP:
            self.last_unclear = self.t
        # peaks: theta > 0 is towards the arms (the rear of the arc)
        deg = math.degrees(theta)
        if self.prev is not None:
            if self.prev[1] > 0.0 >= omega:  # a rear peak
                rear = max(math.degrees(self.prev[0]), deg)
                if self.last_rear_t is not None and self.last_unclear < self.last_rear_t:
                    self.best = max(self.best, min(rear, self.front_since_rear))
                self.last_rear_t = self.t
                self.front_since_rear = 0.0
            self.front_since_rear = max(self.front_since_rear, -deg)
        self.prev = (theta, omega)
        self.gaps = [self.gaps[1], gap]
        self.full = self.full or (self.harm is None and self.best >= FULL_DEG)
        if self.harm is not None:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif self.best >= HALF_DEG:
            self.score = 0.5
        elif self.best >= QUARTER_DEG:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)
SUCCESS_HOLD_S = 1.0


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full and TRIAL.harm is None)


# -- the scripted demo -------------------------------------------------------------------------------------------
class Grip(float):
    """A waypoint yaw that carries a whole TCP rotation (as demokit's grips do), for pushes with the fingers level."""

    rot: np.ndarray = np.eye(3)


if not getattr(_demo.grip_rotation, "_takes_grips", False):
    _plain_rotation = _demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return _plain_rotation(yaw) if rot is None else rot

    _grip_rotation._takes_grips = True
    _grip_rotation._plain = _plain_rotation
    _demo.grip_rotation = _grip_rotation
_plain = _demo.grip_rotation._plain


def tilted(theta_deg: float, phi_deg: float = 0.0) -> Grip:
    """Tilted theta from straight down towards phi from +x, the fingers closing across that direction (demokit's)."""
    phi = math.radians(phi_deg)
    g = Grip(0.0)
    g.rot = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain(phi)
    return g


HAND_Y = 0.07  # each hand's push point, either side of the back's centre
PARK = (np.array([0.30, 0.0, 1.00]), 34.0, 0.33)  # parked hands: tip point (x, z), grip pitch, y either side
TIP = 0.013  # the finger pads' tip points reach this far past the TCP
WAIT_GAP = 0.03  # where a hand's tip waits behind the back's turning point
APPROACH = 3.0  # s: from READY to just behind the still seat
REACH_X = 0.66  # the hands' tips go no further forward than this while pushing a moving seat
PLAN = dict(k=300.0, fmax=25.0, lag=0.09, ff=0.10, lead=0.07, max_lead=0.10, ramp=0.30, push=0.42, stop=0.25, target_deg=26.5,
            setup_deg=21.5, first_lead=0.05, first_push=0.45)
# The swing's state measured in the simulator (s: swing angle in degrees, rad/s) once the hands have let go after
# a push; the plan resumes from it, since the model's push is only approximate.
ANCHORS = {3.80: (-5.016, 0.0285), 5.50: (-8.763, -0.2348), 7.45: (-16.020, -0.3746)}


def _mj(a, b, s):
    s = min(max(s, 0.0), 1.0)
    return a + (b - a) * (10 * s**3 - 15 * s**4 + 6 * s**5)


def _free_path(theta: float, omega: float, span: float, h: float = 0.005):
    """The swing's free motion for `span` seconds, sampled every h: a function of time from now -> angle."""
    th = [theta]
    a, w = theta, omega
    for _ in range(int(span / h) + 2):
        a, w = free_step(a, w, h)
        th.append(a)
    th = np.array(th)
    return lambda tau: float(np.interp(tau, np.arange(len(th)) * h, th))


class _Model:
    """The planner's swing (pendulum, damped) pushed by two springy hands that trail their commands by LAG."""

    def __init__(self, params, anchors=None):
        self.p = params
        self.anchors = dict(ANCHORS if anchors is None else anchors)
        self.th, self.om, self.t = 0.0, 0.0, APPROACH
        self.act = panel_point(0.0) + panel_normal(0.0) * (WAIT_GAP * 0.5)
        self.steps, self.gaps, self.peaks, self.strikes, self.decisions = [], [], [], [], []

    def copy(self):
        c = _Model(self.p, anchors={})  # a trial push is judged on the model alone
        c.th, c.om, c.t, c.act = self.th, self.om, self.t, self.act.copy()
        c.steps, c.gaps, c.peaks, c.strikes = list(self.steps), list(self.gaps), list(self.peaks), list(self.strikes)
        c.decisions = list(self.decisions)
        return c

    def advance(self, cmd_fn, until, pushing=False):
        k, fmax = self.p["k"], self.p["fmax"]
        while self.t < until - 1e-9:
            t_next = round(self.t + DT, 6)
            cmd, pitch, yoff = cmd_fn(t_next)
            cmd = np.asarray(cmd, float)
            self.steps.append((t_next, cmd, float(pitch), float(yoff)))
            self.act = self.act + (cmd - self.act) * (1.0 - math.exp(-DT / self.p["lag"]))
            for _ in range(10):
                pen = -float((self.act - panel_point(self.th)) @ panel_normal(self.th)) if yoff < 0.12 else -1.0
                force = min(fmax * 2, k * 2 * pen) if pen > 0 else 0.0
                if force > 0 and self.om * R_BACK > -AWAY_SPEED and abs(math.degrees(self.th)) >= STILL_DEG:
                    self.strikes.append(round(t_next, 2))
                self.th, self.om = free_step(self.th, self.om, DT / 10, PUSH_Z * force)
            self.t = t_next
            if t_next in self.anchors:
                self.th, self.om = math.radians(self.anchors[t_next][0]), self.anchors[t_next][1]
            if not pushing:
                pts = np.array([self.act + np.array([0.0, s * yoff, 0.0]) for s in (1, -1)])
                self.gaps.append((float(swing_gap(pts, self.th).min()), t_next))

    def next_rear_peak(self):
        self.decisions.append(round(self.t, 2))
        a, w, tt = self.th, self.om, self.t
        while tt - self.t < 5.0:
            a2, w2 = free_step(a, w, 0.002)
            tt += 0.002
            if w > 0.0 >= w2 and a2 > 0.0:
                return tt, max(a, a2)
            a, w = a2, w2
        return None

    def push_from_rest(self):
        t0, lead, d = self.t, self.p["first_lead"], self.p["first_push"]
        self.advance(lambda tt: (panel_point(0.0) + panel_normal(0.0) * _mj(WAIT_GAP * 0.5, -lead, (tt - t0) / d), 0.0, HAND_Y), t0 + d, True)
        end, t1 = self.steps[-1][1], self.t
        out = end + panel_normal(0.0) * 0.10
        self.advance(lambda tt: (_mj(end, out, (tt - t1) / 0.5), 0.0, HAND_Y), t1 + 0.5)

    def push_after(self, tp, thp, lead):
        """Wait behind the back's turning point at tp, then follow the seat out, closing to `lead` past its back."""
        wait = panel_point(thp) + panel_normal(thp) * WAIT_GAP
        here, tm0, tm1 = self.steps[-1][1], self.t, tp - 0.35
        if tm1 - tm0 < 0.3:
            raise RuntimeError("no time to reach the waiting point")
        self.advance(lambda tt: (_mj(here, wait, (tt - tm0) / (tm1 - tm0)), 0.0, HAND_Y), tm1)
        self.advance(lambda tt: (wait, 0.0, HAND_Y), tp)
        ramp, dpush, dstop = self.p["ramp"], self.p["push"], self.p["stop"]
        path = _free_path(thp, 0.0, dpush + dstop + 1.0)

        def follow(tt):
            tau = max(tt - tp, 0.0)
            a = path(tau)
            return panel_point(a) + panel_normal(a) * _mj(WAIT_GAP, -lead, tau / ramp)

        # end the push early enough that the hands, stopping, stay within reach (tip x at most REACH_X)
        tau = DT
        while tau < dpush - 1e-9:
            nxt = follow(tp + tau + DT)
            v = (nxt - follow(tp + tau)) / DT
            if nxt[0] + max(v[0], 0.0) * dstop / 2 > REACH_X:
                break
            tau += DT
        ff = self.p["ff"]  # lead the arm by its lag, so its hands keep up with the seat

        def follow_cmd(tt):
            return follow(tt) + ff * (follow(tt) - follow(tt - DT)) / DT

        self.advance(lambda tt: (follow_cmd(tt), 0.0, HAND_Y), tp + tau, True)
        ts = self.t
        p_end, v_end = follow(ts), (follow(ts) - follow(ts - DT)) / DT

        def stop_cmd(tt):
            u = min(tt - ts, dstop)
            return p_end + v_end * (u - u * u / (2 * dstop)) + ff * v_end * (1.0 - u / dstop)

        self.advance(lambda tt: (stop_cmd(tt), 0.0, HAND_Y), ts + dstop)
        # draw back 6 cm while the seat runs on; the next wait is decided after this, from where the seat is then
        here, tr = self.steps[-1][1], self.t
        self.advance(lambda tt: (_mj(here, here + panel_normal(thp) * 0.06, (tt - tr) / 0.3), 0.0, HAND_Y), tr + 0.3)

    def park(self, wait_s=9.0):
        here, tq = self.steps[-1][1], self.t
        tip, pitch, yoff = PARK
        self.advance(lambda tt: (_mj(here, tip, (tt - tq) / 1.0), _mj(0.0, pitch, (tt - tq) / 1.0), _mj(HAND_Y, yoff, (tt - tq) / 0.6)), tq + 1.0)
        self.advance(lambda tt: (tip, pitch, yoff), tq + wait_s)


def _lead_for(mdl, tp, thp, goal_deg, hi):
    """The push lead, up to hi, that the model says brings the next rear turning point to goal_deg."""
    lo = 0.0
    for _ in range(14):
        mid = (lo + hi) / 2
        trial = mdl.copy()
        trial.push_after(tp, thp, mid)
        if math.degrees(trial.next_rear_peak()[1]) < goal_deg:
            lo = mid
        else:
            hi = mid
    return hi


def _outcome(mdl, tp, thp, lead):
    trial = mdl.copy()
    trial.push_after(tp, thp, lead)
    return math.degrees(trial.next_rear_peak()[1])


def _plan(params=PLAN):
    """The demo, planned on _Model from the still swing: one push from rest, then one push just after each rear turning
    point, never past `setup_deg` until the last one, which is sized so the swing goes on to about `target_deg`."""
    mdl = _Model(params)
    mdl.push_from_rest()
    for _ in range(14):
        tp, thp = mdl.next_rear_peak()
        mdl.peaks.append((round(tp, 3), round(math.degrees(thp), 2)))
        if math.degrees(thp) >= params["target_deg"] - 0.5:
            break
        if _outcome(mdl, tp, thp, params["max_lead"]) >= params["target_deg"]:
            mdl.push_after(tp, thp, _lead_for(mdl, tp, thp, params["target_deg"], params["max_lead"]))
            tp, thp = mdl.next_rear_peak()
            mdl.peaks.append((round(tp, 3), round(math.degrees(thp), 2)))
            break
        lead = params["lead"]
        if _outcome(mdl, tp, thp, lead) > params["setup_deg"]:
            lead = _lead_for(mdl, tp, thp, params["setup_deg"], lead)
        mdl.push_after(tp, thp, lead)
    mdl.park()
    return mdl


def _pose(centre, pitch, yoff, side):
    grip = tilted(90.0 - pitch, 0.0)
    sy = 1.0 if side == "left" else -1.0
    return tuple(float(v) for v in centre + np.array([0.0, sy * yoff, 0.0]) - TIP * grip.rot[:, 2]), grip


def demo():
    mdl = _plan()
    start = panel_point(0.0) + panel_normal(0.0) * (WAIT_GAP * 0.5)
    plans = {}
    for side in ("left", "right"):
        p0, g0 = _pose(start + panel_normal(0.0) * 0.10, 0.0, HAND_Y, side)
        p1, g1 = _pose(start, 0.0, HAND_Y, side)
        wps = [Waypoint(APPROACH - 0.8, p0, FINGER_CLOSED, "close in behind the still seat", g0),
               Waypoint(0.8, p1, FINGER_CLOSED, "hands on the back", g1)]
        for t, centre, pitch, yoff in mdl.steps:
            p, g = _pose(centre, pitch, yoff, side)
            wps.append(Waypoint(DT, p, FINGER_CLOSED, "push", g))
        plans[side] = wps
    return plans
