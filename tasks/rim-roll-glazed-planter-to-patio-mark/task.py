"""Roll a filled 14-inch glazed ceramic planter on the edge of its foot across a patio, onto a taped spot.

The planter is an At Home "Rolled White Ceramic Outdoor Planter, Large" at full size (14.0 in across its rolled lip, 12.0 in tall,
22.2 lb empty), filled with potting mix to 5 cm below the rim. It stands on two 60 x 60 x 2 cm porcelain patio pavers laid across
the table. Its spot is a square of blue painter's tape on the right-hand paver, 24 cm to the right of where it stands, with a short
tape cross at its centre. Filled, the real pot weighs about 18.5 kg: too heavy to carry comfortably, and paving guides say to lift
planters rather than drag them across the slabs. The way across without lifting or dragging: tip it a little onto the edge of its
foot, roll it along that edge, and stand it back up.

Model (all contact, nothing triggered by proximity):
- Planter ("planter", one free rigid body): the glazed body up to the soil surface is 36 convex wedges of its lathe profile; the
  rolled lip and the wall above the soil are 32 segments, each two convex hulls, so the 3.2 cm roll can be pinched and the open top
  above the soil stays open. Foot 20 cm across with a rounded heel. Mass 18.5 kg, the real pot filled (10.1 kg empty plus about
  15 L of moist potting mix: README, "Scale and mass"), centre of mass 15 cm up the axis: balance point 37 degrees.
- Pavers ("patio", fixed): one flush collider for both pavers and their filled 5 mm joint. Friction 0.7 static, 0.6 dynamic.
- Distractors: SimReady sunglasses and a mug on the far corners of the pavers, and a bottle of water on the table beyond them; the
  scorer ignores them.
- Heading stripe: 24 mm blue tape across the top of the rim, visual only, for reading the planter's turn from above.

Scoring (read every control step by `Trial.step`; `score()` returns its level):
- 0.0 for the rest of the run if the planter ever tilts more than 32 degrees (5 degrees short of its balance point), if its foot ever
  leaves the pavers by more than 1 cm (lifted), or if it has skidded more than 6 cm in all (dragged). Skid is read only at rests
  (upright within 2 degrees and still), from the rim's centre and a tape stripe on the rim, as a person reads them off the overhead
  video: travel during a rest counts in full; a move between rests counts its travel less 11 cm times the stripe's turn (radians),
  not below 0, the turn credited only if the planter tipped at least 10 degrees during it (`Trial`). Cleared only at a reset.
- 1.0: resting (axis within 2 degrees of vertical, foot centre within 3 mm of the paver tops, still), every fingertip point at least
  5 cm from it, its foot centre within 3 cm of the tape cross, held 1 s.
- 0.5: standing as above with hands clear, foot centre within 8 cm of the cross.
- 0.25: standing as above with hands clear, foot centre at least 10 cm closer to the cross than at the start (24 cm away).
- 0.0 otherwise.

    python demokit.py task.py OUT --mode full
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
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Move the planter onto the taped square on the patio."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
TOP = TABLE_TOP[SCENE]

# -- the patio: two 60 x 60 x 2 cm porcelain pavers with a 5 mm joint ------------------------------------------------------------
PAVER = 0.60
PAVER_T = 0.02
JOINT = 0.005
PAVER_X = 0.46  # paver centres, metres in front of the arm bases (near edges at 0.16)
PAVER_YS = ((PAVER + JOINT) / 2, -(PAVER + JOINT) / 2)
FLOOR = TOP + PAVER_T  # the pavers' top: the planter stands on it

# -- the planter: At Home "Rolled White Ceramic Outdoor Planter, Large", 14.0 x 14.0 x 12.0 in, 22.2 lb empty ---------------------
H = 0.305  # 12.0 in tall
R_LIP = 0.178  # 14.0 in across the rolled lip
R_LIP_IN = 0.146  # the opening inside the roll: 29.2 cm
R_WALL_IN = 0.149  # the inside of the wall below the roll
SOIL = 0.255  # potting mix 5 cm below the rim
LIP_Z0 = 0.274  # the roll's underside
MASS = 18.5  # the real pot filled: 10.1 kg empty plus 8.4 kg of moist potting mix (README, "Scale and mass")
COM_Z = 0.15  # centre of mass up the axis: shell 15.5 cm, soil 14.3 cm (README)
FRICTION = (0.7, 0.6)  # unglazed foot on porcelain paver
SEG = 72
START = (0.48, 0.12)  # the foot's centre at the start, world x, y
MARK = (0.48, -0.12)  # the tape cross: 24 cm to the right
TRAVEL = float(math.hypot(START[0] - MARK[0], START[1] - MARK[1]))

# outer profile (radius, height) of the glazed body, from the foot up to just under the roll
BODY = [(0.100, 0.0), (0.106, 0.0015), (0.111, 0.0045), (0.116, 0.009), (0.122, 0.015), (0.130, 0.025), (0.138, 0.037), (0.145, 0.052),
        (0.151, 0.070), (0.156, 0.090), (0.160, 0.115), (0.1625, 0.145), (0.164, 0.180), (0.1645, 0.210), (0.164, 0.240), (0.163, 0.262),
        (0.162, 0.276)]
# the rolled lip's section (radius, height), closed
ROLL = [(0.1490, LIP_Z0), (0.1625, LIP_Z0), (0.1690, 0.2765), (0.1750, 0.2815), (0.1780, 0.2890), (0.1772, 0.2975), (0.1725, 0.3030),
        (0.1650, H), (0.1530, H), (0.1475, 0.3015), (R_LIP_IN, 0.2950), (0.1470, 0.2830)]
FOOT = BODY[:6]  # the heel the planter tips and rolls on


def _r_at(z: float) -> float:
    """The body's outer radius at height z."""
    for (r0, z0), (r1, z1) in zip(BODY[:-1], BODY[1:]):
        if z0 <= z <= z1:
            return r0 + (r1 - r0) * (z - z0) / (z1 - z0)
    return BODY[-1][0]


LOOKS = {
    "glaze": m.Look(color=m.rgb(0.90, 0.89, 0.84), roughness=0.22),
    "lip": m.Look(color=m.rgb(0.90, 0.89, 0.84), roughness=0.22),  # the rolled lip is glazed white like the body
    "soil": m.Look(color=m.rgb(0.20, 0.14, 0.10), roughness=0.98),
    "paver": m.Look(color=m.rgb(0.62, 0.61, 0.58), roughness=0.85),
    "joint": m.Look(color=m.rgb(0.36, 0.34, 0.31), roughness=0.95),
    "tape": m.Look(color=m.rgb(0.20, 0.45, 0.85), roughness=0.7),
}


def _sweep(section, a0: float, a1: float) -> m.Mesh:
    """A section (radius, height), convex and closed, extruded straight from bearing a0 to a1: one convex piece of a ring."""
    k = len(section)
    v = np.array([(r * math.cos(a), r * math.sin(a), z) for a in (a0, a1) for r, z in section])
    f = [(0, i + 1, i) for i in range(1, k - 1)] + [(k, k + i, k + i + 1) for i in range(1, k - 1)]
    for i in range(k):
        j = (i + 1) % k
        f += [(i, j, k + j), (i, k + j, k + i)]
    return m.Mesh(v, np.array(f))


def _planter() -> m.Part:
    body = m.lathe([(0.0, 0.0)] + BODY + [(R_WALL_IN, BODY[-1][1]), (R_WALL_IN, SOIL), (0.0, SOIL)], SEG)
    roll = m.lathe(ROLL + [ROLL[0]], SEG)
    soil = m.cylinder(R_WALL_IN - 0.0005, 0.006, (0.0, 0.0, SOIL - 0.0025), segments=SEG)
    # the body up to the soil as 36 wedges of 10 degrees, each a convex hull of 53 points (PhysX keeps at most 64 points of a hull)
    prof = [(r, z) for r, z in BODY if z <= SOIL] + [(_r_at(SOIL), SOIL)]
    cols = []
    for i in range(36):
        angs = [math.radians(10 * i + d) for d in (0.0, 5.0, 10.0)]
        pts = [(0.0, 0.0, 0.0), (0.0, 0.0, SOIL)] + [(r * math.cos(a), r * math.sin(a), z) for a in angs for r, z in prof]
        cols.append(m.col_hull(f"c_body{i:02d}", m.Mesh(np.array(pts), np.array([(0, 1, 2)]))))
    wall = [(R_WALL_IN, SOIL - 0.004), (_r_at(SOIL) - 0.0003, SOIL - 0.004), (0.1622, LIP_Z0 + 0.004), (R_WALL_IN, LIP_Z0 + 0.004)]
    n = 32
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        for tag, sec in (("w", wall), ("r", ROLL)):
            cols.append(m.col_hull(f"c_{tag}{i:02d}", _sweep(sec, a0, a1)))
    # the heading stripe: 24 mm blue tape, 3 cm long, radial across the flat top of the roll at the bearing that starts on the left (+y)
    stripe = m.box((0.024, 0.030, 0.0006), (0.0, 0.159, H + 0.0003))
    parts = [("body", body, "glaze"), ("roll", roll, "lip"), ("soil", soil, "soil"), ("stripe", stripe, "tape")]
    return m.free("planter", (START[0], START[1], FLOOR), parts, cols, LOOKS, mass=MASS, friction=FRICTION, com=(0.0, 0.0, COM_Z))


def _patio() -> m.Part:
    parts = []
    for i, y in enumerate(PAVER_YS):
        parts.append((f"paver{i}", m.box((PAVER, PAVER, PAVER_T - 0.001), (PAVER_X, y, (PAVER_T - 0.001) / 2)), "paver"))
    parts.append(("joint", m.box((PAVER, JOINT, PAVER_T - 0.003), (PAVER_X, 0.0, (PAVER_T - 0.003) / 2)), "joint"))
    cols = [m.col_box("c_pavers", (PAVER, 2 * PAVER + JOINT, PAVER_T), (PAVER_X, 0.0, PAVER_T / 2))]
    # the spot: 24 mm blue painter's tape outside a 40 cm square, and a cross 8 cm across at its centre
    mx, my, tw, half, tz = MARK[0], MARK[1], 0.024, 0.20, PAVER_T + 0.0003
    strips = ((mx - half - tw / 2, my, tw, 2 * half + 2 * tw), (mx + half + tw / 2, my, tw, 2 * half + 2 * tw),
              (mx, my - half - tw / 2, 2 * half, tw), (mx, my + half + tw / 2, 2 * half, tw), (mx, my, 0.08, tw), (mx, my, tw, 0.08))
    for k, (cx, cy, sx, sy) in enumerate(strips):
        parts.append((f"tape{k}", m.box((sx, sy, 0.0006), (cx, cy, tz)), "tape"))
    return m.fixed("patio", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=FRICTION)


PLANTER = _planter()
PATIO = _patio()
DISTRACTORS = [
    SimReady("sunglasses", pos=(0.68, 0.48, FLOOR), size=(0.145, 0.174, 0.057), mass=0.03, yaw=1.9,
             package="accessory_eyeglasses_a01", usd="sm_accessory_eyeglasses_a01_01.usd"),
    SimReady("mug", pos=(0.68, -0.48, FLOOR), size=(0.114, 0.090, 0.080), mass=0.35, yaw=2.2,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("water_bottle", pos=(0.83, -0.34, TOP), size=(0.068, 0.068, 0.245), mass=0.55,
             package="food_beverage_bottledWater_b01", usd="sm_food_beverage_bottledWater_b01_01.usd"),
]
OBJECTS = [PATIO, PLANTER, *DISTRACTORS]

# -- scoring -----------------------------------------------------------------------------------------------------------------------
CONTROL_HZ = 20
FULL_R = 0.03  # foot centre within 3 cm of the cross for full credit
HALF_R = 0.08  # within 8 cm for half
QUARTER_GAIN = 0.10  # at least 10 cm closer to the cross than at the start for a quarter
UPRIGHT_DEG = 2.0  # standing: axis within 2 degrees of vertical
TOPPLE_DEG = 32.0  # 5 degrees short of the 37-degree balance point
LIFT = 0.010  # the foot's lowest point more than 1 cm above the pavers: lifted
# dragging, read only at rest checkpoints (upright and still), from the overhead video: the rim's centre and a tape stripe on the rim
SKID_LIMIT = 0.06  # at most 6 cm of skid in all (the trip is 24 cm)
R_HEEL = 0.11  # a move's turn is worth this much travel per radian: the radius of the rounded heel it rolls on
ROLL_MIN_DEG = 10.0  # a move earns that allowance only if the planter tipped at least this far during it
SEATED = 0.003  # standing: the foot centre within 3 mm of the paver tops
STILL_V, STILL_W = 0.01, math.radians(3.0)  # m/s and rad/s
CLEAR = 0.05  # every fingertip point at least 5 cm from the planter
HOLD_STEPS = 20  # 1 s at 20 Hz

# the planter's outline in (radius, height), its open top counted as closed at the rim: the hands' gap is measured to it
OUTLINE = np.array([(0.0, 0.0)] + BODY + [(0.1690, 0.2765), (0.1750, 0.2815), (0.1780, 0.2890), (0.1772, 0.2975), (0.1725, 0.3030),
                                         (0.1650, H), (0.0, H)])
# foot points: the heel's rings at 144 bearings, in the planter's frame
_BEAR = np.linspace(0.0, 2 * math.pi, 144, endpoint=False)
FOOT_PTS = np.array([(r * math.cos(a), r * math.sin(a), z) for r, z in FOOT for a in _BEAR])
HEEL_PTS = np.array([(FOOT[1][0] * math.cos(a), FOOT[1][0] * math.sin(a), FOOT[1][1]) for a in _BEAR])


def _rot_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    v = np.cross(a, b)
    s, c = float(np.linalg.norm(v)), float(a @ b)
    if s < 1e-9:
        return np.eye(3)
    k = v / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    ang = math.atan2(s, c)
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def heading(rot: np.ndarray) -> float:
    """The planter's turn about its own axis (radians), tilt taken out: the bearing of a radial tape stripe on its rim, seen from above."""
    M = _rot_between(rot[:, 2], np.array([0.0, 0.0, 1.0])) @ rot
    return math.atan2(M[1, 0], M[0, 0])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def outline_gap(local: np.ndarray) -> np.ndarray:
    """Distance (m) from points in the planter's frame to its outline (0 inside it or inside its open top)."""
    q = np.c_[np.hypot(local[:, 0], local[:, 1]), local[:, 2]]
    d = np.full(len(q), np.inf)
    poly = np.vstack([OUTLINE, OUTLINE[:1]])
    for a, b in zip(poly[:-1], poly[1:]):
        d = np.minimum(d, _seg_dist(q, a, b))
    inside = np.zeros(len(q), bool)
    for a, b in zip(poly[:-1], poly[1:]):  # even-odd rule
        cross = (a[1] > q[:, 1]) != (b[1] > q[:, 1])
        xint = a[0] + (q[:, 1] - a[1]) * (b[0] - a[0]) / ((b[1] - a[1]) if b[1] != a[1] else 1e-12)
        inside ^= cross & (q[:, 0] < xint)
    return np.where(inside, 0.0, d)


class Trial:
    """Per-step record. The harm record (tipped, lifted, dragged) is cleared only in `reset`, the mechkit hook run after every reset.

    Dragging is read only where a person can read it from the overhead video: at rest checkpoints, when the planter stands upright
    (axis within 2 degrees) and still. At the first and last frame of each rest period the checker notes the rim's centre and the
    bearing of a radial tape stripe on the rim. Any travel of the rim's centre within a rest period is skid in full (an upright pot
    can't roll). For each move between rest periods, the skid is the rim centre's travel less R_HEEL times the stripe's turn (the
    smaller way round), not below 0; the turn counts only if the planter tipped at least ROLL_MIN_DEG during the move."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.max_tilt = 0.0
        self.tipped = False
        self.lifted = False
        self.dragged = False
        self.skid_done = 0.0  # committed skid (m)
        self.skid_open = 0.0  # travel within the rest period now running (m)
        self.skid = 0.0
        self.resting = False
        self.rest_start = None  # (rim centre xy, heading) at the first frame of the current or last rest period
        self.rest_last = None  # and at its last frame
        self.move_tilt = 0.0  # the most tilt since the last rest period ended
        self.hold = 0
        self.full = False
        self.level = 0.0
        self.d = TRAVEL
        self.gap = np.inf

    def step(self, env):
        s = m.state(env, "planter")
        pos, rot = np.asarray(s.pos, float), s.rotation()
        tilt = s.tilt_deg()
        self.max_tilt = max(self.max_tilt, tilt)
        if tilt > TOPPLE_DEG:
            self.tipped = True
        foot = pos + FOOT_PTS @ rot.T
        low = int(np.argmin(foot[:, 2]))
        if foot[low, 2] - FLOOR > LIFT:
            self.lifted = True
        still = float(np.linalg.norm(s.lin_vel)) < STILL_V and float(np.linalg.norm(s.ang_vel)) < STILL_W
        rest = tilt <= UPRIGHT_DEG and abs(float(pos[2]) - FLOOR) <= SEATED and still
        # dragging, read at rest checkpoints only (see the class docstring)
        here = ((pos + rot @ np.array([0.0, 0.0, H]))[:2], heading(rot))
        if rest:
            if not self.resting:  # a rest period begins: score the move since the last one
                if self.rest_last is not None:
                    d = float(np.linalg.norm(here[0] - self.rest_last[0]))
                    turn = abs(_wrap(here[1] - self.rest_last[1])) if self.move_tilt >= ROLL_MIN_DEG else 0.0
                    self.skid_done += max(0.0, d - R_HEEL * turn)
                self.rest_start = here
            self.rest_last = here
            self.skid_open = float(np.linalg.norm(here[0] - self.rest_start[0]))
        else:
            if self.resting:  # a rest period ends: its travel is skid
                self.skid_done += self.skid_open
                self.skid_open = 0.0
                self.move_tilt = 0.0
            self.move_tilt = max(self.move_tilt, tilt)
        self.resting = rest
        self.skid = self.skid_done + self.skid_open
        if self.skid > SKID_LIMIT:
            self.dragged = True
        self.gap = float(np.min(outline_gap(s.to_local(m.fingertips(env)))))
        standing = rest and self.gap >= CLEAR
        self.d = float(np.hypot(pos[0] - MARK[0], pos[1] - MARK[1]))
        harm = self.tipped or self.lifted or self.dragged
        ok = standing and self.d <= FULL_R and not harm
        self.hold = self.hold + 1 if ok else 0
        self.full = ok and self.hold >= HOLD_STEPS
        if harm or not standing:
            self.level = 0.0
        elif self.full:
            self.level = 1.0
        elif self.d <= HALF_R:
            self.level = 0.5
        elif TRAVEL - self.d >= QUARTER_GAIN:
            self.level = 0.25
        else:
            self.level = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.level)


# -- demo: clamp the rolled lip either side of the arms' side, tip the planter toward the arms onto its heel, roll it right in strokes ---
GRIP_R = 0.1625  # the clamp's centre: the middle of the roll's section, radially
GRIP_Z = 0.290  # and in height
BETA = math.radians(50.0)  # the hand points 50 degrees from straight down, in toward the axis (40 degrees below level)
GRIP_W, GRIP_F = 0.028, 40.0  # the roll is about 2.8 cm across along the line the fingers close on; 40 N a finger, capped (FINGER_CAP)
SPREAD = math.radians(30.0)  # each hand takes the lip 30 degrees either side of the bearing the planter leans to
HALF = math.radians(15.0)  # each stroke turns the planter 30 degrees: each hand from 15 degrees behind its place to 15 ahead
R_ROLL = 0.106  # the heel's radius where it rolls
TH_ROLL = math.radians(22.0)  # tilt while rolling
STEER = math.radians(12.0)  # the most a stroke is aimed off straight right, to correct drift toward the arms
DRIFT = 0.006  # how far a stroke's heel drifts toward the arms (seen in the simulator)
SERVO_GAIN, SERVO_STEP, TRIM_MAX = 0.15, 0.002, 0.05
FINGER_CAP = 40.0  # N: the demo sets each finger drive's force limit to this (the YAM gives about 40 N a finger), so no grip exceeds it


def _rz(b: float) -> np.ndarray:
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def grip_local(alpha: float):
    """The TCP's position and rotation, in the planter's frame, for a clamp of the rolled lip at bearing `alpha`: the hand comes from
    above and outside, pointing in toward the axis 50 degrees from straight down, and its fingers close across the roll's section on a
    line 50 degrees below level: one finger on the inside top of the roll, the other under its outer bulge. A pull toward the arms then
    bears on the inner finger instead of sliding the pads off the round of the roll."""
    out = np.array([math.cos(alpha), math.sin(alpha), 0.0])
    a = math.cos(BETA) * np.array([0.0, 0.0, -1.0]) - math.sin(BETA) * out
    c = out - (out @ a) * a
    c = c / np.linalg.norm(c)
    return np.array([GRIP_R * math.cos(alpha), GRIP_R * math.sin(alpha), GRIP_Z]), np.c_[c, np.cross(a, c), a]


def grip_world(pose, alpha: float):
    c, R = pose
    p, r = grip_local(alpha)
    return c + R @ p, R @ r


def demo_script(ctx):
    """A stroke: both hands clamp the rolled lip 30 degrees either side of the bearing that faces the arms (15 degrees round ahead of
    that), tip the planter 22 degrees toward the arms onto its heel, turn it 30 degrees clockwise (seen from above) about its own axis,
    the hands moving like a steering wheel's, so that the heel rolls and the planter travels about 5.5 cm to the right, stand it back up
    and let go. Five strokes take it to the cross; each is aimed at the cross from where the planter stands. Then the hands move away.
    Closed loop: every step reads the planter's pose (it never acts on the scorer; its log lines print the tracker's skid) and shifts each hand's command until the lip point it holds is
    where the planned pose puts it.
    Failure checks (--mode): none does nothing; drag clamps the lip and slides the planter upright to the cross; shove pushes its side
    low down with a closed hand, straight to the cross; lift tries to lift it clear; onearm rolls it with the right hand alone; tip
    rolls it tilted 34 degrees; hold stops at the cross still holding it; skid makes the same strokes but slides the tipped planter
    along on its heel without turning it.
    Grip force: at the start the script sets each finger drive's force limit to FINGER_CAP (40 N) on both arms, and logs the largest
    finger drive force asked for and applied."""
    import demokit as d
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN, READY

    mode = ctx.mode
    if mode not in ("full", "none", "drag", "shove", "lift", "onearm", "tip", "hold", "skid"):
        raise ValueError(mode)
    arms = {"left": ctx.left, "right": ctx.right}
    trim = {"left": np.zeros(3), "right": np.zeros(3)}
    grip_at = {}  # side -> the bearing, in the planter's frame, of its clamp
    sides = ("left", "right") if mode != "onearm" else ("right",)
    theta = math.radians(34.0) if mode == "tip" else TH_ROLL

    def now():
        s = ctx.state("planter")
        return np.asarray(s.pos, float), m.quat_matrix(s.quat_xyzw)

    def report(label):
        c, R = now()
        tilt = math.degrees(math.acos(np.clip(R[2, 2], -1, 1)))
        print(f"[demo] {label}: foot centre ({c[0]:.3f}, {c[1]:.3f}, {(c[2] - FLOOR) * 1000:+.1f} mm), tilt {tilt:.1f} deg, skid so far "
              f"{TRIAL.skid * 100:.1f} cm, finger drive force so far {peak['demand']:.1f} N asked, {peak['applied']:.1f} N applied "
              f"(cap {FINGER_CAP:.0f} N)", flush=True)

    def drive(pose_fn, seconds):
        """Each step: the planned pose `pose_fn(u)`, u from 0 to 1; each hand is commanded to its clamp on that pose, shifted by a trim
        that closes the gap between where its clamp is planned and where the planter's seen pose puts it."""
        n = max(1, round(seconds * CONTROL_HZ))
        for k in range(1, n + 1):
            want = pose_fn(ik.smoothstep(k / n))
            seen = now()
            for side in sides:
                p_d, r_d = grip_world(want, grip_at[side])
                p_m, _ = grip_world(seen, grip_at[side])
                trim[side] = trim[side] + np.clip(SERVO_GAIN * (p_d - p_m), -SERVO_STEP, SERVO_STEP)
                nrm = float(np.linalg.norm(trim[side]))
                if nrm > TRIM_MAX:
                    trim[side] *= TRIM_MAX / nrm
                arms[side]._solve(p_d + trim[side], r_d)
                arms[side].grip = d._as_grip(r_d)
            yield
            finger_force()

    def spin_of(pose):
        """The planter's turn about its own axis, with any tilt taken out."""
        return heading(pose[1])

    # the grip force cap: each finger drive's force limit set to FINGER_CAP on both arms, and the largest drive force logged
    from yam_arena.constants import FINGER_JOINTS
    env = ctx.emb._env.unwrapped
    robots = [env.scene[f"{side}_robot"] for side in ("left", "right")]
    fids = [r.find_joints(list(FINGER_JOINTS), preserve_order=True)[0] for r in robots]
    for r, ids in zip(robots, fids):
        r.write_joint_effort_limit_to_sim_index(limits=FINGER_CAP, joint_ids=ids)
    peak = {"demand": 0.0, "applied": 0.0}

    def finger_force():
        for r, ids in zip(robots, fids):
            peak["demand"] = max(peak["demand"], float(np.max(np.abs(m._np(r.data.computed_torque)[0, ids]))))
            peak["applied"] = max(peak["applied"], float(np.max(np.abs(m._np(r.data.applied_torque)[0, ids]))))

    def jmove(side, pos, rot, finger, t, label):
        """Go to a pose by straight-line joint motion, to the best of the IK solutions from where the arm is and from its ready pose."""
        arm = arms[side]
        best = None
        for seed in (arm.q, np.array(READY, float)):
            q, pe, re_ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), seed, base_pos=arm.base, iters=150)
            err = pe * 1000 + math.degrees(re_)
            if best is None or err < best[0]:
                best = (err, q, pe, re_)
        q0, f0 = arm.q.copy(), arm.finger
        n = max(1, round(t * CONTROL_HZ))
        for k in range(1, n + 1):
            u = ik.smoothstep(k / n)
            arm.q = q0 + (best[1] - q0) * u
            arm.cmd = np.r_[arm.q, float(np.clip((f0 + (finger - f0) * u) / FINGER_OPEN, 0.0, 1.0))]
            yield
        arm.finger, arm.grip = finger, d._as_grip(rot)
        arm._note(best[2] * 1000, math.degrees(best[3]), label)

    def take(side, pose, bearing, t=2.5):
        """Bring one open hand to the lip at this world bearing (about the planter's axis) and clamp it there."""
        arm = arms[side]
        grip_at[side] = bearing - spin_of(pose)
        p, r = grip_world(pose, grip_at[side])
        yield from jmove(side, p - 0.05 * r[:, 2], r, FINGER_OPEN, t, f"{side}: to the lip")
        p, r = grip_world(now(), grip_at[side])
        yield from arm.move(p, grip=d._as_grip(r), t=1.0, label=f"{side}: onto the lip")
        yield from arm.close(GRIP_W, GRIP_F, label=f"{side}: clamp")
        trim[side] = np.zeros(3)

    def release(side):
        arm = arms[side]
        yield from arm.open(t=0.5, label=f"{side}: let go")
        yield from arm.move(arm.at() - 0.06 * arm.approach_axis() + np.array([0.0, 0.0, 0.02]), t=0.8, label=f"{side}: off the lip")
        trim[side] = np.zeros(3)

    def leaned(pose, lean, th):
        """`pose` tipped about the edge of its heel toward the horizontal direction `lean` until its axis is `th` from vertical."""
        c, R = pose
        want = np.array([lean[0] * math.sin(th), lean[1] * math.sin(th), math.cos(th)])
        Q = _rot_between(R[:, 2], want)
        heel = c + HEEL_PTS @ R.T
        q = heel[int(np.argmax(heel[:, :2] @ lean))]
        return q + Q @ (c - q), Q @ R

    def tilt_to(lean, th, seconds):
        start = now()
        th0 = math.acos(np.clip(start[1][2, 2], -1, 1))
        yield from drive(lambda u: leaned(start, lean, th0 + (th - th0) * u), seconds)

    def roll(lean, way, dist, seconds):
        """Roll `dist` metres along `way` on the heel: the planter turns clockwise (seen from above) about its own axis. In skid mode it
        doesn't turn: the heel slides."""
        c0, R0 = leaned(now(), lean, theta)
        spin = 0.0 if mode == "skid" else 1.0
        yield from drive(lambda u: (c0 + dist * u * np.array([way[0], way[1], 0.0]), R0 @ _rz(-spin * dist * u / R_ROLL)), seconds)

    for _ in range(5):
        yield
    report(f"{mode} start")
    if mode == "none":
        yield from ctx.left.wait(4.0)
        report("end")
        return

    if mode == "shove":  # the left hand, closed, pushes the planter's left side low down, straight across to the cross
        c0 = now()[0]
        g = d.tilted(60.0, -90.0)
        z = TOP + PAVER_T + 0.08
        y0 = c0[1] + _r_at(0.08) + 0.03
        yield from ctx.left.move((c0[0], y0 + 0.04, z + 0.10), grip=g, finger=0.0, t=2.5, label="left: beside it")
        yield from ctx.left.move((c0[0], y0, z), t=1.5, label="left: down beside it")
        yield from ctx.left.move((c0[0], y0 - (START[1] - MARK[1]) - 0.03, z), t=10.0, label="left: shove")
        report("shoved")
        yield from ctx.left.move((c0[0] - 0.12, y0 - 0.2, z + 0.15), t=2.0, label="left: away")
        yield from ctx.left.wait(2.0)
        report("end")
        return

    stroke = 2 * HALF * R_ROLL
    for k in range(8):
        c = now()[0]
        to_go = np.array([MARK[0] - c[0], MARK[1] - c[1]])
        if -to_go[1] < 0.008:  # level with the cross, or past it
            break
        left_n = math.ceil(-to_go[1] / stroke - 0.05)  # strokes still to go
        dist = float(-to_go[1]) if left_n <= 1 else float(np.linalg.norm(to_go))
        # aimed at the cross, allowing for the 6 mm a stroke drifts toward the arms, within 12 degrees of straight right
        head = float(np.clip(math.atan2(to_go[0] + DRIFT * left_n, -to_go[1]), -STEER, STEER))
        way = np.array([math.sin(head), -math.cos(head)])
        lean = np.array([way[1], -way[0]])  # the travel turned a quarter turn clockwise: toward the arms for travel to the right
        pose = now()
        bear = {"left": math.pi - SPREAD + HALF, "right": math.pi + SPREAD + HALF}
        yield from ctx.together(*(take(s, pose, bear[s], t=2.5 if k == 0 else 1.8) for s in sides))
        if mode == "lift":
            c0, R0 = now()
            yield from drive(lambda u: (c0 + np.array([0.0, 0.0, 0.08 * u]), R0), 4.0)
            report("lift")
            break
        if mode == "drag":
            c0, R0 = now()
            yield from drive(lambda u: (c0 + dist * u * np.array([way[0], way[1], 0.0]), R0), 8.0)
            report("dragged")
            break
        report(f"  took {k + 1}")
        yield from tilt_to(lean, theta, 3.0)
        report(f"  tipped {k + 1}")
        go = min(stroke, dist)
        yield from roll(lean, way, go, 0.6 + 2.4 * go / stroke)
        report(f"  rolled {k + 1}")
        yield from tilt_to(lean, 0.0, 2.5)
        report(f"stroke {k + 1}")
        c = now()[0]
        if mode == "hold" and (dist <= stroke or math.hypot(MARK[0] - c[0], MARK[1] - c[1]) < 0.03):
            yield from ctx.left.wait(4.0)
            report("end")
            return
        yield from ctx.together(*(release(s) for s in sides))
    park = {"left": ((0.20, 0.38, TOP + 0.30), d.tilted(60.0, -30.0)), "right": ((0.20, -0.42, TOP + 0.30), d.tilted(60.0, 30.0))}
    yield from ctx.together(*(release(s) for s in sides if mode in ("lift", "drag")))
    yield from ctx.together(*(arms[s].move(park[s][0], grip=park[s][1], t=2.0, label=f"{s}: away") for s in sides))
    yield from ctx.left.wait(2.0)
    report("end")
