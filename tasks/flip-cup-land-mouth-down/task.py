"""Flip cup: flick an empty 16 oz plastic party cup off the table edge so that it turns over once and lands mouth down.

On the station table stands a 48 x 46 cm melamine tabletop board, 4 cm thick, whose edge facing the right arm (the line
y = -0.05, along x) stands in for the edge of a party table. An empty Solo 16 oz red party cup (P16R: 9.85 cm rim, 6.43 cm
base, 12.0 cm tall, 11.8 g) stands mouth up on the board with its rim 1.5 cm out over that edge, the way the cup is set up
in the party game Flip Cup. A soda can, a beer bottle and a napkin stand on the far side of the board (distractors, not
scored).

Model (declared in code):
- Cup ("cup", one free rigid body, 11.8 g): a 24-panel tapered wall (each panel a 3 mm thick convex hull), a 1.5 mm base
  disc and a rolled rim bead of 24 convex segments standing 2.5 mm out. PhysX takes its mass distribution from the
  colliders, which is shell-like (most of the mass in the wall), as in the real thin-walled cup. Depenetration speed
  capped at 1 m/s.
- Tabletop ("tabletop", fixed): one box, melamine friction (0.40 static, 0.35 dynamic); cup polystyrene (0.45, 0.40).

Scoring (`Trial` below, every control step at 20 Hz, from the cup's pose and the fingertip sample points):
- A finger *touches* the cup when any fingertip sample point is within 12 mm of the cup's outer surface (the points sit
  4.5 mm behind the finger pad, so this is a gap of about 7.5 mm).
- Recorded for the trial (cleared only in on_reset): *handled*, a finger touched the cup while the cup's axis was more than
  45 degrees from upright (carried, turned or righted by hand instead of flicked); *fell*, any point of the cup's base or
  rim circle went more than 2 cm below the tabletop (off the table); the largest *turn* of its axis from upright.
- Resting: lowest point within 5 mm of the tabletop, midpoint of its axis over the board, still (under 2 cm/s and
  0.3 rad/s), every fingertip point 5 cm or more from it. Mouth down: axis within 15 degrees of straight down.
- Score: handled or fell -> 0. Otherwise, resting: mouth down for 1 s in a row -> 1.0 (success); else a turn of 135
  degrees or more -> 0.5; else a turn of 100 degrees or more (past lying on its side, 82 degrees) -> 0.25; else 0.
  Not resting -> 0.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Flip the empty cup off the table edge so it lands mouth down on the table."
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
DEBUG = os.environ.get("CUP_DEBUG", "") not in ("", "0")

# -- the tabletop: a melamine panel standing in for the edge of a party table ------------------------------------------
TABLE_X = (0.12, 0.60)  # x extent (m)
TABLE_Y = (-0.05, 0.41)  # y extent; the edge the cup overhangs is y = -0.05, facing the right arm
TABLE_T = 0.04
TABLE_TOP_Z = TOP + TABLE_T
EDGE_Y = TABLE_Y[0]

# -- the cup: Solo P16R 16 oz party cup, HIPS, 3.876 in rim, 2.53 in base, 4.73 in tall -------------------------------
R_RIM = 0.0492  # outer radius of the rolled rim
R_BASE = 0.0321
H = 0.120
BEAD_H = 0.004  # the rolled rim's height
BEAD_D = 0.0025  # how far the rim stands out from the wall below it
WALL_T = 0.003  # collider thickness (the real wall is about 0.5 mm; thicker colliders keep a fast finger from passing in)
R_WALL_TOP = R_RIM - BEAD_D  # the wall's outer radius just below the bead
CUP_MASS = 0.0118
OVERHANG = 0.015  # how far the rim stands out past the table edge
CUP_XY = (0.33, EDGE_Y - OVERHANG + R_RIM)  # base centre: the rim stands OVERHANG past the edge
N_SEG = 24
CUP_FRICTION = (0.45, 0.40)
TABLE_FRICTION = (0.40, 0.35)


def wall_r(z: float) -> float:
    """The cup's outer radius at height z above its base (m)."""
    if z >= H - BEAD_H:
        return R_RIM
    return R_BASE + (R_WALL_TOP - R_BASE) * max(z, 0.0) / (H - BEAD_H)


def _hull8(pts) -> m.Mesh:
    """A convex hexahedron from 8 corners: bottom four then top four, each counter-clockwise from above."""
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return m.Mesh(np.array(pts, float), np.array(f, int))


def _segment(r0o, r1o, z0, z1, t, a0, a1) -> m.Mesh:
    """One wall segment between angles a0 and a1: outer radius r0o at z0 to r1o at z1, thickness t inward."""
    def p(r, a, z):
        return (r * math.cos(a), r * math.sin(a), z)
    bottom = [p(r0o - t, a0, z0), p(r0o - t, a1, z0), p(r0o, a1, z0), p(r0o, a0, z0)]
    top = [p(r1o - t, a0, z1), p(r1o - t, a1, z1), p(r1o, a1, z1), p(r1o, a0, z1)]
    return _hull8(bottom + top)


# -- the cup's body: PhysX's depenetration speed capped at 1 m/s, so a fast finger cannot launch the light cup out of an
#    overlap (mechkit writes this cap for links, not for free parts) ---------------------------------------------------
_file0 = getattr(m._file, "_plain", m._file)
_HEAD = '    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]\n)\n{\n'


def _file_cup(root, head_api, attrs, body, looks, friction):
    text = _file0(root, head_api, attrs, body, looks, friction)
    if root == "P_cup":
        text = text.replace(_HEAD, '    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]\n)\n{\n'
                            "    float physxRigidBody:maxDepenetrationVelocity = 1.0\n", 1)
    return text


_file_cup._plain = _file0
m._file = _file_cup

LOOKS = {
    "red": m.Look(color=m.rgb(0.80, 0.06, 0.07), roughness=0.35),
    "white": m.Look(color=m.rgb(0.95, 0.95, 0.94), roughness=0.4),
    "melamine": m.Look(color=m.rgb(0.78, 0.72, 0.62), roughness=0.5),
    "edge": m.Look(color=m.rgb(0.25, 0.22, 0.20), roughness=0.6),
}


def _cup() -> m.Part:
    zt = H - BEAD_H
    outside = m.lathe([(0.0, 0.0005), (R_BASE - 0.002, 0.0005), (R_BASE, 0.003), (R_WALL_TOP, zt), (R_RIM, zt + 0.001), (R_RIM, H - 0.001),
                       (R_RIM - 0.002, H)], segments=48)
    inside = m.lathe([(R_RIM - 0.0021, H - 0.0002), (R_WALL_TOP - WALL_T, zt), (R_BASE - WALL_T, 0.004), (0.0, 0.004)], segments=48)
    parts = [("outside", outside, "red"), ("inside", inside, "white")]
    cols = [m.col_cylinder("base_c", R_BASE - 0.001, 0.0015, (0, 0, 0.00075))]
    step = 2 * math.pi / N_SEG
    for k in range(N_SEG):
        a0, a1 = k * step - 0.002, (k + 1) * step + 0.002
        cols.append(m.col_hull(f"wall{k:02d}_c", _segment(R_BASE, R_WALL_TOP, 0.0, zt, WALL_T, a0, a1)))
        cols.append(m.col_hull(f"bead{k:02d}_c", _segment(R_RIM, R_RIM, zt, H, BEAD_D + WALL_T, a0, a1)))
    return m.free("cup", (CUP_XY[0], CUP_XY[1], TABLE_TOP_Z + 0.003), parts, cols, LOOKS, mass=CUP_MASS, friction=CUP_FRICTION, torsional_patch=0.01)


def _table() -> m.Part:
    sx, sy = TABLE_X[1] - TABLE_X[0], TABLE_Y[1] - TABLE_Y[0]
    c = ((TABLE_X[0] + TABLE_X[1]) / 2, (TABLE_Y[0] + TABLE_Y[1]) / 2)
    parts = [("top", m.box((sx, sy, TABLE_T - 0.002), (0, 0, (TABLE_T - 0.002) / 2)), "edge"),
             ("laminate", m.box((sx, sy, 0.002), (0, 0, TABLE_T - 0.001)), "melamine")]
    return m.fixed("tabletop", (c[0], c[1], TOP), parts, [m.col_box("top_c", (sx, sy, TABLE_T), (0, 0, TABLE_T / 2))], LOOKS, friction=TABLE_FRICTION)


CUP = _cup()
TABLE = _table()
DISTRACTORS = [
    SimReady("soda_can", pos=(0.53, 0.02, TABLE_TOP_Z), size=(0.067, 0.067, 0.092), mass=0.38, package="food_beverage_can_b01",
             usd="sm_food_beverage_can_b01_01.usd", convex=True),
    SimReady("beer_bottle", pos=(0.53, 0.12, TABLE_TOP_Z), size=(0.060, 0.060, 0.222), mass=0.20, package="food_beverage_beerBottle_a01",
             usd="sm_food_beverage_beerBottle_a01_01.usd", convex=True),
    SimReady("napkin", pos=(0.46, 0.28, TABLE_TOP_Z), size=(0.26, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01",
             usd="sm_cleaning_cloth_a01_01.usd", yaw=0.0),
]
OBJECTS = [TABLE, CUP, *DISTRACTORS]


# -- reading the cup ---------------------------------------------------------------------------------------------------

_TH = np.linspace(0.0, 2 * np.pi, 32, endpoint=False)
_RING_PTS = np.concatenate([np.c_[R_BASE * np.cos(_TH), R_BASE * np.sin(_TH), np.zeros_like(_TH)],
                            np.c_[R_RIM * np.cos(_TH), R_RIM * np.sin(_TH), np.full_like(_TH, H)]])


def tilt_deg(s) -> float:
    a = s.rotation()[:, 2]
    return float(math.degrees(math.acos(float(np.clip(a[2], -1.0, 1.0)))))


def outline(s) -> np.ndarray:
    """World points round the cup's base and rim."""
    return np.asarray(s.to_world(_RING_PTS), float)


def over_table(p) -> bool:
    return bool(TABLE_X[0] <= p[0] <= TABLE_X[1] and TABLE_Y[0] <= p[1] <= TABLE_Y[1])


def centre(s) -> np.ndarray:
    """The midpoint of the cup's axis, in the world."""
    return np.asarray(s.to_world(np.array([[0.0, 0.0, H / 2]]))[0], float)


def finger_gap(s, tips) -> float:
    """The smallest distance (m) from a fingertip point to the cup's outer surface (inside the cup counts as 0)."""
    loc = np.asarray(s.to_local(tips), float)
    best = 1.0
    for x, y, z in loc:
        rr = math.hypot(x, y)
        if 0.0 <= z <= H:
            d = max(rr - wall_r(z), 0.0)
        elif z < 0.0:
            d = math.hypot(max(rr - R_BASE, 0.0), -z)
        else:
            d = math.hypot(max(rr - R_RIM, 0.0), z - H)
        best = min(best, d)
    return float(best)


def now(env) -> dict:
    s = m.state(env, "cup")
    pts = outline(s)
    gap = finger_gap(s, m.fingertips(env))
    return dict(s=s, tilt=tilt_deg(s), low=float(pts[:, 2].min() - TABLE_TOP_Z), over=over_table(centre(s)), gap=gap,
                still=float(np.linalg.norm(s.lin_vel)) < 0.02 and float(np.linalg.norm(s.ang_vel)) < 0.3)


HOLD_STEPS = 20  # 1 s at 20 Hz
TOUCH = 0.012  # m from a fingertip point to the cup's surface
HANDLED_DEG = 45.0
FELL_DEPTH = 0.02
REST_TOL = 0.005
CLEAR = 0.05
MOUTH_DOWN_DEG = 165.0
TURNED_DEG = 135.0
LEFT_SIDE_DEG = 100.0  # past lying on its side (82 degrees for this taper): it left the table turning


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded violation is cleared
        self.handled = False
        self.fell = False
        self.level = 0.0
        self.max_tilt = 0.0
        self.hold = 0
        self.steps = 0
        self.log = []

    def step(self, env):
        c = now(env)
        self.steps += 1
        if c["gap"] < TOUCH and c["tilt"] > HANDLED_DEG:
            self.handled = True
        if c["low"] < -FELL_DEPTH:
            self.fell = True
        self.max_tilt = max(self.max_tilt, c["tilt"])
        rest = abs(c["low"]) <= REST_TOL and c["over"] and c["still"] and c["gap"] >= CLEAR
        down = rest and c["tilt"] >= MOUTH_DOWN_DEG and not (self.handled or self.fell)
        self.hold = self.hold + 1 if down else 0
        if self.handled or self.fell or not rest:
            self.level = 0.0
        elif self.hold >= HOLD_STEPS:
            self.level = 1.0
        elif self.max_tilt >= TURNED_DEG:
            self.level = 0.5
        elif self.max_tilt >= LEFT_SIDE_DEG:
            self.level = 0.25
        else:
            self.level = 0.0
        if DEBUG:
            s = c["s"]
            print(f"[trial] {self.steps} pos {np.round(s.pos, 3)} tilt {c['tilt']:.0f} low {c['low'] * 1000:.0f} gap {c['gap'] * 1000:.0f} "
                            f"w {np.round(s.ang_vel, 1)} tip {np.round(m.fingertips(env)[12:].mean(0), 3)} handled {self.handled} fell {self.fell} level {self.level}", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.level >= 1.0)


def score(objects) -> float:
    return float(TRIAL.level)


# -- the demonstration -------------------------------------------------------------------------------------------------

_OVR = {}


def _env(k, v):
    """A strike setting: the default, or a red-team mode's override."""
    return float(_OVR.get(k, v))


def demo_script(ctx):
    """The right gripper, closed and level, pointing along +x, waits beside the table edge a few centimetres below the
    overhanging rim, then flicks up and a little inward: its joint targets are thrown well past the strike for 0.1 s, so
    the joints run at their torque limits and the finger meets the rim's underside at about 1.5 m/s, then pulled back
    down and away so nothing touches the turning cup. Open loop: the cup does not move before the strike.
    Red-team modes: none (does nothing), push (tips the cup over slowly with the finger, as a hand would), swat (a fast
    sideways hit at the rim with no lift), hard (the same flick with far too much lift)."""
    import demokit as d
    from yam_arena.constants import FINGER_CLOSED

    r = ctx.right
    mode = ctx.mode
    _OVR.clear()
    if mode.startswith("p:"):
        _OVR.update(kv.split("=") for kv in mode[2:].split(":"))
        mode = "full"
    for _ in range(3):
        yield
    c = ctx.pos("cup")
    if DEBUG:
        print("[cup] start", np.round(c, 4), "tilt", tilt_deg(ctx.state("cup")), flush=True)
    if mode not in ("full", "none", "push", "swat", "hard"):
        raise ValueError(mode)
    if mode == "none":
        yield from r.wait(4.0)
        return
    if mode == "swat":
        _OVR.update(FY=0.25, FZ=0.0, ZF=0.095)
    if mode == "hard":
        _OVR.update(FY=0.15, FZ=0.60)
    grip = d.level(0.0)
    zf = TABLE_TOP_Z + _env("ZF", 0.065)
    yf = EDGE_Y - _env("YOFF", 0.020)
    xf = c[0] + _env("XOFF", 0.002)
    yield from r.move((xf - 0.02, yf - 0.06, zf - 0.04), grip=grip, finger=FINGER_CLOSED, t=2.0, label="beside the edge")
    yield from r.move((xf, yf, zf - 0.02), t=1.0, label="under the rim")
    yield from r.move((xf, yf, zf), t=0.6, label="touch")
    yield from r.wait(0.3)
    if mode == "push":  # tipping it over by hand: the finger pushes the rim inward and up, slowly
        yield from r.move((xf, yf, TABLE_TOP_Z + 0.105), t=0.8, label="under the bead")
        yield from r.move((xf, yf + 0.10, TABLE_TOP_Z + 0.16), t=3.0, label="push over")
        yield from r.move((xf - 0.03, yf - 0.10, TABLE_TOP_Z + 0.16), t=1.5, label="clear")
        yield from r.wait(3.0)
        return
    if DEBUG:
        print("[cup] tcp", np.round(r.tcp(), 4), "tips", np.round(m.fingertips(ctx.emb._env)[12:], 3).tolist(), flush=True)
    # the flick: joint targets thrown well past the strike, so the joints run at their torque limits, then pulled back
    from yam_arena import ik
    base = np.asarray(station.BASES["right"], float)
    q0 = np.array(r.q, dtype=float)
    p0, _ = ik.tcp_pose(q0, base)
    jac = np.zeros((3, 6))
    for j in range(6):
        dq = np.zeros(6)
        dq[j] = 1e-4
        jac[:, j] = (ik.tcp_pose(q0 + dq, base)[0] - p0) / 1e-4
    strike = np.array([0.0, _env("FY", 0.04), _env("FZ", 0.35)])
    dq = np.linalg.lstsq(jac, strike, rcond=None)[0]
    dq[4:] = 0.0 if _env("NOWRIST", 0) else dq[4:]
    q1 = np.clip(q0 + dq, ik.LOWER + 0.02, ik.UPPER - 0.02)
    for _ in range(int(_env("NS", 2))):
        r.q = q1.copy()
        r.cmd = np.r_[q1, 0.0]
        yield
    back = np.linalg.lstsq(jac, np.array([0.0, -_env("BY", 0.10), _env("BZ", -0.03)]), rcond=None)[0]
    q2 = np.clip(q0 + back, ik.LOWER + 0.02, ik.UPPER - 0.02)
    for _ in range(int(_env("NB", 8))):
        r.q = q2.copy()
        r.cmd = np.r_[q2, 0.0]
        yield
    yield from r.move(r.tcp() + np.array([-0.03, -0.08, 0.0]), t=1.0, label="clear")
    yield from r.wait(3.0)
