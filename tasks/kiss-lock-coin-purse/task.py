"""Put two coins back into an open kiss-lock coin purse and snap its ball clasp shut.

A small coin purse on a round 8.5 cm sew-in kiss-lock frame stands on the table, open. Two euro coins lie beside it. The two frame halves
each carry a ball on top; to shut the purse the front half is swung over until its ball rides past the back half's ball and snaps home.
The snap is a detent: in the last 9.7 degrees before shut the balls push the front half back open (0.25 N m, about 5 N at the ball),
and inside the last 2 degrees they pull it shut and hold it there. Let go anywhere in between and it is pushed back out to 9.7 degrees open.

The scene, scorer and scripted demo; run commands are in README.md.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the coins back in the coin purse and close it."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 3.0  # success() latches after the tracker's own 2 s hold; 3 s more lets the video show the hands leaving
HOLD_STEPS = 40  # 2 s at 20 Hz

# -- the purse (its own frame: origin at the bottom's centre, x forward, y along the frame's width) ----------------------------------
PX, PY = 0.300, 0.000  # where it stands
YAW = -90.0  # turned so the frame's width runs along world x: the opening (front) half faces the right arm (-y), the back half the left arm
FRAME_W = 0.085  # the kiss-lock frame's width, rivet to rivet
ARCH_H = 0.040  # the arch's height above its rivets
BODY_H = 0.045  # the fabric body below the rivets
BELLY_D = 0.056  # the body's depth at the bottom
RIM_D = 0.036  # the body's depth at the rivet line
WALL = 0.003  # fabric thickness
HX = RIM_D / 2  # each frame half is hinged on its own rim line, x = +-HX
LEAN = math.degrees(math.asin(HX / ARCH_H))  # 26.7 degrees: the lean of each half when shut
OPEN_OUT = 25.0  # the front half starts leaning 25 degrees outward
SHUT = -(LEAN + OPEN_OUT)  # the front half's joint angle when shut: -51.7 degrees
SNAP_DEG = 2.0  # within this of shut, the balls have passed: pulled shut and held
RIDE_DEG = 9.7  # from SNAP_DEG to this, the balls ride over each other and push the front half open
SPRING = 2.0  # N m/rad: the detent's stiffness
RIDE_TORQUE = 0.25  # N m pushing open while the balls ride over (about 5 N at the ball, 4.75 cm from the hinge)
HOLD_TORQUE = 0.30  # N m pulling shut once snapped
BALL_R = 0.0045
BALL_DY = 0.005  # each ball sits 5 mm off the centre line, so they pass side by side
P_MASS = 0.035

# -- the coins: euro 2 coins, 25.75 mm across, 2.20 mm thick, 8.5 g ---------------------------------------------------------------
COIN_R, COIN_T, COIN_M = 0.025750 / 2, 0.00220, 0.0085
COINS = {"coin_left": (0.215, 0.110), "coin_right": (0.215, -0.110)}

LOOKS = {
    "fabric": m.Look(color=m.rgb(0.62, 0.10, 0.14), roughness=0.85),
    "brass": m.Look(color=m.rgb(0.80, 0.66, 0.34), metallic=0.9, roughness=0.3),
    "coin_out": m.Look(color=m.rgb(0.75, 0.75, 0.74), metallic=0.85, roughness=0.3),  # the 2-euro coin's white cupronickel ring
    "coin_in": m.Look(color=m.rgb(0.78, 0.66, 0.36), metallic=0.85, roughness=0.35),  # and its yellow nickel-brass centre
}


def _half_panel() -> m.Mesh:
    """One fabric half above the rivet line, filling the arch: a half ellipse 85 mm wide and 40 mm tall, 3 mm thick, standing in the x = 0 plane."""
    n = 24
    out = [(FRAME_W / 2 * math.cos(math.pi * k / n), ARCH_H * math.sin(math.pi * k / n)) for k in range(n + 1)]
    outline = [(a, b) for a, b in out]  # counter-clockwise in (y, z)
    p = m.prism(outline, -WALL / 2, WALL / 2)
    return m.Mesh(p.v[:, [2, 0, 1]], p.f)  # (a, b, t) -> x = t, y = a, z = b: a cyclic swap keeps the faces outward


def _arch_bar() -> m.Mesh:
    pts = [(0.0, FRAME_W / 2 * math.cos(math.pi * k / 20), ARCH_H * math.sin(math.pi * k / 20)) for k in range(21)]
    return m.tube(pts, 0.0018, sides=8)


def _ball(dy: float) -> m.Mesh:
    return m.join(m.cylinder(0.0012, 0.004, (0.0, dy, ARCH_H + 0.002)), m.sphere(BALL_R, (0.0, dy, ARCH_H + 0.003 + BALL_R)))


def _half(deg: float, hinge_x: float, dy: float, tag: str):
    """A frame half turned `deg` about y through its rim line: visual parts and colliders."""
    def place(mesh):
        return mesh.turned(deg, "y").moved((hinge_x, 0.0, BODY_H))
    panel, bar, ball = place(_half_panel()), place(_arch_bar()), place(_ball(dy))
    parts = [(f"{tag}_panel", panel, "fabric"), (f"{tag}_bar", bar, "brass"), (f"{tag}_ball", ball, "brass")]
    cols = [m.col_hull(f"{tag}_panel_c", panel), m.col_hull(f"{tag}_ball_c", place(m.sphere(BALL_R, (0.0, dy, ARCH_H + 0.003 + BALL_R))))]
    return parts, cols


def _body():
    """The fabric body below the rivet line, a tapered pouch open at the top, and the back frame half, which the body's back is sewn to."""
    parts, cols = [], []
    slant = math.degrees(math.atan2((BELLY_D - RIM_D) / 2, BODY_H))
    mid_x = (BELLY_D + RIM_D) / 4
    height = BODY_H / math.cos(math.radians(slant))
    for k, sx in enumerate((1.0, -1.0)):  # front and back walls, leaning in towards the top
        wall = m.box((WALL, FRAME_W + 2 * WALL, height)).turned(-sx * slant, "y").moved((sx * (mid_x - WALL / 2), 0.0, BODY_H / 2))
        parts.append((f"wall{k}", wall, "fabric"))
        cols.append(m.col_hull(f"wall{k}_c", wall))
    for k, sy in enumerate((1.0, -1.0)):  # the two gussets at the ends
        side = m.prism([(-BELLY_D / 2, 0.0), (BELLY_D / 2, 0.0), (RIM_D / 2, BODY_H), (-RIM_D / 2, BODY_H)], 0.0, WALL)
        side = m.Mesh(side.v[:, [0, 2, 1]] * np.array([1.0, 1.0, 1.0]), side.f[:, ::-1]).moved((0.0, sy * (FRAME_W / 2 + WALL / 2) - WALL / 2, 0.0))
        parts.append((f"gusset{k}", side, "fabric"))
        cols.append(m.col_hull(f"gusset{k}_c", side))
    floor = m.box((BELLY_D, FRAME_W, WALL), (0.0, 0.0, WALL / 2))
    parts.append(("floor", floor, "fabric"))
    cols.append(m.col_box("floor_c", (BELLY_D, FRAME_W, WALL), (0.0, 0.0, WALL / 2)))
    for k, (sx, sy) in enumerate([(sx, sy) for sx in (1, -1) for sy in (1, -1)]):  # rivets
        parts.append((f"rivet{k}", m.cylinder(0.0025, 0.004, (sx * HX, sy * (FRAME_W / 2 + 0.002), BODY_H), axis="y", segments=12), "brass"))
    bp, bc = _half(LEAN, -HX, +BALL_DY, "back")
    return parts + bp, cols + bc


def _purse() -> m.Part:
    fp, fc = _body()
    jp, jc = _half(OPEN_OUT, HX, -BALL_DY, "front")
    jaw = m.hinge("jaw", "Y", (HX, 0.0, BODY_H), SHUT, 8.0, jp, jc, mass=0.008, spring=SPRING, damping=0.01, friction=0.01, effort=5.0,
                  inertia=(1.5e-5, 6e-6, 1e-5))
    return m.mechanism("purse", (PX, PY, TOP), [jaw], LOOKS, frame_parts=fp, frame_colliders=fc, free_mass=P_MASS, yaw_deg=YAW,
                       frame_inertia=(2.5e-5, 1.5e-5, 2.5e-5), friction=(0.9, 0.8))


def _coin(name, xy) -> m.Part:
    face = m.cylinder(COIN_R, COIN_T, (0, 0, COIN_T / 2), segments=32)
    core = m.cylinder(COIN_R * 0.72, COIN_T + 0.0002, (0, 0, COIN_T / 2), segments=32)
    return m.free(name, (xy[0], xy[1], TOP + 0.0002), [("ring", face, "coin_out"), ("core", core, "coin_in")],
                  [m.col_cylinder("c", COIN_R, COIN_T, (0, 0, COIN_T / 2))], LOOKS, mass=COIN_M, friction=(0.9, 0.8))


OBJECTS = [
    _purse(),
    *[_coin(n, xy) for n, xy in COINS.items()],
    # distractors (SimReady, loose): a pair of glasses and a mug, out of the task's way
    SimReady("glasses", pos=(0.45, 0.20, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
             usd="sm_accessory_eyeglasses_a01_01.usd", yaw=math.pi / 2),
    SimReady("mug", pos=(0.43, -0.21, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
             usd="sm_dishware_mug_a01_01.usda", yaw=2.4),
]

# -- scoring ----------------------------------------------------------------------------------------------------------------------
CLEAR = 0.03  # every fingertip point at least 3 cm from the purse's envelope and from each coin
# the purse's envelope in its own frame, with the frame open: body and back half, plus the front half swung out 25 degrees
ENV_LO = np.array([-BELLY_D / 2, -(FRAME_W / 2 + WALL), 0.0])
ENV_HI = np.array([HX + (ARCH_H + 0.003 + 2 * BALL_R) * math.sin(math.radians(OPEN_OUT)) + BALL_R, FRAME_W / 2 + WALL, BODY_H + ARCH_H + 0.003 + 2 * BALL_R])


def coin_inside(purse, coin_pos) -> bool:
    """The coin's centre inside the fabric body, below the rivet line and inside its tapered walls."""
    p = purse.to_local(np.asarray(coin_pos, float))[0]
    if not (0.0 < p[2] < BODY_H):
        return False
    half_d = BELLY_D / 2 - (BELLY_D - RIM_D) / 2 * p[2] / BODY_H - WALL
    return bool(abs(p[0]) < half_d and abs(p[1]) < FRAME_W / 2)


def _box_gap(local, lo, hi) -> float:
    out = np.maximum(np.maximum(lo - local, local - hi), 0.0)
    return float(np.min(np.linalg.norm(out, axis=1)))


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.lost, self.hold, self.full, self.score = False, 0, False, 0.0

    def detent(self, env, purse):
        a = purse.deg("jaw")
        if a <= SHUT + SNAP_DEG:
            target = math.radians(SHUT) - HOLD_TORQUE / SPRING  # snapped: pulled onto its stop
        elif a <= SHUT + RIDE_DEG:
            target = math.radians(a) + RIDE_TORQUE / SPRING  # the balls riding over each other: pushed back open
        else:
            target = math.radians(a)  # free: the frame stays where it is put
        m.set_target(env, "purse", "jaw", target)
        return a

    def step(self, env):
        purse = m.state(env, "purse")
        a = self.detent(env, purse)
        tips = m.fingertips(env)
        hands_off = _box_gap(purse.to_local(tips), ENV_LO, ENV_HI) >= CLEAR
        if purse.pos[2] < TOP - 0.02:
            self.lost = True  # the purse off the table: latched for the trial
        inside = 0
        for name in COINS:
            c = m.state(env, name)
            if c.pos[2] < TOP - 0.02:
                self.lost = True  # a coin off the table: latched for the trial
            clear = float(np.min(np.linalg.norm(tips - c.pos, axis=1))) >= CLEAR
            inside += int(coin_inside(purse, c.pos) and clear)
        snapped = a <= SHUT + SNAP_DEG
        ok = inside == 2 and snapped and hands_off and not self.lost
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or (self.hold >= HOLD_STEPS and not self.lost)
        if self.lost:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif inside == 2 and snapped:
            self.score = 0.75
        else:
            self.score = 0.25 * inside


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full and not TRIAL.lost)


# -- the scripted demo: open loop, from the published starting layout -----------------------------------------------------------
# The purse is turned -90 degrees: its own +x (the front, opening half) points along world -y, its own +y along world +x.
HI = TOP + 0.14
RIVET_Z = TOP + BODY_H
PICK_Z = TOP + 0.0125  # TCP height to pinch a coin lying flat on the table (fingertips 1.3 cm below the TCP)
DROP_Y = PY - 0.018  # over the middle of the open mouth, between the back half's top and the open front half's top
PARK_L, PARK_R = (0.20, 0.27, TOP + 0.20), (0.20, -0.27, TOP + 0.20)
SQUEEZE_X = PX + 0.024  # the right hand pinches the two frame halves together here, beside the balls (they sit within 1 cm of the middle)
SQUEEZE_TIP = RIVET_Z + 0.022  # fingertips 2.2 cm above the rivets
SHOVE_Y = PY - 0.0106  # red team "shove": a closed hand pushed from the front only, 3 mm past the front half's shut position
MODE = "full"  # red-team copies of this file change it: "nothing", "shove", "onecoin"


def _pick_drop(xy, drop_x, side):
    x, y = xy
    return [
        Waypoint(1.5, (x, y, TOP + 0.06), FINGER_OPEN, f"{side}: over its coin"),
        Waypoint(1.2, (x, y, PICK_Z), FINGER_OPEN, f"{side}: fingers down either side of the coin"),
        Waypoint(1.0, (x, y, PICK_Z), FINGER_CLOSED, f"{side}: pinch the coin across its rim"),
        Waypoint(1.0, (x, y, HI), FINGER_CLOSED, f"{side}: lift"),
        Waypoint(1.5, (drop_x, DROP_Y, HI), FINGER_CLOSED, f"{side}: over the purse's mouth"),
        Waypoint(1.0, (drop_x, DROP_Y, TOP + 0.115), FINGER_CLOSED, f"{side}: lower to just above the frame"),
        Waypoint(0.6, (drop_x, DROP_Y, TOP + 0.115), FINGER_OPEN, f"{side}: let the coin drop in"),
        Waypoint(1.0, (drop_x, DROP_Y, HI), FINGER_OPEN, f"{side}: up"),
    ]


def _dur(wps):
    return sum(w.duration for w in wps)


def demo():
    """Each hand drops one coin into the mouth. Then the right hand straddles the frame with its fingers open, one finger outside
    each half, and closes: the front finger swings the open half over, the back finger holds the back half, and the balls snap past
    each other between them, as a thumb and finger squeeze a kiss lock shut."""
    if MODE == "nothing":
        return {"left": [Waypoint(30.0, PARK_L, FINGER_OPEN, "wait")], "right": [Waypoint(30.0, PARK_R, FINGER_OPEN, "wait")]}
    left = _pick_drop(COINS["coin_left"], PX - 0.015, "left") + [Waypoint(1.2, PARK_L, FINGER_OPEN, "left: out of the way")]
    right = [Waypoint(_dur(left), PARK_R, FINGER_OPEN, "right: wait")]
    if MODE != "onecoin":
        right += _pick_drop(COINS["coin_right"], PX + 0.015, "right")
    tcp = SQUEEZE_TIP + 0.013
    if MODE == "shove":
        right += [
            Waypoint(1.5, (SQUEEZE_X, PY - 0.065, HI), FINGER_CLOSED, "right: off the front half, fingers closed"),
            Waypoint(1.5, (SQUEEZE_X, PY - 0.065, tcp), FINGER_CLOSED, "right: down beside the open front half"),
            Waypoint(3.0, (SQUEEZE_X, SHOVE_Y, tcp), FINGER_CLOSED, "right: push the front half over"),
            Waypoint(0.5, (SQUEEZE_X, SHOVE_Y, tcp), FINGER_CLOSED, "right: hold"),
            Waypoint(1.2, (SQUEEZE_X, PY - 0.065, tcp), FINGER_CLOSED, "right: back off"),
        ]
    else:
        right += [
            Waypoint(1.5, (SQUEEZE_X, PY, HI), FINGER_OPEN, "right: over the frame, fingers open"),
            Waypoint(1.5, (SQUEEZE_X, PY, tcp), FINGER_OPEN, "right: one finger outside each frame half"),
            Waypoint(2.0, (SQUEEZE_X, PY, tcp), FINGER_CLOSED, "right: squeeze the halves together until the balls snap"),
            Waypoint(0.5, (SQUEEZE_X, PY, tcp), FINGER_CLOSED, "right: hold"),
            Waypoint(0.8, (SQUEEZE_X, PY, tcp), FINGER_OPEN, "right: let go"),
        ]
    right += [
        Waypoint(1.0, (right[-1].pos[0], right[-1].pos[1], HI), right[-1].finger, "right: up"),
        Waypoint(1.5, PARK_R, FINGER_OPEN, "right: away"),
        Waypoint(4.0, PARK_R, FINGER_OPEN, "right: wait"),
    ]
    left.append(Waypoint(_dur(right) - _dur(left), PARK_L, FINGER_OPEN, "left: wait"))
    return {"left": left, "right": right}
