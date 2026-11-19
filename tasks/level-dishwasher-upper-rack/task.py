"""A dishwasher's loaded upper rack, pulled out about 30 cm on its runners, left higher on one side: its right height adjuster
is in the top notch and its left one in the bottom notch, so the rack is tilted about 6 degrees across. The rack has a
height adjuster on each side (as on Bosch RackMatic and similar racks): a grey grip on top of the side rail with a lever on
its outer face. Pushing the lever inward frees that side to move down or up; letting go latches the side in the notch at or
below it. A side can also be lifted into the next notch up without the lever. The rack docks on the water inlet in the
tub's back wall only when both sides sit at the same height. The rack holds two tumblers and a mug, upside down.

Model (declared in code):
- "dishwasher": a mechanism. Frame: the tub's upper section (stainless side walls, ceiling, back wall with the inlet
  manifold) and its runners. Links: "rack" (the rack's runners, a slider along x, 0 to 30 cm: 30 cm is docked);
  "lift" (the left adjuster, a slider along z, 0 to 5 cm); "basket" (the wire rack itself, a hinge along x through the
  left adjuster line, so the right side's height follows from the tilt); "lever_l", "lever_r" (the two levers,
  sprung sliders along y, 8 mm travel, inward).
- The adjusters' notches (code, each control step): a latched side is held at its notch by a stiff spring (4000 N/m);
  a side lifted more than 60% of the way to the next notch clicks into it; a side whose lever is pressed in more than 4
  mm is free (held only by joint damping) between the lowest and highest notch; on letting go it latches in the notch at
  or below it (within 4 mm). Notches 0, 2.5 and 5.0 cm.
- The rack's weight is not on the adjusters (links have no gravity): a freed side stays where it is until pushed.
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
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the upper rack back into the dishwasher so it is ready to run."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the rack (all heights are above the table top; the mechanism's origin is (0, 0, TOP))
W = 0.50  # rack width: the distance between the two adjuster lines (the side walls)
D = 0.46  # rack depth
WALL = 0.10  # side and end wall height above the rack floor
X0 = 0.13  # the rack's front wall, pulled out
TRAVEL = 0.30  # runner travel from pulled out to docked
FLOOR0 = 0.015  # the rack floor's top at the left side, adjuster in the bottom notch
STEP = 0.025  # adjuster notch pitch
NOTCHES = 3
START_L, START_R = 0, 2  # the starting notches
PHI0 = -math.asin(START_R * STEP / W)  # the authored tilt about +x (right side up)
PIVOT = np.array([0.0, W / 2, FLOOR0])  # the tilt axis: along x through the left adjuster line, at floor height
HANDLE_X = (X0 + 0.06, X0 + 0.15)  # the adjuster grips along the side rails
HANDLE_T, HANDLE_H = 0.014, 0.035  # grip thickness (y) and height above the top rail
LEVER_GAP, LEVER_T, LEVER_H = 0.008, 0.006, 0.030  # lever's gap to the grip, thickness, height
LEVER_TRAVEL = 0.008
PRESSED = 0.004  # a lever pressed in further than this frees its side
K_LATCH = 4000.0  # N/m at the side
K_TILT = K_LATCH * W * W  # N m/rad: the same stiffness felt at the right side
K_LEVER = 400.0
CLICK = 0.6  # of a notch pitch, lifted without the lever: clicks into the next notch up
DROP_TOL = 0.004  # on letting go, the side latches in the notch at or below (this far above a notch still counts as at it)

# -- the tub (the dishwasher's upper section) and the counter over it
TUB_X0, TUB_X1 = 0.45, 0.93
TUB_Y = 0.28  # inner half width
TUB_H = 0.40  # inner height above the table
COUNTER_Z = 0.41

# -- scoring tolerances
NOTCH_TOL = 0.003  # a side counts as at a notch within 3 mm of it
DOCK_TOL = 0.005  # docked: within 5 mm of the runners' end stop
CLEAR = 0.03  # every fingertip point at least 3 cm from the rack (with grips and levers)

LOOKS = {
    "steel": m.Look(color=m.rgb(0.72, 0.73, 0.74), metallic=0.9, roughness=0.35),
    "wire": m.Look(color=m.rgb(0.78, 0.79, 0.80), roughness=0.45),
    "grip": m.Look(color=m.rgb(0.32, 0.33, 0.35), roughness=0.5),
    "lever": m.Look(color=m.rgb(0.55, 0.57, 0.60), roughness=0.45),
    "runner": m.Look(color=m.rgb(0.60, 0.61, 0.63), metallic=0.7, roughness=0.4),
    "inlet": m.Look(color=m.rgb(0.20, 0.21, 0.23), roughness=0.5),
    "counter": m.Look(color=m.rgb(0.55, 0.53, 0.50), roughness=0.7),
}


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def tilted(mesh: m.Mesh) -> m.Mesh:
    """A mesh built level (rack floor at FLOOR0) turned to the authored start tilt about the left adjuster line."""
    v = (mesh.v - PIVOT) @ _rx(PHI0).T + PIVOT
    return m.Mesh(v, mesh.f)


def _bx(size, center) -> m.Mesh:
    return m.box(size, center)


# -- the wire rack, built level, then tilted to its start pose --------------------------------------------------------------------------
Y_L, Y_R = W / 2, -W / 2
F = FLOOR0


def _rack_visual() -> m.Mesh:
    t = 0.004  # wire thickness
    r = 0.007  # rail thickness
    pieces = []
    # floor wires: along x every 2.5 cm, across every 5 cm
    for y in np.arange(Y_R + 0.025, Y_L - 0.01, 0.025):
        pieces.append(_bx((D, t, t), (X0 + D / 2, y, F - t / 2)))
    for x in np.arange(X0 + 0.05, X0 + D - 0.01, 0.05):
        pieces.append(_bx((t, W, t), (x, 0.0, F - t / 2)))
    # top and bottom rails round the rim
    for z in (F, F + WALL):
        pieces += [_bx((D, r, r), (X0 + D / 2, Y_L - r / 2, z)), _bx((D, r, r), (X0 + D / 2, Y_R + r / 2, z)),
                   _bx((r, W, r), (X0 + r / 2, 0.0, z)), _bx((r, W, r), (X0 + D - r / 2, 0.0, z))]
    # upright wires in the walls every 3 cm, and cup tines down the middle (two rows of 9 cm prongs)
    for x in np.arange(X0 + 0.03, X0 + D - 0.01, 0.03):
        for y in (Y_L - r / 2, Y_R + r / 2):
            pieces.append(_bx((t, t, WALL), (x, y, F + WALL / 2)))
    for y in np.arange(Y_R + 0.03, Y_L - 0.01, 0.03):
        for x in (X0 + r / 2, X0 + D - r / 2):
            pieces.append(_bx((t, t, WALL), (x, y, F + WALL / 2)))
    # the spray-arm feed coupling at the back, centre
    pieces.append(m.cylinder(0.012, 0.03, (X0 + D + 0.01, 0.0, F + 0.012), axis="x", segments=16))
    return tilted(m.join(*pieces))


def _rack_colliders():
    th = 0.005
    boxes = {
        "floor_c": ((D, W, th), (X0 + D / 2, 0.0, F - th / 2)),
        "front_c": ((0.006, W, WALL), (X0 + 0.003, 0.0, F + WALL / 2)),
        "back_c": ((0.006, W, WALL), (X0 + D - 0.003, 0.0, F + WALL / 2)),
        "left_c": ((D, 0.006, WALL), (X0 + D / 2, Y_L - 0.003, F + WALL / 2)),
        "right_c": ((D, 0.006, WALL), (X0 + D / 2, Y_R + 0.003, F + WALL / 2)),
    }
    return [m.col_hull(n, tilted(_bx(s, c))) for n, (s, c) in boxes.items()]


def _grip_box(side: float):
    """An adjuster grip on top of the side rail: (size, centre), level."""
    y = Y_L - 0.003 if side > 0 else Y_R + 0.003
    return ((HANDLE_X[1] - HANDLE_X[0], HANDLE_T, HANDLE_H), ((HANDLE_X[0] + HANDLE_X[1]) / 2, y, F + WALL + HANDLE_H / 2 - 0.003))


def _lever_box(side: float):
    """The adjuster lever beside the grip's outer face: (size, centre), level, released."""
    y_grip = _grip_box(side)[1][1]
    y = y_grip + side * (HANDLE_T / 2 + LEVER_GAP + LEVER_T / 2)
    return ((HANDLE_X[1] - HANDLE_X[0] - 0.01, LEVER_T, LEVER_H), ((HANDLE_X[0] + HANDLE_X[1]) / 2, y, F + WALL + LEVER_H / 2))


def _basket_link():
    parts = [("rack", _rack_visual(), "wire")]
    cols = _rack_colliders()
    for side, tag in ((1.0, "l"), (-1.0, "r")):
        s, c = _grip_box(side)
        parts.append((f"grip_{tag}", tilted(_bx(s, c)), "grip"))
        cols.append(m.col_hull(f"grip_{tag}_c", tilted(_bx(s, c))))
    return m.hinge("basket", "X", tuple(PIVOT), -1.2, 12.6, parts, cols, mass=2.5, spring=K_TILT, damping=8.0, friction=0.05, effort=3000.0,
                   armature=0.01, parent="lift", inertia=(0.08, 0.06, 0.12))


def _lever_link(side: float, tag: str):
    s, c = _lever_box(side)
    lo, hi = (-LEVER_TRAVEL, 0.0) if side > 0 else (0.0, LEVER_TRAVEL)  # inward: -y on the left, +y on the right
    at = tuple((np.asarray(c) - PIVOT) @ _rx(PHI0).T + PIVOT)
    return m.slider(f"lever_{tag}", "Y", lo, hi, [(f"lever_{tag}", tilted(_bx(s, c)), "lever")], [m.col_hull(f"lever_{tag}_c", tilted(_bx(s, c)))],
                    at=at, mass=0.03, spring=K_LEVER, damping=2.0, friction=0.01, effort=100.0, parent="basket", inertia=(2e-5, 2e-5, 2e-5))


def _runner_link():
    # the rack's own runner rails and wheels along both sides, under the side rails (shown only; they ride inside the tub's runners)
    parts = []
    for y in (Y_L + 0.008, Y_R - 0.008):
        parts.append((f"runner_{'l' if y > 0 else 'r'}", m.box((D, 0.008, 0.012), (X0 + D / 2, y, F + 0.02)), "runner"))
    return m.slider("rack", "X", 0.0, TRAVEL, parts, [], mass=0.6, spring=0.0, damping=10.0, friction=3.0, effort=100.0, inertia=(0.02, 0.02, 0.02))


def _lift_link():
    return m.slider("lift", "Z", 0.0, (NOTCHES - 1) * STEP, [], [], at=tuple(PIVOT), mass=0.3, spring=K_LATCH, damping=40.0, friction=0.3, effort=3000.0,
                    parent="rack", inertia=(0.01, 0.01, 0.01))


def _tub():
    parts, cols = [], []
    wall = 0.01
    for tag, y in (("l", TUB_Y + wall / 2), ("r", -TUB_Y - wall / 2)):
        s, c = (TUB_X1 - TUB_X0, wall, TUB_H), ((TUB_X0 + TUB_X1) / 2, y, TUB_H / 2)
        parts.append((f"wall_{tag}", _bx(s, c), "steel"))
        cols.append(m.col_box(f"wall_{tag}_c", s, c))
        # the tub's runner tracks (shown)
        parts.append((f"track_{tag}", _bx((TUB_X1 - TUB_X0 - 0.02, 0.012, 0.02), ((TUB_X0 + TUB_X1) / 2, math.copysign(TUB_Y - 0.006, y), F + 0.02)), "runner"))
    s, c = (TUB_X1 - TUB_X0, 2 * TUB_Y + 2 * wall, wall), ((TUB_X0 + TUB_X1) / 2, 0.0, TUB_H + wall / 2)
    parts.append(("ceiling", _bx(s, c), "steel"))
    cols.append(m.col_box("ceiling_c", s, c))
    s, c = (wall, 2 * TUB_Y + 2 * wall, TUB_H + wall), (TUB_X1 + wall / 2, 0.0, (TUB_H + wall) / 2)
    parts.append(("back", _bx(s, c), "steel"))
    cols.append(m.col_box("back_c", s, c))
    # the water inlet manifold on the back wall: a vertical channel the rack's coupling meets at any of its three heights
    parts.append(("inlet", _bx((0.02, 0.05, 0.11), (TUB_X1 - 0.01, 0.0, F + 0.05)), "inlet"))
    # the tub's front flange
    for tag, y in (("l", TUB_Y + 0.015), ("r", -TUB_Y - 0.015)):
        parts.append((f"flange_{tag}", _bx((0.01, 0.03, TUB_H + 0.01), (TUB_X0 + 0.005, y, (TUB_H + 0.01) / 2)), "steel"))
    return parts, cols


_tp, _tc = _tub()
DISHWASHER = m.mechanism("dishwasher", (0.0, 0.0, TOP), [_runner_link(), _lift_link(), _basket_link(), _lever_link(1.0, "l"), _lever_link(-1.0, "r")],
                         LOOKS, frame_parts=_tp, frame_colliders=_tc, friction=(1.0, 0.8), solver_iters=48)

_cs, _cc = (0.52, 0.64, 0.03), (TUB_X0 + 0.25, 0.0, COUNTER_Z + 0.015)
COUNTER = m.fixed("counter", (0.0, 0.0, TOP), [("top", _bx(_cs, _cc), "counter")], [m.col_box("top_c", _cs, _cc)], LOOKS, friction=(0.9, 0.8))

# -- the load: SimReady glassware and a mug, upside down in the rack (the props' origins are the centre of their base, now at the top)
FLIP = (1.0, 0.0, 0.0, 0.0)  # xyzw: half a turn about x


def floor_z(y: float) -> float:
    """The tilted rack floor's height at y, above the table (start pose)."""
    return FLOOR0 + (Y_L - y) * math.tan(-PHI0)


LOAD = {
    "tumbler_a": ((X0 + 0.12, 0.10), 0.085, "dishware_glass_tumbler_a01", "sm_dishware_glass_tumbler_a01_01.usd", 0.25),
    "tumbler_b": ((X0 + 0.20, -0.09), 0.085, "dishware_glass_tumbler_a01", "sm_dishware_glass_tumbler_a01_01.usd", 0.25),
    "mug": ((X0 + 0.33, 0.02), 0.080, "dishware_mug_a01", "sm_dishware_mug_a01_01.usda", 0.35),
}
LOAD_OBJECTS = [SimReady(n, pos=(x, y, TOP + floor_z(y) + h + 0.003), size=(0.08, 0.08, h), mass=mass, package=pkg, usd=usd, quat_xyzw=FLIP, convex=True)
                for n, ((x, y), h, pkg, usd, mass) in LOAD.items()]

# -- distractors (SimReady, loose) on the counter over the dishwasher
SOAP = SimReady("dish_soap", pos=(0.52, 0.19, TOP + COUNTER_Z + 0.03), size=(0.066, 0.114, 0.271), mass=0.45, package="cleaning_dishSoap_a01",
                usd="sm_cleaning_dishSoap_a01_01.usd", yaw=0.4)
SPONGE = SimReady("sponge", pos=(0.51, -0.18, TOP + COUNTER_Z + 0.03), size=(0.071, 0.101, 0.033), mass=0.02, package="cleaning_sponge_b01",
                  usd="sm_cleaning_sponge_b01_01.usd", yaw=0.3)
COFFEE_MUG = SimReady("coffee_mug", pos=(0.53, 0.01, TOP + COUNTER_Z + 0.03), size=(0.092, 0.137, 0.092), mass=0.33, package="dishware_mug_b01",
                      usd="sm_dishware_mug_b01_01.usda", yaw=2.2)

OBJECTS = [DISHWASHER, COUNTER, *LOAD_OBJECTS, SOAP, SPONGE, COFFEE_MUG]


# -- the adjusters' notches -------------------------------------------------------------------------------------------------------------


def side_heights(s) -> tuple[float, float]:
    """(left, right) side heights above their bottom notch (m), from the lift and tilt joints."""
    lh = s.joint("lift")
    return lh, lh - W * math.sin(PHI0 + s.joint("basket"))


def levers_in(s) -> tuple[float, float]:
    """How far each lever is pressed in (m)."""
    return -s.joint("lever_l"), s.joint("lever_r")


def tilt_for(h_right: float, lh: float) -> float:
    """The tilt joint value that puts the right side at h_right with the left side at lh."""
    return math.asin(max(-0.99, min(0.99, -(h_right - lh) / W))) - PHI0


def _drive(env, link, target, stiffness):
    asset, j = m._joint(env, "dishwasher", link)
    m._TOUCHED.add(("dishwasher", link))
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


HI = (NOTCHES - 1) * STEP


class Adjusters:
    """The two height adjusters' pawls and notches."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.notch = [START_L, START_R]
        self.free = [False, False]

    def step(self, env):
        s = m.state(env, "dishwasher")
        h = side_heights(s)
        pressed = [p > PRESSED for p in levers_in(s)]
        for i in (0, 1):
            if pressed[i]:
                self.free[i] = True
            elif self.free[i]:  # let go: the pawl drops into the notch at or below
                self.free[i] = False
                self.notch[i] = int(min(NOTCHES - 1, max(0, math.floor((h[i] + DROP_TOL) / STEP))))
            elif self.notch[i] < NOTCHES - 1 and h[i] > (self.notch[i] + CLICK) * STEP:  # lifted: clicks into the next notch up
                self.notch[i] += 1
        lh = h[0]
        # left side: the lift joint
        if self.free[0]:
            _drive(env, "lift", 0.0, 0.0)
        else:
            _drive(env, "lift", self.notch[0] * STEP, K_LATCH)
        # right side: the tilt joint, re-aimed each step so the right side keeps its own height whatever the left does
        if self.free[1]:
            if h[1] < -0.001:
                _drive(env, "basket", tilt_for(0.0, lh), K_TILT)
            elif h[1] > HI + 0.001:
                _drive(env, "basket", tilt_for(HI, lh), K_TILT)
            else:
                _drive(env, "basket", tilt_for(h[1], lh), 0.0)
        else:
            _drive(env, "basket", tilt_for(self.notch[1] * STEP, lh), K_TILT)


ADJ = Adjusters()
m.on_step(ADJ.step)
m.on_reset(ADJ.reset)


# -- the scorer ------------------------------------------------------------------------------------------------------------------------


def to_rack(s, pts) -> np.ndarray:
    """World points into the level rack's own frame: x along the rack (front wall at X0), y across (left wall at W/2), z up from its floor."""
    p = np.atleast_2d(np.asarray(pts, float)) - np.asarray(s.pos)  # the mechanism's frame is fixed, unturned
    p = p - np.array([s.joint("rack"), 0.0, s.joint("lift")]) - PIVOT
    p = p @ _rx(-(PHI0 + s.joint("basket"))).T
    return p + np.array([0.0, W / 2, 0.0])


RACK_BOX = (np.array([X0, Y_R - 0.03, -0.01]), np.array([X0 + D, Y_L + 0.03, WALL + HANDLE_H]))


def rack_gap(s, tips) -> float:
    q = to_rack(s, tips)
    d = np.maximum(np.maximum(RACK_BOX[0] - q, q - RACK_BOX[1]), 0.0)
    return float(np.min(np.linalg.norm(d, axis=1)))


def notch_at(h: float):
    n = int(round(h / STEP))
    return n if 0 <= n < NOTCHES and abs(h - n * STEP) <= NOTCH_TOL else None


def load_in_rack(s, env) -> bool:
    from yam_arena.scene import object_states
    st = object_states(env, tuple(LOAD))
    q = to_rack(s, np.array([st[n].pos for n in LOAD]))
    inside = (q[:, 0] > X0 - 0.01) & (q[:, 0] < X0 + D + 0.01) & (q[:, 1] > Y_R - 0.01) & (q[:, 1] < Y_L + 0.01) & (q[:, 2] > -0.03) & (q[:, 2] < 0.25)
    return bool(np.all(inside))


def now(env) -> dict:
    s = m.state(env, "dishwasher")
    hl, hr = side_heights(s)
    nl, nr = notch_at(hl), notch_at(hr)
    pl, pr = levers_in(s)
    released = pl < 0.002 and pr < 0.002
    gap = rack_gap(s, m.fingertips(env))
    return {"hl": hl, "hr": hr, "nl": nl, "nr": nr, "released": released, "level": nl is not None and nl == nr,
            "one_apart": nl is not None and nr is not None and abs(nl - nr) == 1, "docked": s.joint("rack") >= TRAVEL - DOCK_TOL,
            "slide": s.joint("rack"), "clear": gap >= CLEAR, "gap": gap, "load_ok": load_in_rack(s, env)}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.spilled = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.k = 0

    def step(self, env):
        c = now(env)
        if not c["load_ok"]:
            self.spilled = True
        ok = c["level"] and c["released"] and c["docked"] and c["clear"] and not self.spilled
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.spilled:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["level"] and c["released"]:
            self.score = 0.5
        elif c["one_apart"] and c["released"]:
            self.score = 0.25
        else:
            self.score = 0.0
        self.k += 1
        if DEBUG and self.k % 10 == 1:
            print(f"[dw] k {self.k} hl {c['hl']*1000:.1f} hr {c['hr']*1000:.1f} n {c['nl']},{c['nr']} adj {ADJ.notch} free {ADJ.free} rel {c['released']} "
                  f"slide {c['slide']*1000:.1f} gap {c['gap']*1000:.0f} load {c['load_ok']} score {self.score}", flush=True)


DEBUG = False
TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published starting layout) ----------------------------------------------------------------
OPEN = -0.030  # finger opening for the approach (about 7 cm between the pads)


def _right_level_and_dock():
    """Right arm: pinch the right adjuster's grip and lever together (pressing the lever in), push that side down two notches,
    let go (it latches in the bottom notch, level with the left), then push the rack in by its front wall until it docks."""
    gx = (HANDLE_X[0] + HANDLE_X[1]) / 2
    gy = Y_R  # midway across the grip and the pressed-in lever
    z_hi = TOP + FLOOR0 + START_R * STEP + WALL + 0.022  # the TCP with the pads over the grip's upper 3 cm (right side up)
    down = START_R * STEP + 0.004
    push_z = TOP + FLOOR0 + 0.065
    return [
        Waypoint(1.5, (gx, gy, TOP + 0.215), OPEN, "over the right adjuster, fingers either side"),
        Waypoint(1.2, (gx, gy, z_hi), OPEN, "pads either side of grip and lever"),
        Waypoint(0.8, (gx, gy, z_hi), 0.0, "squeeze: lever in"),
        Waypoint(0.4, (gx, gy, z_hi), 0.0, "hold"),
        Waypoint(2.5, (gx, gy, z_hi - down), 0.0, "push the right side down two notches"),
        Waypoint(0.5, (gx, gy, z_hi - down), 0.0, "hold"),
        Waypoint(0.6, (gx, gy, z_hi - down), OPEN, "let go: it latches"),
        Waypoint(1.0, (gx, gy, TOP + 0.20), OPEN, "up"),
        Waypoint(1.5, (0.10, -0.10, TOP + 0.17), 0.0, "in front of the rack"),
        Waypoint(1.0, (0.10, -0.10, push_z), 0.0, "down in front of the front wall"),
        Waypoint(3.0, (X0 + TRAVEL - 0.018 + 0.006, -0.10, push_z), 0.0, "push the rack in to its stop"),
        Waypoint(0.4, (X0 + TRAVEL - 0.018 + 0.006, -0.10, push_z), 0.0, "hold"),
        Waypoint(0.8, (X0 + TRAVEL - 0.04, -0.10, push_z), 0.0, "back off"),
        Waypoint(1.0, (X0 + TRAVEL - 0.04, -0.10, TOP + 0.15), 0.0, "up"),
        Waypoint(1.2, (0.25, -0.20, TOP + 0.20), 0.0, "clear"),
        Waypoint(4.0, (0.25, -0.20, TOP + 0.20), 0.0, "wait"),
    ]


def demo():
    return {"right": _right_level_and_dock()}
