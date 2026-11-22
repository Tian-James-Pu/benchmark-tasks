"""The inertia demonstration: an index card lies across the mouth of a juice glass with a quarter on top. Get the quarter
into the glass without touching it or lifting it.

The knack: the card has to leave fast. Paper on a coin grips at a friction coefficient of about 0.35, so the coin can be
given at most about 3.4 m/s^2 by the card. Pulled or pushed gently, the card simply carries the coin away with it. Only a
card that accelerates at several times that, leaving at more than about 1 m/s, slips out from under the coin before
friction has moved it more than a centimetre or two, and the coin then drops straight down into the glass. The robot
cannot lift the coin across, slide it off with a finger, or tip the glass under it: those are recorded and score 0.

Model (declared in code):
- Glass ("glass"): NVIDIA SimReady dishware_glass_8oz_a01 (an 8 oz glass tumbler, 6.6 cm across, 10.0 cm tall), loose,
  150 g, colliding as its real (concave) shape through a signed distance field, so the coin can fall inside it.
- Card ("card"): a ruled 3 x 5 in index card, 127 x 76.2 mm, 0.3 mm thick, 2.0 g, one rigid box.
- Coin ("coin"): a US quarter (United States Mint specification: 24.26 mm, 1.75 mm, 5.67 g), one rigid body with
  speculative CCD so it cannot pass through the glass's 3 mm wall when it falls.
- Friction (static, dynamic; PhysX averages the two parts' values): coin 0.35/0.30, card 0.40/0.35.
- Distractors (SimReady, loose; the scorer ignores them except that none may touch the coin): a notebook, a mug, an
  apple.

Scoring (`Trial` below, every control step at 20 Hz). Positions are in the glass's own frame where they say so.
- Harm, recorded for the trial and cleared only in mechkit's `on_reset`:
  * a finger point (mechkit `fingertips`: each finger's tip and 3 cm up it, middle and both edges) within 2.5 cm of
    the coin's centre (the coin was touched);
  * the coin's centre more than 1.0 cm above its starting height (the coin was lifted, or the card was lifted with it);
  * the glass tipped (axis more than 10 degrees from vertical), slid (base centre more than 2.0 cm from its mark) or
    lifted (base more than 1.0 cm above the table);
  * a distractor's bounding box within 5 mm of the coin's rim.
- The card is *clear* when, seen from above, no part of it is within 3.3 cm of the glass's axis (none of it is over
  the mouth).
- The coin is *in the glass* when its centre is within 3.0 cm of the glass's axis and between the glass's base and
  1.0 cm below its rim.
- *Hands clear*: every finger point at least 5 cm from the glass (its 3.3 cm radius, 10 cm height) and from the coin.
- Full: coin in the glass, card clear, hands clear, glass upright and on its mark, coin and glass still (under 1 cm/s
  and 0.5 rad/s), no harm, for 20 consecutive steps (1 s). Then 1.0 for the rest of the trial unless harm follows.
- Otherwise: harm -> 0; card not clear -> 0; card clear and the coin's centre within 6.0 cm of the glass's axis
  (horizontally) -> 0.5; card clear and the coin farther away -> 0.25.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
import coinkit as ck  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, READY, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

LOOKS = {
    "card": m.Look(color=m.rgb(0.96, 0.96, 0.93), roughness=0.9),
    "rule": m.Look(color=m.rgb(0.45, 0.62, 0.85), roughness=0.9),
    "redrule": m.Look(color=m.rgb(0.85, 0.30, 0.32), roughness=0.9),
    "coin": m.Look(color=m.rgb(0.80, 0.80, 0.82), metallic=0.9, roughness=0.28),
    "glass": m.Look(color=m.rgb(0.80, 0.88, 0.92), roughness=0.08, opacity=0.40),
}

# -- the glass: SimReady 8 oz tumbler, standing on its tape mark ----------------------------------------------------------
# Its mesh (pinned SimReady package dishware_glass_8oz_a01) is drawn as is; it collides as a 4 mm base and 24 wall panels
# that follow the mesh's measured inside and outside radii, so a card lies on a flat rim and the coin can fall inside.
GLASS_XY = np.array([0.360, -0.150])
GLASS_R = 0.0331  # outside radius at the rim
GLASS_RI = 0.0300  # inside radius used for "in the glass"
GLASS_H = 0.1003  # rim height (mesh)
GLASS_MASS = 0.150
GLASS_BASE = 0.004
RIM_Z = TOP + GLASS_H
N_PANELS = 24
_WALL = [(0.004, 0.0255, 0.0311), (0.020, 0.0285, 0.0323), (GLASS_H, 0.0301, 0.0331)]  # (z, inside r, outside r)


def _glass_mesh() -> m.Mesh:
    from simready import fetch

    from pxr import Usd, UsdGeom

    root = fetch("dishware_glass_8oz_a01", "sm_dishware_glass_8oz_a01_01.usd")
    stage = Usd.Stage.Open(str(root.parent / "payloads" / "geometries.usd"))
    verts, faces = [], []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = np.array(mesh.GetPointsAttr().Get(), float)
        counts = np.array(mesh.GetFaceVertexCountsAttr().Get(), int)
        idx = np.array(mesh.GetFaceVertexIndicesAttr().Get(), int)
        base, k = len(verts), 0
        verts.extend(pts.tolist())
        for c in counts:
            poly = idx[k:k + c] + base
            for j in range(1, c - 1):
                faces.append((poly[0], poly[j], poly[j + 1]))
            k += c
    return m.Mesh(np.array(verts), np.array(faces, int))


class _LazyParts:
    """The glass's visual mesh, read from the SimReady file only when the scene is built (after the simulator has started:
    importing pxr before that breaks the simulator's own USD modules)."""

    def __iter__(self):
        return iter([("glass", _glass_mesh(), "glass")])


def _panel(k: int) -> m.Mesh:
    a0, a1 = 2 * math.pi * (k - 0.5) / N_PANELS, 2 * math.pi * (k + 0.5) / N_PANELS
    pts = []
    for z, ri, ro in _WALL:
        for r in (ri, ro / math.cos(math.pi / N_PANELS)):
            for a in (a0, a1):
                pts.append((r * math.cos(a), r * math.sin(a), z))
    return np.array(pts)


_HEX = np.array([(q[0], q[i], q[i + 1]) for q in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
                 for i in (1, 2)], int)


def _glass() -> m.Part:
    cols = [m.col_cylinder("base_c", 0.0311, GLASS_BASE, (0, 0, GLASS_BASE / 2))]
    for k in range(N_PANELS):
        p = _panel(k)
        cols.append(m.col_hull(f"wall{k:02d}_lo_c", m.Mesh(p[:8], _HEX)))
        cols.append(m.col_hull(f"wall{k:02d}_hi_c", m.Mesh(p[4:], _HEX)))
    return m.free("glass", (GLASS_XY[0], GLASS_XY[1], TOP), _LazyParts(), cols, LOOKS, mass=GLASS_MASS,
                  friction=(0.4, 0.4), torsional_patch=0.01, com=(0.0, 0.0, 0.03))


# -- the card: a ruled 3 x 5 in index card, long side along x, centred on the glass --------------------------------------
CARD_L, CARD_W, CARD_T = 0.127, 0.0762, 0.0003
CARD_MASS = 0.0020
CARD_Z = RIM_Z + CARD_T / 2 + 0.0002


def _card() -> m.Part:
    lines = []
    for k in range(9):  # blue rules across the short side, 6.35 mm apart, a red heading rule
        y = -CARD_W / 2 + 0.014 + k * 0.00635
        look = "redrule" if k == 8 else "rule"
        lines.append((f"rule{k}", m.box((CARD_L - 0.004, 0.0005, 0.00005), (0, y, CARD_T / 2 + 0.00003)), look))
    parts = [("sheet", m.box((CARD_L, CARD_W, CARD_T), (0, 0, 0)), "card")] + lines
    cols = [m.col_box("sheet_c", (CARD_L, CARD_W, CARD_T), (0, 0, 0))]
    return m.free("card", (GLASS_XY[0], GLASS_XY[1], CARD_Z), parts, cols, LOOKS, mass=CARD_MASS, friction=(0.40, 0.35),
                  torsional_patch=0.01)


# -- the coin: a US quarter lying flat on the card over the middle of the glass ------------------------------------------
COIN_Z0 = CARD_Z + CARD_T / 2 + ck.COIN_T / 2 + 0.0002


def _coin() -> m.Part:
    mesh = ck.revolve(ck.coin_profile(), 48)
    cols = [m.col_hull("body_c", m.cylinder(ck.COIN_R, ck.COIN_T, (0, 0, 0), segments=32))]
    part = ck.spinning_free("coin", (GLASS_XY[0], GLASS_XY[1], COIN_Z0), [("coin", mesh, "coin")], cols, LOOKS, ck.COIN_MASS,
                            (ck.COIN_I_DIAM, ck.COIN_I_DIAM, ck.COIN_I_AXIS), friction=(0.35, 0.30), ang_damping=0.05)
    return part


_spin_free_file = m._file


def _file_ccd(root, head_api, attrs, body, looks, friction):
    """Speculative CCD on the coin, so a falling coin cannot step through the glass wall in one 1/240 s substep."""
    if root == m._root_name("coin") and "enableSpeculativeCCD" not in attrs:
        attrs = attrs + "\n    bool physxRigidBody:enableSpeculativeCCD = 1"
    return _spin_free_file(root, head_api, attrs, body, looks, friction)


m._file = _file_ccd
ck.collision("coin", contact_offset=0.002, rest_offset=0.0)

# -- distractors (SimReady, CC-BY 4.0) ----------------------------------------------------------------------------------
NOTEBOOK = SimReady("notebook", pos=(0.20, 0.27, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01",
                    usd="sm_misc_notebook_a01_01.usd", yaw=math.pi / 2)
MUG = SimReady("mug", pos=(0.43, 0.17, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
               usd="sm_dishware_mug_a01_01.usda", yaw=math.radians(200))
APPLE = SimReady("apple", pos=(0.50, -0.33, TOP), size=(0.07, 0.07, 0.07), mass=0.18, package="food_produce_apple_b01",
                 usd="sm_food_produce_apple_b01_01.usd")
DISTRACTORS = ("notebook", "mug", "apple")

OBJECTS = [_glass(), _card(), _coin(), NOTEBOOK, MUG, APPLE]

# -- scoring -------------------------------------------------------------------------------------------------------------
TOUCH = 0.025  # finger point to coin centre
LIFT_MAX = 0.010  # coin centre above its start
TILT_MAX, SLIDE_MAX, GLASS_LIFT = 10.0, 0.020, 0.010
NEAR_COIN = 0.005  # distractor box to the coin's rim
CLEAR_R = GLASS_R  # card: no part within this of the glass axis
BELOW_RIM = 0.010
HANDS = 0.05
NEAR = 0.060  # 0.5: coin within this of the glass axis
HOLD_STEPS = 20


def _glass_state(env):
    s = m.state(env, "glass")
    return s, s.rotation(), np.asarray(s.pos, float)  # the part's origin is the centre of its base


def _rect_dist(p_local, half) -> float:
    dx = max(abs(p_local[0]) - half[0], 0.0)
    dy = max(abs(p_local[1]) - half[1], 0.0)
    return math.hypot(dx, dy)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place recorded harm is cleared
        self.harm = set()
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.steps = 0

    def step(self, env):
        self.steps += 1
        coin, card = m.state(env, "coin"), m.state(env, "card")
        gs, Rg, gbase = _glass_state(env)
        tips = m.fingertips(env)

        # harm
        if float(np.min(np.linalg.norm(tips - coin.pos, axis=1))) < TOUCH:
            self.harm.add("coin touched")
        if coin.pos[2] - COIN_Z0 > LIFT_MAX:
            self.harm.add("coin lifted")
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(Rg[2, 2])))))
        if tilt > TILT_MAX:
            self.harm.add("glass tipped")
        if math.hypot(gbase[0] - GLASS_XY[0], gbase[1] - GLASS_XY[1]) > SLIDE_MAX:
            self.harm.add("glass slid")
        if gbase[2] - TOP > GLASS_LIFT:
            self.harm.add("glass lifted")
        from yam_arena.scene import object_states

        for name, s in object_states(env, DISTRACTORS).items():
            d = np.maximum(np.maximum(s.low - coin.pos, coin.pos - s.high), 0.0)
            if float(np.linalg.norm(d)) < ck.COIN_R + NEAR_COIN:
                self.harm.add(f"{name} touched the coin")

        # card over the mouth?
        axis_in_card = card.to_local(np.array([[gbase[0], gbase[1], card.pos[2]]]))[0]
        card_clear = _rect_dist(axis_in_card, (CARD_L / 2, CARD_W / 2)) > CLEAR_R

        # coin in the glass?
        cl = Rg.T @ (coin.pos - gbase)
        coin_r = math.hypot(cl[0], cl[1])
        coin_in = coin_r < GLASS_RI and 0.0 < cl[2] < GLASS_H - BELOW_RIM
        coin_near = math.hypot(coin.pos[0] - gbase[0], coin.pos[1] - gbase[1]) < NEAR

        # hands clear of the glass and the coin
        tl = (tips - gbase) @ Rg
        radial = np.hypot(tl[:, 0], tl[:, 1]) - GLASS_R
        vert = np.maximum(-tl[:, 2], tl[:, 2] - GLASS_H)
        gd = np.sqrt(np.maximum(radial, 0) ** 2 + np.maximum(vert, 0) ** 2)
        cd = np.linalg.norm(tips - coin.pos, axis=1)
        hands = float(np.min(gd)) >= HANDS and float(np.min(cd)) >= HANDS

        still = coin.resting(0.01, 0.5) and bool(np.linalg.norm(gs.lin_vel) < 0.01 and np.linalg.norm(gs.ang_vel) < 0.5)
        ok = coin_in and card_clear and hands and still and not self.harm
        self.hold = self.hold + 1 if ok else 0
        self.full = bool(not self.harm and (self.full or self.hold >= HOLD_STEPS))
        if self.harm:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif not card_clear:
            self.score = 0.0
        else:
            self.score = 0.5 if coin_near else 0.25



TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- scripted demo --------------------------------------------------------------------------------------------------------
# Right arm only, open loop from the known layout. The gripper comes from the robot's side pointing forward and 15 degrees
# down, its jaws one above and one below the card, and pinches the 3 cm of card that overhang the glass on the robot's
# side. It then whips the card out sideways (towards -y), 23 cm in a tenth of a second, lets the card drop on the table
# beside the glass and backs away. The left arm holds its ready pose.


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, Orient) else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


def _roty(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


PITCH = 15.0  # degrees the gripper points down
R_PINCH = _roty(PITCH) @ np.column_stack([(0, 0, 1), (0, -1, 0), (1, 0, 0)]).astype(float)  # pointing +x, jaws above/below
APPROACH = R_PINCH[:, 2]
TIP = 0.013  # fingertips this far beyond the TCP along the approach
TIP_GAP = 0.012  # fingertips stop this far short of the glass wall
PINCH = np.array([GLASS_XY[0] - GLASS_R - TIP_GAP, GLASS_XY[1], CARD_Z]) - TIP * APPROACH
YANK_T = 0.10  # s
YANK = np.array([0.0, -0.23, 0.0])


def _w(t, p, finger, label) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, Orient(R_PINCH))


def _lead_in(pos, finger: float, label: str, seconds: float = 3.0, steps: int = 12) -> list:
    """From the ready pose to pos in a straight line in joint space, as TCP waypoints (keeps one IK branch)."""
    base = np.array(station.BASES["right"])
    q0 = np.array(READY, float)
    best = None
    for seed in (READY, (0.3, 1.2, 0.8, 0.5, 0.9, 0.3), (0.0, 1.0, 1.0, 0.0, 0.0, 0.0), (0.0, 1.5, 1.5, 0.0, 0.0, -1.57)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(R_PINCH), np.asarray(seed, float), base_pos=base, iters=400)
        inside = bool(np.all(q >= ik.LOWER - 1e-6) and np.all(q <= ik.UPPER + 1e-6))
        key = (not inside, pe > 5e-4 or re > 2e-3, float(np.sum(np.abs(q - q0))))
        if best is None or key < best[0]:
            best = (key, q)
    q1 = best[1]
    out = []
    for k in range(1, steps + 1):
        p, R = ik.tcp_pose(q0 + (q1 - q0) * k / steps, base)
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), float(finger), label, Orient(R)))
    return out


def yank_plan(yank_t: float = YANK_T) -> list:
    pre = PINCH - np.array([0.06, 0.0, 0.0])
    end = PINCH + YANK
    high = pre + np.array([-0.04, 0.0, 0.12])
    return _lead_in(high, FINGER_OPEN, "above and behind the card") + [
        _w(1.5, pre, FINGER_OPEN, "down behind the card"),
        _w(0.5, pre, FINGER_OPEN, "settle"),
        _w(1.2, PINCH, FINGER_OPEN, "jaws round the overhang"),
        _w(0.6, PINCH, FINGER_CLOSED, "pinch"),
        _w(0.5, PINCH, FINGER_CLOSED, "steady"),
        _w(yank_t, end, FINGER_CLOSED, "whip the card out sideways"),
        _w(0.6, end, FINGER_CLOSED, "stop"),
        _w(0.5, end, FINGER_OPEN, "drop the card"),
        _w(1.2, end + np.array([0.0, 0.0, 0.08]), FINGER_OPEN, "back away"),
        _w(6.0, end + np.array([0.0, 0.0, 0.08]), FINGER_OPEN, "wait"),
    ]


def _left_wait(total: float) -> list:
    lp, lr = ik.tcp_pose(np.array(READY, float), np.array(station.BASES["left"]))
    return [Waypoint(total, tuple(float(v) for v in lp), FINGER_OPEN, "wait", Orient(lr))]


def demo():
    right = yank_plan()
    return {"left": _left_wait(sum(w.duration for w in right)), "right": right}
