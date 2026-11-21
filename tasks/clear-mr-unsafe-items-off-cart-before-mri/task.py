"""Get an MR Conditional supply cart ready to go into the MRI scanner room (MRI Zone IV).

The bench stands for the floor of Zone III, just outside the scanner room. On it (positions are x forward from the arm bases, y to the
left, in metres; heights above the bench top):
- The cart's top shelf: a grey PVC tray, 61 cm deep (x 0.24 to 0.85) and 45.7 cm wide (y -0.229 to 0.229), its floor 2.5 cm thick with
  a 2.5 cm lip all round. A yellow triangular MR Conditional label is on its front lip. The cart is MR Conditional; what is on it is not
  all safe to take in.
- On the shelf, within reach: a smartphone lying screen up (centre (0.33, -0.155)); an otoscope (battery handle and head) lying on its
  side, handle along x, head towards the robot (x 0.27 to 0.46, y about -0.03); a box of nitrile examination gloves (x 0.35 to 0.47,
  y 0.03 to 0.21) with a pair of stainless-steel surgical scissors lying on it, blades along the box and finger rings over its front edge
  (x 0.30 to 0.47, y 0.12); a cotton gauze roll at (0.46, -0.14).
- Further back on the shelf: a 500 ml plastic water bottle standing at (0.54, 0.10).
- On the bench either side of the cart, two rectangles of red floor tape (x 0.12 to 0.42; y 0.25 to 0.43 on the left, -0.43 to -0.25
  on the right): the hold areas where items that may not enter the scanner room are left in Zone III.
- Behind the right hold area, a free-standing red warning sign on an easel, its raised white letters reading
  "MRI ZONE IV / MAGNET / ALWAYS ON".
- Also on the bench, off the cart: a hand-sanitiser pump bottle (0.80, 0.38) and a paper cup (0.82, -0.32).

Scoring (see README.md): each of the three MR Unsafe items (phone, scissors, otoscope) is cleared when every part of it lies inside one of
the taped hold areas (out to the tape's outer edge), its lowest point at most 1.2 cm above the bench, still, with every fingertip point
of both grippers at least 3 cm from it. 0.25 per cleared item. 1.0 when all three are cleared and the glove box, gauze roll and water
bottle are all still on the cart's shelf, held for 3 s. While any of those three is off the shelf the score is at most 0.5. An MR Unsafe
item that goes over the bench's edge to the floor scores 0 for the trial.
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
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
INSTRUCTION = "Get the cart ready for the patient's MRI scan."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the tracker below counts its own hold
HOLD_STEPS = 60  # 3 s at 20 Hz

LOOKS = {
    "pvc": m.Look(color=m.rgb(0.36, 0.38, 0.41), roughness=0.6),
    "pvc_dark": m.Look(color=m.rgb(0.26, 0.27, 0.30), roughness=0.6),
    "mr_yellow": m.Look(color=m.rgb(0.98, 0.80, 0.05), roughness=0.4),
    "mr_black": m.Look(color=m.rgb(0.02, 0.02, 0.02), roughness=0.4),
    "tape_red": m.Look(color=m.rgb(0.80, 0.06, 0.05), roughness=0.5),
    "sign_red": m.Look(color=m.rgb(0.78, 0.05, 0.05), roughness=0.4),
    "sign_white": m.Look(color=m.rgb(0.98, 0.98, 0.98), roughness=0.3, emissive=m.rgb(0.6, 0.6, 0.6)),
    "frame_grey": m.Look(color=m.rgb(0.25, 0.25, 0.27), roughness=0.5),
    "phone_case": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.5),
    "phone_glass": m.Look(color=m.rgb(0.02, 0.02, 0.03), roughness=0.1, metallic=0.2),
    "phone_lens": m.Look(color=m.rgb(0.35, 0.35, 0.38), roughness=0.2, metallic=0.8),
}

# -- the cart's top shelf (AliMed MRI Nonmagnetic Utility Cart footprint: 18 in wide, 24 in deep)
CART_X0, CART_X1, CART_HY = 0.24, 0.85, 0.2286
FLOOR_T, LIP_H, LIP_W = 0.025, 0.025, 0.015
SHELF_Z = TOP + FLOOR_T
INNER_X = (CART_X0 + LIP_W, CART_X1 - LIP_W)
INNER_HY = CART_HY - LIP_W


def _cart():
    """The shelf's floor and lip are plain fixed boxes (they collide); the MR Conditional label is a seen-only kit part."""
    cx, lx = (CART_X0 + CART_X1) / 2, CART_X1 - CART_X0
    grey, dark = (0.36, 0.38, 0.41), (0.26, 0.27, 0.30)
    boxes = [Box("cart_shelf", pos=(cx, 0.0, TOP + FLOOR_T / 2), size=(lx, 2 * CART_HY, FLOOR_T), color=grey)]
    for n, (sz, c) in {
        "cart_lip_front": ((LIP_W, 2 * CART_HY, LIP_H), (CART_X0 + LIP_W / 2, 0, FLOOR_T + LIP_H / 2)),
        "cart_lip_back": ((LIP_W, 2 * CART_HY, LIP_H), (CART_X1 - LIP_W / 2, 0, FLOOR_T + LIP_H / 2)),
        "cart_lip_left": ((lx, LIP_W, LIP_H), (cx, CART_HY - LIP_W / 2, FLOOR_T + LIP_H / 2)),
        "cart_lip_right": ((lx, LIP_W, LIP_H), (cx, -CART_HY + LIP_W / 2, FLOOR_T + LIP_H / 2)),
    }.items():
        boxes.append(Box(n, pos=(c[0], c[1], TOP + c[2]), size=sz, color=dark))
    # the yellow MR Conditional triangle (ASTM F2503) on the front lip's outer face, 4 cm a side
    tri = m.prism(np.array([(-0.02, 0.0), (0.02, 0.0), (0.0, 0.0346)]), 0.0, 0.0008)  # in (y, z, -x)
    v = tri.v
    label = m.Mesh(np.c_[CART_X0 - v[:, 2] - 0.0002, v[:, 0] + 0.12, v[:, 1] + FLOOR_T + 0.0005], tri.f)
    return boxes + [m.fixed("cart_label", (0.0, 0.0, TOP), [("mr_label", label, "mr_yellow")], (), LOOKS)]


# -- hold areas: red floor tape on the bench (seen, not collided with)
HOLD_X = (0.12, 0.42)
HOLD_Y = {"left": (0.25, 0.43), "right": (-0.43, -0.25)}
TAPE_W = 0.025


def _tape():
    parts = []
    for side, (y0, y1) in HOLD_Y.items():
        x0, x1 = HOLD_X
        for n, sz, c in (("n", (x1 - x0 + TAPE_W, TAPE_W, 0.001), ((x0 + x1) / 2, y0, 0.0005)),
                         ("f", (x1 - x0 + TAPE_W, TAPE_W, 0.001), ((x0 + x1) / 2, y1, 0.0005)),
                         ("a", (TAPE_W, y1 - y0, 0.001), (x0, (y0 + y1) / 2, 0.0005)),
                         ("b", (TAPE_W, y1 - y0, 0.001), (x1, (y0 + y1) / 2, 0.0005))):
            parts.append((f"tape_{side}_{n}", m.box(sz, c), "tape_red"))
    return m.fixed("hold_tape", (0.0, 0.0, TOP), parts, (), LOOKS)


def hold_box(side):
    """A hold area as an xy box out to the tape's outer edges."""
    (x0, x1), (y0, y1) = HOLD_X, HOLD_Y[side]
    return np.array([x0 - TAPE_W / 2, y0 - TAPE_W / 2]), np.array([x1 + TAPE_W / 2, y1 + TAPE_W / 2])


# -- the Zone IV warning sign: raised stroke lettering on a red board, on an easel, tilted back to face the room
GLYPHS = {
    "Z": [(0, 6, 4, 6), (4, 6, 0, 0), (0, 0, 4, 0)],
    "O": [(1, 0, 3, 0), (3, 0, 4, 1), (4, 1, 4, 5), (4, 5, 3, 6), (3, 6, 1, 6), (1, 6, 0, 5), (0, 5, 0, 1), (0, 1, 1, 0)],
    "N": [(0, 0, 0, 6), (0, 6, 4, 0), (4, 0, 4, 6)],
    "E": [(0, 0, 0, 6), (0, 6, 4, 6), (0, 3, 3, 3), (0, 0, 4, 0)],
    "I": [(2, 0, 2, 6), (1, 6, 3, 6), (1, 0, 3, 0)],
    "V": [(0, 6, 2, 0), (2, 0, 4, 6)],
    "M": [(0, 0, 0, 6), (0, 6, 2, 2.5), (2, 2.5, 4, 6), (4, 6, 4, 0)],
    "R": [(0, 0, 0, 6), (0, 6, 3, 6), (3, 6, 4, 5), (4, 5, 4, 4), (4, 4, 3, 3), (3, 3, 0, 3), (2, 3, 4, 0)],
    "A": [(0, 0, 2, 6), (2, 6, 4, 0), (1, 2.5, 3, 2.5)],
    "G": [(4, 5, 3, 6), (3, 6, 1, 6), (1, 6, 0, 5), (0, 5, 0, 1), (0, 1, 1, 0), (1, 0, 3, 0), (3, 0, 4, 1), (4, 1, 4, 3), (4, 3, 2, 3)],
    "T": [(0, 6, 4, 6), (2, 6, 2, 0)],
    "L": [(0, 6, 0, 0), (0, 0, 4, 0)],
    "W": [(0, 6, 1, 0), (1, 0, 2, 4), (2, 4, 3, 0), (3, 0, 4, 6)],
    "Y": [(0, 6, 2, 3), (4, 6, 2, 3), (2, 3, 2, 0)],
    "S": [(4, 5, 3, 6), (3, 6, 1, 6), (1, 6, 0, 5), (0, 5, 0, 4), (0, 4, 1, 3), (1, 3, 3, 3), (3, 3, 4, 2), (4, 2, 4, 1), (4, 1, 3, 0),
          (3, 0, 1, 0), (1, 0, 0, 1)],
}
SIGN_W, SIGN_H, SIGN_T = 0.46, 0.30, 0.006
SIGN_TILT = 49.0  # degrees back from vertical
SIGN_POS = (0.57, -0.47)
SIGN_YAW = -28.0  # its face turned towards the room (and the camera)
SIGN_BASE_Z = 0.03
LINES = (("MRI ZONE IV", 0.040, 0.235), ("MAGNET", 0.060, 0.135), ("ALWAYS ON", 0.040, 0.045))  # text, letter height, baseline v


def _stroke(x0, y0, x1, y1, w, depth):
    """A raised stroke from (x0, y0) to (x1, y1) in the board's (u, v) plane, standing `depth` proud of it."""
    L = math.hypot(x1 - x0, y1 - y0)
    b = m.box((L + w, w, depth), (0, 0, depth / 2))
    return b.turned(math.degrees(math.atan2(y1 - y0, x1 - x0)), "z").moved(((x0 + x1) / 2, (y0 + y1) / 2, 0))


def _text_mesh():
    meshes = []
    for text, h, base in LINES:
        s = h / 6.0
        w = 0.18 * h
        pitch = 0.92 * h
        u0 = -pitch * len(text) / 2 + (pitch - 4 * s) / 2
        for k, ch in enumerate(text):
            for (a, b, c, d) in GLYPHS.get(ch, []):
                ux = u0 + k * pitch
                meshes.append(_stroke(ux + a * s, base + b * s, ux + c * s, base + d * s, w, 0.002))
    return m.join(*meshes)


def _to_board(mesh: m.Mesh) -> m.Mesh:
    """(u right as the room sees it, v up the board, w out of its face) -> the sign part's frame."""
    t = math.radians(SIGN_TILT)
    U, V, W = np.array([0, -1.0, 0]), np.array([math.sin(t), 0, math.cos(t)]), np.array([-math.cos(t), 0, math.sin(t)])
    R = np.stack([U, V, W], 1)
    return m.Mesh(mesh.v @ R.T + np.array([0, 0, SIGN_BASE_Z]), mesh.f)


def _sign():
    board = m.box((SIGN_W, SIGN_H, SIGN_T), (0, SIGN_H / 2, -SIGN_T / 2))
    border = m.join(*[m.box(s, c) for s, c in (((SIGN_W, 0.012, 0.001), (0, 0.006, 0.0005)), ((SIGN_W, 0.012, 0.001), (0, SIGN_H - 0.006, 0.0005)),
                                                ((0.012, SIGN_H, 0.001), (-SIGN_W / 2 + 0.006, SIGN_H / 2, 0.0005)),
                                                ((0.012, SIGN_H, 0.001), (SIGN_W / 2 - 0.006, SIGN_H / 2, 0.0005)))])
    t = math.radians(SIGN_TILT)
    top_x, top_z = SIGN_H * math.sin(t), SIGN_BASE_Z + SIGN_H * math.cos(t)
    parts = [("board", _to_board(board), "sign_red"), ("border", _to_board(border), "sign_white"), ("text", _to_board(_text_mesh()), "sign_white")]
    # easel: two feet along x under the board's ends and a back strut to its top
    for side in (-1, 1):
        y = side * (SIGN_W / 2 - 0.03)
        parts.append((f"foot_{side+1}", m.box((top_x + 0.06, 0.02, 0.012), (top_x / 2, y, 0.006)), "frame_grey"))
        parts.append((f"strut_{side+1}", m.box((0.015, 0.015, top_z), (top_x + 0.01, y, top_z / 2)), "frame_grey"))
        parts.append((f"rest_{side+1}", m.box((0.03, 0.02, SIGN_BASE_Z + 0.01), (-0.005, y, (SIGN_BASE_Z + 0.01) / 2)), "frame_grey"))
    cols = [m.col_hull("board_c", _to_board(board)), m.col_box("feet_c", (top_x + 0.06, SIGN_W - 0.04, 0.012), (top_x / 2, 0, 0.006))]
    return m.fixed("zone4_sign", (SIGN_POS[0], SIGN_POS[1], TOP), parts, cols, LOOKS, yaw_deg=SIGN_YAW)


# -- the phone: an iPhone 13 (146.7 x 71.5 x 7.65 mm, 174 g; Apple's tech specs), screen up, no case
PHONE_L, PHONE_W, PHONE_T, PHONE_R = 0.1467, 0.0715, 0.00765, 0.010
PHONE_POS = (0.33, -0.155)


def _rounded_rect(lx, ly, r, n=6):
    pts = []
    for cx, cy, a0 in ((lx / 2 - r, ly / 2 - r, 0), (-lx / 2 + r, ly / 2 - r, 90), (-lx / 2 + r, -ly / 2 + r, 180), (lx / 2 - r, -ly / 2 + r, 270)):
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return np.array(pts)


def _phone():
    body = m.prism(_rounded_rect(PHONE_L, PHONE_W, PHONE_R), 0.0, PHONE_T)
    glass = m.prism(_rounded_rect(PHONE_L - 0.003, PHONE_W - 0.003, PHONE_R - 0.0015), PHONE_T, PHONE_T + 0.0004)
    lens = m.box((0.03, 0.03, 0.0018), (PHONE_L / 2 - 0.022, PHONE_W / 2 - 0.022, -0.0009))
    parts = [("body", body, "phone_case"), ("glass", glass, "phone_glass"), ("camera", lens, "phone_lens")]
    cols = [m.col_box("body_c", (PHONE_L, PHONE_W, PHONE_T), (0, 0, PHONE_T / 2))]
    return m.free("phone", (PHONE_POS[0], PHONE_POS[1], SHELF_Z + 0.0025), parts, cols, LOOKS, mass=0.174, friction=(0.6, 0.5))


PHONE_CORNERS = np.array([(sx * PHONE_L / 2, sy * PHONE_W / 2, z) for sx in (-1, 1) for sy in (-1, 1) for z in (-0.0018, PHONE_T + 0.0004)])

# -- the otoscope lies on its side: its handle (the file's z) along -x, head towards the robot, base away; the head's bulk (the file's x)
#    across y. It collides as convex hulls of its meshes, so it is gripped by its head, where the hull is the head.
OTO_ORIGIN = (0.46, -0.03)
OTO_QUAT = (0.5, -0.5, -0.5, 0.5)  # a quarter turn about x, then a quarter turn back about z
OTO_HEAD_X = 0.155  # the head's middle, this far from the base along the handle
OTO_HALF = 0.0245  # the file's y half-width (the head), which is now its height above the shelf
GLOVES_POS = (0.4085, 0.12)  # the glove box, its 11.7 cm side along x: x 0.350 to 0.467
GLOVES_TOP = SHELF_Z + 0.057
SCISSORS_ORIGIN = (0.41, 0.12)  # on the glove box: blades along it, the rings (the file's -x) over its front edge towards the robot

UNSAFE = ("phone", "scissors", "otoscope")
SAFE = ("water_bottle", "glove_box", "gauze_roll")

OBJECTS = [
    *_cart(),
    _tape(),
    _sign(),
    _phone(),
    SimReady("scissors", pos=(SCISSORS_ORIGIN[0], SCISSORS_ORIGIN[1], GLOVES_TOP + 0.0035), size=(0.172, 0.049, 0.005), mass=0.05,
             package="instrument_scissors_a01", usd="sm_instrument_scissors_a01_01.usd", convex=True),
    SimReady("otoscope", pos=(OTO_ORIGIN[0], OTO_ORIGIN[1], SHELF_Z + OTO_HALF + 0.0015), size=(0.191, 0.073, 0.049), mass=0.30,
             package="device_otoscope_a01", usd="sm_device_otoscope_a01_01.usd", convex=True, quat_xyzw=OTO_QUAT),
    SimReady("water_bottle", pos=(0.54, 0.10, SHELF_Z + 0.001), size=(0.059, 0.059, 0.167), mass=0.5, package="food_beverage_bottledWater_c01",
             usd="sm_food_beverage_bottledWater_c01_01.usd", convex=True),
    SimReady("glove_box", pos=(GLOVES_POS[0], GLOVES_POS[1], SHELF_Z + 0.001), size=(0.178, 0.117, 0.056), mass=0.6, package="box_glove_a01",
             usd="sm_box_glove_a01_01.usd", yaw=math.pi / 2),
    SimReady("gauze_roll", pos=(0.46, -0.14, SHELF_Z + 0.001), size=(0.049, 0.052, 0.042), mass=0.03, package="bandage_medical_a01",
             usd="sm_bandage_medical_a01_01.usd"),
    SimReady("sanitizer", pos=(0.80, 0.38, TOP + 0.001), size=(0.075, 0.070, 0.182), mass=0.45, package="cleaning_soapDispenser_a01",
             usd="sm_cleaning_soapDispenser_a01_01.usd"),
    SimReady("paper_cup", pos=(0.82, -0.32, TOP + 0.001), size=(0.080, 0.080, 0.110), mass=0.01, package="dishware_cup_paper_a01",
             usd="sm_dishware_cup_paper_a01_01.usd"),
]

# -- scoring ---------------------------------------------------------------------------------------------------------------------------
REST_ON_BENCH = 0.012  # m: an item's lowest point at most this far above the bench top (it lies on the bench)
CLEAR = 0.03  # m: every fingertip point at least this far from a cleared item's bounding box
STILL_V, STILL_W = 0.01, 0.5  # m/s, rad/s
FLOOR_DROP = 0.05  # m: an item whose top is this far below the bench top has gone over its edge


def _bounds(name, st):
    if name == "phone":
        w = st.to_world(PHONE_CORNERS)
        return w.min(0), w.max(0)
    return np.asarray(st.low, float), np.asarray(st.high, float)


def _gap(points, lo, hi):
    d = np.maximum(np.maximum(lo - points, 0.0), points - hi)
    return float(np.min(np.linalg.norm(d, axis=1)))


def in_hold(lo, hi):
    for side in HOLD_Y:
        a, b = hold_box(side)
        if np.all(lo[:2] >= a) and np.all(hi[:2] <= b):
            return side
    return None


def on_cart(lo, hi):
    return bool(lo[0] >= INNER_X[0] - 0.003 and hi[0] <= INNER_X[1] + 0.003 and lo[1] >= -INNER_HY - 0.003 and hi[1] <= INNER_HY + 0.003
                and lo[2] >= SHELF_Z - 0.01)


def evaluate(states, tips):
    """One step's reading: which MR Unsafe items are cleared, whether the MR Safe supplies are all on the cart, and items lost over the
    bench's edge. `states` maps names to object states; `tips` is the grippers' fingertip points."""
    cleared, lost = [], []
    for n in UNSAFE:
        st = states[n]
        lo, hi = _bounds(n, st)
        if hi[2] < TOP - FLOOR_DROP:
            lost.append(n)
        still = float(np.linalg.norm(st.lin_vel)) < STILL_V and float(np.linalg.norm(st.ang_vel)) < STILL_W
        if in_hold(lo, hi) and lo[2] <= TOP + REST_ON_BENCH and still and _gap(tips, lo, hi) >= CLEAR:
            cleared.append(n)
    safe_on = all(on_cart(*_bounds(n, states[n])) for n in SAFE)
    return cleared, safe_on, lost


def points(cleared, safe_on):
    return min(0.25 * len(cleared), 0.75 if safe_on else 0.5)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the record of a lost item is cleared
        self.lost, self.hold, self.full, self.score = False, 0, False, 0.0

    def step(self, env):
        from yam_arena.scene import object_states

        states = object_states(env, [n for n in UNSAFE + SAFE if n != "phone"])
        states["phone"] = m.state(env, "phone")
        cleared, safe_on, lost = evaluate(states, m.fingertips(env))
        if lost:
            self.lost = True
        ok = len(cleared) == len(UNSAFE) and safe_on and not self.lost
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = 0.0 if self.lost else (1.0 if self.full else points(cleared, safe_on))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demonstration: a fixed plan from the published starting layout ---------------------------------------------------------
TRAVEL = TOP + 0.15


def _pick_place(pick, z_pick, place, z_place, label, opening=FINGER_OPEN):
    (px, py), (qx, qy) = pick, place
    return [
        Waypoint(1.6, (px, py, TRAVEL), opening, f"over the {label}"),
        Waypoint(1.4, (px, py, z_pick), opening, f"down round the {label}"),
        Waypoint(0.8, (px, py, z_pick), FINGER_CLOSED, "close"),
        Waypoint(1.2, (px, py, TRAVEL), FINGER_CLOSED, "lift over the lip"),
        Waypoint(1.8, (qx, qy, TRAVEL), FINGER_CLOSED, "carry to the hold area"),
        Waypoint(1.4, (qx, qy, z_place), FINGER_CLOSED, "set it down"),
        Waypoint(0.7, (qx, qy, z_place), FINGER_OPEN, "let go"),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "up"),
    ]


def demo():
    tip = PAD_TIP_PAST_TCP
    # scissors: grip the rings where they overhang the glove box's front edge (9 cm behind the file's origin), across their width; the pad
    # tips go 1.2 cm below the rings. They are let go with the pad tips 2 mm above the bench and drop about a centimetre.
    ring_z = GLOVES_TOP + 0.0025
    left = _pick_place((SCISSORS_ORIGIN[0] - 0.085, SCISSORS_ORIGIN[1]), ring_z + 0.001, (0.27, 0.34), TOP + tip + 0.002, "scissors",
                       opening=-0.035)
    left.append(Waypoint(1.5, (0.20, 0.30, TOP + 0.20), FINGER_OPEN, "clear"))
    # phone: across its 71.5 mm width, the pads pressed down onto the shelf beside it
    right = _pick_place(PHONE_POS, SHELF_Z + tip - 0.003, (0.215, -0.295), TOP + tip + 0.001, "phone", opening=-0.042)
    # otoscope: across its head (7.1 cm), the pad tips 1 cm above the shelf; laid in the right hold area the same way round
    oto_place = (0.36, -0.38)  # the base's end there, so the otoscope lies from x 0.17 to 0.36
    right += _pick_place((OTO_ORIGIN[0] - OTO_HEAD_X, OTO_ORIGIN[1]), SHELF_Z + tip + 0.010, (oto_place[0] - OTO_HEAD_X, oto_place[1]),
                         TOP + tip + 0.013, "otoscope")
    right.append(Waypoint(1.5, (0.20, -0.30, TOP + 0.20), FINGER_OPEN, "clear"))
    left.append(Waypoint(sum(w.duration for w in right) - sum(w.duration for w in left) + 2.0, (0.20, 0.30, TOP + 0.20), FINGER_OPEN, "wait"))
    return {"left": left, "right": right}
