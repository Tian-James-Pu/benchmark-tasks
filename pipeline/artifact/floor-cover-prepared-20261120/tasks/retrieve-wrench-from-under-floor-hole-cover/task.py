"""A wrench has dropped into a 200 mm square opening in a plywood sleeper floor, under the hole's cover. Get it out and put it in the basket.

A section of sleeper floor is fixed on the table: 18 mm plywood on 38 x 64 mm sleepers over the concrete slab (the table top), its walking surface 82 mm up.
A 200 x 200 mm opening goes through it, framed with blocking, centred 0.30 m in front of the arms on the midline. Over it lies the hole's cover, built as
construction safety guidance says: a 300 x 300 mm square of 3/4 in (19 mm) plywood, painted safety orange and lettered HOLE, with two 19 x 38 mm cleats
screwed underneath that drop into the opening (6 mm play a side), so it cannot be kicked or slid off. A 36 x 110 mm hand slot through its middle is
how it is lifted. A 9/16 in combination wrench lies on the slab inside the opening, under the cover. A blue plastic basket stands to the right of the
deck. Leaving the hole open is the hazard: someone steps into it.

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/retrieve-wrench-from-under-floor-hole-cover/task.py \
      -E object_jitter=0 -E control_hz=20 --instruction "..." --scorer task_score --max-steps 900
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 1 s hold, so score() and success() agree
HOLD_STEPS = 20  # 1 s at 20 Hz

# -- the floor ----------------------------------------------------------------------------------
PLY = 0.018  # 23/32 in subfloor plywood
SLEEPER_W, SLEEPER_H = 0.038, 0.064  # 2x3 sleepers on edge
DECK_H = PLY + SLEEPER_H  # walking surface 82 mm above the slab
DECK_Z = TOP + DECK_H
DECK_X = (0.10, 0.50)
DECK_Y = (-0.17, 0.45)
HOLE_C = np.array([0.30, 0.0])
HOLE = 0.200  # square opening

# -- the cover ----------------------------------------------------------------------------------
COVER = 0.300  # square, 50 mm over the deck all round
COVER_T = 0.019  # 3/4 in plywood
CLEAT_W, CLEAT_H, CLEAT_L = 0.038, 0.019, 0.180  # 1x2 cleats laid flat
CLEAT_OUT = 0.094  # cleat outer face from the centre: 188 mm outside to outside, 6 mm play a side in the 200 mm opening
BEVEL = 0.006  # 45 degree bevel planed on each cleat's lower outer edge
SLOT_W, SLOT_L = 0.036, 0.110  # hand slot through the middle, along x
COVER_MASS = 1.05
COVER0 = (float(HOLE_C[0]), float(HOLE_C[1]), DECK_Z + COVER_T / 2)  # its origin: the plate's centre

# -- the basket (tool tote) and the wrench --------------------------------------------------
BASKET_POS = (0.30, -0.335, TOP)  # long side along x
BASKET_IN_X, BASKET_IN_Y = 0.1567, 0.1092  # inside half-lengths after the 90 degree turn (walls' inner faces)
BASKET_FLOOR = 0.0075  # the basket floor's top above the table: where the wrench's lowest point comes to rest in the simulator
WRENCH_YAW = 0.35
WRENCH_C = np.array([0.30, -0.044])  # the wrench's middle on the slab
WRENCH_MID = -0.0150  # the middle along the wrench, in its file's frame (it spans -110.3 to +80.2 mm)
WRENCH_POS = (float(WRENCH_C[0] - WRENCH_MID * math.cos(WRENCH_YAW)), float(WRENCH_C[1] - WRENCH_MID * math.sin(WRENCH_YAW)), TOP)

LOOKS = {
    "ply": m.Look(color=m.rgb(0.80, 0.66, 0.45), roughness=0.8),
    "sleeper": m.Look(color=m.rgb(0.86, 0.74, 0.55), roughness=0.8),
    "slab": m.Look(color=m.rgb(0.58, 0.58, 0.56), roughness=0.95),
    "orange": m.Look(color=m.rgb(0.98, 0.42, 0.05), roughness=0.6),
    "ink": m.Look(color=m.rgb(0.04, 0.04, 0.04), roughness=0.7),
    "cleat": m.Look(color=m.rgb(0.84, 0.70, 0.50), roughness=0.8),
    "pencil": m.Look(color=m.rgb(0.25, 0.25, 0.27), roughness=0.9),
}


def _box(parts, cols, name, size, centre, look, collide=True):
    parts.append((name, m.box(size, centre), look))
    if collide:
        cols.append(m.col_box(f"{name}_c", size, centre))


def _deck() -> m.Part:
    """Plywood on sleepers, with the opening framed by blocking; the slab under the opening is the table, shown in concrete grey."""
    parts, cols = [], []
    x0, x1 = DECK_X
    y0, y1 = DECK_Y
    h0, h1 = HOLE_C[0] - HOLE / 2, HOLE_C[0] + HOLE / 2
    g0, g1 = HOLE_C[1] - HOLE / 2, HOLE_C[1] + HOLE / 2
    zp = SLEEPER_H + PLY / 2
    # plywood sheet around the opening, as four boards
    _box(parts, cols, "ply_near", (h0 - x0, y1 - y0, PLY), ((x0 + h0) / 2, (y0 + y1) / 2, zp), "ply")
    _box(parts, cols, "ply_far", (x1 - h1, y1 - y0, PLY), ((h1 + x1) / 2, (y0 + y1) / 2, zp), "ply")
    _box(parts, cols, "ply_right", (HOLE, g0 - y0, PLY), (HOLE_C[0], (y0 + g0) / 2, zp), "ply")
    _box(parts, cols, "ply_left", (HOLE, y1 - g1, PLY), (HOLE_C[0], (g1 + y1) / 2, zp), "ply")
    zs = SLEEPER_H / 2
    # sleepers along y at the near and far edges, along x at the sides
    _box(parts, cols, "sl_near", (SLEEPER_W, y1 - y0, SLEEPER_H), (x0 + SLEEPER_W / 2, (y0 + y1) / 2, zs), "sleeper")
    _box(parts, cols, "sl_far", (SLEEPER_W, y1 - y0, SLEEPER_H), (x1 - SLEEPER_W / 2, (y0 + y1) / 2, zs), "sleeper")
    _box(parts, cols, "sl_right", (x1 - x0 - 2 * SLEEPER_W, SLEEPER_W, SLEEPER_H), ((x0 + x1) / 2, y0 + SLEEPER_W / 2, zs), "sleeper")
    _box(parts, cols, "sl_left", (x1 - x0 - 2 * SLEEPER_W, SLEEPER_W, SLEEPER_H), ((x0 + x1) / 2, y1 - SLEEPER_W / 2, zs), "sleeper")
    # blocking round the opening: its faces are the opening's walls
    _box(parts, cols, "blk_near", (SLEEPER_W, HOLE + 2 * SLEEPER_W, SLEEPER_H), (h0 - SLEEPER_W / 2, HOLE_C[1], zs), "sleeper")
    _box(parts, cols, "blk_far", (SLEEPER_W, HOLE + 2 * SLEEPER_W, SLEEPER_H), (h1 + SLEEPER_W / 2, HOLE_C[1], zs), "sleeper")
    _box(parts, cols, "blk_right", (HOLE, SLEEPER_W, SLEEPER_H), (HOLE_C[0], g0 - SLEEPER_W / 2, zs), "sleeper")
    _box(parts, cols, "blk_left", (HOLE, SLEEPER_W, SLEEPER_H), (HOLE_C[0], g1 + SLEEPER_W / 2, zs), "sleeper")
    # an intermediate sleeper under the left half, where a cover may be set down
    _box(parts, cols, "sl_mid", (x1 - x0 - 2 * SLEEPER_W, SLEEPER_W, SLEEPER_H), ((x0 + x1) / 2, 0.29, zs), "sleeper")
    _box(parts, cols, "slab", (HOLE, HOLE, 0.001), (HOLE_C[0], HOLE_C[1], 0.0005), "slab", collide=False)
    # the cover's outline pencilled on the deck, 1 mm lines (seen only)
    c, L = COVER / 2 + 0.001, COVER + 0.004
    for k, (sz, ctr) in enumerate((((0.001, L), (c, 0)), ((0.001, L), (-c, 0)), ((L, 0.001), (0, c)), ((L, 0.001), (0, -c)))):
        _box(parts, cols, f"outline{k}", (sz[0], sz[1], 0.0003), (HOLE_C[0] + ctr[0], HOLE_C[1] + ctr[1], DECK_H + 0.00015), "pencil", collide=False)
    return m.fixed("deck", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.45))


def _cleat_mesh(sy: float) -> m.Mesh:
    """One cleat along x under the plate, at side sy (+1 left, -1 right), its lower outer edge bevelled."""
    yo = sy * CLEAT_OUT
    yi = sy * (CLEAT_OUT - CLEAT_W)
    zt, zb = -COVER_T / 2, -COVER_T / 2 - CLEAT_H
    prof = [(yi, zt), (yo, zt), (yo, zb + BEVEL), (yo - sy * BEVEL, zb), (yi, zb)]  # (y, z)
    v = [(x, y, z) for x in (-CLEAT_L / 2, CLEAT_L / 2) for (y, z) in prof]
    v = np.array(v, float)
    n = len(prof)
    f = []
    for i in range(1, n - 1):
        f.append((0, i + 1, i))
        f.append((n, n + i, n + i + 1))
    for i in range(n):
        j = (i + 1) % n
        f.append((i, j, n + j))
        f.append((i, n + j, n + i))
    f = np.array(f, int)
    if sy < 0:  # mirrored profile: flip the winding
        f = f[:, ::-1]
    return m.Mesh(v, f)


def _letters():
    """HOLE in 50 mm black letters beyond the slot's far end, reading from the robot's side (up is +x, left to right is -y)."""
    sw, lw, lh, gap = 0.008, 0.034, 0.050, 0.012
    x0 = SLOT_L / 2 + 0.020
    strokes = {
        "H": [(0, 0, sw, lh), (lw - sw, 0, sw, lh), (0, (lh - sw) / 2, lw, sw)],
        "O": [(0, 0, sw, lh), (lw - sw, 0, sw, lh), (0, 0, lw, sw), (0, lh - sw, lw, sw)],
        "L": [(0, 0, sw, lh), (0, 0, lw, sw)],
        "E": [(0, 0, sw, lh), (0, 0, lw, sw), (0, (lh - sw) / 2, lw * 0.85, sw), (0, lh - sw, lw, sw)],
    }
    total = 4 * lw + 3 * gap
    out = []
    for k, ch in enumerate("HOLE"):
        u0 = -total / 2 + k * (lw + gap)
        for j, (u, v, du, dv) in enumerate(strokes[ch]):
            cx = x0 + v + dv / 2
            cy = -(u0 + u + du / 2)
            out.append((f"ink_{ch}{k}_{j}", m.box((dv, du, 0.0006), (cx, cy, COVER_T / 2 + 0.0003)), "ink"))
    return out


def _cover() -> m.Part:
    parts, cols = [], []
    half, sl, sw = COVER / 2, SLOT_L / 2, SLOT_W / 2
    side_w = half - sw
    for k, sy in enumerate((-1, 1)):  # plate either side of the slot
        _box(parts, cols, f"plate_s{k}", (COVER, side_w, COVER_T), (0.0, sy * (sw + side_w / 2), 0.0), "orange")
    end_l = half - sl
    for k, sx in enumerate((-1, 1)):  # plate beyond the slot's ends
        _box(parts, cols, f"plate_e{k}", (end_l, SLOT_W, COVER_T), (sx * (sl + end_l / 2), 0.0, 0.0), "orange")
    for k, sy in enumerate((-1, 1)):
        mesh = _cleat_mesh(sy)
        parts.append((f"cleat{k}", mesh, "cleat"))
        cols.append(m.col_hull(f"cleat{k}_c", mesh))
    parts.extend(_letters())
    return m.free("cover", COVER0, parts, cols, LOOKS, mass=COVER_MASS, friction=(0.5, 0.45), com=(0.0, 0.0, -0.003))


DECK = _deck()
COVER_PART = _cover()
WRENCH = SimReady("wrench", pos=WRENCH_POS, size=(0.191, 0.029, 0.009), mass=0.11, package="handTool_wrench_9_16in_b01",
                  usd="sm_handTool_wrench_9_16in_b01_01.usd", yaw=WRENCH_YAW, convex=True)
BASKET = SimReady("basket", pos=BASKET_POS, size=(0.332, 0.254, 0.132), package="storage_basket_plastic_a01",
                  usd="sm_storage_basket_plastic_a01_01.usd", yaw=math.pi / 2)
# distractors, ignored by the scorer
KNIFE = SimReady("utility_knife", pos=(0.585, 0.40, TOP), size=(0.160, 0.061, 0.026), mass=0.12, package="handTool_knife_retractableBlade_a01",
                 usd="sm_handTool_knife_retractableBlade_a01_01.usd", yaw=0.3)
SQUARE = SimReady("speed_square", pos=(0.60, 0.16, TOP), size=(0.290, 0.218, 0.008), mass=0.15, package="measurement_ruler_speedsquare_a02",
                  usd="sm_measurement_ruler_speedsquare_a02_01.usd", yaw=math.pi / 2)
GLASSES = SimReady("safety_glasses", pos=(0.60, -0.12, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                   usd="sm_accessory_eyeglasses_a01_01.usd", yaw=-0.3)
OBJECTS = [DECK, COVER_PART, WRENCH, BASKET, KNIFE, SQUARE, GLASSES]

# -- what the scorer checks ------------------------------------------------------------------
LIFT_TOL = 0.004  # each corner of the plate's underside within 4 mm of the deck's top
CENTRE_TOL = 0.010  # plate centre within 10 mm of the opening's centre (the cleats allow 6 mm)
TILT_TOL = 2.0  # degrees; and lettered face up
FLOOR_TOL = 0.005  # wrench's lowest point within 5 mm of the basket floor's top
CLEAR = 0.030  # every fingertip point at least 3 cm from the cover and the wrench


def _box_gap(points: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> float:
    out = np.maximum(np.maximum(lo - points, points - hi), 0.0)
    return float(np.min(np.linalg.norm(out, axis=1)))


def measure(cover: m.PartState, wrench, tips: np.ndarray | None) -> dict:
    rot = cover.rotation()
    corners = np.array([[sx * COVER / 2, sy * COVER / 2, -COVER_T / 2] for sx in (-1, 1) for sy in (-1, 1)])
    world = cover.pos + corners @ rot.T
    lift = world[:, 2] - DECK_Z
    up = float(rot[2, 2])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up))))
    off = float(np.linalg.norm(np.asarray(cover.pos[:2]) - HOLE_C))
    covered = bool(np.all(np.abs(lift) <= LIFT_TOL) and off <= CENTRE_TOL and up > 0 and tilt <= TILT_TOL)
    lo, hi = np.asarray(wrench.low), np.asarray(wrench.high)
    wc = (lo + hi) / 2
    in_opening = bool(abs(wc[0] - HOLE_C[0]) < HOLE / 2 and abs(wc[1] - HOLE_C[1]) < HOLE / 2 and wc[2] < DECK_Z)
    bx, by = BASKET_POS[0], BASKET_POS[1]
    inside = bool(lo[0] >= bx - BASKET_IN_X and hi[0] <= bx + BASKET_IN_X and lo[1] >= by - BASKET_IN_Y and hi[1] <= by + BASKET_IN_Y)
    floor_gap = float(lo[2] - (TOP + BASKET_FLOOR))
    on_floor = bool(inside and floor_gap <= FLOOR_TOL)
    put_away = bool(on_floor and wrench.resting())
    if tips is None:
        clear = False
        gap_cover = gap_wrench = float("nan")
    else:
        local = (tips - cover.pos) @ rot
        clo = np.array([-COVER / 2, -COVER / 2, -COVER_T / 2 - CLEAT_H])
        chi = np.array([COVER / 2, COVER / 2, COVER_T / 2])
        gap_cover = _box_gap(local, clo, chi)
        gap_wrench = _box_gap(tips, lo, hi)
        clear = bool(gap_cover >= CLEAR and gap_wrench >= CLEAR)
    still = bool(cover.resting() and wrench.resting())
    return {"covered": covered, "lift_mm": [round(float(v) * 1000, 1) for v in lift], "off_mm": round(off * 1000, 1), "tilt": round(tilt, 2),
            "face_up": up > 0, "in_opening": in_opening, "inside_basket": inside, "floor_gap_mm": round(floor_gap * 1000, 1),
            "put_away": put_away, "gap_cover_mm": round(gap_cover * 1000, 1), "gap_wrench_mm": round(gap_wrench * 1000, 1),
            "clear": clear, "still": still, "wrench_c": [round(float(v), 4) for v in wc]}


def tier(r: dict) -> float:
    """The score of one step's measurements, before the 1 s hold."""
    if not r or r["in_opening"]:
        return 0.0
    if r["put_away"] and r["covered"]:
        return 1.0 if (r["clear"] and r["still"]) else 0.75
    if r["covered"]:
        return 0.5
    if r["put_away"]:
        return 0.25
    return 0.0


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.steps, self.hold, self.last = 0, 0, {}

    def step(self, env):
        from yam_arena.scene import object_states

        self.steps += 1
        cover = m.state(env, "cover")
        wrench = object_states(env, ("wrench",))["wrench"]
        self.last = measure(cover, wrench, m.fingertips(env))
        self.hold = self.hold + 1 if tier(self.last) == 1.0 else 0
        if self.steps % 10 == 0 or self.hold == HOLD_STEPS:
            t = round(self.steps * float(env.unwrapped.step_dt), 2)
            print("[holecover] " + json.dumps({"t": t, "hold": self.hold, "tier": tier(self.last), **self.last}), flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    """1.0 once every full-score condition has held for 1 s; until then, the step's tier capped at 0.75."""
    if TRIAL.hold >= HOLD_STEPS:
        return 1.0
    return float(min(tier(TRIAL.last), 0.75))


def success(objects) -> bool:
    return bool(TRIAL.hold >= HOLD_STEPS)


# -- the scripted demonstration: a fixed plan from the published starting layout ------------------
TCP_TIP = 0.013  # the finger pads' tips sit 13 mm below the TCP
COVER_TOP = DECK_Z + COVER_T
SLOT_TCP = COVER_TOP - 0.022 + TCP_TIP  # fingertips 22 mm into the slot, 3 mm below the plate
SPREAD = -0.022  # fingers pushed apart inside the 36 mm slot (they touch its sides at about -0.009)
HOLD_AT = (0.30, 0.255)  # where the left arm holds the cover, clear of the opening, over the deck's left half
CARRY_Z = SLOT_TCP + 0.060  # cleats 4 cm over the deck while carried
SEAT_PUSH = float(os.environ.get("SEAT_PUSH", "0.012"))  # the TCP goes this far below the seated grip height (the arm sags under the cover)


def demo() -> dict[str, list[Waypoint]]:
    hx, hy = float(HOLE_C[0]), float(HOLE_C[1])
    L = []
    L.append(Waypoint(2.5, (hx, hy, SLOT_TCP + 0.06), 0.0, "over the hand slot, fingers closed"))
    L.append(Waypoint(1.5, (hx, hy, SLOT_TCP), 0.0, "fingers into the slot"))
    L.append(Waypoint(1.0, (hx, hy, SLOT_TCP), SPREAD, "spread the fingers against the slot's sides"))
    L.append(Waypoint(2.0, (hx, hy, CARRY_Z), SPREAD, "lift the cover off the opening"))
    L.append(Waypoint(2.0, (HOLD_AT[0], HOLD_AT[1], CARRY_Z), SPREAD, "hold it aside over the deck"))
    L.append(Waypoint(13.0, (HOLD_AT[0], HOLD_AT[1], CARRY_Z), SPREAD, "hold it while the wrench comes out"))
    L.append(Waypoint(2.5, (hx, hy, CARRY_Z), SPREAD, "back over the opening"))
    L.append(Waypoint(2.0, (hx, hy, SLOT_TCP + 0.012), SPREAD, "lower until the cleats are in"))
    L.append(Waypoint(1.5, (hx, hy, SLOT_TCP - SEAT_PUSH), SPREAD, "press it flat"))
    L.append(Waypoint(1.0, (hx, hy, SLOT_TCP - SEAT_PUSH), 0.0, "fingers together: let go"))
    L.append(Waypoint(1.5, (hx, hy, SLOT_TCP + 0.06), 0.0, "out of the slot"))
    L.append(Waypoint(1.5, (0.20, 0.30, SLOT_TCP + 0.08), FINGER_OPEN, "clear"))
    L.append(Waypoint(3.0, (0.20, 0.30, SLOT_TCP + 0.08), FINGER_OPEN, "wait"))

    wx, wy = float(WRENCH_C[0]), float(WRENCH_C[1])
    grip_z = TOP + 0.003 + TCP_TIP
    high = DECK_Z + 0.075
    bx, by = BASKET_POS[0], BASKET_POS[1]
    R = []
    R.append(Waypoint(9.0, (0.22, -0.30, high), FINGER_OPEN, "wait for the opening", yaw=WRENCH_YAW))
    R.append(Waypoint(2.0, (wx, wy, high), FINGER_OPEN, "over the wrench", yaw=WRENCH_YAW))
    R.append(Waypoint(2.0, (wx, wy, grip_z), -0.024, "down into the opening", yaw=WRENCH_YAW))
    R.append(Waypoint(1.0, (wx, wy, grip_z), 0.0, "grip the shank", yaw=WRENCH_YAW))
    R.append(Waypoint(2.0, (wx, wy, high), 0.0, "lift it out", yaw=WRENCH_YAW))
    R.append(Waypoint(2.0, (bx, by, high), 0.0, "over the basket", yaw=0.0))
    R.append(Waypoint(1.5, (bx, by, TOP + 0.075), 0.0, "lower into the basket", yaw=0.0))
    R.append(Waypoint(1.0, (bx, by, TOP + 0.075), FINGER_OPEN, "let go", yaw=0.0))
    R.append(Waypoint(1.5, (bx, by, high + 0.02), FINGER_OPEN, "up", yaw=0.0))
    R.append(Waypoint(2.0, (0.18, -0.30, high + 0.02), FINGER_OPEN, "clear", yaw=0.0))
    R.append(Waypoint(12.0, (0.18, -0.30, high + 0.02), FINGER_OPEN, "wait", yaw=0.0))
    return {"left": L, "right": R}
