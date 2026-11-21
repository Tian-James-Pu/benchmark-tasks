"""Pack three filled tiers into a stainless-steel tiffin carrier, put the lid on and latch the handle.

The carrier is a generic 3-tier, 14 cm Indian/Thai-style stainless-steel food carrier at full size: a base cross with two upright side
bars, three round tiers (14 cm across, 5 cm deep) and a lid, each with a pair of side lugs (eyes) that the bars pass through, and a
separate carrying handle whose end eyes drop over the bars' heads and whose sprung catches snap under the heads. The carrier frame stands
empty on the counter; the three tiers, filled (rigid food stand-ins), the lid and the handle lie beside it, the tiers turned at random.

Model (declared in code, all collision geometry is real contact, nothing is triggered by proximity):
- Carrier ("carrier", free, 0.20 kg): the base cross (two 3 mm steel strips) and two 6 mm bars, 152 mm apart, 172 mm tall,
  each topped by a 9 mm head with a pointed tip. No links.
- Tiers ("tier_a", "tier_b", "tier_c", free, 0.35 kg each: 0.20 kg of steel and 0.15 kg of food): a 2 mm wall of 32 convex segments, a 3 mm
  floor, and two lug eyes at opposite sides of the rim, 14 x 14 mm inside, so a 6 mm bar has 4 mm of play each way and the 9 mm head
  2.5 mm. A tier lowered with its lugs off the bars by more than that lands its lug on the bar's head or on the rim below and sits cocked.
- Lid ("lid", free, 0.09 kg): a 142 mm disc with a knob and the same pair of lugs.
- Handle ("handle", a free mechanism, 0.07 kg): a steel strip with an eye at each end and a carry grip. In each eye a sprung catch (a
  slider link, 2.5 mm travel, 400 N/m) is cammed aside by the bar's pointed head as the handle is pressed down and springs out under the
  head once the handle is fully home on the lid's knob. With the lid or a tier standing proud, the handle rests on the knob above the
  heads' undersides and the catches cannot close.

Scoring: see `score_text` below (task.json carries the same words).

    python demokit.py task.py OUT --mode full
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
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Pack the three filled tiers into the tiffin carrier, put the lid on and latch the handle."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- dimensions (metres) --------------------------------------------------------------------------------------------------------------
CARRIER_XY = (0.22, 0.0)
R_OUT = 0.070  # a tier's outside radius: 14 cm
WALL = 0.002  # wall thickness (collider; the real wall is about 0.6 mm with a rolled rim)
H_T = 0.050  # a tier's depth: tiers stack rim to floor, 50 mm a level
FLOOR = 0.003
BASE_T = 0.003  # the base cross's strip thickness: tier 1's floor rests on it
R_BAR = 0.076  # the bars' (and the lugs' eyes') distance from the carrier's axis
BAR_R = 0.003  # 6 mm bars
H_HEAD0, H_HEAD1, H_TIP = 0.172, 0.177, 0.183  # each bar's head: a collar from H_HEAD0 to H_HEAD1, then a point up to H_TIP
HEAD_R = 0.0045
EYE = 0.007  # half the inside of a lug's eye (14 x 14 mm)
LUG_T = 0.002  # lug plate thickness
LUG_H = 0.016  # lug height: the lugs' tops are flush with the rim
R_LID = 0.071
LID_T = 0.003
KNOB_R, KNOB_H = 0.0125, 0.010
LID_LUG_H = 0.012
N_SEG = 32
LEVEL_Z = [BASE_T + k * H_T for k in range(4)]  # a seated tier's (or the lid's) floor height above the counter, by level
H_LID_TOP = LEVEL_Z[3] + LID_T + KNOB_H  # the knob's top with the lid on three tiers: 0.168

LOOKS = {
    "steel": m.Look(color=m.rgb(0.80, 0.80, 0.82), metallic=0.95, roughness=0.22),
    "steel_dark": m.Look(color=m.rgb(0.55, 0.56, 0.58), metallic=0.9, roughness=0.35),
    "rice": m.Look(color=m.rgb(0.95, 0.93, 0.86), roughness=0.9),
    "dal": m.Look(color=m.rgb(0.88, 0.62, 0.12), roughness=0.6),
    "sabzi": m.Look(color=m.rgb(0.35, 0.55, 0.18), roughness=0.8),
    "grip": m.Look(color=m.rgb(0.12, 0.12, 0.12), roughness=0.7),
}


def _sector(r0, r1, z0, z1, a0, a1) -> m.Mesh:
    pts = [(r * math.cos(a), r * math.sin(a), z) for a in (a0, a1) for r in (r0, r1) for z in (z0, z1)]
    return m.Mesh(np.array(pts), np.array([(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]))


def _lug(prefix: str, sign: float, z0: float, z1: float):
    """One lug's eye at x = sign * R_BAR: two side plates and an outer plate; the inner side is the wall (or the lid's edge)."""
    xo = sign * (R_BAR + EYE + LUG_T / 2)
    xl = (R_BAR + EYE + LUG_T) - R_OUT
    zc, h = (z0 + z1) / 2, z1 - z0
    cols = [m.col_box(f"{prefix}_out", (LUG_T, 2 * (EYE + LUG_T), h), (xo, 0.0, zc)),
            m.col_box(f"{prefix}_s1", (xl, LUG_T, h), (sign * (R_OUT + xl / 2), EYE + LUG_T / 2, zc)),
            m.col_box(f"{prefix}_s2", (xl, LUG_T, h), (sign * (R_OUT + xl / 2), -(EYE + LUG_T / 2), zc))]
    vis = m.join(m.box((LUG_T, 2 * (EYE + LUG_T), h), (xo, 0.0, zc)),
                 m.box((xl, LUG_T, h), (sign * (R_OUT + xl / 2), EYE + LUG_T / 2, zc)),
                 m.box((xl, LUG_T, h), (sign * (R_OUT + xl / 2), -(EYE + LUG_T / 2), zc)))
    return vis, cols


def _tier(food: str):
    prof = [(0.0, 0.0), (R_OUT - 0.004, 0.0), (R_OUT, 0.004), (R_OUT, H_T - 0.0015), (R_OUT + 0.0012, H_T - 0.0008), (R_OUT, H_T),
            (R_OUT - WALL, H_T), (R_OUT - WALL, FLOOR + 0.003), (R_OUT - WALL - 0.003, FLOOR), (0.0, FLOOR)]
    parts = [("body", m.lathe(prof, 64), "steel"),
             ("food", m.lathe([(0.0, FLOOR), (R_OUT - WALL - 0.002, FLOOR), (R_OUT - WALL - 0.002, 0.018), (R_OUT - 0.02, 0.020), (0.0, 0.021)], 48), food)]
    cols = [m.col_cylinder("floor_c", R_OUT, FLOOR, (0.0, 0.0, FLOOR / 2))]
    for k in range(N_SEG):
        a0, a1 = 2 * math.pi * k / N_SEG, 2 * math.pi * (k + 1) / N_SEG + 0.01
        cols.append(m.col_hull(f"w{k:02d}", _sector(R_OUT - WALL, R_OUT, FLOOR, H_T, a0, a1)))
    for i, s in enumerate((1.0, -1.0)):
        v, c = _lug(f"lug{i}", s, H_T - LUG_H, H_T)
        parts.append((f"lug{i}", v, "steel"))
        cols += c
    return parts, cols


def _lid():
    prof = [(0.0, 0.0), (R_LID, 0.0), (R_LID, LID_T - 0.001), (R_LID - 0.004, LID_T), (0.03, LID_T + 0.002), (0.0, LID_T + 0.002)]
    parts = [("disc", m.lathe(prof, 64), "steel"),
             ("knob", m.lathe([(0.0, LID_T), (KNOB_R, LID_T), (KNOB_R, LID_T + KNOB_H - 0.002), (KNOB_R - 0.002, LID_T + KNOB_H), (0.0, LID_T + KNOB_H)], 32), "steel_dark")]
    cols = [m.col_cylinder("disc_c", R_LID, LID_T, (0.0, 0.0, LID_T / 2)),
            m.col_cylinder("knob_c", KNOB_R, KNOB_H, (0.0, 0.0, LID_T + KNOB_H / 2))]
    for i, s in enumerate((1.0, -1.0)):
        v, c = _lug(f"lug{i}", s, 0.0, LID_LUG_H)
        parts.append((f"lug{i}", v, "steel"))
        cols += c
    return parts, cols


def _carrier():
    parts, cols = [], []
    parts.append(("strip_x", m.box((2 * (R_BAR + 0.012), 0.022, BASE_T), (0.0, 0.0, BASE_T / 2)), "steel"))
    parts.append(("strip_y", m.box((0.022, 0.15, BASE_T), (0.0, 0.0, BASE_T / 2)), "steel"))
    cols.append(m.col_box("strip_x_c", (2 * (R_BAR + 0.012), 0.022, BASE_T), (0.0, 0.0, BASE_T / 2)))
    cols.append(m.col_box("strip_y_c", (0.022, 0.15, BASE_T), (0.0, 0.0, BASE_T / 2)))
    for i, s in enumerate((1.0, -1.0)):
        x = s * R_BAR
        h = H_HEAD0 - BASE_T
        parts.append((f"bar{i}", m.cylinder(BAR_R, h, (x, 0.0, BASE_T + h / 2), segments=16), "steel"))
        head = m.lathe([(0.0, H_HEAD0), (HEAD_R, H_HEAD0), (HEAD_R, H_HEAD1), (0.0012, H_TIP), (0.0, H_TIP)], 16).moved((x, 0.0, 0.0))
        parts.append((f"head{i}", head, "steel_dark"))
        cols.append(m.col_cylinder(f"bar{i}_c", BAR_R, h, (x, 0.0, BASE_T + h / 2)))
        cols.append(m.col_hull(f"head{i}_c", head))
    return parts, cols


# -- the handle: a strip with an eye at each end, a carry grip, and a sprung catch in each eye --------------------------------------------
HANDLE_T = 0.003
GRIP_Z0, GRIP_Z1 = 0.026, 0.036  # the carry grip's underside and top above the strip's underside
CATCH_TRAVEL = 0.0035
CATCH_K = 1000.0
CATCH_TOP = 0.0045  # the catch's top above the handle's underside: 1.5 mm under the heads' collars with the handle home on the knob


def _handle_frame():
    parts, cols = [], []
    inner = R_BAR - EYE  # the eyes' inboard edges: the strip's ends
    parts.append(("strip", m.box((2 * inner, 0.022, HANDLE_T), (0.0, 0.0, HANDLE_T / 2)), "steel"))
    cols.append(m.col_box("strip_c", (2 * inner - 0.002, 0.022, HANDLE_T), (0.0, 0.0, HANDLE_T / 2)))
    for i, s in enumerate((1.0, -1.0)):
        x0, x1 = inner - 0.002, R_BAR + EYE + LUG_T  # the side plates run from the strip's end to the outer plate
        side = (x1 - x0, LUG_T, HANDLE_T)
        boxes = [((LUG_T, 2 * (EYE + LUG_T), HANDLE_T), (s * (R_BAR + EYE + LUG_T / 2), 0.0, HANDLE_T / 2)),
                 (side, (s * (x0 + x1) / 2, EYE + LUG_T / 2, HANDLE_T / 2)),
                 (side, (s * (x0 + x1) / 2, -(EYE + LUG_T / 2), HANDLE_T / 2))]
        parts.append((f"eye{i}", m.join(*[m.box(sz, c) for sz, c in boxes]), "steel"))
        cols += [m.col_box(f"eye{i}_{k}", sz, c) for k, (sz, c) in enumerate(boxes)]
    # the carry grip: two posts and a bar, along x
    for i, s in enumerate((1.0, -1.0)):
        parts.append((f"post{i}", m.box((0.006, 0.012, GRIP_Z0 - HANDLE_T + 0.001), (s * 0.045, 0.0, HANDLE_T + (GRIP_Z0 - HANDLE_T) / 2)), "steel"))
        cols.append(m.col_box(f"post{i}_c", (0.006, 0.012, GRIP_Z0 - HANDLE_T), (s * 0.045, 0.0, HANDLE_T + (GRIP_Z0 - HANDLE_T) / 2)))
    parts.append(("grip", m.box((0.096, 0.016, GRIP_Z1 - GRIP_Z0), (0.0, 0.0, (GRIP_Z0 + GRIP_Z1) / 2)), "grip"))
    cols.append(m.col_box("grip_c", (0.096, 0.016, GRIP_Z1 - GRIP_Z0), (0.0, 0.0, (GRIP_Z0 + GRIP_Z1) / 2)))
    return parts, cols


def _catch(i: int, s: float):
    """A catch: a small block in the eye's inboard side, sprung outboard (towards the bar) along x, with a sloped underside so the bar's
    head cams it inboard as the handle comes down. At rest its nose reaches to 3.4 mm from the bar's axis: under the head's 4.5 mm collar,
    clear of the 3 mm bar."""
    nose = R_BAR - 0.0034  # the nose's x (outboard face) at rest, in the handle's frame, for s = +1
    back = R_BAR - EYE + 0.0002
    z0, z1 = 0.0005, CATCH_TOP  # it stands from just above the strip's underside to 1.5 mm above its top
    pts = []
    for y in (-0.004, 0.004):
        pts += [(back, y, z0), (back, y, z1), (nose, y, z1), (nose, y, z0 + 0.0025), (nose - 0.0025, y, z0)]
    mesh = m.Mesh(np.array([(s * x, y, z) for x, y, z in pts]), np.array([(0, 1, 2)]))
    link = m.slider(f"catch{i}", "X", *((-CATCH_TRAVEL, 0.0) if s > 0 else (0.0, CATCH_TRAVEL)), (), (m.col_hull(f"catch{i}_c", mesh),),
                    mass=0.01, spring=CATCH_K, damping=0.5, friction=0.0, effort=20.0)
    return link, mesh


# -- objects --------------------------------------------------------------------------------------------------------------------------
_cp, _cc = _carrier()
CARRIER = m.free("carrier", (CARRIER_XY[0], CARRIER_XY[1], TOP), _cp, _cc, LOOKS, mass=0.20, friction=(0.9, 0.8), torsional_patch=0.02, solver_iters=32,
                 com=(0.0, 0.0, 0.05))

TIER_START = {"tier_a": ((0.21, -0.205), 25.0, "rice"), "tier_c": ((0.33, 0.205), -35.0, "sabzi"), "tier_b": ((0.38, -0.20), 60.0, "dal")}
TIERS = ("tier_a", "tier_c", "tier_b")  # the demo's order: rice at the bottom
TIER_PARTS = []
for _n in TIERS:
    (_x, _y), _yaw, _food = TIER_START[_n]
    _tp, _tc = _tier(_food)
    TIER_PARTS.append(m.free(_n, (_x, _y, TOP), _tp, _tc, LOOKS, mass=0.35, friction=(0.9, 0.8), torsional_patch=0.02, solver_iters=32,
                             yaw_deg=_yaw, com=(0.0, 0.0, 0.017)))

LID_START, LID_YAW = (0.15, 0.20), 15.0
_lp, _lc = _lid()
LID = m.free("lid", (LID_START[0], LID_START[1], TOP), _lp, _lc, LOOKS, mass=0.09, friction=(0.9, 0.8), torsional_patch=0.02, solver_iters=32, yaw_deg=LID_YAW,
             com=(0.0, 0.0, 0.004))

HANDLE_START, HANDLE_YAW = (0.42, 0.0), 60.0
_hp, _hc = _handle_frame()
_catches = [_catch(i, s) for i, s in enumerate((1.0, -1.0))]
for _i, (_lk, _mesh) in enumerate(_catches):
    _hp.append((f"catch{_i}_v", _mesh, "steel_dark"))  # drawn on the frame: the catch's travel is 2.5 mm
HANDLE = m.mechanism("handle", (HANDLE_START[0], HANDLE_START[1], TOP + 0.0005), [lk for lk, _ in _catches], LOOKS, frame_parts=_hp, frame_colliders=_hc,
                     friction=(0.9, 0.8), solver_iters=32, free_mass=0.07, frame_inertia=(0.00003, 0.00018, 0.0002), yaw_deg=HANDLE_YAW)

# -- distractors (SimReady): a banana, a steel spoon and a dish cloth on the counter -----------------------------------------------------
BANANA = SimReady("banana", pos=(0.52, -0.16, TOP), size=(0.039, 0.197, 0.074), mass=0.15, package="food_banana_a01",
                  usd="sm_food_banana_a01_01.usda", yaw=0.4)
SPOON = SimReady("spoon", pos=(0.55, 0.12, TOP), size=(0.157, 0.035, 0.011), mass=0.006, yaw=0.5,
                 package="dishware_plastic_spoon_a01", usd="sm_dishware_plastic_spoon_a01_01.usd")
CLOTH = SimReady("dish_cloth", pos=(0.52, 0.30, TOP), size=(0.260, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01",
                 usd="sm_cleaning_cloth_a01_01.usd", yaw=0.3)

OBJECTS = [CARRIER, *TIER_PARTS, LID, HANDLE, BANANA, SPOON, CLOTH]
DISTRACTORS = ("banana", "spoon", "dish_cloth")


# -- the scorer -------------------------------------------------------------------------------------------------------------------------
SPILL_DEG = 20.0
SEAT_Z_TOL = 0.003
SEAT_TILT = 3.0
BAR_IN_EYE = EYE - BAR_R + 0.0005  # a bar's axis within this of its eye's centre (the bar is inside the eye)


def _yaw(r: np.ndarray) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _bars_in_lugs(carrier, part) -> bool:
    """Both bars pass through both of this part's lug eyes: each bar's axis, at the lug's height, lies inside an eye."""
    ok = 0
    for s in (1.0, -1.0):
        foot = carrier.to_world(np.array([[s * R_BAR, 0.0, BASE_T]]))[0]
        top = carrier.to_world(np.array([[s * R_BAR, 0.0, H_HEAD0]]))[0]
        q = part.to_local(np.vstack([foot, top]))
        # the bar's axis where it crosses this part's lug band (z = H_T - LUG_H / 2 for a tier; the lid's lugs are lower, but the
        # bars stand within 0.3 degree of the parts' axes, so the axis's xy changes by < 0.1 mm over a lug)
        zc = 0.5 * (q[0, 2] + q[1, 2])
        lam = 0.0 if abs(q[1, 2] - q[0, 2]) < 1e-6 else (zc - q[0, 2]) / (q[1, 2] - q[0, 2])
        p = q[0] + lam * (q[1] - q[0])
        for t in (1.0, -1.0):
            if abs(p[0] - t * R_BAR) <= BAR_IN_EYE and abs(p[1]) <= BAR_IN_EYE:
                ok += 1
    return ok >= 2


def measure(st: dict, tips=None) -> dict:
    car = st["carrier"]
    c = {"carrier_tilt": car.tilt_deg(), "carrier_floor": float(car.pos[2] - TOP)}
    tiers = []
    for n in TIERS:
        s = st[n]
        loc = car.to_local(s.pos)[0]
        tiers.append({"name": n, "tilt": s.tilt_deg(), "z": float(loc[2]), "off": float(math.hypot(loc[0], loc[1])), "on_bars": _bars_in_lugs(car, s)})
    c["tiers"] = tiers
    tiers_sorted = sorted((t for t in tiers if t["on_bars"]), key=lambda t: t["z"])
    seated = 0
    for t in tiers_sorted:
        if seated < 3 and t["on_bars"] and t["tilt"] <= SEAT_TILT and abs(t["z"] - LEVEL_Z[seated]) <= SEAT_Z_TOL:
            seated += 1
        else:
            break
    c["seated"] = seated
    lid = st["lid"]
    lloc = car.to_local(lid.pos)[0]
    c["lid_z"], c["lid_tilt"], c["lid_on_bars"] = float(lloc[2]), lid.tilt_deg(), _bars_in_lugs(car, lid)
    c["lid_on"] = bool(seated >= 1 and c["lid_on_bars"] and c["lid_tilt"] <= SEAT_TILT and abs(c["lid_z"] - LEVEL_Z[seated]) <= SEAT_Z_TOL)
    hd = st["handle"]
    hloc = car.to_local(hd.pos)[0]
    c["handle_z"], c["handle_tilt"] = float(hloc[2]), hd.tilt_deg()
    c["handle_on_bars"] = _bars_in_lugs(car, hd)
    c["catches"] = [float(hd.joint(f"catch{i}")) for i in range(2)]
    # latched: both bars through the handle's eyes and the handle home, its underside below the heads' collars by at least 1 mm (so the
    # catches' noses, which stand 3.5 mm below the underside's top... are under the collars), resting on the knob
    c["handle_latched"] = bool(c["handle_on_bars"] and c["handle_tilt"] <= 4.0 and c["handle_z"] + HANDLE_T <= H_HEAD0 - 0.001
                               and all(abs(v) < 0.0008 for v in c["catches"]))
    v = [np.linalg.norm(st[n].lin_vel) for n in ("carrier", *TIERS, "lid", "handle")]
    w = [np.linalg.norm(st[n].ang_vel) for n in ("carrier", *TIERS, "lid", "handle")]
    c["still"] = bool(max(v) < 0.01 and max(w) < 0.1)
    if tips is not None:
        q = car.to_local(tips)
        dr = np.maximum(np.hypot(q[:, 0], q[:, 1]) - (R_BAR + EYE + LUG_T), 0.0)
        dz = np.maximum(q[:, 2] - (H_TIP + 0.04), 0.0)
        c["hand_gap"] = float(np.min(np.hypot(dr, dz)))
    return c


def level(c: dict) -> float:
    if c["seated"] >= 2 and c["lid_on"]:
        return 0.67
    if c["seated"] >= 1:
        return 0.33
    return 0.0


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place a recorded harm is cleared
        self.spilled = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        self.max_tilt = {n: 0.0 for n in TIERS}

    def step(self, env):
        st = {n: m.state(env, n) for n in ("carrier", *TIERS, "lid", "handle")}
        c = measure(st, m.fingertips(env))
        self.last = c
        for t in c["tiers"]:
            self.max_tilt[t["name"]] = max(self.max_tilt[t["name"]], t["tilt"])
            if t["tilt"] > SPILL_DEG:
                self.spilled = True
        lv = level(c)
        full_now = (c["seated"] == 3 and c["lid_on"] and c["handle_latched"] and c["carrier_tilt"] <= 3.0 and c["carrier_floor"] <= 0.003
                    and c["still"] and c["hand_gap"] >= 0.03)
        ok = (not self.spilled) and full_now
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= HOLD_STEPS:
            self.full = True
        if self.full and not (c["seated"] == 3 and c["lid_on"] and c["handle_latched"]):  # credit does not outlast the state that earned it
            self.full = False
        if self.spilled:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = lv


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration ------------------------------------------------------------------------------------------------------------------
def _report(ctx, label=""):
    c = TRIAL.last
    if not c:
        return
    ts = " ".join(f"{t['name'][-1]}:z{t['z'] * 1000:+6.1f} off{t['off'] * 1000:5.1f} tilt{t['tilt']:4.1f} bars{int(t['on_bars'])}" for t in c["tiers"])
    print(f"[tiffin] t {ctx.t:5.1f} {label} {ts} | seated {c['seated']} lid z{c['lid_z'] * 1000:+6.1f} bars{int(c['lid_on_bars'])} on{int(c['lid_on'])} | "
          f"handle z{c['handle_z'] * 1000:+6.1f} bars{int(c['handle_on_bars'])} catches {[round(v * 1000, 2) for v in c['catches']]} latched{int(c['handle_latched'])} | "
          f"carrier tilt {c['carrier_tilt']:.1f} | hands {c.get('hand_gap', 0) * 1000:5.0f} still {int(c['still'])} | max tilt {[round(v, 1) for v in TRIAL.max_tilt.values()]} "
          f"spilled {int(TRIAL.spilled)} score {TRIAL.score:.2f} full {int(TRIAL.full)}", flush=True)


def demo_script(ctx):
    import demokit as d

    yield from _demo(ctx, d)


def _demo(ctx, d):
    from yam_arena.constants import FINGER_OPEN

    for _ in range(3):
        yield
    mode = ctx.mode
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for _ in range(60):
            yield
        _report(ctx, "end:")
        return
    up = np.array([0.0, 0.0, 1.0])

    def rz(deg):
        a = math.radians(deg)
        return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])

    TILT = 20.0

    def grip(psi, tilt=TILT):
        """Fingers closing along (sin psi, -cos psi): across a tier's wall at its rim, at the side 90 degrees from its lugs. Leaned `tilt`
        degrees towards (cos psi, sin psi), about the closing axis, so the pads stay flat on what they hold."""
        return d._as_grip(rz(psi) @ d.tilted(tilt, 0.0).rot)

    def tier_pose(name):
        s = ctx.state(name)
        r = m.quat_matrix(s.quat_xyzw)
        return np.asarray(s.pos, float), _yaw(r)

    def wrap180(a):
        return (a + 180.0) % 360.0 - 180.0

    def carrier_pose():
        s = ctx.state("carrier")
        return np.asarray(s.pos, float), _yaw(m.quat_matrix(s.quat_xyzw))

    PINCH = 0.025  # the pads' tips this far below the rim
    TIP = 0.013  # the pads' tips below the TCP, along the approach axis

    def tcp_for(rim_point, g):
        """The TCP that puts the pads' tips PINCH below this rim point, for grip g."""
        return rim_point - up * PINCH - TIP * g.rot[:, 2]

    def wrap90(a):
        return (a + 90.0) % 180.0 - 90.0

    def transfer(arm, name, level_k, side):
        """Pinch a tier's rim at the side that will face the arm once seated (side -1: the carrier's -y side, the right arm; +1: +y, the
        left), lift it, bring it over the carrier with its lugs over the bars, lower it straight down onto the level below, let go."""
        c, psi_t = tier_pose(name)
        cpos, cyaw = carrier_pose()
        turn = wrap90(cyaw - psi_t)  # the tier turns by this (degrees) on the way, to end with its lugs on the carrier's x
        n0 = rz(cyaw - turn) @ np.array([0.0, side, 0.0])  # the pinch's direction from the tier's axis, now
        rim = c + n0 * (R_OUT - WALL / 2) + up * H_T
        g0 = grip(cyaw - turn)
        above = tcp_for(rim, g0) + up * 0.05
        yield from arm.move(above, grip=g0, finger=FINGER_OPEN, t=2.5, label=f"{name}: over the rim")
        yield from arm.servo(above, timeout=1.0, label=f"{name}: over the rim")
        yield from arm.move(tcp_for(rim, g0), t=1.2, label=f"{name}: round the rim")
        yield from arm.servo(tcp_for(rim, g0), timeout=1.0, label=f"{name}: round the rim")
        yield from arm.close(WALL, 48.0, t=0.6, label=f"{name}: pinch the rim")
        _report(ctx, f"{name} pinched:")
        z_carry = H_TIP + 0.012 + LUG_H - H_T  # the floor's height above the counter that keeps the lugs 6 mm over the bars' points
        lift_rim = TOP + z_carry + H_T
        here = arm.at()
        yield from arm.move(here + up * (lift_rim - rim[2]), t=2.5, label=f"{name}: lift")
        yield from arm.servo(np.r_[ctx.pos(name)[:2], TOP + z_carry], read=lambda: ctx.pos(name), tol=0.002, timeout=1.0, label=f"{name}: lift (droop)")
        _report(ctx, f"{name} lifted:")
        g1 = grip(cyaw)
        rim_goal = cpos + rz(cyaw) @ np.array([0.0, side * (R_OUT - WALL / 2), 0.0])
        rim_goal[2] = lift_rim
        yield from arm.move(tcp_for(rim_goal, g1) + up * 0.03, grip=g1, t=4.5, label=f"{name}: over the carrier")
        _report(ctx, f"{name} over:")

        def tier_centre():
            return ctx.pos(name)

        for k in range(3):  # closed loop: the tier's axis over the carrier's, its lugs square to the bars
            _, ty = tier_pose(name)
            cp, cy = carrier_pose()
            err = wrap90(ty - cy)
            if abs(err) > 0.4:
                yield from arm.turn(-err, t=0.5, label=f"{name}: square the lugs")
            yield from arm.servo(np.r_[cp[:2], TOP + z_carry], read=tier_centre, tol=0.0012, timeout=1.5, gain=0.5, label=f"{name}: lugs over the bars")
        _report(ctx, f"{name} aligned:")
        z_end = LEVEL_Z[level_k] + 0.002
        steps = int((2.5 + 0.5 * (3 - level_k)) * 20)
        for k in range(steps):  # straight down, holding the axis on the carrier's
            cp, _ = carrier_pose()
            goal = np.r_[cp[:2], TOP + z_carry + (z_end - z_carry) * (k + 1) / steps]
            err = goal - tier_centre()
            yield from arm.move(arm.at() + err * np.array([0.5, 0.5, 1.0]), t=0.05, label=f"{name}: down")
        yield from arm.wait(0.4)
        _report(ctx, f"{name} down:")
        yield from arm.open(t=0.5, label=f"{name}: let go")
        yield from arm.move(arm.at() + up * (PINCH + 0.015), t=0.8, label=f"{name}: fingers up out of the tier")
        out = rz(cyaw) @ np.array([0.0, side, 0.0])
        yield from arm.move(arm.at() + out * 0.06, t=0.8, label=f"{name}: away")
        _report(ctx, f"{name} placed:")

    def place_on_top(arm, name, grasp_z, close_w, force, z_clear, z_seat, press=0.0, tilt=TILT):
        """The lid or the handle: grasp at its centre (grasp_z above its origin), lift, bring it over the carrier turned so its lugs or
        eyes lie on the bars' line, lower it straight down until it rests, press `press` more, let go and lift clear."""
        s0 = ctx.state(name)
        c = np.asarray(s0.pos, float)
        yaw0 = _yaw(m.quat_matrix(s0.quat_xyzw))
        cpos, cyaw = carrier_pose()
        turn = wrap90(cyaw - yaw0)
        g0 = grip(cyaw - turn, tilt)
        tip = c + up * grasp_z
        tcp = tip - TIP * g0.rot[:, 2]
        yield from arm.move(tcp + up * 0.06, grip=g0, finger=FINGER_OPEN, t=2.5, label=f"{name}: over it")
        yield from arm.servo(tcp + up * 0.06, timeout=1.0, label=f"{name}: over it")
        yield from arm.move(tcp, t=1.2, label=f"{name}: down to it")
        yield from arm.servo(tcp, timeout=1.0, label=f"{name}: down to it")
        yield from arm.close(close_w, force, t=0.6, label=f"{name}: grasp")
        yield from arm.move(arm.at() + up * (z_clear + 0.03 - (c[2] - TOP)), t=2.0, label=f"{name}: lift")
        g1 = grip(cyaw, tilt)
        goal_tip = np.r_[cpos[:2], TOP + z_clear + 0.03 + grasp_z]
        yield from arm.move(goal_tip - TIP * g1.rot[:, 2], grip=g1, t=3.5, label=f"{name}: over the carrier")

        def centre():
            return ctx.pos(name)

        for k in range(3):
            st = ctx.state(name)
            cp, cy = carrier_pose()
            err = wrap90(_yaw(m.quat_matrix(st.quat_xyzw)) - cy)
            if abs(err) > 0.4:
                yield from arm.turn(-err, t=0.5, label=f"{name}: square it")
            yield from arm.servo(np.r_[cp[:2], TOP + z_clear], read=centre, tol=0.0012, timeout=1.5, gain=0.5, label=f"{name}: over the bars")
        _report(ctx, f"{name} aligned:")
        steps = 50
        for k in range(steps):
            cp, _ = carrier_pose()
            goal = np.r_[cp[:2], TOP + z_clear + (z_seat - z_clear) * (k + 1) / steps]
            err = goal - centre()
            yield from arm.move(arm.at() + err * np.array([0.5, 0.5, 1.0]), t=0.05, label=f"{name}: down")
        if press:
            yield from arm.move(arm.at() - up * press, t=0.8, label=f"{name}: press home")
            yield from arm.wait(0.3)
        yield from arm.wait(0.3)
        _report(ctx, f"{name} down:")
        yield from arm.open(t=0.5, label=f"{name}: let go")
        yield from arm.move(arm.at() + up * 0.05, t=0.8, label=f"{name}: up")
        _report(ctx, f"{name} placed:")

    l, r = ctx.left, ctx.right
    aside_r = np.array([0.20, -0.30, TOP + 0.22])
    aside_l = np.array([0.20, 0.30, TOP + 0.22])
    yield from transfer(r, "tier_a", 0, -1.0)
    yield from r.move(aside_r, grip=d.down(0.0), t=1.5, label="right aside")
    if mode == "probe":
        for k in range(40):
            yield
        _report(ctx, "end:")
        return
    yield from transfer(l, "tier_c", 1, 1.0)
    yield from l.move(aside_l, grip=d.down(0.0), t=1.5, label="left aside")
    yield from transfer(r, "tier_b", 2, -1.0)
    yield from r.move(aside_r, grip=d.down(0.0), t=1.5, label="right aside")
    lid_clear = H_TIP + 0.012  # the lid's floor this high clears the bars' points by 12 mm
    yield from place_on_top(l, "lid", LID_T + KNOB_H - 0.004, 2 * KNOB_R, 30.0, lid_clear, LEVEL_Z[3] + 0.001)
    yield from l.move(aside_l, grip=d.down(0.0), t=1.5, label="left aside")
    yield from place_on_top(r, "handle", (GRIP_Z0 + GRIP_Z1) / 2 + 0.004, 0.016, 30.0, H_TIP + 0.012, H_LID_TOP - 0.001, press=0.006, tilt=30.0)
    yield from r.move(aside_r, grip=d.down(0.0), t=1.5, label="right aside")
    for k in range(50):
        yield
        if k % 10 == 9:
            _report(ctx, "end:")
