"""Hang a push mower's grass catcher bag on the rod under its sprung rear chute cover, and let the cover down onto the bag.

The mower is a 20 in corded electric push mower modelled on the Greenworks 25022 (25.4 kg, 7 in front and 10 in rear wheels),
standing on the bench with its rear towards the robot. Its rear chute opening is closed by a chute cover hung on a steel rod
across the top of the opening; a torsion spring and the cover's own weight hold it shut. The manual's procedure: lift the
chute cover, and while holding it up, drop the bag's two hanger hooks over the rod; then the cover comes down onto the bag.

Model (all in code below):
- "mower": a free articulation (25.4 kg frame on four wheels) whose one link is the chute cover, a hinge on the rod's axis
  (joint 0 = shut, opening positive, stop at 150 degrees), sprung shut (K_FLAP N m/rad towards a target of -PRELOAD rad, so
  it presses on its stop when shut) and with gravity. The rod is a 10 mm steel collider across the opening, its ends in
  the chute's side walls; only ROD_HALF either side of the centre is exposed between the walls.
- "bag": one rigid body (fabric bag, wire frame, mouth collar, two hanger hooks of 6 mm wire, a carry handle loop on top).
  The hooks are open downwards; their saddle (the underside of each hook's top bar) rests on the rod when the bag hangs.
  The bag hangs only by contact: the hooks on the rod, its front face against the rear plate below the opening.
- Scoring reads the actual finite rod: each hook's saddle, in the mower's own frame, must sit on the rod's exposed span
  (no extrapolation of its axis), across it within the hook's jaw and resting on top of it within a few millimetres.
  The mower's own position is not scored: it is free, and every check is made in its frame.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demokit as d  # noqa: E402  (only its grip helpers: they let a Waypoint carry any TCP rotation)
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the grass bag back on the mower, ready to mow."
SUCCESS_HOLD_S = 3.0
TOP = TABLE_TOP[SCENE]

# -- the mower: its frame's origin is the centre of the rod (the chute cover's hinge axis); x towards the mower's front
RX, RZ = 0.40, 0.40  # the rod: 40 cm in front of the arm bases, 40 cm above the bench (the mower's wheels stand on it)
G = -RZ  # the bench in the mower's frame
MOWER_MASS = 25.4  # kg (56 lb, Greenworks 25022 manual)
R_REAR, R_FRONT = 0.127, 0.089  # 10 in and 7 in wheels (manual parts list)
ROD_R = 0.005  # 10 mm steel rod
ROD_HALF = 0.13  # exposed between the chute's side walls
ROD_ENDS = 0.15  # the rod's ends, in the side walls
OPEN_Z0, OPEN_Z1 = -0.115, -0.005  # the chute opening under the rod
OPEN_Y = 0.13
PLATE_X = -0.004  # the rear plate's face below the opening

# -- the chute cover (link "flap"), modelled shut: a 4 mm plate hanging from the rod, a 35 mm lip bent out along its lower edge
FLAP_W = 0.16  # half width: the cover (32 cm) overlaps the chute's side walls
FLAP_X0, FLAP_X1 = -0.011, -0.007  # the plate's outer and inner faces (shut)
FLAP_Z0, FLAP_Z1 = -0.110, -0.008  # its lower edge and its top (by the knuckles)
LIP_X0 = -0.042
LIP_Z0 = -0.114
NOTCH_Y0, NOTCH_Y1, NOTCH_Z = 0.050, 0.110, -0.060  # the two openings (60 x 52 mm) the bag's hooks pass through
K_FLAP = 0.25  # N m/rad
PRELOAD = 0.6  # rad of wind-up when shut: 0.15 N m on the stop
FLAP_MASS = 0.20
FLAP_MAX = 150.0

# -- the bag: its frame's origin is the hooks' saddle centre (where the rod's axis sits when it hangs); x towards the mower
BAG_X0, BAG_X1 = -0.372, -0.012  # the bag's body (fabric over the wire frame)
BAG_Y = 0.16
BAG_Z0, BAG_Z1 = -0.250, -0.025
COLLAR_X1 = 0.030  # the mouth collar that goes into the chute opening
COLLAR_Y = 0.115
COLLAR_Z0, COLLAR_Z1 = -0.105, -0.050
HOOK_Y = 0.08  # the two hanger hooks
WIRE = 0.003  # half the hooks' 6 mm wire
SADDLE_Z = 0.005  # the underside of a hook's top bar
JAW = 0.008  # the hook's jaw: inner faces at +-8 mm
HANDLE_X, HANDLE_Z, HANDLE_R, HANDLE_HALF = -0.15, 0.045, 0.010, 0.045
BAG_MASS = 1.0
BAG_COM = (HANDLE_X, 0.0, -0.13)

# -- starting layout: the bag lies on the bench behind the mower, mouth to it, hooks under the shut cover's lip
BAG_START = (RX - 0.100, 0.0, TOP - BAG_Z0 + 0.001)  # its hooks 10 cm short of the rod

LOOKS = {
    "green": m.Look(color=m.rgb(0.40, 0.66, 0.16), roughness=0.5),
    "black": m.Look(color=m.rgb(0.06, 0.06, 0.065), roughness=0.6),
    "dark": m.Look(color=m.rgb(0.02, 0.02, 0.02), roughness=0.9),
    "tyre": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.85),
    "hub": m.Look(color=m.rgb(0.20, 0.20, 0.21), roughness=0.5),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.9, roughness=0.35),
    "fabric": m.Look(color=m.rgb(0.09, 0.10, 0.09), roughness=0.95),
    "frame": m.Look(color=m.rgb(0.30, 0.31, 0.32), metallic=0.6, roughness=0.45),
    "grip": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.7),
}


def _ycyl(r, y0, y1, x, z, segments=32):
    return m.cylinder(r, y1 - y0, (x, (y0 + y1) / 2, z), axis="y", segments=segments)


def _span(lo, hi):
    return tuple(h - l for l, h in zip(lo, hi)), tuple((l + h) / 2 for l, h in zip(lo, hi))


def _boxpart(name, lo, hi, look, parts, cols, collide=True):
    size, c = _span(lo, hi)
    parts.append((name, m.box(size, c), look))
    if collide:
        cols.append(m.col_box(name + "_c", size, c))


# -- the mower ---------------------------------------------------------------------------------------------------------------


def _mower_frame():
    parts, cols = [], []
    # deck shell and the motor housing on it
    _boxpart("deck", (0.03, -0.215, G + 0.05), (0.60, 0.215, G + 0.21), "green", parts, cols)
    parts.append(("motor", m.cylinder(0.15, 0.15, (0.33, 0.0, G + 0.285)), "green"))
    cols.append(m.col_cylinder("motor_c", 0.15, 0.15, (0.33, 0.0, G + 0.285)))
    parts.append(("motor_cap", m.cylinder(0.11, 0.02, (0.33, 0.0, G + 0.37)), "black"))
    # the rear chute housing round the opening: the rear plate below it, side walls (which hold the rod's ends), a roof, the
    # back of the chute; the opening itself is dark
    _boxpart("rear_plate", (PLATE_X, -0.17, G + 0.05), (0.03, 0.17, OPEN_Z0 - 0.005), "black", parts, cols)
    for s, n in ((1, "l"), (-1, "r")):
        lo, hi = (PLATE_X, OPEN_Y, OPEN_Z0 - 0.005), (0.17, 0.17, 0.035)
        if s < 0:
            lo, hi = (lo[0], -hi[1], lo[2]), (hi[0], -lo[1], hi[2])
        _boxpart(f"jamb_{n}", lo, hi, "black", parts, cols)
    _boxpart("chute_roof", (0.03, -0.17, 0.012), (0.17, 0.17, 0.035), "black", parts, cols)
    _boxpart("chute_back", (0.16, -OPEN_Y, OPEN_Z0 - 0.005), (0.17, OPEN_Y, 0.012), "dark", parts, cols)
    _boxpart("chute_floor", (0.03, -OPEN_Y, OPEN_Z0 - 0.015), (0.16, OPEN_Y, OPEN_Z0 - 0.005), "dark", parts, cols)
    # the rod across the opening, its ends in the side walls
    parts.append(("rod", _ycyl(ROD_R, -ROD_ENDS, ROD_ENDS, 0.0, 0.0, 20), "steel"))
    cols.append(m.col_hull("rod_c", _ycyl(ROD_R, -ROD_ENDS, ROD_ENDS, 0.0, 0.0, 16)))
    # wheels: 10 in at the rear, 7 in at the front
    for name, x, y, r, w in (("wheel_rl", 0.10, 0.25, R_REAR, 0.045), ("wheel_rr", 0.10, -0.25, R_REAR, 0.045),
                             ("wheel_fl", 0.46, 0.235, R_FRONT, 0.04), ("wheel_fr", 0.46, -0.235, R_FRONT, 0.04)):
        z = G + r
        parts.append((name, _ycyl(r, y - w / 2, y + w / 2, x, z, 40), "tyre"))
        parts.append((name + "_hub", _ycyl(r * 0.45, y - w / 2 - 0.002, y + w / 2 + 0.002, x, z, 24), "hub"))
        cols.append(m.col_hull(name + "_c", _ycyl(r, y - w / 2, y + w / 2, x, z, 24)))
    # the handle, folded forward over the deck for storage (seen only)
    for s, n in ((1, "l"), (-1, "r")):
        parts.append((f"handle_{n}", m.tube([(0.02, s * 0.225, -0.10), (0.30, s * 0.225, 0.02), (0.62, s * 0.225, 0.06)], 0.011), "black"))
    parts.append(("handle_bar", _ycyl(0.011, -0.225, 0.225, 0.62, 0.06, 16), "black"))
    return parts, cols


def _flap_link():
    parts, cols = [], []
    _boxpart("plate", (FLAP_X0, -FLAP_W, FLAP_Z0), (FLAP_X1, FLAP_W, NOTCH_Z), "black", parts, cols)
    for name, y0, y1 in (("top_l", NOTCH_Y1, FLAP_W), ("top_m", -NOTCH_Y0, NOTCH_Y0), ("top_r", -FLAP_W, -NOTCH_Y1)):
        _boxpart(name, (FLAP_X0, y0, NOTCH_Z), (FLAP_X1, y1, FLAP_Z1), "black", parts, cols)
    _boxpart("lip", (LIP_X0, -FLAP_W, LIP_Z0), (FLAP_X1, FLAP_W, FLAP_Z0), "black", parts, cols)
    for name, y0, y1 in (("knuckle_l", 0.115, 0.150), ("knuckle_m", -0.04, 0.04), ("knuckle_r", -0.150, -0.115)):
        parts.append((name, _ycyl(0.0085, y0, y1, 0.0, 0.0, 20), "black"))
    com = ((FLAP_X0 + FLAP_X1) / 2 - 0.002, 0.0, (FLAP_Z0 + FLAP_Z1) / 2 - 0.004)
    return m.hinge("flap", "Y", (0.0, 0.0, 0.0), 0.0, FLAP_MAX, parts, cols, mass=FLAP_MASS, spring=K_FLAP, damping=0.02, friction=0.005,
                   effort=5.0, gravity=True, com=com, inertia=(0.0006, 0.0008, 0.0003))


_mp, _mc = _mower_frame()
MOWER = m.mechanism("mower", (RX, 0.0, TOP + RZ), [_flap_link()], LOOKS, frame_parts=_mp, frame_colliders=_mc, free_mass=MOWER_MASS,
                    frame_inertia=(0.95, 1.10, 1.35), friction=(1.0, 0.9), solver_iters=48)

# -- the grass bag -------------------------------------------------------------------------------------------------------------


def _bag():
    parts, cols = [], []
    _boxpart("body", (BAG_X0, -BAG_Y, BAG_Z0), (BAG_X1, BAG_Y, BAG_Z1), "fabric", parts, cols)
    _boxpart("collar", (BAG_X1, -COLLAR_Y, COLLAR_Z0), (COLLAR_X1, COLLAR_Y, COLLAR_Z1), "frame", parts, cols)
    # the wire frame's edges (seen): round the mouth, along the top
    fr = 0.004
    mouth = [(BAG_X1 + 0.002, -BAG_Y, BAG_Z1), (BAG_X1 + 0.002, BAG_Y, BAG_Z1), (BAG_X1 + 0.002, BAG_Y, BAG_Z0 + 0.01),
             (BAG_X1 + 0.002, -BAG_Y, BAG_Z0 + 0.01), (BAG_X1 + 0.002, -BAG_Y, BAG_Z1)]
    parts.append(("frame_mouth", m.tube(mouth, fr, 8), "frame"))
    for s, n in ((1, "l"), (-1, "r")):
        parts.append((f"frame_top_{n}", m.tube([(BAG_X1, s * BAG_Y, BAG_Z1 + 0.002), (BAG_X0 + 0.02, s * BAG_Y, BAG_Z1 + 0.002)], fr, 8), "frame"))
    # the two hanger hooks, open downwards: a top bar, a short front leg with a flared end, and the stem down to the frame
    for s, n in ((1, "l"), (-1, "r")):
        y = s * HOOK_Y
        top = ((-JAW - 2 * WIRE, y - WIRE, SADDLE_Z), (JAW + 2 * WIRE, y + WIRE, SADDLE_Z + 2 * WIRE))
        _boxpart(f"hook_{n}_top", *top, "frame", parts, cols)
        front = ((JAW, y - WIRE, -0.006), (JAW + 2 * WIRE, y + WIRE, SADDLE_Z))
        _boxpart(f"hook_{n}_front", *front, "frame", parts, cols)
        flare = m.tube([(JAW + WIRE, y, -0.006), (0.020, y, -0.010)], WIRE, 8)
        parts.append((f"hook_{n}_flare", flare, "frame"))
        cols.append(m.col_hull(f"hook_{n}_flare_c", flare))
        stem = m.tube([(-JAW - WIRE, y, SADDLE_Z + WIRE), (-0.022, y, BAG_Z1 - 0.001)], WIRE, 8)
        parts.append((f"hook_{n}_stem", stem, "frame"))
        cols.append(m.col_hull(f"hook_{n}_stem_c", stem))
    # the carry handle: a loop across the top, with a moulded round grip
    for s, n in ((1, "l"), (-1, "r")):
        leg = ((HANDLE_X - 0.005, s * HANDLE_HALF - 0.005, BAG_Z1), (HANDLE_X + 0.005, s * HANDLE_HALF + 0.005, HANDLE_Z))
        if s < 0:
            leg = ((leg[0][0], -HANDLE_HALF - 0.005, BAG_Z1), (leg[1][0], -HANDLE_HALF + 0.005, HANDLE_Z))
        _boxpart(f"handle_leg_{n}", *leg, "frame", parts, cols)
    grip = _ycyl(HANDLE_R, -HANDLE_HALF, HANDLE_HALF, HANDLE_X, HANDLE_Z, 24)
    parts.append(("handle_grip", grip, "grip"))
    cols.append(m.col_hull("handle_grip_c", _ycyl(HANDLE_R, -HANDLE_HALF, HANDLE_HALF, HANDLE_X, HANDLE_Z, 16)))
    return parts, cols


_bp, _bc = _bag()
BAG = m.free("bag", BAG_START, _bp, _bc, LOOKS, mass=BAG_MASS, friction=(0.9, 0.8), com=BAG_COM)

# -- distractors (SimReady), on the bench to the left, clear of the work
TOWEL = SimReady("shop_towel", pos=(0.08, 0.48, TOP), size=(0.205, 0.212, 0.034), mass=0.08,
                 package="consumable_microfiber_towel_folded_small_b01", usd="sm_consumable_microfiber_towel_folded_small_b01_01.usd", yaw=0.2)
MUG = SimReady("mug", pos=(0.30, 0.53, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
               usd="sm_dishware_mug_a01_01.usda", yaw=2.4)
LUBE = SimReady("lube_can", pos=(0.64, 0.46, TOP), size=(0.052, 0.104, 0.272), mass=0.35, package="consumable_lubricant_oil_multi_purpose_a01",
                usd="sm_consumable_lubricant_oil_multi_purpose_a01_01.usd", yaw=0.5)

OBJECTS = [MOWER, BAG, TOWEL, MUG, LUBE]

# -- the scorer ----------------------------------------------------------------------------------------------------------------

BAG_BOX = (np.array([BAG_X0, -BAG_Y, BAG_Z0]), np.array([COLLAR_X1, BAG_Y, HANDLE_Z + HANDLE_R]))  # the bag with its hooks and handle
FLAP_BOX = (np.array([LIP_X0, -FLAP_W, LIP_Z0]), np.array([FLAP_X1, FLAP_W, FLAP_Z1]))  # the cover, in its own (shut) frame
BAG_CLEAR = 0.03  # every fingertip point at least this far from the bag
FLAP_CLEAR = 0.05  # and from the cover
SEAT_X = 0.004  # the rod's axis within 4 mm of the saddle's centre line (the jaw leaves 3 mm either side)
SEAT_Z0, SEAT_Z1 = 0.003, 0.008  # the saddle 3 to 8 mm above the rod's axis: on the 5 mm-radius rod, at most 3 mm clear of it
TIP_GAP = 0.006  # the cover's tip within 6 mm of the bag's top
STILL_V, STILL_W = 0.01, 0.05  # m/s, rad/s
FLAP_STILL = math.radians(0.5)  # per control step
MOWER_TILT = 5.0  # degrees: all four wheels on the bench


def _box_gap(points, lo, hi) -> float:
    d_ = np.maximum(np.maximum(lo - points, points - hi), 0.0)
    return float(np.min(np.linalg.norm(d_, axis=1)))


def _ry(a: float) -> np.ndarray:
    """The cover's rotation about the rod (y) by its joint angle `a` (rad, opening positive), in the mower's frame."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def hooks_on_rod(mower, bag) -> tuple[bool, list[dict]]:
    """Each hook's saddle point, in the mower's frame, against the real, finite rod: on its exposed span (both edges of the
    hook's wire inside it), across it within the jaw, and resting on it."""
    out = []
    for s in (1.0, -1.0):
        p = mower.to_local(bag.to_world(np.array([[0.0, s * HOOK_Y, SADDLE_Z]])))[0]
        on_span = abs(p[1]) + WIRE <= ROD_HALF
        across = abs(p[0]) <= SEAT_X
        resting = SEAT_Z0 <= p[2] <= SEAT_Z1
        out.append({"p": p, "on_span": on_span, "across": across, "resting": resting, "ok": on_span and across and resting})
    return all(h["ok"] for h in out), out


def flap_on_bag(mower, bag) -> tuple[bool, float]:
    """The cover's tip (the inner face's lower edge, at its centre and both corners) lies on the bag's top: within TIP_GAP above it and
    over it."""
    a = mower.joint("flap")
    tips = np.array([[FLAP_X1, y, LIP_Z0] for y in (-FLAP_W + 0.01, 0.0, FLAP_W - 0.01)])
    w = mower.to_world((_ry(a) @ tips.T).T)
    q = bag.to_local(w)
    gap = q[:, 2] - BAG_Z1
    over = np.all((q[:, 0] > BAG_X0) & (q[:, 0] < BAG_X1) & (np.abs(q[:, 1]) < BAG_Y))
    return bool(over and np.all(gap >= -0.003) and np.all(gap <= TIP_GAP)), float(np.max(gap))


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # nothing in this task is an "ever happened" record; per-trial state is cleared here only
        self.prev_flap = None
        self.n = 0
        self.ok = False
        self.score = 0.0
        self.info = {}
        if env is not None:
            m.set_target(env, "mower", "flap", -PRELOAD)  # the spring's wind-up: the shut cover presses on its stop

    def step(self, env):
        mower, bag = m.state(env, "mower"), m.state(env, "bag")
        tips = m.fingertips(env)
        hung, hooks = hooks_on_rod(mower, bag)
        bag_gap = _box_gap(bag.to_local(tips), *BAG_BOX)
        a = mower.joint("flap")
        flap_local = (_ry(a).T @ mower.to_local(tips).T).T
        flap_gap = _box_gap(flap_local, *FLAP_BOX)
        still = float(np.linalg.norm(bag.lin_vel)) < STILL_V and float(np.linalg.norm(bag.ang_vel)) < STILL_W
        flap_still = self.prev_flap is not None and abs(a - self.prev_flap) < FLAP_STILL
        self.prev_flap = a
        resting, tip_gap = flap_on_bag(mower, bag)
        released = bag_gap >= BAG_CLEAR
        upright = mower.tilt_deg() <= MOWER_TILT  # the mower still on its four wheels (where it stands is not scored)
        hung_ok = hung and released and still and upright
        self.ok = bool(hung_ok and resting and flap_still and flap_gap >= FLAP_CLEAR)
        self.score = 1.0 if self.ok else (0.5 if hung_ok else 0.0)
        self.info = {"hooks": hooks, "bag_gap": bag_gap, "flap_gap": flap_gap, "flap_deg": math.degrees(a), "tip_gap": tip_gap, "still": still,
                     "resting": resting, "mower": mower.pos, "tilt": mower.tilt_deg()}
        self.n += 1
        if os.environ.get("MOWER_DEBUG") and self.n % 10 == 0:
            h = hooks
            print(f"[mower] step {self.n} flap {math.degrees(a):6.1f} hooks "
                  f"{np.round(h[0]['p'] * 1000, 1)} {np.round(h[1]['p'] * 1000, 1)} hung {hung} bag_gap {bag_gap:.3f} flap_gap {flap_gap:.3f} "
                  f"tip_gap {tip_gap * 1000:.1f} still {still} tilt {mower.tilt_deg():.1f} bagrot {np.round(np.degrees([math.atan2(bag.rotation()[2, 1], bag.rotation()[2, 2]), -math.asin(max(-1.0, min(1.0, bag.rotation()[2, 0]))), math.atan2(bag.rotation()[1, 0], bag.rotation()[0, 0])]), 1)} bag {np.round(bag.pos, 3)} mower {np.round(mower.pos, 4)} score {self.score}", flush=True)
            lt, rt = tips[:12], tips[12:]
            print(f"[tips] L mean {np.round(lt.mean(0) - [0, 0, TOP], 3)} zmin {lt[:, 2].min() - TOP:.3f} zmax {lt[:, 2].max() - TOP:.3f} xmax {lt[:, 0].max():.3f} | "
                  f"R mean {np.round(rt.mean(0) - [0, 0, TOP], 3)} zmin {rt[:, 2].min() - TOP:.3f} zmax {rt[:, 2].max() - TOP:.3f}", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.ok)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, a fixed plan from the published starting layout) ------------------------------------------

ROLL = math.radians(90.0)
GRIP_PHI = 30.0  # the right hand's fingers point 30 degrees left of forward, level, and close up and down across the handle's round grip
GRIP_AXIS = np.array([math.cos(math.radians(GRIP_PHI)), math.sin(math.radians(GRIP_PHI)), 0.0])
GRIP_X = d._as_grip(axis_angle_matrix(GRIP_AXIS, ROLL) @ d.level(GRIP_PHI).rot)
HALF_OPEN = 0.6 * FINGER_OPEN  # pads 56 mm apart: room for the 20 mm grip, clear of the bag's top below
GRIP_BAR = d.level(-75.0)  # the left hand: fingers level, pointing right and 15 deg forward; closed, a 9 mm x 36 mm bar nearly parallel to the rod
HINGE = np.array([RX, 0.0, TOP + RZ])
BAR_Y = 0.145  # the left TCP's y: the closed fingers (y 0.132 to 0.175) under the cover's left end, the hand's body beyond the mower's rear housing (0.17)
SAG_Z, SAG_Y = 0.025, -0.012
HOLD_DEG = 120.0  # the cover held here while the bag goes in under it
LOW_DEG = 96.0  # lowered to here (the bar just above the hung bag's top), then the bar slides out and the cover drops onto the bag


def _corner(theta_deg: float) -> np.ndarray:
    """The cover's lower inner corner (where the lip meets the inner face), in the world, with the cover at `theta_deg`."""
    a = math.radians(theta_deg)
    return HINGE + _ry(a) @ np.array([FLAP_X1, 0.0, LIP_Z0])


def _bar(theta_deg: float, y: float = BAR_Y) -> tuple:
    """The left TCP for the cover at `theta_deg`: the top of the closed fingers' bar placed a few millimetres inside the cover (4 mm
    into the lip's underside while it is nearly shut, 3 mm into the inner face near its lower edge from 45 degrees), so the hand drives
    the cover round and the cover rests on the fingers."""
    f = min(max((theta_deg - 20.0) / 25.0, 0.0), 1.0)
    s_local = np.array([-0.024 + 0.014 * f, 0.0, -0.110 + 0.006 * f])
    top = HINGE + _ry(math.radians(theta_deg)) @ s_local
    return (float(top[0]), float(y), float(top[2] - 0.018))


def _handle(origin) -> tuple:
    return (float(origin[0] + HANDLE_X), float(origin[1]), float(origin[2] + HANDLE_Z))


def _lead_in(arm: str, pos, grip, t: float, label: str, finger: float, steps: int = 6) -> list:
    """Waypoints from the arm's ready pose to `pos` with `grip` that follow a straight line in joint space (the forward kinematics of
    evenly spaced joint angles), so the harness's warm-started IK stays on one branch: a straight Cartesian move to a sideways grip
    otherwise ends 30-50 mm off. Pure kinematics of the published layout; no simulator state."""
    base = station.BASES[arm]
    q_rot = ik.matrix_to_quat_xyzw(grip.rot)
    best = None
    for init in (READY, (0, 1.5, 1.0, 0.5, 0, 0), (0, 2.0, 1.5, 0.3, 0, 0), (0, 1.0, 0.5, 0.5, 0, 0), (0, 1.5, 1.0, 0.0, 1.5, 0.0),
                 (0, 1.5, 1.0, 0.0, -1.5, 0.0), (0, 1.5, 1.0, 0.0, 1.5, 1.5), (0, 1.5, 1.0, 0.0, -1.5, -1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), q_rot, np.asarray(init, float), base_pos=base, iters=300)
        if pe < 5e-4 and re < math.radians(0.2):
            dist = float(np.abs(q - np.asarray(READY)).max())
            if best is None or dist < best[0]:
                best = (dist, q)
    q1, q0 = best[1], np.asarray(READY, float)
    out = []
    for k in range(1, steps):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, base_pos=base)
        out.append(Waypoint(t / steps, tuple(float(v) for v in p), finger, f"{label} ({k}/{steps})", d._as_grip(np.asarray(r))))
    out.append(Waypoint(t / steps, tuple(float(v) for v in pos), finger, label, grip))
    return out


def demo():
    """Left: brings its closed fingers in under the cover's lip from behind and carries the cover up round its hinge to 120 degrees,
    holding it there on the fingers. Right: takes the bag's handle from behind (fingers either side of the grip), lifts the bag so
    the hooks clear the rod, carries it in under the held cover (the hooks pass through the cover's openings), lowers the hooks
    onto the rod and lets go. Left: lowers the cover to 96 degrees and slides the fingers out sideways past the cover's end, so the
    cover drops onto the bag's top; both hands move clear."""
    o0 = np.array(BAG_START)
    hang = HINGE.copy()
    lift = 0.026  # hooks 11 mm clear of the rod's top while carried
    b0 = _bar(0.0)
    left = _lead_in("left", (b0[0] - 0.05, BAR_Y, b0[2] + 0.05), GRIP_BAR, 3.0, "left over the bag's mouth, level, fingers to the right", FINGER_CLOSED)
    left += [Waypoint(1.2, (b0[0] - 0.05, BAR_Y, b0[2] - 0.013), FINGER_CLOSED, "down to below the lip", GRIP_BAR),
             Waypoint(1.5, (b0[0], BAR_Y, b0[2] - 0.013), FINGER_CLOSED, "fingers in under the lip", GRIP_BAR),
             Waypoint(0.6, b0, FINGER_CLOSED, "up to the lip", GRIP_BAR)]
    for th in range(8, int(HOLD_DEG) + 1, 8):
        left.append(Waypoint(0.45, _bar(float(th)), FINGER_CLOSED, f"carry the cover up {th}", GRIP_BAR))
    t_hold_from = sum(w.duration for w in left)

    h0 = _handle(o0)
    back = -0.07 * GRIP_AXIS
    right = _lead_in("right", (h0[0] + back[0], h0[1] + back[1], h0[2] + 0.06), GRIP_X, 3.0, "right up, behind the handle", HALF_OPEN)
    right += [Waypoint(1.0, (h0[0] + back[0], h0[1] + back[1], h0[2]), HALF_OPEN, "level with the grip", GRIP_X),
              Waypoint(1.5, h0, HALF_OPEN, "fingers either side of the grip", GRIP_X),
              Waypoint(0.8, h0, FINGER_CLOSED, "grip the handle", GRIP_X)]
    wait = max(0.5, t_hold_from - sum(w.duration for w in right) + 0.5)
    right.append(Waypoint(wait, h0, FINGER_CLOSED, "wait for the cover", GRIP_X))
    sag = np.array([0.0, SAG_Y, SAG_Z])  # the loaded arm's measured sag (trial 1: 25 mm down, 15 mm left), made up while it carries
    up = np.array([o0[0], 0.0, hang[2] + lift])
    right += [Waypoint(3.0, tuple(np.array(_handle(up)) + sag), FINGER_CLOSED, "lift the bag", GRIP_X),
              Waypoint(3.0, tuple(np.array(_handle(np.array([hang[0], 0.0, hang[2] + lift]))) + sag), FINGER_CLOSED, "carry it in under the cover", GRIP_X),
              Waypoint(2.0, _handle(hang + np.array([0.0, 0.0, -0.004])), FINGER_CLOSED, "hooks down onto the rod", GRIP_X),
              Waypoint(0.8, _handle(hang + np.array([0.0, 0.0, -0.004])), HALF_OPEN, "let go", GRIP_X),
              Waypoint(1.5, tuple(np.array(_handle(hang + np.array([0.0, 0.0, -0.004]))) + 1.2 * back), HALF_OPEN, "fingers back off the grip", GRIP_X),
              Waypoint(1.5, (0.12, -0.15, TOP + 0.45), HALF_OPEN, "right clear", GRIP_X)]
    t_released = sum(w.duration for w in right)
    hold = max(0.3, t_released - sum(w.duration for w in left) + 0.3)
    left.append(Waypoint(hold, _bar(HOLD_DEG), FINGER_CLOSED, "hold the cover up", GRIP_BAR))
    for th in (110.0, 102.0, LOW_DEG):
        left.append(Waypoint(0.7, _bar(th), FINGER_CLOSED, f"lower the cover {th:.0f}", GRIP_BAR))
    lo = _bar(LOW_DEG)
    out = 0.04 * np.array([-math.cos(math.radians(75.0)), math.sin(math.radians(75.0)), 0.0])  # back along the fingers
    left += [Waypoint(1.5, (lo[0] + out[0], lo[1] + out[1], lo[2]), FINGER_CLOSED, "slide the fingers out sideways past the cover's end", GRIP_BAR),
             Waypoint(1.5, (lo[0] + out[0] - 0.06, lo[1] + out[1], lo[2] + 0.04), FINGER_CLOSED, "left clear", GRIP_BAR)]
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 4.5
    for plan in (left, right):
        last = plan[-1]
        plan.append(Waypoint(total - sum(w.duration for w in plan), last.pos, last.finger, "wait", last.yaw))
    return {"left": left, "right": right}
