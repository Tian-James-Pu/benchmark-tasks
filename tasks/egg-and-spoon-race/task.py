"""Egg-and-spoon race: carry the egg on the spoon, holding the spoon only by the end of its handle, round the turning
marker (an upturned paper cup) and tip the egg into the finish bowl, without touching the egg.

The scene (world frame: x away from the arms, y to the left, z up; metres):
- Spoon ("spoon"): a 22.5 cm wooden race spoon (SimReady kitchenware_spoon_c01, 25 g), lying bowl-up along +y with the
  egg in its bowl. It collides as the signed distance field of its mesh, so its bowl is a real 3.9 cm wide, 5 mm deep
  dish and its handle a 1.1 cm round rod.
- Egg ("egg"): a wooden practice egg, 45 g, 58 x 46 mm (SimReady food_egg_chicken_a01, scaled 0.72), lying lengthwise in
  the spoon's bowl. It collides as its convex hull and rolls freely (PhysX has no rolling resistance).
- Turning marker ("marker"): a paper cup upside down (SimReady dishware_cup_paper_a01, 8 cm rim, 11 cm tall), fixed.
- Finish bowl ("bowl"): a 17.3 cm ceramic bowl, 6 cm tall (SimReady kitchenware_bowl_b01, 350 g), loose.
- Distractors that scoring ignores: a plastic tumbler, a roll of tape and a pencil (SimReady, loose).

Scoring (`Race`; task.json says the same):
- On the spoon: the egg's centre, in the spoon's frame, within ON_SPOON (3 cm) of where it settled at the start. The
  first step it is not, the egg has left the spoon, for good.
- Rounding the marker: seen from above, the egg's centre sweeps ROUND_DEG (270 degrees) round the marker's axis,
  counted only on steps when it is on the spoon, at least LIFT (8 cm) above the table and ROUND_MIN to ROUND_REACH (6 to
  20 cm) from the axis: past the marker on one side, behind it and back along the other.
- Finish zone: the egg's centre within FINISH_R (10 cm), horizontally, of the bowl's centre.
- In the bowl: the egg off the spoon, its centre within IN_BOWL_R (6.5 cm) of the bowl's centre and below its rim,
  still (under 1 cm/s and 0.5 rad/s), every fingertip point at least CLEAR (3 cm) and the spoon at least SPOON_CLEAR
  (1.5 cm) from its surface.
- Fouls, recorded across steps and cleared only by mechkit's reset hook: a fingertip point within TOUCH (1 cm) of the
  egg's surface; a fingertip point within 1 cm of the spoon anywhere but its handle's last 7 cm; the bowl moved more
  than BOWL_MOVED (3 cm).
- Score: 0 after any foul, if the egg left the spoon before rounding the marker, or if the marker was never rounded;
  1.0 once the egg has rounded the marker, left the spoon inside the finish zone and been in the bowl for 1 s (and from
  then on unless a foul follows); 0.5 if it rounded the marker and is on the spoon inside the finish zone now, or left
  the spoon inside the finish zone; 0.25 if it rounded the marker but left the spoon outside the finish zone, or is on
  the spoon outside the zone.

    python demokit.py task.py OUT            (README)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402  importing it hooks yam_arena (on_step, on_reset, fingertips)
import simready as _sr  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Run the egg-and-spoon race: holding the spoon only by the end of its handle, carry the egg round the upturned cup "
               "and tip it into the bowl, without touching the egg.")
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the spoon: SimReady kitchenware_spoon_c01, 22.5 cm long; its file's frame has +y towards the bowl, origin on the table
SPOON_XY = (0.20, -0.15)
SPOON_MASS = 0.025
END_Y = -0.1438  # the handle's end, along the spoon (its file's frame)
HANDLE_END_ZONE = 0.07  # "the end of its handle": its last 7 cm
SPOON_HALF_W = 0.0195
GRIP_Y = END_Y + 0.040  # where the demo grips: 4 cm from the end

# -- the egg: SimReady food_egg_chicken_a01 (63 x 80 mm in its file), scaled to a large egg's 45 x 58 mm
EGG_SCALE = 0.72
EGG_R = 0.0317 * EGG_SCALE  # half-width
EGG_HALF_L = 0.0400 * EGG_SCALE  # half-length
EGG_MASS = 0.045  # a wooden practice egg of that size (beech, 0.72 g/cm3)
EGG_IN_SPOON = (0.0, 0.040, 0.0300)  # the egg's centre in the spoon's frame where it is laid (it settles about 1 mm lower)

# -- the course
MARKER_XY = (0.35, -0.10)
MARKER_H = 0.110
BOWL_XY = (0.24, 0.05)
BOWL_TOP = 0.060  # the bowl's rim above the table

# -- scoring thresholds
ON_SPOON = 0.03
LIFT = 0.08
ROUND_DEG = 270.0
ROUND_REACH = 0.20
ROUND_MIN = 0.06  # round it, not over it: the egg's centre at least 6 cm from the cup's axis (its rim is 4 cm)
FINISH_R = 0.10
IN_BOWL_R = 0.065
TOUCH = 0.01
CLEAR = 0.03
BOWL_MOVED = 0.03
SPOON_CLEAR = 0.015


def _yaw_quat(deg):
    h = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


SPOON = SimReady("spoon", pos=(SPOON_XY[0], SPOON_XY[1], TOP + 0.0005), size=(0.039, 0.225, 0.014), mass=SPOON_MASS,
                 package="kitchenware_spoon_c01", usd="sm_kitchenware_spoon_c01_01.usd")
# The egg's rigid body sits at its centre, turned 90 degrees about x in its file, and a reset places that body: so `pos` is the
# egg's centre and `quat_xyzw` lays its long axis along y.
EGG = SimReady("egg", pos=(SPOON_XY[0] + EGG_IN_SPOON[0], SPOON_XY[1] + EGG_IN_SPOON[1], TOP + EGG_IN_SPOON[2]), size=(0.045, 0.058, 0.045),
               mass=EGG_MASS, package="food_egg_chicken_a01", usd="sm_food_egg_chicken_a01_01.usd", scale=EGG_SCALE, convex=True,
               quat_xyzw=(math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))
MARKER = SimReady("marker", pos=(MARKER_XY[0], MARKER_XY[1], TOP + MARKER_H), size=(0.080, 0.080, 0.110), package="dishware_cup_paper_a01",
                  usd="sm_dishware_cup_paper_a01_01.usd", quat_xyzw=(1.0, 0.0, 0.0, 0.0))  # upside down, fixed (taped)
BOWL = SimReady("bowl", pos=(BOWL_XY[0], BOWL_XY[1], TOP), size=(0.173, 0.173, 0.060), mass=0.35, package="kitchenware_bowl_b01",
                usd="sm_kitchenware_bowl_b01_01.usd")
# distractors (scoring ignores them)
TUMBLER = SimReady("tumbler", pos=(0.42, 0.22, TOP), size=(0.064, 0.064, 0.100), mass=0.03, package="dishware_plastic_cup_a01",
                   usd="sm_dishware_plastic_cup_a01_01.usd")
TAPE = SimReady("tape", pos=(0.31, 0.25, TOP + 0.0095), size=(0.055, 0.055, 0.019), mass=0.06, package="misc_tape_refill_a01",
                usd="sm_misc_tape_refill_a01_01.usd", quat_xyzw=(math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))  # lying flat
PENCIL = SimReady("pencil", pos=(0.17, 0.21, TOP + 0.0042), size=(0.199, 0.008, 0.008), mass=0.006, package="stationery_pencil_watercolor_red_a01",
                  usd="sm_stationery_pencil_watercolor_red_a01_01.usd", quat_xyzw=_yaw_quat(25.0))

OBJECTS = [SPOON, EGG, MARKER, BOWL, TUMBLER, TAPE, PENCIL]

SPOON_FRICTION = (1.0, 0.9)  # static, dynamic of the spoon's wood (PhysX averages it with the other surface's)
_plain_prepare = _sr._prepare


def _prepare(prim, spec):
    """simready's set-up, then the spoon's friction: SimReady authors 0.5 for this wood, which PhysX averages with the gripper pads'
    0.5; dry wood on rubber pads is nearer 0.75, which this gives."""
    _plain_prepare(prim, spec)
    if spec.name != "spoon":
        return
    from pxr import Usd, UsdPhysics

    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.MaterialAPI):
            mat = UsdPhysics.MaterialAPI(p)
            mat.CreateStaticFrictionAttr(SPOON_FRICTION[0])
            mat.CreateDynamicFrictionAttr(SPOON_FRICTION[1])


_sr._prepare = _prepare


# -- reading poses -------------------------------------------------------------------------------------------------------
def _np(t):
    return t.detach().cpu().numpy().astype(float)


def pose(env, name):
    """A prop's rigid-body pose: position (m) and rotation matrix. For the spoon that is its file's frame; for the egg, its centre."""
    base = env.unwrapped
    p = _np(base.arena_world.get_pose_w(name)[0])
    return p[:3] - _np(base.scene.env_origins[0]), _rot(p[3:])


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def egg_gap(egg_p, egg_R, points):
    """Approximate distance of each point from the egg's surface (an ellipsoid; its long axis is its body's z)."""
    local = (np.asarray(points) - egg_p) @ egg_R
    radii = np.array([EGG_R, EGG_R, EGG_HALF_L])
    k = np.linalg.norm(local / radii, axis=1)
    return (k - 1.0) * np.linalg.norm(local, axis=1) / np.maximum(k, 1e-9)


SPOON_SAMPLES = np.array([[0.0, 0.045, 0.008], [0.0, 0.081, 0.009], [SPOON_HALF_W, 0.045, 0.012], [-SPOON_HALF_W, 0.045, 0.012],
                          [0.0, 0.020, 0.010], [0.0, 0.070, 0.008]])  # points on the spoon's bowl, its tip and neck (its file's frame)


def spoon_foul_points(spoon_p, spoon_R, points):
    """Fingertip points within 1 cm of the spoon anywhere but its handle's last 7 cm (its slab, past the end zone)."""
    local = (np.asarray(points) - spoon_p) @ spoon_R
    near = (np.abs(local[:, 0]) <= SPOON_HALF_W + TOUCH) & (local[:, 2] >= -TOUCH) & (local[:, 2] <= 0.014 + TOUCH)
    beyond = (local[:, 1] > END_Y + HANDLE_END_ZONE) & (local[:, 1] <= 0.081 + TOUCH)
    return near & beyond


# -- the tracker ----------------------------------------------------------------------------------------------------------
class Race:
    """The race's record. Fouls and the egg's leaving the spoon are recorded across steps and cleared only in `reset`."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.ref = None  # the egg's settled place in the spoon (spoon frame), read at the first step
        self.bowl0 = None
        self.left = False  # the egg has left the spoon
        self.left_rounded = False
        self.left_in_zone = False
        self.angle = None
        self.sweep = 0.0
        self.rounded = False
        self.zone = False  # carried on the spoon into the finish zone after rounding
        self.touched = False
        self.bad_grip = False
        self.bowl_moved = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}

    def step(self, env):
        from yam_arena.scene import object_states

        sp, sR = pose(env, "spoon")
        ep, eR = pose(env, "egg")
        bowl = object_states(env, ("bowl",))["bowl"]
        egg_state = object_states(env, ("egg",))["egg"]
        bp = np.array([float(bowl.pos[0]), float(bowl.pos[1])])
        tips = m.fingertips(env)
        local = sR.T @ (ep - sp)
        if self.ref is None:
            self.ref, self.bowl0 = local.copy(), bp.copy()
        on = (not self.left) and float(np.linalg.norm(local - self.ref)) <= ON_SPOON
        gap = egg_gap(ep, eR, tips)
        if float(gap.min()) <= TOUCH:
            self.touched = True
        if bool(spoon_foul_points(sp, sR, tips).any()):
            self.bad_grip = True
        if float(np.linalg.norm(bp - self.bowl0)) > BOWL_MOVED:
            self.bowl_moved = True
        to_bowl = float(np.hypot(ep[0] - bp[0], ep[1] - bp[1]))

        # rounding the marker: the egg's sweep about its axis, on the spoon and lifted
        rel = ep[:2] - np.array(MARKER_XY)
        a = math.atan2(rel[1], rel[0])
        if self.angle is not None and on and ep[2] >= TOP + LIFT and ROUND_MIN <= float(np.linalg.norm(rel)) <= ROUND_REACH:
            d = (a - self.angle + math.pi) % (2 * math.pi) - math.pi
            self.sweep += d
            if abs(math.degrees(self.sweep)) >= ROUND_DEG:
                self.rounded = True
        self.angle = a

        self.zone = bool(on and self.rounded and to_bowl <= FINISH_R)  # on the spoon in the finish zone now
        if not self.left and not on:
            self.left, self.left_rounded, self.left_in_zone = True, self.rounded, to_bowl <= FINISH_R

        foul = self.touched or self.bad_grip or self.bowl_moved
        spoon_gap = float(egg_gap(ep, eR, sp + SPOON_SAMPLES @ sR.T).min())
        in_bowl = (self.left and self.left_rounded and self.left_in_zone and to_bowl <= IN_BOWL_R and ep[2] < TOP + BOWL_TOP
                   and egg_state.resting(0.01, 0.5) and float(gap.min()) >= CLEAR and spoon_gap >= SPOON_CLEAR)
        self.hold = self.hold + 1 if in_bowl and not foul else 0
        self.full = self.full or (self.hold >= HOLD_STEPS and not foul)
        if foul:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif self.left and not self.left_rounded:
            self.score = 0.0
        elif self.left_rounded and not self.left_in_zone:
            self.score = 0.25
        elif (self.left and self.left_in_zone) or self.zone:
            self.score = 0.5
        elif self.rounded:
            self.score = 0.25
        else:
            self.score = 0.0
        tilt = math.degrees(math.asin(max(-1.0, min(1.0, float(sR[2, 1])))))
        self.last = {"on": bool(on), "local": local.round(4).tolist(), "sweep_deg": round(math.degrees(self.sweep), 1), "rounded": self.rounded,
                     "zone": self.zone, "left": self.left, "to_bowl": round(to_bowl, 3), "egg_z": round(float(ep[2] - TOP), 3),
                     "pitch_deg": round(tilt, 1), "gap": round(float(gap.min()), 3), "spoon_gap": round(spoon_gap, 3), "touched": self.touched, "bad_grip": self.bad_grip,
                     "bowl_moved": self.bowl_moved, "hold": self.hold, "score": self.score}


RACE = Race()
m.on_step(RACE.step)
m.on_reset(RACE.reset)


def success(objects) -> bool:
    return bool(RACE.full)


def score(objects) -> float:
    return float(RACE.score)


# -- the demonstration (closed loop, demokit; right arm) ------------------------------------------------------------------
GRIP_Z = 0.009  # the TCP above the table when gripping: the pads close round the handle's middle (it is 1 cm thick)
CARRY_Z = 0.17  # the egg's centre above the table while carrying: the handle clears the 11 cm marker


def demo_script(ctx):
    import demokit as d

    r = ctx.right
    env = ctx.emb._env
    mode = ctx.mode
    if mode == "none":
        yield from r.wait(1.0)
        return

    def spoon():
        return pose(env, "spoon")

    def egg():
        return pose(env, "egg")[0]

    fast = {"careless": 4.0, "reckless": 8.0}.get(mode, 1.0)
    sp, sR = spoon()
    grip_y = GRIP_Y if mode != "midgrip" else -0.02
    g_w = sp + sR @ np.array([0.0, grip_y, 0.0])
    g = d.down(90)
    yield from r.move((g_w[0], g_w[1], TOP + 0.10), grip=g, t=2.0, label="over the handle's end")
    grip_z = GRIP_Z + (0.006 if mode == "highgrip" else 0.0)  # the careless grip: the pads 6 mm higher, over the handle's top half
    yield from r.move((g_w[0], g_w[1], TOP + grip_z), t=1.5, label="down round the handle")
    yield from r.close(0.011, force=40.0, label="grip the handle")
    yield from r.wait(0.3)

    if mode == "slide":  # the shortcut: drag the spoon on the table out and behind the cup, lift it there, and carry it back
        for bx, by in ((0.33, -0.182), (0.43, -0.10)):
            sp, sR = spoon()
            b = sp + sR @ np.array([0.0, 0.045, 0.0])
            c = r.at()
            yield from r.move((c[0] + bx - b[0], c[1] + by - b[1], TOP + GRIP_Z + 0.003), t=5.0, label="drag")
        g_w = r.at()
    if mode == "touch":  # the shortcut: the other arm steadies the egg as the spoon is lifted
        l = ctx.left
        e = egg()
        yield from l.move((e[0], e[1] + 0.01, e[2] + 0.06), grip=d.down(0), finger=0.0, t=2.5, label="over the egg")
        yield from l.move((e[0], e[1] + 0.01, e[2] + EGG_R + 0.012), t=1.0, label="on the egg")
    # lift, slowly
    yield from r.move((g_w[0], g_w[1], TOP + 0.05), t=2.5 / fast, label="lift")
    if mode == "touch":
        yield from ctx.left.move((0.25, 0.25, TOP + 0.25), t=2.0, label="away")
    yield from r.move((g_w[0], g_w[1], TOP + CARRY_Z - 0.02), t=3.0 / fast, label="lift")

    def level(n=2):  # turn the grip against the spoon's droop in the pads, read from the spoon's pose
        for _ in range(n):
            _, sR = spoon()
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(sR[2, 1])))))  # the bowl's end up, positive
            roll = math.degrees(math.asin(max(-1.0, min(1.0, float(sR[2, 0])))))
            yield from r.turn(-pitch, axis=(1.0, 0.0, 0.0), t=1.0, label="level: pitch")
            yield from r.turn(roll, axis=(0.0, 1.0, 0.0), t=0.6, label="level: roll")
            yield from r.wait(0.4)

    yield from level()
    if mode.startswith("cal"):  # calibration: tilt slowly (2 degrees a second) until the egg leaves its place in the bowl
        axis = (1.0, 0.0, 0.0) if mode == "cal_pitch" else (0.0, 1.0, 0.0)
        sign = -1.0
        e0 = pose(env, "egg")[0]
        sp0, sR0 = spoon()
        l0 = sR0.T @ (e0 - sp0)
        for k in range(300):
            yield from r.turn(sign * 0.1, axis=axis, t=0.05, label="calibrate")
            sp1, sR1 = spoon()
            l1 = sR1.T @ (pose(env, "egg")[0] - sp1)
            if k % 10 == 0 or float(np.linalg.norm(l1 - l0)) > 0.005:
                print(f"[cal] {mode} commanded {0.1 * (k + 1):.1f} deg, spoon pitch {math.degrees(math.asin(float(sR1[2, 1]))):.1f} "
                      f"roll {math.degrees(math.asin(float(sR1[2, 0]))):.1f}, egg moved {np.linalg.norm(l1 - l0) * 1000:.1f} mm", flush=True)
            if float(np.linalg.norm(l1 - l0)) > 0.03:
                break
        return

    def carry_to(ex, ey, ez, t):
        e = egg()
        c = r.at()
        goal = (c[0] + ex - e[0], c[1] + ey - e[1], c[2] + ez - e[2])
        if mode.startswith("jerky"):  # a step to full speed and a dead stop: no easing in or out
            v = float(mode[5:] or 0.3)
            yield from r.follow(lambda: np.asarray(goal), t=t, speed=v, label="jerky carry")
            r.q = r.q.copy()
            return
        yield from r.move(goal, t=t / fast, label=f"carry to {ex:.2f},{ey:.2f}")

    mx, my = MARKER_XY
    if mode == "direct":  # the shortcut: straight to the bowl, not round the marker
        path = []
    elif mode == "slide":  # the rest of the way from behind the cup, lifted
        path = [(mx + 0.056, my + 0.059, 2.0), (mx - 0.01, my + 0.082, 2.0), (BOWL_XY[0] + 0.07, BOWL_XY[1] - 0.04, 2.5)]
    else:
        path = [(mx - 0.02, my - 0.082, 3.0), (mx + 0.056, my - 0.059, 2.5), (mx + 0.08, my, 2.0), (mx + 0.056, my + 0.059, 2.0),
                (mx - 0.01, my + 0.082, 2.0), (BOWL_XY[0] + 0.07, BOWL_XY[1] - 0.04, 2.5)]
    for ex, ey, t in path:
        yield from carry_to(ex, ey, TOP + CARRY_Z, t)
    yield from carry_to(BOWL_XY[0], BOWL_XY[1] - 0.015, TOP + 0.135, 2.5)
    yield from level(1)
    # tip: the bowl's end down until the egg rolls off into the bowl
    yield from r.turn(-38.0, axis=(1.0, 0.0, 0.0), t=3.0, label="tip")
    yield from r.wait(1.0)
    yield from r.turn(30.0, axis=(1.0, 0.0, 0.0), t=1.5, label="tip back")
    c = r.at()
    yield from r.move((c[0], c[1], TOP + 0.20), t=1.5, label="up")
    yield from r.move((0.26, -0.24, TOP + 0.20), t=2.0, label="away")
    yield from r.wait(1.5)
