"""Drive a toy remote-control car into its garage with its two-lever remote.

The car sits on the table out of the arms' reach, nose to the far edge. Its garage is a shoebox with a door cut in one end, at the right
of the table, its door facing the car's side. The remote lies in a cradle between the arms: the left lever is the throttle (push for
forward, pull for reverse) and the right lever is the steering (left / right). Both levers are spring-centred switches, as on most toy
remotes: past 10 degrees the motor or the steering is fully on, inside it fully off. The car turns no tighter than a 34 cm radius, so it
can't swing round to face the door in the room it has: it needs a forward-and-back shunt, then a straight run through a 13 cm door with
2 cm to spare on each side.

The car is a jointed chain (slide x, slide y, turn z) whose joint velocities follow a toy-car model of the remote's switch states: motor
spin-up and coast-down, steering lag, no sideways slip. Its body collides with the garage, the remote, the arms and the table's props,
so a wall stops it the way a wall stops a real toy car (its wheels slip; it pushes with no more than its tyres' 2 N grip).
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
INSTRUCTION = "Drive the toy car into its garage with the remote control."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]

# -- the table (the station's): x 0.91 m is its far edge, y +-0.65 m its sides
EDGE_X, EDGE_Y = 0.91, 0.65

# -- the car (a 1:24 toy car: 18 cm long, 8.8 cm over the tyres, 190 g)
CAR0 = (0.58, 0.20)  # its centre at the start, heading +x (nose to the far edge)
CAR_L, CAR_W = 0.180, 0.088  # footprint over bumpers and tyres
WHEEL_X, WHEEL_Y, WHEEL_R, WHEEL_T = 0.055, 0.036, 0.022, 0.016  # wheel centres (+-x, +-y from the car's centre), radius, tread
WHEELBASE = 2 * WHEEL_X
V_MAX = 0.60  # m/s with the throttle on
TAU_UP = 0.30  # s: the motor's spin-up (and reversal) time constant
TAU_DOWN = 0.12  # s: coast-down with the throttle off (the motor's drag brakes it)
STEER_MAX = 18.0  # degrees of front-wheel lock: a 34 cm turning radius on the 11 cm wheelbase
TAU_STEER = 0.08  # s: the steering solenoid's travel
C_LIN, F_MAX = 8.0, 2.0  # the joint drives' damping (N s/m) and force cap (N): the tyres' grip on the table
C_YAW, T_MAX = 0.03, 0.15  # and for turning (N m s/rad, N m)
CAR_MASS = 0.19

# -- the remote (20 x 9.5 x 3.2 cm, 180 g with its two AA cells), lying flat between the arms in a cradle taped to the table
REMOTE0 = (0.27, 0.0)
RB = (0.095, 0.200, 0.032)  # body: along x, y, z
LEVER_Y = 0.075  # each lever 7.5 cm either side of the middle
PIVOT_Z = RB[2]
KNOB_R = 0.010  # a 20 mm ball on top of each lever's 5 mm stem
STEM_L = 0.028  # pivot to the ball's centre
KNOB_C = PIVOT_Z + STEM_L  # the ball's centre, above the remote's underside
LEVER_LIMIT = 22.0  # degrees each way
SWITCH_DEG = 10.0  # the switch closes past this
CENTRED_DEG = 5.0  # a released lever is back within this
LEVER_K, LEVER_C = 0.05, 0.003  # return spring (N m/rad: 0.5 N at the knob at full throw) and damping

# -- the garage: a children's shoebox base (28 x 17 x 10 cm outside), lid off, open side up, a 13 x 7.5 cm door cut in one end
GARAGE = (0.68, -0.495)  # the inside's centre
IN_HX, IN_HY, IN_H = 0.080, 0.135, 0.095  # inside half-width (x), half-length (y), height
WALL = 0.005
DOOR_HW, DOOR_H = 0.065, 0.075
DOOR_Y = GARAGE[1] + IN_HY  # -0.36: the door's inner face; the inside runs from here to the back wall at -0.63
WALL_TOL = 0.005  # a car pressed against a side or back wall is inside: its footprint may read up to this past the wall's inner face
LINE_TOL, HEAD_TOL, NEAR = 0.03, 15.0, 0.25  # the lined-up band: off the centre line, off the axis, centre out from the door

LOOKS = {
    "red": m.Look(color=m.rgb(0.80, 0.07, 0.06), roughness=0.35),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.055), roughness=0.5),
    "tyre": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.9),
    "hub": m.Look(color=m.rgb(0.75, 0.75, 0.78), metallic=0.6, roughness=0.3),
    "glass": m.Look(color=m.rgb(0.10, 0.13, 0.18), roughness=0.1),
    "light": m.Look(color=m.rgb(0.95, 0.92, 0.70), roughness=0.3),
    "remote": m.Look(color=m.rgb(0.16, 0.17, 0.19), roughness=0.6),
    "panel": m.Look(color=m.rgb(0.85, 0.65, 0.10), roughness=0.5),
    "knob_l": m.Look(color=m.rgb(0.10, 0.35, 0.80), roughness=0.5),
    "knob_r": m.Look(color=m.rgb(0.85, 0.12, 0.10), roughness=0.5),
    "stem": m.Look(color=m.rgb(0.6, 0.6, 0.62), metallic=0.5, roughness=0.4),
    "card": m.Look(color=m.rgb(0.62, 0.47, 0.30), roughness=0.85),
    "card_dark": m.Look(color=m.rgb(0.45, 0.33, 0.20), roughness=0.85),
}


# -- the car: a frame fixed to the table, two carriages and the body (the third link) ------------------------------------------------


def _car_parts():
    parts = [
        ("chassis", m.box((CAR_L, 0.080, 0.022), (0.0, 0.0, 0.020)), "red"),
        ("bonnet", m.box((0.060, 0.074, 0.010), (0.055, 0.0, 0.035)), "red"),
        ("boot", m.box((0.040, 0.074, 0.012), (-0.068, 0.0, 0.036)), "red"),
        ("cabin", m.box((0.075, 0.068, 0.024), (-0.012, 0.0, 0.043)), "glass"),
        ("roof", m.box((0.055, 0.064, 0.004), (-0.015, 0.0, 0.057)), "red"),
        ("bumper_f", m.box((0.008, 0.078, 0.010), (CAR_L / 2 - 0.004, 0.0, 0.014)), "black"),
        ("bumper_r", m.box((0.008, 0.078, 0.010), (-CAR_L / 2 + 0.004, 0.0, 0.014)), "black"),
        ("lamp_l", m.box((0.004, 0.014, 0.006), (CAR_L / 2 + 0.001, 0.026, 0.026)), "light"),
        ("lamp_r", m.box((0.004, 0.014, 0.006), (CAR_L / 2 + 0.001, -0.026, 0.026)), "light"),
        ("spoiler", m.box((0.014, 0.076, 0.004), (-0.082, 0.0, 0.050)), "black"),
    ]
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        c = (sx * WHEEL_X, sy * WHEEL_Y, WHEEL_R)
        parts.append((f"tyre{i}", m.cylinder(WHEEL_R, WHEEL_T, c, axis="y"), "tyre"))
        parts.append((f"hub{i}", m.cylinder(0.011, WHEEL_T + 0.002, c, axis="y"), "hub"))
    return parts


def _car():
    body = m.hinge("yaw", "Z", (0.0, 0.0, 0.0), -math.inf, math.inf, _car_parts(),
                   (m.col_box("car_c", (CAR_L, CAR_W, 0.058), (0.0, 0.0, 0.006 + 0.029)),),
                   mass=CAR_MASS, spring=0.0, damping=C_YAW, effort=T_MAX, friction=0.0, parent="cy",
                   inertia=(1.8e-4, 5.7e-4, 6.3e-4))
    cx = m.slider("cx", "X", -0.40, 0.45, (), (), mass=0.02, spring=0.0, damping=C_LIN, effort=F_MAX, friction=0.0, inertia=(1e-5, 1e-5, 1e-5))
    cy = m.slider("cy", "Y", -1.0, 0.55, (), (), mass=0.02, spring=0.0, damping=C_LIN, effort=F_MAX, friction=0.0, parent="cx", inertia=(1e-5, 1e-5, 1e-5))
    return m.mechanism("car", (CAR0[0], CAR0[1], TOP), [cx, cy, body], LOOKS, friction=(0.9, 0.8), solver_iters=32)


CAR = _car()


# -- the remote: a free body with two sprung lever switches ----------------------------------------------------------------------------


def _lever(name, y, axis, knob_look):
    """A ball-topped stick (mechkit's lever: a bar and a ball), sprung to centre, with a rubber boot round its foot."""
    lk = m.lever(name, (0.0, y, PIVOT_Z), (0.0, 0.0, 1.0), axis=axis, look="stem", length=STEM_L, radius=0.0025, ball_r=KNOB_R,
                 lower=-LEVER_LIMIT, upper=LEVER_LIMIT, spring=LEVER_K, damping=LEVER_C, friction=0.001, mass=0.012)
    boot = m.cylinder(0.008, 0.006, (0.0, y, PIVOT_Z + 0.003))
    parts = (lk.parts[0], ("ball", lk.parts[1][1], knob_look), ("boot", boot, "black"))
    import dataclasses
    return dataclasses.replace(lk, parts=parts, effort=5.0, inertia=(5e-6, 5e-6, 1e-6))


def _remote():
    hx, hy, hz = RB[0] / 2, RB[1] / 2, RB[2]
    fp = [
        ("body", m.box((RB[0], RB[1] - 0.03, hz), (0.0, 0.0, hz / 2)), "remote"),
        ("grip_l", m.cylinder(hx, hz, (0.0, hy - 0.015, hz / 2), segments=24), "remote"),
        ("grip_r", m.cylinder(hx, hz, (0.0, -hy + 0.015, hz / 2), segments=24), "remote"),
        ("plate_l", m.cylinder(0.022, 0.001, (0.0, LEVER_Y, hz + 0.0005)), "black"),
        ("plate_r", m.cylinder(0.022, 0.001, (0.0, -LEVER_Y, hz + 0.0005)), "black"),
        ("panel", m.box((0.030, 0.050, 0.001), (0.010, 0.0, hz + 0.0005)), "panel"),
        ("switch", m.box((0.010, 0.016, 0.004), (-0.020, 0.0, hz + 0.002)), "black"),
        ("lamp", m.cylinder(0.003, 0.002, (0.025, 0.0, hz + 0.001)), "knob_r"),
        ("antenna", m.cylinder(0.003, 0.11, (0.035, -0.040, hz + 0.055)), "black"),
        ("cradle", m.box((RB[0] + 0.016, RB[1] + 0.016, 0.003), (0.0, 0.0, 0.001)), "card"),
        ("rim_f", m.box((0.004, RB[1] + 0.016, 0.012), (hx + 0.006, 0.0, 0.006)), "card"),
        ("rim_b", m.box((0.004, RB[1] + 0.016, 0.012), (-hx - 0.006, 0.0, 0.006)), "card"),
    ]
    fc = [m.col_box("remote_c", (RB[0], RB[1], hz), (0.0, 0.0, hz / 2))]
    links = [_lever("thr", LEVER_Y, "Y", "knob_l"), _lever("dir", -LEVER_Y, "X", "knob_r")]
    return m.mechanism("remote", (REMOTE0[0], REMOTE0[1], TOP + 0.0005), links, LOOKS, frame_parts=fp, frame_colliders=fc, friction=(1.0, 0.9), solver_iters=32)


REMOTE = _remote()


# -- the garage: a shoebox, fixed (taped down at its back corners) -----------------------------------------------------------------------


def _garage():
    sx, sy = IN_HX + WALL / 2, IN_HY + WALL / 2
    jamb_w = IN_HX + WALL - DOOR_HW
    hw = IN_H + WALL  # the walls' height: the box stands open side up, lid off, so the car can be seen inside from above
    pieces = {
        "side_l": ((WALL, 2 * IN_HY + 2 * WALL, hw), (sx, 0.0, hw / 2)),
        "side_r": ((WALL, 2 * IN_HY + 2 * WALL, hw), (-sx, 0.0, hw / 2)),
        "back": ((2 * IN_HX, WALL, hw), (0.0, -sy, hw / 2)),
        "jamb_l": ((jamb_w - WALL, WALL, hw), (DOOR_HW + (jamb_w - WALL) / 2, sy, hw / 2)),
        "jamb_r": ((jamb_w - WALL, WALL, hw), (-DOOR_HW - (jamb_w - WALL) / 2, sy, hw / 2)),
        "lintel": ((2 * DOOR_HW, WALL, hw - DOOR_H), (0.0, sy, DOOR_H + (hw - DOOR_H) / 2)),
    }
    parts, cols = [], []
    for n, (size, c) in pieces.items():
        parts.append((n, m.box(size, c), "card"))
        cols.append(m.col_box(f"{n}_c", size, c))
    parts.append(("floor", m.box((2 * IN_HX, 2 * IN_HY, 0.001), (0.0, 0.0, 0.0005)), "card_dark"))  # its thin bottom, flush with the table
    return m.fixed("garage", (GARAGE[0], GARAGE[1], TOP), parts, cols, LOOKS, friction=(0.9, 0.8))


GARAGE_PART = _garage()

# -- distractors (SimReady): a mug and a puzzle cube on the far side of the table, clear of the car's way
MUG = SimReady("mug", pos=(0.84, 0.36, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.4)
CUBE = SimReady("puzzle_cube", pos=(0.85, -0.24, TOP), size=(0.072, 0.072, 0.072), mass=0.10, package="toys_rubiksCube_a01", usd="sm_toys_rubiksCube_a01_01.usda", yaw=0.5)

OBJECTS = [CAR, REMOTE, GARAGE_PART, MUG, CUBE]


# -- the toy car's behaviour ----------------------------------------------------------------------------------------------------------


def switch(deg: float) -> int:
    """A lever's switch: +1 past +SWITCH_DEG, -1 past -SWITCH_DEG, else 0."""
    return 1 if deg > SWITCH_DEG else (-1 if deg < -SWITCH_DEG else 0)


def levers(env) -> tuple[float, float]:
    """(throttle lever, steering lever) in degrees. Throttle: + pushed forward (away from the operator). Steering: + pushed left."""
    s = m.state(env, "remote")
    return s.deg("thr"), -s.deg("dir")  # the steering lever turns about +x: its top goes left (+y) for a negative angle


def car_pose(env) -> tuple[float, float, float]:
    """The car's centre (x, y) on the table and its heading (rad from +x, anticlockwise)."""
    s = m.state(env, "car")
    return CAR0[0] + s.joint("cx"), CAR0[1] + s.joint("cy"), s.joint("yaw")


def car_vel(env) -> np.ndarray:
    asset = env.unwrapped.scene["car"]
    q = m._np(asset.data.joint_vel)[0]
    names = asset.joint_names
    return np.array([q[names.index("cx_joint")], q[names.index("cy_joint")], q[names.index("yaw_joint")]])


def corners(x, y, psi, half_l=CAR_L / 2, half_w=CAR_W / 2) -> np.ndarray:
    c, s = math.cos(psi), math.sin(psi)
    return np.array([(x + c * a - s * b, y + s * a + c * b) for a in (half_l, -half_l) for b in (half_w, -half_w)])


def _set_vel(env, link: str, v: float) -> None:
    asset, j = m._joint(env, "car", link)
    asset.set_joint_velocity_target(m._tensor(asset, v), joint_ids=[j])


class Car:
    """The car's motor and steering, driven by the remote's switches: v (m/s along the heading) and the front wheels' angle (deg)."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.v, self.steer, self.frozen = 0.0, 0.0, False
        if env is not None:
            self.stop(env)

    def stop(self, env):
        for link in ("cx", "cy", "yaw"):
            _set_vel(env, link, 0.0)

    def freeze(self, env):
        """Off the table's edge: the run is over for the car, which stays where it went over."""
        self.stop(env)
        for link in ("cx", "cy", "yaw"):
            m.lock(env, "car", link)
        self.frozen = True

    def step(self, env):
        if self.frozen:
            return
        thr_deg, dir_deg = levers(env)
        thr, st = switch(thr_deg), switch(dir_deg)
        target = thr * V_MAX
        tau = TAU_UP if thr != 0 else TAU_DOWN
        self.v += (target - self.v) * (1.0 - math.exp(-DT / tau))
        self.steer += (st * STEER_MAX - self.steer) * (1.0 - math.exp(-DT / TAU_STEER))
        _, _, psi = car_pose(env)
        w = self.v * math.tan(math.radians(self.steer)) / WHEELBASE
        _set_vel(env, "cx", self.v * math.cos(psi))
        _set_vel(env, "cy", self.v * math.sin(psi))
        _set_vel(env, "yaw", w)


CARWORK = Car()
m.on_step(CARWORK.step)
m.on_reset(CARWORK.reset)

_plain_restore = m._restore_touched


def _restore_and_stop(env):
    """Before a reset's settling steps: stop the car's drives as well as the kit's own restore (it does not know velocity targets)."""
    try:
        CARWORK.stop(env)
    except Exception:
        pass
    _plain_restore(env)


m._restore_touched = _restore_and_stop


# -- the scorer -------------------------------------------------------------------------------------------------------------------------


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def tip_gap(env, x, y, psi) -> float:
    """The nearest fingertip point's distance (m) from the car's body (its 18 x 8.8 x 6.4 cm box)."""
    tips = m.fingertips(env)
    c, s = math.cos(psi), math.sin(psi)
    dx, dy = tips[:, 0] - x, tips[:, 1] - y
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    lz = tips[:, 2] - (TOP + 0.032)
    d = np.maximum(np.abs(np.c_[lx, ly, lz]) - np.array([CAR_L / 2, CAR_W / 2, 0.032]), 0.0)
    return float(np.min(np.linalg.norm(d, axis=1)))


def now(env) -> dict:
    x, y, psi = car_pose(env)
    pts = corners(x, y, psi)
    wheels = corners(x, y, psi, WHEEL_X, WHEEL_Y)
    inside = bool(np.all(pts[:, 1] <= DOOR_Y) and np.all(np.abs(pts[:, 0] - GARAGE[0]) <= IN_HX + WALL_TOL) and np.all(pts[:, 1] >= GARAGE[1] - IN_HY - WALL_TOL))
    half = bool(abs(x - GARAGE[0]) <= IN_HX and GARAGE[1] - IN_HY <= y <= DOOR_Y)
    axis_err = math.degrees(min(abs(_wrap(psi + math.pi / 2)), abs(_wrap(psi - math.pi / 2))))  # nose-in or tail-in
    lined = bool(abs(x - GARAGE[0]) <= LINE_TOL and axis_err <= HEAD_TOL and DOOR_Y < y <= DOOR_Y + NEAR)
    off = bool(np.any(wheels[:, 0] > EDGE_X) or np.any(np.abs(wheels[:, 1]) > EDGE_Y) or np.any(wheels[:, 0] < -0.19))
    thr, dr = levers(env)
    vel = car_vel(env)
    return {"x": x, "y": y, "psi": math.degrees(psi), "inside": inside, "half": half, "lined": lined, "axis_err": axis_err, "off": off,
            "thr": thr, "dir": dr, "released": abs(thr) <= CENTRED_DEG and abs(dr) <= CENTRED_DEG, "speed": float(np.hypot(vel[0], vel[1])),
            "spin": abs(float(vel[2])), "gap": tip_gap(env, x, y, psi), "model_v": CARWORK.v}


class Trial:
    """The score, and the record of harm: a hand on the car, the car moved with its motor off, or a wheel over the table's edge."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.touched = False
        self.pushed = False
        self.fell = False
        self.push_run = 0
        self.last = None

    def step(self, env):
        c = now(env)
        self.last = c
        if c["gap"] < 0.010:
            self.touched = True
        # moved with no drive from its own motor: pushed or dragged by something else
        self.push_run = self.push_run + 1 if (abs(CARWORK.v) < 0.02 and (c["speed"] > 0.05 or c["spin"] > 0.5)) else 0
        if self.push_run >= 3:
            self.pushed = True
        if c["off"] and not self.fell:
            self.fell = True
            CARWORK.freeze(env)
        harm = self.touched or self.pushed or self.fell
        ok = c["inside"] and c["released"] and c["speed"] < 0.01 and c["spin"] < 0.05 and abs(CARWORK.v) < 0.01 and c["gap"] >= 0.03
        self.hold = self.hold + 1 if (ok and not harm) else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if harm:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["half"]:
            self.score = 0.5
        elif c["lined"]:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration ------------------------------------------------------------------------------------------------------------------


def _report(ctx, env, label=""):
    c = now(env)
    print(f"[car] t {ctx.t:5.1f} {label}x {c['x']:.3f} y {c['y']:+.3f} psi {c['psi']:+7.1f} v {c['speed']:.2f} model {c['model_v']:+.2f} levers {c['thr']:+5.1f} {c['dir']:+5.1f} "
          f"inside {c['inside']} half {c['half']} lined {c['lined']} gap {c['gap'] * 1000:.0f} mm touched {TRIAL.touched} pushed {TRIAL.pushed} fell {TRIAL.fell} "
          f"score {TRIAL.score:.2f} full {TRIAL.full}", flush=True)


def demo_script(ctx):
    """Left hand: from behind, pinches the throttle's ball across its pivot axis. Right hand: from its own side, tilted outward, pinches
    the steering's ball across its pivot axis (each ball turns freely between the pads as its lever tilts). Each step, both hands put
    their ball where the drive needs it (19 degrees over, past the switch, or centred), corrected by how far the lever is seen to lag,
    from the car's seen pose and speed:
      1. forward on full right lock (throttle only once the steering is seen locked) until the car has turned 30 degrees, or its front
         wheels are 10 cm from the far edge;
      2. reverse on full left lock, which keeps it turning the same way, until it nearly faces the door (heading -80 degrees; it coasts
         a little further); shunt again if it isn't round yet;
      3. forward, steering by taps onto the garage's centre line, through the door, until its tail is just inside, then let it coast;
      4. both levers released, the car left to stop, both hands off the remote and back.
    The throttle is tapped to hold a set speed (0.20 m/s in the shunt, 0.22 then 0.15 m/s towards the door), as a person taps a toy
    car's throttle.
    Failure checks (--mode): none does nothing; onearm works the throttle only (forward, steering never touched); grab reaches for the car
    to carry it into the garage by hand; edge drives straight on at full throttle; half stops with the car half through the door; lined
    stops it lined up about 15 cm short of the door; careless takes the turn with no shunt (full right lock, forward)."""
    import demokit as d

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "grab", "edge", "half", "lined", "careless"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(80):
            yield
        _report(ctx, env, "end: ")
        return

    if mode == "grab":  # the right hand reaches for the car and carries it into the garage (if it can reach it)
        x, y, _ = car_pose(env)  # the left hand, nearer the car: as far towards it as it reaches, down on it, and towards the garage
        yield from l.move((x, y, TOP + 0.15), grip=d.down(0), finger=d.FINGER_OPEN, t=2.5, label="over the car")
        yield from l.move((x, y, TOP + 0.045), t=1.5, label="down on the car")
        yield from l.close(0.07, 20.0, label="grab the car")
        yield from l.move((GARAGE[0], DOOR_Y + 0.05, TOP + 0.12), t=2.5, label="carry it to the door")
        yield from l.move((GARAGE[0], GARAGE[1], TOP + 0.05), t=2.0, label="push it in")
        yield from l.open()
        yield from l.move((0.25, 0.25, TOP + 0.20), t=2.0, label="back")
        for _ in range(40):
            yield
        _report(ctx, env, "end: ")
        return

    remote = lambda: ctx.state("remote")  # noqa: E731
    R_KNOB = KNOB_C - PIVOT_Z
    AMP = math.radians(19.0)  # each knob is put 19 degrees over: past the 10-degree switch, short of the 22-degree stop
    TILT = 35.0

    def knob_target(lever: str, a: float) -> np.ndarray:
        """The knob's middle (world), with its lever tilted `a` (rad): + forward for the throttle, + left for the steering."""
        s = remote()
        if lever == "thr":
            p = (math.sin(a) * R_KNOB, LEVER_Y, PIVOT_Z + math.cos(a) * R_KNOB)
        else:
            p = (0.0, -LEVER_Y + math.sin(a) * R_KNOB, PIVOT_Z + math.cos(a) * R_KNOB)
        return s.to_world(np.array([p]))[0]

    use_steer = mode != "onearm"
    # -- each hand pinches its ball across the lever's pivot axis, so the ball turns freely between the pads as the lever tilts: the
    #    left from behind (fingers along y), the right from its own side, tilted outward (fingers along x), so the wrists stay clear
    #    of each other. Each stops 6 cm back along its fingers from the ball, then slides in round it and pinches
    gl, gr = d.tilted(TILT, 0.0), d.tilted(TILT, 90.0)
    al, ar = l.approach_axis(gl), r.approach_axis(gr)
    tl, tr = knob_target("thr", 0.0), knob_target("dir", 0.0)
    high = np.array([0.0, 0.0, 0.10])
    moves = [l.move(tl - 0.06 * al + high, grip=gl, finger=d.FINGER_OPEN * 0.7, t=2.5, label="high outside the throttle")]
    if use_steer:
        moves.append(r.move(tr - 0.06 * ar + high, grip=gr, finger=d.FINGER_OPEN * 0.7, t=2.5, label="high outside the steering"))
    yield from ctx.together(*moves)
    moves = [l.move(tl - 0.06 * al, t=1.2, label="outside the throttle")] + ([r.move(tr - 0.06 * ar, t=1.2, label="outside the steering")] if use_steer else [])
    yield from ctx.together(*moves)
    moves = [l.servo(tl - 0.06 * al, timeout=1.0, label="outside the throttle")] + ([r.servo(tr - 0.06 * ar, timeout=1.0, label="outside the steering")] if use_steer else [])
    yield from ctx.together(*moves)
    moves = [l.move(tl, t=1.2, label="round the throttle ball")] + ([r.move(tr, t=1.2, label="round the steering ball")] if use_steer else [])
    yield from ctx.together(*moves)
    moves = [l.servo(knob_target("thr", 0.0), timeout=1.0, label="at the throttle")] + ([r.servo(knob_target("dir", 0.0), timeout=1.0, label="at the steering")] if use_steer else [])
    yield from ctx.together(*moves)
    moves = [l.close(2 * KNOB_R, 20.0, t=0.5, label="pinch the throttle")] + ([r.close(2 * KNOB_R, 20.0, t=0.5, label="pinch the steering")] if use_steer else [])
    yield from ctx.together(*moves)
    _report(ctx, env, "knobs pinched: ")

    from yam_arena import ik

    # trims learnt while driving: what each hand's command needs on top of the knob's position to put it where it is sent
    base_l = ik.tcp_pose(l.q, l.base)[0] - knob_target("thr", 0.0)
    base_r = ik.tcp_pose(r.q, r.base)[0] - knob_target("dir", 0.0)
    state = {"thr": 0, "st": 0, "prev": car_pose(env), "speed": 0.0}

    state["I"] = [0.0, 0.0]  # degrees added to each lever's command, learnt from its seen angle (the arms are soft)

    def drive_step(thr: int, st: int):
        """Send both knobs for one step: thr, st in {-1, 0, +1} (forward/left positive). Each knob is sent to its angle plus a correction
        that grows while the lever, as seen, stays short of it."""
        seen = levers(env)
        cmd = []
        for i, (want, now_deg) in enumerate(zip((thr * math.degrees(AMP), st * math.degrees(AMP)), seen)):
            e = want - now_deg
            state["I"][i] = float(np.clip(state["I"][i] + 0.2 * e, -6.0, 6.0))
            cmd.append(math.radians(want + state["I"][i]))
        a_t, a_s = cmd
        cl = knob_target("thr", a_t) + base_l
        mm, dg = l._solve(cl, l.grip.rot)
        l._note(mm, dg, "throttle")
        if use_steer:
            cr = knob_target("dir", a_s) + base_r
            mm, dg = r._solve(cr, r.grip.rot)
            r._note(mm, dg, "steering")
        yield
        x, y, psi = car_pose(env)
        px, py, _ = state["prev"]
        state["speed"] = math.hypot(x - px, y - py) / DT
        state["prev"] = (x, y, psi)

    def paced(direction: int, v_set: float) -> int:
        """The throttle for this step: on while the car is slower than `v_set`, off once it is faster (a tap-tap pace)."""
        return direction if state["speed"] < v_set else 0

    def locked(direction: int, v_set: float, st: int) -> int:
        """`paced`, but only once the steering lever is seen past its switch on the side wanted: lock first, then throttle."""
        return paced(direction, v_set) if levers(env)[1] * st > SWITCH_DEG + 1.0 else 0

    def run(cond, thr_fn, st_fn, timeout_s, label):
        for k in range(round(timeout_s / DT)):
            if cond():
                return True
            yield from drive_step(thr_fn(), st_fn())
        print(f"[demo] '{label}' timed out", flush=True)
        return False

    def settle(seconds=0.8):
        for _ in range(round(seconds / DT)):
            yield from drive_step(0, 0)

    def heading():
        return math.degrees(car_pose(env)[2])

    def front_wheel_x():
        x, y, psi = car_pose(env)
        return x + WHEEL_X * math.cos(psi) + WHEEL_Y * abs(math.sin(psi))

    if mode == "edge":
        yield from run(lambda: TRIAL.fell, lambda: 1, lambda: 0, 6.0, "straight on")
        yield from settle(1.0)
        _report(ctx, env, "end: ")
        return
    if mode == "onearm":
        yield from run(lambda: TRIAL.fell, lambda: paced(1, 0.25), lambda: 0, 8.0, "throttle only")
        yield from settle(1.0)
        _report(ctx, env, "end: ")
        return
    if mode == "careless":
        yield from run(lambda: car_pose(env)[1] < DOOR_Y + 0.05 or TRIAL.fell, lambda: paced(1, 0.25), lambda: -1, 10.0, "turn with no shunt")
        yield from settle(1.0)
        _report(ctx, env, "end: ")
        return

    # 1. forward on right lock until turned 30 degrees, or the front wheels 10 cm from the far edge (it coasts a few cm more)
    yield from run(lambda: heading() <= -30.0 or front_wheel_x() > EDGE_X - 0.10, lambda: locked(1, 0.20, -1), lambda: -1, 6.0, "forward right")
    yield from settle(0.6)
    _report(ctx, env, "turned forward: ")
    # 2. reverse on left lock until facing the door (shunt again if it isn't round yet)
    for shunt in range(3):
        yield from run(lambda: heading() <= -80.0 or car_pose(env)[1] + WHEEL_X > EDGE_Y - 0.10, lambda: locked(-1, 0.20, 1), lambda: 1, 6.0, "reverse left")
        yield from settle(0.6)
        _report(ctx, env, f"reversed ({shunt}): ")
        if heading() <= -75.0:
            break
        yield from run(lambda: heading() <= -80.0 or front_wheel_x() > EDGE_X - 0.10, lambda: locked(1, 0.20, -1), lambda: -1, 6.0, "forward right again")
        yield from settle(0.6)

    # 3. onto the garage's centre line and through the door
    stop_y = {"half": DOOR_Y - 0.02, "lined": DOOR_Y + 0.17}.get(mode, DOOR_Y - CAR_L / 2 - 0.005)

    def steer_to_line():
        x, y, psi = car_pose(env)
        want = -math.pi / 2 - float(np.clip(6.0 * (x - GARAGE[0]), -0.35, 0.35))
        err = math.degrees(_wrap(want - psi))
        return 1 if err > 2.0 else (-1 if err < -2.0 else 0)

    def pace_in():
        y = car_pose(env)[1]
        return paced(1, 0.15 if y < DOOR_Y + 0.25 else 0.22)

    yield from run(lambda: car_pose(env)[1] <= stop_y, pace_in, steer_to_line, 15.0, "to the garage")
    yield from settle(1.2)
    _report(ctx, env, "stopped: ")

    # 4. hands off the remote and back
    back = [l.open(t=0.4, label="let go of the throttle")] + ([r.open(t=0.4, label="let go of the steering")] if use_steer else [])
    yield from ctx.together(*back)
    lift = np.array([0.0, 0.0, 0.03])
    back = [l.move(l.at() - 0.06 * al + lift, t=0.8, label="back off the throttle")] + ([r.move(r.at() - 0.06 * ar + lift, t=0.8, label="back off the steering")] if use_steer else [])
    yield from ctx.together(*back)
    back = [l.move((0.20, 0.25, TOP + 0.22), t=1.5, label="left hand back")] + ([r.move((0.20, -0.25, TOP + 0.22), t=1.5, label="right hand back")] if use_steer else [])
    yield from ctx.together(*back)
    for k in range(60):
        yield
        if k % 30 == 29:
            _report(ctx, env, "end: ")
