"""Focus an open-frame diode laser engraver's module on a plywood sheet with its built-in focus stick, then leave it ready to cut.

The machine is an AlgoLaser DIY KIT MK3 type open-frame diode laser engraver (unplugged; the laser never fires) over an AlgoLaser
320 x 320 mm honeycomb platform, with a 3 mm basswood plywood sheet on the honeycomb. Its laser module hangs on the gantry's carriage
on a vertical slide and is clamped there by a knurled thumbscrew on the carriage, beside the module. A focus stick pivots on the
module's front face: stowed it points up along the face; flipped down it hangs past the light shield, and the module is at focus when
the hanging stick's tip just touches the work. At the start the module is clamped 12 mm too high and the stick is stowed.

Model (declared in code):
- "laser" (one articulation, frame fixed): the carriage plate and the thumbscrew's boss are the frame. Links:
  - "slide": the module (body and light shield), a prismatic joint along z with gravity, 0.6 kg (estimate), 8 N s/m damping (estimate);
  - "stick": the focus stick and its pivot knob, a hinge about x on the module (parent "slide"): 0 deg stowed (up), -180 deg hanging
    (both are stops); 0.03 N m pivot friction (estimate);
  - "screw": the thumbscrew, a hinge about x on the frame: 0 deg seated (a stop), negative is loosened (counter-clockwise as seen from
    the front); 0.05 N m friction (estimate).
- The clamp (code, every control step): while the thumbscrew is loosened less than CLAMP_DEG (45 deg) from seated, the slide is held where
  it is (mechkit lock, 200 kN/m), as the clamp's friction holds the real module; loosened further, the slide is free, held only by its
  damping and whatever holds the module.
- The scorer (code, every control step) keeps the violation record, cleared only in the reset hook.
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
from yam_arena import demo as _ydemo  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Focus the laser module on the plywood sheet with its focus stick, and leave the engraver ready to cut."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- layout (x forward, y left, z up; heights above the table top) -------------------------------------------------------------------
HONEY = (0.26, 0.58, -0.110, 0.210, 0.0225)  # provisional platform footprint and height in metres
SHEET_SIZE = (0.200, 0.300, 0.003)  # basswood plywood: x, y, z
SHEET_C = (0.37, 0.05, HONEY[4] + SHEET_SIZE[2] / 2)
WORK_TOP = HONEY[4] + SHEET_SIZE[2]

MOD_X, MOD_Y = 0.320, 0.050  # the module's axis
MOD_W = 0.050  # body: 50 x 50 mm
X_FACE = MOD_X - MOD_W / 2  # front face (towards the robot)
X_BACK = MOD_X + MOD_W / 2
SHIELD_H, BODY_H = 0.025, 0.090
REACH = 0.010  # the hanging stick's tip below the shield's lower edge
STICK_L = 0.040  # pivot to tip
PIVOT_UP = 0.030  # the pivot above the shield's lower edge
HIGH = 0.012  # the module starts this much above focus
SB0 = WORK_TOP + REACH + HIGH  # the shield's lower edge at the start (0.048)
PIVOT_Y = MOD_Y - 0.015
PIVOT_Z0 = SB0 + PIVOT_UP
STICK_T, STICK_W = 0.003, 0.006  # thickness (x) and width
STICK_X = X_FACE - 0.001 - STICK_T / 2  # the stick lies 1 mm off the face
KNOB_R, KNOB_L = 0.006, 0.008  # the stick's pivot knob, in front of the stick
KNOB_X0 = X_FACE - 0.001 - STICK_T - KNOB_L  # its front face

SCREW_Y, SCREW_Z = MOD_Y - 0.055, 0.085  # the thumbscrew's axis
SCREW_R, SCREW_L = 0.006, 0.020  # knurled head
SCREW_X0 = X_FACE - 0.012  # its front face
CLAMP_DEG = 45.0
LOCK_K, LOCK_F = 2.0e5, 2000.0  # the clamp: N/m and N (the slide's drive while clamped)

GANTRY = (0.36, 0.40, 0.10, 0.14)  # x0, x1, z0, z1 of the gantry beam (along y)

LOOKS = {
    "alu": m.Look(color=m.rgb(0.55, 0.57, 0.60), metallic=0.8, roughness=0.35),
    "alu_dark": m.Look(color=m.rgb(0.30, 0.31, 0.33), metallic=0.7, roughness=0.4),
    "shield": m.Look(color=m.rgb(0.95, 0.45, 0.05), roughness=0.2, opacity=0.75),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.06), roughness=0.5),
    "steel": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=0.95, roughness=0.3),
    "plastic": m.Look(color=m.rgb(0.80, 0.81, 0.82), roughness=0.5),
    "honey": m.Look(color=m.rgb(0.62, 0.63, 0.64), metallic=0.6, roughness=0.6),
    "honey_frame": m.Look(color=m.rgb(0.20, 0.20, 0.22), metallic=0.5, roughness=0.5),
}


# -- the laser: carriage (frame), module on its slide, focus stick, thumbscrew ------------------------------------------------------
def _laser():
    fp, fc = [], []
    plate = ((0.010, 0.120, 0.120), (X_BACK + 0.0065, MOD_Y - 0.010, 0.11))  # carriage plate behind the module (1.5 mm clear)
    fp.append(("carriage", m.box(*plate), "plastic"))
    fc.append(m.col_box("carriage_c", *plate))
    boss = ((X_BACK + 0.001 - (SCREW_X0 + SCREW_L), 0.020, 0.030), ((SCREW_X0 + SCREW_L + X_BACK + 0.001) / 2, SCREW_Y, SCREW_Z))
    fp.append(("screw_boss", m.box(*boss), "plastic"))
    fc.append(m.col_box("screw_boss_c", *boss))

    sb = SB0
    body = ((MOD_W, MOD_W, BODY_H), (MOD_X, MOD_Y, sb + SHIELD_H + BODY_H / 2))
    shield = ((MOD_W - 0.002, MOD_W - 0.002, SHIELD_H), (MOD_X, MOD_Y, sb + SHIELD_H / 2))
    mp = [("body", m.box(*body), "alu"), ("shield", m.box(*shield), "shield"),
          ("fan", m.cylinder(0.020, 0.002, (MOD_X, MOD_Y, sb + SHIELD_H + BODY_H + 0.001)), "black"),
          ("vent_l", m.box((0.025, 0.002, 0.006), (MOD_X, MOD_Y + MOD_W / 2 + 0.0005, sb + SHIELD_H + BODY_H - 0.02)), "black"),
          ("vent_r", m.box((0.025, 0.002, 0.006), (MOD_X, MOD_Y - MOD_W / 2 - 0.0005, sb + SHIELD_H + BODY_H - 0.02)), "black"),
          ("lens", m.cylinder(0.006, 0.010, (MOD_X, MOD_Y, sb + 0.012)), "black")]
    mc = [m.col_box("module_c", (MOD_W, MOD_W, SHIELD_H + BODY_H), (MOD_X, MOD_Y, sb + (SHIELD_H + BODY_H) / 2))]
    slide = m.slider("slide", "Z", -0.035, 0.030, mp, mc, at=(MOD_X, MOD_Y, sb), mass=0.6, damping=8.0, friction=0.0, spring=LOCK_K, effort=LOCK_F,
                     gravity=True, com=(MOD_X, MOD_Y, sb + 0.06), inertia=(8e-4, 8e-4, 3e-4))

    # a flat steel strip with a rounded end (radius STICK_W / 2), 6 mm past the pivot on the other side
    r_end = STICK_W / 2
    strip = m.join(m.box((STICK_T, STICK_W, STICK_L + 0.006 - r_end), (STICK_X, PIVOT_Y, PIVOT_Z0 + (STICK_L - r_end - 0.006) / 2)),
                   m.cylinder(r_end, STICK_T, (STICK_X, PIVOT_Y, PIVOT_Z0 + STICK_L - r_end), axis="x", segments=16))
    sp = [("stick", strip, "steel"),
          ("pivot_knob", m.cylinder(KNOB_R, KNOB_L, (KNOB_X0 + KNOB_L / 2, PIVOT_Y, PIVOT_Z0), axis="x"), "steel")]
    sc = [m.col_hull("stick_c", strip),
          m.col_hull("pivot_knob_c", m.cylinder(KNOB_R, KNOB_L, (KNOB_X0 + KNOB_L / 2, PIVOT_Y, PIVOT_Z0), axis="x", segments=16))]
    stick = m.hinge("stick", "X", (STICK_X, PIVOT_Y, PIVOT_Z0), -180.0, 0.0, sp, sc, mass=0.02, damping=0.002, friction=0.03, effort=5.0,
                    parent="slide", inertia=(1.2e-5, 1.2e-5, 1e-6))

    screw_c = (SCREW_X0 + SCREW_L / 2, SCREW_Y, SCREW_Z)
    knurl = m.cylinder(SCREW_R, SCREW_L, screw_c, axis="x", segments=24)
    rp = [("screw_head", knurl, "steel"), ("screw_mark", m.box((0.001, 0.0015, 0.004), (SCREW_X0 - 0.0004, SCREW_Y, SCREW_Z + 0.003)), "black")]
    rc = [m.col_hull("screw_c", m.cylinder(SCREW_R, SCREW_L, screw_c, axis="x", segments=16))]
    screw = m.hinge("screw", "X", screw_c, -720.0, 0.0, rp, rc, mass=0.01, damping=0.002, friction=0.05, effort=5.0, inertia=(2e-7, 4e-7, 4e-7))
    return m.mechanism("laser", (0.0, 0.0, TOP), [slide, stick, screw], LOOKS, frame_parts=fp, frame_colliders=fc, solver_iters=48, friction=(0.6, 0.5))


LASER = _laser()


# -- the machine's frame (2020-type aluminium profiles) and gantry, and the honeycomb platform ---------------------------------------
def _frame():
    p, c = [], []
    rails = {
        "rail_front": ((0.020, 0.620, 0.020), (0.110, 0.0, 0.010)),
        "rail_back": ((0.020, 0.620, 0.020), (0.790, 0.0, 0.010)),
        "rail_left": ((0.700, 0.020, 0.040), (0.450, 0.300, 0.020)),
        "rail_right": ((0.700, 0.020, 0.040), (0.450, -0.300, 0.020)),
        "gantry": ((GANTRY[1] - GANTRY[0], 0.600, GANTRY[3] - GANTRY[2]), ((GANTRY[0] + GANTRY[1]) / 2, 0.0, (GANTRY[2] + GANTRY[3]) / 2)),
        "gantry_end_l": ((0.060, 0.020, GANTRY[2] - 0.040), ((GANTRY[0] + GANTRY[1]) / 2, 0.300, (GANTRY[2] + 0.040) / 2)),
        "gantry_end_r": ((0.060, 0.020, GANTRY[2] - 0.040), ((GANTRY[0] + GANTRY[1]) / 2, -0.300, (GANTRY[2] + 0.040) / 2)),
    }
    for name, (size, cen) in rails.items():
        p.append((name, m.box(size, cen), "alu_dark"))
        c.append(m.col_box(name + "_c", size, cen))
    x0, x1, y0, y1, t = HONEY
    hc = ((x1 - x0, y1 - y0, t), ((x0 + x1) / 2, (y0 + y1) / 2, t / 2))
    p.append(("honeycomb", m.box((x1 - x0 - 0.02, y1 - y0 - 0.02, t), (hc[1][0], hc[1][1], t / 2 + 0.0002)), "honey"))
    p.append(("honeycomb_frame", m.box(hc[0], (hc[1][0], hc[1][1], t / 2)), "honey_frame"))
    c.append(m.col_box("honeycomb_c", *hc))
    return m.fixed("machine", (0.0, 0.0, TOP), p, c, LOOKS, friction=(0.9, 0.8))


MACHINE = _frame()

LOOKS["ply"] = m.Look(color=m.rgb(0.86, 0.73, 0.53), roughness=0.8)
SHEET = m.free("sheet", (SHEET_C[0], SHEET_C[1], TOP + SHEET_C[2]), [("ply", m.box(SHEET_SIZE), "ply")], [m.col_box("ply_c", SHEET_SIZE)], LOOKS,
               mass=0.09, friction=(0.5, 0.4))

# -- distractors (SimReady): a caliper and mug on the bench behind the gantry
CALIPER = SimReady("caliper", pos=(0.66, 0.13, TOP), size=(0.238, 0.078, 0.008), mass=0.12, package="instrument_caliper_a01",
                   usd="sm_instrument_caliper_a01_01.usd", yaw=math.pi/2)
MUG = SimReady("mug", pos=(0.67, -0.15, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
               usd="sm_dishware_mug_a01_01.usda", yaw=0.6)

OBJECTS = [LASER, MACHINE, SHEET, CALIPER, MUG]


# -- the clamp ---------------------------------------------------------------------------------------------------------------------
def _slide_drive(env, stiffness: float, target: float) -> None:
    asset, j = m._joint(env, "laser", "slide")
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


class Clamp:
    """The slide is authored clamped (its drive holds joint zero, so the module stays put through the reset's settle). Clamping again
    holds it where it is now; freeing it drops the drive's stiffness to zero."""

    def __init__(self):
        self.locked = True

    def reset(self, env=None):
        self.locked = True
        if env is not None:
            _slide_drive(env, LOCK_K, 0.0)

    def step(self, env):
        s = m.state(env, "laser")
        want = -s.deg("screw") < CLAMP_DEG
        if want and not self.locked:
            _slide_drive(env, LOCK_K, s.joint("slide"))
        elif self.locked and not want:
            _slide_drive(env, 0.0, s.joint("slide"))
        self.locked = want


CLAMP = Clamp()
m.on_step(CLAMP.step)
m.on_reset(CLAMP.reset)


# -- the scorer --------------------------------------------------------------------------------------------------------------------
def _box_gap(pts, lo, hi) -> float:
    d = np.maximum(np.maximum(np.asarray(lo) - pts, pts - np.asarray(hi)), 0.0)
    return float(np.min(np.linalg.norm(d, axis=1)))


def _sheet_corners(st) -> np.ndarray:
    q = st.quat_xyzw
    R = m.quat_matrix(q)
    hx, hy, hz = SHEET_SIZE[0] / 2, SHEET_SIZE[1] / 2, SHEET_SIZE[2] / 2
    loc = np.array([[sx * hx, sy * hy, hz] for sx in (-1, 1) for sy in (-1, 1)])
    return np.asarray(st.pos) + loc @ R.T


def now(env) -> dict:
    """This step's state: the focus error, clamp, stick, hands."""
    s = m.state(env, "laser")
    sh = m.state(env, "sheet")
    slide = s.joint("slide")
    sheet_top = float(np.mean(_sheet_corners(sh)[:, 2])) - TOP
    tip = SB0 + slide - REACH  # the hanging stick's tip (wherever the stick is now)
    e = tip - sheet_top
    loose = -s.deg("screw")
    stick = s.deg("stick")
    local = m.fingertips(env) - np.array([0.0, 0.0, TOP])
    sb = SB0 + slide
    mod_gap = _box_gap(local, (KNOB_X0, MOD_Y - MOD_W / 2, sb - REACH - 0.002), (X_BACK, MOD_Y + MOD_W / 2, sb + SHIELD_H + BODY_H))
    scr_gap = _box_gap(local, (SCREW_X0, SCREW_Y - SCREW_R, SCREW_Z - SCREW_R), (SCREW_X0 + SCREW_L, SCREW_Y + SCREW_R, SCREW_Z + SCREW_R))
    return {"e": e, "slide": slide, "loose": loose, "stick": stick, "locked": loose < CLAMP_DEG, "stowed": stick > -15.0,
            "gap": min(mod_gap, scr_gap), "sheet": sh, "sheet_v": float(np.linalg.norm(sh.lin_vel))}


class Trial:
    """Violation record: `dropped` (the module came down more than 1.5 mm in one 50 ms step: over 3 cm/s) and `moved` (a corner of the
    sheet more than 3 mm from where it lay after the settle). Both are cleared only by `reset` (the reset hook)."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.dropped = False
        self.moved = False
        self.corners0 = None
        self.prev = None
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}

    def step(self, env):
        c = now(env)
        corners = _sheet_corners(c["sheet"])
        if self.corners0 is None:
            self.corners0 = corners
        if float(np.max(np.linalg.norm(corners - self.corners0, axis=1))) > 0.003:
            self.moved = True
        if self.prev is not None and self.prev - c["slide"] > 0.0015:
            self.dropped = True
        still = self.prev is not None and abs(self.prev - c["slide"]) < 0.0005 and c["sheet_v"] < 0.01
        self.prev = c["slide"]
        released = c["gap"] >= 0.03
        in_focus, near = abs(c["e"]) <= 0.0005, abs(c["e"]) <= 0.002
        harm = self.dropped or self.moved
        ok = in_focus and c["locked"] and c["stowed"] and released and still and not harm
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS
        if harm:
            sc = 0.0
        elif self.full:
            sc = 1.0
        elif not (released and still):
            sc = 0.0
        elif in_focus and c["locked"]:
            sc = 0.75
        elif near and c["locked"]:
            sc = 0.5
        elif in_focus:
            sc = 0.25
        else:
            sc = 0.0
        self.score = sc
        self.last = {k: c[k] for k in ("e", "loose", "stick", "locked", "gap")}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published starting layout) ------------------------------------------------------------
# yam_arena's Waypoint takes a yaw for a straight-down grip. This task's demo also needs grips pointing forward (horizontal, 3 deg
# down) to pinch the two knobs on the module's front and roll about their axes, so a Waypoint's `yaw` may also be a 3 x 3 rotation
# matrix (the TCP's orientation); a number keeps the stock behaviour.
_stock_grip = _ydemo.grip_rotation


def _grip_rotation(yaw=0.0):
    a = np.asarray(yaw, float)
    return a if a.shape == (3, 3) else _stock_grip(float(yaw))


_ydemo.grip_rotation = _grip_rotation


def _rot(axis, deg) -> np.ndarray:
    ax = np.asarray(axis, float)
    ax = ax / np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    a = math.radians(deg)
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


TILT = 3.0  # the forward grip points this many degrees below horizontal


def forward(phi: float) -> np.ndarray:
    """A grip pointing forward (+x), rolled `phi` degrees about its approach axis (positive turns a pinched knob positive about +x).
    phi 0 closes the fingers along y."""
    h = _rot((0, 1, 0), -90.0 + TILT) @ _stock_grip(0.0)
    return _rot(h[:, 2], phi) @ h


DEMO_MODE = "full"  # red-team copies of this file change only this line (README, "Checks")


def demo():
    """Open loop, from the published starting layout. Right hand, pointing forward: pinch the stick's pivot knob and turn it half a turn,
    so the stick flips down over the module's face. Left hand, straight down: grip the module's body near its top and push up about
    9 mm against the clamp, taking the module's weight. Right hand: pinch the thumbscrew and loosen it a quarter turn. Left hand: lower
    the module 12 mm, onto the hanging stick, and ease off. Right hand: tighten the thumbscrew to its seat; then pinch the stick's knob
    again and turn it back up. Both hands let go and move clear.
    DEMO_MODE (red team): "nothing" plays no motion; "onearm" leaves the left hand out (nothing holds the loosened module); "careless"
    grips the module but loosens the thumbscrew without taking its weight first; "hold" never lets go of the module; "nostow" leaves the
    stick down; "unlocked" never tightens the thumbscrew again."""
    from yam_arena.demo import Waypoint as W
    from yam_arena.constants import FINGER_OPEN
    mode = DEMO_MODE
    if mode == "nothing":
        return {}
    T = TOP
    OPEN_K = -0.016  # 32 mm between the pads, round a knob
    PINCH_K = 0.0  # closed on a 12 mm knob
    GRIP_MOD = -0.014  # closed on the 50 mm module
    DEPTH = 0.013  # pads reach this far past the TCP
    CARRY = 0.0 if mode == "careless" else 0.009  # measured: the left arm sags this much carrying the 0.6 kg module

    def tcp_on(x_front, overlap):
        return x_front + overlap - DEPTH

    knob0 = np.array([tcp_on(KNOB_X0, 0.006), PIVOT_Y, T + PIVOT_Z0])
    knob1 = knob0 - np.array([0.0, 0.0, HIGH])  # after focusing
    screw = np.array([tcp_on(SCREW_X0, 0.010), SCREW_Y, T + SCREW_Z])
    back = np.array([-0.035, 0.0, 0.0])
    top0 = T + SB0 + SHIELD_H + BODY_H
    grip0 = np.array([MOD_X, MOD_Y, top0 - 0.012])
    R = []

    def r(d, p, f, rot, label=""):
        R.append(W(d, tuple(float(v) for v in p), f, label, rot))

    def turn(arm, p, f, phi0, phi1, total, label):
        n = max(1, int(round(abs(phi1 - phi0) / 30.0)))
        for k in range(1, n + 1):
            arm(total / n, p, f, forward(phi0 + (phi1 - phi0) * k / n), label)

    # right: flip the focus stick down by its pivot knob (half a turn, over the module's face)
    r(3.0, knob0 + back, OPEN_K, forward(90.0), "to the stick's knob")
    r(1.5, knob0, OPEN_K, forward(90.0), "round the knob")
    r(0.8, knob0, PINCH_K, forward(90.0), "pinch the knob")
    turn(r, knob0, PINCH_K, 90.0, -90.0, 3.0, "flip the stick down")
    r(0.6, knob0, OPEN_K, forward(-90.0), "let go")
    r(1.0, knob0 + back, OPEN_K, forward(-90.0), "back off")
    # right: loosen the thumbscrew a quarter turn
    r(2.0, screw + back, OPEN_K, forward(45.0), "to the thumbscrew")
    r(1.5, screw, OPEN_K, forward(45.0), "round the thumbscrew")
    r(0.8, screw, PINCH_K, forward(45.0), "pinch the thumbscrew")
    t_loosen = sum(w.duration for w in R)
    turn(r, screw, PINCH_K, 45.0, -45.0, 2.0, "loosen the thumbscrew")
    t_loose = sum(w.duration for w in R)
    LOWER_T = 5.0
    t_tight = t_loose + 0.5 + LOWER_T + 0.8  # the left hand lowers the module meanwhile
    r(t_tight - t_loose, screw, PINCH_K, forward(-45.0), "hold while the module is lowered")
    if mode == "unlocked":
        r(2.5, screw, PINCH_K, forward(-45.0), "(not tightened)")
        end_phi = -45.0
    else:
        turn(r, screw, PINCH_K, -45.0, 50.0, 2.0, "tighten the thumbscrew")
        r(0.5, screw, PINCH_K, forward(50.0), "seated")
        end_phi = 50.0
    t_tightened = sum(w.duration for w in R)
    r(0.6, screw, OPEN_K, forward(end_phi), "let go")
    r(1.0, screw + back, OPEN_K, forward(end_phi), "back off")
    if mode not in ("nostow", "unlocked"):
        # right: stow the stick (half a turn back)
        r(2.0, knob1 + back, OPEN_K, forward(-90.0), "to the stick's knob")
        r(1.5, knob1, OPEN_K, forward(-90.0), "round the knob")
        r(0.8, knob1, PINCH_K, forward(-90.0), "pinch the knob")
        turn(r, knob1, PINCH_K, -90.0, 90.0, 3.0, "stow the stick")
        r(0.6, knob1, OPEN_K, forward(90.0), "let go")
        r(1.2, knob1 + back, OPEN_K, forward(90.0), "back off")
    r(2.5, (0.22, -0.20, T + 0.20), FINGER_OPEN, 0.0, "clear")
    if mode == "onearm":
        return {"right": R}

    # left: grip the module's body near its top (straight down, fingers across y), carry it while it is loose, lower it onto the stick
    L = []

    def l(d, p, f, label=""):
        L.append(W(d, tuple(float(v) for v in p), f, label, 0.0))

    l(3.0, grip0 + np.array([0, 0, 0.035]), FINGER_OPEN, "above the module")
    l(2.0, grip0, FINGER_OPEN, "fingers either side of the module")
    l(0.8, grip0, GRIP_MOD, "grip the module")
    lift = grip0 + np.array([0, 0, CARRY])  # take the module's weight against the clamp (the arm gives about 9 mm under 6 N)
    l(1.0, lift, GRIP_MOD, "take its weight")
    l(t_loosen - 6.8, lift, GRIP_MOD, "hold")
    l(t_loose + 0.5 - t_loosen, lift, GRIP_MOD, "hold while the thumbscrew is loosened")
    goal = lift - np.array([0, 0, HIGH + 0.003])  # 3 mm past the contact: the arm gives
    l(LOWER_T - 0.8, goal, GRIP_MOD, "lower the module onto the stick")
    rest = goal + np.array([0, 0, 0.0025])  # ease off until the stick carries little more than a touch
    l(0.8, rest, GRIP_MOD, "ease off")
    l(t_tightened - (t_loose + 0.5 + LOWER_T), rest, GRIP_MOD, "hold while the thumbscrew is tightened")
    if mode == "hold":
        l(sum(w.duration for w in R) - t_tightened + 4.0, rest, GRIP_MOD, "(keeps holding)")
        return {"left": L, "right": R}
    l(0.6, rest, FINGER_OPEN, "let go")
    l(1.5, rest + np.array([0, 0, 0.04]), FINGER_OPEN, "lift away")
    l(2.0, (0.22, 0.22, T + 0.22), FINGER_OPEN, "clear")
    return {"left": L, "right": R}
