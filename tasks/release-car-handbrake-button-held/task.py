"""Release a car's hand parking brake that has been applied.

A centre-console hand parking-brake lever, installed between the two front seats of a right-hand-drive compact SUV: the
console trim and slot, the armrest box behind the lever, the shifter ahead of it, the passenger seat on the far side and
the inboard edge of the driver's seat cushion on the near side (the arms stand where the driver sits). The lever is
applied to its 7th ratchet click. A ratchet pawl holds it there against the brake cable's pull; under that pull the pawl
is pressed into its tooth and the release button, in the end of the grip, will not go in. Releasing it means taking the
cable's load off the pawl by raising the lever slightly, pushing the button in, and keeping it in all the way down to
the lever's bottom stop: let go of the button part-way and the pawl drops into the next tooth below.

Coordinates: station frame, metres, x forward from the arms, y to the robot's left, z up. The car's front is -y.
Lever angles are degrees above the bottom stop.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the Trial counts its own 1 s hold, so score() and success() agree
HOLD_STEPS = 20  # 1 s at 20 Hz
DEBUG = os.environ.get("HB_DEBUG", "") == "1"

LOOKS = {
    "trim": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.75),  # black grained console plastic
    "trim2": m.Look(color=m.rgb(0.16, 0.16, 0.17), roughness=0.6),
    "seat": m.Look(color=m.rgb(0.22, 0.22, 0.24), roughness=0.95),  # dark grey seat fabric
    "seam": m.Look(color=m.rgb(0.32, 0.32, 0.34), roughness=0.9),
    "steel": m.Look(color=m.rgb(0.30, 0.31, 0.32), metallic=0.6, roughness=0.45),  # black-painted pressed steel
    "grip": m.Look(color=m.rgb(0.06, 0.06, 0.06), roughness=0.85),  # black grip
    "chrome": m.Look(color=m.rgb(0.80, 0.80, 0.82), metallic=0.95, roughness=0.15),
    "boot": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.95),
    "leather": m.Look(color=m.rgb(0.12, 0.11, 0.11), roughness=0.6),
}

# -- the lever ------------------------------------------------------------------------------------------------------
PIVOT = np.array([0.270, -0.210, TOP + 0.050])  # the lever's pivot, its axis along x (across the car)
L_BAR = 0.175  # pressed-steel arm from the pivot to the grip
R_GRIP0, R_GRIP1 = 0.175, 0.275  # the grip, from the pivot
R_GRIP = 0.5 * (R_GRIP0 + R_GRIP1)  # its middle, where a hand pulls: 225 mm from the pivot
R_CAP = 0.290  # the chrome end cap ends here
R_BTN = 0.298  # the button's face, standing 8 mm out of the cap
GRIP_R = 0.017  # the grip, 34 mm round
BAR_T, BAR_D = 0.012, 0.026  # the steel arm: 12 mm thick (x), 26 mm deep (z)
BTN_R, BTN_TRAVEL = 0.008, 0.008  # the button, 16 mm across, 8 mm travel
BTN_FREE = 0.005  # pressed in this far, the pawl is lifted clear of the teeth
BTN_K, BTN_PRE = 1000.0, 0.003  # its spring: 3 N standing out, 11 N fully in (estimated)
JAM = 0.0008  # with the cable's load on the pawl, the button goes in no further than this
PITCH = 4.0  # degrees per ratchet tooth
TEETH = 10  # teeth 1..10, at 4 to 40 degrees
START_TOOTH = 7
START = START_TOOTH * PITCH  # applied: on the 7th click, 28 degrees
UPPER = 40.0  # fully up
MASS = 0.70  # the moving lever: steel arm, ratchet sector, grip and pawl rod (estimated)

# The cable: a spring whose pull on the lever, referred to the middle of the grip, is F_START at the start click and
# falls linearly to F_START / 8 at the bottom stop (the brake's return springs). See README, "The cable's load".
F_START = float(os.environ.get("HB_GRIP_N", "196"))
SLACK_DEG = 4.0  # the spring's rest lies 4 degrees below the bottom stop
K_CABLE = F_START * R_GRIP / math.radians(START + SLACK_DEG)  # N m per radian

Q = (math.sin(math.radians(START) / 2), 0.0, 0.0, math.cos(math.radians(START) / 2))  # both mechanisms are authored on the 7th click


def _lever() -> m.Part:
    parts, cols = [], []
    bar = m.box((BAR_T, L_BAR + 0.012, BAR_D), (0.0, (L_BAR - 0.012) / 2, 0.0))
    boss = m.cylinder(0.016, 0.026, (0.0, 0.0, 0.0), axis="x", segments=24)
    grip = m.lathe([(0.0, R_GRIP0), (GRIP_R - 0.003, R_GRIP0), (GRIP_R, R_GRIP0 + 0.01), (GRIP_R, R_GRIP1 - 0.008), (GRIP_R - 0.002, R_GRIP1), (0.0, R_GRIP1)], 28)
    grip = m.Mesh(grip.v[:, [0, 2, 1]], grip.f[:, ::-1])  # lathe is about z: turn it to lie along +y
    cap = m.cylinder(0.014, R_CAP - R_GRIP1, (0.0, 0.5 * (R_GRIP1 + R_CAP), 0.0), axis="y", segments=28)
    sector = m.prism([(0.0, 0.02), (0.06, 0.035), (0.08, 0.0), (0.06, -0.035), (0.0, -0.02)], -0.003, 0.003)  # the toothed ratchet sector, seen beside the pivot
    sector = m.Mesh(sector.v[:, [2, 0, 1]] * np.array([1.0, 1.0, 1.0]) + np.array([0.009, 0.0, -0.030]), sector.f)
    rod = m.cylinder(0.003, R_GRIP0 - 0.02, (0.0, 0.5 * (R_GRIP0 + 0.02), BAR_D / 2 + 0.002), axis="y", segments=10)
    parts += [("bar", bar, "steel"), ("boss", boss, "steel"), ("grip", grip, "grip"), ("cap", cap, "chrome"), ("sector", sector, "steel"), ("rod", rod, "steel")]
    cols += [m.col_box("bar_c", (BAR_T, L_BAR + 0.012, BAR_D), (0.0, (L_BAR - 0.012) / 2, 0.0)),
             m.col_hull("grip_c", m.cylinder(GRIP_R, R_GRIP1 - R_GRIP0, (0.0, 0.5 * (R_GRIP0 + R_GRIP1), 0.0), axis="y", segments=20)),
             m.col_hull("cap_c", cap)]
    lever = m.hinge("lever", "X", (0.0, 0.0, 0.0), -START, UPPER - START, parts, cols, mass=MASS, spring=K_CABLE, damping=0.6, friction=0.05,
                    effort=500.0, gravity=True, com=(0.0, 0.13, 0.0), inertia=(0.016, 0.0006, 0.016))
    btn_mesh = m.cylinder(BTN_R, R_BTN - R_CAP + 0.004, (0.0, 0.5 * (R_CAP - 0.004 + R_BTN), 0.0), axis="y", segments=24)
    button = m.slider("button", "Y", -BTN_TRAVEL, 0.0, [("button", btn_mesh, "chrome")], [m.col_hull("button_c", btn_mesh)], at=(0.0, R_CAP, 0.0),
                      mass=0.02, spring=BTN_K, damping=3.0, friction=0.0, effort=200.0, parent="lever")
    return m.mechanism("handbrake", tuple(PIVOT), [lever, button], LOOKS, start_quat=Q, solver_iters=64)


def _pawl() -> m.Part:
    """The pawl's stop: where the lever comes to rest on its ratchet. A block under the arm, on its own joint about the
    pivot, that the task holds on the tooth the pawl is in (or drops clear while the button is in)."""
    block = m.box((0.010, 0.060, 0.008), (0.0, 0.070, -BAR_D / 2 - 0.0045))
    stop = m.hinge("stop", "X", (0.0, 0.0, 0.0), -START - 3.0, UPPER - START, [("stop", block, "steel", True)], [m.col_box("stop_c", (0.010, 0.060, 0.008), (0.0, 0.070, -BAR_D / 2 - 0.0045))],
                   mass=0.3, spring=30000.0, damping=60.0, friction=0.0, effort=5000.0, inertia=(0.002, 0.0005, 0.002))
    return m.mechanism("pawl", tuple(PIVOT), [stop], LOOKS, start_quat=Q, solver_iters=64)


# -- the installed surroundings (fixed) -------------------------------------------------------------------------------
CON_X0, CON_X1 = 0.170, 0.370  # the centre console, 200 mm wide
SLOT_X0, SLOT_X1 = 0.255, 0.285  # the slot the lever stands in, 30 mm wide
SLOT_Y0, SLOT_Y1 = -0.240, 0.115
TRIM_Z = 0.025  # the console's top trim round the lever, above the table
ARM_Y0, ARM_Z = 0.130, 0.130  # the armrest box behind the lever: its front face, and its lid's top
PASS_X0 = 0.390  # the passenger seat's inboard bolster
DRV_X1 = 0.150  # the driver's seat cushion's inboard bolster


def _box(parts, cols, name, size, centre, look, collide=True):
    parts.append((name, m.box(size, centre), look))
    if collide:
        cols.append(m.col_box(f"{name}_c", size, centre))


def _cabin() -> m.Part:
    parts, cols = [], []
    w = CON_X1 - CON_X0
    # console: side walls and the top trim, leaving the slot open
    _box(parts, cols, "con_side_l", (0.012, 0.78, TRIM_Z), (CON_X0 + 0.006, -0.03, TRIM_Z / 2), "trim")
    _box(parts, cols, "con_side_r", (0.012, 0.78, TRIM_Z), (CON_X1 - 0.006, -0.03, TRIM_Z / 2), "trim")
    _box(parts, cols, "trim_near", (SLOT_X0 - CON_X0, SLOT_Y1 - SLOT_Y0, 0.006), (0.5 * (CON_X0 + SLOT_X0), 0.5 * (SLOT_Y0 + SLOT_Y1), TRIM_Z - 0.003), "trim2")
    _box(parts, cols, "trim_far", (CON_X1 - SLOT_X1, SLOT_Y1 - SLOT_Y0, 0.006), (0.5 * (CON_X1 + SLOT_X1), 0.5 * (SLOT_Y0 + SLOT_Y1), TRIM_Z - 0.003), "trim2")
    _box(parts, cols, "trim_front", (w, 0.18, 0.006), (0.5 * (CON_X0 + CON_X1), SLOT_Y0 - 0.09, TRIM_Z - 0.003), "trim2")
    _box(parts, cols, "trim_back", (w, ARM_Y0 - SLOT_Y1, 0.006), (0.5 * (CON_X0 + CON_X1), 0.5 * (SLOT_Y1 + ARM_Y0), TRIM_Z - 0.003), "trim2")
    _box(parts, [], "slot_floor", (SLOT_X1 - SLOT_X0, SLOT_Y1 - SLOT_Y0, 0.002), (0.5 * (SLOT_X0 + SLOT_X1), 0.5 * (SLOT_Y0 + SLOT_Y1), 0.004), "boot", collide=False)
    # cup holders ahead of the lever, and the shifter
    for k, y in enumerate((-0.300, -0.375)):
        parts.append((f"cup{k}", m.cylinder(0.038, 0.002, (0.5 * (CON_X0 + CON_X1), y, TRIM_Z + 0.001), segments=32), "boot"))
    parts.append(("shift_boot", m.cylinder(0.045, 0.030, (0.5 * (CON_X0 + CON_X1), -0.47, TRIM_Z + 0.015), segments=24), "boot"))
    stick = m.cylinder(0.008, 0.13, (0.5 * (CON_X0 + CON_X1), -0.47, TRIM_Z + 0.065), segments=12)
    knob = m.sphere(0.024, (0.5 * (CON_X0 + CON_X1), -0.47, TRIM_Z + 0.14))
    parts += [("stick", stick, "steel"), ("knob", knob, "leather")]
    cols += [m.col_hull("stick_c", stick), m.col_hull("knob_c", knob)]
    # the armrest box behind the lever
    _box(parts, cols, "armrest", (w, 0.30, ARM_Z - 0.012), (0.5 * (CON_X0 + CON_X1), ARM_Y0 + 0.15, (ARM_Z - 0.012) / 2), "trim")
    _box(parts, cols, "arm_lid", (w + 0.01, 0.31, 0.012), (0.5 * (CON_X0 + CON_X1), ARM_Y0 + 0.155, ARM_Z - 0.006), "leather")
    # the passenger seat, on the far side: cushion with its inboard bolster, and the backrest behind
    _box(parts, cols, "p_cushion", (0.50, 0.50, 0.070), (PASS_X0 + 0.25, -0.03, 0.035), "seat")
    _box(parts, cols, "p_bolster", (0.06, 0.50, 0.035), (PASS_X0 + 0.03, -0.03, 0.0875), "seat")
    _box(parts, [], "p_seam", (0.002, 0.48, 0.002), (PASS_X0 + 0.061, -0.03, 0.071), "seam", collide=False)
    _box(parts, cols, "p_back", (0.50, 0.11, 0.56), (PASS_X0 + 0.25, 0.275, 0.07 + 0.28), "seat")
    # the driver's seat: only its cushion's inboard bolster; the arms stand where the driver sits
    _box(parts, cols, "d_bolster", (0.08, 0.50, 0.100), (DRV_X1 - 0.04, -0.03, 0.05), "seat")
    return m.fixed("cabin", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.9, 0.8))


# -- props: things left on the passenger seat (distractors; scoring ignores them) ---------------------------------------
SEAT_TOP = TOP + 0.070
BOTTLE = SimReady("water_bottle", pos=(0.470, -0.150, SEAT_TOP), size=(0.068, 0.068, 0.245), mass=0.52, package="food_beverage_bottledWater_b01",
                  usd="sm_food_beverage_bottledWater_b01_01.usd", convex=True)
GLASSES = SimReady("glasses", pos=(0.560, 0.080, SEAT_TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                   usd="sm_accessory_eyeglasses_a01_01.usd", yaw=0.6)
CHIPS = SimReady("chips", pos=(0.600, -0.090, SEAT_TOP), size=(0.169, 0.118, 0.081), mass=0.15, package="food_processed_chips_a01",
                 usd="sm_food_processed_chips_a01_01.usd", yaw=-0.4)

OBJECTS = [_cabin(), _lever(), _pawl(), BOTTLE, GLASSES, CHIPS]


# -- the ratchet, the pawl and the jammed button ----------------------------------------------------------------------
def lever_deg(s) -> float:
    return START + s.deg("lever")


def button_in(s) -> float:
    return -float(s.joint("button"))


def tooth_below(theta: float) -> float:
    """The highest tooth the pawl can drop into with the lever at theta (degrees), or -2 (below the bottom stop) if none."""
    k = int(math.floor((theta - 0.6) / PITCH))
    return float(min(k, TEETH) * PITCH) if k >= 1 else -2.0


class Ratchet:
    """The pawl. Engaged unless the button is in BTN_FREE or more (re-engaging below 4 mm). Engaged, its stop sits on the
    highest tooth the lever has passed, and only rises: raised, the lever clicks over the teeth; lowered, it comes to rest
    on the tooth below. With the lever resting on the stop (the cable's load on the pawl), the button jams at JAM."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.engaged, self.stop, self.jammed = True, START, False
        if env is not None:
            m.set_target(env, "handbrake", "lever", -math.radians(START + SLACK_DEG))  # the cable pulls the lever down
            m.set_target(env, "handbrake", "button", BTN_PRE)  # the button's spring preload
            m.set_target(env, "pawl", "stop", 0.0)

    def step(self, env) -> None:
        s = m.state(env, "handbrake")
        theta, b = lever_deg(s), button_in(s)
        if self.engaged and b >= BTN_FREE:
            self.engaged = False
        elif not self.engaged and b <= BTN_FREE - 0.001:
            self.engaged, self.stop = True, -2.0
        if self.engaged:
            self.stop = max(self.stop, tooth_below(theta))
        target = self.stop if self.engaged else -2.0
        m.set_target(env, "pawl", "stop", math.radians(target - START))
        loaded = self.engaged and self.stop >= PITCH and theta - self.stop < 0.5
        if loaded and not self.jammed:
            m.lock(env, "handbrake", "button", stiffness=60000.0, effort=2000.0)
            m.set_target(env, "handbrake", "button", -min(b, JAM))
            self.jammed = True
        elif not loaded and self.jammed:
            m.unlock(env, "handbrake", "button")
            m.set_target(env, "handbrake", "button", BTN_PRE)
            self.jammed = False


# -- scoring ---------------------------------------------------------------------------------------------------------
HANDS_OFF = 0.03  # every fingertip point at least 3 cm from the lever
ON_STOP = 1.0  # degrees above the bottom stop that count as resting on it
LOWERED = START - 6.0  # at least one and a half teeth below the start click


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def hand_gap(env, s) -> float:
    """The nearest fingertip point's distance (m) to the lever's surface: its arm, grip, cap and button as segments along its axis."""
    local = s.to_local(m.fingertips(env))
    th = math.radians(s.deg("lever"))
    d = np.array([0.0, math.cos(th), math.sin(th)])
    pts = [(np.zeros(3), L_BAR * d, BAR_D / 2), (R_GRIP0 * d, R_CAP * d, GRIP_R), (R_CAP * d, R_BTN * d, BTN_R)]
    return float(min((_seg_dist(local, a, b) - r).min() for a, b, r in pts))


class Trial:
    """Facts about the run so far, cleared only by a reset of the scene (mechkit's on_reset)."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps, self.hold, self.full, self.score, self.last = 0, 0, False, 0.0, {}

    def step(self, env) -> None:
        RATCHET.step(env)
        self.steps += 1
        kick = float(os.environ.get("HB_KICK", "0"))
        if kick and self.steps in (20, 60, 100):
            m.push(env, "handbrake", (0.0, 0.0, kick if self.steps < 100 else -kick) if self.steps != 60 else (0.0, 0.0, 0.0),
                   at=along(lever_deg(m.state(env, "handbrake")), R_GRIP), body="lever")
        s = m.state(env, "handbrake")
        theta, b = lever_deg(s), button_in(s)
        gap = hand_gap(env, s)
        still = self._omega(env) < math.radians(2.0)
        on_stop = theta <= ON_STOP
        released = gap >= HANDS_OFF and b <= 0.001
        if on_stop and released and still:
            tier = 1.0
        elif on_stop:
            tier = 0.5
        elif theta <= LOWERED:
            tier = 0.25
        else:
            tier = 0.0
        self.hold = self.hold + 1 if tier == 1.0 else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else min(tier, 0.5)
        self.last = {"t": round(self.steps / 20.0, 2), "deg": round(theta, 2), "stop": RATCHET.stop if RATCHET.engaged else None, "btn_mm": round(1000 * b, 2),
                     "jam": RATCHET.jammed, "gap_mm": round(1000 * gap, 1), "tier": tier, "hold": self.hold}
        if DEBUG and (self.steps % 5 == 0):
            tips = m.fingertips(env)
            gw = s.to_world(np.array([[0.0, R_GRIP * math.cos(math.radians(s.deg("lever"))), R_GRIP * math.sin(math.radians(s.deg("lever")))]]))[0]
            extra = {"R_tips": np.round(tips[12:].mean(0), 3).tolist(), "R_open_mm": round(1000 * float(np.linalg.norm(tips[12] - tips[18])), 1),
                     "L_tips": np.round(tips[:12].mean(0), 3).tolist(), "grip_w": np.round(gw, 3).tolist()}
            print("[hb] " + json.dumps({**self.last, **extra}), flush=True)

    @staticmethod
    def _omega(env) -> float:
        asset = env.unwrapped.scene["handbrake"]
        i = asset.find_joints("lever_joint")[0][0]
        return abs(float(m._np(asset.data.joint_vel)[0, i]))


RATCHET = Ratchet()
TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(RATCHET.reset)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- the scripted demo ------------------------------------------------------------------------------------------------
def along(theta_deg: float, r: float, dz: float = 0.0) -> tuple[float, float, float]:
    """The world point r along the lever's axis from the pivot, with the lever at theta_deg, raised by dz."""
    th = math.radians(theta_deg)
    return (float(PIVOT[0]), float(PIVOT[1] + r * math.cos(th)), float(PIVOT[2] + r * math.sin(th) + dz))


def demo():
    from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN

    yaw_x = math.pi / 2  # fingers close along x, across the lever
    lift = float(os.environ.get("HB_LIFT", "0.06"))  # how far above the grip the right TCP is sent: the arm's stiffness turns it into a pull
    tcp = float(os.environ.get("HB_TCP", "-0.010"))  # the TCP below the grip's axis: the pads reach well below it, so it cannot wedge them apart
    g0 = along(START, R_GRIP, tcp)
    right = [
        Waypoint(2.5, (g0[0], g0[1], g0[2] + 0.10), FINGER_OPEN, "right: over the grip", yaw_x),
        Waypoint(1.5, g0, FINGER_OPEN, "right: fingers either side of the grip", yaw_x),
        Waypoint(0.8, g0, FINGER_CLOSED, "right: grip it", yaw_x),
        Waypoint(1.5, (g0[0], g0[1], g0[2] + lift), FINGER_CLOSED, "right: raise it off the pawl", yaw_x),
        Waypoint(4.0, (g0[0], g0[1], g0[2] + lift), FINGER_CLOSED, "right: hold it up", yaw_x),
    ]
    left = [Waypoint(14.0, (0.25, 0.25, TOP + 0.25), FINGER_CLOSED, "left: rest")]
    return {"left": left, "right": right}
