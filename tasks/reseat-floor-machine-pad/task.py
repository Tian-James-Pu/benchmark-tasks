"""Recentre a 17-inch floor pad that is lying off-centre on its pad driver, then push the centre lock home until it clicks.

The scene is the underside of a rotary floor machine at pad change: the pad driver (a 40.6 cm plastic disc on its
clutch plate), face up and level, held fixed. A new 17-inch (43.2 cm) red nonwoven floor pad, 2.54 cm thick with a
9.0 cm centre hole, has been dropped onto the driver 5 cm off-centre (towards the robot's left), so its hole is not
over the driver's hub. The centre lock (a moulded plastic plug with a wire spring clip) sits in the hub, raised and
not engaged, its stem just above the pad.

Model (declared in code):
- "driver": a mechanism whose frame is the driver plate and its clutch plate (fixed to the bench), and whose one link
  is the centre lock, a slider along the driver's axis. Joint zero is the raised start pose; -LOCK_TRAVEL is fully
  home (the lock's flange on the pad's top face). A weak spring (LOCK_K) holds it up when nothing presses it. The
  lock's stem (7.0 cm across) fits the pad's hole (9.0 cm) only when the pad is within 1.0 cm of centre; otherwise the
  stem lands on the pad and the lock cannot go down.
- The click (code, each control step): the lock pressed to within CLICK_GAP of fully home latches there (`m.lock`),
  as the wire clip snapping into the hub's groove does. The real lock is removable and released by squeezing the
  clip; here it is captive on its slider and, once clicked, stays clicked.
- "pad": one free rigid body, its ring built from 16 convex segments.

Scoring (see task.json "scoring"): kept by the Trial tracker below; the "pad off the driver" record is cleared only
in the reset hook.
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
INSTRUCTION = "Centre the new pad on the pad driver and lock it on with the centre lock."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the pad driver (frame), origin on the bench under its axis
CX, CY = 0.28, -0.03
HUB_R, HUB_H = 0.06, 0.025  # the clutch plate on the driver's back, which it lies on
DRV_R, DRV_T = 0.203, 0.020  # 16 in plate, grip face included
D = HUB_H + DRV_T  # the driver's face, above the bench
D_W = TOP + D

# -- the pad
PAD_RO, PAD_RI, PAD_T = 0.2159, 0.045, 0.0254  # 17 in, 9.0 cm hole, 1 in thick
PAD_MASS = 0.30
PAD_OFFSET = 0.05  # m off-centre at the start, towards +y
P = D + PAD_T  # the pad's top face when it lies on the driver

# -- the centre lock (link), authored raised
STEM_R, FLANGE_R, FLANGE_T = 0.035, 0.055, 0.012
STEM_L = PAD_T + 0.012  # through the pad and 12 mm into the hub
LOCK_TRAVEL = 0.041
STEM_BOTTOM0 = P - STEM_L + LOCK_TRAVEL  # 3.6 mm above the pad's top face at the start
LOCK_K = 150.0  # N/m: about 6 N at fully home
CLICK_GAP = 0.0025  # the clip catches within 2.5 mm of fully home

# -- scoring thresholds
CENTRE_TOL = 0.010
FLAT_DEG, FLAT_Z = 3.0, 0.003
OFF_Z, OFF_DEG = 0.010, 15.0
CLEAR = 0.03

LOOKS = {
    "driver": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.7),
    "grip": m.Look(color=m.rgb(0.20, 0.20, 0.21), roughness=0.95),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.9, roughness=0.35),
    "pad": m.Look(color=m.rgb(0.72, 0.10, 0.08), roughness=1.0),
    "lock": m.Look(color=m.rgb(0.93, 0.80, 0.12), roughness=0.5),
}


def _driver():
    frame_parts = [
        ("clutch_plate", m.cylinder(HUB_R, HUB_H, (0, 0, HUB_H / 2), segments=48), "steel"),
        ("plate", m.cylinder(DRV_R, DRV_T - 0.003, (0, 0, HUB_H + (DRV_T - 0.003) / 2), segments=64), "driver"),
        ("grip_face", m.lathe([(0.045, D - 0.003), (DRV_R - 0.004, D - 0.003), (DRV_R - 0.004, D), (0.045, D), (0.045, D - 0.003)], 64), "grip"),
        ("hub_ring", m.lathe([(0.037, D - 0.001), (0.045, D - 0.001), (0.045, D + 0.0005), (0.037, D + 0.0005), (0.037, D - 0.001)], 48), "steel"),
    ]
    frame_cols = [m.col_cylinder("hub_c", HUB_R, HUB_H, (0, 0, HUB_H / 2)), m.col_cylinder("plate_c", DRV_R, DRV_T, (0, 0, HUB_H + DRV_T / 2))]
    zf = STEM_BOTTOM0 + STEM_L
    clip = [(0.045 * math.cos(a), 0.045 * math.sin(a), zf + FLANGE_T + 0.002) for a in np.linspace(0.3, 2 * math.pi - 0.3, 40)]
    lock_parts = [
        ("stem", m.cylinder(STEM_R, STEM_L, (0, 0, STEM_BOTTOM0 + STEM_L / 2), segments=48), "lock"),
        ("flange", m.lathe([(0.0, zf), (FLANGE_R, zf), (FLANGE_R, zf + FLANGE_T - 0.003), (FLANGE_R - 0.004, zf + FLANGE_T), (0.0, zf + FLANGE_T)], 48), "lock"),
        ("clip", m.tube(clip, 0.0015, sides=8), "steel"),
    ]
    lock_cols = [m.col_cylinder("stem_c", STEM_R, STEM_L, (0, 0, STEM_BOTTOM0 + STEM_L / 2)),
                 m.col_cylinder("flange_c", FLANGE_R, FLANGE_T, (0, 0, zf + FLANGE_T / 2))]
    lock = m.slider("lock", "Z", -LOCK_TRAVEL, 0.0, lock_parts, lock_cols, mass=0.06, spring=LOCK_K, damping=2.0, friction=0.05, effort=50.0)
    return m.mechanism("driver", (CX, CY, TOP), [lock], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols)


def _pad():
    parts = [("pad", m.lathe([(PAD_RI, 0.0), (PAD_RO, 0.0), (PAD_RO, PAD_T), (PAD_RI, PAD_T), (PAD_RI, 0.0)], 64), "pad")]
    cols, n = [], 16
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        ri = PAD_RI / math.cos(math.pi / n)  # the hole's 16-gon touches the 9.0 cm circle from outside
        pts = [(ri * math.cos(a0), ri * math.sin(a0)), (PAD_RO * math.cos(a0), PAD_RO * math.sin(a0)),
               (PAD_RO * math.cos((a0 + a1) / 2), PAD_RO * math.sin((a0 + a1) / 2)),
               (PAD_RO * math.cos(a1), PAD_RO * math.sin(a1)), (ri * math.cos(a1), ri * math.sin(a1))]
        cols.append(m.col_hull(f"seg{i}_c", m.prism(pts, 0.0, PAD_T)))
    return m.free("pad", (CX, CY + PAD_OFFSET, D_W + 0.001), parts, cols, LOOKS, mass=PAD_MASS, friction=(0.9, 0.8))


DRIVER = _driver()
PAD = _pad()

# distractors: things on a janitor's cart, out of the work area
SPRAY = SimReady("spray_bottle", pos=(0.64, 0.25, TOP), size=(0.118, 0.040, 0.283), mass=0.6, package="cleaning_windowSpray_a01",
                 usd="sm_cleaning_windowSpray_a01_01.usd", yaw=math.radians(90))
GLOVES = SimReady("glove_box", pos=(0.63, -0.24, TOP), size=(0.178, 0.117, 0.056), mass=0.25, package="box_glove_a01",
                  usd="sm_box_glove_a01_01.usd", yaw=math.radians(80))
BRUSH = SimReady("scrub_brush", pos=(0.65, 0.0, TOP), size=(0.162, 0.086, 0.156), mass=0.2, package="cleaning_brush_a01",
                 usd="sm_cleaning_brush_a01_01.usd", yaw=math.radians(15))

OBJECTS = [DRIVER, PAD, SPRAY, GLOVES, BRUSH]


# -- scoring --------------------------------------------------------------------------------------------------------------


def _cyl_dist(pts, c, r, z0, z1):
    dr = np.maximum(0.0, np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) - r)
    dz = np.maximum(0.0, np.maximum(pts[:, 2] - z1, z0 - pts[:, 2]))
    return np.hypot(dr, dz)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the "pad off the driver" record is cleared
        self.off, self.clicked, self.hold, self.full, self.score = False, False, 0, False, 0.0

    def step(self, env):
        drv, pad = m.state(env, "driver"), m.state(env, "pad")
        j = drv.joint("lock")
        if not self.clicked and j <= -(LOCK_TRAVEL - CLICK_GAP):
            self.clicked = True
            m.lock(env, "driver", "lock")
        if pad.pos[2] < D_W - OFF_Z or pad.tilt_deg() > OFF_DEG:
            self.off = True
        centred = math.hypot(pad.pos[0] - CX, pad.pos[1] - CY) <= CENTRE_TOL
        flat = pad.tilt_deg() <= FLAT_DEG and abs(pad.pos[2] - D_W) <= FLAT_Z
        tips = m.fingertips(env)
        lock_top = TOP + STEM_BOTTOM0 + STEM_L + FLANGE_T + j
        clear = bool(np.all(_cyl_dist(tips, pad.pos, PAD_RO, pad.pos[2], pad.pos[2] + PAD_T) >= CLEAR)
                     and np.all(_cyl_dist(tips, (CX, CY), FLANGE_R, TOP + P, lock_top) >= CLEAR))
        ok = self.clicked and centred and flat and clear and not self.off
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.full:
            self.score = 1.0
        elif self.off:
            self.score = 0.0
        elif self.clicked and centred and flat:
            self.score = 0.5
        elif centred and flat:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- scripted demo (open loop, from the published starting layout) ---------------------------------------------------------

TIP = 0.013  # fingertip pads below the TCP
CLOSED = 0.0
HIGH = TOP + 0.15
PUSH_Z = D_W + PAD_T / 2 + TIP  # fingertips at the pad's mid-thickness
PUSH_LEAD = -0.0095  # TCP target relative to where the rim should end: the arm lags its target while pushing (measured)
PUSH_FROM = CY + PAD_OFFSET + PAD_RO + 0.04
PUSH_TO = CY + PAD_RO + PUSH_LEAD
LOCK_TOP0 = TOP + STEM_BOTTOM0 + STEM_L + FLANGE_T
PRESS_Z = LOCK_TOP0 - LOCK_TRAVEL - 0.030 + TIP  # 3 cm past fully home: the arm is compliant; the joint limit stops the lock


def demo():
    left = [
        Waypoint(2.0, (CX, PUSH_FROM, HIGH), CLOSED, "above the pad's far rim"),
        Waypoint(1.5, (CX, PUSH_FROM, PUSH_Z), CLOSED, "down beside the rim"),
        Waypoint(2.5, (CX, PUSH_TO, PUSH_Z), CLOSED, "slide the pad to centre"),
        Waypoint(6.5, (CX, PUSH_TO, PUSH_Z), CLOSED, "hold the pad while the lock goes in"),
        Waypoint(0.6, (CX, PUSH_TO + 0.01, PUSH_Z), CLOSED, "off the rim"),
        Waypoint(1.5, (CX, PUSH_TO + 0.03, HIGH), CLOSED, "up and clear"),
        Waypoint(4.0, (CX, PUSH_TO + 0.03, HIGH), CLOSED, "wait"),
    ]
    right = [
        Waypoint(6.0, (CX, CY - 0.10, HIGH), CLOSED, "wait clear of the pad"),
        Waypoint(1.5, (CX, CY, HIGH), CLOSED, "over the centre lock"),
        Waypoint(1.0, (CX, CY, LOCK_TOP0 + TIP + 0.01), CLOSED, "onto the lock"),
        Waypoint(2.0, (CX, CY, PRESS_Z), CLOSED, "push it home"),
        Waypoint(0.8, (CX, CY, PRESS_Z), CLOSED, "hold"),
        Waypoint(1.5, (CX, CY, HIGH), CLOSED, "lift clear"),
        Waypoint(6.0, (CX, CY - 0.10, HIGH), CLOSED, "wait"),
    ]
    return {"left": left, "right": right}
