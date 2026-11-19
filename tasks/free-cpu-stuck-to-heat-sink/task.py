"""Get a server processor off the heat sink it came out of the socket stuck to, and into the empty pocket of a processor tray, contact side up.

The scene: a 1U passive heat sink for an Intel Xeon Scalable (LGA3647) processor (108 x 78 x 25.5 mm, 455 g, as Supermicro's SNK-P0067PS),
lying upside down on the bench, fins down. On its base, held by the old thermal paste, is the processor (package 76.0 x 56.5 mm), its heat
spreader on the paste and its land side (the gold contacts) facing up. Beside it, an anti-static processor tray with one empty pocket.

Dell's PowerEdge 1955 manual: "the processor might adhere to the heat sink and be removed from the socket" ... "twist or slide the processor off
of the heat sink. Do not pry the processor off of the heat sink." Lenovo's ThinkSystem SR950 manual breaks the processor-to-heat-sink seal
"using a twisting motion", then has the processor held "with the thermal-grease side down and the processor-contact side up", and says "Do not
touch the processor socket or processor contacts."

Model (declared in code):
- The paste bond: while the seal holds, the hook presses the processor onto the heat sink's base with BOND_N and pulls the heat sink up with
  the same force (an internal pair: the heat sink's weight on the bench is unchanged). The processor can only turn or slide on the base
  against the friction this load gives; the heat sink is loose on its fins and turns or slides if dragged harder than its own friction.
- The seal breaks (the pair is removed) once the processor has turned TWIST_DEG each way relative to the heat sink (twisted), or has slid
  SLIDE_M across the base (slid): Dell's two ways.
- Pried or pulled (scored 0): any corner of the processor's underside more than PRY_M off the base while the seal holds.
- Contacts touched (scored 0): any fingertip point over the land side (inset INSET) and within the band that puts a finger on it.
- Dropped (scored 0): the processor falling faster than 0.5 m/s or tilted past 30 degrees, at any time.
All three are latched in Trial.step and cleared only in Trial.reset (the mechkit on_reset hook).
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
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "The processor came out of the socket stuck to the heat sink. Get it off the heat sink and into the empty tray pocket, contact side up."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the heat sink: origin at the centre of its fin tips (on the bench), long side along world x; base top at HS_H
HS_XY = (0.30, -0.02)
YAW = 90.0  # heat sink, processor and tray pocket all lie with their long side along world y
HS_L, HS_W, HS_H, BASE_T = 0.108, 0.078, 0.0255, 0.0035
HS_MASS = 0.455
# -- the processor: origin at the centre of its heat spreader's face (on the paste), long side along x, land side up
CPU_L, CPU_W, CPU_H = 0.076, 0.0565, 0.0055  # package 76.0 x 56.5 mm; overall height approximate
SUB_T = 0.0015  # the substrate on top of the heat spreader
IHS_L, IHS_W = 0.064, 0.048  # heat spreader footprint (approximate)
CPU_MASS = 0.05
CPU_XY = HS_XY
# -- the paste bond
BOND_N = 5.0
PASTE_MU = (0.3, 0.25)  # the processor's friction (PhysX averages it with the other surface's): paste between the faces
TWIST_DEG = 3.0
SLIDE_M = 0.005
PRY_M = 0.0015
INSET = 0.008  # the contact check's inset from the package's edges: a pad gripping an edge reads up to ~5 mm inside it
# -- the tray: origin at the centre of its underside, pocket long side along x
TRAY_XY = (0.21, -0.17)
TRAY_L, TRAY_W, FLOOR_T = 0.11, 0.09, 0.003
POCKET_HX, POCKET_HY, LOC_H, LOC_T, CUT_X = 0.0405, 0.0305, 0.004, 0.003, 0.024

LOOKS = {
    "alu": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.85, roughness=0.35),
    "copper": m.Look(color=m.rgb(0.80, 0.50, 0.32), metallic=0.9, roughness=0.3),
    "paste": m.Look(color=m.rgb(0.55, 0.56, 0.58), roughness=0.8),
    "nickel": m.Look(color=m.rgb(0.80, 0.80, 0.78), metallic=0.95, roughness=0.25),
    "substrate": m.Look(color=m.rgb(0.10, 0.32, 0.18), roughness=0.5),
    "gold": m.Look(color=m.rgb(0.90, 0.72, 0.30), metallic=1.0, roughness=0.3),
    "tray": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.7),
}


def _heatsink():
    parts = [("base", m.box((HS_L, HS_W, BASE_T), (0, 0, HS_H - BASE_T / 2)), "alu"),
             ("paste", m.box((IHS_L + 0.004, IHS_W + 0.004, 0.0002), (0, 0, HS_H + 0.0001)), "paste")]
    n = 24
    for i, y in enumerate(np.linspace(-HS_W / 2 + 0.0004, HS_W / 2 - 0.0004, n)):
        parts.append((f"fin{i}", m.box((HS_L, 0.0008, HS_H - BASE_T), (0, y, (HS_H - BASE_T) / 2)), "alu"))
    cols = [m.col_box("fins_c", (HS_L, HS_W, HS_H - BASE_T), (0, 0, (HS_H - BASE_T) / 2)),
            m.col_box("base_c", (HS_L, HS_W, BASE_T), (0, 0, HS_H - BASE_T / 2))]
    return m.free("heatsink", (HS_XY[0], HS_XY[1], TOP), parts, cols, LOOKS, mass=HS_MASS, friction=(0.9, 0.8), yaw_deg=YAW)


def _cpu():
    ihs_h = CPU_H - SUB_T
    lands = []
    for ix in np.linspace(-0.032, 0.032, 16):
        for iy in np.linspace(-0.022, 0.022, 12):
            lands.append(m.box((0.0018, 0.0018, 0.0002), (ix, iy, CPU_H + 0.0001)))
    parts = [("ihs", m.box((IHS_L, IHS_W, ihs_h), (0, 0, ihs_h / 2)), "nickel"),
             ("substrate", m.box((CPU_L, CPU_W, SUB_T), (0, 0, ihs_h + SUB_T / 2)), "substrate"),
             ("lands", m.join(*lands), "gold")]
    cols = [m.col_box("pkg_c", (CPU_L, CPU_W, CPU_H), (0, 0, CPU_H / 2))]
    return m.free("cpu", (CPU_XY[0], CPU_XY[1], TOP + HS_H + 0.0003), parts, cols, LOOKS, mass=CPU_MASS, friction=PASTE_MU, yaw_deg=YAW)


def _tray():
    """A moulded tray pocket: a floor, a locating wall across each end, and locating walls along each long side from each end to CUT_X
    from the centre, leaving a finger cut-out 48 mm long in the middle of each long side."""
    parts, cols = [("floor", m.box((TRAY_L, TRAY_W, FLOOR_T), (0, 0, FLOOR_T / 2)), "tray")], [m.col_box("floor_c", (TRAY_L, TRAY_W, FLOOR_T), (0, 0, FLOOR_T / 2))]
    z = FLOOR_T + LOC_H / 2
    walls = [((LOC_T, 2 * POCKET_HY + 2 * LOC_T, LOC_H), (sx * (POCKET_HX + LOC_T / 2), 0.0, z)) for sx in (-1, 1)]
    walls += [((POCKET_HX - CUT_X, LOC_T, LOC_H), (sx * (POCKET_HX + CUT_X) / 2, sy * (POCKET_HY + LOC_T / 2), z)) for sx in (-1, 1) for sy in (-1, 1)]
    for k, (size, c) in enumerate(walls):
        parts.append((f"wall{k}", m.box(size, c), "tray"))
        cols.append(m.col_box(f"wall{k}_c", size, c))
    return m.fixed("tray", (TRAY_XY[0], TRAY_XY[1], TOP), parts, cols, LOOKS, friction=(0.9, 0.8), yaw_deg=YAW)


OBJECTS = [
    _heatsink(),
    _cpu(),
    _tray(),
    # distractors (scoring ignores them)
    SimReady("screwdriver", pos=(0.10, 0.17, TOP), size=(0.224, 0.038, 0.036), mass=0.08, package="handTool_screwdriver_torx_t30_a01",
             usd="sm_handTool_screwdriver_torx_t30_a01_01.usd", yaw=0.0),
    SimReady("mug", pos=(0.46, 0.18, TOP), size=(0.114, 0.09, 0.08), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.0),
]

# processor geometry in its own frame
_CORNERS = np.array([[sx * CPU_L / 2, sy * CPU_W / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)])


def _yaw(q) -> float:
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _box_gap(local: np.ndarray, half: np.ndarray, centre: np.ndarray) -> np.ndarray:
    d = np.maximum(np.abs(local - centre) - half, 0.0)
    return np.linalg.norm(d, axis=1)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.pried = self.touched = self.dropped = False
        self.sealed, self.broken, self.clear, self.full = True, False, False, False
        self.hold, self.score = 0, 0.0
        self.tw_max = self.tw_min = 0.0
        self.yaw0, self.off0 = None, None
        self.how = ""
        if env is not None:
            hs, cpu = m.state(env, "heatsink"), m.state(env, "cpu")
            self.yaw0 = _wrap(_yaw(cpu.quat_xyzw) - _yaw(hs.quat_xyzw))
            self.off0 = hs.to_local(cpu.pos)[0][:2].copy()
            m.push(env, "cpu", (0.0, 0.0, -BOND_N))
            m.push(env, "heatsink", (0.0, 0.0, BOND_N))

    def _unbond(self, env):
        self.sealed = False
        m.push(env, "cpu", (0.0, 0.0, 0.0))
        m.push(env, "heatsink", (0.0, 0.0, 0.0))

    def step(self, env):
        hs, cpu = m.state(env, "heatsink"), m.state(env, "cpu")
        if self.yaw0 is None:  # offline or before any reset: nothing to track
            return
        # the processor's underside corners, in the heat sink's frame, over its base
        corners = hs.to_local(cpu.to_world(_CORNERS))
        rise = corners[:, 2] - HS_H
        rel_yaw = _wrap(_yaw(cpu.quat_xyzw) - _yaw(hs.quat_xyzw) - self.yaw0)
        off = hs.to_local(cpu.pos)[0][:2] - self.off0
        if self.sealed:
            if float(rise.max()) > PRY_M:
                self.pried = True
                self.how = "pried"
                self._unbond(env)
            else:
                self.tw_max, self.tw_min = max(self.tw_max, rel_yaw), min(self.tw_min, rel_yaw)
                if (self.tw_max >= TWIST_DEG and self.tw_min <= -TWIST_DEG) or float(np.linalg.norm(off)) >= SLIDE_M:
                    self.broken = True
                    self.how = "twisted" if self.tw_max >= TWIST_DEG and self.tw_min <= -TWIST_DEG else "slid"
                    self._unbond(env)
        # contacts: any fingertip point over the land side (inset INSET), from 8 mm below its top (the finger's underside is ~5 mm above the
        # reported point) to 2 mm above it
        tips = cpu.to_local(m.fingertips(env))
        over = (np.abs(tips[:, 0]) < CPU_L / 2 - INSET) & (np.abs(tips[:, 1]) < CPU_W / 2 - INSET)
        band = (tips[:, 2] > CPU_H - 0.008) & (tips[:, 2] < CPU_H + 0.002)
        if bool(np.any(over & band)):
            self.touched = True
        if float(cpu.lin_vel[2]) < -0.5 or cpu.tilt_deg() > 30.0:
            self.dropped = True
        harm = self.pried or self.touched or self.dropped
        if self.broken and not harm:
            dist = float(np.linalg.norm(hs.to_local(cpu.pos)[0][:2]))
            if float(rise.min()) > 0.015 or dist > 0.07:
                self.clear = True
        # in the tray pocket
        tray = np.array([TRAY_XY[0], TRAY_XY[1], TOP + FLOOR_T])
        world_corners = cpu.to_world(_CORNERS)
        dxy = cpu.pos[:2] - tray[:2]
        yaw_err = abs(_wrap(_yaw(cpu.quat_xyzw) - YAW))
        yaw_err = min(yaw_err, 180.0 - yaw_err)
        tips_gap = _box_gap(tips, np.array([CPU_L / 2, CPU_W / 2, CPU_H / 2]), np.array([0.0, 0.0, CPU_H / 2]))
        seated = (abs(float(dxy[0])) < 0.004 and abs(float(dxy[1])) < 0.004 and yaw_err < 6.0 and cpu.tilt_deg() < 2.0
                  and float(np.max(np.abs(world_corners[:, 2] - tray[2]))) < 0.0015 and float(np.linalg.norm(cpu.lin_vel)) < 0.01
                  and float(tips_gap.min()) > 0.03)
        self.hold = self.hold + 1 if (seated and self.broken and not harm) else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if harm and not self.full:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif self.clear:
            self.score = 0.5
        elif self.broken:
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


# -- the scripted demo -----------------------------------------------------------------------------------------------------------------------

TIP = 0.013  # the pad tips below the TCP
BASE_TOP = TOP + HS_H
Q = math.pi / 2
PLACE_DX = 0.0025  # the arm settles about 2.5 mm short of the TCP target along world x when lowering into the pocket (measured)
PRE = -0.031  # fingers opened to about 62 mm before going down beside the 56.5 mm package


def _twist_plan(mode: str = "twist"):
    cx, cy = CPU_XY
    zg = BASE_TOP + TIP + 0.0015  # pad tips 1.5 mm above the heat sink's base, on the processor's long sides
    hi = BASE_TOP + 0.08
    tx, ty = TRAY_XY
    zt = TOP + FLOOR_T + TIP + 0.0015
    tw = math.radians(8.0)
    rest_l = [Waypoint(30.0, (0.25, 0.25, TOP + 0.20), FINGER_CLOSED, "left: rest")]
    if mode == "none":
        return {"left": rest_l, "right": [Waypoint(30.0, (0.25, -0.25, TOP + 0.20), FINGER_OPEN, "right: rest")]}
    right = [
        Waypoint(2.0, (cx, cy, hi), FINGER_OPEN, "right: over the processor", Q),
        Waypoint(1.0, (cx, cy, hi), PRE, "right: open to just wider than the package", Q),
        Waypoint(2.0, (cx, cy, zg + 0.01), PRE, "right: down", Q),
        Waypoint(1.0, (cx, cy, zg), PRE, "right: down to the sides", Q),
        Waypoint(1.0, (cx, cy, zg), FINGER_CLOSED, "right: grip the edges", Q),
    ]
    if mode == "pull":
        right += [Waypoint(1.5, (cx, cy, hi), FINGER_CLOSED, "right: pull it straight up", Q)]
    else:
        right += [
            Waypoint(1.0, (cx, cy, zg), FINGER_CLOSED, "right: settle the grip", Q),
            Waypoint(1.0, (cx, cy, zg), FINGER_CLOSED, "right: twist one way", Q + tw),
            Waypoint(0.4, (cx, cy, zg), FINGER_CLOSED, "right: hold", Q + tw),
            Waypoint(1.5, (cx, cy, zg), FINGER_CLOSED, "right: twist the other way", Q - tw),
            Waypoint(0.4, (cx, cy, zg), FINGER_CLOSED, "right: hold", Q - tw),
            Waypoint(1.0, (cx, cy, zg), FINGER_CLOSED, "right: back square", Q),
            Waypoint(1.5, (cx, cy, hi), FINGER_CLOSED, "right: lift it off", Q),
        ]
    px = tx + PLACE_DX
    right += [
        Waypoint(2.0, (px, ty, hi), FINGER_CLOSED, "right: carry to the tray", Q),
        Waypoint(1.5, (px, ty, zt + 0.01), FINGER_CLOSED, "right: lower", Q),
        Waypoint(1.0, (px, ty, zt), FINGER_CLOSED, "right: into the pocket", Q),
        Waypoint(0.8, (px, ty, zt), FINGER_OPEN, "right: let go", Q),
        Waypoint(1.0, (px, ty, TOP + 0.10), FINGER_OPEN, "right: up", Q),
        Waypoint(1.5, (0.25, -0.25, TOP + 0.20), FINGER_OPEN, "right: away", 0.0),
        Waypoint(5.0, (0.25, -0.25, TOP + 0.20), FINGER_OPEN, "right: wait", 0.0),
    ]
    return {"left": rest_l, "right": right}


def demo():
    """A fixed plan from the published starting layout: the right hand grips the processor by its long edges, turns it 8 degrees each way
    and back to break the paste seal (the heat sink's weight on its fins keeps it from turning), lifts it off, sets it in the tray pocket
    the same way up (land side up) and lets go. The left arm stays at rest."""
    return _twist_plan("twist")
