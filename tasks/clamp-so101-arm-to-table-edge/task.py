"""Clamp an SO-101 robot arm's base to the front edge of a small table with the two one-handed mini bar clamps its bill of
materials lists.

The SO-101 follower (TheRobotStudio SO-ARM100, full size, posed and rigid, unpowered) sits on its tape marks with the back face
of its base flush with the table's front edge and its arm laid forward over the table, so it needs no holding. Two 6 in (150 mm)
one-handed mini bar clamps (resin body, steel bar, pistol grip and trigger) stand upright on the bench in front of the table,
their jaws open 55 mm. Each must be lifted by its grip, its fixed jaw hooked under the tabletop and its sliding jaw set on one of
the base's two wings, then tightened by pumping the trigger until both jaws bear. The wings and the top together are 35.5 mm
thick.

Model (declared in code):
- Clamp ("clamp_l", "clamp_r"): a free articulation. Frame: the body (the sliding jaw and its pad, the collar round the bar, the
  pistol grip). Links: "bar", a slider along the bar (z) carrying the steel bar and the fixed jaw at its lower end, + closes;
  "trigger", a hinge (about y) under the grip with a return spring, squeezed towards the grip (negative); "release", the
  release lever on the back of the collar, a sprung slider pressed down.
- The clamp's drive and brake (code, each control step), as in a one-handed bar clamp: squeezing the trigger past its free
  play advances the bar towards closed by DRIVE_MM_PER_DEG for each degree; letting the trigger go moves nothing. The bar's
  spring (BAR_K) holds it at the ratchet's position, so the jaws do not open again (the brake) and, once both jaws bear, each
  further squeeze adds clamping force, up to FORCE_MAX. Pressing the release lever down frees the bar, which then slides
  either way.
- SO-101 ("so101"): one rigid body with the URDF's masses (0.632 kg) and centre of mass; collides through convex hulls of each
  wing of the base, the servo tower and each mesh of each link.
- Table: 18 mm birch plywood top on four legs, fixed.

    python build_so101.py                     # once: the SO-101's meshes and hulls (assets/so101_follower.npz)
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Clamp the robot arm's base to the table edge with the two clamps."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
STILL_STEPS = 10  # stillness is judged over the last 0.5 s
DT = 0.05

# -- the table: 18 mm birch plywood, 450 x 400 mm, its top 85 mm above the station's surface, front edge at x = EDGE
EDGE = 0.320
DESK_T = 0.018
DESK_TOP = TOP + 0.085
UNDER = DESK_TOP - DESK_T
DESK_X1, DESK_HY = EDGE + 0.450, 0.200
LEG = 0.040

# -- the SO-101: base_link's origin is 2.4 mm above the base's underside; its flat back face is at x = -22.4 mm
ARM = np.load(HERE / "assets" / "so101_follower.npz")
BACK = -0.0224  # base_link x of the base's back face
FOOT = 0.0024  # base_link's origin above the base's lowest face
WING_TOP = 0.0151  # base_link z of the wings' top faces (each wing spans y 20.5 to 55.5 mm out, x -22.4 to 64.6 mm)
BASE0 = np.array([EDGE - BACK, 0.0, DESK_TOP + FOOT])  # on its marks: back face flush with the table's edge, centred on y = 0
STACK = WING_TOP + FOOT + DESK_T  # wing over tabletop: 35.5 mm
# where the middle of a clamp's upper pad must bear on its wing (base_link frame): 20 to 60 mm in from the back face and
# 31 to 45 mm out from the centre line, so that the whole 20 mm-wide pad is on the wing
ZONE_IN = (0.020, 0.060)
ZONE_OUT = (0.031, 0.045)

# -- the clamps: 6 in one-handed mini bar clamp. Clamp frame: origin on the bar's axis at the sliding jaw's pad face, z up the bar
#    (towards the grip), x towards the jaws. Measurements are of the reference clamp (README) and rounded.
OPEN0 = 0.055  # the jaws' opening at the start
THROAT = 0.062  # jaw reach from the bar's axis (2-7/16 in)
PAD_X0, PAD_X1, PAD_T, PAD_W = 0.036, 0.062, 0.006, 0.020  # pads: from/to (x), thickness, width
PAD_MID = (PAD_X0 + PAD_X1) / 2
JAW_T, JAW_W = 0.016, 0.018
BAR_W, BAR_T, BAR_LEN = 0.012, 0.005, 0.240  # steel bar section (x, y) and length
COLLAR = (-0.012, 0.014, 0.112)  # collar round the bar: x from, x to, top z
TRIG_PIVOT = (0.016, 0.0, 0.055)
TRIG_Z = (0.050, 0.060)
TRIG_X1 = 0.066
GRIP_Z = (0.082, 0.104)
GRIP_X = (0.010, 0.075)
TRIG_STOP = 30.0  # degrees of trigger travel to the grip
TRIG_FREE = 6.0  # free play before the drive bites
DRIVE_MM_PER_DEG = 0.25  # bar travel per degree of squeeze past the free play: 6 mm for a full stroke
BAR_K = 12000.0  # N/m: the brake and drive's stiffness along the bar
FORCE_MAX = 300.0  # N: most clamping force the drive builds
MASS_BODY, MASS_BAR = 0.070, 0.050
CLAMP_Y = 0.043  # where each clamp goes: 43 mm either side of the arm's centre line, over the middle of its wing
BAR_X = EDGE - 0.010  # the bar's axis 10 mm in front of the table's edge, once fitted
START_X, START_Y = EDGE - 0.075, 0.095  # where each clamp stands at the start (its bar's axis)

LOOKS = {
    "ply": m.Look(color=m.rgb(0.80, 0.66, 0.47), roughness=0.7),
    "ply_edge": m.Look(color=m.rgb(0.70, 0.56, 0.38), roughness=0.75),
    "leg": m.Look(color=m.rgb(0.55, 0.42, 0.28), roughness=0.7),
    "tape": m.Look(color=m.rgb(0.10, 0.45, 0.85), roughness=0.6),
    "pla": m.Look(color=m.rgb(0.93, 0.93, 0.90), roughness=0.5),
    "servo": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.45),
    "resin": m.Look(color=m.rgb(0.95, 0.70, 0.05), roughness=0.45),
    "grip": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.8),
    "pad": m.Look(color=m.rgb(0.15, 0.15, 0.16), roughness=0.9),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.85, roughness=0.3),
    "red": m.Look(color=m.rgb(0.80, 0.10, 0.08), roughness=0.5),
}


def _table() -> m.Part:
    parts, cols = [], []
    size, ctr = (DESK_X1 - EDGE, 2 * DESK_HY, DESK_T), ((EDGE + DESK_X1) / 2, 0.0, DESK_TOP - TOP - DESK_T / 2)
    parts.append(("top", m.box(size, ctr), "ply"))
    cols.append(m.col_box("top_c", size, ctr))
    parts.append(("front", m.box((0.0015, 2 * DESK_HY, DESK_T), (EDGE + 0.0007, 0.0, ctr[2])), "ply_edge"))
    h = DESK_TOP - TOP - DESK_T
    for k, (x, y) in enumerate([(EDGE + 0.120, 0.175), (EDGE + 0.120, -0.175), (DESK_X1 - 0.030, 0.175), (DESK_X1 - 0.030, -0.175)]):
        parts.append((f"leg{k}", m.box((LEG, LEG, h), (x, y, h / 2)), "leg"))
        cols.append(m.col_box(f"leg{k}_c", (LEG, LEG, h), (x, y, h / 2)))
    z = DESK_TOP - TOP + 0.0002
    for k, s in enumerate((1, -1)):  # tape marks along the base's two sides at its back corners
        parts.append((f"mark{k}", m.box((0.030, 0.003, 0.0004), (EDGE + 0.015, s * 0.0575, z)), "tape"))
    return m.fixed("work_table", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.6, 0.5))


def _so101() -> m.Part:
    parts, cols = [], []
    k = 0
    while f"vis{k}_v" in ARM.files:
        kind = str(ARM[f"vis{k}_meta"][1])
        parts.append((f"m{k}", m.Mesh(ARM[f"vis{k}_v"].astype(float), ARM[f"vis{k}_f"]), "servo" if kind == "servo" else "pla"))
        k += 1
    k = 0
    while f"hull{k}_v" in ARM.files:
        cols.append(m.col_hull(f"h{k}", m.Mesh(ARM[f"hull{k}_v"].astype(float), ARM[f"hull{k}_f"])))
        k += 1
    return m.free("so101", tuple(BASE0 + np.array([0.0, 0.0, 0.0003])), parts, cols, LOOKS, mass=float(ARM["mass"]),
                  com=tuple(float(c) for c in ARM["com"]), friction=(0.6, 0.5))


def _bx(x0, x1, y, z0, z1):
    return (x1 - x0, 2 * y, z1 - z0), ((x0 + x1) / 2, 0.0, (z0 + z1) / 2)


def _side(outline_xz, half_w: float) -> m.Mesh:
    """A part of uniform width (y) from its side outline in the clamp's xz plane (counter-clockwise, star-shaped)."""
    return m.prism(outline_xz, -half_w, half_w).turned(90, "x")


def _clamp(name: str, side: float) -> m.Part:
    """One clamp standing upright on the bench on its fixed jaw and bar end, its jaws open OPEN0. Colliders are boxes;
    the jaws, grip and trigger are drawn in their own shapes inside those boxes."""
    zb = -OPEN0 - PAD_T - JAW_T  # the fixed jaw's underside
    looks_drawn = {
        "jaw": _side([(-0.006, PAD_T), (THROAT, PAD_T), (THROAT, PAD_T + 0.011), (0.020, PAD_T + JAW_T), (-0.006, PAD_T + JAW_T)], JAW_W / 2),
        "grip": m.join(m.tube([np.array([GRIP_X[0] + 0.002, 0.0, 0.093]), np.array([GRIP_X[1] - 0.011, 0.0, 0.093])], 0.010, sides=16),
                       m.sphere(0.010, (GRIP_X[1] - 0.011, 0.0, 0.093))),
        "fjaw": _side([(-0.006, zb), (0.020, zb), (THROAT, zb + JAW_T - 0.011), (THROAT, zb + JAW_T), (-0.006, zb + JAW_T)], JAW_W / 2),
    }
    fp, fc = [], []  # the frame: the body
    for n, (x0, x1, y, z0, z1), look in [
        ("pad", (PAD_X0, PAD_X1, PAD_W / 2, 0.0, PAD_T), "pad"),
        ("jaw", (-0.006, THROAT, JAW_W / 2, PAD_T, PAD_T + JAW_T), "resin"),
        ("collar", (COLLAR[0], COLLAR[1], 0.011, PAD_T, COLLAR[2]), "resin"),
        ("grip", (GRIP_X[0], GRIP_X[1], 0.010, GRIP_Z[0], GRIP_Z[1]), "grip"),
        ("guard", (GRIP_X[1] - 0.008, GRIP_X[1], 0.008, TRIG_Z[0] - 0.004, GRIP_Z[0]), "resin"),
    ]:
        size, ctr = _bx(x0, x1, y, z0, z1)
        fp.append((n, looks_drawn.get(n, m.box(size, ctr)), look))
        fc.append(m.col_box(f"{n}_c", size, ctr))
    bar_parts, bar_cols = [], []
    for n, (x0, x1, y, z0, z1), look in [
        ("bar", (-BAR_W / 2, BAR_W / 2, BAR_T / 2, zb, zb + BAR_LEN), "steel"),
        ("fjaw", (-0.006, THROAT, JAW_W / 2, zb, zb + JAW_T), "resin"),
        ("fpad", (PAD_X0, PAD_X1, PAD_W / 2, -OPEN0 - PAD_T, -OPEN0), "pad"),
    ]:
        size, ctr = _bx(x0, x1, y, z0, z1)
        bar_parts.append((n, looks_drawn.get(n, m.box(size, ctr)), look))
        bar_cols.append(m.col_box(f"{n}_c", size, ctr))
    bar = m.slider("bar", "Z", -0.090, OPEN0, bar_parts, bar_cols, mass=MASS_BAR, spring=BAR_K, damping=40.0, friction=0.0,
                   effort=FORCE_MAX + 50.0, gravity=True)
    tsize, tctr = _bx(TRIG_PIVOT[0] - 0.004, TRIG_X1, 0.008, TRIG_Z[0], TRIG_Z[1])
    trig_mesh = m.join(m.box((0.012, 0.012, 0.012), (TRIG_PIVOT[0] + 0.002, 0.0, TRIG_PIVOT[2])),
                       m.tube([np.array([TRIG_PIVOT[0], 0.0, 0.055]), np.array([0.045, 0.0, 0.054]), np.array([TRIG_X1 - 0.004, 0.0, 0.057])],
                              0.005, sides=12))
    trigger = m.hinge("trigger", "Y", TRIG_PIVOT, -TRIG_STOP, 0.0, [("trig", trig_mesh, "red")],
                      [m.col_box("trig_c", tsize, tctr)], mass=0.010, spring=0.3, damping=0.01, friction=0.0, effort=20.0)
    rsize, rctr = _bx(COLLAR[0] - 0.014, COLLAR[0], 0.008, 0.094, 0.100)
    release = m.slider("release", "Z", -0.004, 0.0, [("lever", m.box(rsize, rctr), "red")], [m.col_box("lever_c", rsize, rctr)],
                       mass=0.004, spring=300.0, damping=1.0, friction=0.0, effort=20.0)
    pos = (START_X, side * START_Y, TOP - zb + 0.0005)
    return m.mechanism(name, pos, [bar, trigger, release], LOOKS, fp, fc, friction=(0.9, 0.8), free_mass=MASS_BODY,
                       frame_inertia=(1.2e-4, 1.4e-4, 4e-5), solver_iters=64)


OBJECTS = [
    _table(), _so101(), _clamp("clamp_l", 1.0), _clamp("clamp_r", -1.0),
    # distractors (scoring ignores them): a mug and a screwdriver on the table either side of the laid-down arm
    SimReady("mug", pos=(EDGE + 0.250, 0.140, DESK_TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
             usd="sm_dishware_mug_a01_01.usda", yaw=math.radians(200.0)),
    SimReady("screwdriver", pos=(EDGE + 0.300, -0.140, DESK_TOP), size=(0.188, 0.034, 0.032), mass=0.07,
             package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
             yaw=math.radians(-10.0)),
]
CLAMPS = ("clamp_l", "clamp_r")


# -- the clamps' drive and brake ---------------------------------------------------------------------------------------


class Drive:
    """Each clamp's ratchet position (metres of bar travel, + closes) and its trigger's last angle."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.r = {c: 0.0 for c in CLAMPS}
        self.last = {c: 0.0 for c in CLAMPS}
        self.free = {c: False for c in CLAMPS}
        if env is not None:
            for c in CLAMPS:
                m.set_target(env, c, "bar", 0.0)

    def step(self, env):
        for c in CLAMPS:
            s = m.state(env, c)
            q = s.joint("bar")
            squeeze = -math.degrees(s.joint("trigger"))  # degrees towards the grip
            pressed = -s.joint("release") > 0.0025
            if pressed:  # the release lever frees the bar, which then slides either way
                if not self.free[c]:
                    m.lock(env, c, "bar", stiffness=0.0, effort=0.0)
                    self.free[c] = True
                self.r[c] = q
                self.last[c] = squeeze
                continue
            if self.free[c]:
                m.unlock(env, c, "bar")
                self.free[c] = False
                self.r[c] = q
            gain = squeeze - max(self.last[c], TRIG_FREE)
            if gain > 0.0:
                self.r[c] += gain * DRIVE_MM_PER_DEG / 1000.0
            self.r[c] = min(self.r[c], q + FORCE_MAX / BAR_K)  # the drive slips past its rated force
            self.last[c] = squeeze
            m.set_target(env, c, "bar", self.r[c])


DRIVE = Drive()
m.on_reset(DRIVE.reset)
m.on_step(DRIVE.step)

# -- scoring ----------------------------------------------------------------------------------------------------------

_ARM_V = np.concatenate([ARM[k] for k in ARM.files if k.startswith("vis") and k.endswith("_v")])
ARM_BOX = (_ARM_V.min(0) - 0.001, _ARM_V.max(0) + 0.001)  # the SO-101's bounds, base_link frame


def clamp_reading(cs, base) -> dict:
    """One clamp's measurements from its pose and joints and the base's pose (mm, degrees)."""
    rc, rb = cs.rotation(), base.rotation()
    q = cs.joint("bar")
    face = np.array([[PAD_MID, 0.0], [PAD_X0, -PAD_W / 2], [PAD_X0, PAD_W / 2], [PAD_X1, -PAD_W / 2], [PAD_X1, PAD_W / 2]])
    up = cs.pos + np.c_[face, np.zeros(5)] @ rc.T  # the sliding jaw's pad face: its middle and corners
    low = cs.pos + np.c_[face, np.full(5, -OPEN0 + q)] @ rc.T  # the fixed jaw's pad face
    u = (up - base.pos) @ rb  # in the base's frame
    return {
        "open_mm": round((OPEN0 - q) * 1000, 1),
        "up_in_mm": round((u[0, 0] - BACK) * 1000, 1), "up_out_mm": round(u[0, 1] * 1000, 1),
        "up_gap_mm": round(float(u[:, 2].min() - WING_TOP) * 1000, 2),  # the pad's nearest point to the wing's top
        "low_gap_mm": round(float((UNDER - low[:, 2]).min()) * 1000, 2), "low_in_mm": round((low[0, 0] - EDGE) * 1000, 1),
        "tilt": round(cs.tilt_deg(), 1),
        "yaw": round(math.degrees(math.atan2(rc[1, 0], rc[0, 0]) - math.atan2(rb[1, 0], rb[0, 0])), 1),
        "trig": round(-math.degrees(cs.joint("trigger")), 1),
        "pos_mm": [round(float(v) * 1000, 1) for v in cs.pos],
    }


def clamp_wing(r: dict) -> int:
    """The wing (+1 left, -1 right) this clamp secures across it and the tabletop, or 0: the sliding jaw's pad centred in a
    wing's zone and bearing on it (its nearest point within 0.5 mm), the fixed jaw's pad bearing on the tabletop's underside
    (likewise) with its middle at least 20 mm in from the edge, and the bar within 10 degrees of vertical."""
    out = abs(r["up_out_mm"]) / 1000
    ok = (ZONE_IN[0] <= r["up_in_mm"] / 1000 <= ZONE_IN[1] and ZONE_OUT[0] <= out <= ZONE_OUT[1]
          and -2.0 <= r["up_gap_mm"] <= 0.5 and -2.0 <= r["low_gap_mm"] <= 0.5 and r["low_in_mm"] >= 20.0
          and r["tilt"] <= 10.0)
    return (1 if r["up_out_mm"] > 0 else -1) if ok else 0


def base_reading(base) -> dict:
    rb = base.rotation()
    back_x = base.pos[0] + rb[0, 0] * BACK
    return {"back_mm": round((back_x - EDGE) * 1000, 1), "y_mm": round(base.pos[1] * 1000, 1),
            "yaw": round(math.degrees(math.atan2(rb[1, 0], rb[0, 0])), 2), "tilt": round(base.tilt_deg(), 2)}


def base_on_marks(b: dict) -> bool:
    """On its marks: back face within 2 mm of the table's edge (in or over), centred within 4 mm, square within 2 degrees,
    and level within 2.5 degrees (the laid-down pose leans 0.4 degrees)."""
    return bool(abs(b["back_mm"]) <= 2.0 and abs(b["y_mm"]) <= 4.0 and abs(b["yaw"]) <= 2.0 and b["tilt"] <= 2.5)


def _clamp_box(q: float):
    zb = -OPEN0 - PAD_T - JAW_T + q
    return (np.array([COLLAR[0] - 0.015, -0.012, zb]), np.array([GRIP_X[1] + 0.001, 0.012, max(COLLAR[2], zb + BAR_LEN)]))


def hands_clear(tips: np.ndarray, states: dict) -> tuple[bool, float]:
    """Every fingertip point of both grippers at least 30 mm from each clamp's bounds and from the SO-101's bounds."""
    dmin = 1.0
    boxes = [("so101", ARM_BOX)] + [(c, _clamp_box(states[c].joint("bar"))) for c in CLAMPS]
    for name, box in boxes:
        local = states[name].to_local(tips)
        d = np.linalg.norm(np.maximum(np.maximum(box[0] - local, local - box[1]), 0.0), axis=1)
        dmin = min(dmin, float(d.min()))
    return dmin >= 0.030, dmin


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the record of a tipped or dropped arm is cleared
        self.tipped, self.hold, self.score, self.steps, self.last = False, 0, 0.0, 0, {}
        self.track = []  # the last STILL_STEPS + 1 positions of the base and both clamps

    def step(self, env):
        self.steps += 1
        states = {n: m.state(env, n) for n in ("so101", *CLAMPS)}
        b = base_reading(states["so101"])
        if b["tilt"] > 10.0 or states["so101"].pos[2] < DESK_TOP - 0.02:
            self.tipped = True  # the arm tipped or went off the table: latched for the trial
        reads = {}
        for c in CLAMPS:
            r = clamp_reading(states[c], states["so101"])
            r["wing"] = clamp_wing(r)
            r["secured"] = r["wing"] != 0
            reads[c] = r
        tips = m.fingertips(env)
        clear, dmin = hands_clear(tips, states)
        marks = base_on_marks(b)
        n_sec = len({reads[c]["wing"] for c in CLAMPS if reads[c]["wing"]})  # wings secured: two clamps on one wing count once
        self.track = (self.track + [np.concatenate([states[n].pos for n in ("so101", *CLAMPS)])])[-(STILL_STEPS + 1):]
        moved = np.abs(np.array(self.track) - self.track[0]).max() if len(self.track) > STILL_STEPS else 1.0
        still = moved < 0.001  # nothing moved 1 mm in the last 0.5 s
        full_now = (not self.tipped) and marks and clear and n_sec == 2 and still
        self.hold = self.hold + 1 if full_now else 0
        if self.tipped or not marks or not clear:
            self.score = 0.0
        elif self.hold >= HOLD_STEPS:
            self.score = 1.0
        elif n_sec >= 1:
            self.score = 0.5
        else:
            self.score = 0.0
        self.last = {"t": round(self.steps * DT, 2), "score": self.score, "hold": self.hold, "tipped": self.tipped,
                     "clear": clear, "dmin_mm": round(dmin * 1000, 1), "base": b, **reads,
                     "ratchet_mm": {c: round(DRIVE.r[c] * 1000, 1) for c in CLAMPS}}
        if self.steps % 10 == 0:
            print("[clamp] " + json.dumps(self.last), flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.hold >= HOLD_STEPS and not TRIAL.tipped)


# -- the scripted demo: a fixed plan from the published starting layout ------------------------------------------------
# Each hand comes from its own side with its fingers level, pointing 60 degrees in from straight across, and closing up and
# down across the grip and the trigger. It lifts the clamp, hooks the fixed jaw under the tabletop beside the base, sets the
# sliding jaw on the wing, pumps the trigger until both jaws bear and the drive has loaded them, lets go and withdraws.
# Hand orientations go through a float subclass carrying a rotation matrix (yam_arena's Waypoint takes a yaw).
_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_takes_grips", False) and not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)
    _grip_rotation._orient = True
    _grip_rotation._takes_grips = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


class Orient(float):
    rot = np.eye(3)


YAW_IN = 30.0  # the fingers point this far round from straight across (towards +x)
PITCH = 10.0  # and this far down from level
PINCH = np.array([0.042, 0.0, (TRIG_Z[0] + GRIP_Z[1]) / 2])  # clamp frame: between the trigger's underside and the grip's top
TIP_PAST = 0.010  # fingertips this far past the pinch point
F_APPROACH, F_HOLD, F_SQUEEZE = -0.034, -0.0255, -0.013


def _hand(side: float) -> tuple[Orient, np.ndarray]:
    y = math.radians(-side * (90.0 - YAW_IN))  # left hand: fingers towards -y and +x
    p = math.radians(PITCH)
    f = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p)])
    c = np.array([math.sin(p) * math.cos(y), math.sin(p) * math.sin(y), math.cos(p)])
    o = Orient(0.0)
    o.rot = np.column_stack([c, np.cross(f, c), f])
    return o, f


def _arm_plan(side: float, mode: str) -> list[Waypoint]:
    h, f = _hand(side)
    tcp = lambda clamp_origin: tuple(float(v) for v in np.asarray(clamp_origin) + PINCH + f * (TIP_PAST - 0.013))  # noqa: E731
    zb = -OPEN0 - PAD_T - JAW_T
    o0 = np.array([START_X, side * START_Y, TOP - zb])  # the clamp's origin standing on the bench
    z_in = UNDER - 0.008 + OPEN0  # carried with the fixed jaw's pad 8 mm under the tabletop
    y1 = side * CLAMP_Y
    x1 = BAR_X
    if mode == "offwing":  # red team: set 14 mm further out, the pad hanging off the wing's edge
        y1 = side * (CLAMP_Y + 0.014)
    if mode == "tableonly":  # red team: clamped on the bare tabletop beside the base, not over a wing
        y1 = side * (CLAMP_Y + 0.040)
    lift = o0 + np.array([0.0, 0.0, z_in - o0[2] + 0.004])
    front = np.array([START_X, y1, lift[2]])
    hook = np.array([x1, y1, lift[2]])
    seat_z = DESK_TOP + WING_TOP + FOOT - 0.002 if mode != "tableonly" else DESK_TOP - 0.002
    seat = np.array([x1, y1, seat_z])
    w = [
        Waypoint(2.0, (0.20, side * 0.12, TOP + 0.30), F_APPROACH, "turn the hand", yaw=h),
        Waypoint(1.5, tcp(o0 - f * 0.05 + np.array([0.0, 0.0, 0.04])), F_APPROACH, "above beside the grip", yaw=h),
        Waypoint(1.0, tcp(o0 - f * 0.05), F_APPROACH, "beside the grip", yaw=h),
        Waypoint(0.5, tcp(o0 - f * 0.05), F_APPROACH, "settle", yaw=h),
        Waypoint(1.5, tcp(o0), F_APPROACH, "fingers over the grip and under the trigger", yaw=h),
        Waypoint(1.0, tcp(o0), F_HOLD, "hold grip and trigger", yaw=h),
        Waypoint(1.2, tcp(lift), F_HOLD, "lift", yaw=h),
        Waypoint(1.8, tcp(front), F_HOLD, "across to the wing", yaw=h),
        Waypoint(1.8, tcp(hook), F_HOLD, "fixed jaw under the top, sliding jaw over the wing", yaw=h),
        Waypoint(1.0, tcp(seat), F_HOLD, "sliding jaw down on the wing", yaw=h),
    ]
    strokes = 0 if mode == "noclamp" else (2 if mode == "short" else 8)
    for k in range(strokes):
        w += [Waypoint(0.8, tcp(seat), F_SQUEEZE, f"squeeze {k + 1}", yaw=h),
              Waypoint(0.6, tcp(seat), F_HOLD, f"let the trigger back {k + 1}", yaw=h)]
    if mode == "hold":  # red team: never lets go
        return w + [Waypoint(5.0, tcp(seat), F_HOLD, "keep holding", yaw=h)]
    away = seat - f * 0.09
    w += [
        Waypoint(1.0, tcp(seat), F_APPROACH, "let go", yaw=h),
        Waypoint(1.5, tcp(away), F_APPROACH, "back out", yaw=h),
        Waypoint(1.5, tcp(away + np.array([-0.04, 0.0, 0.08])), F_APPROACH, "away", yaw=h),
        Waypoint(4.0, tcp(away + np.array([-0.04, 0.0, 0.08])), F_APPROACH, "wait away", yaw=h),
    ]
    return w


def _stay(side: float, t: float) -> Waypoint:
    """Hold the arm's starting pose."""
    from yam_arena import ik, station
    from yam_arena.constants import FINGER_OPEN, READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES["left" if side > 0 else "right"])
    o = Orient(0.0)
    o.rot = rot
    return Waypoint(t, tuple(float(v) for v in pos), FINGER_OPEN, "wait at the start pose", yaw=o)


def demo() -> dict:
    """Both arms at once, each fitting the clamp on its own side. TASK_DEMO picks a red-team variant."""
    mode = os.environ.get("TASK_DEMO", "full")
    if mode == "none":
        return {"left": [_stay(1.0, 2.0)], "right": [_stay(-1.0, 2.0)]}
    left = _arm_plan(1.0, mode)
    if mode == "onearm":  # red team: only the left clamp is fitted
        return {"left": left, "right": [_stay(-1.0, sum(w.duration for w in left))]}
    return {"left": left, "right": _arm_plan(-1.0, mode)}
