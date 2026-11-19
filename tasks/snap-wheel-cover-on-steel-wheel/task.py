"""Snap a 13-inch plastic wheel cover onto a steel wheel, its valve notch over the valve stem.

A bare 13 x 4.5J steel wheel lies face up on the table (outboard flange up). Its rubber valve stem stands up inside the rim at the
robot's left. A 13-inch plastic wheel cover (hubcap) has been set loosely on the wheel, centred by its clips but turned so that its valve
notch is at the robot's side, a quarter turn away from the stem. The cover rests on its eight retaining clips, which sit on the rim's
outboard bead seat 2.75 cm above home.

Model (declared in code):
- Wheel ("wheel", fixed mechanism): the rim, drawn as a revolved 3 mm steel section, collides through rings of convex segments: the
  flange top (where the cover's lip seats), the bead seat's inner face (the lip the clips ride over), the groove behind it (the safety
  hump's inside, where the clips catch), the well's shoulder below, and the centre disc. Its one link is the valve stem: a rubber
  TR414-class snap-in stem (58 mm overall with cap), on a hinge at its base about the tangent to the rim, with a spring for the rubber's
  bending stiffness (0.5 N m/rad, so about 3 N at the cap bends it 15 degrees).
- Cover ("cover", free mechanism, 0.45 kg): a dished ABS cover 36.0 cm across with a valve notch 3.35 cm wide at the stem's radius and
  eight through slots, colliding through convex pieces (the notch and the slots are real gaps). Its eight retaining clips are links:
  each a sprung slider (the steel retaining ring's spring, 1,500 N/m) with a ramped head. Pressed down, the head's ramp meets the bead
  seat's lip and cams the clip in; 2.2 cm further the head clears the lip and springs out into the groove behind it, where its flat top
  catches under the lip.
- Scoring (code, each control step): a clip is engaged when the cover's lip beside it is within 1.0 mm of the flange and the clip has
  sprung back out to within 1.5 mm of its rest. The stem is in the notch when its axis passes within 1.1 cm of the notch's centre line.
  A stem bent more than 15 degrees at any time scores 0 for the trial (cleared only on reset).
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
INSTRUCTION = "Put the wheel cover on the wheel."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

WC = np.array([0.13, 0.0, TOP])  # the wheel's axis on the table
S = 1.0  # radial scale of the drawn rim and disc sections
DZ = 0.0  # extra rim width over the drawn 4.5J section, added between the well and the outboard bead seat

# -- the rim (wheel frame: origin on the table on the wheel's axis, z up; the outboard flange is up) ---------------------------------------
Z_F = 0.126 + DZ  # flange top: where the cover's lip seats (overall rim width with flanges 12.6 cm)
R_LIP = 0.1625  # the bead seat's inner face (the lip the clips ride over): 13 in bead seat diameter 330 mm, less 2.5 mm
Z_LIP_BOT = Z_F - 0.022  # the bead seat's lower edge: below it, the groove
R_GROOVE = R_LIP + 0.004  # the groove's floor (the safety hump's inside), 1.7 cm deep
R_FLANGE = R_GROOVE + 0.0165
SEG = 40  # collider segments round the rim

# -- the valve stem: at the wheel's +y (robot's left), rubber, hinged at its base about x (the rim's tangent there)
STEM_R, STEM_Z0, STEM_Z1 = 0.142, 0.080 + DZ, 0.138 + DZ
STEM_ANGLE = 90.0
STEM_BENT_DEG = 15.0

# -- the cover (cover frame: origin on the wheel's axis in the plane of the lip's underside; the notch on +x)
R_COVER = 0.180
NOTCH_HALF_DEG = math.degrees(0.01675 / STEM_R)  # 3.35 cm wide at the stem's radius
R_NOTCH_IN = 0.125
SLOT_R0, SLOT_R1, SPOKE_W = 0.072, 0.112, 0.024  # eight through slots 4 cm long radially, between eight parallel-sided spokes 2.4 cm wide
SPOKE_ANGLES = [22.5 + 45.0 * k for k in range(8)]  # the spokes' centre lines (cover frame), beside the clips
CLIP_ANGLES = [22.5 + 45.0 * k for k in range(8)]
CLIP_K = 1500.0  # N/m per clip
CLIP_TOP = -0.023  # the head's catching face, 1 mm under the lip's lower edge when seated
CLIP_OUT = R_LIP + 0.003  # the head's outer face at rest: 3 mm out past the lip
COVER_MASS = 0.45
Z_REST = Z_F + 0.0275  # where the cover rests on its clips' ramps, unseated
START_YAW = 180.0  # the notch at the robot's side, a quarter turn from the stem
NOTCH_TOL = 0.011  # m: the stem's axis from the notch's centre line
ENGAGE_GAP = 0.0010  # m: lip to flange beside a clip
SPRUNG = 0.0015  # m: a clip within this of its rest has snapped out
CLEAR = 0.03  # m: every fingertip point this far from the cover

LOOKS = {
    "steel": m.Look(color=m.rgb(0.13, 0.13, 0.14), metallic=0.6, roughness=0.45),  # black-painted steel wheel
    "steel_dark": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.6),
    "cover": m.Look(color=m.rgb(0.72, 0.73, 0.75), metallic=0.55, roughness=0.32),  # silver-painted ABS
    "cover_dark": m.Look(color=m.rgb(0.16, 0.16, 0.17), roughness=0.5),
    "clip": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.6),
    "rubber": m.Look(color=m.rgb(0.04, 0.04, 0.04), roughness=0.85),
    "brass": m.Look(color=m.rgb(0.75, 0.62, 0.30), metallic=0.9, roughness=0.3),
}


def revolve(section, a0_deg=0.0, a1_deg=360.0, n=64) -> m.Mesh:
    """A solid of revolution about z from a closed (r, z) section, counter-clockwise in the (r, z) plane. A partial sweep gets end caps."""
    sec = np.asarray(section, float)
    k = len(sec)
    full = abs(a1_deg - a0_deg) >= 359.999
    th = np.radians(np.linspace(a0_deg, a1_deg, n, endpoint=not full))
    rows = len(th)
    v = np.vstack([np.c_[sec[:, 0] * math.cos(t), sec[:, 0] * math.sin(t), sec[:, 1]] for t in th])
    f = []
    for i in range(rows if full else rows - 1):
        i2 = (i + 1) % rows
        for j in range(k):
            j2 = (j + 1) % k
            a, b, c, d = i * k + j, i * k + j2, i2 * k + j2, i2 * k + j
            f += [(a, d, c), (a, c, b)]
    if not full:
        c0 = sec.mean(axis=0)
        for i, sgn in ((0, -1), (rows - 1, 1)):
            t = th[i]
            ci = len(v)
            v = np.vstack([v, [c0[0] * math.cos(t), c0[0] * math.sin(t), c0[1]]])
            for j in range(k):
                j2 = (j + 1) % k
                f.append((ci, i * k + j2, i * k + j) if sgn < 0 else (ci, i * k + j, i * k + j2))
    return m.Mesh(v, np.array(f))


def _offset(centre, t):
    """A closed section from a polyline: offset t/2 to both sides."""
    p = np.asarray(centre, float)
    d = np.gradient(p, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    nrm = np.c_[d[:, 1], -d[:, 0]]
    return np.vstack([p + nrm * t / 2, (p - nrm * t / 2)[::-1]])


def hull(pts) -> m.Mesh:
    """Points as a mesh for a convex-hull collider: a fan over every point, so that all of them are referenced."""
    p = np.asarray(pts, float)
    return m.Mesh(p, np.array([(0, i, i + 1) for i in range(1, len(p) - 1)]))


def _wedge(name, r0, r1, z0, z1, a0, a1):
    """A ring segment collider: from r0 to r1, z0 to z1, angles a0 to a1 (degrees)."""
    pts = []
    for a in np.radians(np.linspace(a0, a1, 3)):
        for r in (r0, r1):
            for z in (z0, z1):
                pts.append((r * math.cos(a), r * math.sin(a), z))
    return m.col_hull(name, hull(pts))


# -- the wheel --------------------------------------------------------------------------------------------------------------------------

_RIM13 = [(0.181, 0.007), (0.181, 0.0015), (0.166, 0.0015), (0.164, 0.006), (0.164, 0.022), (0.167, 0.027), (0.167, 0.032), (0.147, 0.042),
          (0.147, 0.072), (0.168, 0.086), (0.168, 0.1025), (0.164, 0.1055), (0.164, 0.1245), (0.181, 0.1245), (0.181, 0.118)]
RIM_LINE = [(r * S, z + (DZ if z > 0.07 else 0.0)) for r, z in _RIM13]
DISC_LINE = [(r * S, z + DZ) for r, z in [(0.029, 0.088), (0.065, 0.088), (0.080, 0.076), (0.110, 0.076), (0.128, 0.083), (0.143, 0.083),
                                          (0.1475, 0.068)]]
PCD_R = 0.050  # 4 x 100 mm


def _wheel_frame():
    parts = [("rim", revolve(_offset(RIM_LINE, 0.003), n=96), "steel"), ("disc", revolve(_offset(DISC_LINE, 0.004), n=96), "steel")]
    for i in range(4):
        a = math.radians(45 + 90 * i)
        parts.append((f"stud_hole_{i}", m.cylinder(0.0070, 0.0012, (PCD_R * math.cos(a), PCD_R * math.sin(a), 0.0906 + DZ), segments=20),
                      "steel_dark"))
    for i in range(4):  # the disc's vent holes
        a = math.radians(90 * i)
        parts.append((f"vent_{i}", m.cylinder(0.014, 0.0012, (0.095 * math.cos(a), 0.095 * math.sin(a), 0.0786 + DZ), segments=20), "steel_dark"))
    cols = []
    da = 360.0 / SEG
    for i in range(SEG):
        a0, a1 = i * da, (i + 1) * da
        cols.append(_wedge(f"lip_{i}", R_LIP, R_GROOVE, Z_LIP_BOT, Z_F, a0, a1))
        cols.append(_wedge(f"flange_{i}", R_GROOVE, R_FLANGE, Z_F - 0.006, Z_F, a0, a1))
        cols.append(_wedge(f"groove_{i}", R_GROOVE, R_GROOVE + 0.0045, Z_F - 0.039, Z_LIP_BOT, a0, a1))
        cols.append(_wedge(f"shoulder_{i}", 0.147 * S, R_GROOVE + 0.0045, Z_F - 0.051, Z_F - 0.039, a0, a1))
    cols.append(m.col_cylinder("disc_c", 0.125 * S, 0.016, (0, 0, 0.080 + DZ)))
    return parts, cols


def _stem_link():
    a = math.radians(STEM_ANGLE)
    x, y = STEM_R * math.cos(a), STEM_R * math.sin(a)
    body = m.lathe([(0.0, STEM_Z0), (0.0072, STEM_Z0), (0.0072, STEM_Z0 + 0.004), (0.0057, STEM_Z0 + 0.006), (0.0047, STEM_Z1 - 0.019), (0.0, STEM_Z1 - 0.019)], 24)
    core = m.cylinder(0.0030, 0.007, (0, 0, STEM_Z1 - 0.0155), segments=16)
    cap = m.lathe([(0.0, STEM_Z1 - 0.012), (0.0042, STEM_Z1 - 0.012), (0.0042, STEM_Z1 - 0.001), (0.0035, STEM_Z1), (0.0, STEM_Z1)], 20)
    parts = [("stem_body", body.moved((x, y, 0)), "rubber"), ("stem_core", core.moved((x, y, 0)), "brass"), ("stem_cap", cap.moved((x, y, 0)), "rubber")]
    cols = [m.col_cylinder("stem_c", 0.0055, STEM_Z1 - STEM_Z0 - 0.006, (x, y, (STEM_Z0 + 0.006 + STEM_Z1) / 2))]
    return m.hinge("stem", "X", (x, y, STEM_Z0), -70.0, 70.0, parts, cols, mass=0.02, spring=0.5, damping=0.01, friction=0.0, effort=20.0,
                   armature=0.0005, inertia=(2.5e-5, 2.5e-5, 1e-6))


_wf, _wc = _wheel_frame()
WHEEL = m.mechanism("wheel", tuple(WC), [_stem_link()], LOOKS, frame_parts=_wf, frame_colliders=_wc, friction=(0.5, 0.4))

# -- the cover --------------------------------------------------------------------------------------------------------------------------

TOP_LINE = [(r * S if r < 0.17 else r + (R_COVER - 0.180), z) for r, z in
            [(0.180, 0.006), (0.174, 0.015), (0.162, 0.023), (0.145, 0.029), (0.125, 0.033), (0.100, 0.037), (0.050, 0.041), (0.0, 0.042)]]


def _top(r):
    """The cover's top surface height above its lip's underside at radius r."""
    rs, zs = zip(*TOP_LINE[::-1])
    return float(np.interp(r, rs, zs))


def _band(r0, r1):
    """The dish's closed (r, z) section from r0 to r1: flat underside at 4 mm, the top line above."""
    mid = [p for p in TOP_LINE if r0 < p[0] < r1]
    return [(r0, 0.004), (r1, 0.004), (r1, _top(r1))] + mid + [(r0, _top(r0))]


def _sector_hull(name, r0, r1, a0, a1, outer=False):
    sec = [(R_NOTCH_IN, 0.004), (0.165 * S, 0.0), (R_COVER, 0.0)] + [p for p in TOP_LINE if p[0] >= R_NOTCH_IN] if outer else _band(r0, r1)
    pts = [(r * math.cos(a), r * math.sin(a), z) for a in np.radians(np.linspace(a0, a1, 3)) for r, z in sec]
    return m.col_hull(name, hull(pts))


def _cover_frame():
    outer = [(R_NOTCH_IN, 0.004), (0.165 * S, 0.0), (R_COVER, 0.0)] + [p for p in TOP_LINE if p[0] >= R_NOTCH_IN]
    parts = [("hub", revolve(_band(0.0, SLOT_R0), n=72), "cover"), ("ring", revolve(_band(SLOT_R1, R_NOTCH_IN), n=72), "cover"),
             ("rim", revolve(outer, NOTCH_HALF_DEG, 360.0 - NOTCH_HALF_DEG, n=72), "cover"),
             ("cap", m.cylinder(0.032, 0.003, (0, 0, 0.0425), segments=32), "cover_dark")]
    cols = [m.col_hull("hub_c", revolve(_band(0.0, SLOT_R0), n=24))]
    for i, a in enumerate(SPOKE_ANGLES):  # the spokes between the slots
        lo = SLOT_R0 - 0.004
        hgt = _top(SLOT_R1) - 0.004
        spoke = m.box((SLOT_R1 + 0.004 - lo, SPOKE_W, hgt), ((SLOT_R1 + 0.004 + lo) / 2, 0.0, 0.004 + hgt / 2)).turned(a, "z")
        parts.append((f"spoke_{i}", spoke, "cover"))
        cols.append(m.col_hull(f"spoke_{i}_c", spoke))
    for i in range(16):
        cols.append(_sector_hull(f"ring_{i}_c", SLOT_R1, R_NOTCH_IN, 22.5 * i, 22.5 * (i + 1)))
    n = 35
    span = 360.0 - 2 * NOTCH_HALF_DEG
    for i in range(n):
        cols.append(_sector_hull(f"ann_{i}", 0, 0, NOTCH_HALF_DEG + span * i / n, NOTCH_HALF_DEG + span * (i + 1) / n, outer=True))
    return parts, cols


def clip_axis(theta_deg):
    c, s = math.cos(math.radians(theta_deg)), math.sin(math.radians(theta_deg))
    return ("X", 1.0 if c > 0 else -1.0) if abs(c) > abs(s) else ("Y", 1.0 if s > 0 else -1.0)


def _area(sec):
    return 0.5 * float(np.sum(sec[:, 0] * np.roll(sec[:, 1], -1) - np.roll(sec[:, 0], -1) * sec[:, 1]))


def _clip_link(k, theta):
    u = np.array([math.cos(math.radians(theta)), math.sin(math.radians(theta)), 0.0])
    t = np.array([-u[1], u[0], 0.0])
    w = 0.008

    def pts(rz):
        return np.array([r * u + s * w * t + np.array([0, 0, z]) for r, z in rz for s in (-1, 1)])

    head = pts([(CLIP_OUT, CLIP_TOP), (CLIP_OUT, CLIP_TOP - 0.001), (R_LIP - 0.0045, CLIP_TOP - 0.014), (R_LIP - 0.0085, CLIP_TOP - 0.014), (R_LIP - 0.0085, CLIP_TOP)])
    tab = pts([(R_LIP - 0.0075, CLIP_TOP), (R_LIP - 0.004, CLIP_TOP), (R_LIP - 0.004, -0.001), (R_LIP - 0.0075, -0.001)])
    half = math.degrees(w / R_LIP)

    def solid(p):  # the clip as seen: a closed solid through the same points
        sec = np.array([(np.dot(q, u), q[2]) for q in p[::2]])
        return revolve(sec[::-1] if _area(sec) < 0 else sec, theta - half, theta + half, n=3)

    parts = [(f"clip{k}_head", solid(head), "clip"), (f"clip{k}_tab", solid(tab), "clip")]
    cols = [m.col_hull(f"clip{k}_head_c", hull(head)), m.col_hull(f"clip{k}_tab_c", hull(tab))]
    axis, sgn = clip_axis(theta)
    lo, hi = (-0.008, 0.0) if sgn > 0 else (0.0, 0.008)
    return m.slider(f"clip{k}", axis, lo, hi, parts, cols, mass=0.01, spring=CLIP_K, damping=4.0, friction=0.0, effort=200.0, armature=0.001)


_cf, _cc = _cover_frame()
COVER = m.mechanism("cover", (WC[0], WC[1], TOP + Z_REST + 0.002), [_clip_link(k, a) for k, a in enumerate(CLIP_ANGLES)], LOOKS, frame_parts=_cf,
                    frame_colliders=_cc, yaw_deg=START_YAW, friction=(0.4, 0.35), free_mass=COVER_MASS - 0.08, frame_inertia=(0.0036, 0.0036, 0.0073),
                    solver_iters=48)

# -- distractors (SimReady, loose) -----------------------------------------------------------------------------------------------------
RAG = SimReady("rag", pos=(0.52, 0.30, TOP), size=(0.26, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01", usd="sm_cleaning_cloth_a01_01.usd",
               yaw=math.radians(15.0))
SCREWDRIVER = SimReady("screwdriver", pos=(0.50, -0.23, TOP), size=(0.221, 0.038, 0.036), mass=0.12, package="handTool_screwdriver_slotted_65mm_a01",
                       usd="sm_handTool_screwdriver_slotted_65mm_a01_01.usd", yaw=math.radians(100.0))
GLOVES = SimReady("gloves", pos=(0.64, 0.02, TOP), size=(0.247, 0.231, 0.016), mass=0.06, package="ppe_gloves_lightDuty_a01",
                  usd="sm_ppe_gloves_lightDuty_a01_01.usd", yaw=math.radians(-20.0))

OBJECTS = [WHEEL, COVER, RAG, SCREWDRIVER, GLOVES]

# -- scoring ----------------------------------------------------------------------------------------------------------------------------


def stem_tip(stem_rad: float) -> np.ndarray:
    """The stem's cap (station frame) at hinge angle `stem_rad` (about x through its base)."""
    a = math.radians(STEM_ANGLE)
    x, y = STEM_R * math.cos(a), STEM_R * math.sin(a)
    L = STEM_Z1 - STEM_Z0
    return WC + np.array([x, y - L * math.sin(stem_rad), STEM_Z0 + L * math.cos(stem_rad)])


def clip_heads(cover) -> np.ndarray:
    """Each clip's catching face (outer top corner) at rest, in the station frame."""
    pts = [(CLIP_OUT * math.cos(math.radians(a)), CLIP_OUT * math.sin(math.radians(a)), CLIP_TOP) for a in CLIP_ANGLES]
    return cover.to_world(np.array(pts))


def clip_out(cover, k) -> float:
    """How far clip k sits from its rest, outward positive (<= 0)."""
    _, sgn = clip_axis(CLIP_ANGLES[k])
    return sgn * cover.joint(f"clip{k}")


def cover_gap(cover, tips) -> float:
    q = cover.to_local(tips)
    dr = np.maximum(np.hypot(q[:, 0], q[:, 1]) - R_COVER, 0.0)
    dz = np.maximum(np.maximum(q[:, 2] - 0.043, -q[:, 2]), 0.0)
    return float(np.min(np.hypot(dr, dz)))


def now(env) -> dict:
    cv, wh = m.state(env, "cover"), m.state(env, "wheel")
    stem = wh.joint("stem")
    tip = stem_tip(stem)
    q = cv.to_local(tip[None])[0]
    lateral = abs(q[1]) if q[0] > 0 else 1.0  # off the notch's centre line (the cover's +x)
    in_notch = bool(lateral <= NOTCH_TOL and R_NOTCH_IN <= math.hypot(q[0], q[1]) <= R_COVER)
    heads = clip_heads(cv)
    gaps = heads[:, 2] - (TOP + Z_F + CLIP_TOP)  # lip-to-flange gap beside each clip
    outs = [clip_out(cv, k) for k in range(8)]
    engaged = [bool(gaps[k] <= ENGAGE_GAP and outs[k] >= -SPRUNG) for k in range(8)]
    tips = m.fingertips(env)
    gap = cover_gap(cv, tips)
    return {"stem_deg": math.degrees(stem), "lateral": float(lateral), "in_notch": in_notch, "gaps": gaps.tolist(), "outs": outs,
            "engaged": engaged, "n": int(sum(engaged)), "clear": gap >= CLEAR, "hand_gap": gap,
            "yaw": math.degrees(math.atan2(cv.rotation()[1, 0], cv.rotation()[0, 0])), "z": float(cv.pos[2] - TOP), "tilt": cv.tilt_deg(),
            "xy": [float(cv.pos[0] - WC[0]), float(cv.pos[1] - WC[1])]}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the bent-stem record is cleared
        self.bent = False
        self.hold = 0
        self.full = False
        self.score = 0.0

    def step(self, env):
        c = now(env)
        if abs(c["stem_deg"]) > STEM_BENT_DEG:
            self.bent = True
        ok = c["n"] == 8 and c["in_notch"] and c["clear"] and not self.bent
        self.hold = self.hold + 1 if ok else 0
        self.full = (self.full or self.hold >= HOLD_STEPS) and not self.bent
        if self.bent:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["in_notch"]:
            self.score = 0.2 + 0.075 * c["n"]
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the scripted demo (open loop, from the published starting layout) ------------------------------------------------------------------


def _ring(angle_deg, r, z):
    a = math.radians(angle_deg)
    return (float(WC[0] + r * math.cos(a)), float(WC[1] + r * math.sin(a)), float(TOP + z))


R_GRIP = 0.092  # where the fingers straddle a spoke
GRIP_Z = 0.010  # finger pads this far above the cover's lip plane while gripping a spoke
OPEN_W = -0.019  # finger joint: just wider than a spoke
LIFT = 0.012  # m: the cover lifted clear of the rim while it turns
TURN = 90.0  # degrees: the notch from the robot's side round to the stem
GRIP_AT = 157.5  # world angle of the left arm's spoke at the start (the right arm takes the opposite one)
PRESS_AT = [(112.5, 0.158), (67.5, 0.158), (157.5, 0.158), (22.5, 0.140)]
PRESS_DEPTH = 0.025  # m: pads this far below the seated cover's top surface
ROUNDS = 1


def demo():
    """Both arms straddle two opposite spokes of the cover with open fingers down through its slots, grip them, lift the cover 1.2 cm
    clear of the rim, turn it a quarter turn clockwise (seen from above) until the notch is over the stem, set it back on the rim and
    let go. Then both arms press the cover down over its clips, in mirrored pairs (beside the stem first, then round), twice round, and
    move away."""
    from yam_arena.constants import FINGER_CLOSED as FC
    from yam_arena.demo import Waypoint as W

    lw, rw = [], []
    z_grip = Z_REST + GRIP_Z + 0.013
    z_over = Z_REST + 0.041 + 0.013
    for wl, a0, flip in ((lw, GRIP_AT, 0.0), (rw, GRIP_AT - 180.0, 180.0)):
        y0 = math.radians(((a0 + 90.0) % 180.0) - 90.0 + flip)  # fingers close along the rim's tangent there (either way round)
        wl.append(W(2.0, _ring(a0, R_GRIP, z_over), OPEN_W, "over a spoke", y0))
        wl.append(W(1.2, _ring(a0, R_GRIP, z_grip), OPEN_W, "straddle the spoke", y0))
        wl.append(W(0.8, _ring(a0, R_GRIP, z_grip), FC, "grip", y0))
        wl.append(W(1.0, _ring(a0, R_GRIP, z_grip + LIFT), FC, "lift", y0))
        steps = 8
        for k in range(1, steps + 1):
            d = TURN * k / steps
            wl.append(W(0.6, _ring(a0 - d, R_GRIP, z_grip + LIFT), FC, "turn the cover", y0 - math.radians(d)))
        y1 = y0 - math.radians(TURN)
        wl.append(W(1.2, _ring(a0 - TURN, R_GRIP, z_grip - 0.001), FC, "set it on the rim", y1))
        wl.append(W(0.8, _ring(a0 - TURN, R_GRIP, z_grip - 0.001), OPEN_W, "let go", y1))
        wl.append(W(1.0, _ring(a0 - TURN, R_GRIP, z_over), OPEN_W, "out", y1))
    for _ in range(ROUNDS):
        for a, r in PRESS_AT:
            touch = Z_F + _top(r) + 0.013  # TCP height with the pads on the seated cover
            for wl, sgn in ((lw, 1.0), (rw, -1.0)):
                ang = sgn * a
                wl.append(W(1.0, _ring(ang, r, touch + 0.036), FC, "over a clip", 0.0))
                wl.append(W(1.2, _ring(ang, r, touch - PRESS_DEPTH), FC, "press", 0.0))
                wl.append(W(0.4, _ring(ang, r, touch - PRESS_DEPTH), FC, "hold", 0.0))
                wl.append(W(0.6, _ring(ang, r, touch + 0.036), FC, "lift", 0.0))
    lw.append(W(1.5, _ring(100.0, 0.26, 0.15), FC, "away", 0.0))
    rw.append(W(1.5, _ring(-100.0, 0.26, 0.15), FC, "away", 0.0))
    lw.append(W(3.0, _ring(100.0, 0.26, 0.15), FC, "wait", 0.0))
    rw.append(W(3.0, _ring(-100.0, 0.26, 0.15), FC, "wait", 0.0))
    return {"left": lw, "right": rw}
