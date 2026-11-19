"""Refit an Instant Pot lid's float valve cap after washing.

The washed lid of a 6-quart electric pressure cooker stands on edge in a dish rack, top face toward the left arm. Its
stainless float valve has been dropped back into its hole from the top and is loose: nothing holds it in. The small
silicone cap that keeps it in the lid (and seals it when it rises under pressure) lies on the counter beside the rack.
The cap has to be pushed onto the valve's end on the underside until its lip snaps into the valve's groove. Pushed on
alone, the cap just shoves the loose valve back out of its hole, so the valve has to be held in from the top while
the cap goes on (the maker's quick start guide: "Place one finger on the flat top of float valve ... Attach silicone
cap to bottom of float valve").

Frames: world x forward from the arms, y left, z up, metres. The lid's mechanism frame sits on the valve hole's axis at
the lid's mid-plane, unrotated: +y is out of the lid's top face.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from scipy.spatial import ConvexHull  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the silicone cap back on the bottom of the pressure cooker lid's float valve."
SUCCESS_HOLD_S = 3.0  # after full credit (the scorer counts its own 1 s hold), so the video shows the arms leaving
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- dimensions (m) -------------------------------------------------------------------------------------------------------
VALVE = np.array([0.25, 0.0, TOP + 0.150])  # the valve hole's axis at the lid's mid-plane
LID_C = np.array([0.06, 0.0, 0.0])  # the lid's centre, in the lid frame
LID_R = 0.125  # lid radius (250 mm across)
PLATE_T = 0.006  # half the lid's thickness round the valve (12 mm: steel inner lid and plastic cover)
HEAD_R, HEAD_T = 0.0050, 0.0025  # the valve's flat top: 10 mm across, 2.5 mm thick
SHAFT_R = 0.0025  # 5 mm shank in a 6 mm hole
SHOULDER = -0.0090  # the shank's end below the lid (3 mm under the underside), at travel 0
GROOVE_R, GROOVE_L = 0.0017, 0.0012  # groove core 3.4 mm across, 1.2 mm long
KNOB_L = 0.0018  # the end knob past the groove: 5.4 mm across, chamfered to 3.2 mm at the tip
KNOB_R, KNOB_TIP_R, CORE_R = 0.0027, 0.0016, 0.0013
BACK = SHOULDER - GROOVE_L  # the knob's back face
TIP = BACK - KNOB_L  # the valve's end
TRAVEL = 0.012  # simulated outward travel (stops the shank in the hole); past FELL_OUT it would drop out
FELL_OUT = 0.010
BARB_K = 150.0  # N/m per quarter: the cap's lip stretching over the knob, a few newtons to push on
BARB_IN = 0.0020  # how far each quarter may give

CAP_R, CAP_L = 0.0060, 0.0065  # cap: 12 mm across, 6.5 mm long
LIP_R, LIP_T, MOUTH_R = 0.0018, 0.0010, 0.0024  # its lip: 3.6 mm hole, 1 mm thick, mouth chamfered to 4.8 mm
BORE_R, BORE_D = 0.0030, 0.0045  # bore behind the lip, to the closed end
CAP_MASS = 0.002
CAP0 = np.array([0.20, -0.12, TOP])  # lying open side down on the counter

LOOKS = {
    "cover": m.Look(color=m.rgb(0.80, 0.81, 0.82), metallic=0.6, roughness=0.35),
    "steel": m.Look(color=m.rgb(0.72, 0.73, 0.75), metallic=0.9, roughness=0.25),
    "black": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.5),
    "seal": m.Look(color=m.rgb(0.86, 0.86, 0.84), roughness=0.7),
    "dark": m.Look(color=m.rgb(0.15, 0.15, 0.16), roughness=0.6),
}
CAP_LOOKS = {"silicone": m.Look(color=m.rgb(0.93, 0.93, 0.90), roughness=0.8)}
RACK_LOOKS = {"wire": m.Look(color=m.rgb(0.92, 0.92, 0.93), roughness=0.5), "tray": m.Look(color=m.rgb(0.75, 0.80, 0.85), roughness=0.6)}


def _hull(points) -> m.Mesh:
    p = np.asarray(points, float)
    h = ConvexHull(p)
    faces = []
    c = p.mean(axis=0)
    for s in h.simplices:
        a, b, d = p[s]
        n = np.cross(b - a, d - a)
        faces.append(list(s) if np.dot(n, a - c) > 0 else [s[0], s[2], s[1]])
    return m.Mesh(p, np.array(faces))


def _ring_y(r0, r1, y0, y1, segments=32) -> m.Mesh:
    """An annulus about the lid frame's y axis, from y0 to y1."""
    ring = m.lathe([(r0, y0), (r1, y0), (r1, y1), (r0, y1), (r0, y0)], segments)
    return ring.turned(-90.0, "x")


def _sector_points(r_by_level, a0, a1, axis="y", n=5):
    """Points of an annular sector (angles a0..a1, degrees) at several levels (along `axis`), each (level, r_in, r_out)."""
    pts = []
    for a in np.linspace(math.radians(a0), math.radians(a1), n):
        for lev, ri, ro in r_by_level:
            for r in (ri, ro):
                if axis == "y":
                    pts.append((r * math.cos(a), lev, r * math.sin(a)))
                else:
                    pts.append((r * math.cos(a), r * math.sin(a), lev))
    return pts


# -- the lid, the valve and the cap's grip on it --------------------------------------------------------------------------

def _lid() -> m.Part:
    cx = LID_C[0]
    frame_parts = [
        ("cover", m.cylinder(LID_R - 0.007, 0.008, (cx, 0.002, 0.0), axis="y", segments=64), "cover"),
        ("inner", m.cylinder(LID_R - 0.013, 0.004, (cx, -0.004, 0.0), axis="y", segments=64), "steel"),
        ("rim", _ring_y(LID_R - 0.012, LID_R, -0.020, 0.005, 64).moved((cx, 0.0, 0.0)), "cover"),
        ("seal", _ring_y(0.098, 0.106, -0.0135, -0.006, 64).moved((cx, 0.0, 0.0)), "seal"),
        ("handle", m.box((0.034, 0.028, 0.20), (cx, PLATE_T + 0.014, 0.0)), "black"),
        ("release", m.cylinder(0.012, 0.018, (cx + 0.045, PLATE_T + 0.009, 0.045), axis="y"), "black"),
        ("shield", m.cylinder(0.011, 0.010, (cx + 0.045, -PLATE_T - 0.005, 0.045), axis="y"), "steel"),
        ("pin_lock", m.cylinder(0.004, 0.010, (cx - 0.02, PLATE_T + 0.005, 0.085), axis="y"), "dark"),
        ("hole_top", m.cylinder(0.0062, 0.0004, (0.0, PLATE_T + 0.0002, 0.0), axis="y"), "dark"),
        ("hole_bot", m.cylinder(0.0035, 0.0004, (0.0, -PLATE_T - 0.0002, 0.0), axis="y"), "dark"),
    ]
    frame_cols = [
        m.col_hull("plate_c", m.cylinder(LID_R - 0.007, 2 * PLATE_T, (cx, 0.0, 0.0), axis="y", segments=32)),
        m.col_box("handle_c", (0.034, 0.028, 0.20), (cx, PLATE_T + 0.014, 0.0)),
        m.col_hull("release_c", m.cylinder(0.012, 0.018, (cx + 0.045, PLATE_T + 0.009, 0.045), axis="y", segments=16)),
        m.col_hull("shield_c", m.cylinder(0.011, 0.010, (cx + 0.045, -PLATE_T - 0.005, 0.045), axis="y", segments=16)),
    ]
    for k in range(16):  # the rim: sixteen convex pieces
        pts = _sector_points([(-0.020, LID_R - 0.012, LID_R), (0.005, LID_R - 0.012, LID_R)], 22.5 * k, 22.5 * (k + 1), "y", 3)
        frame_cols.append(m.col_hull(f"rim{k}_c", _hull(np.asarray(pts) + LID_C)))

    shank = m.cylinder(SHAFT_R, PLATE_T - SHOULDER, (0.0, (PLATE_T + SHOULDER) / 2, 0.0), axis="y", segments=24)
    head = m.cylinder(HEAD_R, HEAD_T, (0.0, PLATE_T + HEAD_T / 2, 0.0), axis="y", segments=32)
    groove = m.cylinder(GROOVE_R, GROOVE_L, (0.0, (SHOULDER + BACK) / 2, 0.0), axis="y", segments=24)
    core = m.cylinder(CORE_R, KNOB_L, (0.0, (BACK + TIP) / 2, 0.0), axis="y", segments=24)
    valve = m.slider("valve", "Y", 0.0, TRAVEL, [("head", head, "steel"), ("shank", shank, "steel"), ("groove", groove, "steel"), ("core", core, "steel")],
                     [m.col_hull("head_c", head), m.col_hull("shank_c", shank), m.col_hull("groove_c", groove)],  # the core is seen, not collided: the four quarters are the knob
                     mass=0.010, friction=0.05, damping=0.2)
    links = [valve]
    # the knob past the groove, in four quarters that give inward: the cap's lip stretching over it, as silicone does
    for name, ang, axis, sgn in (("barb_px", 0.0, "X", 1.0), ("barb_pz", 90.0, "Z", 1.0), ("barb_nx", 180.0, "X", -1.0), ("barb_nz", 270.0, "Z", -1.0)):
        pts = _sector_points([(BACK, CORE_R, KNOB_R), (TIP, CORE_R, KNOB_TIP_R)], ang - 40.0, ang + 40.0, "y", 5)
        # in the lid frame, the "z" of _sector_points' angle is world z: angle 0 is +x, 90 is +z
        mesh = _hull(pts)
        lo, hi = (-BARB_IN, 0.0) if sgn > 0 else (0.0, BARB_IN)
        links.append(m.slider(name, axis, lo, hi, [(name, mesh, "steel")], [m.col_hull(f"{name}_c", mesh)], parent="valve",
                              mass=0.004, spring=BARB_K, damping=2.0, friction=0.0, effort=50.0))
    return m.mechanism("lid", tuple(VALVE), links, LOOKS, frame_parts, frame_cols, friction=(0.15, 0.1), solver_iters=64)


def _cap() -> m.Part:
    """The silicone cap; its origin is the centre of its open face, +z into it toward its closed end."""
    vis = m.lathe([(MOUTH_R, 0.0), (CAP_R - 0.0005, 0.0), (CAP_R, 0.0005), (CAP_R, CAP_L - 0.001), (CAP_R - 0.001, CAP_L), (0.0, CAP_L)], 32)
    cols = []
    n = 12
    for k in range(n):
        a0, a1 = 360.0 * k / n, 360.0 * (k + 1) / n
        lip = _sector_points([(0.0, MOUTH_R, CAP_R), (LIP_T * 0.5, LIP_R, CAP_R), (LIP_T, LIP_R, CAP_R)], a0, a1, "z", 3)
        cols.append(m.col_hull(f"lip{k}_c", _hull(lip)))
        wall = _sector_points([(LIP_T, BORE_R, CAP_R), (BORE_D, BORE_R, CAP_R)], a0, a1, "z", 3)
        cols.append(m.col_hull(f"wall{k}_c", _hull(wall)))
    cols.append(m.col_cylinder("end_c", CAP_R, CAP_L - BORE_D, (0.0, 0.0, (CAP_L + BORE_D) / 2)))
    return m.free("cap", tuple(CAP0), [("cap", vis, "silicone")], cols, CAP_LOOKS, CAP_MASS,
                  friction=(0.9, 0.8), torsional_patch=0.010, solver_iters=64, com=(0.0, 0.0, 0.0038))


def _rack() -> m.Part:
    """The dish rack's lid slot: a drip tray, a bar the lid's edge stands on and two rows of plastic-coated tines."""
    parts, cols = [], []
    lx = VALVE[0] + LID_C[0]
    tray = ((0.30, 0.13, 0.012), (lx, -0.006, 0.006))
    parts.append(("tray", m.box(*tray), "tray"))
    cols.append(m.col_box("tray_c", *tray))
    bar = ((0.22, 0.03, 0.013), (lx, -0.0075, 0.0185))
    parts.append(("bar", m.box(*bar), "wire"))
    cols.append(m.col_box("bar_c", *bar))
    for j, y in enumerate((-0.026, 0.011)):
        for i, x in enumerate(np.linspace(lx - 0.09, lx + 0.09, 5)):
            c = m.cylinder(0.0025, 0.07, (x, y, 0.047), segments=12)
            parts.append((f"tine{j}{i}", c, "wire"))
            cols.append(m.col_hull(f"tine{j}{i}_c", c))
        rail = m.cylinder(0.0025, 0.19, (lx, y, 0.082), axis="x", segments=12)
        parts.append((f"rail{j}", rail, "wire"))
        cols.append(m.col_hull(f"rail{j}_c", rail))
    return m.fixed("rack", (0.0, 0.0, TOP), parts, cols, RACK_LOOKS, friction=(0.6, 0.5))


LID, CAP, RACK = _lid(), _cap(), _rack()
SOAP = SimReady("dish_soap", pos=(0.47, 0.22, TOP), size=(0.066, 0.114, 0.271), mass=0.40, package="cleaning_dishSoap_a01", usd="sm_cleaning_dishSoap_a01_01.usd", yaw=90.0)
MUG = SimReady("mug", pos=(0.47, -0.21, TOP), size=(0.114, 0.090, 0.080), mass=0.32, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=30.0)
SPONGE = SimReady("sponge", pos=(0.13, 0.23, TOP), size=(0.071, 0.101, 0.033), mass=0.02, package="cleaning_sponge_b01", usd="sm_cleaning_sponge_b01_01.usd", yaw=10.0)
OBJECTS = [LID, CAP, RACK, SOAP, MUG, SPONGE]


# -- the scorer -----------------------------------------------------------------------------------------------------------

def fit(lid, cap) -> dict:
    """Where the cap is on the valve. `seated`: its lip in the groove (open face within 0.6 mm behind and 0.4 mm past the shank's end), all four
    knob quarters back out (under 0.3 mm in), axis within 10 deg of the valve's and within 0.6 mm of it."""
    s = lid.joint("valve")
    rel = cap.pos - VALVE  # the lid frame is unrotated
    shoulder = SHOULDER + s
    axis = cap.rotation()[:, 2]
    ang = math.degrees(math.acos(float(np.clip(-axis[1], -1.0, 1.0))))
    radial = float(math.hypot(rel[0], rel[2]))
    depth = float(rel[1] - shoulder)  # 0: the open face against the shank's end; negative: short of it
    out = max(abs(lid.joint(b)) for b in ("barb_px", "barb_pz", "barb_nx", "barb_nz"))
    seated = -0.0006 <= depth <= 0.0004 and out < 0.0003 and ang <= 10.0 and radial <= 0.0006
    started = -0.0030 <= depth <= 0.0004 and ang <= 30.0 and radial <= 0.0015  # on the valve's end, the lip at least over the knob's tip
    return {"seated": bool(seated), "started": bool(started), "travel": float(s), "depth": depth, "ang": ang, "radial": radial, "barb": float(out)}


def hands_gap(env, lid, cap) -> float:
    tips = m.fingertips(env)
    head = VALVE + np.array([0.0, PLATE_T + HEAD_T + lid.joint("valve"), 0.0])
    centre = cap.pos + cap.rotation() @ np.array([0.0, 0.0, CAP_L / 2])
    return float(min(np.linalg.norm(tips - head, axis=1).min(), np.linalg.norm(tips - centre, axis=1).min()))


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.fell = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}

    def step(self, env):
        lid, cap = m.state(env, "lid"), m.state(env, "cap")
        f = fit(lid, cap)
        if not self.full and f["travel"] > FELL_OUT:
            self.fell = True  # pushed out of its hole: a real valve drops out here
        gap = hands_gap(env, lid, cap)
        still = float(np.linalg.norm(cap.lin_vel)) < 0.01
        ok = f["seated"] and gap >= 0.02 and still
        self.hold = self.hold + 1 if ok else 0
        if not self.full and not self.fell and self.hold >= HOLD_STEPS:
            self.full = True
        if self.full:
            self.score = 1.0
        elif self.fell:
            self.score = 0.0
        else:  # this step only: the cap left on the valve's end, not snapped home
            self.score = 0.25 if (f["started"] and gap >= 0.02 and still) else 0.0
        self.last = dict(f, gap=gap, still=still, fell=self.fell, score=self.score)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the scripted demonstration (a fixed plan from the published starting layout) ------------------------------------------

PAD = 0.013  # pad tips past the TCP
GRIP_DEPTH = 0.0025  # the cap's open face past the pad tips once picked (pads 2.5 mm above the counter)
CAP_TRIM = np.array([0.0, 0.0, 0.0])  # measured: where the held cap's open face sits from its planned place


def _plan(mode: str = "full"):
    import demokit as d
    from yam_arena.demo import Waypoint

    def w(t, pos, finger, label, grip):
        return Waypoint(t, tuple(float(v) for v in pos), float(finger), label, grip)

    gd, gr, gl = d.down(90.0), d.level(90.0), d.level(-90.0)
    SHUT, OPEN = 0.0, -0.012
    PINCH = -(CAP_R - 0.0025)  # each finger commanded 2.5 mm into the 12 mm cap: a firm pinch
    y_face = lambda yf: VALVE + np.array([0.0, yf - GRIP_DEPTH - PAD, 0.0]) - CAP_TRIM  # right TCP for the cap's face at lid-frame y
    pick = CAP0 + np.array([0.0, 0.0, PAD + GRIP_DEPTH])
    right = [
        w(2.0, pick + [0, 0, 0.06], OPEN, "above the cap", gd),
        w(1.5, pick, OPEN, "fingers either side of the cap", gd),
        w(1.0, pick, PINCH, "pinch the cap", gd),
        w(0.5, pick, PINCH, "settle the grip", gd),
        w(1.5, pick + [0, 0, 0.06], PINCH, "lift it", gd),
        w(2.5, (VALVE[0], -0.11, VALVE[2]), PINCH, "turn it open side to the lid, below the valve", gr),
        w(1.5, y_face(-0.030), PINCH, "in line with the valve", gr),
        w(1.0, y_face(-0.030), PINCH, "settle", gr),
        w(1.5, y_face(TIP - 0.0005), PINCH, "up to the valve's end", gr),
        w(2.5, y_face(SHOULDER + 0.010), PINCH, "push the cap on over the knob", gr),
        w(0.8, y_face(SHOULDER + 0.010), PINCH, "hold", gr),
        w(0.3, y_face(SHOULDER - 0.0005), PINCH, "ease off the push", gr),
        w(0.6, y_face(SHOULDER - 0.0005), OPEN, "let go", gr),
        w(1.2, y_face(-0.060), OPEN, "back off", gr),
        w(2.0, (0.20, -0.25, TOP + 0.22), OPEN, "right hand clear", gd),
        w(6.0, (0.20, -0.25, TOP + 0.22), OPEN, "wait", gd),
    ]
    press = VALVE + np.array([0.0, PLATE_T + HEAD_T + PAD - 0.006, 0.0])  # tips 6 mm into the valve's top: the arm's stiffness holds it in
    t_push_end = sum(x.duration for x in right[:10])
    left = [
        w(3.0, press + [0, 0.05, 0], SHUT, "in front of the lid's top face", gl),
        w(1.5, press + [0, 0.01, 0], SHUT, "up to the valve", gl),
        w(1.0, press, SHUT, "finger on the valve's flat top", gl),
    ]
    held = sum(x.duration for x in left)
    left += [
        w(max(t_push_end + 1.4 - held, 0.5), press, SHUT, "hold the valve in", gl),
        w(1.0, press + [0, 0.05, 0], SHUT, "off the valve", gl),
        w(2.0, (0.20, 0.25, TOP + 0.22), SHUT, "left hand clear", d.down(0.0)),
        w(4.0, (0.20, 0.25, TOP + 0.22), SHUT, "wait", d.down(0.0)),
    ]
    if mode == "none":
        return {"left": [w(20.0, (0.20, 0.25, TOP + 0.22), SHUT, "wait", d.down(0.0))], "right": [w(20.0, (0.20, -0.25, TOP + 0.22), OPEN, "wait", d.down(0.0))]}
    if mode == "held":  # pushes the cap on but never lets go of it
        right = right[:10] + [w(14.0, y_face(SHOULDER - 0.0005), PINCH, "keep holding", gr)]
    if mode == "onearm":
        left = [w(20.0, (0.20, 0.25, TOP + 0.22), SHUT, "wait", d.down(0.0))]
    return {"left": left, "right": right}


def demo():
    return _plan("full")
