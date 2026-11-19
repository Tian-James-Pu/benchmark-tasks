"""On the station table, right of centre: a Feetech STS3215 serial bus servo (black case 45.2 x 24.7 x 35 mm) stands
upright in a printed bench holder with its 25-tooth output spline (5.9 mm across, 3.4 mm proud of the case top) at
(0.300, -0.080), the case's long side along x. The servo is powered at its middle position (2048), this joint's zero.
A blue aluminium single-arm 25T servo horn (hub 15 mm across and 6 mm tall, arm 7.5 mm wide, about 28 mm from the
axis to its tip, about 2 g) lies hub-down on the table at (0.215, -0.215), its arm pointing 30 deg left of +x.
Distractors, not scored: a mug, a roll of clear tape and a Phillips screwdriver.

The horn slides onto the spline only with its teeth between the shaft's: at 25 angles 14.4 deg apart. Anywhere else it
sits on the tooth tips, 2.7 mm proud. Pushed home it stays on the tooth it went on at. The goal is the tooth that puts the
arm as near as the spline allows to pointing straight left (+y), square to the case: here 2 deg past square.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 1e-3

# -- the servo: Feetech STS3215 (vendor specification: 45.2 x 24.7 x 35 mm, 55 g, output 25T / OD 5.9 mm) ------------
CASE = (45.2 * MM, 24.7 * MM, 35.0 * MM)
SHAFT_FROM_END = 12.5 * MM  # spline axis from the case's +x end (vendor drawing)
SPLINE_PROUD = 3.4 * MM  # spline above the case top (vendor drawing)
SPLINE_R = 5.9 * MM / 2
ROOT_R = 2.6 * MM  # tooth root radius: drawn only
HOLDER_FLOOR = 5.0 * MM
CASE_TOP = TOP + HOLDER_FLOOR + CASE[2]
SHAFT = np.array([0.300, -0.080, CASE_TOP])  # the spline's axis on the case top: the servo part's origin
CASE_CX = -(CASE[0] / 2 - SHAFT_FROM_END)  # case centre from the axis, along x

TEETH = 25
PITCH = 360.0 / TEETH  # 14.4 deg
SQUARE = 90.0  # the zero: arm pointing straight left, square to the case's long side
PHASE = 2.0  # the shaft's tooth phase at 2048: the nearest tooth is this far past square
A0 = SQUARE + PHASE  # arm angle (deg, from +x, anticlockwise from above) on the correct tooth
SEAT = 0.4 * MM  # hub bottom above the case top when home
CREST = 3.1 * MM  # hub bottom above the case top when it rests on the tooth tips
STANDOFF = CREST - SEAT  # 2.7 mm
SEAT_FORCE = 10.0  # N to push the horn along the spline (estimate: "light finger pressure")
LOCK_K = 50000.0  # N/m: the tooth tips, and the press fit's elastic give
CAPTURE_DEG = 3.0  # the chamfered tooth tips let the horn in within this of a tooth
CAPTURE_OFFSET = 0.5 * MM  # bore over the shaft within this
CAPTURE_TILT = 4.0
ENGAGED = 0.5 * MM  # pushed this far onto the spline, the teeth are in mesh
STRIP_DEG = 3.0  # a horn in mesh turned further than this has sheared teeth

# -- the horn: 25T single-arm aluminium horn (vendor drawing: hub 15 mm, arm 7.5 mm, holes 19 and 24 mm, about 2 g) ---
HUB_R = 7.5 * MM
HUB_H = 6.0 * MM
BORE_R = 3.1 * MM  # over the 5.9 mm spline: 0.15 mm clearance
ARM_W = 7.5 * MM
ARM_T = 3.5 * MM
ARM_HOLE = (19.0 * MM, 24.0 * MM)
ARM_TIP = ARM_HOLE[1] + ARM_W / 2  # 27.75 mm from the axis
HORN_MASS = 0.002
HORN0 = np.array([0.215, -0.215, TOP])
HORN0_YAW = 30.0

LOOKS = {
    "case": m.Look(m.rgb(0.05, 0.05, 0.055), roughness=0.55),
    "label": m.Look(m.rgb(0.75, 0.75, 0.72), roughness=0.7),
    "brass": m.Look(m.rgb(0.78, 0.62, 0.36), metallic=0.8, roughness=0.35),
    "dark": m.Look(m.rgb(0.02, 0.02, 0.02), roughness=0.8),
    "holder": m.Look(m.rgb(0.85, 0.85, 0.82), roughness=0.75),
    "blue": m.Look(m.rgb(0.10, 0.42, 0.85), metallic=0.6, roughness=0.35),
}


def _star(r_out, r_in, n, z0, z1, phase_deg=0.0):
    pts = []
    for i in range(n):
        base = math.radians(phase_deg + i * 360.0 / n)
        step = math.radians(360.0 / n)
        for frac, r in ((-0.22, r_out), (0.22, r_out), (0.28, r_in), (0.72, r_in)):
            a = base + frac * step
            pts.append((r * math.cos(a), r * math.sin(a)))
    return m.prism(pts, z0, z1)


def _servo():
    hx = CASE_CX
    hold_o = (CASE[0] + 10 * MM, CASE[1] + 10 * MM)
    zf = -CASE[2] - HOLDER_FLOOR  # holder floor bottom (= table) in the servo frame
    wall_h = 15 * MM
    holder = [
        ("floor", (hold_o[0], hold_o[1], HOLDER_FLOOR), (hx, 0, zf + HOLDER_FLOOR / 2)),
        ("wall_px", (5 * MM, hold_o[1], wall_h), (hx + CASE[0] / 2 + 2.5 * MM, 0, -CASE[2] + wall_h / 2)),
        ("wall_nx", (5 * MM, hold_o[1], wall_h), (hx - CASE[0] / 2 - 2.5 * MM, 0, -CASE[2] + wall_h / 2)),
        ("wall_py", (CASE[0], 5 * MM, wall_h), (hx, CASE[1] / 2 + 2.5 * MM, -CASE[2] + wall_h / 2)),
        ("wall_ny", (CASE[0], 5 * MM, wall_h), (hx, -CASE[1] / 2 - 2.5 * MM, -CASE[2] + wall_h / 2)),
    ]
    parts = [("case", m.box(CASE, (hx, 0, -CASE[2] / 2)), "case"),
             ("label", m.box((26 * MM, 0.3 * MM, 14 * MM), (hx - 4 * MM, -CASE[1] / 2 - 0.15 * MM, -CASE[2] / 2)), "label"),
             ("spline", _star(SPLINE_R, ROOT_R, TEETH, 0.0, SPLINE_PROUD, PHASE), "brass"),
             ("screw_hole", m.cylinder(1.3 * MM, 0.1 * MM, (0, 0, SPLINE_PROUD + 0.05 * MM)), "dark")]
    cols = [m.col_box("case_c", CASE, (hx, 0, -CASE[2] / 2)),
            m.col_cylinder("spline_c", SPLINE_R, 3.0 * MM, (0, 0, 1.5 * MM)),
            m.col_hull("spline_tip_c", m.lathe([(0.0, 3.0 * MM), (SPLINE_R, 3.0 * MM), (SPLINE_R - 0.5 * MM, SPLINE_PROUD), (0.0, SPLINE_PROUD)]))]
    for n, size, c in holder:
        parts.append((n, m.box(size, c), "holder"))
        cols.append(m.col_box(n + "_c", size, c))
    # The tooth tips and the press fit, as one hidden ring under the hub: it holds the horn 2.7 mm proud until the horn's
    # teeth line up with the shaft's (Trial.step), then gives way only to a push of SEAT_FORCE, down to home.
    seat = m.slider("seat", "Z", -STANDOFF, 0.0, (), (m.col_cylinder("seat_c", 7.0 * MM, 1.0 * MM, (0, 0, CREST - 0.5 * MM)),),
                    mass=0.05, damping=50.0, friction=0.0, effort=2000.0)
    return m.mechanism("servo", tuple(SHAFT), [seat], LOOKS, frame_parts=parts, frame_colliders=cols)


def _horn():
    seg = 16
    half = math.pi / seg
    r_in = BORE_R / math.cos(half)
    r_out = HUB_R
    cols = []
    for i in range(seg):
        pts = []
        for a in (2 * half * i - half, 2 * half * i + half):
            ca, sa = math.cos(a), math.sin(a)
            for r, z in ((r_in, HUB_H), (r_in, 0.3 * MM), (r_in + 0.3 * MM, 0.0), (r_out, 0.0), (r_out, HUB_H)):
                pts.append((r * ca, r * sa, z))
        pts = np.array(pts)
        cols.append(m.col_hull(f"hub{i}_c", m.Mesh(pts, ConvexHull(pts).simplices)))
    arm_len = ARM_TIP - 6.5 * MM
    cols.append(m.col_box("arm_c", (arm_len, ARM_W, ARM_T), (6.5 * MM + arm_len / 2, 0, HUB_H - ARM_T / 2)))
    # what is seen
    hub = m.lathe([(BORE_R, 0.0), (HUB_R, 0.0), (HUB_R, HUB_H), (BORE_R, HUB_H), (BORE_R, 0.0)], 40)
    outline = [(6.0 * MM, -ARM_W / 2)]
    outline += [(ARM_HOLE[1] + ARM_W / 2 * math.cos(t), ARM_W / 2 * math.sin(t)) for t in np.linspace(-math.pi / 2, math.pi / 2, 13)]
    outline += [(6.0 * MM, ARM_W / 2)]
    arm = m.prism(outline, HUB_H - ARM_T, HUB_H)
    parts = [("hub", hub, "blue"), ("arm", arm, "blue"),
             ("bore", _star(BORE_R, BORE_R - 0.3 * MM, TEETH, 0.5 * MM, HUB_H - 0.5 * MM), "dark"),
             ("slot", m.box((HUB_R - BORE_R, 0.8 * MM, 0.2 * MM), ((HUB_R + BORE_R) / 2 + 0.1 * MM, 0, HUB_H)), "dark")]
    for k, x in enumerate(ARM_HOLE):
        parts.append((f"hole{k}", m.cylinder(1.5 * MM, 0.2 * MM, (x, 0, HUB_H)), "dark"))
    return m.free("horn", tuple(HORN0), parts, cols, LOOKS, mass=HORN_MASS, friction=(0.9, 0.8), torsional_patch=0.004, yaw_deg=HORN0_YAW)


SERVO = _servo()
HORN = _horn()

OBJECTS = [
    SERVO,
    HORN,
    SimReady("mug", pos=(0.17, 0.21, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=math.pi,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("tape_roll", pos=(0.44, 0.17, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
             package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd"),
    SimReady("screwdriver", pos=(0.47, -0.22, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=math.pi / 2, convex=True,
             package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd"),
]


# -- the spline and the scorer ---------------------------------------------------------------------------------------


def _wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def measure(horn) -> dict:
    """The horn against the spline: bore offset (m), hub bottom above home (m), tilt and arm angle (deg), nearest tooth."""
    rot = horn.rotation()
    a = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
    k = int(round(_wrap(a - A0) / PITCH))
    k = (k + 12) % TEETH - 12  # -12 .. 12
    return {"offset": float(np.linalg.norm(horn.pos[:2] - SHAFT[:2])), "h": float(horn.pos[2] - (CASE_TOP + SEAT)),
            "tilt": horn.tilt_deg(), "arm": a, "k": k, "d": _wrap(a - A0 - k * PITCH)}


def clearance(horn, tips) -> float:
    """The nearest fingertip point's distance (m) from the horn's hub and arm."""
    p = horn.to_local(tips)
    radial = np.maximum(np.linalg.norm(p[:, :2], axis=1) - HUB_R, 0.0)
    axial = np.maximum(np.maximum(p[:, 2] - HUB_H, -p[:, 2]), 0.0)
    d_hub = np.hypot(radial, axial)
    c = np.array([(6.5 * MM + ARM_TIP) / 2, 0.0, HUB_H - ARM_T / 2])
    half = np.array([(ARM_TIP - 6.5 * MM) / 2, ARM_W / 2, ARM_T / 2])
    d_arm = np.linalg.norm(np.maximum(np.abs(p - c) - half, 0.0), axis=1)
    return float(np.min(np.minimum(d_hub, d_arm)))


HOLD_STEPS = 40  # 2 s at 20 Hz


class Trial:
    """The spline's state and the trial's record. `reset` is the only place the record is cleared."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.target = 0.0  # the hidden ring's drive target (m; 0 = tooth tips, -STANDOFF = home)
        self.mesh = False  # the horn's teeth are in the shaft's
        self.tooth = None  # which tooth they went in at (0 = the correct one)
        self.stripped = False  # a horn in mesh was turned on the spline
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        if env is not None:
            m.lock(env, "servo", "seat", stiffness=LOCK_K, effort=2000.0)
            m.set_target(env, "servo", "seat", 0.0)

    def step(self, env):
        hs, q = m.state(env, "horn"), m.state(env, "servo").joint("seat")
        g = measure(hs)
        near = g["offset"] < 9 * MM and g["h"] < STANDOFF + 4 * MM
        aligned = abs(g["d"]) <= CAPTURE_DEG and g["offset"] <= CAPTURE_OFFSET and g["tilt"] <= CAPTURE_TILT and g["h"] <= STANDOFF + 0.8 * MM
        if self.mesh and (g["h"] > STANDOFF - 0.3 * MM or not near):
            self.mesh, self.tooth = False, None  # pulled off the spline
        if self.mesh and abs(_wrap(g["arm"] - A0 - self.tooth * PITCH)) > STRIP_DEG:
            self.stripped = True
        if near:  # lifted, the ring follows the hub back up; it never follows it down
            self.target = max(self.target, min(g["h"] - STANDOFF, 0.0))
        else:
            self.target = 0.0
        if self.mesh or aligned:  # the press fit: it gives way only to SEAT_FORCE
            if LOCK_K * (self.target - q) > SEAT_FORCE:
                self.target = q + SEAT_FORCE / LOCK_K
            self.target = max(self.target, -STANDOFF)
            if not self.mesh and -q >= ENGAGED:
                self.mesh, self.tooth = True, g["k"]
        m.set_target(env, "servo", "seat", self.target)

        home = self.mesh and g["h"] <= 0.3 * MM and g["tilt"] <= 2.0
        correct = self.mesh and self.tooth == 0
        started = (correct and not home) or (not self.mesh and aligned and g["k"] == 0)
        clear = clearance(hs, m.fingertips(env))
        still = np.linalg.norm(hs.lin_vel) < 0.005 and np.linalg.norm(hs.ang_vel) < math.radians(5.0)
        ok = home and correct and clear >= 0.02 and still and not self.stripped
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.stripped:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif home and correct:
            self.score = 0.5
        elif started:
            self.score = 0.25
        else:
            self.score = 0.0
        self.last = dict(g, q=q, target=self.target, mesh=self.mesh, tooth=self.tooth, aligned=aligned, home=home, clear=clear,
                         still=bool(still), stripped=self.stripped, score=self.score)
        if os.environ.get("HORN_DEBUG"):
            self._n = getattr(self, "_n", 0) + 1
            if self._n % int(os.environ["HORN_DEBUG"]) == 0:
                tp = m.fingertips(env)[12:]
                print("HORNDBG", self._n, " ".join(f"{k}={v:.5f}" if isinstance(v, float) else f"{k}={v}" for k, v in self.last.items()),
                      "pos", np.round(hs.pos, 5).tolist(), "tips", np.round(tp.mean(0), 4).tolist(), "sep", np.round(tp[:6].mean(0) - tp[6:].mean(0), 4).tolist(),
                      "tipz", round(float(tp[:, 2].min()), 4), flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)
SUCCESS_HOLD_S = 0.0  # the trial counts its own 2 s hold


def success(objects) -> bool:
    return bool(TRIAL.full and not TRIAL.stripped)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the scripted demo: a fixed plan from the published starting layout ------------------------------------------------

GRIP_Z = 0.0145  # TCP above the hub's bottom when holding it (pad tips 1.5 mm above the hub's bottom)
PRESS = float(os.environ.get("HORN_PRESS", "0.012"))  # how far past home the press is commanded (arm compliance)


def demo():
    hx, hy, _ = HORN0
    sx, sy, _ = SHAFT
    pick, place = math.radians(HORN0_YAW), math.radians(A0)
    crest = CASE_TOP + CREST + GRIP_Z + 0.0005
    home = CASE_TOP + SEAT + GRIP_Z
    up = CASE_TOP + 0.05
    return {"right": [
        Waypoint(1.5, (hx, hy, TOP + 0.08), FINGER_OPEN, "over the horn", pick),
        Waypoint(1.2, (hx, hy, TOP + GRIP_Z), FINGER_OPEN, "down to its hub", pick),
        Waypoint(0.8, (hx, hy, TOP + GRIP_Z), FINGER_CLOSED, "pinch the hub across", pick),
        Waypoint(1.0, (hx, hy, TOP + 0.08), FINGER_CLOSED, "lift", pick),
        Waypoint(2.0, (sx, sy, up), FINGER_CLOSED, "over the spline, arm turned to the left", place),
        Waypoint(1.5, (sx, sy, crest), FINGER_CLOSED, "onto the tooth tips", place),
        Waypoint(0.8, (sx, sy, crest), FINGER_CLOSED, "settle", place),
        Waypoint(1.5, (sx, sy, home - PRESS), FINGER_CLOSED, "push it home", place),
        Waypoint(1.0, (sx, sy, home - PRESS), FINGER_CLOSED, "hold the push", place),
        Waypoint(0.8, (sx, sy, home + 0.001), FINGER_CLOSED, "ease off", place),
        Waypoint(0.8, (sx, sy, home + 0.001), FINGER_OPEN, "let go", place),
        Waypoint(1.2, (sx, sy, up + 0.03), FINGER_OPEN, "straight up", place),
        Waypoint(1.5, (0.22, -0.25, TOP + 0.16), FINGER_OPEN, "away", place),
        Waypoint(4.0, (0.22, -0.25, TOP + 0.16), FINGER_OPEN, "wait", place),
    ]}
