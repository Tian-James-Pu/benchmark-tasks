"""Plug a 3 in PVC drain stub-up with a mechanical test plug for the rough-in pressure test.

On the station table, standing in for a framed bathroom floor: a 3/4 in plywood subfloor with a 2x4 stud wall at its back.
A 3 in Schedule 40 PVC DWV stub-up (88.9 mm OD, 77.9 mm bore) rises 45 mm out of the subfloor, its centre 305 mm (12 in)
in front of the wall's studs. Beside it, standing upright on the subfloor, is a 3 in end-of-pipe Gripper test plug (Cherne
270237, an Oatey brand): two glass-reinforced ABS plates with a natural-rubber O-ring between them on a galvanised carriage
bolt, and an extra-large zinc wing nut on top. The upper plate's 84 mm lip is wider than the bore, so the plug cannot
fall in. Turning the wing nut clockwise draws the bolt (and the lower plate) up, squeezing the O-ring out against the
bore until it grips.

The knack: the nut's thread drags the whole plug round with it, and a plug sitting loose in the pipe resists only by its
lip's friction on the pipe end, which is less. Turned on its own, the plug just spins in the pipe and never tightens. One
hand has to hold the plug still (pressing on the lip, outside the wing nut's sweep) while the other runs the wing nut down
about three turns, and the plug has to stay seated square on the pipe end while the rubber takes hold.

Model (mechkit, all contacts real):
- "stub" (fixed): the subfloor, the pipe wall as 24 convex segments of the annulus, the stud wall.
- "plug" (free articulation, 0.18 kg): frame = upper plate with its lip and eight moulded ribs, lower plate (colliders). Links:
  "nut", an unlimited hinge about the plug's axis carrying the wing nut (hub and two wings, real colliders);
  "bolt", a slider along the axis carrying the visible bolt end (no collider);
  "ring_px", "ring_nx", "ring_py", "ring_ny": four quarter-segments of the O-ring, each on a radial slider with a
  20 kN/m spring (the rubber's give), whose rest point the thread moves out as the nut turns.
- The thread (code, each control step): the nut's joint drive is a friction clutch. Its target is re-set to where the
  nut is, and its torque limit is the thread's drag torque T(draw-up), so turning the nut drags the plug with up to that
  torque (0.20 to 0.25 N m while the free O-ring is squeezed, then 0.25 N m more per mm). Whatever the nut turns
  relative to the plug feeds the thread: draw-up = turns x 1.5875 mm (3/8-16 bolt), the
  O-ring's rest radius grows 0.6 mm per mm of draw-up. The O-ring meets the bore after 2.0 mm of draw-up (1.3 turns).
- Not modelled: force or torque limits on the hands (none are scored), rubber hysteresis, the bolt's thread geometry.

Scoring: see `score()` and task.json; read by the tracker every control step.
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena import demo as _demo
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001

# -- the room: subfloor, stub-up, stud wall ------------------------------------------------------------------------
PLY_T = 0.018  # 23/32 in plywood subfloor
FLOOR = TOP + PLY_T
STUB_X, STUB_Y = 0.30, 0.0  # the stub-up's centre
STUB_H = 0.045  # it stands 45 mm proud of the subfloor
ZP = FLOOR + STUB_H  # the pipe's end face
PIPE_RO = 0.5 * 88.9 * MM  # 3 in Sch 40: 3.500 in OD
PIPE_RI = 0.5 * 77.93 * MM  # 3.068 in bore
WALL_X = STUB_X + 0.305 - 0.0127  # stud face: 12 in rough-in from the finished (1/2 in drywall) face

# -- the plug, in its own frame: origin on its axis at the lip's underside (the face that seats on the pipe end), +z out
LIP_R, LIP_T = 42.0 * MM, 6.0 * MM  # the upper plate's lip: wider than the bore, narrower than the pipe's OD
BODY_R = 37.0 * MM  # upper and lower plates below the lip (1.97 mm clear of the bore)
TOPBODY_Z = (-12.0 * MM, 0.0)
RING_Z = -18.0 * MM  # the O-ring's mid-plane
RING_TUBE = 4.5 * MM  # its cross-section radius
RING_R0 = 37.75 * MM  # its relaxed outer radius: 1.2 mm clear of the bore
BOTTOM_Z = (-29.0 * MM, -24.0 * MM)
HUB_R, HUB_Z = 9.5 * MM, (LIP_T, 17.0 * MM)
WING_T = 4.0 * MM  # each wing's thickness
WING_R = (6.0 * MM, 25.4 * MM)  # inner and outer reach of each wing (2 in span)
WING_Z = (8.0 * MM, 34.0 * MM)
BOLT_R = 0.5 * 9.525 * MM  # 3/8 in carriage bolt
BOLT_TOP0 = HUB_Z[1] - 8.0 * MM  # the bolt's end starts 8 mm down inside the nut's hub

PITCH = 25.4 * MM / 16  # 3/8-16 thread: 1.5875 mm a turn
EXPAND = 0.6  # O-ring radius gained per unit of draw-up
TOUCH = (PIPE_RI - RING_R0) / EXPAND  # draw-up at which the O-ring meets the bore: 2.0 mm
DRAW_MAX = 6.5 * MM  # O-ring squeezed solid
DRAW_SET = 4.2 * MM  # "expanded": the bolt end has risen 4.2 mm (2.6 turns), O-ring 1.3 mm into the bore's size
RING_K = 20000.0  # N/m each quarter: the rubber's radial give
RIBS, RIB_R, RIB_W, RIB_H = 8, (28.0 * MM, 41.5 * MM), 3.0 * MM, 4.0 * MM  # moulded stiffening ribs on the upper plate


def thread_torque(draw):
    """The torque (N m) the thread and the nut's face need to turn the nut at a given draw-up (m): what the plug must
    resist for the nut to advance. Squeezing the free O-ring, then pressing it against the bore."""
    if draw < TOUCH:
        return 0.20 + 0.05 * draw / TOUCH
    if draw < DRAW_MAX:
        return 0.25 + 250.0 * (draw - TOUCH)
    return 3.0


LOOKS = {
    "pvc": m.Look(m.rgb(0.93, 0.93, 0.90), roughness=0.45),
    "bore": m.Look(m.rgb(0.55, 0.55, 0.53), roughness=0.7),
    "ply": m.Look(m.rgb(0.78, 0.64, 0.44), roughness=0.85),
    "stud": m.Look(m.rgb(0.86, 0.74, 0.55), roughness=0.8),
    "abs": m.Look(m.rgb(0.95, 0.76, 0.10), roughness=0.5),
    "rubber": m.Look(m.rgb(0.10, 0.10, 0.10), roughness=0.9),
    "zinc": m.Look(m.rgb(0.74, 0.75, 0.77), metallic=0.85, roughness=0.3),
    "steel": m.Look(m.rgb(0.62, 0.63, 0.64), metallic=0.8, roughness=0.35),
}


def _annulus_sector(r0, r1, a0, a1, z0, z1, n=3):
    """A convex sector of an annulus (angles in radians), as a closed solid."""
    outer = [(r1 * math.cos(a), r1 * math.sin(a)) for a in np.linspace(a0, a1, n)]
    inner = [(r0 * math.cos(a), r0 * math.sin(a)) for a in np.linspace(a1, a0, n)]
    return m.prism(outer + inner, z0, z1)


def _stub():
    parts, cols = [], []
    # subfloor (in the stub part's frame: origin at the stub's centre on the table top)
    ply = m.box((0.60, 0.84, PLY_T), (0.40 - STUB_X, 0.0, PLY_T / 2))
    parts.append(("subfloor", ply, "ply"))
    cols.append(m.col_box("subfloor_c", (0.60, 0.84, PLY_T), (0.40 - STUB_X, 0.0, PLY_T / 2)))
    # the pipe: outside, end face and a darker bore, from the subfloor up
    shell = m.lathe([(PIPE_RO, PLY_T), (PIPE_RO, PLY_T + STUB_H), (PIPE_RI, PLY_T + STUB_H), (PIPE_RI, PLY_T)], 48)
    parts.append(("pipe", shell, "pvc"))
    parts.append(("bore", m.cylinder(PIPE_RI, 0.001, (0, 0, PLY_T + 0.001), segments=48), "bore"))  # the pipe's dark depths
    n = 24
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        cols.append(m.col_hull(f"pipe{k}_c", _annulus_sector(PIPE_RI, PIPE_RO, a0, a1, PLY_T, PLY_T + STUB_H)))
    # the stud wall: bottom plate, two studs 16 in on centre, a cut top piece
    wx = WALL_X - STUB_X
    plate = (0.089, 0.80, 0.038)
    parts.append(("plate", m.box(plate, (wx + 0.0445, 0.0, PLY_T + 0.019)), "stud"))
    cols.append(m.col_box("plate_c", plate, (wx + 0.0445, 0.0, PLY_T + 0.019)))
    for j, y in enumerate((-0.2032, 0.2032)):
        stud = (0.089, 0.038, 0.42)
        c = (wx + 0.0445, y, PLY_T + 0.038 + 0.21)
        parts.append((f"stud{j}", m.box(stud, c), "stud"))
        cols.append(m.col_box(f"stud{j}_c", stud, c))
    block = (0.089, 0.3684, 0.038)
    c = (wx + 0.0445, 0.0, PLY_T + 0.038 + 0.30)
    parts.append(("block", m.box(block, c), "stud"))
    cols.append(m.col_box("block_c", block, c))
    return parts, cols


STUB_PARTS, STUB_COLS = _stub()
STUB = m.fixed("stub", (STUB_X, STUB_Y, TOP), STUB_PARTS, STUB_COLS, LOOKS, friction=(0.6, 0.5))  # PVC on ABS and rubber


def _ring_quarter(axis_deg):
    """One quarter of the O-ring: a tube along an 84-degree arc, centred on direction axis_deg."""
    rc = RING_R0 - RING_TUBE
    a = np.radians(np.linspace(axis_deg - 42, axis_deg + 42, 8))
    pts = np.c_[rc * np.cos(a), rc * np.sin(a), np.full(len(a), RING_Z)]
    return m.tube(pts, RING_TUBE, sides=10)


def _wing(sign):
    """One wing, along +x (sign 1) or -x: a plate WING_T thick, taller at its outer end."""
    r0, r1 = WING_R
    z0, z1 = WING_Z
    outline = [(r0, z0), (r1, z0 + 6 * MM), (r1, z1), (r0 + 4 * MM, z1 - 2 * MM), (r0, z1 - 6 * MM)]
    w = m.prism(outline, -WING_T / 2, WING_T / 2).turned(90.0, "x")  # outline (radius, height) -> (x, z)
    return w if sign > 0 else w.turned(180.0, "z")


def _plug_links():
    bore = BOLT_R + 0.3 * MM
    hub = m.lathe([(HUB_R, HUB_Z[0]), (HUB_R, HUB_Z[1]), (bore, HUB_Z[1]), (bore, HUB_Z[0])], 12)  # the bolt end shows inside it
    nut_parts = [("hub", hub, "zinc"), ("wing_p", _wing(1), "zinc"), ("wing_n", _wing(-1), "zinc")]
    wing_len = WING_R[1] - WING_R[0]
    wing_h = WING_Z[1] - WING_Z[0]
    wing_c = (WING_R[0] + WING_R[1]) / 2
    nut_cols = [
        m.col_cylinder("hub_c", HUB_R, HUB_Z[1] - HUB_Z[0], (0, 0, (HUB_Z[0] + HUB_Z[1]) / 2)),
        m.col_box("wing_p_c", (wing_len, WING_T, wing_h), (wing_c, 0, (WING_Z[0] + WING_Z[1]) / 2)),
        m.col_box("wing_n_c", (wing_len, WING_T, wing_h), (-wing_c, 0, (WING_Z[0] + WING_Z[1]) / 2)),
    ]
    nut = m.hinge("nut", "Z", (0, 0, 0), float("-inf"), float("inf"), nut_parts, nut_cols, mass=0.03, spring=5.0,
                  damping=0.002, friction=0.0, effort=thread_torque(0.0), inertia=(6e-6, 6e-6, 6e-6))
    bolt_len = BOLT_TOP0 - BOTTOM_Z[0]
    bolt = m.slider("bolt", "Z", 0.0, DRAW_MAX + 0.0005,
                    [("bolt", m.cylinder(BOLT_R, bolt_len, (0, 0, BOTTOM_Z[0] + bolt_len / 2), segments=16), "steel")], [],
                    mass=0.015, spring=3000.0, damping=5.0, effort=100.0, friction=0.0)
    rings = []
    for name, deg, axis, sgn in (("ring_px", 0, "X", 1), ("ring_nx", 180, "X", -1), ("ring_py", 90, "Y", 1), ("ring_ny", 270, "Y", -1)):
        q = _ring_quarter(deg)
        lo, hi = (0.0, 0.004) if sgn > 0 else (-0.004, 0.0)
        rings.append(m.slider(name, axis, lo, hi, [("ring", q, "rubber")], [m.col_hull(f"{name}_c", q)],
                              mass=0.008, spring=RING_K, damping=60.0, effort=200.0, friction=0.0, armature=0.02))
    return [nut, bolt] + rings


RING_SIGN = {"ring_px": 1, "ring_nx": -1, "ring_py": 1, "ring_ny": -1}


def _plug_frame():
    lip = m.cylinder(LIP_R, LIP_T, (0, 0, LIP_T / 2), segments=48)
    top = m.cylinder(BODY_R, TOPBODY_Z[1] - TOPBODY_Z[0], (0, 0, (TOPBODY_Z[0] + TOPBODY_Z[1]) / 2), segments=48)
    bot = m.lathe([(0.0, BOTTOM_Z[0]), (BODY_R - 5.0 * MM, BOTTOM_Z[0]), (BODY_R, BOTTOM_Z[0] + 4.0 * MM), (BODY_R, BOTTOM_Z[1]), (0.0, BOTTOM_Z[1])], 48)  # chamfered lead-in
    ribs = []
    for k in range(RIBS):  # moulded radial ribs on the upper plate's top, outside the wing nut's sweep
        a = 22.5 + k * 360.0 / RIBS
        ribs.append(m.box((RIB_R[1] - RIB_R[0], RIB_W, RIB_H), ((RIB_R[0] + RIB_R[1]) / 2, 0, LIP_T + RIB_H / 2)).turned(a, "z"))
    parts = [("lip", lip, "abs"), ("top", top, "abs"), ("bottom", bot, "abs")] + [(f"rib{k}", r, "abs") for k, r in enumerate(ribs)]
    cols = [m.col_cylinder("lip_c", LIP_R, LIP_T, (0, 0, LIP_T / 2)),
            m.col_cylinder("top_c", BODY_R, TOPBODY_Z[1] - TOPBODY_Z[0], (0, 0, (TOPBODY_Z[0] + TOPBODY_Z[1]) / 2)),
            m.col_hull("bottom_c", bot)] + [m.col_hull(f"rib{k}_c", r) for k, r in enumerate(ribs)]
    return parts, cols


PLUG_YAW = 135.0  # the wings point along 135 degrees (and -45)
PLUG_START = (0.21, -0.15)  # standing on its lower plate on the subfloor
PLUG_Z0 = FLOOR - BOTTOM_Z[0] + 0.0005
FRAME_PARTS, FRAME_COLS = _plug_frame()
PLUG = m.mechanism("plug", (PLUG_START[0], PLUG_START[1], PLUG_Z0), _plug_links(), LOOKS, frame_parts=FRAME_PARTS,
                   frame_colliders=FRAME_COLS, yaw_deg=PLUG_YAW, friction=(0.6, 0.5), solver_iters=64, free_mass=0.10,
                   frame_inertia=(6e-5, 6e-5, 9e-5))

OBJECTS = [
    STUB,
    PLUG,
    # Distractors (SimReady props) on the subfloor, which scoring ignores.
    SimReady("pliers", pos=(0.47, -0.30, FLOOR), size=(0.246, 0.065, 0.014), mass=0.28, yaw=math.radians(70.0),
             package="handTool_pliers_groove_joint_10in_b01", usd="sm_handTool_pliers_groove_joint_10in_b01_01.usd"),
    SimReady("utility_knife", pos=(0.48, 0.27, FLOOR), size=(0.160, 0.061, 0.026), mass=0.12, yaw=0.4,
             package="handTool_knife_retractableBlade_a01", usd="sm_handTool_knife_retractableBlade_a01_01.usd"),
    SimReady("marker", pos=(0.42, 0.17, FLOOR), size=(0.109, 0.008, 0.007), mass=0.01, yaw=1.2,
             package="stationery_marker_thin_blue_a01", usd="sm_stationery_marker_thin_blue_a01_01.usd"),
]

# -- the thread and the scoring tracker ----------------------------------------------------------------------------

SUCCESS_HOLD_S = 0.0  # the tracker times its own 3 s hold
HOLD_S = 3.0
GAP_MAX = 1.5 * MM  # lip-to-pipe-end gap allowed all round
GAP_MIN = -1.5 * MM  # solver penetration tolerance
TILT_MAX = 2.0  # degrees
CLEAR = 0.03  # fingertip points this far from the plug
STILL_STEPS, STILL_MM, STILL_DEG = 20, 1.0, 1.0  # at rest: over 1 s, lip moved under 1 mm, nut turned under 1 degree
LIP_PTS = np.array([(0.040 * math.cos(a), 0.040 * math.sin(a), 0.0) for a in np.radians(np.arange(0, 360, 30))])
BODY_PT = np.array([0.0, 0.0, RING_Z])
PLUG_Z = (BOTTOM_Z[0], WING_Z[1])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _set_effort(env, link, value):
    asset, j = m._joint(env, "plug", link)
    asset.write_joint_effort_limit_to_sim(m._tensor(asset, value), joint_ids=[j])


def lip_gaps(st):
    """Heights (m) of 12 points under the lip, 40 mm out from the axis, above the pipe's end face."""
    return st.to_world(LIP_PTS)[:, 2] - ZP


def in_bore(st):
    """The plug's body (its O-ring plane) inside the stub-up: within 10 mm of the pipe's axis and below its end."""
    p = st.to_world(BODY_PT)[0]
    return bool(math.hypot(p[0] - STUB_X, p[1] - STUB_Y) < 0.010 and p[2] < ZP - 0.005)


def hands_clear(st, tips):
    """Every fingertip point more than CLEAR from the plug (a cylinder of the lip's radius over its full length)."""
    if tips is None:
        return False
    loc = st.to_local(tips)
    dr = np.maximum(np.hypot(loc[:, 0], loc[:, 1]) - LIP_R, 0.0)
    dz = np.maximum(np.maximum(loc[:, 2] - PLUG_Z[1], PLUG_Z[0] - loc[:, 2]), 0.0)
    return bool(np.hypot(dr, dz).min() > CLEAR)


class Trial:
    """The thread's state and the score, advanced once per control step. Cleared only by a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.prev = None  # the nut's last joint reading
        self.tight = 0.0  # radians the nut has turned clockwise (seen from above) relative to the plug, net
        self.draw = 0.0  # draw-up of the bolt (m)
        self.hold = 0.0
        self.full = False
        self.value = 0.0
        self.tips = None
        self.nut_rate = 0.0
        self.recent = []  # the last STILL_STEPS readings of the lip points (world) and the nut's net turn
        if env is not None:
            self._drive(env)

    def _drive(self, env):
        m.set_target(env, "plug", "bolt", self.draw)
        e = EXPAND * self.draw
        for name, sgn in RING_SIGN.items():
            m.set_target(env, "plug", name, sgn * e)
        _set_effort(env, "nut", thread_torque(self.draw))

    def step(self, env):
        st = m.state(env, "plug")
        th = st.joint("nut")
        d = 0.0 if self.prev is None else _wrap(th - self.prev)
        self.prev = th
        self.nut_rate = abs(d)
        # clockwise from above is negative about the plug's +z: it tightens. Backing the nut off past finger-snug
        # just spins it loose (draw-up stays 0), up to two turns.
        self.tight = float(np.clip(self.tight - d, -4 * math.pi, DRAW_MAX / PITCH * 2 * math.pi))
        self.draw = float(np.clip(self.tight / (2 * math.pi) * PITCH, 0.0, DRAW_MAX))
        m.set_target(env, "plug", "nut", th)  # the clutch sticks again where the nut now is
        self._drive(env)
        self.tips = m.fingertips(env)
        self.update(st, env.unwrapped.step_dt)

    def update(self, st, dt):
        inside = in_bore(st)
        expanded = st.joint("bolt") >= DRAW_SET - 0.05 * MM  # the bolt end's rise, as a depth gauge reads it
        gaps = lip_gaps(st)
        square = bool(np.all(gaps <= GAP_MAX) and np.all(gaps >= GAP_MIN) and st.tilt_deg() <= TILT_MAX)
        # at rest: over the last second no lip point has moved 1 mm and the nut has not turned 1 degree. (Instantaneous
        # velocities are no use here: the solver's contact jitter on the squeezed O-ring reads as 0.02 m/s with no motion.)
        lip = st.to_world(LIP_PTS)
        self.recent = (self.recent + [(lip, self.tight)])[-STILL_STEPS:]
        still = bool(len(self.recent) == STILL_STEPS
                     and max(float(np.linalg.norm(lip - q, axis=1).max()) for q, _ in self.recent) < STILL_MM * MM
                     and max(abs(self.tight - t) for _, t in self.recent) < math.radians(STILL_DEG))
        ready = inside and expanded and square and still and hands_clear(st, self.tips)
        self.hold = self.hold + float(dt) if ready else 0.0
        self.full = bool(ready and self.hold >= HOLD_S - 1e-9)
        if self.full:
            self.value = 1.0
        elif inside and expanded:
            self.value = 0.5
        elif inside:
            self.value = 0.25
        else:
            self.value = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo (open loop: it never reads the scene or the tracker) ---------------------------------------


class Orient(float):
    """A waypoint 'yaw' that carries a whole TCP rotation."""

    rot = np.eye(3)


def _orient(r):
    o = Orient(0.0)
    o.rot = r
    return o


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


def tilted(theta_deg, phi_deg):
    """Tilted theta from straight down towards phi (from +x), fingers closing across that direction."""
    phi = math.radians(phi_deg)
    r = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain_grip(phi)
    return _orient(r)


def down(deg):
    return math.radians(deg)


TIP = 0.013  # pad tips past the TCP
PINCH_Z = 19.0 * MM  # fingertips 2 mm above the hub, on the wings' upper 15 mm (plug frame)
STROKES = 7  # half-turns of the wing nut
STROKE_S = 3.0
PRESS = 0.018  # the holder is commanded this far into the lip (the wrist is soft: this is a few newtons)
HOLD_TILT = 45.0
# red-team knobs (a variant file sets them after this module's code): the demo uses the defaults
GRIP_DZ = 0.0005  # the turning grip's height over the seated wings (negative presses down on the nut)
HOLD_STROKES = STROKES  # half-turns the left hand holds the plug for
USE_HOLDER = True


def _turn(pos, yaw_from, yaw_to, t, finger, label):
    """Waypoints turning the grip about the vertical from yaw_from to yaw_to (degrees) in 45-degree steps."""
    n = max(1, int(math.ceil(abs(yaw_to - yaw_from) / 45.0 - 1e-9)))
    return [Waypoint(t / n, pos, finger, f"{label} {k + 1}/{n}", down(yaw_from + (yaw_to - yaw_from) * (k + 1) / n)) for k in range(n)]


def holder_tcp(press=None):
    """Left arm: closed fingers, tilted 45 degrees towards -y (its body leaning out, away from the wing nut), the pads'
    lowest corner on the lip 39 mm out on the +y side."""
    press = PRESS if press is None else press
    th = math.radians(HOLD_TILT)
    a = np.array([0.0, -math.sin(th), -math.cos(th)])  # pointing direction
    b = np.array([0.0, math.cos(th), -math.sin(th)])  # across the finger's width, towards its lower corner
    corner = np.array([STUB_X, STUB_Y + 0.039, ZP + LIP_T - press])
    tip = corner - 0.0077 * b
    return tip - TIP * a


def demo():
    """The right arm pinches the wing nut's two wings above the hub, carries the plug over the stub-up and lowers it in
    until its lip seats on the pipe end, and lets go. The left arm presses its closed fingers on the lip beside the nut
    and holds the plug still. The right arm closes on the wings again and runs the nut down in seven half-turn strokes
    (open, turn back, close between strokes). Both arms let go and leave."""
    sx, sy = PLUG_START
    pick = (sx, sy, PLUG_Z0 + PINCH_Z + TIP)
    above_pick = (sx, sy, FLOOR + 0.11)
    carry = (STUB_X, STUB_Y, FLOOR + 0.127)  # straight-down grips higher than this reach joint 4's limit
    seat = (STUB_X, STUB_Y, ZP + 0.002 + PINCH_Z + TIP)  # lip 2 mm over the pipe end
    grip = (STUB_X, STUB_Y, ZP + PINCH_Z + TIP + GRIP_DZ)  # not pressing down on the nut
    grip_up = grip[:2] + (grip[2] + 0.012,)  # open fingers turn back above the holder's fingers
    y0, y1 = PLUG_YAW, PLUG_YAW - 180.0
    right = [
        Waypoint(2.0, above_pick, FINGER_OPEN, "over the plug", down(y0)),
        Waypoint(1.2, pick, FINGER_OPEN, "fingers round the wings", down(y0)),
        Waypoint(0.6, pick, FINGER_CLOSED, "pinch the wings", down(y0)),
        Waypoint(2.0, above_pick[:2] + (carry[2],), FINGER_CLOSED, "lift", down(y0)),
        Waypoint(3.5, carry, FINGER_CLOSED, "over the stub-up (slowly: the plug hangs from a pinch)", down(y0)),
        Waypoint(1.0, carry, FINGER_CLOSED, "let it hang still", down(y0)),
        Waypoint(2.0, seat[:2] + (seat[2] + 0.008,), FINGER_CLOSED, "lower plate into the pipe", down(y0)),
        Waypoint(0.5, seat[:2] + (seat[2] + 0.008,), FINGER_OPEN, "let it drop and seat", down(y0)),
        Waypoint(0.6, seat[:2] + (seat[2] + 0.02,), FINGER_OPEN, "up off it", down(y0)),
        Waypoint(2.0, seat[:2] + (seat[2] + 0.02,), FINGER_OPEN, "wait for the holder", down(y0)),
        Waypoint(0.8, grip, FINGER_OPEN, "down round the wings", down(y0)),
        Waypoint(0.5, grip, FINGER_CLOSED, "pinch", down(y0)),
    ]
    stroke_end = []
    for k in range(STROKES):
        right += _turn(grip, y0, y1, STROKE_S, FINGER_CLOSED, f"half-turn {k + 1}")
        right.append(Waypoint(0.4, grip, FINGER_OPEN, "open", down(y1)))
        stroke_end.append(sum(w.duration for w in right))
        if k < STROKES - 1:
            right.append(Waypoint(0.4, grip_up, FINGER_OPEN, "up", down(y1)))
            right += _turn(grip_up, y1, y0, 1.6, FINGER_OPEN, "back")
            right.append(Waypoint(0.4, grip, FINGER_OPEN, "down", down(y0)))
            right.append(Waypoint(0.4, grip, FINGER_CLOSED, "pinch", down(y0)))
    right += [
        Waypoint(0.8, grip[:2] + (grip[2] + 0.05,), FINGER_OPEN, "up", down(y1)),
        Waypoint(1.5, (0.20, -0.20, FLOOR + 0.12), FINGER_OPEN, "away", down(0.0)),
        Waypoint(5.0, (0.20, -0.20, FLOOR + 0.12), FINGER_OPEN, "wait while the plug is checked", down(0.0)),
    ]
    t_right_press = sum(w.duration for w in right[:11])  # the plug has seated; the right arm is up off it
    t_turned = stroke_end[min(HOLD_STROKES, STROKES) - 1]
    hold = tilted(HOLD_TILT, -90.0)
    press = tuple(holder_tcp())
    over = tuple(holder_tcp(-0.015) + np.array([0.0, 0.006, 0.0]))
    left = [
        Waypoint(2.5, (0.22, 0.16, FLOOR + 0.20), FINGER_CLOSED, "stand by", hold),
        Waypoint(t_right_press - 2.5 - 2.0 - 0.4, (0.22, 0.16, FLOOR + 0.20), FINGER_CLOSED, "wait", hold),
        Waypoint(1.2, over, FINGER_CLOSED, "over the lip", hold),
        Waypoint(0.8, press, FINGER_CLOSED, "press on the lip", hold),
    ]
    t_left = sum(w.duration for w in left)
    left += [
        Waypoint(t_turned - t_left, press, FINGER_CLOSED, "hold the plug still", hold),
        Waypoint(0.8, over[:2] + (over[2] + 0.04,), FINGER_CLOSED, "lift off", hold),
        Waypoint(1.5, (0.22, 0.20, FLOOR + 0.22), FINGER_CLOSED, "away", hold),
        Waypoint(5.0, (0.22, 0.20, FLOOR + 0.22), FINGER_CLOSED, "wait while the plug is checked", hold),
    ]
    if not USE_HOLDER:
        left = [Waypoint(3.0, (0.22, 0.20, FLOOR + 0.22), FINGER_CLOSED, "stay away", hold)]
    return {"left": left, "right": right}
