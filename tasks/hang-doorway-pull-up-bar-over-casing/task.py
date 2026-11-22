"""A full-size interior doorway built into a stud-wall section that stands across the station table, and a
leverage-style doorway pull-up bar lying on the table in front of it, its hooks pointing through the doorway.

The doorway (only its top 30 cm is modelled: the table top stands in for a plane 1.73 m above the floor, so the
head jamb is at the height of a standard 6 ft 8 in door opening relative to the bar): an 81.0 cm opening between
19 mm pine jambs 115 mm deep (2x4 studs plus 12.7 mm drywall each side), a 10 x 32 mm door stop on the head jamb,
and 15 x 64 mm casing trim on both faces, set back 3 mm from the jamb's edge. The wall runs 30 cm above the
opening's top and 18 cm past the casing on each side.

The bar is a leverage bar of the Iron Gym "Total Upper Body Workout Bar" kind: 93 x 33 x 29.5 cm overall,
2.15 kg of 25.4 mm steel tube. Two hooks (inverted L, 3 cm wide, foam lined) hang over the top of the casing on
the far side of the wall; from them two side rails pass under the head jamb, and on the near side carry a pad bar,
whose two foam pads (5 cm across) bear on the near casing's two side legs, and the grip bar with its foam grips.
Nothing is screwed: the bar's own weight (and in use, the user's) turns it about the hooks, and the pads stop it.

x is towards the wall (the doorway's "far" side is beyond it), y is along the wall, z is up. Lengths in metres.
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the doorway, in its own frame: origin at the far casing's outer face (x), the opening's centre line (y) and the
# underside of the head jamb (z) ---------------------------------------------------------------------------------

OPEN_ABOVE = 0.30          # the head jamb's underside above the table top
X_FAR = 0.615              # world x of the far casing's outer face
DOOR_ORIGIN = (X_FAR, 0.0, TOP + OPEN_ABOVE)
HALF_OPEN = 0.405          # 81.0 cm between the side jambs
JAMB_T = 0.019             # jamb thickness (3/4 in)
CASE_T, CASE_W, REVEAL = 0.015, 0.064, 0.003   # casing thickness and width, and its set-back from the jamb's edge
WALL_IN = (-0.130, -0.015)  # x of the wall's two faces (the jamb is flush with them), between the casings
X_NEAR = -0.145            # the near casing's face
CASE_TOP = REVEAL + CASE_W  # 0.067: the top edge of the head casings
CASE_OUT = HALF_OPEN + REVEAL + CASE_W  # 0.472: the outer edge of the side casings
STOP = (-0.082, -0.050, 0.010)  # the head stop's x span and depth below the jamb
WALL_UP, WALL_OUT = 0.16, 0.65  # the wall section's top above the opening (cut low so the camera sees over it), and its half-width

DOOR_LOOKS = {
    "wall": m.Look(m.rgb(0.90, 0.88, 0.84), roughness=0.85),
    "trim": m.Look(m.rgb(0.97, 0.97, 0.95), roughness=0.35),
    "jamb": m.Look(m.rgb(0.95, 0.95, 0.93), roughness=0.4),
    "cut": m.Look(m.rgb(0.78, 0.66, 0.48), roughness=0.9),  # the wall section's sawn top: studs and drywall
}


def _bx(x0, x1, y0, y1, z0, z1):
    return (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)


def _door_boxes():
    """(name, look, size, centre) for every board of the doorway, in the doorway's frame."""
    lo = -OPEN_ABOVE  # the table top
    w0, w1 = WALL_IN
    b = []
    b.append(("head_jamb", "jamb", *_bx(w0, w1, -HALF_OPEN - JAMB_T, HALF_OPEN + JAMB_T, 0.0, JAMB_T)))
    for s, side in ((1, "l"), (-1, "r")):
        y0, y1 = sorted((s * HALF_OPEN, s * (HALF_OPEN + JAMB_T)))
        b.append((f"side_jamb_{side}", "jamb", *_bx(w0, w1, y0, y1, lo, JAMB_T)))
        y0, y1 = sorted((s * (HALF_OPEN - STOP[2]), s * HALF_OPEN))
        b.append((f"side_stop_{side}", "trim", *_bx(STOP[0], STOP[1], y0, y1, lo, 0.0)))
        y0, y1 = sorted((s * (HALF_OPEN + JAMB_T), s * WALL_OUT))
        b.append((f"side_wall_{side}", "wall", *_bx(w0, w1, y0, y1, lo, WALL_UP)))
        y0, y1 = sorted((s * (HALF_OPEN + REVEAL), s * CASE_OUT))
        b.append((f"near_leg_{side}", "trim", *_bx(X_NEAR, w0, y0, y1, lo, CASE_TOP)))
        b.append((f"far_leg_{side}", "trim", *_bx(w1, 0.0, y0, y1, lo, CASE_TOP)))
    b.append(("head_stop", "trim", *_bx(STOP[0], STOP[1], -HALF_OPEN, HALF_OPEN, -STOP[2], 0.0)))
    b.append(("header_wall", "wall", *_bx(w0, w1, -HALF_OPEN - JAMB_T, HALF_OPEN + JAMB_T, JAMB_T, WALL_UP)))
    b.append(("near_head", "trim", *_bx(X_NEAR, w0, -CASE_OUT, CASE_OUT, REVEAL, CASE_TOP)))
    b.append(("far_head", "trim", *_bx(w1, 0.0, -CASE_OUT, CASE_OUT, REVEAL, CASE_TOP)))
    return b


DOOR_BOXES = _door_boxes()
_door_parts = [(n, m.box(s, c), look) for n, look, s, c in DOOR_BOXES]
_door_parts.append(("cut_top", m.box((WALL_IN[1] - WALL_IN[0], 2 * WALL_OUT, 0.002), ((WALL_IN[0] + WALL_IN[1]) / 2, 0.0, WALL_UP + 0.001)), "cut"))
DOOR = m.fixed("doorway", DOOR_ORIGIN, _door_parts, [m.col_box(f"{n}_c", s, c) for n, _, s, c in DOOR_BOXES], DOOR_LOOKS,
               friction=(0.8, 0.6))

# -- the bar, in its own frame: where it sits when mounted, the frame then coinciding with the doorway's ------------

HOOK_Y = 0.33                     # the hooks and side rails, either side of the centre line
HOOK_W = 0.030                    # each hook's width along y
TUBE_R = 0.0127                   # 25.4 mm steel tube
FOAM_R = 0.016                    # foam grips, 32 mm
RAIL_Z = -0.080                   # the side rails' axis under the head jamb
HOOK_FOOT = (-0.013, 0.025, CASE_TOP, CASE_TOP + 0.022)   # x0, x1, z0, z1: rests on the far casing's top edge
HOOK_LEG = (0.0, 0.025, RAIL_Z - TUBE_R, CASE_TOP + 0.022)  # bears on the far casing's outer face
BEND = (-0.180, RAIL_Z)           # where each rail turns down, on the near side
PAD_BAR = (-0.200, -0.150)        # x, z of the pad bar's axis
PAD_Y, PAD_R = 0.440, 0.025       # the pads, on the casing's side legs
PAD_X = (-0.185, X_NEAR)          # the pads' x span: their faces meet the near casing when mounted
GRIP = (-0.290, -0.190)           # x, z of the grip bar's axis
GRIP_HALF = 0.465                 # 93 cm grip bar
FOAM_Y = (0.12, 0.30)             # foam grip sections, either side
BAR_MASS = 2.15

BAR_LOOKS = {
    "steel": m.Look(m.rgb(0.08, 0.08, 0.09), metallic=0.6, roughness=0.35),
    "foam": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.95),
    "accent": m.Look(m.rgb(0.10, 0.35, 0.75), roughness=0.6),
}


def _seg(a, b, y):
    return [(a[0], y, a[1]), (b[0], y, b[1])]


def _bar():
    parts, cols, mass_pts = [], [], []

    def tube(name, pts, r, look, kg_per_m=0.72):
        mesh = m.tube(pts, r, sides=16)
        parts.append((name, mesh, look))
        cols.append(m.col_hull(f"{name}_c", m.tube(pts, r, sides=12)))
        p0, p1 = np.asarray(pts[0], float), np.asarray(pts[-1], float)
        mass_pts.append(((p0 + p1) / 2, kg_per_m * float(np.linalg.norm(p1 - p0))))

    for s, side in ((1, "l"), (-1, "r")):
        y = s * HOOK_Y
        fx0, fx1, fz0, fz1 = HOOK_FOOT
        lx0, lx1, lz0, lz1 = HOOK_LEG
        foot = m.box((fx1 - fx0, HOOK_W, fz1 - fz0), ((fx0 + fx1) / 2, y, (fz0 + fz1) / 2))
        leg = m.box((lx1 - lx0, HOOK_W, lz1 - lz0), ((lx0 + lx1) / 2, y, (lz0 + lz1) / 2))
        parts += [(f"hook_foot_{side}", foot, "foam"), (f"hook_leg_{side}", leg, "steel")]
        cols += [m.col_box(f"hook_foot_{side}_c", (fx1 - fx0, HOOK_W, fz1 - fz0), ((fx0 + fx1) / 2, y, (fz0 + fz1) / 2)),
                 m.col_box(f"hook_leg_{side}_c", (lx1 - lx0, HOOK_W, lz1 - lz0), ((lx0 + lx1) / 2, y, (lz0 + lz1) / 2))]
        mass_pts.append((np.array([0.006, y, 0.02]), 0.09))
        tube(f"rail_{side}", _seg(((lx0 + lx1) / 2, RAIL_Z), BEND, y), TUBE_R, "steel")
        tube(f"drop_{side}", _seg(BEND, PAD_BAR, y), TUBE_R, "steel")
        tube(f"handle_{side}", _seg(PAD_BAR, GRIP, y), FOAM_R, "foam", 0.8)
        pad = m.cylinder(PAD_R, PAD_X[1] - PAD_X[0], ((PAD_X[0] + PAD_X[1]) / 2, s * PAD_Y, PAD_BAR[1]), axis="x", segments=20)
        parts.append((f"pad_{side}", pad, "foam"))
        cols.append(m.col_hull(f"pad_{side}_c", m.cylinder(PAD_R, PAD_X[1] - PAD_X[0], ((PAD_X[0] + PAD_X[1]) / 2, s * PAD_Y, PAD_BAR[1]), axis="x", segments=14)))
        mass_pts.append((np.array([(PAD_X[0] + PAD_X[1]) / 2, s * PAD_Y, PAD_BAR[1]]), 0.02))
        # grip bar: foam grip, then the bare steel end (the middle is joined below)
        tube(f"foam_{side}", [(GRIP[0], s * FOAM_Y[0], GRIP[1]), (GRIP[0], s * FOAM_Y[1], GRIP[1])], FOAM_R, "foam", 0.80)
        tube(f"grip_end_{side}", [(GRIP[0], s * FOAM_Y[1], GRIP[1]), (GRIP[0], s * GRIP_HALF, GRIP[1])], TUBE_R, "steel")
        # the blue end caps and joint collars, for looks only
        parts.append((f"cap_{side}", m.cylinder(0.0135, 0.012, (GRIP[0], s * (GRIP_HALF - 0.006), GRIP[1]), axis="y", segments=16), "accent"))
        parts.append((f"collar_{side}", m.cylinder(0.019, 0.04, (GRIP[0], y, GRIP[1]), axis="y", segments=16), "accent"))
    tube("pad_bar", [(PAD_BAR[0], -PAD_Y + PAD_R, PAD_BAR[1]), (PAD_BAR[0], PAD_Y - PAD_R, PAD_BAR[1])], TUBE_R, "steel")
    tube("grip_mid", [(GRIP[0], -FOAM_Y[0], GRIP[1]), (GRIP[0], FOAM_Y[0], GRIP[1])], TUBE_R, "steel")
    pts = np.array([p for p, _ in mass_pts])
    w = np.array([k for _, k in mass_pts])
    com = (pts * w[:, None]).sum(0) / w.sum()
    return parts, cols, tuple(float(v) for v in com)


BAR_PARTS, BAR_COLS, BAR_COM = _bar()


def _ry(deg):
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(deg))


def _bar_points():
    """Points on the bar's colliders (its frame): ends of every tube with their radius, and hook corners."""
    p = []
    for s in (1, -1):
        y = s * HOOK_Y
        for x in (HOOK_FOOT[0], HOOK_FOOT[1]):
            for z in (HOOK_FOOT[2], HOOK_FOOT[3]):
                for dy in (-HOOK_W / 2, HOOK_W / 2):
                    p.append((x, y + dy, z))
        for x in (HOOK_LEG[0], HOOK_LEG[1]):
            for z in (HOOK_LEG[2], HOOK_LEG[3]):
                for dy in (-HOOK_W / 2, HOOK_W / 2):
                    p.append((x, y + dy, z))
        for (a, b, r) in ((((HOOK_LEG[0] + HOOK_LEG[1]) / 2, RAIL_Z), BEND, TUBE_R), (BEND, PAD_BAR, TUBE_R), (PAD_BAR, GRIP, FOAM_R)):
            for t in np.linspace(0, 1, 6):
                c = np.array(a) + t * (np.array(b) - np.array(a))
                for ang in np.linspace(0, 2 * math.pi, 12, endpoint=False):
                    p.append((c[0] + r * math.cos(ang), y, c[1] + r * math.sin(ang)))
        for x in PAD_X:
            for ang in np.linspace(0, 2 * math.pi, 12, endpoint=False):
                p.append((x, s * PAD_Y + PAD_R * math.cos(ang), PAD_BAR[1] + PAD_R * math.sin(ang)))
    for yy in np.linspace(-GRIP_HALF, GRIP_HALF, 15):
        r = FOAM_R if FOAM_Y[0] <= abs(yy) <= FOAM_Y[1] else TUBE_R
        for ang in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            p.append((GRIP[0] + r * math.cos(ang), yy, GRIP[1] + r * math.sin(ang)))
    for yy in np.linspace(-PAD_Y, PAD_Y, 13):
        for ang in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            p.append((PAD_BAR[0] + TUBE_R * math.cos(ang), yy, PAD_BAR[1] + TUBE_R * math.sin(ang)))
    return np.array(p)


BAR_POINTS = _bar_points()


def _floor_pitch():
    """The pitch (deg, near end up) at which the bar lies on a flat floor, on its grip bar and its two pads."""
    is_grip = (abs(BAR_POINTS[:, 0] - GRIP[0]) <= FOAM_R + 1e-6) & (abs(BAR_POINTS[:, 2] - GRIP[1]) <= FOAM_R + 1e-6)
    is_pad = (BAR_POINTS[:, 0] >= PAD_X[0] - 1e-6) & (abs(BAR_POINTS[:, 1]) >= PAD_Y - PAD_R - 1e-6)
    for k in range(0, 4001):
        a = k * 0.01
        z = (BAR_POINTS @ _ry(a).T)[:, 2]
        if z[is_pad].min() <= z[is_grip].min():
            return a
    raise ValueError("no floor pitch found")


FLOOR_PITCH = _floor_pitch()                       # about 14 degrees
START_R = _ry(FLOOR_PITCH)
_lowest = float((BAR_POINTS @ START_R.T)[:, 2].min())
START_GRIP_X = 0.270                               # the grip bar's axis, world x, as it lies
_g = START_R @ np.array([GRIP[0], 0.0, GRIP[1]])
START_POS = np.array([START_GRIP_X - _g[0], 0.0, TOP - _lowest + 0.0005])


def _xyzw(r):
    t = np.trace(r)
    s = math.sqrt(max(t + 1.0, 1e-12)) * 2
    q = np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    return tuple(float(v) for v in q / np.linalg.norm(q))


BAR = m.free("bar", tuple(START_POS), BAR_PARTS, BAR_COLS, BAR_LOOKS, mass=BAR_MASS, friction=(1.0, 0.9),
             start_quat=_xyzw(START_R), com=BAR_COM, solver_iters=48)

OBJECTS = [
    DOOR,
    BAR,
    # Distractors (SimReady props), which scoring ignores: a water bottle and a folded towel on the floor nearby.
    SimReady("water_bottle", pos=(0.12, -0.52, TOP), size=(0.068, 0.068, 0.245), mass=0.52,
             package="food_beverage_bottledWater_b01", usd="sm_food_beverage_bottledWater_b01_01.usd"),
    SimReady("towel", pos=(0.16, 0.53, TOP), size=(0.205, 0.212, 0.034), mass=0.06, yaw=0.3,
             package="consumable_microfiber_towel_folded_small_b01", usd="sm_consumable_microfiber_towel_folded_small_b01_01.usd"),
]

# -- reading the bar ----------------------------------------------------------------------------------------------

DOOR_O = np.array(DOOR_ORIGIN)
SEAT_CORNER = np.array([0.0, CASE_TOP])      # the far casing's top outer edge (x, z), doorway frame


def _door(p):
    """World points to the doorway's frame (it is not turned)."""
    return np.asarray(p, float) - DOOR_O


def hook_reading(st, s):
    """For the hook on side s (+1 left, -1 right): (dx, dz, over), where (dx, dz) is the hook's inner corner (the
    foot's underside meeting the leg's inner face) from the far casing's top outer edge, and `over` is how far the
    foot reaches back over the casing's top (m)."""
    corner = _door(st.to_world(np.array([[HOOK_LEG[0], s * HOOK_Y, HOOK_FOOT[2]]]))[0])
    tip = _door(st.to_world(np.array([[HOOK_FOOT[0], s * HOOK_Y, HOOK_FOOT[2]]]))[0])
    return float(corner[0] - SEAT_CORNER[0]), float(corner[2] - SEAT_CORNER[1]), float(-tip[0])


def hook_on(st, s):
    """The hook rests on the far casing's top: its foot's underside within -2..+5 mm of the casing's top, reaching at
    least 5 mm back over it, and its leg within 1 cm outside the casing's face."""
    dx, dz, over = hook_reading(st, s)
    return -0.002 <= dz <= 0.005 and over >= 0.005 and -0.002 <= dx <= 0.010


def pad_gap(st, s):
    """How far the pad on side s stands off the near casing's face (m; negative: pressed in), and whether its face's
    centre is on the casing's side leg."""
    face = _door(st.to_world(np.array([[PAD_X[1], s * PAD_Y, PAD_BAR[1]]]))[0])
    on_leg = (HALF_OPEN + REVEAL) <= abs(face[1]) <= CASE_OUT and -OPEN_ABOVE < face[2] < CASE_TOP
    return float(face[0] - X_NEAR) * -1.0, bool(on_leg)


def grip_level_deg(st):
    """The grip bar's slope along its length, degrees."""
    a = st.to_world(np.array([[GRIP[0], -GRIP_HALF, GRIP[1]], [GRIP[0], GRIP_HALF, GRIP[1]]]))
    d = a[1] - a[0]
    return float(math.degrees(math.asin(abs(d[2]) / np.linalg.norm(d))))


# Segments of the bar for the hands-clear check: (a, b, radius), bar frame.
CLEAR_SEGS = []
for _s in (1, -1):
    _y = _s * HOOK_Y
    CLEAR_SEGS += [((HOOK_LEG[0], _y, HOOK_LEG[2]), (HOOK_LEG[0], _y, HOOK_LEG[3]), 0.02),
                   ((0.0125, _y, RAIL_Z), (BEND[0], _y, BEND[1]), TUBE_R),
                   ((BEND[0], _y, BEND[1]), (PAD_BAR[0], _y, PAD_BAR[1]), TUBE_R),
                   ((PAD_BAR[0], _y, PAD_BAR[1]), (GRIP[0], _y, GRIP[1]), FOAM_R),
                   ((PAD_X[0], _s * PAD_Y, PAD_BAR[1]), (PAD_X[1], _s * PAD_Y, PAD_BAR[1]), PAD_R)]
CLEAR_SEGS += [((PAD_BAR[0], -PAD_Y, PAD_BAR[1]), (PAD_BAR[0], PAD_Y, PAD_BAR[1]), TUBE_R),
               ((GRIP[0], -GRIP_HALF, GRIP[1]), (GRIP[0], GRIP_HALF, GRIP[1]), FOAM_R)]


def hands_gap(st, tips):
    """The least distance from any fingertip point to the bar's surface (m)."""
    local = st.to_local(tips)
    d = np.full(len(local), np.inf)
    for a, b, r in CLEAR_SEGS:
        a, b = np.asarray(a, float), np.asarray(b, float)
        t = np.clip((local - a) @ (b - a) / np.dot(b - a, b - a), 0.0, 1.0)
        d = np.minimum(d, np.linalg.norm(local - (a + t[:, None] * (b - a)), axis=1) - r)
    return float(d.min())


# -- scoring ------------------------------------------------------------------------------------------------------

SUCCESS_HOLD_S = 0.0      # the trial counts its own 3 s hold
SCORE_OVER = "end"
HOLD_S = 3.0
PAD_TOL = 0.003           # a pad bears: within 3 mm of the casing's face
LEVEL_DEG = 2.0
CLEAR = 0.05              # every fingertip point at least 5 cm from the bar
DROP_SPEED = 1.0          # m/s downward: the bar fell (free fall of 5 cm)
STILL_V, STILL_W = 0.01, 0.05


class Trial:
    """Updated once per control step. `reset` is the only place a recorded drop is cleared."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.dropped = False
        self.hold = 0.0
        self.full = False
        self.value = 0.0
        self.parts = {}

    def update(self, st, tips, dt):
        if float(st.lin_vel[2]) < -DROP_SPEED:
            self.dropped = True
        hooks = [hook_on(st, s) for s in (1, -1)]
        pads = [pad_gap(st, s) for s in (1, -1)]
        pads_bear = all(-PAD_TOL <= g <= PAD_TOL and leg for g, leg in pads)
        level = grip_level_deg(st) <= LEVEL_DEG
        seated = all(hooks) and pads_bear and level
        still = float(np.linalg.norm(st.lin_vel)) < STILL_V and float(np.linalg.norm(st.ang_vel)) < STILL_W
        clear = tips is not None and hands_gap(st, tips) >= CLEAR
        ready = seated and still and clear and not self.dropped
        self.hold = self.hold + float(dt) if ready else 0.0
        self.full = bool(ready and self.hold >= HOLD_S - 1e-9)
        if self.dropped:
            self.value = 0.0
        elif self.full:
            self.value = 1.0
        elif seated:
            self.value = 0.75
        elif all(hooks):
            self.value = 0.5
        else:
            self.value = 0.0       # one hook or none: a bar on one hook twists off
        self.parts = dict(hooks=hooks, pads=pads, level=grip_level_deg(st), still=still, clear=clear, dropped=self.dropped)

    def step(self, environment):
        st = m.state(environment, "bar")
        tips = m.fingertips(environment)
        self.update(st, tips, environment.unwrapped.step_dt)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo (closed loop: run with demokit.py) ---------------------------------------------------------
#
# Both grippers take the pad bar, which runs through the bar's balance point, at y = +-21 cm, from above and behind
# (25 degrees from vertical, the fingers closing across the tube). From then on the script steers the bar, not the
# hands: each control step it reads the bar's pose (as a camera-based policy would see it), compares it with where
# the plan wants the bar now, and shifts each hand's command by a share of the difference, so that the arms' give
# under the bar's weight and any turn of the tube in the fingers are taken out as they happen. The plan, in the
# doorway's frame (`corner` is the hooks' inner corner, where the foot's underside meets the leg):
#   1. lift the bar 2 cm off the table at its starting tilt (12 degrees, near end up);
#   2. push it 3.4 cm through, so that the hook feet are clear of the far casing's face;
#   3. raise it until the corners are 1.5 cm above the casing's top edge;
#   4. turn it down to 3 degrees about the corners, bringing the corners back to 7.5 mm outside the casing's face;
#   5. lower it onto the casing while turning it level: the feet come down on the casing's top, the legs against its
#      face and the pads against the near casing, all together;
#   6. let the doorway take the weight, let go, back the hands off along the fingers and move them away.

GRASP_Y = 0.21                # where each hand takes the pad bar, either side of the centre line
GRASP_BETA = -25.0            # each grip's tilt from straight down, towards the doorway, as it takes the bar
CORNER = np.array([HOOK_LEG[0], 0.0, HOOK_FOOT[2]])   # the hooks' inner corner, bar frame (y aside)
SERVO_GAIN = 0.25             # the share of the bar's error taken out per control step
SERVO_STEP = 0.003            # m: the most a hand's command shifts in one step
SERVO_TURN = 0.4              # degrees: the most the grips' tilt shifts in one step


def pitch_deg(st):
    """The bar's pitch, degrees, near end up."""
    r = st.rotation()
    return float(math.degrees(math.atan2(r[0, 2], r[0, 0])))


def from_corner(cx, cz, th):
    """The bar's origin (doorway frame, x and z) when its hooks' inner corners are at (cx, cz) and it is pitched th."""
    c = _ry(th) @ CORNER
    return cx - c[0], cz - c[2]


def demo_script(ctx):
    import demokit as d

    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "holdon"):
        raise ValueError(mode)
    if mode == "none":
        yield from ctx.until(lambda: False, timeout=20.0, label="do nothing")
        return
    sides = [(ctx.left, 1, -90.0)] + ([] if mode == "onearm" else [(ctx.right, -1, 90.0)])
    hold = {s: np.array([PAD_BAR[0], s * GRASP_Y, PAD_BAR[1]]) for _, s, _ in sides}

    def bar():
        return ctx.state("bar")

    st0 = bar()
    r0 = st0.rotation()
    # each grip, in the bar's frame: tilted GRASP_BETA from straight down, fingers closing along the doorway's x
    grip_in_bar = {s: r0.T @ _ry(GRASP_BETA) @ d.down(yaw).rot for _, s, yaw in sides}

    def tcp(s, origin, rot):
        return origin + rot @ hold[s], rot @ grip_in_bar[s]

    # -- take the pad bar
    for arm, s, _ in sides:
        arm.grip = d._as_grip(r0 @ grip_in_bar[s])
    approach = {s: r0 @ grip_in_bar[s] @ np.array([0.0, 0.0, 1.0]) for _, s, _ in sides}
    yield from ctx.together(*[a.move(tcp(s, st0.pos, r0)[0] - 0.07 * approach[s], grip=d._as_grip(tcp(s, st0.pos, r0)[1]),
                                     finger=FINGER_OPEN, t=2.5, label="over the pad bar") for a, s, _ in sides])
    yield from ctx.together(*[a.move(tcp(s, bar().pos, bar().rotation())[0], t=1.5, label="round the pad bar") for a, s, _ in sides])
    yield from ctx.together(*[a.servo(tcp(s, bar().pos, bar().rotation())[0], timeout=1.0, label="round the pad bar") for a, s, _ in sides])
    yield from ctx.together(*[a.close(None, t=0.8, label="grip") for a, s, _ in sides])
    for arm, _, _ in sides:
        arm.trim = np.zeros(3)   # the steering below keeps its own account of the arms' give

    # -- steer the bar
    trim = {s: np.zeros(3) for _, s, _ in sides}
    turn = [0.0]

    def steer(path, t, settle=0.0, integrate=True, label="steer"):
        """Follow `path(u)` -> (dx, dz, pitch) in the doorway's frame for u from 0 to 1 over `t` s, then hold `settle` s."""
        n = max(1, round(t * 20))
        for k in range(n + round(settle * 20)):
            dx, dz, th = path(ik.smoothstep(min(1.0, (k + 1) / n)))
            origin = DOOR_O + np.array([dx, 0.0, dz])
            rot = _ry(th)
            st = bar()
            if integrate:
                for _, s, _ in sides:
                    err = (origin + rot @ hold[s]) - st.to_world(hold[s][None])[0]
                    trim[s] = trim[s] + np.clip(SERVO_GAIN * err, -SERVO_STEP, SERVO_STEP)
                turn[0] += float(np.clip(SERVO_GAIN * (th - pitch_deg(st)), -SERVO_TURN, SERVO_TURN))
            for arm, s, _ in sides:
                p, r = tcp(s, origin, rot)
                arm._solve(p + trim[s], _ry(turn[0]) @ r)
            yield
        hk = [hook_reading(bar(), s) for s in (1, -1)]
        print(f"[demo] {label}: pitch {pitch_deg(bar()):.2f}, hooks (dx, dz, over) mm "
              + " ".join(f"({a * 1000:.1f}, {b * 1000:.1f}, {c * 1000:.1f})" for a, b, c in hk)
              + f", trims {[np.round(trim[s] * 1000, 1).tolist() for _, s, _ in sides]} mm, turn {turn[0]:.1f}", flush=True)

    def line(a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        return lambda u: tuple(a + u * (b - a))

    st = bar()
    p0 = (float(st.pos[0] - DOOR_O[0]), float(st.pos[2] - DOOR_O[2]), pitch_deg(st))
    th0 = p0[2]
    lifted = (p0[0], p0[1] + 0.02, th0)
    yield from steer(line(p0, lifted), 1.5, settle=0.5, label="lift off the table")
    if mode == "careless":
        # straight up from where it lies: the hook feet are still under the far casing
        up = (p0[0], from_corner(0.0, CASE_TOP + 0.015, th0)[1], th0)
        yield from steer(line(lifted, up), 4.0, settle=1.0, label="straight up")
    else:
        through = (0.012, lifted[1], th0)
        yield from steer(line(lifted, through), 2.0, settle=0.5, label="hooks through, clear of the far casing")
        cx, cz = through[0] + (_ry(th0) @ CORNER)[0], CASE_TOP + 0.015
        raised = (*from_corner(cx, cz, th0), th0)
        yield from steer(line(through, raised), 4.0, settle=0.5, label="raise the hooks over the casing")

        def corner_path(c0, c1):
            c0, c1 = np.asarray(c0, float), np.asarray(c1, float)

            def f(u):
                cx_, cz_, th_ = c0 + u * (c1 - c0)
                return (*from_corner(cx_, cz_, th_), th_)
            return f

        yield from steer(corner_path((cx, cz, th0), (0.0075, cz, 3.0)), 3.0, settle=0.5, label="turn down over the casing")
        # the feet set down 2 mm into the casing's top (the hands press the hooks onto it) and the bar half a degree short
        # of level, so that the pads come within 2 mm of the trim and close the last of it by themselves once let go
        yield from steer(corner_path((0.0075, cz, 3.0), (0.0005, CASE_TOP - 0.002, 0.5)), 3.0, settle=1.0, label="onto the casing, pads to the trim")
    if mode == "holdon":
        yield from ctx.until(lambda: False, timeout=8.0, label="keep holding")
        return
    # let the doorway take the weight: ease the hands' extra push away, holding the bar where it is
    st = bar()
    now = (float(st.pos[0] - DOOR_O[0]), float(st.pos[2] - DOOR_O[2]), pitch_deg(st))
    origin, rot = DOOR_O + np.array([now[0], 0.0, now[1]]), _ry(now[2])
    n = 30
    for k in range(n):
        u = 1.0 - (k + 1) / n
        for arm, s, _ in sides:
            p, r = tcp(s, origin, rot)
            arm._solve(p + u * trim[s], _ry(u * turn[0]) @ r)
        yield
    for arm, s, _ in sides:
        arm.grip = d._as_grip(tcp(s, origin, rot)[1])
    yield from ctx.together(*[a.open(t=0.6, label="let go") for a, s, _ in sides])
    yield from ctx.together(*[a.move(a.at() - 0.08 * a.approach_axis(), t=1.2, label="back off") for a, s, _ in sides])
    yield from ctx.together(*[a.move(a.at() + np.array([-0.10, 0.0, 0.05]), t=1.5, label="away") for a, s, _ in sides])
    yield from ctx.until(lambda: False, timeout=5.0, label="hands clear")

