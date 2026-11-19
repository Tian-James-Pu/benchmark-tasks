"""On the station table (a workbench): two parts of a small flat-pack shelf unit, lying flat, ready to be joined on
wooden dowels as in a flat-pack assembly step.

- The shelf: white melamine-faced chipboard, 18 mm thick, 160 mm along its front edge (world y) and 200 mm deep (world
  x), 0.38 kg. Its near edge (facing the arms, at x 0.270) carries two fluted beech dowels, 8 mm across and 30 mm long,
  pressed 16 mm into holes in the edge, so 14 mm of each stands out towards the arms. They are 96 mm apart (y -48 and
  +48 mm), at mid-thickness (9 mm above the bench), each with a 1 mm chamfer at its tip.
- The rail: planed pine, 18 x 44 x 300 mm, 0.12 kg, lying flat in front of the shelf with its long axis along y, turned
  4 deg and 10 mm to the left of the shelf's centre line. Its far long face has two round blind holes, 8.2 mm across and
  16 mm deep, 96 mm apart at mid-thickness: they take the shelf's dowels when the rail is pushed onto them.
- Around them, ignored by scoring: a Phillips screwdriver and a mug, left of and right of the shelf, and a utility knife
  beyond it.
"""

import json
import math
import os
import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- dimensions (metres, kilograms) -------------------------------------------------------------------------------------

T = 0.018  # both boards' thickness
SHELF_W, SHELF_D = 0.160, 0.200  # along the dowel edge (local y), away from it (local x)
SHELF_MASS = 0.38  # 0.018 x 0.16 x 0.20 m at 655 kg/m^3 (EGGER Eurodekor EPD)
RAIL_W, RAIL_L = 0.044, 0.300  # across (local x), along (local y)
RAIL_MASS = 0.12  # 0.018 x 0.044 x 0.30 m at 510 kg/m^3 (Scots pine)
DOWEL_R = 0.004  # 8 mm dowel
DOWEL_LEN = 0.030
DOWEL_OUT = 0.014  # standing out of the shelf's edge (30 mm dowel, 16 mm deep hole)
CHAMFER = 0.001
HOLE_R = 0.0041  # 8.2 mm hole: 0.2 mm over the dowel's nominal diameter
HOLE_DEPTH = 0.016
DOWEL_Y = (-0.048, 0.048)  # 96 mm apart (three 32 mm steps), 32 mm in from the shelf's corners
AXIS_Z = T / 2  # dowels and holes at mid-thickness
FACE_X = RAIL_W / 2  # the rail's holed face, in its own frame

# Rail frame: origin at the middle of its underside, x across its width towards the holed face, y along it, z up.
# Shelf frame: origin at the middle of the dowelled edge, on the bench; x into the shelf, y along the edge, z up.
SHELF_POS = (0.270, 0.0, TOP + 0.0005)
RAIL_POS = (0.200, 0.010, TOP + 0.0005)
RAIL_YAW = 4.0  # degrees

LOOKS = {
    "melamine": m.Look(m.rgb(0.93, 0.93, 0.91), roughness=0.35),
    "edge": m.Look(m.rgb(0.86, 0.86, 0.84), roughness=0.5),
    "chip": m.Look(m.rgb(0.70, 0.60, 0.45), roughness=0.9),
    "beech": m.Look(m.rgb(0.84, 0.68, 0.48), roughness=0.75),
    "pine": m.Look(m.rgb(0.86, 0.72, 0.50), roughness=0.7),
    "grain": m.Look(m.rgb(0.78, 0.62, 0.40), roughness=0.7),
    "hole": m.Look(m.rgb(0.20, 0.14, 0.08), roughness=0.95),
}


def _prism_x(points_yz, x0, x1):
    """A convex prism along x, from a convex outline in (y, z) (counter-clockwise), as a mesh."""
    pts = [tuple(p) for p in points_yz]
    n = len(pts)
    v = [(x0, y, z) for y, z in pts] + [(x1, y, z) for y, z in pts]
    f = [(0, i + 1, i) for i in range(1, n - 1)] + [(n, n + i, n + i + 1) for i in range(1, n - 1)]
    for i in range(n):
        j = (i + 1) % n
        f += [(i, j, n + j), (i, n + j, n + i)]
    return m.Mesh(np.array(v, float), np.array(f, int))


# -- the shelf, with its two dowels --------------------------------------------------------------------------------------


def _dowel_visual(y):
    """A fluted dowel standing out of the edge along -x: 12 shallow flutes, and the chamfered tip."""
    outline = []
    for k in range(48):
        a = 2 * math.pi * k / 48
        r = DOWEL_R - (0.00035 if k % 4 == 0 else 0.0)
        outline.append((r * math.cos(a), r * math.sin(a)))
    body = m.prism(outline, 0.0, DOWEL_OUT + 0.001 - CHAMFER)
    tip = m.lathe([(0.0, DOWEL_OUT + 0.001 - CHAMFER - 0.0002), (DOWEL_R, DOWEL_OUT + 0.001 - CHAMFER - 0.0002),
                   (DOWEL_R - CHAMFER, DOWEL_OUT + 0.001), (0.0, DOWEL_OUT + 0.001)], segments=32)
    return [m.Mesh(mm.aimed("-X").v + np.array([0.001, y, AXIS_Z]), mm.f) for mm in (body, tip)]


def _dowel_collider(y):
    """The dowel's standing part as one convex hull (round, 32 sides), its tip chamfered 1 mm."""
    prof = [(0.0, 0.0), (DOWEL_R, 0.0), (DOWEL_R, DOWEL_OUT + 0.001 - CHAMFER), (DOWEL_R - CHAMFER, DOWEL_OUT + 0.001), (0.0, DOWEL_OUT + 0.001)]
    mesh = m.lathe(prof, segments=32).aimed("-X")
    return m.Mesh(mesh.v + np.array([0.001, y, AXIS_Z]), mesh.f)


def _shelf():
    parts = [
        ("core", m.box((SHELF_D - 0.001, SHELF_W - 0.001, T - 0.001), (SHELF_D / 2, 0.0, T / 2)), "chip"),
        ("top", m.box((SHELF_D, SHELF_W, 0.0006), (SHELF_D / 2, 0.0, T - 0.0003)), "melamine"),
        ("bottom", m.box((SHELF_D, SHELF_W, 0.0006), (SHELF_D / 2, 0.0, 0.0003)), "melamine"),
        ("band_front", m.box((0.0006, SHELF_W, T), (0.0003, 0.0, T / 2)), "edge"),
        ("band_left", m.box((SHELF_D, 0.0006, T), (SHELF_D / 2, SHELF_W / 2 - 0.0003, T / 2)), "edge"),
        ("band_right", m.box((SHELF_D, 0.0006, T), (SHELF_D / 2, -SHELF_W / 2 + 0.0003, T / 2)), "edge"),
    ]
    cols = [m.col_box("board_c", (SHELF_D, SHELF_W, T), (SHELF_D / 2, 0.0, T / 2))]
    for i, y in enumerate(DOWEL_Y):
        body, tip = _dowel_visual(y)
        parts += [(f"dowel{i}", body, "beech"), (f"dowel{i}_tip", tip, "beech")]
        cols.append(m.col_hull(f"dowel{i}_c", _dowel_collider(y)))
    return parts, cols


# -- the rail, with its two blind holes ----------------------------------------------------------------------------------

BORE_SIDES = 16
CHANNEL = 0.006  # half-width of the square channel around each hole that the board's own boxes leave open
RING_OUT = 0.0060  # the ring pieces reach out to here from the hole's axis, filling the channel's corners


def _rail():
    x0, x1 = FACE_X - HOLE_DEPTH, FACE_X  # the holes run from the face (x1) to their flat bottoms (x0)
    cols = [m.col_box("back_c", (x0 + RAIL_W / 2, RAIL_L, T), ((x0 - RAIL_W / 2) / 2, 0.0, T / 2))]
    # the face's strip between and beyond the holes
    edges = [-RAIL_L / 2] + [e for y in DOWEL_Y for e in (y - CHANNEL, y + CHANNEL)] + [RAIL_L / 2]
    for k in range(0, len(edges), 2):
        a, b = edges[k], edges[k + 1]
        cols.append(m.col_box(f"face{k // 2}_c", (x1 - x0, b - a, T), ((x0 + x1) / 2, (a + b) / 2, T / 2)))
    rv = HOLE_R / math.cos(math.pi / BORE_SIDES)  # the bore's polygon has HOLE_R as its inscribed radius
    for i, yc in enumerate(DOWEL_Y):
        lo, hi = AXIS_Z - HOLE_R, AXIS_Z + HOLE_R
        cols += [
            m.col_box(f"h{i}_below_c", (x1 - x0, 2 * CHANNEL, lo), ((x0 + x1) / 2, yc, lo / 2)),
            m.col_box(f"h{i}_above_c", (x1 - x0, 2 * CHANNEL, T - hi), ((x0 + x1) / 2, yc, (hi + T) / 2)),
            m.col_box(f"h{i}_right_c", (x1 - x0, CHANNEL - HOLE_R, hi - lo), ((x0 + x1) / 2, yc - (CHANNEL + HOLE_R) / 2, AXIS_Z)),
            m.col_box(f"h{i}_left_c", (x1 - x0, CHANNEL - HOLE_R, hi - lo), ((x0 + x1) / 2, yc + (CHANNEL + HOLE_R) / 2, AXIS_Z)),
        ]
        for s in range(BORE_SIDES):  # ring pieces round the bore: together they make it round (16 flats)
            a0 = 2 * math.pi * (s - 0.5) / BORE_SIDES - math.pi / 2
            a1 = 2 * math.pi * (s + 0.5) / BORE_SIDES - math.pi / 2
            quad = [(yc + rv * math.cos(a0), AXIS_Z + rv * math.sin(a0)), (yc + RING_OUT * math.cos(a0), AXIS_Z + RING_OUT * math.sin(a0)),
                    (yc + RING_OUT * math.cos(a1), AXIS_Z + RING_OUT * math.sin(a1)), (yc + rv * math.cos(a1), AXIS_Z + rv * math.sin(a1))]
            cols.append(m.col_hull(f"h{i}_ring{s}_c", _prism_x(quad, x0, x1)))
    parts = [("board", m.box((RAIL_W, RAIL_L, T), (0.0, 0.0, T / 2)), "pine")]
    for k, y in enumerate((-0.11, -0.03, 0.06, 0.12)):  # a few grain streaks on the top face
        parts.append((f"grain{k}", m.box((0.0015, 0.08, 0.0004), (-0.012 + 0.008 * k, y, T + 0.0001)), "grain"))
    for i, yc in enumerate(DOWEL_Y):
        disc = m.cylinder(HOLE_R, 0.0004, (0.0, 0.0, 0.0), segments=32).aimed("+X")
        parts.append((f"hole{i}", m.Mesh(disc.v + np.array([FACE_X + 0.0002, yc, AXIS_Z]), disc.f), "hole"))
    return parts, cols


SHELF_PARTS, SHELF_COLS = _shelf()
RAIL_PARTS, RAIL_COLS = _rail()

OBJECTS = [
    m.free("shelf", SHELF_POS, SHELF_PARTS, SHELF_COLS, LOOKS, mass=SHELF_MASS, friction=(0.5, 0.4), solver_iters=48),
    m.free("rail", RAIL_POS, RAIL_PARTS, RAIL_COLS, LOOKS, mass=RAIL_MASS, friction=(0.5, 0.4), solver_iters=48, yaw_deg=RAIL_YAW),
    # Distractors (SimReady props), ignored by scoring. pos is each prop's origin, the middle of its underside.
    SimReady("screwdriver", pos=(0.36, 0.215, TOP), size=(0.188, 0.034, 0.032), mass=0.07, yaw=0.25,
             package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd"),
    SimReady("mug", pos=(0.40, -0.215, TOP), size=(0.114, 0.090, 0.080), mass=0.30, yaw=-1.2,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("knife", pos=(0.545, 0.03, TOP), size=(0.035, 0.184, 0.009), mass=0.10, yaw=0.1,
             package="handTool_knife_foldingUtility_a01", usd="sm_handTool_knife_foldingUtility_a01_01.usd"),
]

# -- scoring -------------------------------------------------------------------------------------------------------------

SUCCESS_HOLD_S = 0.0  # the tracker below counts its own 3 s hold
SCORE_OVER = "end"  # only the final state counts
HOLD_S = 3.0
FLAT_DEG = 3.0  # each board lies flat
BENCH_TOL = 0.002  # each board's mid-thickness within 2 mm of 9 mm above the bench: on the bench, not on the other
IN_RADIAL = 0.001  # a dowel's tip within 1 mm of a hole's axis ...
IN_DEPTH = 0.002  # ... and at least 2 mm past the rail's face: it is in that hole
TIPS_GAP = DOWEL_OUT - IN_DEPTH  # 12 mm: both dowels started in their holes
HALF_GAP = 0.007  # half of each dowel's 14 mm in
SEAT_GAP = 0.0005  # closed: a 0.5 mm feeler does not enter anywhere along the seam
OVERLAP = -0.0005  # a seam gap below this is not a real seam (boards overlapping)
CLEAR = 0.03  # every gripper point this far from both boards
STILL_V, STILL_W = 0.005, 0.05  # m/s, rad/s


def _upright_tilt(st):
    t = st.tilt_deg()
    return min(t, 180.0 - t)  # a board may lie on either face


def measure(shelf, rail):
    """The joint, from the two boards' poses: per dowel, its depth in the nearest hole, the tip's distance from that
    hole's axis, and the seam gap at the dowel; the seam gap at both ends of the shelf's edge; flat and on the bench."""
    on_bench = {}
    for name, st, centre in (("shelf", shelf, (SHELF_D / 2, 0.0, T / 2)), ("rail", rail, (0.0, 0.0, T / 2))):
        z = float(st.to_world(np.array(centre))[0, 2]) - TOP
        on_bench[name] = bool(abs(z - T / 2) <= BENCH_TOL and _upright_tilt(st) <= FLAT_DEG)
    holes = np.array([(y, AXIS_Z) for y in DOWEL_Y])
    dowels = []
    for y in DOWEL_Y:
        root = rail.to_local(shelf.to_world(np.array([0.0, y, AXIS_Z])))[0]
        tip = rail.to_local(shelf.to_world(np.array([-DOWEL_OUT, y, AXIS_Z])))[0]
        radial = float(np.min(np.hypot(tip[1] - holes[:, 0], tip[2] - holes[:, 1])))
        depth = float(FACE_X - tip[0])
        gap = float(root[0] - FACE_X)
        inside = bool(depth >= IN_DEPTH and radial <= IN_RADIAL and gap >= OVERLAP)
        dowels.append({"depth_mm": round(depth * 1000, 2), "radial_mm": round(radial * 1000, 2), "gap_mm": round(gap * 1000, 2), "in": inside})
    corners = [float(rail.to_local(shelf.to_world(np.array([0.0, s * SHELF_W / 2, AXIS_Z])))[0, 0] - FACE_X) for s in (-1, 1)]
    return {"on_bench": on_bench, "dowels": dowels, "corner_gaps_mm": [round(c * 1000, 2) for c in corners]}


def rung(r):
    """The score of the joint as it is now, not counting the release (0, 0.25, 0.5 or 0.75)."""
    if not all(r["on_bench"].values()) or not all(d["in"] for d in r["dowels"]):
        return 0.0
    gaps = [d["gap_mm"] / 1000 for d in r["dowels"]]
    corners = [c / 1000 for c in r["corner_gaps_mm"]]
    if max(gaps + corners) <= SEAT_GAP and min(corners) >= OVERLAP:
        return 0.75
    if max(gaps) <= HALF_GAP:
        return 0.5
    if max(gaps) <= TIPS_GAP:
        return 0.25
    return 0.0


def clear_of(st, tips, lo, hi):
    """Smallest distance (m) from the gripper points to a board's box (its own frame, corners lo and hi)."""
    local = st.to_local(tips)
    out = np.maximum(np.maximum(np.asarray(lo) - local, local - np.asarray(hi)), 0.0)
    return float(np.min(np.linalg.norm(out, axis=1)))


def _pose(st):
    """x, y, z (mm) of a board's origin and its yaw (deg), for the run log."""
    r = st.rotation()
    return [round(float(v) * 1000, 2) for v in st.pos] + [round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 3)]


SHELF_BOX = ((-DOWEL_OUT, -SHELF_W / 2, 0.0), (SHELF_D, SHELF_W / 2, T))
RAIL_BOX = ((-RAIL_W / 2, -RAIL_L / 2, 0.0), (RAIL_W / 2, RAIL_L / 2, T))


class Trial:
    """Updated once per control step. Only the final state counts; the 3 s hold is the only thing carried over steps,
    and it restarts whenever a 1.0 condition fails. Cleared only at a reset (on_reset)."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.hold, self.full, self.value, self.last = 0.0, False, 0.0, {}

    def update(self, shelf, rail, tips, dt):
        r = measure(shelf, rail)
        base = rung(r)
        gap = min(clear_of(shelf, tips, *SHELF_BOX), clear_of(rail, tips, *RAIL_BOX)) if tips is not None else 0.0
        still = all(float(np.linalg.norm(s.lin_vel)) < STILL_V and float(np.linalg.norm(s.ang_vel)) < STILL_W for s in (shelf, rail))
        ready = base == 0.75 and gap > CLEAR and still
        self.hold = self.hold + float(dt) if ready else 0.0
        self.full = bool(ready and self.hold >= HOLD_S - 1e-9)
        self.value = 1.0 if self.full else base
        self.last = {**r, "rung": base, "hands_clear_mm": round(gap * 1000, 1), "still": still, "hold_s": round(self.hold, 2), "score": self.value,
                     "shelf": _pose(shelf), "rail": _pose(rail)}
        if tips is not None:
            self.last["hands"] = [[round(float(v) * 1000, 1) for v in tips[k:k + 12].mean(axis=0)] for k in (0, 12)]
        return self.value

    def step(self, environment):
        self.update(m.state(environment, "shelf"), m.state(environment, "rail"), m.fingertips(environment), environment.unwrapped.step_dt)
        self.steps = getattr(self, "steps", 0) + 1
        if self.steps % 5 == 0 or self.full:
            print("[dowels] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "step": self.steps, **self.last}), flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo (open loop, from the published starting layout) ---------------------------------------------------

PAD_TIP = 0.013  # the pads' tips below the TCP
GRIP_Z = TOP + PAD_TIP + 0.004  # pads' tips 4 mm above the bench, across the rail's 18 mm thickness
LIFT = 0.004  # carry the rail this far above the bench
GRIP_ALONG = 0.12  # each hand takes the rail this far from its middle, outside the shelf's 160 mm edge
APPROACH = 0.006  # the holed face stops this far short of the dowels' tips, square to them
OVERPUSH = 0.0015  # the arms push this far past the closed seam
UP = 0.08


def _rail_point(origin_xy, yaw_deg, along, z):
    a = math.radians(yaw_deg)
    return (origin_xy[0] - along * math.sin(a), origin_xy[1] + along * math.cos(a), z)


def plan(start_xy=RAIL_POS[:2], start_yaw=RAIL_YAW, shelf_xy=SHELF_POS[:2], arms=("left", "right"), push=True, release=True, square=True, stop_short=0.0):
    """Waypoints per arm. The left arm takes the rail's +y end, the right arm its -y end, pads closing across its width.
    They lift it 4 mm, turn it square to the shelf with its holes on the dowels' axes, set it down 6 mm short of the
    dowels' tips, and push it on until the seam closes; then both let go and rise."""
    sx, sy = shelf_xy
    pre = (sx - DOWEL_OUT - APPROACH - FACE_X, sy)
    seat = (sx - FACE_X + OVERPUSH - stop_short, sy)
    yaw_end = 0.0 if square else 4.0
    out = {}
    for arm in arms:
        along = GRIP_ALONG if arm == "left" else -GRIP_ALONG
        base_yaw = -math.pi / 2 if arm == "left" else math.pi / 2  # fingers closing along the rail's width; the other sign is past a wrist limit
        y0 = base_yaw + math.radians(start_yaw)
        y1 = base_yaw + math.radians(yaw_end)
        g0 = _rail_point(start_xy, start_yaw, along, GRIP_Z)
        g1 = _rail_point(pre, yaw_end, along, GRIP_Z)
        g2 = _rail_point(seat, yaw_end, along, GRIP_Z)
        up = np.array([0.0, 0.0, UP])
        lift = np.array([0.0, 0.0, LIFT])
        wps = [
            Waypoint(2.5, tuple(np.array(g0) + up), FINGER_OPEN, "above the rail's end", y0),
            Waypoint(1.5, g0, FINGER_OPEN, "pads either side of the rail", y0),
            Waypoint(0.8, g0, FINGER_CLOSED, "grip", y0),
            Waypoint(0.8, tuple(np.array(g0) + lift), FINGER_CLOSED, "lift 4 mm", y0),
            Waypoint(2.5, tuple(np.array(g1) + lift), FINGER_CLOSED, "square to the shelf, holes on the dowels' axes", y1),
            Waypoint(0.8, g1, FINGER_CLOSED, "set it down on the bench", y1),
        ]
        if push:
            wps += [Waypoint(4.0, g2, FINGER_CLOSED, "push it onto the dowels", y1), Waypoint(0.6, g2, FINGER_CLOSED, "hold", y1)]
        end = g2 if push else g1
        if release:
            wps += [
                Waypoint(0.8, end, FINGER_OPEN, "let go", y1),
                Waypoint(1.5, tuple(np.array(end) + up), FINGER_OPEN, "rise", y1),
                Waypoint(1.5, tuple(np.array(end) + up + np.array([-0.06, 0.0, 0.04])), FINGER_OPEN, "back away", y1),
            ]
        out[arm] = wps
    return out


def demo():
    return plan()
