"""Roll five dice out of a dice cup into a dice tray, then put the cup down.

On the table: a felt-lined dice cup standing upright with five dice in it, and a velvet-lined folding dice tray. A
score pad, a mug and a glass stand nearby (distractors, not part of the task). The robot rolls the dice into the tray,
the way each Yahtzee turn starts, and puts the cup back down on the table beside the tray.

Model (declared in code):
- Dice ("die1" to "die5"): opaque 16 mm acrylic dice in five unique colors, with 1.5 mm chamfered edges and corners
  (one convex collider each), 4.5 g, white pips drawn on (they do not collide).
- Cup ("cup"): one rigid body, 83 mm across, 95 mm tall, a 3 mm wall (24 flat panels) and a 4 mm base, 80 g.
- Tray ("tray"): fixed (it lies flat on the table and is not moved), velvet floor and four padded walls.
- Restitution (bounciness) is set per part: dice 0.35, tray 0.2, cup 0.1 (estimates; PhysX averages the two).

Scoring (`Trial` below, every control step at 20 Hz):
- A die has *left the cup* the first step its centre is outside the cup's inside (above the rim plane, or outside
  the wall). At that step its upper-facing number is recorded for that colored die.
- A die is *touched* if a gripper finger ever comes within 6 mm of its faces (finger sample points, mechkit
  `fingertips`; the points sit 4.5 mm behind the pad surface). Touched is recorded for the trial.
- A die *counts* when: its upper-facing number changed at least once after it left the cup; it was never touched; and now
  it rests flat on the tray floor (lowest point within 3 mm of the floor, one face within 8 degrees of level, centre
  inside the tray's walls), still (under 1 cm/s and 0.3 rad/s).
- The cup is *set down* when it stands upright (axis within 10 degrees of vertical) with its base within 3 mm of the
  table, its footprint wholly outside the tray's outline, every finger point 3 cm or more from it, and it is still.
- Score: 0 dice count -> 0; 1 or 2 -> 0.25; 3 or 4 -> 0.5; all 5 -> 0.75; all 5 and the cup set down -> 1.0
  (success; the trial ends once it has held for SUCCESS_HOLD_S).
The per-die records are cleared only in `on_reset`.
"""

from __future__ import annotations

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
from yam_arena import ik, station  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, READY, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 3.0
DEBUG = os.environ.get("DICE_DEBUG", "") not in ("", "0")

# -- restitution per part (mechkit writes 0 for every part; this sets the value in each part's physics material) ------
_REST = {"P_die": 0.35, "P_tray": 0.2, "P_cup": 0.1}
_file0 = getattr(m._file, "_plain", m._file)


def _file_rest(root, head_api, attrs, body, looks, friction):
    text = _file0(root, head_api, attrs, body, looks, friction)
    for prefix, e in _REST.items():
        if root.startswith(prefix):
            text = text.replace("float physics:restitution = 0\n", f"float physics:restitution = {e}\n", 1)
    return text


_file_rest._plain = _file0
m._file = _file_rest

LOOKS = {
    "pip": m.Look(color=m.rgb(0.97, 0.97, 0.93), roughness=0.4),
    "green": m.Look(color=m.rgb(0.04, 0.55, 0.16), roughness=0.25),
    "pink": m.Look(color=m.rgb(0.76, 0.12, 0.40), roughness=0.25),
    "black": m.Look(color=m.rgb(0.035, 0.04, 0.05), roughness=0.25),
    "blue": m.Look(color=m.rgb(0.04, 0.18, 0.76), roughness=0.25),
    "red": m.Look(color=m.rgb(0.78, 0.045, 0.035), roughness=0.25),
    "leather": m.Look(color=m.rgb(0.07, 0.06, 0.06), roughness=0.55),
    "felt": m.Look(color=m.rgb(0.55, 0.06, 0.07), roughness=0.95),
    "velvet": m.Look(color=m.rgb(0.07, 0.10, 0.28), roughness=0.95),
    "pu": m.Look(color=m.rgb(0.10, 0.09, 0.09), roughness=0.5),
}

# -- the tray: a folding velvet dice tray, snapped up (outside 22.9 x 15.2 cm, walls 3.2 cm), lying flat ---------------
TRAY_C = np.array([0.285, 0.065])
TRAY_OUT = (0.152, 0.229)  # outside along x, y (m)
TRAY_WALL_T = 0.006
TRAY_H = 0.032  # wall height above the table
TRAY_FLOOR_T = 0.004  # floor (padding and velvet) thickness
FLOOR_Z = TOP + TRAY_FLOOR_T  # the tray floor's top
TRAY_IN = (TRAY_OUT[0] - 2 * TRAY_WALL_T, TRAY_OUT[1] - 2 * TRAY_WALL_T)


def _tray() -> m.Part:
    ox, oy = TRAY_OUT
    ix, iy = TRAY_IN
    t, h, f = TRAY_WALL_T, TRAY_H, TRAY_FLOOR_T
    pieces = {
        "floor": ((ix, iy, f), (0, 0, f / 2), "velvet"),
        "wall_xn": ((t, oy, h), (-(ox - t) / 2, 0, h / 2), "pu"),
        "wall_xp": ((t, oy, h), ((ox - t) / 2, 0, h / 2), "pu"),
        "wall_yn": ((ix, t, h), (0, -(oy - t) / 2, h / 2), "pu"),
        "wall_yp": ((ix, t, h), (0, (oy - t) / 2, h / 2), "pu"),
    }
    parts = [(k, m.box(s, c), look) for k, (s, c, look) in pieces.items()]
    cols = [m.col_box(f"{k}_c", s, c) for k, (s, c, _) in pieces.items()]
    # the floor collider goes 6 mm below its top surface so a fast die cannot pass through it
    cols[0] = m.col_box("floor_c", (ix, iy, 0.006), (0, 0, f - 0.003))
    return m.fixed("tray", (TRAY_C[0], TRAY_C[1], TOP), parts, cols, LOOKS, friction=(0.5, 0.45))


# -- the dice cup: felt-lined leather, 3 1/4 in (83 mm) across, 3 3/4 in (95 mm) tall ------------------------------
CUP_R = 0.0415
CUP_H = 0.095
CUP_WALL = 0.003
CUP_BASE = 0.004
CUP_RI = CUP_R - CUP_WALL
CUP_MASS = 0.080
CUP_POS = np.array([0.300, -0.170])
N_PANELS = 24


def _cup() -> m.Part:
    shell = m.lathe([(0.0, 0.0), (CUP_R, 0.0), (CUP_R, CUP_H), (CUP_RI, CUP_H), (CUP_RI, CUP_BASE), (0.0, CUP_BASE)], segments=48)
    felt = m.join(m.cylinder(CUP_RI - 0.0004, 0.0006, (0, 0, CUP_BASE + 0.0003), segments=48))
    cols = [m.col_cylinder("base_c", CUP_R, CUP_BASE, (0, 0, CUP_BASE / 2))]
    w = 2 * CUP_R * math.tan(math.pi / N_PANELS) + 0.001
    for k in range(N_PANELS):
        panel = m.box((CUP_WALL, w, CUP_H - CUP_BASE), (CUP_R - CUP_WALL / 2, 0, CUP_BASE + (CUP_H - CUP_BASE) / 2))
        cols.append(m.col_hull(f"wall{k:02d}_c", panel.turned(360.0 * k / N_PANELS, "z")))
    parts = [("shell", shell, "leather"), ("felt", felt, "felt")]
    return m.free("cup", (CUP_POS[0], CUP_POS[1], TOP), parts, cols, LOOKS, mass=CUP_MASS, friction=(0.8, 0.7), torsional_patch=0.01)


# -- the dice: 16 mm acrylic, 1.5 mm chamfers, 4.5 g; 1 opposite 6, 2 opposite 5, 3 opposite 4 ---------------------------
DIE = 0.016
DIE_CH = 0.0015
DIE_MASS = 0.0045
_h, _c = DIE / 2, DIE / 2 - DIE_CH
DIE_PTS = np.array([(sx * a, sy * b, sz * c) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
                    for (a, b, c) in ((_h, _c, _c), (_c, _h, _c), (_c, _c, _h))])  # 24 corners of the chamfered cube


def _hull_faces(pts: np.ndarray) -> np.ndarray:
    """Outward triangles of the convex hull of `pts` (brute force: few points)."""
    n, faces, cen = len(pts), [], pts.mean(axis=0)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nrm = np.cross(pts[j] - pts[i], pts[k] - pts[i])
                if np.linalg.norm(nrm) < 1e-12:
                    continue
                d = (pts - pts[i]) @ nrm
                if np.all(d <= 1e-9):
                    faces.append((i, j, k))
                elif np.all(d >= -1e-9):
                    faces.append((i, k, j))
    # keep one triangulation per face plane: drop triangles that are not part of a fan from the plane's first vertex
    out, seen = [], {}
    for f in faces:
        nrm = np.cross(pts[f[1]] - pts[f[0]], pts[f[2]] - pts[f[0]])
        nrm = nrm / np.linalg.norm(nrm)
        key = tuple(np.round(nrm, 4))
        seen.setdefault(key, []).append(f)
    for key, fs in seen.items():
        verts = sorted({v for f in fs for v in f})
        nrm = np.array(key)
        c = pts[verts].mean(axis=0)
        u = pts[verts[0]] - c
        u = u / np.linalg.norm(u)
        v = np.cross(nrm, u)
        ang = sorted(verts, key=lambda q: math.atan2((pts[q] - c) @ v, (pts[q] - c) @ u))
        for q in range(1, len(ang) - 1):
            out.append((ang[0], ang[q], ang[q + 1]))
    del cen
    return np.array(out, dtype=int)


DIE_MESH = m.Mesh(DIE_PTS.copy(), _hull_faces(DIE_PTS))
_PIP_UV = {1: [(0, 0)], 2: [(-1, -1), (1, 1)], 3: [(-1, -1), (0, 0), (1, 1)], 4: [(-1, -1), (-1, 1), (1, -1), (1, 1)],
           5: [(-1, -1), (-1, 1), (0, 0), (1, -1), (1, 1)], 6: [(-1, -1), (-1, 0), (-1, 1), (1, -1), (1, 0), (1, 1)]}
_FACE = {1: "+Z", 6: "-Z", 2: "+X", 5: "-X", 3: "+Y", 4: "-Y"}  # die frame


def _pips() -> m.Mesh:
    meshes = []
    for n, face in _FACE.items():
        for (u, v) in _PIP_UV[n]:
            r = 0.0017 if n != 1 else 0.0024
            pip = m.cylinder(r, 0.0004, (u * 0.0042, v * 0.0042, _h + 0.0001), segments=12)
            meshes.append(pip.aimed(face))
    return m.join(*meshes)


PIPS = _pips()
# Starting layout in the cup (cup frame, m): a 2 x 2 square with 3 mm gaps and a fifth die beside it.
# Each die starts with a different face up (turn in degrees about x, y, then yaw about z).
DIE_START = [((0.0095, 0.0095), (0, 0, 0)), ((-0.0095, 0.0095), (90, 0, 0)), ((0.0095, -0.0095), (0, 90, 0)),
             ((-0.0095, -0.0095), (180, 0, 0)), ((-0.028, 0.0), (-90, 0, 0))]
DIE_COLORS = ("green", "pink", "black", "blue", "red")


def _quat_from_mat(R: np.ndarray) -> tuple:
    q = ik.matrix_to_quat_xyzw(R)
    return tuple(float(v) for v in q)


def _rot(axis, deg: float) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


_FACE_BY_UP_AXIS = {(0, 1): 2, (0, -1): 5, (1, 1): 3, (1, -1): 4, (2, 1): 1, (2, -1): 6}


def _upper_face(R: np.ndarray) -> int:
    """Return the die face whose outward normal points most upward."""
    axis = int(np.argmax(np.abs(R[2, :])))
    sign = 1 if R[2, axis] >= 0 else -1
    return _FACE_BY_UP_AXIS[(axis, sign)]


def _die(k: int) -> m.Part:
    (dx, dy), (rx, ry, rz) = DIE_START[k]
    R = _rot((0, 0, 1), rz) @ _rot((0, 1, 0), ry) @ _rot((1, 0, 0), rx)
    pos = (CUP_POS[0] + dx, CUP_POS[1] + dy, TOP + CUP_BASE + _h + 0.0008)
    parts = [("body", DIE_MESH, DIE_COLORS[k]), ("pips", PIPS, "pip")]
    return m.free(f"die{k + 1}", pos, parts, [m.col_hull("body_c", DIE_MESH)], LOOKS, mass=DIE_MASS,
                  friction=(0.4, 0.35), torsional_patch=0.004, start_quat=_quat_from_mat(R))


DICE = [f"die{k + 1}" for k in range(5)]

# -- distractors (SimReady, CC-BY 4.0): a score pad, a mug and a glass ---------------------------------------------------
SCORE_PAD = SimReady("score_pad", pos=(0.17, 0.30, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01",
                     usd="sm_misc_notebook_a01_01.usd", yaw=math.pi / 2)
MUG = SimReady("mug", pos=(0.43, 0.25, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
               usd="sm_dishware_mug_a01_01.usda", yaw=math.radians(200))
GLASS = SimReady("glass", pos=(0.45, -0.17, TOP), size=(0.080, 0.080, 0.085), mass=0.25, package="dishware_glass_tumbler_a01",
                 usd="sm_dishware_glass_tumbler_a01_01.usd")

OBJECTS = [_tray(), _cup(), *[_die(k) for k in range(5)], SCORE_PAD, MUG, GLASS]

# -- scoring -----------------------------------------------------------------------------------------------------------
TOUCH = 0.006  # finger point within this of a die's faces = touched (m)
FLAT_GAP = 0.003  # lowest point of a die above the tray floor (m)
FLAT_DEG = 8.0
STILL_V, STILL_W = 0.01, 0.3
CUP_TILT = 10.0
CUP_GAP = 0.003
HANDS_OFF = 0.03
_GRID = [(s, t) for s in np.linspace(0, 1, 5) for t in np.linspace(0, 1, 4)]


def _finger_grid(tips: np.ndarray) -> np.ndarray:
    """Points over each finger's face: a 5 x 4 grid spanned by its sample corners (tip and 3 cm up, both edges)."""
    out = []
    for f in range(4):
        p = tips[6 * f:6 * f + 6]
        a, b, c, d = p[0], p[1], p[4], p[5]  # edge-, tip; edge-, up; edge+, tip; edge+, up
        for s, t in _GRID:
            out.append((1 - s) * ((1 - t) * a + t * b) + s * ((1 - t) * c + t * d))
    return np.array(out)


def _in_tray_xy(p, margin: float = 0.0) -> bool:
    return bool(abs(p[0] - TRAY_C[0]) <= TRAY_IN[0] / 2 - margin and abs(p[1] - TRAY_C[1]) <= TRAY_IN[1] / 2 - margin)


def _rect_dist(p) -> float:
    """Distance in plan from point p to the tray's outside outline (0 inside it)."""
    dx = max(abs(p[0] - TRAY_C[0]) - TRAY_OUT[0] / 2, 0.0)
    dy = max(abs(p[1] - TRAY_C[1]) - TRAY_OUT[1] / 2, 0.0)
    return math.hypot(dx, dy)


def _still(s) -> bool:
    return bool(np.linalg.norm(s.lin_vel) < STILL_V and np.linalg.norm(s.ang_vel) < STILL_W)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the per-die records are cleared
        self.exit_face = {d: None for d in DICE}  # upper face when the colored die first left the cup
        self.turned_after_exit = {d: False for d in DICE}  # latched when a different face becomes uppermost
        self.touched = {d: False for d in DICE}
        self.counts = {d: False for d in DICE}
        self.cup_down = False
        self.n = 0
        self.score = 0.0
        self.full = False
        self.steps = 0

    def step(self, env):
        self.steps += 1
        cup = m.state(env, "cup")
        Rc = cup.rotation()
        fingers = _finger_grid(m.fingertips(env))
        for d in DICE:
            s = m.state(env, d)
            R = s.rotation()
            local_f = (fingers - s.pos) @ R  # finger points in the die's frame
            gap = np.linalg.norm(np.maximum(np.abs(local_f) - _h, 0.0), axis=1)  # distance from each point to the die's box
            if float(np.min(gap)) < TOUCH:
                if DEBUG and not self.touched[d]:
                    print(f"[dice] step {self.steps} {d} touched: gap {float(np.min(gap)) * 1000:.1f} mm at finger point {int(np.argmin(gap))}, die at {np.round(s.pos, 3)}", flush=True)
                self.touched[d] = True
            in_cup = Rc.T @ (s.pos - cup.pos)
            inside = math.hypot(in_cup[0], in_cup[1]) < CUP_R and -0.002 < in_cup[2] < CUP_H
            if self.exit_face[d] is None and not inside:
                self.exit_face[d] = _upper_face(R)
            elif self.exit_face[d] is not None and _upper_face(R) != self.exit_face[d]:
                self.turned_after_exit[d] = True
            low = float(np.min((s.pos + DIE_PTS @ R.T)[:, 2])) - FLOOR_Z
            up = float(np.max(np.abs(R[2, :])))  # the most vertical face axis
            flat = (low < FLAT_GAP and up > math.cos(math.radians(FLAT_DEG)) and _in_tray_xy(s.pos, _h - 0.001))
            self.counts[d] = bool(self.exit_face[d] is not None and self.turned_after_exit[d] and not self.touched[d]
                                  and flat and _still(s))
        cup_local_f = (fingers - cup.pos) @ Rc
        radial = np.hypot(cup_local_f[:, 0], cup_local_f[:, 1]) - CUP_R
        below, above = -cup_local_f[:, 2], cup_local_f[:, 2] - CUP_H
        dist = np.sqrt(np.maximum(radial, 0) ** 2 + np.maximum(np.maximum(below, above), 0) ** 2)
        self.cup_down = bool(cup.tilt_deg() < CUP_TILT and cup.pos[2] - TOP < CUP_GAP and _rect_dist(cup.pos) >= CUP_R
                             and float(np.min(dist)) >= HANDS_OFF and _still(cup))
        self.n = sum(self.counts.values())
        if self.n == 5:
            self.score = 1.0 if self.cup_down else 0.75
        else:
            self.score = 0.5 if self.n >= 3 else (0.25 if self.n >= 1 else 0.0)
        self.full = bool(self.n == 5 and self.cup_down)
        if DEBUG and self.steps % 5 == 0:
            print(f"[dice] step {self.steps} n={self.n} cup_down={self.cup_down} cup z={cup.pos[2] - TOP:.4f} tilt={cup.tilt_deg():.1f} "
                  + " ".join(
                      f"{d}:{'-' if self.exit_face[d] is None else self.exit_face[d]}"
                      f"{'R' if self.turned_after_exit[d] else ''}{'T' if self.touched[d] else ''}"
                      f"{'G' if self.counts[d] else ''}" for d in DICE), flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- scripted demo ------------------------------------------------------------------------------------------------------
# Right arm only, open loop from the known layout. The gripper comes from behind the cup pointing forward and 10 deg
# down, closes across the cup 60 mm up its side, lifts it and rattles it (four quick 1.5 cm side moves), carries it over
# the near half of the tray (grip 20 cm above the table), winds the wrist back 25 deg, then rolls it 175 deg about its
# pointing direction in 0.8 s while sweeping 6 cm along the tray, so the mouth turns down towards the tray: the dice
# tumble toward the tray with a changed upper-facing number after cup exit. It rolls the cup back upright, sets it down on its
# spot, lets go and backs away. The left arm holds its ready pose. A Waypoint's ``yaw`` may carry a full TCP rotation.


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, Orient) else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation

R_LEVEL = np.column_stack([(0, -1, 0), (0, 0, -1), (1, 0, 0)]).astype(float)  # pointing +x, fingers closing along y
DOWN_DEG = 10.0
R_GRIP = _rot((0, 1, 0), DOWN_DEG) @ R_LEVEL
APPROACH = R_GRIP[:, 2]
ROLL_AXIS = APPROACH  # the wrist rolls about the gripper's pointing direction


def _r(roll: float) -> Orient:
    """The grip rolled `roll` deg about its pointing direction (negative turns the cup's mouth towards +y)."""
    return Orient(_rot(ROLL_AXIS, roll) @ R_GRIP)


def _w(t, p, finger, label, roll=0.0) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, _r(roll))


GRASP_Z = TOP + 0.060
BACK = 0.012  # the TCP sits this far behind the cup's axis, along the pointing direction
HOLD = FINGER_CLOSED


def _lead_in(pos, rot: Orient, finger: float, label: str, seconds: float = 3.0, steps: int = 12) -> list:
    """From the ready pose to (pos, rot) in a straight line in joint space, as TCP waypoints (keeps one IK branch)."""
    base = np.array(station.BASES["right"])
    q0 = np.array(READY, float)
    best = None
    for seed in (READY, (1.0, 1.2, 0.1, 0.5, 0.9, 0.3), (1.2, 1.0, 0.3, 0.0, 1.1, 0.3), (0.9, 1.4, 0.2, 0.4, 1.0, 0.0)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot.rot), np.asarray(seed, float), base_pos=base, iters=300)
        key = (pe > 5e-4 or re > 2e-3, abs(q[5]))
        if best is None or key < best[0]:
            best = (key, q)
    q1 = best[1]
    out = []
    for k in range(1, steps + 1):
        p, R = ik.tcp_pose(q0 + (q1 - q0) * k / steps, base)
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), float(finger), label, Orient(R)))
    return out


def demo():
    cx, cy = CUP_POS
    g = np.array([cx, cy, GRASP_Z]) - BACK * APPROACH
    pre = g - 0.10 * APPROACH + np.array([0, 0, 0.02])
    up = g + np.array([0, 0, 0.09])
    over = np.array([TRAY_C[0] + 0.01, TRAY_C[1] - 0.075, TOP + 0.20])
    sweep = np.array([0, 0.06, 0])
    right = _lead_in(pre, _r(0.0), FINGER_OPEN, "behind the cup") + [
        _w(1.5, g, FINGER_OPEN, "around the cup"),
        _w(0.8, g, HOLD, "close"),
        _w(1.2, up, HOLD, "lift"),
    ]
    for k in range(4):
        right.append(_w(0.25, up + np.array([0, 0.015 if k % 2 == 0 else -0.015, 0]), HOLD, "rattle"))
    right.append(_w(0.25, up, HOLD, "rattle"))
    right += [
        _w(2.0, over, HOLD, "over the tray"),
        _w(0.5, over, HOLD, "wind up", 25.0),
        _w(0.8, over + sweep, HOLD, "roll", -150.0),
        _w(0.8, over + sweep, HOLD, "empty", -150.0),
        _w(1.5, over + np.array([0, -0.05, 0.0]), HOLD, "upright"),
        _w(2.0, up, HOLD, "back"),
        _w(1.5, g + np.array([0, 0, 0.002]), HOLD, "set down"),
        _w(0.8, g + np.array([0, 0, 0.002]), FINGER_OPEN, "let go"),
        _w(1.5, pre, FINGER_OPEN, "back away"),
        _w(1.5, pre + np.array([-0.03, 0, 0.06]), FINGER_OPEN, "clear"),
    ]
    lp, lr = ik.tcp_pose(np.array(READY, float), np.array(station.BASES["left"]))
    left = [Waypoint(sum(w.duration for w in right), tuple(float(v) for v in lp), FINGER_OPEN, "wait", Orient(lr))]
    return {"left": left, "right": right}
