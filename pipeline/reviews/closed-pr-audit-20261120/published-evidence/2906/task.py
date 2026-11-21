"""Put an electric carving knife's blades back together and into the handle (yam_arena task file, station scene).

The knife's two serrated blades lie washed on the counter, apart. Blade A lies inner face up, its rivet (at the tip)
pointing up and its plastic grease guard underneath; blade B lies inner face down, guard on top, with the keyhole
slot near its tip. The power handle (unplugged) lies on its side to the right, its blade socket facing +y.
Joined, the blades go into the socket together; a sprung latch in the handle clicks into a window in each blade's
tail. Every part collides; see README.md.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 2 s hold
SCORE_OVER = "end"

# ---------------------------------------------------------------------------------------------------------------
# Blade dimensions (metres), in a blade's own frame: origin at the rear end of its tail on the centre line, +y towards
# the tip, steel from z 0 to T. Blade A's guard is under it (z -G..0), blade B's on top (z T..T+G).
T = 0.0012  # stainless steel blade, 1.2 mm
L = 0.265  # overall length: 3.5 cm tail, 1.4 cm guard, 21.6 cm serrated blade
TAIL_L, TAIL_HW = 0.035, 0.006  # tail behind the guard, 12 mm wide
WIN_Y0, WIN_Y1, WIN_HW = 0.006, 0.0175, 0.003  # latch window through the tail, 11.5 x 6 mm
CHAMFER = 0.004  # the tail's end is chamfered over 4 mm
G, GL, GHW = 0.006, 0.014, 0.018  # nylon grease guard: 6 mm thick, 14 mm long, 36 mm wide
GY0, GY1 = TAIL_L, TAIL_L + GL
BASE_HW, KEY_HW = 0.011, 0.007  # blade half-width at the guard and in the keyhole region
KEY_Y0, KEY_Y1 = L - 0.030, L - 0.006  # keyhole region; tip beyond
HOLE_Y, HOLE_R = L - 0.020, 0.0035  # keyhole's round end (B), drawn round, colliding as an octagon
SLOT_END, SLOT_HW = L - 0.014, 0.0015  # narrow slot runs 6 mm from the hole towards the tip, 3 mm wide
RIV_Y, RIV_R, RIV_SH = SLOT_END - 0.001, 0.0010, T + 0.0005  # rivet on A: 2 mm shank standing 1.7 mm; at the slot's end the guards line up
HEAD_R, HEAD_H = 0.0025, 0.0009  # 5 mm rivet head
BLADE_MASS = 0.048

# Handle (power handle lying on its side), in its own frame: origin on the table under the centre of the socket's
# mouth, the socket running along -y, z up.
H_LEN, H_HW, H_TH = 0.215, 0.034, 0.050  # 21.5 cm long, 6.8 cm deep, 5.0 cm thick
NOSE_HW, NOSE_TOP, NOSE_L = 0.014, 0.034, 0.060  # the nose round the blade socket: 2.8 cm deep, 3.4 cm thick, 6 cm long
ZC = 0.025  # socket centre height
CH_HH, CH_HW = 0.0020, 0.0065  # socket: 4.0 mm x 13 mm (two 1.2 mm tails, 12 mm wide, with play)
CH_DEPTH, FUN = 0.040, 0.006  # socket depth, and the lead-in funnel at the mouth
LATCH_Y0, LATCH_Y1 = -0.0275, -0.0235  # latch lip's deep face and the foot of its ramp
LATCH_RAMP = 0.0052  # 20 deg ramp towards the mouth
LATCH_PRO, LATCH_TRAVEL = 0.0019, 0.0024  # lip protrudes 1.9 mm into the socket at rest
LATCH_K, LATCH_PRE = 1500.0, 0.0010  # spring (N/m) and preload (m past the rest stop): 1.5 N held, 4.4 N fully lifted
HANDLE_MASS = 0.55
HANDLE_XY = np.array([0.30, -0.03])

A_START = np.array([0.18, 0.02])  # tail end of blade A
B_START = np.array([0.27, 0.035])  # tail end of blade B

LOOKS = {
    "steel": m.Look(color=m.rgb(0.78, 0.79, 0.81), metallic=0.9, roughness=0.25),
    "edge": m.Look(color=m.rgb(0.55, 0.56, 0.58), metallic=0.9, roughness=0.35),
    "guard": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "rivet": m.Look(color=m.rgb(0.66, 0.66, 0.68), metallic=0.85, roughness=0.3),
    "body": m.Look(color=m.rgb(0.92, 0.92, 0.90), roughness=0.45),
    "grip": m.Look(color=m.rgb(0.20, 0.21, 0.23), roughness=0.75),
    "button": m.Look(color=m.rgb(0.75, 0.10, 0.10), roughness=0.5),
    "latch": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=0.8, roughness=0.35),
}


# ---------------------------------------------------------------------------------------------------------------
# Geometry helpers


def _hull8(x0, x1, y0, y1, z0, z1, hw0=None, hw1=None):
    """A slab hull from y0 to y1, x from x0 to x1 (or +-hw0 at y0 and +-hw1 at y1 for a taper)."""
    if hw0 is not None:
        pts = [(sx * (hw0 if y == y0 else hw1), y, z) for y in (y0, y1) for sx in (-1, 1) for z in (z0, z1)]
    else:
        pts = [(x, y, z) for y in (y0, y1) for x in (x0, x1) for z in (z0, z1)]
    return np.array(pts, float)


def _hmesh(pts) -> m.Mesh:
    q = np.asarray(pts, float)
    faces = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    return m.Mesh(q, np.array(faces))


def _piece(name, pts, look, parts, cols, visual=True):
    mesh = _hmesh(pts)
    cols.append(m.col_hull(f"{name}_c", mesh))
    if visual:
        parts.append((name, mesh, look))


def _bx(name, x0, x1, y0, y1, z0, z1, look, parts, cols, visual=True):
    _piece(name, _hull8(x0, x1, y0, y1, z0, z1), look, parts, cols, visual)


def _tri(name, p, z0, z1, look, parts, cols):
    """A triangular extruded wedge from three xy points (any order)."""
    p = np.asarray(p, float)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]
    mesh = m.prism(p, z0, z1)
    cols.append(m.col_hull(f"{name}_c", mesh))
    parts.append((name, mesh, look))


def _blade_common(prefix, parts, cols, z0, z1):
    """Tail with its latch window, and the blade from the guard to the keyhole region (shared by A and B)."""
    # the tail's end is ground to a wedge (full thickness 4 mm back, 0.4 mm at the end) so it rides up the latch ramps
    zm, th = (z0 + z1) / 2, 0.0002
    end = [(x, y, z) for y in (0.0, CHAMFER) for x in (-TAIL_HW, TAIL_HW) for z in ((zm - th, zm + th) if y == 0.0 else (z0, z1))]
    _piece(f"{prefix}tail_end", end, "steel", parts, cols)
    _bx(f"{prefix}tail_r", -TAIL_HW, TAIL_HW, CHAMFER, WIN_Y0, z0, z1, "steel", parts, cols)
    _bx(f"{prefix}tail_l", -TAIL_HW, -WIN_HW, WIN_Y0, WIN_Y1, z0, z1, "steel", parts, cols)
    _bx(f"{prefix}tail_s", WIN_HW, TAIL_HW, WIN_Y0, WIN_Y1, z0, z1, "steel", parts, cols)
    _bx(f"{prefix}tail_f", -TAIL_HW, TAIL_HW, WIN_Y1, GY0, z0, z1, "steel", parts, cols)
    _bx(f"{prefix}under_guard", -BASE_HW, BASE_HW, GY0, GY1, z0, z1, "steel", parts, cols, visual=False)
    _piece(f"{prefix}body", _hull8(0, 0, GY1, KEY_Y0, z0, z1, BASE_HW, KEY_HW), "steel", parts, cols)
    # serrations, drawn along the cutting edge (-x side)
    n = 36
    for i in range(n):
        y0 = GY1 + 0.004 + i * (KEY_Y0 - GY1 - 0.008) / n
        y1 = y0 + (KEY_Y0 - GY1 - 0.008) / n
        hw = BASE_HW - (BASE_HW - KEY_HW) * ((y0 + y1) / 2 - GY1) / (KEY_Y0 - GY1)
        tooth = m.prism(np.array([(-hw + 0.0001, y0), (-hw + 0.0001, y1), (-hw + 0.0022, (y0 + y1) / 2)]), z1, z1 + 0.00005)
        parts.append((f"{prefix}tooth{i}", tooth, "edge"))
    # tip
    tip = _hull8(0, 0, KEY_Y1, L, z0, z1, KEY_HW, 0.002)
    _piece(f"{prefix}tip", tip, "steel", parts, cols)


def _blade_a():
    """Blade A: plain keyhole region with the rivet standing up from its inner face; guard underneath."""
    parts, cols = [], []
    _blade_common("", parts, cols, 0.0, T)
    _bx("key", -KEY_HW, KEY_HW, KEY_Y0, KEY_Y1, 0.0, T, "steel", parts, cols)
    _bx("guard", -GHW, GHW, GY0, GY1, -G, 0.0, "guard", parts, cols)
    shank = m.cylinder(RIV_R, RIV_SH, (0, RIV_Y, T + RIV_SH / 2), segments=16)
    head = m.cylinder(HEAD_R, HEAD_H, (0, RIV_Y, T + RIV_SH + HEAD_H / 2), segments=24)
    parts += [("rivet_shank", shank, "rivet"), ("rivet_head", head, "rivet")]
    cols += [m.col_cylinder("shank_c", RIV_R, RIV_SH, (0, RIV_Y, T + RIV_SH / 2)), m.col_cylinder("head_c", HEAD_R, HEAD_H, (0, RIV_Y, T + RIV_SH + HEAD_H / 2))]
    return m.free("blade_a", (A_START[0], A_START[1], TOP + G + 0.0005), parts, cols, LOOKS, mass=BLADE_MASS, friction=(0.45, 0.4),
                  torsional_patch=0.015, solver_iters=48, com=(0.0, 0.10, -0.001))


def _blade_b():
    """Blade B: keyhole slot through the steel near the tip (round end towards the guard); guard on top."""
    parts, cols = [], []
    _blade_common("", parts, cols, 0.0, T)
    hy, r, s = HOLE_Y, HOLE_R, 0.00145  # octagon round the hole: apothem r, half-side s
    _bx("key_rear", -KEY_HW, KEY_HW, KEY_Y0, hy - r, 0.0, T, "steel", parts, cols)
    _bx("key_l", -KEY_HW, -r, hy - r, hy + r, 0.0, T, "steel", parts, cols)
    _bx("key_r", r, KEY_HW, hy - r, hy + r, 0.0, T, "steel", parts, cols)
    for k, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        _tri(f"key_corner{k}", [(sx * r, hy + sy * r), (sx * r, hy + sy * s), (sx * s, hy + sy * r)], 0.0, T, "steel", parts, cols)
    _bx("slot_l", -KEY_HW, -SLOT_HW, hy + r, SLOT_END, 0.0, T, "steel", parts, cols)
    _bx("slot_r", SLOT_HW, KEY_HW, hy + r, SLOT_END, 0.0, T, "steel", parts, cols)
    _bx("key_front", -KEY_HW, KEY_HW, SLOT_END, KEY_Y1, 0.0, T, "steel", parts, cols)
    _bx("guard", -GHW, GHW, GY0, GY1, T, T + G, "guard", parts, cols)
    return m.free("blade_b", (B_START[0], B_START[1], TOP + 0.0005), parts, cols, LOOKS, mass=BLADE_MASS, friction=(0.45, 0.4),
                  torsional_patch=0.015, solver_iters=48, com=(0.0, 0.10, T + 0.001))


# ---------------------------------------------------------------------------------------------------------------
# The power handle: a free body with the blade socket, and two sprung latches (one per blade) as slider links


def _latch(name, top: bool):
    """The latch lip over (top) or under the socket: deep face vertical, a 20 deg ramp facing the mouth."""
    sgn = 1.0 if top else -1.0
    face = ZC + sgn * CH_HH  # the socket wall it comes through
    tip = face - sgn * LATCH_PRO
    back = face + sgn * LIP_BACK
    yz = [(LATCH_Y0, tip), (LATCH_Y1, tip), (LATCH_Y1 + LATCH_RAMP, face), (LATCH_Y1 + LATCH_RAMP, back), (LATCH_Y0, back)]
    hw = LIP_HW
    n = len(yz)
    v = [(x, y, z) for x in (-hw, hw) for y, z in yz]
    f = [(0, i, i + 1) for i in range(1, n - 1)] + [(n, n + i + 1, n + i) for i in range(1, n - 1)]
    f += [t for i in range(n) for t in ((i, (i + 1) % n + n, i + n), (i, (i + 1) % n, (i + 1) % n + n))]
    mesh = m.Mesh(np.array(v, float), np.array(f))
    col = m.col_hull(f"{name}_c", mesh)
    vis = _hmesh(_hull8(-hw, hw, LATCH_Y0, LATCH_Y1 + LATCH_RAMP, min(tip, back), max(tip, back)))
    lo, hi = (0.0, LATCH_TRAVEL) if top else (-LATCH_TRAVEL, 0.0)
    return m.slider(name, "Z", lo, hi, [("lip", vis, "latch")], [col], mass=0.01, spring=LATCH_K, damping=5.0, friction=0.0, effort=50.0,
                    armature=0.0005, inertia=(1e-7, 1e-7, 1e-7))


LIP_HW, LIP_BACK = 0.0022, 0.003  # latch lip half-width; its body reaches 3 mm into the wall


def _pocketed(name, y0, y1, z0, z1, top, parts, cols):
    """A wall of the socket with a pocket the latch lip slides in (so no link sits inside a frame collider)."""
    py0, py1 = LATCH_Y0 - 0.0005, LATCH_Y1 + LATCH_RAMP + 0.0005
    px = LIP_HW + 0.0005
    depth = LIP_BACK + LATCH_TRAVEL + 0.0005
    zc0, zc1 = (z0 + depth, z1) if top else (z0, z1 - depth)
    _bx(f"{name}_front", -NOSE_HW, NOSE_HW, py1, y1, z0, z1, "body", parts, cols)
    _bx(f"{name}_back", -NOSE_HW, NOSE_HW, y0, py0, z0, z1, "body", parts, cols)
    _bx(f"{name}_l", -NOSE_HW, -px, py0, py1, z0, z1, "body", parts, cols)
    _bx(f"{name}_r", px, NOSE_HW, py0, py1, z0, z1, "body", parts, cols)
    _bx(f"{name}_cap", -px, px, py0, py1, zc0, zc1, "body", parts, cols)


def _handle():
    parts, cols = [], []
    # socket block at the front, built round the socket
    y0, y1 = -CH_DEPTH, -FUN
    _pocketed("floor", y0, y1, 0.0, ZC - CH_HH, False, parts, cols)
    _pocketed("ceil", y0, y1, ZC + CH_HH, NOSE_TOP, True, parts, cols)
    _bx("nose_back", -NOSE_HW, NOSE_HW, -NOSE_L, y0, 0.0, NOSE_TOP, "body", parts, cols)
    _bx("wall_l", -NOSE_HW, -CH_HW, y0, y1, ZC - CH_HH, ZC + CH_HH, "body", parts, cols)
    _bx("wall_r", CH_HW, NOSE_HW, y0, y1, ZC - CH_HH, ZC + CH_HH, "body", parts, cols)
    fz, fx = 0.003, 0.004  # funnel opening
    pts = [(x, y, z) for y in (y1, 0.0) for x in (-NOSE_HW, NOSE_HW) for z in (0.0, ZC - CH_HH - (fz if y == 0.0 else 0.0))]
    _piece("fun_floor", pts, "body", parts, cols)
    pts = [(x, y, z) for y in (y1, 0.0) for x in (-NOSE_HW, NOSE_HW) for z in (ZC + CH_HH + (fz if y == 0.0 else 0.0), NOSE_TOP)]
    _piece("fun_ceil", pts, "body", parts, cols)
    for nm, s in (("fun_l", -1), ("fun_r", 1)):
        pts = [(s * xx, y, z) for y in (y1, 0.0) for xx in ((CH_HW if y == y1 else CH_HW + fx), NOSE_HW) for z in (ZC - CH_HH - fz, ZC + CH_HH + fz)]
        _piece(nm, pts, "body", parts, cols)
    # the body behind the socket, rounded in the drawing; a box collider
    cols.append(m.col_box("rear_c", (2 * H_HW, H_LEN - NOSE_L, H_TH), (0, -(H_LEN + NOSE_L) / 2, H_TH / 2)))
    outline = np.array([(-H_HW, -NOSE_L), (H_HW, -NOSE_L), (H_HW, -0.150), (H_HW - 0.006, -H_LEN + 0.004), (0.0, -H_LEN),
                        (-H_HW + 0.006, -H_LEN + 0.004), (-H_HW, -0.150)])
    parts.append(("rear", m.prism(outline, 0.0, H_TH), "body"))
    parts.append(("grip_top", m.box((0.040, 0.100, 0.002), (0.0, -0.120, H_TH + 0.001)), "grip"))
    parts.append(("trigger", m.box((0.008, 0.050, 0.020), (-H_HW - 0.004, -0.085, ZC)), "button"))
    parts.append(("release_top", m.cylinder(0.006, 0.002, (0.0, -0.075, H_TH + 0.001)), "grip"))
    parts.append(("cord_boot", m.cylinder(0.008, 0.020, (0.0, -H_LEN - 0.006, ZC), axis="y"), "grip"))
    links = [_latch("latch_b", True), _latch("latch_a", False)]
    ixx = HANDLE_MASS / 12 * (H_LEN ** 2 + H_TH ** 2)
    iyy = HANDLE_MASS / 12 * ((2 * H_HW) ** 2 + H_TH ** 2)
    izz = HANDLE_MASS / 12 * ((2 * H_HW) ** 2 + H_LEN ** 2)
    return m.mechanism("handle", (HANDLE_XY[0], HANDLE_XY[1], TOP + 0.0005), links, LOOKS, frame_parts=parts, frame_colliders=cols,
                       friction=(0.5, 0.4), solver_iters=48, free_mass=HANDLE_MASS, frame_inertia=(ixx, iyy, izz))


BLADE_A, BLADE_B, HANDLE = _blade_a(), _blade_b(), _handle()
OBJECTS = [
    BLADE_A, BLADE_B, HANDLE,
    # distractors, ignored by the scoring
    SimReady("sponge", pos=(0.50, -0.20, TOP), size=(0.071, 0.101, 0.033), mass=0.02, package="cleaning_sponge_b01",
             usd="sm_cleaning_sponge_b01_01.usd", yaw=0.0, convex=True),
    SimReady("mug", pos=(0.50, 0.24, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
             usd="sm_dishware_mug_a01_01.usda", yaw=-math.pi / 2, convex=True),
]


# ---------------------------------------------------------------------------------------------------------------
# Scoring: the tracker reads the parts every control step

HOLD_STEPS = 40  # 2 s at 20 Hz
LATCH_DROPPED = 0.00035  # a latch lip within 0.35 mm of its rest stop is down in its window
LATCH_GAP = 0.0015  # guard within 1.5 mm of the socket's mouth


def rivet_in_slot(a, b) -> bool:
    """The blades joined: A's rivet in the narrow part of B's keyhole slot (3.5 to 6.5 mm from the centre of the
    round end, within 2 mm of the slot's centre line), the blades face to face (A's inner face within 0.6 mm of
    B's) and parallel within 4 deg. With the rivet there the head overlaps the slot's sides and holds B on."""
    p = b.to_local(a.to_world(np.array([[0.0, RIV_Y, T]])))[0]
    ra, rb = a.rotation(), b.rotation()
    cosang = float(np.clip(np.dot(ra[:, 2], rb[:, 2]), -1, 1))
    return bool(abs(p[0]) < 0.002 and HOLE_Y + 0.0035 <= p[1] <= SLOT_END + 0.0005 and abs(p[2]) < 0.0006 and math.degrees(math.acos(cosang)) < 4.0)


def depth_gap(blade, h) -> float:
    """How far (m) the blade's guard stands off the socket's mouth, measured along the socket."""
    p = h.to_local(blade.to_world(np.array([[0.0, GY0, 0.0]])))[0]
    return float(p[1])


def latched(blade, h, link: str) -> bool:
    """The blade's latch is down in its tail's window: the lip within 0.35 mm of its rest stop, the guard within
    1.5 mm of the mouth, and the tail's end inside the socket (within 2 mm of its centre line sideways and within the
    socket's 4 mm height), so a blade lying on the nose with its guard at the mouth does not count."""
    p = h.to_local(blade.to_world(np.array([[0.0, GY0, 0.0]])))[0]
    e = h.to_local(blade.to_world(np.array([[0.0, 0.0, T / 2]])))[0]  # the tail's end, on its centre line
    in_socket = abs(e[0]) < 0.002 and abs(e[2] - ZC) < CH_HH and e[1] < -0.03
    return bool(abs(h.joint(link)) < LATCH_DROPPED and -0.001 < p[1] < LATCH_GAP and abs(p[0]) < 0.002 and in_socket)


def still(*states) -> bool:
    return bool(all(np.linalg.norm(s.lin_vel) < 0.01 and np.linalg.norm(s.ang_vel) < 0.2 for s in states))


def hands_clear(tips, a, b, h) -> bool:
    """Every fingertip point more than 4 cm from both guards and from the socket's mouth."""
    tips = np.atleast_2d(tips)
    for s, y in ((a, GY0 + GL / 2), (b, GY0 + GL / 2)):
        c = s.to_world(np.array([[0.0, y, 0.0]]))[0]
        if np.min(np.linalg.norm(tips - c, axis=1)) < 0.04:
            return False
    mouth = h.to_world(np.array([[0.0, 0.0, ZC]]))[0]
    return bool(np.min(np.linalg.norm(tips - mouth, axis=1)) >= 0.04)


def off_table(*states) -> bool:
    return bool(any(s.pos[2] < TOP - 0.05 for s in states))


def lip_wanted(blade, h) -> str:
    """The latch's cam, as a rule (the simulator does not cam a sprung lip reliably on 1.2 mm steel): the lip is
    lifted clear while the blade's tail passes under it, and drops when the tail's window lies under it."""
    p = h.to_local(blade.to_world(np.array([[0.0, 0.0, T / 2]])))[0]  # the tail's end, on its centre line
    if abs(p[0]) > CH_HW or abs(p[2] - ZC) > CH_HH + 0.001 or p[1] > LATCH_Y1 + LATCH_RAMP + 0.004:
        return "rest"  # no tail in the socket up to the lip
    if p[1] + WIN_Y0 <= LATCH_Y0 and p[1] + WIN_Y1 >= LATCH_Y1:
        return "rest"  # the window is under the lip: it drops in
    return "lifted"


def _drive_latches(trial, env, a, b, h):
    for link, blade, sgn in (("latch_a", a, -1.0), ("latch_b", b, 1.0)):
        want = lip_wanted(blade, h)
        if want != trial.lip[link]:
            m.set_target(env, "handle", link, sgn * (LATCH_TRAVEL if want == "lifted" else -LATCH_PRE))
            trial.lip[link] = want


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared: a reset to the starting layout
        self.dropped, self.hold, self.full, self.score, self.steps = False, 0, False, 0.0, 0
        self.lip = {"latch_a": None, "latch_b": None}
        if env is not None:  # latch preload against its rest stop (physics, not scoring)
            m.set_target(env, "handle", "latch_b", -LATCH_PRE)
            m.set_target(env, "handle", "latch_a", LATCH_PRE)

    def step(self, env):
        self.steps += 1
        a, b, h = m.state(env, "blade_a"), m.state(env, "blade_b"), m.state(env, "handle")
        _drive_latches(self, env, a, b, h)
        if off_table(a, b, h):
            self.dropped = True
        joined = rivet_in_slot(a, b)
        la, lb = latched(a, h, "latch_a"), latched(b, h, "latch_b")
        part = 0.25 * joined + 0.25 * la + 0.25 * lb
        ok = joined and la and lb and not self.dropped and hands_clear(m.fingertips(env), a, b, h) and still(a, b, h)
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.score = 0.0 if self.dropped else (1.0 if self.full else float(part))
        _debug(self, env, a, b, h, joined, la, lb)


DEBUG = None  # set to a path to log the parts while developing


def _debug(trial, env, a, b, h, joined, la, lb):
    if DEBUG is None or not (trial.steps <= 2 or trial.steps % 10 == 0):
        return
    p = b.to_local(a.to_world(np.array([[0.0, RIV_Y, T]])))[0]
    with open(DEBUG, "a") as f:
        f.write(f"{trial.steps} A={np.round(a.pos - [0, 0, TOP], 4).tolist()} At={a.tilt_deg():.2f} B={np.round(b.pos - [0, 0, TOP], 4).tolist()} "
                f"Bt={b.tilt_deg():.2f} H={np.round(h.pos - [0, 0, TOP], 4).tolist()} Ht={h.tilt_deg():.2f} riv_in_B={np.round(p * 1000, 2).tolist()} "
                f"gapA={depth_gap(a, h) * 1000:.1f} gapB={depth_gap(b, h) * 1000:.1f} la={h.joint('latch_a') * 1000:.2f} lb={h.joint('latch_b') * 1000:.2f} "
                f"J={joined} LA={la} LB={lb} score={trial.score:.2f} tipL={np.round(m.fingertips(env)[:12].mean(axis=0) - [0, 0, TOP], 4).tolist()}\n")


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# ---------------------------------------------------------------------------------------------------------------
# Scripted demo: a fixed plan from the published starting layout (open loop)


def _w(d, x, y, z, finger, label, yaw_deg=0.0):
    return Waypoint(float(d), (float(x), float(y), TOP + float(z)), float(finger), label, math.radians(yaw_deg))


def _f(width, force=25.0):
    """The finger target that closes on something `width` across with about `force` newtons a finger."""
    return float(np.clip(-(width / 2 - force / 5000.0), FINGER_OPEN, FINGER_CLOSED))


def _pad(plan, until, label="wait"):
    t = sum(w.duration for w in plan)
    if until > t + 1e-6:
        last = plan[-1]
        plan.append(Waypoint(until - t, last.pos, last.finger, label, last.yaw))


def _clock(plan):
    return sum(w.duration for w in plan)


GRIP_YAW = -90.0  # the left gripper closes along x, across the guards
INS_YAW = GRIP_YAW  # (+90 would put joint 6 past its limit)
TCP_OVER_PAD = 0.013  # fingertip pads reach this far below the TCP
B_GRIP = np.array([B_START[0], B_START[1] + GY0 + GL / 2])  # B's guard
TILT_A = math.radians(1.59)  # blade A rests on its guard and its tip (measured)
A_ORIGIN = np.array([A_START[0], A_START[1] + 0.0002, 0.0074])  # settled (measured)
RIVET_W = A_ORIGIN + np.array([0.0, RIV_Y * math.cos(TILT_A), -RIV_Y * math.sin(TILT_A) + T])
PLACE_DX, PLACE_DY = 0.0008, -0.0002  # carry offsets (measured)
SLIDE = 0.0075  # B slides back until the rivet stops at the slot's end: the guards line up
PAIR_DZ = 0.0071  # the blades' joint face below the TCP when the pair is gripped by both guards (measured)
TCP_GRIP_B = 0.015
PAIR_GRIP = (0.1807, 0.0617, 0.0145)  # over both guards after the slide
PAIR_DX, PAIR_DY = 0.0, 0.042  # the tails' ends: this far behind the TCP
BRACE = 0.0055
STAGE = 9
PAIR_FORCE = 50.0  # the finger actuators' limit
PAIR_SAG = 0.0006  # the pair rides this much lower in the grip than at pickup, tip down about 3.5 deg (measured)


def demo():
    L_, R_ = [], []
    bx, by = B_GRIP
    hole_ahead = HOLE_Y - (GY0 + GL / 2)
    px, py = RIVET_W[0] + PLACE_DX, RIVET_W[1] - hole_ahead + PLACE_DY
    zr = 0.0086 + TCP_GRIP_B  # B's underside level with A's highest point
    gy = 1
    L_ += [_w(2.0, bx, by, 0.06, FINGER_OPEN, "over blade B's guard", GRIP_YAW),
           _w(1.5, bx, by, TCP_GRIP_B, FINGER_OPEN, "down to the guard", GRIP_YAW),
           _w(0.8, bx, by, TCP_GRIP_B, _f(2 * GHW), "grip it", GRIP_YAW),
           _w(1.2, bx, by, 0.045, _f(2 * GHW), "lift blade B", GRIP_YAW),
           _w(2.0, px, py, 0.045, _f(2 * GHW), "over blade A, keyhole over the rivet", GRIP_YAW),
           _w(1.5, px, py, zr + 0.003, _f(2 * GHW), "lower", GRIP_YAW),
           _w(0.8, px, py, zr, _f(2 * GHW), "onto blade A", GRIP_YAW),
           _w(0.6, px, py, zr, FINGER_OPEN, "let go: the rivet head drops through the keyhole", GRIP_YAW),
           _w(0.8, px, py, zr + 0.02, FINGER_OPEN, "up", GRIP_YAW),
           _w(0.6, px, py, zr + 0.02, FINGER_OPEN, "wait", GRIP_YAW)]
    if STAGE >= 2:
        zg = 0.0072 + 0.0012 + TCP_GRIP_B - 0.0005
        L_ += [_w(0.8, px, py, zg, FINGER_OPEN, "down to B's guard", GRIP_YAW),
               _w(0.8, px, py, zg, _f(2 * GHW), "grip it", GRIP_YAW),
               _w(1.5, px, py - SLIDE, zg, _f(2 * GHW), "slide B back: the rivet runs along the slot", GRIP_YAW),
               _w(0.6, px, py - SLIDE, zg, FINGER_OPEN, "let go", GRIP_YAW),
               _w(0.6, px, py - SLIDE, zg + 0.02, FINGER_OPEN, "up", GRIP_YAW)]
    if STAGE >= 3:
        gx, gy, gz = PAIR_GRIP
        hx, hy = HANDLE_XY
        mz = ZC + PAIR_DZ + PAIR_SAG  # TCP height that puts the guards' joint face on the socket centre line once the socket levels the pair
        y_pre = hy + 0.018 + PAIR_DY
        y_ins = hy + PAIR_DY - TAIL_L - 0.0008  # guards pressed 0.8 mm past the mouth: the fingers pass beside the nose
        L_ += [_w(1.0, gx, gy, 0.04, FINGER_OPEN, "over both guards", INS_YAW),
               _w(1.0, gx, gy, gz, FINGER_OPEN, "down beside the guards", INS_YAW),
               _w(0.8, gx, gy, gz, _f(2 * GHW, PAIR_FORCE), "grip both guards", INS_YAW),
               _w(1.5, gx, gy, 0.07, _f(2 * GHW, PAIR_FORCE), "lift the joined blades", INS_YAW),
               _w(3.0, hx + PAIR_DX, y_pre + 0.02, 0.07, _f(2 * GHW, PAIR_FORCE), "to the handle", INS_YAW),
               _w(1.5, hx + PAIR_DX, y_pre, mz, _f(2 * GHW, PAIR_FORCE), "tails in line with the socket", INS_YAW),
               _w(1.5, hx + PAIR_DX, y_pre, mz, _f(2 * GHW, PAIR_FORCE), "steady", INS_YAW)]
        t_ins = _clock(L_)
        L_ += [_w(4.0, hx + PAIR_DX, y_ins, mz, _f(2 * GHW, PAIR_FORCE), "slide the tails in until both latches click", INS_YAW),
               _w(1.0, hx + PAIR_DX, y_ins, mz, _f(2 * GHW, PAIR_FORCE), "hold", INS_YAW),
               _w(0.6, hx + PAIR_DX, y_ins, mz, FINGER_OPEN, "let go", INS_YAW),
               _w(1.0, hx + PAIR_DX, y_ins, 0.10, FINGER_OPEN, "up", INS_YAW),
               _w(1.5, 0.22, 0.25, 0.15, FINGER_OPEN, "away", 0.0)]
        t_done = _clock(L_) - 1.5
        rear = hy - H_LEN
        R_ += [_w(2.0, 0.22, -0.25, 0.12, FINGER_CLOSED, "wait", 0.0)]
        _pad(R_, t_ins - 3.5)
        R_ += [_w(1.5, hx, rear - 0.03, 0.06, FINGER_CLOSED, "behind the handle", 0.0),
               _w(1.0, hx, rear - 0.03, 0.028, FINGER_CLOSED, "down", 0.0),
               _w(1.0, hx, rear - BRACE, 0.028, FINGER_CLOSED, "against its back: hold it against the push", 0.0)]
        _pad(R_, t_done)
        R_ += [_w(0.8, hx, rear - 0.03, 0.028, FINGER_CLOSED, "off", 0.0),
               _w(1.5, 0.22, -0.25, 0.15, FINGER_OPEN, "away", 0.0)]
    else:
        _pad(L_, _clock(L_) + 3.0)
        R_ += [_w(2.0, 0.20, -0.20, 0.15, FINGER_OPEN, "wait")]
    end = max(_clock(L_), _clock(R_)) + 2.0
    _pad(L_, end)
    _pad(R_, end)
    return {"left": L_, "right": R_}
