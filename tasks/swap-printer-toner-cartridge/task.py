"""Swap the toner cartridge of a desktop mono laser printer and close its front door.

The printer is modelled on the HP LaserJet Pro M404 series (published size 381 x 357 x 216 mm, 8.9 kg with its cartridge) and
its HP 58A (CF258A) cartridge (0.72 kg). The user guide's procedure is the job: open the front door (done: it lies open on its
stop), take the used cartridge out, install the new one along the tracks until it is firmly seated, close the front door.

Model (declared in code; see README "The model" for which numbers are published and which are estimates):
- "printer", a fixed mechanism. Frame: the case (base and tray 2, side walls, top cover, rear block), the bay floor and, on each
  side wall, the cartridge track: a ledge sloping 12 degrees down toward the back, with a seat notch and a back stop at its end.
  Links: the front door, an L-shaped cover (front panel and the front strip of the top) hinged along the bay's bottom front edge,
  authored open at 80 degrees (joint 0 = open, 80 = shut), with gravity, hinge friction and damping; and the door release
  button on the right side (a sprung slider). The latch is code: the door locks once it reaches shut, and only the release
  button unlocks it.
- "old" and "new": two free cartridges of the same shape: a 330 x 100 x 90 mm body with a raised ridge along its top and the
  drum's axle boss on each end, which rides the track ledges and drops into the seat notches.

    pipeline: run with yam_scripted (demo()), or inspect-robots run ... -E task=task.py (README)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demokit as d  # noqa: E402  (only its grips: importing it lets a Waypoint carry a full TCP rotation)
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Replace the toner cartridge in the printer with the new one on the table, leave the old one on the table, and close the printer's front door."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the printer: its frame P has its origin on the table at the middle of the front face; x toward the back, y to the left, z up
PX = 0.26  # the printer's front face, in front of the arm bases
W, D, H = 0.381, 0.357, 0.216  # published (HP user guide, Table 1-3)
HW = W / 2
BASE_H = 0.075  # base and tray 2; the door's hinge line and the bay floor
WALL_IN = 0.178  # the bay's inner half width
COVER_Z = 0.200  # underside of the top cover
FLANGE_D = 0.080  # how far the door's top strip reaches back (the top cover starts there)
BAY_BACK = 0.200  # the rear block (fuser, laser scanner) starts here
DOOR_T = 0.016
DOOR_OPEN = 90.0  # degrees the door lies open, flat on its stop
DOOR_MASS = 0.45
DOOR_FRICTION = 0.25  # N m at the hinge
LATCH_AT = DOOR_OPEN - 1.0  # the door latches once its joint is within 1 degree of shut

# the track on each side wall: the boss centre rests on the ledge along zc(x); the notch at the end holds it 4 mm lower
BOSS_R = 0.008
TRACK_TAN = math.tan(math.radians(12.0))
LEDGE_X0, LEDGE_X1 = 0.004, 0.090
LEDGE_Y0, LEDGE_Y1 = 0.168, WALL_IN  # the ledge stands out 10 mm from the wall
SEAT_BOSS = np.array([0.097, 0.100])  # x, z of the boss centre when seated
STOP_X = SEAT_BOSS[0] + BOSS_R + 0.0005


def zc(x):
    """The boss centre's height when it rests on the ledge at x (printer frame)."""
    return 0.104 + TRACK_TAN * (LEDGE_X1 - x)


# -- the cartridge: its frame C has its origin at the middle of the body's underside; x toward the printer's back once installed
BODY = (0.100, 0.330, 0.090)
BOSS_C = np.array([0.025, 0.025])  # x, z of the boss axis
BOSS_Y0, BOSS_Y1 = 0.165, 0.176
HANDLE_LEAN = 30.0  # the carry handle (a fin along the top) leans this far toward the front
HANDLE = (-0.005, 0.088, 0.036, 0.016, 0.100)  # its base centre x, z; length along its lean; thickness; half length along y
DRUM = (0.034, 0.013, 0.013, 0.150)  # x, z, radius, half length of the exposed imaging drum
SEATED = np.array([SEAT_BOSS[0] - BOSS_C[0], 0.0, SEAT_BOSS[1] - BOSS_C[1]])  # the cartridge origin in P when seated
NEW_MASS, OLD_MASS = 0.72, 0.65
_HA = np.array([-math.sin(math.radians(HANDLE_LEAN)), 0.0, math.cos(math.radians(HANDLE_LEAN))])  # up the handle
_HN = np.array([math.cos(math.radians(HANDLE_LEAN)), 0.0, math.sin(math.radians(HANDLE_LEAN))])  # across it
GRIP_C = np.array([HANDLE[0], 0.0, HANDLE[1]]) + 0.030 * _HA  # where the TCP holds the handle, in C
NEW_START = (PX - 0.058, -0.38)  # on the table beside the printer's right side, square to it
OLD_GOAL_Y = 0.38


def p2w(p):
    p = np.asarray(p, float)
    return p + np.array([PX, 0.0, TOP])


LOOKS = {
    "case": m.Look(color=m.rgb(0.80, 0.81, 0.80), roughness=0.55),
    "case_dark": m.Look(color=m.rgb(0.30, 0.31, 0.32), roughness=0.6),
    "bay": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.7),
    "door": m.Look(color=m.rgb(0.83, 0.84, 0.83), roughness=0.5),
    "tray": m.Look(color=m.rgb(0.62, 0.63, 0.64), roughness=0.55),
    "panel": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.35),
    "display": m.Look(color=m.rgb(0.25, 0.45, 0.55), roughness=0.2, emissive=(0.05, 0.12, 0.16)),
    "button": m.Look(color=m.rgb(0.55, 0.56, 0.57), roughness=0.5),
    "track": m.Look(color=m.rgb(0.18, 0.18, 0.19), roughness=0.6),
    "toner": m.Look(color=m.rgb(0.045, 0.045, 0.05), roughness=0.45),
    "endcap": m.Look(color=m.rgb(0.35, 0.36, 0.37), roughness=0.5),
    "drum": m.Look(color=m.rgb(0.13, 0.42, 0.22), roughness=0.3, metallic=0.2),
    "chip": m.Look(color=m.rgb(0.80, 0.66, 0.25), metallic=0.8, roughness=0.3),
    "label": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.6),
}


def _hull_box(name, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return m.col_box(name, tuple(hi - lo), tuple((hi + lo) / 2))


def _hull_mesh(points):
    """A closed mesh over the convex hull of `points`."""
    from scipy.spatial import ConvexHull
    v = np.asarray(points, float)
    h = ConvexHull(v)
    f = h.simplices.copy()
    c = v.mean(axis=0)
    for i, (a, b, k) in enumerate(f):  # outward winding
        if np.dot(np.cross(v[b] - v[a], v[k] - v[a]), v[a] - c) < 0:
            f[i] = (a, k, b)
    return m.Mesh(v, f)


def _ledge_hull(s):
    """One side's sloped ledge, as a convex point set: its top follows zc - r, with a 6 mm lead-in at the front and a taper in y."""
    yin, yout = s * LEDGE_Y0, s * LEDGE_Y1
    ytip = s * (LEDGE_Y1 - 0.004)
    pts = []
    for x, top, yi in ((LEDGE_X0, zc(LEDGE_X0) - BOSS_R - 0.006, ytip), (0.018, zc(0.018) - BOSS_R, yin), (LEDGE_X1, zc(LEDGE_X1) - BOSS_R, yin)):
        for y in (yi, yout):
            pts += [(x, y, top), (x, y, top - 0.010)]
    return _hull_mesh(pts)


def _ledge_mesh(s):
    return _ledge_hull(s)


def _printer_frame():
    parts, cols = [], []

    def solid(name, lo, hi, look, collide=True):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        parts.append((name, m.box(tuple(hi - lo), tuple((hi + lo) / 2)), look))
        if collide:
            cols.append(_hull_box(name + "_c", lo, hi))

    solid("base", (0.0, -HW, 0.0), (D, HW, BASE_H), "case")
    for s, tag in ((1, "l"), (-1, "r")):
        y0, y1 = sorted((s * WALL_IN, s * HW))
        solid(f"wall_{tag}", (0.0, y0, BASE_H), (D, y1, COVER_Z), "case")
        yi0, yi1 = sorted((s * (WALL_IN - 0.001), s * WALL_IN))
        solid(f"wall_in_{tag}", (0.004, yi0, BASE_H + 0.002), (BAY_BACK, yi1, COVER_Z - 0.002), "bay", collide=False)
        parts.append((f"ledge_{tag}", _ledge_mesh(s), "track"))
        cols.append(m.col_hull(f"ledge_{tag}_c", _ledge_hull(s)))
        ly0, ly1 = sorted((s * LEDGE_Y0, s * LEDGE_Y1))
        solid(f"notch_{tag}", (LEDGE_X1, ly0, BASE_H), (STOP_X, ly1, SEAT_BOSS[1] - BOSS_R - 0.0005), "track")
        sy0, sy1 = sorted((s * (BOSS_Y0 - 0.0), s * WALL_IN))
        solid(f"stop_{tag}", (STOP_X, sy0, BASE_H), (STOP_X + 0.02, sy1, 0.135), "track")
    solid("top", (FLANGE_D, -HW, COVER_Z), (D, HW, H), "case")
    solid("rear", (BAY_BACK, -WALL_IN, BASE_H), (D, WALL_IN, COVER_Z), "bay")
    solid("floor", (0.0, -WALL_IN, BASE_H - 0.001), (BAY_BACK, WALL_IN, BASE_H), "bay", collide=False)
    # what is seen only: the output bin on top, the control panel at the top right, tray 2's front, the power button
    solid("bin", (0.15, -0.085, H), (0.33, 0.13, H + 0.002), "case_dark", collide=False)
    solid("bin_stop", (0.15, -0.085, H), (0.162, 0.13, H + 0.012), "case", collide=False)
    solid("cpanel", (0.118, -0.182, H), (0.20, -0.098, H + 0.018), "panel", collide=False)
    solid("display", (0.13, -0.172, H + 0.018), (0.155, -0.110, H + 0.019), "display", collide=False)
    for k, y in enumerate((-0.170, -0.150, -0.130, -0.112)):
        parts.append((f"key_{k}", m.cylinder(0.004, 0.004, (0.175, y, H + 0.019)), "button"))
    solid("tray2_line", (-0.0008, -0.17, 0.068), (0.0, 0.17, 0.070), "case_dark", collide=False)
    solid("tray2_grip", (-0.0008, -0.06, 0.02), (0.0, 0.06, 0.035), "case_dark", collide=False)
    parts.append(("power", m.cylinder(0.006, 0.002, (-0.001, -0.165, 0.05), axis="x"), "button"))
    return parts, cols


# -- the door, authored open: meshes made shut, then turned about the hinge line
HINGE = np.array([0.0, 0.0, BASE_H])


def _open(mesh):
    return mesh.moved(-HINGE).turned(-DOOR_OPEN, "y").moved(HINGE)


def _door_shut_boxes():
    """(lo, hi) of the door's two slabs, shut, in P."""
    return [((-DOOR_T, -HW + 0.001, BASE_H + 0.001), (0.0, HW - 0.001, COVER_Z)),
            ((-DOOR_T, -HW + 0.001, COVER_Z), (FLANGE_D - 0.001, HW - 0.001, H))]


def _door_link():
    parts, cols = [], []
    for k, (lo, hi) in enumerate(_door_shut_boxes()):
        lo, hi = np.asarray(lo), np.asarray(hi)
        b = m.box(tuple(hi - lo), tuple((hi + lo) / 2))
        parts.append((f"slab{k}", _open(b), "door"))
        cols.append(m.col_hull(f"door_slab{k}_c", _open(b)))
    # Tray 1 folded into the door's inside, and the finger recess at its top edge (seen only)
    parts.append(("tray1", _open(m.box((0.002, 0.30, 0.085), (0.001, 0.0, BASE_H + 0.060))), "tray"))
    parts.append(("recess", _open(m.box((0.002, 0.10, 0.012), (-DOOR_T - 0.001, 0.0, COVER_Z - 0.012))), "case_dark"))
    com_shut = np.array([0.0079, 0.0, BASE_H + 0.0829])
    com = _open(m.Mesh(com_shut[None, :], np.zeros((0, 3), int))).v[0]  # Mesh.turned only rotates points
    return m.hinge("door", "Y", tuple(HINGE), 0.0, DOOR_OPEN, parts, cols, mass=DOOR_MASS, spring=0.0, damping=0.3, friction=DOOR_FRICTION,
                   effort=50.0, armature=0.002, gravity=True, com=tuple(com), inertia=(0.004, 0.004, 0.004))


RELEASE_AT = (0.035, -HW, 0.175)
RELEASE = m.button("release", RELEASE_AT, outward="-Y", look="button", radius=0.009, height=0.006, travel=0.004, spring=300.0)
_fp, _fc = _printer_frame()
PRINTER = m.mechanism("printer", (PX, 0.0, TOP), [_door_link(), RELEASE], LOOKS, frame_parts=_fp, frame_colliders=_fc, friction=(0.35, 0.3))


# -- the cartridges
def _handle_mesh():
    """The carry handle: a fin along the top of the body, leaning toward the front."""
    b = np.array([HANDLE[0], 0.0, HANDLE[1]])
    pts = []
    for s in (0.0, HANDLE[2]):
        for t in (-HANDLE[3] / 2, HANDLE[3] / 2):
            for y in (-HANDLE[4], HANDLE[4]):
                p = b + s * _HA + t * _HN
                pts.append((p[0], y, p[2]))
    return _hull_mesh(pts)


def _cartridge(name, pos, mass, label):
    bx, by, bz = BODY
    parts = [("body", m.box((bx, 2 * (BOSS_Y0 - 0.007), bz), (0, 0, bz / 2)), "toner")]
    for s in (1, -1):
        parts.append((f"cap{s:+d}", m.box((bx, 0.007, bz), (0, s * (BOSS_Y0 - 0.0035), bz / 2)), "endcap"))
        parts.append((f"boss{s:+d}", m.cylinder(BOSS_R, BOSS_Y1 - BOSS_Y0, (BOSS_C[0], s * (BOSS_Y0 + BOSS_Y1) / 2, BOSS_C[1]), axis="y"), "endcap"))
    parts.append(("handle", _handle_mesh(), "toner"))
    parts.append(("hopper", m.cylinder(0.035, 2 * (BOSS_Y0 - 0.008), (0.012, 0, bz - 0.032), axis="y"), "toner"))
    dx, dz, dr, dl = DRUM
    parts.append(("drum", m.cylinder(dr, 2 * dl, (dx + 0.004, 0, dz - 0.001), axis="y"), "drum"))
    parts.append(("chip", m.box((0.012, 0.002, 0.010), (-0.02, -BOSS_Y0 - 0.001, 0.05)), "chip"))
    parts.append(("label", m.box((0.001, 0.12, 0.035), (-bx / 2 - 0.0005, 0.04, 0.045)), label))
    cols = [m.col_box("body_c", (bx, 2 * BOSS_Y0, bz), (0, 0, bz / 2)),
            m.col_hull("handle_c", _handle_mesh())]
    for s in (1, -1):
        cols.append(m.col_hull(f"boss{s:+d}_c", m.cylinder(BOSS_R, BOSS_Y1 - BOSS_Y0, (BOSS_C[0], s * (BOSS_Y0 + BOSS_Y1) / 2, BOSS_C[1]), axis="y", segments=16)))
    return m.free(name, pos, parts, cols, LOOKS, mass=mass, friction=(0.5, 0.4), com=(0.0, 0.0, 0.042), solver_iters=32)


OLD = _cartridge("old", tuple(p2w(SEATED + [0, 0, 0.0005])), OLD_MASS, "label")
NEW = _cartridge("new", (NEW_START[0], NEW_START[1], TOP + 0.0005), NEW_MASS, "label")

# -- distractors (SimReady): a ream of printer paper, a mug, a tape dispenser
PAPER = SimReady("paper_ream", pos=(0.49, 0.40, TOP), size=(0.303, 0.212, 0.062), mass=2.5, package="misc_printerPaper_a01",
                 usd="sm_misc_printerPaper_a01_01.usd", yaw=math.pi / 2)
MUG = SimReady("mug", pos=(0.46, -0.31, TOP), size=(0.114, 0.090, 0.080), mass=0.35, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
               yaw=-0.6)
TAPE = SimReady("tape_dispenser", pos=(0.36, -0.48, TOP), size=(0.159, 0.064, 0.066), mass=0.4, package="misc_tape_dispenser_a01",
                usd="sm_misc_tape_dispenser_a01_01.usd")

OBJECTS = [PRINTER, OLD, NEW, PAPER, MUG, TAPE]


# -- geometry for the scorer -----------------------------------------------------------------------------------------------------------------
def _corners(lo, hi):
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


BODY_LO, BODY_HI = np.array([-BODY[0] / 2, -BOSS_Y0, 0.0]), np.array([BODY[0] / 2, BOSS_Y0, BODY[2]])
CART_LO, CART_HI = np.array([-BODY[0] / 2, -BOSS_Y1, 0.0]), np.array([BODY[0] / 2, BOSS_Y1, HANDLE[1] + HANDLE[2] * _HA[2] + 0.005])  # with handle and bosses
DRUM_LO = np.array([DRUM[0] - DRUM[2], -DRUM[3], -0.002])
DRUM_HI = np.array([DRUM[0] + DRUM[2] + 0.006, DRUM[3], DRUM[1] + DRUM[2]])
FOOT_LO, FOOT_HI = np.array([PX - 0.15, -HW]), np.array([PX + D, HW])  # the printer's footprint with its open door
CLEAR = 0.03  # fingertip clearance for "let go"
DRUM_TOUCH = 0.004
DROP_V = 0.6  # m/s downward: a free fall of 1.8 cm


def box_gap(points_local, lo, hi):
    d = np.maximum(np.maximum(lo - points_local, points_local - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def door_gap(printer_state, tips):
    q = tips - np.array([PX, 0.0, TOP])
    return float(min(box_gap(q, np.array(lo), np.array(hi)).min() for lo, hi in _door_shut_boxes()))


def quat_yaw(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def now(env):
    pr, old, new = m.state(env, "printer"), m.state(env, "old"), m.state(env, "new")
    tips = m.fingertips(env)
    c = {}
    c["door"] = pr.deg("door")
    c["release"] = pr.joint("release")
    # the old cartridge: on the table, upright, outside the printer's footprint, still, let go
    oc = old.to_world(_corners(BODY_LO, BODY_HI))
    lo2, hi2 = oc[:, :2].min(0), oc[:, :2].max(0)
    outside = bool(hi2[1] < FOOT_LO[1] or lo2[1] > FOOT_HI[1] or hi2[0] < FOOT_LO[0] or lo2[0] > FOOT_HI[0])
    bottom = float(oc[:, 2].min() - TOP)
    c["old_gap"] = float(box_gap(old.to_local(tips), CART_LO, CART_HI).min())
    c["old_still"] = bool(np.linalg.norm(old.lin_vel) < 0.02 and np.linalg.norm(old.ang_vel) < 0.2)
    c["old_ok"] = bool(outside and -0.003 <= bottom <= 0.005 and old.tilt_deg() <= 10.0 and c["old_still"] and c["old_gap"] >= CLEAR)
    c["old_bottom"], c["old_tilt"], c["old_outside"] = bottom, old.tilt_deg(), outside
    # the new cartridge: seated (pose within 3 mm and 2 degrees of the seated pose), still, let go
    off = new.pos - p2w(SEATED)
    c["new_off_mm"] = float(np.linalg.norm(off) * 1000)
    c["new_tilt"], c["new_yaw"] = new.tilt_deg(), quat_yaw(new.quat_xyzw)
    c["new_gap"] = float(box_gap(new.to_local(tips), CART_LO, CART_HI).min())
    c["new_still"] = bool(np.linalg.norm(new.lin_vel) < 0.02 and np.linalg.norm(new.ang_vel) < 0.2)
    c["seated"] = bool(c["new_off_mm"] <= 3.0 and c["new_tilt"] <= 2.0 and abs(c["new_yaw"]) <= 2.0)
    c["new_ok"] = bool(c["seated"] and c["new_still"] and c["new_gap"] >= CLEAR)
    c["drum_gap"] = float(box_gap(new.to_local(tips), DRUM_LO, DRUM_HI).min())
    c["door_gap"] = door_gap(pr, tips)
    c["fall"] = bool(min(old.lin_vel[2], new.lin_vel[2]) < -DROP_V)
    c["off_table"] = bool(min(old.pos[2], new.pos[2]) < TOP - 0.05)
    return c


class Trial:
    """The door's latch and the scorer's record. The record (dropped, drum touched) is cleared only in reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.latched = False
        self.dropped = False
        self.drum = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.c = {}
        self.n = 0

    def step(self, env):
        c = now(env)
        self.n += 1
        # the latch: it catches the door at shut; the release button lets it go
        if not self.latched and c["door"] >= LATCH_AT:
            m.lock(env, "printer", "door")
            self.latched = True
        elif self.latched and c["release"] > 0.0025:
            m.unlock(env, "printer", "door")
            self.latched = False
        c["latched"] = self.latched
        if c["fall"] or c["off_table"]:
            self.dropped = True
        if c["drum_gap"] < DRUM_TOUCH:
            self.drum = True
        door_ok = self.latched and c["new_ok"]
        ok = c["old_ok"] and c["new_ok"] and door_ok and c["door_gap"] >= CLEAR
        self.hold = self.hold + 1 if ok and not (self.dropped or self.drum) else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.dropped or self.drum:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = min(0.25 * c["old_ok"] + 0.5 * c["new_ok"] + 0.25 * door_ok, 0.75)
        self.c = c
        if _DEBUG and self.n % 5 == 0:
            _debug(env, self)


_DEBUG = os.environ.get("TONER_DEBUG")


def _debug(env, trial):
    import json
    from yam_arena import ik
    base = env.unwrapped
    row = {"n": trial.n, "score": trial.score, "latched": trial.latched, "dropped": trial.dropped, "drum": trial.drum}
    row.update({k: (round(v, 4) if isinstance(v, float) else v) for k, v in trial.c.items()})
    for name in ("old", "new"):
        s = m.state(env, name)
        row[name] = [round(float(v), 4) for v in s.pos] + [round(s.tilt_deg(), 2), round(quat_yaw(s.quat_xyzw), 2)]
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        q = m._np(robot.data.joint_pos)[0][:6]
        from yam_arena import station
        row["tcp_" + side] = [round(float(v), 4) for v in ik.tcp_pose(q, station.BASES[side])[0]]
    with open(_DEBUG, "a") as f:
        f.write(json.dumps(row) + "\n")


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration: a fixed plan from the published starting layout (open loop), played by yam_scripted ---------------------------------
def _Rx(a):
    a = math.radians(a)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _Ry(a):
    a = math.radians(a)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


# the fingers slide down the leaning handle: they point down and in, and close across it
GRIP_L = d._as_grip(_Ry(-HANDLE_LEAN) @ d.down(-90).rot)
GRIP_R = d._as_grip(_Ry(-HANDLE_LEAN) @ d.down(90).rot)
PUSH_L = d.down(-90)
F_RIDGE = -0.017  # the fingers 34 mm apart round the 16 mm handle
SAG = {"left": np.array([0.0, 0.0, 0.0]), "right": np.array([0.0, 0.0, 0.0])}  # extra command while carrying a cartridge (measured)


def tcp_for(origin_p, side, loaded=True):
    """The TCP target (world) that holds a cartridge's origin at `origin_p` (printer frame), level and square."""
    return tuple(float(v) for v in p2w(np.asarray(origin_p, float) + GRIP_C) + (SAG[side] if loaded else 0.0))


def boss_to_origin(bx, bz):
    return np.array([bx - BOSS_C[0], 0.0, bz - BOSS_C[1]])


UP_HANDLE = 0.05 * _HA  # where the fingers start, back up the handle's lean from the grip
CHANNEL = boss_to_origin(-0.033, zc(LEDGE_X0) + 0.006)  # drawn out of the bay, in front of it, just above the open door


def _approach(W, origin, side, label):
    W(2.0, tuple(np.array(tcp_for(origin, side, False)) + UP_HANDLE + [0, 0, 0.03]), F_RIDGE, f"over the {label} cartridge's handle")
    W(1.2, tuple(np.array(tcp_for(origin, side, False)) + UP_HANDLE), F_RIDGE, "fingers in line with the handle")
    W(1.2, tcp_for(origin, side, False), F_RIDGE, "fingers down round the handle")
    W(0.8, tcp_for(origin, side, False), FINGER_CLOSED, "grip the handle")


def _plan_left(mode):
    out = []
    W = lambda t, p, f, label, grip=GRIP_L: out.append(Waypoint(t, tuple(float(v) for v in p), f, label, grip))  # noqa: E731
    _approach(W, SEATED, "left", "old")
    W(0.8, tcp_for(SEATED + [0, 0, 0.008], "left"), FINGER_CLOSED, "lift the bosses out of the notches")
    for bx, t in ((0.086, 1.0), (0.050, 1.4), (LEDGE_X0, 1.4)):
        W(t, tcp_for(boss_to_origin(bx, zc(bx) + 0.004), "left"), FINGER_CLOSED, "draw it out along the tracks")
    W(1.0, tcp_for(CHANNEL, "left"), FINGER_CLOSED, "out in front of the bay")
    W(4.0, tcp_for(CHANNEL + [0, OLD_GOAL_Y, 0.002], "left"), FINGER_CLOSED, "carry it out to the left, above the open door")
    W(1.6, tcp_for([CHANNEL[0], OLD_GOAL_Y, 0.008], "left"), FINGER_CLOSED, "lower it onto the table")
    goal = np.array([CHANNEL[0], OLD_GOAL_Y, 0.003])
    W(0.6, tcp_for(goal, "left"), F_RIDGE, "let go")
    W(1.0, tuple(np.array(tcp_for(goal, "left", False)) + UP_HANDLE + [0, 0, 0.04]), F_RIDGE, "hand up off it")
    return out


def _plan_right(mode):
    out = []
    W = lambda t, p, f, label, grip=GRIP_R: out.append(Waypoint(t, tuple(float(v) for v in p), f, label, grip))  # noqa: E731
    o0 = np.array([NEW_START[0] - PX, NEW_START[1], 0.0])  # the new cartridge's origin in P, on the table
    _approach(W, o0, "right", "new")
    W(1.5, tcp_for([o0[0], o0[1], CHANNEL[2] + 0.004], "right"), FINGER_CLOSED, "lift it")
    W(max(0.1, 13.4 - sum(w.duration for w in out)), tcp_for([o0[0], o0[1], CHANNEL[2] + 0.004], "right"), FINGER_CLOSED, "wait for the bay to clear")
    W(4.0, tcp_for(CHANNEL + [0, 0, 0.004], "right"), FINGER_CLOSED, "carry it in front of the bay")
    W(1.2, tcp_for(boss_to_origin(-0.010, zc(LEDGE_X0) + 0.006), "right"), FINGER_CLOSED, "bosses at the tracks' lead-in")
    for bx, t in ((0.020, 1.0), (0.055, 1.4), (0.088, 1.4)):
        W(t, tcp_for(boss_to_origin(bx, zc(bx) + 0.003), "right"), FINGER_CLOSED, "slide it in along the tracks")
    W(0.8, tcp_for(SEATED + [0, 0, 0.004], "right"), FINGER_CLOSED, "over the seat")
    W(0.8, tcp_for(SEATED + [0, 0, 0.0005], "right"), FINGER_CLOSED, "bosses down into the notches")
    if mode == "hold":
        W(12.0, tcp_for(SEATED + [0, 0, 0.0005], "right"), FINGER_CLOSED, "keep holding it")
        return out
    W(0.6, tcp_for(SEATED + [0, 0, 0.0005], "right", False), F_RIDGE, "let go")
    W(1.0, tuple(np.array(tcp_for(SEATED, "right", False)) + UP_HANDLE), F_RIDGE, "fingers back up off the handle")
    W(1.5, (0.24, -0.26, TOP + 0.28), F_RIDGE, "hand away to the right")
    return out


def door_point(theta_deg, u, outer=True):
    """A point of the door's top strip, `u` back from the front face, on its outer (top) side, with the door `theta_deg` from shut
    (negative: open), in P; and the strip's outward normal there."""
    t = math.radians(theta_deg)
    uz = (H if outer else COVER_Z) - BASE_H
    p = np.array([uz * math.sin(t) + u * math.cos(t), 0.0, BASE_H + uz * math.cos(t) - u * math.sin(t)])
    return p, np.array([math.sin(t), 0.0, math.cos(t)])


FACE = 0.012  # the closed fingers' face, ahead of the TCP along x
DOOR_Y = 0.07
U_EDGE = FLANGE_D - 0.001  # the top strip's free edge
U_PRESS = 0.072


def _door_plan(start_t, plan):
    """The left hand shuts the door. The side of its closed fingers pushes the top strip's free edge, from the robot's side, while
    rising faster than the edge does, to 25 degrees from shut; then the fingertips press the strip down until the door latches."""
    out = list(plan)
    W = lambda t, p, f, label: out.append(Waypoint(t, tuple(float(v) for v in p), f, label, PUSH_L))  # noqa: E731

    def push_pt(th, below):
        c, _ = door_point(th, U_EDGE)
        return p2w(c + np.array([-FACE + 0.004, DOOR_Y, -below + 0.013]))

    c0, _ = door_point(-DOOR_OPEN, U_EDGE)
    W(1.5, p2w(c0 + np.array([-0.04, DOOR_Y, 0.06])), FINGER_CLOSED, "above, in front of the door's top strip")
    W(max(0.1, start_t - sum(w.duration for w in out)), p2w(c0 + np.array([-0.04, DOOR_Y, 0.06])), FINGER_CLOSED, "wait for the new cartridge to be in")
    W(1.0, push_pt(-DOOR_OPEN, 0.035) + np.array([-0.012, 0, 0]), FINGER_CLOSED, "fingers in front of the strip's edge")
    ths = [-80, -70, -60, -50, -40, -32, -25]
    for k, th in enumerate(ths):
        below = 0.035 - 0.029 * (k + 1) / len(ths)
        W(0.7, push_pt(th, below), FINGER_CLOSED, "push the strip's edge up and in")
    c, _ = door_point(-25, U_EDGE)
    W(0.5, p2w(c + np.array([-FACE - 0.006, DOOR_Y, 0.02 + 0.013])), FINGER_CLOSED, "up over the edge")
    p, n = door_point(-25, U_PRESS)
    W(0.6, p2w(p + 0.012 * n + np.array([0, DOOR_Y, 0.013])), FINGER_CLOSED, "fingertips over the top strip")
    for th in (-25, -18, -12, -6, -2, 1.5):
        p, n = door_point(th, U_PRESS)
        W(0.6, p2w(p - 0.003 * n + np.array([0, DOOR_Y, 0.013])), FINGER_CLOSED, "press the top strip down until it latches")
    p, n = door_point(0, U_PRESS)
    W(1.0, p2w(p + np.array([0, DOOR_Y, 0.07])), FINGER_CLOSED, "hand up off the door")
    W(1.5, (0.20, 0.30, TOP + 0.28), F_RIDGE, "hand away to the left")
    return out


def demo():
    mode = os.environ.get("TASK_DEMO", "full")
    left, right = _plan_left(mode), _plan_right(mode)
    t_in = sum(w.duration for w in right) - 1.5
    left = _door_plan(t_in, left)
    return {"left": left, "right": right}
