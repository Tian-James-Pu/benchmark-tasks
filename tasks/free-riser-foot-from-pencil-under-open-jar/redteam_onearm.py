"""On the station table: a dark-stained oak riser (26 x 7 x 4 cm) lies on the midline, its long side pointing away from
the arms, with an open glass jar of varnish (a SimReady drinking glass) standing on it. A pencil lies across the table to the right with
its tip caught 1.5 cm under the riser's right edge, so the riser does not sit flat: it rests on its left edge
and the pencil, and the jar leans with it. The loaded riser pins the pencil, so pulling the pencil drags the
riser and the jar along instead of freeing it.
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from mechkit import fingertips, on_reset, on_step  # noqa: E402
from simready import SimReady  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

RISER_SIZE = (0.26, 0.07, 0.04)
RISER_XY = (0.27, 0.0)
PENCIL_SIZE = (0.007, 0.175, 0.007)
PENCIL_X = 0.25  # 11 cm behind the riser's front end
UNDER = 0.015  # how far the pencil's tip reaches under the riser's right edge
PENCIL_Y = -RISER_SIZE[1] / 2 + UNDER - PENCIL_SIZE[1] / 2
JAR_XY = (0.31, 0.0)
JAR_H = 0.10  # the glass's height; its reported position is its centre

# The riser is spawned level, just above the pencil tip, and rocks down onto its left edge while the scene
# settles; the jar is spawned standing on it.
RISER_Z0 = TOP + PENCIL_SIZE[2] + RISER_SIZE[2] / 2 + 0.001
RISER = Box("riser", pos=(*RISER_XY, RISER_Z0), size=RISER_SIZE, mass=0.50, color=(0.27, 0.17, 0.10))
PENCIL = Box(
    "pencil", pos=(PENCIL_X, PENCIL_Y, TOP + PENCIL_SIZE[2] / 2), size=PENCIL_SIZE, mass=0.006, color=(0.95, 0.78, 0.10)
)
# The jar is a SimReady drinking glass, spawned by the centre of its bottom.
JAR = SimReady(
    "jar", pos=(*JAR_XY, RISER_Z0 + RISER_SIZE[2] / 2 + 0.002), size=(0.066, 0.066, 0.10), mass=0.30, convex=True,
    package="dishware_glass_8oz_a01", usd="sm_dishware_glass_8oz_a01_01.usd",
)
# Distractors beyond the riser's far end, which scoring ignores.
MUG = SimReady(
    "mug", pos=(0.48, 0.25, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=np.pi,
    package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
)
TAPE = SimReady(
    "tape_roll", pos=(0.51, 0.12, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
    package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
)
SCREWDRIVER = SimReady(
    "screwdriver", pos=(0.48, -0.10, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
    package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
)
OBJECTS = [RISER, PENCIL, JAR, MUG, TAPE, SCREWDRIVER]

SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
FLAT = 0.002  # every bottom corner of the riser within this of the table
MOVED = 0.01  # the riser's centre from where it started
TURNED = 5.0  # degrees, the riser's long side from straight away from the arms
CLEAR = 0.02  # between the pencil and the riser, seen from above
JAR_SPILL = 15.0  # degrees from vertical at which the varnish runs over the rim
JAR_FOOT_TOL = 0.004  # the jar's lowest point within 4 mm of the riser's top face: standing on it
HANDS_OFF = 0.02  # "let go": no point on either gripper's fingers within 2 cm of the riser, the jar or the pencil
_harm = False  # the jar tipped past JAR_SPILL at some step of this trial
_tips = None  # points on both grippers' fingers at the latest step


@on_reset
def _new_trial(environment=None):
    """The record of a spill is cleared only by a reset of the scene, never by anything that happens in a trial."""
    global _harm, _tips
    _harm, _tips = False, None


@on_step
def _see_grippers(environment):
    global _tips
    _tips = fingertips(environment)


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _unrotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    return _rotate(np.array([-x, -y, -z, w]), v)


def _tilt_deg(obj):
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _corner_heights(riser):
    """Height of each of the riser's four bottom corners above the table (m)."""
    hx, hy, hz = (s / 2 for s in RISER_SIZE)
    return [
        float((riser.pos + _rotate(riser.quat_xyzw, np.array([sx * hx, sy * hy, -hz])))[2] - TOP)
        for sx in (-1, 1)
        for sy in (-1, 1)
    ]


def _riser_turn_deg(riser):
    axis = _rotate(riser.quat_xyzw, np.array([1.0, 0.0, 0.0]))
    return float(abs(np.degrees(np.arctan2(axis[1], abs(axis[0])))))


def _pencil_gap(objects):
    """Shortest distance, seen from above, from the pencil to the riser's outline (m); 0 when any part of the
    pencil is under or over the riser."""
    riser, pencil = objects["riser"], objects["pencil"]
    half = np.array([0.0, PENCIL_SIZE[1] / 2, 0.0])
    gap = np.inf
    for s in np.linspace(-1.0, 1.0, 29):
        p = _unrotate(riser.quat_xyzw, pencil.pos + _rotate(pencil.quat_xyzw, s * half) - riser.pos)
        d = np.maximum(np.abs(p[:2]) - np.array(RISER_SIZE[:2]) / 2, 0.0)
        gap = min(gap, float(np.linalg.norm(d)))
    return max(0.0, gap - PENCIL_SIZE[0] / 2)


def _jar_on_riser(objects):
    """The jar stands on the riser's top face: the centre of its base inside the top's outline and its lowest point
    within 4 mm of the face."""
    riser, jar = objects["riser"], objects["jar"]
    base = jar.pos - _rotate(jar.quat_xyzw, np.array([0.0, 0.0, JAR_H / 2]))
    p = _unrotate(riser.quat_xyzw, base - riser.pos)
    top = float(riser.pos[2]) + RISER_SIZE[2] / 2
    return bool(
        abs(p[0]) <= RISER_SIZE[0] / 2 and abs(p[1]) <= RISER_SIZE[1] / 2 and abs(float(jar.low[2]) - top) <= JAR_FOOT_TOL
    )


def _box_gap(obj, size):
    """The smallest distance (m) from any fingertip point to the object's box, or 0 before the first step."""
    if _tips is None:
        return 0.0
    local = np.array([_unrotate(obj.quat_xyzw, p - obj.pos) for p in _tips])
    return float(np.linalg.norm(np.maximum(np.abs(local) - np.array(size) / 2, 0.0), axis=1).min())


def _aabb_gap(obj):
    """The smallest distance (m) from any fingertip point to the object's world bounding box."""
    if _tips is None:
        return 0.0
    lo, hi = np.asarray(obj.low, dtype=float), np.asarray(obj.high, dtype=float)
    return float(np.linalg.norm(np.maximum(np.maximum(lo - _tips, _tips - hi), 0.0), axis=1).min())


def _hands_off(objects):
    return bool(
        _box_gap(objects["riser"], RISER_SIZE) > HANDS_OFF
        and _box_gap(objects["pencil"], PENCIL_SIZE) > HANDS_OFF
        and _aabb_gap(objects["jar"]) > HANDS_OFF
    )


def _track(objects):
    """Remember, for the whole trial, whether the jar ever tilted far enough to spill."""
    global _harm
    _harm = bool(_harm or _tilt_deg(objects["jar"]) > JAR_SPILL)


def score(objects):
    """0 if the jar ever tilted more than 15 degrees. Otherwise, with the riser, jar and pencil at rest and no
    gripper finger within 2 cm of any of them: 1.0 for the riser flat (all four bottom corners within 2 mm of the
    table), within 1 cm and 5 degrees of where it started, the jar standing upright on it (within 3 degrees) and the
    pencil at least 2 cm clear of the riser; 0.5 if the riser is flat with the jar upright on it and no part of the
    pencil under or on it, but the riser moved more than 1 cm or turned more than 5 degrees, or the pencil lies
    within 2 cm of it; 0 otherwise."""
    _track(objects)
    if _harm:
        return 0.0
    riser, pencil, jar = objects["riser"], objects["pencil"], objects["jar"]
    still = riser.resting() and pencil.resting() and jar.resting()
    flat = max(abs(h) for h in _corner_heights(riser)) <= FLAT
    gap = _pencil_gap(objects)
    if not (still and flat and _tilt_deg(jar) <= 3 and _jar_on_riser(objects) and gap > 0.0 and _hands_off(objects)):
        return 0.0
    in_place = (
        np.linalg.norm(riser.pos[:2] - np.array(RISER_XY)) <= MOVED and _riser_turn_deg(riser) <= TURNED
    )
    return 1.0 if in_place and gap >= CLEAR else 0.5


def success(objects):
    return score(objects) == 1.0


GRIP_X = 0.165  # the left gripper holds the riser across its width, 2.5 cm behind its front end
GRIP_Y = 0.003  # the leaning riser's centreline at grip height, so closing the fingers does not turn it
GRIP_Z = TOP + 0.032
RAISE = 0.025
PENCIL_GRIP_Y = -0.15
PENCIL_GRIP_Z = TOP + 0.015
PENCIL_AWAY = 0.07  # how far right the pencil is carried
ALONG_X = np.pi / 2  # right arm: fingers close along x, across the pencil


def demo():
    """The left arm grips the riser's front end across its width and raises it 2.5 cm, pivoting it on its far
    end, and holds it there; the right arm takes the freed pencil away to the right; the left arm lowers the
    riser flat and lets go."""
    travel = TOP + 0.14
    hold, up = (GRIP_X, GRIP_Y, GRIP_Z), (GRIP_X, GRIP_Y, GRIP_Z + RAISE)
    left = [
        Waypoint(2.0, (GRIP_X, GRIP_Y, travel), FINGER_OPEN, "above the riser's front end"),
        Waypoint(1.5, hold, FINGER_OPEN, "descend"),
        Waypoint(1.5, hold, FINGER_CLOSED, "grip across the width"),
        Waypoint(2.0, up, FINGER_CLOSED, "raise the front end"),
        Waypoint(5.5, up, FINGER_CLOSED, "hold while the pencil is taken"),
        Waypoint(2.5, (GRIP_X, GRIP_Y, GRIP_Z - 0.004), FINGER_CLOSED, "lower flat"),
        Waypoint(1.0, (GRIP_X, GRIP_Y, GRIP_Z - 0.004), FINGER_OPEN, "let go"),
        Waypoint(1.5, (GRIP_X, GRIP_Y, travel), FINGER_OPEN, "retreat"),
    ]
    px, py = PENCIL_X, PENCIL_GRIP_Y
    right = [
        Waypoint(2.0, (px, py, travel), FINGER_OPEN, "above the pencil", ALONG_X),
        Waypoint(3.0, (px, py, TOP + 0.05), FINGER_OPEN, "wait", ALONG_X),
        Waypoint(2.0, (px, py, PENCIL_GRIP_Z), FINGER_OPEN, "descend", ALONG_X),
        Waypoint(1.0, (px, py, PENCIL_GRIP_Z), FINGER_CLOSED, "grip the pencil", ALONG_X),
        Waypoint(2.0, (px, py - PENCIL_AWAY, PENCIL_GRIP_Z + 0.01), FINGER_CLOSED, "take it away", ALONG_X),
        Waypoint(0.8, (px, py - PENCIL_AWAY, PENCIL_GRIP_Z + 0.003), FINGER_CLOSED, "set it down", ALONG_X),
        Waypoint(0.7, (px, py - PENCIL_AWAY, PENCIL_GRIP_Z + 0.003), FINGER_OPEN, "let go", ALONG_X),
        Waypoint(1.5, (px, py - PENCIL_AWAY, travel), FINGER_OPEN, "retreat", ALONG_X),
    ]
    return {"left": left, "right": [Waypoint(5.0, (0.2, -0.15, TOP + 0.2), FINGER_OPEN, "idle")]}
