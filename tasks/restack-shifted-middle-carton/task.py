"""On the station table: a stack of three loose cartons, each 14 x 5 x 4 cm, lying lengthwise away from the
arms on the midline. The brown bottom carton and the white top carton (glass vials) are in line; the blue middle
carton has slid 5 cm away from the arms, so it sticks out of the far side of the stack and the top carton
rests on only 9 cm of it, with 5 cm of its near end over nothing. Behind the stack, out of the way: a ceramic
mug, a roll of clear tape and a screwdriver, which are NVIDIA SimReady props (CC-BY 4.0) loaded by simready.py.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

X = 0.24  # the stack's centre, in front of the arm bases, on the midline
SIZE = (0.14, 0.05, 0.04)
H = SIZE[2]
SHIFT = 0.05  # how far the middle carton has slid away from the arms

BOTTOM = Box("bottom_carton", pos=(X, 0.0, TOP + 0.5 * H), size=SIZE, mass=0.30, color=(0.60, 0.44, 0.28))
MIDDLE = Box("middle_carton", pos=(X + SHIFT, 0.0, TOP + 1.5 * H + 0.001), size=SIZE, mass=0.30, color=(0.20, 0.42, 0.72))
TOPC = Box("top_carton", pos=(X, 0.0, TOP + 2.5 * H + 0.002), size=SIZE, mass=0.30, color=(0.93, 0.93, 0.90))
# Distractors behind the stack, which scoring ignores. pos is each prop's bottom centre.
MUG = SimReady(
    "mug", pos=(0.48, 0.24, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=np.pi,
    package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
)
TAPE = SimReady(
    "tape_roll", pos=(0.49, -0.24, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
    package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
)
SCREWDRIVER = SimReady(
    "screwdriver", pos=(0.48, 0.0, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
    package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
)
OBJECTS = [BOTTOM, MIDDLE, TOPC, MUG, TAPE, SCREWDRIVER]

SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
BOTTOM_START = np.array(BOTTOM.pos[:2])
TIP_DEG = 20.0  # the top carton tilting this far at any time counts as tipping it
_harm = False


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _tilt_deg(obj):
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(abs(up[2]), -1.0, 1.0))))


def _tip_deg(obj):
    """Tilt of the carton's own top face from level, 0 to 180 (upside down)."""
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _offset(upper, lower):
    """Horizontal distance between two cartons' centres (m)."""
    return float(np.linalg.norm(upper.pos[:2] - lower.pos[:2]))


def _skew_deg(upper, lower):
    """Angle between two cartons' long edges, seen from above (0 to 90 degrees)."""
    a = _rotate(upper.quat_xyzw, np.array([1.0, 0.0, 0.0]))[:2]
    b = _rotate(lower.quat_xyzw, np.array([1.0, 0.0, 0.0]))[:2]
    c = abs(float(a @ b)) / max(float(np.linalg.norm(a) * np.linalg.norm(b)), 1e-9)
    return float(np.degrees(np.arccos(np.clip(c, 0.0, 1.0))))


def _lies_on(upper, lower):
    return bool(abs(upper.pos[2] - (lower.pos[2] + H)) < 0.01 and _tilt_deg(upper) < 3)


def _track(objects):
    """Remember, for the whole trial, whether the top carton was ever tipped."""
    global _harm
    bottom, middle, top = (objects[n] for n in ("bottom_carton", "middle_carton", "top_carton"))
    at_start = (
        float(np.linalg.norm(bottom.pos[:2] - BOTTOM_START)) < 0.005
        and abs(middle.pos[0] - bottom.pos[0] - SHIFT) < 0.005
        and _offset(top, bottom) < 0.005
        and _lies_on(middle, bottom)
        and _lies_on(top, middle)
        and _tip_deg(top) < 1
        and all(o.resting() for o in (bottom, middle, top))
    )
    if at_start:
        _harm = False
    _harm = bool(_harm or _tip_deg(top) > TIP_DEG)


def score(objects):
    """0 if the top carton ever tilted more than 20 degrees. Otherwise, with the middle carton lying on the
    bottom one and the top carton on the middle one, all three level and at rest on the table top: 1.0 if each
    carton's centre is within 1.5 cm of the centre of the one below, its long edges within 10 degrees of the one
    below, and the bottom carton within 2 cm of where it started; 0.5 if the centres are within 3 cm, the long
    edges within 20 degrees and the bottom carton within 5 cm of its start; else 0."""
    _track(objects)
    if _harm:
        return 0.0
    bottom, middle, top = (objects[n] for n in ("bottom_carton", "middle_carton", "top_carton"))
    stacked = (
        abs(bottom.pos[2] - (TOP + 0.5 * H)) < 0.01
        and _tilt_deg(bottom) < 3
        and _lies_on(middle, bottom)
        and _lies_on(top, middle)
        and all(o.resting() for o in (bottom, middle, top))
    )
    if not stacked:
        return 0.0
    off = max(_offset(middle, bottom), _offset(top, middle))
    skew = max(_skew_deg(middle, bottom), _skew_deg(top, middle))
    moved = float(np.linalg.norm(bottom.pos[:2] - BOTTOM_START))
    if off <= 0.015 and skew <= 10 and moved <= 0.02:
        return 1.0
    if off <= 0.03 and skew <= 20 and moved <= 0.05:
        return 0.5
    return 0.0


def success(objects):
    return score(objects) == 1.0


# Measured in the simulator: with the fingers closing along y, a carton gripped on its centre line ends up held
# about 2 cm to the right of the commanded point and moves back 0.5 to 1 cm on release, so the demo sets each
# carton down with the gripper commanded 1.5 cm left of the stack's centre line.
PLACE_Y = 0.015


def demo():
    """The left arm lifts the top carton off and holds it in the air to the left; the right arm lifts the middle
    carton, brings it back over the bottom one and lets go; the left arm sets the top carton back."""
    high = TOP + 0.16
    held = TOP + 0.16
    top_z = TOP + 2.5 * H - 0.005
    mid_z = TOP + 1.5 * H
    g = PLACE_Y
    left = [
        Waypoint(2.0, (X, 0.0, high), FINGER_OPEN, "above the top carton"),
        Waypoint(1.5, (X, 0.0, top_z), FINGER_OPEN, "descend"),
        Waypoint(1.0, (X, 0.0, top_z), FINGER_CLOSED, "grip the top carton"),
        Waypoint(1.5, (X, 0.0, held), FINGER_CLOSED, "lift it off"),
        Waypoint(1.5, (X, 0.17, held), FINGER_CLOSED, "carry it left"),
        Waypoint(10.5, (X, 0.17, held), FINGER_CLOSED, "hold it while the middle carton is put back"),
        Waypoint(1.5, (X, g, held), FINGER_CLOSED, "bring it back over the stack"),
        Waypoint(2.0, (X, g, top_z + 0.002), FINGER_CLOSED, "lower it onto the middle carton"),
        Waypoint(1.0, (X, g, top_z + 0.002), FINGER_OPEN, "let go"),
        Waypoint(1.0, (X, g, high), FINGER_OPEN, "rise"),
        Waypoint(1.5, (X, 0.15, high), FINGER_OPEN, "retreat"),
    ]
    right = [
        Waypoint(2.0, (X + SHIFT, -0.18, high), FINGER_OPEN, "wait to the right"),
        Waypoint(5.5, (X + SHIFT, -0.18, high), FINGER_OPEN, "wait"),
        Waypoint(1.5, (X + SHIFT, 0.0, TOP + 0.15), FINGER_OPEN, "above the middle carton"),
        Waypoint(1.5, (X + SHIFT, 0.0, mid_z), FINGER_OPEN, "descend"),
        Waypoint(1.0, (X + SHIFT, 0.0, mid_z), FINGER_CLOSED, "grip the middle carton"),
        Waypoint(1.0, (X + SHIFT, g, mid_z + 0.015), FINGER_CLOSED, "lift it clear"),
        Waypoint(1.0, (X, g, mid_z + 0.015), FINGER_CLOSED, "bring it back in line"),
        Waypoint(1.0, (X, g, mid_z + 0.002), FINGER_CLOSED, "lower it onto the bottom carton"),
        Waypoint(1.0, (X, g, mid_z + 0.002), FINGER_OPEN, "let go"),
        Waypoint(1.0, (X, g, TOP + 0.15), FINGER_OPEN, "rise"),
        Waypoint(1.5, (X, -0.18, high), FINGER_OPEN, "retreat"),
    ]
    return {"left": left, "right": right}

