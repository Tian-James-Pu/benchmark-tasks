"""On the station table: a galley mug rail. A beech retaining rod (12 mm square, 38 cm long) runs left to right
across the table, 33 cm out from the arm bases with its centre 3.5 cm above the table, lying loose in two open-topped
oak brackets (U-shaped cradles, fixed to the table) 24 cm apart. The rod passes through the handles of two mugs that
stand on the table on the robot's side of it. The rod's right end is open, so a mug comes off only by travelling
along the rod and past that end.

The red mug, on the overhanging right part of the rod, was tugged towards the open end while twisted: it stands
turned 26 degrees with its handle cocked on the rod (the 12 mm rod just fits the 15.5 mm handle opening at that
angle), and the rod was dragged 3 cm to the right with it, so the rod's left end reaches only 1 cm into the left
bracket. Pulled further the same way the handle grips the rod and takes it along; 1 cm more and the rod drops out of
the left bracket. The blue mug, between the brackets, is square on the rod and should stay there.

Two distractors stand clear of the rail: a tin of tuna beyond the rod and a kitchen sponge near the robot.
"""

import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from simready import SimReady  # noqa: E402
import mechkit as m  # noqa: E402  only for its reset/step hooks and fingertips(); no kit parts in this scene

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The mug is the YCB mug scan scaled to 10 cm from handle tip to the far side of the body: body 7.1 cm across and
# 7.1 cm tall, handle opening 1.55 cm wide and 4.1 cm tall.
MUG_EXTENT = 0.10
K = MUG_EXTENT / 0.116
HOLE_DX = 0.041 * K  # the handle opening's centre, from the mug's bounding-box centre, along the handle
BODY_DX = -0.011 * K  # the round body's centre, on the same axis
GRIP_DX = HOLE_DX - BODY_DX  # body centre to handle opening
MUG_H = 0.082 * K

ROD_X = 0.33
ROD_Z = TOP + 0.041 * K  # mid-height of the handle opening
ROD_SIZE = (0.012, 0.38, 0.012)
SHIFT = 0.03  # how far right of its seat the rod has been dragged
LEFT_Y, RIGHT_Y = 0.18, -0.06  # bracket centres
BRACKET_W = 0.02  # along the rod
LEFT_IN, LEFT_OUT = LEFT_Y - BRACKET_W / 2, LEFT_Y + BRACKET_W / 2  # the left bracket's inner and outer faces
RIGHT_IN = RIGHT_Y + BRACKET_W / 2  # the right bracket's inner face
ROD_Y = LEFT_Y - ROD_SIZE[1] / 2  # centre at the start: the left end is level with the middle of the left bracket
SEAT_H = ROD_Z - ROD_SIZE[2] / 2 - TOP
CHEEK_H = SEAT_H + 0.012

BEECH = (0.87, 0.74, 0.53)
OAK = (0.50, 0.36, 0.22)


def _bracket(tag, y):
    return [
        Box(f"{tag}_seat", pos=(ROD_X, y, TOP + SEAT_H / 2), size=(0.016, BRACKET_W, SEAT_H), color=OAK),
        Box(f"{tag}_front", pos=(ROD_X - 0.011, y, TOP + CHEEK_H / 2), size=(0.006, BRACKET_W, CHEEK_H), color=OAK),
        Box(f"{tag}_back", pos=(ROD_X + 0.011, y, TOP + CHEEK_H / 2), size=(0.006, BRACKET_W, CHEEK_H), color=OAK),
    ]


def _mug_xy(hole_y, yaw):
    """Where the mug's bounding-box centre goes for its handle opening to sit round the rod at ``hole_y``."""
    return (ROD_X - HOLE_DX * np.cos(yaw), hole_y - HOLE_DX * np.sin(yaw))


def _body_xy(hole_y, yaw, dx=0.0, dy=0.0):
    return (ROD_X - GRIP_DX * np.cos(yaw) + dx, hole_y - GRIP_DX * np.sin(yaw) + dy)


TWIST = np.radians(26.0)
RED_HOLE_Y, BLUE_HOLE_Y = -0.125, 0.03
ROD = Box("rod", pos=(ROD_X, ROD_Y, ROD_Z + 0.002), size=ROD_SIZE, mass=0.03, color=BEECH)
RED = Asset("red_mug", "mug_ycb_robolab", xy=_mug_xy(RED_HOLE_Y, TWIST), extent=MUG_EXTENT, mass=0.15, yaw=TWIST)
BLUE = Asset("blue_mug", "mug_ycb_robolab", xy=_mug_xy(BLUE_HOLE_Y, 0.0), extent=MUG_EXTENT, mass=0.15,
             color=(0.15, 0.30, 0.70))
# Distractors (NVIDIA SimReady props), loose, outside the arms' paths; score() ignores them.
DISTRACTORS = [
    SimReady("tuna_can", pos=(0.46, -0.02, TOP), size=(0.086, 0.086, 0.034), mass=0.17,
             package="food_can_tunaFish_a01", usd="sm_food_can_tunaFish_a01_01.usda"),
    SimReady("sponge", pos=(0.16, 0.03, TOP), size=(0.071, 0.101, 0.033), mass=0.015,
             package="cleaning_sponge_b01", usd="sm_cleaning_sponge_b01_01.usd"),
]
OBJECTS = _bracket("left", LEFT_Y) + _bracket("right", RIGHT_Y) + [ROD, RED, BLUE] + DISTRACTORS

SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
BLUE_START = np.array(_mug_xy(BLUE_HOLE_Y, 0.0))
CLEAR = 0.02  # the red mug's gap to the rod, seen from above
HANDS_OFF = 0.03  # every fingertip at least this far from the red mug's and the rod's outlines
_harm = False  # an "ever happened" harm, kept for the whole trial
_tips = None  # both grippers' fingertip points at this control step (from mechkit's step hook)


def _reset(env=None):
    """The only place the recorded harm is cleared: mechkit calls this after every environment reset."""
    global _harm, _tips
    _harm = False
    _tips = None


def _step(env):
    global _tips
    _tips = m.fingertips(env)


m.on_reset(_reset)
m.on_step(_step)


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _tilt_deg(obj):
    """Angle between the object's own vertical axis and vertical."""
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _rod_slope_deg(rod):
    """How far the rod's length is from level."""
    a = _rotate(rod.quat_xyzw, np.array([0.0, 1.0, 0.0]))
    return float(np.degrees(np.arcsin(np.clip(abs(a[2]), 0.0, 1.0))))


def _gap(mug, rod):
    """Horizontal gap between the mug and the rod as seen from above (m); 0 when they overlap."""
    dx = max(rod.low[0] - mug.high[0], mug.low[0] - rod.high[0])
    dy = max(rod.low[1] - mug.high[1], mug.low[1] - rod.high[1])
    return float(max(dx, dy, 0.0))


def _hands_off(*objs):
    """True when no fingertip of either gripper is within HANDS_OFF of any of the objects' outlines."""
    if _tips is None:
        return False
    for o in objs:
        lo, hi = np.asarray(o.low, float), np.asarray(o.high, float)
        d = np.linalg.norm(np.maximum(np.maximum(lo - _tips, _tips - hi), 0.0), axis=1)
        if float(d.min()) < HANDS_OFF:
            return False
    return True


def _track(objects):
    """Remember, for the whole trial, whether the rod came out of a bracket or a mug went over. Only a reset clears it."""
    global _harm
    rod, red, blue = objects["rod"], objects["red_mug"], objects["blue_mug"]
    rod_out = _rod_slope_deg(rod) > 5 or rod.high[1] < LEFT_IN or rod.low[1] > RIGHT_IN
    _harm = bool(_harm or rod_out or _tilt_deg(blue) > 20 or _tilt_deg(red) > 45)


def score(objects):
    """0 if the rod ever sloped more than 5 degrees or had an end inside the span between the brackets' inner faces
    (it came out of a bracket), the blue mug ever tilted more than 20 degrees or the red mug more than 45. Otherwise
    the red mug must stand upright and still on the table at least 2 cm clear of the rod as seen from above, with the
    rod still and level in both brackets, the blue mug upright within 5 cm of where it started and no fingertip within
    3 cm of the red mug or the rod (both let go): 1.0 if also the
    rod's left end is at or past the left bracket's outer face and the blue mug is within 2 cm of its start, else
    0.5."""
    _track(objects)
    if _harm:
        return 0.0
    rod, red, blue = objects["rod"], objects["red_mug"], objects["blue_mug"]
    red_off = bool(
        _gap(red, rod) >= CLEAR and _tilt_deg(red) < 5 and red.low[2] < TOP + 0.005 and red.resting()
    )
    rod_seated = bool(
        _rod_slope_deg(rod) < 3
        and abs(rod.pos[2] - ROD_Z) < 0.003
        and abs(rod.pos[0] - ROD_X) < 0.004
        and rod.resting()
    )
    blue_moved = float(np.linalg.norm(blue.pos[:2] - BLUE_START))
    blue_ok = bool(_tilt_deg(blue) < 5 and blue_moved <= 0.05 and blue.resting())
    if not (red_off and rod_seated and blue_ok and _hands_off(red, rod)):
        return 0.0
    return 1.0 if rod.high[1] >= LEFT_OUT and blue_moved <= 0.02 else 0.5


def success(objects):
    return score(objects) == 1.0


HOLD_Y = 0.105  # where the left gripper pinches the rod, between the blue mug and the left bracket
ACROSS = {"left": -np.pi / 2, "right": np.pi / 2}  # fingers closing along x, across the rod


def demo():
    """The left arm pinches the rod between the blue mug and the left bracket and holds it. The right arm grips the
    red mug across its body, lifts it 6 mm, turns it square about the rod, carries it along the rod past the open
    end and sets it down nearer the robot. The left arm then pushes the rod 4 cm back through the left bracket and
    lets go."""
    hi = TOP + 0.20
    grip_z = TOP + MUG_H - 0.02
    carry_z = grip_z + 0.006
    rod_tcp = ROD_Z + 0.005
    yaw_l = ACROSS["left"]
    left = [
        Waypoint(2.0, (ROD_X, HOLD_Y, TOP + 0.14), FINGER_OPEN, "above the rod", yaw_l),
        Waypoint(1.5, (ROD_X, HOLD_Y, rod_tcp), FINGER_OPEN, "descend", yaw_l),
        Waypoint(1.0, (ROD_X, HOLD_Y, rod_tcp), FINGER_CLOSED, "pinch the rod", yaw_l),
        Waypoint(12.5, (ROD_X, HOLD_Y, rod_tcp), FINGER_CLOSED, "hold the rod", yaw_l),
        Waypoint(2.0, (ROD_X, HOLD_Y + 0.04, rod_tcp), FINGER_CLOSED, "push the rod back", yaw_l),
        Waypoint(1.0, (ROD_X, HOLD_Y + 0.04, rod_tcp), FINGER_OPEN, "let go", yaw_l),
        Waypoint(1.5, (ROD_X, HOLD_Y + 0.04, TOP + 0.14), FINGER_OPEN, "retreat", yaw_l),
    ]

    def at(yaw, z, hole_y=RED_HOLE_Y, dx=0.0):
        x, y = _body_xy(hole_y, yaw, dx)
        return (x, y, z)

    off_y = ROD_Y - ROD_SIZE[1] / 2 - 0.03  # handle opening 3 cm past the rod's open end
    right = [
        Waypoint(3.0, at(TWIST, hi), FINGER_OPEN, "above the red mug", TWIST),
        Waypoint(2.0, at(TWIST, grip_z), FINGER_OPEN, "descend", TWIST),
        Waypoint(1.0, at(TWIST, grip_z), FINGER_CLOSED, "grip", TWIST),
        Waypoint(0.8, at(TWIST, carry_z), FINGER_CLOSED, "lift 6 mm", TWIST),
    ]
    for frac in (2 / 3, 1 / 3, 0.0):
        right.append(Waypoint(0.9, at(TWIST * frac, carry_z), FINGER_CLOSED, "turn square", TWIST * frac))
    right += [
        Waypoint(3.5, at(0.0, carry_z, off_y), FINGER_CLOSED, "along the rod, off its end", 0.0),
        Waypoint(2.0, at(0.0, carry_z, off_y, -0.07), FINGER_CLOSED, "towards the robot", 0.0),
        Waypoint(1.0, at(0.0, grip_z + 0.001, off_y, -0.07), FINGER_CLOSED, "set down", 0.0),
        Waypoint(1.0, at(0.0, grip_z + 0.001, off_y, -0.07), FINGER_OPEN, "let go", 0.0),
        Waypoint(1.5, at(0.0, hi, off_y, -0.07), FINGER_OPEN, "retreat", 0.0),
    ]
    return {"left": left, "right": right}
