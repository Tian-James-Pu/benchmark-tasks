"""On the station table: a pine shelf board (40 x 6 x 1.8 cm) spans two fixed grey steel brackets, 8 cm high,
with a full tin of soup standing on its left half. The right-hand bracket has just been replaced; while it was out, a
red timber prop (12 x 4 x 8.8 cm, 8 mm taller than the brackets) was stood under the shelf 8 cm to the left of
that bracket, and it is still there. The shelf rests on the prop and on the left bracket, 8 mm clear of the new
right bracket, and the prop blocks the space under the shelf. The prop's front 6 cm sticks out in front of the
shelf, toward the arms. A blue rubber mat in front of the right arm is where the prop is kept. Nothing is fixed
to anything else except the brackets and the mat, which are fixed to the table. A mug, a roll of tape and a
screwdriver lie behind the shelf as distractors. The tin and the distractors are NVIDIA SimReady props; the rest
are boxes.
"""

import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from mechkit import fingertips, on_reset, on_step  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

X = 0.30  # the shelf and both brackets lie on this line across the table
BRACKET_Y = 0.18
BRACKET_SIZE = (0.06, 0.04, 0.08)
SHELF_SIZE = (0.06, 0.40, 0.018)
PROP_SIZE = (0.12, 0.04, 0.088)
PROP_XY = (0.27, -0.10)
TIN_D, TIN_H, TIN_Y = 0.067, 0.102, 0.15
MAT_SIZE = (0.16, 0.08, 0.004)
MAT_XY = (0.18, -0.24)
BRACKET_TOP = TOP + BRACKET_SIZE[2]
SEAT_Z = BRACKET_TOP + SHELF_SIZE[2] / 2  # the shelf's centre height when it lies on both brackets

STEEL = (0.45, 0.47, 0.50)
LEFT = Box("left_bracket", pos=(X, BRACKET_Y, TOP + BRACKET_SIZE[2] / 2), size=BRACKET_SIZE, color=STEEL)
RIGHT = Box("right_bracket", pos=(X, -BRACKET_Y, TOP + BRACKET_SIZE[2] / 2), size=BRACKET_SIZE, color=STEEL)
MAT = Box("mat", pos=(*MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.15, 0.30, 0.75))
PROP = Box("prop", pos=(*PROP_XY, TOP + PROP_SIZE[2] / 2), size=PROP_SIZE, mass=0.15, color=(0.80, 0.12, 0.10))
# Spawned level just above the prop; its left end drops 8 mm onto the left bracket while the scene settles.
SHELF = Box(
    "shelf", pos=(X, 0.0, TOP + PROP_SIZE[2] + SHELF_SIZE[2] / 2 + 0.001), size=SHELF_SIZE, mass=0.22,
    color=(0.85, 0.70, 0.45),
)
# A full tin of soup, a SimReady prop. This prop's origin is its centre, so pos is the centre: 2 mm clear of the shelf.
TIN = SimReady(
    "tin", pos=(X, TIN_Y, TOP + PROP_SIZE[2] + SHELF_SIZE[2] + TIN_H / 2 + 0.002), size=(TIN_D, TIN_D, TIN_H),
    mass=0.35, convex=True, package="food_can_tomatoSoup_a01", usd="sm_food_can_tomatoSoup_a01_01.usda",
)
# Distractors behind the shelf, which scoring ignores. pos is each one's origin, on the table top.
MUG = SimReady(
    "mug", pos=(0.48, 0.24, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=np.pi,
    package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
)
TAPE = SimReady(
    "tape_roll", pos=(0.49, -0.22, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
    package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
)
SCREWDRIVER = SimReady(
    "screwdriver", pos=(0.47, 0.0, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
    package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
)
OBJECTS = [LEFT, RIGHT, MAT, PROP, SHELF, TIN, MUG, TAPE, SCREWDRIVER]

SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
END_START = {"left": np.array([X, SHELF_SIZE[1] / 2]), "right": np.array([X, -SHELF_SIZE[1] / 2])}
SUPPORT_TOL = 0.003  # "lying on": a centre or face within 3 mm of the height it has when resting on its support
CLEAR = 0.02  # "let go": no point on either gripper's fingers within 2 cm of the shelf, the tin or the prop
_harm = False  # the tin fell, the shelf dropped, or the prop was knocked over under the shelf, at any step
_tips = None  # points on both grippers' fingers at the latest step


@on_reset
def _new_trial(environment=None):
    """The record of harm is cleared only by a reset of the scene, never by anything that happens in a trial."""
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


def _tilt_deg(obj):
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _ends(shelf):
    """The centres of the shelf's left and right end faces."""
    axis = _rotate(shelf.quat_xyzw, np.array([0.0, 1.0, 0.0]))
    if axis[1] < 0:
        axis = -axis
    half = axis * SHELF_SIZE[1] / 2
    return {"left": shelf.pos + half, "right": shelf.pos - half}


def _seated(objects):
    """The shelf lies on both brackets: each end reaches at least 1 cm onto its bracket and is within 3 cm of the
    bracket's centre line front to back, the shelf's underside is within 3 mm of the brackets' tops, and one end
    is no more than 3 mm higher than the other."""
    shelf = objects["shelf"]
    ends = _ends(shelf)
    left, right = objects["left_bracket"], objects["right_bracket"]
    reach_left = min(ends["left"][1], left.high[1]) - max(ends["right"][1], left.low[1])
    reach_right = min(ends["left"][1], right.high[1]) - max(ends["right"][1], right.low[1])
    in_line = abs(ends["left"][0] - X) < 0.03 and abs(ends["right"][0] - X) < 0.03
    level = abs(ends["left"][2] - ends["right"][2]) <= SUPPORT_TOL
    down = abs(shelf.pos[2] - SEAT_Z) <= SUPPORT_TOL
    return bool(reach_left >= 0.01 and reach_right >= 0.01 and in_line and level and down)


def _shelf_shift(objects):
    ends = _ends(objects["shelf"])
    return max(float(np.linalg.norm(ends[k][:2] - END_START[k])) for k in ends)


def _tin_ok(objects):
    """The tin stands upright on the shelf."""
    tin, shelf = objects["tin"], objects["shelf"]
    on_shelf = abs(tin.pos[2] - (shelf.pos[2] + SHELF_SIZE[2] / 2 + TIN_H / 2)) <= SUPPORT_TOL
    return bool(_tilt_deg(tin) < 10 and on_shelf)


def _prop_under_shelf(objects):
    """Seen from above, any part of the prop overlaps the shelf."""
    prop, shelf = objects["prop"], objects["shelf"]
    return bool(all(prop.low[i] < shelf.high[i] and prop.high[i] > shelf.low[i] for i in (0, 1)))


def _prop_on_mat(objects):
    prop, mat = objects["prop"], objects["mat"]
    over = all(mat.low[i] <= prop.pos[i] <= mat.high[i] for i in (0, 1))
    return bool(over and prop.low[2] <= mat.high[2] + SUPPORT_TOL)


def _track(objects):
    """Remember, for the whole trial, whether the tin fell, the shelf came down off a bracket, or the prop was
    knocked over while still under the shelf."""
    global _harm
    prop, shelf, tin = objects["prop"], objects["shelf"], objects["tin"]
    tin_fell = _tilt_deg(tin) > 30 or tin.pos[2] < BRACKET_TOP
    shelf_fell = shelf.low[2] < BRACKET_TOP - 0.03
    prop_knocked = _tilt_deg(prop) > 30 and _prop_under_shelf(objects)
    _harm = bool(_harm or tin_fell or shelf_fell or prop_knocked)


def _box_gap(obj, size):
    """The smallest distance (m) from any fingertip point to the object's box, or 0 before the first step."""
    if _tips is None:
        return 0.0
    x, y, z, w = obj.quat_xyzw
    local = np.array([_rotate((-x, -y, -z, w), p - obj.pos) for p in _tips])
    return float(np.linalg.norm(np.maximum(np.abs(local) - np.array(size) / 2, 0.0), axis=1).min())


def _hands_off(objects):
    return bool(
        _box_gap(objects["shelf"], SHELF_SIZE) > CLEAR
        and _box_gap(objects["prop"], PROP_SIZE) > CLEAR
        and _box_gap(objects["tin"], (TIN_D, TIN_D, TIN_H)) > CLEAR
    )


def score(objects):
    """0 if the tin ever tipped past 30 degrees or left the shelf, any part of the shelf ever dropped more than
    3 cm below the brackets' tops, or the prop ever tipped past 30 degrees while any part of it was under the shelf.
    Otherwise, with everything at rest, no gripper finger within 2 cm of the shelf, the tin or the prop, the shelf
    lying on both brackets, the tin upright on it and no part of the prop under the shelf: 1.0 if the prop is on the mat and neither end of the
    shelf is more than 2 cm from where it started, else 0.5. Anything else 0."""
    _track(objects)
    if _harm:
        return 0.0
    still = all(objects[n].resting() for n in ("prop", "shelf", "tin"))
    if not (still and _hands_off(objects) and _seated(objects) and _tin_ok(objects)) or _prop_under_shelf(objects):
        return 0.0
    return 1.0 if _prop_on_mat(objects) and _shelf_shift(objects) <= 0.02 else 0.5


def success(objects):
    return score(objects) == 1.0


HOLD_Y = 0.02  # where along the shelf the left arm grips it, across its width
HOLD_Z = TOP + 0.094  # the shelf's centre height there at the start (it slopes 1.6 degrees)
LIFT = 0.03  # commanded; the loaded arm sags, so the shelf rises about half of this
PROP_GRIP_Z = TOP + PROP_SIZE[2] - 0.015
PULL = 0.10
# The arms track a commanded point about this far short of it (toward their own base) at these poses, measured
# in the simulator; the demo commands that much further so the grips close centred on the shelf and the prop.
LEFT_DX = 0.014
RIGHT_DY = 0.012


def demo():
    """The left arm grips the shelf across its width just left of centre and raises it 1.5 cm, which takes its
    weight off the prop; the right arm draws the prop out toward the arms by its front end; the left arm lowers
    the shelf onto both brackets and lets go; the right arm then carries the prop to the mat."""
    yaw = -np.pi / 2
    travel = TOP + 0.17
    lx = X + LEFT_DX
    hold = (lx, HOLD_Y, HOLD_Z)
    up = (lx, HOLD_Y, HOLD_Z + LIFT)
    seat = (lx, HOLD_Y, SEAT_Z + 0.001)
    left = [
        Waypoint(1.5, (0.25, 0.08, TOP + 0.20), FINGER_OPEN, "come round in front of the tin", yaw),
        Waypoint(1.5, (lx, HOLD_Y, travel), FINGER_OPEN, "above the shelf", yaw),
        Waypoint(1.5, hold, FINGER_OPEN, "descend", yaw),
        Waypoint(1.0, hold, FINGER_CLOSED, "grip the shelf", yaw),
        Waypoint(1.5, up, FINGER_CLOSED, "take the weight", yaw),
        Waypoint(3.0, up, FINGER_CLOSED, "hold while the prop comes out", yaw),
        Waypoint(1.5, seat, FINGER_CLOSED, "lower onto both brackets", yaw),
        Waypoint(10.0, seat, FINGER_CLOSED, "keep holding", yaw),
    ]
    px, py = PROP_XY[0] - 0.035, PROP_XY[1] + RIGHT_DY
    front = (px, py, PROP_GRIP_Z)
    out = (px - PULL, py, PROP_GRIP_Z)
    mid = (PROP_XY[0] - PULL, py, PROP_GRIP_Z)
    carry_z = PROP_GRIP_Z + 0.04
    my = MAT_XY[1] + RIGHT_DY / 2
    over_mat = (MAT_XY[0], my, carry_z)
    on_mat = (MAT_XY[0], my, PROP_GRIP_Z + MAT_SIZE[2] + 0.002)
    right = [
        Waypoint(2.0, (px, py, travel), FINGER_OPEN, "above the prop's front end"),
        Waypoint(1.5, front, FINGER_OPEN, "descend"),
        Waypoint(1.0, front, FINGER_CLOSED, "grip the prop"),
        Waypoint(2.7, front, FINGER_CLOSED, "wait for the shelf to rise"),
        Waypoint(2.5, out, FINGER_CLOSED, "draw the prop out"),
        Waypoint(0.8, out, FINGER_OPEN, "let go"),
        Waypoint(1.0, (out[0], py, carry_z + 0.03), FINGER_OPEN, "up"),
        Waypoint(1.0, (mid[0], py, carry_z + 0.03), FINGER_OPEN, "over the prop's middle"),
        Waypoint(1.0, mid, FINGER_OPEN, "descend"),
        Waypoint(1.0, mid, FINGER_CLOSED, "grip"),
        Waypoint(1.0, (mid[0], py, carry_z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, over_mat, FINGER_CLOSED, "carry to the mat"),
        Waypoint(1.0, on_mat, FINGER_CLOSED, "set down"),
        Waypoint(0.8, on_mat, FINGER_OPEN, "let go"),
        Waypoint(1.5, (MAT_XY[0], my, travel), FINGER_OPEN, "retreat"),
        Waypoint(2.0, (MAT_XY[0], my, travel), FINGER_OPEN, "wait"),
    ]
    return {"left": left, "right": right}
