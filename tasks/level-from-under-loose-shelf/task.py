"""On the station table: a low riser shelf on the left, a loose black melamine board lying across two loose hardwood
blocks, with a drinking glass standing on it. A yellow aluminium spirit level (30 x 5 x 2.5 cm) lies flat on the table
pointing left to right, with its left 9 cm pushed in under the board; 1.5 cm of air separates its top from the
board's underside. A green cutting mat lies at the front of the table. Lifting the level where it lies catches
its end under the board, which is only resting on the blocks, and tips the board and the glass. A roll of tape and a screwdriver lie
at the back of the table as distractors.
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

X = 0.30  # the level lies across the table on this line
BLOCK_SIZE = (0.06, 0.14, 0.04)
BOARD_SIZE = (0.30, 0.14, 0.006)
SHELF_Y = 0.19  # centre of the blocks and the board; the board's right edge is at y = 0.12
BOARD_EDGE = SHELF_Y - BOARD_SIZE[1] / 2
LEVEL_SIZE = (0.05, 0.30, 0.025)
LEVEL_Y = 0.06  # the level's centre at the start: it spans y -0.09 to 0.21, 9 cm of it under the board
MAT_SIZE = (0.10, 0.36, 0.003)
MAT_POS = (0.17, -0.07)
JAR_D, JAR_H = 0.066, 0.100
JAR_XY = (0.38, SHELF_Y)

WOOD = (0.45, 0.30, 0.18)
NEAR = Box("near_block", pos=(0.19, SHELF_Y + 0.01, TOP + BLOCK_SIZE[2] / 2), size=BLOCK_SIZE, mass=0.25, color=WOOD)
FAR = Box("far_block", pos=(0.41, SHELF_Y + 0.01, TOP + BLOCK_SIZE[2] / 2), size=BLOCK_SIZE, mass=0.25, color=WOOD)
BOARD_Z = TOP + BLOCK_SIZE[2] + BOARD_SIZE[2] / 2
BOARD = Box("shelf_board", pos=(X, SHELF_Y, BOARD_Z + 0.001), size=BOARD_SIZE, mass=0.15, color=(0.16, 0.16, 0.18))
# An empty 8 oz drinking glass, a SimReady prop, named "jar" in the scorer. pos is the centre of its bottom, 2 mm
# clear of the board.
JAR = SimReady(
    "jar", pos=(JAR_XY[0], JAR_XY[1], BOARD_Z + BOARD_SIZE[2] / 2 + 0.002), size=(JAR_D, JAR_D, JAR_H), mass=0.18,
    convex=True, package="dishware_glass_8oz_a01", usd="sm_dishware_glass_8oz_a01_01.usd",
)
LEVEL = Box("level", pos=(X, LEVEL_Y, TOP + LEVEL_SIZE[2] / 2), size=LEVEL_SIZE, mass=0.35, color=(0.95, 0.80, 0.10))
MAT = Box("mat", pos=(MAT_POS[0], MAT_POS[1], TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.10, 0.45, 0.30))
# Distractors at the back of the table, which scoring ignores. pos is each one's origin, on the table top.
TAPE = SimReady(
    "tape_roll", pos=(0.49, -0.25, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
    package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
)
SCREWDRIVER = SimReady(
    "screwdriver", pos=(0.48, -0.06, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
    package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
)
OBJECTS = [NEAR, FAR, BOARD, JAR, LEVEL, MAT, TAPE, SCREWDRIVER]

SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
BOARD_START = np.array([X, SHELF_Y])
JAR_START = np.array(JAR_XY)
SUPPORT_TOL = 0.003  # "lying on" a surface: the object's centre within 3 mm of the height it has resting on it
CLEAR = 0.02  # "let go": no point on either gripper's fingers within 2 cm of the level, the board or the glass
JAR_FOOT_TOL = 0.004  # the glass's lowest point within 4 mm of the board's top face: standing on the board
_harm = False  # the board lifted or the glass tipped or off the board, at any step of this trial
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


def _shift(objects):
    """How far the board and the glass are from where they started (m, horizontal), the larger of the two."""
    board = float(np.linalg.norm(objects["shelf_board"].pos[:2] - BOARD_START))
    jar = float(np.linalg.norm(objects["jar"].pos[:2] - JAR_START))
    return max(board, jar)


def _track(objects):
    """Remember, for the whole trial, whether the board was tilted past 5 degrees, or the glass tipped past 30
    degrees or came off the board (its lowest point under 3 cm above the table)."""
    global _harm
    jar = objects["jar"]
    tipped = _tilt_deg(jar) > 30
    lifted = _tilt_deg(objects["shelf_board"]) > 5  # an edge of the board raised about 1.2 cm off its blocks
    off = jar.low[2] < TOP + BLOCK_SIZE[2] - 0.01  # lower than anything standing on the board can be
    _harm = bool(_harm or tipped or off or lifted)


def _box_gap(obj, size):
    """The smallest distance (m) from any fingertip point to the object's box, or 0 before the first step."""
    if _tips is None:
        return 0.0
    x, y, z, w = obj.quat_xyzw
    local = np.array([_rotate((-x, -y, -z, w), p - obj.pos) for p in _tips])
    return float(np.linalg.norm(np.maximum(np.abs(local) - np.array(size) / 2, 0.0), axis=1).min())


def _aabb_gap(obj):
    """The smallest distance (m) from any fingertip point to the object's world bounding box."""
    if _tips is None:
        return 0.0
    lo, hi = np.asarray(obj.low, dtype=float), np.asarray(obj.high, dtype=float)
    return float(np.linalg.norm(np.maximum(np.maximum(lo - _tips, _tips - hi), 0.0), axis=1).min())


def _hands_off(objects):
    return bool(
        _box_gap(objects["level"], LEVEL_SIZE) > CLEAR
        and _box_gap(objects["shelf_board"], BOARD_SIZE) > CLEAR
        and _aabb_gap(objects["jar"]) > CLEAR
    )


def _level_state(objects):
    """(clear, on_mat): the level flat and at rest, lying on the table or the mat, with no part of it under the
    board; and also lying wholly on the mat."""
    level, board = objects["level"], objects["shelf_board"]
    flat = _tilt_deg(level) < 5 and level.resting()
    z = float(level.pos[2]) - LEVEL_SIZE[2] / 2
    on_table = abs(z - TOP) <= SUPPORT_TOL
    on_mat_top = abs(z - (TOP + MAT_SIZE[2])) <= SUPPORT_TOL
    clear = bool(flat and (on_table or on_mat_top) and level.high[1] < board.low[1])
    lo = np.array(MAT_POS) - np.array(MAT_SIZE[:2]) / 2
    hi = np.array(MAT_POS) + np.array(MAT_SIZE[:2]) / 2
    inside = bool(np.all(level.low[:2] >= lo) and np.all(level.high[:2] <= hi))
    return clear, bool(clear and inside and on_mat_top)


def _shelf_standing(objects):
    """The board flat and at rest on its blocks (its centre within 3 mm of its starting height) and the glass
    upright and at rest standing on the board."""
    board, jar = objects["shelf_board"], objects["jar"]
    board_on_blocks = _tilt_deg(board) < 3 and abs(float(board.pos[2]) - BOARD_Z) <= SUPPORT_TOL and board.resting()
    board_top = float(board.pos[2]) + BOARD_SIZE[2] / 2
    jar_on_board = _tilt_deg(jar) < 5 and abs(float(jar.low[2]) - board_top) <= JAR_FOOT_TOL and jar.resting()
    return bool(board_on_blocks and jar_on_board)


def score(objects):
    """0 if the board was ever tilted more than 5 degrees, or the glass ever tipped past 30 degrees or came off the
    board. Otherwise, with the board flat on its blocks, the glass upright on it, and no gripper finger within
    2 cm of the level, the board or the glass:
    1.0 for the level lying flat and at rest wholly on the mat, with the board and the glass each within 1 cm of
    where they started; 0.5 for the level lying flat and at rest with no part of it under the board, but either
    not wholly on the mat or with the board or glass shifted 1 to 3 cm; 0 for anything else."""
    _track(objects)
    if _harm:
        return 0.0
    clear, on_mat = _level_state(objects)
    shift = _shift(objects)
    if not clear or not _shelf_standing(objects) or shift > 0.03 or not _hands_off(objects):
        return 0.0
    return 1.0 if on_mat and shift <= 0.01 else 0.5


def success(objects):
    return score(objects) == 1.0


# The demo grips the level across its width at two points along its length (metres left of its centre), both on
# the part that sticks out from under the board.
GRIP_S = {"left": 0.01, "right": -0.13}
# Fingers close along x, across the level; each arm turns its wrist the way that keeps joint6 inside its limits.
ACROSS = {"left": -np.pi / 2, "right": np.pi / 2}
GRIP_Z = TOP + 0.016
PULL = 0.13  # how far right the level is carried before it is raised: 4 cm past the board's edge


def demo():
    """Both arms grip the level across its width on the exposed part, raise it 6 mm (under the board, which is 1.5 cm above it),
    carry it right until it is clear of the board, then raise it, bring it to the mat and set it down."""
    travel = TOP + 0.14
    plan = {}
    for arm, s in GRIP_S.items():
        yaw = ACROSS[arm]
        y0 = LEVEL_Y + s
        y1 = y0 - PULL
        ym = MAT_POS[1] + s
        low = GRIP_Z + 0.006
        plan[arm] = [
            Waypoint(2.0, (X, y0, travel), FINGER_OPEN, "above the level", yaw),
            Waypoint(1.5, (X, y0, GRIP_Z), FINGER_OPEN, "descend", yaw),
            Waypoint(1.0, (X, y0, GRIP_Z), FINGER_CLOSED, "grip", yaw),
            Waypoint(1.0, (X, y0, low), FINGER_CLOSED, "raise 6 mm", yaw),
            Waypoint(2.5, (X, y1, low), FINGER_CLOSED, "draw out from under the board", yaw),
            Waypoint(1.5, (X, y1, TOP + 0.10), FINGER_CLOSED, "raise", yaw),
            Waypoint(2.5, (MAT_POS[0], ym, TOP + 0.10), FINGER_CLOSED, "carry to the mat", yaw),
            Waypoint(1.5, (MAT_POS[0], ym, GRIP_Z + MAT_SIZE[2] + 0.002), FINGER_CLOSED, "lower", yaw),
            Waypoint(1.0, (MAT_POS[0], ym, GRIP_Z + MAT_SIZE[2] + 0.002), FINGER_OPEN, "let go", yaw),
            Waypoint(1.5, (MAT_POS[0], ym, travel), FINGER_OPEN, "retreat", yaw),
        ]
    return plan
