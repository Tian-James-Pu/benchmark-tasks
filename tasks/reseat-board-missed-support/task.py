"""On the station table: two loose wooden support blocks stand 30 cm apart, left and right of the midline. A
pine board (40 x 5 x 2 cm) was meant to bridge them but was set down 6 cm too far left: its left end lies on
the left block and its right end missed the right block and rests on the table, so the board slopes. The
blocks are free-standing. An empty glass beer bottle stands on the table just behind the board's low end, where a careless
swing of the board would knock it over. Further back, out of the way: a ceramic mug, a roll of clear tape and a
screwdriver. The bottle, mug, tape and screwdriver are NVIDIA SimReady props (CC-BY 4.0), loaded by simready.py.
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

X = 0.30  # the board and both blocks lie on this line across the table
BLOCK_Y = 0.17  # each block's centre, left (+) and right (-) of the midline
BLOCK_SIZE = (0.08, 0.04, 0.08)
BOARD_SIZE = (0.05, 0.40, 0.02)
SHIFT = 0.06  # how far left of its seat the board was set down
BLOCK_TOP = TOP + BLOCK_SIZE[2]
SEAT_Z = BLOCK_TOP + BOARD_SIZE[2] / 2  # the board's centre height when it bridges the blocks

WOOD = (0.55, 0.40, 0.25)
LEFT = Box("left_block", pos=(X, BLOCK_Y, TOP + BLOCK_SIZE[2] / 2), size=BLOCK_SIZE, mass=0.20, color=WOOD)
RIGHT = Box("right_block", pos=(X, -BLOCK_Y, TOP + BLOCK_SIZE[2] / 2), size=BLOCK_SIZE, mass=0.20, color=WOOD)
# Spawned level just above the left block; its right end has nothing under it, so it drops to the table while
# the scene settles.
BOARD = Box("board", pos=(X, SHIFT, SEAT_Z + 0.002), size=BOARD_SIZE, mass=0.25, color=(0.85, 0.70, 0.45))
# The hazard: an empty beer bottle 22.2 cm tall, 4.5 cm behind the board's back edge, beside its low end. pos is
# each prop's bottom centre.
BOTTLE = SimReady(
    "bottle", pos=(0.40, -0.12, TOP), size=(0.06, 0.06, 0.222), mass=0.20,
    package="food_beverage_beerBottle_a01", usd="sm_food_beverage_beerBottle_a01_01.usd",
)
# Distractors on the far side of the work line, which scoring ignores.
MUG = SimReady(
    "mug", pos=(0.46, 0.22, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=np.pi,
    package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
)
TAPE = SimReady(
    "tape_roll", pos=(0.48, -0.25, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
    package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
)
SCREWDRIVER = SimReady(
    "screwdriver", pos=(0.44, -0.01, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
    package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
)
OBJECTS = [LEFT, RIGHT, BOARD, BOTTLE, MUG, TAPE, SCREWDRIVER]

BLOCK_START = {b.name: np.array(b.pos[:2]) for b in (LEFT, RIGHT)}
BOTTLE_START = np.array(BOTTLE.pos[:2])
SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
MIN_OVERLAP = 0.03  # of each board end over its block, along the board
_harm = False
_prev = None  # every object's position at the previous call
_started = False


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _tilt_deg(obj):
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _block_moved(objects, name):
    return float(np.linalg.norm(objects[name].pos[:2] - BLOCK_START[name]))


def _overlaps(objects):
    """How far each end of the board reaches over its block (m, along y), left then right; 0 when the board is
    not lying on that block."""
    board = objects["board"]
    axis = _rotate(board.quat_xyzw, np.array([0.0, 1.0, 0.0]))
    if axis[1] < 0:
        axis = -axis
    ends = {"left_block": board.pos + axis * BOARD_SIZE[1] / 2, "right_block": board.pos - axis * BOARD_SIZE[1] / 2}
    out = []
    for name, end in ends.items():
        block = objects[name]
        on_top = abs(board.pos[2] - (block.high[2] + BOARD_SIZE[2] / 2)) < 0.01 and abs(end[0] - block.pos[0]) < 0.03
        if name == "left_block":
            reach = min(end[1], block.high[1]) - block.low[1]
        else:
            reach = block.high[1] - max(end[1], block.low[1])
        out.append(max(0.0, float(reach)) if on_top else 0.0)
    return out


def _track(objects):
    """Remember, for the whole trial, whether a block toppled, the board came off both blocks or the bottle was hit.
    The record is cleared only on a reset: the first call, or a step in which the objects jump (more than 5 cm
    since the previous step, which nothing does at 20 Hz in this scene) into the starting layout. Objects that
    drift back to where they started never clear it."""
    global _harm, _prev
    board = objects["board"]
    now = {name: o.pos.copy() for name, o in objects.items()}
    jumped = _prev is not None and any(np.linalg.norm(now[n] - _prev[n]) > 0.05 for n in now)
    _prev = now
    at_start = (
        all(_block_moved(objects, n) < 0.005 for n in BLOCK_START)
        and abs(board.pos[1] - SHIFT) < 0.02
        and _tilt_deg(board) > 8
        and np.linalg.norm(objects["bottle"].pos[:2] - BOTTLE_START) < 0.005
        and _tilt_deg(objects["bottle"]) < 5
    )
    if _first() or (jumped and at_start):
        _harm = False
    toppled = any(_tilt_deg(objects[n]) > 20 for n in BLOCK_START)
    dropped = board.high[2] < TOP + 0.04  # the whole board lies on the table
    bottle = objects["bottle"]
    bottle_hit = _tilt_deg(bottle) > 20 or np.linalg.norm(bottle.pos[:2] - BOTTLE_START) > 0.02
    _harm = bool(_harm or toppled or dropped or bottle_hit)


def _first():
    global _started
    first, _started = not _started, True
    return first


def score(objects):
    """0 if a block ever tilted past 20 degrees, the board ever lay flat on the table, or the bottle ever tilted
    past 20 degrees or moved more than 2 cm. Otherwise, with the board at rest, level within 3 degrees and lying on
    both blocks (each end at least 0.5 cm over its block) and both blocks upright within 5 degrees: 1.0 with at
    least 3 cm of each end over its block and neither block moved more than 2 cm; 0.5 with less overlap or a block
    moved 2 to 5 cm. 0 otherwise."""
    _track(objects)
    if _harm:
        return 0.0
    board = objects["board"]
    left, right = _overlaps(objects)
    moved = max(_block_moved(objects, n) for n in BLOCK_START)
    upright = all(_tilt_deg(objects[n]) < 5 for n in BLOCK_START)
    bridged = bool(left > 0.005 and right > 0.005 and _tilt_deg(board) < 3 and board.resting() and upright)
    if not bridged or moved > 0.05:
        return 0.0
    return 1.0 if min(left, right) >= MIN_OVERLAP and moved <= 0.02 else 0.5


def success(objects):
    return score(objects) == 1.0


# Where the board comes to rest at the start (measured in the simulator): its centre, and its slope down to the
# right. The demo grips it across its width at two points along its length (metres left of its centre).
START_Y, START_Z, START_SLOPE = 0.0558, 0.812, np.radians(15.91)
GRIP_S = {"left": 0.06, "right": -0.11}
# Fingers close along x, across the board, so it can pitch between the pads; each arm turns its wrist the way
# that keeps joint6 inside its limits.
ACROSS = {"left": -np.pi / 2, "right": np.pi / 2}


def demo():
    """Both arms grip the board across its width, raise it level and clear of the blocks together, carry it
    right until it is centred, lower it onto both blocks and let go."""
    travel = TOP + 0.16
    lifted = SEAT_Z + 0.025
    plan = {}
    for arm, s in GRIP_S.items():
        yaw = ACROSS[arm]
        grip = (X, START_Y + s * np.cos(START_SLOPE), START_Z + s * np.sin(START_SLOPE))
        level = (X, START_Y + s, lifted)
        over = (X, s, lifted)
        seat = (X, s, SEAT_Z + 0.002)
        plan[arm] = [
            Waypoint(2.0, (grip[0], grip[1], travel), FINGER_OPEN, "above the board", yaw),
            Waypoint(1.5, grip, FINGER_OPEN, "descend", yaw),
            Waypoint(1.0, grip, FINGER_CLOSED, "grip", yaw),
            Waypoint(2.5, level, FINGER_CLOSED, "raise level", yaw),
            Waypoint(2.0, over, FINGER_CLOSED, "carry right", yaw),
            Waypoint(1.5, seat, FINGER_CLOSED, "lower onto the blocks", yaw),
            Waypoint(1.0, seat, FINGER_OPEN, "let go", yaw),
            Waypoint(1.5, (X, s, travel), FINGER_OPEN, "retreat", yaw),
        ]
    return plan
