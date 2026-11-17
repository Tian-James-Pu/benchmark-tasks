"""On the station table: a storage rack for sheet material, made of a fixed back fence, a narrow raised strip along
its foot and, 7 cm in front, one short stop block for each pane. Two clear acrylic panes (16 x 7 cm, 1.6 cm thick)
stand on edge on the strip, one on the left and one on the right. The strip carries only the rear third of a pane's
thickness, so a pane with nothing in front of it tips forward off the strip and falls flat. A chock belongs between
a pane and its stop block, and there is room for only one chock there. The left pane has no chock: it is held by
the one loose oak prop block, standing beside the chock place. The right pane is held by a split chock painted red,
which has to come out before a new one fits. Two new beech chocks wait at the front of the table. Behind the rack,
out of the way: a ceramic mug, a roll of clear tape and a screwdriver (NVIDIA SimReady props, CC-BY 4.0, loaded by
simready.py), which scoring ignores.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

FENCE_FACE = 0.375  # x of the fence's front face
STRIP = (0.007, 0.004)  # depth in front of the fence, height
PANE_SIZE = (0.016, 0.16, 0.07)
PANE_X = FENCE_FACE - 0.001 - PANE_SIZE[0] / 2
PANE_FRONT = PANE_X - PANE_SIZE[0] / 2  # x of an upright pane's front face (0.358)
PANE_Y = 0.18
STOP_FACE = 0.303  # x of the stop blocks' faces towards the panes
STOP_SIZE = (0.015, 0.07, 0.015)
CHOCK_SIZE = (0.04, 0.04, 0.09)
PROP_SIZE = (0.045, 0.05, 0.09)
CHOCK_Y = 0.225  # where a chock sits along its pane, and the centre of that pane's stop block
PROP_Y = 0.13  # where the prop block sits along a pane
NEW_X = 0.18  # the new chocks wait on this line

STEEL = (0.45, 0.47, 0.50)
BEECH = (0.87, 0.74, 0.53)
FENCE = Box("fence", pos=(FENCE_FACE + 0.01, 0.0, TOP + 0.015), size=(0.02, 0.56, 0.03), color=STEEL)
STRIP_BOX = Box(
    "strip", pos=(FENCE_FACE - STRIP[0] / 2, 0.0, TOP + STRIP[1] / 2), size=(STRIP[0], 0.56, STRIP[1]), color=STEEL
)
STOP_X = STOP_FACE - STOP_SIZE[0] / 2
STOP_L = Box("left_stop", pos=(STOP_X, CHOCK_Y, TOP + STOP_SIZE[2] / 2), size=STOP_SIZE, color=STEEL)
STOP_R = Box("right_stop", pos=(STOP_X, -CHOCK_Y, TOP + STOP_SIZE[2] / 2), size=STOP_SIZE, color=STEEL)
PANE_Z = TOP + STRIP[1] + PANE_SIZE[2] / 2 + 0.001
ACRYLIC = (0.62, 0.82, 0.86)
PANE_L = Box("left_pane", pos=(PANE_X, PANE_Y, PANE_Z), size=PANE_SIZE, mass=0.20, color=ACRYLIC)
PANE_R = Box("right_pane", pos=(PANE_X, -PANE_Y, PANE_Z), size=PANE_SIZE, mass=0.20, color=ACRYLIC)
PROP_START = (PANE_FRONT - 0.0005 - PROP_SIZE[0] / 2, PROP_Y)
PROP = Box("prop_block", pos=(*PROP_START, TOP + PROP_SIZE[2] / 2), size=PROP_SIZE, mass=0.15, color=(0.42, 0.28, 0.16))
RED_START = (PANE_FRONT - 0.0005 - CHOCK_SIZE[0] / 2, -CHOCK_Y)
RED = Box("red_chock", pos=(*RED_START, TOP + CHOCK_SIZE[2] / 2), size=CHOCK_SIZE, mass=0.10, color=(0.78, 0.12, 0.10))
NEW_L = Box("new_chock_left", pos=(NEW_X, CHOCK_Y, TOP + CHOCK_SIZE[2] / 2), size=CHOCK_SIZE, mass=0.10, color=BEECH)
NEW_R = Box("new_chock_right", pos=(NEW_X, -CHOCK_Y, TOP + CHOCK_SIZE[2] / 2), size=CHOCK_SIZE, mass=0.10, color=BEECH)
# Distractors behind the rack. pos is each prop's bottom centre.
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
OBJECTS = [FENCE, STRIP_BOX, STOP_L, STOP_R, PANE_L, PANE_R, PROP, RED, NEW_L, NEW_R, MUG, TAPE, SCREWDRIVER]

PANES = ("left_pane", "right_pane")
STOPS = {"left_pane": "left_stop", "right_pane": "right_stop"}
LOOSE = PANES + ("prop_block", "red_chock", "new_chock_left", "new_chock_right")
NEW = ("new_chock_left", "new_chock_right")
SUCCESS_HOLD_S = 3.0
FALL_DEG = 30.0  # a pane tilted this far has fallen
LIFT = 0.02  # a pane's lowest point this far above the strip has been lifted out
UPRIGHT_DEG = 10.0
KEEP_CLEAR = 0.03  # the prop block and the red chock must end at least this far from a pane that counts
_harm = False


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _tilt_deg(obj):
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _gap(a, b):
    """Horizontal distance between two objects' bounding boxes (m); 0 when they overlap in plan."""
    d = np.maximum(0.0, np.maximum(a.low[:2] - b.high[:2], b.low[:2] - a.high[:2]))
    return float(np.linalg.norm(d))


def _near(obj, xy, tol=0.01):
    return bool(np.linalg.norm(obj.pos[:2] - np.asarray(xy)) < tol)


def _track(objects):
    """Remember, for the whole trial, whether a pane fell over or was lifted off the rack."""
    global _harm
    at_start = (
        _near(objects["prop_block"], PROP_START)
        and _near(objects["red_chock"], RED_START)
        and all(_near(objects[n], (NEW_X, objects[n].pos[1])) for n in NEW)
        and all(_tilt_deg(objects[p]) < 5 for p in PANES)
    )
    if at_start:
        _harm = False
    fell = any(_tilt_deg(objects[p]) > FALL_DEG for p in PANES)
    lifted = any(objects[p].low[2] > TOP + STRIP[1] + LIFT for p in PANES)
    _harm = bool(_harm or fell or lifted)


def _chocked(objects, pane_name):
    """The pane stands at rest on the strip within 10 degrees of upright, a new chock stands on the table between it
    and its stop block, inside the length of both, and neither the prop block nor the red chock is within 3 cm of
    it."""
    pane = objects[pane_name]
    stop = objects[STOPS[pane_name]]
    if _tilt_deg(pane) > UPRIGHT_DEG or not pane.resting():
        return False
    if pane.high[0] < FENCE_FACE - STRIP[0]:  # its rear edge has come forward off the strip
        return False
    if any(_gap(pane, objects[n]) < KEEP_CLEAR for n in ("prop_block", "red_chock")):
        return False
    for name in NEW:
        c = objects[name]
        seated = (
            _tilt_deg(c) < UPRIGHT_DEG
            and c.resting()
            and c.low[2] < TOP + 0.005
            and c.low[0] >= STOP_FACE
            and c.high[0] <= pane.pos[0]
            and c.low[1] >= max(stop.low[1], pane.low[1])
            and c.high[1] <= min(stop.high[1], pane.high[1])
        )
        if seated:
            return True
    return False


def score(objects):
    """0 if either pane ever tilted more than 30 degrees or was ever raised more than 2 cm. Otherwise 0.5 for each
    pane that ends chocked: at rest on the strip within 10 degrees of upright, with a new chock standing between it
    and its stop block inside the length of both, and with the prop block and the red chock both at least 3 cm away
    from it."""
    _track(objects)
    if _harm:
        return 0.0
    return 0.5 * sum(_chocked(objects, p) for p in PANES)


def success(objects):
    return bool(score(objects) == 1.0 and all(objects[n].resting() for n in LOOSE))


TRAVEL = TOP + 0.16
GRIP_Z = TOP + CHOCK_SIZE[2] - 0.02  # chocks and the prop block are the same height
SEAT_X = PANE_FRONT - 0.0045 - CHOCK_SIZE[0] / 2  # a new chock is set down 4.5 mm in front of the pane
PROP_RIGHT_X = PANE_FRONT - 0.002 - PROP_SIZE[0] / 2
HANDOVER = (0.22, 0.0)  # the prop block changes arms on the midline
SCRAP = (0.18, -0.10)  # where the red chock is put down


def _move(pick, place):
    return pick_place_waypoints((*pick, GRIP_Z), (*place, GRIP_Z + 0.002), travel_z=TRAVEL)


def demo():
    """Left arm: fit a new chock to the left pane, then take the prop block to the midline. Right arm: stand the
    prop block against the right pane, take the red chock out, fit the new chock, take the prop block away."""
    left = _move((NEW_X, CHOCK_Y), (SEAT_X, CHOCK_Y)) + _move(PROP_START, HANDOVER)
    left.append(Waypoint(1.5, (0.20, 0.22, TRAVEL), FINGER_OPEN, "park"))
    right = [Waypoint(2.0, (0.20, -0.12, TRAVEL), FINGER_OPEN, "park"), Waypoint(18.5, (0.20, -0.12, TRAVEL), FINGER_OPEN, "wait for the prop block")]
    right += _move(HANDOVER, (PROP_RIGHT_X, -PROP_Y))
    right += _move(RED_START, SCRAP)
    right += _move((NEW_X, -CHOCK_Y), (SEAT_X, -CHOCK_Y))
    right += _move((PROP_RIGHT_X, -PROP_Y), HANDOVER)
    return {"left": left, "right": right}
