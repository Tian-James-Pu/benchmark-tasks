"""On the station table: a steel floor channel is fixed to the table, running away from the arms, centred 23 cm
in front of them and 6 cm left of the midline. Inside it is 18.4 cm long and 8.8 cm wide; its left wall is 8 cm
high, its right wall 11 cm, its far end is closed by a 5 cm wall and its near end is open. A 5 mm high pivot
rib runs along its floor beside the left wall. A plywood display panel (18 cm long, 3 cm thick, 16 cm tall)
belongs upright along the left wall with its bottom edge on the rib; on the rib it cannot stand by itself. A
red beech chock (3.5 x 5 x 15 cm) standing in the channel beside it, between the panel and the right wall, is
what keeps it up. The chock has come out and stands on the table to the right of the channel, and the panel
has fallen over to the right: its bottom edge is still in the channel and it leans about 27 degrees, resting
on the top of the right wall.
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

X, Y0 = 0.23, 0.06  # the channel's centre
PANEL_SIZE = (0.18, 0.03, 0.16)
CHOCK_SIZE = (0.035, 0.05, 0.15)
IN_X, IN_Y = 0.184, 0.088  # the channel's inside length (front to back) and width (left to right)
WALL = 0.01
LEFT_H, RIGHT_H, END_H = 0.08, 0.11, 0.05  # wall heights
RIB_H, RIB_W = 0.005, 0.006
Y_LEFT, Y_RIGHT = Y0 + IN_Y / 2, Y0 - IN_Y / 2  # inside faces of the left and right walls
PANEL_Y = Y_LEFT - PANEL_SIZE[1] / 2  # the panel's centre when it stands against the left wall
PANEL_Z = TOP + RIB_H + PANEL_SIZE[2] / 2
MOUTH_X = X - IN_X / 2  # the channel's open end
CHOCK_Y = Y_RIGHT + CHOCK_SIZE[1] / 2 + 0.004  # the chock's centre when it is down in the channel
CHOCK_Z = TOP + CHOCK_SIZE[2] / 2
CHOCK_X = X - IN_X / 2 + CHOCK_SIZE[0] / 2 + 0.006  # where the demo puts it: just inside the open end
CHOCK_START = (0.12, -0.14, CHOCK_Z)

STEEL = (0.35, 0.37, 0.40)
OBJECTS = [
    Box("left_wall", pos=(X + WALL / 2, Y_LEFT + WALL / 2, TOP + LEFT_H / 2), size=(IN_X + WALL, WALL, LEFT_H), color=STEEL),
    Box("right_wall", pos=(X + WALL / 2, Y_RIGHT - WALL / 2, TOP + RIGHT_H / 2), size=(IN_X + WALL, WALL, RIGHT_H), color=STEEL),
    Box("end_wall", pos=(X + IN_X / 2 + WALL / 2, Y0, TOP + END_H / 2), size=(WALL, IN_Y, END_H), color=STEEL),
    Box("rib", pos=(X, PANEL_Y + 0.008, TOP + RIB_H / 2), size=(IN_X, RIB_W, RIB_H), color=STEEL),
    # Spawned upright on the rib; its centre is right of the rib, so it falls to the right while the scene
    # settles and comes to rest leaning on the right wall.
    Box("panel", pos=(X, PANEL_Y - 0.004, PANEL_Z + 0.002), size=PANEL_SIZE, mass=0.45, color=(0.85, 0.70, 0.45)),
    Box("chock", pos=CHOCK_START, size=CHOCK_SIZE, mass=0.18, color=(0.75, 0.25, 0.15)),
    # Distractors beyond the channel (SimReady props), which scoring ignores. pos is each one's origin, on the table top.
    SimReady(
        "mug", pos=(0.48, 0.26, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=np.pi,
        package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
    ),
    SimReady(
        "tape_roll", pos=(0.48, -0.22, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
        package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
    ),
    SimReady(
        "screwdriver", pos=(0.47, -0.03, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
        package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
    ),
]

SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
UPRIGHT_DEG = 6.0  # the panel counts as upright within this of vertical
FELL_DEG = 20.0  # once it has been upright, leaning past this is a fall
OVER_DEG = 60.0  # leaning past this at any time is the panel going over
SEAT_TOL = 0.01  # how far the panel's bottom edge and the chock may sit from their places
FOOT_Z_TOL = 0.003  # the bottom edge's centre is on the rib within this (on the channel floor it is 0.5 cm low)
CLEAR = 0.02  # "let go": no point on either gripper's fingers is closer to the panel or the chock than this
_stood = False
_harm = False
_tips = None  # points on both grippers' fingers at the latest step


@on_reset
def _new_trial(environment=None):
    """The record of a fall is cleared only by a reset of the scene, never by anything that happens in a trial."""
    global _stood, _harm, _tips
    _stood, _harm, _tips = False, False, None


@on_step
def _see_grippers(environment):
    global _tips
    _tips = fingertips(environment)


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _up(obj):
    return _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))


def _tilt_deg(obj):
    return float(np.degrees(np.arccos(np.clip(_up(obj)[2], -1.0, 1.0))))


def _foot(panel):
    """The centre of the panel's bottom face."""
    return panel.pos - _up(panel) * PANEL_SIZE[2] / 2


def _track(objects):
    """Remember, for the whole trial, whether the panel fell after being stood up, went over, or had its bottom
    edge taken out of the channel."""
    global _stood, _harm
    panel = objects["panel"]
    tilt = _tilt_deg(panel)
    foot = _foot(panel)
    if tilt <= UPRIGHT_DEG:
        _stood = True
    fell = _stood and tilt > FELL_DEG
    over = tilt > OVER_DEG
    out = bool(foot[2] > TOP + END_H or abs(foot[1] - Y0) > IN_Y / 2 or abs(foot[0] - X) > 0.02)
    _harm = bool(_harm or fell or over or out)


def _let_go(obj, size):
    """No fingertip point within CLEAR of the box's surface."""
    if _tips is None:
        return False
    x, y, z, w = obj.quat_xyzw
    local = np.array([_rotate((-x, -y, -z, w), p - obj.pos) for p in _tips])
    gap = np.linalg.norm(np.maximum(np.abs(local) - np.array(size) / 2, 0.0), axis=1)
    return bool(gap.min() > CLEAR)


def _chock_inside(chock):
    """How much of the chock's 3.5 cm thickness is inside the channel, past its open end (m)."""
    return float(np.clip(chock.pos[0] + CHOCK_SIZE[0] / 2 - MOUTH_X, 0.0, CHOCK_SIZE[0]))


def score(objects):
    """0 if the panel ever fell (leaned past 20 degrees after having been within 6 degrees of vertical), ever
    leaned past 60 degrees, or ever had the centre of its bottom edge more than 5 cm above the table, outside
    the channel's side walls or more than 2 cm along the channel from where it started. Otherwise the panel
    must be at rest within 6 degrees of vertical with the centre of its bottom edge on the rib (within 0.3 cm in
    height) and within 1 cm of its place, the chock at rest, upright within 6 degrees, standing on the channel
    floor between the panel and the right wall, and both grippers more than 2 cm from the panel and the chock:
    1.0 if all 3.5 cm of the chock is inside the channel, 0.5 if 1 cm to 3.5 cm of it is, 0 for anything else."""
    _track(objects)
    if _harm:
        return 0.0
    panel, chock = objects["panel"], objects["chock"]
    foot = _foot(panel)
    panel_ok = (
        _tilt_deg(panel) <= UPRIGHT_DEG
        and abs(foot[0] - X) <= SEAT_TOL
        and abs(foot[1] - PANEL_Y) <= SEAT_TOL
        and abs(foot[2] - (TOP + RIB_H)) <= FOOT_Z_TOL
        and panel.resting()
    )
    chock_ok = (
        _tilt_deg(chock) <= UPRIGHT_DEG
        and abs(chock.pos[1] - CHOCK_Y) <= SEAT_TOL
        and abs(chock.pos[2] - CHOCK_Z) <= 0.005
        and chock.resting()
    )
    if not (panel_ok and chock_ok and _let_go(panel, PANEL_SIZE) and _let_go(chock, CHOCK_SIZE)):
        return 0.0
    inside = _chock_inside(chock)
    if inside >= CHOCK_SIZE[0] - 0.001:
        return 1.0
    return 0.5 if inside >= 0.01 else 0.0


def success(objects):
    return score(objects) == 1.0


# The leaning panel at the start (measured in the simulator): its centre and the direction of its tall axis.
START_POS = np.array([X, Y0 - 0.0109, TOP + 0.0781])
START_UP = np.array([0.0, -0.4522, 0.8919])
FORK_X = X + 0.06  # the left gripper works over the far half of the panel, the right gripper at the near end
FORK_DOWN = 0.02  # the fork's TCP is this far below the panel's top edge, along the panel
ACROSS = np.pi / 2  # the right gripper closes along x, across the chock's 3.5 cm
CHOCK_GRIP_Z = CHOCK_SIZE[2] / 2 - 0.012
# The right arm settles a few millimetres short of this target with the chock in hand (measured in the simulator).
CHOCK_LEAD = (0.002, -0.002)


def _fork(gap):
    """Finger target for the pads this far apart (m)."""
    return FINGER_OPEN * gap / 0.094


def demo():
    """The left gripper, fingers a few centimetres apart, straddles the leaning panel's top edge like a fork,
    swings the panel upright about its bottom edge and holds it against the left wall. The right arm carries
    the chock to the channel's open end and slides it in beside the panel, then lets go. Only then does the
    left gripper open and leave."""
    reach = PANEL_SIZE[2] - FORK_DOWN  # from the panel's bottom edge to the fork, along the panel
    foot = START_POS - START_UP * PANEL_SIZE[2] / 2
    lean = np.arctan2(-START_UP[1], START_UP[2])

    def on_arc(a):
        return (FORK_X, foot[1] - reach * np.sin(a), foot[2] + reach * np.cos(a))

    start = on_arc(lean)
    held = (FORK_X, PANEL_Y + 0.006, PANEL_Z - PANEL_SIZE[2] / 2 + reach)
    left = [
        Waypoint(2.0, (FORK_X, start[1] + 0.01, TOP + 0.168), FINGER_OPEN, "above the panel's top edge"),
        Waypoint(1.5, start, _fork(0.075), "fork over the edge"),
    ]
    for k, gap in ((1, 0.065), (2, 0.055), (3, 0.045)):
        left.append(Waypoint(0.8, on_arc(lean * (1 - k / 4)), _fork(gap), "swing upright"))
    left += [
        Waypoint(1.0, held, _fork(0.038), "against the left wall"),
        Waypoint(5.7, held, _fork(0.038), "hold while the chock goes in"),
        Waypoint(1.0, held, FINGER_OPEN, "let go"),
        Waypoint(1.5, (FORK_X + 0.08, PANEL_Y + 0.006, held[2]), FINGER_OPEN, "retreat past the panel's far end"),
    ]
    cx, cy, cz = CHOCK_START
    grip_z = cz + CHOCK_GRIP_Z
    carry_z = grip_z + 0.006  # the chock's bottom 6 mm above the table
    tx, ty = CHOCK_X + CHOCK_LEAD[0], CHOCK_Y + CHOCK_LEAD[1]
    mouth = (X - IN_X / 2 - CHOCK_SIZE[0] / 2 - 0.02, ty, carry_z)  # in line with the slot, just outside the open end
    seat = (tx, ty, carry_z)
    right = [
        Waypoint(2.0, (cx, cy, grip_z + 0.02), FINGER_OPEN, "above the chock", ACROSS),
        Waypoint(1.0, (cx, cy, grip_z), FINGER_OPEN, "descend", ACROSS),
        Waypoint(1.0, (cx, cy, grip_z), FINGER_CLOSED, "grip", ACROSS),
        Waypoint(1.0, (cx, cy, carry_z), FINGER_CLOSED, "lift clear of the table", ACROSS),
        Waypoint(3.0, mouth, FINGER_CLOSED, "carry to the channel's open end", ACROSS),
        Waypoint(1.0, mouth, FINGER_CLOSED, "steady", ACROSS),
        Waypoint(2.5, seat, FINGER_CLOSED, "slide in beside the panel", ACROSS),
        Waypoint(1.0, seat, _fork(0.06), "let go", ACROSS),
        Waypoint(1.0, (tx, ty, grip_z + 0.03), _fork(0.06), "up off the chock", ACROSS),
        Waypoint(1.0, (cx, cy + 0.04, grip_z + 0.03), FINGER_OPEN, "retreat", ACROSS),
    ]
    return {"left": left, "right": right}
