"""On the station table: a pine shelf plank (36 x 6 x 2 cm) runs left to right, 8 cm above the table. Its left
end lies on a fixed steel stand; its right end is held up by a loose timber prop (14 x 4 x 8 cm) standing under
it. A 170 g tin of tuna stands on the plank. The prop has been knocked towards the robot: only 1.5 cm of it is
still under the plank's 6 cm width, the other 12.5 cm sticks out in front. The prop is loaded, so it does not
slide under the plank when pushed: the plank goes with it.
"""

import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

X = 0.30  # the plank's centre line
H = 0.08  # height of the stand and of the prop
PLANK_SIZE = (0.06, 0.36, 0.02)
PLANK_Y = 0.06
STAND_SIZE = (0.10, 0.05, H)
STAND_Y = 0.215
PROP_SIZE = (0.14, 0.04, H)
PROP_Y = -0.095
START_OVERLAP = 0.015  # of the prop under the plank's width at the start
PROP_X = X - PLANK_SIZE[0] / 2 + START_OVERLAP - PROP_SIZE[0] / 2
TIN_D, TIN_H, TIN_Y = 0.086, 0.034, 0.14
SEAT_Z = TOP + H + PLANK_SIZE[2] / 2  # the plank's centre height when it lies on the stand and the prop
TIN_STAND = PLANK_SIZE[2] / 2  # the tin's base above the plank's centre when it stands on the plank

STAND = Box("stand", pos=(X, STAND_Y, TOP + H / 2), size=STAND_SIZE, color=(0.45, 0.47, 0.50))
PROP = Box("prop", pos=(PROP_X, PROP_Y, TOP + H / 2), size=PROP_SIZE, mass=0.20, color=(0.55, 0.40, 0.25))
PLANK = Box("plank", pos=(X, PLANK_Y, SEAT_Z + 0.001), size=PLANK_SIZE, mass=0.25, color=(0.85, 0.70, 0.45))
# A tin of tuna, a SimReady prop, wider than the plank: it overhangs 1.3 cm front and back. pos is the centre of
# its base, 2 mm clear of the plank.
TIN = SimReady(
    "tin", pos=(X, TIN_Y, SEAT_Z + TIN_STAND + 0.002), size=(TIN_D, TIN_D, TIN_H),
    mass=0.17, convex=True, package="food_can_tunaFish_a01", usd="sm_food_can_tunaFish_a01_01.usda",
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
OBJECTS = [STAND, PROP, PLANK, TIN, MUG, TAPE, SCREWDRIVER]

SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
FULL_OVERLAP, PART_OVERLAP = 0.055, 0.03  # of the prop under the plank's 6 cm width
FULL_SHIFT, PART_SHIFT = 0.01, 0.03  # how far the plank's centre may end up from where it started
DROP = 0.03  # the plank's lowest point this far below its supported height counts as dropped
import os  # DEBUG
DEBUG = os.environ.get("TASK_DEBUG")  # DEBUG
_harm = False
_prev = None  # the previous step's positions, to tell a reset (a jump) from objects drifting back


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _tilt_deg(obj):
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _overlap(objects):
    """How much of the prop lies under the plank (m), measured across the plank's width and along its length."""
    prop, plank = objects["prop"], objects["plank"]
    rel = prop.pos - plank.pos
    out = []
    for axis, plank_len, prop_len in ((0, PLANK_SIZE[0], PROP_SIZE[0]), (1, PLANK_SIZE[1], PROP_SIZE[1])):
        unit = np.zeros(3)
        unit[axis] = 1.0
        c = float(np.dot(rel, _rotate(plank.quat_xyzw, unit)))
        out.append(max(0.0, min(plank_len / 2, c + prop_len / 2) - max(-plank_len / 2, c - prop_len / 2)))
    return out


def _plank_shift(objects):
    return float(np.linalg.norm(objects["plank"].pos[:2] - np.array([X, PLANK_Y])))


def _tin_on_plank(objects):
    """The tin's centre is over the plank and its base no more than 2 cm below where it is when it stands on it."""
    tin, plank = objects["tin"], objects["plank"]
    inside = np.all(tin.pos[:2] > plank.low[:2]) and np.all(tin.pos[:2] < plank.high[:2])
    return bool(inside and tin.pos[2] > plank.pos[2] + TIN_STAND - 0.02)


def _track(objects):
    """Remember, for the whole trial, whether the plank dropped or the tin tipped over or came off the plank. The
    memory is cleared only by a reset: a step in which something jumps (further than it can move in 1/20 s of
    normal handling) and everything is back in the exact starting layout. Drifting back does not clear it."""
    global _harm, _prev
    prop, plank, tin = objects["prop"], objects["plank"], objects["tin"]
    now = (np.array(prop.pos), np.array(plank.pos), np.array(tin.pos), float(plank.low[2]), _tilt_deg(tin))
    jumped = _prev is not None and (
        max(float(np.linalg.norm(now[i] - _prev[i])) for i in range(3)) > 0.05
        or abs(now[3] - _prev[3]) > 0.025
        or abs(now[4] - _prev[4]) > 25
    )
    _prev = now
    at_start = (
        np.linalg.norm(prop.pos[:2] - np.array([PROP_X, PROP_Y])) < 0.005
        and _plank_shift(objects) < 0.005
        and abs(plank.pos[2] - SEAT_Z) < 0.003
        and np.linalg.norm(tin.pos[:2] - np.array([X, TIN_Y])) < 0.005
        and _tilt_deg(tin) < 3
    )
    if jumped and at_start:
        _harm = False
    dropped = plank.low[2] < TOP + H - DROP
    spilled = _tilt_deg(tin) > 30 or not _tin_on_plank(objects)
    _harm = bool(_harm or dropped or spilled)


def score(objects):
    """0 if the plank ever dropped 3 cm or the tin ever tipped past 30 degrees or left the plank. Otherwise the
    plank must lie at rest and level on the stand and the upright prop with the tin upright on it; then 1.0 with
    at least 5.5 cm of the plank's 6 cm width over the prop and the plank within 1 cm of where it started, 0.5
    with at least 3 cm over the prop and the plank within 3 cm, else 0."""
    _track(objects)
    if _harm:
        return 0.0
    prop, plank, tin = objects["prop"], objects["plank"], objects["tin"]
    under, along = _overlap(objects)
    supported = bool(
        _tilt_deg(plank) < 2
        and abs(plank.pos[2] - SEAT_Z) < 0.005
        and plank.resting()
        and _tilt_deg(prop) < 3
        and abs(prop.pos[2] - (TOP + H / 2)) < 0.005
        and prop.resting()
        and along >= 0.035
        and _tilt_deg(tin) < 5
        and tin.resting()
    )
    if DEBUG:
        open(DEBUG, "a").write(f"{under:.4f} {along:.4f} {_plank_shift(objects):.4f} {plank.pos} {_tilt_deg(plank):.2f} {prop.pos} {tin.pos} {_tilt_deg(tin):.2f} {supported} {_harm}\n")
    if not supported:
        return 0.0
    shift = _plank_shift(objects)
    if under >= FULL_OVERLAP and shift <= FULL_SHIFT:
        return 1.0
    if under >= PART_OVERLAP and shift <= PART_SHIFT:
        return 0.5
    return 0.0


def success(objects):
    return score(objects) == 1.0


HOLD_Y = 0.03  # where the left gripper takes the plank, between the tin and the prop
HOLD_YAW = -np.pi / 2  # fingers close along x, across the plank's width
HOLD_LIFT = 0.02
# Measured in the simulator: as the fingers close they pull the gripped part about 1.5 cm towards one finger,
# so each grip is aimed that much to the other side to leave the plank and the prop where they are.
HOLD_DX, PROP_DY = 0.016, 0.012
PROP_GRIP_X = PROP_X - PROP_SIZE[0] / 2 + 0.015  # near the prop's front end, clear of the plank
PROP_TRAVEL = 0.055  # leaves the prop under the whole width of the plank and the gripper clear of it


def demo():
    """The left arm grips the plank between the tin and the prop and raises that end about 2 cm, taking the load. The right
    arm grips the prop's front end, moves it 5.5 cm back under the plank and lets go. The left arm
    lowers the plank onto the prop and lets go."""
    travel = TOP + 0.20
    low_travel = TOP + 0.13  # the left arm is near the end of its reach here and cannot go higher
    hold = (X + HOLD_DX, HOLD_Y, SEAT_Z)
    held = (X + HOLD_DX, HOLD_Y, SEAT_Z + HOLD_LIFT)
    left = [
        Waypoint(2.0, (X + HOLD_DX, HOLD_Y, low_travel), FINGER_OPEN, "above the plank", HOLD_YAW),
        Waypoint(1.5, hold, FINGER_OPEN, "descend", HOLD_YAW),
        Waypoint(1.0, hold, FINGER_CLOSED, "grip the plank", HOLD_YAW),
        Waypoint(1.5, held, FINGER_CLOSED, "take the load", HOLD_YAW),
        Waypoint(8.0, held, FINGER_CLOSED, "hold while the prop is moved", HOLD_YAW),
        Waypoint(2.0, hold, FINGER_CLOSED, "lower onto the prop", HOLD_YAW),
        Waypoint(8.0, hold, FINGER_CLOSED, "keep holding", HOLD_YAW),
    ]
    gz = TOP + 0.055
    g0 = (PROP_GRIP_X, PROP_Y + PROP_DY, gz)
    g1 = (PROP_GRIP_X + PROP_TRAVEL, PROP_Y + PROP_DY, gz)
    up = 0.003
    right = [
        Waypoint(2.0, (PROP_GRIP_X, PROP_Y + PROP_DY, travel), FINGER_OPEN, "above the prop"),
        Waypoint(4.0, (PROP_GRIP_X, PROP_Y + PROP_DY, travel), FINGER_OPEN, "wait for the plank to be held"),
        Waypoint(1.5, g0, FINGER_OPEN, "descend"),
        Waypoint(1.0, g0, FINGER_CLOSED, "grip the prop"),
        Waypoint(0.5, (g0[0], g0[1], gz + up), FINGER_CLOSED, "unweight"),
        Waypoint(2.0, (g1[0], g1[1], gz + up), FINGER_CLOSED, "move under the plank"),
        Waypoint(0.5, g1, FINGER_CLOSED, "set down"),
        Waypoint(1.0, g1, FINGER_OPEN, "let go"),
        Waypoint(1.5, (g1[0], g1[1], travel), FINGER_OPEN, "retreat"),
    ]
    return {"left": left, "right": right}
