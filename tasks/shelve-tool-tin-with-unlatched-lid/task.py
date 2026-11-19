"""On the station table, across the midline between the arms: a red steel tool tin (22 cm long left to right,
6 cm front to back, 7 cm tall, full of drill bits, 1.2 kg) with a grey lift-off lid (22 x 5.4 cm, 2.5 cm deep) on top. The
lid has no hinge or catch: it only rests on the tin, pushed 1.5 cm to the right so the tin's left end shows, and
it comes away if it is lifted. Directly beyond the tin is a fixed wooden shelf, 3 cm high. Behind the
shelf lie three distractors (a mug, a roll of tape, a screwdriver), which scoring ignores.
"""

import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from simready import SimReady  # noqa: E402
import mechkit as m  # noqa: E402  hooks yam_arena: on_step/on_reset and fingertips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
TIN_XY = (0.18, 0.0)
TIN_SIZE = (0.06, 0.22, 0.07)  # x (front to back), y (left to right), z
LID_SIZE = (0.054, 0.22, 0.025)
LID_SHIFT = -0.015  # the lid's centre from the tin's centre along y: pushed to the right

SHELF_XY = (0.33, 0.0)
SHELF_SIZE = (0.14, 0.30, 0.03)
SHELF_TOP = TOP + SHELF_SIZE[2]

TIN = Box("tin", pos=(*TIN_XY, TOP + TIN_SIZE[2] / 2), size=TIN_SIZE, mass=1.2, color=(0.70, 0.12, 0.10))
LID = Box(
    "lid",
    pos=(TIN_XY[0], TIN_XY[1] + LID_SHIFT, TOP + TIN_SIZE[2] + LID_SIZE[2] / 2 + 0.001),
    size=LID_SIZE,
    mass=0.08,
    color=(0.62, 0.64, 0.66),
)
SHELF = Box("shelf", pos=(*SHELF_XY, TOP + SHELF_SIZE[2] / 2), size=SHELF_SIZE, color=(0.55, 0.40, 0.25))
# Distractors behind the shelf (SimReady props; pos is each prop's bottom centre), which scoring ignores.
MUG = SimReady(
    "mug", pos=(0.48, 0.24, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=np.pi,
    package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
)
TAPE = SimReady(
    "tape_roll", pos=(0.49, -0.24, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
    package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
)
SCREWDRIVER = SimReady(
    "screwdriver", pos=(0.48, -0.044, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
    package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
)
OBJECTS = [TIN, LID, SHELF, MUG, TAPE, SCREWDRIVER]

SUCCESS_HOLD_S = 3.0  # long enough for the grippers to let go and move away before the trial ends
LID_GAP = 0.01  # the lid is off once its underside is more than 1 cm from the tin's top face
SPILL_DEG = 30.0  # the tin counts as spilt once it tilts past this
FLAT_DEG = 3.0
ON_SHELF = 0.003  # the tin's lowest point within 3 mm of the shelf top
HANDS_CLEAR = 0.02  # every fingertip point more than 2 cm from the tin and the lid


def _rot(st):
    x, y, z, w = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _tilt_deg(st):
    return float(np.degrees(np.arccos(np.clip(_rot(st)[2, 2], -1.0, 1.0))))


def _lid_local(objects):
    """The lid's centre in the tin's own frame (m): x, y across the tin's top, z above the tin's centre."""
    tin, lid = objects["tin"], objects["lid"]
    return _rot(tin).T @ (np.asarray(lid.pos, float) - np.asarray(tin.pos, float))


def _lid_on(objects):
    """The lid's centre is over the tin's top face and its underside is within 1 cm of that face."""
    lx, ly, lz = _lid_local(objects)
    gap = lz - (TIN_SIZE[2] / 2 + LID_SIZE[2] / 2)
    return bool(abs(lx) <= TIN_SIZE[0] / 2 and abs(ly) <= TIN_SIZE[1] / 2 and abs(gap) <= LID_GAP)


def _gap_to_box(points, st, size):
    """Smallest distance (m) from any of the points to a box of this size at the object's pose (0 if inside)."""
    local = (np.asarray(points, float) - np.asarray(st.pos, float)) @ _rot(st)
    out = np.maximum(np.abs(local) - np.asarray(size) / 2, 0.0)
    return float(np.min(np.linalg.norm(out, axis=1)))


class Trial:
    """Per-trial record, updated after every control step. A failure is latched here and cleared only by
    reset(), which mechkit calls after the simulator resets the scene; never because the objects look right."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.failed = False  # the lid has been off the tin, or the tin has tilted past 30 degrees
        self.hands_clear = False

    def step(self, env):
        from yam_arena.scene import object_states  # simulator-only import; offline loading never reaches it

        objects = object_states(env, ("tin", "lid"))
        self.failed = bool(self.failed or not _lid_on(objects) or _tilt_deg(objects["tin"]) > SPILL_DEG)
        tips = m.fingertips(env)
        gap = min(_gap_to_box(tips, objects["tin"], TIN_SIZE), _gap_to_box(tips, objects["lid"], LID_SIZE))
        self.hands_clear = bool(gap > HANDS_CLEAR)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    """0 if the lid has been off the tin or the tin has tilted more than 30 degrees at any step. Otherwise, with
    the tin and lid at rest and no fingertip within 2 cm of either: 1.0 for the tin level and flat on the shelf top
    with its whole base inside the shelf's edges; 0.5 if its lowest point is on the shelf top and its centre is
    over the shelf, but part of its base overhangs an edge or it is not level; 0 in every other case."""
    if TRIAL.failed or not TRIAL.hands_clear:
        return 0.0
    tin, lid = objects["tin"], objects["lid"]
    if not (tin.resting() and lid.resting() and _lid_on(objects)):
        return 0.0
    hx, hy = SHELF_SIZE[0] / 2, SHELF_SIZE[1] / 2
    centre_over = bool(abs(tin.pos[0] - SHELF_XY[0]) <= hx and abs(tin.pos[1] - SHELF_XY[1]) <= hy)
    on_shelf = bool(abs(tin.low[2] - SHELF_TOP) <= ON_SHELF)
    if not (centre_over and on_shelf):
        return 0.0
    whole = bool(
        tin.low[0] >= SHELF_XY[0] - hx
        and tin.high[0] <= SHELF_XY[0] + hx
        and tin.low[1] >= SHELF_XY[1] - hy
        and tin.high[1] <= SHELF_XY[1] + hy
    )
    return 1.0 if whole and _tilt_deg(tin) <= FLAT_DEG else 0.5


def success(objects):
    return bool(score(objects) == 1.0)


# ---- scripted demo ---------------------------------------------------------------------------------------
# Each arm grips the tin itself across its 6 cm depth, below the lid, 8.5 cm either side of its middle. Together
# they lift it with the lid riding on top, carry it over the shelf, set it down and let go.
GRIP_Y = {"left": 0.085, "right": -0.085}
GRIP_Z = TOP + 0.040  # pads on the tin's side walls, 1.7 cm below the lid
# Fingers close along x, across the tin; each arm turns its wrist the way that keeps joint6 inside its limits.
ACROSS = {"left": -np.pi / 2, "right": np.pi / 2}
TRAVEL_Z = TOP + 0.16
CLEAR_Z = TOP + 0.18  # after letting go the fingertips end about 4 cm above the lid (higher is out of reach over the shelf)
LIFT = 0.06


def _arm(arm):
    y, yaw = GRIP_Y[arm], ACROSS[arm]
    x0, x1 = TIN_XY[0], SHELF_XY[0]
    z_place = GRIP_Z + SHELF_SIZE[2] + 0.003
    return [
        Waypoint(2.0, (x0, y, TRAVEL_Z), FINGER_OPEN, "above the tin", yaw),
        Waypoint(2.0, (x0, y, GRIP_Z), FINGER_OPEN, "down past the lid", yaw),
        Waypoint(1.0, (x0, y, GRIP_Z), FINGER_CLOSED, "grip the tin's walls", yaw),
        Waypoint(2.0, (x0, y, GRIP_Z + LIFT), FINGER_CLOSED, "lift", yaw),
        Waypoint(3.0, (x1, y, GRIP_Z + LIFT), FINGER_CLOSED, "carry over the shelf", yaw),
        Waypoint(2.0, (x1, y, z_place), FINGER_CLOSED, "lower onto the shelf", yaw),
        Waypoint(1.0, (x1, y, z_place), FINGER_OPEN, "let go", yaw),
        Waypoint(1.5, (x1, y, CLEAR_Z), FINGER_OPEN, "retreat", yaw),
    ]


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
