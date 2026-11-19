"""On the station table: a ceramic wall tile (30 x 15 x 0.8 cm) was left standing on its long edge in the open,
20 cm in front of the arms, with nothing holding it: 3 degrees of tilt either way and it falls flat. Behind it
is a tile rack fixed to the table: two wooden posts 10 cm tall and, 4.5 cm in front of them, a 1 cm high toe
strip. A tile is safe in the rack when its bottom edge is on the table between the strip and the posts and its
back leans on both posts. Behind the rack, out of the way: a ceramic mug, a roll of clear tape and a
screwdriver (NVIDIA SimReady props, CC-BY 4.0, loaded by simready.py), which scoring ignores.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import mechkit as m  # noqa: E402  importing it hooks yam_arena (step/reset hooks, fingertips)
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

TILE_SIZE = (0.008, 0.30, 0.15)  # thickness, length, height as it stands
TILE_X = 0.20  # where it was left standing
POST_SIZE = (0.03, 0.03, 0.10)
POST_Y = 0.10
POST_FACE = 0.36  # x of the posts' front faces
STRIP_SIZE = (0.01, 0.26, 0.01)
SLOT = 0.045  # between the toe strip's back face and the posts' front faces

TILE = Box("tile", pos=(TILE_X, 0.0, TOP + TILE_SIZE[2] / 2), size=TILE_SIZE, mass=0.75, color=(0.92, 0.92, 0.88))
WOOD = (0.55, 0.40, 0.25)
POST_X = POST_FACE + POST_SIZE[0] / 2
LEFT_POST = Box("left_post", pos=(POST_X, POST_Y, TOP + POST_SIZE[2] / 2), size=POST_SIZE, color=WOOD)
RIGHT_POST = Box("right_post", pos=(POST_X, -POST_Y, TOP + POST_SIZE[2] / 2), size=POST_SIZE, color=WOOD)
STRIP = Box(
    "toe_strip",
    pos=(POST_FACE - SLOT - STRIP_SIZE[0] / 2, 0.0, TOP + STRIP_SIZE[2] / 2),
    size=STRIP_SIZE,
    color=(0.35, 0.25, 0.15),
)
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
OBJECTS = [TILE, LEFT_POST, RIGHT_POST, STRIP, MUG, TAPE, SCREWDRIVER]

HOLD_S = 4.0  # the 1.0 state must hold this long with both grippers clear
CONTROL_HZ = 20
HOLD_STEPS = int(HOLD_S * CONTROL_HZ)
SUCCESS_HOLD_S = 0.0  # the tracker below counts its own hold
FELL_DEG = 45.0
POST_GAP = 0.005  # back face within 5 mm of each post's top front edge
CLEAR = 0.03  # every fingertip point at least 3 cm outside the tile


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _pose(tile):
    """The tile's tilt from vertical (degrees), its upward edge direction and its face normal (toward the
    posts)."""
    up = _rotate(tile.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    normal = _rotate(tile.quat_xyzw, np.array([1.0, 0.0, 0.0]))
    if normal[0] < 0:
        normal = -normal
    if up[2] < 0:
        up = -up
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0)))), up, normal


def _post_gaps(tile, normal):
    """Distance from the tile's back face to the top front edge of each post (m)."""
    gaps = []
    for y in (POST_Y, -POST_Y):
        edge = np.array([POST_FACE, y, TOP + POST_SIZE[2]])
        gaps.append(float(normal @ (edge - tile.pos)) - TILE_SIZE[0] / 2)
    return gaps


def _hand_gap(tile, tips):
    """The smallest distance from any fingertip point to the tile's box (m)."""
    q = np.asarray(tile.quat_xyzw, dtype=float)
    axes = np.array([_rotate(q, e) for e in np.eye(3)])  # rows: the tile's x, y, z in the world
    local = (np.asarray(tips, dtype=float) - tile.pos) @ axes.T
    outside = np.maximum(np.abs(local) - np.array(TILE_SIZE) / 2, 0.0)
    return float(np.min(np.linalg.norm(outside, axis=1)))


def _assess(tile, tips):
    """This step's state: (fell, leaning, full). Leaning and full both need the tile still, its back on both posts
    and every fingertip at least 3 cm from it."""
    tilt, up, normal = _pose(tile)
    fell = tilt > FELL_DEG
    on_posts = all(abs(g) <= POST_GAP for g in _post_gaps(tile, normal))
    clear = _hand_gap(tile, tips) >= CLEAR
    leaning = bool(tile.resting() and 5.0 <= tilt <= FELL_DEG and up[0] > 0 and on_posts and clear)
    bottom = tile.pos - up * TILE_SIZE[2] / 2
    in_slot = bool(POST_FACE - SLOT <= bottom[0] <= POST_FACE and tile.low[2] < TOP + 0.003)
    full = bool(leaning and tilt <= 25.0 and in_slot and abs(float(tile.pos[1])) <= 0.03)
    return bool(fell), leaning, full


class Trial:
    """The trial's record. A fall is latched for the rest of the trial; only a reset clears it."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the recorded fall is cleared
        self.fell, self.hold, self.full, self.score = False, 0, False, 0.0

    def step(self, env):
        from yam_arena.scene import object_states  # simulator-only import

        tile = object_states(env, ("tile",))["tile"]
        fell, leaning, full = _assess(tile, m.fingertips(env))
        self.fell = self.fell or fell
        self.hold = self.hold + 1 if full and not self.fell else 0
        self.full = bool(self.full or (self.hold >= HOLD_STEPS and not self.fell))
        if self.fell:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = 0.5 if leaning else 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    """0 if the tile ever tilted more than 45 degrees from vertical (it fell). Otherwise 1.0 once the tile has
    rested for 4 s leaning 5 to 25 degrees on both posts, its bottom edge in the slot, centred within 3 cm, with
    every fingertip at least 3 cm from it; 0.5 if at the end it rests on both posts at 5 to 45 degrees with the
    fingertips clear but short of that; else 0."""
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full)


GRIP_Y = {"left": 0.09, "right": -0.09}
# Fingers close along x, across the tile's thickness; each arm turns its wrist the way that keeps joint6 inside
# its limits.
ACROSS = {"left": -np.pi / 2, "right": np.pi / 2}
GRIP_Z = TOP + TILE_SIZE[2] - 0.02
PLACE_X = POST_FACE - 0.03  # tile centre when set down in the slot, still upright
LOOSE = 0.3 * FINGER_OPEN  # pads about 28 mm apart: the tile stands free between them but cannot fall


def demo():
    """Both arms pinch the tile's top edge, lift it over the toe strip, set it down upright in the slot, loosen
    their grip, tip its top back onto the posts with the near fingers, and only then let go and back away."""
    travel = TOP + 0.16
    plan = {}
    for arm, y in GRIP_Y.items():
        yaw = ACROSS[arm]
        plan[arm] = [
            Waypoint(2.0, (TILE_X, y, travel), FINGER_OPEN, "above the tile", yaw),
            Waypoint(1.5, (TILE_X, y, GRIP_Z), FINGER_OPEN, "descend", yaw),
            Waypoint(1.0, (TILE_X, y, GRIP_Z), FINGER_CLOSED, "grip", yaw),
            Waypoint(1.0, (TILE_X, y, GRIP_Z + 0.025), FINGER_CLOSED, "lift", yaw),
            Waypoint(2.5, (PLACE_X, y, GRIP_Z + 0.025), FINGER_CLOSED, "carry to the rack", yaw),
            Waypoint(1.5, (PLACE_X, y, GRIP_Z + 0.002), FINGER_CLOSED, "set down in the slot", yaw),
            Waypoint(1.0, (PLACE_X, y, GRIP_Z + 0.002), LOOSE, "loosen", yaw),
            Waypoint(2.5, (PLACE_X + 0.042, y, GRIP_Z - 0.004), LOOSE, "tip back onto the posts", yaw),
            Waypoint(0.5, (PLACE_X + 0.034, y, GRIP_Z - 0.004), LOOSE, "ease off", yaw),
            Waypoint(0.5, (PLACE_X + 0.034, y, GRIP_Z - 0.004), FINGER_OPEN, "let go", yaw),
            Waypoint(1.5, (PLACE_X + 0.034, y, travel), FINGER_OPEN, "retreat", yaw),
            Waypoint(2.0, (TILE_X + 0.07, y, travel), FINGER_OPEN, "back off", yaw),
            Waypoint(4.5, (TILE_X + 0.07, y, travel), FINGER_OPEN, "wait clear of the tile", yaw),
        ]
    return plan
