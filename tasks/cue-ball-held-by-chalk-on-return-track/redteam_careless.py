"""On the station table: a sloping ball-return track, made of two loose aluminium rails 1.2 cm apart whose left
ends rest on a fixed wooden riser and whose right ends rest on the table. The track's lower (right) end is open:
it has no end stop. A white cue ball sits on the track and is kept from rolling down only by a cube of blue cue
chalk someone set on the rails just below it. A mug of coffee stands on the table 6 cm past the open end.
Between the track and the arms are a wooden ball stand (a square frame with a 4 cm opening the ball sits in), on
the left, and a black rubber coaster for the chalk, on the right. Beyond the track are two distractors (a roll of tape and a
screwdriver) that scoring ignores.
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena (step/reset hooks, fingertips)
from simready import SimReady  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The track runs across the table along y, high on the left.
TRACK_X = 0.33
RISER_SIZE = (0.10, 0.04, 0.025)
RISER_Y = 0.12
RAIL_SIZE = (0.02, 0.32, 0.015)
RAIL_GAP = 0.012
RAIL_Y = -0.02  # centre of each rail as spawned (level); the rails pivot on the riser's inner edge as they settle
RAIL_DX = RAIL_GAP / 2 + RAIL_SIZE[0] / 2
BALL_R = 0.0285
CHALK = 0.025
MUG_Y = -0.285

# The ball stand: four fixed bars around a square opening the ball sits in. The coaster: a fixed rubber square.
STAND_X, STAND_Y = 0.175, 0.08
STAND_OPEN, STAND_BAR, STAND_H = 0.04, 0.02, 0.01
COASTER_X, COASTER_Y = 0.175, -0.08
COASTER_SIZE = (0.10, 0.10, 0.005)

ALU = (0.72, 0.74, 0.76)
WOOD = (0.55, 0.40, 0.25)
RAIL_Z = TOP + RISER_SIZE[2] + RAIL_SIZE[2] / 2 + 0.002
_ON_RAILS = float(np.sqrt(BALL_R**2 - (RAIL_GAP / 2) ** 2))  # ball centre above the rails' top face
BALL_Y0, CHALK_Y0 = 0.045, 0.0035

RISER = Box("riser", pos=(TRACK_X, RISER_Y, TOP + RISER_SIZE[2] / 2), size=RISER_SIZE, color=WOOD)
RAIL_A = Box("far_rail", pos=(TRACK_X + RAIL_DX, RAIL_Y, RAIL_Z), size=RAIL_SIZE, mass=0.26, color=ALU)
RAIL_B = Box("near_rail", pos=(TRACK_X - RAIL_DX, RAIL_Y, RAIL_Z), size=RAIL_SIZE, mass=0.26, color=ALU)
# The ball and the chalk are spawned just above the level rails and come down with them while the scene settles.
BALL = Sphere(
    "cue_ball",
    pos=(TRACK_X, BALL_Y0, RAIL_Z + RAIL_SIZE[2] / 2 + _ON_RAILS + 0.001),
    radius=BALL_R,
    mass=0.17,
    color=(0.95, 0.94, 0.88),
)
CHALK_CUBE = Box(
    "chalk",
    pos=(TRACK_X, CHALK_Y0, RAIL_Z + RAIL_SIZE[2] / 2 + CHALK / 2 + 0.001),
    size=(CHALK, CHALK, CHALK),
    mass=0.025,
    color=(0.10, 0.30, 0.75),
)
# SimReady props: the mug (its origin is the centre of the bottom of its cup; the handle points away from the
# track) and two distractors beyond the track.
MUG = SimReady(
    "mug", pos=(TRACK_X, MUG_Y, TOP), size=(0.114, 0.09, 0.08), mass=0.35, yaw=-np.pi / 2,
    package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
)
TAPE = SimReady(
    "tape_roll", pos=(0.49, -0.22, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
    package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
)
SCREWDRIVER = SimReady(
    "screwdriver", pos=(0.48, -0.04, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
    package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
)


def _bar(name, dx, dy, sx, sy):
    return Box(name, pos=(STAND_X + dx, STAND_Y + dy, TOP + STAND_H / 2), size=(sx, sy, STAND_H), color=WOOD)


_H = STAND_OPEN / 2 + STAND_BAR / 2
_OUT = STAND_OPEN + 2 * STAND_BAR
STAND = [
    _bar("stand_far_bar", _H, 0, STAND_BAR, _OUT),
    _bar("stand_near_bar", -_H, 0, STAND_BAR, _OUT),
    _bar("stand_left_bar", 0, _H, STAND_OPEN, STAND_BAR),
    _bar("stand_right_bar", 0, -_H, STAND_OPEN, STAND_BAR),
]
COASTER = Box(
    "coaster", pos=(COASTER_X, COASTER_Y, TOP + COASTER_SIZE[2] / 2), size=COASTER_SIZE, color=(0.12, 0.12, 0.13)
)
OBJECTS = [RISER, RAIL_A, RAIL_B, BALL, CHALK_CUBE, MUG, *STAND, COASTER, TAPE, SCREWDRIVER]

# The seated ball's centre above the table: it rests on the four inner edges of the stand's opening.
SEAT_Z = STAND_H + float(np.sqrt(BALL_R**2 - (STAND_OPEN / 2) ** 2))
SEAT_TOL = 0.003  # the seated ball's centre at most 3 mm above its seated height: actually resting on the edges
CHALK_TOL = 0.003  # the chalk's underside at most 3 mm above the coaster: actually lying on it
ON_TABLE = 0.003  # the ball's underside this close to the table top counts as on the table
MUG_TIP = 20.0  # degrees from upright
BALL_CLEAR = BALL_R + 0.02  # every fingertip point at least 2 cm from the ball's surface
CHALK_CLEAR = 0.035  # every fingertip point at least 3.5 cm from the chalk's centre (about 2 cm from its corners)
HOLD_STEPS = 60  # 3 s at the 20 Hz control rate
SUCCESS_HOLD_S = 0.0  # the trial record below counts its own 3 s hold


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _tilt_deg(obj):
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _over_stand(ball, margin):
    return bool(abs(ball.pos[0] - STAND_X) < margin and abs(ball.pos[1] - STAND_Y) < margin)


def _clear(obj, tips, dist):
    return bool(np.min(np.linalg.norm(np.asarray(tips) - np.asarray(obj.pos), axis=1)) >= dist)


def _ball_seated(ball, tips):
    """The ball sits in the stand's opening and nothing holds it: centre within 1 cm of the opening's centre in x
    and in y, no more than 3 mm above its seated height, at rest, and every fingertip 2 cm or more from it."""
    return bool(
        _over_stand(ball, 0.01)
        and ball.pos[2] < TOP + SEAT_Z + SEAT_TOL
        and ball.resting()
        and _clear(ball, tips, BALL_CLEAR)
    )


def _chalk_on_coaster(chalk, tips):
    """The chalk lies on the coaster and nothing holds it: centre over the coaster, underside within 3 mm of the
    coaster's top, at rest, and every fingertip about 2 cm or more from it."""
    p = chalk.pos
    return bool(
        abs(p[0] - COASTER_X) < COASTER_SIZE[0] / 2
        and abs(p[1] - COASTER_Y) < COASTER_SIZE[1] / 2
        and p[2] < TOP + COASTER_SIZE[2] + CHALK / 2 + CHALK_TOL
        and chalk.resting()
        and _clear(chalk, tips, CHALK_CLEAR)
    )


def _ball_on_table(ball):
    """The ball's underside is less than 3 mm above the table top (or below it: on the floor), anywhere but with
    its centre over the stand's opening. On the track, the stand's rim or the coaster it is at least 5 mm up."""
    return bool(ball.pos[2] < TOP + BALL_R + ON_TABLE and not _over_stand(ball, STAND_OPEN / 2))


class Trial:
    """The trial's record. Harm (the ball reaching the table, or the mug tipping) is latched for the rest of the
    trial; only a reset clears it, never a return of the objects to their places."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the recorded harm is cleared
        self.harm, self.hold, self.full, self.score = False, 0, False, 0.0

    def step(self, env):
        from yam_arena.scene import object_states  # simulator-only import

        o = object_states(env, ("cue_ball", "chalk", "mug"))
        tips = m.fingertips(env)
        ball, chalk = o["cue_ball"], o["chalk"]
        self.harm = bool(self.harm or _ball_on_table(ball) or _tilt_deg(o["mug"]) > MUG_TIP)
        seated = _ball_seated(ball, tips)
        both = seated and _chalk_on_coaster(chalk, tips)
        self.hold = self.hold + 1 if both and not self.harm else 0
        self.full = bool(self.full or (self.hold >= HOLD_STEPS and not self.harm))
        if self.harm:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = 0.5 if seated else 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    """0 if the ball ever reached the table or the mug ever tipped past 20 degrees. Otherwise 1.0 once the ball has
    sat released in the stand and the chalk lain released on the coaster, both at rest, for 3 s; 0.5 with only the
    ball released and at rest in the stand; else 0."""
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full)


# Where the ball and the chalk rest at the start (measured in the simulator); the demo is planned from these.
START = {
    "cue_ball": np.array([0.3300, 0.0406, 0.8102]),
    "chalk": np.array([0.3301, 0.0013, 0.7913]),
}
ACROSS = {"left": -np.pi / 2, "right": np.pi / 2}  # fingers close along x, across the track
TRAVEL = TOP + 0.15
REACH_Z = TOP + 0.10  # the right arm cannot hold the travel height this far across the midline
BALL_SLOT = (STAND_X, STAND_Y)
CHALK_SLOT = (COASTER_X, COASTER_Y)


def demo():
    """The left arm grips the ball on the track and lifts it clear; only then does the right arm take the chalk
    off the rails. The ball is seated in its stand and the chalk set on the coaster."""
    bx, by, bz = START["cue_ball"]
    cx, cy, cz = START["chalk"]
    cz += 0.008  # grip the chalk by its upper half, fingertips clear of the rails
    ball_down = (*BALL_SLOT, TOP + SEAT_Z + 0.002)
    chalk_down = (*CHALK_SLOT, TOP + COASTER_SIZE[2] + CHALK / 2 + 0.008 + 0.003)
    yl, yr = ACROSS["left"], ACROSS["right"]
    left = [
        Waypoint(2.0, (bx, by, TRAVEL), FINGER_OPEN, "above the ball", yl),
        Waypoint(1.5, (bx, by, bz), FINGER_OPEN, "descend", yl),
        Waypoint(1.0, (bx, by, bz), FINGER_CLOSED, "grip the ball", yl),
        Waypoint(1.5, (bx, by, TRAVEL), FINGER_CLOSED, "lift it off the track", yl),
        Waypoint(2.0, (*BALL_SLOT, TRAVEL), FINGER_CLOSED, "carry to the stand", yl),
        Waypoint(1.5, ball_down, FINGER_CLOSED, "lower", yl),
        Waypoint(1.0, ball_down, FINGER_OPEN, "let go", yl),
        Waypoint(1.5, (*BALL_SLOT, TRAVEL), FINGER_OPEN, "up", yl),
        Waypoint(1.5, (0.20, 0.22, TRAVEL), FINGER_OPEN, "move away", yl),
    ]
    right = [
        Waypoint(2.0, (cx, -0.14, TRAVEL), FINGER_OPEN, "wait beside the track", yr),
        
        Waypoint(1.5, (cx, cy, REACH_Z), FINGER_OPEN, "above the chalk", yr),
        Waypoint(1.5, (cx, cy, cz), FINGER_OPEN, "descend", yr),
        Waypoint(1.0, (cx, cy, cz), FINGER_CLOSED, "grip the chalk", yr),
        Waypoint(1.5, (cx, cy, REACH_Z), FINGER_CLOSED, "lift", yr),
        Waypoint(2.5, (*CHALK_SLOT, TRAVEL), FINGER_CLOSED, "carry to the coaster", yr),
        Waypoint(1.5, chalk_down, FINGER_CLOSED, "lower", yr),
        Waypoint(1.0, chalk_down, FINGER_OPEN, "let go", yr),
        Waypoint(1.5, (*CHALK_SLOT, TRAVEL), FINGER_OPEN, "up", yr),
        Waypoint(1.5, (0.20, -0.22, TRAVEL), FINGER_OPEN, "move away", yr),
    ]
    return {"left": [Waypoint(5.0, (0.20, 0.22, TRAVEL), FINGER_OPEN, "rest", yl)], "right": right}
