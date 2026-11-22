"""On the station table, in front of the robot: a life-size adult left hand form stands upright on a 150 x 150 mm steel
plate, fingers up and spread in one vertical plane, palm toward the robot; its ring finger is vertical about 300 mm in
front of the arms, midway between them, with a soft silicone skin and three thin brown painted lines. A plain
stainless-steel ring (US 7.5: 17.75 mm bore, 4 mm wide) lies flat on the table in front of the left arm.

The job: put the ring on the hand form's ring finger, all the way down to the base of the finger, and leave it there.
The ring is a rigid body, the hand form a rigid body resting on the table, and the ring finger's skin native volume FEM;
nothing is attached to anything. Load it with yam_arena (``-E task=task.py``) or run it with run.py. Scoring:
scoring.py (the card's rule, read from the ring's and hand form's poses).
"""
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from yam_arena.task_file import TABLE_TOP as _YAM_TABLE_TOP  # noqa: E402

import arena_ring  # noqa: E402  (installs the yam_arena hooks)
import geometry as G  # noqa: E402
import scene as layout  # noqa: E402
from arena_ring import TRIAL, HandForm, Skin, SteelRing  # noqa: E402
from scoring import ScoreTracker  # noqa: E402

if abs(_YAM_TABLE_TOP['station'] - G.TABLE_TOP) > 1e-9:
    raise RuntimeError('station table height changed; re-derive the layout')

SCENE = 'station'
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 0.0   # the scorer's Still condition already needs 1 s
INSTRUCTION = ("Put the ring on the hand form's ring finger, all the way down to the base of the finger, and leave it there.")

PHYSICS_HZ = int(os.environ.get('ROBOCURVE_RING_PHYSICS_HZ', str(layout.PHYSICS_HZ)))
try:
    from yam_arena import scene as _yscene  # noqa: E402
    _yscene.PHYSICS_DT = 1.0 / PHYSICS_HZ
except ImportError:  # pragma: no cover
    pass

SEED = TRIAL['seed']
HAND_ORIGIN = layout.hand_origin(SEED)
RING_START = layout.ring_start(SEED)
_skin_c = layout.skin_centre()
OBJECTS = [
    HandForm('hand', pos=HAND_ORIGIN, size=(.15, .15, .2), color=layout.COLORS['pla']),
    Skin('skin', pos=tuple(float(HAND_ORIGIN[i] + _skin_c[i]) for i in range(3)), size=(.02, .02, .06),
         color=layout.COLORS['skin']),
    SteelRing('ring', pos=RING_START, size=(.0216, .0216, .004), color=layout.COLORS['ring']),
]


def evaluate(objects):
    """One scorer update per control step from the rigid poses (robot distance read by run.py at the end)."""
    tracker = TRIAL['tracker']
    if tracker is None:
        tracker = TRIAL['tracker'] = ScoreTracker(20.0)
    ring, hand = objects['ring'], objects['hand']
    return tracker.update(ring.pos, ring.rot, hand.pos, hand.rot, read_robot=False)


def score(objects):
    return float(evaluate(objects)['score'])


def success(objects):
    return bool(evaluate(objects)['success'])
