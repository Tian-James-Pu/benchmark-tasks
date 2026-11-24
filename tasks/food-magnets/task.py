"""On the station table, in front of the robot: a galvanized steel sheet (610 x 305 mm) stands upright on a plywood
stand 400 mm out. On it, one per cell of six in a random order: a red strawberry, a tan cookie and a pink-topped
doughnut magnet, a plain black disc magnet holding a white 4 x 6 in note card, and two ordinary plain black magnets (a
30 mm disc and a 30 mm square) that hold nothing. A small plywood box with its hinged lid resting open sits on the table
between the arms and the board, at one of four spots.

The job (instruction below): take the three food-shaped magnets off the board into the box and shut the lid, leaving
the note, its plain magnet and the two ordinary magnets where they are. Every body is a native PhysX rigid body; the board and box body are
static (taped), the lid turns on a revolute hinge, and each magnet is held to the steel by a gap-dependent attraction
(arena_fm.MagnetForces). Load it with yam_arena (``-E task=task.py``) or run it with run.py. Scoring: scoring.py.
"""
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from yam_arena.task_file import TABLE_TOP as _YAM_TABLE_TOP  # noqa: E402

import arena_fm  # noqa: E402  (installs the yam_arena hooks)
import geometry as G  # noqa: E402
import scene as layout  # noqa: E402
from arena_fm import TRIAL, Lid, Magnet, NoteCard  # noqa: E402

if abs(_YAM_TABLE_TOP['station'] - G.TABLE_TOP) > 1e-9:
    raise RuntimeError('station table height changed; re-derive the layout')

SCENE = 'station'
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 1.0
INSTRUCTION = ("Dad has Alzheimer's and moves in tonight. Get the kitchen magnet board ready for him. "
               "Anything you take down goes in the wooden box, lid shut.")

PHYSICS_HZ = int(os.environ.get('ROBOCURVE_FM_PHYSICS_HZ', str(layout.PHYSICS_HZ)))
try:
    from yam_arena import scene as _yscene  # noqa: E402
    _yscene.PHYSICS_DT = 1.0 / PHYSICS_HZ
except ImportError:  # pragma: no cover
    pass

SEED = TRIAL['seed']
DRAW = TRIAL['draw'] = layout.draw(SEED)
OBJECTS = [
    *(Magnet(n, pos=layout.body_start(n, DRAW), size=(G.THICK[n], *G.EXTENT[n]), color=layout.COLORS[n])
      for n in ('strawberry', 'cookie', 'donut', 'plain') + G.ORDINARY),
    NoteCard('card', pos=layout.card_start(DRAW), size=(G.CARD_T, G.CARD_W, G.CARD_H), color=layout.COLORS['card']),
    Lid('lid', pos=layout.lid_start(DRAW), size=(G.BOX_LX, G.BOX_LY, G.LID_T), color=layout.COLORS['box']),
]


def _tracker():
    from scoring import ScoreTracker
    if TRIAL['tracker'] is None:
        TRIAL['tracker'] = ScoreTracker(DRAW, 20.0)
    return TRIAL['tracker']


def evaluate(objects):
    """One scorer update per control step from the rigid poses (the grippers' distances are read by run.py)."""
    poses = {n: (objects[n].pos, objects[n].rot) for n in ('strawberry', 'cookie', 'donut', 'plain', 'card', 'lid') + G.ORDINARY}
    return _tracker().update(poses)


def score(objects):
    return float(evaluate(objects)['score'])


def success(objects):
    return bool(evaluate(objects)['success'])
