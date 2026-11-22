"""The scene of task.py, unchanged, seen by the overhead camera moved out to the front right of the bed, for the setup image's
wide view of the whole bed frame. Not a trial: run it for a few steps with any policy.

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=<this folder>/setup_view.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=640 -E cam_height=360 --instruction "look" --scorer task_score --max-steps 3 --store-frames
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from task import *  # noqa: E402,F401,F403  the same scene, scorer and demo
from task import TOP, _xyzw  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.rig import Camera  # noqa: E402

EYE = np.array([3.05, -2.75, TOP + 2.05])
LOOK_AT = np.array([0.95, -0.80, TOP + 0.10])
_overhead = station.top_camera


def _wide(variant: str = "calibrated") -> Camera:
    cam = _overhead(variant)
    back = EYE - LOOK_AT
    z = back / np.linalg.norm(back)  # OpenGL convention: the camera looks along its -z, its y up
    x = np.cross((0.0, 0.0, 1.0), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return Camera(tuple(float(v) for v in EYE), _xyzw(np.column_stack([x, y, z])), "opengl", cam.lens)


station.top_camera = _wide
station.camera_stand = lambda camera: {}  # the overhead camera's stand would stand in front of this view
