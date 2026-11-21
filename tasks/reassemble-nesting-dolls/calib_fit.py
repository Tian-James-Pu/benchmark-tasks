"""Calibration run (not the task): the first doll's top set square on its spigot with no hands, then a slowly growing downward force on
it (1 N/s, on its axis at the rim) until it seats; then, with the bottom held down, a growing upward pull until it comes off. Writes FIT_LOG."""
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
os.environ["TASK_DEMO"] = "none"
import numpy as np  # noqa: E402

import mechkit as m  # noqa: E402
import task as T  # noqa: E402
from task import INSTRUCTION, SCENE, SUCCESS_HOLD_S, TOP, demo, score, success  # noqa: E402,F401

K = int(os.environ.get("FIT_DOLL", "1"))
_b = T.B1_POS if K == 1 else T.B2_POS
_d = T.dims(K)
OBJECTS = [o for o in T.OBJECTS if o.name != f"top{K}"] + [T._top(f"top{K}", K, (_b[0], _b[1], TOP + (_d["zs"] + _d["sp_h"] - 1.0) * T.MM), 180.0)]
LOG = os.environ.get("FIT_LOG", "/tmp/fit.log")
_n = [0]


def _calib(env):
    _n[0] += 1
    k = _n[0]
    st_t, st_b = m.state(env, f"top{K}"), m.state(env, f"bottom{K}")
    p, tilt, turn = T._rel(st_t, st_b)
    dz = (p[2] - _d["zs"] * T.MM) * 1000
    if k < 40:
        f = 0.0
        m.push(env, f"top{K}", (0.0, 0.0, 0.0))
    elif k < 380:
        f = -0.05 * (k - 40)  # down, 1 N/s, to 17 N, on the top's axis at its rim (the centre of the bore's mouth)
        m.push(env, f"top{K}", (0.0, 0.0, f), at=tuple(st_t.pos))
    elif k < 420:
        f = 0.0
        m.push(env, f"top{K}", (0.0, 0.0, 0.0))
        m.push(env, f"bottom{K}", (0.0, 0.0, -30.0))
    else:
        f = 0.05 * (k - 420)  # up, 1 N/s, on the same point
        m.push(env, f"top{K}", (0.0, 0.0, f), at=tuple(st_t.pos))
    if k % 5 == 0:
        pads = [round(v * 1000, 2) for v in st_b.joints.values()]
        with open(LOG, "a") as fh:
            fh.write(f"{k} force {f:+.2f} N dz {dz:+.2f} mm off {math.hypot(p[0], p[1]) * 1000:.2f} tilt {tilt:.2f} pads {pads}\n")


m.on_step(_calib)
