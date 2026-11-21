"""Offline checks of the end-state scorer (no simulator needed): python tasks/seat-mini-itx-board-in-server-chassis/test_scoring.py

Each case puts the board at a pose relative to its seated pose and checks which of the four holes count as engaged, and the bay test.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task as t  # noqa: E402

SEAT = np.array([t.BX, t.BY, t.Z_SEAT])
ZERO = np.zeros(3)


def quat(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return np.r_[a * math.sin(h), math.cos(h)]  # x, y, z, w


def check(name, offset=(0, 0, 0), q=(0, 0, 0, 1), n=None, in_bay=None, vel=ZERO):
    c = t.board_check(SEAT + np.asarray(offset, float), np.asarray(q, float), vel, ZERO)
    ok = (n is None or c["n"] == n) and (in_bay is None or c["in_bay"] == in_bay)
    print(f"{'ok  ' if ok else 'FAIL'} {name}: engaged {c['n']}/4, in bay {c['in_bay']}, "
          f"offsets {np.round(c['dxy'] * 1000, 2)} mm, gaps {np.round(c['dz'] * 1000, 2)} mm")
    return ok


cases = [
    check("seated", n=4, in_bay=True),
    check("0.5 mm off rearward (inside the 0.6 mm limit)", (0.0005, 0, 0), n=4),
    check("misaligned 0.8 mm sideways", (0, 0.0008, 0), n=0, in_bay=True),
    check("misaligned 2 mm (the earlier tolerance)", (0.002, 0, 0), n=0, in_bay=True),
    check("set down 7 mm forward, resting on the standoff tops", (-0.007, 0, 0), n=0, in_bay=True),
    check("held 0.6 mm above all four (a shim's height)", (0, 0, 0.0006), n=0, in_bay=True),
    check("held 0.25 mm above (a 0.3 mm gauge would not enter)", (0, 0, 0.00025), n=4),
    check("held 5 mm above", (0, 0, 0.005), n=0, in_bay=False),
    check("rotated 0.5 degrees about its centre", q=quat((0, 0, 1), 0.5), n=0),
    check("tilted 0.3 degrees about its left-right axis", q=quat((0, 1, 0), 0.3), n=2),
    check("on the foam, start pose", (t.SX - t.BX, t.SY - t.BY, t.Z_START - t.Z_SEAT), n=0, in_bay=False),
]
print(f"{sum(cases)}/{len(cases)} passed")
sys.exit(0 if all(cases) else 1)
