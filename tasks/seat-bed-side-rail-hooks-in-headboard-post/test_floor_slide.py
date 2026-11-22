"""Regression for the review's floor-slide route: the rail's near end resting on the floor anywhere beside or in line with the
leg, short of or past its face, must have neither hook in, so it scores 0. And the seated pose must still read as seated.

Pure Python: it runs task.py's own `hook_readings`, `hooked` and `seated` on rail poses built with task.py's `rail_frame` (the
rail lying on the helper support's dowel, its near end's underside on the floor). Needs only yam_arena importable (no simulator).

    python tasks/seat-bed-side-rail-hooks-in-headboard-post/test_floor_slide.py
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task as t  # noqa: E402


class _Pose:
    """A rail pose: rotation R and origin p (its near end's underside centre), as PartState.to_world reads it."""

    def __init__(self, end):
        self.p = np.asarray(end, float)
        self.r = t.rail_frame(self.p)

    def to_world(self, pts):
        return np.asarray(pts, float) @ self.r.T + self.p


def hooks_in(end):
    read = t.hook_readings(_Pose(end))
    return sum(t.hooked(*read[h]) for h in t.HOOKS), read


def main():
    worst = 0
    n = 0
    for dx in np.arange(-0.12, 0.1201, 0.005):  # end's centre from the slots' line: beside the leg either side, and in line
        for dy in np.arange(-0.05, 0.1501, 0.005):  # short of the face (negative) to 15 cm past it
            end = (t.X_R + dx, t.Y0 + dy, t.TOP)
            k, read = hooks_in(end)
            n += 1
            worst = max(worst, k)
            if k:
                print("FAIL: hook in with the end on the floor at", end, read)
    print(f"floor poses checked: {n}; most hooks in at any of them: {worst}")
    k, read = hooks_in((t.X_R, t.Y0, t.TOP + t.HB))
    seated = all(t.seated(*read[h]) for h in t.HOOKS)
    print(f"seated pose: hooks in {k}, both seated {seated}")
    ok = worst == 0 and k == 2 and seated
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
