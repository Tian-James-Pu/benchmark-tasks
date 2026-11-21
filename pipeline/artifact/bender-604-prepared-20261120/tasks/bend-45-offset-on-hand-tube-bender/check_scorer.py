"""Checks every score line of task.py on hand-built tube shapes, with no simulator: python check_scorer.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task as t  # noqa: E402


def shape(*bends: tuple[float, float]) -> np.ndarray:
    """Hinge angles for bends given as (signed degrees, curve middle from E2 in m), each wrapped at the bend radius."""
    a = np.zeros(len(t.HINGES))
    for deg, mid in bends:
        span = t.R * math.radians(abs(deg))
        a0, a1 = mid - span / 2, mid + span / 2
        for k, u in enumerate(t.HINGES):
            lo, hi = max(a0, u - t.HINGE_STEP / 2), min(a1, u + t.HINGE_STEP / 2)
            if hi > lo:
                a[k] += math.copysign(math.degrees((hi - lo) / t.R), deg)
    return a


def run(hinges: np.ndarray, hands: float = 0.10, still: bool = True, steps: int = 25, scrap: str = "") -> float:
    tr = t.Trial()
    tr.scrap = scrap
    for _ in range(steps):
        tr.update(hinges, hands, still, 0.05)
    return tr.score


CASES = [
    ("straight tube", shape(), 0.0),
    ("perfect offset", shape((45, t.MARK_B), (-45, t.MARK_A)), 1.0),
    ("perfect offset, other hand", shape((-45, t.MARK_B), (45, t.MARK_A)), 1.0),
    ("43 and 47 degrees", shape((43, t.MARK_B), (-47, t.MARK_A)), 1.0),
    ("bends 4 mm off their marks", shape((45, t.MARK_B + 0.004), (-45, t.MARK_A + 0.004)), 1.0),
    ("bends 8 mm off their marks", shape((45, t.MARK_B + 0.008), (-45, t.MARK_A + 0.008)), 0.75),
    ("one bend 8 mm off", shape((45, t.MARK_B - 0.008), (-45, t.MARK_A)), 0.75),
    ("39 and 45 degrees at the marks", shape((39, t.MARK_B), (-45, t.MARK_A)), 0.75),
    ("bends 20 mm off their marks", shape((45, t.MARK_B + 0.02), (-45, t.MARK_A + 0.02)), 0.5),
    ("30 degree bends", shape((30, t.MARK_B), (-30, t.MARK_A)), 0.0),
    ("a U: both bends the same way", shape((45, t.MARK_B), (45, t.MARK_A)), 0.0),
    ("bend A only, at its mark", shape((-45, t.MARK_A)), 0.25),
    ("bend A only, 8 mm off", shape((-45, t.MARK_A + 0.008)), 0.0),
    ("one 30 degree bend", shape((30, t.MARK_A)), 0.0),
]


def main() -> None:
    bad = 0
    for name, hinges, want in CASES:
        got = run(hinges)
        bends = [(round(a, 1), round(1000 * u, 1)) for a, u in t.bends_from(hinges)]
        flag = "ok " if abs(got - want) < 1e-9 else "BAD"
        bad += flag == "BAD"
        print(f"{flag} {name}: {got} (want {want}); bends {bends}; offset {1000 * t.offset_of(hinges):.1f} mm")
    perfect = shape((45, t.MARK_B), (-45, t.MARK_A))
    for name, got, want in [("perfect, a gripper 2 cm from the tube", run(perfect, hands=0.02), 0.0),
                            ("perfect, still moving", run(perfect, still=False), 0.0),
                            ("perfect, clear for only 0.5 s", run(perfect, steps=10), 0.0),
                            ("perfect, but bent out of plane earlier", run(perfect, scrap="skew"), 0.0)]:
        flag = "ok " if got == want else "BAD"
        bad += flag == "BAD"
        print(f"{flag} {name}: {got} (want {want})")
    print("all ok" if not bad else f"{bad} BAD")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
