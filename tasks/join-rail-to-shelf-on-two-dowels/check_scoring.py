"""Offline checks of the scorer in task.py on made-up board poses (no simulator): python check_scoring.py"""

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location("dowel_task", HERE / "task.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
import mechkit as m  # noqa: E402

DT = 0.05
FAR = np.full((24, 3), 5.0)  # gripper points far away: hands clear


def quat(yaw_deg=0.0, roll_deg=0.0):
    """xyzw for a yaw about z after a roll about x."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    # q = q_yaw * q_roll
    w = cy * cr
    x = cy * sr
    y = sy * sr
    z = sy * cr
    return np.array([x, y, z, w])


def state(pos, q=None, v=(0, 0, 0), w=(0, 0, 0)):
    p = np.asarray(pos, float)
    return m.PartState(pos=p, quat_xyzw=quat() if q is None else q, lin_vel=np.asarray(v, float), ang_vel=np.asarray(w, float), low=p, high=p)


SX = 0.27
SHELF = state((SX, 0.0, T.TOP))


def rail_at(gap_mm, dy=0.0, dz=0.0, yaw=0.0, roll=0.0, shelf_x=SX, shelf_y=0.0):
    """The rail square to the shelf, its holed face `gap_mm` from the shelf's edge."""
    x = shelf_x - T.FACE_X - gap_mm / 1000
    z = T.TOP + dz
    if abs(roll) > 90:  # upside down about its own x: the origin (underside) is now on top
        z += T.T
    return state((x, shelf_y + dy, z), quat(yaw, roll))


def run(shelf, rail, tips=FAR, steps=1, trial=None):
    trial = trial or T.Trial()
    v = None
    for _ in range(steps):
        v = trial.update(shelf, rail, tips, DT)
    return v, trial


def tips_on_rail(rail):
    """Gripper points on the rail's ends, as while it is still held."""
    return np.vstack([rail.to_world(np.array([[0.0, y, 0.009]] * 12)) for y in (0.12, -0.12)])


CASES = []


def case(name, got, want):
    ok = abs(got - want) < 1e-9
    CASES.append(ok)
    print(f"{'ok ' if ok else 'BAD'} {name}: {got} (want {want})")


# the starting layout
start = state((0.200, 0.010, T.TOP), quat(4.0))
case("starting layout", run(SHELF, start)[0], 0.0)
# seated, released, still: 0.75 until the hold reaches 3 s, then 1.0
seated = rail_at(0.0)
case("seated, hands clear, 2.9 s", run(SHELF, seated, steps=58)[0], 0.75)
case("seated, hands clear, 3.0 s", run(SHELF, seated, steps=60)[0], 1.0)
case("seated, still held", run(SHELF, seated, tips_on_rail(seated), steps=100)[0], 0.75)
case("seated, a hand 25 mm from the rail", run(SHELF, seated, np.tile(seated.to_world(np.array([0.0, 0.0, 0.018 + 0.025])), (24, 1)), steps=100)[0], 0.75)
case("seated, hands 35 mm above the rail", run(SHELF, seated, np.tile(seated.to_world(np.array([0.0, 0.0, 0.018 + 0.035])), (24, 1)), steps=60)[0], 1.0)
case("seated but the rail sliding at 6 mm/s", run(SHELF, state(seated.pos, seated.quat_xyzw, v=(0.006, 0, 0)), steps=100)[0], 0.75)
case("seam 0.5 mm", run(SHELF, rail_at(0.5), steps=60)[0], 1.0)
case("seam 0.6 mm", run(SHELF, rail_at(0.6), steps=60)[0], 0.5)
# partial rungs (final state)
case("seam 7.0 mm (half on)", run(SHELF, rail_at(7.0))[0], 0.5)
case("seam 7.1 mm", run(SHELF, rail_at(7.1))[0], 0.25)
case("seam 12.0 mm (tips in)", run(SHELF, rail_at(12.0))[0], 0.25)
case("seam 12.1 mm", run(SHELF, rail_at(12.1))[0], 0.0)
case("seam 30 mm", run(SHELF, rail_at(30.0))[0], 0.0)
# shortcuts and impossible poses
case("rail stacked on the shelf over the dowels", run(SHELF, rail_at(-10.0, dz=T.T))[0], 0.0)
case("rail on the shelf, seam -2 mm overlap", run(SHELF, rail_at(-2.0, dz=T.T))[0], 0.0)
case("boards overlapping by 2 mm at bench height", run(SHELF, rail_at(-2.0))[0], 0.0)
case("rail held 2.5 mm above the bench, seam 5 mm", run(SHELF, rail_at(5.0, dz=0.0025))[0], 0.0)
case("rail held 1.5 mm above the bench, seam 5 mm", run(SHELF, rail_at(5.0, dz=0.0015))[0], 0.0)  # tip 1.5 mm off the axis
case("rail 1.5 mm to the side, seam 5 mm", run(SHELF, rail_at(5.0, dy=0.0015))[0], 0.0)
case("rail 96 mm to the side (one dowel in the other hole)", run(SHELF, rail_at(5.0, dy=0.096))[0], 0.0)
case("rail turned 1 deg, nearer corner touching", run(SHELF, rail_at(0.0, yaw=1.0))[0], 0.0)
case("rail tilted 3.5 deg, seam 0", run(SHELF, rail_at(0.0, roll=3.5))[0], 0.0)
case("rail upside down, seated", run(SHELF, rail_at(0.0, roll=180.0), steps=60)[0], 1.0)
case("shelf pushed 10 cm along the bench, rail seated on it", run(state((SX + 0.10, 0.02, T.TOP)), rail_at(0.0, shelf_x=SX + 0.10, shelf_y=0.02), steps=60)[0], 1.0)
case("shelf lifted 5 mm (held up), rail seated", run(state((SX, 0.0, T.TOP + 0.005)), rail_at(0.0, dz=0.005), steps=60)[0], 0.0)
# only the final state counts
tr = T.Trial()
run(SHELF, seated, steps=40, trial=tr)
case("seated 2 s then pulled apart to 20 mm", run(SHELF, rail_at(20.0), trial=tr)[0], 0.0)
tr = T.Trial()
run(SHELF, rail_at(12.0), steps=40, trial=tr)
case("tips in, then pulled off", run(SHELF, rail_at(15.0), trial=tr)[0], 0.0)
tr = T.Trial()
run(SHELF, seated, steps=50, trial=tr)
run(SHELF, seated, tips_on_rail(seated), steps=1, trial=tr)  # touched again: the hold restarts
case("hold restarts when a hand comes back", run(SHELF, seated, steps=50, trial=tr)[0], 0.75)
tr = T.Trial()
run(SHELF, seated, steps=50, trial=tr)
tr.reset()
case("reset clears the hold", tr.hold, 0.0)
# measure() on the real geometry
r = T.measure(SHELF, start)
print("start:", r)
assert all(abs(d["gap_mm"] - 0.0) > 30 for d in r["dowels"])

print(f"{sum(CASES)}/{len(CASES)} passed")
sys.exit(0 if all(CASES) else 1)
