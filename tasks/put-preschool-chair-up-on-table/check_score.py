"""Feeds the real Trial in task.py hand-built chair poses and fingertip points, with no simulator, and checks the score
each sequence earns. Run from this folder: python check_score.py"""

import importlib.util
import math
import pathlib

import numpy as np

spec = importlib.util.spec_from_file_location("chair_task", pathlib.Path(__file__).with_name("task.py"))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
from mechkit import PartState  # noqa: E402  (task.py put this folder on the path)
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

FAR = np.tile([0.0, 0.6, 1.6], (24, 1))  # every fingertip well away from the chair


def quat(R):
    w = math.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-3:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
    # 180-degree turns: about y for the chair here
    axis = np.array([math.sqrt(max(0, (R[i, i] + 1) / 2)) for i in range(3)])
    return np.array([*axis, 0.0])


def state(R, pos, v=0.0):
    pos = np.asarray(pos, float)
    return PartState(pos=pos, quat_xyzw=quat(R), lin_vel=np.array([v, 0, 0]), ang_vel=np.zeros(3), low=pos, high=pos)


def ry(deg):
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(deg))


def holding(R, pos):
    """Fingertip points pinching both side edges of the seat (on its top face and on the rim's lower edge)."""
    pts = []
    for s in (1, -1):
        for z in (0.0, -t.RIM_D):
            for k in range(6):
                pts.append((t.GRIP_X, s * (t.SEAT_W / 2 - 0.01), z))
    return np.array(pts) @ R.T + pos


UP = np.eye(3)
FLOOR = (t.CHAIR_X0, 0.0, t.TOP + t.SEAT_H)


def on_table(overhang, tilt=0.0, lift=0.0):
    """Upside down on the top, the seat's rear edge `overhang` past the near edge; `tilt` raises the rear edge (the seat
    then rests on its front edge)."""
    R = ry(180 + tilt)
    p = np.array([t.X_EDGE - overhang + t.HX, 0.0, 0.0])
    face = t.SEAT_FACE @ R.T + p
    p[2] = t.TABLE_Z + lift - face[:, 2].min()
    return R, tuple(p)


def run(seq):
    tr = t.Trial()
    for st, tips, n in seq:
        for _ in range(n):
            tr.update(st, tips)
    return tr.score, tr


def lifted(h):
    return state(UP, (FLOOR[0], 0, FLOOR[2] + h), 0.05)


def air(deg):
    R = ry(deg)
    return state(R, (0.2, 0.0, t.TABLE_Z + 0.3), 0.05), holding(R, np.array((0.2, 0.0, t.TABLE_Z + 0.3)))


lift_seq = [(lifted(0.06), holding(UP, np.array((FLOOR[0], 0, FLOOR[2] + 0.06))), 5)]
turn_seq = lift_seq + [(air(180)[0], air(180)[1], 5)]


def placed(overhang, tilt=0.0, tips=None, n=80, lift=0.0, v=0.0):
    R, p = on_table(overhang, tilt, lift)
    st = state(R, p, v)
    return [(st, holding(R, np.array(p)), 5), (st, FAR if tips is None else tips(R, np.array(p)), n)]


CASES = [
    ("left standing, hands away", [(state(UP, FLOOR), FAR, 100)], 0.0),
    ("lifted 6 cm in both hands, set back down", lift_seq + [(state(UP, FLOOR), FAR, 60)], 0.25),
    ("turned over in the air, put upside down on the floor", turn_seq + [(state(ry(180), (0.1, 0, t.TOP + 0.0)), FAR, 60)], 0.5),
    ("set down flat, 11 cm over the edge, let go (not pushed home)", turn_seq + placed(0.11), 0.75),
    ("set down flat and pushed home (2 cm over), let go", turn_seq + placed(0.02), 1.0),
    ("pushed home, 3.9 cm over", turn_seq + placed(0.039), 1.0),
    ("pushed home but 4.5 cm over", turn_seq + placed(0.045), 0.75),
    ("home, but a hand left 2 cm from the seat", turn_seq + placed(0.02, tips=lambda R, p: holding(R, p) + np.array([0, 0, 0.02 + t.RIM_D])), 0.5),
    ("home, but still held", turn_seq + placed(0.02, tips=holding), 0.5),
    ("home, but still for only 1.5 s", turn_seq + placed(0.02, n=30), 0.75),
    ("on the top tilted 8 deg", turn_seq + placed(0.02, tilt=8.0), 0.75),
    ("still sliding at 2 cm/s", turn_seq + placed(0.02, v=0.02), 0.5),
]


def drop_case():
    """Let go with the seat 4 cm over the top (the careless way), then it lands home: still 0."""
    R, p = on_table(0.02, 0.0, 0.04)
    falling = state(R, p, 0.0)
    return turn_seq + [(falling, FAR, 2)] + placed(0.02)


def leg_case():
    """Carried turned past level so a front leg's shaft touches the tabletop, then set home: 0."""
    R = ry(215)
    st = state(R, (0.0, 0.0, 0.0))
    legs = st.to_world(t.LEG_PTS)
    d = t.table_top_distance(legs) - t.TUBE_R
    # move the chair so its nearest leg point sits 2 mm from the top's face
    i = int(np.argmin(d))
    shift = np.array([t.X_EDGE + 0.05 - legs[i, 0], 0.0, t.TABLE_Z + t.TUBE_R + 0.002 - legs[i, 2]])
    st = state(R, shift)
    return turn_seq + [(st, holding(R, shift), 1)] + placed(0.02)


CASES += [("let go 4 cm over the top, lands home", drop_case(), 0.0),
          ("a leg's shaft within 2 mm of the top while held, then home", leg_case(), 0.0)]

ok = True
for name, seq, want in CASES:
    got, tr = run(seq)
    flag = "ok " if abs(got - want) < 1e-9 else "BAD"
    ok &= flag == "ok "
    print(f"{flag} {got:.2f} (want {want:.2f})  {name}")
print("all as stated" if ok else "MISMATCH")
