"""Take the top crisp out of a fresh can of Pringles, whole, and put it on the plate.

A fresh 5.2 oz can of Pringles Original stands on the table with its lid off and its foil seal peeled away. The
saddle-shaped crisps are stacked nested, the top one about 1.4 cm below the rim. The robot must take out the top
crisp unbroken and leave it resting on the plate, without breaking the next crisp or pulling it out of its place.

Mechanism (mechkit.py, rigid bodies joined by PhysX joints):

- Each of the top two crisps is two rigid halves (the two sides of its long axis), joined along the crisp's long
  axis by a brittle bond: a hinge about that line on a bending spring of K_BEND N m/rad that lets go for good at
  SNAP_DEG degrees of bend (a bending moment of about 0.01 N m), and also breaks outright when the force through
  it passes SNAP_N newtons. Squeezing the crisp across its width harder than about 1.5 N, pressing an unsupported
  edge down, or snagging it hard on the can snaps it in two, and it stays snapped for the rest of the trial.
- Each crisp rests on the one below on four 1 mm contact pads near its middle, standing for the surface
  irregularity that sets a real stack's pitch of about 2.6 mm (a 5.2 oz can holds about 75 crisps), so a crisp's
  edge stands about 1.4 mm clear of the edge below it.
- The rest of the stack (crisp three downward) is part of the can's rigid body; tipping the can past 30 degrees
  counts as spilling the stack.

Scoring reads only end-state geometry that a person can check by eye, a ruler and a photo: whether each crisp is
in one piece, where its pieces lie, whether the next crisp is still in its place in the can, and whether the can
stayed upright.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  rigid parts, brittle bonds, per-episode memory
from simready import SimReady  # noqa: E402  NVIDIA SimReady props
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the can: Pringles Original 5.2 oz (147 g), lid off, foil seal peeled ----------------------------------------
CAN_XY = (0.25, -0.18)
CAN_R_IN = 0.0355  # 71 mm bore
CAN_WALL = 0.002  # collider wall; outer diameter 75 mm
CAN_H = 0.232  # can body height without the lid
CAN_MASS = 0.18  # can plus the ~73 crisps left below the top two
N_WALL = 24

# -- a crisp: a saddle (hyperbolic paraboloid) z = CX x^2 - CY y^2 over a 66 x 46 mm oval -------------------------
CRISP_A, CRISP_B = 0.033, 0.023  # half-length (x, the long axis) and half-width (y)
RISE, DROP = 0.006, 0.005  # the long-axis tips stand RISE above the middle; the short-axis edges DROP below it
CX, CY = RISE / CRISP_A**2, DROP / CRISP_B**2
CRISP_T = 0.0012  # thickness
PITCH = 0.0026  # stack pitch
NUB_H = 0.0009  # contact pads under each crisp
NX, NY = 8, 6  # tiles
HALF_MASS = 0.00095  # a crisp is about 1.9 g
Z1 = CAN_H - 0.014  # height of the top crisp's middle above the can's bottom
Z2, Z3 = Z1 - PITCH, Z1 - 2 * PITCH

# -- the brittle bond (estimated from a crisp snapping at about 1.5 N) --------------------------------------------
K_BEND = 0.15  # N m/rad
SNAP_DEG = 4.0  # bend at which it snaps: about 0.0105 N m
SNAP_N = 1.5  # force through the bond at which it snaps

# -- the plate and the scoring -------------------------------------------------------------------------------------
PLATE_XY = (0.26, 0.02)
PLATE_R_OK = 0.07  # a piece's centre within this of the plate's centre counts as on the plate (plate is 172 mm)
ON_PLATE_DZ = 0.03  # and its bottom within this of the plate's bottom
SEAT_MM, SEAT_DEG = 4.0, 8.0  # the next crisp still in its place in the can
UPRIGHT_DEG, SPILL_DEG = 10.0, 30.0
CLEAR_M = 0.03  # every finger point this far from every piece of the top crisp
STILL_V = 0.02
SUCCESS_HOLD_S = 1.0

RED = (0.72, 0.04, 0.05)
FOIL = (0.78, 0.78, 0.80)
CRISP = (0.93, 0.76, 0.38)


def saddle(x, y):
    return CX * x * x - CY * y * y


def _tile(x, y, dx, dy, color=None):
    zx, zy = 2 * CX * x, -2 * CY * y
    n = np.array([-zx, -zy, 1.0])
    n /= np.linalg.norm(n)
    tx = np.array([1.0, 0.0, zx])
    tx /= np.linalg.norm(tx)
    ty = np.cross(n, tx)
    q = mk.mat_to_quat(np.column_stack([tx, ty, n]))
    size = (dx * math.sqrt(1 + zx * zx) * 1.04, dy * math.sqrt(1 + zy * zy) * 1.04, CRISP_T)
    return mk.box(size, (x, y, saddle(x, y)), color=color, quat=q)


def _tile_centres():
    dx, dy = 2 * CRISP_A / NX, 2 * CRISP_B / NY
    out = []
    for i in range(NX):
        x = -CRISP_A + dx * (i + 0.5)
        for j in range(NY):
            y = -CRISP_B + dy * (j + 0.5)
            if (x / CRISP_A) ** 2 + (y / CRISP_B) ** 2 <= 1.0:
                out.append((x, y))
    return out, dx, dy


TILES, TDX, TDY = _tile_centres()
NUBS = [(sx * 0.008, sy * 0.010) for sx in (-1, 1) for sy in (-1, 1)]


def _half(sign, nubs=True, color=None):
    boxes = [_tile(x, y, TDX, TDY, color) for x, y in TILES if y * sign > 0]
    if nubs:
        for x, y in NUBS:
            if y * sign > 0:
                z = saddle(x, y) - 0.0006 - NUB_H / 2 + 0.0002
                boxes.append(mk.box((0.003, 0.003, NUB_H + 0.0004), (x, y, z), color=color))
    return boxes


kit = mk.Kit()
CX0, CY0 = CAN_XY

# The can, with the rest of the stack inside it as part of its body.
_can = []
for k in range(N_WALL):
    a = 2 * math.pi * k / N_WALL
    r = CAN_R_IN + CAN_WALL / 2
    _can.append(mk.box((CAN_WALL, 2 * math.pi * r / N_WALL * 1.08, CAN_H), (r * math.cos(a), r * math.sin(a), CAN_H / 2),
                       color=RED, yaw=a))
_can.append(mk.box((0.05, 0.05, 0.003), (0.0, 0.0, 0.0015), color=FOIL))
_fill_top = Z3 - DROP - 0.002
_can.append(mk.box((0.058, 0.038, _fill_top - 0.003), (0.0, 0.0, 0.003 + (_fill_top - 0.003) / 2), color=CRISP))
for b in _half(1, nubs=False) + _half(-1, nubs=False):
    _can.append(mk.box(b.size, (b.at[0], b.at[1], b.at[2] + Z3), color=CRISP, quat=b.quat))
kit.part("can", (CX0, CY0, TOP), _can, mass=CAN_MASS, color=RED)

CHIPS = {"chip1": Z1, "chip2": Z2}
HALVES = []
for name, z in CHIPS.items():
    for tag, sign in (("a", 1), ("b", -1)):
        kit.part(f"{name}_{tag}", (CX0, CY0, TOP + z), _half(sign), mass=HALF_MASS, color=CRISP)
        HALVES.append(f"{name}_{tag}")
    kit.bond(f"{name}_snap", f"{name}_b", f"{name}_a", break_force=SNAP_N, anchor=(CX0, CY0, TOP + z),
             twist_axis=(1.0, 0.0, 0.0), twist_release_deg=SNAP_DEG, twist_stiffness=K_BEND)

OBJECTS = [
    *kit.objects(),
    SimReady("plate", pos=(PLATE_XY[0], PLATE_XY[1], TOP), size=(0.172, 0.177, 0.026), mass=0.15,
             package="dishware_plastic_bigPlate_a01", usd="sm_dishware_plastic_bigPlate_a01_01.usd"),
    # distractors: scoring ignores them
    SimReady("soda_can", pos=(0.20, 0.17, TOP), size=(0.067, 0.067, 0.092), mass=0.37,
             package="food_beverage_can_b01", usd="sm_food_beverage_can_b01_01.usd"),
    SimReady("paper_cup", pos=(0.40, 0.16, TOP), size=(0.080, 0.080, 0.110), mass=0.012,
             package="dishware_cup_paper_a01", usd="sm_dishware_cup_paper_a01_01.usd"),
]

# -- reading the scene -------------------------------------------------------------------------------------------

def finger_points():
    """World points on the lower 4 cm of all four finger pads (sim only; empty offline): for each arm, from its TCP
    (between the pads, 13 mm short of their tips), the gripper's approach axis, the line between its fingers and the
    finger joints' opening."""
    env = mk._EP.get("env")
    if env is None:
        return np.zeros((0, 3))
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    tcps = mk.tcps()
    out = []
    for k, key in enumerate(("left_robot", "right_robot")):
        art = base.scene[key]
        names = list(art.body_names)
        pos = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin
        pos = pos.detach().cpu().numpy().astype(float)
        jn = list(art.joint_names)
        q = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0].detach().cpu().numpy().astype(float)
        half = (abs(q[jn.index("joint7")]) + abs(q[jn.index("joint8")])) / 2
        tcp = np.asarray(tcps[k], float)
        a = tcp - pos[names.index("gripper")]
        a /= np.linalg.norm(a)
        c = pos[names.index("tip_left")] - pos[names.index("tip_right")]
        c -= (c @ a) * a
        c /= max(np.linalg.norm(c), 1e-9)
        wdir = np.cross(a, c)
        for side in (1.0, -1.0):
            for off in (0.0, 0.004):
                for along in (0.013, 0.0, -0.015, -0.03):
                    for wd in (-0.0077, 0.0, 0.0077):
                        out.append(tcp + side * (half + off) * c + along * a + wd * wdir)
    return np.array(out)


def _tilt_deg(quat_xyzw) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(mk.quat_to_mat(quat_xyzw)[2, 2])))))


def _rel(objects, part):
    """`part`'s pose in the can's frame, and its reference pose there."""
    tc, tp = kit._pose_T(objects, "can"), kit._pose_T(objects, part)
    now = np.linalg.inv(tc) @ tp
    ref = np.linalg.inv(kit.parts["can"].ref) @ kit.parts[part].ref
    return now, ref


def _seated(objects, part) -> bool:
    now, ref = _rel(objects, part)
    d = np.linalg.norm(now[:3, 3] - ref[:3, 3]) * 1000.0
    ang = math.degrees(mk._rot_angle(now[:3, :3] @ ref[:3, :3].T))
    return d <= SEAT_MM and ang <= SEAT_DEG


def _piece_points(objects, part):
    return np.array([kit.point(objects, part, (x, y, saddle(x, y))) for x, y in TILES
                     if (y > 0) == part.endswith("_a")])


def _on_plate(objects, part) -> bool:
    st, plate = objects[part], objects["plate"]
    d = math.hypot(st.pos[0] - plate.pos[0], st.pos[1] - plate.pos[1])
    dz = float(st.low[2] - plate.low[2])
    return bool(d <= PLATE_R_OK and -0.005 <= dz <= ON_PLATE_DZ and np.linalg.norm(st.lin_vel) < STILL_V)


def _hands_clear(objects) -> bool:
    f = finger_points()
    if len(f) == 0:
        return bool(mk.MEMORY.get("offline_clear", True))
    for part in ("chip1_a", "chip1_b"):
        p = _piece_points(objects, part)
        if np.min(np.linalg.norm(f[:, None, :] - p[None, :, :], axis=2)) < CLEAR_M:
            return False
    return True


def _rules(objects) -> None:
    """Every control step: latch a spill. Cleared only at reset (mechkit clears MEMORY in its reset hook)."""
    if _tilt_deg(objects["can"].quat_xyzw) > SPILL_DEG:
        mk.MEMORY["spilled"] = True


kit.on_step(_rules)


def evaluate(objects) -> dict:
    spilled = bool(mk.MEMORY.get("spilled", False))
    can = objects["can"]
    upright = _tilt_deg(can.quat_xyzw) <= UPRIGHT_DEG
    top_whole = not kit.broken(objects, "chip1_snap")
    next_whole = not kit.broken(objects, "chip2_snap")
    next_seated = upright and all(_seated(objects, p) for p in ("chip2_a", "chip2_b"))
    on_plate = [_on_plate(objects, p) for p in ("chip1_a", "chip1_b")]
    clear = _hands_clear(objects)
    still = float(np.linalg.norm(can.lin_vel)) < STILL_V
    top_ok = top_whole and all(on_plate)
    top_part = (not top_whole) and any(on_plate)
    next_ok = next_whole and next_seated
    if spilled or not clear or not (top_ok or top_part):
        s = 0.0
    elif top_ok and next_ok:
        s = 1.0
    elif top_ok or (top_part and next_ok):
        s = 0.5
    else:
        s = 0.25
    return dict(score=s, full=bool(s == 1.0 and still), spilled=spilled, upright=upright, top_whole=top_whole,
                next_whole=next_whole, next_seated=next_seated, on_plate=on_plate, clear=clear)


def score(objects) -> float:
    return float(evaluate(objects)["score"])


def success(objects) -> bool:
    return bool(evaluate(objects)["full"])


# -- scripted demo (closed loop; run with demokit.py) --------------------------------------------------------------

GRIP_TILT = 15.0
ENTRY_W = 0.052  # finger opening while inside the can
SQUEEZE_N = 0.3  # per finger
TIP_DX = 0.005  # fingertips this far forward (+x) of the crisp's middle
TIP_BELOW_EDGE = 0.0020  # lowest finger point this far below the top crisp's edge top corner


def _edge():
    """The top crisp's outer edge (top corner) in its own frame: half-width and height at x = 0."""
    ys, zs = [], []
    for b in _half(1, nubs=False):
        if abs(b.at[0]) > TDX:
            continue
        r = mk.quat_to_mat(b.quat)
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                c = np.asarray(b.at) + r @ (np.array([sx, sy, 0.5]) * b.size)
                ys.append(c[1])
                zs.append(c[2])
    i = int(np.argmax(ys))
    return float(ys[i]), float(zs[i])


EDGE_Y, EDGE_Z = _edge()


def demo_script(ctx):
    import demokit as d

    r = ctx.right
    grip = d.tilted(GRIP_TILT, 0.0)
    a = grip.rot[:, 2]  # approach axis (down and forward)
    w = np.cross(grip.rot[:, 0], a)  # unused: kept for clarity of the frame
    del w

    def crisp():
        st = {n: ctx.state(n) for n in ("chip1_a", "chip1_b")}
        pa, _ = kit.pose(st, "chip1_a")
        pb, _ = kit.pose(st, "chip1_b")
        return (pa + pb) / 2

    # the lowest finger point sits about 2.6 mm below (TCP + 13 mm along the axis), at the back corner of the pad
    def tcp_for(tip_low):
        return tip_low - 0.013 * a + np.array([0.0, 0.0, 0.0026])

    c = crisp()
    tip = c + np.array([TIP_DX, 0.0, EDGE_Z - TIP_BELOW_EDGE])
    grasp = tcp_for(tip)
    above = grasp - 0.04 * a
    open_f = -ENTRY_W / 2
    yield from r.move(above + np.array([0.0, 0.0, 0.01]), grip=grip, finger=open_f, t=2.5, label="over can")
    yield from r.move(above, t=0.8, label="above")
    yield from r.servo(above, tol=0.001, timeout=1.5, label="above")
    if ctx.mode == "none":
        yield from r.wait(1.0)
        return
    yield from r.move(grasp, t=1.6, label="in")
    yield from r.servo(grasp, tol=0.0004, timeout=2.0, label="in")
    if ctx.mode == "squeeze":
        yield from r.close(2 * EDGE_Y, 25.0, t=0.8, label="pinch")
    else:
        # close slowly until the fingers lag their command by SQUEEZE_N / stiffness: they are pressing that hard
        from yam_arena.constants import FINGER_OPEN, GRIPPER
        lag_goal = SQUEEZE_N / GRIPPER["stiffness"]
        f = r.finger
        for _ in range(200):
            q_obs = float(ctx.observed("right")[6]) * FINGER_OPEN
            if f - q_obs >= lag_goal:
                break
            f = min(f + 0.00004, 0.0)
            r.finger = f
            r.cmd = np.r_[r.q, float(np.clip(f / FINGER_OPEN, 0.0, 1.0))]
            yield
        print(f"[demo] pinch: command {f * 1000:.3f} mm, fingers at {q_obs * 1000:.3f} mm", flush=True)
        yield from r.wait(0.3)
    lift = r.at() + np.array([0.0, 0.0, 0.045])
    yield from r.move(lift, t=1.6, label="lift")
    plate = ctx.state("plate")
    goal_tip_low = np.array([plate.pos[0], plate.pos[1], plate.low[2] + 0.006 + 0.004 + DROP])
    over = tcp_for(goal_tip_low) + np.array([0.0, 0.0, 0.06])
    yield from r.move(lift + np.array([-0.02, 0.07, -0.005]), t=1.2, label="clear of can")
    yield from r.move(np.array([over[0], over[1], over[2] + 0.04]), t=1.8, label="across")
    yield from r.move(over, t=1.0, label="over plate")
    yield from r.move(over - np.array([0.0, 0.0, 0.06]), t=1.2, label="down")
    yield from r.move(r.at(), finger=-(2 * EDGE_Y + 0.012) / 2, t=0.6, label="let go")
    yield from r.move(r.at() - 0.06 * a, t=1.0, label="back off")
    yield from r.move(r.at() + np.array([0.0, 0.05, 0.08]), t=1.0, label="away")
    yield from r.wait(2.0)


# -- offline self-test ---------------------------------------------------------------------------------------------

if __name__ == "__main__":
    print("tiles per crisp", len(TILES), "edge y/z", round(EDGE_Y * 1000, 2), round(EDGE_Z * 1000, 2))
    print("objects", [o.name for o in OBJECTS])

# DEBUG-BEGIN (temporary)
import os as _os  # noqa: E402
if _os.environ.get("PRINGLES_DEBUG"):
    def _dbg(objects):
        n = mk._EP["step"]
        if n % 10:
            return
        e = evaluate(objects)
        can = objects["can"]
        f = finger_points()
        msg = [f"[dbg] step {n} score {e['score']} top_whole {e['top_whole']} next_whole {e['next_whole']} seated {e['next_seated']} plate {e['on_plate']} clear {e['clear']} can tilt {_tilt_deg(can.quat_xyzw):.1f} can pos {np.round(can.pos, 4)}"]
        for p in HALVES:
            pos, _ = kit.pose(objects, p)
            now, ref = _rel(objects, p)
            msg.append(f"   {p} pos {np.round(pos, 4)} rel_d {np.linalg.norm(now[:3, 3]-ref[:3, 3])*1000:.2f}mm twist {kit.q_deg(objects, p[:5] + '_snap'):.2f}")
        if len(f):
            fr = f[len(f) // 2:]
            art = mk._EP["env"].unwrapped.scene["right_robot"]; jn = list(art.joint_names)
            qq = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0].detach().cpu().numpy()
            msg.append(f"   fingers q7 {qq[jn.index('joint7')]*1000:.2f} q8 {qq[jn.index('joint8')]*1000:.2f} mm")
            msg.append(f"   right fingers low z {fr[:, 2].min():.4f} y-range {fr[:, 1].min():.4f} {fr[:, 1].max():.4f} tcp {np.round(mk.tcps()[1], 4)}")
        print("\n".join(msg), flush=True)
    kit.on_step(_dbg)
# DEBUG-END
