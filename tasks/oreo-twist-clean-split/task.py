"""Twist an Oreo open into its two wafers and lay both halves on the plate.

A Nabisco Oreo Original sandwich cookie (44 mm across, 11 mm thick, 11.3 g) lies flat on the table beside a plate.
The robot must open it into its two wafers and leave both halves lying on the plate, apart, with its grippers clear.

Mechanism (mechkit.py, rigid bodies joined by PhysX joints; nothing is triggered by proximity):

- The base wafer (the one the creme was deposited on at the factory, "wafer 1" in Owens et al. 2022) carries the creme
  disc: one rigid body. The top wafer is a second rigid body.
- The creme holds the top wafer by a twist bond (mechkit `bond(..., twist_axis=...)`): a revolute joint about the
  cookie's axis on a torsion spring of K_TWIST N m/rad. It lets go for good once the wafers have been turned
  TWIST_RELEASE_DEG against each other, at about 0.1 N m: the creme's brittle, adhesive failure measured by Owens et
  al. (torque to open about 0.1 N m; nearly all creme stays on one wafer). The bond is rigid in pull, push and
  bending (break force unlimited): pulling or prying a held Oreo does not open it here. Once the bond has let go the
  two wafers are separate bodies that only touch.
- Pulling straight off is estimated (squeeze-flow pull-off of a yield-stress layer, 2 pi sigma R^3 / 3H with the
  paper's 17 kPa, R = 17.5 mm, H = 4 mm) at about 48 N, beyond what fingertips on a 3.5 mm wafer rim hold.

Wafer integrity (the one force rule, asked for in review): the wafers are rigid here, so a crushing pinch is scored
instead of simulated. Every control step each of the four fingertips' normal force on each wafer is read: the
gripper's finger-drive force (the force its fingertips push with) while that fingertip's pad touches that wafer. A
force above BREAK_N on two consecutive control steps marks that wafer cracked, which scores 0 for the trial. BREAK_N is an estimate from published biscuit bend data (see
the README); the real trial checks the same thing by photo.

Otherwise scoring reads end-state geometry a person can check by eye and with a ruler: whether the cookie is open,
whether each half rests on the plate's floor, and whether the hands are off them.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  rigid parts, bonds, per-episode memory
from simready import SimReady  # noqa: E402  NVIDIA SimReady props
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Open this Oreo into its two halves and put both halves on the plate."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 1 s hold

# -- the cookie: Nabisco Oreo Original -------------------------------------------------------------------------------
COOKIE_XY = (0.44, 0.0)
R_WAFER = 0.022  # 44 mm across
T_WAFER = 0.0035  # each wafer
R_CREME = 0.0175  # the creme disc stops short of the rim
H_CREME = 0.004  # creme height (11 mm overall)
M_TOP = 0.0039  # g: two wafers about 7.8 g, creme about 3.5 g (11.3 g a cookie)
M_BASE = 0.0039 + 0.0035
TWIST_RELEASE_DEG = 10.0
TORQUE_OPEN = 0.10  # N m (Owens et al. 2022, Sec. III A)
K_TWIST = TORQUE_OPEN / math.radians(TWIST_RELEASE_DEG)  # N m/rad
WAFER = (0.035, 0.022, 0.016)  # near-black chocolate wafer
CREME = (0.93, 0.91, 0.84)


def _disc(r, t, z, color=None):
    """A 16-sided disc about z: eight strips through the axis, turned 22.5 degrees apart."""
    w = 2 * r * math.tan(math.pi / 16)
    return [mk.box((2 * r, w, t), (0.0, 0.0, z), color=color, yaw=k * math.pi / 8) for k in range(8)]


mk.TORSIONAL_PATCH = 0.006  # fingertip-sized contact patch on the wafers' rims
mk.SOLVER_ITERS = 64  # light wafers squeezed between stiff fingers need a stiffer solve
kit = mk.Kit()
Z_TOP_WAFER = T_WAFER + H_CREME  # the top wafer's underside above the table
kit.part("wafer_base", (COOKIE_XY[0], COOKIE_XY[1], TOP),
         _disc(R_WAFER, T_WAFER, T_WAFER / 2) + _disc(R_CREME, H_CREME, T_WAFER + H_CREME / 2, color=CREME),
         mass=M_BASE, color=WAFER)
kit.part("wafer_top", (COOKIE_XY[0], COOKIE_XY[1], TOP), _disc(R_WAFER, T_WAFER, Z_TOP_WAFER + T_WAFER / 2),
         mass=M_TOP, color=WAFER)
kit.bond("creme", "wafer_base", "wafer_top", anchor=(COOKIE_XY[0], COOKIE_XY[1], TOP + Z_TOP_WAFER),
         twist_axis=(0.0, 0.0, 1.0), twist_release_deg=TWIST_RELEASE_DEG, twist_stiffness=K_TWIST,
         slip_mm=3.0, slip_deg=5.0)
HALVES = ("wafer_top", "wafer_base")

PLATE_XY = (0.25, 0.0)
OBJECTS = [
    *kit.objects(),
    SimReady("plate", pos=(PLATE_XY[0], PLATE_XY[1], TOP), size=(0.172, 0.177, 0.026), mass=0.15,
             package="dishware_plastic_bigPlate_a01", usd="sm_dishware_plastic_bigPlate_a01_01.usd"),
    # distractors: scoring ignores them
    SimReady("milk_glass", pos=(0.52, 0.17, TOP), size=(0.066, 0.066, 0.100), mass=0.40,
             package="dishware_glass_8oz_a01", usd="sm_dishware_glass_8oz_a01_01.usd"),
    SimReady("spoon", pos=(0.53, -0.15, TOP), size=(0.157, 0.035, 0.011), mass=0.02,
             package="dishware_plastic_spoon_a01", usd="sm_dishware_plastic_spoon_a01_01.usd", yaw=1.2),
    SimReady("mug", pos=(0.12, 0.25, TOP), size=(0.114, 0.090, 0.080), mass=0.30,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
]

# -- reading the scene -----------------------------------------------------------------------------------------------
PLATE_R_OK = 0.064  # a half's centre within 64 mm of the plate's centre: the whole wafer inside the 172 mm plate
PLATE_FLOOR = 0.0016  # the plate's floor surface above its lowest point: a wafer resting on the floor in the sim
REST_MAX = 0.004  # resting: the half's lowest point at most 4 mm above the plate's floor surface ...
REST_MIN = -0.001  # ... and not below it (1 mm allows for contact penetration)
PLATE_UPRIGHT_DEG = 10.0  # the plate itself upright: an inverted plate has no floor to rest on
BREAK_N = 15.0  # N: a fingertip's normal force on one wafer that cracks it (estimate; README "Wafer breaking force")
BREAK_STEPS = 2  # control steps in a row above BREAK_N
FLAT_DEG = 20.0  # lying flat (either face up)
APART_M = 0.045  # centres at least 45 mm apart: the halves do not overlap (44 mm wafers)
CLEAR_M = 0.03  # every finger point at least 3 cm from either half
STILL_V = 0.02
HOLD_STEPS = 20
FELL_M = 0.03  # a part more than 3 cm below the table top has fallen off the table


def finger_points():
    """World points on the lower 4 cm of all four finger pads (sim only; empty offline)."""
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
    """Angle between the part's axis and vertical, folded so that upside down counts as flat (0..90)."""
    c = abs(float(mk.quat_to_mat(quat_xyzw)[2, 2]))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _centre(objects, part):
    z = T_WAFER / 2 if part == "wafer_top" else (T_WAFER + H_CREME) / 2
    z += Z_TOP_WAFER if part == "wafer_top" else 0.0
    return kit.point(objects, part, (0.0, 0.0, z))


def _rim_points(objects, part):
    """Points round the wafer's rim and on its axis, in world."""
    zc = Z_TOP_WAFER + T_WAFER / 2 if part == "wafer_top" else T_WAFER / 2
    pts = [(0.0, 0.0, zc)]
    for k in range(8):
        a = k * math.pi / 4
        pts.append((R_WAFER * math.cos(a), R_WAFER * math.sin(a), zc))
    return np.array([kit.point(objects, part, p) for p in pts])


def _underside_z(objects, part) -> float:
    """The lowest point of the wafer's underside rim, in world z (works either face up)."""
    zc = Z_TOP_WAFER + T_WAFER / 2 if part == "wafer_top" else T_WAFER / 2
    zs = []
    for dz in (-T_WAFER / 2, T_WAFER / 2):
        for k in range(16):
            a = k * math.pi / 8
            zs.append(kit.point(objects, part, (R_WAFER * math.cos(a), R_WAFER * math.sin(a), zc + dz))[2])
    return float(min(zs))


def _plate_tilt_deg(objects) -> float:
    c = float(mk.quat_to_mat(objects["plate"].quat_xyzw)[2, 2])  # not folded: upside down is 180
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _rest_gap(objects, part) -> float:
    """The half's lowest point above the plate's floor surface (m), for an upright plate."""
    return _underside_z(objects, part) - float(objects["plate"].low[2] + PLATE_FLOOR)


def _on_plate(objects, part) -> bool:
    """Resting on the plate: the plate upright, the half's centre inside its floor, and the half's lowest point within
    REST_MIN..REST_MAX of the floor surface, so it is held up by the floor and not by something else or by nothing."""
    c, plate = _centre(objects, part), objects["plate"]
    d = math.hypot(c[0] - plate.pos[0], c[1] - plate.pos[1])
    upright = _plate_tilt_deg(objects) <= PLATE_UPRIGHT_DEG
    gap = _rest_gap(objects, part)
    return bool(upright and d <= PLATE_R_OK and REST_MIN <= gap <= REST_MAX)


# -- wafer integrity: fingertip normal force on each wafer -----------------------------------------------------------
TOUCH_M = 0.004  # a fingertip pad point this close to a wafer's rim or faces is pressing on that wafer


def _finger_drive_n():
    """(2,) the larger finger-drive force of each gripper, N (left, right): the force each fingertip pushes with.
    In a pinch both fingertips carry this force as normal force on what is between them. None offline."""
    env = mk._EP.get("env")
    if env is None:
        return None
    base = env.unwrapped
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        jn = list(art.joint_names)
        eff = getattr(art.data.applied_torque, "torch", art.data.applied_torque)[0].detach().cpu().numpy()
        out.append(max(abs(float(eff[jn.index("joint7")])), abs(float(eff[jn.index("joint8")]))))
    return np.array(out)


def _touching(objects, pts, part) -> bool:
    """Any of these world points within TOUCH_M of the wafer's solid (a disc R_WAFER x T_WAFER)."""
    pos, q = kit.pose(objects, part)
    r = mk.quat_to_mat(q)
    loc = (np.asarray(pts) - pos) @ r  # into the part's frame
    z0 = Z_TOP_WAFER if part == "wafer_top" else 0.0
    rad = np.hypot(loc[:, 0], loc[:, 1])
    dr = np.maximum(rad - R_WAFER, 0.0)
    dz = np.maximum(np.maximum(z0 - loc[:, 2], loc[:, 2] - (z0 + T_WAFER)), 0.0)
    return bool(np.any(np.hypot(dr, dz) <= TOUCH_M))


def tip_forces(objects):
    """(4, 2) each fingertip's normal force on each wafer, N: rows left-arm tip +c, -c, right-arm tip +c, -c; columns
    (wafer_top, wafer_base). A fingertip's force is its gripper's finger-drive force while one of its pad points
    touches that wafer, else 0. None offline."""
    drive = _finger_drive_n()
    if drive is None:
        return None
    pts = finger_points().reshape(2, 2, -1, 3)  # arm, fingertip side, pad points
    out = np.zeros((4, 2))
    for arm in range(2):
        for side in range(2):
            for j, part in enumerate(HALVES):
                if _touching(objects, pts[arm, side], part):
                    out[2 * arm + side, j] = drive[arm]
    return out


def _hands_clear(objects) -> bool:
    f = finger_points()
    if len(f) == 0:
        return bool(mk.MEMORY.get("offline_clear", True))
    for part in HALVES:
        p = _rim_points(objects, part)
        if np.min(np.linalg.norm(f[:, None, :] - p[None, :, :], axis=2)) < CLEAR_M:
            return False
    return True


def _twist_deg(objects) -> float:
    return abs(kit.q_deg(objects, "creme"))


def evaluate(objects) -> dict:
    """This step's readings and level (no hold, no memory writes)."""
    m = mk.MEMORY
    broken = kit.broken(objects, "creme")
    opened = bool(broken and m.get("max_twist", 0.0) >= TWIST_RELEASE_DEG)
    on_plate = [_on_plate(objects, p) for p in HALVES]
    flat = [_tilt_deg(objects[p].quat_xyzw) <= FLAT_DEG for p in HALVES]
    ca, cb = _centre(objects, "wafer_top"), _centre(objects, "wafer_base")
    apart = math.hypot(ca[0] - cb[0], ca[1] - cb[1]) >= APART_M
    clear = _hands_clear(objects)
    still = all(float(np.linalg.norm(objects[p].lin_vel)) < STILL_V for p in HALVES)
    harm = bool(m.get("fell", False) or m.get("torn", False) or m.get("cracked", False))
    if harm or not opened:
        lv = 0.0
    elif all(on_plate) and all(flat) and apart and clear:
        lv = 1.0
    elif all(on_plate) and clear:
        lv = 0.75
    else:
        lv = 0.5
    return dict(level=lv, opened=opened, on_plate=on_plate, flat=flat, apart=apart, clear=clear, still=still,
                harm=harm, cracked=m.get("cracked", False), twist=m.get("max_twist", 0.0))


def _rules(objects) -> None:
    """Every control step. MEMORY is cleared only by mechkit's reset hook: the only place a recorded harm is cleared."""
    m = mk.MEMORY
    if not m.get("released_seen"):
        m["max_twist"] = max(m.get("max_twist", 0.0), _twist_deg(objects))
    if kit.broken(objects, "creme") and not m.get("released_seen"):
        m["released_seen"] = True
        if m.get("max_twist", 0.0) < TWIST_RELEASE_DEG:
            m["torn"] = True  # the bond gave way without the twist (a solver blow-up under a hard pull): no credit
    for p in HALVES:
        if _centre(objects, p)[2] < TOP - FELL_M:
            m["fell"] = True
    f = tip_forces(objects)
    if f is not None:
        over = m.setdefault("over", np.zeros((4, 2), int))
        over[:] = np.where(f > BREAK_N, over + 1, 0)
        peak = m.setdefault("peak_n", np.zeros((4, 2)))
        np.maximum(peak, f, out=peak)
        m.setdefault("force_log", []).append(np.round(f, 3).tolist())
        for j, part in enumerate(HALVES):
            if np.any(over[:, j] >= BREAK_STEPS) and not m.get("cracked"):
                m["cracked"] = part  # recorded for the episode; cleared only by mechkit's reset hook
        if len(m["force_log"]) % 20 == 0:
            _dump_forces()
    e = evaluate(objects)
    m["hold"] = m.get("hold", 0) + 1 if (e["level"] == 1.0 and e["still"]) else 0
    if m["hold"] >= HOLD_STEPS:
        m["full"] = True
    if m.get("full") and e["level"] < 1.0:  # credit does not outlast the state that earned it
        m["full"] = False
    m["last"] = e


kit.on_step(_rules)


def score(objects) -> float:
    e = mk.MEMORY.get("last") or evaluate(objects)
    if mk.MEMORY.get("full"):
        return 1.0
    return float(min(e["level"], 0.75))


def success(objects) -> bool:
    return bool(mk.MEMORY.get("full", False))


# -- scripted demo (closed loop; run with demokit.py) ----------------------------------------------------------------
TILT_L = 50.0  # the left hand comes in from +y, tilted 40 degrees below level so its knuckles clear the table
TIP_L = (-0.001, 0.0110)  # its fingertips: y past the cookie's axis, z above the cookie's underside
TIP_R = (-0.002, -0.0035)  # the right hand's fingertips (it comes in level from the robot's side, along +x): x past the axis, z below the base wafer's underside
LIFT_Z = 0.14
TWIST_DEG = -15.0
PHI_R = 0.0  # the right hand comes in level from the robot side, pointing straight ahead (+x)
GRIP_N = 6.0  # newtons a finger, well under BREAK_N
CRUSH_N = 40.0  # red-team "crush": the YAM's full grip, about 40 N a finger
PLACE_TILT_R = 70.0


def _report(ctx, label=""):
    e = mk.MEMORY.get("last")
    if not e:
        print(f"[oreo] t {ctx.t:5.1f} {label}", flush=True)
        return
    ca, cb = ctx.pos("wafer_top"), ctx.pos("wafer_base")
    m = mk.MEMORY
    objs = {n: ctx.state(n) for n in (*HALVES, "plate")}
    gaps = [round(1000 * (_underside_z(objs, p) - float(objs["plate"].low[2])), 2) for p in HALVES]
    peak = np.round(m.get("peak_n", np.zeros((4, 2))), 2).tolist()
    print(f"[oreo] t {ctx.t:5.1f} {label} level {e['level']} opened {e['opened']} twist {e['twist']:.1f} on_plate "
          f"{e['on_plate']} flat {e['flat']} apart {e['apart']} clear {e['clear']} still {e['still']} harm {e['harm']} "
          f"cracked {e['cracked']} | underside above plate low (mm, top/base) {gaps} plate tilt "
          f"{_plate_tilt_deg(objs):.1f} | peak tip N (tips x top/base) {peak} | "
          f"top {np.round(ca, 4).tolist()} base {np.round(cb, 4).tolist()} full {m.get('full', False)}", flush=True)


def demo_script(ctx):
    """Left: comes in low from the +y side, tilted 40 degrees down, and pinches the top wafer across its rim, then lifts
    the cookie 14 cm (it hangs from the top wafer). Right: comes in level from the robot's side and pinches the base
    wafer's rim below the top wafer. The left holds still while the right turns the base wafer 15 degrees about the
    cookie's axis; the creme lets go; the left lifts the top wafer off. The right lays the base half on the plate and
    backs away, then the left lays the top half beside it and backs away.
    Red-team modes: crush (as full, but both hands pinch at CRUSH_N), none (nothing), onearm (the left alone: lifts the cookie and twists it in one hand), pull (both
    hands hold, then pull straight apart without turning), careless (the right turns 15 degrees, then both let go over
    the plate without lifting the wafers apart)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "crush", "none", "onearm", "pull", "careless"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for _ in range(100):
            yield
        _report(ctx, "end:")
        return

    gl, gr = d.tilted(TILT_L, -90.0), d.level(PHI_R)
    al, ar = gl.rot[:, 2], gr.rot[:, 2]  # approach axes

    def base_xy():
        return ctx.pos("wafer_base")[:2]

    def cookie_bottom():
        st = ctx.state("wafer_base")
        return float(st.low[2])

    # left: pinch the top wafer's rim where the cookie lies
    c = np.r_[base_xy(), cookie_bottom()]
    tip_l = c + np.array([0.0, TIP_L[0], TIP_L[1]])
    tcp_l = tip_l - 0.013 * al
    back = -0.06 * al
    yield from ctx.together(l.move(tcp_l + back + np.array([0, 0, 0.03]), grip=gl, finger=FINGER_OPEN, t=2.5, label="left beside the cookie"),
                            r.move(np.array([0.32, -0.04, TOP + 0.15]), grip=gr, finger=FINGER_OPEN, t=2.5, label="right ready"))
    yield from l.move(tcp_l + back, t=0.8, label="left down")
    yield from l.move(tcp_l, t=1.2, label="left in round the top wafer")
    _report(ctx, "left in:")
    yield from l.servo(tcp_l, timeout=1.0, label="left in")
    grip_n = CRUSH_N if mode == "crush" else GRIP_N
    yield from l.close(2 * R_WAFER, grip_n, t=0.6, label="left pinch the top wafer")
    _report(ctx, "left holds:")
    yield from l.move(l.at() + np.array([0.0, 0.0, LIFT_Z]), t=2.0, label="lift the cookie")
    yield from l.wait(0.5)
    _report(ctx, "lifted:")

    if mode == "onearm":
        yield from l.turn(30.0, axis=(0.0, 0.0, 1.0), t=1.5, label="one hand twists")
        yield from l.turn(-30.0, axis=(0.0, 0.0, 1.0), t=1.5, label="and back")
        yield from l.move(np.r_[PLATE_XY[0], PLATE_XY[1] + 0.03, TOP + 0.05] - 0.013 * al + np.array([0, 0.02, 0]), t=2.5, label="over the plate")
        yield from l.open(t=0.5, label="let go")
        yield from l.move(l.at() + np.array([0.0, 0.10, 0.08]), t=1.5, label="left away")
        for k in range(60):
            yield
        _report(ctx, "end:")
        return

    # right: pinch the base wafer's rim, from below the top wafer
    c = np.r_[base_xy(), cookie_bottom()]
    tip_r = c - TIP_R[0] * ar + np.array([0.0, 0.0, TIP_R[1]])
    tcp_r = tip_r - 0.013 * ar
    yield from l.wait(1.0)
    yield from r.move(tcp_r - 0.035 * ar, t=2.0, label="right level beside the cookie")
    _report(ctx, "right beside:")
    c = np.r_[base_xy(), cookie_bottom()]
    tcp_r = c - TIP_R[0] * ar + np.array([0.0, 0.0, TIP_R[1]]) - 0.013 * ar
    for k in range(6):  # in slowly, re-reading where the hanging cookie is
        c = np.r_[base_xy(), cookie_bottom()]
        tcp_r = c - TIP_R[0] * ar + np.array([0.0, 0.0, TIP_R[1]]) - 0.013 * ar
        yield from r.move(r.at() + (tcp_r - r.at()) / (6 - k), t=0.4, label="right in")
    yield from r.move(tcp_r, t=1.2, label="right in round the base wafer")
    _report(ctx, "right in:")
    yield from r.servo(tcp_r, timeout=1.0, label="right in")
    yield from r.close(2 * R_WAFER, grip_n, t=0.6, label="right pinch the base wafer")
    _report(ctx, "both hold:")

    if mode == "pull":
        yield from l.move(l.at() + np.array([0.0, 0.0, 0.03]), t=1.5, label="pull straight apart")
        yield from l.wait(0.5)
        _report(ctx, "pulled:")
        yield from l.move(l.at() + np.array([0.0, 0.0, -0.03]), t=1.0, label="back")
    else:
        axis_pt = np.r_[base_xy(), 0.0]
        yield from r.arc(axis_pt, (0.0, 0.0, 1.0), TWIST_DEG, t=2.0, label="right turns the base wafer")
        yield from r.wait(0.4)
        _report(ctx, "twisted:")
        if mode != "careless":
            yield from l.move(l.at() + np.array([0.0, 0.0, 0.025]), t=1.0, label="left lifts the top wafer off")
            _report(ctx, "separated:")

    # each hand lays its half on the plate: the right (nearer the plate) first, then the left
    drop = 0.022  # the half's underside this far above the plate when let go
    tgt_l = np.array([PLATE_XY[0], PLATE_XY[1] + 0.045])
    tgt_r = np.array([PLATE_XY[0], PLATE_XY[1] - 0.045])
    if mode == "careless":
        tgt_l = tgt_r = np.array(PLATE_XY)
    if mode != "careless":
        yield from l.move(l.at() + np.array([0.0, 0.04, 0.04]), t=1.2, label="left holds the top wafer clear")
    gr_place = d.tilted(PLACE_TILT_R, PHI_R + TWIST_DEG)
    orr = r.at() - np.r_[ctx.pos("wafer_base")[:2], 0.0]
    yield from r.move(np.r_[tgt_r, r.at()[2]] + np.r_[orr[:2], 0.0], grip=gr_place, t=2.0, label="right over the plate")
    zb = float(ctx.state("wafer_base").low[2])
    yield from r.move(r.at() + np.array([0.0, 0.0, (TOP + 0.006 + drop) - zb]), t=1.5, label="right down to the plate")
    if mode == "careless":  # both let go together, the wafers still stacked
        yield from ctx.together(l.open(t=0.5), r.open(t=0.5))
        yield from l.move(l.at() + np.array([0.0, 0.07, 0.05]), t=1.2, label="left backs off")
    else:
        yield from r.open(t=0.5, label="right lets go")
    yield from r.move(r.at() - 0.07 * r.approach_axis() + np.array([0.0, 0.0, 0.04]), t=1.2, label="right backs off")
    yield from r.move(np.array([0.25, -0.30, TOP + 0.20]), grip=d.down(), t=1.5, label="right away")
    _report(ctx, "base half down:")
    if mode != "careless":
        ol = l.at() - np.r_[ctx.pos("wafer_top")[:2], 0.0]
        yield from l.move(np.r_[tgt_l, l.at()[2]] + np.r_[ol[:2], 0.0], t=2.0, label="left over the plate")
        zt = float(ctx.state("wafer_top").low[2])
        yield from l.move(l.at() + np.array([0.0, 0.0, (TOP + 0.006 + drop) - zt]), t=1.5, label="left down to the plate")
        yield from l.open(t=0.5, label="left lets go")
        yield from l.move(l.at() - 0.07 * al + np.array([0.0, 0.0, 0.04]), t=1.2, label="left backs off")
    yield from l.move(np.array([0.25, 0.30, TOP + 0.20]), grip=d.down(), t=1.5, label="left away")
    _report(ctx, "top half down:")
    for k in range(60):
        yield
        if k % 20 == 19:
            _report(ctx, "end:")


def _dump_forces():
    """Write the per-step fingertip force log (N; steps x 4 tips x top/base) where OREO_FORCE_LOG says, if set."""
    import json
    import os

    out = os.environ.get("OREO_FORCE_LOG")
    if out and mk.MEMORY.get("force_log"):
        with open(out, "w") as fh:
            json.dump(dict(break_n=BREAK_N, break_steps=BREAK_STEPS, cols=list(HALVES),
                           forces=mk.MEMORY["force_log"]), fh)
