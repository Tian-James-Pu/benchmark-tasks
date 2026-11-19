"""A dish pit's pre-rinse unit, left locked on, over its sink.

On the station table stands the near end of a stainless dish table, its deck 12.5 cm up, with a pre-rinse sink
basin 38 cm front to back, 46 cm wide and 12.5 cm deep let into it. A wall-mounted pre-rinse faucet on the
backsplash feeds a riser; from the overhead spring at its top a 50 cm hose hangs down over the middle of the
basin to a chrome pre-rinse spray valve (16 cm long, 0.45 kg), which hangs nozzle down with its spray face 10 cm
above the deck. The valve's squeeze handle (left side) is held squeezed by its steel lock ring, hooked round the
handle's tip and the body just above the spray head, so the valve is spraying into the basin. The head was let go a
moment ago and swings gently on its hose.

To shut it off, the handle has to be squeezed a little further (which takes the load off the ring), the ring pushed
down off the handle's tip, and the handle let go so that it springs open. The riser, spring, faucet and its
handles are fixed and out of reach.
"""

import json
import math
import os
import pathlib
import sys
from dataclasses import dataclass

import numpy as np

from yam_arena import demo as _demo
from yam_arena import ik as _ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "The pre-rinse sprayer has been left on. Turn it off and leave it hanging over the sink."
SUCCESS_HOLD_S = 0.0  # the per-step tracker enforces the full three-second hold

# -- the dish table and its pre-rinse sink (station frame, metres; heights above the table top) -------------------
DECK = 0.125  # deck top
TBL_X0, TBL_X1, TBL_Y = 0.08, 0.64, 0.42  # the dish table's top, front to back and half its width
BASIN_X0, BASIN_X1, BASIN_Y = 0.14, 0.52, 0.23  # the basin's opening (inside walls)
SPLASH_T, SPLASH_H = 0.015, 0.25  # backsplash thickness and height above the deck

# -- the pre-rinse unit ---------------------------------------------------------------------------------------------
X_V, Y_V = 0.33, 0.0  # under the spring's end: where the head hangs
Z_F = DECK + 0.100  # spray face at rest, above the table top
VALVE_LEN = 0.160  # spray face to the top of the hose swivel
HOSE_LEN = 0.50  # free hose, from the spring's end to the swivel
Z_A = Z_F + VALVE_LEN + HOSE_LEN  # the spring's end (hose anchor)
Z_M = Z_F + VALVE_LEN + HOSE_LEN / 2  # where the hose is modelled as bending
HOSE_R = 0.0095  # 3/4 in hose

R_BODY = 0.013  # valve body radius
R_HEAD = 0.017  # spray head radius
Z_HEAD = 0.022  # top of the spray head (shoulder), above the spray face
Z_BODY = 0.138  # top of the body
L_H, W_H, T_H = 0.100, 0.016, 0.008  # handle: pivot to tip, width, thickness
PIVOT = (0.0, R_BODY + T_H / 2, 0.1337)  # handle pivot, valve frame (y left, z up from the spray face)
GAP_OFF, GAP_LOCK, GAP_MAX = 0.020, 0.008, 0.0035  # handle tip's gap to the body: let go, locked on, fully squeezed
ALPHA_LOCK = math.asin(GAP_LOCK / L_H)
H_OFF = math.asin(GAP_OFF / L_H) - ALPHA_LOCK  # handle joint (rad, 0 = locked on): at its off stop
H_MAX = math.asin(GAP_MAX / L_H) - ALPHA_LOCK  # squeezed to its stop (negative)
H_SQUEEZED = math.radians(-1.2)  # squeezed past the lock point this far: the ring is unloaded
H_OPEN = H_OFF - math.radians(2.5)  # the valve passes water at or below this
H_SHUT = H_OFF - math.radians(1.0)  # back on its off stop
H_PRELOAD = math.radians(5.0)  # the handle spring's preload against the off stop
K_HANDLE = 2.16  # N m/rad: 0.45 N m at the lock point (about 12 N at the middle of the handle)

Z_RING = 0.040  # ring's plane when locked on (in the groove just above the handle's tip end)
TIP_END = Z_RING - 0.006  # the handle tip's lower end
RING_WIRE = 0.0015
RING_HX = 0.0145 + RING_WIRE  # wire centreline half-extent across the body (x)
RING_Y0, RING_Y1 = -R_BODY - 0.0005 - RING_WIRE, 0.0290 + RING_WIRE  # wire centreline across body and handle tip (y)
RING_YC = (RING_Y0 + RING_Y1) / 2
R_LO = (Z_HEAD + RING_WIRE) - Z_RING  # ring down on the spray head's shoulder
R_OFF = (TIP_END - RING_WIRE) - Z_RING - 0.0005  # ring's top below the tip's end: off the tip
TAB = (0.014, 0.019, 0.0025)  # thumb tab on the ring's left side
TAB_Y = RING_Y1 + RING_WIRE + TAB[1] / 2 - 0.001
RING_FRICTION = 1.0  # N: the ring's detent, beats its own weight (0.15 N)

V_MASS, H_MASS, RING_MASS = 0.40, 0.04, 0.015
HOSE_MASS = 0.28 * HOSE_LEN / 2  # each half of the hose
D_SWING, D_BEND, K_BEND = 0.12, 0.05, 0.3  # hose damping at the anchor and the bend (N m s/rad), bending spring (N m/rad)
SWING0 = 0.20  # rad/s: the head's swing at the start, about 3 cm each way

LOOKS = {
    "steel": m.Look(color=m.rgb(0.70, 0.71, 0.73), metallic=0.85, roughness=0.32),
    "steel_dark": m.Look(color=m.rgb(0.52, 0.53, 0.55), metallic=0.85, roughness=0.4),
    "chrome": m.Look(color=m.rgb(0.86, 0.87, 0.89), metallic=1.0, roughness=0.12),
    "hose": m.Look(color=m.rgb(0.07, 0.07, 0.08), roughness=0.7),
    "rubber": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.85),
    "grip": m.Look(color=m.rgb(0.30, 0.42, 0.62), roughness=0.55),
    "drain": m.Look(color=m.rgb(0.25, 0.25, 0.27), metallic=0.6, roughness=0.5),
    "water": m.Look(color=m.rgb(0.70, 0.86, 1.0), roughness=0.05, opacity=0.45),
    "red": m.Look(color=m.rgb(0.75, 0.10, 0.10), roughness=0.4),
    "blue": m.Look(color=m.rgb(0.10, 0.25, 0.75), roughness=0.4),
}


# -- geometry helpers -----------------------------------------------------------------------------------------------
def _bezier(p0, p1, p2, p3, n):
    t = np.linspace(0.0, 1.0, n)[:, None]
    p0, p1, p2, p3 = (np.asarray(p, float) for p in (p0, p1, p2, p3))
    return (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3


def _helix(curve, radius, pitch, per_turn=14):
    """A coil spring's wire centreline round a curve (parallel-transport frame)."""
    seg = np.linalg.norm(np.diff(curve, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    n = max(int(s[-1] / pitch * per_turn), 8)
    ss = np.linspace(0.0, s[-1], n)
    pts = np.array([np.interp(ss, s, curve[:, k]) for k in range(3)]).T
    d = np.gradient(pts, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    u = np.cross(d[0], [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    out = []
    for k in range(n):
        u = u - u.dot(d[k]) * d[k]
        u /= np.linalg.norm(u)
        w = np.cross(d[k], u)
        a = 2 * math.pi * ss[k] / pitch
        out.append(pts[k] + radius * (math.cos(a) * u + math.sin(a) * w))
    return np.array(out)


def _rounded_rect(hx, y0, y1, r, z, n=6):
    """A closed rounded-rectangle path in a horizontal plane: x within +-hx, y from y0 to y1, corner radius r."""
    pts = []
    corners = [(hx - r, y1 - r, 0.0), (-hx + r, y1 - r, 90.0), (-hx + r, y0 + r, 180.0), (hx - r, y0 + r, 270.0)]
    for cx, cy, a0 in corners:
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
    pts.append(pts[0])
    pts.append(pts[1])
    return np.array(pts)


def _v(x, y, z):
    """A point of the valve (spray face centre at the origin, z up its axis) in the unit's frame at rest."""
    return (x, y, Z_F + z)


# -- the dish table and sink (fixed) --------------------------------------------------------------------------------
def _dish_table() -> m.Part:
    parts, cols = [], []

    def slab(name, x0, x1, y0, y1, z0, z1, look="steel", collide=True):
        size, centre = (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
        parts.append((name, m.box(size, centre), look))
        if collide:
            cols.append(m.col_box(f"{name}_c", size, centre))

    slab("front", TBL_X0, BASIN_X0, -TBL_Y, TBL_Y, 0.0, DECK)
    slab("back", BASIN_X1, TBL_X1, -TBL_Y, TBL_Y, 0.0, DECK)
    slab("left", BASIN_X0, BASIN_X1, BASIN_Y, TBL_Y, 0.0, DECK)
    slab("right", BASIN_X0, BASIN_X1, -TBL_Y, -BASIN_Y, 0.0, DECK)
    slab("floor", BASIN_X0, BASIN_X1, -BASIN_Y, BASIN_Y, 0.0, 0.003, "steel_dark")
    slab("splash", TBL_X1 - SPLASH_T, TBL_X1, -TBL_Y, TBL_Y, DECK, DECK + SPLASH_H)
    slab("lip", TBL_X0 - 0.012, TBL_X0, -TBL_Y, TBL_Y, DECK - 0.03, DECK + 0.006, collide=False)  # rolled front edge
    drain = m.cylinder(0.045, 0.002, ((BASIN_X0 + BASIN_X1) / 2, 0.0, 0.004))
    parts.append(("drain", drain, "drain"))
    for k in range(6):  # strainer slots
        a = math.pi * k / 6
        slot = m.box((0.07, 0.004, 0.001), ((BASIN_X0 + BASIN_X1) / 2, 0.0, 0.0055)).turned(math.degrees(a), "z")
        slot = m.Mesh(slot.v + np.array([(BASIN_X0 + BASIN_X1) / 2 * (1 - math.cos(a)), -(BASIN_X0 + BASIN_X1) / 2 * math.sin(a), 0.0]), slot.f)
        parts.append((f"slot{k}", slot, "rubber"))
    return m.fixed("dish_table", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.6, 0.5))


# -- the pre-rinse unit (fixed riser and spring; hose, valve, handle and ring on joints) -----------------------------
def _frame_parts():
    xr = 0.60 - X_V  # riser axis, in the unit's frame
    zf = DECK + 0.20  # faucet body height
    z_top = Z_A + 0.06
    parts = [
        ("faucet", m.cylinder(0.021, 0.24, (xr + 0.012, 0.0, zf), axis="y"), "chrome"),
        ("riser", m.cylinder(0.0125, z_top - zf, (xr, 0.0, (zf + z_top) / 2)), "chrome"),
        ("elbow", m.sphere(0.016, (xr, 0.0, z_top)), "chrome"),
        ("wall_l", m.cylinder(0.012, 0.03, (xr + 0.03, 0.12, zf), axis="x"), "chrome"),
        ("wall_r", m.cylinder(0.012, 0.03, (xr + 0.03, -0.12, zf), axis="x"), "chrome"),
    ]
    for k, sy in enumerate((1, -1)):  # wrist-blade handles, hot left, cold right
        parts.append((f"hub{k}", m.cylinder(0.014, 0.03, (xr - 0.012, sy * 0.08, zf + 0.025)), "chrome"))
        parts.append((f"blade{k}", m.box((0.012, 0.012, 0.075), (xr - 0.012, sy * 0.08, zf + 0.075)), "chrome"))
        parts.append((f"dot{k}", m.cylinder(0.006, 0.002, (xr - 0.012, sy * 0.08, zf + 0.0415)), "red" if sy > 0 else "blue"))
    curve = _bezier((xr, 0.0, z_top), (xr, 0.0, z_top + 0.12), (0.0, 0.0, Z_A + 0.20), (0.0, 0.0, Z_A), 60)
    parts.append(("spring", m.tube(_helix(curve, 0.020, 0.011), 0.0016, sides=6), "chrome"))
    parts.append(("hose_top", m.tube(curve, HOSE_R, sides=12), "hose"))
    parts.append(("spring_end", m.cylinder(0.022, 0.012, (0.0, 0.0, Z_A + 0.006)), "chrome"))
    return parts


def _valve_parts():
    def at(mesh):
        return mesh.moved((0.0, 0.0, Z_F))

    bumper = at(m.lathe([(0.0, 0.0), (R_HEAD + 0.0005, 0.0), (R_HEAD + 0.0005, 0.006), (0.0, 0.006)]))
    face = at(m.cylinder(0.011, 0.001, (0.0, 0.0, -0.0004)))
    head = at(m.lathe([(0.0, 0.006), (R_HEAD, 0.006), (R_HEAD, 0.017), (R_BODY, Z_HEAD), (0.0, Z_HEAD)]))
    body = at(m.cylinder(R_BODY, Z_BODY - Z_HEAD, (0.0, 0.0, (Z_BODY + Z_HEAD) / 2)))
    lug = at(m.box((0.012, 0.007, 0.014), (0.0, R_BODY + 0.002, PIVOT[2])))
    swivel = at(m.cylinder(0.0115, VALVE_LEN - Z_BODY, (0.0, 0.0, (VALVE_LEN + Z_BODY) / 2), segments=6))
    hose = m.cylinder(HOSE_R, Z_M - (Z_F + VALVE_LEN), (0.0, 0.0, (Z_M + Z_F + VALVE_LEN) / 2), segments=16)
    jet = at(m.lathe([(0.0, -0.205), (0.040, -0.205), (0.010, -0.002), (0.0, -0.002)], segments=24))
    parts = [("bumper", bumper, "rubber"), ("face", face, "drain"), ("head", head, "chrome"), ("body", body, "chrome"),
             ("lug", lug, "chrome"), ("swivel", swivel, "chrome"), ("hose_low", hose, "hose"), ("jet", jet, "water")]
    cols = [
        m.col_cylinder("head_c", R_HEAD, Z_HEAD, _v(0.0, 0.0, Z_HEAD / 2)),
        m.col_cylinder("body_c", R_BODY, Z_BODY - Z_HEAD, _v(0.0, 0.0, (Z_BODY + Z_HEAD) / 2)),
        m.col_box("lug_c", (0.012, 0.007, 0.014), _v(0.0, R_BODY + 0.002, PIVOT[2])),
        m.col_cylinder("swivel_c", 0.0115, VALVE_LEN - Z_BODY, _v(0.0, 0.0, (VALVE_LEN + Z_BODY) / 2)),
        m.col_cylinder("hose_low_c", HOSE_R, Z_M - (Z_F + VALVE_LEN), (0.0, 0.0, (Z_M + Z_F + VALVE_LEN) / 2)),
    ]
    return parts, cols


def _handle_link() -> m.Link:
    bar = m.box((W_H, T_H, L_H), (0.0, 0.0, -L_H / 2)).turned(math.degrees(ALPHA_LOCK), "x").moved(_v(*PIVOT))
    knuckle = m.cylinder(0.0055, W_H, (0.0, 0.0, 0.0), axis="x").moved(_v(*PIVOT))
    return m.hinge("handle", "X", _v(*PIVOT), math.degrees(H_MAX), math.degrees(H_OFF), [("grip", bar, "grip"), ("knuckle", knuckle, "chrome")],
                   [m.col_hull("handle_c", bar)], mass=H_MASS, spring=K_HANDLE, damping=0.004, friction=0.0, effort=5.0, gravity=True,
                   com=_v(0.0, PIVOT[1] + 0.5 * L_H * math.sin(ALPHA_LOCK), PIVOT[2] - 0.5 * L_H), inertia=(3.5e-5, 3.5e-5, 2e-6), parent="valve")


def _ring_link() -> m.Link:
    path = _rounded_rect(RING_HX, RING_Y0, RING_Y1, 0.010, Z_F + Z_RING)
    loop = m.tube(path, RING_WIRE, sides=8)
    tab = m.box(TAB, _v(0.0, TAB_Y, Z_RING + 0.0005))
    return m.slider("ring", "Z", R_LO, 0.0, [("loop", loop, "chrome"), ("tab", tab, "chrome")],
                    [m.col_hull("ring_c", loop), m.col_box("tab_c", TAB, _v(0.0, TAB_Y, Z_RING + 0.0005))],
                    at=_v(0.0, RING_YC, Z_RING), mass=RING_MASS, damping=0.3, friction=RING_FRICTION, effort=20.0, gravity=True,
                    com=_v(0.0, RING_YC + 0.006, Z_RING), inertia=(6e-6, 6e-6, 1e-5), parent="valve")


def _sprayer() -> m.Part:
    hose_up = m.cylinder(HOSE_R, Z_A - Z_M, (0.0, 0.0, (Z_A + Z_M) / 2), segments=16)
    vparts, vcols = _valve_parts()
    v_com_z = (V_MASS * (Z_F + 0.075) + HOSE_MASS * (Z_M + Z_F + VALVE_LEN) / 2) / (V_MASS + HOSE_MASS)
    links = [
        m.hinge("swing_x", "X", (0.0, 0.0, Z_A), -70.0, 70.0, [], [], mass=0.01, damping=D_SWING, friction=0.0, gravity=True,
                com=(0.0, 0.0, Z_A), inertia=(1e-5, 1e-5, 1e-5)),
        m.hinge("hose_up", "Y", (0.0, 0.0, Z_A), -70.0, 70.0, [("hose", hose_up, "hose")], [m.col_cylinder("hose_up_c", HOSE_R, Z_A - Z_M, (0.0, 0.0, (Z_A + Z_M) / 2))],
                mass=HOSE_MASS, damping=D_SWING, friction=0.0, gravity=True, com=(0.0, 0.0, (Z_A + Z_M) / 2),
                inertia=(HOSE_MASS * (Z_A - Z_M) ** 2 / 12, HOSE_MASS * (Z_A - Z_M) ** 2 / 12, 1e-5), parent="swing_x"),
        m.hinge("bend_x", "X", (0.0, 0.0, Z_M), -80.0, 80.0, [], [], mass=0.01, spring=K_BEND, damping=D_BEND, friction=0.0, gravity=True,
                com=(0.0, 0.0, Z_M), inertia=(1e-5, 1e-5, 1e-5), parent="hose_up"),
        m.hinge("valve", "Y", (0.0, 0.0, Z_M), -80.0, 80.0, vparts, vcols, mass=V_MASS + HOSE_MASS, spring=K_BEND, damping=D_BEND,
                friction=0.0, gravity=True, com=(0.0, 0.0, v_com_z), inertia=(0.004, 0.004, 1e-4), parent="bend_x"),
        _handle_link(),
        _ring_link(),
    ]
    return m.mechanism("sprayer", (X_V, Y_V, TOP), links, LOOKS, frame_parts=_frame_parts(), friction=(0.9, 0.8), solver_iters=64)


DISTRACTORS = [
    SimReady("sponge", pos=(0.25, -0.33, TOP + DECK), size=(0.071, 0.091, 0.027), mass=0.02, yaw=0.3,
             package="cleaning_sponge_c01", usd="sm_cleaning_sponge_c01_01.usd"),
    SimReady("plate", pos=(0.46, 0.32, TOP + DECK), size=(0.172, 0.177, 0.026), mass=0.12,
             package="dishware_plastic_bigPlate_a01", usd="sm_dishware_plastic_bigPlate_a01_01.usd"),
    SimReady("mug", pos=(0.47, -0.31, TOP + DECK), size=(0.114, 0.09, 0.08), mass=0.30, yaw=2.4,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
]
OBJECTS = [_dish_table(), _sprayer(), *DISTRACTORS]

# -- the ring lock, the water, and the record kept for scoring ------------------------------------------------------
JET_PATH = f"{m.ENV_PATH}/sprayer/valve/visual/jet"
REST = np.array([X_V, Y_V, TOP + Z_F])  # spray face at rest
HANG_TOL = 0.08
CLEAR = 0.03
STILL_V, STILL_S = 0.03, 2.0
JET_REACH, JET_R0, JET_SPREAD = 0.25, 0.015, 0.15  # how far below the face a gripper deflects the spray, and its cone


def _set_limits(env, link: str, lo: float, hi: float) -> None:
    import torch

    asset = env.unwrapped.scene["sprayer"]
    j = asset.find_joints(f"{link}_joint")[0][0]
    asset.write_joint_position_limit_to_sim_index(limits=torch.tensor([[[float(lo), float(hi)]]], dtype=torch.float32, device=asset.device),
                                                  joint_ids=[j], warn_limit_violation=False)


def _write_joint(env, link: str, pos: float | None = None, vel: float = 0.0) -> None:
    import torch

    asset = env.unwrapped.scene["sprayer"]
    j = asset.find_joints(f"{link}_joint")[0][0]
    if pos is not None:
        asset.write_joint_position_to_sim_index(position=torch.tensor([[float(pos)]], dtype=torch.float32, device=asset.device), joint_ids=[j])
    asset.write_joint_velocity_to_sim_index(velocity=torch.tensor([[float(vel)]], dtype=torch.float32, device=asset.device), joint_ids=[j])


def _valve_pose(env):
    """The valve link's pose (position of the unit's frame origin as carried by the valve, rotation), station frame."""
    asset = env.unwrapped.scene["sprayer"]
    i = asset.body_names.index("valve")
    pose = m._np(asset.data.body_link_pose_w)[0, i]
    origin = m._np(env.unwrapped.scene.env_origins[0])
    return pose[:3] - origin, m.quat_matrix(pose[3:7])


def _to_valve(points, pos, rot):
    """Station points in the valve's own frame: origin at the spray face, z up its axis."""
    return (np.atleast_2d(points) - pos) @ rot - np.array([0.0, 0.0, Z_F])


def _box_dist(p, lo, hi):
    return np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0), axis=1)


@dataclass
class Record:
    hooked: bool = True
    spray_out: bool = False
    why: str = ""
    steps: int = 0
    still: int = 0
    full_steps: int = 0
    face: tuple = (0.0, 0.0, 0.0)
    handle: float = 0.0
    ring: float = 0.0
    clear: bool = False
    is_open: bool = True
    limits: tuple = ()


REC = Record()
_JET_SHOWN = [None]


def _apply_lock(env, h: float, r: float) -> None:
    """The ring's hold on the handle, as end stops: hooked round the tip, it keeps the handle from opening past the lock
    point; loaded by the handle (not squeezed past the lock point), it cannot slide; off the tip, it cannot go back up
    over the tip unless the handle is squeezed in to the lock point."""
    squeezed = h <= H_SQUEEZED
    if REC.hooked and r <= R_OFF:
        REC.hooked = False
    elif not REC.hooked and r > R_OFF and h <= math.radians(0.3):
        REC.hooked = True
    handle_lim = (H_MAX, 0.0) if REC.hooked else (H_MAX, H_OFF)
    if REC.hooked and not squeezed:
        ring_lim = (min(r, 0.0), min(r, 0.0))
    elif not REC.hooked and h > math.radians(0.3):
        ring_lim = (R_LO, R_OFF)
    else:
        ring_lim = (R_LO, 0.0)
    limits = (handle_lim, ring_lim if ring_lim[0] != ring_lim[1] else ("pinned",))
    if limits != REC.limits:
        _set_limits(env, "handle", *handle_lim)
        _set_limits(env, "ring", *ring_lim)
        REC.limits = limits


def _show_jet(on: bool) -> None:
    if _JET_SHOWN[0] != on:
        m.set_visible(JET_PATH, on)
        _JET_SHOWN[0] = on


@m.on_reset
def _reset(env) -> None:
    global REC
    REC = Record()
    _set_limits(env, "handle", H_MAX, 0.0)
    _set_limits(env, "ring", 0.0, 0.0)
    REC.limits = ((H_MAX, 0.0), ("pinned",))
    m.set_target(env, "sprayer", "handle", H_OFF + H_PRELOAD)  # the handle spring: toward its off stop, preloaded
    _write_joint(env, "swing_x", vel=SWING0)
    _JET_SHOWN[0] = None
    _show_jet(True)


@m.before_reset
def _put_back(env) -> None:
    """A later episode starts as the first: every joint of the unit back where it was authored, locked on."""
    for link in ("swing_x", "hose_up", "bend_x", "valve", "handle", "ring"):
        _write_joint(env, link, pos=0.0, vel=0.0)
    _set_limits(env, "handle", H_MAX, 0.0)
    _set_limits(env, "ring", 0.0, 0.0)


@m.on_step
def _step(env) -> None:
    st = m.state(env, "sprayer")
    h, r = st.joint("handle"), st.joint("ring")
    pos, rot = _valve_pose(env)
    face = pos + rot @ np.array([0.0, 0.0, Z_F])
    jet = -rot[:, 2]
    tips = m.fingertips(env)
    REC.steps += 1
    is_open = h <= H_OPEN
    if is_open and not REC.spray_out:
        rim = TOP + DECK
        why = ""
        if face[2] > rim:
            if jet[2] > -0.05:
                why = "jet level or rising above the rim"
            else:
                p = face + jet * (face[2] - rim) / -jet[2]
                if not (BASIN_X0 < p[0] < BASIN_X1 and abs(p[1]) < BASIN_Y):
                    why = f"jet meets the rim plane outside the basin at ({p[0]:.3f}, {p[1]:.3f})"
        elif jet[2] > 0.05:
            p = face + jet * (rim - face[2]) / jet[2]
            if BASIN_X0 < p[0] < BASIN_X1 and abs(p[1]) < BASIN_Y:
                why = "jet rising out of the basin"
        v = tips - face
        along = v @ jet
        off = np.linalg.norm(v - np.outer(along, jet), axis=1)
        hit = (along > 0.0) & (along < JET_REACH) & (off < JET_R0 + JET_SPREAD * along)
        if hit.any():
            why = "a gripper in the spray"
        if why:
            REC.spray_out, REC.why = True, why
    _apply_lock(env, h, r)
    _show_jet(bool(h <= H_OPEN))
    # hands off the valve and the hose's lower half
    loc = _to_valve(tips, pos, rot)
    d_valve = _box_dist(loc, np.array([-0.018, -0.018, -0.002]), np.array([0.018, TAB_Y + TAB[1] / 2, VALVE_LEN]))
    d_hose = _box_dist(loc, np.array([-HOSE_R, -HOSE_R, VALVE_LEN]), np.array([HOSE_R, HOSE_R, Z_M - Z_F]))
    REC.clear = bool(min(d_valve.min(), d_hose.min()) >= CLEAR)
    dt = float(env.unwrapped.step_dt)
    speed = np.linalg.norm(face - np.asarray(REC.face)) / dt if REC.steps > 1 else 1.0
    REC.still = REC.still + 1 if speed < STILL_V else 0
    REC.face, REC.handle, REC.ring, REC.is_open = tuple(float(v) for v in face), float(h), float(r), bool(is_open)
    full_now = (not REC.spray_out and REC.clear and speed < STILL_V
                and h >= H_SHUT and r <= R_OFF
                and np.linalg.norm(face - REST) <= HANG_TOL)
    REC.full_steps = REC.full_steps + 1 if full_now else 0
    if os.environ.get("SPRAYER_LOG") and REC.steps % 5 == 0:
        print("[sprayer] " + json.dumps({"t": round(REC.steps * dt, 2), "h_deg": round(math.degrees(h), 2), "ring_mm": round(r * 1000, 1),
                                         "hooked": REC.hooked, "open": bool(is_open), "face": [round(v, 4) for v in face],
                                         "off_rest_mm": round(float(np.linalg.norm(face - REST)) * 1000, 1), "clear": REC.clear,
                                         "still": REC.still, "spray_out": REC.why, "score": score(None)}), flush=True)


def _outcome() -> tuple[float, str]:
    if REC.steps == 0:
        return 0.0, "not started"
    if REC.spray_out:
        return 0.0, f"sprayed out of the sink ({REC.why})"
    if not REC.clear:
        return 0.0, "a gripper within 3 cm of the valve or hose"
    if REC.still < round(STILL_S * 20):
        return 0.0, "head still moving"
    shut = REC.handle >= H_SHUT and REC.ring <= R_OFF
    if not shut:
        return 0.0, "still on"
    if np.linalg.norm(np.asarray(REC.face) - REST) <= HANG_TOL and REC.full_steps >= 60:
        return 1.0, "shut, hanging free over the sink"
    return 0.4, "shut and released; final position or three-second hold incomplete"


def score(objects) -> float:
    if os.environ.get("SPRAYER_LOG") and objects is not None and REC.steps in (1, 200, 400, 600):
        print("[sprayer] distractors " + json.dumps({n: [round(float(v), 4) for v in objects[n].pos] for n in ("sponge", "plate", "mug")}), flush=True)
    return float(_outcome()[0])


def success(objects) -> bool:
    return bool(_outcome()[0] >= 1.0)


# -- scripted demo ----------------------------------------------------------------------------------------------------
@dataclass
class RotWaypoint(Waypoint):
    """A waypoint with the gripper's full orientation: `approach` is where the fingers point, `closing` the line they
    close along (world vectors)."""

    approach: tuple = (0.0, 0.0, -1.0)
    closing: tuple = (0.0, 1.0, 0.0)


def _rot(approach, closing) -> np.ndarray:
    z = np.asarray(approach, float)
    z /= np.linalg.norm(z)
    x = np.asarray(closing, float)
    x = x - x.dot(z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _wp_rot(w) -> np.ndarray:
    return _rot(w.approach, w.closing) if isinstance(w, RotWaypoint) else _demo.grip_rotation(w.yaw)


def _target(self, t: float):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal, rot = np.asarray(w.pos, dtype=float), _wp_rot(w)
        if t < w.duration:
            s = _ik.smoothstep(t / w.duration)
            return prev_pos + s * (goal - prev_pos), _ik.slerp_matrix(prev_rot, rot, s), prev_finger + s * (w.finger - prev_finger), w.label
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


if getattr(_demo.ArmPlan.target, "__name__", "") != "_target":
    _demo.ArmPlan.target = _target

HOLD_A, HOLD_C = (1.0, 0.0, 0.0), (0.0, -1.0, 0.0)  # holding hand: fingers point forward, close across the valve (y)
PUSH_A, PUSH_C = (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)  # ring hand: fingers point right, close up and down (a flat blade)
GRIP_Z = TOP + Z_F + 0.100  # holding grip height: across body and handle, 3.4 cm below the handle's pivot
GRIP_Y = Y_V + (-R_BODY + (PIVOT[1] + T_H / 2 + 0.034 * GAP_LOCK / L_H)) / 2
PUSH_Y = Y_V + RING_Y1 + RING_WIRE + 0.0035 + 0.013  # ring hand's TCP: fingertips 3.5 mm outside the ring
PUSH_Z0 = TOP + Z_F + Z_RING + 0.002 + 0.006 + 0.008  # blade 8 mm above the tab
PUSH_Z1 = TOP + Z_F + Z_HEAD + RING_WIRE + 0.002 + 0.006 - 0.003  # ring pushed down onto the spray head, 3 mm over


def demo():
    H = lambda d, p, f, lab: RotWaypoint(d, p, f, lab, 0.0, HOLD_A, HOLD_C)  # noqa: E731
    P = lambda d, p, f, lab: RotWaypoint(d, p, f, lab, 0.0, PUSH_A, PUSH_C)  # noqa: E731
    x_in = X_V + 0.002
    right = [
        H(3.0, (X_V - 0.14, GRIP_Y, GRIP_Z), FINGER_OPEN, "in front of the valve"),
        H(3.0, (X_V - 0.14, GRIP_Y, GRIP_Z), FINGER_OPEN, "wait for the swing to die down"),
        H(2.5, (x_in, GRIP_Y, GRIP_Z), FINGER_OPEN, "fingers round body and handle"),
        H(1.0, (x_in, GRIP_Y, GRIP_Z), FINGER_CLOSED, "squeeze body and handle"),
        H(8.0, (x_in, GRIP_Y, GRIP_Z), FINGER_CLOSED, "hold while the ring comes off"),
        H(1.0, (x_in, GRIP_Y, GRIP_Z), FINGER_OPEN, "let go: the handle springs open"),
        H(2.0, (X_V - 0.14, GRIP_Y, GRIP_Z), FINGER_OPEN, "back out"),
        H(2.5, (0.17, -0.16, TOP + 0.38), FINGER_OPEN, "park"),
        H(8.0, (0.17, -0.16, TOP + 0.38), FINGER_OPEN, "wait"),
    ]
    left = [
        P(3.5, (X_V, PUSH_Y + 0.065, PUSH_Z0 + 0.05), FINGER_CLOSED, "beside the valve"),
        P(6.0, (X_V, PUSH_Y + 0.065, PUSH_Z0 + 0.05), FINGER_CLOSED, "wait"),
        P(1.5, (X_V, PUSH_Y + 0.05, PUSH_Z0), FINGER_CLOSED, "down to the ring's height"),
        P(1.5, (X_V, PUSH_Y, PUSH_Z0), FINGER_CLOSED, "blade over the ring's tab"),
        P(2.0, (X_V, PUSH_Y, PUSH_Z1), FINGER_CLOSED, "push the ring down off the handle's tip"),
        P(1.5, (X_V, PUSH_Y + 0.06, PUSH_Z1), FINGER_CLOSED, "out to the side"),
        P(2.0, (X_V, PUSH_Y + 0.065, PUSH_Z0 + 0.08), FINGER_CLOSED, "up and away"),
        P(2.5, (0.22, 0.12, TOP + 0.40), FINGER_CLOSED, "park"),
        P(10.0, (0.22, 0.12, TOP + 0.40), FINGER_CLOSED, "wait"),
    ]
    return {"right": right, "left": left}
