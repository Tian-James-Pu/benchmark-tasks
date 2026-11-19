"""Recover a commercial pre-rinse valve left locked on above a dish sink.

The assembly follows the T&S B-0107-J / B-0107-J-SWV manufacturer drawings:
274 mm face-to-swivel length, 38 mm nozzle and a plain 51/38 x 6 mm hold-down
ring. The curved trigger pivots near the nozzle and extends toward the hose.
Squeeze the trigger to unload the ring, lift the ring past its upper free end,
and release the spring-return trigger. Undimensioned contours are estimates.
The hose is a two-link pendulum. The ring is a free rigid torus; its contacts
with the grip and trigger hold the trigger open without a scripted latch.
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
X_V, Y_V = 0.33, 0.0
Z_F = DECK + 0.005  # face just above the sink rim; full-scale valve remains reachable
VALVE_LEN = 0.274
HOSE_LEN = 0.50
Z_A = Z_F + VALVE_LEN + HOSE_LEN
Z_M = Z_F + VALVE_LEN + HOSE_LEN / 2
HOSE_R = 0.0095
BODY_Y = -0.020  # grip/hose centre is offset from nozzle axis, as in the drawing
R_BODY, R_HEAD = 0.013, 0.019
Z_HEAD, Z_BODY = 0.067, 0.139
PIVOT = (0.0, 0.018, 0.088)
L_H, W_H, T_H = 0.143, 0.016, 0.006
ALPHA_LOCK = math.radians(13.3)
H_OFF, H_MAX = -ALPHA_LOCK, math.radians(2.0)
H_SQUEEZED = math.radians(1.0)
H_OPEN, H_SHUT = H_OFF + math.radians(2.5), H_OFF + math.radians(1.0)
H_PRELOAD, K_HANDLE = math.radians(3.0), 3.0
Z_RING = 0.222
TIP_END = 0.231
RING_R, RING_RADIAL, RING_HALF_Z = 0.02225, 0.00325, 0.003
RING_YC = BODY_Y + 0.0055
R_OFF, R_HI = TIP_END + RING_HALF_Z + 0.002 - Z_RING, 0.030
RING_FRICTION = 0.04  # light sliding resistance; gravity exceeds it when unloaded
V_MASS, H_MASS, RING_MASS = 1.080, 0.088, 0.012
HOSE_MASS = 0.28 * HOSE_LEN / 2
D_SWING, D_BEND, K_BEND = 0.12, 0.05, 0.3
SWING0 = 0.0  # no artificial initial swing; the hanging valve moves when touched

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
    curve = _bezier((xr, 0.0, z_top), (xr, 0.0, z_top + 0.12), (0.0, 0.0, Z_A + 0.20), (0.0, BODY_Y, Z_A), 60)
    parts.append(("spring", m.tube(_helix(curve, 0.020, 0.011), 0.0016, sides=6), "chrome"))
    parts.append(("hose_top", m.tube(curve, HOSE_R, sides=12), "hose"))
    parts.append(("spring_end", m.cylinder(0.022, 0.012, (0.0, BODY_Y, Z_A + 0.006)), "chrome"))
    return parts


def _valve_parts():
    parts, cols = [], []
    def cyl(name, r, z0, z1, y=0.0, look="chrome", segments=32):
        c=_v(0.0,y,(z0+z1)/2)
        parts.append((name,m.cylinder(r,z1-z0,c,segments=segments),look))
        cols.append(m.col_cylinder(name+"_c",r,z1-z0,c))
    cyl("nozzle",R_HEAD,0.0,Z_HEAD,look="rubber")
    cyl("nozzle_neck",0.011,Z_HEAD,0.088)
    cyl("body",0.014,0.083,0.123)
    elbow=m.tube([_v(0,0,.114),_v(0,-.010,.125),_v(0,BODY_Y,.133),_v(0,BODY_Y,.139)],.013)
    parts.append(("elbow",elbow,"chrome"));cols.append(m.col_hull("elbow_c",elbow))
    cyl("grip_nut",.016,.134,.156,BODY_Y,segments=6)
    cyl("grip",R_BODY,.156,.245,BODY_Y,"grip")
    cyl("grip_flange",.016,.245,.251,BODY_Y,"grip")
    cyl("swivel",.018,.251,VALVE_LEN,BODY_Y)
    cartridge=m.cylinder(.014,.027,_v(0,.017,.114),axis="y")
    parts.append(("cartridge",cartridge,"chrome"));cols.append(m.col_hull("cartridge_c",cartridge))
    face=m.cylinder(.012,.001,_v(0,0,-.0004))
    parts.append(("face",face,"drain"))
    hose=m.cylinder(HOSE_R,Z_M-Z_F-VALVE_LEN,(0,BODY_Y,(Z_M+Z_F+VALVE_LEN)/2))
    parts.append(("hose_low",hose,"hose"));cols.append(m.col_hull("hose_low_c",hose))
    jet=m.lathe([(0,-.125),(.028,-.125),(.010,-.002),(0,-.002)],segments=24).moved((0,0,Z_F))
    parts.append(("jet",jet,"water"))
    return parts,cols


def _handle_link() -> m.Link:
    # Side contour reconstructed from the manufacturer's drawing. This is a
    # curved flat trigger, not an enlarged robot gripping attachment.
    yz=np.array([[.018,.088],[.030,.101],[.040,.116],[.042,.130],
                 [.036,.145],[.022,.164],[.020,.179],[.026,.199],
                 [.033,.220],[.035,.231]])
    segs=[]
    pivot=np.asarray(PIVOT)
    for (y0,z0),(y1,z1) in zip(yz[:-1],yz[1:]):
        dy,dz=y1-y0,z1-z0
        bar=m.box((W_H,T_H,float(np.hypot(dy,dz))))
        bar=bar.turned(-math.degrees(math.atan2(dy,dz)),"x").moved((0,(y0+y1)/2,(z0+z1)/2))
        bar=bar.moved(-pivot).turned(math.degrees(ALPHA_LOCK),"x").moved(pivot+np.array([0,0,Z_F]))
        segs.append(bar)
    bar=m.join(*segs)
    pin=m.cylinder(.0055,.020,_v(*PIVOT),axis="x")
    cols=[m.col_hull("handle_%d_c"%i,v).replace('contactOffset = 0.004','contactOffset = 0.001') for i,v in enumerate(segs)]
    return m.hinge("handle","X",_v(*PIVOT),math.degrees(H_OFF),math.degrees(H_MAX),
                   [("lever",bar,"chrome"),("pin",pin,"chrome")],cols,
                   mass=H_MASS,spring=K_HANDLE,damping=.012,friction=0,effort=5,gravity=True,
                   com=tuple(np.mean(bar.v,axis=0)),inertia=(.00016,.00016,.000004),parent="valve")


def _ring_link() -> m.Link:
    # 51 mm OD, 38 mm ID, 6 mm thickness. Elliptical-section brass ring.
    theta=np.linspace(0,2*math.pi,49)
    path=np.array([(RING_R*math.cos(t),RING_YC+RING_R*math.sin(t),Z_F+Z_RING) for t in theta])
    segs=[]
    for k in range(48):
        mesh=m.tube(path[k:k+2],RING_RADIAL,sides=12)
        mesh.v[:,2]=Z_F+Z_RING+(mesh.v[:,2]-Z_F-Z_RING)*RING_HALF_Z/RING_RADIAL
        segs.append(mesh)
    loop=m.join(*segs)
    cols=[m.col_hull("ring_%d_c"%k,mesh).replace('contactOffset = 0.004','contactOffset = 0.001') for k,mesh in enumerate(segs)]
    return m.slider("ring","Z",-0.10,R_HI,[("loop",loop,"chrome")],cols,
                    at=_v(0,RING_YC,Z_RING),mass=RING_MASS,damping=.1,friction=RING_FRICTION,
                    effort=20,gravity=True,com=_v(0,RING_YC,Z_RING),
                    inertia=(3e-6,3e-6,6e-6),parent="valve")


def _sprayer() -> m.Part:
    hose_up = m.cylinder(HOSE_R, Z_A - Z_M, (0.0, BODY_Y, (Z_A + Z_M) / 2), segments=16)
    vparts, vcols = _valve_parts()
    v_com_z = (V_MASS * (Z_F + 0.150) + HOSE_MASS * (Z_M + Z_F + VALVE_LEN) / 2) / (V_MASS + HOSE_MASS)
    links = [
        m.hinge("swing_x", "X", (0.0, BODY_Y, Z_A), -70.0, 70.0, [], [], mass=0.01, damping=D_SWING, friction=0.0, gravity=True,
                com=(0.0, BODY_Y, Z_A), inertia=(1e-5, 1e-5, 1e-5)),
        m.hinge("hose_up", "Y", (0.0, BODY_Y, Z_A), -70.0, 70.0, [("hose", hose_up, "hose")], [m.col_cylinder("hose_up_c", HOSE_R, Z_A - Z_M, (0.0, BODY_Y, (Z_A + Z_M) / 2))],
                mass=HOSE_MASS, damping=D_SWING, friction=0.0, gravity=True, com=(0.0, BODY_Y, (Z_A + Z_M) / 2),
                inertia=(HOSE_MASS * (Z_A - Z_M) ** 2 / 12, HOSE_MASS * (Z_A - Z_M) ** 2 / 12, 1e-5), parent="swing_x"),
        m.hinge("bend_x", "X", (0.0, BODY_Y, Z_M), -80.0, 80.0, [], [], mass=0.01, spring=K_BEND, damping=D_BEND, friction=0.0, gravity=True,
                com=(0.0, BODY_Y, Z_M), inertia=(1e-5, 1e-5, 1e-5), parent="hose_up"),
        m.hinge("valve", "Y", (0.0, BODY_Y, Z_M), -80.0, 80.0, vparts, vcols, mass=V_MASS + HOSE_MASS, spring=K_BEND, damping=D_BEND,
                friction=0.0, gravity=True, com=(0.0, -0.014, v_com_z), inertia=(0.009, 0.009, 0.0003), parent="bend_x"),
        _handle_link(),

    ]
    return m.mechanism("sprayer", (X_V, Y_V, TOP), links, LOOKS, frame_parts=_frame_parts(), friction=(0.4, 0.3), solver_iters=64)


DISTRACTORS = [
    SimReady("sponge", pos=(0.25, -0.33, TOP + DECK), size=(0.071, 0.091, 0.027), mass=0.02, yaw=0.3,
             package="cleaning_sponge_c01", usd="sm_cleaning_sponge_c01_01.usd"),
    SimReady("plate", pos=(0.46, 0.32, TOP + DECK), size=(0.172, 0.177, 0.026), mass=0.12,
             package="dishware_plastic_bigPlate_a01", usd="sm_dishware_plastic_bigPlate_a01_01.usd"),
    SimReady("mug", pos=(0.47, -0.31, TOP + DECK), size=(0.114, 0.09, 0.08), mass=0.30, yaw=2.4,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
]
_ring_spec = _ring_link()
_loose_ring = m.free("ring", (X_V,Y_V,TOP), _ring_spec.parts,_ring_spec.colliders,LOOKS,RING_MASS,friction=(.4,.3),torsional_patch=.002,solver_iters=64)
OBJECTS = [_dish_table(), _sprayer(), _loose_ring, *DISTRACTORS]

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


REC = Record()
_JET_SHOWN = [None]



def _show_jet(on: bool) -> None:
    if _JET_SHOWN[0] != on:
        m.set_visible(JET_PATH, on)
        _JET_SHOWN[0] = on


@m.on_reset
def _reset(env) -> None:
    global REC
    REC = Record()
    _set_limits(env, "handle", H_OFF, H_MAX)
    m.set_target(env, "sprayer", "handle", H_OFF - H_PRELOAD)  # the handle spring: toward its off stop, preloaded
    _write_joint(env, "swing_x", vel=SWING0)
    _JET_SHOWN[0] = None
    _show_jet(True)


@m.before_reset
def _put_back(env) -> None:
    """A later episode starts as the first: every joint of the unit back where it was authored, locked on."""
    for link in ("swing_x", "hose_up", "bend_x", "valve", "handle"):
        _write_joint(env, link, pos=0.0, vel=0.0)
    _set_limits(env, "handle", H_OFF, H_MAX)


@m.on_step
def _step(env) -> None:
    st = m.state(env, "sprayer")
    h = st.joint("handle")
    ring = m.state(env,"ring")
    pos, rot = _valve_pose(env)
    r = float(np.asarray((ring.to_world(np.array([0.0,RING_YC,Z_F+Z_RING])) - pos) @ rot[:,2] - Z_F-Z_RING).item())
    face = pos + rot @ np.array([0.0, 0.0, Z_F])
    jet = -rot[:, 2]
    tips = m.fingertips(env)
    REC.steps += 1
    is_open = h >= H_OPEN
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
    _show_jet(bool(h >= H_OPEN))
    # hands off the valve and the hose's lower half
    loc = _to_valve(tips, pos, rot)
    d_valve = _box_dist(loc, np.array([-0.026, BODY_Y - 0.020, -0.002]), np.array([0.026, 0.050, VALVE_LEN]))
    d_hose = _box_dist(loc, np.array([-HOSE_R, BODY_Y-HOSE_R, VALVE_LEN]), np.array([HOSE_R, BODY_Y+HOSE_R, Z_M - Z_F]))
    ring_loc = np.atleast_2d(ring.to_local(tips)) - np.array([0.0,RING_YC,Z_F+Z_RING])
    ring_radial = np.linalg.norm(ring_loc[:,:2],axis=1)
    d_ring = np.hypot(ring_radial-RING_R,ring_loc[:,2])-RING_RADIAL
    REC.clear = bool(min(d_valve.min(), d_hose.min(),d_ring.min()) >= CLEAR)
    dt = float(env.unwrapped.step_dt)
    speed = np.linalg.norm(face - np.asarray(REC.face)) / dt if REC.steps > 1 else 1.0
    REC.still = REC.still + 1 if speed < STILL_V else 0
    REC.face, REC.handle, REC.ring, REC.is_open = tuple(float(v) for v in face), float(h), float(r), bool(is_open)
    full_now = (not REC.spray_out and REC.clear and speed < STILL_V
                and h <= H_SHUT
                and np.linalg.norm(face - REST) <= HANG_TOL)
    REC.full_steps = REC.full_steps + 1 if full_now else 0
    if os.environ.get("SPRAYER_LOG") and REC.steps in (150,180,200,240):
        print("[contact] "+json.dumps({"step":REC.steps,"left_points":np.round(tips[:12],4).tolist()}),flush=True)
    if os.environ.get("SPRAYER_LOG") and REC.steps % 5 == 0:
        print("[sprayer] " + json.dumps({"t": round(REC.steps * dt, 2), "h_deg": round(math.degrees(h), 2), "ring_mm": round(r * 1000, 1),
                                         "open": bool(is_open), "ring_center": np.round(ring.to_world(np.array([0.0,RING_YC,Z_F+Z_RING])),4).tolist(), "face": [round(v, 4) for v in face],
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
    shut = REC.handle <= H_SHUT
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
GRIP_Z = TOP + Z_F + 0.180
GRIP_Y = Y_V - 0.016
PUSH_Y = Y_V + RING_YC + RING_R + RING_RADIAL - 0.006
PUSH_Z0 = TOP + Z_F + Z_RING - RING_HALF_Z - 0.008
PUSH_Z1 = TOP + Z_F + Z_RING + R_HI - RING_HALF_Z - 0.004


def demo():
    from yam_arena.constants import READY
    from yam_arena import station
    result={}
    for side in ('left','right'):
        pos,rot=_ik.tcp_pose(READY,station.BASES[side])
        result[side]=[RotWaypoint(20.,tuple(pos),FINGER_OPEN,'remain at initial pose',0.,tuple(rot[:,2]),tuple(rot[:,0]))]
    return result
