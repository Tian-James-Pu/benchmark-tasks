"""On a kitchen counter (the station table): a 1.7 L stainless cordless electric kettle, filled with 1 L of water,
stands on its round black power base. Its handle points left (+y) and its spout right (-y). The hinged lid (hinge
at the handle side, catch at the spout side) has been dropped shut but not pressed home: it rests on its spring
catch, about 5 degrees open, its front edge about 1 cm proud of the rim. Pressed down at its front with about 10 N,
the lid passes the catch and clicks shut; from then on it stays latched until the lid-release button on top of the
handle is pressed. A mug, a box of sugar and a sponge stand around it.

The kettle's body, lid, release button and switch are the meshes of NVIDIA SimReady `kitchenware_kettle_a01`,
read from the pinned dataset at first run (see README); the lid's hinge, catch and latch are task code.
"""

import math
import os
import pathlib
import subprocess
import sys

import numpy as np

from yam_arena import demo as _demo
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import CACHE, REPO, REVISION, SimReady, fetch  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the SimReady kettle's meshes ---------------------------------------------------------------------------------------

PACKAGE, USD = "kitchenware_kettle_a01", "sm_kitchenware_kettle_a01_01.usd"
_EXTRACT = r"""
import sys, numpy as np
from pxr import Usd, UsdGeom
st = Usd.Stage.Open(sys.argv[1]); cache = UsdGeom.XformCache(); out = {}
for prim in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
    if not prim.IsA(UsdGeom.Mesh):
        continue
    mesh = UsdGeom.Mesh(prim)
    M = np.array(cache.GetLocalToWorldTransform(prim))
    v = np.array(mesh.GetPointsAttr().Get(), float) @ M[:3, :3] + M[3, :3]
    counts = np.array(mesh.GetFaceVertexCountsAttr().Get()); idx = np.array(mesh.GetFaceVertexIndicesAttr().Get())
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    tris = [[idx[s], idx[s + k], idx[s + k + 1]] for s, c in zip(starts, counts) for k in range(1, c - 1)]
    owner = np.repeat(np.arange(len(counts)), counts - 2)
    tris = np.array(tris, int)
    key = [n for n in ("body", "lid", "button", "switch") if n in prim.GetName()][0]
    subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
    groups = [(s.GetPrim().GetName(), np.array(s.GetIndicesAttr().Get(), int)) for s in subsets] or [("all", np.arange(len(counts)))]
    for name, faces in groups:
        sel = np.isin(owner, faces)
        look = "steel" if "metal" in name else "plastic"
        out[f"{key}__{look}__v"] = v
        out[f"{key}__{look}__f"] = tris[sel]
np.savez_compressed(sys.argv[2], **out)
"""


def _kettle_meshes():
    """{(piece, look): Mesh} in the kettle file's frame: origin at the centre of its bottom, z up, handle +y.
    Extracted once from the pinned USD in a separate process (pxr is not imported into the task's own process)."""
    usd = fetch(PACKAGE, USD)
    out = CACHE / REPO / REVISION / PACKAGE / "kettle_meshes_v1.npz"
    if not out.is_file():
        tmp = out.with_suffix(".tmp.npz")
        subprocess.run([sys.executable, "-c", _EXTRACT, str(usd), str(tmp)], check=True)
        os.replace(tmp, out)
    data = np.load(out)
    meshes = {}
    for k in data.files:
        if k.endswith("__f"):
            piece, look, _ = k.split("__")
            f = data[k]
            used = np.unique(f)
            remap = np.full(int(data[f"{piece}__{look}__v"].shape[0]), -1)
            remap[used] = np.arange(len(used))
            meshes[(piece, look)] = m.Mesh(data[f"{piece}__{look}__v"][used], remap[f])
    return meshes


MESH = _kettle_meshes()

# -- the kettle and its base ----------------------------------------------------------------------------------------------

BASE_R, BASE_H = 0.088, 0.016  # power base: 17.6 cm across, 1.6 cm tall
KX, KY = 0.30, 0.0  # kettle and base centre on the counter
BASE_TOP = TOP + BASE_H
HINGE = (0.0, 0.0492, 0.2455)  # lid hinge, along x, at the handle side (from the SimReady file's lid joint)
REL_PIVOT = (0.0, 0.0598, 0.2459)  # lid-release button's pivot (from the file's button joint)
AJAR_DEG = 5.0  # the lid resting on its catch: this far open from shut
LATCH_DEG = 4.6  # past this (of the 5 towards shut) the catch snaps over the lid's tab
RAMP_K = 15.0  # N m/rad: the catch's spring holding the lid up; ~1.3 N m (about 12 N at the front) to click it
LID_MASS, BTN_MASS = 0.09, 0.005
KETTLE_MASS = 2.0  # 1.0 kg empty kettle (less lid) + 1.0 L of water, carried as rigid mass
KETTLE_COM = (0.0, 0.004, 0.080)  # water low in the body, handle offset
KETTLE_INERTIA = (0.0135, 0.0135, 0.0065)
RELEASE_DEG = 4.0  # the release button frees the latch once pressed this far (of 6)

LOOKS = {
    "steel": m.Look(m.rgb(0.78, 0.79, 0.80), metallic=0.9, roughness=0.28),
    "plastic": m.Look(m.rgb(0.05, 0.05, 0.055), roughness=0.5),
    "base": m.Look(m.rgb(0.06, 0.06, 0.065), roughness=0.55),
    "contact": m.Look(m.rgb(0.55, 0.55, 0.57), metallic=0.7, roughness=0.35),
    "cord": m.Look(m.rgb(0.92, 0.92, 0.90), roughness=0.6),
}


def _rot_x(deg, about):
    """Turn points about the x axis through `about`."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    r = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    a = np.asarray(about, float)
    return lambda v: (np.asarray(v) - a) @ r.T + a


_ajar = _rot_x(-AJAR_DEG, HINGE)  # opening is a negative turn about +x: the front (-y) rises


def _hull(points):
    """The convex hull of a point cloud as a Mesh (for a collider)."""
    from scipy.spatial import ConvexHull
    h = ConvexHull(points)
    used = np.unique(h.simplices)
    remap = np.full(len(points), -1)
    remap[used] = np.arange(len(used))
    return m.Mesh(points[used], remap[h.simplices])


def _kettle():
    body_steel, body_plastic = MESH[("body", "steel")], MESH[("body", "plastic")]
    switch = m.join(*[MESH[k] for k in MESH if k[0] == "switch"])
    lid = m.join(*[MESH[k] for k in MESH if k[0] == "lid"])
    button = m.join(*[MESH[k] for k in MESH if k[0] == "button"])
    pts = np.vstack([body_steel.v, body_plastic.v])
    jug = pts[pts[:, 1] < 0.052]
    handle = pts[pts[:, 1] >= 0.052]
    frame_parts = [("body_steel", body_steel, "steel"), ("body_plastic", body_plastic, "plastic"),
                   ("switch", switch, "plastic")]
    frame_cols = [m.col_hull("jug_c", _hull(jug)), m.col_hull("handle_c", _hull(handle))]
    lid_ajar = m.Mesh(_ajar(lid.v), lid.f)
    links = [
        # joint + turns the lid towards shut: 0 = resting on the catch, +5 = shut, -40 = open
        m.hinge("lid", "X", HINGE, -40.0, AJAR_DEG, (("lid", lid_ajar, "plastic"),),
                (m.col_hull("lid_c", _hull(lid_ajar.v)),), mass=LID_MASS, spring=RAMP_K, damping=0.05, effort=50.0,
                friction=0.002, gravity=True),
        # the lid-release button on top of the handle: pressing its outer end down is a negative turn
        m.hinge("release", "X", REL_PIVOT, -6.0, 0.0, (("button", button, "plastic"),),
                (m.col_hull("release_c", _hull(button.v)),), mass=BTN_MASS, spring=0.15, damping=0.002, effort=5.0,
                friction=0.0),
    ]
    return frame_parts, frame_cols, links


FRAME_PARTS, FRAME_COLS, LINKS = _kettle()
KETTLE = m.mechanism("kettle", (KX, KY, BASE_TOP + 0.0005), LINKS, LOOKS, frame_parts=FRAME_PARTS,
                     frame_colliders=FRAME_COLS, friction=(1.0, 0.8), solver_iters=48, free_mass=KETTLE_MASS,
                     frame_inertia=KETTLE_INERTIA, frame_com=KETTLE_COM)

_base_mesh = m.lathe([(0.0, 0.0), (BASE_R - 0.004, 0.0), (BASE_R, 0.004), (BASE_R, BASE_H - 0.004),
                      (BASE_R - 0.005, BASE_H), (0.0, BASE_H)], 48)
BASE = m.fixed("power_base", (KX, KY, TOP), [
    ("plate", _base_mesh, "base"),
    ("contact", m.cylinder(0.014, 0.0015, (0, 0, BASE_H + 0.0002)), "contact"),
    ("cord", m.tube([(0.0, 0.085, 0.005), (0.0, 0.13, 0.004), (0.05, 0.19, 0.004)], 0.0028), "cord"),
], [m.col_cylinder("plate_c", BASE_R, BASE_H, (0, 0, BASE_H / 2))], LOOKS)

OBJECTS = [
    BASE,
    KETTLE,
    # Distractors (SimReady props) that belong on a kitchen counter; scoring ignores them.
    SimReady("mug", pos=(0.30, 0.235, TOP), size=(0.114, 0.090, 0.080), mass=0.35, yaw=0.0,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("sugar", pos=(0.47, 0.17, TOP), size=(0.093, 0.045, 0.176), mass=0.50, yaw=0.3,
             package="food_box_sugar_a01", usd="sm_food_box_sugar_a01_01.usda"),
    SimReady("sponge", pos=(0.17, 0.20, TOP), size=(0.071, 0.101, 0.033), mass=0.03, yaw=0.4,
             package="cleaning_sponge_b01", usd="sm_cleaning_sponge_b01_01.usd"),
]

# -- the catch and latch ------------------------------------------------------------------------------------------------

LID_STATE = {"mode": "ramp"}  # "ramp": on (or pressed onto) the catch; "free": lifted clear of it; "latched"


@m.on_reset
def _reset_lid(environment=None):
    LID_STATE["mode"] = "ramp"
    TRIAL.reset()


@m.on_step
def _lid(environment):
    """A lid lifted clear of the catch swings free under gravity. Lowered onto the catch it rests on the catch's spring;
    pressed past LATCH_DEG it clicks under the catch and is held shut until the release button is pressed."""
    st = m.state(environment, "kettle")
    q, b = st.deg("lid"), st.deg("release")
    mode = LID_STATE["mode"]
    if mode == "latched":
        if -b >= RELEASE_DEG:
            m.unlock(environment, "kettle", "lid")  # back onto the catch's spring: the lid lifts to rest on it
            mode = "ramp"
    elif mode == "ramp":
        if q >= LATCH_DEG:
            m.hold_at(environment, "kettle", "lid", math.radians(AJAR_DEG), stiffness=400.0, damping=1.0, effort=30.0)
            mode = "latched"
        elif q < -1.0:
            m.slacken(environment, "kettle", "lid", 0.01)
            mode = "free"
    elif mode == "free" and q > -0.3:
        m.unlock(environment, "kettle", "lid")
        mode = "ramp"
    LID_STATE["mode"] = mode


# -- scoring ------------------------------------------------------------------------------------------------------------

SUCCESS_HOLD_S = 0.0  # Trial counts its own three-second hold
HOLD_SECONDS = 3.0
TIP_DEG = 15.0  # tilting the full kettle further than this at any time is a harm (a pour starts)
UPRIGHT_DEG = 2.0
SEAT_XY = 0.010
SEAT_Z = 0.002
CLEAR = 0.03


def latched(st):
    return LID_STATE["mode"] == "latched" and st.deg("lid") >= LATCH_DEG


def seated(st):
    """On its base: centred within 1 cm, bottom within 2 mm of the base's top, upright within 2 degrees."""
    d = math.hypot(st.pos[0] - KX, st.pos[1] - KY)
    return bool(d <= SEAT_XY and abs(st.pos[2] - BASE_TOP) <= SEAT_Z and st.tilt_deg() <= UPRIGHT_DEG)


def hands_off(st, tips):
    """Every fingertip point more than 3 cm from the kettle (body 8 cm in radius from the base's bottom to 26 cm up,
    and its handle) and so from the base."""
    if tips is None:
        return False
    p = st.to_local(tips)
    r = np.hypot(p[:, 0], p[:, 1])
    d_body = np.hypot(np.maximum(r - 0.080, 0.0), np.maximum(np.maximum(-BASE_H - p[:, 2], p[:, 2] - 0.26), 0.0))
    lo, hi = np.array([-0.02, 0.05, 0.05]), np.array([0.02, 0.115, 0.25])
    d_handle = np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0), axis=1)
    return bool(np.minimum(d_body, d_handle).min() > CLEAR)


class Trial:
    """Updated once per control step after physics. The tip record is cleared only by a reset."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.tipped, self.hold, self.full, self.value = False, 0.0, False, 0.0

    def update(self, st, tips, dt):
        if st.tilt_deg() > TIP_DEG:
            self.tipped = True
        lat = latched(st)
        still = np.linalg.norm(st.lin_vel) < 0.005 and np.linalg.norm(st.ang_vel) < 0.05
        ready = bool(lat and not self.tipped and seated(st) and still and hands_off(st, tips))
        self.hold = self.hold + float(dt) if ready else 0.0
        self.full = bool(ready and self.hold >= HOLD_SECONDS - 1e-9)
        self.value = 0.0 if self.tipped or not lat else (1.0 if self.full else 0.5)

    def step(self, environment):
        st, tips = m.state(environment, "kettle"), m.fingertips(environment)
        self.update(st, tips, environment.unwrapped.step_dt)


TRIAL = Trial()
m.on_step(TRIAL.step)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo --------------------------------------------------------------------------------------------------

class Orient(float):
    """A waypoint 'yaw' that carries a whole TCP rotation (the lid is too high to reach pointing straight down)."""

    rot = np.eye(3)


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


def tilted(theta_deg, phi_deg):
    """Tilted theta from straight down towards phi (from +x), fingers closing across that direction."""
    phi = math.radians(phi_deg)
    o = Orient(0.0)
    o.rot = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain_grip(phi)
    return o


PRESS_Y = -0.060  # on the lid, towards its front (spout side), in the kettle's frame
PAD = 0.013  # finger pads beyond the TCP, along the fingers
LID_TOP_SHUT = 0.2495  # the lid's top surface there, shut (SimReady mesh)
PRESS_PAST = 0.055  # command the press this far below the shut surface (the wrist is compliant)
TILT, TOWARD = 60.0, 90.0  # fingers pointing 60 degrees from straight down, towards +y (the kettle, from the right arm)


def demo():
    """The right arm comes over the kettle's spout side with its fingers closed and pointing down and in, presses the
    lid's front straight down until it clicks under the catch, lifts off and leaves. The left arm stays where it is."""
    g = tilted(TILT, TOWARD)
    along = g.rot @ np.array([0.0, 0.0, 1.0])  # the TCP's +z: the way the fingers point
    lid_front_ajar = LID_TOP_SHUT + (HINGE[1] - PRESS_Y) * math.sin(math.radians(AJAR_DEG))
    x, y = KX, KY + PRESS_Y

    def tcp(z_tip, back=0.0):
        return tuple(np.array([x, y - back, z_tip]) - PAD * along)

    up = np.array([0.0, 0.0, 1.0])
    right = [
        Waypoint(2.5, tuple(np.array(tcp(BASE_TOP + lid_front_ajar, 0.04)) + 0.05 * up), FINGER_CLOSED, "over the lid's front", g),
        Waypoint(1.5, tcp(BASE_TOP + lid_front_ajar + 0.008), FINGER_CLOSED, "down to the lid", g),
        Waypoint(1.5, tcp(BASE_TOP + LID_TOP_SHUT - PRESS_PAST), FINGER_CLOSED, "press it shut: click", g),
        Waypoint(0.6, tcp(BASE_TOP + LID_TOP_SHUT - PRESS_PAST), FINGER_CLOSED, "hold", g),
        Waypoint(1.2, tuple(np.array(tcp(BASE_TOP + lid_front_ajar, 0.04)) + 0.05 * up), FINGER_CLOSED, "lift off", g),
        Waypoint(1.5, (0.32, -0.16, TOP + 0.30), FINGER_OPEN, "away", g),
        Waypoint(4.0, (0.32, -0.16, TOP + 0.30), FINGER_OPEN, "wait clear", g),
    ]
    return {"right": right}
