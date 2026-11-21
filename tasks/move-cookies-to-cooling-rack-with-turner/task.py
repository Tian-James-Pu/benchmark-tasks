"""Move four cookies from a quarter sheet pan to a wire cooling rack with a turner, without touching them by hand.

Scene (world frame: x forward from the arms, y left, z up, metres; TOP is the table top):

- a Nordic Ware Naturals quarter sheet pan (aluminium, 12.1 x 8.8 x 1.1 in inside; outside size and 0.45 kg estimated), loose,
  its long side pointing away from the robot;
- four cookie replicas, each a 57 mm, 10 mm thick, 16 g disc with a rounded underside edge (the size and mass of a
  USDA "medium" home-baked chocolate chip cookie), two by two at the pan's far end;
- a footed wire cooling rack for a quarter sheet pan (12 x 8.5 in), loose, to the robot's right of the pan;
- a 13 in stainless solid turner (NVIDIA SimReady kitchenware_spatula_i01: its mesh, with simple colliders),
  lying on the table to the right of the rack, handle towards the robot;
- distractors (SimReady): a rolling pin, a mixing bowl and a salt shaker.

Scoring is in `Trial` below and in task.json; `demo()` is an open-loop plan for the stock yam_scripted policy.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import demokit as d  # noqa: E402  (its grip helpers; importing it lets a Waypoint carry any TCP rotation)
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady, fetch  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = os.environ.get("COOKIE_DEBUG", "") != ""

LOOKS = {
    "alu": m.Look(m.rgb(0.80, 0.80, 0.82), metallic=0.75, roughness=0.35),
    "chrome": m.Look(m.rgb(0.85, 0.86, 0.88), metallic=0.95, roughness=0.15),
    "steel": m.Look(m.rgb(0.75, 0.76, 0.78), metallic=0.85, roughness=0.25),
    "handle": m.Look(m.rgb(0.06, 0.06, 0.06), roughness=0.7),
    "dough": m.Look(m.rgb(0.80, 0.58, 0.33), roughness=0.85),
    "chip": m.Look(m.rgb(0.22, 0.12, 0.07), roughness=0.6),
}


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return (x, y, z, w)


def hull(points) -> m.Mesh:
    """A closed mesh of the convex hull of `points` (for a collider made from a few corner points)."""
    from scipy.spatial import ConvexHull

    pts = np.asarray(points, float)
    return m.Mesh(pts, ConvexHull(pts).simplices)


def material(colliders: str, name: str, friction: tuple[float, float]) -> tuple[str, str]:
    """Bind `colliders` (USD text) to their own physics material instead of the part's, and return (colliders, the material)."""
    text = colliders.replace("PhysicsMaterial>", f"{name}>")
    mat = f"""
    def Material "{name}" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {{
        float physics:staticFriction = {friction[0]}
        float physics:dynamicFriction = {friction[1]}
        float physics:restitution = 0
    }}"""
    return text, mat


# -- the sheet pan ------------------------------------------------------------------------------------------------------
# Nordic Ware Naturals Baker's Quarter Sheet: 12.1 x 8.8 x 1.1 in inside (307 x 224 x 28 mm). Mass, floor and flare are estimates.
PAN_IN = (0.307, 0.224)  # inside length (along x here), width (along y)
PAN_H = 0.028
PAN_FLARE = 0.010  # each wall leans out 10 mm, which with the 3 mm bead gives the outside size
PAN_FLOOR = 0.0015  # floor thickness: the cookies sit at this height in the pan's frame
PAN_T = 0.0015
PAN_MASS = 0.45
PAN_POS = np.array([0.335, 0.020, TOP])


def _wall(length: float, along_x: bool, sign: float) -> m.Mesh:
    """One flared wall: a slab leaning outwards, its inside foot on the floor's edge."""
    half_in = (PAN_IN[1] if along_x else PAN_IN[0]) / 2
    b = m.box((PAN_T, length, PAN_H - PAN_FLOOR), (0.0, 0.0, (PAN_H + PAN_FLOOR) / 2))
    v = b.v.copy()
    lean = (v[:, 2] - PAN_FLOOR) / (PAN_H - PAN_FLOOR) * PAN_FLARE
    v[:, 0] = v[:, 0] + half_in + PAN_T / 2 + lean
    v[:, 0] *= sign
    mesh = m.Mesh(v, b.f if sign > 0 else b.f[:, ::-1])
    return mesh.turned(90.0, "z") if along_x else mesh


def _pan() -> m.Part:
    L, W = PAN_IN
    walls = {
        "wall_xp": _wall(W + 2 * PAN_FLARE + 0.006, False, 1.0),
        "wall_xn": _wall(W + 2 * PAN_FLARE + 0.006, False, -1.0),
        "wall_yp": _wall(L + 2 * PAN_FLARE + 0.006, True, 1.0),
        "wall_yn": _wall(L + 2 * PAN_FLARE + 0.006, True, -1.0),
    }
    ox, oy = L / 2 + PAN_T + PAN_FLARE, W / 2 + PAN_T + PAN_FLARE
    bead = m.tube([(-ox, -oy, PAN_H), (ox, -oy, PAN_H), (ox, oy, PAN_H), (-ox, oy, PAN_H), (-ox, -oy + 1e-4, PAN_H)], 0.003, sides=10)
    floor = m.box((L + 2 * PAN_T, W + 2 * PAN_T, PAN_FLOOR), (0, 0, PAN_FLOOR / 2))
    parts = [("floor", floor, "alu"), ("bead", bead, "alu")] + [(k, v, "alu") for k, v in walls.items()]
    cols = [m.col_box("floor_c", (L + 2 * PAN_T, W + 2 * PAN_T, PAN_FLOOR), (0, 0, PAN_FLOOR / 2))]
    for k, v in walls.items():  # each wall as five hulls: one long thin hull may not collide
        long_axis = 1 if k.startswith("wall_x") else 0
        lo, hi = v.v[:, long_axis].min(), v.v[:, long_axis].max()
        edges = np.linspace(lo, hi, 6)
        for i in range(5):
            piece = v.v.copy()
            piece[:, long_axis] = np.clip(piece[:, long_axis], edges[i], edges[i + 1])
            cols.append(m.col_hull(f"{k}_c{i}", hull(np.unique(np.round(piece, 6), axis=0))))
    return m.free("pan", PAN_POS, parts, cols, LOOKS, mass=PAN_MASS, friction=(0.45, 0.40), torsional_patch=None)


# -- the cookies --------------------------------------------------------------------------------------------------------
# USDA FoodData Central 172808, chocolate chip cookies made from a recipe with butter: "1 cookie, medium (2-1/4" dia)" = 16 g.
COOKIE_R = 0.0285
COOKIE_H = 0.010
COOKIE_MASS = 0.016
COOKIE_PROFILE = [(0.0, 0.0), (0.0245, 0.0), (0.0268, 0.0007), (0.0281, 0.0020), (0.0285, 0.0040), (0.0279, 0.0065),
                  (0.0255, 0.0085), (0.018, 0.0096), (0.0, 0.0100)]
# On the pan's floor, two by two at the far end; pan frame (x along the pan's length, from its centre).
COOKIE_AT = {"cookie_1": (0.040, -0.055), "cookie_2": (0.040, 0.055), "cookie_3": (0.115, -0.055), "cookie_4": (0.115, 0.055)}
COOKIES = tuple(COOKIE_AT)


def _cookie_mesh() -> tuple[m.Mesh, m.Mesh]:
    body = m.lathe(COOKIE_PROFILE, 40)
    rng = np.random.default_rng(7)
    chips = []
    for _ in range(9):
        r, t = 0.021 * math.sqrt(rng.random()), 2 * math.pi * rng.random()
        x, y = r * math.cos(t), r * math.sin(t)
        top = np.interp(math.hypot(x, y), [p[0] for p in COOKIE_PROFILE[::-1][:4]], [p[1] for p in COOKIE_PROFILE[::-1][:4]])
        chips.append(m.cylinder(0.0028, 0.0022, (x, y, top - 0.0003), segments=8))
    return body, m.join(*chips)


def _cookies() -> list[m.Part]:
    body, chips = _cookie_mesh()
    out = []
    for name, (px, py) in COOKIE_AT.items():
        pos = PAN_POS + np.array([px, py, PAN_FLOOR + 0.0002])
        out.append(m.free(name, pos, [("dough", body, "dough"), ("chips", chips, "chip")], [m.col_hull("c", body)], LOOKS,
                          mass=COOKIE_MASS, friction=(0.50, 0.45), torsional_patch=None))
    return out


# -- the cooling rack ---------------------------------------------------------------------------------------------------
# A footed quarter-size wire cooling rack, 12 x 8.5 in (305 x 216 mm), chrome-plated steel, 0.62 lb shipping weight.
RACK_SIZE = (0.305, 0.216)
RACK_FOOT = 0.0145  # feet: wires 14.5 mm tall
RACK_TOP = RACK_FOOT + 0.005  # top of the grid: 19.5 mm above the table
RACK_PITCH = 0.0127  # 1/2 in between the long wires
RACK_MASS = 0.28
RACK_POS = np.array([0.350, -0.235, TOP])


def _rack() -> m.Part:
    L, W = RACK_SIZE
    parts, cols = [], []
    zc = RACK_FOOT + 0.0015  # cross wires, 3 mm, on the feet
    zl = RACK_FOOT + 0.004  # long wires, 2 mm, on the cross wires
    frame = m.tube([(-L / 2, -W / 2, zl - 0.001), (L / 2, -W / 2, zl - 0.001), (L / 2, W / 2, zl - 0.001), (-L / 2, W / 2, zl - 0.001),
                    (-L / 2, -W / 2 + 1e-4, zl - 0.001)], 0.002, sides=10)
    parts.append(("frame", frame, "chrome"))
    for i, s in enumerate((-1, 1)):
        cols.append(m.col_box(f"frame_x{i}", (L + 0.004, 0.004, 0.004), (0, s * W / 2, zl - 0.001)))
        cols.append(m.col_box(f"frame_y{i}", (0.004, W + 0.004, 0.004), (s * L / 2, 0, zl - 0.001)))
    for i, x in enumerate(np.linspace(-L / 2 + 0.02, L / 2 - 0.02, 6)):
        w = m.cylinder(0.0015, W, (x, 0, zc), axis="y", segments=10)
        parts.append((f"cross{i}", w, "chrome"))
        cols.append(m.col_box(f"cross{i}_c", (0.003, W, 0.003), (x, 0, zc)))
    n = int(W / RACK_PITCH)
    for i in range(1, n):
        y = -W / 2 + i * W / n
        w = m.cylinder(0.001, L, (0, y, zl), axis="x", segments=8)
        parts.append((f"long{i}", w, "chrome"))
        cols.append(m.col_box(f"long{i}_c", (L, 0.002, 0.002), (0, y, zl)))  # a thin hull of a 305 mm wire does not collide
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        x, y = sx * (L / 2 - 0.02), sy * (W / 2 - 0.015)
        foot = m.cylinder(0.0015, RACK_FOOT, (x, y, RACK_FOOT / 2), segments=8)
        parts.append((f"foot{i}", foot, "chrome"))
        cols.append(m.col_box(f"foot{i}_c", (0.003, 0.003, RACK_FOOT), (x, y, RACK_FOOT / 2)))
    return m.free("rack", RACK_POS, parts, cols, LOOKS, mass=RACK_MASS, friction=(0.55, 0.50), torsional_patch=None)


# Where the cookies go on the rack (rack frame), and the order: far ones first, so the blade never passes over a cookie.
RACK_AT = {"cookie_1": (0.090, 0.045), "cookie_2": (0.090, -0.045), "cookie_3": (0.015, 0.045), "cookie_4": (0.015, -0.045)}

# -- the turner ---------------------------------------------------------------------------------------------------------
# SimReady kitchenware_spatula_i01: a 330 mm solid turner, 85 mm wide blade, 19.5 x 19 mm handle. In its own frame the
# blade points +y. Its blade's underside rises 10.8 mm per 100 mm towards the front edge (a slight offset).
TURNER_PKG, TURNER_USD = "kitchenware_spatula_i01", "sm_kitchenware_spatula_i01_01.usd"
BLADE_Y0, BLADE_Y1 = -0.012, 0.0790  # heel (where the blade reaches full width) and front edge (turner frame)
BLADE_HALF = 0.0424
BLADE_SLOPE = 0.108
BLADE_T = 0.0010  # blade thickness: the prop's blade, about 2.5 mm, is thinned to this (a turner blade is about 1 mm)
TURNER_MASS = 0.100
TURNER_COM = (0.0, -0.060, 0.008)
GRIP_Y = -0.175  # where the gripper holds the handle (turner frame)
TURNER_POS = np.array([0.330, -0.405, TOP])
TURNER_YAW = -90.0  # blade towards +x


def blade_z(y):
    """Height of the blade's underside above the turner's origin."""
    return 0.0004 + BLADE_SLOPE * (np.asarray(y) - BLADE_Y0)


def _turner_mesh() -> tuple[m.Mesh, m.Mesh]:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(fetch(TURNER_PKG, TURNER_USD)))
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    vs, fs, n = [], [], 0
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        p = np.array([xf.Transform(q) for q in pts], float) * mpu
        counts, idx, k = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get(), 0
        for c in counts:
            for j in range(1, c - 1):
                fs.append((n + idx[k], n + idx[k + j], n + idx[k + j + 1]))
            k += c
        vs.append(p)
        n += len(p)
    v, f = np.vstack(vs), np.array(fs)
    # Thin the blade to BLADE_T about its underside (a mesh is only what is seen).
    on = v[:, 1] > BLADE_Y0 - 0.002
    zb = blade_z(v[on, 1])
    v[on, 2] = zb + np.clip(v[on, 2] - zb, 0.0, None) * (BLADE_T / 0.0026)
    steel = v[f].mean(axis=1)[:, 1] > -0.133
    return m.Mesh(v, f[steel]), m.Mesh(v, f[~steel])


class LazyParts(list):
    """The turner's visual meshes, read from the SimReady prop only when the part is written out (after the simulator
    has started: importing pxr before it would load a second USD library)."""

    plate = None  # the solid blade plate (the collider's shape), shown too: the prop's blade is slotted, the simulated one solid

    def __iter__(self):
        if not super().__len__():
            steel, handle = _turner_mesh()
            self.extend([("blade", steel, "steel"), ("handle", handle, "handle")])
            if self.plate is not None:
                self.append(("blade_plate", self.plate, "steel"))
        return super().__iter__()


def _turner() -> m.Part:
    parts = LazyParts()
    # Blade: a thin plate on the blade's underside plane, its front 8 mm bevelled to 0.35 mm.
    ys = np.array([BLADE_Y0, BLADE_Y1 - 0.008, BLADE_Y1])
    top = np.array([BLADE_T, BLADE_T, 0.00035])
    pts = []
    for x in (-BLADE_HALF, BLADE_HALF):
        for y, t in zip(ys, top):
            pts += [(x, y, float(blade_z(y))), (x, y, float(blade_z(y)) + t)]
    blade = hull(pts)
    parts.plate = blade
    # The neck arches up from the handle and down to the blade: three hulls, so that nothing fills in under the arch.
    def seg(*rows, w=0.0042):
        return hull(np.array([(sx * (w if len(r) == 3 else r[3]), r[0], z) for r in rows for sx in (-1, 1) for z in r[1:3]]))

    neck_a = seg((-0.136, 0.0050, 0.0170), (-0.115, 0.0091, 0.0141), (-0.070, 0.0083, 0.0133))
    neck_b = seg((-0.070, 0.0083, 0.0133), (-0.045, 0.0052, 0.0098), (-0.036, 0.0038, 0.0084))
    heel = seg((-0.036, 0.0038, 0.0084, 0.012), (BLADE_Y0 + 0.001, float(blade_z(BLADE_Y0 + 0.001)), float(blade_z(BLADE_Y0 + 0.001)) + BLADE_T, BLADE_HALF - 0.004))
    hpts = [(sx * 0.0097, y, z) for sx in (-1, 1) for y, z in ((-0.251, 0.0030), (-0.251, 0.0175), (-0.245, 0.0006), (-0.245, 0.0192),
                                                             (-0.140, 0.0022), (-0.140, 0.0212))]
    handle_c = m.col_hull("handle_c", hull(hpts))
    handle_c, handle_mat = material(handle_c, "GripMaterial", (1.0, 0.9))
    cols = [m.col_hull("blade_c", blade), m.col_hull("neck_a", neck_a), m.col_hull("neck_b", neck_b), m.col_hull("heel_c", heel), handle_c, handle_mat]
    return m.free("turner", TURNER_POS, parts, cols, LOOKS, mass=TURNER_MASS, friction=(0.20, 0.18), yaw_deg=TURNER_YAW,
                  com=TURNER_COM)


# -- distractors (SimReady, ignored by the scorer) ---------------------------------------------------------------------
DISTRACTORS = [
    SimReady("rolling_pin", pos=(0.330, 0.330, TOP), size=(0.063, 0.440, 0.063), mass=0.35, package="kitchenware_rollingPin_a01",
             usd="sm_kitchenware_rollingPin_a01_01.usd", yaw=math.pi / 2, angular_damping=2.0),
    SimReady("mixing_bowl", pos=(0.600, 0.120, TOP), size=(0.173, 0.173, 0.060), mass=0.30, package="kitchenware_bowl_b01",
             usd="sm_kitchenware_bowl_b01_01.usd"),
    SimReady("salt_shaker", pos=(0.560, -0.150, TOP), size=(0.062, 0.062, 0.110), mass=0.12, package="kitchenware_shaker_a01",
             usd="sm_kitchenware_shaker_a01_01.usd"),
]

PAN, RACK, TURNER = _pan(), _rack(), _turner()
OBJECTS = [PAN, RACK, TURNER, *_cookies(), *DISTRACTORS]

# -- scoring ------------------------------------------------------------------------------------------------------------
TOUCH_MM = 5.0  # a finger point this close to a cookie's surface counts as touching it
DROP_SPEED = 0.45  # m/s downward at any point of the cookie: the speed of a 10 mm free fall (the scripted deposit peaks at 0.35)
REST_GAP_MM = 4.0  # a cookie within this of the surface under it is resting on it
DROP_GAP_MM = 13.0  # or, falling, this far above whatever is under it; or resting this much lower than when it last rested
GOOD_TILT = 8.0  # degrees
SEATED_MM = 3.0  # a cookie's underside within this of the grid's top: resting on the wires
CLEAR_M = 0.05  # grippers this far from every cookie, the rack and the turner for the last 0.2
PAN_LIFT_MM, PAN_TILT_DEG = 20.0, 10.0
PAN_SHIFT_MM, PAN_TURN_DEG = 30.0, 10.0
HOLD_STEPS = 20  # 1 s at 20 Hz


RIM_POINTS = np.array([(COOKIE_R * math.cos(t), COOKIE_R * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, 12, endpoint=False)
                       for z in (0.0, COOKIE_H / 2)])


def _yaw_deg(r: np.ndarray) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


class Trial:
    """Everything that can only be judged over time: touches, drops, flips and the pan's tipping. Cleared only on a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.touched = {c: False for c in COOKIES}
        self.dropped = {c: False for c in COOKIES}
        self.flipped = {c: False for c in COOKIES}
        self.max_pt = {c: 0.0 for c in COOKIES}  # debug: the fastest downward speed seen at any point of the cookie
        self.rest_z = {c: None for c in COOKIES}  # the cookie's lowest point the last time it was seen resting on something
        self.pan_harm = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.states = {}
        self.steps = 0
        self.last = {}

    def _cookie_touch(self, s: m.PartState, tips: np.ndarray) -> bool:
        local = s.to_local(tips)
        r = np.hypot(local[:, 0], local[:, 1])
        out_r = np.clip(r - COOKIE_R, 0.0, None)
        out_z = np.maximum(np.clip(-local[:, 2], 0.0, None), np.clip(local[:, 2] - COOKIE_H, 0.0, None))
        return bool(np.any(np.hypot(out_r, out_z) < TOUCH_MM / 1000.0))

    @staticmethod
    def _point_vz(s, turner=None) -> float:
        """The most negative vertical speed (m/s) of any point on the cookie's rim, top or bottom (a tumbling cookie's
        far edge can slap down much faster than its centre moves). With `turner` (the cookie is over its blade), the
        speed is taken relative to the blade at the same point, so a cookie carried down on the blade is not falling."""
        rel = s.rotation() @ RIM_POINTS.T
        pts = s.pos[None, :] + rel.T
        v = s.lin_vel[:, None] + np.cross(s.ang_vel, rel.T).T
        vz = np.concatenate([[s.lin_vel[2]], v[2]])
        if turner is not None:
            com = turner.to_world(np.array([TURNER_COM]))[0]
            allp = np.vstack([s.pos[None, :], pts])
            vt = turner.lin_vel[None, :] + np.cross(turner.ang_vel[None, :], allp - com)
            vz = vz - vt[:, 2]
        return float(vz.min())

    @staticmethod
    def _over_blade(s, turner) -> bool:
        b = turner.to_local(s.pos[None])[0]
        return bool(abs(b[0]) <= BLADE_HALF + 0.01 and BLADE_Y0 - 0.01 <= b[1] <= BLADE_Y1 + 0.01 and -0.01 <= b[2] - float(blade_z(np.clip(b[1], BLADE_Y0, BLADE_Y1))) <= 0.02)

    @staticmethod
    def _turner_gap(turner, tips) -> float:
        """Distance (m) from the finger points to the turner: boxes that enclose its blade, neck and handle."""
        local = turner.to_local(tips)
        top = float(blade_z(BLADE_Y1)) + BLADE_T
        boxes = [((-BLADE_HALF, BLADE_Y0, 0.0), (BLADE_HALF, BLADE_Y1, top)),  # blade
                 ((-0.012, -0.140, 0.0), (0.012, BLADE_Y0, 0.0175)),  # neck
                 ((-0.0097, -0.251, 0.0), (0.0097, -0.140, 0.0212))]  # handle
        gaps = []
        for lo, hi in boxes:
            lo, hi = np.array(lo), np.array(hi)
            d = np.maximum(np.maximum(lo - local, local - hi), 0.0)
            gaps.append(np.linalg.norm(d, axis=1).min())
        return float(min(gaps))

    def _support_gap(self, c: str, st) -> float:
        """How far (m) the cookie is above the highest surface under its centre: the table, the pan's floor, the rack's
        grid, the turner's blade or another cookie. Measured both from the centre of its underside and from its lowest
        rim point, and the smaller is returned, so that a cookie lying tilted on a tilted blade, or leaning with its
        rim on the wires while the rest is still on the blade, counts as supported."""
        s = st[c]
        r = s.rotation()
        centre = s.pos[2] + min(0.0, COOKIE_H * r[2, 2])
        rim = centre - COOKIE_R * math.hypot(r[2, 0], r[2, 1])
        under = [TOP]
        pan, rack, turner = st["pan"], st["rack"], st["turner"]
        p = pan.to_local(s.pos[None])[0]
        if abs(p[0]) <= PAN_IN[0] / 2 and abs(p[1]) <= PAN_IN[1] / 2:
            under.append(pan.pos[2] + PAN_FLOOR)
        q = rack.to_local(s.pos[None])[0]
        if abs(q[0]) <= RACK_SIZE[0] / 2 and abs(q[1]) <= RACK_SIZE[1] / 2:
            under.append(rack.pos[2] + RACK_TOP)
        b = turner.to_local(s.pos[None])[0]
        if abs(b[0]) <= BLADE_HALF + 0.01 and BLADE_Y0 - 0.01 <= b[1] <= BLADE_Y1 + 0.01:
            ly = float(np.clip(b[1], BLADE_Y0, BLADE_Y1))
            under.append(float(turner.to_world(np.array([(b[0], ly, float(blade_z(ly)) + BLADE_T)]))[0][2]))
        for o in COOKIES:
            if o != c and math.hypot(*(st[o].pos[:2] - s.pos[:2])) < 2 * COOKIE_R:
                under.append(st[o].pos[2] + COOKIE_H)
        def gap(z0):
            below = [z for z in under if z <= z0 + 0.005]
            return z0 - (max(below) if below else TOP)

        return min(gap(centre), gap(rim))

    def step(self, env):
        self.steps += 1
        tips = m.fingertips(env)
        st = {n: m.state(env, n) for n in ("pan", "rack", "turner", *COOKIES)}
        for c in COOKIES:
            s = st[c]
            if self._cookie_touch(s, tips):
                self.touched[c] = True
            if s.rotation()[2, 2] < 0.0:
                self.flipped[c] = True
        for c in COOKIES:  # with every cookie's state known: falling, and well above whatever is under it
            s = st[c]
            gap = self._support_gap(c, st)
            vz = self._point_vz(s)
            if vz < -DROP_SPEED and self._over_blade(s, st["turner"]):
                vz = self._point_vz(s, st["turner"])  # riding on the blade: only a fall relative to the blade counts
            self.max_pt[c] = max(self.max_pt[c], -vz)
            if vz < -DROP_SPEED or (s.lin_vel[2] < -0.05 and gap > DROP_GAP_MM / 1000.0):
                self.dropped[c] = True
            if gap <= REST_GAP_MM / 1000.0:
                # Resting on something now. A fall shorter than one control step (50 ms) is seen only as a cookie
                # that rested at one height and next rests well below it.
                # Heights are of the cookie's lowest point, so a cookie leaning on a rim that then settles flat does
                # not count as falling.
                r = s.rotation()
                low = float(s.pos[2] + min(0.0, COOKIE_H * r[2, 2]) - COOKIE_R * math.hypot(r[2, 0], r[2, 1]))
                if self.rest_z[c] is not None and self.rest_z[c] - low > DROP_GAP_MM / 1000.0:
                    self.dropped[c] = True
                self.rest_z[c] = low
        pan = st["pan"]
        if pan.pos[2] - TOP > PAN_LIFT_MM / 1000.0 or pan.tilt_deg() > PAN_TILT_DEG:
            self.pan_harm = True
        self.states = st
        self.score, ok, info = self.evaluate(st, tips)
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if DEBUG and self.steps % 10 == 1:
            print("[cookie] " + repr({"t": round(self.steps / 20, 2), **info}), flush=True)

    def cookie_status(self, c: str, st, tips) -> str:
        """good, spoiled or left."""
        s = st[c]
        if self.touched[c] or self.dropped[c] or self.flipped[c]:
            return "spoiled"
        rack, pan, turner = st["rack"], st["pan"], st["turner"]
        on_rack = rack.to_local(s.pos[None])[0]
        on_pan = pan.to_local(s.pos[None])[0]
        inside_rack = abs(on_rack[0]) <= RACK_SIZE[0] / 2 - COOKIE_R and abs(on_rack[1]) <= RACK_SIZE[1] / 2 - COOKIE_R
        inside_pan = abs(on_pan[0]) <= PAN_IN[0] / 2 + 0.005 and abs(on_pan[1]) <= PAN_IN[1] / 2 + 0.005
        tilt = s.tilt_deg()
        if inside_rack and -0.005 <= on_rack[2] - RACK_TOP <= 0.03:
            # On the rack: good only if resting flat on the wires, still, and clear of the turner's blade.
            seated = abs(on_rack[2] - RACK_TOP) <= SEATED_MM / 1000.0
            blade = turner.to_local(s.pos[None])[0]
            on_blade = abs(blade[0]) <= BLADE_HALF + COOKIE_R and BLADE_Y0 - COOKIE_R <= blade[1] <= BLADE_Y1 + COOKIE_R and abs(blade[2]) < 0.03
            still = float(np.linalg.norm(s.lin_vel)) < 0.01 and float(np.linalg.norm(s.ang_vel)) < 0.5
            return "good" if seated and tilt <= GOOD_TILT and still and not on_blade else "left"
        on_pan_tilt = math.degrees(math.acos(float(np.clip(s.rotation()[:, 2] @ pan.rotation()[:, 2], -1.0, 1.0))))
        if inside_pan and on_pan[2] <= PAN_FLOOR + 0.005 and on_pan_tilt <= GOOD_TILT:
            return "left"  # still lying on the pan's floor
        if (inside_rack and on_rack[2] > RACK_TOP - 0.005) or (inside_pan and on_pan[2] > PAN_FLOOR) or s.pos[2] - TOP > 0.025:
            # Above the pan or rack, or well above the table: on the turner, on another cookie, leaning on a rim, being carried.
            return "left"
        return "spoiled"  # on the table, under the rack, anywhere else

    def evaluate(self, st, tips):
        status = {c: self.cookie_status(c, st, tips) for c in COOKIES}
        good = sum(v == "good" for v in status.values())
        spoiled = sum(v == "spoiled" for v in status.values())
        base = max(0.0, 0.2 * good - 0.1 * spoiled)
        pan, turner, rack = st["pan"], st["turner"], st["rack"]
        pan_shift = float(np.linalg.norm(pan.pos[:2] - PAN_POS[:2])) * 1000.0
        pan_turn = abs(_yaw_deg(pan.rotation()))
        turner_down = turner.pos[2] - TOP < 0.01 and turner.tilt_deg() < 10.0 and float(np.linalg.norm(turner.lin_vel)) < 0.01
        # Turner: the distance to boxes enclosing its blade, neck and handle (never more than the true distance).
        turner_gap = self._turner_gap(turner, tips)
        # Cookies: distance to the underside's centre less the radius (never more than the true distance).
        cookie_gap = min(float(np.min(np.linalg.norm(tips - st[c].pos, axis=1))) for c in COOKIES) - COOKIE_R
        rack_local = rack.to_local(tips)
        rack_gap = float(np.min(np.maximum(np.abs(rack_local[:, 0]) - RACK_SIZE[0] / 2, 0) ** 2 + np.maximum(np.abs(rack_local[:, 1]) - RACK_SIZE[1] / 2, 0) ** 2
                                + np.maximum(rack_local[:, 2] - RACK_TOP, 0) ** 2) ** 0.5)
        clear = min(turner_gap, cookie_gap, rack_gap) >= CLEAR_M
        ok = (good == len(COOKIES) and pan_shift <= PAN_SHIFT_MM and pan_turn <= PAN_TURN_DEG and pan.tilt_deg() <= PAN_TILT_DEG
              and turner_down and clear and not self.pan_harm)
        score = 0.0 if self.pan_harm else (1.0 if ok and self.hold + 1 >= HOLD_STEPS else base)
        info = {"status": status, "base": round(base, 2), "pan_mm": round(pan_shift, 1), "pan_deg": round(pan_turn, 1), "turner_down": bool(turner_down),
                "gaps_mm": [round(turner_gap * 1000), round(cookie_gap * 1000), round(rack_gap * 1000)], "pan_harm": self.pan_harm,
                "touched": [c for c in COOKIES if self.touched[c]], "dropped": [c for c in COOKIES if self.dropped[c]],
                "turner": [round(v, 4) for v in turner.pos.tolist()] + [round(turner.tilt_deg(), 1)],
                "cookies": {c: [round(v, 4) for v in st[c].pos.tolist()] + [round(st[c].tilt_deg(), 1)] for c in COOKIES}}
        if DEBUG:
            tl = turner.to_local(tips[12:])
            info["tips_in_turner"] = np.round(tl[[0, 1, 6, 7]], 4).tolist()
            info["turner_quat"] = np.round(turner.quat_xyzw, 3).tolist()
            info["cookie_vz"] = {c: round(float(st[c].lin_vel[2]), 3) for c in COOKIES}
            info["rack"] = [round(v, 4) for v in rack.pos.tolist()] + [round(_yaw_deg(rack.rotation()), 1)]
            info["max_pt_vz"] = {c: round(self.max_pt[c], 3) for c in COOKIES}
            info["gap_mm"] = {c: round(float(self._support_gap(c, st)) * 1000, 1) for c in COOKIES}
        return float(score), bool(ok), info


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)
SUCCESS_HOLD_S = HOLD_STEPS / 20.0  # the episode ends once every full-credit condition has held for 1 s together


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    # True only while every full-credit condition holds at this step (never remembered: a later touch or a moved
    # turner ends it at once).
    return bool(TRIAL.hold >= 1)


# -- the scripted demo (open loop, from the published starting layout) -------------------------------------------------
R_TURNER0 = rot_z(TURNER_YAW)
BLADE_PITCH0 = math.degrees(math.atan(BLADE_SLOPE))  # the blade's own rise towards its front edge, in the turner's frame
REACH = 0.028  # the blade goes in until its edge is this far past where the cookie's centre was (the blade shoves it 1 to 2 cm first)
DEP_Z = -0.001  # the blade's edge is set on the wires' top, barely pressed (pressing harder drags the loose rack)
DEP_DEG = 20.0  # and tipped this far nose down about it
DEPOSIT = 0.010  # and is set on the wires with its edge this far past where the cookie should land
GRIP_LOCAL = np.array([0.0, GRIP_Y, 0.0105])  # the TCP, in the turner's frame, when it holds the handle
R_GRASP = R_TURNER0.T @ d.down(0.0).rot  # the TCP's rotation in the turner's frame (fingers across the handle)
TIP_LOCAL = np.array([0.0, BLADE_Y1, float(blade_z(BLADE_Y1))])


def turner_pose(tip, nose_down_deg: float):
    """The turner's rotation and origin that put its blade's front edge (underside) at `tip`, the blade's underside
    pitched `nose_down_deg` below level, its length along +x."""
    r = rot_y(nose_down_deg + BLADE_PITCH0) @ R_TURNER0
    return r, np.asarray(tip, float) - r @ TIP_LOCAL


def tcp_for(tip, nose_down_deg: float, label: str, t: float, finger=FINGER_CLOSED) -> Waypoint:
    r, o = turner_pose(tip, nose_down_deg)
    return Waypoint(t, tuple(float(v) for v in (o + r @ GRIP_LOCAL)), finger, label, d._as_grip(r @ R_GRASP))


def demo():
    """Right arm only. Pick the turner up by its handle; for each cookie, set the blade's edge on the pan's floor just
    short of it, nose down, and push it in under the cookie while levelling the blade; lift it level, carry it over the
    rim to the rack, set the blade on the wires, tip it nose down and draw it back so the cookie stays on the wires;
    then lay the turner back where it was and move clear."""
    floor = TOP + PAN_FLOOR
    wires = TOP + RACK_TOP
    r0, o0 = R_TURNER0, TURNER_POS + np.array([0.0, 0.0, 0.002])  # as it settles
    grip0 = o0 + r0 @ GRIP_LOCAL
    down = d.down(0.0)
    plan = [
        Waypoint(2.0, (grip0[0], grip0[1], grip0[2] + 0.10), FINGER_OPEN, "above the handle", down),
        Waypoint(1.5, tuple(grip0), FINGER_OPEN, "down to the handle", down),
        Waypoint(1.0, tuple(grip0), FINGER_CLOSED, "grip the handle", down),
    ]
    tip0 = o0 + r0 @ TIP_LOCAL
    plan.append(tcp_for((tip0[0], tip0[1], tip0[2] + 0.08), 0.0, "lift the turner", 1.5))
    order = ("cookie_1", "cookie_2", "cookie_3", "cookie_4")
    for c in order:
        cx, cy = PAN_POS[0] + COOKIE_AT[c][0], PAN_POS[1] + COOKIE_AT[c][1]
        edge = cx - COOKIE_R
        rim = PAN_POS[0] - PAN_IN[0] / 2  # the near wall's foot
        entry = max(8.0, math.degrees(math.asin(0.040 / (edge - 0.012 - rim))))  # steep enough for the neck to clear the rim
        plan += [
            tcp_for((edge - 0.020, cy, floor + 0.05), entry, f"{c}: over the pan", 2.0),
            tcp_for((edge - 0.012, cy, floor - 0.002), entry, f"{c}: blade edge on the floor", 1.5),
            tcp_for((edge + 0.020, cy, floor - 0.002), 8.0, f"{c}: under the cookie's edge", 1.5),
            tcp_for((cx + REACH, cy, floor - 0.001), 8.0, f"{c}: blade under the cookie", 2.0),
            tcp_for((cx + REACH, cy, floor + 0.015), 8.0, f"{c}: lift level", 1.0),
            tcp_for((cx + REACH, cy, floor + 0.060), 0.0, f"{c}: up over the rim", 1.0),
        ]
        rx, ry = RACK_POS[0] + RACK_AT[c][0], RACK_POS[1] + RACK_AT[c][1]
        plan += [
            tcp_for((rx + DEPOSIT, ry, wires + 0.050), 0.0, f"{c}: carry to the rack", 2.5),
            tcp_for((rx + DEPOSIT, ry, wires + 0.003), 0.0, f"{c}: blade down on the wires", 1.5),
            tcp_for((rx + DEPOSIT, ry, wires + DEP_Z), DEP_DEG, f"{c}: tip it nose down on its edge", 1.5),
            tcp_for((rx - 0.035, ry, wires + DEP_Z), DEP_DEG, f"{c}: draw the blade out", 2.5),
            tcp_for((rx - 0.050, ry, wires + 0.050), 0.0, f"{c}: blade up", 1.0),
            # The handle can pitch a little between the pads, so the blade may hang nose down: lift well clear of the
            # rack and its cookies before moving sideways.
            tcp_for((rx + 0.030, ry, wires + 0.130), 0.0, f"{c}: well clear", 1.0),
        ]
    plan += [
        tcp_for((tip0[0], tip0[1], tip0[2] + 0.13), 0.0, "back over its place", 2.5),
        tcp_for((tip0[0], tip0[1], tip0[2] + 0.03), 0.0, "lower the turner", 1.5),
        tcp_for((tip0[0], tip0[1], tip0[2] + 0.004), 0.0, "lay the turner down", 1.5),
        Waypoint(1.0, tuple(grip0 + np.array([0.0, 0.0, 0.004])), FINGER_OPEN, "let go", down),
        Waypoint(1.5, (grip0[0], grip0[1], grip0[2] + 0.15), FINGER_OPEN, "clear", down),
        Waypoint(4.0, (grip0[0], grip0[1], grip0[2] + 0.15), FINGER_OPEN, "wait", down),
    ]
    return {"right": plan}
