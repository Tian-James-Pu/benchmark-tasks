"""A round stainless pedal bin on the bench, its plastic inner bucket put back turned a quarter turn, so the lid won't shut.

The bin: a 25 cm stainless body, 27.5 cm to its rim, on a black plastic base, with a domed lid hinged at the back and a
pedal at the front that opens the lid. Inside the back wall, 9 mm below the rim, is the steel guide of the lid's lift
rod, standing 11 mm out of the wall. The grey polypropylene inner bucket (22.9 cm across its rim, 25 cm tall) has a
4 x 2 cm notch in its rim that has to sit round that guide. It was dropped back in a quarter turn out, notch to the
left, so its rim rests on the guide, about 11 mm proud at the back, and the lid rests on the rim, about 10 degrees
open. Its wire bail lies folded flat across its top.

Frames: the bin's origin is the middle of its bottom on the bench, x toward its back (away from the arms), y left.
The bucket's origin is the middle of its bottom, x toward its notch, its bail pivots on its y axis.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
import yam_arena.demo as _harness_demo  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena import station  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"

# -- dimensions (m) --------------------------------------------------------------------------------------------------
BIN_XY = (0.32, 0.03)
R_OUT, R_BORE = 0.125, 0.1185          # body outside, bore (inside of the steel wall)
Z_FLOOR, Z_RIM = 0.020, 0.275          # inner floor and rim heights above the bench
BIN_MASS = 2.2
# lift-rod guide on the inside of the back wall: 20 mm wide, 11 mm out of the wall, 14.5 mm tall, top 9 mm under the rim
GUIDE_IN, GUIDE_W, GUIDE_Z = 0.1075, 0.020, (0.2515, 0.2660)
# lid: 25.6 cm dome on a hinge 15 cm behind the bin's axis, 8 mm over the rim; underside on the rim when shut
LID_R, LID_DOME = 0.128, 0.031
HINGE = (0.150, 0.0, 0.283)
LID_MASS, LID_MAX_DEG = 0.40, 85.0
LID_AUTHORED_DEG = 14.0                # the lid is modelled (joint zero) this far open; it settles onto the bucket's rim
LID_DAMP = 0.45                        # soft-close damper, N m s/rad: about 2 s from fully open to shut
LID_K = 6.0                            # the lift linkage while the pedal is down, N m/rad (pushes only, never pulls)
LID_FULL_DEG = 80.0                    # lid opening with the pedal fully down
# pedal: 9 x 5 cm plate, top 4 cm above the bench, on a hinge inside the base; 20 deg (about 15 mm at its middle) opens the lid fully
PEDAL_PIVOT = (-0.110, 0.0, 0.025)
PEDAL_FULL_DEG, PEDAL_STOP_DEG = 20.0, 25.0
PEDAL_K, PEDAL_PRELOAD = 1.1, 0.25     # N m/rad and rad: about 6 N to start it, 15 N fully down
# bucket: tapered wall, a stiff rim band, a notch through the band at its +x side
B_H = 0.250
B_R0, B_R1 = 0.102, 0.1055             # wall outside at the bottom and under the band
BAND_Z, BAND_R = 0.235, 0.1145         # band from BAND_Z to B_H, outside radius
WALL_T = 0.003
NOTCH_Z = 0.230                        # notch bottom; it is two 10-degree segments wide (4.0 cm at the band)
BUCKET_MASS = 0.30
# bail: 4 mm steel wire on pivots in the band, a 9 cm black grip 18 mm thick, 7 cm above the pivots
PIVOT_Z, PIVOT_R = 0.242, 0.1035
GRIP_H, GRIP_R, GRIP_HALF = 0.070, 0.009, 0.045
BAIL_MASS = 0.06
SEG = 36                               # 10-degree segments
NOTCH_SEGS = (35, 0)                   # the two segments either side of +x

# start: bucket turned +90 deg (notch to +y), its band just over the guide; bail folded flat toward the notch
START_YAW_DEG = 90.0
PROUD = GUIDE_Z[1] - (Z_FLOOR + BAND_Z)  # 11 mm
BUCKET_SPAWN_Z = TOP + Z_FLOOR + PROUD + 0.002

# scoring
SEAT_DZ, SEAT_DXY, SEAT_TILT, NOTCH_TOL = 0.003, 0.008, 2.0, 15.0
SHUT_DEG = 0.2  # at most about 1 mm front-edge clearance from the closed stop
LOST_Z = TOP - 0.05
STILL_V, STILL_DEG_S, STILL_STEPS = 0.01, 2.0, 20
CLEAR = 0.03

STEEL = m.Look(m.rgb(0.80, 0.80, 0.79), metallic=0.85, roughness=0.32)
LOOKS = {
    "steel": STEEL,
    "base": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.6),
    "rubber": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.9),
    "pp": m.Look(m.rgb(0.55, 0.57, 0.58), roughness=0.55),
    "grip": m.Look(m.rgb(0.08, 0.08, 0.09), roughness=0.5),
    "wire": m.Look(m.rgb(0.72, 0.72, 0.70), metallic=0.8, roughness=0.35),
    "guide": m.Look(m.rgb(0.62, 0.62, 0.60), metallic=0.8, roughness=0.4),
}


# -- mesh helpers ------------------------------------------------------------------------------------------------------


def _outward(mesh: m.Mesh) -> m.Mesh:
    """Flip any face whose normal points toward the mesh's centroid (each piece here is near-convex)."""
    v, f = mesh.v, mesh.f.copy()
    c = v.mean(axis=0)
    for i, (a, b, d) in enumerate(f):
        n = np.cross(v[b] - v[a], v[d] - v[a])
        if np.dot(n, (v[a] + v[b] + v[d]) / 3 - c) < 0:
            f[i] = (a, d, b)
    return m.Mesh(v, f)


def ring_seg(a0: float, a1: float, z0: float, z1: float, ri0: float, ro0: float, ri1: float, ro1: float, n: int = 3) -> m.Mesh:
    """A closed piece of a (tapered) ring between angles a0..a1 (rad), radii ri/ro at z0 (…0) and z1 (…1)."""
    loops = []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        c, s = math.cos(a), math.sin(a)
        loops.append([(ro0 * c, ro0 * s, z0), (ro1 * c, ro1 * s, z1), (ri1 * c, ri1 * s, z1), (ri0 * c, ri0 * s, z0)])
    v = np.array([p for loop in loops for p in loop], float)
    f = []
    for k in range(n):
        for i in range(4):
            j = (i + 1) % 4
            p, q, r, s_ = 4 * k + i, 4 * k + j, 4 * (k + 1) + j, 4 * (k + 1) + i
            f += [(p, q, r), (p, r, s_)]
    last = 4 * n
    f += [(0, 1, 2), (0, 2, 3), (last, last + 2, last + 1), (last, last + 3, last + 2)]
    return _outward(m.Mesh(v, np.array(f)))


def _about_y(mesh: m.Mesh, deg: float, at) -> m.Mesh:
    """Turn a mesh `deg` about the y axis through `at` (positive lifts the -x side)."""
    a = np.asarray(at, float)
    return mesh.moved(-a).turned(deg, "y").moved(a)


def _pt_about_y(p, deg: float, at) -> tuple:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = np.asarray(p, float) - np.asarray(at, float)
    return tuple(float(v) for v in np.array([c * x + s * z, y, -s * x + c * z]) + np.asarray(at, float))


def _seg_angles(i: int) -> tuple[float, float]:
    a = 2 * math.pi / SEG
    return i * a, (i + 1) * a  # segments 35 and 0 straddle +x symmetrically


def _r_wall(z: float) -> float:
    return B_R0 + (B_R1 - B_R0) * z / BAND_Z


# -- the bin: body (frame), lid and pedal (links) ---------------------------------------------------------------------


def _bin() -> m.Part:
    fp, fc = [], []
    # visual: black base with rubber foot, stainless wall with a rolled rim
    fp.append(("foot", m.lathe([(0.0, 0.0), (0.1240, 0.0), (0.1250, 0.006), (0.0, 0.006)], 48), "rubber"))
    fp.append(("base", m.lathe([(0.0, 0.006), (0.1255, 0.006), (0.1258, 0.045), (0.1190, 0.045), (0.1190, Z_FLOOR), (0.0, Z_FLOOR)], 48), "base"))
    wall = [(R_BORE, 0.044), (R_OUT, 0.044), (R_OUT, Z_RIM - 0.007), (0.1262, Z_RIM - 0.004), (0.1262, Z_RIM - 0.001), (0.1235, Z_RIM), (R_BORE, Z_RIM), (R_BORE, 0.044)]
    fp.append(("wall", m.lathe(wall, 72), "steel"))
    # lift-rod guide and the rod running down the back wall to the base
    gx = (GUIDE_IN + R_BORE + 0.002) / 2
    gsize = (R_BORE + 0.002 - GUIDE_IN, GUIDE_W, GUIDE_Z[1] - GUIDE_Z[0])
    gc = (gx, 0.0, sum(GUIDE_Z) / 2)
    fp.append(("guide", m.box(gsize, gc), "guide"))
    fp.append(("rod", m.cylinder(0.003, GUIDE_Z[0] - Z_FLOOR, (0.1150, 0.0, (GUIDE_Z[0] + Z_FLOOR) / 2)), "guide"))
    fc.append(m.col_box("guide_c", gsize, gc))
    # hinge bracket behind the rim
    hb_size, hb_c = (0.036, 0.08, 0.020), (0.142, 0.0, Z_RIM - 0.010)
    fp.append(("hinge_bracket", m.box(hb_size, hb_c), "base"))
    fc.append(m.col_box("hinge_bracket_c", hb_size, hb_c))
    # pedal housing at the front of the base
    fp.append(("pedal_housing", m.box((0.03, 0.11, 0.030), (-0.122, 0.0, 0.021)), "base"))
    # colliders: solid base under the floor, and 36 wall pieces 6.5 mm thick
    fc.append(m.col_cylinder("floor_c", 0.124, Z_FLOOR, (0.0, 0.0, Z_FLOOR / 2)))
    for i in range(SEG):
        a0, a1 = _seg_angles(i)
        fc.append(m.col_hull(f"wall_c{i}", ring_seg(a0, a1, Z_FLOOR, Z_RIM, R_BORE, R_OUT, R_BORE, R_OUT, 1)))

    # lid: a stainless dome on a flat rim, with a knuckle round the hinge pin; modelled LID_AUTHORED_DEG open
    rs = (LID_R ** 2 + LID_DOME ** 2) / (2 * LID_DOME)
    z0 = Z_RIM + 0.004
    dome = [(0.0, Z_RIM), (LID_R, Z_RIM), (LID_R, z0)]
    for k in range(1, 9):
        r = LID_R * (1 - k / 9)
        dome.append((r, z0 + math.sqrt(rs * rs - r * r) - (rs - LID_DOME)))
    dome.append((0.0, z0 + LID_DOME))
    knuckle_size, knuckle_c = (0.040, 0.06, 0.014), (0.140, 0.0, HINGE[2])
    tilt = LID_AUTHORED_DEG
    lid = m.hinge("lid", "Y", HINGE, -LID_AUTHORED_DEG, LID_MAX_DEG - LID_AUTHORED_DEG,
                  [("dome", _about_y(m.lathe(dome, 72), tilt, HINGE), "steel"), ("knuckle", _about_y(m.box(knuckle_size, knuckle_c), tilt, HINGE), "base")],
                  [m.col_hull("lid_c", _about_y(m.lathe(dome, 24), tilt, HINGE)), m.col_hull("knuckle_c", _about_y(m.box(knuckle_size, knuckle_c), tilt, HINGE))],
                  mass=LID_MASS, spring=0.0, damping=LID_DAMP, friction=0.01, gravity=True,
                  com=_pt_about_y((0.0, 0.0, Z_RIM + 0.012), tilt, HINGE), inertia=(0.0017, 0.0017, 0.0033))
    # pedal: a black plate on an arm into the base
    plate_size, plate_c = (0.050, 0.090, 0.010), (-0.150, 0.0, 0.035)
    pedal = m.hinge("pedal", "Y", PEDAL_PIVOT, -PEDAL_STOP_DEG, 0.0,
                    [("plate", m.box(plate_size, plate_c), "base"), ("arm", m.box((0.03, 0.03, 0.008), (-0.118, 0.0, 0.029)), "steel")],
                    [m.col_box("plate_c", plate_size, plate_c)],
                    mass=0.12, spring=PEDAL_K, damping=0.05, friction=0.005)
    return m.mechanism("bin", (BIN_XY[0], BIN_XY[1], TOP), [lid, pedal], LOOKS, frame_parts=fp, frame_colliders=fc,
                       friction=(0.8, 0.6), free_mass=BIN_MASS, frame_inertia=(0.030, 0.030, 0.032), solver_iters=48)


# -- the inner bucket (frame) and its bail (link) ---------------------------------------------------------------------


def _bucket() -> m.Part:
    fp, fc = [], []
    for i in range(SEG):
        a0, a1 = _seg_angles(i)
        top = NOTCH_Z if i in NOTCH_SEGS else BAND_Z
        rt = _r_wall(top)
        fp.append((f"wall{i}", ring_seg(a0, a1, 0.0, top, B_R0 - WALL_T, B_R0, rt - WALL_T, rt, 3), "pp"))
        fc.append(m.col_hull(f"wall_c{i}", ring_seg(a0, a1, 0.0, top, B_R0 - 0.005, B_R0, rt - 0.005, rt, 1)))
        if i in NOTCH_SEGS:
            continue
        fp.append((f"band{i}", ring_seg(a0, a1, BAND_Z, B_H, B_R1 - WALL_T, BAND_R, B_R1 - WALL_T, BAND_R, 3), "pp"))
        fc.append(m.col_hull(f"band_c{i}", ring_seg(a0, a1, BAND_Z, B_H, B_R1 - WALL_T, BAND_R, B_R1 - WALL_T, BAND_R, 1)))
    fp.append(("bottom", m.cylinder(B_R0, 0.004, (0.0, 0.0, 0.002), segments=48), "pp"))
    fc.append(m.col_cylinder("bottom_c", B_R0, 0.004, (0.0, 0.0, 0.002)))
    for name, sy in (("boss_n", -1.0), ("boss_p", 1.0)):  # pivot bosses on the band's inside
        fp.append((name, m.cylinder(0.006, 0.006, (0.0, sy * (B_R1 - WALL_T - 0.002), PIVOT_Z), axis="y"), "pp"))
    # bail, modelled folded flat toward +x (joint zero); -90 deg stands it up, -180 folds it flat toward -x
    gz, piv = PIVOT_Z + GRIP_H, (0.0, 0.0, PIVOT_Z)
    wire = m.tube([(0.0, -PIVOT_R, PIVOT_Z), (0.0, -GRIP_HALF - 0.004, gz), (0.0, GRIP_HALF + 0.004, gz), (0.0, PIVOT_R, PIVOT_Z)], 0.002, 8)
    grip = m.cylinder(GRIP_R, 2 * GRIP_HALF, (0.0, 0.0, gz), axis="y", segments=20)
    cols = [m.col_hull("grip_c", _about_y(m.cylinder(GRIP_R, 2 * GRIP_HALF, (0.0, 0.0, gz), axis="y", segments=12), 90.0, piv))]
    for name, sy in (("leg_c_n", -1.0), ("leg_c_p", 1.0)):
        cols.append(m.col_hull(name, _about_y(m.tube([(0.0, sy * PIVOT_R, PIVOT_Z), (0.0, sy * (GRIP_HALF + 0.004), gz)], 0.0025, 8), 90.0, piv)))
    bail = m.hinge("bail", "Y", piv, -180.0, 0.0, [("wire", _about_y(wire, 90.0, piv), "wire"), ("grip", _about_y(grip, 90.0, piv), "grip")], cols,
                   mass=BAIL_MASS, spring=0.0, damping=0.002, friction=0.004, gravity=True, com=_pt_about_y((0.0, 0.0, PIVOT_Z + 0.044), 90.0, piv),
                   inertia=(1.0e-4, 1.2e-4, 2.0e-4))
    return m.mechanism("bucket", (BIN_XY[0], BIN_XY[1], BUCKET_SPAWN_Z), [bail], LOOKS, frame_parts=fp, frame_colliders=fc,
                       friction=(0.4, 0.3), free_mass=BUCKET_MASS, yaw_deg=START_YAW_DEG,
                       frame_inertia=(0.0032, 0.0032, 0.0034), solver_iters=48)


BIN = _bin()
BUCKET = _bucket()
GLOVES = SimReady("gloves", pos=(0.50, -0.30, TOP), size=(0.178, 0.117, 0.056), mass=0.35, package="box_glove_a01",
                  usd="sm_box_glove_a01_01.usd", yaw=math.radians(20.0))
SPRAY = SimReady("disinfectant", pos=(0.30, -0.25, TOP), size=(0.045, 0.067, 0.191), mass=0.50, package="disinfectant_a01",
                 usd="sm_disinfectant_a01_01.usd", yaw=math.radians(-30.0))
OBJECTS = [BIN, BUCKET, GLOVES, SPRAY]


def lid_deg(bin_state) -> float:
    """The lid's opening from shut, in degrees."""
    return bin_state.deg("lid") + LID_AUTHORED_DEG


def bail_deg(bucket_state) -> float:
    """The bail from upright, degrees: +90 folded flat toward the notch, -90 folded flat away from it."""
    return bucket_state.deg("bail") + 90.0


# -- the pedal-to-lid linkage: the lift rod pushes the lid open as far as the pedal sends it, and never holds it down ----

_LID = {"k": None}


def _linkage(env) -> None:
    s = m.state(env, "bin")
    frac = min(max(-s.joint("pedal") / math.radians(PEDAL_FULL_DEG), 0.0), 1.0)
    cmd = math.radians(LID_FULL_DEG * frac - LID_AUTHORED_DEG)
    k = LID_K if s.joint("lid") < cmd else 0.0
    asset, j = m._joint(env, "bin", "lid")
    if k != _LID["k"]:
        asset.write_joint_stiffness_to_sim(m._tensor(asset, k), joint_ids=[j])
        _LID["k"] = k
    asset.set_joint_position_target(m._tensor(asset, cmd), joint_ids=[j])
    m._TOUCHED.add(("bin", "lid"))  # the kit gives the lid back its own (zero) spring before every reset


def _linkage_reset(env) -> None:
    _LID["k"] = None
    m.set_target(env, "bin", "pedal", PEDAL_PRELOAD)  # the pedal's return spring is preloaded against its top stop


m.on_reset(_linkage_reset)
m.on_step(_linkage)


# -- scoring -----------------------------------------------------------------------------------------------------------


def _cyl_dist(p: np.ndarray, r: float, z0: float, z1: float) -> np.ndarray:
    """Distance of points (n, 3), in a frame whose z axis is the cylinder's, to a solid cylinder."""
    dr = np.maximum(np.hypot(p[:, 0], p[:, 1]) - r, 0.0)
    dz = np.maximum(np.maximum(p[:, 2] - z1, z0 - p[:, 2]), 0.0)
    return np.hypot(dr, dz)


def _box_dist(p: np.ndarray, lo, hi) -> np.ndarray:
    d = np.maximum(np.maximum(np.asarray(lo) - p, p - np.asarray(hi)), 0.0)
    return np.linalg.norm(d, axis=1)


def clearance(tips: np.ndarray, b, k, lid: float) -> float:
    """The closest any finger point comes to the bin (body, lid as it stands, hinge, pedal) or the bucket, m."""
    p = b.to_local(tips)
    d = [_cyl_dist(p, R_OUT + 0.002, 0.0, Z_RIM), _box_dist(p, (-0.180, -0.050, 0.0), (-0.120, 0.050, 0.046)),
         _box_dist(p, (0.118, -0.045, Z_RIM - 0.022), (0.162, 0.045, HINGE[2] + 0.010))]
    c, s = math.cos(math.radians(-lid)), math.sin(math.radians(-lid))
    h = np.asarray(HINGE)
    q = p - h
    q = np.c_[c * q[:, 0] + s * q[:, 2], q[:, 1], -s * q[:, 0] + c * q[:, 2]] + h  # into the shut lid's pose
    d.append(_cyl_dist(q, LID_R, Z_RIM, Z_RIM + 0.004 + LID_DOME))
    d.append(_cyl_dist(k.to_local(tips), BAND_R, 0.0, PIVOT_Z + GRIP_H + GRIP_R))
    return float(min(x.min() for x in d))


def seated_check(b, k) -> tuple[bool, dict]:
    rb = b.rotation()
    rel = rb.T @ k.rotation()
    off = rb.T @ (np.asarray(k.pos) - np.asarray(b.pos))
    tilt = math.degrees(math.acos(min(1.0, max(-1.0, rel[2, 2]))))
    notch = math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
    ok = abs(off[2] - Z_FLOOR) <= SEAT_DZ and math.hypot(off[0], off[1]) <= SEAT_DXY and tilt <= SEAT_TILT and abs(notch) <= NOTCH_TOL
    return bool(ok), {"dz": float(off[2] - Z_FLOOR), "dxy": float(math.hypot(off[0], off[1])), "tilt": tilt, "notch": notch}


class Trial:
    """Final physical state scoring; loss below the bench is latched until reset."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.harm, self.why = False, ""
        self.start_xy = None
        self.last_lid = None
        self.still = 0
        self.score, self.full = 0.0, False
        self.info: dict = {}

    def step(self, env) -> None:
        b, k = m.state(env, "bin"), m.state(env, "bucket")
        if self.start_xy is None:
            self.start_xy = np.asarray(b.pos[:2], float).copy()
        # A part that fell below the bench has been lost from this work area.
        # Harmless sliding, temporary tilt and ordinary lowering are valid alternatives.
        if min(float(b.pos[2]), float(k.pos[2])) < LOST_Z:
            self.harm, self.why = True, self.why or "part fell below bench"
        lid = lid_deg(b)
        rate = 0.0 if self.last_lid is None else abs(lid - self.last_lid) * 20.0
        self.last_lid = lid
        moving = (float(np.linalg.norm(k.lin_vel)) > STILL_V or float(np.linalg.norm(b.lin_vel)) > STILL_V
                  or math.degrees(float(np.linalg.norm(k.ang_vel))) > STILL_DEG_S
                  or math.degrees(float(np.linalg.norm(b.ang_vel))) > STILL_DEG_S or rate > STILL_DEG_S)
        self.still = 0 if moving else self.still + 1
        seated, sinfo = seated_check(b, k)
        shut = lid <= SHUT_DEG
        clear = clearance(m.fingertips(env), b, k, lid)
        ends_ok = clear >= CLEAR and self.still >= STILL_STEPS
        if self.harm or not ends_ok or not seated:
            self.score = 0.0
        else:
            self.score = 1.0 if shut else 0.5
        self.full = self.score == 1.0
        self.info = {"lid": lid, "seated": seated, **sinfo, "clear": clear, "still": self.still, "harm": self.why, "score": self.score}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- debug dump (temporary) ------------------------------------------------------------------------------------------
_DBG = {"t": 0, "rows": []}


def _debug(env) -> None:
    path = os.environ.get("BIN_DEBUG")
    if not path:
        return
    b, k = m.state(env, "bin"), m.state(env, "bucket")
    _DBG["t"] += 1
    _DBG["rows"].append({"t": _DBG["t"], "bin_pos": b.pos.tolist(), "bin_q": b.quat_xyzw.tolist(), "bin_j": b.joints,
                         "bk_pos": k.pos.tolist(), "bk_q": k.quat_xyzw.tolist(), "bk_j": k.joints,
                         "bk_v": k.lin_vel.tolist(), "tips": m.fingertips(env).tolist(), "sc": TRIAL.info})
    if _DBG["t"] % 20 == 0 or _DBG["t"] < 3:
        with open(path, "w") as f:
            json.dump(_DBG["rows"], f)


m.on_step(_debug)


# -- demo: a fixed plan from the published starting layout ---------------------------------------------------------------


class Grip(float):
    """A Waypoint yaw that carries a whole TCP rotation (the same device as the shared demokit.py)."""

    rot: np.ndarray = np.eye(3)


def _install_grips() -> None:
    if getattr(_harness_demo.grip_rotation, "_takes_grips", False):
        return
    plain = _harness_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return plain(yaw) if rot is None else rot

    grip_rotation._takes_grips = True
    grip_rotation._plain = plain
    _harness_demo.grip_rotation = grip_rotation


_install_grips()
_plain = _harness_demo.grip_rotation._plain


def tilted(theta_deg: float, phi_deg: float) -> Grip:
    """Tilted theta from straight down toward the direction phi from +x; the fingers close across that direction."""
    phi = math.radians(phi_deg)
    g = Grip(0.0)
    g.rot = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain(phi)
    return g


# settled start, measured in the simulator: the folded grip's centre and the bail's pivot height
GRIP0 = (BIN_XY[0], BIN_XY[1] + GRIP_H, TOP + Z_FLOOR + 0.0055 + PIVOT_Z)
OPEN_W, PINCH_W, RELEASE_W = -0.018, 0.0, -0.022   # finger targets (m per finger)
TH = 55.0                                            # the left grip's tilt from vertical


def _left() -> list[Waypoint]:
    x, y = BIN_XY
    ready = ArmPlan([], station.BASES["left"])
    ready_grip = Grip(0.0)
    ready_grip.rot = ready.start_rot
    gx, gy, gz = GRIP0
    pz = gz  # the pivot axis, through the folded grip's centre
    seat = TOP + Z_FLOOR + PIVOT_Z                    # the pivot axis with the bucket seated
    g0, g90 = tilted(TH, 0.0), tilted(TH, -90.0)
    w = [Waypoint(6.0, ready.start_pos, -0.045, "wait outside the lid swing", ready_grip),
         Waypoint(3.0, (gx, gy, gz + 0.07), OPEN_W, "over the folded grip", g0),
         Waypoint(1.5, (gx, gy, gz), OPEN_W, "down to the grip", g0),
         Waypoint(0.8, (gx, gy, gz), PINCH_W, "pinch", g0)]
    for a in (75, 60, 45, 30, 15, 0):  # stand the bail up about its pivots
        r = math.radians(a)
        w.append(Waypoint(0.5, (x, y + GRIP_H * math.sin(r), pz + GRIP_H * math.cos(r)), PINCH_W, "stand the bail", g0))
    top = pz + GRIP_H
    w += [Waypoint(1.5, (x, y, top + 0.020), PINCH_W, "lift off the guide", g0),
          Waypoint(4.0, (x, y, top + 0.020), PINCH_W, "turn a quarter, clockwise", g90),
          Waypoint(2.0, (x, y, seat + GRIP_H - 0.002), PINCH_W, "lower round the guide", g90)]
    for b in (15, 30, 45, 60, 75, 82):  # fold the bail toward the front
        r = math.radians(b)
        w.append(Waypoint(0.5, (x - GRIP_H * math.sin(r), y, seat + GRIP_H * math.cos(r)), PINCH_W, "fold the bail", g90))
    r = math.radians(82)
    fx, fz = x - GRIP_H * math.sin(r), seat + GRIP_H * math.cos(r)
    w += [Waypoint(0.7, (fx, y, fz), RELEASE_W, "let go", g90),
          Waypoint(1.2, (fx, y + 0.02, fz + 0.06), RELEASE_W, "up", g90),
          Waypoint(2.0, ready.start_pos, RELEASE_W, "clear of the lid", ready_grip)]
    return w


PRESS = tilted(30.0, 0.0)


def _right(t_release: float) -> list[Waypoint]:
    x, y = BIN_XY
    px = x - 0.165                                    # the pedal's front half
    tip = np.array([px, y, TOP + 0.003])              # 2.5 mm past where the pedal meets its stop
    tcp = tuple(tip - (0.013) * PRESS.rot[:, 2])
    over = (tcp[0], tcp[1], TOP + 0.12)
    w = [Waypoint(2.0, over, 0.0, "over the pedal", PRESS),
         Waypoint(3.0, tcp, 0.0, "press the pedal down", PRESS)]
    t = 5.0
    w.append(Waypoint(t_release - t, tcp, 0.0, "hold the lid open", PRESS))
    w += [Waypoint(1.0, over, 0.0, "let the pedal up", PRESS),
          Waypoint(8.0, over, 0.0, "watch the lid shut", PRESS)]
    return w


def demo():
    left = _left()
    t_left = sum(p.duration for p in left)
    return {"left": left, "right": _right(t_left + 0.5)}


def demo():
    right = ArmPlan([], station.BASES["right"])
    g = Grip(0.0)
    g.rot = right.start_rot
    push = tilted(55, 0)
    return {"left": [Waypoint(3, (.24, .03, TOP+.36), 0, "over lid", push),
                     Waypoint(3, (.24, .03, TOP+.27), 0, "push lid shut", push),
                     Waypoint(2, (.24, .03, TOP+.27), 0, "hold pressure", push),
                     Waypoint(2, (.24, .03, TOP+.36), 0, "release", push)],
            "right": [Waypoint(10, right.start_pos, -.045, "idle", g)]}
