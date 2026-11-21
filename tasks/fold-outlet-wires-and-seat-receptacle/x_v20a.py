"""Fold a receptacle's three solid copper conductors into its wall box, seat it flush and drive its two mounting screws.

A 15 A duplex receptacle hangs 12 cm out of a single-gang PVC box in a drywall wall, on three 14 AWG solid copper
conductors (black, white, bare) that run from the cable clamp at the back of the box to its terminals. A push-to-start
cordless screwdriver lies on the bench. Done: the receptacle flush on the wall, both mounting screws driven home,
every conductor inside the box, and both hands off it.

Files next to this one: conductors.py (the copper conductors), fastening.py (the screwdriver and the screws' threads),
mechkit.py and demokit.py (unchanged, see README), closedloop.py (plays demo_script under yam_scripted), simready.py.
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
import conductors as cu  # noqa: E402
import fastening as fx  # noqa: E402
import demokit as d  # noqa: E402
import closedloop  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0

# -- layout (station frame, m: x away from the arms, y left, z up) --------------------------------------------------
XW = 0.32                      # the drywall's face
YB, ZB = -0.04, TOP + 0.11     # the box opening's centre
WALL_H = 0.20
DRY_T = 0.0127                 # 1/2 in gypsum board
BOX_W, BOX_H, BOX_D, BOX_T = 0.0572, 0.0984, 0.0826, 0.003   # Carlon B120A outside: 2-1/4 x 3-7/8 x 3-1/4 in
SCREW_SP = 0.0833              # 3-9/32 in, device mounting screw spacing
# receptacle (Leviton T5320-W class: 1.31 x 0.98 x 4.2 in)
DEV_W, DEV_H, STRAP_T = 0.0333, 0.1067, 0.0009
BODY_H, BODY_D, FACE_D = 0.067, 0.0200, 0.0040
DEV_X0 = XW - 0.08             # the strap's back face at the start
DEV_Z0 = ZB + 0.005
SHANK_L = 0.017                # the captive screws stand this far out behind the strap
HOLE_HALF = 0.0026             # the boss's screw hole: half its width (a 6-32 shank is 3.5 mm across)

LOOKS = {
    "drywall": m.Look(m.rgb(0.90, 0.89, 0.86), roughness=0.9),
    "wood": m.Look(m.rgb(0.78, 0.64, 0.45), roughness=0.8),
    "box": m.Look(m.rgb(0.10, 0.30, 0.75), roughness=0.5),
    "ivory": m.Look(m.rgb(0.93, 0.91, 0.84), roughness=0.4),
    "slot": m.Look(m.rgb(0.15, 0.13, 0.12), roughness=0.6),
    "steel": m.Look(m.rgb(0.72, 0.72, 0.70), metallic=0.9, roughness=0.35),
    "brass": m.Look(m.rgb(0.78, 0.62, 0.30), metallic=0.9, roughness=0.35),
    "black": m.Look(m.rgb(0.05, 0.05, 0.05), roughness=0.5),
    "white": m.Look(m.rgb(0.92, 0.92, 0.90), roughness=0.5),
    "copper": m.Look(m.rgb(0.80, 0.45, 0.25), metallic=0.9, roughness=0.3),
    "sheath": m.Look(m.rgb(0.93, 0.88, 0.55), roughness=0.7),
    "toolgreen": m.Look(m.rgb(0.05, 0.35, 0.40), roughness=0.5),
    "toolgrey": m.Look(m.rgb(0.20, 0.21, 0.22), roughness=0.6),
    "chrome": m.Look(m.rgb(0.80, 0.80, 0.80), metallic=1.0, roughness=0.2),
}


def _boss(k: int, sz: int):
    """A screw boss inside the box at the top (sz 1) or bottom (sz -1): a block with a square hole on the screw's line,
    open to the front, with a 1.5 mm lead-in at its mouth. Returns (parts, colliders)."""
    zc = ZB + sz * SCREW_SP / 2
    zin = ZB + sz * (BOX_H / 2 - BOX_T)        # the inside of the box's top (or bottom) wall
    depth, side = 0.022, 0.003
    h = HOLE_HALF
    parts, cols = [], []
    pieces = [  # (size, centre): two cheeks beside the hole and a web on the box-centre side of it
        ((depth, side, abs(zin - (zc - sz * (h + side)))), (XW + depth / 2, YB - h - side / 2, (zin + zc - sz * (h + side)) / 2)),
        ((depth, side, abs(zin - (zc - sz * (h + side)))), (XW + depth / 2, YB + h + side / 2, (zin + zc - sz * (h + side)) / 2)),
        ((depth, 2 * (h + side), side), (XW + depth / 2, YB, zc - sz * (h + side / 2))),
    ]
    for j, (s, c) in enumerate(pieces):
        parts.append((f"boss{k}{j}", m.box(s, c), "box"))
        cols.append(m.col_box(f"boss{k}{j}_c", s, c))
    return parts, cols


def _wall():
    parts, cols = [], []
    y0, y1, z0, z1 = YB - 0.13, YB + 0.13, TOP, TOP + WALL_H
    by0, by1, bz0, bz1 = YB - BOX_W / 2, YB + BOX_W / 2, ZB - BOX_H / 2, ZB + BOX_H / 2
    xc = XW + DRY_T / 2
    pieces = [((y0, by0), (z0, z1)), ((by1, y1), (z0, z1)), ((by0, by1), (z0, bz0)), ((by0, by1), (bz1, z1))]
    for i, ((ya, yb), (za, zb)) in enumerate(pieces):
        s, c = (DRY_T, yb - ya, zb - za), (xc, (ya + yb) / 2, (za + zb) / 2)
        parts.append((f"dry{i}", m.box(s, c), "drywall"))
        cols.append(m.col_box(f"dry{i}_c", s, c))
    # 2x4 stud beside the box (the box is nailed to it), on a 2x4 plate
    s, c = (0.089, 0.038, WALL_H - 0.038), (XW + DRY_T + 0.0445, by0 - 0.019, TOP + 0.038 + (WALL_H - 0.038) / 2)
    parts.append(("stud", m.box(s, c), "wood"))
    cols.append(m.col_box("stud_c", s, c))
    s, c = (0.089, y1 - y0, 0.038), (XW + DRY_T + 0.0445, (y0 + y1) / 2, TOP + 0.019)
    parts.append(("plate", m.box(s, c), "wood"))
    cols.append(m.col_box("plate_c", s, c))
    # the box: five walls, open to the front, its rim flush with the drywall's face
    t, dd = BOX_T, BOX_D
    walls = [((dd, BOX_W, t), (XW + dd / 2, YB, bz1 - t / 2)), ((dd, BOX_W, t), (XW + dd / 2, YB, bz0 + t / 2)),
             ((dd, t, BOX_H), (XW + dd / 2, by0 + t / 2, ZB)), ((dd, t, BOX_H), (XW + dd / 2, by1 - t / 2, ZB)),
             ((t, BOX_W, BOX_H), (XW + dd - t / 2, YB, ZB))]
    for i, (s, c) in enumerate(walls):
        parts.append((f"box{i}", m.box(s, c), "box"))
        cols.append(m.col_box(f"box{i}_c", s, c))
    for k, sz in enumerate((1, -1)):
        p, c = _boss(k, sz)
        parts += p
        cols += c
    return m.fixed("wall", (0.0, 0.0, 0.0), parts, cols, LOOKS, friction=(0.6, 0.5))


WALL = _wall()

# terminals (station frame at the start) and the cable clamp's exits
T_BLACK = np.array([DEV_X0 + 0.008, YB - DEV_W / 2 - 0.0012, DEV_Z0 - 0.012])
T_WHITE = np.array([DEV_X0 + 0.008, YB + DEV_W / 2 + 0.0012, DEV_Z0 - 0.012])
T_BARE = np.array([DEV_X0 + 0.0025, YB + 0.008, DEV_Z0 - DEV_H / 2 + 0.004])
CLAMP = XW + BOX_D - 0.016, ZB - BOX_H / 2 + 0.0095
C_BLACK = np.array([CLAMP[0], YB - 0.0045, CLAMP[1]])
C_WHITE = np.array([CLAMP[0], YB + 0.0045, CLAMP[1]])
C_BARE = np.array([CLAMP[0] - 0.002, YB, CLAMP[1] + 0.0005])


def _line(a, b, n, sag=0.0):
    """A wire from a to b: a quadratic curve whose middle hangs `sag` below the straight line, `n` links of equal length."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = (a + b) / 2 - np.array([0.0, 0.0, 2 * sag])
    t = np.linspace(0, 1, 2001)
    pts = (1 - t)[:, None] ** 2 * a + 2 * ((1 - t) * t)[:, None] * c + t[:, None] ** 2 * b
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    return tuple(tuple(pts[np.argmin(np.abs(s - u))]) for u in np.linspace(0, s[-1], n + 1))


NSEG = 16
SAG = -0.06
AWG14 = 0.001628
WIRES = [cu.Wire("black", _line(C_BLACK, T_BLACK, NSEG, SAG), 0.00140, "black", copper_d=AWG14, mass_per_m=0.0225),
         cu.Wire("white", _line(C_WHITE, T_WHITE, NSEG, SAG), 0.00140, "white", copper_d=AWG14, mass_per_m=0.0225),
         cu.Wire("bare", _line(C_BARE, T_BARE, NSEG, SAG), 0.000814, "copper", copper_d=AWG14, mass_per_m=0.0186)]

SCREW_Z = (DEV_Z0 + SCREW_SP / 2, DEV_Z0 - SCREW_SP / 2)   # top, bottom (device frame)
HEAD_H = 0.0023


def _shank(zc):
    """A 6-32 screw's shank behind the strap with a pointed tip, as one convex hull."""
    x0 = DEV_X0
    body = m.cylinder(0.00175, SHANK_L - 0.003, (x0 + (SHANK_L - 0.003) / 2, YB, zc), axis="x", segments=10)
    tip = m.Mesh(np.array([[x0 + SHANK_L, YB, zc]]), np.zeros((0, 3), int))
    return m.Mesh(np.vstack([body.v, tip.v]), body.f)


def _device():
    x0, z0 = DEV_X0, DEV_Z0
    parts, cols = [], []
    s, c = (STRAP_T, 0.022, DEV_H), (x0 - STRAP_T / 2, YB, z0)
    parts.append(("strap", m.box(s, c), "steel"))
    cols.append(m.col_box("strap_c", s, c))
    s, c = (BODY_D, DEV_W, BODY_H), (x0 + BODY_D / 2, YB, z0)
    parts.append(("body", m.box(s, c), "ivory"))
    cols.append(m.col_box("body_c", s, c))
    s, c = (FACE_D, DEV_W - 0.002, BODY_H - 0.002), (x0 - STRAP_T - FACE_D / 2, YB, z0)
    parts.append(("face", m.box(s, c), "ivory"))
    cols.append(m.col_box("face_c", s, c))
    for k, dz in enumerate((0.0165, -0.0165)):
        xf = x0 - STRAP_T - FACE_D - 0.0003
        parts.append((f"hot{k}", m.box((0.0006, 0.0024, 0.0075), (xf, YB - 0.0064, z0 + dz + 0.002)), "slot"))
        parts.append((f"neu{k}", m.box((0.0006, 0.0024, 0.0095), (xf, YB + 0.0064, z0 + dz + 0.002)), "slot"))
        parts.append((f"gnd{k}", m.cylinder(0.0025, 0.0006, (xf, YB, z0 + dz - 0.0075), axis="x"), "slot"))
    for k, zc in enumerate(SCREW_Z):
        head = m.cylinder(0.0032, HEAD_H, (x0 - STRAP_T - HEAD_H / 2, YB, zc), axis="x")
        parts.append((f"head{k}", head, "steel"))
        parts.append((f"slot{k}a", m.box((0.0004, 0.0045, 0.0009), (x0 - STRAP_T - HEAD_H - 0.0001, YB, zc)), "slot"))
        parts.append((f"slot{k}b", m.box((0.0004, 0.0009, 0.0045), (x0 - STRAP_T - HEAD_H - 0.0001, YB, zc)), "slot"))
        parts.append((f"shank{k}", _shank(zc), "steel"))
        cols.append(m.col_hull(f"head{k}_c", m.cylinder(0.0032, HEAD_H, (x0 - STRAP_T - HEAD_H / 2, YB, zc), axis="x", segments=12)))
        cols.append(m.col_hull(f"shank{k}_c", _shank(zc)))
    for k, (t, look) in enumerate(((T_BLACK, "brass"), (T_WHITE, "steel"))):
        parts.append((f"term{k}", m.cylinder(0.0035, 0.002, (t[0], t[1] + (0.0008 if k else -0.0008), t[2]), axis="y"), look))
    return cu.DeviceSpec(tuple(parts), tuple(cols), mass=0.045, com=(x0 + 0.006, YB, z0), inertia=(4e-5, 4e-5, 1e-5))


SHEATH = [("sheath", m.tube([(CLAMP[0], YB, CLAMP[1] - 0.010), (CLAMP[0], YB, TOP + 0.040)], 0.0045), "sheath")]
LAW = cu.CopperLaw()
OUTLET = cu.wired_device("outlet", WIRES, _device(), LOOKS, frame_parts=SHEATH, law=LAW)
COPPER = cu.Copper("outlet", WIRES, LAW)

# -- the screwdriver: a push-to-start cordless driver (Bosch GO class: 190 x 40 x 40 mm, 0.31 kg) --------------------
DRV_POS = (0.240, -0.215, TOP + 0.020)     # its housing's centre, lying on the bench, pointing at the wall (+x)
DRV_L, DRV_W = 0.150, 0.040
NOSE_L, CHUCK_L, BIT_L = 0.018, 0.022, 0.025
SPINDLE_K, SPINDLE_TRAVEL = 2000.0, 0.004
_nose_x = DRV_L / 2
_chuck_x = _nose_x + NOSE_L
BIT_TIP = np.array([_chuck_x + CHUCK_L + BIT_L, 0.0, 0.0])   # in the driver's frame, spindle out
DRV_GRIP_X = 0.05              # the demo takes the driver this far forward of its housing's centre


def _driver():
    housing = [("housing", m.box((DRV_L, DRV_W, DRV_W), (0, 0, 0)), "toolgreen"),
               ("grip", m.box((0.07, DRV_W + 0.002, DRV_W + 0.002), (-0.02, 0, 0)), "toolgrey"),
               ("button", m.box((0.012, 0.010, 0.004), (0.035, 0, DRV_W / 2 + 0.001)), "toolgrey"),
               ("nose", m.cylinder(0.014, NOSE_L, (_nose_x + NOSE_L / 2, 0, 0), axis="x"), "toolgrey")]
    hcols = [m.col_box("housing_c", (DRV_L, DRV_W, DRV_W), (0, 0, 0)),
             m.col_hull("nose_c", m.cylinder(0.014, NOSE_L, (_nose_x + NOSE_L / 2, 0, 0), axis="x", segments=12))]
    chuck = m.cylinder(0.0075, CHUCK_L, (_chuck_x + CHUCK_L / 2, 0, 0), axis="x")
    bit = m.cylinder(0.0032, BIT_L, (_chuck_x + CHUCK_L + BIT_L / 2, 0, 0), axis="x", segments=12)
    spindle = m.slider("spindle", "X", -SPINDLE_TRAVEL, 0.0,
                       [("chuck", chuck, "chrome"), ("bit", bit, "steel")],
                       [m.col_hull("chuck_c", m.cylinder(0.0075, CHUCK_L - 0.002, (_chuck_x + CHUCK_L / 2 + 0.001, 0, 0), axis="x", segments=12)),
                        m.col_hull("bit_c", bit)],
                       spring=SPINDLE_K, damping=5.0, friction=0.0, effort=200.0, mass=0.02)
    return m.mechanism("driver", DRV_POS, [spindle], LOOKS, frame_parts=housing, frame_colliders=hcols, free_mass=0.31,
                       frame_inertia=(0.31 * 2 * DRV_W ** 2 / 12, 0.31 * (DRV_L ** 2 + DRV_W ** 2) / 12, 0.31 * (DRV_L ** 2 + DRV_W ** 2) / 12),
                       friction=(1.0, 0.8))


DRIVER = _driver()

# the threads: per screw, its strap-back point, tip and head-front point in the device's authored frame
SCREWS = fx.Screws("outlet",
                   strap_pts=[(DEV_X0, YB, z) for z in SCREW_Z],
                   tip_pts=[(DEV_X0 + SHANK_L, YB, z) for z in SCREW_Z],
                   head_pts=[(DEV_X0 - STRAP_T - HEAD_H, YB, z) for z in SCREW_Z],
                   holes=[(XW, YB, ZB + SCREW_SP / 2), (XW, YB, ZB - SCREW_SP / 2)],
                   wall_x=XW, driver="driver", bit_tip_local=BIT_TIP)

# -- distractors: a lineman's pliers and a long-nose pliers on the bench, left of the work ---------------------------
PLIERS_A = SimReady("lineman_pliers", pos=(0.06, 0.24, TOP), size=(0.189, 0.069, 0.026), mass=0.32, yaw=0.0,
                    package="handTool_pliers_combination_a01", usd="sm_handTool_pliers_combination_a01_01.usd")
PLIERS_B = SimReady("long_nose_pliers", pos=(0.20, 0.30, TOP), size=(0.235, 0.069, 0.024), mass=0.18, yaw=0.0,
                    package="handTool_pliers_long_nose_a01", usd="sm_handTool_pliers_long_nose_a01_01.usd")

OBJECTS = [WALL, OUTLET, DRIVER, PLIERS_A, PLIERS_B]
NAMES = ("wall", "outlet", "driver", "lineman_pliers", "long_nose_pliers")


# -- reading the scene (the scorer and the demo both use these) -------------------------------------------------------
def device_pose(env):
    return cu.poses(env, "outlet")["device"]


def dev_point(env, p):
    R, t = device_pose(env)
    return R @ np.asarray(p, float) + t


def wire_pts(env, per_seg=3):
    P = cu.poses(env, "outlet")
    return {w.name: cu.wire_points(P, w, per_seg) for w in WIRES}


LOG = os.environ.get("OUTLET_LOG")

# -- scoring thresholds (README, Scoring) --
FLUSH_TOL = 0.0010        # m: the strap's back at each screw at most this far off the wall's face
UPRIGHT_DEG = 3.0         # the receptacle's face within this of square to the wall
WIRE_FRONT_TOL = 0.0010   # m: no conductor's centreline more than this in front of the wall's face
HANDS_CLEAR = 0.03        # m: every fingertip point this far from the receptacle and from every conductor
BIT_CLEAR = 0.02          # m: the driver's bit this far from the receptacle
STILL_V = 0.005           # m/s
HOLD_STEPS = 20           # 1 s at 20 Hz


def _dev_samples():
    """Points over the receptacle's strap, face and body (device frame), for the hands-off check."""
    pts = []
    for z in np.linspace(DEV_Z0 - DEV_H / 2, DEV_Z0 + DEV_H / 2, 9):
        for y in (YB - 0.011, YB, YB + 0.011):
            pts.append((DEV_X0 - STRAP_T, y, z))
    for z in np.linspace(DEV_Z0 - BODY_H / 2, DEV_Z0 + BODY_H / 2, 5):
        for y in (YB - DEV_W / 2, YB + DEV_W / 2):
            for x in (DEV_X0 - STRAP_T - FACE_D, DEV_X0 + BODY_D):
                pts.append((x, y, z))
    return pts


DEV_SAMPLES = _dev_samples()


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        """After every reset: fresh wire, screws backed out to their starting place, no hold counted."""
        self.n, self.hold = 0, 0
        self.score, self.full = 0.0, False
        self.why = {}
        if env is not None:
            COPPER.reset(env)
            SCREWS.reset(env)

    def judge(self, env):
        """This step's level, from the end-state rules (see README, Scoring)."""
        R, p = device_pose(env)
        gaps = SCREWS.gaps(R, p)
        upright = float(np.degrees(np.arccos(np.clip(R[0, 0], -1.0, 1.0))))
        W = wire_pts(env)
        front = max(float(XW - v[:, 0].min()) for v in W.values())      # how far the furthest-out conductor stands in front of the wall
        inside = front <= WIRE_FRONT_TOL
        dev_pts = (np.asarray(DEV_SAMPLES) @ R.T) + p
        tips = m.fingertips(env)
        wire_all = np.vstack(list(W.values()))
        clear_dev = float(np.min(np.linalg.norm(tips[:, None, :] - dev_pts[None], axis=2)))
        clear_wire = float(np.min(np.linalg.norm(tips[:, None, :] - wire_all[None], axis=2)))
        bit, _, _ = SCREWS.bit(env)
        bit_off = float(np.min(np.linalg.norm(dev_pts - bit, axis=1)))
        hands_off = clear_dev >= HANDS_CLEAR and clear_wire >= HANDS_CLEAR and bit_off >= BIT_CLEAR
        _, vel = cu.device_velocity(env, "outlet")
        still = float(np.linalg.norm(vel)) < STILL_V
        home = SCREWS.home.copy()
        flush = bool(np.all(gaps <= FLUSH_TOL)) and upright <= UPRIGHT_DEG
        self.why = {"gaps_mm": (gaps * 1000).round(2).tolist(), "upright_deg": round(upright, 2), "front_mm": round(front * 1000, 2),
                    "clear_dev_cm": round(clear_dev * 100, 1), "clear_wire_cm": round(clear_wire * 100, 1), "bit_off_cm": round(bit_off * 100, 1),
                    "home": home.tolist(), "still": still}
        if not (inside and hands_off):
            return 0.0, False
        if home.all():
            return (1.0, still) if flush else (0.5, False)
        if home.any():
            return 0.25, False
        return 0.0, False

    def step(self, env):
        COPPER.step(env)
        SCREWS.step(env)
        self.n += 1
        level, full_now = self.judge(env)
        self.hold = self.hold + 1 if full_now else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else (0.5 if level == 1.0 else level)
        if LOG and self.n % 5 == 1:
            R, p = device_pose(env)
            W = wire_pts(env, 1)
            dvs = m.state(env, "driver")
            rec = {"n": self.n, "dev": (R @ np.array([DEV_X0, YB, DEV_Z0]) + p).round(4).tolist(),
                   "tilt": float(np.degrees(np.arccos(np.clip(R[0, 0], -1, 1)))),
                   "gaps": SCREWS.gaps(R, p).round(4).tolist(), "tips_in": SCREWS.tips_in_holes(R, p).tolist(),
                   "caught": SCREWS.caught.tolist(), "home": SCREWS.home.tolist(), "turns": SCREWS.turns.round(2).tolist(),
                   "drv": dvs.pos.round(4).tolist(), "drv_tilt": round(dvs.tilt_deg(), 1), "spindle": round(dvs.joint("spindle"), 4),
                   "wmin_x": {k: float(v[:, 0].min().round(4)) for k, v in W.items()},
                   "wires": {k: v[::2].round(3).tolist() for k, v in W.items()},
                   "plastic": float(COPPER.plastic.sum()), "tips": m.fingertips(env)[[0, 12]].round(3).tolist()}
            with open(LOG, "a") as f:
                f.write(json.dumps(rec) + "\n")


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full)


# -- the demo ---------------------------------------------------------------------------------------------------------
def demo():
    """yam_scripted's plan: empty, because closedloop.py plays `demo_script` instead."""
    return {}


FACE_PT = np.array((DEV_X0 - STRAP_T - FACE_D, YB, DEV_Z0))   # the face's centre, device frame
SEAT = np.array([XW - DEV_X0, 0.0, ZB - DEV_Z0])             # the shift from the starting pose to the seated one
HEAD_PTS = [np.array((DEV_X0 - STRAP_T - HEAD_H, YB, z)) for z in SCREW_Z]
STRAP_PTS = [np.array((DEV_X0, YB, z)) for z in SCREW_Z]
GRASP = np.array((DEV_X0 - STRAP_T - FACE_D + 0.012, YB, DEV_Z0 + 0.012))   # where the holding fingertips close: the body's sides


def push(arm, goal, read, lead=0.02, gain=0.25, speed=0.04, timeout=4.0, tol=0.001, label="push"):
    """Move `arm`'s command so that the point `read()` goes to `goal()`, at most `speed` m/s, never commanding the TCP more
    than `lead` metres from where it is now: a bounded push against something that resists. Returns whether it got there."""
    good = 0
    for _ in range(max(1, round(timeout * d.CONTROL_HZ))):
        err = np.asarray(goal(), float) - np.asarray(read(), float)
        if np.linalg.norm(err) < tol:
            good += 1
            if good >= 3:
                return True
        else:
            good = 0
        c = d.ik.tcp_pose(arm.q, arm.base)[0]
        step = np.clip(gain * err, -speed * d.DT, speed * d.DT)
        c = c + step
        now = arm.tcp()
        off = c - now
        n = float(np.linalg.norm(off))
        if n > lead:
            c = now + off * (lead / n)
        arm._solve(c, arm.grip.rot)
        yield
    print(f"[demo] {label}: stopped {np.linalg.norm(err) * 1000:.1f} mm short", flush=True)
    return False


def demo_script(ctx):
    """The scripted demo (README, The scripted demo). Left: takes the receptacle by the sides of its body and pushes it
    home, square and on the line of the box's screw holes, folding the conductors into the box, and holds it there.
    Right: picks up the driver, drives the lower screw, then (left hand gone) the upper one, and puts the driver down.
    It reads object poses only: the receptacle's, the driver's and, for nothing but its own arms, the joints.
    Red-team modes (TASK_DEMO): none, hold, onearm, noscrew, onescrew."""
    env = closedloop._ENV["env"]
    H, D = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        yield from H.wait(10.0)
        return
    g_h = d.tilted(60.0, 0.0)
    a_h = g_h.rot[:, 2]
    axis = np.array([1.0, 0.0, 0.0])
    grasp = lambda: dev_point(env, GRASP)
    holder = lambda: m.state(env, "driver").to_world(BIT_TIP)[0]       # where the bit's tip is with the spindle out
    head = lambda i: dev_point(env, HEAD_PTS[i])
    gap = lambda i: XW - dev_point(env, STRAP_PTS[i])[0]

    def seat():
        yield from H.move(grasp() - 0.073 * a_h + np.array([0.0, 0.0, 0.02]), grip=g_h, finger=d.FINGER_OPEN, t=2.5, label="H over")
        yield from H.move(grasp() - 0.043 * a_h, t=1.0, label="H before")
        yield from H.move(grasp() - 0.013 * a_h, t=1.0, label="H in")
        yield from H.close(DEV_W, 40.0, label="H grip")
        yield from H.move(H.at() + np.array([0.0, 0.0, -0.02]), t=1.5, label="H lower")
        yield from push(H, lambda: FACE_PT + SEAT + 0.0005 * axis, lambda: dev_point(env, FACE_PT), lead=0.06, timeout=10.0, tol=0.0012, label="H press")

    def fetch():
        # take it by the front of its housing, so that the hand stays well out from the arm's base with the bit at the wall
        front = lambda: m.state(env, "driver").to_world(np.array([DRV_GRIP_X, 0.0, 0.0]))[0]
        yield from D.pick(front, 0.012, width=DRV_W, grip=d.down(0.0), above=0.08, lift=0.10, label="D pick")

    def drive(i):
        y0 = float(holder()[1])
        yield from push(D, lambda: np.array([head(i)[0] - 0.05, y0, head(i)[2]]), holder, lead=0.03, speed=0.08, timeout=4.0, tol=0.003, label=f"D level with {i}")
        yield from push(D, lambda: head(i) - 0.03 * axis, holder, lead=0.03, speed=0.08, timeout=5.0, tol=0.0015, label=f"D line up {i}")
        yield from push(D, lambda: head(i) - 0.006 * axis, holder, lead=0.02, speed=0.03, timeout=3.0, tol=0.0008, label=f"D aim {i}")
        for _ in range(32):            # push the bit on toward where the head sits when the strap is home, until it is
            yield from push(D, lambda: HEAD_PTS[i] + SEAT + 0.0025 * axis, holder, lead=0.03, speed=0.03, timeout=0.5, tol=0.0004, label=f"D drive {i}")
            if gap(i) < 0.0006:
                break
        yield from D.wait(0.6)
        yield from D.move(D.at() - 0.035 * axis, t=1.0, label="D back")

    def let_go():
        yield from H.open(label="H let go")
        yield from H.move(H.at() - 0.05 * a_h + np.array([0.0, 0.0, 0.03]), t=1.2, label="H away")
        yield from H.move(np.array([0.15, 0.20, TOP + 0.22]), grip=d.down(0.0), t=2.0, label="H rest")

    if mode == "onearm":                # the driving hand alone: the bit pushes the receptacle in by its screw heads
        yield from fetch()
        yield from drive(1)
        yield from drive(0)
        yield from D.wait(2.0)
        return
    if mode in ("hold", "noscrew"):     # seat it by hand; hold it there, or let go, with no screw driven
        yield from seat()
        if mode == "noscrew":
            yield from let_go()
        yield from H.wait(4.0)
        return
    yield from ctx.together(seat(), fetch())
    yield from drive(1)
    yield from let_go()
    if mode != "onescrew":
        yield from drive(0)
    yield from D.place(np.array(DRV_POS) + np.array([DRV_GRIP_X, 0.0, 0.0]), 0.015, above=0.06, back=0.0, label="D put down")
    yield from D.move(D.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="D up")
    yield from D.wait(2.0)


closedloop.install(demo_script, NAMES)
