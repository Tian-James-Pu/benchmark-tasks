"""On the station table (a living-room side table): a small table lamp with a fitted A19 bulb in its E26 socket and no
harp, its 9-inch replacement harp lying flat on the table to the right of the lamp, a candle, a mug and a pair of
glasses.

The lamp: a 13 cm round base and a glazed ceramic body; its brass E26 socket (34 mm across, 63 mm tall) stands on a
harp saddle 15 cm above the table. The saddle is a 1.6 mm steel strap under the socket whose two ends turn up into
C-shaped channels 64 mm apart, each open on its outer side, along the station's x axis (one channel nearer the arms,
one farther).

The harp: a 9 in (229 mm, saddle to top) detachable lamp harp of 3.2 mm spring-steel wire, about 100 mm across at its
widest, with a 1/4-27 finial stud on top. Its two straight lower legs rest 70 mm apart, 6 mm wider than the saddle's
channels, and carry a sliding ferrule each (9 mm across, 8 mm long), slid up with its lower end 20.5 mm above the leg
end. It lies flat with its legs toward the right arm and its finial stud toward the lamp's side of the table.

The harp is fitted by squeezing its legs 6 mm closer and holding them there, setting the leg ends into the saddle's
channels, and sliding each ferrule 12 mm down over the joint. A leg let go before its ferrule is down springs back out
of its channel.
"""

import json
import math
import os
import pathlib
import sys

import numpy as np

from yam_arena import demo as _demo
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
INCH = 0.0254

# -- the lamp ---------------------------------------------------------------------------------------------------------

LX, LY = 0.34, 0.0  # the lamp's axis on the table
SADDLE_H = 0.150  # the saddle's top above the table
ZS = TOP + SADDLE_H
CH_R = 32.0 * MM  # each channel's centre from the lamp's axis: 64 mm apart
SOCKET_R, SOCKET_H = 17.0 * MM, 63.0 * MM
BULB_L, BULB_R = 4.44 * INCH, 2.38 * INCH / 2  # A19
BULB_Z = SADDLE_H + SOCKET_H - 0.025  # the bulb's base tip, 25 mm of it screwed into the socket
WALL_H, LEAD_H = 6.0 * MM, 2.0 * MM  # channel walls that hold the wire, then a 2 mm flared lip
C_IN, C_WALL = 2.6 * MM, 0.6 * MM  # channel half-width inside, wall thickness
C_VIS_H = 11.0 * MM  # the channel's formed height as seen

LOOKS = {
    "ceramic": m.Look(m.rgb(0.20, 0.33, 0.42), roughness=0.25),
    "felt": m.Look(m.rgb(0.10, 0.10, 0.10), roughness=0.95),
    "brass": m.Look(m.rgb(0.70, 0.52, 0.24), metallic=0.85, roughness=0.28),
    "steel": m.Look(m.rgb(0.62, 0.62, 0.60), metallic=0.8, roughness=0.35),
    "nickel": m.Look(m.rgb(0.80, 0.80, 0.78), metallic=0.9, roughness=0.22),
    "glass": m.Look(glass=(0.98, 0.99, 1.0)),
    "filament": m.Look(m.rgb(0.15, 0.13, 0.10), metallic=0.6, roughness=0.5),
}


def _hex(pts):
    """A closed mesh on 8 corners (bottom 4 then top 4), for a convex-hull collider."""
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return m.Mesh(np.asarray(pts, float), np.array(f))


def _c_channel(sx):
    """One saddle end's C channel (visual), centred on the leg's seat, open on its outer side."""
    pts = []
    r_in, r_out = C_IN + 0.3 * MM, C_IN + C_WALL + 0.3 * MM
    a0, a1 = math.radians(45), math.radians(315)  # the gap faces outward (+x for sx=+1)
    n = 18
    outline = [(r_out * math.cos(a0 + (a1 - a0) * k / n), r_out * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]
    outline += [(r_in * math.cos(a1 - (a1 - a0) * k / n), r_in * math.sin(a1 - (a1 - a0) * k / n)) for k in range(n + 1)]
    v, f = [], []
    for z in (0.0, C_VIS_H):
        for x, y in outline:
            v.append((sx * x + sx * CH_R, y, SADDLE_H + z))
    k = len(outline)
    for i in range(k):
        j = (i + 1) % k
        f += [(i, j, k + j), (i, k + j, k + i)]
    for i in range(1, n):  # caps as strips between the outer and inner arcs
        o0, o1, i0, i1 = i - 1, i, 2 * n + 1 - i, 2 * n + 2 - i
        f += [(k + o0, k + o1, k + i0), (k + o0, k + i0, k + i1)]
        f += [(o0, i0, o1), (o0, i1, i0)]
    return m.Mesh(np.array(v), np.array(f))


def _lamp():
    parts, cols = [], []
    base = m.lathe([(0, 0), (0.063, 0), (0.065, 0.004), (0.065, 0.016), (0.058, 0.024), (0, 0.024)], 64)
    parts.append(("base", base, "felt"))
    cols.append(m.col_hull("base_c", base))
    body = m.lathe([(0, 0.024), (0.040, 0.024), (0.058, 0.041), (0.064, 0.064), (0.056, 0.087), (0.030, 0.106),
                    (0.016, 0.116), (0, 0.118)], 64)
    parts.append(("body", body, "ceramic"))
    cols.append(m.col_hull("body_c", body))
    neck = m.cylinder(0.010, SADDLE_H - 0.0016 - 0.116, (0, 0, (SADDLE_H - 0.0016 + 0.116) / 2))
    parts.append(("neck", neck, "brass"))
    cols.append(m.col_hull("neck_c", neck))
    # the saddle: a 1.6 mm strap, 9 mm wide, along x, under the socket; its ends turned up into C channels
    strap = m.box((2 * (CH_R + C_IN + C_WALL), 0.009, 0.0016), (0, 0, SADDLE_H - 0.0008))
    parts.append(("saddle", strap, "steel"))
    cols.append(m.col_box("saddle_c", (2 * (CH_R + C_IN + C_WALL), 0.009, 0.0016), (0, 0, SADDLE_H - 0.0008)))
    for sx, tag in ((-1, "near"), (1, "far")):
        parts.append((f"channel_{tag}", _c_channel(sx), "steel"))
        cx = sx * CH_R
        # what holds the wire: the inner wall, two side walls to the open outer side, and a 2 mm flared lip on each
        inner_x = cx - sx * (C_IN + C_WALL / 2)
        cols.append(m.col_box(f"inner_{tag}_c", (C_WALL, 2 * (C_IN + C_WALL), WALL_H), (inner_x, 0, SADDLE_H + WALL_H / 2)))
        x0, x1 = sorted((inner_x, cx + sx * 2.0 * MM))
        for sy in (-1, 1):
            cols.append(m.col_box(f"side_{tag}_{sy + 1}_c", (x1 - x0, C_WALL, WALL_H),
                                  ((x0 + x1) / 2, sy * (C_IN + C_WALL / 2), SADDLE_H + WALL_H / 2)))
            # the lip leans out: a hull of the wall's top edge and the same edge 1.2 mm further out, 2 mm higher
            lo = np.array([[x0, sy * C_IN, SADDLE_H + WALL_H], [x1, sy * C_IN, SADDLE_H + WALL_H],
                           [x0, sy * (C_IN + C_WALL), SADDLE_H + WALL_H], [x1, sy * (C_IN + C_WALL), SADDLE_H + WALL_H]])
            hi = lo + np.array([0.0, sy * 1.2 * MM, LEAD_H])
            cols.append(m.col_hull(f"lip_{tag}_{sy + 1}_c", _hex(np.vstack([lo, hi]))))
    socket = m.lathe([(0, SADDLE_H), (0.0150, SADDLE_H), (SOCKET_R, SADDLE_H + 0.004), (SOCKET_R, SADDLE_H + SOCKET_H - 0.003),
                      (0.0155, SADDLE_H + SOCKET_H), (0, SADDLE_H + SOCKET_H)], 48)
    parts.append(("socket", socket, "brass"))
    cols.append(m.col_hull("socket_c", socket))
    profile = [(0, .025), (.0125, .025), (.014, .040), (.021, .055), (BULB_R, .074), (BULB_R, .084), (.026, .099), (.016, .109), (0, BULB_L)]
    envelope = m.lathe([(r, z + BULB_Z) for r, z in profile], 64)
    parts.append(("bulb", envelope, "glass"))
    cols.append(m.col_hull("bulb_c", envelope))
    parts.append(("stem", m.cylinder(.002, .042, (0, 0, BULB_Z + .046)), "glass"))
    coil = [(.0006 * math.cos(24 * math.pi * i / 120), -.011 + .022 * i / 120, BULB_Z + .070 + .0006 * math.sin(24 * math.pi * i / 120)) for i in range(121)]
    parts.append(("filament", m.tube(coil, .00013, 6), "filament"))
    return m.fixed("table_lamp", (LX, LY, TOP), parts, cols, LOOKS, friction=(1.0, 0.8))


# -- the harp ---------------------------------------------------------------------------------------------------------
# Authored lying flat, in its own (u, w) plane: u across the legs, w from the leg ends (w = 0) up to the top. Part frame:
# x = u, y = w, z = up from the table. Standing on the saddle it is turned +90 degrees about x: then x = u, z = w.

WIRE_R = 1.6 * MM
REST_U = 35.0 * MM  # the legs' rest: 70 mm apart, 6 mm wider than the channels
PIVOT = (30.0 * MM, 205.0 * MM)  # where each leg flexes from, at the shoulder
LEG_PATH = [(REST_U, 0.0), (REST_U, 110 * MM), (49 * MM, 140 * MM), (49 * MM, 175 * MM), PIVOT]
CROWN_PATH = [(-PIVOT[0], PIVOT[1]), (-18 * MM, 218 * MM), (-8 * MM, 224.6 * MM), (8 * MM, 224.6 * MM), (18 * MM, 218 * MM), PIVOT]
HARP_H = 9 * INCH  # saddle to top of the boss
BOSS_R, BOSS_T = 9.0 * MM, 4.0 * MM
STUD_R, STUD_L = 0.125 * INCH, 0.5 * INCH
FER_R, FER_L = 4.5 * MM, 8.0 * MM  # ferrule: 9 mm across, 8 mm long
FER_UP = 20.5 * MM  # its lower end above the leg end, slid up
FER_TRAVEL = 12.0 * MM  # slid down, its lower end 8.5 mm above the leg end: over the channel's top
C0 = (0.0, 115 * MM)  # the free joints' pivot
ZC = BOSS_R + 0.0005  # wire centres above the table as authored (it settles onto its ferrules)
HX, HY = 0.19, -0.240  # the harp's leg ends' midpoint as authored
SPRING = 44.0  # N m/rad at each leg: about 5 N to hold a leg end 3 mm in
LEG_MASS, CROWN_MASS, FER_MASS = 0.012, 0.016, 0.002
PINCH_W = 90 * MM  # where the demo holds the legs


def _p(u, w, v=0.0):
    return (u, w, v)


def _wire(points, radius=WIRE_R):
    return m.tube([_p(u, w) for u, w in points], radius, 10)


def _segments(name, points):
    return [m.col_hull(f"{name}{i}", m.tube([_p(*points[i]), _p(*points[i + 1])], WIRE_R, 8)) for i in range(len(points) - 1)]


def _mirror(points, s):
    return [(s * u, w) for u, w in points]


def _harp():
    free = dict(mass=0.002, spring=0.0, damping=0.0, friction=0.0, armature=1e-5, inertia=(1e-7, 1e-7, 1e-7))
    inf = float("inf")
    c0 = _p(*C0)
    links = [
        m.slider("sx", "X", -0.6, 0.6, (), (), at=c0, **free),
        m.slider("sy", "Y", -0.6, 0.6, (), (), at=c0, parent="sx", **free),
        m.slider("sz", "Z", -0.1, 0.8, (), (), at=c0, parent="sy", **free),
        m.hinge("rz", "Z", c0, -inf, inf, (), (), parent="sz", **free),
        m.hinge("ry", "Y", c0, -inf, inf, (), (), parent="rz", **free),
    ]
    boss = m.cylinder(BOSS_R, BOSS_T, _p(0, HARP_H - BOSS_T / 2), axis="y")
    stud = m.cylinder(STUD_R, STUD_L, _p(0, HARP_H + STUD_L / 2), axis="y")
    crown_parts = [("crown", _wire(CROWN_PATH), "nickel"), ("boss", boss, "nickel"), ("stud", stud, "nickel")]
    crown_cols = _segments("crown_c", CROWN_PATH) + [m.col_hull("boss_c", boss), m.col_hull("stud_c", stud)]
    links.append(m.hinge("crown", "X", c0, -inf, inf, crown_parts, crown_cols, parent="ry", mass=CROWN_MASS, gravity=True,
                         com=_p(0, 0.222), damping=0.0, friction=0.0, armature=1e-6))
    for s, tag in ((-1, "l"), (1, "r")):
        path = _mirror(LEG_PATH, s)
        lo, hi = (-4.0, 0.3) if s > 0 else (-0.3, 4.0)  # squeezing turns the right leg negative, the left positive
        links.append(m.hinge(f"leg_{tag}", "Z", _p(s * PIVOT[0], PIVOT[1]), lo, hi, [(f"leg", _wire(path), "nickel")],
                             _segments(f"leg_{tag}_c", path), parent="crown", mass=LEG_MASS, gravity=True,
                             com=_p(s * 0.040, 0.10), spring=SPRING, damping=0.05, effort=20.0, friction=0.0, armature=2e-4))
        fer_c = _p(s * REST_U, FER_UP + FER_L / 2)
        tube = m.lathe([(0.0038, -FER_L / 2), (FER_R, -FER_L / 2), (FER_R, FER_L / 2), (0.0038, FER_L / 2)], 32)
        tube = m.Mesh(np.c_[tube.v[:, 0], tube.v[:, 2], tube.v[:, 1]] + np.array(fer_c), tube.f[:, ::-1])
        links.append(m.slider(f"fer_{tag}", "Y", -FER_TRAVEL, 0.0, [("ferrule", tube, "nickel")],
                              [m.col_hull(f"fer_{tag}_c", m.cylinder(FER_R, FER_L, fer_c, axis="y", segments=16))],
                              at=fer_c, parent=f"leg_{tag}", mass=FER_MASS, damping=2.0, friction=1.0, armature=1e-3))
    return m.mechanism("harp", (HX, HY, TOP + ZC), links, LOOKS, solver_iters=64)


LAMP = _lamp()
HARP = _harp()

OBJECTS = [
    LAMP,
    HARP,
    # Distractors (SimReady props), which scoring ignores. pos is each one's origin, the centre of its base.
    SimReady("candle", pos=(0.50, 0.20, TOP), size=(0.074, 0.074, 0.075), mass=0.30, yaw=0.0,
             package="decor_candle_a01", usd="sm_decor_candle_a01_01.usd"),
    SimReady("mug", pos=(0.52, -0.20, TOP), size=(0.114, 0.090, 0.080), mass=0.35, yaw=2.4,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("glasses", pos=(0.17, 0.22, TOP), size=(0.145, 0.174, 0.057), mass=0.03, yaw=1.2,
             package="accessory_eyeglasses_a01", usd="sm_accessory_eyeglasses_a01_01.usd"),
]

# -- the harp's state, read in a hook -----------------------------------------------------------------------------------

BODIES = ("crown", "leg_l", "leg_r", "fer_l", "fer_r")
FOOT = {"l": np.array(_p(-REST_U, 0.0)), "r": np.array(_p(REST_U, 0.0))}
SIDE = {"l": -1, "r": 1}  # leg_l goes in the near channel (x < lamp), leg_r in the far one


def _samples():
    """Points along the harp's wire, boss and ferrules in each body's (authored) frame, for the hands-clear check."""
    def along(points, step=0.008):
        out = []
        for (u0, w0), (u1, w1) in zip(points[:-1], points[1:]):
            n = max(1, int(math.hypot(u1 - u0, w1 - w0) / step))
            out += [_p(u0 + (u1 - u0) * k / n, w0 + (w1 - w0) * k / n) for k in range(n + 1)]
        return np.array(out)
    pts = {"crown": np.vstack([along(CROWN_PATH), [_p(0, HARP_H - BOSS_T / 2), _p(0, HARP_H + STUD_L)]])}
    for tag, s in SIDE.items():
        pts[f"leg_{tag}"] = along(_mirror(LEG_PATH, s))
        pts[f"fer_{tag}"] = np.array([_p(s * REST_U, FER_UP), _p(s * REST_U, FER_UP + FER_L)])
    return pts


SAMPLES = _samples()


def _poses(env):
    base = env.unwrapped
    asset = base.scene["harp"]
    origin = m._np(base.scene.env_origins[0])
    pose = m._np(asset.data.body_link_pose_w)[0]
    vel = m._np(asset.data.body_link_vel_w)[0]
    out = {}
    for name in BODIES:
        i = asset.body_names.index(name)
        out[name] = (pose[i, :3] - origin, m.quat_matrix(pose[i, 3:7]), vel[i, :3])
    return out


def _world(poses, body, local):
    p, r, _ = poses[body]
    return np.atleast_2d(local) @ r.T + p


def _foot(poses, tag):
    """The leg end's offsets from its channel's seat: radial (+ outward, toward the channel's open side), sideways and
    height above the saddle's top, metres."""
    f = _world(poses, f"leg_{tag}", FOOT[tag])[0]
    s = SIDE[tag]
    return s * (f[0] - (LX + s * CH_R)), f[1] - LY, f[2] - ZS


def seated(offsets):
    """The leg end inside its channel: between the side walls, not out past the open side, down on the saddle."""
    dr, dt, dz = offsets
    return bool(-1.5 * MM <= dr <= 1.5 * MM and abs(dt) <= 1.3 * MM and -0.8 * MM <= dz <= 2.0 * MM)


def harp_tilt_deg(poses):
    up = poses["crown"][1] @ np.array([0.0, 1.0, 0.0])
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def hands_clear(poses, tips):
    pts = np.vstack([_world(poses, b, SAMPLES[b]) for b in BODIES])
    d = np.linalg.norm(tips[:, None, :] - pts[None, :, :], axis=2)
    return bool(d.min() - BOSS_R >= CLEAR)  # 3 cm clear of the wire, ferrules and the 9 mm boss


# -- the ferrules' hold and scoring --------------------------------------------------------------------------------------

FER_DOWN = -10.5 * MM  # a ferrule counts as down over the joint within 1.5 mm of its stop
FER_CLEAR = -9.0 * MM  # and as clear of the channel above this: its lower end 11.5 mm above the leg end, over the 11 mm channel
CHAIN = ("sx", "sy", "sz", "rz", "ry", "crown")
SUCCESS_HOLD_S = 0.0  # the Trial counts its own three-second hold
SCORE_OVER = "end"
HOLD_S = 3.0
TILT_DEG = 3.0
CLEAR = 0.03
STILL = 0.01


def _damping(env, link, value):
    asset, j = m._joint(env, "harp", link)
    asset.write_joint_damping_to_sim(m._tensor(asset, value), joint_ids=[j])


def _weld(env, on):
    """Both legs held by their ferrules: the harp is fixed to the saddle where it stands (and freed again)."""
    for link in CHAIN:
        rot = link in ("rz", "ry", "crown")
        if on:
            m.lock(env, "harp", link, stiffness=20.0 if rot else 20000.0, effort=10.0 if rot else 200.0)
            _damping(env, link, 0.15 if rot else 60.0)
        else:
            m.unlock(env, "harp", link)
            _damping(env, link, 0.0)


class Trial:
    """Updated once per control step; score() and success() only read it."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place any of this is cleared
        self.armed = {"l": False, "r": False}  # the leg sat in its channel with its ferrule still clear above it
        self.held = {"l": False, "r": False}  # the ferrule then slid down over the joint: the leg can't spring out
        self.welded = False
        self.hold = 0.0
        self.full = False
        self.value = 0.0
        self.n_seated = 0
        if env is not None:
            for link in CHAIN:
                _damping(env, link, 0.0)

    def step(self, env):
        poses = _poses(env)
        st = m.state(env, "harp")
        tips = m.fingertips(env)
        seat = {}
        for tag in ("l", "r"):
            off = _foot(poses, tag)
            seat[tag] = seated(off)
            fer = st.joint(f"fer_{tag}")
            if not seat[tag]:
                self.armed[tag] = False
            elif fer > FER_CLEAR:
                self.armed[tag] = True
            hold = bool(seat[tag] and self.armed[tag] and fer <= FER_DOWN)
            if hold != self.held[tag]:
                (m.lock(env, "harp", f"leg_{tag}", stiffness=400.0, effort=40.0) if hold else m.unlock(env, "harp", f"leg_{tag}"))
                self.held[tag] = hold
        both = self.held["l"] and self.held["r"]
        if both != self.welded:
            _weld(env, both)
            self.welded = both
        self.n_seated = int(seat["l"]) + int(seat["r"])
        speed = float(np.linalg.norm(poses["crown"][2]))
        ready = bool(both and harp_tilt_deg(poses) <= TILT_DEG and hands_clear(poses, tips) and speed < STILL)
        self.hold = self.hold + float(env.unwrapped.step_dt) if ready else 0.0
        self.full = bool(ready and self.hold >= HOLD_S - 1e-9)
        self.value = 1.0 if self.full else (0.5 if self.n_seated == 2 else (0.25 if self.n_seated == 1 else 0.0))
        if os.environ.get("HARP_DEBUG"):
            _debug(env, poses, st, tips, seat)


_LOG = []


def _debug(env, poses, st, tips, seat):
    _LOG.append({"t": len(_LOG) / 20, "joints": st.joints, "feet": {t: [round(v * 1000, 2) for v in _foot(poses, t)] for t in ("l", "r")},
                 "seat": seat, "held": dict(TRIAL.held), "welded": TRIAL.welded, "tilt": harp_tilt_deg(poses),
                 "crown": poses["crown"][0].tolist(), "value": TRIAL.value, "hold": TRIAL.hold,
                 "pinch": {t: _world(poses, f"leg_{t}", _p(SIDE[t] * REST_U, PINCH_W))[0].tolist() for t in ("l", "r")},
                 "fer_top": {t: _world(poses, f"fer_{t}", _p(SIDE[t] * REST_U, FER_UP + FER_L))[0].tolist() for t in ("l", "r")},
                 "tips": tips.tolist()})
    if len(_LOG) % 10 == 1:
        pathlib.Path(os.environ["HARP_DEBUG"]).write_text(json.dumps(_LOG))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo: an open-loop plan from the fixed starting layout -------------------------------------------------


class Grip(float):
    """A waypoint 'yaw' that carries a whole TCP rotation."""

    rot = np.eye(3)


def _grip(r):
    g = Grip(0.0)
    g.rot = r
    return g


_plain = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_takes_grips", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return _plain(yaw) if rot is None else rot

    _grip_rotation._takes_grips = True
    _grip_rotation._plain = _plain
    _demo.grip_rotation = _grip_rotation


def down(yaw_deg):
    return _grip(_plain(math.radians(yaw_deg)))


def level(phi_deg):
    """Fingers level, pointing along phi from +x, closing across it."""
    phi = math.radians(phi_deg)
    return _grip(axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.pi / 2) @ _plain(phi))


# measured from the settled start and a first run (see README)
STUD_Y = HY + HARP_H + 0.020  # the TCP 20 mm past the boss along the stud: the pads cover the stud's outer 11.7 mm
PICK_Z = TOP + 0.013  # the pad tips at the table, the stud (9 mm up) between the pads
HANG = (LX, 0.096)  # where the right hand holds the stud, the harp hanging upright beside the lamp: its plane at y 0.10
PINCH = (LX, 0.100)  # where the left hand takes the legs: the TCP on the harp's plane
WIDE = FINGER_OPEN
SQUEEZE = -0.0338  # finger target: the pads hold the leg ends 64 mm apart against their spring
CARRY_Z = ZS + 0.012 + PINCH_W  # leg ends 12 mm above the saddle, clear of the channels' lips
HANG_Z = CARRY_Z - PINCH_W + 0.2486  # the stud's TCP for that: 248.6 mm above the leg ends
IN_Z = ZS + 0.001 + PINCH_W  # leg ends 1 mm above the saddle
FORK = -0.0030  # half-open: 6 mm between the pads, astride the 3.2 mm wire
FER_TOP = ZS + 0.001 + FER_UP + FER_L  # a slid-up ferrule's top, the harp seated
FORK_Z = FER_TOP + 0.018 + 0.001  # the pads' lower edges 1 mm above it
HIGH_L = (LX, 0.13, TOP + 0.29)


def demo():
    """Right arm: takes the flat harp by its finial stud, turns it upright and holds it hanging beside the lamp. Left
    arm: takes both legs from the left side and squeezes their ends 64 mm apart; the right lets go of the stud. The left
    carries the squeezed harp round the bulb, sets the leg ends in the saddle's channels and holds the squeeze. The right
    comes back level from the right side with its half-open fingers astride each leg's wire above its ferrule, and
    pushes each ferrule 12 mm down over its channel. Then the left lets go and both arms leave."""
    near, far = LX - CH_R, LX + CH_R
    right = [
        Waypoint(2.5, (HX, STUD_Y, TOP + 0.08), WIDE, "over the finial stud", down(90)),
        Waypoint(1.5, (HX, STUD_Y, PICK_Z), WIDE, "pads either side of the stud", down(90)),
        Waypoint(0.8, (HX, STUD_Y, PICK_Z), 0.0, "grip the stud", down(90)),
        Waypoint(0.4, (HX, STUD_Y, PICK_Z), 0.0, "hold", down(90)),
        Waypoint(1.5, (HX, STUD_Y, TOP + 0.17), 0.0, "lift", down(90)),
        Waypoint(2.5, (HX + 0.03, STUD_Y + 0.03, TOP + 0.30), 0.0, "turn it upright", level(90)),
        Waypoint(2.5, (HANG[0], HANG[1], HANG_Z), 0.0, "hang it beside the lamp", level(90)),
        Waypoint(3.6, (HANG[0], HANG[1], HANG_Z), 0.0, "hold it for the other hand", level(90)),
        Waypoint(0.6, (HANG[0], HANG[1], HANG_Z), WIDE, "let go of the stud", level(90)),
        Waypoint(1.5, (HANG[0], LY, HANG_Z + 0.02), WIDE, "back out", level(90)),
        Waypoint(2.0, (near, LY - 0.10, FORK_Z + 0.10), FORK, "round to the near leg", level(90)),
        Waypoint(1.5, (near, LY - 0.10, FORK_Z), FORK, "beside the near leg", level(90)),
        Waypoint(0.3, (near, LY - 0.10, FORK_Z), FORK, "wait for the harp", level(90)),
        Waypoint(1.5, (near, LY, FORK_Z), FORK, "astride the wire, over the ferrule", level(90)),
        Waypoint(1.5, (near, LY, FORK_Z - 0.013), FORK, "ferrule down over the channel", level(90)),
        Waypoint(0.5, (near, LY, FORK_Z - 0.013), FORK, "settle", level(90)),
        Waypoint(1.2, (near, LY - 0.06, FORK_Z - 0.013), FORK, "back out", level(90)),
        Waypoint(1.5, (far, LY - 0.06, FORK_Z), FORK, "beside the far leg", level(90)),
        Waypoint(1.5, (far, LY, FORK_Z), FORK, "astride the wire, over the ferrule", level(90)),
        Waypoint(1.5, (far, LY, FORK_Z - 0.013), FORK, "ferrule down over the channel", level(90)),
        Waypoint(0.5, (far, LY, FORK_Z - 0.013), FORK, "settle", level(90)),
        Waypoint(1.2, (far, LY - 0.10, FORK_Z - 0.013), FORK, "back out", level(90)),
        Waypoint(2.0, (0.30, -0.14, TOP + 0.25), FORK, "away", level(90)),
        Waypoint(6.0, (0.30, -0.14, TOP + 0.25), FORK, "wait", level(90)),
    ]
    left = [
        Waypoint(3.0, HIGH_L, WIDE, "up on the left", level(-90)),
        Waypoint(1.5, (PINCH[0], PINCH[1] + 0.035, CARRY_Z), WIDE, "beside where the harp will hang", level(-90)),
        Waypoint(7.7, (PINCH[0], PINCH[1] + 0.035, CARRY_Z), WIDE, "wait for the harp", level(-90)),
        Waypoint(1.5, (PINCH[0], PINCH[1], CARRY_Z), WIDE, "fingers either side of the legs", level(-90)),
        Waypoint(0.8, (PINCH[0], PINCH[1], CARRY_Z), SQUEEZE, "squeeze the legs", level(-90)),
        Waypoint(3.0, (PINCH[0], PINCH[1], CARRY_Z), SQUEEZE, "hold while the other hand lets go", level(-90)),
        Waypoint(2.0, (LX, LY, CARRY_Z), SQUEEZE, "over the saddle, round the bulb", level(-90)),
        Waypoint(1.5, (LX, LY, IN_Z), SQUEEZE, "leg ends into the channels", level(-90)),
        Waypoint(10.1, (LX, LY, IN_Z), SQUEEZE, "hold the squeeze", level(-90)),
        Waypoint(0.6, (LX, LY, IN_Z), WIDE, "let go", level(-90)),
        Waypoint(1.5, (LX, LY + 0.12, IN_Z), WIDE, "back out", level(-90)),
        Waypoint(2.0, HIGH_L, WIDE, "away", level(-90)),
        Waypoint(5.0, HIGH_L, WIDE, "wait", level(-90)),
    ]
    return {"left": left, "right": right}
