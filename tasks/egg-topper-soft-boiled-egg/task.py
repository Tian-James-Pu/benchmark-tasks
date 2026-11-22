"""At a breakfast place setting: a soft-boiled large egg (57 g, 43 x 57 mm, pointed end up) stands in a porcelain egg cup. Beside
it stands a drop-weight egg topper (stainless steel, the pattern of the Clack soft-boiled egg topper: a 40 mm bell whose inner
edge is a 34 mm ring of teeth, a silicone collar to hold it by, a 4 mm rod and a 45 g weight that slides 10 cm up the rod). A side
plate, a teaspoon, a tea cup and a salt shaker are on the table too (distractors; scoring ignores them).

How the topper works (the maker's pattern): the bell is set on the egg's crown and held there; the weight is lifted up the rod
and let fall onto the collar. The blow drives the ring of teeth through the shell in a clean circle, and the cap then lifts off in
one piece, carried away in the bell when the topper is lifted. Here (`Breakfast`):
- A drop counts when the weight falls freely (from its highest point to its rest in at most 0.4 s) from at least DROP_FULL
  (8 cm) above its rest, while the bell sits on the egg (its ring plane within 6 mm of the shell's ring height, its axis within
  12 mm of the egg's).
- Such a drop with the bell centred on the egg's axis within CENTRE_TOL (3 mm) and the topper's axis within TILT_TOL (8 degrees)
  of the egg's scores the shell in a clean ring: the cap is then free. Off-centre or tilted beyond that, the blow lands on the
  egg's side and cracks it below the ring (recorded, the egg is spoiled). Shorter drops do nothing and can be repeated.
- Until the shell is cut the cap is part of the egg: the hook holds it in place on the lower shell every step. Once it is cut, the
  hook carries it in the bell (gripped by the teeth), so lifting the topper takes the top off the egg.

    python demokit.py task.py OUT            (README)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402  importing it hooks yam_arena (on_step, on_reset, fingertips)
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Take the top off the soft-boiled egg with the egg topper, then stand the topper back up on the table."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz

# -- the egg: a large egg, 43 mm wide, 57 mm long (26 mm blunt half below the equator, 31 mm pointed half above) ----------
EGG_A = 0.0215  # equatorial radius
EGG_CB = 0.026  # blunt half (down, in the cup)
EGG_CT = 0.031  # pointed half (up)
EGG_MASS = 0.057
RING_R = 0.017  # the topper's ring of teeth, inner radius
R_CUT = 0.0165  # where the teeth cut: the cap's rim radius
Z_CUT = EGG_CT * math.sqrt(1 - (R_CUT / EGG_A) ** 2)  # 0.0199 above the equator
Z_RING = EGG_CT * math.sqrt(1 - (RING_R / EGG_A) ** 2)  # 0.0190: where the ring rests on the shell
CAP_MASS = 0.0053  # 5.1 cm3 of shell and white above the cut
LOWER_MASS = EGG_MASS - CAP_MASS

# -- the egg cup: porcelain, 55 mm tall, 38 mm inside the rim -------------------------------------------------------------
CUP_XY = (0.30, 0.0)
CUP_H = 0.055
CUP_RIM_IN = 0.019
CUP_RIM_OUT = 0.0235
CUP_FLOOR = 0.036
CUP_MASS = 0.075
EGG_SEAT = CUP_H + EGG_CB * math.sqrt(1 - (CUP_RIM_IN / EGG_A) ** 2)  # the egg's centre above the table: 0.0672

# -- the topper (frame origin: the centre of the ring of teeth, at the bell's open bottom) --------------------------------
BELL_R = 0.020
BELL_H = 0.020
COLLAR_R = 0.011
COLLAR_Z = (0.020, 0.070)
ROD_R = 0.004
ROD_TOP = 0.215
KNOB = (0.0075, 0.010)
WEIGHT_R = 0.015
WEIGHT_H = 0.035
TRAVEL = 0.100
WEIGHT_MASS = 0.045
FRAME_MASS = 0.070
TOPPER_XY = (0.36, 0.10)
GRIP_Z = 0.042  # where the demo holds the collar, in the topper's frame

# -- distractors ---------------------------------------------------------------------------------
PLATE_XY = (0.27, -0.17)

# -- scoring thresholds ---------------------------------------------------------------------------------------------------
DROP_FULL = 0.080
DROP_MAX_S = 0.4
CENTRE_TOL = 0.003
TILT_TOL = 8.0
SEAT_AXIAL = 0.006
SEAT_LATERAL = 0.012
EGG_OUT = 0.012  # the egg's centre more than this from the cup's axis: knocked out of its cup
EGG_TILT = 25.0
CUP_TILT = 20.0
TOPPER_OFF = 0.05
CAP_OFF = 0.03
STAND_TILT = 15.0
STAND_Z = 0.015
CLEAR = 0.03
STILL_V = 0.01

LOOKS = {
    "steel": m.Look(color=m.rgb(0.78, 0.78, 0.80), metallic=0.9, roughness=0.25),
    "silicone": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.7),
    "porcelain": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.25),
    "shell": m.Look(color=m.rgb(0.93, 0.84, 0.70), roughness=0.65),
    "white": m.Look(color=m.rgb(0.98, 0.98, 0.96), roughness=0.5),
}


def egg_r(z: float) -> float:
    c = EGG_CB if z < 0 else EGG_CT
    return EGG_A * math.sqrt(max(0.0, 1 - (z / c) ** 2))


def _egg_profile(z0: float, z1: float, n: int = 14):
    zs = np.linspace(z0, z1, n)
    return [(round(egg_r(float(z)), 6), float(z)) for z in zs]


_lower_prof = [(0.0, -EGG_CB)] + [p for p in _egg_profile(-EGG_CB, Z_CUT, 22)[1:]] + [(0.0, Z_CUT)]
_cap_prof = [(0.0, Z_CUT)] + _egg_profile(Z_CUT, EGG_CT, 8)[:-1] + [(0.0, EGG_CT)]
LOWER_MESH = m.lathe(_lower_prof, 32)
CAP_MESH = m.lathe(_cap_prof, 32)

EGG_POS = (CUP_XY[0], CUP_XY[1], TOP + EGG_SEAT)
EGG = m.free("egg", EGG_POS, [("shell", LOWER_MESH, "shell")], [m.col_hull("shell_c", LOWER_MESH)], LOOKS, mass=LOWER_MASS,
             friction=(0.8, 0.7), torsional_patch=0.008)
CAP = m.free("cap", EGG_POS, [("shell", CAP_MESH, "shell")], [m.col_hull("cap_c", CAP_MESH)], LOOKS, mass=CAP_MASS,
             friction=(0.8, 0.7), torsional_patch=0.006)


# egg cup: a lathed foot, stem and bowl; colliders: foot, stem, floor, and a ring of wall segments round the egg
def _cup_parts():
    outer = [(0.0, 0.0), (0.022, 0.0), (0.022, 0.004), (0.012, 0.010), (0.009, 0.018), (0.012, 0.030), (0.019, 0.040),
             (CUP_RIM_OUT, 0.052), (CUP_RIM_OUT - 0.001, CUP_H), (CUP_RIM_IN, CUP_H), (CUP_RIM_IN - 0.002, 0.046),
             (0.012, CUP_FLOOR + 0.002), (0.0, CUP_FLOOR)]
    return [("cup", m.lathe(outer, 40), "porcelain")]


def _ring_segments(prefix, r_in, r_out, z0, z1, n, r_in_bottom=None):
    out = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        rb = r_in if r_in_bottom is None else r_in_bottom
        pts = []
        for a in (a0, a1):
            c, s = math.cos(a), math.sin(a)
            pts += [(rb * c, rb * s, z0), (r_out * c, r_out * s, z0), (r_in * c, r_in * s, z1), (r_out * c, r_out * s, z1)]
        quads = [(0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5), (0, 2, 3, 1), (4, 5, 7, 6)]
        faces = [t for a, b, c, e in quads for t in ((a, b, c), (a, c, e))]
        out.append(m.col_hull(f"{prefix}{i}", m.Mesh(np.array(pts), np.array(faces))))
    return out


CUP_COLS = ([m.col_cylinder("foot_c", 0.022, 0.004, (0, 0, 0.002)), m.col_cylinder("stem_c", 0.011, 0.028, (0, 0, 0.018)),
             m.col_cylinder("floor_c", 0.017, 0.004, (0, 0, CUP_FLOOR - 0.002))]
            + _ring_segments("wall_c", CUP_RIM_IN, CUP_RIM_OUT, 0.036, CUP_H, 16, r_in_bottom=0.013))
EGG_CUP = m.free("egg_cup", (CUP_XY[0], CUP_XY[1], TOP), _cup_parts(), CUP_COLS, LOOKS, mass=CUP_MASS, friction=(0.9, 0.8))


# topper: frame (bell, collar, rod, knob) and the weight on a slider along its rod
def _topper():
    bell = m.lathe([(RING_R, 0.0), (BELL_R, 0.0), (BELL_R, 0.012), (0.017, BELL_H), (COLLAR_R, BELL_H), (COLLAR_R, BELL_H - 0.002),
                    (0.0175, 0.0145), (0.0185, 0.004), (RING_R, 0.002)], 40)
    collar = m.cylinder(COLLAR_R, COLLAR_Z[1] - COLLAR_Z[0], (0, 0, sum(COLLAR_Z) / 2))
    rod = m.cylinder(ROD_R, ROD_TOP - COLLAR_Z[1], (0, 0, (ROD_TOP + COLLAR_Z[1]) / 2), segments=16)
    knob = m.cylinder(KNOB[0], KNOB[1], (0, 0, ROD_TOP + KNOB[1] / 2))
    frame_parts = [("bell", bell, "steel"), ("collar", collar, "silicone"), ("rod", rod, "steel"), ("knob", knob, "steel")]
    frame_cols = (_ring_segments("bell_c", RING_R, BELL_R, 0.0, 0.016, 14)
                  + [m.col_cylinder("top_c", BELL_R - 0.001, 0.004, (0, 0, 0.018)),
                     m.col_cylinder("collar_c", COLLAR_R, COLLAR_Z[1] - COLLAR_Z[0], (0, 0, sum(COLLAR_Z) / 2)),
                     m.col_cylinder("rod_c", ROD_R, ROD_TOP - COLLAR_Z[1], (0, 0, (ROD_TOP + COLLAR_Z[1]) / 2)),
                     m.col_cylinder("knob_c", KNOB[0], KNOB[1], (0, 0, ROD_TOP + KNOB[1] / 2))])
    wz = COLLAR_Z[1] + WEIGHT_H / 2
    wmesh = m.lathe([(ROD_R + 0.0005, COLLAR_Z[1]), (WEIGHT_R - 0.002, COLLAR_Z[1]), (WEIGHT_R, COLLAR_Z[1] + 0.003),
                     (WEIGHT_R, COLLAR_Z[1] + WEIGHT_H - 0.003), (WEIGHT_R - 0.002, COLLAR_Z[1] + WEIGHT_H), (ROD_R + 0.0005, COLLAR_Z[1] + WEIGHT_H),
                     (ROD_R + 0.0005, COLLAR_Z[1])], 32)
    weight = m.slider("weight", "Z", 0.0, TRAVEL, [("weight", wmesh, "steel")],
                      [m.col_cylinder("weight_c", WEIGHT_R, WEIGHT_H, (0, 0, wz))], at=(0, 0, wz),
                      mass=WEIGHT_MASS, damping=0.002, friction=0.005, gravity=True, com=(0, 0, wz),
                      inertia=(5e-6, 5e-6, 6e-6))
    return m.mechanism("topper", (TOPPER_XY[0], TOPPER_XY[1], TOP), [weight], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols,
                       free_mass=FRAME_MASS, frame_inertia=(1.6e-4, 1.6e-4, 1.2e-5), friction=(0.9, 0.8))


TOPPER = _topper()

# distractors (scoring ignores them)
PLATE = SimReady("plate", pos=(PLATE_XY[0], PLATE_XY[1], TOP), size=(0.172, 0.177, 0.026), mass=0.15,
                 package="dishware_plastic_bigPlate_a01", usd="sm_dishware_plastic_bigPlate_a01_01.usd")
TEASPOON = SimReady("teaspoon", pos=(0.16, 0.22, TOP), size=(0.035, 0.191, 0.014), mass=0.02, package="kitchenware_spoon_b01",
                    usd="sm_kitchenware_spoon_b01_01.usd", yaw=0.3, convex=True)
TEACUP = SimReady("teacup", pos=(0.50, 0.25, TOP), size=(0.119, 0.084, 0.102), mass=0.20, package="dishware_cup_tea_a01",
                  usd="sm_dishware_cup_tea_a01_01.usd", yaw=-math.pi / 2, convex=True)
SHAKER = SimReady("salt_shaker", pos=(0.47, -0.04, TOP), size=(0.062, 0.062, 0.110), mass=0.12, package="kitchenware_shaker_a01",
                  usd="sm_kitchenware_shaker_a01_01.usd")

OBJECTS = [EGG_CUP, EGG, CAP, TOPPER, PLATE, TEASPOON, TEACUP, SHAKER]


# -- geometry helpers -----------------------------------------------------------------------------------------------------
def egg_gap(egg_state, points):
    """Approximate distance of world points from the whole egg's surface (two half-ellipsoids)."""
    local = egg_state.to_local(points)
    c = np.where(local[:, 2] < 0, EGG_CB, EGG_CT)
    k = np.linalg.norm(local / np.c_[np.full(len(local), EGG_A), np.full(len(local), EGG_A), c], axis=1)
    return (k - 1.0) * np.linalg.norm(local, axis=1) / np.maximum(k, 1e-9)


def topper_gap(t_state, points):
    local = t_state.to_local(points)
    r = np.hypot(local[:, 0], local[:, 1])
    dr = np.maximum(r - BELL_R, 0.0)
    dz = np.maximum(np.maximum(-local[:, 2], local[:, 2] - (ROD_TOP + KNOB[1])), 0.0)
    return np.hypot(dr, dz)


def seat(egg_state, t_state):
    """The bell's ring centre in the egg's frame: lateral offset (m), axial height (m), and the axis angle (degrees)."""
    p = egg_state.to_local(np.asarray(t_state.pos)[None])[0]
    axis_e = egg_state.rotation()[:, 2]
    axis_t = t_state.rotation()[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(axis_e @ axis_t)))))
    return float(math.hypot(p[0], p[1])), float(p[2]), tilt


def _np(t):
    return t.detach().cpu().numpy().astype(float)


def _pose7(state):
    return np.r_[state.pos, state.quat_xyzw]


def _compose(a, b):
    """Pose a (world) times pose b (in a's frame), both [x, y, z, qx, qy, qz, qw]."""
    ra = m.quat_matrix(a[3:])
    rot = ra @ m.quat_matrix(b[3:])
    return np.r_[a[:3] + ra @ b[:3], _quat_from_matrix(rot)]


def _relative(a, b):
    """Pose b expressed in pose a's frame."""
    ra = m.quat_matrix(a[3:])
    return np.r_[ra.T @ (b[:3] - a[:3]), _quat_from_matrix(ra.T @ m.quat_matrix(b[3:]))]


def _quat_from_matrix(r):
    w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])


def _carry_cap(env, host, rel):
    """Hold the cap at `rel` in `host`'s frame, moving with it: on the lower shell while the shell is whole, in the bell (gripped by
    its teeth) once the ring is cut."""
    import torch

    base = env.unwrapped
    cap, h = base.scene["cap"], base.scene[host]
    origin = _np(base.scene.env_origins[0])
    pose = _compose(_pose7(m.state(env, host)), rel)
    pose[:3] += origin
    if host == "topper":  # a free mechanism: its frame body's velocity
        i = h.body_names.index("frame")
        vel = h.data.body_link_vel_w[:, i].clone()
    else:
        vel = h.data.root_vel_w.clone()
    cap.write_root_pose_to_sim_index(root_pose=torch.tensor([pose.tolist()], dtype=torch.float32, device=cap.device))
    cap.write_root_velocity_to_sim_index(root_velocity=vel)


# -- the tracker ----------------------------------------------------------------------------------------------------------
class Breakfast:
    """Records the drops, the shell's state and the end state. Harm (a cracked side, the egg out of its cup) is recorded across
    steps and cleared only in `reset`."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.scored = False
        self.cracked = False
        self.knocked = False
        self.drops = []  # (height, lateral, axial, tilt, outcome)
        self.prev_q = 0.0
        self.cap_in_bell = None  # the cap's pose in the topper's frame once the ring is cut
        self.fall_start = None  # (step, q) where the current descent began
        self.k = 0
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}

    def step(self, env):
        self.k += 1
        egg, cap, cup, top = (m.state(env, n) for n in ("egg", "cap", "egg_cup", "topper"))
        q = top.joint("weight")
        lat, ax, tilt = seat(egg, top)
        on_egg = abs(ax - Z_RING) <= SEAT_AXIAL and lat <= SEAT_LATERAL

        # drops: a descent from its highest point to the rest, in at most DROP_MAX_S
        if q >= self.prev_q - 1e-4 or self.fall_start is None:  # not falling: a descent starts from here
            self.fall_start = (self.k, q)
        elif q <= 0.002 < self.prev_q:
            k0, h = self.fall_start
            if (self.k - k0) / 20.0 <= DROP_MAX_S and h >= DROP_FULL and on_egg and not self.scored and not self.cracked:
                clean = lat <= CENTRE_TOL and tilt <= TILT_TOL
                if clean:
                    self.cap_in_bell = _relative(_pose7(top), _pose7(cap))
                self.scored = self.scored or clean
                self.cracked = self.cracked or not clean
                self.drops.append((round(h, 3), round(lat * 1000, 1), round((ax - Z_RING) * 1000, 1), round(tilt, 1), "clean" if clean else "cracked"))
            self.fall_start = (self.k, q)
        self.prev_q = q
        if self.scored:
            _carry_cap(env, "topper", self.cap_in_bell)
        else:
            _carry_cap(env, "egg", np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))

        # the egg in its cup
        rel = egg.pos - cup.pos
        egg_in = math.hypot(rel[0], rel[1]) <= EGG_OUT and egg.tilt_deg() <= EGG_TILT and cup.tilt_deg() <= CUP_TILT
        if not egg_in:
            self.knocked = True

        tips = m.fingertips(env)
        cap_off = float(np.linalg.norm(cap.pos - egg.pos)) > CAP_OFF
        topper_off = float(np.linalg.norm(top.pos - egg.to_world(np.array([[0.0, 0.0, Z_RING]]))[0])) > TOPPER_OFF
        cap_centroid = cap.to_world(np.array([[0.0, 0.0, Z_CUT + 0.004]]))[0]
        bell = top.to_world(np.array([[0.0, 0.0, 0.0]]))[0]
        standing = top.tilt_deg() <= STAND_TILT and bell[2] <= TOP + STAND_Z  # stood back up on the table
        cap_gap = float(np.linalg.norm(tips - cap_centroid, axis=1).min()) - R_CUT
        e_gap = float(egg_gap(egg, tips).min())
        t_gap = float(topper_gap(top, tips).min())
        clear = cap_gap >= CLEAR and e_gap >= CLEAR and t_gap >= CLEAR
        still = all(float(np.linalg.norm(s.lin_vel)) <= STILL_V for s in ((egg, top) if self.scored else (cap, egg, top)))  # a cut cap rides in the bell
        harm = self.cracked or self.knocked
        ok = self.scored and cap_off and topper_off and standing and egg_in and clear and still and not harm
        self.hold = self.hold + 1 if ok else 0
        self.full = (self.full or self.hold >= HOLD_STEPS) and not harm
        if harm or not self.scored:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif cap_off and topper_off and standing:
            self.score = 0.75
        elif cap_off and topper_off:
            self.score = 0.5
        else:
            self.score = 0.25
        self.last = {"k": self.k, "q": round(q, 4), "lat_mm": round(lat * 1000, 1), "ax_mm": round((ax - Z_RING) * 1000, 1), "tilt": round(tilt, 1),
                     "on_egg": on_egg, "scored": self.scored, "cracked": self.cracked, "knocked": self.knocked, "drops": self.drops,
                     "cap_off": cap_off, "topper_off": topper_off, "standing": standing, "clear": clear, "still": still,
                     "gaps": [round(cap_gap, 3), round(e_gap, 3), round(t_gap, 3)], "hold": self.hold, "score": self.score}


TRIAL = Breakfast()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (closed loop, demokit) -----------------------------------------------------------------------------
def _report(ctx, tag):
    t = ctx.state("topper")
    e = ctx.state("egg")
    lat, ax, tilt = seat(e, t)
    print(f"[egg] {tag} topper {np.round(t.pos, 4).tolist()} tilt {t.tilt_deg():.1f} weight {t.joint('weight'):.4f} | egg {np.round(e.pos, 4).tolist()} "
          f"tilt {e.tilt_deg():.1f} | cap {np.round(ctx.pos('cap'), 4).tolist()} plate {np.round(ctx.pos('plate'), 3).tolist()} | seat lat {lat * 1000:.1f} mm ax {(ax - Z_RING) * 1000:.1f} mm tilt {tilt:.1f}", flush=True)


def demo_script(ctx):
    """Left: takes the topper by its collar (fingers level, from the left), carries it over the egg and lowers the bell onto the
    crown, servoing the ring's centre onto the egg's axis from the seen poses, and holds it there. Right: takes the weight (level,
    from the right), lifts it to the top of the rod and lets go, and moves away. Left lifts the topper off, the cap in its bell, stands
    it back up on the table where it was and moves away.
    Red-team modes: none, short (the weight lifted only 5 cm), offcentre (the bell held 5 mm off the axis), onearm (the bell set on
    the egg and let go; the same arm then lifts the weight), grabcap (the cap pulled off without the topper), lowered (the weight
    lifted the full height and carried back down instead of dropped)."""
    import demokit as d

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "short", "offcentre", "onearm", "grabcap", "lowered"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        yield from r.wait(3.0)
        _report(ctx, "end:")
        return

    def egg_axis_point(z):
        e = ctx.state("egg")
        return e.to_world(np.array([[0.0, 0.0, z]]))[0]

    def topper_point(z):
        t = ctx.state("topper")
        return t.to_world(np.array([[0.0, 0.0, z]]))[0]

    if mode == "grabcap":
        c = ctx.pos("cap")
        cz = c[2] + Z_CUT
        yield from r.move((c[0], c[1], cz + 0.06), grip=d.down(0), t=2.5, label="over the cap")
        yield from r.move((c[0], c[1], cz + 0.0115), t=1.5, label="down to the cap")
        yield from r.close(0.028, force=12.0, label="pinch the cap")
        yield from r.move((c[0], c[1], cz + 0.08), t=1.5, label="pull")
        yield from r.move((PLATE_XY[0], PLATE_XY[1], TOP + 0.08), t=2.0, label="to the plate")
        yield from r.open()
        yield from r.move((0.20, -0.30, TOP + 0.20), t=1.5, label="away")
        yield from r.wait(2.0)
        _report(ctx, "end:")
        return

    gl = d.level(-90)
    gr = d.level(90)
    arm = l
    # 1. take the topper by its collar: down round the rod from above, fingers level
    HIGH = TOP + ROD_TOP + KNOB[1] + 0.035
    tp = topper_point(GRIP_Z)
    yield from l.move((tp[0], tp[1], HIGH), grip=gl, t=3.0, label="over the topper")
    yield from l.servo(lambda s: (lambda p: np.array([p[0], p[1], HIGH]))(topper_point(GRIP_Z)), timeout=1.0, label="over the topper")
    yield from l.servo(lambda s: topper_point(GRIP_Z) + np.array([0.0, 0.0, max(0.0, (HIGH - TOP - GRIP_Z) * (1 - s / 2.5))]), timeout=4.0, label="down round the collar")
    yield from l.close(0.022, force=20.0, label="hold the collar")
    yield from l.wait(0.3)
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.08]), t=1.5, label="lift the topper")
    _report(ctx, "lifted:")
    # 2. over the egg, then down onto the crown, centring the ring on the egg's axis
    off = np.array([0.0, 0.005, 0.0]) if mode == "offcentre" else np.zeros(3)
    above = egg_axis_point(Z_RING) + off + np.array([0.0, 0.0, 0.04])
    yield from l.move(l.at() + (above - ctx.pos("topper")), t=3.0, label="over the egg")
    yield from l.servo(lambda s: egg_axis_point(Z_RING) + off + np.array([0.0, 0.0, 0.03]), read=lambda: ctx.pos("topper"), timeout=2.0, tol=0.002, label="centre over")
    yield from l.servo(lambda s: egg_axis_point(Z_RING) + off + np.array([0.0, 0.0, max(-0.001, 0.03 - 0.02 * s)]), read=lambda: ctx.pos("topper"), timeout=3.5, tol=0.0015,
                       label="down onto the crown")
    yield from l.wait(0.5)
    _report(ctx, "seated:")
    if mode == "onearm":
        yield from l.open()
        yield from l.move(l.at() + np.array([0.0, 0.08, 0.0]), t=1.0, label="out")
        _report(ctx, "let go:")
        arm, g = l, gl
        side = 1.0
    else:
        arm, g, side = r, gr, -1.0
    # 3. lift the weight to the top of the rod and let it fall
    lift = 0.05 if mode == "short" else TRAVEL - 0.004
    wz = COLLAR_Z[1] + WEIGHT_H / 2
    wp = topper_point(wz)
    yield from arm.move((wp[0], wp[1] + side * 0.10, wp[2] + 0.01), grip=g, t=3.0, label="beside the weight")
    yield from arm.move((wp[0], wp[1] + side * 0.10, wp[2]), t=0.8, label="level with the weight")
    yield from arm.servo(lambda s: topper_point(wz) + np.array([0.0, side * max(0.0, 0.10 - 0.10 * s / 1.5), 0.0]), timeout=3.0, label="in round the weight")
    yield from arm.close(0.030, force=15.0, label="pinch the weight")
    yield from arm.wait(0.3)
    yield from arm.servo(lambda s: topper_point(wz) + np.array([0.0, 0.0, min(lift, lift * s / 2.0)]), read=lambda: topper_point(wz + ctx.state("topper").joint("weight")),
                         timeout=4.0, tol=0.003, label="lift the weight")
    _report(ctx, "weight up:")
    if mode == "lowered":  # carried back down by hand instead of dropped
        yield from arm.servo(lambda s: topper_point(wz) + np.array([0.0, 0.0, max(0.0, lift * (1 - s / 2.0))]), read=lambda: topper_point(wz + ctx.state("topper").joint("weight")),
                             timeout=3.0, tol=0.003, label="lower the weight")
    yield from arm.open(t=0.15, label="let go")
    yield from arm.wait(0.8)
    _report(ctx, "dropped:")
    yield from arm.move(arm.at() + np.array([0.0, side * 0.08, 0.02]), t=1.0, label="out")
    if mode == "onearm":
        yield from l.move((0.25, 0.26, TOP + 0.22), grip=d.down(0), t=2.0, label="away")
        yield from l.wait(2.0)
        _report(ctx, "end:")
        return
    yield from r.move(r.at() + np.array([0.0, -0.06, 0.04]), t=1.0, label="out")
    yield from r.move((0.30, -0.30, TOP + 0.20), grip=d.down(0), t=2.5, label="away")
    # 4. lift the topper off (the cut cap comes up in its bell) and stand it back on the table where it was
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.07]), t=1.5, label="topper up")
    _report(ctx, "topper up:")
    yield from l.move((TOPPER_XY[0], TOPPER_XY[1], l.at()[2]), t=2.5, label="topper back")
    yield from l.servo(lambda s: np.array([TOPPER_XY[0], TOPPER_XY[1], TOP + 0.002 + max(0.0, 0.05 - 0.04 * s)]), read=lambda: ctx.pos("topper"), timeout=2.5, tol=0.003,
                       label="topper down")
    yield from l.open()
    yield from l.move((l.at()[0], l.at()[1], HIGH), t=2.0, label="up off the rod")
    yield from l.move(l.at() + np.array([0.0, 0.10, 0.0]), t=1.5, label="clear of the rod")
    yield from l.move((0.25, 0.28, TOP + 0.22), grip=d.down(0), t=2.5, label="away")
    yield from l.wait(2.0)
    _report(ctx, "end:")
