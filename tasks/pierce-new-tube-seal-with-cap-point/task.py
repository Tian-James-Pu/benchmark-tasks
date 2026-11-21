"""Open a new, membrane-sealed aluminium tube of glue: unscrew its cap, turn the cap over and push the point on its top
through the seal in the neck, then screw the cap back down.

A 30 ml aluminium tube (25 mm across, 100 mm long, a factory-sealed nozzle "to be pierced with cap") stands upright, its
flat crimped end pushed into a saw kerf in a pine block, which stands in for the hand that holds the tube. Its white
ribbed polypropylene cap is screwed down on the M11 neck and carries a piercing point on its top. The neck's 4.5 mm
orifice is closed by the tube's own aluminium membrane.

Model (declared here and in the README):
- The cap is a free rigid body. Its thread is a step hook. While the cap is on the thread, the hook keeps it on the
  neck's axis and upright, at the height that its anticlockwise turn from fully down sets (1.5 mm per turn), and lets it
  turn only between fully down and 1.5 turns out. Turned out past 1.5 turns, it is off the thread and rests on the
  thread's crest until lifted. A cap set back on the neck, open end down and within 2 mm of its axis, rests on the crest;
  turning it clockwise there takes up the thread again. Pulling or tilting cannot take it off the thread.
- The membrane is a sprung slider in the neck's bore (5,000 N/m), 0.5 mm below the rim. Pushed 2 mm in (10 N), it tears
  and stays open. Only something narrower than the 4.5 mm bore reaches it: the cap's point.
- Squeezing: fingertip points of one gripper on both sides of the tube's body, each within 4 mm of its surface, for 5
  control steps running (0.25 s), latch a squeeze for the trial. The record is cleared only at a reset.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Open this new tube of glue so it is ready to use, then put its cap back on."
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

# -- the holder: a pine block with a saw kerf that takes the tube's crimped end
TX, TY = 0.30, -0.12  # the tube's axis
BLOCK = (0.05, 0.09, 0.03)  # x, y, z
KERF = 0.003
BZ = TOP + BLOCK[2]  # the block's top: the tube's frame origin

# -- the tube (its frame: origin on the tube's axis at the block's top). 25 mm x 100 mm, 30 ml
R_BODY = 0.0125
CRIMP_W, CRIMP_T, CRIMP_D = 0.040, 0.0015, 0.020  # the folded end: 40 mm wide, 20 mm of it in the kerf
Z_ROUND = 0.030  # the body is round above this, flattening to the crimp below
Z_SHOULDER0, Z_SHOULDER1 = 0.077, 0.081  # 80 mm of body (plus 20 mm in the kerf), then the shoulder
R_NECK, R_BORE = 0.0055, 0.00225  # M11 neck, 4.5 mm orifice
Z_RIM = 0.090
SEAL_R, SEAL_T, SEAL_DEPTH = 0.0020, 0.0006, 0.0005
SEAL_K, SEAL_TEAR = 5000.0, 0.002  # N/m and m: tears at 10 N

# -- the cap (its frame: origin on its axis at the skirt's open end)
CAP_R, CAP_BORE, SKIRT_H, CAP_TOP = 0.0075, 0.0060, 0.0085, 0.003  # 15 mm across, 11.5 mm tall
SPIKE_R, SPIKE_H = 0.00175, 0.005  # the point: 3.5 mm at its base, 5 mm tall
CAP_H = SKIRT_H + CAP_TOP
CAP_MASS = 0.002
CAP_SEATED_Z = BZ + Z_RIM - SKIRT_H  # its origin when screwed down: inner top on the rim
LEAD = 0.0015  # m per turn
OFF_DEG = 540.0  # 1.5 turns from fully down to off the thread
CREST_Z = CAP_SEATED_Z + LEAD * OFF_DEG / 360.0  # where a cap off the thread rests on the neck
DOWN_DEG = 90.0  # "screwed down": within a quarter turn of fully down (0.375 mm)
CAPTURE_R = 0.0025  # a cap set on the neck this close to its axis finds the thread's lead-in

SQUEEZE_STEPS = 5
SQUEEZE_GAP = 0.004
CLEAR = 0.03
HOLD_STEPS = 20  # 1 s at 20 Hz

LOOKS = {
    "alu": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.4, metallic=0.4),
    "label": m.Look(color=m.rgb(0.95, 0.75, 0.10), roughness=0.5),
    "crimp": m.Look(color=m.rgb(0.80, 0.80, 0.80), roughness=0.4, metallic=0.8),
    "neck": m.Look(color=m.rgb(0.85, 0.85, 0.85), roughness=0.4, metallic=0.6),
    "bore": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.9),
    "foil": m.Look(color=m.rgb(0.80, 0.80, 0.82), roughness=0.2, metallic=1.0),
    "torn": m.Look(color=m.rgb(0.30, 0.30, 0.32), roughness=0.3, metallic=1.0),
    "cap": m.Look(color=m.rgb(0.96, 0.96, 0.95), roughness=0.5),
    "pine": m.Look(color=m.rgb(0.80, 0.64, 0.42), roughness=0.8),
}


def _sector(i: int, n: int, r0: float, r1: float, z0: float, z1: float) -> m.Mesh:
    a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
    pts = [(r0 * math.cos(a0), r0 * math.sin(a0)), (r1 * math.cos(a0), r1 * math.sin(a0)),
           (r1 * math.cos(a1), r1 * math.sin(a1)), (r0 * math.cos(a1), r0 * math.sin(a1))]
    return m.prism(pts, z0, z1)


def _ring(prefix: str, n: int, r0: float, r1: float, z0: float, z1: float) -> list[str]:
    return [m.col_hull(f"{prefix}{i}", _sector(i, n, r0, r1, z0, z1)) for i in range(n)]


def _loft(z0: float, z1: float, rings: int = 10, seg: int = 32) -> m.Mesh:
    """The body from the crimp (flat, wide along y) up to round."""
    vs, fs = [], []
    for z in np.linspace(z0, z1, rings):
        s = (z - z0) / (z1 - z0)
        a = CRIMP_W / 2 + (R_BODY - CRIMP_W / 2) * s ** 0.7  # half-width along y
        b = CRIMP_T / 2 + (R_BODY - CRIMP_T / 2) * s ** 0.7  # half-thickness along x
        for k in range(seg):
            t = 2 * math.pi * k / seg
            vs.append((b * math.cos(t), a * math.sin(t), z))
    for r in range(rings - 1):
        for k in range(seg):
            a0, a1 = r * seg + k, r * seg + (k + 1) % seg
            fs += [(a0, a1, a1 + seg), (a0, a1 + seg, a0 + seg)]
    vs += [(0, 0, z0), (0, 0, z1)]
    c0, c1, top = len(vs) - 2, len(vs) - 1, (rings - 1) * seg
    for k in range(seg):
        fs += [(c0, (k + 1) % seg, k), (c1, top + k, top + (k + 1) % seg)]
    return m.Mesh(np.array(vs, float), np.array(fs, int))


SEAL_Z = Z_RIM - SEAL_DEPTH - SEAL_T / 2


def _tube():
    loft = _loft(0.0, Z_ROUND)
    shoulder = m.lathe([(R_BODY, Z_SHOULDER0), (0.0068, Z_SHOULDER1), (0.0, Z_SHOULDER1)], segments=48)
    torn = m.join(*[m.box((0.0016, 0.0005, 0.0025), (0, 0, 0)).turned(20, "x").turned(a, "z").moved(
        (0.0013 * math.cos(math.radians(a + 90)), 0.0013 * math.sin(math.radians(a + 90)), Z_RIM - 0.0022)) for a in (0, 90, 180, 270)])
    parts = [
        ("crimp", m.box((CRIMP_T, CRIMP_W, CRIMP_D), (0, 0, -CRIMP_D / 2)), "crimp"),
        ("lower", loft, "alu"),
        ("body", m.cylinder(R_BODY, Z_SHOULDER0 - Z_ROUND, (0, 0, (Z_ROUND + Z_SHOULDER0) / 2), segments=48), "alu"),
        ("label", m.cylinder(R_BODY + 0.0003, 0.030, (0, 0, 0.052), segments=48), "label"),
        ("shoulder", shoulder, "neck"),
        ("neck", m.lathe([(R_NECK, Z_SHOULDER1), (R_NECK, Z_RIM), (R_BORE, Z_RIM), (R_BORE, Z_SHOULDER1 + 0.0005),
                          (0.0, Z_SHOULDER1 + 0.0005)], segments=32), "neck"),
        ("bore", m.cylinder(R_BORE - 0.0001, 0.0004, (0, 0, Z_SHOULDER1 + 0.0007)), "bore"),
        ("torn", torn, "torn", True),
    ]
    cols = [
        m.col_box("crimp_c", (CRIMP_T, CRIMP_W, CRIMP_D), (0, 0, -CRIMP_D / 2)),
        m.col_hull("lower_c", loft),
        m.col_cylinder("body_c", R_BODY, Z_SHOULDER0 - Z_ROUND, (0, 0, (Z_ROUND + Z_SHOULDER0) / 2)),
        m.col_hull("shoulder_c", m.lathe([(R_BODY, Z_SHOULDER0), (0.0068, Z_SHOULDER1), (0.0, Z_SHOULDER1)], segments=24)),
        m.col_cylinder("bore_floor_c", R_BORE, 0.0005, (0, 0, Z_SHOULDER1 + 0.00025)),
    ] + _ring("neck_c", 16, R_BORE, R_NECK, Z_SHOULDER1, Z_RIM)
    seal = m.slider("seal", "Z", -0.006, 0.0,
                    [("foil", m.cylinder(SEAL_R, SEAL_T, (0, 0, SEAL_Z)), "foil")],
                    [m.col_cylinder("seal_c", SEAL_R, SEAL_T, (0, 0, SEAL_Z))],
                    at=(0, 0, SEAL_Z), spring=SEAL_K, damping=2.0, friction=0.0, mass=0.01, effort=200.0)
    return m.mechanism("tube", (TX, TY, BZ), [seal], LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.3, 0.25))


def _cap():
    z0 = CAP_H
    star = []
    for k in range(8):  # a four-ribbed point, tapering to its tip
        r = SPIKE_R if k % 2 == 0 else SPIKE_R * 0.6
        a = k * math.pi / 4
        star.append((r * math.cos(a), r * math.sin(a)))
    vs = [(x, y, z0) for x, y in star] + [(0, 0, z0 + SPIKE_H), (0, 0, z0)]
    fs = [(8, k, (k + 1) % 8) for k in range(8)] + [(9, (k + 1) % 8, k) for k in range(8)]
    spike = m.Mesh(np.array(vs, float), np.array(fs, int))
    body = m.lathe([(CAP_BORE, 0.0), (CAP_R, 0.0), (CAP_R, CAP_H), (0.0, CAP_H), (0.0, SKIRT_H), (CAP_BORE, SKIRT_H)], segments=48)
    ribs = m.join(*[m.box((0.0006, 0.001, CAP_H - 0.0015), (0, 0, CAP_H / 2)).moved((CAP_R, 0, 0)).turned(a, "z") for a in range(0, 360, 15)])
    cone = m.lathe([(SPIKE_R, z0), (0.0003, z0 + SPIKE_H - 0.0002), (0.0, z0 + SPIKE_H - 0.0002)], segments=16)
    gon = [(CAP_R * math.cos(2 * math.pi * k / 16), CAP_R * math.sin(2 * math.pi * k / 16)) for k in range(16)]
    cols = _ring("skirt_c", 16, CAP_BORE, CAP_R, 0.0, SKIRT_H) + [
        m.col_hull("top_c", m.prism(gon, SKIRT_H, CAP_H)),
        m.col_hull("spike_c", cone)]
    return m.free("cap", (TX, TY, CAP_SEATED_Z), [("body", body, "cap"), ("ribs", ribs, "cap"), ("spike", spike, "cap")], cols, LOOKS,
                  mass=CAP_MASS, friction=(1.0, 0.9), torsional_patch=0.004, solver_iters=48)


def _holder():
    half = (BLOCK[0] - KERF) / 2
    parts, cols = [], []
    for s, nm in ((-1, "a"), (1, "b")):
        c = (s * (KERF / 2 + half / 2), 0.0, -BLOCK[2] / 2)
        parts.append((f"half_{nm}", m.box((half, BLOCK[1], BLOCK[2]), c), "pine"))
        cols.append(m.col_box(f"half_{nm}_c", (half, BLOCK[1], BLOCK[2]), c))
    floor = (0.0, 0.0, -BLOCK[2] + (BLOCK[2] - CRIMP_D) / 2)
    parts.append(("kerf_floor", m.box((KERF, BLOCK[1], BLOCK[2] - CRIMP_D), floor), "pine"))
    return m.fixed("holder", (TX, TY, BZ), parts, cols, LOOKS, friction=(0.8, 0.6))


HOLDER, TUBE, CAP = _holder(), _tube(), _cap()

# -- distractors (SimReady), on a home repair table: a mug with its handle off, a roll of clear tape, a notebook
MUG = SimReady("mug", pos=(0.20, 0.10, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
               usd="sm_dishware_mug_a01_01.usda", yaw=2.6)
TAPE = SimReady("tape_roll", pos=(0.42, 0.06, TOP), size=(0.10, 0.10, 0.05), mass=0.06, package="tape_clear_a01",
                usd="sm_tape_clear_a01_01.usd")
NOTEBOOK = SimReady("notebook", pos=(0.16, 0.27, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01",
                    usd="sm_misc_notebook_a01_01.usd", yaw=0.2)

OBJECTS = [HOLDER, TUBE, CAP, MUG, TAPE, NOTEBOOK]


# -- the thread, the seal and the scorer -----------------------------------------------------------------------------

def _yaw(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _put_cap(env, z: float, yaw: float, wz: float) -> None:
    """Hold the cap on the neck's axis, upright, at height z, turning at wz about the vertical."""
    import torch

    base = env.unwrapped
    asset = base.scene["cap"]
    o = base.scene.env_origins[0]
    pose = torch.tensor([[float(o[0]) + TX, float(o[1]) + TY, float(o[2]) + z, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]],
                        dtype=torch.float32, device=asset.device)
    asset.write_root_pose_to_sim_index(root_pose=pose)
    vel = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, float(wz)]], dtype=torch.float32, device=asset.device)
    asset.write_root_velocity_to_sim_index(root_velocity=vel)


class Thread:
    """The cap's thread: 'on' (turned `turned` degrees out from fully down), 'crest' (off the thread, resting on the
    neck), or 'free'."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.mode, self.turned, self.last_yaw, self.crest_yaw = "on", 0.0, None, 0.0

    def step(self, env, cap) -> None:
        yaw = _yaw(cap.quat_xyzw)
        if self.last_yaw is None:
            self.last_yaw = yaw
        d = math.degrees(_wrap(yaw - self.last_yaw))
        self.last_yaw = yaw
        off_axis = math.hypot(cap.pos[0] - TX, cap.pos[1] - TY)
        if self.mode == "on":
            t = self.turned + d
            if t < 0.0:  # fully down: it turns no further clockwise
                yaw -= math.radians(t)
                t = 0.0
            self.turned, self.last_yaw = t, yaw
            if t > OFF_DEG:
                self.mode, self.crest_yaw = "crest", yaw
                _put_cap(env, CREST_Z, yaw, 0.0)
                return
            _put_cap(env, CAP_SEATED_Z + LEAD * t / 360.0, yaw, float(cap.ang_vel[2]))
        elif self.mode == "crest":
            if cap.pos[2] > CREST_Z + 0.002 or off_axis > CAPTURE_R or cap.tilt_deg() > 12.0:
                self.mode = "free"  # lifted off the neck
                return
            back = math.degrees(_wrap(yaw - self.crest_yaw))
            if back < -20.0:  # turned clockwise on the crest: the thread takes up
                self.mode, self.turned = "on", OFF_DEG + back
                _put_cap(env, CAP_SEATED_Z + LEAD * self.turned / 360.0, yaw, float(cap.ang_vel[2]))
            elif back > 20.0:  # turned anticlockwise on the crest: it just turns there
                self.crest_yaw = yaw - math.radians(20.0)
            if cap.pos[2] < CREST_Z - 0.0002:  # it rests on the thread's crest
                _put_cap(env, CREST_Z, yaw, float(cap.ang_vel[2]))
        else:  # free: set on the neck, open end down and close to its axis, it finds the thread's start and rests on it
            if off_axis < CAPTURE_R and cap.tilt_deg() < 12.0 and CREST_Z - 0.003 < cap.pos[2] < BZ + Z_RIM + 0.001:
                self.mode, self.crest_yaw = "crest", yaw
                _put_cap(env, CREST_Z, yaw, 0.0)


class Trial:
    def __init__(self):
        self.thread = Thread()
        self.reset()

    def reset(self, env=None):  # the ONLY place the recorded squeeze (and every other record) is cleared
        self.thread.reset(env)
        self.torn, self.squeezed, self.squeeze_run, self.came_off = False, False, 0, False
        self.hold, self.ok, self.score = 0, False, 0.0
        if env is not None:
            self.thread.last_yaw = _yaw(m.state(env, "cap").quat_xyzw)
            m.set_visible(f"{m.ENV_PATH}/tube/seal/visual/foil", True)
            m.set_visible(f"{m.ENV_PATH}/tube/frame/visual/torn", False)

    def _squeeze(self, tips: np.ndarray) -> bool:
        rel = tips - np.array([TX, TY, BZ])
        r = np.hypot(rel[:, 0], rel[:, 1])
        near = (rel[:, 2] > 0.0) & (rel[:, 2] < Z_SHOULDER0) & (r < R_BODY + SQUEEZE_GAP)
        for arm in (slice(0, 12), slice(12, 24)):  # one gripper: a finger's points, then the other's
            a, b = near[arm][:6], near[arm][6:]
            if a.any() and b.any():
                pa = rel[arm][:6][a][:, :2].mean(0)
                pb = rel[arm][6:][b][:, :2].mean(0)
                if float(np.dot(pa, pb)) < 0.0:  # on opposite sides of the body
                    return True
        return False

    def step(self, env):
        cap = m.state(env, "cap")
        self.thread.step(env, cap)
        cap = m.state(env, "cap")
        if self.thread.mode != "on":
            self.came_off = True
        tube = m.state(env, "tube")
        if not self.torn and tube.joint("seal") <= -SEAL_TEAR:
            self.torn = True
            m.set_target(env, "tube", "seal", -0.006)
            m.set_visible(f"{m.ENV_PATH}/tube/seal/visual/foil", False)
            m.set_visible(f"{m.ENV_PATH}/tube/frame/visual/torn", True)
        tips = m.fingertips(env)
        self.squeeze_run = self.squeeze_run + 1 if self._squeeze(tips) else 0
        if self.squeeze_run >= SQUEEZE_STEPS:
            self.squeezed = True
        mid = cap.to_world(np.array([[0.0, 0.0, CAP_H / 2]]))[0]
        clear = float(np.linalg.norm(tips - mid, axis=1).min()) > CLEAR
        down = self.thread.mode == "on" and self.thread.turned <= DOWN_DEG
        now = bool(self.torn and down and clear and not self.squeezed)
        self.hold = self.hold + 1 if now else 0
        self.ok = self.hold >= HOLD_STEPS
        self._n = getattr(self, "_n", 0) + 1  # DEBUG
        if self._n % 10 == 0:  # DEBUG
            print(f"[dbg] {self._n} mode {self.thread.mode} turned {self.thread.turned:.0f} seal {tube.joint('seal') * 1000:.2f} torn {self.torn} sq {self.squeezed} "
                  f"cap {np.round(cap.pos, 4).tolist()} tilt {cap.tilt_deg():.0f} ax {np.round(cap.rotation()[:, 2], 2).tolist()} "
                  f"clear {clear} hold {self.hold} score {self.score}", flush=True)  # DEBUG
        if self.squeezed:
            self.score = 0.0
        elif self.ok:
            self.score = 1.0
        elif self.torn and self.thread.mode == "on":
            self.score = 0.75
        elif self.torn:
            self.score = 0.5
        elif self.came_off:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.ok)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the scripted demonstration (open loop, right arm) ------------------------------------------------------------------
# A fixed plan from the published starting layout. yam_arena's waypoints are straight-down grips turned about the
# vertical; `_Grip` lets a waypoint carry any TCP rotation (columns: closing, finger width, approach), which
# yam_arena's own IK and interpolation then follow.
#
# 1. Unscrew: three anticlockwise strokes of 220 degrees with a straight-down grip, re-gripping between them.
# 2. Grip the loose cap with the fingers' edge at it, lift it, tip the grip level (phi 60 degrees) and lay the cap on
#    its side on the table, its point towards the robot.
# 3. Grip the lying cap from above with its point at the fingers' edge, tip the grip level again: the point is now
#    straight down below the fingers. Push it 4.5 mm into the nozzle, through the membrane, and withdraw.
# 4. Tip back and lay the cap down; grip it from the side, tip back to straight down: it is open end down again.
# 5. Set it on the neck, and screw it down with three clockwise strokes. Let go and move away.

PHI = 60.0
U = np.array([math.cos(math.radians(PHI)), math.sin(math.radians(PHI)), 0.0])  # the level grip's pointing direction
Z = np.array([0.0, 0.0, 1.0])
LAY = np.array([0.36, -0.31])  # where the cap is laid on its side (its middle)
CAP_MID = SKIRT_H / 2 + CAP_TOP / 2  # its middle above its open end: 5.75 mm
TIP_FROM_MID = CAP_H - CAP_MID + SPIKE_H  # 10.75 mm


class _Grip(float):
    """A waypoint yaw that carries a whole TCP rotation."""

    rot: np.ndarray = np.eye(3)


def _install_grips() -> None:
    import yam_arena.demo as hd

    if getattr(hd.grip_rotation, "_takes_rot", False):
        return
    plain = hd.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return plain(yaw) if rot is None else rot

    grip_rotation._takes_rot = True
    grip_rotation._plain = plain
    hd.grip_rotation = grip_rotation


def _axis_angle(axis, deg: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    t = math.radians(deg)
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * (k @ k)


def _down(yaw_deg: float) -> np.ndarray:
    import yam_arena.demo as hd

    plain = getattr(hd.grip_rotation, "_plain", hd.grip_rotation)
    return _axis_angle(Z, yaw_deg) @ plain(0.0)


def _tilt(theta_deg: float, phi_deg: float = PHI) -> np.ndarray:
    """Tipped `theta_deg` from straight down towards `phi_deg`, about the closing direction."""
    p = math.radians(phi_deg)
    return _axis_angle((math.sin(p), -math.cos(p), 0.0), theta_deg) @ _down(phi_deg)


def _g(rot: np.ndarray) -> _Grip:
    g = _Grip(0.0)
    g.rot = rot
    return g


def demo():
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    _install_grips()
    O, C = FINGER_OPEN, FINGER_CLOSED
    plan = []

    def go(t, pos, finger, label, rot):
        plan.append(Waypoint(t, tuple(float(v) for v in pos), finger, label, _g(rot)))

    def tcp_for(mid, rot, in_grip):
        """The TCP that puts the cap's middle at `mid` when it sits at `in_grip` in the gripper's frame."""
        return np.asarray(mid, float) - rot @ np.asarray(in_grip, float)

    rim = np.array([TX, TY, BZ + Z_RIM])
    mid_down = np.array([TX, TY, CAP_SEATED_Z + CAP_MID])
    centred = np.array([0.0, 0.0, 0.0065])  # cap middle 6.5 mm past the TCP, between the pads

    # 1. unscrew: three anticlockwise strokes from yaw -80 to +140 degrees
    turned = 0.0
    for k in range(3):
        mid = mid_down + Z * LEAD * min(turned, OFF_DEG) / 360.0
        p = tcp_for(mid, _down(-80), centred)
        go(3.0 if k == 0 else 0.6, p + Z * 0.03, O, f"over cap {k}", _down(-80))
        go(1.0, p, O, f"down to cap {k}", _down(-80))
        go(0.5, p, C, f"grip {k}", _down(-80))
        for a in (-25, 30, 85, 140):
            turned += 55.0
            m_ = mid_down + Z * LEAD * min(turned, OFF_DEG) / 360.0
            go(1.1, tcp_for(m_, _down(a), centred), C, f"unscrew {k}", _down(a))
        go(0.4, tcp_for(m_, _down(140), centred), O, f"let go {k}", _down(140))
        q = tcp_for(m_, _down(140), centred) + Z * 0.004
        for a in (85, 30, -25, -80):
            go(0.5, q, O, f"turn back {k}", _down(a))

    # 2. grip the loose cap (on the thread's crest) with its middle 10 mm off the TCP along the fingers' width,
    #    which tipping level turns into "near the fingers' lower edge"; lift; lay it on its side
    crest = np.array([TX, TY, CREST_Z + CAP_MID])
    edge = np.array([0.0, 0.010, 0.0065])
    p = tcp_for(crest, _down(PHI), edge)
    go(1.2, p + Z * 0.03, O, "over loose cap", _down(PHI))
    go(1.0, p, O, "down to loose cap", _down(PHI))
    go(0.5, p, C, "grip loose cap", _down(PHI))
    go(1.0, p + Z * 0.05, C, "lift cap", _down(PHI))
    lay_mid = np.array([LAY[0], LAY[1], TOP + 0.0075 + 0.0015])
    for th in (30, 60, 90):
        go(1.0, tcp_for(lay_mid + Z * 0.05, _tilt(th), edge), C, "tip level", _tilt(th))
    go(1.5, tcp_for(lay_mid, _tilt(90), edge), C, "lay down", _tilt(90))
    go(0.5, tcp_for(lay_mid, _tilt(90), edge), O, "let go laid", _tilt(90))
    go(1.0, tcp_for(lay_mid, _tilt(90), edge) + Z * 0.05, O, "up from laid", _tilt(90))

    # 3. grip it from above with its point at the fingers' edge; tip level: point straight down; pierce
    lying = np.array([LAY[0], LAY[1], TOP + 0.0075]) + LAID_SHIFT
    point_edge = np.array([0.0, 0.017 - TIP_FROM_MID, 0.007])
    p = tcp_for(lying, _down(PHI), point_edge)
    for th in (60, 30, 0):
        go(0.8, p + Z * 0.05, O, "straighten", _tilt(th))
    go(1.2, p, O, "down to lying cap", _down(PHI))
    go(0.5, p, C, "grip lying cap", _down(PHI))
    go(1.0, p + Z * 0.06, C, "lift lying cap", _down(PHI))
    tip_in = np.array([0.0, 0.017, 0.007])  # the point's tip in the gripper's frame
    over = rim + Z * 0.03
    for th in (30, 60, 90):
        go(1.0, tcp_for(over + (lying + Z * 0.06 - over) * (1 - th / 90), _tilt(th), tip_in), C, "point down", _tilt(th))
    lv = _tilt(90)
    go(1.0, tcp_for(rim + Z * 0.008, lv, tip_in) + PIERCE_TRIM, C, "over nozzle", lv)
    go(1.0, tcp_for(rim + Z * 0.002, lv, tip_in) + PIERCE_TRIM, C, "to nozzle", lv)
    go(1.5, tcp_for(rim - Z * 0.0045, lv, tip_in) + PIERCE_TRIM, C, "pierce", lv)
    go(0.5, tcp_for(rim - Z * 0.0045, lv, tip_in) + PIERCE_TRIM, C, "pierce hold", lv)
    go(1.0, tcp_for(rim + Z * 0.03, lv, tip_in) + PIERCE_TRIM, C, "withdraw", lv)

    # 4. lay it down again (point towards the robot), grip it from the side, tip back to straight down
    mid_in = np.array([0.0, 0.017 - TIP_FROM_MID, 0.007])
    lay2 = np.array([LAY[0], LAY[1], TOP + 0.0075 + 0.001])
    for th in (60, 30, 0):
        go(1.0, tcp_for(lay2 + Z * 0.05 + (rim + Z * 0.03 - lay2 - Z * 0.05) * (th / 90), _tilt(th), mid_in), C, "tip back", _tilt(th))
    go(1.2, tcp_for(lay2, _down(PHI), mid_in), C, "lay down again", _down(PHI))
    go(0.5, tcp_for(lay2, _down(PHI), mid_in), O, "let go again", _down(PHI))
    go(0.8, tcp_for(lay2, _down(PHI), mid_in) + Z * 0.05, O, "up again", _down(PHI))
    lying2 = np.array([LAY[0], LAY[1], TOP + 0.0075]) + LAID2_SHIFT
    p = tcp_for(lying2, lv, edge)
    for th in (30, 60, 90):
        go(0.8, p + Z * 0.05, O, "level grip", _tilt(th))
    go(1.2, p, O, "down beside cap", lv)
    go(0.5, p, C, "grip cap from side", lv)
    go(1.0, p + Z * 0.05, C, "lift cap again", lv)
    above = np.array([TX, TY, CREST_Z + CAP_MID + 0.03])
    for th in (60, 30, 0):
        go(1.0, tcp_for(above + (lying2 + Z * 0.05 - above) * (th / 90), _tilt(th), edge), C, "open end down", _tilt(th))

    # 5. set it on the neck, screw it down
    on = np.array([TX, TY, CREST_Z + CAP_MID + 0.001])
    go(1.5, tcp_for(on, _down(PHI), edge) + RECAP_TRIM, C, "onto neck", _down(PHI))
    go(0.5, tcp_for(on, _down(PHI), edge) + RECAP_TRIM, O, "let go on neck", _down(PHI))
    go(0.8, tcp_for(on, _down(PHI), edge) + RECAP_TRIM + Z * 0.03, O, "up from neck", _down(PHI))
    left = OFF_DEG + 30.0  # degrees to turn clockwise: the crest's 20 degrees, 1.5 turns, and 10 to spare
    yaw = PHI
    for k in range(3):
        stroke = min(220.0, left)
        start = -80.0 + stroke
        mid = mid_down + Z * LEAD * min(left, OFF_DEG) / 360.0  # the cap's height is its remaining turn
        p = tcp_for(mid, _down(start), centred)
        n = max(1, int(math.ceil(abs(start - yaw) / 60.0)))
        for j in range(1, n + 1):
            a = yaw + (start - yaw) * j / n
            go(0.5, p + Z * 0.02, O, f"turn for stroke {k}", _down(a))
        go(0.8, p, O, f"down for stroke {k}", _down(start))
        go(0.5, p, C, f"grip for stroke {k}", _down(start))
        n = max(1, int(math.ceil(stroke / 55.0)))
        for j in range(1, n + 1):
            a = start - stroke * j / n
            done = left - stroke * j / n
            m_ = mid_down + Z * LEAD * max(0.0, min(done, OFF_DEG)) / 360.0
            go(1.1 * stroke / n / 55.0, tcp_for(m_, _down(a), centred), C, f"screw down {k}", _down(a))
        yaw = -80.0
        left -= stroke
        go(0.4, tcp_for(m_, _down(-80), centred), O, f"release {k}", _down(-80))
        go(0.6, tcp_for(m_, _down(-80), centred) + Z * 0.02, O, f"up {k}", _down(-80))
        if left <= 0.0:
            break
    go(2.0, (0.22, -0.30, TOP + 0.22), O, "away", _down(0))
    go(4.0, (0.22, -0.30, TOP + 0.22), O, "rest", _down(0))
    return {"right": plan}


# Measured in the simulator (open-loop corrections): where the laid cap actually comes to rest, and small TCP trims.
LAID_SHIFT = np.zeros(3)
LAID2_SHIFT = np.zeros(3)
PIERCE_TRIM = np.zeros(3)
RECAP_TRIM = np.zeros(3)
