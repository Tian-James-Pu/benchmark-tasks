"""Push the nano-SIM out of a new triple-cut SIM carrier card, leaving the micro and mini frames in the card.

A blank triple-cut (2FF/3FF/4FF) SIM carrier card: an ISO/IEC 7810 ID-1 card, 85.6 x 54 x 0.76 mm PVC, about 5 g. Three nested
cut-outs sit near one short end: the 25 x 15 mm mini-SIM (2FF) outline in the card, the 15 x 12 mm micro-SIM (3FF) outline inside
it (a 1.5 mm ring round the nano), and the 12.3 x 8.8 mm nano-SIM (4FF) inside that, each part hanging on breakaway tabs. The card
lies on a stack of four closed notebooks on the desk, chip side up, with its cut-out end out over the stack's edge.

Model (declared in code):
- The card ("card") is a free articulation (frame 4.6 g). Its frame: origin at the centre of the card's underside; X along its
  length (-X: the cut-out end), Y across it, Z out of the chip side. Its links nest, each on a slider along Z (the card's normal):
  "mini" in the frame, "micro" in "mini", "nano" in "micro". (Three weightless locked stubs with no geometry hang on the frame
  only so that PhysX roots the articulation at the frame.)
- Tabs: each part's tabs are a stiff spring (K_TAB, 0.25 mm at 5 N) on its slider, so its slider's deflection times K_TAB is the
  load through its tabs. They give way past the part's breaking load: nano 5 N, micro 7 N, mini 9 N (estimates in the 5-8 N range
  of such tabs; the smallest part's tabs the weakest). A press on the nano alone loads all three sets in series, so the nano's give
  way first; a press on the micro ring, or one that bridges a cut line, loads the outer sets and not the nano's, so they give way.
  Torn tabs never heal: recorded until reset.
- A frame whose tabs gave way is loose on its slider (it can drop up to 3 mm out of either face, carrying whatever is still
  attached to it). A nano whose tabs gave way drags on its cut edges (EXIT_DAMP) until it is clear of the card's face (or for 0.5 s),
  then carries on as a free body ("nano_free"), which takes the nano's pose and velocity; the link is hidden and parked 0.5 m below.
- Fingertip contact is PhysX contact on the parts' own colliders; nothing is triggered by proximity.

    inspect-robots run ... -E task=tasks/sim-card-nano-punch-out/task.py (README)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Push the nano-SIM out of the card, leaving the micro and mini frames in the card."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]

# -- the card (ISO/IEC 7810 ID-1, triple-cut), in its own frame -------------------------------------------------------------------
CL, CW, CT = 0.0856, 0.0540, 0.00076  # length (X), width (Y), thickness (Z)
MINI = (0.025, 0.015)  # 2FF
MICRO = (0.015, 0.012)  # 3FF
NANO = (0.0123, 0.0088)  # 4FF
CX, CY = -0.02405, 0.00302  # the cut-outs' common centre: the mini's near edge 6.25 mm from the card's short end
GAP = 0.0001  # each inner part's outline is this much inside its hole, all round (the cut's kerf, about 0.2 mm)
FREE_GAP = 0.0002  # the free nano's collider is this much smaller all round than the nano (it must not catch on the kerf as it drops)
M_FRAME, M_MINI, M_MICRO, M_NANO = 0.0046, 0.00025, 0.0001, 0.00012  # kg: PVC at 1.38 g/cm^3
M_NANO_FREE = 0.0003  # kg: the loose nano is 0.3 g in the simulation (real: 0.1 g) to keep PhysX steady with so small a body

K_TAB = 20000.0  # N/m: a part's tabs, before they give way (0.25 mm at 5 N)
D_TAB = 20.0  # N s/m
TAB_ARMATURE = 0.003  # kg: joint-space inertia on each tab slider while its tabs hold (keeps PhysX's drives on a chain of light parts steady)
BREAK_N = {"nano": 5.0, "micro": 7.0, "mini": 9.0}  # the load at which each part's tabs give way (estimates)
POP = 0.003  # m: how far a loose frame can fall out of either face
EXIT_DAMP = 1000.0  # N s/m: a torn-loose nano dragging on its cut edges until it is clear of the card's face
EXIT_CLEAR = CT + 0.0002  # m of travel: clear of the card's face
EXIT_TIME = 0.5  # s: a loose nano not pushed clear drops out by then
PARK_Z = -0.50  # m along the card's normal: where the hidden nano link is parked once the free nano has taken over (under the table top)

# -- where things are --------------------------------------------------------------------------------------------------------------
NB_X, NB_Y, NB_H = 0.30, 0.085, 0.040  # the stack of four notebooks the card lies on: its centre and height; its -y edge is at y 0
OVERHANG = 0.035  # m: the card's cut-out end out over the stack's edge (the whole mini outline is over the edge)
CARD_YAW = 90.0  # card +X along world +y: the cut-out end over the stack's -y edge
CARD_X = 0.30
CARD_Y = 0.0 - OVERHANG + CL / 2  # the card's cut-out end OVERHANG out over the box's edge
CARD_Z = TOP + NB_H + 0.0003
CARD_POS = (CARD_X, CARD_Y, CARD_Z)
PARK_FREE = (0.70, 0.50, TOP + 0.0003)  # the free nano waits here, hidden, until it takes over (out of reach and out of view)

LOOKS = {
    "pvc": m.Look(color=m.rgb(0.94, 0.94, 0.92), roughness=0.5),
    "print": m.Look(color=m.rgb(0.10, 0.36, 0.72), roughness=0.5),
    "cut": m.Look(color=m.rgb(0.55, 0.56, 0.58), roughness=0.6),
    "gold": m.Look(color=m.rgb(0.83, 0.68, 0.30), metallic=0.9, roughness=0.3),
}


def _ring(name: str, outer, hole, z0: float = 0.0, z1: float = CT, centre=(CX, CY)):
    """Four boxes: an outline `outer` (x, y) with a centred hole `hole`, at `centre`."""
    (ox, oy), (hx, hy) = outer, hole
    cx, cy = centre
    t = z1 - z0
    zc = (z0 + z1) / 2
    side = (ox - hx) / 2
    band = (oy - hy) / 2
    return [
        (f"{name}_n", (ox, band, t), (cx, cy + (hy + band) / 2, zc)),
        (f"{name}_s", (ox, band, t), (cx, cy - (hy + band) / 2, zc)),
        (f"{name}_w", (side, hy, t), (cx - (hx + side) / 2, cy, zc)),
        (f"{name}_e", (side, hy, t), (cx + (hx + side) / 2, cy, zc)),
    ]


def _frame_boxes():
    """The card round the mini's hole."""
    hx, hy = MINI
    x0, x1 = CX - hx / 2, CX + hx / 2
    y0, y1 = CY - hy / 2, CY + hy / 2
    zc = CT / 2
    return [
        ("card_w", (x0 + CL / 2, CW, CT), ((x0 - CL / 2) / 2, 0.0, zc)),
        ("card_e", (CL / 2 - x1, CW, CT), ((x1 + CL / 2) / 2, 0.0, zc)),
        ("card_n", (hx, CW / 2 - y1, CT), (CX, (y1 + CW / 2) / 2, zc)),
        ("card_s", (hx, y0 + CW / 2, CT), (CX, (y0 - CW / 2) / 2, zc)),
    ]


def _parts_cols(boxes, look="pvc"):
    parts = [(n, m.box(s, c), look) for n, s, c in boxes]
    cols = [m.col_box(f"{n}_c", s, c) for n, s, c in boxes]
    return parts, cols


def _outline(name: str, size, look="cut", w=0.0003, centre=(CX, CY)):
    """A printed-looking cut line just round an outline, on the chip side (seen, not colliding)."""
    sx, sy = size
    cx, cy = centre
    z = CT + 0.00002
    return [
        (f"{name}_ln", m.box((sx + 2 * w, w, 0.00002), (cx, cy + sy / 2 + w / 2, z)), look),
        (f"{name}_ls", m.box((sx + 2 * w, w, 0.00002), (cx, cy - sy / 2 - w / 2, z)), look),
        (f"{name}_lw", m.box((w, sy, 0.00002), (cx - sx / 2 - w / 2, cy, z)), look),
        (f"{name}_le", m.box((w, sy, 0.00002), (cx + sx / 2 + w / 2, cy, z)), look),
    ]


def _nano_parts(prefix: str, hidden: bool = False):
    """The nano's look: white PVC with its eight gold contacts on the chip side."""
    nx, ny = NANO[0] - 2 * GAP, NANO[1] - 2 * GAP
    parts = [(f"{prefix}body", m.box((nx, ny, CT), (CX, CY, CT / 2)), "pvc"),
             (f"{prefix}pad", m.box((0.0095, 0.0068, 0.00003), (CX, CY, CT + 0.000015)), "gold")]
    return [p + (True,) for p in parts] if hidden else parts


def _card():
    fb = _frame_boxes()
    fparts, fcols = _parts_cols(fb)
    fparts += [("print", m.box((0.040, CW - 0.008, 0.00002), (0.016, 0.0, CT + 0.00001)), "print")]  # the issuer's printed panel
    fparts += _outline("mini", MINI)
    mini_boxes = _ring("mini", (MINI[0] - 2 * GAP, MINI[1] - 2 * GAP), MICRO)
    mparts, mcols = _parts_cols(mini_boxes)
    mparts += _outline("micro", MICRO)
    micro_boxes = _ring("micro", (MICRO[0] - 2 * GAP, MICRO[1] - 2 * GAP), NANO)
    uparts, ucols = _parts_cols(micro_boxes)
    uparts += _outline("nano", NANO)
    nx, ny = NANO[0] - 2 * GAP, NANO[1] - 2 * GAP
    ncol = [m.col_box("nano_c", (nx, ny, CT), (CX, CY, CT / 2))]
    tab = dict(spring=K_TAB, damping=D_TAB, friction=0.0, effort=500.0, armature=TAB_ARMATURE, at=(CX, CY, 0.0))
    # The parts nest: mini in the frame, micro in mini, nano in micro, each on its own slider along the card's normal, so a press on
    # the nano loads all three sets of tabs in series. PhysX roots a free articulation at the middle of its longest chain; three
    # weightless locked stubs on the frame (no geometry) balance the chain so that the root is the frame itself.
    links = [
        m.slider("mini", "Z", -POP, POP, mparts, mcols, mass=M_MINI, gravity=True, inertia=(2e-9, 2e-9, 4e-9), **tab),
        m.slider("micro", "Z", -POP, POP, uparts, ucols, mass=M_MICRO, gravity=True, inertia=(1e-9, 1e-9, 2e-9), parent="mini", **tab),
        m.slider("nano", "Z", PARK_Z - 0.05, POP, _nano_parts("n_"), ncol, mass=M_NANO, gravity=True, inertia=(1e-9, 1e-9, 2e-9), parent="micro",
                 **tab),
    ]
    for k in range(3):
        links.append(m.slider(f"stub{k}", "Z", -0.001, 0.001, (), (), at=(0.02, 0.0, 0.0), mass=1e-5, inertia=(1e-10, 1e-10, 1e-10),
                              parent=None if k == 0 else f"stub{k - 1}", spring=50000.0, damping=10.0, effort=100.0, armature=1e-6))
    inertia = (M_FRAME * (CW ** 2 + CT ** 2) / 12, M_FRAME * (CL ** 2 + CT ** 2) / 12, M_FRAME * (CL ** 2 + CW ** 2) / 12)
    return m.mechanism("card", CARD_POS, links, LOOKS, frame_parts=fparts, frame_colliders=fcols, free_mass=M_FRAME, yaw_deg=CARD_YAW,
                       frame_inertia=inertia, friction=(0.5, 0.4), solver_iters=64)


CARD = _card()
_fx, _fy = NANO[0] - 2 * FREE_GAP, NANO[1] - 2 * FREE_GAP
NANO_FREE = m.free("nano_free", PARK_FREE, _nano_parts("f_", hidden=True), [m.col_box("free_c", (_fx, _fy, CT), (CX, CY, CT / 2))], LOOKS,
                   mass=M_NANO_FREE, friction=(0.5, 0.4), torsional_patch=None, solver_iters=32, yaw_deg=CARD_YAW)

# -- the notebooks under the card, and the distractors (SimReady) ----------------------------------------------------------------------
NOTEBOOKS = [SimReady(f"notebook{k}", pos=(NB_X + dx, NB_Y, TOP + 0.010 * k), size=(0.229, 0.170, 0.010), mass=None,
                      package=f"misc_notebook_a0{1 + k % 2}", usd=f"sm_misc_notebook_a0{1 + k % 2}_01.usd", yaw=0.0, convex=True)
             for k, dx in enumerate((0.004, -0.003, 0.002, 0.0))]
MUG = SimReady("mug", pos=(0.52, 0.22, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
               yaw=2.2)
GLASSES = SimReady("eyeglasses", pos=(0.56, -0.16, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                   usd="sm_accessory_eyeglasses_a01_01.usd", yaw=1.2)
PEN = SimReady("pen", pos=(0.17, -0.20, TOP), size=(0.178, 0.015, 0.012), mass=0.012, package="stationery_pen_plastic_a01",
               usd="sm_stationery_pen_plastic_a01_01.usd", yaw=0.5)

OBJECTS = [CARD, NANO_FREE, *NOTEBOOKS, MUG, GLASSES, PEN]

NANO_LOCAL = np.array([CX, CY, CT / 2])  # the nano's centre, in the card's frame (and in the free nano's)
CORNERS = np.array([[sx * CL / 2, sy * CW / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)])


def _array(x) -> np.ndarray:
    return m._np(x)


def _set_joint(env, link: str, value: float, stiffness: float, damping: float, armature: float | None = None) -> None:
    asset, j = m._joint(env, "card", link)
    m._TOUCHED.add(("card", link))
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    asset.write_joint_damping_to_sim(m._tensor(asset, damping), joint_ids=[j])
    if armature is not None:
        asset.write_joint_armature_to_sim(m._tensor(asset, armature), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, value), joint_ids=[j])


def _hidden_paths(prefix: str, root: str):
    return [f"{m.ENV_PATH}/{root}/visual/{prefix}{n}" for n in ("body", "pad")]


def _box_gap(local: np.ndarray, half: np.ndarray, centre: np.ndarray) -> np.ndarray:
    d = np.maximum(np.abs(local - centre) - half, 0.0)
    return np.linalg.norm(d, axis=1)


class Card:
    """The tabs, the nano's exit and the score. `torn` (which parts' tabs have given way), `freed` (the nano carried on as a free
    body) and `fell` are recorded across steps and cleared only in `reset`."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.torn = {k: False for k in BREAK_N}
        self.torn_at = None
        self.freed = False
        self.fell = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.peak = {k: 0.0 for k in BREAK_N}
        self.load = {k: 0.0 for k in BREAK_N}
        self.last = {}
        self.t = 0.0
        if env is not None:
            for k in BREAK_N:  # mechkit gives back stiffness and effort before the settle; damping and joint inertia are given back here
                _set_joint(env, k, 0.0, K_TAB, D_TAB, armature=TAB_ARMATURE)
            for p in _hidden_paths("n_", "card/nano"):
                m.set_visible(p, True)
            for p in _hidden_paths("f_", "nano_free"):
                m.set_visible(p, False)

    def _free_the_nano(self, env):
        """The free nano takes the nano link's pose and velocity; the link is hidden and parked 0.5 m below the card (under the
        table top), where nothing reaches it."""
        base = env.unwrapped
        card = base.scene["card"]
        i = card.body_names.index("nano")
        pose = card.data.body_link_pose_w[0, i].clone()
        vel = card.data.body_link_vel_w[0, i].clone()
        free = base.scene["nano_free"]
        free.write_root_pose_to_sim(pose.reshape(1, 7))
        free.write_root_velocity_to_sim(vel.reshape(1, 6))
        j = card.find_joints("nano_joint")[0][0]
        q = card.data.joint_pos.clone()
        qd = card.data.joint_vel.clone()
        q[0, j], qd[0, j] = PARK_Z, 0.0
        card.write_joint_state_to_sim(q, qd)
        _set_joint(env, "nano", PARK_Z, 50000.0, 50.0)
        for p in _hidden_paths("n_", "card/nano"):
            m.set_visible(p, False)
        for p in _hidden_paths("f_", "nano_free"):
            m.set_visible(p, True)
        self.freed = True

    def step(self, env):
        self.t += DT
        s = m.state(env, "card")
        d = {k: s.joint(k) for k in BREAK_N}
        # the tabs: the parts nest on sliders in series, so each part's tab load is K_TAB times its slider's deflection
        for k in ("nano", "micro", "mini"):
            if self.torn[k]:
                continue
            self.load[k] = K_TAB * d[k]
            self.peak[k] = max(self.peak[k], abs(self.load[k]))
            if abs(self.load[k]) > BREAK_N[k]:
                self.torn[k] = True
                if k == "nano":
                    self.torn_at = self.t
                    _set_joint(env, k, 0.0, 0.0, EXIT_DAMP)
                else:
                    _set_joint(env, k, 0.0, 0.0, 0.01, armature=1e-5)  # a loose frame falls freely out of its hole
        if self.torn["nano"] and not self.freed:
            if abs(d["nano"]) > EXIT_CLEAR or self.t - self.torn_at >= EXIT_TIME - 1e-9:
                self._free_the_nano(env)
        if s.pos[2] < TOP - 0.02:
            self.fell = True

        # the score
        tips = m.fingertips(env)
        gaps = _box_gap(s.to_local(tips), np.array([CL / 2, CW / 2, CT / 2]), np.array([0.0, 0.0, CT / 2]))
        card_gap = float(gaps.min())
        nano_out, nano_rest, nano_gap = False, False, 1.0
        if self.freed:
            f = m.state(env, "nano_free")
            centre = f.to_world(NANO_LOCAL)[0]
            nano_out = bool(centre[2] >= TOP - 0.01)
            nano_rest = bool(nano_out and centre[2] <= TOP + NB_H + 0.003 and np.linalg.norm(f.lin_vel) < 0.01)
            nano_gap = float(_box_gap(f.to_local(tips), np.array([NANO[0] / 2, NANO[1] / 2, CT / 2]), NANO_LOCAL).min())
        frames_ok = not (self.torn["micro"] or self.torn["mini"])
        still = float(np.linalg.norm(s.lin_vel)) < 0.01 and float(np.linalg.norm(s.ang_vel)) < math.radians(3.0)
        flat = s.tilt_deg() <= 10.0
        clear = card_gap >= 0.03 and nano_gap >= 0.03
        ok = nano_out and nano_rest and frames_ok and still and flat and clear and not self.fell
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.fell:
            self.score = 0.0
        elif nano_out and frames_ok:
            self.score = 1.0 if self.full else 0.75
        elif nano_out:
            self.score = 0.5
        elif not frames_ok:
            self.score = 0.25
        else:
            self.score = 0.0
        corners = s.to_world(CORNERS)
        self.last = {"t": round(self.t, 2), "torn": dict(self.torn), "freed": self.freed, "peak_N": {k: round(v, 2) for k, v in self.peak.items()},
                     "load_N": {k: round(v, 2) for k, v in self.load.items()},
                     "joints_mm": {k: round(1000 * d[k], 3) for k in BREAK_N}, "card_gap": round(card_gap, 3), "gap_lr": (round(float(gaps[:12].min()), 3), round(float(gaps[12:].min()), 3)),
                     "nano_gap": round(nano_gap, 3), "nano_out": nano_out, "nano_rest": nano_rest, "still": still, "tilt": round(s.tilt_deg(), 1),
                     "low_mm": round(1000 * float(corners[:, 2].min() - TOP), 1), "hold": self.hold, "score": self.score}


CARDWORK = Card()
m.on_step(CARDWORK.step)
m.on_reset(CARDWORK.reset)


def success(objects) -> bool:
    return bool(CARDWORK.full and CARDWORK.score == 1.0)


def score(objects) -> float:
    return float(CARDWORK.score)


# -- the demonstration (closed loop, demokit) ---------------------------------------------------------------------------------------
PIN_LOCAL = np.array([0.015, -0.016, CT])  # where the left hand holds the card down: on the notebooks, 20 mm in from their edge
PRESS_STEP = 0.0004  # m the right hand's command goes down each step while it presses
PRESS_MAX = 0.035  # m below first contact: the most it pushes
PIN_DEPTH = 0.016  # m the left hand's command is below the card's top (the arm's give turns this into a few newtons)
TILT = 45.0  # degrees: the right hand presses with one fingertip's corner, tilted along the nano's length
GAVE = 0.0006  # m: a part sunk this far into the card has given way (more than any part's tabs bend before they break; the press stops)


def _report(ctx, label=""):
    """A diagnostic line in the run's log (the script never acts on it)."""
    print(f"[sim-card] {label} {CARDWORK.last}", flush=True)


def _finger_corner(rot: np.ndarray, sx: float, opening: float) -> np.ndarray:
    """The lowest point of one open fingertip's end, from the TCP, in the world frame."""
    ends = [rot @ np.array([sx * (opening + 0.0015), dy, 0.013]) for dy in (-0.006, 0.006)]
    return min(ends, key=lambda p: p[2])


def demo_script(ctx):
    """Left: comes down on the card's end on the notebooks with its fingers closed and holds it down. Right: gripper open, tilted 45
    degrees along the nano's length, brings one fingertip's lower corner onto the middle of the nano, presses down a little more each
    step until a part of the card sinks into it (read from the parts' own positions, which the cameras show), and lifts at once. The
    nano drops onto the table. Then the left lets go and both move clear.
    Red-team modes: none (nothing), onearm (the right alone: the card is not held), careless (the same press landing 5.2 mm off the
    nano's centre, across it: on the micro ring by the nano's cut line), hold (as full, but the left never lets go)."""
    import demokit as d
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "hold"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for _ in range(120):
            yield
        _report(ctx, "end:")
        return

    def card_world(p):
        return ctx.state("card").to_world(np.asarray(p, float))[0]

    def nano_dropped():  # the nano has come out of the card (from then on it is a loose piece, seen on its own)
        return float(np.linalg.norm(ctx.pos("nano_free") - np.asarray(PARK_FREE))) > 0.05

    up = np.array([0.0, 0.0, 1.0])
    pin = card_world(PIN_LOCAL)
    tip_z = 0.0125  # the closed fingertips' underside below the TCP
    if mode != "onearm":
        yield from l.move(pin + up * 0.08, grip=d.down(0.0), finger=FINGER_CLOSED, t=2.0, label="left over the card's end")
        yield from l.move(pin + up * (tip_z + 0.004), t=1.0, label="left down to the card")
        yield from l.servo(pin + up * (tip_z + 0.004), timeout=1.0, label="left down to the card")
        yield from l.move(pin + up * (tip_z - PIN_DEPTH), t=0.8, label="left holds the card down")
        _report(ctx, "held:")

    gr = d.tilted(TILT, 90.0)
    off = _finger_corner(gr.rot, -1.0, abs(FINGER_OPEN))
    where = NANO_LOCAL + np.array([0.0, 0.0052 if mode == "careless" else 0.0, CT / 2])
    aim = card_world(where) - off
    yield from r.move(aim + up * 0.03, grip=gr, finger=FINGER_OPEN, t=2.5, label="right over the nano, tilted")
    yield from r.servo(aim + up * 0.03, timeout=1.0, label="right over the nano")
    yield from r.move(aim + up * 0.003, t=1.0, label="right just above the nano")
    yield from r.servo(aim + up * 0.003, timeout=1.0, label="right just above the nano")
    _report(ctx, "over nano:")
    base = r.at()
    k = 0
    while k * PRESS_STEP < PRESS_MAX + 0.003:
        if min(ctx.joint("card", p) for p in ("nano", "micro", "mini")) < -GAVE or nano_dropped():
            break  # something gave
        k += 1
        yield from r.move(base - up * k * PRESS_STEP, t=DT, label="press")
    _report(ctx, f"gave after {k} steps:")
    yield from r.move(r.at() + up * 0.04, t=0.4, label="lift at once")
    _report(ctx, "lifted:")
    yield from ctx.until(lambda: nano_dropped() and float(np.linalg.norm(ctx.state("nano_free").lin_vel)) < 0.01, timeout=2.0, label="nano down")
    _report(ctx, "nano down:")
    keep = mode in ("hold", "onearm")
    if not keep:
        yield from l.move(l.at() + up * 0.06, t=0.8, label="left lets go")
    yield from ctx.together(r.move((0.20, -0.22, TOP + 0.20), t=1.5, label="right clear"),
                            l.wait(1.5) if keep else l.move((0.20, 0.22, TOP + 0.20), t=1.5, label="left clear"))
    for _ in range(50):
        yield
    _report(ctx, "end:")
