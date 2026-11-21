"""Fit the wing of a Guillow's Jetfire (No. 30) balsa glider through its body, centred and square, and leave the glider on the table.

The glider is the 12-inch (305 mm span) Jetfire at full size. Its tailplane and plastic nose clip are already on; the one-piece wing lies flat on
the table beside it. The wing has to go through a slot in the 3.2 mm balsa body that is 0.8 mm taller than the wing is thick, and be pushed on
until it stands out equally on both sides and square to the body. A wing pushed through tilted wedges the slot open and splits the thin strip
of balsa under it.

Model (declared in code, all collision geometry is real contact, nothing is triggered by proximity):
- Glider ("glider", free articulation, 5 g): the body as a flat balsa profile 280 mm long and 3.2 mm thick (convex pieces round the wing slot),
  the tailplane (120 x 40 x 1.6 mm, a press fit, part of the body) and the nose clip. The wing slot is 95 x 2.8 mm (longer than the wing's chord, as on Guillow's gliders, for trim), its bottom 7 mm above the
  body's lower edge. The 7 mm strip of balsa under the slot (link "lip") slides down against an 8 N/mm spring: the slot's give, lumped. Forcing
  it more than 0.5 mm open (about 4 N of wedging) is the balsa splitting at the slot's ends: a crack, recorded and kept until a reset.
- Wing ("wing", free, 4 g): a flat balsa plate 2.0 mm thick, 305 mm span, 75 mm root chord tapering to 45 mm at the tips, straight trailing edge
  and swept leading edge. Its built-in dihedral is not modelled.
- Not modelled: force on the wing itself (only the slot can crack), the balsa's flex, the printed graphics. Uniform rubber-like friction on pads.

Scoring (end state, read by the tracker every control step; `score()` returns it). Offset = how far the wing's centre is from the body's mid-plane,
along the span (half the difference of the two tip overhangs measured from the body's sides). Out of square = the angle between the wing's span
and the perpendicular to the body, seen from above. Through = the wing passes through the wing slot.
- 0.0 if the slot has ever cracked; cleared only at a reset.
- 1.0: through, offset at most 3 mm, out of square at most 3 degrees, the glider standing on the table (its lower edge within 3 mm of the
  table, tilted at most 10 degrees), everything still, every fingertip point at least 3 cm from glider and wing, held for 1 s.
- 0.5: through, offset at most 10 mm and out of square at most 8 degrees, without the rest of 1.0.
- 0.25: through, further off-centre or out of square than that.
- 0.0 otherwise (the wing not in the slot).
The partial levels need no hands clear (they are progress), full credit does.

    python demokit.py task.py OUT --mode full
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
INSTRUCTION = "Fit the wing through the glider's body, centred and square, and leave the glider on the table."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the body (glider frame: origin on the body's lower edge under the wing slot's centre, x to the nose, z up, y to the left wing) -----------
T_F = 0.0032  # 1/8 in balsa sheet
SLOT_X = 0.0475  # half the wing slot's length: 95 mm, longer than the 75 mm root chord, so the wing can be set fore and aft (trim)
Z_S0, H_S = 0.0070, 0.0028  # the wing slot's bottom above the body's lower edge, and its height
Z_S1 = Z_S0 + H_S
Z_MID = Z_S0 + H_S / 2
X_NOSE, X_TAIL = 0.1225, -0.1575  # 280 mm overall
LIP_K = 8000.0  # N/m: the strip under the slot, lumped
CRACK = 0.0005  # m of give: about 4 N of wedging
# -- the wing (wing frame: origin at the root chord's middle, mid-thickness; x forward, y along the span to the left tip, z up) -----------------
T_W = 0.0020
HALF_SPAN = 0.1525
TE_X = -0.0375  # the straight trailing edge
LE_ROOT, LE_TIP = 0.0375, 0.0075  # 75 mm root chord, 45 mm tip chord


def le_x(y: float) -> float:
    return LE_ROOT + (LE_TIP - LE_ROOT) * min(abs(y), HALF_SPAN) / HALF_SPAN


LOOKS = {
    "balsa": m.Look(color=m.rgb(0.86, 0.76, 0.58), roughness=0.85),
    "balsa_edge": m.Look(color=m.rgb(0.78, 0.66, 0.46), roughness=0.9),
    "slot": m.Look(color=m.rgb(0.12, 0.10, 0.08), roughness=0.9),
    "print_red": m.Look(color=m.rgb(0.78, 0.10, 0.08), roughness=0.6),
    "print_blue": m.Look(color=m.rgb(0.10, 0.22, 0.62), roughness=0.6),
    "clip": m.Look(color=m.rgb(0.80, 0.12, 0.10), roughness=0.35),
}


def sheet(outline_xz, t: float, y0: float = 0.0) -> m.Mesh:
    """A flat piece in the body's plane: a convex (x, z) outline, t thick, centred on y = y0."""
    return m.prism([(x, z) for x, z in outline_xz], -t / 2, t / 2).turned(90, "x").moved((0.0, y0, 0.0))


# body pieces (x, z), each convex, counter-clockwise seen from +y... prism wants counter-clockwise in its own (x, y) = our (x, z)
FWD = [(SLOT_X, 0.0), (0.100, 0.0), (X_NOSE, 0.004), (X_NOSE, 0.013), (0.100, 0.020), (SLOT_X, 0.026)]
UPPER = [(-SLOT_X, Z_S1), (SLOT_X, Z_S1), (SLOT_X, 0.026), (0.012, 0.034), (-0.020, 0.033), (-SLOT_X, 0.026)]
AFT = [(X_TAIL, 0.0), (-SLOT_X, 0.0), (-SLOT_X, 0.026), (X_TAIL, 0.014)]
FIN = [(X_TAIL, 0.012), (-0.104, 0.012), (-0.128, 0.058), (X_TAIL + 0.004, 0.064)]
LIP = [(-SLOT_X, 0.0), (SLOT_X, 0.0), (SLOT_X, Z_S0), (-SLOT_X, Z_S0)]
TAIL_X0, TAIL_X1, TAIL_HALF, TAIL_T = -0.1525, -0.1125, 0.060, 0.0016
CLIP_X0 = 0.098  # the nose clip covers the nose from here forward


def _glider():
    parts = [
        ("fwd", sheet(FWD, T_F), "balsa"),
        ("upper", sheet(UPPER, T_F), "balsa"),
        ("aft", sheet(AFT, T_F), "balsa"),
        ("fin", sheet(FIN, T_F), "balsa"),
        ("canopy_l", sheet([(-0.012, 0.022), (0.026, 0.022), (0.014, 0.031), (-0.006, 0.030)], 0.0004, T_F / 2 + 0.0002), "print_blue"),
        ("canopy_r", sheet([(-0.012, 0.022), (0.026, 0.022), (0.014, 0.031), (-0.006, 0.030)], 0.0004, -T_F / 2 - 0.0002), "print_blue"),
        ("stripe_l", sheet([(-0.100, 0.006), (-0.045, 0.012), (-0.045, 0.016), (-0.100, 0.010)], 0.0004, T_F / 2 + 0.0002), "print_red"),
        ("stripe_r", sheet([(-0.100, 0.006), (-0.045, 0.012), (-0.045, 0.016), (-0.100, 0.010)], 0.0004, -T_F / 2 - 0.0002), "print_red"),
        ("fin_l", sheet([(-0.150, 0.030), (-0.128, 0.030), (-0.136, 0.050), (-0.150, 0.054)], 0.0004, T_F / 2 + 0.0002), "print_red"),
        ("fin_r", sheet([(-0.150, 0.030), (-0.128, 0.030), (-0.136, 0.050), (-0.150, 0.054)], 0.0004, -T_F / 2 - 0.0002), "print_red"),
        ("tailplane", m.box((TAIL_X1 - TAIL_X0, 2 * TAIL_HALF, TAIL_T), ((TAIL_X0 + TAIL_X1) / 2, 0.0, TAIL_T / 2)), "balsa"),
        ("tail_tip_l", m.box((0.030, 0.012, 0.0003), (TAIL_X0 + 0.018, TAIL_HALF - 0.008, TAIL_T + 0.00015)), "print_red"),
        ("tail_tip_r", m.box((0.030, 0.012, 0.0003), (TAIL_X0 + 0.018, -TAIL_HALF + 0.008, TAIL_T + 0.00015)), "print_red"),
        ("clip", m.join(m.box((X_NOSE + 0.003 - CLIP_X0, 0.009, 0.006), ((CLIP_X0 + X_NOSE + 0.003) / 2, 0.0, 0.003)),
                        m.box((X_NOSE + 0.003 - CLIP_X0, 0.009, 0.006), ((CLIP_X0 + X_NOSE + 0.003) / 2, 0.0, 0.012))), "clip"),
        ("slot_l", sheet([(-SLOT_X, Z_S0), (SLOT_X, Z_S0), (SLOT_X, Z_S1), (-SLOT_X, Z_S1)], 0.0002, T_F / 2 + 0.0001), "slot"),
        ("slot_r", sheet([(-SLOT_X, Z_S0), (SLOT_X, Z_S0), (SLOT_X, Z_S1), (-SLOT_X, Z_S1)], 0.0002, -T_F / 2 - 0.0001), "slot"),
    ]
    cols = [
        m.col_hull("fwd_c", sheet(FWD, T_F)),
        m.col_hull("upper_c", sheet(UPPER, T_F)),
        m.col_hull("aft_c", sheet(AFT, T_F)),
        m.col_hull("fin_c", sheet(FIN, T_F)),
        m.col_box("tail_c", (TAIL_X1 - TAIL_X0, 2 * TAIL_HALF, TAIL_T), ((TAIL_X0 + TAIL_X1) / 2, 0.0, TAIL_T / 2)),
        m.col_box("clip_c", (X_NOSE + 0.003 - CLIP_X0, 0.009, 0.015), ((CLIP_X0 + X_NOSE + 0.003) / 2, 0.0, 0.0075)),
    ]
    return parts, cols


GLIDER_XY = (0.30, 0.10)
_gp, _gc = _glider()
LIP_LINK = m.slider("lip", "Z", -0.003, 0.0, (("lip", sheet(LIP, T_F), "balsa"),), (m.col_hull("lip_c", sheet(LIP, T_F)),), mass=0.0005, spring=LIP_K,
                    damping=2.0, friction=0.0, effort=200.0)
GLIDER = m.mechanism("glider", (GLIDER_XY[0], GLIDER_XY[1], TOP), [LIP_LINK], LOOKS, frame_parts=_gp, frame_colliders=_gc, friction=(0.6, 0.5), solver_iters=64,
                     free_mass=0.005, frame_inertia=(1.5e-6, 3.0e-5, 3.0e-5))


# -- the wing --------------------------------------------------------------------------------------------------------------------------------
WING_OUTLINE = [(TE_X, -HALF_SPAN), (LE_TIP, -HALF_SPAN), (LE_ROOT, 0.0), (LE_TIP, HALF_SPAN), (TE_X, HALF_SPAN)]


def _wing():
    plate = m.prism(WING_OUTLINE, -T_W / 2, T_W / 2)
    parts = [("plate", plate, "balsa"),
             ("star_l", m.prism([(-0.030, 0.085), (-0.005, 0.085), (-0.005, 0.115), (-0.030, 0.115)], T_W / 2, T_W / 2 + 0.0003), "print_blue"),
             ("star_r", m.prism([(-0.030, -0.115), (-0.005, -0.115), (-0.005, -0.085), (-0.030, -0.085)], T_W / 2, T_W / 2 + 0.0003), "print_blue"),
             ("band", m.prism([(TE_X, -0.030), (-0.020, -0.030), (-0.020, 0.030), (TE_X, 0.030)], T_W / 2, T_W / 2 + 0.0003), "print_red")]
    # colliders: seven box strips along the span, each as long as the chord at its middle (a convex hull 2 mm thin falls through the
    # table on GPU physics; boxes do not). The leading edge steps by at most 4 mm; it never bears on anything.
    cols = []
    edges = np.linspace(-HALF_SPAN, HALF_SPAN, 8)
    for i in range(7):
        y0, y1 = edges[i], edges[i + 1]
        le = le_x(0.5 * (y0 + y1)) if y0 * y1 > 0 else LE_ROOT - (LE_ROOT - LE_TIP) * max(abs(y0), abs(y1)) / HALF_SPAN / 2
        cols.append(m.col_box(f"wing_{i}", (le - TE_X, y1 - y0, T_W), ((le + TE_X) / 2, 0.5 * (y0 + y1), 0.0)))
    return parts, cols


WING_START = (0.30, -0.135)
WING_YAW = 6.0  # as it was laid down, not quite square to the body
_wp, _wc = _wing()
WING = m.free("wing", (WING_START[0], WING_START[1], TOP + T_W / 2 + 0.0002), _wp, _wc, LOOKS, mass=0.004, friction=(0.6, 0.5), torsional_patch=0.01,
              solver_iters=64, yaw_deg=WING_YAW)

# -- distractors (SimReady): a mug, a notebook, a marker ------------------------------------------------------------------------------------
MUG = SimReady("mug", pos=(0.50, 0.24, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.4)
NOTEBOOK = SimReady("notebook", pos=(0.53, -0.20, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd",
                    yaw=0.25)
MARKER = SimReady("marker", pos=(0.47, 0.02, TOP), size=(0.109, 0.008, 0.007), mass=0.01, yaw=1.2, package="stationery_marker_thin_blue_a01",
                  usd="sm_stationery_marker_thin_blue_a01_01.usd")

OBJECTS = [GLIDER, WING, MUG, NOTEBOOK, MARKER]


# -- the scorer ------------------------------------------------------------------------------------------------------------------------------
def _box_gap(q: np.ndarray, lo, hi) -> np.ndarray:
    d = np.maximum(np.maximum(np.asarray(lo) - q, q - np.asarray(hi)), 0.0)
    return np.linalg.norm(d, axis=1)


def measure(glider, wing, tips=None) -> dict:
    """Everything the score reads, in the glider's frame."""
    rg, rw = glider.rotation(), wing.rotation()
    w0 = glider.to_local(wing.pos)[0]
    span = rg.T @ rw[:, 1]
    normal = rg.T @ rw[:, 2]
    out = {
        "glider_tilt": glider.tilt_deg(),
        "glider_floor": float(glider.pos[2] - TOP),
        "lip": float(-glider.joints.get("lip_joint", 0.0)),
        "square": float(np.degrees(math.atan2(-span[0], span[1]))),  # the span's turn from the body's perpendicular, seen from above
        "wing_pitch": float(np.degrees(math.atan2(normal[0], normal[2]))),
        "wing_roll": float(np.degrees(math.atan2(-normal[1], normal[2]))),
        "w0": w0,
        "v": (float(np.linalg.norm(glider.lin_vel)), float(np.linalg.norm(wing.lin_vel))),
    }
    through, offset, cross = False, float("nan"), np.full(3, np.nan)
    if abs(span[1]) > 0.7:
        s = -w0[1] / span[1]  # along the span, from the wing's centre to the body's mid-plane
        cross = w0 + s * span
        offset = float(-s)  # + : the wing's centre lies to the left (+y) of the body
        through = bool(abs(s) <= HALF_SPAN - 0.005 and abs(cross[0]) <= SLOT_X + 0.003 and Z_S0 - 0.002 <= cross[2] <= Z_S1 + 0.002)
    out.update(through=through, offset=offset, cross=cross)
    if tips is not None:
        qg = glider.to_local(tips)
        qw = wing.to_local(tips)
        gaps = np.minimum.reduce([
            _box_gap(qg, (X_TAIL, -0.006, -0.001), (X_NOSE + 0.004, 0.006, 0.066)),
            _box_gap(qg, (TAIL_X0, -TAIL_HALF, -0.001), (TAIL_X1, TAIL_HALF, TAIL_T + 0.001)),
            _box_gap(qw, (TE_X, -HALF_SPAN, -T_W / 2), (LE_ROOT, HALF_SPAN, T_W / 2)),
        ])
        out["hand_gap"] = float(np.min(gaps))
    return out


def level(c: dict) -> float:
    """The progress level of a state (no hands rule, no harm)."""
    if not c["through"]:
        return 0.0
    off, sq = abs(c["offset"]), abs(c["square"])
    if off <= 0.003 and sq <= 3.0 and c["glider_floor"] <= 0.003 and c["glider_tilt"] <= 10.0:
        return 1.0
    if off <= 0.010 and sq <= 8.0:
        return 0.5
    return 0.25


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place a recorded harm is cleared
        self.cracked = False
        self.max_lip = 0.0
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}
        self.prev = None

    def step(self, env):
        glider, wing = m.state(env, "glider"), m.state(env, "wing")
        c = measure(glider, wing, m.fingertips(env))
        now = np.r_[glider.pos, wing.pos, c["square"] * 0.001]
        prev = self.prev if self.prev is not None else now
        c["still"] = bool(np.max(np.abs(now - prev)) < 0.0005)  # no origin moved 0.5 mm, nor the wing turned 0.5 degree, in a step
        self.prev = now
        self.last = c
        self.max_lip = max(self.max_lip, c["lip"])
        if c["lip"] > CRACK:
            self.cracked = True
        lv = level(c)
        ok = (not self.cracked) and lv == 1.0 and c["still"] and c["hand_gap"] >= 0.03
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= HOLD_STEPS:
            self.full = True
        if self.full and (self.cracked or lv < 1.0):  # credit does not outlast the state that earned it
            self.full = False
        if self.cracked:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = min(lv, 0.5)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration -----------------------------------------------------------------------------------------------------------------------
DISTRACTORS = ("mug", "notebook", "marker")


def _report(ctx, label=""):
    if label.startswith(("end", "right clear", "left clear")):
        print("[distractors] " + ", ".join(f"{n} {np.round(ctx.pos(n)[:2] - np.array(o.pos[:2]), 4).tolist()}" for n, o in
                                          zip(DISTRACTORS, (MUG, NOTEBOOK, MARKER))), flush=True)
    c = TRIAL.last
    if not c:
        return
    print(f"[glider] t {ctx.t:5.1f} {label} through {c['through']} offset {c['offset'] * 1000:+7.1f} mm square {c['square']:+5.1f} pitch {c['wing_pitch']:+5.2f} "
          f"roll {c['wing_roll']:+5.1f} w0 {np.round(c['w0'] * 1000, 1).tolist()} | lip {c['lip'] * 1000:5.2f} max {TRIAL.max_lip * 1000:5.2f} mm | glider floor "
          f"{c['glider_floor'] * 1000:+6.1f} tilt {c['glider_tilt']:4.1f} | hands {c.get('hand_gap', 0) * 1000:5.0f} mm still {c['still']} | "
          f"score {TRIAL.score:.2f} full {TRIAL.full} cracked {TRIAL.cracked}", flush=True)


def demo_script(ctx):
    """Left: takes the body from above just behind the wing slot (fingers either side of the 3.2 mm balsa) and lifts the glider 7 cm. Right:
    pinches the wing across its chord near its right tip, lifts it, squares it to the body and brings it level with the slot, feeds the left tip
    in and pushes the wing through along its own plane, re-levelling it to the slot as it goes, until the two overhangs are equal; lets go and
    moves clear. The left then sets the glider down on the table, lets go and moves clear.
    Red-team modes: none (nothing), onearm (the right alone, the glider not held), careless (the wing fed and pushed 2.5 degrees nose-down to the
    slot's plane),
    held (centred but the glider still held at the end), offcentre (stopped 15 mm short of centre)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "held", "offcentre"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for _ in range(100):
            yield
        _report(ctx, "end:")
        return
    up = np.array([0.0, 0.0, 1.0])

    def gstate():
        return ctx.state("glider")

    def wstate():
        return ctx.state("wing")

    def g_world(p_local):
        return gstate().to_world(np.asarray(p_local, dtype=float))[0]

    def w_world(p_local):
        return wstate().to_world(np.asarray(p_local, dtype=float))[0]

    def g_yaw():
        rg = gstate().rotation()
        return math.degrees(math.atan2(rg[1, 0], rg[0, 0]))

    def w_yaw():
        rw = wstate().rotation()
        return math.degrees(math.atan2(rw[1, 0], rw[0, 0]))

    hold_glider = mode != "onearm"
    LIFT = 0.0  # the left steadies the glider where it stands; the wing goes in 7 to 10 mm above the table
    GRIP_X = -0.130  # the left's fingers on the rear of the body, under the fin: well clear of the right hand when the wing is centred
    GRIP_Z = 0.024
    if hold_glider:
        p = g_world((GRIP_X, 0.0, GRIP_Z))
        grip_l = d.down(g_yaw())
        yield from l.move(p + up * 0.08, grip=grip_l, finger=FINGER_OPEN, t=2.5, label="left over the body")
        yield from l.servo(p + up * 0.08, timeout=1.0, label="left over the body")
        yield from l.move(p, t=1.2, label="left round the body")
        yield from l.servo(p, timeout=1.0, tol=0.0015, label="left round the body")
        yield from l.close(T_F, 25.0, t=0.6, label="hold the body")
        _report(ctx, "glider held:")

    # pick the wing across its chord 35 mm right of its root (fingers on its leading and trailing edges): close to its centre of mass, so it
    # does not roll between the pads, and far enough out that the fingers stay 25 mm clear of the body when the wing is centred
    Y_HOLD = -0.035
    X_HOLD = (TE_X + le_x(Y_HOLD)) / 2
    chord = le_x(Y_HOLD) - TE_X
    p = w_world((X_HOLD, Y_HOLD, 0.0))
    grip_r = d.down(90.0 + w_yaw())
    Z_PAD = 0.0122  # TCP above the wing's mid-plane: the finger tips (12.8 mm past the TCP, 15 mm wide) 0.3 mm above the table
    yield from r.move(p + up * 0.06, grip=grip_r, finger=FINGER_OPEN, t=2.5, label="over the wing tip")
    yield from r.servo(p + up * 0.06, timeout=1.0, label="over the wing tip")
    yield from r.move(p + up * Z_PAD, t=1.2, label="round the wing tip")
    yield from r.servo(p + up * Z_PAD, timeout=1.0, tol=0.0015, label="round the wing tip")
    yield from r.close(chord, 6.0, t=0.6, label="pinch the wing")
    yield from r.move(r.at() + up * 0.03, t=0.8, label="lift the wing")
    _report(ctx, "wing lifted:")
    yield from r.move(r.at() + up * 0.02, t=1.0, label="wing up")
    _report(ctx, "wing up:")
    if not hold_glider:
        yield from r.wait(0.5)

    # square the wing to the body: turn the hand by the wing's yaw error
    def yaw_err():
        return ((w_yaw() - g_yaw()) + 180.0) % 360.0 - 180.0

    yield from r.turn(-yaw_err(), t=1.0, label="square the wing")
    _report(ctx, "squared:")

    TIP = np.array([-0.015, HALF_SPAN, 0.0])  # the leading point of the left tip, on the wing's mid-plane
    HOLD = np.array([X_HOLD, Y_HOLD, 0.0])  # where the fingers hold the wing
    ENTRY = np.array([-0.015, -T_F / 2 - 0.006, Z_MID])  # 6 mm short of the body's right face, level with the slot's middle

    def attitude():
        """The wing's pitch (about the body's y) and roll (about its x) relative to the slot's plane, degrees. Roll > 0: left tip up."""
        rg, rw = gstate().rotation(), wstate().rotation()
        n = rg.T @ rw[:, 2]
        return math.degrees(math.atan2(n[0], n[2])), math.degrees(math.atan2(-n[1], n[2]))

    def level_to_slot(label, tol_pitch=0.12, tol_roll=0.1):
        """Turn the hand about the body's y and x so that the wing lies in the slot's plane."""
        rg = gstate().rotation()
        pitch, roll = attitude()
        if abs(pitch) > tol_pitch:
            yield from r.turn(-pitch, axis=rg[:, 1], t=max(0.3, abs(pitch) / 10.0), label=f"{label}: pitch")
        if abs(roll) > tol_roll:
            yield from r.turn(-roll, axis=rg[:, 0], t=max(0.3, abs(roll) / 10.0), label=f"{label}: roll")

    def lost():
        return float(np.linalg.norm(w_world(HOLD) - r.tcp())) > 0.02

    def tip_local():
        return gstate().to_local(w_world(TIP))[0]

    def entered():
        return bool(tip_local()[1] > -T_F / 2 + 0.0008)  # 0.8 mm into the 3.2 mm body: only possible in the slot

    yield from level_to_slot("level")
    yield from r.move(g_world(ENTRY) - (w_world(TIP) - r.tcp()) + np.array([0.0, -0.01, 0.0]), t=1.5, label="tip to the slot")
    # Find the slot by feel, as a person does: line the tip up with the slot 6 mm short of the body's side from the poses, press it lightly
    # against the side (1 mm past it) and sweep it slowly up and down across the slot's height until it drops in. If it has not after two
    # sweeps, back off, level the wing and line it up again.
    ok = False
    for attempt in range(4):
        for k in range(3):
            yield from level_to_slot("fine level")
            yield from r.wait(0.3)
            yield from r.move(r.at() + g_world(ENTRY) - w_world(TIP), t=0.5, label="tip to the slot")
        yield from r.wait(0.3)
        if attempt == 0:
            _report(ctx, "at the slot:")
        rg = gstate().rotation()
        hand0 = r.at().copy()
        tip0 = tip_local()
        for k in range(48):
            if entered():
                ok = True
                break
            sweep = 0.0012 * math.sin(2.0 * math.pi * k / 24.0)
            press = min(1.0, k / 8.0) * (ENTRY[1] - tip0[1] + 0.006 + 0.001)  # from 6 mm short of the side to 1 mm past it
            yield from r.move(hand0 + rg[:, 1] * press + rg[:, 2] * (Z_MID - tip0[2] + sweep), t=0.1, label="search")
        if ok or entered():
            ok = True
            break
        print(f"[entry] attempt {attempt}: not in after two sweeps", flush=True)
        yield from r.move(hand0, t=0.4, label="back off")
    print(f"[entry] in {ok} after {attempt + 1} attempts", flush=True)
    _report(ctx, "tip in the slot:" if ok else "tip not in:")

    # push through along the body's y in 3 mm steps. Until the tip is in, steer its height; after, keep the wing level with the slot by
    # raising or lowering the hand (the wing pivots about the slot), and re-level its pitch now and then.
    target = 0.0 if mode != "offcentre" else -0.015
    last_y, stall = None, 0
    for k in range(160):
        if lost():
            print("[push] the wing is no longer in the hand", flush=True)
            break
        g = gstate()
        rg = g.rotation()
        w0 = g.to_local(wstate().pos)[0]
        togo = target - w0[1]
        if abs(togo) < 0.0006:
            break
        stall = stall + 1 if (last_y is not None and abs(w0[1] - last_y) < 0.0003) else 0
        last_y = w0[1]
        if stall >= 6:
            print("[push] stalled", flush=True)
            break
        step = float(np.clip(togo, -0.003, 0.003))
        tip_l = g.to_local(w_world(TIP))[0]
        pitch, roll = attitude()
        if mode == "careless":  # the careless push: the hand rises 0.6 mm with every 3 mm pushed, so the wing is levered in the slot
            dz = 0.0006
        else:  # level the wing about the slot with the hand's height (the wing pivots about the slot)
            arm = abs(g.to_local(r.tcp())[0][1])
            dz = float(np.clip(0.5 * arm * math.tan(math.radians(roll)), -0.0005, 0.0005))
        yield from r.move(r.at() + rg[:, 1] * step + rg[:, 2] * dz, t=0.2, label="push")
        if k % 10 == 9:
            _report(ctx, f"push {k + 1}:")
            if mode != "careless" and abs(pitch) > 0.25:
                yield from level_to_slot("relevel", tol_pitch=0.25, tol_roll=99.0)
    _report(ctx, "centred:")
    # square with the wing in the slot, and fine-centre
    for _ in range(3):
        if lost():
            break
        e = yaw_err()
        if abs(e) > 0.6:
            yield from r.turn(-e, t=0.8, label="square in the slot")
        yield from r.wait(0.2)
        g = gstate()
        togo = target - g.to_local(wstate().pos)[0][1]
        if abs(togo) > 0.0006:
            yield from r.move(r.at() + g.rotation()[:, 1] * togo, t=0.5, label="centre")
    _report(ctx, "squared in the slot:")
    yield from r.open(t=0.8, label="let go of the wing")
    yield from r.move(r.at() + up * 0.05, t=1.2, label="right up")
    yield from r.move(np.array([0.22, -0.30, TOP + 0.20]), grip=d.down(90.0), t=1.5, label="right away")
    _report(ctx, "right clear:")
    if hold_glider and mode != "held":
        yield from l.wait(0.4)
        yield from l.open(t=0.8, label="let go of the body")
        yield from l.move(l.at() + up * 0.08, t=1.0, label="left up and clear")
        yield from l.move(np.array([0.20, 0.30, TOP + 0.20]), t=1.2, label="left away")
        _report(ctx, "left clear:")
    for k in range(60):
        yield
        if k % 10 == 9:
            _report(ctx, "end:")
