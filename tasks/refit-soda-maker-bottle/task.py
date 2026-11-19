"""Refit a filled 1 L carbonating bottle in a sparkling water maker's Snap-Lock holder.

The machine is modelled on a SodaStream Terra (about 13 x 20 x 44 cm; 3.0 kg here with a CO2 cylinder fitted, an estimate): a base with a drip area, a tower, and a head that overhangs the
bottle bay. Under the head hangs the Snap-Lock holder, a cup on a hinge, which rests tilted forward 15 degrees. The manual's steps: with the
holder angled forward, put the bottle's neck into the Snap-Lock, push the bottle up and back; the Snap-Lock holds it, with a small gap between
the bottle's bottom and the base.

Model (declared in code):
- Machine ("maker"): a free-standing articulation (3.0 kg frame: base, tower and head, the CO2 cylinder's mass included) with three links.
  - holder: the Snap-Lock cup, on a hinge about y behind the neck. Joint zero is its rest, tilted 15 degrees forward (the bottle's bottom
    towards the robot); -15 degrees is upright, where its stop is. A spring (1.5 N m/rad, preloaded by 0.6 N m) holds it forward.
  - jaw_l, jaw_r: two sprung jaws (sliders on the holder, 800 N/m each, 4 mm of travel), ramped below and flat on top. The bottle's neck
    collar cams them apart on the way up; they close under the collar, which then rests on them.
  - The detent latch: when the holder reaches upright (within 0.6 degrees) with a bottle's collar in the jaws, the holder's joint is locked
    there (mechkit lock), as the Snap-Lock's catch does. A latched holder is not released in this task.
- Bottle ("bottle"): a free rigid body, 1.05 kg (the bottle plus 1 L of water as rigid mass), PET bottle 8.5 cm across and 28 cm tall with a
  2.8 cm neck and a 3.4 cm collar, colliding as convex hulls of its body, shoulder, neck and collar. The holder's bore round the neck is
  3.16 cm across with a 4 mm chamfer at its mouth.

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
INSTRUCTION = "Put the bottle back on the sparkling water maker so that it locks in place."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 60  # 3 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the machine's frame: origin on the counter, on the axis of a fitted bottle; x away from the robot, y to its left
MAKER_POS = np.array([0.25, -0.03, TOP])
BASE_X0, BASE_X1, HALF_W, BASE_H = -0.05, 0.15, 0.065, 0.025  # base and drip area: 20 cm deep, 13 cm wide, 2.5 cm tall
TOWER_X0 = 0.05  # the tower's front face, behind the bottle bay
HEAD_X0, HEAD_Z0, HEIGHT = -0.045, 0.335, 0.44  # the head overhangs the bay; the machine is 44 cm tall

# -- the bottle (its frame: origin at the centre of its bottom, z up its axis)
R_BODY, H_BOTTLE = 0.0425, 0.28
R_FOOT = 0.036  # the bottom's rounded edge: radius where it meets the base
SHOULDER_Z0, SHOULDER_Z1 = 0.20, 0.245
R_NECK, R_COLLAR = 0.014, 0.017
COLLAR_Z0, COLLAR_Z1 = 0.257, 0.261
BOTTLE_MASS = 1.05
BOTTLE_COM = (0.0, 0.0, 0.095)  # water to the 1 L fill line (17.6 cm)
BOTTLE_START = (0.25, -0.17)

# -- the Snap-Lock, as fitted (upright), in the machine's frame
GAP0 = 0.010  # the bottle's bottom above the base when it hangs in the jaws
Z_JAW = BASE_H + GAP0 + COLLAR_Z0  # the jaws' flat tops, under the collar
PIVOT = np.array([0.035, 0.0, 0.30])  # the holder's hinge, behind the neck
TILT = 15.0  # degrees forward at rest
K_HOLDER = 1.5  # N m/rad
PRELOAD_DEG = 23.0  # the spring's rest past the forward stop
K_JAW, JAW_TRAVEL = 800.0, 0.004
JAW_IN, JAW_RAMP_IN, JAW_OUT, JAW_DEPTH, JAW_HALF_X = 0.0145, 0.0215, 0.030, 0.010, 0.008
R_BORE, R_CHAMBER, R_CUP = 0.0158, 0.019, 0.027
Z_CHAMBER_TOP = Z_JAW + 0.008  # the collar can rise 4 mm above its seat before it meets the bore's step
Z_CHAMFER = Z_CHAMBER_TOP + 0.004  # the bore's chamfer: 4 mm tall
Z_CEIL = BASE_H + GAP0 + H_BOTTLE + 0.003
Z_CUP0 = Z_JAW - JAW_DEPTH - 0.002
R_LEAD, H_LEAD, R_SKIRT = 0.034, 0.010, 0.037  # the mouth's lead-in: a cone from the chamber's radius out to R_LEAD, H_LEAD below the cup

LOOKS = {
    "abs_black": m.Look(color=m.rgb(0.06, 0.06, 0.065), roughness=0.45),
    "abs_grey": m.Look(color=m.rgb(0.32, 0.33, 0.35), roughness=0.5),
    "abs_dark": m.Look(color=m.rgb(0.14, 0.14, 0.15), roughness=0.5),
    "pom": m.Look(color=m.rgb(0.85, 0.85, 0.82), roughness=0.4),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.9, roughness=0.35),
    "tray": m.Look(color=m.rgb(0.20, 0.20, 0.22), roughness=0.3),
    "button": m.Look(color=m.rgb(0.55, 0.56, 0.58), metallic=0.6, roughness=0.3),
    "pet": m.Look(color=m.rgb(0.80, 0.88, 0.95), roughness=0.08, opacity=0.35),
    "water": m.Look(color=m.rgb(0.55, 0.75, 0.92), roughness=0.05, opacity=0.35),
    "neck": m.Look(color=m.rgb(0.75, 0.82, 0.90), roughness=0.15, opacity=0.6),
    "label": m.Look(color=m.rgb(0.10, 0.35, 0.65), roughness=0.5),
}


def _ry(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def tilted(mesh: m.Mesh, deg: float = TILT) -> m.Mesh:
    """A mesh of the upright holder, turned forward `deg` about the hinge."""
    return mesh.moved(-PIVOT).turned(deg, "y").moved(PIVOT)


def _hull(points) -> m.Mesh:
    from scipy.spatial import ConvexHull

    pts = np.asarray(points, float)
    return m.Mesh(pts, ConvexHull(pts).simplices)


def _cone_sector(r_top, r_bot, r_out, a0, a1, z_bot, z_top, n=4) -> m.Mesh:
    """A sector of a conical lead-in (inner radius r_bot at z_bot to r_top at z_top, outer r_out), as a solid for a hull."""
    a = np.radians(np.linspace(a0, a1, n))
    pts = [(r * math.cos(t), r * math.sin(t), z) for t in a for r, z in ((r_top, z_top), (r_bot, z_bot), (r_out, z_top), (r_out, z_bot))]
    return _hull(pts)


def _sector(r0, r1, a0, a1, z0, z1, n=4) -> m.Mesh:
    """An annular sector (angles in degrees from +x, about z), as a solid for a hull."""
    a = np.radians(np.linspace(a0, a1, n))
    ring = [(r1 * math.cos(t), r1 * math.sin(t)) for t in a] + [(r0 * math.cos(t), r0 * math.sin(t)) for t in a[::-1]]
    pts = np.array(ring)
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    return m.prism(pts if area > 0 else pts[::-1], z0, z1)


# -- the machine's frame ---------------------------------------------------------------------------------------------------------------------


def _frame():
    parts, cols = [], []
    base = ((BASE_X1 - BASE_X0, 2 * HALF_W, BASE_H), ((BASE_X0 + BASE_X1) / 2, 0.0, BASE_H / 2))
    tower = ((BASE_X1 - TOWER_X0, 2 * HALF_W, HEAD_Z0 - BASE_H), ((TOWER_X0 + BASE_X1) / 2, 0.0, (BASE_H + HEAD_Z0) / 2))
    head = ((BASE_X1 - HEAD_X0, 2 * HALF_W, HEIGHT - HEAD_Z0), ((HEAD_X0 + BASE_X1) / 2, 0.0, (HEAD_Z0 + HEIGHT) / 2))
    for name, (size, c), look in (("base", base, "abs_black"), ("tower", tower, "abs_black"), ("head", head, "abs_black")):
        parts.append((name, m.box(size, c), look))
        cols.append(m.col_box(name + "_c", size, c))
    # the drip area's grille, the head's front panel, the carbonating button on top, the tower's side panel
    parts.append(("drip", m.box((0.095, 0.11, 0.002), (-0.0005, 0.0, BASE_H + 0.001)), "tray"))
    parts.append(("head_face", m.box((0.002, 0.11, 0.08), (HEAD_X0 - 0.001, 0.0, (HEAD_Z0 + HEIGHT) / 2)), "abs_grey"))
    parts.append(("button", m.cylinder(0.022, 0.012, (0.06, 0.0, HEIGHT + 0.006)), "button"))
    parts.append(("tower_face", m.box((0.002, 0.10, 0.25), (TOWER_X0 - 0.001, 0.0, 0.17)), "abs_dark"))
    # the holder's hangers from the head, either side of its hinge, and the hinge pin
    for s in (-1.0, 1.0):
        parts.append((f"hanger_{'l' if s > 0 else 'r'}", m.box((0.016, 0.006, HEAD_Z0 - PIVOT[2] + 0.008), (PIVOT[0], s * 0.034, (HEAD_Z0 + PIVOT[2]) / 2 - 0.004)), "abs_dark"))
    parts.append(("pin", m.cylinder(0.003, 0.074, tuple(PIVOT), axis="y", segments=12), "steel"))
    return parts, cols


# -- the Snap-Lock holder and its jaws (authored at the holder's rest: tilted forward) ----------------------------------------------------------


def _holder_link():
    parts, cols = [], []
    cup = m.lathe([(R_CHAMBER, Z_CUP0), (R_CUP, Z_CUP0), (R_CUP, Z_CEIL + 0.006), (0.0, Z_CEIL + 0.006)], 32)
    parts.append(("cup", tilted(cup), "abs_dark"))
    parts.append(("ring", tilted(m.lathe([(R_CHAMBER, Z_CUP0 - 0.0005), (R_CUP + 0.0008, Z_CUP0 - 0.0005), (R_CUP + 0.0008, Z_CUP0 + 0.004), (R_CHAMBER, Z_CUP0 + 0.004)], 32)), "abs_grey"))
    parts.append(("lead_in", tilted(m.lathe([(R_LEAD, Z_CUP0 - H_LEAD), (R_SKIRT, Z_CUP0 - H_LEAD), (R_SKIRT, Z_CUP0 - 0.002), (R_CUP, Z_CUP0 + 0.002),
                                             (R_CHAMBER, Z_CUP0 + 0.002), (R_LEAD, Z_CUP0 - H_LEAD)], 32)), "abs_grey"))
    for s in (-1.0, 1.0):  # the lugs from the cup's back to the hinge
        lug = m.box((PIVOT[0] - 0.015, 0.006, 0.014), ((PIVOT[0] + 0.015) / 2, s * 0.026, PIVOT[2]))
        parts.append((f"lug_{'l' if s > 0 else 'r'}", tilted(lug), "abs_dark"))
    k = 0
    # the bore round the neck, and its ceiling
    for i in range(16):  # the bore, with a 4 mm chamfer at its mouth from the chamber's radius down to the bore's
        cols.append(m.col_hull(f"bore_{i}", tilted(_sector(R_BORE, R_CUP, 22.5 * i, 22.5 * (i + 1), Z_CHAMFER, Z_CEIL))))
        cols.append(m.col_hull(f"chamfer_{i}", tilted(_cone_sector(R_BORE, R_CHAMBER, R_CUP, 22.5 * i, 22.5 * (i + 1), Z_CHAMBER_TOP, Z_CHAMFER))))
    cols.append(m.col_hull("ceiling", tilted(m.cylinder(R_CUP, 0.006, (0.0, 0.0, Z_CEIL + 0.003), segments=16))))
    # the collar's chamber above the jaws, and the mouth's walls between the jaws (front and back)
    for i in range(12):
        cols.append(m.col_hull(f"chamber_{i}", tilted(_sector(R_CHAMBER, R_CUP, 30.0 * i, 30.0 * (i + 1), Z_JAW, Z_CHAMBER_TOP))))
    for name, a0 in (("mouth_b", -40.0), ("mouth_f", 140.0)):
        for j in range(2):
            cols.append(m.col_hull(f"{name}{j}", tilted(_sector(R_CHAMBER, R_CUP, a0 + 40.0 * j, a0 + 40.0 * (j + 1), Z_CUP0, Z_JAW))))
            k += 1
    for i in range(16):  # the mouth's lead-in cone
        cols.append(m.col_hull(f"lead_{i}", tilted(_cone_sector(R_CHAMBER, R_LEAD, R_SKIRT, 22.5 * i, 22.5 * (i + 1), Z_CUP0 - H_LEAD, Z_CUP0))))
    return m.hinge("holder", "Y", tuple(PIVOT), -TILT, 0.0, parts, cols, mass=0.06, spring=K_HOLDER, damping=0.05, friction=0.01, effort=20.0, armature=0.002)


def _jaw_mesh(s: float) -> m.Mesh:
    """One jaw, on the +y side for s = +1: ramped below, flat on top, 16 mm wide (x)."""
    pts = np.array([(JAW_RAMP_IN, Z_JAW - JAW_DEPTH), (JAW_OUT, Z_JAW - JAW_DEPTH), (JAW_OUT, Z_JAW), (JAW_IN, Z_JAW)])
    if s < 0:
        pts = np.c_[-pts[:, 0], pts[:, 1]][::-1]
    block = m.prism(pts, -JAW_HALF_X, JAW_HALF_X)
    return m.Mesh(block.v[:, [2, 0, 1]], block.f)  # the outline's (y, z), extruded along x


def _jaw_link(name: str, s: float):
    mesh = tilted(_jaw_mesh(s))
    lower, upper = (0.0, JAW_TRAVEL) if s > 0 else (-JAW_TRAVEL, 0.0)
    return m.slider(name, "Y", lower, upper, [(name, mesh, "pom")], [m.col_hull(name + "_c", mesh)], at=tuple(PIVOT), mass=0.03, spring=K_JAW,
                    damping=3.0, friction=0.01, effort=50.0, armature=0.01, parent="holder")


_fp, _fc = _frame()
MAKER = m.mechanism("maker", tuple(MAKER_POS), [_holder_link(), _jaw_link("jaw_l", 1.0), _jaw_link("jaw_r", -1.0)], LOOKS, frame_parts=_fp, frame_colliders=_fc,
                    friction=(1.0, 0.9), free_mass=3.0, frame_inertia=(0.054, 0.052, 0.015), solver_iters=48)

# -- the bottle ----------------------------------------------------------------------------------------------------------------------------------

_BODY_PROFILE = [(0.0, 0.0), (R_FOOT, 0.0), (0.0405, 0.004), (R_BODY, 0.012), (R_BODY, SHOULDER_Z0)]
_SHOULDER_PROFILE = [(R_BODY, SHOULDER_Z0), (0.036, 0.222), (0.024, 0.236), (R_NECK, SHOULDER_Z1)]


def _bottle():
    shell = m.lathe(_BODY_PROFILE[:-1] + _SHOULDER_PROFILE + [(R_NECK, COLLAR_Z0), (R_COLLAR, COLLAR_Z0), (R_COLLAR, COLLAR_Z1), (R_NECK, COLLAR_Z1),
                                                             (R_NECK, H_BOTTLE), (0.0, H_BOTTLE)], 40)
    parts = [("bottle", shell, "pet"),
             ("water", m.lathe([(0.0, 0.003), (0.039, 0.003), (0.0405, 0.012), (0.0405, 0.176), (0.0, 0.176)], 40), "water"),
             ("neck", m.lathe([(R_NECK + 0.0003, SHOULDER_Z1), (R_COLLAR + 0.0003, COLLAR_Z0), (R_COLLAR + 0.0003, COLLAR_Z1), (R_NECK + 0.0003, COLLAR_Z1),
                               (R_NECK + 0.0003, H_BOTTLE)], 32), "neck"),
             ("fill_line", m.lathe([(R_BODY + 0.0004, 0.175), (R_BODY + 0.0004, 0.178)], 40), "label")]
    cols = [m.col_hull("body_c", m.lathe(_BODY_PROFILE + [(0.0, SHOULDER_Z0)], 24)),
            m.col_hull("shoulder_c", m.lathe([(0.0, SHOULDER_Z0)] + _SHOULDER_PROFILE + [(0.0, SHOULDER_Z1)], 24)),
            m.col_hull("neck_c", m.cylinder(R_NECK, H_BOTTLE - SHOULDER_Z1, (0.0, 0.0, (SHOULDER_Z1 + H_BOTTLE) / 2), segments=20)),
            m.col_hull("collar_c", m.cylinder(R_COLLAR, COLLAR_Z1 - COLLAR_Z0, (0.0, 0.0, (COLLAR_Z0 + COLLAR_Z1) / 2), segments=24))]
    return m.free("bottle", (BOTTLE_START[0], BOTTLE_START[1], TOP), parts, cols, LOOKS, mass=BOTTLE_MASS, friction=(0.9, 0.8), com=BOTTLE_COM)


BOTTLE = _bottle()

# -- distractors (SimReady): a drinking glass, a roll of kitchen paper and dish soap on the counter
GLASS = SimReady("glass", pos=(0.17, 0.17, TOP), size=(0.080, 0.080, 0.085), mass=0.30, package="dishware_glass_tumbler_a01", usd="sm_dishware_glass_tumbler_a01_01.usd")
TOWEL = SimReady("paper_towel", pos=(0.40, 0.24, TOP), size=(0.124, 0.124, 0.231), mass=0.20, package="consumable_paperTowel_roll_a01",
                 usd="sm_consumable_paperTowel_roll_a01_01.usd")
SOAP = SimReady("dish_soap", pos=(0.42, 0.10, TOP), size=(0.066, 0.114, 0.271), mass=0.50, package="cleaning_dishSoap_a01", usd="sm_cleaning_dishSoap_a01_01.usd",
                yaw=math.pi / 2)

OBJECTS = [MAKER, BOTTLE, GLASS, TOWEL, SOAP]


# -- the Snap-Lock's latch, and the scorer -------------------------------------------------------------------------------------------------------


def holder_tilt(maker) -> float:
    """The holder's forward tilt from upright, degrees."""
    return TILT + maker.deg("holder")


def to_upright_holder(maker, points) -> np.ndarray:
    """World points in the frame of the holder as if turned upright: the jaws' tops at z = Z_JAW, the bore's axis on x = y = 0."""
    p = maker.to_local(points)
    return (p - PIVOT) @ _ry(-holder_tilt(maker)).T + PIVOT


def collar_state(maker, bottle) -> dict:
    """Where the bottle's collar is in the holder: its centre's distance from the bore's axis, its underside's height over the jaws' tops."""
    c = to_upright_holder(maker, bottle.to_world(np.array([[0.0, 0.0, (COLLAR_Z0 + COLLAR_Z1) / 2]])))[0]
    axis = to_upright_holder(maker, bottle.to_world(np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]])))
    a = axis[1] - axis[0]
    lean = math.degrees(math.acos(np.clip(a[2] / np.linalg.norm(a), -1.0, 1.0)))
    jaws = max(maker.joint("jaw_l"), -maker.joint("jaw_r"))
    under = c[2] - (COLLAR_Z1 - COLLAR_Z0) / 2 - Z_JAW
    return {"off": float(math.hypot(c[0], c[1])), "under": float(under), "lean": lean, "jaws": float(jaws)}


def captured(cs) -> bool:
    """The collar is above the jaws' tops (within 1.5 mm), inside the bore (within 5 mm of its axis), with the jaws closed back under it."""
    return cs["under"] >= -0.0015 and cs["off"] <= 0.005 and cs["lean"] <= 8.0 and cs["jaws"] <= 0.001


_BOXES = [  # the machine's outside, as boxes in its frame (lo, hi), for the hands-off check
    ((BASE_X0, -HALF_W, 0.0), (BASE_X1, HALF_W, BASE_H)),
    ((TOWER_X0, -HALF_W, 0.0), (BASE_X1, HALF_W, HEIGHT)),
    ((HEAD_X0, -HALF_W, HEAD_Z0), (BASE_X1, HALF_W, HEIGHT + 0.012)),
    ((-0.05, -0.04, Z_CUP0 - 0.02), (0.05, 0.04, HEAD_Z0)),  # the holder and its lead-in, at any tilt
]


def gaps(maker, bottle, tips) -> tuple[float, float]:
    """The nearest fingertip point's distance (m) from the bottle (a cylinder 8.5 x 28 cm) and from the machine."""
    q = bottle.to_local(tips)
    dr = np.maximum(np.hypot(q[:, 0], q[:, 1]) - R_BODY, 0.0)
    dz = np.maximum(np.maximum(-q[:, 2], q[:, 2] - H_BOTTLE), 0.0)
    g_bottle = float(np.min(np.hypot(dr, dz)))
    p = maker.to_local(tips)
    g_maker = min(float(np.min(np.linalg.norm(np.maximum(np.maximum(np.array(lo) - p, p - np.array(hi)), 0.0), axis=1))) for lo, hi in _BOXES)
    return g_bottle, g_maker


def bottom_gap(maker, bottle) -> float:
    """The lowest point of the bottle's bottom above the base's top (m)."""
    lo = maker.to_local(bottle.to_world(np.array([[0.0, 0.0, 0.0]])))[0]
    return float(lo[2] - BASE_H - R_FOOT * math.sin(math.radians(bottle.tilt_deg())))


class Trial:
    """The latch and the score. The harm record is cleared only at a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.latched = False
        self.harm = ""
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.info = {}
        self.maker0 = None
        self.n = 0
        if env is not None:
            m.set_target(env, "maker", "holder", math.radians(PRELOAD_DEG))
            self.maker0 = m.state(env, "maker").pos.copy()

    def step(self, env):
        mk, bt = m.state(env, "maker"), m.state(env, "bottle")
        if self.maker0 is None:
            self.maker0 = mk.pos.copy()
        self.n += 1
        cs = collar_state(mk, bt)
        tilt = holder_tilt(mk)
        cap = captured(cs)
        if not self.latched and cap and tilt <= 0.6:
            m.lock(env, "maker", "holder")
            self.latched = True
        # harms, recorded for the rest of the trial
        if bt.tilt_deg() > 45.0:
            self.harm = self.harm or "bottle fell over"
        if bt.lin_vel[2] < -1.4:
            self.harm = self.harm or "bottle dropped"
        slide = float(np.linalg.norm((mk.pos - self.maker0)[:2]))
        if slide > 0.02:
            self.harm = self.harm or "machine slid"
        if mk.pos[2] - self.maker0[2] > 0.015:
            self.harm = self.harm or "machine lifted"
        if mk.tilt_deg() > 5.0:
            self.harm = self.harm or "machine tipped"
        g_b, g_m = gaps(mk, bt, m.fingertips(env))
        gap = bottom_gap(mk, bt)
        hands_off = g_b >= 0.02 and g_m >= 0.02
        ok = self.latched and cap and bt.tilt_deg() <= 3.0 and 0.002 <= gap <= 0.015 and hands_off and not self.harm
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.harm:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif cap and tilt <= 5.0:
            self.score = 0.5
        elif cap:
            self.score = 0.25
        else:
            self.score = 0.0
        self.info = {"tilt": tilt, "cap": cap, "latched": self.latched, "harm": self.harm, "gap": gap, "g_b": g_b, "g_m": g_m, "slide": slide,
                     "mk_tilt": mk.tilt_deg(), "bt_tilt": bt.tilt_deg(), **cs}
        if self.n % 20 == 1:
            print("[soda] step %d score %.2f full %s %s" % (self.n, self.score, self.full,
                  " ".join(f"{k} {v:.4f}" if isinstance(v, float) else f"{k} {v}" for k, v in self.info.items())), flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)




# -- the demonstration (closed loop, with demokit.py) --------------------------------------------------------------------------------------------

GRIP_H = 0.13  # the hand's height on the bottle, from its bottom
GRIP_TILT = 60.0  # the grip's tilt from straight down
SWING_PAST = 6.0  # degrees the hand swings past upright, onto the holder's stop: the bottle gives a little in the hand
Z_LEAD = Z_CUP0 - H_LEAD  # the lead-in's bottom, in the upright holder's frame
SEAT_H = H_BOTTLE - COLLAR_Z0 + (Z_JAW - Z_LEAD)  # the neck tip's height over the lead-in's bottom when the collar sits on the jaws


def _neck_goal(mk, h: float, v: float, lean: float, tilt: float = TILT):
    """World goal for the bottle's neck tip and axis: the tip h m up the holder's axis from the lead-in's bottom and v m in front of that
    axis, with the holder at `tilt` degrees forward, the bottle leaning `lean` degrees forward. Read through the machine's seen pose."""
    tip = _ry(tilt) @ (np.array([-v, 0.0, Z_LEAD + h]) - PIVOT) + PIVOT
    ax = _ry(lean) @ np.array([0.0, 0.0, 1.0])
    R = mk.rotation()
    return mk.to_world(tip[None])[0], R @ ax


def demo_script(ctx):
    """Right hand only (the left is not needed: the machine stands on its own). It takes the bottle from the front round its body (fingers
    either side, 15 cm up), lifts it, leans it 30 degrees, brings the neck under the holder's mouth, into the lead-in, leans it back to the
    holder's 15 degrees while pushing it up the holder's axis until the collar is past the jaws, lets it settle on them, then swings the
    bottle's bottom back about the holder's hinge until the holder is upright and latched, lets go and moves away. Closed loop on the
    bottle's seen pose (where its neck tip is and which way its axis points), so a heavy bottle's droop in a soft hand is taken out.
    Failure checks (--mode): none does nothing; naive lifts the bottle upright under the holder and pushes it straight up, then lets go;
    early lets go before the swing back; hold does the full route but keeps hold of the bottle."""
    import demokit as d
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import axis_angle_matrix

    mode = ctx.mode
    if mode not in ("full", "none", "naive", "early", "hold"):
        raise ValueError(mode)
    r = ctx.right
    for _ in range(4):
        yield
    if mode == "none":
        for _ in range(200):
            yield
        return

    def bottle_now():
        b = ctx.state("bottle")
        return b.to_world(np.array([[0.0, 0.0, H_BOTTLE]]))[0], b.rotation()[:, 2]

    def report(label):
        mk = ctx.state("maker")
        tip, ax = bottle_now()
        print(f"[demo] t {ctx.t:5.1f} {label}: holder {TILT + mk.deg('holder'):5.2f} deg fwd, jaws {mk.joint('jaw_l') * 1000:4.1f} mm, "
              f"tip {np.round(tip, 3)}, axis {np.round(ax, 3)}, maker {np.round(mk.pos, 3)}", flush=True)

    state = {"c": None, "R": None}

    def steer(goal, t: float, settle: float = 0.6, gain: float = 0.3, label: str = "steer", until=None):
        """Bring the bottle's neck tip and axis along a straight path to `goal()` (tip, axis), in `t` s, then hold `settle` s: each step, the
        command moves by the path's step plus `gain` of the seen error, turning about the neck tip."""
        c, R = state["c"], state["R"]
        tip0, ax0 = bottle_now()
        g_tip, g_ax = goal()
        n, m_ = max(1, round(t * 20)), round(settle * 20)
        worst = 0.0
        for k in range(1, n + m_ + 1):
            s = ik.smoothstep(min(k / n, 1.0))
            g_tip, g_ax = goal()
            w_tip = tip0 + s * (g_tip - tip0)
            w_ax = ax0 + s * (g_ax - ax0)
            w_ax /= np.linalg.norm(w_ax)
            tip, ax = bottle_now()
            dp = np.clip(gain * (w_tip - tip), -0.004, 0.004)
            cr = np.cross(ax, w_ax)
            ang = math.asin(min(1.0, float(np.linalg.norm(cr))))
            turn = np.eye(3) if ang < 1e-6 else axis_angle_matrix(cr / np.linalg.norm(cr), min(gain * ang, math.radians(1.2)))
            c = tip + turn @ (c - tip) + dp
            R = turn @ R
            mm, dg = r._solve(c, R)
            worst = max(worst, mm)
            yield
            if until is not None and until():
                break
        state["c"], state["R"] = c, R
        r.grip = d._as_grip(R)
        tip, ax = bottle_now()
        print(f"[demo] '{label}': tip off by {np.linalg.norm(g_tip - tip) * 1000:.1f} mm, axis by "
              f"{math.degrees(math.acos(np.clip(ax @ g_ax, -1, 1))):.1f} deg, IK worst {worst:.1f} mm", flush=True)

    level = d.level(0)
    grip = d.tilted(GRIP_TILT, 0.0)  # fingers forward and 30 degrees down, closing across the bottle (along y)
    fwd = grip.rot[:, 2]
    b0 = ctx.pos("bottle")
    at = b0 + np.array([0.003, 0.0, GRIP_H])
    yield from r.move(at - 0.12 * fwd + np.array([0.0, 0.0, 0.06]), grip=grip, finger=FINGER_OPEN, t=2.5, label="up and in front of the bottle")
    yield from r.move(at - 0.07 * fwd, t=1.0, label="in line with the bottle")
    yield from r.servo(at - 0.07 * fwd, timeout=1.0, label="in line")
    yield from r.move(at, t=1.5, label="fingers either side of the bottle")
    yield from r.close(2 * R_BODY, 40.0, t=0.8, label="grip the bottle")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.06]), t=1.5, label="lift it")
    yield from r.move(r.at() + np.array([-0.06, 0.06, 0.0]), t=1.5, label="out in front of the machine")
    state["c"], state["R"] = ik.tcp_pose(r.q, r.base)
    report("lifted")

    mk0 = ctx.state("maker")  # the machine where it stands at the start: goals do not follow it if it is pushed
    mk = lambda: mk0  # noqa: E731
    if mode == "naive":  # upright, straight under the holder's mouth, straight up
        yield from steer(lambda: (mk().to_world(np.array([[-0.10, 0.0, BASE_H + 0.012 + H_BOTTLE]]))[0], np.array([0.0, 0.0, 1.0])), 2.5, label="to the front, upright")
        yield from steer(lambda: (mk().to_world(np.array([[0.0, 0.0, BASE_H + 0.006 + H_BOTTLE]]))[0], np.array([0.0, 0.0, 1.0])), 2.0, label="under the holder")
        yield from steer(lambda: (mk().to_world(np.array([[0.0, 0.0, BASE_H + GAP0 + H_BOTTLE + 0.006]]))[0], np.array([0.0, 0.0, 1.0])), 2.0, label="push straight up")
        report("pushed up")
    else:
        for label, t, h, v, lean in (("to the front of the machine, leaning", 2.5, -0.012, 0.10, 30.0),
                                     ("neck under the lead-in", 1.5, -0.006, 0.0, 30.0),
                                     ("neck into the lead-in", 1.2, 0.010, 0.0, 30.0),
                                     ("up and leaning back", 1.2, 0.022, 0.0, 20.0),
                                     ("to the holder's angle", 1.0, 0.032, 0.0, 15.0),
                                     ("push up the holder's axis", 1.5, SEAT_H + 0.0025, 0.0, 15.0),
                                     ("let the collar settle on the jaws", 1.0, SEAT_H, 0.0, 15.0)):
            yield from steer(lambda h=h, v=v, lean=lean: _neck_goal(mk(), h, v, lean), t, label=label)
            report(label)
        if mode != "early":
            yield from steer(lambda: _neck_goal(mk(), SEAT_H, 0.0, -SWING_PAST, -SWING_PAST), 3.0, settle=1.0, label="swing the bottom back about the holder's hinge",
                             until=lambda: TILT + ctx.deg("maker", "holder") < 0.65)  # the holder is on its stop: the Snap-Lock has clicked
            # stop pushing: command where the hand is now
            p_now, r_now = ik.tcp_pose(ctx.observed("right")[:6], r.base)
            r._solve(p_now, r_now)
            r.grip = d._as_grip(r_now)
            yield from r.wait(0.4)
            report("swung back")
            if mode == "hold":
                for _ in range(200):
                    yield
                report("still held")
                return
    yield from r.open(t=0.8, label="let go")
    report("let go")
    yield from r.move(r.at() - 0.09 * r.approach_axis(), t=1.5, label="back off")
    yield from r.move((0.20, -0.22, TOP + 0.25), grip=level, t=2.0, label="away")
    for k in range(120):
        yield
    report("end")
