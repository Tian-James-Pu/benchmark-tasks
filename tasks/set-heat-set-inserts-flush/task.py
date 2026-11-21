"""Set four brass M3 heat-set inserts into a printed PETG plate with a soldering iron, and finish each one flush with a steel block.

On the bench, in front of the arms: a blue silicone soldering mat with an orange PETG sensor plate (40 x 80 x 12 mm) on it. Four brass
M3 heat-set inserts (5.7 mm long, 4.6 mm across) stand loose in the plate's four 4.0 mm holes, dropped in by hand, each 4.4 mm proud of
the plate's top face. To the right, a Hakko FX-8801 soldering iron (217 mm, 46 g) with an M3 insert tip lies tip-down in its holder tube,
switched on and hot. To the left, a hardened steel 1-2-3 block (25.4 x 50.8 x 76.2 mm, 0.48 kg) stands on end. A digital caliper and a mug are
also on the bench.

How inserts are set (CNC Kitchen, the inserts' maker): "always melt the inserts only to about 90% of the depth with the soldering iron
tip and do the last, short way with a tool"; "Hold them in the final position for a few seconds until the plastic has cooled back down
and solidified"; "the inserts have a tendency to move slightly out of the component immediately after melting". Here the flat face of
a steel 1-2-3 block, laid on the insert, does the last short way.

Model (declared in code, run every control step in `Plastic.step`):
- Each insert is a slider (along the hole) in the plate's articulation, held at its "set" depth by a stiff spring (40 kN/m). It
  collides as a ring round its 2.8 mm bore, with a lead-in chamfer at the mouth; the iron's tip collides as its shoulder plus a
  1.5 mm cone (the first part of its pin), which finds the bore.
- Heat: the iron heats an insert while its tip is on it: the shoulder within 0.8 mm of the insert's top, the tip's axis within 2 mm
  of the insert's, and the iron within 10 degrees of the hole's axis. On the iron the heat rises 0 to 1 in 2 s; off it, it falls 1 to 0
  in 10 s in air, 4 s with the steel block on the insert. Soft while heat >= 0.5 (1 s after the iron goes on; 5 s after it leaves).
- Flow: while soft, a push on the insert of more than 0.3 N (its spring deflected) moves its set depth down at 80 mm/s per mm of
  deflection beyond that, at most 3 mm/s; the block's own weight (4.7 N) is enough. While soft and not pushed, it creeps back up at
  0.15 mm/s, at most 0.35 mm above the deepest it reached. Cold, it does not move.
- Melt mark (the iron's hot shoulder on the plate's face): the iron's shoulder within 0.25 mm of the face while its tip is on an
  insert, or the iron's tip within 0.5 mm of the face anywhere on the plate. Latched for the nearest insert, shown as a dark ring.
- Harm (latched, cleared only by a reset): a fingertip within 1 cm of the iron's hot end (tip to 95 mm up); a fingertip within 1 cm of an
  insert's top while that insert is hot (heat over 0.05); the iron out of its holder and in no hand for 2 s; the iron or the block
  dropped below the table top.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena import demo as _harness_demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Set the four threaded inserts flush into the plate with the soldering iron, then put the iron back in its holder."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
SCORE_OVER = "end"
HOLD_STEPS = 20  # 1 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]

# -- the plate and inserts (the "plate" articulation's frame: origin on the table top under the plate's centre) -------------------------
PLATE_C = np.array([0.28, -0.03])  # plate centre, world x, y
PLATE = (0.040, 0.080, 0.012)  # x, y, z: 40 x 80 x 12 mm, long side along y
MAT_T = 0.002
FACE = MAT_T + PLATE[2]  # the plate's top face above the table
HOLES = [(-0.012, -0.030), (0.012, -0.030), (0.012, 0.030), (-0.012, 0.030)]  # in the plate's frame: 24 x 60 mm pattern
INS_L, INS_R = 0.0057, 0.0023  # CNC Kitchen M3 standard: 5.7 mm long, 4.6 mm across the knurl
PROUD0 = 0.0044  # loose in its hole: the insert's top 4.4 mm above the face
HOLE_DEPTH = 0.007  # blind hole 7 mm (1.3 mm deeper than the insert)
LOWER = -(PROUD0 + HOLE_DEPTH - INS_L)  # bottomed in its hole: top 1.3 mm below the face
UPPER = 0.0005
K_INS = 40000.0  # N/m: the spring that holds an insert at its set depth
# -- the plastic
T_SOFT_HEAT = 2.0  # s on the iron from 0 to 1
T_COOL_AIR, T_COOL_BLOCK = 10.0, 4.0
SOFT = 0.5
F_DEAD = 0.3  # N
K_FLOW = 80.0  # mm/s per mm of deflection beyond the dead band
V_MAX = 0.003  # m/s
CREEP_V, CREEP_MAX = 0.00015, 0.00035
ENGAGE_R, ENGAGE_Z, ENGAGE_TILT = 0.0020, 0.0008, 10.0
MARK_SHOULDER, MARK_FACE = 0.00025, 0.0005
HOT_INSERT = 0.05
# -- the scored bands (insert top above the face, m)
FLUSH_HI, FLUSH_LO = 0.00020, -0.00030
SEATED_HI = 0.0015

# -- the iron (its frame: origin at the centre of the tip's shoulder, +z along the iron to the cord) -------------------------------
IRON_L = 0.217  # Hakko FX-8801 with a T18 tip, without cord
IRON_MASS = 0.046
HOT_LEN = 0.095  # tip and heater barrel: the hot end
GRIP0, GRIP1, GRIP_R = 0.095, 0.200, 0.0095
GRASP_AT = 0.170  # where the hand takes it: 170 mm up from the tip
# -- the holder tube (square, 30 mm inside, 120 mm long), sloping 45 degrees: the iron's axis u, tip down towards -y
U = np.array([0.0, math.sqrt(0.5), math.sqrt(0.5)])  # tip -> cord, as the iron lies in its holder
N_DOWN = np.array([0.0, math.sqrt(0.5), -math.sqrt(0.5)])  # across the tube, towards its lowest wall
TUBE_IN, TUBE_L, WALL = 0.015, 0.120, 0.002
PIN_L = 0.0045  # the tip's pin, below its shoulder
GRASP_REST = np.array([0.28, -0.13, 0.16])  # the iron's axis 170 mm up from its tip, as it lies in the holder (above the table)
IRON_O = GRASP_REST - GRASP_AT * U  # its tip's shoulder
TUBE_O = IRON_O - (TUBE_IN - GRIP_R) * N_DOWN  # the tube's axis at its closed end (the grip's front end lies on the lowest wall)
IRON_Q = (-math.sin(math.radians(22.5)), 0.0, 0.0, math.cos(math.radians(22.5)))  # local z onto U: -45 degrees about x
# -- the steel 1-2-3 block (its frame: origin at the centre of its bottom face)
BLOCK = (0.0254, 0.0508, 0.0762)
BLOCK_MASS = 0.476  # All Industrial 55510: 2.10 lb the pair
BLOCK_XY = np.array([0.22, 0.12])

LOOKS = {
    "mat": m.Look(color=m.rgb(0.10, 0.35, 0.70), roughness=0.8),
    "petg": m.Look(color=m.rgb(0.95, 0.45, 0.08), roughness=0.45),
    "brass": m.Look(color=m.rgb(0.80, 0.62, 0.30), metallic=0.9, roughness=0.35),
    "bore": m.Look(color=m.rgb(0.15, 0.12, 0.08), roughness=0.6),
    "mark": m.Look(color=m.rgb(0.35, 0.12, 0.03), roughness=0.2),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.66), metallic=0.85, roughness=0.3),
    "dark": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.6),
    "grip": m.Look(color=m.rgb(0.08, 0.20, 0.55), roughness=0.7),
    "tip": m.Look(color=m.rgb(0.75, 0.55, 0.30), metallic=0.9, roughness=0.3),
    "stand": m.Look(color=m.rgb(0.18, 0.18, 0.20), metallic=0.4, roughness=0.5),
    "tube": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.35),
}


# -- the plate: mat and plate are the frame; each insert a slider ---------------------------------------------------------------------

def _plate():
    frame_parts = [
        ("mat", m.box((0.20, 0.30, MAT_T), (0.0, -0.02, MAT_T / 2)), "mat"),
        ("plate", m.box(PLATE, (0.0, 0.0, MAT_T + PLATE[2] / 2)), "petg"),
    ]
    frame_cols = [m.col_box("mat_c", (0.20, 0.30, MAT_T), (0.0, -0.02, MAT_T / 2))]
    # the plate collides as boxes round its four 4.0 mm holes (a 4 x 4 mm gap at each), so the tip's pin can follow its insert down
    xs = [-PLATE[0] / 2, -0.014, -0.010, 0.010, 0.014, PLATE[0] / 2]
    ys = [-PLATE[1] / 2, -0.032, -0.028, 0.028, 0.032, PLATE[1] / 2]
    for a_ in range(5):
        for b_ in range(5):
            if a_ in (1, 3) and b_ in (1, 3):
                continue
            size = (xs[a_ + 1] - xs[a_], ys[b_ + 1] - ys[b_], PLATE[2])
            ctr = ((xs[a_ + 1] + xs[a_]) / 2, (ys[b_ + 1] + ys[b_]) / 2, MAT_T + PLATE[2] / 2)
            frame_cols.append(m.col_box(f"plate_c{a_}{b_}", size, ctr))
    links = []
    for i, (hx, hy) in enumerate(HOLES):
        frame_parts.append((f"mark{i}", m.cylinder(0.0042, 0.0003, (hx, hy, FACE + 0.00015), segments=24), "mark", True))
        top = FACE + PROUD0
        parts = [
            (f"ins{i}_body", m.cylinder(INS_R, INS_L - 0.0002, (hx, hy, top - INS_L / 2 - 0.0001), segments=20), "brass"),
            (f"ins{i}_bore", m.cylinder(0.00125, 0.0002, (hx, hy, top - 0.0001), segments=16), "bore"),
        ]
        cols = _insert_ring(f"ins{i}", hx, hy, top)
        links.append(m.slider(f"ins{i}", "Z", LOWER, UPPER, parts, cols, at=(hx, hy, top), spring=K_INS, damping=40.0, friction=0.0, mass=0.01,
                              effort=400.0))
    return m.mechanism("plate", (PLATE_C[0], PLATE_C[1], TOP), links, LOOKS, frame_parts, frame_cols, friction=(0.8, 0.6))


# -- the holder: a weighted base and a sloping square tube -----------------------------------------------------------------------------

def _rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _holder():
    R = _rot_x(-45.0)  # tube frame (axis along its z) -> world: z onto U, y onto N_DOWN
    o = TUBE_O

    def place(mesh):
        return m.Mesh(mesh.v @ R.T + o, mesh.f)

    t = TUBE_IN + WALL / 2
    slabs = {
        "wall_low": m.box((2 * TUBE_IN + 2 * WALL, WALL, TUBE_L + PIN_L), (0.0, t, (TUBE_L - PIN_L) / 2)),
        "wall_high": m.box((2 * TUBE_IN + 2 * WALL, WALL, TUBE_L + PIN_L), (0.0, -t, (TUBE_L - PIN_L) / 2)),
        "wall_l": m.box((WALL, 2 * TUBE_IN, TUBE_L + PIN_L), (t, 0.0, (TUBE_L - PIN_L) / 2)),
        "wall_r": m.box((WALL, 2 * TUBE_IN, TUBE_L + PIN_L), (-t, 0.0, (TUBE_L - PIN_L) / 2)),
        "end": m.box((2 * TUBE_IN + 2 * WALL, 2 * TUBE_IN + 2 * WALL, WALL), (0.0, 0.0, -0.0015 - WALL / 2)),  # the pin's collider rests on it
    }
    parts, cols = [], []
    for name, mesh in slabs.items():
        w = place(mesh)
        parts.append((name, w, "tube"))
        cols.append(m.col_hull(name + "_c", w))
    base_c = (o[0], o[1] + 0.02, 0.006)
    parts.append(("base", m.box((0.09, 0.11, 0.012), base_c), "stand"))
    cols.append(m.col_box("base_c", (0.09, 0.11, 0.012), base_c))
    low = place(m.box((0.01, 0.002, 0.01), (0.0, t + WALL, 0.03)))  # a post from the base to the tube's lowest wall, 3 cm along
    p_top = low.v.mean(axis=0)
    parts.append(("post", m.box((0.012, 0.012, p_top[2] - 0.012), (o[0], p_top[1], 0.012 + (p_top[2] - 0.012) / 2)), "stand"))
    return m.fixed("holder", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.6, 0.5))


# -- the iron --------------------------------------------------------------------------------------------------------------------------

# The tip's centring pin (2.1 mm, 4.5 mm long, pointed) is drawn in full; it collides as a 1.5 mm cone that guides it into the bore.
PIN = m.lathe([(0.0, -0.0045), (0.0006, -0.0040), (0.00105, -0.0030), (0.00105, 0.0)], segments=12)


BORE_R = 0.00140  # M3 thread: about 2.8 mm across its crests
BORE_TOP_R = 0.00210  # a lead-in chamfer at the bore's mouth
PIN_COL = m.lathe([(0.0, -0.0015), (0.0011, 0.0)], segments=12)  # the pin collides as its first 1.5 mm, a cone: it centres the tip


def _insert_ring(name, hx, hy, top):
    """The insert's colliders: 8 segments of a ring round its bore (the shoulder presses on their tops; the pin goes in between them)."""
    cols = []
    for k in range(8):
        a0, a1 = math.radians(45.0 * k), math.radians(45.0 * (k + 1))
        pts = []
        for a in (a0, a1):
            c, s_ = math.cos(a), math.sin(a)
            pts += [(hx + INS_R * c, hy + INS_R * s_, top), (hx + INS_R * c, hy + INS_R * s_, top - INS_L),
                    (hx + BORE_TOP_R * c, hy + BORE_TOP_R * s_, top), (hx + BORE_R * c, hy + BORE_R * s_, top - 0.0008),
                    (hx + BORE_R * c, hy + BORE_R * s_, top - INS_L)]
        fan = np.array([(0, j, j + 1) for j in range(1, len(pts) - 1)])  # faces only so the points are written: PhysX takes their hull
        cols.append(m.col_hull(f"{name}_c{k}", m.Mesh(np.array(pts), fan)))
    return cols


def _octagon(r, z0, z1):
    a = [math.radians(22.5 + 45.0 * k) for k in range(8)]
    rr = r / math.cos(math.radians(22.5))  # flats at r
    return m.prism([(rr * math.cos(t), rr * math.sin(t)) for t in a], z0, z1)


def _iron():
    parts = [
        ("pin", PIN, "tip"),
        ("tip", m.lathe([(0.0025, 0.0), (0.0030, 0.010), (0.0040, 0.025)], segments=20), "tip"),
        ("barrel", m.cylinder(0.005, 0.070, (0.0, 0.0, 0.060), segments=20), "steel"),
        ("nut", m.cylinder(0.0065, 0.012, (0.0, 0.0, 0.089), segments=20), "steel"),
        ("grip", m.cylinder(GRIP_R, GRIP1 - GRIP0, (0.0, 0.0, (GRIP0 + GRIP1) / 2), segments=24), "grip"),
        ("tail", m.cylinder(0.004, IRON_L - GRIP1, (0.0, 0.0, (GRIP1 + IRON_L) / 2), segments=16), "dark"),
    ]
    cols = [
        m.col_cylinder("tip_c", 0.0025, 0.025, (0.0, 0.0, 0.0125)),
        m.col_hull("pin_c", PIN_COL),
        m.col_cylinder("barrel_c", 0.005, 0.070, (0.0, 0.0, 0.060)),
        m.col_hull("grip_c", _octagon(GRIP_R, GRIP0, GRIP1)),  # an octagonal prism: flats under the finger pads
        m.col_cylinder("tail_c", 0.004, IRON_L - GRIP1, (0.0, 0.0, (GRIP1 + IRON_L) / 2)),
    ]
    return m.free("iron", (IRON_O[0], IRON_O[1], TOP + IRON_O[2] + 0.0005), parts, cols, LOOKS, mass=IRON_MASS, friction=(1.2, 1.0), torsional_patch=0.03,
                  start_quat=IRON_Q, com=(0.0, 0.0, 0.10))


def _block():
    h = BLOCK[2]
    parts = [("block", m.box(BLOCK, (0.0, 0.0, h / 2)), "steel")]
    for k, z in enumerate((0.019, 0.038, 0.057)):  # the through holes seen on its broad faces
        for s in (-1, 1):
            parts.append((f"hole{k}{'pn'[s > 0]}", m.cylinder(0.004, 0.0005, (s * (BLOCK[0] / 2 + 0.0002), 0.0, z), axis="x", segments=16), "dark"))
    cols = [m.col_box("block_c", BLOCK, (0.0, 0.0, h / 2))]
    return m.free("block", (BLOCK_XY[0], BLOCK_XY[1], TOP + 0.0005), parts, cols, LOOKS, mass=BLOCK_MASS, friction=(0.8, 0.7))


OBJECTS = [
    _plate(),
    _holder(),
    _iron(),
    _block(),
    # distractors (SimReady, CC-BY 4.0): ignored by the scorer
    SimReady("caliper", pos=(0.40, 0.25, TOP), size=(0.238, 0.078, 0.008), mass=0.15, package="instrument_caliper_a01",
             usd="sm_instrument_caliper_a01_01.usd", yaw=0.0),
    SimReady("mug", pos=(0.42, 0.06, TOP), size=(0.114, 0.090, 0.080), mass=0.35, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
             yaw=math.radians(30.0)),
]


# -- geometry helpers ------------------------------------------------------------------------------------------------------------------

def hole_world(i) -> np.ndarray:
    return np.array([PLATE_C[0] + HOLES[i][0], PLATE_C[1] + HOLES[i][1]])


def _seg_dist(pts, a, b) -> np.ndarray:
    """Distance from each point to the segment a-b."""
    ab = b - a
    t = np.clip(((pts - a) @ ab) / float(ab @ ab), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


def _seg_seg(a0, a1, b0, b1, n=21) -> float:
    s = np.linspace(0.0, 1.0, n)[:, None]
    return float(np.min(_seg_dist(a0 + s * (a1 - a0), b0, b1)))


class Plastic:
    """The inserts' heat, set depth, flow, creep and melt marks; the harms. Cleared only in `reset` (mechkit's on_reset)."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.heat = np.zeros(4)
        self.set = np.zeros(4)  # joint target: 0 at the start, negative = sunk
        self.deepest = np.zeros(4)
        self.marked = [False] * 4
        self.engaged = [False] * 4
        self.harm = ""
        self.loose = 0
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.c = {}
        self.n = 0
        if env is not None:
            for i in range(4):
                m.set_target(env, "plate", f"ins{i}", 0.0)
                m.set_visible(f"{m.ENV_PATH}/plate/frame/visual/mark{i}", False)

    def step(self, env):
        self.n += 1
        p = m.state(env, "plate")
        iron = m.state(env, "iron")
        blk = m.state(env, "block")
        Ri = iron.rotation()
        ax = Ri[:, 2]
        tip = iron.pos - np.array([0.0, 0.0, TOP])  # the shoulder's centre, above the table
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ax[2])))))
        tips = m.fingertips(env) - np.array([0.0, 0.0, TOP])
        tops = np.array([FACE + PROUD0 + p.joint(f"ins{i}") for i in range(4)])
        engaged = [False] * 4
        for i in range(4):
            r = float(np.linalg.norm(tip[:2] - hole_world(i)))
            engaged[i] = r < ENGAGE_R and abs(tip[2] - tops[i]) < ENGAGE_Z and tilt < ENGAGE_TILT
        self.engaged = engaged
        # the block's flat face on an insert
        on_block = [False] * 4
        for i in range(4):
            loc = blk.to_local(np.array([[*hole_world(i), tops[i] + TOP]]))[0]
            on_block[i] = abs(loc[0]) < BLOCK[0] / 2 and abs(loc[1]) < BLOCK[1] / 2 and abs(loc[2]) < 0.001 and blk.tilt_deg() < 10.0
        for i in range(4):
            if engaged[i]:
                self.heat[i] = min(1.0, self.heat[i] + DT / T_SOFT_HEAT)
            else:
                self.heat[i] = max(0.0, self.heat[i] - DT / (T_COOL_BLOCK if on_block[i] else T_COOL_AIR))
            q = p.joint(f"ins{i}")
            if self.heat[i] >= SOFT:
                defl = self.set[i] - q  # pushed down: the joint below its target
                beyond = defl - F_DEAD / K_INS
                if beyond > 0.0:
                    self.set[i] -= DT * min(V_MAX, K_FLOW * beyond)
                    self.deepest[i] = min(self.deepest[i], self.set[i])
                else:
                    self.set[i] = min(self.set[i] + DT * CREEP_V, self.deepest[i] + CREEP_MAX)
            self.set[i] = float(np.clip(self.set[i], LOWER, UPPER))
            m.set_target(env, "plate", f"ins{i}", self.set[i])
            # melt marks
            if engaged[i] and tip[2] < FACE + MARK_SHOULDER:
                self._mark(i)
        in_plate = abs(tip[0] - PLATE_C[0]) < PLATE[0] / 2 + 0.001 and abs(tip[1] - PLATE_C[1]) < PLATE[1] / 2 + 0.001
        if in_plate and tip[2] < FACE + MARK_FACE and not any(engaged):
            self._mark(int(np.argmin([np.linalg.norm(tip[:2] - hole_world(i)) for i in range(4)])))
        # harms
        hot_a, hot_b = tip, tip + HOT_LEN * ax
        if float(np.min(_seg_dist(tips, hot_a, hot_b))) < 0.010 + 0.005:
            self._harm("a fingertip touched the iron's hot end")
        for i in range(4):
            if self.heat[i] > HOT_INSERT:
                c = np.array([*hole_world(i), tops[i]])
                if float(np.min(np.linalg.norm(tips - c, axis=1))) < 0.010:  # the reported tip point sits ~5 mm below the pad
                    self._harm(f"a fingertip touched hot insert {i}")
        held = self._held(tips, tip, ax)
        in_holder = self._in_holder(tip, ax)
        self.loose = 0 if (held or in_holder) else self.loose + 1
        if self.loose >= 40:
            self._harm("the iron was out of its holder and in no hand for 2 s")
        if tip[2] < -0.02 or blk.pos[2] - TOP < -0.02:
            self._harm("the iron or the block fell off the table")
        # the score
        rel = tops - FACE
        flush = [(FLUSH_LO <= rel[i] <= FLUSH_HI) and not self.marked[i] for i in range(4)]
        seated = [(FLUSH_HI < rel[i] <= SEATED_HI) and not self.marked[i] for i in range(4)]
        d_iron = float(np.min(_seg_dist(tips, tip, tip + IRON_L * ax)))
        released = d_iron >= 0.03
        d_ins = float(np.min([np.min(np.linalg.norm(tips - np.array([*hole_world(i), tops[i]]), axis=1)) for i in range(4)]))
        loc = blk.to_local(tips + np.array([0.0, 0.0, TOP]))
        out = np.maximum(np.abs(loc) - np.array([BLOCK[0] / 2, BLOCK[1] / 2, BLOCK[2] / 2]) * 1.0, 0.0)
        out[:, 2] = np.maximum(np.abs(loc[:, 2] - BLOCK[2] / 2) - BLOCK[2] / 2, 0.0)
        d_blk = float(np.min(np.linalg.norm(out, axis=1)))
        still = float(np.linalg.norm(iron.lin_vel)) < 0.01 and float(np.linalg.norm(blk.lin_vel)) < 0.01
        stowed = in_holder and released
        clear = released and d_ins >= 0.03 and d_blk >= 0.03
        ok = (not self.harm) and stowed and all(flush) and clear and still
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.harm or not stowed:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = round(0.2 * sum(flush) + 0.1 * sum(seated), 4)
        self.c = dict(rel=rel, heat=self.heat.copy(), engaged=engaged, on_block=on_block, marked=list(self.marked), held=held, in_holder=in_holder,
                      d_iron=d_iron, d_ins=d_ins, d_blk=d_blk, tip=tip, tilt=tilt)
        if os.environ.get("HSI_DEBUG") and self.n % int(os.environ.get("HSI_DEBUG_EVERY", "10")) == 0:
            self.report(env)

    def report(self, env, label=""):
        c = self.c
        print(f"[hsi] step {self.n} {label} rel(mm) {np.round(c['rel'] * 1000, 2)} heat {np.round(c['heat'], 2)} eng {c['engaged']} blk {c['on_block']} "
              f"marked {c['marked']} held {c['held']} holder {c['in_holder']} d_iron {c['d_iron'] * 1000:.0f} d_ins {c['d_ins'] * 1000:.0f} "
              f"d_blk {c['d_blk'] * 1000:.0f} hd {Plastic.held_diag} tip {np.round(c['tip'] * 1000, 1)} tilt {c['tilt']:.1f} harm '{self.harm}' score {self.score} full {self.full}",
              flush=True)
        if os.environ.get("HSI_DEBUG_GRIP"):
            base = env.unwrapped
            robot = base.scene["right_robot"]
            gi = robot.find_bodies("gripper")[0][0]
            pose = m._np(robot.data.body_link_pose_w)[0, gi]
            Rg = m.quat_matrix(pose[3:7])
            ax = m.state(env, "iron").rotation()[:, 2]
            print(f"[hsi]   gripper y {np.round(Rg[:, 1], 3)} iron {np.round(ax, 3)} angle "
                  f"{math.degrees(math.acos(min(1.0, abs(float(Rg[:, 1] @ ax))))):.1f} gripper-y tilt {math.degrees(math.acos(min(1.0, abs(float(Rg[2, 1]))))):.1f}",
                  flush=True)
        if os.environ.get("HSI_DEBUG_POSE"):
            tips = m.fingertips(env) - np.array([0.0, 0.0, TOP])
            blk = m.state(env, "block")
            print(f"[hsi]   right tips mean {np.round(tips[12:].mean(axis=0) * 1000, 1)} left tips mean {np.round(tips[:12].mean(axis=0) * 1000, 1)} "
                  f"iron axis {np.round(m.state(env, 'iron').rotation()[:, 2], 3)} block {np.round((blk.pos - [0, 0, TOP]) * 1000, 1)} tilt {blk.tilt_deg():.1f}",
                  flush=True)

    def _mark(self, i):
        if not self.marked[i]:
            self.marked[i] = True
            m.set_visible(f"{m.ENV_PATH}/plate/frame/visual/mark{i}", True)

    def _harm(self, why):
        if not self.harm:
            self.harm = why
            print(f"[hsi] HARM at step {self.n}: {why}", flush=True)

    @staticmethod
    def _held(tips, tip, ax) -> bool:
        """One hand's two fingers either side of the iron's grip: the line between the fingers' centres passes within 6 mm of the grip's axis."""
        g0, g1 = tip + GRIP0 * ax, tip + GRIP1 * ax
        held = False
        diag = []
        for a in range(2):
            fa, fb = tips[12 * a:12 * a + 6].mean(axis=0), tips[12 * a + 6:12 * a + 12].mean(axis=0)
            gap, d = float(np.linalg.norm(fa - fb)), _seg_seg(fa, fb, g0, g1)
            diag.append((round(gap * 1000, 1), round(d * 1000, 1)))
            held = held or (gap < 0.045 and d < 0.006)
        Plastic.held_diag = diag
        return held

    @staticmethod
    def _in_holder(tip, ax) -> bool:
        """Lying in its holder: the tip's shoulder within 15 mm of where it lay, the iron within 15 degrees of the tube's axis."""
        return float(np.linalg.norm(tip - IRON_O)) < 0.015 and float(ax @ U) > math.cos(math.radians(15.0))


PLASTIC = Plastic()
m.on_step(PLASTIC.step)
m.on_reset(PLASTIC.reset)


def score(objects) -> float:
    return float(PLASTIC.score)


def success(objects) -> bool:
    return bool(PLASTIC.full)


# -- the scripted demo (open loop, from the published starting layout) ----------------------------------------------------------------

class _Grip(float):
    """A waypoint 'yaw' that carries a whole TCP rotation (the harness's demo takes only a yaw about the vertical)."""

    rot = np.eye(3)


def _install_grips():
    if getattr(_harness_demo.grip_rotation, "_hsi", False):
        return
    plain = _harness_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return plain(yaw) if rot is None else rot

    grip_rotation._hsi = True
    grip_rotation._plain = plain
    _harness_demo.grip_rotation = grip_rotation


_install_grips()


def _axis_angle(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def grip(theta_deg=0.0, phi_deg=0.0, yaw_deg=None) -> _Grip:
    """Straight down turned yaw_deg about the vertical, or tilted theta_deg from down towards phi_deg (fingers closing across that direction)."""
    plain = _harness_demo.grip_rotation._plain
    g = _Grip(0.0)
    if yaw_deg is not None:
        g.rot = plain(math.radians(yaw_deg))
        return g
    phi = math.radians(phi_deg)
    g.rot = _axis_angle([math.sin(phi), -math.cos(phi), 0.0], math.radians(theta_deg)) @ plain(phi)
    return g


# Corrections measured in trials (the hand's droop and the iron's seat in the fingers); set from the debug runs.
IRON_DZ = float(os.environ.get("HSI_IRON_DZ", "0.0047"))
IRON_DXY = (float(os.environ.get("HSI_IRON_DX", "-0.0034")), float(os.environ.get("HSI_IRON_DY", "0.0016")))
# each push turns the iron a little in the fingers (its tip lands 1 to 3 mm along +y): per-hole corrections measured in trials
HOLE_DXY = [(0.0, 0.0), (-0.0005, -0.0028), (0.0, -0.0020), tuple(float(v) for v in os.environ.get("HSI_HOLE3_DXY", "-0.0003,-0.0035").split(","))]
RET_N = float(os.environ.get("HSI_RET_N", "0.006"))  # measured: the iron's axis entered the tube 6 mm low
HOLE_DZ = [0.0, 0.0] + [float(os.environ.get("HSI_HOLE_DZ", "0.0010"))] * 2  # the iron has slid about 1 mm down in the fingers by then
BLOCK_DZ = float(os.environ.get("HSI_BLOCK_DZ", "0.0"))
STOP_PROUD = 0.0008  # the iron stops with the insert 0.8 mm proud ("about 90%")
STOP_LAG = float(os.environ.get("HSI_STOP_LAG", "0.0002"))  # the hand runs on this far past its command under the push: measured
DEMO_MODE = os.environ.get("HSI_DEMO", "full")
ASIDE = tuple(float(v) for v in os.environ.get("HSI_ASIDE", "0.0,-0.065,0.04").split(","))  # where the iron waits while the block presses


def demo():
    """Right hand: the iron, by its grip, 150 mm up from the tip. Left hand: the steel block, by its narrow faces.
    For each insert: the iron's tip into it, held still 1.2 s while it heats, then down at 1.2 mm/s until the insert stands 0.8 mm proud;
    the iron up and aside; at once the block's flat face laid on the insert and let go, so its weight presses it flush and holds it
    while it sets (2.6 s); the block taken again and lifted.
    Then the block back where it stood and the iron back in its holder; both hands away.
    HSI_DEMO modes (red team): none, ironflush (the iron alone, to flush), ironly (the iron alone, stopped 0.8 mm proud, no block),
    late (the block only after 9 s), holdblock (the block pressed in the hand, never let go), coldblock (the block on cold inserts, no iron), keep (the iron kept in hand at the end),
    finger (closed fingers pressed on the hot insert instead of the block)."""
    mode = DEMO_MODE
    tilt = grip(45.0, 90.0)  # the iron as it lies: fingers closing along x, pointing +y and 45 degrees down
    level = grip(90.0, 90.0)  # the iron upright: fingers level, pointing +y
    a_tilt = tilt.rot[:, 2]
    bgrip = grip(yaw_deg=-90.0)  # the block: fingers closing along x
    up = np.array([0.0, 0.0, 1.0])
    W = lambda p: tuple(float(v) for v in (np.asarray(p) + np.array([0.0, 0.0, TOP])))  # noqa: E731
    R, L = [], []
    t_r = t_l = 0.0

    def r(dur, pos, finger, label, g):
        nonlocal t_r
        R.append(Waypoint(dur, W(pos), finger, label, g))
        t_r += dur

    def l(dur, pos, finger, label, g):
        nonlocal t_l
        L.append(Waypoint(dur, W(pos), finger, label, g))
        t_l += dur

    def r_wait_until(t):
        if t > t_r:
            r(t - t_r, np.asarray(R[-1].pos) - [0, 0, TOP], R[-1].finger, "wait", R[-1].yaw)

    def l_wait_until(t):
        if t > t_l:
            l(t - t_l, np.asarray(L[-1].pos) - [0, 0, TOP], L[-1].finger, "wait", L[-1].yaw)

    if mode == "none":
        return {"right": [Waypoint(3.0, W((0.15, -0.25, 0.20)), FINGER_OPEN, "stay", grip(yaw_deg=0.0))]}

    # -- the right hand takes the iron from its holder
    G = GRASP_REST
    if mode != "coldblock":
        r(2.5, G - 0.07 * a_tilt + 0.02 * up, FINGER_OPEN, "behind the iron's grip", tilt)
        r(1.2, G - 0.0 * a_tilt, FINGER_OPEN, "fingers either side of the grip", tilt)
        r(0.6, G, FINGER_CLOSED, "close on the grip", tilt)
        out = G + 0.135 * U
        r(1.5, out, FINGER_CLOSED, "draw it out along the tube", tilt)
        r(1.6, out + np.array([0.0, 0.0, 0.03]), FINGER_CLOSED, "turn it upright", level)
    # -- the left hand takes the block
    bb = np.array([BLOCK_XY[0], BLOCK_XY[1], BLOCK[2] - 0.015])
    park_l = np.array([0.20, 0.15, BLOCK[2] - 0.015 + 0.06])
    if mode == "finger":  # the careless way: no block; the closed fingers press each hot insert
        l(5.8, park_l, FINGER_CLOSED, "left hand ready, no block", bgrip)
    else:
        l(2.0, bb + 0.06 * up, FINGER_OPEN, "over the block", bgrip)
        l(1.0, bb, FINGER_OPEN, "fingers either side of the block", bgrip)
        l(0.6, bb, FINGER_CLOSED, "close on the block", bgrip)
        l(1.0, bb + 0.06 * up, FINGER_CLOSED, "lift the block", bgrip)
        l(1.2, park_l, FINGER_CLOSED, "block ready, clear of the plate", bgrip)

    for i in range(4):
        h = hole_world(i)
        dxy = np.array([IRON_DXY[0] + HOLE_DXY[i][0], IRON_DXY[1] + HOLE_DXY[i][1], 0.0])
        top0 = FACE + PROUD0
        if mode != "coldblock":
            above = np.array([h[0], h[1], top0 + 0.02 + GRASP_AT]) + dxy
            r(1.4, above, FINGER_CLOSED, f"iron over insert {i}", level)
            hover = np.array([h[0], h[1], top0 + 0.004 + GRASP_AT + IRON_DZ]) + dxy
            r(0.6, hover, FINGER_CLOSED, "tip just over the insert", level)
            r(0.5, hover, FINGER_CLOSED, "steady", level)
            touch = np.array([h[0], h[1], top0 - 0.0001 + GRASP_AT + IRON_DZ]) + dxy
            r(1.0, touch, FINGER_CLOSED, f"tip into insert {i}", level)
            r(1.2, touch, FINGER_CLOSED, "hold while it heats", level)
            stop = FACE + (0.0 if mode == "ironflush" else STOP_PROUD)
            depth = top0 - stop
            end = np.array([h[0], h[1], stop + (0.0 if mode == "ironflush" else STOP_LAG) + GRASP_AT + IRON_DZ + HOLE_DZ[i]]) + dxy
            r(depth / 0.0010, end, FINGER_CLOSED, f"melt insert {i} in", level)
            r(0.8, end, FINGER_CLOSED, "stop: let it catch up", level)
            aside = np.array([h[0] + ASIDE[0], h[1] + ASIDE[1], FACE + ASIDE[2] + GRASP_AT])
            r(1.0, end + np.array([0.0, 0.0, 0.015]), FINGER_CLOSED, "iron straight up off the insert", level)
            r(1.0, aside, FINGER_CLOSED, "iron aside", level)
        if mode in ("ironflush", "ironly"):
            continue
        # the block (or the closed fingers) onto the insert
        lag = 9.0 if mode == "late" else 0.0
        l_wait_until(t_r - 1.2 + lag)  # the block sets off as the iron leaves
        if mode == "finger":
            fz = FACE + 0.0
            l(1.0, np.array([h[0], h[1], fz + 0.013 + 0.03]), FINGER_CLOSED, "closed fingers over the insert", grip(yaw_deg=-90.0))
            l(0.8, np.array([h[0], h[1], fz + 0.013 - 0.002]), FINGER_CLOSED, "press with the fingers", grip(yaw_deg=-90.0))
            l(2.5, np.array([h[0], h[1], fz + 0.013 - 0.002]), FINGER_CLOSED, "hold", grip(yaw_deg=-90.0))
            l(0.6, np.array([h[0], h[1], fz + 0.06]), FINGER_CLOSED, "up", grip(yaw_deg=-90.0))
            continue
        bz = FACE + BLOCK[2] - 0.015
        l(0.9, np.array([h[0], h[1], bz + 0.015]), FINGER_CLOSED, f"block over insert {i}", bgrip)
        l(0.6, np.array([h[0], h[1], bz + 0.0012 + BLOCK_DZ]), FINGER_CLOSED, f"block's face down on insert {i}", bgrip)
        if mode == "holdblock":  # the block held in the hand, not let go
            l(3.1, np.array([h[0], h[1], bz - 0.002 + BLOCK_DZ]), FINGER_CLOSED, "press it with the block held", bgrip)
        else:
            l(0.5, np.array([h[0], h[1], bz + 0.0012 + BLOCK_DZ]), FINGER_OPEN, "let go: its weight presses the insert", bgrip)
            l(2.6, np.array([h[0], h[1], bz + 0.0005 + BLOCK_DZ]), FINGER_OPEN, "the insert sets under the block", bgrip)
            l(0.5, np.array([h[0], h[1], bz + 0.0005 + BLOCK_DZ]), FINGER_CLOSED, "take the block again", bgrip)
        l(0.6, np.array([h[0], h[1], bz + 0.03]), FINGER_CLOSED, "block up", bgrip)
        l(1.0, park_l, FINGER_CLOSED, "block back, clear of the iron", bgrip)
        if mode != "coldblock":
            r_wait_until(t_l)
    if mode not in ("ironflush", "ironly", "finger"):
        l(1.2, park_l, FINGER_CLOSED, "block away", bgrip)
        l(1.2, bb + 0.01 * up, FINGER_CLOSED, "block down where it stood", bgrip)
        l(0.6, bb + 0.01 * up, FINGER_OPEN, "let go of the block", bgrip)
        l(1.0, bb + 0.08 * up, FINGER_OPEN, "hand up", bgrip)
        l(1.5, np.array([0.15, 0.25, 0.20]), FINGER_OPEN, "left hand clear", grip(yaw_deg=0.0))
    elif mode == "finger":
        l(1.5, np.array([0.15, 0.25, 0.20]), FINGER_OPEN, "left hand clear", grip(yaw_deg=0.0))
    else:
        l(1.0, park_l + 0.0, FINGER_CLOSED, "block held aside", bgrip)
        l(1.2, bb + 0.01 * up, FINGER_CLOSED, "block down where it stood", bgrip)
        l(0.6, bb + 0.01 * up, FINGER_OPEN, "let go of the block", bgrip)
        l(1.0, bb + 0.08 * up, FINGER_OPEN, "hand up", bgrip)
        l(1.5, np.array([0.15, 0.25, 0.20]), FINGER_OPEN, "left hand clear", grip(yaw_deg=0.0))
    if mode == "coldblock":
        return {"left": L}
    # -- the iron back in its holder
    out = G + 0.135 * U
    r(1.8, out + np.array([0.0, 0.0, 0.03]), FINGER_CLOSED, "iron over its holder, upright", level)
    if mode == "keep":
        r(4.0, out + np.array([0.0, 0.0, 0.03]), FINGER_CLOSED, "keep holding it", level)
        return {"right": R, "left": L}
    lift = -RET_N * N_DOWN  # the iron has turned a few degrees steeper in the fingers: carry it higher across the tube
    r(1.6, out + lift, FINGER_CLOSED, "tilt it to the tube's slope", tilt)
    r(2.2, G + 0.003 * U + lift, FINGER_CLOSED, "slide it down the tube", tilt)
    r(0.6, G + 0.003 * U + lift, FINGER_OPEN, "let go", tilt)
    r(1.0, G - 0.07 * a_tilt + 0.02 * up, FINGER_OPEN, "fingers back off the grip", tilt)
    r(1.5, np.array([0.15, -0.25, 0.20]), FINGER_OPEN, "right hand clear", grip(yaw_deg=0.0))
    end = max(t_r, t_l) + 2.0
    r_wait_until(end)
    l_wait_until(end)
    return {"right": R, "left": L}
