"""Mount a volumetric infusion pump on an IV pole at a tape band, with the pump's own rear screw clamp.

On the station table: a 25 mm stainless IV pole mast stands in a floor flange screwed to the table (the table top stands in
for the pole's lower section; see the README). A 50 mm band of yellow tape on the pole marks the height the ward wants the
pump at, its centre 20 cm above the table. An Alaris GP-type volumetric infusion pump (148 x 225 x 148 mm, real size) stands
on its feet 6 cm in front of the pole, its face towards the arms and its back towards the pole. Its rear pole clamp is
folded out and unscrewed: the throat is 31 mm wide, 6 mm wider than the pole.

The clamp (in the pump's frame): a fixed jaw on one side of the throat, a moving jaw on the other, pushed by a lead screw
(1.75 mm pitch, right-hand) that a 35 mm knob on the clamp's side turns; the knob is on the pump's right side. The screw is
code: each step the moving jaw's drive target is set to the knob's turns times the pitch; once the jaw is stopped by the
pole 1 mm short of that target (200 N on the pole), the knob meets a stiff stop and turns no further. The pump hangs on
friction at the two jaws, 9 cm off the pole's axis, so until the jaw bites, a hand must carry it and keep it upright: let
go early and it drops down the pole.

    python demokit.py task.py OUT          (the closed-loop demo, demo_script below)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Mount the pump on the pole at the tape."

# -- the pump, in its own frame: origin at the middle of its underside, x across its width (the clamp's screw axis,
# +x the knob's side), y from its face (-y) to its back (+y), z up. Real size (scale 1.0) --------------------------------

W, D, H = 0.148, 0.148, 0.225  # BD Alaris GP volumetric pump: 148 mm (w) x 225 mm (h) x 148 mm (d), approx. 2.5 kg (DFU)
REAL_MASS = 2.5
PUMP_MASS = 1.0  # modelled mass (kg): what the YAM arms can carry and hold steady; the real pump is about 2.5 kg
KNOB_MASS, JAW_MASS = 0.03, 0.05
R_POLE = 0.0125  # a 25 mm IV pole (the clamp takes 15 to 40 mm)
CLEAR = 0.003  # each side of the pole in the open throat: 31 mm across
SPINE_Y = D / 2 + 0.006  # the clamp's back plate, 6 mm proud of the pump's back
Y_POLE = SPINE_Y + R_POLE  # the pole's axis when it sits against the back plate
Z_CLAMP = 0.150  # the clamp's centre line, 15 cm up the pump's back
JAW_H = 0.040  # both jaws' faces are 40 mm tall
PAD_Y = (Y_POLE - 0.014, Y_POLE + 0.016)  # the jaws' faces run from 1.4 cm in front of the pole's axis to 1.6 cm behind it
X_FIXED = -(R_POLE + CLEAR)  # the fixed jaw's face
X_JAW = R_POLE + CLEAR  # the moving jaw's face, open, at joint zero
JAW_T = 0.008  # moving jaw's thickness
X_ARM = X_JAW + JAW_T + 0.004  # the screw's housing arm, inner face
ARM_T = 0.010
PITCH = 0.00175  # m per turn: M12 coarse lead screw
TRAVEL = 2 * CLEAR + 0.004  # the jaw's joint range: 4 mm more than closing the throat onto a 25 mm pole
BITE = 0.001  # the jaw's drive target runs at most this far past where the pole stops it: 200 N (stiffness 200 kN/m)
JAW_K, JAW_EFFORT = 200000.0, 200.0
KNOB_STOP_K = 2.0  # N m/rad: the knob's stop once the screw is tight (hand-tight: the pads slip at about 1 N m)
KNOB_R, KNOB_L = 0.0175, 0.025
X_KNOB0 = W / 2 + 0.011  # the knob's inner face, 11 mm out from the pump's side
X_KNOB = X_KNOB0 + KNOB_L / 2  # its centre
HANDLE_GAP, HANDLE_T, HANDLE_W, HANDLE_LEN = 0.037, 0.016, 0.020, 0.110  # carry handle on top, along y: bar 37 mm above the top
HANDLE_Z = H + HANDLE_GAP + HANDLE_T / 2  # the bar's centre line

# -- the scene ----------------------------------------------------------------------------------------------------------

POLE_XY = (0.400, 0.000)
POLE_H = 0.55  # the mast stands 55 cm above the table
FLANGE_R, FLANGE_H = 0.030, 0.008
BAND_C, BAND_H = 0.200, 0.050  # tape band: centre 20 cm above the table top, 50 mm tall (2 in tape)
BAND_LO, BAND_HI = TOP + BAND_C - BAND_H / 2, TOP + BAND_C + BAND_H / 2
YAW = -90.0  # the pump's back (+y) towards +x, where the pole is; its face towards the arms; its +x (knob) towards -y
START_OFF = 0.060  # it starts 6 cm nearer the arms than where it hangs: the pole 3 cm clear of the open throat's mouth
HUNG_XY = (POLE_XY[0] - Y_POLE, POLE_XY[1])  # the pump's origin when the pole sits against the clamp's back plate
PUMP_START = (HUNG_XY[0] - START_OFF, HUNG_XY[1], TOP)

LOOKS = {
    "case": m.Look(color=m.rgb(0.90, 0.91, 0.92), roughness=0.45),
    "case_dark": m.Look(color=m.rgb(0.25, 0.27, 0.30), roughness=0.5),
    "screen": m.Look(color=m.rgb(0.05, 0.08, 0.10), roughness=0.2),
    "door": m.Look(color=m.rgb(0.55, 0.62, 0.70), roughness=0.4),
    "orange": m.Look(color=m.rgb(0.95, 0.45, 0.08), roughness=0.5),
    "key": m.Look(color=m.rgb(0.75, 0.78, 0.80), roughness=0.5),
    "clamp": m.Look(color=m.rgb(0.62, 0.64, 0.66), roughness=0.35, metallic=0.7),
    "knob": m.Look(color=m.rgb(0.07, 0.07, 0.08), roughness=0.6),
    "white": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.5),
    "steel": m.Look(color=m.rgb(0.80, 0.81, 0.83), roughness=0.25, metallic=0.9),
    "tape": m.Look(color=m.rgb(0.98, 0.82, 0.05), roughness=0.7),
    "rubber": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.9),
}


def _pump():
    cz = Z_CLAMP
    body = m.box((W, D, H), (0, 0, H / 2))
    face = -D / 2 - 0.001
    frame_parts = [
        ("body", body, "case"),
        ("screen", m.box((0.100, 0.002, 0.055), (0, face, 0.180)), "screen"),
        ("keys", m.box((0.110, 0.002, 0.030), (0, face, 0.130)), "key"),
        ("door", m.box((0.120, 0.003, 0.085), (0, face - 0.001, 0.060)), "door"),
        ("set_clamp", m.box((0.018, 0.004, 0.012), (0.040, face - 0.003, 0.030)), "orange"),
        ("feet", m.join(*[m.box((0.020, 0.020, 0.004), (sx * 0.055, sy * 0.055, -0.0005)) for sx in (-1, 1) for sy in (-1, 1)]), "rubber"),
        ("handle", m.join(m.box((HANDLE_W, HANDLE_LEN, HANDLE_T), (0, 0, HANDLE_Z)),
                          *[m.box((HANDLE_W, 0.014, HANDLE_GAP + HANDLE_T), (0, sy * (HANDLE_LEN / 2 - 0.007), H + (HANDLE_GAP + HANDLE_T) / 2))
                            for sy in (-1, 1)]), "case_dark"),
        # the folded-out pole clamp: back plate, fixed jaw, the screw's housing arm and the lead screw
        ("clamp_plate", m.box((2 * (X_ARM + ARM_T) + 0.006, SPINE_Y - D / 2, JAW_H + 0.016), (0.0, (SPINE_Y + D / 2) / 2, cz)), "clamp"),
        ("fixed_jaw", m.box((0.010, PAD_Y[1] - D / 2, JAW_H), (X_FIXED - 0.005, (PAD_Y[1] + D / 2) / 2, cz)), "clamp"),
        ("screw_arm", m.box((ARM_T, PAD_Y[1] - D / 2, JAW_H), (X_ARM + ARM_T / 2, (PAD_Y[1] + D / 2) / 2, cz)), "clamp"),
        ("screw", m.cylinder(0.006, X_KNOB0 - X_JAW, ((X_JAW + X_KNOB0) / 2, Y_POLE, cz), axis="x"), "steel"),
    ]
    frame_colliders = [
        m.col_box("c_body", (W, D, H), (0, 0, H / 2)),
        m.col_box("c_bar", (HANDLE_W, HANDLE_LEN, HANDLE_T), (0, 0, HANDLE_Z)),
        *[m.col_box(f"c_post{i}", (HANDLE_W, 0.014, HANDLE_GAP), (0, sy * (HANDLE_LEN / 2 - 0.007), H + HANDLE_GAP / 2)) for i, sy in enumerate((-1, 1))],
        m.col_box("c_plate", (2 * (X_ARM + ARM_T), SPINE_Y - D / 2, JAW_H + 0.016), (0, (SPINE_Y + D / 2) / 2, cz)),
        m.col_box("c_fixed", (0.010, PAD_Y[1] - D / 2, JAW_H), (X_FIXED - 0.005, (PAD_Y[1] + D / 2) / 2, cz)),
        m.col_box("c_arm", (ARM_T, PAD_Y[1] - D / 2, JAW_H), (X_ARM + ARM_T / 2, (PAD_Y[1] + D / 2) / 2, cz)),
    ]
    jaw = m.slider(
        "jaw", "X", -TRAVEL, 0.0005,
        parts=[("jaw_pad", m.box((JAW_T, PAD_Y[1] - PAD_Y[0], JAW_H), (X_JAW + JAW_T / 2, (PAD_Y[0] + PAD_Y[1]) / 2, cz)), "clamp")],
        colliders=[m.col_box("c_jaw", (JAW_T, PAD_Y[1] - PAD_Y[0], JAW_H), (X_JAW + JAW_T / 2, (PAD_Y[0] + PAD_Y[1]) / 2, cz))],
        mass=JAW_MASS, spring=JAW_K, damping=400.0, effort=JAW_EFFORT, friction=0.5)
    ribs = [m.box((KNOB_L, 0.006, 0.006), (X_KNOB, Y_POLE + (KNOB_R + 0.0005) * math.cos(a), cz + (KNOB_R + 0.0005) * math.sin(a)))
            for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
    knob = m.hinge(
        "knob", "X", (X_KNOB, Y_POLE, cz), float("-inf"), float("inf"),
        parts=[("knob_body", m.cylinder(KNOB_R, KNOB_L, (X_KNOB, Y_POLE, cz), axis="x", segments=40), "knob"),
               ("knob_ribs", m.join(*ribs), "knob"),
               ("knob_mark", m.box((0.002, 0.004, 0.012), (X_KNOB0 + KNOB_L + 0.0005, Y_POLE, cz + KNOB_R - 0.007)), "white")],
        colliders=[m.col_hull("c_knob", m.cylinder(KNOB_R, KNOB_L, (X_KNOB, Y_POLE, cz), axis="x", segments=24))],
        mass=KNOB_MASS, friction=0.02, damping=0.01, effort=20.0, inertia=(KNOB_MASS * KNOB_R ** 2 / 2, 2e-6, 2e-6))
    frame_mass = PUMP_MASS - KNOB_MASS - JAW_MASS
    inertia = (frame_mass / 12 * (D * D + H * H), frame_mass / 12 * (W * W + H * H), frame_mass / 12 * (W * W + D * D))
    return m.mechanism("pump", PUMP_START, [jaw, knob], LOOKS, frame_parts=frame_parts, frame_colliders=frame_colliders,
                       yaw_deg=YAW, friction=(1.0, 0.9), solver_iters=48, free_mass=frame_mass, frame_inertia=inertia)


def _pole():
    z1 = POLE_H
    parts = [
        ("mast", m.cylinder(R_POLE, z1 + 0.01, (0, 0, (z1 - 0.01) / 2), segments=32), "steel"),
        ("tape", m.cylinder(R_POLE + 0.0004, BAND_H, (0, 0, BAND_C), segments=32), "tape"),
        ("flange", m.cylinder(FLANGE_R, FLANGE_H, (0, 0, FLANGE_H / 2), segments=32), "case_dark"),
        ("cap", m.cylinder(R_POLE + 0.001, 0.012, (0, 0, z1 + 0.006)), "case_dark"),
    ]
    cols = [m.col_cylinder("c_mast", R_POLE, z1, (0, 0, z1 / 2)),
            m.col_cylinder("c_flange", FLANGE_R, FLANGE_H, (0, 0, FLANGE_H / 2))]
    return m.fixed("pole", (POLE_XY[0], POLE_XY[1], TOP), parts, cols, LOOKS, friction=(1.0, 0.9))


PUMP = _pump()
POLE = _pole()

# distractors (SimReady, loose; scoring ignores them): things on a bedside table
GLOVES = SimReady("glove_box", pos=(0.60, 0.30, TOP), size=(0.178, 0.117, 0.056), mass=0.30, yaw=0.3,
                  package="box_glove_a01", usd="sm_box_glove_a01_01.usd")
CUP = SimReady("water_cup", pos=(0.60, -0.30, TOP), size=(0.064, 0.064, 0.100), mass=0.02,
               package="dishware_plastic_cup_a01", usd="sm_dishware_plastic_cup_a01_01.usd")
BANDAGE = SimReady("bandage", pos=(0.14, 0.27, TOP), size=(0.049, 0.052, 0.042), mass=0.02,
                   package="bandage_medical_a01", usd="sm_bandage_medical_a01_01.usd")
DISTRACTORS = ("glove_box", "water_cup", "bandage")

OBJECTS = [PUMP, POLE, GLOVES, CUP, BANDAGE]

# -- the lead screw, and the scorer ---------------------------------------------------------------------------------------

SUCCESS_HOLD_S = 0.0  # the Trial counts its own 3 s hold
HOLD_S = 3.0
LEVEL_DEG = 3.0
SLIP = 0.005
CLEAR_HANDS = 0.03
HANG_GAP = 0.010  # the pump's lowest point at least 1 cm above the table: hanging, not standing
BEAR_GAP = 0.001  # each jaw within 1 mm of the pole: the clamp is screwed up on it
DROP_V = 0.3  # m/s: the pump moving down faster than this has been dropped, or slid down the pole (a free fall of 4.6 mm or more)
# the pump's outline, for the hands-clear and nothing-under tests: body, handle, clamp and knob
PUMP_LO = np.array([-W / 2, -D / 2 - 0.004, -0.001])
PUMP_HI = np.array([X_KNOB0 + KNOB_L, PAD_Y[1], HANDLE_Z + HANDLE_T / 2])


def screw_travel(knob_rad: float) -> float:
    """The jaw's travel (m, towards the pole) the screw asks for: right-hand thread, so turning the knob clockwise as you
    face it (negative about the pump's +x) closes the jaw."""
    return min(max(0.0, -knob_rad) * PITCH / (2 * math.pi), TRAVEL)


def clamp_reading(st) -> dict:
    """Geometry of the clamp on the pole, from the pump's pose and its jaw's travel (all in the pump's frame)."""
    c_world = st.to_world(np.array([[0.0, Y_POLE, Z_CLAMP]]))[0]
    pole_local = st.to_local(np.array([[POLE_XY[0], POLE_XY[1], c_world[2]]]))[0]
    xj = X_JAW + st.joint("jaw")  # the moving jaw's face (the joint is negative as it closes)
    in_throat = bool(X_FIXED - 0.002 <= pole_local[0] - R_POLE and pole_local[0] + R_POLE <= X_JAW + 0.002
                     and PAD_Y[0] + 0.004 <= pole_local[1] <= PAD_Y[1])
    jaw_gap = float(xj - (pole_local[0] + R_POLE))
    fixed_gap = float((pole_local[0] - R_POLE) - X_FIXED)
    return dict(z=float(c_world[2]), in_throat=in_throat, jaw_gap=jaw_gap, fixed_gap=fixed_gap,
                bearing=bool(in_throat and jaw_gap <= BEAR_GAP and fixed_gap <= BEAR_GAP))


_CORNERS = np.array([[sx * W / 2, sy * D / 2, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, H)])


def lowest(st) -> float:
    return float(st.to_world(_CORNERS)[:, 2].min())


def hand_gaps(st, tips) -> tuple[float, float]:
    """Each arm's closest finger point to the pump's outline (m): left, right."""
    local = st.to_local(tips)
    d = np.linalg.norm(np.maximum(np.maximum(PUMP_LO - local, local - PUMP_HI), 0.0), axis=1)
    return float(d[:12].min()), float(d[12:].min())


def hands_clear(st, tips) -> bool:
    return bool(min(hand_gaps(st, tips)) >= CLEAR_HANDS)


def nothing_under(st, points) -> bool:
    """No distractor stands under the pump's outline (1 cm margin): the pump is not propped on anything."""
    if len(points) == 0:
        return True
    local = st.to_local(np.asarray(points, float))
    under = (np.abs(local[:, 0] - (PUMP_LO[0] + PUMP_HI[0]) / 2) <= (PUMP_HI[0] - PUMP_LO[0]) / 2 + 0.01) & \
            (np.abs(local[:, 1] - (PUMP_LO[1] + PUMP_HI[1]) / 2) <= (PUMP_HI[1] - PUMP_LO[1]) / 2 + 0.01) & (local[:, 2] < 0.0)
    return bool(not under.any())


class Trial:
    """Updated once per control step after physics; score() and success() only read it."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):  # the ONLY place a recorded harm is cleared
        self.harm = False  # the pump dropped, knocked over, or off the table
        self.window = 0.0  # seconds the pump has hung hands-free on its clamp
        self.z0 = None  # the clamp's height when that began
        self.full = False
        self.value = 0.0
        self.last = {}

    def update(self, st, tips, dt, props=()):
        r = clamp_reading(st)
        tilt = st.tilt_deg()
        low = lowest(st)
        if tilt > 45.0 or low < TOP - 0.03 or float(st.lin_vel[2]) < -DROP_V:
            self.harm = True
        free = hands_clear(st, tips)
        alone = nothing_under(st, props)
        hanging = bool(r["bearing"] and low >= TOP + HANG_GAP and free and alone)
        if hanging:
            if self.z0 is None:
                self.z0 = r["z"]
            self.window += dt
        else:
            self.window, self.z0 = 0.0, None
        drop = 0.0 if self.z0 is None else max(0.0, self.z0 - r["z"])
        in_band = BAND_LO <= r["z"] <= BAND_HI
        level = tilt <= LEVEL_DEG
        read = hanging and self.window >= HOLD_S - 1e-9
        if self.harm or not read:
            self.value = 0.0
        elif in_band and level and drop <= SLIP:
            self.value, self.full = 1.0, True
        elif in_band and level:
            self.value = 0.75
        elif in_band:
            self.value = 0.5
        else:
            self.value = 0.25
        self.last = dict(r, tilt=tilt, low=low - TOP, free=free, alone=alone, hanging=hanging, window=self.window, drop=drop,
                         vz=float(st.lin_vel[2]), gaps=tuple(round(g, 3) for g in hand_gaps(st, tips)), harm=self.harm, knob_turns=-SCREW.turned / (2 * math.pi), jaw=st.joint("jaw"),
                         value=self.value)

    def step(self, environment):
        st = m.state(environment, "pump")
        screw(environment, st)
        try:
            from yam_arena.scene import object_states
            props = [np.asarray(s.pos, float) for s in object_states(environment, DISTRACTORS).values()]
        except Exception:  # noqa: BLE001  (a scene without the props)
            props = []
        self.update(st, m.fingertips(environment), environment.unwrapped.step_dt, props)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Screw:
    """The lead screw. PhysX reports an unlimited hinge's angle wrapped into (-2 pi, 2 pi], so the knob's turns are counted
    here, step by step. The jaw's drive target follows the turns, but never more than BITE past where the jaw is: once the
    pole stops the jaw, the jaw presses it with JAW_K * BITE = 200 N. At that moment the knob meets a stop: its joint is
    held where it is by a KNOB_STOP_K spring (the pads slip on it at about 1 N m, hand-tight). Turning it back 3 degrees
    past the stop frees it again."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.raw, self.turned = None, 0.0  # the knob's last reported angle; its unwrapped angle (rad, negative tightens)
        self.stop = None  # the unwrapped angle at which the knob met its stop

    def step(self, environment, st) -> None:
        raw = st.joint("knob")
        self.turned += 0.0 if self.raw is None else _wrap(raw - self.raw)
        self.raw = raw
        want = screw_travel(self.turned)
        got = -st.joint("jaw")
        if self.stop is None and want - got > BITE:
            m.lock(environment, "pump", "knob", stiffness=KNOB_STOP_K, effort=20.0)
            self.stop = self.turned
        elif self.stop is not None and self.turned > self.stop + math.radians(3.0):
            m.unlock(environment, "pump", "knob")
            self.stop = None
        m.set_target(environment, "pump", "jaw", -min(want, got + BITE))


SCREW = Screw()


def screw(environment, st) -> None:
    SCREW.step(environment, st)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)
m.on_reset(SCREW.reset)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo (closed loop, demokit) --------------------------------------------------------------------------


def _pump_point(st, p_local) -> np.ndarray:
    return st.to_world(np.array([p_local], float))[0]


def demo_script(ctx):
    """Left: takes the pump's carry handle from above and behind (fingers tilted 50 degrees from vertical, closing across
    the bar), lifts it 5 cm and carries it back onto the pole so the pole enters the clamp's open throat, and holds it there,
    upright, at the tape, servoing on the pump's pose (`carry`). Right: takes the clamp's knob from the pump's right side
    (fingers level, pointing at the pump, rolled 85 degrees) and turns it clockwise in strokes of 170 degrees, letting go
    and turning back between strokes, until a stroke turns it less than 90 degrees (the jaw has bitten and the knob met its
    stop). The right hand leaves, then the left; the pump hangs on its clamp. Reads only the pump's pose and the knob's angle.
    Red-team modes: none (nothing), onearm (the left hand alone: lifted to the tape and let go, the knob never turned),
    careless (the left lets go after the first stroke, before the jaw bites), low (clamped tight with the pump standing on
    the table), held (clamped at the tape, but the left hand keeps hold)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "low", "held"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    if mode == "none":
        yield from l.wait(6.0)
        return
    st = ctx.state("pump")
    bar = np.array([0.0, 0.006, HANDLE_Z])  # the grip point on the bar: 6 mm behind its middle, over the centre of mass
    g_bar = d.tilted(50.0, 0.0)  # pointing +x, tilted 50 degrees down from vertical; fingers close along y, across the bar
    ax = g_bar.rot[:, 2]
    g0 = d.level(90.0)  # right: pointing +y, fingers closing along x; rolled about +y for the knob's strokes
    rolled = lambda deg: d._as_grip(d.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(deg)) @ g0.rot)  # noqa: E731
    p_bar = _pump_point(st, bar)
    pre = p_bar - 0.06 * ax + np.array([0, 0, 0.02])
    yield from ctx.together(
        l.move(pre, grip=g_bar, finger=FINGER_OPEN, t=2.5, label="left above and behind the handle"),
        r.move(np.array([0.40, -0.135, TOP + 0.28]), grip=rolled(-85.0), finger=-0.030, t=2.5, label="right waits above the knob's path"))
    yield from l.servo(pre, timeout=0.8, label="left above and behind the handle")
    yield from l.move(p_bar, t=1.2, label="fingers round the bar")
    yield from l.servo(p_bar, timeout=1.0, label="fingers round the bar")
    yield from l.close(HANDLE_W, 40.0, t=0.6, label="grip the handle")

    lift = 0.0 if mode == "low" else BAND_C - Z_CLAMP  # the clamp's centre line onto the band's centre
    clamp_local = np.array([0.0, Y_POLE, Z_CLAMP])
    clamp_now = lambda: _pump_point(ctx.state("pump"), clamp_local)  # noqa: E731
    c0 = clamp_now()
    up = np.array([c0[0], c0[1], TOP + Z_CLAMP + lift + 0.004])
    goal_in = np.array([POLE_XY[0] - 0.002, POLE_XY[1], up[2]])  # the pole's axis 2 mm short of the clamp's back plate
    if mode == "low":
        up[2] = goal_in[2] = TOP + Z_CLAMP + 0.004
    yield from carry(ctx, l, lambda s: c0 + (up - c0) * min(1.0, s / 1.5), clamp_now, 2.2, label="lift")
    yield from carry(ctx, l, lambda s: up + (goal_in - up) * min(1.0, s / 2.0), clamp_now, 2.8, label="onto the pole")
    if mode == "low":
        low_goal = np.array([goal_in[0], goal_in[1], TOP + Z_CLAMP - 0.002])
        yield from carry(ctx, l, lambda s: low_goal, clamp_now, 1.0, label="down on the table")
        goal_in = clamp_now()  # standing on the table: hold it where it stands
    if mode == "onearm":
        yield from l.open(t=0.6)
        yield from l.move(l.at() + np.array([-0.06, 0.0, 0.03]), grip=g_bar, t=1.0)
        yield from l.wait(5.0)
        return

    # the right hand onto the knob
    def knob_point():
        return _pump_point(ctx.state("pump"), (X_KNOB, Y_POLE, Z_CLAMP))

    f_open, f_knob = -0.030, -(KNOB_R - 40.0 / 5000.0)
    yield from r.move(knob_point() + np.array([0, -0.04, 0]), grip=rolled(-85.0), finger=f_open, t=1.5, label="right beside the knob")
    yield from r.servo(knob_point() + np.array([0, -0.04, 0]), timeout=0.8, label="right beside the knob")
    yield from r.move(knob_point(), t=1.0, label="fingers round the knob")
    yield from r.servo(knob_point(), timeout=1.0, label="fingers round the knob")
    strokes, last, holding = 0, ctx.joint("pump", "knob"), mode != "low"  # low: standing on the table, the hand just stays
    while strokes < 14:
        hold = carry(ctx, l, lambda s: goal_in, clamp_now, 2.0, label="hold") if holding else l.wait(2.0)
        yield from r.move(r.at(), finger=f_knob, t=0.4, label=f"grip the knob {strokes + 1}")
        yield from ctx.together(r.turn(170.0, axis=(0.0, 1.0, 0.0), t=2.0, label="turn clockwise"), hold)
        yield from r.move(r.at(), finger=f_open, t=0.4, label="let go")
        now = ctx.joint("pump", "knob")
        turned = math.degrees(_wrap(last - now))
        last = now
        strokes += 1
        print(f"[demo] stroke {strokes}: knob turned {turned:.0f} deg", flush=True)
        if mode == "careless" and strokes == 1:
            holding = False
            yield from l.open(t=0.5)
            yield from l.move(l.at() + np.array([-0.06, 0.0, 0.03]), grip=g_bar, t=1.0)
        if turned < 90.0:
            break
        if clamp_now()[2] < goal_in[2] - 0.03:  # it has dropped off the hand: nothing to turn against
            break
        yield from r.turn(-170.0, axis=(0.0, 1.0, 0.0), t=1.2, label="turn back")
        yield from r.servo(knob_point(), timeout=0.6, label="recentre on the knob")
    yield from r.move(r.at() + np.array([0, -0.04, 0]), t=0.8, label="right backs off")
    yield from r.move(np.array([0.40, -0.14, TOP + 0.42]), grip=rolled(0.0), t=1.5, label="right up and away")
    if mode != "held" and mode != "careless":
        yield from l.open(t=0.6, label="let go of the handle")
        yield from l.move(l.at() + np.array([-0.06, 0.0, 0.03]), grip=g_bar, t=1.0, label="left backs off")
        yield from l.move(np.array([0.20, 0.25, TOP + 0.20]), grip=d.tilted(45.0, 0.0), t=1.5, label="left away")
    yield from l.wait(1.0)
    if mode in ("held", "low", "careless"):
        yield from l.wait(6.0)


def carry(ctx, arm, goal, read, t: float, gain: float = 0.25, tilt_gain: float = 0.3, label: str = "carry"):
    """Hold what the arm carries on a path: each step, shift the arm's command by `gain` of the error between `goal(s)` (s
    seconds from the start) and `read()`, and turn its grip by `tilt_gain` of the carried pump's tilt from upright, so the
    wrist's droop under the load is taken out. Reads only the pump's pose."""
    import demokit as d
    from yam_arena import ik
    from yam_arena.urdf import axis_angle_matrix

    c = ik.tcp_pose(arm.q, arm.base)[0]
    rot = arm.grip.rot.copy()
    worst = (0.0, 0.0)
    for k in range(max(1, round(t * d.CONTROL_HZ))):
        err = np.asarray(goal(k * d.DT), float) - read()
        c = c + np.clip(gain * err, -d.SERVO_STEP, d.SERVO_STEP)
        z = ctx.state("pump").rotation()[:, 2]
        axis = np.cross(z, np.array([0.0, 0.0, 1.0]))
        n = float(np.linalg.norm(axis))
        if n > 1e-6:
            rot = axis_angle_matrix(axis / n, tilt_gain * math.asin(min(1.0, n))) @ rot
        mm, deg = arm._solve(c, rot)
        worst = (max(worst[0], mm), max(worst[1], deg))
        yield
    arm.grip = d._as_grip(rot)
    arm._note(*worst, label)
