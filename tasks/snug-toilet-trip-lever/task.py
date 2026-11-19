"""Snug a toilet's loose trip lever: the locknut inside the tank has backed off, so the handle assembly turns in its hole and the handle
has dropped 15 degrees out of level. Turn the handle back to level and hold it from outside while the other hand turns the left-hand
locknut inside the tank until it is snug, without overtightening it.

Model (declared in code):
- Tank ("tank", a fixed articulation): its frame is the vitreous-china tank (walls 12 mm, lid off), the overflow tube and the fill valve.
  Its origin is the trip-lever hole's centre on the front wall's outer face; the tank's lower part stands below the bench top and is
  not modelled (a visual water surface marks the water line 4.5 cm below the hole). The tank stands with its front face towards
  the right arm.
- Link "bushing": the trip lever assembly (escutcheon with two flats, chrome handle, threaded bushing, shaft and trip arm) turning as one
  in the loose hole (hinge about the hole's axis, X), with gravity: loose, it hangs on its stop 15 degrees below level (authored there).
  The handle's own flushing swing is not modelled.
- Link "nut" (child of "bushing"): the plastic locknut on the bushing's left-hand thread; its joint is its angle on the thread, positive
  counterclockwise as seen from inside the tank (tightening). Thread drag is the nut joint's friction and damping, so a nut turned while
  nobody holds the handle drags the whole assembly round with it. At SEAT_DEG the nut meets the spacer on the wall's inside face: past it
  a stiff spring (the washer compressing) resists further turning, and the assembly is clamped (its joint locked).
- Overtightening (Kohler: "Do not overtighten the locknut, or the trip lever will not move freely"): the nut more than OVER_DEG past its
  seat for 3 control steps running, at any time. The nut at the end of the thread (2 turns loose) would drop into the tank: scored 0.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
import demokit as d  # noqa: E402  (lets demo() waypoints carry a full grip rotation)
from simready import SimReady  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "The toilet's flush handle is loose and has dropped out of level. Fix it so it is level and firm."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- layout (world metres) -----------------------------------------------------------------------------------------------------------------
# The tank stands with its front face towards the right arm (facing -y), so the hole's axis runs along world y; its left end (as one
# faces the front) is towards the robot (-x). Tank frame: x into the tank from the front face, y along the front face towards the
# tank's left end, z up. World = O + Rz(90) * tank frame: tank x -> world +y, tank y -> world -x.
XH = 0.20  # the hole's centre, world x
YF = 0.0  # the front wall's outer face, world y
ZH = TOP + 0.08  # the hole's centre, 8 cm above the bench top
O = np.array([XH, YF, ZH])  # the tank's origin
YAW = 90.0
WALL = 0.012
DEPTH, Y_LEFT, Y_RIGHT, RIM = 0.195, 0.075, -0.375, 0.04  # tank outside, in the tank's frame (rim 4 cm above the hole)
FLOOR = -0.08  # the bench top, in the tank's frame
WATER = -0.045  # the water line, 4.5 cm below the hole

START_DEG = -15.0  # the handle's starting angle from level (negative: its end has dropped)
HANDLE_L, HANDLE_T, HANDLE_H = 0.10, 0.008, 0.016  # chrome handle: length (along +Y, outboard), thickness (X), height
HANDLE_X = -0.029  # its centre plane, in front of the escutcheon
ESC_R, ESC_FLAT, ESC_X0 = 0.022, 0.018, -0.025  # escutcheon radius, half distance across its flats, its front face
SPACER = 0.005  # the plastic spacer between the wall's inside face and the nut
NUT_AF, NUT_X0, NUT_X1 = 0.036, WALL + SPACER, WALL + SPACER + 0.015  # locknut: 36 mm across flats, 15 mm long
SEAT_DEG = 90.0  # the nut starts a quarter turn back from its seat
OVER_DEG = 30.0  # past the seat by more than this: overtightened
K_CLAMP = 2.5  # N m/rad: the washer compressing past the seat (1.3 N m at the overtightening mark)
OFF_DEG = -700.0  # two turns loose: the nut leaves the thread
LEVEL_TOL = 5.0
CLEAR = 0.03


def to_world(p):
    p = np.asarray(p, dtype=float)
    return O + np.array([-p[1], p[0], p[2]])


LOOKS = {
    "china": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.15),
    "chrome": m.Look(color=m.rgb(0.82, 0.82, 0.84), metallic=1.0, roughness=0.15),
    "nut": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.5),
    "mark": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.5),
    "brass": m.Look(color=m.rgb(0.72, 0.58, 0.30), metallic=0.8, roughness=0.35),
    "plastic_grey": m.Look(color=m.rgb(0.45, 0.47, 0.50), roughness=0.5),
    "plastic_black": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.5),
    "water": m.Look(color=m.rgb(0.55, 0.70, 0.80), roughness=0.05, opacity=0.6),
}


def _x_cyl(r, x0, x1, y=0.0, z=0.0, segments=32):
    return m.cylinder(r, x1 - x0, ((x0 + x1) / 2, y, z), axis="x", segments=segments)


def _x_prism(outline, x0, x1):
    """An extrusion along X from an outline in (Y, Z)."""
    p = m.prism(outline, x0, x1)
    return m.Mesh(p.v[:, [2, 0, 1]], p.f)


def _hex(af):
    r = af / math.sqrt(3.0)  # corner radius
    return [(r * math.cos(math.radians(30 + 60 * k)), r * math.sin(math.radians(30 + 60 * k))) for k in range(6)]  # flats face +-Y


def _escutcheon_outline():
    pts = []
    for k in range(48):
        a = 2 * math.pi * k / 48
        y, z = ESC_R * math.cos(a), ESC_R * math.sin(a)
        pts.append((max(-ESC_FLAT, min(ESC_FLAT, y)), z))
    out = []
    for p in pts:
        if not out or abs(p[0] - out[-1][0]) > 1e-9 or abs(p[1] - out[-1][1]) > 1e-9:
            out.append(p)
    return out


# -- the tank (frame) -----------------------------------------------------------------------------------------------------------------------


def _tank():
    parts, cols = [], []
    h = RIM - FLOOR
    zc = (RIM + FLOOR) / 2
    walls = (
        ("front", (WALL, Y_LEFT - Y_RIGHT, h), (WALL / 2, (Y_LEFT + Y_RIGHT) / 2, zc)),
        ("back", (WALL, Y_LEFT - Y_RIGHT, h), (DEPTH - WALL / 2, (Y_LEFT + Y_RIGHT) / 2, zc)),
        ("left", (DEPTH - 2 * WALL, WALL, h), (DEPTH / 2, Y_LEFT - WALL / 2, zc)),
        ("right", (DEPTH - 2 * WALL, WALL, h), (DEPTH / 2, Y_RIGHT + WALL / 2, zc)),
    )
    for name, size, c in walls:
        parts.append((name, m.box(size, c), "china"))
        cols.append(m.col_box(name + "_c", size, c))
    inner = (DEPTH - 2 * WALL, Y_LEFT - Y_RIGHT - 2 * WALL)
    parts.append(("water", m.box((inner[0], inner[1], 0.001), (DEPTH / 2, (Y_LEFT + Y_RIGHT) / 2, WATER)), "water"))
    # flush valve's overflow tube (2.5 cm across, its top 2 cm below the hole) and the fill valve at the right end
    ov = (0.11, -0.15)
    parts.append(("spacer", _x_cyl(0.017, WALL, WALL + SPACER), "plastic_grey"))
    parts.append(("overflow", m.cylinder(0.0125, -0.02 - FLOOR, (ov[0], ov[1], (FLOOR - 0.02) / 2)), "plastic_grey"))
    cols.append(m.col_cylinder("overflow_c", 0.0125, -0.02 - FLOOR, (ov[0], ov[1], (FLOOR - 0.02) / 2)))
    fv = (0.12, -0.32)
    parts.append(("fill_valve", m.cylinder(0.02, 0.03 - FLOOR, (fv[0], fv[1], (FLOOR + 0.03) / 2)), "plastic_black"))
    cols.append(m.col_cylinder("fill_valve_c", 0.02, 0.03 - FLOOR, (fv[0], fv[1], (FLOOR + 0.03) / 2)))
    return parts, cols


# -- the trip lever assembly and its locknut ----------------------------------------------------------------------------------------------


def _bushing_link():
    t = START_DEG
    lever = m.box((HANDLE_T, HANDLE_L, HANDLE_H), (HANDLE_X, HANDLE_L / 2, 0.0)).turned(t, "x")
    hub = _x_cyl(0.012, ESC_X0 - 0.010, ESC_X0)
    esc = _x_prism(_escutcheon_outline(), ESC_X0, 0.0).turned(t, "x")
    thread = _x_cyl(0.0105, 0.0, NUT_X1 + 0.012)
    shaft = _x_cyl(0.005, NUT_X1 + 0.012, 0.062)
    arm = m.box((0.004, 0.17, 0.012), (0.060, -0.085, 0.0)).turned(t, "x")
    parts = [("handle", lever, "chrome"), ("hub", hub, "chrome"), ("escutcheon", esc, "chrome"), ("thread", thread, "plastic_black"),
             ("shaft", shaft, "brass"), ("trip_arm", arm, "brass")]
    cols = [m.col_hull("handle_c", lever), m.col_hull("hub_c", hub), m.col_hull("escutcheon_c", esc)]
    # Gravity on: loose, the assembly hangs on its handle and rests 15 degrees below level (its lower stop, the trip arm on the flush
    # valve's flapper chain), so an unclamped handle drops back. Centre of mass 2.5 cm out along the handle's side.
    com = (HANDLE_X / 2, 0.025 * math.cos(math.radians(t)), 0.025 * math.sin(math.radians(t)))
    return m.hinge("bushing", "X", (0.0, 0.0, 0.0), 0.0, 45.0, parts, cols, mass=0.10, friction=0.01, damping=0.05, armature=0.002,
                   gravity=True, com=com)


def _nut_link():
    body = _x_prism(_hex(NUT_AF), NUT_X0, NUT_X1)
    mark = m.box((0.001, 0.003, 0.012), (NUT_X1 + 0.0005, 0.0, 0.0105))  # a painted line on the nut's face, to read its angle by
    return m.hinge("nut", "X", (0.0, 0.0, 0.0), OFF_DEG - 20.0, SEAT_DEG + 60.0, [("nut", body, "nut"), ("mark", mark, "mark")],
                   [m.col_hull("nut_c", body)], mass=0.02, friction=0.05, damping=0.4, armature=0.001, parent="bushing",
                   inertia=(4e-6, 3e-6, 3e-6))


_tp, _tc = _tank()
TANK = m.mechanism("tank", tuple(O), [_bushing_link(), _nut_link()], LOOKS, frame_parts=_tp, frame_colliders=_tc, friction=(1.2, 1.0),
                   yaw_deg=YAW)

# -- distractors (SimReady, loose) ---------------------------------------------------------------------------------------------------------
SOAP = SimReady("bar_soap", pos=(0.33, -0.16, TOP), size=(0.060, 0.094, 0.033), mass=0.11, package="cleaning_barSoap_a01",
                usd="sm_cleaning_barSoap_a01_01.usd", yaw=0.4)
SPONGE = SimReady("sponge", pos=(0.14, -0.23, TOP), size=(0.071, 0.101, 0.033), mass=0.02, package="cleaning_sponge_b01",
                  usd="sm_cleaning_sponge_b01_01.usd", yaw=-0.3)
TOWEL = SimReady("paper_towel", pos=(0.47, -0.19, TOP), size=(0.124, 0.124, 0.231), mass=0.20, package="consumable_paperTowel_roll_a01",
                 usd="sm_consumable_paperTowel_roll_a01_01.usd")

OBJECTS = [TANK, SOAP, SPONGE, TOWEL]


# -- the thread's seat and the clamp --------------------------------------------------------------------------------------------------------


def _drive(env, part, link, target, stiffness):
    asset, j = m._joint(env, part, link)
    m._TOUCHED.add((part, link))
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


class Thread:
    """Past the seat the washer pushes back (K_CLAMP); from the seat on (within 3 degrees) the assembly is clamped in its hole."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.clamped = False
        self.sprung = False

    def step(self, env):
        s = m.state(env, "tank")
        a = s.deg("nut")
        past = a > SEAT_DEG
        if past != self.sprung:
            _drive(env, "tank", "nut", math.radians(SEAT_DEG), K_CLAMP if past else 0.0)
            self.sprung = past
        seated = a >= SEAT_DEG - 3.0
        if seated and not self.clamped:
            m.lock(env, "tank", "bushing")
            self.clamped = True
        elif not seated and self.clamped:
            m.unlock(env, "tank", "bushing")
            self.clamped = False


THREAD = Thread()
m.on_step(THREAD.step)
m.on_reset(THREAD.reset)


# -- the scorer ------------------------------------------------------------------------------------------------------------------------------


def _box_gap(q, size, center):
    dd = np.maximum(np.abs(q - np.asarray(center)) - np.asarray(size) / 2, 0.0)
    return np.linalg.norm(dd, axis=1)


def hand_gap(s, tips) -> float:
    """The nearest fingertip point's distance (m) from the handle, escutcheon or nut, in the assembly's own turned frame."""
    q = s.to_local(tips)
    a = -math.radians(START_DEG + s.deg("bushing"))
    ca, sa = math.cos(a), math.sin(a)
    r = np.c_[q[:, 0], ca * q[:, 1] - sa * q[:, 2], sa * q[:, 1] + ca * q[:, 2]]
    g = [_box_gap(r, (HANDLE_T, HANDLE_L, HANDLE_H), (HANDLE_X, HANDLE_L / 2, 0.0)),
         _box_gap(r, (-ESC_X0 + 0.010, 2 * ESC_R, 2 * ESC_R), ((ESC_X0 - 0.010) / 2, 0.0, 0.0)),
         _box_gap(r, (NUT_X1 - NUT_X0, NUT_AF * 1.16, NUT_AF * 1.16), ((NUT_X0 + NUT_X1) / 2, 0.0, 0.0))]
    return float(np.min(np.minimum.reduce(g)))


def now(env) -> dict:
    s = m.state(env, "tank")
    nut, handle = s.deg("nut"), START_DEG + s.deg("bushing")
    gap = hand_gap(s, m.fingertips(env))
    return {"nut": nut, "handle": handle, "gap": gap, "level": abs(handle) <= LEVEL_TOL, "seated": nut >= SEAT_DEG - 3.0,
            "clear": gap >= CLEAR}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place a recorded harm is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.over = 0
        self.overtight = False
        self.off = False
        self.last = {}

    def step(self, env):
        c = now(env)
        self.last = c
        self.over = self.over + 1 if c["nut"] > SEAT_DEG + OVER_DEG else 0
        if self.over >= 3:
            self.overtight = True
        if c["nut"] <= OFF_DEG:
            self.off = True
        harm = self.overtight or self.off
        ok = c["seated"] and c["level"] and c["clear"] and not harm
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if harm:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["seated"] and c["clear"]:  # firm but not level (or level for less than 2 s)
            self.score = 0.5
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration: a fixed plan from the published starting layout -------------------------------------------------------------------

NUT_TCP_X = 0.030  # the left hand's TCP, in the tank's frame: 3 cm in from the front face (its pads then cover the nut's back 14 mm)
H_TCP = 0.005  # the TCP above the nut's axis: the pads' tips about 9 mm below it
PITCH = -30.0  # the left grip leans its fingertips 30 degrees towards the front wall, keeping the fingers' wide upper part off it
GRIP_Y = 0.07  # the right hand's grip, along the handle from its hub
HANDLE_DZ = 0.0  # the right hand's TCP above the handle's centre line
NUT_OPEN = -0.024
HANDLE_OPEN = -0.014


def _rx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _nut_grip(deg):
    """Fingers closing along world x, across two of the nut's flats, turned deg about the hole's axis (world +y: tightening)."""
    return d._as_grip(_ry(deg) @ _rx(PITCH) @ d._plain_grip(-math.pi / 2))


def _nut_tcp(deg, up=0.0):
    p = to_world((NUT_TCP_X, 0.0, 0.0)) + _ry(deg) @ np.array([0.0, 0.0, H_TCP])
    return tuple(p + np.array([0.0, 0.0, up]))


def _handle_tcp(deg, lift=0.0):
    a = math.radians(deg)
    return tuple(to_world((HANDLE_X, GRIP_Y * math.cos(a), GRIP_Y * math.sin(a) + HANDLE_DZ + lift)))


def _turn(a0, a1, finger, t, label, steps=4):
    return [Waypoint(t / steps, _nut_tcp(a0 + (a1 - a0) * (k + 1) / steps), finger, label, _nut_grip(a0 + (a1 - a0) * (k + 1) / steps))
            for k in range(steps)]


def _plan(mode="full"):
    """Right: takes the handle near its end from above (fingers across its thickness), lifts it 15 degrees to level about the hole's
    axis and holds it there. Left: comes down inside the front wall either side of the locknut, takes it across two flats once the
    handle is level, and turns it counterclockwise (seen from inside the tank) 60 degrees, regrips, and 45 degrees more (105
    commanded; about 90 on the nut after slip in the grip, snug on the spacer). Then it lets go and leaves; the right hand lets go after it.
    Modes: none does nothing; onearm turns the nut the same way without holding the handle; careless keeps turning: 60, 60 and 40 degrees."""
    rest_l, rest_r = (0.25, 0.25, TOP + 0.20), (0.25, -0.25, TOP + 0.20)
    if mode == "none":
        return {"left": [Waypoint(30.0, rest_l, FINGER_OPEN, "left: rest")], "right": [Waypoint(30.0, rest_r, FINGER_OPEN, "right: rest")]}
    t_level = 2.5 + 2.0 + 0.8 + 4 * 0.5 + 0.5  # when the right hand has the handle level
    left = [
        Waypoint(2.5, _nut_tcp(0.0, 0.10), NUT_OPEN, "left: over the nut", _nut_grip(0.0)),
        Waypoint(2.0, _nut_tcp(0.0), NUT_OPEN, "left: down either side of the nut", _nut_grip(0.0)),
        Waypoint(t_level - 4.5, _nut_tcp(0.0), NUT_OPEN, "left: wait, open, while the handle is lifted", _nut_grip(0.0)),
        Waypoint(0.8, _nut_tcp(0.0), FINGER_CLOSED, "left: grip the nut's flats", _nut_grip(0.0)),
    ]
    strokes = [60.0, 60.0, 40.0] if mode == "careless" else [60.0, 45.0]
    for k, a in enumerate(strokes):
        left += _turn(0.0, a, FINGER_CLOSED, 2.5 * a / 60.0, f"left: turn the nut, stroke {k + 1}")
        left += [Waypoint(0.6, _nut_tcp(a), NUT_OPEN, "left: let go", _nut_grip(a))]
        if k < len(strokes) - 1:
            left += _turn(a, 0.0, NUT_OPEN, 2.0 * a / 60.0, "left: back round, open")
            left += [Waypoint(0.6, _nut_tcp(0.0), FINGER_CLOSED, "left: grip again", _nut_grip(0.0))]
    a = strokes[-1]
    left += [
        Waypoint(1.0, _nut_tcp(a, 0.10), NUT_OPEN, "left: up out of the tank", _nut_grip(a)),
        Waypoint(1.5, rest_l, NUT_OPEN, "left: away", d.down(-90)),
        Waypoint(20.0, rest_l, NUT_OPEN, "left: wait", d.down(-90)),
    ]
    t_left = sum(w.duration for w in left[:-3]) + 1.0  # when the left hand is up out of the tank
    G = d.down(0.0)
    if mode == "onearm":
        right = [Waypoint(40.0, rest_r, FINGER_OPEN, "right: rest")]
    else:
        right = [
            Waypoint(2.5, _handle_tcp(START_DEG, 0.06), HANDLE_OPEN, "right: over the handle's end", G),
            Waypoint(2.0, _handle_tcp(START_DEG), HANDLE_OPEN, "right: fingers either side of the handle", G),
            Waypoint(0.8, _handle_tcp(START_DEG), FINGER_CLOSED, "right: grip the handle", G),
        ]
        right += [Waypoint(0.5, _handle_tcp(START_DEG * (1 - (k + 1) / 4)), FINGER_CLOSED, "right: lift the handle level", G) for k in range(4)]
        t_r = sum(w.duration for w in right)
        right += [
            Waypoint(t_left - t_r, _handle_tcp(0.0), FINGER_CLOSED, "right: hold the handle level", G),
            Waypoint(0.6, _handle_tcp(0.0), HANDLE_OPEN, "right: let go", G),
            Waypoint(1.0, _handle_tcp(0.0, 0.06), HANDLE_OPEN, "right: up", G),
            Waypoint(1.5, rest_r, FINGER_OPEN, "right: away"),
            Waypoint(20.0, rest_r, FINGER_OPEN, "right: wait"),
        ]
    return {"left": left, "right": right}


def demo():
    return _plan("full")
