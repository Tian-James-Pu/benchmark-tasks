"""Seat a motor-driver HAT on a Raspberry Pi 4's 40-pin GPIO header, on a robot's top deck.

On the table: a Raspberry Pi 4 Model B bolted to a robot's aluminium top deck, its four 11 mm brass hex standoffs already screwed in
at its mounting holes, as the maker's add-on guides have them fitted first. Beside it, in a black anti-static kitting tray, the
add-on board: an Adafruit DC & Stepper Motor HAT (65 x 56.5 mm), its 2 x 20 female socket header soldered underneath. The board has
to go on the Pi's header with every pin in its own socket (pin 1 in socket 1), pressed fully home onto the standoffs, and be let go.

Model (declared in code):
- The Pi, its header's plastic, its standoffs, ports and the deck are fixed. The header's 40 pins are drawn; their tips collide as one
  plate ("pins": a link that stays at the tips until a board is on its pins).
- The HAT is a rigid body moved by six joints in a chain (three slides, three turns: "hat"), which stand in for free motion: free,
  with gravity, everywhere the arms can take it. Its socket header collides as one block.
- Mating (code, each control step), from the board's pose relative to the pins:
  - The sockets take the pins when the socket strip sits on the pin tips with every pin within 0.4 mm of a socket's centre (the pins'
    and sockets' lead-in chamfers). Any whole-pitch offset that keeps pins in sockets takes them too (a board one pin off): that is
    not registered and scores 0.
  - Once on its pins the board cannot slide or turn in the plane (the pins hold it): those three joints are held. The tip plate then
    rides down under the sockets to the header's plastic.
  - Pushing it on or pulling it off: Coulomb friction of the 40 contacts, F_IN for insertion and F_OUT for withdrawal, built up over
    the first 1 mm of the lead-in (a stick-slip drive on the vertical slide, re-anchored each step).
  - Rocking: the pins in their sockets resist it with K_ROCK.
- Damage, latched and cleared only at a reset: a pin bends when, more than 1 mm in, the deepest pin's tip is forced more than 0.15 mm
  off its socket's axis (depth x the board's lean), or when the board is pushed onto pins it is not aligned with by more than 15 N.

    pipeline-free:  inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Seat the motor HAT fully on the Raspberry Pi's GPIO header."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]
MM = 0.001

# -- layout (world, m). The Pi's header centre: its long axis along x, the header on the Pi's +y edge.
HX, HY = 0.270, -0.060
DECK_T = 3.0 * MM  # the robot's aluminium top deck
SPACER = 5.0 * MM  # nylon spacers under the Pi
PI_T = 1.4 * MM
PI_TOP = TOP + DECK_T + SPACER + PI_T  # the Pi's board top
PLASTIC = 2.5 * MM  # the header's plastic base
PIN_UP = 6.0 * MM  # pin length above the plastic (header 8.5 mm over the board, Raspberry Pi 4 mechanical drawing)
TIP_Z = PI_TOP + PLASTIC + PIN_UP
STANDOFF_H = 11.0 * MM  # board to board: the 8.51 mm socket body on the 2.5 mm plastic
SEAT_Z = PI_TOP + STANDOFF_H  # the HAT's underside, seated
PITCH = 2.54 * MM
# the HAT's socket strip (Samtec SSQ-120-..-D geometry): 51.31 x 4.95 x 8.51 mm
SOCK_L, SOCK_W, SOCK_H = 51.31 * MM, 4.95 * MM, 8.51 * MM
# the HAT (Raspberry Pi HAT mechanical spec, Adafruit board file): 65 x 56.5 x 1.6 mm; header centre 3.5 mm in from its long edge
HAT_L, HAT_W, HAT_T = 65.0 * MM, 56.5 * MM, 1.6 * MM
HAT_Y0, HAT_Y1 = -53.0 * MM, 3.5 * MM  # board frame: from the header's centre line, the far (terminal) edge and the near edge
HOLES = [(-29.0 * MM, 0.0), (29.0 * MM, 0.0), (-29.0 * MM, -49.0 * MM), (29.0 * MM, -49.0 * MM)]
HAT_MASS = 0.025
# the tray: the HAT starts in it at the same x, its header line TRAY_DY to the left of the Pi's
TRAY_DY = 0.11
RAIL_H, TRAY_BASE = 10.0 * MM, 2.0 * MM
START_Z = TOP + TRAY_BASE + RAIL_H  # the HAT's underside in the tray
HAT0 = np.array([HX, HY + TRAY_DY, START_Z])  # the HAT part's origin: its header's centre on its underside, at the start
# joint values (slides, from the start) that put the board's sockets square over the pins
ALIGN = np.array([HX, HY, 0.0]) - np.array([HAT0[0], HAT0[1], 0.0])
Z_ON_TIPS = TIP_Z + SOCK_H - START_Z  # tz with the socket strip's underside at the pin tips
Z_SEATED = SEAT_Z - START_Z

# -- mating (estimates and catalogue values: README "The model")
N_PINS = 40
F_IN = 1.67 * N_PINS  # N: Samtec SSQ standard contact, 1.67 N average insertion force per contact
F_OUT = 1.39 * N_PINS  # N: its withdrawal force, 1.39 N average per contact
LEAD = 1.0 * MM  # the insertion force builds up over the lead-in
CAPTURE = 0.4 * MM  # a pin finds its socket within this of the socket's centre
K_Z = 2.0e5  # N/m: the stick spring of the friction model (0.18 mm of give at F_IN)
K_ROCK = 15.0  # N m/rad: the pins' resistance to rocking the board on them
MISFIT = 0.15 * MM  # a pin tip forced this far off its socket's axis bends the pin
DEEP = 1.0 * MM
F_CRUSH = 15.0  # N: pushing a misaligned board onto the pin tips harder than this bends them
K_TIPS = 2.0e5
MATED = 3.68 * MM  # the socket's rated minimum insertion depth (Samtec SSQ: 3.68 to 6.35 mm)
SEATED_GAP = 0.3 * MM  # a 0.3 mm feeler gauge does not enter between the HAT and any standoff top
CLEAR = 0.03  # every fingertip point this far from the board to count as let go

LOOKS = {
    "pcb_green": m.Look(color=m.rgb(0.05, 0.36, 0.16), roughness=0.5),
    "pcb_blue": m.Look(color=m.rgb(0.10, 0.18, 0.55), roughness=0.5),
    "black": m.Look(color=m.rgb(0.03, 0.03, 0.035), roughness=0.55),
    "gold": m.Look(color=m.rgb(0.85, 0.68, 0.28), metallic=1.0, roughness=0.3),
    "brass": m.Look(color=m.rgb(0.78, 0.62, 0.30), metallic=0.9, roughness=0.35),
    "steel": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=0.9, roughness=0.3),
    "alu": m.Look(color=m.rgb(0.62, 0.63, 0.66), metallic=0.8, roughness=0.45),
    "white": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.6),
    "chip": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.4),
    "term": m.Look(color=m.rgb(0.10, 0.55, 0.25), roughness=0.55),
    "cap": m.Look(color=m.rgb(0.12, 0.15, 0.35), roughness=0.4),
    "tray": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.8),
    "silk": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.6),
}


def _rounded(l, w, t, r, center):
    """A board outline with rounded corners (radius r), as a prism."""
    pts = []
    for cx, cy, a0 in ((l / 2 - r, w / 2 - r, 0), (-l / 2 + r, w / 2 - r, 90), (-l / 2 + r, -w / 2 + r, 180), (l / 2 - r, -w / 2 + r, 270)):
        for k in range(5):
            a = math.radians(a0 + 22.5 * k)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return m.prism(pts, -t / 2, t / 2).moved(center)


# -- the Pi on the robot's deck (fixed) ---------------------------------------------------------------------------------------------------
# Pi frame = world, with the origin at the header's centre on the Pi's board top. Raspberry Pi 4 Model B drawing: 85 x 56 mm, holes 3.5 mm
# in from the edges on a 58 x 49 mm grid, header centred 29 mm from the first hole, 3.5 mm in from the edge.
PI_X0, PI_X1, PI_Y0, PI_Y1 = -32.5 * MM, 52.5 * MM, -52.5 * MM, 3.5 * MM


def _pi():
    parts, cols = [], []
    z0 = -(PI_TOP - TOP)  # the table top in this frame
    deck = ((0.13, 0.095, DECK_T), (0.010, -0.024, z0 + DECK_T / 2))
    parts.append(("deck", m.box(*deck), "alu"))
    cols.append(m.col_box("deck_c", *deck))
    pcb_c = ((PI_X0 + PI_X1) / 2, (PI_Y0 + PI_Y1) / 2, -PI_T / 2)
    parts.append(("pcb", _rounded(PI_X1 - PI_X0, PI_Y1 - PI_Y0, PI_T, 3.0 * MM, pcb_c), "pcb_green"))
    cols.append(m.col_box("pcb_c", (PI_X1 - PI_X0, PI_Y1 - PI_Y0, PI_T), pcb_c))
    for k, (hx, hy) in enumerate(HOLES):
        sp = m.cylinder(2.75 * MM, SPACER, (hx, hy, -PI_T - SPACER / 2), segments=6)
        parts.append((f"spacer{k}", sp, "white"))
        so = m.cylinder(2.9 * MM, STANDOFF_H, (hx, hy, STANDOFF_H / 2), segments=6)  # 5 mm across flats
        parts.append((f"standoff{k}", so, "brass"))
        cols.append(m.col_cylinder(f"standoff{k}_c", 2.6 * MM, STANDOFF_H, (hx, hy, STANDOFF_H / 2)))
    # the header's plastic and its 40 gold pins (seen only; the tips collide through the "pins" link)
    parts.append(("hdr_plastic", m.box((50.8 * MM, 5.08 * MM, PLASTIC), (0, 0, PLASTIC / 2)), "black"))
    pins = [m.box((0.64 * MM, 0.64 * MM, PIN_UP + PLASTIC), ((i - 9.5) * PITCH, (j - 0.5) * PITCH, (PIN_UP + PLASTIC) / 2)) for i in range(20) for j in range(2)]
    parts.append(("hdr_pins", m.join(*pins), "gold"))
    # ports at the far end (Raspberry Pi 4 drawing: Ethernet 13.5 mm and two USB stacks 16 mm tall, overhanging the edge by 2 mm)
    for name, (cy, w, h) in {"eth": (-6.75 * MM, 16.0 * MM, 13.5 * MM), "usb1": (-25.5 * MM, 13.1 * MM, 16.0 * MM), "usb2": (-43.5 * MM, 13.1 * MM, 16.0 * MM)}.items():
        b = ((21.0 * MM, w, h), (PI_X1 - 8.5 * MM, cy, h / 2))
        parts.append((name, m.box(*b), "steel"))
        cols.append(m.col_box(name + "_c", *b))
    # the SoC, RAM and USB controller (seen only)
    for name, (cx, cy, sx, sy, h) in {"soc": (-7.0, -26.75, 15.0, 15.0, 2.4), "ram": (5.0, -26.75, 15.0, 12.0, 1.2),
                                      "usbc": (20.0, -32.0, 6.0, 6.0, 1.0)}.items():
        parts.append((name, m.box((sx * MM, sy * MM, h * MM), (cx * MM, cy * MM, h * MM / 2)), "chip"))
    return parts, cols


_pp, _pc = _pi()
PI = m.fixed("pi", (HX, HY, PI_TOP), _pp, _pc, LOOKS, friction=(0.8, 0.6))

# the pin tips as one plate (48.9 x 3.2 mm, the 40 tips' envelope, plus 0.25 mm), on a slide that rides down under a board on its pins
TIPS_L, TIPS_W, TIPS_T = 49.4 * MM, 3.7 * MM, 2.0 * MM
_tips = m.slider("tips", "Z", -(PIN_UP + 0.01 * MM), 0.0, [], [m.col_box("tips_c", (TIPS_L, TIPS_W, TIPS_T), (0, 0, -TIPS_T / 2))],
                 mass=0.005, spring=K_TIPS, damping=20.0, friction=0.0, effort=500.0, armature=0.0005)
PINS = m.mechanism("pins", (HX, HY, TIP_Z), [_tips], LOOKS, friction=(0.6, 0.5))


# -- the tray (fixed) ---------------------------------------------------------------------------------------------------------------------
def _tray():
    parts, cols = [], []
    base = ((0.090, 0.085, TRAY_BASE), (0.0, -0.025, TRAY_BASE / 2))
    parts.append(("base", m.box(*base), "tray"))
    cols.append(m.col_box("base_c", *base))
    for s in (-1, 1):
        rail = ((6.0 * MM, 0.066, RAIL_H), (s * 29.5 * MM, -0.025, TRAY_BASE + RAIL_H / 2))
        parts.append((f"rail{s + 1}", m.box(*rail), "tray"))
        cols.append(m.col_box(f"rail{s + 1}_c", *rail))
    return parts, cols


_tp, _tc = _tray()
TRAY = m.fixed("tray", (HX, HY + TRAY_DY, TOP), _tp, _tc, LOOKS, friction=(0.8, 0.6))


# -- the HAT: a rigid board on six joints --------------------------------------------------------------------------------------------------
# Board frame: origin at the socket header's centre on the board's underside; x along the header, y across (+y to the near, header edge).
# Components from Adafruit's board file (Motor HAT rev A), turned into this frame (x - 32.5 mm, y - 53 mm).
TERMINALS = {"j1": (-16.498, 17.5), "j3": (2.171, 17.5), "mpower": (20.205, 7.0)}  # centre x (mm), length along x (mm); 3.5 mm pitch
TERM_Y, TERM_D, TERM_H = -50.587 * MM, 7.0 * MM, 10.0 * MM
CAPS = {"c4": (7.0, -40.3), "c5": (-9.64, -40.17)}
CAP_R, CAP_H = 3.15 * MM, 5.4 * MM


def _hat_parts():
    parts, cols = [], []
    pcb_c = (0.0, (HAT_Y0 + HAT_Y1) / 2, HAT_T / 2)
    parts.append(("pcb", _rounded(HAT_L, HAT_W, HAT_T, 3.0 * MM, pcb_c), "pcb_blue"))
    cols.append(m.col_box("pcb_c", (HAT_L, HAT_W, HAT_T), pcb_c))
    sock = ((SOCK_L, SOCK_W, SOCK_H), (0.0, 0.0, -SOCK_H / 2))
    parts.append(("socket", m.box(*sock), "black"))
    cols.append(m.col_box("socket_c", *sock))
    tails = [m.box((0.6 * MM, 0.6 * MM, 1.0 * MM), ((i - 9.5) * PITCH, (j - 0.5) * PITCH, HAT_T + 0.5 * MM)) for i in range(20) for j in range(2)]
    parts.append(("tails", m.join(*tails), "steel"))  # the socket's solder joints on top
    for name, (cx, ln) in TERMINALS.items():
        b = ((ln * MM, TERM_D, TERM_H), (cx * MM, TERM_Y, HAT_T + TERM_H / 2))
        parts.append((name, m.box(*b), "term"))
        cols.append(m.col_box(name + "_c", *b))
    for name, (cx, cy) in CAPS.items():
        c = m.cylinder(CAP_R, CAP_H, (cx * MM, cy * MM, HAT_T + CAP_H / 2), segments=16)
        parts.append((name, c, "cap"))
        cols.append(m.col_cylinder(name + "_c", CAP_R, CAP_H, (cx * MM, cy * MM, HAT_T + CAP_H / 2)))
    for name, (cx, cy, sx, sy) in {"tb1": (17.5, -40.3, 7.8, 8.0), "tb2": (0.75, -40.2, 7.8, 8.0), "pca": (-7.1, -30.0, 9.7, 6.4)}.items():
        parts.append((name, m.box((sx * MM, sy * MM, 1.2 * MM), (cx * MM, cy * MM, HAT_T + 0.6 * MM)), "chip"))
    for k, (hx, hy) in enumerate(HOLES):
        parts.append((f"hole{k}", m.cylinder(1.375 * MM, 0.2 * MM, (hx, hy, HAT_T + 0.1 * MM), segments=12), "brass"))
    parts.append(("pin1", m.box((1.2 * MM, 1.2 * MM, 0.1 * MM), (-9.5 * PITCH, -0.5 * PITCH - 2.6 * MM, HAT_T + 0.05 * MM)), "silk"))
    return parts, cols


_hp, _hc = _hat_parts()
HAT_COM = (0.0, -24.0 * MM, 2.5 * MM)
_V = dict(mass=0.001, damping=0.002, friction=0.0, effort=500.0, armature=0.0, inertia=(1e-7, 1e-7, 1e-7))
_R = dict(mass=0.001, damping=1e-5, friction=0.0, effort=5.0, armature=0.0, inertia=(1e-7, 1e-7, 1e-7))
_HAT_LINKS = [
    m.slider("tx", "X", -0.10, 0.10, [], [], **_V),
    m.slider("ty", "Y", -0.25, 0.12, [], [], parent="tx", **_V),
    m.slider("tz", "Z", -0.010, 0.25, [], [], parent="ty", **_V),
    m.hinge("rz", "Z", (0.0, 0.0, 0.0), -179.0, 179.0, [], [], parent="tz", **_R),
    m.hinge("ry", "Y", (0.0, 0.0, 0.0), -75.0, 75.0, [], [], parent="rz", **_R),
    m.hinge("board", "X", (0.0, 0.0, 0.0), -75.0, 75.0, _hp, _hc, parent="ry", mass=HAT_MASS, damping=1e-5, friction=0.0, effort=5.0, armature=0.0,
            gravity=True, com=HAT_COM, inertia=(8.0e-6, 1.0e-5, 1.7e-5)),
]
HAT = m.mechanism("hat", tuple(HAT0 + np.array([0.0, 0.0, 0.0002])), _HAT_LINKS, LOOKS, friction=(1.0, 0.9), solver_iters=48)

# -- distractors (SimReady, loose): a PH1 screwdriver for the HAT's M2.5 screws, long-nose pliers, a roll of tape --------------------------
SCREWDRIVER = SimReady("screwdriver", pos=(0.43, 0.10, TOP), size=(0.147, 0.027, 0.026), mass=0.05, package="handTool_screwdriver_phillips_ph1_a01",
                       usd="sm_handTool_screwdriver_phillips_ph1_a01_01.usd", yaw=0.4)
PLIERS = SimReady("pliers", pos=(0.43, -0.20, TOP), size=(0.235, 0.069, 0.024), mass=0.12, package="handTool_pliers_long_nose_a01",
                  usd="sm_handTool_pliers_long_nose_a01_01.usd", yaw=-0.5)
TAPE = SimReady("tape", pos=(0.15, 0.20, TOP), size=(0.10, 0.10, 0.05), mass=0.12, package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd")

OBJECTS = [PI, PINS, TRAY, HAT, SCREWDRIVER, PLIERS, TAPE]


# -- the mating ---------------------------------------------------------------------------------------------------------------------------
def _np(x):
    return m._np(x)


def _drive(env, part, link, target, stiffness, effort=None, damping=None):
    asset, j = m._joint(env, part, link)
    m._TOUCHED.add((part, link))
    if effort is not None:
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
    if damping is not None:
        asset.write_joint_damping_to_sim(m._tensor(asset, damping), joint_ids=[j])
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


def board_pose(env):
    """(position, rotation matrix, linear velocity, angular velocity) of the board link, station frame."""
    base = env.unwrapped
    asset = base.scene["hat"]
    i = asset.find_bodies("board")[0][0]
    pose = _np(asset.data.body_link_pose_w)[0, i]
    vel = _np(asset.data.body_link_vel_w)[0, i]
    origin = _np(base.scene.env_origins[0])
    return pose[:3] - origin, m.quat_matrix(pose[3:7]), vel[:3], vel[3:6]


def lean(R) -> tuple[float, float]:
    """The board's lean (rad) about x (across the header) and about y (along it)."""
    return math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))


class Header:
    """The pins and sockets: which pins the board is on, the insertion friction, and the record of a bent pin (cleared only at a reset)."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.cap = None  # (i, j): the board's sockets on the pins, offset by i pitches along the header and j across; None: off them
        self.anchor = 0.0
        self.bent = False
        self.why = ""
        self.over = 0
        self.crush = 0
        self.depth = 0.0
        self.max_depth = 0.0
        self.misfit = 0.0
        self.tip_force = 0.0
        self.offset = (0.0, 0.0, 0.0)
        if env is not None:
            self._free(env)

    def _free(self, env):
        for link in ("tx", "ty", "tz"):
            _drive(env, "hat", link, 0.0, 0.0, effort=500.0, damping=_V["damping"])
        for link in ("rz", "ry", "board"):
            _drive(env, "hat", link, 0.0, 0.0, effort=5.0, damping=1e-5)
        _drive(env, "pins", "tips", 0.0, K_TIPS, effort=500.0, damping=20.0)

    def _capture(self, env, s, i, j, dx, dy):
        self.cap = (i, j)
        q = s.joints
        _drive(env, "hat", "tx", q["tx_joint"] - (dx - i * PITCH), 2.0e4, effort=500.0, damping=50.0)
        _drive(env, "hat", "ty", q["ty_joint"] - (dy - j * PITCH), 2.0e4, effort=500.0, damping=50.0)
        _drive(env, "hat", "rz", 0.0, 40.0, effort=50.0, damping=0.02)
        for link in ("ry", "board"):
            _drive(env, "hat", link, 0.0, K_ROCK, effort=50.0, damping=0.01)
        self.anchor = q["tz_joint"]
        _drive(env, "hat", "tz", self.anchor, K_Z, effort=0.01, damping=20.0)
        _drive(env, "pins", "tips", 0.0, 100.0, effort=0.2, damping=0.5)  # rides down under the sockets

    def step(self, env):
        s = m.state(env, "hat")
        p, R, v, w = board_pose(env)
        bottom = p + R @ np.array([0.0, 0.0, -SOCK_H])  # the socket strip's underside, at its centre
        dx, dy, dz = bottom[0] - HX, bottom[1] - HY, bottom[2] - TIP_Z
        yaw = math.atan2(R[1, 0], R[0, 0])
        ax, ay = lean(R)
        self.offset = (dx, dy, yaw)
        tips = m.state(env, "pins").joint("tips")
        if self.cap is None:
            self.depth = 0.0
            # pushed onto the tips: the tip plate's give under its stiff spring
            self.tip_force = max(0.0, -tips) * K_TIPS
            self.crush = self.crush + 1 if self.tip_force > F_CRUSH else 0
            if self.crush >= 2 and not self.bent:
                self.bent, self.why = True, f"pushed onto pins it was not on, {self.tip_force:.0f} N"
            if dz < 0.3 * MM and abs(ax) < math.radians(15) and abs(ay) < math.radians(15):
                for i in (0, -1, 1, -2, 2):
                    for j in (0, -1, 1):
                        ex = abs(dx - i * PITCH) + abs(yaw) * 1.27 * MM
                        ey = abs(dy - j * PITCH) + abs(yaw) * 9.5 * PITCH
                        if ex <= CAPTURE and ey <= CAPTURE:
                            self._capture(env, s, i, j, dx, dy)
                            break
                    if self.cap is not None:
                        break
            return
        # on the pins
        self.tip_force = 0.0
        d = -dz
        if dz > 0.3 * MM:  # drawn off the pins
            self.cap = None
            self._free(env)
            return
        self.depth = max(0.0, d)
        self.max_depth = max(self.max_depth, self.depth)
        f_in = F_IN * min(1.0, self.depth / LEAD)
        f_out = F_OUT * min(1.0, self.depth / LEAD)
        z = s.joint("tz")
        give = self.anchor - z  # > 0: pushed down past the anchor
        if give > f_in / K_Z:
            self.anchor = z + f_in / K_Z
        elif -give > f_out / K_Z:
            self.anchor = z - f_out / K_Z
        effort = max(f_in if self.anchor >= z else f_out, 0.01)
        _drive(env, "hat", "tz", self.anchor, K_Z, effort=effort)
        # a bent pin: the deepest tip forced off its socket's axis by the lean
        deepest = self.depth + 9.5 * PITCH * abs(math.sin(ay)) + 0.5 * PITCH * abs(math.sin(ax))
        self.misfit = deepest * max(abs(math.tan(ax)), abs(math.tan(ay)))
        self.over = self.over + 1 if (deepest > DEEP and self.misfit > MISFIT) else 0
        if self.over >= 2 and not self.bent:
            self.bent, self.why = True, f"leaned {math.degrees(ax):.1f}/{math.degrees(ay):.1f} deg at {deepest * 1000:.1f} mm"


HEADER = Header()
m.on_step(HEADER.step)
m.on_reset(HEADER.reset)


# -- the scorer ---------------------------------------------------------------------------------------------------------------------------
_BOX_LO = np.array([-HAT_L / 2, HAT_Y0 - 1.1 * MM, -SOCK_H])
_BOX_HI = np.array([HAT_L / 2, HAT_Y1, HAT_T + TERM_H])


def gaps(p, R) -> list[float]:
    """The gap (m) between the HAT's underside and each standoff's top."""
    return [float((p + R @ np.array([hx, hy, 0.0]))[2] - SEAT_Z) for hx, hy in HOLES]


def hands_gap(p, R, tips) -> float:
    q = (tips - p) @ R  # into the board's frame
    d = np.maximum(np.maximum(_BOX_LO - q, q - _BOX_HI), 0.0)
    return float(np.min(np.linalg.norm(d, axis=1)))


def now(env) -> dict:
    p, R, v, w = board_pose(env)
    g = gaps(p, R)
    reg = HEADER.cap == (0, 0)
    return {"registered": reg, "on": HEADER.cap, "depth": HEADER.depth, "gaps": g, "seated": reg and max(g) <= SEATED_GAP,
            "mated": reg and HEADER.depth >= MATED, "started": reg and HEADER.depth >= DEEP,
            "hands": hands_gap(p, R, m.fingertips(env)), "still": float(np.linalg.norm(v)) < 0.003 and float(np.linalg.norm(w)) < math.radians(3.0)}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.k = 0

    def step(self, env):
        self.k += 1
        c = now(env)
        ok = c["seated"] and c["hands"] >= CLEAR and c["still"] and not HEADER.bent
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if HEADER.bent:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["seated"]:
            self.score = 0.75
        elif c["mated"]:
            self.score = 0.5
        elif c["started"]:
            self.score = 0.25
        else:
            self.score = 0.0
        if os.environ.get("HAT_LOG"):
            with open(os.environ["HAT_LOG"], "a") as f:
                p, R, v, w = board_pose(env)
                ax, ay = lean(R)
                f.write(f"{self.k} cap={HEADER.cap} off=({HEADER.offset[0] * 1000:.2f},{HEADER.offset[1] * 1000:.2f})mm yaw={math.degrees(HEADER.offset[2]):.2f} "
                        f"lean=({math.degrees(ax):.2f},{math.degrees(ay):.2f}) depth={HEADER.depth * 1000:.2f} gaps={[round(x * 1000, 2) for x in c['gaps']]} "
                        f"misfit={HEADER.misfit * 1000:.3f} tipF={HEADER.tip_force:.1f} hands={c['hands'] * 1000:.0f} still={c['still']} "
                        f"bent={HEADER.bent} {HEADER.why} score={self.score} p={np.round(p, 4).tolist()} "
                        f"fR={np.round(m.fingertips(env)[12:18].mean(0), 4).tolist()} {np.round(m.fingertips(env)[18:24].mean(0), 4).tolist()}\n")


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published starting layout) ---------------------------------------------------------------------
FINGER_END = 0.008  # the fingers' ends below the TCP (measured: the reported tip point is 5 mm past the real end)
GRIP_Y = -24.75 * MM  # the grip's centre across the board (between its two long edges)


def demo():
    gx = HX
    gy0 = HY + TRAY_DY + GRIP_Y
    gy1 = HY + GRIP_Y
    z_grip = START_Z + 0.003  # finger ends 5 mm below the board's underside: the pads over both long edges
    lift = TOP + 0.09
    on_tips = TIP_Z + SOCK_H + 0.003 - 0.0005
    w = [
        Waypoint(2.5, (gx, gy0, lift), FINGER_OPEN, "over the HAT"),
        Waypoint(2.0, (gx, gy0, z_grip), FINGER_OPEN, "fingers either side of the board"),
        Waypoint(1.0, (gx, gy0, z_grip), FINGER_CLOSED, "grip its long edges"),
        Waypoint(1.5, (gx, gy0, lift), FINGER_CLOSED, "lift"),
        Waypoint(2.5, (gx, gy1, lift), FINGER_CLOSED, "over the header"),
        Waypoint(2.0, (gx, gy1, on_tips + 0.004), FINGER_CLOSED, "down near the tips"),
        Waypoint(1.5, (gx, gy1, on_tips), FINGER_CLOSED, "sockets onto the tips"),
        Waypoint(0.8, (gx, gy1, on_tips), FINGER_OPEN, "let go"),
        Waypoint(1.2, (gx, gy1, lift), FINGER_OPEN, "up"),
    ]
    press = SEAT_Z + HAT_T + FINGER_END
    w += [
        Waypoint(1.0, (HX, HY - 2.0 * MM, lift), FINGER_CLOSED, "over the header line"),
        Waypoint(1.5, (HX, HY - 2.0 * MM, press + 0.012), FINGER_CLOSED, "down to the board"),
        Waypoint(3.0, (HX, HY - 2.0 * MM, press - 0.115), FINGER_CLOSED, "press"),
        Waypoint(1.5, (HX, HY - 2.0 * MM, press - 0.115), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (HX, HY - 2.0 * MM, press + 0.06), FINGER_CLOSED, "up"),
        Waypoint(1.5, (0.20, -0.22, TOP + 0.20), FINGER_OPEN, "away"),
    ]
    return {"right": w}
