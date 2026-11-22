"""Play one level of a platform game with a gamepad.

A 24-inch monitor at the back of the desk shows a side-on platform game: a character at the left, a narrow gap, a tall step, a wide gap, and
a flag at the right. An Xbox Wireless Controller (model 1914, real size and mass) lies face up in a cradle taped to the desk between the
arms, its bumpers toward the monitor. The left analog stick runs the character (a 2-DOF gimbal, spring-centred, 21 degrees each way; the
game reads its left-right tilt, with a 20 % deadzone, and full speed from 90 % of the throw). The A button jumps (a sprung cap, 1.5 mm of
travel, closing at 1.0 mm): only a fresh press starts a jump, and the jump keeps rising for as long as A is held (a variable-height jump, as
in most platform games). So the step needs A held, the wide gap needs A held at full speed, and A must be let go between jumps. Three lives.

The game is a pure-Python model stepped once per control step (20 Hz) from the stick's and the button's joints, and drawn on the monitor's
screen as flat visual-only shapes moved by their USD transforms. `game.html`, next to this file, runs the same level with the same constants
in a browser with a real controller.
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
INSTRUCTION = "Use the game controller to get the character to the flag."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]

# ======================================================================================================================================
# The game. Every game constant is here, and game.html mirrors them one for one. Screen pixels, x right, y up from the picture's bottom
# left; one game step per 20 Hz control step; the inputs are read once per step.
# ======================================================================================================================================
GAME = {
    "HZ": 20,  # fixed steps per second
    "W": 1920, "H": 1080,  # the picture
    "CHAR_W": 72, "CHAR_H": 96,  # the character's box
    "START_X": 130, "START_Y": 200,  # the character's bottom centre at the start and after a fall
    "V_RUN": 450.0,  # px/s at full stick
    "ACCEL": 1800.0, "DECEL": 2400.0,  # px/s^2: towards the stick's speed; and to a stop with the stick in its deadzone
    "JUMP_V": 1050.0,  # px/s up, on a fresh press of A while standing
    "GRAVITY": 2400.0,  # px/s^2
    "JUMP_CUT": 150.0,  # px/s: letting go of A while rising cuts the upward speed to this (the variable-height jump)
    "FALL_MAX": 1500.0,  # px/s
    "DEADZONE": 0.20, "SATURATION": 0.90,  # stick axis value: no motion below 0.20; full speed from 0.90; linear between
    "LIVES": 3,
    "RESPAWN_STEPS": 10,  # 0.5 s after a fall, the character is back at the start
    # solid blocks: name, left x, right x, top y (each runs down past the bottom of the picture; the gaps between them are pits)
    "BLOCKS": [["A", 0, 520, 200], ["B", 610, 1400, 200], ["S", 820, 1000, 350], ["C", 1700, 1920, 200]],
    "FLAG_X": 1830,  # touching the pole clears the level
}
MILESTONE = {"B": 1, "S": 2, "C": 3}  # standing on: the ground between the gaps, the step's top, the ground past the wide gap; the flag is 4
STICK_MAX_DEG = 21.0  # the stick's throw each way: axis value = left-right tilt / 21 degrees (+ right)
A_DOWN_MM, A_UP_MM = 1.0, 0.7  # the A switch closes at 1.0 mm of travel and opens again above 0.7 mm


def stick_axis(tilt_deg: float) -> float:
    """The gamepad's axes[0] for a left-right tilt in degrees (+ right)."""
    return float(np.clip(tilt_deg / STICK_MAX_DEG, -1.0, 1.0))


def speed_fraction(axis: float) -> float:
    g = GAME
    return float(np.clip((abs(axis) - g["DEADZONE"]) / (g["SATURATION"] - g["DEADZONE"]), 0.0, 1.0))


class Game:
    """The level's state and its fixed-step update: exactly what game.html does in a browser."""

    def __init__(self):
        self.reset()

    def reset(self):
        g = GAME
        self.x, self.y = float(g["START_X"]), float(g["START_Y"])
        self.vx = self.vy = 0.0
        self.ground, self.on, self.jumping = True, "A", False
        self.a_prev = False
        self.lives, self.best = g["LIVES"], 0
        self.clear = self.over = False
        self.wait = 0
        self.steps = 0
        self.jumps = 0

    def step(self, axis: float, a_down: bool) -> None:
        g = GAME
        dt = 1.0 / g["HZ"]
        pressed = a_down and not self.a_prev
        self.a_prev = a_down
        self.steps += 1
        if self.over:
            return
        if self.wait > 0:
            self.wait -= 1
            if self.wait == 0:
                self.x, self.y = float(g["START_X"]), float(g["START_Y"])
                self.vx = self.vy = 0.0
                self.ground, self.on, self.jumping = True, "A", False
            return
        f = speed_fraction(axis)
        target = math.copysign(f * g["V_RUN"], axis) if f > 0.0 else 0.0
        rate = (g["ACCEL"] if f > 0.0 else g["DECEL"]) * dt
        self.vx += float(np.clip(target - self.vx, -rate, rate))
        if pressed and self.ground:
            self.vy, self.ground, self.jumping = g["JUMP_V"], False, True
            self.jumps += 1
        if self.jumping and not a_down and self.vy > g["JUMP_CUT"]:
            self.vy = g["JUMP_CUT"]
        if self.vy <= 0.0:
            self.jumping = False
        self.vy = max(self.vy - g["GRAVITY"] * dt, -g["FALL_MAX"])
        hw = g["CHAR_W"] / 2
        # across, then up or down, each stopped by the blocks
        self.x += self.vx * dt
        for _, x0, x1, top in g["BLOCKS"]:
            if self.x + hw > x0 and self.x - hw < x1 and self.y < top:
                self.x = x0 - hw if self.vx > 0 else x1 + hw
                self.vx = 0.0
        self.x = float(np.clip(self.x, hw, g["W"] - hw))
        y0 = self.y
        self.y += self.vy * dt
        self.ground = False
        if self.vy <= 0.0:
            land = None
            for name, x0, x1, top in g["BLOCKS"]:
                if self.x + hw > x0 and self.x - hw < x1 and y0 >= top - 1e-6 and self.y < top and (land is None or top > land[1]):
                    land = (name, top)
            if land is not None:
                self.y, self.vy, self.ground, self.on = float(land[1]), 0.0, True, land[0]
        if self.ground:
            self.best = max(self.best, MILESTONE.get(self.on, 0))
        if self.x + hw >= g["FLAG_X"]:
            self.clear = self.over = True
            self.best = 4
        elif self.y + g["CHAR_H"] < 0.0:
            self.lives -= 1
            if self.lives <= 0:
                self.over = True
            else:
                self.wait = g["RESPAWN_STEPS"]


# ======================================================================================================================================
# The scene
# ======================================================================================================================================

LOOKS = {
    "shell": m.Look(color=m.rgb(0.09, 0.09, 0.10), roughness=0.55),
    "shell_dark": m.Look(color=m.rgb(0.04, 0.04, 0.045), roughness=0.6),
    "rubber": m.Look(color=m.rgb(0.06, 0.06, 0.065), roughness=0.9),
    "btn_a": m.Look(color=m.rgb(0.20, 0.62, 0.20), roughness=0.35),
    "btn_b": m.Look(color=m.rgb(0.80, 0.15, 0.13), roughness=0.35),
    "btn_x": m.Look(color=m.rgb(0.15, 0.35, 0.85), roughness=0.35),
    "btn_y": m.Look(color=m.rgb(0.90, 0.72, 0.10), roughness=0.35),
    "guide": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=0.4, roughness=0.3),
    "cradle": m.Look(color=m.rgb(0.55, 0.57, 0.60), roughness=0.7),
    # the game's picture: emissive, as a lit screen
    "sky": m.Look(color=m.rgb(0.42, 0.68, 0.95), roughness=0.4, emissive=m.rgb(0.36, 0.60, 0.88)),
    "cloud": m.Look(color=m.rgb(0.97, 0.97, 0.98), roughness=0.4, emissive=m.rgb(0.90, 0.90, 0.92)),
    "grass": m.Look(color=m.rgb(0.25, 0.70, 0.20), roughness=0.4, emissive=m.rgb(0.20, 0.62, 0.16)),
    "dirt": m.Look(color=m.rgb(0.55, 0.33, 0.15), roughness=0.4, emissive=m.rgb(0.48, 0.28, 0.12)),
    "brick": m.Look(color=m.rgb(0.72, 0.38, 0.18), roughness=0.4, emissive=m.rgb(0.64, 0.32, 0.14)),
    "mortar": m.Look(color=m.rgb(0.35, 0.18, 0.08), roughness=0.4, emissive=m.rgb(0.30, 0.15, 0.06)),
    "pole": m.Look(color=m.rgb(0.85, 0.85, 0.85), roughness=0.4, emissive=m.rgb(0.78, 0.78, 0.78)),
    "flag": m.Look(color=m.rgb(0.10, 0.80, 0.25), roughness=0.4, emissive=m.rgb(0.08, 0.72, 0.20)),
    "hero": m.Look(color=m.rgb(0.92, 0.12, 0.10), roughness=0.4, emissive=m.rgb(0.85, 0.08, 0.06)),
    "hero_legs": m.Look(color=m.rgb(0.12, 0.22, 0.75), roughness=0.4, emissive=m.rgb(0.08, 0.18, 0.66)),
    "face": m.Look(color=m.rgb(0.98, 0.80, 0.62), roughness=0.4, emissive=m.rgb(0.90, 0.72, 0.55)),
    "ink": m.Look(color=m.rgb(0.02, 0.02, 0.02), roughness=0.4),
    "heart": m.Look(color=m.rgb(0.95, 0.10, 0.25), roughness=0.4, emissive=m.rgb(0.88, 0.06, 0.20)),
    "pip": m.Look(color=m.rgb(0.25, 0.30, 0.40), roughness=0.4, emissive=m.rgb(0.18, 0.22, 0.30)),
    "pip_lit": m.Look(color=m.rgb(1.00, 0.84, 0.10), roughness=0.4, emissive=m.rgb(0.95, 0.78, 0.06)),
    "banner": m.Look(color=m.rgb(0.05, 0.05, 0.10), roughness=0.4, emissive=m.rgb(0.02, 0.02, 0.05)),
    "text_win": m.Look(color=m.rgb(1.00, 0.90, 0.20), roughness=0.4, emissive=m.rgb(0.95, 0.85, 0.15)),
    "text_lose": m.Look(color=m.rgb(1.00, 0.30, 0.25), roughness=0.4, emissive=m.rgb(0.95, 0.25, 0.20)),
}

# -- the controller: an Xbox Wireless Controller (model 1914), 153 x 104 x 61 mm, 287 g with two AA cells. Its frame: x toward the bumpers
#    (the monitor), y to the player's left, z up, origin under its middle on the cradle's floor
PAD0 = (0.25, 0.0)
FLOOR_T = 0.004  # the cradle's floor
PAD_MASS = 0.287
Z_FACE = 0.038  # the face around the sticks and buttons, above the controller's lowest point
LS = (0.012, 0.037)  # the left stick's centre
LS_PIVOT_Z = Z_FACE - 0.008
CAP_R, CAP_Z0, CAP_Z1 = 0.00775, Z_FACE + 0.005, Z_FACE + 0.011  # the stick's cap: 15.5 mm across, its top 11 mm above the face
CAP_C = (CAP_Z0 + CAP_Z1) / 2
R_CAP = CAP_C - LS_PIVOT_Z  # pivot to the cap's middle
STICK_K, STICK_C = 0.036, 0.004  # N m/rad (0.7 N at the cap's top at full throw), N m s/rad
ABXY = (0.012, -0.037)
BTN_R, BTN_PROUD, BTN_TRAVEL, BTN_K = 0.0048, 0.003, 0.0015, 400.0  # 9.6 mm caps, 3 mm proud, 1.5 mm travel, 0.4 N at 1 mm
BUTTONS = {"a": (ABXY[0] - 0.011, ABXY[1]), "b": (ABXY[0], ABXY[1] - 0.011), "x": (ABXY[0], ABXY[1] + 0.011), "y": (ABXY[0] + 0.011, ABXY[1])}
GRIP_A, GRIP_B, GRIP_R = (0.004, 0.0505, 0.018), (-0.040, 0.0595, 0.017), 0.017  # each grip: a rounded bar from A to B (y mirrored), below the face
PAD_BOX = (-0.058, 0.046, -0.0765, 0.0765, 0.0, CAP_Z1)  # the controller's outline in its frame, sticks included, for hands-off checks

# -- the monitor: SimReady computer_monitor_office_a01 (54.2 x 17.2 x 39.4 cm; screen 53.5 x 29.0 cm), facing the arms
MON_X = 0.49  # as near as the top camera still sees the whole screen
MON_FACE_X = MON_X - 0.0029  # its screen's surface
SCREEN_CZ = 0.2458  # the screen's middle above the table
PITCH = 0.290 / 1080  # metres per game pixel: the 1920 x 1080 picture fills the screen's height, 51.6 x 29.0 cm
MON_BOX = (MON_X - 0.086, MON_X + 0.086, -0.271, 0.271, 0.0, 0.394)  # its outline (world x, y, height), for "touched"


def _rounded(x0, x1, y0, y1, r, n=5):
    pts = []
    for cx, cy, a0 in ((x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180), (x1 - r, y0 + r, 270)):
        for a in np.radians(np.linspace(a0, a0 + 90, n)):
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _grip(side: float) -> m.Mesh:
    a = np.array([GRIP_A[0], side * GRIP_A[1], GRIP_A[2]])
    b = np.array([GRIP_B[0], side * GRIP_B[1], GRIP_B[2]])
    pts = [a + (b - a) * t for t in np.linspace(0.0, 1.0, 6)]
    return m.join(m.tube(pts, GRIP_R, sides=16), m.sphere(GRIP_R, b, rings=8, segments=16), m.sphere(GRIP_R, a, rings=8, segments=16))


def _cap_mesh(dish: bool = True) -> m.Mesh:
    """The stick's cap, about its middle: 15.5 mm across, its rounded side a band of a sphere (so that it keeps its width between two
    pads however the stick tilts), with a dished top."""
    h = (CAP_Z1 - CAP_Z0) / 2
    zs = np.linspace(-h, h, 7)
    prof = [(0.0, -h)] + [(float(math.sqrt(CAP_R ** 2 - z ** 2)), float(z)) for z in zs]
    prof += [(CAP_R * 0.72, h - 0.0010), (0.0, h - 0.0014)] if dish else [(0.0, h)]
    return m.lathe(prof, segments=28)


def _controller():
    sx = m.hinge("sx", "X", (LS[0], LS[1], LS_PIVOT_Z), -STICK_MAX_DEG, STICK_MAX_DEG, (), (), mass=0.002, spring=STICK_K, damping=STICK_C,
                 friction=0.0005, effort=1.0, armature=2e-4, inertia=(1e-7, 1e-7, 1e-7))
    shaft = m.cylinder(0.0035, CAP_Z0 - LS_PIVOT_Z + 0.001, (LS[0], LS[1], (LS_PIVOT_Z + CAP_Z0) / 2))
    dome = m.sphere(0.0085, (LS[0], LS[1], LS_PIVOT_Z), rings=8, segments=20)
    cap = _cap_mesh().moved((LS[0], LS[1], CAP_C))
    sy = m.hinge("sy", "Y", (LS[0], LS[1], LS_PIVOT_Z), -STICK_MAX_DEG, STICK_MAX_DEG,
                 [("dome", dome, "shell_dark"), ("shaft", shaft, "shell_dark"), ("cap", cap, "rubber")],
                 [m.col_hull("cap_c", _cap_mesh(dish=False).moved((LS[0], LS[1], CAP_C)))], mass=0.003, spring=STICK_K, damping=STICK_C, friction=0.0005, effort=1.0, armature=2e-4,
                 inertia=(3e-7, 3e-7, 1e-7), parent="sx")
    buttons = []
    for name, (bx, by) in BUTTONS.items():
        cap_b = m.cylinder(BTN_R, 0.004, (bx, by, Z_FACE + BTN_PROUD - 0.002), segments=24)
        buttons.append(m.slider(name, "Z", -BTN_TRAVEL, 0.0, [(f"{name}_cap", cap_b, f"btn_{name}")],
                                [m.col_cylinder(f"{name}_c", BTN_R, 0.004, (bx, by, Z_FACE + BTN_PROUD - 0.002))], at=(bx, by, Z_FACE),
                                spring=BTN_K, damping=0.5, friction=0.0, mass=0.002, armature=1e-5, effort=20.0, inertia=(1e-8, 1e-8, 1e-8)))
    body = m.prism(_rounded(-0.020, 0.044, -0.068, 0.068, 0.020), 0.014, Z_FACE)
    gl, gr = _grip(1.0), _grip(-1.0)
    fp = [("body", body, "shell"), ("grip_l", gl, "shell"), ("grip_r", gr, "shell")]
    for s, side in (("l", 1.0), ("r", -1.0)):
        fp.append((f"bumper_{s}", m.box((0.010, 0.040, 0.009), (0.041, side * 0.042, 0.031)), "shell_dark"))
        fp.append((f"trigger_{s}", m.box((0.016, 0.028, 0.016), (0.036, side * 0.044, 0.008)), "shell_dark"))
    fp += [("ls_well", m.cylinder(0.0115, 0.001, (LS[0], LS[1], Z_FACE + 0.0003)), "shell_dark"),
           ("rs_well", m.cylinder(0.0115, 0.001, (-0.012, -0.019, Z_FACE + 0.0003)), "shell_dark"),
           ("rs_shaft", m.cylinder(0.0035, 0.008, (-0.012, -0.019, Z_FACE + 0.003)), "shell_dark"),
           ("rs_cap", _cap_mesh().moved((-0.012, -0.019, CAP_C)), "rubber"),
           ("dpad_a", m.box((0.020, 0.0065, 0.003), (-0.012, 0.019, Z_FACE + 0.0012)), "shell_dark"),
           ("dpad_b", m.box((0.0065, 0.020, 0.003), (-0.012, 0.019, Z_FACE + 0.0012)), "shell_dark"),
           ("guide", m.cylinder(0.0075, 0.002, (0.031, 0.0, Z_FACE + 0.0008)), "guide"),
           ("view", m.cylinder(0.003, 0.002, (0.012, 0.011, Z_FACE + 0.0008)), "shell_dark"),
           ("menu", m.cylinder(0.003, 0.002, (0.012, -0.011, Z_FACE + 0.0008)), "shell_dark"),
           ("share", m.box((0.004, 0.006, 0.002), (0.002, 0.0, Z_FACE + 0.0008)), "shell_dark"),
           ("abxy_well", m.cylinder(0.019, 0.0006, (ABXY[0], ABXY[1], Z_FACE + 0.0002)), "shell_dark")]
    fc = [m.col_box("body_c", (0.064, 0.136, Z_FACE - 0.014), (0.012, 0.0, (0.014 + Z_FACE) / 2)),
          m.col_hull("grip_l_c", gl), m.col_hull("grip_r_c", gr),
          m.col_box("trigger_l_c", (0.016, 0.028, 0.016), (0.036, 0.044, 0.008)), m.col_box("trigger_r_c", (0.016, 0.028, 0.016), (0.036, -0.044, 0.008))]
    inertia = (PAD_MASS / 12 * (0.153 ** 2 + 0.04 ** 2), PAD_MASS / 12 * (0.104 ** 2 + 0.04 ** 2), PAD_MASS / 12 * (0.104 ** 2 + 0.153 ** 2))
    return m.mechanism("pad", (PAD0[0], PAD0[1], TOP + FLOOR_T), [sx, sy, *buttons], LOOKS, frame_parts=fp, frame_colliders=fc, friction=(0.9, 0.8),
                       solver_iters=32, free_mass=PAD_MASS, frame_inertia=inertia)


PAD = _controller()


def _cradle():
    """A grey plastic controller cradle (3D-printed), taped to the desk: a 4 mm floor and 14 mm rims with 3 mm of play round the controller."""
    x0, x1, y1 = -0.061, 0.047, 0.0795  # the rims' inner faces
    t, h = 0.004, 0.014
    pieces = {
        "floor": ((x1 - x0 + 2 * t, 2 * y1 + 2 * t, FLOOR_T), ((x0 + x1) / 2, 0.0, FLOOR_T / 2)),
        "rim_f": ((t, 2 * y1 + 2 * t, h), (x1 + t / 2, 0.0, FLOOR_T + h / 2)),
        "rim_l": ((x1 - x0 + 2 * t, t, h), ((x0 + x1) / 2, y1 + t / 2, FLOOR_T + h / 2)),
        "rim_r": ((x1 - x0 + 2 * t, t, h), ((x0 + x1) / 2, -y1 - t / 2, FLOOR_T + h / 2)),
        "rim_bl": ((t, 0.045, h), (x0 - t / 2, y1 - 0.0225, FLOOR_T + h / 2)),
        "rim_br": ((t, 0.045, h), (x0 - t / 2, -y1 + 0.0225, FLOOR_T + h / 2)),
    }
    parts, cols = [], []
    for n, (size, c) in pieces.items():
        parts.append((n, m.box(size, c), "cradle"))
        cols.append(m.col_box(f"{n}_c", size, c))
    return m.fixed("cradle", (PAD0[0], PAD0[1], TOP), parts, cols, LOOKS, friction=(0.9, 0.8))


CRADLE = _cradle()

MONITOR = SimReady("monitor", pos=(MON_X, 0.0, TOP), size=(0.172, 0.542, 0.394), mass=None, package="computer_monitor_office_a01",
                   usd="sm_computer_monitor_office_a01_01.usd", yaw=-math.pi / 2)


# -- the picture on the screen: a fixed part whose origin is the screen's middle; its meshes lie in the screen's plane, out of the face by d
R_AX = np.array([0.0, -1.0, 0.0])  # the picture's right, as seen from in front
U_AX = np.array([0.0, 0.0, 1.0])  # its up
N_AX = np.array([-1.0, 0.0, 0.0])  # out of its face, toward the viewer


def screen_point(px: float, py: float, d: float = 0.0) -> np.ndarray:
    """A point of the picture in the screen part's frame: pixels from the bottom left (y up), `d` metres out of the face."""
    return (px - GAME["W"] / 2) * PITCH * R_AX + (py - GAME["H"] / 2) * PITCH * U_AX + d * N_AX


def to_pixels(p) -> tuple[float, float]:
    q = np.asarray(p, float)
    return float(q @ R_AX / PITCH + GAME["W"] / 2), float(q @ U_AX / PITCH + GAME["H"] / 2)


def _flat(outline_px, d: float, at=(0.0, 0.0)) -> m.Mesh:
    """A flat shape on the face, from an outline in pixels about an anchor (x right, y up), 0.1 mm thick, `d` out of the face."""
    pts = np.asarray(outline_px, float) * PITCH
    pr = m.prism(pts, d, d + 0.0001)
    v = pr.v[:, :1] * R_AX + pr.v[:, 1:2] * U_AX + pr.v[:, 2:3] * N_AX
    return m.Mesh(v, pr.f).moved(screen_point(at[0], at[1]) if at != (0.0, 0.0) else np.zeros(3))


def _rect(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _static(outline_px, d: float) -> m.Mesh:
    """A flat shape that stays put, from an outline in the picture's own pixels (from its bottom left)."""
    return _flat(np.asarray(outline_px, float) - np.array([GAME["W"] / 2, GAME["H"] / 2]), d)


def _rects(rects, d):
    return m.join(*[_static(_rect(*r), d) for r in rects])


FONT = {
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "G": ["01111", "10000", "10000", "10011", "10001", "10001", "01111"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
}


def _text(s: str, cx: float, cy: float, cell: float, d: float) -> m.Mesh:
    width = (len(s) * 6 - 1) * cell
    x = cx - width / 2
    rects = []
    for ch in s:
        rows = FONT.get(ch)
        if rows:
            for r, row in enumerate(rows):
                for c, bit in enumerate(row):
                    if bit == "1":
                        rects.append((x + c * cell, x + (c + 1) * cell, cy + (3 - r) * cell, cy + (4 - r) * cell))
        x += 6 * cell
    return _rects(rects, d)


def _heart():
    t = np.linspace(0.0, 2 * np.pi, 40, endpoint=False)
    return np.c_[16 * np.sin(t) ** 3, 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)][::-1] * 1.9


D_SKY, D_CLOUD, D_DIRT, D_TOP, D_FLAG, D_HERO, D_HERO2, D_HERO3, D_HUD, D_HUD2, D_BANNER, D_TEXT = (
    0.0008, 0.0010, 0.0012, 0.0014, 0.0014, 0.0018, 0.0020, 0.0022, 0.0016, 0.0018, 0.0026, 0.0028)
HERO = [("hero_legs", _rect(-30, 30, 0, 34), D_HERO, "hero_legs"), ("hero_body", _rect(-36, 36, 30, 66), D_HERO2, "hero"),
        ("hero_head", _rect(-26, 26, 62, 96), D_HERO, "face"), ("hero_eye", _rect(6, 16, 74, 88), D_HERO3, "ink")]
HEARTS = [(70 + 80 * k, 1010) for k in range(GAME["LIVES"])]
PIPS = [(1560 + 90 * k, 1010) for k in range(4)]


def _screen():
    g = GAME
    W, H = g["W"], g["H"]
    parts = [("panel_black", _rects([(-45, W + 45, -12, H + 12)], D_SKY - 0.0003), "ink"), ("sky", _rects([(0, W, 0, H)], D_SKY), "sky"),
             ("clouds", m.join(_flat(_rounded(-90, 90, -28, 28, 26), D_CLOUD, (420, 820)), _flat(_rounded(-60, 60, -22, 22, 20), D_CLOUD, (500, 860)),
                               _flat(_rounded(-110, 110, -30, 30, 28), D_CLOUD, (1250, 760)), _flat(_rounded(-70, 70, -24, 24, 22), D_CLOUD, (1340, 800))), "cloud")]
    dirt, grass, bricks, mortar = [], [], [], []
    for name, x0, x1, top in g["BLOCKS"]:
        if name == "S":
            bricks.append((x0, x1, g["BLOCKS"][1][3], top))
            for k, yy in enumerate(range(g["BLOCKS"][1][3] + 50, top, 50)):
                mortar.append((x0, x1, yy - 3, yy + 3))
            for yy in range(g["BLOCKS"][1][3], top, 50):
                off = 0 if (yy // 50) % 2 else 45
                for xx in range(x0 + off + 45, x1 - 5, 90):
                    mortar.append((xx - 3, xx + 3, yy, min(yy + 50, top)))
        else:
            dirt.append((x0, x1, 0, top - 22))
            grass.append((x0, x1, top - 22, top))
    parts += [("dirt", _rects(dirt, D_DIRT), "dirt"), ("grass", _rects(grass, D_TOP), "grass"), ("step", _rects(bricks, D_DIRT), "brick"),
              ("mortar", _rects(mortar, D_TOP), "mortar")]
    fx = g["FLAG_X"]
    parts += [("pole", _rects([(fx, fx + 12, 200, 560), (fx - 8, fx + 20, 200, 216)], D_FLAG), "pole"),
              ("flag", _static([(fx + 12, 470), (fx + 120, 510), (fx + 12, 550)], D_FLAG), "flag")]
    start = (g["START_X"], g["START_Y"])
    for name, outline, d, look in HERO:  # drawn at the start, about the character's bottom centre; moved from there each step
        parts.append((name, _flat(outline, d, start), look))
    for k, at in enumerate(HEARTS):
        parts.append((f"heart{k}", _flat(_heart(), D_HUD, at), "heart"))
    for k, at in enumerate(PIPS):
        parts.append((f"pip{k}", _flat(_rect(-30, 30, -30, 30), D_HUD, at), "pip"))
        parts.append((f"pip_lit{k}", _flat(_rect(-30, 30, -30, 30), D_HUD2, at), "pip_lit", True))
    parts += [("banner", _rects([(380, 1540, 470, 730)], D_BANNER), "banner", True),
              ("text_clear", _text("LEVEL CLEAR", 960, 600, 16, D_TEXT), "text_win", True),
              ("text_over", _text("GAME OVER", 960, 600, 16, D_TEXT), "text_lose", True)]
    return m.fixed("screen", (MON_FACE_X, 0.0, TOP + SCREEN_CZ), parts, (), LOOKS)


SCREEN = _screen()
HERO_HOME = screen_point(GAME["START_X"], GAME["START_Y"])  # where the character's shapes are authored; each step moves them from there

# -- distractors (SimReady): the computer keyboard pushed aside to the left, a mug on the right
KEYBOARD = SimReady("keyboard", pos=(0.62, 0.47, TOP), size=(0.442, 0.142, 0.020), mass=0.55, package="computer_keyboard_a01",
                    usd="sm_computer_keyboard_a01_01.usd", yaw=0.3)
MUG = SimReady("mug", pos=(0.46, -0.42, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.4)

OBJECTS = [PAD, CRADLE, MONITOR, SCREEN, KEYBOARD, MUG]


# ======================================================================================================================================
# The console: reads the controller, steps the game, draws it
# ======================================================================================================================================


def _stage_prim(name: str):
    import isaaclab.sim as sim_utils

    prim = sim_utils.get_current_stage().GetPrimAtPath(f"{m.ENV_PATH}/screen/visual/{name}")
    if not prim.IsValid():
        raise RuntimeError(f"no prim at screen/visual/{name}")
    return prim


def _move(name: str, at) -> None:
    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xformable(_stage_prim(name))
    ops = xf.GetOrderedXformOps()
    op = ops[0] if ops else xf.AddTranslateOp()
    op.Set(Gf.Vec3d(float(at[0]), float(at[1]), float(at[2])))


def _show(name: str, visible: bool) -> None:
    m.set_visible(f"{m.ENV_PATH}/screen/visual/{name}", visible)


def pad_inputs(env) -> tuple[float, float, float]:
    """(stick left-right tilt in degrees, + right; stick fore-aft tilt; A's travel in mm), read off the controller's joints."""
    s = m.state(env, "pad")
    return s.deg("sx"), s.deg("sy"), -s.joint("a") * 1000.0


class Console:
    """The game console: samples the stick and A once per control step, steps the game and draws the picture."""

    def __init__(self):
        self.game = Game()
        self.a_down = False
        self.drawn: dict = {}

    def reset(self, env=None):
        self.game.reset()
        self.a_down = False
        self.drawn = {}
        if env is not None:
            self.draw(force=True)

    def step(self, env):
        tilt, _, a_mm = pad_inputs(env)
        self.a_down = a_mm >= A_DOWN_MM or (self.a_down and a_mm > A_UP_MM)
        self.game.step(stick_axis(tilt), self.a_down)
        self.draw()

    def draw(self, force: bool = False):
        g = self.game
        hero_shown = (g.wait == 0) and g.y >= 0.0 and not (g.over and not g.clear)
        want = {"hero": (round(g.x, 1), round(g.y, 1), hero_shown), "lives": g.lives, "best": g.best, "clear": g.clear, "over": g.over and not g.clear}
        old = {} if force else self.drawn
        if old.get("hero") != want["hero"]:
            for name, _, _, _ in HERO:
                _move(name, (screen_point(g.x, g.y) - HERO_HOME) if hero_shown else np.zeros(3))
                _show(name, hero_shown)
        if old.get("lives") != want["lives"]:
            for k in range(GAME["LIVES"]):
                _show(f"heart{k}", k < g.lives)
        if old.get("best") != want["best"]:
            for k in range(4):
                _show(f"pip_lit{k}", k < g.best)
        if old.get("clear") != want["clear"] or old.get("over") != want["over"]:
            _show("banner", want["clear"] or want["over"])
            _show("text_clear", want["clear"])
            _show("text_over", want["over"])
        self.drawn = want


CONSOLE = Console()

# ======================================================================================================================================
# The scorer
# ======================================================================================================================================


def pad_gap(env) -> float:
    """The nearest fingertip point's distance (m) from the controller's outline (sticks included)."""
    s = m.state(env, "pad")
    local = s.to_local(m.fingertips(env))
    lo = np.array([PAD_BOX[0], PAD_BOX[2], PAD_BOX[4]])
    hi = np.array([PAD_BOX[1], PAD_BOX[3], PAD_BOX[5]])
    d = np.maximum(np.maximum(lo - local, local - hi), 0.0)
    return float(np.min(np.linalg.norm(d, axis=1)))


def touches_monitor(env, margin: float = 0.005) -> bool:
    tips = m.fingertips(env)
    lo = np.array([MON_BOX[0], MON_BOX[2], TOP + MON_BOX[4]]) - margin
    hi = np.array([MON_BOX[1], MON_BOX[3], TOP + MON_BOX[5]]) + margin
    return bool(np.any(np.all((tips >= lo) & (tips <= hi), axis=1)))


SCORE_OF = {0: 0.0, 1: 0.25, 2: 0.5, 3: 0.75, 4: 0.75}


class Trial:
    """The score, and the record of harm: the controller moved out of its place in the cradle, or a finger on the monitor."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        CONSOLE.reset(env)
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.moved = False
        self.monitor = False
        self.start = None
        self.last = {}
        self.n = 0

    def step(self, env):
        CONSOLE.step(env)
        g = CONSOLE.game
        s = m.state(env, "pad")
        if self.start is None:
            self.start = np.asarray(s.pos, float).copy()
        shift = float(np.linalg.norm(np.asarray(s.pos, float) - self.start))
        tilt = s.tilt_deg()
        if shift > 0.010 or tilt > 10.0:
            if not self.moved:
                print(f"[pad] the controller moved {shift * 1000:.0f} mm, tilted {tilt:.1f} deg: out of its cradle", flush=True)
            self.moved = True
        if touches_monitor(env):
            if not self.monitor:
                print("[pad] a finger touched the monitor", flush=True)
            self.monitor = True
        sx, sy, a_mm = pad_inputs(env)
        gap = pad_gap(env)
        centred = abs(sx) <= 3.0 and abs(sy) <= 3.0
        harm = self.moved or self.monitor
        ok = g.clear and gap >= 0.03 and centred and not CONSOLE.a_down and a_mm < A_UP_MM
        self.hold = self.hold + 1 if (ok and not harm) else 0
        self.full = self.hold >= HOLD_STEPS
        if harm:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = SCORE_OF[g.best]
        self.last = {"x": g.x, "y": g.y, "best": g.best, "lives": g.lives, "clear": g.clear, "over": g.over, "gap": gap, "sx": sx, "sy": sy, "a_mm": a_mm,
                     "shift": shift, "tilt": tilt}
        self.n += 1


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# ======================================================================================================================================
# The demonstration
# ======================================================================================================================================


def read_screen() -> dict:
    """What the monitor shows, read off the drawn picture: the character's drawn bottom centre in game pixels (None while not drawn),
    and whether the LEVEL CLEAR or GAME OVER banner is up."""
    from pxr import UsdGeom

    prim = _stage_prim("hero_body")
    ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
    shown = UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible
    hero = None
    if shown:
        hero = to_pixels(HERO_HOME + (np.array(ops[0].Get(), float) if ops else np.zeros(3)))
    vis = lambda n: UsdGeom.Imageable(_stage_prim(n)).ComputeVisibility() != UsdGeom.Tokens.invisible  # noqa: E731
    return {"hero": hero, "clear": vis("text_clear"), "over": vis("text_over")}


def _report(ctx, env, label=""):
    c = TRIAL.last
    if not c:
        return
    print(f"[pad] t {ctx.t:5.1f} {label}hero ({c['x']:.0f}, {c['y']:.0f}) best {c['best']} lives {c['lives']} clear {c['clear']} over {c['over']} "
          f"stick {c['sx']:+5.1f} {c['sy']:+5.1f} A {c['a_mm']:.2f} mm gap {c['gap'] * 1000:.0f} mm shift {c['shift'] * 1000:.1f} mm tilt {c['tilt']:.1f} "
          f"moved {TRIAL.moved} monitor {TRIAL.monitor} score {TRIAL.score:.2f}", flush=True)


MODES = ("full", "none", "onearm", "mash", "tap", "walk", "hold", "grab")
# where the demo wants the character's front (right) edge to leave the ground, for gap 1, the step and gap 2; by how it plays
TAKEOFF = {"full": (495.0, 700.0, 1375.0), "tap": (525.0, 760.0, 1415.0), "walk": (525.0, 730.0, 1400.0)}


def next_takeoff(x: float, y: float, plan) -> float | None:
    """The take-off point for the next obstacle, from where the character is drawn (bottom centre); None past the last one."""
    b = GAME["BLOCKS"]
    if x < b[1][1]:  # not yet over gap 1
        return plan[0]
    if y < b[2][3] - 1.0 and x < b[2][2]:  # not yet up on the step
        return plan[1]
    if x < b[3][1]:  # not yet over gap 2
        return plan[2]
    return None


def want_jump(h, prev, delay: int, plan) -> bool:
    """Press A now? `h`, `prev`: the character as drawn now and a step ago. Yes when it stands and will be at the next take-off point
    after the `delay` steps that a press takes to reach the game."""
    if h is None or prev is None:
        return False
    standing = abs(h[1] - prev[1]) < 0.5 and any(abs(h[1] - blk[3]) < 0.5 for blk in GAME["BLOCKS"])
    t = next_takeoff(h[0], h[1], plan)
    if not standing or t is None:
        return False
    vx = (h[0] - prev[0]) / DT
    return h[0] + GAME["CHAR_W"] / 2 + vx * DT * delay >= t


def finger_low(side: str, grip, opening: float = 0.0) -> np.ndarray:
    """The closed fingers' lowest point, from the robot's own collision hulls, as an offset from the TCP in world axes, for a grip."""
    from yam_arena import analysis, ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY

    base = np.array(station.BASES[side], float)
    target = np.array([PAD0[0], 0.04 if side == "left" else -0.04, TOP + 0.08])
    q, _, _ = ik.solve(target, ik.matrix_to_quat_xyzw(grip.rot), np.array(READY, float), base_pos=base, iters=80)
    qd = {j: float(v) for j, v in zip(ARM_JOINTS, q)}
    qd.update({FINGER_JOINTS[0]: opening, FINGER_JOINTS[1]: opening})
    poses = analysis.fk_link_poses_world(qd, base)
    tcp = ik.tcp_pose(q, base)[0]
    v = np.vstack([h.vertices for _, h in analysis.world_hulls(poses, ["tip_left", "tip_right"])]) - tcp
    band = v[v[:, 2] < v[:, 2].min() + 0.001]
    return band.mean(axis=0)


def demo_script(ctx):
    """Left hand: nearly straight down (leaned 15 degrees out so the wrists stay apart), its fingers opened 1.2 mm wider than the left
    stick's cap on either side of it (left and right of it), so it steers the stick as a thumb does, pushing the cap's side, with no
    squeeze. Right hand: closed, tilted 35 degrees inward from its own side, its fingers' lowest point over A. Each step: the fingers
    are put where the stick should tilt, plus a sideways correction learnt from the tilt seen (the arm gives a few mm under the stick's
    push); A is pressed (fingertips 5 mm below A's top: A bottoms out at 1.5 mm) or let up (4 mm above it). The script reads only the
    controller's pose, the stick's tilt and A's travel (both visible), and the character as drawn on the screen:
      1. a calibration jump in place at the start, which measures the steps from pressing A to the character leaving the ground;
      2. full stick right (20.5 degrees); before each obstacle, A pressed and held 0.45 s once the character, standing, will be at a
         take-off point ahead of it after that delay;
      3. after a fall, the same again from the start;
      4. on LEVEL CLEAR: stick back to centre, A up, both hands open and off the controller.
    --mode: none does nothing; onearm runs with the stick only; mash holds A down from the start; tap only taps A (0.15 s); walk runs
    at half stick (10.5 degrees); hold does it all but keeps both hands on the controller; grab pinches the stick and lifts the
    controller out of its cradle."""
    import demokit as d
    from yam_arena import ik

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in MODES:
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(100):
            yield
        _report(ctx, env, "end: ")
        return

    pad = lambda: ctx.state("pad")  # noqa: E731

    def cap_point(a_deg: float, s=None) -> np.ndarray:
        """The cap's middle (world) with the stick tilted `a_deg` to the right."""
        s = pad() if s is None else s
        a = math.radians(a_deg)
        p = (LS[0], LS[1] - math.sin(a) * R_CAP, LS_PIVOT_Z + math.cos(a) * R_CAP)
        return s.to_world(np.array([p]))[0]

    from yam_arena.urdf import axis_angle_matrix

    # fingers closing along y (the pads either side of the cap), leaned 15 degrees out about x so the two wrists stay 3-4 cm apart
    gl = d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(-15.0)) @ d.down(0.0).rot)
    gr = d.tilted(35.0, 90.0)  # tilted inward (+y) from the right arm's side, fingers closing along x
    FORK = -(CAP_R + 0.0012)  # the left fingers' opening: 1.2 mm of play each side of the cap
    low = finger_low("right", gr)  # the closed right fingers' lowest point, from the TCP (the arm's own geometry)
    tip_l = finger_low("left", gl, FORK)  # and the left's, opened round the cap

    def a_point(depth: float, s=None) -> np.ndarray:
        """The TCP for the fingertips' lowest edge at A's middle, `depth` below the top of its cap (negative: above)."""
        s = pad() if s is None else s
        bx, by = BUTTONS["a"]
        p = s.to_world(np.array([(bx, by, Z_FACE + BTN_PROUD - depth)]))[0]
        return p - low

    PINCH_TCP = (Z_FACE + 0.006 - CAP_C) - tip_l[2]  # the TCP above the cap's middle that puts the lower fingertip 6 mm over the face

    up_l = -gl.rot[:, 2] / -gl.rot[2, 2]  # back up the leaned fingers' axis, per metre of height

    def cap_tcp(a_deg: float, s=None) -> np.ndarray:
        """The left TCP that centres the fingers on the cap (tilted `a_deg`), the lower fingertip 6 mm over the face."""
        return cap_point(a_deg, s) + up_l * PINCH_TCP

    use_r = mode != "onearm"
    high = np.array([0.0, 0.0, 0.08])
    moves = [l.move(cap_tcp(0.0) + high, grip=gl, finger=FORK - 0.006, t=2.5, label="over the stick")]
    if use_r:
        moves.append(r.move(a_point(-0.03) + high, grip=gr, finger=d.FINGER_CLOSED, t=2.5, label="over A"))
    yield from ctx.together(*moves)
    moves = [l.move(cap_tcp(0.0) + np.array([0.0, 0.0, 0.02]), t=1.0, label="just over the stick")] + ([r.move(a_point(-0.012), t=1.0, label="just over A")] if use_r else [])
    yield from ctx.together(*moves)
    moves = [l.servo(cap_tcp(0.0) + np.array([0.0, 0.0, 0.02]), timeout=1.0, label="just over the stick")] + ([r.servo(a_point(-0.012), timeout=1.0, label="just over A")] if use_r else [])
    yield from ctx.together(*moves)
    yield from l.move(cap_tcp(0.0), t=1.0, label="round the cap")
    yield from l.servo(cap_tcp(0.0), timeout=1.0, label="round the cap")
    yield from l.move(l.at(), finger=FORK, t=0.5, label="fingers either side of the cap")
    _report(ctx, env, "fingers round the cap: ")

    if mode == "grab":  # pinch the stick, straight down and low, and lift the controller by it, as if to pick it up, and put it back
        gd = d.down(0.0)
        grab_at = cap_point(0.0) + np.array([0.0, 0.0, (Z_FACE + 0.0025 - CAP_C) - finger_low("left", gd, FORK)[2]])
        yield from l.move(l.at() + np.array([0.0, 0.0, 0.03]), finger=FORK - 0.006, t=0.6, label="up off the stick")
        yield from l.move(grab_at + np.array([0.0, 0.0, 0.03]), grip=gd, t=1.0, label="over the stick, straight down")
        yield from l.move(grab_at, t=0.8, label="round the cap")
        yield from l.servo(grab_at, timeout=1.0, label="round the cap")
        yield from l.close(2 * CAP_R, 20.0, t=0.5, label="pinch the cap")
        yield from l.move(l.at() + np.array([0.0, 0.0, 0.05]), t=1.5, label="lift the controller")
        _report(ctx, env, "lifted: ")
        yield from l.move(l.at() - np.array([0.0, 0.0, 0.05]), t=1.5, label="put it back")
        yield from l.open(t=0.4)
        yield from l.move(l.at() + np.array([0.0, 0.0, 0.06]), t=1.0, label="off")
        for _ in range(30):
            yield
        _report(ctx, env, "end: ")
        return

    base_l = ik.tcp_pose(l.q, l.base)[0] - cap_tcp(0.0)  # what the left hand's command needs on top of the cap's place (droop)
    base_r = (ik.tcp_pose(r.q, r.base)[0] - a_point(-0.012)) if use_r else np.zeros(3)
    st = {"I": 0.0, "a": False}  # I: metres

    def drive(tilt_want: float, press: bool):
        """One step: the cap toward `tilt_want` degrees (+ right), corrected by the tilt seen; A pressed or let up."""
        s = pad()
        seen = s.deg("sx")
        # a sideways correction (m, + to the stick's right), learnt from the tilt seen: the arm gives under the stick's push
        st["I"] = float(np.clip(st["I"] + 0.0001 * (tilt_want - seen), -0.010, 0.010)) if abs(tilt_want) > 0.5 else 0.0
        goal = cap_tcp(float(np.clip(tilt_want, -STICK_MAX_DEG, STICK_MAX_DEG)), s) + s.rotation() @ np.array([0.0, -st["I"], 0.0])
        goal[2] = cap_tcp(0.0, s)[2]  # the fingers push sideways only, at one height
        mm, dg = l._solve(goal + base_l, l.grip.rot)
        l._note(mm, dg, "stick")
        if use_r:
            mm, dg = r._solve(a_point(0.005 if press else -0.004, s) + base_r, r.grip.rot)
            r._note(mm, dg, "A")
        st["a"] = press
        yield

    # -- the run
    g = GAME
    hw = g["CHAR_W"] / 2
    full_tilt = 10.5 if mode == "walk" else 20.5
    hold_steps = 3 if mode == "tap" else 9  # a tap: A sent down for 3 steps (0.15 s); a held jump: 9 steps (0.45 s)
    plan = TAKEOFF.get(mode, TAKEOFF["full"])

    def hero():
        return read_screen()["hero"]

    if mode == "mash":  # A down from the start and never let up
        for _ in range(4):
            yield from drive(0.0, True)
        t0 = ctx.t
        while ctx.t - t0 < 30.0:
            scr = read_screen()
            if scr["over"] or scr["clear"]:
                break
            yield from drive(full_tilt, True)
        _report(ctx, env, "end: ")
        for _ in range(20):
            yield from drive(0.0, True)
        _report(ctx, env, "end: ")
        return

    # 1. calibration: press A, standing, and count the steps until the drawn character rises
    delay = 3
    if use_r:
        y0 = hero()[1]
        for k in range(20):
            yield from drive(0.0, k < 8)
            h = hero()
            if h is not None and h[1] > y0 + 1.0:
                delay = k + 1
                break
        for _ in range(25):
            yield from drive(0.0, False)
        print(f"[pad] A to take-off: {delay} steps", flush=True)
    _report(ctx, env, "calibrated: ")

    # 2. the run
    press_left, release_left = 0, 0
    prev = hero()
    t0 = ctx.t
    done = False
    while ctx.t - t0 < 40.0:
        scr = read_screen()
        h = scr["hero"]
        if scr["clear"] or scr["over"]:
            done = True
            break
        press = False
        if press_left > 0:
            press = True
            press_left -= 1
        elif use_r and release_left <= 0 and want_jump(h, prev, delay, plan):
            press = True
            press_left = hold_steps - 1
            release_left = hold_steps + 4
            print(f"[pad] t {ctx.t:5.1f} jump: front {h[0] + hw:.0f} speed {(h[0] - prev[0]) / DT:.0f} px/s, take-off point {next_takeoff(h[0], h[1], plan):.0f}", flush=True)
        release_left -= 1
        prev = h
        yield from drive(full_tilt, press)
    _report(ctx, env, "level over: " if done else "run timed out: ")

    # 4. let go: stick centred, A up, hands open and away
    for _ in range(6):
        yield from drive(0.0, False)
    if mode == "hold":
        for _ in range(40):
            yield from drive(0.0, False)
        _report(ctx, env, "end: ")
        return
    back = [l.open(t=0.4, label="let go of the stick")]
    if use_r:
        back.append(r.move(r.at() + np.array([0.0, -0.02, 0.03]), t=0.5, label="off A"))
    yield from ctx.together(*back)
    back = [l.move(l.at() + np.array([0.0, 0.0, 0.05]), t=0.8, label="up off the stick")]
    yield from ctx.together(*back)
    back = [l.move((0.20, 0.25, TOP + 0.22), t=1.5, label="left hand back")] + ([r.move((0.20, -0.25, TOP + 0.22), t=1.5, label="right hand back")] if use_r else [])
    yield from ctx.together(*back)
    for k in range(60):
        yield
        if k % 20 == 19:
            _report(ctx, env, "end: ")
