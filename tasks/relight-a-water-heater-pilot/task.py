"""Relight a gas water heater's standing pilot after a gas shutoff, and leave it set to HOT.

The lower front of a 40-gallon atmospheric natural-gas water heater stands on the table (the table is the floor): a full-size section of its 20 in
(508 mm) jacket, the outer burner door off so that the inner door and its sight glass show, and on the jacket its gas control, modelled on the
Resideo (Honeywell) WV8840B standing-pilot control fitted to A.O. Smith heaters (trade replacement WT8840B1000), at the height A.O. Smith gives
for a GCR-40: the 1/2 in gas inlet 12-1/4 in above the floor. The gas was shut off for a repair and is back on: the pilot is out, the knob is at
OFF and the status light is dark.

The control, as its maker and the heater's maker document it (README, Sources):
- One knob, 29 mm across, on the control's front face: OFF, PILOT, VAC, LOW, HOT (the 120 F mark), A, B, C, VERY HOT, clockwise. At PILOT, and only
  there, it pushes in about 1/4 in (6.35 mm) against its pilot valve's spring; held in, pilot gas flows.
- The piezo igniter, a square button 40 mm below the knob on the same face, pushed straight in; each full press snaps and sparks.
- The status light, 78 mm right of the knob: it starts to blink (once every 3 s) when the pilot's thermopile is hot enough for the control to hold the
  pilot valve itself. Released before that, the knob springs out, the valve shuts and the pilot goes out; the control then needs 10 minutes before
  another lighting attempt (A.O. Smith), longer than the trial. Turned back to OFF, the pilot goes out the same way.

Model (README "Model"): the flame, the spark and the thermopile are bookkeeping, not combustion. A spark lights the pilot when the knob is held in at
least 5.5 mm at PILOT. The thermopile warms over 40 s of continuous flame (the control then holds the valve and the light blinks) and cools over 60 s
without it. Knob, push-in and igniter are joints of one articulation with sprung, estimated forces.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "The gas is back on after a repair. Light the water heater and leave it set to HOT."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 100  # 5 s at 20 Hz: lit, held by the control, knob released at HOT, hands clear

# -- the gas control, in its own frame: origin on the knob's axis in the cover's front plane; x into the control (towards the tank), y left, z up
CTRL = np.array([0.32, 0.05, TOP + 0.3225])  # the knob's axis 322.5 mm above the floor: the gas inlet, 11.5 mm below it, at 12-1/4 in (A.O. Smith GCR-40, D)
COVER_Y = (-0.1095, 0.0255)  # 135 mm wide; the knob's axis 25.5 mm from the cover's left edge (the robot's left, +y)
COVER_Z = (-0.0565, 0.0765)  # 133 mm tall; the knob's axis 76.5 mm below the top
COVER_D = 0.081  # 3-3/16 in deep, front face to the back
KNOB_R, KNOB_BODY, KNOB_RIDGE = 0.0145, 0.011, 0.005  # 29 mm knob: an 11 mm body, and a 5 mm grip ridge across its face (16 mm proud in all)
RIDGE_W, RIDGE_L = 0.005, 0.026
PUSH = 0.00635  # 1/4 in: how far the knob goes in at PILOT
PUSH_OPEN, PUSH_SHUT = 0.0055, 0.0050  # held in this far, the pilot valve is open; let out past the second, it shuts
KNOB_OUT = 0.001  # the knob counts as let go (out) when it is in less than this
K_KNOB, KNOB_PRE = 1500.0, -0.0040  # pilot-valve spring (N/m) and its preload offset: about 6 N to start, 15.5 N fully in (estimates)
IGN = np.array([0.0, -0.0025, -0.040])  # the igniter's centre on the face: 40 mm below the knob, 2.5 mm right
IGN_SIDE, IGN_PROUD, IGN_TRAVEL, IGN_SNAP, IGN_REARM = 0.011, 0.007, 0.005, 0.004, 0.0015
K_IGN, IGN_PRE, IGN_AFTER = 3000.0, -0.0027, 0.0030  # 8 N to start, 20 N at the snap (4 mm), then the hammer is released (estimates)
LED = np.array([0.0, -0.078, 0.0145])  # the status light: 78 mm right of the knob, 14.5 mm above it
# knob angles, degrees clockwise as the face is seen (positive about +x), from OFF; read off the maker's label (README, Sources)
DIAL = {"OFF": 0.0, "PILOT": 30.0, "VAC": 60.0, "LOW": 92.0, "HOT": 122.0, "A": 155.0, "B": 182.0, "C": 206.0, "VERY HOT": 244.0}
OFF_CLOCK = 150.0  # the clock angle of OFF on the dial (12 o'clock = 0, clockwise as seen)
PILOT, HOT = DIAL["PILOT"], DIAL["HOT"]
PILOT_WIN, DETENT_WIN, HOT_TOL = 4.0, 5.0, 8.0
# the pilot and its thermopile (bookkeeping)
T_WARM, T_COOL = 40.0, 60.0  # s of flame until the control can hold the pilot valve; s for the thermopile to cool
LOCKOUT_S = 600.0  # A.O. Smith: 10 minutes between lighting attempts
UNLIT_LIMIT = 90.0  # A.O. Smith label: click the igniter "for up to 90 seconds"; an operating limit on pilot gas let out with no flame
BLINK_PERIOD, BLINK_ON = 3.0, 0.5
CLEAR = 0.03  # every fingertip point this far from the knob and the igniter: hands off

# -- the heater section (world frame, heights from the table top)
R_JACKET = 0.254  # 20 in jacket (A.O. Smith GCR-40, C)
JACKET_FRONT = CTRL[0] + COVER_D + 0.005  # the control's back stands 5 mm off the jacket
XJ = JACKET_FRONT + R_JACKET  # the tank's axis
SECTION_H, SECTION_DEG = 0.56, 58.0
DOOR = (0.080, 0.035, 0.215)  # the burner-door opening: half width, bottom, top
GLASS = (0.0, 0.110, 0.013, 0.010)  # the inner door's sight glass: y, z, half width, half height

LOOKS = {
    "jacket": m.Look(color=m.rgb(0.80, 0.80, 0.78), roughness=0.45, metallic=0.2),
    "section": m.Look(color=m.rgb(0.62, 0.62, 0.60), roughness=0.8),
    "door": m.Look(color=m.rgb(0.30, 0.31, 0.33), roughness=0.5, metallic=0.4),
    "chamber": m.Look(color=m.rgb(0.03, 0.03, 0.03), roughness=0.9),
    "glass": m.Look(glass=(0.85, 0.85, 0.85)),
    "flame": m.Look(color=m.rgb(0.25, 0.45, 1.0), emissive=(0.3, 0.6, 4.0), roughness=0.5),
    "flame_tip": m.Look(color=m.rgb(1.0, 0.65, 0.2), emissive=(3.0, 1.6, 0.3), roughness=0.5),
    "pilot": m.Look(color=m.rgb(0.55, 0.55, 0.55), roughness=0.4, metallic=0.8),
    "pipe": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.5, metallic=0.6),
    "brass": m.Look(color=m.rgb(0.78, 0.62, 0.30), roughness=0.35, metallic=0.9),
    "handle": m.Look(color=m.rgb(0.95, 0.80, 0.05), roughness=0.5),
    "alu": m.Look(color=m.rgb(0.75, 0.75, 0.76), roughness=0.4, metallic=0.8),
    "cover": m.Look(color=m.rgb(0.70, 0.70, 0.69), roughness=0.45, metallic=0.3),
    "clip": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.6),
    "ink": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.7),
    "print": m.Look(color=m.rgb(0.45, 0.45, 0.45), roughness=0.7),
    "knob": m.Look(color=m.rgb(0.85, 0.08, 0.08), roughness=0.45),
    "white": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.5),
    "button": m.Look(color=m.rgb(0.90, 0.10, 0.08), roughness=0.5),
    "bezel": m.Look(color=m.rgb(0.55, 0.55, 0.55), roughness=0.5),
    "led_off": m.Look(color=m.rgb(0.25, 0.05, 0.05), roughness=0.2),
    "led_on": m.Look(color=m.rgb(1.0, 0.06, 0.03), emissive=(8.0, 0.2, 0.06), roughness=0.2),
    "led_ring": m.Look(color=m.rgb(0.35, 0.35, 0.35), roughness=0.4, metallic=0.5),
}


# -- small helpers ----------------------------------------------------------------------------------------------------------------------------------

def face(u, v, x=0.0):
    """A point on the control's face: u to the right as the face is seen (-y), v up (+z), x into the control."""
    return np.array([x, -u, v])


def clock(alpha_deg, r):
    """A point on the face at clock angle alpha (12 o'clock = 0, clockwise as seen) and radius r from the knob's axis: (u, v)."""
    a = math.radians(alpha_deg)
    return r * math.sin(a), r * math.cos(a)


def seg(p0, p1, width, depth, x):
    """A thin bar on the face from (u0, v0) to (u1, v1), standing `depth` proud of the plane x."""
    (u0, v0), (u1, v1) = p0, p1
    L = math.hypot(u1 - u0, v1 - v0)
    bar = m.box((depth, max(L, 1e-4) + width, width))  # length along y before turning
    th = math.degrees(math.atan2(v1 - v0, -(u1 - u0)))  # direction in the y-z plane
    return bar.turned(th, "x").moved(face((u0 + u1) / 2, (v0 + v1) / 2, x - depth / 2))


# a stroke font, on a 4 x 6 grid, for the dial's words
FONT = {
    "A": [((0, 0), (2, 6)), ((2, 6), (4, 0)), ((1, 3), (3, 3))],
    "B": [((0, 0), (0, 6)), ((0, 6), (3, 6)), ((3, 6), (4, 5)), ((4, 5), (3, 3)), ((3, 3), (0, 3)), ((3, 3), (4, 2)), ((4, 2), (4, 1)), ((4, 1), (3, 0)), ((3, 0), (0, 0))],
    "C": [((4, 6), (0, 6)), ((0, 6), (0, 0)), ((0, 0), (4, 0))],
    "E": [((0, 0), (0, 6)), ((0, 6), (4, 6)), ((0, 3), (3, 3)), ((0, 0), (4, 0))],
    "F": [((0, 0), (0, 6)), ((0, 6), (4, 6)), ((0, 3), (3, 3))],
    "H": [((0, 0), (0, 6)), ((4, 0), (4, 6)), ((0, 3), (4, 3))],
    "I": [((2, 0), (2, 6)), ((1, 0), (3, 0)), ((1, 6), (3, 6))],
    "L": [((0, 6), (0, 0)), ((0, 0), (4, 0))],
    "O": [((0, 0), (4, 0)), ((4, 0), (4, 6)), ((4, 6), (0, 6)), ((0, 6), (0, 0))],
    "P": [((0, 0), (0, 6)), ((0, 6), (3, 6)), ((3, 6), (4, 5)), ((4, 5), (4, 4)), ((4, 4), (3, 3)), ((3, 3), (0, 3))],
    "R": [((0, 0), (0, 6)), ((0, 6), (3, 6)), ((3, 6), (4, 5)), ((4, 5), (4, 4)), ((4, 4), (3, 3)), ((3, 3), (0, 3)), ((2, 3), (4, 0))],
    "S": [((4, 6), (0, 6)), ((0, 6), (0, 3)), ((0, 3), (4, 3)), ((4, 3), (4, 0)), ((4, 0), (0, 0))],
    "T": [((0, 6), (4, 6)), ((2, 6), (2, 0))],
    "U": [((0, 6), (0, 0)), ((0, 0), (4, 0)), ((4, 0), (4, 6))],
    "V": [((0, 6), (2, 0)), ((2, 0), (4, 6))],
    "W": [((0, 6), (1, 0)), ((1, 0), (2, 3)), ((2, 3), (3, 0)), ((3, 0), (4, 6))],
    "Y": [((0, 6), (2, 3)), ((4, 6), (2, 3)), ((2, 3), (2, 0))],
}


def text(word, cu, cv, height, x=-0.0002):
    """Meshes for `word`, centred at (cu, cv) on the face, capital height `height`."""
    s = height / 6.0
    adv = 5.4 * s
    width = len(word) * adv - 1.4 * s
    out = []
    for k, ch in enumerate(word):
        if ch == " ":
            continue
        u0 = cu - width / 2 + k * adv
        for (a, b) in FONT[ch]:
            out.append(seg((u0 + a[0] * s, cv - 3 * s + a[1] * s), (u0 + b[0] * s, cv - 3 * s + b[1] * s), 0.75 * s, 0.0002, x))
    return out


# -- the gas control (a mechanism: the cover is its frame; push-in, knob and igniter are its links) ------------------------------------------------

def _control():
    fp, fc = [], []
    cy0, cy1 = COVER_Y
    cz0, cz1 = COVER_Z
    cover = m.box((COVER_D - 0.004, cy1 - cy0, cz1 - cz0), ((COVER_D + 0.004) / 2, (cy0 + cy1) / 2, (cz0 + cz1) / 2))
    front = m.box((0.004, cy1 - cy0 - 0.006, cz1 - cz0 - 0.006), (0.002, (cy0 + cy1) / 2, (cz0 + cz1) / 2))
    fp += [("cover", cover, "cover"), ("cover_front", front, "cover")]
    fc.append(m.col_box("cover_c", (COVER_D, cy1 - cy0, cz1 - cz0), (COVER_D / 2, (cy0 + cy1) / 2, (cz0 + cz1) / 2)))
    # the black mounting clips at the cover's sides, the cast inlet boss (left) and the valve body under the cover
    for k, (yy, zz) in enumerate(((cy1 + 0.004, cz1 - 0.025), (cy1 + 0.004, cz0 + 0.03), (cy0 - 0.004, cz1 - 0.025), (cy0 - 0.004, cz0 + 0.03))):
        fp.append((f"clip{k}", m.box((0.03, 0.008, 0.035), (0.04, yy, zz)), "clip"))
    fp.append(("inlet", m.box((0.045, 0.022, 0.037), (0.0525, cy1 + 0.011, -0.0115)), "alu"))
    fc.append(m.col_box("inlet_c", (0.045, 0.022, 0.037), (0.0525, cy1 + 0.011, -0.0115)))
    fp.append(("body", m.box((0.06, 0.10, 0.018), (0.051, -0.03, cz0 - 0.009)), "alu"))
    fc.append(m.col_box("body_c", (0.06, 0.10, 0.018), (0.051, -0.03, cz0 - 0.009)))
    # printed dial: ticks, the OFF ring, the PILOT triangle, the words
    ink = []
    for name, ang in DIAL.items():
        a = OFF_CLOCK + ang
        if name == "OFF":
            cu, cv = clock(a, 0.0168)
            ink.append(m.cylinder(0.0011, 0.0002, tuple(face(cu, cv, -0.0001)), axis="x", segments=12))
        elif name == "PILOT":
            cu, cv = clock(a, 0.0165)
            ink += [seg((cu - 0.0010, cv - 0.0008), (cu + 0.0010, cv - 0.0008), 0.0004, 0.0002, -0.0002),
                    seg((cu - 0.0010, cv - 0.0008), (cu, cv + 0.0009), 0.0004, 0.0002, -0.0002),
                    seg((cu + 0.0010, cv - 0.0008), (cu, cv + 0.0009), 0.0004, 0.0002, -0.0002)]
        else:
            ink.append(seg(clock(a, 0.0155), clock(a, 0.0178), 0.0007, 0.0002, -0.0002))
        if name == "VERY HOT":
            cu, cv = clock(a, 0.0225)
            ink += text("VERY", cu, cv + 0.0016, 0.0022) + text("HOT", cu, cv - 0.0016, 0.0022)
        else:
            cu, cv = clock(a, 0.0215 if len(name) < 3 else 0.0225)
            ink += text(name, cu, cv, 0.0022)
    ink += text("STATUS", -LED[1], LED[2] - 0.0075, 0.0016)
    fp.append(("dial", m.join(*ink), "ink"))
    # the small printed labels (lighting steps, scald warning): lines standing for 1.5 mm print
    lines = [seg((0.022, 0.004 - 0.0035 * k), (0.062 - 0.006 * (k == 0), 0.004 - 0.0035 * k), 0.0010, 0.0002, -0.0002) for k in range(4)]
    lines += [seg((-0.012, 0.030 - 0.0035 * k), (0.012, 0.030 - 0.0035 * k), 0.0010, 0.0002, -0.0002) for k in range(2)]
    fp.append(("labels", m.join(*lines), "print"))
    # the lightning mark beside the igniter
    lu, lv = -IGN[1] - 0.0115, IGN[2]
    zig = [seg((lu + 0.0012, lv + 0.0040), (lu - 0.0008, lv + 0.0002), 0.0006, 0.0002, -0.0002), seg((lu - 0.0008, lv + 0.0002), (lu + 0.0010, lv), 0.0006, 0.0002, -0.0002),
           seg((lu + 0.0010, lv), (lu - 0.0010, lv - 0.0040), 0.0006, 0.0002, -0.0002)]
    fp.append(("bolt", m.join(*zig), "ink"))
    # the status light: a ring, its dark lens and (hidden until lit) its glowing lens
    led = face(-LED[1], LED[2])
    fp.append(("led_ring", m.cylinder(0.00375, 0.0010, tuple(led + [-0.0005, 0, 0]), axis="x", segments=20), "led_ring"))
    fp.append(("led_lens", m.cylinder(0.00225, 0.0006, tuple(led + [-0.0013, 0, 0]), axis="x", segments=20), "led_off"))
    fp.append(("led_on", m.cylinder(0.00235, 0.0008, tuple(led + [-0.0015, 0, 0]), axis="x", segments=20), "led_on", True))
    # the igniter's raised square bezel (15 mm outside, 12.4 mm inside, 1.5 mm proud)
    iy, iz = IGN[1], IGN[2]
    for k, (dy, dz, sy, sz) in enumerate(((0.0, 0.0069, 0.015, 0.0012), (0.0, -0.0069, 0.015, 0.0012), (0.0069, 0.0, 0.0012, 0.015), (-0.0069, 0.0, 0.0012, 0.015))):
        fp.append((f"bezel{k}", m.box((0.0015, sy, sz), (-0.00075, iy + dy, iz + dz)), "bezel"))
    # thermopile connector and COM connector on the face
    fp.append(("tp_conn", m.box((0.004, 0.014, 0.016), (-0.002, -0.077, -0.035)), "clip"))
    fp.append(("com_conn", m.box((0.004, 0.020, 0.008), (-0.002, -0.037, -0.046)), "clip"))
    # links
    plunger = m.slider("plunger", "X", 0.0, PUSH, (), (), at=(0.0, 0.0, 0.0), spring=K_KNOB, damping=5.0, effort=60.0, friction=0.0, mass=0.02)
    body = m.cylinder(KNOB_R, KNOB_BODY, (-KNOB_BODY / 2, 0.0, 0.0), axis="x", segments=32)
    skirt = m.cylinder(KNOB_R + 0.0008, 0.0015, (-0.00075, 0.0, 0.0), axis="x", segments=32)
    ridge = m.box((KNOB_RIDGE, RIDGE_W, RIDGE_L), (-KNOB_BODY - KNOB_RIDGE / 2, 0.0, 0.0)).turned(OFF_CLOCK, "x")
    pointer = m.box((0.0004, 0.0016, 0.009), (-KNOB_BODY - KNOB_RIDGE - 0.0002, 0.0, 0.0075)).turned(OFF_CLOCK, "x")
    knob = m.hinge("knob", "X", (0.0, 0.0, 0.0), 0.0, DIAL["VERY HOT"], (("body", body, "knob"), ("skirt", skirt, "knob"), ("ridge", ridge, "knob"), ("pointer", pointer, "white")),
                   (m.col_hull("knob_body_c", m.cylinder(KNOB_R, KNOB_BODY, (-KNOB_BODY / 2, 0.0, 0.0), axis="x", segments=20)), m.col_hull("knob_ridge_c", ridge)),
                   spring=0.4, damping=0.002, friction=0.02, effort=0.15, mass=0.03, parent="plunger")
    cap = m.box((IGN_PROUD + 0.004, IGN_SIDE, IGN_SIDE), (-(IGN_PROUD - 0.004) / 2, iy, iz))
    igniter = m.slider("igniter", "X", 0.0, IGN_TRAVEL, (("cap", cap, "button"),), (m.col_box("igniter_c", (IGN_PROUD + 0.004, IGN_SIDE, IGN_SIDE), (-(IGN_PROUD - 0.004) / 2, iy, iz)),),
                       spring=K_IGN, damping=3.0, effort=60.0, friction=0.0, mass=0.01)
    return m.mechanism("control", tuple(CTRL), [plunger, knob, igniter], LOOKS, frame_parts=fp, frame_colliders=fc, friction=(1.0, 0.9), solver_iters=48)


def _arc_solid(r, a0, a1, z0, z1, n=24):
    """The circular segment of the jacket between angles a0..a1 (degrees, about the tank's axis, 180 = facing the robot), as a convex outline extruded between two heights."""
    th = np.radians(np.linspace(a0, a1, n))
    pts = np.c_[XJ + r * np.cos(th), r * np.sin(th)]
    pts = pts[::-1] if np.cross(pts[1] - pts[0], pts[2] - pts[1]) < 0 else pts
    return m.prism(pts, z0, z1)


def _heater():
    """The heater section: the jacket's front from the floor line up 0.42 m, the burner-door opening with the inner door and sight glass, the pilot
    behind it (its flame shown only while lit), the gas supply with its shutoff valve, and the pilot and thermopile tubes."""
    parts, cols = [], []
    a0, a1 = 180.0 - SECTION_DEG, 180.0 + SECTION_DEG
    half = math.degrees(math.asin(DOOR[0] / R_JACKET))
    parts.append(("jacket_up", _arc_solid(R_JACKET, a0, a1, DOOR[2], SECTION_H), "jacket"))
    parts.append(("jacket_low", _arc_solid(R_JACKET, a0, a1, 0.0, DOOR[1]), "jacket"))
    parts.append(("jacket_l", _arc_solid(R_JACKET, 180.0 + half, a1, DOOR[1], DOOR[2], 12), "jacket"))
    parts.append(("jacket_r", _arc_solid(R_JACKET, a0, 180.0 - half, DOOR[1], DOOR[2], 12), "jacket"))
    parts.append(("cut", _arc_solid(R_JACKET - 0.003, a0 + 0.5, a1 - 0.5, SECTION_H, SECTION_H + 0.0008), "section"))
    xb = XJ - R_JACKET * math.cos(math.radians(SECTION_DEG))  # the section's flat back
    for k in range(8):  # colliders: slices of the jacket's front
        ya, yb = -R_JACKET * math.sin(math.radians(SECTION_DEG)) * (1 - k / 4), -R_JACKET * math.sin(math.radians(SECTION_DEG)) * (1 - (k + 1) / 4)
        yy = max(abs(ya), abs(yb)) if ya * yb > 0 else 0.0
        xf = XJ - math.sqrt(R_JACKET ** 2 - yy ** 2)
        cols.append(m.col_box(f"jacket_c{k}", (xb - xf, abs(yb - ya), SECTION_H), ((xf + xb) / 2, (ya + yb) / 2, SECTION_H / 2)))
    # inner door, recessed in the opening, with its sight glass
    xd = XJ - math.sqrt(R_JACKET ** 2 - DOOR[0] ** 2) + 0.002
    gy, gz, gw, gh = GLASS
    y0, y1, z0, z1 = -DOOR[0], DOOR[0], DOOR[1], DOOR[2]
    for k, (ya, yb, za, zb) in enumerate(((y0, y1, z0, gz - gh), (y0, y1, gz + gh, z1), (y0, gy - gw, gz - gh, gz + gh), (gy + gw, y1, gz - gh, gz + gh))):
        parts.append((f"door{k}", m.box((0.002, yb - ya, zb - za), (xd, (ya + yb) / 2, (za + zb) / 2)), "door"))
    parts.append(("glass", m.box((0.001, 2 * gw, 2 * gh), (xd - 0.0005, gy, gz)), "glass"))
    parts.append(("chamber", m.box((0.004, 2 * DOOR[0], z1 - z0), (xd + 0.09, 0.0, (z0 + z1) / 2)), "chamber"))
    for k, yy in enumerate((-DOOR[0], DOOR[0])):
        parts.append((f"chamber_side{k}", m.box((0.09, 0.004, z1 - z0), (xd + 0.045, yy, (z0 + z1) / 2)), "chamber"))
    parts.append(("chamber_top", m.box((0.09, 2 * DOOR[0], 0.004), (xd + 0.045, 0.0, z1)), "chamber"))
    parts.append(("chamber_floor", m.box((0.09, 2 * DOOR[0], 0.004), (xd + 0.045, 0.0, z0)), "chamber"))
    # the pilot burner and thermopile behind the glass; the flame (hidden until lit)
    px, py, pz = xd + 0.035, gy + 0.004, gz - 0.006
    parts.append(("pilot_hood", m.cylinder(0.003, 0.012, (px, py, pz), segments=12), "pilot"))
    parts.append(("thermopile", m.cylinder(0.0025, 0.020, (px, py - 0.008, pz + 0.002), segments=12), "pilot"))
    flame = m.lathe([(0.0, pz + 0.006), (0.0030, pz + 0.009), (0.0026, pz + 0.016), (0.0, pz + 0.024)], 16).moved((px, py, 0.0))
    tip = m.lathe([(0.0, pz + 0.018), (0.0016, pz + 0.021), (0.0, pz + 0.028)], 12).moved((px, py, 0.0))
    parts += [("flame", flame, "flame", True), ("flame_tip", tip, "flame_tip", True)]
    # pilot tube, thermopile leads and manifold tube from under the control into the door
    cx, cy, cz = CTRL[0], CTRL[1], CTRL[2] - TOP
    bottom = cz + COVER_Z[0] - 0.018
    for k, (yy, r) in enumerate(((cy - 0.020, 0.0024), (cy - 0.032, 0.0018), (cy - 0.044, 0.0018), (cy - 0.060, 0.005))):
        parts.append((f"tube{k}", m.tube([(cx + 0.055, yy, bottom), (cx + 0.07, yy, bottom - 0.004), (xd + 0.004, yy * 0.7, z1 - 0.012)], r, 10), "alu" if k != 3 else "pilot"))
    # gas supply: 1/2 in black pipe from the control's inlet, a tee with a capped drip leg, the riser with its shutoff valve (open)
    gx, gzp = cx + 0.0525, cz - 0.0115
    yin = cy + COVER_Y[1] + 0.022
    yt = cy + 0.13
    r_p = 0.0107
    parts.append(("pipe_in", m.cylinder(r_p, yt - yin, (gx, (yin + yt) / 2, gzp), axis="y", segments=16), "pipe"))
    parts.append(("tee", m.cylinder(0.014, 0.05, (gx, yt, gzp), segments=16), "pipe"))
    parts.append(("drip", m.cylinder(r_p, 0.08, (gx, yt, gzp - 0.065), segments=16), "pipe"))
    parts.append(("drip_cap", m.cylinder(0.0135, 0.014, (gx, yt, gzp - 0.108), segments=16), "pipe"))
    parts.append(("riser", m.cylinder(r_p, SECTION_H + 0.05 - gzp, (gx, yt, (gzp + SECTION_H + 0.05) / 2), segments=16), "pipe"))
    vz = gzp + 0.13
    parts.append(("valve", m.box((0.034, 0.034, 0.05), (gx, yt, vz)), "brass"))
    parts.append(("valve_stem", m.cylinder(0.004, 0.012, (gx - 0.023, yt, vz), axis="x", segments=10), "brass"))
    parts.append(("valve_handle", m.box((0.006, 0.016, 0.085), (gx - 0.031, yt, vz + 0.03)), "handle"))
    for nm, mesh in (("pipe_in", m.cylinder(r_p, yt - yin, (gx, (yin + yt) / 2, gzp), axis="y", segments=12)), ("riser", m.cylinder(r_p, SECTION_H + 0.05 - gzp, (gx, yt, (gzp + SECTION_H + 0.05) / 2), segments=12)),
                     ("drip", m.cylinder(r_p, 0.09, (gx, yt, gzp - 0.07), segments=12))):
        cols.append(m.col_hull(nm + "_c", mesh))
    cols.append(m.col_box("valve_c", (0.034, 0.034, 0.05), (gx, yt, vz)))
    cols.append(m.col_box("handle_c", (0.006, 0.016, 0.085), (gx - 0.031, yt, vz + 0.03)))
    return m.fixed("heater", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.8, 0.7))


CONTROL = _control()
HEATER = _heater()
# distractors (SimReady): a plumber's tools and rag on the floor, clear of the heater and of the arms' paths to the control
WRENCH = SimReady("wrench", pos=(0.20, 0.27, TOP), size=(0.210, 0.064, 0.014), mass=0.30, package="handTool_wrench_adjustable_a01", usd="sm_handTool_wrench_adjustable_a01_01.usd",
                  yaw=math.radians(-20.0))
PLIERS = SimReady("pliers", pos=(0.52, -0.42, TOP), size=(0.246, 0.065, 0.014), mass=0.28, package="handTool_pliers_groove_joint_10in_b01", usd="sm_handTool_pliers_groove_joint_10in_b01_01.usd",
                  yaw=math.radians(70.0))
RAG = SimReady("rag", pos=(0.55, 0.42, TOP), size=(0.260, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01", usd="sm_cleaning_cloth_a01_01.usd", yaw=math.radians(10.0))
OBJECTS = [HEATER, CONTROL, WRENCH, PLIERS, RAG]


# -- the control's own behaviour: interlocks, the igniter's snap, the pilot, the thermopile and the status light --------------------------------------

LED_PATH = f"{m.ENV_PATH}/control/frame/visual/led_on"
LENS_PATH = f"{m.ENV_PATH}/control/frame/visual/led_lens"
FLAME_PATHS = (f"{m.ENV_PATH}/heater/visual/flame", f"{m.ENV_PATH}/heater/visual/flame_tip")


class GasControl:
    """What the real control and pilot do, as their makers document it. Its state is the device's, not the scorer's: the scorer reads it, and the
    demo reads only what it shows (whether the status light is lit). The flame behind the sight glass is drawn while the pilot burns."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.t = 0.0
        self.lit = False
        self.lit_ever = False
        self.charge = 0.0  # thermopile warmth: 1 = hot enough for the control to hold the pilot valve
        self.held = False  # the control holds the pilot valve open (status light blinking)
        self.held_since = 0.0
        self.lockout_until = -1.0
        self.hand_open = False  # the knob held in far enough to open the pilot valve
        self.unlit_gas_s = 0.0  # seconds of pilot gas flowing with no flame
        self.armed = True
        self.snapped = False
        self.sparks = 0
        self.outs = 0
        self.locks = {"plunger": None, "knob": None}
        self.detent = None
        self.led = None
        self.flame = None
        if env is not None:
            m.set_target(env, "control", "plunger", KNOB_PRE)
            m.set_target(env, "control", "igniter", IGN_PRE)
            m.set_target(env, "control", "knob", 0.0)
            self._show(env, force=True)

    def _lock(self, env, link, on, rest):
        if self.locks[link] == on:
            return
        self.locks[link] = on
        if on:
            m.lock(env, "control", link)
        else:
            m.unlock(env, "control", link)
            m.set_target(env, "control", link, rest)
            if link == "knob":
                self.detent = None

    def step(self, env):
        dt = float(env.unwrapped.step_dt)
        s = m.state(env, "control")
        ang, push, ign = s.deg("knob"), s.joint("plunger"), s.joint("igniter")
        at_pilot = abs(ang - PILOT) <= PILOT_WIN
        # interlocks: the knob goes in only at PILOT; pushed in, it does not turn
        self._lock(env, "plunger", (not at_pilot) and push < 0.0005, KNOB_PRE)
        if push > 0.001:
            self._lock(env, "knob", True, 0.0)
        elif push < 0.0005:
            self._lock(env, "knob", False, math.radians(ang))
        if not self.locks["knob"]:  # light detents at OFF and PILOT; elsewhere the knob stays where it is turned
            want = "OFF" if abs(ang - DIAL["OFF"]) < DETENT_WIN else ("PILOT" if abs(ang - PILOT) < DETENT_WIN else None)
            target = math.radians(DIAL[want]) if want else math.radians(ang)
            if want != self.detent or want is None:
                m.set_target(env, "control", "knob", target)
                self.detent = want
        # the pilot valve: held open by the knob (all the way in, at PILOT), or by the control once its thermopile is hot; shut at OFF
        if at_pilot and push >= PUSH_OPEN:
            self.hand_open = True
        elif push < PUSH_SHUT or not at_pilot:
            self.hand_open = False
        knob_off = ang < (DIAL["OFF"] + PILOT) / 2
        if knob_off:
            self.held = False
        valve = (self.hand_open or self.held) and not knob_off
        # the igniter: a full press snaps (the hammer lets go, so the button's force drops) and sparks once; let out, it springs back and re-arms
        spark = False
        if self.armed and ign >= IGN_SNAP:  # the hammer lets go: one spark, and the button's force drops
            spark, self.armed, self.snapped = True, False, True
            self.sparks += 1
            m.set_target(env, "control", "igniter", IGN_AFTER)
        elif not self.armed:
            if self.snapped and ign < IGN_SNAP - 0.0005:  # let out: the return spring pushes the button back and cocks the hammer
                self.snapped = False
                m.set_target(env, "control", "igniter", IGN_PRE)
            if ign < IGN_REARM:
                self.armed = True
        if spark and valve and not self.lit:
            self.lit = self.lit_ever = True
        if self.lit and not valve:  # gas shut off: the pilot goes out, and the control will not hold again for 10 minutes
            self.lit, self.held = False, False
            self.lockout_until = self.t + LOCKOUT_S
            self.outs += 1
        if valve and not self.lit:
            self.unlit_gas_s += dt
        self.charge = min(1.0, self.charge + dt / T_WARM) if self.lit else max(0.0, self.charge - dt / T_COOL)
        if self.lit and self.charge >= 1.0 and not self.held and self.t >= self.lockout_until:
            self.held, self.held_since = True, self.t
        self.t += dt
        self._show(env)

    def led_on(self) -> bool:
        if self.held:
            return ((self.t - self.held_since) % BLINK_PERIOD) < BLINK_ON
        return self.lit_ever and not self.lit and self.charge > 0.02  # solid while the thermopile cools after the pilot went out

    def _show(self, env, force=False):
        led, flame = self.led_on(), self.lit
        if force or led != self.led:
            m.set_visible(LED_PATH, led)
            m.set_visible(LENS_PATH, not led)
            self.led = led
        if force or flame != self.flame:
            for p in FLAME_PATHS:
                m.set_visible(p, flame)
            self.flame = flame


GAS = GasControl()
m.on_step(GAS.step)
m.on_reset(GAS.reset)


# -- the scorer ---------------------------------------------------------------------------------------------------------------------------------------

def _keep_out_points():
    """Points on the knob (its front, rim and axis) and on the igniter's button, in the control's frame at rest."""
    pts = [[-KNOB_BODY - KNOB_RIDGE, 0.0, 0.0], [-KNOB_BODY / 2, 0.0, 0.0]]
    for a in np.linspace(0.0, 2 * math.pi, 8, endpoint=False):
        pts.append([-KNOB_BODY, KNOB_R * math.cos(a), KNOB_R * math.sin(a)])
    pts.append([-IGN_PROUD, IGN[1], IGN[2]])
    return np.array(pts)


KEEP_OUT = _keep_out_points()


def hands_clear(env) -> bool:
    tips = m.fingertips(env)
    pts = m.state(env, "control").to_world(KEEP_OUT)
    return bool(np.min(np.linalg.norm(tips[:, None, :] - pts[None], axis=2)) > CLEAR)


def moment(ang, push, gas) -> float:
    """The score for the scene as it is now, before the 5 s hold and with no recorded violation: see task.json's scoring."""
    if gas.lit and gas.held and push < KNOB_OUT:  # the pilot burns, held by the control, the knob let out
        return 0.5 if ang <= HOT + HOT_TOL else 0.25  # at HOT or cooler (PILOT, VAC, LOW); hotter (A to VERY HOT) is a scald risk
    return 0.25 if gas.lit_ever else 0.0


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.violation = False

    def step(self, env):
        s = m.state(env, "control")
        ang, push = s.deg("knob"), s.joint("plunger")
        clear = hands_clear(env)
        if GAS.unlit_gas_s > UNLIT_LIMIT:
            self.violation = True
        ok = (not self.violation) and GAS.lit and GAS.held and push < KNOB_OUT and clear and abs(ang - HOT) <= HOT_TOL
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.score = 0.0 if self.violation else (1.0 if self.full else moment(ang, push, GAS))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration ----------------------------------------------------------------------------------------------------------------------------------

def shown(path) -> bool:
    """Whether a prim is drawn now: what the cameras show (the status light on the control's face)."""
    import isaaclab.sim as sim_utils
    from pxr import UsdGeom

    prim = sim_utils.get_current_stage().GetPrimAtPath(path)
    return UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible


DEMO_MODES = ("full", "none", "skip", "onearm", "half", "early", "nospark", "holdon", "stay", "hotter", "handon")


def demo_script(ctx):
    """The reference: the label's lighting steps, with the arms split as a person splits their hands.

    Left: pinches the knob's ridge and turns it from OFF to PILOT, then comes down onto the ridge with closed fingertips (tilted 60 degrees from
    straight down) and pushes the knob all the way in, and keeps it in, closing the loop on how far in the knob is. Right: closed fingertips, level,
    clicks the igniter about once a second, each press closed on the button's travel until it snaps, until the status light blinks. Then the right
    hand leaves, the left lets the knob out slowly and backs off; the light keeps blinking; the left pinches the ridge again and turns the knob to
    HOT, and leaves.
    It reads what the cameras could show (the status light's state, the knob's angle and travel, the igniter's travel) and the knob's place from the
    published layout. It never reads the gas model or the scorer.
    Failure checks (--mode): none does nothing; skip turns the knob straight to HOT; onearm uses the left arm alone (knob in, let go, then clicks);
    half holds the knob in only 4 mm while clicking; early lets the knob out after 5 clicks; nospark holds the knob in 92 s before the first click,
    then finishes; holdon never lets the knob out; stay leaves the knob at PILOT; hotter turns on past HOT to A; handon sets HOT and keeps hold of the ridge."""
    import demokit as d
    from yam_arena.urdf import axis_angle_matrix

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in DEMO_MODES:
        raise ValueError(mode)
    for _ in range(10):
        yield
    if mode == "none":
        for _ in range(60):
            yield
        return
    K = CTRL.copy()  # the knob's axis on the cover's face
    X = np.array([1.0, 0.0, 0.0])
    ign = CTRL + IGN
    ign_face = np.array([ign[0] - IGN_PROUD, ign[1], ign[2]])

    def knob_deg():
        return ctx.deg("control", "knob")

    def push_mm():
        return ctx.joint("control", "plunger") * 1000.0

    def ign_mm():
        return ctx.joint("control", "igniter") * 1000.0

    def mid(side):
        """The middle of an arm's two fingertips."""
        p = m.fingertips(env)[:12] if side == "left" else m.fingertips(env)[12:]
        return 0.5 * (p[2] + p[8])

    def status():
        return shown(LED_PATH)

    def rolled(deg):
        return d._as_grip(axis_angle_matrix(X, math.radians(deg)) @ d.level(0).rot)

    def pinch_and_turn(to_deg, label, leave=True):
        """Level, fingers either side of the ridge as it now stands; pinch it and turn the hand about the knob's axis."""
        now = knob_deg()
        g = rolled(now - PILOT)  # level(0) straddles the ridge when it stands upright (at PILOT)
        tip_x = K[0] - KNOB_BODY - KNOB_RIDGE + 0.0035  # pad tips 3.5 mm over the ridge, short of the knob's face
        tcp = np.array([tip_x - 0.013, K[1], K[2]])
        yield from l.move(tcp - 0.05 * X, grip=g, finger=-0.010, t=2.5, label=f"{label}: in front of the knob")
        yield from l.move(tcp, t=1.0, label=f"{label}: fingers either side of the ridge")
        yield from l.close(RIDGE_W, 20.0, t=0.5, label=f"{label}: pinch the ridge")
        yield from l.arc(tcp, X, to_deg - now, t=max(1.0, abs(to_deg - now) / 30.0), label=f"{label}: turn")
        for _ in range(6):
            yield
        err = to_deg - knob_deg()
        if abs(err) > 2.0:
            yield from l.arc(tcp, X, err, t=0.8, label=f"{label}: trim")
        if leave:
            yield from l.move(l.at(), finger=-0.010, t=0.4, label=f"{label}: let go of the ridge")
            yield from l.move(l.at() - 0.05 * X, t=1.0, label=f"{label}: back off")

    if mode == "skip":
        yield from pinch_and_turn(HOT, "the knob straight to HOT")
        yield from l.move(l.at() + np.array([-0.06, 0.08, 0.06]), t=1.5, label="left clear")
        for _ in range(140):
            yield
        return

    # 1. the knob from OFF to PILOT
    yield from pinch_and_turn(PILOT, "OFF to PILOT")

    # 2. the left hand pushes the knob in with closed fingertips on its ridge, from above, and keeps it in (closed loop on the knob's travel)
    gl = d.tilted(60.0, 0.0)
    al = gl.rot[:, 2]
    ridge_x = K[0] - KNOB_BODY - KNOB_RIDGE
    goal = np.array([ridge_x - 0.004, K[1], K[2] + 0.006])  # the fingertips 4 mm short of the ridge's front, 6 mm above the axis
    yield from l.move(goal - 0.06 * al, grip=gl, finger=0.0, t=2.5, label="closed fingertips above and in front of the knob")
    for _ in range(3):
        yield from l.move(l.at() + (goal - mid("left")), t=0.8, label="onto the ridge")
        for _ in range(6):
            yield
    hold = {"base": l.at(), "inn": 0.0, "target": 4.0 if mode == "half" else PUSH * 1000.0 - 0.15, "seen": False}

    def hold_tick():
        """One step of keeping the knob in: push further while it is not as far in as wanted."""
        if hold["target"] - push_mm() > 0.1:
            hold["inn"] = min(hold["inn"] + 0.0005, 0.05)
        l._solve(hold["base"] + hold["inn"] * X, gl.rot)
        hold["seen"] = hold["seen"] or status()

    def holding(gen):
        """Run another primitive (the right hand's) while the left keeps the knob in."""
        for _ in gen:
            hold_tick()
            yield

    def hold_for(seconds, until=None):
        for _ in range(max(1, round(seconds * 20))):
            hold_tick()
            yield
            if until is not None and until():
                return True
        return False

    yield from hold_for(4.0, until=lambda: push_mm() >= hold["target"])
    print(f"[demo] knob in {push_mm():.2f} mm, pushing {hold['inn'] * 1000:.1f} mm past touch", flush=True)

    def let_go_left():
        """Let the knob out slowly, then back off along the fingers and away."""
        start = hold["inn"]
        for k in range(20):
            l._solve(hold["base"] + start * (1 - (k + 1) / 20) * X, gl.rot)
            yield
        yield from l.move(l.at() - 0.04 * al, t=0.8, label="the left hand off the knob")
        yield from l.move(np.array([0.20, 0.15, TOP + 0.40]), t=1.5, label="the left hand back")

    def click(arm, base, gen_wrap):
        """One click: press until the button snaps (closed on its travel), let it out until it has re-armed."""
        dd = 0.0
        for _ in range(30):
            if ign_mm() >= IGN_SNAP * 1000.0 + 0.4:
                break
            dd = min(dd + 0.003, 0.05)
            yield from gen_wrap(arm.move(base + dd * X, t=0.05, label="press the igniter"))
        yield from gen_wrap(arm.move(base + dd * X, t=0.1, label="hold the press"))
        yield from gen_wrap(arm.move(base, t=0.3, label="let the igniter out"))
        for _ in range(20):
            if ign_mm() < 1.0:
                break
            yield from gen_wrap(arm.wait(0.05))
        yield from gen_wrap(arm.wait(0.15))

    if mode == "onearm":  # the left hand alone: the knob in for 2 s, let go, then click the igniter
        yield from hold_for(2.0)
        yield from let_go_left()
        g1 = d.level(-10.0)
        a1 = g1.rot[:, 2]
        yield from l.move(ign_face - 0.07 * a1 + np.array([0.0, 0.04, 0.0]), grip=g1, finger=0.0, t=2.5, label="left in front of the igniter")
        for _ in range(3):
            yield from l.move(l.at() + (ign_face - 0.002 * X - mid("left")), t=0.8, label="left on the igniter")
        base1 = l.at()
        for _ in range(4):
            yield from click(l, base1, lambda g: g)
        yield from l.move(l.at() - 0.08 * a1, t=1.0, label="left away")
        for _ in range(60):
            yield
        return

    if mode == "nospark":
        yield from hold_for(92.0)

    # 3. the right hand: closed fingertips, level, onto the igniter's button, while the left keeps the knob in
    gr = d.level(10.0)
    ar = gr.rot[:, 2]
    yield from holding(r.move(np.array([ign_face[0] - 0.08, ign_face[1] - 0.04, ign_face[2]]), grip=gr, finger=0.0, t=2.5, label="the right hand in front of the igniter"))
    for _ in range(3):
        yield from holding(r.move(r.at() + (ign_face - 0.002 * X - mid("right")), t=0.8, label="onto the igniter"))
        yield from hold_for(0.3)
    r_base = r.at()

    # 4. click about once a second until the status light blinks (A.O. Smith: "for up to 90 seconds or until Status Light begins to blink")
    clicks = 0
    limit = 5 if mode == "early" else (12 if mode == "half" else 85)
    while clicks < limit and not hold["seen"]:
        yield from click(r, r_base, holding)
        clicks += 1
    print(f"[demo] {clicks} clicks; status light seen {hold['seen']}; knob in {push_mm():.2f} mm", flush=True)
    yield from holding(r.move(r.at() - 0.06 * ar, t=0.8, label="the right hand off the igniter"))
    yield from holding(r.move(np.array([0.20, -0.15, TOP + 0.30]), t=1.5, label="the right hand away"))
    if mode == "holdon":
        yield from hold_for(30.0)
        return

    # 5. the left hand lets the knob out; the pilot should stay lit (the light keeps blinking)
    yield from hold_for(0.5)
    yield from let_go_left()
    for _ in range(60):
        yield
    print(f"[demo] after letting go: knob in {push_mm():.2f} mm, status light now {status()}", flush=True)
    if mode == "stay":
        for _ in range(140):
            yield
        return

    # 6. the knob to HOT, then both hands clear
    yield from pinch_and_turn(HOT, "PILOT to HOT", leave=mode != "handon")
    if mode == "hotter":
        yield from pinch_and_turn(DIAL["A"], "on to A")
    print(f"[demo] knob at {knob_deg():.1f} deg", flush=True)
    if mode != "handon":
        yield from l.move(np.array([0.20, 0.15, TOP + 0.40]), grip=gl, t=2.0, label="the left hand clear")
    for _ in range(160):
        yield


def demo():
    """The same reference as `demo_script`, open loop, for the harness's own scripted player (yam_scripted): the commanded positions are the ones
    the closed-loop script settles on from the published layout (the knob pushed 36 mm past first touch, each igniter press 33 mm past it), and the
    hold is timed (44 s of clicking from the first press) instead of watching the status light."""
    import demokit as d
    from yam_arena.demo import Waypoint
    from yam_arena.urdf import axis_angle_matrix

    X = np.array([1.0, 0.0, 0.0])
    K = CTRL

    def rolled(deg):
        return d._as_grip(axis_angle_matrix(X, math.radians(deg)) @ d.level(0).rot)

    def w(t, p, finger, label, grip):
        return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, grip)

    OPEN, SHUT = -0.010, 0.0
    tcp = np.array([K[0] - KNOB_BODY - KNOB_RIDGE + 0.0035 - 0.013, K[1], K[2]])  # on the knob's axis, the pads either side of the ridge
    gl, gr = d.tilted(60.0, 0.0), d.level(10.0)
    al, ar = gl.rot[:, 2], gr.rot[:, 2]
    hold = np.array([0.2888, 0.0496, TOP + 0.3356])  # the left's command with its fingertips 4 mm short of the ridge, 6 mm above the axis
    press = np.array([0.2981, 0.0448, TOP + 0.2826])  # the right's command with its fingertips 2 mm short of the igniter's button
    clear_l, clear_r = np.array([0.20, 0.15, TOP + 0.40]), np.array([0.20, -0.15, TOP + 0.30])
    turn_to_hot = HOT - PILOT
    left = [
        w(0.5, tcp - 0.05 * X + np.array([0.0, 0.0, 0.03]), OPEN, "wait", rolled(-PILOT)),
        w(2.0, tcp - 0.05 * X, OPEN, "OFF to PILOT: in front of the knob", rolled(-PILOT)),
        w(1.0, tcp, OPEN, "fingers either side of the ridge", rolled(-PILOT)),
        w(0.5, tcp, SHUT, "pinch the ridge", rolled(-PILOT)),
        w(1.0, tcp, SHUT, "turn to PILOT", rolled(0.0)),
        w(0.3, tcp, SHUT, "turn to PILOT", rolled(0.0)),
        w(0.4, tcp, OPEN, "let go of the ridge", rolled(0.0)),
        w(1.0, tcp - 0.05 * X, OPEN, "back off", rolled(0.0)),
        w(2.5, hold - 0.06 * al, SHUT, "closed fingertips above and in front of the knob", gl),
        w(1.0, hold, SHUT, "onto the ridge", gl),
        w(3.0, hold + 0.036 * X, SHUT, "push the knob all the way in", gl),
    ]
    t_left = sum(p.duration for p in left)
    clicks, period = 40, 1.1
    right = [
        w(t_left - 3.0, np.array([0.25, -0.20, TOP + 0.30]), SHUT, "wait", gr),
        w(2.5, np.array([press[0] - 0.07, press[1] - 0.04, press[2]]), SHUT, "the right hand in front of the igniter", gr),
        w(1.5, press, SHUT, "onto the igniter", gr),
    ]
    for _ in range(clicks):
        right += [w(0.5, press + 0.033 * X, SHUT, "press the igniter", gr), w(0.1, press + 0.033 * X, SHUT, "hold the press", gr),
                  w(0.3, press, SHUT, "let the igniter out", gr), w(period - 0.9, press, SHUT, "let the igniter out", gr)]
    right += [w(0.8, press - 0.06 * ar, SHUT, "the right hand off the igniter", gr), w(1.5, clear_r, SHUT, "the right hand away", gr)]
    t_right = sum(p.duration for p in right)
    left += [
        w(t_right - t_left + 0.5, hold + 0.036 * X, SHUT, "hold the knob in", gl),
        w(1.0, hold, SHUT, "let the knob out", gl),
        w(0.8, hold - 0.04 * al, SHUT, "the left hand off the knob", gl),
        w(1.5, clear_l, SHUT, "the left hand back", gl),
        w(3.0, clear_l, SHUT, "watch the status light", gl),
        w(2.5, tcp - 0.05 * X, OPEN, "PILOT to HOT: in front of the knob", rolled(0.0)),
        w(1.0, tcp, OPEN, "fingers either side of the ridge", rolled(0.0)),
        w(0.5, tcp, SHUT, "pinch the ridge", rolled(0.0)),
        w(turn_to_hot / 30.0, tcp, SHUT, "turn to HOT", rolled(turn_to_hot)),
        w(0.3, tcp, SHUT, "turn to HOT", rolled(turn_to_hot)),
        w(0.4, tcp, OPEN, "let go of the ridge", rolled(turn_to_hot)),
        w(1.0, tcp - 0.05 * X, OPEN, "back off", rolled(turn_to_hot)),
        w(2.0, clear_l, OPEN, "the left hand clear", gl),
        w(8.0, clear_l, OPEN, "hands clear", gl),
    ]
    right += [w(sum(p.duration for p in left) - t_right, clear_r, SHUT, "hands clear", gr)]
    return {"left": left, "right": right}
