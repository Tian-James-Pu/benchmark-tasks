"""Seat a Mini-ITX server motherboard in a 1U server chassis: on its four standoffs, rear I/O through the I/O shield.

The scene: an open-top 1U Mini-ITX server chassis (437 x 249 x 43 mm outside, as Supermicro's SC505-203B), its four board standoffs
(6-32 hex, 6.35 mm tall) on the Mini-ITX hole pattern at the rear left of the tray, its 1U power supply (192 x 76 x 40.3 mm, as the
chassis' PWS-203-1H) beside the board bay, and its full-height rear panel with the I/O shield. In front of it on the bench, a Mini-ITX
server board (170 x 170 mm, as Supermicro's X11SCL-iF, two DDR4 UDIMMs fitted) lies on a sheet of anti-static foam.

The step modelled: "lay the motherboard in the chassis, aligning it with the standoffs", before the screws go in. The screws are
outside the trial.

What makes it a manipulation: the board's two D-sub connectors (VGA and COM) stand 6 mm proud of their flanges and pass through
cut-outs in the I/O shield 1.0 mm larger all round. The rear panel is full height, so the board cannot be lowered straight onto its
standoffs: it has to come in a little forward of its place (or tilted, I/O edge first) and go rearward until the connector faces meet
the shield, with the D-sub shells in their cut-outs. Lowered straight down, the shells land on the shield's lower lip. Once the
connector faces are against the shield and the shells are in their cut-outs, the four mounting holes lie over the four standoffs,
which is how the real shield locates a board.

Model (declared in code):
- The board is one rigid body: the FR-4 board with its connectors, sockets and the two fitted memory modules. Mass and centre of mass
  are estimates. The standoffs are plain-topped (female thread), as in the chassis; nothing enters the board's holes until the screws
  do, so "engaged" means a hole sitting on its standoff, concentric with it. Small headers, capacitors and the shield's grounding
  fingers are not modelled. The power supply is a fixed block; the drive bracket is not fitted.
- Engaged (each of the four holes): the hole's centre within 0.6 mm (horizontal) of its standoff's axis, so the standoff's 6-32 tapped
  bore (about 2.7 mm) shows whole through the 3.96 mm hole; and the board's underside there resting on the standoff top: no more than
  0.3 mm above it (and no more than 0.5 mm into it, the solver's contact allowance).
- Dropped (scored 0): the board falling faster than 0.5 m/s at any time. Latched in Trial.step, cleared only in Trial.reset (the
  mechkit on_reset hook).
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
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Install the motherboard in the server chassis, seated on its standoffs ready for its screws."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# ---- the chassis (world-aligned): rear panel inner face at RW_X, the rear panel along y; the board bay at the rear left
RW_X = 0.480  # the rear panel's (and the I/O shield's) inner face
WALL_T = 0.001  # rear panel and shield thickness (ATX 2.2: a back panel the shield clips into is 0.94 to 1.32 mm)
FLOOR_T = 0.001
CH_D, CH_W, CH_H = 0.249, 0.437, 0.043  # SC505-203B: 437 x 43 x 249 mm (W x H x D)
LEFT_IN = 0.130  # the left side wall's inner face (estimate: the board's left edge 45 mm in from it)
CH_X0 = RW_X + WALL_T - CH_D  # the front panel's outer face
CH_Y0 = LEFT_IN + WALL_T - CH_W  # the right side wall's outer face
SO_H, SO_R = 0.00635, 0.0024  # standoffs: ATX 2.2 needs >= 0.250 in under the board; 3/16 in hex (drawn as a round of 4.8 mm)

# ---- the board (frame: origin at the centre of the board's underside, +x toward the rear I/O edge, +y toward the PCIe slot)
PCB = 0.00157  # ATX 2.2: a typical 0.062 in board
BOARD_MASS = 0.50  # estimate: board, two DIMMs, socket cover
COM = (0.012, -0.006, 0.008)  # estimate: the I/O block and the memory pull it rearward and toward the DIMM side
FACE_X = 0.07484 + 0.01130  # ATX 2.2: connector faces 0.445 in behind the datum hole C, which is 0.400 in from the rear edge
NOSE_L = 0.006  # D-sub shell standing proud of its flange (approximate)
NOSE_W, NOSE_H, NOSE_ZC = 0.0169, 0.0084, 0.00625  # D-sub shell size 1 / DE-15 and DE-9 (approximate); centre above the board top
FLANGE_H = 0.0125
CUT = 0.0010  # shield cut-out clearance all round a shell (approximate)
STACK_CUT = 0.0005  # shield opening clearance round the USB, LAN and IPMI port stacks (approximate)
HOLES = {  # mounting holes (ATX/Mini-ITX: C 0.400 in from the rear edge, F 0.9 in forward of C, H and J 6.1 in forward of C; 6.2 in across)
    "C": (0.07484, 0.07865),
    "F": (0.05198, -0.07883),
    "H": (-0.08010, 0.07865),
    "J": (-0.08010, -0.07883),
}
HOLE_D = 0.00396
# rear I/O, from the X11SCL-iF quick reference layout (y ranges, depth into the board, height above the board top); approximate
VGA_Y, VGA_NOSE_Y = (0.0301, 0.0616), (0.0373, 0.0541)
COM_Y, COM_NOSE_Y = (-0.0796, -0.0481), (-0.0724, -0.0553)
STACKS = [("lan", (-0.0094, 0.0077), 0.021, 0.026), ("usb", (-0.0265, -0.0121), 0.017, 0.016), ("ipmi", (-0.0476, -0.0280), 0.021, 0.026)]
PCIE_X, PCIE_YC, PCIE_W, PCIE_H = (-0.047, 0.042), 0.0769, 0.0072, 0.0112  # PCIe x16 slot, 89 mm long
DIMM_X, DIMM_Y, DIMM_H = (-0.083, 0.058), (-0.0656, -0.0477), 0.032  # two slots with DDR4 UDIMMs fitted
ATX24_X, ATX24_Y, ATX24_H = (-0.005, 0.047), (-0.0841, -0.0733), 0.013
SOCKET_C, SOCKET_S = (-0.007, -0.003), (0.054, 0.054, 0.007)
PCH_C, PCH_S = (-0.060, -0.003), (0.025, 0.025, 0.003)

# seated: connector faces against the shield's inner face
BX, BY = RW_X - FACE_X, 0.0
Z_SEAT = TOP + FLOOR_T + SO_H  # the board's underside on the standoffs
# start: on the anti-static foam in front of the chassis
FOAM = (0.180, 0.200, 0.006)
FOAM_XY = (0.142, 0.0)
SX, SY = 0.142, 0.0
Z_START = TOP + FOAM[2]

LOOKS = {
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.64), metallic=0.8, roughness=0.45),
    "shield": m.Look(color=m.rgb(0.78, 0.79, 0.80), metallic=0.9, roughness=0.3),
    "brass": m.Look(color=m.rgb(0.80, 0.66, 0.30), metallic=0.9, roughness=0.35),
    "pcb": m.Look(color=m.rgb(0.10, 0.30, 0.16), roughness=0.5),
    "hole": m.Look(color=m.rgb(0.03, 0.03, 0.03), roughness=0.9),
    "pad": m.Look(color=m.rgb(0.85, 0.70, 0.35), metallic=1.0, roughness=0.3),
    "blue": m.Look(color=m.rgb(0.05, 0.20, 0.60), roughness=0.5),
    "black": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.6),
    "white": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.6),
    "dimm": m.Look(color=m.rgb(0.08, 0.35, 0.18), roughness=0.5),
    "chip": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.4),
    "metal": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=0.9, roughness=0.3),
    "psu": m.Look(color=m.rgb(0.35, 0.36, 0.37), metallic=0.7, roughness=0.5),
    "foam": m.Look(color=m.rgb(0.85, 0.45, 0.55), roughness=0.95),
}


def _bx(lo, hi):
    """A box from its low and high corners: (size, centre)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return tuple(hi - lo), tuple((hi + lo) / 2)


def _board_pieces():
    """(name, size, centre, look) for every solid piece of the board, in the board's frame."""
    t = PCB
    p = [("pcb", (0.170, 0.170, t), (0.0, 0.0, t / 2), "pcb")]
    for nm, ys, ns in (("vga", VGA_Y, VGA_NOSE_Y), ("com", COM_Y, COM_NOSE_Y)):
        p.append((f"{nm}_body", *_bx((FACE_X - 0.012, ys[0], t), (FACE_X, ys[1], t + FLANGE_H)), "blue"))
        p.append((f"{nm}_nose", *_bx((FACE_X, ns[0], t + NOSE_ZC - NOSE_H / 2), (FACE_X + NOSE_L, ns[1], t + NOSE_ZC + NOSE_H / 2)), "metal"))
    for nm, ys, depth, h in STACKS:
        p.append((nm, *_bx((FACE_X - depth, ys[0], t), (FACE_X, ys[1], t + h)), "metal"))
    p.append(("pcie", *_bx((PCIE_X[0], PCIE_YC - PCIE_W / 2, t), (PCIE_X[1], PCIE_YC + PCIE_W / 2, t + PCIE_H)), "black"))
    p.append(("dimms", *_bx((DIMM_X[0], DIMM_Y[0], t), (DIMM_X[1], DIMM_Y[1], t + DIMM_H)), "dimm"))
    p.append(("atx24", *_bx((ATX24_X[0], ATX24_Y[0], t), (ATX24_X[1], ATX24_Y[1], t + ATX24_H)), "white"))
    p.append(("socket", SOCKET_S, (SOCKET_C[0], SOCKET_C[1], t + SOCKET_S[2] / 2), "metal"))
    p.append(("pch", PCH_S, (PCH_C[0], PCH_C[1], t + PCH_S[2] / 2), "chip"))
    return p


def _board():
    parts, cols = [], []
    for nm, size, c, look in _board_pieces():
        parts.append((nm, m.box(size, c), look))
        cols.append(m.col_box(f"{nm}_c", size, c))
    # the DIMM pair drawn as two modules in their slots, over the collider
    for k, yc in enumerate((-0.0517, -0.0616)):
        parts.append((f"slot{k}", m.box((DIMM_X[1] - DIMM_X[0], 0.0065, 0.0055), ((DIMM_X[0] + DIMM_X[1]) / 2, yc, PCB + 0.00275)), "blue"))
    for nm, (hx, hy) in HOLES.items():
        parts.append((f"pad{nm}", m.cylinder(0.0035, 0.0002, (hx, hy, PCB + 0.0001)), "pad"))
        parts.append((f"hole{nm}", m.cylinder(HOLE_D / 2, 0.0003, (hx, hy, PCB + 0.00015)), "hole"))
    return m.free("board", (SX, SY, Z_START), parts, cols, LOOKS, mass=BOARD_MASS, friction=(0.6, 0.5), com=COM, solver_iters=48)


def _chassis():
    parts, cols = [], []

    def add(nm, lo, hi, look="steel"):
        size, c = _bx(lo, hi)
        parts.append((nm, m.box(size, c), look))
        cols.append(m.col_box(f"{nm}_c", size, c))

    x0, x1, y0, y1 = CH_X0, RW_X + WALL_T, CH_Y0, LEFT_IN + WALL_T
    add("floor", (x0, y0, 0.0), (x1, y1, FLOOR_T))
    add("front", (x0, y0, 0.0), (x0 + WALL_T, y1, CH_H))
    add("left", (x0, LEFT_IN, 0.0), (x1, y1, CH_H))
    add("right", (x0, y0, 0.0), (x1, y0 + WALL_T, CH_H))
    # the rear panel, with the I/O shield over the board's I/O: a cut-out round each D-sub shell (which passes through it) and an
    # opening in front of each port stack (whose face sits flush with the shield's inner face)
    zb = FLOOR_T + SO_H + PCB  # the board's top surface when seated
    zlo, zhi = zb + NOSE_ZC - NOSE_H / 2 - CUT, zb + NOSE_ZC + NOSE_H / 2 + CUT
    openings = [(BY + COM_NOSE_Y[0] - CUT, BY + COM_NOSE_Y[1] + CUT, zhi), (BY + VGA_NOSE_Y[0] - CUT, BY + VGA_NOSE_Y[1] + CUT, zhi)]
    openings += [(BY + ys[0] - STACK_CUT, BY + ys[1] + STACK_CUT, zb + h + STACK_CUT) for _, ys, _, h in STACKS]
    openings.sort()
    sh_lo, sh_hi = BY - 0.0845, BY + 0.0700  # the shield's extent across the panel
    add("rear_r", (RW_X, y0, 0.0), (x1, sh_lo, CH_H))
    add("rear_l", (RW_X, sh_hi, 0.0), (x1, y1, CH_H))
    add("shield_lo", (RW_X, sh_lo, 0.0), (x1, sh_hi, zlo), "shield")
    y = sh_lo
    for k, (ya, yb, ztop) in enumerate(openings):
        add(f"shield_bar{k}", (RW_X, y, zlo), (x1, ya, CH_H), "shield")  # full height between openings
        add(f"shield_top{k}", (RW_X, ya, ztop), (x1, yb, CH_H), "shield")  # above the opening
        y = yb
    add("shield_bar_end", (RW_X, y, zlo), (x1, sh_hi, CH_H), "shield")
    # the power supply (PWS-203-1H, 192 x 76 x 40.3 mm), its rear face at the rear panel, 34 mm to the right of the board bay (estimate)
    add("psu", (RW_X - 0.192, BY - 0.085 - 0.034 - 0.076, FLOOR_T), (RW_X, BY - 0.085 - 0.034, FLOOR_T + 0.0403), "psu")
    # the four standoffs
    for nm, (hx, hy) in HOLES.items():
        c = (BX + hx, BY + hy, FLOOR_T + SO_H / 2)
        parts.append((f"so{nm}", m.cylinder(SO_R, SO_H, c, segments=6), "brass"))
        cols.append(m.col_cylinder(f"so{nm}_c", SO_R, SO_H, c))
    return m.fixed("chassis", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.6, 0.5))


def _foam():
    size = FOAM
    c = (0.0, 0.0, size[2] / 2)
    return m.fixed("foam", (FOAM_XY[0], FOAM_XY[1], TOP), [("sheet", m.box(size, c), "foam")], [m.col_box("sheet_c", size, c)], LOOKS,
                   friction=(1.0, 0.9))


OBJECTS = [
    _chassis(),
    _foam(),
    _board(),
    # distractors (scoring ignores them)
    SimReady("screwdriver", pos=(0.12, -0.20, TOP), size=(0.188, 0.034, 0.032), mass=0.10, convex=True,
             package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd"),
    SimReady("tape_roll", pos=(0.13, 0.21, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
             package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd"),
    SimReady("mug", pos=(0.42, 0.23, TOP), size=(0.114, 0.090, 0.080), mass=0.30, yaw=-20.0,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
]

# ---- scoring ------------------------------------------------------------------------------------------------------------------------------
ENGAGE_XY = 0.0006
ENGAGE_Z = 0.0003
ENGAGE_PEN = 0.0005
BAY_XY = 0.020
BAY_Z = 0.003
BAY_TILT = 5.0
CLEAR = 0.03
_HOLE_L = np.array([[hx, hy, 0.0] for hx, hy in HOLES.values()])
_SO_TOP = np.array([[BX + hx, BY + hy, Z_SEAT] for hx, hy in HOLES.values()])
_BB_LO = np.array([-0.085, -0.085, 0.0])
_BB_HI = np.array([FACE_X + NOSE_L, 0.085, PCB + DIMM_H])


def board_check(pos, quat_xyzw, lin_vel, ang_vel, tips_local=None):
    """The end-state checks from the board's pose (and the fingertip points in the board's frame, if known)."""
    rot = m.quat_matrix(quat_xyzw)
    holes = (rot @ _HOLE_L.T).T + np.asarray(pos)
    dxy = np.linalg.norm(holes[:, :2] - _SO_TOP[:, :2], axis=1)
    dz = holes[:, 2] - _SO_TOP[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, rot[2, 2]))))
    engaged = (dxy <= ENGAGE_XY) & (dz <= ENGAGE_Z) & (dz >= -ENGAGE_PEN)
    centre = float(np.linalg.norm(np.asarray(pos[:2]) - np.array([BX, BY])))
    in_bay = centre <= BAY_XY and tilt <= BAY_TILT and bool(np.all(np.abs(dz) <= BAY_Z))
    still = float(np.linalg.norm(lin_vel)) < 0.005 and float(np.linalg.norm(ang_vel)) < 0.05
    clear = None
    if tips_local is not None:
        d = np.maximum(np.maximum(_BB_LO - tips_local, tips_local - _BB_HI), 0.0)
        clear = float(np.linalg.norm(d, axis=1).min()) >= CLEAR
    return dict(engaged=engaged, n=int(engaged.sum()), dxy=dxy, dz=dz, tilt=tilt, in_bay=in_bay, still=still, clear=clear)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.dropped = False
        self.hold, self.full, self.score = 0, False, 0.0
        self.last = None

    def step(self, env):
        b = m.state(env, "board")
        if float(b.lin_vel[2]) < -0.5:
            self.dropped = True
        tips = b.to_local(m.fingertips(env))
        c = board_check(b.pos, b.quat_xyzw, b.lin_vel, b.ang_vel, tips)
        self.last = c
        seated = c["n"] == 4
        ok = seated and c["clear"] and c["still"] and not self.dropped
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS  # current: drops again if the state stops holding
        if self.dropped:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif seated:
            self.score = 0.5
        elif c["in_bay"]:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# ---- the scripted demo: a fixed plan from the published starting layout -----------------------------------------------------------------
TIP = 0.013  # pad tips below the TCP
L_GX, R_GX = COM[0] + 0.015, COM[0] - 0.035  # left hand on the slot's rear half, right hand on the modules' front half: the right
# hand's fingers on the modules' rear half would meet the rear panel as the board goes home
L_Y = PCIE_YC  # left hand: across the PCIe slot
R_Y = (DIMM_Y[0] + DIMM_Y[1]) / 2  # right hand: across the two memory modules
L_DZ = PCB + 0.002 + TIP  # TCP above the board's underside: pads 2 mm above the board, on the slot's sides
R_DZ = PCB + DIMM_H - 0.009 + TIP  # pads on the modules' upper part, clear of the 24-pin connector beside them
L_PRE, R_PRE = -0.011, -0.016  # openings before going down: about 22 mm and 32 mm
LIFT = TOP + CH_H + 0.020  # the board's underside while it is carried over the chassis wall
BACK = 0.0070  # set down this far forward of its place (the D-sub shells 1 mm short of the shield, the rear holes still over their standoffs), then slid rearward
COMP_L, COMP_R = (0.0051, -0.0004), (0.0032, 0.0003)  # measured in the simulator: lowered into the bay, the board sits 4 mm short of
# the hands' targets and turned 0.8 degrees anticlockwise; each hand's target is moved, and both grips turned, to take that out
COMP_YAW = math.radians(-0.7)
PUSH = 0.0030  # slid this far past its place, so the connector faces meet the shield
PRESS = 0.0010  # the hands aim the board this far above the standoff tops: under its weight it sags onto them


def _hands(bx, by, bz, fl, fr, label, t):
    """Both hands for a board origin at (bx, by, bz)."""
    lw = Waypoint(t, (bx + L_GX, by + L_Y, bz + L_DZ), fl, f"left: {label}")
    rw = Waypoint(t, (bx + R_GX, by + R_Y, bz + R_DZ), fr, f"right: {label}")
    return lw, rw


def _plan(mode: str = "full"):
    rest_l = Waypoint(1.5, (0.25, 0.25, TOP + 0.20), FINGER_OPEN, "left: away")
    rest_r = Waypoint(1.5, (0.25, -0.25, TOP + 0.20), FINGER_OPEN, "right: away")
    if mode == "none":
        return {"left": [Waypoint(30.0, (0.25, 0.25, TOP + 0.20), FINGER_CLOSED, "left: rest")],
                "right": [Waypoint(30.0, (0.25, -0.25, TOP + 0.20), FINGER_CLOSED, "right: rest")]}
    seq = []

    def go(bx, by, bz, fl, fr, label, t, comp=False):
        lw, rw = _hands(bx, by, bz, fl, fr, label, t)
        if comp:  # the arms' measured lag under the board's weight in the bay, taken out of the plan
            lw = Waypoint(t, (lw.pos[0] + COMP_L[0], lw.pos[1] + COMP_L[1], lw.pos[2]), fl, lw.label, COMP_YAW)
            rw = Waypoint(t, (rw.pos[0] + COMP_R[0], rw.pos[1] + COMP_R[1], rw.pos[2]), fr, rw.label, COMP_YAW)
        seq.append((lw, rw))

    hi = Z_START + 0.06
    go(SX, SY, hi, FINGER_OPEN, FINGER_OPEN, "over the board", 2.0)
    go(SX, SY, hi, L_PRE, R_PRE, "open to the slot and the modules", 0.8)
    go(SX, SY, Z_START + 0.01, L_PRE, R_PRE, "down", 1.5)
    go(SX, SY, Z_START, L_PRE, R_PRE, "down beside them", 0.8)
    go(SX, SY, Z_START, FINGER_CLOSED, FINGER_CLOSED, "grip", 0.8)
    go(SX, SY, Z_START, FINGER_CLOSED, FINGER_CLOSED, "settle the grip", 0.4)
    go(SX, SY, LIFT, FINGER_CLOSED, FINGER_CLOSED, "lift it level", 1.5)
    if mode == "careless":  # straight down onto its place, as if there were no rear panel
        go(BX, BY, LIFT, FINGER_CLOSED, FINGER_CLOSED, "carry over its place", 2.5)
        go(BX, BY, Z_SEAT - PRESS, FINGER_CLOSED, FINGER_CLOSED, "straight down", 2.0)
        go(BX, BY, Z_SEAT - PRESS, FINGER_CLOSED, FINGER_CLOSED, "hold", 0.5)
    else:
        go(BX - BACK, BY, LIFT, FINGER_CLOSED, FINGER_CLOSED, "carry over the bay", 2.5)
        go(BX - BACK, BY, LIFT, FINGER_CLOSED, FINGER_CLOSED, "steady", 0.8)
        go(BX - BACK, BY, Z_SEAT + 0.006, FINGER_CLOSED, FINGER_CLOSED, "lower into the bay", 1.5, True)
        go(BX - BACK, BY, Z_SEAT + 0.006, FINGER_CLOSED, FINGER_CLOSED, "steady", 0.5, True)
        if mode == "high":  # slid rearward held above the standoffs (aimed 6 mm up)
            go(BX - BACK, BY, Z_SEAT + 0.006, FINGER_CLOSED, FINGER_CLOSED, "held above the standoffs", 1.0, True)
            go(BX + PUSH, BY, Z_SEAT + 0.006, FINGER_CLOSED, FINGER_CLOSED, "slide rearward", 2.0, True)
        else:
            go(BX - BACK, BY, Z_SEAT + PRESS, FINGER_CLOSED, FINGER_CLOSED, "set it on the standoffs", 1.0, True)
            go(BX - BACK, BY, Z_SEAT + PRESS, FINGER_CLOSED, FINGER_CLOSED, "settle", 0.4, True)
            if mode != "forward":  # red team "forward": let go where it was set down, never slid home
                go(BX + PUSH, BY, Z_SEAT + PRESS, FINGER_CLOSED, FINGER_CLOSED, "slide it rearward into the shield", 2.0, True)
        end_x = BX - BACK if mode == "forward" else BX + PUSH
        go(end_x, BY, Z_SEAT + PRESS, FINGER_CLOSED, FINGER_CLOSED, "hold", 0.4, True)
    left = [w[0] for w in seq]
    right = [w[1] for w in seq]
    if mode == "hold":
        left.append(Waypoint(30.0, left[-1].pos, FINGER_CLOSED, "left: keep holding"))
        right.append(Waypoint(30.0, right[-1].pos, FINGER_CLOSED, "right: keep holding"))
        return {"left": left, "right": right}
    if mode == "drop":  # let go over the bay, at carrying height
        return {"left": left[:9] + [Waypoint(0.6, left[8].pos, FINGER_OPEN, "left: let go"), rest_l, Waypoint(20.0, rest_l.pos, FINGER_OPEN, "left: wait")],
                "right": right[:9] + [Waypoint(0.6, right[8].pos, FINGER_OPEN, "right: let go"), rest_r, Waypoint(20.0, rest_r.pos, FINGER_OPEN, "right: wait")]}
    lp, rp = left[-1].pos, right[-1].pos
    left += [Waypoint(0.8, lp, L_PRE, "left: let go"), Waypoint(1.0, (lp[0], lp[1], lp[2] + 0.05), L_PRE, "left: up"), rest_l,
             Waypoint(20.0, rest_l.pos, FINGER_OPEN, "left: wait")]
    right += [Waypoint(0.8, rp, R_PRE, "right: let go"), Waypoint(1.0, (rp[0], rp[1], rp[2] + 0.05), R_PRE, "right: up"), rest_r,
              Waypoint(20.0, rest_r.pos, FINGER_OPEN, "right: wait")]
    if mode == "onearm":  # the right hand alone, on the modules
        left = [Waypoint(30.0, (0.25, 0.25, TOP + 0.20), FINGER_CLOSED, "left: rest")]
    return {"left": left, "right": right}


def demo():
    """A fixed plan from the published starting layout. The left hand pinches the PCIe slot and the right hand the two memory modules, both
    across the board's centre-of-mass line; they lift the board level over the chassis wall, set it flat on the four standoffs 7 mm forward
    of its place, slide it rearward on them until the D-sub shells are through the shield and the connector faces meet it, let go and
    move away."""
    return _plan("full")
