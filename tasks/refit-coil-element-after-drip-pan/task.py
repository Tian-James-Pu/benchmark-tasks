"""Refit a 6-inch plug-in coil element after its drip pan has been cleaned.

On the table: one corner of a coil-element electric cooktop (a white enamelled steel deck 4.5 cm high), with a 6-inch drip pan
seated in its round well, the pan's terminal notch facing +x towards the receptacle. The receptacle is a ceramic terminal block
under the deck at the well's +x edge, with two slots, each holding a sprung contact leaf. Beside the cooktop, lying on a folded
microfiber towel, the 6-inch plug-in element: five turns of 6.5 mm sheathed tube on a Y-frame support with three legs, a hub
medallion in the middle, and a terminal head with two blade terminals. It must go back so that its terminals are pushed fully
home in the receptacle and the coil lies flat on its legs in the pan.

Model (declared in code):
- Cooktop deck and the receptacle's ceramic block are fixed. The deck's well is a 24-sided hole; the receptacle is a mechanism
  whose frame is the block (two 3.6 x 9 mm slots) and whose links are the two contact leaves, each on a sprung slider across its
  slot, preloaded against its stop, so a blade pushed in deflects the leaf and is pinched by it.
- The drip pan is loose (porcelain-enamelled steel, 120 g): a flange resting on the deck, a sloped wall, a flat floor, and a notch
  in the wall and flange at +x for the terminal head.
- The element is one loose rigid body (350 g): tube rings as convex segments, legs, hub, head and blades as convex pieces.
- The coil does not fit in the pan while it is more than about 8 mm short of home (its far side lands on the flange or the deck),
  and the terminals sit 1 cm below the coil, so the element has to be held at slot height, pushed in with its far side riding the
  flange, and only then let down into the pan.

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ...
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
TOP = TABLE_TOP[SCENE]
MM = 0.001
HOLD_STEPS = 20  # 1 s at 20 Hz of the full-score state, counted by the tracker
SUCCESS_HOLD_S = 2.0  # then the episode ends 2 s later, so the video shows the arm leaving

# -- layout (world, m) -----------------------------------------------------------------------------------------------------------
XC, YC = float(os.environ.get("COIL_XC", "0.26")), float(os.environ.get("COIL_YC", "0.0"))  # the well's (and pan's) centre
# The well's frame ("local"): origin at the well's centre, +x along the terminals toward the receptacle, which faces away from the arms (world +x)
# Every cooktop part is built in it and turned by WELL_YAW.
WELL_YAW = float(os.environ.get("COIL_WY", "0.0"))
_C, _S = math.cos(math.radians(WELL_YAW)), math.sin(math.radians(WELL_YAW))


def to_world(lx, ly):
    """World (x, y) of a point given in the well's frame."""
    return XC + _C * lx - _S * ly, YC + _S * lx + _C * ly


def to_local(p):
    """World points (n, 3) in the well's frame."""
    p = np.atleast_2d(np.asarray(p, float))
    dx, dy = p[:, 0] - XC, p[:, 1] - YC
    return np.c_[_C * dx + _S * dy, -_S * dx + _C * dy, p[:, 2]]

DECK_H = 0.045
ZD = TOP + DECK_H  # deck top
SHEET_T = 3.0 * MM
DECK_X0, DECK_X1, DECK_Y0, DECK_Y1 = -0.125, 0.165, -0.14, 0.14  # in the well's frame (world x 0.15 to 0.43, y -0.135 to 0.155)
HOLE_R = 89.0 * MM

# drip pan (pan frame: origin at the pan's centre on the deck top)
PAN_RIM_R = 96.5 * MM  # 19.3 cm across
PAN_TOP_R = 84.0 * MM  # bowl wall at the flange
PAN_BOT_R = 64.0 * MM  # bowl wall at the floor
PAN_DEPTH = 25.0 * MM  # floor top below the deck top
PAN_T = 1.0 * MM
NOTCH_DEG = 16.0  # half-angle of the terminal notch
PAN_MASS = 0.12

# element (element frame: origin at the coil's centre, on the tubes' centre line; +x toward the terminals)
TUBE_R = 3.25 * MM
RINGS = [72.75 * MM - k * 11.0 * MM for k in range(5)]  # 15.2 cm outside diameter
COIL_OUT = RINGS[0] + TUBE_R
LEG_R = 55.0 * MM
LEG_ANG = (60.0, 180.0, 300.0)
LEG_Z = -(PAN_DEPTH - 2.0 * MM)  # leg bottom below the tube centre line, seated: -23 mm
HUB_R = 12.5 * MM  # the centre medallion, 25 mm across
HUB_Z0, HUB_Z1 = -3.0 * MM, 4.0 * MM
HEAD_X0, HEAD_X1 = 76.0 * MM, 98.0 * MM
HEAD_W = 30.0 * MM
HEAD_Z0, HEAD_Z1 = -16.0 * MM, -4.0 * MM
BLADE_Y = 8.0 * MM
BLADE_T, BLADE_H, BLADE_L = 1.6 * MM, 6.0 * MM, 24.0 * MM
BLADE_Z = -10.0 * MM
ELEM_MASS = 0.35
ELEM_COM = (6.0 * MM, 0.0, -2.0 * MM)

SEAT_Z = ZD - 2.0 * MM  # element origin height when seated (legs on the pan floor)
FACE_X = HEAD_X1  # receptacle face, well's frame: the head's shoulder bottoms here
SLOT_Z = SEAT_Z + BLADE_Z  # slot centre line
SLOT_W, SLOT_H, SLOT_D = 3.6 * MM, 9.0 * MM, 28.0 * MM
BLOCK_D, BLOCK_W = 35.0 * MM, 48.0 * MM
BLOCK_Z0, BLOCK_Z1 = ZD - 24.0 * MM, ZD - SHEET_T  # 21 mm tall
LEAF_IN = 0.4 * MM  # how far a leaf at rest stands into a blade's path
LEAF_K = 1000.0  # N/m
LEAF_PRE = 2.0 * MM  # preload: the spring's target lies this far past the leaf's stop

# start: the element on a folded towel on the table beside the cooktop
TOWEL_H = 0.034  # the folded towel's nominal height
TOWEL_REST = 0.0213  # where the element's legs settle on it (measured in the simulator: the towel's collision is lower than its 3.4 cm)
START = (0.21, 0.265, TOP + TOWEL_REST - LEG_Z + 0.001)  # terminals pointing +x, like the receptacle

LOOKS = {
    "enamel": m.Look(color=m.rgb(0.93, 0.93, 0.91), roughness=0.25),
    "pan": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.2, metallic=0.2),
    "tube": m.Look(color=m.rgb(0.22, 0.22, 0.23), metallic=0.7, roughness=0.45),
    "frame": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.35),
    "chrome": m.Look(color=m.rgb(0.85, 0.85, 0.86), metallic=1.0, roughness=0.15),
    "ceramic": m.Look(color=m.rgb(0.92, 0.90, 0.84), roughness=0.6),
    "brass": m.Look(color=m.rgb(0.78, 0.62, 0.30), metallic=0.9, roughness=0.35),
}


def hexa(c) -> m.Mesh:
    """A hexahedron from 8 corners ordered like a box: bottom four counter-clockwise, then the top four above them."""
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return m.Mesh(np.asarray(c, float), np.array(f))


def _clip(poly, x0, x1, y0, y1):
    """Sutherland-Hodgman clip of a convex polygon to a rectangle."""
    def side(pts, inside, cut):
        out = []
        for i in range(len(pts)):
            a, b = pts[i - 1], pts[i]
            ia, ib = inside(a), inside(b)
            if ib:
                if not ia:
                    out.append(cut(a, b))
                out.append(b)
            elif ia:
                out.append(cut(a, b))
        return out

    def cx(x):
        return lambda a, b: (x, a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]))

    def cy(y):
        return lambda a, b: (a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]), y)

    p = list(poly)
    p = side(p, lambda q: q[0] >= x0, cx(x0))
    p = side(p, lambda q: q[0] <= x1, cx(x1))
    p = side(p, lambda q: q[1] >= y0, cy(y0))
    p = side(p, lambda q: q[1] <= y1, cy(y1))
    return p


# -- the cooktop deck (fixed) ------------------------------------------------------------------------------------------------------
def _deck():
    parts, cols = [], []
    x0, x1, y0, y1 = DECK_X0, DECK_X1, DECK_Y0, DECK_Y1
    n = 24
    for i in range(n):
        a0, a1 = 2 * math.pi * (i - 0.5) / n, 2 * math.pi * (i + 0.5) / n
        quad = [(HOLE_R * math.cos(a0), HOLE_R * math.sin(a0)), (HOLE_R * math.cos(a1), HOLE_R * math.sin(a1)),
                (1.0 * math.cos(a1), 1.0 * math.sin(a1)), (1.0 * math.cos(a0), 1.0 * math.sin(a0))]
        poly = _clip(quad, x0, x1, y0, y1)
        mesh = m.prism(poly, -SHEET_T, 0.0)
        parts.append((f"sheet{i}", mesh, "enamel"))
        cols.append(m.col_hull(f"sheet{i}_c", mesh))
    wt, wh = 4.0 * MM, DECK_H - SHEET_T
    walls = {"wall_n": ((x1 - x0, wt, wh), ((x0 + x1) / 2, y1 - wt / 2, -SHEET_T - wh / 2)),
             "wall_s": ((x1 - x0, wt, wh), ((x0 + x1) / 2, y0 + wt / 2, -SHEET_T - wh / 2)),
             "wall_e": ((wt, y1 - y0 - 2 * wt, wh), (x1 - wt / 2, (y0 + y1) / 2, -SHEET_T - wh / 2)),
             "wall_w": ((wt, y1 - y0 - 2 * wt, wh), (x0 + wt / 2, (y0 + y1) / 2, -SHEET_T - wh / 2))}
    for k, (s, c) in walls.items():
        parts.append((k, m.box(s, c), "enamel"))
        cols.append(m.col_box(k + "_c", s, c))
    return parts, cols


_dp, _dc = _deck()
DECK = m.fixed("cooktop", (XC, YC, ZD), _dp, _dc, LOOKS, yaw_deg=WELL_YAW, friction=(0.6, 0.5))


# -- the receptacle: ceramic block (frame) with two sprung contact leaves (links) ---------------------------------------------------
# Receptacle frame: origin on the face (x = 0), on the slots' centre line (z = 0), +x into the block.
def _receptacle():
    parts, cols = [], []
    z0, z1 = BLOCK_Z0 - SLOT_Z, BLOCK_Z1 - SLOT_Z
    sz0, sz1 = -SLOT_H / 2, SLOT_H / 2
    boxes = {"below": ((BLOCK_D, BLOCK_W, sz0 - z0), (BLOCK_D / 2, 0.0, (z0 + sz0) / 2)),
             "above": ((BLOCK_D, BLOCK_W, z1 - sz1), (BLOCK_D / 2, 0.0, (z1 + sz1) / 2)),
             "back": ((BLOCK_D - SLOT_D, BLOCK_W, SLOT_H), ((SLOT_D + BLOCK_D) / 2, 0.0, 0.0))}
    edges = [-BLOCK_W / 2, -BLADE_Y - SLOT_W / 2, -BLADE_Y + SLOT_W / 2, BLADE_Y - SLOT_W / 2, BLADE_Y + SLOT_W / 2, BLOCK_W / 2]
    for k, (a, b) in enumerate(((edges[0], edges[1]), (edges[2], edges[3]), (edges[4], edges[5]))):
        boxes[f"web{k}"] = ((SLOT_D, b - a, SLOT_H), (SLOT_D / 2, (a + b) / 2, 0.0))
    for k, (s, c) in boxes.items():
        parts.append((k, m.box(s, c), "ceramic"))
        cols.append(m.col_box(k + "_c", s, c))
    # brass screw terminals on the back, where the range's wires come in (seen only)
    for s in (-1, 1):
        parts.append((f"screw{s + 1}", m.cylinder(3.0 * MM, 3.0 * MM, (BLOCK_D + 1.5 * MM, s * BLADE_Y, 0.0), axis="x", segments=12), "brass"))
    return parts, cols


def _leaf(name, yb):
    """A contact leaf on the slot's outer wall (+y side for the +y slot, -y for the -y slot), sprung across the slot."""
    s = 1.0 if yb > 0 else -1.0
    wall = yb + s * SLOT_W / 2  # the slot's outer wall
    face = yb + s * (BLADE_T / 2 - LEAF_IN)  # the leaf's contact face at rest
    x0, x1, ramp = 3.0 * MM, 25.0 * MM, 3.0 * MM
    h = 7.0 * MM
    # a wedge: flush with the wall at its mouth end, standing in to the face over `ramp`
    pts = []
    for z in (-h / 2, h / 2):
        pts += [(x0, wall, z), (x0 + ramp, face, z), (x1, face, z), (x1, wall, z)]
    pts = np.array([pts[0], pts[1], pts[2], pts[3], pts[4], pts[5], pts[6], pts[7]])
    outline = [(q[0], q[1]) for q in pts[:4]]
    mesh = m.prism(outline if s > 0 else outline[::-1], -h / 2, h / 2)
    # pushed out (away from the blade) is +y for the +y leaf: slider along Y with limits (0, 3 mm) or (-3 mm, 0)
    lo, hi = (0.0, 3.0 * MM) if s > 0 else (-3.0 * MM, 0.0)
    return m.slider(name, "Y", lo, hi, [(name + "_v", mesh, "brass")], [m.col_hull(name + "_c", mesh)], mass=0.01, spring=LEAF_K,
                    damping=1.0, friction=0.0, effort=50.0)


_rp, _rc = _receptacle()
RECEPTACLE = m.mechanism("receptacle", (*to_world(FACE_X, 0.0), SLOT_Z), [_leaf("leaf_l", BLADE_Y), _leaf("leaf_r", -BLADE_Y)], LOOKS,
                         frame_parts=_rp, frame_colliders=_rc, yaw_deg=WELL_YAW, friction=(1.0, 0.8))


# -- the drip pan (loose) ----------------------------------------------------------------------------------------------------------
def _pan():
    parts, cols = [], []
    n = 20
    a_lo, a_hi = math.radians(NOTCH_DEG), math.radians(360.0 - NOTCH_DEG)
    angs = np.linspace(a_lo, a_hi, n + 1)
    slope = math.atan2(PAN_TOP_R - PAN_BOT_R, PAN_DEPTH)
    dn = PAN_T  # wall thickness, outward
    for i in range(n):
        a0, a1 = angs[i], angs[i + 1]
        def p(r, a, z):
            return (r * math.cos(a), r * math.sin(a), z)
        rb, rt = PAN_BOT_R, PAN_TOP_R
        wall = hexa([p(rb, a0, -PAN_DEPTH), p(rb + dn, a0, -PAN_DEPTH - dn * 0.5), p(rb + dn, a1, -PAN_DEPTH - dn * 0.5), p(rb, a1, -PAN_DEPTH),
                     p(rt, a0, 0.0), p(rt + dn, a0, -dn * 0.5), p(rt + dn, a1, -dn * 0.5), p(rt, a1, 0.0)])
        # (corner order: bottom 4 then top 4, matched)
        parts.append((f"wall{i}", wall, "pan"))
        cols.append(m.col_hull(f"wall{i}_c", wall))
        fl = hexa([p(rt, a0, 0.0), p(PAN_RIM_R, a0, 0.0), p(PAN_RIM_R, a1, 0.0), p(rt, a1, 0.0),
                   p(rt, a0, 1.2 * MM), p(PAN_RIM_R, a0, 1.2 * MM), p(PAN_RIM_R, a1, 1.2 * MM), p(rt, a1, 1.2 * MM)])
        parts.append((f"flange{i}", fl, "pan"))
        cols.append(m.col_hull(f"flange{i}_c", fl))
    floor = (PAN_BOT_R + PAN_T, 3.0 * MM, (0.0, 0.0, -PAN_DEPTH - 1.5 * MM))
    parts.append(("floor", m.cylinder(floor[0], floor[1], floor[2], segments=40), "pan"))
    cols.append(m.col_cylinder("floor_c", *floor))
    _ = slope
    return parts, cols


_pp, _pc = _pan()
PAN = m.free("drip_pan", (XC, YC, ZD + 0.0005), _pp, _pc, LOOKS, mass=PAN_MASS, friction=(0.6, 0.5), solver_iters=32, yaw_deg=WELL_YAW)


# -- the element (loose) -----------------------------------------------------------------------------------------------------------
def _element():
    parts, cols = [], []
    for k, r in enumerate(RINGS):
        n = 20 if k == 0 else 12
        th = np.linspace(0.0, 2 * math.pi, 49)
        pts = np.c_[r * np.cos(th), r * np.sin(th), np.zeros_like(th)]
        parts.append((f"ring{k}", m.tube(pts, TUBE_R, sides=10), "tube"))
        seg = 2 * r * math.sin(math.pi / n) + 1.0 * MM
        for i in range(n):
            a = 360.0 * (i + 0.5) / n
            c = m.cylinder(TUBE_R, seg, (0.0, 0.0, 0.0), axis="y", segments=10).moved((r, 0.0, 0.0)).turned(a, "z")
            cols.append(m.col_hull(f"ring{k}_{i}_c", c))
    # the two tube ends, from the outer turn down into the terminal head
    for s in (-1, 1):
        lead = np.array([(RINGS[0] - 1.0 * MM, s * 9.0 * MM, 0.0), (HEAD_X0 + 2.0 * MM, s * 9.0 * MM, -2.0 * MM), (HEAD_X0 + 6.0 * MM, s * 9.0 * MM, HEAD_Z1 - 1.0 * MM)])
        parts.append((f"lead{s + 1}", m.tube(lead, TUBE_R, sides=10), "tube"))
    # Y-frame: three arms under the coil, with a leg at the end of each
    for k, a in enumerate(LEG_ANG):
        arm = m.box((LEG_R + 12.0 * MM - HUB_R, 6.0 * MM, 2.5 * MM), ((HUB_R + LEG_R + 12.0 * MM) / 2, 0.0, -TUBE_R - 1.25 * MM)).turned(a, "z")
        parts.append((f"arm{k}", arm, "frame"))
        cols.append(m.col_hull(f"arm{k}_c", arm))
        leg = m.box((5.0 * MM, 5.0 * MM, -LEG_Z - TUBE_R - 2.5 * MM), (LEG_R, 0.0, (LEG_Z - TUBE_R - 2.5 * MM) / 2)).turned(a, "z")
        parts.append((f"leg{k}", leg, "frame"))
        cols.append(m.col_hull(f"leg{k}_c", leg))
    hub = m.cylinder(HUB_R, HUB_Z1 - HUB_Z0, (0.0, 0.0, (HUB_Z0 + HUB_Z1) / 2), segments=32)
    parts.append(("hub", hub, "chrome"))
    cols.append(m.col_cylinder("hub_c", HUB_R, HUB_Z1 - HUB_Z0, (0.0, 0.0, (HUB_Z0 + HUB_Z1) / 2)))
    head = ((HEAD_X1 - HEAD_X0, HEAD_W, HEAD_Z1 - HEAD_Z0), ((HEAD_X0 + HEAD_X1) / 2, 0.0, (HEAD_Z0 + HEAD_Z1) / 2))
    parts.append(("head", m.box(*head), "ceramic"))
    cols.append(m.col_box("head_c", *head))
    for s in (-1, 1):
        # a flat blade with a 1 mm chamfered tip
        x0, x1, t, hh = HEAD_X1, HEAD_X1 + BLADE_L, BLADE_T / 2, BLADE_H / 2
        y, z = s * BLADE_Y, BLADE_Z
        blade = hexa([(x0, y - t, z - hh), (x1 - 1.0 * MM, y - t, z - hh), (x1 - 1.0 * MM, y + t, z - hh), (x0, y + t, z - hh),
                      (x0, y - t, z + hh), (x1 - 1.0 * MM, y - t, z + hh), (x1 - 1.0 * MM, y + t, z + hh), (x0, y + t, z + hh)])
        tip = hexa([(x1 - 1.0 * MM, y - t, z - hh), (x1, y - t * 0.4, z - hh * 0.6), (x1, y + t * 0.4, z - hh * 0.6), (x1 - 1.0 * MM, y + t, z - hh),
                    (x1 - 1.0 * MM, y - t, z + hh), (x1, y - t * 0.4, z + hh * 0.6), (x1, y + t * 0.4, z + hh * 0.6), (x1 - 1.0 * MM, y + t, z + hh)])
        parts.append((f"blade{s + 1}", m.join(blade, tip), "chrome"))
        cols.append(m.col_hull(f"blade{s + 1}_c", m.join(blade, tip)))
    return parts, cols


_ep, _ec = _element()
ELEMENT = m.free("element", START, _ep, _ec, LOOKS, mass=ELEM_MASS, friction=(1.0, 0.8), solver_iters=48, com=ELEM_COM, yaw_deg=WELL_YAW)

# -- SimReady props: the towel the element lies on, and two distractors ------------------------------------------------------------
TOWEL = SimReady("towel", pos=(START[0], START[1], TOP), size=(0.205, 0.212, 0.034), mass=0.05,
                 package="consumable_microfiber_towel_folded_small_b01", usd="sm_consumable_microfiber_towel_folded_small_b01_01.usd")
SPONGE = SimReady("sponge", pos=(0.41, 0.24, TOP), size=(0.071, 0.101, 0.033), mass=0.02, package="cleaning_sponge_b01",
                  usd="sm_cleaning_sponge_b01_01.usd", yaw=0.3)
SPRAY = SimReady("spray_bottle", pos=(0.47, -0.22, TOP), size=(0.118, 0.040, 0.283), mass=0.45, package="cleaning_windowSpray_a01",
                 usd="sm_cleaning_windowSpray_a01_01.usd", yaw=2.0)

OBJECTS = [DECK, RECEPTACLE, PAN, ELEMENT, TOWEL, SPONGE, SPRAY]


# -- the scorer ----------------------------------------------------------------------------------------------------------------------
LEGS = np.array([(LEG_R * math.cos(math.radians(a)), LEG_R * math.sin(math.radians(a)), LEG_Z) for a in LEG_ANG])
SHOULDER = np.array([HEAD_X1, 0.0, BLADE_Z])  # where the blades leave the head, on its front face
TIP = np.array([HEAD_X1 + BLADE_L, 0.0, BLADE_Z])
# points on the element, for the hands-clear check
_th = np.linspace(0.0, 2 * math.pi, 36, endpoint=False)
SAMPLES = np.vstack([np.c_[r * np.cos(_th), r * np.sin(_th), np.zeros_like(_th)] for r in RINGS]
                    + [np.array([(x, y, z) for x in np.linspace(HEAD_X0, HEAD_X1 + BLADE_L, 6) for y in (-HEAD_W / 2, 0, HEAD_W / 2) for z in (HEAD_Z0, HEAD_Z1)]),
                       np.array([(r * math.cos(a), r * math.sin(a), z) for r in (0, HUB_R) for a in _th[::6] for z in (HUB_Z0, HUB_Z1)])])

CLEAR = 0.03  # every fingertip point this far from the element to count as let go
REST_V = 0.01  # m/s
DROP_V = 0.8  # m/s
D_FULL, D_HALF, D_MOUTH = 1.5 * MM, 6.0 * MM, 15.0 * MM
TILT_FULL = 1.0  # deg
LEG_TOL = 2.0 * MM
PAN_SHIFT = 5.0 * MM
BEND_OFF = 3.0 * MM  # shoulder forced this far off the slots' axis while the blades are in the block
BEND_DEG = 5.0  # or the blades turned this far off the slots' axis while in the block
IN_BLOCK = 5.0 * MM


def measure(e, p) -> dict:
    """Everything the scorer needs, from the element's and the pan's poses."""
    Re = e.rotation()
    sh = to_local(e.to_world(SHOULDER))[0]
    tip = to_local(e.to_world(TIP))[0]
    d = FACE_X - sh[0]  # shoulder to receptacle face, along the slots' axis (0: fully home)
    axis = tip - sh
    axis /= np.linalg.norm(axis)
    off_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(axis[0])))))
    lat = max(abs(sh[1]), abs(sh[2] - SLOT_Z))
    # how far the blade tips are into the block's slots (negative: not in them, or not in front of them)
    inside = tip[0] - FACE_X if max(abs(tip[1]), abs(tip[2] - SLOT_Z)) < 10.0 * MM else -1.0
    legs_pan = p.to_local(e.to_world(LEGS))
    gaps = legs_pan[:, 2] + PAN_DEPTH  # each leg's height over the pan floor
    head_pan = p.to_local(e.to_world(np.array([(HEAD_X0 + HEAD_X1) / 2, 0.0, (HEAD_Z0 + HEAD_Z1) / 2])))[0]
    in_notch = abs(math.degrees(math.atan2(head_pan[1], head_pan[0]))) < NOTCH_DEG and abs(head_pan[1]) < 19.0 * MM
    pan_shift = float(np.hypot(p.pos[0] - XC, p.pos[1] - YC))
    pan_ok = pan_shift <= PAN_SHIFT and p.tilt_deg() < 3.0 and abs(p.pos[2] - ZD) < 4.0 * MM
    return {"d": float(d), "tilt": float(e.tilt_deg()), "gaps": gaps, "in_notch": bool(in_notch), "pan_ok": bool(pan_ok), "pan_shift": pan_shift,
            "inside": float(inside), "lat": float(lat), "off_deg": float(off_deg), "speed": float(np.linalg.norm(e.lin_vel)),
            "still": float(np.linalg.norm(e.lin_vel)) < REST_V, "Re": Re}


def grade(c) -> float:
    """The score of one moment, before the hands-clear and at-rest gate and the damage cap."""
    resting = bool(min(c["gaps"]) <= LEG_TOL) and c["in_notch"] and c["pan_ok"]
    plugged = c["inside"] > 0.0 and c["lat"] <= BEND_OFF  # both blades in their slots, the shoulder on the slots' axis
    if not plugged:
        return 0.0
    if c["d"] <= D_FULL and c["tilt"] < TILT_FULL and bool(max(abs(g) for g in c["gaps"]) <= LEG_TOL) and c["in_notch"] and c["pan_ok"]:
        return 1.0
    if c["d"] <= D_HALF and resting:
        return 0.5
    if c["d"] <= D_MOUTH and c["inside"] > 0.0 and c["pan_ok"] and not resting:
        return 0.25
    return 0.0


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.dropped = False
        self.bent = False
        self.why = ""
        self.rest = 0
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.k = 0
        if env is not None:
            for leaf, s in (("leaf_l", -1.0), ("leaf_r", 1.0)):
                m.set_target(env, "receptacle", leaf, s * LEAF_PRE)  # preload against its stop

    def step(self, env):
        self.k += 1
        e, p = m.state(env, "element"), m.state(env, "drip_pan")
        c = measure(e, p)
        if c["speed"] > DROP_V and not self.dropped:
            self.dropped, self.why = True, f"element moved at {c['speed']:.2f} m/s"
        if c["inside"] > IN_BLOCK and (c["lat"] > BEND_OFF or c["off_deg"] > BEND_DEG) and not self.bent:
            self.bent, self.why = True, f"blades forced {c['lat'] * 1000:.1f} mm / {c['off_deg']:.1f} deg off the slots' axis"
        tips = m.fingertips(env)
        local = e.to_local(tips)
        hands = float(np.min(np.linalg.norm(local[:, None, :] - SAMPLES[None, :, :], axis=2)))
        self.rest = self.rest + 1 if c["still"] else 0
        gate = hands >= CLEAR and self.rest >= 20
        g = grade(c) if gate else 0.0
        if self.dropped or self.bent:
            g = min(g, 0.25)
        self.hold = self.hold + 1 if g == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.score = g
        if os.environ.get("COIL_LOG"):
            with open(os.environ["COIL_LOG"], "a") as f:
                leaves = m.state(env, "receptacle")
                f.write(f"{self.k} d={c['d'] * 1000:.2f} tilt={c['tilt']:.2f} gaps={np.round(c['gaps'] * 1000, 1).tolist()} notch={c['in_notch']} "
                        f"pan={c['pan_shift'] * 1000:.1f}/{c['pan_ok']} inside={c['inside'] * 1000:.1f} lat={c['lat'] * 1000:.2f} off={c['off_deg']:.2f} "
                        f"v={c['speed']:.3f} hands={hands * 1000:.0f} rest={self.rest} g={g} full={self.full} {self.why} "
                        f"leaves={[round(v * 1000, 2) for v in leaves.joints.values()]} e={np.round(e.pos, 4).tolist()} p={np.round(p.pos, 4).tolist()} "
                        f"tipsL={np.round(tips[:6].mean(0), 4).tolist()} {np.round(tips[6:12].mean(0), 4).tolist()} tipsR={np.round(tips[12:18].mean(0), 4).tolist()} {np.round(tips[18:24].mean(0), 4).tolist()}\n")


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published starting layout) ---------------------------------------------------------------
# The right hand pinches the hub across its 28 mm, fingers closing along world x. A pinch is a hinge: on its own it lets the coil
# pitch about that line (its terminal end is the heavier). The left hand's closed fingers rest on the top of the coil's outer turn on
# the far side and set the pitch: far side up 2.5 degrees while the blades go in, so the coil clears the pan's flange, then level as
# both let it down into the pan. Each arm's forearm stays on its own side of the coil.
GRIP_DZ = 0.006  # hub pinch: TCP above the element's origin, finger ends 2 mm below it
FINGER_END = 0.013  # the closed fingers' ends below the TCP
PRESS_X = -(RINGS[0])  # the outer turn on the far side, element frame, where the left hand's fingers bear
PRESS_R = float(os.environ.get("COIL_PR", "0.080"))  # the left hand's centre, out from the coil's centre: 8 cm keeps the two grippers' bodies (68 mm across) apart
PRESS_ANG = float(os.environ.get("COIL_PANG", "0"))  # the press point's angle off the far side's centre line, toward the right arm
PRESS_IN = 0.001  # pressed this far into the turn, to keep contact
LEFT_YAW = float(os.environ.get("COIL_LYAW", "0.0"))  # press hand: fingers closing along world y, so the gripper's narrow side faces the hub hand
HUB_ARM, PRESS_ARM = ("left", "right") if not os.environ.get("COIL_SWAP") else ("right", "left")  # the receptacle faces the left arm; each hand stays on its own side of the coil
H_AWAY, P_AWAY = (0.20, 0.22), (0.16, -0.24)
PRE = 0.040  # offset before the push: blade tips 16 mm short of the face, inside the deck's hole
THETA = math.radians(2.5)
HUB_YAW = float(os.environ.get("COIL_HYAW", "0.0"))  # fingers closing along world y, across the medallion
HUB_OPEN = -0.0155  # pads 31 mm apart: either side of the 25 mm medallion; the 6 mm-thick finger ends stay inside the innermost turn (51 mm across inside)


def _poses(o, th, cz):
    """World TCP targets of the hub pinch and the far-side press, for the element `o` short of home, far side up `th`, origin at `cz`."""
    hub = (*to_world(-o, 0.0), cz + GRIP_DZ)
    a = math.radians(PRESS_ANG)
    press = (*to_world(-o - PRESS_R * math.cos(a) * math.cos(th), -PRESS_R * math.sin(a)), cz - PRESS_X * math.cos(a) * math.sin(th) + TUBE_R + FINGER_END - PRESS_IN)
    return hub, press


def _wp(t, p, f, label, yaw=0.0):
    return Waypoint(t, tuple(float(v) for v in p), f, label, yaw)


PIVOT_O = float(os.environ.get("COIL_PO", "0.045"))  # how far short of home the far side is set on the deck
TILT = float(os.environ.get("COIL_TILT", "3.0"))  # deg, far side up
PIVOT_PAST = float(os.environ.get("COIL_PP", "0.001"))  # how far the medallion is pressed below the far-turn-on-deck height
PUSH_PAST = 0.001  # pushed this far past home, so the head bottoms on the receptacle


def over_at(o):
    return (*to_world(-o, 0.0), ZD + 0.07)


def demo_plan(mode="full"):
    sx, sy, _ = START
    sz = TOP + TOWEL_REST - LEG_Z  # settled on the towel (measured)
    up = ZD + 0.07
    Y = HUB_YAW
    R = [
        _wp(2.5, (sx, sy, up + 0.03), HUB_OPEN, "over the hub", Y),
        _wp(1.5, (sx, sy, sz + GRIP_DZ + 0.025), HUB_OPEN, "just above the hub", Y),
        _wp(1.0, (sx, sy, sz + GRIP_DZ + 0.025), HUB_OPEN, "settle above the hub", Y),
        _wp(1.5, (sx, sy, sz + GRIP_DZ), HUB_OPEN, "fingers either side of the hub", Y),
        _wp(1.0, (sx, sy, sz + GRIP_DZ), FINGER_CLOSED, "pinch the hub", Y),
        _wp(1.5, (sx, sy, up), FINGER_CLOSED, "lift", Y),
    ]
    over = (*to_world(-PRE, 0.0), up)
    if mode == "naive":
        # one hand: set the coil down over the pan where it lands, let go, then shove it toward the receptacle
        down = (*to_world(-PRE, 0.0), SEAT_Z + GRIP_DZ + 0.006)
        behind = to_world(-PRE - 0.025, 0.0)
        shoved = to_world(-0.012, 0.0)
        R += [
            _wp(3.0, over, FINGER_CLOSED, "over the well", Y),
            _wp(2.0, down, FINGER_CLOSED, "down onto the pan", Y),
            _wp(0.8, down, HUB_OPEN, "let go", Y),
            _wp(1.0, (*down[:2], SEAT_Z + 0.05), HUB_OPEN, "up", Y),
            _wp(1.0, (*behind, SEAT_Z + 0.05), FINGER_CLOSED, "behind the hub", Y),
            _wp(1.0, (*behind, SEAT_Z + 0.004), FINGER_CLOSED, "down beside the hub", Y),
            _wp(3.0, (*shoved, SEAT_Z + 0.004), FINGER_CLOSED, "shove it toward the receptacle", Y),
            _wp(1.0, (*shoved, SEAT_Z + 0.08), FINGER_CLOSED, "up", Y),
            _wp(1.5, (*H_AWAY, ZD + 0.15), FINGER_OPEN, "away", Y),
            _wp(4.0, (*H_AWAY, ZD + 0.15), FINGER_OPEN, "wait", Y),
        ]
        return {HUB_ARM: R}
    if mode in ("pivot", "hold1"):
        # One hand. The medallion pinch is a friction hinge about the pinch line. Set the far side of the coil on the deck behind
        # the well and press the medallion down past it: the coil pitches far side up by about TILT and the pinch keeps that pitch.
        # Lift until the blades are at slot height, push the blades home with the far side clear of the flange, then let the far
        # side down into the pan, pivoting about the seated terminals.
        o_set = PIVOT_O
        th = math.radians(TILT)
        z_set = ZD + TUBE_R - RINGS[0] * math.sin(th)  # origin height with the far turn on the deck, far side up `th`
        z_in = SLOT_Z - BLADE_Z + 0.110 * math.sin(th)  # blades' middle at slot height
        R += [
            _wp(3.0, over_at(o_set), FINGER_CLOSED, "over the deck behind the well", Y),
            _wp(2.0, (*to_world(-o_set, 0.0), z_set + 0.012 + GRIP_DZ), FINGER_CLOSED, "far side down to the deck", Y),
            _wp(1.5, (*to_world(-o_set, 0.0), z_set + GRIP_DZ - PIVOT_PAST), FINGER_CLOSED, "press past it: far side tips up", Y),
            _wp(0.5, (*to_world(-o_set, 0.0), z_set + GRIP_DZ - PIVOT_PAST), FINGER_CLOSED, "settle", Y),
            _wp(1.0, (*to_world(-o_set, 0.0), z_in + GRIP_DZ), FINGER_CLOSED, "blades up to slot height", Y),
            _wp(3.5, (*to_world(PUSH_PAST, 0.0), z_in + GRIP_DZ), FINGER_CLOSED, "push the blades home", Y),
            _wp(2.0, (*to_world(PUSH_PAST, 0.0), SEAT_Z + GRIP_DZ - 0.0005), FINGER_CLOSED, "let the far side down into the pan", Y),
            _wp(1.0, (*to_world(PUSH_PAST, 0.0), SEAT_Z + GRIP_DZ - 0.0005), FINGER_CLOSED, "settle", Y),
        ]
        h_home = (*to_world(PUSH_PAST, 0.0), SEAT_Z + GRIP_DZ - 0.0005)
        if mode == "hold1":
            return {HUB_ARM: R + [_wp(8.0, h_home, FINGER_CLOSED, "keep holding", Y)]}
        return {HUB_ARM: R + _leave_hub(h_home)}
    cz_in = SLOT_Z + 0.010 + 0.110 * math.sin(THETA)  # blades' middle at slot height with the far side up THETA
    h_pre, p_pre = _poses(PRE, THETA, cz_in)
    h_mid, p_mid = _poses(0.008, THETA, cz_in)
    h_home, p_home = _poses(0.0, 0.0, SEAT_Z)
    R += [_wp(3.0, over, FINGER_CLOSED, "over the well", Y)]
    L = [
        _wp(10.0, (*P_AWAY, up + 0.08), FINGER_CLOSED, "wait clear", LEFT_YAW),
        _wp(2.0, (*p_pre[:2], up + 0.04), FINGER_CLOSED, "over the far side", LEFT_YAW),
        _wp(1.5, (*p_pre[:2], up + p_pre[2] - h_pre[2]), FINGER_CLOSED, "press the far side level with the hub", LEFT_YAW),
    ]
    R += [_wp(1.5, over, FINGER_CLOSED, "hold while the far side is set", Y)]
    seq = [
        (2.0, h_pre, p_pre, "both down: far side up 2.5 deg, blades at slot height"),
        (1.0, h_pre, p_pre, "settle"),
        (3.0, h_mid, p_mid, "push the blades in, far side riding over the flange"),
        (2.0, h_home, p_home, "home, and let the far side down into the pan"),
        (1.0, h_home, p_home, "settle"),
    ]
    hub_seq = [_wp(t, h, FINGER_CLOSED, label, Y) for t, h, p, label in seq]
    if mode == "onearm":
        return {HUB_ARM: R[:7] + hub_seq + _leave_hub(h_home)}
    R += hub_seq
    L += [_wp(t, p, FINGER_CLOSED, label, LEFT_YAW) for t, h, p, label in seq]
    if mode == "hold":
        R.append(_wp(8.0, h_home, FINGER_CLOSED, "keep holding", Y))
        L.append(_wp(8.0, p_home, FINGER_CLOSED, "keep pressing", LEFT_YAW))
        return {PRESS_ARM: L, HUB_ARM: R}
    R += _leave_hub(h_home)
    L += [
        _wp(1.0, (*p_home[:2], p_home[2] + 0.06), FINGER_CLOSED, "up", LEFT_YAW),
        _wp(1.5, (*P_AWAY, ZD + 0.15), FINGER_CLOSED, "away", LEFT_YAW),
        _wp(3.0, (*P_AWAY, ZD + 0.15), FINGER_CLOSED, "wait", LEFT_YAW),
    ]
    return {PRESS_ARM: L, HUB_ARM: R}


def _leave_hub(h_home):
    Y = HUB_YAW
    return [
        _wp(1.0, h_home, HUB_OPEN, "let go of the hub", Y),
        _wp(1.5, (*h_home[:2], SEAT_Z + 0.08), HUB_OPEN, "up", Y),
        _wp(1.5, (*H_AWAY, ZD + 0.15), FINGER_OPEN, "away", Y),
        _wp(4.0, (*H_AWAY, ZD + 0.15), FINGER_OPEN, "wait", Y),
    ]


def demo():
    mode = os.environ.get("COIL_MODE", "full")
    if mode == "none":
        return {}
    if mode == "probe":
        return {"right": [_wp(3.0, (0.21, -0.20, 0.85), FINGER_CLOSED, "a", HUB_YAW), _wp(2.0, (0.21, -0.20, 0.85), FINGER_CLOSED, "a", HUB_YAW),
                          _wp(2.0, (0.21, -0.20, 0.82), FINGER_CLOSED, "b", HUB_YAW), _wp(2.0, (0.21, -0.20, 0.82), FINGER_CLOSED, "b", HUB_YAW),
                          _wp(2.0, (0.21, -0.20, 0.82), FINGER_CLOSED, "c", 0.0), _wp(2.0, (0.21, -0.20, 0.82), FINGER_CLOSED, "c", 0.0)]}
    return demo_plan(mode)
