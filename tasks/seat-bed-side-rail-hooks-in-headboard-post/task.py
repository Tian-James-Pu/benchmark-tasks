"""On the station table, extended into a bedroom floor: a solid-wood queen bed frame being put together (Woodworker's Journal's
Greene & Greene-inspired queen bed plan, with Rockler 6" hook-in bed rail fasteners). The headboard stands across the left of the
scene, running away from the arms: its near leg (70 x 35 mm, 978 mm tall) stands just left of the table's middle, at x = 0.265 to
0.335 and y = +0.070 to +0.105, its face towards the robot's right; its far leg is 1.58 m further away. The far side rail is
already hooked onto the far leg; the footboard stands beyond the rails' far ends, waiting to be hooked on last; the bed's six
cross supports and two angle irons lie on the floor inside the frame. Mortised flush in the near leg's face is the slotted half of
the fastener (151 x 16 x 3.6 mm steel): two 6.35 x 25.4 mm slots, 82.5 mm apart one above the other. The near side rail (poplar,
28.6 x 165 mm, 2.06 m long, 4.4 kg) runs along y to the right: its far end rests on a helper support by the footboard; its near
end, with the hook half of the fastener (two steel hooks 4 mm thick, standing 11 mm out of its end), rests on the floor 30 mm
short of the leg and 25 mm towards the arms. A mug, a screwdriver and the assembly booklet lie on the floor out of the way.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
IN = 0.0254

# -- the fastener: Rockler 6" x 5/8" Heavy Duty Wrought Steel Bed Rail Fastener (item 28597), from its installation drawing ----------
PLATE_L = (5 + 15 / 16) * IN  # 150.8 mm: both plates
PLATE_W = 5 / 8 * IN  # 15.9 mm
PLATE_T = 9 / 64 * IN  # 3.6 mm
HOOK_T = 5 / 32 * IN  # 4.0 mm: each hook's thickness across
HOOK_OUT = 7 / 16 * IN  # 11.1 mm: how far a hook stands out of the rail's end
HOOK_H = 7 / 8 * IN  # 22.2 mm: a hook's height
TIP_H = 3 / 8 * IN  # 9.5 mm: its tip, hanging below its neck
NECK_H = HOOK_H - TIP_H  # 12.7 mm
THROAT = 0.0040  # the tip's inner face, at the neck, from the rail's end: 0.4 mm more than the post plate is thick
SLOT_W = 1 / 4 * IN  # 6.35 mm
SLOT_H = 1.0 * IN  # 25.4 mm
HOOK_PITCH = 3.25 * IN  # 82.5 mm between the two hooks (and the two slots)
HOOK_TOP_DOWN = 1.12 * IN  # the upper hook's top below the plate's top end (scaled from the drawing)
CAVITY = 0.5 * IN  # the hook mortise behind each slot: 12.7 mm deep (the drawing's "1/4 in holes, 1/2 in deep")

# -- the bed: Woodworker's Journal Greene & Greene-inspired queen bed (materials list), rails 6 1/2" deep for the 6" fastener ----
RAIL_T = 1.125 * IN  # 28.6 mm (the plan's 1 1/8")
RAIL_H = 0.165  # 6 1/2": deep enough for the 151 mm plate (the plan's 6" rail takes the 4" fastener)
RAIL_LEN = 81.0 * IN  # 2.057 m (the plan's 81")
RAIL_MASS = 4.4  # poplar at 450 kg/m^3: 28.6 x 165 x 2057 mm
LEG_X = 2.75 * IN  # 70 mm: a leg along the headboard
LEG_Y = 1.375 * IN  # 35 mm: a leg across it (the rail's end meets its 70 mm face)
HB_LEG_H = 38.5 * IN  # 978 mm
FB_LEG_H = 21.5 * IN  # 546 mm
X_R = 0.30  # the near rail's and the near legs' centre line
Y0 = 0.07  # the near headboard leg's face: where the rail's end sits when seated
HB = 0.035  # the rails' underside above the floor when seated
X_FAR = X_R + 1.549 + RAIL_T  # the far rail's centre line: 61" between the rails, for a 60" queen mattress
Y_HB = Y0 + LEG_Y / 2  # the headboard's centre plane
Y_BAR = Y0 - RAIL_LEN + 0.10  # the helper support's bar under the near rail, 10 cm from its far end
BAR_R = 0.010
CHEEK_GAP = 0.0015  # clearance either side of the rail in the support
POST_TOP = HB_LEG_H

PLATE_Z0 = (RAIL_H - PLATE_L) / 2  # the hook plate's lower end, above the rail's underside
HOOKS = ("upper", "lower")
ZT = {"upper": PLATE_Z0 + PLATE_L - HOOK_TOP_DOWN, "lower": PLATE_Z0 + PLATE_L - HOOK_TOP_DOWN - HOOK_PITCH}  # hook tops, rail frame
SLOT_BOT = {k: HB + v - NECK_H for k, v in ZT.items()}  # post frame: a seated hook's neck rests on its slot's bottom
SLOT_TOP = {k: v + SLOT_H for k, v in SLOT_BOT.items()}
CAV_BOT = {k: v - 0.013 for k, v in SLOT_BOT.items()}
WEDGE_SHORT = 0.003  # the hooks' taper pinches the post plate over their last 3 mm of drop
WEDGE_K = 20000.0  # N/m
WEDGE_BREAK = 16.0  # N per hook: what it takes to drive a hook further down that last 3 mm
WEDGE_GIVE = WEDGE_BREAK / WEDGE_K

LOOKS = {
    "wood": m.Look(m.rgb(0.70, 0.50, 0.32), roughness=0.55),
    "endgrain": m.Look(m.rgb(0.60, 0.42, 0.26), roughness=0.75),
    "post": m.Look(m.rgb(0.55, 0.36, 0.21), roughness=0.5),
    "panel": m.Look(m.rgb(0.48, 0.30, 0.17), roughness=0.5),
    "steel": m.Look(m.rgb(0.30, 0.31, 0.33), metallic=0.85, roughness=0.35),
    "screw": m.Look(m.rgb(0.62, 0.62, 0.60), metallic=0.9, roughness=0.3),
    "dark": m.Look(m.rgb(0.04, 0.035, 0.03), roughness=0.95),
    "pine": m.Look(m.rgb(0.86, 0.74, 0.55), roughness=0.7),
    "block": m.Look(m.rgb(0.78, 0.66, 0.48), roughness=0.8),
    "floor": m.Look((0.792, 0.524, 0.273), roughness=0.6),  # the station table's own (linear) colour
}


def _yz_prism(outline_yz, x_half):
    """A solid from a counter-clockwise (y, z) outline, x_half either side of x = 0."""
    p = m.prism(outline_yz, -x_half, x_half)
    return m.Mesh(p.v[:, [2, 0, 1]], p.f)


def _neck(zt):
    return _yz_prism([(0.0, zt - NECK_H), (HOOK_OUT, zt - NECK_H), (HOOK_OUT, zt - 0.002), (HOOK_OUT - 0.002, zt), (0.0, zt)], HOOK_T / 2)


def _tip(zt):
    return _yz_prism([(THROAT, zt - NECK_H), (HOOK_OUT - 0.0008, zt - HOOK_H), (HOOK_OUT, zt - HOOK_H + 0.0005), (HOOK_OUT, zt - NECK_H)], HOOK_T / 2)


def _flip_y(mesh, dy):
    """The mesh mirrored in y (faces turned to stay outward), then moved dy along y."""
    return m.Mesh(mesh.v * np.array([1.0, -1.0, 1.0]) + np.array([0.0, dy, 0.0]), mesh.f[:, ::-1].copy())


def _box(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _cbox(name, x0, x1, y0, y1, z0, z1):
    return m.col_box(name, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


# -- the near side rail: frame at its end, centre of its thickness, on its underside; it runs along -y --------------------------
def _rail():
    L = RAIL_LEN
    parts = [
        ("body", _box(-RAIL_T / 2, RAIL_T / 2, -L + 0.0004, -0.0004, 0.0, RAIL_H), "wood"),
        ("end", _box(-RAIL_T / 2, RAIL_T / 2, -0.0004, 0.0, 0.0, RAIL_H), "endgrain"),
        ("far_end", _box(-RAIL_T / 2, RAIL_T / 2, -L, -L + 0.0004, 0.0, RAIL_H), "endgrain"),
        ("plate", _box(-PLATE_W / 2, PLATE_W / 2, -0.0010, 0.0002, PLATE_Z0, PLATE_Z0 + PLATE_L), "steel"),
        ("far_plate", _box(-PLATE_W / 2, PLATE_W / 2, -L - 0.0002, -L + 0.0010, PLATE_Z0, PLATE_Z0 + PLATE_L), "steel"),
    ]
    for k, z in enumerate((PLATE_Z0 + 0.012, PLATE_Z0 + PLATE_L / 2, PLATE_Z0 + PLATE_L - 0.012)):
        parts.append((f"screw{k}", m.cylinder(0.0045, 0.0012, (0.0, 0.0008, z), axis="y", segments=16), "screw"))
    cols = [m.col_box("body_c", (RAIL_T, L, RAIL_H), (0.0, -L / 2, RAIL_H / 2))]
    for h in HOOKS:
        parts += [(f"neck_{h}", _neck(ZT[h]), "steel"), (f"tip_{h}", _tip(ZT[h]), "steel")]
        cols += [m.col_hull(f"neck_{h}_c", _neck(ZT[h])), m.col_hull(f"tip_{h}_c", _tip(ZT[h]))]
        # the far end's hooks, for the footboard (seen only; they never reach it in a trial)
        parts += [(f"far_neck_{h}", _flip_y(_neck(ZT[h]), -L), "steel"), (f"far_tip_{h}", _flip_y(_tip(ZT[h]), -L), "steel")]
    return parts, cols


def rail_frame(end):
    """The rail's rotation when its end's underside (centre) is at world point `end` and its underside lies on the support's bar."""
    d = np.asarray(end, float) - np.array([X_R, Y_BAR, TOP + HB])
    y_l = d / np.linalg.norm(d)
    x_l = np.cross(y_l, (0.0, 0.0, 1.0))
    x_l /= np.linalg.norm(x_l)
    return np.column_stack([x_l, y_l, np.cross(x_l, y_l)])


def _xyzw(r):
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s)
    q = np.array(q)
    return tuple(float(v) for v in q / np.linalg.norm(q))


START_END = np.array([X_R - 0.025, Y0 - 0.030, TOP])  # on the floor, 30 mm short of the leg, 25 mm towards the arms
START_R = rail_frame(START_END)
START_FAR_Y = float((START_R @ np.array([0.0, -RAIL_LEN, 0.0]) + START_END)[1])
Y_FB = START_FAR_Y - 0.060  # the footboard legs' inner faces: 6 cm beyond the rails' far ends
RAIL_PARTS, RAIL_COLS = _rail()
RAIL = m.free("rail", tuple(START_END + (0.0, 0.0, 0.001)), RAIL_PARTS, RAIL_COLS, LOOKS, mass=RAIL_MASS, friction=(0.6, 0.5),
              start_quat=_xyzw(START_R), com=(0.0, -RAIL_LEN / 2, RAIL_H / 2), solver_iters=48)


# -- the near headboard leg, its slotted plate and the wedge fit (post frame: world axes, origin on the floor at the leg's face) --
def _post():
    hx, sw, d1, d2 = LEG_X / 2, SLOT_W / 2, PLATE_T, PLATE_T + CAVITY
    pz0, pz1 = HB + PLATE_Z0, HB + PLATE_Z0 + PLATE_L
    lo, up = "lower", "upper"
    cols = []

    def cb(name, *b):
        cols.append(_cbox(name, *b))

    cb("back_c", -hx, hx, d2, LEG_Y, 0.0, POST_TOP)
    cb("side_l_c", -hx, -sw, 0.0, d2, 0.0, POST_TOP)
    cb("side_r_c", sw, hx, 0.0, d2, 0.0, POST_TOP)
    cb("c0_c", -sw, sw, 0.0, d2, 0.0, CAV_BOT[lo])
    cb("plate_lo_c", -sw, sw, 0.0, d1, CAV_BOT[lo], SLOT_BOT[lo])
    cb("c1_c", -sw, sw, 0.0, d2, SLOT_TOP[lo], CAV_BOT[up])
    cb("plate_up_c", -sw, sw, 0.0, d1, CAV_BOT[up], SLOT_BOT[up])
    cb("c2_c", -sw, sw, 0.0, d2, SLOT_TOP[up], POST_TOP)
    pw = PLATE_W / 2
    parts = [
        ("post", _box(-hx, hx, d1 + 0.0002, LEG_Y, 0.0, POST_TOP - 0.006), "post"),
        ("face_l", _box(-hx, -pw, 0.0, d1 + 0.0003, 0.0, POST_TOP - 0.006), "post"),
        ("face_r", _box(pw, hx, 0.0, d1 + 0.0003, 0.0, POST_TOP - 0.006), "post"),
        ("face_lo", _box(-pw, pw, 0.0, d1 + 0.0003, 0.0, pz0), "post"),
        ("face_hi", _box(-pw, pw, 0.0, d1 + 0.0003, pz1, POST_TOP - 0.006), "post"),
        ("cap", _box(-hx + 0.004, hx - 0.004, 0.004, LEG_Y - 0.004, POST_TOP - 0.006, POST_TOP), "post"),
        ("splate_l", _box(-pw, -sw, -0.0003, d1, pz0, pz1), "steel"),
        ("splate_r", _box(sw, pw, -0.0003, d1, pz0, pz1), "steel"),
        ("splate_0", _box(-sw, sw, -0.0003, d1, pz0, SLOT_BOT[lo]), "steel"),
        ("splate_1", _box(-sw, sw, -0.0003, d1, SLOT_TOP[lo], SLOT_BOT[up]), "steel"),
        ("splate_2", _box(-sw, sw, -0.0003, d1, SLOT_TOP[up], pz1), "steel"),
    ]
    for h in HOOKS:
        parts.append((f"mortise_{h}", _box(-sw, sw, d1 + 0.0004, d1 + 0.0010, SLOT_BOT[h], SLOT_TOP[h]), "dark"))
    for k, z in enumerate((pz0 + 0.012, (pz0 + pz1) / 2, pz1 - 0.012)):
        parts.append((f"screw{k}", m.cylinder(0.0045, 0.0012, (0.0, -0.0008, z), axis="y", segments=16), "screw"))
    links = []
    for h in HOOKS:
        top = SLOT_BOT[h] - TIP_H + WEDGE_SHORT
        block = _box(-0.0028, 0.0028, 0.0050, 0.0150, top - 0.006, top)
        links.append(m.slider(f"wedge_{h}", "Z", -(WEDGE_SHORT + 0.0002), 0.0, [(f"wedge_{h}_v", block, "dark", True)],
                              [m.col_box(f"wedge_{h}_c", (0.0056, 0.010, 0.006), (0.0, 0.010, top - 0.003))],
                              at=(0.0, 0.010, top - 0.003), spring=WEDGE_K, damping=80.0, effort=400.0, friction=0.0, mass=0.1,
                              armature=0.01))
    return parts, cols, links


POST_PARTS, POST_COLS, POST_LINKS = _post()
POST = m.mechanism("post", (X_R, Y0, TOP), POST_LINKS, LOOKS, frame_parts=POST_PARTS, frame_colliders=POST_COLS,
                   friction=(0.4, 0.3), solver_iters=48)


# -- the rest of the bed (world x, y; heights above the floor) --------------------------------------------------------------------
def _slotted_plate(name, x, y_face, facing):
    """A post's slotted plate (seen only), flush in a leg's face at y_face, facing +y (1) or -y (-1)."""
    y0, y1 = sorted((y_face, y_face - facing * 0.0004))
    z0 = HB + PLATE_Z0
    out = [(f"{name}_plate", _box(x - PLATE_W / 2, x + PLATE_W / 2, y0 - 0.0002 * facing, y1, z0, z0 + PLATE_L), "steel")]
    for h in HOOKS:
        out.append((f"{name}_slot_{h}", _box(x - SLOT_W / 2, x + SLOT_W / 2, min(y0, y1) - 0.0006, max(y0, y1) + 0.0006,
                                             SLOT_BOT[h], SLOT_TOP[h]), "dark"))
    return out


def _end_frame(name, y_c, leg_h, top_h, panel_top):
    """A headboard or footboard: the far leg (the near headboard leg is the post), a bottom rail level with the side rails,
    a panel and a top rail. Returns (parts, colliders)."""
    parts, cols = [], []
    x0, x1 = X_R + LEG_X / 2, X_FAR - LEG_X / 2
    pieces = {
        "bottom": (x0, x1, y_c - 0.0125, y_c + 0.0125, HB, HB + RAIL_H, "post"),
        "panel": (x0, x1, y_c - 0.0095, y_c + 0.0095, HB + RAIL_H, panel_top, "panel"),
        "top": (x0, x1, y_c - LEG_Y / 2, y_c + LEG_Y / 2, panel_top, panel_top + top_h, "post"),
        "far_leg": (X_FAR - LEG_X / 2, X_FAR + LEG_X / 2, y_c - LEG_Y / 2, y_c + LEG_Y / 2, 0.0, leg_h, "post"),
    }
    if name == "footboard":
        pieces["near_leg"] = (X_R - LEG_X / 2, X_R + LEG_X / 2, y_c - LEG_Y / 2, y_c + LEG_Y / 2, 0.0, leg_h, "post")
    for k, (a, b, c, d, e, f, look) in pieces.items():
        parts.append((f"{name}_{k}", _box(a, b, c, d, e, f), look))
        cols.append(_cbox(f"{name}_{k}_c", a, b, c, d, e, f))
    for k in range(1, 6):  # the panel's vertical battens (a Greene & Greene headboard's look)
        xb = x0 + (x1 - x0) * k / 6
        parts.append((f"{name}_batten{k}", _box(xb - 0.025, xb + 0.025, y_c - 0.013, y_c + 0.013, HB + RAIL_H, panel_top), "post"))
    return parts, cols


def _bed():
    hb_parts, hb_cols = _end_frame("headboard", Y_HB, HB_LEG_H, 3 * IN, HB_LEG_H - 0.02 - 3 * IN)
    fb_c = Y_FB - LEG_Y / 2
    fb_parts, fb_cols = _end_frame("footboard", fb_c, FB_LEG_H, 3 * IN, FB_LEG_H - 0.02 - 3 * IN)
    fb_parts += _slotted_plate("fb_near", X_R, Y_FB, 1) + _slotted_plate("fb_far", X_FAR, Y_FB, 1)
    parts, cols = hb_parts + fb_parts, hb_cols + fb_cols
    # the far side rail, already hooked onto the far headboard leg; its far end on a support block by the footboard
    far_y1 = Y0 - RAIL_LEN
    parts.append(("far_rail", _box(X_FAR - RAIL_T / 2, X_FAR + RAIL_T / 2, far_y1, Y0, HB, HB + RAIL_H), "wood"))
    cols.append(_cbox("far_rail_c", X_FAR - RAIL_T / 2, X_FAR + RAIL_T / 2, far_y1, Y0, HB, HB + RAIL_H))
    parts.append(("far_block", _box(X_FAR - 0.045, X_FAR + 0.045, far_y1 + 0.02, far_y1 + 0.11, 0.0, HB), "block"))
    cols.append(_cbox("far_block_c", X_FAR - 0.045, X_FAR + 0.045, far_y1 + 0.02, far_y1 + 0.11, 0.0, HB))
    # the plan's six cross supports (3/4 x 4 1/2 x 60 3/4 in, pine), stacked two high along the bed, and its two 1 1/2 in steel
    # angle irons, on the floor inside the frame, well clear of the near rail
    sw, sl, st = 4.5 * IN, 60.75 * IN, 0.75 * IN
    ys1 = -0.40
    for k in range(6):
        xc = 0.98 + (k % 3) * (sw + 0.004)
        z0 = (k // 3) * st
        parts.append((f"support{k}", _box(xc - sw / 2, xc + sw / 2, ys1 - sl, ys1, z0, z0 + st), "pine"))
    cols.append(_cbox("supports_c", 0.98 - sw / 2, 0.98 + 2 * (sw + 0.004) + sw / 2, ys1 - sl, ys1, 0.0, 2 * st))
    ai, at = 1.5 * IN, 0.125 * IN
    for k, xa in enumerate((1.45, 1.53)):
        y_a0, y_a1 = -1.85, -1.85 + 80 * IN
        parts.append((f"angle{k}_a", _box(xa, xa + ai, y_a0, y_a1, 0.0, at), "steel"))
        parts.append((f"angle{k}_b", _box(xa, xa + at, y_a0, y_a1, 0.0, ai), "steel"))
    # the helper support under the near rail's far end: a block with a 20 mm dowel across its top between two guide cheeks
    g = RAIL_T / 2 + CHEEK_GAP
    sup = {
        "sup_block": _box(X_R - 0.045, X_R + 0.045, Y_BAR - 0.045, Y_BAR + 0.045, 0.0, HB - BAR_R),
        "sup_cheek_l": _box(X_R - g - 0.012, X_R - g, Y_BAR - 0.02, Y_BAR + 0.02, HB - BAR_R, HB + 0.05),
        "sup_cheek_r": _box(X_R + g, X_R + g + 0.012, Y_BAR - 0.02, Y_BAR + 0.02, HB - BAR_R, HB + 0.05),
    }
    bar = m.cylinder(BAR_R, 2 * g, (X_R, Y_BAR, HB - BAR_R), axis="x", segments=24)
    for k, v in sup.items():
        parts.append((k, v, "block"))
        cols.append(m.col_hull(f"{k}_c", v))
    parts.append(("sup_bar", bar, "pine"))
    cols.append(m.col_hull("sup_bar_c", bar))
    return parts, cols


BED_PARTS, BED_COLS = _bed()
BED = m.fixed("bed", (0.0, 0.0, TOP), BED_PARTS, BED_COLS, LOOKS, friction=(0.3, 0.25))


# -- the bedroom floor: the table top, extended level all round the bed --------------------------------------------------------
def _floor():
    tx0, tx1, ty = -0.19, 0.91, 0.65  # the station table's top
    areas = {"far": (tx1, 2.4, -2.6, 1.0), "right": (tx0, tx1, -2.6, -ty), "left": (tx0, tx1, ty, 1.0)}
    parts = [(k, _box(a, b, c, d, -0.03, 0.0), "floor") for k, (a, b, c, d) in areas.items()]
    cols = [_cbox(f"{k}_c", a, b, c, d, -0.03, 0.0) for k, (a, b, c, d) in areas.items()]
    return parts, cols


FLOOR_PARTS, FLOOR_COLS = _floor()
ROOM_FLOOR = m.fixed("bedroom_floor", (0.0, 0.0, TOP), FLOOR_PARTS, FLOOR_COLS, LOOKS, friction=(0.6, 0.5))

OBJECTS = [
    POST,
    RAIL,
    BED,
    ROOM_FLOOR,
    # Distractors on the floor inside the frame, clear of the work (SimReady props), which scoring ignores.
    SimReady("mug", pos=(0.62, -0.36, TOP), size=(0.114, 0.090, 0.080), mass=0.30, yaw=0.6,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("screwdriver", pos=(0.60, -0.03, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=0.5, convex=True,
             package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd"),
    SimReady("booklet", pos=(0.86, -0.30, TOP), size=(0.229, 0.170, 0.010), mass=0.12, yaw=0.3,
             package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd"),
]


# -- reading the joint --------------------------------------------------------------------------------------------------------------
FORCED = 0.0015  # past the leg's face, or below a slot's bottom: only by force
E_TOL = 0.002  # a hook's centre within 2 mm sideways of its slot's centre (the slot itself allows 1.2 mm)
IN_GAP = 0.0075  # a hook 3.6 mm or more past the leg's face: through its slot
IN_S = TIP_H  # its tip below the slot's bottom: dropped behind the plate
SEAT_TOL = 0.001  # seated: s and gap each within 1 mm of zero, either way
WEDGED_S = 0.0045


def hook_readings(rail):
    """For each hook: (s, gap, e) in metres. s: its top's height above where it sits when seated (the rail's drop still to go;
    negative: below it); gap: the rail's end face from the leg's face at the hook's height (positive: short of it); e: its
    centre's sideways offset from its slot's centre."""
    out = {}
    for h in HOOKS:
        top, face = rail.to_world(np.array([[0.0, HOOK_OUT / 2, ZT[h]], [0.0, 0.0, ZT[h] - NECK_H / 2]]))
        out[h] = (float(top[2] - TOP - (SLOT_BOT[h] + NECK_H)), float(Y0 - face[1]), float(top[0] - X_R))
    return out


def hooked(s, gap, e):
    """A hook is in (through its slot and dropped behind the plate): every reading inside a two-sided band."""
    return -FORCED < gap <= IN_GAP and -FORCED < s <= IN_S and abs(e) <= E_TOL


def seated(s, gap, e):
    """A hook fully home: in, with s and gap each within 1 mm of zero."""
    return hooked(s, gap, e) and abs(s) <= SEAT_TOL and abs(gap) <= SEAT_TOL


def forced(s, gap, e):
    """A hook or the rail's end driven more than FORCED into the leg's wood or steel (only possible by force)."""
    if abs(e) >= LEG_X / 2 or s + SLOT_BOT["upper"] + NECK_H > POST_TOP:
        return False  # beside or above the leg
    into_face = gap < -FORCED
    into_slot_bottom = gap < HOOK_OUT / 2 and s < -FORCED
    into_slot_side = gap < HOOK_OUT - FORCED and abs(e) > SLOT_W / 2 - HOOK_T / 2 + FORCED
    return bool(into_face or into_slot_bottom or into_slot_side)


def _rail_gap(rail, tips):
    """The fingertip points' distance from the rail (its body and the hooks' box), metres."""
    local = rail.to_local(tips)
    lo = np.array([-RAIL_T / 2, -RAIL_LEN, 0.0])
    hi = np.array([RAIL_T / 2, HOOK_OUT, RAIL_H])
    return np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1)


SUCCESS_HOLD_S = 0.0  # the Trial counts its own hold
HOLD_S = 3.0
CLEAR = 0.03
DROP_V = 0.5


class Trial:
    """Updated once per control step, after physics; score() and success() only read it."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.harm, self.why = False, ""
        self.hold, self.full, self.value = 0.0, False, 0.0
        self.prev_end = None
        self.target = {h: 0.0 for h in HOOKS}
        self.read = None

    def _wedges(self, env, post, read):
        """The wedge fit: once a hook is in and down on its stop, the stop gives only to more than WEDGE_BREAK; out, it is free again."""
        for h in HOOKS:
            q, t = post.joint(f"wedge_{h}"), self.target[h]
            if not hooked(*read[h]):
                t = 0.0
            elif q < t - WEDGE_GIVE:
                t = q + WEDGE_GIVE
            if t != self.target[h]:
                self.target[h] = t
                m.set_target(env, "post", f"wedge_{h}", t)

    def step(self, env):
        rail, post = m.state(env, "rail"), m.state(env, "post")
        tips = m.fingertips(env)
        read = hook_readings(rail)
        self.read = read
        self._wedges(env, post, read)
        dt = float(env.unwrapped.step_dt)
        near = _rail_gap(rail, tips)
        end = rail.to_world(np.array([[0.0, -0.03, 0.0]]))[0]
        v = 0.0 if self.prev_end is None else (end[2] - self.prev_end[2]) / dt
        self.prev_end = end
        if not self.harm:
            if v < -DROP_V and near.min() > 0.02:
                self.harm, self.why = True, f"dropped ({v:.2f} m/s)"
            for h, (s, gap, e) in read.items():
                if forced(s, gap, e):
                    self.harm, self.why = True, f"forced {h}: s {s * 1000:.1f} gap {gap * 1000:.1f} e {e * 1000:.1f} mm"
        n = sum(hooked(*read[h]) for h in HOOKS)
        both = n == 2
        home = all(seated(*read[h]) for h in HOOKS)
        wedged = both and all(read[h][0] <= WEDGED_S for h in HOOKS)
        still = np.linalg.norm(rail.lin_vel) < 0.01 and np.linalg.norm(rail.ang_vel) < 0.1
        clear = bool(near.min() >= CLEAR)
        ok = home and still and clear and not self.harm
        self.hold = self.hold + dt if ok else 0.0
        self.full = bool(ok and self.hold >= HOLD_S - 1e-9)
        if self.harm:
            self.value = 0.0
        elif self.full:
            self.value = 1.0
        elif wedged:
            self.value = 0.75
        elif both:
            self.value = 0.5
        elif n == 1:
            self.value = 0.25
        else:
            self.value = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo ----------------------------------------------------------------------------------------------------------
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402


class Orient(float):
    """A waypoint 'yaw' that carries a whole TCP rotation."""

    rot = np.eye(3)


def _orient(r):
    o = Orient(0.0)
    o.rot = r
    return o


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


def tilted(theta_deg, phi_deg, turn_deg=0.0):
    """Tilted theta from straight down towards phi (degrees from +x), the fingers closing across that direction; then the whole
    grip turned turn_deg about the vertical."""
    phi = math.radians(phi_deg)
    r = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain_grip(phi)
    return _orient(axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(turn_deg)) @ r)


GRIP = {"left": -0.22, "right": -0.36}  # where each hand takes the rail's top edge, from its end
GRIP_PHI = {"left": -90.0, "right": 90.0}  # each hand leans 45 degrees along the rail, towards the other
GRIP_TURN = {"left": 35.0, "right": 0.0}  # each grip turned about the vertical, degrees: the left wrist kept clear of the leg
TILT = {"left": 45.0, "right": 45.0}  # each grip's lean from straight down, degrees
GRIP_DEPTH = 0.010  # the TCP this far below the top edge
SAG = {"left": 0.035, "right": 0.035}  # how far each arm sags under its share of the rail (measured), added to its lifting targets
SHIFT = {"left": (0.0, 0.0), "right": (0.0, 0.0)}  # measured (x, y) offsets of each hand's hold from its command


BACK = {"left": 0.07, "right": 0.05}


def _grip_target(arm, end, sag=True):
    r = rail_frame(end)
    p = r @ np.array([0.0, GRIP[arm], RAIL_H]) + np.asarray(end, float) + np.array([0.0, 0.0, -GRIP_DEPTH])
    if sag:
        p = p + np.array([SHIFT[arm][0], SHIFT[arm][1], SAG[arm]])
    turn = math.degrees(math.atan2(-r[0, 1], r[1, 1]))
    return tuple(float(v) for v in p), tilted(TILT[arm], GRIP_PHI[arm], turn + GRIP_TURN[arm])


def _back(arm, p, dist):
    """A point `dist` back up the grip's approach axis from p."""
    phi = math.radians(GRIP_PHI[arm] + GRIP_TURN[arm])
    a = np.array([math.cos(phi) * math.sin(math.radians(TILT[arm])), math.sin(phi) * math.sin(math.radians(TILT[arm])), -math.cos(math.radians(TILT[arm]))])
    return tuple(float(v) for v in np.asarray(p) - dist * a)


VIA = {}  # no via point: with the grips this far back, both hands come straight down to the rail
ARMS = ("left", "right")


DEMO_MODE = "full"  # red-team copies of this file set "nothing", "onearm", "careless", "nodrive" or "floorslide"


class Servo:
    """The demo's feedback, from what the cameras see: the rail's pose. The soft arms sag several centimetres under the rail and
    lag when pushing it, and the rail twists in their grip, so while the hands carry or press it they move their commands
    together by the integrated error between where the rail's end should be and where it is seen (GAIN per control step, within
    LIMIT), shift sideways to bring the hooks, as seen, in line with the slots, and roll their grip to bring the two hooks into one
    vertical line. Only the demo uses this; it never reads the scorer's state."""

    GAIN = np.array([0.3, 0.08, 0.08])
    LIMIT_LO = np.array([-0.15, -0.010, -0.03])  # the full-length rail swings its end far sideways as the soft arms take its load
    LIMIT_HI = np.array([0.15, 0.08, 0.14])
    START = np.array([0.0, 0.0, 0.045])  # a first guess of the sag, refined by the loop
    GAIN_ROLL, MAX_ROLL = 0.3, 0.14

    def __init__(self):
        self.live, self.profile, self.k = {}, [], 0
        self.reset()

    def reset(self, env=None):
        self.k, self.roll = 0, 0.0
        self.c = self.START.copy() if DEMO_MODE != "floorslide" else np.zeros(3)
        self._apply()

    def _apply(self):
        turn = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), self.roll)
        for arm, items in self.live.items():
            for arr, base, mode, grip, rot0 in items:
                arr[:] = base + (self.c if mode == "servo" else np.array([self.c[0], 0.0, 0.0]) if mode == "hold" else 0.0)
                if mode != "plain":
                    grip.rot = turn @ rot0

    def wanted(self, t):
        """Where the rail's end should be at time t, if this is a servoed stretch of the plan."""
        for t0, t1, a, b in self.profile:
            if t0 <= t < t1:
                s = (t - t0) / (t1 - t0)
                return a + (3 * s * s - 2 * s * s * s) * (b - a)
        return None

    def step(self, env):
        if not self.profile:
            return
        self.k += 1
        want = self.wanted(self.k * float(env.unwrapped.step_dt))
        if want is not None:
            rail = m.state(env, "rail")
            seen = rail.to_world(np.zeros((1, 3)))[0]
            read = hook_readings(rail)
            e_up, e_lo = read["upper"][2], read["lower"][2]
            err = want - seen
            if DEMO_MODE != "floorslide":  # the floor-slide red team steers by the rail's end, not by the slots
                err[0] = -(e_up + e_lo) / 2
            self.c = np.clip(self.c + self.GAIN * err, self.LIMIT_LO, self.LIMIT_HI)
            self.roll = float(np.clip(self.roll - self.GAIN_ROLL * (e_up - e_lo) / HOOK_PITCH, -self.MAX_ROLL, self.MAX_ROLL))
        self._apply()


SERVO = Servo()
m.on_step(SERVO.step)
m.on_reset(SERVO.reset)




def demo():
    """Both hands take the rail's top edge, the left near its end. They lift the end to just above the slots' height and push its
    hooks against the post's plate, then lower it slowly while still pushing: the hooks pass into the slots as they come level with
    them, drop behind the plate onto the wedge fit, and the hands drive the rail on down its last 3 mm until it is seated. They
    ease off, let go and move away. While carrying and pressing, the hands correct for their sag and the rail's twist (Servo).
    Red-team modes: "nothing" (no plan), "onearm" (the left hand alone does it all), "careless" (lifts the end only to its seated
    height, pushes it at the post and lets go), "nodrive" (hangs the rail on its hooks but never drives it down), "floorslide"
    (slides the rail's end along the floor beside the leg, aiming 7 cm past its face, and lets go)."""
    seat = np.array([X_R, Y0, TOP + HB])
    poses = [
        ("start", START_END),
        ("lifted, short of the post", seat + (0.0, -0.013, 0.018)),
        ("hooks against the plate", seat + (0.0, 0.004, 0.018)),
        ("down past the slots", seat + (0.0, 0.004, 0.002)),
        ("driven home", seat + (0.0, 0.002, -0.025)),
        ("unloaded", seat + (0.0, 0.0, 0.0)),
    ]
    steps = (("lifted, short of the post", 3.0, 1.0), ("hooks against the plate", 2.0, 1.0), ("down past the slots", 4.0, 0.8),
             ("driven home", 2.0, 1.0))
    if DEMO_MODE == "careless":
        poses = [("start", START_END), ("lifted, short of the post", seat + (0.0, -0.013, 0.0)),
                 ("hooks against the plate", seat + (0.0, 0.004, 0.0)), ("unloaded", seat + (0.0, 0.004, 0.0))]
        steps = (("lifted, short of the post", 3.0, 1.0), ("hooks against the plate", 2.0, 1.0))
    elif DEMO_MODE == "floorslide":
        beside = X_R - LEG_X / 2 - RAIL_T / 2 - 0.012
        poses = [("start", START_END), ("beside the leg, on the floor", np.array([beside, Y0 - 0.030, TOP])),
                 ("slid past its face", np.array([beside, Y0 + 0.070, TOP])), ("unloaded", np.array([beside, Y0 + 0.070, TOP]))]
        steps = (("beside the leg, on the floor", 3.0, 1.0), ("slid past its face", 4.0, 2.0))
    elif DEMO_MODE == "nodrive":
        steps = steps[:3]
        poses = [(n, (e + (0.0, 0.0, 0.003) if n == "unloaded" else e)) for n, e in poses]
    arms = {"full": ARMS, "careless": ARMS, "nodrive": ARMS, "floorslide": ARMS, "onearm": ("left",), "nothing": ()}[DEMO_MODE]
    plans = {}
    SERVO.live = {}
    for arm in arms:
        tg = {name: _grip_target(arm, end, sag=False) for name, end in poses}
        p0, g0 = tg["start"]
        live = []

        def add(dur, pos, finger, label, grip, mode="plain"):
            base = np.array(pos, float)
            arr = base.copy()
            mine = _orient(np.array(grip.rot))
            live.append((arr, base, mode, mine, np.array(grip.rot)))
            wps.append(Waypoint(dur, arr, finger, label, mine))

        wps = []
        if arm in VIA:
            add(1.5, VIA[arm], FINGER_OPEN, "in front of the post", g0)
        add(2.0 if arm in VIA else 3.5, _back(arm, p0, BACK[arm]), FINGER_OPEN, "behind the rail's top edge", g0)
        add(1.5, p0, FINGER_OPEN, "round its top edge", g0)
        add(0.8, p0, FINGER_CLOSED, "grip", g0)
        for name, dur, rest in steps:
            p, g = tg[name]
            add(dur, p, FINGER_CLOSED, name, g, "servo")
            add(rest, p, FINGER_CLOSED, name + " (settle)", g, "servo")
        p, g = tg["unloaded"]
        add(1.0, p, FINGER_CLOSED, "ease off", g, "hold")
        add(0.8, p, FINGER_OPEN, "let go", g, "hold")
        add(1.2, _back(arm, p, BACK[arm]), FINGER_OPEN, "back off along the fingers", g, "hold")
        b = _back(arm, p, BACK[arm])
        if arm == "left":
            add(0.8, tuple(np.add(b, (0.0, 0.0, 0.02))), FINGER_OPEN, "up, the fingers still either side", g, "hold")
            add(1.5, tuple(np.add(b, (-0.10, 0.0, 0.02))), FINGER_OPEN, "away", g)
        else:  # higher up, the right arm reaches only with its grip turned 30 degrees about the vertical
            turned = tilted(TILT[arm], 60.0)
            add(1.2, tuple(np.add(b, (0.0, 0.0, 0.04))), FINGER_OPEN, "up, the fingers still either side", turned)
            add(1.5, tuple(np.add(b, (-0.08, 0.0, 0.04))), FINGER_OPEN, "away", turned)
        SERVO.live[arm] = live
        plans[arm] = wps
    ends = dict(poses)
    t0, prev, profile = 5.8, ends["start"], []
    for name, dur, rest in steps:
        profile += [(t0, t0 + dur, np.array(prev), np.array(ends[name])), (t0 + dur, t0 + dur + rest, np.array(ends[name]), np.array(ends[name]))]
        t0, prev = t0 + dur + rest, ends[name]
    SERVO.profile = profile
    SERVO.reset()
    return plans
