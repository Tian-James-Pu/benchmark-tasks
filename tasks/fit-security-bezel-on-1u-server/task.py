"""A Dell PowerEdge R650 1U server in a 19-inch rack, its front bezel (Dell 1U standard bezel, part 325-BCHJ /
V7YM5) standing loose on the table in front of it. The bezel goes on the way Dell's R650 manual says: its two right
tabs go into the slots in the right control panel, then the bezel is pressed shut until its spring-loaded release
latch clicks into the left control panel.

The rack's two front posts stand on the table. Two R650s are in it, one above the other: the lower one has its
bezel fitted; the upper one (the task's) has none. The units above are empty. The loose bezel stands upright on
its bottom edge, its front to the arms, 14 cm in front of the rack.

Frames (metres): world x away from the arms, y to the arms' left, z up. The server's frame: origin on the plane of
its control panels' front faces, mid-width, at its chassis bottom. The bezel's frame: origin mid-depth, mid-length,
at its bottom edge; x toward its back (the server), y toward its left (latch) end.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0   # the tracker below counts its own 1 s at rest and 2 s hold
SCORE_OVER = "end"

# -- the rack and the R650 front (Dell R650 Technical Specifications: 482 mm wide over the ears, 434 mm without,
#    42.8 mm high; 22 mm from the rack flange to the front without the bezel, 35.84 mm with it) ---------------------
XF = 0.410                     # front faces of the control panels ("ears"), world x
U = 0.04445                    # one rack unit (EIA-310)
U_LOW = TOP + 0.010            # bottom of the lower server's unit
U_TASK = U_LOW + U             # bottom of the task server's unit
CH_H = 0.0428                  # chassis height
CH_GAP = 0.0008                # chassis bottom above its unit's bottom
EAR_IN, EAR_OUT = 0.217, 0.241  # control-panel inner and outer faces, |y| (434 mm between them, 482 mm overall)
EAR_D = 0.022                  # control panel depth: front face to rack flange
DRIVE_X = 0.019                # drive-bay face behind the panels' front faces (estimate from Dell's figures)
CHASSIS_D = 0.45               # chassis modelled this far back (the R650 is 0.70-0.75 m deep)
POST_X0, POST_D, POST_W = EAR_D, 0.045, 0.032  # front posts: their front face at the rack flange
POST_IN = 0.2250               # 450 mm opening between the posts (EIA-310)
POST_H = 8 * U + 0.010

# -- the bezel (Dell 1U standard bezel 325-BCHJ / V7YM5); sizes estimated from the R650 figures, see README --------
BZ_L, BZ_H, BZ_D = 0.431, 0.0424, 0.024   # length between end faces, height, depth
BZ_END = BZ_L / 2                          # end faces at local y +-0.2155; 1.5 mm side play each end
BZ_FRONT, BZ_BACK = -0.012, 0.012         # local x of its front face and back edge
BZ_PROUD = 0.0138                          # fitted, its front stands 13.8 mm proud of the panels (35.84 - 22 mm)
BZ_MASS = 0.30
CAP = 0.020                                # solid end caps, 2 cm long
SKIN = 0.003                               # front sheet, rails and lips
LIP_H = 0.012                              # rear lips under the top rail and over the bottom rail
# fitted ("nominal") pose: hanging on its right tabs, the tabs resting on the slots' floors
BZ_X = XF - BZ_PROUD - BZ_FRONT            # bezel origin x
BZ_Z = U_TASK + CH_GAP + 0.00025           # bezel bottom
# right tabs: steel, 4 mm (x) by 8 mm (z), standing 5 mm out of the right end face
TAB_X = (0.0063, 0.0103)
TAB_Z = ((0.0062, 0.0142), (0.0282, 0.0362))
TAB_OUT = 0.005
# left latch tongue: acetal, slides along the bezel; its nose stands 5 mm out of the left end face, ramped at its back
NOSE_X = (0.0063, 0.0120)
NOSE_Z = (0.0172, 0.0252)
NOSE_OUT = 0.005
LATCH_TRAVEL = 0.0055
LATCH_K, LATCH_PRELOAD = 200.0, 0.003      # N/m and m: about 0.6 N out, 1.7 N pressed fully in (estimate)
# slots and window in the panels' inner faces (server frame)
SLOT_X = (0.0025, 0.0105)                  # 2 mm either side of a tab; 2.5 mm of steel in front
SLOT_DEPTH = 0.0065
WIN_X = (0.0035, 0.0115)                   # the catch lip in front of the window is 3.5 mm
WIN_DEPTH = 0.0065
DZ_BZ = BZ_Z - (U_TASK + CH_GAP)           # bezel bottom above the chassis bottom
SLOTS_Z = tuple((DZ_BZ + a, DZ_BZ + b + 0.0025) for a, b in TAB_Z)
WIN_Z = (DZ_BZ + NOSE_Z[0] - 0.0025, DZ_BZ + NOSE_Z[1] + 0.0025)
STOP_X = 0.0104                            # chassis front frame behind each end cap: the bezel's stop
STOP_Y = (0.1960, 0.2168)

# start: standing on its bottom edge on the table, front to the arms
START_X = 0.255

# scoring
TAB_IN_MIN = 0.0015      # each right tab at least 1.5 mm inside its slot
FLUSH_TOL = 0.002        # front face at both ends within 2 mm of its fitted plane
HEIGHT_TOL = 0.003       # bottom edge at both ends within 3 mm of its fitted height
LATCH_OUT_TOL = 0.0008   # release button back out to within 0.8 mm
NOSE_HOLD_MIN = 0.001    # latch nose at least 1 mm past the panel's inner face, behind the catch lip
NOSE_BEHIND_MIN = -0.0005  # its front no more than 0.5 mm forward of the lip's back face (it bears on it when pulled)
CLEAR = 0.03             # every fingertip point at least 3 cm from the bezel
STILL_V, STILL_W, STILL_STEPS = 0.005, math.radians(2.0), 20
PULL_N = 5.0             # retention check: 5 N pulled straight out at the left end's front, for 1 s
PULL_STEPS = 20
PULL_MOVE = 0.002        # the left end may come out at most 2 mm under the pull
HOLD_STEPS = 20          # and the bezel stays latched, hands clear, 1 s more after the pull
LOST_Z = TOP - 0.05

LOOKS = {
    "bezel": m.Look(m.rgb(0.10, 0.105, 0.11), roughness=0.45),
    "grille": m.Look(m.rgb(0.035, 0.035, 0.04), roughness=0.7),
    "trim": m.Look(m.rgb(0.42, 0.43, 0.45), metallic=0.6, roughness=0.35),
    "latch": m.Look(m.rgb(0.55, 0.56, 0.58), roughness=0.5),
    "tab": m.Look(m.rgb(0.75, 0.75, 0.73), metallic=0.85, roughness=0.3),
    "steel": m.Look(m.rgb(0.70, 0.71, 0.72), metallic=0.7, roughness=0.35),
    "panel": m.Look(m.rgb(0.06, 0.06, 0.065), roughness=0.55),
    "drive": m.Look(m.rgb(0.16, 0.165, 0.17), roughness=0.6),
    "orange": m.Look(m.rgb(0.85, 0.42, 0.12), roughness=0.5),
    "led": m.Look(m.rgb(0.15, 0.45, 0.95), emissive=(0.1, 0.3, 0.9)),
    "green": m.Look(m.rgb(0.2, 0.75, 0.3), emissive=(0.05, 0.4, 0.1)),
    "post": m.Look(m.rgb(0.08, 0.08, 0.085), metallic=0.5, roughness=0.5),
    "hole": m.Look(m.rgb(0.01, 0.01, 0.01), roughness=0.9),
}


def _span(a, b):
    return (min(a, b), max(a, b))


def _bx(name, x, y, z):
    """A box from axis ranges: (prim name, mesh)."""
    (x0, x1), (y0, y1), (z0, z1) = _span(*x), _span(*y), _span(*z)
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _cb(name, x, y, z):
    (x0, x1), (y0, y1), (z0, z1) = _span(*x), _span(*y), _span(*z)
    return m.col_box(name, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


# -- the bezel's looks, shared by the loose bezel and the one fitted to the lower server ------------------------------


def _bezel_looks(prefix: str, dx=0.0, dz=0.0):
    """Visual meshes of a bezel in its own frame (moved by dx, dz for the fitted copy)."""
    t = np.array([dx, 0.0, dz])
    v = []
    v.append((f"{prefix}top", _bx("", (BZ_FRONT, BZ_BACK), (-BZ_END + CAP, BZ_END - CAP), (BZ_H - SKIN, BZ_H)).moved(t), "bezel"))
    v.append((f"{prefix}bot", _bx("", (BZ_FRONT, BZ_BACK), (-BZ_END + CAP, BZ_END - CAP), (0.0, SKIN)).moved(t), "bezel"))
    v.append((f"{prefix}band_t", _bx("", (BZ_FRONT, BZ_FRONT + SKIN), (-BZ_END + CAP, BZ_END - CAP), (BZ_H - 0.007, BZ_H - SKIN)).moved(t), "bezel"))
    v.append((f"{prefix}band_b", _bx("", (BZ_FRONT, BZ_FRONT + SKIN), (-BZ_END + CAP, BZ_END - CAP), (SKIN, 0.007)).moved(t), "bezel"))
    v.append((f"{prefix}grille", _bx("", (BZ_FRONT + 0.004, BZ_FRONT + 0.005), (-BZ_END + CAP, BZ_END - CAP), (0.007, BZ_H - 0.007)).moved(t), "grille"))
    v.append((f"{prefix}lip_t", _bx("", (BZ_BACK - SKIN, BZ_BACK), (-BZ_END + CAP, BZ_END - CAP), (BZ_H - LIP_H, BZ_H - SKIN)).moved(t), "bezel"))
    v.append((f"{prefix}lip_b", _bx("", (BZ_BACK - SKIN, BZ_BACK), (-BZ_END + CAP, BZ_END - CAP), (SKIN, LIP_H)).moved(t), "bezel"))
    for s, name in ((1.0, "l"), (-1.0, "r")):
        v.append((f"{prefix}cap_{name}", _bx("", (BZ_FRONT, BZ_BACK), (s * (BZ_END - CAP), s * BZ_END), (0.0, BZ_H)).moved(t), "bezel"))
    # slanted ribs across the grille, as on the Dell bezel
    for k in range(-5, 6):
        y0 = k * 0.034
        outline = [(y0 - 0.010, 0.007), (y0 - 0.004, 0.007), (y0 + 0.010, BZ_H - 0.007), (y0 + 0.004, BZ_H - 0.007)]
        rib = m.prism([(p[0], p[1]) for p in outline], 0.0, 1.0)
        # prism builds in (x=y_bezel, y=z_bezel); place it as a thin plate in the bezel's y-z plane
        verts = np.c_[BZ_FRONT + 0.0005 + rib.v[:, 2] * 0.004, rib.v[:, 0], rib.v[:, 1]]
        v.append((f"{prefix}rib{k + 5}", m.Mesh(verts + t, rib.f), "trim"))  # (x, y, z) -> (z, x, y) keeps the faces' winding
    # key lock cylinder near the left end (left unlocked, not modelled)
    v.append((f"{prefix}lock", m.cylinder(0.0055, 0.004, (BZ_FRONT - 0.002, BZ_END - CAP - 0.012, BZ_H / 2), axis="x").moved(t), "trim"))
    return v


def _bezel() -> m.Part:
    fp = _bezel_looks("")
    for i, (z0, z1) in enumerate(TAB_Z):
        fp.append((f"tab{i}", _bx("", TAB_X, (-BZ_END, -BZ_END - TAB_OUT), (z0, z1)), "tab"))
    fc = [
        _cb("front_c", (BZ_FRONT, BZ_FRONT + SKIN), (-BZ_END + CAP, BZ_END - CAP), (0.0, BZ_H)),
        _cb("top_c", (BZ_FRONT + SKIN, BZ_BACK), (-BZ_END + CAP, BZ_END - CAP), (BZ_H - SKIN, BZ_H)),
        _cb("bot_c", (BZ_FRONT + SKIN, BZ_BACK), (-BZ_END + CAP, BZ_END - CAP), (0.0, SKIN)),
        _cb("lip_t_c", (BZ_BACK - SKIN, BZ_BACK), (-BZ_END + CAP, BZ_END - CAP), (BZ_H - LIP_H, BZ_H - SKIN)),
        _cb("lip_b_c", (BZ_BACK - SKIN, BZ_BACK), (-BZ_END + CAP, BZ_END - CAP), (SKIN, LIP_H)),
        _cb("cap_l_c", (BZ_FRONT, BZ_BACK), (BZ_END - CAP, BZ_END), (0.0, BZ_H)),
        _cb("cap_r_c", (BZ_FRONT, BZ_BACK), (-BZ_END, -BZ_END + CAP), (0.0, BZ_H)),
    ]
    for i, (z0, z1) in enumerate(TAB_Z):
        fc.append(_cb(f"tab{i}_c", TAB_X, (-BZ_END, -BZ_END - TAB_OUT), (z0, z1)))
    # the latch tongue: a nose out of the left end face, ramped on its back so pressing the bezel home cams it in,
    # square at its front so it holds behind the catch lip; the release button is its pad on the left end cap's front
    x0, x1 = NOSE_X
    y0, y1 = BZ_END, BZ_END + NOSE_OUT
    ramp = [(x0, y0), (x1, y0), (x1, y0 + 0.0010), (x0 + 0.0020, y1), (x0, y1)]
    nose = m.prism(ramp, NOSE_Z[0], NOSE_Z[1])
    body = _bx("", (x0 - 0.001, x1), (BZ_END - 0.016, BZ_END), (NOSE_Z[0] + 0.001, NOSE_Z[1] - 0.001))
    pad = _bx("", (BZ_FRONT - 0.0025, BZ_FRONT), (BZ_END - 0.0145, BZ_END - 0.0015), (BZ_H / 2 - 0.007, BZ_H / 2 + 0.007))
    latch = m.slider("latch", "Y", -LATCH_TRAVEL, 0.0,
                     [("nose", nose, "latch"), ("body", body, "latch"), ("button", pad, "latch")],
                     [m.col_hull("nose_c", nose), m.col_hull("button_c", pad)],
                     at=((x0 + x1) / 2, BZ_END, sum(NOSE_Z) / 2), mass=0.01, spring=LATCH_K, damping=1.0, friction=0.01, effort=50.0)
    ixx = BZ_MASS * BZ_L ** 2 / 12
    return m.mechanism("bezel", (START_X, 0.0, TOP + 0.0005), [latch], LOOKS, frame_parts=fp, frame_colliders=fc,
                       friction=(0.5, 0.4), free_mass=BZ_MASS, frame_inertia=(ixx, BZ_MASS * (BZ_D ** 2 + BZ_H ** 2) / 12, ixx), solver_iters=48)


# -- the rack: two front posts, the lower R650 with its bezel, the task R650 with its slots and window -------------------


def _ear(side: float, z0: float, pockets, px, depth, tag: str, cols: list):
    """Colliders of one control panel (|y| EAR_IN..EAR_OUT, x 0..EAR_D, z z0..z0+CH_H) with pockets in its inner face."""
    yi, yo = side * EAR_IN, side * EAR_OUT
    zt = z0 + CH_H
    if not pockets:
        cols.append(_cb(f"{tag}_c", (0.0, EAR_D), (yi, yo), (z0, zt)))
        return
    yd = side * (EAR_IN + depth)
    cols.append(_cb(f"{tag}_front_c", (0.0, px[0]), (yi, yo), (z0, zt)))
    cols.append(_cb(f"{tag}_back_c", (px[1], EAR_D), (yi, yo), (z0, zt)))
    cols.append(_cb(f"{tag}_outer_c", px, (yd, yo), (z0, zt)))
    edges = [z0] + [z0 + v for p in pockets for v in p] + [zt]
    for k in range(0, len(edges), 2):
        if edges[k + 1] - edges[k] > 1e-5:
            cols.append(_cb(f"{tag}_web{k}_c", px, (yi, yd), (edges[k], edges[k + 1])))


def _server(z0: float, tag: str, task: bool, vis: list, cols: list):
    """One R650 front, in the rack's frame (x from the panels' front faces, z from the table top)."""
    zt = z0 + CH_H
    vis.append((f"{tag}_chassis", _bx("", (DRIVE_X, DRIVE_X + CHASSIS_D), (-EAR_IN, EAR_IN), (z0, zt)), "steel"))
    cols.append(_cb(f"{tag}_chassis_c", (DRIVE_X, DRIVE_X + CHASSIS_D), (-EAR_IN, EAR_IN), (z0, zt)))
    # drive bay face: ten 2.5-inch carriers in two rows, each with its orange release button
    vis.append((f"{tag}_bay", _bx("", (DRIVE_X - 0.0005, DRIVE_X), (-EAR_IN + 0.004, EAR_IN - 0.004), (z0 + 0.001, zt - 0.001)), "drive"))
    for row, zz in enumerate((z0 + 0.0115, z0 + 0.0315)):
        for k in range(5):
            yc = -0.165 + k * 0.074
            vis.append((f"{tag}_btn{row}{k}", m.cylinder(0.0028, 0.002, (DRIVE_X - 0.0015, yc - 0.028, zz), axis="x"), "orange"))
    for s, side in ((1.0, "l"), (-1.0, "r")):
        vis.append((f"{tag}_ear_{side}", _bx("", (0.0, EAR_D), (s * EAR_IN, s * EAR_OUT), (z0, zt)), "panel"))
        vis.append((f"{tag}_flange_{side}", _bx("", (EAR_D - 0.003, EAR_D), (s * EAR_OUT, s * (EAR_OUT + 0.0003)), (z0, zt)), "panel"))
        cols.append(_cb(f"{tag}_stop_{side}_c", (STOP_X, DRIVE_X + 0.001), (s * STOP_Y[0], s * STOP_Y[1]), (z0 + 0.002, zt - 0.002)))
        vis.append((f"{tag}_stop_{side}", _bx("", (STOP_X, DRIVE_X), (s * STOP_Y[0], s * STOP_Y[1]), (z0 + 0.002, zt - 0.002)), "drive"))
    vis.append((f"{tag}_leds", _bx("", (-0.0004, 0.0), (EAR_IN + 0.009, EAR_IN + 0.013), (z0 + 0.008, z0 + 0.034)), "led"))
    vis.append((f"{tag}_power", m.cylinder(0.0035, 0.001, (-0.0005, -(EAR_IN + 0.012), z0 + 0.031), axis="x"), "green"))
    vis.append((f"{tag}_usb", _bx("", (-0.0004, 0.0), (-(EAR_IN + 0.016), -(EAR_IN + 0.008)), (z0 + 0.010, z0 + 0.0145)), "hole"))
    if task:
        _ear(-1.0, z0, SLOTS_Z, SLOT_X, SLOT_DEPTH, f"{tag}_ear_r", cols)
        _ear(1.0, z0, (WIN_Z,), WIN_X, WIN_DEPTH, f"{tag}_ear_l", cols)
        for i, (a, b) in enumerate(SLOTS_Z):  # the slot mouths, dark, so they can be seen
            vis.append((f"{tag}_slot{i}", _bx("", (SLOT_X[0], SLOT_X[1]), (-EAR_IN + 0.0002, -EAR_IN - SLOT_DEPTH), (z0 + a, z0 + b)), "hole"))
        vis.append((f"{tag}_window", _bx("", WIN_X, (EAR_IN - 0.0002, EAR_IN + WIN_DEPTH), (z0 + WIN_Z[0], z0 + WIN_Z[1])), "hole"))
    else:
        _ear(-1.0, z0, (), None, None, f"{tag}_ear_r", cols)
        _ear(1.0, z0, (), None, None, f"{tag}_ear_l", cols)
        # its bezel, fitted and latched
        bz = z0 + DZ_BZ
        vis.extend(_bezel_looks(f"{tag}_bz_", dx=-BZ_PROUD - BZ_FRONT, dz=bz))
        cols.append(_cb(f"{tag}_bezel_c", (-BZ_PROUD, -BZ_PROUD + BZ_D), (-BZ_END, BZ_END), (bz, bz + BZ_H)))


def _rack() -> m.Part:
    vis, cols = [], []
    for s, side in ((1.0, "l"), (-1.0, "r")):
        y0, y1 = s * POST_IN, s * (POST_IN + POST_W)
        vis.append((f"post_{side}", _bx("", (POST_X0, POST_X0 + POST_D), (y0, y1), (0.0, POST_H)), "post"))
        cols.append(_cb(f"post_{side}_c", (POST_X0, POST_X0 + POST_D), (y0, y1), (0.0, POST_H)))
        # EIA-310 square holes, three per unit, in the empty units above the servers
        for u in range(2, 8):
            for d in (0.00635, 0.02223, 0.0381):
                zc = U_LOW - TOP + u * U + d
                vis.append((f"hole_{side}{u}{int(d * 1e5)}", _bx("", (POST_X0 - 0.0003, POST_X0), (s * (EAR_OUT + 0.004), s * (EAR_OUT + 0.0135)), (zc - 0.0048, zc + 0.0048)), "hole"))
    _server(U_LOW - TOP + CH_GAP, "low", False, vis, cols)
    _server(U_TASK - TOP + CH_GAP, "srv", True, vis, cols)
    return m.fixed("rack", (XF, 0.0, TOP), vis, cols, LOOKS, friction=(0.5, 0.4))


BEZEL = _bezel()
RACK = _rack()
# distractors on the bench: the Torx T30 screwdriver Dell lists for the R650, a utility knife and a roll of tape
TORX = SimReady("torx_t30", pos=(0.13, -0.22, TOP), size=(0.224, 0.038, 0.036), mass=0.06, package="handTool_screwdriver_torx_t30_a01",
                usd="sm_handTool_screwdriver_torx_t30_a01_01.usd", yaw=math.radians(90.0))
KNIFE = SimReady("utility_knife", pos=(0.12, 0.12, TOP), size=(0.160, 0.061, 0.026), mass=0.12, package="handTool_knife_retractableBlade_a01",
                 usd="sm_handTool_knife_retractableBlade_a01_01.usd", yaw=math.radians(90.0))
TAPE = SimReady("tape", pos=(0.14, 0.33, TOP), size=(0.10, 0.10, 0.05), mass=0.10, package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd")
OBJECTS = [RACK, BEZEL, TORX, KNIFE, TAPE]


# -- geometry for scoring -------------------------------------------------------------------------------------------------


def _world(s, p):
    return s.to_world(np.atleast_2d(np.asarray(p, float)))


def measure(s) -> dict:
    """Everything the score needs, from the bezel's pose and latch (world and server frames)."""
    srv0 = np.array([XF, 0.0, U_TASK + CH_GAP])
    # right tabs: their tips' depth into the panel, and whether their centres are inside the slot pockets
    tabs_in = []
    for z0, z1 in TAB_Z:
        tip = _world(s, ((TAB_X[0] + TAB_X[1]) / 2, -BZ_END - TAB_OUT, (z0 + z1) / 2))[0] - srv0
        corners = _world(s, [(TAB_X[0], -BZ_END - TAB_OUT, z0), (TAB_X[1], -BZ_END - TAB_OUT, z1)]) - srv0
        depth = -EAR_IN - tip[1]
        inside = (SLOT_X[0] - 0.0005 <= corners[:, 0].min() and corners[:, 0].max() <= SLOT_X[1] + 0.0005)
        zin = any(a - 0.001 <= tip[2] <= b + 0.001 for a, b in SLOTS_Z)
        tabs_in.append(bool(depth >= TAB_IN_MIN and inside and zin))
    ends = _world(s, [(BZ_FRONT, BZ_END - 0.025, 0.0), (BZ_FRONT, -BZ_END + 0.025, 0.0)])
    flush = [float(e[0] - (XF - BZ_PROUD)) for e in ends]
    drop = [float(e[2] - BZ_Z) for e in ends]
    latch = s.joint("latch")
    nose = _world(s, [(NOSE_X[0], BZ_END + NOSE_OUT + latch, sum(NOSE_Z) / 2)])[0] - srv0
    nose_in = float(nose[1] - EAR_IN)
    nose_behind = float(nose[0] - WIN_X[0])
    return {"tabs": tabs_in, "flush": flush, "drop": drop, "latch": float(latch), "nose_in": nose_in, "nose_behind": nose_behind}


def clearance(s, tips: np.ndarray) -> float:
    """Closest fingertip point to the bezel's box (its tabs and latch included), m."""
    p = s.to_local(tips)
    lo = np.array([BZ_FRONT - 0.003, -BZ_END - TAB_OUT, 0.0])
    hi = np.array([BZ_BACK, BZ_END + NOSE_OUT, BZ_H])
    d = np.maximum(np.maximum(lo - p, p - hi), 0.0)
    return float(np.linalg.norm(d, axis=1).min())


class Trial:
    """Per-step scorer. The record of a dropped bezel is cleared only by a reset (on_reset)."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:   # the ONLY place a recorded harm is cleared
        self.dropped = False
        self.still = 0
        self.phase = "wait"      # wait -> pull (the 5 N retention check) -> passed
        self.pull_k = 0
        self.pull_ref = 0.0
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.info: dict = {}
        if env is not None:
            m.set_target(env, "bezel", "latch", LATCH_PRELOAD)  # the latch spring presses the tongue out against its stop

    def _stop_pull(self, env, phase: str) -> None:
        m.push(env, "bezel", (0.0, 0.0, 0.0))
        self.phase = phase

    def step(self, env) -> None:
        s = m.state(env, "bezel")
        if float(s.pos[2]) < LOST_Z:
            self.dropped = True
        g = measure(s)
        hooked = all(g["tabs"])
        closed = hooked and all(abs(f) <= FLUSH_TOL for f in g["flush"]) and all(abs(d) <= HEIGHT_TOL for d in g["drop"])
        latched = (closed and g["latch"] >= -LATCH_OUT_TOL and g["nose_in"] >= NOSE_HOLD_MIN
                   and g["nose_behind"] >= NOSE_BEHIND_MIN)
        clear = clearance(s, m.fingertips(env))
        moving = float(np.linalg.norm(s.lin_vel)) > STILL_V or float(np.linalg.norm(s.ang_vel)) > STILL_W
        self.still = 0 if moving else self.still + 1
        hands_off = clear >= CLEAR
        if self.phase == "wait":
            if latched and hands_off and self.still >= STILL_STEPS and not self.dropped:
                self.pull_ref, self.pull_k = g["flush"][0], 0
                at = s.to_world(np.array([[BZ_FRONT, BZ_END - CAP / 2, BZ_H / 2]]))[0]
                m.push(env, "bezel", (-PULL_N, 0.0, 0.0), at=at)   # pulled straight out of the rack
                self.phase = "pull"
        elif self.phase == "pull":
            self.pull_k += 1
            if not hands_off or not latched or g["flush"][0] - self.pull_ref < -PULL_MOVE:
                self._stop_pull(env, "wait")       # it came out (or a hand interfered): the check starts over
            elif self.pull_k >= PULL_STEPS:
                self._stop_pull(env, "passed")
        elif self.phase == "passed" and not (latched and hands_off):
            self.phase = "wait"
        ok = self.phase == "passed" and latched and hands_off and not self.dropped
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS
        if self.dropped or not hooked:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif closed and hands_off:
            self.score = 0.5
        else:
            self.score = 0.25
        self.info = {**g, "hooked": hooked, "closed": closed, "latched": latched, "clear": clear, "still": self.still,
                     "phase": self.phase, "hold": self.hold, "dropped": self.dropped, "score": self.score}


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- demo: a fixed plan from the published starting layout ------------------------------------------------------------------

OPEN, SHUT = -0.030, 0.0
YAW_L, YAW_R = -math.pi / 2, math.pi / 2       # both grips close across the bezel's depth
GRIP_Y = 0.150                                  # each arm pinches the top edge 15 cm from the middle
OVER = 0.001                                    # TCP this far above the top edge: the pads cover its upper 12 mm
PIVOT = np.array([BZ_X + (TAB_X[0] + TAB_X[1]) / 2, -BZ_END - TAB_OUT / 2])  # the right tabs, fitted
THETA = 8.0                                     # the left end swung this far open while the tabs go in
DZ_HIGH = 0.018                                 # carried this far above its fitted height (clear of the lower bezel)
DZ_SHUT = 0.0050                                # while it swings shut and clicks
PRESS = 0.030                                   # the press aims this far past the fitted plane (the frame stops the bezel)
PRESS_Y = 0.185                                 # pressed 3 cm in from its left end (nearer, the wrist meets the rack post)
PRESS_Z = BZ_Z + 0.024                          # TCP height: the pads cover the bezel's middle, where the latch is
DZ_SLOT = 0.0070                                # commanded height at the slots; the right end hangs about 2.5 mm low in the pinch


def bezel_pose(theta_deg=0.0, along=0.0, dx=0.0, dz=0.0):
    """The bezel's origin (x, y, z) and yaw: fitted, swung open theta about its right tabs, slid `along` its length toward
    its left end, then moved dx, dz."""
    a = math.radians(theta_deg)
    c, s_ = math.cos(a), math.sin(a)
    o = np.array([BZ_X, 0.0]) - PIVOT
    o = PIVOT + np.array([c * o[0] - s_ * o[1], s_ * o[0] + c * o[1]]) + along * np.array([-s_, c])
    return np.array([o[0] + dx, o[1], BZ_Z + dz]), a


def grip_at(pose, y_local, start=False):
    """The TCP over the bezel's top edge at y_local, for a bezel at `pose` (or standing at its start)."""
    if start:
        return (START_X, y_local, TOP + 0.0005 + BZ_H + OVER)
    (ox, oy, oz), a = pose
    return (ox - math.sin(a) * y_local, oy + math.cos(a) * y_local, oz + BZ_H + OVER)


def demo():
    L, R = [], []
    up = 0.06

    def both(t, pose, finger, label, start=False):
        a = 0.0 if start else pose[1]
        L.append(Waypoint(t, grip_at(pose, GRIP_Y, start), finger, label, YAW_L + a))
        R.append(Waypoint(t, grip_at(pose, -GRIP_Y, start), finger, label, YAW_R + a))

    def lift(p, h):
        return (p[0], p[1], p[2] + h)

    s0 = grip_at(None, GRIP_Y, True)
    L.append(Waypoint(2.5, lift(s0, up), OPEN, "over the bezel's top edge", YAW_L))
    R.append(Waypoint(2.5, lift(grip_at(None, -GRIP_Y, True), up), OPEN, "over the bezel's top edge", YAW_R))
    L.append(Waypoint(1.5, s0, OPEN, "down to the edge", YAW_L))
    R.append(Waypoint(1.5, grip_at(None, -GRIP_Y, True), OPEN, "down to the edge", YAW_R))
    L.append(Waypoint(0.8, s0, SHUT, "pinch", YAW_L))
    R.append(Waypoint(0.8, grip_at(None, -GRIP_Y, True), SHUT, "pinch", YAW_R))
    # lift clear of the table, carry it to the rack swung 8 degrees open, slid 6 mm toward its left end and 2.5 cm out
    lifted = (np.array([START_X, 0.0, BZ_Z + 0.012]), 0.0)
    L.append(Waypoint(1.5, grip_at(lifted, GRIP_Y), SHUT, "lift", YAW_L))
    R.append(Waypoint(1.5, grip_at(lifted, -GRIP_Y), SHUT, "lift", YAW_R))
    # carried high enough to pass over the lower server's bezel, in beside the right panel, then down level with the slots
    both(3.0, bezel_pose(THETA, 0.006, -0.025, DZ_HIGH), SHUT, "carry to the rack")
    both(1.5, bezel_pose(THETA, 0.006, 0.0, DZ_HIGH), SHUT, "right end in beside the panel, above the lower bezel")
    both(1.2, bezel_pose(THETA, 0.006, 0.0, DZ_SLOT), SHUT, "down level with the slots")
    both(1.5, bezel_pose(THETA, 0.0, 0.0, DZ_SLOT), SHUT, "tabs into the slots")
    both(0.5, bezel_pose(THETA, 0.0, 0.0, DZ_SLOT), SHUT, "settle")
    # the right hand lets go; the left keeps its pinch and swings the bezel shut about its tabs, through the click
    hooked = bezel_pose(THETA, 0.0, 0.0, DZ_SLOT)
    rg = grip_at(hooked, -GRIP_Y)
    R.append(Waypoint(0.8, rg, -0.017, "let go", YAW_R + hooked[1]))
    R.append(Waypoint(1.0, lift(rg, 0.025), -0.017, "up off the edge", YAW_R + hooked[1]))
    R.append(Waypoint(1.0, (rg[0] - 0.05, rg[1], rg[2] + 0.05), -0.017, "back", YAW_R + hooked[1]))
    R.append(Waypoint(2.0, (0.30, -0.22, TOP + 0.17), -0.017, "out of the way", YAW_R))
    L.append(Waypoint(2.0, grip_at(hooked, GRIP_Y), SHUT, "hold while the right hand leaves", YAW_L + hooked[1]))
    for th, dz in ((6.5, DZ_SLOT), (5.0, DZ_SLOT), (3.5, DZ_SLOT), (2.5, DZ_SLOT), (1.5, DZ_SHUT), (0.8, DZ_SHUT)):
        p = bezel_pose(th, 0.0, 0.0, dz)
        L.append(Waypoint(0.7, grip_at(p, GRIP_Y), SHUT, "swing it shut", YAW_L + p[1]))
    # with the latch nose against the catch, let go and press the left end home at the latch's height
    near = bezel_pose(0.8, 0.0, 0.0, DZ_SHUT)
    lg = grip_at(near, GRIP_Y)
    L.append(Waypoint(0.8, lg, -0.017, "let go at the catch", YAW_L + near[1]))
    L.append(Waypoint(1.0, lift(lg, 0.025), -0.017, "up off the edge", YAW_L + near[1]))
    (ox, oy, oz), a = near
    front = ox - math.sin(a) * PRESS_Y + math.cos(a) * BZ_FRONT
    L.append(Waypoint(1.2, (front - 0.030, PRESS_Y, PRESS_Z + 0.05), SHUT, "in front of the left end", YAW_L))
    L.append(Waypoint(1.0, (front - 0.030, PRESS_Y, PRESS_Z), SHUT, "down to the latch's height", YAW_L))
    home = XF - BZ_PROUD - 0.009
    L.append(Waypoint(1.5, (home + PRESS, PRESS_Y, PRESS_Z), SHUT, "press it home until it clicks", YAW_L))
    L.append(Waypoint(0.8, (home + PRESS, PRESS_Y, PRESS_Z), SHUT, "hold", YAW_L))
    L.append(Waypoint(1.0, (home - 0.040, PRESS_Y, PRESS_Z), SHUT, "back off", YAW_L))
    L.append(Waypoint(1.2, (home - 0.060, PRESS_Y, PRESS_Z + 0.08), SHUT, "up and away", YAW_L))
    L.append(Waypoint(6.0, (home - 0.060, PRESS_Y, PRESS_Z + 0.08), SHUT, "wait", YAW_L))
    t_l = sum(w.duration for w in L)
    t_r = sum(w.duration for w in R)
    R.append(Waypoint(max(0.5, t_l - t_r), (0.30, -0.22, TOP + 0.17), -0.017, "wait", YAW_R))
    return {"left": L, "right": R}


# -- debug dump (temporary) -------------------------------------------------------------------------------------------
import json as _json  # noqa: E402
import os as _os  # noqa: E402

_DBG = {"t": 0, "rows": []}


def _debug(env) -> None:
    path = _os.environ.get("BZ_DEBUG")
    if not path:
        return
    s = m.state(env, "bezel")
    _DBG["t"] += 1
    yaw = math.degrees(math.atan2(s.rotation()[1, 0], s.rotation()[0, 0]))
    info = {k: (v if not isinstance(v, (np.floating, np.bool_)) else v.item()) for k, v in TRIAL.info.items()}
    tips = m.fingertips(env)
    tcp = [np.r_[tips[k:k + 12:2].mean(axis=0)[:2], tips[k:k + 12:2, 2].mean() + 0.013].round(5).tolist() for k in (0, 12)]
    _DBG["rows"].append({"t": _DBG["t"], "pos": [round(float(v), 5) for v in s.pos], "yaw": round(yaw, 3), "tilt": round(s.tilt_deg(), 3),
                         "tcp": tcp, "info": info})
    if _DBG["t"] % 10 == 0 or _DBG["t"] < 3:
        with open(path, "w") as f:
            _json.dump(_DBG["rows"], f, default=float)


m.on_step(_debug)
