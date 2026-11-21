"""A 24-inch TV, its two mount brackets already bolted on, stands on the TV console leaning against the wall just
under its tilting wall plate. Both brackets hook over the plate's top rail, blind behind the panel, and their lower
ends come to rest against the plate's bottom rail, so the TV hangs flat against the plate.

The TV is an LG 24LQ520S-PU (22.2 x 13.4 x 2.3 in, 7.2 lb without its stand, VESA 75 x 75, LG's spec sheet), at
full size and its real mass. The mount is a Monoprice EZ Series tilt wall mount, model 5915 (wall plate 17.5 x 8.5
in, the TV 1.9 in out from the wall, VESA 75 x 75 and up), at full size. The wall is a 92 x 56 cm wall section
standing on the station table, which stands in for the console top: the plate is lag-bolted to it so that the hung
TV clears the console by 4 cm.

World frame: x forward from the arms (towards the wall), y left, z up. The TV's own frame: origin at the middle of
its bottom edge on the screen face, x towards its back (the wall), y and z as the world's when it hangs.
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Hang the TV on its wall mount."

# -- the TV (LG 24LQ520S-PU, full size, real mass) ---------------------------------------------------------------
TV_W, TV_H, TV_D = 0.5639, 0.3404, 0.0584  # 22.2 x 13.4 x 2.3 in, LG's spec sheet, without the stand
PANEL_D = 0.020  # the panel and its bezel rim all round the edge: what the hands hold (approximate)
HOUSING = (TV_D - PANEL_D, 0.400, 0.250)  # the rear housing bulge, depth x width x height (approximate)
HOUSING_Z0 = 0.045  # its bottom, above the TV's bottom edge
TV_MASS = float(__import__("os").environ.get("PROBE_MASS", "3.27"))
TV_COM = (0.020, 0.0, TV_H / 2)  # centre of mass, TV frame (approximate: the panel is most of the mass)
VESA = 0.075  # 75 x 75 mm pattern: the two brackets 75 mm apart
ZV = 0.180  # VESA centre above the TV's bottom edge (approximate)

# -- the mount (Monoprice 5915, full size) ---------------------------------------------------------------------
STANDOFF = 0.048  # 1.9 in: wall to the TV's back
PLATE_W, PLATE_H = 0.4445, 0.216  # 17.5 x 8.5 in
T = 0.002  # sheet steel
FLANGE = 0.020  # each rail stands this far out from the wall
LIP = 0.014  # and turns up this far: a channel 12 mm deep behind the lip
BR_D, BR_W = 0.020, 0.025  # bracket: a 20 mm deep steel channel, 25 mm wide
BR_LOW, BR_HIGH = ZV - 0.125, ZV + 0.111  # bracket ends, TV frame
HOOK_T = 0.003  # hook tab thickness
FINGER = (0.004, 0.010)  # the hook's down-turned finger: thick (x) x long (z)

# -- where things are --------------------------------------------------------------------------------------------
X_WALL = 0.450  # the wall's face
B_H = 0.025  # the hung TV's bottom edge above the console
Z_P = ZV + B_H  # the plate's centre above the table
X_TV = X_WALL - STANDOFF - TV_D  # the hung TV's screen face (its frame origin's x)
Y0 = 0.025  # the TV starts this far left of the pencil mark
TOP_LIP = Z_P + PLATE_H / 2  # top rail's lip top, above the table
LOW_LIP = Z_P - PLATE_H / 2 + LIP  # bottom rail's lip top
LOW_BASE = Z_P - PLATE_H / 2  # bottom rail's underside

# TV frame x of the wall when hung, and of the hook finger
XW_T = TV_D + STANDOFF
FIN_X1 = XW_T - 0.010
FIN_X0 = FIN_X1 - FINGER[0]
TAB_Z = ZV + PLATE_H / 2  # the hook tab's underside, TV frame: rests on the top lip
FOOT_X1 = XW_T - FLANGE - 0.0005  # the bracket foot's wall-side face, TV frame: 0.5 mm off the bottom lip when plumb
FOOT_Z = ZV - 0.104  # the foot's centre height, TV frame

LOOKS = {
    "bezel": m.Look(color=m.rgb(0.035, 0.035, 0.04), roughness=0.45),
    "screen": m.Look(color=m.rgb(0.012, 0.014, 0.018), roughness=0.08, metallic=0.1),
    "housing": m.Look(color=m.rgb(0.06, 0.06, 0.065), roughness=0.7),
    "steel": m.Look(color=m.rgb(0.10, 0.10, 0.11), metallic=0.6, roughness=0.45),
    "logo": m.Look(color=m.rgb(0.55, 0.55, 0.58), metallic=0.5, roughness=0.4),
    "wall": m.Look(color=m.rgb(0.86, 0.84, 0.80), roughness=0.9),
    "pencil": m.Look(color=m.rgb(0.25, 0.25, 0.27), roughness=0.9),
}


def _tv():
    parts, cols = [], []
    panel = ((PANEL_D, TV_W, TV_H), (PANEL_D / 2, 0.0, TV_H / 2))
    parts.append(("panel", m.box(*panel), "bezel"))
    parts.append(("screen", m.box((0.001, TV_W - 0.020, TV_H - 0.030), (-0.0004, 0.0, TV_H / 2 + 0.004)), "screen"))
    parts.append(("logo", m.box((0.0012, 0.030, 0.005), (-0.0004, 0.0, 0.007)), "logo"))
    cols.append(m.col_box("c_panel", *panel))
    hz = HOUSING_Z0 + HOUSING[2] / 2
    housing = (HOUSING, (PANEL_D + HOUSING[0] / 2, 0.0, hz))
    parts.append(("housing", m.box(*housing), "housing"))
    cols.append(m.col_box("c_housing", *housing))
    for tag, y in (("l", VESA / 2), ("r", -VESA / 2)):
        body = ((BR_D, BR_W, BR_HIGH - BR_LOW), (TV_D + BR_D / 2, y, (BR_LOW + BR_HIGH) / 2))
        tab = ((FIN_X1 - TV_D - BR_D, BR_W, HOOK_T), ((TV_D + BR_D + FIN_X1) / 2, y, TAB_Z + HOOK_T / 2))
        fin = ((FINGER[0], BR_W, FINGER[1]), ((FIN_X0 + FIN_X1) / 2, y, TAB_Z - FINGER[1] / 2))
        # the bracket's lower end bears on the bottom rail's lip when the TV hangs plumb (0.5 mm short of it)
        foot = ((FOOT_X1 - TV_D - BR_D, BR_W, 0.016), ((TV_D + BR_D + FOOT_X1) / 2, y, FOOT_Z))
        for nm, b in (("bracket", body), ("tab", tab), ("finger", fin), ("foot", foot)):
            parts.append((f"{nm}_{tag}", m.box(*b), "steel"))
            cols.append(m.col_box(f"c_{nm}_{tag}", *b))
        for dz in (-VESA / 2, VESA / 2):  # the four VESA bolt heads
            parts.append((f"bolt_{tag}{'u' if dz > 0 else 'd'}", m.cylinder(0.006, 0.004, (TV_D + BR_D + 0.002, y, ZV + dz), axis="x"), "steel"))
    return parts, cols


_tp, _tc = _tv()
TV = m.free("tv", (X_TV, Y0, TOP + 0.001), _tp, _tc, LOOKS, TV_MASS, friction=(0.8, 0.7), solver_iters=48, com=TV_COM)


def _mount():
    parts, cols = [], []
    pieces = {
        "web": ((T, PLATE_W, PLATE_H), (-T / 2, 0.0, Z_P)),
        "top_flange": ((FLANGE - T, PLATE_W, T), (-(FLANGE + T) / 2, 0.0, Z_P + PLATE_H / 2 - LIP + T / 2)),
        "top_lip": ((T, PLATE_W, LIP), (-FLANGE - T / 2 + T, 0.0, Z_P + PLATE_H / 2 - LIP / 2)),
        "low_flange": ((FLANGE - T, PLATE_W, T), (-(FLANGE + T) / 2, 0.0, Z_P - PLATE_H / 2 + T / 2)),
        "low_lip": ((T, PLATE_W, LIP), (-FLANGE - T / 2 + T, 0.0, Z_P - PLATE_H / 2 + LIP / 2)),
    }
    for nm, (size, c) in pieces.items():
        parts.append((nm, m.box(size, c), "steel"))
        cols.append(m.col_box(f"c_{nm}", size, c))
    for i, y in enumerate((-0.2032, 0.2032)):  # lag bolts into the studs, 16 in apart
        for j, z in enumerate((Z_P + 0.06, Z_P - 0.06)):
            parts.append((f"lag{i}{j}", m.cylinder(0.009, 0.005, (-T - 0.0025, y * 0.9, z), axis="x"), "steel"))
    return parts, cols


_mp, _mc = _mount()
MOUNT = m.fixed("mount", (X_WALL, 0.0, TOP), _mp, _mc, LOOKS, friction=(0.4, 0.3))

MARK_Z = B_H + TV_H + 0.020  # the pencil mark, 2 cm above where the hung TV's top edge goes
_wall = ((0.020, 0.92, 0.56), (0.010, 0.0, 0.28))
WALL = m.fixed("wall", (X_WALL, 0.0, TOP), [("board", m.box(*_wall), "wall"),
                                             ("mark", m.box((0.0006, 0.0025, 0.030), (-0.0003, 0.0, MARK_Z)), "pencil"),
                                             ("tick", m.box((0.0006, 0.020, 0.0025), (-0.0003, 0.0, MARK_Z)), "pencil")],
               [m.col_box("c_board", *_wall)], LOOKS, friction=(0.8, 0.7))
MARK_Y = 0.0  # the pencil mark: where the TV's centre goes

DISTRACTORS = [
    SimReady("screwdriver", pos=(0.20, 0.12, TOP), size=(0.188, 0.034, 0.032), mass=0.10, package="handTool_screwdriver_phillips_ph2_a01",
             usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd", yaw=math.radians(25), convex=True),
    SimReady("candle", pos=(0.21, -0.14, TOP), size=(0.074, 0.074, 0.075), mass=0.35, package="decor_candle_a01",
             usd="sm_decor_candle_a01_01.usd", convex=True),
    SimReady("manual", pos=(0.12, -0.01, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01",
             usd="sm_misc_notebook_a01_01.usd", yaw=math.radians(-10), convex=True),
]

OBJECTS = [WALL, MOUNT, TV, *DISTRACTORS]

# -- scoring -------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 0.0  # the Trial counts its own hold
SCORE_OVER = "end"
HOLD_STEPS = 20  # 1 s at 20 Hz
CENTRE_TOL = 0.015  # m from the pencil mark
LEVEL_TOL = 2.0  # degrees, either way
FOOT_TOL = 0.003  # each bracket's lower end within this of the bottom rail's lip
CLEAR = 0.04  # every fingertip point at least this far from the TV and its brackets
STILL_V, STILL_W = 0.01, 0.05  # m/s and rad/s
DROP_V = 0.5  # m/s downward: a free fall of 13 mm
FALL_TILT = 30.0  # degrees from upright


def _plate(st, pts):
    """Points of the TV, in the plate's frame: x out of the wall's face (negative in front of it), z up from the table."""
    return st.to_world(np.asarray(pts, float)) - np.array([X_WALL, 0.0, TOP])


def hook_seated(st):
    """Each hook: its finger's bottom inside the top rail's channel (between lip and plate, x), and 6.5-12.5 mm below
    the lip's top: the tab resting on the lip within 3.5 mm."""
    out = []
    for y in (VESA / 2, -VESA / 2):
        p = _plate(st, [((FIN_X0 + FIN_X1) / 2, y, TAB_Z - FINGER[1])])[0]
        inside = -FLANGE + T < p[0] < -T and abs(p[1]) < PLATE_W / 2 - 0.01
        depth = TOP_LIP - p[2]
        out.append(bool(inside and 0.0065 <= depth <= 0.0125))
    return out


def foot_home(st):
    """Each bracket's lower end: its wall-side face within 3 mm of the bottom rail lip's front face, and level with the lip."""
    out = []
    for y in (VESA / 2, -VESA / 2):
        p = _plate(st, [(FOOT_X1, y, FOOT_Z)])[0]
        gap = -FLANGE - p[0]  # how far in front of the lip's face
        out.append(bool(-0.001 <= gap <= FOOT_TOL and LOW_BASE + 0.002 <= p[2] <= LOW_LIP - 0.002 and abs(p[1]) < PLATE_W / 2 - 0.01))
    return out


def level_err(st):
    r = st.rotation()
    roll = math.degrees(math.asin(np.clip(r[2, 1], -1, 1)))  # the TV's width axis against the horizontal
    pitch = math.degrees(math.asin(np.clip(r[2, 0], -1, 1)))  # its depth axis: the tilt
    return roll, pitch


BOX_LO = np.array([0.0, -TV_W / 2, 0.0])
BOX_HI = np.array([XW_T - 0.010, TV_W / 2, TV_H])


def hands_gap(st, tips):
    local = st.to_local(tips)
    d = np.maximum(np.maximum(BOX_LO - local, local - BOX_HI), 0.0)
    return float(np.linalg.norm(d, axis=1).min())


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.dropped, self.fell = False, False
        self.hold, self.full, self.score = 0, False, 0.0
        self.info = {}

    def step(self, env):
        st = m.state(env, "tv")
        tips = m.fingertips(env)
        if st.lin_vel[2] < -DROP_V:
            self.dropped = True
        if st.tilt_deg() > FALL_TILT:
            self.fell = True
        hooks, feet = hook_seated(st), foot_home(st)
        roll, pitch = level_err(st)
        centred = abs(st.pos[1] - MARK_Y) <= CENTRE_TOL
        level = abs(roll) <= LEVEL_TOL and abs(pitch) <= LEVEL_TOL
        gap = hands_gap(st, tips)
        clear = gap >= CLEAR
        still = np.linalg.norm(st.lin_vel) < STILL_V and np.linalg.norm(st.ang_vel) < STILL_W
        harm = self.dropped or self.fell
        ok = all(hooks) and all(feet) and centred and level and clear and still and not harm
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS
        if harm or not clear:
            s = 0.0
        elif self.full:
            s = 1.0
        elif all(hooks):
            s = 0.5
        elif any(hooks):
            s = 0.25
        else:
            s = 0.0
        self.score = s
        self.info = dict(hooks=hooks, feet=feet, y=float(st.pos[1]), roll=roll, pitch=pitch, gap=gap, still=bool(still),
                         dropped=self.dropped, fell=self.fell, x=float(st.pos[0] - X_TV), z=float(st.pos[2] - TOP))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full)


# -- the closed-loop demonstration -------------------------------------------------------------------------------
GRIP_Z = 0.110  # where the hands take the side edges, above the TV's bottom edge (its centre of mass is at 0.17)
GRIP_IN = 0.006  # TCP this far in from the edge: the pads overlap the rim, the gripper body stays outside it
GRIP_L = np.array([PANEL_D / 2, TV_W / 2 - GRIP_IN, GRIP_Z])
GRIP_R = np.array([PANEL_D / 2, -(TV_W / 2 - GRIP_IN), GRIP_Z])
TILT = 40.0  # the grip leans this far from straight down, its fingers closing across the rim (along x)


def demo_script(ctx):
    import demokit as d

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        yield from ctx.until(lambda: False, timeout=30.0, label="do nothing")
        return
    gl, gr = d.tilted(TILT, -90.0, True), d.tilted(TILT, 90.0, True)

    def tv():
        return ctx.state("tv")

    def world(p):
        return tv().to_world(p)[0]

    def both(dl, dr=None, t=1.0, label="both"):
        dr = dl if dr is None else dr
        yield from ctx.together(l.move(l.at() + np.asarray(dl), t=t, label=label), r.move(r.at() + np.asarray(dr), t=t, label=label))

    def unpitch(label="level the TV"):
        """Turn both grips about the line between them until the TV reads plumb (its pose, as a camera sees it)."""
        _, pitch = level_err(tv())
        if abs(pitch) > 0.4:
            yield from ctx.together(l.turn(pitch, axis=(0.0, 1.0, 0.0), t=0.5, label=label), r.turn(pitch, axis=(0.0, 1.0, 0.0), t=0.5, label=label))

    out_l, out_r = np.array([0.0, 0.055, 0.005]), np.array([0.0, -0.055, 0.005])
    front = np.array([-0.07, 0.0, 0.02])
    arms = [(l, GRIP_L, gl, out_l)] + ([] if mode == "onearm" else [(r, GRIP_R, gr, out_r)])
    # out to the side first, in front of the screen, so that the far finger passes outside the TV's edge
    yield from ctx.together(*[a.move(world(g) + o + front, grip=gp, finger=FINGER_OPEN, t=2.0, label="out to the side") for a, g, gp, o in arms])
    yield from ctx.together(*[a.move(world(g) + o, t=1.2, label="beside the edge") for a, g, gp, o in arms])
    yield from ctx.together(*[a.move(world(g), t=1.2, label="onto the edge") for a, g, gp, o in arms])
    yield from ctx.together(*[a.servo(world(g), timeout=1.0, label="onto the edge") for a, g, gp, o in arms])
    for a, g, gp, o in arms:  # DEBUG-REMOVE
        print("[dbg] onto", a.side, "tcp local", np.round(tv().to_local(a.tcp()[None])[0], 4), "goal", g, flush=True)  # DEBUG-REMOVE
    yield from ctx.together(*[a.close(PANEL_D, 40.0, t=0.6, label="pinch the rim") for a, g, gp, o in arms])
    for a, g, gp, o in arms:  # DEBUG-REMOVE
        print("[dbg] closed", a.side, "tcp local", np.round(tv().to_local(a.tcp()[None])[0], 4), flush=True)  # DEBUG-REMOVE
    if mode == "onearm":
        yield from l.move(l.at() + np.array([-0.02, -Y0, 0.06]), t=2.5, label="lift with one hand")
        yield from l.wait(1.0)
        yield from l.open(t=0.5)
        yield from l.move(l.at() + np.array([-0.08, 0.04, 0.04]), t=1.0)
        yield from l.wait(3.0)
        return
    def hold_at(origin, t=1.0, label="carry"):
        """Bring the TV's origin to `origin`: both hands move by the TV's offset from it, then each hand servoes its own grip
        point on the TV (read from the TV's pose, as a camera sees it) to where it belongs, closing out the arms' droop."""
        origin = np.asarray(origin, dtype=float)
        yield from both(origin - tv().pos, t=t, label=label)
        yield from ctx.together(l.servo(origin + GRIP_L, read=lambda: world(GRIP_L), timeout=1.5, tol=0.002, label=label),
                                r.servo(origin + GRIP_R, read=lambda: world(GRIP_R), timeout=1.5, tol=0.002, label=label))

    p0 = tv().pos.copy()
    hook_z = TOP + B_H + 0.006  # the TV's origin when its hooks are 6 mm over the lip
    yield from hold_at(p0 + np.array([0.0, 0.0, 0.008]), t=1.0, label="take the weight")
    for a in (l, r):  # DEBUG-REMOVE
        print("[dbg] weight", a.side, "tcp local", np.round(tv().to_local(a.tcp()[None])[0], 4), "cmd", np.round(a.at() + a.trim, 3), "tcp", np.round(a.tcp(), 3), flush=True)  # DEBUG-REMOVE
    if mode != "careless":
        yield from hold_at(p0 + np.array([-0.022, 0.0, 0.008]), t=1.0, label="hooks off the plate")
        yield from hold_at((X_TV - 0.022, MARK_Y, hook_z), t=2.0, label="lift over the rail, on the mark")
        yield from unpitch()
        yield from hold_at((X_TV - 0.022, MARK_Y, hook_z), t=0.4, label="over the rail")
        yield from hold_at((X_TV, MARK_Y, hook_z), t=1.0, label="hooks in over the channel")
        yield from unpitch()
        yield from hold_at((X_TV, MARK_Y, hook_z), t=0.4, label="hooks over the channel")
    else:  # straight up from where it leans, then in
        yield from hold_at((p0[0], MARK_Y, hook_z), t=2.0, label="lift straight up")
        yield from hold_at((X_TV, MARK_Y, hook_z), t=1.0, label="push to the wall")
    if mode == "drop":  # let go 3 cm over the rail
        yield from both((0.0, 0.0, 0.024), t=0.6, label="too high")
    else:
        yield from both((0.0, 0.0, -0.009), t=1.0, label="lower onto the rail")
        yield from l.wait(0.4)
    yield from ctx.together(l.open(t=0.6, label="let go"), r.open(t=0.6, label="let go"))
    if mode == "holdon":
        yield from ctx.together(l.close(PANEL_D, 40.0, t=0.4), r.close(PANEL_D, 40.0, t=0.4))
        yield from ctx.until(lambda: False, timeout=6.0)
        return
    yield from both((0.0, 0.025, 0.0), (0.0, -0.025, 0.0), t=0.8, label="off the edge")
    yield from both((-0.13, 0.0, 0.03), t=1.5, label="hands back")
    yield from both((0.0, 0.03, 0.0), (0.0, -0.03, 0.0), t=0.8, label="hands aside")
    yield from l.wait(4.0)


import os as _os  # DEBUG-REMOVE
if _os.environ.get("TV_DEBUG"):  # DEBUG-REMOVE
    _k = [0]  # DEBUG-REMOVE

    def _dbg(env):  # DEBUG-REMOVE
        _k[0] += 1  # DEBUG-REMOVE
        if _k[0] % int(_os.environ.get("TV_DEBUG")) == 0:  # DEBUG-REMOVE
            print("[tv]", _k[0], TRIAL.score, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in TRIAL.info.items()}, flush=True)  # DEBUG-REMOVE
            _loc = m.state(env, "tv").to_local(m.fingertips(env))  # DEBUG-REMOVE
            print("[tips] L", np.round(_loc[:12].min(0), 3), np.round(_loc[:12].max(0), 3), "R", np.round(_loc[12:].min(0), 3), np.round(_loc[12:].max(0), 3), flush=True)  # DEBUG-REMOVE
    m.on_step(_dbg)  # DEBUG-REMOVE
