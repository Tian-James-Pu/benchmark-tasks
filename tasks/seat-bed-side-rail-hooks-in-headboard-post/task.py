"""On the station table, standing in for a bedroom floor: the near corner of a solid-wood queen bed frame being put together.
A headboard post (76 mm square hardwood, cropped 212 mm above the floor) stands just left of the table's middle, at x = 0.262 to
0.338 and y = +0.07 to +0.146, its face towards the robot's right, with a stub of the headboard's bottom rail running away from the arms. Mortised flush in that
face is the slotted half of a hook-in bed rail fastener (Rockler 6" heavy-duty wrought steel, 151 x 16 x 3.6 mm): two 6.35 x 25.4 mm
slots, 82.5 mm apart one above the other. The bed's side rail (poplar, 25 x 165 mm, 2.03 m long, 3.8 kg; shown cropped 0.62 m from
its near end) runs along y towards the right of the table: its far end already rests in the footboard, seen here as a steel cradle
that lets it pitch, slide along its length and swing a little. Its near end, with the hook half of the fastener (two steel hooks
4 mm thick, standing 11 mm out of its end), rests on the floor 30 mm short of the post and 25 mm towards the arms. A mug, a
screwdriver and the assembly booklet lie on the floor out of the way.
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

# -- the rail and the layout ----------------------------------------------------------------------------------------------------
RAIL_T = 1.0 * IN  # 25.4 mm
RAIL_H = 0.165
RAIL_LEN = 2.032  # 80 in: a queen side rail
CROP = 0.62  # shown from its near end
RAIL_MASS = 3.8  # poplar at 450 kg/m^3, the whole 2.03 m rail
X_R = 0.30  # the rail's and the post's centre line
Y0 = 0.07  # the post's face: where the rail's end sits when seated
HB = 0.035  # the rail's underside above the floor when seated (post cropped below)
POST_W = 3.0 * IN
POST_TOP = HB + RAIL_H + 0.012
Y_BAR = -0.50  # the cradle's bar, under the rail
BAR_R = 0.010
CHEEK_GAP = 0.0015  # clearance either side of the rail in the cradle

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
    "cut": m.Look(m.rgb(0.86, 0.78, 0.62), roughness=0.85),
    "post": m.Look(m.rgb(0.55, 0.36, 0.21), roughness=0.5),
    "steel": m.Look(m.rgb(0.30, 0.31, 0.33), metallic=0.85, roughness=0.35),
    "screw": m.Look(m.rgb(0.62, 0.62, 0.60), metallic=0.9, roughness=0.3),
    "dark": m.Look(m.rgb(0.04, 0.035, 0.03), roughness=0.95),
    "cradle": m.Look(m.rgb(0.20, 0.21, 0.23), metallic=0.6, roughness=0.5),
}


def _yz_prism(outline_yz, x_half):
    """A solid from a counter-clockwise (y, z) outline, x_half either side of x = 0."""
    p = m.prism(outline_yz, -x_half, x_half)
    return m.Mesh(p.v[:, [2, 0, 1]], p.f)


def _neck(zt):
    return _yz_prism([(0.0, zt - NECK_H), (HOOK_OUT, zt - NECK_H), (HOOK_OUT, zt - 0.002), (HOOK_OUT - 0.002, zt), (0.0, zt)], HOOK_T / 2)


def _tip(zt):
    return _yz_prism([(THROAT, zt - NECK_H), (HOOK_OUT - 0.0008, zt - HOOK_H), (HOOK_OUT, zt - HOOK_H + 0.0005), (HOOK_OUT, zt - NECK_H)], HOOK_T / 2)


def _box(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


# -- the side rail: frame at its end, centre of its thickness, on its underside; it runs along -y --------------------------------
def _rail():
    parts = [
        ("body", _box(-RAIL_T / 2, RAIL_T / 2, -CROP + 0.002, -0.0003, 0.0, RAIL_H), "wood"),
        ("end", _box(-RAIL_T / 2, RAIL_T / 2, -0.0004, 0.0, 0.0, RAIL_H), "endgrain"),
        ("cut", _box(-RAIL_T / 2, RAIL_T / 2, -CROP, -CROP + 0.002, 0.0, RAIL_H), "cut"),
        ("plate", _box(-PLATE_W / 2, PLATE_W / 2, -0.0010, 0.0002, PLATE_Z0, PLATE_Z0 + PLATE_L), "steel"),
    ]
    for k, z in enumerate((PLATE_Z0 + 0.012, PLATE_Z0 + PLATE_L / 2, PLATE_Z0 + PLATE_L - 0.012)):
        parts.append((f"screw{k}", m.cylinder(0.0045, 0.0012, (0.0, 0.0008, z), axis="y", segments=16), "screw"))
    cols = [m.col_box("body_c", (RAIL_T, CROP, RAIL_H), (0.0, -CROP / 2, RAIL_H / 2))]
    for h in HOOKS:
        parts += [(f"neck_{h}", _neck(ZT[h]), "steel"), (f"tip_{h}", _tip(ZT[h]), "steel")]
        cols += [m.col_hull(f"neck_{h}_c", _neck(ZT[h])), m.col_hull(f"tip_{h}_c", _tip(ZT[h]))]
    return parts, cols


def rail_frame(end):
    """The rail's rotation when its end's underside (centre) is at world point `end` and its underside lies on the cradle's bar."""
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


START_END = np.array([X_R - 0.025, Y0 - 0.030, TOP])  # on the floor, 30 mm short of the post, 25 mm towards the arms
START_R = rail_frame(START_END)
RAIL_PARTS, RAIL_COLS = _rail()
RAIL = m.free("rail", tuple(START_END + (0.0, 0.0, 0.001)), RAIL_PARTS, RAIL_COLS, LOOKS, mass=RAIL_MASS, friction=(0.6, 0.5),
              start_quat=_xyzw(START_R), com=(0.0, -(Y0 - Y_BAR) / 2, RAIL_H / 2), solver_iters=48)


# -- the headboard post, its slotted plate and the wedge fit (post frame: world axes, origin on the floor at the post's face) -----
def _post():
    hx, sw, d1, d2 = POST_W / 2, SLOT_W / 2, PLATE_T, PLATE_T + CAVITY
    pz0, pz1 = HB + PLATE_Z0, HB + PLATE_Z0 + PLATE_L
    lo, up = "lower", "upper"
    cols = []

    def cb(name, *b):
        mesh_b = _box(*b)
        cols.append(m.col_box(name, (b[1] - b[0], b[3] - b[2], b[5] - b[4]), ((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2)))
        return mesh_b

    cb("back_c", -hx, hx, d2, POST_W, 0.0, POST_TOP)
    cb("side_l_c", -hx, -sw, 0.0, d2, 0.0, POST_TOP)
    cb("side_r_c", sw, hx, 0.0, d2, 0.0, POST_TOP)
    cb("c0_c", -sw, sw, 0.0, d2, 0.0, CAV_BOT[lo])
    cb("plate_lo_c", -sw, sw, 0.0, d1, CAV_BOT[lo], SLOT_BOT[lo])
    cb("c1_c", -sw, sw, 0.0, d2, SLOT_TOP[lo], CAV_BOT[up])
    cb("plate_up_c", -sw, sw, 0.0, d1, CAV_BOT[up], SLOT_BOT[up])
    cb("c2_c", -sw, sw, 0.0, d2, SLOT_TOP[up], POST_TOP)
    stub = (hx, hx + 0.22, POST_W / 2 - RAIL_T / 2, POST_W / 2 + RAIL_T / 2, HB, HB + RAIL_H)
    cb("stub_c", *stub)
    pw = PLATE_W / 2
    parts = [
        ("post", _box(-hx, hx, d1 + 0.0002, POST_W, 0.0, POST_TOP), "post"),
        ("face_l", _box(-hx, -pw, 0.0, d1 + 0.0003, 0.0, POST_TOP), "post"),
        ("face_r", _box(pw, hx, 0.0, d1 + 0.0003, 0.0, POST_TOP), "post"),
        ("face_lo", _box(-pw, pw, 0.0, d1 + 0.0003, 0.0, pz0), "post"),
        ("face_hi", _box(-pw, pw, 0.0, d1 + 0.0003, pz1, POST_TOP), "post"),
        ("cap", _box(-hx, hx, 0.0, POST_W, POST_TOP - 0.002, POST_TOP), "cut"),
        ("stub", _box(*stub), "post"),
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


# -- the far end's support: the footboard and the rest of the rail, cropped -------------------------------------------------------
def _cradle():
    g = RAIL_T / 2 + CHEEK_GAP
    stop_y = float((START_R @ np.array([0.0, -CROP, 0.0]) + START_END)[1]) - 0.025 - Y_BAR
    pieces = {
        "bar": m.cylinder(BAR_R, 2 * g, (0.0, 0.0, HB - BAR_R), axis="x", segments=24),
        "cheek_l": _box(-g - 0.006, -g, -0.010, 0.010, 0.0, HB + 0.05),
        "cheek_r": _box(g, g + 0.006, -0.010, 0.010, 0.0, HB + 0.05),
        "base": _box(-0.05, 0.05, stop_y - 0.006, 0.03, 0.0, 0.006),
        "stop": _box(-0.03, 0.03, stop_y - 0.006, stop_y, 0.0, HB + 0.06),
    }
    parts = [(k, v, "cradle") for k, v in pieces.items()]
    cols = [m.col_hull(f"{k}_c", v) for k, v in pieces.items()]
    return parts, cols


CRADLE_PARTS, CRADLE_COLS = _cradle()
CRADLE = m.fixed("cradle", (X_R, Y_BAR, TOP), CRADLE_PARTS, CRADLE_COLS, LOOKS, friction=(0.3, 0.25))

OBJECTS = [
    POST,
    RAIL,
    CRADLE,
    # Distractors on the floor, clear of the work (SimReady props), which scoring ignores.
    SimReady("mug", pos=(0.52, 0.40, TOP), size=(0.114, 0.090, 0.080), mass=0.30, yaw=0.6,
             package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda"),
    SimReady("screwdriver", pos=(0.60, -0.27, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=0.5, convex=True,
             package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd"),
    SimReady("booklet", pos=(0.70, 0.27, TOP), size=(0.229, 0.170, 0.010), mass=0.12, yaw=0.3,
             package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd"),
]


# -- reading the joint --------------------------------------------------------------------------------------------------------------
FORCED = 0.0015


def hook_readings(rail):
    """For each hook: (s, gap, e) in metres. s: its top's height above where it sits when seated (the rail's drop still to go);
    gap: the rail's end face from the post's face at the hook's height (positive: short of it); e: its centre's sideways offset
    from its slot's centre."""
    out = {}
    for h in HOOKS:
        top, face = rail.to_world(np.array([[0.0, HOOK_OUT / 2, ZT[h]], [0.0, 0.0, ZT[h] - NECK_H / 2]]))
        out[h] = (float(top[2] - TOP - (SLOT_BOT[h] + NECK_H)), float(Y0 - face[1]), float(top[0] - X_R))
    return out


def hooked(s, gap, e):
    """A hook is in (hooked behind the plate, it cannot come straight out): its tip is behind the plate and below its slot."""
    return gap <= 0.0075 and s <= TIP_H and abs(e) <= SLOT_W / 2


def forced(s, gap, e):
    """A hook or the rail's end driven more than FORCED into the post's wood or steel (only possible by force)."""
    if abs(e) >= POST_W / 2 or s + SLOT_BOT["upper"] + NECK_H > POST_TOP:
        return False  # beside or above the post
    into_face = gap < -FORCED
    into_slot_bottom = gap < HOOK_OUT / 2 and s < -FORCED
    into_slot_side = gap < HOOK_OUT - FORCED and abs(e) > SLOT_W / 2 - HOOK_T / 2 + FORCED
    return bool(into_face or into_slot_bottom or into_slot_side)


def _rail_gap(rail, tips):
    """The fingertip points' distance from the rail (its body and the hooks' box), metres."""
    local = rail.to_local(tips)
    lo = np.array([-RAIL_T / 2, -CROP, 0.0])
    hi = np.array([RAIL_T / 2, HOOK_OUT, RAIL_H])
    return np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1)


SUCCESS_HOLD_S = 0.0  # the Trial counts its own hold
HOLD_S = 3.0
SEAT_TOL = 0.001
GAP_TOL = 0.001
WEDGED_S = 0.0045
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
        self.log = None

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
        seated = both and all(read[h][0] <= SEAT_TOL and read[h][1] <= GAP_TOL for h in HOOKS)
        wedged = both and all(read[h][0] <= WEDGED_S for h in HOOKS)
        still = np.linalg.norm(rail.lin_vel) < 0.01 and np.linalg.norm(rail.ang_vel) < 0.1
        clear = bool(near.min() >= CLEAR)
        ok = seated and still and clear and not self.harm
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


GRIP = {"left": -0.13, "right": -0.25}  # where each hand takes the rail's top edge, from its end
GRIP_PHI = {"left": -90.0, "right": 90.0}  # each hand leans 45 degrees along the rail, towards the other
TILT = 45.0
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
    return tuple(float(v) for v in p), tilted(TILT, GRIP_PHI[arm], turn)


def _back(arm, p, dist):
    """A point `dist` back up the grip's approach axis from p."""
    phi = math.radians(GRIP_PHI[arm])
    a = np.array([math.cos(phi) * math.sin(math.radians(TILT)), math.sin(phi) * math.sin(math.radians(TILT)), -math.cos(math.radians(TILT))])
    return tuple(float(v) for v in np.asarray(p) - dist * a)


VIA = {"left": (0.21, 0.10, TOP + 0.24)}  # the left hand passes in front of the post, not over it
ARMS = ("left", "right")


class Servo:
    """The demo's feedback, from what the cameras see: the rail's pose. The soft arms sag several centimetres under the rail and
    lag when pushing it, and the rail twists in their grip, so while the hands carry or press it they move their commands
    together by the integrated error between where the rail's end should be and where it is seen (GAIN per control step, within
    LIMIT), shift sideways to bring the hooks, as seen, in line with the slots, and roll their grip to bring the two hooks into one
    vertical line. Only the demo uses this; it never reads the scorer's state."""

    GAIN = np.array([0.3, 0.08, 0.08])
    LIMIT_LO = np.array([-0.012, -0.010, -0.03])
    LIMIT_HI = np.array([0.012, 0.025, 0.14])
    START = np.array([0.0, 0.0, 0.045])  # a first guess of the sag, refined by the loop
    GAIN_ROLL, MAX_ROLL = 0.3, 0.14

    def __init__(self):
        self.live, self.profile, self.k = {}, [], 0
        self.reset()

    def reset(self, env=None):
        self.k, self.roll = 0, 0.0
        self.c = self.START.copy()
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
            err[0] = -(e_up + e_lo) / 2
            self.c = np.clip(self.c + self.GAIN * err, self.LIMIT_LO, self.LIMIT_HI)
            self.roll = float(np.clip(self.roll - self.GAIN_ROLL * (e_up - e_lo) / HOOK_PITCH, -self.MAX_ROLL, self.MAX_ROLL))
        self._apply()


SERVO = Servo()
m.on_step(SERVO.step)
m.on_reset(SERVO.reset)


DEMO_MODE = "full"  # red-team copies of this file set "nothing", "onearm", "careless" or "nodrive"


def demo():
    """Both hands take the rail's top edge, the left near its end. They lift the end to just above the slots' height and push its
    hooks against the post's plate, then lower it slowly while still pushing: the hooks pass into the slots as they come level with
    them, drop behind the plate onto the wedge fit, and the hands drive the rail on down its last 3 mm until it is seated. They
    ease off, let go and move away. While carrying and pressing, the hands correct for their sag and the rail's twist (Servo).
    Red-team modes: "nothing" (no plan), "onearm" (the left hand alone does it all), "careless" (lifts the end only to its seated
    height, pushes it at the post and lets go), "nodrive" (hangs the rail on its hooks but never drives it down)."""
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
    elif DEMO_MODE == "nodrive":
        steps = steps[:3]
        poses = [(n, (e + (0.0, 0.0, 0.003) if n == "unloaded" else e)) for n, e in poses]
    arms = {"full": ARMS, "careless": ARMS, "nodrive": ARMS, "onearm": ("left",), "nothing": ()}[DEMO_MODE]
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
            turned = tilted(TILT, 60.0)
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
