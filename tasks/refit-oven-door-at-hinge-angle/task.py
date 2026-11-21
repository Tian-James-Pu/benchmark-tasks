"""Put a lift-off oven door back on its range: hold the door at the hinges' stop angle and lower both of its bottom slots
over both hinge arms at once, then close it.

The range is a Hotpoint RB525DDWW 30-inch free-standing electric range (standard clean, "frameless lift-off oven door").
Its owner's manual: "To replace the door, make sure the hinges are in the special stop position. Position the slots in
the bottom of the door squarely over the hinges and slowly lower it over both hinges." The two hinge arms stand out of the
range's front frame at the oven's bottom corners, held at the special (broil) stop, about 7.5 degrees open. The door
was lifted off for cleaning and now stands on a folded towel in front of the range, its top leaning on the range's front.

Scale and mass (declared, README "Scale and mass"): the range, door and hinges are modelled uniformly at 85 % of real
size, so both arms reach the door's sides; the door's hinge line sits on the station table (standing in for the top of
the range's storage drawer). The real door weighs about 11 kg (estimate); the model weighs 2.5 kg, what the two YAM arms
can carry and steady.

Model (declared in code):
- "range" (an articulation welded to the world): the range's body (frame) and its two hinge arms (links), each a flat steel
  blade on a hinge about y at the door's bottom back edge. Joint zero is the special stop (7.5 degrees open); +7.5 degrees
  is shut (a joint limit: the door against the frame). The hinge's stop is the lower joint limit (the travel past it, to
  fully open, is not modelled). Its spring: a light detent holds the stop; pushed more than 3 degrees towards shut, the
  hinge's closing spring takes over and pulls the door shut (on_step, below).
- "door" (a free rigid body, 2.5 kg): the porcelain door with its window and visor handle, and two slots in its bottom
  edge (10 x 26 mm, 64 mm deep, built from the door's own box colliders round an empty channel). The door is held on the
  range only by its slots round the blades, by contact.
- Scoring reads poses only: each blade's tip against its slot's top, the hinges' angle, the fingertips' distance from the
  door, and the door's speed.

    python demokit.py task.py OUT --mode full
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
from yam_arena.urdf import axis_angle_matrix as axis_angle  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the oven door back on the range and close it."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- sizes: the real product (inches and mm in the comments) times SCALE ---------------------------------------------------
SCALE = 0.85
INCH = 0.0254
RANGE_W = 29.875 * INCH * SCALE  # overall width 29 7/8 in (spec sheet)
DOOR_W = 29.5 * INCH * SCALE  # the frameless door: just inside the range's width (estimated from the 29 7/8 in)
DOOR_H = 20.0 * INCH * SCALE  # 20 in: depth with the door open (46 3/8 in) minus the closed depth (26 3/8 in)
DOOR_T = 2.0 * INCH * SCALE  # about 2 in thick (estimated)
VISOR_OUT = 2.125 * INCH * SCALE  # the visor handle stands 2 1/8 in proud (28 1/2 in to its front, 26 3/8 in to the door's)
DOOR_MASS = 2.5  # kg (the real door: about 11 kg, estimated)
STOP_DEG = 7.5  # the special stop: the door's top about 2.6 in (66 mm) out from the frame at full size
HINGE_Y = (23.75 / 2 + 1.0) * INCH * SCALE  # the hinge slots: 1 in outside the oven's 23 3/4 in wide cavity
BLADE = (0.018, 0.0043, 0.064)  # the hinge arm's blade: along the door's thickness, across, standing out of the frame
SLOT = (0.021, 0.008, 0.064)  # the door's slot round it: along the thickness, across, deep
SLOT_X = -0.016  # the slot's (and blade's) centre, from the door's back face (the door's back face is x = 0)
GAP = 0.003  # the shut door's back face stands this far off the range's front frame
DRAWER_H = 0.035  # the door's bottom edge above the station table (the storage drawer's top, lowered)
RANGE_D = 0.30  # the range's body is modelled 30 cm deep (its front part)
COOKTOP_H = 30.0 * INCH * SCALE  # from the door's bottom edge to the cooktop (36 in cooktop, door bottom about 6 in up)
BACKGUARD_H = 10.125 * INCH * SCALE  # 46 1/8 in overall height less the 36 in cooktop

XF = 0.46  # the range's front frame, in front of the arms
ZB = TOP + DRAWER_H  # the hinge line: the shut door's bottom back edge
PIVOT_X = -GAP  # in the range's frame (origin on its front frame at the hinge line's height)

LOOKS = {
    "porcelain": m.Look(color=m.rgb(0.94, 0.94, 0.92), roughness=0.25),
    "enamel_dark": m.Look(color=m.rgb(0.05, 0.05, 0.06), roughness=0.3),
    "glass": m.Look(color=m.rgb(0.03, 0.03, 0.04), roughness=0.05, metallic=0.2),
    "cavity": m.Look(color=m.rgb(0.10, 0.10, 0.12), roughness=0.6),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.9, roughness=0.35),
    "chrome": m.Look(color=m.rgb(0.82, 0.82, 0.84), metallic=1.0, roughness=0.2),
    "knob": m.Look(color=m.rgb(0.90, 0.90, 0.88), roughness=0.35),
    "slot": m.Look(color=m.rgb(0.02, 0.02, 0.02), roughness=0.9),
}


def ry(deg: float) -> np.ndarray:
    """Rotation about +y: positive turns +z towards +x (the door's top towards the range: shutting)."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quat_y(deg: float) -> tuple:
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def turned_mesh(mesh: m.Mesh, deg: float, about) -> m.Mesh:
    about = np.asarray(about, float)
    return m.Mesh((mesh.v - about) @ ry(deg).T + about, mesh.f)


# -- the range: body (frame) and hinge arms (links). Frame origin: on the front frame, at the hinge line's height, mid-width.
RANGE_ORIGIN = np.array([XF, 0.0, ZB])


def _range_frame():
    parts, cols = [], []
    w, d = RANGE_W, RANGE_D
    z0, z1 = -DRAWER_H, COOKTOP_H  # from the station table to the cooktop
    parts.append(("body", m.box((d, w, z1 - z0), (d / 2, 0.0, (z0 + z1) / 2)), "porcelain"))
    cols.append(m.col_box("body_c", (d, w, z1 - z0), (d / 2, 0.0, (z0 + z1) / 2)))
    # the oven's cavity seen through the empty door opening, and the frame's dark gasket line round it
    cw, ch = 23.75 * INCH * SCALE, 19.0 * INCH * SCALE
    cz = 0.035 + ch / 2
    parts.append(("cavity", m.box((0.002, cw, ch), (-0.0008, 0.0, cz)), "cavity"))
    parts.append(("gasket", m.box((0.001, cw + 0.03, ch + 0.03), (-0.0003, 0.0, cz)), "enamel_dark"))
    for k, sy in enumerate((-1, 1)):  # the hinge slots in the front frame
        parts.append((f"hslot{k}", m.box((0.002, 0.012, 0.03), (-0.0009, sy * HINGE_Y, 0.012)), "slot"))
    # cooktop (upswept porcelain) with four coil elements, the backguard with its knobs
    parts.append(("cooktop", m.box((d + 0.01, w, 0.012), (d / 2 - 0.005, 0.0, z1 + 0.006)), "porcelain"))
    for k, (ex, ey, er) in enumerate(((0.07, -0.16, 0.085), (0.07, 0.16, 0.065), (0.22, -0.16, 0.065), (0.22, 0.16, 0.085))):
        parts.append((f"bowl{k}", m.cylinder(er * SCALE + 0.01, 0.004, (ex, ey, z1 + 0.014)), "chrome"))
        parts.append((f"coil{k}", m.cylinder(er * SCALE, 0.006, (ex, ey, z1 + 0.017)), "enamel_dark"))
    parts.append(("backguard", m.box((0.06, w, BACKGUARD_H), (d - 0.03, 0.0, z1 + 0.012 + BACKGUARD_H / 2)), "porcelain"))
    for k, ky in enumerate(np.linspace(-0.22, 0.22, 5)):
        parts.append((f"knob{k}", m.cylinder(0.018, 0.02, (d - 0.07, ky, z1 + 0.012 + BACKGUARD_H * 0.55), axis="x"), "knob"))
    # the storage drawer's front below the door (lowered: only its top lip shows above the table)
    parts.append(("drawer", m.box((0.004, w - 0.02, DRAWER_H - 0.004), (-0.002, 0.0, -DRAWER_H / 2)), "porcelain"))
    return parts, cols


def _blade_mesh(sy: float) -> m.Mesh:
    """The blade at the shut pose, in the range's frame: in the shut door's slot, from the hinge line up."""
    c = (PIVOT_X + SLOT_X, sy * HINGE_Y, BLADE[2] / 2)
    return m.box(BLADE, c)


def _hinge(name: str, sy: float) -> m.Link:
    pivot = (PIVOT_X, sy * HINGE_Y, 0.0)
    blade = turned_mesh(_blade_mesh(sy), -STOP_DEG, pivot)  # authored at the special stop
    # the hinge's body: a plate from the blade's foot back into the frame's slot (seen only)
    plate = turned_mesh(m.box((0.03, 0.004, 0.012), (PIVOT_X + 0.012, sy * HINGE_Y, 0.008)), -STOP_DEG, pivot)
    return m.hinge(name, "Y", pivot, 0.0, STOP_DEG, [("blade", blade, "steel"), ("plate", plate, "steel")],
                   [m.col_hull(f"{name}_blade_c", blade)], mass=0.3, spring=DETENT_K, damping=0.5, friction=0.02,
                   effort=50.0, armature=0.01, inertia=(4e-4, 4e-4, 1e-4))


DETENT_K = 20.0  # N m/rad: the stop's catch ("where you feel the hinge catch slightly")
CLOSE_K = 40.0  # N m/rad: the closing spring, towards shut, once the door is pushed past SNAP_DEG
SNAP_DEG = 3.0

_rp, _rc = _range_frame()
RANGE = m.mechanism("range", tuple(RANGE_ORIGIN), [_hinge("hinge_l", 1.0), _hinge("hinge_r", -1.0)], LOOKS,
                    frame_parts=_rp, frame_colliders=_rc, friction=(0.25, 0.2), solver_iters=48)


# -- the door. Its frame: origin at the middle of its bottom back edge; x towards the range (back face at x = 0), z up.
def _door():
    w, h, t = DOOR_W, DOOR_H, DOOR_T
    parts, cols = [], []
    sx0, sx1 = SLOT_X - SLOT[0] / 2, SLOT_X + SLOT[0] / 2
    parts.append(("upper", m.box((t, w, h - SLOT[2]), (-t / 2, 0.0, (h + SLOT[2]) / 2)), "porcelain"))
    cols.append(m.col_box("upper_c", (t, w, h - SLOT[2]), (-t / 2, 0.0, (h + SLOT[2]) / 2)))
    # the bottom band, round the two slots
    zc, zh = SLOT[2] / 2, SLOT[2]
    yi = HINGE_Y - SLOT[1] / 2  # the slots' inner sides
    yo = HINGE_Y + SLOT[1] / 2  # outer sides
    pieces = [("mid", (t, 2 * yi, zh), (-t / 2, 0.0, zc))]
    for k, sy in enumerate((1, -1)):
        pieces.append((f"out{k}", (t, w / 2 - yo, zh), (-t / 2, sy * (yo + w / 2) / 2, zc)))
        pieces.append((f"front{k}", (sx0 + t, SLOT[1], zh), ((-t + sx0) / 2, sy * HINGE_Y, zc)))
        pieces.append((f"back{k}", (-sx1, SLOT[1], zh), (sx1 / 2, sy * HINGE_Y, zc)))
    for name, size, c in pieces:
        parts.append((name, m.box(size, c), "porcelain"))
        cols.append(m.col_box(f"{name}_c", size, c))
    for k, sy in enumerate((1, -1)):  # the slots' mouths, seen from below
        parts.append((f"mouth{k}", m.box((SLOT[0], SLOT[1], 0.001), (SLOT_X, sy * HINGE_Y, -0.0004)), "slot"))
    # the window (dark glass) and the visor handle across the top
    parts.append(("window", m.box((0.002, w * 0.62, h * 0.34), (-t - 0.0006, 0.0, h * 0.50)), "glass"))
    vz = h - 0.03
    visor = m.box((VISOR_OUT, w * 0.86, 0.012), (-t - VISOR_OUT / 2, 0.0, vz))
    lip = m.box((0.010, w * 0.86, 0.030), (-t - VISOR_OUT + 0.005, 0.0, vz - 0.009))
    parts.append(("visor", visor, "porcelain"))
    parts.append(("visor_lip", lip, "porcelain"))
    cols.append(m.col_box("visor_c", (VISOR_OUT, w * 0.86, 0.012), (-t - VISOR_OUT / 2, 0.0, vz)))
    cols.append(m.col_box("visor_lip_c", (0.010, w * 0.86, 0.030), (-t - VISOR_OUT + 0.005, 0.0, vz - 0.009)))
    return parts, cols


TOWEL_T = 0.0175  # the folded microfiber towel's thickness
LEAN_DEG = 8.0  # the door leans its top on the range's front frame
DOOR_START = np.array([XF - DOOR_H * math.sin(math.radians(LEAN_DEG)) - 0.002, 0.0, TOP + TOWEL_T + 0.002])
_dp, _dc = _door()
DOOR = m.free("door", tuple(DOOR_START), _dp, _dc, LOOKS, mass=DOOR_MASS, friction=(0.9, 0.8), solver_iters=48,
              start_quat=quat_y(LEAN_DEG), com=(-DOOR_T / 2, 0.0, DOOR_H * 0.5))

TOWEL = SimReady("towel", pos=(XF - 0.115, 0.0, TOP), size=(0.209, 0.409, 0.017), mass=0.21, yaw=math.pi / 2,
                 package="consumable_microfiber_towel_folded_large_a01", usd="sm_consumable_microfiber_towel_folded_large_a01_01.usd")
# Distractors beside the range on the floor (the station table), which scoring ignores
SPRAY = SimReady("glass_cleaner", pos=(0.56, 0.45, TOP), size=(0.118, 0.040, 0.283), mass=0.5, yaw=-0.4,
                 package="cleaning_windowSpray_a01", usd="sm_cleaning_windowSpray_a01_01.usd")
SPONGE = SimReady("sponge", pos=(0.52, -0.43, TOP), size=(0.071, 0.101, 0.033), mass=0.03, yaw=0.3,
                  package="cleaning_sponge_b01", usd="sm_cleaning_sponge_b01_01.usd")
PAPER = SimReady("paper_towels", pos=(0.66, -0.45, TOP), size=(0.124, 0.124, 0.231), mass=0.15,
                 package="consumable_paperTowel_roll_a01", usd="sm_consumable_paperTowel_roll_a01_01.usd")

OBJECTS = [RANGE, DOOR, TOWEL, SPRAY, SPONGE, PAPER]


# -- the hinges' springs: a light catch at the stop; pushed 3 degrees towards shut, the closing spring takes over ------------
def _gain(env, link: str, k: float, target_deg: float) -> None:
    asset, j = m._joint(env, "range", link)
    m._TOUCHED.add(("range", link))
    asset.write_joint_stiffness_to_sim(m._tensor(asset, k), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, math.radians(target_deg)), joint_ids=[j])


class Hinges:
    def __init__(self):
        self.shutting = {"hinge_l": None, "hinge_r": None}

    def reset(self, env=None):
        self.shutting = {"hinge_l": None, "hinge_r": None}
        if env is not None:
            self.step(env)

    def step(self, env):
        st = m.state(env, "range")
        for link in self.shutting:
            want = st.deg(link) > SNAP_DEG
            if want != self.shutting[link]:
                _gain(env, link, CLOSE_K if want else DETENT_K, STOP_DEG if want else 0.0)
                self.shutting[link] = want


HINGES = Hinges()
m.on_step(HINGES.step)
m.on_reset(HINGES.reset)


# -- geometry for the scorer -------------------------------------------------------------------------------------------------
def blade_tip(range_state, link: str, sy: float):
    """A blade's tip (middle of its end) and its axis (root to tip), in the world, at the hinge's angle now."""
    q = math.degrees(range_state.joint(link)) - STOP_DEG  # from the shut pose
    r = ry(q)
    pivot = np.array([PIVOT_X, sy * HINGE_Y, 0.0])
    tip = pivot + r @ (np.array([PIVOT_X + SLOT_X, sy * HINGE_Y, BLADE[2]]) - pivot)
    return RANGE_ORIGIN + tip, r @ np.array([0.0, 0.0, 1.0])


def slot_top(door_state, sy: float):
    return door_state.to_world(np.array([SLOT_X, sy * HINGE_Y, SLOT[2]]))[0]


def seat(door_state, range_state, link: str, sy: float):
    """(gap, off): how far the slot's top stands past the blade's tip along the blade (0 = seated), and the blade tip's
    distance from the slot's axis."""
    tip, axis = blade_tip(range_state, link, sy)
    d = slot_top(door_state, sy) - tip
    gap = float(d @ axis)
    off = float(np.linalg.norm(d - gap * axis))
    return gap, off


SEATED_MM = 3.0
STARTED_MM = 15.0  # the blade at least this far into the slot
SHUT_TOL_DEG = 0.5
CLEAR = 0.03
STILL = 0.005
DROP_SPEED = 0.6
FALLEN_DEG = 60.0
VISOR_Z = DOOR_H - 0.03


def door_gap(door_state, tips) -> float:
    """The nearest fingertip point's distance (m) from the door's slab and visor."""
    q = door_state.to_local(tips)
    slab = np.maximum(np.abs(q - np.array([-DOOR_T / 2, 0.0, DOOR_H / 2])) - np.array([DOOR_T, DOOR_W, DOOR_H]) / 2, 0.0)
    vc = np.array([-DOOR_T - VISOR_OUT / 2, 0.0, VISOR_Z - 0.006])
    visor = np.maximum(np.abs(q - vc) - np.array([VISOR_OUT, DOOR_W * 0.86, 0.04]) / 2, 0.0)
    return float(min(np.linalg.norm(slab, axis=1).min(), np.linalg.norm(visor, axis=1).min()))


def on_visor(door_state, tips) -> bool:
    """Both fingers of one gripper on the visor handle (within 2 cm of it): a hold by the handle."""
    q = door_state.to_local(tips)
    vc = np.array([-DOOR_T - VISOR_OUT / 2, 0.0, VISOR_Z - 0.006])
    d = np.linalg.norm(np.maximum(np.abs(q - vc) - np.array([VISOR_OUT, DOOR_W * 0.86, 0.04]) / 2, 0.0), axis=1)
    for arm in range(2):
        f0, f1 = d[12 * arm:12 * arm + 6], d[12 * arm + 6:12 * arm + 12]
        if f0.min() < 0.02 and f1.min() < 0.02:
            return True
    return False


def now(env) -> dict:
    door, rng = m.state(env, "door"), m.state(env, "range")
    tips = m.fingertips(env)
    gl, ol = seat(door, rng, "hinge_l", 1.0)
    gr, orr = seat(door, rng, "hinge_r", -1.0)
    inside = ol < 0.006 and orr < 0.006
    started = inside and gl < BLADE[2] - STARTED_MM / 1000 and gr < BLADE[2] - STARTED_MM / 1000
    seated = inside and abs(gl) <= SEATED_MM / 1000 and abs(gr) <= SEATED_MM / 1000
    shut = all(rng.deg(k) >= STOP_DEG - SHUT_TOL_DEG for k in ("hinge_l", "hinge_r"))
    clear = door_gap(door, tips) >= CLEAR
    still = float(np.linalg.norm(door.lin_vel)) < STILL and float(np.linalg.norm(door.ang_vel)) < 0.05
    lifted = float(door.pos[2]) > TOP + TOWEL_T + 0.012
    return {"gaps": (gl, gr), "offs": (ol, orr), "started": started, "seated": seated, "shut": shut, "clear": clear,
            "still": still, "hinges": (rng.deg("hinge_l"), rng.deg("hinge_r")), "tilt": door.tilt_deg(),
            "vz": float(door.lin_vel[2]), "lifted": lifted, "visor": on_visor(door, tips), "gap_hand": door_gap(door, tips)}


class Trial:
    """The score, and the record of harm: the door dropped (falling faster than 0.6 m/s), fallen over (more than 60 degrees
    from upright) or lifted by its visor handle (both fingers of a gripper on it while the door is off the towel and not on
    the hinges). The record is cleared only here, at a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.harm = None
        self.last = {}

    def step(self, env):
        c = now(env)
        self.last = c
        if self.harm is None:
            if c["vz"] < -DROP_SPEED:
                self.harm = "dropped"
            elif c["tilt"] > FALLEN_DEG:
                self.harm = "fell over"
            elif c["visor"] and c["lifted"] and not c["started"]:
                self.harm = "lifted by the handle"
        ok = c["seated"] and c["shut"] and c["clear"] and c["still"] and self.harm is None
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS
        if self.harm is not None:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["clear"] and c["still"] and c["seated"]:
            self.score = 0.5
        elif c["clear"] and c["still"] and c["started"]:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration -----------------------------------------------------------------------------------------------------
GRASP_S = 0.06  # where the hands take the door's sides, up from its bottom edge
GRASP_IN = 0.020  # and in from its side edges (the pads' middles)
APPROACH_DEG = 50.0  # the fingers come in from above and outside, this far from the door's own downward direction (the gripper's body stays outside the door's edge)


def door_pose(door_state):
    return door_state.rotation(), np.asarray(door_state.pos, float)


def grasp_local(sy: float) -> np.ndarray:
    return np.array([-DOOR_T / 2, sy * (DOOR_W / 2 - GRASP_IN), GRASP_S])


def grip_rot(door_r: np.ndarray, sy: float) -> np.ndarray:
    """The TCP's rotation for a pinch across the door's thickness at its side (sy +1: the left side, the left arm)."""
    up, n = door_r[:, 2], door_r[:, 0]  # the door's up and its normal towards the range
    inward = -sy * door_r[:, 1]
    b = math.radians(APPROACH_DEG)
    a = -math.cos(b) * up + math.sin(b) * inward
    a /= np.linalg.norm(a)
    x = n - (n @ a) * a
    x /= np.linalg.norm(x)
    return np.c_[x, np.cross(a, x), a]


def seated_pose(hinge_deg: float = 0.0):
    """The door's pose seated on the blades with the hinges at `hinge_deg` from the stop (positive: towards shut)."""
    r = ry(-STOP_DEG + hinge_deg)
    pivot = RANGE_ORIGIN + np.array([PIVOT_X, 0.0, 0.0])
    return r, pivot  # the door's origin is on the pivot line


def _report(ctx, env, label=""):
    c = now(env)
    d = m.state(env, "door")
    print(f"[oven] t {ctx.t:5.1f} {label}door at {np.round(d.pos, 4)} tilt {c['tilt']:5.2f} slot gaps {c['gaps'][0] * 1000:6.1f} {c['gaps'][1] * 1000:6.1f} mm, "
          f"offs {c['offs'][0] * 1000:5.1f} {c['offs'][1] * 1000:5.1f} mm, hinges {c['hinges'][0]:5.2f} {c['hinges'][1]:5.2f} deg, started {c['started']} seated {c['seated']} "
          f"shut {c['shut']} clear {c['clear']} ({c['gap_hand'] * 1000:.0f} mm) harm {TRIAL.harm} score {TRIAL.score:.2f} full {TRIAL.full}", flush=True)


def demo_script(ctx):
    """Both arms take the door's sides 13 cm up from its bottom edge, pinching across its thickness from above and outside.
    They lift it off the towel, turn it from leaning on the range to the hinges' stop angle and carry it over the blades
    (closed loop on the door's seen pose), lower it straight down the blades until it sits on them, let go and back off.
    Then both push the door's face near its sides until the hinges' closing spring takes it shut, and move away.
    Failure checks (--mode): none does nothing; onearm lifts with the left arm alone; careless lowers the door at its
    leaning angle instead of the stop angle; hold stays on the door after it is seated and shut; noclose hangs the door
    and leaves it open."""
    import demokit as d
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "careless", "hold", "noclose"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(80):
            yield
        _report(ctx, env, "end: ")
        return

    arms = [(l, 1.0)] if mode == "onearm" else [(l, 1.0), (r, -1.0)]

    def tcp_for(door_r, door_p, sy):
        rot = grip_rot(door_r, sy)
        return door_p + door_r @ grasp_local(sy), d._as_grip(rot)

    # -- reach in round the door's sides and pinch. The open fingers' plates are 68 mm wide, so each hand comes down in
    #    front of the door and outside its edge, moves back beside the edge, then in over the door's side margin.
    dr, dp = door_pose(ctx.state("door"))
    stage = {}
    for arm, sy in arms:
        g, grip = tcp_for(dr, dp, sy)
        beside = np.array([g[0], sy * (DOOR_W / 2 + 0.041), g[2] - 0.03])
        stage[arm.side] = (g, grip, beside, beside - np.array([0.09, 0.0, 0.0]))
    yield from ctx.together(*[arm.move(stage[arm.side][3], grip=stage[arm.side][1], finger=FINGER_OPEN, t=3.0, label="in front, outside the door's edge") for arm, _ in arms])
    yield from ctx.together(*[arm.move(stage[arm.side][2], t=1.5, label="beside the door's edge") for arm, _ in arms])
    yield from ctx.together(*[arm.servo(stage[arm.side][2], timeout=1.0, label="beside the door's edge") for arm, _ in arms])
    _report(ctx, env, "beside: ")
    yield from ctx.together(*[arm.move(stage[arm.side][0], t=1.5, label="fingers round the door's side") for arm, _ in arms])
    yield from ctx.together(*[arm.servo(stage[arm.side][0], timeout=1.0, label="at the side") for arm, _ in arms])
    _report(ctx, env, "round the side: ")
    yield from ctx.together(*[arm.close(DOOR_T, 40.0, t=0.6, label="pinch the door") for arm, _ in arms])
    _report(ctx, env, "pinched: ")

    # the hand's pose in the door's frame, as gripped (absorbs the grasp's small errors)
    held = {}
    dr, dp = door_pose(ctx.state("door"))
    for arm, sy in arms:
        p, rot = ik.tcp_pose(arm.q, arm.base)
        held[arm.side] = (dr.T @ (p - dp), dr.T @ rot)
    loop = {"corr": {arm.side: np.zeros(3) for arm, _ in arms}, "tilt": 0.0}
    mouth = {arm.side: np.array([SLOT_X, sy * HINGE_Y, 0.0]) for arm, sy in arms}  # each side's slot mouth, door frame

    def carry(path, seconds, gain=0.25, settle=0.0, tilt_gain=0.2, label="carry"):
        """Move the door along `path(u)` (u 0..1) -> (door rotation, door origin), closed loop on the door's seen pose.
        Each hand has its own correction (an integral, kept from one stage to the next) that brings its side's slot mouth to
        where the path puts it: it takes up each arm's sag under the door and squares the door. A shared angle loop holds
        the door's tilt."""
        n = max(1, round(seconds * 20))
        for k in range(n + round(settle * 20)):
            u = min(1.0, (k + 1) / n)
            u = u * u * (3 - 2 * u)
            want_r, want_p = path(u)
            seen_r, seen_p = door_pose(ctx.state("door"))
            # the door's pitch error about y (positive: the door's top should go further towards +x)
            e_t = math.degrees(math.atan2(want_r[0, 2], want_r[2, 2]) - math.atan2(seen_r[0, 2], seen_r[2, 2]))
            loop["tilt"] = float(np.clip(loop["tilt"] + tilt_gain * e_t, -10.0, 10.0))
            cmd_r = ry(loop["tilt"]) @ want_r
            for arm, sy in arms:
                e = (want_p + want_r @ mouth[arm.side]) - (seen_p + seen_r @ mouth[arm.side])
                corr = loop["corr"][arm.side] + np.clip(gain * e, -0.004, 0.004)
                nrm = float(np.linalg.norm(corr))
                loop["corr"][arm.side] = corr if nrm < 0.05 else corr * 0.05 / nrm
                lp, lr = held[arm.side]
                mm, dg = arm._solve(want_p + loop["corr"][arm.side] + cmd_r @ lp, cmd_r @ lr)
                arm._note(mm, dg, label)
            yield

    def slot_errors():
        st, rs = ctx.state("door"), ctx.state("range")
        out = []
        for arm, sy in arms:
            link = "hinge_l" if sy > 0 else "hinge_r"
            tip, axis = blade_tip(rs, link, sy)
            d_ = st.to_world(mouth[arm.side])[0] - tip
            out.append(float(np.linalg.norm(d_ - (d_ @ axis) * axis)))
        return out

    lean_r, lean_p = dr, dp
    stop_r, seat_p = seated_pose(0.0)
    blade_axis = stop_r[:, 2]
    tilt_goal = -STOP_DEG + (LEAN_DEG + 1.0 if mode == "careless" else 0.0)  # careless: keeps the door near its lean
    goal_r = ry(tilt_goal)
    over_p = seat_p + 0.078 * blade_axis  # the slots' mouths 14 mm over the blades' tips

    def lerp_r(a, b, u):
        ta = math.degrees(math.atan2(a[0, 2], a[2, 2]))
        tb = math.degrees(math.atan2(b[0, 2], b[2, 2]))
        return ry(ta + (tb - ta) * u)

    upright = ry(0.0) if mode != "careless" else lean_r
    yield from carry(lambda u: (lerp_r(lean_r, upright, u), lean_p), 2.0, label="stand it upright on its bottom edge")
    lift_p = lean_p + np.array([0.0, 0.0, 0.03])
    yield from carry(lambda u: (upright, lean_p + u * (lift_p - lean_p)), 1.5, settle=0.5, label="lift off the towel")
    _report(ctx, env, "lifted: ")
    high_p = np.array([lift_p[0], 0.0, over_p[2] + 0.012])
    yield from carry(lambda u: (lerp_r(upright, goal_r, u), lift_p + u * (high_p - lift_p)), 3.0, label="up, and turn to the stop angle")
    yield from carry(lambda u: (goal_r, high_p + u * (over_p - high_p)), 2.5, settle=1.0, label="over the blades")
    yield from carry(lambda u: (goal_r, over_p), 0.1, settle=2.0, gain=0.3, label="square over the blades")
    _report(ctx, env, "over the blades: ")
    print(f"[oven] slot mouths off the blades' lines: {np.round(np.array(slot_errors()) * 1000, 1)} mm", flush=True)
    down_to = seat_p - 0.002 * blade_axis
    # down the blades: the door's angle is now set by the blades in its slots, so the angle loop holds what it has
    yield from carry(lambda u: (goal_r, over_p + u * (down_to - over_p)), 3.0, settle=1.0, gain=0.15, tilt_gain=0.0, label="down the blades")
    _report(ctx, env, "lowered: ")

    # let go and back off along the fingers' axis, then up and out
    yield from ctx.together(*[arm.open(t=0.5, label="let go") for arm, _ in arms])
    yield from ctx.together(*[arm.move(arm.at() - 0.07 * arm.approach_axis(), t=1.0, label="back off") for arm, _ in arms])
    yield from ctx.together(*[arm.move(arm.at() - np.array([0.10, 0.0, 0.0]), t=1.0, label="back towards the robot") for arm, _ in arms])
    yield from ctx.together(*[arm.wait(0.5) for arm, _ in arms])
    _report(ctx, env, "let go: ")
    if mode in ("noclose", "onearm", "careless"):
        for k in range(60):
            yield
        _report(ctx, env, "end: ")
        return

    # push the door shut: closed fingers on its face near each side, 22 cm up, pointing at the range
    push_s = 0.22
    gens = []
    for arm, sy in arms:
        p_local = np.array([-DOOR_T - 0.012, sy * (DOOR_W / 2 - 0.06), push_s])
        st = ctx.state("door")
        dr, dp = door_pose(st)
        p = dp + dr @ p_local - np.array([0.04, 0.0, 0.0])
        gens.append(arm.move(p, grip=d.level(0.0), finger=0.0, t=2.0, label="in front of the door"))
    yield from ctx.together(*gens)
    pivot = RANGE_ORIGIN + np.array([PIVOT_X, 0.0, 0.0])
    gens = []
    for arm, sy in arms:
        p_local = np.array([-DOOR_T - 0.012, sy * (DOOR_W / 2 - 0.06), push_s])
        st = ctx.state("door")
        dr, dp = door_pose(st)
        p = dp + dr @ p_local
        gens.append(arm.move(p, t=1.0, label="fingers on the door"))
    yield from ctx.together(*gens)
    # push: swing both hands' commands about the hinge line, a little each step, until the door is seen shut
    def pitch():
        r_ = ctx.state("door").rotation()
        return math.degrees(math.atan2(r_[0, 2], r_[2, 2]))

    step_r = axis_angle(np.array([0.0, 1.0, 0.0]), math.radians(0.25))
    shut_for = 0
    for k in range(160):
        shut_for = shut_for + 1 if pitch() > -0.4 else 0
        if shut_for >= 10:
            break
        for arm, _ in arms:
            c, rot = ik.tcp_pose(arm.q, arm.base)
            if pitch() < 1.0:
                c, rot = pivot + step_r @ (c - pivot), step_r @ rot
            mm, dg = arm._solve(c, rot)
            arm._note(mm, dg, "push it shut")
        yield
    yield from ctx.together(*[arm.wait(0.6) for arm, _ in arms])
    _report(ctx, env, "pushed: ")
    if mode == "hold":
        for k in range(80):
            yield
        _report(ctx, env, "end (still pushing): ")
        return
    yield from ctx.together(*[arm.move(arm.at() - np.array([0.08, 0.0, 0.0]), t=1.0, label="hands off the door") for arm, _ in arms])
    yield from ctx.together(l.move(np.array([0.18, 0.30, TOP + 0.22]), grip=d.down(0), t=1.5, label="left clear"),
                            r.move(np.array([0.18, -0.30, TOP + 0.22]), grip=d.down(0), t=1.5, label="right clear"))
    for k in range(70):
        yield
        if k % 35 == 34:
            _report(ctx, env, "end: ")
