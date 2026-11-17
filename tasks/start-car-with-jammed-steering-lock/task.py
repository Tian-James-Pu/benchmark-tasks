"""Turn a parked car's key to ON when the steering lock has jammed it: turn the wheel off the lock bolt, hold it there against the tyres'
wind-up, and turn the key to ON while it is held.

The rig is the driver's end of a car's steering column: a steering wheel on a raked column (25 degrees from horizontal), its column lock,
and a keyed ignition on the column's right side, behind the wheel. The car was parked and the wheel moved after the key came out, so the
lock bolt (pin) has dropped into a notch of the lock collar on the steering shaft and the wheel is pressed clockwise against the notch's
wall by the wind-up stored in the front tyres (a torsion spring here). The wall pinches the pin, so the key's cam cannot draw it back.

Model (declared in code):
- Column ("column"): its frame is the shroud and the ignition housing; its one link is the steering wheel (SimReady
  steeringSystem_steering_wheel_e01, shown as authored, colliding through convex hulls taken from its own mesh) with the steering shaft and
  the lock collar. Wheel joint: about the column axis, positive clockwise as the driver sees it, limits +-25 degrees (travel past the lock
  play is not modelled). The tyres' wind-up is the joint's spring: K_WHEEL (N m/degree) towards REST_DEG past the notch's wall.
- Lock ("lock", a second articulation so that its parts collide with the collar): the pin, a slider into the collar's notch, and the key, a
  hinge in the ignition. The pin is pinched by real contact with the notch's wall (steel on steel friction) plus the friction of its guide,
  which PhysX scales with the load through the joint.
- The key's cam (code, each control step): the pin is driven to the cam's position for the key's angle (drawn back 8 mm from 2.5 to 25
  degrees), never pulled with more than the key's forcing torque could pull it through the cam (120 N). While the pin is short of the
  cam's position, the key is held at the angle the pin's actual position allows, with the stiffness that makes the two one rigid cam. So
  the key turns only as far as the pin moves. The torque the cam takes from the key is recorded; more than 2.45 N m for 0.15 s is forcing
  the key (UNECE R116's locked-cylinder torque), and scores 0.
- Key detents: LOCK 0, ACC 30, ON 65 degrees; START 80 to 92 springs back to ON.
- The load: TAU_PRELOAD (8 N m) and K_WHEEL are estimates until measured on a car (README, "The load").

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady, fetch  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Turn the key to ON."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]

# -- the column's frame: origin at the wheel's centre (the rim's mid-plane, on the column's axis); X along the column away from the driver,
#    Y to the driver's left, Z up the wheel's face (its 12 o'clock). The column rakes RAKE degrees: X points down towards the dash.
RAKE = 25.0
CENTRE = np.array([0.25, 0.0, TOP + 0.22])
QUAT = (0.0, math.sin(math.radians(RAKE) / 2), 0.0, math.cos(math.radians(RAKE) / 2))  # about world y: +X dips towards the dash
_ca, _sa = math.cos(math.radians(RAKE)), math.sin(math.radians(RAKE))
ROT = np.array([[_ca, 0.0, _sa], [0.0, 1.0, 0.0], [-_sa, 0.0, _ca]])  # column frame -> world


def world(p_local) -> np.ndarray:
    return CENTRE + ROT @ np.asarray(p_local, float)


def local(p_world) -> np.ndarray:
    return ROT.T @ (np.asarray(p_world, float) - CENTRE)


AXIS_TO_DRIVER = ROT @ np.array([-1.0, 0.0, 0.0])  # anticlockwise as the driver sees it is positive about this

# -- the tyres' wind-up and the wheel
K_WHEEL = 0.5  # N m per degree at the wheel: the front tyres' twist and the column's give (an estimate)
TAU_PRELOAD = 8.0  # N m the wheel presses the lock's wall with at the start (provisional until measured on a car: see README)
REST_DEG = TAU_PRELOAD / K_WHEEL  # the spring's rest, clockwise past the notch's wall
WHEEL_LIMIT = 25.0
WHEEL_PKG, WHEEL_USD = "steeringSystem_steering_wheel_e01", "sm_steeringSystem_steering_wheel_e01_01.usd"
ASSET_X = 0.127  # the wheel file's origin (its mounting face) along the column: its rim's mid-plane is at X 0
RIM_A, RIM_B, RIM_HALF_R, RIM_HALF_X = 0.155, 0.150, 0.018, 0.021  # the rim's centre line (half widths along Y and Z) and its section's half sizes

# -- the lock collar on the steering shaft, and the pin under it
X_COL = 0.25
R_OUT, R_ROOT = 0.022, 0.016  # the collar's outside, and its notch's floor
PIN_W, PIN_D, PIN_L = 0.008, 0.010, 0.025  # the pin's width (Y), depth (X), length (Z)
GAP = 0.0002
PLAY_DEG = 8.0  # the notch's free play past the pin's width, at mid-depth
Y_WALL_CW = -(PIN_W / 2 + GAP)  # the wall that meets the pin when the wheel turns clockwise
Y_WALL_ACW = PIN_W / 2 + GAP + 0.019 * math.radians(PLAY_DEG)
PIN_TIP = -(R_ROOT + 0.0003)
PIN_TRAVEL = 0.008
K_PIN = 1.0e5  # N/m: the cam's stiffness at the pin
PIN_FRICTION = 0.2

# -- the ignition: the key's axis parallel to the column's, 12 cm right of it and 4.5 cm below
KEY_Y, KEY_Z = -0.12, -0.045
X_FACE = 0.175  # the ignition's face
BOW = (0.026, 0.011, 0.032)  # the key's head (plastic over the transponder): along the axis, thick, across
BOW_X = X_FACE - 0.005 - BOW[0] / 2
LOCK_DEG, ACC_DEG, ON_DEG, START_DEG, KEY_MAX = 0.0, 30.0, 65.0, 80.0, 92.0
PHI0, PHI1 = 2.5, 25.0  # the cam's dead band, and where the pin is fully drawn
CAM = PIN_TRAVEL / math.radians(PHI1 - PHI0)  # m of pin per radian of key
K_KEY = K_PIN * CAM ** 2  # N m/rad: the key's stiffness through the cam
K_DETENT, K_START = 0.5, 1.5  # N m/rad
FORCE_LIMIT = 2.45  # N m through the key: forcing it
PIN_LAG = 0.0003  # m: a pin this far short of where the cam sent it is held back
PULL_MAX = FORCE_LIMIT / CAM  # N: the cam's pull on the pin at the forcing torque (about 120 N)

LOOKS = {
    "abs": m.Look(color=m.rgb(0.07, 0.07, 0.08), roughness=0.55),
    "abs_dark": m.Look(color=m.rgb(0.035, 0.035, 0.04), roughness=0.5),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.9, roughness=0.35),
    "chrome": m.Look(color=m.rgb(0.80, 0.80, 0.82), metallic=1.0, roughness=0.2),
    "nickel": m.Look(color=m.rgb(0.78, 0.77, 0.72), metallic=0.95, roughness=0.3),
    "key_head": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.45),
    "mark_white": m.Look(color=m.rgb(0.92, 0.92, 0.90), roughness=0.4),
    "mark_amber": m.Look(color=m.rgb(0.95, 0.62, 0.10), roughness=0.4),
    "mark_green": m.Look(color=m.rgb(0.20, 0.75, 0.30), roughness=0.4),
    "mark_red": m.Look(color=m.rgb(0.85, 0.15, 0.12), roughness=0.4),
    "dash": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.8),
}


def _x_cyl(r, x0, x1, y=0.0, z=0.0, segments=32):
    return m.cylinder(r, x1 - x0, ((x0 + x1) / 2, y, z), axis="x", segments=segments)


# -- the column ------------------------------------------------------------------------------------------------------------------------------


def _column_frame():
    parts, cols = [], []
    sx0, sx1, sy, sz, t = 0.135, 0.33, 0.05, 0.06, 0.005
    parts.append(("shroud", m.box((sx1 - sx0, 2 * sy, 2 * sz), ((sx0 + sx1) / 2, 0.0, 0.0)), "abs"))
    for name, size, c in (("shroud_top", (sx1 - sx0, 2 * sy, t), (0, 0, sz - t / 2)), ("shroud_bottom", (sx1 - sx0, 2 * sy, t), (0, 0, -sz + t / 2)),
                          ("shroud_left", (sx1 - sx0, t, 2 * sz), (0, sy - t / 2, 0)), ("shroud_right", (sx1 - sx0, t, 2 * sz), (0, -sy + t / 2, 0))):
        cols.append(m.col_box(name + "_c", size, ((sx0 + sx1) / 2, c[1], c[2])))
    # the ignition housing on the shroud's right, its face towards the driver, with its bezel and the position marks
    parts.append(("ign_boss", _x_cyl(0.026, X_FACE, X_FACE + 0.05, KEY_Y, KEY_Z), "abs"))
    by0, by1 = -0.047, KEY_Y + 0.015
    bracket = ((0.04, by0 - by1, 0.03), (X_FACE + 0.025, (by0 + by1) / 2, KEY_Z + 0.005))
    parts.append(("ign_bracket", m.box(*bracket), "abs"))
    ring = [(X_FACE - 0.0008, KEY_Y + 0.0235 * math.cos(a), KEY_Z + 0.0235 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 49)]
    parts.append(("ign_bezel", m.tube(ring, 0.0016, sides=8), "chrome"))
    parts.append(("ign_face", _x_cyl(0.021, X_FACE - 0.0008, X_FACE, KEY_Y, KEY_Z), "abs_dark"))
    parts.append(("ign_slot", m.box((0.0012, 0.0025, 0.012), (X_FACE - 0.0012, KEY_Y, KEY_Z)), "steel"))
    for deg, look in ((LOCK_DEG, "mark_white"), (ACC_DEG, "mark_amber"), (ON_DEG, "mark_green"), (88.0, "mark_red")):
        a = math.radians(deg)  # clockwise as the driver sees it, from 12 o'clock
        parts.append((f"mark_{int(deg)}", _x_cyl(0.0022, X_FACE - 0.0015, X_FACE - 0.0002, KEY_Y - 0.0185 * math.sin(a), KEY_Z + 0.0185 * math.cos(a), 12), look))
    cols.append(m.col_hull("ign_boss_c", _x_cyl(0.026, X_FACE, X_FACE + 0.05, KEY_Y, KEY_Z, 16)))
    cols.append(m.col_box("ign_bracket_c", *bracket))
    return parts, cols


def _tooth_outline(yw: float, sgn: float) -> np.ndarray:
    """One tooth of the collar beside the notch, in the (Y, Z) plane: its wall (the plane Y = yw) from inside the core out to the collar's
    outside, then 60 degrees of the outside away from the notch (sgn -1: towards -Y; +1: towards +Y), and back inside the core.
    Counter-clockwise in (Y, Z)."""
    zc = -math.sqrt(R_OUT ** 2 - yw ** 2)
    a_wall = math.atan2(zc, yw)
    pts = [(yw, -0.011), (yw, zc)]
    for k in range(1, 13):
        a = a_wall + sgn * math.radians(5.0 * k)
        pts.append((R_OUT * math.cos(a), R_OUT * math.sin(a)))
    a_end = a_wall + sgn * math.radians(60.0)
    pts.append((0.011 * math.cos(a_end), 0.011 * math.sin(a_end)))
    pts = np.array(pts)
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    return pts if area > 0 else pts[::-1]


def _tooth_mesh(yw: float, sgn: float) -> m.Mesh:
    x0, x1 = X_COL - 0.009, X_COL + 0.009
    ring = m.prism(_tooth_outline(yw, sgn), x0, x1)
    return m.Mesh(ring.v[:, [2, 0, 1]], ring.f)  # the block's (x, y, z) to the column's (Y, Z, X): a turn, so faces stay outward


TEETH = (("tooth_cw", Y_WALL_CW, -1.0), ("tooth_acw", Y_WALL_ACW, 1.0))


def _collar_hulls():
    """The lock collar on the shaft: a core to the notch's floor, and teeth to its outside either side of the notch (one notch: the wheel's
    travel here never brings another to the pin). The notch's walls are planes parallel to the pin's sides at the start."""
    cols = [m.col_hull("collar_core", _x_cyl(R_ROOT, X_COL - 0.009, X_COL + 0.009, segments=24))]
    return cols + [m.col_hull(name, _tooth_mesh(yw, sgn)) for name, yw, sgn in TEETH]


def _collar_visual():
    parts = [("shaft", _x_cyl(0.009, 0.12, 0.30), "steel"), ("collar_core", _x_cyl(R_ROOT, X_COL - 0.009, X_COL + 0.009), "steel")]
    return parts + [(name, _tooth_mesh(yw, sgn), "steel") for name, yw, sgn in TEETH]


_HULLS = json.loads((HERE / "wheel_colliders.json").read_text())


def _wheel_link():
    cols = [m.col_hull(f"w_{name}", m.Mesh(np.array(h["v"]), np.array(h["f"]))) for name, h in _HULLS.items()] + _collar_hulls()
    parts = _collar_visual()
    return m.hinge("wheel", "X", (0.0, 0.0, 0.0), -WHEEL_LIMIT, WHEEL_LIMIT, parts, cols, mass=1.0, spring=math.degrees(K_WHEEL), damping=0.4,
                   friction=0.02, effort=80.0, armature=0.02, inertia=(0.026, 0.014, 0.014))


_cfp, _cfc = _column_frame()
_COLUMN = m.mechanism("column", tuple(CENTRE), [_wheel_link()], LOOKS, frame_parts=_cfp, frame_colliders=_cfc, start_quat=QUAT, friction=(1.0, 0.9))

_ASSET_ROT = (0.0, 1 / math.sqrt(2), 0.0, -1 / math.sqrt(2))  # wxyz: the file's +z (its face) to -X, its -x (the open top) to +Z
_BUTTONS = {  # the file's own placement of each button (translate, orient wxyz), from its base layer
    "button_01": ((-0.0009562388877, 0.08580813557, 0.1415439993), (0.9908408979, 0.03460092255, -0.1304466957, 0.004555299044)),
    "button_02": ((0.01268479414, 0.08570830524, 0.1426340044), (0.9980211968, 0.03485166814, 0.05230407045, -0.001826498009)),
    "button_03": ((0.006225758698, 0.09873288125, 0.1435670853), (0.9984396281, 0.03488752781, -0.04359282942, 0.000913561153)),
    "button_04": ((0.01283229701, 0.1116956398, 0.1443449855), (0.9976790607, 0.04355960853, 0.052286144, -0.002282862667)),
    "button_05": ((0.0004714782117, 0.1117062941, 0.1437709928), (0.9939160595, 0.03470831358, -0.1044647874, 0.003647990494)),
    "button_06": ((-0.0009379402618, -0.08565954864, 0.1414200217), (0.9908408979, -0.03460092255, -0.1304466957, -0.004555299044)),
    "button_07": ((0.01270310115, -0.08555971086, 0.1424679905), (0.9980211968, -0.03485166814, 0.05230407045, 0.001826498009)),
    "button_08": ((0.006427020766, -0.1041744947, 0.1436942071), (0.9980971617, -0.0435991473, -0.0435832056, -0.001293941984)),
}


def _wheel_asset_usda() -> str:
    """The SimReady wheel's look, as authored, inside the wheel link: its meshes and materials only (its own bodies, joints and colliders
    are left out; the link's hulls above collide)."""
    inst = fetch(WHEEL_PKG, WHEEL_USD).parent / "payloads" / "instances.usda"

    def one(name, mesh, xform=""):
        return f"""
        def Xform "{name}" (
            prepend references = @{inst}@</Instances/{mesh}>
        )
        {{{xform}
            over "{mesh}" (
                apiSchemas = ["MaterialBindingAPI"]
            )
            {{
                bool physics:collisionEnabled = 0
            }}
        }}"""

    body = one("body", "body_01_mesh_00")
    for name, (t, q) in _BUTTONS.items():
        body += one(name, f"{name}_mesh_00", f"""
            double3 xformOp:translate = ({t[0]}, {t[1]}, {t[2]})
            quatd xformOp:orient = ({q[0]}, {q[1]}, {q[2]}, {q[3]})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]""")
    w, x, y, z = _ASSET_ROT
    return f"""
    def Xform "wheel_asset"
    {{
        double3 xformOp:translate = ({ASSET_X}, 0, 0)
        quatd xformOp:orient = ({w}, {x}, {y}, {z})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]{body}
    }}
"""


def _column_with_wheel_asset():
    import dataclasses
    plain = _COLUMN.build

    def build() -> str:
        text = plain()
        head = text.index('def Xform "wheel" (')
        at = text.index('def Xform "visual"', head)
        at = text.rindex("\n", 0, at) + 1
        return text[:at] + _wheel_asset_usda() + text[at:]

    return dataclasses.replace(_COLUMN, build=build)


COLUMN = _column_with_wheel_asset()

# -- the lock: the pin and the key ------------------------------------------------------------------------------------------------------------


def _pin_link():
    c = (X_COL, 0.0, PIN_TIP - PIN_L / 2)
    parts = [("pin", m.box((PIN_D, PIN_W, PIN_L), c), "steel")]
    cols = [m.col_box("pin_c", (PIN_D, PIN_W, PIN_L), c)]
    return m.slider("pin", "Z", -PIN_TRAVEL, 0.0, parts, cols, at=(X_COL, 0.0, PIN_TIP), mass=0.08, spring=K_PIN, damping=100.0, friction=PIN_FRICTION,
                    effort=3000.0, armature=0.02)


def _key_link():
    bow_c = (BOW_X, KEY_Y, KEY_Z)
    parts = [("head", m.box(BOW, bow_c), "key_head"),
             ("shank", m.box((0.006, 0.0022, 0.009), (X_FACE - 0.003, KEY_Y, KEY_Z)), "nickel"),
             ("blade", m.box((0.026, 0.0022, 0.008), (X_FACE + 0.013, KEY_Y, KEY_Z)), "nickel"),
             ("ring", m.box((0.004, 0.0094, 0.010), (BOW_X - BOW[0] / 2 + 0.004, KEY_Y, KEY_Z)), "abs_dark")]
    cols = [m.col_box("head_c", BOW, bow_c)]
    return m.hinge("key", "X", (X_FACE, KEY_Y, KEY_Z), LOCK_DEG, KEY_MAX, parts, cols, mass=0.03, spring=K_DETENT, damping=0.08, friction=0.0, effort=40.0,
                   armature=0.0005, inertia=(2.0e-6, 3.0e-6, 3.0e-6))


LOCK = m.mechanism("lock", tuple(CENTRE), [_pin_link(), _key_link()], LOOKS, start_quat=QUAT, friction=(0.3, 0.2))

# -- the dash behind the column (custom structural boards) -----------------------------------------------------------------------------------
DASH_X0, DASH_X1, DASH_H = 0.50, 0.72, 0.30


def _dash():
    parts, cols = [], []
    for name, size, c in (("dash_face", (0.02, 0.50, DASH_H), (DASH_X0 + 0.01, 0.0, DASH_H / 2)),
                          ("dash_ledge", (DASH_X1 - DASH_X0, 0.50, 0.02), ((DASH_X0 + DASH_X1) / 2, 0.0, DASH_H - 0.01))):
        parts.append((name, m.box(size, c), "dash"))
        cols.append(m.col_box(name + "_c", size, c))
    return parts, cols


_dp, _dc = _dash()
DASH = m.fixed("dash", (0.0, 0.0, TOP), _dp, _dc, LOOKS, friction=(0.9, 0.8))

# -- distractors (SimReady): sunglasses on the dash ledge, a coffee cup and the car's service book beside the dash
SUNGLASSES = SimReady("sunglasses", pos=(0.61, 0.14, TOP + DASH_H), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                      usd="sm_accessory_eyeglasses_a01_01.usd", yaw=math.pi / 2)
CUP = SimReady("coffee_cup", pos=(0.58, 0.36, TOP), size=(0.085, 0.085, 0.117), mass=0.25, package="food_beverage_coffeeCup_a01",
               usd="sm_food_beverage_coffeeCup_a01_01.usd")
NOTEBOOK = SimReady("service_book", pos=(0.33, 0.39, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01",
                    usd="sm_misc_notebook_a01_01.usd", yaw=0.3)

OBJECTS = [COLUMN, LOCK, DASH, SUNGLASSES, CUP, NOTEBOOK]


# -- the lock's working (cam, detents) and the tyres' wind-up ------------------------------------------------------------------------------


def cam_draw(phi: float) -> float:
    """How far the cam has drawn the pin back (m) at key angle `phi` (rad)."""
    return PIN_TRAVEL * min(1.0, max(0.0, (phi - math.radians(PHI0)) / math.radians(PHI1 - PHI0)))


def detent(phi_deg: float) -> tuple[float, float]:
    """The key's own spring: (rest angle in degrees, stiffness N m/rad)."""
    if phi_deg > START_DEG:
        return ON_DEG, K_START
    return min((LOCK_DEG, ACC_DEG, ON_DEG), key=lambda d: abs(d - phi_deg)), K_DETENT


def _joint_vel(env, part, link) -> float:
    asset = env.unwrapped.scene[part]
    return float(m._np(asset.data.joint_vel)[0, asset.find_joints(f"{link}_joint")[0][0]])


def _drive(env, part, link, target, stiffness):
    asset, j = m._joint(env, part, link)
    m._TOUCHED.add((part, link))
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


class Lock:
    """The cam between key and pin, the key's detents, and the record of a forced key. Cleared only at a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.forced = False
        self.over = 0
        self.tau = 0.0
        self.tau_max = 0.0
        self.sent = 0.0  # where the cam last sent the pin (m drawn)
        if env is not None:
            m.set_target(env, "column", "wheel", math.radians(REST_DEG))
            self.step(env)

    def step(self, env):
        s = m.state(env, "lock")
        phi, drawn = s.joint("key"), -s.joint("pin")
        w = _joint_vel(env, "lock", "key")
        # the pin held back: short of where the cam puts it for the key's angle now (a free pin follows within a step)
        held = drawn < cam_draw(phi) - PIN_LAG and drawn < PIN_TRAVEL - PIN_LAG
        phi_pin = math.radians(PHI0) + drawn / CAM + PIN_LAG / CAM  # the furthest the key turns with the pin where it is
        k_c = K_KEY if (held and phi > phi_pin) else 0.0
        self.tau = K_KEY * (phi - phi_pin) if k_c else 0.0  # the torque the cam takes from the key now
        self.tau_max = max(self.tau_max, self.tau)
        self.over = self.over + 1 if self.tau > FORCE_LIMIT else 0
        if self.over >= 3:
            self.forced = True
        ahead = phi + w * DT  # the key's angle at the end of this step, if it keeps turning
        # the cam sends the pin where the key will be, pulling it with no more than the key's forcing torque could through the cam
        self.sent = min(cam_draw(ahead), drawn + PULL_MAX / K_PIN)
        m.set_target(env, "lock", "pin", -self.sent)
        d_deg, k_d = detent(math.degrees(phi))
        k = k_c + k_d
        _drive(env, "lock", "key", (k_c * phi_pin + k_d * math.radians(d_deg)) / k, k)


LOCKWORK = Lock()
m.on_step(LOCKWORK.step)
m.on_reset(LOCKWORK.reset)


# -- the scorer ------------------------------------------------------------------------------------------------------------------------------


def bow_gap(lock_state, tips) -> float:
    """The nearest fingertip point's distance (m) from the key's head."""
    q = lock_state.to_local(tips)
    a = -lock_state.joint("key")  # into the key's own frame: turn back about its axis
    y, z = q[:, 1] - KEY_Y, q[:, 2] - KEY_Z
    ca, sa = math.cos(a), math.sin(a)
    q2 = np.c_[q[:, 0] - BOW_X, ca * y - sa * z, sa * y + ca * z]
    d = np.maximum(np.abs(q2) - np.array(BOW) / 2, 0.0)
    return float(np.min(np.linalg.norm(d, axis=1)))


def rim_gap(col_state, tips) -> float:
    """The nearest fingertip point's distance (m) from the wheel's rim (its section as a 36 x 42 mm box round an oval centre line)."""
    q = col_state.to_local(tips)
    a = -col_state.joint("wheel")
    ca, sa = math.cos(a), math.sin(a)
    y, z = ca * q[:, 1] - sa * q[:, 2], sa * q[:, 1] + ca * q[:, 2]
    ang = np.arctan2(z, y)
    r_mid = 1.0 / np.sqrt((np.cos(ang) / RIM_A) ** 2 + (np.sin(ang) / RIM_B) ** 2)
    dr = np.maximum(np.abs(np.hypot(y, z) - r_mid) - RIM_HALF_R, 0.0)
    dx = np.maximum(np.abs(q[:, 0]) - RIM_HALF_X, 0.0)
    return float(np.min(np.hypot(dr, dx)))


def now(env) -> dict:
    lk, col = m.state(env, "lock"), m.state(env, "column")
    tips = m.fingertips(env)
    k = lk.deg("key")
    gaps = (bow_gap(lk, tips), rim_gap(col, tips))  # the rim's gap is only reported: the wheel is not scored, and a hand may stay on it
    return {"key": k, "wheel": col.deg("wheel"), "drawn": -lk.joint("pin"), "on": 60.0 <= k <= 70.0, "acc": 25.0 <= k <= 40.0,
            "clear": gaps[0] >= 0.03, "gaps": gaps, "forced": LOCKWORK.forced}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0

    def step(self, env):
        c = now(env)
        ok = c["on"] and c["clear"] and not c["forced"]
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if c["forced"]:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = 0.5 if (c["acc"] and c["clear"]) else 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration -----------------------------------------------------------------------------------------------------------------------


def _report(ctx, env, label=""):
    c = now(env)
    print(f"[lock] t {ctx.t:5.1f} {label}key {c['key']:6.2f} deg, wheel {c['wheel']:+6.2f} deg, pin drawn {c['drawn'] * 1000:4.1f} mm, cam torque {LOCKWORK.tau:.2f} "
          f"(max {LOCKWORK.tau_max:.2f}) N m, forced {c['forced']}, gaps bow {c['gaps'][0] * 1000:.0f} rim {c['gaps'][1] * 1000:.0f} mm, score {TRIAL.score:.2f} "
          f"full {TRIAL.full}", flush=True)


def demo_script(ctx):
    """Left: takes the rim at 10 o'clock from the driver's side, turns the wheel 4 degrees anticlockwise, off the lock's wall, and holds it
    there against the tyres' spring (closed loop on the wheel's seen angle). Right: comes down over the key's head, pinches it and turns it
    clockwise through ACC to ON (about the key's axis), lets go and moves away outside the rim. Then the left lets the wheel unwind to where
    the tyres hold it, lets go and moves back off the rim.
    Failure checks (--mode): none does nothing; onearm turns the key without holding the wheel; wrongway holds the wheel 4 degrees the
    other way, onto the lock's wall, while the key is turned; acc stops the key at ACC; hold keeps the key pinched at ON.
    pull is a check of the lock, not a demo: with the pin's cam drive off, a growing pull on the pin (prints where it lets go)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "wrongway", "acc", "hold", "pull"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "pull":
        down_w = ROT @ np.array([0.0, 0.0, -1.0])
        for k in range(160):
            f = 2.5 * k
            _drive(env, "lock", "pin", 0.0, 0.0)
            m.push(env, "lock", tuple(f * down_w), body="pin")
            yield
            if k % 10 == 9:
                st = m.state(env, "lock")
                print(f"[pull] pull {f:5.1f} N: pin drawn {-st.joint('pin') * 1000:5.2f} mm, wheel {m.state(env, 'column').deg('wheel'):+.2f} deg", flush=True)
                if -st.joint("pin") > 0.005:
                    break
        m.push(env, "lock", (0.0, 0.0, 0.0), body="pin")
        return
    if mode == "none":
        for k in range(80):
            yield
        _report(ctx, env, "end: ")
        return

    # -- the left hand on the rim at 10 o'clock: fingers either side of the rim's section, across it, coming from the driver's side along the
    #    column (20 degrees out from it)
    th, psi = math.radians(60.0), math.radians(340.0)
    radial_l = np.array([0.0, math.sin(th), math.cos(th)])
    rim_l = np.array([0.0, RIM_A * math.sin(th), RIM_B * math.cos(th)])
    tan_l = np.array([0.0, math.cos(th), -math.sin(th)])  # the rim's way anticlockwise here
    z_g = ROT @ (math.cos(psi) * np.array([1.0, 0.0, 0.0]) - math.sin(psi) * radial_l)
    x_g = np.cross(ROT @ tan_l, z_g)
    x_g /= np.linalg.norm(x_g)
    grip_rim = d._as_grip(np.c_[x_g, np.cross(z_g, x_g), z_g])
    rim_w = world(rim_l)
    roll = -4.0 if mode == "wrongway" else 4.0  # degrees anticlockwise, as the driver sees it

    def turned(p_local, deg):
        """A point of the wheel, turned anticlockwise (as the driver sees it) by `deg` about the column's axis."""
        a = -math.radians(deg)  # anticlockwise as the driver sees it is negative about the column's X
        y, z = p_local[1], p_local[2]
        return world((p_local[0], math.cos(a) * y - math.sin(a) * z, math.sin(a) * y + math.cos(a) * z))

    def rim_now():
        return turned(rim_l, -ctx.deg("column", "wheel"))

    def hold_left(seconds, deg, gain=0.2):
        """Keep the rim's point turned `deg` anticlockwise for `seconds` (None: for ever): closed loop on the wheel's seen angle, shifting the
        hand's command (an integral: the offset this builds holds the tyres' spring)."""
        from yam_arena import ik
        goal = turned(rim_l, deg)
        c = ik.tcp_pose(l.q, l.base)[0]
        k = 0
        while seconds is None or k < round(seconds * 20):
            c = c + np.clip(gain * (goal - rim_now()), -0.003, 0.003)
            mm, dg = l._solve(c, l.grip.rot)
            l._note(mm, dg, "hold the wheel")
            k += 1
            yield

    take_left = mode != "onearm"

    def holding(gen):
        """Run the right hand's `gen` while the left keeps the wheel where it has turned it."""
        if not take_left:
            yield from gen
            return
        h = hold_left(None, roll)
        for _ in gen:
            next(h)
            yield

    if take_left:
        yield from l.move(rim_w - 0.07 * z_g, grip=grip_rim, finger=FINGER_OPEN, t=2.5, label="in front of the rim at 10 o'clock")
        yield from l.servo(rim_w - 0.07 * z_g, timeout=1.0, label="in front of the rim")
        yield from l.move(rim_w + 0.004 * z_g, t=1.2, label="fingers either side of the rim")
        yield from l.servo(rim_w + 0.004 * z_g, timeout=1.0, label="round the rim")
        yield from l.close(0.030, 45.0, t=0.6, label="grip the rim")
        _report(ctx, env, "rim taken: ")
        yield from l.arc(CENTRE, AXIS_TO_DRIVER, roll, t=1.5, label="turn the wheel")
        yield from hold_left(2.0, roll)
        _report(ctx, env, "wheel turned: ")

    # -- the right hand on the key's head: from above, fingers either side of its 11 mm, the pads over most of the head and clear of the
    #    ignition's face; then about the key's axis, clockwise as the driver sees it
    key_axis_w = world((BOW_X, KEY_Y, KEY_Z))
    pinch = world((BOW_X - 0.004, KEY_Y, KEY_Z))
    up = np.array([0.0, 0.0, 1.0])
    yield from holding(r.move(pinch + 0.04 * up, grip=d.down(0), finger=FINGER_OPEN * 0.45, t=2.5, label="over the key's head"))
    yield from holding(r.servo(pinch + 0.04 * up, timeout=1.0, label="over the key"))
    yield from holding(r.move(pinch, t=1.2, label="fingers either side of the key's head"))
    yield from holding(r.servo(pinch, timeout=1.0, label="at the key's head"))
    yield from holding(r.close(BOW[1], 25.0, t=0.5, label="pinch the key"))
    _report(ctx, env, "key pinched: ")
    target = ACC_DEG + 3.0 if mode == "acc" else ON_DEG + 2.0
    yield from holding(r.arc(key_axis_w, ROT @ np.array([1.0, 0.0, 0.0]), target, t=3.0, label="turn the key"))
    yield from holding(r.wait(1.0))
    _report(ctx, env, "key turned: ")
    if mode == "hold":
        yield from holding(r.wait(4.0))
        _report(ctx, env, "end (key still held): ")
        return
    back = -r.approach_axis()
    yield from holding(r.open(t=0.4, label="let go of the key"))
    yield from holding(r.move(r.at() + 0.04 * back, t=0.8, label="the hand off the key"))
    yield from holding(r.move(np.array([0.35, -0.25, TOP + 0.21]), grip=d.down(0), t=1.5, label="the right hand clear, outside the rim"))
    _report(ctx, env, "key let go: ")
    if take_left:  # let the wheel unwind to where the tyres hold it, then let go and move back off the rim
        yield from l.arc(CENTRE, AXIS_TO_DRIVER, -(roll + REST_DEG), t=2.0, label="let the wheel unwind")
        yield from l.wait(0.4)
        yield from l.open(t=0.5, label="let go of the rim")
        yield from l.move(l.at() - 0.05 * z_g, t=1.0, label="the hand back off the rim")
        yield from l.move(l.at() - 0.04 * z_g + np.array([0.0, 0.0, 0.03]), finger=FINGER_OPEN * 0.3, t=1.0, label="the left hand clear")
    for k in range(70):
        yield
        if k % 35 == 34:
            _report(ctx, env, "end: ")
