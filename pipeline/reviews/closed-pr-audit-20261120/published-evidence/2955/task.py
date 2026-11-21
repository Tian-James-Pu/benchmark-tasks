"""Unscrew a hidden (cache) aerator from a lavatory faucet with its plastic service key, then refit it flush.

The faucet is NVIDIA SimReady's single-lever lavatory faucet (fixture_faucet_singleLever_a01), shown as authored, colliding through convex
hulls taken from its own mesh. Its spout ends in a hidden aerator (a cache aerator, M16.5 x 1 "Tom Thumb" size): nothing of it stands
proud of the spout, and its face has four ribs that only the service key's four lugs can turn. Limescale has bonded its thread.

Model (declared in code):
- Faucet ("faucet"): the frame is the vanity deck (a block). Link `body` is the faucet on its deck mount, a hinge about the faucet's own axis:
  the mounting nut, hand-tight, holds it by friction up to SLIP_NM, past which it turns on the deck and stays turned (a clutch: the joint's
  drive is effort-capped at SLIP_NM and its target is dragged along). Link `aer_z` slides the aerator along its axis; link `aer` turns it.
- The thread (code, each control step): the aerator's axial position follows its turns out at PITCH per turn, for THREAD_TURNS; past them
  it has left the thread and drops DROP (into the key, which carries it). Turning it back the other way re-enters the thread.
- The scale bond: until broken, the aerator is held at its seat by a stiff effort-capped drive (BREAK_NM). It breaks once that torque is
  reached, for the rest of the trial. Then it turns against a running torque RUN_NM, and the seat (turns 0) is a stop it tightens against.
- The key ("key") is a free plastic part: a grip block, a disc with four lugs and a centring pin on top. The lugs turn the aerator only by contact.

The scorer (Trial) keeps four records, cleared only by mechkit's on_reset hook: broken loose, out of the thread, faucet twisted, aerator
forced past flush.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import os as _os

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady, fetch  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Take the aerator out of the faucet's spout with its key, then screw it back in flush."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- layout. The faucet's own frame: x along the spout, z up, origin on its axis at the deck. It stands turned 180 degrees, spout to the robot.
DECK_H = 0.08  # the vanity deck above the table (the basin under the spout is open space down to the table)
FAUCET_YAW = 180.0  # the spout's heading in the world (degrees from straight ahead, + to the left): it points at the robot
FAUCET = np.array([0.33, 0.04, TOP + DECK_H])
_C, _S = math.cos(math.radians(FAUCET_YAW)), math.sin(math.radians(FAUCET_YAW))
AX = 0.095  # the aerator's axis along the spout (faucet frame)
ZF = float(_os.environ.get("AER_ZF", "0.0505"))  # the aerator's face (faucet frame), at the spout's underside


def world(p_local) -> np.ndarray:
    p = np.asarray(p_local, float)
    return FAUCET + np.array([_C * p[0] - _S * p[1], _S * p[0] + _C * p[1], p[2]])


AXW = world((AX, 0.0, ZF))  # the aerator's face centre at the start, in the world

# -- the thread and the bond (estimates: see README, "The model")
PITCH = 0.001  # m per turn (M16.5 x 1)
THREAD_TURNS = 1.0  # turns of engaged thread (fewer than a real one: see the fidelity note)
DROP = 0.004  # m the aerator drops once out of the thread
OUT_TURNS = THREAD_TURNS + 0.1  # past this it is out (fully dropped)
BREAK_NM = float(_os.environ.get("AER_BREAK", "0.2"))  # N m to break the scale bond
RUN_NM = float(_os.environ.get("AER_RUN", "0.04"))  # N m running torque once broken
SLIP_NM = float(_os.environ.get("AER_SLIP", "0.1"))  # N m at which the faucet turns on its deck mount
K_BOND, K_RUN, K_SEAT, SEAT_NM = 40.0, 20.0, 1.5, 1.0  # the seat is the washer: a soft progressive stop
K_MOUNT = 120.0
SEAT_BAND = math.radians(1.0)
TWIST_DEG = 5.0  # the faucet turned this far on the deck: harm
SEATED_DEG = 15.0  # refitted flush: within this of the seat (0.04 mm proud)
FORCED_DEG = 15.0  # driven this far past the seat (0.39 N m into the washer): forced in, harm
CLEAR_M = 0.03

# -- ribs and lugs: four each, at radius R_TOOTH
N_TEETH = 4
R_TOOTH, TOOTH_RAD, TOOTH_TAN, TOOTH_H = 0.0065, 0.003, 0.0015, 0.003
AER_R = 0.0083

LOOKS = {
    "deck": m.Look(color=m.rgb(0.86, 0.85, 0.82), roughness=0.35),
    "aerator": m.Look(color=m.rgb(0.16, 0.16, 0.17), roughness=0.5),
    "aer_face": m.Look(color=m.rgb(0.30, 0.30, 0.31), roughness=0.6),
    "scale": m.Look(color=m.rgb(0.80, 0.77, 0.66), roughness=0.9),
    "mark": m.Look(color=m.rgb(0.95, 0.95, 0.92), roughness=0.5),
    "key": m.Look(color=m.rgb(0.80, 0.12, 0.10), roughness=0.5),
}


def _tooth(theta_deg: float, z0: float, z1: float, cx: float = 0.0) -> m.Mesh:
    b = m.box((TOOTH_RAD, TOOTH_TAN, z1 - z0), (R_TOOTH, 0.0, (z0 + z1) / 2))
    return b.turned(theta_deg, "z").moved((cx, 0.0, 0.0))


# -- the faucet -------------------------------------------------------------------------------------------------------------------------------

_HULLS = json.loads((HERE / "faucet_colliders.json").read_text())
DECK = ((-0.16, 0.02), (-0.28, 0.28))  # the deck block in the faucet frame (x, y), from -DECK_H to 0


def _deck():
    (x0, x1), (y0, y1) = DECK
    size, c = (x1 - x0, y1 - y0, DECK_H), ((x0 + x1) / 2, (y0 + y1) / 2, -DECK_H / 2)
    return [("deck", m.box(size, c), "deck")], [m.col_box("deck_c", size, c)]


def _body_link():
    cols = [m.col_hull(f"f_{name}", m.Mesh(np.array(h["v"]), np.array(h["f"]))) for name, h in _HULLS.items()]
    return m.hinge("body", "Z", (0.0, 0.0, 0.0), -30.0, 30.0, [], cols, mass=0.9, spring=K_MOUNT, damping=0.5, friction=0.0, effort=SLIP_NM,
                   armature=0.002, inertia=(2e-3, 2e-3, 1.5e-3))


def _aer_z_link():
    return m.slider("aer_z", "Z", -(THREAD_TURNS * PITCH + DROP + 0.002), 0.003, [], [], at=(AX, 0.0, ZF), parent="body", mass=0.01,
                    spring=2.0e4, damping=20.0, friction=0.0, effort=300.0, armature=0.02)


def _aer_link():
    c = (AX, 0.0, ZF + 0.0045)
    parts = [("aer_body", m.cylinder(AER_R, 0.009, c), "aerator"),
             ("aer_face", m.cylinder(AER_R - 0.0012, 0.0004, (AX, 0.0, ZF - 0.0002)), "aer_face"),
             ("mark", m.box((0.0045, 0.0012, 0.0003), (AX + 0.0026, 0.0, ZF - 0.00045)), "mark"),
             ("scale", m.cylinder(AER_R + 0.0006, 0.0012, (AX, 0.0, ZF + 0.0008), segments=24), "scale")]
    cols = [m.col_cylinder("aer_c", AER_R, 0.009, c)]
    for k in range(N_TEETH):
        th = k * 360.0 / N_TEETH
        t = _tooth(th, ZF - TOOTH_H, ZF + 0.0002, AX)
        parts.append((f"tooth{k}", t, "aerator"))
        cols.append(m.col_hull(f"tooth{k}_c", t))
    return m.hinge("aer", "Z", (AX, 0.0, ZF), float("-inf"), float("inf"), parts, cols, parent="aer_z", mass=0.01, spring=K_BOND, damping=0.002,
                   friction=0.0, effort=BREAK_NM, armature=0.0005, inertia=(4e-7, 4e-7, 4e-7))


_dp, _dc = _deck()
_FAUCET = m.mechanism("faucet", tuple(FAUCET), [_body_link(), _aer_z_link(), _aer_link()], LOOKS, frame_parts=_dp, frame_colliders=_dc,
                      yaw_deg=FAUCET_YAW, friction=(1.0, 0.8))

FAUCET_PKG, FAUCET_USD = "fixture_faucet_singleLever_a01", "sm_fixture_faucet_singleLever_a01_01.usd"


def _faucet_asset_usda() -> str:
    """The SimReady faucet's look, as authored, inside the `body` link: its meshes and materials only (its own bodies, joints and colliders
    are left out; the link's hulls above collide)."""
    inst = fetch(FAUCET_PKG, FAUCET_USD).parent / "payloads" / "instances.usda"

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

    body = one("faucet_body", "body_01_mesh_00")
    body += one("faucet_lever", "handle_01_mesh_00", """
            double3 xformOp:translate = (0, 0, 0.08399999887)
            uniform token[] xformOpOrder = ["xformOp:translate"]""")
    body += one("faucet_popup", "body_02_mesh_00", """
            double3 xformOp:translate = (-0.02648457512, 0, 0.01573199965)
            quatd xformOp:orient = (0.9848077559, 0, -0.1736481613, 0)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]""")
    return f"""
    def Xform "faucet_asset"
    {{{body}
    }}
"""


def _faucet_with_asset():
    import dataclasses
    plain = _FAUCET.build

    def build() -> str:
        text = plain()
        head = text.index('def Xform "body" (')
        at = text.index("float physxRigidBody:maxDepenetrationVelocity", head)
        at = text.index("\n", at) + 1
        return text[:at] + _faucet_asset_usda() + text[at:]

    return dataclasses.replace(_FAUCET, build=build)


FAUCET_PART = _faucet_with_asset()

# -- the key: a grip block, a disc on it, and four lugs and a centring pin on top (65 mm tall) ----------------------------------------------------
KEY_R = 0.0125  # the disc under the cup
BLOCK = (0.030, 0.024, 0.056)  # the grip block below the disc: 30 mm wide (key's x), 24 mm thick (key's y), 56 mm tall
DISC_H = 0.006
DISC_TOP = BLOCK[2] + DISC_H  # the disc's top face, where the lugs stand, above the key's bottom
KEY_POS = np.array([0.33, -0.085])  # where the key stands on the deck at the start (its axis)
GRIP_Z = 0.024  # the scripted grip's TCP this far above the key's bottom (the pads clear the deck below and the spout above)
H0 = float(_os.environ.get("AER_H0", "100.0"))  # the grip's heading (fingers pointing, world degrees) at the first engagement
KEY_YAW0 = float(_os.environ.get("AER_PICK", "30.0"))  # the key's yaw at the start: its block's wide side faces the robot's right hand
PERIOD = 360.0 / N_TEETH
LUG0 = (FAUCET_YAW + PERIOD / 2 - H0) % PERIOD  # lugs sit mid-way between ribs when the key's yaw is H0

PIN_R = 0.0045  # a pin at the key's centre, inside the ribs' inner ends (5 mm out): it centres the key on the aerator


def _key():
    bc = (0.0, 0.0, BLOCK[2] / 2)
    dc = (0.0, 0.0, BLOCK[2] + DISC_H / 2)
    parts = [("block", m.box(BLOCK, bc), "key"), ("disc", m.cylinder(KEY_R, DISC_H, dc), "key"),
             ("rib_a", m.box((0.020, 0.0012, 0.046), (0.0, BLOCK[1] / 2 + 0.0004, bc[2])), "deck"),
             ("rib_b", m.box((0.020, 0.0012, 0.046), (0.0, -BLOCK[1] / 2 - 0.0004, bc[2])), "deck")]
    cols = [m.col_box("block_c", BLOCK, bc), m.col_cylinder("disc_c", KEY_R, DISC_H, dc)]
    pin = m.lathe([(0.0, DISC_TOP - 0.0002), (PIN_R, DISC_TOP - 0.0002), (PIN_R, DISC_TOP + 0.0005), (0.0022, DISC_TOP + TOOTH_H), (0.0, DISC_TOP + TOOTH_H)], segments=16)
    if _os.environ.get("AER_PIN", "1") == "1":
        parts.append(("pin", pin, "key"))
        cols.append(m.col_hull("pin_c", pin))
    for k in range(N_TEETH):
        t = _tooth(LUG0 + k * PERIOD, DISC_TOP - 0.0002, DISC_TOP + TOOTH_H)
        parts.append((f"lug{k}", t, "key"))
        cols.append(m.col_hull(f"lug{k}_c", t))
    return m.free("key", (float(KEY_POS[0]), float(KEY_POS[1]), TOP + DECK_H), parts, cols, LOOKS, mass=0.025, friction=(float(_os.environ.get("AER_KMU", "0.35")), float(_os.environ.get("AER_KMU", "0.35")) * 0.85),
                  torsional_patch=0.03, yaw_deg=KEY_YAW0)


KEY = _key()

# -- distractors (SimReady) on the vanity deck
DTOP = TOP + DECK_H
SOAP = SimReady("soap_dispenser", pos=(0.44, -0.19, DTOP), size=(0.075, 0.070, 0.182), mass=0.35, package="cleaning_soapDispenser_a01",
                usd="sm_cleaning_soapDispenser_a01_01.usd")
TUMBLER = SimReady("tumbler", pos=(0.43, 0.25, DTOP), size=(0.080, 0.080, 0.085), mass=0.25, package="dishware_glass_tumbler_a01",
                   usd="sm_dishware_glass_tumbler_a01_01.usd")
BAR_SOAP = SimReady("bar_soap", pos=(0.445, 0.15, DTOP), size=(0.060, 0.094, 0.033), mass=0.12, package="cleaning_barSoap_a01",
                    usd="sm_cleaning_barSoap_a01_01.usd", yaw=0.4)

OBJECTS = [FAUCET_PART, KEY, SOAP, TUMBLER, BAR_SOAP]


# -- the thread, the bond, the deck mount ------------------------------------------------------------------------------------------------------


def _joint_pos(env, part, link) -> float:
    asset, j = m._joint(env, part, link)
    return float(m._np(asset.data.joint_pos)[0, j])


def _drive(env, part, link, target, stiffness, effort):
    asset, j = m._joint(env, part, link)
    m._TOUCHED.add((part, link))
    asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiffness), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


def face_drop(turns_out: float) -> float:
    """How far (m) the aerator's face is below its seated height, `turns_out` turns out."""
    t = max(0.0, turns_out)
    return PITCH * min(t, THREAD_TURNS) + DROP * min(1.0, max(0.0, (t - THREAD_TURNS) / (OUT_TURNS - THREAD_TURNS)))


class Mech:
    """The aerator's thread, bond and seat, and the faucet's deck mount. `cum` is the aerator's angle (rad, unwrapped; negative is out)."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.bonded = True
        self.cum = 0.0
        self.raw = None
        self.mount_target = 0.0
        if env is not None:
            self.raw = _joint_pos(env, "faucet", "aer")
            _drive(env, "faucet", "aer", self.raw, K_BOND, BREAK_NM)
            m.set_target(env, "faucet", "aer_z", 0.0)

    def turns_out(self) -> float:
        return -self.cum / (2 * math.pi)

    def step(self, env):
        raw = _joint_pos(env, "faucet", "aer")
        if self.raw is None:
            self.raw = raw
        d = (raw - self.raw + math.pi) % (2 * math.pi) - math.pi
        self.cum += d
        self.raw = raw
        if self.bonded:
            if abs(self.cum) >= 0.95 * BREAK_NM / K_BOND:
                self.bonded = False
            else:
                _drive(env, "faucet", "aer", raw - self.cum, K_BOND, BREAK_NM)
        if not self.bonded:
            if self.cum > 0.0:  # past the seat: a stiff stop pushing it back (it acts only against tightening)
                _drive(env, "faucet", "aer", raw - self.cum, K_SEAT, SEAT_NM)
            else:  # running torque: a clutch whose target is dragged along
                _drive(env, "faucet", "aer", raw - math.copysign(RUN_NM / K_RUN, d) if abs(d) > 1e-6 else raw, K_RUN, RUN_NM)
        m.set_target(env, "faucet", "aer_z", -face_drop(self.turns_out()))
        # the deck mount: friction up to SLIP_NM, and it stays where it slips to
        b = _joint_pos(env, "faucet", "body")
        lim = SLIP_NM / K_MOUNT
        self.mount_target = min(max(self.mount_target, b - lim), b + lim)
        _drive(env, "faucet", "body", self.mount_target, K_MOUNT, SLIP_NM)


MECH = Mech()
m.on_step(MECH.step)
m.on_reset(MECH.reset)


# -- the scorer ------------------------------------------------------------------------------------------------------------------------------


def _key_centre(env) -> np.ndarray:
    s = m.state(env, "key")
    return s.to_world(np.array([[0.0, 0.0, DISC_TOP]]))[0]


def now(env) -> dict:
    fs = m.state(env, "faucet")
    face = world((AX, 0.0, ZF)) + np.array([0.0, 0.0, -face_drop(MECH.turns_out())])
    tips = m.fingertips(env)
    tip_gap = float(np.min(np.linalg.norm(tips - face, axis=1)))
    key_gap = float(np.linalg.norm(_key_centre(env) - face))
    return {"turns": MECH.turns_out(), "body": math.degrees(_joint_pos(env, "faucet", "body")), "tip_gap": tip_gap, "key_gap": key_gap,
            "bonded": MECH.bonded, "z": -fs.joint("aer_z")}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the records are cleared
        self.broken = False
        self.out = False
        self.twisted = False
        self.forced = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.steps = 0
        self.last = {}

    def step(self, env):
        c = now(env)
        self.steps += 1
        self.last = c
        if abs(c["body"]) >= TWIST_DEG:
            self.twisted = True
        if c["turns"] <= -FORCED_DEG / 360.0:
            self.forced = True
        if c["turns"] >= 30.0 / 360.0:
            self.broken = True
        if c["turns"] >= OUT_TURNS:
            self.out = True
        seated = c["turns"] <= SEATED_DEG / 360.0
        ok = self.out and seated and c["key_gap"] >= CLEAR_M and c["tip_gap"] >= CLEAR_M and not self.twisted and not self.forced
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.twisted or self.forced:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif self.out:
            self.score = 0.5
        elif self.broken:
            self.score = 0.25
        else:
            self.score = 0.0
        if self.steps % 40 == 0:
            print(f"[aerator] t {self.steps / 20:5.1f} turns {c['turns']:+.3f} drop {c['z'] * 1000:+.2f} mm body {c['body']:+.2f} deg bonded {c['bonded']} "
                  f"key_gap {c['key_gap'] * 1000:.0f} tip_gap {c['tip_gap'] * 1000:.0f} mm broken {self.broken} out {self.out} twisted {self.twisted} forced {self.forced} "
                  f"score {self.score}", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published starting layout) ------------------------------------------------------------------------
# The right hand holds the key by its grip block, fingers sloping 10 degrees down and closing across the block, its TCP on the key's axis, so
# turning the grip about the vertical turns the key in place. It presses the key up into the aerator and turns it in strokes: a wrist turns
# only so far, so after each stroke it drops the lugs out, turns back and presses them in again. The lugs drop into the ribs' gaps as the key
# turns. The left hand pinches the spout's end from above and holds it until the key is away.
TILT = 80.0
T_TURN, T_DOWN, T_SWING, T_UP = (float(x) for x in _os.environ.get("AER_T", "4.2,0.4,2.0,0.6").split(","))
H1 = float(_os.environ.get("AER_H1", "-5.0"))
T_SETTLE = float(_os.environ.get("AER_SETTLE", "0.3"))  # the stroke's other end
Z_FF = float(_os.environ.get("AER_ZFF", "0.008"))  # m: the hand is commanded this far up while the lugs are in (it presses them up)
DISENGAGE = float(_os.environ.get("AER_DIS", "0.006"))  # m below engaged: lugs out
N_OUT = int(_os.environ.get("AER_NOUT", "6"))
N_IN = int(_os.environ.get("AER_NIN", "6"))
LAST_IN = float(_os.environ.get("AER_LAST", "1.0"))  # the last refit stroke's share of a full stroke
PAD = 0.013


def _grip(phi: float):
    import demokit as d
    return d.tilted(TILT, phi)


def _engaged(turns_out: float) -> float:
    """The TCP's height with the lugs fully in and their tops on the aerator's face, `turns_out` turns out."""
    return AXW[2] - face_drop(turns_out) - TOOTH_H - DISC_TOP + GRIP_Z


# where the key's top sat, relative to the command, in a static probe of the grip under the aerator (mm, by heading): fed back as an offset
_CORR_H = (-10.0, 50.0, 110.0)
_CORR = ((2.5, 0.6), (0.5, 0.7), (-5.0, 0.5))


def _ax(z: float, dx: float = 0.0, dy: float = 0.0, h: float | None = None) -> tuple:
    if h is not None and _os.environ.get("AER_CORR", "0") == "1":
        ex = float(np.interp(h, _CORR_H, [c[0] for c in _CORR])) / 1000
        ey = float(np.interp(h, _CORR_H, [c[1] for c in _CORR])) / 1000
        dx, dy = dx - ex, dy - ey
    return (float(AXW[0] + dx), float(AXW[1] + dy), float(z))


def plan(mode: str = "full"):
    """Both arms' waypoints."""
    R = []
    kz = TOP + DECK_H + GRIP_Z
    a0 = math.radians(KEY_YAW0)
    back = np.array([-math.cos(a0), -math.sin(a0), 0.0])
    pick = np.array([KEY_POS[0], KEY_POS[1], kz])
    g0 = _grip(KEY_YAW0)
    R += [Waypoint(2.2, tuple(pick + 0.07 * back + (0, 0, 0.02)), FINGER_OPEN, "behind the key's grip", g0),
          Waypoint(1.4, tuple(pick + 0.002 * back), FINGER_OPEN, "fingers either side of the grip", g0),
          Waypoint(0.7, tuple(pick + 0.002 * back), FINGER_CLOSED, "hold the key", g0),
          Waypoint(0.8, tuple(pick + (0, 0, 0.012)), FINGER_CLOSED, "lift the key off the deck", g0)]
    low = _engaged(0.0) - 0.012  # the key's lugs well below the aerator
    side = np.array(_ax(low, 0.0, -0.10))
    R += [Waypoint(1.4, (float(pick[0]) - 0.075, float(pick[1]), float(pick[2]) + 0.012), FINGER_CLOSED, "off the deck, over the basin", _grip(KEY_YAW0)),
          Waypoint(1.2, (float(side[0]), float(side[1]), float(low)), FINGER_CLOSED, "down, beside the spout", _grip((KEY_YAW0 + H0) / 2)),
          Waypoint(1.0, tuple(side), FINGER_CLOSED, "turned to the first stroke", _grip(H0)),
          Waypoint(1.6, _ax(low, h=H0), FINGER_CLOSED, "the key under the aerator", _grip(H0)),
          Waypoint(0.8, _ax(_engaged(0.0) + Z_FF, h=H0), FINGER_CLOSED, "lugs up against the aerator", _grip(H0))]

    if mode == "probe":
        for h in (H0, 50.0, H1):
            R += [Waypoint(1.5, _ax(low, h=h), FINGER_CLOSED, "probe low", _grip(h)), Waypoint(2.0, _ax(low, h=h), FINGER_CLOSED, "probe low hold", _grip(h)),
                  Waypoint(1.0, _ax(_engaged(0.0) + Z_FF, h=h), FINGER_CLOSED, "probe up", _grip(h)),
                  Waypoint(2.5, _ax(_engaged(0.0) + Z_FF, h=h), FINGER_CLOSED, "probe up hold", _grip(h)),
                  Waypoint(0.8, _ax(low, h=h), FINGER_CLOSED, "probe down", _grip(h))]
        return {"right": R}

    def turn(h_from, h_to, z, t, label, steps=6):
        hs = [h_from + (k / steps) * (h_to - h_from) for k in range(1, steps + 1)]
        return [Waypoint(t / steps, _ax(z, h=h), FINGER_CLOSED, label, _grip(h)) for h in hs]

    # Strokes are one rib period plus SLACK long, and so is each turn back, so after the first stroke the lugs come back down just SLACK short of
    # the next rib's face, and each stroke turns the aerator one period. `aer` is the aerator's expected angle (deg, out positive).
    w = 2 * math.degrees(math.atan2(TOOTH_TAN / 2, R_TOOTH))  # a lug's and a rib's half widths together
    S = H0 - H1
    aer = S - (PERIOD / 2 - w)  # the first stroke starts mid-gap
    n_out = {"breakonly": 1, "partout": 2}.get(mode, N_OUT)
    for k in range(n_out):
        R += turn(H0, H1, _engaged(aer / 360.0) + Z_FF, T_TURN, f"stroke {k + 1} out")
        if k == n_out - 1 and mode == "full":
            break
        R.append(Waypoint(T_DOWN, _ax(_engaged(aer / 360.0) - DISENGAGE, h=H1), FINGER_CLOSED, "lugs down out of the ribs", _grip(H1)))
        if k == n_out - 1:
            break
        R += turn(H1, H0, _engaged(aer / 360.0) - DISENGAGE, T_SWING, "turn back", steps=4)
        R.append(Waypoint(T_SETTLE, _ax(_engaged(aer / 360.0) - DISENGAGE, h=H0), FINGER_CLOSED, "steady", _grip(H0)))
        R.append(Waypoint(T_UP, _ax(_engaged(aer / 360.0) + Z_FF, h=H0), FINGER_CLOSED, "lugs up against the aerator", _grip(H0)))
        aer += PERIOD
    h_end = H1
    if mode == "full":
        for k in range(N_IN):
            share = LAST_IN if k == N_IN - 1 else 1.0
            h_to = H1 + share * S
            if k > 0:
                R.append(Waypoint(T_DOWN, _ax(_engaged(aer / 360.0) - DISENGAGE, h=H0), FINGER_CLOSED, "lugs down", _grip(H0)))
                R += turn(H0, H1, _engaged(aer / 360.0) - DISENGAGE, T_SWING, "turn back", steps=4)
                R.append(Waypoint(T_SETTLE, _ax(_engaged(aer / 360.0) - DISENGAGE, h=H1), FINGER_CLOSED, "steady", _grip(H1)))
                R.append(Waypoint(T_UP, _ax(_engaged(aer / 360.0) + Z_FF, h=H1), FINGER_CLOSED, "lugs up", _grip(H1)))
            push = share * S - ((PERIOD - 2 * w) if k == 0 else (S - PERIOD))
            aer = max(0.0, aer - max(0.0, push))
            R += turn(H1, h_to, _engaged(aer / 360.0) + Z_FF, T_TURN * share, f"refit stroke {k + 1}")
            h_end = h_to
        print(f"[plan] expected aerator angle at the end: {aer:.1f} deg", flush=True)
    # the key down off the aerator, out from beside the spout, back onto the deck where it stood, let go
    R += [Waypoint(0.6, _ax(low - 0.004, h=h_end), FINGER_CLOSED, "the key down off the aerator", _grip(h_end)),
          Waypoint(1.6, (float(side[0]), float(side[1]), float(low - 0.004)), FINGER_CLOSED, "the key out from under the spout", _grip(min(h_end, 100.0))),
          Waypoint(1.2, (float(pick[0]) - 0.075, float(pick[1]), float(pick[2]) + 0.012), FINGER_CLOSED, "back by the deck", _grip(KEY_YAW0)),
          Waypoint(1.2, tuple(pick + (0, 0, 0.012)), FINGER_CLOSED, "over its place", g0),
          Waypoint(0.7, tuple(pick + (0, 0, 0.002)), FINGER_CLOSED, "the key down on the deck", g0),
          Waypoint(0.6, tuple(pick + (0, 0, 0.002)), FINGER_OPEN, "let go of the key", g0),
          Waypoint(1.5, tuple(pick + 0.08 * back + (0, 0, 0.03)), FINGER_OPEN, "the right hand clear", g0)]
    t_right = sum(w.duration for w in R)
    t_key_away = t_right - (0.6 + 1.2 + 1.2 + 0.7 + 0.6 + 1.5)  # once the key is out from under the spout

    L = []
    if mode not in ("onearm", "none"):
        import demokit as d
        Lg = world((float(_os.environ.get("AER_LX", "0.115")), 0.0, 0.0))
        zg = FAUCET[2] + 0.063 + PAD
        LY = d.down(((FAUCET_YAW + 90.0) % 180.0) - 90.0)
        L += [Waypoint(2.5, (Lg[0], Lg[1], zg + 0.045), FINGER_OPEN, "over the spout", LY),
              Waypoint(1.5, (Lg[0], Lg[1], zg), FINGER_OPEN, "fingers either side of the spout", LY),
              Waypoint(0.8, (Lg[0], Lg[1], zg), FINGER_CLOSED, "hold the spout", LY)]
        t_left = sum(w.duration for w in L)
        L += [Waypoint(max(0.1, t_key_away - t_left), (Lg[0], Lg[1], zg), FINGER_CLOSED, "steady the spout", LY),
              Waypoint(0.6, (Lg[0], Lg[1], zg), FINGER_OPEN, "let go of the spout", LY),
              Waypoint(1.5, (Lg[0] - 0.02, Lg[1] + 0.07, zg + 0.04), FINGER_OPEN, "the left hand clear", LY)]
    if mode == "none":
        return {"right": [Waypoint(1.0, tuple(pick + 0.08 * back + (0, 0, 0.06)), FINGER_OPEN, "nothing", g0)]}
    return {"left": L, "right": R} if L else {"right": R}


def demo():
    return plan(_os.environ.get("AER_MODE", "full"))
