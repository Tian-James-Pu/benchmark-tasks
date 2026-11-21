"""Swap the dead CR2032 in a car's smart-key fob: twist a coin in the shell's seam notch to unclip the halves, replace the
cell + side up and press the shell shut.

On the table: a 3-button smart-key fob (an aftermarket shell for Toyota's HYQ14FBA smart key), 64 x 39 x 15 mm, lying
buttons up with its key end (the end the emergency blade slides out of, already removed) towards the arms. Its two
glass-fibre nylon half-shells meet at a seam 8 mm above the table and are held together by four snap clips. At the key end
the seam has a 12 x 3 mm notch made for a coin or flat blade. Inside, a dead CR2032 sits + side up in the holder on the
circuit board. Beside the fob lie a US quarter and a new CR2032, + side up. A pair of glasses and a mug are distractors.

Mechanism (mechkit): the base half-shell is a free 22 g body; the button half-shell is an 8 g link on a hinge at the far
(key-ring) end, standing in for the interlocking hooks there. The four clips act on that hinge as one latch: shut, they hold
the notch end down with 24 N (four 6 N clips); a wedge that lifts the notch end 1.0 mm frees them, after which the half
moves freely; pushed back down, the clips' ramps resist with 10 N until the gap is under 0.3 mm, where they snap and pull
the seam shut. The quarter and both cells are loose rigid bodies; each cell's + face is its top.
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena import demo as D
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Replace the dead battery in the car key fob with the new one and close the fob."

# -- the fob, in its own frame: origin on the table under the middle, x along its length (key end at -x), z up ----------------
L, W, H = 0.064, 0.039, 0.015  # 64 x 39 x 15 mm
RC = 0.008  # plan-view corner radius
SEAM = 0.008  # the seam's height: base half 8 mm, button half 7 mm
WALL, ROOF = 0.0016, 0.0016
DECK = 0.0048  # top of the base's solid floor and circuit board, where the cell holder starts
NOTCH_W, NOTCH_H = 0.014, 0.003  # the coin notch in the seam at the key end: 14 mm wide, 3 mm tall (1.5 mm into each half)
HINGE_X = L / 2 - 0.002  # the hinge (the interlocking hooks) at the key-ring end, at seam height
ARM = HINGE_X + L / 2  # 62 mm from the hinge to the notch end
CELL_R, CELL_T, CELL_M = 0.010, 0.0032, 0.003  # CR2032: 20.0 x 3.2 mm, 3.0 g
CELL_X = -0.004  # the holder's centre along the fob
COIN_R, COIN_T, COIN_M = 0.02426 / 2, 0.00175, 0.00567  # US quarter: 24.26 x 1.75 mm, 5.67 g
FOB_M, COVER_M = 0.022, 0.008

CLIP_N, RIDE_N = 12.0, 8.0  # the four clips' hold at the notch end, and their ramps' push while being pressed back on
CLIP_TORQUE, RIDE_TORQUE = CLIP_N * ARM, RIDE_N * ARM  # about the hinge, N m
RELEASE_GAP, SNAP_GAP = 0.0010, 0.0003  # notch-end gap that frees the clips, and below which they snap back
K_CLIP = 2.0  # N m/rad: the latch's drive; with its target well past shut it pushes at its effort cap

FX, FY = 0.280, -0.010  # the fob's centre on the table
FOB_YAW = -90.0  # turned so its key end (the notch) faces the left arm (+y) and its far end the right arm
COIN_XY = (0.190, 0.140)
NEW_XY = (0.200, -0.130)
DEAD_DROP = (0.360, -0.120)  # where the demo puts the dead cell down


def fob_world(p):
    """A point in the fob's frame (as laid out on the table) to station coordinates."""
    x, y, z = p
    return FX + y, FY - x, TOP + z

LOOKS = {
    "shell": m.Look(color=m.rgb(0.05, 0.05, 0.055), roughness=0.35),
    "base": m.Look(color=m.rgb(0.09, 0.09, 0.10), roughness=0.55),
    "seam": m.Look(color=m.rgb(0.01, 0.01, 0.01), roughness=0.9),
    "chrome": m.Look(color=m.rgb(0.78, 0.78, 0.80), metallic=0.9, roughness=0.25),
    "rubber": m.Look(color=m.rgb(0.16, 0.16, 0.17), roughness=0.8),
    "panic": m.Look(color=m.rgb(0.72, 0.08, 0.06), roughness=0.6),
    "white": m.Look(color=m.rgb(0.92, 0.92, 0.92), roughness=0.5),
    "pcb": m.Look(color=m.rgb(0.07, 0.32, 0.14), roughness=0.5),
    "holder": m.Look(color=m.rgb(0.70, 0.60, 0.30), metallic=0.8, roughness=0.35),
    "cell": m.Look(color=m.rgb(0.80, 0.80, 0.82), metallic=0.9, roughness=0.28),
    "cell_dark": m.Look(color=m.rgb(0.45, 0.45, 0.47), metallic=0.8, roughness=0.4),
    "coin": m.Look(color=m.rgb(0.74, 0.74, 0.73), metallic=0.85, roughness=0.32),
    "coin_face": m.Look(color=m.rgb(0.62, 0.62, 0.61), metallic=0.85, roughness=0.4),
}


def _rrect(l, w, r, n=6):
    pts = []
    for cx, cy, a0 in ((l / 2 - r, w / 2 - r, 0), (-l / 2 + r, w / 2 - r, 90), (-l / 2 + r, -w / 2 + r, 180), (l / 2 - r, -w / 2 + r, 270)):
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _corner(sx, sy, z0, z1, n=6):
    """A plan-view corner of the shell wall, as the convex circular segment between its outer arc and the chord."""
    cx, cy = sx * (L / 2 - RC), sy * (W / 2 - RC)
    a0 = {(1, 1): 0, (-1, 1): 90, (-1, -1): 180, (1, -1): 270}[(sx, sy)]
    pts = [(cx + RC * math.cos(math.radians(a0 + 90 * k / n)), cy + RC * math.sin(math.radians(a0 + 90 * k / n))) for k in range(n + 1)]
    pts.append((cx, cy))  # include the arc's centre so the piece is a solid quarter disc: thick enough to collide
    return m.prism(pts, z0, z1)


def _walls(tag, z0, z1, notch_z0, notch_z1, notch_inside):
    """Wall pieces between heights z0 and z1: two long sides, the far end, the near (key) end split around the notch with a
    bridge `notch_inside` (the solid part of this half beyond the notch), and four corners. Returns (meshes, colliders)."""
    meshes, cols = [], []
    sl = L - 2 * RC
    sw = W - 2 * RC
    hz, cz = z1 - z0, (z0 + z1) / 2
    for k, sy in enumerate((1, -1)):
        size, c = (sl, WALL, hz), (0.0, sy * (W / 2 - WALL / 2), cz)
        meshes.append((f"{tag}_side{k}", m.box(size, c)))
        cols.append(m.col_box(f"{tag}_side{k}_c", size, c))
    size, c = (WALL, sw, hz), (L / 2 - WALL / 2, 0.0, cz)
    meshes.append((f"{tag}_far", m.box(size, c)))
    cols.append(m.col_box(f"{tag}_far_c", size, c))
    piece = (sw - NOTCH_W) / 2
    for k, sy in enumerate((1, -1)):
        size, c = (WALL, piece, hz), (-L / 2 + WALL / 2, sy * (NOTCH_W / 2 + piece / 2), cz)
        meshes.append((f"{tag}_near{k}", m.box(size, c)))
        cols.append(m.col_box(f"{tag}_near{k}_c", size, c))
    lo, hi = notch_inside
    size, c = (WALL, NOTCH_W, hi - lo), (-L / 2 + WALL / 2, 0.0, (lo + hi) / 2)
    meshes.append((f"{tag}_bridge", m.box(size, c)))
    cols.append(m.col_box(f"{tag}_bridge_c", size, c))
    for k, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        mesh = _corner(sx, sy, z0, z1)
        meshes.append((f"{tag}_corner{k}", mesh))
        cols.append(m.col_hull(f"{tag}_corner{k}_c", mesh))
    return meshes, cols


def _base():
    parts, cols = [], []
    block = m.prism(_rrect(L, W, RC), 0.0, DECK - 0.0012)
    parts.append(("block", block, "base"))
    pcb = m.prism(_rrect(L - 2 * WALL, W - 2 * WALL, RC - WALL), DECK - 0.0012, DECK)
    parts.append(("pcb", pcb, "pcb"))
    cols.append(m.col_hull("block_c", m.prism(_rrect(L, W, RC), 0.0, DECK)))
    wm, wc = _walls("b", DECK, SEAM, SEAM - NOTCH_H / 2, SEAM, (DECK, SEAM - NOTCH_H / 2))
    parts += [(n, mesh, "base") for n, mesh in wm]
    cols += wc
    # the cell holder: two gold retaining tabs either side across the fob, and low lips at the ends where fingers can reach in
    for k, sy in enumerate((1, -1)):
        size, c = (0.012, 0.0015, 0.002), (CELL_X, sy * (CELL_R + 0.0003 + 0.00075), DECK + 0.001)
        parts.append((f"tab{k}", m.box(size, c), "holder"))
        cols.append(m.col_box(f"tab{k}_c", size, c))
    for k, sx in enumerate((1, -1)):
        size, c = (0.001, 0.008, 0.0005), (CELL_X + sx * (CELL_R + 0.0003 + 0.0005), 0.0, DECK + 0.00025)
        parts.append((f"lip{k}", m.box(size, c), "holder"))
        cols.append(m.col_box(f"lip{k}_c", size, c))
    parts.append(("contact", m.cylinder(0.004, 0.0002, (CELL_X, 0.0, DECK + 0.0001), segments=16), "holder"))
    parts.append(("seam_line", m.prism(_rrect(L + 0.0002, W + 0.0002, RC), SEAM - 0.0003, SEAM), "seam"))
    parts.append(("notch_shadow", m.box((0.0004, NOTCH_W, NOTCH_H / 2), (-L / 2 + 0.0001, 0.0, SEAM - NOTCH_H / 4)), "seam"))
    return parts, cols


def _cover():
    parts, cols = [], []
    roof_lo = H - ROOF
    roof = m.join(m.prism(_rrect(L, W, RC), roof_lo, H - 0.0012), m.prism(_rrect(L - 0.002, W - 0.002, RC - 0.001), H - 0.0012, H))
    parts.append(("roof", roof, "shell"))
    cols.append(m.col_hull("roof_c", m.prism(_rrect(L, W, RC), roof_lo, H)))
    wm, wc = _walls("c", SEAM, roof_lo, SEAM, SEAM + NOTCH_H / 2, (SEAM + NOTCH_H / 2, roof_lo))
    parts += [(n, mesh, "shell") for n, mesh in wm]
    cols += wc
    # three buttons (lock, unlock, panic) and a chrome strip: seen only
    for k, (bx, look) in enumerate(((-0.013, "rubber"), (0.003, "rubber"), (0.019, "panic"))):
        parts.append((f"button{k}", m.cylinder(0.0052, 0.0006, (bx, 0.0, H + 0.0002), segments=24), look))
    parts.append(("lock_icon", m.box((0.0028, 0.0022, 0.0002), (-0.013, 0.0, H + 0.0006)), "white"))
    parts.append(("unlock_icon", m.box((0.0028, 0.0010, 0.0002), (0.003, 0.0, H + 0.0006)), "white"))
    parts.append(("trim", m.box((0.050, 0.0015, 0.0003), (0.003, 0.0135, H + 0.0001)), "chrome"))
    parts.append(("trim2", m.box((0.050, 0.0015, 0.0003), (0.003, -0.0135, H + 0.0001)), "chrome"))
    parts.append(("notch_shadow_c", m.box((0.0004, NOTCH_W, NOTCH_H / 2), (-L / 2 + 0.0001, 0.0, SEAM + NOTCH_H / 4)), "seam"))
    return parts, cols


def _fob():
    fp, fc = _base()
    cp, cc = _cover()
    cover = m.hinge("cover", "Y", (HINGE_X, 0.0, SEAM), 0.0, 120.0, cp, cc, mass=COVER_M, spring=K_CLIP, damping=0.004,
                    friction=0.002, effort=CLIP_TORQUE, armature=2e-5, com=(0.0, 0.0, SEAM + 0.004),
                    inertia=(1.1e-6, 2.8e-6, 3.7e-6))
    return m.mechanism("fob", (FX, FY, TOP), [cover], LOOKS, frame_parts=fp, frame_colliders=fc, free_mass=FOB_M, yaw_deg=FOB_YAW,
                       frame_inertia=(2.9e-6, 7.6e-6, 1.03e-5), friction=(0.7, 0.6), solver_iters=48)


def _cell(name, xyz, look_top):
    body = m.cylinder(CELL_R, CELL_T - 0.0004, (0, 0, CELL_T / 2), segments=40)
    top = m.cylinder(CELL_R - 0.0002, 0.0002, (0, 0, CELL_T - 0.0001), segments=40)  # the + face: the whole can's flat top
    neg = m.cylinder(CELL_R - 0.0025, 0.0002, (0, 0, 0.0001), segments=32)  # the - face: a smaller raised disc
    plus = m.join(m.box((0.006, 0.0012, 0.0001), (0, 0, CELL_T + 0.00005)), m.box((0.0012, 0.006, 0.0001), (0, 0, CELL_T + 0.00005)))
    parts = [("can", body, "cell"), ("top", top, look_top), ("neg", neg, "cell_dark"), ("plus", plus, "seam")]
    return m.free(name, xyz, parts, [m.col_cylinder("c", CELL_R, CELL_T, (0, 0, CELL_T / 2))], LOOKS, mass=CELL_M, friction=(0.6, 0.5))


def _coin():
    rim = m.cylinder(COIN_R, COIN_T, (0, 0, COIN_T / 2), segments=48)
    face = m.cylinder(COIN_R - 0.0012, COIN_T + 0.0002, (0, 0, COIN_T / 2), segments=48)
    return m.free("coin", (COIN_XY[0], COIN_XY[1], TOP + 0.0002), [("rim", rim, "coin"), ("face", face, "coin_face")],
                  [m.col_cylinder("c", COIN_R, COIN_T, (0, 0, COIN_T / 2))], LOOKS, mass=COIN_M, friction=(0.8, 0.7))


OBJECTS = [
    _fob(),
    _cell("cell_dead", fob_world((CELL_X, 0.0, DECK + 0.0002)), "cell"),
    _cell("cell_new", (NEW_XY[0], NEW_XY[1], TOP + 0.0002), "cell"),
    _coin(),
    # distractors (SimReady, loose), which scoring ignores
    SimReady("glasses", pos=(0.47, 0.20, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
             usd="sm_accessory_eyeglasses_a01_01.usd", yaw=math.pi / 2),
    SimReady("mug", pos=(0.47, -0.23, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01",
             usd="sm_dishware_mug_a01_01.usda", yaw=2.4),
]

# -- the clips, and scoring ------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 3.0
HOLD_STEPS = 60  # 3 s at 20 Hz: score() reports 1.0 only after the full-credit state has lasted this long
CLEAR = 0.03  # every fingertip point at least 3 cm from the fob and from both cells
SEAT_XY, SEAT_Z, UP_DEG = 0.0025, 0.001, 15.0
CLOSED_GAP, OPEN_GAP = 0.0005, 0.0010
DEBUG = False
DEBUG_FROM, DEBUG_TO = 185, 262


def notch_gap(fob) -> float:
    """The seam's gap at the notch end, the widest point of the seam (m)."""
    return max(0.0, math.sin(fob.joint("cover"))) * ARM


def _box_gap(local, lo, hi) -> float:
    out = np.maximum(np.maximum(lo - local, local - hi), 0.0)
    return float(np.min(np.linalg.norm(out, axis=1)))


def cell_seated_plus_up(fob, cell) -> bool:
    """The cell's bottom centre within 2.5 mm of the holder's centre and 1 mm of the board, and its + face up (axis within 15
    degrees of the fob's)."""
    p = fob.to_local(np.asarray(cell.pos, float))[0]
    flat = math.hypot(p[0] - CELL_X, p[1]) <= SEAT_XY and abs(p[2] - DECK) <= SEAT_Z
    up = float(cell.rotation()[:, 2] @ fob.rotation()[:, 2]) >= math.cos(math.radians(UP_DEG))
    return bool(flat and up)


def cell_out(fob, cell) -> bool:
    """The cell's centre outside the fob's plan outline and on the table (not held up)."""
    p = fob.to_local(np.asarray(cell.pos, float))[0]
    return bool((abs(p[0]) > L / 2 or abs(p[1]) > W / 2) and cell.pos[2] < TOP + 0.006)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.dropped, self.hold, self.score, self.ok = False, 0, 0.0, False
        self.clip = "shut"  # shut, ramp (pressed back on, not yet snapped), free
        self.n = 0
        if env is not None:
            self._drive(env, "shut")

    def _drive(self, env, state):
        if state == "free":
            m.lock(env, "fob", "cover", stiffness=0.0, effort=CLIP_TORQUE)  # no spring: the half stays where it is put
        else:
            m.lock(env, "fob", "cover", stiffness=K_CLIP, effort=CLIP_TORQUE if state == "shut" else RIDE_TORQUE)
            m.set_target(env, "fob", "cover", -CLIP_TORQUE / K_CLIP if state == "shut" else 2.0)
        self.clip = state

    def clips(self, env, fob):
        gap = notch_gap(fob)
        if self.clip == "shut" and gap >= RELEASE_GAP:
            self._drive(env, "free")  # the wedge has lifted the hooks off their catches
        elif self.clip == "free" and gap < RELEASE_GAP:
            self._drive(env, "ramp")  # pressed back down: the hooks ride up their ramps and push back
        elif self.clip == "ramp":
            if gap < SNAP_GAP:
                self._drive(env, "shut")  # past the ramps: the hooks snap over their catches and pull the seam shut
            elif gap >= RELEASE_GAP:
                self._drive(env, "free")
        return gap

    def step(self, env):
        fob = m.state(env, "fob")
        gap = self.clips(env, fob)
        new, dead, coin = m.state(env, "cell_new"), m.state(env, "cell_dead"), m.state(env, "coin")
        for s in (fob, new, dead, coin):
            if s.pos[2] < TOP - 0.03:
                self.dropped = True  # off the table: latched for the trial
        tips = m.fingertips(env)
        clear = (_box_gap(fob.to_local(tips), np.array([-L / 2, -W / 2, 0.0]), np.array([L / 2, W / 2, H])) >= CLEAR
                 and all(float(np.min(np.linalg.norm(tips - np.asarray(c.pos), axis=1))) >= CLEAR for c in (new, dead)))
        flat = fob.tilt_deg() <= 10.0 and fob.pos[2] <= TOP + 0.003
        still = float(np.linalg.norm(fob.lin_vel)) < 0.01 and float(np.linalg.norm(fob.ang_vel)) < 0.2
        seated = cell_seated_plus_up(fob, new)
        out = cell_out(fob, dead)
        closed = gap < CLOSED_GAP
        self.ok = bool(seated and out and closed and flat and still and clear and not self.dropped)
        self.hold = self.hold + 1 if self.ok else 0
        if self.dropped:
            self.score = 0.0
        elif self.ok and self.hold >= HOLD_STEPS:
            self.score = 1.0
        elif seated and out:
            self.score = 0.75
        elif out:
            self.score = 0.5
        elif gap >= OPEN_GAP:
            self.score = 0.25
        else:
            self.score = 0.0
        self.n += 1
        if DEBUG and (self.n % DEBUG == 0 or DEBUG_FROM <= self.n <= DEBUG_TO):
            pd = fob.to_local(np.asarray(dead.pos, float))[0]
            pn = fob.to_local(np.asarray(new.pos, float))[0]
            pc = fob.to_local(np.asarray(coin.pos, float))[0]
            tl, tr = tips[:12].mean(0), tips[12:].mean(0)
            tlo, tro = tips[:12, 2].min() - TOP, tips[12:, 2].min() - TOP
            f3 = lambda v: "(" + ",".join(f"{x*1000:.1f}" for x in v) + ")"
            print(f"[fob] n={self.n} q={fob.joint('cover')*1000:.2f}mrad gap={gap*1000:.2f}mm deg={fob.deg('cover'):.1f} clip={self.clip} fob={f3(fob.pos - np.array([0, 0, TOP]))} tilt={fob.tilt_deg():.1f} "
                  f"coin_l={f3(pc)} coin_up={coin.rotation()[2, 2]:.2f} dead_l={f3(pd)} new_l={f3(pn)} new_up={new.rotation()[2, 2]:.2f} "
                  f"L={f3(tl - np.array([0, 0, TOP]))} Llow={tlo*1000:.1f} R={f3(tr - np.array([0, 0, TOP]))} Rlow={tro*1000:.1f} "
                  f"seated={seated} out={out} clear={clear} score={self.score}", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.ok and not TRIAL.dropped)


# -- the scripted demo: open loop, from the published starting layout ------------------------------------------------------
class Grip(float):
    """A waypoint yaw carrying a whole TCP rotation (the same device as demokit's grips), so a waypoint can roll the wrist."""

    rot = np.eye(3)


if not getattr(D.grip_rotation, "_takes_grips", False):
    _plain_rot = D.grip_rotation

    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return _plain_rot(yaw) if rot is None else rot

    _grip_rotation._takes_grips = True
    _grip_rotation._plain = _plain_rot
    D.grip_rotation = _grip_rotation
_PLAIN = D.grip_rotation._plain



def grip(yaw_deg=0.0, roll_deg=0.0, lean_deg=0.0) -> Grip:
    """Straight down turned `yaw_deg` about the vertical, leaned `lean_deg` about world x, then rolled `roll_deg` about world y
    (the coin's insertion axis)."""
    g = Grip(0.0)
    g.rot = (axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(roll_deg))
             @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(lean_deg)) @ _PLAIN(math.radians(yaw_deg)))
    return g


MODE = "full"  # red-team copies change it
YAW_L, YAW_R = -90.0, 90.0
LEAN_R = 45.0
HI = TOP + 0.08
PARK_L, PARK_R = (0.20, 0.27, TOP + 0.20), (0.20, -0.27, TOP + 0.20)
TIP = 0.013  # TCP above the fingertips' bottom
COIN_BACK, COIN_DOWN = 0.006, 0.0116  # the held coin's centre: 6 mm ahead of the pads' middle (towards -y), 11.6 mm below the TCP
INSERT = 0.0030  # how far the coin's edge is sent into the notch (it lands about 1 mm less: the arm stops short)
PUSH_IN = 0.020  # and during the twist the hand keeps pushing it in, as a thumb does
COIN_RISE = 0.0020  # the coin sits this much lower in the pads after the lift than when picked, and the arm sags: aim higher
ROLL = 25.0  # the twist
LEVER = 0.008  # then the coin lifts the freed notch end this much
OPEN_TO = 112.0  # the button half is swung open past upright to here
PINCH_X = -0.018  # where a hand pinches the button half's sides, along the fob (fob frame)
PINCH_UP = 0.0035  # and how far above the seam (button half's frame)
SHUT_TO = 4.0
LIGHT = -(W / 2 - 0.002)  # finger opening for a light pinch across the fob's 39 mm: 2 mm of squeeze on each side
HOLD_F = -(W / 2 - 0.003)
CELL_OPEN = -0.0135  # fingers either side of a 20 mm cell


def _coin_tcp(c, roll_deg):
    """TCP position putting the held coin's centre at `c` with the wrist rolled `roll_deg` about y through the coin."""
    off = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(roll_deg)) @ np.array([0.0, -COIN_BACK, -COIN_DOWN])
    return tuple(float(v) for v in np.asarray(c, float) - off)


def _pinch_point(deg):
    """World point on the button half's side wall at PINCH_X, PINCH_UP above the seam, with the half opened `deg`."""
    a = math.radians(deg)
    rx, rz = PINCH_X - HINGE_X, PINCH_UP
    return fob_world((HINGE_X + rx * math.cos(a) + rz * math.sin(a), 0.0, SEAM - rx * math.sin(a) + rz * math.cos(a)))


def _arc(d0, d1, steps, t, finger, label, g):
    out = []
    for k in range(1, steps + 1):
        d = d0 + (d1 - d0) * k / steps
        p = _pinch_point(d)
        out.append(Waypoint(t / steps, (p[0], p[1], p[2] + TIP - 0.0025), finger, f"{label} {d:.0f} deg", g))
    return out


def _dur(wps):
    return sum(w.duration for w in wps)


def _wait_until(wps, t, pos, finger, label, g):
    d = t - _dur(wps)
    if d > 0:
        wps.append(Waypoint(d, pos, finger, label, g))


def demo():
    if MODE == "nothing":
        return {"left": [Waypoint(40.0, PARK_L, FINGER_OPEN, "wait")], "right": [Waypoint(40.0, PARK_R, FINGER_OPEN, "wait")]}
    gl = grip(YAW_L)  # left hand, fingers closing along x (across the fob)
    gr = grip(YAW_R, 0.0, LEAN_R)  # right hand, fingers closing along x, leaned away from the left hand
    g0 = grip(0)  # fingers closing along y (along the fob)
    left, right = [], []
    # right: pinch the fob's far end lightly across its width, fingertips 2 mm above the table, to hold it for the twist
    hold = fob_world((0.023, 0.0, 0.002 + TIP))
    right += [Waypoint(2.5, (hold[0], hold[1], HI), FINGER_OPEN, "right: over the fob's far end", gr),
              Waypoint(1.2, hold, FINGER_OPEN, "right: fingers either side", gr),
              Waypoint(0.8, hold, HOLD_F, "right: hold the fob", gr)]
    if MODE == "pinch":  # red team: no coin; pinch the button half's sides hard at the notch end and lift it
        p_c = _pinch_point(0.0)
        left += [Waypoint(3.0, (p_c[0], p_c[1], p_c[2] + 0.04), FINGER_OPEN, "left: over the notch end", gl),
                 Waypoint(1.2, (p_c[0], p_c[1], p_c[2] + TIP - 0.0025), FINGER_OPEN, "left: fingers either side", gl),
                 Waypoint(0.8, (p_c[0], p_c[1], p_c[2] + TIP - 0.0025), FINGER_CLOSED, "left: pinch hard", gl)]
        _wait_until(right, _dur(left), hold, HOLD_F, "right: hold", gr)
        left += _arc(0.0, 30.0, 6, 3.0, FINGER_CLOSED, "left: lift", gl)
        end = _dur(left) + 3.0
        _wait_until(left, end, left[-1].pos, FINGER_CLOSED, "left: hold", gl)
        _wait_until(right, end, hold, HOLD_F, "right: hold", gr)
        return {"left": left, "right": right}
    # left: pick the quarter up flat by its rim
    cx, cy = COIN_XY
    pick = (cx, cy + COIN_BACK, TOP + 0.0125)
    left += [Waypoint(2.0, (pick[0], pick[1], TOP + 0.06), FINGER_OPEN, "left: over the quarter", gl),
             Waypoint(1.2, pick, FINGER_OPEN, "left: fingers either side of it", gl),
             Waypoint(0.8, pick, FINGER_CLOSED, "left: pinch it by its rim", gl),
             Waypoint(1.0, (pick[0], pick[1], TOP + 0.06), FINGER_CLOSED, "left: lift", gl)]
    # left: the coin's edge into the notch, then the twist
    c_in = np.array(fob_world((-L / 2 - COIN_R + INSERT, 0.0, SEAM + COIN_RISE)))
    c_back = c_in + np.array([0.0, 0.010, 0.0])
    left += [Waypoint(2.0, _coin_tcp(c_back + np.array([0, 0, 0.03]), 0), FINGER_CLOSED, "left: in front of the notch", gl),
             Waypoint(1.2, _coin_tcp(c_back, 0), FINGER_CLOSED, "left: down to the seam", gl),
             Waypoint(1.2, _coin_tcp(c_in, 0), FINGER_CLOSED, "left: the coin's edge into the notch", gl)]
    _wait_until(right, _dur(left), hold, HOLD_F, "right: hold", gr)
    lever_end = c_in
    if MODE != "pinch":
        for k in range(1, 6):
            r = ROLL * k / 5
            c_tw = c_in + np.array([0.0, -PUSH_IN * k / 5, 0.0])
            left.append(Waypoint(0.5, _coin_tcp(c_tw, r), FINGER_CLOSED, f"left: twist the coin {r:.0f} deg", grip(YAW_L, r)))
        left.append(Waypoint(0.5, _coin_tcp(c_tw, ROLL), FINGER_CLOSED, "left: hold the twist", grip(YAW_L, ROLL)))
        c_up = c_tw + np.array([0.0, -0.0005, LEVER])
        lever_end = c_up
        left += [Waypoint(1.0, _coin_tcp(c_up, ROLL), FINGER_CLOSED, "left: lever the freed end up", grip(YAW_L, ROLL)),
                 Waypoint(1.0, _coin_tcp(c_up + np.array([0, 0.012, 0]), ROLL), FINGER_CLOSED, "left: coin out", grip(YAW_L, ROLL))]
    left += [Waypoint(1.2, _coin_tcp(lever_end + np.array([0, 0.02, 0.05]), 0), FINGER_CLOSED, "left: up", gl),
             Waypoint(1.5, (0.17, 0.11, TOP + 0.03), FINGER_CLOSED, "left: over the table", gl),
             Waypoint(0.5, (0.17, 0.11, TOP + 0.03), FINGER_OPEN, "left: put the coin down", gl),
             Waypoint(0.8, (0.17, 0.11, HI), FINGER_OPEN, "left: up", gl)]
    _wait_until(right, _dur(left) - 4.0, hold, HOLD_F, "right: hold", gr)  # the right hand lets go once the coin is out
    right += [Waypoint(0.5, hold, FINGER_OPEN, "right: let go", gr),
              Waypoint(1.0, (hold[0], hold[1], HI), FINGER_OPEN, "right: up", gr),
              Waypoint(1.5, PARK_R, FINGER_OPEN, "right: out of the way", g0)]
    # left: pinch the button half's sides near its free end and swing it open past upright
    d0 = math.degrees(math.asin(min(1.0, (LEVER + 0.003) / ARM)))
    p0 = _pinch_point(d0)
    left += [Waypoint(1.5, (p0[0], p0[1], p0[2] + 0.04), FINGER_OPEN, "left: over the freed half", gl),
             Waypoint(1.0, (p0[0], p0[1], p0[2] + TIP - 0.0025), FINGER_OPEN, "left: fingers either side of it", gl),
             Waypoint(0.6, (p0[0], p0[1], p0[2] + TIP - 0.0025), LIGHT, "left: light pinch", gl)]
    left += _arc(d0, OPEN_TO, 10, 5.0, LIGHT, "left: swing it open", gl)
    pe = _pinch_point(OPEN_TO)
    left += [Waypoint(0.5, (pe[0], pe[1], pe[2] + TIP - 0.0025), FINGER_OPEN, "left: let go", gl),
             Waypoint(0.8, (pe[0], pe[1], pe[2] + 0.05), FINGER_OPEN, "left: up", gl),
             Waypoint(1.5, PARK_L, FINGER_OPEN, "left: out of the way", g0)]
    # right: the dead cell out (fingers along the fob, into the holder's open ends), the new one in
    hx, hy, _ = fob_world((CELL_X, 0.0, 0.0))
    _wait_until(right, _dur(left) - 1.5, PARK_R, FINGER_OPEN, "right: wait", g0)
    in_z = TOP + DECK + 0.0011 + TIP
    right += [Waypoint(1.5, (hx, hy, HI), CELL_OPEN, "right: over the holder", g0),
              Waypoint(1.2, (hx, hy, in_z), CELL_OPEN, "right: fingers into the holder's ends", g0),
              Waypoint(0.8, (hx, hy, in_z), FINGER_CLOSED, "right: pinch the dead cell", g0),
              Waypoint(1.0, (hx, hy, HI), FINGER_CLOSED, "right: lift it out", g0),
              Waypoint(1.5, (DEAD_DROP[0], DEAD_DROP[1], TOP + 0.03), FINGER_CLOSED, "right: over the table", g0),
              Waypoint(0.8, (DEAD_DROP[0], DEAD_DROP[1], TOP + 0.0145), FINGER_CLOSED, "right: down", g0),
              Waypoint(0.5, (DEAD_DROP[0], DEAD_DROP[1], TOP + 0.0145), CELL_OPEN, "right: put it down", g0),
              Waypoint(0.8, (DEAD_DROP[0], DEAD_DROP[1], HI), CELL_OPEN, "right: up", g0)]
    if MODE != "nonew":
        nx, ny = NEW_XY
        right += [Waypoint(1.5, (nx, ny, TOP + 0.05), CELL_OPEN, "right: over the new cell", g0),
                  Waypoint(1.2, (nx, ny, TOP + 0.0135), CELL_OPEN, "right: fingers either side", g0),
                  Waypoint(0.8, (nx, ny, TOP + 0.0135), FINGER_CLOSED, "right: pinch it, + side up", g0),
                  Waypoint(1.0, (nx, ny, HI), FINGER_CLOSED, "right: lift", g0),
                  Waypoint(1.5, (hx, hy, HI), FINGER_CLOSED, "right: over the holder", g0),
                  Waypoint(1.2, (hx, hy, in_z), FINGER_CLOSED, "right: into the holder", g0),
                  Waypoint(0.6, (hx, hy, in_z), CELL_OPEN, "right: let go", g0),
                  Waypoint(1.0, (hx, hy, HI), CELL_OPEN, "right: up", g0)]
    right += [Waypoint(1.5, PARK_R, FINGER_OPEN, "right: out of the way", g0)]
    # left: swing the button half back down, then press the notch end until the clips snap
    _wait_until(left, _dur(right) - 1.5, PARK_L, FINGER_OPEN, "left: wait", g0)
    left += [Waypoint(1.5, (pe[0], pe[1], pe[2] + 0.04), FINGER_OPEN, "left: over the open half", gl),
             Waypoint(1.0, (pe[0], pe[1], pe[2] + TIP - 0.0025), FINGER_OPEN, "left: fingers either side", gl),
             Waypoint(0.6, (pe[0], pe[1], pe[2] + TIP - 0.0025), LIGHT, "left: light pinch", gl)]
    left += _arc(OPEN_TO, SHUT_TO, 10, 5.0, LIGHT, "left: swing it back", gl)
    ps = _pinch_point(SHUT_TO)
    press = fob_world((-L / 2 + 0.012, 0.0, 0.0))
    if MODE != "noclose":
        left += [Waypoint(0.5, (ps[0], ps[1], ps[2] + TIP - 0.0025), FINGER_OPEN, "left: let go", gl),
                 Waypoint(0.8, (ps[0], ps[1], ps[2] + 0.03), FINGER_CLOSED, "left: up, fingers closed", gl),
                 Waypoint(1.0, (press[0], press[1], TOP + H + 0.015 + TIP), FINGER_CLOSED, "left: over the notch end", gl),
                 Waypoint(1.2, (press[0], press[1], TOP + H - 0.003 + TIP), FINGER_CLOSED, "left: press the halves together", gl),
                 Waypoint(0.6, (press[0], press[1], TOP + H - 0.003 + TIP), FINGER_CLOSED, "left: hold", gl),
                 Waypoint(0.8, (press[0], press[1], TOP + H + 0.04 + TIP), FINGER_CLOSED, "left: up", gl)]
    else:
        left += [Waypoint(0.5, (ps[0], ps[1], ps[2] + TIP - 0.0025), FINGER_OPEN, "left: let go", gl),
                 Waypoint(0.8, (ps[0], ps[1], ps[2] + 0.04), FINGER_OPEN, "left: up", gl)]
    left += [Waypoint(1.5, PARK_L, FINGER_OPEN, "left: out of the way", g0)]
    end = max(_dur(left), _dur(right)) + 5.0
    _wait_until(left, end, PARK_L, FINGER_OPEN, "left: wait", g0)
    _wait_until(right, end, PARK_R, FINGER_OPEN, "right: wait", g0)
    return {"left": left, "right": right}
