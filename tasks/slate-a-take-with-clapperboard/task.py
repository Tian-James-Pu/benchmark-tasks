"""Slate a take: on the station table a phone on a tabletop tripod films toward the robot, and an acrylic film clapperboard leans back,
sticks shut, against a cardboard box in front of the arms, its writing face toward the phone. The take is slated when the clapperboard is
facing the phone, fully in its picture, and its clapstick is clapped shut once, fast, while the board stays still.

Model (declared in code):
- Clapperboard ("slate"): a free articulation. Its frame is the black acrylic writing board (300 x 200 x 5 mm) with the fixed lower clapstick
  (300 x 25 x 8 mm) along its top edge; its one link is the upper clapstick (300 x 25 x 8 mm) on a hinge pin through both sticks, 12 mm in
  from the board's end on the robot's right (the camera's left, as on the real board). The hinge turns about the board's normal: 0 deg is
  shut (the joint's lower limit: sticks are one articulation and do not collide with each other), 50 deg fully open. The hinge's tension
  screw is joint friction, so the stick stays where it is left; the stick's own weight is left out (the tension screw holds it anyway).
- Slate frame (part frame): origin at the middle of the board's bottom edge, X the board's normal out of the writing face (toward the
  phone), Y along the board (world +y when it faces the phone), Z up the board.
- Scored at every physics step (240 Hz, the same rate as the real trial's 240 fps video): the stick's opening angle, the slate's pose.
- The phone's view is a pinhole check of the slate's four outer corners against the field of view of a 26 mm-equivalent main camera filming
  16:9 (69.4 x 42.6 deg), not a render.

Scoring (the first clap is the take's sync mark and is the one judged; a clap is the stick reaching 0.5 deg or less, the sticks meeting,
after it has been opened 15 deg or more):
- 0: no clap, or at the first clap the board's face is more than 20 deg off the line from its centre to the phone's lens, or any of its
  four outer corners is outside the phone's picture.
- 0.25: the first clap faces the phone and is in its picture.
- 0.5: and it is fast: at most 12 frames at 240 fps (50 ms) from the last frame with the stick 10 deg or more open to the first frame shut.
- 0.75: and the board is still: its face centre stays within 1 cm (range of positions) over the 60 frames (0.25 s) before and after the clap.
- 1.0 (success): and for the 1 s after the clap the stick never opens 5 deg or more again (no bounce, no second clap) and the board stays
  facing the phone and in its picture.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Slate the take: clap the clapperboard shut once, with it facing the phone camera and fully in its picture."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
TOP = TABLE_TOP[SCENE]
CONTROL_DT = 0.05
DEBUG = bool(os.environ.get("SLATE_DEBUG"))

# -- the clapperboard (dimensions: README "The clapperboard") ----------------------------------------------------------------------------------
W = 0.300  # board and sticks, long side
BOARD_H, BOARD_T = 0.200, 0.005
STICK_H, STICK_T = 0.025, 0.008
Z_SEAM = BOARD_H + STICK_H  # where the two sticks meet
H_ALL = BOARD_H + 2 * STICK_H  # 250 mm overall
HINGE_IN = 0.012
HINGE = (0.0, -W / 2 + HINGE_IN, Z_SEAM)  # the pin, on the robot's right end
R_TIP = W - HINGE_IN  # hinge to the stick's free end: 288 mm
OPEN_MAX = 50.0
BOARD_MASS = 0.43  # board + lower stick (acrylic, 1.18 g/cm3: 354 g + 71 g)
STICK_MASS = 0.071  # 60 cm3 of acrylic
HINGE_FRICTION = 0.06

# -- the layout ----------------------------------------------------------------------------------------------------------------------------------
LEAN = 40.0  # deg: the slate leans back on the box's far top edge
BOX_SIZE = (0.264, 0.345, 0.166)  # x, y, z once turned 90 deg (SimReady box_cardboard_d01: 345 x 264 x 166 mm)
BOX_FAR_X = 0.140
BOX_C = (BOX_FAR_X - BOX_SIZE[0] / 2, 0.0)
SLATE_X = BOX_FAR_X + BOX_SIZE[2] * math.tan(math.radians(LEAN)) + BOARD_T / (2 * math.cos(math.radians(LEAN))) + 0.001
SLATE_Y = 0.0
LENS = np.array([0.72, 0.0, TOP + 0.130])  # the phone's main camera, looking along -x
TAN_H, TAN_V = 18.0 / 26.0, 10.125 / 26.0  # half field of view of a 26 mm-equivalent lens, 16:9 frame (36 x 20.25 mm of a 36 x 24 frame)

# -- scoring thresholds --------------------------------------------------------------------------------------------------------------------------
ARM_DEG = 15.0  # a clap needs the stick opened at least this far first
TEN_DEG = 10.0  # the closing time is counted from the last frame at or above this
SHUT_DEG = 0.5  # sticks meeting
REOPEN_DEG = 5.0  # a bounce or second clap
FACE_DEG = 20.0
FAST_FRAMES = 12  # at 240 fps: 50 ms
STILL_M = 0.010
WINDOW = 60  # frames either side (0.25 s at 240 Hz)
HOLD_S = 1.0

LOOKS = {
    "acrylic_black": m.Look(color=m.rgb(0.03, 0.03, 0.035), roughness=0.15),
    "print_white": m.Look(color=m.rgb(0.93, 0.93, 0.92), roughness=0.4),
    "stick_white": m.Look(color=m.rgb(0.94, 0.94, 0.93), roughness=0.2),
    "stick_black": m.Look(color=m.rgb(0.03, 0.03, 0.03), roughness=0.2),
    "steel": m.Look(color=m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.3),
    "phone_back": m.Look(color=m.rgb(0.55, 0.62, 0.70), roughness=0.35),
    "phone_frame": m.Look(color=m.rgb(0.62, 0.66, 0.70), metallic=0.8, roughness=0.3),
    "glass": m.Look(color=m.rgb(0.02, 0.02, 0.025), roughness=0.05),
    "tripod": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "tape": m.Look(color=m.rgb(0.95, 0.75, 0.10), roughness=0.8),
}


def _x_extrude(outline_yz, x0, x1) -> m.Mesh:
    """A solid with a (Y, Z) outline, extruded along X from x0 to x1."""
    p = m.prism(np.asarray(outline_yz, float), x0, x1)
    return m.Mesh(p.v[:, [2, 0, 1]], p.f)  # (x, y, z) -> (Y, Z, X): a cyclic turn, faces stay outward


def _stick_meshes(prefix: str, z0: float):
    """A white clapstick, 300 x 25 x 8 mm, from z0 up, with black 45-degree stripes on both faces."""
    parts = [(f"{prefix}_body", m.box((STICK_T, W, STICK_H), (0.0, 0.0, z0 + STICK_H / 2)), "stick_white")]
    band = 0.025
    sgn = 1.0 if prefix == "upper" else -1.0  # the two sticks' stripes lean opposite ways, as printed
    y = -W / 2 + 0.01
    k = 0
    while y + band + STICK_H < W / 2:
        a, b = y, y + band
        outline = [(a, z0 + 0.001), (b, z0 + 0.001), (b + sgn * (STICK_H - 0.002), z0 + STICK_H - 0.001), (a + sgn * (STICK_H - 0.002), z0 + STICK_H - 0.001)]
        if sgn < 0:
            outline = [(a + STICK_H, z0 + 0.001), (b + STICK_H, z0 + 0.001), (b + 0.002, z0 + STICK_H - 0.001), (a + 0.002, z0 + STICK_H - 0.001)]
        for side, (x0, x1) in (("f", (STICK_T / 2, STICK_T / 2 + 0.0004)), ("b", (-STICK_T / 2 - 0.0004, -STICK_T / 2))):
            parts.append((f"{prefix}_stripe{k}{side}", _x_extrude(outline, x0, x1), "stick_black"))
        y += 2 * band
        k += 1
    return parts


def _board_print():
    """The white printed fields on the writing face (PROD., ROLL / SCENE / TAKE, DIRECTOR / CAMERA / DATE)."""
    x0, x1 = BOARD_T / 2, BOARD_T / 2 + 0.0003
    lines = []
    lw = 0.0025
    for i, z in enumerate((0.150, 0.100, 0.050)):
        lines.append((f"print_h{i}", m.box((x1 - x0, W - 0.03, lw), ((x0 + x1) / 2, 0.0, z)), "print_white"))
    for i, y in enumerate((-0.05, 0.05)):
        lines.append((f"print_v{i}", m.box((x1 - x0, lw, 0.05), ((x0 + x1) / 2, y, 0.125)), "print_white"))
    for i, y in enumerate((-0.04, 0.05)):
        lines.append((f"print_w{i}", m.box((x1 - x0, lw, 0.05), ((x0 + x1) / 2, y, 0.025)), "print_white"))
    frame = [("print_top", (W - 0.016, lw), (0.0, BOARD_H - 0.008)), ("print_bot", (W - 0.016, lw), (0.0, 0.008)),
             ("print_l", (lw, BOARD_H - 0.016), (W / 2 - 0.008, BOARD_H / 2)), ("print_r", (lw, BOARD_H - 0.016), (-W / 2 + 0.008, BOARD_H / 2))]
    for name, (sy, sz), (cy, cz) in frame:
        lines.append((name, m.box((x1 - x0, sy, sz), ((x0 + x1) / 2, cy, cz)), "print_white"))
    return lines


def _slate():
    frame_parts = [("board", m.box((BOARD_T, W, BOARD_H), (0.0, 0.0, BOARD_H / 2)), "acrylic_black")] + _board_print() + _stick_meshes("lower", BOARD_H)
    frame_parts.append(("pin", m.cylinder(0.004, STICK_T + 0.006, HINGE, axis="x", segments=16), "steel"))
    frame_cols = [m.col_box("board_c", (BOARD_T, W, BOARD_H), (0.0, 0.0, BOARD_H / 2)),
                  m.col_box("lower_c", (STICK_T, W, STICK_H), (0.0, 0.0, BOARD_H + STICK_H / 2))]
    stick_cols = [m.col_box("upper_c", (STICK_T, W, STICK_H), (0.0, 0.0, Z_SEAM + STICK_H / 2))]
    ix = STICK_MASS * (W ** 2 + STICK_H ** 2) / 12  # about the board's normal, through the stick's centre
    stick = m.hinge("stick", "X", HINGE, 0.0, OPEN_MAX, _stick_meshes("upper", Z_SEAM), stick_cols, mass=STICK_MASS, friction=HINGE_FRICTION,
                    damping=0.002, armature=0.0002, inertia=(ix, STICK_MASS * (STICK_H ** 2 + STICK_T ** 2) / 12, ix))
    a = math.radians(LEAN)
    quat = (0.0, math.sin(-a / 2), 0.0, math.cos(a / 2))  # about world y: the top leans back toward the arms
    mi = BOARD_MASS
    inertia = (mi * (W ** 2 + BOARD_H ** 2) / 12, mi * (BOARD_H ** 2 + BOARD_T ** 2) / 12, mi * (W ** 2 + BOARD_T ** 2) / 12)
    return m.mechanism("slate", (SLATE_X, SLATE_Y, TOP + 0.001), [stick], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols,
                       free_mass=BOARD_MASS, start_quat=quat, frame_inertia=inertia, friction=(0.6, 0.5))


SLATE = _slate()


def _phone_rig():
    """iPhone 15 (147.6 x 71.6 x 7.8 mm) in landscape in a phone clamp on a tabletop tripod, its main camera at LENS, looking along -x.
    Fixed in place. A strip of yellow tape on the table along the camera's axis (a measuring mark for the real trial)."""
    parts, cols = [], []
    L, Hh, D = 0.1476, 0.0716, 0.0078
    c = LENS + np.array([D / 2, -(L / 2 - 0.020), -(Hh / 2 - 0.020)])  # the lens 20 mm in from the top edge and one end
    loc = lambda p: tuple(np.asarray(p) - np.array([0.0, 0.0, TOP]))  # noqa: E731  part frame: origin on the table top
    parts.append(("phone_body", m.box((D, L, Hh), loc(c)), "phone_frame"))
    parts.append(("phone_back", m.box((0.0004, L - 0.004, Hh - 0.004), loc(c - [D / 2 + 0.0002, 0, 0])), "phone_back"))
    parts.append(("phone_screen", m.box((0.0004, L - 0.006, Hh - 0.006), loc(c + [D / 2 + 0.0002, 0, 0])), "glass"))
    parts.append(("lens_ring", m.cylinder(0.0075, 0.0016, loc(LENS - [0.0008, 0, 0]), axis="x"), "phone_frame"))
    parts.append(("lens", m.cylinder(0.0055, 0.0018, loc(LENS - [0.0012, 0, 0]), axis="x"), "glass"))
    parts.append(("lens2", m.cylinder(0.0055, 0.0018, loc(LENS + [-0.0012, -0.022, 0]), axis="x"), "glass"))
    cols.append(m.col_box("phone_c", (D, L, Hh), loc(c)))
    # clamp across the phone's middle, ball head, centre post and three legs
    cx = c[0] + D / 2 + 0.006
    parts.append(("clamp_back", m.box((0.008, 0.06, Hh + 0.012), loc((cx, c[1], c[2]))), "tripod"))
    parts.append(("clamp_top", m.box((0.016, 0.06, 0.008), loc((c[0] + 0.002, c[1], c[2] + Hh / 2 + 0.004))), "tripod"))
    parts.append(("clamp_bot", m.box((0.016, 0.06, 0.008), loc((c[0] + 0.002, c[1], c[2] - Hh / 2 - 0.004))), "tripod"))
    head = np.array([cx + 0.012, c[1], c[2] - Hh / 2 - 0.018])
    parts.append(("ball", m.sphere(0.012, loc(head)), "tripod"))
    hub = np.array([head[0], head[1], TOP + 0.045])
    parts.append(("post", m.tube([loc(hub), loc(head)], 0.007, sides=12), "tripod"))
    for k in range(3):
        a = math.radians(90.0 + 120.0 * k)
        foot = hub + np.array([0.075 * math.cos(a), 0.075 * math.sin(a), 0.0])
        foot[2] = TOP + 0.006
        parts.append((f"leg{k}", m.tube([loc(hub), loc(foot)], 0.006, sides=10), "tripod"))
        cols.append(m.col_hull(f"leg{k}_c", m.tube([loc(hub), loc(foot)], 0.006, sides=8)))
    cols.append(m.col_box("clamp_c", (0.016, 0.06, Hh + 0.02), loc((cx, c[1], c[2]))))
    # the tape strip on the camera's axis, 48 mm wide, from 0.40 to 0.62 m out
    parts.append(("tape", m.box((0.22, 0.048, 0.0004), (0.51, LENS[1], 0.0002)), "tape"))
    return m.fixed("phone", (0.0, 0.0, TOP), parts, cols, LOOKS)


PHONE = _phone_rig()

BOX = SimReady("box", pos=(BOX_C[0], BOX_C[1], TOP), size=(BOX_SIZE[1], BOX_SIZE[0], BOX_SIZE[2]), mass=1.5, package="box_cardboard_d01",
               usd="sm_box_cardboard_d01_01.usd", yaw=math.pi / 2, convex=True)
# distractors that belong on a film set's table (scoring ignores them)
CUP = SimReady("coffee_cup", pos=(0.52, 0.31, TOP), size=(0.085, 0.085, 0.117), mass=0.25, package="food_beverage_coffeeCup_a01",
               usd="sm_food_beverage_coffeeCup_a01_01.usd")
TAPE_ROLL = SimReady("tape_roll", pos=(0.66, 0.22, TOP), size=(0.10, 0.10, 0.05), mass=0.12, package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd")
NOTEBOOK = SimReady("script_notebook", pos=(0.53, -0.30, TOP), size=(0.229, 0.170, 0.010), mass=0.12, package="misc_notebook_a01",
                    usd="sm_misc_notebook_a01_01.usd", yaw=0.4)

OBJECTS = [SLATE, PHONE, BOX, CUP, TAPE_ROLL, NOTEBOOK]


# -- geometry the scorer reads ---------------------------------------------------------------------------------------------------------------------

CORNERS = np.array([[BOARD_T / 2, sy * W / 2, z] for sy in (-1.0, 1.0) for z in (0.0, H_ALL)])
FACE_CENTRE = np.array([BOARD_T / 2, 0.0, H_ALL / 2])


def view(pos, quat) -> dict:
    """Facing angle (deg) and whether all four corners are in the phone's picture, for a slate pose."""
    R = m.quat_matrix(quat)
    centre = np.asarray(pos) + R @ FACE_CENTRE
    normal = R[:, 0]
    d = LENS - centre
    facing = math.degrees(math.acos(max(-1.0, min(1.0, float(normal @ d / np.linalg.norm(d))))))
    corners = np.asarray(pos) + CORNERS @ R.T
    rel = corners - LENS
    fwd = -rel[:, 0]
    inside = bool(np.all(fwd > 0.05) and np.all(np.abs(rel[:, 1]) <= TAN_H * fwd) and np.all(np.abs(rel[:, 2]) <= TAN_V * fwd))
    return {"facing": facing, "in_frame": inside, "centre": centre, "ok": bool(facing <= FACE_DEG and inside)}


class Clap:
    """Reads the stick and the slate at every physics step (240 Hz) and keeps the record of claps. Cleared only by `reset`."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.k = 0  # physics frames since the reset
        self.armed = False
        self.last10 = None
        self.first = None  # the judged clap: dict
        self.reopened = False
        self.history = []  # (frame, face centre) over the last WINDOW frames
        self.peak = 0.0
        self.per_step = 0
        self.samples_this_step = 0

    def sample(self, env):
        s = m.state(env, "slate")
        th = s.deg("stick")
        self.peak = max(self.peak, th)
        R = m.quat_matrix(s.quat_xyzw)
        centre = np.asarray(s.pos) + R @ FACE_CENTRE
        self.history.append((self.k, centre))
        if DEBUG:
            self.trace = (getattr(self, "trace", []) + [round(th, 1)])[-40:]
        if len(self.history) > 2 * WINDOW + 2:
            self.history.pop(0)
        if th >= TEN_DEG:
            self.last10 = self.k
        if th >= ARM_DEG:
            self.armed = True
        if self.first is not None and th >= REOPEN_DEG and self.k > self.first["frame"]:
            self.reopened = True
        if self.armed and th <= SHUT_DEG:
            self.armed = False
            if self.first is None:
                v = view(s.pos, s.quat_xyzw)
                frames = self.k - self.last10 if self.last10 is not None else 10 ** 6
                self.first = {"frame": self.k, "frames": frames, "facing": v["facing"], "in_frame": v["in_frame"], "ok": v["ok"],
                              "before": [c for f, c in self.history if f >= self.k - WINDOW], "after": [], "done": False}
                if DEBUG:
                    print(f"[slate] CLAP at frame {self.k} ({self.k / 240:.3f} s): {frames} frames from 10 deg to shut, facing {v['facing']:.1f} deg, "
                          f"in frame {v['in_frame']}, peak opening {self.peak:.1f} deg; last 40 frames: {self.trace}", flush=True)
            elif DEBUG:
                print(f"[slate] another clap at frame {self.k}", flush=True)
        if self.first is not None and not self.first["done"] and self.k > self.first["frame"]:
            self.first["after"].append(centre)
            if len(self.first["after"]) >= WINDOW:
                self.first["done"] = True
        self.k += 1
        self.samples_this_step += 1

    def moved(self) -> float:
        """Range (m) of the face centre's positions over the frames around the judged clap so far."""
        pts = np.array(self.first["before"] + self.first["after"])
        return float(np.max(np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2))) if len(pts) else 0.0


CLAP = Clap()


def _install_sampler(env):
    """Call CLAP.sample after every physics step: wraps the scene's per-physics-step buffer update (Isaac Lab calls it once per physics
    step inside the control step)."""
    scene = env.unwrapped.scene
    if getattr(scene, "_slate_sampler", False):
        return
    plain = scene.update

    def update(*a, **k):
        out = plain(*a, **k)
        try:
            CLAP.sample(env)
        except Exception as e:  # never break the simulator over a scorer read
            print(f"[slate] sample failed: {e!r}", flush=True)
        return out

    scene.update = update
    scene._slate_sampler = True


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.score = 0.0
        self.full = False
        self.hold_ok = True
        self.steps = 0
        self.reason = "no clap yet"
        if env is not None:
            CLAP.reset(env)
            _install_sampler(env)

    def step(self, env):
        self.steps += 1
        if CLAP.samples_this_step not in (0, 12) and self.steps > 1 and DEBUG:
            print(f"[slate] note: {CLAP.samples_this_step} physics samples this control step", flush=True)
        CLAP.samples_this_step = 0
        s = m.state(env, "slate")
        v = view(s.pos, s.quat_xyzw)
        f = CLAP.first
        if f is None:
            self.score, self.reason = 0.0, "no clap"
        elif not f["ok"]:
            self.score, self.reason = 0.0, f"first clap not facing/in frame ({f['facing']:.1f} deg, in frame {f['in_frame']})"
        else:
            level = 0.25
            self.reason = "facing and in frame"
            if f["frames"] <= FAST_FRAMES:
                level, self.reason = 0.5, "fast"
                mv = CLAP.moved()
                if mv < STILL_M:
                    level, self.reason = 0.75, f"still ({mv * 1000:.1f} mm)"
                    since = (CLAP.k - 1 - f["frame"]) / 240.0
                    if since <= HOLD_S + 1e-9:
                        self.hold_ok = self.hold_ok and v["ok"] and not CLAP.reopened
                    if f["done"] and since >= HOLD_S and self.hold_ok and not CLAP.reopened:
                        self.full = True
                    if self.full:
                        level, self.reason = 1.0, f"clean clap ({f['frames']} frames, slate moved {mv * 1000:.1f} mm), held 1 s"
                else:
                    self.reason = f"slate moved {mv * 1000:.1f} mm"
            else:
                self.reason = f"slow: {f['frames']} frames"
            self.score = level
        if DEBUG and (self.steps % 5 == 0 or f is not None and self.steps % 1 == 0 and CLAP.k - f["frame"] < 400):
            print(f"[slate] step {self.steps}: stick {s.deg('stick'):5.1f} deg, facing {v['facing']:5.1f}, in frame {v['in_frame']}, "
                  f"slate {np.round(np.asarray(s.pos) - [0, 0, TOP], 3)}, tilt {s.tilt_deg():.1f}, score {self.score} ({self.reason})", flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the scripted demonstration (open loop from the published starting layout) ----------------------------------------------------------------


MODES = ("full", "none", "onearm", "letgo", "slow", "snap_pinched")
UP = -2.0  # the demo holds the board 2 deg past upright, toward the phone
THETA = 25.0  # the demo opens the stick this far
PAD = 0.013  # the closed fingertips are this far past the TCP along the fingers


def _ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def slate_world(p_local, lean_deg):
    """A point of the slate in the world with the board leaning back `lean_deg` (negative: toward the phone) about its bottom edge."""
    return np.array([SLATE_X, SLATE_Y, TOP]) + _ry(-lean_deg) @ np.asarray(p_local, float)


def stick_point(r, h, theta_deg):
    """A point on the upper stick (slate frame): `r` along it from the hinge, `h` up from its bottom edge, the stick open `theta_deg`."""
    t = math.radians(theta_deg)
    return np.array(HINGE) + r * np.array([0.0, math.cos(t), math.sin(t)]) + h * np.array([0.0, -math.sin(t), math.cos(t)])


def _grip(close_dir, approach):
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(close_dir, float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    return np.c_[x, np.cross(z, x), z]


def demo_plan(mode: str = "full"):
    """Open loop from the published layout. Right hand: pinches the board's right edge across its 5 mm (40 mm in, 130 mm up), stands the
    board up about its bottom edge to 2 deg past upright and holds it there on the table. Left hand: pinches the upper stick's last 2 cm
    end-on (fingers along the stick, closing across its 8 mm), opens it 25 deg along its arc and lets go (the hinge holds it), backs off,
    closes its fingers and strikes the stick's top edge shut with the fingertips, then lifts away.
    Red-team modes (TASK_DEMO): none; onearm (left hand only, on the leaning board); letgo (the board stood up and let go before the clap);
    slow (the stick closed again at an ordinary pace, still pinched); snap_pinched (snapped shut while still pinched)."""
    if mode not in MODES:
        raise ValueError(f"TASK_DEMO must be one of {MODES}")
    import demokit as d
    from yam_arena.demo import Waypoint

    OPEN, SHUT = -0.04695, 0.0
    AJAR = -0.014  # fingers 3.7 cm apart: clear of the box behind the board

    def wp(t, p, finger, label, rot):
        return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, d._as_grip(rot))

    down = d.down(0).rot
    park_l = np.array([0.20, 0.27, TOP + 0.20])
    park_r = np.array([0.20, -0.27, TOP + 0.20])
    if mode == "none":
        return {"left": [wp(3.0, park_l, OPEN, "stay away", down)], "right": [wp(3.0, park_r, OPEN, "stay away", down)]}

    # -- right hand
    g_local = np.array([0.0, -W / 2 + 0.040, 0.130])

    def hold_rot(lean):
        Rl = _ry(-lean)
        d = math.radians(40.0)  # fingers pointing along the board (+y) and 40 deg down it, in its plane
        return _grip(Rl @ [1.0, 0.0, 0.0], Rl @ [0.0, math.cos(d), -math.sin(d)])

    a_r = hold_rot(LEAN)[:, 2]
    right = [wp(2.5, slate_world(g_local, LEAN) - 0.08 * a_r + [0, 0, 0.03], AJAR, "beside the board's right edge", hold_rot(LEAN)),
             wp(1.2, slate_world(g_local, LEAN) - 0.08 * a_r, AJAR, "in line with the grip point", hold_rot(LEAN)),
             wp(1.2, slate_world(g_local, LEAN), AJAR, "fingers either side of the board", hold_rot(LEAN)),
             wp(0.8, slate_world(g_local, LEAN), SHUT, "pinch the board", hold_rot(LEAN))]
    for i in range(1, 9):
        lean = LEAN + (UP - LEAN) * i / 8
        right.append(wp(0.45, slate_world(g_local - [0, 0, 0.0015], lean), SHUT, "stand the board up", hold_rot(lean)))
    t_up = sum(w.duration for w in right)

    # -- left hand
    lean_now = LEAN if mode == "onearm" else UP
    Rl = _ry(-lean_now)
    n_back = Rl @ np.array([-1.0, 0.0, 0.0])  # the fingers' closing axis: across the stick, x toward the arms

    def end_on_l(theta, offset=10.0):
        """Pinching the stick's end: fingers along it toward the hinge, `offset` deg further down than the stick (the arm's reach)."""
        t = math.radians(theta + offset)
        return _grip(n_back, Rl @ np.array([0.0, -math.cos(t), -math.sin(t)]))

    def pinch_at(theta):
        return slate_world(stick_point(R_TIP - 0.035, STICK_H / 2 + 0.013, theta), lean_now)  # fingers on the last 43 mm, upper 18 mm only

    e0 = Rl @ np.array([0.0, 1.0, 0.0])
    left = [wp(t_up if mode != "onearm" else 2.0, park_l, OPEN, "wait for the board", down),
            wp(2.0, pinch_at(0.0) + 0.06 * e0 + [0, 0, 0.015], AJAR, "beyond the stick's free end", end_on_l(0.0, 20.0)),
            wp(1.0, pinch_at(0.0) + 0.06 * e0, AJAR, "level with the stick's end", end_on_l(0.0, 20.0)),
            wp(1.2, pinch_at(0.0), AJAR, "fingers either side of the stick's end", end_on_l(0.0)),
            wp(0.6, pinch_at(0.0), SHUT, "pinch the stick's end", end_on_l(0.0))]
    for i in range(1, 7):
        th = THETA * i / 6
        left.append(wp(0.3, pinch_at(th), SHUT, "open the stick", end_on_l(th)))
    if mode == "slow":  # the careless way: close it again at an ordinary pace, still pinched
        for i in range(1, 7):
            th = THETA * (1 - i / 6)
            left.append(wp(0.25, pinch_at(th) - [0, 0, 0.004 if i == 6 else 0.0], SHUT, "close the stick", end_on_l(th)))
        left.append(wp(0.6, left[-1].pos, SHUT, "stay", end_on_l(0.0)))
    elif mode == "snap_pinched":  # snap it shut while still pinching its end
        left.append(wp(0.8, pinch_at(THETA), SHUT, "pause", end_on_l(THETA)))
        left.append(wp(0.05, slate_world(stick_point(R_TIP - 0.008, STICK_H / 2 - 0.03, -10.0), lean_now), SHUT, "snap shut", end_on_l(THETA)))
        left.append(wp(0.6, left[-1].pos, SHUT, "stay", end_on_l(THETA)))
    else:
        # the strike: the closed fingers' lower side (18 mm below the TCP, in the board's plane) chops down on the stick's top edge
        rot = end_on_l(THETA)
        srot = end_on_l(THETA, 0.0)  # fingers along the open stick, above it
        below = srot[:, 1]  # the fingers' local y: down, in the board's plane
        r_tcp = 0.110  # the chop lands 10 to 14 cm out from the hinge: a given hand speed turns the stick faster there than at its end
        wT = Rl @ np.array([0.0, -math.sin(math.radians(THETA)), math.cos(math.radians(THETA))])
        strike_from = slate_world(stick_point(r_tcp, STICK_H, THETA), lean_now) - 0.018 * below + 0.050 * wT
        # the strike itself is an elbow swing: the same joints as at `strike_from` but the elbow (joint 3, the arm's strong shoulder group)
        # turned until the fingers' lower side is 3 cm below where the shut stick's top edge is, so the soft wrist joints barely move
        from yam_arena import ik, station
        from yam_arena.constants import READY
        q_from = ik.solve(strike_from, ik.matrix_to_quat_xyzw(srot), np.array(READY, float), base_pos=station.BASES["left"], iters=300)[0]
        z_goal = slate_world(stick_point(r_tcp, STICK_H - 0.060, 0.0), lean_now)[2]
        strike_to, strike_rot = strike_from, srot
        for dq in np.radians(np.arange(1.0, 80.0, 1.0)):
            q = q_from.copy()
            q[2] -= dq
            p, r = ik.tcp_pose(q, station.BASES["left"])
            strike_to, strike_rot = p, r
            if (p + 0.018 * r[:, 1])[2] <= z_goal:
                break
        rest_on = slate_world(stick_point(r_tcp, STICK_H - 0.002, 0.0), lean_now) - 0.018 * below
        left += [wp(0.6, pinch_at(THETA), AJAR, "let go: the hinge holds it open", rot),
                 wp(0.8, pinch_at(THETA) + 0.025 * wT, AJAR, "fingers up off the stick", rot),
                 wp(0.8, strike_from + [0, 0, 0.04], AJAR, "over the stick, toward the hinge", srot),
                 wp(0.5, strike_from + [0, 0, 0.04], SHUT, "close the fingers", srot),
                 wp(0.6, strike_from, SHUT, "fingers just over the stick's top edge", srot),
                 wp(0.5, strike_from, SHUT, "steady", srot),
                 wp(0.05, strike_to, SHUT, "strike it shut: an elbow swing", strike_rot),
                 wp(0.15, rest_on, SHUT, "rest on the shut stick", srot),
                 wp(0.5, rest_on, SHUT, "stay on it", srot)]
    left += [wp(1.0, np.array(left[-1].pos) + [0.0, 0.03, 0.06], SHUT, "hand up off the stick", left[-1].yaw.rot),
             wp(2.0, park_l, SHUT, "the left hand away", down)]

    if mode == "onearm":  # the left hand works the stick on the slate where it leans; the right stays away
        return {"left": left, "right": [wp(3.0, park_r, OPEN, "stays away", down)]}
    t_left = sum(w.duration for w in left)
    if mode == "letgo":  # stand the board up, let it go, then clap
        right += [wp(0.6, slate_world(g_local, UP), AJAR, "let go of the board", hold_rot(UP)),
                  wp(1.0, slate_world(g_local + [0, -0.06, 0], UP), AJAR, "hand away", hold_rot(UP))]
    right.append(wp(max(1.0, t_left - sum(w.duration for w in right) + 3.0), right[-1].pos, right[-1].finger, "hold the board", right[-1].yaw.rot))
    return {"left": left, "right": right}


def demo():
    return demo_plan(os.environ.get("TASK_DEMO", "full"))
