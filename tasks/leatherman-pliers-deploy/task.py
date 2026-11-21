"""Open the pliers out of a closed Leatherman Wave+ multitool lying on a desk.

The tool (Leatherman Wave+, stainless: 4.0 x 1.5 x 0.7 in closed, 8.5 oz, 6.3 in with the pliers open, as measured by
OutdoorGearLab) lies closed, flat on the desk. Its two handles lie side by side; each is pivoted at one end on the tang
of one plier jaw. Closed, the plier head sits inside the two handles' channels. To open the pliers, each handle swings
half a turn about its own pivot, flat in the plane of the desk, the two in opposite directions, until the handles meet
again on the other side of the head, back to back, and the jaws stand out. A detent in each pivot holds a handle shut in
its first 12 degrees and pulls it home to its open stop in its last 40 degrees ("click into place"); in between a handle
turns freely against its pivot's friction. The tool is loose on the desk: a handle pushed round on its own drags the
whole tool along unless something holds the rest of it.

Model (declared in code):
- "tool": a free articulation (241 g). Frame = the plier head (71 g): its origin is midway between the two handle
  pivots, on the desk (the tool's underside); +x points from the pivots towards where the closed handles' free ends
  are; the jaws point to -x. Handle "ha" (+y side, 85 g) turns about z through (0, +9.5 mm), "hb" (-y side, 85 g)
  about z through (0, -9.5 mm). Joint zero is closed (both handles along -x). Opening is negative for ha and positive
  for hb; `opened()` returns both as positive degrees, 0 shut to 180 at the open stop.
- Friction: PhysX averages the two materials in a contact, and the simulator's desk and gripper pads share one material
  (0.5). The tool's underside is given 0.2/0.15, so steel on the desk comes out at 0.35/0.33; the handles' top and side
  faces 1.4/1.3, so a rubber pad on them comes out at 0.95/0.9 (README, Simplifications).

Score, read every step from the state at that step (nothing but a fall is remembered; see README):
  0 if the tool has fallen off the desk at any time in the trial;
  1.0 once both handles are at their open stops (the handles' free ends no more than 2 mm apart), the tool flat on the
      desk (underside within 3 mm of it, tilted 5 degrees or less), every fingertip at least 30 mm from the tool, and all
      of it still for 1 s (20 steps);
  0.5 if both handles have swung at least 90 degrees (the jaws are out of the channels);
  0.25 if one handle has swung at least 90 degrees;
  0 otherwise (the start).

    inspect-robots run ... -E task=tasks/leatherman-pliers-deploy/task.py (README)
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
INSTRUCTION = "Open the pliers out of the multitool, ready to use, and leave it on the desk."
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the tool (Leatherman Wave+), in its own frame -------------------------------------------------------------------------
L_CLOSED = 0.1016  # 4.0 in closed
W_CLOSED = 0.0381  # 1.5 in: the two handles side by side
T = 0.0178  # 0.7 in thick
HW = W_CLOSED / 2  # one handle's width
R_END = HW / 2  # the handle's rounded pivot end: its pivot is at the centre of that half-round
L_FREE = L_CLOSED - R_END  # pivot to the handle's free end: 92.1 mm
PIV_A = np.array([0.0, +R_END, T / 2])
PIV_B = np.array([0.0, -R_END, T / 2])
JAW_LEN = 0.068  # pivot line to the jaws' tip: open length = 92.1 + 68 = 160 mm (6.3 in)
HEAD_T = 0.0090  # the plier head's thickness
M_HEAD, M_HANDLE = 0.071, 0.085  # 71 + 2 x 85 = 241 g (8.5 oz)

# detents (estimates, see README): a spring towards the shut stop (preloaded 6 degrees into it) inside the first 12
# degrees, and towards the open stop (6 degrees past it) inside the last 40; free in between
K_DETENT = 0.10  # N m/rad
SHUT_ZONE, OPEN_ZONE, PRELOAD = 12.0, 140.0, 6.0  # degrees of opening
PIVOT_FRICTION = 0.008  # N m

# the score
OUT_DEG = 90.0  # a handle swung this far has cleared the jaws
GAP_FULL = 0.002  # the handles' free ends, open: back to back within 2 mm
FLAT_Z, FLAT_TILT = 0.003, 5.0
CLEAR = 0.030
FELL_Z = 0.05  # the tool's underside this far below the desk top: it has fallen off

X0, Y0 = 0.30, 0.0  # where the pivots' midpoint lies on the desk; the tool is turned half round (YAW0): pivots towards the robot
YAW0 = 180.0

LOOKS = {
    "steel": m.Look(color=m.rgb(0.72, 0.73, 0.75), metallic=0.9, roughness=0.32),
    "satin": m.Look(color=m.rgb(0.60, 0.61, 0.63), metallic=0.85, roughness=0.45),
    "blade": m.Look(color=m.rgb(0.80, 0.81, 0.83), metallic=0.95, roughness=0.18),
    "dark": m.Look(color=m.rgb(0.10, 0.10, 0.11), metallic=0.4, roughness=0.5),
    "jaw": m.Look(color=m.rgb(0.66, 0.67, 0.69), metallic=0.9, roughness=0.28),
    "tools": m.Look(color=m.rgb(0.45, 0.46, 0.48), metallic=0.8, roughness=0.4),
}


def _grip_col(text: str) -> str:
    """A collider of the handles' top and side faces: bound to the grip material defined in the frame (see the docstring)."""
    return text.replace("/PhysicsMaterial>", "/frame/GripMaterial>")


GRIP_MATERIAL = """
    def Material "GripMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {
        float physics:staticFriction = 1.4
        float physics:dynamicFriction = 1.3
        float physics:restitution = 0
    }"""


def _jaw_outline():
    """The plier head seen from above (both jaws shut): wide at the pivots, needle-nosed at the tip."""
    return np.array([[0.006, -0.0125], [0.006, 0.0125], [-0.012, 0.0125], [-0.030, 0.0095], [-0.052, 0.0055], [-JAW_LEN, 0.0020],
                     [-JAW_LEN, -0.0020], [-0.052, -0.0055], [-0.030, -0.0095], [-0.012, -0.0125]])


def _head():
    z0, z1 = T / 2 - HEAD_T / 2, T / 2 + HEAD_T / 2
    outline = _jaw_outline()
    jaws = m.prism(outline[::-1] if _cw(outline) else outline, z0, z1)
    parts = [
        ("jaws", jaws, "jaw"),
        ("seam", m.box((JAW_LEN - 0.016, 0.0008, 0.0004), (-(JAW_LEN + 0.016) / 2 - 0.000, 0.0, z1 + 0.0001)), "dark"),
        ("pivot", m.cylinder(0.0045, 0.0006, (-0.016, 0.0, z1 + 0.0003)), "satin"),
        ("cutter", m.box((0.010, 0.0035, 0.0004), (-0.022, 0.0, z1 + 0.0002)), "dark"),
        ("tang_a", m.cylinder(0.0048, HEAD_T + 0.001, (0.0, R_END, T / 2)), "satin"),
        ("tang_b", m.cylinder(0.0048, HEAD_T + 0.001, (0.0, -R_END, T / 2)), "satin"),
    ]
    cols = [m.col_hull("jaws_c", jaws), GRIP_MATERIAL]
    return parts, cols


def _cw(outline: np.ndarray) -> bool:
    x, y = outline[:, 0], outline[:, 1]
    return float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)) < 0


def _handle(name: str, side: float):
    """One handle at joint zero (shut): x from -L_FREE to the pivot, rounded about the pivot; y on its own side of the seam."""
    yc = side * R_END
    cx = -L_FREE / 2
    parts = [
        ("body", m.box((L_FREE, HW, T), (cx, yc, T / 2)), "steel"),
        ("end", m.cylinder(R_END, T, (0.0, yc, T / 2), segments=24), "steel"),
        # the channel face (it faces the other handle while shut, and outwards once open): the folded inside tools
        ("tools", m.box((L_FREE - 0.012, 0.0004, T - 0.005), (cx - 0.002, yc - side * (HW / 2 + 0.0001), T / 2)), "tools"),
        ("pivot", m.cylinder(0.0042, 0.0005, (0.0, yc, T + 0.00025)), "satin"),
    ]
    if side > 0:  # the outside-opening 420HC knife lies along the top of this handle, with its thumb hole
        parts += [("blade", m.box((0.070, 0.011, 0.0005), (-0.048, yc, T + 0.00025)), "blade"),
                  ("hole", m.cylinder(0.0042, 0.0006, (-0.020, yc, T + 0.0004)), "dark")]
    else:  # the serrated blade along the top of the other
        parts += [("blade", m.box((0.066, 0.011, 0.0005), (-0.050, yc, T + 0.00025)), "blade"),
                  ("teeth", m.box((0.060, 0.0012, 0.0006), (-0.052, yc + 0.0049, T + 0.0003)), "dark"),
                  ("hole", m.cylinder(0.0042, 0.0006, (-0.020, yc, T + 0.0004)), "dark")]
    e = 0.0005
    cols = [
        m.col_box(f"{name}_core", (L_FREE, HW - 2 * e, T - 2 * e), (cx, yc, (T - 2 * e) / 2)),
        m.col_cylinder(f"{name}_end", R_END - e, T - 2 * e, (0.0, yc, (T - 2 * e) / 2)),
        _grip_col(m.col_box(f"{name}_top", (L_FREE, HW, 0.002), (cx, yc, T - 0.001))),
        _grip_col(m.col_box(f"{name}_side_o", (L_FREE, e, T - 0.0015), (cx, yc + side * (HW / 2 - e / 2), 0.0015 + (T - 0.0015) / 2))),
        _grip_col(m.col_box(f"{name}_side_i", (L_FREE, e, T - 0.0015), (cx, yc - side * (HW / 2 - e / 2), 0.0015 + (T - 0.0015) / 2))),
    ]
    lower, upper = (-180.0, 0.0) if side > 0 else (0.0, 180.0)
    inertia = (M_HANDLE / 12 * (HW ** 2 + T ** 2), M_HANDLE / 12 * (L_CLOSED ** 2 + T ** 2), M_HANDLE / 12 * (L_CLOSED ** 2 + HW ** 2))
    return m.hinge(name, "Z", tuple(PIV_A if side > 0 else PIV_B), lower, upper, parts, cols, mass=M_HANDLE, spring=K_DETENT, damping=0.01,
                   friction=PIVOT_FRICTION, effort=2.0, armature=2e-5, gravity=True, com=(cx + 0.004, yc, T / 2), inertia=inertia)


_hp, _hc = _head()
TOOL = m.mechanism("tool", (X0, Y0, TOP + 0.0004), [_handle("ha", +1.0), _handle("hb", -1.0)], LOOKS, frame_parts=_hp, frame_colliders=_hc,
                   free_mass=M_HEAD, frame_inertia=(M_HEAD / 12 * (0.025 ** 2 + HEAD_T ** 2), M_HEAD / 12 * (JAW_LEN ** 2 + HEAD_T ** 2),
                                                    M_HEAD / 12 * (JAW_LEN ** 2 + 0.025 ** 2)),
                   friction=(0.2, 0.15), solver_iters=48, yaw_deg=YAW0)

# -- the desk's other things (SimReady distractors; the score ignores them) ------------------------------------------------
MUG = SimReady("mug", pos=(0.50, 0.21, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
               yaw=2.4)
GLASSES = SimReady("glasses", pos=(0.53, -0.20, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                   usd="sm_accessory_eyeglasses_a01_01.usd", yaw=1.3)
TAPE = SimReady("tape_dispenser", pos=(0.575, 0.0, TOP), size=(0.159, 0.064, 0.066), mass=0.35, package="misc_tape_dispenser_a01",
                usd="sm_misc_tape_dispenser_a01_01.usd", yaw=math.pi / 2)

OBJECTS = [TOOL, MUG, GLASSES, TAPE]


# -- reading the tool --------------------------------------------------------------------------------------------------------
def opened(st) -> tuple[float, float]:
    """Each handle's opening, in degrees: 0 shut, 180 at its open stop."""
    return -st.deg("ha"), st.deg("hb")


def end_gap(a_deg: float, b_deg: float) -> float:
    """How far apart the two handles' free ends are, across the tool, once both are swung round (m): 0 back to back."""
    a, b = math.radians(a_deg), math.radians(b_deg)
    ya = R_END + L_FREE * math.sin(a) + R_END * math.cos(a)  # the free-end corner of ha's back (its shut-time outer face)
    yb = -(R_END + L_FREE * math.sin(b) + R_END * math.cos(b))
    return ya - yb


def _box_dist(p: np.ndarray, lo, hi) -> np.ndarray:
    d = np.maximum(np.maximum(np.asarray(lo) - p, p - np.asarray(hi)), 0.0)
    return np.linalg.norm(d, axis=1)


def _rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tool_gap(st, points: np.ndarray) -> float:
    """The smallest distance from these world points to the tool (head and both handles, each where it is now), in m."""
    local = st.to_local(points)
    d = _box_dist(local, (-JAW_LEN, -0.0125, T / 2 - HEAD_T / 2), (0.006, 0.0125, T / 2 + HEAD_T / 2))
    for link, piv in (("ha", PIV_A), ("hb", PIV_B)):
        q = (local - piv) @ _rot_z(st.deg(link)) + piv  # into the handle's own (shut) pose
        d = np.minimum(d, _box_dist(q, (-L_FREE, piv[1] - HW / 2, 0.0), (R_END, piv[1] + HW / 2, T)))
    return float(d.min())


class Trial:
    """The detents in the pivots, and the score. `fell` (the tool off the desk) is recorded across steps and cleared only in
    `reset`."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.fell = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.prev = None
        self.n = 0
        self.last = {}
        if env is not None:
            self._detents(env)

    def _detents(self, env):
        st = m.state(env, "tool")
        for link, sign, open_deg in (("ha", -1.0, opened(st)[0]), ("hb", 1.0, opened(st)[1])):
            if open_deg < SHUT_ZONE:
                goal = -PRELOAD
            elif open_deg > OPEN_ZONE:
                goal = 180.0 + PRELOAD
            else:
                goal = open_deg  # free: the spring rests where the handle is
            m.set_target(env, "tool", link, sign * math.radians(goal))

    def step(self, env):
        self._detents(env)
        st = m.state(env, "tool")
        a, b = opened(st)
        under = float(st.pos[2]) - TOP  # the frame's origin is on the tool's underside
        if under < -FELL_Z:
            self.fell = True
        gap = end_gap(a, b)
        at_stops = a > 170.0 and b > 170.0 and gap <= GAP_FULL
        flat = abs(under) <= FLAT_Z and st.tilt_deg() <= FLAT_TILT
        clear = tool_gap(st, m.fingertips(env)) >= CLEAR
        moved = 0.0 if self.prev is None else max(abs(a - self.prev[0]), abs(b - self.prev[1]))
        self.prev = (a, b)
        still = float(np.linalg.norm(st.lin_vel)) < 0.01 and float(np.linalg.norm(st.ang_vel)) < 0.1 and moved < 0.5
        ok = at_stops and flat and clear and still and not self.fell
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS
        n_out = int(a >= OUT_DEG) + int(b >= OUT_DEG)
        if self.fell:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        else:
            self.score = {0: 0.0, 1: 0.25, 2: 0.5}[n_out]
        self.last = {"a": a, "b": b, "gap_mm": gap * 1000, "under_mm": under * 1000, "tilt": st.tilt_deg(), "clear": clear, "still": still,
                     "hold": self.hold, "score": self.score}
        self.n += 1
        if os.environ.get("TOOL_DEBUG") and self.n % int(os.environ.get("TOOL_DEBUG")) == 0:
            print("[tool]", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in self.last.items()}, flush=True)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full and not TRIAL.fell)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (closed loop, demokit) -----------------------------------------------------------------------------------
# The tool lies with its pivots towards the robot and the handles' free ends away from it; "ha" is the right-hand handle as the
# robot sees it, "hb" the left-hand one. The tool is loose, so one gripper pins it while the other swings a handle, then they swap.
# 1: the left gripper, pointing straight down with its fingers open 40 mm and in line with the tool, presses both fingertips on
#    the top of the left-hand handle, near its pivot: two points of contact, so the tool can neither slide nor turn. The right
#    gripper, shut and tilted 60 degrees out to the right (so that the two wrists stay apart over the narrow tool), presses on
#    the right-hand handle's top near its free end and drags it round its pivot, flat on the desk: out to the right and back
#    towards the robot. Its fingertip goal runs ahead along the handle's arc, but never more than LEAD degrees ahead of where
#    the handle is seen to be (closed loop on the handle's visible angle and the tool's pose).
# 2: past 145 degrees the right lifts off and the open detent pulls the handle home to its stop; the left lets go.
# 3: swap: the right gripper pins the open handle the same way; the left, tilted out to the left, drags the left-hand handle
#    round the other way. Past 145 degrees the right lets go first, then the left lifts off, and the second handle clicks home
#    back to back with the first. Both arms go clear.
R_DRAG = 0.088  # m from the pivot, along the handle's centre line: where the swinging gripper presses
LATERAL = float(os.environ.get("DEMO_LAT", "-0.006"))  # and this far across from the centre line (+: towards the handle's outer edge, shut): on the trailing half, so that the lead keeps the fingertip on the top
R_PIN = 0.030  # where the pinning gripper's two fingertips straddle, along the handle from its pivot
PIN_OPEN = 0.020  # each pinning finger's opening (m): the fingertips are about 49 mm apart
TILT = 60.0
PRESS = float(os.environ.get("DEMO_PRESS", "0.013"))  # how far below the handle's top the lowest fingertip is commanded: the arm's stiffness turns this into the press
LEAD = float(os.environ.get("DEMO_LEAD", "12.0"))
RATE = 30.0  # degrees/s
LIFT_AT = 145.0
TIP_PTS = np.array([[0.0, 0.0, 0.0122], [0.0, 0.0065, 0.0119], [0.0, -0.0065, 0.0119], [0.004, 0.0, 0.0122], [-0.004, 0.0, 0.0122]])  # the shut fingertips' underside, from the TCP in the grip's frame (measured off the arm's collision meshes)


def demo_script(ctx):
    import demokit as d

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "sameway", "early", "hold"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    if mode == "none":
        for _ in range(100):
            yield
        return
    def top_point(link: str, open_deg: float, reach: float, lateral: float) -> np.ndarray:
        """The point on `link`'s top face `reach` out along it from its pivot and `lateral` across it (tool frame, handle shut),
        with the handle opened `open_deg`, in the world."""
        st = ctx.state("tool")
        piv = PIV_A if link == "ha" else PIV_B
        sign = -1.0 if link == "ha" else 1.0
        out = _rot_z(sign * open_deg) @ np.array([-reach, lateral, 0.0])
        return np.asarray(st.pos, float) + st.rotation() @ (piv + out + np.array([0.0, 0.0, T / 2]))

    def tcp_for(point: np.ndarray, grip, depth: float) -> np.ndarray:
        """The TCP that puts the fingertips' lowest point `depth` below `point` (negative: above it)."""
        corners = TIP_PTS @ grip.rot.T
        low = corners[corners[:, 2] <= corners[:, 2].min() + 0.001].mean(axis=0)
        return point - low - np.array([0.0, 0.0, depth])

    def seen(link: str) -> float:
        return opened(ctx.state("tool"))[0 if link == "ha" else 1]

    def tool_yaw_deg() -> float:
        R = ctx.state("tool").rotation()
        return math.degrees(math.atan2(R[1, 0], R[0, 0]))

    lift_at = 120.0 if mode == "early" else LIFT_AT  # "early": lets go before the open detent can take the handle

    def pin(arm, link: str, open_deg: float):
        """Fingers open PIN_OPEN and in line with the handle, both fingertips pressed on its top R_PIN from its pivot."""
        along = tool_yaw_deg() + 180.0 + (-open_deg if link == "ha" else open_deg)  # the handle's direction, pivot to free end
        grip = d.down((along - 90.0 + 180.0) % 360.0 - 180.0)  # fingers closing along the handle
        p = top_point(link, open_deg, R_PIN, 0.0)
        yield from arm.move(tcp_for(p, grip, -0.04), grip=grip, finger=-PIN_OPEN, t=2.5, label=f"over {link} (pin)")
        yield from arm.move(tcp_for(top_point(link, open_deg, R_PIN, 0.0), grip, PRESS), t=1.2, label=f"pin {link}")

    def press(arm, grip, link: str, lateral: float):
        p = top_point(link, 0.0, R_DRAG, lateral)
        yield from arm.move(tcp_for(p, grip, -0.04), grip=grip, finger=FINGER_CLOSED, t=2.5, label=f"over {link}")
        yield from arm.move(tcp_for(top_point(link, 0.0, R_DRAG, lateral), grip, PRESS), t=1.2, label=f"press {link}")

    def swing(arm, grip, link: str, lateral: float, wrong_way: bool = False):
        """Drag `link` round to `lift_at` (closed loop), pressing R_DRAG out."""
        goal, best, stalled = 0.0, 0.0, 0
        for _ in range(int(16.0 * 20)):
            now = seen(link)
            if wrong_way:  # towards the other handle, against the shut stop, as far as the arm is commanded
                goal = min(goal + RATE / 20.0, 30.0)
                target, finished = -goal, goal >= 30.0
            else:
                goal = min(goal + RATE / 20.0, now + LEAD, lift_at)
                target, finished = goal, goal >= lift_at - 0.5 and now >= lift_at - 6.0
                stalled = 0 if now > best + 0.5 else stalled + 1
                best = max(best, now)
                if stalled >= 12:  # the handle has not moved for 0.6 s: the fingertip has slipped. Lift, put it back on the handle and press again
                    yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.02]), t=0.3, label=f"regrip {link}")
                    now = seen(link)
                    yield from arm.move(tcp_for(top_point(link, now, R_DRAG, lateral), grip, -0.01), t=0.5, label=f"regrip {link}")
                    yield from arm.move(tcp_for(top_point(link, seen(link), R_DRAG, lateral), grip, PRESS), t=0.4, label=f"regrip {link}")
                    goal, stalled = seen(link), 0
                    continue
            p = tcp_for(top_point(link, target, R_DRAG, lateral), grip, PRESS)
            yield from arm.follow(lambda p=p: p, t=1.0 / 20.0, speed=0.3, label=f"drag {link}")
            if finished:
                break

    def up(arm, h=0.05, t=0.8):
        yield from arm.move(arm.at() + np.array([0.0, 0.0, h]), t=t, label="lift off")

    g_right, g_left = d.tilted(TILT, 90.0), d.tilted(TILT, -90.0)
    one = mode == "onearm"
    # 1. the left pins the tool by the left-hand handle; the right comes down near the right-hand handle's free end
    gens = [press(r, g_right, "ha", LATERAL)]
    if not one:
        gens.append(pin(l, "hb", 0.0))
    yield from ctx.together(*gens)
    for _ in range(6):
        yield
    # 2. the right swings the right-hand handle round; then lifts off, and the left lets go
    yield from swing(r, g_right, "ha", LATERAL)
    yield from up(r)
    if not one:
        yield from up(l)
    for _ in range(10):
        yield
    # 3. swap: the right pins the open handle, the left swings the left-hand handle
    if one:  # the right arm does it all, with no pin
        yield from press(r, g_right, "hb", -LATERAL)
        yield from swing(r, g_right, "hb", -LATERAL)
        yield from up(r)
    else:
        yield from ctx.together(pin(r, "ha", 180.0), press(l, g_left, "hb", -LATERAL))
        for _ in range(6):
            yield
        yield from swing(l, g_left, "hb", -LATERAL, wrong_way=(mode == "sameway"))
        if mode == "hold":  # keeps pressing at the end: never lets go
            for _ in range(200):
                yield
            return
        yield from up(r, 0.04, 0.5)  # the pin first, so that the handle can click home past it
        yield from up(l)
    # 4. clear, and wait
    parks = {"left": np.array([0.20, 0.22, TOP + 0.16]), "right": np.array([0.20, -0.22, TOP + 0.16])}
    yield from ctx.together(l.move(parks["left"], grip=d.down(0.0), t=1.5, label="clear"), r.move(parks["right"], grip=d.down(0.0), t=1.5, label="clear"))
    for _ in range(120):
        yield


def demo():
    """Open-loop fallback for `yam_scripted` (README): the demo is `demo_script` (demokit), closed loop."""
    o = FINGER_OPEN
    return {"left": [Waypoint(2.0, (0.25, 0.20, TOP + 0.16), o, "wait")], "right": [Waypoint(2.0, (0.25, -0.20, TOP + 0.16), o, "wait")]}
