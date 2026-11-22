"""Swat a house fly that has landed on a wall, with the plastic fly swatter that hangs on a peg at the wall's end.

A 60 x 40 cm painted wall section stands across the far side of the table, facing the robot. A plastic fly swatter (Smart Savers HA063:
16.75 in (42.5 cm) long, a 5 x 4.7 in (12.7 x 11.9 cm) head) hangs by its handle's hole on a 5 cm steel peg in the wall's left end. A house
fly (7 mm) sits on the wall 14 cm above the table. It takes off if anything looms near it: once any point of the swatter or of either
gripper's fingers is within 15 cm of it at a 20 Hz check, it flies off at the second check after (0.1 s later) unless it has been swatted
by then. So the swatter has to be lined up about 18 cm off and then close the last 15 cm in 0.1 s (about 1.5 m/s), and its head has to land
flat on the wall over the fly: a slow push scares it off, and an edge-on or tilted hit misses it.

Model (declared in code):
- Wall ("wall", fixed): a 60 x 40 x 1.2 cm painted panel on a foot, its face at x = WALL_X, facing -x.
- Peg ("hook", fixed): a 5 cm long, 5 mm steel peg standing out of the wall's left end along -y, 44 cm above the table.
- Swatter ("swatter", a free articulation, 30 g): the handle (frame, 18 g) is a 1.5 cm x 4 mm strip with a ring end round a 10 mm hanging
  hole, 29.8 cm from the hole's centre to the neck; the head (12 g) is a 12.7 x 11.9 cm x 2 mm plate on a hinge at the neck (the bending of
  the plastic neck: a 0.15 N m/rad spring, +-40 degrees), so the head can lie flat on the wall while the handle meets it at an angle, as a
  real one does.
- Fly ("fly", fixed): a solid 7 x 4 x 3 mm body (with two wings, seen only) on the wall's face. Its box collider is what the swatter's head
  must touch: a swat needs the head's face within the 0.5 mm contact tolerance of the fly's body, not merely near the wall. It is hidden when
  it flies off, and its body then stays as a 3 mm bump that no longer counts (the real stand-in's pad pulls back into the wall).

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

SCENE = "station"
INSTRUCTION = "Kill the fly on the wall with the fly swatter."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]

# -- the wall ---------------------------------------------------------------------------------------------------------------------------
WALL_X = 0.420  # the wall's face
WALL_T = 0.012
WALL_Y0, WALL_Y1 = 0.030, 0.630  # its ends
WALL_H = 0.400
WALL_YC = 0.5 * (WALL_Y0 + WALL_Y1)

# -- the swatter (Honeywell HA063: 16.75 in long, head 5 x 4.7 in) -----------------------------------------------------------------------
LEN = 0.425
HEAD_W, HEAD_L, HEAD_T = 0.127, 0.119, 0.002  # across, along the handle, thick
NECK = LEN - HEAD_L - 0.008  # from the hole's centre to the neck: 0.298 m (the hole is 8 mm from the handle's end)
STRIP_W, STRIP_T = 0.015, 0.004
HOLE_R, RING_W = 0.005, 0.006  # the hanging hole's radius, and the ring's width round it
HANDLE_MASS, HEAD_MASS = 0.018, 0.012
NECK_K, NECK_LIM = 0.15, 40.0  # N m/rad, degrees

# -- the hook ---------------------------------------------------------------------------------------------------------------------------
HOOK_X = WALL_X + WALL_T / 2  # the hook is screwed into the wall's left end, standing out to -y
HOOK_Z = 0.440  # the hook's wire axis above the table
HOOK_R = 0.0025
HOOK_D = 0.050  # it stands this far out of the wall's end
SWAT_Y = WALL_Y0 - 0.030  # the hanging swatter's plane

# -- the fly ----------------------------------------------------------------------------------------------------------------------------
FLY_Y, FLY_Z = 0.490, 0.140  # where the fly sits on the wall (y, and its height above the table)
NEAR = 0.15  # anything this close scares it
ESCAPE_STEPS = 2  # it flies off this many checks (0.1 s) after the first check that finds something within NEAR
FLAT_DEG = 10.0  # a hit must be this flat to the wall
FLY_H, FLY_W, FLY_L = 0.003, 0.004, 0.007  # the fly's solid body: high (off the wall), wide (along y), long (up the wall)
TOUCH = 0.0005  # contact: the head's face this close to the fly's body (PhysX rests colliders at 0 gap; this is the measuring tolerance)
EDGE = 0.004  # the fly's centre at least this far inside the head's edge

# -- where things are -------------------------------------------------------------------------------------------------------------------
CLEAR = 0.05  # hands at least this far from the swatter for the hold
REST_Z = 0.04  # every swatter point this low above the table: lying on it


LOOKS = {
    "wall": m.Look(color=m.rgb(0.90, 0.88, 0.82), roughness=0.85),
    "trim": m.Look(color=m.rgb(0.75, 0.73, 0.68), roughness=0.7),
    "steel": m.Look(color=m.rgb(0.75, 0.75, 0.77), metallic=0.8, roughness=0.3),
    "swatter": m.Look(color=m.rgb(0.10, 0.45, 0.85), roughness=0.5),
    "mesh": m.Look(color=m.rgb(0.12, 0.50, 0.90), roughness=0.5, opacity=0.85),
    "fly": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.4),
    "wing": m.Look(color=m.rgb(0.70, 0.72, 0.75), roughness=0.2, opacity=0.5),
}

# -- wall and hook ----------------------------------------------------------------------------------------------------------------------
_wall_c = (WALL_X + WALL_T / 2, WALL_YC, WALL_H / 2)
_foot_c = (WALL_X + 0.06, WALL_YC, 0.009)
WALL = m.fixed("wall", (0.0, 0.0, TOP), [
    ("panel", m.box((WALL_T, WALL_Y1 - WALL_Y0, WALL_H), _wall_c), "wall"),
    ("foot", m.box((0.10, WALL_Y1 - WALL_Y0, 0.018), _foot_c), "trim"),
    ("cap", m.box((WALL_T + 0.004, WALL_Y1 - WALL_Y0 + 0.004, 0.012), (WALL_X + WALL_T / 2, WALL_YC, WALL_H + 0.006)), "trim"),
], [m.col_box("panel_c", (WALL_T, WALL_Y1 - WALL_Y0, WALL_H), _wall_c), m.col_box("foot_c", (0.10, WALL_Y1 - WALL_Y0, 0.018), _foot_c)],
    looks=LOOKS)

_hook_wire = m.cylinder(HOOK_R, HOOK_D, (HOOK_X, WALL_Y0 - HOOK_D / 2, HOOK_Z), axis="y")
_hook_plate = m.cylinder(0.006, 0.003, (HOOK_X, WALL_Y0 - 0.0015, HOOK_Z), axis="y")
HOOK = m.fixed("hook", (0.0, 0.0, TOP), [("wire", _hook_wire, "steel"), ("plate", _hook_plate, "steel")],
               [m.col_hull("wire_c", _hook_wire)], looks=LOOKS)

# -- the swatter: origin at the hole's centre, handle down -z, its flat in the y-z plane (thickness along x) --------------------------
_ring_o = HOLE_R + RING_W
_handle_parts = [
    ("strip", m.box((STRIP_T, STRIP_W, NECK - HOLE_R - RING_W * 0.5), (0, 0, -(HOLE_R + RING_W * 0.5 + NECK) / 2)), "swatter"),
    ("ring_l", m.box((STRIP_T, RING_W, 2 * _ring_o), (0, HOLE_R + RING_W / 2, 0)), "swatter"),
    ("ring_r", m.box((STRIP_T, RING_W, 2 * _ring_o), (0, -HOLE_R - RING_W / 2, 0)), "swatter"),
    ("ring_t", m.box((STRIP_T, 2 * HOLE_R, RING_W), (0, 0, HOLE_R + RING_W / 2)), "swatter"),
    ("ring_b", m.box((STRIP_T, 2 * HOLE_R, RING_W), (0, 0, -HOLE_R - RING_W / 2)), "swatter"),
]
_handle_cols = [
    m.col_box("strip_c", (STRIP_T, STRIP_W, NECK - HOLE_R - 0.002), (0, 0, -(HOLE_R + 0.002 + NECK) / 2)),
    m.col_box("ring_l_c", (STRIP_T, RING_W, 2 * _ring_o), (0, HOLE_R + RING_W / 2, 0)),
    m.col_box("ring_r_c", (STRIP_T, RING_W, 2 * _ring_o), (0, -HOLE_R - RING_W / 2, 0)),
    m.col_box("ring_t_c", (STRIP_T, 2 * HOLE_R, RING_W), (0, 0, HOLE_R + RING_W / 2)),
]
_head_c = (0.0, 0.0, -NECK - HEAD_L / 2)
_head_parts = [
    ("plate", m.box((HEAD_T, HEAD_W, HEAD_L), _head_c), "mesh"),
    ("rim_t", m.box((0.004, HEAD_W, 0.004), (0, 0, -NECK - 0.002)), "swatter"),
    ("rim_b", m.box((0.004, HEAD_W, 0.004), (0, 0, -NECK - HEAD_L + 0.002)), "swatter"),
    ("rim_l", m.box((0.004, 0.004, HEAD_L), (0, HEAD_W / 2 - 0.002, _head_c[2])), "swatter"),
    ("rim_r", m.box((0.004, 0.004, HEAD_L), (0, -HEAD_W / 2 + 0.002, _head_c[2])), "swatter"),
]
HEAD = m.hinge("head", "Y", (0.0, 0.0, -NECK), -NECK_LIM, NECK_LIM, _head_parts,
               [m.col_box("plate_c", (HEAD_T, HEAD_W, HEAD_L - 0.004), (0, 0, -NECK - 0.002 - HEAD_L / 2))],
               mass=HEAD_MASS, spring=NECK_K, damping=0.003, friction=0.0, armature=1e-5, gravity=True, com=_head_c,
               inertia=(HEAD_MASS * (HEAD_W ** 2 + HEAD_L ** 2) / 12, HEAD_MASS * HEAD_L ** 2 / 12, HEAD_MASS * HEAD_W ** 2 / 12))
_hang_z = HOOK_Z - (HOLE_R - HOOK_R) + 0.0005  # the ring's top bar just above the hook's wire
SWATTER = m.mechanism("swatter", (HOOK_X, SWAT_Y, TOP + _hang_z), [HEAD], LOOKS, yaw_deg=90.0, frame_parts=_handle_parts, frame_colliders=_handle_cols,
                      free_mass=HANDLE_MASS, friction=(0.9, 0.8), solver_iters=48,
                      frame_inertia=(HANDLE_MASS * NECK ** 2 / 12, HANDLE_MASS * NECK ** 2 / 12, 3e-6))

# -- the fly: a thin solid body on the wall that the head must touch ----------------------------------------------------------------
_body = m.Mesh(m.sphere(0.0035).v * np.array([0.43, 0.57, 1.0]), m.sphere(0.0035).f)  # 7 mm long, 4 mm wide, 3 mm off the wall
_fly_parts = [("body", _body.moved((-FLY_H / 2, 0.0, 0.0)), "fly"),
              ("wing_l", m.box((0.0003, 0.004, 0.005), (-FLY_H + 0.0002, 0.0022, -0.0015)), "wing"),
              ("wing_r", m.box((0.0003, 0.004, 0.005), (-FLY_H + 0.0002, -0.0022, -0.0015)), "wing")]
FLY = m.fixed("fly", (WALL_X, FLY_Y, TOP + FLY_Z), _fly_parts, [m.col_box("fly_c", (FLY_H, FLY_W, FLY_L), (-FLY_H / 2, 0.0, 0.0))], looks=LOOKS)
FLY_PRIMS = [f"{m.ENV_PATH}/fly/visual/{n}" for n in ("body", "wing_l", "wing_r")]
FLY_AT = np.array([WALL_X - FLY_H / 2, FLY_Y, TOP + FLY_Z])  # the fly's centre, 1.5 mm off the wall
# the four corners of the fly body's top face (the face towards the room), which the head's face must reach
FLY_TOP = np.array([[WALL_X - FLY_H, FLY_Y + sy * FLY_W / 2, TOP + FLY_Z + sz * FLY_L / 2] for sy in (-1, 1) for sz in (-1, 1)])

# -- distractors (NVIDIA SimReady, loose) -----------------------------------------------------------------------------------------------
MUG = SimReady("mug", pos=(0.30, -0.30, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.4)
BOOK = SimReady("book", pos=(0.20, -0.12, TOP), size=(0.229, 0.170, 0.010), mass=0.25, package="misc_notebook_a01", usd="sm_misc_notebook_a01_01.usd", yaw=1.75)
CANDLE = SimReady("candle", pos=(0.55, -0.22, TOP), size=(0.074, 0.074, 0.075), mass=0.35, package="decor_candle_a01", usd="sm_decor_candle_a01_01.usd")

OBJECTS = [WALL, HOOK, SWATTER, FLY, MUG, BOOK, CANDLE]

# -- geometry of the swatter in the world -----------------------------------------------------------------------------------------------
_gy, _gz = np.meshgrid(np.linspace(-HEAD_W / 2, HEAD_W / 2, 7), np.linspace(-NECK, -NECK - HEAD_L, 7))
HEAD_PTS = np.c_[np.zeros(_gy.size), _gy.ravel(), _gz.ravel()]  # the head's plane at joint zero, in the swatter's frame
HANDLE_PTS = np.array([[0.0, s * 0.0075, -z] for z in np.linspace(-0.01, NECK, 16) for s in (-1, 1)])


def head_frame(s):
    """The head's rotation and the neck point in the world: (R, neck)."""
    R = s.rotation()
    q = s.joint("head")
    c, sn = math.cos(q), math.sin(q)
    Ry = np.array([[c, 0, sn], [0, 1, 0], [-sn, 0, c]])
    return R @ Ry, s.to_world(np.array([[0.0, 0.0, -NECK]]))[0]


def swatter_points(s):
    """Points on the head and the handle, in the world."""
    Rh, neck = head_frame(s)
    head = neck + (HEAD_PTS - np.array([0.0, 0.0, -NECK])) @ Rh.T
    return np.vstack([head, s.to_world(HANDLE_PTS)])


def hit(s, fly=None):
    """Whether the head lies flat on the fly's body now, touching it: (swat, tilt in degrees, gap in m).

    The gap is the smallest distance, along the head's normal, from the head's wall-side face to a corner of the fly body's top face
    (negative if pressed into it). The fly's collider stops the head at a gap of about 0, so a swat needs real contact; a head held off
    the fly, even by 1 mm, is not a swat."""
    Rh, neck = head_frame(s)
    n = Rh[:, 0]  # the head's normal
    tilt = math.degrees(math.acos(min(1.0, abs(float(n[0])))))
    local = Rh.T @ (FLY_AT - neck)  # the fly's centre in the head's frame (origin at the neck)
    inside = abs(local[1]) <= HEAD_W / 2 - EDGE and -HEAD_L + EDGE <= local[2] <= -EDGE
    base = Rh.T @ (np.array([WALL_X, FLY_Y, TOP + FLY_Z]) - neck)  # the fly's foot on the wall: which side of the head the wall is
    side = 1.0 if base[0] >= 0 else -1.0
    corners = (FLY_TOP - neck) @ Rh  # the top face's corners in the head's frame
    gap = float(np.min(side * corners[:, 0])) - HEAD_T / 2
    return bool(inside and tilt <= FLAT_DEG and gap <= TOUCH), tilt, gap


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place the fly's fate (swatted, flown off) is cleared
        self.steps = 0
        self.swatted = False
        self.escaped = False
        self.first_near = None  # the step at which something first came within NEAR: the fly's escape timer starts
        self.late_flat = False  # a flat slap over the spot after it flew off
        self.tried = False  # what first came within NEAR was a swatter already taken off its peg (an eligible approach)
        self.min_gap = None  # the smallest head-to-fly gap seen while the fly was there (m), for the log
        self.hold = 0
        self.full = False
        self.score = 0.0
        self._last_score = 0.0
        self.log = []
        if env is not None:
            for p in FLY_PRIMS:
                m.set_visible(p, True)

    def _note(self, msg):
        self.log.append(msg)
        print(f"[swat] {msg}", flush=True)

    def step(self, env):
        self.steps += 1
        s = m.state(env, "swatter")
        fly = FLY_AT
        tips = m.fingertips(env)
        pts = swatter_points(s)
        off_hook = float(np.linalg.norm(s.pos - np.array([HOOK_X, SWAT_Y, TOP + _hang_z]))) > 0.05
        if not self.swatted and not self.escaped:
            d = float(min(np.linalg.norm(pts - fly, axis=1).min(), np.linalg.norm(tips - fly, axis=1).min()))
            ok, tilt, gap = hit(s)
            self.min_gap = gap if self.min_gap is None else min(self.min_gap, gap)
            if ok:  # the swat test runs first at each check
                self.swatted = True
                self._note(f"swatted at step {self.steps} (tilt {tilt:.1f} deg, gap {gap * 1000:.2f} mm)")
            else:
                if d < NEAR and self.first_near is None:  # the escape timer starts, and runs whatever happens next
                    self.first_near = self.steps
                    self.tried = off_hook
                    allp = np.vstack([pts, tips])
                    near_pt = allp[int(np.argmin(np.linalg.norm(allp - fly, axis=1)))] - fly
                    self._note(f"something within {NEAR * 100:.0f} cm at step {self.steps} (swatter off its peg: {off_hook}; nearest point "
                               f"{near_pt[0] * -100:.1f} cm out from the wall, {near_pt[1] * 100:+.1f} cm along it, {near_pt[2] * 100:+.1f} cm up)")
                # the fly takes off ESCAPE_STEPS checks after the timer started, even if everything has moved away again since
                if self.first_near is not None and self.steps - self.first_near >= ESCAPE_STEPS:
                    self.escaped = True
                    self._note(f"flew off at step {self.steps} (timer started at {self.first_near}; tilt {tilt:.1f}, gap {gap * 1000:.1f} mm; "
                               f"smallest gap before {self.min_gap * 1000:.2f} mm)")
                    for p in FLY_PRIMS:
                        m.set_visible(p, False)
        elif self.escaped and not self.late_flat:
            if hit(s)[0]:
                self.late_flat = True
                self._note(f"late flat slap over the empty spot at step {self.steps}")
        # the hold: swatter lying on the table, let go, everything still
        rest = bool(np.all(pts[:, 2] < TOP + REST_Z) and np.all(pts[:, 2] > TOP - 0.01))
        still = float(np.linalg.norm(s.lin_vel)) < 0.02 and float(np.linalg.norm(s.ang_vel)) < 0.2
        clear = float(np.min(np.linalg.norm(tips[:, None, :] - pts[None, :, :], axis=2))) > CLEAR
        ok = self.swatted and rest and still and clear
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.full:
            self.score = 1.0
        elif self.swatted:
            self.score = 0.75
        elif self.late_flat:
            self.score = 0.5
        elif self.escaped and self.tried:  # only once the fly has actually flown off, after an eligible approach
            self.score = 0.25
        else:
            self.score = 0.0
        if self.score != getattr(self, "_last_score", 0.0):
            self._note(f"score {self.score} at step {self.steps}")
            self._last_score = self.score


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- the demonstration ------------------------------------------------------------------------------------------------------------------
GRIP_BELOW = 0.035  # the left hand pinches the handle this far below the hole's centre
ALPHA = 20.0  # degrees: the handle meets the wall at this angle, the head flexed flat
COCK = -15.0  # and is drawn back to this angle before the strike
OVERDRIVE = 45.0  # degrees past the wall that the flick is commanded to


def demo_script(ctx):
    """Left arm only. Pinch the hanging swatter's handle across its thickness with the gripper level and pointing at the wall's end; slide
    it off the peg and draw it back; turn the gripper to point along -y and then down, which swings the handle out level to +y with the head
    upright; line it up with the fly, the handle drawn back 15 degrees from the wall and the head about 18 cm off it; flick the wrist round
    in one step (commanded 45 degrees past the wall and taken back two steps later, because the wrist joint lags), so that the head slaps
    flat on the wall over the fly; then roll the swatter onto its face, lay it on the table, let go and move clear.
    Modes: none does nothing; slow turns the head onto the fly over 2.4 s; close lines up with the head 12 degrees from the wall (within
    15 cm of the fly) before flicking; edge strikes with the head edge-on; poke jabs at the fly with the bare fingers; hold swats but
    keeps hold of the swatter; retreat lines up, swings the head briefly inside 15 cm and straight back out; nearmiss lines up and turns the
    head to about 16 cm from the fly, never inside 15 cm, then backs off; side lines up 25 cm further along the wall and slides the head
    slowly (5 cm/s) along the wall towards the fly from the side; hover flicks but stops the head short of the fly."""
    import demokit as d

    mode = ctx.mode
    if mode not in ("full", "none", "slow", "close", "edge", "poke", "hold", "retreat", "nearmiss", "side", "hover"):
        raise ValueError(mode)
    l = ctx.left
    for _ in range(10):
        yield
    if mode == "none":
        for _ in range(100):
            yield
        return
    if mode == "poke":  # the cheapest try: no swatter, a quick jab at the fly with the closed fingers
        fy, fz = ctx.pos("fly")[1], ctx.pos("fly")[2]
        yield from l.move(np.array([0.27, fy - 0.06, fz + 0.04]), grip=d.down(0.0), finger=0.0, t=2.5)
        yield from l.move(np.array([0.41, fy - 0.03, fz + 0.01]), grip=d.down(0.0), t=0.1)
        yield from l.wait(1.0)
        yield from l.move(np.array([0.27, fy - 0.06, fz + 0.06]), t=1.0)
        for _ in range(40):
            yield
        return
    sw = ctx.state("swatter")
    grip_pt = sw.to_world(np.array([[0.0, 0.0, -GRIP_BELOW]]))[0]
    lv = d.level(0.0)  # pointing +x, the fingers closing along y
    yield from l.move(grip_pt + [-0.06, 0.0, 0.0], grip=lv, finger=-0.012, t=2.0)
    yield from l.move(grip_pt, grip=lv, finger=-0.012, t=1.2)
    yield from l.close(width=STRIP_T, force=25.0)
    yield from l.move(l.at() + [0.0, 0.0, 0.002], t=0.5)
    yield from l.move(l.at() + [0.0, -0.065, 0.0], t=1.2)  # slide it off the peg's end
    yield from l.move(l.at() + [-0.10, 0.0, 0.03], t=1.0)
    yield from l.move(np.array([0.30, 0.05, TOP + 0.46]), grip=d.level(-90.0), t=1.6)  # the handle still hangs; now its flat faces x
    # where the fly is, as seen
    fy, fz = ctx.pos("fly")[1], ctx.pos("fly")[2]
    c = (NECK - GRIP_BELOW) * math.sin(math.radians(ALPHA)) + 0.003
    reach_y = (NECK - GRIP_BELOW) * math.cos(math.radians(ALPHA)) + HEAD_L / 2
    tcp = np.array([WALL_X - c, fy - reach_y, fz])

    def grip(a):
        return d.down(-90.0 - a)

    if mode == "edge":
        from yam_arena.urdf import axis_angle_matrix as _aam

        def grip(a):  # noqa: F811  the head edge-on: the gripper rolled 80 degrees about the handle
            u = np.array([math.sin(math.radians(a)), math.cos(math.radians(a)), 0.0])
            return d._as_grip(_aam(u, math.radians(80.0)) @ d.down(-90.0 - a).rot)
    # turn the gripper from level to pointing down, lowering it as the handle swings out level (the head clears the table)
    for th, h in ((75.0, 0.44), (60.0, 0.40), (45.0, 0.35), (30.0, 0.29), (15.0, 0.24)):
        yield from l.move(np.array([0.25, 0.05, TOP + h]), grip=d.tilted(th, -90.0), t=0.8)
    yield from l.move(np.array([0.25, 0.05, TOP + 0.19]), grip=d.down(-90.0), t=0.8)
    cock = ALPHA - 12.0 if mode == "close" else COCK
    if mode == "side":  # line up 25 cm further from the fly along the wall, the head nearly flat to it, then slide in slowly
        start = tcp + [0.0, -0.25, 0.0]
        yield from l.move(start + [-0.02, 0.0, 0.02], grip=grip(cock), t=1.5)
        yield from l.move(start, grip=grip(ALPHA - 4.0), t=1.5)
        yield from l.wait(0.6)
        yield from l.move(tcp, grip=grip(ALPHA - 4.0), t=5.0)  # 5 cm/s, side-on
        yield from l.move(tcp, grip=grip(ALPHA + 2.0), t=0.6)
        yield from l.wait(2.0)
        return
    yield from l.move(tcp + [-0.02, 0.0, 0.02], grip=grip(cock), t=1.5)
    yield from l.move(tcp, grip=grip(cock), t=1.0)
    yield from l.wait(0.6)
    if mode == "nearmiss":  # never inside 15 cm: turn the head in a little, then back off
        yield from l.move(tcp, grip=grip(cock + 2.0), t=0.5)
        yield from l.wait(1.0)
        yield from l.move(tcp + [-0.05, 0.0, 0.0], grip=grip(cock - 10.0), t=1.0)
        yield from l.wait(2.0)
        return
    if mode == "retreat":  # dip the head inside 15 cm for one step and straight back out
        yield from l.move(tcp, grip=grip(cock + 25.0), t=0.05)
        yield from l.move(tcp, grip=grip(cock - 15.0), t=0.15)
        yield from l.move(tcp + [-0.05, 0.0, 0.0], grip=grip(cock - 15.0), t=0.8)
        yield from l.wait(2.0)
        return
    if mode == "hover":  # the flick from 1 cm further back and without the overdrive: the head comes up short of the fly
        yield from l.move(tcp + [-0.010, 0.0, 0.0], grip=grip(ALPHA + 2.0), t=0.05)
        yield from l.wait(1.5)
        return
    if mode == "slow":
        yield from l.move(tcp, grip=grip(ALPHA + 3.0), t=2.4)
    else:
        # the wrist is soft (its joint lags a command with a time constant of about 0.15 s), so the flick is commanded well past the wall
        # and taken back two steps later: the head is still moving fast when the wall stops it
        yield from l.move(tcp, grip=grip(ALPHA + OVERDRIVE), t=0.05)
        yield from l.wait(0.10)
        yield from l.move(tcp, grip=grip(ALPHA + 2.0), t=0.15)
    yield from l.wait(0.5)
    yield from l.move(tcp, grip=grip(COCK), t=0.6)
    if mode == "hold":
        yield from l.wait(3.0)
        return
    # lay it down: roll the head over towards the robot, lower it to just above the table and let go, so it falls flat
    from yam_arena.urdf import axis_angle_matrix
    yield from l.move(tcp + [-0.08, 0.0, 0.02], grip=grip(0.0), t=1.0)
    roll = d._as_grip(axis_angle_matrix(np.array([0.0, -1.0, 0.0]), math.radians(60.0)) @ d.down(-90.0).rot)  # the head tipped towards the robot
    yield from l.move(np.array([0.30, tcp[1], TOP + 0.12]), grip=roll, t=1.5)
    yield from l.move(np.array([0.30, tcp[1], TOP + 0.07]), grip=roll, t=1.0)
    yield from l.open()
    yield from l.move(l.at() + [-0.03, 0.0, 0.10], t=1.0)
    yield from l.move(np.array([0.25, 0.20, TOP + 0.20]), grip=d.down(-90.0), t=1.5)
    for _ in range(40):
        yield
