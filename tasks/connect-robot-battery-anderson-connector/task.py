"""Connect a mobile robot's battery: mate the battery lead's Anderson SB50 to the robot's SB50.

On the station table: on the right, a 12 V sealed lead-acid battery (fixed, as if strapped in its bay). Its lead (a red
and black 6 AWG pair, 30 cm, modelled as one 15 mm round lead with a stiff formed shape) leaves the battery's top,
bends down its front face and lies along the table, ending in a red Anderson SB50 housing (48.0 x 35.1 x 19.1 mm,
6331G1 with two 5900 contacts) that lies upside down. On the left, the robot's chassis (an aluminium tube frame) carries
the robot side of the harness: an identical lead and SB50, lying right side up. Both housings point toward the arms.

The SB50 is genderless: two housings mate face to face with one turned over, and their front 14.7 mm interleave (the
mated pair is 81.3 mm long). The flat contacts resist being pushed together with 10 lbf (44 N, the datasheet's average
for low-detent contacts) until they pass their detent at 10 mm of engagement; then they draw the housings home and hold
them with the same 44 N. A housing on its lead can't be pushed into another lying loose on its own lead: each must be
held, turned to face the other, lined up within a couple of millimetres and pushed home, and the leads must not be
dragged taut (pulling a battery by its cables damages the lugs and the battery's tabs).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
Closed-loop demo:  python demokit.py task.py OUT   (modes: full, none, onearm, careless)
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the leads below are simulated)
import mechkit as m  # noqa: E402  (on_step / on_reset hooks, push, fingertips)
import numpy as np  # noqa: E402
from cablekit import Plug, cable, formed_world  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Connect the robot's battery: plug the battery lead's connector into the robot's power connector."

# -- the connector: Anderson SB50 (datasheet DS-SB50) --------------------------------------------------------------
HL, HW, HH = 0.0480, 0.0351, 0.0191  # housing length (along the lead), width, height
TONGUE = 0.0147  # interleaved front length: two 48.0 mm housings mate to 81.3 mm
GAP = 0.0006  # clearance between the two interleaved halves
HOUSING_MASS = 0.035  # kg: housing and two crimped contacts
MATE_N = 44.0  # N: average connect / disconnect force, low-detent contacts (10 lbf)
DETENT = 0.010  # m of engagement at which the contacts pass their detent
ALIGN = 0.002  # m: the housings' axes must be this close (side to side and up and down) for the contacts to meet
ANG = 6.0  # degrees: and this square to each other
RED = (0.78, 0.10, 0.10)

# -- the leads -----------------------------------------------------------------------------------------------------
LEAD_LEN, LEAD_SEGS, LEAD_R = 0.30, 15, 0.0055
LEAD_MASS = 0.045  # kg: two 10 AWG copper conductors with insulation, 30 cm
LEAD_K = 0.3  # N m/rad per 2 cm joint: the 10 AWG pair keeps the shape it was formed to
LEAD_COLOR = (0.06, 0.06, 0.06)
D30 = math.radians(30.0)
VERT = 5  # segments running straight down
# the formed shape: 1 segment level off the anchor, down through 90 degrees, VERT segments down, back to level, the rest level
TURN = 3  # joints at the end of the level run that turn the housing 90 degrees toward the other side


def bends(side: float) -> tuple:
    """side +1 for the robot's lead (at +y, turning toward -y), -1 for the battery's."""
    level = LEAD_SEGS - 1 - 6 - VERT - TURN
    return (((0.0, D30),) * 3 + ((0.0, 0.0),) * VERT + ((0.0, -D30),) * 3 + ((0.0, 0.0),) * level
            + ((side * D30, 0.0),) * TURN)
SEG = LEAD_LEN / LEAD_SEGS


def _drop() -> float:
    """How far the formed lead's last segment centre sits below its first."""
    probe = cable("probe", start=(0.0, 0.0, 0.0), yaw=math.pi, length=LEAD_LEN, radius=LEAD_R, segment=SEG,
                  bends=bends(1.0))
    w = formed_world(probe)
    return float(w[0][0][2] - w[-1][0][2])


HOUSING_Z = TOP + HH / 2 + 0.0005  # the housing's centre, lying on the table
ANCHOR_Z = HOUSING_Z + _drop()  # where the lead leaves the battery / the robot's frame
ANCHOR_X = 0.485  # the robot lead's start (its first 2 cm lie on the robot's crossbar)
BAT_ANCHOR_X = 0.455  # the battery lead's start (its first 2 cm lie on the battery's top)
ANCHOR_Y = 0.16  # robot side at +y, battery side at -y

ROBOT_LEAD = cable("robot_lead", start=(ANCHOR_X, ANCHOR_Y, ANCHOR_Z), yaw=math.pi, length=LEAD_LEN, radius=LEAD_R,
                   segment=SEG, bends=bends(1.0), anchor_start=True, cable_mass=LEAD_MASS, bend_stiffness=LEAD_K,
                   bend_damping=0.02, armature=2e-4, color=LEAD_COLOR, friction=0.8,
                   end_plug=Plug(size=(HL, HW, HH), mass=HOUSING_MASS, color=RED, tongue=TONGUE, gap=GAP))
BATTERY_LEAD = cable("battery_lead", start=(BAT_ANCHOR_X, -ANCHOR_Y, ANCHOR_Z), yaw=math.pi, length=LEAD_LEN, radius=LEAD_R,
                     segment=SEG, bends=bends(-1.0), anchor_start=True, cable_mass=LEAD_MASS, bend_stiffness=LEAD_K,
                     bend_damping=0.02, armature=2e-4, color=LEAD_COLOR, friction=0.8,
                     end_plug=Plug(size=(HL, HW, HH), mass=HOUSING_MASS, color=RED, tongue=TONGUE, gap=GAP, flip=True))
LEADS = (ROBOT_LEAD, BATTERY_LEAD)

# -- the battery and the robot's frame -----------------------------------------------------------------------------
BAT_H = 0.173
BATTERY = SimReady("battery", pos=(BAT_ANCHOR_X + 0.0635, -0.20, TOP), size=(0.163, 0.239, BAT_H), mass=None,
                   package="electricalSystem_battery_c01", usd="sm_electricalSystem_battery_c01_01.usd")
AL = (0.70, 0.71, 0.73)
T1 = 0.0254  # 1 x 1 in aluminium tube
CROSS_Z = ANCHOR_Z - LEAD_R - T1 / 2  # the crossbar the robot's lead is clamped on
FRAME = [
    Box("frame_rail", pos=(ANCHOR_X + 0.002, 0.20, TOP + T1 / 2), size=(T1, 0.26, T1), color=AL),
    Box("frame_post_a", pos=(ANCHOR_X + 0.002, 0.09, (TOP + T1 + CROSS_Z - T1 / 2) / 2),
        size=(T1, T1, CROSS_Z - T1 / 2 - TOP - T1), color=AL),
    Box("frame_post_b", pos=(ANCHOR_X + 0.002, 0.31, (TOP + T1 + CROSS_Z - T1 / 2) / 2),
        size=(T1, T1, CROSS_Z - T1 / 2 - TOP - T1), color=AL),
    Box("frame_crossbar", pos=(ANCHOR_X + 0.002, 0.20, CROSS_Z), size=(T1, 0.26, T1), color=AL),
]

# -- distractors (scoring ignores them) ----------------------------------------------------------------------------
DISTRACTORS = [
    SimReady("screwdriver", pos=(0.62, 0.0, TOP), size=(0.188, 0.034, 0.032), mass=0.08, yaw=math.pi / 2,
             package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd"),
    SimReady("tape", pos=(0.66, -0.36, TOP), size=(0.055, 0.019, 0.055), mass=0.05, yaw=0.3,
             package="misc_tape_refill_a01", usd="sm_misc_tape_refill_a01_01.usd"),
    SimReady("wrench", pos=(0.66, 0.36, TOP), size=(0.26, 0.046, 0.017), mass=0.12, yaw=math.pi / 2 - 0.2,
             package="handTool_wrench_combination_a01", usd="sm_handTool_wrench_combination_a01_01.usd"),
]

OBJECTS = [ROBOT_LEAD, BATTERY_LEAD, BATTERY, *FRAME, *DISTRACTORS]


# -- reading the housings ------------------------------------------------------------------------------------------
def housing(env, lead: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Centre (station frame), rotation (columns: out of the front, across, up) and velocity of a lead's housing."""
    base = env.unwrapped
    art = base.scene[lead]
    i = list(art.body_names).index("end_plug")
    origin = m._np(base.scene.env_origins[0])
    pose = m._np(art.data.body_link_pose_w)[0, i]
    vel = m._np(art.data.body_link_vel_w)[0, i]
    return pose[:3] - origin, m.quat_matrix(pose[3:7]), vel[:3]


def anchor_point(spec) -> np.ndarray:
    """Where the lead leaves the battery / the robot's frame: the far end of its fixed first segment."""
    return np.asarray(spec.start, float)


def mating(pa, ra, pb, rb) -> dict:
    """The robot housing A and the battery housing B, seen in A's frame."""
    d = ra.T @ (pb - pa)
    facing = float(-(ra[:, 0] @ rb[:, 0]))  # 1 when they face each other
    flipped = float(-(ra[:, 2] @ rb[:, 2]))  # 1 when B is the other way up
    square = facing >= math.cos(math.radians(ANG)) and flipped >= math.cos(math.radians(ANG))
    aligned = bool(square and abs(d[1]) <= ALIGN and abs(d[2]) <= ALIGN)
    return {"e": float(HL - d[0]), "lat": float(d[1]), "vert": float(d[2]), "aligned": aligned, "square": square,
            "facing": facing, "flipped": flipped}


# -- the contacts and the scorer -----------------------------------------------------------------------------------
HOLD_STEPS = 20  # 1 s at 20 Hz
SEATED = 0.012  # m of engagement: mated length at most 84.0 mm
STARTED = 0.002  # m: the halves have begun to interleave
CLEAR = 0.04  # m: every fingertip point at least this far from both housings' centres... measured to their boxes
STILL = 0.02  # m/s
TAUT = 0.97  # a lead pulled straight: its ends this fraction of its length apart


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.latched = False
        self.taut = False
        self.hold = 0
        self.score = 0.0
        self.full = False
        self.info = {}

    def _contacts(self, env, mt, ra):
        """The SB50's flat contacts: 44 N against the push until the detent, then 44 N drawing the housings home."""
        e = mt["e"]
        if self.latched and (e < 0.003 or not mt["square"] or abs(mt["lat"]) > 2 * ALIGN or abs(mt["vert"]) > 2 * ALIGN):
            self.latched = False  # pulled apart (or wrenched sideways out of line)
        if not self.latched and mt["aligned"] and e >= DETENT:
            self.latched = True
        f = 0.0
        if self.latched:
            f = MATE_N  # A drawn toward B
        elif mt["aligned"] and STARTED < e < DETENT:
            f = -MATE_N * min(1.0, (e - STARTED) / 0.001)  # A pushed back from B
        ax = ra[:, 0]
        m.push(env, "robot_lead", tuple(f * ax), body="end_plug")
        m.push(env, "battery_lead", tuple(-f * ax), body="end_plug")

    def step(self, env):
        pa, ra, va = housing(env, "robot_lead")
        pb, rb, vb = housing(env, "battery_lead")
        mt = mating(pa, ra, pb, rb)
        self._contacts(env, mt, ra)
        for spec, p, r in ((ROBOT_LEAD, pa, ra), (BATTERY_LEAD, pb, rb)):
            rear = p - r[:, 0] * HL / 2
            if np.linalg.norm(rear - anchor_point(spec)) >= TAUT * LEAD_LEN:
                self.taut = True
        tips = m.fingertips(env)
        gap = min(_box_gap(tips, p, r) for p, r in ((pa, ra), (pb, rb)))
        clear = gap >= CLEAR
        still = max(np.linalg.norm(va), np.linalg.norm(vb)) <= STILL
        seated = self.latched and mt["aligned"] and mt["e"] >= SEATED
        ok = seated and clear and still and not self.taut
        self.hold = self.hold + 1 if ok else 0
        self.full = self.hold >= HOLD_STEPS
        if self.taut:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif seated:
            self.score = 0.5
        elif mt["aligned"] and mt["e"] >= STARTED:
            self.score = 0.25
        else:
            self.score = 0.0
        self.info = dict(mt, gap=float(gap), latched=self.latched, taut=self.taut, hold=self.hold)


def _box_gap(points: np.ndarray, p: np.ndarray, r: np.ndarray) -> float:
    """Smallest distance from any point to a housing's box."""
    local = (np.atleast_2d(points) - p) @ r
    half = np.array([HL / 2, HW / 2, HH / 2])
    out = np.maximum(np.abs(local) - half, 0.0)
    return float(np.min(np.linalg.norm(out, axis=1)))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)

SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"


def score(objects) -> float:
    return float(TRIAL.score)


def success(objects) -> bool:
    return bool(TRIAL.full)


# -- closed-loop demo (demokit) ------------------------------------------------------------------------------------
MATE_AT = np.array([0.32, 0.0, TOP + 0.06])  # where the two housings meet (the interface plane)


def demo_script(ctx):
    """Each arm takes its housing across its body, top-down, lifts it and turns it 90 degrees to face the other. They
    meet at MATE_AT, line up on what they see, and push together through the detent; then both let go and back off.
    Failure checks (--mode): none does nothing; onearm pushes the battery housing at the robot one, which lies where
    it is; careless pulls the battery housing to the robot side on its lead."""
    import demokit as d

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    for _ in range(4):
        yield
    pa, ra, _ = housing(env, "robot_lead")
    pb, rb, _ = housing(env, "battery_lead")
    print(f"[demo] start A {np.round(pa, 4)} B {np.round(pb, 4)} anchor z {ANCHOR_Z - TOP:.4f}", flush=True)
    if mode == "none":
        for _ in range(60):
            yield
        return
    grip_back = 0.008  # grip the housing's body, behind its interleaving front
    up = 0.006  # TCP above the housing's centre: pad tips 3 mm above its underside

    def grasp(arm, p, rot):
        g = p - rot[:, 0] * grip_back + [0.0, 0.0, up]
        h = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))  # fingers close across the housing: grip yaw = heading
        yaw = (h + 90.0) % 180.0 - 90.0  # in [-90, 90)
        if arm is r and yaw < -45.0:
            yaw += 180.0  # the right arm's wrist reaches -60..120 degrees here, the left's -120..60
        if arm is l and yaw > 45.0:
            yaw -= 180.0
        return g, yaw

    ga, ya = grasp(l, pa, ra)
    gb, yb = grasp(r, pb, rb)
    if mode == "careless":
        yield from r.move(gb + [0, 0, 0.08], grip=d.down(yb), finger=-0.03)
        yield from r.move(gb , t=1.0)
        yield from r.close(HW, force=40.0)
        yield from r.move(np.array([0.22, 0.20, TOP + 0.05]), t=3.0)
        yield from r.open()
        yield from r.move(np.array([0.2, -0.2, TOP + 0.2]), t=1.0)
        return
    if mode == "onearm":
        yield from r.move(gb + [0, 0, 0.08], grip=d.down(yb), finger=-0.03)
        yield from r.move(gb , t=1.0)
        yield from r.close(HW, force=40.0)
        yield from r.move(gb + [0, 0, 0.03], t=1.0)
        # turn it to face the robot housing where it lies, and push it in
        face = pa + ra[:, 0] * (HL - 0.0)  # B's centre when just touching, at A's front, turned to face A
        yield from r.move(face + ra[:, 0] * 0.03 + [0, 0, 0.03], grip=d.down(math.degrees(math.atan2(-ra[1, 0], -ra[0, 0])) - 90.0), t=2.0)
        yield from r.move(face + ra[:, 0] * 0.03, t=1.0)
        yield from r.move(pa + ra[:, 0] * (HL - TONGUE - 0.01), t=2.0)
        yield from r.wait(1.0)
        yield from r.open()
        yield from r.move(r.at() + [0, 0, 0.10], t=1.0)
        for _ in range(40):
            yield
        return
    # full: both hands take their housings, lift them 1 cm, turn them square to the line x = MATE_AT[0] and bring them
    # to 12 mm apart across MATE_AT; then they push together along that line, correcting side to side, through the detent
    yield from ctx.together(l.move(ga + [0, 0, 0.08], grip=d.down(ya), finger=-0.03),
                            r.move(gb + [0, 0, 0.08], grip=d.down(yb), finger=-0.03))
    yield from ctx.together(l.move(ga, t=1.0), r.move(gb, t=1.0))
    yield from ctx.together(l.close(HW, force=40.0), r.close(HW, force=40.0))
    yield from ctx.together(l.move(l.at() + [0, 0, 0.012], t=0.6), r.move(r.at() + [0, 0, 0.012], t=0.6))
    yield from _square_up(ctx, env, l, r, ya, yb, grip_back, up)
    pa, ra, _ = housing(env, "robot_lead")
    pb, rb, _ = housing(env, "battery_lead")
    print(f"[demo] faced: {mating(pa, ra, pb, rb)}", flush=True)
    yield from _push_home(ctx, env, l, r)
    for _ in range(10):
        yield
    pa, ra, _ = housing(env, "robot_lead")
    pb, rb, _ = housing(env, "battery_lead")
    print(f"[demo] pushed: {mating(pa, ra, pb, rb)}", flush=True)
    yield from ctx.together(l.open(), r.open())
    yield from ctx.together(l.move(l.at() + [0, 0, 0.10], t=1.0), r.move(r.at() + [0, 0, 0.10], t=1.0))
    yield from ctx.together(l.move(np.array([0.20, 0.25, TOP + 0.20]), t=1.0), r.move(np.array([0.20, -0.25, TOP + 0.20]), t=1.0))
    for _ in range(90):
        yield
    pa, ra, _ = housing(env, "robot_lead")
    pb, rb, _ = housing(env, "battery_lead")
    print(f"[demo] end: {mating(pa, ra, pb, rb)}", flush=True)


def _heading(rot) -> float:
    return math.degrees(math.atan2(rot[1, 0], rot[0, 0]))


def _square_up(ctx, env, l, r, ya, yb, grip_back, up):
    """Turn each housing to point along the mating line (A toward -y, B toward +y) and carry it to its side of MATE_AT,
    correcting for how it sits in the fingers, from what is seen."""
    import demokit as d

    ga, gb = ya, yb
    for it in range(3):
        pa, ra, _ = housing(env, "robot_lead")
        pb, rb, _ = housing(env, "battery_lead")
        ga += (-90.0 - _heading(ra) + 180.0) % 360.0 - 180.0
        gb += (90.0 - _heading(rb) + 180.0) % 360.0 - 180.0
        # where each hand must go for its housing's centre to land on the line, 6 mm + half a housing from MATE_AT
        ca = MATE_AT + np.array([0.0, HL / 2 + 0.006, 0.0])
        cb = MATE_AT - np.array([0.0, HL / 2 + 0.006, 0.0])
        ta = l.at() + (ca - pa) * np.array([1.0, 1.0, 0.0])
        tb = r.at() + (cb - pb) * np.array([1.0, 1.0, 0.0])
        yield from ctx.together(l.move(ta, grip=d.down(ga), t=1.5 if it == 0 else 0.6),
                                r.move(tb, grip=d.down(gb), t=1.5 if it == 0 else 0.6))
        for _ in range(4):
            yield
        pa, ra, _ = housing(env, "robot_lead")
        pb, rb, _ = housing(env, "battery_lead")
        print(f"[square] {it} A {np.round(pa, 4)} hd {_heading(ra):.1f}  B {np.round(pb, 4)} hd {_heading(rb):.1f}", flush=True)


def _push_home(ctx, env, l, r):
    """Push both housings together along the line, each hand correcting its own side-to-side error, past the detent."""
    for k in range(160):
        pa, ra, _ = housing(env, "robot_lead")
        pb, rb, _ = housing(env, "battery_lead")
        mt = mating(pa, ra, pb, rb)
        if mt["e"] >= TONGUE - 0.001 and mt["aligned"]:
            break
        fwd = 0.0 if (abs(pa[0] - MATE_AT[0]) > 0.0015 or abs(pb[0] - MATE_AT[0]) > 0.0015) and mt["e"] < STARTED else 0.0015
        sa = np.array([(MATE_AT[0] - pa[0]) * 0.6, -fwd, 0.0])
        sb = np.array([(MATE_AT[0] - pb[0]) * 0.6, fwd, 0.0])
        if k % 5 == 0:
            print(f"[push] {k} e {mt['e'] * 1000:.1f} lat {mt['lat'] * 1000:.1f} vert {mt['vert'] * 1000:.1f} "
                  f"facing {mt['facing']:.4f} flipped {mt['flipped']:.4f} A {np.round(pa, 4)} B {np.round(pb, 4)}", flush=True)
        yield from ctx.together(l.move(l.at() + sa, t=0.1), r.move(r.at() + sb, t=0.1))


