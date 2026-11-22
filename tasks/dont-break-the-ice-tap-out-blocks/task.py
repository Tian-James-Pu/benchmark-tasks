"""Don't Break the Ice (Hasbro C2093): tap out the six marked ice blocks with the game's mallet, and keep Phillip up.

The game stands on a table in front of the arms: a square blue ice-tray frame on four legs, holding 32 small plastic ice
blocks and one large block (2 x 2 cells) carrying Phillip the Penguin, packed six by six. The blocks have no ledge under
them: each is held only by friction against its neighbours and the frame. Six blocks carry a red sticker dot. The game's
plastic mallet lies on the table to the right. A plastic cup, a small plate and an apple are distractors.

The ice sheet is a mechanism (mechkit): each block slides on a vertical joint, held by a capped spring (static friction
against its neighbours: up to 4 N before it slips), and a per-step law in `Trial.step` couples neighbours:
- while its faces stick, a block drags each touching neighbour down to 0.8 of its own depth (the sheet sags together);
- a block driven more than 4 mm deeper than its deepest touching neighbour has slipped out of its faces and falls alone,
  as a sharp tap does: the neighbours have no time to follow;
- a block pressed 8 mm down while its neighbours follow (a slow push) breaks the sheet: it falls and takes every touching
  neighbour that is more than 5.5 mm down with it;
- a block left without support on two opposite sides (a neighbour or the frame on both its x sides, or on both its
  y sides) falls.

Credit needs the mallet: each step the scorer records which body last touched each block (the mallet's head, its handle,
a finger, a distractor). A marked block earns credit only if its hold let go within 5 control steps (0.25 s) of the
mallet head touching it, with nothing else (a finger, the handle) within 2 mm of it in between. The trial is void if a
distractor comes within 2 mm of the ice or Phillip, a finger presses into them, or a block drops out with a finger as its
last toucher.

    python demokit.py task.py OUT --mode full                                   # the demonstration, with video (GPU)
    python demokit.py task.py OUT --no-record --mode none,careless,fingers,offcentre   # the red-team modes
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
import simready  # noqa: E402,F401
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Using the game's mallet, knock out only the six ice blocks marked with red dots, without letting Phillip fall through."

# -- the game (metres; world axes; heights above the table top) -------------------------------------------------------
X0, Y0 = 0.27, -0.10  # the tray's centre
N = 6  # cells per side
P = 0.036  # cell pitch
BW = 0.0355  # block width (blocks are squeezed in, about 0.5 mm play)
BH = 0.035  # block height
LEG = 0.050  # underside of the ice above the table (the tray's legs)
RIM = 0.014  # frame rim width
Z_BOT = LEG  # blocks' underside, in the tray's frame
Z_TOP = LEG + BH  # blocks' top
FALLEN = -(LEG - 0.004)  # a block that fell through lies on the table under the tray (joint lower limit)
BLOCK_M = 0.004  # small block, kg
BIG_M = 0.016 + 0.010  # large block plus Phillip
PENGUIN_R, PENGUIN_H = 0.019, 0.062

# the ice law
HOLD_K = 2000.0  # N/m, the stuck faces' stiffness
HOLD_F = 4.0  # N, the most the faces hold before slipping
CARRY = 0.8  # a stuck neighbour is dragged to this fraction of a block's depth
SLIP = 0.004  # depth past the deepest touching neighbour at which a block slips out alone
SAG = 0.008  # depth at which a block breaks the sheet
TAKE = 0.0055  # neighbours deeper than this go with a block that breaks the sheet
OUT = 0.035  # a block is out once its top is below the tray's underside

# scoring
PENGUIN_DROP = 0.010  # Phillip's block sunk 1 cm: he fell through
CLEAR = 0.03  # hands and mallet head at least 3 cm from the ice for full credit
STILL = 0.005  # m/s
HOLD_STEPS = 20  # 1 s at 20 Hz
STRIKE = 0.002  # a body touches a block when a point of it (or its bounding box) is within 2 mm of the block's box
STRIKE_WINDOW = 5  # control steps (0.25 s): a block's hold must let go this soon after the mallet head's last touch
DISTRACTORS = ("cup", "plate", "apple")
SUCCESS_HOLD_S = 0.0  # the Trial counts its own hold


def cell_xy(r, c):
    """Centre of cell (r, c) in the tray's frame: r along +x (away from the arms), c along -y (c 0 on the left)."""
    return np.array([(r - 2.5) * P, (2.5 - c) * P])


BLOCKS = {}  # name -> list of cells
for r in range(N):
    for c in range(N):
        if r in (2, 3) and c in (2, 3):
            continue
        BLOCKS[f"b{r}{c}"] = [(r, c)]
BLOCKS["big"] = [(2, 2), (2, 3), (3, 2), (3, 3)]
NAMES = list(BLOCKS)
CELL = {cell: name for name, cells in BLOCKS.items() for cell in cells}
MARKED = ["b05", "b12", "b31", "b41", "b24", "b44"]


def _neighbours(name):
    out = set()
    for r, c in BLOCKS[name]:
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            other = CELL.get((r + dr, c + dc))
            if other is not None and other != name:
                out.add(other)
    return sorted(out)


NEIGHBOURS = {n: _neighbours(n) for n in NAMES}


def _sides(name):
    """For each of -x, +x, -y, +y: None if the frame is on that side, else the blocks touching that side."""
    cells = BLOCKS[name]
    out = {}
    for key, (dr, dc) in (("-x", (-1, 0)), ("+x", (1, 0)), ("+y", (0, -1)), ("-y", (0, 1))):
        touching = [(r + dr, c + dc) for r, c in cells if (r + dr, c + dc) not in cells]
        if any(not (0 <= rr < N and 0 <= cc < N) for rr, cc in touching):
            out[key] = None
        else:
            out[key] = sorted({CELL[t] for t in touching})
    return out


SIDES = {n: _sides(n) for n in NAMES}


def footprint(name):
    """(centre xy, half size xy) of a block in the tray's frame."""
    xy = np.mean([cell_xy(*c) for c in BLOCKS[name]], axis=0)
    span = 1 if name != "big" else 2
    half = np.array([(span - 1) * P + BW, (span - 1) * P + BW]) / 2
    return xy, half


# -- looks --------------------------------------------------------------------------------------------------------------
LOOKS = {
    "tray": m.Look(color=m.rgb(0.16, 0.45, 0.85), roughness=0.4),
    "ice": m.Look(color=m.rgb(0.86, 0.94, 0.99), roughness=0.25),
    "dot": m.Look(color=m.rgb(0.85, 0.08, 0.08), roughness=0.5),
    "black": m.Look(color=m.rgb(0.05, 0.06, 0.09), roughness=0.5),
    "white": m.Look(color=m.rgb(0.96, 0.96, 0.96), roughness=0.5),
    "orange": m.Look(color=m.rgb(0.98, 0.55, 0.08), roughness=0.5),
    "mallet": m.Look(color=m.rgb(0.20, 0.55, 0.95), roughness=0.45),
    "handle": m.Look(color=m.rgb(0.95, 0.95, 0.97), roughness=0.45),
}


def _frame():
    half_in = N * P / 2
    half_out = half_in + RIM
    zc = (Z_BOT + Z_TOP) / 2
    parts, cols = [], []
    for k, (sx, sy, cx, cy) in enumerate((
            (2 * half_out, RIM, 0.0, half_in + RIM / 2), (2 * half_out, RIM, 0.0, -half_in - RIM / 2),
            (RIM, 2 * half_in, half_in + RIM / 2, 0.0), (RIM, 2 * half_in, -half_in - RIM / 2, 0.0))):
        size, centre = (sx, sy, BH + 0.004), (cx, cy, zc + 0.002)
        parts.append((f"rim{k}", m.box(size, centre), "tray"))
        cols.append(m.col_box(f"rim{k}_c", size, centre))
    leg = half_out - 0.007
    for k, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        centre = (sx * leg, sy * leg, Z_BOT / 2)
        parts.append((f"leg{k}", m.cylinder(0.006, Z_BOT, centre), "tray"))
        cols.append(m.col_cylinder(f"leg{k}_c", 0.006, Z_BOT, centre))
    return parts, cols


def _penguin_parts(cx, cy, z0):
    body = m.lathe([(0.0, 0.0), (0.017, 0.002), (0.019, 0.016), (0.017, 0.032), (0.013, 0.042), (0.0, 0.044)], segments=24)
    head = m.sphere(0.0125, (0.0, 0.0, 0.050))
    belly = m.lathe([(0.0, 0.003), (0.012, 0.006), (0.012, 0.030), (0.0, 0.034)], segments=20).moved((-0.008, 0.0, 0.0))
    beak = m.cylinder(0.004, 0.012, (-0.016, 0.0, 0.050), axis="x")
    feet = m.box((0.016, 0.024, 0.004), (-0.010, 0.0, 0.002))
    return [("p_body", body.moved((cx, cy, z0)), "black"), ("p_head", head.moved((cx, cy, z0 + 0.004)), "black"),
            ("p_belly", belly.moved((cx, cy, z0)), "white"), ("p_beak", beak.moved((cx, cy, z0 + 0.004)), "orange"),
            ("p_feet", feet.moved((cx, cy, z0)), "orange")]


def _block_link(name):
    xy, half = footprint(name)
    size = (2 * half[0], 2 * half[1], BH)
    centre = (xy[0], xy[1], (Z_BOT + Z_TOP) / 2)
    parts = [(f"{name}_v", m.box(size, centre), "ice")]
    cols = [m.col_box(f"{name}_c", size, centre)]
    mass = BLOCK_M
    if name in MARKED:
        parts.append((f"{name}_dot", m.cylinder(0.006, 0.0006, (xy[0], xy[1], Z_TOP + 0.0003)), "dot"))
    if name == "big":
        mass = BIG_M
        parts += _penguin_parts(xy[0], xy[1], Z_TOP)
        cols.append(m.col_cylinder("penguin_c", PENGUIN_R, PENGUIN_H, (xy[0], xy[1], Z_TOP + PENGUIN_H / 2)))
    return m.slider(name, "Z", FALLEN, 0.0, parts, cols, mass=mass, spring=HOLD_K, effort=HOLD_F, damping=0.05, friction=0.0, gravity=True,
                    com=(xy[0], xy[1], (Z_BOT + Z_TOP) / 2))


_fp, _fc = _frame()
ICE = m.mechanism("ice", (X0, Y0, TOP), [_block_link(n) for n in NAMES], LOOKS, frame_parts=_fp, frame_colliders=_fc, solver_iters=16)

# -- the mallet: the game's plastic hammer. Its frame: origin at the head's centre, head along z, handle along -x.
HEAD_R, HEAD_L = 0.012, 0.050
HANDLE_W, HANDLE_T, HANDLE_L = 0.009, 0.014, 0.110  # a flat plastic handle: width (across the grip), depth, length
GRIP_X = -0.060  # where the hand takes the handle
MALLET_START = (0.30, -0.30)  # head centre on the table, handle pointing at the arms
_head = m.cylinder(HEAD_R, HEAD_L, (0.0, 0.0, 0.0))
_HANDLE_C = (-HEAD_R - HANDLE_L / 2 + 0.002, 0.0, 0.0)
_handle = m.box((HANDLE_L, HANDLE_W, HANDLE_T), _HANDLE_C)
MALLET = m.free("mallet", (MALLET_START[0], MALLET_START[1], TOP + HEAD_L / 2 + 0.002),
                [("head", _head, "mallet"), ("handle", _handle, "handle")],
                [m.col_cylinder("head_c", HEAD_R, HEAD_L, (0.0, 0.0, 0.0)), m.col_box("handle_c", (HANDLE_L, HANDLE_W, HANDLE_T), _HANDLE_C)],
                LOOKS, mass=0.020, friction=(0.9, 0.8))

OBJECTS = [
    ICE,
    MALLET,
    # Distractors (SimReady), which scoring ignores.
    SimReady("cup", pos=(0.33, 0.27, TOP + 0.001), size=(0.064, 0.064, 0.100), mass=0.03,
             package="dishware_plastic_cup_a01", usd="sm_dishware_plastic_cup_a01_01.usd"),
    SimReady("plate", pos=(0.19, 0.20, TOP + 0.001), size=(0.099, 0.099, 0.045), mass=0.05,
             package="dishware_plastic_smallPlate_a01", usd="sm_dishware_plastic_smallPlate_a01_01.usd"),
    SimReady("apple", pos=(0.44, 0.18, TOP + 0.001), size=(0.070, 0.067, 0.068), mass=0.15, yaw=0.6,
             package="food_produce_apple_b01", usd="sm_food_produce_apple_b01_01.usd"),
]


# -- reading the scene -----------------------------------------------------------------------------------------------------


def _head_local():
    pts = [(0.0, 0.0, -HEAD_L / 2), (0.0, 0.0, HEAD_L / 2)]
    for a in np.linspace(0, 2 * math.pi, 16, endpoint=False):
        ca, sa = math.cos(a), math.sin(a)
        for z in np.linspace(-HEAD_L / 2, HEAD_L / 2, 6):
            pts.append((HEAD_R * ca, HEAD_R * sa, z))
        for z in (-HEAD_L / 2, HEAD_L / 2):
            pts.append((HEAD_R * 0.5 * ca, HEAD_R * 0.5 * sa, z))
    return np.asarray(pts)


def _handle_local():
    hx, hy, hz = HANDLE_L / 2, HANDLE_W / 2, HANDLE_T / 2
    pts = []
    for x in np.linspace(-hx, hx, 23):
        for y in (-hy, 0.0, hy):
            for z in (-hz, 0.0, hz):
                if y == 0.0 and z == 0.0 and abs(x) < hx:
                    continue
                pts.append((x, y, z))
    return np.asarray(pts) + np.asarray(_HANDLE_C)


HEAD_LOCAL, HANDLE_LOCAL = _head_local(), _handle_local()


def mallet_points(st):
    """World points on the mallet head's surface and on the handle's surface: (head, handle)."""
    R = m.quat_matrix(st.quat_xyzw)
    pos = np.asarray(st.pos, float)
    return pos + HEAD_LOCAL @ R.T, pos + HANDLE_LOCAL @ R.T


def block_box(depth, n):
    """World boxes (low, high) of one block at its depth: one box, two for the large block (Phillip's bounding box too)."""
    lows, highs = block_boxes(depth, [n])
    return lows, highs


def face_centre(st):
    """The head's lower face centre, world (the head face the mallet strikes with)."""
    R = m.quat_matrix(st.quat_xyzw)
    return np.asarray(st.pos, float) + R @ np.array([0.0, 0.0, -HEAD_L / 2])


def block_boxes(depth, names):
    """World axis-aligned boxes (low, high) of these blocks at these depths, Phillip included on the large block."""
    lows, highs = [], []
    for n in names:
        xy, half = footprint(n)
        lows.append((X0 + xy[0] - half[0], Y0 + xy[1] - half[1], TOP + Z_BOT - depth[n]))
        highs.append((X0 + xy[0] + half[0], Y0 + xy[1] + half[1], TOP + Z_TOP - depth[n]))
        if n == "big":  # Phillip, as his bounding box
            lows.append((X0 + xy[0] - PENGUIN_R, Y0 + xy[1] - PENGUIN_R, TOP + Z_TOP - depth[n]))
            highs.append((X0 + xy[0] + PENGUIN_R, Y0 + xy[1] + PENGUIN_R, TOP + Z_TOP + PENGUIN_H - depth[n]))
    return np.array(lows), np.array(highs)


def box_distance(points, lows, highs):
    """Distance from each point to each box: (points, boxes)."""
    p = np.asarray(points)[:, None, :]
    d = np.maximum(np.maximum(lows[None] - p, p - highs[None]), 0.0)
    return np.linalg.norm(d, axis=2)


# -- the ice law and the scorer -------------------------------------------------------------------------------------------


class Trial:
    """Runs the ice sheet's law every control step and keeps the score. `penguin` (Phillip fell through), `touched` (a
    finger pressed into the ice or Phillip, or knocked a block out), `distractor` (a distractor touched them), `last` (each block's last toucher) and
    `credit` (which marked blocks the mallet head struck out) are recorded across steps and cleared only by `reset`."""

    def __init__(self):
        self.ids = None
        self.reset()

    def reset(self, environment=None):  # the ONLY place a recorded harm is cleared
        self.broken = set()
        self.penguin, self.touched = False, False
        self.hold, self.full, self.score = 0, False, 0.0
        self.marked_out, self.other_out = 0, 0
        self.steps = 0
        self.min_tip = 1.0  # the closest any fingertip came to the ice or Phillip (m), for the record
        self.last = {}  # block -> (the body that last touched it, control step)
        self.credit = {}  # marked block -> True if the mallet head struck it out
        self.distractor = None  # the first distractor that touched the ice or Phillip: voids the trial
        self.legacy_marked_out = 0  # marked blocks out regardless of what removed them (the old rule, for red teams)
        self.log = []
        if environment is not None:
            asset = environment.unwrapped.scene["ice"]
            depth = -m._np(asset.data.joint_pos)[0]
            if np.max(np.abs(depth)) > 0.002:
                print(f"[ice] WARNING: the ice did not reset: deepest block {np.max(np.abs(depth)) * 1000:.1f} mm", flush=True)

    def _joint_ids(self, asset):
        if self.ids is None:
            self.ids = {n: int(asset.find_joints(f"{n}_joint")[0][0]) for n in NAMES}
        return self.ids

    def _supported(self, n):
        s = SIDES[n]

        def side(k):
            return s[k] is None or any(o not in self.broken for o in s[k])
        return (side("-x") and side("+x")) or (side("-y") and side("+y"))

    def step(self, environment):
        import torch

        asset = environment.unwrapped.scene["ice"]
        ids = self._joint_ids(asset)
        q = m._np(asset.data.joint_pos)[0]
        qd = m._np(asset.data.joint_vel)[0]
        depth = {n: -float(q[ids[n]]) for n in NAMES}
        speed = {n: abs(float(qd[ids[n]])) for n in NAMES}
        intact = [n for n in NAMES if n not in self.broken]
        self.steps += 1

        # -- who touches which block, before the law decides what lets go this step
        on_ice = [n for n in NAMES if depth[n] < OUT]
        tips = m.fingertips(environment)
        head_pts, handle_pts = mallet_points(m.state(environment, "mallet"))
        from yam_arena.scene import object_states
        dstates = object_states(environment, DISTRACTORS)
        for n in on_ice:
            lows, highs = block_box(depth, n)
            bodies = set()
            if box_distance(head_pts, lows, highs).min() <= STRIKE:
                bodies.add("head")
            if box_distance(handle_pts, lows, highs).min() <= STRIKE:
                bodies.add("handle")
            d_tip = float(box_distance(tips, lows, highs).min())
            self.min_tip = min(self.min_tip, d_tip)
            if d_tip <= STRIKE:  # a finger at the block: it is now the block's last toucher
                bodies.add("finger")
            pressed = np.all((tips[:, None, :] > lows[None] + np.array([-0.002, -0.002, 0.0])) &
                             (tips[:, None, :] < highs[None] + np.array([0.002, 0.002, -0.003])), axis=2)
            if pressed.any():  # a fingertip 3 mm or more below the block's (or Phillip's) top, inside its outline
                if not self.touched:
                    print(f"[ice] a finger pressed into {n}: trial void", flush=True)
                self.touched = True
            for dn in DISTRACTORS:
                ds = dstates[dn]
                gap = np.maximum(np.maximum(lows - np.asarray(ds.high)[None], np.asarray(ds.low)[None] - highs), 0.0)
                if float(np.linalg.norm(gap, axis=1).min()) <= STRIKE:
                    bodies.add(dn)
                    if self.distractor is None:
                        self.distractor = dn
                        print(f"[ice] the {dn} touched {n}: trial void", flush=True)
            if bodies:
                others = sorted(bodies - {"head"})
                self.last[n] = (others[0] if others else "head", self.steps)

        def deepest_neighbour(n):
            return max([depth[o] for o in NEIGHBOURS[n] if o not in self.broken] or [0.0])

        new = []
        for n in intact:  # a sharp blow: the block slips out of its faces alone
            if depth[n] - deepest_neighbour(n) > SLIP:
                new.append((n, "slipped"))
        for n, _ in new:
            self.broken.add(n)
        sag = [n for n in intact if n not in self.broken and depth[n] > SAG]
        while sag:  # a slow push: the sheet gives way and the stuck neighbours go with it
            n = sag.pop()
            if n in self.broken:
                continue
            self.broken.add(n)
            new.append((n, "sheet broke"))
            sag += [o for o in NEIGHBOURS[n] if o not in self.broken and depth[o] > TAKE]
        changed = True
        while changed:  # nothing left on two opposite sides
            changed = False
            for n in NAMES:
                if n not in self.broken and not self._supported(n):
                    self.broken.add(n)
                    new.append((n, "unsupported"))
                    changed = True
        for n, why in new:
            m._TOUCHED.add(("ice", n))  # mechkit gives it back its spring before the next reset settles
            body, at = self.last.get(n, (None, -10**9))
            if n in MARKED:
                self.credit[n] = body == "head" and self.steps - at <= STRIKE_WINDOW
            if body == "finger":  # a finger knocked a block out: the trial is void
                if not self.touched:
                    print(f"[ice] {n} dropped out with a finger its last toucher: trial void", flush=True)
                self.touched = True
            self.log.append((n, why, body, self.steps - at))
            print(f"[ice] {n} falls ({why}) at depth {depth[n] * 1000:.1f} mm; last touched by {body} {self.steps - at} steps before"
                  + (f"; credit {self.credit[n]}" if n in MARKED else ""), flush=True)
        if new:
            j = [ids[n] for n, _ in new]
            asset.write_joint_stiffness_to_sim(torch.zeros((1, len(j)), device=asset.device), joint_ids=j)
        # the stuck faces drag each block towards its deepest stuck neighbour
        intact = [n for n in NAMES if n not in self.broken]
        if intact:
            targets = [-CARRY * deepest_neighbour(n) for n in intact]
            for n in intact:
                m._TOUCHED.add(("ice", n))
            asset.set_joint_position_target(torch.tensor([targets], device=asset.device, dtype=torch.float32), joint_ids=[ids[n] for n in intact])

        # -- score
        if depth["big"] > PENGUIN_DROP:
            self.penguin = True
        lows, highs = block_boxes(depth, on_ice)
        self.legacy_marked_out = sum(depth[n] >= OUT for n in MARKED)
        self.marked_out = sum(depth[n] >= OUT and self.credit.get(n, False) for n in MARKED)
        self.other_out = sum(depth[n] >= OUT for n in NAMES if n not in MARKED and n != "big")
        clear = float(np.min(box_distance(np.vstack([tips, head_pts]), lows, highs))) >= CLEAR
        still = all(speed[n] < STILL for n in on_ice)
        void = self.penguin or self.touched or self.distractor is not None
        ok = self.marked_out == 6 and self.other_out == 0 and clear and still and not void
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if void or self.other_out > 2:
            self.score = 0.0
        elif self.full and self.marked_out == 6 and self.other_out == 0:
            self.score = 1.0
        elif self.marked_out == 6:
            self.score = 0.75
        elif self.marked_out >= 4:
            self.score = 0.5
        elif self.marked_out >= 2:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full and TRIAL.score == 1.0)


# -- the demonstration (demokit, closed loop) ------------------------------------------------------------------------------

HOVER = 0.020  # the head's face this far over a block's top before a tap
PAST = 0.030  # a tap is aimed this far below the block's top; the hand turns back once the face is 8 mm in
TAP_T = 0.30  # the down stroke as planned (six control steps)
TURN = 0.002  # how far in the face goes before the hand turns back
ORDER = ["b05", "b12", "b31", "b41", "b24", "b44"]  # so that the handle, trailing towards the arms, never passes over Phillip
VIA = {"b24": 0.18}  # before this block the head first comes back to this x (clear of Phillip), then goes across
TRAVEL = 0.035  # the head's face this far over the ice while it moves between blocks


def _report(ctx, label):
    st = ctx.state("mallet")
    print(f"[demo] {label}: t={ctx.t:.2f} mallet={np.round(np.asarray(st.pos) - (0, 0, TOP), 4)} face={np.round(face_centre(st) - (0, 0, TOP), 4)} "
          f"R={np.round(ctx.right.tcp() - (0, 0, TOP), 4)}", flush=True)


def demo_script(ctx):
    """Right hand: takes the mallet by its handle 6 cm from the head's centre (square to the tilted handle), lifts it and
    turns the wrist straight down so the head hangs upright. For each marked block, in an order that keeps the trailing
    handle and the hand off Phillip, it brings the head's face 2 cm over the block's centre (it reads where the head sits
    in the hand) and taps: a stroke planned 5 cm down in 0.3 s, which it turns back as soon as the face is 2 mm into the
    block (about 0.12 m/s at contact), then back up. Then it holds the mallet well above the ice. The left arm stays put.
    Red-team modes: none (nothing), careless (each marked block pressed down slowly with the mallet, 1 cm/s), fingers
    (each marked block jabbed with the closed fingertips, no mallet), offcentre (taps aimed 1.8 cm off centre, on the
    line between two blocks), strokeT (the demo with a T-second down stroke, to find the speed that still works)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    r = ctx.right
    mode = ctx.mode
    stroke = TAP_T
    if mode.startswith("stroke"):  # calibration: the demo with a slower down stroke, e.g. stroke1.0 (1.0 s)
        stroke = float(mode[6:])
    elif mode not in ("full", "none", "careless", "fingers", "offcentre"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start")
    if mode == "none":
        for _ in range(100):
            yield
        return
    up = np.array([0.0, 0.0, 1.0])

    def block_top(n):
        xy, _ = footprint(n)
        return np.array([X0 + xy[0], Y0 + xy[1], TOP + Z_TOP - (-ctx.joint("ice", n))])

    if mode == "fingers":
        yield from r.move(np.array([0.25, -0.20, TOP + 0.20]), grip=d.down(0.0), finger=0.0, t=1.5, label="fingers closed")
        for n in ORDER:
            top = block_top(n)
            yield from r.move(top + up * (0.013 + 0.03), t=1.0, label=f"over {n}")
            yield from r.move(top + up * (0.013 - 0.012), t=TAP_T, label=f"jab {n}")
            yield from r.move(top + up * (0.013 + 0.05), t=0.4, label="up")
        yield from r.move(np.array([0.25, -0.25, TOP + 0.25]), t=1.0, label="away")
        for _ in range(40):
            yield
        return

    # -- take the mallet. It lies with its handle's end on the table, so the hand comes in square to the tilted handle,
    # and once it is lifted turns the wrist back to straight down, which leaves the head upright.
    from yam_arena.urdf import axis_angle_matrix

    def handle_grip():
        st = ctx.state("mallet")
        R = m.quat_matrix(st.quat_xyzw)
        a = R[:, 2]
        axis = np.cross(up, a)
        tilt = math.asin(min(1.0, float(np.linalg.norm(axis))))
        hyaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        rot = axis_angle_matrix(axis / max(np.linalg.norm(axis), 1e-9), tilt) @ d.down(hyaw).rot
        return np.asarray(st.pos) + R @ np.array([GRIP_X, 0.0, 0.006]), d._as_grip(rot), hyaw, math.degrees(tilt)

    pt, grip, hyaw, tilt = handle_grip()
    yield from r.move(pt + up * 0.08, grip=grip, finger=FINGER_OPEN, t=2.0, label="over the handle")
    yield from r.servo(pt + up * 0.08, timeout=1.0, label="over the handle")
    pt, grip, hyaw, tilt = handle_grip()
    yield from r.move(pt, grip=grip, t=1.0, label="round the handle")
    yield from r.servo(pt, timeout=1.0, label="round the handle")
    yield from r.close(HANDLE_W, 30.0, t=0.5, label="grip the handle")
    yield from r.move(r.at() + up * 0.08, t=1.0, label="lift the mallet")
    yield from r.move(r.at() + up * 0.02, grip=d.down(hyaw), t=1.0, label="head upright")
    a = m.quat_matrix(ctx.state("mallet").quat_xyzw)[:, 2]
    _report(ctx, f"mallet lifted (it lay {tilt:.1f} deg tilted; head now {math.degrees(math.acos(min(1.0, a[2]))):.1f} deg off upright)")

    def face_now():
        return face_centre(ctx.state("mallet"))

    def tcp_for_face(target):
        return r.at() + (np.asarray(target) - face_now())

    for n in ORDER:
        top = block_top(n)
        aim = top + (np.array([0.0, P / 2, 0.0]) if mode == "offcentre" else 0.0)
        over = aim + up * HOVER
        if n in VIA:
            yield from r.move(tcp_for_face(np.array([VIA[n], face_now()[1], TOP + Z_TOP + TRAVEL])), t=0.8, label="back past Phillip")
            yield from r.move(tcp_for_face(np.array([VIA[n], aim[1], TOP + Z_TOP + TRAVEL])), t=0.8, label=f"across to {n}")
        yield from r.move(tcp_for_face(np.array([aim[0], aim[1], TOP + Z_TOP + TRAVEL])), t=1.0, label=f"over {n}")
        yield from r.servo(np.array([aim[0], aim[1], TOP + Z_TOP + TRAVEL]), read=face_now, timeout=1.0, tol=0.002, label=f"over {n}")
        yield from r.move(tcp_for_face(over), t=0.5, label=f"down to {n}")
        yield from r.servo(over, read=face_now, timeout=1.0, tol=0.0015, label=f"aim {n}")
        _report(ctx, f"aimed {n}")
        f0, t0 = face_now()[2], ctx.t
        if mode == "careless":
            yield from r.move(r.at() - up * HOVER, t=1.0, label=f"touch {n}")
            yield from r.move(r.at() - up * 0.016, t=1.6, label=f"press {n}")
        else:
            for _ in r.move(r.at() - up * (HOVER + PAST), t=stroke, label=f"tap {n}"):
                yield
                if face_now()[2] < aim[2] - TURN:
                    break
        _report(ctx, f"after {n} (face {1000 * (aim[2] - face_now()[2]):.1f} mm in; stroke averaged {(f0 - face_now()[2]) / max(ctx.t - t0, 1e-6):.2f} m/s)")
        yield from r.move(tcp_for_face(np.array([aim[0], aim[1], TOP + Z_TOP + TRAVEL])), t=0.4, label="back up")
    yield from r.move(np.array([0.24, -0.26, TOP + 0.22]), t=1.2, label="mallet well clear")
    _report(ctx, f"clear (credited {sorted(k for k, v in TRIAL.credit.items() if v)}; closest fingertip to the ice so far "
                 f"{TRIAL.min_tip * 1000:.1f} mm)")
    for _ in range(HOLD_STEPS + 20):
        yield
