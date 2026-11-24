"""Scripted demo for food-magnets: camera frames, joint readings and the published rig calibration only.

No simulator pose, no pose write, no attachment: the policy sees ``observation.images`` (top_cam, left_cam, right_cam) and
``observation.state['joint_pos']`` and returns 14-D absolute joint targets like any yam_arena policy. The board plane,
the box spots and the part sizes are the card's drawings; where the magnets and the box are is found with the cameras.

Plan (one arm, the one on the box's side; the other stays at its ready pose):
  survey   top camera: the three food-coloured blobs on the board (back-projected onto the magnets' front plane) and the
           box spot (the one of four drawn spots that does not look like bare table)
  for each food magnet:
    approach  jaws open 75 mm, closing along world z, fingers pointing at the steel 100 then 60 mm off it; the wrist
              camera re-fits the blob there (0.7 mm/px)
    pinch     straight in until the pad tips are 3 mm off the steel (the reviewer's note 4: pads close off the steel,
              never dragging it), TCP trimmed from the joint readings, close to ~20 N per pad
    pull      straight back 60 mm at 20 mm/s; held only if the gripper reading stays above GRIP_HELD (re-pick: open, re-fit
              with the wrist camera, retry, up to 3 tries; build/ring-on-finger lesson)
    drop      pitched 75 deg down over the box, the magnet 45 mm above the rim, open; lift
  lid      fingers closed, pitched 45 deg: push the open lid's outer face from x 140 to 215 mm at 85-95 mm height; past
           vertical it falls shut on its hinge
  home     both arms at ready, grippers open; hold 1.5 s
Mode 'hover' carries the last magnet over the box and holds it there (a baseline: never released, lid open).
Mode 'boxall' (r5 red team, not a demo) is the route review TK-9NQBPY names, every magnet that does not hold the note
boxed: the demo plus the two ordinary magnets, whose drawn centres it reads from the trial's layout (an oracle, the
strongest form of that route); each is pinched 4 mm inside its own height.
"""
import math
from dataclasses import dataclass, field

import numpy as np

import geometry as G
import perception as V

READY = np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])
PAD_TIP = 0.013                 # yam_arena.constants.PAD_TIP_PAST_TCP
TIP_OFF_STEEL = 0.003           # pad tips at the pinch, off the steel
STAGE_OFF_STEEL = 0.150         # pad tips this far off the steel at the end of every joint-space transfer ...
STAGE_Z = 0.260                 # ... and at least this high above the table: clear of the open lid (top edge 112 mm, x 160-183;
                                # job 4296645 seed 1: a staging pose at z 150, x 187 knocked the lid shut)
HELD_IN_GRIPPER = 0.011         # magnet centre past the TCP along the approach axis when pinched (tips 3 mm off the steel)
GRIP_OPEN = 0.80                # 75 mm between the pads
GRIP_PINCH = (38.0 - 8.0) / 93.9   # pads commanded 4 mm inside a 38 mm body each side (~20 N each)
GRIP_HELD = 0.36                # held 38-40 mm bodies read 0.40-0.43; empty jaws close to GRIP_PINCH (0.32)
DROP_X = 0.230                  # magnet centre over the box (inner walls x 186..254 mm), clear of the open lid
DROP_ABOVE_RIM = 0.045
DROP_DY = (-0.039, 0.0, 0.039)  # side by side along the box (30 + 40 + 40 = 110 <= 116)
DROP_PITCH = 75.0
LID_PITCH = 45.0
LID_PUSH = ((0.140, 0.095), (0.215, 0.085))   # fingertip path (x, z above the table), m
SPEED = 0.15                    # m/s, free Cartesian moves
JOINT_SPEED = 1.2               # rad/s, joint-space transfers
SEEDS = (READY, np.array([0.0, 1.4, 1.2, -0.2, 0.0, 0.0]), np.array([0.5, 1.2, 1.0, -0.4, 0.0, 0.0]),
         np.array([-0.5, 1.2, 1.0, -0.4, 0.0, 0.0]))


def rot(pitch_deg=0.0, flip=False):
    """Gripper frame: approach z toward +x pitched down by ``pitch_deg`` about world y; jaws (x) close along world z
    (rotated with the pitch); ``flip`` turns it 180 deg about the approach axis."""
    h = np.column_stack([(0, 0, 1.0), (0, -1.0, 0), (1.0, 0, 0)])
    if flip:
        h = h @ np.diag([-1.0, -1.0, 1.0])
    return G.rot_y(pitch_deg) @ h


def quat(r):
    w, x, y, z = G.matrix_to_quat_wxyz(r)
    return (float(x), float(y), float(z), float(w))


@dataclass
class Phase:
    name: str
    start: int
    end: int = None
    info: dict = field(default_factory=dict)


class MagnetDemo:
    def __init__(self, control_hz=20, mode='demo', cam_width=320, cam_height=176):
        if (cam_width, cam_height) != (V.CAM_W, V.CAM_H):
            raise ValueError('the demo is calibrated for 320 x 176 observations')
        self.hz, self.mode = control_hz, mode
        self.log, self.step, self.done, self.events = [], 0, False, []
        self.side = None
        self.q_cmd = {'left': READY.copy(), 'right': READY.copy()}
        self.grip = {'left': 1.0, 'right': 1.0}
        self.offset = np.zeros(3)
        self.arm = None
        self.flip = False
        self._obs = None
        self._plan = self._script()

    # -- helpers ------------------------------------------------------------------------------------------
    def _phase(self, name, **info):
        if self.log:
            self.log[-1].end = self.step
        self.log.append(Phase(name, self.step, info=info))

    def q(self):
        j = np.asarray(self._obs.state['joint_pos'], float)
        return j[:6] if self.side == 'left' else j[7:13]

    def reading(self):
        j = np.asarray(self._obs.state['joint_pos'], float)
        return float(j[6] if self.side == 'left' else j[13])

    def _ik(self, tcp, r, seed=None):
        from yam_arena.ik import solve
        best = None
        for s in ([seed] if seed is not None else []) + list(SEEDS):
            q, err, rerr = solve(np.asarray(tcp) + self.offset, quat(r), q_init=s, base_pos=self.arm.base, iters=200, tol=2e-5)
            if best is None or err + 0.05 * rerr < best[1] + 0.05 * best[2]:
                best = (q, err, rerr)
            if err < 1e-4 and rerr < 1e-3:
                break
        info = self.log[-1].info
        info['worst_ik_mm'] = round(max(info.get('worst_ik_mm', 0.0), best[1] / G.MM), 3)
        return best[0]

    def _tcp_cmd(self):
        return self.arm.tcp(self.q_cmd[self.side])[0] - self.offset

    def _move(self, target, r, speed=SPEED, settle=4):
        """Straight-line TCP move (fixed orientation ``r``) from the current command to ``target``."""
        start = self._tcp_cmd()
        n = max(1, int(math.ceil(np.linalg.norm(np.asarray(target) - start) / (speed / self.hz))))
        for k in range(1, n + 1):
            self.q_cmd[self.side] = self._ik(start + (np.asarray(target) - start) * k / n, r, seed=self.q_cmd[self.side])
            yield
        for _ in range(settle):
            yield

    def _joint_to(self, target, r, settle=6):
        """Joint-space transfer to the IK solution of (target, r)."""
        q1 = self._ik(target, r, seed=self.q_cmd[self.side])
        yield from self._joint_move(q1, settle=settle)

    def _joint_move(self, q1, settle=6):
        q0 = self.q_cmd[self.side].copy()
        n = max(4, int(math.ceil(np.abs(q1 - q0).max() / (JOINT_SPEED / self.hz))))
        for k in range(1, n + 1):
            self.q_cmd[self.side] = q0 + (q1 - q0) * k / n
            yield
        for _ in range(settle):
            yield

    def _correct(self, target, tol=5e-4, rounds=4, wait=8, gain=0.6):
        for _ in range(rounds):
            err = np.asarray(target) - self.arm.tcp(self.q())[0]
            self.log[-1].info['tcp_error_mm'] = round(float(np.linalg.norm(err) / G.MM), 3)
            if np.linalg.norm(err) < tol:
                return
            self.offset += gain * err
            self.q_cmd[self.side] = self._ik(target, self._r_now, seed=self.q_cmd[self.side])
            for _ in range(wait):
                yield

    def _grip_to(self, value, steps=12):
        g0 = self.grip[self.side]
        for k in range(1, steps + 1):
            self.grip[self.side] = g0 + (value - g0) * k / steps
            yield

    def _hold(self, n):
        for _ in range(n):
            yield

    def _tcp_for_tip(self, x_tip, y, z, r):
        """TCP so the pad tips are at x_tip, the jaw centre on (y, z above the table)."""
        tip = np.array([x_tip, y, G.TABLE_TOP + z])
        return tip - r[:, 2] * PAD_TIP

    def _refit(self, yz):
        """Wrist camera at the 60 mm pre-pinch pose: the food blob nearest the predicted spot."""
        cam_name = 'left_cam' if self.side == 'left' else 'right_cam'
        cam = self.arm.camera(self.q())
        img = np.asarray(self._obs.images[cam_name])
        blobs = V.find_food(img, cam, min_px=40)
        guess = np.array(yz)
        best = None
        for b in blobs:
            d = np.linalg.norm(np.array(b['yz']) - guess)
            if d < 0.025 and (best is None or d < best[0]):
                best = (d, b)
        ev = dict(step=self.step, what='wrist refit', guess_mm=(guess / G.MM).round(1).tolist(),
                  blobs=[dict(yz_mm=(np.array(b['yz']) / G.MM).round(1).tolist(), px=b['px']) for b in blobs[:6]])
        self.events.append(ev)
        if best is not None:
            ev['used_mm'] = (np.array(best[1]['yz']) / G.MM).round(1).tolist()
            return tuple(best[1]['yz'])
        return tuple(yz)

    # -- the script ---------------------------------------------------------------------------------------
    def _script(self):
        self._phase('survey')
        yield from self._hold(4)
        top = V.top_camera_model()
        img = np.asarray(self._obs.images['top_cam'])
        blobs = V.find_food(img, top, min_px=35)
        spot, diffs = V.find_box(img, top)
        box_y = G.SPOTS[spot]
        self.side = 'left' if box_y > 0 else 'right'
        self.arm = V.ArmModel(self.side)
        self.flip = self.side == 'left'          # pitched poses: IK branch that reaches every spot on the arm's side
        foods = [b['yz'] for b in blobs[:3]]
        pinch_h = {}                             # boxall only: target index -> its height (m) for the pinch
        if self.mode == 'boxall':
            import os
            import scene
            d = scene.draw(int(os.environ.get('ROBOCURVE_FM_SEED', '0')))
            for n in G.ORDINARY:
                pinch_h[tuple(d[n])] = G.EXTENT[n][1]
                foods.append(tuple(d[n]))
        self.log[-1].info.update(box_spot=spot, box_y_mm=round(box_y / G.MM), spot_diffs=diffs, arm=self.side,
                                 food_yz_mm=[(np.array(f) / G.MM).round(1).tolist() for f in foods],
                                 blobs=[dict(yz_mm=(np.array(b['yz']) / G.MM).round(1).tolist(), px=b['px'], rgb=b['rgb'])
                                        for b in blobs[:6]])
        # nearest the arm's base first (shorter transfers)
        base_y = self.arm.base[1]
        foods.sort(key=lambda f: abs(f[0] - base_y))
        r_pinch = rot(0.0, False)
        r_drop = rot(DROP_PITCH, self.flip)
        for k, yz in enumerate(foods):
            last = k == len(foods) - 1
            h = pinch_h.get(tuple(yz))
            g_pinch = GRIP_PINCH if h is None else (h / G.MM - 8.0) / 93.9
            g_held = GRIP_HELD if h is None else (h / G.MM - 4.0) / 93.9
            held = False
            for attempt in range(3):
                self._phase(f'approach{k + 1}' + (f'_retry{attempt}' if attempt else ''))
                self._r_now = r_pinch
                self.offset = np.zeros(3)
                yield from self._grip_to(GRIP_OPEN, 8)
                # joint-space transfers stay 200 mm off the steel (job 4296373 step 6, seed 2: the right arm's direct
                # joint move from ready to a high-row pre-grasp swept past the note's plain magnet and moved it 17.7 mm);
                # the last 100 mm toward the board are straight Cartesian lines at the magnet's height.
                stage = self._tcp_for_tip(G.FACE_X - STAGE_OFF_STEEL, yz[0], max(yz[1], STAGE_Z), r_pinch)
                yield from self._joint_to(stage, r_pinch)
                far = self._tcp_for_tip(G.FACE_X - 0.100, yz[0], yz[1], r_pinch)
                yield from self._move(far, r_pinch, settle=2)
                near = self._tcp_for_tip(G.FACE_X - 0.060, yz[0], yz[1], r_pinch)
                yield from self._move(near, r_pinch, settle=8)
                if h is None:
                    yz = self._refit(yz)
                self.log[-1].info['yz_mm'] = (np.array(yz) / G.MM).round(1).tolist()
                self._phase(f'pinch{k + 1}')
                near = self._tcp_for_tip(G.FACE_X - 0.060, yz[0], yz[1], r_pinch)
                yield from self._move(near, r_pinch, speed=0.03, settle=4)
                at = self._tcp_for_tip(G.FACE_X - TIP_OFF_STEEL, yz[0], yz[1], r_pinch)
                yield from self._move(at, r_pinch, speed=0.03, settle=6)
                yield from self._correct(at)
                yield from self._grip_to(g_pinch, 12)
                yield from self._hold(10)
                self.log[-1].info['pinch_reading'] = round(self.reading(), 4)
                self._phase(f'pull{k + 1}')
                yield from self._move(self._tcp_for_tip(G.FACE_X - 0.060, yz[0], yz[1], r_pinch), r_pinch, speed=0.02, settle=8)
                rd = self.reading()
                self.log[-1].info['pulled_reading'] = round(rd, 4)
                if rd > g_held:
                    held = True
                    break
            self.log[-1].info['held'] = held
            yield from self._move(self._tcp_for_tip(G.FACE_X - 0.060, yz[0], max(yz[1], STAGE_Z), r_pinch), r_pinch,
                                  speed=0.08, settle=0)     # up first, at 60 mm off the steel: over the open lid only when high
            yield from self._move(self._tcp_for_tip(G.FACE_X - STAGE_OFF_STEEL, yz[0], max(yz[1], STAGE_Z), r_pinch), r_pinch,
                                  speed=0.10, settle=2)
            # transfer over the box
            self._phase(f'drop{k + 1}')
            self._r_now = r_drop
            self.offset = np.zeros(3)
            c = np.array([DROP_X, box_y + DROP_DY[k % 3], G.TABLE_TOP + G.BOX_H + DROP_ABOVE_RIM])
            high = c + [0.0, 0.0, 0.08] - r_drop[:, 2] * HELD_IN_GRIPPER
            low = c - r_drop[:, 2] * HELD_IN_GRIPPER
            yield from self._joint_to(high, r_drop, settle=6)
            yield from self._move(low, r_drop, speed=0.06, settle=6)
            self.log[-1].info['reading_over_box'] = round(self.reading(), 4)
            if self.mode == 'hover' and last:
                self._phase('hover_hold')
                yield from self._hold(60)
                self.done = True
                return
            yield from self._grip_to(GRIP_OPEN, 6)
            yield from self._hold(12)
            yield from self._move(high, r_drop, speed=0.10, settle=2)
        # the lid
        self._phase('lid')
        r_lid = rot(LID_PITCH, self.flip)
        self._r_now = r_lid
        self.offset = np.zeros(3)
        yield from self._grip_to(0.0, 10)
        (x0, z0), (x1, z1) = LID_PUSH
        start = np.array([x0, box_y, G.TABLE_TOP + z0]) - r_lid[:, 2] * PAD_TIP
        above = start + [0.0, 0.0, 0.08]
        yield from self._joint_to(above, r_lid, settle=4)
        yield from self._move(start, r_lid, speed=0.08, settle=4)
        end = np.array([x1, box_y, G.TABLE_TOP + z1]) - r_lid[:, 2] * PAD_TIP
        yield from self._move(end, r_lid, speed=0.05, settle=10)
        yield from self._move(end + [-0.06, 0.0, 0.03], r_lid, speed=0.08, settle=2)
        yield from self._move(end + [-0.06, 0.0, 0.15], r_lid, speed=0.12, settle=2)
        self._phase('home')
        self.offset = np.zeros(3)
        yield from self._grip_to(1.0, 8)
        yield from self._joint_move(READY, settle=4)
        self._phase('final_hold')
        yield from self._hold(30)
        self.done = True

    # -- policy interface ---------------------------------------------------------------------------------
    def act(self, observation):
        self._obs = observation
        if not self.done:
            try:
                next(self._plan)
            except StopIteration:
                self.done = True
        self.step += 1
        return np.r_[self.q_cmd['left'], self.grip['left'], self.q_cmd['right'], self.grip['right']]

    def report(self):
        if self.log and self.log[-1].end is None:
            self.log[-1].end = self.step
        return dict(phases=[dict(name=p.name, start=p.start, end=p.end, **_jsonable(p.info)) for p in self.log],
                    events=_jsonable(self.events), arm=self.side, final_offset_mm=(self.offset / G.MM).round(3).tolist())


def _jsonable(x):
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return x.item()
    return x


def make_policy(cam_width, cam_height, control_hz, mode='demo'):
    return MagnetDemo(control_hz, mode, cam_width, cam_height)
