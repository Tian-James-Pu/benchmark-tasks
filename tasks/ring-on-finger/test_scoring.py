"""Scorer tests: every row of generated/v2/cards/ring-on-finger/scorer_test.txt (35 rows: 34 negative controls and
the blind-policy sweep), ported from that card's scorer.py, plus checks that ScoreTracker reads the same quantities
from rigid poses. CPU only: python -m unittest test_scoring -v"""
import math
import unittest

import numpy as np

import geometry as G
import scoring
from scoring import ScoreTracker, score

P = G.P
RING, RF, SW = P['ring'], P['hand']['ring_finger'], P['sweep']
BASE = dict(ring_offset_mm=0.4, ring_low_u_mm=float(P['hand']['palm_top_u_mm']),
            ring_top_u_mm=P['hand']['palm_top_u_mm'] + RING['width_mm'], robot_min_dist_mm=60, ring_move_1s_mm=0.0,
            ring_move_1s_deg=0.0, stand_move_max_mm=0.5, ring_min_z_ever_mm=0.0)


def row(**kw):
    d = dict(BASE)
    d.update(kw)
    if 'ring_low_u_mm' in kw and 'ring_top_u_mm' not in kw:
        d['ring_top_u_mm'] = None if kw['ring_low_u_mm'] is None else kw['ring_low_u_mm'] + RING['width_mm']
    return d


start_u = 0.0 - P['hand']['web_z_mm']
start_off = math.hypot(P['ring_start']['xy_mm'][0] - P['hand']['axis_xy_mm'][0],
                       P['ring_start']['xy_mm'][1] - P['hand']['axis_xy_mm'][1])
CASES = [
    ('start scene: ring flat on the table', row(ring_offset_mm=start_off, ring_low_u_mm=start_u, robot_min_dist_mm=200), 0.0),
    ('do nothing for the whole run', row(ring_offset_mm=start_off, ring_low_u_mm=start_u, robot_min_dist_mm=200), 0.0),
    ('ring held still in the seated pose, pads on it', row(robot_min_dist_mm=0), 0.0),
    ('ring held over the fingertip (hover)', row(ring_offset_mm=0.5, ring_low_u_mm=75, robot_min_dist_mm=0), 0.0),
    ('full: ring dropped to the palm top (u 0) after the knuckle, released, still', row(), 1.0),
    ('full: ring pushed down to u 6, not at the web', row(ring_low_u_mm=6), 1.0),
    ('full: ring tilted 5 deg resting on the palm (lower edge highest 1.88, top 5.86)',
     row(ring_offset_mm=1.2, ring_low_u_mm=1.88, ring_top_u_mm=5.86), 1.0),
    ('tilted 5 deg on the shaft: lower edge 7.1 palm side, highest 9.0', row(ring_low_u_mm=9.0, ring_top_u_mm=12.98), 0.5),
    ('tilted 5 deg, highest lower-edge point 8.0 (boundary)', row(ring_low_u_mm=8.0, ring_top_u_mm=11.98), 1.0),
    ('seated, spinning 3 deg about its own axis in the last 1 s (axis still)', row(ring_move_1s_deg=0.0), 1.0),
    ('ring dropped on the table, picked up again, finished', row(ring_min_z_ever_mm=0.0), 1.0),
    ('seated boundary: low face at 8.0', row(ring_low_u_mm=8.0), 1.0),
    ('just outside seated: low face at 8.1 (stuck on the shaft)', row(ring_low_u_mm=8.1), 0.5),
    ('past-PIP boundary: top face at 26.0', row(ring_low_u_mm=22.0), 0.5),
    ('just outside past-PIP: top face at 26.1', row(ring_low_u_mm=22.1), 0.25),
    ('ring jammed across the PIP crest (top at 30)', row(ring_low_u_mm=26.0), 0.25),
    ('past-DIP boundary: top face at 50.0', row(ring_low_u_mm=46.0), 0.25),
    ('just outside past-DIP: top at 50.1', row(ring_low_u_mm=46.1), 0.0),
    ('ring perched on the fingertip, not around it', row(ring_offset_mm=3.0, ring_low_u_mm=70.0), 0.0),
    ('ring on the middle finger, at its base', row(ring_offset_mm=19.5, ring_low_u_mm=2.0), 0.0),
    ('around boundary: offset 5.0', row(ring_offset_mm=5.0), 1.0),
    ('around just outside: offset 5.1', row(ring_offset_mm=5.1), 0.0),
    ('released boundary: robot 30.0 mm away', row(robot_min_dist_mm=30.0), 1.0),
    ('released just outside: robot 29.9 mm away', row(robot_min_dist_mm=29.9), 0.0),
    ('not still: ring slid 2.0 mm in the last 1 s', row(ring_move_1s_mm=2.0), 0.0),
    ('still boundary: 1.9 mm, 1.9 deg', row(ring_move_1s_mm=1.9, ring_move_1s_deg=1.9), 1.0),
    ('not still: ring turned 2.0 deg in the last 1 s', row(ring_move_1s_deg=2.0), 0.0),
    ('hand form dragged 5.1 mm, then pushed back, ring seated', row(stand_move_max_mm=5.1), 0.0),
    ('hand form moved 5.0 mm (boundary), ring seated', row(stand_move_max_mm=5.0), 1.0),
    ('ring knocked off the table, replaced, seated', row(ring_min_z_ever_mm=-25.0), 0.0),
    ('off-table boundary: lowest ring height -19.9 mm, seated', row(ring_min_z_ever_mm=-19.9), 1.0),
    ('just outside off-table: lowest ring height -20.1 mm, seated', row(ring_min_z_ever_mm=-20.1), 0.0),
    ('reading unknown (ring hidden): counts as fail', row(ring_low_u_mm=None), 0.0),
    ('ring past the DIP only, but still held', row(ring_low_u_mm=40.0, robot_min_dist_mm=5), 0.0),
]


def blind_end_state(draw, aim):
    hdx, hdy, cdx, cdy = draw
    ax, ay = aim
    if abs(cdy) > SW['pad_catch_half_width_mm']:
        return row(ring_offset_mm=start_off, ring_low_u_mm=start_u, robot_min_dist_mm=200), 'missed the ring'
    ex, ey = ax - hdx, ay + cdy - hdy
    e = math.hypot(ex, ey)
    if e > SW['tip_capture_mm']:
        return row(ring_offset_mm=e, ring_low_u_mm=RF['length_mm'] - 1.0, ring_move_1s_mm=0.0), 'lands on the fingertip'
    if e > SW['pip_pass_mm']:
        return row(ring_offset_mm=e, ring_low_u_mm=RF['pip_crest_u_mm'] - RING['width_mm'] / 2), 'stalls at the PIP crest'
    return row(ring_offset_mm=e), 'passes the PIP, drops to the web'


def sweep():
    best = None
    n = int(SW['aim_grid_max_mm'] / SW['aim_grid_step_mm'])
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            aim = (i * SW['aim_grid_step_mm'], j * SW['aim_grid_step_mm'])
            m = sum(score(blind_end_state(d, aim)[0]) for d in SW['draws']) / len(SW['draws'])
            if best is None or m > best[0] + 1e-12 or (abs(m - best[0]) <= 1e-12 and math.hypot(*aim) < math.hypot(*best[1])):
                best = (m, aim)
    return best


class CardRows(unittest.TestCase):
    def test_negative_controls(self):
        self.assertEqual(len(CASES), 34)
        for name, state, expected in CASES:
            with self.subTest(name):
                self.assertAlmostEqual(score(state), expected, msg=name)

    def test_blind_sweep(self):
        m, aim = sweep()
        self.assertLessEqual(m, SW['mean_max'])
        self.assertAlmostEqual(m, 0.1)
        self.assertEqual(aim, (0.0, 0.0))


def _rot_x(deg):
    t = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])


class TrackerReadsPoses(unittest.TestCase):
    hand = np.array([*G.HAND_AXIS_XY, G.TABLE_TOP])
    eye = np.eye(3)

    def run_still(self, ring_pos, ring_rot=None, steps=25, robot_far=True):
        t = ScoreTracker(20)
        for _ in range(steps):
            res = t.update(ring_pos, self.eye if ring_rot is None else ring_rot, self.hand, self.eye, robot_hulls=None,
                           read_robot=False)
        t.robot_min_dist_mm = 200.0 if robot_far else 5.0
        t.last['robot_min_dist_mm'] = t.robot_min_dist_mm
        return t.result()

    def test_start_flat_on_table_scores_zero(self):
        res = self.run_still(np.array([*G.RING_START_XY, G.TABLE_TOP + G.RING_W / 2]))
        st = res['state']
        self.assertAlmostEqual(st['ring_min_z_ever_mm'], 0.0, places=6)
        self.assertAlmostEqual(st['ring_low_u_mm'], -130.0, places=6)
        self.assertEqual(res['score'], 0.0)

    def test_seated_on_palm_scores_one(self):
        res = self.run_still(self.hand + [0.0004, 0, G.WEB_Z + G.RING_W / 2])
        self.assertAlmostEqual(res['state']['ring_low_u_mm'], 0.0, places=6)
        self.assertAlmostEqual(res['state']['ring_top_u_mm'], 4.0, places=6)
        self.assertEqual(res['score'], 1.0)

    def test_tilted_on_shaft(self):
        # 5 deg about x, centre at u 7.1 + 2: the high lower-edge point is above 8 -> 0.5
        res = self.run_still(self.hand + [0, 0, G.WEB_Z + 0.0091], _rot_x(5))
        self.assertGreater(res['state']['ring_low_u_mm'], 8.0)
        self.assertEqual(res['score'], 0.5)

    def test_held_scores_zero(self):
        self.assertEqual(self.run_still(self.hand + [0, 0, G.WEB_Z + 0.002], robot_far=False)['score'], 0.0)

    def test_needs_a_full_second(self):
        t = ScoreTracker(20)
        res = t.update(self.hand + [0, 0, G.WEB_Z + 0.002], self.eye, self.hand, self.eye, read_robot=False)
        self.assertIsNone(res['state']['ring_move_1s_mm'])
        self.assertEqual(res['score'], 0.0)

    def test_plate_drag_latches(self):
        t = ScoreTracker(20)
        ring = self.hand + [0, 0, G.WEB_Z + 0.002]
        t.update(ring, self.eye, self.hand, self.eye, read_robot=False)
        moved = self.hand + [0.0052, 0, 0]
        t.update(ring + [0.0052, 0, 0], self.eye, moved, self.eye, read_robot=False)
        for _ in range(25):
            t.update(ring, self.eye, self.hand, self.eye, read_robot=False)
        t.last['robot_min_dist_mm'] = 200.0
        self.assertGreater(t.last['stand_move_max_mm'], 5.0)
        self.assertEqual(t.result()['score'], 0.0)

    def test_moving_ring_not_still(self):
        t = ScoreTracker(20)
        for k in range(25):
            t.update(self.hand + [0, 0, G.WEB_Z + 0.002 + 0.0002 * k], self.eye, self.hand, self.eye, read_robot=False)
        t.last['robot_min_dist_mm'] = 200.0
        self.assertGreaterEqual(t.last['ring_move_1s_mm'], 2.0)
        self.assertEqual(t.result()['score'], 0.0)

    def test_hull_distance(self):
        class H:
            pass
        h = H()
        # unit-cube hull 0.1 m above the ring
        c = np.array([0.3, 0.0, 1.0])
        h.vertices = c + np.array([[sx, sy, sz] for sx in (-.05, .05) for sy in (-.05, .05) for sz in (-.05, .05)])
        normals = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)
        h.planes = np.c_[normals, normals @ c + .05]
        d = scoring.min_distance_to_hulls(np.array([[0.3, 0.0, 0.9]]), [('x', h)])
        self.assertAlmostEqual(d, 0.05, places=9)


if __name__ == '__main__':
    unittest.main()
