"""Scorer tests: every negative-control row of the card's scorer (card_scorer.py, r5: review TK-9NQBPY added two
ordinary magnets that must stay on the board) through scoring.score, plus ScoreTracker readings from synthetic poses."""
import unittest

import numpy as np

import card_scorer
import geometry as G
import scene
import scoring


def start_poses(d):
    p = {n: (np.array(scene.body_start(n, d)), np.eye(3)) for n in ('strawberry', 'cookie', 'donut', 'plain') + G.ORDINARY}
    p['card'] = (np.array(scene.card_start(d)), np.eye(3))
    p['lid'] = (np.array(scene.lid_start(d)), G.LID_OPEN_ROT.copy())
    return p


def boxed(d, p):
    q = dict(p)
    for k, n in enumerate(G.NAMES):
        c = np.array([G.BOX_X, d['box_y'] + (k - 1) * 0.039, G.TABLE_TOP + G.FLOOR_T + 0.006])
        q[n] = (c, G.rot_y(90))      # lying flat on the floor
    q['lid'] = (np.array(scene.lid_start(d)), np.eye(3))
    return q


class FarHull:
    def __init__(self, c):
        from types import SimpleNamespace
        v = np.array(c) + np.array([[x, y, z] for x in (-.01, .01) for y in (-.01, .01) for z in (-.01, .01)])
        planes = np.array([[1, 0, 0, c[0] + .01], [-1, 0, 0, -c[0] + .01], [0, 1, 0, c[1] + .01], [0, -1, 0, -c[1] + .01],
                           [0, 0, 1, c[2] + .01], [0, 0, -1, -c[2] + .01]], float)
        self.h = SimpleNamespace(vertices=v, planes=planes)


def hulls_at(c):
    return [('gripper', FarHull(c).h)]


class CardRows(unittest.TestCase):
    def test_card_rows(self):
        rows = card_scorer.rows()
        self.assertEqual(len(rows), 69)
        for name, st, exp in rows:
            with self.subTest(name):
                self.assertAlmostEqual(scoring.score(st), exp)
                self.assertAlmostEqual(card_scorer.score(st), exp)

    def test_box_every_loose_magnet_scores_zero(self):
        """r5 (review TK-9NQBPY): boxing every magnet that does not hold the note (3 food + 2 ordinary), lid shut: 0."""
        st = card_scorer.base([card_scorer.food_in_box(-30, -40), card_scorer.food_in_box(), card_scorer.food_in_box(25, 40)])
        self.assertEqual(scoring.score(st), 1.0)
        for o in st['ordinary']:
            o.update(**card_scorer.BOXED)
        self.assertEqual(scoring.score(st), 0.0)

    def test_blind_sweep(self):
        """r5: every fixed (blind) plan is bounded by 0.05; the 3-cell reference is exactly 1/80; band sweeps 0;
        cross-checked through scoring.score on 2,000 random draws."""
        import random
        self.assertAlmostEqual(card_scorer.analytic_bound(), 0.05)
        lib = card_scorer.pattern_library()
        probs = {k: card_scorer.cell_probs(lib[k]) for k in ('none', '1@(-15,-15)z')}
        cells3 = ['1@(-15,-15)z'] * 3 + ['none'] * 3
        self.assertAlmostEqual(card_scorer.plan_mean(cells3, probs), 1 / 80)
        self.assertEqual(scoring.score(card_scorer.base([card_scorer.food_on_table() for _ in range(3)], card_scorer.LID_OPEN)), 0.0)
        plan = card_scorer.to_abs(cells3, lib)
        rng = random.Random(20261124)
        mc = sum(scoring.score(card_scorer.draw_state(plan, card_scorer.concrete_draw(rng), 0)) for _ in range(2000)) / 2000
        self.assertLess(mc, 0.05)

    def test_same_thresholds(self):
        self.assertEqual(scoring.S, card_scorer.S)
        self.assertEqual(scoring.BOX, card_scorer.BOX)


class Tracker(unittest.TestCase):
    def setUp(self):
        self.d = scene.draw(1)
        self.p0 = start_poses(self.d)

    def run_steps(self, p, hull_c=(0.3, 0.305, 1.0), n=25):
        t = scoring.ScoreTracker(self.d, 20)
        t.update(self.p0, hulls_at(hull_c))
        for _ in range(n):
            r = t.update(p, hulls_at(hull_c))
        return r

    def test_start_scene_zero(self):
        r = self.run_steps(self.p0)
        self.assertEqual(r['score'], 0.0, r['explanation'])
        self.assertAlmostEqual(r['state']['box']['lid_gap'], 72.87, places=1)
        self.assertLess(abs(r['state']['plain'][0]['gap']), 0.2)

    def test_full(self):
        r = self.run_steps(boxed(self.d, self.p0))
        self.assertEqual(r['score'], 1.0, r['explanation'])

    def test_lid_open(self):
        p = boxed(self.d, self.p0)
        p['lid'] = self.p0['lid']
        self.assertEqual(self.run_steps(p)['score'], 0.5)

    def test_gripper_near(self):
        p = boxed(self.d, self.p0)
        r = self.run_steps(p, hull_c=(G.BOX_X, self.d['box_y'], G.TABLE_TOP + 0.08))
        self.assertEqual(r['score'], 0.0)

    def test_note_moved(self):
        p = boxed(self.d, self.p0)
        c, rr = p['card']
        p['card'] = (c + [0, 0, -0.012], rr)
        self.assertEqual(self.run_steps(p)['score'], 0.0)

    def test_magnet_on_table(self):
        p = boxed(self.d, self.p0)
        p['cookie'] = (np.array([0.15, 0.0, G.TABLE_TOP + 0.004]), G.rot_y(90))
        self.assertEqual(self.run_steps(p)['score'], 0.0)

    def test_two_of_three(self):
        p = boxed(self.d, self.p0)
        p['donut'] = self.p0['donut']
        self.assertEqual(self.run_steps(p)['score'], 0.0)

    def test_ordinary_kept_readings(self):
        r = self.run_steps(boxed(self.d, self.p0))
        for o in r['state']['ordinary']:
            self.assertLess(o['disp'], 1e-6)
            self.assertLess(abs(o['gap']), 0.2)

    def test_ordinary_boxed_scores_zero(self):
        """r5: the route the review names, every loose magnet boxed (here both ordinary magnets on the box floor)."""
        p = boxed(self.d, self.p0)
        for k, n in enumerate(G.ORDINARY):
            c = np.array([G.BOX_X + 0.02, self.d['box_y'] + (k - 0.5) * 0.04, G.TABLE_TOP + G.FLOOR_T + 0.016])
            p[n] = (c, G.rot_y(90))
        r = self.run_steps(p)
        self.assertEqual(r['score'], 0.0, r['explanation'])
        self.assertIn('not kept', r['explanation'])

    def test_ordinary_slid_on_board(self):
        p = boxed(self.d, self.p0)
        c, rr = p['square']
        p['square'] = (c + [0, 0.0105, 0], rr)
        self.assertEqual(self.run_steps(p)['score'], 0.0)
        p['square'] = (c + [0, 0.0095, 0], rr)
        self.assertEqual(self.run_steps(p)['score'], 1.0)

    def test_ordinary_lifted_off_steel(self):
        p = boxed(self.d, self.p0)
        c, rr = p['round']
        p['round'] = (c + [-0.0016, 0, 0], rr)          # back face 1.6 mm off the steel
        self.assertEqual(self.run_steps(p)['score'], 0.0)


class Draws(unittest.TestCase):
    def test_draws_keep_gaps(self):
        for seed in range(0, 200):
            d = scene.draw(seed)
            self.assertTrue(scene.gaps_ok(d), seed)
            self.assertIn(d['box_y'], G.SPOTS)
            cells = {d[n + '_cell'] for n in scene.ITEMS}
            self.assertEqual(len(cells), 6)

    def test_seed1_keeps_the_r4_layout(self):
        """r5 fills the two cells left empty before; seed 1's note and food magnets and box spot are unchanged."""
        d = scene.draw(1)
        got = {n: (round(d[n][0] * 1000), round(d[n][1] * 1000)) for n in scene.ITEMS}
        self.assertEqual({n: got[n] for n in ('plain', 'strawberry', 'cookie', 'donut')},
                         dict(plain=(-4, 298), strawberry=(-133, 158), cookie=(141, 151), donut=(5, 161)))
        self.assertEqual(round(d['box_y'] * 1000), 195)


class Magnet(unittest.TestCase):
    def test_pull_law(self):
        d = scene.draw(1)
        p, _ = np.array(scene.body_start('cookie', d)), None
        f, c, g = G.pull('cookie', p, np.eye(3))
        self.assertAlmostEqual(f, G.F0 / (1 + g / 1e-3) ** 2, places=6)
        self.assertLess(g, 1e-4)
        f2, _, g2 = G.pull('cookie', p - [0.001, 0, 0], np.eye(3))
        self.assertAlmostEqual(f2, G.F0 / (1 + g2 / 1e-3) ** 2, places=6)
        self.assertLess(f2, G.F0 / 3.9)
        f3, _, _ = G.pull('cookie', np.array([0.22, 0.0, G.TABLE_TOP + 0.01]), np.eye(3))
        self.assertEqual(f3, 0.0)


if __name__ == '__main__':
    unittest.main()
