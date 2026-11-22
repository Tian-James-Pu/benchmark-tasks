"""Geometry tests (CPU, NumPy): the skin mesh, the ring pieces and the hand form match the card's numbers and start
without overlaps. python -m unittest test_geometry -v"""
import math
import unittest

import numpy as np

import geometry as G


class Skin(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = G.skin_tets()

    def test_positive_and_about_1mm(self):
        v = self.m.volumes()
        self.assertTrue((v > 0).all())
        edges = np.concatenate([self.m.points[self.m.tets[:, a]] - self.m.points[self.m.tets[:, b]]
                                for a, b in ((0, 1), (1, 2), (2, 3))])
        lengths = np.linalg.norm(edges, axis=1) / G.MM
        self.assertGreater(v.min(), 1.2e-9)   # PhysX: tets under 1 mm^3 are 'degenerate'
        self.assertLess(lengths.max(), 6.0)

    def test_closed_boundary(self):
        f = self.m.surface
        e = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
        _, counts = np.unique(e, axis=0, return_counts=True)
        self.assertTrue((counts == 2).all())

    def test_profile(self):
        outer = self.m.points[self.m.node_outer]
        r = np.hypot(outer[:, 0], outer[:, 1]) / G.MM
        u = self.m.node_u[self.m.node_outer]
        crest = r[np.isclose(u, 28.0, atol=0.6)]
        self.assertAlmostEqual(crest.max() * 2 * math.cos(math.pi / G.SKIN_AROUND), 18.0, delta=0.01)
        self.assertAlmostEqual(r[np.isclose(u, 4.0, atol=0.6)].max() * 2 * math.cos(math.pi / G.SKIN_AROUND), 17.0, delta=0.01)
        dip = r[np.isclose(u, 52.0, atol=0.6)]
        self.assertAlmostEqual(dip.max() * 2 * math.cos(math.pi / G.SKIN_AROUND), 16.4, delta=0.01)
        # the polygon's flats lose < 0.01 mm of the crest radius at 48 cells round
        self.assertLess(9.0 * (1 / math.cos(math.pi / G.SKIN_AROUND) - 1), 0.08)  # polygon vertices at most 0.08 mm proud

    def test_interference_at_pip(self):
        self.assertAlmostEqual((18.0 - G.RING['id_mm']) / 2, 0.125)
        self.assertGreater(G.RING_ID / 2, G.finger_outer_r_mm(np.array([5.0, 52.0, 60.0, 64.5])).max() * G.MM)

    def test_clear_of_bone_cap_and_palm(self):
        pts = self.m.points
        for prim in G.hand_prims():
            d = G.prim_signed_distance(pts, prim)
            with self.subTest(prim.name):
                self.assertGreater(d.min(), 0.0)
        self.assertGreater(G.prim_signed_distance(pts, next(p for p in G.hand_prims() if p.name == 'bone')).min(), 0.05 * G.MM)


class RingAndHand(unittest.TestCase):
    def test_ring_dims(self):
        segs = G.ring_segments()
        pts = np.concatenate([p for p, _ in segs])
        rho = np.hypot(pts[:, 0], pts[:, 1])
        self.assertAlmostEqual(rho.min() * 2 / G.MM, 17.75, places=6)
        self.assertAlmostEqual(rho.max() * 2 / G.MM, 21.55, places=6)
        self.assertAlmostEqual((pts[:, 2].max() - pts[:, 2].min()) / G.MM, 4.0, places=6)
        # inscribed polygon: the bore between piece corners is at most 0.02 mm smaller than the round bore
        sag = G.RING_ID / 2 * (1 - math.cos(math.pi / G.RING_SEGMENTS)) / G.MM
        self.assertLess(sag, 0.02)
        self.assertEqual(len(segs), 48)

    def test_ring_mass_from_params(self):
        sec = G.ring_section(8)
        x, z = sec[:, 0], sec[:, 1]
        area = 0.5 * abs(np.dot(x, np.roll(z, 1)) - np.dot(z, np.roll(x, 1)))
        volume = 2 * math.pi * x.mean() * area / G.MM ** 3
        self.assertAlmostEqual(volume * 7.98e-3, G.RING['mass_g'], delta=0.15)  # the 0.5 mm edge radii remove ~3%; the sim sets the mass to 3.74 g

    def test_neighbours_clear_seated_ring(self):
        ring = G.ring_surface_points() + [0, 0, G.WEB_Z + G.RING_W / 2]
        for prim in G.hand_prims():
            if prim.name in ('middle', 'little', 'index', 'thumb'):
                with self.subTest(prim.name):
                    self.assertGreater(G.prim_signed_distance(ring, prim).min(), 0.0007)

    def test_palm_top_is_web(self):
        palm = next(p for p in G.hand_prims() if p.name == 'palm')
        self.assertAlmostEqual(palm.centre[2] + palm.size[2] / 2, G.WEB_Z, places=9)

    def test_readings_flat_ring(self):
        r = G.ring_readings([0.2, 0.16, G.TABLE_TOP + 0.002], np.eye(3), [0.3, 0, G.TABLE_TOP], np.eye(3))
        self.assertAlmostEqual(r['ring_lowest_mm'], 0.0, places=6)
        self.assertAlmostEqual(r['ring_offset_mm'], math.hypot(100, 160), places=6)


if __name__ == '__main__':
    unittest.main()
