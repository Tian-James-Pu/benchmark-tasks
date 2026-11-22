"""Demo-policy tests without the simulator: synthetic wrist-camera frames ray-cast through the calibrated camera model
(perception's own silhouettes), perfect joint tracking. Checks that the fits recover the drawn positions, that every
planned pose is reachable by URDF IK, and that the policy reads nothing but images and joint readings.
Needs yam_arena importable (pure-Python parts: URDF, IK, station calibration); skipped otherwise."""
import math
import unittest
from types import SimpleNamespace

import numpy as np

import geometry as G

try:
    import yam_arena  # noqa: F401
    HAVE_YAM = True
except ImportError:
    HAVE_YAM = False

OAK = np.array([205, 192, 164], np.uint8)     # the station table as rendered (job 4292803 frames)
SILICONE = np.array([236, 232, 224], np.uint8)
PLA = np.array([219, 173, 143], np.uint8)
STEEL = np.array([170, 172, 176], np.uint8)   # the ring as rendered (s ~0.1, v ~0.67)


def render(cam, hand_xy, ring_pose):
    """Synthetic left_cam frame: oak table, PLA palm block, silicone ring finger, steel ring. ring_pose = (centre, axis)."""
    import perception as V
    vv, uu = np.mgrid[0:cam.height, 0:cam.width]
    us, vs = uu.ravel(), vv.ravel()
    img = np.tile(OAK, (len(us), 1))
    web = G.TABLE_TOP + G.WEB_Z
    rays = cam.rays(us, vs)
    t = (web - cam.pos[2]) / rays[:, 2]
    hit = cam.pos + rays * t[:, None]
    palm = (np.abs(hit[:, 0] - hand_xy[0]) < 0.014) & (np.abs(hit[:, 1] - (hand_xy[1] - 0.012)) < 0.04)
    img[palm] = PLA
    img[V.finger_silhouette(cam, us, vs, hand_xy, web)] = SILICONE
    if ring_pose is not None:
        img[V.ring_silhouette(cam, us, vs, *ring_pose)] = STEEL
    return img.reshape(cam.height, cam.width, 3)


@unittest.skipUnless(HAVE_YAM, 'needs yam_arena (URDF, IK, station calibration)')
class Fits(unittest.TestCase):
    def setUp(self):
        import demo_policy as D
        import perception as V
        self.D, self.V = D, V
        self.arm = V.ArmModel('left')
        self.q0 = np.array([-0.7, 1.8, 1.5, -1.2, 0.0, 0.9])

    def cam_at(self, tcp):
        from yam_arena.ik import solve
        q, err, _ = solve(tcp, self.D.Q_GRIP, q_init=self.q0, base_pos=self.arm.base, iters=300)
        self.assertLess(err, 1e-3)
        return self.arm.camera(q)

    def test_finger_fit(self):
        web = G.TABLE_TOP + G.WEB_Z
        for dx, dy in ((0, 0), (15, 1), (-15, 15), (15, -15)):
            true = np.array(G.HAND_AXIS_XY) + [dx * G.MM, dy * G.MM]
            policy = self.D.RingDemo()
            cam = self.cam_at(policy._survey_tcp((*G.HAND_AXIS_XY, web + 0.060)))
            img = render(cam, true, None)
            fit = self.V.locate_finger(img, cam, np.array(G.HAND_AXIS_XY), web)
            with self.subTest((dx, dy)):
                self.assertTrue(fit['ok'], fit)
                self.assertLess(np.linalg.norm(fit['xy'] - true), 0.6 * G.MM, fit)

    def test_flat_ring_fit(self):
        for dx, dy in ((0, 0), (-9, -3), (20, -20), (-20, 20)):
            true = np.array(G.RING_START_XY) + [dx * G.MM, dy * G.MM]
            policy = self.D.RingDemo()
            cam = self.cam_at(policy._survey_tcp((*G.RING_START_XY, G.TABLE_TOP + G.RING_W)))
            img = render(cam, (1.0, 1.0), ((*true, G.TABLE_TOP + G.RING_W / 2), (0, 0, 1)))
            fit = self.V.locate_flat_ring(img, cam, np.array(G.RING_START_XY), G.TABLE_TOP)
            with self.subTest((dx, dy)):
                self.assertTrue(fit['ok'], fit)
                self.assertLess(np.linalg.norm(fit['xy'] - true), 0.6 * G.MM, fit)

    def test_pick_geometry(self):
        z, ring = self.D.PICK_TCP_Z, self.D.RING_IN_GRIPPER
        r = self.D.R_GRIP
        centre = np.array([0, 0, z]) + r @ ring
        self.assertAlmostEqual(centre[2], G.RING_W / 2, places=9)
        low = min((r @ np.array([0, y, self.D.PAD_TIP_Z]))[2] + z for y in (-0.0077, 0.0077))
        self.assertAlmostEqual(low, self.D.PAD_LOW_PICK, places=9)
        self.assertTrue(np.allclose(r[:, 0], (-1, 0, 0)))   # jaws close along world x


class _Lazy:
    """A frame rendered only when the policy looks at it."""

    def __init__(self, make):
        self.make = make

    def __array__(self, dtype=None, copy=None):
        a = self.make()
        return a if dtype is None else a.astype(dtype)


@unittest.skipUnless(HAVE_YAM, 'needs yam_arena (URDF, IK, station calibration)')
class DryRun(unittest.TestCase):
    """The whole demo on perfect joint tracking and synthetic frames: phases complete, IK stays exact."""

    def run_policy(self, seed, mode='demo', miss_first=False):
        import demo_policy as D
        import scene
        pdx, pdy, rdx, rdy = scene.draws(seed)
        hand_xy = np.array(G.HAND_AXIS_XY) + [pdx * G.MM, pdy * G.MM]
        ring_xy = np.array(G.RING_START_XY) + [rdx * G.MM, rdy * G.MM]
        policy = D.RingDemo(mode=mode)
        q = np.r_[D.READY, 1.0, D.READY, 1.0]
        held = False
        steps = 0
        while not policy.done and steps < 2400:
            cam = policy.arm.camera(q[:6])
            ring_pose = None if held else ((*ring_xy, G.TABLE_TOP + G.RING_W / 2), (0, 0, 1))
            img = _Lazy(lambda cam=cam, ring_pose=ring_pose: render(cam, hand_xy, ring_pose))
            obs = SimpleNamespace(images={'left_cam': img}, state={'joint_pos': q.copy()})
            q = np.asarray(policy.act(obs), float)
            name = policy.log[-1].name
            if q[6] < 0.3 and (name.startswith('repick') or (name == 'pick' and not miss_first)):
                held = True
            if held and q[6] < 0.205:
                q[6] = 0.205          # the band stops the jaws (an empty pinch reads the 0.144 command)
            steps += 1
        return policy, steps, hand_xy, ring_xy

    def test_demo_seed1(self):
        policy, steps, hand_xy, ring_xy = self.run_policy(1)
        rep = policy.report()
        names = [p['name'] for p in rep['phases']]
        for n in ('survey_hand', 'survey_ring', 'pick', 'over_tip', 'thread', 'release', 'retreat', 'home', 'final_hold'):
            pass
        for n in ('survey_hand', 'survey_ring', 'pick', 'over_tip', 'thread', 'release', 'retreat', 'home', 'final_hold'):
            self.assertIn(n, names)
        self.assertLess(steps, 1200)
        self.assertLess(np.linalg.norm(policy.finger_xy - hand_xy), 1.5 * G.MM)   # synthetic frames carry no fit bias; the demo corrects ~1 mm
        self.assertLess(np.linalg.norm(policy.ring_xy - ring_xy), 1.0 * G.MM)
        for p in rep['phases']:
            with self.subTest(p['name']):
                self.assertLess(p.get('worst_ik_mm', 0.0), 1.0, p)
                self.assertLess(p.get('worst_ik_deg', 0.0), 0.5, p)

    def test_repick_after_empty_pinch(self):
        policy, steps, _, _ = self.run_policy(1, miss_first=True)
        names = [p['name'] for p in policy.report()['phases']]
        self.assertIn('repick1', names)
        self.assertIn('final_hold', names)

    def test_hover_holds(self):
        policy, steps, _, _ = self.run_policy(2, mode='hover')
        self.assertEqual(policy.report()['phases'][-1]['name'], 'hover_hold')

    def test_reads_only_images_and_joints(self):
        import inspect
        import demo_policy as D
        src = inspect.getsource(D)
        for forbidden in ('arena_ring', 'object_states', 'root_link_pose', 'nodal_pos', 'scene[', '_env', 'TRIAL'):
            self.assertNotIn(forbidden, src)


if __name__ == '__main__':
    unittest.main()
