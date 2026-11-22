"""Demo-policy tests on synthetic frames (ray-cast flat colours of the drawn scene through the calibrated cameras):
the survey finds the three food magnets and the box spot, and a dry run with ideal joint tracking reaches every phase
with small IK errors inside the card's run limit."""
import types
import unittest

import numpy as np

import geometry as G
import perception as V
import scene

TABLE_RGB = np.array([150, 110, 75], float)


def render(cam, d, held=()):
    vv, uu = np.mgrid[0:cam.height, 0:cam.width]
    rays = cam.rays(uu.ravel(), vv.ravel())
    n = len(rays)
    img = np.tile(np.array([200, 210, 215], float), (n, 1))
    depth = np.full(n, np.inf)

    def paint(t, ok, rgb):
        nonlocal depth
        m = ok & (t > 0) & (t < depth)
        img[m] = rgb
        depth = np.where(m, t, depth)

    t = (G.TABLE_TOP - cam.pos[2]) / rays[:, 2]
    paint(t, np.ones(n, bool), TABLE_RGB)
    t = (G.FACE_X - cam.pos[0]) / rays[:, 0]
    hit = cam.pos + rays * t[:, None]
    paint(t, (np.abs(hit[:, 1]) <= G.SHEET_Y_HALF) & (hit[:, 2] > G.TABLE_TOP + G.SHEET_Z0) & (hit[:, 2] < G.TABLE_TOP + G.SHEET_Z1),
          np.array(scene.COLORS['sheet']) * 255)
    y, z = d['plain']
    cz = G.TABLE_TOP + z - G.CARD_BELOW_MAGNET
    paint(t - 1e-4, (np.abs(hit[:, 1] - y) <= G.CARD_W / 2) & (np.abs(hit[:, 2] - cz) <= G.CARD_H / 2), np.array(scene.COLORS['card']) * 255)
    for name in ('plain',) + G.NAMES:
        if name in held:
            continue
        y, z = d[name]
        w, hh = G.EXTENT[name]
        back = G.FACE_X - (G.CARD_T if name == 'plain' else 0)
        for x in np.linspace(back - G.THICK[name], back - 0.0005, 5):   # stacked sections: the prism's silhouette
            tt = (x - cam.pos[0]) / rays[:, 0]
            h = cam.pos + rays * tt[:, None]
            ok = ((h[:, 1] - y) / (w / 2)) ** 2 + ((h[:, 2] - G.TABLE_TOP - z) / (hh / 2)) ** 2 <= 1
            paint(tt, ok, np.array(scene.COLORS[name]) * 255)
    tt = (G.TABLE_TOP + G.BOX_H - cam.pos[2]) / rays[:, 2]
    h = cam.pos + rays * tt[:, None]
    paint(tt, (np.abs(h[:, 0] - G.BOX_X) <= G.BOX_LX / 2) & (np.abs(h[:, 1] - d['box_y']) <= G.BOX_LY / 2),
          np.array(scene.COLORS['box']) * 255 * 0.8)
    return img.reshape(cam.height, cam.width, 3).clip(0, 255).astype(np.uint8)


class Survey(unittest.TestCase):
    def test_top_camera_survey(self):
        cam = V.top_camera_model()
        for seed in (1, 2, 3, 4, 5, 6, 7, 8):
            d = scene.draw(seed)
            img = render(cam, d)
            blobs = V.find_food(img, cam, min_px=8)
            self.assertGreaterEqual(len(blobs), 3, seed)
            for n in G.NAMES:
                err = min(np.linalg.norm(np.array(b['yz']) - np.array(d[n])) for b in blobs[:3])
                self.assertLess(err, 0.012, (seed, n, err))
            spot, _ = V.find_box(img, cam)
            self.assertEqual(G.SPOTS[spot], d['box_y'], seed)


def dry_run(seed, mode='demo', max_steps=2400):
    from demo_policy import make_policy
    d = scene.draw(seed)
    pol = make_policy(320, 176, 20, mode=mode)
    top = V.top_camera_model()
    arms = {s: V.ArmModel(s) for s in ('left', 'right')}
    act = np.r_[pol.q_cmd['left'], 1.0, pol.q_cmd['right'], 1.0]
    picked = set()
    for step in range(max_steps):
        joints = act.copy()
        phase = pol.log[-1].name if pol.log else ''
        for gi in (6, 13):
            if phase.startswith(('pinch', 'pull', 'drop', 'hover')) and act[gi] < 0.5:
                joints[gi] = max(act[gi], 0.42)
        imgs = dict(top_cam=render(top, d, held=picked) if step < 6 else np.zeros((176, 320, 3), np.uint8))
        for s, key in (('left', 'left_cam'), ('right', 'right_cam')):
            q = act[:6] if s == 'left' else act[7:13]
            imgs[key] = render(arms[s].camera(q), d, held=picked) if phase.startswith('approach') else np.zeros((176, 320, 3), np.uint8)
        obs = types.SimpleNamespace(images=imgs, state={'joint_pos': joints})
        act = np.asarray(pol.act(obs), float)
        if phase.startswith('drop'):
            k = int(phase[4]) - 1
            picked.add(sorted(G.NAMES, key=lambda n: abs(d[n][0] - arms[pol.side].base[1]))[k])
        if pol.done:
            break
    return pol, step + 1


class DryRun(unittest.TestCase):
    def test_demo_dry_run(self):
        for seed in (1, 2):
            pol, steps = dry_run(seed)
            rep = pol.report()
            names = [p['name'] for p in rep['phases']]
            self.assertTrue(pol.done, names)
            self.assertIn('lid', names)
            self.assertEqual(names[-1], 'final_hold')
            self.assertLess(steps, 2400)
            worst = max(p.get('worst_ik_mm', 0) for p in rep['phases'])
            self.assertLess(worst, 1.0, [(p['name'], p.get('worst_ik_mm')) for p in rep['phases']])
            print('seed', seed, 'steps', steps, 'arm', rep['arm'], 'worst ik mm', worst)

    def test_hover_dry_run(self):
        pol, steps = dry_run(1, 'hover')
        names = [p['name'] for p in pol.report()['phases']]
        self.assertEqual(names[-1], 'hover_hold')
        self.assertNotIn('lid', names)


if __name__ == '__main__':
    unittest.main()
