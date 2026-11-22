"""Wrist-camera perception for the demo policy: robot-observable data only.

Inputs: a 320 x 176 RGB frame from ``left_cam``, the left arm's measured joint angles, the rig's published calibration
(arm base position, the D405's mount in the gripper link and its intrinsics: yam_arena.station) and the known part
drawings (ring and hand form sizes, the table height). No simulator pose is read.

Method: colour mask (silicone skin, steel ring), then a model fit: for a candidate (x, y) of the part, the part's
silhouette is ray-cast through the calibrated camera and compared with the mask (intersection over union); a pattern
search refines the candidate from a centroid back-projection. The ring finger is a vertical solid of revolution with
the card's profile; the ring is a flat annulus 4 mm thick.
"""
import math

import numpy as np

import geometry as G

CAM_W, CAM_H = 320, 176
RENDER_W, RENDER_H = 640, 360


# -- camera model -------------------------------------------------------------------------------------------

class Camera:
    """OpenGL-convention pinhole (looks along -z, +y up) with intrinsics at the observation size."""

    def __init__(self, pos, rot, fx, fy, cx, cy, width=CAM_W, height=CAM_H):
        self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)
        self.fx, self.fy, self.cx, self.cy, self.width, self.height = fx, fy, cx, cy, width, height

    def rays(self, us, vs):
        d = np.stack([(np.asarray(us, float) - self.cx) / self.fx, -(np.asarray(vs, float) - self.cy) / self.fy,
                      -np.ones(np.shape(us))], -1)
        d = d @ self.rot.T
        return d / np.linalg.norm(d, axis=-1, keepdims=True)

    def project(self, pts):
        p = (np.asarray(pts, float) - self.pos) @ self.rot
        z = -p[..., 2]
        return np.stack([self.cx + self.fx * p[..., 0] / z, self.cy - self.fy * p[..., 1] / z], -1), z


def lens_at_observation(lens):
    """Calibrated render-size intrinsics (640 x 360) mapped through cv2.resize to 320 x 176 (pixel centres)."""
    sx, sy = CAM_W / lens.width, CAM_H / lens.height
    return lens.fx * sx, lens.fy * sy, (lens.cx + .5) * sx - .5, (lens.cy + .5) * sy - .5


class ArmModel:
    """Left-arm kinematics and wrist camera from the rig calibration (yam_arena.station, URDF)."""

    def __init__(self, side='left'):
        from yam_arena import YAM_URDF, station
        from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
        self.urdf, self.joints, self.tcp_offset = YAM_URDF, ARM_JOINTS, np.asarray(TCP_OFFSET)
        self.base = np.asarray(station.BASES[side], float)
        cam = station.wrist_camera(side)
        self.cam_pos_g = np.asarray(cam.pos, float)
        self.cam_rot_g = G.quat_xyzw_to_matrix(cam.rot_xyzw)
        if cam.convention != 'opengl':
            raise ValueError('expected the station wrist camera in the OpenGL convention')
        self.intrinsics = lens_at_observation(cam.lens)

    def gripper(self, q):
        from yam_arena.urdf import forward_kinematics
        t = forward_kinematics(self.urdf, {n: float(v) for n, v in zip(self.joints, q)})['gripper'].copy()
        t[:3, 3] += self.base
        return t

    def tcp(self, q):
        t = self.gripper(q)
        return t[:3, :3] @ self.tcp_offset + t[:3, 3], t[:3, :3]

    def camera(self, q):
        t = self.gripper(q)
        return Camera(t[:3, :3] @ self.cam_pos_g + t[:3, 3], t[:3, :3] @ self.cam_rot_g, *self.intrinsics)


# -- colour masks -------------------------------------------------------------------------------------------

def hsv(img):
    x = np.asarray(img, float)[..., :3] / 255.0
    mx, mn = x.max(-1), x.min(-1)
    sat = np.where(mx > 1e-6, (mx - mn) / np.maximum(mx, 1e-6), 0)
    return x, mx, sat


def box3(mask, k=3):
    m = np.asarray(mask, float)
    h = k // 2
    p = np.pad(m, h, mode='edge')
    return sum(p[i:i + m.shape[0], j:j + m.shape[1]] for i in range(k) for j in range(k)) / (k * k)


def silicone_mask(img):
    """The ring finger's natural silicone (and its dome): light and nearly grey. PLA skin tone, oak and the dark
    plate fail it; the ring passes too, so callers exclude the ring's own pixels."""
    x, v, s = hsv(img)
    v, s = box3(v, 5), box3(s, 5)          # the RTX frames are speckled: judge 5 x 5 means
    return (s < 0.08) & (v > 0.60)


def steel_mask(img):
    x, v, s = hsv(img)
    v, s = box3(v, 5), box3(s, 5)
    # job 4292803 frames: ring s ~0.10, v 0.46-0.75; station table s >= 0.165, v ~0.84; plate v ~0.44
    return (s < 0.14) & (v > 0.42) & (v < 0.80)


def components(mask):
    """Connected components (4-neighbour) as a label image and sizes, no SciPy."""
    h, w = mask.shape
    labels = np.zeros((h, w), int)
    sizes = [0]
    cur = 0
    for y0 in range(h):
        for x0 in range(w):
            if mask[y0, x0] and not labels[y0, x0]:
                cur += 1
                stack = [(y0, x0)]
                labels[y0, x0] = cur
                n = 0
                while stack:
                    y, x = stack.pop()
                    n += 1
                    for yy, xx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                        if 0 <= yy < h and 0 <= xx < w and mask[yy, xx] and not labels[yy, xx]:
                            labels[yy, xx] = cur
                            stack.append((yy, xx))
                sizes.append(n)
    return labels, np.asarray(sizes)


# -- silhouettes --------------------------------------------------------------------------------------------

_U_TABLE = np.linspace(0.0, 72.0, 1441)
_R_TABLE = G.finger_outer_r_mm(_U_TABLE) * G.MM
_LINES = [tuple(v) for k, v in G.RF['lines_u_mm'].items() if k != 'width']


def _plane_t(cam_pos, rays, z):
    return (z - cam_pos[2]) / rays[..., 2]


def finger_silhouette(cam, us, vs, axis_xy, web_z, samples=170):
    """Per pixel: 1 = ray hits the ring finger's silicone (skin or dome), 0 = misses."""
    rays = cam.rays(us, vs)
    t_top = _plane_t(cam.pos, rays, web_z + 72.5 * G.MM)
    t_bot = _plane_t(cam.pos, rays, web_z)
    t0, t1 = np.minimum(t_top, t_bot), np.maximum(t_top, t_bot)
    s = np.linspace(0, 1, samples)
    t = t0[:, None] + (t1 - t0)[:, None] * s[None]
    p = cam.pos + rays[:, None, :] * t[..., None]
    rho = np.hypot(p[..., 0] - axis_xy[0], p[..., 1] - axis_xy[1])
    u = (p[..., 2] - web_z) / G.MM
    inside = (u >= 0) & (u <= 72.0) & (rho <= np.interp(u, _U_TABLE, _R_TABLE))
    hit = inside.any(1)
    first = np.argmax(inside, 1)
    uh = u[np.arange(len(u)), first]
    return hit   # the painted lines are not drawn in sim (native.py)


def ring_silhouette(cam, us, vs, centre, axis=(0., 0., 1.), samples=24):
    """Per pixel: 1 = ray hits a ring (annulus ID/OD, width 4 mm) with ``centre`` and band ``axis`` (world)."""
    rays = cam.rays(us, vs)
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    c = np.asarray(centre, float)
    # sample the ray where it crosses the band's slab |(p - c).a| <= w/2 (+ the OD's tilt allowance)
    denom = rays @ a
    denom = np.where(np.abs(denom) < 1e-6, 1e-6, denom)
    half = G.RING_W / 2 + 1e-4
    ta = ((c - cam.pos) @ a - half) / denom
    tb = ((c - cam.pos) @ a + half) / denom
    t0, t1 = np.minimum(ta, tb), np.maximum(ta, tb)
    s = np.linspace(0, 1, samples)
    t = t0[:, None] + (t1 - t0)[:, None] * s[None]
    p = cam.pos + rays[:, None, :] * t[..., None] - c
    along = p @ a
    rho = np.linalg.norm(p - along[..., None] * a, axis=-1)
    inside = (np.abs(along) <= G.RING_W / 2) & (rho >= G.RING_ID / 2) & (rho <= G.RING_OD / 2)
    return inside.any(1)


def _iou(pred, obs):
    inter = np.logical_and(pred, obs).sum()
    union = np.logical_or(pred, obs).sum()
    return inter / union if union else 0.0


def pattern_search(f, x0, steps=(2e-3, 1e-3, 5e-4, 2.5e-4, 1.25e-4)):
    x = np.asarray(x0, float)
    best = f(x)
    evals = 1
    for st in steps:
        improved = True
        while improved:
            improved = False
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                cand = x + st * np.array([dx, dy])
                val = f(cand)
                evals += 1
                if val > best + 1e-9:
                    best, x, improved = val, cand, True
            if evals > 400:
                break
    return x, best, evals


def roi_pixels(cam, centre_xy, z_lo, z_hi, radius, pad=6):
    """Pixel grid of the image box covering a vertical cylinder (centre, radius, z range)."""
    ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
    pts = np.array([[centre_xy[0] + radius * math.cos(a), centre_xy[1] + radius * math.sin(a), z]
                    for a in ang for z in (z_lo, z_hi)])
    uv, depth = cam.project(pts)
    if (depth <= 0).any():
        return None
    u0, v0 = np.floor(uv.min(0)) - pad
    u1, v1 = np.ceil(uv.max(0)) + pad
    u0, v0 = max(0, int(u0)), max(0, int(v0))
    u1, v1 = min(cam.width - 1, int(u1)), min(cam.height - 1, int(v1))
    if u1 <= u0 or v1 <= v0:
        return None
    vv, uu = np.mgrid[v0:v1 + 1, u0:u1 + 1]
    return uu.ravel(), vv.ravel()


def _seed_from_mask(cam, mask, uu, vv, z):
    sel = mask[vv, uu]
    if sel.sum() < 6:
        return None
    rays = cam.rays(uu[sel], vv[sel])
    t = _plane_t(cam.pos, rays, z)
    p = cam.pos + rays * t[:, None]
    return np.median(p[:, :2], 0)


def locate_finger(img, cam, guess_xy, web_z, search_radius=0.03, exclude=None):
    """Fit the ring finger's axis (x, y) to the silicone mask around ``guess_xy``. Returns dict or None."""
    roi = roi_pixels(cam, guess_xy, web_z, web_z + 0.073, search_radius + 0.01)
    if roi is None:
        return None
    uu, vv = roi
    mask = silicone_mask(img)
    if exclude is not None:          # occluded pixels (held ring, pads) leave the comparison altogether
        keep = ~exclude[vv, uu]
        uu, vv = uu[keep], vv[keep]
        mask = mask & ~exclude
    obs = mask[vv, uu]
    seed = _seed_from_mask(cam, mask, uu, vv, web_z + 0.050)
    if seed is None:
        return dict(ok=False, reason='no silicone pixels', pixels=int(obs.sum()))
    if np.hypot(*(seed - guess_xy)) > search_radius:
        seed = np.asarray(guess_xy, float)
    xy, iou, evals = pattern_search(lambda c: _iou(finger_silhouette(cam, uu, vv, c, web_z), obs), seed,
                                    steps=(4e-3, 2e-3, 1e-3, 5e-4, 2.5e-4, 1.25e-4))
    return dict(ok=bool(iou > 0.4), xy=xy, iou=float(iou), seed=seed, evals=evals, pixels=int(obs.sum()))


def locate_flat_ring(img, cam, guess_xy, table_z, search_radius=0.035, exclude=None):
    """Fit a flat ring's centre (x, y) on the table to the steel mask around ``guess_xy``."""
    roi = roi_pixels(cam, guess_xy, table_z, table_z + G.RING_W, search_radius + 0.012)
    if roi is None:
        return None
    uu, vv = roi
    mask = steel_mask(img)
    if exclude is not None:
        mask = mask & ~exclude
    obs = mask[vv, uu]
    seed = _seed_from_mask(cam, mask, uu, vv, table_z + G.RING_W)
    if seed is None:
        return dict(ok=False, reason='no steel pixels', pixels=int(obs.sum()))
    if np.hypot(*(seed - guess_xy)) > search_radius:
        seed = np.asarray(guess_xy, float)
    zc = table_z + G.RING_W / 2
    xy, iou, evals = pattern_search(lambda c: _iou(ring_silhouette(cam, uu, vv, (c[0], c[1], zc)), obs), seed,
                                    steps=(4e-3, 2e-3, 1e-3, 5e-4, 2.5e-4, 1.25e-4))
    return dict(ok=bool(iou > 0.5), xy=xy, iou=float(iou), seed=seed, evals=evals, pixels=int(obs.sum()))


def dark_mask(img):
    x, v, s = hsv(img)
    return box3(v, 5) < 0.30          # the black pads and gripper


def locate_held_ring(img, cam, tcp, rot, along, guess=(0.0, 0.0)):
    """Fit the held ring's centre offset in the gripper's x-y plane (m), its band axis vertical (picked flat) and its
    centre ``along`` the approach axis from the TCP. Dark pixels (pads) leave the comparison. Returns a dict."""
    centre0 = tcp + rot @ np.array([guess[0], guess[1], along])
    roi = roi_pixels(cam, centre0[:2], centre0[2] - 0.004, centre0[2] + 0.004, 0.016)
    if roi is None:
        return dict(ok=False, reason='ring outside the image')
    uu, vv = roi
    keep = ~dark_mask(img)[vv, uu]
    uu, vv = uu[keep], vv[keep]
    obs = steel_mask(img)[vv, uu]
    if obs.sum() < 6:
        return dict(ok=False, reason='no steel pixels', pixels=int(obs.sum()))
    f = lambda c: _iou(ring_silhouette(cam, uu, vv, tcp + rot @ np.array([c[0], c[1], along]), (0.0, 0.0, 1.0)), obs)
    xy, iou, evals = pattern_search(f, np.asarray(guess, float), steps=(2e-3, 1e-3, 5e-4, 2.5e-4, 1.25e-4))
    return dict(ok=bool(iou > 0.4), offset=xy, iou=float(iou), evals=evals, pixels=int(obs.sum()))


def held_ring_pixels(cam, centre, grow=2):
    """Image pixels covered by a level ring at ``centre`` (world), grown by ``grow`` px."""
    vv, uu = np.mgrid[0:cam.height, 0:cam.width]
    hit = ring_silhouette(cam, uu.ravel(), vv.ravel(), centre, (0.0, 0.0, 1.0)).reshape(cam.height, cam.width)
    for _ in range(grow):
        hit = box3(hit) > 0
    return hit
