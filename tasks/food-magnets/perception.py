"""Camera perception for the food-magnets demo: robot-observable data only.

Inputs: 320 x 176 RGB frames from ``top_cam`` (the overhead D435) and the wrist cameras, the arms' measured joint angles,
the rig's published calibration (yam_arena.station: arm bases, camera poses and intrinsics) and the card's drawings
(board plane, box spots, part sizes). No simulator pose is read.

Method: the board face is a known plane (x = 400 mm, the steel). Food-coloured pixels (saturated: red, tan, pink; the
steel is grey, the note white, the plain magnet black) inside the board's image region are grouped into connected
blobs; each blob's centroid ray is cut with the plane of the magnets' front faces. The box is found among its four
drawn spots as the spot whose image patch differs most from the other three (bare table).
"""
import math

import numpy as np

import geometry as G

CAM_W, CAM_H = 320, 176


# -- camera model (from build/ring-on-finger/perception.py) ---------------------------------------------------
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



def top_camera_model():
    from yam_arena import station
    cam = station.top_camera('calibrated')
    if cam.convention != 'opengl':
        raise ValueError('expected the station top camera in the OpenGL convention')
    return Camera(cam.pos, G.quat_xyzw_to_matrix(cam.rot_xyzw), *lens_at_observation(cam.lens))


# -- masks ----------------------------------------------------------------------------------------------------

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


def food_mask(img):
    """Saturated, not dark: the red, tan and pink PLA. Steel (grey), the note (white) and the plain disc (black) fail."""
    x, v, s = hsv(img)
    return (box3(s, 3) > 0.28) & (box3(v, 3) > 0.25)


def components(mask):
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


def board_region(cam, margin=0.0):
    """Pixels whose ray meets the steel face inside the sheet (less ``margin``)."""
    vv, uu = np.mgrid[0:cam.height, 0:cam.width]
    rays = cam.rays(uu.ravel(), vv.ravel())
    t = (G.FACE_X - cam.pos[0]) / rays[:, 0]
    hit = cam.pos + rays * t[:, None]
    ok = (t > 0) & (np.abs(hit[:, 1]) <= G.SHEET_Y_HALF - margin) & \
         (hit[:, 2] >= G.TABLE_TOP + G.SHEET_Z0 + margin) & (hit[:, 2] <= G.TABLE_TOP + G.SHEET_Z1 - margin)
    return ok.reshape(cam.height, cam.width)


def plane_point(cam, u, v, x_plane):
    ray = cam.rays(np.array([u]), np.array([v]))[0]
    t = (x_plane - cam.pos[0]) / ray[0]
    return cam.pos + ray * t


def find_food(img, cam, exclude=None, min_px=6, front_x=None):
    """Food blobs on the board: list of dict(yz (m, above the table), px, uv), largest first."""
    region = board_region(cam)
    m = food_mask(img) & region
    if exclude is not None:
        m &= ~exclude
    labels, sizes = components(m)
    out = []
    x_plane = G.FACE_X - 0.010 if front_x is None else front_x
    for k in np.argsort(sizes)[::-1]:
        if k == 0 or sizes[k] < min_px:
            continue
        vs, us = np.nonzero(labels == k)
        p = plane_point(cam, us.mean(), vs.mean(), x_plane)
        rgb = np.asarray(img, float)[vs, us].mean(0)
        out.append(dict(yz=(float(p[1]), float(p[2] - G.TABLE_TOP)), px=int(sizes[k]), uv=(float(us.mean()), float(vs.mean())),
                        rgb=rgb.round(1).tolist()))
    return out


def _outline_points(y):
    """The box's top rim outline and the open lid's free edge (world), at drawn spot y."""
    pts = []
    corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        for t in np.linspace(0, 1, 25, endpoint=False):
            sx, sy = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            pts.append((G.BOX_X + sx * G.BOX_LX / 2, y + sy * G.BOX_LY / 2, G.TABLE_TOP + G.BOX_H))
    edge = G.lid_free_edge(np.array([G.HINGE_X, y, G.TABLE_TOP + G.BOX_H]), G.LID_OPEN_ROT)
    for t in np.linspace(-0.45, 0.45, 25):
        pts.append((edge[0], y + t * G.BOX_LY, edge[2]))
    return np.array(pts)


def find_box(img, cam):
    """Index of the box spot: the drawn spot whose projected rim outline and open-lid edge carry the most image edge
    (smoothed luminance gradient). A bare spot is table texture or lamp shadow, with no rectangle of edges there."""
    lum = box3(np.asarray(img, float)[..., :3].mean(-1), 3)
    gy, gx = np.gradient(lum)
    gm = np.hypot(gx, gy)
    scores = []
    for y in G.SPOTS:
        uv, _ = cam.project(_outline_points(y))
        vals = [gm[int(round(v)), int(round(u))] for u, v in uv if 0 <= round(u) < cam.width and 0 <= round(v) < cam.height]
        scores.append(float(np.mean(vals)) if vals else 0.0)
    return int(np.argmax(scores)), [round(v, 2) for v in scores]
