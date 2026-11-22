"""ring-on-finger geometry, pure NumPy (no Isaac). Every number comes from params.json (the card's).

Frames
- World (yam_arena station): x forward, y left, z up, metres; arm bases at (0, +-0.305); table top z 0.7475.
- Hand frame H: origin on the ring finger's axis at the table top, axes parallel to the world at rest.
  u (mm) = distance along the ring finger from its palm-side web = 1000 * z_H - web_z_mm.
- Ring frame R: origin at the band centre, +z along the band axis, the band spans z in [-w/2, +w/2].

Contents: the hand form's rigid primitives (fab/hand_form.scad, as params.json lists them), the ring finger's
silicone skin as a structured tetrahedral mesh (about 1 mm cells), the ring as convex segments with 0.5 mm edge
radii, surface sample points for the release distance, and the scorer's ring readings (u of the band edges).
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
P = json.loads((HERE / 'params.json').read_text())
MM = 1e-3
TABLE_TOP = P['frame']['table_world_z_m']           # 0.7475, = yam_arena.task_file.TABLE_TOP['station']
HAND = P['hand']
RF = HAND['ring_finger']
RING = P['ring']
WEB_Z = HAND['web_z_mm'] * MM                         # 0.130 m above the table
HAND_AXIS_XY = tuple(v * MM for v in HAND['axis_xy_mm'])  # (0.300, 0.000)
RING_START_XY = tuple(v * MM for v in P['ring_start']['xy_mm'])  # (0.200, 0.160)

# -- the ring -----------------------------------------------------------------------------------------------
RING_ID = RING['id_mm'] * MM
RING_OD = RING['od_mm'] * MM
RING_W = RING['width_mm'] * MM
RING_EDGE_R = RING['edge_radius_mm'] * MM
RING_MASS = RING['mass_g'] * 1e-3
RING_SEGMENTS = 48   # convex pieces round the band; the polygon's inner radius varies by 0.019 mm (see test)

# -- the skin (FEM) -----------------------------------------------------------------------------------------
BONE_R = RF['bone_dia_mm'] / 2 * MM                   # 4.8 mm: the skin's bore
BONE_COLLIDER_R = BONE_R - 0.10 * MM                  # rigid bone 0.1 mm under the bore (the bore's flats sit at 4.76)
TIP_R = RF['tip_dome_radius_mm'] * MM                 # 7.5 mm dome
TIP_CENTRE_U = RF['profile_u_mm'][-1]                 # 64.5: dome centre (the profile's last station)
SKIN_TOP_U = 56.0                                     # skin ends under the rigid dome cap (see docs below)
# Cell size. PhysX rejects tetrahedra under 1 mm^3 ('degenerate or inverted', job 4292803: the 1 mm skin of 0.08-0.13 mm^3
# tets had no elasticity and collapsed onto the palm), so the cells are ~2 x 1.5-2 x 4 mm (tets >= 1.2 mm^3).
SKIN_RADIAL = 2                                       # cells through the 3.7-4.2 mm wall
SKIN_AROUND = 24                                      # cells round
SKIN_ALONG = 14                                       # cells along u 0..56 (4 mm; nodes at the PIP crest 28 and DIP crest 52)
POLY_COMP = 1.0 / math.cos(math.pi / SKIN_AROUND)     # outer nodes pushed out so the polygon's flats sit on the profile
SKIN_LIFT = 0.05 * MM                                 # authored 0.05 mm above the palm top (no start overlap)


def profile_d_mm(u_mm):
    """Skin outer diameter (mm) at u (mm), linear between the card's stations; the dome above 64.5."""
    u = np.asarray(u_mm, float)
    d = np.interp(u, RF['profile_u_mm'], RF['profile_d_mm'])
    dome = u > TIP_CENTRE_U
    if np.any(dome):
        h = np.clip(u - TIP_CENTRE_U, 0, RF['tip_dome_radius_mm'])
        d = np.where(dome, 2 * np.sqrt(np.maximum(RF['tip_dome_radius_mm'] ** 2 - h ** 2, 0)), d)
    return d


def finger_outer_r_mm(u_mm):
    """Outer radius of the simulated ring finger (skin below SKIN_TOP_U, rigid dome cap above it)."""
    u = np.asarray(u_mm, float)
    skin = profile_d_mm(u) / 2
    cap = np.sqrt(np.maximum(TIP_R ** 2 - (u * MM - TIP_CENTRE_U * MM) ** 2, 0)) / MM
    return np.where(u <= SKIN_TOP_U, skin, np.maximum(cap, np.where(u <= TIP_CENTRE_U, BONE_COLLIDER_R / MM, 0)))


@dataclass
class TetMesh:
    points: np.ndarray      # (n, 3) hand frame, m
    tets: np.ndarray        # (m, 4)
    surface: np.ndarray     # (f, 3) boundary triangles, outward
    grid: tuple             # (radial, around, along) cell counts
    node_u: np.ndarray      # u (mm) of each node in the rest shape
    node_outer: np.ndarray  # bool: node on the outer skin surface

    def volumes(self):
        a, b, c, d = (self.points[self.tets[:, i]] for i in range(4))
        return np.einsum('ij,ij->i', np.cross(b - a, c - a), d - a) / 6

    def volume(self):
        return float(self.volumes().sum())


# Kuhn (Freudenthal) split of a unit cube into 6 tets along its 0-7 diagonal; corners indexed by bits (i, j, k).
_KUHN = [(0, 1, 3, 7), (0, 1, 5, 7), (0, 2, 3, 7), (0, 2, 6, 7), (0, 4, 5, 7), (0, 4, 6, 7)]


def skin_tets(radial=SKIN_RADIAL, around=SKIN_AROUND, along=SKIN_ALONG, top_u=SKIN_TOP_U):
    """The ring finger's cast skin as a structured tet mesh in the hand frame (metres).

    A tube from the palm top (u 0) to u 58.5: bore = the 9.6 mm bone, outer surface = the card's profile
    (17.0 base, 18.0 PIP crest at 28, 16.4 DIP crest at 52). Above u 58.5 the 7.5 mm tip dome is a rigid part of the
    hand form (simulation simplification: the ring clears the dome by 1.375 mm radially, so the dome's softness
    only matters to a ring that lands on it), which also caps the skin so it cannot ride up the bone."""
    us = np.linspace(0.0, top_u, along + 1)
    th = np.linspace(0.0, 2 * np.pi, around, endpoint=False)
    rs_frac = np.linspace(0.0, 1.0, radial + 1)
    r_out = profile_d_mm(us) / 2 * MM
    idx = np.zeros((radial + 1, around, along + 1), int)
    pts, node_u, outer = [], [], []
    n = 0
    for i, f in enumerate(rs_frac):
        for j, t in enumerate(th):
            for k, u in enumerate(us):
                r = BONE_R + (r_out[k] * POLY_COMP - BONE_R) * f
                pts.append((r * math.cos(t), r * math.sin(t), WEB_Z + u * MM + SKIN_LIFT))
                node_u.append(u)
                outer.append(i == radial)
                idx[i, j, k] = n
                n += 1
    pts = np.asarray(pts)
    tets = []
    for i in range(radial):
        for j in range(around):
            jn = (j + 1) % around
            for k in range(along):
                corner = {}
                for bit in range(8):
                    di, dj, dk = bit & 1, (bit >> 1) & 1, (bit >> 2) & 1
                    corner[bit] = idx[i + di, jn if dj else j, k + dk]
                for t in _KUHN:
                    tets.append([corner[b] for b in t])
    tets = np.asarray(tets, int)
    a, b, c, d = (pts[tets[:, i]] for i in range(4))
    vol = np.einsum('ij,ij->i', np.cross(b - a, c - a), d - a)
    flip = vol < 0
    tets[flip] = tets[flip][:, [0, 2, 1, 3]]
    surface = _boundary(tets, pts)
    return TetMesh(pts, tets, surface, (radial, around, along), np.asarray(node_u), np.asarray(outer))


def _boundary(tets, pts):
    faces = np.concatenate([tets[:, [1, 2, 3]], tets[:, [0, 3, 2]], tets[:, [0, 1, 3]], tets[:, [0, 2, 1]]])
    key = np.sort(faces, axis=1)
    _, inv, counts = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    return faces[counts[inv] == 1]


# -- the rigid hand form ------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Prim:
    name: str
    kind: str              # box | capsule | sphere
    centre: tuple          # hand frame, m
    size: tuple = ()       # box: (x, y, z); capsule: (radius, spine length); sphere: (radius,)
    axis: tuple = (0., 0., 1.)  # capsule spine direction
    part: str = 'pla'      # pla | plate | cap (dome cap coloured like the skin)


def _finger(name, base_y, base_z, tilt_deg, length, dia):
    t = math.radians(tilt_deg)
    axis = (0.0, math.sin(t), math.cos(t))
    r = dia / 2 * MM
    spine = length * MM - r          # the capsule's far end is the fingertip
    base = np.array((0.0, base_y * MM, base_z * MM))
    centre = base + np.asarray(axis) * spine / 2
    return Prim(name, 'capsule', tuple(centre), (r, spine), axis)


def hand_prims():
    """Rigid parts of the hand form plus its bolted steel plate, in the hand frame (one rigid body)."""
    pl, wr, pa, of = HAND['plate_mm'], HAND['wrist_mm'], HAND['palm_mm'], HAND['other_fingers']
    web = HAND['web_z_mm']
    prims = [
        Prim('plate', 'box', (pl['center_dx'] * MM, 0.0, pl['z'] / 2 * MM), (pl['x'] * MM, pl['y'] * MM, pl['z'] * MM), part='plate'),
        Prim('wrist', 'box', (0.0, 0.0, (wr['bottom_z'] + wr['z'] / 2) * MM), (wr['x'] * MM, wr['y'] * MM, wr['z'] * MM)),
        Prim('palm', 'box', (0.0, pa['center_y'] * MM, (pa['bottom_z'] + pa['height_z'] / 2) * MM),
             (pa['thickness_x'] * MM, pa['width_y'] * MM, pa['height_z'] * MM)),
        # The ring finger's 9.6 mm PLA bone, 10 mm into the palm, up into the dome cap.
        Prim('bone', 'capsule', (0.0, 0.0, WEB_Z + 26.5 * MM), (BONE_COLLIDER_R, 63.0 * MM)),
        Prim('tip_cap', 'sphere', (0.0, 0.0, WEB_Z + TIP_CENTRE_U * MM), (TIP_R,), part='cap'),
    ]
    for name in ('middle', 'index', 'little'):
        f = of[name]
        prims.append(_finger(name, f['base_y_mm'], web + f['base_dz_mm'], f['tilt_deg'], f['length_mm'], f['dia_mm']))
    t = of['thumb']
    prims.append(_finger('thumb', t['base_y_mm'], t['base_z_mm'], t['tilt_deg'], t['length_mm'], t['dia_mm']))
    return prims


def capsule_rot(axis):
    """3x3 rotation taking +z to ``axis`` (rotation about x; every hand capsule lies in the y-z plane)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    if abs(a[0]) > 1e-9:
        raise ValueError('hand capsules lie in the y-z plane')
    ang = math.atan2(-a[1], a[2])
    c, s = math.cos(ang), math.sin(ang)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def prim_signed_distance(points, prim):
    """Signed distance (m) of hand-frame points to one rigid primitive."""
    p = np.asarray(points, float) - np.asarray(prim.centre)
    if prim.kind == 'box':
        q = np.abs(p) - np.asarray(prim.size) / 2
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(1), 0)
    if prim.kind == 'sphere':
        return np.linalg.norm(p, axis=1) - prim.size[0]
    r, spine = prim.size
    a = np.asarray(prim.axis, float)
    s = np.clip(p @ a, -spine / 2, spine / 2)
    return np.linalg.norm(p - s[:, None] * a, axis=1) - r


def prim_surface_points(prim, step=0.004):
    """Points on one primitive's surface (hand frame), spaced about ``step``."""
    if prim.kind == 'box':
        sx, sy, sz = prim.size
        out = []
        for axis in range(3):
            dims = [sx, sy, sz]
            others = [i for i in range(3) if i != axis]
            g = [np.linspace(-dims[i] / 2, dims[i] / 2, max(2, int(dims[i] / step) + 1)) for i in others]
            a, b = np.meshgrid(*g, indexing='ij')
            for sign in (-1, 1):
                pts = np.zeros((a.size, 3))
                pts[:, others[0]], pts[:, others[1]] = a.ravel(), b.ravel()
                pts[:, axis] = sign * dims[axis] / 2
                out.append(pts)
        return np.concatenate(out) + np.asarray(prim.centre)
    if prim.kind == 'sphere':
        r = prim.size[0]
        n = max(20, int(4 * math.pi * r * r / step ** 2))
        k = np.arange(n) + .5
        phi, th = np.arccos(1 - 2 * k / n), math.pi * (1 + 5 ** .5) * k
        return np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)] * r + np.asarray(prim.centre)
    r, spine = prim.size
    rot = capsule_rot(prim.axis)
    nz = max(2, int((spine + 2 * r) / step) + 1)
    nt = max(8, int(2 * math.pi * r / step))
    z = np.linspace(-spine / 2 - r, spine / 2 + r, nz)
    t = np.linspace(0, 2 * math.pi, nt, endpoint=False)
    zz, tt = np.meshgrid(z, t, indexing='ij')
    rad = np.where(np.abs(zz) <= spine / 2, r, np.sqrt(np.maximum(r * r - (np.abs(zz) - spine / 2) ** 2, 0)))
    local = np.c_[(rad * np.cos(tt)).ravel(), (rad * np.sin(tt)).ravel(), zz.ravel()]
    return local @ rot.T + np.asarray(prim.centre)


def hand_surface_points(step=0.004):
    return np.concatenate([prim_surface_points(p, step) for p in hand_prims()])


# -- the ring -----------------------------------------------------------------------------------------------

def ring_section(n_arc=3):
    """The band's cross-section (rho, z) in metres: a 1.9 x 4.0 mm rectangle with 0.5 mm corner radii, CCW."""
    ri, ro, h, e = RING_ID / 2, RING_OD / 2, RING_W / 2, RING_EDGE_R
    corners = [((ro - e, -h + e), -90.0), ((ro - e, h - e), 0.0), ((ri + e, h - e), 90.0), ((ri + e, -h + e), 180.0)]
    pts = []
    for (cr, cz), start in corners:
        for a in np.linspace(start, start + 90.0, n_arc):
            pts.append((cr + e * math.cos(math.radians(a)), cz + e * math.sin(math.radians(a))))
    return np.asarray(pts)


def ring_segments(n=RING_SEGMENTS):
    """Convex pieces of the band in the ring frame: list of (points (k, 3), faces list of index lists).

    Each piece is the cross-section swept straight between two angles (its side quads are planar), so it is
    convex; together they make the band with its 0.5 mm edge radii. The bore's polygon sag is 0.019 mm at 48 pieces,
    under the 0.125 mm radial interference at the PIP crest."""
    sec = ring_section()
    m = len(sec)
    out = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        pts = []
        for a in (a0, a1):
            for rho, z in sec:
                pts.append((rho * math.cos(a), rho * math.sin(a), z))
        faces = [list(range(m))[::-1], list(range(m, 2 * m))]
        for k in range(m):
            k1 = (k + 1) % m
            faces.append([k, k1, m + k1, m + k])
        out.append((np.asarray(pts), faces))
    return out


def ring_surface_points(n_around=72):
    sec = ring_section(4)
    a = np.linspace(0, 2 * math.pi, n_around, endpoint=False)
    rho, z = sec[:, 0], sec[:, 1]
    return np.c_[(rho[None] * np.cos(a)[:, None]).ravel(), (rho[None] * np.sin(a)[:, None]).ravel(),
                 np.repeat(z[None], len(a), 0).ravel()]


def ring_edge_circles(n=72):
    """Points on the band's lower and upper faces' outer and inner circles, ring frame: (lower (2n,3), upper (2n,3))."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    out = []
    for z in (-RING_W / 2, RING_W / 2):
        pts = [np.c_[r * np.cos(a), r * np.sin(a), np.full(n, z)] for r in (RING_OD / 2, RING_ID / 2)]
        out.append(np.concatenate(pts))
    return out


def quat_xyzw_to_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ring_readings(ring_pos, ring_rot, hand_pos, hand_rot):
    """The scorer's per-frame ring readings (mm) from the rigid poses (world, m):
    offset of the ring centre from the ring finger's axis, the highest u of the band's lower and upper edges, the
    lowest band point above the table top, and the band axis (world). The 'lower' face is the one nearer the web."""
    rp, rr, hp, hr = (np.asarray(v, float) for v in (ring_pos, ring_rot, hand_pos, hand_rot))
    lower, upper = ring_edge_circles()
    to_h = lambda pts: (pts - hp) @ hr                     # world -> hand frame
    c_h = to_h(rp[None])[0]
    a = to_h((lower @ rr.T) + rp)
    b = to_h((upper @ rr.T) + rp)
    if a[:, 2].mean() > b[:, 2].mean():
        a, b = b, a
    u = lambda pts: (pts[:, 2] - WEB_Z) / MM
    band_w = np.concatenate([(lower @ rr.T) + rp, (upper @ rr.T) + rp])
    return dict(ring_offset_mm=float(np.hypot(c_h[0], c_h[1]) / MM), ring_low_u_mm=float(u(a).max()),
                ring_top_u_mm=float(u(b).max()), ring_lowest_mm=float((band_w[:, 2].min() - TABLE_TOP) / MM),
                axis=rr[:, 2].copy(), centre=rp.copy())


def matrix_to_quat_wxyz(r):
    r = np.asarray(r, float)
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return np.array([0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = np.zeros(4)
    q[0] = (r[k, j] - r[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (r[j, i] + r[i, j]) / s
    q[1 + k] = (r[k, i] + r[i, k]) / s
    return q


def plate_corners_world(hand_pos, hand_rot):
    pl = HAND['plate_mm']
    cx = pl['center_dx'] * MM
    hx, hy = pl['x'] / 2 * MM, pl['y'] / 2 * MM
    local = np.array([[cx + sx * hx, sy * hy, 0.0] for sx in (-1, 1) for sy in (-1, 1)])
    return local @ np.asarray(hand_rot).T + np.asarray(hand_pos)
