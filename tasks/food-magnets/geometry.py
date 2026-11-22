"""food-magnets geometry: every dimension from params.json (the card's), in metres, world frame of yam_arena's station
(x forward, y left, origin at the arm-base midpoint; world z = table top + the card's height).

Pure Python + NumPy (no Isaac imports), shared by scene.py, scoring.py, perception.py and the demo.
"""
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
P = json.loads((HERE / 'params.json').read_text())
MM = 1e-3
TABLE_TOP = 0.7475                      # yam_arena.task_file.TABLE_TOP['station'] (checked in task.py)

PANEL = P['panel']
FACE_X = PANEL['face_x'] * MM           # steel front face
SHEET_T = PANEL['sheet_t'] * MM
SHEET_Y_HALF = PANEL['sheet_y_half'] * MM
SHEET_Z0, SHEET_Z1 = PANEL['sheet_z_bottom'] * MM, PANEL['sheet_z_top'] * MM   # above the table
UPRIGHT_T, UPRIGHT_Z = PANEL['upright_t'] * MM, PANEL['upright_z'] * MM
BASE_X, BASE_T = PANEL['base_x'] * MM, PANEL['base_t'] * MM

DISC = P['disc']
DISC_R = DISC['dia'] / 2 * MM
F0 = DISC['pull_rated_N']               # 6.27 N rated pull (G1 not measured: the rated value, an upper bound)
FALLOFF_G0 = 1.0 * MM                   # F(g) = F0 / (1 + g / 1 mm)^2 (geometry.md; shim fit is reviewer note 1, open)
FORCE_CUTOFF = 0.060                    # no pull beyond a 60 mm back-face gap (F < 0.002 N there)

FOOD = P['food']
NAMES = ('strawberry', 'cookie', 'donut')
THICK = dict(strawberry=FOOD['strawberry']['t'] * MM, cookie=FOOD['cookie']['t'] * MM, donut=FOOD['donut']['t'] * MM,
             plain=P['plain']['t'] * MM)
MASS = dict(strawberry=FOOD['strawberry']['mass_g'] * 1e-3, cookie=FOOD['cookie']['mass_g'] * 1e-3,
            donut=FOOD['donut']['mass_g'] * 1e-3, plain=P['plain']['mass_g'] * 1e-3)
# extents in the board plane (y width, z height)
EXTENT = dict(strawberry=(FOOD['strawberry']['w'] * MM, FOOD['strawberry']['h'] * MM),
              cookie=(FOOD['cookie']['dia'] * MM,) * 2, donut=(FOOD['donut']['od'] * MM,) * 2,
              plain=(P['plain']['dia'] * MM,) * 2)

CARD = P['cards']
CARD_W, CARD_H, CARD_T = CARD['w'] * MM, CARD['h'] * MM, CARD['t'] * MM
CARD_MASS = CARD['mass_g'] * 1e-3
CARD_BELOW_MAGNET = CARD['A']['magnet_above_card_centre'] * MM   # card centre = magnet centre - 25.5 mm in z

BOX = P['box']
BOX_X = BOX['spot_x'] * MM
BOX_LY, BOX_LX, BOX_H = BOX['y_len'] * MM, BOX['x_depth'] * MM, BOX['body_h'] * MM
WALL = BOX['wall'] * MM
INNER_Y, INNER_X, INNER_D = BOX['inner_y'] * MM, BOX['inner_x'] * MM, BOX['inner_depth'] * MM
FLOOR_T = BOX_H - INNER_D               # 3 mm
HINGE_X = BOX['hinge_x'] * MM
LID_T = BOX['lid_t'] * MM
LID_PLATE_T = 3 * MM                    # lid top plate (estimate: the listing gives only the outer 49 mm height)
LID_OPEN_DEG = BOX['lid_open_deg']
BOX_MASS = BOX['mass_g'] * 1e-3
TABLE = P['table']

R = P['randomisation']
COLS = [c * MM for c in R['cols_y']]
ROWS = {k: (v[0] * MM, v[1] * MM) for k, v in R['rows_z'].items()}
CELLS = [(c, r) for r in ('low', 'high') for c in range(3)]
JITTER_Y = R['jitter_y'] * MM
SPOTS = [s * MM for s in R['box_spots_y']]


def rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def quat_xyzw_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat_wxyz(r):
    r = np.asarray(r, float)
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    i = int(np.argmax(np.diag(r)))
    if i == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        return np.array([(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s])
    if i == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        return np.array([(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s])
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
    return np.array([(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s])


# Lid at rest: rotated about the hinge line (y axis) so its free edge rises toward the robot; lid frame origin on the
# hinge line, lid body along local +x, its lower (inner) face at local z 0.
LID_OPEN_ROT = rot_y(-LID_OPEN_DEG)


def outline(name, n=40):
    """(y, z) outline of a body's face in the board plane, centred on its centre (m)."""
    w, h = EXTENT[name]
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    if name == 'strawberry':
        # rounded, narrower toward the tip (bottom): an ellipse scaled across by 0.55..1.0 from tip to top
        y, z = np.cos(t), np.sin(t)
        f = 0.55 + 0.45 * (z + 1) / 2
        y = y * f
        y = y / np.abs(y).max() * w / 2
        return np.column_stack([y, z * h / 2])
    return np.column_stack([np.cos(t) * w / 2, np.sin(t) * h / 2])


def prism(name, n=40):
    """Convex prism points (body frame: x = thickness, back face at +t/2 toward the steel) for the collider."""
    o = outline(name, n)
    t = THICK[name]
    return np.vstack([np.column_stack([np.full(len(o), -t / 2), o]), np.column_stack([np.full(len(o), t / 2), o])])


def back_offset(name):
    """Disc centre (flush in the back face) in the body frame."""
    return np.array([THICK[name] / 2, 0.0, 0.0])


def magnet_gap(name, pos, rot):
    """(back-face gap to the steel face (m), cos of the back normal to +x, disc centre world) for a body pose."""
    c = np.asarray(pos) + np.asarray(rot) @ back_offset(name)
    n = np.asarray(rot)[:, 0]
    u, v = np.asarray(rot)[:, 1], np.asarray(rot)[:, 2]
    reach = DISC_R * math.hypot(u[0], v[0])
    return FACE_X - (c[0] + reach), float(n[0]), c


def on_sheet(c):
    return abs(c[1]) <= SHEET_Y_HALF and TABLE_TOP + SHEET_Z0 <= c[2] <= TABLE_TOP + SHEET_Z1


def pull(name, pos, rot):
    """Magnetic attraction (N, along +x, applied at the disc centre). F0 / (1 + g / 1 mm)^2 while the disc lies over the
    sheet and faces it; zero beyond FORCE_CUTOFF or behind the face. Returns (force_x, disc_centre, gap)."""
    g, nx, c = magnet_gap(name, pos, rot)
    if nx <= 0 or g > FORCE_CUTOFF or not on_sheet(c):
        return 0.0, c, g
    return F0 * nx / (1.0 + max(g, 0.0) / FALLOFF_G0) ** 2, c, g


def card_corners(centre, rot):
    """The note card's four corners (world) from its centre and rotation (card frame: y across, z up)."""
    out = []
    for sy in (-1, 1):
        for sz in (-1, 1):
            out.append(np.asarray(centre) + np.asarray(rot) @ np.array([0.0, sy * CARD_W / 2, sz * CARD_H / 2]))
    return np.array(out)


def lid_free_edge(pos, rot):
    """Midpoint of the lid's free (far) edge, lower face (world)."""
    return np.asarray(pos) + np.asarray(rot) @ np.array([BOX_LX, 0.0, 0.0])
