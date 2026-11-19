"""Key a V-belt sheave onto a motor shaft and lock it with its set screw.

The bench holds a NEMA 56 frame motor (5/8 in shaft, 3/16 in keyseat), a cast iron A/4L sheave of the Browning AK30X5/8 pattern started
8 mm onto the shaft end with its keyway turned 30 degrees off the keyseat, its 3/16 in square key lying on the bench, and a 5/32 in hex L-key
standing in a drilled wooden holder. The sheave's one set screw (5/16-18 x 3/8 cup point, over the keyway) is backed out 1.5 mm.

Model (declared in code; README "Fidelity"):
- "drive" is one articulation. Its frame is the motor and its shaft, with the keyseat cut into the shaft's colliders. Its links: the sheave's
  slide along the shaft (its upper limit is the shaft shoulder), the sheave's turn about the shaft (free), and the set screw's feed along its
  own axis. Self-collision is off inside the articulation, so the bore's fit on the shaft is the joints'; the key (a free body) collides with
  the keyseat's walls and the hub's keyway walls by contact, with ANSI class 1 side clearances.
- The set screw's thread is code: while the L-key's tip is in the screw's socket and lined up with it, the key's twist about the screw's
  axis turns the screw, which advances 1/18 in (1.41 mm) a turn until its cup point meets what is under it (the key, or the bare shaft).
  Turns past that first contact are counted; 45 degrees past it is the simulation's "tight" (standing in for 110-130 in-lb, which a YAM
  cannot apply); at 90 degrees the screw stalls. The screw resists the key with a token torque (TORQUE_*), applied to the L-key.
- With the cup point bearing (5 degrees past contact or more) the sheave's slide and turn are locked: the clamp.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

_plain_object = m._arena_object


def _arena_object(spec):
    """mechkit's spawn, with the articulation's reset writing its starting joint values as well as its root (Arena's default restores the root
    only, leaving the joints at the file's zero)."""
    obj = _plain_object(spec)
    if spec.role == "jointed":
        from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
        obj._pose_event_cfg.func = reset_articulation_pose_and_joints
    return obj


m._arena_object = _arena_object
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Mount the sheave on the motor shaft, all the way in against the shaft shoulder, and lock it there."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
DT = 0.05
TOP = TABLE_TOP[SCENE]
IN = 0.0254

# -- the motor (NEMA 56, ABB/Baldor-Reliance NEMA chart): shaft height D 3-1/2 in, frame diameter P 6-5/8 in, U 5/8 in, V 1-7/8 in,
#    AH 2-1/16 in; keyseat R 33/64 in, S 3/16 in. The drive's frame: origin on the shaft's axis at its shoulder; x along the shaft towards the
#    motor, z up.
AXIS_H = 3.5 * IN
X_SH = 0.265  # world x of the shoulder
ORIGIN = np.array([X_SH, 0.0, TOP + AXIS_H])
R_SHAFT = 0.3125 * IN
V_LEN = 1.875 * IN  # usable shaft length, shoulder to end
FACE_X = (2.0625 - 1.875) * IN  # the end shield's face, past the shoulder
JOURNAL_R = 0.0085  # the shaft steps up to the 17 mm bearing journal (a 6203 bearing) at the shoulder: the sheave's stop
R_FRAME = 6.625 / 2 * IN
MOTOR_LEN = 0.24
KS_HALF = 0.19 * IN / 2  # keyseat width 0.190 in (3/16 + 0.0025: ANSI class 1)
KS_FLOOR = (33 / 64 - 0.3125) * IN  # the keyseat's floor above the axis
KS_X1 = -0.125 * IN
KS_X0 = KS_X1 - 1.41 * IN  # ES, the keyseat's usable length: 1.41 in for a 56 frame (Techtop NEMA chart)
EDGE = 0.0004  # edge break on the keyseat's and keyway's edges, and the key's chamfers

# -- the key: 3/16 in square, 1-1/4 in long, steel
KEY_W = 0.1875 * IN
KEY_L = 1.25 * IN
KEY_MASS = 0.0057
KEY_SEAT = np.array([KS_X1 - KEY_L / 2 - 0.001, 0.0, KS_FLOOR + KEY_W / 2])  # its centre at the keyseat's inner end, drive frame (the
KEY_TOP = KS_FLOOR + KEY_W  # scorer uses only its height and centre line: the key may lie anywhere along the keyseat)
REST = np.array([0.195, 0.135])  # the key rest: a wooden block with a narrow rib on top that the key lies along, its sides overhanging
REST_SIZE = (0.040, 0.040, 0.025)  # x, y, height to the rib's top
RIB = (0.040, 0.0035, 0.013)  # the rib: x, y, height (on the block, which is 12 high)
KEY0 = (REST[0], REST[1], TOP + REST_SIZE[2] + KEY_W / 2 + 0.0003)  # on the rest, its length along x

# -- the sheave: Browning AK30X5/8 (its CAD model, inches; axial positions from the hub's inner end)
OD_R = 1.53 * IN
GROOVE_R = 1.092 * IN
RIM_IN = 0.95 * IN  # the rim's inside under the groove (not in the model's arcs: ours)
HUB_R = 0.785 * IN
BORE_R = 0.3132 * IN
KW_HALF = 0.191 * IN / 2  # hub keyway 0.1896 in in the model; 0.191 here (class 1)
KW_TOP = KEY_TOP + 0.00013  # the keyway's top: 0.13 mm over the key


def xc(z_cad: float) -> float:
    """The model's axial coordinate (inches, 0 at the rim's outer face, -1.19 at the hub's inner end) in the drive frame at home."""
    return -(1.19 + z_cad) * IN


RIM_F, RIM_B = xc(0.0), xc(-0.75)  # the rim's outer and inner faces
WEB_F, WEB_B = xc(-0.16), xc(-0.40)
SX = xc(-0.8802)  # the set screw's axis: 0.130 in past the rim's inner face (the model's hole is 5 deg off radial; radial here)
TRAVEL = 0.002 - V_LEN  # the slide at the start: the hub's inner end 2 mm onto the shaft end (just started on it)
TURN0 = 30.0  # degrees: the keyway turned towards -y (the right arm's side) at the start
SHEAVE_MASS = 0.445  # 0.98 lb

# -- the set screw: 5/16-18 x 3/8 cup point, socket 5/32 in, 0.140 in deep (Unbrako)
PITCH = IN / 18
SCREW_R = 0.3125 * IN / 2
SCREW_LEN = 0.375 * IN
SOCKET_D = 0.140 * IN
SOCKET_R = 0.0026  # a round socket here, 0.3 mm over the key's corners
CUP_R = 0.203 * IN / 2
BACKED = 0.0004  # the cup point's start above the key's top (backed out about 100 degrees)
TIP0 = KEY_TOP + BACKED  # the screw's tip at feed 0 (drive frame, along the screw's axis)
FEED_LO, FEED_HI = -(TIP0 - KS_FLOOR) - 0.0003, 0.0015
ALPHA_SNUG, ALPHA_TIGHT, ALPHA_STALL = 5.0, 45.0, 90.0  # degrees past the cup point's first contact
TORQUE_RUN, TORQUE_TIGHT, TORQUE_STALL = 0.02, 0.2, 1.5  # N m on the L-key: running, at 45 deg, stalled (token values: README)

# -- the L-key: 5/32 in hex, 2-1/2 in long arm, 27/32 in short arm (Eklind 16110). Frame: origin where the arms' centre lines meet; long arm
#    down -z, short arm along +x.
HEX_AF = 0.15625 * IN
HEX_R = HEX_AF / math.sqrt(3)  # across corners / 2
LONG = 2.5 * IN - HEX_AF / 2
SHORT = 27 / 32 * IN - HEX_AF / 2
LKEY_MASS = 0.009
HOLDER = np.array([0.205, -0.150])
HOLDER_H, HOLE_R, HOLE_DEPTH = 0.040, 0.0035, 0.035
LKEY0 = (HOLDER[0], HOLDER[1], TOP + HOLDER_H - HOLE_DEPTH + LONG + 0.001)

LOOKS = {
    "iron": m.Look(color=m.rgb(0.30, 0.31, 0.32), metallic=0.3, roughness=0.55),
    "groove": m.Look(color=m.rgb(0.40, 0.41, 0.42), metallic=0.8, roughness=0.35),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.9, roughness=0.3),
    "keysteel": m.Look(color=m.rgb(0.75, 0.74, 0.70), metallic=0.9, roughness=0.35),
    "oxide": m.Look(color=m.rgb(0.05, 0.05, 0.055), metallic=0.6, roughness=0.4),
    "motor": m.Look(color=m.rgb(0.20, 0.28, 0.38), metallic=0.2, roughness=0.5),
    "motor_dark": m.Look(color=m.rgb(0.12, 0.13, 0.15), metallic=0.2, roughness=0.6),
    "wood": m.Look(color=m.rgb(0.62, 0.45, 0.28), roughness=0.8),
    "hole": m.Look(color=m.rgb(0.10, 0.07, 0.05), roughness=0.9),
}


# -- meshes about the drive's x axis ----------------------------------------------------------------------------------------------------------


def _pt(x, r, phi):
    """A point at axial x, radius r, angle phi (rad) about +x from +z (a positive angle takes +z towards -y)."""
    return (x, -r * math.sin(phi), r * math.cos(phi))


def two_sided(mesh: m.Mesh) -> m.Mesh:
    return m.Mesh(mesh.v, np.vstack([mesh.f, mesh.f[:, ::-1]]))


def revolve(profile, segments=48) -> m.Mesh:
    """A closed (r, x) loop revolved about x."""
    n = len(profile)
    v = [_pt(x, r, 2 * math.pi * k / segments) for k in range(segments) for r, x in profile]
    f = []
    for k in range(segments):
        k2 = (k + 1) % segments
        for i in range(n):
            j = (i + 1) % n
            a, b, c, d = k * n + i, k * n + j, k2 * n + j, k2 * n + i
            f += [(a, b, c), (a, c, d)]
    return two_sided(m.Mesh(np.array(v, float), np.array(f)))


def sector(r0, r1, a0, a1, x0, x1, n=4) -> m.Mesh:
    """An annular sector (angles in degrees) from x0 to x1, as a closed solid (for hulls and visuals)."""
    angs = np.radians(np.linspace(a0, a1, n + 1))
    pts = [_pt(x, r, a) for x in (x0, x1) for r in (r0, r1) for a in angs]
    hull = _hull_faces(np.array(pts, float))
    return hull


def _hull_faces(pts: np.ndarray) -> m.Mesh:
    """A mesh of points for a convex hull collider (the faces only matter to the visual: drawn two-sided)."""
    c = pts.mean(axis=0)
    f = []
    n = len(pts)
    for i in range(1, n - 1):  # a fan: good enough for colliders, which PhysX hulls from the points
        f.append((0, i, i + 1))
    return m.Mesh(pts, np.array(f))


def hull_of(points) -> m.Mesh:
    return _hull_faces(np.asarray(points, float))


def x_cyl(r, x0, x1, y=0.0, z=0.0, segments=32) -> m.Mesh:
    return m.cylinder(r, x1 - x0, ((x0 + x1) / 2, y, z), axis="x", segments=segments)


def hex_prism(af, z0, z1, start=30.0) -> m.Mesh:
    rr = af / math.sqrt(3)
    outline = [(rr * math.cos(math.radians(start + 60 * k)), rr * math.sin(math.radians(start + 60 * k))) for k in range(6)]
    return m.prism(outline, z0, z1)


# -- the drive's frame: motor, shaft and keyseat ----------------------------------------------------------------------------------------------


def _shaft_colliders():
    cols = [m.col_hull("shaft_end", x_cyl(R_SHAFT, -V_LEN, KS_X0, segments=24)),
            m.col_hull("shaft_in", x_cyl(R_SHAFT, KS_X1, 0.0005, segments=24))]
    th = np.linspace(0, 2 * math.pi, 49)
    circ = [(R_SHAFT * math.sin(t), R_SHAFT * math.cos(t)) for t in th]
    low = [(y, z) for y, z in circ if z <= KS_FLOOR] + [(-math.sqrt(R_SHAFT ** 2 - KS_FLOOR ** 2), KS_FLOOR), (math.sqrt(R_SHAFT ** 2 - KS_FLOOR ** 2), KS_FLOOR)]
    cols.append(m.col_hull("shaft_low", hull_of([(x, y, z) for x in (KS_X0, KS_X1) for y, z in low])))
    for side, sgn in (("l", 1.0), ("r", -1.0)):
        z_edge = math.sqrt(R_SHAFT ** 2 - KS_HALF ** 2)
        pts = [(KS_HALF, KS_FLOOR), (KS_HALF, z_edge - EDGE), (KS_HALF + EDGE, math.sqrt(R_SHAFT ** 2 - (KS_HALF + EDGE) ** 2))]
        pts += [(y, z) for y, z in circ if y >= KS_HALF + EDGE and z >= KS_FLOOR]
        pts += [(math.sqrt(R_SHAFT ** 2 - KS_FLOOR ** 2), KS_FLOOR)]
        cols.append(m.col_hull(f"shaft_side_{side}", hull_of([(x, sgn * y, z) for x in (KS_X0, KS_X1) for y, z in pts])))
    return cols


def arc_strip(r, x0, x1, a0, a1, n=40) -> m.Mesh:
    """A cylindrical surface about x from angle a0 to a1 (rad), drawn two-sided."""
    angs = np.linspace(a0, a1, n + 1)
    v = [_pt(x, r, a) for a in angs for x in (x0, x1)]
    f = []
    for k in range(n):
        i = 2 * k
        f += [(i, i + 1, i + 3), (i, i + 3, i + 2)]
    return two_sided(m.Mesh(np.array(v, float), np.array(f)))


def _shaft_visual():
    z_edge = math.sqrt(R_SHAFT ** 2 - KS_HALF ** 2)
    a_e = math.asin(KS_HALF / R_SHAFT)
    t = 0.0002
    return [("shaft_end", two_sided(x_cyl(R_SHAFT, -V_LEN + 0.0006, KS_X0, segments=48)), "steel"),
            ("shaft_in", two_sided(x_cyl(R_SHAFT, KS_X1, 0.0005, segments=48)), "steel"),
            ("shaft_seat", arc_strip(R_SHAFT, KS_X0, KS_X1, a_e, 2 * math.pi - a_e, 48), "steel"),
            ("seat_floor", m.box((KS_X1 - KS_X0, 2 * KS_HALF, t), ((KS_X0 + KS_X1) / 2, 0.0, KS_FLOOR - t / 2)), "oxide"),
            ("seat_wall_l", m.box((KS_X1 - KS_X0, t, z_edge - KS_FLOOR), ((KS_X0 + KS_X1) / 2, KS_HALF + t / 2, (KS_FLOOR + z_edge) / 2)), "oxide"),
            ("seat_wall_r", m.box((KS_X1 - KS_X0, t, z_edge - KS_FLOOR), ((KS_X0 + KS_X1) / 2, -KS_HALF - t / 2, (KS_FLOOR + z_edge) / 2)), "oxide"),
            ("chamfer", revolve([(R_SHAFT - 0.0006, -V_LEN), (R_SHAFT, -V_LEN + 0.0006), (0.0, -V_LEN + 0.0006), (0.0, -V_LEN)]), "steel")]


SHIELD = [(0.0, FACE_X), (0.018, FACE_X), (0.045, FACE_X + 0.014), (R_FRAME, FACE_X + 0.032), (R_FRAME, FACE_X + 0.040), (0.0, FACE_X + 0.040)]
# the drive-end shield: a flat bearing hub 36 mm across at the face, then the bell sloping back to the frame


def _motor():
    """The motor: shoulder and bearing boss, a domed end shield, the frame (P 6-5/8 in), the rear end shield, a rigid base with two feet."""
    parts = [
        ("boss", revolve([(0.0, 0.0), (JOURNAL_R, 0.0), (JOURNAL_R, FACE_X), (0.0, FACE_X)]), "steel"),
        ("shield", revolve(SHIELD), "motor"),
        ("frame", revolve([(0.0, FACE_X + 0.040), (R_FRAME, FACE_X + 0.040), (R_FRAME, MOTOR_LEN - 0.03), (0.0, MOTOR_LEN - 0.03)]), "motor"),
        ("rear", revolve([(0.0, MOTOR_LEN - 0.03), (R_FRAME, MOTOR_LEN - 0.03), (0.070, MOTOR_LEN), (0.0, MOTOR_LEN)]), "motor_dark"),
        ("box", m.box((0.075, 0.06, 0.05), (0.13, -0.075, 0.05)), "motor"),
    ]
    base_z = -AXIS_H
    parts += [("base", m.box((0.17, 0.14, 0.006), (0.12, 0.0, base_z + 0.003)), "motor_dark"),
              ("foot_f", m.box((0.02, 0.12, 0.012), (0.06, 0.0, base_z + 0.009)), "motor_dark"),
              ("foot_r", m.box((0.02, 0.12, 0.012), (0.18, 0.0, base_z + 0.009)), "motor_dark"),
              ("cradle", m.box((0.13, 0.08, AXIS_H - R_FRAME + 0.004), (0.12, 0.0, base_z + (AXIS_H - R_FRAME) / 2 + 0.006)), "motor_dark")]
    cols = [m.col_hull("boss_c", x_cyl(JOURNAL_R, 0.0, FACE_X, segments=20)),
            m.col_hull("shield_c", revolve(SHIELD, 24)),
            m.col_hull("frame_c", x_cyl(R_FRAME, FACE_X + 0.040, MOTOR_LEN, segments=24)),
            m.col_box("base_c", (0.17, 0.14, AXIS_H - R_FRAME + 0.006), (0.12, 0.0, base_z + (AXIS_H - R_FRAME + 0.006) / 2))]
    return parts, cols


# -- the sheave (link "turn") ------------------------------------------------------------------------------------------------------------------


def _sheave():
    prof = [(BORE_R, 0.0), (HUB_R, 0.0), (HUB_R, WEB_B), (RIM_IN, WEB_B), (RIM_IN, RIM_B), (OD_R, RIM_B), (OD_R, xc(-0.625)), (GROOVE_R, xc(-0.4911)),
            (GROOVE_R, xc(-0.2589)), (OD_R, xc(-0.125)), (OD_R, RIM_F), (RIM_IN, RIM_F), (RIM_IN, WEB_F), (BORE_R + 0.0016, WEB_F), (BORE_R, WEB_F + 0.0016)]
    parts = [("sheave", revolve(prof, 64), "iron")]
    # the groove's worn-bright faces, and the keyway and the tapped hole drawn dark over the hub
    parts.append(("groove", revolve([(GROOVE_R + 0.0002, xc(-0.4911)), (OD_R - 0.0005, xc(-0.625) - 0.0001), (OD_R - 0.0005, xc(-0.125) + 0.0001), (GROOVE_R + 0.0002, xc(-0.2589))], 64), "groove"))
    parts.append(("keyway_end", m.box((0.0004, 2 * KW_HALF, KW_TOP - BORE_R + 0.0004), (0.0002, 0.0, (KW_TOP + BORE_R) / 2)), "oxide"))
    parts.append(("keyway_front", m.box((0.0004, 2 * KW_HALF, KW_TOP - BORE_R + 0.0004), (WEB_F - 0.0002, 0.0, (KW_TOP + BORE_R) / 2)), "oxide"))
    cols = []
    # rim: 16 sectors; web: 8; hub: 10 sectors round the bottom, and the top built round the keyway and the tapped hole
    for k in range(16):
        cols.append(m.col_hull(f"rim_{k}", sector(RIM_IN, OD_R, 22.5 * k, 22.5 * (k + 1), RIM_F, RIM_B, 2)))
    for k in range(8):
        cols.append(m.col_hull(f"web_{k}", sector(HUB_R - 0.001, RIM_IN + 0.0005, 45 * k, 45 * (k + 1), WEB_F, WEB_B, 2)))
    for k in range(10):
        cols.append(m.col_hull(f"hub_{k}", sector(BORE_R, HUB_R, 30 + 30 * k, 60 + 30 * k, WEB_F, 0.0, 2)))
    hole0, hole1 = SX - SCREW_R - 0.0002, SX + SCREW_R + 0.0002

    def wall(x0, x1, y0, y1):
        """The keyway's +y wall from x0 (inner face at y0) to x1 (at y1): out to the hub's surface and round to 30 degrees."""
        pts = []
        for x, yi in ((x0, y0), (x1, y1)):
            pts += [(x, yi, math.sqrt(BORE_R ** 2 - yi ** 2)), (x, yi, math.sqrt(HUB_R ** 2 - yi ** 2))]
            pts += [(x, BORE_R * math.sin(a), BORE_R * math.cos(a)) for a in np.linspace(math.asin(yi / BORE_R), math.radians(30.0), 5)]
            pts += [(x, HUB_R * math.sin(a), HUB_R * math.cos(a)) for a in np.linspace(math.asin(yi / HUB_R), math.radians(30.0), 6)]
        return pts

    segs = (("a", WEB_F, hole0, KW_HALF, KW_HALF), ("h", hole0, hole1, SCREW_R + 0.0002, SCREW_R + 0.0002), ("b", hole1, -EDGE, KW_HALF, KW_HALF),
            ("e", -EDGE, 0.0, KW_HALF, KW_HALF + EDGE))  # "e": the edge break where the keyway meets the hub's inner end
    for seg, x0, x1, y0, y1 in segs:
        pts = wall(x0, x1, y0, y1)
        cols.append(m.col_hull(f"kw_wall_p{seg}", hull_of(pts)))
        cols.append(m.col_hull(f"kw_wall_n{seg}", hull_of([(x, -y, z) for x, y, z in pts])))
    for seg, x0, x1, e1 in (("a", WEB_F, hole0, 0.0), ("b", hole1, -EDGE, 0.0), ("e", -EDGE, 0.0, EDGE)):  # the keyway's top
        pts = []
        for x, e in ((x0, 0.0), (x1, e1)):
            w = KW_HALF + e
            pts += [(x, y, KW_TOP + e) for y in (-w, w)] + [(x, y, math.sqrt(HUB_R ** 2 - y ** 2)) for y in (-w, w)]
        cols.append(m.col_hull(f"kw_top_{seg}", hull_of(pts)))
    return parts, cols


def _screw_link():
    """The set screw on its feed (a slider along the screw's axis, z in the sheave's frame at home). Its socket is a round pocket here."""
    top = TIP0 + SCREW_LEN
    parts = [("screw", m.cylinder(SCREW_R - 0.0003, SCREW_LEN - 0.0004, (SX, 0.0, TIP0 + SCREW_LEN / 2)), "oxide"),
             ("socket", m.Mesh(hex_prism(HEX_AF + 0.0001, top - SOCKET_D, top + 0.00005).v + np.array([SX, 0.0, 0.0]), hex_prism(HEX_AF + 0.0001, 0, 1).f), "hole")]
    cols = []
    for k in range(8):  # the socket's wall, with its countersunk entry (1 mm deep, out to 3.6 mm)
        a0, a1 = math.radians(45 * k), math.radians(45 * (k + 1))
        pts = []
        for a in np.linspace(a0, a1, 3):
            pts += [(SX + SOCKET_R * math.cos(a), SOCKET_R * math.sin(a), z) for z in (top - SOCKET_D, top - 0.001)]
            pts += [(SX + 0.0036 * math.cos(a), 0.0036 * math.sin(a), top)]
            pts += [(SX + (SCREW_R - 0.0002) * math.cos(a), (SCREW_R - 0.0002) * math.sin(a), z) for z in (top - SOCKET_D, top)]
        cols.append(m.col_hull(f"socket_{k}", hull_of(pts)))
    cols.append(m.col_hull("socket_floor", m.cylinder(SOCKET_R + 0.0002, 0.001, (SX, 0.0, top - SOCKET_D - 0.0005), segments=16)))
    cols.append(m.col_hull("cup", m.cylinder(CUP_R, 0.0008, (SX, 0.0, TIP0 + 0.0004), segments=16)))
    return m.slider("feed", "Z", FEED_LO, FEED_HI, parts, cols, at=(SX, 0.0, TIP0), parent="turn", mass=0.004, spring=40000.0, damping=60.0,
                    friction=0.0, effort=300.0, armature=0.004)


_fp = _shaft_visual()
_mp, _mc = _motor()
_sp, _sc = _sheave()
SLIDE = m.slider("slide", "X", TRAVEL, 0.0, [], [], at=(0.0, 0.0, 0.0), mass=0.02, friction=0.3, damping=4.0, armature=0.01)
TURN = m.hinge("turn", "X", (0.0, 0.0, 0.0), float("-inf"), float("inf"), _sp, _sc, parent="slide", mass=SHEAVE_MASS, friction=0.003, damping=0.004,
               armature=0.0002, inertia=(2.3e-4, 1.4e-4, 1.4e-4))
DRIVE = m.mechanism("drive", tuple(ORIGIN), [SLIDE, TURN, _screw_link()], LOOKS, frame_parts=_fp + _mp, frame_colliders=_shaft_colliders() + _mc,
                    friction=(0.35, 0.3), solver_iters=48, start=[("slide", TRAVEL), ("turn", math.radians(TURN0))])


# -- the key, the L-key and its holder ----------------------------------------------------------------------------------------------------------


def _key_mesh(c=EDGE):
    h, l = KEY_W / 2, KEY_L / 2
    oct_ = [(h, h - c), (h - c, h), (-h + c, h), (-h, h - c), (-h, -h + c), (-h + c, -h), (h - c, -h), (h, -h + c)]
    pts = [(x, y, z) for y, z in oct_ for x in (-l + c, l - c)]
    pts += [(x, y * (h - c) / h, z * (h - c) / h) for y, z in oct_ for x in (-l, l)]
    return hull_of(pts)


KEY = m.free("key", KEY0, [("key", m.box((KEY_L - 0.0002, KEY_W - 0.0001, KEY_W - 0.0001)), "keysteel")], [m.col_hull("key_c", _key_mesh())], LOOKS, mass=KEY_MASS,
             friction=(0.35, 0.3), torsional_patch=0.002, solver_iters=48)


def _lkey():
    long_ = hex_prism(HEX_AF, -LONG, HEX_AF / 2, start=0.0)  # flats facing +-y on both arms, as the bend about y leaves them
    short = hex_prism(HEX_AF, 0.0, SHORT, start=0.0)
    short = m.Mesh(short.v[:, [2, 1, 0]] * np.array([1.0, 1.0, -1.0]), short.f)  # along +x
    parts = [("long", two_sided(long_), "oxide"), ("short", two_sided(short), "oxide")]
    cols = [m.col_hull("long_c", long_), m.col_hull("short_c", short)]
    return parts, cols


_lp, _lc = _lkey()
LKEY_YAW = 150.0  # the short arm points this way (degrees from +x) in its holder
LKEY = m.free("lkey", LKEY0, _lp, _lc, LOOKS, mass=LKEY_MASS, friction=(0.6, 0.5), torsional_patch=0.002, solver_iters=48, yaw_deg=LKEY_YAW)


def _holder():
    s, h = 0.040, HOLDER_H
    parts = [("block", m.box((s, s, h), (0.0, 0.0, h / 2)), "wood"), ("hole", m.cylinder(HOLE_R, 0.0004, (0.0, 0.0, h + 0.0002), segments=20), "hole")]
    cols = [m.col_box("floor", (s, s, h - HOLE_DEPTH), (0.0, 0.0, (h - HOLE_DEPTH) / 2))]
    for name, size, c in (("w_px", (s / 2 - HOLE_R, s, HOLE_DEPTH), ((s / 2 + HOLE_R) / 2, 0.0, h - HOLE_DEPTH / 2)),
                          ("w_nx", (s / 2 - HOLE_R, s, HOLE_DEPTH), (-(s / 2 + HOLE_R) / 2, 0.0, h - HOLE_DEPTH / 2)),
                          ("w_py", (2 * HOLE_R, s / 2 - HOLE_R, HOLE_DEPTH), (0.0, (s / 2 + HOLE_R) / 2, h - HOLE_DEPTH / 2)),
                          ("w_ny", (2 * HOLE_R, s / 2 - HOLE_R, HOLE_DEPTH), (0.0, -(s / 2 + HOLE_R) / 2, h - HOLE_DEPTH / 2))):
        cols.append(m.col_box(name, size, c))
    return parts, cols


_hp, _hc = _holder()
HOLDER_PART = m.fixed("holder", (HOLDER[0], HOLDER[1], TOP), _hp, _hc, LOOKS, friction=(0.6, 0.5))
_BASE_H = REST_SIZE[2] - RIB[2]
KEY_REST = m.fixed("rest", (REST[0], REST[1], TOP),
                   [("block", m.box((REST_SIZE[0], REST_SIZE[1], _BASE_H), (0.0, 0.0, _BASE_H / 2)), "wood"),
                    ("rib", m.box(RIB, (0.0, 0.0, _BASE_H + RIB[2] / 2)), "wood")],
                   [m.col_box("block_c", (REST_SIZE[0], REST_SIZE[1], _BASE_H), (0.0, 0.0, _BASE_H / 2)),
                    m.col_box("rib_c", RIB, (0.0, 0.0, _BASE_H + RIB[2] / 2))], LOOKS, friction=(0.6, 0.5))

# -- distractors (SimReady): a shop rag, a combination wrench, a mug
RAG = SimReady("rag", pos=(0.47, 0.27, TOP), size=(0.26, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01", usd="sm_cleaning_cloth_a01_01.usd", yaw=0.3)
WRENCH = SimReady("wrench", pos=(0.42, -0.24, TOP), size=(0.26, 0.046, 0.017), mass=0.25, package="handTool_wrench_combination_a01",
                  usd="sm_handTool_wrench_combination_a01_01.usd", yaw=0.0)
MUG = SimReady("mug", pos=(0.62, -0.16, TOP), size=(0.114, 0.09, 0.08), mass=0.35, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.0)

OBJECTS = [DRIVE, KEY, LKEY, HOLDER_PART, KEY_REST, RAG, WRENCH, MUG]


# -- geometry in the scene ------------------------------------------------------------------------------------------------------------------------


def _rx(phi: float) -> np.ndarray:
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def drive_joints(env) -> dict:
    s = m.state(env, "drive")
    return {"slide": s.joint("slide"), "turn": s.joint("turn"), "feed": s.joint("feed")}


def screw_axis(j) -> tuple[np.ndarray, np.ndarray]:
    """The screw's axis (a point on the shaft's axis under it, and its outward direction), drive frame."""
    return np.array([SX + j["slide"], 0.0, 0.0]), _rx(j["turn"]) @ np.array([0.0, 0.0, 1.0])


def to_drive(p_world) -> np.ndarray:
    return np.asarray(p_world, float) - ORIGIN


def key_surface(c, u, kpos, krot):
    """Where the screw's axis line c + t u meets the key's upward face, as t (m), or None."""
    best = None
    for col in (1, 2):
        for sgn in (1.0, -1.0):
            n = sgn * krot[:, col]
            if n @ u < 0.9:
                continue
            fc = kpos + n * KEY_W / 2
            t = float((fc - c) @ n / (u @ n))
            p = c + t * u - kpos
            along = abs(p @ krot[:, 0])
            across = abs(p @ krot[:, 3 - col])
            if along <= KEY_L / 2 - 0.0005 and across <= KEY_W / 2:
                best = t if best is None else max(best, t)
    return best


def surface_under_screw(j, kpos, krot) -> tuple[float, str]:
    """The first surface under the cup point along the screw's axis: (t, "key" | "floor" | "shaft")."""
    c, u = screw_axis(j)
    t_key = key_surface(c, u, kpos, krot)
    in_seat = KS_X0 < c[0] < KS_X1 and abs(KS_FLOOR * math.tan(j["turn"])) < KS_HALF - CUP_R * 0.5 and math.cos(j["turn"]) > 0.0
    t_shaft = (KS_FLOOR / math.cos(j["turn"]), "floor") if in_seat else (R_SHAFT, "shaft")
    if t_key is not None and t_key > t_shaft[0]:
        return t_key, "key"
    return t_shaft


def lkey_points(st) -> dict:
    R = st.rotation()
    p = np.asarray(st.pos, float)
    return {"tip": p + R @ np.array([0.0, 0.0, -LONG]), "corner": p, "end": p + R @ np.array([SHORT, 0.0, 0.0]),
            "axis": R @ np.array([0.0, 0.0, 1.0]), "arm": R @ np.array([1.0, 0.0, 0.0])}


def sheave_gap(points_world, j) -> np.ndarray:
    """Each point's distance (m) from the sheave's outside (a cylinder of its OD, rim face to hub end)."""
    q = np.atleast_2d(points_world) - ORIGIN
    x0, x1 = RIM_F + j["slide"], j["slide"]
    rho = np.hypot(q[:, 1], q[:, 2])
    dx = np.maximum(np.maximum(x0 - q[:, 0], q[:, 0] - x1), 0.0)
    dr = np.maximum(rho - OD_R, 0.0)
    return np.hypot(dx, dr)


def key_seated(kpos, krot):
    """Whether the key lies fully down in the keyseat (drive frame), with its offset from seated (m), long-axis tilt and roll (degrees)."""
    off = kpos - KEY_SEAT
    tilt = math.degrees(math.acos(float(np.clip(abs(krot[:, 0] @ np.array([1.0, 0.0, 0.0])), -1.0, 1.0))))
    roll = math.degrees(math.acos(float(np.clip(max(abs(krot[2, 1]), abs(krot[2, 2])), -1.0, 1.0))))  # a face of the key off level (mod 90)
    home = (roll < 10.0 and abs(off[2]) < KEY_TOL and abs(off[1]) < KEY_TOL and KS_X0 - KEY_TOL < kpos[0] - KEY_L / 2
            and kpos[0] + KEY_L / 2 < KS_X1 + KEY_TOL and tilt < 1.5)
    return home, off, tilt, roll


def key_gap(points_world, kpos_d, krot) -> np.ndarray:
    q = (np.atleast_2d(points_world) - ORIGIN - kpos_d) @ krot
    d = np.maximum(np.abs(q) - np.array([KEY_L, KEY_W, KEY_W]) / 2, 0.0)
    return np.linalg.norm(d, axis=1)


# -- the set screw's thread, and the clamp ---------------------------------------------------------------------------------------------------


def _wrench(env, part, force, torque, body=None):
    import torch
    asset = env.unwrapped.scene[part]
    i = 0 if body is None else int(asset.find_bodies(body)[0][0])
    f = torch.tensor([[[float(v) for v in force]]], device=asset.device)
    t = torch.tensor([[[float(v) for v in torque]]], device=asset.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=t, body_ids=[i], is_global=True)
    m._PUSHED.add((part, i))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Screw:
    """The set screw's turn (theta, rad, tightening positive) and what it bears on. Cleared only at a reset."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.theta = 0.0
        self.psi = None  # the L-key's twist about the screw's axis at the last step, while engaged
        self.engaged = False
        self.alpha = -1e9  # degrees past first contact
        self.on = "none"
        self.locked = False
        self.turn_held = False
        self.keyed = False
        self.max_alpha = -1e9
        self.torque = 0.0
        if env is not None:
            m.unlock(env, "drive", "slide")
            m.unlock(env, "drive", "turn")
            self.step(env)

    def lkey_engaged(self, env, j):
        st = m.state(env, "lkey")
        pts = lkey_points(st)
        c, u = screw_axis(j)
        top = c + u * (TIP0 + j["feed"] + SCREW_LEN)
        tip = to_drive(pts["tip"])
        rel = tip - top
        depth = float(-rel @ u)
        lateral = float(np.linalg.norm(rel - (rel @ u) * u))
        tilt = math.degrees(math.acos(float(np.clip(pts["axis"] @ u, -1.0, 1.0))))
        ok = lateral < 0.0011 and 0.0018 < depth < SOCKET_D + 0.0012 and tilt < 12.0
        e1 = np.array([1.0, 0.0, 0.0])
        e2 = np.cross(u, e1)
        a = pts["arm"]
        psi = math.atan2(float(a @ e2), float(a @ e1))
        return ok, psi, u, {"lateral": lateral, "depth": depth, "tilt": tilt}

    def step(self, env):
        j = drive_joints(env)
        ks = m.state(env, "key")
        kpos, krot = to_drive(ks.pos), ks.rotation()
        t_surf, what = surface_under_screw(j, kpos, krot)
        theta_c = 2 * math.pi * (TIP0 - t_surf) / PITCH
        ok, psi, u, self.diag = self.lkey_engaged(env, j)
        torque = 0.0
        if ok and self.psi is not None and self.engaged:
            d = -_wrap(psi - self.psi)  # tightening: clockwise seen from outside
            stall = math.radians(ALPHA_STALL) + theta_c
            new = min(self.theta + d, max(self.theta, stall))
            new = max(new, -2 * math.pi)
            moved = new - self.theta
            self.theta = new
            alpha_deg = math.degrees(self.theta - theta_c)
            t_res = TORQUE_RUN if alpha_deg < 0 else TORQUE_RUN + (TORQUE_TIGHT - TORQUE_RUN) * alpha_deg / ALPHA_TIGHT
            rate = d / DT
            if d > 0 and abs(moved) < 0.5 * abs(d):  # stalled: the screw holds the key
                torque = TORQUE_STALL * min(1.0, abs(rate) / 0.3)
            else:
                torque = t_res * float(np.clip(rate / 0.3, -1.0, 1.0))
            # torque on the L-key about the screw's axis, against its turn (tightening is negative about u)
        self.engaged = ok
        self.psi = psi if ok else None
        self.torque = torque
        _wrench(env, "lkey", (0.0, 0.0, 0.0), tuple(u * torque))
        self.alpha = math.degrees(self.theta - theta_c)
        self.on = what if self.alpha >= 0.0 else "none"
        self.max_alpha = max(self.max_alpha, self.alpha)
        tip = TIP0 - PITCH * self.theta / (2 * math.pi)
        tip = max(tip, t_surf + 0.00005)
        m.set_target(env, "drive", "feed", float(np.clip(tip - TIP0, FEED_LO, FEED_HI)))
        clamp = self.alpha >= ALPHA_SNUG
        # the key's drive: with the key down in the keyseat and the hub's keyway lined up over it, the key stops the sheave
        # turning (the joint is held, since PhysX lets a 6 g key be squeezed sideways between the hub and the shaft)
        on_key = j["slide"] > kpos[0] - KEY_L / 2 + 0.002 and j["slide"] + WEB_F < kpos[0] + KEY_L / 2 - 0.002  # the hub 2 mm or more over it
        keyed = key_seated(kpos, krot)[0] and on_key and abs(math.degrees(_wrap(j["turn"]))) < 2.0
        self.keyed = keyed
        if clamp != self.locked:
            (m.lock if clamp else m.unlock)(env, "drive", "slide")
            self.locked = clamp
        if (clamp or keyed) != self.turn_held:
            (m.lock if (clamp or keyed) else m.unlock)(env, "drive", "turn")
            self.turn_held = clamp or keyed


SCREW = Screw()
m.on_step(SCREW.step)
m.on_reset(SCREW.reset)


# -- the scorer --------------------------------------------------------------------------------------------------------------------------------


KEY_TOL = 0.0004  # m: the key's seated pose, across, in height and at its ends (PhysX settles a seated key within about 0.3 mm)
TEST_FORCE, TEST_TORQUE = 20.0, 1.0  # N along the shaft, off its end; N m about it
TEST_MOVE, TEST_TURN = 0.0003, 0.3  # m, degrees


def now(env) -> dict:
    j = drive_joints(env)
    ks = m.state(env, "key")
    kpos, krot = to_drive(ks.pos), ks.rotation()
    tips = m.fingertips(env)
    lk = lkey_points(m.state(env, "lkey"))
    lk_pts = np.array([lk["tip"], lk["corner"], lk["end"]])
    key_home, off, tilt, roll = key_seated(kpos, krot)
    covered = j["slide"] - (0.0 - WEB_F) < kpos[0] + KEY_L / 2  # the hub over part of the key
    key_free = covered or float(np.min(key_gap(tips, kpos, krot))) >= 0.01
    home = j["slide"] > -0.0005
    aligned = abs(math.degrees(_wrap(j["turn"]))) < 2.0
    hands = float(np.min(sheave_gap(tips, j)))
    tool = float(np.min(sheave_gap(lk_pts, j)))
    c, u = screw_axis(j)
    socket = ORIGIN + c + u * (TIP0 + j["feed"] + SCREW_LEN)
    tool_out = float(np.linalg.norm(lk["tip"] - socket)) >= 0.015
    return {"key_home": key_home, "key_free": key_free, "home": home, "aligned": aligned, "on": SCREW.on, "alpha": SCREW.alpha,
            "hands": hands, "tool": tool, "tool_out": tool_out, "j": j, "key_off": off, "key_tilt": tilt, "key_roll": roll}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.level = 0.0
        self.testing = False
        self.ref = None
        self.cool = 0
        self.c = {}

    def _test(self, env, on: bool):
        if on and not self.testing:
            self.testing = True
            j = drive_joints(env)
            self.ref = (j["slide"], j["turn"])
            _wrench(env, "drive", (-TEST_FORCE, 0.0, 0.0), (TEST_TORQUE, 0.0, 0.0), body="turn")
        elif not on and self.testing:
            self.testing = False
            _wrench(env, "drive", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), body="turn")

    def step(self, env):
        c = now(env)
        self.c = c
        level = 0.0
        if c["key_home"] and c["key_free"]:
            level = 0.25
            if c["home"] and c["aligned"]:
                level = 0.5
                if c["on"] == "key" and c["alpha"] >= ALPHA_SNUG:
                    level = 0.75
        self.level = level
        ready = level == 0.75 and c["alpha"] >= ALPHA_TIGHT and c["tool_out"] and c["hands"] >= 0.03 and c["tool"] >= 0.03
        if self.cool > 0:
            self.cool -= 1
            ready = False
        self._test(env, ready)
        moved = False
        if self.testing:
            j = c["j"]
            moved = abs(j["slide"] - self.ref[0]) > TEST_MOVE or abs(math.degrees(_wrap(j["turn"] - self.ref[1]))) > TEST_TURN
            if moved:
                self._test(env, False)
                self.cool = 20
        self.hold = self.hold + 1 if (ready and self.testing and not moved) else 0
        if self.hold >= HOLD_STEPS:
            self.full = True
        self.score = 1.0 if self.full else level
        if self.full and self.testing:
            self._test(env, False)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)

_LOG = None


def _debug(env):
    import os
    path = os.environ.get("SHEAVE_LOG")
    if not path:
        return
    c = TRIAL.c
    ks = m.state(env, "key")
    lk = m.state(env, "lkey")
    tips = m.fingertips(env)
    with open(path, "a") as f:
        f.write(f"{c.get('j')} key {np.round(to_drive(ks.pos)*1000, 2).tolist()} off {np.round(np.asarray(c.get('key_off', [0,0,0]))*1000, 2).tolist()} "
                f"tilt {c.get('key_tilt', 0):.2f} roll {c.get('key_roll', 0):.2f} home {c.get('key_home')} lvl {TRIAL.level} th {math.degrees(SCREW.theta):.1f} a {SCREW.alpha:.1f} on {SCREW.on} "
                f"eng {SCREW.engaged} {({k: round(v*1000, 2) if k != 'tilt' else round(v, 1) for k, v in SCREW.diag.items()})} tq {SCREW.torque:.2f} "
                f"lkey {np.round(np.asarray(lk.pos)*1000, 1).tolist()} hands {c.get('hands', 0)*1000:.0f} tool {c.get('tool', 0)*1000:.0f} out {c.get('tool_out')} "
                f"test {TRIAL.testing} hold {TRIAL.hold} full {TRIAL.full} L {np.round(tips[:12].mean(0)*1000,1).tolist()} R {np.round(tips[12:].mean(0)*1000,1).tolist()} "
                f"Ltip {np.round(tips[[2, 8]].mean(0)*1000,1).tolist()} Rtip {np.round(tips[[14, 20]].mean(0)*1000,1).tolist()} lkq {np.round(np.asarray(lk.quat_xyzw), 3).tolist()} kq {np.round(np.asarray(ks.quat_xyzw), 3).tolist()}\n")


m.on_step(_debug)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration (open loop, from the published layout) ---------------------------------------------------------------------------------

AXZ = TOP + AXIS_H
# commanded corrections found from earlier runs of this same plan (the arm's droop and the key's seat in the fingers); 0 = nominal
KEY_DX, KEY_DY, KEY_DZ = 0.0, 0.0, 0.0  # corrections to where the hand lets the key go (none needed)
LK_TIP = (0.0023, -0.0021)  # where the held L-key's tip hangs from below the TCP at the wrist's yaw LKEY_YAW (it leans 3 degrees in the
#                              pinch; measured in earlier runs of this plan). The offset turns with the wrist, so each waypoint over the socket allows for it
TURN_FIX = -0.7  # degrees: the rolled sheave lags the hand by about this much, so the roll overshoots by it
PAD_TIP = 0.013  # the finger tips below the TCP (their inner faces taper back near the tip)
UNDER = 0.008  # used for the rim grip only: the pads 15 mm down the rim's faces
KEY_HOLD = 0.0032  # the finger tips this far below the key's underside while it is pinched across its width (their lowest 3 mm taper back
#                    and do not grip: the key is held by the pads' faces above that, beside the rest's rib)
LK_GRIP = -0.006  # the TCP above the L-key's corner when it is gripped in its holder: the pads on the long arm's top 14 mm and the short arm's root
LK_HELD = LK_GRIP - 0.001  # the TCP above the L-key's corner once it is lifted out


def demo_plan():
    import demokit as d
    from yam_arena.demo import Waypoint
    from yam_arena.urdf import axis_angle_matrix

    OPEN, SHUT = -0.04695, 0.0

    def w(t, p, finger, label, grip):
        return Waypoint(float(t), tuple(float(v) for v in p), float(finger), label, grip)

    def rolled(phi_deg, yaw):
        return d._as_grip(axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(phi_deg)) @ d.down(yaw).rot)

    g0, g90 = d.down(0.0), d.down(-90.0)
    # -- left: the key into the keyseat, the sheave's keyway lined up, the sheave pushed home
    # the key is pinched across its width (its two side faces flat in the pads, so it cannot roll); over the keyseat it is let go with the
    # finger tips 8 mm above the axis (the keyseat's edges are 7.6 mm up), drops about 6 mm into the keyseat and is pressed home along its
    # length with the closed fingers
    kp = np.array([KEY0[0], KEY0[1], TOP + REST_SIZE[2] + PAD_TIP - KEY_HOLD])
    seat = ORIGIN + KEY_SEAT
    seat_c = ORIGIN + KEY_SEAT + np.array([KEY_DX, KEY_DY, 0.0])  # where the hand lets the key go
    kplace = np.array([seat_c[0], seat_c[1], AXZ + 0.0080 + PAD_TIP + KEY_DZ])  # the tips 8 mm above the axis, just over the keyseat's edges
    press = AXZ + KEY_TOP + PAD_TIP - 0.008  # the closed fingers' tips commanded 8 mm into the key's top: the arm's stiffness presses it
    rim_x = X_SH + TRAVEL + (RIM_F + RIM_B) / 2
    r_tcp = OD_R + UNDER - 0.010  # the pads 15 mm down the rim's faces
    front = X_SH + TRAVEL + RIM_F
    left = [
        w(2.0, kp + [0, 0, 0.05], -0.012, "above the key", g0),  # opened 24 mm: the fingers close slowly
        w(1.2, kp, -0.012, "fingers either side of the key", g0),
        w(1.2, kp, SHUT, "pinch the key across its width", g0),
        w(1.0, kp + [0, 0, 0.05], SHUT, "lift the key", g0),
        w(2.0, kplace + [0, 0, 0.03], SHUT, "over the keyseat", g0),
        w(1.5, kplace + [0, 0, 0.003], SHUT, "the key down to the keyseat", g0),
        w(1.0, kplace, SHUT, "just over the keyseat", g0),
        w(0.3, kplace, SHUT, "settle", g0),
        w(0.5, kplace, -0.006, "let go of the key", g0),
        w(0.6, kplace + [0, 0, 0.012], -0.006, "up", g0),
        w(0.8, kplace + [0, 0, 0.012], SHUT, "close the fingers", g0),
        # the closed fingers' lower edges (36 mm long) press the key's whole length down at once
        w(0.8, (seat[0], 0.0, press), SHUT, "press it home along its length", g0),
        w(0.6, (seat[0], 0.0, press), SHUT, "press it home along its length", g0),
        w(0.6, (seat[0], 0.0, press + 0.02), SHUT, "up", g0),
        w(0.8, (seat[0] - 0.02, 0.0, AXZ + 0.09), SHUT, "up clear of the sheave", g90),
        w(1.2, (rim_x, 0.0, AXZ + 0.09), SHUT, "over the rim", g90),
        w(2.0, (rim_x, 0.0, AXZ + r_tcp + 0.03), -0.022, "over the sheave's rim", g90),
        w(1.2, (rim_x, 0.0, AXZ + r_tcp), -0.022, "fingers either side of the rim", g90),
        w(1.2, (rim_x, 0.0, AXZ + r_tcp), SHUT, "pinch the rim", g90),
    ]
    for phi in -np.linspace(0.25, 1.0, 4) * (TURN0 - TURN_FIX):  # turn the sheave 30 degrees: the keyway up to the key
        p = np.array([rim_x, -r_tcp * math.sin(math.radians(phi)), AXZ + r_tcp * math.cos(math.radians(phi))])
        left.append(w(0.7, p, SHUT, "turn the sheave", rolled(phi, -90.0)))
    p30 = np.asarray(left[-1].pos)
    rg = rolled(phi, -90.0)
    home_p = p30 + np.array([-TRAVEL + 0.0015, 0.0, 0.0])  # the rim carried 45.6 mm along the shaft, 1.5 mm past home: pressed on the shoulder
    left += [
        w(0.5, p30, SHUT, "hold the keyway over the key", rg),
        # slid home with the rim still pinched, so the keyway stays lined up with the key as the hub's chamfered keyway takes it
        w(4.5, home_p, SHUT, "slide the sheave home along the shaft over the key", rg),
        w(0.6, home_p, SHUT, "against the shoulder", rg),
        w(0.6, home_p, -0.022, "let go of the rim", rg),
        w(1.0, home_p + [0, 0.02, 0.03], -0.022, "off the rim", rg),
        w(2.0, (0.15, 0.25, TOP + 0.20), SHUT, "the left hand clear", g0),
    ]
    t_left = sum(p.duration for p in left)
    # -- right: the L-key from its holder, its long arm into the set screw, one stroke clockwise, back to its holder
    lean = 0.0021 * (LONG + LK_GRIP) / LONG  # it leans in its hole toward its short arm: the long arm's offset at the grip (measured)
    ly = math.radians(LKEY_YAW)
    pick = np.array([LKEY0[0] + lean * math.cos(ly), LKEY0[1] + lean * math.sin(ly), LKEY0[2] - 0.0010 + LK_GRIP])
    gl = d.down(LKEY_YAW)  # fingers closing across the L-key's flats
    sock = np.array([X_SH + SX, 0.0, AXZ + TIP0 + SCREW_LEN])  # the socket's top, the sheave home

    def over(yaw, zz):
        """The TCP that puts the L-key's tip over the socket with the wrist at `yaw` degrees, at TCP height zz."""
        c, sn = math.cos(math.radians(yaw - LKEY_YAW)), math.sin(math.radians(yaw - LKEY_YAW))
        off = np.array([c * LK_TIP[0] - sn * LK_TIP[1], sn * LK_TIP[0] + c * LK_TIP[1]])
        return (sock[0] - off[0], sock[1] - off[1], zz)
    seat_z = sock[2] - SOCKET_D + LONG + LK_HELD - 0.0005  # TCP with the key's tip on the socket's floor (pressed 0.5 mm)
    above = np.array(over(LKEY_YAW, seat_z + 0.0085))  # the tip 5 mm over the socket
    side = np.array([sock[0], -0.16, above[2]])  # waiting well clear of the sheave and the left hand
    Y0, Y1 = LKEY_YAW, LKEY_YAW - 190.0  # a stroke: 190 degrees of the wrist
    right = [
        w(2.5, pick + [0, 0, 0.04], -0.012, "above the L-key", gl),
        w(1.2, pick, -0.012, "fingers either side of the L-key's long arm and corner", gl),
        w(1.2, pick, SHUT, "pinch the L-key", gl),
        w(1.5, (pick[0], pick[1], TOP + 0.16), SHUT, "lift it out of its holder", gl),
        w(3.0, side, SHUT, "beside the sheave", gl),
    ]
    t_r = sum(p.duration for p in right)
    right.append(w(max(t_left - 2.0 - t_r, 0.5), side, SHUT, "wait beside the sheave until it is home and the left hand is away", gl))
    z = seat_z
    right += [
        w(4.0, above, SHUT, "slowly over the set screw", gl),  # slow: the L-key pivots a little in the pinch
        w(1.0, above, SHUT, "settle", d.down(Y0)),
        w(1.5, over(Y0, z), SHUT, "the tip into the socket", d.down(Y0)),
        w(0.4, over(Y0, z), SHUT, "seat it", d.down(Y0)),
    ]
    for stroke in range(2):  # two strokes of 190 degrees; between them the tip is lifted out and the L-key turned back 180 degrees
        a0 = Y0 if stroke == 0 else Y1 + 180.0
        if stroke == 1:
            right += [w(0.6, over(Y1, z + 0.005), SHUT, "the tip up out of the socket", d.down(Y1)),
                      *[w(0.4, over(Y1 + 45.0 * (q + 1), z + 0.005), SHUT, "turn the L-key back half a turn", d.down(Y1 + 45.0 * (q + 1))) for q in range(4)],
                      w(1.0, over(a0, z), SHUT, "the tip back into the socket", d.down(a0)),
                      w(0.3, over(a0, z), SHUT, "seat it", d.down(a0))]
        n = 8
        for k in range(n):  # the screw advances as it turns: the hand follows it down
            a = a0 + (Y1 - a0) * (k + 1) / n
            z -= PITCH * abs(Y1 - a0) / n / 360.0
            right.append(w(0.9, over(a, z), SHUT, "turn the screw clockwise", d.down(a)))
    put = np.array([0.24, -0.22, TOP + 0.004 + LONG + LK_HELD])  # the L-key's tip 4 mm over the bench, clear of everything
    right += [
        w(0.5, over(Y1, z), SHUT, "hold", d.down(Y1)),
        w(1.0, over(Y1, z + 0.011), SHUT, "the L-key out of the screw", d.down(Y1)),
        w(2.0, (sock[0], -0.20, z + 0.011), SHUT, "out beside the sheave", d.down(Y1)),
        w(2.0, put + [0, 0, 0.03], SHUT, "over the bench", d.down(Y1)),
        w(1.0, put, SHUT, "down to the bench", d.down(Y1)),
        w(0.5, put, OPEN, "put the L-key down", d.down(Y1)),
        w(1.0, put + [0, 0, 0.05], OPEN, "up", d.down(Y1)),
        w(2.0, (0.15, -0.25, TOP + 0.20), OPEN, "the right hand clear", g0),
        w(4.0, (0.15, -0.25, TOP + 0.20), OPEN, "hands clear", g0),
    ]
    t_right = sum(p.duration for p in right)
    left.append(w(max(t_right - t_left, 0.5), (0.15, 0.25, TOP + 0.20), SHUT, "hands clear", g0))
    return {"left": left, "right": right}


def demo():
    return demo_plan()
