"""On a packing bench: a filled 5-gallon open-head steel pail (Uline S-20030: 28-gauge steel, 11 3/4 in outside diameter at
the top bead, 13.4 in tall) with the cover of its lever-lock lid (Uline S-21135: steel cover with a rubber gasket, steel
lever-lock ring) set on it. The cover rides on its uncompressed gasket, 6.5 mm above its seated height. The ring, open, has been
slipped over the cover as the maker's step 3 directs: its channel rests on the cover's curl, and its lever stands open 120
degrees at the front right of the pail, ready to close clockwise. Four strips of blue tape on the pail's side below the bead
mark the far, left, near and right measuring points. A roll of packing tape, a utility knife, a marker and a pair of work
gloves lie on the bench around it.

The maker's closing instructions (Uline IS-21135) continue: "Apply downward pressure to the cover and release the lever,
allowing the ring to slide onto the cover/curl edge", then "Close the ring clockwise by applying pressure to the lever until
it collapses onto the ring." Here the ring's lower lip clears the pail's bead only while the cover is pressed down onto its
gasket; closed without that, the band draws tight on top of the bead and the ring stands about 5 mm proud.
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Close the lever-lock ring to lock the lid on the pail."

# -- dimensions (metres) ------------------------------------------------------------------------------------------------
INCH = 0.0254
H = 13.4 * INCH  # pail height without lid (S-20030): the top of the rolled bead
R_BOTTOM = 10.75 * INCH / 2  # outside radius at the bottom
R_WALL = 0.1432  # outside radius of the wall just under the bead (11 1/4 in inside diameter + 28 ga wall)
R_BEAD = 11.75 * INCH / 2  # outside radius of the rolled bead: 0.1492
BEAD_H = 0.006  # the bead's height (6 mm roll)
R_LID = 12.0 * INCH / 2  # the cover's outside (curl) radius: 0.1524
R_PANEL = 0.1440  # where the curl meets the cover's panel
CURL_H = 0.007  # the curl's depth, top to skirt edge
SEATED_TOP = H + 0.003  # the curl's top with the gasket squeezed flat
GASKET_FREE = 0.0065  # a cover set on loose rests this far above its seated height (an estimate)
LOOSE_TOP = SEATED_TOP + GASKET_FREE
GASKET_K = 3000.0  # N/m for the whole gasket: about 19 N squeezes it the full 6.5 mm (an estimate)
LID_MASS = 0.65  # an estimate
PAIL_MASS = 20.0  # 1.1 kg pail plus 18 L of water-based contents

# The ring: frame = the band's top web and outer wall (rigid), four lower-lip segments that slide in as the band closes.
R_WEB_IN, R_WALL_IN, R_WALL_OUT = 0.1445, 0.1530, 0.1545
SHEET = 0.0015  # collider thickness for the thin steel band
CHANNEL = 0.0085  # from the web's underside to the lip's top: the curl plus the bead, seated
R_LIP_OPEN = 0.1534  # the lip's inner edge with the lever open (clears the 0.1524 curl)
LIP_TRAVEL = 0.0080  # band contraction at the lip: closed, the inner edge is at 0.1454 mid-quadrant, 3.8 mm under the bead
LIP_K, LIP_PRELOAD = 1500.0, 0.003  # N/m; a closed band pushes its lip 3 mm past the stop (4.5 N)
RING_MASS = 0.31  # an estimate
LIP_MASS, LEVER_MASS = 0.03, 0.03
QUAD_HALF = 37.5  # each lip segment spans its quadrant's centre +- this (degrees)
LIPS = {"lip_far": ("X", -1, 0.0), "lip_left": ("Y", -1, 90.0), "lip_near": ("X", 1, 180.0), "lip_right": ("Y", 1, 270.0)}
MARKS = {"far": 0.0, "left": 90.0, "near": 180.0, "right": 270.0}  # tape marks, azimuth in the pail's frame

R_LEVER = 0.1570  # the lever's centreline, closed, hugging the band
LEVER_LEN = 0.075
A_TIP = 225.0  # the closed lever's tip (azimuth): the front right of the pail
A_PIVOT = A_TIP + math.degrees(LEVER_LEN / R_LEVER)  # 252.4
LEVER_SWING = 120.0  # open to closed, clockwise seen from above
CENTRE = 100.0  # the toggle goes over centre here (degrees of closing)
LEVER_K = 0.4  # N m/rad: the band's pull back towards open: 0.7 N m just before centre, about 9 N at the lever's end
LATCH_K = 3.0  # N m/rad: over centre, the toggle holds the lever shut against its stop
CLOSED_TOL = 10.0  # "lever within 10 degrees of the band"

PX, PY = 0.36, 0.0  # the pail's axis on the bench


def _sector(r0, r1, a0, a1, z0, z1):
    """An annular sector as a convex extruded outline (degrees)."""
    a0, a1 = math.radians(a0), math.radians(a1)
    am = (a0 + a1) / 2  # outer corners on the arc; the inner edge is the chord, so keep sectors narrow near a fit
    pts = [(r0 * math.cos(a0), r0 * math.sin(a0)), (r1 * math.cos(a0), r1 * math.sin(a0)), (r1 * math.cos(am), r1 * math.sin(am)),
           (r1 * math.cos(a1), r1 * math.sin(a1)), (r0 * math.cos(a1), r0 * math.sin(a1))]
    return m.prism(pts, z0, z1)


def _ring_of(name, r0, r1, z0, z1, n=24, a0=0.0, a1=360.0):
    step = (a1 - a0) / n
    return [(f"{name}{i}", _sector(r0, r1, a0 + i * step, a0 + (i + 1) * step, z0, z1)) for i in range(n)]


LOOKS = {
    "pail": m.Look(m.rgb(0.55, 0.57, 0.58), metallic=0.6, roughness=0.45),
    "bead": m.Look(m.rgb(0.62, 0.64, 0.65), metallic=0.7, roughness=0.35),
    "lid": m.Look(m.rgb(0.58, 0.60, 0.61), metallic=0.6, roughness=0.4),
    "gasket": m.Look(m.rgb(0.12, 0.12, 0.12), roughness=0.9),
    "ring": m.Look(m.rgb(0.80, 0.81, 0.82), metallic=0.85, roughness=0.3),
    "lever": m.Look(m.rgb(0.74, 0.75, 0.77), metallic=0.85, roughness=0.3),
    "tape": m.Look(m.rgb(0.16, 0.42, 0.85), roughness=0.7),
    "wire": m.Look(m.rgb(0.45, 0.46, 0.47), metallic=0.8, roughness=0.4),
    "grip": m.Look(m.rgb(0.15, 0.15, 0.16), roughness=0.8),
}

# -- the pail and its cover (one articulation: the cover rides its gasket on a vertical slide) --------------------------------

_BODY = m.lathe([(0.0, 0.0), (R_BOTTOM, 0.0), (R_WALL, H - BEAD_H), (0.0, H - BEAD_H)], 48)
_BEAD = m.lathe([(0.0, H - BEAD_H), (R_WALL, H - BEAD_H), (0.1470, H - 0.0052), (0.1488, H - 0.004), (R_BEAD, H - 0.003),
                 (0.1488, H - 0.002), (0.1470, H - 0.0008), (R_WALL, H), (0.0, H)], 48)


def _pail_frame():
    parts = [("body", m.lathe([(0.0, 0.0), (R_BOTTOM, 0.0), (R_BOTTOM + 0.002, 0.004), (R_BOTTOM, 0.008), (R_WALL, H - BEAD_H),
                               (R_WALL - 0.001, H - 0.003), (0.0, H - 0.003)], 64), "pail"),
             ("bead", m.lathe([(R_WALL, H - BEAD_H), (0.1470, H - 0.0052), (0.1488, H - 0.004), (R_BEAD, H - 0.003), (0.1488, H - 0.002),
                               (0.1470, H - 0.0008), (R_WALL, H), (R_WALL - 0.001, H - 0.003)], 64), "bead")]
    for name, az in MARKS.items():  # blue tape strips below the bead
        a = math.radians(az)
        strip = m.box((0.0012, 0.025, 0.030), (0, 0, 0)).turned(az, "z").moved(((R_WALL + 0.0006) * math.cos(a), (R_WALL + 0.0006) * math.sin(a), H - 0.030))
        parts.append((f"tape_{name}", strip, "tape"))
    # the bail: ears at the left and right sides, the wire handle hanging down the far side (seen only; it is not touched)
    for k, az in enumerate((90.0, 270.0)):
        a = math.radians(az)
        parts.append((f"ear{k}", m.box((0.004, 0.028, 0.032), (0, 0, 0)).turned(az, "z").moved(((R_WALL + 0.002) * math.cos(a), (R_WALL + 0.002) * math.sin(a), H - 0.040)), "pail"))
    bail = []
    for t in np.linspace(0.0, 1.0, 25):
        az = math.radians(90.0 - 180.0 * t)
        z = H - 0.040 - 0.16 * math.sin(math.pi * t)
        r = (R_BOTTOM + (R_WALL - R_BOTTOM) * z / H) + 0.009
        bail.append((r * math.cos(az), r * math.sin(az), z))
    parts.append(("bail", m.tube(bail, 0.0024, 10), "wire"))
    grip_mid = len(bail) // 2
    parts.append(("bail_grip", m.tube(bail[grip_mid - 3:grip_mid + 4], 0.008, 12), "grip"))
    cols = [m.col_hull("body_c", _BODY), m.col_hull("bead_c", _BEAD)]
    return parts, cols


def _lid_link():
    z1 = LOOSE_TOP
    parts = [("panel", m.cylinder(R_PANEL, 0.002, (0, 0, z1 - 0.004), segments=64), "lid"),
             ("rib1", m.lathe([(0.10, z1 - 0.003), (0.104, z1 - 0.0015), (0.108, z1 - 0.003)], 64), "lid"),
             ("rib2", m.lathe([(0.125, z1 - 0.003), (0.129, z1 - 0.0015), (0.133, z1 - 0.003)], 64), "lid"),
             ("curl", m.lathe([(R_PANEL, z1 - 0.005), (R_PANEL + 0.002, z1 - 0.001), (0.148, z1), (R_LID - 0.0012, z1 - 0.0015),
                               (R_LID, z1 - 0.004), (R_LID - 0.0004, z1 - CURL_H), (R_LID - 0.0015, z1 - CURL_H), (R_LID - 0.0015, z1 - 0.004),
                               (0.148, z1 - 0.0015), (R_PANEL + 0.002, z1 - 0.003), (R_PANEL, z1 - 0.005)], 64), "lid"),
             ("gasket", m.lathe([(R_BEAD - 0.004, H + 0.0005), (R_BEAD - 0.0005, H + 0.0005), (R_BEAD - 0.0005, z1 - 0.0016),
                                 (R_BEAD - 0.004, z1 - 0.0016)], 64), "gasket")]
    cols = [m.col_hull("panel_c", m.cylinder(R_PANEL, 0.004, (0, 0, z1 - 0.003), segments=48))]
    cols += [m.col_hull(f"curl_c{i}", s) for i, (n, s) in enumerate(_ring_of("c", R_PANEL, R_LID, z1 - CURL_H, z1, n=48))]
    return m.slider("lid", "Z", -GASKET_FREE - 0.0005, 0.0005, parts, cols, at=(0.0, 0.0, z1),
                    mass=LID_MASS, spring=GASKET_K, damping=60.0, effort=400.0, friction=0.0)


_PAIL_PARTS, _PAIL_COLS = _pail_frame()
PAIL = m.mechanism("pail", (PX, PY, TOP), [_lid_link()], LOOKS, frame_parts=_PAIL_PARTS, frame_colliders=_PAIL_COLS,
                   friction=(0.6, 0.5), solver_iters=48, free_mass=PAIL_MASS, frame_inertia=(0.30, 0.30, 0.21))

# -- the lever-lock ring (its own free articulation), authored open, resting on the loose cover ------------------------------

RING_Z0 = LOOSE_TOP + 0.0005  # the ring's origin: on the pail's axis, at the web's underside
LIP_Z0, LIP_Z1 = -CHANNEL - SHEET, -CHANNEL
LEV_Z0, LEV_Z1 = -0.0095, 0.0045


def _rot_about(pts, centre, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    p = np.asarray(pts, float) - centre
    return np.c_[c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1], p[:, 2]] + centre


PIVOT = np.array([R_LEVER * math.cos(math.radians(A_PIVOT)), R_LEVER * math.sin(math.radians(A_PIVOT)), 0.0])


def _lever_open(mesh):
    """A mesh authored on the closed lever, swung open about the pivot."""
    return m.Mesh(_rot_about(mesh.v, PIVOT, LEVER_SWING), mesh.f)


def _ring():
    web = _ring_of("web", R_WEB_IN, R_WALL_OUT, 0.0, SHEET)
    wall = _ring_of("wall", R_WALL_IN, R_WALL_OUT, LIP_Z0, SHEET, n=72)
    frame_parts = [(n, s, "ring") for n, s in web + wall]
    frame_cols = [m.col_hull(f"{n}_c", s) for n, s in web + wall]
    # the lever's knuckle on the band
    frame_parts.append(("knuckle", m.cylinder(0.0045, LEV_Z1 - LEV_Z0 + 0.004, (PIVOT[0], PIVOT[1], (LEV_Z0 + LEV_Z1) / 2), segments=16), "ring"))
    links = []
    for name, (axis, sign, az) in LIPS.items():
        segs = _ring_of(name, R_LIP_OPEN, R_LIP_OPEN + 0.008, LIP_Z0, LIP_Z1, n=15, a0=az - QUAD_HALF, a1=az + QUAD_HALF)
        lo, hi = (-LIP_TRAVEL, 0.0003) if sign < 0 else (-0.0003, LIP_TRAVEL)
        links.append(m.slider(name, axis, lo, hi, [(n, s, "ring") for n, s in segs], [m.col_hull(f"{n}_c", s) for n, s in segs],
                              mass=LIP_MASS, spring=LIP_K, damping=4.0, effort=60.0, friction=0.0))
    # the lever: a curved strap following the band when closed, authored open, with a turned-out tab at its end
    arc = _ring_of("lev", R_LEVER - 0.0015, R_LEVER + 0.0015, LEV_Z0, LEV_Z1, n=5, a0=A_TIP, a1=A_PIVOT)
    lever_parts = [(n, _lever_open(s), "lever") for n, s in arc]
    lever_parts.append(("lever_tab", _lever_open(_sector(R_LEVER + 0.0015, R_LEVER + 0.0065, A_TIP + 1.0, A_TIP + 9.0, LEV_Z0 + 0.003, LEV_Z1 - 0.003)), "lever"))
    links.append(m.hinge("lever", "Z", tuple(PIVOT), -LEVER_SWING, 3.0, lever_parts, [m.col_hull(f"{n}_c", s) for n, s, _ in lever_parts],
                         mass=LEVER_MASS, spring=LEVER_K, damping=0.02, effort=20.0, friction=0.002))
    return frame_parts, frame_cols, links


_RING_PARTS, _RING_COLS, _RING_LINKS = _ring()
RING_FRAME_MASS = RING_MASS - 4 * LIP_MASS - LEVER_MASS
RING = m.mechanism("ring", (PX, PY, TOP + RING_Z0), _RING_LINKS, LOOKS, frame_parts=_RING_PARTS, frame_colliders=_RING_COLS,
                   friction=(0.5, 0.4), solver_iters=48, free_mass=RING_FRAME_MASS, frame_inertia=(0.0019, 0.0019, 0.0037))

OBJECTS = [
    PAIL,
    RING,
    # Distractors on the bench (SimReady props), which scoring ignores. pos is each one's origin, on the table top.
    SimReady("tape_roll", pos=(0.66, 0.22, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
             package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd"),
    SimReady("utility_knife", pos=(0.66, -0.06, TOP), size=(0.160, 0.061, 0.026), mass=0.12, yaw=math.radians(80.0),
             package="handTool_knife_retractableBlade_a01", usd="sm_handTool_knife_retractableBlade_a01_01.usd"),
    SimReady("marker", pos=(0.12, 0.02, TOP), size=(0.109, 0.008, 0.007), mass=0.01, yaw=1.2,
             package="stationery_marker_thin_blue_a01", usd="sm_stationery_marker_thin_blue_a01_01.usd"),
    SimReady("gloves", pos=(0.64, 0.40, TOP), size=(0.247, 0.231, 0.016), mass=0.06, yaw=math.radians(-20.0),
             package="ppe_gloves_lightDuty_a01", usd="sm_ppe_gloves_lightDuty_a01_01.usd"),
]

# -- the ring's mechanism: over-centre lever, contracting band ---------------------------------------------------------------


class Band:
    """Per control step: the toggle and the band's contraction. Every state change follows the joints' own positions, which
    contact decides; nothing is triggered by where a gripper is."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.latched = False  # the lever is over centre
        self.lips_locked = False  # the band is drawn tight: its lips stay where contact let them go
        self.tight_steps = 0

    def step(self, env):
        s = m.state(env, "ring")
        th = -s.deg("lever")  # degrees of closing, 0 open .. 120 shut
        if not self.latched and th >= CENTRE:
            self.latched = True
            m.lock(env, "ring", "lever", stiffness=LATCH_K, effort=20.0)
            m.set_target(env, "ring", "lever", -math.radians(LEVER_SWING + 5.0))
        elif self.latched and th < CENTRE:
            self.latched = False  # pulled back over centre: the band's pull swings it open again
            m.unlock(env, "ring", "lever")
        if self.latched:
            if not self.lips_locked:
                for name, (axis, sign, az) in LIPS.items():
                    m.set_target(env, "ring", name, sign * (LIP_TRAVEL + LIP_PRELOAD))
                self.tight_steps += 1
                if self.tight_steps >= 6 and th >= LEVER_SWING - CLOSED_TOL:
                    for name in LIPS:
                        m.lock(env, "ring", name)
                    self.lips_locked = True
        else:
            if self.lips_locked:
                for name in LIPS:
                    m.unlock(env, "ring", name)
                self.lips_locked = False
            self.tight_steps = 0
            f = min(max(th / CENTRE, 0.0), 1.0)
            for name, (axis, sign, az) in LIPS.items():
                m.set_target(env, "ring", name, sign * LIP_TRAVEL * f)


BAND = Band()
m.on_step(BAND.step)
m.on_reset(BAND.reset)

# -- scoring ------------------------------------------------------------------------------------------------------------

SUCCESS_HOLD_S = 0.0  # the Trial counts its own hold
HOLD_S = 1.0
R_MEASURE = 0.150  # where the ring's top is read at each mark: the middle of its web
SEATED_RING_TOP = SEATED_TOP + SHEET + 0.001  # the ring's top on a correctly sealed pail, above the pail's base: 0.3459 m
# (measured in the simulator: a closed ring holds the cover 1 mm short of squeezing its gasket fully flat)
SEAT_TOL = 0.002  # "within 2 mm of seated"; a ring closed over a loose cover stands about 5 mm higher
RING_OFFSET = 0.008
TIP_HARM_DEG = 10.0
UPRIGHT_DEG = 5.0
CLEAR = 0.03
CLEAR_R, CLEAR_Z0, CLEAR_Z1 = 0.17, H - 0.020, H + 0.035


def ring_tops(p, r):
    """The height of the ring's top at each tape mark, above the pail's base (pail frame), in metres."""
    o = p.to_local(r.pos)[0]
    n = p.rotation().T @ r.rotation()[:, 2]
    top = o + n * SHEET
    out = {}
    for k, az in MARKS.items():
        a = math.radians(az)
        x, y = R_MEASURE * math.cos(a), R_MEASURE * math.sin(a)
        out[k] = float(top[2] - (n[0] * (x - top[0]) + n[1] * (y - top[1])) / n[2])
    return out


def ring_on(p, r):
    c = p.to_local(r.pos)[0]
    return bool(math.hypot(c[0], c[1]) <= RING_OFFSET and H <= c[2] <= H + 0.015)


def lever_closed(r):
    return bool(-r.deg("lever") >= LEVER_SWING - CLOSED_TOL)


def hands_clear(p, tips):
    if tips is None:
        return False
    local = p.to_local(tips)
    rho = np.hypot(local[:, 0], local[:, 1])
    dr = np.maximum(rho - CLEAR_R, 0.0)
    dz = np.maximum(np.maximum(CLEAR_Z0 - local[:, 2], local[:, 2] - CLEAR_Z1), 0.0)
    return bool(np.hypot(dr, dz).min() >= CLEAR)


class Trial:
    """Updated once per control step; score() and success() only read it."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):  # the ONLY place a recorded harm is cleared
        self.tipped = False
        self.hold = 0.0
        self.full = False
        self.value = 0.0
        self.proud = 4

    def update(self, p, r, tips, dt):
        if p.tilt_deg() > TIP_HARM_DEG:
            self.tipped = True
        base = (not self.tipped and p.tilt_deg() <= UPRIGHT_DEG and ring_on(p, r) and lever_closed(r) and hands_clear(p, tips))
        tops = ring_tops(p, r)
        self.proud = sum(z > SEATED_RING_TOP + SEAT_TOL for z in tops.values())
        if not base:
            level = 0.0
        elif self.proud >= 2:
            level = 0.25
        elif self.proud == 1:
            level = 0.5
        else:
            level = 0.75
        still = (np.linalg.norm(r.lin_vel) < 0.01 and np.linalg.norm(r.ang_vel) < 0.1 and np.linalg.norm(p.lin_vel) < 0.01)
        ready = bool(level == 0.75 and still)
        self.hold = self.hold + float(dt) if ready else 0.0
        self.full = bool(ready and self.hold >= HOLD_S - 1e-9)
        self.value = 1.0 if self.full else level

    def step(self, env):
        p, r, tips = m.state(env, "pail"), m.state(env, "ring"), m.fingertips(env)
        self.update(p, r, tips, env.unwrapped.step_dt)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demo (closed loop, run by demokit.py) ------------------------------------------------------------------

DEBUG = False
FRAC = 0.85  # where along the lever (pivot 0, tip 1) the right hand pushes
LEV_MID = (LEV_Z0 + LEV_Z1) / 2


def lever_point(frac, closing_deg, z=LEV_MID):
    """A point on the lever's centreline in the ring's frame, `frac` of the way from pivot to tip, closed by closing_deg."""
    a = math.radians(A_PIVOT - frac * (A_PIVOT - A_TIP))
    p = np.array([[R_LEVER * math.cos(a), R_LEVER * math.sin(a), z]])
    return _rot_about(p, PIVOT, LEVER_SWING - closing_deg)[0]


def _report(ctx, tag):
    p, r = ctx.state("pail"), ctx.state("ring")
    tops = ring_tops(p, r)
    c = p.to_local(r.pos)[0]
    print(f"[seal] {tag} lid {p.joint('lid') * 1000:+.2f} mm, lever {-r.deg('lever'):.1f} deg, lips "
          + " ".join(f"{r.joint(n) * 1000:+.1f}" for n in LIPS)
          + f", ring off-axis {math.hypot(c[0], c[1]) * 1000:.1f} mm z {(c[2] - H) * 1000:+.2f} mm, tops above seated "
          + " ".join(f"{k} {(z - SEATED_RING_TOP) * 1000:+.2f}" for k, z in tops.items())
          + f" mm, pail tilt {p.tilt_deg():.2f} deg, pail xy ({p.pos[0]:.3f}, {p.pos[1]:.3f})", flush=True)


def demo_script(ctx):
    """Left: comes down on the cover just left of its centre with closed fingers and presses it onto its gasket (closed loop on
    the cover's seen height), and keeps it there. Right: comes down behind the open lever near its end and pushes it clockwise
    round its pivot, leading the lever a little and leading further when it stalls (closed loop on the lever's seen angle),
    until it has gone over centre and lies on the band; then backs off outward and up. The left lets the cover go, and both
    arms move away.
    Red-team modes: none (nothing), onearm (the right closes the lever, the cover is not pressed), late (the lever is closed
    first and the cover pressed only afterwards), sequential (the cover is pressed and let go, then the lever is closed), hold
    (the left never lets go of the cover), pressdown (one arm: the right bears down on the lever's top edge as it pushes it
    round, the cover not held)."""
    import demokit as d

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "onearm", "late", "sequential", "hold", "pressdown"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, f"{mode} start:")
    if mode == "none":
        for _ in range(60):
            yield
        _report(ctx, "end:")
        return

    shared = {"pressed": False, "release": False, "freeze": False, "lever_done": False}
    grip_l = d.tilted(45.0, -45.0)
    grip_r = d.tilted(50.0, 60.0)
    PAD = 0.013

    def pail_world(pt):
        return ctx.state("pail").to_world(np.asarray(pt, float))[0]

    def ring_world(pt):
        return ctx.state("ring").to_world(np.asarray(pt, float))[0]

    def press():
        """Left: press the cover down onto its gasket and hold it until told to let go."""
        spot = np.array([-0.02, 0.03, 0.0])
        panel = LOOSE_TOP - 0.001  # the panel's top, uncompressed, in the pail frame
        ax = grip_l.rot[:, 2]
        above = pail_world(spot + np.array([0.0, 0.0, panel + 0.04])) - ax * PAD
        yield from l.move(above, grip=grip_l, finger=d.FINGER_CLOSED, t=2.5, label="left over the cover")
        near = pail_world(spot + np.array([0.0, 0.0, panel + 0.004])) - ax * PAD
        yield from l.move(near, t=1.2, label="left down to the cover")
        c = ik_tcp(l)
        target_mm = -(GASKET_FREE * 1000.0 - 0.7)
        for _ in range(int(40.0 * 20)):
            if shared["release"]:
                break
            if shared["freeze"]:
                return  # the left arm stays where it is, still on the cover
            lid_mm = ctx.joint("pail", "lid") * 1000.0
            if lid_mm <= target_mm + 0.3:
                shared["pressed"] = True
            err = lid_mm - target_mm  # positive: not far enough down
            c = c + np.array([0.0, 0.0, -float(np.clip(0.0008 * err, -0.001, 0.0015))])
            l._solve(c, grip_l.rot)
            yield
        _report(ctx, "cover let go:")
        lift = ik_tcp(l) + np.array([0.0, 0.0, 0.05])
        yield from l.move(lift, t=1.0, label="left off the cover")
        yield from l.move(np.array([0.25, 0.33, TOP + 0.30]), t=1.5, label="left away")

    def ik_tcp(arm):
        from yam_arena import ik
        return ik.tcp_pose(arm.q, arm.base)[0]

    def close_lever():
        """Right: push the lever round clockwise from behind until it is over centre and lies on the band."""
        ax = grip_r.rot[:, 2]

        def tcp_for(closing, back_mm=0.0, up=0.0):
            pt = lever_point(FRAC, closing, LEV_Z1 + 0.002 if mode == "pressdown" else LEV_MID)
            v = pt - PIVOT
            mdir = np.array([v[1], -v[0], 0.0]) / math.hypot(v[0], v[1])  # the way this point moves as the lever closes
            w = ring_world(pt - mdir * back_mm / 1000.0 + np.array([0.0, 0.0, up]))
            return w - ax * PAD

        th0 = -ctx.deg("ring", "lever")
        yield from r.move(tcp_for(th0 - 25.0, 0.0, 0.03), grip=grip_r, finger=d.FINGER_CLOSED, t=2.5, label="right over behind the lever")
        while mode in ("full", "hold") and not shared["pressed"]:
            yield
        th0 = -ctx.deg("ring", "lever")
        yield from r.move(tcp_for(th0 - 25.0, 0.0, 0.0), t=1.0, label="right down behind the lever")
        c = ik_tcp(r)
        lead, extra, best, stall = 14.0, 0.0, th0, 0
        for k in range(int(12.0 * 20)):
            th = -ctx.deg("ring", "lever")
            if th > best + 0.5:
                best, stall = th, 0
            else:
                stall += 1
                if stall > 8:  # stalled: lead further, and push past the lever (the arm's own give takes up the difference)
                    lead, extra, stall = min(lead + 3.0, 30.0), min(extra + 0.006, 0.05), 0
            if th >= LEVER_SWING - 2.0:
                break
            ahead = min(th + lead, LEVER_SWING + 10.0)
            pt = lever_point(FRAC, ahead)
            v = pt - PIVOT
            mdir = ring_world(np.array([v[1], -v[0], 0.0]) / math.hypot(v[0], v[1])) - ring_world(np.zeros(3))
            goal = tcp_for(ahead) + mdir * extra
            if mode == "pressdown":  # bear down on the lever's top edge as it is pushed round
                goal = goal - np.array([0.0, 0.0, 0.040])
            c = c + np.clip(goal - c, -0.004, 0.004)
            r._solve(c, grip_r.rot)
            if DEBUG and k % 10 == 0:
                act = r.tcp()
                lev = ring_world(lever_point(FRAC, th))
                print(f"[push] k {k} th {th:.1f} lead {lead:.0f} cmd-act {np.round((c - act) * 1000, 1)} mm, tip-lever "
                      f"{np.round((act + ax * PAD - lev) * 1000, 1)} mm", flush=True)
            yield
        for _ in range(10):
            yield
        _report(ctx, "lever shut:")
        shared["lever_done"] = True
        out = ik_tcp(r)
        pt = lever_point(FRAC, LEVER_SWING)
        radial = ring_world(np.array([pt[0], pt[1], 0.0])) - ring_world(np.zeros(3))
        radial = radial / np.linalg.norm(radial)
        yield from r.move(out + radial * 0.03, t=0.8, label="right out from the lever")
        yield from r.move(out + radial * 0.04 + np.array([0.0, 0.0, 0.05]), t=0.8, label="right up")
        yield from r.move(np.array([0.25, -0.30, TOP + 0.25]), t=1.5, label="right away")

    if mode in ("full", "hold"):
        def left_all():
            yield from press()

        def right_all():
            yield from close_lever()
            for _ in range(10):
                yield
            shared["freeze" if mode == "hold" else "release"] = True

        yield from ctx.together(left_all(), right_all())
    elif mode in ("onearm", "pressdown"):
        yield from close_lever()
    elif mode in ("late", "sequential"):
        def stop_later():
            for _ in range(int(8.0 * 20)):
                if shared["pressed"]:
                    break
                yield
            for _ in range(30):
                yield
            shared["release"] = True

        if mode == "late":
            yield from close_lever()
        yield from ctx.together(press(), stop_later())
        if mode == "sequential":
            yield from close_lever()
    _report(ctx, "hands away:")
    for _ in range(40):
        yield
    _report(ctx, "end:")
