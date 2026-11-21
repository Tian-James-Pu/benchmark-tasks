"""On the station table: a RIDGID 604 lever tube bender (1/4 in tube, 5/8 in bend radius) clamped by its vise clamp block in a
bench vise, bend plane level. A 360 mm length of 1/4 in soft copper tube lies in the form's groove under the tube latch, with two
black bend marks 70.7 mm apart. The carriage handle is swung 15 degrees away from the form, so its rollers are off the tube.

A 45 degree offset needs two opposite 45 degree bends, each centred on its mark. Between the bends the tube has to be slid along
the groove to the second mark and turned half a turn about its own axis; bending it again without turning it makes a U, not an
offset. Every bend is one way: copper cannot be unbent.

The bender: the lever (carriage handle) is a hinge about the form's axis. While its rollers press the tube (lever at 0 degrees
or more), the tube cannot slide or turn, and the lever is resisted by the tube's plastic bending moment, through a stop that
follows the tube's set angle (an effort-capped spring: `Bend`). The tube is a chain of 4 mm links on hinges in the plane of
its marks; the scorer reads its shape from those hinges."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
INSTRUCTION = "Bend a 45 degree offset in the copper tube, with its two bends centred on the black marks."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = station.TABLE_TOP

# -- the bender and the tube (bender frame: origin on the form's axis at the top of the clamp block, z up) --------------------
PIVOT = (0.31, 0.0)
FRAME_Z = 0.11  # the frame origin above the bench: the clamp block sits in the vise's jaws, whose tops are 9.9 cm up
VISE_SCALE = 0.55  # the SimReady 5.5 in bench vise drawn at 3 in (README)
R = 0.015875  # 5/8 in bend radius to the tube's centre line (RIDGID 604)
TUBE_D = 0.00635  # 1/4 in OD
TUBE_R = TUBE_D / 2
ZT = 0.024  # the tube's centre line above the frame origin
HANDLE_X = 0.015  # each handle's centre line, beside the form's axis
LEVER_Z, LEVER_GRIP_Z = 0.006, 0.014  # the carriage handle rides 8 mm clear of the bench
HANDLE_LEN = 0.36  # each handle, from the form's axis (estimate: the tool ships in a 15.5 in carton)
GRIP_AT = 0.295  # the cushion grips' middle, from the form's axis
GRIP_LEN = 0.14
LEVER0 = -15.0  # the lever's start: 15 degrees back from the form's 0 line, rollers off the tube
TUBE_LEN = 0.360
MARK_B, MARK_A = 0.150, 0.150 + 0.0707  # bend marks, from end E2 (on the form handle's side); 70.7 mm = 50 mm offset x 1.414
ARC45 = R * math.pi / 4  # 18.7 mm of tube in a 45 degree bend
START_SHORT = 0.025  # at the start mark A is 25 mm short of the carriage's 45 degree line
Y_E2 = ARC45 / 2 - START_SHORT - MARK_A  # end E2's y in the bender frame at slide 0
HINGE_STEP = 0.004
HINGES = [round(0.110 + HINGE_STEP * k, 4) for k in range(39)]  # 0.110 .. 0.262 m from E2
M_P = 2.5  # N m: the plastic bending moment of 1/4 x 0.030 in annealed copper (README: estimate)
SPRINGBACK = 1.0  # degrees of elastic springback after a bend (README)
K_LEVER = M_P / math.radians(SPRINGBACK)

# -- scoring --------------------------------------------------------------------------------------------------------------
TARGET, TOL_FULL, TOL_PART = 45.0, 3.0, 7.0
PARALLEL_FULL, PARALLEL_PART = 4.0, 8.0
MARK_FULL, MARK_PART = 0.005, 0.012
OFFSET, OFFSET_TOL = 0.0505, 0.004
SCRAP_DEG = 55.0
SKEW_DEG = 15.0
CLEAR = 0.03
SEEN_DEG = 0.3  # a hinge carrying more than this belongs to a bend

LOOKS = {"steel": m.Look(m.rgb(0.6, 0.62, 0.64), metallic=0.85, roughness=0.35), "dark": m.Look(m.rgb(0.16, 0.17, 0.18), metallic=0.6, roughness=0.5),
         "red": m.Look(m.rgb(0.75, 0.08, 0.06), roughness=0.55), "grip": m.Look(m.rgb(0.08, 0.08, 0.08), roughness=0.85), "copper": m.Look(m.rgb(0.83, 0.47, 0.27), metallic=0.85, roughness=0.3),
         "ink": m.Look(m.rgb(0.02, 0.02, 0.02), roughness=0.9), "white": m.Look(m.rgb(0.95, 0.95, 0.92), roughness=0.6), "yellow": m.Look(m.rgb(0.95, 0.8, 0.1), roughness=0.6)}


def _rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _turned(mesh: m.Mesh, deg: float) -> m.Mesh:
    return m.Mesh(mesh.v @ _rz(deg).T, mesh.f)


def _line(angle_deg: float, r0: float, r1: float, z: float, w: float = 0.0012) -> m.Mesh:
    """A thin radial line on the form's top, at this angle from +x (the 0 line is +x, where the tube leaves the form)."""
    return _turned(m.box((r1 - r0, w, 0.0006), ((r0 + r1) / 2, 0.0, z)), angle_deg)


def _seg(u0: float, u1: float, r: float = TUBE_R) -> m.Mesh:
    return m.cylinder(r, u1 - u0, (R, Y_E2 + (u0 + u1) / 2, ZT), axis="y", segments=16)


def _tube_link(name: str, u0: float, u1: float, kind: str, parent: str | None, at_u: float | None) -> m.Link:
    parts = [(f"{name}_t", _seg(u0, u1), "copper")]
    for k, mark in enumerate((MARK_B, MARK_A)):
        if u0 <= mark < u1:
            parts.append((f"{name}_mark{k}", _seg(max(u0, mark - 0.002), min(u1, mark + 0.002), TUBE_R + 0.0003), "ink"))
    cols = [m.col_hull(f"{name}_c", _seg(u0 + 0.0002, u1 - 0.0002))]
    mass = 0.003 if u1 - u0 < 0.01 else 0.02
    if kind == "roll":
        return m.hinge(name, "Y", (R, 0.0, ZT), -60.0, 240.0, parts, cols, mass=mass, spring=0.0, damping=0.02, friction=0.01, effort=50.0, armature=0.0005, parent=parent)
    return m.hinge(name, "Z", (R, Y_E2 + at_u, ZT), -60.0, 60.0, parts, cols, mass=mass, spring=30.0, damping=0.05, friction=0.0, effort=20.0, armature=0.0003, parent=parent)


def _bender() -> m.Part:
    frame_parts = [
        ("form_core", m.cylinder(R - TUBE_R, ZT + TUBE_R + 0.008, (0.0, 0.0, (ZT + TUBE_R) / 2), segments=40), "steel"),
        ("form_lower", m.cylinder(R + 0.0057, ZT - TUBE_R + 0.008, (0.0, 0.0, (ZT - TUBE_R - 0.008) / 2), segments=48), "steel"),
        ("clamp_block", m.box((0.025, 0.045, 0.030), (-HANDLE_X, -0.050, -0.021)), "dark"),
        ("form_upper", m.cylinder(R + 0.0057, 0.004, (0.0, 0.0, ZT + TUBE_R + 0.002), segments=48), "steel"),
        ("deg0", _line(0.0, R - 0.002, R + 0.0057, ZT + TUBE_R + 0.0043), "ink"),
        ("deg45", _line(45.0, R - 0.002, R + 0.0057, ZT + TUBE_R + 0.0043), "ink"),
        ("deg90", _line(90.0, R - 0.002, R + 0.0057, ZT + TUBE_R + 0.0043), "ink"),
        ("form_handle", m.box((0.022, HANDLE_LEN - 0.03, 0.012), (-HANDLE_X, -(0.03 + HANDLE_LEN) / 2, -0.002)), "red"),
        ("form_grip", m.cylinder(0.014, GRIP_LEN, (-HANDLE_X, -GRIP_AT, 0.006), axis="y", segments=20), "grip"),
    ]
    frame_cols = [m.col_cylinder("form_c", R + 0.0057, ZT - TUBE_R + 0.008, (0.0, 0.0, (ZT - TUBE_R - 0.008) / 2)), m.col_box("handle_c", (0.022, HANDLE_LEN - 0.03, 0.012), (-HANDLE_X, -(0.03 + HANDLE_LEN) / 2, -0.002)),
                  m.col_hull("form_grip_c", m.cylinder(0.014, GRIP_LEN, (-HANDLE_X, -GRIP_AT, 0.006), axis="y", segments=16))]
    # the carriage and its handle, drawn at lever angle 0 (rollers on the tube at the form's 0 line) and turned to the start
    roll_x = R + TUBE_R + 0.0065
    carriage = [("car_side", m.box((0.010, 0.056, 0.026), (roll_x + 0.0115, 0.019, ZT)), "steel"),
                ("car_top", m.box((0.026, 0.056, 0.003), (roll_x + 0.0035, 0.019, ZT + 0.0075)), "steel"),
                ("roller0", m.cylinder(0.0065, 0.012, (roll_x, 0.006, ZT)), "dark"),
                ("roller1", m.cylinder(0.0065, 0.012, (roll_x, 0.034, ZT)), "dark"),
                ("line0", m.box((0.018, 0.0012, 0.0006), (roll_x + 0.0035 - 0.004, 0.0, ZT + 0.0093)), "ink"),
                ("line45", m.box((0.018, 0.0012, 0.0006), (roll_x + 0.0035 - 0.004, ARC45 / 2, ZT + 0.0093)), "yellow"),
                ("arm", m.box((0.012, 0.030, 0.012), (roll_x + 0.0115, 0.062, LEVER_Z)), "steel"),
                ("lever_bar", m.box((0.022, HANDLE_LEN - 0.07, 0.012), (HANDLE_X, (0.07 + HANDLE_LEN) / 2, LEVER_Z)), "red"),
                ("lever_link", m.box((roll_x + 0.0115 - HANDLE_X, 0.012, 0.012), ((roll_x + 0.0115 + HANDLE_X) / 2, 0.072, LEVER_Z)), "steel"),
                ("lever_grip", m.cylinder(0.014, GRIP_LEN, (HANDLE_X, GRIP_AT, LEVER_GRIP_Z), axis="y", segments=20), "grip")]
    carriage = [(n, _turned(mesh, LEVER0), k) for n, mesh, k in carriage]
    lever_cols = [m.col_hull("lever_bar_c", _turned(m.box((0.022, HANDLE_LEN - 0.07, 0.012), (HANDLE_X, (0.07 + HANDLE_LEN) / 2, LEVER_Z)), LEVER0)),
                  m.col_hull("lever_grip_c", _turned(m.cylinder(0.014, GRIP_LEN, (HANDLE_X, GRIP_AT, LEVER_GRIP_Z), axis="y", segments=16), LEVER0))]
    lever = m.hinge("lever", "Z", (0.0, 0.0, 0.0), -10.0, 110.0, carriage, lever_cols, mass=0.25, spring=0.0, damping=0.3, friction=0.05, effort=200.0, armature=0.002)
    # the tube latch: a hooked plate over the tube on the form handle's side, sprung shut
    lp = (R + 0.017, -0.032)
    latch = m.hinge("latch", "Z", (lp[0], lp[1], ZT), -5.0, 90.0,
                    [("latch_plate", m.box((0.030, 0.010, 0.002), (lp[0] - 0.012, lp[1], ZT + TUBE_R + 0.0015)), "steel"),
                     ("latch_post", m.cylinder(0.004, 0.016, (lp[0], lp[1], ZT)), "dark")], [], mass=0.02, spring=0.2, damping=0.02, effort=5.0)
    # the tube: a slide along the groove, a turn about its own axis, then 4 mm links on hinges in the plane of its marks
    links = [lever, latch, m.slider("tslide", "Y", -0.06, 0.16, [], [], at=(R, 0.0, ZT), mass=0.01, damping=2.0, friction=0.3, effort=500.0),
             _tube_link("troll", 0.0, HINGES[0], "roll", "tslide", None)]
    parent = "troll"
    for k, u in enumerate(HINGES):
        u1 = HINGES[k + 1] if k + 1 < len(HINGES) else TUBE_LEN
        links.append(_tube_link(f"t{k:02d}", u, u1, "hinge", parent, u))
        parent = f"t{k:02d}"
    return m.mechanism("bender", (*PIVOT, TOP + FRAME_Z), links, LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols, solver_iters=32,
                       friction=(0.6, 0.5))


SQUARE = SimReady("speed_square", pos=(0.62, -0.02, TOP), size=(0.29, 0.22, 0.008), mass=0.15, package="measurement_ruler_speedsquare_a02", usd="sm_measurement_ruler_speedsquare_a02_01.usd", yaw=0.3)
PLIERS = SimReady("pliers", pos=(0.56, 0.30, TOP), size=(0.19, 0.07, 0.026), mass=0.32, package="handTool_pliers_combination_a01", usd="sm_handTool_pliers_combination_a01_01.usd", yaw=1.2, convex=True)
DRIVER = SimReady("screwdriver", pos=(0.58, -0.28, TOP), size=(0.22, 0.04, 0.036), mass=0.12, package="handTool_screwdriver_slotted_65mm_a01", usd="sm_handTool_screwdriver_slotted_65mm_a01_01.usd", yaw=-0.4, convex=True)
VISE = SimReady("vise", pos=(PIVOT[0] - HANDLE_X - 0.106 * VISE_SCALE, PIVOT[1] - 0.050, TOP), size=(0.178, 0.128, 0.099), package="equipment_bench_vise_a01",
               usd="sm_equipment_bench_vise_a01_01.usd", scale=VISE_SCALE)
OBJECTS = [_bender(), VISE, SQUARE, PLIERS, DRIVER]


# -- the tube's shape, from its joints --------------------------------------------------------------------------------------


def bends_from(hinge_deg: np.ndarray) -> list[tuple[float, float]]:
    """(signed angle in degrees, curve middle in m from E2) for each run of hinges carrying a bend."""
    out, run = [], []
    for k, a in enumerate(hinge_deg):
        if abs(a) > SEEN_DEG and (not run or np.sign(a) == np.sign(hinge_deg[run[-1]])) and (not run or k - run[-1] <= 2):
            run.append(k)
            continue
        if run:
            out.append(run)
        run = [k] if abs(a) > SEEN_DEG else []
    if run:
        out.append(run)
    res = []
    for run in out:
        a = np.array([hinge_deg[k] for k in run])
        u = np.array([HINGES[k] for k in run])
        res.append((float(a.sum()), float((u * np.abs(a)).sum() / np.abs(a).sum())))
    return res


def centre_line(hinge_deg: np.ndarray) -> np.ndarray:
    """The tube's centre line in its own flat frame: x across, y along, E2 at the origin, 1 mm apart up to the last link."""
    pts, d, p, u = [np.zeros(2)], np.array([0.0, 1.0]), np.zeros(2), 0.0
    stops = list(HINGES) + [TUBE_LEN]
    k = 0
    while u < TUBE_LEN - 1e-9:
        nxt = min(stops[k], u + 0.001) if k < len(stops) else min(TUBE_LEN, u + 0.001)
        p = p + d * (nxt - u)
        u = nxt
        pts.append(p.copy())
        if k < len(HINGES) and abs(u - HINGES[k]) < 1e-9:
            a = math.radians(hinge_deg[k])
            d = np.array([d[0] * math.cos(a) - d[1] * math.sin(a), d[0] * math.sin(a) + d[1] * math.cos(a)])
            k += 1
    return np.array(pts)


def offset_of(hinge_deg: np.ndarray) -> float:
    """How far the E1 end's straight starts from the E2 end's centre line, measured square to it: the offset, for parallel ends."""
    bends = bends_from(hinge_deg)
    if not bends:
        return 0.0
    a, mid = max(bends, key=lambda b: b[1])
    u_end = mid + R * math.radians(abs(a)) / 2 + 0.002
    line = centre_line(hinge_deg)
    return float(abs(line[min(int(round(u_end * 1000)), len(line) - 1)][0]))  # 1 mm points; E2's straight runs along y through x = 0


def tube_world(st, hinge_deg: np.ndarray) -> np.ndarray:
    """The tube's centre line in the station frame, from the bender's pose and joints."""
    flat = centre_line(hinge_deg)
    slide, roll = st.joint("tslide"), st.joint("troll")
    local = np.stack([flat[:, 0], flat[:, 1], np.zeros(len(flat))], 1)  # x across, y along, z out of the plane
    c, s = math.cos(roll), math.sin(roll)
    rolled = np.stack([local[:, 0] * c + local[:, 2] * s, local[:, 1], -local[:, 0] * s + local[:, 2] * c], 1)
    frame = rolled + np.array([R, Y_E2 + slide, ZT])
    return st.to_world(frame)


def gap(points: np.ndarray, line: np.ndarray) -> float:
    """The least distance from any of the points to the polyline."""
    a, b = line[:-1], line[1:]
    best = np.inf
    for p in points:
        ab = b - a
        t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(p - (a + t[:, None] * ab), axis=1).min()))
    return best


def handle_lines(st) -> list[np.ndarray]:
    lever = math.radians(st.deg("lever") + LEVER0)
    rot = np.array([[math.cos(lever), -math.sin(lever)], [math.sin(lever), math.cos(lever)]])
    a, b = rot @ np.array([HANDLE_X, 0.05]), rot @ np.array([HANDLE_X, HANDLE_LEN])
    return [st.to_world(np.array([[a[0], a[1], LEVER_Z], [b[0], b[1], LEVER_Z]])), st.to_world(np.array([[-HANDLE_X, -0.03, 0.0], [-HANDLE_X, -HANDLE_LEN, 0.0]]))]


def grade(bends: list[tuple[float, float]], offset: float) -> float:
    """The score of a finished tube shape, before the stillness and hands-off conditions."""
    if len(bends) == 2:
        (a1, u1), (a2, u2) = sorted(bends, key=lambda b: b[1])
        if np.sign(a1) != np.sign(a2):
            par = abs(a1 + a2)
            dB, dA = abs(u1 - MARK_B), abs(u2 - MARK_A)
            near = all(abs(abs(a) - TARGET) <= TOL_FULL for a in (a1, a2))
            fair = all(abs(abs(a) - TARGET) <= TOL_PART for a in (a1, a2))
            if near and par <= PARALLEL_FULL and max(dA, dB) <= MARK_FULL and abs(offset - OFFSET) <= OFFSET_TOL:
                return 1.0
            if fair and par <= PARALLEL_PART and max(dA, dB) <= MARK_PART:
                return 0.75
            if fair and par <= PARALLEL_PART:
                return 0.5
        return 0.0
    if len(bends) == 1:
        a, u = bends[0]
        if abs(abs(a) - TARGET) <= TOL_FULL and min(abs(u - MARK_A), abs(u - MARK_B)) <= MARK_FULL:
            return 0.25
    return 0.0


# -- the bender's working, and the trial-long record -----------------------------------------------------------------------


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        """The ONLY place the record is cleared."""
        self.records: list[list[float]] = []  # [u at the 0 line, set angle (rad), sign in the tube's frame]
        self.scrap = ""
        self.clamped = False
        self.lever_locked = False
        self.targets = {}
        self.recent: list[np.ndarray] = []
        self.settled_s = 0.0
        self.full, self.score = False, 0.0
        self.steps = 0

    def _clamp(self, env, on: bool) -> None:
        if on == self.clamped:
            return
        self.clamped = on
        for link in ("tslide", "troll"):
            if on:
                m.lock(env, "bender", link, stiffness=20000.0, effort=500.0)
            else:
                m.unlock(env, "bender", link)

    def _lever(self, env, target: float | None) -> None:
        """Hold the lever's bending stop at `target` (radians of bend), or let it go free."""
        if target is None:
            if self.lever_locked:
                m.unlock(env, "bender", "lever")
                self.lever_locked = False
            return
        if not self.lever_locked:
            m.lock(env, "bender", "lever", stiffness=K_LEVER, effort=M_P)
            self.lever_locked = True
        m.set_target(env, "bender", "lever", target - math.radians(LEVER0))

    def step(self, env) -> None:
        self.steps += 1
        st = m.state(env, "bender")
        bend = math.radians(st.deg("lever") + LEVER0)  # the carriage's 0 line from the form's 0 line
        on = bend > math.radians(-1.0) if not self.clamped else bend > math.radians(-3.0)
        self._clamp(env, on)
        current = None
        if on:
            u0 = -(Y_E2 + st.joint("tslide"))  # the tube's point at the form's 0 line
            for rec in self.records:
                if abs(rec[0] - u0) < 0.002:
                    current = rec
            if current is None and bend > math.radians(0.5):
                roll = st.joint("troll")
                if abs(math.sin(roll)) > math.sin(math.radians(SKEW_DEG)) and not self.scrap:
                    self.scrap = f"bent with the tube turned {math.degrees(roll):.0f} degrees out of its plane"
                current = [u0, 0.0, 1.0 if math.cos(roll) > 0 else -1.0]
                self.records.append(current)
            if current is not None:
                e = math.radians(SPRINGBACK)
                if bend > current[1] + e:
                    current[1] = bend - e
                if bend >= current[1]:
                    self._lever(env, current[1])
                else:
                    self._lever(env, None)
            else:
                self._lever(env, None)
        else:
            self._lever(env, None)
        # the tube follows: each record wraps its arc onto the form from its 0 line
        want = np.zeros(len(HINGES))
        for rec in self.records:
            shown = rec[1]
            if rec is current and bend > rec[1]:
                shown = min(bend, rec[1] + math.radians(SPRINGBACK))
            a0, a1 = rec[0], rec[0] + R * shown
            if math.degrees(rec[1]) > SCRAP_DEG and not self.scrap:
                self.scrap = f"a bend set past {SCRAP_DEG:.0f} degrees"
            if (a0 < HINGES[0] - HINGE_STEP / 2 or a1 > HINGES[-1] + HINGE_STEP / 2) and rec[1] > math.radians(1.0) and not self.scrap:
                self.scrap = "a bend more than 4 cm from the marks"
            for k, u in enumerate(HINGES):
                lo, hi = max(a0, u - HINGE_STEP / 2), min(a1, u + HINGE_STEP / 2)
                if hi > lo:
                    want[k] += rec[2] * (hi - lo) / R
        for k, v in enumerate(want):
            if abs(self.targets.get(k, 0.0) - v) > 1e-5:
                m.set_target(env, "bender", f"t{k:02d}", float(v))
                self.targets[k] = float(v)
        hinge_deg = np.array([st.deg(f"t{k:02d}") for k in range(len(HINGES))])
        tips = m.fingertips(env)
        hands = min(gap(tips, tube_world(st, hinge_deg)), *(gap(tips, ln) for ln in handle_lines(st)))
        joints = np.array([*hinge_deg, st.deg("troll"), st.deg("lever"), 1000.0 * st.joint("tslide")])
        self.recent = (self.recent + [joints])[-6:]
        still = len(self.recent) == 6 and float(np.max(np.ptp(np.array(self.recent), axis=0))) < 0.3
        self.update(hinge_deg, hands, still, env.unwrapped.step_dt)
        if self.steps % 20 == 0:
            print(f"[offset] t={self.steps * env.unwrapped.step_dt:.1f} lever={math.degrees(bend):.1f} slide={1000 * st.joint('tslide'):.1f}mm roll={st.deg('troll'):.1f} "
                  f"bends={[(round(a, 1), round(1000 * u, 1)) for a, u in bends_from(hinge_deg)]} hands={hands:.3f} still={still} score={self.score} scrap={self.scrap!r} "
                  f"frame=({100 * st.pos[0]:.1f},{100 * st.pos[1]:.1f}) yaw={math.degrees(2 * math.atan2(st.quat_xyzw[2], st.quat_xyzw[3])):.1f}", flush=True)

    def update(self, hinge_deg: np.ndarray, hands: float, still: bool, dt: float) -> None:
        """The scorer, from the tube's hinge angles, the grippers' least distance to the tube and handles, and stillness. check_scorer.py calls it directly."""
        settled = still and hands >= CLEAR
        self.settled_s = self.settled_s + dt if settled else 0.0
        if self.scrap or self.settled_s < SUCCESS_HOLD_S - 1e-6:
            self.score = 0.0
        else:
            self.score = grade(bends_from(hinge_deg), offset_of(hinge_deg))
        self.full = bool(self.score == 1.0)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """The full-score shape, with the tube and lever still and both grippers 3 cm clear of the tube and both handles, for 1 s."""
    return TRIAL.full


def score(objects) -> float:
    """See task.json's scoring: 1.0, 0.75, 0.5, 0.25 or 0, read once everything has been still with the grippers clear for 1 s."""
    return float(TRIAL.score)


# -- the scripted demo: open loop, from the published starting layout ------------------------------------------------------

TUBE_X = PIVOT[0] + R
TUBE_Z = TOP + FRAME_Z + ZT
LEFT_PARK = (0.22, 0.32, TOP + 0.14)
RIGHT_PARK = (0.22, -0.32, TOP + 0.14)


def lever_grip(bend_deg: float) -> tuple[float, float, float]:
    a = math.radians(bend_deg)
    x = HANDLE_X * math.cos(a) - GRIP_AT * math.sin(a)
    y = HANDLE_X * math.sin(a) + GRIP_AT * math.cos(a)
    return (PIVOT[0] + x, PIVOT[1] + y, TOP + FRAME_Z + LEVER_GRIP_Z)


def lever_yaw(bend_deg: float) -> float:
    return math.radians(bend_deg - 90.0)


LAG = 4.0  # degrees the arm falls short while pulling against the tube (measured in a run)
OVERBEND = SPRINGBACK + 0.2


def _pull(name: str, top: float) -> list[Waypoint]:
    """Left arm: take the lever's grip, swing it from the start to `top` degrees of bend, back to the start, let go."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    g0 = lever_grip(LEVER0)
    wps = [Waypoint(1.5, (g0[0], g0[1], g0[2] + 0.04), o, f"{name}: over the lever", lever_yaw(LEVER0)),
           Waypoint(1.0, g0, o, f"{name}: on the lever", lever_yaw(LEVER0)),
           Waypoint(0.6, g0, c, f"{name}: hold the lever", lever_yaw(LEVER0))]
    for b in np.arange(LEVER0 + 5.0, top + 0.01, 5.0).tolist() + [top]:
        wps.append(Waypoint(0.45, lever_grip(b), c, f"{name}: pull to {b:.0f}", lever_yaw(b)))
    wps.append(Waypoint(1.0, lever_grip(top), c, f"{name}: hold at {top:.0f}", lever_yaw(top)))
    for b in np.arange(top - 5.0, LEVER0 - 0.01, -5.0).tolist() + [LEVER0]:
        wps.append(Waypoint(0.25, lever_grip(b), c, f"{name}: back to {b:.0f}", lever_yaw(b)))
    wps += [Waypoint(0.6, g0, o, f"{name}: let go", lever_yaw(LEVER0)),
            Waypoint(0.8, (g0[0], g0[1], g0[2] + 0.045), o, f"{name}: up", lever_yaw(LEVER0))]
    return wps


def _slide(name: str, at_y: float, dy: float) -> list[Waypoint]:
    """Right arm: pinch the tube on the form handle's side and slide it along the groove by dy."""
    o, c, yaw = FINGER_OPEN, FINGER_CLOSED, math.radians(90.0)
    return [Waypoint(1.5, (TUBE_X, at_y, TUBE_Z + 0.04), o, f"{name}: over the tube", yaw),
            Waypoint(1.0, (TUBE_X, at_y, TUBE_Z + 0.002), o, f"{name}: down", yaw),
            Waypoint(0.6, (TUBE_X, at_y, TUBE_Z + 0.002), c, f"{name}: pinch", yaw),
            Waypoint(2.0, (TUBE_X, at_y + dy, TUBE_Z + 0.002), c, f"{name}: slide", yaw),
            Waypoint(0.5, (TUBE_X, at_y + dy, TUBE_Z + 0.002), o, f"{name}: let go", yaw),
            Waypoint(0.8, (TUBE_X, at_y + dy, TUBE_Z + 0.04), o, f"{name}: up", yaw)]


SLIDE_LAG = (1.0, 3.9)  # mm the tube falls short of the hand in each slide (measured in a run of this fixed start)
ROLL_REACH = 0.05  # along the bent leg from the end of bend A, where the left hand takes it
ROLL_EXTRA = -7.5  # degrees past 180 the hand goes: the tube overran by 7.7 degrees in a run


def _roll() -> list[Waypoint]:
    """Left arm: take the bent leg past mark A and carry it over the top, half a turn about the tube's straight part."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    y_start = Y_E2 + (START_SHORT + 0.0707) + MARK_A - ARC45 / 2  # after both slides, where bend A's curve begins
    lx = R * math.cos(math.radians(45.0)) - ROLL_REACH * math.sqrt(0.5)
    ly = y_start + R * math.sin(math.radians(45.0)) + ROLL_REACH * math.sqrt(0.5)
    dx = lx - R  # from the turning axis, across
    wps = []
    for k, phi in enumerate(np.linspace(0.0, 180.0 + ROLL_EXTRA, 13)):
        p = math.radians(phi)
        pos = (PIVOT[0] + R + dx * math.cos(p), PIVOT[1] + ly, TOP + FRAME_Z + ZT - dx * math.sin(p))
        yaw = math.atan2(-math.sqrt(0.5), math.sqrt(0.5) * math.cos(p))
        if k == 0:
            wps += [Waypoint(1.5, (pos[0], pos[1], pos[2] + 0.04), o, "roll: over the bent leg", yaw),
                    Waypoint(1.0, pos, o, "roll: down", yaw), Waypoint(0.6, pos, c, "roll: pinch the leg", yaw)]
        else:
            wps.append(Waypoint(0.6, pos, c, f"roll: {phi:.0f} degrees", yaw))
    last = wps[-1]
    wps += [Waypoint(0.6, last.pos, o, "roll: let go", last.yaw), Waypoint(0.8, (last.pos[0], last.pos[1], last.pos[2] + 0.035), o, "roll: up", last.yaw)]
    return wps


def _dur(wps: list[Waypoint]) -> float:
    return sum(w.duration for w in wps)


def _wait(at, s: float, label: str) -> Waypoint:
    return Waypoint(s, at, FINGER_OPEN, label, 0.0)


FORM_GRIP = (PIVOT[0] - HANDLE_X, PIVOT[1] - GRIP_AT, TOP + FRAME_Z + 0.006)


def demo():
    """Open loop, from the fixed starting layout: the right hand slides mark A to the carriage's 45 line; the left pulls bend A and
    swings the lever back; the right slides the tube on to mark B; the left turns the tube half a turn by its bent leg and pulls
    bend B; both let go."""
    top = TARGET + OVERBEND + LAG
    s1 = _slide("slide to mark A", -0.10, START_SHORT + SLIDE_LAG[0] / 1000)
    b1 = _pull("bend A", top)
    s2 = _slide("slide to mark B", -0.135, 0.0707 + SLIDE_LAG[1] / 1000)
    rl = _roll()
    b2 = _pull("bend B", top)
    left = [Waypoint(1.0, LEFT_PARK, FINGER_OPEN, "park", 0.0)]
    right = [Waypoint(1.0, RIGHT_PARK, FINGER_OPEN, "park", 0.0), *s1, Waypoint(1.0, RIGHT_PARK, FINGER_OPEN, "park", 0.0)]
    left.append(_wait(LEFT_PARK, _dur(right) - 1.0 - _dur(left), "wait for the slide"))
    left += b1 + [Waypoint(1.0, LEFT_PARK, FINGER_OPEN, "park", 0.0)]
    right.append(_wait(RIGHT_PARK, _dur(left) - 1.0 - _dur(right), "wait for bend A"))
    right += s2 + [Waypoint(1.0, RIGHT_PARK, FINGER_OPEN, "park", 0.0)]
    left.append(_wait(LEFT_PARK, _dur(right) - 1.0 - _dur(left), "wait for the slide"))
    left += rl + b2 + [Waypoint(1.2, LEFT_PARK, FINGER_OPEN, "clear", 0.0)]
    end = max(_dur(left), _dur(right)) + 3.0
    left.append(_wait(LEFT_PARK, end - _dur(left), "done"))
    right.append(_wait(RIGHT_PARK, end - _dur(right), "done"))
    return {"left": left, "right": right}


def demo():
    """Red team, the careless way: slide to mark B but never turn the tube, so bend B goes the same way as bend A (a U)."""
    top = TARGET + OVERBEND + LAG
    s1 = _slide("slide to mark A", -0.10, START_SHORT + SLIDE_LAG[0] / 1000)
    b1 = _pull("bend A", top)
    s2 = _slide("slide to mark B", -0.135, 0.0707 + SLIDE_LAG[1] / 1000)
    b2 = _pull("bend B", top)
    left = [Waypoint(1.0, LEFT_PARK, FINGER_OPEN, "park", 0.0)]
    right = [Waypoint(1.0, RIGHT_PARK, FINGER_OPEN, "park", 0.0), *s1, Waypoint(1.0, RIGHT_PARK, FINGER_OPEN, "park", 0.0)]
    left.append(_wait(LEFT_PARK, _dur(right) - 1.0 - _dur(left), "wait"))
    left += b1 + [Waypoint(1.0, LEFT_PARK, FINGER_OPEN, "park", 0.0)]
    right.append(_wait(RIGHT_PARK, _dur(left) - 1.0 - _dur(right), "wait"))
    right += s2 + [Waypoint(1.0, RIGHT_PARK, FINGER_OPEN, "park", 0.0)]
    left.append(_wait(LEFT_PARK, _dur(right) - 1.0 - _dur(left), "wait"))
    left += b2 + [Waypoint(1.2, LEFT_PARK, FINGER_OPEN, "clear", 0.0)]
    end = max(_dur(left), _dur(right)) + 3.0
    left.append(_wait(LEFT_PARK, end - _dur(left), "done"))
    right.append(_wait(RIGHT_PARK, end - _dur(right), "done"))
    return {"left": left, "right": right}
