"""Close the roller clamp on a primed gravity IV line that has been left running free.

An IV training forearm lies on the right of an overbed table, hand toward the arms, with a 20 G cannula taped on its back. A primed gravity
administration set runs from a 1 L bag of saline on an IV pole beside the table's left edge: its line hangs from the drip chamber down to the
table, runs across the table to a roller clamp lying in front of the arms, and goes on from the clamp to the cannula. The clamp lies wheel up,
wide open (its blue wheel at the open, upstream end of its track), so the line runs free. Closing it means rolling the wheel 22 mm along its
track to the narrow, patient end, where it squeezes the tubing shut against the clamp's ramped floor. The clamp is a 54 mm, 6 g plastic
channel on soft tubing: pushed by its wheel alone, it slides along the table with the line instead of closing.

"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cordkit  # noqa: E402
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
FLOOR = -0.0125  # the station's floor (station.FLOOR_Z)
SUCCESS_HOLD_S = 0.0  # Trial tracks every full condition for three seconds.
STILL_STEPS = 20  # 1 s at 20 Hz

LOOKS = {
    "pp": m.Look(color=m.rgb(0.94, 0.94, 0.92), roughness=0.45),  # the clamp's white polypropylene body
    "slot": m.Look(color=m.rgb(0.35, 0.36, 0.38), roughness=0.6),
    "wheel": m.Look(color=m.rgb(0.12, 0.42, 0.85), roughness=0.4),
    "tube": m.Look(color=m.rgb(0.84, 0.89, 0.92), roughness=0.25),
    "skin": m.Look(color=m.rgb(0.87, 0.71, 0.60), roughness=0.7),
    "nail": m.Look(color=m.rgb(0.93, 0.80, 0.74), roughness=0.4),
    "pink": m.Look(color=m.rgb(0.93, 0.45, 0.62), roughness=0.4),
    "film": m.Look(color=m.rgb(0.95, 0.96, 0.97), roughness=0.2, opacity=0.35),
    "tape": m.Look(color=m.rgb(0.96, 0.96, 0.94), roughness=0.8),
    "bag": m.Look(color=m.rgb(0.90, 0.93, 0.95), roughness=0.2, opacity=0.8),
    "label": m.Look(color=m.rgb(0.97, 0.97, 0.97), roughness=0.6),
    "print": m.Look(color=m.rgb(0.15, 0.35, 0.70), roughness=0.6),
    "white": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.5),
    "chamber": m.Look(color=m.rgb(0.90, 0.94, 0.96), roughness=0.1, opacity=0.5),
    "fluid": m.Look(color=m.rgb(0.80, 0.90, 0.97), roughness=0.1, opacity=0.6),
}

# -- the roller clamp (clamp frame: x across, y along it from the narrow tail (-y, toward the patient) to the open head (+y, upstream), z up from its underside)
CL, CW, WALL_T = 0.054, 0.018, 0.003  # length, width, wall thickness
FLOOR_T, WALL_H = 0.002, 0.010  # its floor plate, and the walls' top over its underside
TUBE_R = 0.00205  # PVC tubing, 4.1 mm outside
TUBE_Z = FLOOR_T + TUBE_R  # the tubing's axis inside the clamp, at the head
WHEEL_R, WHEEL_W = 0.006, 0.009  # the wheel: 12 mm across, 9 mm wide, ribbed
WHEEL_Y0, WHEEL_Z = 0.014, 0.0125  # its axle at the open end of its slots: the wheel's rim 0.4 mm clear of the tubing
TRAVEL = 0.022  # the wheel's travel along its slots, open to closed, toward the tail
RISE = 0.0034  # the channel's floor ramps up 3.4 mm under the wheel's travel: closed, the 4.1 mm tubing is squeezed to 1.1 mm (its two walls)
CLOSED_AT = TRAVEL - 0.0005  # at the narrow stop, with 0.5 mm numerical/measurement tolerance
PARTIAL_AT = 0.6 * TRAVEL
WHEEL_HOLD = 3.0  # N to move the wheel along its slots (approximated: to be measured on a real clamp)
BODY_MASS, WHEEL_MASS = 0.0048, 0.0012  # 6 g total, estimated from the modeled polypropylene volume
CARRY = 0.050  # the model's range for the clamp sliding along its line, each way (a joint limit: see SLID_AT)
SLID_AT = 0.040  # slid this far along the line voids the trial: a real clamp would keep sliding, down toward the cannula
CARRIER_HOLD = 1.5  # N to slide the open clamp along its tubing (estimated; below WHEEL_HOLD, since an open clamp slides on its line)
CARRIER_MASS = 0.0008  # the 54 mm of saline-filled tubing inside the clamp
CX, CY = 0.300, 0.000  # the clamp's centre on the table
CLAMP_POS = (CX, CY, TOP + 0.0003)
SETTLED = (-0.0039, 0.0009)  # the clamp's centre after the reset's settle, from CLAMP_POS (measured)
HEAD = np.array([CX, CY + CL / 2, TOP + 0.0003 + TUBE_Z])  # where the upstream line enters
TAIL = np.array([CX, CY - CL / 2, TOP + 0.0003 + TUBE_Z + RISE])  # where the line to the patient leaves, off the ramp


def _wheel_mesh() -> m.Mesh:
    rim = m.cylinder(WHEEL_R - 0.0004, WHEEL_W, (0.0, 0.0, 0.0), axis="x", segments=32)
    ribs = []
    for k in range(20):  # the knurled rim a thumb rolls
        a = 2 * math.pi * k / 20
        rib = m.box((WHEEL_W * 0.9, 0.0011, 0.0009), (0.0, 0.0, WHEEL_R - 0.00045)).turned(math.degrees(a), "x")
        ribs.append(rib)
    return m.join(rim, *ribs)


def _clamp(name: str = "rclamp", pos=None, free: bool = True) -> m.Part:
    parts, cols = [], []
    parts.append(("floor", m.box((CW, CL, FLOOR_T), (0.0, 0.0, FLOOR_T / 2)), "pp"))
    cols.append(m.col_box("floor_c", (CW, CL, FLOOR_T), (0.0, 0.0, FLOOR_T / 2)))
    inner = CW - 2 * WALL_T
    y0, y1 = WHEEL_Y0, WHEEL_Y0 - TRAVEL  # the ramp runs under the wheel's travel, rising toward the tail
    ramp = m.prism([(y0, FLOOR_T), (y1, FLOOR_T + RISE), (y1, FLOOR_T)], -inner / 2, inner / 2)  # in (y, z), across x
    ramp = m.Mesh(ramp.v[:, [2, 0, 1]], ramp.f)
    flat = ((inner, y1 + CL / 2, RISE), (0.0, (y1 - CL / 2) / 2, FLOOR_T + RISE / 2))
    parts += [("ramp", ramp, "pp"), ("ramp_top", m.box(*flat), "pp")]
    cols += [m.col_hull("ramp_c", ramp), m.col_box("ramp_top_c", *flat)]
    for s, n in ((1.0, "l"), (-1.0, "r")):
        xc = s * (CW / 2 - WALL_T / 2)
        size = (WALL_T, CL, WALL_H - FLOOR_T)
        centre = (xc, 0.0, FLOOR_T + (WALL_H - FLOOR_T) / 2)
        parts.append((f"wall_{n}", m.box(size, centre), "pp"))
        cols.append(m.col_box(f"wall_{n}_c", size, centre))
        # the axle's slot, seen on the wall's outside, and grip ridges at the tail
        parts.append((f"slot_{n}", m.box((0.0006, TRAVEL + 0.0036, 0.0034), (s * (CW / 2 + 0.0002), WHEEL_Y0 - TRAVEL / 2, WALL_H - 0.0022)), "slot"))
        for k in range(5):
            parts.append((f"ridge_{n}{k}", m.box((0.0008, 0.0012, WALL_H - 0.005), (s * (CW / 2 + 0.0003), -0.013 - 0.0026 * k, WALL_H / 2 - 0.001)), "pp"))
    # The tubing through the clamp is the `carrier` link: the piece of line in the channel, from where the upstream section ends (the head)
    # to where the patient section starts (the tail). Both sections are fixed to it, and it slides along the clamp on a joint held only by
    # friction (CARRIER_HOLD), standing in for the tubing's grip in the open channel: the clamp slides along its line, as a real one does.
    tube = m.tube([np.array([0.0, -CL / 2 - 0.001, TUBE_Z + RISE]), np.array([0.0, CL / 2 + 0.001, TUBE_Z])], TUBE_R, sides=16)
    carrier = m.slider("carrier", "Y", -CARRY, CARRY, [("tube_in", tube, "tube")], [], at=(0.0, 0.0, TUBE_Z + RISE / 2), mass=CARRIER_MASS,
                       damping=0.01, friction=CARRIER_HOLD, effort=50.0, inertia=(2.0e-7, 2.0e-9, 2.0e-7))
    wheel_at = (0.0, WHEEL_Y0, WHEEL_Z)
    wheel = _wheel_mesh().moved(wheel_at)
    axle = m.cylinder(0.0016, CW + 0.0006, wheel_at, axis="x", segments=12)  # its ends show in the walls' slots
    link = m.slider("wheel", "Y", -TRAVEL, 0.0, [("wheel", wheel, "wheel"), ("axle", axle, "slot")],
                    [m.col_hull("wheel_c", m.cylinder(WHEEL_R, WHEEL_W, wheel_at, axis="x", segments=20))],
                    at=wheel_at, mass=WHEEL_MASS, damping=0.05, friction=WHEEL_HOLD, effort=50.0)
    inertia = (BODY_MASS / 12 * (CL ** 2 + WALL_H ** 2), BODY_MASS / 12 * (CW ** 2 + WALL_H ** 2), BODY_MASS / 12 * (CL ** 2 + CW ** 2))
    return m.mechanism(name, CLAMP_POS if pos is None else pos, [link, carrier], LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.4, 0.3),
                       solver_iters=64, free_mass=BODY_MASS if free else None, frame_inertia=inertia)


def wheel_travel(c) -> float:
    """How far (m) the wheel has rolled from its open end toward the narrow stop."""
    return -float(c.joint("wheel"))


# -- the training forearm, its cannula and dressing (fixed), along x at y ARM_Y; hand toward the arms
ARM_Y = -0.245
WRIST_X, ELBOW_X, END_X = 0.370, 0.630, 0.690


def _section(x: float) -> tuple[float, float]:
    """Half-width and half-height of the forearm's elliptical section at x (resting on the table)."""
    pts = [(WRIST_X, 0.031, 0.021), (0.50, 0.042, 0.031), (ELBOW_X, 0.045, 0.038), (END_X, 0.047, 0.041)]
    xs = [p[0] for p in pts]
    return float(np.interp(x, xs, [p[1] for p in pts])), float(np.interp(x, xs, [p[2] for p in pts]))


def _loft(rings: list[np.ndarray]) -> m.Mesh:
    """A closed solid through rings of equal point count (each counter-clockwise seen from its +x side)."""
    n = len(rings[0])
    v = np.vstack(rings + [rings[0].mean(0, keepdims=True), rings[-1].mean(0, keepdims=True)])
    f = []
    for r in range(len(rings) - 1):
        a, b = r * n, (r + 1) * n
        for i in range(n):
            j = (i + 1) % n
            f += [(a + i, b + j, b + i), (a + i, a + j, b + j)]
    c0, c1 = len(rings) * n, len(rings) * n + 1
    last = (len(rings) - 1) * n
    for i in range(n):
        j = (i + 1) % n
        f += [(c0, j, i), (c1, last + i, last + j)]
    return m.Mesh(v, np.array(f))


def _ring(x: float, a: float, b: float, y0: float, n: int = 20, z0: float = 0.0) -> np.ndarray:
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[np.full(n, x), y0 + a * np.cos(t), z0 + b + b * np.sin(t)]


def skin_z(x: float, y: float) -> float:
    """The forearm's top surface over (x, y), above the table (0 off the arm)."""
    a, b = _section(x)
    u = (y - ARM_Y) / a
    return b + b * math.sqrt(max(0.0, 1 - u * u)) if abs(u) < 1 else 0.0


HUB_X = 0.440  # the cannula hub's luer end, facing the hand (-x)
HUB = np.array([HUB_X, ARM_Y, TOP + skin_z(HUB_X + 0.007, ARM_Y) + 0.004])


def _forearm() -> m.Part:
    parts, cols = [], []
    xs = np.linspace(WRIST_X, END_X, 9)
    rings = [_ring(x, *_section(x), ARM_Y) for x in xs]
    parts.append(("forearm", _loft(rings), "skin"))
    for k in range(len(xs) - 1):
        cols.append(m.col_hull(f"forearm_c{k}", _loft([rings[k], rings[k + 1]])))
    # the hand, palm down, fingers toward the arms (-x); a left hand, so the thumb is on the +y side
    hx = np.array([WRIST_X + 0.004, 0.345, 0.310, 0.275])
    hand = [_ring(x, a, b, ARM_Y + dy, z0=0.0) for x, a, b, dy in zip(hx, (0.031, 0.040, 0.0425, 0.041), (0.019, 0.016, 0.015, 0.012), (0.0, -0.002, -0.003, -0.003))]
    parts.append(("hand", _loft(hand), "skin"))
    for k in range(len(hx) - 1):
        cols.append(m.col_hull(f"hand_c{k}", _loft([hand[k], hand[k + 1]])))
    for k, (dy, length) in enumerate(((-0.028, 0.075), (-0.009, 0.085), (0.010, 0.082), (0.028, 0.065))):
        y = ARM_Y - 0.003 + dy
        p0, p1 = np.array([0.283, y, 0.011]), np.array([0.283 - length, y, 0.009])
        finger = m.tube([p0, (p0 + p1) / 2, p1], 0.0085, sides=12)
        parts.append((f"finger{k}", finger, "skin"))
        parts.append((f"nail{k}", m.box((0.010, 0.010, 0.001), (p1[0] + 0.007, y, 0.0172)), "nail"))
        cols.append(m.col_hull(f"finger{k}_c", finger))
    thumb = m.tube([np.array([0.352, ARM_Y + 0.030, 0.012]), np.array([0.322, ARM_Y + 0.046, 0.011]), np.array([0.290, ARM_Y + 0.050, 0.010])], 0.0095, sides=12)
    parts.append(("thumb", thumb, "skin"))
    cols.append(m.col_hull("thumb_c", thumb))
    # the cannula: a pink 20 G hub with wings under a clear film dressing, its catheter into the skin toward the elbow, the set's luer in its end
    hz = HUB[2] - TOP
    parts.append(("hub", m.cylinder(0.0035, 0.014, (HUB_X + 0.015, ARM_Y, hz), axis="x", segments=20), "pink"))
    parts.append(("wings", m.box((0.009, 0.024, 0.0012), (HUB_X + 0.016, ARM_Y, hz - 0.0030)), "pink"))
    parts.append(("luer", m.cylinder(0.0032, 0.008, (HUB_X + 0.004, ARM_Y, hz), axis="x", segments=16), "tube"))
    parts.append(("catheter", m.cylinder(0.0006, 0.010, (HUB_X + 0.026, ARM_Y, hz - 0.0015), axis="x", segments=8), "white"))
    sz = skin_z(HUB_X + 0.018, ARM_Y)
    parts.append(("dressing", m.box((0.050, 0.032, 0.0006), (HUB_X + 0.018, ARM_Y, sz + 0.0062)), "film"))
    parts.append(("tape", m.box((0.012, 0.040, 0.0008), (HUB_X - 0.010, ARM_Y, hz + TUBE_R + 0.0008)), "tape"))
    return m.fixed("forearm", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.8, 0.7))


# -- the line: two capsule chains, from the drip chamber to the clamp's head and from the clamp's tail to the cannula
SEG = 0.025
LINE_KG_PER_M = 0.015  # PVC tubing 4.1 x 3.0 mm, full of saline
EI = 1.0e-4  # N m^2: soft PVC tubing (approximated, to be measured)
POLE = (0.450, 0.740)  # the IV pole's mast, beside the table's left edge
HANG = np.array([POLE[0], POLE[1] - 0.184])  # under the pole's inner hook tip, where the line hangs down
OUTLET_Z = 0.588  # the drip chamber's outlet over the table


def _smooth(points, n: int = 400) -> np.ndarray:
    """A smooth curve through the points (Catmull-Rom), densely sampled."""
    p = np.asarray(points, float)
    p = np.vstack([2 * p[0] - p[1], p, 2 * p[-1] - p[-2]])
    out = []
    per = max(4, n // (len(p) - 3))
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1], p[i], p[i + 1], p[i + 2]
        for t in np.linspace(0, 1, per, endpoint=False):
            t2, t3 = t * t, t * t * t
            out.append(0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(p[-2])
    out = np.array(out)
    out[:, 2] = np.maximum(out[:, 2], TOP + R_LIE)  # never into the table
    return out


def _walk(path: np.ndarray, seg: float) -> list:
    """Nodes along a dense path, each `seg` from the last in a straight line, starting at its first point (an incomplete end is dropped)."""
    nodes, start = [path[0]], 1
    while True:
        far = np.nonzero(np.linalg.norm(path[start:] - nodes[-1], axis=1) >= seg)[0]
        if not len(far):
            return nodes
        i = start + int(far[0])
        a, b = path[i - 1], path[i]
        lo, hi = 0.0, 1.0
        for _ in range(50):  # where on this piece the chord is exactly seg
            mid = (lo + hi) / 2
            if np.linalg.norm(a + mid * (b - a) - nodes[-1]) < seg:
                lo = mid
            else:
                hi = mid
        nodes.append(a + hi * (b - a))
        start = i


def _chords(path: np.ndarray, seg: float) -> np.ndarray:
    """Equal chords from the path's start that end exactly at its end: the chord length is tuned near `seg`."""
    grid = np.linspace(0.92 * seg, 1.08 * seg, 161)
    walks = [_walk(path, g) for g in grid]
    best = None
    for k in range(len(grid) - 1):  # a drop in the count, with the last node then at the path's end
        n = len(walks[k]) - 1
        if len(walks[k + 1]) - 1 < n and np.linalg.norm(walks[k][-1] - path[-1]) < 0.15 * seg:
            if best is None or abs(grid[k] - seg) < abs(grid[best[0]] - seg):
                best = (k, n)
    k, n = best
    lo, hi = grid[k], grid[k + 1]
    for _ in range(60):
        mid = (lo + hi) / 2
        if len(_walk(path, mid)) - 1 >= n:
            lo = mid
        else:
            hi = mid
    nodes = _walk(path, lo)[: n + 1]
    assert np.linalg.norm(nodes[-1] - path[-1]) < 1e-7, np.linalg.norm(nodes[-1] - path[-1])
    nodes[-1] = path[-1]
    return np.array(nodes)


R_LIE = TUBE_R + 0.0004  # a link's axis when it lies on the table


def _upstream_nodes() -> np.ndarray:
    """From the clamp's head, out along the table and up to the drip chamber's outlet (reversed below so the line starts at the outlet)."""
    z = TOP + R_LIE
    hx, hy = HANG
    table = [HEAD, (CX, CY + 0.060, z), (CX, CY + 0.120, z), (CX, CY + 0.180, z), (CX, CY + 0.240, z), (0.318, 0.320, z), (0.372, 0.420, z),
             (0.425, 0.500, z), (hx - 0.004, hy - 0.030, z), (hx, hy - 0.004, TOP + 0.012), (hx, hy, TOP + 0.045)]
    path = np.vstack([_smooth(table, 1200), [(hx, hy, TOP + 0.045 + d) for d in np.linspace(0.002, OUTLET_Z - 0.045, 400)]])
    return _chords(path, SEG)


def _patient_nodes() -> np.ndarray:
    """From the clamp's tail, in an S across the table, up the forearm's side and along its back to the cannula's luer."""
    z = TOP + R_LIE
    pts = [TAIL, (CX, CY - 0.060, z), (CX, CY - 0.092, z), (CX - 0.008, CY - 0.118, z), (0.268, -0.142, z), (0.268, -0.163, z), (0.292, -0.176, z),
           (0.340, -0.178, z), (0.375, -0.183, z), (0.392, -0.192, TOP + 0.010), (0.400, -0.204, TOP + 0.034), (0.408, -0.222, TOP + 0.050),
           (0.420, ARM_Y + 0.004, HUB[2] - 0.001), HUB]
    return _chords(_smooth(pts, 1500), SEG)


UP_NODES = _upstream_nodes()[::-1]  # outlet first
PT_NODES = _patient_nodes()
OUTLET = UP_NODES[0]


def _line(name: str, nodes: np.ndarray, **anchors) -> cordkit.Rope:
    seg = float(np.linalg.norm(nodes[1] - nodes[0]))
    length = seg * (len(nodes) - 1)
    ei = EI / seg
    return cordkit.rope(name, nodes=nodes, radius=TUBE_R, mass=LINE_KG_PER_M * length, friction=0.6, bend_stiffness=ei,
                        bend_damping=ei * 0.05, max_bend=1.3, twist_stiffness=2.0e-4 / seg, twist_damping=2.0e-4, linear_damping=0.3,
                        angular_damping=1.0, contact_offset=0.001, solver_position_iterations=48, solver_velocity_iterations=8,
                        color=(0.84, 0.89, 0.92), **anchors)


LINE_UP = _line("line_up", UP_NODES, anchor_start=True, anchor_end_body="rclamp/carrier", anchor_end_at=CLAMP_POS,
                anchor_filter=("rclamp/frame",))
LINE_PT = _line("line_pt", PT_NODES, anchor_body="rclamp/carrier", anchor_at=CLAMP_POS, anchor_end=True,
                anchor_filter=("rclamp/frame",))


# -- the bag, spike and drip chamber, hanging from the pole's hook (fixed; above the top camera's view)
def _bagset() -> m.Part:
    hx, hy = HANG
    z_out = OUTLET[2] - TOP
    parts = []
    z = z_out
    parts.append(("outlet", m.lathe([(0.0, z), (TUBE_R + 0.0003, z), (0.004, z + 0.012), (0.0, z + 0.012)], 16).moved((hx, hy, 0.0)), "chamber"))
    parts.append(("chamber", m.cylinder(0.0105, 0.060, (hx, hy, z + 0.042), segments=24), "chamber"))
    parts.append(("drops", m.cylinder(0.0095, 0.016, (hx, hy, z + 0.020), segments=24), "fluid"))
    parts.append(("spike", m.cylinder(0.0065, 0.030, (hx, hy, z + 0.087), segments=20), "white"))
    parts.append(("vent", m.box((0.010, 0.006, 0.010), (hx + 0.010, hy, z + 0.085)), "white"))
    parts.append(("port", m.cylinder(0.0045, 0.030, (hx, hy, z + 0.117), segments=16), "bag"))
    parts.append(("port2", m.cylinder(0.0045, 0.025, (hx + 0.030, hy, z + 0.120), segments=16), "bag"))
    zb = z + 0.132  # the bag's bottom seam
    outline = np.array([[-0.065, 0.0], [0.065, 0.0], [0.068, 0.02], [0.068, 0.23], [0.06, 0.25], [-0.06, 0.25], [-0.068, 0.23], [-0.068, 0.02]])
    body = m.prism(outline, -0.022, 0.022).turned(90.0, "x").moved((hx, hy, zb))
    parts.append(("bag", body, "bag"))
    parts.append(("label", m.box((0.090, 0.0008, 0.120), (hx, hy - 0.0225, zb + 0.120)), "label"))
    parts.append(("label_print", m.box((0.070, 0.0004, 0.012), (hx, hy - 0.0231, zb + 0.150)), "print"))
    parts.append(("tab", m.box((0.040, 0.002, 0.030), (hx, hy, zb + 0.262)), "bag"))
    return m.fixed("bagset", (0.0, 0.0, TOP), parts, [], LOOKS)


# -- props
IV_POLE = SimReady("iv_pole", pos=(POLE[0], POLE[1], FLOOR), size=(0.722, 0.571, 1.8), package="IV_pole_a01", usd="sm_IV_pole_a01_01.usd")
CUP = SimReady("cup", pos=(0.560, -0.060, TOP), size=(0.064, 0.064, 0.100), mass=0.020, package="dishware_plastic_cup_a01", usd="sm_dishware_plastic_cup_a01_01.usd", convex=True)
BANDAGE = SimReady("bandage", pos=(0.170, 0.120, TOP), size=(0.049, 0.052, 0.042), mass=0.016, package="bandage_medical_a01", usd="sm_bandage_medical_a01_01.usd", convex=True)
GLOVES = SimReady("gloves", pos=(0.640, -0.420, TOP), size=(0.178, 0.117, 0.056), mass=0.45, package="box_glove_a01", usd="sm_box_glove_a01_01.usd", yaw=-0.3)

OBJECTS = [_forearm(), _bagset(), _clamp(), LINE_UP, LINE_PT, IV_POLE, CUP, BANDAGE, GLOVES]


# -- scoring ------------------------------------------------------------------------------------------

def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Distance from each point p (n, 3) to each segment a-b (k, 3): (n, k)."""
    ab = b - a
    t = np.clip(np.einsum("nkd,kd->nk", p[:, None, :] - a[None], ab) / np.maximum(np.einsum("kd,kd->k", ab, ab), 1e-12), 0.0, 1.0)
    close = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(p[:, None, :] - close, axis=2)


def _box_dist(local: np.ndarray, lo, hi) -> np.ndarray:
    d = np.maximum(np.maximum(np.asarray(lo) - local, local - np.asarray(hi)), 0.0)
    return np.linalg.norm(d, axis=1)


CLAMP_LO = (-CW / 2, -CL / 2, 0.0)
CLAMP_HI = (CW / 2, CL / 2, WHEEL_Z + WHEEL_R)


class Trial:
    """Whole-run facts, cleared only by a reset of the scene (mechkit's on_reset), never by anything that happens in a run."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.taut = ""  # which section of the line was ever pulled straight
        self.slid = False  # the clamp was ever slid SLID_AT along its line
        self.still = 0
        self.full_steps = 0
        self.prev = None
        self.score = 0.0
        self.ok = False

    def step(self, env) -> None:
        self.steps += 1
        c = m.state(env, "rclamp")
        s = wheel_travel(c)
        slide = float(c.joint("carrier"))  # the clamp's slide along its line (m), + toward the patient
        if abs(slide) >= SLID_AT and not self.slid:
            self.slid = True
            print(f"[clamp] slid {1000 * slide:.1f} mm along its line at step {self.steps}", flush=True)
        up, pt = cordkit.rope_state(env, "line_up"), cordkit.rope_state(env, "line_pt")
        spans = {}
        for name, rope, spec in (("upstream", up, LINE_UP), ("patient", pt, LINE_PT)):
            spans[name] = float(np.linalg.norm(rope.end_tip - rope.start_tip)), float(spec.length)
            if spans[name][0] >= spans[name][1] - 0.015 and not self.taut:
                self.taut = name
                print(f"[clamp] the {name} line pulled straight at step {self.steps}: {spans[name][0]:.3f} of {spans[name][1]:.3f} m", flush=True)
        # hands: every fingertip point at least 2 cm from the clamp's body and wheel and from every link of both sections
        tips = m.fingertips(env)
        d_clamp = float(_box_dist(c.to_local(tips), CLAMP_LO, CLAMP_HI).min())
        segs = np.vstack([r.endpoints for r in (up, pt)])
        d_line = float(_seg_dist(tips, segs[:, 0], segs[:, 1]).min()) - TUBE_R
        hands = min(d_clamp, d_line)
        # Stillness uses visible translation. Axial spin of a round tube is irrelevant to closure/slack.
        cur = np.r_[s, slide]
        moving = (np.linalg.norm(c.lin_vel) > 0.01 or (self.prev is not None and np.abs(cur - self.prev).max() > 0.0002)
                  or np.linalg.norm(up.linear_velocities, axis=1).max() > 0.02
                  or np.linalg.norm(pt.linear_velocities, axis=1).max() > 0.02)
        self.prev = cur
        self.still = 0 if moving else self.still + 1
        if self.taut or self.slid:
            mo = 0.0
        elif s >= CLOSED_AT:
            mo = 1.0
        elif s >= PARTIAL_AT:
            mo = 0.4
        else:
            mo = 0.0
        ends_ok = hands >= 0.02 and self.still >= STILL_STEPS
        full_now = bool(mo == 1.0 and ends_ok)
        self.full_steps = self.full_steps + 1 if full_now else 0
        self.ok = bool(full_now and self.full_steps >= 60)
        self.score = float((1.0 if self.ok else min(mo, 0.4)) if ends_ok else 0.0)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.ok)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the scripted demo --------------------------------------------------------------------------------

SETTLED_YAW = math.radians(-8.1)  # the clamp's yaw after the reset's settle (measured): the lines turn it a little as they settle


def _on_clamp(u: float, v: float) -> tuple[float, float]:
    """World (x, y) of a point (u across, v along) in the settled clamp's frame."""
    c, s = math.cos(SETTLED_YAW), math.sin(SETTLED_YAW)
    return CX + SETTLED[0] + c * u - s * v, CY + SETTLED[1] + s * u + c * v


def demo():
    """One hand, as a thumb does it. The left gripper's closed fingertips come down on the top of the wheel, press it (and through it the
    clamp) down onto the table hard enough that the clamp cannot slide, and roll the wheel along its slots to the narrow stop. The hand then
    lifts straight off and goes away."""
    from yam_arena.demo import FINGER_CLOSED

    press = 0.016  # how far the TCP is sent below the wheel's top: the arm's stiffness turns it into a press
    hi = TOP + 0.10
    tip = 0.013  # the TCP over the fingertips
    top = TOP + 0.0003 + WHEEL_Z + WHEEL_R  # the wheel's top
    along = SETTLED_YAW
    p0 = _on_clamp(0.0, WHEEL_Y0)
    p1 = _on_clamp(0.0, WHEEL_Y0 - TRAVEL - 0.016)
    roll = 4.0
    left = [
        Waypoint(2.0, (p0[0], p0[1], hi), FINGER_CLOSED, "left: over the wheel", along),
        Waypoint(1.5, (p0[0], p0[1], top + tip + 0.003), FINGER_CLOSED, "left: down to the wheel", along),
        Waypoint(1.0, (p0[0], p0[1], top + tip - press), FINGER_CLOSED, "left: press the wheel down", along),
        Waypoint(roll, (p1[0], p1[1], top + tip - press), FINGER_CLOSED, "left: roll it to the narrow stop", along),
        Waypoint(0.8, (p1[0], p1[1], top + tip + 0.004), FINGER_CLOSED, "left: ease off", along),
        Waypoint(1.0, (p1[0], p1[1], hi), FINGER_CLOSED, "left: lift away", along),
        Waypoint(1.5, (0.25, 0.25, TOP + 0.20), FINGER_CLOSED, "left: away", 0.0),
        Waypoint(9.0, (0.25, 0.25, TOP + 0.20), FINGER_CLOSED, "left: wait", 0.0),
    ]
    right = [Waypoint(20.8, (0.25, -0.25, TOP + 0.20), FINGER_CLOSED, "right: rest")]
    return {"left": left, "right": right}
