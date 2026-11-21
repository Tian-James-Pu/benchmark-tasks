"""The scripted demo for task.py: Cartesian TCP waypoints per arm, played open loop by yam_scripted from the published starting layout.

Grips other than straight down are passed as `demokit` grips (a waypoint's yaw that carries a whole rotation); importing demokit installs that.
No simulator state is read: every target comes from the task's published geometry.

The full demo: both hands lift the filter by two of its thumbscrews and lower it onto the dew shield; both turn those two screws in until
they are snug; the right hand caps the finder; the left hand pans the telescope on its handle, bringing the third screw round to the
robot's side, and turns it in; both arms go back to their ready poses.

Modes (TASK_DEMO): full (the 1.0 run); and red-team variants: none (do nothing), slip_on (filter put on, no screw turned),
two_screws (the two screws reachable without panning, then the cap; the third never turned), no_cap (filter secured, finder left open),
hold (everything done, the left hand left on the third screw), one_arm (the left arm alone), poke (the film pressed with a fingertip
after the full job).
"""

from __future__ import annotations

import math

import numpy as np

import demokit as d
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.urdf import axis_angle_matrix

T = None  # the task's module namespace, given by task.demo()
OPEN, SHUT = FINGER_OPEN, FINGER_CLOSED
R_TCP = 0.05255 + 0.0025 + 0.013  # TCP out from the cell's axis when the pad tips stand 2.5 mm off the skirt
BACK = 0.06
HALF = -0.018
PART = -0.014  # fingers 28 mm apart round a 10 mm screw head: the lower one stays clear of the case top  # fingers 36 mm apart: enough round the handle's 20 mm grip
PAN_DEG = 115.0
CYCLES = 6  # regrips per screw (110 degrees each): the screw locks once snug, after which the pads slip on its head


def _e(a: float) -> np.ndarray:
    return np.array([math.cos(a), math.sin(a), 0.0])


def level(bearing_out: float, roll_deg: float = 0.0) -> d.Grip:
    """Fingers level, pointing in along -bearing_out (at a part whose outward direction is bearing_out), turned roll_deg about the pointing direction."""
    g = d.level(math.degrees(bearing_out) + 180.0)
    a = g.rot[:, 2]
    return d._as_grip(axis_angle_matrix(a, math.radians(roll_deg)) @ g.rot)


def park(arm: str):
    """The ready pose: well clear of everything."""
    from yam_arena import ik, station
    from yam_arena.constants import READY
    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), OPEN, d._as_grip(rot)


class Plan:
    def __init__(self):
        self.w: list[Waypoint] = []
        self.t = 0.0

    def go(self, dur, pos, finger, grip, label=""):
        self.w.append(Waypoint(float(dur), tuple(float(v) for v in pos), float(finger), label, grip))
        self.t += float(dur)

    def wait_until(self, t, label="wait"):
        if t > self.t + 1e-9:
            last = self.w[-1]
            self.go(t - self.t, last.pos, last.finger, last.yaw, label)


ZC = 0.0


def screw_tcp(center_xy, z_screw, bearing, r=R_TCP):
    return np.array([center_xy[0], center_xy[1], 0.0]) + r * _e(bearing) + np.array([0.0, 0.0, z_screw + ZC])


def tighten(p: Plan, center_xy, z, bearing, cycles: int, swing: float = 110.0, label="tighten"):
    """Turn a screw clockwise (seen from its head) by `cycles` x `swing` degrees: close, turn, let go, roll back open."""
    at = screw_tcp(center_xy, z, bearing)
    lo = -swing / 2
    p.go(0.5, at, OPEN, level(bearing, lo), f"{label}: roll back")
    for k in range(cycles):
        p.go(0.6, at, SHUT, level(bearing, lo), f"{label}: close")
        p.go(swing / 100.0, at, SHUT, level(bearing, lo + swing), f"{label}: turn")
        p.go(0.4, at, OPEN, level(bearing, lo + swing), f"{label}: let go")
        if k < cycles - 1:
            p.go(swing / 110.0, at, OPEN, level(bearing, lo), f"{label}: roll back")
    p.go(0.7, at + BACK * _e(bearing), OPEN, level(bearing, 0.0), f"{label}: back off")


def sync(*plans):
    t = max(p.t for p in plans)
    for p in plans:
        p.wait_until(t)


def plan(mode: str, task_ns) -> dict:
    global T
    T = task_ns
    TOP = T.TOP
    if mode == "none":
        return {"left": [Waypoint(110.0, park("left")[0], OPEN, "idle", park("left")[2])],
                "right": [Waypoint(110.0, park("right")[0], OPEN, "idle", park("right")[2])]}
    L, R = Plan(), Plan()
    if mode == "one_arm":
        R.go(1.0, *park("right"), "stay parked")
    b1, b2, b3 = T.SCREW_BEARINGS  # s1 (left hand), s2 (right hand), s3 (away from the robot until the telescope is panned)
    c0 = T.CELL0
    z_case = TOP + T.CASE_TOP + T.BOSS_H + T.SCREW_Z
    t0 = T.tube_axis(0.0)
    z_seat = T.RIM_Z + T.SCREW_Z
    z_hover = z_seat + 0.030
    one_arm = mode == "one_arm"

    # 1. both hands pinch a thumbscrew head each, lift the filter, carry it over the dew shield and lower it on
    for p, b, arm in ((L, b1, "left"), (R, b2, "right")):
        PR = 90.0 if arm == "left" else 0.0  # the left pads close above and below the head, centring it in height (its long reach sags)
        if one_arm and arm == "right":
            continue
        if arm == "left":  # in front of the tube and under the finder, not past them
            p.go(1.5, (0.16, 0.12, z_case + 0.06), OPEN, level(b, PR), "out in front")
            p.go(1.2, (0.16, -0.04, z_case + 0.06), OPEN, level(b, PR), "across")
            p.go(1.2, screw_tcp(c0, z_case + 0.04, b, R_TCP + 0.035), PART, level(b, PR), "out from the screw")
            p.go(0.8, screw_tcp(c0, z_case, b, R_TCP + 0.035), PART, level(b, PR), "level with it")
            p.go(0.8, screw_tcp(c0, z_case, b), PART, level(b, PR), "in")
        else:  # fingers closing across the head: straight down onto it
            p.go(3.0, screw_tcp(c0, z_case + 0.06, b), OPEN, level(b, PR), "over the screw")
            p.go(1.0, screw_tcp(c0, z_case, b), OPEN, level(b, PR), "down")
            p.go(1.5, screw_tcp(c0, z_case, b), OPEN, level(b, PR), "wait for the left hand")
        p.go(0.8, screw_tcp(c0, z_case, b), SHUT, level(b, PR), "pinch")
        p.go(0.4, screw_tcp(c0, z_case, b), SHUT, level(b, PR), "hold")
        p.go(1.0, screw_tcp(c0, z_case + 0.05, b), SHUT, level(b, PR), "lift")
        p.go(2.5, screw_tcp(c0, z_hover, b), SHUT, level(b, PR), "raise")
        p.go(2.5, screw_tcp(t0, z_hover, b), SHUT, level(b, PR), "carry")
        p.go(0.8, screw_tcp(t0, z_hover, b), SHUT, level(b, PR), "settle")
        p.go(1.6, screw_tcp(t0, z_seat + 0.010, b), SHUT, level(b, PR), "lower")
        p.go(0.5, screw_tcp(t0, z_seat + 0.010, b), PART, level(b, PR), "let it drop on")
        p.go(0.8, screw_tcp(t0, z_seat, b, R_TCP + 0.004), OPEN, level(b), "follow")
        p.go(0.5, screw_tcp(t0, z_seat, b), OPEN, level(b), "in")
    if mode == "slip_on":
        L.go(0.8, screw_tcp(t0, z_seat, b1, R_TCP + BACK), OPEN, level(b1), "back off")
        R.go(0.8, screw_tcp(t0, z_seat, b2, R_TCP + BACK), OPEN, level(b2), "back off")
    else:
        # 2. turn both screws in until they are snug (both hands at once)
        tighten(L, t0, z_seat, b1, CYCLES, label="s1")
        if not one_arm:
            tighten(R, t0, z_seat, b2, CYCLES, label="s2")
    sync(L, R)

    # the pan handle, for step 4
    pan = PAN_DEG
    hz = TOP + T.HANDLE_Z + 0.013 - 0.004
    hr = T.HANDLE_R1 - 0.025  # the grip's middle
    hb0 = T.PSI0 + T.HANDLE_BEAR
    P3 = np.array([T.P[0], T.P[1], 0.0])

    def at(a, dz=0.0):
        return P3 + hr * _e(a) + np.array([0.0, 0.0, hz + dz])

    def yaw(a):  # fingers closing across the handle (this branch keeps the left wrist inside its range through the whole sweep)
        return math.degrees(a) + 180.0

    # 3. cap the finder (right hand), from its boss in the case, before the telescope is panned; the left hand waits clear, over the handle
    if mode not in ("no_cap", "one_arm", "slip_on"):
        capg = level(math.pi)  # fingers level, pointing +x, closing across y
        grab = -0.006  # TCP this far behind the cap's axis: the pad tips 7 mm past it
        cap_at = np.array([T.CAP0[0] + grab, T.CAP0[1], TOP + T.CASE_TOP + T.CAPBOSS_H - 0.006])
        fa = T.finder_axis(0.0)
        on = np.array([fa[0] + grab, fa[1], T.FCELL_Z - 0.006])
        R.go(1.2, R.w[-1].pos + np.array([-0.06, -0.04, 0.03]), OPEN, R.w[-1].yaw, "clear")
        R.go(2.0, cap_at + np.array([-0.05, 0.0, 0.05]), OPEN, capg, "towards the cap")
        R.go(1.2, cap_at, OPEN, capg, "in")
        R.go(0.8, cap_at, SHUT, capg, "pinch")
        R.go(1.0, cap_at + np.array([0.0, 0.0, 0.05]), SHUT, capg, "lift")
        if not one_arm:
            L.go(2.0, at(hb0, 0.06), OPEN, d.down(yaw(hb0)), "wait over the handle")
        R.go(1.5, np.array([0.17, -0.06, on[2] + 0.03]), SHUT, capg, "round the front of the tube")
        R.go(1.5, on + np.array([-0.08, 0.0, 0.025]), SHUT, capg, "towards the finder")
        R.go(1.2, on + np.array([0.0, 0.0, 0.02]), SHUT, capg, "over the finder")
        R.go(1.5, on, SHUT, capg, "on")
        R.go(0.4, on - np.array([0.0, 0.0, 0.002]), SHUT, capg, "press")
        R.go(0.5, on - np.array([0.0, 0.0, 0.002]), OPEN, capg, "let go")
        R.go(1.0, on + np.array([-0.08, 0.0, 0.0]), OPEN, capg, "out")
        R.go(1.2, np.array([0.17, -0.06, on[2]]), OPEN, capg, "round the front of the tube")
    R.go(2.0, *park("right"), "park")
    if not one_arm:
        L.wait_until(R.t)

    if mode not in ("two_screws", "slip_on"):
        # 4. pan the telescope on its handle (left hand), bringing the third screw round to the robot's side: two strokes
        a0 = hb0
        L.go(2.0, at(a0, 0.06), OPEN, d.down(yaw(a0)), "over the handle")
        L.go(1.0, at(a0), OPEN, d.down(yaw(a0)), "down")
        L.go(0.8, at(a0), SHUT, d.down(yaw(a0)), "grip")
        steps = 12
        for k in range(1, steps + 1):
            a = a0 + math.radians(pan * k / steps)
            L.go(0.5, at(a), SHUT, d.down(yaw(a)), "pan")
        L.go(0.5, at(a), OPEN, d.down(yaw(a)), "let go")
        L.go(0.8, at(a, 0.06), OPEN, d.down(yaw(a)), "up")
        # 5. the third screw (left hand)
        t1 = T.tube_axis(math.radians(pan))
        b3w = b3 + math.radians(pan)
        L.go(2.5, screw_tcp(t1, z_seat + 0.04, b3w, R_TCP + BACK), OPEN, level(b3w), "to the third screw")
        L.go(1.0, screw_tcp(t1, z_seat, b3w, R_TCP + BACK), OPEN, level(b3w), "level with it")
        L.go(1.0, screw_tcp(t1, z_seat, b3w), OPEN, level(b3w), "in")
        tighten(L, t1, z_seat, b3w, CYCLES, label="s3")
        if mode == "hold":
            L.go(0.8, screw_tcp(t1, z_seat, b3w), OPEN, level(b3w), "back in")
            L.go(0.6, screw_tcp(t1, z_seat, b3w), SHUT, level(b3w), "hold the screw")
            L.go(30.0, screw_tcp(t1, z_seat, b3w), SHUT, level(b3w), "hold")
        if mode == "poke":  # fingers level over the front plate, lowered onto the film
            g = level(b3w)
            over = screw_tcp(t1, T.RIM_Z + T.PLATE + 0.03, b3w, 0.03)
            L.go(2.0, over, SHUT, g, "over the film")
            L.go(1.5, over - np.array([0.0, 0.0, 0.024]), SHUT, g, "press the film")
            L.go(1.0, over, SHUT, g, "up")
    if mode != "hold":
        L.go(2.5, *park("left"), "park")
    end = max(L.t, R.t) + 5.0
    L.wait_until(end, "rest")
    R.wait_until(end, "rest")
    return {"left": L.w, "right": R.w}
