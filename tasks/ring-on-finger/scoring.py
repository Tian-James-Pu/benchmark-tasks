"""ring-on-finger scorer: the card's Scoring field, ported from generated/v2/cards/ring-on-finger/scorer.py.

``score(state)`` and ``conditions(state)`` are that file's functions unchanged (thresholds from params.json).
``ScoreTracker`` makes the state dict from the simulator's rigid poses, the way the card says the sim reads it:
- ring_offset_mm, ring_low_u_mm, ring_top_u_mm: the ring's and hand form's rigid poses at read time
  (geometry.ring_readings: highest point of each band edge, u along the ring finger from the web);
- robot_min_dist_mm: least distance from any robot collision hull to the ring and to the hand form (its rigid parts
  and the skin's FEM nodes) at read time;
- ring_move_1s_mm / ring_move_1s_deg: largest centre shift and band-axis direction change over the final 1 s
  (spin about the band's own axis is not read);
- stand_move_max_mm: largest displacement of a plate corner from its start, whole run;
- ring_min_z_ever_mm: lowest band point above the table top, whole run.
Read once at the end (task.SCORE_OVER = 'end'); a missing reading counts as failed. Nothing here reaches a policy.
"""
import math
from collections import deque

import numpy as np

import geometry as G

P = G.P
S, RING, RF = P['scoring'], P['ring'], P['hand']['ring_finger']
KEYS = ('ring_offset_mm', 'ring_low_u_mm', 'ring_top_u_mm', 'robot_min_dist_mm', 'ring_move_1s_mm', 'ring_move_1s_deg',
        'stand_move_max_mm', 'ring_min_z_ever_mm')
TIERS = (1.0, 0.5, 0.25, 0.0)


def conditions(s):
    v = {k: s.get(k) for k in KEYS}
    if any(x is None or (isinstance(x, float) and math.isnan(x)) for x in v.values()):
        return None
    top = v['ring_top_u_mm']
    return {
        'latched': v['stand_move_max_mm'] > S['stand_move_mm'] or v['ring_min_z_ever_mm'] < S['off_table_z_mm'],
        'released': v['robot_min_dist_mm'] >= S['release_mm'],
        'still': v['ring_move_1s_mm'] < S['still_mm'] and v['ring_move_1s_deg'] < S['still_deg'],
        'around': v['ring_offset_mm'] <= S['around_mm'],
        'past_dip': top <= S['past_dip_top_u_mm'],
        'past_pip': top <= S['past_pip_top_u_mm'],
        'seated': v['ring_low_u_mm'] <= S['seated_low_u_mm'],
    }


def score(s):
    c = conditions(s)
    if c is None or c['latched'] or not (c['released'] and c['still'] and c['around']):
        return 0.0
    if c['past_pip'] and c['seated']:
        return 1.0
    if c['past_pip']:
        return 0.5
    if c['past_dip']:
        return 0.25
    return 0.0


def explain(s):
    c = conditions(s)
    if c is None:
        return 'reading unknown: ' + ', '.join(k for k in KEYS if s.get(k) is None)
    if c['latched']:
        return 'latched zero (plate moved or ring off the table)'
    failed = [k for k in ('released', 'still', 'around') if not c[k]]
    if failed:
        return 'not ' + ', '.join(failed)
    if c['past_pip'] and c['seated']:
        return 'seated past the PIP'
    if c['past_pip']:
        return 'past the PIP, not seated'
    if c['past_dip']:
        return 'past the DIP only'
    return 'not past the DIP'


def _angle_deg(a, b):
    c = abs(float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(min(1.0, c)))


def min_distance_to_hulls(points, hulls):
    """Least distance (m) from points to convex hulls (yam_arena.analysis.Hull, world planes). Conservative: a
    point's distance to a hull is its largest plane distance, exact over faces and short near edges and corners."""
    best = np.inf
    if len(points) == 0:
        return best
    lo_p, hi_p = points.min(0), points.max(0)
    for _, h in hulls:
        lo, hi = h.vertices.min(0), h.vertices.max(0)
        gap = np.maximum(lo - hi_p, lo_p - hi).max()
        if gap > best:
            continue
        d = (points @ h.planes[:, :3].T - h.planes[:, 3]).max(1).min()
        best = min(best, float(d))
    return best


class ScoreTracker:
    """Per-step reader of the rigid poses; ``reading()`` gives the scorer's state dict at the current step."""

    def __init__(self, control_hz=20.0):
        self.window = deque(maxlen=int(round(S['hold_s'] * control_hz)) + 1)
        self.plate0 = None
        self.stand_move_max = 0.0
        self.ring_min_z = math.inf
        self.last = None
        self.robot_min_dist_mm = None
        self.steps = 0

    def update(self, ring_pos, ring_rot, hand_pos, hand_rot, robot_hulls=None, skin_nodes=None, read_robot=True):
        r = G.ring_readings(ring_pos, ring_rot, hand_pos, hand_rot)
        corners = G.plate_corners_world(hand_pos, hand_rot)
        if self.plate0 is None:
            self.plate0 = corners
        self.stand_move_max = max(self.stand_move_max, float(np.linalg.norm(corners - self.plate0, axis=1).max() / G.MM))
        self.ring_min_z = min(self.ring_min_z, r['ring_lowest_mm'])
        self.window.append((r['centre'], r['axis']))
        if read_robot:
            self.robot_min_dist_mm = None if robot_hulls is None else self._robot_distance(
                ring_pos, ring_rot, hand_pos, hand_rot, robot_hulls, skin_nodes)
        full = len(self.window) == self.window.maxlen
        c0, a0 = self.window[0]
        move = max(float(np.linalg.norm(c - c0)) for c, _ in self.window) / G.MM if full else None
        turn = max(_angle_deg(a, a0) for _, a in self.window) if full else None
        self.steps += 1
        self.last = dict(ring_offset_mm=r['ring_offset_mm'], ring_low_u_mm=r['ring_low_u_mm'], ring_top_u_mm=r['ring_top_u_mm'],
                         robot_min_dist_mm=self.robot_min_dist_mm, ring_move_1s_mm=move, ring_move_1s_deg=turn,
                         stand_move_max_mm=self.stand_move_max, ring_min_z_ever_mm=self.ring_min_z)
        return self.result()

    @staticmethod
    def _robot_distance(ring_pos, ring_rot, hand_pos, hand_rot, hulls, skin_nodes):
        ring_pts = G.ring_surface_points() @ np.asarray(ring_rot).T + np.asarray(ring_pos)
        hand_pts = _HAND_POINTS() @ np.asarray(hand_rot).T + np.asarray(hand_pos)
        if skin_nodes is not None and len(skin_nodes):
            hand_pts = np.concatenate([hand_pts, np.asarray(skin_nodes)])
        return min(min_distance_to_hulls(ring_pts, hulls), min_distance_to_hulls(hand_pts, hulls)) / G.MM

    def result(self):
        s = self.last
        c = conditions(s) if s is not None else None
        value = score(s) if s is not None else 0.0
        return dict(score=value, success=value == 1.0, explanation=explain(s) if s else 'no reading',
                    conditions=c, state=dict(s) if s else None)


_HP = []


def _HAND_POINTS():
    if not _HP:
        _HP.append(G.hand_surface_points(0.004))
    return _HP[0]
