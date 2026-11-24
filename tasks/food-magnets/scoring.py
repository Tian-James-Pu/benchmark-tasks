"""food-magnets scoring: exactly the card's rule (card_scorer.py, r5: review TK-9NQBPY added two ordinary magnets that
must stay on the board), read from the simulator's rigid poses.

``score(state)`` is the card scorer's function, line for line (thresholds from params.json). ``ScoreTracker`` builds
that state dict from the bodies' poses every control step:
  food:  box_xy (centre in the box frame, mm), box_top (highest point above the box floor, mm; None outside the walls),
         off_table_max (largest excursion beyond the table edge or below the table top over the whole run, mm),
         grip_clear (least distance from any gripper hull to it over the last 1 s, mm)
  plain: disp (centre displacement from its start), gap (back face to the card's front face), grip_clear
  ordinary (r5, round and square): disp (centre displacement from its start), gap (back face to the steel: the largest
         back-face rim point's distance off the sheet's front face), grip_clear
  cards: disp (largest corner displacement from its start), rot (turn from its start, deg), grip_clear
  box:   lid_gap (lid free edge above the rim), grip_clear (box body), lid_grip_clear
Released is read over the last 1 s of the run, grippers as the policy left them. Then (card, Scoring) the arms are
jogged to ready (run.py), and In, Lid shut and Kept are read from that final state; success() uses the same reading.
Gripper distances: every collision hull of links gripper, tip_left and tip_right (yam_arena.analysis), the largest
plane distance to each object surface sample (conservative: never more than the true distance).
"""
import math
from collections import deque

import numpy as np

import geometry as G

P = G.P
S, BOX, R = P["scoring"], P["box"], P["randomisation"]
GRIPPER_LINKS = ('gripper', 'tip_left', 'tip_right')
FOOD = G.NAMES
ORDINARY = G.ORDINARY


# ---------------------------------------------------------------- the card's scorer (card_scorer.py r5), unchanged
def _num(d, k):
    v = d.get(k) if isinstance(d, dict) else None
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _le(d, k, t):
    v = _num(d, k); return v is not None and v <= t


def _ge(d, k, t):
    v = _num(d, k); return v is not None and v >= t


def in_walls(f):
    xy = f.get("box_xy") if isinstance(f, dict) else None
    if not (isinstance(xy, list) and len(xy) == 2 and all(isinstance(c, (int, float)) and not isinstance(c, bool) for c in xy)):
        return False
    return abs(xy[0]) <= BOX["inner_x"] / 2 and abs(xy[1]) <= BOX["inner_y"] / 2


def inside(f):
    h = _num(f, "box_top")
    return in_walls(f) and h is not None and 0 <= h <= BOX["inner_depth"] + S["in_above_rim_max"]


def kept(st):
    pl, ca, od = st.get("plain") or [], st.get("cards") or [], st.get("ordinary") or []
    if len(pl) != P["plain"]["count"] or len(ca) != P["cards"]["count"] or len(od) != P["ordinary"]["count"]:
        return False
    return all(_le(p, "disp", S["kept_disp_max"]) and _le(p, "gap", S["kept_gap_max"]) for p in pl + od) and \
        all(_le(c, "disp", S["kept_disp_max"]) and _le(c, "rot", S["card_rot_max"]) for c in ca)


def released(st):
    b = st.get("box") or {}
    parts = (st.get("food") or []) + (st.get("plain") or []) + (st.get("ordinary") or []) + (st.get("cards") or [])
    return all(_ge(p, "grip_clear", S["release_clear"]) for p in parts) and \
        _ge(b, "grip_clear", S["release_clear"]) and _ge(b, "lid_grip_clear", S["release_clear"])


def latched(st):
    fo = st.get("food") or []
    return any(_num(f, "off_table_max") is None or f["off_table_max"] >= S["off_table"] for f in fo)


def score(st):
    fo = st.get("food") or []
    if len(fo) != P["food"]["count"] or latched(st) or not released(st):
        return 0.0
    k, all_in = kept(st), all(inside(f) for f in fo)
    lid = _le(st.get("box") or {}, "lid_gap", S["lid_gap_max"])
    if all_in and lid and k:
        return 1.0
    if all_in and k:
        return 0.5
    return 0.0


def explain(st):
    fo = st.get("food") or []
    if latched(st):
        return 'zero: a food magnet went off the table'
    if not released(st):
        return 'zero: not released (a gripper within 30 mm of a magnet, the note, the box or the lid in the last 1 s)'
    parts = [f"{n} {'in' if inside(f) else 'not in'}" for n, f in zip(FOOD, fo)]
    lid = _le(st.get("box") or {}, "lid_gap", S["lid_gap_max"])
    return ', '.join(parts + [f"lid {'shut' if lid else 'open'}",
                              f"note and ordinary magnets {'kept' if kept(st) else 'not kept'}"])


# ---------------------------------------------------------------- pose readings
def surface_points(name, pos, rot):
    """Surface samples of a body (world, m): magnet prisms, the card plate, the box body, the lid."""
    pos, rot = np.asarray(pos, float), np.asarray(rot, float)
    if name in G.EXTENT:
        local = G.prism(name, 24)
        mid = local.copy()
        mid[:, 0] = 0.0
        local = np.vstack([local, mid, np.array([[s * G.THICK[name] / 2, 0, 0] for s in (-1, 1)])])
    elif name == 'card':
        ys, zs = np.meshgrid(np.linspace(-G.CARD_W / 2, G.CARD_W / 2, 13), np.linspace(-G.CARD_H / 2, G.CARD_H / 2, 9))
        local = np.column_stack([np.zeros(ys.size), ys.ravel(), zs.ravel()])
    elif name == 'lid':
        xs, ys = np.meshgrid(np.linspace(0, G.BOX_LX, 8), np.linspace(-G.BOX_LY / 2, G.BOX_LY / 2, 12))
        local = np.vstack([np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)]) for z in (0.0, G.LID_T)])
    elif name == 'box':
        xs, ys = np.meshgrid(np.linspace(-G.BOX_LX / 2, G.BOX_LX / 2, 8), np.linspace(-G.BOX_LY / 2, G.BOX_LY / 2, 12))
        zs = np.linspace(0, G.BOX_H, 5)
        rim = [(x, y) for x, y in zip(xs.ravel(), ys.ravel())
               if abs(abs(x) - G.BOX_LX / 2) < 1e-9 or abs(abs(y) - G.BOX_LY / 2) < 1e-9]
        local = np.array([(x, y, z) for x, y in rim for z in zs])
    else:
        raise KeyError(name)
    return local @ rot.T + pos


def gripper_hulls(hulls):
    return [(n, h) for n, h in hulls if n in GRIPPER_LINKS]


def min_distance_to_hulls(points, hulls):
    """Least distance (m) from points to convex hulls (largest plane distance: never more than the true distance)."""
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


def _turn_deg(r0, r1):
    c = (np.trace(np.asarray(r0).T @ np.asarray(r1)) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class ScoreTracker:
    """Per-step reader of the rigid poses. ``update(poses, hulls=None)`` with poses = {name: (pos, rot)} for the three food
    magnets, ``plain``, ``card`` and ``lid``; ``hulls`` = the robot's world hulls when read this step."""

    def __init__(self, draw, control_hz=20):
        self.draw = draw
        self.hz = int(round(control_hz))
        self.box_c = np.array([G.BOX_X, draw['box_y'], G.TABLE_TOP])
        self.start = None
        self.off = {n: -np.inf for n in FOOD}
        self.clear = deque(maxlen=self.hz)        # last 1 s of gripper readings (one per step when read every step)
        self.steps = 0

    def _grip(self, poses, hulls):
        gh = gripper_hulls(hulls)
        d = {n: min_distance_to_hulls(surface_points(n, *poses[n]), gh) / G.MM for n in (*FOOD, 'plain', *ORDINARY, 'card', 'lid')}
        d['box'] = min_distance_to_hulls(surface_points('box', self.box_c, np.eye(3)), gh) / G.MM
        return d

    def update(self, poses, hulls=None):
        self.steps += 1
        if self.start is None:
            self.start = {n: (np.asarray(p, float).copy(), np.asarray(r, float).copy()) for n, (p, r) in poses.items()}
        for n in FOOD:
            pts = surface_points(n, *poses[n])
            t = G.TABLE
            beyond = max((t['x_min'] * G.MM - pts[:, 0]).max(), (pts[:, 0] - t['x_max'] * G.MM).max(),
                         (np.abs(pts[:, 1]) - t['y_half'] * G.MM).max(), (G.TABLE_TOP - pts[:, 2]).max())
            self.off[n] = max(self.off[n], beyond / G.MM)
        if hulls is not None:
            self.clear.append(self._grip(poses, hulls))
        return self.result(poses)

    def state(self, poses):
        st = dict(food=[], plain=[], ordinary=[], cards=[], box={})
        clear = {}
        if len(self.clear) == self.clear.maxlen:
            for k in self.clear[0]:
                clear[k] = float(min(c[k] for c in self.clear))
        floor = G.TABLE_TOP + G.FLOOR_T
        for n in FOOD:
            p, r = poses[n]
            local = np.asarray(p) - self.box_c
            xy = [float(local[0] / G.MM), float(local[1] / G.MM)]
            top = float((surface_points(n, p, r)[:, 2].max() - floor) / G.MM)
            st['food'].append(dict(box_xy=xy, box_top=top, off_table_max=float(self.off[n]), grip_clear=clear.get(n)))
        p, r = poses['plain']
        p0, _ = self.start['plain']
        cp, cr = poses['card']
        rim = np.asarray(p) + np.asarray(r) @ (G.back_offset('plain')[:, None] + 0 * np.zeros((3, 1)))[:, 0]
        # gap: the plain magnet's back-face rim points to the card's front face (signed, along the card normal)
        n_card = np.asarray(cr)[:, 0]
        front = np.asarray(cp) - n_card * G.CARD_T / 2
        u, v = np.asarray(r)[:, 1], np.asarray(r)[:, 2]
        ring = [rim + G.DISC_R * (math.cos(a) * u + math.sin(a) * v) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
        gaps = [float(np.dot(front - q, n_card)) for q in ring + [rim]]
        st['plain'].append(dict(disp=float(np.linalg.norm(np.asarray(p) - p0) / G.MM), gap=float(min(gaps) / G.MM),
                                grip_clear=clear.get('plain')))
        for n in ORDINARY:
            # r5: an ordinary magnet stays where it was, back face on the steel (gap = geometry.magnet_gap: the back-face
            # disc's nearest point to the sheet's front face, the same reading the magnetic pull uses; None if it faces away)
            p, r = poses[n]
            p0, _ = self.start[n]
            g, nx, _ = G.magnet_gap(n, p, r)
            st['ordinary'].append(dict(name=n, disp=float(np.linalg.norm(np.asarray(p) - p0) / G.MM),
                                       gap=float(g / G.MM) if nx > 0 else None, grip_clear=clear.get(n)))
        c0, r0 = self.start['card']
        corners0, corners1 = G.card_corners(c0, r0), G.card_corners(cp, cr)
        st['cards'].append(dict(disp=float(np.linalg.norm(corners1 - corners0, axis=1).max() / G.MM),
                                rot=float(_turn_deg(r0, cr)), grip_clear=clear.get('card')))
        lp, lr = poses['lid']
        st['box'] = dict(lid_gap=float((G.lid_free_edge(lp, lr)[2] - (G.TABLE_TOP + G.BOX_H)) / G.MM),
                         grip_clear=clear.get('box'), lid_grip_clear=clear.get('lid'))
        return st

    def result(self, poses):
        st = self.state(poses)
        s = score(st)
        return dict(score=s, success=s >= 1.0, explanation=explain(st), state=st)
