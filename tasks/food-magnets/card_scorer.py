"""food-magnets scorer (pure stdlib), r5 (review TK-9NQBPY, Construct validity: two ordinary magnets that must stay on
the board; r4 red-team p3 before that). score(state) implements card.md "Scoring" exactly, reading every threshold from
params.json. Run as a script: prints the negative-control table and the blind-policy sweep, exits non-zero on any
mismatch.

state (raw numbers only, mm / deg; a missing value counts as failed). Released is read over the last 1 s of the run,
grippers as the policy left them; everything else at the end (r3: Kept is an end-state reading, no whole-run maximum):
  food:     3 dicts {box_xy: [x, y] centre in the box frame (box centre, x toward +x), box_top: highest point above the
            box floor, off_table_max: largest distance beyond the table edge or below the table top over the whole run
            (<= 0 = never), grip_clear: least gripper distance over the last 1 s}
  plain:    1 dict (r3: magnet A only) {disp: centre displacement from its start at the end, gap: back-face gap to its
            card at the end, grip_clear}
  ordinary: 2 dicts (r5: the round and the square ordinary magnets, holding nothing) {disp: centre displacement from its
            start at the end, gap: back-face gap to the steel at the end, grip_clear}
  cards:    1 dict {disp: largest displacement of any of its 4 corners from its start at the end, rot: turn (deg) at the
            end, grip_clear}
  box:      {lid_gap: lid free edge above the rim at the side photo, grip_clear, lid_grip_clear}
            (the box is taped at its drawn spot; the In rim is params box.inner_depth, set from gate G2)
"""
import itertools, json, os, random, sys

P = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "params.json")))
S, BOX, R = P["scoring"], P["box"], P["randomisation"]


def _num(d, k):
    v = d.get(k) if isinstance(d, dict) else None
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _le(d, k, t):
    v = _num(d, k); return v is not None and v <= t


def _ge(d, k, t):
    v = _num(d, k); return v is not None and v >= t


def in_walls(f):                 # centre inside the box's inner walls
    xy = f.get("box_xy") if isinstance(f, dict) else None
    if not (isinstance(xy, list) and len(xy) == 2 and all(isinstance(c, (int, float)) and not isinstance(c, bool) for c in xy)):
        return False
    return abs(xy[0]) <= BOX["inner_x"] / 2 and abs(xy[1]) <= BOX["inner_y"] / 2


def inside(f):                   # centre inside the inner walls, no point more than in_above_rim_max above the rim
    h = _num(f, "box_top")
    return in_walls(f) and h is not None and 0 <= h <= BOX["inner_depth"] + S["in_above_rim_max"]


def kept(st):                    # end state: magnet A, every note corner and (r5) both ordinary magnets kept
    pl, ca, od = st.get("plain") or [], st.get("cards") or [], st.get("ordinary") or []
    if len(pl) != P["plain"]["count"] or len(ca) != P["cards"]["count"] or len(od) != P["ordinary"]["count"]:
        return False
    return all(_le(p, "disp", S["kept_disp_max"]) and _le(p, "gap", S["kept_gap_max"]) for p in pl + od) and \
        all(_le(c, "disp", S["kept_disp_max"]) and _le(c, "rot", S["card_rot_max"]) for c in ca)


def released(st):                # every part of both grippers >= release_clear from every magnet, the note, box and lid
    b = st.get("box") or {}
    parts = (st.get("food") or []) + (st.get("plain") or []) + (st.get("ordinary") or []) + (st.get("cards") or [])
    return all(_ge(p, "grip_clear", S["release_clear"]) for p in parts) and \
        _ge(b, "grip_clear", S["release_clear"]) and _ge(b, "lid_grip_clear", S["release_clear"])


def latched(st):                 # a food magnet ever off_table or more beyond the table edge or below its top
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


# ---------------------------------------------------------------- negative controls
def food_on_board():
    return {"box_xy": None, "box_top": None, "off_table_max": -300, "grip_clear": 200}


def food_in_box(x=0, y=0, h=12):
    return {"box_xy": [x, y], "box_top": h, "off_table_max": -300, "grip_clear": 200}


def food_on_table():
    return {"box_xy": [150, 0], "box_top": 12, "off_table_max": -300, "grip_clear": 200}


def base(food, lid_gap=0.0):
    return {"food": food,
            "plain": [{"disp": 0, "gap": 0, "grip_clear": 200} for _ in range(P["plain"]["count"])],
            "ordinary": [{"disp": 0, "gap": 0, "grip_clear": 200} for _ in range(P["ordinary"]["count"])],
            "cards": [{"disp": 0, "rot": 0, "grip_clear": 200} for _ in range(P["cards"]["count"])],
            "box": {"lid_gap": lid_gap, "grip_clear": 200, "lid_grip_clear": 200}}


LID_OPEN = P["geometry"]["lid_open_edge_height"]  # lid free edge above the rim at the 100 deg rest, geometry.md
RIM = BOX["inner_depth"]
ORD_NAMES = P["ordinary"]["names"]                # round, square
BOXED = dict(disp=150, gap=120)                   # an ordinary magnet taken off the board into the box


def mod(st, f):
    f(st); return st


def fm(f, **kw):
    f.update(kw); return f


def rows():
    T = []
    add = lambda name, st, exp: T.append((name, st, exp))
    B3 = lambda lid=0.0: base([food_in_box() for _ in range(3)], lid)
    A = S["in_above_rim_max"]
    add("start scene / do nothing", base([food_on_board() for _ in range(3)], LID_OPEN), 0.0)
    add("full: 3 in box, lid shut, note and ordinary magnets kept, released", base([food_in_box(-30, -40), food_in_box(0, 0), food_in_box(25, 40)]), 1.0)
    add("full, magnets stacked (stack top 32 above the floor)", base([food_in_box(0, 0, 12), food_in_box(0, 0, 24), food_in_box(0, 0, 32)]), 1.0)
    add("magnet top 4 above the rim, lid open (boundary in)", base([food_in_box(), food_in_box(), food_in_box(0, 0, RIM + A)], LID_OPEN), 0.5)
    add("magnet top 4.5 above the rim, lid open (boundary out)", base([food_in_box(), food_in_box(), food_in_box(0, 0, RIM + A + 0.5)], LID_OPEN), 0.0)
    add("cookie leaning on the far wall, top 3.07 above the rim, lid resting on it (gap 3.2): shut", base([food_in_box(), food_in_box(), food_in_box(30, 0, RIM + 3.07)], 3.2), 1.0)
    add("magnet propped on edge, 4.5 above the rim, lid on it (gap 4.7)", base([food_in_box(), food_in_box(), food_in_box(30, 0, RIM + 4.5)], 4.7), 0.0)
    add("full, lid gap 4 mm (boundary in)", B3(4.0), 1.0)
    add("lid gap 4.5 mm (propped, boundary out)", B3(4.5), 0.5)
    add("3 in box, lid left open", B3(LID_OPEN), 0.5)
    add("lid bounced back open by the side photo (gap 12)", B3(12.0), 0.5)
    add("2 in box + 1 on the counter, lid shut (r3: no counter tier)", base([food_in_box(), food_in_box(), food_on_table()]), 0.0)
    add("all 3 knocked onto the counter, note kept (r3: no counter tier)", base([food_on_table() for _ in range(3)], LID_OPEN), 0.0)
    add("2 in box, 1 still on the board", base([food_in_box(), food_in_box(), food_on_board()]), 0.0)
    add("held just above the box at the run end, not released (grip_clear 5)", base([food_in_box(), food_in_box(), fm(food_in_box(0, 0, 50), grip_clear=5)]), 0.0)
    add("third magnet still pinched over the open box at the run end (grip_clear 0)", base([food_in_box(), food_in_box(), fm(food_in_box(0, 0, 60), grip_clear=0)], LID_OPEN), 0.0)
    add("gripper 29 mm from the lid (boundary out)", mod(B3(), lambda s: s["box"].update(lid_grip_clear=29)), 0.0)
    add("gripper 30 mm from everything (boundary in)", mod(B3(), lambda s: s["box"].update(grip_clear=30, lid_grip_clear=30)), 1.0)
    for part, path in (("a food magnet", ("food", 0)), ("plain magnet A", ("plain", 0)), ("the round ordinary magnet", ("ordinary", 0)),
                       ("the square ordinary magnet", ("ordinary", 1)), ("the note card", ("cards", 0)), ("the box body", ("box", None))):
        for g, e in ((29, 0.0), (30, 1.0)):
            def setg(s, path=path, g=g):
                (s[path[0]] if path[1] is None else s[path[0]][path[1]]).update(grip_clear=g)
            add(f"gripper {g} mm from {part}, all else full ({'boundary out' if g < 30 else 'boundary in'})", mod(B3(), setg), e)
    add("magnet inside the shut box still settling (not read)", base([food_in_box(), food_in_box(), fm(food_in_box(), move_mm=3)]), 1.0)
    add("magnet in the box corner (x 34, y 58: boundary in)", base([food_in_box(34, 58), food_in_box(), food_in_box()]), 1.0)
    add("magnet centre just outside the inner wall (x 35)", base([food_in_box(35, 0, 12), food_in_box(), food_in_box()]), 0.0)
    add("magnet centre just outside the inner end wall (y 59)", base([food_in_box(0, 59, 12), food_in_box(), food_in_box()]), 0.0)
    add("everything boxed incl. the note, its magnet and both ordinary magnets", mod(B3(),
        lambda s: (s["plain"][0].update(disp=180, gap=150), s["cards"][0].update(disp=170, rot=60),
                   [o.update(**BOXED) for o in s["ordinary"]])), 0.0)
    # r5 (review TK-9NQBPY): the route the review names, every magnet not holding the note boxed, no longer scores
    add("box every loose magnet (every magnet with no card under it: the 3 food and both ordinary magnets), lid shut "
        "(r5: the ordinary magnets are no hazard and must stay up)",
        mod(base([food_in_box(-30, -40), food_in_box(0, 0), food_in_box(25, 40)]), lambda s: [o.update(**BOXED) for o in s["ordinary"]]), 0.0)
    add("box every loose magnet, lid left open", mod(B3(LID_OPEN), lambda s: [o.update(**BOXED) for o in s["ordinary"]]), 0.0)
    for i, nm in enumerate(ORD_NAMES):
        add(f"the 3 food and the {nm} ordinary magnet boxed, lid shut", mod(B3(), lambda s, i=i: s["ordinary"][i].update(**BOXED)), 0.0)
    add("box every magnet: plain magnet A (cookie-sized disc) and both ordinary magnets boxed too, the note fallen",
        mod(B3(), lambda s: (s["plain"][0].update(disp=180, gap=150), s["cards"][0].update(disp=120, rot=40),
                             [o.update(**BOXED) for o in s["ordinary"]])), 0.0)
    add("square ordinary magnet knocked onto the counter, 3 food boxed, lid shut", mod(B3(), lambda s: s["ordinary"][1].update(disp=260, gap=200)), 0.0)
    add("round ordinary magnet slid 40 mm along the board, 3 food boxed, lid shut", mod(B3(), lambda s: s["ordinary"][0].update(disp=40, gap=0)), 0.0)
    add("ordinary magnet moved 10 mm (boundary in)", mod(B3(), lambda s: s["ordinary"][0].update(disp=10)), 1.0)
    add("ordinary magnet moved 10.5 mm (boundary out)", mod(B3(), lambda s: s["ordinary"][1].update(disp=10.5)), 0.0)
    add("ordinary magnet gap 1 mm to the steel (boundary in)", mod(B3(), lambda s: s["ordinary"][1].update(gap=1)), 1.0)
    add("ordinary magnet tilted off the steel, gap 1.5 mm", mod(B3(), lambda s: s["ordinary"][0].update(gap=1.5)), 0.0)
    add("ordinary magnet taken down and put back on its marks (end: disp 3, gap 0)", mod(B3(), lambda s: s["ordinary"][0].update(disp=3, gap=0)), 1.0)
    add("3 in box, lid open, an ordinary magnet boxed too (0.5 also needs Kept)", mod(B3(LID_OPEN), lambda s: s["ordinary"][0].update(**BOXED)), 0.0)
    add("missing reading (an ordinary magnet's gap unknown)", mod(B3(), lambda s: s["ordinary"][1].update(gap=None)), 0.0)
    add("missing reading (only one ordinary magnet read)", mod(B3(), lambda s: s["ordinary"].pop()), 0.0)
    add("note knocked down (corner disp 120, rot 40)", mod(B3(), lambda s: s["cards"][0].update(disp=120, rot=40)), 0.0)
    add("note nudged: worst corner 10 mm, 5 deg (boundary in)", mod(B3(), lambda s: s["cards"][0].update(disp=10, rot=5)), 1.0)
    add("note nudged: worst corner 10.5 mm (boundary out)", mod(B3(), lambda s: s["cards"][0].update(disp=10.5)), 0.0)
    add("note turned 5 deg about its magnet + slid 1: centre 3.9, worst corner 10.9", mod(B3(), lambda s: s["cards"][0].update(disp=10.9, rot=5)), 0.0)
    add("note turned 5.5 deg, not moved (boundary out)", mod(B3(), lambda s: s["cards"][0].update(rot=5.5)), 0.0)
    add("note taken down and re-hung on its marks (end: corners 4, 2 deg, A gap 0) (r3: harmless, end-state reading)",
        mod(B3(), lambda s: (s["cards"][0].update(disp=4, rot=2), s["plain"][0].update(disp=3, gap=0))), 1.0)
    add("note re-hung 15 mm low", mod(B3(), lambda s: (s["cards"][0].update(disp=15, rot=1), s["plain"][0].update(disp=15))), 0.0)
    add("plain magnet A moved 10 mm (boundary in)", mod(B3(), lambda s: s["plain"][0].update(disp=10)), 1.0)
    add("plain magnet A moved 10.5 mm (boundary out)", mod(B3(), lambda s: s["plain"][0].update(disp=10.5)), 0.0)
    add("plain magnet A gap 1 mm (boundary in)", mod(B3(), lambda s: s["plain"][0].update(gap=1)), 1.0)
    add("plain magnet A lifted 1.5 mm off its card", mod(B3(), lambda s: s["plain"][0].update(gap=1.5)), 0.0)
    add("food magnet dropped off the table, then put in the box", base([food_in_box(), food_in_box(), fm(food_in_box(), off_table_max=40)]), 0.0)
    add("off-table excursion 9 mm (boundary in)", base([food_in_box(), food_in_box(), fm(food_in_box(), off_table_max=9)]), 1.0)
    add("off-table excursion 10 mm (boundary out)", base([food_in_box(), food_in_box(), fm(food_in_box(), off_table_max=10)]), 0.0)
    add("missing reading (lid gap unknown)", mod(B3(), lambda s: s["box"].update(lid_gap=None)), 0.5)
    add("missing reading (food pose unknown)", base([food_in_box(), food_in_box(), {"box_xy": None}]), 0.0)
    add("missing reading (note corner displacement unknown)", mod(B3(), lambda s: s["cards"][0].update(disp=None)), 0.0)
    add("missing reading (A's gap unknown)", mod(B3(), lambda s: s["plain"][0].update(gap=None)), 0.0)
    add("lid shut, magnets left on the board", base([food_on_board() for _ in range(3)], 0.0), 0.0)
    return T


# ---------------------------------------------------------------- blind-policy sweep (r5: six items in six cells)
DIMS = {"strawberry": (P["food"]["strawberry"]["w"], P["food"]["strawberry"]["h"]),
        "cookie": (P["food"]["cookie"]["dia"], P["food"]["cookie"]["dia"]),
        "donut": (P["food"]["donut"]["od"], P["food"]["donut"]["od"]),
        "A": (P["plain"]["dia"], P["plain"]["dia"]),
        "round": (P["ordinary"]["round"]["dia"], P["ordinary"]["round"]["dia"]),
        "square": (P["ordinary"]["square"]["w"], P["ordinary"]["square"]["h"])}
FOODS = ("strawberry", "cookie", "donut")
KEEP = ("A", "round", "square")                                       # must not be taken
ITEMS = FOODS + KEEP
HALF_OPEN, PAD_HALF = P["geometry"]["finger_open"] / 2, 10     # pad half-width 10 mm (estimate)
SPOTS = R["box_spots_y"]
J = R["jitter_y"]
ROWS = R["rows_z"]
CELLS = [(c, r) for r in ("low", "high") for c in R["cols_y"]]          # 6 cells
CENTRE = {cell: (cell[0], (ROWS[cell[1]][0] + ROWS[cell[1]][1]) / 2) for cell in CELLS}
DZ = (ROWS["low"][1] - ROWS["low"][0]) / 2                              # z jitter half-range in a row (10)
GRID = [i - J for i in range(2 * J + 1)]                               # 1 mm jitter grid, y and z, -10..10


def captured(pick, item, y, z):
    """Generous capture model: fingers open 94; a pinch closing along z takes an item whose z extent fits between the
    open fingers and whose y extent overlaps a 20 mm pad (and the converse for y). Used for every item."""
    py, pz, axis = pick
    w, h = DIMS[item]
    if axis == "z":
        return abs(py - y) <= w / 2 + PAD_HALF and abs(pz - z) <= HALF_OPEN - h / 2
    return abs(pz - z) <= h / 2 + PAD_HALF and abs(py - y) <= HALF_OPEN - w / 2


def assignments():
    """r5: the six items fill the six cells in any order: 6! = 720."""
    return list(itertools.permutations(CELLS))


def cell_probs(pattern):
    """pattern: list of (dy, dz, axis) offsets from a cell centre (all cells alike: same jitter ranges, no pinch reaches
    a neighbouring cell). Returns P(capture) for each item over the 21 x 21 jitter grid (uniform)."""
    out = {}
    for item in ITEMS:
        hit = sum(any(captured(p, item, dy, dz) for p in pattern) for dy in GRID for dz in GRID)
        out[item] = hit / len(GRID) ** 2
    return out


def plan_mean(cell_patterns, probs, A_=None):
    """Exact mean over all 720 assignments x 4 box spots x the jitter grid of a plan that pinches cell_patterns[i] in cell
    i, carries every capture to one fixed drop spot and pushes the lid there. Success (1.0) needs all three food
    captured, none of A, round and square captured, and the box drawn at the drop spot (spots 130 apart > the 122 box:
    one spot only); every other outcome scores 0."""
    tot = 0.0
    A_ = A_ or assignments()
    for cells in A_:
        pr = 1.0
        for item, c in zip(ITEMS, cells):
            q = probs[cell_patterns[CELLS.index(c)]][item]
            pr *= q if item in FOODS else 1 - q
        tot += pr
    return tot / len(A_) / len(SPOTS)


def pattern_library():
    lib = {"none": []}
    for ax in ("z", "y"):
        for dy in range(-40, 41, 5):
            for dz in range(-40, 41, 5):
                lib[f"1@({dy},{dz}){ax}"] = [(dy, dz, ax)]
        for a in range(0, 41, 2):
            lib[f"2y+-{a}{ax}"] = [(-a, 0, ax), (a, 0, ax)]
            lib[f"2z+-{a}{ax}"] = [(0, -a, ax), (0, a, ax)]
            for b in range(0, 41, 4):
                lib[f"4+-{a},+-{b}{ax}"] = [(da, db, ax) for da in (-a, a) for db in (-b, b)]
    lib["8 both axes +-30,+-30"] = [(da, db, ax) for da in (-30, 30) for db in (-30, 30) for ax in ("y", "z")]
    return lib


def analytic_bound():
    """Upper bound for EVERY fixed plan. The cookie, the doughnut and A share the 40 x 40 outline, so any cell's pattern
    captures each of them with the same probability p_c (checked over the library below). Success needs the cookie's
    and the doughnut's cells to capture and A's cell not to: E[p_a p_b (1 - p_c)] over ordered distinct cells (a, b, c)
    of the random order, a multilinear function of p in [0, 1]^6, so its maximum is at a 0/1 vertex: k cells that
    capture give k (k - 1) (6 - k) / 120, at most 0.2 (k = 4). The box must also stand at the one drop spot: 1/4.
    (The strawberry, round and square conditions only lower it.)"""
    best = max(k * (k - 1) * (6 - k) for k in range(7)) / 120
    return best / len(SPOTS)


def draw_state(plan_abs, draw, drop):
    """End state of a fixed absolute plan on one concrete draw (generous: every capture lands at the drop spot; the lid is
    pushed shut there; a captured A takes its note down; a captured ordinary magnet is boxed)."""
    (n, ny, nz), foods, ords, spot = draw
    hit = drop == spot
    fo = []
    for name, y, z in foods:
        if any(captured(p, name, y, z) for p in plan_abs):
            fo.append(food_in_box() if hit else food_on_table())
        else:
            fo.append(food_on_board())
    st = base(fo, 0.0 if hit else LID_OPEN)
    if any(captured(p, "A", ny, nz) for p in plan_abs):
        st["plain"][0].update(disp=150, gap=150); st["cards"][0].update(disp=150, rot=40)
    for i, (name, y, z) in enumerate(ords):
        if any(captured(p, name, y, z) for p in plan_abs):
            st["ordinary"][i].update(**BOXED)
    return st


def concrete_draw(rng):
    cells = rng.choice(assignments())
    jit = lambda cell: (CENTRE[cell][0] + rng.uniform(-J, J), CENTRE[cell][1] + rng.uniform(-DZ, DZ))
    pos = {item: jit(c) for item, c in zip(ITEMS, cells)}
    foods = [(f,) + pos[f] for f in FOODS]
    ords = [(o,) + pos[o] for o in ("round", "square")]
    return ((cells[3],) + pos["A"], foods, ords, rng.randrange(len(SPOTS)))


def to_abs(cell_patterns, lib):
    return [(CENTRE[c][0] + dy, CENTRE[c][1] + dz, ax) for c, k in zip(CELLS, cell_patterns) for dy, dz, ax in lib[k]]


def main():
    bad = 0
    print("negative controls (expected from the written ladder vs scorer.py)")
    T = rows()
    for name, st, exp in T:
        got = score(st)
        ok = abs(got - exp) < 1e-9
        bad += not ok
        print(f"  {'ok ' if ok else 'BAD'} {name}: expected {exp:.2f}, scorer {got:.2f}")
    print(f"{len(T)} negative-control rows")
    lib = pattern_library()
    probs = {k: cell_probs(v) for k, v in lib.items()}
    A_ = assignments()
    print(f"\nblind-policy sweep (r5): 6 cells {CELLS} (z ranges {ROWS}); the note's magnet A, 3 food and 2 ordinary magnets fill "
          f"them in a random order ({len(A_)} assignments), each centre jittered y +/-{J}, z +/-{DZ:g} (1 mm grid), x 4 box spots {SPOTS}")
    cookie_eq = all(abs(pr["cookie"] - pr["A"]) < 1e-12 and abs(pr["donut"] - pr["A"]) < 1e-12 for pr in probs.values())
    print(f"pattern library: {len(lib)} per-cell pinch patterns (1, 2, 4 or 8 pinches, both closing axes); cookie, doughnut and A "
          f"captured with equal probability by every pattern: {cookie_eq}")
    bad += not cookie_eq
    bound = analytic_bound()
    print(f"upper bound for every fixed plan (see analytic_bound): {bound:.4f}")
    same = max(((plan_mean([k] * len(CELLS), probs, A_), k) for k in probs), key=lambda t: t[0])
    print(f"best plan using one pattern in every cell (exact over the library): {same[1]}: mean {same[0]:.4f}")
    take = next(k for k in probs if probs[k]["cookie"] == 1.0 and probs[k]["strawberry"] == 1.0)
    cells3 = [take, take, take, "none", "none", "none"]
    m3 = plan_mean(cells3, probs, A_)
    print(f"cell-level reference (take everything in 3 fixed cells with {take}, none of the other 3: succeeds only if the 3 food "
          f"sit in those cells, 1/20, and the box at the drop spot, 1/4): exact mean {m3:.4f}")
    plan_abs = to_abs(cells3, lib)
    rng = random.Random(20261124)
    N = 20000
    mc = sum(score(draw_state(plan_abs, d, 0)) for d in (concrete_draw(rng) for _ in range(N))) / N
    print(f"cross-check: that plan, drop at spot {SPOTS[0]}, scorer.score on {N} random draws (seed 20261124): mean {mc:.4f}")
    loose = base([food_in_box(-30, -40), food_in_box(0, 0), food_in_box(25, 40)])
    for o in loose["ordinary"]:
        o.update(**BOXED)
    print(f"the reviewed route, every loose magnet (no card under it) boxed and the lid shut: {score(loose):.2f}")
    bad += score(loose) != 0.0
    sweep = score(base([food_on_table() for _ in range(3)], LID_OPEN))
    print(f"band sweeps alone (every food magnet knocked to the counter, note kept): {sweep:.2f} (no counter tier)")
    ok = max(bound, same[0], m3, mc) <= 0.125
    print(f"sweep: bound {bound:.4f}, best uniform {same[0]:.4f}, 3-cell {m3:.4f}, Monte Carlo {mc:.4f} (target well under 0.25; "
          f"check <= 0.125): {'ok' if ok else 'FAIL'}")
    bad += not ok
    print("ALL PASS" if not bad else f"{bad} FAILURES")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
