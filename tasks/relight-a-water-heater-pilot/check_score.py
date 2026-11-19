"""The gas control's model and the scorer on hand-built states, with no simulator.

    python check_score.py

Exits non-zero if any case gives another value than the scoring text (task.json) or the control's documented behaviour.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys
import types


class _Any:
    """Stands in for every geometry and USD call made while task.py builds its scene."""

    def __call__(self, *a, **kw):
        return self

    def __getattr__(self, name):
        return self

    def __str__(self):
        return "/W"

    __format__ = lambda self, spec: "/W"  # noqa: E731


for name in ("demokit", "mechkit", "simready", "yam_arena", "yam_arena.task_file"):
    sys.modules[name] = types.ModuleType(name)
sys.modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.0}
sys.modules["simready"].SimReady = _Any()
mk = sys.modules["mechkit"]
mk.__getattr__ = lambda name: _Any()
mk.ENV_PATH = "/W"
for fn in ("set_target", "lock", "unlock", "set_visible", "on_step", "on_reset"):
    setattr(mk, fn, lambda *a, **kw: None)
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

bad = 0


def check(label, got, want):
    global bad
    ok = got == want if isinstance(want, bool) else abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {label}: {got} (want {want})")


class State:
    """The control's joints as the simulator reports them: knob in degrees, push-in and igniter in metres."""

    def __init__(self):
        self.knob, self.plunger, self.igniter = 0.0, 0.0, 0.0

    def deg(self, link):
        return self.knob

    def joint(self, link):
        return {"plunger": self.plunger, "igniter": self.igniter, "knob": math.radians(self.knob)}[link]


S = State()
ENV = types.SimpleNamespace(unwrapped=types.SimpleNamespace(step_dt=0.05))
t.m.state = lambda env, name: S
CLEAR = {"now": True}
t.hands_clear = lambda env: CLEAR["now"]


def run(seconds, click_every=None):
    """Step the control and the scorer for `seconds`; with `click_every`, press the igniter fully once that often."""
    n = round(seconds / 0.05)
    for k in range(n):
        if click_every:
            phase = (k * 0.05) % click_every
            S.igniter = 0.0045 if phase < 0.3 else 0.0
        t.GAS.step(ENV)
        t.TRIAL.step(ENV)
    S.igniter = 0.0


def start():
    t.GAS.reset()
    t.TRIAL.reset()
    S.knob, S.plunger, S.igniter = 0.0, 0.0, 0.0
    CLEAR["now"] = True


# -- the control ---------------------------------------------------------------------------------------------------------------------------------------
start()
S.knob, S.plunger = t.PILOT, 0.0
run(2.0, click_every=1.0)
check("clicking with the knob not pushed in: no flame", t.GAS.lit_ever, False)
S.plunger = 0.004
run(3.0, click_every=1.0)
check("knob in 4 mm (valve still shut): no flame", t.GAS.lit_ever, False)
check("knob in 4 mm: no gas let out", t.GAS.unlit_gas_s, 0.0)
S.plunger = 0.0062
run(1.0)
check("knob in 6.2 mm without a spark: gas flows unlit for 1 s", round(t.GAS.unlit_gas_s, 6), 1.0)
run(1.0, click_every=1.0)
check("knob in 6.2 mm and a spark: lit", t.GAS.lit, True)
sparks = t.GAS.sparks
run(5.0, click_every=1.0)
check("each full press after it re-arms sparks once (5 more presses)", t.GAS.sparks - sparks, 5)
check("not yet held by the control after 6 s of flame", t.GAS.held, False)
run(35.0)
check("held by the control after 40 s of flame (status light blinking)", t.GAS.held, True)
S.plunger = 0.0
run(5.0)
check("knob let out after the light blinks: still lit", t.GAS.lit, True)
S.knob = t.HOT
run(2.0)
check("turned to HOT: still lit", t.GAS.lit, True)
S.knob = 0.0
run(0.5)
check("turned back to OFF: out", t.GAS.lit, False)

start()
S.knob, S.plunger = t.PILOT, 0.0062
run(20.0, click_every=1.0)
check("lit, held in 20 s", t.GAS.lit, True)
S.plunger = 0.0
run(0.5)
check("let out before the light blinks: out", t.GAS.lit, False)
S.plunger = 0.0062
run(60.0, click_every=1.0)
check("relit at once: burns while the knob is held in", t.GAS.lit, True)
check("relit at once: the control will not hold it (10-minute wait)", t.GAS.held, False)

# -- the scorer: tiers ------------------------------------------------------------------------------------------------------------------------------------
G = types.SimpleNamespace
for label, gas, ang, push, want in (
    ("lit, held, knob out at HOT", G(lit=True, held=True, lit_ever=True), t.HOT, 0.0, 0.5),
    ("lit, held, knob out at HOT + 8", G(lit=True, held=True, lit_ever=True), t.HOT + 8.0, 0.0, 0.5),
    ("lit, held, knob out at PILOT", G(lit=True, held=True, lit_ever=True), t.PILOT, 0.0, 0.5),
    ("lit, held, knob out at LOW", G(lit=True, held=True, lit_ever=True), t.DIAL["LOW"], 0.0, 0.5),
    ("lit, held, knob out at HOT + 8.5", G(lit=True, held=True, lit_ever=True), t.HOT + 8.5, 0.0, 0.25),
    ("lit, held, knob out at A", G(lit=True, held=True, lit_ever=True), t.DIAL["A"], 0.0, 0.25),
    ("lit, held, knob out at VERY HOT", G(lit=True, held=True, lit_ever=True), t.DIAL["VERY HOT"], 0.0, 0.25),
    ("lit and held, knob still pushed in", G(lit=True, held=True, lit_ever=True), t.PILOT, 0.006, 0.25),
    ("lit, not yet held", G(lit=True, held=False, lit_ever=True), t.PILOT, 0.006, 0.25),
    ("lit once, out now", G(lit=False, held=False, lit_ever=True), t.HOT, 0.0, 0.25),
    ("never lit", G(lit=False, held=False, lit_ever=False), t.HOT, 0.0, 0.0),
):
    check(f"tier, {label}", t.moment(ang, push, gas), want)

# -- the scorer: the 5 s hold, hands clear, and the unburnt-gas record --------------------------------------------------------------------------------
start()
S.knob, S.plunger = t.PILOT, 0.0062
run(45.0, click_every=1.0)
S.plunger, S.knob = 0.0, t.HOT
run(4.95)
check("at HOT, hands clear, 99 steps", t.TRIAL.score, 0.5)
check("no success after 99 steps", t.success(None), False)
run(0.05)
check("at HOT, hands clear, 100 steps (5 s)", t.TRIAL.score, 1.0)
check("success after 100 steps", t.success(None), True)

start()
S.knob, S.plunger = t.PILOT, 0.0062
run(45.0, click_every=1.0)
S.plunger, S.knob = 0.0, t.HOT
CLEAR["now"] = False
run(10.0)
check("at HOT with a hand within 3 cm of the knob, 10 s", t.TRIAL.score, 0.5)
check("no success with a hand on the knob", t.success(None), False)

start()
S.knob, S.plunger = t.PILOT, 0.0062
run(90.0)
check("knob in 90 s unlit: at the limit, no violation yet", t.TRIAL.violation, False)
run(0.1)
check("knob in 90.1 s unlit: violation", t.TRIAL.violation, True)
run(45.0, click_every=1.0)
S.plunger, S.knob = 0.0, t.HOT
run(10.0)
check("then lit, held and set to HOT for 10 s: still 0", t.TRIAL.score, 0.0)
check("no success after the violation", t.success(None), False)
S.knob, S.plunger = 0.0, 0.0
run(1.0)
check("knob back at OFF, the starting state: the violation stays", t.TRIAL.violation, True)
t.TRIAL.reset()
check("a reset clears the record", t.TRIAL.violation, False)

print("all checks passed" if not bad else f"{bad} checks FAILED")
sys.exit(1 if bad else 0)
