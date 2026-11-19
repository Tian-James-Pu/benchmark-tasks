"""Adversarial observer replay, not a physics or robot feasibility test."""
import json
import sys
from types import SimpleNamespace
import numpy as np
from yam_arena import task_file

t = task_file.load(sys.argv[1])
g = t.score.__globals__
origin = np.array([g['X_V'], g['Y_V'], g['TOP']])
pose = origin.copy()
handle = g['H_OFF']
far = np.tile([0., 2., 2.], (24, 1))
tips = far.copy()


class Ring:
    def to_world(self, points):
        return np.asarray(points) + origin

    def to_local(self, points):
        return np.asarray(points) - origin


g['m'].state = lambda env, name: Ring() if name == 'ring' else SimpleNamespace(joint=lambda name: handle)
g['m'].fingertips = lambda env: tips
g['_valve_pose'] = lambda env: (pose.copy(), np.eye(3))
g['_show_jet'] = lambda *args: None
g['_set_limits'] = lambda *args: None
g['_write_joint'] = lambda *args, **kwargs: None
g['m'].set_target = lambda *args: None
env = SimpleNamespace(unwrapped=SimpleNamespace(step_dt=.05))


def step(n=1):
    for _ in range(n):
        g['_step'](env)


g['_reset'](env)
step(60)
assert g['REC'].full_steps == 59 and t.score(None) == .4 and not t.success(None)
step()
assert t.score(None) == 1. and t.success(None)
before = vars(g['REC']).copy()
for _ in range(20):
    assert t.score(None) == 1. and t.success(None)
assert vars(g['REC']) == before, 'score queries mutate observer history'

# A hand returning to the valve must cancel full credit and restart its dwell.
tips = np.tile(origin + [0., 0., g['Z_F']+.18], (24, 1))
step()
assert t.score(None) == 0. and g['REC'].full_steps == 0
tips = far.copy()
step(59)
assert t.score(None) == .4 and not t.success(None)
step()
assert t.score(None) == 1.

# Spray exposure remains disqualifying after restoring the initial body poses,
# shutting off the valve and waiting longer than the success dwell.
handle = 0.
tips = np.tile(g['REST'] + [0., 0., -.1], (24, 1))
step()
assert g['REC'].spray_out
tips = far.copy()
handle = g['H_OFF']
pose = origin.copy()
step(100)
assert t.score(None) == 0. and not t.success(None)
g['_reset'](env)
step(61)
assert t.score(None) == 1. and not g['REC'].spray_out

# A stationary head outside the hanging region can get only the stated partial.
pose = origin + [0., .09, 0.]
step(100)
assert t.score(None) == .4 and not t.success(None)
handle = 0.
step(100)
assert t.score(None) == 0.
print(json.dumps({'observer_replay':'pass', 'checks':[
    '59/60-step dwell boundary', 'pure repeated score/success calls',
    'held-state rejection and dwell restart', 'permanent spray exposure despite restored poses',
    'actual reset callback clears episode history', 'partial off-position outcome', 'on-state rejection'
], 'physics_validated':False}))
