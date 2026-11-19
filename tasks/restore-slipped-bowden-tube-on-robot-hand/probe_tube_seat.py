# SPDX-License-Identifier: CERN-OHL-S-2.0
"""Isolated tube-seat load diagnostic. No robot repair or enabled task score.

The skeleton is clamped. A rigid, gravity-disabled 10mm tube coupon starts at
the actual index-extension seat and receives a gradually increasing axial
load. This checks collision support, not tube compliance or complete hardware.
"""

import math
import pathlib
import re
import sys
from dataclasses import replace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from hardware import skeleton_arrays
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

original_patch = m._Finder.targets['yam_arena.scene']


def fine_physics(scene):
    original_patch(scene)
    scene.PHYSICS_DT = 1. / 2400.


m._Finder.targets['yam_arena.scene'] = fine_physics
SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.
INSTRUCTION = 'Engineering tube-seat contact diagnostic.'
C = np.array([.30, 0., TOP + .25])
SEAT = np.array([.0317966046277, -.00999801786384, -.0591213963926])
LENGTH, RO, RI = .010, .0015, .00075
START = C + SEAT - np.array([0., 0., LENGTH / 2 + .00001])
LOOKS = {'body': m.Look(color=m.rgb(.35, .39, .43), roughness=.65),
         'tube': m.Look(color=m.rgb(.90, .90, .85), roughness=.5),
         'mount': m.Look(color=m.rgb(.20, .22, .24), metallic=.6, roughness=.5)}
v, f = skeleton_arrays()
mesh = m.Mesh(v, f)
collider = m.col_hull('skeleton_collision', mesh).replace('"convexHull"', '"none"')
OBJECTS = [m.fixed('skeleton', C, [('skeleton', mesh, 'body')], [collider], LOOKS,
                   friction=(.15, .10)),
           m.fixed('fixture', (.30, 0., TOP),
                   [('base', m.box((.075, .075, .020), (0., 0., .010)), 'mount')],
                   [m.col_box('base_c', (.075, .075, .020), (0., 0., .010))], LOOKS)]

parts, colliders = [], []
for i in range(32):
    a, b = 2 * math.pi * i / 32, 2 * math.pi * (i + 1) / 32
    polygon = [(RO * math.cos(a), RO * math.sin(a)),
               (RO * math.cos(b), RO * math.sin(b)),
               (RI * math.cos(b), RI * math.sin(b)),
               (RI * math.cos(a), RI * math.sin(a))]
    wall = m.prism(polygon, -LENGTH / 2, LENGTH / 2)
    parts.append(('wall_' + str(i), wall, 'tube'))
    colliders.append(m.col_hull('wall_c_' + str(i), wall))

mass = 2200. * math.pi * (RO ** 2 - RI ** 2) * LENGTH
coupon = m.free('coupon', START, parts, colliders, LOOKS, mass=mass,
                friction=(.15, .10), torsional_patch=None, solver_iters=64, com=(0., 0., 0.))
original_build = coupon.build


def coupon_usd():
    text = original_build().replace('"PhysicsMassAPI"',
                                   '"PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI"')
    return re.sub(r'(def Xform "P_coupon"\s*\([^)]*\)\s*\{)',
                  r'\1\n    bool physxRigidBody:disableGravity = true\n    float physxContactReport:threshold = 0',
                  text, count=1)


OBJECTS.append(replace(coupon, build=coupon_usd))


class Trial:
    def reset(self, env=None):
        self.steps = 0
        self.aborted = False

    def step(self, env):
        self.steps += 1
        state = m.state(env, 'coupon')
        if np.linalg.norm(state.pos - START) > .020 or np.linalg.norm(state.lin_vel) > .2:
            self.aborted = True
        force = 0. if self.steps < 20 or self.steps >= 100 or self.aborted else min(3., (self.steps - 20) * .05)
        m.push(env, 'coupon', (0., 0., force))
        if self.steps == 1 or self.steps % 10 == 0:
            tip = state.to_world([(0., 0., LENGTH / 2)])[0]
            print('TUBE_SEAT', self.steps, {'force_n': force, 'position': state.pos.tolist(),
                  'quat': state.quat_xyzw.tolist(), 'tip_offset_mm': ((tip - C - SEAT) * 1000).tolist(),
                  'speed': float(np.linalg.norm(state.lin_vel)), 'aborted': self.aborted}, flush=True)


trial = Trial()
trial.reset()
m.on_reset(trial.reset)
m.on_step(trial.step)


def score(objects):
    return 0.


def success(objects):
    return False


def demo():
    return {'right': [Waypoint(7., (.20, -.25, TOP + .12), -.04695, 'idle', 0.)]}
