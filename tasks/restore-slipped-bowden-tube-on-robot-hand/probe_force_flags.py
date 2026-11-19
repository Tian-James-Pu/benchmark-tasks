"""Numerical unit calibration, not a hardware task. Scores always zero.

A 1 kg cube on the table tests contact-force reporting against gravity and a
known downward load. A separate gravity-disabled 1 kg cube tests free-body
acceleration under a known horizontal force. No hand geometry is used.
"""
import pathlib
import re
import sys
from dataclasses import replace
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
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
INSTRUCTION = 'Numerical force-unit calibration.'
looks = {'body': m.Look(color=m.rgb(.4, .5, .6))}

def cube(name, pos, gravity):
    p = m.free(name, pos, [('visual', m.box((.1, .1, .1)), 'body')],
               [m.col_box('collision', (.1, .1, .1))], looks, mass=1.,
               torsional_patch=None, solver_iters=64)
    build = p.build
    def usd():
        text = build().replace('"PhysicsMassAPI"',
                              '"PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI"')
        return re.sub(r'(def Xform "P_' + name + r'"\s*\([^)]*\)\s*\{)',
                      r'\1\n    bool physxRigidBody:disableGravity = ' + ('false' if gravity else 'true')
                      + '\n    float physxContactReport:threshold = 0', text, count=1)
    return replace(p, build=usd)

OBJECTS = [cube('resting', (.35, 0., TOP + .05), True),
           cube('floating', (.15, .15, TOP + .30), False)]

class Trial:
    def reset(self, env=None):
        self.step_number = 0
        self.contacts = None
    def step(self, env):
        self.step_number += 1
        if self.contacts is None:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            paths = [str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse()
                     if p.GetName() == 'resting' and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(paths) != 1:
                raise RuntimeError(paths)
            self.contacts = PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths)
        n = self.step_number
        if n==1 or n%5==0:
            import isaaclab.sim as sim_utils
            prim=sim_utils.get_current_stage().GetPrimAtPath('/World/envs/env_0/floating')
            asset=env.unwrapped.scene['floating']
            view=asset.root_view
            info={'usd_disable_gravity':prim.GetAttribute('physxRigidBody:disableGravity').Get(), 'schemas':prim.GetAppliedSchemas()}
            if n==1:
                info['view_methods']=[x for x in dir(view) if any(k in x for k in ('gravit','mass','flag'))]
            for method in ('get_masses','get_disable_gravities'):
                fn=getattr(view,method,None)
                if callable(fn):
                    try:
                        data=fn();info[method]=data.numpy().tolist()
                    except Exception as e:info[method]=str(e)
            print('FORCE_FLAGS',n,info,flush=True)

        down = 5. if 20 <= n < 40 else 0.
        horizontal = .1 if 20 <= n < 30 else 0.
        m.push(env, 'resting', (0., 0., -down))
        m.push(env, 'floating', (horizontal, 0., 0.))
        if n == 1 or n % 5 == 0:
            resting, floating = m.state(env, 'resting'), m.state(env, 'floating')
            print('FORCE_UNITS', n, {
                'down_n': down, 'horizontal_n': horizontal,
                'contact_n': self.contacts.get_net_contact_forces(dt=1./2400.).numpy().tolist(),
                'resting_position': resting.pos.tolist(), 'resting_velocity': resting.lin_vel.tolist(),
                'floating_position': floating.pos.tolist(), 'floating_velocity': floating.lin_vel.tolist(),
            }, flush=True)

trial = Trial()
trial.reset()
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right': [Waypoint(3., (.20, -.25, TOP + .12), -.04695, 'idle', 0.)]}
