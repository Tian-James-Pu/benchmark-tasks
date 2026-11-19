# SPDX-License-Identifier: CERN-OHL-S-2.0
"""Full curved tube in the source skeleton, with gravity and no tendon load.

Carrier-mass representation comparison. This checks whether the proposed
seated baseline survives its own elastic
relaxation. It has no robot intervention, full hand assembly, or enabled score.
"""
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from hardware import skeleton_arrays
from redistributed_carrier_tube import make_tube
from smooth_extension_route import POINTS as DENSE_ROUTE
POINTS = DENSE_ROUTE[::3].copy()
from physics_hooks import on_physics_step
from carrier_tube_energy import energy
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

original_patch = m._Finder.targets['yam_arena.scene']
def fine_physics(scene):
    original_patch(scene)
    scene.PHYSICS_DT = 1./4800.
m._Finder.targets['yam_arena.scene'] = fine_physics
SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = 'Inspect the seated elastic tube in a clamped hand skeleton.'
SUCCESS_HOLD_S = 0.
C = np.array([.30, 0., TOP+.25])
LOOKS = {'body': m.Look(color=m.rgb(.35, .39, .43), roughness=.65),
         'mount': m.Look(color=m.rgb(.20, .22, .24), metallic=.6, roughness=.5)}
v, f = skeleton_arrays()
mesh = m.Mesh(v, f)
collider = m.col_hull('skeleton_collision', mesh).replace('"convexHull"', '"none"')
tube = make_tube('tube', C, POINTS)
from scaled_convex import scaled_convex_part
tube.part = scaled_convex_part(tube.part)
OBJECTS = [m.fixed('skeleton', C, [('skeleton', mesh, 'body')], [collider], LOOKS,
                  friction=(.15, .10)),
           m.fixed('fixture', (.30, 0., TOP),
                   [('base', m.box((.075, .075, .020), (0., 0., .010)), 'mount')],
                   [m.col_box('base_c', (.075, .075, .020), (0., 0., .010))], LOOKS),
           tube.part]

class Trial:
    def reset(self, env=None):
        self.steps = 0
        if env is not None:
            print('FULL_TUBE_PROPERTIES', tube.properties, flush=True)
    def physics(self, env, dt):
        tube.set_rest_targets(env)
        self.physics_calls = getattr(self, 'physics_calls', 0) + 1
        if self.physics_calls == 1 or self.physics_calls % 480 == 0:
            print('TUBE_ENERGY', self.physics_calls, energy(env, tube, C[2]), flush=True)
    def step(self, env):
        self.steps += 1
        if self.steps != 1 and self.steps%10:
            return
        asset = env.unwrapped.scene['tube']
        tips = []
        for name, point in ((tube.bodies[0], POINTS[0]), (tube.bodies[-1], POINTS[-1])):
            i = asset.body_names.index(name)
            pose = m._np(asset.data.body_link_pose_w)[0, i]
            tips.append(pose[:3]-m._np(env.unwrapped.scene.env_origins[0])+
                        m.quat_matrix(pose[3:])@point)
        tips = np.array(tips)
        q = m.state(env, 'tube').joints
        energy = 0.
        for i, stiffness in enumerate(tube.properties['bending_stiffness_Nm_per_rad'], 1):
            for name in (f'bend_{i}', f'segment_{i}'):
                energy += .5*stiffness*(q[name+'_joint']-tube.rest_targets[name])**2
        print('FULL_TUBE_STATE', self.steps, {
            'tips_local': (tips-C).tolist(),
            'tip_displacement_mm': ((tips-C-POINTS[[0, -1]])*1000).tolist(),
            'root_position': m.state(env, 'tube').pos.tolist(),
            'root_speed_m_s': float(np.linalg.norm(m.state(env, 'tube').lin_vel)),
            'bending_energy_j': float(energy),
            'max_joint_angle_deg': float(max(abs(x) for x in q.values())*180/np.pi)}, flush=True)

trial = Trial()
on_physics_step(trial.physics)
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right': [Waypoint(3., (.20, -.25, TOP+.22), -.04695, 'idle fixture', 0.)]}
