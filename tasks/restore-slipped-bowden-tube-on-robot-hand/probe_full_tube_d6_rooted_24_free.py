# SPDX-License-Identifier: CERN-OHL-S-2.0
"""Free curved tube relaxation without gravity or skeleton contact.

Carrier-free D6 discretization comparison. This checks whether the proposed
seated baseline survives its own elastic
relaxation. It has no robot intervention, full hand assembly, or enabled score.
"""
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from hardware import skeleton_arrays
from spherical_tube_rooted import make_tube
from smooth_extension_route import POINTS as DENSE_ROUTE
POINTS = DENSE_ROUTE.copy()
from physics_hooks import on_physics_step
from tube_momentum import momentum
from spherical_tube import energy as physical_energy
def energy(env, tube, reference_height):
    values = physical_energy(env,tube,reference_height)
    values['gravity_j'] = 0.
    values['total_j'] = values['kinetic_j']+values['bending_j']
    return values
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

original_patch = m._Finder.targets['yam_arena.scene']
def fine_physics(scene):
    original_patch(scene)
    scene.PHYSICS_DT = 1./2400.
    original_callback = scene.env_cfg_callback
    def callback(control_hz=None):
        original = original_callback(control_hz)
        def configure(cfg):
            cfg = original(cfg)
            cfg.sim.gravity = (0.,0.,0.)
            return cfg
        return configure
    scene.env_cfg_callback = callback
m._Finder.targets['yam_arena.scene'] = fine_physics
SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = 'Diagnose free elastic relaxation without gravity or skeleton contact.'
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
OBJECTS = [m.fixed('skeleton', C, [('skeleton', mesh, 'body')], [], LOOKS,
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
        asset = env.unwrapped.scene['tube']
        if getattr(self,'_layout_checked_asset',None) is not asset:
            poses = m._np(asset.data.body_link_pose_w)[0]
            origin = m._np(env.unwrapped.scene.env_origins[0])
            errors = [float(np.linalg.norm(poses[asset.body_names.index(name),:3]-origin-C-center))
                      for name,center in tube.body_centers.items()]
            print('TUBE_RUNTIME_LAYOUT',{'body_names':asset.body_names,
                  'maximum_body_center_error_m':max(errors),'errors_m':errors},flush=True)
            if max(errors) > 2e-6:
                raise RuntimeError('Runtime tube layout differs from authored initial geometry')
            self._layout_checked_asset = asset
        tube.set_rest_targets(env)
        self.physics_calls = getattr(self, 'physics_calls', 0) + 1
        if self.physics_calls == 1 or self.physics_calls % 240 == 0:
            print('TUBE_ENERGY', self.physics_calls, energy(env, tube, C[2]), flush=True)
            print('TUBE_MOMENTUM',self.physics_calls,momentum(env,tube),flush=True)
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
                        m.quat_matrix(pose[3:])@(point-tube.body_centers[name]))
        tips = np.array(tips)
        q = m.state(env, 'tube').joints
        energy = sum(.5*tube.joint_stiffness[n]*(q[n]-rest)**2
                     for n,rest in tube.rest_targets.items())
        directions = np.diff(POINTS,axis=0)
        directions /= np.linalg.norm(directions,axis=1)[:,None]
        poses = m._np(asset.data.body_link_pose_w)[0]
        tangents = np.array([m.quat_matrix(poses[asset.body_names.index(name),3:])@direction
                             for name,direction in zip(tube.bodies,directions)])
        spread = float(np.degrees(np.arccos(np.clip(tangents@tangents.T,-1.,1.))).max())
        print('FULL_TUBE_STATE', self.steps, {
            'tips_local': (tips-C).tolist(),
            'tip_displacement_mm': ((tips-C-POINTS[[0, -1]])*1000).tolist(),
            'root_position': m.state(env, 'tube').pos.tolist(),
            'root_speed_m_s': float(np.linalg.norm(m.state(env, 'tube').lin_vel)),
            'bending_energy_j': float(energy),
            'segment_direction_spread_deg': spread,
            'max_joint_angle_deg': float(max(abs(x) for x in q.values())*180/np.pi)}, flush=True)

trial = Trial()
on_physics_step(trial.physics)
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right': [Waypoint(1., (.20, -.25, TOP+.22), -.04695, 'idle fixture', 0.)]}
