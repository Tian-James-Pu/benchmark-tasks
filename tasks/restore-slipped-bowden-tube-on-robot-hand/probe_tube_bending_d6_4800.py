"""Small-load cantilever check of the discrete hollow beam; score disabled."""
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from spherical_tube import make_tube
from physics_hooks import on_physics_step
from point_forces import apply_point_loads
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

original_patch = m._Finder.targets['yam_arena.scene']
def configure_scene(scene):
    original_patch(scene)
    scene.PHYSICS_DT = 1./4800.
    original_callback = scene.env_cfg_callback
    def callback(control_hz=None):
        original = original_callback(control_hz)
        def configure(cfg):
            cfg = original(cfg)
            cfg.sim.gravity = (0., 0., 0.)
            return cfg
        return configure
    scene.env_cfg_callback = callback
m._Finder.targets['yam_arena.scene'] = configure_scene

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = 'Measure hollow-tube bending under a known load.'
SUCCESS_HOLD_S = 0.
POINTS = np.column_stack((np.zeros(9), np.zeros(9), np.linspace(0., .064, 9)))
POSITION = np.array([.30, 0., TOP+.18])
FORCE = .01
tube = make_tube('tube', POSITION, POINTS, fixed_root=True)
from scaled_convex import scaled_convex_part
tube.part = scaled_convex_part(tube.part)
OBJECTS = [tube.part]
FREE_LENGTH = float(POINTS[-1, 2]-POINTS[1, 2])
EI = tube.properties['flexural_modulus_pa']*tube.properties['second_moment_m4']
CONTINUUM_DEFLECTION = FORCE*FREE_LENGTH**3/(3*EI)
DISCRETE_DEFLECTION = FORCE*sum(
    (POINTS[-1, 2]-POINTS[i, 2])**2/stiffness
    for i, stiffness in enumerate(tube.properties['bending_stiffness_Nm_per_rad'], 1))

class Trial:
    active = False
    def tip(self, env):
        asset = env.unwrapped.scene['tube']
        i = asset.body_names.index(tube.bodies[-1])
        pose = m._np(asset.data.body_link_pose_w)[0, i].copy()
        pose[:3] -= m._np(env.unwrapped.scene.env_origins[0])
        return pose[:3]+m.quat_matrix(pose[3:])@(POINTS[-1]-tube.body_centers[tube.bodies[-1]])
    def reset(self, env=None):
        self.active = env is not None
        self.time = 0.
        self.steps = self.calls = 0
        if self.active:
            self.start_tip = self.tip(env)
            print('BENDING_PROPERTIES', dict(tube.properties,
                continuum_deflection_m=CONTINUUM_DEFLECTION,
                discrete_deflection_m=DISCRETE_DEFLECTION,
                free_length_m=FREE_LENGTH), flush=True)
    def physics(self, env, dt):
        tube.set_rest_targets(env)
        loads = []
        if self.active:
            self.time += dt
            force = FORCE*min(1., self.time/.25) if self.time <= 1.5 else 0.
            loads = [('tube', tube.bodies[-1], (force, 0., 0.), self.tip(env))]
            self.calls += 1
        apply_point_loads(env, ['tube'], loads)
    def step(self, env):
        self.steps += 1
        print('TUBE_BENDING', self.steps, {
            'time_s': self.time, 'physics_calls': self.calls,
            'tip_delta_m': (self.tip(env)-self.start_tip).tolist(),
            'continuum_deflection_m': CONTINUUM_DEFLECTION,
            'discrete_deflection_m': DISCRETE_DEFLECTION,
            'joints': m.state(env, 'tube').joints}, flush=True)

trial = Trial()
on_physics_step(trial.physics)
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right': [Waypoint(3., (.20, -.25, TOP+.22), -.04695, 'idle', 0.)]}
