"""Two offset forces on one body: numerical wrench calibration, score disabled."""

import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from physics_hooks import on_physics_step
from point_forces import apply_point_loads
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

original_patch = m._Finder.targets['yam_arena.scene']
def configure_scene(scene):
    original_patch(scene)
    scene.PHYSICS_DT = 1. / 240.
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
INSTRUCTION = 'Numerical point-force couple calibration.'
SUCCESS_HOLD_S = 0.
MASS, SIDE, FORCE = 1., .10, .1
INERTIA = MASS * SIDE**2 / 6.
TORQUE = SIDE * FORCE
OBJECTS = [m.free('mass', (.35, 0., TOP+.25),
    [('body', m.box((SIDE,)*3), 'body')],
    [m.col_box('collision', (SIDE,)*3)],
    {'body': m.Look(color=m.rgb(.35, .55, .75))}, mass=MASS,
    torsional_patch=None, solver_iters=64)]

class Trial:
    active = False
    calls = 0
    def physics(self, env, dt):
        loads = []
        if self.active:
            s = m.state(env, 'mass')
            R = s.rotation()
            for sign in (-1., 1.):
                loads.append(('mass', None, R @ np.array([sign*FORCE, 0., 0.]),
                              s.pos + R @ np.array([0., 0., sign*SIDE/2])))
            self.calls += 1
        apply_point_loads(env, ['mass'], loads)
    def reset(self, env=None):
        self.active = env is not None
        self.steps, self.calls = 0, 0
        if env is not None:
            import isaaclab.sim as sim_utils
            from pxr import UsdPhysics, PhysxSchema
            bodies = [p for p in sim_utils.get_current_stage().Traverse()
                      if 'mass' in str(p.GetPath()).lower() and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(bodies) != 1:
                raise RuntimeError('Expected exactly one calibration mass')
            self.angular_damping = float(PhysxSchema.PhysxRigidBodyAPI(bodies[0]).GetAngularDampingAttr().Get())
            print('COUPLE_PROPERTIES', {'angular_damping_per_s': self.angular_damping,
                  'mass_kg': MASS, 'uniform_cube_inertia_kg_m2': INERTIA}, flush=True)
            s = m.state(env, 'mass')
            self.start = s.pos.copy()
            self.w0 = s.ang_vel.copy()
    def step(self, env):
        self.steps += 1
        s = m.state(env, 'mass')
        t = self.steps * env.unwrapped.step_dt
        d = self.angular_damping
        response = t if d == 0 else -np.expm1(-d*t)/d
        expected = self.w0*np.exp(-d*t) + np.array([0., TORQUE / INERTIA * response, 0.])
        print('POINT_COUPLE', self.steps,
              {'time_s': float(t), 'physics_calls': self.calls,
               'drift_m': (s.pos-self.start).tolist(), 'v_m_s': s.lin_vel.tolist(),
               'omega_rad_s': s.ang_vel.tolist(), 'expected_omega_rad_s': expected.tolist(),
               'omega_error_rad_s': (s.ang_vel-expected).tolist()}, flush=True)

trial = Trial()
on_physics_step(trial.physics)
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right': [Waypoint(2., (.20, -.25, TOP+.22), -.04695, 'idle', 0.)]}
