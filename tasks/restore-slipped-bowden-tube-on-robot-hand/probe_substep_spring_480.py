"""Analytic oscillator calibration of physics-rate forces; not a robot task."""

import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from physics_hooks import on_physics_step
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

original_patch = m._Finder.targets['yam_arena.scene']
def configure_scene(scene):
    original_patch(scene)
    scene.PHYSICS_DT = 1. / 480.
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
SUCCESS_HOLD_S = 0.
INSTRUCTION = 'Numerical continuous-force calibration.'
ANCHOR_X, MASS, STIFFNESS, DAMPING = .30, 1., 500., 2.
OBJECTS = [m.free('mass', (.35,0.,TOP+.20),
    [('body',m.box((.05,.05,.05)),'body')],
    [m.col_box('collision',(.05,.05,.05))],
    {'body':m.Look(color=m.rgb(.35,.55,.75))},mass=MASS,
    torsional_patch=None,solver_iters=64)]

class Trial:
    calls = 0
    def physics(self,env,dt):
        state = m.state(env,'mass')
        force = -STIFFNESS*(state.pos[0]-ANCHOR_X)-DAMPING*state.lin_vel[0]
        m.push(env,'mass',(force,0.,0.))
        self.calls += 1
        if hasattr(self, 'baseline_calls') and self.calls-self.baseline_calls <= 8:
            print('SPRING_SUBSTEP', self.calls-self.baseline_calls,
                  {'dt':float(dt),'x':float(state.pos[0]-ANCHOR_X),
                   'vx':float(state.lin_vel[0]),'force':float(force)},flush=True)
    def reset(self,env=None):
        self.steps = 0
        self.baseline_calls = self.calls
        if env is not None:
            state = m.state(env,'mass')
            self.x0 = state.pos[0]-ANCHOR_X
            self.v0 = state.lin_vel[0]
            self.start = state.pos.copy()
            print('SPRING_RESET',{'x0':self.x0,'v0':self.v0,'calls':self.calls},flush=True)
    def step(self,env):
        self.steps += 1
        state = m.state(env,'mass')
        t = self.steps*env.unwrapped.step_dt
        gamma = DAMPING/(2*MASS)
        omega = np.sqrt(STIFFNESS/MASS-gamma**2)
        exact = np.exp(-gamma*t)*(self.x0*np.cos(omega*t)+(self.v0+gamma*self.x0)/omega*np.sin(omega*t))
        x = state.pos[0]-ANCHOR_X
        print('SUBSTEP_SPRING',self.steps,{'time_s':float(t),'x_m':float(x),
            'exact_x_m':float(exact),'error_m':float(x-exact),
            'vx':float(state.lin_vel[0]),'physics_calls':self.calls-self.baseline_calls,
            'cross_axis_drift_m':(state.pos[1:]-self.start[1:]).tolist(),
            'energy_j':float(.5*MASS*state.lin_vel[0]**2+.5*STIFFNESS*x*x)},flush=True)

trial = Trial()
on_physics_step(trial.physics)
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right':[Waypoint(4,(.20,-.25,TOP+.22),-.04695,'idle',0.)]}
