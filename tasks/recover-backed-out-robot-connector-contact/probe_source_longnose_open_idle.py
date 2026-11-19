"""Inspect open passive needle-nose pliers under gravity; no benchmark score."""
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from articulated_tool import tool
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = 'Inspect the tool articulation.'
SUCCESS_HOLD_S = 0.
from dataclasses import replace
import math
OBJECTS = [replace(tool('pliers', (.16, .04, TOP+.001), 'handTool_pliers_long_nose_a01',
               'sm_handTool_pliers_long_nose_a01_01.usd',
               [('joint_handle_02_joint_00', -47., 0.)]),
               start_joints=(('joint_handle_02_joint_00', math.radians(-7.)),))]

class Trial:
    def reset(self, env=None):
        self.steps = 0
        if env is not None:
            a = env.unwrapped.scene['pliers']
            print('SOURCE_TOOL_PROPERTIES', {'bodies':a.body_names, 'joints':a.joint_names,
                  'mass_kg':m._np(a.data.default_mass).tolist(),
                  'stiffness':m._np(a.data.joint_stiffness).tolist(),
                  'damping':m._np(a.data.joint_damping).tolist()}, flush=True)
    def step(self, env):
        self.steps += 1
        if self.steps == 1 or self.steps % 10 == 0:
            s = m.state(env, 'pliers')
            print('SOURCE_TOOL_STATE', self.steps, {'position':s.pos.tolist(),
                  'velocity':s.lin_vel.tolist(), 'joints':s.joints}, flush=True)

trial = Trial()
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right':[Waypoint(3., (.20, -.25, TOP+.22), -.04695, 'idle', 0.)]}
