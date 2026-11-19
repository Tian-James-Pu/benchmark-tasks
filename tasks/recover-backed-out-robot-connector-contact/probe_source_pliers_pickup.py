"""Pick up the source spring-loaded circlip pliers and vary handle squeeze; score disabled."""
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from articulated_tool import tool
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = 'Inspect the tool articulation.'
SUCCESS_HOLD_S = 0.
OBJECTS = [tool('pliers', (.28, .08, TOP+.001), 'handTool_pliers_circlip_a01',
               'sm_handTool_pliers_circlip_a01_01.usd',
               [('joint_handle_01_joint_00', 0., 10.),
                ('joint_handle_02_joint_00', -10., 0.)],
               root_offset=(.02517325431, .0006719973171, .00780573301))]

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
            a = env.unwrapped.scene['pliers']
            poses=m._np(a.data.body_link_pose_w)[0]
            centers=[]
            for name, y in [('handle_01_obj_00',.001726939),('handle_02_obj_00',-.001037284)]:
                pose=poses[a.body_names.index(name)]
                local=np.array([.09-.03972828388,y-.000344729051,.00781585-.007903505117])
                centers.append(pose[:3]+m.quat_matrix(pose[3:])@local)
            print('SOURCE_TOOL_STATE', self.steps, {'jaw_center_distance_mm':float(np.linalg.norm(centers[0]-centers[1]))*1000,'jaw_centers':np.asarray(centers).tolist(),'position':s.pos.tolist(),
                  'velocity':s.lin_vel.tolist(), 'joints':s.joints}, flush=True)

trial = Trial()
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    def w(t,z,f,label):
        return Waypoint(t,(.27,.080345,TOP+z),f,label,0.)
    return {'left':[
        w(1.5,.12,-.04695,'hover handles'),
        w(1.5,.014,-.04695,'approach handles'),
        w(1.,.014,-.039,'squeeze handles'),
        w(2.,.16,-.039,'lift pliers'),
        w(2.,.16,-.039,'hold open jaws'),
        w(2.,.16,-.044,'relax squeeze'),
        w(2.,.16,-.039,'open jaws again'),
        w(2.,.014,-.039,'lower pliers'),
        w(1.,.014,-.04695,'release handles'),
        w(2.,.12,-.04695,'withdraw')]}
