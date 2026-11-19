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
        self.contacts = None
        if env is not None:
            a = env.unwrapped.scene['pliers']
            print('SOURCE_TOOL_PROPERTIES', {'bodies':a.body_names, 'joints':a.joint_names,
                  'mass_kg':m._np(a.data.default_mass).tolist(),
                  'stiffness':m._np(a.data.joint_stiffness).tolist(),
                  'damping':m._np(a.data.joint_damping).tolist()}, flush=True)
    def step(self, env):
        self.steps += 1
        if self.contacts is None:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            bodies=[p for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            sensors=[str(p.GetPath()) for p in bodies if '/pliers/' in str(p.GetPath())]
            filters=[str(p.GetPath()) for p in bodies if '/LeftArm/' in str(p.GetPath()) and p.GetName() in ('tip_left','tip_right')]
            print('TOOL_CONTACT_PATHS',sensors,filters,flush=True)
            self.contacts=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=4096)
        if self.steps == 1 or self.steps % 10 == 0:
            s = m.state(env, 'pliers')
            robot=env.unwrapped.scene['left_robot']
            finger=float(m._np(robot.data.joint_pos)[0,robot.find_joints('joint7')[0][0]])
            print('TOOL_GRIP_FORCES',self.steps,{'finger_position':finger,'forces':self.contacts.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist()},flush=True)
            a = env.unwrapped.scene['pliers']
            poses=m._np(a.data.body_link_pose_w)[0]
            centers=[]
            for name, y in [('handle_01_obj_00',.001726939),('handle_02_obj_00',-.001037284)]:
                pose=poses[a.body_names.index(name)]
                local=np.array([.09-.02872828388,y-.000344729051,.00781585-.007903505117])
                centers.append(pose[:3]+m.quat_matrix(pose[3:])@local)
            print('SOURCE_TOOL_STATE', self.steps, {'jaw_center_distance_mm':float(np.linalg.norm(centers[0]-centers[1]))*1000,'jaw_centers':np.asarray(centers).tolist(),'position':s.pos.tolist(),
                  'velocity':s.lin_vel.tolist(), 'joints':s.joints}, flush=True)

trial = Trial()
m.on_reset(trial.reset)
m.on_step(trial.step)
def score(objects): return 0.
def success(objects): return False
from dataclasses import dataclass
from yam_arena import demo as _orientation_demo, ik
@dataclass
class Oriented(Waypoint):
    approach: tuple = (0, 0, -1)
    closing: tuple = (0, 1, 0)

def rotation(w):
    if not isinstance(w, Oriented):
        return _orientation_demo.grip_rotation(w.yaw)
    z=np.asarray(w.approach,dtype=float); z/=np.linalg.norm(z)
    x=np.asarray(w.closing,dtype=float); x-=x.dot(z)*z; x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))

def target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        g=np.asarray(w.pos,dtype=float); q=rotation(w)
        if t<w.duration:
            a=ik.smoothstep(t/w.duration)
            return p+a*(g-p),ik.slerp_matrix(r,q,a),f+a*(w.finger-f),w.label
        t-=w.duration; p,r,f=g,q,w.finger
    return p,r,f,'done'

_orientation_demo.ArmPlan.target=target

def demo():
    def w(t,z,f,label,a=(0,0,-1)):
        return Oriented(t,(.27,.080345,TOP+z),f,label,0.,a,(0,-1,0))
    return {'left':[
        w(1.5,.12,-.04695,'hover handles'),
        w(1.5,.014,-.04695,'approach handles'),
        w(1.,.014,-.035,'squeeze handles'),
        w(1.5,.16,-.035,'lift and turn jaws down',(-1,0,0)),
        w(3.,.16,-.035,'hold jaws down',(-1,0,0)),
        w(2.,.16,-.038,'relax squeeze',(-1,0,0)),
        w(2.,.16,-.035,'open jaws again',(-1,0,0)),
        w(2.,.014,-.035,'lower and turn flat'),
        w(1.,.014,-.04695,'release handles'),
        w(2.,.12,-.04695,'withdraw')]}
