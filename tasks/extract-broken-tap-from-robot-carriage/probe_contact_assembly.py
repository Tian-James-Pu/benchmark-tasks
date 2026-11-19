"""Native insertion gate; all scores zero, no attachment or thread-release event."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task_contact_assembly import *
from yam_arena import demo as motion,ik
from yam_arena.urdf import quat_xyzw_to_matrix
oldbuild=EXTRACTOR.build
EXTRACTOR=replace(EXTRACTOR,build=lambda:oldbuild().replace('"PhysicsMassAPI"','"PhysicsMassAPI", "PhysxContactReportAPI"'))
OBJECTS=[VISE,PLATE,FRAGMENT,EXTRACTOR,BUNDLE,SLEEVE]
_OBS=None
_STOP=None
class Contact:
    def reset(self,env=None):
        global _OBS,_STOP
        _OBS=None;_STOP=None;self.n=0;self.view=None
    def step(self,env):
        global _OBS
        self.n+=1;a=env.unwrapped.scene['right_robot']
        ids=[a.find_joints('joint'+str(j))[0][0] for j in range(1,7)]
        q=m._np(a.data.joint_pos)[0,ids];f=float(m._np(a.data.joint_pos)[0,a.find_joints('joint7')[0][0]])
        s=m.state(env,'extractor');_OBS={'q':q,'finger':f,'tool':s}
        if self.view is None:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            bodies=[p for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            self.filters=[str(p.GetPath()) for p in bodies if any(n in str(p.GetPath()) for n in ('/extractor','/finger_bundle/','/support_sleeve'))]
            sensors=[str(p.GetPath()) for p in bodies if '/RightArm/' in str(p.GetPath()) and p.GetName() in ('tip_left','tip_right')]
            self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[self.filters]*len(sensors),max_contact_data_count=4096)
            print('TAP_CONTACT_FILTERS',self.filters,flush=True)
        if self.n==1 or self.n%10==0:
            print('TAP_CONTACT',self.n,{'q':q.tolist(),'finger':f,'gripper_tool_N':self.view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist(),'effort':m._np(a.data.applied_torque)[0].tolist()},flush=True)
C=Contact();m.on_reset(C.reset);m.on_step(C.step)
_original=motion.ArmPlan.action7
RGOAL=quat_xyzw_to_matrix(np.asarray(Q))
def action7(self,t):
    global _STOP
    if self.base_pos[1]>0 or t<9 or _OBS is None:return _original(self,t)
    if _STOP is not None:return _STOP,'stopped'
    q=_OBS['q'];tool=_OBS['tool'];tcp,tcpR=ik.tcp_pose(q,self.base_pos)
    if not hasattr(self,'local_tcp'):
        self.local_tcp=tool.rotation().T@(tcp-tool.pos);self.local_rot=tool.rotation().T@tcpR
        self.start_tool=tool.pos.copy();self.start_R=tool.rotation().copy();self.bias=np.zeros(3)
        print('TAP_GRASP',{'local_tcp':self.local_tcp.tolist(),'local_R':self.local_rot.tolist()},flush=True)
    slip=float(np.linalg.norm(tcp-(tool.pos+tool.rotation()@self.local_tcp)))
    if slip>.010:
        _STOP=np.r_[q,float(np.clip(_OBS['finger']/-.04695,0,1))]
        print('TAP_ABORT',{'time':t,'root_relative_grasp_change_mm':slip*1000},flush=True)
        return _STOP,'stopped'
    approach=TARGET+np.array([-.025,0,0]);seated=TARGET+np.array([.0005,0,0])
    if t<15:
        a=ik.smoothstep((t-9)/6);goal=self.start_tool+a*(approach-self.start_tool);rot=ik.slerp_matrix(self.start_R,RGOAL,a)
    else:
        a=ik.smoothstep((t-15)/6);goal=approach+a*(seated-approach);rot=RGOAL
    self.bias=np.clip(self.bias+.04*(goal-tool.pos),-.008,.008)
    target=goal+self.bias+rot@self.local_tcp
    qa,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(rot@self.local_rot),q_init=self.q,base_pos=self.base_pos)
    self.q=qa
    if int(t*20)%20==0:
        print('TAP_INSERT_TARGET',{'t':t,'goal':goal.tolist(),'error_mm':((goal-tool.pos)*1000).tolist(),'ik_error_mm':pe*1000},flush=True)
    return np.r_[qa,0.], 'insert extractor'
motion.ArmPlan.action7=action7
def demo():
    return {'right':[Waypoint(2,(.260,-.235,TOP+.10),-.04695,'approach',yaw=math.pi/2),Waypoint(2,(.260,-.235,TOP+.0123),-.04695,'descend',yaw=math.pi/2),Waypoint(2,(.260,-.235,TOP+.0123),0,'grasp wrench',yaw=math.pi/2),Waypoint(3,(.260,-.235,TOP+.10),0,'lift',yaw=math.pi/2),Waypoint(15,(.260,-.235,TOP+.10),0,'insert',yaw=math.pi/2)], 'left':[Waypoint(24,(.10,.35,1.05),-.04695,'park')]}
