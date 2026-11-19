"""Physical open-jaw engagement and elastic head torque gate; always score0.

Only native arm/finger position commands are issued. Observed tool pose supplies
the grasp transform; head angle/forces are diagnostics, never success or bonds.
"""
import pathlib,sys
from dataclasses import replace,dataclass
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *
from yam_arena import demo as motion,ik
from yam_arena.urdf import axis_angle_matrix

_build=TARGET.build
TARGET=replace(TARGET,build=lambda:_build().replace('"PhysicsMassAPI"','"PhysicsMassAPI", "PhysxContactReportAPI"').replace('float physics:mass =','float physxContactReport:threshold = 0\n        float physics:mass ='))
OBJECTS=[ROBOT,SCREWS,TARGET,PLIERS]
RGOAL=axis_angle_matrix(np.array([0.,0.,1.]),math.radians(5))
PGOAL=SCREW_CENTER-np.array([.03488,0.,0.])
LEFT_TCP=np.array([PGOAL[0]-.105,PGOAL[1],PGOAL[2]+.008])
LEFT_TCP[0]-=.006

@dataclass
class Angled(Waypoint):
    approach:tuple=(0.,0.,-1.)

def _rotation(w):
    if not isinstance(w,Angled):return motion.grip_rotation(w.yaw)
    z=np.asarray(w.approach,dtype=float);z/=np.linalg.norm(z)
    x=np.array([0.,-1.,0.]);return np.column_stack((x,np.cross(z,x),z))

def _target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        goal=np.asarray(w.pos,dtype=float);rot=_rotation(w)
        if t<w.duration:
            a=ik.smoothstep(t/w.duration)
            return p+a*(goal-p),ik.slerp_matrix(r,rot,a),f+a*(w.finger-f),w.label
        t-=w.duration;p,r,f=goal,rot,w.finger
    return p,r,f,'done'
motion.ArmPlan.target=_target
_OBS=None
_STOP=None

class Observe:
    def reset(self,env=None):
        global _OBS,_STOP
        _OBS=None;_STOP=None;self.n=0;self.view=None;self.head_view=None;self.clamp_history=[]
    def step(self,env):
        global _OBS
        self.n+=1;q={};f={};tcp={};rot={}
        for arm in ('left','right'):
            a=env.unwrapped.scene[arm+'_robot']
            ids=[a.find_joints('joint'+str(j))[0][0] for j in range(1,7)]
            q[arm]=m._np(a.data.joint_pos)[0,ids].copy()
            f[arm]=float(m._np(a.data.joint_pos)[0,a.find_joints('joint7')[0][0]])
        tool=m.state(env,'pliers')
        _OBS={'q':q,'finger':f,'tool':tool,'head':m.state(env,'target_screw')}
        if self.view is None:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            b=[p for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            filters=[str(p.GetPath()) for p in b if '/pliers/' in str(p.GetPath())]
            sensors=[str(p.GetPath()) for p in b if ('/RightArm/' in str(p.GetPath()) or '/LeftArm/' in str(p.GetPath())) and p.GetName() in ('tip_left','tip_right')]
            v=PhysxManager.get_physics_sim_view()
            self.view=v.create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=4096)
            self.head_view=v.create_rigid_contact_view(['/World/envs/env_0/target_screw/head'],filter_patterns=[filters],max_contact_data_count=4096)
            print('HEAD_GATE_SENSORS',sensors,filters,flush=True)
        head_force=self.head_view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy()[0]
        self.clamp_history.append(float(np.min(np.linalg.norm(head_force,axis=1))))
        self.clamp_history=self.clamp_history[-10:]
        _OBS['recent_clamp_N']=max(self.clamp_history)
        if self.n==1 or self.n%10==0:
            print('HEAD_GATE_CONTACT',self.n,{'finger_tool_N':self.view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist(),
                  'head_tool_N':self.head_view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist()},flush=True)
O=Observe();m.on_reset(O.reset);m.on_step(O.step)
_original=motion.ArmPlan.action7

def stop(why,t):
    global _STOP
    _STOP={a:np.r_[q,float(np.clip(_OBS['finger'][a]/-.04695,0,1))] for a,q in _OBS['q'].items()}
    print('HEAD_GATE_ABORT',{'t':t,'reason':why},flush=True)

def action7(self,t):
    arm='left' if self.base_pos[1]>0 else 'right'
    if _OBS is None:return _original(self,t)
    if _STOP is not None:return _STOP[arm],'stopped'
    p,r,f,label=self.target(t)
    if label not in ('guide tool','release guide','twist tool','guide withdrawn'):
        return _original(self,t)
    q=_OBS['q'][arm];tcp,tcpR=ik.tcp_pose(q,self.base_pos)
    tool=_OBS['tool'];tr=tool.rotation()
    if label=='guide tool':
        if not hasattr(self,'guide_start'):
            self.guide_start=(t,tool.pos.copy(),tr.copy())
            self.local_tcp=tr.T@(tcp-tool.pos);self.local_rot=tr.T@tcpR
            print('HEAD_GATE_GRASP',{'arm':arm,'tcp_local':self.local_tcp.tolist(),'tool_R':tr.tolist()},flush=True)
        start,pp,rr=self.guide_start;dt=t-start
        slip=float(np.linalg.norm(tcp-(tool.pos+tr@self.local_tcp)))
        spin=float(np.linalg.norm(ik._rot_error(tcpR,tr@self.local_rot)))
        if slip>.008 or spin>math.radians(15):
            stop(f'guide grasp slipped {slip*1000:.2f}mm {math.degrees(spin):.2f}deg',t)
            return _STOP[arm],'stopped'
        aligned=np.array([.30,PGOAL[1],PGOAL[2]])
        if dt<4:
            a=ik.smoothstep(dt/4);goal=pp+a*(aligned-pp);gr=ik.slerp_matrix(rr,RGOAL,a)
        else:
            a=ik.smoothstep((dt-4)/4);goal=aligned+a*(PGOAL-aligned);gr=RGOAL
        # Correct observed native tracking sag, without changing any pose,
        # constraint, material or actuator limit. Corrections stay bounded.
        self.position_bias=np.clip(getattr(self,'position_bias',np.zeros(3))+.06*(goal-tool.pos),-.008,.008)
        self.rotation_bias=np.clip(getattr(self,'rotation_bias',np.zeros(3))+.06*ik._rot_error(gr,tr),-.12,.12)
        bias_angle=float(np.linalg.norm(self.rotation_bias))
        bias_R=axis_angle_matrix(self.rotation_bias/bias_angle,bias_angle) if bias_angle>1e-9 else np.eye(3)
        rotation=bias_R@gr@self.local_rot
        target=goal+self.position_bias+bias_R@gr@self.local_tcp
    elif label in ('release guide','guide withdrawn'):
        if not hasattr(self,'release_start'):self.release_start=(t,tcp.copy(),tcpR.copy())
        start,pp,rr=self.release_start
        target=pp-np.array([.065*ik.smoothstep((t-start-1)/2),0.,0.])
        rotation=rr;f=-.04695
    else:
        if not hasattr(self,'turn_start'):
            if _OBS['recent_clamp_N']<1.:
                stop('no observed two-jaw head clamp before torque stroke',t)
                return _STOP[arm],'stopped'
            self.turn_start=(t,tool.pos.copy(),tr.copy())
            self.turn_tcp=tr.T@(tcp-tool.pos);self.turn_rot=tr.T@tcpR
        start,pp,rr=self.turn_start
        rx=axis_angle_matrix(np.array([1.,0.,0.]),-math.radians(20)*ik.smoothstep((t-start)/5))
        goal=SCREW_CENTER+rx@(pp-SCREW_CENTER);gr=rx@rr
        target=goal+gr@self.turn_tcp;rotation=gr@self.turn_rot
    candidate,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(rotation),q_init=self.q,base_pos=self.base_pos)
    if pe>.0005 or re>math.radians(.1) or np.any(candidate<ik.LOWER) or np.any(candidate>ik.UPPER):
        stop(f'IK error {pe*1000:.3f}mm {math.degrees(re):.3f}deg',t)
        return _STOP[arm],'stopped'
    self.q=candidate
    if not hasattr(self,'last_log') or t-self.last_log>.5:
        print('HEAD_GATE_CONTROL',{'t':t,'arm':arm,'phase':label,'tcp_goal':target.tolist(),'tool_pos':tool.pos.tolist(),
              'head_torque_Nm':HEAD_K*_OBS['head'].joint('head'),'ik_mm':pe*1000,
              'position_bias':getattr(self,'position_bias',np.zeros(3)).tolist()},flush=True)
        self.last_log=t
    return np.r_[candidate,float(np.clip(f/-.04695,0,1))],label
motion.ArmPlan.action7=action7

def demo():
    right=[Waypoint(1.5,(.265,-.059,TOP+.100),-.010,'hover single handle'),
           Waypoint(1.5,(.265,-.059,TOP+.012),-.010,'straddle handle near pivot'),
           Waypoint(2.,(.265,-.059,TOP+.012),-.001,'grasp original handle'),
           Waypoint(2.,(.265,-.059,TOP+.130),-.001,'lift tool'),
           Waypoint(1.,(.265,-.059,TOP+.130),-.001,'observe'),
           Waypoint(14.,(.265,-.059,TOP+.130),-.001,'guide tool'),
           Waypoint(3.,(.265,-.059,TOP+.130),-.04695,'release guide'),
           Waypoint(7.,(.265,-.059,TOP+.130),-.04695,'guide withdrawn')]
    left=[Waypoint(16.,(.22,.20,TOP+.120),-.04695,'wait away from tool'),
          Waypoint(2.,tuple(LEFT_TCP+[-.060,0,.015]),-.04695,'approach along handle axis'),
          Waypoint(2.,tuple(LEFT_TCP),-.04695,'surround distal handles'),
          Waypoint(2.,tuple(LEFT_TCP),-.010,'squeeze on screw head'),
          Waypoint(3.,tuple(LEFT_TCP),-.010,'hold while guide releases'),
          Waypoint(7.,tuple(LEFT_TCP),-.010,'twist tool')]
    left=[Angled(w.duration,w.pos,w.finger,w.label,w.yaw,(2**-.5,0.,-2**-.5)) for w in left]
    return {'right':right,'left':left}
