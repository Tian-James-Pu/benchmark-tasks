"""Native two-arm plier head engagement/elastic torque diagnostic; score0.

Each gripper grasps one original handle. Arm motion closes the physical pivot;
observed joint motion keeps the jaw bisector centered. No grasp attachments.
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
def rz(deg):return axis_angle_matrix(np.array([0.,0.,1.]),math.radians(deg))
def desired_root(q):
    angle=(OPEN_DEG-q)/2
    return SCREW_CENTER-np.array([TIP_X*math.cos(math.radians(angle))+.0001,0.,0.]),rz(angle)
PGOAL,RGOAL=desired_root(0.)
LEFT_TCP=PGOAL+RGOAL@rz(-OPEN_DEG)@np.array([-.105,.0165,.008])

@dataclass
class Angled(Waypoint):
    approach:tuple=(0.,0.,-1.)
def rotation(w):
    if not isinstance(w,Angled):return motion.grip_rotation(w.yaw)
    z=np.asarray(w.approach,dtype=float);z/=np.linalg.norm(z)
    x=np.array([0.,-1.,0.]);return np.column_stack((x,np.cross(z,x),z))
def target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        goal=np.asarray(w.pos,dtype=float);rot=rotation(w)
        if t<w.duration:
            a=ik.smoothstep(t/w.duration)
            return p+a*(goal-p),ik.slerp_matrix(r,rot,a),f+a*(w.finger-f),w.label
        t-=w.duration;p,r,f=goal,rot,w.finger
    return p,r,f,'done'
motion.ArmPlan.target=target
_OBS=None
_STOP=None
class Observe:
    def reset(self,env=None):
        global _OBS,_STOP
        _OBS=None;_STOP=None;self.n=0;self.view=None;self.head_view=None;self.clamp_history=[]
    def step(self,env):
        global _OBS
        self.n+=1;q={};f={};efforts={}
        for arm in ('left','right'):
            a=env.unwrapped.scene[arm+'_robot']
            ids=[a.find_joints('joint'+str(j))[0][0] for j in range(1,7)]
            q[arm]=m._np(a.data.joint_pos)[0,ids].copy()
            f[arm]=float(m._np(a.data.joint_pos)[0,a.find_joints('joint7')[0][0]])
            efforts[arm]=m._np(a.data.applied_torque)[0].tolist()
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
                  'head_tool_N':head_force.tolist(),'applied_effort':efforts,'arm_q':{a:x.tolist() for a,x in q.items()}},flush=True)
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
    if label not in ('guide tool','close moving handle','turn both handles'):
        return _original(self,t)
    q=_OBS['q'][arm];tcp,tcpR=ik.tcp_pose(q,self.base_pos)
    tool=_OBS['tool'];tr=tool.rotation();hinge=tool.deg('moving_half')
    bodyR=tr if arm=='right' else tr@rz(hinge-OPEN_DEG)
    if not hasattr(self,'grasp_start'):
        self.grasp_start=(t,tool.pos.copy(),tr.copy())
        self.local_tcp=bodyR.T@(tcp-tool.pos);self.local_rot=bodyR.T@tcpR
        print('HEAD_GATE_GRASP',{'arm':arm,'tcp_local':self.local_tcp.tolist(),'body_R':bodyR.tolist()},flush=True)
    slip=float(np.linalg.norm(tcp-(tool.pos+bodyR@self.local_tcp)))
    spin=float(np.linalg.norm(ik._rot_error(tcpR,bodyR@self.local_rot)))
    if slip>.008 or spin>math.radians(15):
        stop(f'{arm} handle grasp slipped {slip*1000:.2f}mm {math.degrees(spin):.2f}deg',t)
        return _STOP[arm],'stopped'
    goal,gr=desired_root(hinge if t>=24 else 0.)
    if arm=='right' and t<16:
        start,pp,rr=self.grasp_start;dt=t-start
        aligned=np.array([.30,PGOAL[1],PGOAL[2]])
        if dt<4:
            a=ik.smoothstep(dt/4);goal=pp+a*(aligned-pp);gr=ik.slerp_matrix(rr,RGOAL,a)
        else:
            a=ik.smoothstep((dt-4)/4);goal=aligned+a*(PGOAL-aligned);gr=RGOAL
    if t>=29:
        if not hasattr(self,'turn_checked'):
            if _OBS['recent_clamp_N']<1.:
                stop('no observed two-jaw head clamp before torque stroke',t)
                return _STOP[arm],'stopped'
            self.turn_checked=True
        rx=axis_angle_matrix(np.array([1.,0.,0.]),-math.radians(20)*ik.smoothstep((t-29)/5))
        goal=SCREW_CENTER+rx@(goal-SCREW_CENTER);gr=rx@gr
    if arm=='right':
        self.position_bias=np.clip(getattr(self,'position_bias',np.zeros(3))+.06*(goal-tool.pos),-.008,.008)
        self.rotation_bias=np.clip(getattr(self,'rotation_bias',np.zeros(3))+.06*ik._rot_error(gr,tr),-.12,.12)
        angle=float(np.linalg.norm(self.rotation_bias))
        bias=axis_angle_matrix(self.rotation_bias/angle,angle) if angle>1e-9 else np.eye(3)
        bodygoal=bias@gr
        destination=goal+self.position_bias+bodygoal@self.local_tcp
    else:
        commanded_hinge=OPEN_DEG*ik.smoothstep((t-24)/4)
        bodygoal=gr@rz(commanded_hinge-OPEN_DEG)
        destination=goal+bodygoal@self.local_tcp
    rot=bodygoal@self.local_rot
    candidate,pe,re=ik.solve(destination,ik.matrix_to_quat_xyzw(rot),q_init=self.q,base_pos=self.base_pos)
    if pe>.0005 or re>math.radians(.1) or np.any(candidate<ik.LOWER) or np.any(candidate>ik.UPPER):
        stop(f'{arm} IK error {pe*1000:.3f}mm {math.degrees(re):.3f}deg',t)
        return _STOP[arm],'stopped'
    self.q=candidate
    if not hasattr(self,'last_log') or t-self.last_log>.5:
        print('HEAD_GATE_CONTROL',{'t':t,'arm':arm,'phase':label,'tcp_goal':destination.tolist(),
              'tool_pos':tool.pos.tolist(),'head_torque_Nm':HEAD_K*_OBS['head'].joint('head'),
              'ik_mm':pe*1000,'hinge_deg':hinge,'slip_mm':slip*1000},flush=True)
        self.last_log=t
    return np.r_[candidate,float(np.clip(f/-.04695,0,1))],label
motion.ArmPlan.action7=action7
def demo():
    right=[Waypoint(1.5,(.230,-.0725,TOP+.100),-.014,'hover original handle'),
           Waypoint(1.5,(.230,-.0725,TOP+.015),-.014,'straddle original handle'),
           Waypoint(2.,(.230,-.0725,TOP+.015),-.001,'grasp original handle'),
           Waypoint(2.,(.230,-.0725,TOP+.130),-.001,'lift tool'),
           Waypoint(1.,(.230,-.0725,TOP+.130),-.001,'observe'),
           Waypoint(28.,(.230,-.0725,TOP+.130),-.001,'guide tool')]
    left=[Waypoint(17.,(.20,.20,TOP+.120),-.014,'wait away'),
          Waypoint(2.,tuple(LEFT_TCP+[-.060,0,.030]),-.014,'approach original moving handle'),
          Waypoint(2.,tuple(LEFT_TCP),-.014,'surround original moving handle'),
          Waypoint(3.,tuple(LEFT_TCP),-.001,'grasp original moving handle'),
          Waypoint(5.,tuple(LEFT_TCP),-.001,'close moving handle'),
          Waypoint(7.,tuple(LEFT_TCP),-.001,'turn both handles')]
    left=[Angled(w.duration,w.pos,w.finger,w.label,w.yaw,(2**-.5,0.,-2**-.5)) for w in left]
    return {'right':right,'left':left}
