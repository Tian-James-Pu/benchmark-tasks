"""Physical sleeve preparation, two original wrench handles, short native turn."""
import pathlib,sys
from dataclasses import dataclass
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task_contact_assembly import *
from yam_arena import demo as motion,ik
from yam_arena.urdf import axis_angle_matrix
for var in ('EXTRACTOR','BUNDLE','SLEEVE','FRAGMENT'):
    obj=globals()[var];build=obj.build
    globals()[var]=replace(obj,build=lambda build=build:build().replace('"PhysicsMassAPI"','"PhysicsMassAPI", "PhysxContactReportAPI"'))
OBJECTS=[VISE,PLATE,FRAGMENT,EXTRACTOR,BUNDLE,SLEEVE]
@dataclass
class Tilt(Waypoint):
    angle:float=30.
    side:float=1.
def grip_rotation(w):
    if not isinstance(w,Tilt):return motion.grip_rotation(w.yaw)
    a=math.radians(w.angle);x=np.array([w.side,0,0]);z=np.array([0.,w.side*math.sin(a),-math.cos(a)])
    return np.column_stack((x,np.cross(z,x),z))
def target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        goal=np.asarray(w.pos);rot=grip_rotation(w)
        if t<w.duration:
            a=ik.smoothstep(t/w.duration);return p+a*(goal-p),ik.slerp_matrix(r,rot,a),f+a*(w.finger-f),w.label
        t-=w.duration;p,r,f=goal,rot,w.finger
    return p,r,f,'done'
motion.ArmPlan.target=target
_ORIGINAL=motion.ArmPlan.action7
_OBS=None;_STOP=None;_GRASPS={};_HAND=None;_BOTH=None;_BIAS=np.zeros(3)
class Contact:
    def reset(self,env=None):
        global _OBS,_STOP,_GRASPS,_HAND,_BOTH,_BIAS
        _OBS=None;_STOP=None;_GRASPS={};_HAND=None;_BOTH=None;_BIAS=np.zeros(3)
        self.n=0;self.view=None;self.hold_checked=False;self.prep_checked=False;self.insert_checked=False;self.turn_checked=False
    def step(self,env):
        global _OBS
        self.n+=1;qs={};fs={};efforts={}
        for arm in ('left','right'):
            a=env.unwrapped.scene[arm+'_robot'];ids=[a.find_joints('joint'+str(j))[0][0] for j in range(1,7)]
            qs[arm]=m._np(a.data.joint_pos)[0,ids];fs[arm]=float(m._np(a.data.joint_pos)[0,a.find_joints('joint7')[0][0]])
            efforts[arm]=m._np(a.data.applied_torque)[0].tolist()
        states={n:m.state(env,n) for n in ('extractor','finger_bundle','support_sleeve','tap_fragment')}
        asset=env.unwrapped.scene['finger_bundle'];positions=m._np(asset.data.body_link_pos_w)[0];quats=m._np(asset.data.body_link_quat_w)[0]
        tips=[]
        for k in range(3):
            i=asset.find_bodies(f'finger{k}_3')[0][0];a=2*math.pi*k/3
            q=quats[i];local=np.array([radial_center*math.cos(a),radial_center*math.sin(a),-.006])
            tips.append(positions[i]+m.quat_matrix(q)@local)
        if self.view is None:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            b=[p for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            self.filters=[str(p.GetPath()) for p in b if any(n in str(p.GetPath()) for n in ('/extractor','/finger_bundle/','/support_sleeve'))]
            self.sensors=[str(p.GetPath()) for p in b if any(n in str(p.GetPath()) for n in ('/RightArm/','/LeftArm/')) and p.GetName() in ('tip_left','tip_right')]
            v=PhysxManager.get_physics_sim_view();self.view=v.create_rigid_contact_view(self.sensors,filter_patterns=[self.filters]*len(self.sensors),max_contact_data_count=4096)
            self.tapview=v.create_rigid_contact_view(['/World/envs/env_0/tap_fragment'],filter_patterns=[self.filters],max_contact_data_count=8192)
            print('TAP_BIMANUAL_FILTERS',self.sensors,self.filters,flush=True)
        forces=self.view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy()
        idx=next(i for i,n in enumerate(self.filters) if n.endswith('/extractor'))
        grips={a:[float(np.linalg.norm(forces[i,idx])) for i,n in enumerate(self.sensors) if ('/LeftArm/' if a=='left' else '/RightArm/') in n] for a in ('left','right')}
        _OBS={'q':qs,'finger':fs,'states':states,'tips':np.asarray(tips),'grips':grips}
        if self.n==1 or self.n%10==0:
            print('TAP_BIMANUAL',self.n,{'tool_pos':states['extractor'].pos.tolist(),'tool_quat':states['extractor'].quat_xyzw.tolist(),'sleeve_pos':states['support_sleeve'].pos.tolist(),'bundle_joints':states['finger_bundle'].joints,'tips':np.asarray(tips).tolist(),'fragment_pos':states['tap_fragment'].pos.tolist(),'fragment_quat':states['tap_fragment'].quat_xyzw.tolist(),'grip_force_N':grips,'efforts':efforts},flush=True)
            normals,points,directions,seps,counts,starts=self.tapview.get_contact_data(dt=env.unwrapped.physics_dt)
            normals,points,directions,counts,starts=[x.numpy() for x in (normals,points,directions,counts,starts)]
            torque=np.zeros(3);normal_sum=0
            for j in range(len(self.filters)):
                count=int(counts[0,j]);start=int(starts[0,j]);sl=slice(start,start+count)
                if count:
                    force=normals[sl].reshape(-1,1)*directions[sl];torque+=np.cross(points[sl]-states['tap_fragment'].pos,force).sum(0);normal_sum+=float(normals[sl].sum())
            print('TAP_NORMAL_CONTACT',self.n,{'summed_normal_N':normal_sum,'normal_moment_Nm':torque.tolist(),'counts':counts.tolist()},flush=True)
C=Contact();m.on_reset(C.reset);m.on_step(C.step)
def halt(t,why):
    global _STOP
    _STOP={a:np.r_[q,float(np.clip(_OBS['finger'][a]/-.04695,0,1))] for a,q in _OBS['q'].items()}
    print('TAP_BIMANUAL_ABORT',{'time':t,'reason':why},flush=True)
def pose(arm):return ik.tcp_pose(_OBS['q'][arm],(0,.3 if arm=='left' else -.3,.75))
def capture(arm):
    tool=_OBS['states']['extractor'];tcp,r=pose(arm)
    _GRASPS[arm]=(tool.rotation().T@(tcp-tool.pos),tool.rotation().T@r)
def solve(self,arm,t,pos,rot,f):
    q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),q_init=self.q,base_pos=self.base_pos)
    if pe>.003 or re>math.radians(5):
        halt(t,f'{arm} IK residual {pe*1000:.2f}mm/{math.degrees(re):.2f}deg');return _STOP[arm],'stopped'
    self.q=q;return np.r_[q,float(np.clip(f/-.04695,0,1))],'physical tool operation'
def action7(self,t):
    global _HAND,_BOTH,_BIAS
    arm='left' if self.base_pos[1]>0 else 'right'
    if _OBS is None:return _ORIGINAL(self,t)
    if _STOP is not None:return _STOP[arm],'stopped'
    tool=_OBS['states']['extractor'];sleeve=_OBS['states']['support_sleeve'];bundle=_OBS['states']['finger_bundle']
    if not np.isfinite(tool.pos).all() or not np.isfinite(bundle.pos).all():
        halt(t,'nonfinite physical state');return _STOP[arm],'stopped'
    if t>=6 and not C.hold_checked:
        C.hold_checked=True
        print('TAP_BENCH_HOLD',_OBS['grips']['left'],flush=True)
        if min(_OBS['grips']['left'],default=0)<1:
            halt(t,'left original wrench handle not grasped before sleeve adjustment');return _STOP[arm],'stopped'
    if t>=18 and not C.prep_checked:
        C.prep_checked=True;slide=float((tool.rotation().T@(sleeve.pos-tool.pos))[2])
        print('TAP_SLEEVE_PREP',{'local_offset_m':slide},flush=True)
        if not -.021<slide<-.0155:
            halt(t,'sleeve not physically extended on bench');return _STOP[arm],'stopped'
    if (arm=='right' and t<27) or (arm=='left' and t<32):return _ORIGINAL(self,t)
    if _HAND is None:
        if min(_OBS['grips']['right'],default=0)<1:
            halt(t,'right wrench grasp not established');return _STOP[arm],'stopped'
        _HAND=(tool.pos.copy(),tool.rotation().copy());capture('right')
    hand=np.array([.30,-.08,.83]);Rhand=_HAND[1]
    if arm=='left' and t<42:
        a=math.radians(60);x=np.array([-1.,0,0]);z=np.array([0.,-math.sin(a),-math.cos(a)]);r=np.column_stack((x,np.cross(z,x),z))
        knob=tool.pos+tool.rotation()@np.array([0,.100,.060])-.002*z
        if t<35:height=.070
        elif t<38:height=.070*(1-ik.smoothstep((t-35)/3))
        else:height=0.
        if not hasattr(self,'left_start'):self.left_start=pose('left')
        p=knob+np.array([0,0,height])
        if t<35:
            a=ik.smoothstep((t-32)/3);p=self.left_start[0]+a*(p-self.left_start[0]);r=ik.slerp_matrix(self.left_start[1],r,a)
        f=-.04695*(1-ik.smoothstep((t-38)/2)) if t>=38 else -.04695
        return solve(self,arm,t,p,r,f)
    if t<42:
        a=ik.smoothstep((t-27)/5);goal=_HAND[0]+a*(hand-_HAND[0]);rot=Rhand
    else:
        if _BOTH is None:
            if any(min(_OBS['grips'][a],default=0)<1 for a in ('left','right')):
                halt(t,'two original wrench handles not both grasped');return _STOP[arm],'stopped'
            capture('right');capture('left')
            relative=tool.rotation().T@bundle.rotation()
            goalrot=_OBS['states']['tap_fragment'].rotation()@relative.T
            _BOTH=(tool.pos.copy(),tool.rotation().copy(),goalrot);_BIAS[:]=0
            print('TAP_BOTH_GRASPS',{a:(p.tolist(),r.tolist()) for a,(p,r) in _GRASPS.items()},flush=True)
        approach=TARGET+np.array([-.025,0,0]);seated=TARGET+np.array([.0005,0,0])
        if t<48:
            a=ik.smoothstep((t-42)/6);goal=_BOTH[0]+a*(approach-_BOTH[0]);rot=ik.slerp_matrix(_BOTH[1],_BOTH[2],a)
        else:
            if not C.insert_checked:
                C.insert_checked=True;err=tool.pos-approach;angle=np.linalg.norm(ik._rot_error(_BOTH[2],tool.rotation()))
                if np.linalg.norm(err[1:])>.001 or angle>math.radians(3):
                    halt(t,'approach not aligned before insertion');return _STOP[arm],'stopped'
            a=ik.smoothstep((t-48)/6);goal=approach+a*(seated-approach);rot=_BOTH[2]
            if t>=56:
                if not C.turn_checked:
                    C.turn_checked=True;tips=_OBS['tips'];depth=float(np.min(tips[:,0]-TARGET[0]))
                    sleevefront=sleeve.pos+sleeve.rotation()@np.array([0,0,.015])
                    if depth<.002 or np.linalg.norm(tool.pos-seated)>.001 or abs(sleevefront[0]-TARGET[0])>.001:
                        halt(t,'holder/fingers/sleeve not seated before turn');return _STOP[arm],'stopped'
                angle=math.radians(45)*ik.smoothstep((t-56)/4)
                rot=_BOTH[2]@axis_angle_matrix(np.array([0.,0.,1.]),angle)
                goal=seated+_BOTH[2]@np.array([0,0,PITCH*angle/(2*math.pi)])
    if arm=='right':_BIAS=np.clip(_BIAS+.04*(goal-tool.pos),-.008,.008)
    localp,localr=_GRASPS[arm];dest=goal+_BIAS+rot@localp
    if int(t*20)%20==0 and arm=='right':print('TAP_BIMANUAL_TARGET',{'t':t,'error_mm':((goal-tool.pos)*1000).tolist(),'angle_error_deg':math.degrees(np.linalg.norm(ik._rot_error(rot,tool.rotation())))},flush=True)
    return solve(self,arm,t,dest,rot@localr,0.)
motion.ArmPlan.action7=action7
def demo():
    # Hold the original wrench while sliding its sleeve: Walton's ordinary body-held adjustment.
    zleft=TOP+.0103+.001+.0015
    yleft=-.09+.002*math.sin(math.radians(60))
    return {'right':[
      Waypoint(6,(.10,-.35,1.05),-.04695,'wait for left body hold'),
      Waypoint(2,(.2975,-.19,TOP+.09),-.04695,'approach sleeve'),Waypoint(2,(.2975,-.19,TOP+.0135),-.04695,'descend sleeve'),Waypoint(2,(.2975,-.19,TOP+.0135),0,'grasp sleeve'),Waypoint(3,(.3145,-.19,TOP+.0135),0,'extend sleeve'),Waypoint(1,(.3145,-.19,TOP+.0135),-.04695,'release sleeve'),Waypoint(2,(.3145,-.19,TOP+.09),-.04695,'withdraw'),
      Tilt(2,(.260,-.235,TOP+.10),-.04695,'approach wrench'),Tilt(2,(.260,-.235,TOP+.0123),-.04695,'descend wrench'),Tilt(2,(.260,-.235,TOP+.0123),0,'grasp wrench'),Tilt(3,(.260,-.235,TOP+.10),0,'lift'),Tilt(35,(.260,-.235,TOP+.10),0,'recover')],
      'left':[
      Tilt(2,(.26,yleft,TOP+.09),-.04695,'approach original wrench knob',angle=60,side=-1),
      Tilt(2,(.26,yleft,zleft),-.04695,'descend original wrench knob',angle=60,side=-1),
      Tilt(2,(.26,yleft,zleft),0,'hold original wrench',angle=60,side=-1),
      Tilt(12,(.26,yleft,zleft),0,'hold body during sleeve adjustment',angle=60,side=-1),
      Tilt(2,(.26,yleft,zleft),-.04695,'release body',angle=60,side=-1),
      Tilt(2,(.26,yleft,TOP+.09),-.04695,'withdraw left',angle=60,side=-1),
      Waypoint(10,(.10,.35,1.05),-.04695,'park for handoff'),
      Waypoint(30,(.10,.35,1.05),-.04695,'dynamic handoff')]}
