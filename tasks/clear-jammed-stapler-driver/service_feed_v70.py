"""Component composition: isolated-tool tip, early tool pickup, reposition, and feed opening.

Hardware and stored mechanical load are unchanged. Native movement and contact
remain authoritative; no complete repair or battery-removal claim from this file.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import bench_tip_v51 as prior
from bench_tip_v51 import *
from yam_arena.demo import DOWN_GRIP
from yam_arena import ik,station
from yam_arena.urdf import quat_xyzw_to_matrix
import interarm_clearance_v53
_STAGE='tip';_VIEW=None;_LEFT_MIN=0.;_JOINTS=None
# Actual load-bearing housing grasp measured in archived v38, rather than the
# deeper commanded pose that compliant contact prevented the fingers reaching.
GP=np.array([.12079293,.09179527,.02769443])
GR=np.array([[-.9998284285,.0057048292,.0176229454],[.0024079136,.9833484884,-.1817139298],[-.0183661437,-.1816403183,-.9831935106]])
@m.on_step
def service_observe(env):
 global _VIEW,_LEFT_MIN,_JOINTS
 a=m.state(env,'stapler');_JOINTS={n:a.joint(n) for n in ['driver','cover','follower','release']}
 if _VIEW is None:
  from isaaclab_physx.physics import PhysxManager
  from pxr import UsdPhysics
  import isaaclab.sim as sim_utils
  paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  sensors=[next(p for p in paths if '/'+side+'Arm/' in p and p.endswith('/'+n)) for side in ['Left','Right'] for n in ['tip_left','tip_right','gripper','link5']]
  targets=[next(p for p in paths if '/stapler/' in p and p.endswith('/'+n)) for n in ['frame','cover','release']]
  _VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[targets]*8,max_contact_data_count=4096)
 forces=_VIEW.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().copy();_LEFT_MIN=min(float(np.linalg.norm(v)) for v in forces[:2,0])
 if _STAGE!='tip' and np.linalg.norm(forces[4:],axis=-1).sum()>5.:raise RuntimeError('Unexpected right hand contact with stapler '+json.dumps({'step':base._k,'stage':_STAGE,'right_vectors_N':forces[4:].tolist()}))
 if base._CTX is not None and base._k%5==0:
  hands={}
  for name,arm in [('left',base._CTX.left),('right',base._CTX.right)]:
   robot=env.unwrapped.scene[name+'_robot'];q=robot.data.joint_pos[0,:6].detach().cpu().numpy().astype(float);p,R=ik.tcp_pose(q,station.BASES[name]);hands[name]={'measured_tcp':p.tolist(),'measured_R':R.tolist(),'command_tcp':arm.pos.tolist(),'command_R':arm.rot.tolist(),'body_in_tcp':(R.T@(a.pos-p)).tolist(),'body_R_in_tcp':(R.T@a.rotation()).tolist(),'finger_q':robot.data.joint_pos[0,6:].detach().cpu().numpy().tolist()}
  print('SERVICE_HANDS '+json.dumps({'step':base._k,'stage':_STAGE,'hands':hands}),flush=True)
 if base._k%5==0:print('SERVICE_FEED '+json.dumps({'step':base._k,'stage':_STAGE,'joints':_JOINTS,'frame_pos':a.pos.tolist(),'frame_R':a.rotation().tolist(),'left_min_frame_N':_LEFT_MIN,'normal_force_vectors_N':forces.tolist()}),flush=True)
def tcp_for_tool(toolR,tip_world,gp,gr):
 R=toolR@gr.T;return tip_world-toolR@np.array([.10286,0,.01321])-R@gp,R

def free_joint_approach(arm,p,R,duration=3.,finger=-.008):
 # Ordinary smooth joint-space transit, as used by the controller's home move.
 # Native contact guards still apply; no object pose, force or joint is changed.
 q0=arm.q.copy();q1,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),q0,base_pos=arm.base_pos,iters=400)
 if pe>.0019 or re>np.radians(1.5):raise RuntimeError('Free approach endpoint unreachable')
 print('SERVICE_FREE_APPROACH '+json.dumps({'position_error_m':float(pe),'orientation_error_deg':float(np.degrees(re)),'scope':'Free staging only; button contact candidates retain1mm/1deg limits'}),flush=True)
 steps=max(1,round(duration/arm.dt));arm.finger=finger
 for i in range(1,steps+1):
  arm.q=q0+ik.smoothstep(i/steps)*(q1-q0)
  arm.pos,arm.rot=ik.tcp_pose(arm.q,arm.base_pos)
  yield

def demo_script(ctx):
 global _STAGE
 yield from prior.demo_script(ctx)
 L,R=ctx.left,ctx.right;O=-.045;LH=-.020;H=-.008
 _STAGE='tool_pick'
 tp,tr=base.tool_pose(R);axis=tr[:,0];yaw=float(np.arctan2(axis[1],axis[0]));rr=axis_angle_matrix(np.array([0.,0.,1.]),yaw)@quat_xyzw_to_matrix(DOWN_GRIP)
 pick=tp+tr@np.array([-.030,0,.01321])+np.array([0,0,.0075])
 if base._POSES is None:pick=np.array([TOOL[0]-.010,TOOL[1],TOP+.023]);rr=quat_xyzw_to_matrix(DOWN_GRIP)
 yield from R.move(pick+np.array([0,0,.100]),1.5,O,rot=rr)
 yield from R.move(pick,1.2,O,rot=rr)
 yield from R.grip(H,.7)
 yield from R.move(R.pos+np.array([0,0,.120]),1.5,H,rot=R.rot)
 yield from R.hold(.3)
 if base._POSES is None:tgp,tgr=base.GRASP_P,base.GRASP_R
 else:
  tp,tr=base.tool_pose(R);tgp=R.measured_rot.T@(tp-R.measured_pos);tgr=R.measured_rot.T@tr
 _STAGE='clear_right'
 yield from R.move((.35,-.25,TOP+.20),1.5,H,rot=DOWN_GRIP)
 P,B=(np.array([.05,.0675,TOP+.0405]),np.eye(3)) if base._POSES is None else base.frame()
 tilt=float(np.degrees(np.arccos(np.clip(B[2,2],-1,1))))
 if tilt>12.:print('SERVICE_STOP '+json.dumps({'reason':'case did not settle on side','tilt_deg':tilt}),flush=True);return
 _STAGE='housing_grasp';lp=P+B@GP;lr=B@GR
 yield from L.move(lp+np.array([0,0,.080]),1.5,O,rot=lr)
 yield from L.move(lp,1.5,O,rot=lr)
 yield from L.grip(LH,.8)
 yield from L.hold(.5)
 if _VIEW is not None and _LEFT_MIN<10.:print('SERVICE_STOP '+json.dumps({'reason':'housing grasp missed','min_tip_N':_LEFT_MIN}),flush=True);return
 P,B=(P,B) if base._POSES is None else base.frame();ap=L.pos if base._POSES is None else L.measured_pos;ar=L.rot if base._POSES is None else L.measured_rot;gp=B.T@(ap-P);gr=B.T@ar
 _STAGE='reposition'
 yield from L.move(L.pos+np.array([0,0,.040]),1.5,LH,rot=L.rot)
 end=np.array([.18,-.20,TOP+.0807]);yield from L.move(end+gp,3.,LH,rot=gr)
 yield from L.move(end+gp-np.array([0,0,.040]),1.5,LH,rot=gr)
 yield from L.grip(O,.6)
 yield from L.move(L.pos+np.array([0,.100,.040]),1.,O,rot=L.rot)
 yield from L.hold(1.)
 _STAGE='tool_position'
 yield from R.move((.225,-.260,TOP+.143),1.2,H,rot=DOWN_GRIP)
 yield from R.hold(.2)
 if base._POSES is None:tgp,tgr=base.GRASP_P,base.GRASP_R
 else:
  tp,tr=base.tool_pose(R);tgp=R.measured_rot.T@(tp-R.measured_pos);tgr=R.measured_rot.T@tr
 P,B=(np.array([.18,-.20,TOP+.0407]),np.eye(3)) if base._POSES is None else base.frame()
 # Select among physically equivalent blade rolls, requiring reachable approach,
 # button depression and the first8mm of cover travel using the measured grasp.
 # 50deg/roll30 passed measured-state reach and complete tool-mesh clearance.
 # 60deg was rejected for actual cover interference; hardware is unchanged.
 t=np.radians(50.);A=np.column_stack([[-np.cos(t),0,-np.sin(t)],[0,1,0],[np.sin(t),0,-np.cos(t)]])
 candidates=[]
 for roll in [30.]:
  T=B@A@axis_angle_matrix(np.array([1.,0.,0.]),np.radians(roll));errs=[];q=R.q.copy()
  for pt in [[.100,.223,0],[.066,.223,0],[.066,.231,0]]:
   cp,cr=tcp_for_tool(T,P+B@pt,tgp,tgr);q,pe,re=ik.solve(cp,ik.matrix_to_quat_xyzw(cr),q,base_pos=station.BASES['right'],iters=200);errs.append((pe,re))
  if max(e[0] for e in errs)<.001 and max(e[1] for e in errs)<np.radians(1.):candidates.append((float(np.linalg.norm(q-R.q)),float(roll),T))
 if not candidates:print('SERVICE_STOP '+json.dumps({'reason':'No reachable50deg release tool roll with actual grasp'}),flush=True);return
 _,roll,T=min(candidates,key=lambda a:a[0]);print('SERVICE_TOOL_GRASP '+json.dumps({'tcp_local_position':tgp.tolist(),'tcp_local_rotation':tgr.tolist(),'release_tilt_deg':50.,'release_roll_deg':roll}),flush=True)
 _STAGE='cover_grasp';t=np.radians(30.);cr=B@np.column_stack([[0,-np.sin(t),np.cos(t)],[-1,0,0],[0,-np.cos(t),-np.sin(t)]]);cp=P+B@np.array([.041,.2215,0])-cr[:,2]*.013
 yield from L.move(cp-cr[:,2]*.070,1.5,-.022,rot=cr)
 yield from L.move(cp,1.5,-.022,rot=cr)
 yield from L.grip(-.008,.7)
 _STAGE='release_button';rp,rr=tcp_for_tool(T,P+B@np.array([.100,.223,0]),tgp,tgr)
 yield from free_joint_approach(R,rp+np.array([0,0,.0175]),rr,3.,H)
 yield from R.move(rp,.8,H,rot=rr)
 for i in range(17):
  target=P+B@np.array([.100-.002*(i+1),.223,0]);rp,rr=tcp_for_tool(T,target,tgp,tgr)
  yield from R.move(rp,.15,H,rot=rr)
 yield from R.hold(.4)
 if _JOINTS is not None and _JOINTS['release']>-.003:
  print('SERVICE_STOP '+json.dumps({'reason':'release button not depressed','joints':_JOINTS}),flush=True);return
 _STAGE='cover_initial_pull';lp=L.pos.copy()
 for i in range(16):
  travel=.0005*(i+1);rp,rr=tcp_for_tool(T,P+B@np.array([.066,.223+travel,0]),tgp,tgr)
  yield from ctx.both(L.move(lp+B@np.array([0,travel,0]),.1,-.008,rot=cr),R.move(rp,.1,H,rot=rr))
 yield from R.move(R.pos-T[:,0]*.020+B@np.array([0,0,.002]),1.,H,rot=R.rot)
 _STAGE='cover_open'
 yield from L.move(lp+B@np.array([0,.150,0]),3.,-.008,rot=cr)
 yield from L.hold(.5)
 yield from L.grip(-.022,.5)
 yield from L.move(L.pos-cr[:,2]*.080,1.5,-.022,rot=cr)
 yield from L.hold(3.)
 print('SERVICE_FEED_STOP '+json.dumps({'joints':_JOINTS}),flush=True)
