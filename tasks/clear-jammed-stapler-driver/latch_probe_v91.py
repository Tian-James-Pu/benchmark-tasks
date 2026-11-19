"""Near-tip screwdriver grasp via native two-arm handoff, then latch diagnostic.

Battery starts isolated. Hardware and stored load are unchanged. Real bench tool
pickup and magazine grip precede pressing. This is not a complete repair demo.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import bench_tip_v51 as prior
from bench_tip_v51 import *
from yam_arena.demo import DOWN_GRIP
from yam_arena import ik,station
from yam_arena.urdf import quat_xyzw_to_matrix
import interarm_clearance_v53
import latch_contacts_v89 as latch_contacts
from dataclasses import replace
SERVICE_P=np.array([0.18099604547023773, -0.16548392176628113, 0.7890524864196777])
SERVICE_B=np.array([[0.9971454558017814, -0.07544133167321182, -0.0030895761373743655], [0.07549065853637088, 0.9969121891490124, 0.021615911591227523], [0.0014493029639238778, -0.021787442152468448, 0.999761575020765]])
TOOL=np.array([.235,.260,TOP+.001])
OBJECTS=[replace(x,pos=tuple(SERVICE_P),start_quat=tuple(ik.matrix_to_quat_xyzw(SERVICE_B))) if x.name in ('stapler','bent_staple') else replace(x,pos=tuple(TOOL)) if x.name=='screwdriver' else x for x in OBJECTS]
HANDLE_GP=np.array([0.00045439427355637265, -0.03180902313752352, -0.002979311374956056])
HANDLE_GR=np.array([[0.0072519148614126295, 0.9870661455528557, -0.16014892243602533], [-0.9928295963285225, 0.02621683224772197, 0.11662791175464442], [0.11931806975807935, 0.15815481431887388, 0.9801786045094584]])
TOOL_REST_P=np.array([0.24449075758457184, 0.2207594871520996, 0.7724775671958923])
TOOL_REST_R=np.array([[0.9407813164506881, 0.3016296773047783, -0.15475802142973283], [0.3229518923698821, -0.9362269733115776, 0.13849510673576848], [-0.10311441529942567, -0.1802730072530414, -0.9781967184821454]])
_STAGE='tip';_VIEW=None;_LEFT_MIN=0.;_JOINTS=None;_HAVE_TOOL=False
# Actual load-bearing housing grasp measured in archived v38, rather than the
# deeper commanded pose that compliant contact prevented the fingers reaching.
GP=np.array([.12079293,.09179527,.02769443])
GR=np.array([[-.9998284285,.0057048292,.0176229454],[.0024079136,.9833484884,-.1817139298],[-.0183661437,-.1816403183,-.9831935106]])
@m.on_step
def service_observe(env):
 global _VIEW,_LEFT_MIN,_JOINTS
 if _HAVE_TOOL and base._CTX is not None:
  arm=base._CTX.right;tp,tr=base.tool_pose(arm)
  if np.linalg.norm(tp+tr@np.array([.072,0,.01321])-arm.measured_pos)>.035:raise RuntimeError('Screwdriver blade contact point lost from native grasp')
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
 global _STAGE,_HAVE_TOOL
 base._CTX=ctx
 L,R=ctx.left,ctx.right;O=-.045;LH=-.020;H=0.0
 _STAGE='settle_and_measure';yield from R.hold(3.)
 _STAGE='left_handle_pick'
 tp,tr=(TOOL_REST_P,TOOL_REST_R) if base._POSES is None else base.tool_pose(L)
 axis=tr[:,0];yaw=float(np.arctan2(axis[1],axis[0]));rr=axis_angle_matrix(np.array([0.,0.,1.]),yaw)@quat_xyzw_to_matrix(DOWN_GRIP)
 pick=tp+tr@np.array([-.030,0,.01321])+np.array([0,0,.0075])
 yield from L.move(pick+np.array([0,0,.100]),1.5,O,rot=rr)
 yield from L.move(pick,1.2,O,rot=rr)
 yield from L.grip(H,.8)
 yield from L.move(L.pos+np.array([0,0,.120]),1.5,H,rot=L.rot)
 yield from L.hold(.3)
 if base._POSES is not None:
  row=latch_contacts.LAST;forces=[v['sum_normal_magnitudes_N'] for k,v in row['terminal'].items() if k.startswith(('left_grip_tool:tip_left:','left_grip_tool:tip_right:'))]
  if len(forces)!=2 or min(forces)<1.:print('SERVICE_STOP '+json.dumps({'reason':'Left handle pickup missed','forces_N':forces}),flush=True);return
 _STAGE='handoff_position'
 handoff_R=axis_angle_matrix(np.array([0.,0.,1.]),np.radians(-75.))@quat_xyzw_to_matrix(DOWN_GRIP)
 yield from free_joint_approach(L,np.array([.250,.060,.900]),handoff_R,3.,H)
 yield from L.hold(.5)
 tp,tr=(L.pos+L.rot@HANDLE_GP,L.rot@HANDLE_GR) if base._POSES is None else base.tool_pose(L)
 yaw=float(np.arctan2(tr[1,0],tr[0,0]))+np.pi;rr=axis_angle_matrix(np.array([0.,0.,1.]),yaw)@quat_xyzw_to_matrix(DOWN_GRIP)
 pick=tp+tr@np.array([.072,0,.01321])+np.array([0,0,.0075])
 _STAGE='right_blade_acquire'
 yield from free_joint_approach(R,np.array([.225,-.260,TOP+.143]),rr,2.,-.020)
 yield from R.move(pick+np.array([0,0,.040]),2.,-.020,rot=rr)
 yield from R.move(pick,1.2,-.020,rot=rr)
 yield from R.grip(H,.8)
 yield from R.hold(.3)
 if base._POSES is None:
  base.GRASP_P=R.rot.T@(tp-R.pos);base.GRASP_R=R.rot.T@tr
 else:
  row=latch_contacts.LAST;forces=[v['sum_normal_magnitudes_N'] for k,v in row['terminal'].items() if k.startswith(('grip_tool:tip_left:','grip_tool:tip_right:'))]
  if len(forces)!=2 or min(forces)<1.:print('SERVICE_STOP '+json.dumps({'reason':'Right blade acquisition missed','forces_N':forces}),flush=True);return
 _HAVE_TOOL=True;_STAGE='left_release_tool'
 yield from L.grip(O,.6)
 yield from L.move(L.pos+np.array([0,0,.040]),1.,O,rot=L.rot)
 yield from L.home(2.)
 yield from R.hold(.5)
 if base._POSES is not None:
  tp,tr=base.tool_pose(R);point=tp+tr@np.array([.072,0,.01321]);error=float(np.linalg.norm(point-R.measured_pos))
  print('HANDOFF_RETAINED '+json.dumps({'tool_root':tp.tolist(),'tool_rotation':tr.tolist(),'contact_point_to_tcp_m':error,'scope':'Native poses after left release; subsequent contact/grasp checks still apply'}),flush=True)
  if error>.035:print('SERVICE_STOP '+json.dumps({'reason':'Tool not retained after handoff'}),flush=True);return
 _STAGE='tool_position'
 yield from R.move((.225,-.260,TOP+.143),1.2,H,rot=DOWN_GRIP)
 yield from R.hold(.2)
 if base._POSES is None:tgp,tgr=base.GRASP_P,base.GRASP_R
 else:
  tp,tr=base.tool_pose(R);tgp=R.measured_rot.T@(tp-R.measured_pos);tgr=R.measured_rot.T@tr
 P,B=(SERVICE_P,SERVICE_B) if base._POSES is None else base.frame()
 # Select among physically equivalent blade rolls, requiring reachable approach,
 # button depression and the first8mm of cover travel using the measured grasp.
 # 50deg/roll25 passed v77 measured-state reach and complete tool-mesh clearance.
 # 60deg was rejected for actual cover interference; hardware is unchanged.
 t=np.radians(50.);A=np.column_stack([[-np.cos(t),0,-np.sin(t)],[0,1,0],[np.sin(t),0,-np.cos(t)]])
 candidates=[]
 for roll in [25.]:
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
 yield from free_joint_approach(R,rp+np.array([0,0,.060]),rr,3.,H)
 yield from R.move(rp,.8,H,rot=rr)
 for i in range(17):
  target=P+B@np.array([.100-.002*(i+1),.223,0]);rp,rr=tcp_for_tool(T,target,tgp,tgr)
  yield from R.move(rp,.15,H,rot=rr)
 yield from R.hold(.4)
 _STAGE='release_feedback';travel=0.
 if base._POSES is not None:
  for attempt in range(30):
   row=latch_contacts.LAST
   forces={k:v['max_normal_magnitudes_N']+v['max_friction_magnitudes_N'] for k,v in row['peaks'].items() if k.startswith('assembly_tool:')}
   if any(v>5. for k,v in forces.items() if ':release:' in k) or any(v>.5 for k,v in forces.items() if ':release:' not in k):
    print('SERVICE_STOP '+json.dumps({'reason':'Unexpected tool load during release feedback','forces_N':forces}),flush=True);return
   if _JOINTS['release']<=-.0035:break
   nowP,nowB=base.frame();tp,tr=base.tool_pose(R);actual_tip=tp+tr@np.array([.10286,0,.01321])
   target=nowP+nowB@np.array([.066,.223+_JOINTS['cover'],0])
   error=target-actual_tip
   if np.linalg.norm(error)>.012:raise RuntimeError('Release feedback error exceeds12mm')
   correction=error*.6;length=float(np.linalg.norm(correction))
   if length>.0005:correction*=.0005/length
   travel+=float(np.linalg.norm(correction))
   if travel>.012:print('SERVICE_STOP '+json.dumps({'reason':'Release feedback travel exceeds12mm'}),flush=True);return
   print('LATCH_FEEDBACK '+json.dumps({'iteration':attempt,'error_world_m':error.tolist(),'correction_world_m':correction.tolist(),'release_m':_JOINTS['release'],'forces_N':forces}),flush=True)
   yield from R.move(R.pos+correction,.2,H,rot=R.rot)
  yield from R.hold(.2)
 if _JOINTS is not None and _JOINTS['release']>-.003:
  print('SERVICE_STOP '+json.dumps({'reason':'release button not depressed','joints':_JOINTS}),flush=True);return
 _STAGE='cover_initial_pull';lp=L.pos.copy();press_tcp=R.pos.copy();press_rot=R.rot.copy()
 for i in range(16):
  travel=.0005*(i+1);rp=press_tcp+B@np.array([0,travel,0]);rr=press_rot
  yield from ctx.both(L.move(lp+B@np.array([0,travel,0]),.1,-.008,rot=cr),R.move(rp,.1,H,rot=rr))
 yield from R.move(R.pos+B@np.array([.020,0,0]),1.,H,rot=R.rot)
 _STAGE='cover_open'
 yield from L.move(lp+B@np.array([0,.150,0]),3.,-.008,rot=cr)
 yield from L.hold(.5)
 yield from L.grip(-.022,.5)
 yield from L.move(L.pos-cr[:,2]*.080,1.5,-.022,rot=cr)
 yield from L.hold(3.)
 print('SERVICE_FEED_STOP '+json.dumps({'joints':_JOINTS}),flush=True)
