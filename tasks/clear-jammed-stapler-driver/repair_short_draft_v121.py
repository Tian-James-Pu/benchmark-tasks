"""Inactive end-face-grip repair composition with bench-supported tool regrasp, from an already isolated service state.

Native feed-opening, screwdriver driver return and open-front wire sweep.
No full task score; battery isolation must be composed separately and validated.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import latch_probe_v118 as feed
from latch_probe_v118 import *
import contact_full_case_v49 as contacts
_STAGE='feed';_WIRE=None;_MAX_TOOL_NORMAL=0.
@m.on_step
def repair_observe(env):
 global _WIRE,_MAX_TOOL_NORMAL
 a=m.state(env,'stapler');w=m.state(env,'bent_staple');_WIRE=(w.pos.copy(),w.rotation().copy())
 if contacts._view is not None:
  values=contacts._view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().copy();_MAX_TOOL_NORMAL=float(np.linalg.norm(values[:,1,:],axis=-1).max())
 if _STAGE.startswith('wire_') and _MAX_TOOL_NORMAL>10.:raise RuntimeError('Open-front tool sweep exceeded10N sampled normal-force bound '+str(_MAX_TOOL_NORMAL))
 if base._k%5==0:print('REPAIR_POSE '+json.dumps({'step':base._k,'stage':_STAGE,'max_sampled_tool_normal_N':_MAX_TOOL_NORMAL,'body_pos':a.pos.tolist(),'body_R':a.rotation().tolist(),'driver_m':a.joint('driver'),'cover_m':a.joint('cover'),'wire_pos':w.pos.tolist(),'wire_R':w.rotation().tolist(),'wire_in_body':(a.rotation().T@(w.pos-a.pos)).tolist()}),flush=True)
def case():return (feed.SERVICE_P,feed.SERVICE_B) if base._POSES is None else base.frame()
def grasp(R):
 if base._POSES is None:return base.GRASP_P,base.GRASP_R
 p,r=base.tool_pose(R);return R.measured_rot.T@(p-R.measured_pos),R.measured_rot.T@r

def move_tip(R,local,Tlocal,seconds,H=0.0):
 P,B=case();gp,gr=grasp(R);p,r=feed.tcp_for_tool(B@Tlocal,P+B@np.array(local),gp,gr)
 yield from R.move(p,seconds,H,rot=r)
def feasible(R,Tlocal,points):
 P,B=case();gp,gr=grasp(R);q=R.q.copy();errors=[]
 for local in points:
  p,r=feed.tcp_for_tool(B@Tlocal,P+B@np.array(local),gp,gr);q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),q,base_pos=station.BASES['right'],iters=240);errors.append((pe,re))
 return max(x[0] for x in errors),max(x[1] for x in errors)
def demo_script(ctx):
 global _STAGE
 yield from feed.demo_script(ctx)
 if feed._JOINTS is not None and feed._JOINTS['cover']<.140:print('REPAIR_STOP '+json.dumps({'reason':'Magazine did not open','joints':feed._JOINTS}),flush=True);return
 L,R=ctx.left,ctx.right;H=0.0;LH=-.020;O=-.045;_STAGE='reset_approach'
 # Park the original screwdriver on the bench and regrasp its handle
 # transversely for horizontal driver entry. No object state is written.
 yield from R.move((.240,-.150,.940),1.5,H,rot=R.rot)
 yield from R.move(R.pos,1.,H,rot=DOWN_GRIP)
 yield from R.move((.245,-.340,TOP+.145),1.5,H,rot=DOWN_GRIP)
 _STAGE='tool_bench_place'
 gp,gr=grasp(R);tr=R.rot@gr
 bbox=np.array([[x,y,z] for x in [-.04516935,.10285867] for y in [-.01361795,.01361942] for z in [.00020916,.02620728]])
 root=np.array([.245,-.340,TOP+.003-float((bbox@tr.T)[:,2].min())])
 put=root-R.rot@gp
 yield from R.move(put,1.5,H,rot=R.rot)
 feed._HAVE_TOOL=False
 yield from R.grip(O,.6)
 yield from R.hold(.8)
 yield from R.move(R.pos+np.array([0,0,.100]),1.,O,rot=R.rot)
 yield from R.hold(.8)
 _STAGE='tool_transverse_regrasp'
 if base._POSES is None:
  yaw=float(np.arctan2(tr[1,0],tr[0,0]));oldyaw=float(np.arctan2(feed.TOOL_REST_R[1,0],feed.TOOL_REST_R[0,0]));tr=axis_angle_matrix(np.array([0.,0.,1.]),yaw-oldyaw)@feed.TOOL_REST_R
  tp=np.array([root[0],root[1],TOP+feed.TOOL_REST_P[2]-TOP])
 else:tp,tr=base.tool_pose(R)
 yaw=float(np.arctan2(tr[1,0],tr[0,0]));rr=axis_angle_matrix(np.array([0.,0.,1.]),yaw)@quat_xyzw_to_matrix(DOWN_GRIP)
 pick=tp+tr@np.array([0.,0,.01321])+np.array([0,0,.0075])
 if base._POSES is None:base.GRASP_P=rr.T@(tp-pick);base.GRASP_R=rr.T@tr
 yield from R.move(pick+np.array([0,0,.100]),1.2,O,rot=rr)
 yield from R.move(pick,1.2,O,rot=rr)
 yield from R.grip(H,.7)
 yield from R.hold(.8)
 if base._POSES is not None:
  row=feed.latch_contacts.LAST;forces=[v['sum_normal_magnitudes_N'] for k,v in row['terminal'].items() if k.startswith(('grip_tool:tip_left:','grip_tool:tip_right:'))]
  print('REPAIR_TRANSVERSE_PRELIFT '+json.dumps({'tip_normal_N':forces}),flush=True)
  if len(forces)!=2 or min(forces)<1.:print('REPAIR_STOP '+json.dumps({'reason':'No bilateral handle contact after regrasp'}),flush=True);return
 yield from R.move(R.pos+np.array([0,0,.120]),1.5,H,rot=R.rot)
 yield from R.hold(.3)
 feed._HAVE_TOOL=True
 yield from R.move((.245,-.260,TOP+.120),1.5,H,rot=DOWN_GRIP)
 for p in [(.245,-.200,TOP+.170),(.020,-.200,TOP+.170),(.020,-.200,TOP+.055)]:yield from R.move(p,1.2,H,rot=DOWN_GRIP)
 P,B=case();lr=B@feed.GR;lp=P+B@feed.GP
 yield from L.move(lp+np.array([0,0,.060]),1.5,O,rot=lr)
 yield from L.move(lp,1.2,O,rot=lr)
 yield from L.grip(LH,.7)
 yield from L.hold(.5)
 if feed._VIEW is not None and feed._LEFT_MIN<10.:print('REPAIR_STOP '+json.dumps({'reason':'Case stabilization grasp missed'}),flush=True);return
 flat=[];entry_points=[[-.012,-.0008,0],[.040,-.0008,0]]
 for T in [np.eye(3),np.diag([1.,-1.,-1.])]:
  pe,re=feasible(R,T,entry_points)
  if pe<.0019 and re<np.radians(1.5):flat.append((pe,T))
 if not flat:print('REPAIR_STOP '+json.dumps({'reason':'No reachable flat-blade entry with actual grasp'}),flush=True);return
 T=min(flat,key=lambda x:x[0])[1]
 P,B=case();gp,gr=grasp(R)
 yield from R.move(R.pos,1.,H,rot=B@T@gr.T)
 yield from R.hold(.2)
 for _ in range(3):yield from move_tip(R,[-.012,-.0008,0],T,.7);yield from R.hold(.2)
 for _ in range(4):yield from move_tip(R,[-.004,-.0008,0],T,.7);yield from R.hold(.3)
 _STAGE='driver_entry';P,B=case();origin=P.copy();entered=False;rot=R.rot.copy()
 for j in range(30):
  P,B=case();local=B.T@(base.tip(R)-P)
  if local[0]>=.004:entered=True;break
  side=np.array([0.,-.0008-local[1],-local[2]]);correction=B@np.clip(side*.7,-.0008,.0008)
  yield from R.move(R.pos+B@np.array([.0008,0,0])+correction,.3,H,rot=rot)
  yield from R.hold(.15)
  if np.linalg.norm(case()[0]-origin)>.020:raise RuntimeError('Case moved20mm during driver entry')
 P,B=case();entered=entered or (B.T@(base.tip(R)-P))[0]>=.004
 print('REPAIR_ENTRY '+json.dumps({'entered':entered,'iterations':j+1,'driver_m':base._DRIVER}),flush=True)
 if not entered and base._POSES is not None:return
 _STAGE='driver_return';P,B=case();start=R.pos.copy()
 for i in range(110):
  yield from R.move(start+B@np.array([.0005*(i+1),0,0]),.05,H,rot=rot)
  if base._DRIVER is not None and base._DRIVER>=.0395:break
  if np.linalg.norm(case()[0]-P)>.020:raise RuntimeError('Case moved20mm during driver return')
 print('REPAIR_RETURN '+json.dumps({'driver_m':base._DRIVER,'iterations':i+1,'command_travel_bound_m':.055}),flush=True)
 if base._DRIVER is not None and base._DRIVER<.0395:print('REPAIR_STOP '+json.dumps({'reason':'55mm commanded reset travel bound without full driver return','driver_m':base._DRIVER}),flush=True);return
 yield from R.hold(1.)
 yield from move_tip(R,[-.015,-.0008,0],T,2.5)
 yield from R.hold(.5)
 if base._DRIVER is not None and base._DRIVER<.039:print('REPAIR_STOP '+json.dumps({'reason':'Driver not retained after tool withdrawal'}),flush=True);return
 _STAGE='wire_approach';choices=[]
 for angle in [39.,40.,41.]:
  for roll in [0.,180.]:
   W=axis_angle_matrix(np.array([0.,0.,1.]),np.radians(-angle))@axis_angle_matrix(np.array([1.,0.,0.]),np.radians(roll));pre=np.array([.0445,-.003,0])-W[:,0]*.035
   path=[[-.030,.020,0],pre.tolist(),[.0445,-.003,0],[.0445,.012,0],[.040,.016,0],[.015,.016,0]];pe,re=feasible(R,W,path)
   if pe<.0019 and re<np.radians(1.5):choices.append((pe,angle,roll,W,path))
 if not choices:print('REPAIR_STOP '+json.dumps({'reason':'Open-front wire sweep unreachable with actual grasp'}),flush=True);return
 _,angle,roll,W,path=min(choices,key=lambda x:x[0]);print('REPAIR_SWEEP_PLAN '+json.dumps({'angle_deg':angle,'roll_deg':roll,'path_m':path,'scope':'Native contact outcome required; no pose or force applied to wire'}),flush=True)
 # Front clearance waypoint precedes rotation; final diagonal enters through
 # the actual open cover face and clears the fixed front land.
 yield from move_tip(R,[-.030,.020,0],T,1.5)
 yield from move_tip(R,[-.030,.020,0],W,1.)
 yield from move_tip(R,path[1],W,1.5)
 wire_origin=case()[0].copy()
 for stage,start,end in [('wire_entry',path[1],path[2]),('wire_feed_push',path[2],path[3]),('wire_outward_sweep',path[3],path[4]),('wire_clear',path[4],path[5])]:
  _STAGE=stage;start=np.array(start);end=np.array(end);n=int(np.ceil(np.linalg.norm(end-start)/.001))
  for i in range(n):
   yield from move_tip(R,start+(end-start)*(i+1)/n,W,.15)
   yield from R.hold(.05)
   if np.linalg.norm(case()[0]-wire_origin)>.020:raise RuntimeError('Case moved20mm during wire sweep')
 yield from move_tip(R,[-.030,.025,0],W,1.5)
 yield from R.hold(3.)
 _STAGE='release_case';yield from L.grip(O,.5)
 yield from L.move(L.pos+np.array([0,.100,.040]),1.2,O,rot=L.rot)
 yield from L.hold(3.)
