"""Robot isolation and bench-supported service roll with unchanged hardware."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import isolation_park_v45 as prior
from isolation_park_v45 import *
from yam_arena.urdf import axis_angle_matrix
_STAGE='isolation';_VIEW=None;_NORMAL=0.;_LEFT_LOAD=0.
@m.on_step
def turn_observe(env):
 global _VIEW,_NORMAL,_LEFT_LOAD
 a=m.state(env,'stapler')
 if _VIEW is None:
  from isaaclab_physx.physics import PhysxManager
  from pxr import UsdPhysics
  import isaaclab.sim as sim_utils
  paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  sensors=[next(p for p in paths if '/'+arm+'Arm/' in p and p.endswith('/'+n)) for arm in ['Left','Right'] for n in ['tip_left','tip_right']]
  filt=[next(p for p in paths if '/stapler/' in p and p.endswith('/frame'))]
  _VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filt]*4,max_contact_data_count=2048)
 forces=_VIEW.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().copy()
 _NORMAL=sum(max(0.,float(v@a.rotation()[:,2])) for v in forces[2:,0]);_LEFT_LOAD=min(float(np.linalg.norm(v)) for v in forces[:2,0])
 if base._k%5==0:
  hands={}
  for name in ['left','right']:
   rob=env.unwrapped.scene[name+'_robot'];q=rob.data.joint_pos[0,:6].detach().cpu().numpy().astype(float);p,R=base.ik.tcp_pose(q,base.station.BASES[name]);hands[name]={'tcp':p.tolist(),'rotation':R.tolist(),'in_body':(a.rotation().T@(p-a.pos)).tolist()}
  print('TURN_POSE '+json.dumps({'step':base._k,'stage':_STAGE,'body_pos':a.pos.tolist(),'body_R':a.rotation().tolist(),'right_normal_N':_NORMAL,'left_min_tip_N':_LEFT_LOAD,'tip_forces_N':forces.tolist(),'hands':hands}),flush=True)
def demo_script(ctx):
 global _STAGE
 yield from prior.demo_script(ctx)
 L,R=ctx.left,ctx.right;O=-.045;LH=-.020;RH=-.010
 _STAGE='upper_grasp';p,B=base.frame()
 down=axis_angle_matrix(np.array([0.,1.,0.]),np.radians(-30))@np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])@np.diag([-1.,-1.,1.])
 lp=p+B@np.array([.115,.215,0.]);above=-down[:,2]*.020
 yield from L.move(lp+above+np.array([0,.080,0]),2.,O,rot=down)
 yield from L.move(lp+above,1.,O,rot=down)
 yield from L.move(lp,1.,O,rot=down)
 yield from L.grip(LH,1.)
 yield from L.hold(.5)
 if _VIEW is not None and _LEFT_LOAD<10.:
  print('TURN_GRASP_MISSED '+json.dumps({'left_min_tip_N':_LEFT_LOAD}),flush=True);return
 _STAGE='handle_brace';p,B=base.frame();rr=B@np.diag([1.,-1.,-1.]);rp=p+B@np.array([.215,.130,.060])
 yield from R.move(rp+np.array([0,0,.10]),1.5,RH,rot=rr)
 yield from R.move(rp,1.5,RH,rot=rr)
 offset=.040
 for i in range(12):
  offset=.040-.001*i;p,B=base.frame()
  yield from R.move(p+B@np.array([.215,.130,offset]),.2,RH,rot=B@np.diag([1.,-1.,-1.]))
  yield from R.hold(.1)
  if _NORMAL>20.:raise RuntimeError('Handle brace exceeds20N')
  if _NORMAL>=3.:break
 _STAGE='bench_roll';p,B=base.frame()
 ap=L.pos if base._POSES is None else L.measured_pos;ar=L.rot if base._POSES is None else L.measured_rot
 localP=B.T@(ap-p);localR=B.T@ar;verts=[]
 for key,mesh in ALL_SHAPES.items():
  part=key.split('/')[0]
  if part=='wire':continue
  v=mesh.v.copy()
  if part=='follower':v[:,1]-=.198
  if part=='driver':v[:,0]-=.0053
  verts.extend(v)
 verts=np.array(verts)
 for i in range(160):
  f=(i+1)/160;s=f*f*(3-2*f);A=axis_angle_matrix(np.array([1.,0.,0.]),-np.pi/3*s);BR=A@B
  BP=p.copy();BP[0]=p[0]*(1-s)+.180*s;BP[1]=p[1]*(1-s)-.200*s
  BP[2]=TOP-float((verts@BR.T)[:,2].min())+.003*s
  actualP,actualR=(BP,BR) if base._POSES is None else base.frame()
  yield from ctx.both(L.move(BP+BR@localP,.05,LH,rot=BR@localR),R.move(actualP+actualR@np.array([.215,.130,offset]),.05,RH,rot=actualR@np.diag([1.,-1.,-1.])))
  if _NORMAL>25.:raise RuntimeError('Handle brace exceeds25N during roll')
 yield from L.hold(1.)
 _STAGE='release'
 yield from L.grip(-.0305,.5)
 yield from L.move(L.pos-L.rot[:,2]*.060,1.,-.0305,rot=L.rot)
 _STAGE='supported_settle'
 for i in range(100):
  if base._POSES is None:
   f=(i+1)/100;actualR=axis_angle_matrix(np.array([1.,0.,0.]),-np.pi/6*f)@BR;actualP=BP.copy();actualP[2]=TOP-float((verts@actualR.T)[:,2].min())
  else:actualP,actualR=base.frame()
  yield from R.move(actualP+actualR@np.array([.215,.130,offset]),.05,RH,rot=actualR@np.diag([1.,-1.,-1.]))
  if _NORMAL>25.:raise RuntimeError('Handle brace exceeds25N settling')
 yield from R.move(R.pos-R.rot[:,2]*.060,1.,RH,rot=R.rot)
 yield from L.move(L.pos+np.array([0,0,.080]),1.,O,rot=L.rot)
 yield from L.hold(3.)
