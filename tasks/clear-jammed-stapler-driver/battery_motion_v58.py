"""Native inverted bench-supported battery isolation, unchanged source hardware.

Both robot fingertip ends press the opposed pads normally. No fixture wrenches,
pose assignments, logical release, or revised battery geometry are used.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import service_base_v49 as base
from service_base_v49 import *
import magnetic_return_v21
_BAT=None;_VIEW=None;_LOADS=None
@m.on_step
def battery_observe(env):
 global _BAT,_VIEW,_LOADS
 b=m.state(env,'battery');a=m.state(env,'stapler');_BAT=b
 if _VIEW is None:
  from isaaclab_physx.physics import PhysxManager
  from pxr import UsdPhysics
  import isaaclab.sim as sim_utils
  paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  sensors=[next(p for p in paths if '/'+arm+'Arm/' in p and p.endswith('/'+n)) for arm in ['Left','Right'] for n in ['tip_left','tip_right']]
  filters=[next(p for p in paths if '/battery/' in p and p.endswith('/'+n)) for n in ['clip_0','clip_2','frame']]
  _VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*4,max_contact_data_count=2048)
  print('BATTERY_CONTACT_PATHS '+json.dumps({'sensors':sensors,'filters':filters}),flush=True)
 force=_VIEW.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().copy()
 _LOADS=[sum(max(0.,float(f@(-b.rotation()[:,2]))) for f in force[:2,0]),sum(max(0.,float(f@b.rotation()[:,2])) for f in force[2:,1])]
 if base._k%5==0:print('BATTERY_ROBOT '+json.dumps({'step':base._k,'battery_in_frame':(a.rotation().T@(b.pos-a.pos)).tolist(),'battery_R_in_frame':(a.rotation().T@b.rotation()).tolist(),'clips':[b.joint('clip_0'),b.joint('clip_2')],'loads_N':_LOADS,'finger_forces_N':force.tolist(),'body_pose':a.pos.tolist(),'battery_pos':b.pos.tolist(),'measured_tcp':None if base._CTX is None else {name:arm.measured_pos.tolist() for name,arm in [('left',base._CTX.left),('right',base._CTX.right)]}}),flush=True)
def bp():return (NOSE,m.quat_matrix(UPRIGHT_Q)) if _BAT is None else (_BAT.pos,_BAT.rotation())
def demo_script(ctx):
 base._CTX=ctx;L,R=ctx.left,ctx.right;H=-.010
 p,B=bp();lr=B@np.diag([-1.,-1.,1.]);rr=B@np.diag([1.,-1.,-1.])
 lp=p+B@np.array([.216,.243,-.100]);rp=p+B@np.array([.216,.243,.100])
 yield from ctx.both(L.move(lp,2.,H,rot=lr),R.move(rp,2.,H,rot=rr))
 yield from L.hold(.5)
 p,B=bp();lp=p+B@np.array([.216,.243,-.048]);rp=p+B@np.array([.216,.243,.048])
 yield from ctx.both(L.move(lp,1.5,H,rot=lr),R.move(rp,1.5,H,rot=rr))
 yield from L.hold(.4)
 ready=False
 for i in range(14):
  p,B=bp();depth=.0005*(i+1)
  lp=p+B@np.array([.216,.243,-(.048-depth)]);rp=p+B@np.array([.216,.243,.048-depth])
  yield from ctx.both(L.move(lp,.15,H,rot=lr),R.move(rp,.15,H,rot=rr))
  yield from L.hold(.1)
  if _LOADS is not None and max(_LOADS)>25.:raise RuntimeError('Battery press exceeded25N bound')
  ready=_BAT is None or (min(-_BAT.joint('clip_0'),_BAT.joint('clip_2'))>=np.radians(7.5) and min(_LOADS)>=6.)
  if ready and _BAT is not None:break
 print('BATTERY_PRESS_GATE '+json.dumps({'ready':ready,'iterations':i+1,'loads_N':_LOADS,'clips':None if _BAT is None else [_BAT.joint('clip_0'),_BAT.joint('clip_2')]}),flush=True)
 if not ready:return
 p,B=bp();up=B[:,1]*.080
 yield from ctx.both(L.move(L.pos+up,3.,H,rot=lr),R.move(R.pos+up,3.,H,rot=rr))
 yield from L.hold(2.)

 # Move clear of the stapler while high, then lower until the stem meets bench.
 # Stable resting orientation after release is left to physical contacts.
 p,B=bp();com=p+B@np.array([.194432205,.237609743,0.])
 if _BAT is None:com=com+B[:,1]*.080
 target=com.copy();target[0]=.500;target[1]=0.
 delta=target-com
 yield from ctx.both(L.move(L.pos+delta,2.5,H,rot=lr),R.move(R.pos+delta,2.5,H,rot=rr))
 yield from L.hold(.5)
 p,B=bp();com=p+B@np.array([.194432205,.237609743,0.]) if _BAT is not None else target
 # Nominal stem172mm from root and COM237.61mm: stem bottom3mm above bench.
 down=np.array([0.,0.,TOP+.068609743-com[2]])
 yield from ctx.both(L.move(L.pos+down,3.,H,rot=lr),R.move(R.pos+down,3.,H,rot=rr))
 yield from L.hold(.5)
 yield from ctx.both(L.move(L.pos+np.array([0.,.070,0.]),1.,H,rot=lr),R.move(R.pos+np.array([0.,-.070,0.]),1.,H,rot=rr))
 yield from L.hold(3.)
