"""Bounded robot contact gate from the already-isolated bench service state.

Original hardware, spring, mass and friction. Opposed fingertip end contacts
provide a pressure-controlled bench tip; no object motion or forces are assigned.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import service_base_v49 as base
from service_base_v49 import *
import magnetic_return_v21
from yam_arena.urdf import axis_angle_matrix
from scipy.spatial import ConvexHull
_STAGE='approach';_VIEW=None;_FORCES=np.zeros(2)
_HULLS=[ConvexHull(mesh.v).equations for name,mesh in ALL_SHAPES.items() if name.startswith('frame/')]
@m.on_step
def observe_tip(env):
 global _VIEW,_FORCES
 a=m.state(env,'stapler')
 if _VIEW is None:
  from isaaclab_physx.physics import PhysxManager
  from pxr import UsdPhysics
  import isaaclab.sim as sim_utils
  paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  sensors=[next(p for p in paths if '/'+arm+'Arm/' in p and p.endswith('/'+n)) for arm in ['Left','Right'] for n in ['tip_left','tip_right']]
  filters=[next(p for p in paths if '/stapler/' in p and p.endswith('/frame'))]
  _VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*4,max_contact_data_count=2048)
 forces=_VIEW.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().copy()
 _FORCES=np.array([sum(max(0.,float(v[1])) for v in forces[:2,0]),sum(max(0.,float(v@a.rotation()[:,2])) for v in forces[2:,0])])
 if base._k%5==0:print('BENCH_TIP '+json.dumps({'step':base._k,'stage':_STAGE,'body_pos':a.pos.tolist(),'body_R':a.rotation().tolist(),'normal_components_N':_FORCES.tolist(),'finger_normal_vectors_N':forces.tolist(),'roll_deg':float(np.degrees(np.arctan2(a.rotation()[2,1],a.rotation()[2,2])))}),flush=True)
def outer_y(P,B,x,z):
 """Read-only horizontal ray against the authored convex housing solids."""
 hits=[];direction=B.T@np.array([0.,1.,0.])
 for dx in [-.008,.008]:
  for dz in [-.006,0.,.006]:
   origin=B.T@(np.array([x+dx,0.,z+dz])-P)
   for planes in _HULLS:
    c=planes[:,:3]@origin+planes[:,3];v=planes[:,:3]@direction
    if np.any((np.abs(v)<1e-10)&(c>1e-9)):continue
    low=max((-c[v< -1e-10]/v[v< -1e-10]).tolist(),default=-np.inf)
    high=min((-c[v>1e-10]/v[v>1e-10]).tolist(),default=np.inf)
    if low<=high and np.isfinite(high):hits.append(high)
 if not hits:raise RuntimeError('No housing surface for support contact')
 return max(hits)
def ideal(theta):
 B=axis_angle_matrix(np.array([1.,0.,0.]),np.radians(theta));pivot=B@np.array([0.,-.027,-.0405])
 return np.array([.050,.0405-pivot[1],TOP-pivot[2]]),B
def demo_script(ctx):
 global _STAGE
 base._CTX=ctx;L,R=ctx.left,ctx.right;H=-.006
 P,B=base.frame();leftR=B@np.diag([-1.,-1.,1.]);rightR=B@np.diag([1.,-1.,-1.]);z0=float((P+B@np.array([.215,.130,0]))[2]);x0=float((P+B@np.array([.215,.130,0]))[0])
 lp=np.array([x0,.090,z0]);rp=P+B@np.array([.215,.130,.090])
 yield from ctx.both(L.move(lp+np.array([0,0,.08]),1.5,H,rot=leftR),R.move(rp+np.array([0,0,.08]),1.5,H,rot=rightR))
 yield from ctx.both(L.move(lp,1.,H,rot=leftR),R.move(rp,1.,H,rot=rightR))
 P,B=base.frame();contact=P+B@np.array([.215,.130,0.]);lp=np.array([contact[0],outer_y(P,B,contact[0],z0)+.0148425,z0]);rp=P+B@np.array([.215,.130,.0348425])
 yield from ctx.both(L.move(lp,1.5,H,rot=leftR),R.move(rp,1.5,H,rot=rightR))
 yield from L.hold(.5)
 indent=np.zeros(2);last_theta=90.;right_clear=False
 for i in range(300):
  _STAGE='acquire' if i<40 else 'supported_tip'
  desired=max(0.,90.-max(0,i-40)*.4)
  P,B=ideal(desired) if base._POSES is None else base.frame()
  theta=float(np.degrees(np.arctan2(B[2,1],B[2,2])));omega=(theta-last_theta)/.05;last_theta=theta
  if i>=40 and theta<6.:break
  if max(_FORCES)>25.:raise RuntimeError('Bench tip normal component exceeded25N')
  if i<40:goals=np.array([2.,2.])
  else:
   t=np.radians(theta);equilibrium=2.67*9.81*(.0405*np.sin(t)-(.077485517+.027)*np.cos(t))/.157
   differential=equilibrium+.25*(theta-desired)+.03*(omega+8.)
   goals=np.array([max(1.,1.-differential),max(1.,1.+differential)])
   goals=np.minimum(goals,[18.,12.])
  indent=np.clip(indent+np.clip((goals-_FORCES)*.00025,-.0015,.0015),0.,.012)
  contact=P+B@np.array([.215,.130,0]);z=min(z0,float((P+B@np.array([.215,.214,.020]))[2]));z=max(TOP+.060,z)
  lp=np.array([contact[0],outer_y(P,B,contact[0],z)+.0128425-indent[0],z])
  if theta<25.:
   if not right_clear:
    right_clear=True;clear_step=i;right_start=R.pos.copy();rightR=R.rot.copy()
   rp=right_start+np.array([0.,-.080,.020])*min(1.,(i-clear_step+1)/20)
  else:rp=P+B@np.array([.215,.130,.0328425-indent[1]]);rightR=B@np.diag([1.,-1.,-1.])
  yield from ctx.both(L.move(lp,.05,H,rot=leftR),R.move(rp,.05,H,rot=rightR))
 print('BENCH_TIP_STOP '+json.dumps({'roll_deg':theta,'iterations':i+1,'force_components_N':_FORCES.tolist()}),flush=True)
 _STAGE='release'
 yield from ctx.both(L.move(L.pos+np.array([0,.080,.040]),1.,H,rot=L.rot),R.move(R.pos+np.array([0,-.040,.040]),1.,H,rot=R.rot))
 yield from L.hold(3.)
