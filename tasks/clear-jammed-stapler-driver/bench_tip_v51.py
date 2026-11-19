"""Bounded isolated-tool bench tip by actual gripper contact; no repair claim.

A fingertip end pushes the unchanged handle past its balance point, then clears
while the original housing settles on its side. Battery starts separately in
its archived released pose. No object force, velocity or pose is assigned.
"""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import bench_tip_v49 as obs
from bench_tip_v49 import *
@m.on_step
def wire_observe(env):
 if base._k%5==0:
  a=m.state(env,'stapler');w=m.state(env,'bent_staple')
  print('TIP_WIRE '+json.dumps({'step':base._k,'wire_pos':w.pos.tolist(),'wire_R':w.rotation().tolist(),'wire_in_body':(a.rotation().T@(w.pos-a.pos)).tolist(),'driver_m':a.joint('driver'),'cover_m':a.joint('cover')}),flush=True)
def demo_script(ctx):
 base._CTX=ctx;R=ctx.right;H=-.006
 P,B=base.frame();rot=B@np.diag([1.,-1.,-1.]);p=P+B@np.array([.215,.130,.090])
 obs._STAGE='approach'
 yield from R.move(p+np.array([0,0,.080]),1.5,H,rot=rot)
 yield from R.move(p,1.,H,rot=rot)
 yield from R.move(P+B@np.array([.215,.130,.0368425]),1.5,H,rot=rot)
 indent=0.;theta=90.
 for i in range(180):
  obs._STAGE='handle_tip'
  P,B=ideal(max(54.,90.-i*.3)) if base._POSES is None else base.frame()
  theta=float(np.degrees(np.arctan2(B[2,1],B[2,2])))
  if theta<55.:break
  force=float(obs._FORCES[1])
  if force>15.:raise RuntimeError('Bench tip handle force exceeds15N')
  goal=min(10.,1.+i*.12)
  indent=float(np.clip(indent+np.clip((goal-force)*.00020,-.001,.001),0.,.018))
  yield from R.move(P+B@np.array([.215,.130,.0348425-indent]),.05,H,rot=B@np.diag([1.,-1.,-1.]))
 print('BENCH_TIP_STOP '+json.dumps({'roll_deg':theta,'iterations':i+1,'force_components_N':obs._FORCES.tolist()}),flush=True)
 obs._STAGE='clear_and_settle'
 yield from R.move(R.pos+np.array([0,-.080,.080]),1.,H,rot=R.rot)
 yield from R.hold(4.)
