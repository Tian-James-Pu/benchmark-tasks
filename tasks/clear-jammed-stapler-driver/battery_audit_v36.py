"""Native force-fixture component controls; NOT a robot demo or scored success.

Pull each real latch before pressing it, then press and pull. Forces are bounded
and stop at measured60mm travel. No positions, velocities or joint flags are set.
"""
import sys,pathlib,json
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from full_case_v33 import *
import magnetic_return_v21
from axial_observer_v15 import on_physics_step
import torch
from dataclasses import replace

def reporting(part):
 old=part.build
 def build():
  from pxr import Sdf,Usd,UsdPhysics,PhysxSchema
  l=Sdf.Layer.CreateAnonymous('.usda');l.ImportFromString(old());stage=Usd.Stage.Open(l)
  for prim in stage.Traverse():
   if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
  return l.ExportToString()
 return replace(part,build=build)
OBJECTS=[reporting(x) if x.name in ('stapler','battery') else x for x in OBJECTS]
N=0;LAST={};V=None;_AUDIT={};_SUBSTEP=0

def apply(env,part,entries,add=False):
 global _AUDIT
 a=env.unwrapped.scene[part];s=m.state(env,part);origin=env.unwrapped.scene.env_origins[0].detach().cpu().numpy()
 ids=[];forces=[];points=[]
 for body,force,point in entries:
  point=np.array(point,dtype=float)
  if part=='battery' and body.startswith('clip_'):
   side=-1 if body=='clip_0' else 1;pivot=np.array([.216,.262,side*.034]);angle=s.joint(body)
   from yam_arena.urdf import axis_angle_matrix
   point=pivot+axis_angle_matrix(np.array([1.,0.,0.]),angle)@(point-pivot)
  if part=='stapler' and body in ('cover','release'):
   point[1]+=s.joint('cover')
   if body=='release':point[0]+=s.joint('release')
  ids.append(int(a.find_bodies(body)[0][0]));forces.append((s.rotation()@np.array(force)).tolist());points.append((s.pos+s.rotation()@np.array(point)+origin).tolist())
 f=torch.tensor([forces],dtype=torch.float32,device=a.device);p=torch.tensor([points],dtype=torch.float32,device=a.device)
 fn=a.permanent_wrench_composer.add_forces_and_torques_index if add else a.permanent_wrench_composer.set_forces_and_torques_index
 fn(forces=f,torques=torch.zeros_like(f),positions=p,body_ids=ids,is_global=True)
 if part=='battery':
  comp=a.permanent_wrench_composer;actual=comp.global_force_w.numpy()[0].copy();torque=comp.global_torque_w.numpy()[0].copy();expected=np.zeros_like(actual)
  for i,body in enumerate(ids):expected[body]=forces[i]
  err=float(np.max(np.abs(actual-expected)))
  if err>1e-5:raise RuntimeError('Battery fixture force mismatch '+str(err))
  _AUDIT={'ids':ids,'expected_force_N':expected.tolist(),'actual_force_N':actual.tolist(),'world_origin_torque_Nm':torque.tolist(),'force_error_N':err,'points_world_m':points}


@on_physics_step
def fixture(env,dt):
 global LAST,_SUBSTEP
 _SUBSTEP+=1
 a=m.state(env,'stapler');b=m.state(env,'battery');relative=a.rotation().T@(b.pos-a.pos)
 bp=4. if (20<=N<60 or 80<=N<140) and relative[1]<.060 else 0.
 squeeze=5. if 60<=N<140 else 0.
 apply(env,'battery',[('frame',(0,bp,0),(.216,.243,0)),('clip_0',(0,0,squeeze),(.216,.243,-.035)),('clip_2',(0,0,-squeeze),(.216,.243,.035))])
 pull=2. if (180<=N<220 or 240<=N<300) and a.joint('cover')<.060 else 0.
 press=1.6 if 220<=N<300 else 0.
 # The magnetic callback resets this articulation first each substep; add the
 # fixture loads afterwards without erasing its balanced internal forces.
 apply(env,'stapler',[('cover',(0,pull,0),(.045,.2215,0)),('release',(-press,0,0),(.0685,.223,0))],add=True)
 if 76<=N<=90:
  print('PHYS_BATTERY '+json.dumps(dict(_AUDIT,control=N,substep=_SUBSTEP,pos=b.pos.tolist(),quat_xyzw=b.quat_xyzw.tolist(),lin_vel=b.lin_vel.tolist(),ang_vel=b.ang_vel.tolist(),relative=relative.tolist())),flush=True)
 LAST={'step':N,'battery_in_frame':relative.tolist(),'battery_R_in_frame':(a.rotation().T@b.rotation()).tolist(),'battery_pull_N':bp,'squeeze_N_per_clip':squeeze,'clips':[b.joint('clip_0'),b.joint('clip_2')],'cover_m':a.joint('cover'),'release_m':a.joint('release'),'cover_pull_N':pull,'release_press_N':press,'driver_m':a.joint('driver')}

@m.on_step
def evidence(env):
 global N,V
 N+=1
 if V is None:
  from isaaclab_physx.physics import PhysxManager
  from pxr import UsdPhysics
  import isaaclab.sim as sim_utils
  paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  sensors=[next(p for p in paths if '/battery/' in p and p.endswith('/'+n)) for n in ['clip_0','clip_2']]+[next(p for p in paths if '/stapler/' in p and p.endswith('/release'))]
  target=[next(p for p in paths if '/stapler/' in p and p.endswith('/frame'))]
  V=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[target]*3,max_contact_data_count=1024)
  print('LATCH_CONTACT_COVERAGE '+json.dumps({'sensors':sensors,'target':target}),flush=True)
 if N==10:
  asset=env.unwrapped.scene['battery'];print('NATIVE_MASS '+json.dumps({'body_names':asset.body_names,'mass_kg':m._np(asset.data.body_mass).tolist(),'local_com_m':m._np(asset.data.body_com_pos_b).tolist()}),flush=True)
 if N%5==0:
  force=V.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().copy()
  print('LATCH_CONTROL '+json.dumps(dict(LAST,normal_contact_force_N=force[:,0,:].tolist())),flush=True)
def demo_script(ctx):yield from ctx.left.hold(8.)
