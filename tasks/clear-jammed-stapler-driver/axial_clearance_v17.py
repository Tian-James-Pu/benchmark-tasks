"""Native hand/wrist versus all static-collider contact audit; read only."""
import json,numpy as np
import mechkit as m
from axial_observer_v15 import on_physics_step
V=None;S=[];N=0;B=[];F=[]
def initialize(env):
 global V,B,F
 from pxr import Usd,UsdPhysics
 import isaaclab.sim as sim_utils
 from isaaclab_physx.physics import PhysxManager
 stage=sim_utils.get_current_stage();prims=list(Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies()))
 B=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and any('/'+side+'Arm/' in str(p.GetPath()) for side in ('Left','Right')) and p.GetName() in ('link4','link5','gripper','tip_left','tip_right')]
 F=[]
 for p in prims:
  if not p.HasAPI(UsdPhysics.CollisionAPI):continue
  a=p;rigid=False
  while a and not a.IsPseudoRoot():
   rigid|=a.HasAPI(UsdPhysics.RigidBodyAPI);a=a.GetParent()
  if not rigid:F.append(str(p.GetPath()))
 if len(B)!=10 or not F:raise RuntimeError('Incomplete hand/static coverage '+str((B,F)))
 V=PhysxManager.get_physics_sim_view().create_rigid_contact_view(B,filter_patterns=[F]*len(B),max_contact_data_count=16384)
 print('AXIAL_CLEARANCE_COVERAGE '+json.dumps({'bodies':B,'static_colliders':F}),flush=True)
def capture(dt):
 force=np.asarray(V.get_contact_force_matrix(dt=dt).numpy()).copy()
 normal,_,_,_,counts,starts=V.get_contact_data(dt=dt);counts=counts.numpy().copy();starts=starts.numpy().copy()
 if counts.sum()>=16384 or (counts<0).any():raise RuntimeError('Static contact overflow')
 active=counts>0;b=starts[active];e=b+counts[active];order=np.argsort(b)
 if (b<0).any() or (e>len(normal)).any() or (b[order][1:]<e[order][:-1]).any():raise RuntimeError('Invalid contact indexing')
 return {'max_normal_N':float(np.linalg.norm(force,axis=-1).max()),'points':int(counts.sum()),'active_bodies':[B[i] for i in np.flatnonzero(active.any(axis=1))]}
@on_physics_step
def physics(env,dt):
 if V is not None:S.append(capture(dt))
@m.on_step
def terminal(env):
 global N
 N+=1
 if V is None:initialize(env);S.clear();return
 S.append(capture(env.unwrapped.physics_dt))
 if len(S)!=25:raise RuntimeError('Expected25 interval samples, got'+str(len(S)))
 row={'step':N,'samples':len(S),'max_normal_N':max(x['max_normal_N'] for x in S),'max_points':max(x['points'] for x in S),'active_bodies':sorted(set(b for x in S for b in x['active_bodies']))}
 if N%10==0 or row['max_points']:print('AXIAL_CLEARANCE '+json.dumps(row),flush=True)
 if row['max_points']:raise RuntimeError('Hand/static contact guard: '+str(row))
 S.clear()
