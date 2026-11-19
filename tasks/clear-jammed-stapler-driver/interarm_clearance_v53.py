"""Read-only native contact guard between the complete left and right robots."""
import json,numpy as np
import mechkit as m
from axial_observer_v15 import on_physics_step
V=None;S=[];N=0;B=[];F=[]
def initialize(env):
 global V,B,F
 from pxr import Usd,UsdPhysics
 import isaaclab.sim as sim_utils
 from isaaclab_physx.physics import PhysxManager
 prims=list(Usd.PrimRange(sim_utils.get_current_stage().GetPseudoRoot(),Usd.TraverseInstanceProxies()))
 B=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and '/LeftArm/' in str(p.GetPath())]
 F=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and '/RightArm/' in str(p.GetPath())]
 if len(B)<9 or len(F)<9:raise RuntimeError('Incomplete inter-arm body coverage '+str((B,F)))
 V=PhysxManager.get_physics_sim_view().create_rigid_contact_view(B,filter_patterns=[F]*len(B),max_contact_data_count=16384)
 print('INTERARM_COVERAGE '+json.dumps({'left_bodies':B,'right_bodies':F}),flush=True)
def capture(dt):
 force=V.get_contact_force_matrix(dt=dt).numpy().copy();normal,_,_,_,counts,starts=V.get_contact_data(dt=dt);counts=counts.numpy().copy();starts=starts.numpy().copy()
 if counts.sum()>=16384 or (counts<0).any():raise RuntimeError('Inter-arm contact capacity')
 active=counts>0;b=starts[active];e=b+counts[active];order=np.argsort(b)
 if (b<0).any() or (e>len(normal)).any() or (b[order][1:]<e[order][:-1]).any():raise RuntimeError('Inter-arm contact indexing')
 return {'points':int(counts.sum()),'max_pair_normal_N':float(np.linalg.norm(force,axis=-1).max()),'pairs':[[B[i],F[j]] for i,j in zip(*np.where(active))]}
@on_physics_step
def physics(env,dt):
 if V is not None:S.append(capture(dt))
@m.on_step
def terminal(env):
 global N
 N+=1
 if V is None:initialize(env);S.clear();return
 S.append(capture(env.unwrapped.physics_dt))
 if len(S)!=25:raise RuntimeError('Expected25 inter-arm samples, got'+str(len(S)))
 row={'step':N,'samples':len(S),'max_points':max(x['points'] for x in S),'max_pair_normal_N':max(x['max_pair_normal_N'] for x in S),'pairs':sorted(set(tuple(p) for x in S for p in x['pairs']))}
 if N%10==0 or row['max_points']:print('INTERARM_CLEARANCE '+json.dumps(row),flush=True)
 if row['max_points']:raise RuntimeError('Inter-arm contact guard '+str(row))
 S.clear()
