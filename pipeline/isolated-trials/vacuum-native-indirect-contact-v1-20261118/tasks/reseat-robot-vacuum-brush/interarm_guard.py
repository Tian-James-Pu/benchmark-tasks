"""Read-only native left/right arm contact guard for transfer diagnostics."""
import json,numpy as np
import mechkit as m
from physics_observer import on_physics_step
VIEW=None;LEFT=[];RIGHT=[];SAMPLES=0;PEAK=None;STEPS=0
CAPACITY=4096

def initialize(env):
 global VIEW,LEFT,RIGHT,PEAK
 from pxr import UsdPhysics
 from isaaclab_physx.physics import PhysxManager
 import isaaclab.sim as sim_utils
 paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
 LEFT=[p for p in paths if '/LeftArm/' in p];RIGHT=[p for p in paths if '/RightArm/' in p]
 if len(LEFT)!=9 or len(RIGHT)!=9:raise RuntimeError('Incomplete interarm coverage')
 VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(LEFT,filter_patterns=[RIGHT]*len(LEFT),max_contact_data_count=CAPACITY)
 PEAK=np.zeros((len(LEFT),len(RIGHT)))
 print('INTERARM_COVERAGE',json.dumps(dict(left=LEFT,right=RIGHT)),flush=True)

@on_physics_step
def capture(env,dt):
 global SAMPLES
 if VIEW is None:return
 normal,_,_,_,counts,starts=VIEW.get_contact_data(dt=dt)
 normal=np.asarray(normal.numpy(),float).reshape(-1);counts=np.asarray(counts.numpy());starts=np.asarray(starts.numpy())
 if counts.shape!=PEAK.shape or counts.sum()>=CAPACITY or (counts<0).any():raise RuntimeError('Invalid interarm contact counts')
 for i,j in zip(*np.where(counts>0)):
  n=int(counts[i,j]);s=int(starts[i,j])
  if s<0 or s+n>len(normal):raise RuntimeError('Invalid interarm contact range')
  force=float(np.abs(normal[s:s+n]).sum())
  if not np.isfinite(force):raise RuntimeError('Nonfinite interarm force')
  PEAK[i,j]=max(PEAK[i,j],force)
 SAMPLES+=1

@m.on_step
def step(env):
 global STEPS,SAMPLES
 STEPS+=1
 if VIEW is None:initialize(env)
 capture(env,env.unwrapped.physics_dt)
 expected=int(round(.05/env.unwrapped.physics_dt))+1
 if STEPS>1 and SAMPLES!=expected:raise RuntimeError(f'Incomplete interarm sampling {SAMPLES}/{expected}')
 maximum=float(PEAK.max());i,j=np.unravel_index(np.argmax(PEAK),PEAK.shape)
 if STEPS%20==0 or maximum>.25:print('INTERARM_CONTACT',json.dumps(dict(step=STEPS,samples=SAMPLES,peak_normal_N=maximum,left=LEFT[i],right=RIGHT[j])),flush=True)
 if maximum>.25:raise RuntimeError('Interarm contact guard exceeded0.25N')
 PEAK[:]=0.;SAMPLES=0

@m.on_reset
def reset(env=None):
 global VIEW,PEAK,SAMPLES,STEPS
 VIEW=None;PEAK=None;SAMPLES=0;STEPS=0
