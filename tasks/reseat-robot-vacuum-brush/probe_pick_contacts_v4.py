"""Read-only contact diagnosis of roller pickup; score always zero."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import probe_insert_v1 as controller
from probe_insert_v1 import *
from dataclasses import replace
from physics_observer import on_physics_step

def reporting(part):
 original=part.build
 def build():
  from pxr import Sdf,Usd,UsdPhysics,PhysxSchema
  layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(original());stage=Usd.Stage.Open(layer)
  for prim in stage.Traverse():
   if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
  return stage.GetRootLayer().ExportToString()
 return replace(part,build=build)
OBJECTS=[reporting(x) if x.name in ('vacuum','roller') else x for x in OBJECTS]
VIEW=None;BODIES=[];FILTERS=[];SAMPLES=[];STEP=0;LAST_CONTACTS=[]

def initialize(env):
 global VIEW,BODIES,FILTERS
 from isaaclab_physx.physics import PhysxManager
 from pxr import UsdPhysics
 import isaaclab.sim as sim_utils
 paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
 BODIES=[p for p in paths if '/vacuum/' in p or '/roller/' in p]
 FILTERS=[p for p in paths if ('/LeftArm/' in p or '/RightArm/' in p or '/vacuum/' in p)]
 if len(BODIES)!=4 or not FILTERS:raise RuntimeError(str((BODIES,FILTERS)))
 VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(BODIES,filter_patterns=[FILTERS]*len(BODIES),max_contact_data_count=8192)
 print('PICK_CONTACT_PATHS',json.dumps(dict(bodies=BODIES,filters=FILTERS)),flush=True)

@on_physics_step
def sample(env,dt):
 if VIEW is None:return
 f=np.asarray(VIEW.get_contact_force_matrix(dt=dt).numpy(),float)
 if f.shape!=(len(BODIES),len(FILTERS),3) or not np.isfinite(f).all():raise RuntimeError('Invalid contact matrix')
 SAMPLES.append(np.linalg.norm(f,axis=-1))

def contacts(env):
 global STEP,LAST_CONTACTS
 STEP+=1
 if VIEW is None:initialize(env)
 sample(env,env.unwrapped.physics_dt)
 maximum=np.max(SAMPLES,axis=0);pairs=[]
 for i,j in zip(*np.where(maximum>.05)):
  pairs.append(dict(body='/'.join(BODIES[i].split('/')[-2:]),robot=FILTERS[j].split('/')[4]+'/'+FILTERS[j].split('/')[-1],peak_normal_resultant_N=float(maximum[i,j])))
 LAST_CONTACTS=pairs
 print('PICK_CONTACT',json.dumps(dict(step=STEP,physics_samples=len(SAMPLES),pairs=pairs)),flush=True)
 SAMPLES.clear()
m.on_step(contacts)

def reset(env=None):
 global VIEW,STEP,LAST_CONTACTS
 VIEW=None;STEP=0;LAST_CONTACTS=[];SAMPLES.clear()
 controller.COUNT=0;controller.SNAP.clear();controller.LIVE.clear();controller.ABORT=False;controller.LAST.clear()
 controller.pickup._probe_steps=0
m.on_reset(reset)

def demo():
 # Stop after pickup and held observation: insertion waypoints are omitted.
 return {'left':controller.demo()['left'][:6]}
