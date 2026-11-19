"""Score-disabled original end-rail cover pickup diagnostic; no state writes."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import *
from dataclasses import dataclass,replace
from scipy.spatial.transform import Rotation
from yam_arena import demo as d,ik,station
from physics_observer import on_physics_step
import interarm_guard
COUNT=0;SNAP={};VIEW=None;BODIES=[];FILTERS=[];SAMPLES=[]

def reporting(part):
 original=part.build
 def build():
  from pxr import Sdf,Usd,UsdPhysics,PhysxSchema
  layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(original());stage=Usd.Stage.Open(layer)
  for prim in stage.Traverse():
   if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
  return stage.GetRootLayer().ExportToString()
 return replace(part,build=build)
OBJECTS=[reporting(p) if p.name in ('vacuum','roller','cover') else p for p in OBJECTS]

def initialize_contact(env):
 global VIEW,BODIES,FILTERS
 from isaaclab_physx.physics import PhysxManager
 from pxr import UsdPhysics
 import isaaclab.sim as sim_utils
 paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
 BODIES=[p for p in paths if any('/'+n+'/' in p for n in ('vacuum','roller','cover'))];FILTERS=[p for p in paths if '/LeftArm/' in p or '/RightArm/' in p]
 if len(BODIES)!=7 or not FILTERS:raise RuntimeError(str((BODIES,FILTERS)))
 VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(BODIES,filter_patterns=[FILTERS]*len(BODIES),max_contact_data_count=8192)
 print('COVER_CONTACT_PATHS',json.dumps(dict(bodies=BODIES,filters=FILTERS)),flush=True)

@on_physics_step
def sample(env,dt):
 if VIEW is None:return
 force=np.asarray(VIEW.get_contact_force_matrix(dt=dt).numpy(),float)
 if force.shape!=(len(BODIES),len(FILTERS),3) or not np.isfinite(force).all():raise RuntimeError('Bad contact matrix')
 SAMPLES.append(np.linalg.norm(force,axis=-1))

def fail(reason):
 print('COVER_PICK_ABORT',COUNT,reason,flush=True);raise RuntimeError(reason)

def observe(env):
 global COUNT
 COUNT+=1;cover=m.state(env,'cover')
 if COUNT==1:SNAP.update(position=cover.pos.copy(),rotation=cover.rotation().copy())
 if VIEW is None:initialize_contact(env)
 sample(env,env.unwrapped.physics_dt);maximum=np.max(SAMPLES,axis=0);pairs=[]
 for i,j in zip(*np.where(maximum>.05)):
  pairs.append(dict(body='/'.join(BODIES[i].split('/')[-2:]),robot=FILTERS[j].split('/')[4]+'/'+FILTERS[j].split('/')[-1],force_N=float(maximum[i,j])))
 samples=len(SAMPLES);SAMPLES.clear()
 unexpected=[x for x in pairs if not (x['body']=='cover/frame' and x['robot'] in ('RightArm/tip_left','RightArm/tip_right')) and x['force_N']>5]
 if unexpected:fail('unexpected contact '+str(unexpected))
 q=m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0,:6];tcp,rot=ik.tcp_pose(q,station.BASES['right']);gp=rot.T@(cover.pos-tcp);gr=rot.T@cover.rotation()
 if COUNT==180:
  forces={tip:max([x['force_N'] for x in pairs if x['body']=='cover/frame' and x['robot']=='RightArm/'+tip],default=0.) for tip in ('tip_left','tip_right')}
  print('COVER_GRIP_CONTACT',forces,flush=True)
  if min(forces.values())<.25:fail('no opposing native tip contact on original cover rail')
  SNAP.update(grip_p=gp.copy(),grip_r=gr.copy())
 if COUNT in (280,320):
  slip=float(np.linalg.norm(gp-SNAP['grip_p']));turn=float(np.degrees(np.linalg.norm(ik._rot_error(SNAP['grip_r'],gr))));lift=float(cover.pos[2]-SNAP['position'][2]);print('COVER_RETAINED',json.dumps(dict(step=COUNT,lift_m=lift,grip_slip_m=slip,grip_turn_deg=turn)),flush=True)
  if slip>.008 or turn>10 or lift<.10:fail('cover not securely lifted on original rail')
 if COUNT%10==0:print('COVER_PICK_STATUS',json.dumps(dict(step=COUNT,physics_samples=samples,cover_pos=cover.pos.tolist(),cover_rotation=cover.rotation().tolist(),tcp=tcp.tolist(),grip_p=gp.tolist(),grip_r=gr.tolist(),pairs=pairs)),flush=True)
m.on_step(observe)

def reset(env=None):
 global COUNT,VIEW
 COUNT=0;VIEW=None;SNAP.clear();SAMPLES.clear()
m.on_reset(reset)

@dataclass
class Point(Waypoint):goal:object=None
previous_target=d.ArmPlan.target

def target(self,time):
 if not all(isinstance(p,Point) for p in self.waypoints):return previous_target(self,time)
 p0,r0,f0=self.start_pos,self.start_rot,self.finger_start
 for point in self.waypoints:
  p1,r1=point.goal()
  if time<point.duration:
   f=ik.smoothstep(time/point.duration);return p0+(p1-p0)*f,ik.slerp_matrix(r0,r1,f),f0+(point.finger-f0)*f,point.label
  time-=point.duration;p0,r0,f0=p1,r1,point.finger
 return p0,r0,f0,'hold lifted cover'
d.ArmPlan.target=target
original_action=d.ArmPlan.action7

def action7(self,time):
 action,label=original_action(self,time);p,r,_,_=self.target(time);ap,ar=ik.tcp_pose(action[:6],self.base_pos)
 if np.linalg.norm(ap-p)>.002 or np.linalg.norm(ik._rot_error(r,ar))>np.radians(2):fail('commanded IK residual')
 return action,label
d.ArmPlan.action7=action7

def goal(height=0.,away=0.):
 p=SNAP.get('position',np.array(COVER.pos));r=SNAP.get('rotation',np.eye(3));hand=r@Rotation.from_euler('y',-35,degrees=True).as_matrix()@d.grip_rotation(0)
 return p+r@np.array([0,-.104,0])-hand@np.array([0,0,.014])+np.array([0,-away,height]),hand

def point(duration,finger,label,fn):return Point(duration,(0,0,0),finger,label,0.,fn)
def demo():
 ready=ik.tcp_pose(d.READY,station.BASES['right'])
 return dict(right=[point(1.,-.016,'wait at ready',lambda:ready),point(3.,-.016,'above original end rail',lambda:goal(.16)),point(3.,-.016,'straddle original end rail',lambda:goal()),point(1.,0.,'grip end rail',lambda:goal()),point(1.,0.,'verify opposing contacts',lambda:goal()),point(2.,0.,'clear adjacent body overhang',lambda:goal(.005,.015)),point(3.,0.,'lift cover',lambda:goal(.16,.015)),point(2.,0.,'inspect retained cover',lambda:goal(.16,.015))])
