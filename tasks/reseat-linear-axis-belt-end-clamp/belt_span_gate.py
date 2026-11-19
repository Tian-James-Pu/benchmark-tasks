"""Diagnostic only: native segmented elastic belt, supported at its span ends.

Source Festo2019: free span246mm, mass45.9g/m, preload267–282N.
EA=260N/.0016 derives from catalog strain at max axial feed force; it is an
operating-point reconstruction, not a measured tangent modulus. Width15mm,
backing effective thickness2.4mm and rubber bending modulus5MPa are disclosed
estimates. The native bodies/joints carry load; no commanded belt poses or
computed frequency is returned as a measurement. Fixture supports exist only
in this material diagnostic, not as a proposed final task shortcut.
"""
from pathlib import Path
from dataclasses import replace
import math,json,sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import mechkit as m
from yam_arena.demo import Waypoint
SCENE='station'; SUCCESS_HOLD_S=0.
INSTRUCTION='Observe the belt representation diagnostic.'
PITCH=.003;N=82;MU=.0459;EA=162500.;PRELOAD=274.5
WIDTH=.015;THICK=.0024;EI=5e6*WIDTH*THICK**3/12
SPAN=PITCH*N;EXT=PRELOAD/EA;DX=PITCH*(1+EXT*(N-1)/N)
BASE=np.array([.30,-SPAN/2,.90]);MASS=MU*PITCH
I=np.array([MASS*(WIDTH**2+THICK**2)/12,MASS*(PITCH**2+THICK**2)/12,MASS*(PITCH**2+WIDTH**2)/12])
# Local X is longitudinal; placed along worldX for this material diagnostic.
BASE=np.array([.16,0.,.90])
LOOKS={'belt':m.Look(color=m.rgb(.15,.16,.17),roughness=.8), 'fixture':m.Look(color=m.rgb(.55,.56,.58),metallic=.6)}
def joint(name,body0,body1,p0,p1,axial=True):
 axes=('transX','transY','transZ','rotX','rotY','rotZ')
 schema=[f'PhysicsLimitAPI:{a}' for a in axes if a in ('transY','transZ') or (a=='transX' and not axial)]
 schema += [f'PhysicsDriveAPI:{a}' for a in axes if a=='transX' and axial or a.startswith('rot')]
 body=f'rel physics:body0 = <{body0}>\n' if body0 else ''
 if body1:body+=f'rel physics:body1 = <{body1}>\n'
 s=f'def PhysicsJoint "{name}" (prepend apiSchemas = {json.dumps(schema)}) {{\n'+body
 s+=f'point3f physics:localPos0 = {tuple(map(float,p0))}\npoint3f physics:localPos1 = {tuple(map(float,p1))}\n'
 s+='bool physics:collisionEnabled = false\n'
 for a in axes:
  locked=a in ('transY','transZ') or (a=='transX' and not axial)
  if locked:
   s+=f'float limit:{a}:physics:low = 1\nfloat limit:{a}:physics:high = -1\n'
   continue
  if a=='transX': k=EA/PITCH;c=2*.05*math.sqrt(k*MASS/2);target=0
  else:
   # Weak-axis bend controls the out-of-plane mode. In-plane bending uses
   # corresponding rectangular-section inertia; twist is an estimate.
   k=(EI if a=='rotY' else (5e6*THICK*WIDTH**3/12 if a=='rotZ' else EI*.6))/PITCH
   c=2*.01*math.sqrt(k*I[{'rotX':0,'rotY':1,'rotZ':2}[a]]/2)
   k*=math.pi/180;c*=math.pi/180;target=0
  s+=f'token drive:{a}:physics:type = "force"\nfloat drive:{a}:physics:stiffness = {k}\nfloat drive:{a}:physics:damping = {c}\nfloat drive:{a}:physics:targetPosition = {target}\n'
 return s+'}\n'
def segment(i):
 name=f'belt{i:03d}'; x=DX*(i+.5)
 pos=BASE+[x,0,.00015*math.sin(math.pi*(i+.5)/N)]
 shape=m.box((PITCH,WIDTH,THICK)); part=m.free(name,pos,[(name,shape,'belt')],
  [m.col_box('solid',(PITCH,WIDTH,THICK),(0,0,0)).replace('contactOffset = 0.004','contactOffset = 0.00002')],LOOKS,MASS,friction=(.3,.25),torsional_patch=None,solver_iters=64,com=(0,0,0))
 build=part.build
 def native():
  s=build().replace('"PhysicsRigidBodyAPI", "PhysicsMassAPI"','"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"')
  s=s.replace('point3f physics:centerOfMass',f'float physics:mass = {MASS}\nfloat3 physics:diagonalInertia = {tuple(map(float,I))}\nint physxRigidBody:solverPositionIterationCount = 64\nint physxRigidBody:solverVelocityIterationCount = 8\npoint3f physics:centerOfMass')
  extra=''
  if i<N-1:extra+=joint('next',f'/P_{name}',f'/World/envs/env_0/belt{i+1:03d}',(PITCH/2,0,0),(-PITCH/2,0,0))
  if i==0:extra+=joint('left_support','',f'/P_{name}',BASE,(-PITCH/2,0,0),False)
  if i==N-1:extra+=joint('right_support',f'/P_{name}','',(PITCH/2,0,0),BASE+[N*DX,0,0],False)
  at=s.rfind('}');return s[:at]+extra+s[at:]
 return replace(part,build=native)
OBJECTS=[segment(i) for i in range(N)]
# Visual/physical diagnostic fixture blocks remain clear of belt free span.
for i,x in enumerate((-.008,N*DX+.008)):
 shape=m.box((.012,.045,.06),(0,0,-.03))
 OBJECTS.append(m.fixed('support'+str(i),BASE+[x,0,0],[('block',shape,'fixture')],[m.col_hull('solid',shape)],LOOKS))
_scene=m._Finder.targets['yam_arena.scene']
def settings(scene):
 _scene(scene);scene.PHYSICS_DT=1/4800.
m._Finder.targets['yam_arena.scene']=settings
class Observe:
 def reset(self,env=None):self.n=0;self.installed=False;self.samples=[];self.native=None
 def step(self,env):
  self.n+=1
  if not self.installed:
   from isaaclab_physx.physics import PhysxManager
   import isaaclab.sim as sim_utils
   from pxr import UsdPhysics
   stage=sim_utils.get_current_stage()
   paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and '/belt' in str(p.GetPath())]
   view=PhysxManager.get_physics_sim_view().create_rigid_body_view(paths)
   self.native=view
   print('BELT_NATIVE_CONFIG',json.dumps(dict(paths=paths,masses=view.get_masses().numpy().tolist(),inertias=view.get_inertias().numpy().tolist(),coms=view.get_coms().numpy().tolist(),EA=EA,EI=EI,preload=PRELOAD,span=SPAN,dt=env.unwrapped.physics_dt)),flush=True)
   sim=env.unwrapped.sim;original=sim.step
   def sample_step(*args,**kw):
    out=original(*args,**kw)
    if len(self.samples)<12000:
     p=view.get_transforms().numpy();v=view.get_velocities().numpy()
     self.samples.append(dict(i=len(self.samples),middle=p[len(paths)//2].tolist(),max_speed=float(np.linalg.norm(v[:,:3],axis=1).max()),max_displacement=float(np.linalg.norm(p[:,:3]-(BASE+np.c_[DX*(np.arange(N)+.5),np.zeros(N),np.zeros(N)]),axis=1).max())))
    return out
   sim.step=sample_step;self.installed=True
  if self.n==1 or self.n%5==0:
   print('BELT_NATIVE',json.dumps(dict(step=self.n,transforms=self.native.get_transforms().numpy().tolist(),velocities=self.native.get_velocities().numpy().tolist())),flush=True)
  if self.n%5==0:
   print('BELT_SAMPLES',json.dumps(self.samples),flush=True);self.samples=[]
O=Observe();m.on_reset(O.reset);m.on_step(O.step)
def score(objects):return 0.
def success(objects):return False
def demo():return {'right':[Waypoint(1.1,(.26,-.22,1.0275),-.04695,'observe material gate')]}
