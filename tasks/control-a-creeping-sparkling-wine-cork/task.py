"""Feasibility prototype: independent compliant cork under gas pressure. NOT submission ready."""
from pathlib import Path
import sys, math, json, os
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
import mechkit as m
from simready import SimReady
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
SCENE='station'; TOP=TABLE_TOP[SCENE]; SUCCESS_HOLD_S=3.
INSTRUCTION='Open the sparkling-wine bottle and leave the bottle and cork on the table.'
LOOK={'glass':m.Look(color=m.rgb(.10,.23,.10),roughness=.3), 'cork':m.Look(color=m.rgb(.66,.47,.26)), 'label':m.Look(color=m.rgb(.8,.72,.45))}
P=np.array([.24,-.15,TOP+.04]); MOUTH=.268; Q=(-.5,0.,0.,math.sqrt(.75)); R=m.quat_matrix(Q); C0=P+R@np.array([0,0,MOUTH])
def bottle():
 a=np.load(Path(__file__).with_name('bottle_mesh.npz'))
 parts=[('simready_bottle',m.Mesh(a['v'],a['f']),'glass')]
 cols=[m.col_cylinder('body',.040,.17,(0,0,.085)),m.col_hull('shoulder',m.lathe([(.04,.17),(.015,.225),(.015,.237),(0,.237)]))]
 # Hollow neck: eight planar glass sectors, 18 mm minimum bore, 28 mm outside.
 for i in range(8):
  theta=i*45
  wall=m.box((.005,.010,.031),(.0115,0,.2525)).turned(theta,'z')
  cols.append(m.col_hull('neck'+str(i),wall));parts.append(('neck'+str(i),wall,'glass'))
 return m.free('bottle',P,parts,cols,LOOK,mass=1.4,friction=(.35,.25),solver_iters=64,com=(0,0,.12),start_quat=Q)
def cork():
 links=[];start=[]
 for ring,z in enumerate((-.020,-.012,-.004)):
  for i,(axis,sign) in enumerate((('X',1),('X',-1),('Y',1),('Y',-1))):
   c=np.array([0.,0.,z]);c[0 if axis=='X' else 1]=sign*.010
   size=(.004,.009,.007) if axis=='X' else (.009,.004,.007)
   name=f'pad{ring}_{i}'; limits=(-.0045,0) if sign==1 else (0,.0045)
   links.append(m.slider(name,axis,*limits,parts=[(name,m.box(size,c),"cork")],colliders=[m.col_box(name+'_c',size,c)],mass=.0003,com=tuple(c),inertia=tuple(.0003/12*(sum(t*t for t in size)-v*v) for v in size),spring=25000,damping=2.,friction=0.,effort=80,gravity=True,armature=.000001))
   start.append((name,-sign*.0035))
 parts=[('head',m.cylinder(.016,.018,(0,0,.020)),'cork'),('stem',m.cylinder(.0085,.036,(0,0,-.007)),'cork')]
 cols=[m.col_cylinder('head_c',.016,.018,(0,0,.020))]
 return m.mechanism('cork',C0,links,LOOK,frame_parts=parts,frame_colliders=cols,free_mass=.004,frame_inertia=(1e-6,1e-6,1e-6),friction=(.35,.25),solver_iters=64,start=start,start_quat=Q)
OBJECTS=[bottle(),cork(),m.fixed('support',(.24,0,TOP),[('upper',m.box((.10,.04,.08),(0,-.02,.04)),'label'),('stop',m.box((.10,.025,.065),(0,-.19,.0325)),'label')],[m.col_box('upper_c',(.10,.04,.08),(0,-.02,.04)),m.col_box('stop_c',(.10,.025,.065),(0,-.19,.0325))],LOOK),SimReady('glass',(.48,.30,TOP),(.07,.07,.16),mass=.15,package='dishware_glass_wine_a01',usd='sm_dishware_glass_wine_a01_01.usd'),SimReady('mug',(.48,-.30,TOP),(.10,.08,.09),mass=.3,package='dishware_mug_a01',usd='sm_dishware_mug_a01_01.usda')]
class Trial:
 def __init__(self): self.reset()
 def reset(self,env=None):
  self.vented=False;self.harm=False;
  if hasattr(self,"vent_tensor"):del self.vent_tensor
  self.steps=0;self.result=0.;self.maxspeed=0.;self.lastlog=-1
  if env is not None:
   base=env.unwrapped
   if not getattr(base.action_manager,'_cork_pressure',False):
    original=base.action_manager.apply_action
    def apply():
     self.physics(env)
     return original()
    base.action_manager.apply_action=apply;base.action_manager._cork_pressure=True
 def physics(self,env):
  import torch
  base=env.unwrapped;b=base.scene['bottle'];c=base.scene['cork']
  ci=c.body_names.index('frame')
  bp=b.data.root_link_pose_w.torch[0];cp=c.data.body_link_pose_w.torch[0,ci]
  x,y,z,w=bp[3:7];axis=torch.stack((2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)))
  delta=cp[:3]-bp[:3];height=(delta*axis).sum();radial2=(delta*delta).sum()-height*height
  if not hasattr(self,'vent_tensor'):self.vent_tensor=torch.zeros((),dtype=torch.bool,device=b.device)
  self.vent_tensor=self.vent_tensor | (height-MOUTH>=.025) | (radial2>.020**2)
  area=math.pi*.009**2;dx=torch.clamp(height-MOUTH,min=0)
  force=torch.clamp(4.7093e5*(25e-6/(25e-6+area*dx))**1.3-1.013e5,min=0)*area
  force=torch.where(self.vent_tensor,torch.zeros_like(force),force)
  if os.environ.get('CORK_NO_PRESSURE'):force=force*0
  f=(axis*force).reshape(1,1,3);zero=torch.zeros_like(f)
  c.permanent_wrench_composer.set_forces_and_torques_index(forces=f,torques=zero,body_ids=[ci],is_global=True)
  b.permanent_wrench_composer.set_forces_and_torques_index(forces=-f,torques=zero,body_ids=[0],is_global=True)
 def step(self,env):
  self.steps+=1;b=m.state(env,'bottle');c=m.state(env,'cork')
  self.vented=bool(self.vent_tensor.item()) if hasattr(self,'vent_tensor') else False
  self.maxspeed=max(self.maxspeed,float(np.linalg.norm(c.lin_vel-b.lin_vel)))
  if self.steps%20==0:print('CORK_DIAG',self.steps,{'cork':c.pos.tolist(),'bottle':b.pos.tolist(),'maxspeed':self.maxspeed,'vented':self.vented,'harm':self.harm,'joints':c.joints},flush=True)
  self.result=0.
T=Trial();m.on_reset(T.reset);m.on_step(T.step)
def score(objects):return float(T.result)
def success(objects):return False
def demo():
 return {'left':[Waypoint(12,(.10,.35,TOP+.25),1.,'wait')],'right':[Waypoint(12,(.10,-.35,TOP+.25),1.,'wait')]}
