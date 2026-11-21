"""Restore a balance's load path and demonstrate released zero/load/zero readings."""
from pathlib import Path
import sys,itertools,math,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import mechkit as m
from hardware import build,camera,TOP
import scene_hooks
import robot_contact_gate
from display import Electronics
from simready import SimReady
from repair_motion import demo
from tool_bounds import ToolBounds
SCENE='station';SUCCESS_HOLD_S=0.
INSTRUCTION='Restore accurate weighing, then leave the 100 g check weight beside the balance.'
camera()
OBJECTS=build()+[
 SimReady('spatula',pos=(.493,.01,TOP),size=(.243,.026,.013),mass=.040,
  package='instrument_spatula_a01',usd='sm_instrument_spatula_a01_01.usd',yaw=math.pi/2),
 SimReady('paper_cup',pos=(.42,-.28,TOP),size=(.080,.080,.110),mass=.006,
  package='dishware_cup_paper_a01',usd='sm_dishware_cup_paper_a01_01.usd')]

def bounds(state,low,high):
 corners=np.array(list(itertools.product(*zip(low,high))))
 v=state.to_world(corners)
 return v.min(0),v.max(0)
def gap(a,b):
 return float(np.linalg.norm(np.maximum(np.maximum(a[0]-b[1],b[0]-a[1]),0.)))
def connected_props(boxes,seeds):
 # Both supplied props have contact offsets <=20 mm; 40 mm covers their sum.
 # AABB closure is conservative and includes intervening grain/tool chains.
 held=set(seeds)
 while True:
  added={n for n,box in boxes.items() if n not in held and
   any(gap(box,boxes[k])<=.04000001 for k in held)}
  if not added:return held
  held.update(added)
def slow(s):return np.linalg.norm(s.lin_vel)<.002 and np.linalg.norm(s.ang_vel)<.05

class Monitor:
 def reset(self,env=None):
  self.e=Electronics();self.e.reset(env);self.steps=0;self.epoch=0
  self.zero_s=0.;self.load_s=0.;self.return_s=0.;self.zero_seen=False;self.load_seen=False
  self.fingers=[];self.fallen=False;self.last={};self.current=0.
  self.tools=ToolBounds()
  if env is not None:self.tools.reset(env)
 def step(self,env):
  self.steps+=1
  if robot_contact_gate.STEPS!=self.steps:
   raise RuntimeError("Contact gate must observe the current interval before scoring")
  contact=robot_contact_gate.LAST
  self.e.step(env)
  states={name:m.state(env,name) for name in ('balance','pan','verification_mass','grain')}
  b,p,w,g=[states[n] for n in ('balance','pan','verification_mass','grain')]
  if self.epoch!=self.e.epoch:
   self.epoch=self.e.epoch;self.zero_seen=False;self.load_seen=False
   self.zero_s=self.load_s=self.return_s=0.
  # Conservative world AABBs bound the standard helper's symmetric finger
  # samples, with an extra 5 mm thickness margin, then 20 mm release clearance.
  points=m.fingertips(env).reshape(4,6,3)
  self.fingers=[(q.min(0)-.005,q.max(0)+.005) for q in points]
  bb=bounds(b,(-.068,-.075,0.),(.068,.075,.0575))
  pb=bounds(p,(-.035,-.035,.0525),(.035,.035,.0575))
  wb=bounds(w,(-.011,-.011,0.),(.011,.011,.039))
  if min(bb[0][2],pb[0][2],wb[0][2])<TOP-.020:self.fallen=True
  hands_clear=all(gap(f,o)>.020 for f in self.fingers for o in (bb,pb,wb))
  # Supplied-object bounds near a finger can transmit indirect manipulation.
  # Unheld nearby bench objects are ignored; their mere proximity is harmless.
  tools=self.tools.boxes(env)
  tools["grain"]=bounds(g,(-.0015,)*3,(.0015,)*3)
  near_gripper={n:(n in contact["contact_objects"] or
    any(gap(f,t)<.002 for f in self.fingers)) for n,t in tools.items()}
  held_props=connected_props(tools,{n for n,v in near_gripper.items() if v})
  tools_clear=all(n not in held_props or all(gap(t,o)>.020 for o in (bb,pb,wb))
    for n,t in tools.items())
  pan_local=b.to_local(p.pos)[0]
  seated=bool(np.linalg.norm(pan_local[:2]-[0,.016])<.002 and
   abs(pan_local[2]-b.joint('loadcell'))<.0005 and p.tilt_deg()<1. and slow(p))
  on_bench=bool(abs(bb[0][2]-TOP)<.001 and b.tilt_deg()<1. and slow(b))
  weight_bench=bool(abs(wb[0][2]-TOP)<.0005 and w.tilt_deg()<2. and slow(w) and gap(wb,bb)>.020)
  weight_pan=bool(np.linalg.norm(p.to_local(w.pos)[0][:2])<.020 and
   abs(wb[0][2]-pb[1][2])<.0005 and w.tilt_deg()<2. and slow(w))
  r=self.e.readout
  valid=bool(not self.fallen and on_bench and seated and hands_clear and contact["direct_clear"] and tools_clear and
   self.e.on and not self.e.startup and r.stable and not r.error and r.reading_g is not None)
  zero=valid and weight_bench and abs(r.reading_g)<=.05
  loaded=valid and weight_pan and abs(r.reading_g-100.)<=.05
  self.zero_s=self.zero_s+.05 if zero else 0.
  if self.zero_s>=2.-1e-9:self.zero_seen=True
  self.load_s=self.load_s+.05 if loaded and self.zero_seen else 0.
  if self.load_s>=2.-1e-9:self.load_seen=True
  self.return_s=self.return_s+.05 if zero and self.load_seen else 0.
  self.current=0. if self.fallen else (1. if self.return_s>=2.-1e-9 else
   .5 if self.load_seen and on_bench and seated and self.e.on and not r.error else 0.)
  self.last={'display':r.text,'stable':r.stable,'offset_g':r.offset_g,'epoch':self.epoch,
   'robot_clear':contact['direct_clear'],'robot_contacts':contact['contact_objects'],
   'contact_samples':contact['samples'],'zero_s':self.zero_s,'load_s':self.load_s,'return_s':self.return_s,'hands_clear':hands_clear,
   'tool_bounds':{n:[a.tolist(),b.tolist()] for n,(a,b) in tools.items()},
   'tool_near_gripper':near_gripper,'connected_held_props':sorted(held_props),'tools_clear':tools_clear,'seated':seated,'on_bench':on_bench,'weight_pan':weight_pan,
   'weight_bench':weight_bench,'fallen':self.fallen,'score':self.current,
   'pan_pos':p.pos.tolist(),'body_pos':b.pos.tolist(),'weight_pos':w.pos.tolist(),'grain_pos':g.pos.tolist()}
  if self.steps==1 or self.steps%20==0:print('BALANCE_RECOVERY',self.steps,self.last,flush=True)
monitor=Monitor();monitor.reset();m.on_reset(monitor.reset);m.on_step(monitor.step)
m.on_step(scene_hooks.refresh_lcd_pixels)
def score(objects):return float(monitor.current)
def success(objects):return bool(monitor.current==1.)
