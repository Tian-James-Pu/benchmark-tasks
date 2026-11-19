"""Visual LCD and normal-mode physical key response; estimated firmware filter."""
import numpy as np
import mechkit as m
from hardware import GLYPHS
from readout import Readout
ROOT='/World/envs/env_0/balance/frame/visual/'
class Electronics:
 def reset(self,env=None):
  self.readout=Readout();self.on=True;self.power_down=False;self.startup=0;self.epoch=0
  self.last_offset=55.;self.steps=0
  if env is not None:
   raw=-20000*m.state(env,'balance').joint('loadcell')/9.81*1000
   self.readout.filtered_g=raw;self.readout.reading_g=round(raw-55.,2)
   self.paint(env)
 def step(self,env):
  self.steps+=1;b=m.state(env,'balance');pressed=b.joint('power')<-.00015
  if pressed and not self.power_down:
   self.on=not self.on;self.epoch+=1
   if self.on:self.startup=40
  self.power_down=pressed
  if self.on:
   self.readout.update(-20000*b.joint('loadcell')/9.81*1000,b.joint('rezero')<-.00015 and self.startup==0)
   if self.startup:
    self.startup=max(1,self.startup-1)
    if self.startup==1 and self.readout.stable:
     # Published normal startup zeros the displayed load. Its exact accepted
     # power-on range is not published; the approximation accepts stable loads.
     self.readout.offset_g=self.readout.filtered_g;self.readout.error=None
     self.readout.reading_g=0.;self.startup=0
   if self.readout.offset_g!=self.last_offset:self.epoch+=1;self.last_offset=self.readout.offset_g
  self.paint(env)
 def paint(self,env):
  text='888888' if self.startup else self.readout.text.replace('.','').rjust(6)[-6:]
  for i,ch in enumerate(text):
   for seg in 'abcdefg':m.set_visible(ROOT+f'lcd_{i}_{seg}',self.on and seg in GLYPHS.get(ch,''))
  m.set_visible(ROOT+'lcd_decimal',self.on and not self.readout.error)
  m.set_visible(ROOT+'lcd_stable',self.on and not self.startup and self.readout.stable and not self.readout.error)
  m.set_visible(ROOT+'unit_g',self.on)
  # The bubble is a visual indicator, not a physical actuator. Its limited
  # translation follows gravity projected into the tilted balance frame.
  import isaaclab.sim as sim_utils
  from pxr import UsdGeom,Gf
  prim=sim_utils.get_current_stage().GetPrimAtPath(ROOT+'level_bubble')
  xform=UsdGeom.Xformable(prim)
  op=next((o for o in xform.GetOrderedXformOps() if o.GetOpName()=='xformOp:translate'),None)
  if op is None:op=xform.AddTranslateOp()
  g=np.array([0.,0.,1.])@m.state(env,'balance').rotation()
  delta=np.clip(g[:2]*.04,-.003,.003)
  op.Set(Gf.Vec3d(float(delta[0]),float(delta[1]),float(delta[1])/3))
