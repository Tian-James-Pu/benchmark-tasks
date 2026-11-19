"""Closed empty magazine and battery: native assembly diagnostic, score0."""
import sys,pathlib,json
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from full_case_v20 import *
import magnetic_return_v18
@m.on_step
def battery_status(env):
 if _tick[0]%5==0:
  a=m.state(env,'stapler');b=m.state(env,'battery')
  print('BATTERY_ASSEMBLY '+json.dumps({'step':_tick[0],'battery_in_frame':(a.rotation().T@(b.pos-a.pos)).tolist(),'clips':[b.joint('clip_0'),b.joint('clip_2')],'cover':a.joint('cover'),'release':a.joint('release'),'driver':a.joint('driver')}),flush=True)
 _tick[0]+=1
_tick=[0]
def demo_script(ctx):yield from ctx.left.hold(2.)
