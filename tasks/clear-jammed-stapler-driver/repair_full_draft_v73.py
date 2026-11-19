"""Inactive full-battery composition candidate. No success or scorer claim.

Restores the ordinary attached initial battery pose while preserving the exact
mechanism. Battery removal only changes its physical position, never stored load.
"""
import sys,pathlib,itertools
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import repair_service_v72 as service
from repair_service_v72 import *
import battery_motion_v58 as battery
from dataclasses import replace
OBJECTS=[replace(x,pos=tuple(NOSE),start_quat=UPRIGHT_Q) if x.name=='battery' else x for x in OBJECTS]

def stem_socket_gap():
 if battery._BAT is None:return None
 a,b=base.frame(),battery._BAT;P,B=a
 stem=b.to_world(np.array(list(itertools.product([.2015,.2305],[.172,.222],[-.013,.013]))))
 socket=np.array(list(itertools.product([.2005,.2315],[.170,.222],[-.014,.014])))@B.T+P
 axes=np.concatenate([B.T,b.rotation().T,np.cross(B.T[:,None,:],b.rotation().T[None,:,:]).reshape(-1,3)]);n=np.linalg.norm(axes,axis=1);axes=axes[n>1e-8]/n[n>1e-8,None];x=stem@axes.T;y=socket@axes.T
 return float(np.maximum(y.min(0)-x.max(0),x.min(0)-y.max(0)).max())

def demo_script(ctx):
 base._CTX=ctx
 yield from battery.demo_script(ctx)
 gap=stem_socket_gap()
 print('FULL_PACK_ISOLATION_GATE '+json.dumps({'stem_socket_separating_gap_m':gap,'scope':'Physical complete stem clearance; no spring or magnet state changes'}),flush=True)
 if gap is not None and gap<.002:return
 yield from ctx.both(ctx.left.home(3.),ctx.right.home(3.))
 yield from service.demo_script(ctx)
