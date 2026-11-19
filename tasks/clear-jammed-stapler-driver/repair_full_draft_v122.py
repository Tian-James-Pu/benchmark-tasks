"""Inactive full battery-isolation/service composition. No validation claim.

Same component hardware. OEM total-mass correction and final scorer remain
outstanding. The native battery, driver and wire never receive pose assignments.
"""
import sys,pathlib,itertools
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import repair_short_draft_v121 as repair
from repair_short_draft_v121 import *
import battery_motion_v58 as battery
from dataclasses import replace
OBJECTS=[replace(x,pos=tuple(NOSE),start_quat=UPRIGHT_Q) if x.name in ('battery','stapler','bent_staple') else x for x in OBJECTS]
_STAGE='battery_isolation'
def stem_socket_gap():
 if battery._BAT is None:return None
 P,B=base.frame();b=battery._BAT
 stem=b.to_world(np.array(list(itertools.product([.2015,.2305],[.172,.222],[-.013,.013]))))
 socket=np.array(list(itertools.product([.2005,.2315],[.170,.222],[-.014,.014])))@B.T+P
 axes=np.concatenate([B.T,b.rotation().T,np.cross(B.T[:,None,:],b.rotation().T[None,:,:]).reshape(-1,3)]);n=np.linalg.norm(axes,axis=1);axes=axes[n>1e-8]/n[n>1e-8,None];x=stem@axes.T;y=socket@axes.T
 return float(np.maximum(y.min(0)-x.max(0),x.min(0)-y.max(0)).max())
def demo_script(ctx):
 global _STAGE
 base._CTX=ctx;L,R=ctx.left,ctx.right;O=-.045;LH=-.020
 yield from battery.demo_script(ctx)
 gap=stem_socket_gap()
 print('FULL_PACK_ISOLATION_GATE '+json.dumps({'stem_socket_separating_gap_m':gap,'scope':'Physical stem clearance; no stored-load changes'}),flush=True)
 if gap is not None and gap<.002:return
 yield from ctx.both(L.home(3.),R.home(3.))
 _STAGE='bench_tip';feed._STAGE='tip'
 yield from feed.prior.demo_script(ctx)
 yield from R.home(2.)
 P,B=(np.array([.05,.0675,TOP+.0405]),np.eye(3)) if base._POSES is None else base.frame()
 tilt=float(np.degrees(np.arccos(np.clip(B[2,2],-1,1))))
 if tilt>12.:print('FULL_REPAIR_STOP '+json.dumps({'reason':'Case did not settle on side','tilt_deg':tilt}),flush=True);return
 _STAGE='reposition';feed._STAGE='housing_grasp'
 lp=P+B@feed.GP;lr=B@feed.GR
 yield from L.move(lp+np.array([0,0,.080]),1.5,O,rot=lr)
 yield from L.move(lp,1.5,O,rot=lr)
 yield from L.grip(LH,.8)
 yield from L.hold(.5)
 if feed._VIEW is not None and feed._LEFT_MIN<10.:print('FULL_REPAIR_STOP '+json.dumps({'reason':'Housing grasp missed','min_tip_N':feed._LEFT_MIN}),flush=True);return
 P,B=(P,B) if base._POSES is None else base.frame();ap=L.pos if base._POSES is None else L.measured_pos;ar=L.rot if base._POSES is None else L.measured_rot;gp=B.T@(ap-P);gr=B.T@ar
 yield from L.move(L.pos+np.array([0,0,.040]),1.5,LH,rot=L.rot)
 end=np.array([.18,-.20,TOP+.0807]);yield from L.move(end+gp,3.,LH,rot=gr)
 yield from L.move(end+gp-np.array([0,0,.040]),1.5,LH,rot=gr)
 yield from L.grip(O,.6)
 yield from L.move(L.pos+np.array([0,.100,.040]),1.,O,rot=L.rot)
 yield from L.hold(1.)
 yield from L.home(2.)
 _STAGE='isolated_repair'
 yield from repair.demo_script(ctx)
