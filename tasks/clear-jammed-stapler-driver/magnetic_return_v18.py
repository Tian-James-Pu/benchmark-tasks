"""Inactive until imported: physical magnetic return approximation.

P361 BOM names a drive-cylinder/magnet assembly. Techtronic US9662777B2
explains annular magnetic piston retention; exact product mapping is inferred.
Force scale uses K&J R842's12.7/6.35/3.175mm ring and5.35lbf steel pull as a
reference, not as an identified P361 part. Equal-area cylinder field-squared
falloff approximates the unknown annular/finite-piston force-distance curve.
No power flag, tool event, pose assignment, or logical latch controls this force.
"""
import math,json
import numpy as np
import mechkit as m
from axial_observer_v15 import on_physics_step
STOP=.040
OD=.0127;ID=.00635;THICKNESS=.003175
RADIUS=math.sqrt((OD/2)**2-(ID/2)**2)
CONTACT_FORCE=5.35*4.4482216152605
SCALE=1.0
_N=0;_last=None

def field(g):
 return (g+THICKNESS)/math.sqrt((g+THICKNESS)**2+RADIUS**2)-g/math.sqrt(g*g+RADIUS**2)

def attraction(gap):
 return SCALE*CONTACT_FORCE*(field(max(0.,float(gap)))/field(0.))**2

@on_physics_step
def magnet_force(env,dt):
 global _N,_last
 s=m.state(env,'stapler');q=s.joint('driver');gap=STOP-q
 magnitude=attraction(gap);force=s.rotation()@np.array([magnitude,0.,0.])
 point=s.pos+s.rotation()@np.array([.100+q,0.,0.])
 if not np.isfinite(force).all() or magnitude<0 or magnitude>SCALE*CONTACT_FORCE+1e-9:
  raise RuntimeError('Invalid magnetic force')
 # Colocated equal/opposite forces conserve whole-tool momentum. Fixed native
 # spring and joint stops remain active and unchanged through battery removal.
 m.push(env,'stapler',force,at=point,body='driver')
 m.push(env,'stapler',-force,at=point,body='frame')
 _N+=1;_last={'q_m':q,'gap_m':gap,'attraction_N':magnitude,
  'spring_N':-200.*(q+.010),'force_world_N':force.tolist(),'point_world_m':point.tolist()}

@m.on_step
def report(env):
 if _N%240==0 and _last is not None:print('MAGNETIC_RETURN '+json.dumps(_last),flush=True)
