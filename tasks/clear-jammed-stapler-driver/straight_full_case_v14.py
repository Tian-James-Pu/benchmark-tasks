"""Matched straight1/2inch staple; same29.55mm centerline and mass."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from full_case_v14 import *
p,c=[],[]
box(p,c,'crown',(.001,.0012,.00515),(.006,0,0),body='straight')
for side in (-1,1):
 box(p,c,'leg'+str(side+1),(.0122,.001,.0012),(-.0001,0,side*.002575),body='straight')
wire=m.free('bent_staple',tuple(NOSE),p,c,LOOKS,mass=.000278367,
 friction=(.4,.3),torsional_patch=None,solver_iters=64)
OBJECTS=[wire if x.name=='bent_staple' else x for x in OBJECTS]
