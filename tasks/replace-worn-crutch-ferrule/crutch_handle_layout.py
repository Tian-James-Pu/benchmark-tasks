"""Alternative ordinary bench orientation exposing the original handgrip.

The geometry, mass and contact properties are unchanged. Only the initial
bench placement changes; the authentic crosswise handgrip can counterhold
axial insertion through jaw contact instead of shaft friction.
"""
import numpy as np
from scipy.spatial.transform import Rotation
import mechkit as m
from crutch import build,TOP
AXIS=np.array([.4,np.sqrt(.84),0.])
R=np.column_stack(([0.,0.,1.],[AXIS[1],-AXIS[0],0.],AXIS))
Q=tuple(Rotation.from_matrix(R).as_quat())
POS=np.array([.15,-.24,TOP+.0445])
def part(pos=POS):
    return m.Part('crutch',pos=tuple(pos),size=(.12,.20,.90),mass=.388579,role='free',build=build,start_quat=Q,friction=(.45,.35),solver_iters=64)
