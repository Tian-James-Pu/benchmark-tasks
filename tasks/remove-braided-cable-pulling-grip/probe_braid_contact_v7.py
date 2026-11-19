"""Final bounded solver diagnostic: v6 with zero TGS velocity iterations.

PhysX documents poor D6-drive behavior with TGS velocity iterations. Retain
all geometry, masses, spring laws, contact settings, 64 position iterations
and 4800 Hz. No hardware or robot validation follows from this coupon.
"""
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from probe_braid_contact_v6 import *

partitioned_build = braid.build


def build_zero_velocity_iterations():
    from pxr import Usd, Sdf
    layer = Sdf.Layer.CreateAnonymous('.usda')
    if not layer.ImportFromString(partitioned_build()):
        raise ValueError('Invalid partitioned braid USD')
    stage = Usd.Stage.Open(layer)
    changed = 0
    for prim in stage.Traverse():
        for name in ('physxRigidBody:solverVelocityIterationCount',
                     'physxArticulation:solverVelocityIterationCount'):
            attr = prim.GetAttribute(name)
            if attr and attr.HasAuthoredValue():
                attr.Set(0)
                changed += 1
    assert changed == 1662+36, changed
    return layer.ExportToString()


braid = replace(braid, build=build_zero_velocity_iterations)
OBJECTS = [braid, mandrel, platen]
