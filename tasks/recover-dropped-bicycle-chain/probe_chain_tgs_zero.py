"""TGS convergence follow-up with zero velocity iterations in the same island.

The one-iteration trial still injects large roller velocities before appreciable
anchor error. PhysX documents zero velocity iterations as sufficient for TGS
when initial geometric errors are small. Physical scene inputs stay unchanged.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_chain_tgs_velocity import *

def zero_velocity(part, articulation):
    original = part.build
    def build():
        from pxr import Sdf, Usd, UsdPhysics
        layer = Sdf.Layer.CreateAnonymous('.usda')
        layer.ImportFromString(original())
        stage = Usd.Stage.Open(layer)
        count = 0
        for prim in stage.Traverse():
            api = UsdPhysics.ArticulationRootAPI if articulation else UsdPhysics.RigidBodyAPI
            if prim.HasAPI(api):
                attr = prim.GetAttribute(('physxArticulation' if articulation else 'physxRigidBody')
                    + ':solverVelocityIterationCount')
                assert attr.Get() == 1
                attr.Set(0)
                count += 1
        assert count == (1 if articulation else 212)
        return layer.ExportToString()
    return replace(part, build=build)

CHAIN = zero_velocity(CHAIN, False)
DRIVE = zero_velocity(DRIVE, True)
OBJECTS = [CHAIN, DRIVE]
