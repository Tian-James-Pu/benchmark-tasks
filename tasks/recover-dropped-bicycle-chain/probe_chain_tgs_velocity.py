"""One numerical convergence check: TGS with one velocity iteration.

Geometry, material, masses, joint response, drives and time step stay unchanged.
The baseline authored eight rigid-body velocity iterations; Nvidia recommends
very few for TGS. This test does not calibrate or validate the physical task.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_drive_native_audit import *

_original_chain = CHAIN.build
def one_velocity():
    from pxr import Sdf, Usd, UsdPhysics
    layer = Sdf.Layer.CreateAnonymous('.usda')
    layer.ImportFromString(_original_chain())
    stage = Usd.Stage.Open(layer)
    count = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            attr = prim.GetAttribute('physxRigidBody:solverVelocityIterationCount')
            assert attr.Get() == 8
            attr.Set(1)
            count += 1
    assert count == 212
    return layer.ExportToString()

CHAIN = replace(CHAIN, build=one_velocity)
OBJECTS = [CHAIN, DRIVE]
