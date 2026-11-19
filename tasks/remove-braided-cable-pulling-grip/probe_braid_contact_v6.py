"""Numerical diagnostic: bounded reduced-coordinate rod blocks.

Retains v5 geometry, mass, constitutive assumptions and 4800 Hz timestep.
Each wire is partitioned into at most 48-link articulation trees; the original
D6 joints between blocks and the closing world clamp remain ordinary joints.
No mechanical joint is deleted. Self collision stays enabled with the same
adjacent-segment filters. This tests solver error, not hardware fidelity or
robot feasibility. Score remains zero.
"""
from dataclasses import replace
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from probe_braid_contact_v5 import *

BLOCK_LINKS = 48
original_build = braid.build


def build_partitioned_braid():
    from pxr import Usd, Sdf, UsdPhysics
    layer = Sdf.Layer.CreateAnonymous('.usda')
    if not layer.ImportFromString(original_build()):
        raise ValueError('Invalid source braid USD')
    stage = Usd.Stage.Open(layer)
    blocks = []
    for wi, points in enumerate(PATHS):
        n = len(points)-1
        for start in range(0, n, BLOCK_LINKS):
            stop = min(start+BLOCK_LINKS, n)
            if start == 0:
                root = stage.GetPrimAtPath(f'/P_braid/clamp_{wi}_a')
            else:
                root = stage.GetPrimAtPath(f'/P_braid/w{wi}_s{start:04d}')
                boundary = UsdPhysics.Joint.Get(stage, f'/P_braid/j{wi}_{start:04d}')
                boundary.CreateExcludeFromArticulationAttr(True)
            UsdPhysics.ArticulationRootAPI.Apply(root)
            root.AddAppliedSchema('PhysxArticulationAPI')
            for name, kind, value in [
                ('enabledSelfCollisions', Sdf.ValueTypeNames.Bool, True),
                ('solverPositionIterationCount', Sdf.ValueTypeNames.Int, 64),
                ('solverVelocityIterationCount', Sdf.ValueTypeNames.Int, 8),
            ]:
                root.CreateAttribute('physxArticulation:'+name, kind).Set(value)
            blocks.append({'wire':wi, 'start':start, 'stop':stop, 'root':str(root.GetPath())})
        UsdPhysics.Joint.Get(stage, f'/P_braid/clamp_{wi}_b').CreateExcludeFromArticulationAttr(True)
    print('BRAID_ARTICULATION_BLOCKS', json.dumps(blocks), flush=True)
    return layer.ExportToString()


braid = replace(braid, build=build_partitioned_braid)
OBJECTS = [braid, mandrel, platen]
