"""Manufacturer meshes at original scale; generated with build_assets.py."""
from pathlib import Path
import json
from dataclasses import replace
import numpy as np
import mechkit as m
CACHE=Path.home()/'.cache'/'jetson-fan-recovery'
if not (CACHE/'geometry.npz').exists():
    raise RuntimeError('Run build_assets.py with cadquery installed before loading the scene.')
DATA=np.load(CACHE/'geometry.npz')
META=json.loads((CACHE/'sources.json').read_text())
def mesh(kind,index):return m.Mesh(DATA[f'{kind}_{index}_v'],DATA[f'{kind}_{index}_f'])
def sdf(name,shape):
    text=m.col_hull(name,shape).replace('"convexHull"','"sdf"').replace('"PhysicsMeshCollisionAPI"','"PhysicsMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"').replace('contactOffset = 0.004','contactOffset = 0.00005')
    return text.replace('uniform token subdivisionScheme', 'int physxSDFMeshCollision:sdfResolution = 256\n        int physxSDFMeshCollision:sdfSubgridResolution = 6\n        uniform token subdivisionScheme')
LOOKS={'black':m.Look(color=m.rgb(.09,.10,.11),roughness=.55),'green':m.Look(color=m.rgb(.06,.29,.12),roughness=.6),'silver':m.Look(color=m.rgb(.62,.64,.65),metallic=.7,roughness=.5),'gold':m.Look(color=m.rgb(.67,.49,.16),metallic=.6,roughness=.5),'nylon':m.Look(color=m.rgb(.86,.87,.81),roughness=.6)}
def jetson(pos):
    parts=[]
    # Full source geometry; black packages, green PCBs, silver connectors/heatsink.
    for i in range(META['jetson']['solid_count']):
        look='green' if i in (0,678) else 'silver' if i in (59,93,104,660,679) else 'black'
        parts.append((f'part_{i}',mesh('jetson',i),look))
    # Static triangle meshes preserve heatsink gaps and connector geometry.
    cols=[m.col_hull(f'col_{i}',mesh('jetson',i)).replace('"convexHull"','"none"').replace('contactOffset = 0.004','contactOffset = 0.00005') for i in [0,59,93,103,104,349,625,660,678,679]]
    return m.fixed('jetson',pos,parts,cols,LOOKS,friction=(.35,.25))
def fan(pos):
    frame,rotor=mesh('fan',0),mesh('fan',1)
    link=m.hinge('rotor','Z',(0,0,0),-float('inf'),float('inf'),[('impeller',rotor,'black')],[sdf('impeller_collision',rotor)],mass=.0027,spring=0.,damping=.000004,friction=0.,effort=.0004,armature=0.,gravity=True,com=(0,0,.0067446),inertia=(1.051865e-7,1.051865e-7,1.95085e-7))
    result=m.mechanism('fan',pos,[link],LOOKS,frame_parts=[('frame',frame,'black')],frame_colliders=[sdf('frame_collision',frame)],solver_iters=32,friction=(.35,.25))

    original_build=result.build
    def build():
        from pxr import Usd,Sdf
        layer=Sdf.Layer.CreateAnonymous('.usda');assert layer.ImportFromString(original_build())
        stage=Usd.Stage.Open(layer);rotor=stage.GetDefaultPrim().GetChild('rotor')
        rotor.CreateAttribute('physxRigidBody:maxAngularVelocity',Sdf.ValueTypeNames.Float).Set(36000.)
        rotor.CreateAttribute('physxRigidBody:angularDamping',Sdf.ValueTypeNames.Float).Set(0.)
        return layer.ExportToString()
    return replace(result,build=build)
