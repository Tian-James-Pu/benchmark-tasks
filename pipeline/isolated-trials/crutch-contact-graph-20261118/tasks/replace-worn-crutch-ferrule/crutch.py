"""Original-size USDCraft crutch; replace its solid fixed tip with a real bore.

The source contains a 22 mm shaft ending at z=30 mm inside a solid fixed tip.
The 22 mm tube is continued to z=8 mm, the rebuilt ferrule's seating stop.
The continuation is disclosed; no external grip or support is added.
"""
from dataclasses import replace
import numpy as np
import mechkit as m
from asset import source_path

TOP=.7475
AXIS=np.array([.60,.80,0.])
# Native +X points upward, so the cuff and grip rest on their original sides.
R=np.column_stack(([0.,0.,1.],[.8,-.6,0.],AXIS))
Q=tuple(__import__('scipy').spatial.transform.Rotation.from_matrix(R).as_quat())
POS=np.array([.15,-.20,TOP+.0445])

def build():
    from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf
    src=Usd.Stage.Open(str(source_path()))
    layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(src.Flatten().ExportToString())
    stage=Usd.Stage.Open(layer)
    stage.RemovePrim('/Asset/Joints');stage.RemovePrim('/Asset/Links/tip')
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):p.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if p.HasAPI(UsdPhysics.MassAPI):p.RemoveAPI(UsdPhysics.MassAPI)
        p.RemoveAppliedSchema('PhysxArticulationAPI')
    root=stage.GetDefaultPrim()
    UsdPhysics.RigidBodyAPI.Apply(root)
    mass=UsdPhysics.MassAPI.Apply(root);mass.CreateMassAttr(.388579)
    # A single convex hull around the source's bent shaft fills empty space.
    # Analytic cylinders/capsules preserve its source cross-section and bend.
    mesh=UsdGeom.Mesh(stage.GetPrimAtPath('/Asset/Links/shaft/Visuals/shaft_tube/Geom/mesh'))
    a=np.asarray(mesh.GetPointsAttr().Get());centres=a[:3616].reshape(-1,32,3).mean(1)
    stage.RemovePrim('/Asset/Links/shaft/Collisions/shaft_tube')
    def col(p):
        UsdPhysics.CollisionAPI.Apply(p.GetPrim());p.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
        p.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.00005)
        p.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
    # Same z8..500mm cylindrical union, split above the45mm ferrule contact region.
    # This only conditions the collision representation; full source geometry remains.
    for name,low,high in [('lower_tube_c',.008,.068),('lower_tube_remainder_c',.068,.500)]:
        c=UsdGeom.Cylinder.Define(stage,'/Asset/Links/shaft/'+name)
        c.CreateRadiusAttr(.011);c.CreateHeightAttr(high-low);c.CreateAxisAttr('Z')
        c.AddTranslateOp().Set(Gf.Vec3d(0,0,(low+high)/2));c.CreateVisibilityAttr('invisible');col(c)
    path=np.vstack(([0,0,.5],centres[centres[:,2]>.5][::5],centres[-1]))
    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
        d=b-a
        if np.linalg.norm(d)<1e-6:continue
        c=UsdGeom.Capsule.Define(stage,f'/Asset/Links/shaft/upper_tube_c{i}')
        c.CreateRadiusAttr(.011);c.CreateHeightAttr(float(np.linalg.norm(d)));c.CreateAxisAttr('Z')
        c.AddTranslateOp().Set(Gf.Vec3d(*((a+b)/2)))
        c.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*(d/np.linalg.norm(d)))).GetQuat()))
        c.CreateVisibilityAttr('invisible');col(c)
    # Visual continuation: actual hollow aluminum tube with a 2 mm wall.
    v=m.lathe([(.011,.030),(.011,.008),(.009,.008),(.009,.030)],32)
    ext=UsdGeom.Mesh.Define(stage,'/Asset/Links/shaft/tube_extension')
    ext.CreatePointsAttr([Gf.Vec3f(*x) for x in v.v]);ext.CreateFaceVertexCountsAttr([3]*len(v.f))
    ext.CreateFaceVertexIndicesAttr(v.f.reshape(-1).tolist());ext.CreateSubdivisionSchemeAttr('none')
    ext.CreateDisplayColorAttr([(.65,.66,.67)])
    return layer.ExportToString()

def part(pos=POS):
    return m.Part('crutch',pos=tuple(pos),size=(.12,.20,.90),mass=.388579,role='free',build=build,start_quat=Q,friction=(.45,.35),solver_iters=64)
