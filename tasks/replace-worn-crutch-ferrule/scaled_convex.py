"""Preserve world geometry while cooking small convex meshes at larger scale.

Tiny source coordinates can produce severely distorted PhysX hulls. Author
collider vertices1000times larger with a0.001 child transform. Body frames,
visual meshes, joints, mass, inertia, materials and contact offsets are retained.
USD imports are deferred until build, after the simulator selects its libraries.
"""
from dataclasses import replace


def scaled_convex_part(part):
    original_build = part.build
    def build():
        import numpy as np
        from pxr import Gf, Sdf, Usd, UsdGeom
        layer = Sdf.Layer.CreateAnonymous('.usda')
        if not layer.ImportFromString(original_build()):
            raise ValueError('Invalid input geometry')
        stage = Usd.Stage.Open(layer)
        for prim in stage.Traverse():
            if prim.GetTypeName() != 'Mesh' or prim.GetAttribute('physics:approximation').Get() != 'convexHull':
                continue
            mesh = UsdGeom.Mesh(prim)
            transform = UsdGeom.Xformable(prim)
            if transform.GetOrderedXformOps():
                raise ValueError('Scale correction expects point-authored collider without existing transforms')
            original = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            mesh.GetPointsAttr().Set([Gf.Vec3f(*v) for v in original*1000.])
            transform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(.001))
            restored = np.asarray(mesh.GetPointsAttr().Get(),dtype=float)*.001
            if np.max(np.abs(restored-original)) > 1e-7:
                raise ValueError('Collider world geometry changed beyond0.1micrometre roundoff')
        return layer.ExportToString()
    return replace(part, build=build)
