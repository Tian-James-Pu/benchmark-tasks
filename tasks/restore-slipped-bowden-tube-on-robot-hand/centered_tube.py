"""Equivalent tube with each body frame centered at its center of mass.

This diagnostic changes coordinate representation only. The original tube puts
all tiny bodies at a common CAD origin, up to 120 mm from their centers of mass.
Centered frames reduce the dynamic range of local joint and inertia quantities.
Geometry, material, masses, contacts and joint targets are preserved.
"""
from dataclasses import replace

import numpy as np

from compliant_tube import make_tube as original_make_tube


def make_tube(name, position, points, **kwargs):
    tube = original_make_tube(name, position, points, **kwargs)
    centers = {}
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        centers['frame' if i == 0 else f'segment_{i}'] = (a+b)/2
        if i:
            centers[f'bend_{i}'] = np.asarray(a)
    base = centers['frame']
    original_build = tube.part.build

    def build():
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
        layer = Sdf.Layer.CreateAnonymous('.usda')
        if not layer.ImportFromString(original_build()):
            raise ValueError('Invalid tube USD')
        stage = Usd.Stage.Open(layer)
        root = stage.GetDefaultPrim()
        for body, center in centers.items():
            prim = root.GetChild(body)
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*(center-base)))
            UsdPhysics.MassAPI(prim).GetCenterOfMassAttr().Set(Gf.Vec3f(0.))
            for child in Usd.PrimRange(prim):
                if child.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(child)
                    coordinates = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
                    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(
                        (coordinates-center).astype(np.float32)))
        for prim in root.GetChildren():
            if not prim.IsA(UsdPhysics.RevoluteJoint):
                continue
            joint = UsdPhysics.Joint(prim)
            for attr, relation in ((joint.GetLocalPos0Attr(), joint.GetBody0Rel()),
                                   (joint.GetLocalPos1Attr(), joint.GetBody1Rel())):
                body = relation.GetTargets()[0].name
                attr.Set(Gf.Vec3f(*(np.asarray(attr.Get())-centers[body])))
        return layer.ExportToString()

    tube.part = replace(tube.part, pos=tuple(np.asarray(position)+base), build=build)
    tube.body_centers = centers
    return tube
