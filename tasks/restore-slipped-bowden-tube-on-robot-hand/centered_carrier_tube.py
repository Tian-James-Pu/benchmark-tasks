"""Center the redistributed tube's body frames without changing its physics.

The earlier centering control used the original tiny-carrier masses and
uncorrected cooked convex hulls. This helper applies the same coordinate
change to the current carrier representation. It is a diagnostic, not a
material calibration or evidence of a working assembled hand.
"""
from dataclasses import replace
import numpy as np
from redistributed_carrier_tube import make_tube as original_make_tube


def make_tube(name, position, points, **kwargs):
    tube = original_make_tube(name, position, points, **kwargs)
    centers = {body: values['center'].copy()
               for body, values in tube.mass_properties.items()}
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
            xf = UsdGeom.Xformable(prim)
            if xf.GetOrderedXformOps():
                raise ValueError('Expected common-origin tube body frames')
            xf.AddTranslateOp().Set(Gf.Vec3d(*(center-base)))
            UsdPhysics.MassAPI(prim).GetCenterOfMassAttr().Set(Gf.Vec3f(0.))
            for child in Usd.PrimRange(prim):
                if child.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(child)
                    if UsdGeom.Xformable(child).GetOrderedXformOps():
                        raise ValueError('Center frames before scaling convex points')
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
    # Energy instrumentation consumes coordinates in each body's new frame.
    # Copy records: original_build closes over the original mass properties.
    tube.mass_properties = {body: dict(values, center=np.zeros(3))
                            for body, values in tube.mass_properties.items()}
    return tube
