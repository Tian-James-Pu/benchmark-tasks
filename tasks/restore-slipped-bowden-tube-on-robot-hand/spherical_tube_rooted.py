"""Explicit floating-base root for the centered D6 tube diagnostic.

With an articulation root on the container, PhysX may select a middle body.
Arena's initial root pose then places that body's frame at the requested pose.
Once body frames are centered separately, choosing another root changes the
runtime layout despite identical authored world geometry. Root the floating
tree explicitly on frame; fixed-base trees already have their world joint.
"""
from dataclasses import replace
from spherical_tube import make_tube as original_make_tube, energy


def make_tube(*args, **kwargs):
    tube = original_make_tube(*args, **kwargs)
    if kwargs.get('fixed_root',False):
        return tube
    original_build = tube.part.build

    def build():
        from pxr import Usd,Sdf,UsdPhysics
        layer = Sdf.Layer.CreateAnonymous('.usda')
        if not layer.ImportFromString(original_build()):
            raise ValueError('Invalid D6 tube USD')
        stage = Usd.Stage.Open(layer)
        root = stage.GetDefaultPrim()
        frame = root.GetChild('frame')
        settings = [(a.GetName(),a.GetTypeName(),a.Get()) for a in root.GetAttributes()
                    if a.GetName().startswith('physxArticulation:')]
        root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        root.RemoveAppliedSchema('PhysxArticulationAPI')
        UsdPhysics.ArticulationRootAPI.Apply(frame)
        frame.AddAppliedSchema('PhysxArticulationAPI')
        for name,kind,value in settings:
            frame.CreateAttribute(name,kind).Set(value)
            root.RemoveProperty(name)
        return layer.ExportToString()

    tube.part = replace(tube.part,build=build)
    tube.properties = dict(tube.properties,explicit_floating_root='frame')
    return tube
