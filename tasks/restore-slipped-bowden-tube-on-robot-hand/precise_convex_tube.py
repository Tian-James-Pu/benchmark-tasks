"""Collision-cooking diagnostic preserving authored tube geometry and dynamics.

PhysX defaults convex-hull minimum thickness to1mm. The24 annular wedges
include submillimetre widths. Explicit10um minimum avoids imposing a larger
cooking thickness than the physical facets. This is not material calibration.
https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/schemas/physxschema.html
"""
from dataclasses import replace
from redistributed_carrier_tube import make_tube as original_make_tube


def make_tube(*args, **kwargs):
    tube = original_make_tube(*args, **kwargs)
    original_build = tube.part.build
    def build():
        source = original_build()
        source = source.replace('"PhysicsMeshCollisionAPI",',
            '"PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI",')
        source = source.replace('uniform token physics:approximation = "convexHull"',
            'uniform token physics:approximation = "convexHull"\n        float physxConvexHullCollision:minThickness = 0.00001')
        return source
    tube.part = replace(tube.part, build=build)
    tube.properties = dict(tube.properties, convex_hull_min_thickness_m=.00001)
    return tube
