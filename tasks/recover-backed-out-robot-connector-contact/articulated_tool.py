"""Load a SimReady tool while retaining its authored bodies and joints.

The ordinary SimReady prop loader merges articulated tools. This diagnostic
loader instead references the original asset, keeps its materials and SDF
collisions, and exposes its joints to Arena. No meshes or textures are copied
into the submission folder. Source spring values are not measured hardware data.
"""

import math

import mechkit as m
import simready as sr


def tool(name, pos, package, usd, joints, yaw_deg=0., root_offset=(0., 0., 0.)):
    """joints is [(name, lower degrees, upper degrees), ...] from source USD."""
    def build():
        from pxr import Usd, UsdGeom, UsdPhysics, Sdf

        source = sr.fetch(package, usd)
        source_stage = Usd.Stage.Open(str(source))
        source_root = source_stage.GetDefaultPrim()
        if not source_root:
            source_root = source_stage.GetPrimAtPath('/RootNode')
        if not source_root or UsdGeom.GetStageMetersPerUnit(source_stage) != 1.:
            raise ValueError('Expected the inspected metre-scale RootNode asset')
        layer = Sdf.Layer.CreateAnonymous('articulated_tool.usda')
        layer.subLayerPaths = [str(source)]
        stage = Usd.Stage.Open(layer)
        # defaultPrim metadata belongs to the root layer, not its sublayers.
        root = stage.GetPrimAtPath(source_root.GetPath())
        stage.SetDefaultPrim(root)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        # Uninstance the body roots so collision settings can be overridden.
        for prim in list(stage.Traverse()):
            if prim.IsInstance():
                prim.SetInstanceable(False)
        guides = [p for p in stage.Traverse() if p.GetName() == 'Grasp']
        for prim in guides:
            prim.SetActive(False)
        bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        for body in bodies:
            body.AddAppliedSchema('PhysxContactReportAPI')
            body.CreateAttribute('physxContactReport:threshold', Sdf.ValueTypeNames.Float).Set(0.)
        source_joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
        if len(bodies) < 2 or {p.GetName() for p in source_joints} != {j[0] for j in joints}:
            raise ValueError('Source tool body/joint structure changed')
        UsdPhysics.ArticulationRootAPI.Apply(root)
        root.AddAppliedSchema('PhysxArticulationAPI')
        for name_, kind, value in (
                ('enabledSelfCollisions', Sdf.ValueTypeNames.Bool, False),
                ('solverPositionIterationCount', Sdf.ValueTypeNames.Int, 64),
                ('solverVelocityIterationCount', Sdf.ValueTypeNames.Int, 8)):
            root.CreateAttribute('physxArticulation:'+name_, kind).Set(value)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.AddAppliedSchema('PhysxCollisionAPI')
                prim.CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.00005)
                prim.CreateAttribute('physxCollision:restOffset', Sdf.ValueTypeNames.Float).Set(0.)
        for prim in source_joints:
            joint = UsdPhysics.Joint(prim)
            # Source targets point at mesh children of the actual rigid bodies.
            # Resolve explicitly to the rigid-body ancestor, preserving its frame.
            for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()):
                targets = rel.GetTargets()
                if len(targets) != 1:
                    raise ValueError('Expected a joint between two tool bodies')
                body = stage.GetPrimAtPath(targets[0])
                while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                    body = body.GetParent()
                if not body:
                    raise ValueError('Joint target lacks a rigid-body ancestor')
                rel.SetTargets([body.GetPath()])
        parents = {str(UsdPhysics.Joint(p).GetBody0Rel().GetTargets()[0]) for p in source_joints}
        children = {str(UsdPhysics.Joint(p).GetBody1Rel().GetTargets()[0]) for p in source_joints}
        roots = parents-children
        if len(roots) != 1:
            raise ValueError('Expected one source articulation root body')
        transform = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(roots.pop()))
        if max(abs(float(transform[3][i])-root_offset[i]) for i in range(3)) > 1e-8:
            raise ValueError('Source root-body offset differs from inspected geometry')
        # Arena restores the physical root body at Part.pos, not the USD asset
        # origin. Rebase all geometry together and compensate Part.pos below,
        # preserving the requested source-origin placement through reset.
        # These source bodies reset the parent transform stack, so translating
        # their Geometry parent would have no effect. Each inspected body has
        # identity rotation; shift its own frame while keeping child meshes and
        # joint-local anchors unchanged.
        cache = UsdGeom.XformCache()
        for body in bodies:
            matrix = cache.GetLocalToWorldTransform(body)
            if any(abs(float(matrix[i][j])-(1. if i == j else 0.)) > 1e-8
                   for i in range(3) for j in range(3)):
                raise ValueError('Source body rotation needs an explicit rebase')
            UsdGeom.Xformable(body).AddTranslateOp(opSuffix='root_rebase').Set(tuple(-x for x in root_offset))
        return layer.ExportToString()

    # None preserves the drive stiffness, damping and effort limits read from
    # the asset, rather than silently replacing its springs with motor gains.
    angle = math.radians(yaw_deg)
    dx, dy, dz = root_offset
    root_pos = (pos[0]+math.cos(angle)*dx-math.sin(angle)*dy,
                pos[1]+math.sin(angle)*dx+math.cos(angle)*dy, pos[2]+dz)
    return m.Part(name, pos=root_pos, size=(.24, .12, .03), role='jointed',
                  build=build, yaw_deg=yaw_deg,
                  drives=tuple((n, (None, None, None, 0., 0.)) for n, _, _ in joints),
                  limits=tuple((n, math.radians(lo), math.radians(hi)) for n, lo, hi in joints))
