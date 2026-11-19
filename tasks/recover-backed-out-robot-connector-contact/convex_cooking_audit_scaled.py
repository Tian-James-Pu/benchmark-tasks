"""Diagnostic only: compare cooked hulls with authored vertices in isolated stages.

Uses the synchronous cooking API illustrated by NVIDIA's ConvexMeshDataDemo.
The active physics scene is read; all comparison attributes are authored only
in temporary stages. Does not change any task body, joint, material or score.
"""
import numpy as np


def inspect_cooked_meshes(env):
    import isaaclab.sim as sim_utils
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdUtils, PhysicsSchemaTools
    from omni.physx import get_physx_cooking_interface
    from scipy.spatial import ConvexHull
    stage = sim_utils.get_current_stage()
    wanted = ('/probe/pin/pin_tip_c', '/probe/pin/pin_shaft_c',
              '/housing/lance/lance_tooth_c', '/tube/frame/wall_0_c')
    matches = [p for p in stage.Traverse() if any(str(p.GetPath()).endswith(s) for s in wanted)]
    cooking = get_physx_cooking_interface()
    for prim in matches:
        source = UsdGeom.Mesh(prim)
        vertices = np.asarray(source.GetPointsAttr().Get(), dtype=float)
        planes = ConvexHull(vertices).equations
        for name, minimum, limit in [('default', None, None), ('thin', .00001, None), ('thin_255', .00001, 255), ('millimetres', None, None), ('millimetres_255', None, 255), ('live', None, None)]:
            scale = .001 if name.startswith('millimetres') else 1.
            temp = Usd.Stage.CreateInMemory()
            UsdGeom.SetStageMetersPerUnit(temp, UsdGeom.GetStageMetersPerUnit(stage))
            mesh = UsdGeom.Mesh.Define(temp, '/Hull')
            mesh.CreatePointsAttr((vertices/scale).tolist())
            if scale != 1.:
                UsdGeom.Xformable(mesh).AddScaleOp().Set((scale,scale,scale))
            mesh.CreateFaceVertexCountsAttr(source.GetFaceVertexCountsAttr().Get())
            mesh.CreateFaceVertexIndicesAttr(source.GetFaceVertexIndicesAttr().Get())
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
            if minimum is not None or limit is not None:
                api = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(mesh.GetPrim())
                if minimum is not None:
                    api.CreateMinThicknessAttr(minimum)
                if limit is not None:
                    api.CreateHullVertexLimitAttr(limit)
            cache = UsdUtils.StageCache.Get()
            if name == 'live':
                temp = stage
            stage_id = cache.Insert(temp)
            results = []
            def received(result, hulls):
                results.append({'result':str(result), 'hulls':hulls})
            try:
                cooking.request_convex_collision_representation(
                    stage_id=stage_id.ToLongInt(),
                    collision_prim_id=PhysicsSchemaTools.sdfPathToInt(str(prim.GetPath()) if name == 'live' else '/Hull'),
                    run_asynchronously=False, on_result=received)
                rows = []
                for result in results:
                    for hull in result['hulls']:
                        cooked_raw = np.array([[v.x,v.y,v.z] for v in hull.vertices])
                        cooked = cooked_raw*scale
                        rows.append({'result':result['result'], 'vertex_count':len(cooked),
                            'cooked_raw_min':cooked_raw.min(axis=0).tolist(),
                            'mesh_scale':scale,
                            'cooked_min_m':cooked.min(axis=0).tolist(),
                            'cooked_max_m':cooked.max(axis=0).tolist(),
                            'maximum_outside_source_m':float(np.max(cooked@planes[:,:3].T+planes[:,3]))})
                print('CONVEX_COOKING_AUDIT', {'source':str(prim.GetPath()), 'variant':name,
                    'source_vertex_count':len(vertices), 'source_min_m':vertices.min(axis=0).tolist(),
                    'source_max_m':vertices.max(axis=0).tolist(),'callback_received':bool(results),'hulls':rows},flush=True)
            finally:
                if name != 'live':
                    cache.Erase(stage_id)
    print('CONVEX_COOKING_AUDIT_COUNT', len(matches),flush=True)
