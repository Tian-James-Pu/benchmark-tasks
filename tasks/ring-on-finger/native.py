"""The ring finger's silicone skin as a native PhysX volume-FEM body. Import Isaac modules only after SimulationApp.

Same construction as our pillowcase pillow (build/pillowcase/native_pillow.py, which ran in jobs 4285184-4286255):
IsaacLab 3's MeshCuboidCfg + PhysxDeformableBodyMaterialCfg through an Arena DeformableObject, with a pre-built
UsdGeom.TetMesh authored at {prim}/sim_mesh just before IsaacLab's define_deformable_body_properties, so IsaacLab uses
our ~1 mm structured mesh instead of re-tetrahedralising (pytetwild coarsens). The render mesh is the tet boundary.
The skin is not attached to anything: it is a sleeve on the rigid bone, resting on the palm top under the rigid dome.
"""
import numpy as np

ACCEPTED_MATERIAL_FIELDS = []
TET_SUMMARY = {}


def linear_rgb(srgb):
    return tuple(c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4 for c in srgb)


def ensure_gpu_scene(stage):
    """GPU dynamics are required for FEM deformables (peg-out-a-tshirt, pillowcase)."""
    from pxr import PhysxSchema, UsdPhysics
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not scenes:
        raise RuntimeError('Create the simulation context before the scene bodies')
    for prim in scenes:
        api = PhysxSchema.PhysxSceneAPI.Apply(prim)
        api.CreateEnableGPUDynamicsAttr(True)
        api.CreateBroadphaseTypeAttr('GPU')
        api.CreateEnableCCDAttr(False)
        api.CreateSolverTypeAttr('TGS')


def _material(density):
    from isaaclab_physx.sim.spawners.materials import PhysxDeformableBodyMaterialCfg
    import scene as S
    base = dict(youngs_modulus=S.SKIN_E, poissons_ratio=S.SKIN_NU, density=density)
    for extra in (dict(static_friction=S.MU_SKIN, dynamic_friction=S.MU_SKIN, elasticity_damping=0.005),
                  dict(dynamic_friction=S.MU_SKIN), {}):
        try:
            cfg = PhysxDeformableBodyMaterialCfg(**base, **extra)
        except TypeError:
            continue
        ACCEPTED_MATERIAL_FIELDS[:] = sorted({**base, **extra})
        return cfg
    raise RuntimeError('PhysxDeformableBodyMaterialCfg rejected the basic FEM fields')


def skin_object(name, center):
    """Arena DeformableObject for the skin; ``center`` = world position of the rest-shape node centroid."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.deformable_object import DeformableObject
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg

    import geometry as G
    import scene as S

    tet = G.skin_tets()
    centroid = tet.points.mean(0)
    local = tet.points - centroid
    TET_SUMMARY.clear()
    TET_SUMMARY.update(nodes=len(tet.points), tets=len(tet.tets), grid=list(tet.grid), volume_mm3=tet.volume() * 1e9,
                       min_tet_volume_mm3=float(tet.volumes().min() * 1e9), centroid_hand_frame=centroid.tolist())
    size = tuple(float(v) for v in (local.max(0) - local.min(0)))
    props = PhysxDeformableBodyPropertiesCfg(solver_position_iteration_count=S.SKIN_ITERATIONS, linear_damping=0.0)
    common = dict(deformable_props=props,
                  collision_props=[PhysxCollisionCfg(rest_offset=S.REST_OFFSET, contact_offset=S.CONTACT_OFFSET_FINE)],
                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(S.COLORS['skin']), roughness=.6))

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        import trimesh
        from isaaclab.sim.spawners.meshes import meshes as mesh_spawners
        from isaaclab.sim.utils.stage import get_current_stage
        stage = get_current_stage()
        ensure_gpu_scene(stage)
        used = np.unique(tet.surface)
        remap = -np.ones(len(local), int)
        remap[used] = np.arange(len(used))
        visual = trimesh.Trimesh(vertices=local[used], faces=remap[tet.surface], process=False)
        schemas = mesh_spawners.schemas
        original = schemas.define_deformable_body_properties

        def define(body_path, *args, **kw):
            if str(body_path) == str(prim_path):
                mesh_spawners.create_prim(str(prim_path) + '/sim_mesh', prim_type='TetMesh', stage=stage,
                                          attributes={'points': local.astype(np.float64),
                                                      'tetVertexIndices': tet.tets.astype(np.int32)})
            return original(body_path, *args, **kw)

        schemas.define_deformable_body_properties = define
        try:
            mesh_spawners._spawn_mesh_geom_from_mesh(prim_path, cfg, visual, translation, orientation, None, stage=stage)
        finally:
            schemas.define_deformable_body_properties = original
        # The painted 1 mm lines are for the real photos only (render-only; a 4 mm cell cannot hold a 1 mm band), so
        # S.paint_skin_lines is not applied in sim; the scorer reads poses.
        TET_SUMMARY['paint'] = dict(painted=False, reason='render-only lines not drawn on 4 mm cells')
        return stage.GetPrimAtPath(prim_path)

    cfg = sim_utils.MeshCuboidCfg(func=spawn, size=size, physics_material=_material(S.SKIN_DENSITY), **common)
    return DeformableObject(name=name, spawner_cfg=cfg,
                            initial_pose=Pose(position_xyz=tuple(float(v) for v in center), rotation_xyzw=(0., 0., 0., 1.)))
