"""Precision contact and reset correction for the service spring scene."""
import math
from dataclasses import replace
import numpy as np
import mechkit as m
from spring_scene import C,Q,START_Q,PIN_R,WIRE_R,HOOK_R,objects as original_objects,service_build

_original_object=m._arena_object
def precise_object(spec):
    obj=_original_object(spec)
    if spec.name=='spring_hook':
        import isaaclab.sim as sim_utils
        obj.spawn_cfg_addon['collision_props']=sim_utils.CollisionPropertiesCfg(contact_offset=.00005,rest_offset=0.)
        obj.spawn_cfg_addon['rigid_props']=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=64,solver_velocity_iteration_count=8)
    if spec.name=='saw':
        from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
        obj._pose_event_cfg.func=reset_articulation_pose_and_joints
    return obj
m._arena_object=precise_object


def precision_service():
    from pxr import Usd,Sdf,UsdGeom,Gf,UsdPhysics
    layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(service_build())
    stage=Usd.Stage.Open(layer)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.AddAppliedSchema('PhysxCollisionAPI')
            prim.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.00005)
            prim.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
    p=UsdGeom.Cylinder.Define(stage,'/Asset/Links/housing/fixed_spring_anchor')
    p.CreateRadiusAttr(.001);p.CreateHeightAttr(.005);p.CreateAxisAttr('X')
    p.AddTranslateOp().Set(Gf.Vec3d(-.0705,0.,.087));p.CreateDisplayColorAttr([(.62,.64,.66)])
    UsdPhysics.CollisionAPI.Apply(p.GetPrim())
    return layer.ExportToString()


def capsule_hook(part):
    original=part.build
    def build():
        from pxr import Usd,Sdf,UsdGeom,UsdPhysics,Gf
        layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(original())
        stage=Usd.Stage.Open(layer);root=stage.GetDefaultPrim().GetPath()
        for prim in list(stage.Traverse()):
            if str(prim.GetName()).endswith('_c'):stage.RemovePrim(prim.GetPath())
        pts=[(-.006,-HOOK_R,0.),(0.,-HOOK_R,0.)]
        pts += [(HOOK_R*math.cos(a),HOOK_R*math.sin(a),0.) for a in np.linspace(-math.pi/2,math.pi/2,17)[1:]]
        pts += [(-.003,HOOK_R,0.)]
        for i,(a,b) in enumerate(zip(pts[:-1],pts[1:])):
            a=np.array(a);b=np.array(b);d=b-a
            cap=UsdGeom.Capsule.Define(stage,str(root)+f'/wire_cap_{i}')
            cap.CreateRadiusAttr(WIRE_R);cap.CreateHeightAttr(float(np.linalg.norm(d)))
            cap.CreateAxisAttr('Z');cap.CreateVisibilityAttr('invisible')
            cap.AddTranslateOp().Set(Gf.Vec3d(*((a+b)/2)))
            cap.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*(d/np.linalg.norm(d)))).GetQuat()))
            UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            cap.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
            cap.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.00005)
            cap.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
        return layer.ExportToString()
    return replace(part,build=build)


def objects(engaged=True):
    saw,hook=original_objects(engaged)
    return [replace(saw,build=precision_service),capsule_hook(hook)]
