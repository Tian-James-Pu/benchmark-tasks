"""Precision contact margins and articulation reset for the rubber-foot scene."""
import mechkit as m
_scene=m._Finder.targets['yam_arena.scene']
def fine(scene):_scene(scene);scene.PHYSICS_DT=1./960.
m._Finder.targets['yam_arena.scene']=fine
_obj=m._arena_object
def precise(spec):
    obj=_obj(spec)
    if spec.role=='free':
        import isaaclab.sim as su
        obj.spawn_cfg_addon['collision_props']=su.CollisionPropertiesCfg(contact_offset=.00005,rest_offset=0.)
    if spec.role=='jointed':
        from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
        obj._pose_event_cfg.func=reset_articulation_pose_and_joints
    return obj
m._arena_object=precise
