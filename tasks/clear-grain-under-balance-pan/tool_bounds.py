"""Cached physical tool bounds transformed by each actual simulated body pose."""
import itertools,numpy as np
import mechkit as m
class ToolBounds:
 def reset(self,env):
  from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema
  import isaaclab.sim as sim_utils
  stage=sim_utils.get_current_stage();cache=UsdGeom.XformCache();self.corners={}
  for name in ('spatula','paper_cup'):
   root=stage.GetPrimAtPath('/World/envs/env_0/'+name)
   bodies=[p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
   if len(bodies)!=1:raise RuntimeError('Expected one rigid body for '+name)
   from isaaclab_physx.physics import PhysxManager
   body_path=str(bodies[0].GetPath())
   view=PhysxManager.get_physics_sim_view().create_rigid_body_view(body_path)
   offsets=np.asarray(view.get_contact_offsets().numpy(),dtype=float).copy()
   if view.count!=1 or view.max_shapes<1 or offsets.shape!=(1,view.max_shapes):
    raise RuntimeError(f'Incomplete native collider coverage for {body_path}: {offsets.shape}')
   if not np.isfinite(offsets).all() or np.any(offsets<0) or np.any(offsets>.020000001):
    raise RuntimeError(f'Native contact offsets exceed pairwise gate coverage: {body_path}: {offsets.tolist()}')
   print('TOOL_CONTACT_OFFSETS',name,{'body':body_path,'native_m':offsets.tolist()},flush=True)
   inverse=cache.GetLocalToWorldTransform(bodies[0]).GetInverse();vertices=[]
   for prim in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
    if prim.GetTypeName()!='Mesh' or not prim.HasAPI(UsdPhysics.CollisionAPI):continue
    if prim.GetAttribute('physics:collisionEnabled').Get() is False:continue
    attr=PhysxSchema.PhysxCollisionAPI(prim).GetContactOffsetAttr()
    print('TOOL_AUTHORED_CONTACT_OFFSET',str(prim.GetPath()),
      {'value':str(attr.Get()),'authored':attr.HasAuthoredValueOpinion()},flush=True)
    points=UsdGeom.Mesh(prim).GetPointsAttr().Get()
    if points is None or len(points)==0:continue
    matrix=np.asarray(cache.GetLocalToWorldTransform(prim)*inverse)
    vertices.append(np.asarray(points,dtype=float)@matrix[:3,:3]+matrix[3,:3])
   if not vertices:raise RuntimeError('No collision mesh for '+name)
   v=np.concatenate(vertices);low=v.min(0);high=v.max(0)
   self.corners[name]=np.array(list(itertools.product(*zip(low,high))))
   print('TOOL_BOUNDS',name,{'low':low.tolist(),'high':high.tolist()},flush=True)
 def boxes(self,env):
  base=env.unwrapped;origin=m._np(base.scene.env_origins)[0];out={}
  for name,corners in self.corners.items():
   pose=m._np(base.scene[name].data.root_link_pose_w)[0]
   pts=corners@m.quat_matrix(pose[3:7]).T+pose[:3]-origin
   out[name]=(pts.min(0),pts.max(0))
  return out
