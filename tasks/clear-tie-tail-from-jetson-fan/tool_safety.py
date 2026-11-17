"""Conservative tool bounds derived from the actual spawned collision meshes."""
import numpy as np
import mechkit as m

TOOLS=('long_nose_pliers','phillips_screwdriver')

def overlap(a,b):
    """Separating-axis test for two oriented boxes."""
    ca,ra,ha=a;cb,rb,hb=b;delta=cb-ca
    axes=[*ra.T,*rb.T]
    axes += [np.cross(x,y) for x in ra.T for y in rb.T]
    for axis in axes:
        if np.linalg.norm(axis)<1e-10:continue
        limit=np.dot(ha,np.abs(ra.T@axis))+np.dot(hb,np.abs(rb.T@axis))
        if abs(np.dot(delta,axis))>limit+1e-9:return False
    return True

class ToolGeometry:
    def __init__(self):self.parts={}
    def reset(self,env):
        from pxr import Usd,UsdGeom,UsdPhysics
        import isaaclab.sim as sim_utils
        stage=sim_utils.get_current_stage();cache=UsdGeom.XformCache();self.parts={}
        for name in TOOLS:
            root=stage.GetPrimAtPath('/World/envs/env_0/'+name)
            if not root.IsValid():raise RuntimeError('Missing tool root '+name)
            bodies=[p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(bodies)!=1:raise RuntimeError('Expected one merged tool body for '+name)
            body=bodies[0];inverse=cache.GetLocalToWorldTransform(body).GetInverse();parts=[]
            for prim in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
                if prim.GetTypeName()!='Mesh' or not prim.HasAPI(UsdPhysics.CollisionAPI):continue
                if prim.GetAttribute('physics:collisionEnabled').Get() is False:continue
                points=UsdGeom.Mesh(prim).GetPointsAttr().Get()
                if points is None or len(points)==0:continue
                matrix=np.asarray(cache.GetLocalToWorldTransform(prim)*inverse)
                vertices=np.asarray(points,dtype=float)@matrix[:3,:3]+matrix[3,:3]
                low=vertices.min(0);high=vertices.max(0)
                # Vertices are retained as positive witnesses in the diagnostic;
                # the conservative per-mesh boxes govern the safety rule.
                parts.append((str(prim.GetPath()).split('/'+name+'/')[-1],(low+high)/2,(high-low)/2,vertices))
            if not parts:raise RuntimeError('No physical collision meshes found for '+name)
            self.parts[name]=parts
            print('TOOL_GEOMETRY',name,{'collision_meshes':len(parts),'vertices':sum(len(v) for _,_,_,v in parts)},flush=True)
    def boxes(self,env):
        base=env.unwrapped;origin=m._np(base.scene.env_origins)[0]
        for name,parts in self.parts.items():
            pose=m._np(base.scene[name].data.root_link_pose_w)[0]
            position=pose[:3]-origin;rotation=m.quat_matrix(pose[3:7])
            for tag,center,half,vertices in parts:
                yield name+'/'+tag,(position+rotation@center,rotation,half),vertices@rotation.T+position
    def audit(self,env,fan):
        envelope=(np.asarray(fan)+[0,0,.0067],np.eye(3),np.array([.0185,.0185,.0037]))
        hits=[];witnesses=0
        for name,box,vertices in self.boxes(env):
            if overlap(box,envelope):hits.append(name)
            p=vertices-fan
            witnesses+=int(np.sum((np.linalg.norm(p[:,:2],axis=1)<.0185)&(p[:,2]>.003)&(p[:,2]<.0104)))
        return hits,witnesses
