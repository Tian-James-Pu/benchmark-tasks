"""Read-only native contact evidence for source-aligned full-case retained jam."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from full_case_v14 import *
from dataclasses import replace

def reporting(part):
    original=part.build
    def build():
        from pxr import Sdf,Usd,UsdPhysics,PhysxSchema
        layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(original())
        stage=Usd.Stage.Open(layer)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        return layer.ExportToString()
    return replace(part,build=build)
OBJECTS=[reporting(x) if x.name in ('stapler','bent_staple') else x for x in OBJECTS]
_view=None
_sim=None
_steps=0
@m.on_step
def contact_evidence(env):
    global _view,_sim,_steps
    _steps+=1
    if _view is None:
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics
        import isaaclab.sim as sim_utils
        paths=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        wire=[p for p in paths if '/bent_staple' in p]
        driver=[p for p in paths if '/stapler/' in p and p.endswith('/driver')]
        tool=[p for p in paths if '/screwdriver' in p]
        frame=[p for p in paths if '/stapler/' in p and p.endswith('/frame')]
        if not len(wire)==len(driver)==len(tool)==len(frame)==1:raise RuntimeError(str((wire,driver,tool)))
        print('DRIVER_CONTACT_PATHS '+json.dumps({'sensors':driver+wire+frame,'filters':wire+tool}),flush=True)
        _sim=PhysxManager.get_physics_sim_view()
        _view=_sim.create_rigid_contact_view(driver+wire+frame,filter_patterns=[wire+tool]*3,max_contact_data_count=1024)
    if _steps%5:return
    normal,points,directions,distances,counts,starts=_view.get_contact_data(dt=1/480)
    normal=normal.numpy().copy();points=points.numpy().copy();directions=directions.numpy().copy();counts=counts.numpy().copy();starts=starts.numpy().copy()
    friction,_,fc,fs=_view.get_friction_data(dt=1/480)
    friction=friction.numpy().copy();fc=fc.numpy().copy();fs=fs.numpy().copy()
    pairs={}
    for k,sensor in enumerate(['driver','wire','frame']):
      for i,name in enumerate(['wire','screwdriver']):
        if sensor!='driver' and name!='screwdriver':continue
        n=int(counts[k,i]);s=int(starts[k,i]);fn=int(fc[k,i]);f=int(fs[k,i])
        if n>=1024 or fn>=1024 or (n and (s<0 or s+n>len(normal))) or (fn and (f<0 or f+fn>len(friction))):raise RuntimeError('contact range or capacity')
        nv=np.sum(normal.reshape(-1)[s:s+n,None]*directions[s:s+n],axis=0) if n else np.zeros(3)
        fv=np.sum(friction[f:f+fn],axis=0) if fn else np.zeros(3)
        pairs[sensor+':'+name]={'normal_count':n,'friction_count':fn,'normal_force_N':nv.tolist(),'friction_force_N':fv.tolist(),'total_force_N':(nv+fv).tolist(),'points_m':points[s:s+min(n,2)].tolist() if n else []}
    print('DRIVER_CONTACT '+json.dumps({'step':_steps,'pairs':pairs}),flush=True)
