"""Read-only native contact evidence for flat-blade feed-side retraction."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from retract_v11 import *
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
OBJECTS=[reporting(x) if x.name=='stapler' else x for x in OBJECTS]
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
        if not len(wire)==len(driver)==len(tool)==1:raise RuntimeError(str((wire,driver,tool)))
        print('DRIVER_CONTACT_PATHS '+json.dumps({'driver':driver,'filters':wire+tool}),flush=True)
        _sim=PhysxManager.get_physics_sim_view()
        _view=_sim.create_rigid_contact_view(driver,filter_patterns=[wire+tool],max_contact_data_count=512)
    if _steps%10:return
    normal,points,directions,distances,counts,starts=_view.get_contact_data(dt=1/480)
    normal=normal.numpy().copy();points=points.numpy().copy();directions=directions.numpy().copy();counts=counts.numpy().copy();starts=starts.numpy().copy()
    friction,_,fc,fs=_view.get_friction_data(dt=1/480)
    friction=friction.numpy().copy();fc=fc.numpy().copy();fs=fs.numpy().copy()
    pairs={}
    for i,name in enumerate(['wire','screwdriver']):
        n=int(counts[0,i]);s=int(starts[0,i]);fn=int(fc[0,i]);f=int(fs[0,i])
        if n>=512 or fn>=512 or s<0 or s+n>len(normal) or f<0 or f+fn>len(friction):raise RuntimeError('contact range or capacity')
        nv=np.sum(normal.reshape(-1)[s:s+n,None]*directions[s:s+n],axis=0)
        fv=np.sum(friction[f:f+fn],axis=0)
        pairs[name]={'normal_count':n,'friction_count':fn,'normal_force_N':nv.tolist(),'friction_force_N':fv.tolist(),'total_force_N':(nv+fv).tolist(),'points_m':points[s:s+min(n,4)].tolist()}
    print('DRIVER_CONTACT '+json.dumps({'step':_steps,'pairs':pairs}),flush=True)
