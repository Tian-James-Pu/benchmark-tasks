"""Read-only native contact evidence for source-aligned full-case retained jam."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from full_case_v33 import *
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
OBJECTS=[reporting(x) if x.name in ('stapler','bent_staple','battery') else x for x in OBJECTS]
# Ordinary inverted service pose, broad motor housing resting on bench.
NOSE=np.array([.050,0.,TOP+.027])
UPRIGHT_Q=(2**-.5,0.,0.,2**-.5)
OBJECTS=[replace(x,pos=tuple(NOSE),start_quat=UPRIGHT_Q) if x.name in ('stapler','bent_staple','battery') else x for x in OBJECTS]

# Component initial state: battery already isolated, using archived v44 released pose.
from yam_arena.ik import matrix_to_quat_xyzw
_battery_R=np.array([[0.9999999999997092, 6.583286175696484e-07, -3.8491478440492215e-07], [-3.8491478440492215e-07, 2.0281565371593757e-07, -0.9999997971841454], [-6.583286175696484e-07, 0.9999997971840029, 2.028159071798541e-07]])@np.array([[-3.311399107299294e-07, -0.9992874538592904, -0.03774185214714373], [0.9999997286060537, -3.72469175354079e-07, -6.604266121908499e-07], [7.144274049845979e-07, -0.037741913070420026, 0.9992872511878556]])
OBJECTS=[replace(x,pos=(0.6653935313224792, -0.006109805777668953, 0.6104999780654907),start_quat=tuple(matrix_to_quat_xyzw(_battery_R))) if x.name=='battery' else x for x in OBJECTS]

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
