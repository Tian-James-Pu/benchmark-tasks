"""Read-only contact diagnosis of the failed passive wire-retention gate."""
import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from jam_idle_v5 import *
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
OBJECTS=[reporting(x) if x.name=='bent_staple' else x for x in OBJECTS]
_cv=None
_sv=None
_count=0
@m.on_step
def diagnose(env):
    global _cv,_sv,_count
    _count+=1
    base=env.unwrapped
    if _cv is None:
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics
        import isaaclab.sim as sim_utils
        stage=sim_utils.get_current_stage()
        paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        wire=[p for p in paths if '/bent_staple' in p]
        driver=[p for p in paths if '/stapler/' in p and p.endswith('/driver')]
        frame=[p for p in paths if '/stapler/' in p and p.endswith('/frame')]
        print('CONTACT_PATHS '+json.dumps({'wire':wire,'driver':driver,'frame':frame}),flush=True)
        if not(len(wire)==len(driver)==len(frame)==1):raise RuntimeError('contact path coverage')
        _sv=PhysxManager.get_physics_sim_view()
        _cv=_sv.create_rigid_contact_view(wire,filter_patterns=[driver+frame],max_contact_data_count=512)
    if _count%5==0:
        a=base.scene['stapler'];b=base.scene['bent_staple']
        row={'step':_count,'body_names':a.body_names,
             'poses':a.data.body_link_pose_w[0].detach().cpu().numpy().tolist(),
             'wire_pose':b.data.root_link_pose_w[0].detach().cpu().numpy().tolist(),
             'forces':_cv.get_contact_force_matrix(dt=1/480).numpy().tolist()}
        print('CONTACT_DIAG '+json.dumps(row),flush=True)

def demo_script(ctx):
    yield from ctx.left.hold(2.)
