"""Bounded physical feed-core diagnostic, score always zero.

No robot-repair claim: the housing is fixed and a laboratory force presses the
button axis. Nothing teleports the lead or changes its attachment state.
Clip/retraction, removable threaded nose and jam are not yet represented here.
"""
import pathlib,sys,json
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from pencil_geometry import feed_core,lead
_scene=m._Finder.targets['yam_arena.scene']
def fine(scene):
    _scene(scene);scene.PHYSICS_DT=1/960.
m._Finder.targets['yam_arena.scene']=fine
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=0.
INSTRUCTION='Make the mechanical pencil ready to write again.'
ORIGIN=np.array([.30,-.055,.82]);AXIS=np.array([2**-.5,2**-.5,0.])
OBJECTS=[feed_core(ORIGIN,fixed=True),lead(ORIGIN)]
class Observe:
    def reset(self,env=None):self.n=0;self.first=None
    def step(self,env):
        self.n+=1
        a=env.unwrapped.scene['pencil']
        names=a.joint_names;idx=names.index('feed_joint')
        if self.n==1:
            print('PENCIL_NATIVE',json.dumps(dict(body_names=a.body_names,
                masses=a.root_physx_view.get_masses().numpy().tolist(),
                inertias=a.root_physx_view.get_inertias().numpy().tolist())),flush=True)
        # 1 s settle, then one 1 s press/release cycle, then final observation.
        press=20<=self.n<60 and (self.n-20)%40<20
        import torch
        jp=getattr(a.data.joint_pos,'torch',a.data.joint_pos)
        effort=torch.tensor([[-2.2 if press else 0.]],device=jp.device,dtype=torch.float32)
        a.set_joint_effort_target_index(target=effort,joint_ids=[idx])
        if self.n==1 or self.n%5==0:
            l=m.state(env,'lead');p=m.state(env,'pencil')
            row=dict(step=self.n,diagnostic_button_force_N=-2.2 if press else 0.,
                lead_origin=l.pos.tolist(),lead_q=l.quat_xyzw.tolist(),
                lead_v=l.lin_vel.tolist(),lead_w=l.ang_vel.tolist(),
                lead_axial_m=float((l.pos-ORIGIN)@AXIS),joints=p.joints,
                joint_velocity=m._np(a.data.joint_vel)[0].tolist(),
                joint_names=names,applied_effort=m._np(a.data.applied_torque)[0].tolist())
            print('PENCIL_FEED',json.dumps(row),flush=True)
O=Observe();m.on_reset(O.reset);m.on_step(O.step)
def score(objects):return 0.
def success(objects):return False
def demo():
    return {'right':[Waypoint(4.,(.26,-.22,TOP+.28),-.04695,'observe component test')]}
