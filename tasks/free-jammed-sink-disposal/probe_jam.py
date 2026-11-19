"""Diagnostic only: real spoon contact under a modest declared torque ramp.

The diagnostic wrench load is not a calibrated Moen stall or breakaway value.
There is no success score and no electrical-safety or completed-repair claim.
"""
import pathlib,sys,json
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from disposal_geometry import objects
from yam_arena.demo import Waypoint
SCENE='station';SUCCESS_HOLD_S=0.
INSTRUCTION='Clear the metal spoon from the stopped sink disposal.'
OBJECTS=objects()
_scene=m._Finder.targets['yam_arena.scene']
def settings(scene):
 _scene(scene);scene.PHYSICS_DT=1/480.
m._Finder.targets['yam_arena.scene']=settings

class Observe:
 def reset(self,env=None):self.n=0;self.contact=None;self.contact_error=None
 def step(self,env):
  self.n+=1
  a=env.unwrapped.scene['rotor'];i=a.joint_names.index('turntable_joint')
  #0.5s rest,1s CW ramp,1s hold,1s unload,1s CCW ramp,1s hold.
  t=self.n/20
  torque=0. if t<.5 else (.10*min(1.,t-.5) if t<2.5 else
   (0. if t<3.5 else -.10*min(1.,t-3.5)))
  import torch
  jp=getattr(a.data.joint_pos,'torch',a.data.joint_pos)
  a.set_joint_effort_target_index(target=torch.tensor([[torque]],dtype=torch.float32,device=jp.device),joint_ids=[i])
  if self.n==1:
   try:
    import isaaclab.sim as sim_utils
    from isaaclab_physx.physics import PhysxManager
    from pxr import UsdPhysics
    bodies=[str(p.GetPath()) for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    sensors=[p for p in bodies if '/spoon/' in p or p.endswith('/spoon')]
    filters=[p for p in bodies if '/rotor/' in p]
    self.contact=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=4096)
    print('DISPOSAL_CONTACT_PATHS',json.dumps(dict(sensors=sensors,rotor_filters=filters)),flush=True)
   except Exception as e:self.contact_error=repr(e)
  if self.n==1 or self.n%5==0:
   from yam_arena.scene import object_states
   s=object_states(env,('spoon',)).get('spoon')
   row=dict(step=self.n,diagnostic_torque_Nm=torque,joint_names=a.joint_names,
    joints=m._np(a.data.joint_pos)[0].tolist(),joint_vel=m._np(a.data.joint_vel)[0].tolist(),
    applied_torque=m._np(a.data.applied_torque)[0].tolist())
   if s is not None:row.update(spoon_pos=s.pos.tolist(),spoon_quat=s.quat_xyzw.tolist(),
    spoon_lin_vel=s.lin_vel.tolist(),spoon_ang_vel=s.ang_vel.tolist())
   if self.contact is not None:
    try:row.update(spoon_rotor_contact_N=self.contact.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist(),spoon_net_contact_N=self.contact.get_net_contact_forces(dt=env.unwrapped.physics_dt).numpy().tolist())
    except Exception as e:self.contact_error=repr(e)
   if self.contact_error is not None:row['contact_instrumentation_error']=self.contact_error
   if self.n==1:row.update(body_names=a.body_names,masses=a.root_physx_view.get_masses().numpy().tolist(),
    inertias=a.root_physx_view.get_inertias().numpy().tolist())
   print('DISPOSAL_NATIVE',json.dumps(row),flush=True)
O=Observe();m.on_reset(O.reset);m.on_step(O.step)
def score(objects):return 0.
def success(objects):return False
def demo():return {'right':[Waypoint(6.,(.26,-.22,1.0275),-.04695,'observe mechanism gate')]}
