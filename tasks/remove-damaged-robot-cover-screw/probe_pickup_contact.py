"""Native pickup retry: seat pads lower and close on the existing handles.

Same hardware as task.py, no altered robot limits. This is still always score0.
"""
import pathlib
import sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from task import *

class Contacts:
    def reset(self,env=None):self.steps=0;self.view=None
    def step(self,env):
        self.steps+=1
        if self.view is None:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            bodies=[p for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            filters=[str(p.GetPath()) for p in bodies if '/pliers/' in str(p.GetPath())]
            sensors=[str(p.GetPath()) for p in bodies if '/RightArm/' in str(p.GetPath()) and p.GetName() in ('tip_left','tip_right')]
            print('SCREW_GATE_CONTACT_PATHS',sensors,filters,flush=True)
            self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=1024)
        if self.steps==1 or self.steps%10==0:
            r=env.unwrapped.scene['right_robot']
            print('SCREW_GATE_FORCES',self.steps,{'finger_contact_N':self.view.get_contact_force_matrix(dt=env.unwrapped.physics_dt).numpy().tolist(),
                  'native_applied_finger_effort_N':m._np(r.data.applied_torque)[0,-2:].tolist()},flush=True)
C=Contacts();m.on_reset(C.reset);m.on_step(C.step)

def demo():
    return {'right':[
        Waypoint(1.5,(.225,-.164,TOP+.100),-.04695,'hover handles'),
        Waypoint(1.5,(.225,-.164,TOP+.012),-.04695,'seat pads beside handles'),
        Waypoint(2.,(.225,-.164,TOP+.012),-.010,'close on handles'),
        Waypoint(2.,(.225,-.164,TOP+.100),-.010,'lift tool'),
        Waypoint(2.,(.225,-.164,TOP+.100),-.010,'hold lifted tool')
    ]}
