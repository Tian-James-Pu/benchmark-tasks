"""Bounded failure isolation: unchanged closed pin chain, external gears absent.

This is not a task or a valid final drivetrain. Early free fall distinguishes
internal chain instability from chain/sprocket contact without tuning either.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_chain_closed_loop import *
from physics_hooks import on_physics_step
m._STEP.remove(observer.observe)
m._RESET.remove(observer.reset)
OBJECTS=[CHAIN]

class FreeTrace:
    def __init__(self):self.count=0;self.view=None
    def physics(self,env,dt):
        if self.view is None:
            import omni.physics.tensors as tensors
            self.sim=tensors.create_simulation_view('torch');self.sim.set_subspace_roots('/')
            self.view=self.sim.create_rigid_body_view('/World/envs/env_0/chain/*')
            self.indices={p.rsplit('/',1)[-1]:i for i,p in enumerate(self.view.prim_paths)}
        if self.count in (0,1,2,4,8,16,32,64,120,240,300):
            poses=m._np(self.view.get_transforms());vel=m._np(self.view.get_velocities())
            error=[]
            for i in range(106):
                a=poses[self.indices[f'chain_{i:03}']];b=poses[self.indices[f'chain_{(i+1)%106:03}']]
                pa=a[:3]+m.quat_matrix(a[3:])@np.array([.00635,0,0])
                pb=b[:3]+m.quat_matrix(b[3:])@np.array([-.00635,0,0])
                error.append(float(np.linalg.norm(pa-pb)))
            print('FREE_CHAIN_PHYSICS',self.count,dict(observed_time_s=self.count*dt,
                max_pin_anchor_error_m=max(error),max_linear_speed_m_s=float(np.linalg.norm(vel[:,:3],axis=1).max()),
                bounds_m=[poses[:,:3].min(0).tolist(),poses[:,:3].max(0).tolist()],
                maximum_angular_speed_rad_s=float(np.linalg.norm(vel[:,3:],axis=1).max())),flush=True)
        self.count+=1
free_trace=FreeTrace();on_physics_step(free_trace.physics)
