"""Observe the unchanged v2 mechanism before and during reset settling."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_chain_closed_loop import *
from physics_hooks import on_physics_step

class Trace:
    def __init__(self):
        self.count=0
        self.view=None
    def physics(self,env,dt):
        if self.view is None:
            import omni.physics.tensors as tensors
            self.sim=tensors.create_simulation_view('torch')
            self.sim.set_subspace_roots('/')
            self.view=self.sim.create_rigid_body_view('/World/envs/env_0/chain/*')
            self.indices={p.rsplit('/',1)[-1]:i for i,p in enumerate(self.view.prim_paths)}
        if self.count in (0,1,2,4,8,16,32,64,120,240,300):
            poses=m._np(self.view.get_transforms())
            velocities=m._np(self.view.get_velocities())
            origin=m._np(env.unwrapped.scene.env_origins[0])
            positions={name:poses[i,:3]-origin for name,i in self.indices.items()}
            rotations={name:m.quat_matrix(poses[i,3:]) for name,i in self.indices.items()}
            errors=[]
            for i in range(106):
                a=f'chain_{i:03}';b=f'chain_{(i+1)%106:03}'
                left=positions[a]+rotations[a]@np.array([.00635,0,0])
                right=positions[b]+rotations[b]@np.array([-.00635,0,0])
                errors.append(float(np.linalg.norm(left-right)))
            shifts=[float(np.linalg.norm(positions[b.name]-ORIGIN-b.pos)) for b in CHAIN_BODIES]
            selected={name:dict(pose=poses[self.indices[name]].tolist(),expected_pos=(ORIGIN+np.array(b.pos)+origin).tolist())
                      for b in CHAIN_BODIES if (name:=b.name) in ('chain_000','chain_001','chain_052','chain_105')}
            state=m.state(env,'drive')
            print('CHAIN_PHYSICS',self.count,dict(time_s=self.count*dt,env_origin=origin.tolist(),
                  max_pin_anchor_error_m=max(errors),max_translation_from_authored_m=max(shifts),
                  max_linear_speed_m_s=float(np.linalg.norm(velocities[:,:3],axis=1).max()),
                  bounds_m=[poses[:,:3].min(0).tolist(),poses[:,:3].max(0).tolist()],
                  drive_root=state.pos.tolist(),drive_joints=state.joints,selected=selected),flush=True)
        self.count+=1
trace=Trace()
on_physics_step(trace.physics)
