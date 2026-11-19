"""Native fixture-only axial load with force, torque and contact measurements.

No robot manipulation or repair credit. Original geometry, gravity, masses and
ideal3mm lead remain unchanged. Forces below are explicit calibration inputs.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import json
import math
import numpy as np
import mechkit as m
import closedloop as cl
from physics_observer import on_physics_step

HERE=Path(__file__).resolve().parent
SCENE='station'
INSTRUCTION='Observe bounded fixture torque pulses on the passive vise; the robot stays stationary.'
SUCCESS_HOLD_S=0.
JOINTS=['joint_slidingJaw_01_joint_00','joint_screw_01_joint_00','joint_handle_01_joint_00']
OBJECTS=[m.Part('vise',pos=(.28,-.05,.7475),size=(.325,.233,.181),role='jointed',yaw_deg=-90.,
               build=lambda:(HERE/'vise-passive-component.usda').read_text(),
               drives=tuple((name,(0.,0.,5.,0.,0.)) for name in JOINTS))]


class Probe:
    def reset(self,env=None):
        self.step=0;self.samples=[];self.contact=None;self.paths=[]
        self.load=0.;self.last_load=0.
    def initialize(self,env):
        from pxr import Usd,UsdPhysics
        import isaaclab.sim as sim_utils
        from isaaclab_physx.physics import PhysxManager
        prims=list(Usd.PrimRange(sim_utils.get_current_stage().GetPseudoRoot(),Usd.TraverseInstanceProxies()))
        bodies=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        handle=[p for p in bodies if '/vise/' in p and p.endswith('/handle_01_obj_00')]
        bench=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)
               and (str(p.GetPath()).endswith('/table') or '/table/' in str(p.GetPath()))]
        if len(handle)!=1 or not bench:raise RuntimeError('Incomplete vise contact coverage '+str((handle,bench)))
        self.paths=[handle,bench]
        self.contact=PhysxManager.get_physics_sim_view().create_rigid_contact_view(handle,filter_patterns=[bench],max_contact_data_count=4096)
        print('VISE_CONTACT_COVERAGE '+json.dumps({'handle':handle,'bench':bench}),flush=True)
        asset=env.unwrapped.scene['vise']
        print('VISE_FORCE_METADATA '+json.dumps({'body_names':asset.body_names,'joint_names':asset.joint_names,
              'masses_kg':asset.root_physx_view.get_masses().numpy().tolist(),
              'friction_properties':asset.root_physx_view.get_dof_friction_properties().numpy().tolist()}),flush=True)
    def capture(self,env,dt):
        asset=env.unwrapped.scene['vise']
        q=asset.root_physx_view.get_dof_positions().numpy().copy()[0]
        v=asset.root_physx_view.get_dof_velocities().numpy().copy()[0]
        if not np.isfinite(q).all() or not np.isfinite(v).all():raise RuntimeError('Nonfinite native vise states')
        ids=[asset.joint_names.index(n) for n in JOINTS]
        forces=self.contact.get_contact_force_matrix(dt=dt).numpy().copy()
        if not np.isfinite(forces).all():raise RuntimeError('Nonfinite contact forces')
        incoming=asset.root_physx_view.get_link_incoming_joint_force().numpy().copy()[0]
        projected=asset.root_physx_view.get_dof_projected_joint_forces().numpy().copy()[0]
        actuation=asset.root_physx_view.get_dof_actuation_forces().numpy().copy()[0]
        if not all(np.isfinite(a).all() for a in (incoming,projected,actuation)):
            raise RuntimeError('Nonfinite native force observation')
        return {'q':q[ids].tolist(),'v':v[ids].tolist(),
                'coupling_error_m':float(q[ids[0]]-.003*q[ids[1]]/(2*math.pi)),
                'handle_bench_normal_N':forces.reshape(-1,3).sum(0).tolist(),
                'incoming_joint_wrenches':incoming.tolist(),
                'projected_joint_forces':projected[ids].tolist(),
                'actuation_forces':actuation[ids].tolist(),'previous_applied_axial_load_N':self.last_load}
    def apply_fixture_load(self,env):
        import torch
        asset=env.unwrapped.scene['vise']
        ids=[asset.body_names.index(n) for n in ['slidingJaw_01_obj_00','stationaryJaw_01_obj_00']]
        # Original vise +X is world-Y at its -90degree placement. Colocated
        # opposite calibration loads conserve net force/torque of the assembly.
        force=np.array([0.,-self.load,0.])
        origin=env.unwrapped.scene.env_origins[0].detach().cpu().numpy()
        point=origin+np.array([.28,-.156,.9075])
        forces=torch.tensor([[force.tolist(),(-force).tolist()]],dtype=torch.float32,device=asset.device)
        positions=torch.tensor([[point.tolist(),point.tolist()]],dtype=torch.float32,device=asset.device)
        composer=asset.permanent_wrench_composer
        composer.set_forces_and_torques_index(forces=forces,torques=torch.zeros_like(forces),positions=positions,body_ids=ids,is_global=True)
        actual=composer.global_force_w.numpy()[0].copy();moments=composer.global_torque_w.numpy()[0].copy()
        expected=np.zeros_like(actual);expected[ids[0]]=force;expected[ids[1]]=-force
        if np.max(np.abs(actual-expected))>1e-5 or np.linalg.norm(actual.sum(0))>1e-5 or np.linalg.norm(moments.sum(0))>1e-5:
            raise RuntimeError('Unbalanced vise fixture load')
        self.last_load=self.load
    def physics(self,env,dt):
        if self.contact is not None:
            self.samples.append(self.capture(env,dt))
            self.apply_fixture_load(env)
    def observe(self,env):
        import torch
        self.step+=1
        if self.contact is None:self.initialize(env)
        else:
            self.samples.append(self.capture(env,env.unwrapped.physics_dt))
            expected=round(env.unwrapped.step_dt/env.unwrapped.physics_dt)+1
            if len(self.samples)!=expected:raise RuntimeError('Physics observation count '+str((len(self.samples),expected)))
        torque=.20 if 20<=self.step<25 else -.20 if 45<=self.step<50 else 0.
        self.load=50. if 30<=self.step<70 else 0.
        asset=env.unwrapped.scene['vise']
        effort=torch.zeros_like(asset.data.joint_pos)
        effort[0,asset.joint_names.index(JOINTS[1])]=torque
        asset.set_joint_effort_target(effort)
        print('VISE_LOAD '+json.dumps({'step':self.step,'physics_dt':float(env.unwrapped.physics_dt),
              'next_fixture_torque_Nm':torque,'next_fixture_axial_load_N':self.load,'samples':self.samples,
              'sample_convention':'First sample repeats previous boundary; exclude it from interval impulse/work sums.'}),flush=True)
        self.samples.clear()


PROBE=Probe();PROBE.reset()
on_physics_step(PROBE.physics);m.on_reset(PROBE.reset);m.on_step(PROBE.observe)
demo=cl.placeholder_demo
def demo_script(ctx):yield from ctx.right.hold(4.)
def score(objects):return 0.
def success(objects):return False
