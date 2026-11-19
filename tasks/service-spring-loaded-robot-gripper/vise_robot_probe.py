"""Robot grasp/turn diagnostic with passive approximate vise friction.

No fixture effort, pose writes, grasp attachment or task credit. Thread/guide
model follows the measured-load calibration. This variant adds journal drag
mu * measured transverse screw reaction * measured6.55mm shaft radius.
Bearing normal-force sums and material coefficients remain approximations.
"""
from pathlib import Path
import os
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
INSTRUCTION='Grasp and turn the vise handle. Component diagnostic, no repair credit.'
MU=float(os.environ.get('VISE_FRICTION_MU','.15'))
GUIDE_MU=.15
JOURNAL_MU=.15
JOURNAL_RADIUS=.00655
RADIUS=.006
assert MU in (0.,.15)
SUCCESS_HOLD_S=0.
JOINTS=['joint_slidingJaw_01_joint_00','joint_screw_01_joint_00','joint_handle_01_joint_00']
OBJECTS=[m.Part('vise',pos=(.28,-.05,.7475),size=(.325,.233,.181),role='jointed',yaw_deg=-90.,
               build=lambda:(HERE/'vise-centered-component.usda').read_text(),
               drives=tuple((name,(0.,0.,5.,0.,0.)) for name in JOINTS))]


class Probe:
    def reset(self,env=None):
        self.step=0;self.samples=[];self.contact=None;self.paths=[]
        self.load=0.;self.last_load=0.;self.friction_effort=0.;self.guide_effort=0.
        self.assembly_contact=None;self.contact_names=[];self.latest=None
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
        vise=[p for p in bodies if '/vise/' in p]
        robot=[p for p in bodies if '/RightArm/' in p or '/LeftArm/' in p]
        if len(vise)!=5 or len(robot)<16:raise RuntimeError('Incomplete robot/vise contact coverage')
        filters=robot+bench
        self.assembly_contact=PhysxManager.get_physics_sim_view().create_rigid_contact_view(vise,filter_patterns=[filters]*len(vise),max_contact_data_count=4096)
        self.contact_names=[(v,f) for v in vise for f in filters]
        print('VISE_ROBOT_CONTACT_COVERAGE '+json.dumps({'vise':vise,'filters':filters}),flush=True)
        print('VISE_CONTACT_COVERAGE '+json.dumps({'handle':handle,'bench':bench}),flush=True)
        asset=env.unwrapped.scene['vise']
        print('VISE_FORCE_METADATA '+json.dumps({'body_names':asset.body_names,'joint_names':asset.joint_names,
              'masses_kg':asset.root_physx_view.get_masses().numpy().tolist(),
              'friction_properties':asset.root_physx_view.get_dof_friction_properties().numpy().tolist(),
              'friction_mu':MU,'mean_radius_assumption_m':RADIUS,
              'guide_friction_mu':GUIDE_MU,
              'journal_mu':JOURNAL_MU,'journal_radius_m':JOURNAL_RADIUS,
              'model':'symmetric square-thread small-helix Coulomb approximation; previous-step measured axial reaction'}),flush=True)
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
        normal,_,_,_,counts,starts=self.assembly_contact.get_contact_data(dt=dt)
        normal=normal.numpy().copy().reshape(-1);counts=counts.numpy().copy().reshape(-1);starts=starts.numpy().copy().reshape(-1)
        if counts.sum()>=4096 or not np.isfinite(normal).all():raise RuntimeError('Invalid contact buffer')
        contacts={}
        for (body,target),n,start in zip(self.contact_names,counts,starts):
            if n:
                if start<0 or start+n>len(normal):raise RuntimeError('Invalid contact index')
                value=float(np.abs(normal[start:start+n]).sum())
                contacts[body.rsplit('/',1)[1]+':'+target.split('/env_0/')[-1]]=value
        return {'q':q[ids].tolist(),'v':v[ids].tolist(),
                'coupling_error_m':float(q[ids[0]]-.003*q[ids[1]]/(2*math.pi)),
                'handle_bench_normal_N':forces.reshape(-1,3).sum(0).tolist(),
                'incoming_joint_wrenches':incoming.tolist(),
                'projected_joint_forces':projected[ids].tolist(),
                'actuation_forces':actuation[ids].tolist(),'previous_applied_axial_load_N':self.last_load,
                'previous_friction_effort_Nm':self.friction_effort,
                'previous_guide_friction_effort_N':self.guide_effort,
                'contacts_normal_N':contacts,
                'friction_properties':asset.root_physx_view.get_dof_friction_properties().numpy().copy()[0].tolist()}
    def update_friction(self,env,sample):
        import torch
        asset=env.unwrapped.scene['vise']
        sid=asset.joint_names.index(JOINTS[1])
        hid=asset.joint_names.index(JOINTS[2])
        axial=sample['projected_joint_forces'][0]-sample['actuation_forces'][0]
        effort=MU*RADIUS*abs(axial)
        screw_wrench=sample['incoming_joint_wrenches'][asset.body_names.index('screw_01_obj_00')]
        effort+=JOURNAL_MU*JOURNAL_RADIUS*math.hypot(screw_wrench[1],screw_wrench[2])
        # Incoming wrench is in the child joint frame (native tensors API).
        # Source handle joint has identity child rotation and translation axisY.
        incoming=sample['incoming_joint_wrenches'][asset.body_names.index('handle_01_obj_00')]
        normal=math.hypot(incoming[0],incoming[2])
        guide_effort=GUIDE_MU*normal
        if not math.isfinite(effort):raise RuntimeError('Nonfinite friction effort')
        if not math.isfinite(guide_effort):raise RuntimeError('Nonfinite guide friction effort')
        value=torch.tensor([[effort,guide_effort]],dtype=torch.float32,device=asset.device)
        asset.write_joint_friction_coefficient_to_sim_index(
            joint_friction_coeff=value,joint_dynamic_friction_coeff=value,
            joint_viscous_friction_coeff=torch.zeros_like(value),joint_ids=[sid,hid])
        actual=asset.root_physx_view.get_dof_friction_properties().numpy()[0,[sid,hid]]
        if not np.allclose(actual,[[effort,effort,0.],[guide_effort,guide_effort,0.]],rtol=1e-5,atol=1e-7):
            raise RuntimeError('Friction effort readback mismatch '+str(actual))
        self.friction_effort=effort
        self.guide_effort=guide_effort
    def physics(self,env,dt):
        if self.contact is not None:
            sample=self.capture(env,dt)
            self.samples.append(sample)
            self.update_friction(env,sample)
    def observe(self,env):
        import torch
        self.step+=1
        if self.contact is None:self.initialize(env)
        else:
            self.samples.append(self.capture(env,env.unwrapped.physics_dt))
            expected=round(env.unwrapped.step_dt/env.unwrapped.physics_dt)+1
            if len(self.samples)!=expected:raise RuntimeError('Physics observation count '+str((len(self.samples),expected)))
        self.latest=self.samples[-1] if self.samples else None
        print('VISE_ROBOT '+json.dumps({'step':self.step,'physics_dt':float(env.unwrapped.physics_dt),
              'samples':self.samples,
              'sample_convention':'First sample repeats previous boundary; exclude it from interval impulse/work sums.'}),flush=True)
        self.samples.clear()


PROBE=Probe();PROBE.reset()
on_physics_step(PROBE.physics);m.on_reset(PROBE.reset);m.on_step(PROBE.observe)
demo=cl.placeholder_demo
def demo_script(ctx):
    from yam_arena import ik
    plan=json.loads((HERE/'vise-robot-plan.json').read_text())
    arm=ctx.right
    for row in plan['poses']:
        PROBE.phase=row['phase']
        arm.q=np.asarray(row['q']);arm.finger=row['finger']
        arm.pos,arm.rot=ik.tcp_pose(arm.q,arm.base_pos)
        yield
        if row.get('checkpoint'):
            print('VISE_GRASP_CHECK '+json.dumps({'phase':row['phase'],
                  'target_tcp':arm.pos.tolist(),'measured_tcp':arm.measured_pos.tolist(),
                  'gap':arm.gap(),'state':PROBE.latest}),flush=True)
            if row['checkpoint']=='grasp' and not ctx.offline:
                contacts=PROBE.latest['contacts_normal_N']
                bilateral=[sum(v for k,v in contacts.items() if k.startswith('handle_01_obj_00:') and '/RightArm/' in '/'+k and k.endswith('/'+tip)) for tip in ('tip_left','tip_right')]
                if min(bilateral)<.5:
                    print('VISE_GRASP_STOP '+json.dumps({'reason':'No bilateral handle contact','bilateral_N':bilateral}),flush=True)
                    return
def score(objects):return 0.
def success(objects):return False
