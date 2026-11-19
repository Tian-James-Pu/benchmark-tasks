"""Read-only all-link contact and actuator audit for bounded pose correction."""
import json
import numpy as np
import mechkit as m
from physics_observer import on_physics_step
from support_contacts_guarded_draft import ContactWitness,CAPACITY,CONTACT_EPS_N

class WholeArmAudit:
    pair_sums=ContactWitness.pair_sums
    def __init__(self):
        self.reset();m.on_reset(self.reset);m.on_step(self.step);on_physics_step(self.capture)
    def reset(self,env=None):
        self.view=None;self.steps=0;self.readings=0;self.last={};self.ready=False
        if env is not None:self.initialize(env)
    def initialize(self,env):
        import isaaclab.sim as sim_utils
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics
        prefix='/World/envs/env_0/';prims=list(sim_utils.get_current_stage().Traverse())
        self.bodies=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and any('/'+side+'/' in str(p.GetPath()) for side in ['LeftArm','RightArm'])]
        if len(self.bodies)!=18:raise RuntimeError('Expected18 native robot bodies')
        self.filters=[]
        for prim in prims:
            path=str(prim.GetPath())
            if not path.startswith(prefix):continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):self.filters.append(path)
            elif prim.HasAPI(UsdPhysics.CollisionAPI):
                parent=prim.GetParent();rigid=False
                while parent and str(parent.GetPath()).startswith(prefix):
                    if parent.HasAPI(UsdPhysics.RigidBodyAPI):rigid=True;break
                    parent=parent.GetParent()
                if not rigid:self.filters.append(path)
        self.shape=(len(self.bodies),len(self.filters));self.peak=np.zeros(self.shape)
        self.allowed=np.zeros(self.shape,bool);self.grip=np.zeros(self.shape,bool)
        for i,body in enumerate(self.bodies):
            for j,other in enumerate(self.filters):
                finger=body.endswith(('/tip_left','/tip_right')) and other==prefix+'crutch'
                mount=body.endswith('/Geometry/base') and other.startswith((prefix+'table/',prefix+'floor/'))
                self.allowed[i,j]=finger or mount;self.grip[i,j]=finger
        self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(self.bodies,filter_patterns=[self.filters]*len(self.bodies),max_contact_data_count=CAPACITY)
        print('WHOLE_ARM_AUDIT_FILTERS',json.dumps({'bodies':self.bodies,'filters':self.filters}),flush=True)
    def capture(self,env,dt):
        if self.view is None:return
        net=np.asarray(self.view.get_contact_force_matrix(dt=dt).numpy(),float).copy()
        if net.shape!=(*self.shape,3) or not np.isfinite(net).all():raise RuntimeError('Invalid whole-arm contact matrix')
        normal,_,_,_,counts,starts=self.view.get_contact_data(dt=dt)
        nc=np.asarray(counts.numpy()).copy();ns=np.asarray(starts.numpy()).copy();normal=self.pair_sums(np.abs(np.asarray(normal.numpy(),float).reshape(-1)),nc,ns)
        friction,_,counts,starts=self.view.get_friction_data(dt=dt)
        fc=np.asarray(counts.numpy()).copy();fs=np.asarray(starts.numpy()).copy();friction=self.pair_sums(np.linalg.norm(np.asarray(friction.numpy(),float).reshape(-1,3),axis=1),fc,fs)
        measure=np.maximum(np.linalg.norm(net,axis=2),np.maximum(normal,friction))
        if not np.isfinite(measure).all():raise RuntimeError('Nonfinite whole-arm contact magnitude')
        np.maximum(self.peak,measure,out=self.peak);self.readings+=1
    def step(self,env):
        self.steps+=1
        if self.view is None:self.initialize(env)
        self.capture(env,env.unwrapped.physics_dt)
        if self.readings!=25:raise RuntimeError(f'Expected25 whole-arm samples, got{self.readings}')
        arms={};safe=True
        for side in ['left','right']:
            rob=env.unwrapped.scene[side+'_robot'];ids=rob.find_joints([f'joint{i}' for i in range(1,7)],preserve_order=True)[0];q=m._np(rob.data.joint_pos)[0,ids];limits=m._np(rob.data.joint_pos_limits)[0,ids];term=env.unwrapped.action_manager.get_term(side+'_arm');target=m._np(term.processed_actions)[0]
            torque=m._np(rob.data.applied_torque)[0,ids];caps=m._np(rob.data.joint_effort_limits)[0,ids];margin=float(np.minimum(q-limits[:,0],limits[:,1]-q).min());ratio=float(np.max(np.abs(torque)/caps))
            if not all(np.isfinite(v).all() for v in [q,target,torque,caps,limits]):raise RuntimeError('Nonfinite actuator telemetry')
            arms[side]={'actual_q':q.tolist(),'target_q':target.tolist(),'error_deg':np.degrees(target-q).tolist(),'applied_torque':torque.tolist(),'effort_limits':caps.tolist(),'minimum_joint_margin_rad':margin,'maximum_effort_ratio':ratio};safe=safe and margin>.10 and ratio<.8
        unexpected=self.peak.copy();unexpected[self.allowed]=0.;maximum=float(unexpected.max());pairs=[]
        for flat in np.argsort(self.peak.ravel())[-12:][::-1]:
            i,j=np.unravel_index(flat,self.shape)
            if self.peak[i,j]>CONTACT_EPS_N:pairs.append({'body':self.bodies[i],'other':self.filters[j],'force_measure_n':float(self.peak[i,j]),'allowed_grip_or_mount':bool(self.allowed[i,j])})
        grips={}
        for side,token in [('left','/LeftArm/'),('right','/RightArm/')]:
            rows=np.array([token in body for body in self.bodies]);grips[side]=float(np.where(self.grip[rows],self.peak[rows],0.).sum())
        self.ready=True;self.last={'step':self.steps,'readings':self.readings,'arms':arms,'maximum_unexpected_contact_n':maximum,'grip_force_by_side_n':grips,'correction_allowed':bool(safe and maximum<=CONTACT_EPS_N and min(grips.values())>10.),'peak_pairs':pairs}
        if self.steps==1 or self.steps%20==0:print('WHOLE_ARM_AUDIT',self.steps,json.dumps(self.last),flush=True)
        self.peak[:]=0.;self.readings=0
