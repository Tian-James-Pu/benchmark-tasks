"""Read-only native contact/pose sampling through insertion, with a liner-only control guard."""
import json
import numpy as np
import mechkit as m
import controller_fixed_bore as c
from physics_observer import on_physics_step
from support_contacts_guarded_draft import ContactWitness,CAPACITY


def array(value):
    return np.asarray(value.numpy() if hasattr(value,'numpy') else m._np(value)).copy()


class NativeMouthAudit:
    def __init__(self):
        self.reset();m.on_reset(self.reset);m.on_step(self.step);on_physics_step(self.capture)
    def reset(self,env=None):
        self.view=None;self.rows=[];self.ready=False;self.last={};self.serial=0;self.shape=(13,1)
        if env is not None:self.initialize(env)
    pair_sums=ContactWitness.pair_sums
    def initialize(self,env):
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics
        import isaaclab.sim as sim_utils
        prims=list(sim_utils.get_current_stage().Traverse());prefix='/World/envs/env_0/'
        bodies=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and '/new_ferrule/' in str(p.GetPath())]
        shaft=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith(prefix+'crutch')]
        if len(bodies)!=13 or len(shaft)!=1:raise RuntimeError('Native mouth audit expected13foot and1crutch body')
        self.frame_index=next(i for i,p in enumerate(bodies) if p.rsplit('/',1)[-1]=='frame')
        self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(bodies,filter_patterns=[shaft]*13,max_contact_data_count=CAPACITY)
        self.foot=env.unwrapped.scene['new_ferrule'];self.shaft=env.unwrapped.scene['crutch']
        print('NATIVE_MOUTH_FILTERS',json.dumps({'contact_bodies':bodies,'shaft':shaft,'foot_body_names':self.foot.body_names,'foot_joint_names':self.foot.joint_names}),flush=True)
    def sample(self,env,dt,boundary=False):
        if self.view is None:return
        normal,points,normals,separations,counts,starts=self.view.get_contact_data(dt=dt)
        nc=array(counts);ns=array(starts);nv=array(normal).reshape(-1);npnt=array(points).reshape(-1,3);nn=array(normals).reshape(-1,3);sep=array(separations).reshape(-1)
        self.pair_sums(np.abs(nv),nc,ns)
        friction,friction_points,counts,starts=self.view.get_friction_data(dt=dt)
        fc=array(counts);fs=array(starts);fv=array(friction).reshape(-1,3);fp=array(friction_points).reshape(-1,3)
        self.pair_sums(np.linalg.norm(fv,axis=1),fc,fs)
        contacts=[]
        for i in range(13):
            na,nb=int(ns[i,0]),int(nc[i,0]);fa,fb=int(fs[i,0]),int(fc[i,0])
            if not nb and not fb:continue
            contacts.append({'body_index':i,'normal_force_n':nv[na:na+nb].tolist(),'points_world_m':npnt[na:na+nb].tolist(),'normals':nn[na:na+nb].tolist(),'separation_m':sep[na:na+nb].tolist(),'friction_force_n':fv[fa:fa+fb].tolist(),'friction_points_world_m':fp[fa:fa+fb].tolist()})
        foot_view=self.foot.data._root_view;shaft_view=self.shaft.data._root_view
        row={'serial':self.serial,'controller_step':c.COUNT,'boundary':boundary,'dt':dt,'shaft_transform_xyzw':array(shaft_view.get_transforms()).reshape(-1,7)[0].tolist(),'foot_link_transforms_xyzw':array(foot_view.get_link_transforms()).reshape(-1,7).tolist(),'foot_joint_positions_m':array(foot_view.get_dof_positions()).reshape(-1).tolist(),'contacts':contacts}
        if not all(np.isfinite(np.asarray(row[k])).all() for k in ('shaft_transform_xyzw','foot_link_transforms_xyzw','foot_joint_positions_m')):raise RuntimeError('Invalid native mouth pose data')
        self.serial+=1;self.rows.append(row)
    def capture(self,env,dt):
        if 0<=c.COUNT<=1 or c.INSERT_AT-6<=c.COUNT<=c.INSTALLED_AT+20:self.sample(env,dt)
    def step(self,env):
        if self.view is None:self.initialize(env)
        if 1<=c.COUNT<=2 or c.INSERT_AT-5<=c.COUNT<=c.INSTALLED_AT+21:self.sample(env,env.unwrapped.physics_dt,True)
        if self.rows:
            if len(self.rows)!=25:raise RuntimeError(f'Expected25native mouth readings, got{len(self.rows)}')
            frame_peak=0.;liner_min=float('inf')
            for row in self.rows:
                totals=np.zeros(13)
                for pair in row['contacts']:
                    totals[pair['body_index']]=sum(abs(x) for x in pair['normal_force_n'])+sum(float(np.linalg.norm(x)) for x in pair['friction_force_n'])
                if not np.isfinite(totals).all():raise RuntimeError('Nonfinite native liner contact magnitude')
                frame_peak=max(frame_peak,float(totals[self.frame_index]));liner_min=min(liner_min,float(totals.sum()-totals[self.frame_index]))
            self.last={'step':c.COUNT,'readings':len(self.rows),'fixed_frame_peak_n':frame_peak,'liner_min_n':liner_min,'liner_only':bool(frame_peak<=1e-4 and liner_min>.8)};self.ready=True
            print('NATIVE_LINER_GATE',c.COUNT,json.dumps(self.last),flush=True)
            print('NATIVE_MOUTH_SAMPLES',c.COUNT,json.dumps(self.rows),flush=True);self.rows.clear()

AUDIT=NativeMouthAudit()

c.NATIVE=AUDIT
