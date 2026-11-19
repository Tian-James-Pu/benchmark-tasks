"""Read-only support witness for the ferrule and full crutch.

Samples every physics step and once at the control boundary. Individual normal
and friction magnitudes complement the net matrix so opposing contacts cannot
cancel. The 1e-4 N numerical zero is 1/4905 of the measured 50 g foot weight;
shaft-only control readings were exactly zero, bench support was 0.49050 N.
This is a simulation tolerance, not a claim of physical load certification.
"""
import json
import numpy as np
import mechkit as m
from physics_observer import on_physics_step
from contact_support_graph import robot_contact_path
CONTACT_EPS_N=1e-4
CAPACITY=32768
EXPECTED_READINGS=25  # 24 physics steps at480Hz per20Hz control interval, plus boundary

def reported(part):
    original=part.build
    def build():
        from pxr import Sdf,Usd,UsdPhysics,PhysxSchema
        layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(original());stage=Usd.Stage.Open(layer)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        return stage.GetRootLayer().ExportToString()
    from dataclasses import replace
    return replace(part,build=build)

def enable_prop_reporting():
    """Add only contact-report metadata after the normal SimReady preparation."""
    import simready
    if getattr(simready._prepare,'_ferrule_contact_reporting',False):return
    original=simready._prepare
    def prepare(prim,spec):
        original(prim,spec)
        from pxr import Usd,UsdPhysics,PhysxSchema
        bodies=[p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(bodies)!=1:raise RuntimeError('Expected one original SimReady rigid body')
        PhysxSchema.PhysxContactReportAPI.Apply(bodies[0]).CreateThresholdAttr().Set(0.)
    prepare._ferrule_contact_reporting=True
    simready._prepare=prepare

class ContactWitness:
    def __init__(self):
        enable_prop_reporting()
        self.reset();m.on_reset(self.reset);m.on_step(self.step);on_physics_step(self.capture)
    def reset(self,env=None):
        self.view=None;self.prop_view=None;self.ready=False;self.steps=0;self.readings=0
        self.foot_peak=0.;self.robot_peak=0.;self.point_peak=0
        self.path_in_interval=None;self.prop_point_peak=0
        self.last={'foot_external_n':None,'assembly_robot_n':None,'readings':0,'assembly_robot_connected':None,'robot_contact_path':None}
        if env is not None:self.initialize(env)
    def initialize(self,env):
        import isaaclab.sim as sim_utils
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics
        prims=list(sim_utils.get_current_stage().Traverse());prefix='/World/envs/env_0/'
        foot=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and '/new_ferrule/' in str(p.GetPath())]
        crutch=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith(prefix+'crutch')]
        if len(foot)!=13 or len(crutch)!=1:raise RuntimeError(f'Expected13foot/1crutch bodies, got{len(foot)}/{len(crutch)}')
        self.bodies=foot+crutch;self.foot_count=len(foot);self.filters=[]
        for prim in prims:
            path=str(prim.GetPath())
            if not path.startswith(prefix) or path.startswith(prefix+'new_ferrule') or path.startswith(prefix+'crutch'):continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):self.filters.append(path)
            elif prim.HasAPI(UsdPhysics.CollisionAPI):
                parent=prim.GetParent();rigid=False
                while parent and str(parent.GetPath()).startswith(prefix):
                    if parent.HasAPI(UsdPhysics.RigidBodyAPI):rigid=True;break
                    parent=parent.GetParent()
                if not rigid:self.filters.append(path)
        self.robot_mask=np.array(['/LeftArm/' in p or '/RightArm/' in p for p in self.filters])
        if int(self.robot_mask.sum())!=18:raise RuntimeError('Expectedall18originalrobot bodies in support filter')
        self.shape=(len(self.bodies),len(self.filters))
        self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(self.bodies,filter_patterns=[self.filters]*len(self.bodies),max_contact_data_count=CAPACITY)
        from pxr import PhysxSchema
        stage=sim_utils.get_current_stage()
        roots=('old_ferrule','blue_marker','slotted_screwdriver')
        self.prop_bodies=[p for p in self.filters if any(p.startswith(prefix+r+'/') for r in roots)]
        counts=[sum(p.startswith(prefix+r+'/') for p in self.prop_bodies) for r in roots]
        if counts!=[13,1,1]:raise RuntimeError(f'Incomplete loose-object graph coverage: {counts}')
        self.robot_bodies={p for p,is_robot in zip(self.filters,self.robot_mask) if is_robot}
        self.dynamic_bodies=set(self.bodies+self.prop_bodies)
        self.fixed_bodies=set(self.filters)-self.robot_bodies-set(self.prop_bodies)
        if any(stage.GetPrimAtPath(p).HasAPI(UsdPhysics.RigidBodyAPI) for p in self.fixed_bodies):
            raise RuntimeError('Unclassified rigid body in contact graph')
        for path in self.prop_bodies:
            prim=stage.GetPrimAtPath(path)
            if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI) or prim.GetAttribute('physxContactReport:threshold').Get()!=0.:
                raise RuntimeError(f'Missing native contact reporting: {path}')
        self.mechanical_edges=[]
        for prim in prims:
            if not str(prim.GetPath()).startswith(prefix+'old_ferrule/') or not prim.IsA(UsdPhysics.Joint):continue
            joint=UsdPhysics.Joint(prim);a=joint.GetBody0Rel().GetTargets();b=joint.GetBody1Rel().GetTargets()
            if len(a)!=1 or len(b)!=1 or str(a[0]) not in self.prop_bodies or str(b[0]) not in self.prop_bodies:
                raise RuntimeError('Invalid old-foot mechanical edge')
            if not joint.GetJointEnabledAttr().Get() or joint.GetBreakForceAttr().Get()<1e20 or joint.GetBreakTorqueAttr().Get()<1e20:
                raise RuntimeError('Expected enabled unbreakable old-foot joints')
            self.mechanical_edges.append((str(a[0]),str(b[0])))
        old={p for p in self.prop_bodies if p.startswith(prefix+'old_ferrule/')}
        connected={next(iter(old))}
        for _ in old:
            connected.update(b for a,b in self.mechanical_edges if a in connected)
            connected.update(a for a,b in self.mechanical_edges if b in connected)
        if len(self.mechanical_edges)!=12 or connected!=old:raise RuntimeError('Old-foot mechanical graph is disconnected')
        self.prop_filters=self.prop_bodies+sorted(self.robot_bodies)
        self.prop_shape=(len(self.prop_bodies),len(self.prop_filters))
        self.prop_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(self.prop_bodies,filter_patterns=[self.prop_filters]*len(self.prop_bodies),max_contact_data_count=CAPACITY)
        print('SUPPORT_GRAPH_COVERAGE',json.dumps({'prop_bodies':self.prop_bodies,'prop_filters':self.prop_filters,'fixed_stops':sorted(self.fixed_bodies),'mechanical_edges':self.mechanical_edges}),flush=True)
        print('SUPPORT_WITNESS_FILTERS',json.dumps({'bodies':self.bodies,'filters':self.filters}),flush=True)
    def pair_sums(self,values,counts,starts,shape=None):
        shape=self.shape if shape is None else shape
        if counts.shape!=shape or starts.shape!=shape:raise RuntimeError('Unexpected contact indexing shape')
        if not np.issubdtype(counts.dtype,np.integer) or not np.issubdtype(starts.dtype,np.integer):raise RuntimeError('Contact indices must be integers')
        occupied=counts>0
        intervals=sorted((int(start),int(start)+int(count)) for start,count in zip(starts[occupied],counts[occupied]))
        for previous,current in zip(intervals,intervals[1:]):
            if current[0]<previous[1]:raise RuntimeError('Overlapping contact data ranges')
        if np.any(counts<0) or int(counts.sum())>=CAPACITY:raise RuntimeError('Contactbuffer capacity exceeded')
        out=np.zeros(shape)
        for i,j in np.argwhere(counts>0):
            start=int(starts[i,j]);count=int(counts[i,j])
            if start<0 or start+count>len(values):raise RuntimeError('Invalidcontact data range')
            out[i,j]=float(values[start:start+count].sum())
        return out
    def prop_measure(self,dt):
        net=np.asarray(self.prop_view.get_contact_force_matrix(dt=dt).numpy(),float)
        if net.shape!=(*self.prop_shape,3) or not np.isfinite(net).all():raise RuntimeError('Invalid prop contact matrix')
        normal,_,_,_,counts,starts=self.prop_view.get_contact_data(dt=dt)
        nc=np.asarray(counts.numpy()).copy();ns=np.asarray(starts.numpy()).copy();nv=np.abs(np.asarray(normal.numpy(),float).reshape(-1))
        normal_sum=self.pair_sums(nv,nc,ns,self.prop_shape)
        friction,_,counts,starts=self.prop_view.get_friction_data(dt=dt)
        fc=np.asarray(counts.numpy()).copy();fs=np.asarray(starts.numpy()).copy();fv=np.linalg.norm(np.asarray(friction.numpy(),float).reshape(-1,3),axis=1)
        friction_sum=self.pair_sums(fv,fc,fs,self.prop_shape)
        measure=np.maximum(np.linalg.norm(net,axis=2),np.maximum(normal_sum,friction_sum))
        if not np.isfinite(measure).all():raise RuntimeError('Nonfinite prop contact magnitude')
        self.prop_point_peak=max(self.prop_point_peak,int(nc.sum()))
        return measure
    def capture(self,env,dt):
        if self.view is None:return
        net=np.asarray(self.view.get_contact_force_matrix(dt=dt).numpy(),float)
        if net.shape!=(*self.shape,3) or not np.isfinite(net).all():raise RuntimeError('Invalidcontact force matrix')
        normal,_,_,_,counts,starts=self.view.get_contact_data(dt=dt)
        # The friction getter reuses PhysX index buffers: copy normal indices first.
        nc=np.asarray(counts.numpy()).copy();ns=np.asarray(starts.numpy()).copy();nv=np.abs(np.asarray(normal.numpy(),float).reshape(-1))
        normal_sum=self.pair_sums(nv,nc,ns)
        friction,_,counts,starts=self.view.get_friction_data(dt=dt)
        fc=np.asarray(counts.numpy()).copy();fs=np.asarray(starts.numpy()).copy();fv=np.linalg.norm(np.asarray(friction.numpy(),float).reshape(-1,3),axis=1)
        friction_sum=self.pair_sums(fv,fc,fs)
        measure=np.maximum(np.linalg.norm(net,axis=2),np.maximum(normal_sum,friction_sum))
        if not np.isfinite(measure).all():raise RuntimeError('Nonfinitecontact magnitude')
        self.foot_peak=max(self.foot_peak,float(measure[:self.foot_count].sum()))
        self.robot_peak=max(self.robot_peak,float(measure[:,self.robot_mask].sum()))
        prop_measure=self.prop_measure(dt)
        edges=[(self.bodies[i],self.filters[j],float(measure[i,j])) for i,j in np.argwhere(measure>CONTACT_EPS_N)]
        edges.extend((self.prop_bodies[i],self.prop_filters[j],float(prop_measure[i,j])) for i,j in np.argwhere(prop_measure>CONTACT_EPS_N))
        path=robot_contact_path(assembly=self.bodies,dynamic=self.dynamic_bodies,robots=self.robot_bodies,fixed=self.fixed_bodies,mechanical_edges=self.mechanical_edges,contacts=edges,epsilon=CONTACT_EPS_N)
        if path is not None and self.path_in_interval is None:self.path_in_interval=path
        self.point_peak=max(self.point_peak,int(nc.sum()));self.readings+=1
    def step(self,env):
        self.steps+=1
        if self.view is None:self.initialize(env)
        self.capture(env,env.unwrapped.physics_dt)
        if self.readings!=EXPECTED_READINGS:raise RuntimeError(f'Incomplete contact observation: expected {EXPECTED_READINGS} readings, got {self.readings}')
        self.ready=True
        self.last={'foot_external_n':self.foot_peak,'assembly_robot_n':self.robot_peak,'readings':self.readings,'maximum_contact_points':self.point_peak,'assembly_robot_connected':self.path_in_interval is not None,'robot_contact_path':self.path_in_interval,'prop_maximum_contact_points':self.prop_point_peak}
        if self.steps==1 or self.steps%10==0:print('SUPPORT_WITNESS',self.steps,json.dumps(self.last),flush=True)
        self.foot_peak=0.;self.robot_peak=0.;self.point_peak=0;self.readings=0;self.path_in_interval=None;self.prop_point_peak=0
    @property
    def external_free(self):return self.ready and self.last['foot_external_n']<=CONTACT_EPS_N
    @property
    def robot_free(self):return self.ready and self.last['assembly_robot_n']<=CONTACT_EPS_N and not self.last['assembly_robot_connected']
