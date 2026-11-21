"""Native state/contact adapter for recipe_scoring; no task object pose writes.

Buoyancy is an explicit fluid approximation; robot, food and cookware contacts
remain native PhysX. Contact normals are inspected separately before summing
positive upward support, so opposing contacts cannot cancel into a false float.
"""
import json,math
import numpy as np
import batch_physics as b
from heat_model import Heat,circle_overlap,HOT_C
from recipe_scoring import RecipeScore,FoodObservation,FinishObservation
from contact_rules import classify,symmetric_rows,is_robot_actor,normalize_signed_rows

class Observe:
    def __init__(self,config):self.c=config
    def before_reset(self,env):
        if getattr(self,'sim',None) is not None:
            self.sim.step=self.original_step
            self.sim=None
    def reset(self,env):
        if getattr(self,'sim',None) is not None:self.sim.step=self.original_step
        if getattr(self,'trace',None) is not None:self.trace.close()
        #Genuine episode reset only: restore the declared passive dial configuration.
        #Installed Arena leaves these custom articulation joints at authoredUSD zero.
        #No joint state is written during trial dynamics.
        reset_joints={}
        for name,value in (('front_control',math.radians(150)),('rear_control',0.)):
            asset=env.unwrapped.scene[name]
            q=asset.data.joint_pos.torch.clone();v=asset.data.joint_vel.torch.clone()
            reset_joints[name]={'before':q.cpu().tolist(),'declared_default':asset.data.default_joint_pos.torch.cpu().tolist(),'cfg':asset.cfg.init_state.joint_pos}
            j=asset.joint_names.index('dial_joint');q[:,j]=value;v[:]=0.
            asset.write_joint_position_to_sim_index(position=q)
            asset.write_joint_velocity_to_sim_index(velocity=v)
            reset_joints[name]['after']=asset.data.joint_pos.torch.cpu().tolist()
        (self.c.ROOT/'reset-joints.json').write_text(json.dumps(reset_joints,indent=2)+'\n')
        self.env=env;self.sim=None;self.initialized=False;self.t=0.;self.step_n=0;self.last_observation_t=0.
        self.age=np.full(b.N,b.INITIAL_AGE);self.heat=Heat(self.c.WATER_MASS)
        self.rules=RecipeScore(b.N);self.wet=np.ones(b.N);self.float_hold=np.zeros(b.N)
        self.front=math.radians(150);self.rear=0.;self.spilled=0.;self.actor_cache={}
        self.trace=(self.c.ROOT/'trace.jsonl').open('w');self.last_score=0.
        for attr in ('pose','water_prim'):
            if hasattr(self,attr):delattr(self,attr)
    def initialize(self,env):
        from isaaclab_physx.physics import PhysxManager
        import warp as wp
        self.wp=wp;sv=PhysxManager.get_physics_sim_view()
        self.view=sv.create_rigid_body_view('/World/envs/env_0/gnocchi_*')
        self.paths=list(self.view.prim_paths);assert len(self.paths)==b.N,self.paths
        self.order=np.array([int(p.rsplit('_',1)[-1]) for p in self.paths]);self.inverse=np.argsort(self.order)
        self.food_paths=[self.paths[k] for k in self.inverse]
        self.panview=sv.create_rigid_body_view('/World/envs/env_0/pan')
        self.native_free_mass_kg={name:sv.create_rigid_body_view('/World/envs/env_0/'+name).get_masses().numpy().tolist() for name in ('spoon','plate','pan')}
        self.panindex=wp.array([0],dtype=wp.int32,device='cuda:0')
        self.pan_property_index=wp.array([0],dtype=wp.uint32,device='cpu')
        self.applied_pan_mass=self.c.PAN_MASS
        if not hasattr(self,'initial_pan_inertia'):self.initial_pan_inertia=self.panview.get_inertias().numpy().copy()
        self.panview.set_masses(wp.array(np.array([[self.c.PAN_MASS]],dtype=np.float32),device='cpu'),self.pan_property_index)
        self.panview.set_inertias(wp.array(self.initial_pan_inertia.astype(np.float32),device='cpu'),self.pan_property_index)
        from yam_arena.robot import asset_link_paths
        #Both robots also register ContactReportAPI. Include every native link so
        #a one-sided event owned by an articulated robot cannot disappear.
        robot_paths=[]
        for side in ('left','right'):
            scene=env.unwrapped.scene
            root=scene[f'{side}_robot'].cfg.prim_path.replace(scene.env_regex_ns,scene.env_prim_paths[0]).replace('{ENV_REGEX_NS}',scene.env_prim_paths[0])
            robot_paths.extend(root+rel for rel in asset_link_paths().values())
        control_paths=[f'/World/envs/env_0/{name}_control/dial' for name in ('front','rear')]
        required=self.food_paths+['/World/envs/env_0/spoon','/World/envs/env_0/pan','/World/envs/env_0/plate']+control_paths+robot_paths
        self.contacts=sv.create_rigid_contact_view(required,max_contact_data_count=16384)
        self.sensor_names=[p.split('/env_0/',1)[1] if is_robot_actor(p) or '_control/' in p else p.rsplit('/',1)[-1] for p in self.contacts.sensor_paths]
        self.task_sensor_names=self.c.G_NAMES+['spoon','pan','plate','front_control/dial','rear_control/dial']
        assert set(self.contacts.sensor_paths)==set(required),self.contacts.sensor_paths
        assert set(self.task_sensor_names).issubset(self.sensor_names)
        self.dt=env.unwrapped.physics_dt
        self.indices=wp.array(np.arange(b.N,dtype=np.int32),dtype=wp.int32,device='cuda:0')
        self.sim=env.unwrapped.sim;self.original_step=self.sim.step
        def step(*args,**kwargs):
            self.physics();return self.original_step(*args,**kwargs)
        self.sim.step=step
        schema={'paths':self.paths,'sensors':self.contacts.sensor_paths,'physics_dt':self.dt,'mass_kg':self.view.get_masses().numpy().tolist(),'pan_mass_kg':self.panview.get_masses().numpy().tolist(),'contact_offsets_m':.0002,'force_application':'Independent Archimedes/drag food wrench and equal opposite pan reaction. No pose/velocity writes or kinematic attachments.','contact_method':'get_raw_contact_data, positive vertical normal support per actor; cached native actor ID-to-path mapping','source_dimensions_mm':[322.58,71.12,44.45],'batch_mass_kg':b.MASS*b.N}
        schema['native_free_mass_kg']=self.native_free_mass_kg
        (self.c.ROOT/'native-schema.json').write_text(json.dumps(schema,indent=2)+'\n');self.initialized=True
    def physics(self):
        c=self.c;pose=self.view.get_transforms().numpy()[self.inverse];vel=self.view.get_velocities().numpy()[self.inverse]
        pp=self.panview.get_transforms().numpy()[0];pv=self.panview.get_velocities().numpy()[0]
        R=b.rotations(pp[None,3:7])[0];area=np.pi*c.RADIUS**2
        #Flat free surface, with food displacement and immediate rim overflow.
        #The unmodeled slosh boundary is disclosed; spilling does not produce reward.
        rho=b.RAW_DENSITY+(b.FLOAT_DENSITY-b.RAW_DENSITY)*np.clip((self.age-b.FLOAT_AGES)/2,0,1)
        displaced=float(np.sum(self.wet*b.VOLUME*b.RAW_DENSITY/rho))
        depth=(self.heat.water_mass/1000.+displaced)/area
        surface=pp[2]+R[2,2]*(.005+depth)
        low_rim=pp[2]+R[2,2]*.065-c.RADIUS*np.linalg.norm(R[2,:2])
        if R[2,2]<.5:
            self.spilled+=self.heat.water_mass;self.heat.water_mass=0.
        elif surface>low_rim:
            dm=min(self.heat.water_mass,(surface-low_rim)*area/R[2,2]*1000.)
            self.heat.water_mass-=dm;self.spilled+=dm;surface=low_rim
        #Actual pan contact/height and area overlap determine powered heat input.
        on_hob=abs(pp[2]-c.RING_TOP)<.003 and R[2,2]>math.cos(math.radians(5))
        overlap=[]
        for center,radius in ((c.FRONT_ZONE,.0725),(c.REAR_ZONE,.09)):
            overlap.append(circle_overlap(.098,radius,np.linalg.norm(pp[:2]-center))/(np.pi*radius**2) if on_hob else 0.)
        hot=self.heat.advance(self.dt,self.front,overlap[0],self.rear,overlap[1])
        current_mass=1.1+self.heat.water_mass
        if abs(current_mass-self.applied_pan_mass)>=.001:
            self.panview.set_masses(self.wp.array(np.array([[current_mass]],dtype=np.float32),device='cpu'),self.pan_property_index)
            #Approximate inertia follows remaining mass; no pose or velocity write.
            inertia=self.initial_pan_inertia*(current_mass/self.c.PAN_MASS)
            self.panview.set_inertias(self.wp.array(inertia.astype(np.float32),device='cpu'),self.pan_property_index)
            self.applied_pan_mass=current_mass
        F,T,wet=b.water_forces(pose,vel,self.age,surface,pp[:2],c.RADIUS,pp[2]+.005,pp,pv)
        if self.heat.water_mass<=.001:F[:]=0.;T[:]=0.;wet[:]=0.
        self.age+=self.dt*((wet>=.75)&hot)
        self.view.apply_forces_and_torques_at_position(self.wp.array(F[self.order].astype(np.float32),device='cuda:0'),self.wp.array(T[self.order].astype(np.float32),device='cuda:0'),None,self.indices,True)
        #Water's reaction is carried by the loose pan; it is not anchored to world.
        reaction=-F.sum(axis=0);com=pp[:3]+R@np.array([.010,0,.025])
        moment=-(np.cross(pose[:,:3]-com,F)+T).sum(axis=0)
        self.panview.apply_forces_and_torques_at_position(self.wp.array(reaction[None].astype(np.float32),device='cuda:0'),self.wp.array(moment[None].astype(np.float32),device='cuda:0'),None,self.panindex,True)
        self.pose=pose;self.vel=vel;self.wet=wet;self.surface=surface;self.pan_pose=pp;self.pan_vel=pv;self.t+=self.dt
        height=np.sqrt(np.sum((b.rotations(pose[:,3:7])[:,2,:]*b.HALF)**2,axis=1));self.low=pose[:,2]-height;self.top=pose[:,2]+height
        if not np.isfinite(pose).all() or np.max(np.linalg.norm(vel[:,:3],axis=1))>3:raise RuntimeError('Native food instability')
    def contact_rows(self):
        force,point,normal,separation,count,start,actor=self.contacts.get_raw_contact_data(self.dt)
        f=force.numpy().reshape(-1);p=point.numpy();n=normal.numpy();cnt=count.numpy().reshape(-1);beg=start.numpy().reshape(-1);ids=actor.numpy().reshape(-1)
        limit=self.contacts.max_contact_data_count
        used_end=max([int(a)+int(k) for a,k in zip(beg,cnt) if k>0],default=0)
        if int(cnt.sum())>=limit or used_end>=limit:raise RuntimeError('Contact buffer saturated or invalid; scoring unavailable')
        active=np.concatenate([np.arange(int(a),int(a+k)) for a,k in zip(beg,cnt)])
        missing=sorted(set(int(x) for x in ids[active])-self.actor_cache.keys())
        if missing:
            names=self.contacts.get_other_actor_paths_from_ids(self.wp.array(np.array(missing,dtype=np.uint64),dtype=self.wp.uint64,device='cpu'))
            if len(names)!=len(missing) or any(not name for name in names):raise RuntimeError('Unresolved native contact actor; scoring unavailable')
            self.actor_cache.update(zip(missing,names))
        out={}
        for name,a,k in zip(self.sensor_names,beg,cnt):
            out[name]=[(self.actor_cache[int(ids[j])],float(f[j]),n[j],p[j]) for j in range(int(a),int(a+k))]
        self.last_raw_rows=out
        if self.c is not None and self.step_n%200==0:
            sample={name:[{'actor':a,'scalar_force_n':f,'raw_normal':n.tolist(),'point':p.tolist()} for a,f,n,p in contacts] for name,contacts in out.items()}
            net=self.contacts.get_net_contact_forces(self.dt).numpy()
            with (self.c.ROOT/'signed-contact-samples.jsonl').open('a') as stream:stream.write(json.dumps({'step':self.step_n,'time_s':self.t,'rows':sample,'net_force_n':{name:values.tolist() for name,values in zip(self.sensor_names,net)}})+'\n')
        self.signed_contact_counts={'negative':int(np.sum(f[active]<-.00001)),'positive':int(np.sum(f[active]>.00001))}
        return symmetric_rows(normalize_signed_rows(out),dict(zip(self.sensor_names,self.contacts.sensor_paths)))
    def step(self,env):
        c=self.c;m=c.m
        if not self.initialized:self.initialize(env)
        if not hasattr(self,'pose'):self.physics()
        elapsed=self.t-getattr(self,'last_observation_t',0.);self.last_observation_t=self.t
        if elapsed<=0:return
        spoon=m.state(env,'spoon');pan=m.state(env,'pan');plate=m.state(env,'plate')
        self.front=m.state(env,'front_control').joint('dial');self.rear=m.state(env,'rear_control').joint('dial')
        rows=self.contact_rows()
        bowlR=m.quat_matrix(spoon.quat_xyzw);plateR=m.quat_matrix(plate.quat_xyzw)
        support=classify(rows,self.pose,spoon.pos,bowlR,plate.pos,plateR,c.BOWL_X,c.BOWL_Y)
        onplate=support['plate'];onspoon=support['spoon'];tool_supported=support['tool'];total_up=support['up_n'];robot_touch=support['robot']
        #A250ms unassisted surface observation avoids a single bouncing contact gap.
        at_surface=(np.abs(self.top-self.surface)<.004)&(self.wet>=.75)&(self.age>=b.FLOAT_AGES+2)&(total_up<.002)&(~robot_touch)
        self.float_hold=np.where(at_surface,self.float_hold+elapsed,0.)
        still=(np.linalg.norm(self.vel[:,:3],axis=1)<.01)&(np.linalg.norm(self.vel[:,3:],axis=1)<.2)
        obs=[FoodObservation(float(self.wet[i]),self.heat.temp_c,bool(self.float_hold[i]>=.25),bool(onspoon[i]),bool(onplate[i]),bool(still[i]),bool(tool_supported[i] or robot_touch[i]),gripper_carried=bool(robot_touch[i] and not tool_supported[i] and not onplate[i])) for i in range(b.N)]
        def robot_contact(name):return any(is_robot_actor(a) and f>.002 for a,f,n,p in rows[name])
        def supported_on(name,kind):return sum(max(0.,f*float(n[2])) for a,f,n,p in rows[name] if kind(a))>.02
        is_table=lambda a:('table' in a.lower() and not 'robot' in a)
        is_hob=lambda a:('/hob/' in a or a.endswith('/hob'))
        spoon_low=float((c.SPOON_VERTS@bowlR.T+spoon.pos)[:,2].min())
        spoon_rest=(supported_on('spoon',is_table) and abs(spoon_low-c.TOP)<.004 and np.linalg.norm(spoon.lin_vel)<.01 and np.linalg.norm(spoon.ang_vel)<.1 and not robot_contact('spoon'))
        cookware=(supported_on('pan',lambda a:is_table(a) or is_hob(a)) and supported_on('plate',is_table) and pan.tilt_deg()<3. and plate.tilt_deg()<3. and np.linalg.norm(pan.lin_vel)<.01 and np.linalg.norm(plate.lin_vel)<.01 and np.linalg.norm(pan.ang_vel)<.1 and np.linalg.norm(plate.ang_vel)<.1)
        clear=not any(robot_contact(name) for name in self.task_sensor_names)
        finish=FinishObservation(abs(self.front)<math.radians(5) and abs(self.rear)<math.radians(5),bool(spoon_rest),bool(cookware),bool(clear))
        self.last_score=self.rules.step(elapsed,obs,finish)
        from yam_arena.constants import TCP_OFFSET
        robot=env.unwrapped.scene['left_robot']
        grip_index=robot.body_names.index('gripper')
        native_grip=robot.data.body_link_pose_w.torch[0,grip_index].detach().cpu().numpy()
        native_R=m.quat_matrix(native_grip[3:7]);tcp=native_grip[:3]+native_R@np.array(TCP_OFFSET)
        finger_ids=[robot.joint_names.index(n) for n in ('joint7','joint8')]
        finger_q=robot.data.joint_pos.torch[0,finger_ids].detach().cpu().numpy()
        relative_pos=(spoon.pos-tcp)@native_R;relative_R=native_R.T@bowlR
        right_robot=env.unwrapped.scene['right_robot']
        right_pose=right_robot.data.body_link_pose_w.torch[0,right_robot.body_names.index('gripper')].detach().cpu().numpy()
        right_R=m.quat_matrix(right_pose[3:7]);right_tcp=right_pose[:3]+right_R@np.array(TCP_OFFSET)
        right_fingers=right_robot.data.joint_pos.torch[0,[right_robot.joint_names.index(n) for n in ('joint7','joint8')]].detach().cpu().numpy()
        dial=env.unwrapped.scene['front_control']
        dial_poses={name:pose.tolist() for name,pose in zip(dial.body_names,dial.data.body_link_pose_w.torch[0].detach().cpu().numpy())}
        robot_pairs=[{'sensor':name,'other':a,'force_n':f,'normal':n.tolist(),'point':p.tolist()} for name,contacts in rows.items() if is_robot_actor('/'+name) for a,f,n,p in contacts if f>.002 and any(key in a for key in ('/pan','/front_control','/rear_control'))]
        self.update_water_visual()
        row={'step':self.step_n,'time_s':self.t,'phase':c.phase_at(self.step_n/20.),'score':self.last_score,'signed_contact_counts':self.signed_contact_counts,'float_count':sum(self.rules.floated),'ready_count':sum(self.rules.ready),'minimum_post_float_s':min(self.rules.dwell),'maximum_post_float_s':max(self.rules.dwell),'in_spoon':int(onspoon.sum()),'retrieved':sum(self.rules.spooned),'plate_supported':int(onplate.sum()),'settled_good':sum(x>=1 for x in self.rules.stable),'water_c':float(self.heat.temp_c),'water_kg':float(self.heat.water_mass),'spilled_kg':float(self.spilled),'native_pan_mass_kg':float(self.applied_pan_mass),'surface_m':float(self.surface),'spoon_robot_contact_n':float(sum(f for a,f,n,p in rows['spoon'] if is_robot_actor(a))),'food_robot_supported':int(robot_touch.sum()),'front_deg':math.degrees(self.front),'rear_deg':math.degrees(self.rear),'finish':finish.__dict__,'positions':self.pose[:,:3].tolist(),'food_quat_xyzw':self.pose[:,3:7].tolist(),'food_linear_velocity_m_s':self.vel[:,:3].tolist(),'food_angular_velocity_rad_s':self.vel[:,3:6].tolist(),'food_plate_support':onplate.tolist(),'food_spoon_support':onspoon.tolist(),'food_tool_support':tool_supported.tolist(),'food_robot_support':robot_touch.tolist(),'food_still':still.tolist(),'food_ready':self.rules.ready,'food_spooned':self.rules.spooned,'food_stable_s':self.rules.stable,'age':self.age.tolist(),'post_float':self.rules.dwell,'upward_contact_n':total_up.tolist(),'right_finger_joint_m':right_fingers.tolist(),'right_tcp_pos':right_tcp.tolist(),'right_tcp_rotation':right_R.tolist(),'front_control_body_poses':dial_poses,'robot_cookware_control_contacts':robot_pairs,'pan_tilt_deg':pan.tilt_deg(),'left_tcp_pos':tcp.tolist(),'left_tcp_rotation':native_R.tolist(),'left_finger_joint_m':finger_q.tolist(),'spoon_in_gripper_pos':relative_pos.tolist(),'spoon_in_gripper_rotation':relative_R.tolist(),'spoon_robot_contact_actors':[{'actor':a,'normal_force_n':f,'normal':n.tolist(),'point':p.tolist()} for a,f,n,p in rows['spoon'] if is_robot_actor(a)],'spoon_other_contacts':[{'actor':a,'normal_force_n':f,'normal':n.tolist(),'point':p.tolist()} for a,f,n,p in rows['spoon'] if not is_robot_actor(a)],'spoon_pos':spoon.pos.tolist(),'spoon_quat':spoon.quat_xyzw.tolist(),'pan_pos':pan.pos.tolist(),'plate_pos':plate.pos.tolist(),'max_speed':float(np.linalg.norm(self.vel[:,:3],axis=1).max())}
        self.trace.write(json.dumps(row)+'\n');self.trace.flush()
        if self.step_n%100==0:print('GNOCCHI_FULL',json.dumps({k:v for k,v in row.items() if k not in ('positions','age','post_float','upward_contact_n')}),flush=True)
        if self.step_n%20==0:(c.ROOT/'native-actors.json').write_text(json.dumps(self.actor_cache,indent=2)+'\n')
        self.step_n+=1

    def update_water_visual(self):
        if not hasattr(self,'water_prim'):
            from pxr import UsdGeom
            self.water_prim=self.env.unwrapped.sim.stage.GetPrimAtPath('/World/envs/env_0/water_surface')
            self.water_xf=UsdGeom.Xformable(self.water_prim)
            self.water_translate=next(op for op in self.water_xf.GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
        from pxr import Gf,UsdGeom
        self.water_translate.Set(Gf.Vec3d(float(self.pan_pose[0]),float(self.pan_pose[1]),float(self.surface-.0005)))
        UsdGeom.Imageable(self.water_prim).GetVisibilityAttr().Set('inherited' if self.heat.water_mass>.001 else 'invisible')
