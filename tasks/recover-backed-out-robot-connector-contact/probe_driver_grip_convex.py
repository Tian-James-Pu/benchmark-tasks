"""Compare source screwdriver collision representations during a physical grasp; no task score."""
import pathlib,sys,numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
import simready as sr
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
SCENE='station'; TOP=TABLE_TOP[SCENE]
SUCCESS_HOLD_S=0.
INSTRUCTION='Inspect the screwdriver grasp.'
USE_CONVEX=True
_original_prepare=sr._prepare

def prepare(prim,spec):
    from pxr import Usd,UsdPhysics,PhysxSchema
    while instances:=[p for p in Usd.PrimRange(prim) if p.IsInstance()]:
        for p in instances:p.SetInstanceable(False)
    _original_prepare(prim,spec)
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            api=PhysxSchema.PhysxCollisionAPI.Apply(p)
            api.CreateContactOffsetAttr(.0001);api.CreateRestOffsetAttr(0.)
sr._prepare=prepare
OBJECTS=[sr.SimReady(name='driver',pos=(.24,.18,TOP+.001),size=(.148,.027,.026),
    mass=.03647007420659065,package='handTool_screwdriver_slotted_4mm_a01',
    usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd',convex=USE_CONVEX,scale=1.,yaw=np.pi/2)]
class Trial:
    def reset(self,env=None):self.steps=0;self.contacts=None
    def step(self,env):
        self.steps+=1
        if self.contacts is None:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            bodies=[p for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            sensors=[str(p.GetPath()) for p in bodies if '/driver/' in str(p.GetPath())]
            filters=[str(p.GetPath()) for p in bodies if '/LeftArm/' in str(p.GetPath()) and p.GetName() in ('tip_left','tip_right')]
            print('DRIVER_CONTACT_PATHS',sensors,filters,flush=True)
            self.contacts=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=4096)
        if self.steps==1 or self.steps%10==0:
            dt=env.unwrapped.physics_dt
            force=self.contacts.get_contact_force_matrix(dt=dt).numpy().copy()
            fn,points,normals,separations,counts,starts=self.contacts.get_contact_data(dt=dt)
            fn=fn.numpy().copy();points=points.numpy().copy();normals=normals.numpy().copy()
            separations=separations.numpy().copy();counts=counts.numpy().copy();starts=starts.numpy().copy()
            ff,fp,fc,fs=self.contacts.get_friction_data(dt=dt)
            ff=ff.numpy().copy();fp=fp.numpy().copy();fc=fc.numpy().copy();fs=fs.numpy().copy()
            pairs=[]
            for j in range(counts.shape[1]):
                n=int(counts[0,j]);start=int(starts[0,j]);nf=int(fc[0,j]);sf=int(fs[0,j])
                pairs.append({'finger':j,'normal_force_N':force[0,j].tolist(),
                    'points':points[start:start+n].tolist(),'normals':normals[start:start+n].tolist(),
                    'normal_magnitudes_N':fn[start:start+n].reshape(-1).tolist(),
                    'separations_m':separations[start:start+n].reshape(-1).tolist(),
                    'friction_forces_N':ff[sf:sf+nf].tolist(),'friction_points':fp[sf:sf+nf].tolist()})
            pose=m._np(env.unwrapped.arena_world.get_pose_w('driver'))[0]
            pts=m.fingertips(env);x=pts[8]-pts[2];x/=np.linalg.norm(x);z=pts[2]-pts[3];z/=np.linalg.norm(z)
            tcp=(pts[2]+pts[8])/2-.013*z;R=np.column_stack((x,np.cross(z,x),z))
            robot=env.unwrapped.scene['left_robot'];ids=[robot.find_joints('joint'+str(j))[0][0] for j in range(1,9)]
            print('DRIVER_GRIP_STATE',self.steps,{'convex':USE_CONVEX,'pose':pose.tolist(),'physical_tcp':tcp.tolist(),
                'physical_tcp_R':R.tolist(),'robot_q':m._np(robot.data.joint_pos)[0,ids].tolist(),'contacts':pairs},flush=True)
t=Trial();m.on_reset(t.reset);m.on_step(t.step)
def score(objects):return 0.
def success(objects):return False
def demo():
    return {'left':[
        Waypoint(1.5,(.24,.165,TOP+.12),-.025,'hover handle',-np.pi/2),
        Waypoint(1.5,(.24,.165,TOP+.016),-.025,'approach handle',-np.pi/2),
        Waypoint(1.,(.24,.165,TOP+.016),0.,'grip handle',-np.pi/2),
        Waypoint(2.,(.24,.165,TOP+.15),0.,'lift driver',-np.pi/2),
        Waypoint(3.,(.24,.165,TOP+.15),0.,'hold driver',-np.pi/2),
        Waypoint(2.,(.24,.165,TOP+.016),0.,'lower driver',-np.pi/2),
        Waypoint(1.,(.24,.165,TOP+.016),-.025,'release handle',-np.pi/2),
        Waypoint(1.5,(.24,.165,TOP+.12),-.025,'withdraw',-np.pi/2)]}
