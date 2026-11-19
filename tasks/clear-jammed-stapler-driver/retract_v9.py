"""Development gate only: grip a stock 4 mm screwdriver and reach the driver.

No task success is awarded. This isolates physical access before implementing
the battery, magazine, bent staple, and final scorer. Housing envelope is a
disclosed 274 x 224 x 81 mm approximation of P361, lying on its side.
"""
import pathlib
import sys
import json
from dataclasses import replace
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import mechkit as m
import reset_physics_v6
from scaled_convex import scaled_convex_part
from simready import SimReady
from closedloop import placeholder_demo
from yam_arena.task_file import TABLE_TOP

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 3.
INSTRUCTION = 'Clear the jam and leave the stapler ready to reload.'
LOOKS = {
    'case': m.Look(color=m.rgb(.62, .74, .10), roughness=.65),
    'black': m.Look(color=m.rgb(.08, .09, .09), roughness=.8),
    'steel': m.Look(color=m.rgb(.65, .67, .69), metallic=.7, roughness=.3),
}
NOSE = np.array([.360, .100, TOP + .054])
TOOL = np.array([.250, -.100, TOP + .001])

def box(parts, cols, name, size, center, material='case'):
    parts.append((name, m.box(size, center), material))
    # Fine clearances need explicit offsets rather than mechkit's 4 mm default.
    cols.append(m.col_box(name+'_c',size,center).replace('contactOffset = 0.004','contactOffset = 0.0001'))

def housing():
    parts, cols = [], []
    # Reference origin is the front driver mouth, axis +X into tool.
    # The original 81 mm side envelope rests on the bench; no service fixture.
    box(parts,cols,'motor',( .13,.082,.081),(.195,.025,-.0135))
    box(parts,cols,'rear_battery_socket',(.075,.07,.068),(.185,.162,-.020))
    box(parts,cols,'grip',(.035,.12,.041),(.195,.10,-.020),'black')
    box(parts,cols,'magazine',(.045,.20,.025),(.045,.110,-.025),'steel')
    # 6.8 mm transverse service opening around the 6.35 mm staple crown;
    # estimated from the staple standard, not a claimed manufacturer drawing.
    box(parts,cols,'guide_left',(.043,.005,.019),(.0185,-.0059,0),'steel')
    box(parts,cols,'guide_right',(.043,.005,.019),(.0185,.0059,0),'steel')
    box(parts,cols,'guide_floor',(.044,.0068,.003),(.018,0,-.0024),'steel')
    # Short fixed nose guide bridge; rear service access remains open.
    # 9mm length and1.8mm wire clearance are functional estimates, not a
    # manufacturer dimensioned section; source-match review still pending.
    box(parts,cols,'nose_guide_bridge',(.007,.0068,.003),(.0005,0,.0024),'steel')
    dp, dc = [], []
    box(dp,dc,'driver_blade',(.025,.0058,.0012),(.0193,0,0),'steel')
    driver=m.slider('driver','X',-.010,.025,parts=dp,colliders=dc,mass=.025,
                    spring=200.,damping=2.,friction=.1,armature=.00001,effort=30.)
    return m.mechanism('stapler',tuple(NOSE),[driver],LOOKS,
             frame_parts=parts,frame_colliders=cols,free_mass=2.67,
             start=(),frame_inertia=(.02,.02,.02),friction=(.6,.5),solver_iters=64)

def bent_staple():
    parts, cols=[],[]
    # Damaged18GA wire with an out-of-plane V crown and bent shoulders.
    # Shoulder lies behind the existing roof lip; lower legs run through the
    # original aperture. No new retention surface and no initial solid overlap.
    # The bent wire is rigid: further elastic/plastic yielding is not modeled.
    def segment(name,a,b):
        a=np.asarray(a);b=np.asarray(b);d=b-a;length=np.linalg.norm(d)
        x=d/length;ref=np.array([1.,0.,0.]) if abs(x[0])<.9 else np.array([0.,1.,0.])
        z=np.cross(x,ref);z/=np.linalg.norm(z);y=np.cross(z,x)
        mesh=m.box((length,.0012,.001))
        mesh=m.Mesh(mesh.v@np.column_stack([x,y,z]).T+(a+b)/2,mesh.f)
        parts.append((name,mesh,'steel'))
        cols.append(m.col_hull(name+'_c',mesh).replace('contactOffset = 0.004','contactOffset = 0.0001'))
    for side in (-1,1):
        shoulder=(.006,side*.002575,.0018)
        knee=(.0048,side*.002575,-.00025)
        tip=(-.004,side*.002575,-.00025)
        segment('crown'+str(side+1),(.006,0.,0.),shoulder)
        segment('shoulder'+str(side+1),shoulder,knee)
        segment('leg'+str(side+1),knee,tip)
    return scaled_convex_part(m.free('bent_staple',tuple(NOSE),parts,cols,LOOKS,mass=.00024,
                 friction=(.4,.3),torsional_patch=None,solver_iters=64))

OBJECTS = [housing(),bent_staple(), SimReady('screwdriver',pos=tuple(TOOL),size=(.148,.027,.026),
    mass=.03647,package='handTool_screwdriver_slotted_4mm_a01',
    usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd',convex=False,yaw=np.pi)]

_n=0
_TOOL_POSE=None
@m.on_reset
def reset(env):
    global _n
    _n=0
    # Fixed mechanical rest, independent of battery state.
    # Zero joint geometry is authored at the jam pose to avoid reset overlap.
    reset_physics_v6.snapshot(env.unwrapped,'post_settle')

@m.on_step
def probe(env):
    global _n, _TOOL_POSE
    from yam_arena.scene import object_states
    objects=object_states(env,['stapler','screwdriver','bent_staple'])
    _n+=1
    asset=env.unwrapped.scene['screwdriver']
    pose=asset.data.root_link_pose_w[0].detach().cpu().numpy().astype(float)
    origin=env.unwrapped.scene.env_origins[0].detach().cpu().numpy()
    _TOOL_POSE=(pose[:3]-origin,m.quat_matrix(pose[3:7]))
    # 2 N constant reaction is only an access-gate load, not a claimed real
    # pneumatic preload. No pose changes or contact-triggered driver motion.
    # Driver spring remains preloaded independently of electrical isolation.
    if _n % 10 == 0:
        s=objects['stapler'];t=objects['screwdriver']
        tip=_TOOL_POSE[0]+_TOOL_POSE[1]@np.array([.10286,0,.01321])
        print('ACCESS_GATE '+json.dumps({'step':_n,'driver_m':s.joint('driver'),'staple_pos':objects['bent_staple'].pos.tolist(),'staple_quat':objects['bent_staple'].quat_xyzw.tolist(),
          'body_pos':s.pos.tolist(),'tool_pos':np.asarray(t.pos).tolist(),'tool_tip':tip.tolist(),'tool_root':_TOOL_POSE[0].tolist(),'tool_rotation':_TOOL_POSE[1].tolist()}),flush=True)

def score(objects): return 0.
def success(objects): return False
demo=placeholder_demo

def demo_script(ctx):
    L=ctx.right
    O=-.04695;H=-.008
    pick=(TOOL[0]+.010,TOOL[1],TOP+.023)
    yield from L.move((pick[0],pick[1],TOP+.12),1.5,O)
    yield from L.move(pick,1.5,O)
    yield from L.grip(H,.6)
    yield from L.move((pick[0],pick[1],TOP+.10),1.2,H)
    yield from L.hold(.3)
    # Align the held shaft toward the open feed-side access, preserving roll.
    yield from L.move(np.array([.29,-.10,TOP+.13]),1.5,H)
    direction=np.array([np.cos(np.radians(80.)),0.,-np.sin(np.radians(80.))])
    u=_TOOL_POSE[1][:,0] if _TOOL_POSE is not None else np.array([-1.,0.,0.])
    cross=np.cross(u,direction);sine=np.linalg.norm(cross);cosine=float(u@direction)
    axis=cross/max(sine,1e-12)
    K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
    A=np.eye(3)+sine*K+(1-cosine)*(K@K)
    from yam_arena import ik
    original_rot=L.rot.copy()
    rot=A@L.rot
    tip_now=_TOOL_POSE[0]+_TOOL_POSE[1]@np.array([.10286,0,.01321]) if _TOOL_POSE is not None else L.pos+np.array([-.11286,0,-.010])
    local_tip=L.rot.T@(tip_now-L.pos)
    target=NOSE+np.array([.0053,0,.020])
    tcp=target-rot@local_tip
    q_goal,pe,re=ik.solve(tcp,ik.matrix_to_quat_xyzw(rot),base_pos=L.base_pos,iters=1000)
    if pe>.001 or re>np.radians(1.):
        print('RETRACT_IK_REFUSED '+json.dumps({'mm':pe*1000,'deg':np.degrees(re)}),flush=True)
        return
    q_start=L.q.copy()
    min_tip_z=10.
    for i in range(1,81):
        q=q_start+ik.smoothstep(i/80)*(q_goal-q_start)
        pos,r=ik.tcp_pose(q,L.base_pos)
        min_tip_z=min(min_tip_z,float((pos+r@local_tip)[2]))
    if min_tip_z<TOP+.015:
        print('RETRACT_PATH_REFUSED '+json.dumps({'min_tip_z':min_tip_z}),flush=True)
        return
    print('RETRACT_JOINT_PATH '+json.dumps({'target_error_mm':pe*1000,'target_error_deg':np.degrees(re),'min_tip_z':min_tip_z}),flush=True)
    for i in range(1,81):
        L.q=q_start+ik.smoothstep(i/80)*(q_goal-q_start)
        L.pos,L.rot=ik.tcp_pose(L.q,L.base_pos)
        yield
    for _ in range(3):
        tip=_TOOL_POSE[0]+_TOOL_POSE[1]@np.array([.10286,0,.01321]) if _TOOL_POSE is not None else L.pos+L.rot@local_tip
        yield from L.move(L.pos+target-tip,1.,H,rot=rot)
        yield from L.hold(.2)
    yield from L.move(L.pos+np.array([0,0,-.020]),2.,H,rot=rot)
    yield from L.hold(.3)
    yield from L.move(L.pos+np.array([.012,0,0]),2.,H,rot=rot)
    yield from L.hold(1.)
    yield from L.move(L.pos+np.array([0,0,.05]),1.5,H,rot=rot)
    yield from L.hold(.5)
