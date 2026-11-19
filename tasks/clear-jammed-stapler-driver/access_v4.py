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
TOOL = np.array([.165, .100, TOP + .001])

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
    box(parts,cols,'guide_left',(.044,.005,.019),(.018,-.0059,0),'steel')
    box(parts,cols,'guide_right',(.044,.005,.019),(.018,.0059,0),'steel')
    box(parts,cols,'guide_floor',(.044,.0068,.003),(.018,0,-.004),'steel')
    # Service roof is open, as Fig11 shows after the magazine is withdrawn.
    dp, dc = [], []
    box(dp,dc,'driver_blade',(.025,.0058,.0012),(.024,0,0),'steel')
    driver=m.slider('driver','X',0.,.025,parts=dp,colliders=dc,mass=.025,
                    spring=0.,damping=2.,friction=.1,armature=.00001,effort=30.)
    return m.mechanism('stapler',tuple(NOSE),[driver],LOOKS,
             frame_parts=parts,frame_colliders=cols,free_mass=2.67,
             frame_inertia=(.02,.02,.02),friction=(.6,.5),solver_iters=64)

OBJECTS = [housing(), SimReady('screwdriver',pos=tuple(TOOL),size=(.148,.027,.026),
    mass=.03647,package='handTool_screwdriver_slotted_4mm_a01',
    usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd',convex=False)]

_n=0
_TOOL_POSE=None
@m.on_reset
def reset(env):
    global _n
    _n=0

@m.on_step
def probe(env):
    global _n, _TOOL_POSE
    from yam_arena.scene import object_states
    objects=object_states(env,['stapler','screwdriver'])
    _n+=1
    asset=env.unwrapped.scene['screwdriver']
    pose=asset.data.root_link_pose_w[0].detach().cpu().numpy().astype(float)
    origin=env.unwrapped.scene.env_origins[0].detach().cpu().numpy()
    _TOOL_POSE=(pose[:3]-origin,m.quat_matrix(pose[3:7]))
    # 2 N constant reaction is only an access-gate load, not a claimed real
    # pneumatic preload. No pose changes or contact-triggered driver motion.
    m.push(env,'stapler',(-2.,0,0),body='driver')
    if _n % 10 == 0:
        s=objects['stapler'];t=objects['screwdriver']
        tip=_TOOL_POSE[0]+_TOOL_POSE[1]@np.array([.10286,0,.01321])
        print('ACCESS_GATE '+json.dumps({'step':_n,'driver_m':s.joint('driver'),
          'body_pos':s.pos.tolist(),'tool_pos':np.asarray(t.pos).tolist(),'tool_tip':tip.tolist(),'tool_root':_TOOL_POSE[0].tolist(),'tool_rotation':_TOOL_POSE[1].tolist()}),flush=True)

def score(objects): return 0.
def success(objects): return False
demo=placeholder_demo

def demo_script(ctx):
    L=ctx.left
    O=-.04695;H=-.008
    pick=(TOOL[0]-.010,TOOL[1],TOP+.023)
    yield from L.move((pick[0],pick[1],TOP+.12),1.5,O)
    yield from L.move(pick,1.5,O)
    yield from L.grip(H,.6)
    yield from L.move((pick[0],pick[1],TOP+.10),1.2,H)
    yield from L.hold(.3)
    # Privileged demo-only rigid pose measurement. All movement remains robot contact.
    rot=L.rot.copy()
    if _TOOL_POSE is not None:
        rot=_TOOL_POSE[1].T@L.measured_rot
    yield from L.move(L.pos,1.,H,rot=rot)
    yield from L.hold(.4)
    target=NOSE+np.array([-.012,0,0])
    for _ in range(3):
        if _TOOL_POSE is None:
            tip=np.array([L.pos[0]+.11286,L.pos[1],L.pos[2]-.01])
        else:
            tip=_TOOL_POSE[0]+_TOOL_POSE[1]@np.array([.10286,0,.01321])
        error=target-tip
        yield from L.move(L.pos+error,1.,H,rot=rot)
        yield from L.hold(.2)
    start=L.pos.copy()
    yield from L.move(start+np.array([.045,0,0]),2.5,H,rot=rot)
    yield from L.hold(1.)
    yield from L.move(L.pos+np.array([-.060,0,.040]),1.5,H,rot=rot)
    yield from L.hold(.5)
