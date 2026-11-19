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
import precision_physics
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
    box(parts,cols,'guide_left',(.043,.005,.019),(.0185,-.0059,0),'steel')
    box(parts,cols,'guide_right',(.043,.005,.019),(.0185,.0059,0),'steel')
    box(parts,cols,'guide_floor',(.044,.0068,.003),(.018,0,-.0024),'steel')
    # Short fixed nose guide bridge; rear service access remains open.
    # 9mm length and1.8mm wire clearance are functional estimates, not a
    # manufacturer dimensioned section; source-match review still pending.
    box(parts,cols,'nose_guide_bridge',(.007,.0068,.003),(.0005,0,.0024),'steel')
    dp, dc = [], []
    box(dp,dc,'driver_blade',(.025,.0058,.0012),(.0142,0,0),'steel')
    driver=m.slider('driver','X',-.010,.025,parts=dp,colliders=dc,mass=.025,
                    spring=200.,damping=2.,friction=.1,armature=.00001,effort=30.)
    return m.mechanism('stapler',tuple(NOSE),[driver],LOOKS,
             frame_parts=parts,frame_colliders=cols,free_mass=2.67,
             start=(),frame_inertia=(.02,.02,.02),friction=(.6,.5),solver_iters=64)

def bent_staple():
    parts, cols=[],[]
    # 18GA narrow-crown wire approximation:6.35mm outer crown,10mm legs.
    box(parts,cols,'crown',(.0012,.00635,.001),(.001,0,0),'steel')
    # Bent legs catch the actual muzzle edge; the crown is within its fixed
    # guide, rather than in the open rear feed tray. Both legs splay by0.56mm.
    for side in (-1,1):
        a=np.array([.001,side*.002575,0]);b=np.array([-.009,side*.00314,0])
        mid=(a+b)/2;length=np.linalg.norm(b-a)
        theta=np.degrees(np.arctan2(b[1]-a[1],b[0]-a[0]))
        wire=m.box((length,.0012,.001)).turned(theta,'z').moved(mid)
        parts.append((f'leg{side+1}',wire,'steel'))
        cols.append(m.col_hull(f'leg{side+1}_c',wire).replace('contactOffset = 0.004','contactOffset = 0.0001'))
    return scaled_convex_part(m.free('bent_staple',tuple(NOSE),parts,cols,LOOKS,mass=.00024,
                 friction=(.4,.3),torsional_patch=None,solver_iters=64))

OBJECTS = [housing(),bent_staple(), SimReady('screwdriver',pos=tuple(TOOL),size=(.148,.027,.026),
    mass=.03647,package='handTool_screwdriver_slotted_4mm_a01',
    usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd',convex=False)]

_n=0
_TOOL_POSE=None
@m.on_reset
def reset(env):
    global _n
    _n=0
    # Fixed mechanical rest, independent of battery state.
    # Zero joint geometry is authored at the jam pose to avoid reset overlap.
    m.set_target(env, 'stapler', 'driver', -.010)

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
    yield from ctx.left.hold(5.)
