"""Bounded contact calibration: real loose foot on the ordinary bench.

This is neither a repair demonstration nor an isolated full-score negative.
The fixed fixture tests all-body support coverage and its known weight.
"""
import pathlib,sys,json
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import numpy as np
import mechkit as m
import runtime
_f=m._Finder.targets['yam_arena.scene']
def physics(scene):_f(scene);scene.PHYSICS_DT=1./480.
m._Finder.targets['yam_arena.scene']=physics
from crutch_handle_layout import part,POS,Q,TOP
from ferrule_axial import make
from support_contacts_guarded_draft import reported
from scoring_guarded_draft import Score
from distractors import objects as workshop_objects
SCENE='station';INSTRUCTION='Observe the loose ferrule resting on the bench.';SUCCESS_HOLD_S=0.
OBJECTS=[reported(part()),make('old_ferrule',POS,Q,worn=True),reported(make('new_ferrule',(.35,-.05,TOP+.0002),(0.,0.,0.,1.))),*workshop_objects()]
OBJECTS=[reported(obj) if obj.name in ('crutch','new_ferrule','old_ferrule') else obj for obj in OBJECTS]
result=Score();COUNT=0

def reset(env=None):
    global COUNT
    COUNT=0

def observe(env):
    global COUNT
    COUNT+=1
    if COUNT!=1 and COUNT%10:return
    n=m.state(env,'new_ferrule');w=result.contacts;dt=env.unwrapped.physics_dt
    net=np.asarray(w.view.get_contact_force_matrix(dt=dt).numpy(),float).copy()
    f,_,counts,starts=w.view.get_friction_data(dt=dt);f=np.asarray(f.numpy(),float);counts=np.asarray(counts.numpy()).copy();starts=np.asarray(starts.numpy()).copy()
    w.pair_sums(np.linalg.norm(f,axis=1),counts,starts)
    friction=np.zeros_like(net)
    for i,j in np.argwhere(counts>0):
        start=int(starts[i,j]);count=int(counts[i,j]);friction[i,j]=f[start:start+count].sum(0)
    total=net+friction;pair=total[:w.foot_count].sum(0);indices=np.flatnonzero(np.linalg.norm(pair,axis=1)>1e-5)
    print('BENCH_SUPPORT_CALIBRATION',COUNT,json.dumps({'foot_position':n.pos.tolist(),'foot_linear_velocity':n.lin_vel.tolist(),'finger_clearance_m':float(np.linalg.norm(m.fingertips(env)-n.pos,axis=1).min()),'foot_bottom_above_bench_m':float(n.pos[2]-TOP),'witness':w.last,'external_free':w.external_free,'external_reaction_vector_n':pair.sum(0).tolist(),'foot_sensor_count':w.foot_count,'assembly_sensor_count':len(w.bodies),'robot_filter_count':int(w.robot_mask.sum()),'supporting_targets':[{'target':w.filters[i],'reaction_n':pair[i].tolist()} for i in indices]}),flush=True)
m.on_reset(reset);m.on_step(observe)
score=lambda objects:float(result.value)
success=lambda objects:False
def demo():return {'left':[],'right':[]}
