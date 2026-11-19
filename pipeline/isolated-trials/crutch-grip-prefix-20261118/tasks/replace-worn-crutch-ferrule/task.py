"""Replace the worn foot on a full-size crutch using ordinary bench support."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
import runtime
_f=m._Finder.targets['yam_arena.scene']
def physics(scene):_f(scene);scene.PHYSICS_DT=1./480.
m._Finder.targets['yam_arena.scene']=physics
from crutch_handle_layout import part,POS,Q,TOP
from ferrule_axial import make
from support_contacts_guarded_draft import reported
from whole_arm_contact_audit import WholeArmAudit
audit=WholeArmAudit()
import controller_fixed_bore as controller
from scoring_guarded_draft import Score
from distractors import objects as workshop_objects
import retention_video_view
SCENE='station';INSTRUCTION='Prepare this crutch for its owner.';SUCCESS_HOLD_S=3.
OBJECTS=[reported(part()),make('old_ferrule',POS,Q,worn=True),reported(make('new_ferrule',(.35,-.05,TOP+.0002),(0.,0.,0.,1.))),*workshop_objects()]
OBJECTS=[reported(obj) if obj.name in ('crutch','new_ferrule','old_ferrule') else obj for obj in OBJECTS]
result=Score();controller.CONTACTS=result.contacts;controller.AUDIT=audit
score=lambda objects:float(result.value)
success=lambda objects:result.value==1.
demo=controller.demo

# Three bounded common rigid-pair corrections in free space; no live-axis following.
import numpy as np
from scipy.spatial.transform import Rotation
c=controller
RECENT=[];CAPTURE_AT=None;BASE_P=np.zeros(3);TARGET_P=np.zeros(3);BASE_R=np.eye(3);TARGET_R=np.eye(3)

def reset_alignment(env=None):
    global CAPTURE_AT,BASE_P,TARGET_P,BASE_R,TARGET_R
    c.ALIGN_OFFSET=np.zeros(3);c.ALIGN_BLEND=1.;c.ALIGN_ROTATION=np.eye(3)
    CAPTURE_AT=None;BASE_P=np.zeros(3);TARGET_P=np.zeros(3);BASE_R=np.eye(3);TARGET_R=np.eye(3);RECENT.clear()

def update_alignment(env):
    global CAPTURE_AT,BASE_P,TARGET_P,BASE_R,TARGET_R
    count=c.COUNT
    if count<600 or count>800:return
    crutch=m.state(env,'crutch');foot=m.state(env,'new_ferrule');RECENT.append(crutch.pos.copy())
    if len(RECENT)>21:RECENT.pop(0)
    frame=c.SNAP.get('bore',foot);fa=frame.rotation()[:,2];ca=crutch.rotation()[:,2]
    height=.10 if count<=680 else .039;error=frame.pos+fa*height-crutch.pos
    drift=max(float(np.linalg.norm(p-RECENT[0])) for p in RECENT)
    cross=np.cross(ca,fa);sn=float(np.linalg.norm(cross));angle=float(np.arctan2(sn,np.clip(ca@fa,-1.,1.)))
    correction=Rotation.from_rotvec(cross/max(sn,1e-12)*angle).as_matrix()
    if count in (640,730,770):
        proposed_p=c.ALIGN_OFFSET+error;proposed_r=correction@c.ALIGN_ROTATION
        total_angle=float(np.degrees(Rotation.from_matrix(proposed_r).magnitude()))
        external=result.contacts.last.get('foot_external_n')
        unloaded=external is not None and external<.8
        allowed=(audit.ready and audit.last['correction_allowed'] and unloaded and np.linalg.norm(proposed_p)<=.020 and total_angle<=3. and drift<=.002 and np.degrees(angle)<=3. and (count==640 or (np.linalg.norm(error)<=.005 and np.degrees(angle)<=1.)))
        if allowed:
            c.SNAP.setdefault('bore',foot);BASE_P=c.ALIGN_OFFSET.copy();TARGET_P=proposed_p;BASE_R=c.ALIGN_ROTATION.copy();TARGET_R=proposed_r;CAPTURE_AT=count
        else:c.ABORT=True
        print('PAIR_ALIGNMENT_CAL',count,{'allowed':bool(allowed),'root_error_m':error.tolist(),'recent_root_drift_m':drift,'axis_error_deg':float(np.degrees(angle)),'target_offset_m':proposed_p.tolist(),'target_rotation_vector_rad':Rotation.from_matrix(proposed_r).as_rotvec().tolist(),'total_rotation_deg':total_angle,'bore_pos':frame.pos.tolist(),'bore_rotation':frame.rotation().tolist(),'foot_support_n':external,'audit':audit.last},flush=True)
    if CAPTURE_AT is not None and count<=CAPTURE_AT+20 and not c.ABORT:
        if not audit.last['correction_allowed']:
            c.ABORT=True;print('PAIR_ALIGNMENT_FROZEN',count,audit.last,flush=True)
        else:
            blend=float(c.ik.smoothstep(min(1.,max(0.,(count-CAPTURE_AT)/20.))))
            c.ALIGN_OFFSET=BASE_P+(TARGET_P-BASE_P)*blend;c.ALIGN_ROTATION=c.ik.slerp_matrix(BASE_R,TARGET_R,blend)
    lateral=float(np.linalg.norm(error-fa*(error@fa)))
    if count in (680,800):
        limit=.0015 if count==680 else .0004;angle_limit=.75 if count==680 else .35
        if lateral>limit or abs(error@fa)>.002 or np.degrees(angle)>angle_limit:
            c.ABORT=True;print('PAIR_ALIGNMENT_ABORT',count,{'root_error_m':error.tolist(),'axis_error_deg':float(np.degrees(angle))},flush=True)
    if count%20==0:print('PAIR_ALIGNMENT_STATE',count,{'offset_m':c.ALIGN_OFFSET.tolist(),'rotation_vector_rad':Rotation.from_matrix(c.ALIGN_ROTATION).as_rotvec().tolist(),'root_error_m':error.tolist(),'axis_error_deg':float(np.degrees(angle)),'unexpected_contact_n':audit.last.get('maximum_unexpected_contact_n')},flush=True)
m.on_reset(reset_alignment);m.on_step(update_alignment)


def log_insertion_twist(env):
    if not c.INSERT_AT<=c.COUNT<=c.INSTALLED_AT:return
    root=c.LIVE['c'];foot=c.LIVE['new']
    print('SHAFT_TWIST_STATE',c.COUNT,{'commanded_common_twist_deg':float(np.degrees(c.insertion_twist_angle())),'command_update_step':c.INSERT_COUNT,'command_height_m':c.INSERT_HEIGHT,'axial_tracking_bias_m':c.INSERT_BIAS,'axial_tracking_error_m':c.INSERT_BIAS_ERROR,'axial_bias_allowed':c.INSERT_BIAS_ALLOWED,'loaded_rotation_vector_deg':np.degrees(Rotation.from_matrix(c.LOAD_ROTATION).as_rotvec()).tolist(),'loaded_attitude_error_deg':c.LOAD_ERROR_DEG,'loaded_attitude_allowed':c.LOAD_ALLOWED,'native_liner_guard':None if c.NATIVE is None else c.NATIVE.last,'twist_start_step':c.INSERT_START,'force_retreat':c.INSERT_RETREAT,'insertion_abort_reason':c.INSERT_ABORT_REASON,'crutch_pos':root.pos.tolist(),'crutch_rotation':root.rotation().tolist(),'foot_pos':foot.pos.tolist(),'foot_rotation':foot.rotation().tolist(),'contacts':dict(result.contacts.last),'whole_arm':audit.last},flush=True)
m.on_step(log_insertion_twist)

# Read-only native contact observation supplies the loaded-attitude guard.
import native_mouth_audit
