"""Physical towel-contact adversarial control; assembly starts installed.

The left fingertips press the original service towel while avoiding the vacuum.
Only robot targets change. This is a scorer control, never a repair demo.
"""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_native_support_v2 import *
from yam_arena import demo as d,ik,station
import interarm_guard

OBJECTS=[replace(x,pos=tuple(C+np.array([AX,0,AZ])),start_quat=(0.,0.,0.,1.))
         if x.name=='roller' else x for x in OBJECTS]
OBJECTS=[replace(x,pos=tuple(C+np.array([AX,0,COVER_SEATED_Z])),start_quat=(0.,0.,0.,1.))
         if x.name=='cover' else x for x in OBJECTS]
STATE={'bias':0.}
HOVER=np.array([.191,.02,.95])
CONTACT=np.array([.191,.02,.7805])


def observe_contact(env):
    time=AUDIT.steps/20.
    rows=getattr(AUDIT,'latest',[])
    if not rows:return
    totals=[];unexpected=[]
    for row in rows:
        total=0.
        for p in row['contacts']['pairs']:
            if '/LeftArm/' not in p['other'] and '/RightArm/' not in p['other']:continue
            normal=max(float(np.linalg.norm(p['net_normal_world_N'])),
                       float(np.abs(p['normal_forces_N']).sum()))
            friction=float(np.linalg.norm(np.asarray(p['friction_forces_world_N']).reshape(-1,3),axis=1).sum())
            force=normal+friction
            intended=('/towel/' in p['body'] and '/LeftArm/' in p['other']
                      and p['other'].rsplit('/',1)[-1] in ('tip_left','tip_right'))
            if intended:total+=force
            elif force>.25:unexpected.append((p['body'],p['other'],force))
        totals.append(total)
    maximum=max(totals)
    if maximum>5. or unexpected:
        print('INDIRECT_CONTACT_ABORT',json.dumps(dict(time=time,peak_N=maximum,unexpected=unexpected)),flush=True)
        raise RuntimeError('indirect-contact diagnostic exceeded force/contact guard')
    if 6.<=time<18.:
        delta=float(np.clip(.0001*(1.-maximum),-.0001,.0001))
        STATE['bias']=float(np.clip(STATE['bias']+delta,0.,.012))
    if AUDIT.steps%10==0:
        print('INDIRECT_CONTACT_STATUS',json.dumps(dict(step=AUDIT.steps,time_s=time,
            force_min_N=min(totals),force_max_N=maximum,bias_m=STATE['bias'],readings=len(rows))),flush=True)


m.on_step(observe_contact)


@m.on_reset
def reset_contact(env=None):
    STATE.clear();STATE['bias']=0.


previous_target=d.ArmPlan.target
previous_action=d.ArmPlan.action7


def target(self,time):
    if self.base_pos[1]<0:
        return self.start_pos,self.start_rot,d.FINGER_OPEN,'right READY'
    if time<6.:
        return previous_target(self,time)
    p=CONTACT-np.array([0,0,STATE['bias']])
    if time>=18.:
        fraction=ik.smoothstep(min(1.,(time-18.)/2.))
        p=p+(HOVER-p)*fraction
    return p,d.grip_rotation(),0.,'towel contact' if time<18. else 'withdraw and release'


def action(self,time):
    result=previous_action(self,time)
    if self.worst_ik_mm>2. or self.worst_ik_deg>2.:
        raise RuntimeError('indirect-contact command exceeds2mm/2deg IK guard')
    return result


d.ArmPlan.target=target
d.ArmPlan.action7=action


def demo():
    return {'left':[d.Waypoint(3.,tuple(HOVER),0.,'hover'),
                    d.Waypoint(3.,tuple(CONTACT),0.,'approach towel'),
                    d.Waypoint(18.,tuple(HOVER),0.,'contact and withdraw')],
            'right':[d.Waypoint(24.,tuple(ik.tcp_pose(d.READY,station.BASES['right'])[0]),d.FINGER_OPEN)]}
