"""Observable seating and unsupported retention; no force or pose writes.

The crutch is tilted upward while the replacement foot is unsupported. This
only verifies seating and retention under its own weight. It is not patient
weight-bearing testing or a medical safety certification.
"""
import math
import numpy as np
import mechkit as m
from support_contacts_guarded_draft import ContactWitness

class PoseHold:
    """Bound movement over a continuous observation window.

    High-frequency compliant-contact velocity is not a tape-measure criterion.
    The window restarts if any origin moves more than 3 mm or orientation
    changes more than 3 degrees from its first qualifying observation. The first observation establishes
    the anchor at interval zero;30 elapsed20Hz intervals span1.5seconds.
    """
    def __init__(self):self.clear()
    def clear(self):self.anchor=None;self.steps=0
    def update(self,eligible,*objects):
        if not eligible:self.clear();return 0
        current=[(o.pos.copy(),o.rotation().copy()) for o in objects]
        stable=self.anchor is not None and all(
            np.linalg.norm(p-a)<=.003 and
            math.acos(float(np.clip((np.trace(r@b.T)-1)/2,-1,1)))<=math.radians(3)
            for (p,r),(a,b) in zip(current,self.anchor))
        if not stable:
            self.anchor=current;self.steps=0
        else:self.steps+=1
        return self.steps

from crutch import TOP
from geometry import SHAFT,CUFF

def shaft_clear(crutch,foot):
    p=foot.to_world(np.c_[np.zeros(9),np.zeros(9),np.linspace(.012,.045,9)])
    a=crutch.to_local(p)
    nearest=a.copy();nearest[:,:2]=0.;nearest[:,2]=np.clip(nearest[:,2],.008,.90)
    return float(np.linalg.norm(a-nearest,axis=1).min())>.035

class Score:
    def __init__(self):
        self.contacts=ContactWitness()
        self.reset();m.on_reset(self.reset);m.on_step(self.step)
    def reset(self,env=None):
        self.failed=False;self.value=0.;self.proof=False;self.unsupported_steps=0
        self.clear_steps=0;self.last={};self.steps=0
        self.retention_hold=PoseHold();self.final_hold=PoseHold()
    def step(self,env):
        self.steps+=1
        c=m.state(env,'crutch');o=m.state(env,'old_ferrule');n=m.state(env,'new_ferrule')
        poses=np.r_[c.pos,o.pos,n.pos,c.quat_xyzw,o.quat_xyzw,n.quat_xyzw]
        if not np.isfinite(poses).all():
            self.failed=True
        if self.failed:self.value=0.;self.last={'failed':True};return
        rel=c.to_local(n.pos)[0]
        axis=c.rotation()[:,2];foot_axis=n.rotation()[:,2]
        angle=math.degrees(math.acos(float(np.clip(axis@foot_axis,-1,1))))
        tips=m.fingertips(env)
        clear_n=float(np.linalg.norm(tips-n.pos,axis=1).min())>.065
        clear_o=float(np.linalg.norm(tips-o.pos,axis=1).min())>.065
        old_points=o.to_world([[0,0,.012],[0,0,.045]])
        old_bench=(-.15<o.pos[0]<.87 and -.61<o.pos[1]<.61 and old_points[:,2].min()>=TOP-.020 and old_points[:,2].min()<=TOP+.045)
        old_still=np.linalg.norm(o.lin_vel)<.03 and np.linalg.norm(o.ang_vel)<.3
        removed=shaft_clear(c,o) and old_bench and old_still and clear_o
        seated=np.linalg.norm(rel[:2])<=.001 and abs(rel[2])<=.002 and angle<=3.
        if np.linalg.norm(rel[:2])>.003 or rel[2]<-.005 or rel[2]>.004 or angle>7.:
            self.proof=False;self.unsupported_steps=0;self.retention_hold.clear()
        shaft_up=math.degrees(math.asin(float(np.clip(axis[2],-1,1))))
        foot_clearance=n.pos[2]+min(0.,.045*foot_axis[2])-.019*math.sqrt(max(0.,1-foot_axis[2]**2))-TOP
        unsupported=removed and seated and shaft_up>=60. and foot_clearance>=.025 and clear_n and self.contacts.external_free
        self.unsupported_steps=self.retention_hold.update(unsupported,c,n)
        if self.unsupported_steps>=30:self.proof=True
        local=c.to_local(tips)
        segment=SHAFT[1:]-SHAFT[:-1]
        fraction=np.clip(((local[:,None,:]-SHAFT[:-1])*segment).sum(2)/(segment*segment).sum(1),0.,1.)
        closest=SHAFT[:-1]+fraction[:,:,None]*segment
        clear_c=float(np.linalg.norm(local[:,None,:]-closest,axis=2).min())>.05
        # The original crosswise handgrip and cuff must also be released.
        # Checking only distance from the shaft would misclassify a handle grip.
        for lo,hi in [((-.0218,-.112,.6782),(.0218,.016,.7218)),
                      ((-.044481,-.025524,.806),(.044481,.0745,.90))]:
            distance=np.linalg.norm(np.maximum(np.maximum(np.asarray(lo)-local,local-np.asarray(hi)),0.),axis=1)
            clear_c=clear_c and float(distance.min())>.03
        ends=c.to_world([[0,0,.008],[0,0,.90],[0,-.112,.70],[0,.0745,.85]])
        on_bench=(ends[:,0]>-.17).all() and (ends[:,0]<.89).all() and (ends[:,1]>-.63).all() and (ends[:,1]<.63).all()
        cuff_clearance=float(c.to_world(CUFF)[:,2].min()-TOP)
        on_bench=on_bench and -.003<=foot_clearance<=.003 and -.003<=cuff_clearance<=.003 and abs(axis[2])<.20
        final=removed and seated and self.proof and clear_c and clear_n and clear_o and on_bench and self.contacts.robot_free
        self.clear_steps=self.final_hold.update(final,c,n)
        # A recoverable drop does not poison future scores. Current off-bench
        # task objects still cannot earn partial credit.
        objects_above_bench=min(c.pos[2],o.pos[2],n.pos[2],float(ends[:,2].min()))>=TOP-.03
        # Partial credit requires a fitted foot that has physically retained
        # while unsupported. Removing the worn foot alone leaves bare metal
        # and is not a safe intermediate outcome.
        partial=removed and seated and self.proof and clear_n and objects_above_bench
        self.value=1. if self.clear_steps>=20 else .5 if partial else 0.
        self.last={'score':self.value,'removed':bool(removed),'seated':bool(seated),'relative_m':rel.tolist(),'axis_angle_deg':angle,
                   'shaft_up_deg':shaft_up,'foot_clearance_m':foot_clearance,'unsupported_steps':self.unsupported_steps,
                   'retention_observed':self.proof,'clear_steps':self.clear_steps,'cuff_floor_clearance_m':cuff_clearance,'hands_clear':bool(clear_c and clear_n and clear_o),'external_support_free':bool(self.contacts.external_free),'assembly_robot_free':bool(self.contacts.robot_free),'contacts':dict(self.contacts.last),'failed':self.failed}
        if self.steps%20==0:print('FERRULE_SCORE',self.steps,self.last,flush=True)
