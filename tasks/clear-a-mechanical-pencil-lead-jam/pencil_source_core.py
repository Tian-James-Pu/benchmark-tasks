"""Photo-bounded PG1019-family feed-core continuation.

The official pictures show the exposed chuck ahead of the black body thread,
inside a substantially smaller removable metal tip than the initial prototype.
Nominal 0.9 mm lead is preserved; the photographs' 0.5 mm bore is not copied.
External diameter brackets and internal flexure geometry remain estimates.
"""
import math,re
from dataclasses import replace
import numpy as np
import mechkit as m
from pencil_geometry import LOOK,collar,sector,add,physical,LEAD_R,GUIDE_R

OPEN_ANGLE=.0065  # rad; about 0.105 mm radial opening at the leading jaw end
PIVOT=np.array([0,.024,0.])

def source_core(pos,fixed=False,yaw=-45.):
    fp,fc=[],[]
    add(fp,fc,'source_guide',collar(GUIDE_R,.00065,0,.004),'steel',True)
    add(fp,fc,'source_nib',m.lathe([(GUIDE_R,.004),(.0008,.004),(.00150,.009),
        (.00150,.011),(.00135,.011),(.00135,.008),(.0007,.006),
        (GUIDE_R,.004)],96).aimed('+Y'),'steel',True)
    add(fp,fc,'source_ring_front_stop',collar(.00115,.00135,.00915,.0094),'brass',True)
    add(fp,fc,'source_ring_rear_stop',collar(.00115,.00135,.0112,.01145),'brass',True)
    add(fp,fc,'source_body_thread_envelope',collar(.0015,.0033,.011,.015),'body',True)
    add(fp,fc,'source_grip',collar(.0026,.00475,.015,.051),'steel',True)
    add(fp,fc,'source_barrel',collar(.0026,.0037,.051,.140),'body',True)
    links=[];orient=[]
    pp,pc=[],[]
    add(pp,pc,'source_lead_tank',collar(.0010,.0013,.026,.140),'body',True)
    add(pp,pc,'source_button',m.cylinder(.0033,.010,(0,.145,0),axis='y'),'steel')
    links.append(m.slider('feed','Y',-.0035,0,pp,pc,mass=.003,spring=500.,damping=.1,
        friction=0,armature=0,gravity=True,com=(0,.09,0),inertia=(4e-6,4e-9,4e-6)))
    rp,rc=[],[]
    add(rp,rc,'source_chuck_ring',collar(.00095,.00125,.0100,.0112),'brass',True)
    links.append(m.slider('ring','Y',-.0006,0,rp,rc,mass=.000021,spring=0,damping=.0001,
        friction=0,armature=0,gravity=True,com=(0,.0106,0),inertia=(1.55e-11,2.59e-11,1.55e-11)))
    # Three individually compliant slotted-collet leaves, bending about their
    # rear root. A continuous conical closing face replaces the old cylindrical
    # land; all forces still arise from native ring/jaw/lead contacts.
    profile=[(.0078,.00066),(.0105,.00105),(.0125,.00070),(.024,.00070)]
    for i,ang in enumerate((0,120,240)):
        pp,pc=[],[]
        for k,((y0,r0),(y1,r1)) in enumerate(zip(profile,profile[1:])):
            g=sector(LEAD_R,r0,y0,y1)
            rr=np.hypot(g.v[:,0],g.v[:,2]);outer=rr>LEAD_R+.000001
            fac=(r0+(r1-r0)*(g.v[:,1]-y0)/(y1-y0))/np.maximum(rr,1e-12)
            g.v[outer,0]*=fac[outer];g.v[outer,2]*=fac[outer]
            g=g.moved(-PIVOT).turned(math.degrees(OPEN_ANGLE),'z').moved(PIVOT).turned(ang,'y')
            add(pp,pc,f'source_jaw{i}_band{k}',g,'brass',True)
        pts=np.vstack([p[1].v for p in pp]);com=tuple(pts.mean(axis=0))
        links.append(m.hinge(f'jaw{i}','Z',tuple(PIVOT),-1.,0,pp,pc,parent='feed',
            mass=.000035,spring=.01,damping=.00001,friction=0,armature=0,gravity=True,
            com=com,inertia=(8e-10,2e-11,8e-10)))
        orient.append((f'jaw{i}',ang))
    # Same retainer material approximation as the preceding gate; relocated
    # ahead of the source-visible chuck, not widened or made sticky to pass.
    for i,ang in enumerate((0,120,240)):
        pp,pc=[],[]
        add(pp,pc,f'source_retainer{i}',sector(.00044,.00068,.0055,.0060).turned(ang,'y'),'rubber',True)
        links.append(m.slider(f'retainer{i}','X',0,.00015,pp,pc,mass=2e-7,spring=1000.,
            damping=.0001,friction=0,armature=0,gravity=True,inertia=(5e-14,5e-14,5e-14)))
        orient.append((f'retainer{i}',ang))
    starts=[('feed',-.0012)]+[(f'jaw{i}',-OPEN_ANGLE) for i in range(3)]+[(f'retainer{i}',.00002) for i in range(3)]
    part=physical(m.mechanism('pencil',pos,links,LOOK,fp,fc,yaw_deg=yaw,
        free_mass=None if fixed else .014,frame_inertia=(1.8e-5,1.5e-7,1.8e-5),
        friction=(.4,.3),solver_iters=96,start=starts))
    build=part.build
    def oriented():
        text=build()
        for name,angle in orient:
            q=(math.cos(math.radians(angle)/2),0,math.sin(math.radians(angle)/2),0)
            value='('+', '.join(f'{v:.12g}' for v in q)+')'
            pat=r'(def Physics(?:Prismatic|Revolute)Joint "'+name+r'_joint".*?\n    })'
            text=re.sub(pat,lambda mt:mt.group(1).replace('(1, 0, 0, 0)',value),text,flags=re.S)
        return text
    return replace(part,build=oriented)
