"""Physical feasibility probe: a fallen spider shade around an ordinary lamp harp.

No valid scorer or recovery demonstration yet. Do not submit this prototype.
"""
from __future__ import annotations
import json
import math
import os
import pathlib
import sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from yam_arena import station
from yam_arena.demo import ArmPlan, Waypoint
import yam_arena.demo as hd
from yam_arena.task_file import TABLE_TOP

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
SCORE_OVER = 'end'
INCH = .0254
XY = np.array([.35, -.02])
BASE_R, BASE_H = 7.25*INCH/2, 1.75*INCH
NECK_H = .060
SADDLE_Z = BASE_H + NECK_H
SOCKET_H = 2.5*INCH
HARP_H = 7*INCH
HARP_Z = SADDLE_Z + HARP_H
BULB_L, BULB_R = 4.44*INCH, 2.38*INCH/2
BULB_Z = SADDLE_Z + SOCKET_H - .025
SHADE_R0, SHADE_R1, SHADE_H = 5*INCH, 4*INCH, 7*INCH
SPIDER_Z = SHADE_H - .0127
LOOKS = {
 'wood':m.Look(m.rgb(.27,.12,.065),roughness=.38),
 'brass':m.Look(m.rgb(.68,.49,.21),metallic=.85,roughness=.25),
 'linen':m.Look(m.rgb(.91,.84,.70),roughness=.9),
 'steel':m.Look(m.rgb(.45,.43,.40),metallic=.7,roughness=.35),
 'glass':m.Look(glass=(.98,.99,1.0)),
 'black':m.Look(m.rgb(.035,.035,.035),roughness=.4),
 'filament':m.Look(m.rgb(.15,.13,.10),metallic=.6,roughness=.5),
}

def sector(a0,a1,z0,z1,r0,r1,thick):
    v=[]
    for z,r in ((z0,r0),(z1,r1)):
        for rad in (r-thick,r):
            for a in (a0,a1):v.append((rad*math.cos(a),rad*math.sin(a),z))
    f=np.array([(0,1,3),(0,3,2),(4,6,7),(4,7,5),(0,4,5),(0,5,1),
                (2,3,7),(2,7,6),(0,2,6),(0,6,4),(1,5,7),(1,7,3)])
    return m.Mesh(np.array(v),f)

def lamp():
    parts=[];cols=[]
    profile=[(0,0),(BASE_R*.91,0),(BASE_R,.008),(BASE_R,.014),
             (BASE_R*.84,.025),(BASE_R*.86,BASE_H),(0,BASE_H)]
    base=m.lathe(profile,64)
    parts.append(('base',base,'wood'));cols.append(m.col_hull('base_c',base))
    neck=m.cylinder(.014,NECK_H,(0,0,BASE_H+NECK_H/2))
    parts.append(('neck',neck,'brass'));cols.append(m.col_hull('neck_c',neck))
    socket=m.lathe([(0,SADDLE_Z),(.014,SADDLE_Z),(.019,SADDLE_Z+.016),
                    (.019,SADDLE_Z+SOCKET_H),(0,SADDLE_Z+SOCKET_H)],48)
    parts.append(('socket',socket,'brass'));cols.append(m.col_hull('socket_c',socket))
    # The glass envelope starts after the 25 mm screw base, hidden in the socket.
    profile=[(0,.025),(.0125,.025),(.014,.040),(.021,.055),
             (BULB_R,.074),(BULB_R,.084),(.026,.099),(.016,.109),(0,BULB_L)]
    envelope=m.lathe([(r,z+BULB_Z) for r,z in profile],64)
    parts.append(('clear_envelope',envelope,'glass'))
    cols.append(m.col_hull('glass_c',envelope))
    stem=m.cylinder(.002,.042,(0,0,BULB_Z+.046))
    parts.append(('glass_stem',stem,'glass'))
    for sy in (-1,1):
        parts.append((f'lead_{int(sy>0)}',m.tube([(0,sy*.002,BULB_Z+.026),
                     (0,sy*.011,BULB_Z+.070)],.00035,8),'steel'))
    # Coiled tungsten filament, not LED bars or a frosted plastic globe.
    coil=[]
    for i in range(121):
        a=12*2*math.pi*i/120
        coil.append((.0006*math.cos(a),-.011+.022*i/120,BULB_Z+.070+.0006*math.sin(a)))
    parts.append(('filament',m.tube(coil,.00013,6),'filament'))
    # Harp lies in the y-z plane; the two open sides remain open collision geometry.
    path=[(0,-.018,SADDLE_Z),(0,-.044,SADDLE_Z+.025),
          (0,-.052,HARP_Z-.060),(0,-.040,HARP_Z-.018),
          (0,-.018,HARP_Z),(0,.018,HARP_Z),(0,.040,HARP_Z-.018),
          (0,.052,HARP_Z-.060),(0,.044,SADDLE_Z+.025),(0,.018,SADDLE_Z)]
    parts.append(('harp',m.tube(path,.0016,10),'brass'))
    for i in range(len(path)-1):cols.append(m.col_hull(f'harp_c{i}',m.tube(path[i:i+2],.0016,8)))
    boss=m.cylinder(.009,.004,(0,0,HARP_Z+.002))
    stud=m.cylinder(.00635/2,.0127,(0,0,HARP_Z+.004+.0127/2))
    parts.extend([('boss',boss,'brass'),('stud',stud,'brass')])
    cols.extend([m.col_hull('boss_c',boss),m.col_hull('stud_c',stud)])
    # Ordinary transverse socket push-through bar. Its measured position, not a grasp,
    # determines the electrical state. Friction permits the bar to remain at either end.
    key_z=SADDLE_Z+.030
    key=m.cylinder(.0035,.052,(0,.004,key_z),axis='y')
    button=m.slider('switch','Y',-.008,0,[( 'switch_bar',key,'black')],
                    [m.col_hull('switch_c',key)],mass=.008,spring=0,damping=3,friction=1,
                    at=(0,.004,key_z),inertia=(1e-6,1e-6,1e-6))
    return m.mechanism('fixture',(*XY,TOP),[button],LOOKS,frame_parts=parts,
                       frame_colliders=cols,free_mass=.85,frame_inertia=(.005,.005,.006),
                       friction=(.7,.5),solver_iters=48)

def shade():
    parts=[];cols=[]
    for i in range(48):
        shell=sector(i*math.tau/48,(i+1)*math.tau/48,0,SHADE_H,SHADE_R0,SHADE_R1,.001)
        parts.append((f'linen{i}',shell,'linen'));cols.append(m.col_hull(f'liner_c{i}',shell))
    for k,(z,r) in enumerate(((.001,SHADE_R0),(SHADE_H-.001,SHADE_R1))):
        points=[(r*math.cos(a),r*math.sin(a),z) for a in np.linspace(0,math.tau,65)]
        parts.append((f'bound_rim{k}',m.tube(points,.0015,8),'linen'))
    # 1/4-27 stud passes through a normal spider washer. No enlarged grip tabs.
    for i in range(16):
        washer=sector(i*math.tau/16,(i+1)*math.tau/16,SPIDER_Z,SPIDER_Z+.0015,.0125,.0125,.0087)
        parts.append((f'washer{i}',washer,'brass'));cols.append(m.col_hull(f'washer_c{i}',washer))
    for i in range(3):
        a=i*math.tau/3
        points=[(.011*math.cos(a),.011*math.sin(a),SPIDER_Z+.0007),
                (.097*math.cos(a),.097*math.sin(a),SHADE_H-.002)]
        wire=m.tube(points,.0015,8)
        parts.append((f'spider{i}',wire,'brass'));cols.append(m.col_hull(f'spider_c{i}',wire))
    # Candidate start only. Its settled geometry and contact must be independently checked.
    theta=math.radians(-18)
    sy, cy = math.sin(theta/2), math.cos(theta/2)
    sz, cz = math.sin(5*math.pi/24), math.cos(5*math.pi/24)
    q=(sy*sz,sy*cz,cy*sz,cy*cz)
    return m.free('shade',(XY[0]+.112,XY[1],TOP+.145),parts,cols,LOOKS,
                  mass=.15,friction=(.4,.3),solver_iters=48,start_quat=q)

LAMP=lamp();SHADE=shade()
FINIAL=m.free('finial',(.19,-.20,TOP),
 [ ('cap',m.lathe([(0,0),(.008,0),(.008,.004),(.006,.0127),(0,.0127)],32),'brass')],
 [m.col_cylinder('cap_c',.008,.0127,(0,0,.00635))],LOOKS,mass=.015)
OBJECTS=[LAMP,SHADE,FINIAL]

DEBUG=[]
def probe(env):
    l,s=m.state(env,'fixture'),m.state(env,'shade')
    DEBUG.append({'lamp':l.pos.tolist(),'lamp_q':l.quat_xyzw.tolist(),
                  'shade':s.pos.tolist(),'shade_q':s.quat_xyzw.tolist(),'switch':l.joint('switch')})
    path=os.environ.get('LAMP_DEBUG')
    if path and (len(DEBUG)%20==0 or len(DEBUG)<3):
        pathlib.Path(path).write_text(json.dumps(DEBUG))
m.on_step(probe)

def score(objects):return 0.
def success(objects):return False

class Grip(float):pass
_plain=hd.grip_rotation
def _rotation(yaw=0.):return getattr(yaw,'rot',None) if hasattr(yaw,'rot') else _plain(yaw)
hd.grip_rotation=_rotation
def demo():
    out={}
    for side in ('left','right'):
        a=ArmPlan([],station.BASES[side]);g=Grip(0.);g.rot=a.start_rot
        out[side]=[Waypoint(3.,a.start_pos,-.04695,'observe settled fault',g)]
    return out
