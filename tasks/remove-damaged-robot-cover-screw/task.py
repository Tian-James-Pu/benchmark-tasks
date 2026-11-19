"""SR051 native plier engagement gate. Unfinished task; all scores are zero.

Full-scale maker envelopes and an exposed M4x6 cover screw. Small cover details,
plier curves, tooth profile and spring are disclosed approximations. No grasp bond.
The screw has an elastic torsion fixture in this diagnostic; no extraction is claimed.
"""
import pathlib
import sys
import math
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Remove the damaged screw from the robot's cable cover."
SUCCESS_HOLD_S = 0.
LOOK = {name:m.Look(color=m.rgb(*rgb),metallic=.65 if name=='steel' else 0.,roughness=.5)
        for name,rgb in {'shell':(.78,.80,.81),'steel':(.57,.59,.61),
                         'green':(.045,.30,.22),'dark':(.11,.12,.13)}.items()}

def add(p,c,name,mesh,look='steel',collision=True):
    name=name.replace('-','m').replace('.','p')
    p.append((name,mesh,look))
    if collision:
        c.append(m.col_hull(name+'_c',mesh).replace('contactOffset = 0.004','contactOffset = 0.00005'))

# RH-2FRH2515 mounted envelope. Base and arm lengths from BFP-A3886A Fig2-9.
# Arm is parked away from the serviced cable-cover face; it is not removed.
BASE=np.array([.50,0.,TOP])
bp,bc=[],[]
add(bp,bc,'mounting_flange',m.box((.180,.140,.010),(0,0,.005)),'shell')
add(bp,bc,'base_housing',m.box((.176,.102,.180),(0,0,.100)),'shell')
add(bp,bc,'joint1',m.cylinder(.039,.035,(.040,0,.2025)),'steel')
add(bp,bc,'link1',m.box((.135,.062,.030),(.1075,0,.220)),'shell')
add(bp,bc,'joint2',m.cylinder(.040,.040,(.175,0,.225)),'steel')
add(bp,bc,'link2',m.box((.115,.095,.190),(.2325,0,.335)),'shell')
add(bp,bc,'tool_shaft',m.cylinder(.009,.150,(.290,0,.220)),'steel')
add(bp,bc,'tool_flange',m.cylinder(.020,.016,(.290,0,.145)),'steel')
# Flexible service conduit envelope, no functional manipulation in this task.
for i in range(15):
    a=math.pi*i/14
    x=.145+.105*math.cos(a);z=.430+.110*math.sin(a)
    add(bp,bc,f'conduit_{i}',m.sphere(.013,(x,0,z)),'dark')
for x in (-.060,.060):
    for y in (-.060,.060):
        add(bp,bc,f'base_bolt_{x}_{y}',m.cylinder(.006,.005,(x,y,.0125)))
# Thin original cover and surrounding fasteners are retained. Dimensions and
# positions are scaled photo estimates, not claimed as factory part drawings.
add(bp,bc,'cable_cover',m.box((.0015,.090,.115),(-.0889,0,.075)),'shell')
ROBOT=m.fixed('serviced_robot',tuple(BASE),bp,bc,LOOK,friction=(.4,.3))

def screw_head(parts,cols,tag,center,rounded=False):
    # ISO7380 M4 exterior Ø7.6 x2.2 mm; socket originally AF2.5 mm.
    # Rounded diagnostic recess Ø3.0 x1.3 mm, exterior unchanged.
    rings=[(0.,.0038),(.0006,.0038),(.0009,.00359),(.0016,.0031),(.0022,.0024)]
    for j in range(len(rings)-1):
        z0,r0=rings[j];z1,r1=rings[j+1]
        inner=.0015 if rounded and z0>=.0009 else 0.
        for k in range(96):
            a,b=2*math.pi*k/96,2*math.pi*(k+1)/96
            verts=[]
            for z,r in ((z0,r0),(z1,r1)):
                verts.extend([(inner*math.cos(a),inner*math.sin(a),z),
                              (r*math.cos(a),r*math.sin(a),z),
                              (r*math.cos(b),r*math.sin(b),z),
                              (inner*math.cos(b),inner*math.sin(b),z)])
            faces=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                            [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
            shape=m.Mesh(np.asarray(verts),faces).aimed('-X').moved(center)
            add(parts,cols,f'{tag}_{j}_{k}',shape)

sp,sc=[],[]
SCREW_CENTER=np.array([BASE[0]-.08965,BASE[1]-.035,TOP+.120])
for y in (-.035,.035):
    for z in (.030,.120):
        if y<0 and z>.1:continue
        screw_head(sp,sc,f'head_{y}_{z}',(BASE[0]-.08965,BASE[1]+y,TOP+z),
                   rounded=y<0 and z>.1)
SCREWS=m.fixed('cover_screws',(0,0,0),sp,sc,LOOK,friction=(.4,.3))
hp,hc=[],[]
screw_head(hp,hc,'damaged_head',(0,0,0),rounded=True)
# Diagnostic torsion fixture, NOT the final thread model. Measured head angle
# times100Nm/rad is its elastic reaction. 1.39–1.89Nm is installation torque,
# retained only as a conservative capability target, not calibrated breakaway.
HEAD_K=100.
head=m.hinge('head','X',(0,0,0),-5.,5.,hp,hc,mass=.0012,
             spring=HEAD_K,damping=.005,friction=0.,effort=4.,armature=0.,
             gravity=True,com=(-.001,0,0),inertia=(6e-9,4e-9,4e-9))
TARGET=m.mechanism('target_screw',tuple(SCREW_CENTER),[head],LOOK,
                   friction=(.4,.3),solver_iters=64)

# PZ58 catalogue p4: closed 160x54 mm, handle thickness15 mm. Pivot and
# closed handle outline traced from its dimensioned orthographic drawing.
# Opening20deg is a chosen starting configuration within the photographed
# range, not a claimed mechanical maximum. The unused opening limit is35deg.
TIP_X=.037
OPEN_DEG=20.
def branch(sign,opening=0.):
    p,c=[],[]
    from scipy.spatial import ConvexHull
    def put(name,mesh,look='steel'):
        add(p,c,name,mesh.turned(opening,'z'),look)
    def hull_mesh(verts):
        verts=np.asarray(verts);hull=ConvexHull(verts);faces=hull.simplices.copy()
        for j,face_ids in enumerate(faces):
            vv=verts[face_ids]
            if np.dot(np.cross(vv[1]-vv[0],vv[2]-vv[0]),hull.equations[j,:3])<0:
                faces[j]=face_ids[::-1]
        return m.Mesh(verts,faces)
    put('pivot',m.cylinder(.0085,.00425,(0,0,sign*.002125)))
    put('neck',m.box((.022,.007,.0085),(.013,sign*.004,0)))
    # ENGINEER US8656812B2 figs3/4/8 and manufacturer mechanism diagrams:
    # THREE longitudinal teeth, axially concave, with recessed side flanks.
    # Scale uses catalogue6mm tip thickness,2mm front relief and10mm distal
    # section. Internal proportions below are drawing estimates, not CAD.
    # Rigid contacts do not pretend to simulate plastic tooth indentation.
    zs=np.array([-3,-2,-1.5,-1,-.5,0,.5,1,1.5,2,3])
    front=np.array([0,0,.45,.12,1.,.40,1.,.12,.45,0,0])
    ds=np.array([0,.6,1.64,3.,4.38,5.8,7.12,8.5,10.])
    def axial_relief(depth):
        return np.interp(depth,[0,1.64,4.38,7.12,9.86,10],
                         [0,.356,.767,.822,.575,.56])
    for j,(za,zb) in enumerate(zip(zs[:-1],zs[1:])):
        for k,(da,db) in enumerate(zip(ds[:-1],ds[1:])):
            verts=[]
            for d in (da,db):
                for z in (za,zb):
                    retreat=max(0,abs(z)-1.5)*.8
                    x=TIP_X-(d+retreat)*.001
                    inner=np.interp(z,zs,front)+axial_relief(d)
                    outer=2.8+.26*d
                    verts.extend([(x,sign*inner*.001,z*.001),
                                  (x,sign*outer*.001,z*.001)])
            put(f'groove_{j}_{k}',hull_mesh(verts))
    # Rebuild the steel jaw taper and handle as convex strips preserving the
    # measured plan outline. Rounded TPR edges use an octagonal section.
    taper=m.prism([[.008,0],[.027,0],[.027,.0054],[.008,.00875]],-.003,.003)
    taper=m.Mesh(taper.v*np.array([1,sign,1]),taper.f if sign>0 else taper.f[:,::-1])
    put('jaw_taper',taper)
    # x, innerY, outerY, thickness in millimetres; signs applied below.
    sections=[(-7,1,8.5,8.5),(-13.4,1.7,9.8,8.5),
              (-29,5.6,13.6,13),(-47.4,9.2,19.6,13),
              (-53.6,8.3,21.5,15),(-60.9,15.4,25,15),
              (-70.3,18.4,26.9,15),(-84.9,17.9,27,15),
              (-99.5,15.2,25.2,15),(-113.7,10.2,20.9,15),
              (-123,15.4,16.5,12)]
    for i,(a,b) in enumerate(zip(sections[:-1],sections[1:])):
        verts=[]
        for x,inner,outer,thick in (a,b):
            center=(inner+outer)/2;half=(outer-inner)/2
            for angle in np.arange(8)*math.pi/4:
                verts.append((x*.001,-sign*(center+half*math.cos(angle))*.001,
                              thick*.0005*math.sin(angle)))
        put(f'handle_{i}',hull_mesh(verts),'steel' if i==0 else 'green')
    return p,c

rootp,rootc=branch(1)
linkp,linkc=branch(-1,-OPEN_DEG)
moving=m.hinge('moving_half','Z',(0,0,0),-15.,OPEN_DEG,linkp,linkc,mass=.065,
               spring=.015,damping=.0008,friction=0.,effort=.2,armature=1e-6,
               gravity=True,com=(-.050,.016,0),inertia=(1.5e-5,1.1e-4,1.2e-4))
PLIERS=m.mechanism('pliers',(.30,-.05,TOP+.008),[moving],LOOK,
                   frame_parts=rootp,frame_colliders=rootc,free_mass=.065,
                   frame_inertia=(1.5e-5,1.1e-4,1.2e-4),friction=(.4,.3),solver_iters=64)
OBJECTS=[ROBOT,SCREWS,TARGET,PLIERS]

class Trial:
    def reset(self,env=None):self.steps=0
    def step(self,env):
        self.steps+=1
        if self.steps==1 or self.steps%10==0:
            s=m.state(env,'pliers')
            r=env.unwrapped.scene['right_robot']
            print('SCREW_GATE',self.steps,{'pliers_pos':s.pos.tolist(),
                  'pliers_quat':s.quat_xyzw.tolist(),'hinge_deg':s.deg('moving_half'),
                  'velocity':s.lin_vel.tolist(),
                  'head_angle_deg':m.state(env,'target_screw').deg('head'),
                  'head_elastic_torque_Nm':HEAD_K*m.state(env,'target_screw').joint('head'),
                  'finger_q':m._np(r.data.joint_pos)[0,-2:].tolist()},flush=True)
T=Trial();m.on_reset(T.reset);m.on_step(T.step)
def score(objects):return 0.
def success(objects):return False
def demo():
    return {'right':[
        Waypoint(2.,(.225,-.164,TOP+.110),-.04695,'hover handles'),
        Waypoint(2.,(.225,-.164,TOP+.017),-.04695,'approach handles'),
        Waypoint(2.,(.225,-.164,TOP+.017),-.018,'grasp handles'),
        Waypoint(2.,(.225,-.164,TOP+.105),-.018,'lift tool'),
        Waypoint(2.,(.225,-.164,TOP+.105),-.018,'observe lifted tool'),
        Waypoint(2.,(.225,-.164,TOP+.105),-.010,'squeeze handles'),
        Waypoint(2.,(.225,-.164,TOP+.105),-.010,'observe pivot'),
        Waypoint(2.,(.225,-.164,TOP+.105),-.010,'hold')
    ]}
