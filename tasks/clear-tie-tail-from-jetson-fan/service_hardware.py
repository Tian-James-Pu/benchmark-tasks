"""Physical service hardware around the manufacturer controller and fan."""
from pathlib import Path
from dataclasses import replace
import numpy as np
import mechkit as m
from hardware import LOOKS, CACHE
from scaled_convex import scaled_convex_part

SERVICE_LOOKS = dict(LOOKS, red=m.Look(color=m.rgb(.65,.04,.035)),
    blue=m.Look(color=m.rgb(.02,.16,.65)), yellow=m.Look(color=m.rgb(.85,.66,.04)),
    white=m.Look(color=m.rgb(.8,.8,.8)), base=m.Look(color=m.rgb(.20,.23,.25)))

def col(name,shape):
    return m.col_hull(name,shape).replace('contactOffset = 0.004','contactOffset = 0.00005')

def camera():
    from yam_arena import station
    from yam_arena.rig import Lens
    original = getattr(station.rig, '_fan_base', station.rig)
    def rig(top_cam='calibrated'):
        base = original(top_cam)
        pos = np.asarray(base.top_cam.pos)
        forward = np.asarray((.30,-.05,.80))-pos
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, (0,0,1)); right /= np.linalg.norm(right)
        up = np.cross(right,forward)
        rotation = station.matrix_to_quat_xyzw(np.column_stack((right,up,-forward)))
        # Same physical mount; a narrower lens shows the small service parts.
        cam = replace(base.top_cam, rot_xyzw=tuple(rotation),
                      lens=Lens(950.,950.,320.,180.,640,360), convention='opengl')
        return replace(base, name='station-controller-service', top_cam=cam)
    rig._fan_base = original
    station.rig = rig

def guard(pos):
    data = np.load(CACHE/'guard.npz')
    parts=[];cols=[]
    for i in range(4):
        shape=m.Mesh(data[f'guard_{i}_v'],data[f'guard_{i}_f'])
        parts.append((f'wire_{i}',shape,'silver'))
        cols.append(m.col_hull(f'wire_{i}_collision',shape).replace('"convexHull"','"none"').replace('contactOffset = 0.004','contactOffset = 0.00005'))
    # Four fixed mounting screws pass through the actual frame and guard ears.
    for i,(x,y) in enumerate(((-.016,-.016),(-.016,.016),(.016,-.016),(.016,.016))):
        head=m.cylinder(.0029,.002,(x,y,.001))
        parts.append((f'screw_{i}',head,'silver'));cols.append(col(f'screw_col_{i}',head))
    return m.fixed('guard',pos,parts,cols,SERVICE_LOOKS,friction=(.35,.25))

def supports(board,top):
    parts=[];cols=[]
    # Insulating bench plate and 12 mm nylon supports hold the development kit.
    # This represents a secured bench assembly, not an immovable loose board.
    plate=m.box((.115,.095,.003),(0,-.015,top-board[2]+.0015))
    parts.append(('insulating_plate',plate,'base'));cols.append(col('plate_col',plate))
    # The carrier's four corner mount holes are at raw CAD x=0/86 mm,
    # y=0/58 mm (our board origin is raw x=44,y=39 mm).
    for i,(x,y) in enumerate(((-.044,-.039),(.042,-.039),(-.044,.019),(.042,.019))):
        leg=m.cylinder(.003,.009,(x,y,-.0045))
        parts.append((f'foot_{i}',leg,'nylon'));cols.append(col(f'foot_col_{i}',leg))
        head=m.cylinder(.0028,.0015,(x,y,.0023))
        parts.append((f'board_screw_{i}',head,'silver'));cols.append(col(f'board_screw_col_{i}',head))
    for i,(x,y) in enumerate(((-.052,-.055),(.052,-.055),(-.052,.025),(.052,.025))):
        bolt=m.cylinder(.0025,.0012,(x,y,top-board[2]+.0036))
        parts.append((f'bench_screw_{i}',bolt,'silver'))
    return m.fixed('board_support',tuple(board),parts,cols,SERVICE_LOOKS)

def fan_wires(board,fan):
    parts=[]
    # Four coloured conductors terminate at the fan's frame and J7 header.
    # Routing is fixed to the board with clips, beyond the rotor and extraction path.
    for i,look in enumerate(('black','red','yellow','blue')):
        y=-.016+i*.001
        points=np.array([[.018,y,.004],[.026,y,.003],[.030,-.023,.001],
                         [.030,-.0335,-.009],[.0197+i*.00254,-.0335,-.0133]])
        shape=m.tube(points,.00045,sides=8)
        parts.append((f'{look}_lead',shape,look))
    # J7 is CAD solid 350: carrier-local centre (23.50,-31.21) mm.
    socket=m.box((.01016,.006,.006),(.0235,-.03351,-.0163))
    parts.append(('four_pin_plug',socket,'white'))
    return m.fixed('fan_wiring',tuple(fan),parts,(),SERVICE_LOOKS)

def power_switch(pos):
    # 304 inline rocker as shown in the XSD-0503000SUSD supply drawing.
    # The exterior and pivot travel are approximate; its two electrical states
    # follow the physical rocker angle. No robot-position trigger moves it.
    housing=m.box((.060,.026,.012),(0,0,.006))
    parts=[('housing',housing,'black')]
    cols=[col('housing_col',housing)]
    pivot=np.array([0,0,.013])
    cap=m.box((.021,.009,.0025),(0,0,.001)).turned(12,'y').moved(pivot)
    onmark=m.box((.001,.003,.0003),(.006,0,.0024)).turned(12,'y').moved(pivot)
    # Mechanical zero is the tilted ON endstop. This survives Arena's zero
    # joint reset without any runtime repositioning of the switch.
    link=m.hinge('rocker','Y',(0,0,.013),-24,0,
        [('cap',cap,'black'),('on_mark',onmark,'white')],
        [col('cap_col',cap)],mass=.0015,spring=0.,damping=.002,
        friction=.0025,effort=.1,armature=0.,inertia=(2e-8,6e-8,6e-8))
    return scaled_convex_part(m.mechanism('power',pos,[link],SERVICE_LOOKS,
         frame_parts=parts,frame_colliders=cols,friction=(.4,.3)))
