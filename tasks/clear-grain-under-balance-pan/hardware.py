"""HT-120 envelope and removable-pan construction, reconstructed from its manual.

The shell profile, sensor clearance/stiffness, split masses and key force/travel
are estimates, not manufacturer CAD or firmware. No hidden mechanism moves the
pan or debris. All force transfer is through actual contacts and the load slide.
"""
from dataclasses import replace
import math,numpy as np
import mechkit as m
from pan import build_pan,fine
from verification_weight import weight
TOP=.7475;ORIGIN=(.35,0.,TOP);STIFFNESS=20000.
LOOKS={
 'case':m.Look(color=m.rgb(.80,.81,.79),roughness=.55),
 'edge':m.Look(color=m.rgb(.36,.38,.36),roughness=.65),
 'steel':m.Look(color=m.rgb(.65,.67,.68),metallic=.8,roughness=.35),
 'support':m.Look(color=m.rgb(.18,.18,.19),roughness=.65),
 'screen':m.Look(color=m.rgb(.59,.65,.51),roughness=.45),
 'ink':m.Look(color=m.rgb(.03,.05,.035),roughness=.9),
 'key':m.Look(color=m.rgb(.19,.23,.23),roughness=.7),
 'white':m.Look(color=m.rgb(.9,.92,.88),roughness=.8),
 'rubber':m.Look(color=m.rgb(.06,.07,.06),roughness=.9),
 'green':m.Look(color=m.rgb(.45,.65,.16),roughness=.35),
 'grain':m.Look(color=m.rgb(.92,.92,.88),roughness=.65)}
ANGLE=math.atan(1/3);C=math.cos(ANGLE);S=math.sin(ANGLE)
R=np.array([[1,0,0],[0,C,-S],[0,S,C]])
GLYPHS={'0':'abcdef','1':'bc','2':'abdeg','3':'abcdg','4':'bcfg',
 '5':'acdfg','6':'acdefg','7':'abc','8':'abcdefg','9':'abcdfg','-':'g','E':'adefg',' ':''}
FONT={'O':['01110','10001','10001','10001','10001','10001','01110'],
 'N':['10001','11001','11001','10101','10011','10011','10001'],
 'F':['11111','10000','10000','11110','10000','10000','10000'],
 'U':['10001','10001','10001','10001','10001','10001','01110'],
 'I':['111','010','010','010','010','010','111'],
 'T':['11111','00100','00100','00100','00100','00100','00100'],
 'S':['01111','10000','10000','01110','00001','00001','11110'],
 'R':['11110','10001','10001','11110','10100','10010','10001'],
 'E':['11111','10000','10000','11110','10000','10000','11111'],
 'Z':['11111','00001','00010','00100','01000','10000','11111'],
 'G':['01111','10000','10000','10111','10001','10001','01111'],
 '1':['00100','01100','00100','00100','00100','00100','01110'],
 '0':['01110','10001','10011','10101','11001','10001','01110'],
 'A':['01110','10001','10001','11111','10001','10001','10001'],
 'B':['11110','10001','10001','11110','10001','10001','11110'],
 'L':['10000','10000','10000','10000','10000','10000','11111'],
 ' ':['000']*7}
def panel_z(y):return .050+min(0.,y+.027)/3

def panel(mesh,xy,z=.0002):
 return m.Mesh(mesh.v@R.T+np.array([xy[0],xy[1],panel_z(xy[1])+z]),mesh.f)

def text_mesh(text,pitch=.00030):
 widths=[len(FONT[ch][0]) for ch in text];width=(sum(widths)+len(text)-1)*pitch
 x=-width/2;meshes=[]
 for ch,w in zip(text,widths):
  for row,line in enumerate(FONT[ch]):
   for col,on in enumerate(line):
    if on=='1':meshes.append(m.box((pitch*.8,pitch*.8,.00008),(x+(col+.5)*pitch,(3-row)*pitch,0)))
  x+=(w+1)*pitch
 return m.join(*meshes)

def outline(width,depth,radius):
 p=[]
 for cx,cy,start in [(width/2-radius,depth/2-radius,0),(-width/2+radius,depth/2-radius,90),(-width/2+radius,-depth/2+radius,180),(width/2-radius,-depth/2+radius,270)]:
  for a in np.linspace(start,start+90,7):p.append((cx+radius*math.cos(math.radians(a)),cy+radius*math.sin(math.radians(a))))
 # Add slope break along each straight side before triangulating.
 result=[]
 for i,a in enumerate(p):
  b=p[(i+1)%len(p)];result.append(a)
  if (a[1]+.027)*(b[1]+.027)<0:
   t=(-.027-a[1])/(b[1]-a[1]);result.append((a[0]+t*(b[0]-a[0]),-.027))
 return result

def shell():
 shape=m.prism(outline(.136,.150,.012),.003,.050)
 v=shape.v.copy()
 for q in v:
  if q[2]>.049:q[2]=panel_z(q[1])
 return m.Mesh(v,shape.f)

def make_balance():
 case=shell();parts=[('case',case,'case')];cols=[fine(m.col_hull('case_collision',case))]
 for i,(x,y) in enumerate([(-.05,-.055),(.05,-.055),(-.05,.055),(.05,.055)]):
  foot=m.cylinder(.009,.003,(x,y,.0015));parts.append((f'foot{i}',foot,'rubber'));cols.append(fine(m.col_cylinder(f'foot_collision{i}',.009,.003,(x,y,.0015))))
 # A shallow perimeter seam and battery access cover are visual details only.
 seam=m.prism(outline(.1362,.1502,.012),.012,.0127)
 parts.append(('case_seam',seam,'edge'))
 screen=panel(m.box((.076,.021,.0004)),(-.022,-.053),.0005)
 parts.append(('lcd_background',screen,'screen'))
 # Six 15 mm characters, with a separate decimal point. Digits are visibility
 # driven at 10 Hz by measured spring force, never by object identity.
 for i in range(6):
  x=-.050+i*.0105
  for s,(cx,cy,dx,dy) in {
   'a':(0,.0068,.0052,.0012),'g':(0,0,.0052,.0012),'d':(0,-.0068,.0052,.0012),
   'f':(-.003,.0035,.0012,.0055),'b':(.003,.0035,.0012,.0055),
   'e':(-.003,-.0035,.0012,.0055),'c':(.003,-.0035,.0012,.0055)}.items():
   mesh=panel(m.box((dx,dy,.0001),(cx,cy,0)),(x,-.053),.0008)
   parts.append((f'lcd_{i}_{s}',mesh,'ink',True))
 parts.append(('lcd_decimal',panel(m.cylinder(.0007,.0001),(-.013,-.0595),.0009),'ink',True))
 parts.append(('lcd_stable',panel(m.cylinder(.0012,.0001),(-.057,-.043),.0009),'ink',True))
 parts.append(('unit_g',panel(text_mesh('G',.00045),(.012,-.058),.0009),'ink'))
 # The spirit-level housing is shown; bubble translation follows actual tilt.
 parts.append(('level',panel(m.cylinder(.005,.0005),(-.054,-.032),.0005),'green'))
 parts.append(('level_ring',panel(m.tube([(math.cos(a)*.0025,math.sin(a)*.0025,0) for a in np.linspace(0,2*math.pi,25)],.00015,8),(-.054,-.032),.0009),'ink'))
 parts.append(('level_bubble',panel(m.cylinder(.0018,.0001),(-.054,-.032),.0010),'white'))
 support=m.cylinder(.033,.002,(0,.016,.056))
 cell=m.slider('loadcell','Z',-.001,.001,[('support',support,'support'),('center_screw',m.cylinder(.0025,.0001,(0,.016,.05705)),'steel')],
  [fine(m.col_cylinder('support_collision',.033,.002,(0,.016,.056)))],mass=.025,spring=STIFFNESS,damping=12.,friction=0.,effort=20.,armature=0.,gravity=True,com=(0,.016,.056),inertia=(7e-6,7e-6,1.4e-5))
 links=[cell]
 for name,x,y,words in [('power',.034,-.045,['ON','OFF']),('units',.053,-.045,['UNITS']),('rezero',.034,-.060,['RE','ZERO'])]:
  cap=panel(m.box((.013,.010,.0006)),(x,y),.0008)
  p=[('cap',cap,'key')]
  for j,word in enumerate(words):
   q=text_mesh(word,.00025);q=q.moved((0,(.0013 if j==0 else -.0013) if len(words)>1 else 0,0))
   p.append((f'label{j}',panel(q,(x,y),.00115),'white'))
  links.append(m.slider(name,'Z',-.0003,0.,p,[fine(m.col_hull('cap_collision',cap))],at=(x,y,panel_z(y)),mass=.0005,spring=4000.,damping=1.,friction=0.,effort=3.,armature=0.,gravity=False,inertia=(1e-8,1e-8,1e-8)))
 result=m.mechanism('balance',ORIGIN,links,LOOKS,frame_parts=parts,frame_colliders=cols,free_mass=.5135,friction=(.6,.45),frame_inertia=(.001073,.000901,.001761))
 build=result.build
 def aligned_keys():
  text=build().replace("bool physxArticulation:enabledSelfCollisions = 0",
   "bool physxArticulation:enabledSelfCollisions = 0\n    float physxArticulation:sleepThreshold = 0")
  for name in ('power','units','rezero'):
   token=f'def PhysicsPrismaticJoint "{name}_joint"';a=text.index(token);b=text.index('\n    }',a)
   fragment=text[a:b].replace('quatf physics:localRot0 = (1, 0, 0, 0)',f'quatf physics:localRot0 = ({math.cos(ANGLE/2)}, {math.sin(ANGLE/2)}, 0, 0)').replace('quatf physics:localRot1 = (1, 0, 0, 0)',f'quatf physics:localRot1 = ({math.cos(ANGLE/2)}, {math.sin(ANGLE/2)}, 0, 0)')
   text=text[:a]+fragment+text[b:]
  return text
 return replace(result,build=aligned_keys)

def build():
 grain=m.box((.003,.003,.003))
 return [make_balance(),build_pan(),m.free('grain',(.38475,.016,TOP+.0515),[('salt',grain,'grain')],[fine(m.col_box('salt_collision',(.003,.003,.003)))],LOOKS,mass=2165.*.003**3,friction=(.35,.25),torsional_patch=None),weight((.23,-.18,TOP),text_mesh("100G",.0004))]

def camera():
 from yam_arena import station
 from yam_arena.rig import Lens
 original=station.rig
 def rig(top_cam='calibrated'):
  base=original(top_cam);forward=np.asarray((.31,0.,.80))-np.asarray(base.top_cam.pos);forward/=np.linalg.norm(forward)
  right=np.cross(forward,(0,0,1));right/=np.linalg.norm(right);up=np.cross(right,forward)
  rotation=station.matrix_to_quat_xyzw(np.column_stack((right,up,-forward)))
  cam=replace(base.top_cam,rot_xyzw=tuple(rotation),lens=Lens(900.,900.,320.,180.,640,360),convention='opengl')
  return replace(base,name='station-balance-service',top_cam=cam)
 station.rig=rig
