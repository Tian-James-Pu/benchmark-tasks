"""Source-layout diagnostic, not a completed task or success demonstration.

P361 side profile, stepped nose and open empty feed channel. Battery is removed
for this service-access diagnostic. Cover/latch/battery operation and firing are
not yet validated. Exact internal dimensions are disclosed functional estimates.
"""
import sys,pathlib,json
from dataclasses import replace
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import mechkit as m
from scaled_convex import scaled_convex_part
from simready import SimReady
from closedloop import placeholder_demo
from yam_arena.task_file import TABLE_TOP
import full_case_reset_v20 as full_case_reset_v14
from battery_draft_v20 import clip,make_battery
SCENE='station';TOP=TABLE_TOP[SCENE]
INSTRUCTION='Clear the jam and leave the stapler ready to reload.'
SUCCESS_HOLD_S=3.
NOSE=np.array([.180,-.110,TOP+.0407])
TOOL=np.array([.235,-.260,TOP+.001])
LOOKS={'case':m.Look(color=m.rgb(.62,.74,.10),roughness=.65),
 'black':m.Look(color=m.rgb(.08,.09,.09),roughness=.8),
 'steel':m.Look(color=m.rgb(.65,.67,.69),metallic=.7,roughness=.3)}
# Local X points inward from muzzle, Y follows magazine, Z spans staple crown.
# Projected profile is estimated from official P361-photo-7, not exact CAD.
PROFILE={
 'motor':[(.090,-.022),(.116,-.027),(.245,-.027),(.264,-.018),(.264,.049),(.245,.065),(.174,.068),(.158,.055),(.094,.055)],
 'compressor':[(.091,.045),(.152,.045),(.152,.191),(.141,.209),(.108,.208),(.093,.192)],
 'grip':[(.188,.058),(.231,.058),(.221,.100),(.242,.172),(.237,.194),(.199,.190),(.193,.163),(.198,.113),(.179,.084)],
 'battery_shoe':[(.146,.191),(.238,.186),(.257,.206),(.257,.219),(.145,.219),(.133,.211)],
}
ALL_SHAPES={}
def add(parts,cols,name,mesh,look='steel',body='frame'):
 parts.append((name,mesh,look));cols.append(m.col_hull(name+'_c',mesh).replace('contactOffset = 0.004','contactOffset = 0.0001'))
 ALL_SHAPES[body+'/'+name]=mesh

def box(parts,cols,name,size,center,look='steel',body='frame'):
 mesh=m.box(size,center)
 parts.append((name,mesh,look));cols.append(m.col_box(name+'_c',size,center).replace('contactOffset = 0.004','contactOffset = 0.0001'))
 ALL_SHAPES[body+'/'+name]=mesh

def housing():
 p,c=[],[]
 # Convex subdivisions preserve the actual hand opening rather than filling it.
 for name in ('motor','compressor'):
  depth={'motor':.081,'compressor':.058,'battery_shoe':.074}[name]
  add(p,c,name,m.prism(PROFILE[name],-depth/2,depth/2),'case')
 # Hollow stem socket retains the outer grip/shoe profiles. Cavity is the
 # photo-scaled29x50x26mm stem plus1mm side clearance; no solid dummy socket.
 profiles=[('grip_upper',[(.188,.058),(.231,.058),(.221,.100),(.198,.113),(.179,.084)]),
 ('grip_lower',[(.198,.105),(.221,.100),(.242,.172),(.237,.194),(.199,.190),(.193,.163)]),
 ('battery_shoe',PROFILE['battery_shoe'])]
 for name,outline in profiles:
  depth=.074 if name=='battery_shoe' else .040
  for side in [-1,1]:
   z0,z1=sorted([side*.014,side*depth/2])
   skins=[outline] if name!='battery_shoe' else [clip(outline,1,.214,False),clip(clip(outline,1,.214,True),0,.199,False),clip(clip(outline,1,.214,True),0,.233,True)]
   for i,skin in enumerate(skins):
    if len(skin)>2:add(p,c,name+'_skin_'+str(side+1)+'_'+str(i),m.prism(skin,z0,z1),'black')
  pieces=[clip(outline,1,.170,False)]
  lower=clip(outline,1,.170,True)
  pieces += [clip(lower,0,.2005,False),clip(lower,0,.2315,True)]
  for i,poly in enumerate(pieces):
   if len(poly)>2:add(p,c,name+'_socket_'+str(i),m.prism(poly,-.014,.014),'black')
 # Fixed catch ledges meet the two spring hooks of the removable battery.
 for side in [-1,1]:
  for i,x in enumerate([.2005,.2315]):
   box(p,c,'battery_catch_'+str(side+1)+'_'+str(i),(.021,.002,.004),(x,.220,side*.035),'black')
 # Nose housing rails. Internal cavity remains connected to the feed channel.
 # The front2mm lip reproduces a broad3mm slot plus central1.8mm extension.
 # These are photo/staple-scaled estimates, not measured P361 section values.
 for side in (-1,1):
  box(p,c,'track_side_'+str(side+1),(.094,.014,.003),(.045,0,side*.0049))
 box(p,c,'track_back',(.094,.003,.0068),(.045,-.0085,0))
 # Feed-side wall is absent at X25..65mm, as required for real magazine feed.
 box(p,c,'track_front_land',(.027,.003,.0068),(.0115,.0085,0))
 box(p,c,'track_rear_land',(.029,.003,.0068),(.0795,.0085,0))
 # The broad mouth occupies Y[-1.5,+1.5],Z[-3.4,+3.4]mm.
 # Its central extension occupies Y[-3.3,-1.5],Z[-1.5,+1.5]mm.
 box(p,c,'lip_feed_side',(.002,.0055,.0068),(-.001,.00425,0))
 box(p,c,'lip_back_center',(.002,.0037,.003),(-.001,-.00515,0))
 for side in (-1,1):
  box(p,c,'lip_back_wing_'+str(side+1),(.002,.0055,.0019),(-.001,-.00425,side*.00245))
 # Real magazine runs across the staple plane, not along its crown width.
 # Max38.1mm staple length fixes the channel's longitudinal envelope.
 box(p,c,'mag_back_rail',(.003,.204,.012),(.0665,.112,0))
 for side in (-1,1):
  box(p,c,'mag_side_rail_'+str(side+1),(.040,.204,.002),(.045,.112,side*.0046))
 # Housing supporting the feed rails connects to the compressor case.
 box(p,c,'mag_mount',(.024,.192,.024),(.080,.117,0),'case')
 dp,dc=[],[]
 box(dp,dc,'driver_blade',(.090,.0012,.0058),(.0518,0,0),body='driver')
 driver=m.slider('driver','X',-.010,.040,dp,dc,mass=.025,spring=200.,damping=2.,
                 friction=.1,armature=.00001,effort=30.,gravity=True)
 # Open cover is a real prismatic part, not an invisible/removed wall.
 cp,cc=[],[]
 box(cp,cc,'cover_plate',(.003,.204,.012),(.0235,.112,0),'steel','cover')
 # Rear release button has a physical recess in the cover end.
 box(cp,cc,'cover_end_main',(.040,.015,.024),(.041,.2215,0),'black','cover')
 for side in [-1,1]:
  box(cp,cc,'cover_end_side_'+str(side+1),(.008,.015,.005),(.065,.2215,side*.0095),'black','cover')
 for i,y in enumerate([.228]):
  box(cp,cc,'cover_end_edge_'+str(i),(.008,.002,.014),(.065,y,0),'black','cover')
 cover=m.slider('cover','Y',0.,.150,cp,cc,mass=.16,friction=.3,damping=.5,gravity=True)
 bp,bc=[],[]
 box(bp,bc,'release_cap',(.003,.008,.012),(.0685,.223,0),'black','release')
 box(bp,bc,'release_neck',(.002,.009,.002),(.068,.2195,0),'black','release')
 box(bp,bc,'release_hook',(.002,.001,.008),(.068,.215,0),'black','release')
 release=m.slider('release','X',-.004,0.,bp,bc,parent='cover',mass=.006,
                   spring=300.,damping=.3,friction=.02,armature=.000001,gravity=True)
 for side in [-1,1]:
  box(p,c,'cover_catch_anchor_'+str(side+1),(.004,.004,.002),(.067,.216,side*.0055),'steel')
  box(p,c,'cover_catch_'+str(side+1),(.003,.002,.003),(.0675,.217,side*.003),'steel')
 # Follower rides with the cover and remains physically in its rail.
 # The fixed spring rest is set before settle; no state-triggered release.
 fp,fc=[],[]
 box(fp,fc,'follower',(.037,.002,.0058),(.045,.210,0),'black','follower')
 follower=m.slider('follower','Y',-.198,0.,fp,fc,parent='cover',mass=.008,
                    spring=20.,damping=.4,friction=.05,armature=.000001,gravity=True)
 part=m.mechanism('stapler',tuple(NOSE),[driver,cover,follower,release],LOOKS,
   frame_parts=p,frame_colliders=c,free_mass=2.477,frame_inertia=(.015,.015,.025),
   friction=(.6,.5),solver_iters=64)
 return replace(part,build=lambda original=part.build:original().replace("bool physxArticulation:enabledSelfCollisions = 0","bool physxArticulation:enabledSelfCollisions = 1"))

def bent_staple():
 p,c=[],[]
 # A 1/2inch staple has6.35mm outer crown and12.7mm outer legs.
 # This rectangular-wire model uses5.15mm crown centerline and12.2mm leg
 # centerlines (crown half-thickness0.5mm), total29.55mm.
 # Folding redistributes that length; rigid bent-wire approximation is disclosed.
 shoulder_y=-.004;half_z=.002575
 shoulder_len=np.hypot(.0012,shoulder_y)
 crown_len=np.hypot(half_z,shoulder_y)
 leg_len=(.02955-2*crown_len-2*shoulder_len)/2
 for side in (-1,1):
  points=[(.006,0,0),(.006,shoulder_y,side*half_z),(.0048,0,side*half_z),(.0048-leg_len,0,side*half_z)]
  for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
   a=np.array(a);b=np.array(b);d=b-a;x=d/np.linalg.norm(d)
   ref=np.array([0.,0.,1.]) if abs(x[2])<.9 else np.array([0.,1.,0.])
   z=np.cross(x,ref);z/=np.linalg.norm(z);y=np.cross(z,x)
   mesh=m.box((np.linalg.norm(d),.0012,.001))
   mesh=m.Mesh(mesh.v@np.column_stack([x,y,z]).T+(a+b)/2,mesh.f)
   add(p,c,'wire_'+str(side+1)+'_'+str(i),mesh,body='wire')
 return scaled_convex_part(m.free('bent_staple',tuple(NOSE),p,c,LOOKS,mass=.000278367,
      friction=(.4,.3),torsional_patch=None,solver_iters=64))

OBJECTS=[housing(),bent_staple(),make_battery(tuple(NOSE)),SimReady('screwdriver',pos=tuple(TOOL),size=(.148,.027,.026),mass=.03647,
 package='handTool_screwdriver_slotted_4mm_a01',usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd',convex=False)]
_n=0
@m.on_reset
def reset(env):
 global _n
 _n=0;full_case_reset_v14.snapshot(env.unwrapped,'post_settle')
@m.on_step
def observe(env):
 global _n
 _n+=1
 if _n%5==0:full_case_reset_v14.snapshot(env.unwrapped,'step_'+str(_n))
def score(objects):return 0.
def success(objects):return False
demo=placeholder_demo
def demo_script(ctx):yield from ctx.left.hold(3.)
