"""Full-scale Moen-family service reconstruction for a bounded contact gate.

GXP50C external envelope and GS181210B sink dimensions are manufacturer values.
Internal grind-ring/impeller dimensions and bearing drag are photo estimates;
they are not OEM CAD or a calibrated jam/breakaway load. No programmed jam.
"""
import math
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
import mechkit as m
from simready import SimReady

CX=.2; DY=.8097; COUNTER=.9144; FLOOR=.737; GRIND_DEPTH=.150; GRIND=FLOOR-GRIND_DEPTH
AXIS=np.array([CX,DY,GRIND])
LOOKS={n:m.Look(color=m.rgb(*rgb),metallic=metal,roughness=.4) for n,rgb,metal in [
 ('steel',(.57,.60,.62),.8),('dark',(.12,.14,.15),0),
 ('counter',(.62,.60,.56),0),('cabinet',(.71,.68,.63),0)]}
CONTACT_OFFSET=.0002

def collider(name,mesh):
 return m.col_hull(name,mesh).replace('contactOffset = 0.004',f'contactOffset = {CONTACT_OFFSET}')

def fixed(name,items,pos=(0,0,0)):
 return m.fixed(name,pos,[(f'p{i}',x,look) for i,(x,look) in enumerate(items)],
  [collider(f'c{i}',x) for i,(x,look) in enumerate(items)],LOOKS,friction=(.35,.25))

def annulus(inner,outer,z0,z1,n=96):
 """Exact polygonal annulus as convex quadrilateral prisms, never filled hull."""
 for i in range(n):
  a,b=2*math.pi*np.array([i,i+1])/n
  yield m.prism([(inner*math.cos(a),inner*math.sin(a)),
   (outer*math.cos(a),outer*math.sin(a)),(outer*math.cos(b),outer*math.sin(b)),
   (inner*math.cos(b),inner*math.sin(b))],z0,z1)

def environment():
 xl,xr=CX-.3685,CX+.3685;yb,yf=.6954,1.1014
 items=[]
 def box(lo,hi,look):
  lo,hi=np.array(lo),np.array(hi);items.append((m.box(hi-lo,(lo+hi)/2),look))
 for lo,hi in [
  ([CX-.45,.63,COUNTER-.025],[CX+.45,yb,COUNTER]),
  ([CX-.45,yf,COUNTER-.025],[CX+.45,1.28,COUNTER]),
  ([CX-.45,yb,COUNTER-.025],[xl,yf,COUNTER]),
  ([xr,yb,COUNTER-.025],[CX+.45,yf,COUNTER])]:box(lo,hi,'counter')
 for lo,hi in [
  ([CX-.45,.65,0],[CX+.45,.668,COUNTER-.025]),
  ([CX-.45,.65,0],[CX-.432,1.25,COUNTER-.025]),
  ([CX+.432,.65,0],[CX+.45,1.25,COUNTER-.025]),
  ([CX-.45,1.232,0],[CX+.45,1.25,COUNTER-.025])]:box(lo,hi,'cabinet')
 for lo,hi in [
  ([xl,yb-.0012,FLOOR],[xr,yb,COUNTER-.025]),
  ([xl,yf,FLOOR],[xr,yf+.0012,COUNTER-.025]),
  ([xl-.0012,yb,FLOOR],[xl,yf,COUNTER-.025]),
  ([xr,yb,FLOOR],[xr+.0012,yf,COUNTER-.025]),
  ([xl,yb,FLOOR-.0012],[CX-.052,yf,FLOOR]),
  ([CX+.052,yb,FLOOR-.0012],[xr,yf,FLOOR]),
  ([CX-.052,yb,FLOOR-.0012],[CX+.052,DY-.052,FLOOR]),
  ([CX-.052,DY+.052,FLOOR-.0012],[CX+.052,yf,FLOOR])]:box(lo,hi,'steel')
 # Close the square floor cutout to the specified circular92mm throat.
 for i in range(96):
  a,b=2*math.pi*np.array([i,i+1])/96
  def edge(t):return .052/max(abs(math.cos(t)),abs(math.sin(t)))
  poly=[(.046*math.cos(a),.046*math.sin(a)),(edge(a)*math.cos(a),edge(a)*math.sin(a)),
   (edge(b)*math.cos(b),edge(b)*math.sin(b)),(.046*math.cos(b),.046*math.sin(b))]
  items.append((m.prism(poly,FLOOR-.0012,FLOOR).moved((CX,DY,0)),'steel'))
 return fixed('sink_cabinet',items)

def housing():
 items=[]
 # Overall340mm tall,136mm maximum diameter. The selected drain opening
 # and150mm photo-informed turntable recess are preserved. The published
 #108mm dishwasher inlet must enter above this rotor; discharge is175mm below
 #the flange.150mm is a disclosed estimate, with140–175mm access bounds.
 for mesh in annulus(.046,.050,GRIND_DEPTH-.065,GRIND_DEPTH):items.append((mesh,'steel'))
 for mesh in annulus(.046,.068,GRIND_DEPTH-.067,GRIND_DEPTH-.063):items.append((mesh,'dark'))
 for mesh in annulus(.056,.068,GRIND_DEPTH-.340,GRIND_DEPTH-.065):items.append((mesh,'dark'))
 # Stationary grind ring: source photo shows rows of holes and ribs.
 # Reconstructed24 windows,8mm tall; the visible dimensions are estimates.
 for mesh in annulus(.055,.059,-.001,.003):items.append((mesh,'steel'))
 for mesh in annulus(.055,.059,.011,.017):items.append((mesh,'steel'))
 for mesh in annulus(.055,.059,.025,.031):items.append((mesh,'steel'))
 for i in range(24):
  for z in (.007,.021):
   mesh=m.box((.004,.004,.008),(.057,0,z)).turned(i*15,'z')
   items.append((mesh,'steel'))
 items.append((m.cylinder(.066,.010,(0,0,GRIND_DEPTH-.335),segments=64),'dark'))
 return fixed('housing',items,AXIS)

def rotor():
 disc=m.cylinder(.0538,.004,(0,0,-.002),segments=96)
 links=[m.hinge('turntable','Z',(0,0,0),-float('inf'),float('inf'),
  [('disc',disc,'steel')],[collider('disc_c',disc)],mass=.45,gravity=True,
  com=(0,0,-.002),inertia=(.000328,.000328,.000652),armature=0,
  friction=0,damping=.002)]
 for i,angle in enumerate((-25,155)):
  # A fixed pin on the rotating turntable carries each freely swivelling lug.
  # Radial face, side and height follow the primary cutaway qualitatively.
  pivot=np.array([.032,0,.002]);mesh=m.box((.021,.016,.016),(.0415,0,.010))
  mesh=mesh.turned(angle,'z');p=Rotation.from_euler('z',angle,degrees=True).apply(pivot)
  links.append(m.hinge(f'impeller{i}','Z',p,-float('inf'),float('inf'),
   [('lug',mesh,'steel')],[collider('lug_c',mesh)],parent='turntable',mass=.035,
   gravity=True,com=Rotation.from_euler('z',angle,degrees=True).apply((.0415,0,.010)),
   inertia=(1.493e-6,2.033e-6,2.033e-6),armature=0,friction=0,damping=.00002))
 part=m.mechanism('rotor',AXIS,links,LOOKS,friction=(.35,.25),solver_iters=64)
 build=part.build
 def native():
  return build().replace('solverVelocityIterationCount = 1','solverVelocityIterationCount = 4')
 return replace(part,build=native)

def spoon():
 # Native unscaled191mm NVIDIA steel spoon. Its authored SDF contact mesh
 # preserves the real bowl/curved handle; no invented groove or gripping tab.
 a=math.radians(30)
 R=np.array([[0,-math.sin(a),math.cos(a)],[1,0,0],[0,math.cos(a),math.sin(a)]])
 low=np.array([.053,0,.0012])+AXIS
 pos=low-R@np.array([0,-.13143453,.0005])
 return SimReady(name='spoon',package='kitchenware_spoon_b01',usd='sm_kitchenware_spoon_b01_01.usd',
  pos=tuple(pos),size=(.035,.191,.014),mass=.035,scale=1.,convex=False,
  quat_xyzw=tuple(Rotation.from_matrix(R).as_quat()))

def objects(include_spoon=True):
 return [environment(),housing(),rotor()]+([spoon()] if include_spoon else [])
