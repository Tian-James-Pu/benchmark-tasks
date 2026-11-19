"""Inactive source-shaped PBP006 battery approximation with physical side clips.

Official photo establishes stem, opposing squeeze clips and pack outline.
Dimensions/mass and clip compliance are explicitly estimated, not OEM CAD/data.
Nothing here changes battery state or stapler driver force by a logical flag.
"""
import numpy as np
import mechkit as m
LOOKS={'case':m.Look(color=m.rgb(.09,.10,.10),roughness=.7),
 'clip':m.Look(color=m.rgb(.19,.20,.20),roughness=.85),
 'contact':m.Look(color=m.rgb(.64,.61,.46),metallic=.7,roughness=.3)}
# Pack body112x42x75mm;29x50x26mm stem;0.42kg total. Approximate envelope.
OUTLINE=[(.137,.226),(.145,.222),(.242,.222),(.249,.230),(.249,.258),(.241,.264),(.145,.264),(.137,.256)]
SHAPES={}
def clip(poly,axis,bound,greater):
 out=[]
 for a,b in zip(poly,poly[1:]+poly[:1]):
  ai=a[axis]>=bound if greater else a[axis]<=bound
  bi=b[axis]>=bound if greater else b[axis]<=bound
  if ai:out.append(a)
  if ai!=bi:
   t=(bound-a[axis])/(b[axis]-a[axis]);p=[a[i]+t*(b[i]-a[i]) for i in range(2)];out.append(tuple(p))
 return out

def make_battery(pos,start_quat=None):
 p,c=[],[]
 def add(parts,cols,name,mesh,look='case',body='frame'):
  parts.append((name,mesh,look));cols.append(m.col_hull(name+'_c',mesh).replace('contactOffset = 0.004','contactOffset = 0.0001'));SHAPES[body+'/'+name]=mesh
 add(p,c,'central_case',m.prism(OUTLINE,-.0275,.0275))
 # Outer side skins leave actual recesses around the moving squeeze plates.
 # Each region is a convex clipping of the original bevelled envelope.
 regions=[clip(OUTLINE,0,.198,False),clip(OUTLINE,0,.234,True)]
 middle=clip(clip(OUTLINE,0,.198,True),0,.234,False)
 regions += [clip(middle,1,.223,False),clip(middle,1,.262,True)]
 for side in [-1,1]:
  z0,z1=sorted([side*.0275,side*.0375])
  for i,region in enumerate(regions):
   if len(region)>2:add(p,c,f'side_{side+1}_{i}',m.prism(region,z0,z1))
 add(p,c,'stem',m.box((.029,.050,.026),(.216,.197,0)))
 # Top shoulder is behind the stem; visible two gold contact pads are physical.
 for side in [-1,1]:
  add(p,c,f'contact_{side+1}',m.box((.010,.008,.0005),(.216,.177,side*.01325)),'contact')
 links=[]
 for side in [-1,1]:
  bp,bc=[],[];name='clip_'+str(side+1)
  add(bp,bc,'squeeze_plate',m.box((.032,.034,.003),(.216,.243,side*.035)), 'clip',name)
  add(bp,bc,'clip_neck',m.box((.006,.009,.002),(.216,.2225,side*.035)), 'clip',name)
  add(bp,bc,'hook_bridge',m.box((.030,.002,.003),(.216,.2175,side*.0345)), 'clip',name)
  add(bp,bc,'latch_hook',m.box((.030,.004,.004),(.216,.2165,side*.034)), 'clip',name)
  # Native hinge and continuous spring; press inward about a lower X pivot.
  bounds=(0.,8.) if side>0 else (-8.,0.)
  links.append(m.hinge(name,'X',(.216,.262,side*.034),*bounds,bp,bc,
   mass=.008,spring=.25,damping=.015,friction=.001,armature=.000001,gravity=True))
 return m.mechanism('battery',pos,links,LOOKS,frame_parts=p,frame_colliders=c,
  free_mass=.404,frame_inertia=(.00022,.00056,.00047),start_quat=start_quat,
  friction=(.6,.5),solver_iters=64)
