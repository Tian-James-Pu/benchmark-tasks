"""100 g KERN347-07 envelope; reduced section profile, not manufacturer CAD.

Manufacturer: 22 mm diameter, 39 mm total height, stainless steel, M1 ±5 mg,
adjustment cavity with seal. The knob/neck subdivisions below are inferred.
This M1 weight verifies readings; it does not meet HT-120's ±2 mg calibration
weight specification. Internal adjustment cavity and seal are visual omissions.
"""
import mechkit as m

def weight(pos,mark=None):
    look={'steel':m.Look(color=m.rgb(.68,.69,.70),metallic=.9,roughness=.24)}
    look['ink']=m.Look(color=m.rgb(.03,.03,.03),roughness=.8)
    parts=[];cols=[]
    if mark is not None:parts.append(('mass_mark',mark.moved((0,0,.0391)),'ink'))
    for name,r,h,z in [('body',.011,.029,.0145),
                       ('neck',.006,.005,.0315),
                       ('knob',.010,.005,.0365)]:
        parts.append((name,m.cylinder(r,h,(0,0,z)),'steel'))
        cols.append(m.col_cylinder(name+'_collision',r,h,(0,0,z))
                    .replace('contactOffset = 0.004','contactOffset = 0.00002'))
    return m.free('verification_mass',pos,parts,cols,look,mass=.100,
                  friction=(.3,.25),torsional_patch=None)
