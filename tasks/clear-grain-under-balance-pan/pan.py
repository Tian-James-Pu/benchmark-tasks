"""Source-sized 70 mm liftable stainless cover; inferred 0.5 mm sheet."""
import math,numpy as np
import mechkit as m
from scaled_convex import scaled_convex_part
TOP=.7475
LOOKS={"steel":m.Look(color=m.rgb(.65,.67,.68),metallic=.8,roughness=.35)}
def fine(c):return c.replace("contactOffset = 0.004","contactOffset = 0.00002")
def build_pan():
    # A separate, liftable cover: 70 mm overall, 0.5 mm sheet. No constraint
    # holds the cover to the sensor. Side walls are separate local box colliders.
    pp=[('top',m.cylinder(.035,.0005,(0,0,.05725)),'steel')]
    pc=[fine(m.col_cylinder('top_collision',.035,.0005,(0,0,.05725)))]
    segments=64
    for i in range(segments):
        a=2*math.pi*i/segments
        radial=np.array([math.cos(a),math.sin(a),0.])
        tangent=np.array([-math.sin(a),math.cos(a),0.])
        # Width overlaps neighbors slightly, keeping the rim continuous.
        R=np.column_stack((radial,tangent,[0,0,1]))
        mesh=m.box((.0005,.0036,.0045),(0,0,0))
        mesh=m.Mesh(mesh.v@R.T+radial*.03475+[0,0,.05475],mesh.f)
        pp.append((f'rim_{i}',mesh,'steel'))
        # Scaled cooking is not needed for these boxes: use a rotated cube USD.
        # Keeping each convex piece explicit preserves the open underside.
        pc.append(fine(m.col_hull(f'rim_collision_{i}',mesh)))
    pan=m.free('pan',(.35,.016,TOP+.003),pp,pc,LOOKS,mass=.030,
        com=(0,0,.056),friction=(.3,.25),torsional_patch=None)
    return scaled_convex_part(pan)
