"""Source-scale Schlage Classic C geometry for a contact feasibility coupon.

This is NOT yet a validated cylinder replica. Bitting and pin lengths are
dimensioned manufacturer values. Warding is the manufacturer's vector diagram,
scaled uniformly to its specified .343-inch blank height; machining clearance,
spring forces and pin-tip shape remain declared reconstruction assumptions.
"""
import math
import numpy as np
import shapely
from shapely.geometry import Polygon, Point, box
from scipy.spatial import ConvexHull
import mechkit as m

INCH = .0254
R = .250 * INCH
PIN_R = .115 * INCH / 2
HEIGHT = .343 * INCH
CENTERS = np.array([.231, .3872, .5434, .6996, .8558, 1.012])*INCH
CUTS = np.array([2, 5, 3, 6, 2, 4])
ROOTS = (.335-.015*CUTS)*INCH
BOTTOM_LENGTHS = (.165+.015*CUTS)*INCH
LENGTH = 1.125*INCH
FLAT = .031*INCH
FLANK = math.tan(math.radians(40))

# Schlage Key Systems catalog, p55, Classic C keyway. Coordinates preserve
# the vector outline. Only its smooth bottom closure is chord approximated.
_C = np.array([(263.382812,276.871094),(263.382812,275.003906),
 (265.949219,274.058594),(263.347656,269.5625),(263.347656,266.148438),
 (266.050781,265.167969),(266.050781,256.753906),(269.164062,256.753906),
 (269.164062,267.363281),(266.390625,268.378906),(269.164062,273.347656),
 (269.164062,277.410156)])
_end = np.array([269.164062,277.410156])
_c1, _c2, _last = np.array([269.164062,277.410156]),np.array([266.332031,277.953125]),_C[0]
_curve = np.array([(1-t)**3*_end+3*(1-t)**2*t*_c1+3*(1-t)*t*t*_c2+t**3*_last
                   for t in np.linspace(0,1,9)[1:-1]])
_C = np.vstack([_C,_curve])
_scale = HEIGHT/(_C[:,1].max()-_C[:,1].min())
_C[:,0] = (_C[:,0]-(266.050781+269.164062)/2)*_scale
_C[:,1] = (_C[:,1].max()-_C[:,1])*_scale-R
KEYWAY = Polygon(_C)
# Not source-certified. Preserve the source opening and inset the blank;
# never widen the opening to fit the robot or extractor.
KEY_CLEARANCE = .00004
BLANK = KEYWAY.buffer(-KEY_CLEARANCE, join_style=2)
BORE_CLEARANCE = .00004
PLUG_CLEARANCE = .00004


def triangles(poly, merge=False):
    out = list(shapely.constrained_delaunay_triangles(poly).geoms)
    assert abs(sum(x.area for x in out)-poly.area) < 1e-12
    if merge:
        changed=True
        while changed:
            changed=False
            for i,a in enumerate(out):
                for j in range(i+1,len(out)):
                    b=out[j]
                    if not a.intersects(b): continue
                    joined=a.union(b)
                    if joined.geom_type=='Polygon' and abs(joined.convex_hull.area-joined.area)<1e-14:
                        out[i]=joined.convex_hull
                        out.pop(j)
                        changed=True
                        break
                if changed: break
    return [np.asarray(t.exterior.coords)[:-1] for t in out if t.area>1e-14]


def hull_mesh(points):
    points = np.asarray(points)
    if len(points)<4:
        return None
    try:
        h = ConvexHull(points)
    except Exception:
        return None
    if h.volume < 1e-16:
        return None
    return m.Mesh(points, h.simplices)


def prism_y(poly, lo, hi):
    return hull_mesh([(x,y,z) for y in (lo,hi) for x,z in poly])


def clip(mesh, normal, offset, inside=True):
    """Convex mesh clipped by n.p<=offset, without altered contact dimensions."""
    if mesh is None:
        return None
    n = np.asarray(normal)* (1 if inside else -1)
    off = offset*(1 if inside else -1)
    v = mesh.v
    distances = v@n-off
    if np.max(distances) < 1e-12:
        return mesh
    if np.min(distances) > -1e-12:
        return None
    keep = [p for p,d in zip(v,distances) if d<=1e-12]
    edges = set(tuple(sorted((int(a),int(b)))) for f in mesh.f for a,b in zip(f,np.roll(f,1)))
    for a,b in edges:
        if distances[a]*distances[b]<0:
            keep.append(v[a]+distances[a]/(distances[a]-distances[b])*(v[b]-v[a]))
    return hull_mesh(np.unique(np.round(keep,12),axis=0))


def top(y):
    y = np.asarray(y)
    cutters = ROOTS[:,None]+FLANK*np.maximum(0,abs(y.reshape(1,-1)-CENTERS[:,None])-FLAT/2)
    return np.minimum(HEIGHT,cutters.min(axis=0))-R


def bitting_breaks():
    # Lower envelope of linear cutter flanks and root flats; exact breakpoints.
    seeds = sorted(set([0.,LENGTH]+[float(c+s*FLAT/2) for c in CENTERS for s in [-1,1]]))
    roots = ROOTS-R
    out = list(seeds)
    for a,b in zip(seeds[:-1],seeds[1:]):
        mid=(a+b)/2
        slopes=np.where(mid<CENTERS-FLAT/2,-FLANK,np.where(mid>CENTERS+FLAT/2,FLANK,0))
        heights=roots+FLANK*np.maximum(0,abs(mid-CENTERS)-FLAT/2)
        lines=list(zip(slopes,heights-slopes*mid))+[(0,HEIGHT-R)]
        for i,(s,c) in enumerate(lines):
            for t,d in lines[i+1:]:
                if abs(s-t)>1e-9 and a<(d-c)/(s-t)<b:
                    out.append((d-c)/(s-t))
    # Remove envelope-collinear intervals to avoid excessive collider pieces.
    pairs=[(y,float(top([y])[0])) for y in sorted(set(out))]
    kept=[]
    for p in pairs:
        while len(kept)>1 and abs(np.cross(np.subtract(kept[-1],kept[-2]),np.subtract(p,kept[-1])))<1e-13:
            kept.pop()
        kept.append(p)
    return kept


def key_cells():
    cells=[]
    for (a,za),(b,zb) in zip(bitting_breaks()[:-1],bitting_breaks()[1:]):
        slope=(zb-za)/(b-a)
        for tri in triangles(BLANK,merge=True):
            p=clip(prism_y(tri,a,b),(0,-slope,1),za-slope*a)
            if p is not None: cells.append(p)
    return cells


def subtract_bore(mesh, center, radius=PIN_R+BORE_CLEARANCE, sides=16):
    # Polygonal circular bore runs along Z. Circumscribed polygon keeps the
    # minimum radial clearance unchanged; maximum radial error is documented.
    v=mesh.v
    if v[:,0].min()>radius or v[:,0].max()<-radius or v[:,1].min()>center+radius or v[:,1].max()<center-radius:
        return [mesh]
    current=mesh
    pieces=[]
    for theta in np.linspace(0,2*np.pi,sides,endpoint=False):
        normal=np.array([math.cos(theta),math.sin(theta),0])
        offset=radius+normal[1]*center
        outside=clip(current,normal,offset,inside=False)
        if outside is not None: pieces.append(outside)
        current=clip(current,normal,offset)
        if current is None: break
    return pieces


def plug_cells():
    cross=Point(0,0).buffer(R,quad_segs=32).difference(KEYWAY)
    cells=[prism_y(t,0,LENGTH) for t in triangles(cross,merge=True)]
    # Pin chambers stop at the bottom of the upper keyway stem. They do not
    # drill through the bottom of the cylinder.
    zbottom=float(top(CENTERS).min())-0.0002
    out=[]
    for cell in cells:
        lower=clip(cell,(0,0,1),zbottom)
        if lower is not None: out.append(lower)
        upper=clip(cell,(0,0,1),zbottom,inside=False)
        if upper is None: continue
        remaining=[upper]
        for c in CENTERS:
            remaining=[piece for p in remaining for piece in subtract_bore(p,c)]
        out.extend(remaining)
    return out


def housing_cells():
    # Exterior test coupon, not a declared commercial mortise housing replica.
    # Only its bore radius is used by the contact retention experiment.
    cross=Point(0,0).buffer(.015,quad_segs=32).difference(Point(0,0).buffer(R+PLUG_CLEARANCE,quad_segs=32))
    cells=[prism_y(t,-.001,LENGTH+.001) for t in triangles(cross)]
    out=[]
    for cell in cells:
        lower=clip(cell,(0,0,1),0)
        if lower is not None: out.append(lower)
        upper=clip(cell,(0,0,1),0,inside=False)
        if upper is None: continue
        remaining=[upper]
        for c in CENTERS:
            remaining=[piece for p in remaining for piece in subtract_bore(p,c)]
        out.extend(remaining)
    return out


def bottom_pin(index):
    length=BOTTOM_LENGTHS[index]
    # Rounded/conical nose is a declared approximation; full .115-inch shank.
    profile=[(0,0),(.00025,0),(PIN_R,PIN_R-.00025),
             (PIN_R,length-.00025),(.0005,length),(0,length)]
    return m.lathe(profile,segments=24).moved((0,CENTERS[index],ROOTS[index]-R))


def parts_and_colliders(cells, prefix, look):
    return ([(f'{prefix}_{i}',cell,look) for i,cell in enumerate(cells)],
            [m.col_hull(f'{prefix}_{i}_c',cell) for i,cell in enumerate(cells)])


def source_dimensions():
    return {'units':'m','cut_codes':CUTS.tolist(),'chamber_centers':CENTERS.tolist(),
     'root_heights':ROOTS.tolist(),'bottom_pin_lengths':BOTTOM_LENGTHS.tolist(),
     'key_blank_height':HEIGHT,'pin_diameter':2*PIN_R,'plug_diameter_inferred':2*R,
     'keyway_width_scaled_from_diagram':KEYWAY.bounds[2]-KEYWAY.bounds[0],
     'assumed_key_clearance':KEY_CLEARANCE,'assumed_bore_clearance':BORE_CLEARANCE,
     'assumed_plug_clearance':PLUG_CLEARANCE,'pin_bore_polygon_sides':16,
     'bore_max_radial_polygon_error':(PIN_R+BORE_CLEARANCE)*(1/math.cos(math.pi/16)-1),
     'declared_limitations':['Warding vector diagram is not a machining drawing.',
      'Spring force, actual clearances, pin nose, housing exterior and plug length need measured assembly comparison.']}
