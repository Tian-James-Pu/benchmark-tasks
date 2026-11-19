"""Geometric chain construction; no engagement flags or pose transmission.

Pitch and inner width are sourced. The roller/plate sizes and the rear-cage
geometry used by the offline preflight are explicitly unresolved estimates.
This module alone does not establish calibrated bicycle dynamics.
"""
from dataclasses import dataclass
import math
import numpy as np

PITCH = .0127
INNER_WIDTH = .00238125
PIN_LENGTH = .0073
# ISO 082 dimensional cross-check, not an exact KMC Z8.3 measurement.
ROLLER_RADIUS = .00775 / 2
PLATE_HEIGHT = .0098
PLATE_THICKNESS = .0008


def pitch_radius(teeth):
    if teeth < 3:
        raise ValueError('A sprocket needs at least three teeth')
    return PITCH / (2 * math.sin(math.pi / teeth))


@dataclass(frozen=True)
class Wheel:
    name: str
    center: tuple
    teeth: int
    wrap_sign: int = 1

    @property
    def radius(self):
        return pitch_radius(self.teeth)


class Route:
    """Taut signed-circle routing, with the guide pulley wrapped in reverse.

    The subsequent chord construction, not arc length, sets actual pin pitch.
    Tangencies and arcs are initialization geometry, never runtime constraints.
    """
    def __init__(self, wheels):
        self.wheels = tuple(wheels)
        outgoing, incoming = [], [None] * len(wheels)
        for i, w in enumerate(wheels):
            v = wheels[(i + 1) % len(wheels)]
            delta = np.asarray(v.center) - w.center
            distance = np.linalg.norm(delta)
            unit = delta / distance
            projection = (w.wrap_sign*w.radius-v.wrap_sign*v.radius)/distance
            if abs(projection) >= 1:
                raise ValueError('Pulley geometry has no nonintersecting tangent')
            normal = projection*unit + math.sqrt(1-projection**2)*np.array([unit[1], -unit[0]])
            outgoing.append(np.asarray(w.center) + w.wrap_sign*w.radius*normal)
            incoming[(i+1) % len(wheels)] = np.asarray(v.center) + v.wrap_sign*v.radius*normal
        self.pieces = []
        for i, w in enumerate(wheels):
            a = math.atan2(*(incoming[i]-w.center)[::-1])
            b = math.atan2(*(outgoing[i]-w.center)[::-1])
            angle = ((b-a)*w.wrap_sign) % (2*math.pi)
            self.pieces.append(dict(kind='arc', length=w.radius*angle, wheel=i,
                                    start_angle=a, angle=angle, sign=w.wrap_sign))
            end = incoming[(i+1) % len(wheels)]
            self.pieces.append(dict(kind='line', length=float(np.linalg.norm(end-outgoing[i])),
                                    start=outgoing[i], end=end))
        self.ends = np.cumsum([p['length'] for p in self.pieces])
        self.length = float(self.ends[-1])

    def at(self, distance):
        s = float(distance) % self.length
        i = min(int(np.searchsorted(self.ends, s, side='right')), len(self.pieces)-1)
        piece = self.pieces[i]
        t = (s-(self.ends[i-1] if i else 0.))/piece['length']
        if piece['kind'] == 'line':
            return (1-t)*piece['start'] + t*piece['end']
        w = self.wheels[piece['wheel']]
        a = piece['start_angle'] + t*piece['angle']*piece['sign']
        return np.asarray(w.center) + w.radius*np.array([math.cos(a), math.sin(a)])

    def pins(self, count, start=0.):
        """Walk exactly one pitch per pin; return closure arc residual.

        Caller selects the ordinary cage angle so the last pitch ends on the
        first pin. No link is stretched to close an arbitrary route.
        """
        distances = [float(start)]
        points = [self.at(start)]
        for _ in range(count):
            lo, hi = distances[-1], distances[-1] + 1.2*PITCH
            origin = points[-1]
            if np.linalg.norm(self.at(hi)-origin) < PITCH:
                raise ValueError('Route curvature exceeds pitch stepping bound')
            for _ in range(42):
                mid = (lo+hi)/2
                if np.linalg.norm(self.at(mid)-origin) < PITCH:
                    lo = mid
                else:
                    hi = mid
            distances.append((lo+hi)/2)
            points.append(self.at(distances[-1]))
        return np.asarray(points[:-1]), distances[-1]-start-self.length


def tooth_outline(teeth, roller_radius=ROLLER_RADIUS, pocket_clearance=.0001,
                  tip_addendum=.002, samples_per_tooth=20, phase=0.):
    """Polygonal roller pockets; valleys remain concave after decomposition.

    This is an ordinary roller-sprocket profile approximation. It does not
    reproduce the manufacturer's shift ramps, pins or asymmetric tooth cuts.
    Phase denotes a roller pocket, rather than a tooth tip.
    """
    radius = pitch_radius(teeth)
    pocket_radius = roller_radius + pocket_clearance
    points = []
    for i in range(teeth*samples_per_tooth):
        angle = phase + i*2*math.pi/(teeth*samples_per_tooth)
        relative = (angle-phase+math.pi/teeth) % (2*math.pi/teeth)-math.pi/teeth
        projection = radius*math.cos(relative)
        discriminant = pocket_radius**2 - (radius*math.sin(relative))**2
        r = min(radius+tip_addendum, projection-math.sqrt(discriminant)) if discriminant >= 0 else radius+tip_addendum
        points.append([r*math.cos(angle), r*math.sin(angle)])
    return np.asarray(points)


def link_frames(pins):
    """Rigid plate centers/yaws for a closed chain; anchors coincide exactly."""
    pins = np.asarray(pins, float)
    ends = np.roll(pins, -1, axis=0)
    lengths = np.linalg.norm(ends-pins, axis=1)
    if len(pins) % 2 or np.max(np.abs(lengths-PITCH)) > 1e-7:
        raise ValueError('Closed alternating chain requires even count and exact 12.7 mm pitch')
    return (pins+ends)/2, np.arctan2(ends[:, 1]-pins[:, 1], ends[:, 0]-pins[:, 0])


def roller_pocket_polygon(teeth, phase=0., bore_radius=.007,
                          roller_radius=ROLLER_RADIUS, pocket_clearance=.0001,
                          tip_addendum=.002, circle_segments=96):
    """Actual circular subtraction, retaining undercuts and the central bore.

    The earlier radial sampling shortcut can bridge a pocket and intersect a
    roller. Constructing the polygonal difference avoids that defect. Profile
    remains a reduced ordinary sprocket, not a manufacturer's shift geometry.
    """
    from shapely.geometry import Point
    radius = pitch_radius(teeth)
    if not 0 < bore_radius < radius-roller_radius-pocket_clearance:
        raise ValueError('Bore must stay inside the solid root of the teeth')
    quarters = max(16, int(math.ceil(circle_segments/4)))
    polygon = Point(0,0).buffer(radius+tip_addendum,quad_segs=max(quarters,teeth*4))
    polygon = polygon.difference(Point(0,0).buffer(bore_radius,quad_segs=quarters))
    for angle in phase+np.arange(teeth)*2*math.pi/teeth:
        center = (radius*math.cos(angle),radius*math.sin(angle))
        polygon = polygon.difference(Point(*center).buffer(roller_radius+pocket_clearance,quad_segs=quarters))
    if not polygon.is_valid or polygon.geom_type != 'Polygon':
        raise ValueError('Sprocket is not a connected polygon')
    return polygon


def standard_sprocket_polygon(teeth, phase=0., bore_radius=.007, arc_samples=24):
    """JIS S roller-sprocket profile from Tsubaki catalog p154 (PDF p155).

    Uses the manufacturer's seating arc, working arc, straight flank and
    topping arc equations in millimetres. This is a standard transmission
    profile, not Shimano's proprietary shift-tooth cuts or lateral chamfers.
    Source: tt-net.tsubakimoto.co.jp/lib/catalog/C_CDC_EN/book/data/all_page.pdf
    """
    from scipy.optimize import brentq
    from shapely.geometry import Point, Polygon
    p, dr = PITCH*1000, ROLLER_RADIUS*2000
    half = math.pi/teeth
    rp = p/(2*math.sin(half))
    outside = p*(.6+1/math.tan(half))/2
    seat = .5025*dr+.038
    # Catalog pa = p*(1+(Ds-dr)/Dp): seating-arc centers lie this much
    # outside the nominal roller pitch circle. Root diameter stays Dp-dr.
    a = np.array([0.,rp+seat-dr/2])
    A = math.radians(35+60/teeth)
    B = math.radians(18-56/teeth)
    c = a + .8*dr*np.array([-math.cos(A),math.sin(A)])
    E = 1.3025*dr+.038
    root_angles = np.linspace(-math.pi/2,-A,arc_samples)
    root = a+seat*np.c_[np.cos(root_angles),np.sin(root_angles)]
    working_angles = np.linspace(-A,-A+B,arc_samples)
    working = c+E*np.c_[np.cos(working_angles),np.sin(working_angles)]
    yz = dr*(1.4*math.sin(math.radians(17-64/teeth))-.8*math.sin(B))
    z = working[-1]+yz*np.array([math.sin(A-B),math.cos(A-B)])
    b = a+1.4*dr*np.array([math.cos(half),-math.sin(half)])
    F = dr*(.8*math.cos(B)+1.4*math.cos(math.radians(17-64/teeth))-1.3025)-.038
    if abs(np.linalg.norm(z-b)-F)>1e-10:
        raise ValueError('Topping-arc equations do not join the flank')
    start = math.atan2(*(z-b)[::-1])
    end = brentq(lambda t:np.linalg.norm(b+F*np.array([math.cos(t),math.sin(t)]))-outside,
                 math.pi/2,start,xtol=1e-14)
    topping_angles=np.linspace(start,end,arc_samples)
    topping=b+F*np.c_[np.cos(topping_angles),np.sin(topping_angles)]
    right=np.vstack([root,working[1:],z,topping[1:]])
    beta=math.atan2(right[-1,0],right[-1,1])
    if not 0<beta<half:
        raise ValueError('Standard outside diameter exceeds the tapered tooth tip')
    theta=np.linspace(beta,2*half-beta,8)
    tip=outside*np.c_[np.sin(theta),np.cos(theta)]
    direction=np.array([math.sin(half),math.cos(half)])
    reflection=2*np.outer(direction,direction)-np.eye(2)
    period=np.vstack([right,tip[1:-1],right[::-1]@reflection.T])
    points=[]
    for i in range(teeth):
        angle=phase-math.pi/2-2*i*half
        rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        points.extend((period[:-1]@rotation.T/1000).tolist())
    polygon=Polygon(points).difference(Point(0,0).buffer(bore_radius,quad_segs=32))
    if not polygon.is_valid or polygon.geom_type!='Polygon':
        raise ValueError('Standard tooth outline is not a connected polygon')
    return polygon
