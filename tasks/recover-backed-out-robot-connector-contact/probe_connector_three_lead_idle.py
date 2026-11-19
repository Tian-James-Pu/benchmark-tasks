"""Three populated cavities idle component diagnostic; all task scores disabled.

33472-4001 envelope; 33012 contact drawing dimensions. Internal cavity/lance
clearances and stiffness are approximations awaiting contact/load validation.
No terminal-to-housing joint, attachment, pose snapping, or success flag.
"""
import pathlib
import sys
import math
from dataclasses import replace
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m
_original_scene_patch = m._Finder.targets['yam_arena.scene']
def _fine_physics_scene(scene):
    _original_scene_patch(scene)
    scene.PHYSICS_DT = 1. / 2400.
m._Finder.targets['yam_arena.scene'] = _fine_physics_scene
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.
INSTRUCTION = 'Restore the robot harness connection.'
C = np.array([.30, 0., TOP + .0107])
CX, CZ = .00175, .003
BACKOUT = .004
LOOK = {k: m.Look(color=m.rgb(*v), metallic=.7 if k == 'metal' else 0., roughness=.55)
        for k, v in {'body': (.13, .14, .15), 'tpa': (.84, .84, .78),
                     'seal': (.10, .45, .20), 'metal': (.70, .72, .74),
                     'wire': (.78, .08, .05), 'plug': (.85, .85, .82)}.items()}

def add(parts, cols, name, shape, look='body', collision=True):
    parts.append((name, shape, look))
    if collision:
        cols.append(m.col_hull(name + '_c', shape).replace('contactOffset = 0.004', 'contactOffset = 0.00005'))

def box(parts, cols, name, size, pos, look='body', collision=True):
    add(parts, cols, name, m.box(size, pos), look, False)
    if collision:
        cols.append(m.col_box(name + '_c', size, pos).replace('contactOffset = 0.004', 'contactOffset = 0.00005'))

# Free housing. Open passages are assembled from separate convex surfaces.
# Outer flange/latch reproduce the measured envelope; internals are a reduced
# reconstruction, not a redistribution of the manufacturer's STEP model.
hp, hc = [], []
for side in (-1, 1):
    tag = 'neg' if side < 0 else 'pos'
    box(hp, hc, 'side_' + tag, (.0049, .037, .0147), (side * .00684, 0, 0))
    box(hp, hc, 'roof_' + tag, (.00878, .037, .0016), (0, 0, side * .00655))
    box(hp, hc, 'flange_' + tag, (.0011, .016, .0208), (side * .00984, .006, 0))
box(hp, hc, 'middle_web', (.00045, .030, .010), (0, .0035, 0))
box(hp, hc, 'row_web', (.0083, .030, .0018), (0, .0035, 0))
box(hp, hc, 'lower_flange', (.0186, .016, .0012), (0, .006, -.00997))
box(hp, hc, 'latch_root', (.006, .003, .005), (0, -.010, .0098))
box(hp, hc, 'mating_latch', (.006, .017, .0015), (0, .001, .01347))
box(hp, hc, 'latch_tooth', (.006, .002, .0018), (0, .0075, .0119))
# Interface seal outside the front barrel; it is visual only in this contact test.
for side in (-1, 1):
    tag = 'neg' if side < 0 else 'pos'
    box(hp, hc, 'seal_x_' + tag, (.001, .0015, .014), (side * .0095, -.008, 0), 'seal', False)
    box(hp, hc, 'seal_z_' + tag, (.018, .0015, .001), (0, -.008, side * .0075), 'seal', False)

# The central housing webs begin at the contact guides, leaving the front
# barrel open for the captive TPA plate's full travel.
# Four 3.5-mm-pitch cavities, two rows 6 mm apart. Key slots are narrower
# than the contact box: a reversed polarization rib meets a physical wall.
for j, (x, z) in enumerate(((CX, CZ), (-CX, CZ), (CX, -CZ), (-CX, -CZ))):
    for side in (-1, 1):
        tag = 'neg' if side < 0 else 'pos'
        box(hp, hc, f'guide_{j}_{tag}', (.00035, .029, .0037), (x + side * .00160, .003, z))
        box(hp, hc, f'key_floor_{j}_{tag}', (.0008, .029, .0003), (x + side * .001, .003, z - .00165))
    # Front shoulder stops axial over-insertion, leaving the blade opening.
    for side in (-1, 1):
        tag = 'neg' if side < 0 else 'pos'
        box(hp, hc, f'front_stop_{j}_{tag}', (.0028, .0005, .0004), (x, -.00845, z + side * .0012))
    if j == 3:
        # Unused circuit carry nonserviceable cavity plugs, not invisible holes.
        add(hp, hc, f'plug_{j}', m.cylinder(.00125, .022, (x, .007, z), axis='y'), 'plug')

# A cantilever approximation of the housing locking finger. The rear shoulder
# of a fully inserted contact catches its tooth. Its ramp permits insertion.
# Symmetric reduced lance reconstruction. Bottom-row cavity is mirrored about
# the housing midplane; the exact molded internal profiles are not sourced CAD.
# All four lances exist, including the unused cavity. No terminal is attached.
lances = []
for j, (x, z) in enumerate(((CX, CZ), (-CX, CZ), (CX, -CZ), (-CX, -CZ))):
    sign = 1 if z > 0 else -1
    lp, lc = [], []
    box(lp, lc, 'lance_beam', (.0015, .0073, .00035), (x, -.00135, z + sign * .00215))
    wedge = m.prism([(.0019, .0010), (.0023, .0010), (.0030, .00185), (.0019, .00185)], -.00075, .00075)
    v = wedge.v[:, [2, 0, 1]]
    v[:,2] *= sign
    wedge = m.Mesh(v + [x, 0, z], wedge.f[:, ::-1] if sign < 0 else wedge.f)
    add(lp, lc, 'lance_tooth', wedge)
    lances.append(m.hinge('lance' if j == 0 else f'lance{j+1}', 'X',
        (x, -.005, z + sign * .00215), 0 if sign > 0 else -15,
        15 if sign > 0 else 0, lp, lc, mass=.00004, spring=.05,
        damping=.00003, friction=0., effort=.1, armature=1e-7,
        gravity=True, com=(x, -.00135, z + sign * .00215),
        inertia=(2e-10, 1e-11, 2e-10)))

# Captive secondary retainer. Five mm travel follows AS-33472-100 D7;
# the exact variant's protrusion vs. service stroke remains a fidelity check.
# Its rail physically obstructs a raised lance as it moves to final position.
tp, tc = [], []
for side in (-1, 1):
    tag = 'neg' if side < 0 else 'pos'
    box(tp, tc, 'tpa_side_' + tag, (.0006, .012, .010), (side * .00395, -.01525, 0), 'tpa')
    box(tp, tc, 'tpa_face_' + tag, (.0085, .001, .0011), (0, -.02075, side * .00505), 'tpa')
# The service drawing shows a front plate with four openings, not an empty
# perimeter. These central webs reconstruct that plate at the existing cavity
# pitch. Web widths are reduced geometric approximations, not measured CAD.
box(tp, tc, 'tpa_face_row_web', (.0073, .001, .0022), (0, -.02075, 0), 'tpa')
box(tp, tc, 'tpa_face_column_web', (.00045, .001, .0099), (0, -.02075, 0), 'tpa')
for j, (x,z) in enumerate(((CX,CZ),(-CX,CZ),(CX,-CZ),(-CX,-CZ))):
    box(tp, tc, f'tpa_reinforcement_{j}', (.0018, .0014, .00045), (x, -.0038, z + math.copysign(.002675,z)), 'tpa')
tpa = m.slider('tpa', 'Y', 0., .005, tp, tc, mass=.0012, spring=8000., damping=6.2,
               friction=.1, effort=40., armature=1e-6, gravity=True,
               com=(0, -.013, 0), inertia=(8e-8, 2e-8, 8e-8))
HOUSING = m.mechanism('housing', tuple(C), [*lances, tpa], LOOK, frame_parts=hp,
                     frame_colliders=hc, free_mass=.00698, friction=(.45, .35),
                     solver_iters=64, frame_inertia=(.9275e-6, .43283e-6, 1.05117e-6),
                     start=[('lance', 0.)])
_build = HOUSING.build
HOUSING = replace(HOUSING, build=lambda: _build().replace('enabledSelfCollisions = 0', 'enabledSelfCollisions = 1').replace('solverVelocityIterationCount = 1', 'solverVelocityIterationCount = 8'))

# Freely moving crimped terminal and wire: no axial slide joint to the housing.
# Contact box 2.55 x 2.80 mm, 9.95-mm box length; total terminal20.45mm.
# Copper alloy sheet is0.30mm. The polarity rib is represented separately.
wp, wc = [], []
for side in (-1, 1):
    tag = 'neg' if side < 0 else 'pos'
    box(wp, wc, 'contact_side_' + tag, (.0003, .00995, .0028), (side * .001125, -.003025, 0), 'metal')
    box(wp, wc, 'contact_cap_' + tag, (.00195, .00995, .0003), (0, -.003025, side * .00125), 'metal')
box(wp, wc, 'polarization_rib', (.0008, .003, .00053), (0, -.003, -.001665), 'metal')
add(wp, wc, 'crimp_conductor', m.cylinder(.00105, .005, (0, .006, 0), axis='y'), 'metal')
add(wp, wc, 'crimp_insulation', m.cylinder(.00135, .002, (0, .01145, 0), axis='y'), 'metal')
box(wp, wc, 'crimp_spine', (.0018, .0105, .0003), (0, .0072, -.0011), 'metal')
L, R = .030, .0012
WIRE_START = .01245
add(wp, wc, 'wire_root', m.cylinder(R, L, (0, WIRE_START + L / 2, 0), axis='y'), 'wire')
links = []
for i in range(1, 4):
    p, c = [], []
    y = WIRE_START + (i + .5) * L
    add(p, c, f'wire{i}', m.cylinder(R, L, (0, y, 0), axis='y'), 'wire')
    links.append(m.hinge(f'wire{i}', 'X' if i % 2 else 'Z', (0, WIRE_START + i * L, 0),
                        -80, 80, p, c, mass=.00048, spring=.01, damping=.00005,
                        friction=0., effort=.15, armature=1e-9, gravity=True,
                        com=(0, y, 0), inertia=(3.617e-8, 3.456e-10, 3.617e-8),
                        parent=f'wire{i-1}' if i > 1 else None))
LEAD = m.mechanism('lead', tuple(C + [CX, BACKOUT, CZ]), links, LOOK,
                   frame_parts=wp, frame_colliders=wc, free_mass=.000871,
                   friction=(.65, .5), solver_iters=64,
                   frame_inertia=(2.06e-7, 8.1e-10, 2.06e-7))
# Housing8.180g + one0.160g plug =8.340g.
# Frame6.980g + TPA1.200g + four0.040g lances =8.340g.
# Each terminal0.391g +120mm wire at16g/m =2.311g.
# Total specimen15.273g, before tools and external harness fixtures.
# Root COM/inertia use a slender-wire/terminal estimate, not manufacturer CAD.
_lead_build = LEAD.build
LEAD = replace(LEAD, build=lambda: _lead_build().replace(
    'float3 physics:diagonalInertia = (',
    'point3f physics:centerOfMass = (0, 0.0154, 0)\n        float3 physics:diagonalInertia = (', 1))
OBJECTS = [HOUSING, LEAD]
# Two additional independent, mechanically retained terminals. Colors are
# diagnostic identifiers, not the manufacturer's specified wire colors.
def additional_lead(name, x, z, color):
    looks = dict(LOOK)
    looks['wire'] = m.Look(color=m.rgb(*color), roughness=.55)
    obj = m.mechanism(name, tuple(C + [x, -.00015, z]), links, looks,
        frame_parts=wp, frame_colliders=wc, free_mass=.000871,
        friction=(.65,.5), solver_iters=64,
        frame_inertia=(2.06e-7,8.1e-10,2.06e-7),
        start_quat=(0.,1.,0.,0.) if z < 0 else None)
    build = obj.build
    return replace(obj, build=lambda: build().replace(
        'float3 physics:diagonalInertia = (',
        'point3f physics:centerOfMass = (0, 0.0154, 0)\n        float3 physics:diagonalInertia = (', 1))
OBJECTS += [additional_lead('can_minus', -CX, CZ, (.10,.30,.70)),
            additional_lead('can_plus', CX, -CZ, (.45,.48,.50))]

class Trial:
    def reset(self, env=None): self.steps = 0
    def step(self, env):
        self.steps += 1
        h = m.state(env, 'housing')
        m.set_target(env, 'housing', 'tpa', 0. if h.joint('tpa') < .0025 else .005)
        if self.steps == 1 or self.steps % 20 == 0:
            print('POPULATED_CONNECTOR', self.steps, {
                'housing':h.pos.tolist(),
                'terminal_local':{n:h.to_local(m.state(env,n).pos)[0].tolist()
                    for n in ('lead','can_minus','can_plus')},
                'lances_deg':{n:h.deg(n) for n in ('lance','lance2','lance3','lance4')},
                'tpa_mm':h.joint('tpa')*1000},flush=True)
t = Trial();t.reset()
m.on_reset(t.reset);m.on_step(t.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    return {'right':[Waypoint(10,(.20,-.25,TOP+.22),-.04695,'idle',0.)]}
