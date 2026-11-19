"""Unvalidated SR-045 connector contact prototype; all task scores disabled.

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
        for k, v in {'body': (.13, .14, .15), 'tpa': (.23, .25, .26),
                     'seal': (.62, .30, .10), 'metal': (.70, .72, .74),
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
box(hp, hc, 'middle_web', (.00045, .037, .010), (0, 0, 0))
box(hp, hc, 'row_web', (.0083, .037, .0018), (0, 0, 0))
box(hp, hc, 'lower_flange', (.0186, .016, .0012), (0, .006, -.00997))
box(hp, hc, 'latch_root', (.006, .003, .005), (0, -.010, .0098))
box(hp, hc, 'mating_latch', (.006, .017, .0015), (0, .001, .01347))
box(hp, hc, 'latch_tooth', (.006, .002, .0018), (0, .0075, .0119))
# Interface seal outside the front barrel; it is visual only in this contact test.
for side in (-1, 1):
    tag = 'neg' if side < 0 else 'pos'
    box(hp, hc, 'seal_x_' + tag, (.001, .0015, .014), (side * .0095, -.008, 0), 'seal', False)
    box(hp, hc, 'seal_z_' + tag, (.018, .0015, .001), (0, -.008, side * .0075), 'seal', False)

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
    if j:
        # Unused circuits carry nonserviceable cavity plugs, not invisible holes.
        add(hp, hc, f'plug_{j}', m.cylinder(.00125, .022, (x, .007, z), axis='y'), 'plug')

# A cantilever approximation of the housing locking finger. The rear shoulder
# of a fully inserted contact catches its tooth. Its ramp permits insertion.
lp, lc = [], []
box(lp, lc, 'lance_beam', (.0015, .0073, .00035), (CX, -.00135, CZ + .00215))
# Wedge extruded in X, authored in Y/Z; its rear ramp lifts under insertion.
wedge = m.prism([(.0019, .0010), (.0023, .0010), (.0030, .00185), (.0019, .00185)], -.00075, .00075)
wedge = m.Mesh(wedge.v[:, [2, 0, 1]] + np.array([CX, 0, CZ]), wedge.f)
add(lp, lc, 'lance_tooth', wedge)
lance = m.hinge('lance', 'X', (CX, -.005, CZ + .00215), 0, 15, lp, lc,
                mass=.00004, spring=.05, damping=.00003, friction=0., effort=.1,
                armature=1e-7, gravity=True, com=(CX, -.00135, CZ + .00215),
                inertia=(2e-10, 1e-11, 2e-10))

# Captive secondary retainer. Five mm travel follows AS-33472-100 D7;
# the exact variant's protrusion vs. service stroke remains a fidelity check.
# Its rail physically obstructs a raised lance as it moves to final position.
tp, tc = [], []
for side in (-1, 1):
    tag = 'neg' if side < 0 else 'pos'
    box(tp, tc, 'tpa_side_' + tag, (.001, .012, .010), (side * .0039, -.01525, 0), 'tpa')
    box(tp, tc, 'tpa_face_' + tag, (.0088, .001, .0011), (0, -.02075, side * .0055), 'tpa')
box(tp, tc, 'tpa_reinforcement', (.0018, .002, .00045), (CX, -.0038, CZ + .002875), 'tpa')
tpa = m.slider('tpa', 'Y', 0., .005, tp, tc, mass=.0012, spring=0., damping=.12,
               friction=.1, effort=2., armature=1e-6, gravity=True,
               com=(0, -.013, 0), inertia=(8e-8, 2e-8, 8e-8))
HOUSING = m.mechanism('housing', tuple(C), [lance, tpa], LOOK, frame_parts=hp,
                     frame_colliders=hc, free_mass=.012, friction=(.45, .35),
                     solver_iters=64, frame_inertia=(1.5e-6, .7e-6, 1.7e-6),
                     start=[('lance', math.radians(5))])
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
                        -80, 80, p, c, mass=.000645, spring=.01, damping=.00005,
                        friction=0., effort=.15, armature=1e-9, gravity=True,
                        com=(0, y, 0), inertia=(4.9e-8, 4.6e-10, 4.9e-8),
                        parent=f'wire{i-1}' if i > 1 else None))
LEAD = m.mechanism('lead', tuple(C + [CX, BACKOUT, CZ]), links, LOOK,
                   frame_parts=wp, frame_colliders=wc, free_mass=.00105,
                   friction=(.65, .5), solver_iters=64,
                   frame_inertia=(3.8e-7, 1.5e-9, 3.8e-7))
OBJECTS = [HOUSING, LEAD]

class Trial:
    def reset(self, env=None): self.steps = 0
    def step(self, env):
        self.steps += 1
        if self.steps in (1, 40, 80, 120, 160, 200, 240, 280, 320, 360, 400, 440):
            h, w = m.state(env, 'housing'), m.state(env, 'lead')
            print('CONNECTOR_PROBE', self.steps, {'housing': h.pos.tolist(),
                  'terminal_local': h.to_local(w.pos)[0].tolist(),
                  'lance_deg': h.deg('lance'), 'tpa_mm': h.joint('tpa') * 1000,
                  'wire_joints': w.joints}, flush=True)

t = Trial()
t.reset()
m.on_reset(t.reset)
m.on_step(t.step)
def score(objects): return 0.
def success(objects): return False
def demo():
    def w(t, p, f, label, yaw): return Waypoint(t, p, f, label, yaw)
    body=(.300007, -.006, TOP+.025)
    wire=(.3018, .086, TOP+.022)
    return {
      'right': [w(2,(body[0],body[1],TOP+.12),-.022,'body hover',math.pi/2),
                w(2,body,-.022,'approach housing',math.pi/2),
                w(2,body,-.008,'hold housing',math.pi/2),
                w(11,body,-.008,'support housing',math.pi/2),
                w(1,body,-.022,'release housing',math.pi/2),
                w(2,(body[0],body[1],TOP+.15),-.022,'withdraw',math.pi/2)],
      'left': [w(6,(wire[0],wire[1],TOP+.12),-.012,'wire hover',-math.pi/2),
               w(2,wire,-.012,'approach wire',-math.pi/2),
               w(2,wire,0.,'pinch wire',-math.pi/2),
               w(3,(wire[0],wire[1]-.0042,wire[2]),0.,'reseat terminal',-math.pi/2),
               w(2,(wire[0],wire[1]-.0042,wire[2]),0.,'hold seated',-math.pi/2),
               w(1,(wire[0],wire[1]-.0042,wire[2]),-.012,'release wire',-math.pi/2),
               w(2,(wire[0],wire[1]-.0042,TOP+.15),-.012,'withdraw wire',-math.pi/2)]}
