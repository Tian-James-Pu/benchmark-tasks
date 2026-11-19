"""Combined reseating, retainer closure and hands-off placement diagnostic; all task scores disabled.

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
    scene.PHYSICS_DT = 1. / 480.
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

# Reverse the complete populated connector, keeping all local cavity positions.
start_center = np.array([.30, -.10, TOP+.0107])
rot_z_pi = np.diag([-1., -1., 1.])
OBJECTS = [replace(o, pos=tuple(start_center + rot_z_pi@(np.asarray(o.pos)-C)),
                   yaw_deg=180., start_quat=(1.,0.,0.,0.) if o.name=='can_plus' else None)
           for o in OBJECTS]
import simready as sr
_original_prepare=sr._prepare
def _fine_tool_contacts(prim,spec):
    _original_prepare(prim,spec)
    if spec.name=='driver':
        from pxr import Usd, UsdPhysics, Sdf, PhysxSchema
        for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
            if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsInstanceProxy():
                p.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.0001)
                p.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
_tool_prepare=_fine_tool_contacts
def _prepare_reports(prim,spec):
    _tool_prepare(prim,spec)
    from pxr import UsdPhysics,PhysxSchema
    for p in prim.GetStage().Traverse():
        if any(a in str(p.GetPath()) for a in ('/RightArm/','/LeftArm/')) and p.GetName() in ('tip_left','tip_right') and p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
sr._prepare=_prepare_reports
OBJECTS += [sr.SimReady(name='driver',pos=(.24,.18,TOP+.001),size=(.148,.027,.026),
    mass=.03647007420659065,package='handTool_screwdriver_slotted_4mm_a01',
    usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd',convex=True,scale=1.,yaw=np.pi/2)]


from detent import PassiveDetent
detent = PassiveDetent()

from dataclasses import dataclass
from yam_arena import demo as _orientation_demo, ik
@dataclass
class Oriented(Waypoint):
    approach: tuple = (0, 0, -1)
    closing: tuple = (0, 1, 0)

def rotation(w):
    if not isinstance(w, Oriented):
        return _orientation_demo.grip_rotation(w.yaw)
    z=np.asarray(w.approach,dtype=float); z/=np.linalg.norm(z)
    x=np.asarray(w.closing,dtype=float); x-=x.dot(z)*z; x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))

def target(self,t):
    p,r,f=self.start_pos,self.start_rot,self.finger_start
    for w in self.waypoints:
        g=np.asarray(w.pos,dtype=float); q=rotation(w)
        if t<w.duration:
            a=ik.smoothstep(t/w.duration)
            return p+a*(g-p),ik.slerp_matrix(r,q,a),f+a*(w.finger-f),w.label
        t-=w.duration; p,r,f=g,q,w.finger
    return p,r,f,'done'

_orientation_demo.ArmPlan.target=target

_STATE = None
_ABORT = None
_DEMO_STATE = None
_PRESS_ABORT = None
_BENCH_LATCHED = False


class Trial:
    def reset(self, env=None):
        global _STATE, _ABORT, _DEMO_STATE, _PRESS_ABORT, _BENCH_LATCHED
        _STATE, _ABORT, _DEMO_STATE, _PRESS_ABORT = None, None, None, None
        _BENCH_LATCHED = False
        self.steps = 0

    def step(self, env):
        global _STATE, _DEMO_STATE
        self.steps += 1
        h = m.state(env, 'housing')
        q, finger, midpoints = {}, {}, {}
        origin = m._np(env.unwrapped.scene.env_origins[0])
        for arm in ('left', 'right'):
            robot = env.unwrapped.scene[arm+'_robot']
            ids = [robot.find_joints('joint'+str(j))[0][0] for j in range(1, 7)]
            q[arm] = m._np(robot.data.joint_pos)[0, ids].copy()
            finger[arm] = float(m._np(robot.data.joint_pos)[0, robot.find_joints('joint7')[0][0]])
            ids = [robot.find_bodies(n)[0][0] for n in ('tip_left', 'tip_right')]
            midpoints[arm] = m._np(robot.data.body_link_pose_w)[0, ids, :3].mean(0)-origin
        leads = {n: m.state(env, n) for n in ('lead', 'can_minus', 'can_plus')}
        local = {n: h.to_local(v.pos)[0] for n, v in leads.items()}
        _STATE = dict(housing=h, q=q, finger=finger, midpoints=midpoints, local=local)
        raw = m._np(env.unwrapped.arena_world.get_pose_w('driver'))[0].copy()
        raw[:3] -= origin
        tool_R = m.quat_matrix(raw[3:])
        tool_tip = raw[:3]+tool_R@np.array([.10284217447, 0., .01320824353])
        from yam_arena import station
        tcp = ik.tcp_pose(q['left'], station.BASES['left'])[0]
        _DEMO_STATE = dict(housing=h, q=q, finger_midpoints=midpoints,
            tool_pos=raw[:3].copy(), tool_R=tool_R, tip=tool_tip, tcp=tcp)

        if self.steps == 1 or self.steps % 20 == 0:
            print('COMBINED_CONNECTOR_STATE', self.steps, {
                'housing': h.pos.tolist(), 'housing_quat': h.quat_xyzw.tolist(),
                'terminal_local': {n: v.tolist() for n, v in local.items()},
                'wire_joints': {n: v.joints for n, v in leads.items()},
                'lances_deg': {n: h.deg(n) for n in ('lance', 'lance2', 'lance3', 'lance4')},
                'tpa_mm': h.joint('tpa')*1000}, flush=True)


trial = Trial()
trial.reset()
m.on_reset(trial.reset)
m.on_step(trial.step)


def score(objects): return 0.
def success(objects): return False


original_action7 = _orientation_demo.ArmPlan.action7


def bench_action7(self, t):
    global _ABORT, _BENCH_LATCHED
    p, r, f, label = self.target(t)
    arm = 'left' if self.base_pos[1] > 0 else 'right'
    if _STATE is None:
        return original_action7(self, t)
    if _ABORT is not None:
        return _ABORT[arm], 'stopped after grasp loss'
    if arm != 'right' or label != 'observe and reseat':
        return original_action7(self, t)
    h = _STATE['housing']
    q = _STATE['q'][arm]
    tcp, tcp_R = ik.tcp_pose(q, self.base_pos)
    hr = h.rotation()
    if not hasattr(self, 'bench_start'):
        self.bench_start = (t, h.pos.copy(), hr.copy())
        self.marker = hr.T@(tcp-h.pos)
        marker = h.pos+hr@self.marker
        self.grasp = (tcp_R.T@(marker-_STATE['midpoints'][arm]), tcp_R.T@hr)
    elapsed = t-self.bench_start[0]
    goal, desired_R = self.bench_start[1].copy(), self.bench_start[2]
    primary = _STATE['local']['lead']
    # Observe contact position to stop the robot's descent. This never writes
    # terminal state or changes any physical constraint or material property.
    # Require the tooth to return toward its unloaded orientation as well as
    # the contact rear shoulder to pass it; near-seat position alone is insufficient.
    if (not hasattr(self, 'seated_at') and primary[1] <= -.0001 and abs(h.deg('lance')) <= 1.
            and np.linalg.norm(primary[[0, 2]]-[CX, CZ]) <= .001):
        self.seated_at = (t, h.pos.copy())
        _BENCH_LATCHED = True
        print('BENCH_SEATED_OBSERVED', {'time_s': t, 'terminal_local': primary.tolist(),
              'lance_deg': h.deg('lance'),
              'descent_m': float(self.bench_start[1][2]-h.pos[2])}, flush=True)
    if hasattr(self, 'seated_at'):
        goal = self.seated_at[1].copy()
        goal[2] += .035*ik.smoothstep(min(1., (t-self.seated_at[0])/3.))
    else:
        goal[2] -= min(.065, .004*elapsed)
    marker = h.pos+hr@self.marker
    vector = tcp_R.T@(marker-_STATE['midpoints'][arm])
    slip = float(np.linalg.norm(vector-self.grasp[0]))
    angle = float(np.linalg.norm(ik._rot_error(tcp_R.T@hr, self.grasp[1])))

    def stop(reason):
        global _ABORT
        _ABORT = {a: np.r_[v, float(np.clip(_STATE['finger'][a]/-.04695, 0., 1.))]
                  for a, v in _STATE['q'].items()}
        print('BENCH_RESEAT_ABORT', {'time_s': t, 'reason': reason,
              'slip_mm': slip*1000, 'slip_deg': np.rad2deg(angle)}, flush=True)
        return _ABORT[arm], 'stopped'

    if slip > .005 or angle > np.deg2rad(12) or np.linalg.norm(goal-h.pos) > .070:
        return stop('grasp/pose guard')
    J = np.zeros((6, 6))
    offset = h.pos-tcp
    for j in range(6):
        q1 = q.copy(); q1[j] += 1e-6
        p1, r1 = ik.tcp_pose(q1, self.base_pos)
        omega = ik._rot_error(r1, tcp_R)/1e-6
        J[:3, j] = (p1-tcp)/1e-6+np.cross(omega, offset)
        J[3:, j] = .2*omega
    e = np.r_[goal-h.pos, .2*ik._rot_error(desired_R, hr)]
    delta = .1*J.T@np.linalg.solve(J@J.T+.02**2*np.eye(6), e)
    delta *= min(1., .015/max(float(np.max(np.abs(delta))), 1e-9))
    previous = getattr(self, 'servo_q', self.q.copy())
    lead = np.array([.3, .3, .3, .6, .6, .6])
    candidate = np.clip(previous+delta, q-lead, q+lead)
    if np.any(candidate < ik.LOWER+.015) or np.any(candidate > ik.UPPER-.015):
        return stop('joint limit')
    self.q = candidate
    self.servo_q = candidate.copy()
    if not hasattr(self, 'last_log') or t-self.last_log >= .5:
        print('BENCH_RESEAT_CONTROL', {'time_s': t, 'goal': goal.tolist(),
              'position_error_mm': float(np.linalg.norm(goal-h.pos))*1000,
              'rotation_error_deg': float(np.linalg.norm(ik._rot_error(desired_R, hr)))*180/np.pi,
              'slip_mm': slip*1000, 'seated_observed': hasattr(self, 'seated_at')}, flush=True)
        self.last_log = t
    return np.r_[candidate, float(np.clip(f/-.04695, 0., 1.))], label



_original_action7=_orientation_demo.ArmPlan.action7

def pose_action7(self,t):
    global _PRESS_ABORT
    p,r,f,label=self.target(t)
    arm='left' if self.base_pos[1]>0 else 'right'
    active=(arm=='left' and label in ('align tip','press retainer','hold retainer','withdraw tool')) or (arm=='right' and label=='hold for retainer press')
    if not active or _DEMO_STATE is None:
        return _original_action7(self,t)
    if _PRESS_ABORT is not None:
        return np.r_[_PRESS_ABORT['q'][arm],float(np.clip(f/-.04695,0.,1.))],'stopped after grasp loss'
    h=_DEMO_STATE['housing']
    if arm=='left':
        elapsed=t
        fraction=1.
        for w in self.waypoints:
            if elapsed<w.duration:
                fraction=ik.smoothstep(elapsed/w.duration);break
            elapsed-=w.duration
        y={'align tip':-.02425,'hold retainer':-.01525,'withdraw tool':-.03225}.get(label)
        if label=='press retainer':y=-.02425+.009*fraction
        if label in ('press retainer','hold retainer') and h.joint('tpa')>=.00495:
            self._retainer_done=True
        if getattr(self,'_retainer_done',False) and label in ('press retainer','hold retainer'):
            y=-.02625
        goal=h.to_world([(0.,y,0.)])[0]
        point=_DEMO_STATE['tip']
        current_R=_DEMO_STATE['tool_R']
        hr=h.rotation()
        if not hasattr(self,'_tool_in_housing_R'):
            self._tool_in_housing_R=hr.T@current_R
        desired_R=hr@self._tool_in_housing_R

    else:
        if not hasattr(self,'_hold_pose'):
            self._hold_pose=(h.pos.copy(),h.rotation().copy())
        goal,desired_R=self._hold_pose
        point=h.pos
        current_R=h.rotation()
    q_actual=_DEMO_STATE['q'][arm]
    tcp,tcp_R=ik.tcp_pose(q_actual,self.base_pos)
    offset=point-tcp
    if arm=='right':
        if not hasattr(self,'_housing_grasp_marker'):
            self._housing_grasp_marker=current_R.T@(tcp-point)
            self._housing_grasp_R=tcp_R.T@current_R
        marker=point+current_R@self._housing_grasp_marker
        # Independent simulated finger joints can shift their midpoint relative
        # to the palm. Measure material slip relative to the actual fingers.
        grasp_center=tcp_R.T@(marker-_DEMO_STATE['finger_midpoints']['right'])
        if not hasattr(self,'_housing_center_reference'):
            self._housing_center_reference=grasp_center.copy()
        translation=float(np.linalg.norm(grasp_center-self._housing_center_reference))*1000
        rotation=float(np.linalg.norm(ik._rot_error(tcp_R.T@current_R,self._housing_grasp_R)))*180/np.pi
        if not hasattr(self,'_grasp_log') or t-self._grasp_log>.25:
            print('RIGHT_GRASP',{'time_s':float(t),'translation_mm':translation,'rotation_deg':rotation},flush=True)
            self._grasp_log=t
        if translation>5. or rotation>12. or np.linalg.norm(goal-point)>.070:
            _PRESS_ABORT={'reason':'housing grasp lost','q':{a:q.copy() for a,q in _DEMO_STATE['q'].items()}}
            print('BIMANUAL_PRESS_ABORT',{'time_s':float(t),'position_error_mm':float(np.linalg.norm(goal-point))*1000,'arm':arm,'translation_mm':translation,'rotation_deg':rotation},flush=True)
            return np.r_[_PRESS_ABORT['q'][arm],float(np.clip(f/-.04695,0.,1.))],'stopped after grasp loss'
    if arm=='left':
        tool_pos=_DEMO_STATE['tool_pos']
        grip_tcp=_DEMO_STATE['tcp']
        if not hasattr(self,'_tool_grasp_marker'):
            self._tool_grasp_marker=current_R.T@(grip_tcp-tool_pos)
        marker=tool_pos+current_R@self._tool_grasp_marker
        # Measure translation at the original grasp, not at the remote tip:
        # small rotations around the grasp move the tip without axial slip.
        grasp=(tcp_R.T@(marker-_DEMO_STATE['finger_midpoints']['left']),tcp_R.T@current_R)
        if not hasattr(self,'_initial_grasp'):self._initial_grasp=grasp
        slip_mm=float(np.linalg.norm(grasp[0]-self._initial_grasp[0]))*1000
        slip_deg=float(np.linalg.norm(ik._rot_error(grasp[1],self._initial_grasp[1])))*180/np.pi
        if not hasattr(self,'_slip_log') or t-self._slip_log>.25:
            print('TOOL_GRASP_VECTOR',{'time_s':float(t),
                'material_marker_delta_actual_jaw_mm':((grasp[0]-self._initial_grasp[0])*1000).tolist(),
                'physical_tcp_minus_fk_mm':((grip_tcp-tcp)*1000).tolist(),
                'rotation_slip_deg':slip_deg},flush=True)
            self._slip_log=t

        if slip_mm>5. or slip_deg>12. or np.linalg.norm(goal-point)>.070:
            self._servo_aborted=True
            _PRESS_ABORT={'reason':'tool grasp lost','q':{a:q.copy() for a,q in _DEMO_STATE['q'].items()}}
            if not getattr(self,'_abort_logged',False):
                print('POSE_ABORT',arm,{'slip_mm':slip_mm,'slip_deg':slip_deg,'position_error_mm':float(np.linalg.norm(goal-point))*1000},flush=True)
                self._abort_logged=True
        if getattr(self,'_servo_aborted',False):
            return np.r_[_PRESS_ABORT['q'][arm],float(np.clip(f/-.04695,0.,1.))],'stopped after grasp loss'
    J=np.zeros((6,6));eps=1e-6
    for j in range(6):
        q1=q_actual.copy();q1[j]+=eps
        p1,r1=ik.tcp_pose(q1,self.base_pos)
        omega=ik._rot_error(r1,tcp_R)/eps
        J[:3,j]=(p1-tcp)/eps+np.cross(omega,offset)
        J[3:,j]=.2*omega
    error=np.r_[goal-point,.2*ik._rot_error(desired_R,current_R)]
    delta=J.T@np.linalg.solve(J@J.T+.02**2*np.eye(6),error)
    delta*=.10
    delta*=min(1.,.015/max(float(np.max(np.abs(delta))),1e-9))
    if not hasattr(self,'_servo_q'):self._servo_q=self.q.copy()
    candidate=self._servo_q+delta
    # Ordinary position commands; keep the stock gains and motor limits.
    lead_limit=np.array([.30,.30,.30,.60,.60,.60])
    candidate=np.clip(candidate,q_actual-lead_limit,q_actual+lead_limit)
    if np.any(candidate<ik.LOWER+.015) or np.any(candidate>ik.UPPER-.015):
        self._servo_aborted=True
        _PRESS_ABORT={'reason':'joint command limit','q':{a:q.copy() for a,q in _DEMO_STATE['q'].items()}}
        print('POSE_ABORT',arm,{'reason':'command approached joint limit','candidate':candidate.tolist()},flush=True)
        return np.r_[_PRESS_ABORT['q'][arm],float(np.clip(f/-.04695,0.,1.))],'stopped after grasp loss'
    self.q=np.clip(candidate,ik.LOWER,ik.UPPER)
    self._servo_q=self.q.copy()
    if not hasattr(self,'_servo_log') or t-self._servo_log>.5:
        print('POSE_CONTROL',arm,label,{'position_error_mm':float(np.linalg.norm(goal-point))*1000,'rotation_error_deg':float(np.linalg.norm(ik._rot_error(desired_R,current_R)))*180/np.pi,'q_command':self.q.tolist(),'q_actual':q_actual.tolist(),'retainer_done':getattr(self,'_retainer_done',False)},flush=True)
        self._servo_log=t
    return np.r_[self.q,float(np.clip(f/-.04695,0.,1.))],label


def combined_action7(self, t):
    global _ABORT
    _, _, f, label = self.target(t)
    arm = 'left' if self.base_pos[1] > 0 else 'right'
    if _ABORT is not None:
        return _ABORT[arm], 'stopped after bench failure'
    if arm == 'right' and label == 'observe and reseat':
        return bench_action7(self, t)
    if _STATE is not None and label in ('align tip', 'press retainer', 'hold retainer') and not _BENCH_LATCHED:
        _ABORT = {a: np.r_[v, float(np.clip(_STATE['finger'][a]/-.04695, 0., 1.))]
                  for a, v in _STATE['q'].items()}
        print('COMBINED_REPAIR_ABORT', {'time_s': t, 'reason': 'contact did not seat before press phase'}, flush=True)
        return _ABORT[arm], 'stopped before press'
    return pose_action7(self, t)

_orientation_demo.ArmPlan.action7 = combined_action7

def demo():
    def w(t, p, f, label, a=(0, 0, -1), c=(1, 0, 0)):
        return Oriented(t, p, f, label, 0, a, c)
    return {'right': [
        w(1.5, (.30, -.106, TOP+.12), -.022, 'hover housing'),
        w(1.5, (.30, -.106, TOP+.019), -.022, 'approach housing'),
        w(1., (.30, -.106, TOP+.019), -.008, 'grip housing'),
        w(2., (.30, -.106, TOP+.12), -.008, 'lift housing'),
        w(2., (.32, -.074, TOP+.168), -.008, 'turn wires downward', (0, 1, 0)),
        w(2., (.32, -.074, TOP+.168), 0., 'grip firmly', (0, 1, 0)),
        w(14., (.32, -.074, TOP+.168), 0., 'observe and reseat', (0, 1, 0)),
        w(34., (.32, -.074, TOP+.168), 0., 'hold for retainer press', (0, 1, 0)),
        w(4., (.30, -.106, TOP+.20), 0., 'return housing horizontal'),
        w(4., (.30, -.106, TOP+.019), 0., 'lower housing onto bench'),
        w(1., (.30, -.106, TOP+.019), -.030, 'release housing'),
        w(3., (.30, -.106, TOP+.12), -.030, 'retreat from housing'),
        w(4., (.30, -.106, TOP+.12), -.030, 'observe hands off')],
    'left': [
        w(1.5, (.24, .165, TOP+.12), -.025, 'hover handle', c=(-1,0,0)),
        w(1.5, (.24, .165, TOP+.016), -.025, 'approach handle', c=(-1,0,0)),
        w(1., (.24, .165, TOP+.016), 0., 'grip handle', c=(-1,0,0)),
        w(2., (.24, .165, TOP+.15), 0., 'lift driver', c=(-1,0,0)),
        w(2., (.24, .12, TOP+.35), 0., 'turn tip down', (0,-1,0), (-1,0,0)),
        w(16., (.24, .12, TOP+.35), 0., 'wait above bench', (0,-1,0), (-1,0,0)),
        w(4., (.319, -.066, TOP+.310), 0., 'move above connector', (0,-1,0), (-1,0,0)),
        w(4., (.319, -.066, TOP+.310), 0., 'align tip', (0,-1,0), (-1,0,0)),
        w(2., (.319, -.066, TOP+.301), 0., 'press retainer', (0,-1,0), (-1,0,0)),
        w(20., (.319, -.066, TOP+.301), 0., 'hold retainer', (0,-1,0), (-1,0,0)),
        w(4., (.319, -.066, TOP+.320), 0., 'withdraw tool', (0,-1,0), (-1,0,0)),
        w(4., (.24, .165, TOP+.20), 0., 'return driver horizontal', c=(-1,0,0)),
        w(4., (.24, .165, TOP+.016), 0., 'lower driver onto bench', c=(-1,0,0)),
        w(1., (.24, .165, TOP+.016), -.025, 'release driver', c=(-1,0,0)),
        w(3., (.24, .165, TOP+.12), -.025, 'retreat from driver', c=(-1,0,0)),
        w(4., (.24, .165, TOP+.12), -.025, 'observe hands off', c=(-1,0,0))]}

_nominal_target = _orientation_demo.ArmPlan.target
def release_target(self, t):
    p, r, f, label = _nominal_target(self, t)
    if _STATE is not None and label in ('return housing horizontal', 'return driver horizontal'):
        arm = 'left' if self.base_pos[1] > 0 else 'right'
        if not hasattr(self, '_return_start'):
            actual_p, actual_r = ik.tcp_pose(_STATE['q'][arm], self.base_pos)
            self._return_start = (actual_p.copy(), actual_r.copy())
        elapsed = t
        for waypoint in self.waypoints:
            if elapsed < waypoint.duration:
                a = ik.smoothstep(elapsed/waypoint.duration)
                p = self._return_start[0]+a*(np.asarray(waypoint.pos)-self._return_start[0])
                r = ik.slerp_matrix(self._return_start[1], rotation(waypoint), a)
                break
            elapsed -= waypoint.duration
    return p, r, f, label
_orientation_demo.ArmPlan.target = release_target

_combined_action = _orientation_demo.ArmPlan.action7
def release_action7(self, t):
    global _ABORT
    arm = 'left' if self.base_pos[1] > 0 else 'right'
    if _STATE is not None and _PRESS_ABORT is not None:
        return np.r_[_PRESS_ABORT['q'][arm], float(np.clip(_STATE['finger'][arm]/-.04695, 0., 1.))], 'stopped after press failure'
    if _STATE is not None and t >= 58. and not getattr(self, '_return_checked', False):
        h = _STATE['housing']
        # Controller guard only, never a task success condition or object-state write.
        if not _BENCH_LATCHED or h.joint('tpa') < .00495:
            _ABORT = {a: np.r_[q, float(np.clip(_STATE['finger'][a]/-.04695, 0., 1.))]
                      for a, q in _STATE['q'].items()}
            print('RELEASE_SEQUENCE_ABORT', {'time_s': t, 'reason': 'repair not complete before placement'}, flush=True)
        self._return_checked = True
    return _combined_action(self, t)
_orientation_demo.ArmPlan.action7 = release_action7

from scaled_convex import scaled_convex_part
OBJECTS = [scaled_convex_part(o) if hasattr(o,'build') else o for o in OBJECTS]
