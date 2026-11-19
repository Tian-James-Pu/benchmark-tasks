"""Full-pose feedback press diagnostic; all task scores disabled.

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
BACKOUT = -.00015
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
    box(tp, tc, 'tpa_side_' + tag, (.0006, .012, .010), (side * .00395, -.01525, 0), 'tpa')
    box(tp, tc, 'tpa_face_' + tag, (.0085, .001, .0011), (0, -.02075, side * .00505), 'tpa')
# The service drawing shows a front plate with four openings, not an empty
# perimeter. These central webs reconstruct that plate at the existing cavity
# pitch. Web widths are reduced geometric approximations, not measured CAD.
box(tp, tc, 'tpa_face_row_web', (.0073, .001, .0022), (0, -.02075, 0), 'tpa')
box(tp, tc, 'tpa_face_column_web', (.00045, .001, .0099), (0, -.02075, 0), 'tpa')
box(tp, tc, 'tpa_reinforcement', (.0018, .0014, .00045), (CX, -.0038, CZ + .002675), 'tpa')
tpa = m.slider('tpa', 'Y', 0., .005, tp, tc, mass=.0012, spring=8000., damping=6.2,
               friction=.1, effort=40., armature=1e-6, gravity=True,
               com=(0, -.013, 0), inertia=(8e-8, 2e-8, 8e-8))
HOUSING = m.mechanism('housing', tuple(C), [lance, tpa], LOOK, frame_parts=hp,
                     frame_colliders=hc, free_mass=.00742, friction=(.45, .35),
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
# Catalog mass budget: housing8.180g plus three0.160g cavity plugs;
# frame7.420g + TPA1.200g + lance0.040g =8.660g.
# Terminal0.391g + first30mm wire0.480g =0.871g root.
# Three further30mm wire links0.480g each. Total assembly10.971g.
# Root COM/inertia use a slender-wire/terminal estimate, not manufacturer CAD.
_lead_build = LEAD.build
LEAD = replace(LEAD, build=lambda: _lead_build().replace(
    'float3 physics:diagonalInertia = (',
    'point3f physics:centerOfMass = (0, 0.0154, 0)\n        float3 physics:diagonalInertia = (', 1))
OBJECTS = [HOUSING, LEAD]
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
        if '/RightArm/' in str(p.GetPath()) and p.GetName() in ('tip_left','tip_right') and p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
sr._prepare=_prepare_reports
OBJECTS += [sr.SimReady(name='driver',pos=(.24,.18,TOP+.001),size=(.148,.027,.026),
    mass=.03647007420659065,package='handTool_screwdriver_slotted_4mm_a01',
    usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd',convex=True,scale=1.,yaw=-np.pi/2)]

_DEMO_STATE = None

class Trial:
    def reset(self, env=None):
        global _DEMO_STATE
        self.steps = 0
        self.contacts=None
        _DEMO_STATE = None
    def step(self, env):
        global _DEMO_STATE
        self.steps += 1
        if self.contacts is None:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics
            bodies=[p for p in sim_utils.get_current_stage().Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            sensors=[str(p.GetPath()) for p in bodies if '/driver/' in str(p.GetPath()) or ('/RightArm/' in str(p.GetPath()) and p.GetName() in ('tip_left','tip_right'))]
            filters=[str(p.GetPath()) for p in bodies if '/housing/' in str(p.GetPath())]
            print('CONTACT_PATHS',sensors,filters,flush=True)
            self.contacts=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=4096)
        if self.steps%10==0:
            force=self.contacts.get_contact_force_matrix(dt=1./2400.).numpy()
            robot=env.unwrapped.scene['right_robot']
            j=robot.find_joints('joint7')[0][0]
            print('ROBOT_CONTACTS',self.steps,{'forces':force.tolist(),'right_grip_position':float(m._np(robot.data.joint_pos)[0,j])},flush=True)
        # Scripted demonstration observations only: no object pose writes.
        raw = m._np(env.unwrapped.arena_world.get_pose_w('driver'))[0]
        raw[:3] -= m._np(env.unwrapped.scene.env_origins[0])
        from yam_arena.urdf import quat_xyzw_to_matrix
        tool_R = quat_xyzw_to_matrix(raw[3:])
        tool_tip = raw[:3] + tool_R @ np.array([.10284217447, 0., .01320824353])
        pts = m.fingertips(env)
        approach = (pts[2] - pts[3]) / np.linalg.norm(pts[2] - pts[3])
        tcp = (pts[2] + pts[8]) / 2 - .013 * approach
        actual_q={}
        for arm in ('left','right'):
            robot=env.unwrapped.scene[arm+'_robot']
            ids=[robot.find_joints('joint'+str(j))[0][0] for j in range(1,7)]
            actual_q[arm]=m._np(robot.data.joint_pos)[0,ids].copy()
        _DEMO_STATE = dict(tip=tool_tip, tcp=tcp, tool_R=tool_R, q=actual_q, housing=m.state(env,'housing'))
        if self.steps % 20 == 0:
            print('TOOL_POSE', self.steps, {'tip':tool_tip.tolist(),'tcp':tcp.tolist(),'pose':raw.tolist()}, flush=True)
        h = m.state(env, 'housing')
        m.set_target(env, 'housing', 'tpa', 0. if h.joint('tpa') < .0025 else .005)
        if self.steps == 1 or self.steps % 20 == 0:
            h, w = m.state(env, 'housing'), m.state(env, 'lead')
            print('CONNECTOR_PROBE', self.steps, {'housing': h.pos.tolist(),
                  'terminal_local': h.to_local(w.pos)[0].tolist(), 'housing_quat':h.quat_xyzw.tolist(), 'lead_quat':w.quat_xyzw.tolist(),
                  'lance_deg': h.deg('lance'), 'tpa_mm': h.joint('tpa') * 1000,
                  'wire_joints': w.joints, 'finger_points': m.fingertips(env).tolist()}, flush=True)

t = Trial()
t.reset()
m.on_reset(t.reset)
m.on_step(t.step)
def score(objects): return 0.
def success(objects): return False

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
    label='done'; fraction=1.
    for w in self.waypoints:
        g=np.asarray(w.pos,dtype=float); q=rotation(w)
        if t<w.duration:
            fraction=ik.smoothstep(t/w.duration)
            p,r,f=p+fraction*(g-p),ik.slerp_matrix(r,q,fraction),f+fraction*(w.finger-f)
            label=w.label
            break
        t-=w.duration; p,r,f=g,q,w.finger
    return p,r,f,label

_orientation_demo.ArmPlan.target=target

_original_action7=_orientation_demo.ArmPlan.action7

def pose_action7(self,t):
    p,r,f,label=self.target(t)
    arm='left' if self.base_pos[1]>0 else 'right'
    active=(arm=='left' and label in ('align tip','press retainer','hold retainer','withdraw tool')) or (arm=='right' and label=='hold for retainer press')
    if not active or _DEMO_STATE is None:
        return _original_action7(self,t)
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
        desired_R=np.column_stack((hr[:,1],hr[:,0],-hr[:,2]))
        if not hasattr(self,'_blade_sign'):
            alternatives=[desired_R,desired_R@np.diag([1.,-1.,-1.])]
            self._blade_sign=min(range(2),key=lambda i:np.linalg.norm(ik._rot_error(alternatives[i],current_R)))
        if self._blade_sign:desired_R=desired_R@np.diag([1.,-1.,-1.])
    else:
        if not hasattr(self,'_hold_pose'):
            self._hold_pose=(h.pos.copy(),h.rotation().copy())
        goal,desired_R=self._hold_pose
        point=h.pos
        current_R=h.rotation()
    q_actual=_DEMO_STATE['q'][arm]
    tcp,tcp_R=ik.tcp_pose(q_actual,self.base_pos)
    offset=point-tcp
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
    self.q=np.clip(candidate,ik.LOWER,ik.UPPER)
    self._servo_q=self.q.copy()
    if not hasattr(self,'_servo_log') or t-self._servo_log>.5:
        print('POSE_CONTROL',arm,label,{'position_error_mm':float(np.linalg.norm(goal-point))*1000,'rotation_error_deg':float(np.linalg.norm(ik._rot_error(desired_R,current_R)))*180/np.pi,'q_command':self.q.tolist(),'q_actual':q_actual.tolist(),'retainer_done':getattr(self,'_retainer_done',False)},flush=True)
        self._servo_log=t
    return np.r_[self.q,float(np.clip(f/-.04695,0.,1.))],label

_orientation_demo.ArmPlan.action7=pose_action7

def demo():
    def w(t,p,f,label,a=(0,0,-1),c=(1,0,0)):
        return Oriented(t,p,f,label,0,a,c)
    return {'right':[
       w(1.5,(.30,-.006,TOP+.12),-.022,'hover'),
       w(1.5,(.30,-.006,TOP+.025),-.022,'approach'),
       w(1,(.30,-.006,TOP+.025),-.008,'grip housing'),
       w(2,(.30,-.006,TOP+.12),-.008,'lift housing'),
       w(2,(.30,-.014,TOP+.114),-.008,'turn lead upright',(0,1,0)),
       w(2,(.30,-.014,TOP+.314),0.,'raise connector and grip firmly',(0,1,0)),
       w(30,(.30,-.014,TOP+.314),0.,'hold for retainer press',(0,1,0))],
       'left':[
       w(1.5,(.24,.195,TOP+.12),-.025,'hover handle',c=(-1,0,0)),
       w(1.5,(.24,.195,TOP+.016),-.025,'approach handle',c=(-1,0,0)),
       w(1,(.24,.195,TOP+.016),0.,'grip handle',c=(-1,0,0)),
       w(2,(.24,.195,TOP+.15),0.,'lift driver',c=(-1,0,0)),
       w(2,(.24,.12,TOP+.35),0.,'turn tip up',(0,-1,0),(-1,0,0)),
       w(1.5,(.24,.12,TOP+.17),0.,'lower beside connector',(0,-1,0),(-1,0,0)),
       w(2,(.30,.002,TOP+.175),0.,'approach below connector',(0,-1,0),(-1,0,0)),
       w(4,(.30,.002,TOP+.175),0.,'align tip',(0,-1,0),(-1,0,0)),
       w(2,(.30,.002,TOP+.184),0.,'press retainer',(0,-1,0),(-1,0,0)),
       w(12,(.30,.002,TOP+.184),0.,'hold retainer',(0,-1,0),(-1,0,0)),
       w(4,(.30,.002,TOP+.165),0.,'withdraw tool',(0,-1,0),(-1,0,0))]}
