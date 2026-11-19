"""Native brush pickup, handoff and original outer-bearing alignment diagnostic.

Only robot joint targets are commanded. No object pose, bearing joint state,
attachment or external force is written. This component never awards a score.
"""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_handoff_v3 import *
import probe_handoff_v3 as h
from scipy.spatial.transform import Rotation
from native_pose_reader import NativePoseReader
from native_contact_reader import NativeContactReader
from physics_observer import on_physics_step

# Contact-report instrumentation only; original geometry/materials are retained.
OBJECTS = [h.contact.reporting(x) if x.name == 'cover' else x for x in h.OBJECTS]
STATE = {}
POSES = NativePoseReader()
DURATIONS = dict(stage_brush=4., left_hover=3., bearing_approach=3.,
                 capture=10., align=3., settle=2., release=1.,
                 bearing_withdraw=2., left_clear=3., restore=4., hold=3.)
LEFT_REL = (Rotation.from_euler('x', 90, degrees=True).as_matrix()
            @ Rotation.from_euler('z', 180, degrees=True).as_matrix())
MIN_FINGER = -.0042  # More-closed commands reach the original inner axle.


def ry(angle):
    return Rotation.from_euler('y', angle).as_matrix()


def wrap_half_turn(angle):
    return (angle + np.pi/2) % np.pi - np.pi/2


class BearingContactAudit:
    def __init__(self):
        self.reset()
        on_physics_step(self.sample)
        m.on_step(self.step)
        m.on_reset(self.reset)

    def reset(self, env=None):
        self.reader = NativeContactReader()
        self.ready = False
        self.samples, self.steps = 0, 0
        self.peaks, self.minima, self.last = {}, {}, {}

    def sample(self, env, dt):
        if not self.ready:
            return
        raw = self.reader.read(dt)
        forces = {}
        for pair in raw['pairs']:
            if not ('/LeftArm/' in pair['other'] or '/RightArm/' in pair['other']):
                continue
            body = '/'.join(pair['body'].split('/')[-2:])
            arm = 'LeftArm' if '/LeftArm/' in pair['other'] else 'RightArm'
            link = pair['other'].split('/')[-1]
            normal = max(float(np.linalg.norm(pair['net_normal_world_N'])),
                         float(np.abs(pair['normal_forces_N']).sum()))
            friction = float(np.linalg.norm(np.asarray(
                pair['friction_forces_world_N'], float).reshape(-1, 3), axis=1).sum())
            forces[(body, arm+'/'+link)] = normal + friction
        for key in set(self.peaks) | set(forces):
            self.peaks[key] = max(self.peaks.get(key, 0.), forces.get(key, 0.))
        for tip in ('tip_left', 'tip_right'):
            key = ('roller/outer_bearing', 'LeftArm/'+tip)
            self.minima[key] = min(self.minima.get(key, float('inf')), forces.get(key, 0.))
        self.last = forces
        self.samples += 1

    def step(self, env):
        self.steps += 1
        if not self.ready:
            self.reader.initialize(env)
            self.ready = True
        self.sample(env, env.unwrapped.physics_dt)
        expected = int(env.unwrapped.cfg.decimation) + 1
        if self.steps > 1 and self.samples != expected:
            h.fail(f'incomplete bearing contact interval {self.samples}/{expected}')
        self.latest = dict(peaks=self.peaks.copy(), minima=self.minima.copy(),
                           last=self.last.copy(), readings=self.samples)
        self.peaks, self.minima, self.samples = {}, {}, 0


CONTACT = BearingContactAudit()


def begin(stage, n):
    STATE.update(stage=stage, start=n)
    print('BEARING_ALIGN_STAGE', n, stage, flush=True)


def initialize(env, snapshot, n):
    a = snapshot.assembly
    rqs = m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0, :6]
    lqs = m._np(env.unwrapped.scene['left_robot'].data.joint_pos)[0, :6]
    rp, rr = ik.tcp_pose(rqs, station.BASES['right'])
    lp, lr = ik.tcp_pose(lqs, station.BASES['left'])
    original_p = np.array(a.roller.pos)
    original_r = a.roller.rotation()
    body_r = a.vacuum.rotation()
    phase = float(np.arctan2((body_r.T@original_r)[0, 2],
                            (body_r.T@original_r)[0, 0]))
    frame = (Rotation.from_euler('z', 30, degrees=True).as_matrix()
             @ Rotation.from_euler('x', 20, degrees=True).as_matrix() @ body_r)
    previous_p, previous_r = h.right_goal(2)
    STATE.update(original_p=original_p, original_r=original_r,
                 frame=frame, phase=phase, center=np.array([.30, 0., .96]),
                 grip_p=rr.T@(original_p-rp), grip_r=rr.T@original_r,
                 bias=np.asarray(previous_p)-rp,
                 rotation_bias=ik._rot_error(previous_r, rr),
                 left_start_p=lp, left_start_r=lr,
                 finger=-.016, closing=False, captured=False, stable_steps=0,
                 right_cmd=(previous_p, previous_r, FINGER_CLOSED, 'hold brush'),
                 left_cmd=(lp, lr, FINGER_OPEN, 'left clear'))
    begin('stage_brush', n)
    print('BEARING_ALIGN_INIT', json.dumps(dict(
        phase=phase, frame=frame.tolist(), center=STATE['center'].tolist(),
        grip_p=STATE['grip_p'].tolist(), grip_r=STATE['grip_r'].tolist(),
        original_p=original_p.tolist(), original_r=original_r.tolist(),
        position_bias=STATE['bias'].tolist(),
        rotation_bias_deg=np.degrees(STATE['rotation_bias']).tolist())), flush=True)


def bounded_delta(error, gain, maximum):
    delta = gain*np.asarray(error)
    length = float(np.linalg.norm(delta))
    return delta if length <= maximum else delta*(maximum/length)


def observe(env):
    n = h.control.COUNT
    if n < 820:
        return
    snapshot = POSES.read(env, n/20.)
    a, velocity = snapshot.assembly, snapshot.kinematics
    if not STATE:
        initialize(env, snapshot, n)
    stage = STATE['stage']
    elapsed = (n-STATE['start'])/20.
    f = ik.smoothstep(min(1., elapsed/DURATIONS[stage]))
    frame, center = STATE['frame'], STATE['center']
    roller_goal_r = frame@ry(STATE['phase'])
    goal_p, goal_r = center, roller_goal_r
    if stage == 'stage_brush':
        goal_p = STATE['original_p']+(center-STATE['original_p'])*f
        goal_r = ik.slerp_matrix(STATE['original_r'], roller_goal_r, f)
    elif stage in ('restore', 'hold'):
        fraction = f if stage == 'restore' else 1.
        goal_p = center+(STATE['original_p']-center)*fraction
        goal_r = ik.slerp_matrix(roller_goal_r, STATE['original_r'], fraction)
    rq = m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0, :6]
    rp, rr = ik.tcp_pose(rq, station.BASES['right'])
    grip_p = rr.T@(np.array(a.roller.pos)-rp)
    grip_r = rr.T@a.roller.rotation()
    slip = float(np.linalg.norm(grip_p-STATE['grip_p']))
    turn = float(np.linalg.norm(ik._rot_error(STATE['grip_r'], grip_r)))
    if slip > .008 or turn > np.radians(8.):
        h.fail(f'right brush grasp drift {slip:.6f}m/{np.degrees(turn):.3f}deg')
    error = goal_p-np.asarray(a.roller.pos)
    rot_error = ik._rot_error(goal_r, a.roller.rotation())
    STATE['bias'] = np.clip(STATE['bias']+bounded_delta(error, .08, .0003), -.020, .020)
    rb = STATE['rotation_bias']+bounded_delta(rot_error, .08, np.radians(.15))
    norm = float(np.linalg.norm(rb))
    STATE['rotation_bias'] = rb if norm <= np.radians(6.) else rb*np.radians(6.)/norm
    rcmd = Rotation.from_rotvec(STATE['rotation_bias']).as_matrix()@goal_r@grip_r.T
    pcmd = goal_p-rcmd@grip_p+STATE['bias']
    STATE['right_cmd'] = (pcmd, rcmd, FINGER_CLOSED, 'hold brush '+stage)

    bc = a.outer_bearing.world((0., .096, 0.))
    br = a.outer_bearing.rotation()
    rel = frame.T@br
    phase = wrap_half_turn(float(np.arctan2(rel[0, 2], rel[0, 0])))
    speed = float(np.dot(velocity.outer_bearing.angular_world_rad_s, frame[:, 1]))
    pairs = CONTACT.latest
    peaks = pairs['peaks']
    tip_keys = [('roller/outer_bearing', 'LeftArm/'+tip) for tip in ('tip_left','tip_right')]
    tip_forces = [pairs['last'].get(key, 0.) for key in tip_keys]
    tip_minima = [pairs['minima'].get(key, 0.) for key in tip_keys]
    maximum = sum(peaks.get(key, 0.) for key in tip_keys)
    if maximum > 8.:
        h.fail(f'left bearing contact exceeded8N: {maximum:.3f}')
    for (body, robot), force in peaks.items():
        expected = (body == 'roller/frame' and robot in ('RightArm/tip_left','RightArm/tip_right'))
        expected |= (body == 'roller/outer_bearing' and robot in ('LeftArm/tip_left','LeftArm/tip_right')
                     and stage in ('capture','align','settle','release'))
        if not expected and force > .25:
            h.fail(f'unexpected robot/assembly contact {body}/{robot}: {force:.3f}N')

    lp, lr = STATE['left_start_p'], STATE['left_start_r']
    finger = FINGER_OPEN
    nominal_r = frame@LEFT_REL
    hover = bc+frame@np.array([0., .06, 0.])
    if stage == 'left_hover':
        lp = lp+(hover-lp)*f
        lr = ik.slerp_matrix(lr, nominal_r, f)
        finger = -.016
    elif stage == 'bearing_approach':
        lp = bc+frame@np.array([0., .06+(.012-.06)*f, 0.])
        lr, finger = nominal_r, -.016
    elif stage == 'capture':
        lp, lr = bc+frame@np.array([0., .012, 0.]), nominal_r
        if not STATE['closing']:
            approaching = phase*speed < 0 and abs(phase) <= np.radians(25.)
            quiet = abs(speed) < .1 and abs(phase) <= np.radians(20.)
            if approaching or quiet:
                STATE['closing'] = True
                print('BEARING_CAPTURE_TRIGGER', n, float(np.degrees(phase)), speed, flush=True)
        if STATE['closing']:
            if abs(phase) > np.radians(30.):
                h.fail('bearing left the audited30deg capture window')
            if max(tip_forces) > 2.:
                STATE['finger'] = max(-.016, STATE['finger']-.0002)
            elif min(tip_forces) < .15:
                delta = .0004 if max(tip_forces) > .05 else .0015
                STATE['finger'] = min(MIN_FINGER, STATE['finger']+delta)
            if min(tip_minima) > .10 and abs(speed) < .15:
                STATE['stable_steps'] += 1
            else:
                STATE['stable_steps'] = 0
            if STATE['stable_steps'] >= 4:
                lq = m._np(env.unwrapped.scene['left_robot'].data.joint_pos)[0,:6]
                actual_p, actual_r = ik.tcp_pose(lq, station.BASES['left'])
                STATE.update(captured=True, bearing_grip_p=actual_r.T@(bc-actual_p),
                             bearing_grip_r=actual_r.T@br, capture_r=br,
                             left_position_bias=lp-actual_p,
                             left_rotation_bias=lr@actual_r.T,
                             align_target_r=frame@ry(round(np.arctan2(rel[0,2],rel[0,0])/np.pi)*np.pi))
        finger = STATE['finger']
    elif stage in ('align','settle'):
        desired = (ik.slerp_matrix(STATE['capture_r'], STATE['align_target_r'], f)
                   if stage == 'align' else STATE['align_target_r'])
        measured_tcp_r = desired@STATE['bearing_grip_r'].T
        lp = bc-measured_tcp_r@STATE['bearing_grip_p']+STATE['left_position_bias']
        lr = STATE['left_rotation_bias']@measured_tcp_r
        finger = STATE['finger']
        if max(tip_forces) > 2.:
            STATE['finger'] = max(-.016, finger-.0001)
        elif min(tip_forces) < .15:
            STATE['finger'] = min(MIN_FINGER, finger+.0001)
        finger = STATE['finger']
        if abs(phase) > np.radians(30.):
            h.fail('bearing alignment exceeded audited30deg range')
    elif stage == 'release':
        lp, lr = STATE['release_p'], STATE['release_r']
        finger = STATE['release_finger']+(-.016-STATE['release_finger'])*f
    elif stage == 'bearing_withdraw':
        lp = STATE['release_p']+frame@np.array([0., (.06-.012)*f, 0.])
        lr, finger = STATE['release_r'], -.016
    elif stage == 'left_clear':
        start_p = STATE['release_p']+frame@np.array([0., .048, 0.])
        lp = start_p+(STATE['left_start_p']-start_p)*f
        lr = ik.slerp_matrix(STATE['release_r'], STATE['left_start_r'], f)
        finger = -.016+(FINGER_OPEN+.016)*f
    STATE['left_cmd'] = (lp, lr, finger, 'align original bearing '+stage)

    if n % 10 == 0:
        print('BEARING_ALIGN_STATUS', json.dumps(dict(step=n,stage=stage,elapsed_s=elapsed,
            bearing_phase_deg=float(np.degrees(phase)),bearing_axial_speed_rad_s=speed,
            bearing_center=bc.tolist(),right_grip_slip_m=slip,right_grip_turn_deg=float(np.degrees(turn)),
            roller_error_m=float(np.linalg.norm(error)),roller_rotation_error_deg=float(np.degrees(np.linalg.norm(rot_error))),
            finger_command=finger,left_tip_forces_N=tip_forces,left_tip_minima_N=tip_minima,
            contact_readings=pairs['readings'],right_q=rq.tolist(),
            left_q=m._np(env.unwrapped.scene['left_robot'].data.joint_pos)[0,:6].tolist(),
            right_position_bias_m=STATE['bias'].tolist(),
            right_rotation_bias_deg=np.degrees(STATE['rotation_bias']).tolist())),flush=True)
    advance = elapsed >= DURATIONS[stage]
    if stage == 'capture':
        advance = STATE['captured']
        if elapsed >= DURATIONS[stage] and not advance:
            h.fail('original bearing was not captured within10s')
    if advance:
        if stage in ('stage_brush','restore'):
            # The nominal path ends here; let the bounded feedback hold that
            # endpoint while the loaded arm settles. Preserve the audited
            # 1 mm / 1 degree gate before allowing the other hand to approach.
            within = np.linalg.norm(error) <= .001 and np.linalg.norm(rot_error) <= np.radians(1.)
            STATE['endpoint_stable_steps'] = STATE.get('endpoint_stable_steps', 0)+1 if within else 0
            if STATE['endpoint_stable_steps'] < 4:
                if elapsed >= DURATIONS[stage]+2.:
                    h.fail('held brush did not settle at audited staging/return pose within2s')
                return
            STATE['endpoint_stable_steps'] = 0
        if stage == 'settle' and (abs(phase) > np.radians(1.) or abs(speed) > .05):
            h.fail('bearing did not become flat and stationary before release')
        if stage == 'settle':
            STATE.update(release_p=lp.copy(),release_r=lr.copy(),release_finger=finger)
        if stage == 'hold':
            # Compare to the original chassis frame after ordinary arm restoration.
            relative = a.vacuum.rotation().T@br
            final_phase=wrap_half_turn(float(np.arctan2(relative[0,2],relative[0,0])))
            final_speed=float(np.dot(velocity.outer_bearing.angular_world_rad_s, a.roller.rotation()[:,1]))
            if abs(final_phase)>np.radians(1.) or abs(final_speed)>.05 or maximum>.01:
                h.fail('released outer bearing did not retain alignment')
            if not STATE.get('complete'):
                STATE['complete']=True
                print('BEARING_ALIGNMENT_COMPONENT_COMPLETE', n, float(np.degrees(final_phase)), final_speed, flush=True)
        else:
            names=list(DURATIONS);begin(names[names.index(stage)+1],n)


m.on_step(observe)
previous_target = d.ArmPlan.target


def target(self, time):
    if time >= 41. and STATE:
        return STATE['left_cmd'] if self.base_pos[1] > 0 else STATE['right_cmd']
    return previous_target(self, time)


d.ArmPlan.target = target


@m.on_reset
def reset_alignment(env=None):
    STATE.clear()
    POSES.reset()


def demo():
    plans = h.demo()
    for side in ('left','right'):
        plans[side].append(h.control.point(47.,FINGER_OPEN if side=='left' else FINGER_CLOSED,
            'align original bearing',lambda side=side:ik.tcp_pose(d.READY,station.BASES[side])))
    return plans
