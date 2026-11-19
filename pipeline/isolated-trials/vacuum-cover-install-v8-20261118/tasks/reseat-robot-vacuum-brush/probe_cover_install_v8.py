"""Two-hand cover-installation diagnostic with an initially seated brush; score stays zero.

The cover begins loose. Only the initial brush pose differs from the repair task.
All assembly motion comes from robot commands and physical contacts.
V8 adds observed wrist-orientation and in-plane TCP feedback after V7 drifted
to the rail edge. The8N pressure target,50mm vertical offset cap and actual
15N/hand30N/body guards remain unchanged. Hardware is unchanged.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_cover_pick_v2 import *
import probe_cover_pick_v2 as c

OBJECTS = [replace(x, pos=tuple(C + np.array([AX, 0, AZ])),
                   start_quat=(0., 0., 0., 1.)) if x.name == 'roller' else x
           for x in c.OBJECTS]
INSTALL = {}
FIT_VIEW = None
FIT_FORCE = 0.
DURATIONS = {'above':4., 'entry':3., 'hooks':2., 'press_hover':3.,
             'press_contact':2., 'press_touch':8., 'front':4.,
             'supported_release':1., 'right_clear':2., 'right_press_hover':3.,
             'right_press_contact':2., 'both_press':12., 'press_withdraw':2., 'hold':3.}
FREE_STAGES = {'supported_release', 'right_clear', 'right_press_hover',
               'right_press_contact', 'both_press', 'press_withdraw', 'hold'}
RIGHT_PRESS_REL = Rotation.from_euler('x', 45, degrees=True).as_matrix() @ d.grip_rotation(0)
RIGHT_PRESS_TIP = np.array([.0049495183957574684, .00640989257199871, -.010988232815891471])
LEFT_READY = ik.tcp_pose(d.READY, station.BASES['left'])
PRESS_REL = Rotation.from_euler('y', -45, degrees=True).as_matrix() @ d.grip_rotation(0)
# Lowest original fingertip vertex for this orientation, from the native URDF.
PRESS_TIP = np.array([.0035240914748887954, -.0005313469119748755, -.013457897004229105])



def cover_endpoint(stage):
    angle = 0. if stage == 'front' or stage in FREE_STAGES else 12.
    rot = Rotation.from_euler('y', angle, degrees=True).as_matrix()
    pivot = np.array([AX + .0425, 0, COVER_SEATED_Z - .0025])
    pos = pivot - rot @ np.array([.0425, 0, -.0025])
    if stage in ('above', 'entry'):
        pos += np.array([-.0085, 0, .05 if stage == 'above' else 0])
    return pos, rot


def cover_trajectory(stage, elapsed):
    f = ik.smoothstep(min(1., elapsed / DURATIONS[stage]))
    if stage == 'front':
        rot = Rotation.from_euler('y', 12 * (1-f), degrees=True).as_matrix()
        pivot = np.array([AX + .0425, 0, COVER_SEATED_Z - .0025])
        return pivot - rot @ np.array([.0425, 0, -.0025]), rot
    pos, rot = cover_endpoint(stage)
    if stage in ('above', 'entry', 'hooks'):
        return (INSTALL['from_p'] + (pos-INSTALL['from_p'])*f,
                ik.slerp_matrix(INSTALL['from_r'], rot, f))
    return pos, rot


def begin_install(stage):
    INSTALL.update(stage=stage, start=c.COUNT,
                   from_p=INSTALL['goal_p'].copy(), from_r=INSTALL['goal_r'].copy())
    print('COVER_INSTALL_STAGE', c.COUNT, stage, flush=True)


@on_physics_step
def sample_fit(env, dt):
    global FIT_FORCE
    if FIT_VIEW is not None:
        forces = np.asarray(FIT_VIEW.get_contact_force_matrix(dt=dt).numpy(), float)
        if not np.isfinite(forces).all():
            c.fail('nonfinite cover contact observation')
        FIT_FORCE = max(FIT_FORCE, float(np.linalg.norm(forces, axis=-1).sum()))


def press_feedback(env, cover, stage, elapsed, side, p, r, force):
    """Correct actual wrist angle and tangential TCP position using joint targets.

    Normal pressure remains controlled by the separate original force loop.
    No correction grows without measured contact; withdrawal decays it smoothly.
    """
    q = m._np(env.unwrapped.scene[side+'_robot'].data.joint_pos)[0,:6]
    actual_p, actual_r = ik.tcp_pose(q, station.BASES[side])
    key = side+'_lateral_bias'
    rotation_key = side+'_press_rotation_bias'
    lateral = INSTALL.get(key,np.zeros(3)).copy()
    rotation = INSTALL.get(rotation_key,np.zeros(3)).copy()
    stages = ('press_touch','front','supported_release','right_clear',
              'right_press_hover','right_press_contact','both_press') if side=='left' else ('both_press',)
    if stage in stages and force > .2:
        error = cover.rotation().T@(p-actual_p)
        error[2] = 0.
        delta = .08*error
        length = float(np.linalg.norm(delta))
        if length > .0003:delta *= .0003/length
        lateral += delta
        length = float(np.linalg.norm(lateral))
        if length > .020:lateral *= .020/length
        delta = .08*ik._rot_error(r,actual_r)
        length = float(np.linalg.norm(delta))
        if length > np.radians(.1):delta *= np.radians(.1)/length
        rotation += delta
        length = float(np.linalg.norm(rotation))
        if length > np.radians(12.):rotation *= np.radians(12.)/length
        INSTALL[key],INSTALL[rotation_key] = lateral,rotation
    scale = 1.
    if stage == 'press_withdraw':scale = 1.-ik.smoothstep(min(1.,elapsed/2.))
    elif stage == 'hold':scale = 0.
    INSTALL[side+'_press_tracking_deg'] = float(np.degrees(np.linalg.norm(ik._rot_error(r,actual_r))))
    return p+cover.rotation()@lateral*scale,Rotation.from_rotvec(rotation*scale).as_matrix()@r


def left_press_command(env, cover, stage, elapsed):
    force = sum(x['force_N'] for x in c.LAST_PAIRS
                if x['body'].startswith('cover/') and
                x['robot'] in ('LeftArm/tip_left', 'LeftArm/tip_right'))
    active = stage in ('press_hover', 'press_contact', 'press_touch', 'front',
                       'supported_release', 'right_clear', 'right_press_hover',
                       'right_press_contact', 'both_press', 'press_withdraw', 'hold')
    if not active:
        return (*LEFT_READY, 0., 'left ready'), force
    c.PRESS_ACTIVE = True
    if stage in ('press_touch', 'front', 'supported_release', 'right_clear',
                 'right_press_hover', 'right_press_contact', 'both_press'):
        # Bounded force-feedback command, not an applied external force.
        # Hold5N during the hand transfer, then request8N per fingertip.
        # Actual15N per-hand and30N assembly guards remain in force.
        target = 8. if stage == 'both_press' else 5.
        delta = np.clip(.0001*(target-force), -.00025, .00025)
        INSTALL['press_bias'] = float(np.clip(INSTALL.get('press_bias', 0.)+delta, 0., .050))
    if stage == 'press_hover':
        height = .08
    elif stage == 'press_contact':
        height = .08 + (.001-.08)*ik.smoothstep(min(1.,elapsed/2.))
    elif stage in ('press_touch', 'front', 'supported_release', 'right_clear',
                 'right_press_hover', 'right_press_contact', 'both_press'):
        height = .001-INSTALL.get('press_bias', 0.)
    elif stage == 'press_withdraw':
        start = .001-INSTALL.get('press_bias', 0.)
        height = start+(.08-start)*ik.smoothstep(min(1.,elapsed/2.))
    else:
        height = .08
    r = cover.rotation() @ PRESS_REL
    p = cover.pos + cover.rotation() @ (np.array([-.029,.045,.003+height])-PRESS_TIP)
    if stage == 'press_hover':
        f = ik.smoothstep(min(1.,elapsed/3.))
        p = LEFT_READY[0] + (p-LEFT_READY[0])*f
        r = ik.slerp_matrix(LEFT_READY[1],r,f)
    p,r=press_feedback(env,cover,stage,elapsed,'left',p,r,force)
    return (p,r,0.,'left cover pressure '+stage),force


def right_free_command(env, cover, stage, elapsed):
    if stage == 'supported_release':
        return (INSTALL['release_tcp'], INSTALL['release_r'],
                -.016*ik.smoothstep(min(1.,elapsed)), 'release supported cover')
    if stage == 'right_clear':
        z=.10*ik.smoothstep(min(1.,elapsed/2.))
        finger=-.016+(FINGER_OPEN+.016)*np.clip((z-.04)/.06,0,1)
        return (INSTALL['release_tcp']+np.array([0,0,z]),INSTALL['release_r'],
                float(finger),'clear original end rail')
    force=sum(x['force_N'] for x in c.LAST_PAIRS
              if x['body'].startswith('cover/') and
              x['robot'] in ('RightArm/tip_left','RightArm/tip_right'))
    if stage in ('right_press_contact','both_press','press_withdraw','hold') and force>15.:
        c.fail('right fingertip pressure exceeded15N guard')
    if stage=='both_press':
        delta=np.clip(.0001*(8.-force),-.00025,.00025)
        INSTALL['right_press_bias']=float(np.clip(INSTALL.get('right_press_bias',0.)+delta,0.,.050))
    if stage=='right_press_hover':height=.08
    elif stage=='right_press_contact':height=.08+(.001-.08)*ik.smoothstep(min(1.,elapsed/2.))
    elif stage=='both_press':height=.001-INSTALL.get('right_press_bias',0.)
    elif stage=='press_withdraw':
        start=.001-INSTALL.get('right_press_bias',0.)
        height=start+(.08-start)*ik.smoothstep(min(1.,elapsed/2.))
    else:height=.08
    r=cover.rotation()@RIGHT_PRESS_REL
    p=cover.pos+cover.rotation()@(np.array([-.029,-.045,.003+height])-RIGHT_PRESS_TIP)
    if stage=='right_press_hover':
        f=ik.smoothstep(min(1.,elapsed/3.))
        p0=INSTALL['release_tcp']+np.array([0,0,.10])
        p=p0+(p-p0)*f;r=ik.slerp_matrix(INSTALL['release_r'],r,f)
    p,r=press_feedback(env,cover,stage,elapsed,'right',p,r,force)
    return p,r,0.,'right cover pressure '+stage


def observe_install(env):
    global FIT_VIEW, FIT_FORCE
    if c.COUNT < 320:
        return
    body = m.state(env, 'vacuum')
    cover = m.state(env, 'cover')
    q = m._np(env.unwrapped.scene['right_robot'].data.joint_pos)[0, :6]
    tcp, hand = ik.tcp_pose(q, station.BASES['right'])
    if not INSTALL:
        from isaaclab_physx.physics import PhysxManager
        bodies = [x for x in c.BODIES if '/cover/' in x]
        filters = [x for x in c.BODIES if '/cover/' not in x]
        if len(bodies) != 3 or len(filters) != 4:
            c.fail('incomplete cover/body contact coverage')
        FIT_VIEW = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            bodies, filter_patterns=[filters]*len(bodies), max_contact_data_count=8192)
        gp = hand.T @ (cover.pos-tcp)
        gr = hand.T @ cover.rotation()
        p0, r0 = c.goal(.16, .015)
        lp = body.rotation().T @ (p0+r0@gp-body.pos)
        lr = body.rotation().T @ r0 @ gr
        INSTALL.update(grip_p=gp, grip_r=gr, goal_p=lp, goal_r=lr,
                       bias_p=np.zeros(3), bias_r=np.zeros(3),
                       cmd=(p0, r0, 0., 'begin cover installation'))
        begin_install('above')
    stage = INSTALL['stage']
    sample_fit(env, env.unwrapped.physics_dt)
    force = FIT_FORCE
    FIT_FORCE = 0.
    if force > 30.:
        c.fail('cover/body contact exceeded diagnostic30N guard')
    elapsed = (c.COUNT-INSTALL['start'])/20
    INSTALL['left_cmd'], left_force = left_press_command(env, cover, stage, elapsed)
    lp, lr = cover_trajectory(stage, elapsed)
    INSTALL.update(goal_p=lp, goal_r=lr)
    p, r = body.pos+body.rotation()@lp, body.rotation()@lr
    error = p-cover.pos
    turn = ik._rot_error(r, cover.rotation())
    slip = np.linalg.norm(hand.T@(cover.pos-tcp)-INSTALL['grip_p'])
    grip_turn = np.linalg.norm(ik._rot_error(INSTALL['grip_r'], hand.T@cover.rotation()))
    if stage not in FREE_STAGES:
        if slip > .008 or grip_turn > np.radians(8):
            c.fail('cover moved within grip during installation')
        delta = .08*error
        norm = np.linalg.norm(delta)
        if norm > .0002:
            delta *= .0002/norm
        INSTALL['bias_p'] = np.clip(INSTALL['bias_p']+delta, -.010, .010)
        delta = .08*turn
        norm = np.linalg.norm(delta)
        if norm > np.radians(.1):
            delta *= np.radians(.1)/norm
        bias = INSTALL['bias_r']+delta
        norm = np.linalg.norm(bias)
        if norm > np.radians(4):
            bias *= np.radians(4)/norm
        INSTALL['bias_r'] = bias
    # Compensate measured grasp compliance using robot targets only.
    # Keep the original8mm/8degree drift guard above, and freeze this
    # transform once releasing so the hand never follows the released cover.
    if stage not in FREE_STAGES:
        INSTALL['command_grip_p'] = hand.T @ (cover.pos-tcp)
        INSTALL['command_grip_r'] = hand.T @ cover.rotation()
    if stage in FREE_STAGES:
        INSTALL['cmd'] = right_free_command(env,cover,stage,elapsed)
    else:
        hand_r = Rotation.from_rotvec(INSTALL['bias_r']).as_matrix()@r@INSTALL['command_grip_r'].T
        hand_p = p-hand_r@INSTALL['command_grip_p']+INSTALL['bias_p']
        INSTALL['cmd'] = (hand_p, hand_r, 0., 'cover '+stage)
    left_q=m._np(env.unwrapped.scene['left_robot'].data.joint_pos)[0,:6]
    left_tcp,left_hand=ik.tcp_pose(left_q,station.BASES['left'])
    left_command=INSTALL['left_cmd']
    checks = dict(step=c.COUNT, stage=stage, elapsed_s=elapsed,
                  center_error_m=float(np.linalg.norm(error)),
                  rotation_error_deg=float(np.degrees(np.linalg.norm(turn))),
                  grip_slip_m=float(slip), grip_turn_deg=float(np.degrees(grip_turn)),
                  right_catch_m=float(cover.joint('right_catch')),
                  left_catch_m=float(cover.joint('left_catch')),
                  cover_body_force_N=force, left_press_force_N=float(left_force),
                  right_press_force_N=float(sum(x['force_N'] for x in c.LAST_PAIRS if x['body'].startswith('cover/') and x['robot'] in ('RightArm/tip_left','RightArm/tip_right'))),
                  right_press_bias_m=INSTALL.get('right_press_bias',0.),
                  left_press_bias_m=INSTALL.get('press_bias',0.),
                  left_lateral_bias_m=INSTALL.get('left_lateral_bias',np.zeros(3)).tolist(),
                  right_lateral_bias_m=INSTALL.get('right_lateral_bias',np.zeros(3)).tolist(),
                  left_pressure_rotation_bias_deg=np.degrees(INSTALL.get('left_press_rotation_bias',np.zeros(3))).tolist(),
                  right_pressure_rotation_bias_deg=np.degrees(INSTALL.get('right_press_rotation_bias',np.zeros(3))).tolist(),
                  left_pressure_tracking_deg=INSTALL.get('left_press_tracking_deg'),
                  right_pressure_tracking_deg=INSTALL.get('right_press_tracking_deg'),
                  body_pos=body.pos.tolist(), body_quat=body.quat_xyzw.tolist(),
                  right_q=q.tolist(),left_q=left_q.tolist(),left_tcp=left_tcp.tolist(),
                  left_command_tcp=np.asarray(left_command[0]).tolist(),
                  left_tcp_error_m=float(np.linalg.norm(left_command[0]-left_tcp)),
                  position_bias_m=INSTALL['bias_p'].tolist(),
                  cover_in_body=body.to_local(cover.pos)[0].tolist())
    if c.COUNT % 10 == 0:
        print('COVER_INSTALL_STATUS', json.dumps(checks), flush=True)
    advance = elapsed >= DURATIONS[stage]
    if stage == 'press_touch':
        INSTALL['press_samples'] = INSTALL.get('press_samples',0)+1 if left_force>=2. else 0
        advance = INSTALL['press_samples']>=10
        if elapsed>=DURATIONS[stage] and not advance:
            c.fail('left cover pressure did not establish2N within8s')
    if stage == 'front':
        advance=(abs(checks['right_catch_m'])>.0002 and abs(checks['left_catch_m'])>.0002
                 and force>2. and left_force>2.)
        if elapsed>=DURATIONS['front'] and not advance:
            c.fail('both original catch ramps not contacted before release')
        if advance:
            # Freeze the last held robot command, including gravity/compliance
            # compensation; replacing it with the measured TCP would drop
            # the servo holding offset abruptly during finger opening.
            INSTALL['release_tcp']=np.asarray(INSTALL['cmd'][0]).copy()
            INSTALL['release_r']=np.asarray(INSTALL['cmd'][1]).copy()
            print('COVER_PRE_RELEASE_CONTACT',json.dumps(checks),flush=True)
    if stage=='both_press':
        seated=(checks['center_error_m']<=.001 and checks['rotation_error_deg']<=1.
                and abs(checks['right_catch_m'])<=.0005 and abs(checks['left_catch_m'])<=.0005)
        INSTALL['seated_samples']=INSTALL.get('seated_samples',0)+1 if seated else 0
        advance=INSTALL['seated_samples']>=20
        if elapsed>=DURATIONS['both_press'] and not advance:
            c.fail('dual fingertip press did not seat cover within12s')
    if advance:
        if stage in ('both_press', 'hold'):
            if (checks['center_error_m'] > .001 or checks['rotation_error_deg'] > 1.
                    or abs(checks['right_catch_m']) > .0005
                    or abs(checks['left_catch_m']) > .0005):
                c.fail('cover seating/returned-catch check failed')
            print('COVER_INSTALL_CHECK', json.dumps(checks), flush=True)
        names = list(DURATIONS)
        if stage != names[-1]:
            begin_install(names[names.index(stage)+1])


m.on_step(observe_install)
previous_target = d.ArmPlan.target


def install_target(self, time):
    if self.base_pos[1] > 0 and INSTALL and 'left_cmd' in INSTALL:
        return INSTALL['left_cmd']
    if self.base_pos[1] < 0 and time >= 16. and INSTALL:
        return INSTALL['cmd']
    return previous_target(self, time)


d.ArmPlan.target = install_target


@m.on_reset
def reset_install(env=None):
    global FIT_VIEW, FIT_FORCE
    INSTALL.clear()
    FIT_VIEW = None
    FIT_FORCE = 0.


def demo():
    plans = c.demo()
    plans['right'].append(c.point(53., 0., 'install cover', lambda: c.goal(.16, .015)))
    plans['left'] = [c.point(69., 0., 'support cover pressing', lambda: LEFT_READY)]
    return plans


# Calibrated read-only native link/contact/clearance acquisition; no score writes.
import probe_native_assembly as native_audit
