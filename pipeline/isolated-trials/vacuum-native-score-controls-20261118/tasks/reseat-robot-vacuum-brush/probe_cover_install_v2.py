"""Cover-installation diagnostic with an initially seated brush; score stays zero.

The cover begins loose. Only the initial brush pose differs from the repair task.
All assembly motion comes from robot commands and physical contacts.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_cover_pick_v1 import *
import probe_cover_pick_v1 as c

OBJECTS = [replace(x, pos=tuple(C + np.array([AX, 0, AZ])),
                   start_quat=(0., 0., 0., 1.)) if x.name == 'roller' else x
           for x in c.OBJECTS]
INSTALL = {}
FIT_VIEW = None
FIT_FORCE = 0.
DURATIONS = {'above': 4., 'entry': 3., 'hooks': 2., 'front': 4.,
             'settle': 3., 'release': 1., 'withdraw': 2., 'hold': 3.}


def cover_endpoint(stage):
    angle = 0. if stage in ('front', 'settle', 'release', 'withdraw', 'hold') else 12.
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
    lp, lr = cover_trajectory(stage, elapsed)
    INSTALL.update(goal_p=lp, goal_r=lr)
    p, r = body.pos+body.rotation()@lp, body.rotation()@lr
    error = p-cover.pos
    turn = ik._rot_error(r, cover.rotation())
    slip = np.linalg.norm(hand.T@(cover.pos-tcp)-INSTALL['grip_p'])
    grip_turn = np.linalg.norm(ik._rot_error(INSTALL['grip_r'], hand.T@cover.rotation()))
    if stage not in ('release', 'withdraw', 'hold'):
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
    hand_r = Rotation.from_rotvec(INSTALL['bias_r']).as_matrix()@r@INSTALL['grip_r'].T
    hand_p = p-hand_r@INSTALL['grip_p']+INSTALL['bias_p']
    finger = 0.
    if stage == 'release':
        finger = -.016*ik.smoothstep(min(1., elapsed))
    if stage in ('withdraw', 'hold'):
        lift = .10*(ik.smoothstep(min(1., elapsed/2)) if stage == 'withdraw' else 1.)
        hand_p += np.array([0, 0, lift])
        finger = -.016+(FINGER_OPEN+.016)*np.clip((lift-.04)/.06, 0, 1)
    INSTALL['cmd'] = (hand_p, hand_r, float(finger), 'cover '+stage)
    checks = dict(step=c.COUNT, stage=stage, elapsed_s=elapsed,
                  center_error_m=float(np.linalg.norm(error)),
                  rotation_error_deg=float(np.degrees(np.linalg.norm(turn))),
                  grip_slip_m=float(slip), grip_turn_deg=float(np.degrees(grip_turn)),
                  right_catch_m=float(cover.joint('right_catch')),
                  left_catch_m=float(cover.joint('left_catch')),
                  cover_body_force_N=force,
                  body_pos=body.pos.tolist(), body_quat=body.quat_xyzw.tolist(),
                  right_q=q.tolist(),
                  position_bias_m=INSTALL['bias_p'].tolist(),
                  cover_in_body=body.to_local(cover.pos)[0].tolist())
    if c.COUNT % 10 == 0:
        print('COVER_INSTALL_STATUS', json.dumps(checks), flush=True)
    if elapsed >= DURATIONS[stage]:
        if stage in ('settle', 'hold'):
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
    plans['right'].append(c.point(26., 0., 'install cover', lambda: c.goal(.16, .015)))
    return plans
