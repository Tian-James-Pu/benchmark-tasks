"""food-magnets trial runner: one seeded scene, one policy, native evidence. Isaac Sim 6.1 only (no CPU fallback).

Policies:
  demo     demo_policy.MagnetDemo: top and wrist cameras + joint readings only (full plan)
  hover    the same, holding the last magnet over the open box (baseline: never released, lid open)
  nothing  hold the ready joint targets (baseline: do nothing)
  dream    Dream-1 (dreamlabs/dream-1-beta via inspect_robots_dream), cameras + joints + the instruction

The seed picks the trial's draw (scene.draw); the bodies are authored at the drawn poses, so no pose is written during
the trial. Actions are clipped to the URDF limits and rate-limited to the joint and gripper speeds as in
peg-out-a-tshirt/run.py. The scorer (scoring.ScoreTracker) reads the bodies' rigid poses after every control step and
the grippers' collision hulls every step; nothing it reads reaches a policy. At the end (card, Scoring) Released is read
over the last 1 s as the policy left the grippers; then the arms are jogged to ready (2 s, grippers unchanged) and In,
Lid shut and Kept are read from that state.

Outputs in --out: run.json, score.json, trace.jsonl.gz, poses.npz, top_cam.mp4 / left_cam.mp4 / right_cam.mp4,
start-*.png / final-*.png / phase-*.png, task-source/.
"""
import argparse
import gzip
import hashlib
import json
import os
import shutil
import signal
import time
from datetime import datetime, timezone
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CONTROL_HZ = 20
MAX_STEPS = 2400          # card run limit
SETTLE_STEPS = 20         # 1 s at the ready pose after the raw reset, every policy
CAMERAS = ('top_cam', 'left_cam', 'right_cam')
EXPLOSION_SPEED = 5.0     # m/s for any body: stop the trial as unstable


def sources():
    out = {}
    for p in sorted(HERE.iterdir()):
        if p.suffix in ('.py', '.json', '.md', '.csv', '.sbatch'):
            if p.suffix == '.py':
                compile(p.read_bytes(), str(p), 'exec')
            elif p.suffix == '.json':
                json.loads(p.read_text())
            out[p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    return out


def package_versions():
    v = {}
    for name in ('isaacsim', 'isaaclab', 'isaaclab_arena', 'yam-arena', 'inspect-robots', 'inspect-robots-yam', 'numpy'):
        try:
            v[name] = version(name)
        except PackageNotFoundError:
            v[name] = None
    return v


def encode(x):
    if isinstance(x, np.ndarray):
        return encode(x.tolist())
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return encode(x.item())
    if isinstance(x, dict):
        return {str(k): encode(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [encode(v) for v in x]
    if isinstance(x, float) and not np.isfinite(x):
        return None
    return x


def audit(stage):
    """Fail closed on attachments, joints outside the robots other than the lid hinge, or a kinematic / missing body."""
    from pxr import UsdPhysics
    res = dict(attachments=[], joints_outside_robots=[], hinge=None, bodies={}, static_fixtures=[], physics_scenes=[])
    names = ('strawberry', 'cookie', 'donut', 'plain', 'card', 'lid')
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        schemas = [str(s) for s in prim.GetAppliedSchemas()]
        if prim.IsA(UsdPhysics.Scene):
            res['physics_scenes'].append(dict(path=path, gpu=prim.GetAttribute('physxScene:enableGPUDynamics').Get(),
                                              solver=prim.GetAttribute('physxScene:solverType').Get()))
        if 'Attachment' in prim.GetTypeName() or any('Attachment' in s for s in schemas):
            res['attachments'].append(path)
        if prim.IsA(UsdPhysics.Joint) and not any(k in path for k in ('LeftArm', 'RightArm', 'left_robot', 'right_robot')):
            if path.endswith('/lid_hinge') and prim.IsA(UsdPhysics.RevoluteJoint):
                res['hinge'] = dict(path=path, lower=prim.GetAttribute('physics:lowerLimit').Get(),
                                    upper=prim.GetAttribute('physics:upperLimit').Get())
            else:
                res['joints_outside_robots'].append(path)
        for name in names:
            if path.endswith('/' + name) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                kin = prim.GetAttribute('physics:kinematicEnabled').Get()
                res['bodies'][name] = dict(path=path, kinematic=bool(kin), mass=prim.GetAttribute('physics:mass').Get())
        if ('/Board/' in path or '/BoxBody/' in path) and prim.HasAPI(UsdPhysics.CollisionAPI):
            res['static_fixtures'].append(dict(path=path, rigid=False))
    if res['attachments'] or res['joints_outside_robots']:
        raise RuntimeError(f"attachment or joint in an attachment-free scene: {res['attachments'] + res['joints_outside_robots']}")
    for name in names:
        b = res['bodies'].get(name)
        if not b or b['kinematic']:
            raise RuntimeError(f'{name} is missing or kinematic')
    if res['hinge'] is None:
        raise RuntimeError('the lid hinge is missing')
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--policy', choices=('demo', 'hover', 'nothing', 'dream'), default='nothing')
    ap.add_argument('--wall-limit', type=float, default=2150.0, help='stop stepping after this many seconds (still scores)')
    ap.add_argument('--dream-rows-per-request', type=int, choices=(1, 2, 4, 8), default=2)
    ap.add_argument('--steps', type=int, default=None, help=f'control steps (nothing: 60; else up to {MAX_STEPS})')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--physics-hz', type=int, choices=(240, 480), default=240)
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--check-only', action='store_true')
    args = ap.parse_args()
    if args.steps is None:
        args.steps = 60 if args.policy == 'nothing' else MAX_STEPS
    if not 1 <= args.steps <= MAX_STEPS:
        ap.error(f'--steps must be 1..{MAX_STEPS}')
    hashes = sources()
    if args.check_only:
        print(json.dumps(dict(status='local_syntax_and_json_passed', source_sha256=hashes), indent=2))
        return
    if args.out is None:
        ap.error('--out is required')
    os.environ['ROBOCURVE_FM_SEED'] = str(args.seed)
    os.environ['ROBOCURVE_FM_PHYSICS_HZ'] = str(args.physics_hz)
    versions = package_versions()
    if not (versions.get('isaacsim') or '').startswith('6.1.'):
        raise RuntimeError('use the pinned Isaac Sim 6.1 environment (ENVIRONMENT.json)')
    args.out.mkdir(parents=True, exist_ok=False)
    frozen = args.out / 'task-source'
    frozen.mkdir()
    for name in hashes:
        shutil.copyfile(HERE / name, frozen / name)
    spec = json.loads((HERE / 'task.json').read_text())
    t0 = time.monotonic()
    report = dict(kind=f'food_magnets_{args.policy}_trial', status='initializing', score=None, policy=args.policy,
                  seed=args.seed, started_utc=datetime.now(timezone.utc).isoformat(), source_sha256=hashes,
                  versions=versions, control_hz=CONTROL_HZ, physics_hz=args.physics_hz, requested_steps=args.steps,
                  completed_steps=0, instruction=spec['instruction'])

    def save():
        report['wall_seconds'] = time.monotonic() - t0
        tmp = args.out / 'run.json.tmp'
        tmp.write_text(json.dumps(encode(report), indent=2) + '\n')
        tmp.replace(args.out / 'run.json')

    save()
    app = trace = dream = None
    request_audit, session_audit = [], {}
    videos = {}

    def terminate(signum, frame):
        raise SystemExit('SIGTERM: trial interrupted')

    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
        from isaaclab.app import AppLauncher
        app = AppLauncher(headless=True, enable_cameras=True).app
        import imageio.v2 as imageio
        import omni.usd
        import torch
        from PIL import Image

        from yam_arena import inspect_embodiment as wrapper
        from yam_arena import scene as yscene
        yscene.PHYSICS_DT = 1.0 / args.physics_hz
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)
        wrapper._APP = app
        emb = wrapper.YamArenaEmbodiment(task=str(HERE / 'task.py'), object_jitter=0.0, control_hz=CONTROL_HZ,
                                         cam_width=320, cam_height=176)
        emb._ensure_sim(None)
        base = emb._base
        emb._instruction = spec['instruction']
        report['build_wall_seconds'] = time.monotonic() - t0
        import arena_fm as AR
        import geometry as G
        from scoring import ScoreTracker
        draw = AR.TRIAL['draw']
        report['draw'] = draw
        report['physics_dt_s'] = float(base.sim.get_physics_dt())
        report['decimation'] = int(getattr(base.cfg, 'decimation', 0))
        report['physics_handles_decimation'] = bool(getattr(base, '_physics_handles_decimation', False))
        report['spawn'] = dict(AR.TRIAL['audit'])
        stage = omni.usd.get_context().get_stage()
        report['native_scene_audit'] = audit(stage)
        magnets = AR.install_magnets(env := emb._env)
        save()

        ready = emb._ready.detach().clone()
        NAMES = ('strawberry', 'cookie', 'donut', 'plain', 'card', 'lid')

        def measure():
            return {n: AR.body_pose(env, n) for n in NAMES}

        def speeds():
            return {n: float(np.linalg.norm(AR.body_velocity(env, n)[0])) for n in NAMES}

        def summary(p):
            out = {}
            for n in ('strawberry', 'cookie', 'donut', 'plain'):
                g, nx, _ = G.magnet_gap(n, *p[n])
                out[n] = dict(pos_mm=(p[n][0] / G.MM).round(2), gap_mm=round(g / G.MM, 3), facing=round(nx, 4))
            out['card'] = dict(pos_mm=(p['card'][0] / G.MM).round(2))
            out['lid_free_edge_above_rim_mm'] = round(float((G.lid_free_edge(*p['lid'])[2] - G.TABLE_TOP - G.BOX_H) / G.MM), 2)
            return out

        # raw reset + 1 s settle at the ready pose (every policy)
        env.reset()
        AR.TRIAL['tracker'] = None
        p_auth = measure()
        report['after_reset'] = summary(p_auth)
        calls0 = magnets.calls
        for k in range(SETTLE_STEPS):
            env.step(ready)
            if k in (0, 4, 9, 19):
                report.setdefault('settle', []).append(dict(step=k + 1, bodies=summary(measure()), speeds=speeds(),
                                                            force=dict(magnets.last)))
        report['magnet_force_writes_per_control_step'] = (magnets.calls - calls0) / SETTLE_STEPS / 4
        for _ in range(4):
            base.sim.render()
        emb._t = 0.0
        save()

        if not args.no_video:
            for cam in CAMERAS:
                videos[cam] = imageio.get_writer(args.out / f'{cam}.mp4', fps=CONTROL_HZ, codec='libx264', pixelformat='yuv420p',
                                                 ffmpeg_params=['-movflags', '+faststart'], macro_block_size=1)
        trace = gzip.open(args.out / 'trace.jsonl.gz', 'wt', compresslevel=1)
        from yam_arena.constants import FINGER_OPEN, GRIPPER, SHOULDER, WRIST
        delta = np.tile(np.array([SHOULDER['velocity'] / CONTROL_HZ] * 3 + [WRIST['velocity'] / CONTROL_HZ] * 3 +
                                 [GRIPPER['velocity'] / abs(FINGER_OPEN) / CONTROL_HZ]), 2)
        policy = dream = None
        request_audit, session_audit, queued = [], {'session_count': 0}, []
        if args.policy in ('demo', 'hover'):
            from demo_policy import make_policy
            policy = make_policy(320, 176, CONTROL_HZ, mode=args.policy)
            report['policy_description'] = 'scripted: top and wrist camera RGB, rig calibration, joint readings only'
        elif args.policy == 'dream':
            from inspect_robots import Scene
            from dream_policy import MODEL, make_policy as make_dream
            dream = make_dream(request_audit, session_audit, rows_per_request=args.dream_rows_per_request)
            dream.bind(emb.info)
            dream.reset(Scene(id='food-magnets', instruction=spec['instruction']))
            report['dream_model'] = MODEL
            report['policy_description'] = 'Dream-1: ordinary camera and joint observations and the instruction only'
        else:
            report['policy_description'] = 'constant ready joint targets'
        tracker = ScoreTracker(draw, CONTROL_HZ)
        obs = emb._observation()
        for cam in CAMERAS:
            Image.fromarray(np.asarray(obs.images[cam], np.uint8)).save(args.out / f'start-{cam}.png')
        active = ready.detach().cpu().numpy()[0].copy()
        poses = dict(step=[], **{n: [] for n in NAMES}, **{n + '_rot': [] for n in NAMES})
        report['status'] = 'running'
        save()

        def control(target):
            nonlocal active
            active = active + np.clip(np.clip(target, emb._low, emb._high) - active, -delta, delta)
            env.step(torch.as_tensor(active[None, :], device=base.device, dtype=torch.float32))

        result, step, frames_saved = None, 0, set()
        for step in range(1, args.steps + 1):
            tp = time.monotonic()
            if policy is not None:
                ref = np.asarray(policy.act(obs), float)
            elif dream is not None:
                if not queued:
                    queued.extend(np.asarray(a.data, float) for a in dream.act(obs).actions)
                ref = queued.pop(0)
            else:
                ref = ready.detach().cpu().numpy()[0]
            policy_s = time.monotonic() - tp
            if ref.shape != (14,) or not np.isfinite(ref).all():
                raise ValueError('invalid 14-D joint action')
            control(ref)
            emb._t = step / CONTROL_HZ
            p = measure()
            hulls = AR.robot_hulls(env)
            result = tracker.update(p, hulls)
            obs = emb._observation()
            st = result['state']
            phase = policy.log[-1].name if policy is not None and policy.log else None
            row = dict(step=step, wall_s=round(time.monotonic() - t0, 3), policy_s=round(policy_s, 4), action=active.tolist(),
                       joints=np.asarray(obs.state['joint_pos'], float).tolist(), score=result['score'],
                       explanation=result['explanation'], bodies={n: p[n][0].tolist() for n in NAMES}, reading=st,
                       force=dict(magnets.last), phase=phase)
            trace.write(json.dumps(encode(row)) + '\n')
            for cam, w in videos.items():
                w.append_data(np.asarray(obs.images[cam], np.uint8))
            poses['step'].append(step)
            for n in NAMES:
                poses[n].append(p[n][0])
                poses[n + '_rot'].append(p[n][1])
            if phase and phase not in frames_saved:
                frames_saved.add(phase)
                for cam in CAMERAS:
                    Image.fromarray(np.asarray(obs.images[cam], np.uint8)).save(args.out / f'phase-{len(frames_saved):02d}-{phase}-{cam}.png')
            sp = speeds()
            if step % 10 == 0:
                report.setdefault('diagnostics', []).append(dict(
                    step=step, phase=phase, score=result['score'], explanation=result['explanation'], bodies=summary(p),
                    speeds=sp, grip=[float(np.asarray(obs.state['joint_pos'])[i]) for i in (6, 13)],
                    clear=st['food'][0].get('grip_clear')))
                report['completed_steps'] = step
                save()
            if not np.isfinite(max(sp.values())) or max(sp.values()) > EXPLOSION_SPEED:
                report.update(status='unstable', unstable_at_step=step, speeds=sp)
                break
            if policy is not None and policy.done:
                report['ended'] = 'policy finished its final hold'
                break
            if time.monotonic() - t0 > args.wall_limit:
                report['ended'] = f'wall-clock limit {args.wall_limit:.0f} s at step {step}'
                break
        report['completed_steps'] = step
        if report['status'] == 'running':
            report['status'] = 'complete'
        for cam in CAMERAS:
            Image.fromarray(np.asarray(obs.images[cam], np.uint8)).save(args.out / f'final-{cam}.png')
        if policy is not None:
            report['demo'] = policy.report()
        # Released: the last 1 s, grippers as the policy left them (read above, every step)
        released_state = tracker.state(measure())
        clear = {k: v for k, v in (
            [(f'food{i}', f.get('grip_clear')) for i, f in enumerate(released_state['food'])] +
            [('plain', released_state['plain'][0].get('grip_clear')), ('card', released_state['cards'][0].get('grip_clear')),
             ('box', released_state['box'].get('grip_clear')), ('lid', released_state['box'].get('lid_grip_clear'))])}
        report['released_clearances_mm'] = clear
        # post-run jog to ready (2 s; grippers unchanged), then the reading for In, Lid shut and Kept
        jog = ready.detach().cpu().numpy()[0].copy()
        jog[6], jog[13] = active[6], active[13]
        for _ in range(2 * CONTROL_HZ):
            control(jog)
        final_state = tracker.state(measure())
        for i, f in enumerate(final_state['food']):
            f['grip_clear'] = released_state['food'][i].get('grip_clear')
        final_state['plain'][0]['grip_clear'] = released_state['plain'][0].get('grip_clear')
        final_state['cards'][0]['grip_clear'] = released_state['cards'][0].get('grip_clear')
        final_state['box']['grip_clear'] = released_state['box'].get('grip_clear')
        final_state['box']['lid_grip_clear'] = released_state['box'].get('lid_grip_clear')
        import scoring
        final_score = scoring.score(final_state)
        for cam in CAMERAS:
            obs = emb._observation()
            Image.fromarray(np.asarray(obs.images[cam], np.uint8)).save(args.out / f'after-jog-{cam}.png')
        final = dict(task='food-magnets', seed=args.seed, policy=args.policy, steps=step, score=final_score,
                     explanation=scoring.explain(final_state), state=final_state, draw=draw,
                     score_before_jog=result['score'] if result else None)
        report['score'] = final_score
        np.savez_compressed(args.out / 'poses.npz', **{k: np.asarray(v) for k, v in poses.items()})
        final['poses_sha256'] = hashlib.sha256((args.out / 'poses.npz').read_bytes()).hexdigest()
        (args.out / 'score.json').write_text(json.dumps(encode(final), indent=2) + '\n')
    except BaseException as exc:
        import traceback
        report.update(status='interrupted' if isinstance(exc, SystemExit) else 'failed', error_type=type(exc).__name__,
                      error_message=str(exc), traceback=traceback.format_exc()[-4000:])
        raise
    finally:
        if dream is not None:
            try:
                from dream_policy import public_transcript
                (args.out / 'dream-transcript.json').write_text(json.dumps(encode(public_transcript(dream)), indent=2) + '\n')
            except Exception as error:  # noqa: BLE001
                report['dream_transcript_error'] = repr(error)
            report.update(dream_requests=len(request_audit), dream_session_audit=session_audit)
        save()
        if trace is not None:
            trace.close()
        for w in videos.values():
            w.close()
        save()
        if app is not None:
            app.close()
        signal.signal(signal.SIGTERM, previous)


if __name__ == '__main__':
    main()
