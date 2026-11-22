"""ring-on-finger trial runner: one seeded scene, one policy, native evidence. Isaac Sim 6.1 only (no CPU fallback).

Policies:
  demo     demo_policy.RingDemo: left wrist camera + joint readings only (full plan)
  hover    the same, stopping with the ring held 4 mm over the fingertip (baseline: held, never released)
  nothing  hold the ready joint targets (baseline: do nothing)

The seed picks the trial's draw (scene.draws: seed k = row k of draws.csv); the bodies are authored at the drawn
poses, so no pose is written during the trial. Actions are clipped to the URDF limits and rate-limited to the joint and
gripper speeds as in peg-out-a-tshirt/run.py. The scorer (scoring.ScoreTracker) reads the ring's and hand form's rigid
poses after every control step and the robot's collision hulls every 10 steps and at the end; nothing it reads
reaches a policy.

Outputs in --out: run.json (status, versions, hashes, audit, settle and diagnostics), score.json (final reading),
trace.jsonl.gz (per step), poses.npz (ring, hand, gripper per step), left_cam.mp4 / top_cam.mp4 / right_cam.mp4,
start-*.png / final-*.png, task-source/ (frozen copy of this folder).
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
EXPLOSION_SPEED = 5.0     # m/s for any skin node or the ring: stop the trial as unstable


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
    """Fail closed on attachments, fixed joints outside the robots, or a kinematic / missing body."""
    from pxr import UsdPhysics
    res = dict(attachments=[], joints_outside_robots=[], bodies={}, physics_scenes=[], skin=None)
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        schemas = [str(s) for s in prim.GetAppliedSchemas()]
        if prim.IsA(UsdPhysics.Scene):
            res['physics_scenes'].append(dict(path=path, gpu=prim.GetAttribute('physxScene:enableGPUDynamics').Get(),
                                              solver=prim.GetAttribute('physxScene:solverType').Get()))
        if 'Attachment' in prim.GetTypeName() or any('Attachment' in s for s in schemas):
            res['attachments'].append(path)
        if prim.IsA(UsdPhysics.Joint) and not any(k in path for k in ('LeftArm', 'RightArm')):
            res['joints_outside_robots'].append(path)
        for name in ('hand', 'ring'):
            if path.endswith('/' + name) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                kin = prim.GetAttribute('physics:kinematicEnabled').Get()
                res['bodies'][name] = dict(path=path, kinematic=bool(kin), colliders=sum(
                    1 for c in prim.GetChildren() if c.HasAPI(UsdPhysics.CollisionAPI)))
        if '/skin' in path and prim.GetTypeName() == 'TetMesh':
            res['skin'] = dict(path=path, schemas=schemas, points=len(prim.GetAttribute('points').Get() or []),
                               tets=len(prim.GetAttribute('tetVertexIndices').Get() or []),
                               rest_offset=prim.GetAttribute('physxCollision:restOffset').Get(),
                               contact_offset=prim.GetAttribute('physxCollision:contactOffset').Get())
    if res['attachments'] or res['joints_outside_robots']:
        raise RuntimeError(f"attachment or joint in an attachment-free scene: {res['attachments'] + res['joints_outside_robots']}")
    for name in ('hand', 'ring'):
        b = res['bodies'].get(name)
        if not b or b['kinematic']:
            raise RuntimeError(f'{name} is missing or kinematic')
    if not res['skin'] or not any('Deformable' in s for s in res['skin']['schemas']):
        raise RuntimeError('the skin is not a volume deformable')
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
    os.environ['ROBOCURVE_RING_SEED'] = str(args.seed)
    os.environ['ROBOCURVE_RING_PHYSICS_HZ'] = str(args.physics_hz)
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
    report = dict(kind=f'ring_on_finger_{args.policy}_trial', status='initializing', score=None, policy=args.policy,
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
        import arena_ring as AR
        import geometry as G
        import native
        import scene
        from scoring import ScoreTracker
        report['draw_mm'] = dict(zip(('plate_dx', 'plate_dy', 'ring_dx', 'ring_dy'), scene.draws(args.seed)))
        report['skin_tet_mesh'] = dict(native.TET_SUMMARY)
        report['skin_material_fields'] = list(native.ACCEPTED_MATERIAL_FIELDS)
        report['physics_dt_s'] = float(base.sim.get_physics_dt())
        report['spawn'] = dict(AR.TRIAL['audit'])
        stage = omni.usd.get_context().get_stage()
        report['native_scene_audit'] = audit(stage)
        AR.TRIAL['skin_nodes'] = native.TET_SUMMARY.get('nodes')
        save()

        env = emb._env
        ready = emb._ready.detach().clone()
        skin_tet = G.skin_tets()
        outer = np.flatnonzero(skin_tet.node_outer)

        def measure():
            rp, rr = AR.body_pose(env, 'ring')
            hp, hr = AR.body_pose(env, 'hand')
            nodes, vel = AR.skin_nodes(env)
            rv, rw = AR.body_velocity(env, 'ring')
            return rp, rr, hp, hr, nodes, vel, rv, rw

        def skin_stats(hp, hr, nodes, vel):
            local = (nodes - hp) @ hr
            rho = np.hypot(local[:, 0], local[:, 1])
            bore = rho[~skin_tet.node_outer] if len(rho) == len(skin_tet.node_outer) else rho
            u = (local[:, 2] - G.WEB_Z) / G.MM
            return dict(nodes=len(nodes), speed_max_m_s=float(np.linalg.norm(vel, axis=1).max()),
                        speed_q99_m_s=float(np.quantile(np.linalg.norm(vel, axis=1), .99)),
                        bore_min_mm=float(bore.min() / G.MM), u_min_mm=float(u.min()), u_max_mm=float(u.max()),
                        pip_outer_r_mm=float(np.max(rho[np.abs(u - 28) < 0.6]) / G.MM) if np.any(np.abs(u - 28) < 0.6) else None)

        # raw reset + 1 s settle at the ready pose (every policy)
        env.reset()
        AR.TRIAL['tracker'] = None
        p0 = measure()
        report['after_reset'] = dict(ring_pos=p0[0], hand_pos=p0[2], skin=skin_stats(p0[2], p0[3], p0[4], p0[5]),
                                     skin_centroid_vs_authored_mm=((p0[4].mean(0) - (np.asarray(scene.hand_origin(args.seed)) +
                                                                                    skin_tet.points.mean(0))) / G.MM))
        for k in range(SETTLE_STEPS):
            env.step(ready)
            if k in (0, 4, 9, 19):
                m = measure()
                report.setdefault('settle', []).append(dict(step=k + 1, ring_pos=m[0], ring_axis=m[1][:, 2], hand_pos=m[2],
                                                            ring_speed=float(np.linalg.norm(m[6])),
                                                            skin=skin_stats(m[2], m[3], m[4], m[5])))
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
            report['policy_description'] = 'scripted: left wrist camera RGB, rig calibration, joint readings only'
        elif args.policy == 'dream':
            from inspect_robots import Scene
            from dream_policy import MODEL, make_policy as make_dream
            dream = make_dream(request_audit, session_audit, rows_per_request=args.dream_rows_per_request)
            dream.bind(emb.info)
            dream.reset(Scene(id='ring-on-finger', instruction=spec['instruction']))
            report['dream_model'] = MODEL
            report['policy_description'] = 'Dream-1: ordinary camera and joint observations and the instruction only'
        else:
            report['policy_description'] = 'constant ready joint targets'
        tracker = ScoreTracker(CONTROL_HZ)
        obs = emb._observation()
        for cam in CAMERAS:
            Image.fromarray(np.asarray(obs.images[cam], np.uint8)).save(args.out / f'start-{cam}.png')
        active = ready.detach().cpu().numpy()[0].copy()
        poses = dict(step=[], ring=[], ring_rot=[], hand=[], gripper=[])
        report['status'] = 'running'
        signal.signal(signal.SIGTERM, terminate)
        save()
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
            active = active + np.clip(np.clip(ref, emb._low, emb._high) - active, -delta, delta)
            env.step(torch.as_tensor(active[None, :], device=base.device, dtype=torch.float32))
            emb._t = step / CONTROL_HZ
            rp, rr, hp, hr, nodes, vel, rv, rw = measure()
            read_robot = (step % 10 == 0 or (policy is not None and policy.done) or step == args.steps
                          or time.monotonic() - t0 > args.wall_limit)
            hulls = AR.robot_hulls(env) if read_robot else None
            result = tracker.update(rp, rr, hp, hr, robot_hulls=hulls, skin_nodes=nodes[outer] if len(nodes) == len(skin_tet.points) else nodes,
                                    read_robot=read_robot)
            obs = emb._observation()
            st = result['state']
            row = dict(step=step, wall_s=round(time.monotonic() - t0, 3), policy_s=round(policy_s, 4), action=active.tolist(), joints=np.asarray(obs.state['joint_pos'], float).tolist(),
                       score=result['score'], explanation=result['explanation'],
                       ring=rp.tolist(), ring_axis=rr[:, 2].tolist(), hand=hp.tolist(),
                       reading={k: st.get(k) for k in st}, phase=policy.log[-1].name if policy is not None and policy.log else None)
            trace.write(json.dumps(encode(row)) + '\n')
            for cam, w in videos.items():
                w.append_data(np.asarray(obs.images[cam], np.uint8))
            gpose = AR._arr(base.scene['left_robot'].data.body_link_pose_w)[0][list(base.scene['left_robot'].body_names).index('gripper')]
            for k, v in (('step', step), ('ring', rp), ('ring_rot', rr), ('hand', hp), ('gripper', gpose)):
                poses[k].append(np.asarray(v, float))
            phase = row['phase']
            if phase and phase not in frames_saved:
                frames_saved.add(phase)
                Image.fromarray(np.asarray(obs.images['left_cam'], np.uint8)).save(args.out / f'phase-{len(frames_saved):02d}-{phase}-left_cam.png')
            fastest = max(float(np.linalg.norm(vel, axis=1).max()) if len(vel) else 0.0, float(np.linalg.norm(rv)))
            if step % 10 == 0:
                report.setdefault('diagnostics', []).append(dict(
                    step=step, phase=phase, score=result['score'], explanation=result['explanation'],
                    ring_offset_mm=st['ring_offset_mm'], ring_low_u_mm=st['ring_low_u_mm'], ring_top_u_mm=st['ring_top_u_mm'],
                    ring_tilt_deg=float(np.degrees(np.arccos(min(1.0, abs(rr[2, 2]))))), robot_min_dist_mm=st['robot_min_dist_mm'],
                    stand_move_max_mm=st['stand_move_max_mm'], ring_speed=float(np.linalg.norm(rv)),
                    skin=skin_stats(hp, hr, nodes, vel), grip=float(np.asarray(obs.state['joint_pos'])[6])))
                report['completed_steps'] = step
                save()
            if not np.isfinite(fastest) or fastest > EXPLOSION_SPEED:
                report.update(status='unstable', unstable_at_step=step)
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
        final = dict(task='ring-on-finger', seed=args.seed, policy=args.policy, steps=step, score=result['score'] if result else 0.0,
                     explanation=result['explanation'] if result else 'no step', conditions=result['conditions'] if result else None,
                     reading=result['state'] if result else None, draw_mm=report.get('draw_mm'))
        report['score'] = final['score']
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
