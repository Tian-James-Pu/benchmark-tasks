"""Score-zero, read-only native pose/contact calibration; no robot motion."""
import sys, json
from pathlib import Path
from dataclasses import replace, asdict
sys.path.insert(0, str(Path(__file__).resolve().parent))
from task import *
import mechkit as m
from physics_observer import on_physics_step
from native_pose_reader import NativePoseReader
from native_contact_reader import NativeContactReader
from native_clearance_reader import NativeClearanceReader


def reporting(part):
    original = part.build
    def build():
        from pxr import Sdf, Usd, UsdPhysics, PhysxSchema
        layer = Sdf.Layer.CreateAnonymous('.usda')
        layer.ImportFromString(original())
        stage = Usd.Stage.Open(layer)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        return stage.GetRootLayer().ExportToString()
    return replace(part, build=build)


OBJECTS = [reporting(p) if p.name in ('vacuum', 'roller', 'cover') else p for p in OBJECTS]


class NativeAssemblyAudit:
    def __init__(self):
        self.reset()
        m.on_reset(self.reset)
        m.on_step(self.step)
        on_physics_step(self.capture)

    def reset(self, env=None):
        self.pose_reader = NativePoseReader()
        self.contact_reader = NativeContactReader()
        self.clearance_reader = NativeClearanceReader()
        self.rows, self.steps = [], 0
        self.last_frame, self.last_pose = None, None
        self.ready = False
        if env is not None:
            self.initialize(env)

    def initialize(self, env):
        self.contact_reader.initialize(env)
        self.clearance_reader.initialize(env)
        self.ready = True
        print('NATIVE_ASSEMBLY_PATHS', json.dumps(dict(
            bodies=self.contact_reader.bodies, filters=self.contact_reader.filters,
            robot_columns=self.contact_reader.robot_columns,
            physics_dt=float(env.unwrapped.physics_dt),
            decimation=int(env.unwrapped.cfg.decimation))), flush=True)

    def sample(self, env, dt, boundary):
        if not self.ready:
            return
        base = env.unwrapped
        # ManagerBasedEnv increments the counter before apply_action, where
        # the hook observes the previous completed frame. At on_step the
        # counter denotes the newly completed frame. Shared interval endpoints
        # reuse the same pose sample rather than fabricate a later timestamp.
        frame = int(base._sim_step_counter) - (0 if boundary else 1)
        if self.last_frame is not None and frame < self.last_frame:
            raise RuntimeError('native frame counter went backwards')
        if frame != self.last_frame:
            self.last_pose = self.pose_reader.read(env, frame*dt)
            self.last_frame = frame
        contacts = self.contact_reader.read(dt)
        clearance = self.clearance_reader.read(env, self.last_pose)
        self.rows.append(dict(frame=frame, boundary=boundary,
                              snapshot=asdict(self.last_pose), contacts=contacts,
                              clearance=clearance))

    def capture(self, env, dt):
        self.sample(env, dt, False)

    def step(self, env):
        self.steps += 1
        if not self.ready:
            # Initial incomplete observations are explicitly not scored.
            self.initialize(env)
        self.sample(env, env.unwrapped.physics_dt, True)
        expected = int(env.unwrapped.cfg.decimation)+1
        frames = [row['frame'] for row in self.rows]
        complete = (len(frames) == expected
                    and all(b == a+1 for a, b in zip(frames, frames[1:])))
        if self.steps > 1 and not complete:
            raise RuntimeError(f'incomplete native interval: {frames}')
        print('NATIVE_ASSEMBLY_INTERVAL', json.dumps(dict(
            step=self.steps, complete=complete, readings=len(self.rows),
            expected=expected, rows=self.rows)), flush=True)
        self.rows.clear()


AUDIT = NativeAssemblyAudit()


def demo():
    # Same empty plan as the existing native settling probes; runner max_steps
    # bounds the diagnostic while both arms remain at their default READY pose.
    return {}
