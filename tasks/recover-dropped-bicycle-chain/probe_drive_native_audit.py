"""Read-only diagnosis of the unchanged parked chain/gear commissioning scene.

Record native drive inertial properties and per-body contact resultants before
the first visible failure. This does not validate a drivetrain or a repair.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_chain_native_audit import *


class DriveAudit:
    def __init__(self):
        self.count = 0
        self.drive = None

    def physics(self, env, dt):
        if self.drive is None:
            import omni.usd
            self.drive = trace.sim.create_articulation_view('/World/envs/env_0/drive')
            paths = list(self.drive.link_paths[0])
            self.paths = paths
            stage = omni.usd.get_context().get_stage()
            masses = m._np(self.drive.get_masses())[0]
            inertias = m._np(self.drive.get_inertias())[0].reshape(-1, 3, 3)
            coms = m._np(self.drive.get_coms())[0]
            rows = []
            for i, path in enumerate(paths):
                prim = stage.GetPrimAtPath(path)
                rows.append(dict(path=path, mass=float(masses[i]),
                    inertia=inertias[i].tolist(), com=coms[i].tolist(),
                    authored_mass=prim.GetAttribute('physics:mass').Get(),
                    authored_inertia=list(prim.GetAttribute('physics:diagonalInertia').Get()),
                    authored_com=list(prim.GetAttribute('physics:centerOfMass').Get())))
            print('DRIVE_NATIVE_PROPERTIES', dict(bodies=rows,
                limits=m._np(self.drive.get_dof_limits()).tolist()), flush=True)
            scenes = []
            for prim in stage.Traverse():
                if prim.GetTypeName() == 'PhysicsScene':
                    scenes.append({a.GetName(): str(a.Get()) for a in prim.GetAttributes()
                        if any(k in a.GetName().lower() for k in ('solver', 'iteration', 'externalforces'))})
            print('DRIVE_NATIVE_SCENES', scenes, flush=True)
        if self.count in (0, 1, 2, 3, 4, 8, 16):
            poses = m._np(trace.view.get_transforms())
            velocities = m._np(trace.view.get_velocities())
            masses = m._np(trace.view.get_masses()).reshape(-1)
            raw = native_audit.contacts.get_contact_data(dt)
            forces, points, normals, separations, counts, starts = [m._np(x) for x in raw]
            force_sums = np.zeros((len(poses), 3))
            torque_sums = np.zeros_like(force_sums)
            absolute_sums = np.zeros(len(poses))
            for i, j in np.argwhere(counts > 0):
                start = int(starts[i, j]); end = start + int(counts[i, j])
                fs = forces[start:end].reshape(-1, 1) * normals[start:end]
                force_sums[i] += fs.sum(0)
                torque_sums[i] += np.cross(points[start:end] - poses[i, :3], fs).sum(0)
                absolute_sums[i] += np.abs(forces[start:end]).sum()
            speed = np.linalg.norm(velocities[:, :3], axis=1)
            bearings = []
            for name, i in trace.indices.items():
                if '_roller_' not in name:
                    continue
                parent, side = name.split('_roller_')
                j = trace.indices[parent]
                rp = m.quat_matrix(poses[j, 3:])
                rr = m.quat_matrix(poses[i, 3:])
                anchor = poses[j, :3] + rp @ np.array([-.00635 if side == 'a' else .00635, 0., 0.])
                bearings.append(dict(name=name, anchor_error_m=float(np.linalg.norm(poses[i, :3]-anchor)),
                    axis_error_deg=float(np.degrees(np.arccos(np.clip(rp[:, 2]@rr[:, 2], -1., 1.))))))
            bearings.sort(key=lambda r: r['anchor_error_m'], reverse=True)
            order = np.argsort(speed)[-12:][::-1]
            rows = [dict(name=trace.view.prim_paths[i].rsplit('/', 1)[-1],
                mass=float(masses[i]), pose=poses[i].tolist(), velocity=velocities[i].tolist(),
                normal_resultant_n=force_sums[i].tolist(),
                normal_moment_nm=torque_sums[i].tolist(),
                normal_absolute_sum_n=float(absolute_sums[i])) for i in order]
            print('DRIVE_CONTACT_GROWTH', self.count, dict(time_s=self.count*dt,
                chain_fastest=rows,
                worst_roller_bearings=bearings[:8],
                drive_paths=self.paths,
                drive_poses=m._np(self.drive.get_link_transforms())[0].tolist(),
                drive_velocities=m._np(self.drive.get_link_velocities())[0].tolist()), flush=True)
        self.count += 1


drive_audit = DriveAudit()
on_physics_step(drive_audit.physics)
