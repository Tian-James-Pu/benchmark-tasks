"""Read-only PhysX pose/twist adapter draft; not installed in the task.

No support, release, contact, integrity or score evidence is inferred here.
The native reset/frame-convention check must pass before using these records.
"""
from dataclasses import dataclass
import numpy as np
from assembly_score import Assembly, BodyVelocity, Kinematics, Pose


LINKS = {
    'vacuum': ('vacuum', 'frame'),
    'roller': ('roller', 'frame'),
    'cover': ('cover', 'frame'),
    'motor_socket': ('vacuum', 'motor_socket'),
    'outer_bearing': ('roller', 'outer_bearing'),
    'right_catch': ('cover', 'right_catch'),
    'left_catch': ('cover', 'left_catch'),
}


def array(value, *, finite=True):
    """Copy now: simulator buffers may be reused on the next API call."""
    if hasattr(value, 'torch'):
        value = value.torch
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    elif hasattr(value, 'numpy'):
        value = value.numpy()
    result = np.asarray(value, dtype=float).copy()
    if finite and not np.isfinite(result).all():
        raise ValueError('nonfinite native body state')
    return result


@dataclass(frozen=True)
class Snapshot:
    time_s: float
    assembly: Assembly
    kinematics: Kinematics
    origin_world_m: tuple
    velocity_transport_max_error_m_s: float


class NativePoseReader:
    """Read all seven measured links using the explicit PhysX xyzw contract.

    IsaacLab PhysX articulation_data.py documents body_link_pose_w and
    body_com_pose_w as xyzw. The similarly named OV API has a different
    convention; refuse it instead of guessing a conversion from values.
    """
    def __init__(self):
        self.last_time = None

    def reset(self):
        self.last_time = None

    def read(self, env, time_s):
        time_s = float(time_s)
        if not np.isfinite(time_s) or time_s < 0:
            raise ValueError('invalid native observation time')
        if self.last_time is not None and time_s <= self.last_time:
            raise ValueError('native snapshot time must strictly increase')
        scene = env.unwrapped.scene
        origin = array(scene.env_origins)[0]
        if origin.shape != (3,):
            raise ValueError('unexpected environment-origin shape')
        assets = {}
        for asset_name in ('vacuum', 'roller', 'cover'):
            asset = scene[asset_name]
            if not type(asset.data).__module__.startswith('isaaclab_physx.'):
                raise ValueError('this adapter requires the verified PhysX xyzw API')
            names = list(asset.body_names)
            expected = [link for obj, link in LINKS.values() if obj == asset_name]
            if len(names) != len(set(names)) or set(names) != set(expected):
                raise ValueError(f'unexpected native links for {asset_name}: {names}')
            link_pose = array(asset.data.body_link_pose_w)
            com_pose = array(asset.data.body_com_pose_w)
            com_velocity = array(asset.data.body_com_vel_w)
            link_velocity = array(asset.data.body_link_vel_w)
            if (link_pose.shape != (1, len(names), 7)
                    or com_pose.shape != link_pose.shape
                    or com_velocity.shape != (1, len(names), 6)
                    or link_velocity.shape != com_velocity.shape):
                raise ValueError('adapter supports exactly one environment')
            assets[asset_name] = (names, link_pose[0], com_pose[0],
                                  com_velocity[0], link_velocity[0])
        poses, velocities, transport_errors = {}, {}, []
        for key, (asset_name, link_name) in LINKS.items():
            names, link_poses, com_poses, com_velocities, link_velocities = assets[asset_name]
            i = names.index(link_name)
            link, com = link_poses[i], com_poses[i]
            velocity, link_velocity = com_velocities[i], link_velocities[i]
            pose = Pose(tuple(link[:3]-origin), tuple(link[3:]))
            pose.rotation()  # validate quaternion; never reorder xyzw silently
            Pose(tuple(com[:3]-origin), tuple(com[3:])).rotation()
            twist = BodyVelocity(tuple(velocity[:3]), tuple(velocity[3:]),
                                 tuple(com[:3]-origin), 'com')
            # Both APIs must describe the same measured rigid-body twist.
            error = float(np.linalg.norm(twist.at(pose.pos)-link_velocity[:3]))
            if error > 5e-5 or np.linalg.norm(velocity[3:]-link_velocity[3:]) > 1e-5:
                raise ValueError('native COM/link velocity convention mismatch')
            poses[key], velocities[key] = pose, twist
            transport_errors.append(error)
        result = Snapshot(time_s, Assembly(**poses), Kinematics(**velocities),
                          tuple(origin), max(transport_errors))
        self.last_time = time_s
        return result
