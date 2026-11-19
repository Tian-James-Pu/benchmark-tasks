"""Energy audit using the redistributed carrier diagnostic's actual mass model."""
import numpy as np
import mechkit as m


def energy(env, tube, height_origin=0.):
    asset = env.unwrapped.scene[tube.part.name]
    poses = m._np(asset.data.body_link_pose_w)[0]
    velocities = m._np(asset.data.body_com_vel_w)[0]
    origin = m._np(env.unwrapped.scene.env_origins[0])
    kinetic, gravity, maximum_com_speed = 0., 0., 0.
    for name, values in tube.mass_properties.items():
        i = asset.body_names.index(name)
        rotation = m.quat_matrix(poses[i, 3:])
        com = poses[i, :3]-origin+rotation@values['center']
        velocity, omega = velocities[i, :3], rotation.T@velocities[i, 3:]
        kinetic += .5*values['mass']*(velocity@velocity)+.5*omega@values['inertia_matrix']@omega
        gravity += values['mass']*9.81*(com[2]-height_origin)
        maximum_com_speed = max(maximum_com_speed, float(np.linalg.norm(velocity)))
    q = m.state(env, tube.part.name).joints
    bending = sum(.5*k*(q[name+'_joint']-tube.rest_targets[name])**2
                  for i, k in enumerate(tube.properties['bending_stiffness_Nm_per_rad'], 1)
                  for name in (f'bend_{i}', f'segment_{i}'))
    return dict(kinetic_j=float(kinetic), gravity_j=float(gravity),
                bending_j=float(bending), total_j=float(kinetic+gravity+bending),
                maximum_com_speed_m_s=maximum_com_speed)
