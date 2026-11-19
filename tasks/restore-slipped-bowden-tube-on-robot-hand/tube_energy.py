"""Mechanical-energy diagnostic for compliant_tube's authored mass model."""
import numpy as np

import mechkit as m
from compliant_tube import _basis


def energy(env, tube, height_origin=0.):
    asset = env.unwrapped.scene[tube.part.name]
    poses = m._np(asset.data.body_link_pose_w)[0]
    velocities = m._np(asset.data.body_com_vel_w)[0]
    origin = m._np(env.unwrapped.scene.env_origins[0])
    properties = tube.properties
    ro, ri = properties['outer_radius_m'], properties['inner_radius_m']
    density = properties['density_kg_m3']
    metadata = []
    for i, (a, b) in enumerate(zip(tube.points[:-1], tube.points[1:])):
        length = np.linalg.norm(b-a)
        mass = density*np.pi*(ro*ro-ri*ri)*length
        carrier_mass = 0. if i == 0 else mass*properties['carrier_mass_fraction']
        solid = mass-carrier_mass
        inertia = np.array([solid*(3*(ro*ro+ri*ri)+length*length)/12]*2
                           + [solid*(ro*ro+ri*ri)/2])
        metadata.append(('frame' if i == 0 else f'segment_{i}', solid,
                         (a+b)/2, _basis(b-a), inertia))
        if i:
            metadata.append((f'bend_{i}', carrier_mass, a, np.eye(3),
                             np.full(3, carrier_mass*ro*ro/6)))
    kinetic, gravity = 0., 0.
    maximum_com_speed = 0.
    for name, mass, center, axes, inertia in metadata:
        i = asset.body_names.index(name)
        rotation = m.quat_matrix(poses[i, 3:])
        local_center = center-getattr(tube, 'body_centers', {}).get(name, 0.)
        com = poses[i, :3]-origin+rotation@local_center
        velocity, omega = velocities[i, :3], velocities[i, 3:]
        principal_omega = axes.T@rotation.T@omega
        kinetic += .5*mass*(velocity@velocity)+.5*np.sum(inertia*principal_omega**2)
        gravity += mass*9.81*(com[2]-height_origin)
        maximum_com_speed = max(maximum_com_speed, float(np.linalg.norm(velocity)))
    q = m.state(env, tube.part.name).joints
    bending = sum(.5*k*(q[name+'_joint']-tube.rest_targets[name])**2
                  for i, k in enumerate(properties['bending_stiffness_Nm_per_rad'], 1)
                  for name in (f'bend_{i}', f'segment_{i}'))
    return dict(kinetic_j=float(kinetic), gravity_j=float(gravity),
                bending_j=float(bending), total_j=float(kinetic+gravity+bending),
                maximum_com_speed_m_s=maximum_com_speed)
