"""Read-only mass-center and momentum diagnostics for physical beam cells."""
import numpy as np
import mechkit as m


def momentum(env,tube):
    asset = env.unwrapped.scene[tube.part.name]
    poses = m._np(asset.data.body_link_pose_w)[0]
    velocities = m._np(asset.data.body_com_vel_w)[0]
    origin = m._np(env.unwrapped.scene.env_origins[0])
    cells = []
    for name,values in tube.mass_properties.items():
        i = asset.body_names.index(name)
        rotation = m.quat_matrix(poses[i,3:])
        center = poses[i,:3]-origin+rotation@values['center']
        cells.append((values['mass'],center,velocities[i,:3],
                      rotation@values['inertia_matrix']@rotation.T@velocities[i,3:]))
    total_mass = sum(v[0] for v in cells)
    center = sum(mass*pos for mass,pos,_,_ in cells)/total_mass
    linear = sum(mass*velocity for mass,_,velocity,_ in cells)
    angular = sum(spin+np.cross(pos-center,mass*velocity)
                  for mass,pos,velocity,spin in cells)
    return {'center_of_mass_m':center.tolist(),'linear_momentum_kg_m_s':linear.tolist(),
            'angular_momentum_about_com_kg_m2_s':angular.tolist(),
            'total_mass_kg':total_mass,
            'finite':bool(np.isfinite(np.r_[center,linear,angular]).all())}
