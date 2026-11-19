"""Numerical carrier-mass diagnostic, preserving each locked cell's moments.

The original two-axis beam uses a tiny body for its first hinge axis. This
variant redistributes mass between that carrier and its segment, preserving
combined mass, center of mass and inertia at the initial pose. Geometry,
springs, damping, joint limits, gravity and contact properties are unchanged.

This is NOT exact dynamic equivalence: the carrier does not follow the second
hinge rotation. Its changed rotational inertia changes that internal motion.
The comparison therefore requires fresh bending and contact checks before use.
"""
from dataclasses import replace
import numpy as np
from compliant_tube import make_tube as original_make_tube, _basis


def _parallel(mass, displacement):
    return mass*((displacement@displacement)*np.eye(3)-np.outer(displacement, displacement))


def make_tube(name, position, points, *, carrier_fraction=.1, **kwargs):
    if not 0 < carrier_fraction < .2:
        raise ValueError('Diagnostic carrier fraction must be between0 and0.2')
    tube = original_make_tube(name, position, points, **kwargs)
    original_build = tube.part.build
    # Do not import USD or build a layer while the task module is being loaded:
    # Isaac Sim must select its own USD libraries before any pxr module loads.
    # Reconstruct the original helper's authored physical properties directly.
    before = {}
    ro, ri = tube.properties['outer_radius_m'], tube.properties['inner_radius_m']
    density = tube.properties['density_kg_m3']
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        a, b = np.asarray(a), np.asarray(b)
        length = float(np.linalg.norm(b-a))
        mass = density*np.pi*(ro*ro-ri*ri)*length
        cm = 0. if i == 0 else .001*mass
        sm = mass-cm
        axes = _basis(b-a)
        principal = np.array([sm*(3*(ro*ro+ri*ri)+length*length)/12]*2
                             + [sm*(ro*ro+ri*ri)/2])
        before['frame' if i == 0 else f'segment_{i}'] = dict(
            mass=sm, center=(a+b)/2, inertia_matrix=axes@np.diag(principal)@axes.T)
        if i:
            before[f'bend_{i}'] = dict(mass=cm, center=a,
                                       inertia_matrix=np.eye(3)*cm*ro*ro/6)
    after = {name: {key: value.copy() if isinstance(value, np.ndarray) else value
                    for key, value in item.items()} for name, item in before.items()}
    comparisons = []
    for i in range(1, len(points)-1):
        carrier, segment = f'bend_{i}', f'segment_{i}'
        c0, s0 = before[carrier], before[segment]
        total = c0['mass']+s0['mass']
        center = (c0['mass']*c0['center']+s0['mass']*s0['center'])/total
        inertia = sum(x['inertia_matrix']+_parallel(x['mass'], x['center']-center)
                      for x in (c0, s0))
        cm = carrier_fraction*total
        sm = total-cm
        sc = (total*center-cm*c0['center'])/sm
        ci = c0['inertia_matrix']*(cm/c0['mass'])
        si = inertia-ci-_parallel(cm, c0['center']-center)-_parallel(sm, sc-center)
        if np.linalg.eigvalsh(si).min() <= 0:
            raise ValueError('Requested redistribution produces nonphysical inertia')
        # Each component must also obey the rigid-body triangle inequalities.
        vals = np.linalg.eigvalsh(si)
        if vals[-1] > vals[:2].sum()+1e-18:
            raise ValueError('Segment principal moments violate triangle inequality')
        after[carrier] = dict(mass=cm, center=c0['center'], inertia_matrix=ci)
        after[segment] = dict(mass=sm, center=sc, inertia_matrix=si)
        reconstructed = ci+si+_parallel(cm, c0['center']-center)+_parallel(sm, sc-center)
        comparisons.append({'segment': i, 'mass_error_kg': abs(cm+sm-total),
            'com_error_m': float(np.linalg.norm((cm*c0['center']+sm*sc)/total-center)),
            'inertia_max_error_kg_m2': float(np.max(np.abs(reconstructed-inertia))),
            'carrier_rotational_inertia_change_norm_kg_m2': float(np.linalg.norm(ci-c0['inertia_matrix']))})

    def build():
        from pxr import Gf, Sdf, Usd, UsdPhysics
        from scipy.spatial.transform import Rotation
        layer = Sdf.Layer.CreateAnonymous('.usda')
        if not layer.ImportFromString(original_build()):
            raise ValueError('Invalid original USD')
        stage = Usd.Stage.Open(layer)
        root = stage.GetDefaultPrim()
        for name, values in after.items():
            api = UsdPhysics.MassAPI(root.GetChild(name))
            inertia, axes = np.linalg.eigh(values['inertia_matrix'])
            if np.linalg.det(axes) < 0:
                axes[:, 0] *= -1
            q = Rotation.from_matrix(axes).as_quat()
            api.GetMassAttr().Set(values['mass'])
            api.GetCenterOfMassAttr().Set(Gf.Vec3f(*values['center']))
            api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*inertia))
            api.GetPrincipalAxesAttr().Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))
        return layer.ExportToString()

    tube.part = replace(tube.part, build=build)
    tube.mass_properties = after
    tube.redistribution_checks = comparisons
    tube.properties = dict(tube.properties, carrier_mass_fraction=carrier_fraction,
                           mass_representation='locked-cell moments preserved; internal hinge inertia differs')
    return tube
