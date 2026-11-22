"""ring-on-finger layout, per-trial draws and USD authoring. The layout half is pure Python; the authoring
functions import pxr / Isaac Lab only when called (after the simulator has started).

Bodies (no attachments, fixed joints or kinematic objects; every contact is native PhysX):
- ``hand``: one dynamic rigid body, the PLA hand form bolted to its 150 x 150 x 10 mm steel plate (1,770 g), resting
  on the table. Colliders: plate, wrist, palm, the ring finger's 9.6 mm bone (capsule), its 7.5 mm tip dome (sphere),
  middle / index / little fingers and thumb (capsules). Per-collider density gives the plate 1,770 g and the form 260 g.
- ``skin``: the ring finger's Dragon Skin 10 Medium skin, a native volume-FEM body (66,816 tets, ~1 mm) sleeved on the
  bone, resting on the palm top and capped by the dome; held only by contact.
- ``ring``: one dynamic rigid body, 48 convex pieces of a 316L band (ID 17.75, OD 21.55, width 4.0, 0.5 mm edge radii),
  3.74 g, lying flat on the table.
Contact: rest offset 0 on the ring, skin and hand colliders, so the 0.125 mm radial interference at the PIP crest is
neither removed nor padded; contact offsets 0.5 mm (ring, skin) and 1 mm (hand) only start contact generation early.
"""
import csv
import math
from pathlib import Path

import numpy as np

import geometry as G

HERE = Path(__file__).resolve().parent
PHYSICS_HZ = 240
RING_SPAWN_GAP = 0.0001       # ring authored 0.1 mm above the table
HAND_SPAWN_GAP = 0.0001       # plate authored 0.1 mm above the table
# Friction (card: ring-skin 0.8, pad-steel 0.5; estimates for gate G1). The ring's material uses combine mode
# 'max', which PhysX lets win over the other material's mode, so ring-skin reads 0.8 and ring-pad / ring-table 0.5.
MU_RING = 0.5
MU_SKIN = G.P['mechanism']['mu_ring_skin']
MU_PLA = 0.4
MU_PLATE = 0.5
SKIN_E = 260e3                 # Pa: datasheet 100% modulus 22 psi by a neo-Hookean fit (params.json skin_E_note)
SKIN_NU = 0.45
SKIN_DENSITY = 1070.0          # SG 1.07
SKIN_ITERATIONS = 32
REST_OFFSET = 0.0
CONTACT_OFFSET_FINE = 0.0005
CONTACT_OFFSET_HAND = 0.001

COLORS = dict(pla=(0.86, 0.68, 0.56), plate=(0.33, 0.34, 0.36), cap=(0.93, 0.91, 0.88), skin=(0.93, 0.91, 0.88),
              line=(0.42, 0.26, 0.16), ring=(0.80, 0.81, 0.83))


def linear_rgb(srgb):
    return tuple(c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4 for c in srgb)


def draws(seed):
    """(plate dx, plate dy, ring dx, ring dy) in mm for a trial: seed 0 = nominal; seed k in 1..100 = row k of
    draws.csv (the real rig's printed table, fab/draws.csv); other seeds draw uniformly as the card says."""
    if seed == 0:
        return (0.0, 0.0, 0.0, 0.0)
    rows = {}
    with open(HERE / 'draws.csv') as f:
        for r in csv.DictReader(line for line in f if not line.startswith('#')):
            rows[int(r['trial'])] = tuple(float(r[k]) for k in ('plate_dx_mm', 'plate_dy_mm', 'ring_dx_mm', 'ring_dy_mm'))
    if seed in rows:
        return rows[seed]
    rng = np.random.default_rng(seed)
    h, r = G.P['random']['hand_offset_mm'], G.P['random']['ring_start_offset_mm']
    return tuple(float(v) for v in (*rng.uniform(-h, h, 2), *rng.uniform(-r, r, 2)))


def hand_origin(seed):
    pdx, pdy, _, _ = draws(seed)
    return (G.HAND_AXIS_XY[0] + pdx * G.MM, G.HAND_AXIS_XY[1] + pdy * G.MM, G.TABLE_TOP + HAND_SPAWN_GAP)


def ring_start(seed):
    _, _, rdx, rdy = draws(seed)
    return (G.RING_START_XY[0] + rdx * G.MM, G.RING_START_XY[1] + rdy * G.MM, G.TABLE_TOP + G.RING_W / 2 + RING_SPAWN_GAP)


def skin_centre():
    """Rest-shape centroid of the skin nodes in the hand frame (the skin prim's origin)."""
    return G.skin_tets().points.mean(0)


def pla_density():
    vol = 0.0
    for p in G.hand_prims():
        if p.part == 'plate':
            continue
        if p.kind == 'box':
            vol += np.prod(p.size)
        elif p.kind == 'sphere':
            vol += 4 / 3 * math.pi * p.size[0] ** 3
        else:
            r, s = p.size
            vol += math.pi * r * r * s + 4 / 3 * math.pi * r ** 3
    return G.HAND['form_mass_g'] * 1e-3 / vol


def plate_density():
    pl = G.HAND['plate_mm']
    return G.HAND['plate_mass_g'] * 1e-3 / (pl['x'] * pl['y'] * pl['z'] * 1e-9)


# -- authoring (Isaac only) ------------------------------------------------------------------------------------

def _xform(stage, path, translation):
    from pxr import Gf, UsdGeom
    x = UsdGeom.Xform.Define(stage, path)
    x.ClearXformOpOrder()
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[float(v) for v in translation]))
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    x.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 1.0))
    return x


def _physics_material(stage, path, mu, combine=None):
    from pxr import PhysxSchema, UsdPhysics, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(mu))
    api.CreateDynamicFrictionAttr(float(mu))
    api.CreateRestitutionAttr(0.0)
    if combine:
        px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
        px.CreateFrictionCombineModeAttr(combine)
        px.CreateRestitutionCombineModeAttr('min')
    return mat


def _look(stage, path, rgb, roughness=0.5, metallic=0.0):
    from pxr import Gf, Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + '/Shader')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(rgb)))
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(float(metallic))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return mat


def _collider(prim, contact_offset, rest_offset=REST_OFFSET):
    from pxr import PhysxSchema, UsdPhysics
    UsdPhysics.CollisionAPI.Apply(prim)
    c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    c.CreateContactOffsetAttr(float(contact_offset))
    c.CreateRestOffsetAttr(float(rest_offset))


def _rigid(prim, iterations, max_depenetration):
    from pxr import PhysxSchema, UsdPhysics
    UsdPhysics.RigidBodyAPI.Apply(prim)
    r = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    r.CreateSolverPositionIterationCountAttr(int(iterations))
    r.CreateSolverVelocityIterationCountAttr(4)
    r.CreateMaxDepenetrationVelocityAttr(float(max_depenetration))
    r.CreateEnableCCDAttr(False)   # GPU dynamics
    r.CreateSleepThresholdAttr(0.0)
    r.CreateStabilizationThresholdAttr(0.0)
    return r


def author_hand(stage, path, translation):
    """The hand form + plate rigid body at ``translation`` (the hand frame origin, world). Returns the root prim."""
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    root = _xform(stage, path, translation)
    prim = root.GetPrim()
    _rigid(prim, iterations=16, max_depenetration=0.5)
    pla = _physics_material(stage, path + '/PhysPLA', MU_PLA)
    steel = _physics_material(stage, path + '/PhysPlate', MU_PLATE)
    looks = {k: _look(stage, f'{path}/Look_{k}', COLORS[k], roughness=0.6 if k != 'plate' else 0.4,
                      metallic=0.0 if k != 'plate' else 0.3) for k in ('pla', 'plate', 'cap')}
    dens = dict(pla=pla_density(), cap=pla_density(), plate=plate_density())
    for p in G.hand_prims():
        cpath = f'{path}/{p.name}'
        if p.kind == 'box':
            g = UsdGeom.Cube.Define(stage, cpath)
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*p.centre))
            g.AddScaleOp().Set(Gf.Vec3d(*p.size))
        elif p.kind == 'sphere':
            g = UsdGeom.Sphere.Define(stage, cpath)
            g.CreateRadiusAttr(float(p.size[0]))
            g.AddTranslateOp().Set(Gf.Vec3d(*p.centre))
        else:
            g = UsdGeom.Capsule.Define(stage, cpath)
            g.CreateRadiusAttr(float(p.size[0]))
            g.CreateHeightAttr(float(p.size[1]))
            g.CreateAxisAttr('Z')
            g.AddTranslateOp().Set(Gf.Vec3d(*p.centre))
            r = G.capsule_rot(p.axis)
            w, x, y, z = G.matrix_to_quat_wxyz(r)
            g.AddOrientOp().Set(Gf.Quatf(float(w), float(x), float(y), float(z)))
        cp = g.GetPrim()
        _collider(cp, CONTACT_OFFSET_HAND)
        UsdPhysics.MassAPI.Apply(cp).CreateDensityAttr(float(dens[p.part]))
        UsdShade.MaterialBindingAPI.Apply(cp).Bind(steel if p.part == 'plate' else pla, UsdShade.Tokens.weakerThanDescendants, 'physics')
        UsdShade.MaterialBindingAPI(cp).Bind(looks[p.part])
    return prim


def author_ring(stage, path, translation):
    """The steel ring rigid body (48 convex pieces) lying flat at ``translation`` (its centre, world)."""
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, Vt
    root = _xform(stage, path, translation)
    prim = root.GetPrim()
    _rigid(prim, iterations=32, max_depenetration=0.2)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(G.RING_MASS))
    phys = _physics_material(stage, path + '/PhysSteel', MU_RING, combine='max')
    look = _look(stage, path + '/LookSteel', COLORS['ring'], roughness=0.3, metallic=0.5)
    for i, (pts, faces) in enumerate(G.ring_segments()):
        m = UsdGeom.Mesh.Define(stage, f'{path}/piece{i:02d}')
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in pts]))
        m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr([int(k) for f in faces for k in f])
        m.CreateSubdivisionSchemeAttr('none')
        cp = m.GetPrim()
        _collider(cp, CONTACT_OFFSET_FINE)
        UsdPhysics.MeshCollisionAPI.Apply(cp).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        UsdShade.MaterialBindingAPI.Apply(cp).Bind(phys, UsdShade.Tokens.weakerThanDescendants, 'physics')
        UsdShade.MaterialBindingAPI(cp).Bind(look)
    return prim


def paint_skin_lines(stage, root_path, centre):
    """Bind the painted 1 mm bands (base 7-8, PIP 26-27, DIP 50-51 mm) as face subsets of the skin's render mesh.
    Render only; returns what it bound (or why not) for run.json."""
    from pxr import Usd, UsdGeom, UsdShade, Vt
    meshes = [p for p in Usd.PrimRange(stage.GetPrimAtPath(root_path)) if p.IsA(UsdGeom.Mesh)]
    if not meshes:
        return dict(painted=False, reason='no render mesh under ' + root_path)
    mesh = UsdGeom.Mesh(meshes[0])
    pts = np.asarray(mesh.GetPointsAttr().Get(), float) + np.asarray(centre)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    if not (counts == 3).all():
        return dict(painted=False, reason='render mesh is not triangles')
    tri = idx.reshape(-1, 3)
    u = (pts[tri][:, :, 2].mean(1) - G.WEB_Z) / G.MM
    rho = np.hypot(pts[tri][:, :, 0].mean(1), pts[tri][:, :, 1].mean(1)) / G.MM
    outer = rho > 6.5
    faces = []
    for lo, hi in (v for k, v in G.RF['lines_u_mm'].items() if k != 'width'):
        faces += np.flatnonzero(outer & (u >= lo - 0.5) & (u <= hi + 0.5)).tolist()
    look = _look(stage, root_path + '/LookLine', COLORS['line'], roughness=0.7)
    subset = UsdGeom.Subset.CreateGeomSubset(mesh, 'PaintedLines', UsdGeom.Tokens.face, Vt.IntArray(sorted(faces)), 'materialBind')
    UsdGeom.Subset.SetFamilyType(mesh, 'materialBind', UsdGeom.Tokens.nonOverlapping)
    UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(look)
    return dict(painted=True, faces=len(faces), mesh=str(mesh.GetPath()))
