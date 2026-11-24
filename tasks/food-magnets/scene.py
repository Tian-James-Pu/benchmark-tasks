"""food-magnets layout, per-trial draws and USD authoring. The layout half is pure Python; the authoring functions
import pxr only when called (after the simulator has started).

Bodies (every contact native PhysX; no attachment, fixed joint, kinematic body or pose write):
- static colliders (taped to the table, card: "box and panel base fixed (taped)"): the galvanized sheet on its plywood
  upright and base; the box body (walls and floor). They carry no rigid body, so nothing can move them.
- ``lid``: one dynamic rigid body on a revolute hinge (the only joint outside the arms) along the box's robot-side top
  edge, resting open at 100 deg against the hinge's limit.
- ``strawberry``, ``cookie``, ``donut``, ``plain``, ``round``, ``square``: dynamic rigid bodies (convex prisms), each
  pulled toward the steel by arena_fm.MagnetForces (F0 / (1 + g / 1 mm)^2 at its back-face disc), held only by that force
  and friction. ``round`` and ``square`` are the two ordinary magnets (r5, review TK-9NQBPY): they hold nothing.
- ``card``: the note, a 152.4 x 101.6 x 0.2 mm dynamic rigid plate, held only by friction under the plain magnet.
"""
import math

import numpy as np

import geometry as G

PHYSICS_HZ = 240
SPAWN_GAP = 0.00005            # bodies authored 0.05 mm off their support
MU_PLA = 0.3                   # PLA on galvanized steel (estimate, gate G1 slide pulls; geometry.md mu 0.3)
MU_STEEL = 0.3
MU_PAPER = 0.4                 # estimate (no published value)
MU_WOOD = 0.4                  # estimate (no published value)
REST_OFFSET = 0.0
CONTACT_OFFSET = 0.001
CONTACT_OFFSET_CARD = 0.0004

COLORS = dict(strawberry=(0.80, 0.10, 0.12), cookie=(0.80, 0.60, 0.36), donut=(0.93, 0.52, 0.68),
              plain=(0.04, 0.04, 0.045), round=(0.04, 0.04, 0.045), square=(0.04, 0.04, 0.045),
              card=(0.96, 0.96, 0.94), sheet=(0.60, 0.62, 0.64), ply=(0.86, 0.74, 0.55), box=(0.86, 0.74, 0.55))
ITEMS = ('plain',) + G.NAMES + G.ORDINARY   # the six magnets, one per cell (r5)


def linear_rgb(srgb):
    return tuple(c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4 for c in srgb)


def draw(seed):
    """One trial's draw: cells (r5: the note's magnet, the 3 food and the 2 ordinary magnets fill the 6 cells in a random
    order), each centre y +/-10 mm and z anywhere in its row, the box spot. seed 0 = a fixed nominal draw.
    Returns dict(name -> (y, z) above the table, m) + box_y."""
    rng = np.random.default_rng(seed if seed else 20261122)
    cells = list(rng.permutation(6))
    out = {}
    for name, ci in zip(ITEMS, cells):
        col, row = G.CELLS[ci]
        y = G.COLS[col] + rng.uniform(-G.JITTER_Y, G.JITTER_Y)
        z = rng.uniform(*G.ROWS[row])
        out[name] = (float(y), float(z))
        out[name + '_cell'] = (int(col), row)
    out['box_y'] = float(G.SPOTS[int(rng.integers(4))])
    return out


def body_start(name, d):
    """World position of a magnet body's centre (unrotated, back face on the steel or on the card)."""
    y, z = d[name]
    back = G.FACE_X - SPAWN_GAP - ((G.CARD_T + SPAWN_GAP) if name == 'plain' else 0.0)
    return (back - G.THICK[name] / 2, y, G.TABLE_TOP + z)


def card_start(d):
    y, z = d['plain']
    return (G.FACE_X - SPAWN_GAP - G.CARD_T / 2, y, G.TABLE_TOP + z - G.CARD_BELOW_MAGNET)


def lid_start(d):
    return (G.HINGE_X, d['box_y'], G.TABLE_TOP + G.BOX_H)


def gaps_ok(d, min_gap=0.020):
    """The card's rule: every draw keeps 20 mm or more between any two items, the note card included (rectangles)."""
    rects = []
    for n in ITEMS:
        y, z = d[n]
        w, h = G.EXTENT[n]
        rects.append((y - w / 2, y + w / 2, z - h / 2, z + h / 2))
    y, z = d['plain']
    rects.append((y - G.CARD_W / 2, y + G.CARD_W / 2, z - G.CARD_BELOW_MAGNET - G.CARD_H / 2, z - G.CARD_BELOW_MAGNET + G.CARD_H / 2))
    card = len(rects) - 1
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            if {i, j} == {0, card}:
                continue      # the plain magnet sits on its card
            a, b = rects[i], rects[j]
            dy = max(a[0] - b[1], b[0] - a[1], 0.0)
            dz = max(a[2] - b[3], b[2] - a[3], 0.0)
            if math.hypot(dy, dz) < min_gap - 1e-9:
                return False
    return True


# -- authoring (Isaac only) ------------------------------------------------------------------------------------

def _xform(stage, path, translation, rot=None):
    from pxr import Gf, UsdGeom
    x = UsdGeom.Xform.Define(stage, path)
    x.ClearXformOpOrder()
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[float(v) for v in translation]))
    w, qx, qy, qz = G.matrix_to_quat_wxyz(np.eye(3) if rot is None else rot)
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(w), float(qx), float(qy), float(qz)))
    x.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 1.0))
    return x


def _physics_material(stage, path, mu):
    from pxr import UsdPhysics, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(mu))
    api.CreateDynamicFrictionAttr(float(mu))
    api.CreateRestitutionAttr(0.0)
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


def _collider(prim, contact_offset=CONTACT_OFFSET):
    from pxr import PhysxSchema, UsdPhysics
    UsdPhysics.CollisionAPI.Apply(prim)
    c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    c.CreateContactOffsetAttr(float(contact_offset))
    c.CreateRestOffsetAttr(float(REST_OFFSET))


def _rigid(prim, mass, iterations=16):
    from pxr import PhysxSchema, UsdPhysics
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
    r = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    r.CreateSolverPositionIterationCountAttr(int(iterations))
    r.CreateSolverVelocityIterationCountAttr(4)
    r.CreateMaxDepenetrationVelocityAttr(0.3)
    r.CreateSleepThresholdAttr(0.0)
    r.CreateStabilizationThresholdAttr(0.0)
    return r


def _bind(prim, phys, look):
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(phys, UsdShade.Tokens.weakerThanDescendants, 'physics')
    UsdShade.MaterialBindingAPI(prim).Bind(look)


def _cube(stage, path, centre, size, phys, look, contact_offset=CONTACT_OFFSET):
    from pxr import Gf, UsdGeom
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(1.0)
    g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in centre]))
    g.AddScaleOp().Set(Gf.Vec3d(*[float(v) for v in size]))
    _collider(g.GetPrim(), contact_offset)
    _bind(g.GetPrim(), phys, look)
    return g.GetPrim()


def _convex(stage, path, pts, phys, look):
    """Convex-hull mesh collider (and render mesh) from prism points: n outline points at the front, n at the back."""
    from pxr import Gf, UsdGeom, UsdPhysics, Vt
    n = len(pts) // 2
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in pts]))
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([int(k) for f in faces for k in f])
    m.CreateSubdivisionSchemeAttr('none')
    _collider(m.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
    _bind(m.GetPrim(), phys, look)
    return m.GetPrim()


def author_magnet(name):
    def author(stage, path, translation):
        root = _xform(stage, path, translation)
        prim = root.GetPrim()
        _rigid(prim, G.MASS[name], iterations=24)
        phys = _physics_material(stage, path + '/PhysPLA', MU_PLA)
        look = _look(stage, path + '/Look', COLORS[name], roughness=0.55)
        _convex(stage, path + '/body', G.prism(name), phys, look)
        return prim
    return author


def author_card(stage, path, translation):
    root = _xform(stage, path, translation)
    prim = root.GetPrim()
    _rigid(prim, G.CARD_MASS, iterations=24)
    phys = _physics_material(stage, path + '/PhysPaper', MU_PAPER)
    look = _look(stage, path + '/Look', COLORS['card'], roughness=0.8)
    _cube(stage, path + '/plate', (0, 0, 0), (G.CARD_T, G.CARD_W, G.CARD_H), phys, look, CONTACT_OFFSET_CARD)
    return prim


def author_fixtures(stage, env_path, box_y):
    """Static colliders: the steel sheet on its plywood stand, and the box body at its drawn spot (both taped)."""
    from pxr import UsdGeom
    T = G.TABLE_TOP
    board = env_path + '/Board'
    UsdGeom.Xform.Define(stage, board)
    steel = _physics_material(stage, board + '/PhysSteel', MU_STEEL)
    wood = _physics_material(stage, board + '/PhysWood', MU_WOOD)
    look_s = _look(stage, board + '/LookSteel', COLORS['sheet'], roughness=0.35, metallic=0.6)
    look_p = _look(stage, board + '/LookPly', COLORS['ply'], roughness=0.7)
    _cube(stage, board + '/sheet', (G.FACE_X + G.SHEET_T / 2, 0.0, T + (G.SHEET_Z0 + G.SHEET_Z1) / 2),
          (G.SHEET_T, 2 * G.SHEET_Y_HALF, G.SHEET_Z1 - G.SHEET_Z0), steel, look_s)
    ux = G.FACE_X + G.SHEET_T + G.UPRIGHT_T / 2
    _cube(stage, board + '/upright', (ux, 0.0, T + G.BASE_T + (G.UPRIGHT_Z - G.BASE_T) / 2),
          (G.UPRIGHT_T, 2 * G.SHEET_Y_HALF, G.UPRIGHT_Z - G.BASE_T), wood, look_p)
    _cube(stage, board + '/base', (G.FACE_X + G.BASE_X / 2, 0.0, T + G.BASE_T / 2), (G.BASE_X, 2 * G.SHEET_Y_HALF, G.BASE_T),
          wood, look_p)
    box = env_path + '/BoxBody'
    x = _xform(stage, box, (G.BOX_X, box_y, T))
    look_b = _look(stage, box + '/Look', COLORS['box'], roughness=0.7)
    ply = _physics_material(stage, box + '/PhysPly', MU_WOOD)
    lx, ly, h, w = G.BOX_LX, G.BOX_LY, G.BOX_H, G.WALL
    _cube(stage, box + '/floor', (0, 0, G.FLOOR_T / 2), (lx, ly, G.FLOOR_T), ply, look_b)
    for nm, c, s in (('wall_near', (-lx / 2 + w / 2, 0, h / 2), (w, ly, h)), ('wall_far', (lx / 2 - w / 2, 0, h / 2), (w, ly, h)),
                     ('wall_left', (0, ly / 2 - w / 2, h / 2), (lx - 2 * w, w, h)),
                     ('wall_right', (0, -ly / 2 + w / 2, h / 2), (lx - 2 * w, w, h))):
        _cube(stage, f'{box}/{nm}', c, s, ply, look_b)
    return x.GetPrim()


def author_lid(box_y_getter):
    def author(stage, path, translation):
        """The lid (top plate + skirt), frame on the hinge line, at rest open 100 deg; its hinge to the static box body."""
        from pxr import Gf, PhysxSchema, UsdPhysics
        env_path = path.rsplit('/', 1)[0]
        box_y = box_y_getter()
        body = author_fixtures(stage, env_path, box_y)
        root = _xform(stage, path, translation, G.LID_OPEN_ROT)
        prim = root.GetPrim()
        _rigid(prim, lid_mass(), iterations=24)
        ply = _physics_material(stage, path + '/PhysPly', MU_WOOD)
        look = _look(stage, path + '/Look', COLORS['box'], roughness=0.7)
        lx, ly, t, pt, w = G.BOX_LX, G.BOX_LY, G.LID_T, G.LID_PLATE_T, G.WALL
        _cube(stage, path + '/plate', (lx / 2, 0, t - pt / 2), (lx, ly, pt), ply, look)
        sk = t - pt
        _cube(stage, path + '/skirt_far', (lx - w / 2, 0, sk / 2), (w, ly, sk), ply, look)
        _cube(stage, path + '/skirt_near', (w / 2, 0, sk / 2), (w, ly, sk), ply, look)
        _cube(stage, path + '/skirt_l', (lx / 2, ly / 2 - w / 2, sk / 2), (lx - 2 * w, w, sk), ply, look)
        _cube(stage, path + '/skirt_r', (lx / 2, -ly / 2 + w / 2, sk / 2), (lx - 2 * w, w, sk), ply, look)
        j = UsdPhysics.RevoluteJoint.Define(stage, env_path + '/lid_hinge')
        j.CreateBody0Rel().SetTargets([body.GetPath()])
        j.CreateBody1Rel().SetTargets([prim.GetPath()])
        j.CreateAxisAttr('Y')
        j.CreateLocalPos0Attr(Gf.Vec3f(float(G.HINGE_X - G.BOX_X), 0.0, float(G.BOX_H)))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLowerLimitAttr(-float(G.LID_OPEN_DEG))     # the hinge stop at the 100 deg rest
        j.CreateUpperLimitAttr(5.0)                         # closed is held by the rim (contact), not the limit
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(0.0)
        return prim
    return author


def lid_mass():
    """The box's 80 g split by volume between body and lid (plate + skirt, 3 mm walls; estimate)."""
    lx, ly, h, w, d = G.BOX_LX, G.BOX_LY, G.BOX_H, G.WALL, G.INNER_D
    body = lx * ly * h - (lx - 2 * w) * (ly - 2 * w) * d
    lid = lx * ly * G.LID_PLATE_T + 2 * (lx + ly - 2 * w) * w * (G.LID_T - G.LID_PLATE_T)
    return G.BOX_MASS * lid / (lid + body)
