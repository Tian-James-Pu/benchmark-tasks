"""Close the open end joint of a laminate plank: tapping block and rubber mallet.

A cropped section of a floating laminate floor being laid, mounted at table height: an 18 mm plywood subfloor panel,
1.64 m long, running straight away from the arms (it overhangs the table's far edge onto a trestle). Rows 0 and 1 are
laid on the right. In row 2, the plank just laid (Pergo Pro, 190 x 1200 mm, 8 mm HDF on a 2 mm attached pad, Uniclic
joints, 1.6 kg) is locked flat along its right long edge, but its far short end stands 5 mm off the plank before it in
the row: the end joint is open. Its near short end, the end groove with its protruding lower lip, faces the arms over
bare subfloor. A Roberts 10-49 Double Tap tapping block (254 x 89 x 19 mm reinforced nylon with a raised D-handle) lies
on the subfloor in front of it; a rubber mallet lies on the laid floor to the right. A retractable utility knife, a
pair of work gloves and a roll of tape are distractors.

The plank slides along its length in its long-edge lock against joint friction calibrated so that the steadiest push
an arm can give through the block does not start it, while a mallet tap on the block moves it about a millimetre
(`calibrate` mode below measures both).

    python demokit.py task.py OUT --mode full                      # the demonstration, with video (GPU)
    python demokit.py task.py OUT --no-record --mode none,onearm,careless,push    # the red-team modes
"""

import math
import os
import pathlib
import sys

import numpy as np

from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
import simready  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Close the open end joint of the laminate plank that was just laid."

# -- dimensions (metres, in the floor's frame: world axes, origin on the table top) ------------------------------------

PLY = 0.018  # plywood subfloor thickness; the floor surface is at z = ZF
ZF = PLY
SUB_X, SUB_Y = (0.10, 1.74), (-0.42, 0.40)  # subfloor panel extent

PLANK_L, PLANK_W, PLANK_T = 1.200, 0.190, 0.010  # Pergo Pro TDS_0053: 1200 x 190 mm, 10 mm = 8 mm HDF + 2 mm pad
PAD_T = 0.002
PLANK_MASS = 1.6  # 17.6 kg per 11-plank carton
GAP0 = 0.005  # the end joint starts open this far
X0 = 0.400  # the plank's near end face (above its groove) at the start
X_JOINT = X0 + PLANK_L + GAP0  # face of the previous plank in the row, where the plank's far end closes
Y_R = -0.065  # the plank's locked long edge, against row 1
Y_L = Y_R + PLANK_W  # its free long edge, over bare subfloor
LIP = (0.006, 0.002, 0.0045)  # the end groove's lower lip: protrudes 6 mm past the face, from 2 to 4.5 mm up
TONGUE = (0.007, 0.0045, 0.0075)  # the far end's tongue: 7 mm long, 4.5 to 7.5 mm up

# The joint's resistance to sliding along the plank's length in its long-edge lock (set at every reset).
STATIC_N = 70.0  # no movement below this push along the plank
DYNAMIC_N = 10.0  # sliding friction once moving
VISCOUS = 20.0  # N s/m

# Roberts 10-49 Double Tap reversible tapping block, click-lock side down. Its own frame: x towards the plank, origin at
# the middle of the bearing edge, on the floor. Body 254 x 89 x 19 mm; a 6 mm high, 8 mm deep relief along the bottom of
# the bearing edge clears the groove's lower lip, so the block bears on the plank's face above it. A raised D-handle,
# offset towards the bearing edge (away from the struck back face): two posts and a bar, 91 mm high overall.
BLK_L, BLK_D, BLK_H = 0.254, 0.089, 0.019
RELIEF = (0.008, 0.006)
BAR_X = (-0.045, -0.021)  # handle bar and posts, front to back span in block x
BAR_Z = (0.067, 0.091)
POST_Y = (0.050, 0.070)  # |y| span of each post; the bar runs between their outer faces
BLOCK_MASS = 0.30
BLOCK_Y = Y_R + 0.003 + BLK_L / 2  # block centre in y when in place: its right end 3 mm clear of row 1
BLOCK_START = (0.255, 0.120, 12.0)  # bearing-edge middle (x, y) and yaw (degrees) where it lies at the start

# Rubber mallet: SimReady handTool_hammer_rubberMallet_b01, 299 mm long. In its file (x along the handle, z up, origin
# under the head): head 67 mm across and 114 mm long (axis along y) centred at (-0.148, 0, 0.0336); handle 28 mm across
# from x -0.115 to 0.118, on the same axis height. Its SDF mesh collider is replaced at spawn by two cylinders that match
# it, and its authored mass (0.35 kg, centre of mass on the handle) by a head-heavy 0.45 kg.
MALLET_HEAD_C = np.array([-0.148, 0.0, 0.0336])
MALLET_HEAD_R, MALLET_HEAD_HALF = 0.0335, 0.057
MALLET_HANDLE = (-0.115, 0.118, 0.014)  # x from, x to, radius
MALLET_MASS = (0.33, 0.12)  # head, handle (kg)
MALLET_START = (0.215, -0.150)  # head centre (x, y), lying on the laid floor, handle pointing right (-y)
MALLET_YAW = -math.pi / 2  # file +x (towards the handle's end) along world -y; the head's faces look along world x
ROW_TOP = ZF + PLANK_T  # top of the laid floor

LOOKS = {
    "ply": m.Look(m.rgb(0.78, 0.66, 0.47), roughness=0.85),
    "decor": m.Look(m.rgb(0.74, 0.58, 0.40), roughness=0.55),
    "decor2": m.Look(m.rgb(0.66, 0.50, 0.33), roughness=0.55),
    "grain": m.Look(m.rgb(0.55, 0.40, 0.26), roughness=0.6),
    "seam": m.Look(m.rgb(0.20, 0.15, 0.10), roughness=0.8),
    "hdf": m.Look(m.rgb(0.50, 0.38, 0.26), roughness=0.8),
    "pad": m.Look(m.rgb(0.12, 0.12, 0.13), roughness=0.95),
    "block": m.Look(m.rgb(0.08, 0.08, 0.09), roughness=0.5),
    "handle": m.Look(m.rgb(0.75, 0.10, 0.10), roughness=0.45),
    "trestle": m.Look(m.rgb(0.45, 0.46, 0.48), metallic=0.5, roughness=0.5),
}


def _plank_meshes(x0, x1, y0, y1, prefix, look="decor", grains=4):
    """Visual meshes for one plank between x0 and x1 (full thickness): pad, HDF core, decor top and grain lines."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    parts = [
        (f"{prefix}_pad", m.box((x1 - x0, y1 - y0, PAD_T), (cx, cy, ZF + PAD_T / 2)), "pad"),
        (f"{prefix}_core", m.box((x1 - x0, y1 - y0, PLANK_T - PAD_T - 0.0006), (cx, cy, ZF + (PLANK_T + PAD_T - 0.0006) / 2)), "hdf"),
        (f"{prefix}_top", m.box((x1 - x0 - 0.001, y1 - y0 - 0.001, 0.0006), (cx, cy, ZF + PLANK_T - 0.0003)), look),
    ]
    for k in range(grains):
        y = y0 + (k + 0.6) * (y1 - y0) / (grains + 0.2)
        a, b = x0 + 0.03 + 0.07 * (k % 2), x1 - 0.05 - 0.06 * ((k + 1) % 2)
        if b > a:
            parts.append((f"{prefix}_g{k}", m.box((b - a, 0.0012, 0.0002), ((a + b) / 2, y, ZF + PLANK_T + 0.0004)), "grain"))
    return parts


def _lip(x_face, y0, y1, prefix):
    """An end groove's protruding lower lip, in front of a face at x_face (towards -x)."""
    return (f"{prefix}_lip", m.box((LIP[0], y1 - y0, LIP[2] - LIP[1]), (x_face - LIP[0] / 2, (y0 + y1) / 2, ZF + (LIP[1] + LIP[2]) / 2)), "hdf")


def _floor():
    """The fixed part of the floor (subfloor, rows 0 and 1, the previous plank of row 2, the trestle) and the plank."""
    fp, fc = [], []
    sx, sy = SUB_X, SUB_Y
    fp.append(("ply", m.box((sx[1] - sx[0], sy[1] - sy[0], PLY), ((sx[0] + sx[1]) / 2, (sy[0] + sy[1]) / 2, PLY / 2)), "ply"))
    fc.append(m.col_box("ply_c", (sx[1] - sx[0], sy[1] - sy[0], PLY), ((sx[0] + sx[1]) / 2, (sy[0] + sy[1]) / 2, PLY / 2)))
    # the trestle under the overhang beyond the table's far edge: two legs and a cross bar, down to the room floor
    for j, y in enumerate((-0.38, 0.36)):
        fp.append((f"leg{j}", m.box((0.04, 0.04, TOP), (1.45, y, -TOP / 2)), "trestle"))
    fp.append(("bar", m.box((0.06, sy[1] - sy[0] - 0.04, 0.03), (1.45, (sy[0] + sy[1]) / 2, -0.015)), "trestle"))
    # rows 0 and 1, laid earlier, cropped at the panel's edges; their end joints staggered from ours
    for row, (y0, joints) in enumerate(((Y_R - 2 * PLANK_W, (0.55,)), (Y_R - PLANK_W, (0.76,)))):
        y0c, y1 = max(y0, sy[0]), y0 + PLANK_W
        cuts = (sx[0], *joints, sx[1])
        for k in range(len(cuts) - 1):
            a, b = cuts[k] + (0.0005 if k else 0.0), cuts[k + 1] - (0.0005 if k + 1 < len(cuts) - 1 else 0.0)
            name = f"r{row}{'ab'[k]}"
            fp += _plank_meshes(a, b, y0c, y1, name, ("decor", "decor2")[(row + k) % 2])
            fc.append(m.col_box(f"{name}_c", (b - a, y1 - y0c, PLANK_T), ((a + b) / 2, (y0c + y1) / 2, ZF + PLANK_T / 2)))
        for x in joints:
            fp.append((f"r{row}_seam", m.box((0.0012, y1 - y0c, 0.0003), (x, (y0c + y1) / 2, ZF + PLANK_T)), "seam"))
        fp.append((f"r{row}_lock", m.box((sx[1] - sx[0], 0.0008, 0.0003), ((sx[0] + sx[1]) / 2, y1, ZF + PLANK_T)), "seam"))
    # row 2: the plank before ours, cropped at the panel's far end, with its end groove facing ours
    fp += _plank_meshes(X_JOINT, sx[1], Y_R, Y_L, "p0", "decor2", grains=3)
    fp.append(_lip(X_JOINT, Y_R, Y_L, "p0"))
    fc.append(m.col_box("p0_c", (sx[1] - X_JOINT, PLANK_W, PLANK_T), ((X_JOINT + sx[1]) / 2, (Y_R + Y_L) / 2, ZF + PLANK_T / 2)))

    # our plank: it slides along x in the long-edge lock (joint 0 = 5 mm open, GAP0 = closed)
    x1 = X0 + PLANK_L
    parts = _plank_meshes(X0, x1, Y_R, Y_L, "plank", "decor", grains=4)
    parts.append(_lip(X0, Y_R, Y_L, "plank"))
    parts.append(("groove", m.box((0.0006, PLANK_W - 0.002, TONGUE[2] - TONGUE[1]), (X0 - 0.0002, (Y_R + Y_L) / 2, ZF + (TONGUE[1] + TONGUE[2]) / 2)), "seam"))
    parts.append(("tongue", m.box((TONGUE[0], PLANK_W - 0.004, TONGUE[2] - TONGUE[1]), (x1 + TONGUE[0] / 2, (Y_R + Y_L) / 2, ZF + (TONGUE[1] + TONGUE[2]) / 2)), "hdf"))
    cols = (m.col_box("plank_c", (PLANK_L, PLANK_W, PLANK_T), ((X0 + x1) / 2, (Y_R + Y_L) / 2, ZF + PLANK_T / 2)),
            m.col_box("lip_c", (LIP[0], PLANK_W, LIP[2] - LIP[1]), (X0 - LIP[0] / 2, (Y_R + Y_L) / 2, ZF + (LIP[1] + LIP[2]) / 2)))
    com = ((X0 + x1) / 2, (Y_R + Y_L) / 2, ZF + PLANK_T / 2)
    inertia = (PLANK_MASS * (PLANK_W ** 2 + PLANK_T ** 2) / 12, PLANK_MASS * (PLANK_L ** 2 + PLANK_T ** 2) / 12,
               PLANK_MASS * (PLANK_L ** 2 + PLANK_W ** 2) / 12)
    plank = m.slider("plank", "X", -0.005, GAP0, parts, cols, at=(X0, Y_R, ZF), mass=PLANK_MASS, com=com, inertia=inertia,
                     damping=0.0, friction=STATIC_N, effort=500.0, spring=0.0)
    return fp, fc, [plank]


FRAME_PARTS, FRAME_COLS, LINKS = _floor()
FLOOR = m.mechanism("flooring", (0.0, 0.0, TOP), LINKS, LOOKS, frame_parts=FRAME_PARTS, frame_colliders=FRAME_COLS,
                    friction=(0.9, 0.8), solver_iters=48)


def _block_pieces():
    d, L, h = BLK_D, BLK_L, BLK_H
    rx, rz = RELIEF
    bx0, bx1 = BAR_X
    pieces = {
        "body": ((d - rx, L, h), (-(d + rx) / 2, 0.0, h / 2)),
        "nose": ((rx, L, h - rz), (-rx / 2, 0.0, (h + rz) / 2)),
        "bar": ((bx1 - bx0, 2 * POST_Y[1], BAR_Z[1] - BAR_Z[0]), ((bx0 + bx1) / 2, 0.0, (BAR_Z[0] + BAR_Z[1]) / 2)),
    }
    for s, side in ((1, "l"), (-1, "r")):
        pieces[f"post_{side}"] = ((bx1 - bx0, POST_Y[1] - POST_Y[0], BAR_Z[0] - h), ((bx0 + bx1) / 2, s * (POST_Y[0] + POST_Y[1]) / 2, (h + BAR_Z[0]) / 2))
    looks = {"body": "block", "nose": "block"}
    parts = [(k, m.box(s, c), looks.get(k, "handle")) for k, (s, c) in pieces.items()]
    cols = [m.col_box(f"{k}_c", s, c) for k, (s, c) in pieces.items()]
    return parts, cols


_BP, _BC = _block_pieces()
BLOCK = m.free("block", (BLOCK_START[0], BLOCK_START[1], TOP + ZF + 0.001), _BP, _BC, LOOKS, mass=BLOCK_MASS,
               friction=(0.9, 0.8), yaw_deg=BLOCK_START[2])


def _mallet_origin(head_xy, z):
    """The mallet file's origin for a head centre at head_xy, lying with the handle pointing to -y."""
    return (head_xy[0], head_xy[1] + MALLET_HEAD_C[0], z)


MALLET = SimReady("mallet", pos=_mallet_origin(MALLET_START, TOP + ROW_TOP + 0.002), size=(0.299, 0.114, 0.067),
                  mass=sum(MALLET_MASS), yaw=MALLET_YAW, package="handTool_hammer_rubberMallet_b01",
                  usd="sm_handTool_hammer_rubberMallet_b01_01.usd")

OBJECTS = [
    FLOOR,
    BLOCK,
    MALLET,
    # Distractors (SimReady), which scoring ignores.
    SimReady("knife", pos=(0.56, -0.33, TOP + ROW_TOP + 0.001), size=(0.160, 0.061, 0.026), mass=0.12, yaw=0.5, convex=True,
             package="handTool_knife_retractableBlade_a01", usd="sm_handTool_knife_retractableBlade_a01_01.usd"),
    SimReady("gloves", pos=(0.80, 0.27, TOP + ZF + 0.001), size=(0.233, 0.252, 0.053), mass=0.12, yaw=0.3,
             package="ppe_gloves_cloth_a01", usd="sm_ppe_gloves_cloth_a01_01.usd"),
    SimReady("tape_roll", pos=(0.56, 0.30, TOP + ZF + 0.001), size=(0.10, 0.10, 0.05), mass=0.06,
             package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd"),
]


# -- the mallet's colliders and mass ------------------------------------------------------------------------------------


def _mallet_inertia():
    """Mass, centre of mass (file frame) and principal inertia of a 0.33 kg head on a 0.12 kg handle."""
    mh, mt = MALLET_MASS
    hx = MALLET_HEAD_C[0]
    tx = (MALLET_HANDLE[0] + MALLET_HANDLE[1]) / 2
    tl, tr = MALLET_HANDLE[1] - MALLET_HANDLE[0], MALLET_HANDLE[2]
    cx = (mh * hx + mt * tx) / (mh + mt)
    r, hl = MALLET_HEAD_R, 2 * MALLET_HEAD_HALF
    head_xx = head_zz = mh * (3 * r * r + hl * hl) / 12
    head_yy = mh * r * r / 2
    hand_xx = mt * tr * tr / 2
    hand_yy = hand_zz = mt * (3 * tr * tr + tl * tl) / 12
    dh, dt = hx - cx, tx - cx
    ixx = head_xx + hand_xx
    iyy = head_yy + mh * dh * dh + hand_yy + mt * dt * dt
    izz = head_zz + mh * dh * dh + hand_zz + mt * dt * dt
    return mh + mt, (cx, 0.0, MALLET_HEAD_C[2]), (ixx, iyy, izz)


def _mallet_colliders(prim) -> None:
    """Replace the mallet's one SDF mesh collider (it jitters on the floor and grips poorly) with a cylinder for the head
    and one for the handle, matching the mesh, and give its body a head-heavy mass."""
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    while instances := [p for p in Usd.PrimRange(prim) if p.IsInstance()]:
        for p in instances:
            p.SetInstanceable(False)
    body = [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)][0]
    for p in Usd.PrimRange(body):
        if p != body and p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            for api in ("PhysicsMassAPI",):
                if api in p.GetAppliedSchemas():
                    p.RemoveAppliedSchema(api)
    # file frame -> body frame
    t = Usd.TimeCode.Default()
    rel = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(t) * UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(t).GetInverse()
    inv = rel.GetInverse()

    def local(p):
        v = inv.Transform(Gf.Vec3d(*[float(c) for c in p]))
        return Gf.Vec3d(v[0], v[1], v[2])

    stage = prim.GetStage()
    mat = UsdShade.Material.Define(stage, prim.GetPath().AppendChild("mallet_physics_material"))
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(0.9)
    pm.CreateRestitutionAttr(0.0)
    hx = MALLET_HEAD_C
    tx = ((MALLET_HANDLE[0] + MALLET_HANDLE[1]) / 2, 0.0, MALLET_HEAD_C[2])
    for name, axis, radius, height, centre in (("col_head", "Y", MALLET_HEAD_R, 2 * MALLET_HEAD_HALF, hx),
                                                 ("col_handle", "X", MALLET_HANDLE[2], MALLET_HANDLE[1] - MALLET_HANDLE[0], tx)):
        cyl = UsdGeom.Cylinder.Define(stage, body.GetPath().AppendChild(name))
        cyl.CreateAxisAttr(axis)
        cyl.CreateRadiusAttr(float(radius))
        cyl.CreateHeightAttr(float(height))
        cyl.AddTranslateOp().Set(local(centre))
        cyl.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        pc = PhysxSchema.PhysxCollisionAPI.Apply(cyl.GetPrim())
        pc.CreateContactOffsetAttr(0.004)
        pc.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    mass, com, inertia = _mallet_inertia()
    ma = UsdPhysics.MassAPI.Apply(body)
    ma.CreateMassAttr(float(mass))
    ma.CreateCenterOfMassAttr(Gf.Vec3f(*[float(c) for c in local(com)]))
    ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in inertia]))
    ma.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


_PLAIN_PREPARE = simready._prepare


def _prepare(prim, spec) -> None:
    _PLAIN_PREPARE(prim, spec)
    if spec.name == "mallet":
        _mallet_colliders(prim)


simready._prepare = _prepare

# -- reading the scene ------------------------------------------------------------------------------------------------


def _head_points():
    """Points on the mallet head's surface (both faces and the barrel), in the mallet file's frame."""
    pts = []
    for a in np.linspace(0, 2 * math.pi, 16, endpoint=False):
        for yy in np.linspace(-MALLET_HEAD_HALF, MALLET_HEAD_HALF, 5):
            pts.append(MALLET_HEAD_C + (MALLET_HEAD_R * math.cos(a), yy, MALLET_HEAD_R * math.sin(a)))
        for r in (0.5 * MALLET_HEAD_R,):
            for yy in (-MALLET_HEAD_HALF, MALLET_HEAD_HALF):
                pts.append(MALLET_HEAD_C + (r * math.cos(a), yy, r * math.sin(a)))
    for yy in (-MALLET_HEAD_HALF, MALLET_HEAD_HALF):
        pts.append(MALLET_HEAD_C + (0.0, yy, 0.0))
    return np.array(pts)


HEAD_POINTS = _head_points()


MALLET_BODY = np.array([0.0, 0.0, 0.03362])  # the SimReady body's frame (its root pose) in the file's frame
MALLET_CENTROID = np.array([-0.03171, -0.00042, 0.03366])  # its bounding box's centre (what object_states reports as pos)


def mallet_frame(st):
    """Rotation and origin of the mallet's file frame in the world, from its reported state (centroid and orientation)."""
    R = m.quat_matrix(st.quat_xyzw)
    return R, np.asarray(st.pos, dtype=float) - R @ MALLET_CENTROID


def mallet_frame_root(environment):
    """The same from the body's root pose (exact), for the scorer."""
    base = environment.unwrapped
    pose = m._np(base.scene["mallet"].data.root_link_pose_w)[0]
    R = m.quat_matrix(pose[3:7])
    return R, pose[:3] - m._np(base.scene.env_origins[0]) - R @ MALLET_BODY


def head_world(frame):
    R, p = frame
    return HEAD_POINTS @ R.T + p


# Plank zones in the plank's own frame at joint zero (floor frame, table top at z = 0).
_PLANK_BOX = (np.array([X0, Y_R, ZF]), np.array([X0 + PLANK_L, Y_L, ZF + PLANK_T]))
_LIP_BOX = (np.array([X0 - LIP[0], Y_R, ZF + LIP[1]]), np.array([X0, Y_L, ZF + LIP[2]]))
_END_BOX = (np.array([X0, Y_R, ZF]), np.array([X0 + 0.025, Y_L, ZF + PLANK_T]))  # the end's face and top edge


def to_plank(points, slide):
    """World points in the plank's frame at joint zero."""
    return np.atleast_2d(points) - np.array([slide, 0.0, TOP])


def box_distance(local, box):
    lo, hi = box
    return np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1)


def block_box_distance(points, st):
    """Distance from world points to the tapping block's bounding box (handle included)."""
    local = st.to_local(points)
    lo = np.array([-BLK_D, -BLK_L / 2, 0.0])
    hi = np.array([0.0, BLK_L / 2, BAR_Z[1]])
    return np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1)


# -- the scorer -------------------------------------------------------------------------------------------------------

CLOSED = 0.0005  # full credit: end gap 0.5 mm or less
HALF = 0.0015
QUARTER = 0.003
STRIKE = 0.002  # the mallet's head within 2 mm of the plank's end: a direct blow
GRIP_TOUCH = 0.007  # a fingertip point within 7 mm of the plank's end (the points lie mid-finger, 4.5 mm inside its face)
CLEAR = 0.03  # every fingertip at least 3 cm from the plank and from the tapping block
STILL = 0.002  # plank sliding slower than 2 mm/s
HOLD_S = 2.0
SUCCESS_HOLD_S = 0.0  # the Trial counts its own hold
SCORE_OVER = "end"


def level(gap):
    """The score for an end gap (metres) with no direct blow recorded, before the hands-clear hold."""
    if gap <= CLOSED + 1e-9:
        return 0.75
    if gap <= HALF + 1e-9:
        return 0.5
    if gap <= QUARTER + 1e-9:
        return 0.25
    return 0.0


_DEBUG = os.environ.get("LAMINATE_DEBUG")


class Trial:
    """Updated once per control step. `struck` records that the mallet's head or a gripper touched the plank's end (a direct
    blow that crushes the groove's lip); it is cleared only in `reset`, at a reset."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):
        self.struck = False
        self.hold = 0.0
        self.full = False
        self.value = 0.0
        self.gap = GAP0
        self.steps = 0
        if environment is not None:
            _set_joint_friction(environment)

    def step(self, environment):
        st = m.state(environment, "flooring")
        slide = st.joint("plank")
        self.gap = max(GAP0 - slide, 0.0)
        local = to_plank(head_world(mallet_frame_root(environment)), slide)
        d_head = float(min(box_distance(local, _LIP_BOX).min(), box_distance(local, _END_BOX).min()))
        tips = m.fingertips(environment)
        tl = to_plank(tips, slide)
        d_grip = float(min(box_distance(tl, _LIP_BOX).min(), box_distance(tl, _END_BOX).min()))
        if d_head <= STRIKE or d_grip <= GRIP_TOUCH:
            self.struck = True
        blk = m.state(environment, "block")
        d_tips = float(min(box_distance(tl, _PLANK_BOX).min(), box_distance(tl, _LIP_BOX).min(), block_box_distance(tips, blk).min()))
        speed = abs(float(m._np(environment.unwrapped.scene["flooring"].data.joint_vel)[0][0]))
        ok = (not self.struck and self.gap <= CLOSED + 1e-9 and d_tips >= CLEAR and speed < STILL)
        dt = float(environment.unwrapped.step_dt)
        self.hold = self.hold + dt if ok else 0.0
        self.full = bool(ok and self.hold >= HOLD_S - 1e-9)
        if self.struck:
            self.value = 0.0
        elif self.full:
            self.value = 1.0
        else:
            self.value = level(self.gap)
        self.steps += 1


def _set_joint_friction(environment):
    """Static, sliding and viscous friction for the plank's slide in the lock."""
    import torch

    asset = environment.unwrapped.scene["flooring"]
    j = asset.find_joints("plank_joint")[0][0]

    def t(v):
        return torch.tensor([[float(v)]], device=asset.device)

    asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=t(STATIC_N), joint_dynamic_friction_coeff=t(DYNAMIC_N),
                                                        joint_viscous_friction_coeff=t(VISCOUS), joint_ids=[j])


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demonstration (closed loop: it reads poses and the plank's visible position, never the scorer) --------

BAR_C = np.array([(BAR_X[0] + BAR_X[1]) / 2, 0.0, (BAR_Z[0] + BAR_Z[1]) / 2])  # the D-handle bar's middle, block frame
GRIP_UP = 0.004  # TCP this far above the bar's middle
HANDLE_GRIP = -0.097  # where the right hand takes the mallet's handle (file x): 1.8 cm from the head's neck, choked up
HIT_Y = Y_R + PLANK_W / 2  # tap in line with the plank's centre
HEAD_Z = ZF + 0.040  # head centre height for a tap: its face's lower edge 6.5 mm above the subfloor
TAP_BACK, TAP_PAST, TAP_T = 0.060, 0.040, 0.15  # a tap: from 60 mm behind the block's back to 40 mm past it, in 0.15 s


def _report(ctx, label):
    if not _DEBUG:
        return
    with open(_DEBUG, "a") as f:
        f.write(f"# {label} gap={(GAP0 - ctx.joint('flooring', 'plank')) * 1000:.3f} blk={np.round(ctx.pos('block') - (0, 0, TOP), 4)} "
                f"mal={np.round(ctx.pos('mallet') - (0, 0, TOP), 4)} L={np.round(ctx.left.tcp() - (0, 0, TOP), 4)} R={np.round(ctx.right.tcp() - (0, 0, TOP), 4)}\n")


def demo_script(ctx):
    """Left hand: takes the tapping block by its D-handle, squares it and sets it flat on the subfloor against the plank's
    near end (the relief over the groove's lower lip) and holds it there with a light push. Right hand: takes the mallet
    by the handle near the head, brings the head level behind the block, in line with the plank's centre, and taps the
    block's back until the plank's far end has closed (it watches the plank's end move). Then it lays the mallet on the
    laid floor and both hands move clear.
    Red-team modes: none (nothing), push (the block pushed hard against the end, no mallet), onearm (the block set against the
    end and let go, then tapped), careless (the mallet's head tapped straight on the plank's end)."""
    import demokit as d
    from yam_arena.constants import FINGER_OPEN

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "push", "onearm", "careless", "punch", "calibrate"):
        raise ValueError(mode)
    for _ in range(3):
        yield
    _report(ctx, f"{mode} start")
    if mode == "none":
        for _ in range(100):
            yield
        return
    if mode == "calibrate":
        yield from _calibrate(ctx)
        return
    if mode == "punch":
        yield from _punch(ctx)
        return
    up = np.array([0.0, 0.0, 1.0])
    floor = TOP + ZF

    def plank_end():
        return X0 + ctx.joint("flooring", "plank")

    def block_pose():
        st = ctx.state("block")
        R = m.quat_matrix(st.quat_xyzw)
        return R, np.asarray(st.pos, dtype=float), math.degrees(math.atan2(R[1, 0], R[0, 0]))

    def bar_world():
        R, p, yaw = block_pose()
        return p + R @ BAR_C, yaw

    # -- left: the block
    if True:
        bar, yaw = bar_world()
        grip_l = d.down(-90.0 + yaw)
        above = bar + up * 0.07
        yield from l.move(above, grip=grip_l, finger=FINGER_OPEN, t=2.0, label="over the block's handle")
        yield from l.servo(above, timeout=1.0, label="over the block's handle")
        bar, yaw = bar_world()
        yield from l.move(bar + up * GRIP_UP, t=1.0, label="round the handle")
        yield from l.servo(bar + up * GRIP_UP, timeout=1.0, label="round the handle")
        yield from l.close(BAR_X[1] - BAR_X[0], 30.0, t=0.5, label="grip the handle")
        yield from l.move(l.at() + up * 0.02, t=0.6, label="lift the block")
        _report(ctx, "block lifted")
        if mode == "careless":
            # the block set aside on the subfloor to the left; the mallet will go straight on the plank's end
            aside = np.array([0.27, 0.26, floor]) + BAR_C + up * GRIP_UP
            yield from l.move(aside + up * 0.02, grip=d.down(-90.0), t=1.5, label="block aside")
            yield from l.move(aside + up * 0.002, t=0.6, label="down")
            yield from l.open(label="let go of the block")
            yield from l.move(np.array([0.20, 0.30, TOP + 0.20]), t=1.2, label="away")
    if mode != "careless":
        # square it 15 mm short of the plank's end, centred where it goes, then down flat on the subfloor
        tgt = np.array([plank_end() - 0.015, BLOCK_Y, floor]) + BAR_C + up * GRIP_UP
        yield from l.move(tgt + up * 0.02, grip=d.down(-90.0), t=1.5, label="square behind the plank's end")
        yield from l.servo(tgt + up * 0.02, timeout=1.0, label="square behind the plank's end")
        yield from l.move(tgt + up * 0.001, t=0.6, label="down flat on the subfloor")
        # slide it on until it bears on the face above the lip
        yield from l.move(np.array([plank_end() + 0.003, BLOCK_Y, floor]) + BAR_C + up * (GRIP_UP + 0.001), t=1.0, label="against the end")
        _report(ctx, "block on")
        if mode == "push":
            # push hard without a mallet: the command 4 cm, then 8 cm, then 12 cm past the end, each held 3 s
            for k, past in enumerate((0.04, 0.08, 0.12)):
                yield from l.move(np.array([plank_end() + past, BLOCK_Y, floor]) + BAR_C + up * GRIP_UP, t=1.0, label="push")
                yield from l.wait(3.0)
                _report(ctx, f"pushed {past * 100:.0f} cm past")
            yield from l.move(np.array([plank_end() - 0.01, BLOCK_Y, floor]) + BAR_C + up * GRIP_UP, t=1.0, label="ease off")
            yield from l.open(label="let go")
            yield from l.move(l.at() + up * 0.06 - np.array([0.03, 0.0, 0.0]), t=1.0, label="up")
            for _ in range(int(HOLD_S * 20) + 20):
                yield
            return
        if mode == "onearm":
            yield from l.open(label="let go of the block")
            yield from l.move(l.at() + up * 0.06 - np.array([0.03, 0.0, 0.0]), t=0.8, label="up")
            yield from l.move(np.array([0.20, 0.30, TOP + 0.20]), t=1.2, label="away")

    # -- right: the mallet
    st = ctx.state("mallet")
    Rm, pm = mallet_frame(st)
    grip_pt = pm + Rm @ np.array([HANDLE_GRIP, 0.0, MALLET_HEAD_C[2]])
    hyaw = math.degrees(math.atan2(Rm[1, 0], Rm[0, 0]))  # the handle's direction
    grip_r = d.down(hyaw + 180.0)  # fingers closing across the handle
    yield from r.move(grip_pt + up * 0.08, grip=grip_r, finger=FINGER_OPEN, t=2.0, label="over the mallet's handle")
    yield from r.servo(grip_pt + up * 0.08, timeout=1.0, label="over the mallet's handle")
    st = ctx.state("mallet")
    Rm, pm = mallet_frame(st)
    grip_pt = pm + Rm @ np.array([HANDLE_GRIP, 0.0, MALLET_HEAD_C[2]])
    yield from r.move(grip_pt + up * 0.001, t=1.0, label="round the handle")
    yield from r.servo(grip_pt + up * 0.001, timeout=1.0, label="round the handle")
    yield from r.close(2 * MALLET_HANDLE[2], 30.0, t=0.5, label="grip the handle")
    yield from r.move(r.at() + up * 0.05, t=0.8, label="lift the mallet")
    _report(ctx, "mallet lifted")

    def head_now():
        st = ctx.state("mallet")
        Rm, pm = mallet_frame(st)
        return pm + Rm @ MALLET_HEAD_C

    # where the head's front face should be: the block's back face, or (careless) the plank's end face
    def strike_face():
        if mode == "careless":
            return plank_end()
        R, p, _ = block_pose()
        return float((p + R @ np.array([-BLK_D, 0.0, 0.0]))[0])

    def tcp_for_head(head_target):
        """The TCP that puts the head's centre at head_target, from where the head sits in the hand now."""
        return r.at() + (np.asarray(head_target) - head_now())

    carry = HEAD_Z + 0.06
    yield from r.move(np.array([strike_face() - MALLET_HEAD_HALF - 0.05, HIT_Y, TOP + carry]) + (r.at() - head_now()), t=1.5, label="head behind the block")
    tgt = tcp_for_head((strike_face() - MALLET_HEAD_HALF - 0.04, HIT_Y, TOP + HEAD_Z))
    yield from r.move(tgt, t=1.0, label="head down level with the block's back")
    yield from r.servo(np.array([strike_face() - MALLET_HEAD_HALF - 0.04, HIT_Y, TOP + HEAD_Z]), read=head_now, timeout=1.0, label="level")
    _report(ctx, "ready to tap")

    def hold_block():
        """The left hand keeps the block on the end with a light push, following the plank as it moves."""
        while True:
            yield from l.move(np.array([plank_end() + 0.003, BLOCK_Y, floor]) + BAR_C + up * GRIP_UP, t=0.1, label="hold the block")

    taps = 0
    pushing = hold_block() if mode in ("full",) else None
    for k in range(16):
        if ctx.joint("flooring", "plank") >= GAP0 - 0.0002:
            break
        face = strike_face()
        back = tcp_for_head((face - MALLET_HEAD_HALF - TAP_BACK, HIT_Y, TOP + HEAD_Z))
        hit = tcp_for_head((face - MALLET_HEAD_HALF + TAP_PAST, HIT_Y, TOP + HEAD_Z))
        for target, t, label in ((back, 0.4, "draw back"), (hit, TAP_T, f"tap {k + 1}"), (back, 0.35, "back")):
            if pushing is not None:
                yield from ctx.together(r.move(target, t=t, label=label), _steps(pushing, t))
            else:
                yield from r.move(target, t=t, label=label)
        taps += 1
        _report(ctx, f"tap {k + 1}")
    _report(ctx, f"taps done ({taps})")
    # lay the mallet on the laid floor to the right, then both hands clear
    yield from r.move(r.at() + up * 0.05, t=0.6, label="lift the mallet")
    rest = np.array([0.24, -0.17, TOP + ROW_TOP + 0.002 + MALLET_HEAD_R]) + (r.at() - head_now())
    yield from r.move(rest + up * 0.04, t=1.5, label="over the laid floor")
    yield from r.move(rest + up * 0.004, t=0.8, label="down")
    yield from r.open(label="let go of the mallet")
    yield from r.move(r.at() + up * 0.10, t=0.8, label="up")
    if mode != "careless" and mode != "onearm":
        yield from l.open(label="let go of the block")
        yield from l.move(l.at() + up * 0.06 - np.array([0.03, 0.0, 0.0]), t=0.8, label="up")
        yield from ctx.together(l.move(np.array([0.20, 0.30, TOP + 0.20]), t=1.2, label="away"), r.move(np.array([0.20, -0.30, TOP + 0.20]), t=1.2, label="away"))
    else:
        yield from r.move(np.array([0.20, -0.30, TOP + 0.20]), t=1.2, label="away")
    _report(ctx, "clear")
    for _ in range(int(HOLD_S * 20) + 20):
        yield


def _steps(gen, seconds):
    """Run a generator for `seconds` of control steps."""
    for _ in range(max(1, round(seconds * 20))):
        next(gen)
        yield


def _punch(ctx):
    """Red team: no tools. The right hand, fingers closed, rams the plank's near end (the face above the lip) again and again."""
    import demokit as d
    from yam_arena.constants import FINGER_CLOSED

    r = ctx.right
    up = np.array([0.0, 0.0, 1.0])
    z = TOP + ZF + 0.007 + 0.013  # fingertips at mid-height of the end face

    def end():
        return X0 + ctx.joint("flooring", "plank")

    yield from r.move(np.array([end() - 0.08, HIT_Y, z + 0.06]), grip=d.down(90.0), finger=FINGER_CLOSED, t=2.0, label="in front of the end")
    yield from r.move(np.array([end() - 0.06, HIT_Y, z]), t=1.0, label="down")
    for k in range(16):
        if ctx.joint("flooring", "plank") >= GAP0 - 0.0002:
            break
        yield from r.move(np.array([end() + 0.04, HIT_Y, z]), t=TAP_T, label=f"ram {k + 1}")
        yield from r.move(np.array([end() - 0.06, HIT_Y, z]), t=0.4, label="back")
        _report(ctx, f"ram {k + 1}")
    yield from r.move(np.array([0.20, -0.30, TOP + 0.20]), t=1.5, label="away")
    for _ in range(int(HOLD_S * 20) + 20):
        yield


def _calibrate(ctx):
    """Steady pushes on the plank (no arms) to read the joint's static friction: each force held 1 s."""
    env = ctx.emb._env
    for f in (10, 20, 30, 40, 50, 60, 70, 80, 100, 120, 150):
        s0 = ctx.joint("flooring", "plank")
        m.push(env, "flooring", (float(f), 0.0, 0.0), body="plank")
        for _ in range(20):
            yield
        s1 = ctx.joint("flooring", "plank")
        with open(_DEBUG or os.devnull, "a") as fh:
            fh.write(f"# push {f} N: moved {(s1 - s0) * 1000:.3f} mm\n")
        print(f"[calibrate] push {f} N: moved {(s1 - s0) * 1000:.3f} mm", flush=True)
        m.push(env, "flooring", (0.0, 0.0, 0.0), body="plank")
        for _ in range(4):
            yield
        if s1 >= GAP0 - 0.0002:
            break
