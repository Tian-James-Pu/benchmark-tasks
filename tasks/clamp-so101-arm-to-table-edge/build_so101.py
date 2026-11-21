"""Build assets/so101_follower.npz: the SO-101 follower arm, posed and merged into one rigid body, from the upstream
SO-ARM100 repository (Apache-2.0) at a pinned commit.

    python build_so101.py            # downloads the URDF and STL meshes into ~/.cache/so-arm100/<commit>/, writes the npz

What it does:
- reads Simulation/SO101/so101_new_calib.urdf (link masses, centres of mass, inertias, joint frames) and runs forward
  kinematics at the resting pose POSE (radians, the URDF's joint convention);
- puts every visual STL of every link in the base_link frame (metres; base_link's origin is on the underside of the
  base, the base's flat back face at x = -22 mm, its arm reaching towards +x);
- decimates each mesh (quadric, open3d) to at most MAX_FACES faces for display;
- takes the convex hull of each link's meshes (decimated further) as that link's collider;
- sums the links' masses, centres of mass and inertias (parallel-axis) into one rigid body.

The arm is posed and treated as rigid because its six STS3215 servos have 1:345 gearing and are not powered while the
arm is mounted (README, "Simulation").
"""

from __future__ import annotations

import io
import json
import pathlib
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np

COMMIT = "a758567c3978dfeefe282ede0500085a48fe8f78"
RAW = f"https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/{COMMIT}/Simulation/SO101"
CACHE = pathlib.Path("~/.cache/so-arm100").expanduser() / COMMIT
OUT = pathlib.Path(__file__).resolve().parent / "assets" / "so101_follower.npz"
# The arm unpowered and laid forward, as it is set down before mounting: upper arm tipped forward 1.2 rad, forearm 0.19 rad
# back, wrist 1.2 rad down, gripper closed, its jaw's tip resting on the table 0.3 m in front of the base. Its centre of
# mass is 171 mm forward of base_link's origin and 63 mm up, between the base and the jaw's tip; nothing of it overhangs
# the base's back face, and nothing within 120 mm of that face stands more than 137 mm high.
POSE = {"shoulder_pan": 0.0, "shoulder_lift": 1.20, "elbow_flex": -0.19, "wrist_flex": -1.20, "wrist_roll": 0.0, "gripper": 0.0}
MAX_FACES = 2500
HULL_POINTS = 60  # PhysX cooks convex hulls of at most 255 vertices; fewer keeps contact cheap


def small_hull(pts: np.ndarray):
    """The convex hull of ``pts``, reduced to about HULL_POINTS of its own vertices (farthest-point sampling)."""
    import trimesh
    v = np.asarray(trimesh.convex.convex_hull(pts).vertices, float)
    if len(v) > HULL_POINTS:
        pick = [int(np.argmax(v[:, 2]))]
        d = np.linalg.norm(v - v[pick[0]], axis=1)
        while len(pick) < HULL_POINTS:
            pick.append(int(np.argmax(d)))
            d = np.minimum(d, np.linalg.norm(v - v[pick[-1]], axis=1))
        v = v[pick]
    h = trimesh.convex.convex_hull(v)
    return np.asarray(h.vertices, float), np.asarray(h.faces, np.int32)


def fetch(rel: str) -> pathlib.Path:
    path = CACHE / rel
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(f"{RAW}/{rel}", timeout=60) as r:
            path.write_bytes(r.read())
    return path


def read_stl(path: pathlib.Path) -> np.ndarray:
    b = path.read_bytes()
    n = int(np.frombuffer(b[80:84], np.uint32)[0])
    rec = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(float)  # (n, 3, 3), metres


def rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def tf(xyz, rpy_) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rpy(*rpy_)
    t[:3, 3] = xyz
    return t


def axis_angle(axis, a) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    t = np.eye(4)
    t[:3, :3] = np.eye(3) + np.sin(a) * kx + (1 - np.cos(a)) * kx @ kx
    return t


def _vals(el, attr, default="0 0 0"):
    return [float(x) for x in (el.get(attr, default) if el is not None else default).split()]


def decimate(tri: np.ndarray, faces: int):
    import open3d as o3d
    v = tri.reshape(-1, 3)
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v), o3d.utility.Vector3iVector(np.arange(len(v)).reshape(-1, 3)))
    m.merge_close_vertices(1e-6)
    m.remove_duplicated_triangles()
    m.remove_degenerate_triangles()
    if len(m.triangles) > faces:
        m = m.simplify_quadric_decimation(faces)
    return np.asarray(m.vertices, float), np.asarray(m.triangles, np.int32)


def main() -> None:
    urdf = ET.parse(fetch("so101_new_calib.urdf")).getroot()
    links = {lk.get("name"): lk for lk in urdf.findall("link")}
    joints = urdf.findall("joint")
    child_joint = {j.find("child").get("link"): j for j in joints}
    world = {"base_link": np.eye(4)}

    def pose_of(name):
        if name not in world:
            j = child_joint[name]
            o = j.find("origin")
            t = pose_of(j.find("parent").get("link")) @ tf(_vals(o, "xyz"), _vals(o, "rpy"))
            if j.get("type") == "revolute":
                t = t @ axis_angle(_vals(j.find("axis"), "xyz", "0 0 1"), POSE.get(j.get("name"), 0.0))
            world[name] = t
        return world[name]

    visuals, hulls = [], []
    mass, mc, inertia_terms = 0.0, np.zeros(3), []
    import trimesh
    for name, lk in links.items():
        tw = pose_of(name)
        inr = lk.find("inertial")
        if inr is not None:
            m = float(inr.find("mass").get("value"))
            if m > 1e-6:
                o = inr.find("origin")
                ti = tw @ tf(_vals(o, "xyz"), _vals(o, "rpy"))
                i = inr.find("inertia")
                ii = np.array([[float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
                               [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
                               [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]])
                inertia_terms.append((m, ti[:3, 3], ti[:3, :3] @ ii @ ti[:3, :3].T))
                mass += m
                mc += m * ti[:3, 3]
        link_pts = []
        for k, vis in enumerate(lk.findall("visual")):
            fn = vis.find("geometry/mesh").get("filename")
            o = vis.find("origin")
            t = tw @ tf(_vals(o, "xyz"), _vals(o, "rpy"))
            tri = read_stl(fetch(fn))
            tri = tri @ t[:3, :3].T + t[:3, 3]
            v, f = decimate(tri, MAX_FACES)
            kind = "servo" if "sts3215" in fn else "printed"
            visuals.append((f"{name}_{k}", kind, v, f))
            link_pts.append(tri.reshape(-1, 3))
        if name == "base_link":  # the base is concave: its two flat wings and its tower collide separately
            pts = np.concatenate(link_pts)
            for hname, keep in (("base_wing_l", (pts[:, 1] > 0.02) & (pts[:, 2] < 0.0152)),
                                ("base_wing_r", (pts[:, 1] < -0.02) & (pts[:, 2] < 0.0152)),
                                ("base_tower", np.abs(pts[:, 1]) < 0.0225)):
                hulls.append((hname, *small_hull(pts[keep])))
        else:  # one hull per mesh: a link's servo and its printed part are each nearly convex
            for k, pts in enumerate(link_pts):
                hulls.append((f"{name}_{k}", *small_hull(pts)))
    com = mc / mass
    inertia = np.zeros((3, 3))
    for m, c, i in inertia_terms:
        d = c - com
        inertia += i + m * (d @ d * np.eye(3) - np.outer(d, d))
    data = {"mass": mass, "com": com, "inertia": inertia, "pose": json.dumps(POSE), "commit": COMMIT}
    for k, (n, kind, v, f) in enumerate(visuals):
        data[f"vis{k}_v"], data[f"vis{k}_f"] = v.astype(np.float32), f
        data[f"vis{k}_meta"] = np.array([n, kind])
    for k, (n, v, f) in enumerate(hulls):
        data[f"hull{k}_v"], data[f"hull{k}_f"] = v.astype(np.float32), f
        data[f"hull{k}_meta"] = np.array([n])
    buf = io.BytesIO()
    np.savez_compressed(buf, **data)
    OUT.write_bytes(buf.getvalue())
    allv = np.concatenate([v for _, _, v, _ in visuals])
    print(f"{OUT.name}: {len(visuals)} meshes, {sum(len(f) for *_, f in visuals)} faces, {len(hulls)} hulls, "
          f"mass {mass:.3f} kg, com {np.round(com * 1000, 1)} mm, bounds {np.round(allv.min(0) * 1000)} .. {np.round(allv.max(0) * 1000)} mm, "
          f"{OUT.stat().st_size // 1024} KB")


if __name__ == "__main__":
    main()
