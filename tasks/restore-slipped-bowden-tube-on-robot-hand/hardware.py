# SPDX-License-Identifier: CERN-OHL-S-2.0
"""Load pinned, licensed hand meshes without committing a CAD archive."""

import hashlib
import io
import pathlib
import tempfile
import urllib.request
import zipfile

import numpy as np

ARCHIVE_URL = 'https://github.com/opsobot/yeah-hand/releases/download/v1.3/Hierarchic.STLs.zip'
ARCHIVE_SHA256 = '5f4ee67b6990c20c5317b8dd3a414de827b0f81b188aaf5314a848bb5a31e3e4'
STL_MEMBER = 'Hierarchic STLs/(LS) Skeleton.stl'
INDEX_PHALANGES_MEMBER = 'Hierarchic STLs/Fingers/Index Phalanges.stl'
INDEX_JOINTS_MEMBER = 'Hierarchic STLs/Fingers/Index Joints and Inserts.stl'


def mesh_arrays(member):
    """Return a released STL's triangles in metres, without geometry repair.

    Preserve every source triangle, including any zero-area or disconnected
    artifacts. A combined finger mesh is not a validated articulation: callers
    must establish the moving bodies and their mechanical connections.
    """
    cache = pathlib.Path.home() / '.cache' / 'robot-hand-reference' / 'v1.3'
    cache.mkdir(parents=True, exist_ok=True)
    archive = cache / 'Hierarchic.STLs.zip'
    if not archive.exists():
        with tempfile.NamedTemporaryFile(dir=cache, suffix='.download', delete=False) as f:
            temporary = pathlib.Path(f.name)
        try:
            urllib.request.urlretrieve(ARCHIVE_URL, temporary)
            if hashlib.sha256(temporary.read_bytes()).hexdigest() != ARCHIVE_SHA256:
                raise ValueError('Reference CAD archive checksum mismatch')
            temporary.replace(archive)
        finally:
            temporary.unlink(missing_ok=True)
    data = archive.read_bytes()
    if hashlib.sha256(data).hexdigest() != ARCHIVE_SHA256:
        raise ValueError('Cached reference CAD archive checksum mismatch')
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        stl = z.read(member)
    count = int.from_bytes(stl[80:84], 'little')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    if len(stl) != 84 + 50 * count:
        raise ValueError('Expected the pinned binary STL format')
    triangles = np.frombuffer(stl, dtype=dtype, offset=84, count=count)['vertices']
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    return vertices.astype(float) * .001, inverse.reshape(-1, 3)


def skeleton_arrays():
    """Return the released static skeleton triangles in metres."""
    return mesh_arrays(STL_MEMBER)


def index_phalanges_arrays():
    """Return the final printable finger, including its tendon channels."""
    return mesh_arrays(INDEX_PHALANGES_MEMBER)


def index_joints_arrays():
    """Return the released flexible joints and anti-wear inserts together."""
    return mesh_arrays(INDEX_JOINTS_MEMBER)


def _face_components(vertices, faces):
    """Label triangles connected through any shared source vertex."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    edges = faces[:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2)
    graph = coo_matrix((np.ones(len(edges), dtype=np.uint8),
                        (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices))).tocsr()
    _, labels = connected_components(graph, directed=False)
    face_labels = labels[faces[:, 0]]
    if not np.all(labels[faces] == face_labels[:, None]):
        raise ValueError('A triangle crosses connected components')
    return face_labels


def _partition(vertices, faces, indices):
    parts = {}
    for name, ids in indices.items():
        used, inverse = np.unique(faces[ids], return_inverse=True)
        parts[name] = {'vertices': vertices[used],
                       'faces': inverse.reshape(-1, 3),
                       'source_face_indices': ids}
    return parts


def index_rigid_parts():
    """Separate the four released rigid finger pieces without repairing triangles.

    The pinned STL contains four large connected components and small detached
    artifacts. Connectivity includes shared vertices, so point-attached
    artifacts stay with their source piece. Detached artifacts are preserved
    separately; they are not another rigid finger body. This is a geometry
    partition, not an articulation or validation for collision cooking.
    """
    vertices, faces = index_phalanges_arrays()
    face_labels = _face_components(vertices, faces)
    unique, counts = np.unique(face_labels, return_counts=True)
    major = unique[counts > 1000]
    if len(major) != 4:
        raise ValueError('Pinned finger no longer has four major rigid pieces')
    # The released straight finger points along +Z: base, proximal, medial, tip.
    major = sorted(major, key=lambda label:
                   vertices[faces[face_labels == label]].mean(axis=(0, 1))[2])
    names = ('base', 'proximal', 'medial', 'distal')
    expected_counts = (4128, 93762, 55336, 123328)
    indices = {}
    for name, label, expected in zip(names, major, expected_counts, strict=True):
        ids = np.flatnonzero(face_labels == label)
        if len(ids) != expected:
            raise ValueError(f'Unexpected triangle count in pinned {name}')
        indices[name] = ids
    indices['source_artifacts'] = np.flatnonzero(~np.isin(face_labels, major))
    return _partition(vertices, faces, indices)


def index_flexible_parts():
    """Separate the three compliant joints and two tendon inserts, unchanged.

    Their geometry does not specify elastic properties. In particular, the
    distal neck must not silently become a rigid hinge at an object origin.
    """
    vertices, faces = index_joints_arrays()
    labels = _face_components(vertices, faces)
    groups = {'joints': [], 'inserts': []}
    for label in np.unique(labels):
        ids = np.flatnonzero(labels == label)
        points = vertices[faces[ids]]
        width = float(np.ptp(points[:, :, 0]))
        groups['joints' if width > .010 else 'inserts'].append(
            (float(points[:, :, 2].mean()), ids))
    if (len(groups['joints']), len(groups['inserts'])) != (3, 2):
        raise ValueError('Unexpected components in pinned joint/insert STL')
    indices = {}
    for group, names, counts in (
        ('joints', ('mcp_joint', 'pip_joint', 'dip_joint'), (1910, 876, 284)),
        ('inserts', ('proximal_insert', 'medial_insert'), (2526, 2638)),
    ):
        for (_, ids), name, count in zip(sorted(groups[group]), names, counts, strict=True):
            if len(ids) != count:
                raise ValueError(f'Unexpected triangle count in pinned {name}')
            indices[name] = ids
    return _partition(vertices, faces, indices)
