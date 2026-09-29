"""Read cone positions out of an Assetto Corsa KN5 track model."""

import struct
from pathlib import Path

import numpy as np


def _str(f):
    size = struct.unpack('<i', f.read(4))[0]
    return f.read(size).decode('utf-8', 'replace')


def _skip_materials(f, version):
    names = []
    count = struct.unpack('<i', f.read(4))[0]
    for _ in range(count):
        name = _str(f)
        shader = _str(f)
        f.read(2)  # short
        if version > 4:
            f.read(4)
        props = struct.unpack('<i', f.read(4))[0]
        diffuse = ''
        for _p in range(props):
            prop = _str(f)
            value = struct.unpack('<f', f.read(4))[0]
            f.read(36)
            if prop == 'ksDiffuse':
                pass
            del value
        textures = struct.unpack('<i', f.read(4))[0]
        for _t in range(textures):
            sample = _str(f)
            f.read(4)
            tex = _str(f)
            if sample == 'txDiffuse':
                diffuse = tex
        names.append((name + ' ' + shader + ' ' + diffuse).lower())
    return names


def _transform(matrix, x, y, z):
    return (
        matrix[0][0] * x + matrix[1][0] * y + matrix[2][0] * z + matrix[3][0],
        matrix[0][1] * x + matrix[1][1] * y + matrix[2][1] * z + matrix[3][1],
        matrix[0][2] * x + matrix[1][2] * y + matrix[2][2] * z + matrix[3][2],
    )


def _mul(local, parent):
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(local[i][k] * parent[k][j] for k in range(4))
    return out


def _ident():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matrix(f):
    vals = struct.unpack('<16f', f.read(64))
    return [list(vals[i * 4:(i + 1) * 4]) for i in range(4)]


def _clusters(xs, ys, zs):
    """One cylinder per cone. A mesh may contain many cones."""
    cell = 0.45
    buckets = {}
    for x, y, z in zip(xs, ys, zs):
        key = (int(x / cell), int(z / cell))
        buckets.setdefault(key, []).append((x, y, z))
    cones = []
    for points in buckets.values():
        if len(points) < 6:
            continue
        arr = np.asarray(points)
        height = float(arr[:, 1].max() - arr[:, 1].min())
        radius = float(max(arr[:, 0].max() - arr[:, 0].min(), arr[:, 2].max() - arr[:, 2].min()) * 0.5)
        if height < 0.15 or height > 0.9 or radius < 0.04 or radius > 0.4:
            continue
        cones.append({
            'xy': (float(arr[:, 0].mean()), float(arr[:, 2].mean())),
            'z0': float(arr[:, 1].min()),
            'height': height,
            'radius': max(0.08, radius),
        })
    return cones


def cone_cylinders(path, name_hint='cone'):
    """Return dicts xy, radius, height, z0 in Assetto Corsa coordinates."""
    hint = name_hint.lower()
    cones = []
    triangles = []
    with open(path, 'rb') as f:
        magic = f.read(6)
        if magic != b'sc6969':
            raise ValueError('not a KN5 file: %s' % path)
        version = struct.unpack('<i', f.read(4))[0]
        if version > 5:
            f.read(4)
        textures = struct.unpack('<i', f.read(4))[0]
        for _ in range(textures):
            f.read(4)
            _str(f)
            size = struct.unpack('<i', f.read(4))[0]
            f.seek(size, 1)
        materials = _skip_materials(f, version)

        def walk(parent):
            node_type = struct.unpack('<i', f.read(4))[0]
            name = _str(f)
            children = struct.unpack('<i', f.read(4))[0]
            f.read(1)
            world = parent
            if node_type == 1:
                local = _matrix(f)
                world = _mul(local, parent)
            elif node_type == 2:
                f.read(3)
                vcount = struct.unpack('<i', f.read(4))[0]
                raw = f.read(vcount * 44)
                icount = struct.unpack('<i', f.read(4))[0]
                index_bytes = f.read(icount * 2)
                mat = struct.unpack('<i', f.read(4))[0]
                f.seek(29, 1)
                mat_name = materials[mat] if 0 <= mat < len(materials) else ''
                lname = name.lower()
                keep = hint in lname or hint in mat_name
                solid = lname.startswith('wall') or 'barrier' in lname or 'fence' in lname
                if keep or solid:
                    pts = np.empty((vcount, 3), dtype=np.float64)
                    for vi in range(vcount):
                        x, y, z = struct.unpack_from('<3f', raw, vi * 44)
                        pts[vi] = _transform(parent, x, y, z)
                    if keep:
                        cones.extend(_clusters(pts[:, 0], pts[:, 1], pts[:, 2]))
                    if solid and icount >= 3:
                        idx = np.frombuffer(index_bytes, dtype='<u2').reshape(-1, 3)
                        # Flat frame: x, forward z, up y.
                        tri = np.empty((len(idx), 3, 3), dtype=np.float32)
                        vert = pts[idx]
                        tri[:, :, 0] = vert[:, :, 0]
                        tri[:, :, 1] = vert[:, :, 2]
                        tri[:, :, 2] = vert[:, :, 1]
                        triangles.append(tri)
            elif node_type == 3:
                f.read(3)
                bones = struct.unpack('<i', f.read(4))[0]
                for _b in range(bones):
                    _str(f)
                    f.seek(64, 1)
                vcount = struct.unpack('<i', f.read(4))[0]
                f.seek(vcount * (12 + 12 + 8 + 44), 1)
                icount = struct.unpack('<i', f.read(4))[0]
                f.seek(icount * 2 + 4 + 12, 1)
            else:
                raise ValueError('unknown KN5 node %s named %s' % (node_type, name))
            for _c in range(children):
                walk(world)

        walk(_ident())
    walls = np.concatenate(triangles) if triangles else np.zeros((0, 3, 3), np.float32)
    return cones, walls
