"""A VLP-16 ray pattern and hits against a flat ground and cone cylinders.

The mesh of the real track is not required to check the pattern. Cones are
upright cylinders. The lidar looks along +x, with +y left and +z up.
"""

import ctypes
import math
from pathlib import Path

import numpy as np

ELEVATIONS_DEG = tuple(-15.0 + 2.0 * i for i in range(16))
AZIMUTH_MIN_DEG = -100.0
AZIMUTH_MAX_DEG = 100.0
AZIMUTH_STEP_DEG = 0.2
SCAN_US = 100_000.0


def rays(lasers=16, fov_upper=15.0, fov_lower=-15.0,
         azimuth_start=AZIMUTH_MIN_DEG, azimuth_end=AZIMUTH_MAX_DEG,
         azimuth_step=AZIMUTH_STEP_DEG):
    """Yield (direction xyz, ring, time_us) across one forward scan."""
    if lasers < 1:
        raise ValueError('a lidar needs at least one laser')
    if lasers == 1:
        elevations = (0.5 * (fov_lower + fov_upper),)
    else:
        span = (fov_upper - fov_lower) / (lasers - 1)
        elevations = tuple(fov_lower + span * i for i in range(lasers))
    count = int(round((azimuth_end - azimuth_start) / azimuth_step))
    for step in range(count + 1):
        azimuth = math.radians(azimuth_start + step * azimuth_step)
        # Later azimuths are later in the spin.
        time_us = (step / count) * SCAN_US
        ca, sa = math.cos(azimuth), math.sin(azimuth)
        for ring, elevation_deg in enumerate(elevations):
            elevation = math.radians(elevation_deg)
            ce, se = math.cos(elevation), math.sin(elevation)
            yield (ca * ce, sa * ce, se), ring, time_us


def hit_ground(direction, origin_z):
    dx, dy, dz = direction
    if dz >= -1e-6:
        return None
    t = -origin_z / dz
    if t <= 0.0:
        return None
    return (t * dx, t * dy, origin_z + t * dz)


def hit_cylinder(direction, origin, center_xy, radius, height, z0=0.0):
    ox, oy, oz = origin
    dx, dy, dz = direction
    cx, cy = center_xy
    a = dx * dx + dy * dy
    if a < 1e-12:
        return None
    fx, fy = ox - cx, oy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    t = (-b - root) / (2.0 * a)
    if t <= 0.05:
        t = (-b + root) / (2.0 * a)
    if t <= 0.05:
        return None
    z = oz + t * dz
    if z < z0 or z > z0 + height:
        return None
    return t


_CAST = None
_PACK = {}


def _cast_lib():
    global _CAST
    if _CAST is not None:
        return _CAST
    path = Path(__file__).with_name('_lidar_cast.so')
    if not path.is_file():
        return None
    lib = ctypes.CDLL(str(path))
    lib.lidar_cast.argtypes = [
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
        ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_double,
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int32),
        ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_double,
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.lidar_cast.restype = None
    _CAST = lib
    return lib


def _packed_scene(cones, walls):
    key = (id(cones), id(walls) if walls is not None else 0)
    cached = _PACK.get(key)
    if cached is not None:
        return cached
    cone_a = np.array(
        [[*cone['xy'], cone['radius'], cone['z0'], cone['height']] for cone in cones],
        dtype=np.float64,
    )
    if not walls:
        cell_xy = np.zeros((0, 2), dtype=np.float64)
        cell_off = np.zeros(1, dtype=np.int32)
        tris = np.zeros(0, dtype=np.float32)
    else:
        cell_xy = np.array([[cx, cy] for cx, cy, _tris in walls], dtype=np.float64)
        parts = [tris.astype(np.float32, copy=False).reshape(-1) for _cx, _cy, tris in walls]
        tris = np.concatenate(parts) if parts else np.zeros(0, dtype=np.float32)
        counts = [len(chunk) for _cx, _cy, chunk in walls]
        cell_off = np.zeros(len(counts) + 1, dtype=np.int32)
        cell_off[1:] = np.cumsum(counts)
    _PACK.clear()
    _PACK[key] = (cone_a, cell_xy, cell_off, tris)
    return _PACK[key]


def _native_ranges(origin, directions, cones, ground, walls, max_range):
    lib = _cast_lib()
    flat = np.ascontiguousarray(directions, dtype=np.float64)
    origin_a = np.ascontiguousarray(origin, dtype=np.float64)
    cone_a, cell_xy, cell_off, tris = _packed_scene(cones, walls)
    out = np.empty(len(flat), dtype=np.float64)
    lib.lidar_cast(
        origin_a.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        flat.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), len(flat),
        cone_a.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), len(cone_a), float(ground),
        cell_xy.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        cell_off.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
        tris.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), len(cell_xy),
        float(max_range), out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )
    return out


def ranges_to_cones(origin, directions, cones, ground, walls=None, max_range=40.0):
    """Nearest range along each ray. origin and directions use x/y ground, z up."""
    if _cast_lib() is not None:
        return _native_ranges(origin, directions, cones, ground, walls, max_range)
    ox, oy, oz = origin
    dx = directions[:, 0]
    dy = directions[:, 1]
    dz = directions[:, 2]
    best = np.full(dx.shape, np.inf, dtype=np.float64)
    down = dz < -1e-4
    ground_t = np.full(dx.shape, np.inf)
    ground_t[down] = (ground - oz) / dz[down]
    ground_ok = (ground_t > 0.2) & (ground_t < max_range)
    best[ground_ok] = ground_t[ground_ok]
    near = [
        cone for cone in cones
        if (cone['xy'][0] - ox) ** 2 + (cone['xy'][1] - oy) ** 2 < (max_range + 2.0) ** 2
    ]
    for cone in near:
        cx, cy = cone['xy']
        aim = _aimed_at(dx, dy, ox, oy, cx, cy, cone['radius'] + 0.08)
        if not np.any(aim):
            continue
        idx = np.flatnonzero(aim)
        sx, sy, sz = dx[idx], dy[idx], dz[idx]
        fx = ox - cx
        fy = oy - cy
        a = sx * sx + sy * sy
        b = 2.0 * (fx * sx + fy * sy)
        c = fx * fx + fy * fy - cone['radius'] ** 2
        disc = b * b - 4.0 * a * c
        hit = (a > 1e-8) & (disc >= 0.0)
        if np.any(hit):
            root = np.sqrt(np.maximum(disc, 0.0))
            t = (-b - root) / (2.0 * a)
            far = (-b + root) / (2.0 * a)
            t = np.where(t > 0.05, t, far)
            z = oz + t * sz
            ok = hit & (t > 0.05) & (t < best[idx]) & (z >= cone['z0']) & (z <= cone['z0'] + cone['height'])
            best[idx[ok]] = t[ok]
        # The side test misses a beam that lands on the lid. Without this
        # the beam continues and can mark a cone behind the first one.
        if np.any(np.abs(sz) > 1e-6):
            t_cap = (cone['z0'] + cone['height'] - oz) / sz
            px = ox + t_cap * sx - cx
            py = oy + t_cap * sy - cy
            on_lid = (
                (t_cap > 0.05) & (t_cap < best[idx])
                & (px * px + py * py <= cone['radius'] ** 2)
            )
            best[idx[on_lid]] = t_cap[on_lid]
    best = _ranges_to_triangles(origin, directions, walls, best, max_range)
    best[best > max_range] = np.nan
    return best


def grid_walls(tris, cell=8.0):
    if tris is None or len(tris) == 0:
        return []
    center = tris.mean(axis=1)
    buckets = {}
    for index, (x, y) in enumerate(center[:, :2]):
        key = (int(x / cell), int(y / cell))
        buckets.setdefault(key, []).append(index)
    return [
        ((key[0] + 0.5) * cell, (key[1] + 0.5) * cell, tris[np.array(ids)])
        for key, ids in buckets.items()
    ]


def _ranges_to_triangles(origin, directions, cells, best, max_range):
    if not cells:
        return best
    ox, oy = origin[0], origin[1]
    dx = directions[:, 0]
    dy = directions[:, 1]
    horiz = np.maximum(np.sqrt(dx * dx + dy * dy), 1e-6)
    hx, hy = dx / horiz, dy / horiz
    for cx, cy, tris in cells:
        vx, vy = cx - ox, cy - oy
        dist = math.hypot(vx, vy)
        if dist > max_range + 8.0:
            continue
        if dist < 6.0:
            chosen = directions
            chosen_index = None
        else:
            align = hx * (vx / dist) + hy * (vy / dist)
            limit = 6.0 / dist
            mask = align > (1.0 - 0.5 * limit * limit)
            if not np.any(mask):
                continue
            chosen = directions[mask]
            chosen_index = np.flatnonzero(mask)
        hit = _triangle_ranges(origin, chosen, tris, max_range)
        if chosen_index is None:
            best = np.minimum(best, hit)
        else:
            best[chosen_index] = np.minimum(best[chosen_index], hit)
    return best


def _aimed_at(dx, dy, ox, oy, cx, cy, radius):
    """Rays whose horizontal direction can meet a vertical circle."""
    vx, vy = cx - ox, cy - oy
    dist = math.hypot(vx, vy)
    if dist <= radius:
        return np.ones(dx.shape, dtype=bool)
    sin_t = min(1.0, radius / dist)
    limit = math.cos(math.asin(sin_t) + math.radians(0.5))
    horiz = np.maximum(np.sqrt(dx * dx + dy * dy), 1e-6)
    align = (dx * vx + dy * vy) / (horiz * dist)
    return align >= limit


def _box_rays(origin, directions, low, high, max_range):
    """Rays that enter an axis-aligned box before max_range."""
    safe = np.where(np.abs(directions) < 1e-8, np.copysign(1e-8, directions), directions)
    inv = 1.0 / safe
    t0 = (low - origin) * inv
    t1 = (high - origin) * inv
    tmin = np.minimum(t0, t1).max(axis=1)
    tmax = np.maximum(t0, t1).min(axis=1)
    return (tmax >= np.maximum(tmin, 0.0)) & (tmin < max_range) & (tmax > 0.05)


def _triangle_ranges(origin, directions, tris, max_range):
    best = np.full(len(directions), np.inf)
    origin = np.asarray(origin, dtype=np.float64)
    for start in range(0, len(tris), 48):
        chunk = tris[start:start + 48]
        low = chunk.min(axis=(0, 1)) - 0.05
        high = chunk.max(axis=(0, 1)) + 0.05
        aim = _box_rays(origin, directions, low, high, max_range)
        if not np.any(aim):
            continue
        chosen = directions[aim]
        v0 = chunk[:, 0]
        v1 = chunk[:, 1]
        v2 = chunk[:, 2]
        e1 = v1 - v0
        e2 = v2 - v0
        dirs = chosen[None, :, :]
        e1b = e1[:, None, :]
        e2b = e2[:, None, :]
        pvec = np.cross(dirs, e2b)
        det = np.sum(e1b * pvec, axis=2)
        ok = np.abs(det) > 1e-7
        inv = np.zeros_like(det)
        inv[ok] = 1.0 / det[ok]
        tvec = origin.astype(np.float32) - v0[:, None, :]
        u = np.sum(tvec * pvec, axis=2) * inv
        qvec = np.cross(tvec, e1b)
        v = np.sum(dirs * qvec, axis=2) * inv
        dist = np.sum(e2b * qvec, axis=2) * inv
        hit = ok & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (dist > 0.05) & (dist < max_range)
        dist = np.where(hit, dist, np.inf)
        best[aim] = np.minimum(best[aim], dist.min(axis=0))
    return best


def scan(origin, cones, max_range=60.0):
    """Return hits as (x, y, z, ring, time_us). The nearest surface wins."""
    hits = []
    ox, oy, oz = origin
    for direction, ring, time_us in rays():
        best = None
        best_d = max_range
        ground = hit_ground(direction, oz)
        if ground is not None:
            dist = math.dist((ox, oy, oz), (ox + ground[0], oy + ground[1], ground[2]))
            if dist < best_d:
                best, best_d = (ox + ground[0], oy + ground[1], ground[2]), dist
        for cone in cones:
            t = hit_cylinder(
                direction, origin, cone['xy'], cone['radius'], cone['height'],
                cone.get('z0', 0.0))
            if t is None or t >= best_d:
                continue
            best = (ox + t * direction[0], oy + t * direction[1], oz + t * direction[2])
            best_d = t
        if best is not None:
            hits.append((best[0], best[1], best[2], ring, time_us))
    return hits
