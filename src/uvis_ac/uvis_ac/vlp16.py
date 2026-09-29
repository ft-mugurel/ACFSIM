"""A VLP-16 ray pattern and hits against a flat ground and cone cylinders.

The mesh of the real track is not required to check the pattern. Cones are
upright cylinders. The lidar looks along +x, with +y left and +z up.
"""

import math

import numpy as np

ELEVATIONS_DEG = tuple(-15.0 + 2.0 * i for i in range(16))
AZIMUTH_MIN_DEG = -100.0
AZIMUTH_MAX_DEG = 100.0
AZIMUTH_STEP_DEG = 0.2
SCAN_US = 100_000.0


def rays():
    """Yield (direction xyz, ring, time_us) across one forward scan."""
    count = int(round((AZIMUTH_MAX_DEG - AZIMUTH_MIN_DEG) / AZIMUTH_STEP_DEG))
    for step in range(count + 1):
        azimuth = math.radians(AZIMUTH_MIN_DEG + step * AZIMUTH_STEP_DEG)
        # Later azimuths are later in the spin.
        time_us = (step / count) * SCAN_US
        ca, sa = math.cos(azimuth), math.sin(azimuth)
        for ring, elevation_deg in enumerate(ELEVATIONS_DEG):
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


def ranges_to_cones(origin, directions, cones, ground, walls=None, max_range=40.0):
    """Nearest range along each ray. origin and directions use x/y ground, z up."""
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
        fx = ox - cx
        fy = oy - cy
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - cone['radius'] ** 2
        disc = b * b - 4.0 * a * c
        hit = (a > 1e-8) & (disc >= 0.0)
        if not np.any(hit):
            continue
        root = np.sqrt(np.maximum(disc, 0.0))
        t = (-b - root) / (2.0 * a)
        far = (-b + root) / (2.0 * a)
        t = np.where(t > 0.05, t, far)
        z = oz + t * dz
        ok = hit & (t > 0.05) & (t < best) & (z >= cone['z0']) & (z <= cone['z0'] + cone['height'])
        best[ok] = t[ok]
        # The side test misses a beam that lands on the lid. Without this
        # the beam continues and can mark a cone behind the first one.
        if np.any(np.abs(dz) > 1e-6):
            t_cap = (cone['z0'] + cone['height'] - oz) / dz
            px = ox + t_cap * dx - cx
            py = oy + t_cap * dy - cy
            on_lid = (
                (t_cap > 0.05) & (t_cap < best)
                & (px * px + py * py <= cone['radius'] ** 2)
            )
            best[on_lid] = t_cap[on_lid]
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


def _triangle_ranges(origin, directions, tris, max_range):
    best = np.full(len(directions), np.inf)
    for start in range(0, len(tris), 48):
        chunk = tris[start:start + 48]
        v0 = chunk[:, 0]
        v1 = chunk[:, 1]
        v2 = chunk[:, 2]
        e1 = v1 - v0
        e2 = v2 - v0
        dirs = directions[None, :, :]
        e1b = e1[:, None, :]
        e2b = e2[:, None, :]
        pvec = np.cross(dirs, e2b)
        det = np.sum(e1b * pvec, axis=2)
        ok = np.abs(det) > 1e-7
        inv = np.zeros_like(det)
        inv[ok] = 1.0 / det[ok]
        tvec = np.asarray(origin, dtype=np.float32) - v0[:, None, :]
        u = np.sum(tvec * pvec, axis=2) * inv
        qvec = np.cross(tvec, e1b)
        v = np.sum(dirs * qvec, axis=2) * inv
        dist = np.sum(e2b * qvec, axis=2) * inv
        hit = ok & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (dist > 0.05) & (dist < max_range)
        dist = np.where(hit, dist, np.inf)
        best = np.minimum(best, dist.min(axis=0))
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
