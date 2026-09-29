// Same cone, ground, and wall tests as vlp16.ranges_to_cones.
// One pass per cone and per triangle, with no temporary ray masks.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace {

struct Vec3 {
    double x, y, z;
};

inline Vec3 operator-(Vec3 a, Vec3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline bool box_hit(Vec3 origin, Vec3 dir, Vec3 low, Vec3 high, double max_range) {
    double inv[3], t0[3], t1[3];
    const double d[3] = {dir.x, dir.y, dir.z};
    const double o[3] = {origin.x, origin.y, origin.z};
    const double lo[3] = {low.x, low.y, low.z};
    const double hi[3] = {high.x, high.y, high.z};
    for (int k = 0; k < 3; ++k) {
        double safe = std::fabs(d[k]) < 1e-8 ? std::copysign(1e-8, d[k]) : d[k];
        inv[k] = 1.0 / safe;
        t0[k] = (lo[k] - o[k]) * inv[k];
        t1[k] = (hi[k] - o[k]) * inv[k];
    }
    double tmin = std::max(std::max(std::min(t0[0], t1[0]), std::min(t0[1], t1[1])), std::min(t0[2], t1[2]));
    double tmax = std::min(std::min(std::max(t0[0], t1[0]), std::max(t0[1], t1[1])), std::max(t0[2], t1[2]));
    return tmax >= std::max(tmin, 0.0) && tmin < max_range && tmax > 0.05;
}

void cast_triangles(
    Vec3 origin, const double* dirs, const double* best, int* keep, int keep_n,
    const float* tris, int tri_n, double max_range, double* out) {
    for (int start = 0; start < tri_n; start += 48) {
        int count = std::min(48, tri_n - start);
        Vec3 low{1e300, 1e300, 1e300};
        Vec3 high{-1e300, -1e300, -1e300};
        for (int t = 0; t < count; ++t) {
            const float* p = tris + (start + t) * 9;
            for (int v = 0; v < 3; ++v) {
                low.x = std::min(low.x, static_cast<double>(p[v * 3]));
                low.y = std::min(low.y, static_cast<double>(p[v * 3 + 1]));
                low.z = std::min(low.z, static_cast<double>(p[v * 3 + 2]));
                high.x = std::max(high.x, static_cast<double>(p[v * 3]));
                high.y = std::max(high.y, static_cast<double>(p[v * 3 + 1]));
                high.z = std::max(high.z, static_cast<double>(p[v * 3 + 2]));
            }
        }
        low.x -= 0.05;
        low.y -= 0.05;
        low.z -= 0.05;
        high.x += 0.05;
        high.y += 0.05;
        high.z += 0.05;
        for (int k = 0; k < keep_n; ++k) {
            int i = keep[k];
            Vec3 dir{dirs[i * 3], dirs[i * 3 + 1], dirs[i * 3 + 2]};
            if (!box_hit(origin, dir, low, high, max_range)) {
                continue;
            }
            double nearest = out[i];
            for (int t = 0; t < count; ++t) {
                const float* p = tris + (start + t) * 9;
                Vec3 v0{p[0], p[1], p[2]};
                Vec3 v1{p[3], p[4], p[5]};
                Vec3 v2{p[6], p[7], p[8]};
                Vec3 e1 = v1 - v0;
                Vec3 e2 = v2 - v0;
                Vec3 pvec = cross(dir, e2);
                double det = dot(e1, pvec);
                if (std::fabs(det) <= 1e-7) {
                    continue;
                }
                double inv = 1.0 / det;
                Vec3 tvec = origin - v0;
                double u = dot(tvec, pvec) * inv;
                Vec3 qvec = cross(tvec, e1);
                double v = dot(dir, qvec) * inv;
                double dist = dot(e2, qvec) * inv;
                if (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && dist > 0.05 && dist < max_range && dist < nearest) {
                    nearest = dist;
                }
            }
            out[i] = nearest;
        }
        (void)best;
    }
}

}  // namespace

extern "C" void lidar_cast(
    const double* origin, const double* dirs, int n,
    const double* cones, int cones_n, double ground,
    const double* cell_xy, const int* cell_off, const float* tris, int cells,
    double max_range, double* out) {
    const double ox = origin[0];
    const double oy = origin[1];
    const double oz = origin[2];
    const double reach = max_range + 2.0;
    const double reach2 = reach * reach;
    for (int i = 0; i < n; ++i) {
        double dz = dirs[i * 3 + 2];
        out[i] = INFINITY;
        if (dz < -1e-4) {
            double t = (ground - oz) / dz;
            if (t > 0.2 && t < max_range) {
                out[i] = t;
            }
        }
    }

    std::vector<double> horiz(static_cast<std::size_t>(n) * 2);
    for (int i = 0; i < n; ++i) {
        double dx = dirs[i * 3];
        double dy = dirs[i * 3 + 1];
        double h = std::sqrt(dx * dx + dy * dy);
        if (h < 1e-6) {
            h = 1e-6;
        }
        horiz[i * 2] = dx / h;
        horiz[i * 2 + 1] = dy / h;
    }

    for (int c = 0; c < cones_n; ++c) {
        double cx = cones[c * 5];
        double cy = cones[c * 5 + 1];
        double radius = cones[c * 5 + 2];
        double z0 = cones[c * 5 + 3];
        double height = cones[c * 5 + 4];
        double vx = cx - ox;
        double vy = cy - oy;
        if (vx * vx + vy * vy >= reach2) {
            continue;
        }
        double dist = std::hypot(vx, vy);
        double aim_r = radius + 0.08;
        bool all = dist <= aim_r;
        double limit = -2.0;
        if (!all && dist > 0.0) {
            double sin_t = std::min(1.0, aim_r / dist);
            limit = std::cos(std::asin(sin_t) + 0.5 * M_PI / 180.0);
        }
        double fx = ox - cx;
        double fy = oy - cy;
        double radius2 = radius * radius;
        double z1 = z0 + height;
        for (int i = 0; i < n; ++i) {
            double dx = dirs[i * 3];
            double dy = dirs[i * 3 + 1];
            double dz = dirs[i * 3 + 2];
            if (!all) {
                double align = (horiz[i * 2] * vx + horiz[i * 2 + 1] * vy) / dist;
                if (align < limit) {
                    continue;
                }
            }
            double a = dx * dx + dy * dy;
            double best = out[i];
            if (a > 1e-8) {
                double b = 2.0 * (fx * dx + fy * dy);
                double disc = b * b - 4.0 * a * (fx * fx + fy * fy - radius2);
                if (disc >= 0.0) {
                    double root = std::sqrt(disc);
                    double t = (-b - root) / (2.0 * a);
                    if (t <= 0.05) {
                        t = (-b + root) / (2.0 * a);
                    }
                    if (t > 0.05 && t < best) {
                        double z = oz + t * dz;
                        if (z >= z0 && z <= z1) {
                            best = t;
                        }
                    }
                }
            }
            if (std::fabs(dz) > 1e-6) {
                double t_cap = (z1 - oz) / dz;
                if (t_cap > 0.05 && t_cap < best) {
                    double px = ox + t_cap * dx - cx;
                    double py = oy + t_cap * dy - cy;
                    if (px * px + py * py <= radius2) {
                        best = t_cap;
                    }
                }
            }
            out[i] = best;
        }
    }

    std::vector<int> keep;
    keep.reserve(static_cast<std::size_t>(n));
    Vec3 org{ox, oy, oz};
    for (int cell = 0; cell < cells; ++cell) {
        double cx = cell_xy[cell * 2];
        double cy = cell_xy[cell * 2 + 1];
        double vx = cx - ox;
        double vy = cy - oy;
        double dist = std::hypot(vx, vy);
        if (dist > max_range + 8.0) {
            continue;
        }
        keep.clear();
        if (dist < 6.0) {
            for (int i = 0; i < n; ++i) {
                keep.push_back(i);
            }
        } else {
            double inv = 1.0 / dist;
            double limit = 6.0 / dist;
            limit = 1.0 - 0.5 * limit * limit;
            for (int i = 0; i < n; ++i) {
                double align = horiz[i * 2] * vx * inv + horiz[i * 2 + 1] * vy * inv;
                if (align > limit) {
                    keep.push_back(i);
                }
            }
        }
        if (keep.empty()) {
            continue;
        }
        int begin = cell_off[cell];
        int end = cell_off[cell + 1];
        cast_triangles(org, dirs, nullptr, keep.data(), static_cast<int>(keep.size()),
                       tris + begin * 9, end - begin, max_range, out);
    }
    for (int i = 0; i < n; ++i) {
        if (out[i] > max_range) {
            out[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
}
