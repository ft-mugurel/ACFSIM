#!/usr/bin/env python3
"""Build empty Assetto Corsa tracks from the EUFS cone CSVs.

Each track is an asphalt plane and one movable cone per cone. The car spawn
is the CSV car_start, rotated so that direction faces the game's +Z.
AC_START_0, AC_PIT_0, and AC_HOTLAP_START_0 sit on that point. There is
no scenery. Cone colors match the CSV: blue on the left, yellow on the
right, orange and big orange where the file says so.

EUFS is x forward, y left, z up. Yaw 0 faces +x and a positive yaw turns
left. Assetto is y-up: x is left, z is forward, and heading 0 faces +z.
The map used by the bridge is the reverse of that, so a CSV point becomes:

    forward = cos(yaw) * dx + sin(yaw) * dy
    left    = -sin(yaw) * dx + cos(yaw) * dy
    ac_x, ac_y, ac_z = left, up, forward

Run from the repo:

    python3 src/assetto-corsa/src/uvis_ac/tools/eufs_to_ac.py

Tracks land in the game's content/tracks as uvis_<name>. Pick one in a
practice session, then restart the AC bridge so the lidar reads the new
model and the map origin is taken again.
"""

import argparse
import csv
import math
import shutil
import struct
import sys
import zlib
from pathlib import Path


# Same heights and collision radius as the Gazebo worlds in world_from_csv.
HEIGHT = {
    'blue': 0.325,
    'yellow': 0.325,
    'orange': 0.325,
    'big_orange': 0.505,
}
RADIUS = 0.11
TAGS = ('blue', 'yellow', 'orange', 'big_orange')

# OpenCV hue is degrees/2. These RGB values stay inside the cone classifier
# after DXT1 rounding: yellow 24-42, blue 96-135, orange 4-23. Assetto's
# exposure lifts the orange texture to about hue 22; yellow stays near 26.
COLOR = {
    'blue': (0, 40, 255),
    'yellow': (255, 220, 0),
    'orange': (255, 80, 0),
    'big_orange': (255, 80, 0),
}

# One asphalt tile covers this many metres. The road UVs repeat it.
_ASPHALT_TILE_M = 4.0
# Keep the collider base just clear of the road so a cone is not born inside it.
_CONE_LIFT_M = 0.01


class Bin:
    def __init__(self):
        self.buf = bytearray()

    def raw(self, data):
        self.buf.extend(data)

    def i32(self, value):
        self.buf.extend(struct.pack('<i', int(value)))

    def u32(self, value):
        self.buf.extend(struct.pack('<I', int(value)))

    def u16(self, value):
        self.buf.extend(struct.pack('<H', int(value)))

    def f32(self, value):
        self.buf.extend(struct.pack('<f', float(value)))

    def u8(self, value):
        self.buf.extend(struct.pack('<B', int(value)))

    def string(self, text):
        raw = text.encode('ascii')
        self.i32(len(raw))
        self.raw(raw)


def _rgb565(red, green, blue):
    return ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)


def solid_dds(red, green, blue):
    """4x4 DXT1 block of one color. Assetto stores the cone color in the texture."""
    color0 = _rgb565(red, green, blue)
    color1 = _rgb565(max(0, red - 16), max(0, green - 16), max(0, blue - 16))
    if color0 <= color1:
        color1 = (color0 - 1) & 0xFFFF
    block = struct.pack('<HH', color0, color1) + b'\x00\x00\x00\x00'
    header = bytearray(128)
    header[0:4] = b'DDS '
    # CAPS, HEIGHT, WIDTH, PIXELFORMAT, MIPMAPCOUNT, LINEARSIZE.
    struct.pack_into('<7I', header, 4, 124, 0xA1007, 4, 4, 8, 0, 1)
    struct.pack_into('<I', header, 76, 32)
    struct.pack_into('<I', header, 80, 0x4)
    header[84:88] = b'DXT1'
    struct.pack_into('<I', header, 108, 0x1000)
    return bytes(header) + block


def _hash01(ix, iy, salt):
    n = (int(ix) * 374761393 + int(iy) * 668265263 + int(salt) * 1442695041) & 0xFFFFFFFF
    n = (n ^ (n >> 13)) * 1274126177 & 0xFFFFFFFF
    return (n & 0xFFFFFF) / float(0xFFFFFF)


def _noise_tile(x, y, cells):
    """Value noise that repeats every `cells` cells, so the tile has no seam."""
    x0 = int(math.floor(x)) % cells
    y0 = int(math.floor(y)) % cells
    x1 = (x0 + 1) % cells
    y1 = (y0 + 1) % cells
    fx = x - math.floor(x)
    fy = y - math.floor(y)
    sx = fx * fx * (3.0 - 2.0 * fx)
    sy = fy * fy * (3.0 - 2.0 * fy)
    n00 = _hash01(x0, y0, cells)
    n10 = _hash01(x1, y0, cells)
    n01 = _hash01(x0, y1, cells)
    n11 = _hash01(x1, y1, cells)
    return (
        n00 * (1.0 - sx) * (1.0 - sy)
        + n10 * sx * (1.0 - sy)
        + n01 * (1.0 - sx) * sy
        + n11 * sx * sy
    )


def _asphalt_rgb(size=256):
    rgb = bytearray(size * size * 3)
    for y in range(size):
        for x in range(size):
            u = x / size
            v = y / size
            # Soft tar patches, then stone grain. No contour lines: a threshold
            # on noise closes into loops and does not look like asphalt.
            patch = _noise_tile(u * 4, v * 4, 4)
            mid = _noise_tile(u * 13, v * 13, 13)
            grain = _noise_tile(u * 32, v * 32, 32)
            grit = _noise_tile(u * 64, v * 64, 64)
            value = (
                68.0
                + (patch - 0.5) * 26.0
                + (mid - 0.5) * 14.0
                + (grain - 0.5) * 10.0
                + (grit - 0.5) * 6.0
            )
            speck = _hash01(x, y, 3)
            if speck > 0.992:
                value += 16.0
            elif speck < 0.012:
                value -= 14.0
            value = max(0.0, min(255.0, value))
            index = (y * size + x) * 3
            rgb[index] = max(0, min(255, int(value + 5)))
            rgb[index + 1] = max(0, min(255, int(value + 3)))
            rgb[index + 2] = max(0, min(255, int(value)))
    return bytes(rgb)


def _rgb_at(rgb, width, x, y):
    index = (y * width + x) * 3
    return rgb[index], rgb[index + 1], rgb[index + 2]


def _dxt1_block(pixels):
    """16 RGB tuples. color0 > color1 so the block stays opaque."""
    def luma(pixel):
        return pixel[0] * 2 + pixel[1] * 4 + pixel[2]

    dark = min(pixels, key=luma)
    light = max(pixels, key=luma)
    color0 = _rgb565(*light)
    color1 = _rgb565(*dark)
    if color0 == color1:
        color0 = (color1 + 1) & 0xFFFF
    if color0 < color1:
        color0, color1 = color1, color0

    def unpack(value):
        return (
            ((value >> 11) & 31) * 255 // 31,
            ((value >> 5) & 63) * 255 // 63,
            (value & 31) * 255 // 31,
        )

    ends = (unpack(color0), unpack(color1))
    palette = (
        ends[0],
        ends[1],
        tuple((2 * ends[0][channel] + ends[1][channel]) // 3 for channel in range(3)),
        tuple((ends[0][channel] + 2 * ends[1][channel]) // 3 for channel in range(3)),
    )
    bits = 0
    for index, pixel in enumerate(pixels):
        best = 0
        best_distance = None
        for choice, color in enumerate(palette):
            distance = sum((pixel[channel] - color[channel]) ** 2 for channel in range(3))
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best = choice
        bits |= best << (2 * index)
    return struct.pack('<HHI', color0, color1, bits)


def _dxt1_level(rgb, width, height):
    if width < 4 or height < 4:
        pixels = []
        for y in range(4):
            for x in range(4):
                pixels.append(_rgb_at(rgb, width, min(x, width - 1), min(y, height - 1)))
        return _dxt1_block(pixels)
    blocks = bytearray()
    for y0 in range(0, height, 4):
        for x0 in range(0, width, 4):
            pixels = []
            for y in range(y0, y0 + 4):
                for x in range(x0, x0 + 4):
                    pixels.append(_rgb_at(rgb, width, x, y))
            blocks.extend(_dxt1_block(pixels))
    return bytes(blocks)


def _box_down(rgb, width, height):
    half_w = width // 2
    half_h = height // 2
    out = bytearray(half_w * half_h * 3)
    for y in range(half_h):
        for x in range(half_w):
            total = [0, 0, 0]
            for dy in (0, 1):
                for dx in (0, 1):
                    pixel = _rgb_at(rgb, width, x * 2 + dx, y * 2 + dy)
                    for channel in range(3):
                        total[channel] += pixel[channel]
            index = (y * half_w + x) * 3
            for channel in range(3):
                out[index + channel] = total[channel] // 4
    return bytes(out), half_w, half_h


def _dds_dxt1(rgb, width, height):
    levels = []
    image = rgb
    level_w = width
    level_h = height
    while True:
        levels.append(_dxt1_level(image, level_w, level_h))
        if level_w == 1 and level_h == 1:
            break
        if level_w == 1 or level_h == 1:
            break
        image, level_w, level_h = _box_down(image, level_w, level_h)
    header = bytearray(128)
    header[0:4] = b'DDS '
    # CAPS, HEIGHT, WIDTH, PIXELFORMAT, MIPMAPCOUNT, LINEARSIZE.
    flags = 0x1 | 0x2 | 0x4 | 0x1000 | 0x20000 | 0x80000
    struct.pack_into('<7I', header, 4, 124, flags, height, width, len(levels[0]), 0, len(levels))
    struct.pack_into('<I', header, 76, 32)
    struct.pack_into('<I', header, 80, 0x4)
    header[84:88] = b'DXT1'
    # TEXTURE, MIPMAP, COMPLEX.
    struct.pack_into('<I', header, 108, 0x1000 | 0x400000 | 0x8)
    return bytes(header) + b''.join(levels)


_ASPHALT_BLOB = None


def _asphalt_dds():
    global _ASPHALT_BLOB
    if _ASPHALT_BLOB is None:
        _ASPHALT_BLOB = _dds_dxt1(_asphalt_rgb(256), 256, 256)
    return _ASPHALT_BLOB


def _png(width, height, rgb):
    def chunk(tag, data):
        return (
            struct.pack('>I', len(data))
            + tag
            + data
            + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    rows = []
    stride = width * 3
    for y in range(height):
        rows.append(b'\x00' + rgb[y * stride:(y + 1) * stride])
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) + chunk(b'IDAT', zlib.compress(b''.join(rows), 9)) + chunk(b'IEND', b'')


def load_csv(path):
    cones = {tag: [] for tag in TAGS}
    start = (-13.0, 10.3, 0.0)
    with path.open(newline='') as handle:
        for row in csv.DictReader(handle):
            tag = row['tag'].strip()
            x = float(row['x'])
            y = float(row['y'])
            if tag == 'car_start':
                start = (x, y, float(row.get('direction') or 0.0))
                continue
            if tag in cones:
                cones[tag].append((x, y))
    return cones, start


def to_ac(x, y, start):
    """CSV metres to Assetto (x left, z forward). The road is y = 0."""
    sx, sy, yaw = start
    dx = x - sx
    dy = y - sy
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    forward = cos_yaw * dx + sin_yaw * dy
    left = -sin_yaw * dx + cos_yaw * dy
    return left, forward


def from_ac(ac_x, ac_z, start):
    sx, sy, yaw = start
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx = cos_yaw * ac_z - sin_yaw * ac_x
    dy = sin_yaw * ac_z + cos_yaw * ac_x
    return dx + sx, dy + sy


def _vertex(out, position, normal, tangent, uv):
    out.f32(position[0])
    out.f32(position[1])
    out.f32(position[2])
    out.f32(normal[0])
    out.f32(normal[1])
    out.f32(normal[2])
    out.f32(uv[0])
    out.f32(uv[1])
    out.f32(tangent[0])
    out.f32(tangent[1])
    out.f32(tangent[2])


def _mesh(out, name, material, vertices, indices, cast_shadows=1):
    if len(vertices) > 65535:
        raise RuntimeError('%s has %d vertices' % (name, len(vertices)))
    out.i32(2)
    out.string(name)
    out.i32(0)
    out.u8(1)
    out.u8(1 if cast_shadows else 0)
    out.u8(1)
    out.u8(0)
    out.i32(len(vertices))
    minimum = [1e9, 1e9, 1e9]
    maximum = [-1e9, -1e9, -1e9]
    for position, normal, tangent, uv in vertices:
        _vertex(out, position, normal, tangent, uv)
        for axis in range(3):
            minimum[axis] = min(minimum[axis], position[axis])
            maximum[axis] = max(maximum[axis], position[axis])
    out.i32(len(indices))
    for index in indices:
        out.u16(index)
    out.i32(material)
    out.u32(0)
    out.f32(0.0)
    out.f32(0.0)
    center = [(minimum[axis] + maximum[axis]) * 0.5 for axis in range(3)]
    radius = 0.0
    for position, _normal, _tangent, _uv in vertices:
        radius = max(radius, math.dist(position, center))
    out.f32(center[0])
    out.f32(center[1])
    out.f32(center[2])
    out.f32(max(radius, 0.01))
    out.u8(1)


def _dummy(out, name, x, y, z):
    """Type-1 parent. The mesh written next is its only child."""
    out.i32(1)
    out.string(name)
    out.i32(1)
    out.u8(1)
    for value in (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1):
        out.f32(value)


def _spawn(out, name, x, y, z):
    """Type-1 empty plus a locator mesh. The mesh has no children.

    fs_uk_sprint stores every spawn and timing point this way: the dummy
    holds the pose, and the child is a 3-vertex marker with the same name.
    Identity rotation faces +Z, which is the rotated CSV start direction.
    """
    out.i32(1)
    out.string(name)
    out.i32(1)
    out.u8(1)
    for value in (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1):
        out.f32(value)
    size = 0.05
    height = 0.02
    vertices = (
        ((0.0, height, size), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0)),
        ((-size, height, -size), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0)),
        ((size, height, -size), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0)),
    )
    _mesh(out, name, 0, vertices, [0, 1, 2, 0, 2, 1], cast_shadows=0)


def _ground(half_x, half_z, step):
    nx = max(1, math.ceil((2 * half_x) / step))
    nz = max(1, math.ceil((2 * half_z) / step))
    x0 = -nx * step / 2.0
    z0 = -nz * step / 2.0
    vertices = []
    for iz in range(nz + 1):
        for ix in range(nx + 1):
            x = x0 + ix * step
            z = z0 + iz * step
            vertices.append((
                (x, 0.0, z),
                (0.0, 1.0, 0.0),
                (1.0, 0.0, 0.0),
                (x / _ASPHALT_TILE_M, z / _ASPHALT_TILE_M),
            ))
    indices = []
    width = nx + 1

    def vid(ix, iz):
        return iz * width + ix

    for iz in range(nz):
        for ix in range(nx):
            v00 = vid(ix, iz)
            v10 = vid(ix + 1, iz)
            v01 = vid(ix, iz + 1)
            v11 = vid(ix + 1, iz + 1)
            # Right-handed winding. The cross product points up, +Y.
            indices.extend((v00, v01, v10, v01, v11, v10))
    return vertices, indices


def _cone(radius, height, sides=12):
    """Traffic cone in local space, centred on the origin.

    The base radius stays `radius` so the lidar cylinder is unchanged.
    The parent dummy carries the world position. Assetto treats a mesh
    named AC_POBJECT as a loose body, and that body pivots about the
    node origin, so the origin has to be the centre of the cone.
    """
    top_radius = radius * 0.18
    half = height * 0.5
    slope = math.hypot(height, radius - top_radius)
    normal_y = (radius - top_radius) / slope
    normal_h = height / slope
    vertices = []
    for index in range(sides):
        angle = 2.0 * math.pi * index / sides
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        normal = (normal_h * cos_a, normal_y, normal_h * sin_a)
        tangent = (sin_a, 0.0, -cos_a)
        around = index / sides
        vertices.append((
            (radius * cos_a, -half, radius * sin_a),
            normal, tangent, (around, 0.0),
        ))
        vertices.append((
            (top_radius * cos_a, half, top_radius * sin_a),
            normal, tangent, (around, 1.0),
        ))
    top = len(vertices)
    vertices.append((
        (0.0, half, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.5, 0.5),
    ))
    indices = []
    for index in range(sides):
        nxt = (index + 1) % sides
        bottom0 = index * 2
        top0 = bottom0 + 1
        bottom1 = nxt * 2
        top1 = bottom1 + 1
        indices.extend((bottom0, top0, bottom1, top0, top1, bottom1))
        # (center, next, current) faces up.
        indices.extend((top, top1, top0))
    return vertices, indices


def _shader_props(out, diffuse):
    props = (
        ('ksAmbient', diffuse),
        ('ksDiffuse', diffuse),
        ('ksSpecular', 0.15),
        ('ksSpecularEXP', 30.0),
        ('ksEmissive', 0.0),
        ('ksAlphaRef', 0.0),
    )
    out.i32(len(props))
    for name, value in props:
        out.string(name)
        out.f32(value)
        for _ in range(9):
            out.f32(0.0)


def _material(out, name, texture, diffuse):
    out.string(name)
    out.string('ksPerPixel')
    out.u8(0)
    out.u8(0)
    out.i32(0)
    _shader_props(out, diffuse)
    out.i32(1)
    out.string('txDiffuse')
    out.i32(0)
    out.string(texture)


def write_kn5(path, placed):
    """placed is (tag, ac_x, ac_z) in mesh order."""
    textures = (
        ('asphalt.dds', _asphalt_dds()),
        ('blue.dds', solid_dds(*COLOR['blue'])),
        ('yellow.dds', solid_dds(*COLOR['yellow'])),
        ('orange.dds', solid_dds(*COLOR['orange'])),
    )
    materials = (
        ('ROAD', 'asphalt.dds', 0.85),
        ('cone_blue', 'blue.dds', 0.9),
        ('cone_yellow', 'yellow.dds', 0.9),
        ('cone_orange', 'orange.dds', 0.9),
        ('cone_big_orange', 'orange.dds', 0.9),
    )
    material_id = {name: index for index, (name, _texture, _diffuse) in enumerate(materials)}
    out = Bin()
    out.raw(b'sc6969')
    out.i32(6)
    out.i32(0)
    out.i32(len(textures))
    for name, blob in textures:
        out.i32(1)
        out.string(name)
        out.i32(len(blob))
        out.raw(blob)
    out.i32(len(materials))
    for name, texture, diffuse in materials:
        _material(out, name, texture, diffuse)

    xs = [point[1] for point in placed] or [0.0]
    zs = [point[2] for point in placed] or [0.0]
    spline_end = max(20.0, max(zs) + 20.0)
    half_x = max(40.0, max(abs(value) for value in xs) + 30.0, 30.0)
    half_z = max(40.0, max(abs(value) for value in zs) + 30.0, spline_end + 10.0)
    ground_vertices, ground_indices = _ground(half_x, half_z, 20.0)

    # Without these nodes the game logs "TRACK :: NO POSITION DATA FOUND"
    # and Content Manager says the main layout is damaged. Practice uses
    # AC_PIT_0, a race uses AC_START_0, hotlap uses AC_HOTLAP_START_0.
    # +X is left, so the time pair crosses the start line.
    spawns = (
        ('AC_START_0', 0.0, 0.15, 0.0),
        ('AC_PIT_0', 0.0, 0.15, 0.0),
        ('AC_HOTLAP_START_0', 0.0, 0.15, 0.0),
        ('AC_TIME_0_L', 4.0, 0.5, 0.0),
        ('AC_TIME_0_R', -4.0, 0.5, 0.0),
    )
    out.i32(1)
    out.string('uvis_track')
    out.i32(1 + len(placed) + len(spawns))
    out.u8(1)
    for value in (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1):
        out.f32(value)
    for name, x, y, z in spawns:
        _spawn(out, name, x, y, z)
    # 1ROAD0 is the drivable plane. A cone must not use the WALL key:
    # that collider is immovable and the car stops dead against it.
    # AC_POBJECT is Assetto's loose-body name. The lidar still sees the
    # mesh because the name contains cone, and the parent matrix puts
    # the local cone back on its CSV point.
    _mesh(out, '1ROAD0', material_id['ROAD'], ground_vertices, ground_indices)
    for index, (tag, ac_x, ac_z) in enumerate(placed):
        height = HEIGHT[tag]
        _dummy(
            out,
            'xform_cone_%s_%d' % (tag, index),
            ac_x,
            height * 0.5 + _CONE_LIFT_M,
            ac_z,
        )
        vertices, indices = _cone(RADIUS, height)
        _mesh(
            out,
            'AC_POBJECT_cone_%s_%d' % (tag, index),
            material_id['cone_%s' % tag],
            vertices,
            indices,
        )
    path.write_bytes(out.buf)
    return spline_end


def write_ai(path, length_m):
    step = 2.0
    count = max(2, int(math.floor(length_m / step)) + 1)
    out = Bin()
    out.i32(7)
    out.i32(count)
    out.i32(0)
    out.i32(0)
    for index in range(count):
        out.f32(0.0)
        out.f32(0.0)
        out.f32(index * step)
        out.f32(index * step)
        out.i32(index)
    out.i32(count)
    for index in range(count):
        gap = step if index + 1 < count else 0.0
        for value in (
            12.0, 0.2, 0.0, 0.0, 500.0, 20.0, 20.0, 0.0, 1.0,
            0.0, 1.0, 0.0, gap, 0.0, 0.0, 1.0, 0.0, 0.0,
        ):
            out.f32(value)
    out.i32(0)
    path.write_bytes(out.buf)


def _text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(text.replace('\n', '\r\n').encode('utf-8'))


def _preview(placed, size=256):
    xs = [0.0] + [point[1] for point in placed]
    zs = [0.0] + [point[2] for point in placed]
    min_x, max_x = min(xs), max(xs)
    min_z, max_z = min(zs), max(zs)
    span_x = max(max_x - min_x, 1.0)
    span_z = max(max_z - min_z, 1.0)
    pad_x = span_x * 0.08
    pad_z = span_z * 0.08
    min_x -= pad_x
    max_x += pad_x
    min_z -= pad_z
    max_z += pad_z
    span_x = max_x - min_x
    span_z = max_z - min_z
    rgb = bytearray((64, 68, 72) * size * size)

    def plot(ac_x, ac_z, color, radius):
        px = int((ac_x - min_x) / span_x * (size - 1))
        py = int((1.0 - (ac_z - min_z) / span_z) * (size - 1))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                ix = px + dx
                iy = py + dy
                if 0 <= ix < size and 0 <= iy < size:
                    start = (iy * size + ix) * 3
                    rgb[start:start + 3] = bytes(color)

    for tag, ac_x, ac_z in placed:
        plot(ac_x, ac_z, COLOR[tag], 2)
    plot(0.0, 0.0, (255, 255, 255), 3)
    return _png(size, size, bytes(rgb))


def _surfaces():
    # KEY=ROAD matches mesh 1ROAD0. WAV_PITCH=extended-0 turns on CSP
    # extended physics so the AC_POBJECT cones can use a cylinder instead
    # of their triangle mesh. The cylinder is in the cone's local frame,
    # and that frame is centred on the cone.
    small = HEIGHT['blue'] * 0.5
    big = HEIGHT['big_orange'] * 0.5
    return """[SURFACE_0]
KEY=ROAD
FRICTION=1
DAMPING=0
WAV=
WAV_PITCH=extended-0
FF_EFFECT=NULL
DIRT_ADDITIVE=0
BLACK_FLAG_TIME=0
IS_VALID_TRACK=1
SIN_HEIGHT=0
SIN_LENGTH=0
IS_PITLANE=0
VIBRATION_GAIN=0
VIBRATION_LENGTH=0

[_EXTRA_PERMISSIONS]
ALLOW_CUSTOM_AI_MANIPULATION=1

[DYNAMIC_OBJECTS_CONE]
MESHES=AC_POBJECT_cone_blue?, AC_POBJECT_cone_yellow?, AC_POBJECT_cone_orange?
MASS=0.6
LINEAR_DAMPING=0.55
ANGULAR_DAMPING=0.7
INTERCOLLISION=0
DETECT_BOX_COLLIDER=0
NO_MESH_COLLIDER=1
BOUNCE=0.1
FRICTION=0.45
INTENSITY=0.05
COLLIDER_CYLINDER_0=%s
COLLIDER_CYLINDER_0_FROM=0, %.4f, 0
COLLIDER_CYLINDER_0_TO=0, %.4f, 0

[DYNAMIC_OBJECTS_BIG]
MESHES=AC_POBJECT_cone_big?
MASS=1.2
LINEAR_DAMPING=0.55
ANGULAR_DAMPING=0.7
INTERCOLLISION=0
DETECT_BOX_COLLIDER=0
NO_MESH_COLLIDER=1
BOUNCE=0.1
FRICTION=0.45
INTENSITY=0.05
COLLIDER_CYLINDER_0=%s
COLLIDER_CYLINDER_0_FROM=0, %.4f, 0
COLLIDER_CYLINDER_0_TO=0, %.4f, 0
""" % (RADIUS, -small, small, RADIUS, -big, big)


def _cameras():
    return """[HEADER]
VERSION=3
CAMERA_COUNT=1
SET_NAME=

[CAMERA_0]
NAME=Start
POSITION=0, 4, -12
FORWARD=0, -0.196116, 0.980581
UP=0, 0.980581, 0.196116
MIN_FOV=10
MAX_FOV=60
IN_POINT=0
OUT_POINT=1
SHADOW_SPLIT0=1.8
SHADOW_SPLIT1=20
SHADOW_SPLIT2=180
NEAR_PLANE=0.1
FAR_PLANE=5000
MIN_EXPOSURE=0
MAX_EXPOSURE=10000
DOF_FACTOR=10
DOF_RANGE=10000
DOF_FOCUS=0
DOF_MANUAL=0
SPLINE=
SPLINE_ROTATION=0
FOV_GAMMA=0
SPLINE_ANIMATION_LENGTH=15
IS_FIXED=1
"""


def _assert_consumed(path):
    """The lidar reader must reach the last byte. A short read is a bad KN5."""
    tool_root = Path(__file__).resolve().parents[1]
    if str(tool_root) not in sys.path:
        sys.path.insert(0, str(tool_root))
    from uvis_ac import kn5_cones as kn5

    names = []
    with path.open('rb') as handle:
        if handle.read(6) != b'sc6969':
            raise RuntimeError('%s is not a KN5' % path.name)
        version = struct.unpack('<i', handle.read(4))[0]
        if version > 5:
            handle.read(4)
        textures = struct.unpack('<i', handle.read(4))[0]
        for _texture in range(textures):
            handle.read(4)
            kn5._str(handle)
            size = struct.unpack('<i', handle.read(4))[0]
            handle.seek(size, 1)
        kn5._skip_materials(handle, version)

        def walk():
            node_type = struct.unpack('<i', handle.read(4))[0]
            names.append(kn5._str(handle))
            children = struct.unpack('<i', handle.read(4))[0]
            handle.read(1)
            if node_type == 1:
                handle.read(64)
            elif node_type == 2:
                handle.read(3)
                vcount = struct.unpack('<i', handle.read(4))[0]
                handle.seek(vcount * 44, 1)
                icount = struct.unpack('<i', handle.read(4))[0]
                handle.seek(icount * 2 + 4 + 29, 1)
            else:
                raise RuntimeError('%s has node type %s' % (path.name, node_type))
            for _child in range(children):
                walk()

        walk()
        if handle.tell() != path.stat().st_size:
            raise RuntimeError(
                '%s ended at %d of %d bytes' % (path.name, handle.tell(), path.stat().st_size))
    return names


def _check_roundtrip(kn5, placed, start):
    tool_root = Path(__file__).resolve().parents[1]
    if str(tool_root) not in sys.path:
        sys.path.insert(0, str(tool_root))
    from uvis_ac.kn5_cones import cone_cylinders

    names = set(_assert_consumed(kn5))
    required = (
        'AC_START_0', 'AC_PIT_0', 'AC_HOTLAP_START_0',
        'AC_TIME_0_L', 'AC_TIME_0_R',
    )
    missing = [name for name in required if name not in names]
    if missing:
        raise RuntimeError('%s missing %s' % (kn5.name, ', '.join(missing)))
    cones, walls = cone_cylinders(kn5)
    # A cone mesh must stay out of the wall list. The lidar still reports
    # the cylinder because the mesh name contains cone.
    if len(walls):
        raise RuntimeError('%s still has %d wall triangles' % (kn5.name, len(walls)))
    for index, (tag, _ac_x, _ac_z) in enumerate(placed):
        mesh_name = 'AC_POBJECT_cone_%s_%d' % (tag, index)
        if mesh_name not in names:
            raise RuntimeError('%s missing %s' % (kn5.name, mesh_name))
    if len(cones) != len(placed):
        raise RuntimeError('%s parsed %d cones, wrote %d' % (kn5.name, len(cones), len(placed)))
    unused = list(cones)
    worst = 0.0
    for tag, ac_x, ac_z in placed:
        nearest = min(unused, key=lambda cone: (cone['xy'][0] - ac_x) ** 2 + (cone['xy'][1] - ac_z) ** 2)
        error = math.hypot(nearest['xy'][0] - ac_x, nearest['xy'][1] - ac_z)
        worst = max(worst, error)
        if error > 0.02:
            raise RuntimeError('%s cone %s off by %.3f m' % (kn5.name, tag, error))
        if abs(nearest['height'] - HEIGHT[tag]) > 0.02:
            raise RuntimeError('%s %s height %.3f' % (kn5.name, tag, nearest['height']))
        if abs(nearest['radius'] - RADIUS) > 0.02:
            raise RuntimeError('%s %s radius %.3f' % (kn5.name, tag, nearest['radius']))
        if abs(nearest['z0'] - _CONE_LIFT_M) > 0.02:
            raise RuntimeError('%s %s base %.3f' % (kn5.name, tag, nearest['z0']))
        back_x, back_y = from_ac(nearest['xy'][0], nearest['xy'][1], start)
        csv_x, csv_y = from_ac(ac_x, ac_z, start)
        if math.hypot(back_x - csv_x, back_y - csv_y) > 0.02:
            raise RuntimeError('%s CSV error %.3f m' % (kn5.name, math.hypot(back_x - csv_x, back_y - csv_y)))
        unused.remove(nearest)
    return worst


def _self_check():
    start = (0.0, 0.0, 0.0)
    ac_x, ac_z = to_ac(5.0, 2.0, start)
    if abs(ac_x - 2.0) > 1e-9 or abs(ac_z - 5.0) > 1e-9:
        raise RuntimeError('yaw 0 map failed')
    turned = (0.0, 0.0, math.pi / 2.0)
    ac_x, ac_z = to_ac(0.0, 5.0, turned)
    if abs(ac_x) > 1e-9 or abs(ac_z - 5.0) > 1e-9:
        raise RuntimeError('yaw 90 map failed')
    sample = (3.5, -2.0)
    origin = (1.0, 4.0, -1.2)
    back = from_ac(*to_ac(*sample, origin), origin)
    if math.hypot(back[0] - sample[0], back[1] - sample[1]) > 1e-9:
        raise RuntimeError('map does not invert')
    vertices, indices = _cone(1.0, 2.0)
    base = vertices[0][0]
    if abs(base[0] - 1.0) > 1e-6 or abs(base[1] + 1.0) > 1e-6 or abs(base[2]) > 1e-6:
        raise RuntimeError('cone is not centred on its pivot')
    position = vertices[0][0]
    top = vertices[1][0]
    nxt = vertices[2][0]
    edge_up = (top[0] - position[0], top[1] - position[1], top[2] - position[2])
    edge_side = (nxt[0] - position[0], nxt[1] - position[1], nxt[2] - position[2])
    cross_x = edge_up[1] * edge_side[2] - edge_up[2] * edge_side[1]
    if cross_x <= 0.0:
        raise RuntimeError('cone winding faces inward')
    if math.hypot(position[0], position[2]) <= math.hypot(top[0], top[2]) + 1e-6:
        raise RuntimeError('cone does not taper')
    cap = [vertices[index][0] for index in indices[6:9]]
    cap_a = (cap[1][0] - cap[0][0], cap[1][1] - cap[0][1], cap[1][2] - cap[0][2])
    cap_b = (cap[2][0] - cap[0][0], cap[2][1] - cap[0][1], cap[2][2] - cap[0][2])
    cap_y = cap_a[2] * cap_b[0] - cap_a[0] * cap_b[2]
    if cap_y <= 0.0:
        raise RuntimeError('cone top faces down')
    ground, _ground_indices = _ground(8.0, 8.0, 4.0)
    tiled = [
        vertex for vertex in ground
        if abs(vertex[0][0] - 8.0) < 1e-6 and abs(vertex[0][2]) < 1e-6
    ]
    if not tiled or abs(tiled[0][3][0] - 2.0) > 1e-6:
        raise RuntimeError('asphalt uv does not repeat every 4 m')
    image = _asphalt_rgb(64)
    if max(image) - min(image) < 20:
        raise RuntimeError('asphalt texture is flat')
    blob = _asphalt_dds()
    if blob[:4] != b'DDS ' or len(blob) < 128 + 8:
        raise RuntimeError('asphalt texture is not a DDS')


def write_track(csv_path, tracks_root):
    name = 'uvis_%s' % csv_path.stem
    folder = tracks_root / name
    marker = folder / 'uvis_eufs_track.txt'
    if folder.exists() and not marker.is_file():
        raise RuntimeError('%s exists and was not generated by this script' % folder)
    cones, start = load_csv(csv_path)
    placed = []
    for tag in TAGS:
        for eufs_x, eufs_y in cones[tag]:
            ac_x, ac_z = to_ac(eufs_x, eufs_y, start)
            placed.append((tag, ac_x, ac_z, eufs_x, eufs_y))
    if folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
    _text(folder / 'uvis_eufs_track.txt', """Generated by eufs_to_ac.py from %s.
Safe to delete this whole folder.
Colors follow the CSV: blue left, yellow right, orange, big orange.
The ground is a repeating asphalt texture. Cones are loose AC_POBJECT bodies, not walls.
The spawn is the CSV car_start and the nose points along that direction.
Path finding uses left_color 1, right_color 2, start_colors 3,4.
Spawn nodes are AC_START_0, AC_PIT_0, and AC_HOTLAP_START_0 at the origin, facing +Z.
AC_TIME_0_L and AC_TIME_0_R sit across the start line.
""" % csv_path.name)
    kn5_name = '%s.kn5' % name
    spline_end = write_kn5(folder / kn5_name, [(tag, ac_x, ac_z) for tag, ac_x, ac_z, _x, _y in placed])
    _text(folder / 'models.ini', '[MODEL_0]\nFILE=%s\nPOSITION=0,0,0\nROTATION=0,0,0\n' % kn5_name)
    _text(folder / 'data' / 'surfaces.ini', _surfaces())
    _text(folder / 'data' / 'lighting.ini', '[LIGHTING]\nSUN_PITCH_ANGLE=50\nSUN_HEADING_ANGLE=20\n')
    _text(folder / 'data' / 'cameras.ini', _cameras())
    (folder / 'ai').mkdir(parents=True, exist_ok=True)
    write_ai(folder / 'ai' / 'fast_lane.ai', spline_end)
    counts = {tag: len(cones[tag]) for tag in TAGS}
    title = 'UVIS EUFS %s' % csv_path.stem.replace('_', ' ')
    description = (
        'Empty copy of the EUFS %s track on asphalt. Blue cones are left, yellow cones '
        'are right, orange and big orange keep their CSV places. The car starts at the '
        'CSV start facing the CSV direction.'
    ) % csv_path.stem
    _text(folder / 'ui' / 'ui_track.json', """{
\t"name": "%s",
\t"description": "%s",
\t"tags": ["circuit", "formula student"],
\t"geotags": ["0", "0"],
\t"country": "Germany",
\t"city": "UVIS",
\t"length": "%.0fm",
\t"width": "3m",
\t"pitboxes": "1",
\t"run": "clockwise"
}
""" % (title, description, max(20.0, spline_end)))
    picture = _preview([(tag, ac_x, ac_z) for tag, ac_x, ac_z, _x, _y in placed])
    (folder / 'ui').mkdir(parents=True, exist_ok=True)
    (folder / 'ui' / 'preview.png').write_bytes(picture)
    (folder / 'ui' / 'outline.png').write_bytes(picture)
    rows = ['tag,ac_x,ac_y,ac_z,eufs_x,eufs_y']
    for tag, ac_x, ac_z, eufs_x, eufs_y in placed:
        rows.append('%s,%.5f,0,%.5f,%.5f,%.5f' % (tag, ac_x, ac_z, eufs_x, eufs_y))
    _text(folder / 'cones.csv', '\n'.join(rows) + '\n')
    worst = _check_roundtrip(
        folder / kn5_name,
        [(tag, ac_x, ac_z) for tag, ac_x, ac_z, _x, _y in placed],
        start,
    )
    return name, counts, worst, folder


def _csv_dir():
    return Path(__file__).resolve().parents[5] / 'src' / 'eufs_sim' / 'eufs_tracks' / 'csv'


def _tracks_root():
    tool_root = Path(__file__).resolve().parents[1]
    if str(tool_root) not in sys.path:
        sys.path.insert(0, str(tool_root))
    from uvis_ac.paths import steam_home
    return steam_home() / '.local/share/Steam/steamapps/common/assettocorsa/content/tracks'


def main():
    parser = argparse.ArgumentParser(description='Copy EUFS cone CSVs into empty Assetto Corsa tracks.')
    parser.add_argument('--csv-dir', type=Path, default=None)
    parser.add_argument('--out', type=Path, default=None, help='content/tracks directory')
    parser.add_argument('tracks', nargs='*', help='CSV stems. Default is every CSV in the directory.')
    args = parser.parse_args()
    _self_check()
    csv_dir = args.csv_dir or _csv_dir()
    tracks_root = args.out or _tracks_root()
    if not csv_dir.is_dir():
        raise SystemExit('No EUFS csv directory at %s' % csv_dir)
    if not tracks_root.is_dir():
        raise SystemExit('No Assetto tracks directory at %s' % tracks_root)
    if args.tracks:
        sources = []
        for name in args.tracks:
            path = csv_dir / ('%s.csv' % name)
            if not path.is_file():
                raise SystemExit('No CSV %s' % path)
            sources.append(path)
    else:
        sources = sorted(csv_dir.glob('*.csv'))
    if not sources:
        raise SystemExit('No CSV files in %s' % csv_dir)
    for path in sources:
        track_name, counts, worst, folder = write_track(path, tracks_root)
        print(
            '%s  blue=%d yellow=%d orange=%d big_orange=%d  max_error=%.4fm  %s'
            % (
                track_name,
                counts['blue'],
                counts['yellow'],
                counts['orange'],
                counts['big_orange'],
                worst,
                folder,
            )
        )


if __name__ == '__main__':
    main()
