"""Load the car sensor poses from config/sensors.yaml."""

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class Pose:
    name: str
    frame: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def translation(self):
        return (self.x, self.y, self.z)

    def quaternion(self):
        return rpy_to_quaternion(self.roll, self.pitch, self.yaw)

    def matrix(self):
        return rpy_matrix(self.roll, self.pitch, self.yaw)


@dataclass(frozen=True)
class Lidar(Pose):
    lasers: int
    fov_upper: float
    fov_lower: float
    azimuth_start: float
    azimuth_end: float
    azimuth_step: float


@dataclass(frozen=True)
class Sensors:
    imu: Pose
    gps: Pose
    lidar: Lidar


def default_path() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('uvis_ac')) / 'config' / 'sensors.yaml'


def load_sensors(path=None) -> Sensors:
    file_path = Path(path) if path else default_path()
    data = yaml.safe_load(file_path.read_text())
    entries = data.get('Sensors') or {}
    imu = gps = None
    lidar = None
    for name, entry in entries.items():
        if not isinstance(entry, dict) or not entry.get('Enabled', True):
            continue
        if name == 'Imu':
            imu = _pose(name, entry, 'imu_base_link')
        elif name == 'Gps':
            gps = _pose(name, entry, 'gps_base_link')
        elif 'NumberOfLasers' in entry or 'HorizontalFOVStart' in entry:
            if lidar is None:
                lidar = _lidar(name, entry)
    if imu is None or gps is None or lidar is None:
        raise ValueError('sensors.yaml needs an enabled Imu, Gps, and lidar')
    return Sensors(imu=imu, gps=gps, lidar=lidar)


def _pose(name, entry, frame):
    return Pose(
        name=name,
        frame=str(entry.get('Frame', frame)),
        x=float(entry.get('X', 0.0)),
        y=float(entry.get('Y', 0.0)),
        z=float(entry.get('Z', 0.0)),
        roll=math.radians(float(entry.get('Roll', 0.0))),
        pitch=math.radians(float(entry.get('Pitch', 0.0))),
        yaw=math.radians(float(entry.get('Yaw', 0.0))),
    )


def _lidar(name, entry):
    pose = _pose(name, entry, 'velodyne')
    return Lidar(
        name=pose.name,
        frame=pose.frame,
        x=pose.x, y=pose.y, z=pose.z,
        roll=pose.roll, pitch=pose.pitch, yaw=pose.yaw,
        lasers=int(entry.get('NumberOfLasers', 16)),
        fov_upper=float(entry.get('VerticalFOVUpper', 15.0)),
        fov_lower=float(entry.get('VerticalFOVLower', -15.0)),
        azimuth_start=float(entry.get('HorizontalFOVStart', -100.0)),
        azimuth_end=float(entry.get('HorizontalFOVEnd', 100.0)),
        azimuth_step=float(entry.get('AzimuthStep', 0.2)),
    )


def rpy_matrix(roll, pitch, yaw):
    """Rotate a lidar-frame vector into base_footprint. R = Rz Ry Rx."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def mount_in_world(offset, heading):
    """base_footprint offset to an AC world delta (x, up, z) at this heading."""
    sx, sy, sz = offset
    s, c = math.sin(heading), math.cos(heading)
    return (-s * sx + c * sy, sz, c * sx + s * sy)


def directions_in_world(directions, mount, heading):
    """Lidar-frame rays to AC world rays (x, z-forward, up)."""
    body = directions @ mount.matrix().T
    s, c = math.sin(heading), math.cos(heading)
    return np.column_stack((
        -s * body[:, 0] + c * body[:, 1],
        c * body[:, 0] + s * body[:, 1],
        body[:, 2],
    ))
