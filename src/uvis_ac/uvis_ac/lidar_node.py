"""Synthetic VLP-16 aimed from the nose of the Assetto Corsa car."""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from uvis_ac.kn5_cones import cone_cylinders
from uvis_ac.pages import graphics_from_bytes, physics_from_bytes
from uvis_ac.paths import bridge_dir, track_kn5
from uvis_ac.vlp16 import grid_walls, ranges_to_cones, rays

TRACK = track_kn5()
BRIDGE = bridge_dir()
LIDAR_FORWARD = 1.70


class LidarNode(Node):
    def __init__(self):
        super().__init__('ac_lidar')
        self.get_logger().info('Reading cones from %s' % TRACK.name)
        cones, walls = cone_cylinders(TRACK)
        self._cones = cones
        self._walls = grid_walls(walls)
        rays_list = list(rays())
        self._dirs = np.array([item[0] for item in rays_list], dtype=np.float64)
        self._rings = np.array([item[1] for item in rays_list], dtype=np.uint16)
        self._times = np.array([item[2] for item in rays_list], dtype=np.float32)
        self._pub = self.create_publisher(
            PointCloud2, '/velodyne_points',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            ))
        broadcaster = StaticTransformBroadcaster(self)
        mount = TransformStamped()
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.header.frame_id = 'base_footprint'
        mount.child_frame_id = 'velodyne'
        mount.transform.translation.x = LIDAR_FORWARD
        mount.transform.rotation.w = 1.0
        broadcaster.sendTransform(mount)
        self._mount = broadcaster
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'VLP-16 over %d cones and %d wall triangles' % (len(cones), len(walls)))

    def _tick(self):
        physics_path = BRIDGE / 'physics.bin'
        graphics_path = BRIDGE / 'graphics.bin'
        if not physics_path.exists() or not graphics_path.exists():
            return
        physics = physics_from_bytes(physics_path.read_bytes())
        graphics = graphics_from_bytes(graphics_path.read_bytes())
        if physics is None or graphics is None:
            return
        cx, cy, cz = graphics.car_coordinates
        # Heading 0 faces the nose. The game's positive heading is a right
        # turn, so the side rays are mirrored. Forward is not.
        heading = physics.heading
        s, c = math.sin(heading), math.cos(heading)
        origin = (cx - s * LIDAR_FORWARD, cz + c * LIDAR_FORWARD, cy)
        ground = cy - max(float(physics.cg_height), 0.2)
        rx, ry, rz = self._dirs[:, 0], self._dirs[:, 1], self._dirs[:, 2]
        flat = np.column_stack((
            -s * rx + c * ry,
            c * rx + s * ry,
            rz,
        ))
        distance = ranges_to_cones(
            origin, flat, self._cones, ground, self._walls)
        keep = np.isfinite(distance)
        if not np.any(keep):
            return
        dist = distance[keep]
        dirs = self._dirs[keep]
        # Pad to 24 bytes. A 22-byte point makes RViz read off the end of the cloud.
        cloud = np.empty(int(keep.sum()), dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32), ('time', np.float32),
            ('ring', np.uint16), ('_', np.uint16),
        ])
        cloud['x'] = (dirs[:, 0] * dist).astype(np.float32)
        cloud['y'] = (dirs[:, 1] * dist).astype(np.float32)
        cloud['z'] = (dirs[:, 2] * dist).astype(np.float32)
        cloud['intensity'] = 1.0
        cloud['time'] = self._times[keep]
        cloud['ring'] = self._rings[keep]
        header = Header()
        # The map pose is published at 50 Hz. A stamp slightly behind now is
        # already in that buffer, so RViz can transform the cloud.
        header.stamp = (self.get_clock().now() - Duration(seconds=0.05)).to_msg()
        header.frame_id = 'velodyne'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
            PointField(name='_', offset=22, datatype=PointField.UINT16, count=1),
        ]
        msg = point_cloud2.create_cloud(header, fields, cloud)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
