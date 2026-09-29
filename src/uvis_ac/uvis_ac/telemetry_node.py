"""Publish Assetto Corsa pose, IMU, and GPS from the bridge files."""

import math
import random
import struct
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from uvis_ac.convert import angular_velocity, pose_from_start, specific_force, world_to_map
from uvis_ac.pages import graphics_from_bytes, physics_from_bytes
from uvis_ac.paths import bridge_dir

PREFIX = bridge_dir()
ORIGIN_LAT = 49.327
ORIGIN_LON = 8.565


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('ac_telemetry')
        self._physics_path = PREFIX / 'physics.bin'
        self._graphics_path = PREFIX / 'graphics.bin'
        self._imu_pub = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self._gps_pub = self.create_publisher(NavSatFix, '/gps', qos_profile_sensor_data)
        self._tf = TransformBroadcaster(self)
        static = StaticTransformBroadcaster(self)
        imu_tf = TransformStamped()
        imu_tf.header.frame_id = 'base_footprint'
        imu_tf.child_frame_id = 'imu_base_link'
        imu_tf.transform.rotation.w = 1.0
        static.sendTransform(imu_tf)
        self._static = static
        self._origin = self._read_origin()
        self.create_timer(0.02, self._tick)
        self.get_logger().info('Reading %s' % PREFIX)

    def _tick(self):
        if not self._physics_path.exists() or not self._graphics_path.exists():
            return
        physics = physics_from_bytes(self._physics_path.read_bytes())
        graphics = graphics_from_bytes(self._graphics_path.read_bytes())
        if physics is None or graphics is None:
            return
        if graphics.status != 2:
            return
        stamp = self.get_clock().now().to_msg()
        x, y, z = world_to_map(graphics.car_coordinates)
        yaw = physics.heading
        if self._origin is None:
            if not math.isfinite(x) or not math.isfinite(y):
                return
            self._origin = (x, y, z)
            self._write_origin(self._origin)
            self.get_logger().info(
                'Map origin latched once at (%.1f, %.1f, %.1f)'
                % self._origin)
        mx, my, mz, myaw = pose_from_start(x, y, z, yaw, self._origin)
        self._publish_tf(stamp, mx, my, mz, myaw)
        self._publish_imu(stamp, physics)
        self._publish_gps(stamp, x, y, z)

    def _read_origin(self):
        path = PREFIX / 'map_origin.bin'
        if not path.exists() or path.stat().st_size < 12:
            return None
        ox, oy, oz = struct.unpack('<3f', path.read_bytes()[:12])
        if not math.isfinite(ox) or not math.isfinite(oy):
            return None
        self.get_logger().info(
            'Reusing the latched map origin (%.1f, %.1f, %.1f)' % (ox, oy, oz))
        return (ox, oy, oz)

    def _write_origin(self, origin):
        path = PREFIX / 'map_origin.bin'
        temporary = path.with_suffix('.bin.tmp')
        temporary.write_bytes(struct.pack('<3f', *origin))
        temporary.replace(path)

    def _publish_tf(self, stamp, x, y, z, yaw):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf.sendTransform(transform)

    def _publish_imu(self, stamp, physics):
        ax, ay, az = specific_force(physics.acc_g)
        wx, wy, wz = angular_velocity(physics.local_angular_vel)
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_base_link'
        msg.linear_acceleration.x = ax + random.gauss(0.0, 0.02)
        msg.linear_acceleration.y = ay + random.gauss(0.0, 0.02)
        msg.linear_acceleration.z = az + random.gauss(0.0, 0.02)
        msg.angular_velocity.x = wx + random.gauss(0.0, 0.001)
        msg.angular_velocity.y = wy + random.gauss(0.0, 0.001)
        msg.angular_velocity.z = wz + random.gauss(0.0, 0.001)
        msg.orientation_covariance[0] = -1.0
        self._imu_pub.publish(msg)

    def _publish_gps(self, stamp, x, y, z):
        north = x + random.gauss(0.0, 2.0)
        east = -y + random.gauss(0.0, 2.0)
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps_base_link'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = ORIGIN_LAT + north / 111320.0
        msg.longitude = ORIGIN_LON + east / (111320.0 * math.cos(math.radians(ORIGIN_LAT)))
        msg.altitude = 100.0 + z
        self._gps_pub.publish(msg)


def main():
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
