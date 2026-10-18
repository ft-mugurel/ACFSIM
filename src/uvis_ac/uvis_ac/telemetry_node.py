"""Publish Assetto Corsa pose, IMU, and GPS from the bridge files."""

import math
import random
import struct
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import AccelStamped, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, Float64MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from uvis_ac.convert import (
    body_rates, body_velocity, heading_to_yaw, pose_from_start,
    specific_force_from_motion, world_to_map,
)
from uvis_ac.imu_noise import ACCEL_WHITE, GYRO_WHITE, ImuNoise
from uvis_ac.vehicle import road_wheel_angle
from uvis_ac.pages import graphics_from_bytes, physics_from_bytes
from uvis_ac.paths import bridge_dir, track_key
from uvis_ac.sensors import load_sensors

PREFIX = bridge_dir()
ORIGIN_LAT = 49.327
ORIGIN_LON = 8.565


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('ac_telemetry')
        self.declare_parameter('sensors_file', '')
        sensor_path = self.get_parameter('sensors_file').value
        sensors = load_sensors(sensor_path or None)
        self._imu_pose = sensors.imu
        self._gps_pose = sensors.gps
        self._physics_path = PREFIX / 'physics.bin'
        self._graphics_path = PREFIX / 'graphics.bin'
        self._imu_pub = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        shown = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._accel_pub = self.create_publisher(AccelStamped, '/imu/accel', shown)
        self._gps_pub = self.create_publisher(NavSatFix, '/gps', qos_profile_sensor_data)
        self._gps_pose_pub = self.create_publisher(PoseStamped, '/gps/pose', shown)
        self._wheel_pub = self.create_publisher(Float64MultiArray, '/vehicle/wheel_speed', shown)
        self._steer_pub = self.create_publisher(Float64, '/vehicle/steering', shown)
        self._tf = TransformBroadcaster(self)
        static = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        static.sendTransform([
            self._static_tf(now, self._imu_pose),
            self._static_tf(now, self._gps_pose),
        ])
        self._static = static
        self._origin = self._read_origin()
        self._prev_velocity = None
        self._prev_velocity_time = None
        self._prev_heading = None
        self._specific_force = (0.0, 0.0, 9.81)
        self._body_rates = (0.0, 0.0, 0.0)
        self.declare_parameter('imu_noise', True)
        self._imu_noise = ImuNoise(enabled=bool(self.get_parameter('imu_noise').value))
        self.create_timer(0.02, self._tick)
        self.get_logger().info('Reading %s' % PREFIX)
        if self._imu_noise.enabled:
            walk = math.degrees(self._imu_noise.gyro_bias[2]) * 60.0
            self.get_logger().info(
                'IMU noise on. Gyro white %.4f rad/s, accel white %.2f m/s^2. '
                'Turn-on yaw bias %+.5f rad/s (%+.2f deg/min).'
                % (GYRO_WHITE, ACCEL_WHITE, self._imu_noise.gyro_bias[2], walk))
        else:
            self.get_logger().info('IMU noise off.')

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
        yaw = heading_to_yaw(physics.heading)
        if self._origin is None:
            if not math.isfinite(x) or not math.isfinite(y):
                return
            self._origin = (x, y, z)
            self._write_origin(self._origin)
            self.get_logger().info(
                'Map origin for %s latched at (%.1f, %.1f, %.1f)'
                % ((track_key() or 'this track',) + self._origin))
        mx, my, mz, myaw = pose_from_start(x, y, z, yaw, self._origin)
        self._publish_tf(stamp, mx, my, mz, myaw)
        self._publish_imu(stamp, physics)
        self._publish_gps(stamp, x, y, z, mx, my, mz)
        self._publish_vehicle(stamp, physics)

    def _read_origin(self):
        path = PREFIX / 'map_origin2.bin'
        if not path.exists() or path.stat().st_size < 12:
            return None
        data = path.read_bytes()
        ox, oy, oz = struct.unpack_from('<3f', data, 0)
        if not math.isfinite(ox) or not math.isfinite(oy):
            return None
        saved = ''
        if len(data) >= 14:
            length = struct.unpack_from('<H', data, 12)[0]
            if 0 < length <= 180 and 14 + length <= len(data):
                saved = data[14:14 + length].decode('utf-8', 'replace')
        current = track_key()
        if saved != current:
            self.get_logger().info(
                'Saved map origin is for %s, this session is %s. Zeroing on the first pose.'
                % (saved or 'an older track', current or 'this track'))
            return None
        self.get_logger().info(
            'Reusing the map origin for %s at (%.1f, %.1f, %.1f)'
            % (current or 'this track', ox, oy, oz))
        return (ox, oy, oz)

    def _write_origin(self, origin):
        path = PREFIX / 'map_origin2.bin'
        key = track_key().encode('utf-8')[:180]
        blob = struct.pack('<3fH', origin[0], origin[1], origin[2], len(key)) + key
        temporary = path.with_suffix('.bin.tmp')
        temporary.write_bytes(blob)
        temporary.replace(path)

    def _static_tf(self, stamp, pose):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = pose.frame
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.translation.z = pose.z
        qx, qy, qz, qw = pose.quaternion()
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

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
        velocity = body_velocity(physics.local_velocity)
        now = time.monotonic()
        dt = 0.0 if self._prev_velocity_time is None else now - self._prev_velocity_time
        wx, wy, wz = body_rates(physics.heading, self._prev_heading, dt)
        if self._prev_velocity is not None and dt > 1e-3 and velocity != self._prev_velocity:
            self._specific_force = specific_force_from_motion(
                velocity, self._prev_velocity, (wx, wy, wz), dt)
        self._prev_velocity = velocity
        self._prev_heading = physics.heading
        self._prev_velocity_time = now
        self._body_rates = (wx, wy, wz)
        ax, ay, az = self._specific_force
        # Published measurement only. The specific-force state above stays
        # on the clean rate. The map pose is not noised.
        (wx, wy, wz), (ax, ay, az) = self._imu_noise.apply((wx, wy, wz), (ax, ay, az))
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_pose.frame
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz
        msg.orientation_covariance[0] = -1.0
        self._imu_pub.publish(msg)
        accel = AccelStamped()
        accel.header = msg.header
        accel.accel.linear = msg.linear_acceleration
        accel.accel.angular = msg.angular_velocity
        self._accel_pub.publish(accel)

    def _publish_vehicle(self, stamp, physics):
        wheels = Float64MultiArray()
        wheels.data = [float(v) for v in physics.wheel_angular_speed]
        self._wheel_pub.publish(wheels)
        steer = Float64()
        steer.data = road_wheel_angle(physics.steer_angle)
        self._steer_pub.publish(steer)

    def _publish_gps(self, stamp, x, y, z, mx, my, mz):
        north = x + random.gauss(0.0, 2.0)
        east = -y + random.gauss(0.0, 2.0)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = mx + random.gauss(0.0, 2.0)
        pose.pose.position.y = my + random.gauss(0.0, 2.0)
        pose.pose.position.z = mz
        pose.pose.orientation.w = 1.0
        self._gps_pose_pub.publish(pose)
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self._gps_pose.frame
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
